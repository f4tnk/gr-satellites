#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2019 Daniel Estevez <daniel@destevez.net>
#
# This file is part of gr-satellites
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

from gnuradio import gr, digital

from ... import nrzi_decode, hdlc_deframer, pdu_length_filter
import pmt


class ax25_header_check(gr.basic_block):
    """
    Checks that the header of an AX.25 frame looks valid to reduce false positives.

     Validation layers:
        1. AX.25 address parsing (2..10 addresses): each 6-byte callsign must use
            valid shifted AX.25 callsign characters.
        2. SSID byte checks per address: reserved bits 5-6 must be set (0b11),
            and extension bit (LSB) must mark end of the address field.
        3. The frame must contain control+PID bytes after the address field.

     This is stricter than fixed-position checks and significantly lowers false
     positives while still accepting frames with digipeater addresses.
    """

    # F4TNK Session 16: 256-byte lookup table for valid shifted callsign bytes.
    # AX.25 address bytes = (ASCII char << 1). Valid chars: A-Z, 0-9, space.
    # Pre-built at class load time — zero per-call cost.
    _VALID_CS = bytearray(256)
    for _c in ' 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ':
        _VALID_CS[ord(_c) << 1] = 1
    del _c

    def __init__(self):
        gr.basic_block.__init__(self, name="ax25_header_check", in_sig=[], out_sig=[])
        self.message_port_register_in(pmt.intern('in'))
        self.message_port_register_out(pmt.intern('out'))
        self.set_msg_handler(pmt.intern('in'), self.handle_msg)

    def _valid_callsign_byte(self, value):
        return self._VALID_CS[value] != 0

    def _parse_address_field(self, packet):
        """
        Parse AX.25 address field and return offset of control byte.
        Returns None if header is invalid.
        """
        if len(packet) < 16:
            return None

        offset = 0
        addr_count = 0
        max_addresses = 10  # destination + source + up to 8 repeaters

        while addr_count < max_addresses:
            if offset + 7 > len(packet):
                return None

            # 6-byte shifted callsign.
            has_non_space = False
            for i in range(6):
                value = packet[offset + i]
                if not self._valid_callsign_byte(value):
                    return None
                if value != 0x40:
                    has_non_space = True

            # Disallow empty callsigns (all spaces).
            if not has_non_space:
                return None

            ssid = packet[offset + 6]

            # AX.25 SSID byte: bits 5-6 are reserved and should be 1.
            if (ssid & 0x60) != 0x60:
                return None

            addr_count += 1
            offset += 7

            # End of address field marker.
            if ssid & 0x01:
                break
        else:
            return None

        if addr_count < 2:
            return None

        # Require at least control + PID after addresses.
        if offset + 2 > len(packet):
            return None

        return offset

    def handle_msg(self, msg_pmt):
        msg = pmt.cdr(msg_pmt)
        packet = bytes(pmt.u8vector_elements(msg))

        if self._parse_address_field(packet) is None:
            return

        self.message_port_pub(pmt.intern('out'), msg_pmt)


class ax25_deframer(gr.hier_block2):
    """
    Hierarchical block to deframe AX.25.

    The input is a float stream of soft symbols. The output are PDUs
    with AX.25 frames.

    The input should be NRZ-I encoded and optionally G3RUH scrambled.

    Args:
        g3ruh_scrambler: use G3RUH descrambling (boolean)
        options: Options from argparse
    """
    def __init__(self, g3ruh_scrambler, options=None):
        gr.hier_block2.__init__(
            self,
            'ax25_deframer',
            gr.io_signature(1, 1, gr.sizeof_float),
            gr.io_signature(0, 0, 0))
        self.message_port_register_hier_out('out')

        self.slicer = digital.binary_slicer_fb()
        self.nrzi = nrzi_decode()
        if g3ruh_scrambler:
            self.descrambler = digital.descrambler_bb(0x21, 0, 16)
        self.deframer = hdlc_deframer(True, 10000)
        self.length_filter = pdu_length_filter(16, 10000)
        self.header_check = ax25_header_check()

        self._blocks = [self, self.slicer, self.nrzi]
        if g3ruh_scrambler:
            self._blocks.append(self.descrambler)
        self._blocks += [self.deframer]

        self.connect(*self._blocks)
        self.msg_connect((self.deframer, 'out'), (self.length_filter, 'in'))
        self.msg_connect((self.length_filter, 'out'), (self.header_check, 'in'))
        self.msg_connect((self.header_check, 'out'), (self, 'out'))
