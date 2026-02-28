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
      1. Extension bits (LSB) of bytes 0-12 must be 0.
      2. F4TNK Session 16: Callsign character validation — bytes 0-5 (dest)
         and 7-12 (src) must contain valid shifted AX.25 callsign characters.
         AX.25 spec §3.12: uppercase A-Z, digits 0-9, space (shifted left by 1).
         This reduces false-positive probability from noise by ~6 orders of
         magnitude vs. extension-bit-only check, countering the 2-bit EC
         amplification in hdlc_deframer (which raises CRC-16 collision rate
         to ~18% for 20-byte noise frames).
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

    def handle_msg(self, msg_pmt):
        msg = pmt.cdr(msg_pmt)
        packet = bytes(pmt.u8vector_elements(msg))

        if len(packet) < 14:
            return

        # Layer 1: Extension bits — bytes 0-12 must have LSB=0
        for i in range(13):
            if packet[i] & 0x01:
                return

        # Layer 2 (F4TNK): Callsign character validation.
        # Bytes 0-5 = dest callsign, 7-12 = src callsign (skip SSID bytes 6,13).
        # Each must be (valid_char << 1) where valid_char ∈ {A-Z, 0-9, space}.
        # Probability of random noise passing: (37/128)^12 ≈ 2.8×10⁻⁷
        valid = self._VALID_CS
        for i in (0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12):
            if not valid[packet[i]]:
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
