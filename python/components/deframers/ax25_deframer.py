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
    Checks that the Extension bits (LSB) of the first 13 bytes are 0.
    """
    def __init__(self):
        gr.basic_block.__init__(self, name="ax25_header_check", in_sig=[], out_sig=[])
        self.message_port_register_in(pmt.intern('in'))
        self.message_port_register_out(pmt.intern('out'))
        self.set_msg_handler(pmt.intern('in'), self.handle_msg)

    def handle_msg(self, msg_pmt):
        msg = pmt.cdr(msg_pmt)
        packet = bytes(pmt.u8vector_elements(msg))
        
        # Valid AX.25 frames usually have Dest(7) + Source(7).
        # The Extension Bit (LSB) should be 0 for all bytes of an address except the last one.
        # Since Dest is followed by Source, Dest's last byte (byte 6) also has LSB=0.
        # So bytes 0-12 should all have LSB=0.
        if len(packet) >= 14:
            valid = True
            for i in range(13):
                if (packet[i] & 0x01) != 0:
                    valid = False
                    break
            
            if valid:
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
