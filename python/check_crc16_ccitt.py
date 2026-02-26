#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 Daniel Estevez <daniel@destevez.net>
#
# This file is part of gr-satellites
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import struct

from gnuradio import gr
import numpy
import pmt

from . import hdlc


class check_crc16_ccitt(gr.basic_block):
    """docstring for block check_crc16_ccitt"""
    def __init__(self, verbose):
        gr.basic_block.__init__(
            self,
            name='check_crc16_ccitt',
            in_sig=[],
            out_sig=[])

        self.verbose = verbose

        self.message_port_register_in(pmt.intern('in'))
        self.set_msg_handler(pmt.intern('in'), self.handle_msg)
        self.message_port_register_out(pmt.intern('ok'))
        self.message_port_register_out(pmt.intern('fail'))

    def handle_msg(self, msg_pmt):
        msg = pmt.cdr(msg_pmt)
        if not pmt.is_u8vector(msg):
            print('[ERROR] Received invalid message type. Expected u8vector')
            return
        packet = pmt.u8vector_elements(msg)

        if len(packet) < 3:
            return

        packet_out = packet[:-2]
        msg_out = pmt.cons(pmt.car(msg_pmt),
                           pmt.init_u8vector(len(packet_out), packet_out))

        crc = hdlc.crc_ccitt(packet_out)
        packet_crc = struct.unpack('<H', bytes(packet[-2:]))[0]
        if crc == packet_crc:
            if self.verbose:
                print('CRC OK')
            self.message_port_pub(pmt.intern('ok'), msg_out)
        else:
            # F4TNK It#5: 1-bit-flip CRC-16-CCITT retry
            corrected = self._try_1bit_flip(packet_out, packet_crc)
            if corrected is not None:
                if self.verbose:
                    print('CRC OK (1-bit corrected)')
                msg_corr = pmt.cons(
                    pmt.car(msg_pmt),
                    pmt.init_u8vector(len(corrected), corrected))
                self.message_port_pub(pmt.intern('ok'), msg_corr)
            else:
                if self.verbose:
                    print('CRC failed')
                self.message_port_pub(pmt.intern('fail'), msg_out)

    def _try_1bit_flip(self, packet_out, expected_crc):
        """F4TNK It#5: try flipping each bit in payload."""
        data = bytearray(packet_out)
        for byte_idx in range(len(data)):
            for bit_idx in range(8):
                data[byte_idx] ^= (1 << bit_idx)
                if hdlc.crc_ccitt(data) == expected_crc:
                    return bytes(data)
                data[byte_idx] ^= (1 << bit_idx)
        return None
