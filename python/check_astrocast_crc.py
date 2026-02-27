#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2018, 2022 Daniel Estevez <daniel@destevez.net>
#
# This file is part of gr-satellites
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

from gnuradio import gr
import numpy
import pmt

from .hdlc_deframer import hdlc_crc_check


class check_astrocast_crc(gr.basic_block):
    """docstring for block check_astrocast_crc"""
    def __init__(self, verbose):
        gr.basic_block.__init__(
            self,
            name='check_astrocast_crc',
            in_sig=[],
            out_sig=[])

        self.verbose = verbose
        self.crc_check = hdlc_crc_check()

        self.message_port_register_in(pmt.intern('in'))
        self.set_msg_handler(pmt.intern('in'), self.handle_msg)
        self.message_port_register_out(pmt.intern('ok'))
        self.message_port_register_out(pmt.intern('fail'))

    def handle_msg(self, msg_pmt):
        msg = pmt.cdr(msg_pmt)
        if not pmt.is_u8vector(msg):
            print('[ERROR] Received invalid message type. Expected u8vector')
            return
        packet = pmt.u8vector_elements(msg)[1:]  # drop initial 0x7e

        # Find final 0x7e
        try:
            idx = packet.index(0x7e)
        except ValueError:
            return

        packet_out = packet[:idx-2]
        msg_out = pmt.cons(pmt.car(msg_pmt),
                           pmt.init_u8vector(len(packet_out), packet_out))
        if self.crc_check.fcs_ok(packet[:idx]):
            if self.verbose:
                print('CRC OK')
            self.message_port_pub(pmt.intern('ok'), msg_out)
        else:
            # F4TNK It#6: 1-bit-flip CRC retry
            corrected = self._try_1bit_flip(packet[:idx])
            if corrected is not None:
                if self.verbose:
                    print('CRC OK (1-bit corrected)')
                pkt_out = corrected[:-2]
                msg_corr = pmt.cons(
                    pmt.car(msg_pmt),
                    pmt.init_u8vector(len(pkt_out), pkt_out))
                self.message_port_pub(pmt.intern('ok'), msg_corr)
            else:
                if self.verbose:
                    print('CRC failed')
                self.message_port_pub(pmt.intern('fail'), msg_out)

    def _try_1bit_flip(self, frame):
        """F4TNK It#6: try flipping each bit in payload (excl. FCS)."""
        frame = bytearray(frame)
        for byte_idx in range(len(frame) - 2):
            for bit_idx in range(8):
                frame[byte_idx] ^= (1 << bit_idx)
                if self.crc_check.fcs_ok(frame):
                    return bytes(frame)
                frame[byte_idx] ^= (1 << bit_idx)
        return None
