#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2017 Daniel Estevez <daniel@destevez.net>
#
# This file is part of gr-satellites
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import struct

from gnuradio import gr
import numpy
import pmt

from . import crc32c
from . import csp_header


class check_crc(gr.basic_block):
    """docstring for block check_crc"""
    def __init__(self, include_header, verbose, force=False):
        gr.basic_block.__init__(
            self,
            name='check_crc',
            in_sig=[],
            out_sig=[])

        self.include_header = include_header
        self.verbose = verbose
        self.force = force

        self.message_port_register_in(pmt.intern('in'))
        self.set_msg_handler(pmt.intern('in'), self.handle_msg)
        self.message_port_register_out(pmt.intern('ok'))
        self.message_port_register_out(pmt.intern('fail'))

    def _try_1bit_flip(self, packet, packet_crc):
        """
        F4TNK It#4: 1-bit-flip CRC-32C retry for CSP frames.

        When CRC-32C fails, try flipping each bit in the payload
        (excluding CSP header and CRC) and recompute. If a single
        bit-flip produces a matching CRC-32C, return the corrected
        packet. Otherwise return None.

        CRC-32C collision probability per flip: ~1/(2^32) ≈ 2.3e-10
        → essentially zero false positives.

        CPU cost: N_bytes × 8 CRC computations. For a typical
        CSP frame (< 256 bytes): ~2000 CRC ops ≈ 200-500 µs in Python.
        Only triggered on CRC-failed frames.
        """
        packet = bytearray(packet)
        # Determine CRC input range
        if self.include_header:
            crc_start = 0
        else:
            crc_start = 4  # skip CSP header
        crc_end = len(packet) - 4  # exclude CRC-32C trailer

        for byte_idx in range(crc_start, crc_end):
            for bit_idx in range(8):
                packet[byte_idx] ^= (1 << bit_idx)
                crc_data = packet[crc_start:crc_end]
                if crc32c.crc(crc_data) == packet_crc:
                    return bytes(packet)
                packet[byte_idx] ^= (1 << bit_idx)
        return None

    def handle_msg(self, msg_pmt):
        msg = pmt.cdr(msg_pmt)
        if not pmt.is_u8vector(msg):
            print('[ERROR] Received invalid message type. Expected u8vector')
            return
        packet = pmt.u8vector_elements(msg)
        try:
            header = csp_header.CSP(packet[:4])
        except ValueError as e:
            if self.verbose:
                print(e)
            return
        if not self.force and not header.crc:
            if self.verbose:
                print('CRC not used')
            self.message_port_pub(pmt.intern('ok'), msg_pmt)
        else:
            if len(packet) < 8:  # 4 bytes CSP header, 4 bytes CRC-32C
                if self.verbose:
                    print('Malformed CSP packet (too short)')
                return
            crc = crc32c.crc(packet[:-4]
                             if self.include_header
                             else packet[4:-4])
            packet_crc = struct.unpack('>I', bytes(packet[-4:]))[0]
            if crc == packet_crc:
                if self.verbose:
                    print('CRC OK')
                self.message_port_pub(pmt.intern('ok'), msg_pmt)
            else:
                # F4TNK It#4: try 1-bit-flip correction before declaring fail
                corrected = self._try_1bit_flip(packet, packet_crc)
                if corrected is not None:
                    if self.verbose:
                        print('CRC OK (1-bit corrected)')
                    corrected_pmt = pmt.cons(
                        pmt.car(msg_pmt),
                        pmt.init_u8vector(len(corrected), corrected))
                    self.message_port_pub(pmt.intern('ok'), corrected_pmt)
                else:
                    if self.verbose:
                        print('CRC failed')
                    self.message_port_pub(pmt.intern('fail'), msg_pmt)
