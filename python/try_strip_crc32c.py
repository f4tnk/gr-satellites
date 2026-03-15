#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2025 F4TNK Melaine
#
# This file is part of gr-satellites
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

"""
Protocol-agnostic CRC-32C strip block.

Tries to verify the last 4 bytes of each PDU as a CRC-32C over
the preceding bytes.  If the CRC matches the frame is forwarded
*without* the trailing 4 CRC bytes.  If the CRC does not match
(or the frame is too short), the frame is forwarded unchanged.

False-positive probability: ~1 / 2^32 ≈ 2.3 × 10⁻¹⁰ per frame.
"""

import struct

from gnuradio import gr
import pmt

from . import crc32c


class try_strip_crc32c(gr.basic_block):
    """Try to verify and strip trailing CRC-32C from PDUs."""

    def __init__(self, verbose=False):
        gr.basic_block.__init__(
            self,
            name='try_strip_crc32c',
            in_sig=[],
            out_sig=[])

        self.verbose = verbose

        self.message_port_register_in(pmt.intern('in'))
        self.set_msg_handler(pmt.intern('in'), self.handle_msg)
        self.message_port_register_out(pmt.intern('out'))

    def handle_msg(self, msg_pmt):
        msg = pmt.cdr(msg_pmt)
        if not pmt.is_u8vector(msg):
            self.message_port_pub(pmt.intern('out'), msg_pmt)
            return

        packet = bytes(pmt.u8vector_elements(msg))

        # Need at least 5 bytes: 1 byte payload + 4 bytes CRC
        if len(packet) < 5:
            self.message_port_pub(pmt.intern('out'), msg_pmt)
            return

        payload = packet[:-4]
        packet_crc = struct.unpack('>I', packet[-4:])[0]
        computed_crc = crc32c.crc(payload)

        if computed_crc == packet_crc:
            if self.verbose:
                print(f'CRC-32C OK — stripped 4 bytes '
                      f'({len(packet)} → {len(payload)})')
            stripped = pmt.cons(
                pmt.car(msg_pmt),
                pmt.init_u8vector(len(payload), payload))
            self.message_port_pub(pmt.intern('out'), stripped)
        else:
            if self.verbose:
                print(f'CRC-32C mismatch — forwarding unchanged '
                      f'({len(packet)} bytes)')
            self.message_port_pub(pmt.intern('out'), msg_pmt)
