#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2017, 2022 Daniel Estevez <daniel@destevez.net>
#
# This file is part of gr-satellites
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import collections

from gnuradio import gr
import numpy
import pmt

from . import crc, hdlc


def pack(s):
    # LSB-first packing: bit[0] is LSB of byte[0], bit[7] is MSB of byte[0]
    # numpy.packbits with bitorder='little' maps index 0 → bit 0 (LSB), identical semantics
    return numpy.packbits(numpy.array(s, dtype=numpy.uint8), bitorder='little').tobytes()


class hdlc_crc_check:
    def __init__(self):
        self.crc_calc = crc(16, 0x1021, 0xFFFF, 0xFFFF, True, True)

    def fcs_ok(self, frame):
        if len(frame) <= 2:
            return False
        out = self.crc_calc.compute(frame[:-2])
        return frame[-2] == (out & 0xff) and frame[-1] == ((out >> 8) & 0xff)


class hdlc_deframer(gr.sync_block):
    """docstring for block hdlc_deframer"""
    def __init__(self, check_fcs, max_length, crc_check_func=None):
        gr.sync_block.__init__(
            self,
            name='hdlc_deframer',
            in_sig=[numpy.uint8],
            out_sig=None)

        self.bits = collections.deque(maxlen=(max_length+2)*8 + 7)
        self.ones = 0  # consecutive ones for flag checking
        self.check = check_fcs
        if crc_check_func is not None:
            self.fcs_ok = crc_check_func
        else:
            self.crc_check = hdlc_crc_check()
            self.fcs_ok = self.crc_check.fcs_ok

        self.message_port_register_out(pmt.intern('out'))
        self._work_count = 0

    def work(self, input_items, output_items):
        """Exception-safe wrapper: catches any crash in work() to prevent
        the GR scheduler thread from dying, which would cascade-stall the
        entire flowgraph and cause 100% UDP drops (F4TNK Session 5)."""
        self._work_count += 1
        try:
            return self._work_inner(input_items, output_items)
        except BaseException as exc:
            import traceback
            import time as _t
            with open('/tmp/hdlc_crash.log', 'a') as _f:
                _f.write(_t.strftime('%H:%M:%S') + ' HDLC CRASH #'
                         + str(self._work_count) + ' type='
                         + type(exc).__name__ + ' msg='
                         + str(exc)[:200] + '\n')
                _f.write('  input_len=' + str(len(input_items[0]))
                         + ' ones=' + str(self.ones)
                         + ' bits_len=' + str(len(self.bits)) + '\n')
                traceback.print_exc(file=_f)
                _f.flush()
            return len(input_items[0])  # Keep block alive

    def _work_inner(self, input_items, output_items):
        in0 = input_items[0]

        for x in in0:
            if x:
                self.ones += 1
                self.bits.append(x)
            else:
                if self.ones == 5:
                    # destuff = do nothing
                    None
                elif self.ones > 5:
                    # Should be ones == 6 unless packet is corrupted.
                    # Flag received. Prepare to send frame
                    for _ in range(min(7, len(self.bits))):
                        # Remove 7 previous flag bits
                        self.bits.pop()
                    if len(self.bits) % 8:
                        # Pad on the left with 0's
                        self.bits.extendleft([0] * (8 - len(self.bits) % 8))
                    frame = pack(self.bits)
                    self.bits.clear()
                    if frame and (not self.check or self.fcs_ok(frame)):
                        # Send frame
                        buff = frame[:-2]  # trim fcs
                        self.message_port_pub(
                            pmt.intern('out'),
                            pmt.cons(pmt.PMT_NIL,
                                     pmt.init_u8vector(len(buff), buff)))
                else:
                    self.bits.append(x)
                self.ones = 0

        return len(input_items[0])
