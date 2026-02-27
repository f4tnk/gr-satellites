#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2017 Daniel Estevez <daniel@destevez.net>
#
# This file is part of gr-satellites
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import collections

from gnuradio import gr
import numpy
import pmt

from .kiss import *

# Import C++ native implementation (F4TNK — 50-200× faster than Python)
# We search sys.modules for the satellites_python binding which was already
# loaded by __init__.py.  Direct relative import can fail in build-dir mode
# due to the 'python'→'satellites' module aliasing.
_kiss_to_pdu_cpp = None
import sys as _sys
for _key, _mod in _sys.modules.items():
    if _key.endswith('satellites_python') and hasattr(_mod, 'kiss_to_pdu'):
        _kiss_to_pdu_cpp = _mod.kiss_to_pdu
        break
if _kiss_to_pdu_cpp is None:
    try:
        from .bindings.satellites_python import kiss_to_pdu as _kiss_to_pdu_cpp
    except (ImportError, ModuleNotFoundError, RuntimeError):
        try:
            from .satellites_python import kiss_to_pdu as _kiss_to_pdu_cpp
        except (ImportError, ModuleNotFoundError, RuntimeError):
            _kiss_to_pdu_cpp = None


class _kiss_to_pdu_python(gr.sync_block):
    """Pure-Python fallback for kiss_to_pdu"""
    def __init__(self, control_byte=True):
        gr.sync_block.__init__(
            self,
            name='kiss_to_pdu',
            in_sig=[numpy.uint8],
            out_sig=[])

        self.pdu = list()
        self.transpose = False
        self.control_byte = control_byte

        self.message_port_register_out(pmt.intern('out'))

    def work(self, input_items, output_items):
        for c in input_items[0]:
            if c == FEND:
                if (self.pdu
                        and (not self.control_byte or not self.pdu[0] & 0x0f)):
                    msg = self.pdu[1:] if self.control_byte else self.pdu
                    self.message_port_pub(
                        pmt.intern('out'),
                        pmt.cons(pmt.PMT_NIL,
                                 pmt.init_u8vector(len(msg), msg)))
                self.pdu = list()
            elif self.transpose:
                if c == TFEND:
                    self.pdu.append(FEND)
                elif c == TFESC:
                    self.pdu.append(FESC)
                self.transpose = False
            elif c == FESC:
                self.transpose = True
            else:
                self.pdu.append(c)

        return len(input_items[0])


def kiss_to_pdu(control_byte=True):
    """Factory: returns C++ fast path or Python fallback.

    The C++ implementation is 50-200× faster: the per-byte KISS state
    machine runs entirely in compiled code with no GIL interaction.
    """
    if _kiss_to_pdu_cpp is not None:
        return _kiss_to_pdu_cpp(control_byte)
    return _kiss_to_pdu_python(control_byte)
