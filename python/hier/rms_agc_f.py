# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: RMS AGC
# Author: Daniel Estevez
# Description: AGC using RMS
# GNU Radio version: 3.8.0.0
#
# F4TNK optimization: replaced the 4-block hier chain
#   (rms_ff → multiply_const_ff → add_const_ff(1e-19) → divide_ff)
# with the single-pass C++ rms_agc_ff block.  Same algorithm, no inter-block
# copies, VOLK-accelerated squaring step.

from gnuradio import gr
from gnuradio.filter import firdes
import sys
import signal
from .. import rms_agc_ff as _rms_agc_ff_block


class rms_agc_f(gr.hier_block2):
    def __init__(self, alpha=1e-2, reference=0.5):
        gr.hier_block2.__init__(
            self,
            'RMS AGC',
            gr.io_signature(1, 1, gr.sizeof_float * 1),
            gr.io_signature(1, 1, gr.sizeof_float * 1),
        )

        self.alpha = alpha
        self.reference = reference

        # Single C++ block replaces the previous 4-block chain
        self.agc = _rms_agc_ff_block.make(float(alpha), float(reference))

        self.connect((self, 0), self.agc, (self, 0))

    def get_alpha(self):
        return self.alpha

    def set_alpha(self, alpha):
        self.alpha = alpha
        self.agc.set_alpha(float(alpha))

    def get_reference(self):
        return self.reference

    def set_reference(self, reference):
        self.reference = reference
        self.agc.set_reference(float(reference))
