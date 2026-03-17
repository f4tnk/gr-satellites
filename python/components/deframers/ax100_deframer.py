#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2019 Daniel Estevez <daniel@destevez.net>
#
# This file is part of gr-satellites
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

from gnuradio import gr, digital

from ... import ax100_decode, u482c_decode
from ...hier.sync_to_pdu_packed import sync_to_pdu_packed
from ...utils.options_block import options_block


_syncword = '10010011000010110101000111011110'


class ax100_deframer(gr.hier_block2, options_block):
    """
    Hierarchical block to deframe the GOMspace AX100 protocols.

    The input is a float stream of soft symbols. The output are PDUs
    with frames (most likely CSP frames).

    Args:
        mode: mode to use ('RS' or 'ASM') (string)
        scrambler: scrambler to use (only for ASM mode) (str)
                   'CCSDS': force CCSDS scrambler ON + RS ON
                   'none': scrambler OFF, RS ON (backward compat)
                   'auto': use Golay header flags to decide
                   'none_nofec': force all FEC OFF (scrambler + RS)
        syncword_threshold: number of bit errors allowed in syncword (int)
        syncword: syncword to use (str)
        options: Options from argparse
    """
    def __init__(self, mode, scrambler='CCSDS', syncword_threshold=None,
                 syncword=_syncword, options=None):
        gr.hier_block2.__init__(
            self,
            'ax100_deframer',
            gr.io_signature(1, 1, gr.sizeof_float),
            gr.io_signature(0, 0, 0))
        options_block.__init__(self, options)

        if scrambler not in ['CCSDS', 'none', 'auto', 'none_nofec']:
            raise ValueError(f'invalid scrambler {scrambler}')

        self.message_port_register_hier_out('out')

        if syncword_threshold is None:
            syncword_threshold = self.options.syncword_threshold

        if mode not in ['RS', 'ASM']:
            raise Exception("Unsupported AX100 mode. Use 'RS' or 'ASM'")

        # u482c_decode FEC mode values: OFF=0, ON=1, AUTO=-1
        if scrambler == 'CCSDS':
            scr_mode = 1   # ON
            rs_mode = 1    # ON
        elif scrambler == 'auto':
            scr_mode = -1  # AUTO (use Golay flags)
            rs_mode = -1   # AUTO (use Golay flags)
        elif scrambler == 'none_nofec':
            scr_mode = 0   # OFF
            rs_mode = 0    # OFF
        else:  # 'none'
            scr_mode = 0   # OFF
            rs_mode = 1    # ON (backward compatible)

        self.slicer = digital.binary_slicer_fb()
        if mode == 'RS':
            self.descrambler = digital.descrambler_bb(0x21, 0, 16)
        self.deframer = sync_to_pdu_packed(
            packlen=256 if mode == 'RS' else 258,
            sync=syncword, threshold=syncword_threshold)
        self.fec = (ax100_decode(self.options.verbose_fec) if mode == 'RS'
                    else u482c_decode(
                        self.options.verbose_fec, 0,
                        scr_mode, rs_mode))
        self._blocks = [self, self.slicer]
        if mode == 'RS':
            self._blocks.append(self.descrambler)
        self._blocks += [self.deframer]

        self.connect(*self._blocks)
        self.msg_connect((self.deframer, 'out'), (self.fec, 'in'))
        self.msg_connect((self.fec, 'out'), (self, 'out'))

    # F4TNK It#1: increased from 4 to 5 — both RS and ASM+u482c modes
    # have RS FEC downstream that corrects errors in the payload.
    _default_sync_threshold = 5

    @classmethod
    def add_options(cls, parser):
        """
        Adds AX100 deframer specific options to the argparse parser
        """
        parser.add_argument(
            '--syncword_threshold', type=int,
            default=cls._default_sync_threshold,
            help='Syncword bit errors [default=%(default)r]')
        parser.add_argument(
            '--verbose_fec', action='store_true', help='Verbose FEC decoder')
