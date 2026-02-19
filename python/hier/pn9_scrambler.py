# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: PN9 scrambler
# Author: Daniel Estevez
# Description: PN9 scrambler (F4TNK: native PDU block — no tagged-stream overhead)
# GNU Radio version: 3.8.0.0

import numpy as np
import pmt
from gnuradio import gr


def _generate_pn9_sequence(length):
    """Generate PN9 pseudo-random sequence (x^9 + x^5 + 1, init=0x1FF).

    Matches GR digital.additive_scrambler_bb(0x21, 0x1FF, 8, bits_per_byte=8).
    Returns numpy uint8 array of 'length' bytes.
    """
    sr = 0x1FF
    seq = np.empty(length, dtype=np.uint8)
    for i in range(length):
        byte_val = 0
        for bit in range(8):
            # Feedback taps: bit 8 (x^9) XOR bit 4 (x^5)
            feedback = ((sr >> 8) ^ (sr >> 4)) & 1
            byte_val = (byte_val >> 1) | (feedback << 7)
            sr = ((sr << 1) | feedback) & 0x1FF
        seq[i] = byte_val
    return seq


# Precompute the full PN9 period (2^9 - 1 = 511 bytes) once at import time
_PN9_SEQ = _generate_pn9_sequence(511)


class pn9_scrambler(gr.basic_block):
    """PN9 additive scrambler — native PDU block.

    F4TNK optimization: replaces the 3-block hier_block2 chain
    (pdu_to_tagged_stream → additive_scrambler_bb → tagged_stream_to_pdu)
    with a single message handler using precomputed PN9 + numpy XOR.
    Eliminates 2 PDU copies and all tagged-stream scheduler overhead.
    """

    def __init__(self):
        gr.basic_block.__init__(
            self,
            name='PN9 scrambler',
            in_sig=None,
            out_sig=None,
        )
        self.message_port_register_in(pmt.intern('in'))
        self.message_port_register_out(pmt.intern('out'))
        self.set_msg_handler(pmt.intern('in'), self._handler)

    def _handler(self, pmt_msg):
        meta = pmt.car(pmt_msg)
        data = np.array(pmt.u8vector_elements(pmt.cdr(pmt_msg)),
                        dtype=np.uint8)
        n = len(data)
        if n == 0:
            return

        # Tile the precomputed PN9 sequence to cover the data length
        if n <= 511:
            pn9 = _PN9_SEQ[:n]
        else:
            reps = (n + 510) // 511
            pn9 = np.tile(_PN9_SEQ, reps)[:n]

        scrambled = np.bitwise_xor(data, pn9)

        self.message_port_pub(
            pmt.intern('out'),
            pmt.cons(meta, pmt.init_u8vector(n, scrambled)))
