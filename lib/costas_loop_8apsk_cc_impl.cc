/* -*- c++ -*- */
/*
 * Copyright 2022 Daniel Estevez <daniel@destevez.net>
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include "costas_loop_8apsk_cc_impl.h"
#include <gnuradio/expj.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/math.h>
#include <volk/volk.h>
#include <algorithm>

namespace gr {
namespace satellites {

costas_loop_8apsk_cc::sptr costas_loop_8apsk_cc::make(float loop_bw)
{
    return gnuradio::make_block_sptr<costas_loop_8apsk_cc_impl>(loop_bw);
}

static int ios[] = { sizeof(gr_complex), sizeof(float), sizeof(float), sizeof(float) };
static std::vector<int> iosig(ios, ios + sizeof(ios) / sizeof(int));

costas_loop_8apsk_cc_impl::costas_loop_8apsk_cc_impl(float loop_bw)
    : gr::sync_block("costas_loop_8apsk_cc",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                     gr::io_signature::makev(1, 4, iosig)),
      blocks::control_loop(loop_bw, 0.1, -0.1),
      d_error(0)
{
}

costas_loop_8apsk_cc_impl::~costas_loop_8apsk_cc_impl() {}

int costas_loop_8apsk_cc_impl::work(int noutput_items,
                                    gr_vector_const_void_star& input_items,
                                    gr_vector_void_star& output_items)
{
    auto in = static_cast<const gr_complex*>(input_items[0]);
    auto out = static_cast<gr_complex*>(output_items[0]);
    auto freq_out =
        output_items.size() >= 2 ? static_cast<float*>(output_items[1]) : NULL;
    auto phase_out =
        output_items.size() >= 3 ? static_cast<float*>(output_items[2]) : NULL;
    auto error_out =
        output_items.size() >= 4 ? static_cast<float*>(output_items[3]) : NULL;

    // F4TNK: Mini-batch VOLK rotator path when diagnostic ports are not connected.
    // Instead of computing gr_expj() per sample, apply a VOLK rotator for M samples
    // and update the loop only once per batch. Saves (M-1) sin/cos per batch.
    if (!freq_out) {
        static constexpr int BATCH = 8;
        int j = 0;
        while (j < noutput_items) {
            const int batch = std::min(BATCH, noutput_items - j);
            gr_complex nco = gr_expj(-d_phase);
            const gr_complex rot = gr_expj(-d_freq);
            // VOLK rotator2: out[k] = in[k] * nco; nco *= rot
            volk_32fc_s32fc_x2_rotator2_32fc(out + j, in + j, &rot, &nco, batch);
            // Advance phase by freq * batch (matches rotator state)
            d_phase += d_freq * batch;
            // Compute error from last output in batch and update loop
            d_error = phase_detector(out[j + batch - 1]);
            advance_loop(d_error);
            phase_wrap();
            frequency_limit();
            j += batch;
        }
    } else {
        // Original per-sample path (needed for diagnostic output ports)
        for (int j = 0; j < noutput_items; ++j) {
            const gr_complex nco_out = gr_expj(-d_phase);
            gr::fast_cc_multiply(out[j], in[j], nco_out);
            d_error = phase_detector(out[j]);
            advance_loop(d_error);
            phase_wrap();
            frequency_limit();

            freq_out[j] = d_freq;
            if (phase_out) {
                phase_out[j] = d_phase;
                if (error_out) {
                    error_out[j] = d_error;
                }
            }
        }
    }

    return noutput_items;
}

} /* namespace satellites */
} /* namespace gr */
