/* -*- c++ -*- */
/*
 * Copyright 2026 Daniel Estevez <daniel@destevez.net>.
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * F4TNK optimization: single-pass float RMS AGC replacing the 4-block hier.
 */

#include "rms_agc_ff_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <cmath>

namespace gr {
namespace satellites {

rms_agc_ff::sptr rms_agc_ff::make(float alpha, float reference)
{
    return gnuradio::make_block_sptr<rms_agc_ff_impl>(alpha, reference);
}

rms_agc_ff_impl::rms_agc_ff_impl(float alpha, float reference)
    : gr::sync_block("rms_agc_ff",
                     gr::io_signature::make(1, 1, sizeof(float)),
                     gr::io_signature::make(1, 1, sizeof(float))),
      d_alpha(alpha),
      d_one_minus_alpha(1.0f - alpha),
      d_reference(reference),
      d_rms_sq(reference * reference)
{
}

rms_agc_ff_impl::~rms_agc_ff_impl() {}

void rms_agc_ff_impl::set_alpha(float alpha)
{
    gr::thread::scoped_lock lock(d_setlock);
    d_alpha = alpha;
    d_one_minus_alpha = 1.0f - alpha;
}

void rms_agc_ff_impl::set_reference(float reference)
{
    gr::thread::scoped_lock lock(d_setlock);
    d_reference = reference;
}

int rms_agc_ff_impl::work(int noutput_items,
                          gr_vector_const_void_star& input_items,
                          gr_vector_void_star& output_items)
{
    const auto in = static_cast<const float*>(input_items[0]);
    auto out = static_cast<float*>(output_items[0]);
    const int n = noutput_items;

    if (static_cast<int>(d_sq_buf.size()) < n) {
        d_sq_buf.resize(n);
        d_gain_buf.resize(n);
    }

    // --- Step 1: x^2 per sample via VOLK (element-wise multiply) ---
    volk_32f_x2_multiply_32f(d_sq_buf.data(), in, in, n);

    // --- Step 2: sequential IIR for running RMS + per-sample gain ---
    const float alpha = d_alpha;
    const float one_m = d_one_minus_alpha;
    const float ref = d_reference;
    const float eps = ref * 1e-19f;
    float rms_sq = d_rms_sq;

    for (int j = 0; j < n; ++j) {
        rms_sq = one_m * rms_sq + alpha * d_sq_buf[j];
        d_gain_buf[j] = ref / (std::sqrt(rms_sq) + eps);
    }
    d_rms_sq = rms_sq;

    // --- Step 3: apply gain (vectorized by -O3 -march=native) ---
    volk_32f_x2_multiply_32f(out, in, d_gain_buf.data(), n);

    return noutput_items;
}

} /* namespace satellites */
} /* namespace gr */
