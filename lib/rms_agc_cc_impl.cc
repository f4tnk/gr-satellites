/* -*- c++ -*- */
/*
 * Copyright 2026 Daniel Estevez <daniel@destevez.net>.
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * F4TNK optimization: single-pass complex RMS AGC replacing the 5-block hier.
 */

#include "rms_agc_cc_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <cmath>

namespace gr {
namespace satellites {

rms_agc_cc::sptr rms_agc_cc::make(float alpha, float reference)
{
    return gnuradio::make_block_sptr<rms_agc_cc_impl>(alpha, reference);
}

rms_agc_cc_impl::rms_agc_cc_impl(float alpha, float reference)
    : gr::sync_block("rms_agc_cc",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                     gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_alpha(alpha),
      d_one_minus_alpha(1.0f - alpha),
      d_reference(reference),
      d_rms_sq(reference * reference) // initialise near target level
{
}

rms_agc_cc_impl::~rms_agc_cc_impl() {}

void rms_agc_cc_impl::set_alpha(float alpha)
{
    gr::thread::scoped_lock lock(d_setlock);
    d_alpha = alpha;
    d_one_minus_alpha = 1.0f - alpha;
}

void rms_agc_cc_impl::set_reference(float reference)
{
    gr::thread::scoped_lock lock(d_setlock);
    d_reference = reference;
}

int rms_agc_cc_impl::work(int noutput_items,
                          gr_vector_const_void_star& input_items,
                          gr_vector_void_star& output_items)
{
    const auto in = static_cast<const gr_complex*>(input_items[0]);
    auto out = static_cast<gr_complex*>(output_items[0]);
    const int n = noutput_items;

    // Grow scratch buffers as needed (volk::vector guarantees SIMD alignment)
    if (static_cast<int>(d_sq_buf.size()) < n) {
        d_sq_buf.resize(n);
        d_gain_buf.resize(n);
    }

    // --- Step 1: compute |z|^2 for the whole block using VOLK ---
    // On AVX2: processes ~8 complex samples per cycle (vs. 1 scalar)
    volk_32fc_magnitude_squared_32f(d_sq_buf.data(), in, n);

    // --- Step 2a: sequential IIR pass (just the recursive update, no sqrt) ---
    // rms_sq[n] = (1-alpha)*rms_sq[n-1] + alpha*|z[n]|^2
    // This pass MUST remain sequential (each value depends on the previous),
    // but removing the per-sample sqrt makes it ~3× faster per iteration.
    const float alpha = d_alpha;
    const float one_m = d_one_minus_alpha;
    const float ref = d_reference;
    float rms_sq = d_rms_sq;

    for (int j = 0; j < n; ++j) {
        rms_sq = one_m * rms_sq + alpha * d_sq_buf[j];
        d_gain_buf[j] = rms_sq + 1e-30f; // floor to prevent invsqrt(0)
    }
    d_rms_sq = rms_sq;

    // --- Step 2b: batch gain = ref / sqrt(rms_sq) via VOLK SIMD ---
    // invsqrt: AVX2 uses vrsqrtps + Newton-Raphson → 8 floats/cycle
    volk_32f_invsqrt_32f(d_gain_buf.data(), d_gain_buf.data(), n);
    // scale by reference level
    volk_32f_s32f_multiply_32f(d_gain_buf.data(), d_gain_buf.data(), ref, n);

    // --- Step 3: apply real gain to complex samples via VOLK SIMD ---
    // AVX2: processes 8 complex samples/cycle (vs. 1 scalar)
    volk_32fc_32f_multiply_32fc(out, in, d_gain_buf.data(), n);

    return noutput_items;
}

} /* namespace satellites */
} /* namespace gr */
