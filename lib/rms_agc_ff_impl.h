/* -*- c++ -*- */
/*
 * Copyright 2026 Daniel Estevez <daniel@destevez.net>.
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SATELLITES_RMS_AGC_FF_IMPL_H
#define INCLUDED_SATELLITES_RMS_AGC_FF_IMPL_H

#include <satellites/rms_agc_ff.h>
#include <volk/volk_alloc.hh>

namespace gr {
namespace satellites {

class rms_agc_ff_impl : public rms_agc_ff
{
private:
    float d_alpha;
    float d_one_minus_alpha;
    float d_reference;
    float d_rms_sq; // running EMA of x^2

    volk::vector<float> d_sq_buf;   // x^2 per sample
    volk::vector<float> d_gain_buf; // per-sample gain

public:
    rms_agc_ff_impl(float alpha, float reference);
    ~rms_agc_ff_impl() override;

    void set_alpha(float alpha) override;
    void set_reference(float reference) override;

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
};

} // namespace satellites
} // namespace gr

#endif /* INCLUDED_SATELLITES_RMS_AGC_FF_IMPL_H */
