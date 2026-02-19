/* -*- c++ -*- */
/*
 * Copyright 2019-2020 Daniel Estevez <daniel@destevez.net>
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "distributed_syncframe_soft_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>

namespace gr {
namespace satellites {

distributed_syncframe_soft::sptr
distributed_syncframe_soft::make(int threshold, const std::string& syncword, int step)
{
    return gnuradio::make_block_sptr<distributed_syncframe_soft_impl>(
        threshold, syncword, step);
}

/*
 * The private constructor
 */
distributed_syncframe_soft_impl::distributed_syncframe_soft_impl(
    int threshold, const std::string& syncword, int step)
    : gr::sync_block("distributed_syncframe_soft",
                     gr::io_signature::make(1, 1, sizeof(float)),
                     gr::io_signature::make(0, 0, 0)),
      d_threshold(threshold),
      d_step(step)
{
    // look at LSB only, as in correlate_access_code_bb_impl.cc
    for (auto s : syncword)
        d_syncword.push_back(s & 1);

    // F4TNK: precompute ±1.0f soft syncword for VOLK dot_prod (step=1)
    d_syncword_soft.resize(d_syncword.size());
    for (size_t j = 0; j < d_syncword.size(); ++j) {
        // syncword bit 1 → expect negative soft symbol → multiply by -1
        // syncword bit 0 → expect positive soft symbol → multiply by +1
        d_syncword_soft[j] = d_syncword[j] ? -1.0f : 1.0f;
    }
    // Soft threshold: hard match >= N - threshold  ↔  dot_prod >= N - 2*threshold
    // (each matching bit contributes +|x|≈+1, each mismatch -|x|≈-1)
    d_soft_threshold = static_cast<float>(d_syncword.size() - 2 * d_threshold);

    set_history(d_syncword.size() * d_step);

    message_port_register_out(pmt::mp("out"));
}

/*
 * Our virtual destructor.
 */
distributed_syncframe_soft_impl::~distributed_syncframe_soft_impl() {}

int distributed_syncframe_soft_impl::work(int noutput_items,
                                          gr_vector_const_void_star& input_items,
                                          gr_vector_void_star& output_items)
{
    const float* in = (const float*)input_items[0];

    if (d_step == 1) {
        // F4TNK: VOLK dot_prod fast path for step=1
        // Soft correlation: metric = Σ in[j] * syncword_soft[j]
        // Positive metric indicates correlation; threshold scaled accordingly
        const size_t sw_len = d_syncword.size();
        for (int i = 0; i < noutput_items; i++) {
            float metric;
            volk_32f_x2_dot_prod_32f(&metric, in + i,
                                     d_syncword_soft.data(), sw_len);
            if (metric >= d_soft_threshold) {
                message_port_pub(
                    pmt::mp("out"),
                    pmt::cons(pmt::PMT_NIL,
                              pmt::init_f32vector(sw_len, in + i)));
            }
        }
    } else {
        // Original scalar path for step > 1 (gather pattern not VOLK-able)
        for (int i = 0; i < noutput_items; i++) {
            size_t match = 0;
            for (size_t j = 0; j < d_syncword.size(); ++j) {
                match += (in[i + j * d_step] < 0.0) ^ d_syncword[j];
            }
            if (match >= d_syncword.size() - d_threshold) {
                message_port_pub(
                    pmt::mp("out"),
                    pmt::cons(pmt::PMT_NIL,
                              pmt::init_f32vector(d_syncword.size() * d_step,
                                                  in + i)));
            }
        }
    }

    return noutput_items;
}

} /* namespace satellites */
} /* namespace gr */
