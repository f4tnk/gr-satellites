/* -*- c++ -*- */
/*
 * Copyright 2025 Daniel Estevez <daniel@destevez.net>.
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "kurtosis_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>

namespace gr {
namespace satellites {

kurtosis::sptr kurtosis::make(size_t block_size, size_t vlen)
{
    return gnuradio::make_block_sptr<kurtosis_impl>(block_size, vlen);
}

kurtosis_impl::kurtosis_impl(size_t block_size, size_t vlen)
    : gr::sync_decimator("kurtosis",
                         gr::io_signature::make(1, 1, sizeof(gr_complex) * vlen),
                         gr::io_signature::make(1, 1, sizeof(float) * vlen),
                         block_size),
      d_block_size(block_size),
      d_vlen(vlen),
      d_sq(block_size),
      d_sq4(block_size)
{
}

kurtosis_impl::~kurtosis_impl() {}

int kurtosis_impl::work(int noutput_items,
                        gr_vector_const_void_star& input_items,
                        gr_vector_void_star& output_items)
{
    auto in = static_cast<const gr_complex*>(input_items[0]);
    auto out = static_cast<float*>(output_items[0]);

    const float M = static_cast<float>(d_block_size);
    for (size_t j = 0; j < static_cast<size_t>(noutput_items); ++j) {
        for (size_t k = 0; k < d_vlen; ++k) {
            if (d_vlen == 1) {
                // Fast path: input is a contiguous complex block
                volk_32fc_magnitude_squared_32f(
                    d_sq.data(), &in[j * d_block_size], d_block_size);
            } else {
                // Interleaved vlen layout: de-interleave component k
                for (size_t l = 0; l < d_block_size; ++l) {
                    const gr_complex z = in[(j * d_block_size + l) * d_vlen + k];
                    d_sq[l] = z.real() * z.real() + z.imag() * z.imag();
                }
            }
            // |z|^4 = (|z|^2)^2
            volk_32f_x2_multiply_32f(
                d_sq4.data(), d_sq.data(), d_sq.data(), d_block_size);
            float sum2 = 0.0f, sum4 = 0.0f;
            volk_32f_accumulator_s32f(&sum2, d_sq.data(), d_block_size);
            volk_32f_accumulator_s32f(&sum4, d_sq4.data(), d_block_size);
            const float kurt =
                M / (M - 1.0f) * ((M + 1.0f) * sum4 / (sum2 * sum2) - 2.0f);
            out[j * d_vlen + k] = kurt;
        }
    }
    return noutput_items;
}

} /* namespace satellites */
} /* namespace gr */
