/* -*- c++ -*- */
/*
 * Copyright 2025 Daniel Estevez <daniel@destevez.net>.
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SATELLITES_KURTOSIS_IMPL_H
#define INCLUDED_SATELLITES_KURTOSIS_IMPL_H

#include <satellites/kurtosis.h>
#include <volk/volk_alloc.hh>

namespace gr {
namespace satellites {

class kurtosis_impl : public kurtosis
{
private:
    const size_t d_block_size;
    const size_t d_vlen;
    volk::vector<float> d_sq;   // |z|^2 scratch buffer
    volk::vector<float> d_sq4;  // |z|^4 scratch buffer

public:
    kurtosis_impl(size_t block_size, size_t vlen);
    ~kurtosis_impl() override;

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
};

} // namespace satellites
} // namespace gr

#endif /* INCLUDED_SATELLITES_KURTOSIS_IMPL_H */
