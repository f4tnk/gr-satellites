/* -*- c++ -*- */
/*
 * Copyright 2022 Daniel Estevez <daniel@destevez.net>.
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_SATELLITES_CRC_CHECK_IMPL_H
#define INCLUDED_SATELLITES_CRC_CHECK_IMPL_H

#include <satellites/crc.h>
#include <satellites/crc_check.h>
#include <vector>

namespace gr {
namespace satellites {

class crc_check_impl : public crc_check
{
private:
    unsigned d_num_bits;
    bool d_swap_endianness;
    bool d_discard_crc;
    crc d_crc;
    unsigned d_header_bytes;

    // F4TNK: Pre-interned PMT symbols — avoid per-PDU hash lookup
    const pmt::pmt_t d_port_ok;
    const pmt::pmt_t d_port_fail;
    const pmt::pmt_t d_port_in;

    // F4TNK: CRC with IV=0, FX=0 for computing per-bit syndromes.
    // syndrome = CRC_0(error_pattern) = CRC(corrupted) ^ CRC(original)
    // Used for O(n) syndrome-based 1-bit error correction (vs O(n²) brute force).
    crc d_crc_zero;

    // F4TNK: Reflected polynomial for LFSR walk syndrome computation.
    // For reflected CRC: reflected_poly = bit_reverse(poly, num_bits)
    // For non-reflected CRC: d_lfsr_poly = poly
    uint64_t d_lfsr_poly;
    uint64_t d_crc_mask;
    bool d_input_reflected;

public:
    crc_check_impl(unsigned num_bits,
                   uint64_t poly,
                   uint64_t initial_value,
                   uint64_t final_xor,
                   bool input_reflected,
                   bool result_reflected,
                   bool swap_endianness,
                   bool discard_crc,
                   unsigned skip_header_bytes);
    ~crc_check_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
    void msg_handler(pmt::pmt_t pmt_msg);
};

} // namespace satellites
} // namespace gr

#endif /* INCLUDED_SATELLITES_CRC_CHECK_IMPL_H */
