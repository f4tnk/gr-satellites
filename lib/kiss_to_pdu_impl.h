/* -*- c++ -*- */
/*
 * Copyright 2024 F4TNK
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_SATELLITES_KISS_TO_PDU_IMPL_H
#define INCLUDED_SATELLITES_KISS_TO_PDU_IMPL_H

#include <satellites/kiss_to_pdu.h>
#include <cstdint>
#include <vector>

namespace gr {
namespace satellites {

class kiss_to_pdu_impl : public kiss_to_pdu
{
private:
    // KISS protocol constants
    static constexpr uint8_t FEND  = 0xC0;
    static constexpr uint8_t FESC  = 0xDB;
    static constexpr uint8_t TFEND = 0xDC;
    static constexpr uint8_t TFESC = 0xDD;

    const bool d_control_byte;
    bool d_transpose;
    std::vector<uint8_t> d_pdu;
    const pmt::pmt_t d_port;

    void flush_pdu();

public:
    kiss_to_pdu_impl(bool control_byte);
    ~kiss_to_pdu_impl() override;

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
};

} // namespace satellites
} // namespace gr

#endif /* INCLUDED_SATELLITES_KISS_TO_PDU_IMPL_H */
