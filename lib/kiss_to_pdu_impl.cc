/* -*- c++ -*- */
/*
 * Copyright 2024 F4TNK
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "kiss_to_pdu_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace satellites {

kiss_to_pdu::sptr kiss_to_pdu::make(bool control_byte)
{
    return gnuradio::make_block_sptr<kiss_to_pdu_impl>(control_byte);
}

kiss_to_pdu_impl::kiss_to_pdu_impl(bool control_byte)
    : gr::sync_block("kiss_to_pdu",
                     gr::io_signature::make(1, 1, sizeof(uint8_t)),
                     gr::io_signature::make(0, 0, 0)),
      d_control_byte(control_byte),
      d_transpose(false),
      d_port(pmt::intern("out"))
{
    d_pdu.reserve(512);
    message_port_register_out(d_port);
}

kiss_to_pdu_impl::~kiss_to_pdu_impl() {}

void kiss_to_pdu_impl::flush_pdu()
{
    if (d_pdu.empty()) {
        return;
    }

    // If control_byte mode, check that the lower nibble of the
    // control byte is 0 (data frame, not a command).
    if (d_control_byte) {
        if (d_pdu[0] & 0x0F) {
            d_pdu.clear();
            return;
        }
        // Strip control byte — publish payload starting at index 1
        message_port_pub(
            d_port,
            pmt::cons(pmt::PMT_NIL,
                      pmt::init_u8vector(d_pdu.size() - 1, d_pdu.data() + 1)));
    } else {
        message_port_pub(
            d_port,
            pmt::cons(pmt::PMT_NIL,
                      pmt::init_u8vector(d_pdu.size(), d_pdu.data())));
    }

    d_pdu.clear();
}

int kiss_to_pdu_impl::work(int noutput_items,
                           gr_vector_const_void_star& input_items,
                           gr_vector_void_star& output_items)
{
    const uint8_t* in = static_cast<const uint8_t*>(input_items[0]);

    for (int i = 0; i < noutput_items; ++i) {
        const uint8_t c = in[i];

        if (c == FEND) {
            flush_pdu();
        } else if (d_transpose) {
            if (c == TFEND) {
                d_pdu.push_back(FEND);
            } else if (c == TFESC) {
                d_pdu.push_back(FESC);
            }
            // Any other byte after FESC is a protocol error — silently drop
            d_transpose = false;
        } else if (c == FESC) {
            d_transpose = true;
        } else {
            d_pdu.push_back(c);
        }
    }

    return noutput_items;
}

} /* namespace satellites */
} /* namespace gr */
