/* -*- c++ -*- */
/*
 * Copyright 2016,2020 Daniel Estevez <daniel@destevez.net>.
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <algorithm>
#include <array>
#include <climits>
#include <utility>

#include <cstdio>
#include <cstring>

#include "ax100_decode_impl.h"
#include <gnuradio/io_signature.h>

extern "C" {
#include "libfec/fec.h"
}

namespace gr {
namespace satellites {

ax100_decode::sptr ax100_decode::make(bool verbose)
{
    return gnuradio::make_block_sptr<ax100_decode_impl>(verbose);
}

/*
 * The private constructor
 */
ax100_decode_impl::ax100_decode_impl(bool verbose)
    : gr::block("ax100_decode",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_verbose(verbose)
{
    message_port_register_out(pmt::mp("out"));
    message_port_register_in(pmt::mp("in"));
    set_msg_handler(pmt::mp("in"), [this](pmt::pmt_t msg) { this->msg_handler(msg); });
}

/*
 * Our virtual destructor.
 */
ax100_decode_impl::~ax100_decode_impl() {}

void ax100_decode_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
}

int ax100_decode_impl::general_work(int noutput_items,
                                    gr_vector_int& ninput_items,
                                    gr_vector_const_void_star& input_items,
                                    gr_vector_void_star& output_items)
{
    return 0;
}

void ax100_decode_impl::msg_handler(pmt::pmt_t pmt_msg)
{
    size_t length(0);
    auto msg = pmt::u8vector_elements(pmt::cdr(pmt_msg), length);

    constexpr int kMinLenByte = 33;   // 1 length + 32 RS parity
    constexpr int kMaxLenByte = 255;  // max representable AX100 length field
    constexpr int kMaxFallbackCorrections = 8;

    if (length != d_data.size()) {
        if (d_verbose) {
            std::printf("AX100 RS decode skipped: invalid frame size %zu (expected 256).\n",
                        length);
        }
        return;
    }

    std::memcpy(d_data.data(), msg, d_data.size());

    auto try_decode_with_len = [&](uint8_t len_byte,
                                   std::array<uint8_t, 256>& out,
                                   int& rs_res,
                                   int& frame_len) {
        if (len_byte < kMinLenByte) {
            return false;
        }

        frame_len = (int)len_byte - 32 - 1;
        if (frame_len < 0 || frame_len > 222) {
            return false;
        }

        out = d_data;
        const int pad = 255 - (int)len_byte + 1;
        rs_res = decode_rs_8(&out[1], NULL, 0, pad);
        return rs_res >= 0;
    };

    const uint8_t rx_len = d_data[0];
    std::array<uint8_t, 256> best_data{};
    int best_rs_res = -1;
    int best_frame_len = -1;
    uint8_t best_len = rx_len;

    // Fast path: decode using received length byte.
    if (!try_decode_with_len(rx_len, best_data, best_rs_res, best_frame_len)) {
        // Fallback: search all plausible AX100 length values and choose the
        // candidate with the lowest RS correction count (tie: closest length).
        int best_score = INT_MAX;
        bool found = false;

        for (int candidate = kMinLenByte; candidate <= kMaxLenByte; candidate++) {
            std::array<uint8_t, 256> candidate_data{};
            int candidate_rs_res = -1;
            int candidate_frame_len = -1;
            if (!try_decode_with_len((uint8_t)candidate,
                                     candidate_data,
                                     candidate_rs_res,
                                     candidate_frame_len)) {
                continue;
            }

            if (candidate_rs_res > kMaxFallbackCorrections) {
                continue;
            }

            const int len_distance = std::abs(candidate - (int)rx_len);
            const int score = candidate_rs_res * 1024 + len_distance;
            if (score < best_score) {
                best_score = score;
                best_data = candidate_data;
                best_rs_res = candidate_rs_res;
                best_frame_len = candidate_frame_len;
                best_len = (uint8_t)candidate;
                found = true;
            }
        }

        if (!found) {
            if (d_verbose) {
                std::printf("RS decode failed.\n");
            }
            return;
        }

        if (d_verbose) {
            std::printf("RS decode recovered via length fallback: rx_len=%u -> len=%u. "
                        "Length: %d. Bytes corrected: %d.\n",
                        rx_len,
                        best_len,
                        best_frame_len,
                        best_rs_res);
        }
    }

    if (d_verbose) {
        std::printf("RS decode OK. Length: %d. Bytes corrected: %d.\n",
                    best_frame_len,
                    best_rs_res);
    }

    message_port_pub(pmt::mp("out"),
                     pmt::cons(pmt::PMT_NIL,
                               pmt::init_u8vector(best_frame_len, &best_data[1])));
}

} /* namespace satellites */
} /* namespace gr */
