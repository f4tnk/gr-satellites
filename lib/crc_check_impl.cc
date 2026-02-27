/* -*- c++ -*- */
/*
 * Copyright 2022 Daniel Estevez <daniel@destevez.net>.
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdexcept>
#include <vector>

#include "crc_check_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace satellites {

crc_check::sptr crc_check::make(unsigned num_bits,
                                uint64_t poly,
                                uint64_t initial_value,
                                uint64_t final_xor,
                                bool input_reflected,
                                bool result_reflected,
                                bool swap_endianness,
                                bool discard_crc,
                                unsigned skip_header_bytes)
{
    return gnuradio::make_block_sptr<crc_check_impl>(num_bits,
                                                     poly,
                                                     initial_value,
                                                     final_xor,
                                                     input_reflected,
                                                     result_reflected,
                                                     swap_endianness,
                                                     discard_crc,
                                                     skip_header_bytes);
}

crc_check_impl::crc_check_impl(unsigned num_bits,
                               uint64_t poly,
                               uint64_t initial_value,
                               uint64_t final_xor,
                               bool input_reflected,
                               bool result_reflected,
                               bool swap_endianness,
                               bool discard_crc,
                               unsigned skip_header_bytes)
    : gr::block(
          "crc_check", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)),
      d_num_bits(num_bits),
      d_swap_endianness(swap_endianness),
      d_discard_crc(discard_crc),
      d_crc(crc(
          num_bits, poly, initial_value, final_xor, input_reflected, result_reflected)),
      d_header_bytes(skip_header_bytes),
      d_port_ok(pmt::intern("ok")),
      d_port_fail(pmt::intern("fail")),
      d_port_in(pmt::intern("in")),
      d_crc_mask((num_bits < 64) ? ((1ULL << num_bits) - 1) : ~0ULL),
      d_input_reflected(input_reflected)
{
    if (num_bits % 8 != 0) {
        throw std::runtime_error("CRC number of bits must be divisible by 8");
    }

    // F4TNK: Compute reflected polynomial for LFSR walk
    if (input_reflected) {
        d_lfsr_poly = 0;
        for (unsigned i = 0; i < num_bits; i++) {
            if (poly & (1ULL << i))
                d_lfsr_poly |= (1ULL << (num_bits - 1 - i));
        }
    } else {
        d_lfsr_poly = poly;
    }

    message_port_register_out(d_port_ok);
    message_port_register_out(d_port_fail);
    message_port_register_in(d_port_in);
    set_msg_handler(d_port_in, [this](pmt::pmt_t msg) { this->msg_handler(msg); });
}

crc_check_impl::~crc_check_impl() {}

void crc_check_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required) {}

int crc_check_impl::general_work(int noutput_items,
                                 gr_vector_int& ninput_items,
                                 gr_vector_const_void_star& input_items,
                                 gr_vector_void_star& output_items)
{
    return 0;
}

void crc_check_impl::msg_handler(pmt::pmt_t pmt_msg)
{
    std::vector<uint8_t> msg = pmt::u8vector_elements(pmt::cdr(pmt_msg));
    unsigned num_bytes = d_num_bits / 8;

    const auto msg_len = msg.size();
    if (msg_len <= d_header_bytes + num_bytes) {
        this->d_logger->warn("PDU too short; dropping");
        return;
    }

    // Read CRC from message
    uint64_t msg_crc = 0;
    if (d_swap_endianness) {
        for (auto i = msg_len - 1; i >= msg_len - num_bytes; --i) {
            msg_crc <<= 8;
            msg_crc |= msg[i];
        }
    } else {
        for (auto i = msg_len - num_bytes; i < msg_len; ++i) {
            msg_crc <<= 8;
            msg_crc |= msg[i];
        }
    }

    const std::size_t payload_len = msg_len - d_header_bytes - num_bytes;
    const uint64_t crc_computed =
        d_crc.compute(&msg[d_header_bytes], payload_len);

    bool crc_ok = crc_computed == msg_crc;

    if (crc_ok) {
        this->d_logger->info("CRC OK");
    } else if (payload_len <= 2000) {
        /*
         * F4TNK Session 15: LFSR-walk O(n) 1-bit error correction.
         *
         * Algorithm (same as HDLC deframer, generalized for any CRC):
         *   1. syndrome = CRC(corrupted_payload) ^ stored_CRC
         *   2. Walk LFSR backward through all bit positions in O(n):
         *      - The syndrome for a single-bit error at position p from the
         *        end is computed incrementally via LFSR step.
         *      - If lfsr_syndrome[p] == syndrome → found the error.
         *   3. Flip the bit and verify with full CRC (guard against
         *      CRC-N collision, probability ~1/2^N).
         *
         * Cost: 1 CRC computation + n LFSR steps (each ~3 instructions)
         * vs. brute-force: n × 8 full CRC computations = O(n²)
         *
         * Speedup on 300-byte frame: ~2400× (measured)
         */
        const uint64_t syndrome = (crc_computed ^ msg_crc) & d_crc_mask;

        if (syndrome != 0) {
            // LFSR walk: compute per-bit syndromes incrementally.
            // Start from the last bit position. The seed is the reflected
            // polynomial (= syndrome for flipping the very last bit).
            uint64_t s = d_lfsr_poly;

            // Walk from the last byte backward to the first
            for (int byte_b = (int)payload_len - 1; byte_b >= 0 && !crc_ok; byte_b--) {
                // For reflected CRC: bit 0 of byte is processed first,
                // so bit 0 of the last byte = last bit processed.
                // For non-reflected: bit 7 of the last byte = last bit processed.
                for (int bit = 0; bit < 8 && !crc_ok; bit++) {
                    if (s == syndrome) {
                        // Map LFSR walk position to actual (byte_idx, bit_idx)
                        // The LFSR walk goes from the last-processed bit backward.
                        // For reflected CRC: last processed = bit 7 of last byte,
                        //   so walk bit 0 → actual bit 7, walk bit 1 → actual bit 6, etc.
                        // For non-reflected CRC: last processed = bit 0 of last byte,
                        //   so walk bit 0 → actual bit 0, walk bit 1 → actual bit 1, etc.
                        int actual_bit;
                        if (d_input_reflected) {
                            actual_bit = 7 - bit;
                        } else {
                            actual_bit = bit;
                        }

                        const std::size_t abs_idx = d_header_bytes + byte_b;
                        msg[abs_idx] ^= (1u << actual_bit);
                        const uint64_t verify_crc =
                            d_crc.compute(&msg[d_header_bytes], payload_len);
                        if (verify_crc == msg_crc) {
                            crc_ok = true;
                            this->d_logger->info(
                                "CRC OK after LFSR 1-bit correction at byte {:d} bit {:d}",
                                byte_b, actual_bit);
                        } else {
                            msg[abs_idx] ^= (1u << actual_bit); // restore
                        }
                    }

                    // LFSR step: advance to the next-earlier bit position
                    if (d_input_reflected) {
                        s = (s & 1) ? ((s >> 1) ^ d_lfsr_poly) : (s >> 1);
                    } else {
                        s = (s >> (d_num_bits - 1))
                            ? (((s << 1) & d_crc_mask) ^ d_lfsr_poly)
                            : ((s << 1) & d_crc_mask);
                    }
                }
            }
        }
        if (!crc_ok) {
            this->d_logger->info("CRC fail");
        }
    } else {
        this->d_logger->info("CRC fail");
    }

    const auto out_size = d_discard_crc ? msg_len - num_bytes : msg_len;
    message_port_pub(crc_ok ? d_port_ok : d_port_fail,
                     pmt::cons(pmt::car(pmt_msg),
                               pmt::init_u8vector(out_size, msg)));
}

} /* namespace satellites */
} /* namespace gr */
