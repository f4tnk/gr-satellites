/* -*- c++ -*- */
/*
 * Copyright 2022 Daniel Estevez <daniel@destevez.net>
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <satellites/crc.h>
#include <stdexcept>
#include <cstdint>

namespace gr {
namespace satellites {

crc::crc(unsigned num_bits,
         uint64_t poly,
         uint64_t initial_value,
         uint64_t final_xor,
         bool input_reflected,
         bool result_reflected)
    : d_num_bits(num_bits),
      d_mask(num_bits == 64 ? ~static_cast<uint64_t>(0)
                            : (static_cast<uint64_t>(1) << num_bits) - 1),
      d_initial_value(initial_value & d_mask),
      d_final_xor(final_xor & d_mask),
      d_input_reflected(input_reflected),
      d_result_reflected(result_reflected)
{
    if ((num_bits < 8) || (num_bits > 64)) {
        throw std::runtime_error("CRC number of bits must be between 8 and 64");
    }

    d_table[0] = 0;
    if (d_input_reflected) {
        poly = reflect(poly);
        uint64_t crc = 1;
        int i = 128;
        do {
            if (crc & 1) {
                crc = (crc >> 1) ^ poly;
            } else {
                crc >>= 1;
            }
            for (int j = 0; j < 256; j += 2 * i) {
                d_table[i + j] = (crc ^ d_table[j]) & d_mask;
            }
            i >>= 1;
        } while (i > 0);
    } else {
        const uint64_t msb = static_cast<uint64_t>(1) << (num_bits - 1);
        uint64_t crc = msb;
        int i = 1;
        do {
            if (crc & msb) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
            for (int j = 0; j < i; ++j) {
                d_table[i + j] = (crc ^ d_table[j]) & d_mask;
            }
            i <<= 1;
        } while (i < 256);
    }

    // F4TNK: Build 3 additional tables for slice-by-4 acceleration.
    // d_table1[i] = CRC of (d_table[i] >> 8) fed through d_table
    // d_table2[i] = CRC of d_table1[i] fed through d_table, etc.
    // This allows processing 4 bytes at a time in compute().
    if (d_input_reflected) {
        for (int i = 0; i < 256; ++i) {
            d_table1[i] = d_table[(d_table[i]) & 0xff] ^ (d_table[i] >> 8);
            d_table1[i] &= d_mask;
        }
        for (int i = 0; i < 256; ++i) {
            d_table2[i] = d_table[(d_table1[i]) & 0xff] ^ (d_table1[i] >> 8);
            d_table2[i] &= d_mask;
        }
        for (int i = 0; i < 256; ++i) {
            d_table3[i] = d_table[(d_table2[i]) & 0xff] ^ (d_table2[i] >> 8);
            d_table3[i] &= d_mask;
        }
    } else {
        const unsigned shift = d_num_bits - 8;
        for (int i = 0; i < 256; ++i) {
            d_table1[i] = d_table[(d_table[i] >> shift) & 0xff] ^ ((d_table[i] << 8) & d_mask);
        }
        for (int i = 0; i < 256; ++i) {
            d_table2[i] = d_table[(d_table1[i] >> shift) & 0xff] ^ ((d_table1[i] << 8) & d_mask);
        }
        for (int i = 0; i < 256; ++i) {
            d_table3[i] = d_table[(d_table2[i] >> shift) & 0xff] ^ ((d_table2[i] << 8) & d_mask);
        }
    }
}

crc::~crc() {}

uint64_t crc::compute(const uint8_t* data, std::size_t len)
{
    uint64_t rem = d_initial_value;

    if (d_input_reflected) {
        // F4TNK: slice-by-4 — process 4 bytes at a time
        while (len >= 4) {
            uint8_t b0 = data[0] ^ (uint8_t)(rem);
            uint8_t b1 = data[1] ^ (uint8_t)(rem >> 8);
            uint8_t b2 = data[2] ^ (uint8_t)(rem >> 16);
            uint8_t b3 = data[3] ^ (uint8_t)(rem >> 24);
            rem = d_table3[b0] ^ d_table2[b1] ^ d_table1[b2] ^ d_table[b3]
                  ^ (rem >> 32);
            data += 4;
            len -= 4;
        }
        // Byte-by-byte tail
        for (std::size_t i = 0; i < len; ++i) {
            uint8_t idx = (rem ^ data[i]) & 0xff;
            rem = d_table[idx] ^ (rem >> 8);
        }
    } else {
        const unsigned shift = d_num_bits - 8;
        // F4TNK: slice-by-4 for non-reflected mode (only safe when num_bits >= 32)
        if (d_num_bits >= 32) {
            while (len >= 4) {
                uint8_t b0 = data[0] ^ (uint8_t)(rem >> shift);
                uint8_t b1 = data[1] ^ (uint8_t)(rem >> (shift - 8));
                uint8_t b2 = data[2] ^ (uint8_t)(rem >> (shift - 16));
                uint8_t b3 = data[3] ^ (uint8_t)(rem >> (shift - 24));
                rem = d_table3[b0] ^ d_table2[b1] ^ d_table1[b2] ^ d_table[b3]
                      ^ ((rem << 32) & d_mask);
                data += 4;
                len -= 4;
            }
        }
        // Byte-by-byte tail (or full loop for num_bits < 32)
        for (std::size_t i = 0; i < len; ++i) {
            uint8_t idx = ((rem >> shift) ^ data[i]) & 0xff;
            rem = (d_table[idx] ^ (rem << 8)) & d_mask;
        }
    }

    if (d_input_reflected != d_result_reflected) {
        rem = reflect(rem);
    }

    rem = rem ^ d_final_xor;
    return rem;
}

uint64_t crc::reflect(uint64_t word)
{
    // Fast bit-reversal using the SWAR (bit-parallel) algorithm.
    // Reverses all 64 bits then shifts right to keep only d_num_bits.
    // ~10x faster than the previous O(n) loop for CRC-32/CRC-16.
    uint64_t v = word;
    v = ((v & UINT64_C(0xAAAAAAAAAAAAAAAA)) >> 1)  | ((v & UINT64_C(0x5555555555555555)) << 1);
    v = ((v & UINT64_C(0xCCCCCCCCCCCCCCCC)) >> 2)  | ((v & UINT64_C(0x3333333333333333)) << 2);
    v = ((v & UINT64_C(0xF0F0F0F0F0F0F0F0)) >> 4)  | ((v & UINT64_C(0x0F0F0F0F0F0F0F0F)) << 4);
    v = ((v & UINT64_C(0xFF00FF00FF00FF00)) >> 8)  | ((v & UINT64_C(0x00FF00FF00FF00FF)) << 8);
    v = ((v & UINT64_C(0xFFFF0000FFFF0000)) >> 16) | ((v & UINT64_C(0x0000FFFF0000FFFF)) << 16);
    v = (v >> 32) | (v << 32);
    // Result is the full 64-bit reversal; shift right so the result occupies
    // the least-significant d_num_bits.
    return v >> (64 - d_num_bits);
}

} /* namespace satellites */
} /* namespace gr */
