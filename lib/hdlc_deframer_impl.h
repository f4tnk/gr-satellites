/* -*- c++ -*- */
/*
 * Copyright 2017, 2022 Daniel Estevez <daniel@destevez.net>
 * Copyright 2025 F4TNK — C++ rewrite of the Python hdlc_deframer block
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_SATELLITES_HDLC_DEFRAMER_IMPL_H
#define INCLUDED_SATELLITES_HDLC_DEFRAMER_IMPL_H

#include <satellites/hdlc_deframer.h>
#include <cstdint>
#include <vector>

namespace gr {
namespace satellites {

class hdlc_deframer_impl : public hdlc_deframer
{
private:
    const bool d_check_fcs;
    const int d_max_bytes;           // max_length + 2 (payload + FCS)
    std::vector<uint8_t> d_pktbuf;   // frame byte accumulator
    int d_byte_count;                // complete bytes in d_pktbuf
    int d_bit_pos;                   // bits accumulated in current byte (0-7)
    int d_ones;                      // consecutive ones counter
    const pmt::pmt_t d_port;

    // F4TNK: persistent error-correction buffers (allocated once, reused per frame)
    struct bit_info {
        int byte_idx;
        int bit_idx;
        uint16_t syndrome;
    };
    std::vector<bit_info> d_bits_ec;           // syndrome table for EC
    std::vector<int32_t> d_syn_index;          // syndrome→position lookup (size 65536)
    std::vector<uint16_t> d_syn_index_touched; // touched syndromes for fast reset

    /*!
     * \brief CRC-16-CCITT (ISO HDLC / X.25 / AX.25 FCS)
     *
     * Poly 0x1021 reflected = 0x8408, init 0xFFFF, final XOR 0xFFFF,
     * reflect-in, reflect-out.
     */
    static uint16_t crc_ccitt(const uint8_t* data, size_t len);

    /*!
     * \brief Check FCS of a complete frame (payload + 2-byte FCS).
     * \return true if FCS matches computed CRC.
     */
    static bool fcs_ok(const uint8_t* frame, size_t len);

    /*!
     * \brief Append a single bit (0 or 1) to the packet buffer.
     */
    inline void push_bit(uint8_t bit);

    /*!
     * \brief Process a completed frame (after flag detection).
     */
    void process_frame();

public:
    hdlc_deframer_impl(bool check_fcs, int max_length);
    ~hdlc_deframer_impl() override;

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
};

} // namespace satellites
} // namespace gr

#endif /* INCLUDED_SATELLITES_HDLC_DEFRAMER_IMPL_H */
