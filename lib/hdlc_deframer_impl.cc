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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "hdlc_deframer_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>

namespace gr {
namespace satellites {

hdlc_deframer::sptr hdlc_deframer::make(bool check_fcs, int max_length)
{
    return gnuradio::make_block_sptr<hdlc_deframer_impl>(check_fcs, max_length);
}

/*
 * CRC-16-CCITT (reflected / LSB-first)
 *
 * Equivalent to:
 *   Python:  crc(16, 0x1021, 0xFFFF, 0xFFFF, True, True).compute(data)
 *   aka CRC-16/X-25, CRC-16/ISO-HDLC, AX.25 FCS
 */
uint16_t hdlc_deframer_impl::crc_ccitt(const uint8_t* data, size_t len)
{
    static constexpr uint16_t POLY = 0x8408; // reflected 0x1021
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ POLY;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFF;
}

bool hdlc_deframer_impl::fcs_ok(const uint8_t* frame, size_t len)
{
    if (len <= 2)
        return false;
    uint16_t computed = crc_ccitt(frame, len - 2);
    return frame[len - 2] == (uint8_t)(computed & 0xFF) &&
           frame[len - 1] == (uint8_t)((computed >> 8) & 0xFF);
}

inline void hdlc_deframer_impl::push_bit(uint8_t bit)
{
    if (d_byte_count >= d_max_bytes)
        return; // overflow guard — frame too long

    // LSB-first shift register: shift right, set MSB conditionally.
    // After 8 pushes the first bit sits at position 0 (LSB).
    d_pktbuf[d_byte_count] >>= 1;
    if (bit)
        d_pktbuf[d_byte_count] |= 0x80;

    if (++d_bit_pos == 8) {
        d_bit_pos = 0;
        d_byte_count++;
    }
}

void hdlc_deframer_impl::process_frame()
{
    // Remove up to 7 flag/trailing bits that were pushed before detection.
    // A standard HDLC flag 01111110 contributes 7 bits to the buffer
    // (leading 0 + 6 ones; the trailing 0 triggers detection and is NOT pushed).
    int total_bits = d_byte_count * 8 + d_bit_pos;
    int remove = std::min(7, total_bits);
    total_bits -= remove;

    d_byte_count = total_bits / 8;
    d_bit_pos = total_bits % 8;

    // Handle partial byte (equivalent to Python extendleft zero-padding).
    // In the shift register, the d_bit_pos data bits occupy the UPPER
    // positions of d_pktbuf[d_byte_count]; the lower bits may contain
    // garbage from a previous frame — mask them to zero.
    if (d_bit_pos > 0) {
        d_pktbuf[d_byte_count] &= (uint8_t)(0xFF << (8 - d_bit_pos));
        d_byte_count++; // count the partial byte
        d_bit_pos = 0;
    }

    // Need at least 3 bytes (1 payload + 2 FCS) to be useful.
    if (d_byte_count > 2) {
        bool send = !d_check_fcs || fcs_ok(d_pktbuf.data(), d_byte_count);

        // Bit-flip CRC retry: if FCS fails, try flipping each bit in the
        // frame (payload + FCS) one at a time. For a typical 300-byte AX.25
        // frame, this is ~2400 CRC-16 checks — takes <100 µs in C++.
        // Recovers all frames with exactly 1 bit-error.
        if (!send && d_check_fcs && d_byte_count <= d_max_bytes) {
            const size_t nbytes = d_byte_count;
            for (size_t byte_idx = 0; byte_idx < nbytes && !send; byte_idx++) {
                for (int bit_idx = 0; bit_idx < 8 && !send; bit_idx++) {
                    d_pktbuf[byte_idx] ^= (1 << bit_idx);   // flip
                    if (fcs_ok(d_pktbuf.data(), nbytes)) {
                        send = true; // corrected frame — keep the fix
                    } else {
                        d_pktbuf[byte_idx] ^= (1 << bit_idx); // restore
                    }
                }
            }
        }

        if (send) {
            size_t payload_len = d_byte_count - 2; // trim FCS
            pmt::pmt_t pdu = pmt::cons(
                pmt::PMT_NIL,
                pmt::init_u8vector(payload_len, d_pktbuf.data()));
            message_port_pub(d_port, pdu);
        }
    }

    // Reset for next frame.
    d_byte_count = 0;
    d_bit_pos = 0;
}

/*
 * Constructor
 */
hdlc_deframer_impl::hdlc_deframer_impl(bool check_fcs, int max_length)
    : gr::sync_block("hdlc_deframer",
                     gr::io_signature::make(1, 1, sizeof(uint8_t)),
                     gr::io_signature::make(0, 0, 0)),
      d_check_fcs(check_fcs),
      d_max_bytes(max_length + 2 + 2), // payload + FCS + margin for flag bits
      d_pktbuf(max_length + 4, 0),
      d_byte_count(0),
      d_bit_pos(0),
      d_ones(0),
      d_port(pmt::intern("out"))
{
    message_port_register_out(d_port);
}

hdlc_deframer_impl::~hdlc_deframer_impl() {}

/*
 * Bit-level HDLC deframing.
 *
 * Algorithm (matches the original Python hdlc_deframer exactly):
 *
 *   for each input bit:
 *     if bit == 1:
 *       ones++; push_bit(1)
 *     else:  // bit == 0
 *       if ones == 5:
 *         bit-destuffing → discard this 0
 *       elif ones > 5:
 *         HDLC flag → process accumulated frame
 *       else:
 *         push_bit(0)
 *       ones = 0
 */
int hdlc_deframer_impl::work(int noutput_items,
                              gr_vector_const_void_star& input_items,
                              gr_vector_void_star& output_items)
{
    const uint8_t* in = (const uint8_t*)input_items[0];

    for (int i = 0; i < noutput_items; i++) {
        if (in[i]) {
            // Bit is 1
            d_ones++;
            push_bit(1);
        } else {
            // Bit is 0
            if (d_ones == 5) {
                // Bit-stuffing: discard this zero
            } else if (d_ones > 5) {
                // HDLC flag (6+ consecutive ones followed by 0)
                process_frame();
            } else {
                push_bit(0);
            }
            d_ones = 0;
        }
    }

    return noutput_items;
}

} /* namespace satellites */
} /* namespace gr */
