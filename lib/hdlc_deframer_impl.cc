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
 * CRC-16-CCITT (reflected / LSB-first) — TABLE-BASED
 *
 * Equivalent to:
 *   Python:  crc(16, 0x1021, 0xFFFF, 0xFFFF, True, True).compute(data)
 *   aka CRC-16/X-25, CRC-16/ISO-HDLC, AX.25 FCS
 *
 * F4TNK: replaced bit-by-bit with 256-entry lookup table (8× fewer ops).
 */
static const uint16_t crc_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
};

uint16_t hdlc_deframer_impl::crc_ccitt(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc_table[(crc ^ data[i]) & 0xFF];
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

        /*
         * F4TNK: Syndrome-based 1-bit error correction — O(n) algorithm.
         *
         * Replaces the previous O(n×8) brute-force (full CRC per bit-flip).
         * Algorithm:
         *   1. Compute CRC over entire frame (payload + FCS).
         *   2. For a correct frame, CRC residual = 0x0F47 (post xorout=0xFFFF).
         *      Syndrome = crc_result ^ 0x0F47.  If 0, frame is already valid.
         *   3. Backward scan: the syndrome of the last bit (byte n-1, bit 7)
         *      is 0x8408 (reflected poly). For each earlier bit, apply a
         *      forward LFSR step.  If syndrome matches target, flip that bit.
         *   4. Verify with fcs_ok() to guard against syndrome collision
         *      (probability ~1/65536).
         */
        if (!send && d_check_fcs && d_byte_count <= d_max_bytes) {
            const size_t nbytes = d_byte_count;

            // Compute CRC-16/X.25 over entire frame including FCS
            uint16_t c = 0xFFFF;
            for (size_t i = 0; i < nbytes; i++) {
                c = (c >> 8) ^ crc_table[(c ^ d_pktbuf[i]) & 0xFF];
            }
            c ^= 0xFFFF;

            // CRC with xorout applied yields 0x0F47 for a valid frame
            const uint16_t target = c ^ 0x0F47;
            if (target == 0) {
                // Already valid (shouldn't happen since fcs_ok failed above,
                // but guard anyway)
                send = true;
            } else {
                // Backward syndrome scan
                uint16_t s = 0x8408;
                for (int i = (int)nbytes - 1; i >= 0 && !send; i--) {
                    for (int b = 7; b >= 0 && !send; b--) {
                        if (s == target) {
                            d_pktbuf[i] ^= (uint8_t)(1u << b);
                            if (fcs_ok(d_pktbuf.data(), nbytes)) {
                                send = true;
                            } else {
                                d_pktbuf[i] ^= (uint8_t)(1u << b);
                            }
                        }
                        // Forward LFSR step: syndrome for one-earlier bit position
                        // (earlier bit → more remaining shifts → one more forward step)
                        s = (s & 1) ? (uint16_t)(((s >> 1) ^ 0x8408u) & 0xFFFFu)
                                    : (uint16_t)((s >> 1) & 0xFFFFu);
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
