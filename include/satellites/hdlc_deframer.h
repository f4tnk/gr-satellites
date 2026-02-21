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

#ifndef INCLUDED_SATELLITES_HDLC_DEFRAMER_H
#define INCLUDED_SATELLITES_HDLC_DEFRAMER_H

#include <gnuradio/sync_block.h>
#include <satellites/api.h>

namespace gr {
namespace satellites {

/*!
 * \brief HDLC deframer (AX.25 / HDLC bit-level deframing with CRC-16-CCITT)
 * \ingroup satellites
 *
 * Processes a stream of unpacked bits (uint8, 0 or 1), detects HDLC flags
 * (0x7E = 01111110), performs bit-destuffing, packs into bytes (LSB-first),
 * optionally checks CRC-16-CCITT (FCS), and publishes frames as PDUs on
 * the "out" message port.
 *
 * This is a C++ port of the original Python hdlc_deframer for performance.
 */
class SATELLITES_API hdlc_deframer : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<hdlc_deframer> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of satellites::hdlc_deframer.
     *
     * \param check_fcs  If true, verify CRC-16-CCITT before publishing.
     * \param max_length Maximum frame length in bytes (excluding FCS).
     */
    static sptr make(bool check_fcs, int max_length);
};

} // namespace satellites
} // namespace gr

#endif /* INCLUDED_SATELLITES_HDLC_DEFRAMER_H */
