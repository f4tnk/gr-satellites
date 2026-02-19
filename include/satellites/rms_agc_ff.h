/* -*- c++ -*- */
/*
 * Copyright 2026 Daniel Estevez <daniel@destevez.net>.
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * F4TNK optimization: single-pass float RMS AGC C++ block replacing the
 * 4-block hier_block2 chain (rms_ff → multiply_const_ff → add_const_ff →
 * divide_ff). Uses VOLK for the squared-magnitude and processes the signal
 * in one pass with no inter-block buffering.
 */

#ifndef INCLUDED_SATELLITES_RMS_AGC_FF_H
#define INCLUDED_SATELLITES_RMS_AGC_FF_H

#include <gnuradio/sync_block.h>
#include <satellites/api.h>

namespace gr {
namespace satellites {

/*!
 * \brief Single-pass float RMS AGC
 * \ingroup satellites
 *
 * \details
 * Normalizes a float stream so that its RMS amplitude equals \p reference.
 * The running RMS is tracked with an exponential moving average of \p alpha.
 *
 * This block replaces the 4-block hier chain:
 *   rms_ff → multiply_const_ff → add_const_ff(1e-19) → divide_ff
 * with a single C++ work() call, eliminating 3 inter-block buffer copies.
 */
class SATELLITES_API rms_agc_ff : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<rms_agc_ff> sptr;

    /*!
     * \brief Build a single-pass float RMS AGC block.
     *
     * \param alpha     EMA coefficient (0 < alpha < 1).
     * \param reference Target RMS output level (default 0.5).
     */
    static sptr make(float alpha = 0.01f, float reference = 0.5f);

    virtual void set_alpha(float alpha) = 0;
    virtual void set_reference(float reference) = 0;
};

} // namespace satellites
} // namespace gr

#endif /* INCLUDED_SATELLITES_RMS_AGC_FF_H */
