/* -*- c++ -*- */
/*
 * Copyright 2026 Daniel Estevez <daniel@destevez.net>.
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * F4TNK optimization: single-pass RMS AGC C++ block replacing the 5-block
 * hier_block2 chain (rms_cf → multiply_const → add_const →
 * float_to_complex → divide_cc).  Uses VOLK for |z|² and processes
 * the signal in one pass with no inter-block buffering.
 */

#ifndef INCLUDED_SATELLITES_RMS_AGC_CC_H
#define INCLUDED_SATELLITES_RMS_AGC_CC_H

#include <gnuradio/sync_block.h>
#include <satellites/api.h>

namespace gr {
namespace satellites {

/*!
 * \brief Single-pass complex RMS AGC
 * \ingroup satellites
 *
 * \details
 * Normalizes a complex IQ stream so that its RMS amplitude equals
 * \p reference.  The running RMS is tracked with an exponential
 * moving average of \p alpha.
 *
 * This block replaces the 5-block hier chain:
 *   rms_cf → multiply_const_ff → add_const_ff → float_to_complex → divide_cc
 * with a single C++ work() call that uses VOLK for the magnitude-squared
 * computation, eliminating 4 inter-block buffer copies.
 */
class SATELLITES_API rms_agc_cc : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<rms_agc_cc> sptr;

    /*!
     * \brief Build a single-pass RMS AGC block.
     *
     * \param alpha   EMA coefficient for the squared RMS estimate (0 < alpha < 1).
     *                Smaller values give slower, smoother tracking.
     * \param reference  Target RMS output level (default 0.5).
     */
    static sptr make(float alpha = 0.01f, float reference = 0.5f);

    virtual void set_alpha(float alpha) = 0;
    virtual void set_reference(float reference) = 0;
};

} // namespace satellites
} // namespace gr

#endif /* INCLUDED_SATELLITES_RMS_AGC_CC_H */
