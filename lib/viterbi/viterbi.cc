// Implementation of ViterbiCodec.
//
// Author: Min Xu <xukmin@gmail.com>
// Date: 01/30/2015
//
// Optimized by F4TNK: std::string elimination — packed uint32_t parity bits
// with __builtin_popcount for Hamming distance.  Zero heap allocations in the
// decode hot loop (~262K string allocs eliminated for a K=7 / 1024-symbol frame).

#include "viterbi.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

std::ostream& operator<<(std::ostream& os, const ViterbiCodec& codec)
{
    os << "ViterbiCodec(" << codec.constraint() << ", {";
    const std::vector<int>& polynomials = codec.polynomials();
    assert(!polynomials.empty());
    os << polynomials.front();
    for (int i = 1; i < (int)polynomials.size(); i++) {
        os << ", " << polynomials[i];
    }
    return os << "})";
}

int ReverseBits(int num_bits, int input)
{
    assert(input < (1 << num_bits));
    int output = 0;
    while (num_bits-- > 0) {
        output = (output << 1) + (input & 1);
        input >>= 1;
    }
    return output;
}

ViterbiCodec::ViterbiCodec(int constraint, const std::vector<int>& polynomials)
    : constraint_(constraint), polynomials_(polynomials),
      trellis_capacity_(0)
{
    assert(!polynomials_.empty());
    assert(num_parity_bits() <= 32);  // must fit in uint32_t
    for (int i = 0; i < (int)polynomials_.size(); i++) {
        assert(polynomials_[i] > 0);
        assert(polynomials_[i] < (1 << constraint_));
    }
    InitializeOutputs();

    // F4TNK: Pre-allocate reusable path metric buffers
    const int num_states = 1 << (constraint_ - 1);
    path_metrics_.resize(num_states);
    new_path_metrics_.resize(num_states);
}

int ViterbiCodec::num_parity_bits() const { return polynomials_.size(); }

int ViterbiCodec::NextState(int current_state, int input) const
{
    return (current_state >> 1) | (input << (constraint_ - 2));
}

uint32_t ViterbiCodec::Output(int current_state, int input) const
{
    return outputs_[current_state | (input << (constraint_ - 1))];
}

std::vector<uint8_t> ViterbiCodec::Encode(const uint8_t* bits, size_t len) const
{
    const int npb = num_parity_bits();
    std::vector<uint8_t> encoded;
    encoded.reserve((len + constraint_ - 1) * npb);
    int state = 0;

    // Encode message bits.
    for (size_t i = 0; i < len; i++) {
        assert(bits[i] <= 1);
        uint32_t out = Output(state, bits[i]);
        for (int j = 0; j < npb; j++) {
            encoded.push_back(static_cast<uint8_t>((out >> j) & 1));
        }
        state = NextState(state, bits[i]);
    }

    // Encode (constraint_ - 1) flushing bits.
    for (int i = 0; i < constraint_ - 1; i++) {
        uint32_t out = Output(state, 0);
        for (int j = 0; j < npb; j++) {
            encoded.push_back(static_cast<uint8_t>((out >> j) & 1));
        }
        state = NextState(state, 0);
    }

    return encoded;
}

void ViterbiCodec::InitializeOutputs()
{
    outputs_.resize(1 << constraint_, 0);
    for (int i = 0; i < (int)outputs_.size(); i++) {
        uint32_t packed = 0;
        for (int j = 0; j < num_parity_bits(); j++) {
            // Reverse polynomial bits to make the convolution code simpler.
            int polynomial = ReverseBits(constraint_, polynomials_[j]);
            int input = i;
            int output = 0;
            for (int k = 0; k < constraint_; k++) {
                output ^= (input & 1) & (polynomial & 1);
                polynomial >>= 1;
                input >>= 1;
            }
            packed |= (static_cast<uint32_t>(output) << j);
        }
        outputs_[i] = packed;
    }
}

int ViterbiCodec::BranchMetric(uint32_t bits,
                               int source_state,
                               int target_state) const
{
    assert((target_state & ((1 << (constraint_ - 2)) - 1)) == source_state >> 1);
    uint32_t output = Output(source_state, target_state >> (constraint_ - 2));
    return __builtin_popcount(bits ^ output);
}

std::pair<int, int> ViterbiCodec::PathMetric(uint32_t bits,
                                             const std::vector<int>& prev_path_metrics,
                                             int state) const
{
    int s = (state & ((1 << (constraint_ - 2)) - 1)) << 1;
    int source_state1 = s | 0;
    int source_state2 = s | 1;

    int pm1 = prev_path_metrics[source_state1];
    if (pm1 < std::numeric_limits<int>::max()) {
        pm1 += BranchMetric(bits, source_state1, state);
    }
    int pm2 = prev_path_metrics[source_state2];
    if (pm2 < std::numeric_limits<int>::max()) {
        pm2 += BranchMetric(bits, source_state2, state);
    }

    if (pm1 <= pm2) {
        return std::make_pair(pm1, source_state1);
    } else {
        return std::make_pair(pm2, source_state2);
    }
}

/*
 * F4TNK: UpdatePathMetrics with pre-allocated flat trellis.
 * Eliminates 2 heap allocations per step (new_path_metrics + new_trellis_column).
 * Uses member buffers path_metrics_ and new_path_metrics_ (swap instead of alloc).
 * Trellis stored in flat layout: trellis_flat_[step * num_states + state].
 */
void ViterbiCodec::UpdatePathMetrics(uint32_t bits, int step)
{
    const int num_states = (int)path_metrics_.size();
    int* trellis_col = &trellis_flat_[step * num_states];

    for (int i = 0; i < num_states; i++) {
        std::pair<int, int> p = PathMetric(bits, path_metrics_, i);
        new_path_metrics_[i] = p.first;
        trellis_col[i] = p.second;
    }

    std::swap(path_metrics_, new_path_metrics_);
}

std::vector<uint8_t> ViterbiCodec::Decode(const uint8_t* bits, size_t len)
{
    const int npb = num_parity_bits();
    const int num_states = 1 << (constraint_ - 1);
    const size_t num_steps = (len + npb - 1) / npb;

    // F4TNK: Ensure flat trellis is large enough (amortized — only reallocates
    // if this frame is larger than any previous frame).
    if (num_steps > trellis_capacity_) {
        trellis_flat_.resize(num_steps * num_states);
        trellis_capacity_ = num_steps;
    }

    // Initialize path metrics
    std::fill(path_metrics_.begin(), path_metrics_.end(),
              std::numeric_limits<int>::max());
    path_metrics_[0] = 0;

    int step = 0;
    for (size_t i = 0; i < len; i += npb) {
        // Pack num_parity_bits() input bits into a uint32_t.
        uint32_t packed = 0;
        int available = std::min(npb, static_cast<int>(len - i));
        for (int j = 0; j < available; j++) {
            packed |= (static_cast<uint32_t>(bits[i + j]) << j);
        }
        UpdatePathMetrics(packed, step);
        step++;
    }

    // Traceback using flat trellis.
    std::vector<uint8_t> decoded(step);
    int state =
        std::min_element(path_metrics_.begin(), path_metrics_.end()) - path_metrics_.begin();
    for (int i = step - 1; i >= 0; i--) {
        decoded[i] = static_cast<uint8_t>((state >> (constraint_ - 2)) ? 1 : 0);
        state = trellis_flat_[i * num_states + state];
    }

    // Remove (constraint_ - 1) flushing bits.
    if ((int)decoded.size() > constraint_ - 1) {
        decoded.resize(decoded.size() - constraint_ + 1);
    }

    return decoded;
}
