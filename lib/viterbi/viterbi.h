// Viterbi Codec.
//
// Author: Min Xu <xukmin@gmail.com>
// Date: 01/30/2015
//
// Optimized by F4TNK: std::string elimination — all internal paths use packed
// uint32_t parity bits with __builtin_popcount for Hamming distance.
// Public API changed from std::string to uint8_t*/vector<uint8_t>.

#ifndef VITERBI_H_
#define VITERBI_H_

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <utility>
#include <vector>

// This class implements both a Viterbi Decoder and a Convolutional Encoder.
// All data paths use packed integer parity bits — zero heap allocations in the
// hot loop.
class ViterbiCodec
{
public:
    // Polynomial Descriptor: lsb-current convention.
    // See http://www.spiral.net/software/viterbi.html
    ViterbiCodec(int constraint, const std::vector<int>& polynomials);

    // Encode / Decode operate on arrays of uint8_t where each element is 0 or 1.
    std::vector<uint8_t> Encode(const uint8_t* bits, size_t len) const;
    std::vector<uint8_t> Decode(const uint8_t* bits, size_t len) const;

    int constraint() const { return constraint_; }

    const std::vector<int>& polynomials() const { return polynomials_; }

private:
    typedef std::vector<std::vector<int>> Trellis;

    int num_parity_bits() const;

    void InitializeOutputs();

    int NextState(int current_state, int input) const;

    // Returns packed parity bits as uint32_t (bit j = polynomial j output).
    uint32_t Output(int current_state, int input) const;

    // Hamming distance via popcount on XOR of packed parity bits.
    int BranchMetric(uint32_t bits, int source_state, int target_state) const;

    std::pair<int, int> PathMetric(uint32_t bits,
                                   const std::vector<int>& prev_path_metrics,
                                   int state) const;

    void UpdatePathMetrics(uint32_t bits,
                           std::vector<int>* path_metrics,
                           Trellis* trellis) const;

    const int constraint_;
    const std::vector<int> polynomials_;

    // Output table: packed parity bits as uint32_t.
    // Index = current_state | (input << (constraint_ - 1)).
    // Bit j of the value = output of polynomial j.
    std::vector<uint32_t> outputs_;
};

std::ostream& operator<<(std::ostream& os, const ViterbiCodec& codec);

int ReverseBits(int num_bits, int input);

#endif // VITERBI_H_
