/* -*- c -*- */
/*
 * Copyright 2015-2019 Miklos Maroti
 * Copyright 2019 Daniel Estevez <daniel@destevez.net> (reentrant version)
 * Copyright 2026 F4TNK — AVX2 SIMD for LLR min-sum (LGPL)
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include "ra_decoder_gen.h"
#include "ra_lfsr.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* --- REPEAT ACCUMULATE GENERIC DECODER --- */
/* F4TNK: SIMD-optimized LLR min-sum inner loop.
 *
 * ra_llr_min(a, b) = sign(a*b) * min(|a|, |b|)
 *
 * Original used fabsf/copysignf — opaque library calls that prevent
 * GCC auto-vectorization of the inner bit loop.
 *
 * Bitwise rewrite: XOR for sign, AND for absolute, compare for min.
 * All operations have 1:1 SIMD equivalents (vpxord/vandps/vminps/vorps).
 *
 * With AVX2: explicit intrinsics process all 16 bits (RA_BITCOUNT)
 * in 2 × __m256 passes instead of 16 scalar iterations.
 * Skylake i7-6700: ~6 instructions per 8 elements vs ~10 per 1 element. */

#ifdef __AVX2__
#include <immintrin.h>
#endif

void ra_prepare_gen(struct ra_context* ctx, float* softbits)
{
    /* F4TNK: memset/memcpy for reliable SIMD — GCC LTO vectorizes these. */
    memset(ctx->ra_dataword_gen, 0,
           ctx->ra_data_length * RA_BITCOUNT * sizeof(float));
    memcpy(ctx->ra_codeword_gen, softbits,
           ctx->ra_code_length * RA_BITCOUNT * sizeof(float));
}

/* Bitwise LLR min-sum: auto-vectorizable scalar fallback.
 * sign(a*b) * min(|a|, |b|) via IEEE 754 bit manipulation. */
static inline float ra_llr_min(float a, float b)
{
    union { float f; uint32_t u; } ua, ub, ur;
    ua.f = a;
    ub.f = b;
    /* Sign of product = XOR of sign bits */
    uint32_t sign = (ua.u ^ ub.u) & 0x80000000u;
    /* Absolute values */
    ua.u &= 0x7FFFFFFFu;
    ub.u &= 0x7FFFFFFFu;
    /* IEEE 754: positive float order == unsigned integer order */
    ur.u = (ua.u < ub.u ? ua.u : ub.u) | sign;
    return ur.f;
}

/* ------------------------------------------------------------------
 * F4TNK: AVX2 bulk LLR min-sum for RA_BITCOUNT = 16 elements.
 *
 * Processes dst[0..15] = llr_min(dst[0..15], src[0..15]) in two
 * __m256 passes (8 elements each).
 *
 * On Skylake i7-6700 (8-wide AVX2):
 *   2 × (load + load + 3 bitwise + vminps + blend + store) = ~16 insns
 *   vs 16 × ~10 scalar insns = ~160 insns → ~10× reduction.
 * ------------------------------------------------------------------ */
#ifdef __AVX2__

static const uint32_t sign_mask_data[8]
    __attribute__((aligned(32))) = {
        0x80000000u, 0x80000000u, 0x80000000u, 0x80000000u,
        0x80000000u, 0x80000000u, 0x80000000u, 0x80000000u};
static const uint32_t abs_mask_data[8]
    __attribute__((aligned(32))) = {
        0x7FFFFFFFu, 0x7FFFFFFFu, 0x7FFFFFFFu, 0x7FFFFFFFu,
        0x7FFFFFFFu, 0x7FFFFFFFu, 0x7FFFFFFFu, 0x7FFFFFFFu};

/* dst[i] = llr_min(dst[i], src[i]) for i in 0..15 */
static inline void ra_llr_min_16(float* dst, const float* src)
{
    const __m256 smask = _mm256_load_ps((const float*)sign_mask_data);
    const __m256 amask = _mm256_load_ps((const float*)abs_mask_data);

    for (int k = 0; k < 16; k += 8) {
        __m256 a = _mm256_loadu_ps(dst + k);
        __m256 b = _mm256_loadu_ps(src + k);
        /* sign = sign(a) XOR sign(b) */
        __m256 sa = _mm256_and_ps(a, smask);
        __m256 sb = _mm256_and_ps(b, smask);
        __m256 s  = _mm256_xor_ps(sa, sb);
        /* min(|a|, |b|) */
        __m256 aa = _mm256_and_ps(a, amask);
        __m256 ab = _mm256_and_ps(b, amask);
        __m256 mn = _mm256_min_ps(aa, ab);
        /* copysign(min, sign_product) */
        _mm256_storeu_ps(dst + k, _mm256_or_ps(mn, s));
    }
}

/* dst[i] += src[i] for i in 0..15 */
static inline void ra_add_16(float* dst, const float* src)
{
    for (int k = 0; k < 16; k += 8) {
        __m256 a = _mm256_loadu_ps(dst + k);
        __m256 b = _mm256_loadu_ps(src + k);
        _mm256_storeu_ps(dst + k, _mm256_add_ps(a, b));
    }
}

/* dst[i] = llr_min(dst[i], src[i]);
 * out[i] = result before min update (forward save) */
static inline void ra_llr_min_16_save(float* dst, const float* src,
                                      float* out)
{
    const __m256 smask = _mm256_load_ps((const float*)sign_mask_data);
    const __m256 amask = _mm256_load_ps((const float*)abs_mask_data);

    for (int k = 0; k < 16; k += 8) {
        __m256 a = _mm256_loadu_ps(dst + k);
        __m256 b = _mm256_loadu_ps(src + k);
        _mm256_storeu_ps(out + k, a); /* save forward */
        __m256 sa = _mm256_and_ps(a, smask);
        __m256 sb = _mm256_and_ps(b, smask);
        __m256 s  = _mm256_xor_ps(sa, sb);
        __m256 aa = _mm256_and_ps(a, amask);
        __m256 ab = _mm256_and_ps(b, amask);
        __m256 mn = _mm256_min_ps(aa, ab);
        _mm256_storeu_ps(dst + k, _mm256_or_ps(mn, s));
    }
}

/* Rotate accu[0..15] left by 1: [a0,a1,...,a15] → [a1,a2,...,a15,a0] */
static inline void ra_rotate_left_16(float* accu)
{
    float saved = accu[0];
    __m256 lo = _mm256_loadu_ps(accu);
    __m256 hi = _mm256_loadu_ps(accu + 8);
    /* Cross-lane left shift: permutevar8x32 + blend boundary elements */
    const __m256i perm = _mm256_setr_epi32(1, 2, 3, 4, 5, 6, 7, 0);
    __m256 lo_s = _mm256_permutevar8x32_ps(lo, perm); /* [a1..a7, a0] */
    __m256 hi_s = _mm256_permutevar8x32_ps(hi, perm); /* [a9..a15, a8] */
    /* lo_s[7] should be hi[0]=a8, hi_s[7] should be saved=a0 */
    __m128 hi0 = _mm256_castps256_ps128(hi);           /* [a8, ...] */
    __m256 hi0_bc = _mm256_broadcastss_ps(hi0);        /* [a8, a8, ...] */
    lo_s = _mm256_blend_ps(lo_s, hi0_bc, 0x80);        /* lo_s[7] = a8 */
    __m256 a0_bc = _mm256_set1_ps(saved);
    hi_s = _mm256_blend_ps(hi_s, a0_bc, 0x80);         /* hi_s[7] = a0 */
    _mm256_storeu_ps(accu, lo_s);
    _mm256_storeu_ps(accu + 8, hi_s);
}

/* Rotate accu[0..15] right by 1: [a0,a1,...,a15] → [a15,a0,...,a14] */
static inline void ra_rotate_right_16(float* accu)
{
    float saved = accu[15];
    __m256 lo = _mm256_loadu_ps(accu);
    __m256 hi = _mm256_loadu_ps(accu + 8);
    const __m256i perm = _mm256_setr_epi32(7, 0, 1, 2, 3, 4, 5, 6);
    __m256 lo_s = _mm256_permutevar8x32_ps(lo, perm); /* [a7, a0..a6] */
    __m256 hi_s = _mm256_permutevar8x32_ps(hi, perm); /* [a15, a8..a14] */
    /* lo_s[0] should be saved=a15, hi_s[0] should be lo[7]=a7 */
    __m256 a15_bc = _mm256_set1_ps(saved);
    lo_s = _mm256_blend_ps(lo_s, a15_bc, 0x01);
    /* Extract lo[7]: upper 128 bits, element 3 */
    __m128 lo_hi128 = _mm256_extractf128_ps(lo, 1);
    __m128 a7_128 = _mm_permute_ps(lo_hi128, 0xFF); /* broadcast [3] */
    __m256 a7_bc = _mm256_broadcastss_ps(a7_128);
    hi_s = _mm256_blend_ps(hi_s, a7_bc, 0x01);
    _mm256_storeu_ps(accu, lo_s);
    _mm256_storeu_ps(accu + 8, hi_s);
}

#else /* scalar fallback */

static inline void ra_llr_min_16(float* dst, const float* src)
{
    for (int i = 0; i < 16; i++)
        dst[i] = ra_llr_min(dst[i], src[i]);
}

static inline void ra_add_16(float* dst, const float* src)
{
    for (int i = 0; i < 16; i++)
        dst[i] += src[i];
}

static inline void ra_llr_min_16_save(float* dst, const float* src,
                                      float* out)
{
    for (int i = 0; i < 16; i++) {
        out[i] = dst[i];
        dst[i] = ra_llr_min(dst[i], src[i]);
    }
}

static inline void ra_rotate_left_16(float* accu)
{
    float saved = accu[0];
    for (int i = 0; i < 15; i++)
        accu[i] = accu[i + 1];
    accu[15] = saved;
}

static inline void ra_rotate_right_16(float* accu)
{
    float saved = accu[15];
    for (int i = 15; i >= 1; i--)
        accu[i] = accu[i - 1];
    accu[0] = saved;
}

#endif /* __AVX2__ */

/* F4TNK: SIMD-optimized ra_improve_gen.
 * Inner bit loops replaced with 16-wide SIMD helpers (AVX2: 2 × __m256).
 * Forward and backward passes process all RA_BITCOUNT=16 bits in parallel.
 * The LFSR-dependent outer loop remains sequential (data-dependent index). */
void ra_improve_gen(struct ra_context* ctx, float* codeword, int puncture, bool half)
{
    int index, bit, pos = 0;
    float accu[RA_BITCOUNT] __attribute__((aligned(32)));

    assert(ctx->ra_data_length > 0);

    for (bit = 0; bit < RA_BITCOUNT; bit++)
        accu[bit] = FLT_MAX;

    /* --- Forward pass --- */
    for (index = 0; index < ctx->ra_data_length; index++) {
        pos = ra_lfsr_next(ctx);

        /* Save accu to forward, then accu = llr_min(accu, dataword[pos]) */
        ra_llr_min_16_save(accu,
                           &ctx->ra_dataword_gen[pos * RA_BITCOUNT],
                           &ctx->ra_forward_gen[index * RA_BITCOUNT]);

        if ((index + 1) % puncture == 0) {
            ra_add_16(accu, codeword);
            codeword += RA_BITCOUNT;
        }

        ra_rotate_left_16(accu);
    }

    if (ctx->ra_data_length % puncture != 0) {
        for (bit = 0; bit < RA_BITCOUNT; bit++) {
            float data = codeword[(bit + 1) % RA_BITCOUNT];
            accu[bit] = accu[bit] + data + data;
        }
    }

    /* --- Backward pass --- */
    for (index = ctx->ra_data_length - 1; index >= 0; index--) {
        ra_rotate_right_16(accu);

        if ((index + 1) % puncture == 0) {
            codeword -= RA_BITCOUNT;
            ra_add_16(accu, codeword);
        }

        /* Compute: left = llr_min(forward[index], accu)
         *          accu = llr_min(accu, dataword[pos])
         *          dataword[pos] = left + dataword[pos] * (half ? 0.5 : 1.0)
         *
         * F4TNK: SIMD backward pass — same 16-wide treatment as forward.
         * The 'half' flag is loop-invariant, so we use a scale vector
         * (0.5 or 1.0) to eliminate the per-element branch entirely. */
#ifdef __AVX2__
        {
            float* fwd = &ctx->ra_forward_gen[index * RA_BITCOUNT];
            float* dw  = &ctx->ra_dataword_gen[pos * RA_BITCOUNT];
            const __m256 smask = _mm256_load_ps((const float*)sign_mask_data);
            const __m256 amask = _mm256_load_ps((const float*)abs_mask_data);
            const __m256 scale = _mm256_set1_ps(half ? 0.5f : 1.0f);

            for (int k = 0; k < 16; k += 8) {
                __m256 fwd_v = _mm256_loadu_ps(fwd + k);
                __m256 acc_v = _mm256_loadu_ps(accu + k);
                __m256 dw_v  = _mm256_loadu_ps(dw + k);

                /* left = llr_min(fwd, accu) */
                __m256 s = _mm256_xor_ps(
                    _mm256_and_ps(fwd_v, smask),
                    _mm256_and_ps(acc_v, smask));
                __m256 left_v = _mm256_or_ps(
                    _mm256_min_ps(
                        _mm256_and_ps(fwd_v, amask),
                        _mm256_and_ps(acc_v, amask)),
                    s);

                /* accu = llr_min(accu, dw) */
                s = _mm256_xor_ps(
                    _mm256_and_ps(acc_v, smask),
                    _mm256_and_ps(dw_v, smask));
                _mm256_storeu_ps(accu + k,
                    _mm256_or_ps(
                        _mm256_min_ps(
                            _mm256_and_ps(acc_v, amask),
                            _mm256_and_ps(dw_v, amask)),
                        s));

                /* dataword[pos] = left + dw * scale */
                _mm256_storeu_ps(dw + k,
                    _mm256_add_ps(left_v, _mm256_mul_ps(dw_v, scale)));
            }
        }
#else
        for (bit = 0; bit < RA_BITCOUNT; bit++) {
            float left = ctx->ra_forward_gen[index * RA_BITCOUNT + bit];
            left = ra_llr_min(left, accu[bit]);

            float data = ctx->ra_dataword_gen[pos * RA_BITCOUNT + bit];
            accu[bit] = ra_llr_min(accu[bit], data);

            if (half)
                data *= 0.5f;

            left += data;
            ctx->ra_dataword_gen[pos * RA_BITCOUNT + bit] = left;
        }
#endif

        pos = ra_lfsr_prev(ctx);
    }
}

void ra_decide_gen(struct ra_context* ctx, ra_word_t* packet)
{
    int index, bit;
    ra_word_t word;
    float data;

    for (index = 0; index < ctx->ra_data_length; index++) {
        word = 0;

        for (bit = 0; bit < RA_BITCOUNT; bit++) {
            data = ctx->ra_dataword_gen[index * RA_BITCOUNT + bit];
            word |= (data < 0.0f) << bit;
        }

        packet[index] = word;
    }
}

void ra_decoder_gen(struct ra_context* ctx,
                    float* softbits,
                    ra_word_t* packet,
                    int passes)
{
    int count, seqno;
    float* codeword;

    ra_prepare_gen(ctx, softbits);

    for (count = 0; count < passes; count++) {
        codeword = ctx->ra_codeword_gen;

        for (seqno = 0; seqno < 4; seqno++) {
            ra_lfsr_init(ctx, seqno);
            ra_improve_gen(ctx, codeword, seqno == 0 ? 1 : RA_PUNCTURE_RATE, count > 0);
            codeword +=
                (seqno == 0 ? ctx->ra_data_length : ctx->ra_chck_length) * RA_BITCOUNT;
        }

        assert(ctx->ra_codeword_gen + ctx->ra_code_length * RA_BITCOUNT == codeword);
    }

    ra_decide_gen(ctx, packet);
}
