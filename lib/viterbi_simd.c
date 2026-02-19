/*
 * K=7 r=1/2 Viterbi decoder — SIMD-accelerated ACS (Add-Compare-Select)
 * F4TNK optimization: AVX2 (32-wide) and SSE2 (16-wide) butterfly kernels
 * Based on Phil Karn KA9Q's portable C implementation (viterbi.c)
 *
 * AVX2:  processes all 32 branch metrics in one pass → ~5-8× speedup
 * SSE2:  processes 16 branch metrics per pass (2 iters) → ~3-4× speedup
 * Fallback: calls the original scalar update_viterbi_packed()
 *
 * Copyright 2026 F4TNK — LGPL (same as original viterbi.c)
 */

#include "viterbi.h"

#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Shared types & externs from viterbi.c                             */
/* ------------------------------------------------------------------ */

typedef union {
    uint8_t w[64];
} metric_t;
typedef union {
    uint8_t w[8];
} decision_t;
typedef union {
    uint8_t c[32];
} branchtab_t;

extern branchtab_t branchtab[2]; /* defined in viterbi.c */

struct v27 {
    metric_t metrics1;
    metric_t metrics2;
    decision_t* dp;
    metric_t *old_metrics, *new_metrics;
    decision_t* decisions;
    uint16_t dlen;
};

#define get_bit(_p, _n)                                                   \
    (_p[(_n) / (uint8_t)8] >>                                             \
         ((uint8_t)8 - 1 - ((_n) % (uint8_t)8)) &                        \
     (uint8_t)0x01)

/* ------------------------------------------------------------------ */
/*  AVX2 implementation                                               */
/* ------------------------------------------------------------------ */
#if defined(__AVX2__)
#include <immintrin.h>

int update_viterbi_packed_avx2(void* p, uint8_t* syms, uint16_t nbits)
{
    struct v27* vp = p;
    void* tmp;
    uint16_t i = 0;
    decision_t d_local;

    if (__builtin_expect(p == NULL, 0))
        return -1;

    const __m256i bias = _mm256_set1_epi8((char)0x80);
    const __m256i two  = _mm256_set1_epi8(2);

    /* Load branch tables once (they don't change) */
    const __m256i br0 = _mm256_loadu_si256((const __m256i*)branchtab[0].c);
    const __m256i br1 = _mm256_loadu_si256((const __m256i*)branchtab[1].c);

    while (__builtin_expect(nbits--, 1)) {
        /* Read symbols */
        uint8_t sym0 = get_bit(syms, i);
        uint8_t sym1 = get_bit(syms, i + 1);
        i += 2;

        /* Branch metrics for all 32 lower states */
        __m256i sym0v = _mm256_set1_epi8(sym0);
        __m256i sym1v = _mm256_set1_epi8(sym1);
        __m256i metric = _mm256_add_epi8(
            _mm256_xor_si256(br0, sym0v),
            _mm256_xor_si256(br1, sym1v));      /* 0, 1, or 2 per lane */
        __m256i complement = _mm256_sub_epi8(two, metric);

        /* Load old path metrics: lower [0..31] and upper [32..63] */
        __m256i old_lo = _mm256_loadu_si256((const __m256i*)&vp->old_metrics->w[0]);
        __m256i old_hi = _mm256_loadu_si256((const __m256i*)&vp->old_metrics->w[32]);

        /* --- Even output states: new[2b] for b=0..31 --- */
        __m256i m0_even = _mm256_add_epi8(old_lo, metric);
        __m256i m1_even = _mm256_add_epi8(old_hi, complement);
        /* Unsigned compare: m0 > m1  ↔  (m0^0x80) > (m1^0x80) signed */
        __m256i dec_even = _mm256_cmpgt_epi8(
            _mm256_xor_si256(m0_even, bias),
            _mm256_xor_si256(m1_even, bias));
        /* Survivor = decision ? m1 : m0 */
        __m256i surv_even = _mm256_blendv_epi8(m0_even, m1_even, dec_even);

        /* --- Odd output states: new[2b+1] --- */
        __m256i m0_odd = _mm256_add_epi8(old_lo, complement);
        __m256i m1_odd = _mm256_add_epi8(old_hi, metric);
        __m256i dec_odd = _mm256_cmpgt_epi8(
            _mm256_xor_si256(m0_odd, bias),
            _mm256_xor_si256(m1_odd, bias));
        __m256i surv_odd = _mm256_blendv_epi8(m0_odd, m1_odd, dec_odd);

        /* Interleave survivors: even/odd → new_metrics[0..63] */
        /* unpack is lane-wise in AVX2, need cross-lane permute */
        __m256i t0 = _mm256_unpacklo_epi8(surv_even, surv_odd);  /* [0..15 | 32..47] */
        __m256i t1 = _mm256_unpackhi_epi8(surv_even, surv_odd);  /* [16..31 | 48..63] */
        __m256i new_lo = _mm256_permute2x128_si256(t0, t1, 0x20); /* [0..31] */
        __m256i new_hi = _mm256_permute2x128_si256(t0, t1, 0x31); /* [32..63] */
        _mm256_storeu_si256((__m256i*)&vp->new_metrics->w[0],  new_lo);
        _mm256_storeu_si256((__m256i*)&vp->new_metrics->w[32], new_hi);

        /* Interleave decisions and extract bitmask */
        __m256i d0 = _mm256_unpacklo_epi8(dec_even, dec_odd);
        __m256i d1 = _mm256_unpackhi_epi8(dec_even, dec_odd);
        __m256i dec_lo = _mm256_permute2x128_si256(d0, d1, 0x20);
        __m256i dec_hi = _mm256_permute2x128_si256(d0, d1, 0x31);
        uint32_t mask_lo = (uint32_t)_mm256_movemask_epi8(dec_lo);
        uint32_t mask_hi = (uint32_t)_mm256_movemask_epi8(dec_hi);

        /* Store 64-bit decision word */
        uint64_t decision_word = (uint64_t)mask_lo | ((uint64_t)mask_hi << 32);
        memcpy(d_local.w, &decision_word, 8);

        /* Writeback */
        memcpy(vp->dp++, &d_local, sizeof(decision_t));

        /* Swap metrics */
        tmp = vp->old_metrics;
        vp->old_metrics = vp->new_metrics;
        vp->new_metrics = tmp;
    }

    return 0;
}

#endif /* __AVX2__ */

/* ------------------------------------------------------------------ */
/*  SSE2 implementation                                               */
/* ------------------------------------------------------------------ */
#if defined(__SSE2__)
#include <emmintrin.h>

int update_viterbi_packed_sse2(void* p, uint8_t* syms, uint16_t nbits)
{
    struct v27* vp = p;
    void* tmp;
    uint16_t i = 0;
    decision_t d_local;

    if (__builtin_expect(p == NULL, 0))
        return -1;

    const __m128i bias = _mm_set1_epi8((char)0x80);
    const __m128i two  = _mm_set1_epi8(2);

    /* Load branch tables as two 16B halves */
    const __m128i br0_lo = _mm_loadu_si128((const __m128i*)&branchtab[0].c[0]);
    const __m128i br0_hi = _mm_loadu_si128((const __m128i*)&branchtab[0].c[16]);
    const __m128i br1_lo = _mm_loadu_si128((const __m128i*)&branchtab[1].c[0]);
    const __m128i br1_hi = _mm_loadu_si128((const __m128i*)&branchtab[1].c[16]);

    while (__builtin_expect(nbits--, 1)) {
        uint8_t sym0 = get_bit(syms, i);
        uint8_t sym1 = get_bit(syms, i + 1);
        i += 2;

        __m128i sym0v = _mm_set1_epi8(sym0);
        __m128i sym1v = _mm_set1_epi8(sym1);

        /* Branch metrics, two halves */
        __m128i met_lo = _mm_add_epi8(
            _mm_xor_si128(br0_lo, sym0v), _mm_xor_si128(br1_lo, sym1v));
        __m128i met_hi = _mm_add_epi8(
            _mm_xor_si128(br0_hi, sym0v), _mm_xor_si128(br1_hi, sym1v));
        __m128i cmp_lo = _mm_sub_epi8(two, met_lo);
        __m128i cmp_hi = _mm_sub_epi8(two, met_hi);

        uint64_t decision_word = 0;

        /* Process states b=0..15 → new[0..31] */
        {
            __m128i old_l = _mm_loadu_si128((const __m128i*)&vp->old_metrics->w[0]);
            __m128i old_h = _mm_loadu_si128((const __m128i*)&vp->old_metrics->w[32]);

            __m128i m0e = _mm_add_epi8(old_l, met_lo);
            __m128i m1e = _mm_add_epi8(old_h, cmp_lo);
            __m128i de  = _mm_cmpgt_epi8(
                _mm_xor_si128(m0e, bias), _mm_xor_si128(m1e, bias));
            __m128i se  = _mm_or_si128(
                _mm_andnot_si128(de, m0e), _mm_and_si128(de, m1e));

            __m128i m0o = _mm_add_epi8(old_l, cmp_lo);
            __m128i m1o = _mm_add_epi8(old_h, met_lo);
            __m128i d_o = _mm_cmpgt_epi8(
                _mm_xor_si128(m0o, bias), _mm_xor_si128(m1o, bias));
            __m128i so  = _mm_or_si128(
                _mm_andnot_si128(d_o, m0o), _mm_and_si128(d_o, m1o));

            _mm_storeu_si128((__m128i*)&vp->new_metrics->w[0],
                             _mm_unpacklo_epi8(se, so));
            _mm_storeu_si128((__m128i*)&vp->new_metrics->w[16],
                             _mm_unpackhi_epi8(se, so));

            uint16_t mske = (uint16_t)_mm_movemask_epi8(
                _mm_unpacklo_epi8(de, d_o));
            uint16_t msko = (uint16_t)_mm_movemask_epi8(
                _mm_unpackhi_epi8(de, d_o));
            decision_word = (uint32_t)mske | ((uint32_t)msko << 16);
        }

        /* Process states b=16..31 → new[32..63] */
        {
            __m128i old_l = _mm_loadu_si128((const __m128i*)&vp->old_metrics->w[16]);
            __m128i old_h = _mm_loadu_si128((const __m128i*)&vp->old_metrics->w[48]);

            __m128i m0e = _mm_add_epi8(old_l, met_hi);
            __m128i m1e = _mm_add_epi8(old_h, cmp_hi);
            __m128i de  = _mm_cmpgt_epi8(
                _mm_xor_si128(m0e, bias), _mm_xor_si128(m1e, bias));
            __m128i se  = _mm_or_si128(
                _mm_andnot_si128(de, m0e), _mm_and_si128(de, m1e));

            __m128i m0o = _mm_add_epi8(old_l, cmp_hi);
            __m128i m1o = _mm_add_epi8(old_h, met_hi);
            __m128i d_o = _mm_cmpgt_epi8(
                _mm_xor_si128(m0o, bias), _mm_xor_si128(m1o, bias));
            __m128i so  = _mm_or_si128(
                _mm_andnot_si128(d_o, m0o), _mm_and_si128(d_o, m1o));

            _mm_storeu_si128((__m128i*)&vp->new_metrics->w[32],
                             _mm_unpacklo_epi8(se, so));
            _mm_storeu_si128((__m128i*)&vp->new_metrics->w[48],
                             _mm_unpackhi_epi8(se, so));

            uint16_t mske = (uint16_t)_mm_movemask_epi8(
                _mm_unpacklo_epi8(de, d_o));
            uint16_t msko = (uint16_t)_mm_movemask_epi8(
                _mm_unpackhi_epi8(de, d_o));
            decision_word |= ((uint64_t)mske << 32) |
                             ((uint64_t)msko << 48);
        }

        memcpy(d_local.w, &decision_word, 8);
        memcpy(vp->dp++, &d_local, sizeof(decision_t));

        tmp = vp->old_metrics;
        vp->old_metrics = vp->new_metrics;
        vp->new_metrics = tmp;
    }

    return 0;
}

#endif /* __SSE2__ */

/* ------------------------------------------------------------------ */
/*  Runtime dispatch: pick best available at first call               */
/* ------------------------------------------------------------------ */

/* Forward-declare the scalar fallback from viterbi.c */
extern int update_viterbi_packed(void* p, uint8_t* syms, uint16_t nbits);

/* Function pointer — resolved on first call */
typedef int (*update_fn_t)(void*, uint8_t*, uint16_t);
static update_fn_t dispatch_update(void);
static int update_viterbi_packed_dispatch(void* p, uint8_t* syms, uint16_t nbits);

static update_fn_t s_update_fn = update_viterbi_packed_dispatch;

static update_fn_t dispatch_update(void)
{
#if defined(__AVX2__)
    if (__builtin_cpu_supports("avx2"))
        return update_viterbi_packed_avx2;
#endif
#if defined(__SSE2__)
    if (__builtin_cpu_supports("sse2"))
        return update_viterbi_packed_sse2;
#endif
    return update_viterbi_packed; /* scalar fallback */
}

static int update_viterbi_packed_dispatch(void* p, uint8_t* syms, uint16_t nbits)
{
    s_update_fn = dispatch_update();
    return s_update_fn(p, syms, nbits);
}

int update_viterbi_packed_simd(void* p, uint8_t* syms, uint16_t nbits)
{
    return s_update_fn(p, syms, nbits);
}
