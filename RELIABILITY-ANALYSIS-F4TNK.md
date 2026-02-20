# gr-satellites — Decoding Reliability Analysis

Branch: `master-f4tnk` (commit `ed943d13`)  
Based on: upstream `main` (`ad332988`)  
Analyst: F4TNK  
Date: 2026-02-20  
Station context: SatNOGS #3762, AirSpy R2 @ 2.5 MSPS, x86-64 (AVX2+BMI2)

---

## Critical Issues

### R1. `pdu_scrambler_impl.cc` — Missing `return` after error → Out-of-Bounds Read

**File**: `lib/pdu_scrambler_impl.cc` **Lines 55–62**  
**Severity**: **CRITICAL** — heap buffer over-read, potential crash or silent data corruption

```cpp
void pdu_scrambler_impl::msg_handler(pmt::pmt_t pmt_msg)
{
    std::vector<uint8_t> msg = pmt::u8vector_elements(pmt::cdr(pmt_msg));

    if (msg.size() > d_sequence.size()) {
        d_logger->error("PDU longer than scrambler sequence; dropping");
        // *** BUG: missing "return;" here ***
    }

    for (size_t j = 0; j < msg.size(); ++j) {
        msg[j] ^= d_sequence[j];  // reads d_sequence[j] out-of-bounds when j >= d_sequence.size()
    }

    message_port_pub(pmt::mp("out"),
                     pmt::cons(pmt::car(pmt_msg), pmt::init_u8vector(msg.size(), msg)));
}
```

**Impact**: When a PDU arrives that is longer than the scrambler sequence, the
code logs "dropping" but does NOT return. It then iterates over `msg.size()`
elements, reading `d_sequence[j]` for indices beyond `d_sequence.size()`. This
is undefined behavior: heap buffer over-read. On a SatNOGS station processing
AX.25 or CCSDS frames, a malformed or noise-triggered frame with incorrect
length can trigger this path. The result is either:
- Silently XORing payload with garbage heap data → corrupted frame passed downstream
- Segfault if the read crosses a mapped page boundary

**Fix**: Add `return;` after the error log.

---

### R2. `viterbi.c` — Wrong decision pointer writeback after `update_viterbi_packed`

**File**: `lib/viterbi.c` **Line 295**  
**Severity**: **HIGH** — incorrect Viterbi decisions if called in multiple chunks

```c
int update_viterbi_packed(void* p, uint8_t* syms, uint16_t nbits)
{
    struct v27* vp = p;
    // ...
    decision_t *dp, *d = &decisions_local;
    // ...
    dp = vp->dp;

    while (likely(nbits--)) {
        // ... process butterfly ...
        memcpy(dp++, d, sizeof(decision_t));  // dp advances through decisions array
        // ...
    }

    vp->dp = d;    // BUG: should be "vp->dp = dp;"
    return 0;
}
```

**Impact**: After processing, `vp->dp` is set to `d` (which points to the
static `decisions_local` buffer) instead of `dp` (the advanced pointer past all
written decisions). If `update_viterbi_packed()` is called in chunks (multiple
calls per frame), the second call would overwrite the first call's decisions
because `dp` would restart from the wrong position.

Currently mitigated because `u482c_decode_impl.cc` calls `update_viterbi_packed_simd()`
(which uses `vp->dp++` directly — correct), and the single-call pattern for
a whole frame means chainback uses `vp->decisions` instead of `vp->dp`.

The SIMD variant (`viterbi_simd.c`) does NOT have this bug — it uses `vp->dp++`
directly. But any future code that calls the scalar fallback in chunks would
silently produce incorrect decoded output.

**Fix**: Change `vp->dp = d;` to `vp->dp = dp;`

---

### R3. `varlen_packet_framer_impl.cc` — `min`/`max` swap forces header to 32 bits

**File**: `lib/varlen_packet_framer_impl.cc` **Line 62**  
**Severity**: **HIGH** — all non-Golay framing ignores user-specified header length

```cpp
d_header_length = ((d_header_length + 7) / 8) * 8;
d_header_length = std::max(32, std::min(8, d_header_length));
//                ^^^^^^^^        ^^^^^^^^  SWAPPED
```

`std::min(8, d_header_length)` returns at most 8. Then `std::max(32, ≤8)`
always returns 32. So `d_header_length` is unconditionally forced to 32
regardless of the user-specified value.

**Intended logic**: Clamp header length to [8, 32] bits:
```cpp
d_header_length = std::min(32, std::max(8, d_header_length));
```

**Impact**: Any satellite protocol using `varlen_packet_framer` with a header
length ≠ 32 and Golay disabled will silently use 32-bit headers, causing
incorrect framing. This corrupts transmitted frames and makes length decoding
at the receiver wrong.

---

## Medium Issues

### R4. `kurtosis_impl.cc` — Division by zero when `sum2 == 0`

**File**: `lib/kurtosis_impl.cc` **Line 64**  
**Severity**: **MEDIUM** — NaN propagation to downstream blocks

```cpp
const float kurt =
    M / (M - 1.0f) * ((M + 1.0f) * sum4 / (sum2 * sum2) - 2.0f);
out[j * d_vlen + k] = kurt;
```

When the input block is all-zero (common during signal absence in a LEO pass,
or when the frontend reports silence between passes), `sum2 = 0.0f` and
`sum4 = 0.0f`, producing `0.0f / 0.0f = NaN`. The NaN then propagates to all
downstream blocks that depend on the kurtosis output (e.g., `level_to_message`
threshold comparison).

Additionally, when `M == 1` (block_size = 1), the denominator `(M - 1.0f) == 0`,
always producing `inf`.

**Fix**:
```cpp
float kurt;
if (sum2 > 0.0f && M > 1.0f) {
    kurt = M / (M - 1.0f) * ((M + 1.0f) * sum4 / (sum2 * sum2) - 2.0f);
} else {
    kurt = 0.0f;  // or 2.0f for Gaussian noise baseline
}
```

---

### R5. `decode_ra_code_impl.cc` — `reserve()` without `resize()` → UB

**File**: `lib/decode_ra_code_impl.cc` **Lines 42, 94, 102**  
**Severity**: **MEDIUM** — technically undefined behavior, works by accident

```cpp
// Constructor:
d_ra_out.reserve(d_size);    // allocates but size() remains 0

// msg_handler:
ra_decoder_gen(d_ra_context.get(), d_ra_in.data(),
               (ra_word_t*)d_ra_out.data(), 40);  // writes into reserved-but-not-owned memory

// ...
message_port_pub(
    pmt::mp("out"),
    pmt::cons(pmt::PMT_NIL, pmt::init_u8vector(d_size, d_ra_out.data())));
    // reads d_size bytes from a vector with size() == 0
```

`reserve()` allocates storage but `size()` stays 0. Accessing elements beyond
`size()` via `data()` is formally undefined behavior per the C++ standard.
In practice this works because the memory *is* allocated, but it's fragile:
any operation that triggers reallocation (e.g., a bug adding elements) could
invalidate the pointer.

**Fix**: Change `d_ra_out.reserve(d_size)` to `d_ra_out.resize(d_size)`.

---

### R6. Costas Loop 8-APSK — Mini-batch reduces tracking bandwidth for LEO

**File**: `lib/costas_loop_8apsk_cc_impl.cc` **Lines 58–74**  
**Severity**: **MEDIUM** for fast-Doppler LEO — tracking degradation at high Doppler rates

```cpp
static constexpr int BATCH = 8;
// ...
d_error = phase_detector(out[j + batch - 1]);  // only last sample in batch
advance_loop(d_error);
```

The PLL error is computed from only the **last** sample in each 8-sample batch.
This means:
1. 7/8 of the error signal information is discarded
2. The effective PLL update rate is reduced by 8×
3. For LEO satellites with high Doppler rate (up to 800 Hz/s at UHF), this can
   cause tracking lag: at 9600 baud, the 8-sample batch spans ~0.83 ms. Doppler
   change over 0.83 ms ≈ 0.66 Hz — not critical, but detectable.

For low-baudrate signals (1200 baud AFSK), the batch spans ~6.7 ms, and Doppler
change is ~5.3 Hz — potentially problematic for narrow-bandwidth signals.

**Mitigation**: The batch size of 8 is a reasonable trade-off for most cases.
Consider making it configurable or reducing to 4 for low-baudrate modes.

---

### R7. `hdlc_deframer.py` — Pure Python per-bit processing in `work()`

**File**: `python/hdlc_deframer.py` **Lines 57–80**  
**Severity**: **MEDIUM** — performance bottleneck under high frame rate

```python
def work(self, input_items, output_items):
    in0 = input_items[0]
    for x in in0:               # Python iteration over every single bit
        if x:
            self.ones += 1
            self.bits.append(x)
        else:
            if self.ones == 5:
                None            # bit-stuffing removal
            elif self.ones > 5:
                # Flag processing...
```

Every received bit is processed in a CPython for-loop with individual attribute
lookups and deque appends. For a 9600 baud FM signal, this is ~9600 Python
operations per second — acceptable. But for higher baudrates or burst traffic
(multiple overlapping stations), this becomes the throughput bottleneck.

Under heavy load, the Python GIL contention between the GR scheduler and the
HDLC deframer can cause the GR flowgraph to back-pressure, dropping samples
from the ADC buffer. This manifests as intermittent frame losses that are
difficult to diagnose.

**Impact**: Reduced decoding reliability under high CPU load or high frame rate.

---

### R8. `fixedlen_to_pdu_impl.cc` — Unbounded `d_packet_infos` growth

**File**: `lib/fixedlen_to_pdu_impl.cc` **Lines 65–93**  
**Severity**: **MEDIUM** — memory growth under noise, potential OOM

```cpp
std::vector<tag_t> tags;
get_tags_in_window(tags, 0, 0, noutput_items, d_syncword_tag);
for (const auto& tag : tags) {
    // ...
    d_packet_infos.push_back(info);  // grows without bound
}
```

Every syncword tag detected triggers a `push_back` into `d_packet_infos`. When
processing noise (no valid signal), the correlator can generate many false
syncword tags. These accumulate in `d_packet_infos` because packets that extend
beyond the current buffer are saved for the next `work()` call (line 140):
```cpp
d_new_packet_infos.push_back(info);
```

If the false tag rate is high and the packet length is large, this vector can
grow indefinitely, consuming memory. On a 24/7 SatNOGS station processing
between passes, this could slowly leak memory over hours.

**Fix**: Add a maximum capacity check or age-based pruning for `d_packet_infos`.

---

## Low-Severity Issues

### R9. `doppler_correction_impl.cc` — `phase_wrap()` uses while-loop

**File**: `lib/doppler_correction_impl.h` **Lines 59–64**  
**Severity**: **LOW** — potential performance issue under extreme conditions

```cpp
void phase_wrap()
{
    while (d_phase > (2 * GR_M_PI))
        d_phase -= 2 * GR_M_PI;
    while (d_phase < (-2 * GR_M_PI))
        d_phase += 2 * GR_M_PI;
}
```

This is called once per sample in the scalar phase ramp (Pass 1). Under normal
conditions, the phase stays bounded. However, if `freq` is very large (badly
corrupted Doppler file), the while-loops could iterate many times per sample.
Using `fmod()` would be constant-time.

---

### R10. `phase_unwrap_impl.cc` — Initial state `d_last_phase = GR_M_PI`

**File**: `lib/phase_unwrap_impl.cc` **Line 25**  
**Severity**: **LOW** — potential off-by-one cycle on first sample

```cpp
phase_unwrap_impl::phase_unwrap_impl()
    : // ...
      d_last_phase(GR_M_PI)   // initialized to π
```

The wrap detection logic:
```cpp
if ((d_last_phase < GR_M_PI / 2.0) && (phase > 3.0 * GR_M_PI / 2.0))
    --d_integer_cycles;
else if ((d_last_phase > 3.0 * GR_M_PI / 2.0) && (phase < GR_M_PI / 2.0))
    ++d_integer_cycles;
```

With `d_last_phase = π`, neither condition triggers on the first sample (π is
in the "dead zone" between π/2 and 3π/2). This is safe but means the initial
transition detection depends on where the first actual sample falls. The choice
of π is defensive — it won't trigger a false wrap.

---

### R11. `crc_check_impl.cc` — Unsigned loop variable in swap_endianness path

**File**: `lib/crc_check_impl.cc` **Lines 101–104**  
**Severity**: **LOW** — theoretically risky, safe in practice due to earlier bounds check

```cpp
if (d_swap_endianness) {
    for (auto i = size - 1; i >= size - num_bytes; --i) {
        msg_crc <<= 8;
        msg_crc |= msg[i];
    }
}
```

`i` is deduced as `size_t` (unsigned). The decrement `--i` when `i == 0` would
wrap to `SIZE_MAX`. However, the earlier check `if (size <= d_header_bytes + num_bytes)`
guarantees that `size - num_bytes >= 1`, preventing the wrap. Still, the
pattern is fragile if the guard is ever modified.

---

### R12. `ViterbiCodec` (`lib/viterbi/viterbi.cc`) — String-based interface

**File**: `lib/viterbi/viterbi.cc` **Entire file**  
**Severity**: **LOW** (performance, not correctness) — documented in OPTIMIZATION-F4TNK.md as F1

The generic Viterbi decoder uses `std::string` for bit representation throughout:
```cpp
std::string ViterbiCodec::Output(int current_state, int input) const {
    return outputs_.at(current_state | (input << (constraint_ - 1)));
}
```

Each `Output()` call returns a string; `HammingDistance()` compares strings
character-by-character; the trellis and traceback work on strings. For a
CCSDS R=1/2 K=7 frame of 256 bytes:
- 1024 `UpdatePathMetrics` calls, each with string creation/comparison
- String `substr` in traceback
- Total: ~50K+ string operations per frame

Already partially mitigated by `std::move` (Lot 3), but a full rewrite to
`std::vector<uint8_t>` would give ~2-4× speedup.

---

### R13. `autopolarization.py` — Potential division by near-zero noise amplitude

**File**: `python/autopolarization.py` **Lines 66–68**  
**Severity**: **LOW** — could produce NaN in dual-polarization setups

```python
alpha = (np.exp(1j*a_phase)
         * sig_ampl[0] / noise_ampl[0])[:, np.newaxis]
beta = (np.exp(1j*b_phase)
        * sig_ampl[1] / noise_ampl[1])[:, np.newaxis]
```

If `noise_ampl[0]` or `noise_ampl[1]` is near zero (e.g., one antenna
disconnected), the division produces `inf` or `NaN`, which then corrupts the
combined output. The `1e-6` epsilon in the `tau` computation (line 61) does not
protect these divisions.

---

## LEO-Specific Concerns

### L1. Hard-coded constants in demodulators

**File**: `python/components/demodulators/bpsk_demodulator.py`  
**Lines**: 197–203

```python
_default_rrc_alpha = 0.35
_default_fll_bw = 25        # Hz
_default_clk_rel_bw = 0.06
_default_clk_limit = 0.004
_default_costas_bw = 50     # Hz
```

These defaults are reasonable for medium-baudrate LEO signals but may cause
issues for:
- **Very low baudrate** (< 300 baud CW/BPSK): FLL BW of 25 Hz is too wide
  relative to signal BW; Costas BW of 50 Hz can track noise instead of signal.
- **High-speed LEO** (38400+ baud): RRC alpha of 0.35 may need adjustment for
  signal with different pulse shapes.

These are configurable via `--fll_bw`, `--costas_bw` etc., so they are not bugs
but rather suboptimal defaults for edge-case LEO baudrates.

### L2. FSK demodulator — square pulse filter length

**File**: `python/components/demodulators/fsk_demodulator.py` **Line 108**

```python
sqfilter_len = int(samp_rate / baudrate)
taps = np.ones(sqfilter_len)/sqfilter_len
```

The integrate-and-dump filter uses exact symbol-length averaging. For LEO
signals with Doppler-induced baudrate offset, this filter has a slight mismatch.
The symbol sync TED (Gardner) compensates, but the initial filter mismatch
reduces the eye opening at the TED input, worsening BER at low SNR.

---

## Summary Table

| ID | File | Line(s) | Severity | Category |
|----|------|---------|----------|----------|
| R1 | `pdu_scrambler_impl.cc` | 55-62 | **CRITICAL** | Buffer over-read |
| R2 | `viterbi.c` | 295 | **HIGH** | Wrong pointer writeback |
| R3 | `varlen_packet_framer_impl.cc` | 62 | **HIGH** | Logic error (min/max swap) |
| R4 | `kurtosis_impl.cc` | 64 | MEDIUM | Division by zero → NaN |
| R5 | `decode_ra_code_impl.cc` | 42 | MEDIUM | UB (reserve vs resize) |
| R6 | `costas_loop_8apsk_cc_impl.cc` | 58-74 | MEDIUM | Tracking degradation |
| R7 | `hdlc_deframer.py` | 57-80 | MEDIUM | Performance/reliability |
| R8 | `fixedlen_to_pdu_impl.cc` | 65-93 | MEDIUM | Memory growth |
| R9 | `doppler_correction_impl.h` | 59-64 | LOW | While-loop phase wrap |
| R10 | `phase_unwrap_impl.cc` | 25 | LOW | Initial state choice |
| R11 | `crc_check_impl.cc` | 101-104 | LOW | Unsigned arithmetic |
| R12 | `viterbi/viterbi.cc` | all | LOW | String-based perf |
| R13 | `autopolarization.py` | 66-68 | LOW | Division by zero risk |

### Files analyzed with NO issues found:

| File | Notes |
|------|-------|
| `ax100_decode_impl.cc` | Clean: bounds check via `std::min`, RS decode checked |
| `convolutional_encoder_impl.cc` | Clean: pre-allocated strings (F4TNK Lot 2) |
| `crc.cc` | Clean: SWAR reflect + slice-by-4 correct, mask applied |
| `crc_append_impl.cc` | Clean: proper size checks |
| `descrambler308_impl.cc` | Clean: sequential LFSR, inherently safe |
| `distributed_syncframe_soft_impl.cc` | Clean: VOLK dot_prod correct, threshold conversion verified |
| `encode_rs_impl.cc` | Clean: bounds managed by stride pattern |
| `frame_counter_impl.cc` | Clean: simple memcpy + message pub |
| `level_to_message_impl.cc` | Clean: simple threshold compare |
| `lilacsat1_demux_impl.cc` | Clean: `.at()` bounds-checked access |
| `manchester_sync_impl.cc` | Clean: VOLK mag_sq correct for decision |
| `matrix_deinterleaver_soft_impl.cc` | Clean: cache-friendly transpose verified |
| `message_counter_impl.cc` | Clean: trivial pass-through |
| `nanocom_golay_decode_length_impl.cc` | Clean: Golay decode return checked |
| `nrzi_decode_impl.cc` | Clean: history-based, no overflow risk |
| `nrzi_encode_impl.cc` | Clean: loop-carried, safe |
| `nusat_decoder_impl.cc` | Clean: RS checked, CRC checked, length validated |
| `packet_csma_impl.cc` | Clean: proper blocking with sleep |
| `pdu_add_meta_impl.cc` | Clean: PMT dict operations |
| `pdu_head_tail_impl.cc` | Clean: `std::min` bounds clamp |
| `pdu_length_filter_impl.cc` | Clean: simple size comparison |
| `rms_agc_cc_impl.cc` | Clean: epsilon anti-div-zero, thread-safe setters |
| `rms_agc_ff_impl.cc` | Clean: same pattern as cc variant |
| `selector_impl.cc` | Clean: mutex-protected, topology-checked |
| `time_dependent_delay_impl.cc` | Clean: polyphase FIR, proper tag propagation |
| `u482c_decode_impl.cc` | Clean: uses SIMD variant, all decode results checked |
| `u482c_encode_impl.cc` | Clean: length validation, proper encoding chain |
| `varlen_packet_tagger_impl.cc` | Clean: MTU check, Golay result checked |
| `viterbi_decoder_impl.cc` | Clean: pre-allocated buffers (F4TNK Lot 2) |
| `viterbi_simd.c` | Clean: dp advanced correctly via `vp->dp++` |
| `viterbi/viterbi.cc` | Functional correctness OK (perf issue R12 noted) |
| `decode_rs_impl.cc` | Clean: size/interleave validated, RS result checked |

---

## Recommended Priority

1. **R1** (pdu_scrambler missing return) — One-line fix, eliminates crash risk
2. **R3** (min/max swap) — One-line fix, restores correct header sizing
3. **R2** (viterbi dp pointer) — One-line fix, prevents future chunked-call bugs
4. **R4** (kurtosis div-by-zero) — Guard against NaN propagation
5. **R5** (RA code reserve→resize) — One-line fix, eliminates UB
6. **R8** (fixedlen_to_pdu memory) — Add capacity limit
