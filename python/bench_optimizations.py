#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Benchmark suite for F4TNK optimization sessions 13-15
# Tests: F1 (Viterbi), F2 (kiss_to_pdu C++), F4 (doppler volk interleave)
#        F5 (crc_check syndrome), F6 (Viterbi flat trellis)
#
# Usage: cd build && python3 ../python/bench_optimizations.py
#

import time
import sys
import os
import numpy as np

# bootstrap satellites module from build dir
# When run as: cd build && python3 ../python/bench_optimizations.py
# sys.path[0] is the script dir (../python/), NOT CWD (build/).
# We need build/ in sys.path so 'import python' finds build/python/.
_script_dir = os.path.dirname(os.path.abspath(__file__))
_build_dir = os.path.join(os.path.dirname(_script_dir), 'build')
if os.path.isdir(os.path.join(_build_dir, 'python')):
    sys.path.insert(0, _build_dir)

try:
    import python as satellites
except ImportError:
    pass
else:
    sys.modules['satellites'] = satellites

from gnuradio import gr, blocks
import pmt


def fmt_rate(count, elapsed):
    """Format throughput."""
    if elapsed == 0:
        return "∞"
    rate = count / elapsed
    if rate >= 1e6:
        return f"{rate/1e6:.1f} M/s"
    elif rate >= 1e3:
        return f"{rate/1e3:.1f} K/s"
    else:
        return f"{rate:.1f} /s"


def bench_viterbi():
    """Benchmark F1: ViterbiCodec (encode + decode roundtrip)."""
    from satellites import convolutional_encoder, viterbi_decoder

    print("=" * 70)
    print("BENCH F1: ViterbiCodec  (encode+decode roundtrip)")
    print("=" * 70)

    configs = [
        ("K=5, poly=[25,23]", 5, [25, 23]),
        ("K=7, poly=[79,109]", 7, [79, 109]),
    ]

    for label, k, p in configs:
        for msg_len in [256, 1024, 4096]:
            enc = convolutional_encoder(k, p)
            dec = viterbi_decoder(k, p)
            dbg = blocks.message_debug()

            # Count how many roundtrips we can do
            n_iter = max(1, 2000 // msg_len)
            data = np.random.randint(2, size=msg_len, dtype='uint8')
            pdu = pmt.cons(pmt.PMT_NIL, pmt.init_u8vector(len(data), data))

            tb = gr.top_block()
            tb.msg_connect((enc, 'out'), (dec, 'in'))
            tb.msg_connect((dec, 'out'), (dbg, 'store'))

            t0 = time.perf_counter()
            for _ in range(n_iter):
                enc.to_basic_block()._post(pmt.intern('in'), pdu)
            enc.to_basic_block()._post(
                pmt.intern('system'),
                pmt.cons(pmt.intern('done'), pmt.from_long(1)))
            tb.start()
            tb.wait()
            elapsed = time.perf_counter() - t0

            # Verify correctness
            n_received = dbg.num_messages()
            ok = True
            if n_received > 0:
                out = np.array(pmt.u8vector_elements(
                    pmt.cdr(dbg.get_message(0))))
                ok = np.array_equal(out, data)

            bits_total = msg_len * n_iter
            status = "✓" if ok else "✗ MISMATCH"
            print(f"  {label}  len={msg_len:5d}  "
                  f"×{n_iter:4d}  {elapsed*1000:7.1f} ms  "
                  f"{fmt_rate(bits_total, elapsed):>12s} bit/s  {status}")

    print()


def bench_kiss_cpp_vs_python():
    """Benchmark F2: kiss_to_pdu C++ vs Python."""
    # Access the build-dir module (already loaded by __init__.py → from .kiss_to_pdu import ...)
    kmod = sys.modules['python.kiss_to_pdu']

    _kiss_to_pdu_python = kmod._kiss_to_pdu_python
    _kiss_to_pdu_cpp = kmod._kiss_to_pdu_cpp

    print("=" * 70)
    print("BENCH F2: kiss_to_pdu  C++ vs Python")
    print("=" * 70)

    # Build KISS test data: N packets of ~200 bytes each, FEND-delimited
    n_packets = 500
    pkt_size = 200
    kiss_stream = bytearray()
    reference_payloads = []
    for _ in range(n_packets):
        payload = np.random.randint(1, 0xBF, size=pkt_size, dtype='uint8')
        # Avoid FEND/FESC in payload for simplicity
        kiss_stream.append(0xC0)  # FEND
        kiss_stream.append(0x00)  # control byte (data frame)
        kiss_stream.extend(payload.tobytes())
        kiss_stream.append(0xC0)  # FEND
        reference_payloads.append(payload)

    kiss_array = np.frombuffer(bytes(kiss_stream), dtype=np.uint8)
    total_bytes = len(kiss_array)

    results = {}

    for name, make_block in [
        ("Python", lambda: _kiss_to_pdu_python(control_byte=True)),
        ("C++", lambda: _kiss_to_pdu_cpp(control_byte=True) if _kiss_to_pdu_cpp else None),
    ]:
        block = make_block()
        if block is None:
            print(f"  {name:8s}  SKIPPED (not available)")
            continue

        n_iter = 20
        dbg = blocks.message_debug()
        src = blocks.vector_source_b(kiss_array.tolist(), repeat=True)
        head = blocks.head(gr.sizeof_char, total_bytes * n_iter)

        tb = gr.top_block()
        tb.connect(src, head, block)
        tb.msg_connect((block, 'out'), (dbg, 'store'))

        t0 = time.perf_counter()
        tb.start()
        tb.wait()
        elapsed = time.perf_counter() - t0

        n_received = dbg.num_messages()
        expected = n_packets * n_iter
        ok = n_received == expected

        results[name] = elapsed
        status = "✓" if ok else f"✗ got {n_received}/{expected}"
        print(f"  {name:8s}  {total_bytes*n_iter:8d} bytes  "
              f"×{n_iter:2d}  {elapsed*1000:7.1f} ms  "
              f"{fmt_rate(total_bytes * n_iter, elapsed):>12s}  {status}")

    if "Python" in results and "C++" in results and results["C++"] > 0:
        speedup = results["Python"] / results["C++"]
        print(f"  → Speedup: {speedup:.1f}×")
    print()


def bench_doppler():
    """Benchmark F4: doppler_correction with volk interleave."""
    import python.bindings.satellites_python as _sp
    doppler_correction = _sp.doppler_correction
    import tempfile

    print("=" * 70)
    print("BENCH F4: doppler_correction  (volk interleave)")
    print("=" * 70)

    samp_rate = 48000.0
    n_samples = int(samp_rate * 5)  # 5 seconds of data

    # Create a Doppler file (time frequency pairs)
    t_start = 1700000000.0
    n_points = 50
    times = [t_start + i * 0.1 for i in range(n_points)]
    freqs = [437.0e6 + i * 100 for i in range(n_points)]

    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.doppler', delete=False)
    for t, f in zip(times, freqs):
        tmp.write(f"{t:.6f} {f:.1f}\n")
    tmp.close()

    # Generate test signal
    signal = np.exp(1j * np.linspace(0, 100 * np.pi, n_samples)).astype(np.complex64)

    n_iter = 5

    src = blocks.vector_source_c(signal.tolist(), repeat=True)
    head = blocks.head(gr.sizeof_gr_complex, n_samples * n_iter)
    doppler = doppler_correction(tmp.name, samp_rate, t_start)
    sink = blocks.null_sink(gr.sizeof_gr_complex)

    tb = gr.top_block()
    tb.connect(src, head, doppler, sink)

    t0 = time.perf_counter()
    tb.start()
    tb.wait()
    elapsed = time.perf_counter() - t0

    import os
    os.unlink(tmp.name)

    total_samples = n_samples * n_iter
    print(f"  {total_samples:,d} samples  ×{n_iter}  {elapsed*1000:.1f} ms  "
          f"{fmt_rate(total_samples, elapsed):>12s} samp/s  "
          f"({total_samples/elapsed/samp_rate:.1f}× realtime)")
    print()


def _build_hdlc_frame_bits(payload, crc_calc, preamble_bytes=4, postamble_bytes=2):
    """Build HDLC bitstream for one frame (LSB-first, with bit-stuffing)."""
    flag = [0, 1, 1, 1, 1, 1, 1, 0]

    data = list(payload)
    crc_val = crc_calc.compute(data)
    data.append(crc_val & 0xFF)
    data.append((crc_val >> 8) & 0xFF)

    bits = flag * preamble_bytes
    data_start = len(bits)

    ones = 0
    for byte in data:
        for _ in range(8):
            bit = byte & 1
            bits.append(bit)
            if bit:
                ones += 1
            else:
                ones = 0
            if ones == 5:
                bits.append(0)
                ones = 0
            byte >>= 1

    data_end = len(bits)
    bits.extend(flag * postamble_bytes)
    return bits, data_start, data_end


def bench_ax25_hdlc():
    """Benchmark AX.25-relevant HDLC deframing throughput and recovery."""
    from satellites import hdlc_deframer, crc

    print("=" * 70)
    print("BENCH AX25: HDLC deframer throughput/recovery")
    print("=" * 70)

    rng = np.random.default_rng(12345)
    n_frames = 3000
    frame_len = 96

    crc_calc = crc(16, 0x1021, 0xFFFF, 0xFFFF, True, True)

    clean_stream = []
    frame_ranges = []
    cursor = 0

    for _ in range(n_frames):
        frame = rng.integers(0, 256, size=frame_len, dtype=np.uint8)
        frame[:13] &= 0xFE
        bits, start, end = _build_hdlc_frame_bits(frame.tolist(), crc_calc)
        clean_stream.extend(bits)
        frame_ranges.append((cursor + start, cursor + end))
        cursor += len(bits)

    clean_stream = np.array(clean_stream, dtype=np.uint8)
    noisy_stream = clean_stream.copy()

    for start, end in frame_ranges:
        if end > start:
            flip_idx = int(rng.integers(start, end))
            noisy_stream[flip_idx] ^= 1

    for label, stream in [("clean", clean_stream), ("1-bit/frame", noisy_stream)]:
        src = blocks.vector_source_b(stream.tolist(), repeat=False)
        deframer = hdlc_deframer(True, 10000)
        dbg = blocks.message_debug()

        tb = gr.top_block()
        tb.connect(src, deframer)
        tb.msg_connect((deframer, 'out'), (dbg, 'store'))

        t0 = time.perf_counter()
        tb.start()
        tb.wait()
        elapsed = time.perf_counter() - t0

        recovered = dbg.num_messages()
        recovery = 100.0 * recovered / n_frames
        bit_rate = fmt_rate(len(stream), elapsed)
        frame_rate = recovered / elapsed if elapsed > 0 else float('inf')

        print(f"  {label:10s}  bits={len(stream):9d}  {elapsed*1000:7.1f} ms  "
              f"{bit_rate:>10s} bit/s  frames={recovered:4d}/{n_frames} "
              f"({recovery:5.1f}%)  {frame_rate:8.1f} fr/s")

    print()


def bench_ax25_header_false_positive():
    """Benchmark AX.25 header filtering: legacy vs strict parser."""
    from satellites.components.deframers.ax25_deframer import ax25_header_check

    print("=" * 70)
    print("BENCH AX25: header false-positive filtering")
    print("         legacy fixed-position check vs strict address parser")
    print("=" * 70)

    rng = np.random.default_rng(20260225)
    n_noise = 200000
    frame_len = 32

    valid_cs = ax25_header_check._VALID_CS
    valid_shifted = np.array([i for i in range(256) if valid_cs[i]], dtype=np.uint8)

    def legacy_check(packet):
        if len(packet) < 14:
            return False
        for i in range(13):
            if packet[i] & 0x01:
                return False
        for i in (0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12):
            if not valid_cs[packet[i]]:
                return False
        return True

    def strict_check(packet):
        if len(packet) < 16:
            return False

        offset = 0
        addr_count = 0
        max_addresses = 10

        while addr_count < max_addresses:
            if offset + 7 > len(packet):
                return False

            for i in range(6):
                if not valid_cs[packet[offset + i]]:
                    return False

            ssid = packet[offset + 6]
            if (ssid & 0x60) != 0x60:
                return False

            addr_count += 1
            offset += 7

            if ssid & 0x01:
                break
        else:
            return False

        if addr_count < 2:
            return False
        if offset + 2 > len(packet):
            return False

        return True

    # Noise conditioned to pass legacy checks (worst case for false positives).
    conditioned = np.empty((n_noise, frame_len), dtype=np.uint8)
    conditioned[:, :] = rng.integers(0, 256, size=(n_noise, frame_len), dtype=np.uint8)
    conditioned[:, 0:6] = rng.choice(valid_shifted, size=(n_noise, 6), replace=True)
    conditioned[:, 7:13] = rng.choice(valid_shifted, size=(n_noise, 6), replace=True)
    conditioned[:, 6] &= 0xFE  # force LSB=0 to satisfy legacy extension-bit check

    t0 = time.perf_counter()
    legacy_pass = 0
    strict_pass = 0
    for row in conditioned:
        pkt = bytes(row)
        if legacy_check(pkt):
            legacy_pass += 1
        if strict_check(pkt):
            strict_pass += 1
    elapsed_noise = time.perf_counter() - t0

    # Valid frames sanity checks: 2-address and 3-address (digipeater) forms.
    def cs_bytes(callsign):
        txt = callsign.ljust(6)[:6]
        return [(ord(ch) << 1) & 0xFE for ch in txt]

    valid_2addr = bytes(
        cs_bytes('CQ') + [0x60] +
        cs_bytes('F4TNK') + [0x61] +
        [0x03, 0xF0, 0x01, 0x02, 0x03]
    )
    valid_3addr = bytes(
        cs_bytes('CQ') + [0x60] +
        cs_bytes('F4TNK') + [0x60] +
        cs_bytes('WIDE1') + [0x61] +
        [0x03, 0xF0, 0x42]
    )

    print(f"  conditioned noise frames: {n_noise}")
    print(f"  legacy pass: {legacy_pass:7d} ({100.0 * legacy_pass / n_noise:6.3f}%)")
    print(f"  strict pass: {strict_pass:7d} ({100.0 * strict_pass / n_noise:6.3f}%)")
    if strict_pass > 0:
        suppression = legacy_pass / strict_pass
        print(f"  suppression factor: {suppression:,.1f}x")
    else:
        print("  suppression factor: infinite (strict accepted 0 conditioned-noise frames)")
    print(f"  runtime: {elapsed_noise*1000:.1f} ms")

    print("  valid frame sanity:")
    print(f"    2-address frame: legacy={legacy_check(valid_2addr)} strict={strict_check(valid_2addr)}")
    print(f"    3-address frame: legacy={legacy_check(valid_3addr)} strict={strict_check(valid_3addr)}")
    print()


def bench_ax25_chain_deep():
    """Deep AX.25 benchmark: end-to-end recovery + noise leakage."""
    from satellites import hdlc_deframer, crc, pdu_length_filter
    from satellites.components.deframers.ax25_deframer import ax25_header_check

    hdlc_factory = hdlc_deframer if callable(hdlc_deframer) else hdlc_deframer.hdlc_deframer

    print("=" * 70)
    print("BENCH AX25: deep end-to-end recovery/noise")
    print("=" * 70)

    def _build_bits(payload, crc_calc):
        flag = [0, 1, 1, 1, 1, 1, 1, 0]
        data = list(payload)
        c = crc_calc.compute(data)
        data += [c & 0xFF, (c >> 8) & 0xFF]

        bits = flag * 4
        start = len(bits)
        ones = 0
        for byte in data:
            for _ in range(8):
                bit = byte & 1
                bits.append(bit)
                if bit:
                    ones += 1
                else:
                    ones = 0
                if ones == 5:
                    bits.append(0)
                    ones = 0
                byte >>= 1
        end = len(bits)
        bits += flag * 2
        return bits, start, end

    def _run_recovery(bits_per_frame):
        rng = np.random.default_rng(991)
        n_frames = 2000
        frame_len = 96
        crc_calc = crc(16, 0x1021, 0xFFFF, 0xFFFF, True, True)

        stream = []
        ranges = []
        cursor = 0
        valid = [ord(c) << 1 for c in ' 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ']

        for _ in range(n_frames):
            frame = rng.integers(0, 256, size=frame_len, dtype=np.uint8)
            frame[0:6] = rng.choice(valid, size=6)
            frame[6] = 0x60
            frame[7:13] = rng.choice(valid, size=6)
            frame[13] = 0x61
            frame[14] = 0x03
            frame[15] = 0xF0

            bits, start, end = _build_bits(frame.tolist(), crc_calc)
            stream.extend(bits)
            ranges.append((cursor + start, cursor + end))
            cursor += len(bits)

        stream = np.array(stream, dtype=np.uint8)
        corrupted = stream.copy()
        for start, end in ranges:
            idx = rng.choice(np.arange(start, end), size=bits_per_frame, replace=False)
            corrupted[idx] ^= 1

        src = blocks.vector_source_b(corrupted.tolist(), repeat=False)
        defr = hdlc_factory(True, 10000)
        lengthf = pdu_length_filter(16, 10000)
        header = ax25_header_check()
        dbg = blocks.message_debug()

        tb = gr.top_block()
        tb.connect(src, defr)
        tb.msg_connect((defr, 'out'), (lengthf, 'in'))
        tb.msg_connect((lengthf, 'out'), (header, 'in'))
        tb.msg_connect((header, 'out'), (dbg, 'store'))

        t0 = time.perf_counter()
        tb.start()
        tb.wait()
        elapsed = time.perf_counter() - t0

        recovered = dbg.num_messages()
        print(f"  {bits_per_frame}-bit/frame: {recovered:4d}/{n_frames} "
              f"({100.0*recovered/n_frames:6.2f}%)  {elapsed*1000:7.1f} ms")

    _run_recovery(1)
    _run_recovery(3)

    # Noise leakage test (end-to-end AX.25 output should be 0 in practice).
    rng = np.random.default_rng(123)
    noise_bits = 2_500_000
    noise = rng.integers(0, 2, size=noise_bits, dtype=np.uint8)

    src = blocks.vector_source_b(noise.tolist(), repeat=False)
    defr = hdlc_factory(True, 10000)
    lengthf = pdu_length_filter(16, 10000)
    header = ax25_header_check()
    dbg = blocks.message_debug()

    tb = gr.top_block()
    tb.connect(src, defr)
    tb.msg_connect((defr, 'out'), (lengthf, 'in'))
    tb.msg_connect((lengthf, 'out'), (header, 'in'))
    tb.msg_connect((header, 'out'), (dbg, 'store'))

    tb.start()
    tb.wait()
    print(f"  noise-only: {noise_bits:,d} bits -> {dbg.num_messages()} AX.25 frames")
    print()


def bench_crc_check_syndrome():
    """Benchmark F5: CRC check with syndrome-based 1-bit correction.

    F4TNK Session 15: Replaced O(n²) brute-force bit-flip loop
    (payload_len × 8 full CRC recomputations) with syndrome table
    lookup: 1 CRC computation + O(1) table lookup.

    For a 300-byte frame: ~2400× speedup on CRC-fail path.
    """
    from satellites import crc_check, crc

    print("=" * 70)
    print("BENCH F5: crc_check syndrome-based 1-bit correction")
    print("         (F4TNK Session 15: O(n²) → O(1) syndrome table)")
    print("=" * 70)

    rng = np.random.default_rng(54321)

    # CRC-16/CCITT (AX.25 standard)
    crc_calc = crc(16, 0x1021, 0xFFFF, 0xFFFF, True, True)

    for frame_len in [64, 256, 512]:
        n_frames = 2000
        checker = crc_check(16, 0x1021, 0xFFFF, 0xFFFF, True, True,
                            False, True, 0)
        dbg_ok = blocks.message_debug()
        dbg_fail = blocks.message_debug()

        tb = gr.top_block()
        tb.msg_connect((checker, 'ok'), (dbg_ok, 'store'))
        tb.msg_connect((checker, 'fail'), (dbg_fail, 'store'))

        # Build PDUs: half clean, half with 1-bit error
        pdus = []
        for i in range(n_frames):
            payload = rng.integers(0, 256, size=frame_len, dtype=np.uint8)
            crc_val = crc_calc.compute(payload.tolist())
            frame = np.concatenate([payload,
                                    np.array([(crc_val >> 8) & 0xFF, crc_val & 0xFF],
                                             dtype=np.uint8)])
            if i >= n_frames // 2:
                # Introduce 1-bit error
                err_byte = int(rng.integers(0, frame_len))
                err_bit = int(rng.integers(0, 8))
                frame[err_byte] ^= (1 << err_bit)

            pdu = pmt.cons(pmt.PMT_NIL, pmt.init_u8vector(len(frame), frame))
            pdus.append(pdu)

        t0 = time.perf_counter()
        for pdu in pdus:
            checker.to_basic_block()._post(pmt.intern('in'), pdu)
        checker.to_basic_block()._post(
            pmt.intern('system'),
            pmt.cons(pmt.intern('done'), pmt.from_long(1)))
        tb.start()
        tb.wait()
        elapsed = time.perf_counter() - t0

        n_ok = dbg_ok.num_messages()
        n_fail = dbg_fail.num_messages()
        clean = n_frames // 2
        corrupted = n_frames - clean

        us_per_pdu = elapsed / n_frames * 1e6
        print(f"  len={frame_len:4d}  ×{n_frames:4d}  {elapsed*1000:7.1f} ms  "
              f"{us_per_pdu:6.1f} µs/PDU  "
              f"ok={n_ok:4d}  fail={n_fail:4d}  "
              f"recovery={100.0*n_ok/(clean+corrupted):5.1f}%")
    print()


def bench_viterbi_standalone():
    """Benchmark Viterbi codec directly via C++ pybind, no flowgraph overhead."""
    print("=" * 70)
    print("BENCH F1b: ViterbiCodec direct  (no flowgraph overhead)")
    print("         (F4TNK Session 15: flat trellis + reusable buffers)")
    print("=" * 70)

    from satellites import convolutional_encoder, viterbi_decoder

    configs = [
        ("K=7, poly=[79,109]", 7, [79, 109]),
    ]

    for label, k, p in configs:
        for msg_len in [256, 1024, 4096]:
            enc = convolutional_encoder(k, p)
            dec = viterbi_decoder(k, p)
            dbg_enc = blocks.message_debug()
            dbg_dec = blocks.message_debug()

            data = np.random.randint(2, size=msg_len, dtype='uint8')
            pdu = pmt.cons(pmt.PMT_NIL, pmt.init_u8vector(len(data), data))

            # --- Encode timing ---
            n_iter = max(10, 5000 // msg_len)
            tb = gr.top_block()
            tb.msg_connect((enc, 'out'), (dbg_enc, 'store'))
            for _ in range(n_iter):
                enc.to_basic_block()._post(pmt.intern('in'), pdu)
            enc.to_basic_block()._post(
                pmt.intern('system'),
                pmt.cons(pmt.intern('done'), pmt.from_long(1)))
            t0 = time.perf_counter()
            tb.start()
            tb.wait()
            t_enc = time.perf_counter() - t0

            # Get encoded data for decode benchmark
            encoded_pdu = dbg_enc.get_message(0)
            encoded_data = np.array(pmt.u8vector_elements(pmt.cdr(encoded_pdu)))

            # --- Decode timing ---
            dec_pdu = pmt.cons(pmt.PMT_NIL,
                               pmt.init_u8vector(len(encoded_data), encoded_data))
            tb2 = gr.top_block()
            dbg_dec = blocks.message_debug()
            tb2.msg_connect((dec, 'out'), (dbg_dec, 'store'))
            for _ in range(n_iter):
                dec.to_basic_block()._post(pmt.intern('in'), dec_pdu)
            dec.to_basic_block()._post(
                pmt.intern('system'),
                pmt.cons(pmt.intern('done'), pmt.from_long(1)))
            t0 = time.perf_counter()
            tb2.start()
            tb2.wait()
            t_dec = time.perf_counter() - t0

            # Verify
            out = np.array(pmt.u8vector_elements(
                pmt.cdr(dbg_dec.get_message(0))))
            ok = np.array_equal(out, data)

            bits = msg_len * n_iter
            status = "✓" if ok else "✗"
            us_per_enc = t_enc / n_iter * 1e6
            us_per_dec = t_dec / n_iter * 1e6
            print(f"  {label}  len={msg_len:5d}  ×{n_iter:4d}  "
                  f"enc={us_per_enc:6.0f} µs  dec={us_per_dec:6.0f} µs  "
                  f"dec_rate={fmt_rate(bits, t_dec):>10s} bit/s  {status}")
    print()


if __name__ == '__main__':
    print()
    print("F4TNK gr-satellites Optimization Benchmarks — Sessions 13-15")
    print(f"{'='*70}")
    print()

    bench_viterbi()
    bench_viterbi_standalone()
    bench_kiss_cpp_vs_python()
    bench_ax25_hdlc()
    bench_ax25_header_false_positive()
    bench_ax25_chain_deep()
    bench_crc_check_syndrome()
    bench_doppler()

    print("Done.")
