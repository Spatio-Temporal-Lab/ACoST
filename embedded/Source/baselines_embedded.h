/**
 * baselines_embedded.h
 *
 * Embedded-friendly implementations of 4 streaming baseline compressors
 * from the Serf paper (SIGMOD 2025), for STM32F103 benchmarking:
 *
 *   1. Gorilla  — Facebook XOR-based lossless (streaming, per-value)
 *   2. Serf-Qt  — Quantize + ZigZag + Elias-Gamma lossy (streaming, per-value)
 *   3. Elf      — Precision-adaptive XOR lossless (streaming, per-value)
 *
 * Each algorithm compresses a SINGLE double value (lon or lat) per call.
 * The benchmark wrapper calls each twice per GPS point (once lon, once lat).
 *
 * All implementations use fixed-size buffers and no dynamic memory.
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>

/* ── Shared embedded output/input bit streams ────────────────────────── */

#define BL_BUF_BYTES 64

struct BlOutBits {
    uint8_t buf[BL_BUF_BYTES];
    int     bit_count;
    void reset() { memset(buf, 0, sizeof(buf)); bit_count = 0; }
    void write(uint64_t val, int len);
    void write_bit(bool b) { write(b ? 1ULL : 0ULL, 1); }
    int  byte_count() const { return (bit_count + 7) / 8; }
};

struct BlInBits {
    const uint8_t *buf;
    int total_bits, pos;
    void     init(const uint8_t *b, int bits) { buf = b; total_bits = bits; pos = 0; }
    uint64_t read(int len);
    uint32_t read_bit() { return static_cast<uint32_t>(read(1)); }
};

/* ════════════════════════════════════════════════════════════════════════
 *  1. Gorilla (Facebook, VLDB 2015)
 *     XOR encoding of consecutive double values.
 *     Lossless, streaming, per-value.
 * ════════════════════════════════════════════════════════════════════════ */

struct GorillaEmbComp {
    bool     first;
    uint64_t prev;
    int      prev_lead, prev_trail;
    int      total_bits;

    void init();
    /** Compress one double value. Returns bits written to out. */
    int  compress(double v, BlOutBits &out);
};

struct GorillaEmbDecomp {
    bool     first;
    uint64_t prev;
    int      prev_lead, prev_trail;

    void init();
    /** Decompress one double value from in. */
    double decompress(BlInBits &in);
};

/* ════════════════════════════════════════════════════════════════════════
 *  2. Serf-Qt (SIGMOD 2025)
 *     Quantize delta from previous recovered value, ZigZag + Elias-Gamma.
 *     Lossy (error-bounded), streaming, per-value.
 * ════════════════════════════════════════════════════════════════════════ */

struct SerfQtEmbComp {
    double max_diff;     /* = epsilon * 0.999 */
    double quant_step;   /* = 2 * max_diff */
    double prev;
    int    point_count;
    int    total_bits;   /* total data bits (excl header) */

    void init(double epsilon);
    /** Compress one double value. Returns bits written to out. */
    int  compress(double v, BlOutBits &out);
};

struct SerfQtEmbDecomp {
    double max_diff;
    double quant_step;
    double prev;
    int    point_count;

    void init(double epsilon);
    /** Decompress one double value from in. */
    double decompress(BlInBits &in);
};

/* ════════════════════════════════════════════════════════════════════════
 *  2b. Serf-XOR (SIGMOD 2025)
 *     Error-bounded XOR encoding: find the representable value within
 *     [v-eps, v+eps] that minimizes trailing bits after XOR with previous.
 *     Lossy (error-bounded), streaming, per-value.
 *     Gorilla-style XOR + Elias-Gamma length coding.
 * ════════════════════════════════════════════════════════════════════════ */

struct SerfXorEmbComp {
    double   max_diff;
    uint64_t prev;
    int      prev_lead, prev_trail;
    bool     first;
    int      total_bits;

    void init(double epsilon);
    /** Compress one double value. Returns bits written to out. */
    int  compress(double v, BlOutBits &out);
};

struct SerfXorEmbDecomp {
    uint64_t prev;
    int      prev_lead, prev_trail;
    bool     first;

    void init();
    /** Decompress one double value from in. */
    double decompress(BlInBits &in);
};

/* ════════════════════════════════════════════════════════════════════════
 *  3. Elf (VLDB 2023)
 *     Precision-adaptive XOR: erase trailing bits beyond significant
 *     decimal digits, then XOR encode like Gorilla.
 *     Lossless (for the erased precision), streaming, per-value.
 *
 *     Simplified embedded version: uses a fixed beta=52 (all mantissa
 *     bits significant) which degrades to Gorilla-like behavior but
 *     with the Elf flag structure. For true precision detection we'd
 *     need the significand-count lookup table (~4KB).
 * ════════════════════════════════════════════════════════════════════════ */

struct ElfEmbComp {
    bool     first;
    uint64_t prev;
    uint64_t prev_erased;
    int      prev_lead, prev_trail;
    int      total_bits;

    void init();
    int  compress(double v, BlOutBits &out);

private:
    /** Erase trailing mantissa bits beyond significant precision. */
    static uint64_t erase_trailing(uint64_t bits);
};

struct ElfEmbDecomp {
    bool     first;
    uint64_t prev;
    uint64_t prev_erased;
    int      prev_lead, prev_trail;

    void init();
    double decompress(BlInBits &in);

    static uint64_t erase_trailing(uint64_t bits);
};

/* ════════════════════════════════════════════════════════════════════════
 * ════════════════════════════════════════════════════════════════════════ */


