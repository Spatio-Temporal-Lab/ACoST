/**
 * acost_embedded.h
 *
 * Embedded-friendly (no STL, no dynamic memory) single-point streaming
 * implementation of the ACoST compressor and decompressor for STM32F103.
 *
 * ACoST improvements over the existing ACoST-DoD embedded version:
 *   1. Extended CTRA range (dt_threshold=2000, no LOW-RATE branch)
 *   2. Smooth exponential CTRA damping (onset=100, tau=200)
 *   3. Adaptive velocity tail (shrinks at large intervals)
 *   4. ZP-selection: per-point 1-bit flag choosing zero-prediction vs
 *      motion-prediction based on Elias-Gamma residual bit cost
 *
 * Wire format (MSB-first bit stream):
 *   First packet:  [block_size 16b][epsilon 64b][lon 64b][lat 64b][ts 64b]
 *   Later packets: [ts raw 64b][zp_flag? 1b][res_lon EG][res_lat EG]
 *
 * Constraints:
 *   - Fixed arrays only; no new/delete, no std::vector.
 *   - Uses only <cstdint>, <cmath>, <cstring>.
 *   - C++ class syntax but no exceptions, no RTTI.
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Configuration ────────────────────────────────────────────────────── */

/** Maximum bytes in one per-point compressed buffer. */
#define ACOST_EMB_BUF_BYTES   64

/** Maximum history points for CTRA prediction. */
#define ACOST_EMB_MAX_HIST     8

/* ── Fixed-size output bit stream (MSB-first) ─────────────────────────── */

struct ACoSTOutBits {
    uint8_t buf[ACOST_EMB_BUF_BYTES];
    int     bit_count;

    void     reset();
    void     write(uint64_t val, int len);
    void     write_bit(bool b);
    int      byte_count() const { return (bit_count + 7) / 8; }
};

/* ── Fixed-size input bit stream (MSB-first) ──────────────────────────── */

struct ACoSTInBits {
    const uint8_t *buf;
    int            total_bits;
    int            pos;

    void     init(const uint8_t *b, int bits);
    uint64_t read(int len);
    uint32_t read_bit();
};

/* ── History point ────────────────────────────────────────────────────── */

struct ACoSTHistPt {
    int64_t  q_lon;
    int64_t  q_lat;
    uint64_t ts;
};

/* ── ACoST configuration (matches PC version defaults) ───────────────── */

struct ACoSTConfig {
    double ctra_omega_eps;       /* 1e-8  */
    double ctra_damp_onset;      /* 100.0 (=10 real seconds in 0.1s units) */
    double ctra_damp_tau;        /* 200.0 (=20 real seconds) */
    double zp_selection_onset;   /* 100.0 */
    int    speed_tail_segments;  /* 3 */
    int    accel_tail_segments;  /* 2 */
    double max_speed_ms;         /* 200.0 */
    double max_cap_abs_m;        /* 50000.0 */
};

/** Default configuration matching acost_compressor.h ACoSTConfig. */
static inline ACoSTConfig acost_default_config() {
    ACoSTConfig c;
    c.ctra_omega_eps     = 1.0e-8;
    c.ctra_damp_onset    = 100.0;
    c.ctra_damp_tau      = 200.0;
    c.zp_selection_onset = 100.0;
    c.speed_tail_segments = 3;
    c.accel_tail_segments = 2;
    c.max_speed_ms       = 200.0;
    c.max_cap_abs_m      = 50000.0;
    return c;
}

/* ── Compressor ───────────────────────────────────────────────────────── */

struct ACoSTCompressor {
    double      epsilon;
    double      quant_step;
    double      lon_scale;     /* cos(ref_lat) */
    int         point_count;
    ACoSTHistPt    hist[ACOST_EMB_MAX_HIST];
    int         hist_count;
    int64_t     cur_q_lon, cur_q_lat;
    uint64_t    cur_ts;
    ACoSTConfig    cfg;

    /* Running statistics */
    uint32_t    total_bits;    /* sum of compressed bits (excl timestamp) */

    void init(double eps);
    void init(double eps, const ACoSTConfig &config);

    /**
     * Compress one GPS point.
     * @return Number of bits written into out.
     */
    int compress(double lon, double lat, uint64_t ts_ms, ACoSTOutBits &out);

private:
    void push_hist(int64_t ql, int64_t qa, uint64_t ts);
    int  encode_value(uint64_t v, ACoSTOutBits &out);
    int  encode_residual(int64_t rl, int64_t ra, ACoSTOutBits &out);
};

/* ── Decompressor ─────────────────────────────────────────────────────── */

struct ACoSTDecompressor {
    double      epsilon;
    double      quant_step;
    double      lon_scale;
    int         point_count;
    ACoSTHistPt    hist[ACOST_EMB_MAX_HIST];
    int         hist_count;
    int64_t     cur_q_lon, cur_q_lat;
    uint64_t    cur_ts;
    ACoSTConfig    cfg;

    void init(double eps);
    void init(double eps, const ACoSTConfig &config);

    /**
     * Decompress one GPS point.
     * @return true on success.
     */
    bool decompress(ACoSTInBits &in, double &lon, double &lat, uint64_t &ts_ms);

private:
    void    push_hist(int64_t ql, int64_t qa, uint64_t ts);
    int64_t decode_value(ACoSTInBits &in);
    void    decode_residual(ACoSTInBits &in, int64_t &rl, int64_t &ra);
};
