/**
 * cost_embedded.h
 *
 * Embedded-friendly (no STL, no dynamic memory) streaming implementation
 * of the CoST compressor and decompressor for STM32F103.
 *
 * CoST key features (all preserved):
 *   1. Three parallel predictors: LDR (Linear Dead Reckoning),
 *      CP (Curve Predictor / quadratic), ZP (Zero Predictor)
 *   2. Cost-based predictor selection (Huffman flag + residual cost)
 *   3. Adaptive Huffman coding for predictor flags (3-symbol: 0/10/11)
 *   4. Mode switching: Multi-Predictor vs LDR-Only
 *   5. ZigZag + Elias-Gamma residual encoding
 *
 * Wire format (MSB-first bit stream):
 *   Header:  [block_size 16b][epsilon 64b][eval_window 16b]
 *            [use_time_window 1b][time_window_seconds? 32b]
 *            [lon 64b][lat 64b][ts 64b]
 *   Multi-Predictor point: [huffman_flag 1-2b][ts_delta 64b][res_lon EG][res_lat EG]
 *   LDR-Only point:        [ts_delta 64b][res_lon EG][res_lat EG]
 *   Mode eval (every N pts): [mode_bit 1b]
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>
#ifdef E2E_TIMESTAMP_DOD
#include "timestamp_dod.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Configuration ────────────────────────────────────────────────────── */

#define COST_EMB_BUF_BYTES     64
#define COST_EMB_MAX_HIST       3
#define COST_EMB_EVAL_WINDOW   96
#define COST_EMB_HUFF_WINDOW  256   /* sliding window for Huffman stats */
#define COST_EMB_COST_WINDOW  96   /* ring buffer for cost evaluation (= eval_window) */

/* ── Reuse the same bit-stream types from acost_embedded.h ─────────── */
/* If both headers are included, these are already defined.  Use
 * separate names to avoid ODR issues. */

struct CostOutBits {
    uint8_t buf[COST_EMB_BUF_BYTES];
    int     bit_count;

    void reset();
    void write(uint64_t val, int len);
    void write_bit(bool b);
    int  byte_count() const { return (bit_count + 7) / 8; }
};

struct CostInBits {
    const uint8_t *buf;
    int            total_bits;
    int            pos;

    void     init(const uint8_t *b, int bits);
    uint64_t read(int len);
    uint32_t read_bit();
};

/* ── Types ────────────────────────────────────────────────────────────── */

enum CostPredType { COST_LDR = 0, COST_CP = 1, COST_ZP = 2 };
enum CostMode     { COST_MODE_MULTI = 0, COST_MODE_LDR_ONLY = 1 };

struct CostGpsPoint {
    double   lon, lat;
    uint64_t ts;
};

struct CostHistState {
    CostGpsPoint pt;
    double       v_lon, v_lat;   /* velocity (degrees/time-unit) */
};

/* ── Compressor ───────────────────────────────────────────────────────── */

struct CostCompressor {
#ifdef E2E_TIMESTAMP_DOD
    E2ETimestampDoD ts_coder;
#endif
    double   epsilon;
    double   quant_step;
    int      point_count;

    CostGpsPoint cur_pt;
    CostHistState hist[COST_EMB_MAX_HIST];
    int       hist_count;

    CostMode  mode;
    int       eval_window;

    /* Huffman: freq[3] and fixed 3-symbol code {0, 10, 11} sorted by freq */
    int       huff_freq[3];
    int       huff_order[3]; /* huff_order[rank] = PredType; rank0 = most freq */
    int       huff_total;

    /* Cost evaluation ring buffers */
    int       cost_multi[COST_EMB_COST_WINDOW];
    int       cost_ldr[COST_EMB_COST_WINDOW];
    int       cost_head, cost_count;
    long long cost_sum_multi, cost_sum_ldr;

    /* Running stat: total compressed data bits (excl timestamps) */
    uint32_t  total_data_bits;

    void init(double eps);

    /**
     * Compress one GPS point.
     * @return Number of bits written into out.
     */
    int compress(double lon, double lat, uint64_t ts_ms, CostOutBits &out);

private:
    void push_hist(const CostGpsPoint &rp);
    void predict(uint64_t ts, CostGpsPoint &ldr, CostGpsPoint &cp, CostGpsPoint &zp);
    int  est_eg_bits(int64_t v);
    int  est_residual_cost(double err_lon, double err_lat);
    int  huff_bit_cost(CostPredType p);
    void huff_add(CostPredType p);
    void huff_rebuild();
    int  encode_residual(double err_lon, double err_lat, CostOutBits &out);
    int  encode_value(uint64_t v, CostOutBits &out);
    void encode_huff(CostPredType p, CostOutBits &out);
    void cost_push(int mc, int lc);
    void cost_eval();
};

/* ── Decompressor ─────────────────────────────────────────────────────── */

struct CostDecompressor {
#ifdef E2E_TIMESTAMP_DOD
    E2ETimestampDoD ts_coder;
#endif
    double   epsilon;
    double   quant_step;
    int      point_count;
    int      eval_window;

    CostGpsPoint cur_pt;
    CostHistState hist[COST_EMB_MAX_HIST];
    int       hist_count;

    CostMode  mode;

    /* Huffman decoder: decode_map[rank] = PredType */
    int       huff_freq[3];
    int       huff_order[3];
    int       huff_total;

    void init(double eps);

    /**
     * Decompress one GPS point.
     * @return true on success.
     */
    bool decompress(CostInBits &in, double &lon, double &lat, uint64_t &ts_ms);

private:
    void push_hist(const CostGpsPoint &rp);
    void predict(uint64_t ts, CostGpsPoint &ldr, CostGpsPoint &cp, CostGpsPoint &zp);
    int64_t decode_value(CostInBits &in);
    CostPredType decode_huff(CostInBits &in);
    void huff_add(CostPredType p);
    void huff_rebuild();
};
