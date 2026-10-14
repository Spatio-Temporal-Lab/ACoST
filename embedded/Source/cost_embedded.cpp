/**
 * cost_embedded.cpp
 *
 * Embedded port of algorithm/cost_compressor.cc for STM32F103.
 * Preserves: LDR/CP/ZP parallel prediction, Huffman flag encoding,
 * cost-based mode switching, ZigZag+EliasGamma residual coding.
 */

#include "cost_embedded.h"

/* ── Helpers ───────────────────────────────────────────────────────────── */

static inline uint64_t c_d2bits(double v) {
    uint64_t b; memcpy(&b, &v, 8); return b;
}
static inline double c_bits2d(uint64_t b) {
    double v; memcpy(&v, &b, 8); return v;
}

static inline int c_ilog2(uint64_t n) {
    int r = 0;
    while (n > 1) { n >>= 1; ++r; }
    return r;
}

static inline int64_t c_zz_enc(int64_t v) { return (v << 1) ^ (v >> 63); }
static inline int64_t c_zz_dec(int64_t v) { return (v >> 1) ^ -(v & 1);  }

static int c_eg_encode(int64_t number, CostOutBits &out) {
    if (number < 1) number = 1;
    int n = c_ilog2(static_cast<uint64_t>(number));
    if (n > 0) out.write(0ULL, n);
    out.write(static_cast<uint64_t>(number), n + 1);
    return 2 * n + 1;
}

static int64_t c_eg_decode(CostInBits &in) {
    int n = 0;
    while (in.read_bit() == 0) {
        ++n;
        if (n > 63) return 1;
    }
    if (n == 0) return 1LL;
    return static_cast<int64_t>((1ULL << n) | in.read(n));
}

/* ── CostOutBits ───────────────────────────────────────────────────────── */

void CostOutBits::reset() {
    memset(buf, 0, sizeof(buf));
    bit_count = 0;
}

void CostOutBits::write(uint64_t val, int len) {
    for (int i = len - 1; i >= 0; --i) {
        int bit = static_cast<int>((val >> i) & 1u);
        int byte_idx = bit_count >> 3;
        int bit_idx  = 7 - (bit_count & 7);
        if (byte_idx < COST_EMB_BUF_BYTES)
            buf[byte_idx] |= static_cast<uint8_t>(bit << bit_idx);
        ++bit_count;
    }
}

void CostOutBits::write_bit(bool b) {
    write(b ? 1ULL : 0ULL, 1);
}

/* ── CostInBits ────────────────────────────────────────────────────────── */

void CostInBits::init(const uint8_t *b, int bits) {
    buf = b; total_bits = bits; pos = 0;
}

uint64_t CostInBits::read(int len) {
    uint64_t result = 0;
    for (int i = 0; i < len; ++i) {
        if (pos >= total_bits) {
            result <<= 1;
        } else {
            int byte_idx = pos >> 3;
            int bit_idx  = 7 - (pos & 7);
            result = (result << 1) | ((buf[byte_idx] >> bit_idx) & 1u);
        }
        ++pos;
    }
    return result;
}

uint32_t CostInBits::read_bit() {
    return static_cast<uint32_t>(read(1));
}

/* ══════════════════════════════════════════════════════════════════════════
 *  CostCompressor
 * ══════════════════════════════════════════════════════════════════════════ */

void CostCompressor::init(double eps) {
#ifdef E2E_TIMESTAMP_DOD
    ts_coder.init();
#endif
    epsilon    = eps * 0.999;
    quant_step = 2.0 * eps * 0.999;
    point_count = 0;
    hist_count  = 0;
    mode        = COST_MODE_MULTI;
    eval_window = COST_EMB_EVAL_WINDOW;
    total_data_bits = 0;

    memset(&cur_pt, 0, sizeof(cur_pt));
    memset(hist, 0, sizeof(hist));

    /* Huffman init: LDR dominant */
    huff_freq[COST_LDR] = 60;
    huff_freq[COST_CP]  = 10;
    huff_freq[COST_ZP]  = 30;
    huff_total = 100;
    huff_rebuild();

    /* Cost windows */
    cost_head = cost_count = 0;
    cost_sum_multi = cost_sum_ldr = 0;
    memset(cost_multi, 0, sizeof(cost_multi));
    memset(cost_ldr, 0, sizeof(cost_ldr));
}

/* ── History ─────────────────────────────────────────────────────────── */

void CostCompressor::push_hist(const CostGpsPoint &rp) {
    double v_lon = 0.0, v_lat = 0.0;
    if (hist_count > 0) {
        const CostHistState &prev = hist[hist_count - 1];
        int64_t dts = static_cast<int64_t>(rp.ts) -
                      static_cast<int64_t>(prev.pt.ts);
        double dt = dts > 0 ? static_cast<double>(dts) : 1.0;
        v_lon = (rp.lon - prev.pt.lon) / dt;
        v_lat = (rp.lat - prev.pt.lat) / dt;
    }

    if (hist_count < COST_EMB_MAX_HIST) {
        hist[hist_count].pt    = rp;
        hist[hist_count].v_lon = v_lon;
        hist[hist_count].v_lat = v_lat;
        ++hist_count;
    } else {
        for (int i = 0; i < COST_EMB_MAX_HIST - 1; ++i)
            hist[i] = hist[i + 1];
        hist[COST_EMB_MAX_HIST - 1].pt    = rp;
        hist[COST_EMB_MAX_HIST - 1].v_lon = v_lon;
        hist[COST_EMB_MAX_HIST - 1].v_lat = v_lat;
    }
}

/* ── Prediction ──────────────────────────────────────────────────────── */

void CostCompressor::predict(uint64_t ts,
                              CostGpsPoint &ldr, CostGpsPoint &cp,
                              CostGpsPoint &zp) {
    /* ZP: repeat current */
    zp = cur_pt;

    if (hist_count < 2) {
        ldr = cur_pt;
        cp  = cur_pt;
        return;
    }

    int64_t dts = static_cast<int64_t>(ts) -
                  static_cast<int64_t>(cur_pt.ts);
    double dt = dts > 0 ? static_cast<double>(dts) : 1.0;

    /* LDR: linear extrapolation */
    double vl = hist[hist_count - 1].v_lon;
    double va = hist[hist_count - 1].v_lat;
    ldr.lon = cur_pt.lon + vl * dt;
    ldr.lat = cur_pt.lat + va * dt;
    ldr.ts  = ts;

    /* CP: quadratic extrapolation */
    if (hist_count >= 3 && dt > 0.0) {
        double pv_lon = hist[hist_count - 2].v_lon;
        double pv_lat = hist[hist_count - 2].v_lat;
        double a_lon = vl - pv_lon;
        double a_lat = va - pv_lat;
        double dt2h = dt * dt * 0.5;
        cp.lon = cur_pt.lon + vl * dt + a_lon * dt2h;
        cp.lat = cur_pt.lat + va * dt + a_lat * dt2h;
        cp.ts  = ts;
    } else {
        cp = ldr;
    }
}

/* ── Huffman ─────────────────────────────────────────────────────────── */

void CostCompressor::huff_rebuild() {
    /* Sort 3 predictors by frequency desc; ties broken by enum order */
    huff_order[0] = 0; huff_order[1] = 1; huff_order[2] = 2;
    /* Simple bubble sort for 3 elements */
    for (int i = 0; i < 2; ++i)
        for (int j = i + 1; j < 3; ++j)
            if (huff_freq[huff_order[j]] > huff_freq[huff_order[i]])
                { int tmp = huff_order[i]; huff_order[i] = huff_order[j]; huff_order[j] = tmp; }
}

int CostCompressor::huff_bit_cost(CostPredType p) {
    if (p == huff_order[0]) return 1;  /* code = 0 */
    return 2;                           /* code = 10 or 11 */
}

void CostCompressor::encode_huff(CostPredType p, CostOutBits &out) {
    if (p == huff_order[0]) {
        out.write_bit(false);  /* 0 */
    } else if (p == huff_order[1]) {
        out.write_bit(true);
        out.write_bit(false);  /* 10 */
    } else {
        out.write_bit(true);
        out.write_bit(true);   /* 11 */
    }
    huff_add(p);
}

void CostCompressor::huff_add(CostPredType p) {
    huff_freq[p]++;
    huff_total++;
    if (huff_total % 100 == 0) huff_rebuild();
}

/* ── Cost estimation ─────────────────────────────────────────────────── */

int CostCompressor::est_eg_bits(int64_t v) {
    if (v <= 0) return 1;
    int n = c_ilog2(static_cast<uint64_t>(v));
    return 2 * n + 1;
}

int CostCompressor::est_residual_cost(double err_lon, double err_lat) {
    int64_t ql = static_cast<int64_t>(std::round(err_lon / quant_step));
    int64_t qa = static_cast<int64_t>(std::round(err_lat / quant_step));
    return est_eg_bits(static_cast<int64_t>(
               static_cast<uint64_t>(c_zz_enc(ql)) + 1))
         + est_eg_bits(static_cast<int64_t>(
               static_cast<uint64_t>(c_zz_enc(qa)) + 1));
}

/* ── Cost window ─────────────────────────────────────────────────────── */

void CostCompressor::cost_push(int mc, int lc) {
    int idx;
    if (cost_count < COST_EMB_COST_WINDOW) {
        idx = (cost_head + cost_count) % COST_EMB_COST_WINDOW;
        cost_count++;
    } else {
        /* evict oldest */
        cost_sum_multi -= cost_multi[cost_head];
        cost_sum_ldr   -= cost_ldr[cost_head];
        idx = cost_head;
        cost_head = (cost_head + 1) % COST_EMB_COST_WINDOW;
    }
    cost_multi[idx] = mc;
    cost_ldr[idx]   = lc;
    cost_sum_multi += mc;
    cost_sum_ldr   += lc;
}

void CostCompressor::cost_eval() {
    if (cost_count < eval_window) return;
    if (mode == COST_MODE_MULTI) {
        if (cost_sum_ldr < cost_sum_multi - 1)
            mode = COST_MODE_LDR_ONLY;
    } else {
        if (cost_sum_multi < cost_sum_ldr - 1)
            mode = COST_MODE_MULTI;
    }
}

/* ── Encoding helpers ────────────────────────────────────────────────── */

int CostCompressor::encode_value(uint64_t v, CostOutBits &out) {
    return c_eg_encode(static_cast<int64_t>(v), out);
}

int CostCompressor::encode_residual(double err_lon, double err_lat,
                                     CostOutBits &out) {
    int64_t ql = static_cast<int64_t>(std::round(err_lon / quant_step));
    int64_t qa = static_cast<int64_t>(std::round(err_lat / quant_step));
    int t = 0;
    t += encode_value(static_cast<uint64_t>(c_zz_enc(ql)) + 1ULL, out);
    t += encode_value(static_cast<uint64_t>(c_zz_enc(qa)) + 1ULL, out);
    return t;
}

/* ── Main compress ───────────────────────────────────────────────────── */

int CostCompressor::compress(double lon, double lat, uint64_t ts_ms,
                              CostOutBits &out) {
    out.reset();

    if (point_count == 0) {
        /* ── Header ──────────────────────────────────────── */
        out.write(1ULL, 16);                    /* block_size */
        out.write(c_d2bits(epsilon), 64);       /* epsilon */
        out.write(static_cast<uint64_t>(eval_window), 16);
        out.write_bit(false);                   /* use_time_window = false */

        out.write(c_d2bits(lon), 64);
        out.write(c_d2bits(lat), 64);
#ifdef E2E_TIMESTAMP_DOD
        if (!e2e_ts_write(out, ts_coder, ts_ms)) return -1;
#else
        out.write(ts_ms, 64);
#endif

        cur_pt.lon = lon; cur_pt.lat = lat; cur_pt.ts = ts_ms;
        CostGpsPoint p = {lon, lat, ts_ms};
        push_hist(p);
        point_count = 1;
        return out.bit_count;
    }

    /* ── Parallel predict ────────────────────────────────── */
    CostGpsPoint pred_ldr, pred_cp, pred_zp;
    predict(ts_ms, pred_ldr, pred_cp, pred_zp);

    /* ── Cost estimation ─────────────────────────────────── */
    int c_ldr = est_residual_cost(lon - pred_ldr.lon, lat - pred_ldr.lat);
    int c_cp  = est_residual_cost(lon - pred_cp.lon,  lat - pred_cp.lat);
    int c_zp  = est_residual_cost(lon - pred_zp.lon,  lat - pred_zp.lat);

    int ct_ldr = huff_bit_cost(COST_LDR) + c_ldr;
    int ct_cp  = huff_bit_cost(COST_CP)  + c_cp;
    int ct_zp  = huff_bit_cost(COST_ZP)  + c_zp;

    CostPredType best = COST_LDR;
    CostGpsPoint best_pred = pred_ldr;
    int best_cost = ct_ldr;

    if (ct_cp < best_cost) { best = COST_CP; best_pred = pred_cp; best_cost = ct_cp; }
    if (ct_zp < best_cost) { best = COST_ZP; best_pred = pred_zp; best_cost = ct_zp; }

    /* Cost windows */
    cost_push(best_cost, c_ldr);

    /* ── Encode ──────────────────────────────────────────── */
    if (mode == COST_MODE_LDR_ONLY) {
        /* LDR-Only: no Huffman flag, just ts_delta + residual */
#ifdef E2E_TIMESTAMP_DOD
        if (!e2e_ts_write(out, ts_coder, ts_ms)) return -1;
#else
        int64_t ts_delta = static_cast<int64_t>(ts_ms) -
                           static_cast<int64_t>(cur_pt.ts);
        out.write(static_cast<uint64_t>(ts_delta), 64);
#endif

        int data_bits = encode_residual(lon - pred_ldr.lon, lat - pred_ldr.lat, out);
        total_data_bits += static_cast<uint32_t>(data_bits);

        /* Reconstruct */
        int64_t ql = static_cast<int64_t>(std::round((lon - pred_ldr.lon) / quant_step));
        int64_t qa = static_cast<int64_t>(std::round((lat - pred_ldr.lat) / quant_step));
        CostGpsPoint rp;
        rp.lon = pred_ldr.lon + ql * quant_step;
        rp.lat = pred_ldr.lat + qa * quant_step;
        rp.ts  = ts_ms;
        push_hist(rp);
        cur_pt = rp;
    } else {
        /* Multi-Predictor: Huffman flag + ts_delta + residual */
        encode_huff(best, out);

#ifdef E2E_TIMESTAMP_DOD
        if (!e2e_ts_write(out, ts_coder, ts_ms)) return -1;
#else
        int64_t ts_delta = static_cast<int64_t>(ts_ms) -
                           static_cast<int64_t>(cur_pt.ts);
        out.write(static_cast<uint64_t>(ts_delta), 64);
#endif

        int data_bits = encode_residual(lon - best_pred.lon, lat - best_pred.lat, out);
        total_data_bits += static_cast<uint32_t>(data_bits);

        /* Reconstruct */
        int64_t ql = static_cast<int64_t>(std::round((lon - best_pred.lon) / quant_step));
        int64_t qa = static_cast<int64_t>(std::round((lat - best_pred.lat) / quant_step));
        CostGpsPoint rp;
        rp.lon = best_pred.lon + ql * quant_step;
        rp.lat = best_pred.lat + qa * quant_step;
        rp.ts  = ts_ms;
        push_hist(rp);
        cur_pt = rp;
    }

    ++point_count;

    /* Mode evaluation every eval_window points */
    if (point_count % eval_window == 0) {
        cost_eval();
        out.write_bit(mode == COST_MODE_LDR_ONLY);
    }

    return out.bit_count;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  CostDecompressor
 * ══════════════════════════════════════════════════════════════════════════ */

void CostDecompressor::init(double eps) {
#ifdef E2E_TIMESTAMP_DOD
    ts_coder.init();
#endif
    epsilon    = eps * 0.999;
    quant_step = 2.0 * eps * 0.999;
    point_count = 0;
    hist_count  = 0;
    mode        = COST_MODE_MULTI;
    eval_window = COST_EMB_EVAL_WINDOW;

    memset(&cur_pt, 0, sizeof(cur_pt));
    memset(hist, 0, sizeof(hist));

    huff_freq[COST_LDR] = 60;
    huff_freq[COST_CP]  = 10;
    huff_freq[COST_ZP]  = 30;
    huff_total = 100;
    huff_rebuild();
}

void CostDecompressor::push_hist(const CostGpsPoint &rp) {
    double v_lon = 0.0, v_lat = 0.0;
    if (hist_count > 0) {
        const CostHistState &prev = hist[hist_count - 1];
        int64_t dts = static_cast<int64_t>(rp.ts) -
                      static_cast<int64_t>(prev.pt.ts);
        double dt = dts > 0 ? static_cast<double>(dts) : 1.0;
        v_lon = (rp.lon - prev.pt.lon) / dt;
        v_lat = (rp.lat - prev.pt.lat) / dt;
    }

    if (hist_count < COST_EMB_MAX_HIST) {
        hist[hist_count].pt    = rp;
        hist[hist_count].v_lon = v_lon;
        hist[hist_count].v_lat = v_lat;
        ++hist_count;
    } else {
        for (int i = 0; i < COST_EMB_MAX_HIST - 1; ++i)
            hist[i] = hist[i + 1];
        hist[COST_EMB_MAX_HIST - 1].pt    = rp;
        hist[COST_EMB_MAX_HIST - 1].v_lon = v_lon;
        hist[COST_EMB_MAX_HIST - 1].v_lat = v_lat;
    }
}

void CostDecompressor::predict(uint64_t ts,
                                CostGpsPoint &ldr, CostGpsPoint &cp,
                                CostGpsPoint &zp) {
    zp = cur_pt;
    if (hist_count < 2) { ldr = cur_pt; cp = cur_pt; return; }

    int64_t dts = static_cast<int64_t>(ts) -
                  static_cast<int64_t>(cur_pt.ts);
    double dt = dts > 0 ? static_cast<double>(dts) : 1.0;

    double vl = hist[hist_count - 1].v_lon;
    double va = hist[hist_count - 1].v_lat;
    ldr.lon = cur_pt.lon + vl * dt;
    ldr.lat = cur_pt.lat + va * dt;
    ldr.ts  = ts;

    if (hist_count >= 3 && dt > 0.0) {
        double pv_lon = hist[hist_count - 2].v_lon;
        double pv_lat = hist[hist_count - 2].v_lat;
        double dt2h = dt * dt * 0.5;
        cp.lon = cur_pt.lon + vl * dt + (vl - pv_lon) * dt2h;
        cp.lat = cur_pt.lat + va * dt + (va - pv_lat) * dt2h;
        cp.ts  = ts;
    } else {
        cp = ldr;
    }
}

void CostDecompressor::huff_rebuild() {
    huff_order[0] = 0; huff_order[1] = 1; huff_order[2] = 2;
    for (int i = 0; i < 2; ++i)
        for (int j = i + 1; j < 3; ++j)
            if (huff_freq[huff_order[j]] > huff_freq[huff_order[i]])
                { int tmp = huff_order[i]; huff_order[i] = huff_order[j]; huff_order[j] = tmp; }
}

CostPredType CostDecompressor::decode_huff(CostInBits &in) {
    uint32_t b = in.read_bit();
    CostPredType p;
    if (b == 0) {
        p = static_cast<CostPredType>(huff_order[0]);
    } else {
        uint32_t b2 = in.read_bit();
        p = b2 == 0 ? static_cast<CostPredType>(huff_order[1])
                     : static_cast<CostPredType>(huff_order[2]);
    }
    huff_add(p);
    return p;
}

void CostDecompressor::huff_add(CostPredType p) {
    huff_freq[p]++;
    huff_total++;
    if (huff_total % 100 == 0) huff_rebuild();
}

int64_t CostDecompressor::decode_value(CostInBits &in) {
    return c_eg_decode(in);
}

bool CostDecompressor::decompress(CostInBits &in,
                                   double &lon, double &lat, uint64_t &ts_ms) {
    if (point_count == 0) {
        /* ── Header ──────────────────────────────────────── */
        in.read(16);                            /* block_size */
        double hdr_eps = c_bits2d(in.read(64));
        quant_step = 2.0 * hdr_eps;
        eval_window = static_cast<int>(in.read(16));
        in.read_bit();                          /* use_time_window (false) */

        lon   = c_bits2d(in.read(64));
        lat   = c_bits2d(in.read(64));
#ifdef E2E_TIMESTAMP_DOD
        if (!e2e_ts_read(in, ts_coder, &ts_ms)) return false;
#else
        ts_ms = in.read(64);
#endif

        cur_pt.lon = lon; cur_pt.lat = lat; cur_pt.ts = ts_ms;
        CostGpsPoint p = {lon, lat, ts_ms};
        push_hist(p);
        point_count = 1;
        return true;
    }

    /* ── Incremental point ────────────────────────────────── */
    CostGpsPoint pred_ldr, pred_cp, pred_zp;

    CostGpsPoint *chosen = &pred_ldr;

    if (mode == COST_MODE_LDR_ONLY) {
        /* Read ts_delta + residual (no Huffman flag) */
#ifdef E2E_TIMESTAMP_DOD
        uint64_t ts;
        if (!e2e_ts_read(in, ts_coder, &ts)) return false;
#else
        uint64_t ts_delta = in.read(64);
        uint64_t ts = static_cast<uint64_t>(
            static_cast<int64_t>(cur_pt.ts) + static_cast<int64_t>(ts_delta));
#endif

        predict(ts, pred_ldr, pred_cp, pred_zp);
        chosen = &pred_ldr;
        ts_ms = ts;
    } else {
        /* Multi-Predictor: Huffman flag + ts_delta + residual */
        CostPredType pred_type = decode_huff(in);

#ifdef E2E_TIMESTAMP_DOD
        uint64_t ts;
        if (!e2e_ts_read(in, ts_coder, &ts)) return false;
#else
        uint64_t ts_delta = in.read(64);
        uint64_t ts = static_cast<uint64_t>(
            static_cast<int64_t>(cur_pt.ts) + static_cast<int64_t>(ts_delta));
#endif

        predict(ts, pred_ldr, pred_cp, pred_zp);

        switch (pred_type) {
        case COST_LDR: chosen = &pred_ldr; break;
        case COST_CP:  chosen = &pred_cp;  break;
        case COST_ZP:  chosen = &pred_zp;  break;
        }
        ts_ms = ts;
    }

    /* Decode residual */
    int64_t el = decode_value(in);
    int64_t ea = decode_value(in);
    int64_t rl = c_zz_dec(el - 1LL);
    int64_t ra = c_zz_dec(ea - 1LL);

    CostGpsPoint rp;
    rp.lon = chosen->lon + rl * quant_step;
    rp.lat = chosen->lat + ra * quant_step;
    rp.ts  = ts_ms;

    push_hist(rp);
    cur_pt = rp;
    ++point_count;

    /* Mode evaluation */
    if (point_count % eval_window == 0) {
        uint32_t mode_bit = in.read_bit();
        mode = mode_bit ? COST_MODE_LDR_ONLY : COST_MODE_MULTI;
    }

    lon = rp.lon;
    lat = rp.lat;
    return true;
}
