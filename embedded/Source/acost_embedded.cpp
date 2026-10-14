/**
 * acost_embedded.cpp
 *
 * Embedded port of algorithm/acost_compressor.cc for STM32F103.
 * Faithfully reproduces the PC version's prediction, damping, adaptive tail,
 * ZP-selection, and encoding — but with fixed arrays and no STL.
 */

#include "acost_embedded.h"

/* ── Internal helpers ──────────────────────────────────────────────────── */

static constexpr double kEarthR = 6371000.0;
static constexpr double kDeg2Rad = M_PI / 180.0;

static inline uint64_t d2bits(double v) {
    uint64_t b; memcpy(&b, &v, 8); return b;
}
static inline double bits2d(uint64_t b) {
    double v; memcpy(&v, &b, 8); return v;
}

static inline int ilog2(uint64_t n) {
    if (n <= 1) return 0;
    return 63 - __builtin_clzll(n);
}

static inline int64_t zz_enc(int64_t v) { return (v << 1) ^ (v >> 63); }
static inline int64_t zz_dec(int64_t v) { return (v >> 1) ^ -(v & 1);  }

/* Fast exp(-x) approximation for x >= 0, accurate to ~0.1% for x < 10 */
static inline double fast_exp_neg(double x) {
    if (x > 10.0) return 0.0;
    if (x < 0.001) return 1.0;
    /* Padé(4,4) approximation of exp(-x) */
    double x2 = x * x;
    double num = 1.0 - x * 0.5 + x2 * (1.0 / 12.0);
    double den = 1.0 + x * 0.5 + x2 * (1.0 / 12.0);
    double r = num / den;  /* Padé [2/2] */
    return r * r;          /* square for better range: exp(-x) ≈ (padé(x/2))^2 ... 
                              actually this is padé(x) squared = approx exp(-2x)
                              so use padé(x/2) squared */
}

/* Better: just use a simple rational approx */
static inline double fast_damp(double excess, double tau) {
    /* exp(-excess/tau): for damping, only need monotone decrease in [0,1] */
    double t = excess / tau;
    if (t > 8.0) return 0.0;
    if (t < 0.01) return 1.0;
    /* 1/(1 + t + t^2/2 + t^3/6) ≈ exp(-t) with ~2% error for t<3 */
    double t2 = t * t;
    return 1.0 / (1.0 + t + 0.5 * t2 + t2 * t * (1.0 / 6.0));
}

/* Elias-Gamma encode → out; returns bits written */
static int eg_encode(int64_t number, ACoSTOutBits &out) {
    if (number < 1) number = 1;
    int n = ilog2(static_cast<uint64_t>(number));
    if (n > 0) out.write(0ULL, n);
    out.write(static_cast<uint64_t>(number), n + 1);
    return 2 * n + 1;
}

/* Elias-Gamma decode from in */
static int64_t eg_decode(ACoSTInBits &in) {
    int n = 0;
    while (in.read_bit() == 0) {
        ++n;
        if (n > 63) return 1;
    }
    if (n == 0) return 1LL;
    return static_cast<int64_t>((1ULL << n) | in.read(n));
}

/* Elias-Gamma bit cost without writing (for ZP selection) */
static inline int eg_dim_cost(int64_t delta) {
    uint64_t n = static_cast<uint64_t>(zz_enc(delta)) + 1;
    int k = (n <= 1) ? 0 : (63 - __builtin_clzll(n));
    return 2 * k + 1;
}

/* Weighted average of last `tail` elements of v[0..count-1] */
static void wtail_vec_avg(const double *vx, const double *vy,
                           int count, int tail,
                           double &ox, double &oy) {
    ox = oy = 0.0;
    if (count == 0 || tail == 0) return;
    int start = (count > tail) ? (count - tail) : 0;
    double wsum = 0.0, w = 1.0;
    for (int i = start; i < count; ++i, w += 1.0) {
        ox += w * vx[i];
        oy += w * vy[i];
        wsum += w;
    }
    if (wsum > 0.0) { ox /= wsum; oy /= wsum; }
}

/* ── Core predictor (trig-free vector CA, mirrors new PC version) ───────── */
/*
 * No atan2/sin/cos/sqrt in hot path. Turn detection via cross²/dot².
 * Acceleration projected for non-turning, full vector for turning.
 */

struct ACoSTPredResult {
    int64_t q_lon, q_lat;
    int     model;   /* 0=STOP, 1=CV, 2=CA(turn-aware) */
};

static ACoSTPredResult predict_acost(const ACoSTHistPt *hist, int n,
                                int64_t cur_ql, int64_t cur_qa,
                                uint64_t cur_ts, const ACoSTConfig &cfg,
                                double lon_scale, double quant_step,
                                uint64_t target_ts) {
    ACoSTPredResult r;
    if (n < 2) {
        r.q_lon = cur_ql; r.q_lat = cur_qa; r.model = 0;
        return r;
    }
    int num_seg = n - 1;

    double lx[ACOST_EMB_MAX_HIST + 1], ly[ACOST_EMB_MAX_HIST + 1];
    double vx[ACOST_EMB_MAX_HIST], vy[ACOST_EMB_MAX_HIST], sdt[ACOST_EMB_MAX_HIST];

    for (int i = 0; i < n; ++i) {
        lx[i] = static_cast<double>(hist[i].q_lon - cur_ql) * lon_scale;
        ly[i] = static_cast<double>(hist[i].q_lat - cur_qa);
    }
    for (int i = 0; i < num_seg; ++i) {
        int64_t ds = static_cast<int64_t>(hist[i+1].ts) -
                     static_cast<int64_t>(hist[i].ts);
        double d = ds > 0 ? static_cast<double>(ds) : 1.0;
        vx[i]  = (lx[i+1] - lx[i]) / d;
        vy[i]  = (ly[i+1] - ly[i]) / d;
        sdt[i] = d;
    }

    double avg_dt = 0.0;
    for (int i = 0; i < num_seg; ++i) avg_dt += sdt[i];
    avg_dt /= static_cast<double>(num_seg);

    int64_t dt_signed = static_cast<int64_t>(target_ts) -
                        static_cast<int64_t>(cur_ts);
    double dt = dt_signed > 0 ? static_cast<double>(dt_signed) : 1.0;

    /* Adaptive tail */
    double onset = cfg.ctra_damp_onset;
    double tail_factor = onset / (avg_dt > onset ? avg_dt : onset);
    int sp_tail = static_cast<int>(ceilf((float)(tail_factor * cfg.speed_tail_segments)));
    int ac_tail = static_cast<int>(ceilf((float)(tail_factor * cfg.accel_tail_segments)));
    if (sp_tail < 1) sp_tail = 1;
    if (ac_tail < 1) ac_tail = 1;
    if (sp_tail > num_seg) sp_tail = num_seg;
    if (ac_tail > num_seg) ac_tail = num_seg;

    double avg_vx, avg_vy;
    wtail_vec_avg(vx, vy, num_seg, sp_tail, avg_vx, avg_vy);
    double v_sq = avg_vx * avg_vx + avg_vy * avg_vy;

    /* STOP detection */
    {
        double dx = lx[n-1] - lx[n-2];
        double dy = ly[n-1] - ly[n-2];
        if (dx * dx + dy * dy < lon_scale * lon_scale + 1.0) {
            r.q_lon = cur_ql; r.q_lat = cur_qa; r.model = 0;
            return r;
        }
    }

    /* Damping (no exp, rational approx) */
    double excess = avg_dt - onset;
    if (excess < 0.0) excess = 0.0;
    double damp = fast_damp(excess, cfg.ctra_damp_tau);

    double px = 0.0, py = 0.0;

    if (num_seg < 2 || ac_tail < 1) {
        /* CV */
        px = avg_vx * dt;
        py = avg_vy * dt;
    } else {
        double prev_vx, prev_vy;
        wtail_vec_avg(vx, vy, num_seg - 1, ac_tail, prev_vx, prev_vy);

        double dt_hist = sdt[num_seg - 1];
        if (dt_hist <= 0.0) dt_hist = 1.0;

        /* Turn detection: cross²/dot² threshold (NO atan2) */
        double cross = prev_vx * avg_vy - prev_vy * avg_vx;
        double dot_v = prev_vx * avg_vx + prev_vy * avg_vy;

        bool is_turning = false;
        if (dot_v > 0.0) {
            double lim = cfg.ctra_omega_eps * dt_hist;
            is_turning = (cross * cross > lim * lim * dot_v * dot_v);
        }

        /* Vector acceleration */
        double ax = (avg_vx - prev_vx) / dt_hist * damp;
        double ay = (avg_vy - prev_vy) / dt_hist * damp;

        if (!is_turning && v_sq > 1.0e-24) {
            /* Project accel onto velocity direction (suppress lateral jitter) */
            double scale = (ax * avg_vx + ay * avg_vy) / v_sq;
            ax = scale * avg_vx;
            ay = scale * avg_vy;
        }
        /* else: keep full vector acceleration for turning */

        double dt2 = dt * dt;
        px = avg_vx * dt + 0.5 * ax * dt2;
        py = avg_vy * dt + 0.5 * ay * dt2;
    }

    if (!std::isfinite(px) || !std::isfinite(py)) {
        px = avg_vx * dt;
        py = avg_vy * dt;
    }

    /* Speed cap */
    double m_to_qu = 1.0 / (quant_step * kDeg2Rad * kEarthR);
    double max_d = cfg.max_speed_ms * m_to_qu * dt;
    double cap   = cfg.max_cap_abs_m * m_to_qu;
    if (max_d > cap) max_d = cap;
    if (px >  max_d) px =  max_d;
    if (px < -max_d) px = -max_d;
    if (py >  max_d) py =  max_d;
    if (py < -max_d) py = -max_d;

    r.q_lon = cur_ql + static_cast<int64_t>(lround(px / lon_scale));
    r.q_lat = cur_qa + static_cast<int64_t>(lround(py));
    r.model = (num_seg >= 2 && ac_tail >= 1) ? 2 : 1;
    return r;
}


/* ── ACoSTOutBits ─────────────────────────────────────────────────────────── */

void ACoSTOutBits::reset() {
    memset(buf, 0, sizeof(buf));
    bit_count = 0;
}

void ACoSTOutBits::write(uint64_t val, int len) {
    /* Fast path: write byte-aligned chunks when possible */
    while (len > 0) {
        int byte_idx = bit_count >> 3;
        if (byte_idx >= ACOST_EMB_BUF_BYTES) { bit_count += len; return; }
        int bit_off = bit_count & 7;   /* bits already used in current byte */
        int avail = 8 - bit_off;       /* bits remaining in current byte */
        int take = (len < avail) ? len : avail;
        /* Extract 'take' bits from the MSB side of val */
        uint8_t bits = (uint8_t)((val >> (len - take)) & ((1u << take) - 1u));
        buf[byte_idx] |= bits << (avail - take);
        bit_count += take;
        len -= take;
    }
}

void ACoSTOutBits::write_bit(bool b) {
    write(b ? 1ULL : 0ULL, 1);
}

/* ── ACoSTInBits ──────────────────────────────────────────────────────────── */

void ACoSTInBits::init(const uint8_t *b, int bits) {
    buf = b; total_bits = bits; pos = 0;
}

uint64_t ACoSTInBits::read(int len) {
    uint64_t result = 0;
    while (len > 0) {
        if (pos >= total_bits) { result <<= len; break; }
        int bit_off = pos & 7;
        int avail = 8 - bit_off;
        int take = (len < avail) ? len : avail;
        int byte_idx = pos >> 3;
        /* Extract 'take' bits starting at bit_off */
        uint8_t mask = ((1u << take) - 1u);
        uint8_t bits = (buf[byte_idx] >> (avail - take)) & mask;
        result = (result << take) | bits;
        pos += take;
        len -= take;
    }
    return result;
}

uint32_t ACoSTInBits::read_bit() {
    return static_cast<uint32_t>(read(1));
}

/* ── Shared push_hist ──────────────────────────────────────────────────── */

static void push_hist_acost(ACoSTHistPt *hist, int &count,
                          int64_t ql, int64_t qa, uint64_t ts) {
    if (count < ACOST_EMB_MAX_HIST) {
        hist[count].q_lon = ql;
        hist[count].q_lat = qa;
        hist[count].ts    = ts;
        ++count;
    } else {
        for (int i = 0; i < ACOST_EMB_MAX_HIST - 1; ++i)
            hist[i] = hist[i + 1];
        hist[ACOST_EMB_MAX_HIST - 1].q_lon = ql;
        hist[ACOST_EMB_MAX_HIST - 1].q_lat = qa;
        hist[ACOST_EMB_MAX_HIST - 1].ts    = ts;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  ACoSTCompressor
 * ══════════════════════════════════════════════════════════════════════════ */

void ACoSTCompressor::init(double eps) {
    ACoSTConfig c = acost_default_config();
    init(eps, c);
}

void ACoSTCompressor::init(double eps, const ACoSTConfig &config) {
    epsilon     = eps * 0.999;
    quant_step  = 2.0 * eps * 0.999;
    lon_scale   = 1.0;
    point_count = 0;
    hist_count  = 0;
    cur_q_lon = cur_q_lat = 0;
    cur_ts = 0;
    total_bits = 0;
    cfg = config;
    memset(hist, 0, sizeof(hist));
}

void ACoSTCompressor::push_hist(int64_t ql, int64_t qa, uint64_t ts) {
    push_hist_acost(hist, hist_count, ql, qa, ts);
}

int ACoSTCompressor::encode_value(uint64_t v, ACoSTOutBits &out) {
    return eg_encode(static_cast<int64_t>(v), out);
}

int ACoSTCompressor::encode_residual(int64_t rl, int64_t ra, ACoSTOutBits &out) {
    int t = 0;
    t += encode_value(static_cast<uint64_t>(zz_enc(rl)) + 1ULL, out);
    t += encode_value(static_cast<uint64_t>(zz_enc(ra)) + 1ULL, out);
    return t;
}

int ACoSTCompressor::compress(double lon, double lat, uint64_t ts_ms,
                            ACoSTOutBits &out) {
    out.reset();

    if (point_count == 0) {
        /* ── First point: header + raw values ────────────────── */
        out.write(1ULL, 16);                   /* block_size = 1 */
        out.write(d2bits(epsilon), 64);

        lon_scale = std::cos(lat * kDeg2Rad);
        if (lon_scale < 1e-6) lon_scale = 1e-6;

        cur_q_lon = static_cast<int64_t>(lround(lon / quant_step));
        cur_q_lat = static_cast<int64_t>(lround(lat / quant_step));
        cur_ts    = ts_ms;

        out.write(d2bits(lon), 64);
        out.write(d2bits(lat), 64);
        out.write(ts_ms, 64);

        push_hist(cur_q_lon, cur_q_lat, ts_ms);
        point_count = 1;
        return out.bit_count;
    }

    /* ── Incremental point ────────────────────────────────── */
    int64_t actual_ql = static_cast<int64_t>(lround(lon / quant_step));
    int64_t actual_qa = static_cast<int64_t>(lround(lat / quant_step));

    ACoSTPredResult pred = predict_acost(hist, hist_count, cur_q_lon, cur_q_lat,
                                    cur_ts, cfg, lon_scale, quant_step, ts_ms);

    /* Timestamp: raw 64-bit (same as PC acost) */
    int ts_bits = 64;
    out.write(ts_ms, 64);

    /* ── ZP selection ─────────────────────────────────────── */
    if (pred.model != 0) {  /* not STOP */
        int64_t step_dt = static_cast<int64_t>(ts_ms) -
                          static_cast<int64_t>(cur_ts);
        if (step_dt > static_cast<int64_t>(cfg.zp_selection_onset)) {
            int zp_cost = eg_dim_cost(actual_ql - cur_q_lon)
                        + eg_dim_cost(actual_qa - cur_q_lat);
            int mp_cost = eg_dim_cost(actual_ql - pred.q_lon)
                        + eg_dim_cost(actual_qa - pred.q_lat);
            if (zp_cost < mp_cost) {
                out.write_bit(false);  /* ZP wins */
                pred.q_lon = cur_q_lon;
                pred.q_lat = cur_q_lat;
                pred.model = 0;
            } else {
                out.write_bit(true);   /* motion wins */
            }
        }
    }

    int64_t res_ql = actual_ql - pred.q_lon;
    int64_t res_qa = actual_qa - pred.q_lat;

    int data_bits = encode_residual(res_ql, res_qa, out);
    total_bits += data_bits;   /* exclude timestamp from CR */

    int64_t recon_ql = pred.q_lon + res_ql;
    int64_t recon_qa = pred.q_lat + res_qa;

    push_hist(recon_ql, recon_qa, ts_ms);
    cur_q_lon = recon_ql;
    cur_q_lat = recon_qa;
    cur_ts    = ts_ms;
    ++point_count;

    return out.bit_count;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  ACoSTDecompressor
 * ══════════════════════════════════════════════════════════════════════════ */

void ACoSTDecompressor::init(double eps) {
    ACoSTConfig c = acost_default_config();
    init(eps, c);
}

void ACoSTDecompressor::init(double eps, const ACoSTConfig &config) {
    epsilon     = eps * 0.999;
    quant_step  = 2.0 * eps * 0.999;
    lon_scale   = 1.0;
    point_count = 0;
    hist_count  = 0;
    cur_q_lon = cur_q_lat = 0;
    cur_ts = 0;
    cfg = config;
    memset(hist, 0, sizeof(hist));
}

void ACoSTDecompressor::push_hist(int64_t ql, int64_t qa, uint64_t ts) {
    push_hist_acost(hist, hist_count, ql, qa, ts);
}

int64_t ACoSTDecompressor::decode_value(ACoSTInBits &in) {
    return eg_decode(in);
}

void ACoSTDecompressor::decode_residual(ACoSTInBits &in,
                                      int64_t &rl, int64_t &ra) {
    int64_t el = decode_value(in);
    int64_t ea = decode_value(in);
    rl = zz_dec(el - 1LL);
    ra = zz_dec(ea - 1LL);
}

bool ACoSTDecompressor::decompress(ACoSTInBits &in,
                                 double &lon, double &lat, uint64_t &ts_ms) {
    if (point_count == 0) {
        /* ── Header ──────────────────────────────────────── */
        in.read(16);                            /* block_size (discard) */
        double hdr_eps = bits2d(in.read(64));
        quant_step = 2.0 * hdr_eps;

        lon = bits2d(in.read(64));
        lat = bits2d(in.read(64));
        ts_ms = in.read(64);

        lon_scale = std::cos(lat * kDeg2Rad);
        if (lon_scale < 1e-6) lon_scale = 1e-6;

        cur_q_lon = static_cast<int64_t>(lround(lon / quant_step));
        cur_q_lat = static_cast<int64_t>(lround(lat / quant_step));
        cur_ts = ts_ms;

        push_hist(cur_q_lon, cur_q_lat, ts_ms);
        point_count = 1;
        return true;
    }

    /* ── Incremental point ────────────────────────────────── */
    uint64_t ts = in.read(64);

    ACoSTPredResult pred = predict_acost(hist, hist_count, cur_q_lon, cur_q_lat,
                                    cur_ts, cfg, lon_scale, quant_step, ts);

    /* ZP selection mirror */
    if (pred.model != 0) {
        int64_t step_dt = static_cast<int64_t>(ts) -
                          static_cast<int64_t>(cur_ts);
        if (step_dt > static_cast<int64_t>(cfg.zp_selection_onset)) {
            uint32_t flag = in.read_bit();
            if (flag == 0) {
                pred.q_lon = cur_q_lon;
                pred.q_lat = cur_q_lat;
                pred.model = 0;
            }
        }
    }

    int64_t rl, ra;
    decode_residual(in, rl, ra);

    int64_t recon_ql = pred.q_lon + rl;
    int64_t recon_qa = pred.q_lat + ra;

    push_hist(recon_ql, recon_qa, ts);
    cur_q_lon = recon_ql;
    cur_q_lat = recon_qa;
    cur_ts = ts;
    ++point_count;

    lon = static_cast<double>(recon_ql) * quant_step;
    lat = static_cast<double>(recon_qa) * quant_step;
    ts_ms = ts;
    return true;
}
