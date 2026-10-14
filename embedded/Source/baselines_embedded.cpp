/**
 * baselines_embedded.cpp
 *
 * Embedded implementations of Gorilla, Serf-Qt, Elf, and FastLZ
 * for STM32F103 benchmarking.
 */

#include "baselines_embedded.h"

/* ── Helpers ───────────────────────────────────────────────────────────── */

static inline uint64_t bl_d2bits(double v) {
    uint64_t b; memcpy(&b, &v, 8); return b;
}
static inline double bl_bits2d(uint64_t b) {
    double v; memcpy(&v, &b, 8); return v;
}
static inline int bl_clz64(uint64_t x) {
    if (x == 0) return 64;
    int n = 0;
    if (!(x & 0xFFFFFFFF00000000ULL)) { n += 32; x <<= 32; }
    if (!(x & 0xFFFF000000000000ULL)) { n += 16; x <<= 16; }
    if (!(x & 0xFF00000000000000ULL)) { n +=  8; x <<=  8; }
    if (!(x & 0xF000000000000000ULL)) { n +=  4; x <<=  4; }
    if (!(x & 0xC000000000000000ULL)) { n +=  2; x <<=  2; }
    if (!(x & 0x8000000000000000ULL)) { n +=  1; }
    return n;
}
static inline int bl_ctz64(uint64_t x) {
    if (x == 0) return 64;
    int n = 0;
    if (!(x & 0x00000000FFFFFFFFULL)) { n += 32; x >>= 32; }
    if (!(x & 0x000000000000FFFFULL)) { n += 16; x >>= 16; }
    if (!(x & 0x00000000000000FFULL)) { n +=  8; x >>=  8; }
    if (!(x & 0x000000000000000FULL)) { n +=  4; x >>=  4; }
    if (!(x & 0x0000000000000003ULL)) { n +=  2; x >>=  2; }
    if (!(x & 0x0000000000000001ULL)) { n +=  1; }
    return n;
}
static inline int bl_ilog2(uint64_t n) {
    int r = 0; while (n > 1) { n >>= 1; ++r; } return r;
}
static inline int64_t bl_zz_enc(int64_t v) { return (v << 1) ^ (v >> 63); }
static inline int64_t bl_zz_dec(int64_t v) { return (v >> 1) ^ -(v & 1); }

static int bl_eg_encode(int64_t number, BlOutBits &out) {
    if (number < 1) number = 1;
    int n = bl_ilog2(static_cast<uint64_t>(number));
    if (n > 0) out.write(0ULL, n);
    out.write(static_cast<uint64_t>(number), n + 1);
    return 2 * n + 1;
}
static int64_t bl_eg_decode(BlInBits &in) {
    int n = 0;
    while (in.read_bit() == 0) { ++n; if (n > 63) return 1; }
    if (n == 0) return 1LL;
    return static_cast<int64_t>((1ULL << n) | in.read(n));
}

/* ── BlOutBits / BlInBits ──────────────────────────────────────────────── */

void BlOutBits::write(uint64_t val, int len) {
    for (int i = len - 1; i >= 0; --i) {
        int bit = static_cast<int>((val >> i) & 1u);
        int bi = bit_count >> 3;
        int bo = 7 - (bit_count & 7);
        if (bi < BL_BUF_BYTES) buf[bi] |= static_cast<uint8_t>(bit << bo);
        ++bit_count;
    }
}

uint64_t BlInBits::read(int len) {
    uint64_t r = 0;
    for (int i = 0; i < len; ++i) {
        if (pos >= total_bits) { r <<= 1; }
        else {
            int bi = pos >> 3, bo = 7 - (pos & 7);
            r = (r << 1) | ((buf[bi] >> bo) & 1u);
        }
        ++pos;
    }
    return r;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  1. Gorilla
 * ══════════════════════════════════════════════════════════════════════════ */

void GorillaEmbComp::init() {
    first = true; prev = 0; prev_lead = 64; prev_trail = 0; total_bits = 0;
}

int GorillaEmbComp::compress(double v, BlOutBits &out) {
    out.reset();
    uint64_t raw = bl_d2bits(v);

    if (first) {
        first = false;
        out.write(raw, 64);
        prev = raw;
        total_bits += 64;
        return 64;
    }

    uint64_t xored = prev ^ raw;
    if (xored == 0) {
        out.write_bit(false);   /* 0: same value */
        prev = raw;
        total_bits += 1;
        return 1;
    }

    out.write_bit(true);        /* 1: different */
    int lead  = bl_clz64(xored);
    int trail = bl_ctz64(xored);
    if (lead >= 32) lead = 31;

    if (lead >= prev_lead && trail >= prev_trail) {
        out.write_bit(false);   /* 10: reuse lead/trail */
        int sig = 64 - prev_lead - prev_trail;
        out.write(xored >> prev_trail, sig);
        int bits = 2 + sig;
        total_bits += bits;
        prev = raw;
        return out.bit_count;
    }

    out.write_bit(true);        /* 11: new lead/trail */
    out.write(static_cast<uint64_t>(lead), 5);
    int sig = 64 - lead - trail;
    if (sig == 64) {
        out.write(0ULL, 6);
    } else {
        out.write(static_cast<uint64_t>(sig), 6);
    }
    out.write(xored >> trail, sig);
    prev_lead  = lead;
    prev_trail = trail;
    total_bits += out.bit_count;
    prev = raw;
    return out.bit_count;
}

void GorillaEmbDecomp::init() {
    first = true; prev = 0; prev_lead = 64; prev_trail = 0;
}

double GorillaEmbDecomp::decompress(BlInBits &in) {
    if (first) {
        first = false;
        prev = in.read(64);
        return bl_bits2d(prev);
    }

    if (in.read_bit() == 0) {
        return bl_bits2d(prev);  /* same value */
    }

    if (in.read_bit() == 1) {
        prev_lead = static_cast<int>(in.read(5));
        int sig = static_cast<int>(in.read(6));
        if (sig == 0) sig = 64;
        prev_trail = 64 - sig - prev_lead;
    }

    int sig = 64 - prev_lead - prev_trail;
    uint64_t val = in.read(sig);
    val <<= prev_trail;
    prev ^= val;
    return bl_bits2d(prev);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  2. Serf-Qt
 * ══════════════════════════════════════════════════════════════════════════ */

void SerfQtEmbComp::init(double epsilon) {
    max_diff   = epsilon * 0.999;
    quant_step = 2.0 * max_diff;
    prev       = 2.0;   /* match PC implementation initial value */
    point_count = 0;
    total_bits  = 0;
}

int SerfQtEmbComp::compress(double v, BlOutBits &out) {
    out.reset();

    if (point_count == 0) {
        /* Header: block_size(32) + max_diff(64) — only once */
        out.write(1ULL, 32);                              /* block_size = 1 (streaming) */
        out.write(bl_d2bits(max_diff), 64);
    }

    int64_t q = static_cast<int64_t>(std::round((v - prev) / quant_step));
    double recovered = prev + quant_step * static_cast<double>(q);

    int bits = bl_eg_encode(bl_zz_enc(q) + 1, out);
    total_bits += bits;
    prev = recovered;
    ++point_count;
    return out.bit_count;
}

void SerfQtEmbDecomp::init(double epsilon) {
    max_diff   = epsilon * 0.999;
    quant_step = 2.0 * max_diff;
    prev       = 2.0;
    point_count = 0;
}

double SerfQtEmbDecomp::decompress(BlInBits &in) {
    if (point_count == 0) {
        in.read(32);                                      /* block_size */
        double hdr_md = bl_bits2d(in.read(64));
        max_diff   = hdr_md;
        quant_step = 2.0 * hdr_md;
    }

    int64_t decoded = bl_zz_dec(bl_eg_decode(in) - 1);
    double recovered = prev + quant_step * static_cast<double>(decoded);
    prev = recovered;
    ++point_count;
    return recovered;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  2b. Serf-XOR (error-bounded XOR, lossy)
 * ══════════════════════════════════════════════════════════════════════════ */

void SerfXorEmbComp::init(double epsilon) {
    max_diff = epsilon * 0.999;
    prev = 0;
    prev_lead = 64; prev_trail = 0;
    first = true; total_bits = 0;
}

static uint64_t serf_xor_find_best(double v, double eps, uint64_t prev_bits) {
    uint64_t raw = bl_d2bits(v);
    uint64_t best = raw;
    int best_trail = (raw ^ prev_bits) == 0 ? 64 : __builtin_ctzll(raw ^ prev_bits);
    for (int erase = 1; erase <= 48; ++erase) {
        uint64_t mask = ~((1ULL << erase) - 1);
        uint64_t cand = raw & mask;
        double cv = bl_bits2d(cand);
        if (cv >= v - eps && cv <= v + eps) {
            uint64_t x = cand ^ prev_bits;
            int t = (x == 0) ? 64 : __builtin_ctzll(x);
            if (t > best_trail) { best = cand; best_trail = t; }
        }
        uint64_t cand_up = cand + (1ULL << erase);
        double cuv = bl_bits2d(cand_up);
        if (cuv >= v - eps && cuv <= v + eps) {
            uint64_t x = cand_up ^ prev_bits;
            int t = (x == 0) ? 64 : __builtin_ctzll(x);
            if (t > best_trail) { best = cand_up; best_trail = t; }
        }
        if (best_trail >= 52) break;
    }
    return best;
}

int SerfXorEmbComp::compress(double v, BlOutBits &out) {
    out.reset();
    if (first) {
        first = false;
        prev = bl_d2bits(v);
        out.write(prev, 64);
        total_bits += 64;
        return 64;
    }
    uint64_t adjusted = serf_xor_find_best(v, max_diff, prev);
    uint64_t xored = adjusted ^ prev;
    prev = adjusted;
    if (xored == 0) {
        out.write(0ULL, 1);
        total_bits += 1;
        return 1;
    }
    int lead  = __builtin_clzll(xored);
    int trail = __builtin_ctzll(xored);
    int meaningful = 64 - lead - trail;
    if (lead >= prev_lead && trail >= prev_trail) {
        out.write(0b10ULL, 2);
        int pm = 64 - prev_lead - prev_trail;
        out.write(xored >> prev_trail, pm);
        int bits = 2 + pm;
        total_bits += bits;
        return bits;
    } else {
        out.write(0b11ULL, 2);
        out.write((uint64_t)lead, 6);
        out.write((uint64_t)meaningful, 6);
        out.write(xored >> trail, meaningful);
        prev_lead = lead; prev_trail = trail;
        int bits = 2 + 6 + 6 + meaningful;
        total_bits += bits;
        return bits;
    }
}

void SerfXorEmbDecomp::init() {
    prev = 0; prev_lead = 64; prev_trail = 0; first = true;
}

double SerfXorEmbDecomp::decompress(BlInBits &in) {
    if (first) {
        first = false;
        prev = in.read(64);
        return bl_bits2d(prev);
    }
    uint32_t flag = in.read_bit();
    if (flag == 0) return bl_bits2d(prev);
    uint32_t flag2 = in.read_bit();
    uint64_t xored;
    if (flag2 == 0) {
        int pm = 64 - prev_lead - prev_trail;
        xored = in.read(pm) << prev_trail;
    } else {
        int lead = (int)in.read(6);
        int meaningful = (int)in.read(6);
        int trail = 64 - lead - meaningful;
        xored = in.read(meaningful) << trail;
        prev_lead = lead; prev_trail = trail;
    }
    prev ^= xored;
    return bl_bits2d(prev);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  3. Elf (simplified: no significand-count table, degrades to Gorilla+)
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * Simplified Elf: uses "erase trailing" based on the observation that
 * GPS coordinates typically have ~6-7 significant decimal digits.
 * We erase the lowest 20 mantissa bits (52-32=20) which is safe for
 * values with ≥7 significant digits. This is a conservative approximation
 * of the full Elf significand-count analysis.
 */
static constexpr int ELF_ERASE_BITS = 20;

uint64_t ElfEmbComp::erase_trailing(uint64_t bits) {
    /* Zero the lowest ELF_ERASE_BITS of the mantissa */
    uint64_t mask = ~((1ULL << ELF_ERASE_BITS) - 1);
    return bits & mask;
}

uint64_t ElfEmbDecomp::erase_trailing(uint64_t bits) {
    uint64_t mask = ~((1ULL << ELF_ERASE_BITS) - 1);
    return bits & mask;
}

void ElfEmbComp::init() {
    first = true; prev = 0; prev_erased = 0;
    prev_lead = 64; prev_trail = 0; total_bits = 0;
}

int ElfEmbComp::compress(double v, BlOutBits &out) {
    out.reset();
    uint64_t raw = bl_d2bits(v);
    uint64_t erased = erase_trailing(raw);

    if (first) {
        first = false;
        out.write(raw, 64);
        prev = raw; prev_erased = erased;
        total_bits += 64;
        return 64;
    }

    uint64_t xored = prev_erased ^ erased;
    if (xored == 0) {
        /* Encode the non-erased trailing bits to recover exact value */
        out.write_bit(false);   /* 0: XOR of erased = 0 */
        /* Write the trailing ELF_ERASE_BITS of current value */
        out.write(raw & ((1ULL << ELF_ERASE_BITS) - 1), ELF_ERASE_BITS);
        total_bits += 1 + ELF_ERASE_BITS;
        prev = raw; prev_erased = erased;
        return out.bit_count;
    }

    out.write_bit(true);   /* 1: XOR ≠ 0 */

    int lead  = bl_clz64(xored);
    int trail = bl_ctz64(xored);
    if (lead >= 32) lead = 31;

    if (lead >= prev_lead && trail >= prev_trail) {
        out.write_bit(false);
        int sig = 64 - prev_lead - prev_trail;
        out.write(xored >> prev_trail, sig);
    } else {
        out.write_bit(true);
        out.write(static_cast<uint64_t>(lead), 5);
        int sig = 64 - lead - trail;
        out.write(sig == 64 ? 0ULL : static_cast<uint64_t>(sig), 6);
        out.write(xored >> trail, sig);
        prev_lead  = lead;
        prev_trail = trail;
    }

    /* Also write trailing bits for exact recovery */
    out.write(raw & ((1ULL << ELF_ERASE_BITS) - 1), ELF_ERASE_BITS);

    total_bits += out.bit_count;
    prev = raw; prev_erased = erased;
    return out.bit_count;
}

void ElfEmbDecomp::init() {
    first = true; prev = 0; prev_erased = 0;
    prev_lead = 64; prev_trail = 0;
}

double ElfEmbDecomp::decompress(BlInBits &in) {
    if (first) {
        first = false;
        prev = in.read(64);
        prev_erased = erase_trailing(prev);
        return bl_bits2d(prev);
    }

    uint64_t erased;
    if (in.read_bit() == 0) {
        erased = prev_erased;  /* XOR = 0 */
    } else {
        if (in.read_bit() == 1) {
            prev_lead = static_cast<int>(in.read(5));
            int sig = static_cast<int>(in.read(6));
            if (sig == 0) sig = 64;
            prev_trail = 64 - sig - prev_lead;
        }
        int sig = 64 - prev_lead - prev_trail;
        uint64_t val = in.read(sig);
        val <<= prev_trail;
        erased = prev_erased ^ val;
    }

    /* Read trailing bits */
    uint64_t trail_bits = in.read(ELF_ERASE_BITS);
    uint64_t full = erased | trail_bits;

    prev = full;
    prev_erased = erased;
    return bl_bits2d(full);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  4. FastLZ (level-1 only, minimal embedded port)
 *     Ported from baselines/lz77/fastlz.c with only FLEVEL=1.
 * ══════════════════════════════════════════════════════════════════════════ */

/* Hash table for LZ77 matching (2^10 = 1024 entries × 2 bytes = 2 KB).
 * Reduced from 2^13 to fit STM32 20KB RAM budget. */
#define FASTLZ_HASH_LOG  10
#define FASTLZ_HASH_SIZE (1 << FASTLZ_HASH_LOG)
#define FASTLZ_HASH_MASK (FASTLZ_HASH_SIZE - 1)

static uint16_t flz_htab[FASTLZ_HASH_SIZE];

static inline uint32_t flz_hash(uint32_t v) {
    return ((v * 2654435769UL) >> (32 - FASTLZ_HASH_LOG)) & FASTLZ_HASH_MASK;
}

static inline uint32_t flz_read32(const uint8_t *p) {
    uint32_t v;
    memcpy(&v, p, 4);
    return v;
}

int fastlz_compress(const void *input, int length, void *output) {
    if (length < 4) {
        /* Too short to compress; store raw */
        uint8_t *op = (uint8_t *)output;
        const uint8_t *ip = (const uint8_t *)input;
        for (int i = 0; i < length; ++i) *op++ = *ip++;
        return length;
    }

    const uint8_t *ip = (const uint8_t *)input;
    const uint8_t *ip_end = ip + length;
    const uint8_t *ip_limit = ip_end - 12;
    uint8_t *op = (uint8_t *)output;

    memset(flz_htab, 0, sizeof(flz_htab));

    /* First byte as literal */
    const uint8_t *anchor = ip;
    ip += 2;

    while (ip < ip_limit) {
        uint32_t hval = flz_hash(flz_read32(ip));
        const uint8_t *ref = (const uint8_t *)input + flz_htab[hval];
        flz_htab[hval] = (uint16_t)(ip - (const uint8_t *)input);

        if (ref >= ip || ip - ref > 8191 ||
            memcmp(ref, ip, 3) != 0) {
            ++ip;
            continue;
        }

        /* Emit literal run */
        int lit = (int)(ip - anchor);
        if (lit > 0) {
            if (lit < 7) {
                /* Short literal: encode count in match token later */
            }
            /* For simplicity, always emit literal bytes one by one */
            while (lit > 262) {
                *op++ = (uint8_t)(31 << 5);  /* long literal */
                *op++ = (uint8_t)(262 - 1);
                for (int i = 0; i < 262; ++i) *op++ = *anchor++;
                lit -= 262;
            }
            if (lit >= 7) {
                *op++ = (uint8_t)((7 << 5) | 0);
                *op++ = (uint8_t)(lit - 7);
                for (int i = 0; i < lit; ++i) *op++ = *anchor++;
            } else {
                /* Store count, will be patched into match token */
                /* Simple approach: emit as 1-byte header + literals */
                *op++ = (uint8_t)(lit << 5);
                for (int i = 0; i < lit; ++i) *op++ = *anchor++;
            }
        }

        /* Count match length */
        int len = 3;
        while (ip + len < ip_end && ip[len] == ref[len] && len < 264) ++len;

        /* Emit match */
        int dist = (int)(ip - ref) - 1;
        if (len < 7) {
            *op++ = (uint8_t)((len - 2) << 5 | (dist >> 8));
        } else {
            *op++ = (uint8_t)((7 << 5) | (dist >> 8));
            *op++ = (uint8_t)(len - 7 - 2);
        }
        *op++ = (uint8_t)(dist & 0xFF);

        ip += len;
        anchor = ip;
    }

    /* Emit remaining literals */
    int lit = (int)(ip_end - anchor);
    if (lit > 0) {
        if (lit >= 7) {
            *op++ = (uint8_t)((7 << 5));
            *op++ = (uint8_t)(lit - 7);
        } else {
            *op++ = (uint8_t)(lit << 5);
        }
        while (anchor < ip_end) *op++ = *anchor++;
    }

    return (int)(op - (uint8_t *)output);
}

int fastlz_decompress(const void *input, int length,
                        void *output, int maxout) {
    const uint8_t *ip = (const uint8_t *)input;
    const uint8_t *ip_end = ip + length;
    uint8_t *op = (uint8_t *)output;
    uint8_t *op_end = op + maxout;

    while (ip < ip_end) {
        uint8_t ctrl = *ip++;
        int code = ctrl >> 5;
        int ofs  = (ctrl & 31) << 8;

        if (code == 0) {
            /* Literal run of (ctrl>>5)+1 bytes — but code==0 means literal count in lower 5 bits */
            int lit = (ctrl & 31) + 1;
            if (lit >= 32) {
                /* Extended literal */
                lit = 32 + *ip++;
            }
            if (op + lit > op_end || ip + lit > ip_end) break;
            memcpy(op, ip, lit);
            op += lit; ip += lit;
        } else {
            /* Match */
            int len = code + 2;
            if (code == 7) {
                if (ip >= ip_end) break;
                len += *ip++;
            }
            if (ip >= ip_end) break;
            ofs |= *ip++;
            int dist = ofs + 1;

            if (op - dist < (uint8_t *)output) break;
            if (op + len > op_end) break;

            const uint8_t *src = op - dist;
            for (int i = 0; i < len; ++i) *op++ = *src++;
        }
    }

    return (int)(op - (uint8_t *)output);
}

/* ── FastLzEmbState ──────────────────────────────────────────────────── */

void FastLzEmbState::init() {
    raw_pos     = 0;
    flush_count = 0;
    total_raw   = 0;
    total_comp  = 0;
}

int FastLzEmbState::feed(double v) {
    if (raw_pos + 8 > FASTLZ_RAW_SIZE) {
        return flush();
    }
    memcpy(raw_buf + raw_pos, &v, 8);
    raw_pos += 8;
    total_raw += 8;

    if (raw_pos >= FASTLZ_RAW_SIZE) {
        return flush();
    }
    return 0;
}

int FastLzEmbState::flush() {
    if (raw_pos < 4) {
        /* Too small, just count as raw */
        total_comp += raw_pos;
        int ret = raw_pos;
        raw_pos = 0;
        flush_count++;
        return ret;
    }
    int comp_len = fastlz_compress(raw_buf, raw_pos, out_buf);
    total_comp += comp_len;
    int ret = comp_len;
    raw_pos = 0;
    flush_count++;
    return ret;
}
