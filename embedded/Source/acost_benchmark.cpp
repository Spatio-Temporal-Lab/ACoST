/**
 * acost_benchmark.cpp  –  Full E2E benchmark:
 *   Ours:      ACoST / CoST
 *   Serf-base: Gorilla / Serf-Qt / Elf / LZ77(FastLZ)
 *   Trad:      OPWTR / DR / SQUISH-E / VOLTCom
 *   Baseline:  Raw
 *
 * Metrics per algorithm (Serf SIGMOD'25 style):
 *   - Compression Ratio = compressed_bits / (points × 128)
 *   - Compression Time  = DWT cycles → μs
 *   - Decompression Time = DWT cycles → μs
 *   - E2E = comp_time + simulated_tx_time + decomp_time
 *
 * Simulated TX: 9600 baud (IoT GPRS), 1 byte = 10 bits / 9600 = 1041.67 μs
 */

#include "acost_benchmark.h"
#include "acost_embedded.h"
#include "cost_embedded.h"
#include "baselines_embedded.h"
#include "trad_algorithms_embedded.h"

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include <stdio.h>
#include <string.h>

/* ── DWT ─────────────────────────────────────────────────────────────── */
#define DWT_CYCCNT  (*((volatile uint32_t*)0xE0001004UL))
#define DWT_CTRL    (*((volatile uint32_t*)0xE0001000UL))
#define DEMCR       (*((volatile uint32_t*)0xE000EDFCUL))

static inline void dwt_enable(void) {
    DEMCR     |= (1UL << 24);
    DWT_CYCCNT = 0;
    DWT_CTRL  |= 1UL;
}

/* DWT → 0.1-μs at 72 MHz */
static inline uint32_t cyc2x10(uint32_t c) { return (c * 10UL) / 72UL; }

/* ── TX time constant: simulated 9600 baud ───────────────────────────── */
#define TX_X10_PER_BYTE   10417UL   /* 1 byte = 1041.7 μs → 10417 × 0.1μs */
#define TX_OVERHEAD_COMP  7UL       /* frame header+trailer for compressed */
#define TX_OVERHEAD_TRAD  5UL       /* frame header for traditional */
#define TRAD_DATA_BYTES   24UL      /* lon(8)+lat(8)+ts(8) */
#define RAW_DATA_BYTES    24UL

/* ── UART helpers (blocking, 115200 for report only) ─────────────────── */
static void u1_putc(uint8_t c) {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, (uint16_t)c);
}
static void u1_puts(const char *s) { while (*s) u1_putc((uint8_t)*s++); }

/* ── Algorithm indices ───────────────────────────────────────────────── */
/* 0=ACoST  1=CoST  2=Gorilla  3=Serf-Qt  4=Serf-XOR  5=Elf  6=LZ77
 * 7=OPWTR  8=DR  9=SQUISH-E  10=VOLTCom  11=Raw */
#define ALG_COUNT 12
#define ALG_ACOST   0
#define ALG_COST     1
#define ALG_GORILLA  2
#define ALG_SERFQT   3
#define ALG_SERFXOR  4
#define ALG_ELF      5
#define ALG_LZ77     6
#define ALG_OPWTR    7
#define ALG_DR       8
#define ALG_SQUISHE  9
#define ALG_VOLTCOM  10
#define ALG_RAW      11

static const char * const ALG_NAMES[ALG_COUNT] = {
    "ACoST", "CoST",
    "Gorilla", "Serf-Qt", "Serf-XOR", "Elf", "LZ77",
    "OPWTR", "DR", "SQUISH-E", "VOLTCom", "Raw"
};

/* ── Per-algorithm stats ─────────────────────────────────────────────── */
static uint64_t s_comp_sum[ALG_COUNT];     /* sum of compression time (0.1μs) */
static uint64_t s_decomp_sum[ALG_COUNT];   /* sum of decompression time (0.1μs) */
static uint64_t s_e2e_sum[ALG_COUNT];      /* sum of E2E (0.1μs) */
static uint32_t s_pts[ALG_COUNT];          /* points processed */
static uint64_t s_total_bits[ALG_COUNT];   /* total compressed bits (excl ts) */
static uint32_t s_emitted[ALG_COUNT];      /* for trad: emitted key points */

/* ── Algorithm state ─────────────────────────────────────────────────── */
static ACoSTCompressor      s_acost_comp;
static ACoSTDecompressor    s_acost_decomp;
static CostCompressor    s_cost_comp;
static CostDecompressor  s_cost_decomp;
static GorillaEmbComp    s_gor_comp_lon, s_gor_comp_lat;
static GorillaEmbDecomp  s_gor_decomp_lon, s_gor_decomp_lat;
static SerfQtEmbComp     s_sqt_comp_lon, s_sqt_comp_lat;
static SerfQtEmbDecomp   s_sqt_decomp_lon, s_sqt_decomp_lat;
static SerfXorEmbComp    s_sxor_comp_lon, s_sxor_comp_lat;
static SerfXorEmbDecomp  s_sxor_decomp_lon, s_sxor_decomp_lat;
static ElfEmbComp        s_elf_comp_lon, s_elf_comp_lat;
static ElfEmbDecomp      s_elf_decomp_lon, s_elf_decomp_lat;
static FastLzEmbState    s_flz_lon, s_flz_lat;
static OpwtrState        s_opwtr;
static DrState           s_dr;
static SquisheState      s_squishe;
static VoltcomState      s_voltcom;

/* ── Pending queues for window-based trad algorithms ─────────────────── */
#define PEND_MAX 32
static uint64_t s_pend[3][PEND_MAX];   /* 0=OPWTR 1=DR 2=VOLTCom */
static int      s_npend[3];

/* ── Dump trigger ────────────────────────────────────────────────────── */
static volatile int s_dump_requested = 0;

/* ══════════════════════════════════════════════════════════════════════════
 *  Init
 * ══════════════════════════════════════════════════════════════════════════ */
void AcostBench_Init(double epsilon) {
    dwt_enable();

    /* NOTE: USART1 hardware init is done by main.c (init_usart1).
     * We only init the algorithms and stats here. */

    /* Algorithm init */
    s_acost_comp.init(epsilon);
    s_acost_decomp.init(epsilon);
    s_cost_comp.init(epsilon);
    s_cost_decomp.init(epsilon);

    /* Serf-paper baselines */
    s_gor_comp_lon.init();  s_gor_comp_lat.init();
    s_gor_decomp_lon.init(); s_gor_decomp_lat.init();
    s_sqt_comp_lon.init(epsilon); s_sqt_comp_lat.init(epsilon);
    s_sqt_decomp_lon.init(epsilon); s_sqt_decomp_lat.init(epsilon);
    s_sxor_comp_lon.init(epsilon); s_sxor_comp_lat.init(epsilon);
    s_sxor_decomp_lon.init(); s_sxor_decomp_lat.init();
    s_elf_comp_lon.init(); s_elf_comp_lat.init();
    s_elf_decomp_lon.init(); s_elf_decomp_lat.init();
    s_flz_lon.init(); s_flz_lat.init();

    double eps_m = epsilon * 111000.0;
    s_opwtr.init(epsilon);
    s_dr.init(eps_m);
    s_squishe.init(epsilon);
    s_voltcom.init(eps_m);

    for (int i = 0; i < ALG_COUNT; ++i) {
        s_comp_sum[i] = s_decomp_sum[i] = s_e2e_sum[i] = 0;
        s_pts[i] = 0;
        s_total_bits[i] = 0;
        s_emitted[i] = 0;
    }
    for (int i = 0; i < 3; ++i) s_npend[i] = 0;
    s_dump_requested = 0;

    u1_puts("\r\n=== E2E Benchmark READY (11 algorithms) ===\r\n");
    u1_puts("ACoST/CoST + Gorilla/Serf-Qt/Elf/LZ77 + 4trad + Raw\r\n");
    u1_puts("Send any byte to dump results.\r\n");
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Feed
 * ══════════════════════════════════════════════════════════════════════════ */
void AcostBench_Feed(double lon_deg, double lat_deg, uint64_t ts_ms) {

    TradPoint tp;
    tp.lon = lon_deg; tp.lat = lat_deg; tp.ts = ts_ms;
    tp.t_arrive = DWT_CYCCNT;

    /* ── 1. ACoST ──────────────────────────────────────────────────── */
    {
        uint32_t t0 = DWT_CYCCNT;
        ACoSTOutBits out;
        s_acost_comp.compress(lon_deg, lat_deg, ts_ms, out);
        uint32_t t1 = DWT_CYCCNT;

        ACoSTInBits in;
        in.init(out.buf, out.bit_count);
        double dl, da; uint64_t dt;
        s_acost_decomp.decompress(in, dl, da, dt);
        uint32_t t2 = DWT_CYCCNT;

        uint32_t comp_x10   = cyc2x10(t1 - t0);
        uint32_t decomp_x10 = cyc2x10(t2 - t1);
        uint32_t tx_x10     = (TX_OVERHEAD_COMP + (uint32_t)out.byte_count()) * TX_X10_PER_BYTE;

        s_comp_sum[ALG_ACOST]   += comp_x10;
        s_decomp_sum[ALG_ACOST] += decomp_x10;
        s_e2e_sum[ALG_ACOST]    += comp_x10 + tx_x10 + decomp_x10;
        /* Bits: subtract 64-bit timestamp (not counted in CR, same as PC) */
        s_total_bits[ALG_ACOST] += static_cast<uint64_t>(out.bit_count) - 64ULL;
        ++s_pts[ALG_ACOST];
    }

    /* ── 2. CoST ────────────────────────────────────────────────────── */
    {
        uint32_t t0 = DWT_CYCCNT;
        CostOutBits out;
        s_cost_comp.compress(lon_deg, lat_deg, ts_ms, out);
        uint32_t t1 = DWT_CYCCNT;

        CostInBits in;
        in.init(out.buf, out.bit_count);
        double dl, da; uint64_t dt;
        s_cost_decomp.decompress(in, dl, da, dt);
        uint32_t t2 = DWT_CYCCNT;

        uint32_t comp_x10   = cyc2x10(t1 - t0);
        uint32_t decomp_x10 = cyc2x10(t2 - t1);
        uint32_t tx_x10     = (TX_OVERHEAD_COMP + (uint32_t)out.byte_count()) * TX_X10_PER_BYTE;

        s_comp_sum[ALG_COST]   += comp_x10;
        s_decomp_sum[ALG_COST] += decomp_x10;
        s_e2e_sum[ALG_COST]    += comp_x10 + tx_x10 + decomp_x10;
        /* Subtract 64-bit timestamp (same as PC: total_bits - timestamp_bits) */
        s_total_bits[ALG_COST] += static_cast<uint64_t>(out.bit_count) - 64ULL;
        ++s_pts[ALG_COST];
    }

    /* ── 3. Gorilla (per-value XOR, lossless) ──────────────────────────── */
    {
        uint32_t t0 = DWT_CYCCNT;
        BlOutBits out_lon, out_lat;
        s_gor_comp_lon.compress(lon_deg, out_lon);
        s_gor_comp_lat.compress(lat_deg, out_lat);
        uint32_t t1 = DWT_CYCCNT;

        BlInBits in_lon, in_lat;
        in_lon.init(out_lon.buf, out_lon.bit_count);
        in_lat.init(out_lat.buf, out_lat.bit_count);
        s_gor_decomp_lon.decompress(in_lon);
        s_gor_decomp_lat.decompress(in_lat);
        uint32_t t2 = DWT_CYCCNT;

        uint32_t comp_x10   = cyc2x10(t1 - t0);
        uint32_t decomp_x10 = cyc2x10(t2 - t1);
        int total_bytes = out_lon.byte_count() + out_lat.byte_count();
        uint32_t tx_x10 = (TX_OVERHEAD_COMP + (uint32_t)total_bytes) * TX_X10_PER_BYTE;

        s_comp_sum[ALG_GORILLA]   += comp_x10;
        s_decomp_sum[ALG_GORILLA] += decomp_x10;
        s_e2e_sum[ALG_GORILLA]    += comp_x10 + tx_x10 + decomp_x10;
        s_total_bits[ALG_GORILLA] += static_cast<uint64_t>(out_lon.bit_count + out_lat.bit_count);
        ++s_pts[ALG_GORILLA];
    }

    /* ── 5. Serf-Qt (per-value quantize, lossy) ─────────────────────── */
    {
        uint32_t t0 = DWT_CYCCNT;
        BlOutBits out_lon, out_lat;
        s_sqt_comp_lon.compress(lon_deg, out_lon);
        s_sqt_comp_lat.compress(lat_deg, out_lat);
        uint32_t t1 = DWT_CYCCNT;

        BlInBits in_lon, in_lat;
        in_lon.init(out_lon.buf, out_lon.bit_count);
        in_lat.init(out_lat.buf, out_lat.bit_count);
        s_sqt_decomp_lon.decompress(in_lon);
        s_sqt_decomp_lat.decompress(in_lat);
        uint32_t t2 = DWT_CYCCNT;

        uint32_t comp_x10   = cyc2x10(t1 - t0);
        uint32_t decomp_x10 = cyc2x10(t2 - t1);
        int total_bytes = out_lon.byte_count() + out_lat.byte_count();
        uint32_t tx_x10 = (TX_OVERHEAD_COMP + (uint32_t)total_bytes) * TX_X10_PER_BYTE;

        s_comp_sum[ALG_SERFQT]   += comp_x10;
        s_decomp_sum[ALG_SERFQT] += decomp_x10;
        s_e2e_sum[ALG_SERFQT]    += comp_x10 + tx_x10 + decomp_x10;
        s_total_bits[ALG_SERFQT] += static_cast<uint64_t>(out_lon.bit_count + out_lat.bit_count);
        ++s_pts[ALG_SERFQT];
    }

    /* ── 5. Serf-XOR (per-value error-bounded XOR) ─────────────────── */
    {
        uint32_t t0 = DWT_CYCCNT;
        BlOutBits out_lon, out_lat;
        s_sxor_comp_lon.compress(lon_deg, out_lon);
        s_sxor_comp_lat.compress(lat_deg, out_lat);
        uint32_t t1 = DWT_CYCCNT;

        BlInBits in_lon, in_lat;
        in_lon.init(out_lon.buf, out_lon.bit_count);
        in_lat.init(out_lat.buf, out_lat.bit_count);
        s_sxor_decomp_lon.decompress(in_lon);
        s_sxor_decomp_lat.decompress(in_lat);
        uint32_t t2 = DWT_CYCCNT;

        uint32_t comp_x10   = cyc2x10(t1 - t0);
        uint32_t decomp_x10 = cyc2x10(t2 - t1);
        int total_bytes = out_lon.byte_count() + out_lat.byte_count();
        uint32_t tx_x10 = (TX_OVERHEAD_COMP + (uint32_t)total_bytes) * TX_X10_PER_BYTE;

        s_comp_sum[ALG_SERFXOR]   += comp_x10;
        s_decomp_sum[ALG_SERFXOR] += decomp_x10;
        s_e2e_sum[ALG_SERFXOR]    += comp_x10 + tx_x10 + decomp_x10;
        s_total_bits[ALG_SERFXOR] += static_cast<uint64_t>(out_lon.bit_count + out_lat.bit_count);
        ++s_pts[ALG_SERFXOR];
    }

    /* ── 6. Elf (per-value precision-adaptive XOR) ──────────────────── */
    {
        uint32_t t0 = DWT_CYCCNT;
        BlOutBits out_lon, out_lat;
        s_elf_comp_lon.compress(lon_deg, out_lon);
        s_elf_comp_lat.compress(lat_deg, out_lat);
        uint32_t t1 = DWT_CYCCNT;

        BlInBits in_lon, in_lat;
        in_lon.init(out_lon.buf, out_lon.bit_count);
        in_lat.init(out_lat.buf, out_lat.bit_count);
        s_elf_decomp_lon.decompress(in_lon);
        s_elf_decomp_lat.decompress(in_lat);
        uint32_t t2 = DWT_CYCCNT;

        uint32_t comp_x10   = cyc2x10(t1 - t0);
        uint32_t decomp_x10 = cyc2x10(t2 - t1);
        int total_bytes = out_lon.byte_count() + out_lat.byte_count();
        uint32_t tx_x10 = (TX_OVERHEAD_COMP + (uint32_t)total_bytes) * TX_X10_PER_BYTE;

        s_comp_sum[ALG_ELF]   += comp_x10;
        s_decomp_sum[ALG_ELF] += decomp_x10;
        s_e2e_sum[ALG_ELF]    += comp_x10 + tx_x10 + decomp_x10;
        s_total_bits[ALG_ELF] += static_cast<uint64_t>(out_lon.bit_count + out_lat.bit_count);
        ++s_pts[ALG_ELF];
    }

    /* ── 7. LZ77/FastLZ (block-based, lossless) ─────────────────────── */
    {
        uint32_t t0 = DWT_CYCCNT;
        s_flz_lon.feed(lon_deg);
        s_flz_lat.feed(lat_deg);
        uint32_t t1 = DWT_CYCCNT;

        uint32_t comp_x10 = cyc2x10(t1 - t0);
        s_comp_sum[ALG_LZ77] += comp_x10;
        /* CR computed from accumulated total_raw / total_comp at report time */
        s_total_bits[ALG_LZ77] = (s_flz_lon.total_comp + s_flz_lat.total_comp) * 8;
        ++s_pts[ALG_LZ77];
        /* E2E: use raw bytes ratio for TX estimate */
        uint32_t raw_bytes_this = 16; /* 8 + 8 */
        double cr_est = (s_flz_lon.total_raw + s_flz_lat.total_raw) > 0
            ? (double)(s_flz_lon.total_comp + s_flz_lat.total_comp)
              / (double)(s_flz_lon.total_raw + s_flz_lat.total_raw)
            : 1.0;
        uint32_t comp_bytes = (uint32_t)(raw_bytes_this * cr_est);
        uint32_t tx_x10 = (TX_OVERHEAD_COMP + comp_bytes) * TX_X10_PER_BYTE;
        s_e2e_sum[ALG_LZ77] += comp_x10 + tx_x10;
    }

    /* ── 8. OPWTR ───────────────────────────────────────────────────── */
    {
        int &np = s_npend[0];
        if (np < PEND_MAX) s_pend[0][np++] = ts_ms;
        uint32_t t0 = DWT_CYCCNT;
        bool emitted = s_opwtr.feed(tp);
        uint32_t t1 = DWT_CYCCNT;
        uint32_t comp_x10 = cyc2x10(t1 - t0);
        s_comp_sum[ALG_OPWTR] += comp_x10;
        ++s_pts[ALG_OPWTR];
        if (emitted) {
            s_emitted[ALG_OPWTR]++;
            for (int i = 0; i < np; ++i) {
                uint64_t wait_x10 = (ts_ms - s_pend[0][i]) * 10000ULL;
                s_e2e_sum[ALG_OPWTR] += wait_x10 + (TX_OVERHEAD_TRAD + TRAD_DATA_BYTES) * TX_X10_PER_BYTE;
            }
            np = 0;
        }
    }

    /* ── 9. Dead Reckoning ──────────────────────────────────────────── */
    {
        int &np = s_npend[1];
        if (np < PEND_MAX) s_pend[1][np++] = ts_ms;
        uint32_t t0 = DWT_CYCCNT;
        bool emitted = s_dr.feed(tp);
        uint32_t t1 = DWT_CYCCNT;
        s_comp_sum[ALG_DR] += cyc2x10(t1 - t0);
        ++s_pts[ALG_DR];
        if (emitted) {
            s_emitted[ALG_DR]++;
            for (int i = 0; i < np; ++i) {
                uint64_t wait_x10 = (ts_ms - s_pend[1][i]) * 10000ULL;
                s_e2e_sum[ALG_DR] += wait_x10 + (TX_OVERHEAD_TRAD + TRAD_DATA_BYTES) * TX_X10_PER_BYTE;
            }
            np = 0;
        }
    }

    /* ── 10. SQUISH-E ────────────────────────────────────────────────── */
    {
        uint32_t t0 = DWT_CYCCNT;
        bool emitted = s_squishe.feed(tp);
        uint32_t t1 = DWT_CYCCNT;
        s_comp_sum[ALG_SQUISHE] += cyc2x10(t1 - t0);
        ++s_pts[ALG_SQUISHE];
        if (emitted) {
            s_emitted[ALG_SQUISHE]++;
            uint64_t wait_x10 = (ts_ms - s_squishe.last_key.ts) * 10000ULL;
            s_e2e_sum[ALG_SQUISHE] += wait_x10 + (TX_OVERHEAD_TRAD + TRAD_DATA_BYTES) * TX_X10_PER_BYTE;
        }
    }

    /* ── 11. VOLTCom ─────────────────────────────────────────────────── */
    {
        int &np = s_npend[2];
        if (np < PEND_MAX) s_pend[2][np++] = ts_ms;
        uint32_t t0 = DWT_CYCCNT;
        bool emitted = s_voltcom.feed(tp);
        uint32_t t1 = DWT_CYCCNT;
        s_comp_sum[ALG_VOLTCOM] += cyc2x10(t1 - t0);
        ++s_pts[ALG_VOLTCOM];
        if (emitted) {
            s_emitted[ALG_VOLTCOM]++;
            for (int i = 0; i < np; ++i) {
                uint64_t wait_x10 = (ts_ms - s_pend[2][i]) * 10000ULL;
                s_e2e_sum[ALG_VOLTCOM] += wait_x10 + (TX_OVERHEAD_TRAD + TRAD_DATA_BYTES) * TX_X10_PER_BYTE;
            }
            np = 0;
        }
    }

    /* ── 12. Raw ─────────────────────────────────────────────────────── */
    {
        s_e2e_sum[ALG_RAW] += (TX_OVERHEAD_TRAD + RAW_DATA_BYTES) * TX_X10_PER_BYTE;
        s_total_bits[ALG_RAW] += 128;  /* lon 64 + lat 64 */
        ++s_pts[ALG_RAW];
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  USART1 ISR
 * ══════════════════════════════════════════════════════════════════════════ */
void AcostBench_USART1_IRQ(void) {
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
        USART_ReceiveData(USART1);
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == RESET) return;
    USART_ReceiveData(USART1);
    s_dump_requested = 1;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Report (CSV)
 * ══════════════════════════════════════════════════════════════════════════ */
void AcostBench_Report(void) {
    s_dump_requested = 0;
    char buf[128];

    u1_puts("\r\n=== STM32 E2E Benchmark Report ===\r\n");
    u1_puts("ALG,Points,CR,AvgCompTime_us,AvgDecompTime_us,AvgE2E_us\r\n");

    for (int i = 0; i < ALG_COUNT; ++i) {
        uint32_t n = s_pts[i];
        if (n == 0) continue;

        uint64_t original_bits = (uint64_t)n * 128ULL;
        uint64_t comp_bits;
        double cr;

        if (i <= ALG_LZ77) {
            /* Streaming/block compressors: use total_bits directly */
            comp_bits = s_total_bits[i];
            cr = original_bits > 0 ? (double)comp_bits / (double)original_bits : 0.0;
        } else if (i == ALG_RAW) {
            comp_bits = s_total_bits[i];
            cr = 1.0;
        } else {
            /* Trad: CR = emitted_keypoints × 128 / (total_points × 128)
             * = emitted / total  (only lon+lat, no timestamp) */
            comp_bits = (uint64_t)s_emitted[i] * 128ULL;
            cr = original_bits > 0 ? (double)comp_bits / (double)original_bits : 0.0;
        }

        /* Averages in 0.1-μs; convert to μs with 1 decimal */
        uint32_t avg_comp_x10   = (uint32_t)(s_comp_sum[i] / n);
        uint32_t avg_decomp_x10 = (uint32_t)(s_decomp_sum[i] / n);
        uint32_t avg_e2e_x10    = (uint32_t)(s_e2e_sum[i] / n);

        /* CR as integer parts to avoid %f issues with nano.specs */
        uint32_t cr_int  = (uint32_t)(cr * 1000000.0);  /* 6 decimal digits */
        uint32_t cr_whole = cr_int / 1000000;
        uint32_t cr_frac  = cr_int % 1000000;

        /* Use %lu only (nano.specs compatible) */
        sprintf(buf, "%s,%lu,%lu.%06lu,%lu.%lu,%lu.%lu,%lu.%lu\r\n",
                ALG_NAMES[i],
                (unsigned long)n,
                (unsigned long)cr_whole,
                (unsigned long)cr_frac,
                (unsigned long)(avg_comp_x10 / 10),
                (unsigned long)(avg_comp_x10 % 10),
                (unsigned long)(avg_decomp_x10 / 10),
                (unsigned long)(avg_decomp_x10 % 10),
                (unsigned long)(avg_e2e_x10 / 10),
                (unsigned long)(avg_e2e_x10 % 10));
        u1_puts(buf);
    }
    u1_puts("====================================\r\n");
}

/* ── Poll helpers ──────────────────────────────────────────────────────── */
int AcostBench_IsDumpRequested(void) { return s_dump_requested; }

uint32_t AcostBench_GetAvgX10us(void) {
    uint32_t n = s_pts[ALG_ACOST];
    return n ? (uint32_t)(s_e2e_sum[ALG_ACOST] / n) : 0u;
}

uint32_t AcostBench_GetCount(void) { return s_pts[ALG_ACOST]; }
