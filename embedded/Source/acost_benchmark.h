/**
 * acost_benchmark.h  –  Updated for ACoST + CoST + ACoST-DoD + 4 trad + Raw
 *
 * Metrics per algorithm (Serf-style):
 *   1. Compression Ratio (CR)
 *   2. Average Compression Time (μs)
 *   3. Average Decompression Time (μs)
 *   4. Average E2E Latency (μs) = comp + simulated TX + decomp
 */

#ifndef ACOST_BENCHMARK_H
#define ACOST_BENCHMARK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise DWT, USART1, and all 8 algorithms.
 * @param epsilon  Error threshold in degrees (e.g. 1e-5 ≈ 1 m).
 */
void AcostBench_Init(double epsilon);

/**
 * Feed one GPS fix to all algorithms.
 */
void AcostBench_Feed(double lon_deg, double lat_deg, uint64_t ts_ms);

/**
 * Dump CSV-format report over USART1 TX.
 */
void AcostBench_Report(void);

/**
 * Must be called from USART1_IRQHandler.
 */
void AcostBench_USART1_IRQ(void);

/** Returns non-zero if a dump was requested. */
int  AcostBench_IsDumpRequested(void);

/** ACoST running average E2E in 0.1-μs units (for OLED). */
uint32_t AcostBench_GetAvgX10us(void);

/** Total GPS fixes processed. */
uint32_t AcostBench_GetCount(void);

#ifdef __cplusplus
}
#endif

#endif /* ACOST_BENCHMARK_H */
