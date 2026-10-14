/**
 * Benchmark Utilities
 * 
 * This header provides utilities to prevent dead code elimination
 * in performance tests when using -O3 optimization.
 * 
 * Based on Google Benchmark's DoNotOptimize and ClobberMemory.
 */

#ifndef COST_UTILS_BENCHMARK_UTILS_H
#define COST_UTILS_BENCHMARK_UTILS_H

#include <cstddef>

namespace benchmark_utils {

/**
 * DoNotOptimize - Prevents the compiler from optimizing away a value
 * 
 * This function forces the compiler to treat the value as if it's been
 * used, preventing dead code elimination.
 */
template <typename T>
inline void DoNotOptimize(T const& value) {
#if defined(__GNUC__) || defined(__clang__)
    // Use inline assembly to create a fake dependency on the value
    // The "r" constraint means the value should be in a register
    // The "g" constraint allows any general operand (register, memory, or immediate)
    // The empty clobber list and empty output means no side effects
    asm volatile("" : : "r,m"(value) : "memory");
#else
    // Fallback for other compilers: use a volatile pointer
    static_cast<void>(const_cast<T&>(value));
#endif
}

/**
 * DoNotOptimize overload for pointers
 */
template <typename T>
inline void DoNotOptimize(T* ptr) {
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : : "r,m"(ptr) : "memory");
#else
    static_cast<void>(ptr);
#endif
}

/**
 * ClobberMemory - Acts as a memory barrier
 * 
 * This forces the compiler to assume that all memory may have been
 * modified, preventing aggressive optimizations across this barrier.
 */
inline void ClobberMemory() {
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : : : "memory");
#else
    // Fallback: use a volatile variable
    static volatile int dummy = 0;
    dummy++;
#endif
}

/**
 * UseValue - Ensures a value is "used" to prevent optimization
 * 
 * This is a simpler alternative when you just want to mark a value as used.
 */
template <typename T>
inline void UseValue(T const& value) {
    DoNotOptimize(value);
}

/**
 * UsePointer - Ensures a pointer and its contents are "used"
 */
template <typename T>
inline void UsePointer(T* ptr, size_t size = 1) {
    if (ptr) {
        DoNotOptimize(ptr);
        // Also prevent optimization of the memory region
        asm volatile("" : : "r,m"(ptr[0]) : "memory");
    }
}

/**
 * PreventElision - Prevents the compiler from eliding a computation
 * 
 * Use this around expensive computations to ensure they're not optimized away.
 */
class PreventElision {
public:
    PreventElision() { ClobberMemory(); }
    ~PreventElision() { ClobberMemory(); }
    
    // Prevent copying
    PreventElision(const PreventElision&) = delete;
    PreventElision& operator=(const PreventElision&) = delete;
};

// Macro for easier usage in loops or blocks
#define PREVENT_OPTIMIZATION_SCOPE() \
    benchmark_utils::PreventElision _prevent_elision_guard

} // namespace benchmark_utils

#endif // COST_UTILS_BENCHMARK_UTILS_H


