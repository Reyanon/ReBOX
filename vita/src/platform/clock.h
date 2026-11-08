#ifndef RBX_PLATFORM_CLOCK_H
#define RBX_PLATFORM_CLOCK_H

// Raise ARM to 444MHz, bus 222MHz, GPU 222MHz, GPU Xbar 166MHz.
void rbx_clock_boost(void);

// Log current clocks over UDP. If measure != 0, also measure (about 45ms, startup only)
void rbx_clock_log(int measure);

// Estimate the real ARM clock (MHz) with a fixed-cycle loop (about 45-180ms)
int rbx_clock_measure_arm_mhz(void);

#ifdef RBX_CLOCK_SWEEP
// Diagnostic clock step test (-DRBX_CLOCK_SWEEP)
void rbx_clock_sweep(void);
#endif

#endif
