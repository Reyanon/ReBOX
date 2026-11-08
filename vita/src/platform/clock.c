// Vita clock boost (docs/RULES.md 3: ARM 444MHz at app start, GPU/bus raised)
#include "platform/clock.h"

#include <psp2/power.h>

#include "platform/log.h"
#include "platform/timer.h"

// Target ARM clock. Acceptance criterion is 444 (docs/RULES.md 1). For testing, e.g. -DRBX_ARM_MHZ=500
#ifndef RBX_ARM_MHZ
#define RBX_ARM_MHZ 444
#endif

void rbx_clock_boost(void)
{
	int ra = scePowerSetArmClockFrequency(RBX_ARM_MHZ);
	int rb = scePowerSetBusClockFrequency(222);
	int rg = scePowerSetGpuClockFrequency(222);
	int rx = scePowerSetGpuXbarClockFrequency(166);

	RBX_LOG("clock: set(arm %d) ret arm=0x%08x bus=0x%08x gpu=0x%08x xbar=0x%08x", RBX_ARM_MHZ, ra, rb, rg, rx);
	(void)ra; (void)rb; (void)rg; (void)rx;
	rbx_clock_log(1);
}

// Estimate the real ARM clock (MHz). Plugins such as PSVshell hook scePowerGet* and
// may return wrong values, so time a loop with 1 cycle per iteration.
int rbx_clock_measure_arm_mhz(void)
{
	const unsigned iters = 20000000u;
	uint64_t t0 = rbx_time_us();
	unsigned n = iters;
	__asm__ volatile(
		"1:\n\t"
		"subs %0, %0, #1\n\t"
		"bne 1b\n\t"
		: "+r"(n) : : "cc");
	uint64_t dt = rbx_time_us() - t0;
	return dt ? (int)(iters / dt) : 0;
}

// measure != 0 also measures the real clock. Holds the CPU for ~45ms, so use only at startup
// (periodic measurement stutters frames each time - PERF.md P6)
void rbx_clock_log(int measure)
{
	// (the battery too: a long device test session on battery ended with the Vita switched off, 2026-11-02)
	RBX_LOG("clock: arm=%d bus=%d gpu=%d xbar=%d (measured arm~%dMHz) battery %d%%%s%s",
		scePowerGetArmClockFrequency(), scePowerGetBusClockFrequency(),
		scePowerGetGpuClockFrequency(), scePowerGetGpuXbarClockFrequency(),
		measure ? rbx_clock_measure_arm_mhz() : 0, scePowerGetBatteryLifePercent(),
		scePowerIsBatteryCharging() ? " charging" : "", scePowerIsLowBattery() ? " LOW" : "");
	(void)measure;
}

#ifdef RBX_CLOCK_SWEEP
#include <psp2/kernel/threadmgr.h>

// Diagnostic: step through ARM clocks, logging return value, read-back and measured value
void rbx_clock_sweep(void)
{
	static const int steps[] = { 111, 222, 333, 444, 500 };
	for (unsigned i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
		int r = scePowerSetArmClockFrequency(steps[i]);
		sceKernelDelayThread(100 * 1000);
		RBX_LOG("sweep: set %d ret=0x%08x get=%d measured~%dMHz",
			steps[i], r, scePowerGetArmClockFrequency(), rbx_clock_measure_arm_mhz());
	}
}
#endif
