#ifndef RBX_PLATFORM_PERF_H
#define RBX_PLATFORM_PERF_H

#include <stdint.h>

#include "platform/timer.h"

// per-thread ms/frame measurement (docs/RULES.md section 7: average, max, 99th percentile)
//
// Each slot is written by one thread only (single producer). The HUD side computes stats
// read-only, so a sample may appear torn, which is acceptable for display.

typedef enum {
	RBX_PERF_EMU,    // CPU0: V33 + video registers + input (excluding vsync wait)
	RBX_PERF_SOUND,  // CPU1: V35 + YM2151 + GA20
	RBX_PERF_RENDER, // CPU2: tile/sprite compositing
	RBX_PERF_FRAME,  // frame interval (incl. vsync). 16.67ms at 60fps
	RBX_PERF_COUNT
} rbx_perf_slot;

#define RBX_PERF_WINDOW 120 // sample count (2 s)

typedef struct {
	float avg_ms;
	float max_ms;
	float p99_ms;
} rbx_perf_stats;

void rbx_perf_record(rbx_perf_slot slot, uint32_t us);
void rbx_perf_stats_get(rbx_perf_slot slot, rbx_perf_stats *out);
const char *rbx_perf_slot_name(rbx_perf_slot slot);

// section timing helpers
//   uint64_t t0 = rbx_time_us(); ... ; rbx_perf_end(RBX_PERF_EMU, t0);
static inline void rbx_perf_end(rbx_perf_slot slot, uint64_t t0)
{
	rbx_perf_record(slot, (uint32_t)(rbx_time_us() - t0));
}

#endif
