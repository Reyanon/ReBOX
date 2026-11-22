// The sampled activity profiler (sampler.h, ADR-155). The emulation thread only stores what it does in rbx_prof_cur;
// this thread wakes every 250 us (sceKernelDelayThread), counts the value it finds, and every 5 seconds logs the shares
// by size with its sample count (a count far below 20000 a window: the thread was starved, the shares are biased).
// No clock calls on the emulation thread (a Vita clock read is a system call, ~0.5 us)
#include "frontend/sampler.h"

#include <stdio.h>
#include <string.h>

#include <psp2/io/stat.h>
#include <psp2/kernel/cpu.h>
#include <psp2/kernel/threadmgr.h>

#include "frontend/core_api.h"
#include "frontend/settings.h"
#include "frontend/threads.h"
#include "platform/log.h"
#include "platform/timer.h"

#ifdef RBX_ENABLE_LOG

#define PERIOD_US 250
#define REPORT_US 5000000u

static SceUID s_thread = -1;
static volatile int s_run;

static void report(const unsigned *n, unsigned total, uint64_t us)
{
	unsigned left[RBX_PROF_IDS + 1];
	memcpy(left, n, sizeof(left));
	char line[480];
	int len = snprintf(line, sizeof(line), "prof: n %u (%.0f ms):", total, us / 1000.0);
	for (;;) {   // (the largest share first; under 0.1% left out)
		int best = -1;
		for (int i = 0; i <= RBX_PROF_IDS; i++)
			if (left[i] && (best < 0 || left[i] > left[best]))
				best = i;
		if (best < 0 || left[best] * 1000u < total || len >= (int)sizeof(line) - 24)
			break;
		const char *nm = best < RBX_PROF_IDS ? rbx_prof_get_name((unsigned)best) : "?";
		if (nm[0])
			len += snprintf(line + len, sizeof(line) - len, " %s %.1f", nm, 100.0 * left[best] / total);
		else
			len += snprintf(line + len, sizeof(line) - len, " id%d %.1f", best, 100.0 * left[best] / total);
		left[best] = 0;
	}
	RBX_LOG("%s", line);
}

static int sampler_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	unsigned n[RBX_PROF_IDS + 1];
	unsigned total = 0;
	memset(n, 0, sizeof(n));
	uint64_t t0 = rbx_time_us();
	while (s_run) {
		sceKernelDelayThread(PERIOD_US);
		const unsigned a = rbx_prof_cur;
		n[a < RBX_PROF_IDS ? a : RBX_PROF_IDS]++;
		total++;
		if ((total & 255) == 0) {   // (the clock read here, on this thread, every 64 ms or so)
			const uint64_t t = rbx_time_us();
			if (t - t0 >= REPORT_US) {
				report(n, total, t - t0);
				memset(n, 0, sizeof(n));
				total = 0;
				t0 = t;
			}
		}
	}
	return sceKernelExitDeleteThread(0);
}

void rbx_sampler_start(void)
{
	if (s_thread >= 0)
		return;
	SceIoStat st;
	const int hi = sceIoGetstat(RBX_DATA_DIR "profhi", &st) >= 0;
	if (!hi && sceIoGetstat(RBX_DATA_DIR "prof", &st) < 0)
		return;
	// (CPU3: the second blitter lane's core, 110; lowest 191 = below everything there, or 64 = above it. CPU3 closed:
	// CPU2, below or above the render thread and the first lane)
	const int cpu3 = rbx_cpu3_usable();
	const int prio = hi ? 64 : 191;
	s_run = 1;
	s_thread = sceKernelCreateThread("rbx_sampler", sampler_thread, prio, 0x8000, 0,
		cpu3 ? 0x00080000 : SCE_KERNEL_CPU_MASK_USER_2, NULL);
	if (s_thread < 0) {
		RBX_LOG("prof: sampler thread create failed 0x%08x", s_thread);
		s_run = 0;
		return;
	}
	sceKernelStartThread(s_thread, 0, NULL);
	RBX_LOG("prof: sampler on CPU%d, priority %d, every %d us", cpu3 ? 3 : 2, prio, PERIOD_US);
}

void rbx_sampler_stop(void)
{
	if (s_thread < 0)
		return;
	s_run = 0;
	sceKernelWaitThreadEnd(s_thread, NULL, NULL);
	s_thread = -1;
}

#else

void rbx_sampler_start(void) {}
void rbx_sampler_stop(void) {}

#endif
