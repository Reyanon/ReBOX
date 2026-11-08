#ifndef RBX_PLATFORM_TIMER_H
#define RBX_PLATFORM_TIMER_H

#include <stdint.h>
#include <psp2/kernel/processmgr.h>

// Process-relative monotonic time (microseconds)
static inline uint64_t rbx_time_us(void)
{
	return (uint64_t)sceKernelGetProcessTimeWide();
}

#endif
