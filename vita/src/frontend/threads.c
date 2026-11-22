#include "frontend/threads.h"

#include <stdlib.h>
#include <string.h>

#include <psp2/kernel/cpu.h>
#include <psp2/kernel/threadmgr.h>
#include <psp2/io/stat.h>

#include "frontend/audio.h"
#include "frontend/core_api.h"
#include "frontend/settings.h"
#include "frontend/video.h"
#include "platform/log.h"
#include "platform/perf.h"

// Sound thread priority: just below the audio output thread (64). Higher than main (default 160)
#define SOUND_PRIORITY   80
// the F3 sound board on its own thread (start_aux3, rbx_threads_tick)
#ifndef RBX_AUX3_THREAD
#define RBX_AUX3_THREAD 1
#endif
#define SOUND_STACK_SIZE (256 * 1024)

static SceUID s_sound = -1;
static volatile int s_running;

void rbx_threads_pin_main(void)
{
	sceKernelChangeThreadCpuAffinityMask(sceKernelGetThreadId(), SCE_KERNEL_CPU_MASK_USER_0);
}

static int cpu3_probe(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	return 0;
}

static int s_ok = -1;   // (rbx_cpu3_usable: -1 not tried yet)
void rbx_cpu3_recheck(void) { s_ok = -1; }

int rbx_cpu3_usable(void)
{
	if (s_ok >= 0)
		return s_ok;
	s_ok = 0;
	// (a test switch: RBX_DATA_DIR "cpu3off" plans for three cores although a plugin opened CPU3, as on a stock system;
	// the measuring Vita has CoreUnlocker80000H on)
	SceIoStat st_off;
	if (sceIoGetstat(RBX_DATA_DIR "cpu3off", &st_off) >= 0) {
		RBX_LOG("threads: CPU3 taken as closed (cpu3off)");
		return s_ok;
	}
	const SceUID t = sceKernelCreateThread("rbx_cpu3", cpu3_probe, 0x60, 0x1000, 0, 0x00080000, NULL);
	if (t < 0) {
		RBX_LOG("threads: CPU3 closed (a thread on it refused: 0x%08x)", t);
		return s_ok;
	}
	SceUInt timeout = 200000;   // (us)
	const int st = sceKernelStartThread(t, 0, NULL);
	if (st >= 0 && sceKernelWaitThreadEnd(t, NULL, &timeout) >= 0) {
		s_ok = 1;
		sceKernelDeleteThread(t);
		RBX_LOG("threads: CPU3 open (a thread ran there)");
	} else   // (created but never run: left as it is, it cannot be deleted while waiting to run)
		RBX_LOG("threads: CPU3 closed (a thread created there did not run: start 0x%08x)", st);
	return s_ok;
}

int rbx_cpu3_mask_or(int fallback_mask) { return rbx_cpu3_usable() ? 0x00080000 : fallback_mask; }

static int sound_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	const int alen = rbx_core_audio_len();
	int16_t *buf = (int16_t *)calloc((size_t)alen * 2 + 16, sizeof(int16_t));

	uint64_t prev = rbx_time_us();
	while (s_running) {
		memset(buf, 0, (size_t)alen * 2 * sizeof(int16_t));
		// run one frame's worth, following main's progress (includes time waiting for main)
		rbx_core_sound_frame(buf);
		if (!s_running)
			break;
#ifdef RBX_AUDIO_DUMP
		rbx_audio_dump("ux0:data/rebox/dump_core.raw", 0, buf, alen);
#endif
		rbx_audio_push(buf, alen);

		uint64_t now = rbx_time_us();
		rbx_perf_record(RBX_PERF_SOUND, (uint32_t)(now - prev));
		prev = now;
	}
	free(buf);
	return sceKernelExitDeleteThread(0);
}

int rbx_threads_start_sound(void)
{
	s_running = 1;
	s_sound = sceKernelCreateThread("rbx_sound", sound_thread, SOUND_PRIORITY, SOUND_STACK_SIZE,
		0, SCE_KERNEL_CPU_MASK_USER_1, NULL);
	if (s_sound < 0) {
		RBX_LOG("threads: sound thread create failed 0x%08x", s_sound);
		s_running = 0;
		return -1;
	}
	sceKernelStartThread(s_sound, 0, NULL);
	RBX_LOG("threads: sound thread started on CPU1");
	return 0;
}

// The driver's worker job on CPU1 (rbx_aux: the F3 ES5510; rbx_core_ym_worker_run runs until rbx_aux_stop)
static SceUID s_ym = -1;
static int ym_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	rbx_core_ym_worker_run();
	return sceKernelExitDeleteThread(0);
}

// The driver's second job (rbx_aux2: the F3 ES5505 voices), until rbx_aux2_stop: on CPU1 with the first one (the F3
// ESP, compiled to ARM code, leaves room there), so that the drawing helper has CPU3 (measured: render 23.4 -> 20.1 ms
// ridingf, 16 -> 14.6 ms ringrage)
static SceUID s_aux2 = -1;
static int aux2_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	while (rbx_aux2_run()) {}
	return sceKernelExitDeleteThread(0);
}

static void start_aux2(void)
{
	if (!rbx_aux2_has_job())
		return;
	rbx_set_aux2_thread(1);
	// (with the third job on CPU1: on CPU3 above the drawing helper; it may lag, the ESP waits for it. A board without
	// the helper may ask for CPU3: spdheat's YM2149s, with the YM2203s' worker and the sound thread on CPU1)
	// (CPU3 closed: on CPU1 with the others)
	// (RBX_AUX2_BY_RENDER: on CPU2 below the render thread and the first worker there (90), a whole core: the CV1000
	// blitter, which on CPU3's half core fell a frame behind (ADR-105). RBX_DATA_DIR "aux2cpu3": on CPU3 as before)
	SceIoStat st_c3;
	const int alone = rbx_aux2_alone();
	const int cpu2 = alone == RBX_AUX2_BY_RENDER && sceIoGetstat(RBX_DATA_DIR "aux2cpu3", &st_c3) < 0;
	const int cpu3 = !cpu2 && alone && rbx_cpu3_usable();
	s_aux2 = sceKernelCreateThread("rbx_aux2", aux2_thread, cpu2 ? SOUND_PRIORITY + 20 : SOUND_PRIORITY, 64 * 1024, 0,
		cpu2 ? SCE_KERNEL_CPU_MASK_USER_2 : cpu3 ? 0x00080000 : SCE_KERNEL_CPU_MASK_USER_1, NULL);
	if (s_aux2 < 0) {
		RBX_LOG("threads: second worker create failed 0x%08x", s_aux2);
		rbx_set_aux2_thread(0);
		return;
	}
	sceKernelStartThread(s_aux2, 0, NULL);
	RBX_LOG("threads: second worker started on CPU%d", cpu2 ? 2 : cpu3 ? 3 : 1);
}

// The driver's third job (rbx_aux3: the F3 sound board, 68000 and the ES5505 control copy), until rbx_aux3_stop: on
// CPU3 above the drawing helper (it runs a scanline at a time right behind the main CPU and sleeps between)
static SceUID s_aux3 = -1, s_aux3_sema = -1;
static int s_aux3_on, s_aux3_alone;
static volatile int s_aux3_quit, s_aux3_asleep;
static void aux3_wake(void)
{
	if (__atomic_load_n(&s_aux3_asleep, __ATOMIC_ACQUIRE))   // (running, it sees the new lines itself)
		sceKernelSignalSema(s_aux3_sema, 1);
}
static int aux3_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	// (above the ESP job on CPU1: woken by the main CPU's kicks, it takes the core at once)
	while (!s_aux3_quit) {
		if (rbx_aux3_try())
			continue;
		__atomic_store_n(&s_aux3_asleep, 1, __ATOMIC_SEQ_CST);
		if (rbx_aux3_try()) {   // (a kick between the look and the flag)
			__atomic_store_n(&s_aux3_asleep, 0, __ATOMIC_SEQ_CST);
			continue;
		}
		SceUInt timeout = 2000;   // (us: a missed wake costs at most this)
		sceKernelWaitSema(s_aux3_sema, 1, &timeout);
		__atomic_store_n(&s_aux3_asleep, 0, __ATOMIC_SEQ_CST);
	}
	return sceKernelExitDeleteThread(0);
}

// The thread may run on CPU2-3 (wherever there is room; CPU1 has the ESP and the voices): it helps when the drawing leaves room (bubblem: slow seconds 25%
// -> 4%) and hurts when it does not (ridingf 4% -> 95%), so rbx_threads_tick turns it on and off by the render thread's
// load; off, the job runs on the emulation thread, TEN_LAG lines behind, with the same results
static void start_aux3(void)
{
	if (!RBX_AUX3_THREAD || !rbx_aux3_has_job())
		return;
	if (!rbx_cpu3_usable()) {   // (on CPU2 alone it slowed the render thread: ridingf 52.6 -> 46.7 fps)
		RBX_LOG("threads: third worker off (CPU3 closed: the sound board on the emulation thread)");
		return;
	}
	// (rbx_aux3_set_alone: alone on CPU3 below everything else of ours (110), on from the start: the CV1000 blitter's
	// second lane, whose work the others take over when it falls behind (ADR-120). RBX_DATA_DIR "e12nocpu3": not
	// started, the lane drawn by the second worker and the emulation thread)
	const int alone = rbx_aux3_alone();
	if (alone) {
		SceIoStat st_n3;
		if (sceIoGetstat(RBX_DATA_DIR "e12nocpu3", &st_n3) >= 0) {
			RBX_LOG("threads: third worker off (e12nocpu3)");
			return;
		}
	}
	s_aux3_sema = sceKernelCreateSema("rbx_aux3", 0, 0, 1000, NULL);
	s_aux3_quit = 0;
	s_aux3_on = 0;
	rbx_set_aux3_wake(aux3_wake);
	s_aux3 = sceKernelCreateThread("rbx_aux3", aux3_thread, alone ? SOUND_PRIORITY + 30 : SOUND_PRIORITY - 10, 256 * 1024,
		0, alone ? 0x00080000 /* CPU3 */ : 0x000c0000 /* CPU2-3 */, NULL);
	if (s_aux3 < 0) {
		RBX_LOG("threads: third worker create failed 0x%08x", s_aux3);
		rbx_set_aux3_thread(0);
		return;
	}
	sceKernelStartThread(s_aux3, 0, NULL);
	if (alone) {
		rbx_set_aux3_thread(1);
		s_aux3_on = 1;
		s_aux3_alone = 1;
		RBX_LOG("threads: third worker started on CPU3 (alone)");
		return;
	}
	RBX_LOG("threads: third worker started (CPU2-3, on by the drawing load)");
}

// M72-family boards without MCU: the Z80 (and the YM2151 it writes) on CPU1 right behind the V30, instead of the YM2151 worker
static int s_ym_is_z80;
static int z80_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	rbx_core_z80_worker_run();
	return sceKernelExitDeleteThread(0);
}

int rbx_threads_start_ym(void)
{
	if (rbx_core_z80_worker()) {
		rbx_core_z80_worker_enable(1);
		s_ym = sceKernelCreateThread("rbx_z80", z80_thread, SOUND_PRIORITY, 64 * 1024, 0, SCE_KERNEL_CPU_MASK_USER_1, NULL);
		if (s_ym < 0) {
			RBX_LOG("threads: z80 thread create failed 0x%08x", s_ym);
			rbx_core_z80_worker_enable(0);
			return -1;
		}
		s_ym_is_z80 = 1;
		sceKernelStartThread(s_ym, 0, NULL);
		RBX_LOG("threads: Z80 worker started on CPU1");
		return 0;
	}
	s_ym_is_z80 = 0;
	if (!rbx_core_ym_worker()) {   // (no first job: the second and third ones still on their threads, the CV1000 blitter)
		start_aux2();
		start_aux3();
		return s_aux2 >= 0 || s_aux3 >= 0 ? 0 : -1;
	}
	rbx_core_ym_worker_enable(1);
	// (a board asking for it: on CPU2 under the render thread, whose drawing is light there: the CV1000's YMZ770, with
	// CPU1 busy with the sound output and the blitter when CPU3 is closed. RBX_DATA_DIR "auxcpu1": on CPU1 as before)
	SceIoStat st_c1;
	const int cpu2 = rbx_aux_by_render() && sceIoGetstat(RBX_DATA_DIR "auxcpu1", &st_c1) < 0;
	// (testing, ADR-176: RBX_DATA_DIR "ymzhigh" = on CPU2 above the render thread (78 < 80), so that the render thread
	// waking at the frame end never takes the core from the worker in an entry)
	const int high = cpu2 && sceIoGetstat(RBX_DATA_DIR "ymzhigh", &st_c1) >= 0;
	s_ym = sceKernelCreateThread("rbx_ym", ym_thread, high ? SOUND_PRIORITY - 2 : cpu2 ? SOUND_PRIORITY + 10 : SOUND_PRIORITY,
		64 * 1024, 0, cpu2 ? SCE_KERNEL_CPU_MASK_USER_2 : SCE_KERNEL_CPU_MASK_USER_1, NULL);
	if (s_ym < 0) {
		RBX_LOG("threads: worker thread create failed 0x%08x", s_ym);
		rbx_core_ym_worker_enable(0);
		return -1;
	}
	sceKernelStartThread(s_ym, 0, NULL);
	RBX_LOG("threads: worker started on CPU%d%s", cpu2 ? 2 : 1, high ? " above the render thread (ymzhigh)" : "");
	start_aux2();
	start_aux3();
	return 0;
}

// The io worker (ADR-160: the CV1000 NAND read ahead, rbx_io_run): it mostly waits in the card reads, so it takes
// little of its core. CPU1 at 72: above the sound thread (80) so a queued read starts at once, below the audio output
// (64); its own priority there. RBX_DATA_DIR "nandcpu3": on CPU3 (at 72, between the sampler's 64 and the second
// blitter lane's 110), "nandsync" / "nandold": no worker (the reads on the emulation thread as before; main.c sets
// RBX_NAND_AHEAD=0 / RBX_NAND_OLD=1 for the driver)
#define IO_PRIORITY 72
static SceUID s_io = -1, s_io_sema = -1;
static volatile int s_io_quit;
static void io_wake(void)
{
	sceKernelSignalSema(s_io_sema, 1);   // (once a chunk or so: about one a frame; a wake too many only loops once more)
}
static int io_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	while (!__atomic_load_n(&s_io_quit, __ATOMIC_ACQUIRE)) {
		if (rbx_io_run())
			continue;
		SceUInt timeout = 20000;   // (us: a wake missed costs at most this, the emulation thread reads it itself)
		sceKernelWaitSema(s_io_sema, 1, &timeout);
	}
	return sceKernelExitDeleteThread(0);
}

void rbx_threads_start_io(void)
{
	if (s_io >= 0 || !rbx_io_wanted())
		return;
	SceIoStat st;
	if (sceIoGetstat(RBX_DATA_DIR "nandsync", &st) >= 0 || sceIoGetstat(RBX_DATA_DIR "nandold", &st) >= 0) {
		RBX_LOG("threads: io worker off (nandsync / nandold)");
		return;
	}
	const int cpu3 = sceIoGetstat(RBX_DATA_DIR "nandcpu3", &st) >= 0 && rbx_cpu3_usable();
	s_io_sema = sceKernelCreateSema("rbx_io", 0, 0, 1 << 20, NULL);
	if (s_io_sema < 0) {
		RBX_LOG("threads: io worker sema failed 0x%08x", s_io_sema);
		return;
	}
	s_io_quit = 0;
	rbx_set_io_wake(io_wake);
	s_io = sceKernelCreateThread("rbx_io", io_thread, IO_PRIORITY, 64 * 1024, 0,
		cpu3 ? 0x00080000 : SCE_KERNEL_CPU_MASK_USER_1, NULL);
	if (s_io < 0) {
		RBX_LOG("threads: io worker create failed 0x%08x", s_io);
		rbx_set_io_wake(NULL);
		sceKernelDeleteSema(s_io_sema);
		s_io_sema = -1;
		return;
	}
	sceKernelStartThread(s_io, 0, NULL);
	rbx_set_io_thread(1);
	RBX_LOG("threads: io worker started on CPU%d, priority %d", cpu3 ? 3 : 1, IO_PRIORITY);
}

void rbx_threads_stop_io(void)
{
	if (s_io < 0)
		return;
	rbx_set_io_thread(0);   // (first: nothing more queued; what is still queued the emulation thread reads or drops)
	__atomic_store_n(&s_io_quit, 1, __ATOMIC_RELEASE);
	sceKernelSignalSema(s_io_sema, 1);
	sceKernelWaitThreadEnd(s_io, NULL, NULL);
	s_io = -1;
	rbx_set_io_wake(NULL);
	sceKernelDeleteSema(s_io_sema);
	s_io_sema = -1;
}

// once a frame: the sound board thread on while the render thread has room (its busy time under 13.5 ms a frame), off
// over 14.5 (the job on the emulation thread)
void rbx_threads_tick(void)
{
	static int n;
	if (s_aux3 < 0 || s_aux3_alone || ++n < 60)
		return;
	n = 0;
	rbx_perf_stats st, se, sf;
	rbx_perf_stats_get(RBX_PERF_RENDER, &st);
	rbx_perf_stats_get(RBX_PERF_EMU, &se);
	rbx_perf_stats_get(RBX_PERF_FRAME, &sf);
	// on: the drawing threads lose some time to it; off: the emulation thread does its job. It goes off only when the
	// drawing is the limit (render near the frame and above the emulation thread), and on again when the emulation
	// thread is the limit or the drawing is light. (A fixed render limit, 14.5 ms, turned it off in pbobble3's heavy
	// play where the emulation thread then went over 16 ms: a quarter of the seconds slow)
	int on = s_aux3_on;
	// (and only with the frames late: pbobble3 at render 15.3 ms and 60 fps turned it off, the emulation thread went to
	// 19 ms and the sound ran out for two seconds, 11-03 30-minute run)
	if (s_aux3_on && st.avg_ms > 15.3f && st.avg_ms > se.avg_ms + 1.0f && sf.avg_ms > 16.8f)
		on = 0;
	else if (!s_aux3_on && (st.avg_ms < 13.5f || se.avg_ms > st.avg_ms + 1.0f))
		on = 1;
	if (on != s_aux3_on) {
		rbx_set_aux3_thread(on);
		s_aux3_on = on;
		RBX_LOG("threads: sound board thread %s (render %.1f, emulation %.1f ms)", on ? "on" : "off", st.avg_ms, se.avg_ms);
	}
}

void rbx_threads_stop(void)
{
	if (s_aux3 >= 0) {
		rbx_set_aux3_thread(0);   // (the queued work done here first)
		s_aux3_quit = 1;
		sceKernelSignalSema(s_aux3_sema, 1);
		sceKernelWaitThreadEnd(s_aux3, NULL, NULL);
		s_aux3 = -1;
		s_aux3_alone = 0;
		rbx_set_aux3_wake(NULL);
		sceKernelDeleteSema(s_aux3_sema);
		s_aux3_sema = -1;
	}
	if (s_aux2 >= 0) {
		rbx_set_aux2_thread(0);   // (the queued work done here first)
		rbx_aux2_stop();
		sceKernelWaitThreadEnd(s_aux2, NULL, NULL);
		s_aux2 = -1;
	}
	if (s_ym >= 0) {
		if (s_ym_is_z80)
			rbx_core_z80_worker_enable(0); // (between frames: the worker waits for the next one)
		else
			rbx_core_ym_worker_enable(0); // (main takes over after the worker has done all queued work)
		if (s_ym_is_z80)
			rbx_core_stop();
		else
			rbx_aux_stop();
		sceKernelWaitThreadEnd(s_ym, NULL, NULL);
		s_ym = -1;
	}
	if (s_sound < 0)
		return;
	s_running = 0;
	rbx_core_stop();
	sceKernelWaitThreadEnd(s_sound, NULL, NULL);
	s_sound = -1;
}
