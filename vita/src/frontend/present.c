// The frame buffer (present.h, ADR-195). Without it the emulation thread draws each frame with vita2d and swaps; the
// swap (sceGxmDisplayQueueAddEntry) blocks while vita2d's display queue holds 2 swaps, so a slow frame can only use
// what that queue has banked. With it, a present thread does the drawing and the swap, and the emulation thread posts
// the frame number and goes on to the next frame; one posted frame may wait behind the one being shown, so the
// emulation banks one more frame of time on its light frames.
//
// Threads: only one thread builds GXM commands at a time. While the present thread runs, the emulation thread calls no
// vita2d drawing (it still writes the overlay canvases' textures and layer positions, as it did while the GPU read
// them late); before it draws itself (menu, state load, reset, game end) it drains the queue, after which the present
// thread waits on its semaphore and touches nothing. The neon corridor (border Graphic) is kicked and drawn by the
// present thread only, as it was by the emulation thread alone before.
#include "frontend/present.h"

#include <stdio.h>
#include <string.h>

#include <psp2/io/stat.h>
#include <psp2/kernel/threadmgr.h>
#include <vita2d.h>

#include "frontend/settings.h"
#include "frontend/video.h"
#include "platform/log.h"
#include "platform/timer.h"

// CPU0 with the emulation thread (main, 160): the same core does the same drawing as before, and above it so a frame
// is drawn and swapped as soon as the display has room (sound 64-100 are on CPU1, the render thread 80 on CPU2)
#define PRESENT_PRIORITY 150
#define PRESENT_STACK    (64 * 1024)

static SceUID s_th = -1;
static SceUID s_full = -1;    // posted frames not yet taken (0 or 1): the present thread waits on it
static SceUID s_empty = -1;   // room for a posted frame (0 or 1): the emulation thread waits on it
static volatile int s_quit;
static volatile uint64_t s_next;               // the posted frame (written before s_full is signalled)
static volatile uint32_t s_posted, s_shown;    // frames posted / swapped (rbx_present_drain)

#ifdef RBX_ENABLE_LOG
// (log builds) every 300 frames shown: the emulation thread's wait for room, the present thread's drawing and swap,
// and the vblanks with no new frame (the time between two swaps returning, past one vblank: the display showed the
// last frame again)
static struct {
	uint64_t wait_sum, draw_sum, swap_sum, last_swap;
	uint32_t wait_max, draw_max, swap_max, n, repeats, queued;
} s_st;
#endif

static int present_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	for (;;) {
		if (sceKernelWaitSema(s_full, 1, NULL) < 0 || s_quit)
			break;
		const uint64_t n = s_next;
		sceKernelSignalSema(s_empty, 1);   // (taken: the emulation thread may post the next one)
#ifdef RBX_ENABLE_LOG
		const uint64_t t0 = rbx_time_us();
#endif
		rbx_video_wait_rendered(n);   // (asleep: a spin here would hold the emulation thread off CPU0)
		vita2d_start_drawing();
		vita2d_clear_screen();
		rbx_video_draw_rendered(n);
		rbx_video_draw_layers();
		vita2d_end_drawing();
		// the corridor's moving parts for the next frame, made on CPU1 while this one waits for the display (its vertex
		// buffers rotate by three: the one written now was read by the scene two swaps back)
		rbx_video_kick_border();
#ifdef RBX_ENABLE_LOG
		const uint64_t t1 = rbx_time_us();
#endif
		vita2d_swap_buffers();
#ifdef RBX_ENABLE_LOG
		const uint64_t t2 = rbx_time_us();
		const uint32_t d = (uint32_t)(t1 - t0), s = (uint32_t)(t2 - t1);
		s_st.draw_sum += d;
		s_st.swap_sum += s;
		if (d > s_st.draw_max) s_st.draw_max = d;
		if (s > s_st.swap_max) s_st.swap_max = s;
		if (s_st.last_swap && t2 - s_st.last_swap > 16683 + 8000)
			s_st.repeats += (uint32_t)((t2 - s_st.last_swap - 8000) / 16683);
		s_st.last_swap = t2;
		{
			int v = 0;
			SceKernelSemaInfo si;
			si.size = sizeof(si);
			if (sceKernelGetSemaInfo(s_full, &si) >= 0)
				v = si.currentCount;
			s_st.queued += v > 0;   // (the next frame already waiting as this one went to the display)
		}
		if (++s_st.n == 300) {
			RBX_LOG("present: 300 frames, repeated vblanks %u, next frame queued %u%%, emu wait %.2f/%.2f ms, draw %.2f/%.2f ms, "
				"swap %.2f/%.2f ms (avg/max)", s_st.repeats, s_st.queued * 100 / 300, s_st.wait_sum / 300000.0,
				s_st.wait_max / 1000.0, s_st.draw_sum / 300000.0, s_st.draw_max / 1000.0, s_st.swap_sum / 300000.0,
				s_st.swap_max / 1000.0);
			const uint64_t keep = s_st.last_swap;
			memset(&s_st, 0, sizeof(s_st));
			s_st.last_swap = keep;
		}
#endif
		__atomic_add_fetch(&s_shown, 1, __ATOMIC_RELEASE);
	}
	return sceKernelExitDeleteThread(0);
}

int rbx_present_running(void) { return s_th >= 0; }

int rbx_present_start(void)
{
	if (s_th >= 0)
		return 0;
	s_quit = 0;
	s_posted = s_shown = 0;
#ifdef RBX_ENABLE_LOG
	memset(&s_st, 0, sizeof(s_st));
#endif
	s_full = sceKernelCreateSema("rbx_pfull", 0, 0, 1, NULL);
	s_empty = sceKernelCreateSema("rbx_pempty", 0, 1, 1, NULL);
	if (s_full >= 0 && s_empty >= 0)
		s_th = sceKernelCreateThread("rbx_present", present_thread, PRESENT_PRIORITY, PRESENT_STACK, 0,
			SCE_KERNEL_CPU_MASK_USER_0, NULL);
	if (s_th < 0 || sceKernelStartThread(s_th, 0, NULL) < 0) {
		RBX_LOG("present: thread failed 0x%08x (frames shown by the emulation thread)", (unsigned)s_th);
		if (s_th >= 0)
			sceKernelDeleteThread(s_th);
		s_th = -1;
		if (s_full >= 0)
			sceKernelDeleteSema(s_full);
		if (s_empty >= 0)
			sceKernelDeleteSema(s_empty);
		s_full = s_empty = -1;
		return -1;
	}
	RBX_LOG("present: frame buffer on (1 frame), present thread on CPU0, priority %d", PRESENT_PRIORITY);
	return 0;
}

void rbx_present_post(uint64_t frame_n)
{
#ifdef RBX_ENABLE_LOG
	const uint64_t t0 = rbx_time_us();
#endif
	sceKernelWaitSema(s_empty, 1, NULL);   // (a frame still queued: the emulation is a frame ahead, it waits here)
#ifdef RBX_ENABLE_LOG
	const uint32_t w = (uint32_t)(rbx_time_us() - t0);
	s_st.wait_sum += w;   // (written by this thread only; logged by the present thread, a torn read is a log line off)
	if (w > s_st.wait_max)
		s_st.wait_max = w;
#endif
	s_next = frame_n;
	s_posted++;
	sceKernelSignalSema(s_full, 1);
}

void rbx_present_drain(void)
{
	if (s_th < 0)
		return;
	while (__atomic_load_n(&s_shown, __ATOMIC_ACQUIRE) != s_posted)
		sceKernelDelayThread(200);
}

void rbx_present_stop(void)
{
	if (s_th < 0)
		return;
	rbx_present_drain();
	s_quit = 1;
	sceKernelSignalSema(s_full, 1);
	sceKernelWaitThreadEnd(s_th, NULL, NULL);
	s_th = -1;
	sceKernelDeleteSema(s_full);
	sceKernelDeleteSema(s_empty);
	s_full = s_empty = -1;
	RBX_LOG("present: frame buffer off");
}

// --- the setting ---
static int s_cv1k;        // the running game is a Cave CV1000 one (Auto = on)
static int s_force = -1;  // the data folder's runahead0 / runahead1 (testing): -1 none

void rbx_present_set_board(const char *board)
{
	s_cv1k = board && !strcmp(board, "Cave CV1000");
}

void rbx_present_check_files(void)
{
	SceIoStat st;
	if (sceIoGetstat(RBX_DATA_DIR "runahead1", &st) >= 0)
		s_force = 1;
	else if (sceIoGetstat(RBX_DATA_DIR "runahead0", &st) >= 0)
		s_force = 0;
	if (s_force >= 0)
		RBX_LOG("present: frame buffer %s for every game (runahead%d)", s_force ? "on" : "off", s_force);
}

int rbx_present_wanted(void)
{
	if (s_force >= 0)
		return s_force;
	return g_settings.frame_buffer == RBX_FRAMEBUF_AUTO ? s_cv1k : g_settings.frame_buffer == RBX_FRAMEBUF_ONE;
}

const char *rbx_present_auto_text(void)
{
	return s_cv1k ? "Auto (1 frame)" : "Auto (Off)";
}
