#include "frontend/audio.h"

#include <string.h>

#include <psp2/audioout.h>
#include <psp2/kernel/threadmgr.h>

#include "frontend/audio_gain.h"
#include "frontend/resample.h"
#include "platform/log.h"

#ifdef RBX_AUDIO_DUMP
#include <stdio.h>
#include <stdlib.h>
#define RBX_AUDIO_DUMP_SAMPLES (48000 * 60) // 60 s (sample pairs)
static uint32_t *s_dump_buf[2];
static uint32_t s_dump_n[2];
static int s_dump_done[2];

void rbx_audio_dump(const char *path, int slot, const int16_t *samples, int frames)
{
	if (s_dump_done[slot])
		return;
	if (!s_dump_buf[slot])
		s_dump_buf[slot] = (uint32_t *)malloc(RBX_AUDIO_DUMP_SAMPLES * sizeof(uint32_t));
	const uint32_t *src = (const uint32_t *)samples;
	for (int i = 0; i < frames && s_dump_n[slot] < RBX_AUDIO_DUMP_SAMPLES; i++)
		s_dump_buf[slot][s_dump_n[slot]++] = src[i];
	if (s_dump_n[slot] >= RBX_AUDIO_DUMP_SAMPLES) {
		FILE *f = fopen(path, "wb");
		if (f) {
			fwrite(s_dump_buf[slot], sizeof(uint32_t), s_dump_n[slot], f);
			fclose(f);
		}
		s_dump_done[slot] = 1;
		RBX_LOG("audio: dump slot %d -> %s (%u samples)", slot, path, s_dump_n[slot]);
	}
}
#endif

#define GRAIN       256              // samples per sceAudioOutOutput call (multiple of 64)
#define RING_FRAMES 8192             // ring buffer size (sample pairs, power of 2, ~170ms)
#define RING_MASK   (RING_FRAMES - 1)
#define PREFILL     1600             // prefill before start (2 frames; = DRC_TARGET, the start waits for s_drc_target)
#define OUT_BUFS    3                // number of rotating output buffers

// single-producer (emu thread) / single-consumer (output thread) ring buffer. Sample pairs stored as uint32
static uint32_t s_ring[RING_FRAMES];
static volatile uint32_t s_head; // total written by producer
static volatile uint32_t s_tail; // total read by consumer

static volatile int32_t s_drc_target;   // (DRC below)
static int s_port = -1;
static SceUID s_thread = -1;
static volatile int s_running;
static volatile int s_reset; // main asks the output thread to start the ring over
static volatile uint32_t s_underruns, s_overflows;

static int audio_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	// sceAudioOutOutput may return before the hardware has read the whole buffer. Overwriting a single buffer right away
	// corrupts the tail of each grain (256 samples), producing a 187.5Hz tone -> rotate 3 buffers
	static uint32_t bufs[OUT_BUFS][GRAIN] __attribute__((aligned(64)));
	int cur = 0;
	int started = 0;

	while (s_running) {
		if (__atomic_load_n(&s_reset, __ATOMIC_ACQUIRE)) { // (a new game: empty ring, prefill again)
			started = 0;
			__atomic_store_n(&s_reset, 0, __ATOMIC_RELEASE);
		}
		uint32_t head = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE);
		uint32_t tail = s_tail;
		uint32_t avail = head - tail;

		if (!started) {
			if (avail < (uint32_t)s_drc_target) {   // (PREFILL, and the frame buffer's frame: the DRC target, ADR-195)
				sceKernelDelayThread(1000);
				continue;
			}
			started = 1;
		}

		uint32_t *buf = bufs[cur];
		cur = (cur + 1) % OUT_BUFS;
		uint32_t n = avail < GRAIN ? avail : GRAIN;
		for (uint32_t i = 0; i < n; i++)
			buf[i] = s_ring[(tail + i) & RING_MASK];
		if (n < GRAIN) {
			memset(buf + n, 0, (GRAIN - n) * sizeof(uint32_t));
			s_underruns++;
		}
		__atomic_store_n(&s_tail, tail + n, __ATOMIC_RELEASE);

#ifdef RBX_AUDIO_DUMP
		if (started)
			rbx_audio_dump("ux0:data/rebox/dump_out.raw", 1, (const int16_t *)buf, GRAIN);
#endif
		sceAudioOutOutput(s_port, buf); // blocks until the hardware consumes it
	}
	return 0;
}

static int32_t s_drc_err;
static int s_block = 1;   // (ADR-180: 0 = the gain a pair at a time as before, "feaudold")
static volatile int32_t s_drc_ppm;
// The output port and thread live as long as the app (in-app game switching): each game only starts the ring over.
// (Opening a port per game leaked ports: a port still in sceAudioOutOutput is not released, and after 8 games the
// open failed, 0x80260005)
static void ring_restart(void)
{
	if (s_thread >= 0) { // the output thread drops its prefilled state (waits at most one grain)
		__atomic_store_n(&s_reset, 1, __ATOMIC_RELEASE);
		for (int i = 0; i < 200 && __atomic_load_n(&s_reset, __ATOMIC_ACQUIRE); i++)
			sceKernelDelayThread(500);
	}
	s_tail = s_head;
	s_drc_err = 0;
	s_drc_ppm = 0;
	s_underruns = s_overflows = 0;
	rbx_resample_init();
}

int rbx_audio_init(void)
{
	if (s_port >= 0) {
		ring_restart();
		return 0;
	}
	s_head = s_tail = 0;
	ring_restart();
	s_port = sceAudioOutOpenPort(SCE_AUDIO_OUT_PORT_TYPE_MAIN, GRAIN, RBX_AUDIO_RATE,
		SCE_AUDIO_OUT_MODE_STEREO);
	if (s_port < 0) {
		RBX_LOG("audio: open port failed 0x%08x", s_port);
		return -1;
	}

	s_running = 1;
	// highest user priority. Core affinity decided in M3
	s_thread = sceKernelCreateThread("rbx_audio", audio_thread, 64, 0x4000, 0, 0, NULL);
	if (s_thread < 0) {
		RBX_LOG("audio: thread create failed 0x%08x", s_thread);
		sceAudioOutReleasePort(s_port);
		s_port = -1;
		return -1;
	}
	sceKernelStartThread(s_thread, 0, NULL);
	RBX_LOG("audio: %d Hz, grain %d, ring %d, rounding %s, %s", RBX_AUDIO_RATE, GRAIN, RING_FRAMES,
		rbx_resample_fast_rint() ? "vcvtr" : "lrintf", rbx_resample_neon() && s_block ? "NEON block" : "as before");
	return 0;
}

void rbx_audio_fini(void)
{
	ring_restart(); // (game end: what is left in the ring is not played; the port stays for the next game)
	gain_reset();
}

void rbx_audio_shutdown(void)
{
	if (s_thread >= 0) {
		s_running = 0;
		sceKernelWaitThreadEnd(s_thread, NULL, NULL); // (the thread ends itself; it is not deleted while running)
		s_thread = -1;
	}
	if (s_port >= 0) {
		const int r = sceAudioOutReleasePort(s_port);
		if (r < 0)
			RBX_LOG("audio: release port failed 0x%08x", r);
		s_port = -1;
	}
}

// --- dynamic rate control (DRC, docs/RULES.md section 3) ---
// The Vita display is 59.94Hz, so 800 samples per frame falls slightly short of 48kHz consumption. Input is stretched or
// shrunk within ±0.5% (16-tap resampling, resample.c) to keep the ring buffer fill at the target (2 frames).
#define DRC_TARGET   1600                  // target fill (sample pairs); also the gain's scale (ppm per pair of error)
#define DRC_MAX_PPM  5000                  // max adjustment 0.5%
// (ADR-195) the frame buffer shows each picture a frame later: the ring holds a frame more (800 pairs) so the sound
// comes as much later and stays with the picture. The gain stays that of DRC_TARGET
static volatile int32_t s_drc_target = DRC_TARGET;   // (declared above for the prefill)
void rbx_audio_set_extra_frames(int frames)
{
	s_drc_target = DRC_TARGET + (frames > 0 ? frames : 0) * (RBX_AUDIO_RATE / 60);
}
static int16_t s_drc_out[(RBX_RESAMPLE_MAX_IN + RBX_RESAMPLE_MAX_IN / 64 + 4) * 2] __attribute__((aligned(16)));
void rbx_audio_set_old(int old)
{
	s_block = !old;
	rbx_resample_set_neon(!old);
}
// s_drc_err: low-passed fill error (sample pairs * 256); s_drc_ppm: current adjustment (stats). Both above rbx_audio_init

static void ring_put(uint32_t *head, uint32_t tail, int16_t l, int16_t r)
{
	if (*head - tail >= RING_FRAMES) {
		s_overflows++;
		return;
	}
	s_ring[*head & RING_MASK] = (uint16_t)l | ((uint32_t)(uint16_t)r << 16);
	(*head)++;
}

void rbx_audio_push(const int16_t *samples, int frames)
{
	uint32_t head = s_head;
	uint32_t tail = __atomic_load_n(&s_tail, __ATOMIC_ACQUIRE);
	int32_t fill = (int32_t)(head - tail);

	// track the error slowly so pitch doesn't wobble (1/16 low-pass)
	s_drc_err += (((fill - s_drc_target) << 8) - s_drc_err) >> 4;
	int32_t ppm = (int32_t)((int64_t)(s_drc_err >> 8) * DRC_MAX_PPM / DRC_TARGET);
	if (ppm > DRC_MAX_PPM) ppm = DRC_MAX_PPM;
	if (ppm < -DRC_MAX_PPM) ppm = -DRC_MAX_PPM;
	s_drc_ppm = ppm;
	// input advance per output sample. With a high fill (ppm>0) advance faster, producing less output
	uint32_t step = (uint32_t)(65536 + (int64_t)65536 * ppm / 1000000);

	int n = rbx_resample(samples, frames, step, s_drc_out);
	if (s_block) {   // (ADR-180: the gain over the block in registers; the ring's room taken once, tail as read above)
		const uint32_t used = head - tail;
		const int room = used >= RING_FRAMES ? 0 : (int)(RING_FRAMES - used), nw = n < room ? n : room;
		gain_block(s_drc_out, n, s_ring, head, RING_MASK, nw);
		s_overflows += (uint32_t)(n - nw);
		__atomic_store_n(&s_head, head + (uint32_t)nw, __ATOMIC_RELEASE);
		return;
	}
	for (int i = 0; i < n; i++) {
		const int32_t l = dc_block(0, s_drc_out[i * 2]), r = dc_block(1, s_drc_out[i * 2 + 1]);
		gain_track(l, r);
		ring_put(&head, tail, out_gain(l), out_gain(r));
	}
	__atomic_store_n(&s_head, head, __ATOMIC_RELEASE);
}

void rbx_audio_get_stats(rbx_audio_stats *out)
{
	out->underruns = s_underruns;
	out->overflows = s_overflows;
	out->fill = (int)(s_head - s_tail);
	out->drc_ppm = s_drc_ppm;
}
