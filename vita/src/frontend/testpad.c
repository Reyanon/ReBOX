// Test input script (log builds only). See testpad.h
#include "frontend/testpad.h"

#ifdef RBX_ENABLE_LOG
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <psp2/ctrl.h>
#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>
#include <psp2/kernel/threadmgr.h>

#include "frontend/settings.h"
#include "platform/log.h"

#define KEYS_PATH RBX_DATA_DIR "keys"
#define MAX_STEPS 512

typedef struct { uint32_t buttons; int frames, gap; int touch, x0, y0, x1, y1; } Step;
static Step s_step[MAX_STEPS];
static int s_n, s_i, s_t;   // steps, current step, frame within it
static unsigned s_poll;     // frames since the file was last looked for
static int s_tx, s_ty, s_ton;

static uint32_t key_bits(const char *name)
{
	static const struct { const char *n; uint32_t b; } k[] = {
		{ "up", SCE_CTRL_UP }, { "down", SCE_CTRL_DOWN }, { "left", SCE_CTRL_LEFT }, { "right", SCE_CTRL_RIGHT },
		{ "cross", SCE_CTRL_CROSS }, { "circle", SCE_CTRL_CIRCLE }, { "square", SCE_CTRL_SQUARE },
		{ "triangle", SCE_CTRL_TRIANGLE }, { "l", SCE_CTRL_LTRIGGER }, { "r", SCE_CTRL_RTRIGGER },
		{ "start", SCE_CTRL_START }, { "select", SCE_CTRL_SELECT },
	};
	uint32_t b = 0;
	char buf[64];
	snprintf(buf, sizeof(buf), "%s", name);
	for (char *p = strtok(buf, "+"); p; p = strtok(NULL, "+"))
		for (unsigned i = 0; i < sizeof(k) / sizeof(k[0]); i++)
			if (!strcmp(p, k[i].n))
				b |= k[i].b;
	return b;
}

static void load(void)
{
	FILE *f = fopen(KEYS_PATH, "r");
	if (!f)
		return;
	char tok[96];
	s_n = s_i = s_t = 0;
	while (s_n < MAX_STEPS && fscanf(f, "%95s", tok) == 1) {
		Step st = { 0, 4, 8, 0, 0, 0, 0, 0 };
		int rep = 1, a, b, c, d, e;
		char name[64];
		if (sscanf(tok, "wait:%d", &a) == 1) {
			st.frames = 0;
			st.gap = a;
		} else if (sscanf(tok, "touch:%d:%d:%d", &a, &b, &c) == 3) {
			st.touch = 1;
			st.x0 = st.x1 = a;
			st.y0 = st.y1 = b;
			st.frames = c;
		} else if (sscanf(tok, "drag:%d:%d:%d:%d:%d", &a, &b, &c, &d, &e) == 5) {
			st.touch = 1;
			st.x0 = a; st.y0 = b; st.x1 = c; st.y1 = d;
			st.frames = e;
		} else if (sscanf(tok, "hold:%63[^:]:%d", name, &a) == 2) {
			st.buttons = key_bits(name);
			st.frames = a;
		} else {
			char *star = strchr(tok, '*');
			if (star) {
				*star = 0;
				rep = atoi(star + 1);
			}
			st.buttons = key_bits(tok);
		}
		for (int r = 0; r < rep && s_n < MAX_STEPS; r++)
			s_step[s_n++] = st;
	}
	fclose(f);
	sceIoRemove(KEYS_PATH);
	RBX_LOG("testpad: %d steps", s_n);
}

// the watcher (rbx_testfile_seen): one flag a file, set by the thread, taken by the frame loop
static const char *const s_watch[] = { KEYS_PATH, RBX_DATA_DIR "shotgame", RBX_DATA_DIR "shotfb" };
#define NWATCH (sizeof(s_watch) / sizeof(s_watch[0]))
static volatile int s_seen[NWATCH];
static volatile int s_watch_on, s_watch_quit;
static SceUID s_watch_th = -1;
static int watch_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	while (!__atomic_load_n(&s_watch_quit, __ATOMIC_ACQUIRE)) {
		for (unsigned i = 0; i < NWATCH; i++) {
			SceIoStat st;
			if (sceIoGetstat(s_watch[i], &st) >= 0)
				__atomic_store_n(&s_seen[i], 1, __ATOMIC_RELEASE);
		}
		sceKernelDelayThread(250000);
	}
	return sceKernelExitDeleteThread(0);
}
void rbx_testfiles_start(void)
{
	if (s_watch_th >= 0)
		return;
	s_watch_quit = 0;
	// (CPU1 at 190: below everything there, the sound thread 80 and the io worker 72; the card stat waits off the CPU)
	s_watch_th = sceKernelCreateThread("rbx_testfiles", watch_thread, 190, 0x4000, 0, 0x00020000 /* CPU1 */, NULL);
	if (s_watch_th < 0 || sceKernelStartThread(s_watch_th, 0, NULL) < 0) {
		RBX_LOG("testpad: file watcher not started (0x%08x): the frame loop looks itself", (unsigned)s_watch_th);
		if (s_watch_th >= 0)
			sceKernelDeleteThread(s_watch_th);
		s_watch_th = -1;
		return;
	}
	__atomic_store_n(&s_watch_on, 1, __ATOMIC_RELEASE);
}
void rbx_testfiles_stop(void)
{
	if (s_watch_th < 0)
		return;
	__atomic_store_n(&s_watch_on, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&s_watch_quit, 1, __ATOMIC_RELEASE);
	sceKernelWaitThreadEnd(s_watch_th, NULL, NULL);
	s_watch_th = -1;
}
int rbx_testfile_seen(const char *path)
{
	if (!__atomic_load_n(&s_watch_on, __ATOMIC_ACQUIRE))
		return 1;
	for (unsigned i = 0; i < NWATCH; i++)
		if (!strcmp(path, s_watch[i]))
			return __atomic_exchange_n(&s_seen[i], 0, __ATOMIC_ACQ_REL);
	return 1;   // (not watched: the caller looks)
}

uint32_t rbx_testpad_step(void)
{
	s_ton = 0;
	if (s_i >= s_n) {
		if (++s_poll % 30 == 0 && rbx_testfile_seen(KEYS_PATH)) {
			SceIoStat st;
			if (sceIoGetstat(KEYS_PATH, &st) >= 0)
				load();
		}
		return 0;
	}
	const Step *st = &s_step[s_i];
	uint32_t b = 0;
	if (s_t < st->frames) {
		b = st->buttons;
		if (st->touch) {
			const int k = st->frames > 1 ? st->frames - 1 : 1;
			s_tx = st->x0 + (st->x1 - st->x0) * s_t / k;
			s_ty = st->y0 + (st->y1 - st->y0) * s_t / k;
			s_ton = 1;
		}
	}
	if (++s_t >= st->frames + st->gap) {
		s_i++;
		s_t = 0;
		if (s_i == s_n)
			RBX_LOG("testpad: done");
	}
	return b;
}

int rbx_testpad_touch(int *x, int *y)
{
	if (!s_ton)
		return 0;
	*x = s_tx;
	*y = s_ty;
	return 1;
}
#endif
