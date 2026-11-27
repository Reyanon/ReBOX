// Frontend view of the ReBOX core (core_api.h)
#include "frontend/core_api.h"

#include "platform/log.h"
#include "zip.h"
#include "platform/timer.h"
#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

static uint32_t s_in[4];
static uint32_t s_sys;
static uint32_t s_keys_sys;   // (the test / service switch keys of the Controls page: kept apart from the medal coin)
static int s_test_pulse;      // (frames the menu's "Test mode" still holds the test switch)

void rbx_core_set_input(int index, int pressed)
{
	static const uint32_t base_bits[8] = { RBX_IN_COIN, RBX_IN_START, RBX_IN_UP, RBX_IN_DOWN, RBX_IN_LEFT, RBX_IN_RIGHT,
	                                       RBX_IN_B1, RBX_IN_B2 };
	int p = -1;
	uint32_t bit = 0;
	if (index >= RBX_IN_P1_COIN && index <= RBX_IN_P4_B2) {
		p = index / 8;
		bit = base_bits[index % 8];
	} else if (index >= RBX_IN_P1_B3 && index <= RBX_IN_P2_B6) {
		int k = index - RBX_IN_P1_B3;
		p = k / 4;
		bit = RBX_IN_B3 << (k % 4);
	} else if (index >= RBX_IN_P1_RUP && index <= RBX_IN_P2_RRIGHT) {
		int k = index - RBX_IN_P1_RUP;
		p = k / 4;
		bit = RBX_IN_RUP << (k % 4);
	}
	if (p < 0)
		return;
	if (pressed)
		s_in[p] |= bit;
	else
		s_in[p] &= ~bit;
}

void rbx_core_set_service(int test, int coin)
{
	s_sys = (test ? RBX_SYS_TEST : 0) | (coin ? RBX_SYS_SERVICE : 0);
}

void rbx_core_set_test_keys(int test, int service)
{
	s_keys_sys = (test ? RBX_SYS_TEST : 0) | (service ? RBX_SYS_SERVICE : 0);
}

void rbx_core_test_pulse(void) { s_test_pulse = 8; }

int rbx_core_has_test(void)
{
	const int g = rbx_running_game();
	return g >= 0 && rbx_game_has_test(g);
}

const char *rbx_core_game_set(int i) { return rbx_game_set(i); }
const char *rbx_core_game_parent(int i) { return rbx_game_parent(i); }
const char *rbx_core_game_title(int i) { return rbx_game_title(i); }
const char *rbx_core_game_board(int i) { return rbx_game_board(i); }
int rbx_core_game_count(void) { return rbx_game_count(); }
const char *rbx_core_game_name(void) { int g = rbx_running_game(); return g >= 0 ? rbx_game_title(g) : ""; }
const char *rbx_core_set_name(void) { int g = rbx_running_game(); return g >= 0 ? rbx_game_set(g) : ""; }
const char *rbx_core_button_label(int b)
{
	const int g = rbx_running_game();
	return g >= 0 ? rbx_game_button_label(g, b) : NULL;
}

int rbx_core_twin_stick(void)   // (each frame: the answer kept for the game)
{
	static int last = -2, twin;
	const int g = rbx_running_game();
	if (g != last) {
		last = g;
		twin = g >= 0 && rbx_game_twin_stick(g);
	}
	return twin;
}
int rbx_core_button_count(void) { int g = rbx_running_game(); return g >= 0 ? rbx_game_buttons(g) : 2; }

void rbx_core_screen(int *w, int *h, int *flags)
{
	rbx_screen(w, h);
	int g = rbx_running_game();
	int rot = g >= 0 ? rbx_game_rot(g) : 0;
	// (MAME ROTn: shown turned n degrees clockwise; VERTICAL is a quarter turn counter-clockwise, FLIPPED a half turn)
	*flags = rot == 90 ? (RBX_SCREEN_VERTICAL | RBX_SCREEN_FLIPPED) : rot == 180 ? RBX_SCREEN_FLIPPED
	       : rot == 270 ? RBX_SCREEN_VERTICAL : 0;
}

int rbx_core_run_frame(void *fb, int pitch, int16_t *audio, int max)
{
	for (int p = 0; p < 4; p++)
		rbx_set_input(p, s_in[p]);
	uint32_t sys = s_sys | s_keys_sys;
	if (s_test_pulse > 0) {   // (pressed for a few frames, then let go: the boards' test switches are buttons)
		s_test_pulse--;
		sys |= RBX_SYS_TEST;
	}
	rbx_set_system(sys);
	return rbx_run_frame(fb, pitch, audio, max);
}

int rbx_core_state_exists(const char *path)
{
	SceIoStat st;
	return sceIoGetstat(path, &st) >= 0;
}

// the state written in parts through a small buffer, when no buffer of its size is left (PGM2 orleg2: ROMs in 130 of
// the 152MB heap)
typedef struct { SceUID fd; uint32_t n; uint8_t b[8192]; } state_out;

static int state_flush(state_out *o)
{
	if (o->n && sceIoWrite(o->fd, o->b, o->n) != (int)o->n)
		return -1;
	o->n = 0;
	return 0;
}

static int state_put(void *user, const void *p, uint32_t n)
{
	state_out *o = (state_out *)user;
	const uint8_t *s = (const uint8_t *)p;
	if (n >= sizeof(o->b)) {
		if (state_flush(o))
			return -1;
		return sceIoWrite(o->fd, s, n) == (int)n ? 0 : -1;
	}
	if (o->n + n > sizeof(o->b) && state_flush(o))
		return -1;
	memcpy(o->b + o->n, s, n);
	o->n += n;
	return 0;
}

static int state_save_stream(const char *path)
{
	static state_out o;
	o.fd = sceIoOpen(path, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
	if (o.fd < 0)
		return -1;
	o.n = 0;
	int rc = rbx_state_stream(state_put, &o);
	if (!rc)
		rc = state_flush(&o);
	sceIoClose(o.fd);
	if (rc)
		sceIoRemove(path);
	RBX_LOG("state: no %u KB buffer free, written in parts: rc %d", rbx_state_size() >> 10, rc);
	return rc;
}

int rbx_core_state_save(const char *path)
{
	const uint32_t n = rbx_state_size();
	if (!n)
		return -1;
	uint8_t *buf = (uint8_t *)malloc(n);
	if (!buf)
		return state_save_stream(path);
	int rc = -1;
	if (!rbx_state_write(buf, n)) {
		const SceUID fd = sceIoOpen(path, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
		if (fd >= 0) {
			rc = sceIoWrite(fd, buf, n) == (int)n ? 0 : -1;
			sceIoClose(fd);
		}
	}
	free(buf);
	return rc;
}

static int state_get(void *user, void *p, uint32_t n)
{
	return sceIoRead(*(SceUID *)user, p, n) == (int)n ? 0 : -1;
}

static int state_skip(void *user, uint32_t n)
{
	return sceIoLseek(*(SceUID *)user, n, SCE_SEEK_CUR) < 0 ? -1 : 0;
}

static int state_rewind(void *user)
{
	return sceIoLseek(*(SceUID *)user, 0, SCE_SEEK_SET) < 0 ? -1 : 0;
}

int rbx_core_state_load(const char *path)
{
	const uint64_t t0 = rbx_time_us();
	SceIoStat st;
	if (sceIoGetstat(path, &st) < 0 || st.st_size <= 0)
		return -1;
	const SceUID fd = sceIoOpen(path, SCE_O_RDONLY, 0);
	if (fd < 0)
		return -1;
	const uint32_t n = (uint32_t)st.st_size;
	uint8_t *buf = (uint8_t *)malloc(n);
	int rc = -1;
	rbx_state_load_us[0] = (uint32_t)(rbx_time_us() - t0);
	const uint64_t t1 = rbx_time_us();
	if (!buf) {   // (no buffer of the state's size free: read in parts, as state_save_stream writes)
		SceUID f = fd;
		rc = rbx_state_unstream(state_get, state_skip, state_rewind, &f, n);
		RBX_LOG("state: no %u KB buffer free, read in parts: rc %d", n >> 10, rc);
		sceIoClose(fd);
		return rc;
	}
	if (sceIoRead(fd, buf, n) == (int)n) {
		rbx_state_load_us[1] = (uint32_t)(rbx_time_us() - t1);
		rc = rbx_state_read(buf, n);
	}
	sceIoClose(fd);
	free(buf);
	return rc;
}

// a file read whole (NUL after it), else NULL
static char *read_all(const char *path, uint32_t *n)
{
	SceIoStat st;
	if (sceIoGetstat(path, &st) < 0 || st.st_size <= 0 || st.st_size > 8 * 1024 * 1024)
		return NULL;
	const SceUID fd = sceIoOpen(path, SCE_O_RDONLY, 0);
	if (fd < 0)
		return NULL;
	*n = (uint32_t)st.st_size;
	char *b = (char *)malloc(*n + 1);
	if (b && sceIoRead(fd, b, *n) != (int)*n) {
		free(b);
		b = NULL;
	}
	sceIoClose(fd);
	if (b)
		b[*n] = 0;
	return b;
}

// the set's MAME cheat XML: cheats/<set>.xml, else <set>.xml in cheats/cheat.zip (MAME's cheat.7z repacked as a zip:
// one entry read, not the whole archive), else the same for the parent set
int rbx_core_cheats_load(const char *dir)
{
	const int g = rbx_running_game();
	if (g < 0)
		return 0;
	const char *sets[2] = { rbx_game_set(g), rbx_game_parent(g) };
	char path[192], *xml = NULL;
	uint32_t n = 0;
	for (int k = 0; k < 2 && !xml; k++) {
		if (!sets[k] || !sets[k][0])
			continue;
		snprintf(path, sizeof(path), "%s%s.xml", dir, sets[k]);
		xml = read_all(path, &n);
		if (xml) {
			RBX_LOG("cheats: %s", path);
			break;
		}
		snprintf(path, sizeof(path), "%scheat.zip", dir);
		rbx_zip *z = rbx_zip_open(path);
		if (!z)
			continue;
		char entry[48];
		snprintf(entry, sizeof(entry), "%s.xml", sets[k]);
		const int i = rbx_zip_find(z, entry, 0, 0);
		if (i >= 0 && rbx_zip_entry_size(z, i) > 0 && rbx_zip_entry_size(z, i) < 8 * 1024 * 1024) {
			n = rbx_zip_entry_size(z, i);
			xml = (char *)malloc(n + 1);
			if (xml && rbx_zip_read(z, i, (uint8_t *)xml)) {
				free(xml);
				xml = NULL;
			}
			if (xml) {
				xml[n] = 0;
				RBX_LOG("cheats: %s in %s", entry, path);
			}
		}
		rbx_zip_close(z);
	}
	if (!xml)
		return 0;
	const int c = rbx_cheat_load_xml(xml, n);
	free(xml);
	return c;
}
int rbx_core_cheat_count(void) { return rbx_cheat_count(); }
const char *rbx_core_cheat_name(int i) { return rbx_cheat_name(i); }
int rbx_core_cheat_options(int i) { return rbx_cheat_options(i); }
const char *rbx_core_cheat_option_name(int i, int o) { return rbx_cheat_option_name(i, o); }
int rbx_core_cheat_get(int i) { return rbx_cheat_get(i); }
void rbx_core_cheat_set(int i, int o) { rbx_cheat_set(i, o); }
