// Frontend view of the ReBOX core (core_api.h)
#include "frontend/core_api.h"

#include "platform/timer.h"
#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>
#include <stdlib.h>
#include <stddef.h>

static uint32_t s_in[4];
static uint32_t s_sys;

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

const char *rbx_core_game_set(int i) { return rbx_game_set(i); }
const char *rbx_core_game_parent(int i) { return rbx_game_parent(i); }
const char *rbx_core_game_title(int i) { return rbx_game_title(i); }
const char *rbx_core_game_board(int i) { return rbx_game_board(i); }
int rbx_core_game_count(void) { return rbx_game_count(); }
const char *rbx_core_game_name(void) { int g = rbx_running_game(); return g >= 0 ? rbx_game_title(g) : ""; }
const char *rbx_core_set_name(void) { int g = rbx_running_game(); return g >= 0 ? rbx_game_set(g) : ""; }
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
	rbx_set_system(s_sys);
	return rbx_run_frame(fb, pitch, audio, max);
}

int rbx_core_state_exists(const char *path)
{
	SceIoStat st;
	return sceIoGetstat(path, &st) >= 0;
}

int rbx_core_state_save(const char *path)
{
	const uint32_t n = rbx_state_size();
	if (!n)
		return -1;
	uint8_t *buf = (uint8_t *)malloc(n);
	int rc = -1;
	if (buf && !rbx_state_write(buf, n)) {
		const SceUID fd = sceIoOpen(path, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
		if (fd >= 0) {
			rc = sceIoWrite(fd, buf, n) == (int)n ? 0 : -1;
			sceIoClose(fd);
		}
	}
	free(buf);
	return rc;
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
	if (buf && sceIoRead(fd, buf, n) == (int)n) {
		rbx_state_load_us[1] = (uint32_t)(rbx_time_us() - t1);
		rc = rbx_state_read(buf, n);
	}
	sceIoClose(fd);
	free(buf);
	return rc;
}

int rbx_core_cheats_load(const char *dir) { (void)dir; return 0; }
int rbx_core_cheat_count(void) { return 0; }
const char *rbx_core_cheat_name(int i) { (void)i; return NULL; }
int rbx_core_cheat_options(int i) { (void)i; return 0; }
const char *rbx_core_cheat_option_name(int i, int o) { (void)i; (void)o; return NULL; }
int rbx_core_cheat_get(int i) { (void)i; return 0; }
void rbx_core_cheat_set(int i, int o) { (void)i; (void)o; }
