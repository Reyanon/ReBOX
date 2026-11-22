// Front touch screen: on-screen key areas and the touch points. See touch.h.
#include "frontend/touch.h"

#include <psp2/ctrl.h>
#include <psp2/touch.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "frontend/settings.h"
#include "frontend/testpad.h"
#include "platform/log.h"
#include "platform/timer.h"

#define TOUCH_PATH RBX_DATA_DIR "touch.cfg"

static ReiTouchArea *s_area;
static int s_n, s_cap, s_gen;
static char s_path[96] = TOUCH_PATH; // the layout file in use: the global one, or the game's (per-game settings)

// read a layout file over the areas now (0 if it is not there)
static int touch_read_file(const char *path)
{
	FILE *f = fopen(path, "r");
	if (!f)
		return 0;
	s_n = 0;
	s_gen++;
	int x, y, w, h, k;
	while (fscanf(f, "%d %d %d %d %d", &x, &y, &w, &h, &k) == 5)
		rbx_touch_add(x, y, w, h, k);
	fclose(f);
	return 1;
}

void rbx_touch_init(void)
{
	sceTouchSetSamplingState(SCE_TOUCH_PORT_FRONT, SCE_TOUCH_SAMPLING_STATE_START);
	if (touch_read_file(TOUCH_PATH))
		RBX_LOG("touch: %d key areas", s_n);
}

void rbx_touch_select(const char *game_path, int keep)
{
	snprintf(s_path, sizeof(s_path), "%s", game_path ? game_path : TOUCH_PATH);
	if (keep) { // the layout now goes on in that file
		rbx_touch_save();
		return;
	}
	s_n = 0;
	s_gen++;
	// a game without its own layout yet starts from the global one
	if (!touch_read_file(s_path) && game_path)
		touch_read_file(TOUCH_PATH);
	RBX_LOG("touch: %d key areas (%s)", s_n, s_path);
}

int rbx_touch_count(void) { return s_n; }
ReiTouchArea *rbx_touch_area(int i) { return i >= 0 && i < s_n ? &s_area[i] : NULL; }
int rbx_touch_gen(void) { return s_gen; }
void rbx_touch_changed(void) { s_gen++; }

int rbx_touch_add(int x, int y, int w, int h, int key)
{
	if (s_n == s_cap) {
		const int cap = s_cap ? s_cap * 2 : 16;
		ReiTouchArea *a = (ReiTouchArea *)realloc(s_area, (size_t)cap * sizeof(*a));
		if (!a)
			return -1;
		s_area = a;
		s_cap = cap;
	}
	ReiTouchArea *a = &s_area[s_n];
	a->x = (int16_t)x;
	a->y = (int16_t)y;
	a->w = (int16_t)w;
	a->h = (int16_t)h;
	a->key = (int16_t)((key >= 0 && key < RBX_TKEY_COUNT) || (key > RBX_TKEY_COMBO && key < RBX_TKEY_COMBO + (1 << RBX_KEY_COUNT))
		? key : RBX_KEY_CROSS);
	s_gen++;
	return s_n++;
}

void rbx_touch_remove(int i)
{
	if (i < 0 || i >= s_n)
		return;
	memmove(&s_area[i], &s_area[i + 1], (size_t)(s_n - i - 1) * sizeof(*s_area));
	s_n--;
	s_gen++;
}

void rbx_touch_clear(void)
{
	s_n = 0;
	s_gen++;
}

void rbx_touch_save(void)
{
	FILE *f = fopen(s_path, "w");
	if (!f) {
		RBX_LOG("touch: save failed %s", s_path);
		return;
	}
	for (int i = 0; i < s_n; i++)
		fprintf(f, "%d %d %d %d %d\n", s_area[i].x, s_area[i].y, s_area[i].w, s_area[i].h, s_area[i].key);
	fclose(f);
}

static int s_menu_hit, s_menu_held; // a menu touch button (RBX_TKEY_MENU): touched since the last take, still held
static uint64_t s_pressed;           // (the overlay shows these pressed)

uint32_t rbx_touch_key_code(int key)
{
	static const uint32_t dirs[4] = { SCE_CTRL_UP, SCE_CTRL_DOWN, SCE_CTRL_LEFT, SCE_CTRL_RIGHT };
	if (key == RBX_TKEY_MENU)
		return 0; // (rbx_touch_menu_take)
	if (key >= RBX_TKEY_UP && key < RBX_TKEY_COUNT)
		return dirs[key - RBX_TKEY_UP];
	if (RBX_TKEY_IS_COMBO(key)) {
		uint32_t b = 0;
		for (int k = 0; k < RBX_KEY_COUNT; k++)
			if ((key - RBX_TKEY_COMBO) & (1 << k))
				b |= rbx_key_mask(k);
		return b;
	}
	return rbx_key_mask(key);
}

const char *rbx_touch_key_name(int key)
{
	static const char *const dirs[4] = { "Up", "Down", "Left", "Right" };
	if (key == RBX_TKEY_MENU)
		return "Menu";
	if (key >= RBX_TKEY_UP && key < RBX_TKEY_COUNT)
		return dirs[key - RBX_TKEY_UP];
	if (RBX_TKEY_IS_COMBO(key)) {
		static char nm[32];
		nm[0] = 0;
		for (int k = 0; k < RBX_KEY_COUNT; k++)
			if ((key - RBX_TKEY_COMBO) & (1 << k))
				snprintf(nm + strlen(nm), sizeof(nm) - strlen(nm), "%s%s", nm[0] ? "+" : "", rbx_key_name(k));
		return nm;
	}
	return rbx_key_name(key);
}

int rbx_touch_read(int *x, int *y)
{
	SceTouchData t;
	int n = 0;
	if (rbx_testpad_touch(&x[0], &y[0])) // (log builds: the test input script)
		return 1;
	if (sceTouchPeek(SCE_TOUCH_PORT_FRONT, &t, 1) < 1)
		return 0;
	for (unsigned i = 0; i < t.reportNum && n < 6; i++, n++) { // panel 1920x1088 -> screen 960x544
		x[n] = t.report[i].x / 2;
		y[n] = t.report[i].y / 2;
	}
	return n;
}

int rbx_touch_on_area(int x, int y)
{
	for (int i = 0; i < s_n; i++) {
		const ReiTouchArea *a = &s_area[i];
		if (x >= a->x && x < a->x + a->w && y >= a->y && y < a->y + a->h)
			return 1;
	}
	return 0;
}

uint32_t rbx_touch_buttons(void)
{
	if (!s_n)
		return 0;
	int x[6], y[6];
	const int n = rbx_touch_read(x, y);
	// (as ReST-V: a coin went in by itself when a game started, the finger that tapped Play still down.) After a pause in
	// these calls (a game start, a closed menu, loading: over 100 ms) the touch buttons wait until no finger is down
	static uint64_t last;
	static int hold;
	const uint64_t now = rbx_time_us();
	if (now - last > 100000)
		hold = 1;
	last = now;
	if (hold) {
		if (n == 0)
			hold = 0;
		s_menu_held = n > 0; // (the finger that was down before the pause is not a new touch)
		s_pressed = 0;
		return 0;
	}
	uint32_t b = 0;
	int menu = 0;
	uint64_t pm = 0;
	for (int k = 0; k < n; k++)
		for (int i = 0; i < s_n; i++) {
			const ReiTouchArea *a = &s_area[i];
			if (x[k] >= a->x && x[k] < a->x + a->w && y[k] >= a->y && y[k] < a->y + a->h) {
				b |= rbx_touch_key_code(a->key);
				menu |= a->key == RBX_TKEY_MENU;
				if (i < 64)
					pm |= 1ull << i;
			}
		}
	s_pressed = pm;
	if (menu && !s_menu_held)
		s_menu_hit = 1; // (once a touch: the menu opens on the touch, not again while the finger stays)
	s_menu_held = menu;
	return b;
}

uint64_t rbx_touch_pressed(void) { return s_pressed; }

int rbx_touch_menu_take(void)
{
	const int h = s_menu_hit;
	s_menu_hit = 0;
	return h;
}
