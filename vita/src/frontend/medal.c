// Medal cabinet panels (Taito O System medal games): drawn in the margins beside the game picture. Left: medal in / paid
// counters and the hopper, which spills medals while the game pays out. Right: the cabinet buttons with their lamps as the
// game lights them (MAME taito_outb_w lamp bits) and the Vita keys that press them.
#include "frontend/medal.h"

#include <stdio.h>
#include <string.h>

#include "common/re_pad.h"
#include "frontend/core_api.h"
#include "frontend/settings.h"
#include "frontend/touch.h"
#include "frontend/ui.h"
#include "frontend/video.h"

#define SCREEN_W 960
#define SCREEN_H 544
#define MIN_MARGIN 96   // narrower margins: no panels
// each margin a canvas of its own, as wide as the widest margin (a vertical game's): a change of one side uploads
// that side only (one screen-wide canvas uploaded the whole screen on each touch, and the game lost frames)
#define PANEL_CW 288

typedef struct { const char *label; int act, lamp; } button_t;   // act: RBX_ACT_* (the Vita key shown)
// the attendant's key (MAME "Reset Key", SERVICE1: clears the errors and a jackpot's hand pay); touch only, no lamp
#define ACT_KEY RBX_ACT_COUNT
#define NO_LAMP 31

// lamp bits per game (MAME taito_o.cpp taito_outb_w notes) and the core's mapping of the cabinet buttons
static const button_t s_parentj[] = {
	{ "BET 1", RBX_ACT_ATTACK, 0 }, { "BET 2", RBX_ACT_JUMP, 1 }, { "BET 3", RBX_ACT_B3, 2 }, { "BET 4", RBX_ACT_B4, 3 },
	{ "START", RBX_ACT_START, 8 }, { "CHECK", RBX_ACT_B5, 9 }, { "DEAL/HIT", RBX_ACT_START, 12 }, { "PAYOUT", RBX_ACT_B6, 13 },
	{ "RESET KEY", ACT_KEY, NO_LAMP },
};
static const button_t s_eibise[] = {
	{ "MAX BET", RBX_ACT_B5, 8 }, { "BET", RBX_ACT_ATTACK, 9 }, { "DOUBLE UP", RBX_ACT_B3, 10 }, { "START", RBX_ACT_START, 12 },
	{ "PAYOUT", RBX_ACT_B6, 13 }, { "RESET KEY", ACT_KEY, NO_LAMP },
};

static UiCanvas *s_cv, *s_cvr;   // left (slot 5), right (slot 6, at SCREEN_W - PANEL_CW)
static const button_t *s_btn;
static int s_nbtn, s_on;
static uint32_t s_last_lamps, s_last_in, s_last_paid;
static int s_last_hop, s_tick, s_drawn_x;
// touch: the button rows and the medal slot as drawn (screen pixels), the buttons touched now
static struct { int x, y, w, h; } s_hit[16], s_slot, s_reset;
static uint32_t s_touched, s_last_touched;   // bit i = button i, bit 31 = medal slot, bit 30 = reset
#define RESET_HOLD 60                        // frames the reset button is held before it clears
static int s_reset_hold, s_reset_done;

int rbx_medal_game(void)
{
	int g = rbx_running_game();
	return g >= 0 && !strcmp(rbx_game_board(g), "Taito O System");
}

void rbx_medal_start(void)
{
	s_on = rbx_medal_game();
	if (!s_on)
		return;
	const char *set = rbx_core_set_name();
	if (!strcmp(set, "eibise")) {
		s_btn = s_eibise;
		s_nbtn = (int)(sizeof(s_eibise) / sizeof(s_eibise[0]));
	} else {
		s_btn = s_parentj;
		s_nbtn = (int)(sizeof(s_parentj) / sizeof(s_parentj[0]));
	}
	if (!s_cv)
		s_cv = ui_canvas(5, 0, 0, PANEL_CW, SCREEN_H);
	if (!s_cvr)
		s_cvr = ui_canvas(6, SCREEN_W - PANEL_CW, 0, PANEL_CW, SCREEN_H);
	s_last_lamps = 0xffffffffu;
	s_tick = 0;
	s_drawn_x = -1;
}

void rbx_medal_stop(void)
{
	if (s_cv)
		ui_hide(s_cv);
	if (s_cvr)
		ui_hide(s_cvr);
	s_on = 0;
	rbx_core_set_service(0, 0);
}

// text that fits: the scale shrinks until it fits maxw
static void text_fit(UiCanvas *c, int x, int y, uint32_t col, float scale, const char *s, int maxw)
{
	while (scale > 0.4f && ui_text_width(scale, s) > maxw)
		scale -= 0.05f;
	ui_text(c, x, y, col, scale, s);
}

static void counter(UiCanvas *c, int x, int y, int w, const char *title, unsigned v, uint32_t col)
{
	char buf[16];
	ui_rect(c, x, y, w, 40, UI_RGBA(0x20, 0x08, 0x18, 0xE0));
	ui_rect(c, x, y, w, 2, col);
	ui_text(c, x + 5, y + 15, UI_RGBA(0xF0, 0xC8, 0xE0, 0xFF), 0.5f, title);
	snprintf(buf, sizeof(buf), "%u", v);
	ui_text(c, x + w - 5 - ui_text_width(0.8f, buf), y + 35, UI_RGBA(0xFF, 0xFF, 0xFF, 0xFF), 0.8f, buf);
}

static void medal(UiCanvas *c, float x, float y, float r)
{
	ui_circle(c, x, y, r, UI_RGBA(0xB0, 0x88, 0x20, 0xFF));
	ui_circle(c, x, y, r * 0.78f, UI_RGBA(0xF0, 0xC8, 0x40, 0xFF));
	ui_circle(c, x - r * 0.25f, y - r * 0.25f, r * 0.25f, UI_RGBA(0xFF, 0xF0, 0xB0, 0xFF));
}

// layout (margins of about 110 px): left column top to bottom = title, IN, PAID, hopper with its chute and the medals
// falling out, the medal slot, the coin key; right column = title and the buttons, 44 px each
static void draw(int px, int pw, int full, int left, int right)
{
	UiCanvas *c = s_cv;
	const int lw = px, rx = px + pw, rw = SCREEN_W - rx;
	if (full) {
		ui_clear(s_cv);
		ui_clear(s_cvr);
	} else {   // (only the margin itself: the rest of the canvas stays clear)
		if (left)
			ui_clear_rect(s_cv, 0, 0, lw < PANEL_CW ? lw : PANEL_CW, SCREEN_H);
		if (right)
			ui_clear_rect(s_cvr, rx - (SCREEN_W - PANEL_CW), 0, rw, SCREEN_H);
	}
	const uint32_t lamps = rbx_medal_lamps();
	const int hop = rbx_medal_hopper_running();
	const rbx_port *pr = &g_settings.port[re_pad_player_profile(0)];
	const uint32_t title = UI_RGBA(0xD8, 0x3C, 0xB0, 0xFF), dim = UI_RGBA(0xC0, 0xA0, 0xB8, 0xFF);
	if (lw >= MIN_MARGIN && lw <= PANEL_CW && (left || full)) {
		int x = 6, w = lw - 12;
		ui_text(c, x, 20, title, 0.65f, "MEDAL");
		counter(c, x, 28, w, "IN", rbx_medal_inserted(), UI_RGBA(0x60, 0xC0, 0xFF, 0xFF));
		counter(c, x, 74, w, "PAID", rbx_medal_paid(), UI_RGBA(0xF0, 0xC8, 0x40, 0xFF));
		// hopper bin
		int hy = 124, hh = 110;
		ui_rect(c, x, hy, w, hh, UI_RGBA(0x30, 0x30, 0x38, 0xF0));
		ui_rect(c, x + 2, hy + 2, w - 4, hh - 4, UI_RGBA(0x18, 0x18, 0x20, 0xF0));
		ui_text(c, x + 5, hy + 16, hop ? UI_RGBA(0xFF, 0xE0, 0x60, 0xFF) : UI_RGBA(0x90, 0x90, 0xA0, 0xFF), 0.5f,
			hop ? "HOPPER ON" : "HOPPER");
		for (int i = 0; i < 12; i++)
			medal(c, (float)(x + 10 + (i * 15) % (w - 20)), (float)(hy + hh - 12 - (i / 4) * 9), 7.0f);
		// chute and the medals it pays out
		int cy = hy + hh;
		ui_rect(c, x + w / 2 - 10, cy, 20, 18, UI_RGBA(0x40, 0x40, 0x48, 0xF0));
		if (hop)
			for (int k = 0; k < 3; k++) {
				int t = (s_tick * 3 + k * 17) % 50;
				medal(c, (float)(x + w / 2), (float)(cy + 26 + t), 8.0f);
			}
		// medal slot (touch: drop a medal)
		int sy = 320;
		const int in_now = (s_touched >> 31) & 1;
		const int locked = !(rbx_medal_outputs() & 0x0800);   // coin lock: the slot returns medals
		ui_rect(c, x, sy, w, 64, in_now ? UI_RGBA(0x50, 0x40, 0x10, 0xF0) : UI_RGBA(0x28, 0x20, 0x10, 0xE0));
		ui_rect(c, x + w / 2 - 4, sy + 8, 8, 26, UI_RGBA(0x08, 0x08, 0x08, 0xFF));
		if (in_now)
			medal(c, (float)(x + w / 2), (float)(sy + 21), 9.0f);
		text_fit(c, x + 5, sy + 56, locked ? UI_RGBA(0xFF, 0x60, 0x60, 0xFF) : UI_RGBA(0xF0, 0xD0, 0x80, 0xFF), 0.5f,
			locked ? "LOCKED" : "TOUCH: MEDAL", w - 10);
		s_slot.x = x; s_slot.y = sy; s_slot.w = w; s_slot.h = 64;
		char buf[48];
		snprintf(buf, sizeof(buf), "%s: medal", rbx_key_name(pr->keymap[RBX_ACT_COIN]));
		text_fit(c, x, sy + 84, dim, 0.5f, buf, w);
		// reset (touch and hold): medal counters and the game's NVRAM (credits)
		int ry = 420;
		ui_rect(c, x, ry, w, 40, UI_RGBA(0x30, 0x10, 0x10, 0xE0));
		if (s_reset_hold)
			ui_rect(c, x, ry, w * s_reset_hold / RESET_HOLD, 40, UI_RGBA(0xC0, 0x30, 0x30, 0xF0));
		text_fit(c, x + 5, ry + 17, UI_RGBA(0xFF, 0xFF, 0xFF, 0xFF), 0.5f, "RESET", w - 10);
		text_fit(c, x + 5, ry + 34, dim, 0.45f, s_reset_done ? "CLEARED" : "HOLD TO CLEAR", w - 10);
		s_reset.x = x; s_reset.y = ry; s_reset.w = w; s_reset.h = 40;
	} else if (lw < MIN_MARGIN || lw > PANEL_CW) {
		s_slot.w = 0;
		s_reset.w = 0;
	}
	if (left || full)
		ui_commit(s_cv);
	c = s_cvr;
	const int ox = SCREEN_W - PANEL_CW;   // (the right canvas' screen x: drawn at x - ox, touched at x)
	if (rw >= MIN_MARGIN && rw <= PANEL_CW && (right || full)) {
		int x = rx + 6 - ox, w = rw - 12;
		ui_text(c, x, 20, title, 0.65f, "PANEL");
		memset(s_hit, 0, sizeof(s_hit));
		for (int i = 0; i < s_nbtn && i < 16; i++) {
			int y = 28 + i * 44;
			int lit = (lamps >> s_btn[i].lamp) & 1, down = (s_touched >> i) & 1;
			ui_rect(c, x, y, w, 40, down ? UI_RGBA(0xA0, 0x30, 0x80, 0xF8) : lit ? UI_RGBA(0x60, 0x18, 0x48, 0xF0)
			        : UI_RGBA(0x20, 0x10, 0x1C, 0xE0));
			s_hit[i].x = x + ox; s_hit[i].y = y; s_hit[i].w = w; s_hit[i].h = 40;
			ui_circle(c, (float)(x + 13), (float)(y + 20), 9.0f, lit ? UI_RGBA(0xFF, 0x70, 0xC8, 0xFF) : UI_RGBA(0x50, 0x30, 0x48, 0xFF));
			if (lit)
				ui_circle(c, (float)(x + 10), (float)(y + 17), 3.0f, UI_RGBA(0xFF, 0xE0, 0xF4, 0xFF));
			text_fit(c, x + 28, y + 17, UI_RGBA(0xFF, 0xFF, 0xFF, 0xFF), 0.5f, s_btn[i].label, w - 32);
			text_fit(c, x + 28, y + 34, dim, 0.45f,
			         s_btn[i].act == ACT_KEY ? "Touch" : rbx_key_name(pr->keymap[s_btn[i].act]), w - 32);
		}
	}
	if (right || full)
		ui_commit(s_cvr);
}

// core inputs of the frontend actions (input.c s_act_input, player 1)
static int act_input(int act)
{
	switch (act) {
	case RBX_ACT_ATTACK: return RBX_IN_P1_B1;
	case RBX_ACT_JUMP: return RBX_IN_P1_B2;
	case RBX_ACT_COIN: return RBX_IN_P1_COIN;
	case RBX_ACT_START: return RBX_IN_P1_START;
	case RBX_ACT_B3: return RBX_IN_P1_B3;
	case RBX_ACT_B4: return RBX_IN_P1_B4;
	case RBX_ACT_B5: return RBX_IN_P1_B5;
	default: return RBX_IN_P1_B6;
	}
}

void rbx_medal_touch(void)
{
	if (!s_on)
		return;
	int tx[6], ty[6];
	int n = rbx_touch_read(tx, ty);
	uint32_t t = 0;
	for (int k = 0; k < n; k++) {
		for (int i = 0; i < s_nbtn && i < 16; i++)
			if (s_hit[i].w && tx[k] >= s_hit[i].x && tx[k] < s_hit[i].x + s_hit[i].w && ty[k] >= s_hit[i].y &&
			    ty[k] < s_hit[i].y + s_hit[i].h)
				t |= 1u << i;
		if (s_slot.w && tx[k] >= s_slot.x && tx[k] < s_slot.x + s_slot.w && ty[k] >= s_slot.y && ty[k] < s_slot.y + s_slot.h)
			t |= 1u << 31;
		if (s_reset.w && tx[k] >= s_reset.x && tx[k] < s_reset.x + s_reset.w && ty[k] >= s_reset.y && ty[k] < s_reset.y + s_reset.h)
			t |= 1u << 30;
	}
	s_touched = t;
	if (t & (1u << 30)) {
		if (!s_reset_done && ++s_reset_hold >= RESET_HOLD) {
			rbx_medal_reset();
			s_reset_done = 1;
			s_reset_hold = 0;
		}
	} else {
		s_reset_hold = 0;
		s_reset_done = 0;
	}
	// pressed on top of the pads (rbx_input_poll set the frame's inputs just before). A tap is held MIN_PRESS frames at
	// least, as a cabinet button's press: a tap of a frame or two came and went between the game's reads of its inputs
	enum { MIN_PRESS = 6 };
	static uint8_t hold[32];
	uint32_t on = 0;
	for (int i = 0; i < 32; i++) {
		if (t & (1u << i))
			hold[i] = MIN_PRESS;
		else if (hold[i])
			hold[i]--;
		if (hold[i])
			on |= 1u << i;
	}
	int key = 0;
	for (int i = 0; i < s_nbtn && i < 16; i++)
		if (on & (1u << i)) {
			if (s_btn[i].act == ACT_KEY)
				key = 1;
			else
				rbx_core_set_input(act_input(s_btn[i].act), 1);
		}
	rbx_core_set_service(0, key);
	if (on & (1u << 31))
		rbx_core_set_input(RBX_IN_P1_COIN, 1);
}

void rbx_medal_tick(void)
{
	if (!s_on || !s_cv || !s_cvr)
		return;
	s_tick++;
	int px, py, pw, ph;
	if (!rbx_video_picture_rect(&px, &py, &pw, &ph))
		return;
	(void)py;
	(void)ph;
	const uint32_t lamps = rbx_medal_lamps(), in = rbx_medal_inserted(), paid = rbx_medal_paid();
	const int running = rbx_medal_hopper_running();
	const int hop = running | (int)(rbx_medal_outputs() & 0x0800); // (lock changes redraw too)
	// redraw on a change (and every 4th frame while the hopper animates or the reset button is held)
	const int anim = (running || (s_touched & (1u << 30))) && (s_tick & 3) == 0;
	// the left panel: counters, hopper, medal slot, reset (touch bits 30, 31); the right: lamps and buttons (bits 0-15)
	const uint32_t tl = s_touched & 0xc0000000u, tr = s_touched & 0xffffu;
	const int left = in != s_last_in || paid != s_last_paid || hop != s_last_hop || tl != (s_last_touched & 0xc0000000u) || anim;
	const int right = lamps != s_last_lamps || tr != (s_last_touched & 0xffffu);
	if (!left && !right && px == s_drawn_x)
		return;
	s_last_touched = s_touched;
	s_last_lamps = lamps;
	s_last_in = in;
	s_last_paid = paid;
	s_last_hop = hop;
	const int full = px != s_drawn_x;
	s_drawn_x = px;
	draw(px, pw, full, left, right);
}
