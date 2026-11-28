#include "frontend/input.h"
#include "frontend/medal.h"

#include <psp2/ctrl.h>

#include "common/re_pad.h"
#include "frontend/settings.h"
#include "frontend/testpad.h"
#include "frontend/touch.h"
#include "frontend/video.h"
#include "frontend/core_api.h"

#define STICK_DEAD 48 // distance from center (128) required to count as a direction
#define ANALOG_DEAD 24 // the same for the analog controls

void rbx_input_init(void)
{
	re_pad_init();
}

// game inputs of each player, in RBX_ACT_* order
static const int s_act_input[2][RBX_ACT_COUNT] = {
	{ RBX_IN_P1_B1, RBX_IN_P1_B2, RBX_IN_P1_COIN, RBX_IN_P1_START, RBX_IN_P1_B3, RBX_IN_P1_B4, RBX_IN_P1_B5, RBX_IN_P1_B6,
	  RBX_IN_P1_RUP, RBX_IN_P1_RDOWN, RBX_IN_P1_RLEFT, RBX_IN_P1_RRIGHT },
	{ RBX_IN_P2_B1, RBX_IN_P2_B2, RBX_IN_P2_COIN, RBX_IN_P2_START, RBX_IN_P2_B3, RBX_IN_P2_B4, RBX_IN_P2_B5, RBX_IN_P2_B6,
	  RBX_IN_P2_RUP, RBX_IN_P2_RDOWN, RBX_IN_P2_RLEFT, RBX_IN_P2_RRIGHT },
};
static const int s_dir_input[2][4] = {
	{ RBX_IN_P1_UP, RBX_IN_P1_DOWN, RBX_IN_P1_LEFT, RBX_IN_P1_RIGHT },
	{ RBX_IN_P2_UP, RBX_IN_P2_DOWN, RBX_IN_P2_LEFT, RBX_IN_P2_RIGHT },
};

// one player: its controller (re_pad) with its own port settings (keys, stick, turbo, combo keys). raw = the
// controller's buttons (system combos), b = with the touch buttons (1P)
// gmask = the keys a General Key Mapping key of two keys holds now (kept from the player's own keys)
static void map_player(int pl, const SceCtrlData *pad, uint32_t raw, uint32_t b, uint32_t gmask)
{
	// the sticks' directions count as keys
	b = ((b & ~RBX_STICK_BITS) | rbx_stick_bits(pad->lx, pad->ly, pad->rx, pad->ry)) & ~gmask;
	// this player's buttons, stick, turbo and combo keys: those of the device it reads (they go with the device)
	const rbx_port *pr = &g_settings.port[re_pad_player_profile(pl)];
	int up = (b & SCE_CTRL_UP) != 0, down = (b & SCE_CTRL_DOWN) != 0;
	int left = (b & SCE_CTRL_LEFT) != 0, right = (b & SCE_CTRL_RIGHT) != 0;
	// analog sticks, each on its own: tilting past STICK_DEAD gives that direction (diagonals included) and each direction
	// presses what the port says (a direction set as a key is that key, not what the stick page says)
	static const int s_btn_act[RBX_TURBO_BUTTONS] = { RBX_ACT_ATTACK, RBX_ACT_JUMP, RBX_ACT_B3, RBX_ACT_B4, RBX_ACT_B5, RBX_ACT_B6 };
	int stick_act[RBX_ACT_COUNT] = { 0 };
	// a twin-stick game (Smash TV): its second joystick is four actions (the right stick at first, any keys)
	const int nact = rbx_core_twin_stick() ? RBX_ACT_COUNT : RBX_ACT_R_UP;
	uint32_t keyed = rbx_key_mask(pr->turbo_key);
	for (int a = 0; a < nact; a++)
		keyed |= rbx_act_bits(pr, a);
	for (int st = 0; st < 2; st++) {
		const int x = st ? pad->rx : pad->lx, y = st ? pad->ry : pad->ly;
		const int on[RBX_SDIR_COUNT] = { y < 128 - STICK_DEAD, y > 128 + STICK_DEAD, x < 128 - STICK_DEAD, x > 128 + STICK_DEAD };
		for (int d = 0; d < RBX_SDIR_COUNT; d++) {
			const int t = on[d] && !(keyed & rbx_key_mask(RBX_KEY_LUP + st * RBX_SDIR_COUNT + d)) ? pr->smap[st][d] : RBX_STGT_NONE;
			if (t == RBX_STGT_UP) up = 1;
			else if (t == RBX_STGT_DOWN) down = 1;
			else if (t == RBX_STGT_LEFT) left = 1;
			else if (t == RBX_STGT_RIGHT) right = 1;
			else if (t >= RBX_STGT_B1 && t <= RBX_STGT_B6) stick_act[s_btn_act[t - RBX_STGT_B1]] = 1;
			else if (t == RBX_STGT_COIN) stick_act[RBX_ACT_COIN] = 1;
			else if (t == RBX_STGT_START) stick_act[RBX_ACT_START] = 1;
		}
	}
	if (rbx_input_rot && re_pad_port(pl) == 0 && !re_pad_is_tv()) {
		// the Vita is held turned (Rotate Vertical L / R): screen directions back to the game's
		// (90: screen right = game up; 270: screen left = game up). Its own controls only, not a controller
		const int u = up, d = down, l = left, r = right;
		if (rbx_input_rot == 90) {
			up = r; down = l; left = u; right = d;
		} else {
			up = l; down = r; left = d; right = u;
		}
	}
	// analog controls (dial, paddle, steering, trackball, light gun, pedal): the left stick as counts per frame
	// (up to +-8; the drivers fall back to the d-pad while it is centred)
	int ax = pad->lx - 128, ay = pad->ly - 128;
	ax = ax > ANALOG_DEAD ? (ax - ANALOG_DEAD) / 12 + 1 : ax < -ANALOG_DEAD ? (ax + ANALOG_DEAD) / 12 - 1 : 0;
	ay = ay > ANALOG_DEAD ? (ay - ANALOG_DEAD) / 12 + 1 : ay < -ANALOG_DEAD ? (ay + ANALOG_DEAD) / 12 - 1 : 0;
	rbx_set_analog(pl, 0, ax);
	rbx_set_analog(pl, 1, ay);
	// the right stick as it is, for rotary joysticks (aim) and dials (turn) of the "second stick" games: while the
	// second stick's left / right are the right stick's (as they are at first), in the screen's directions
	int rx = 0, ry = 0;
	if (nact == RBX_ACT_COUNT && (rbx_act_bits(pr, RBX_ACT_R_LEFT) & rbx_key_mask(RBX_KEY_RLEFT)) &&
	    (rbx_act_bits(pr, RBX_ACT_R_RIGHT) & rbx_key_mask(RBX_KEY_RRIGHT))) {
		rx = pad->rx - 128;
		ry = pad->ry - 128;
		if (rbx_input_rot && re_pad_port(pl) == 0 && !re_pad_is_tv()) {   // (as the d-pad above)
			const int x = rx, y = ry;
			if (rbx_input_rot == 90) {
				rx = y;
				ry = -x;
			} else {
				rx = -y;
				ry = x;
			}
		}
	}
	rbx_set_rstick(pl, rx, ry);
	rbx_core_set_input(s_dir_input[pl][0], up);
	rbx_core_set_input(s_dir_input[pl][1], down);
	rbx_core_set_input(s_dir_input[pl][2], left);
	rbx_core_set_input(s_dir_input[pl][3], right);

	// system combos are not passed to the game: SELECT+START options, SELECT+L quick save, SELECT+R quick load
	uint32_t blocked = 0;
	if ((raw & SCE_CTRL_SELECT) && (raw & SCE_CTRL_START))
		blocked |= SCE_CTRL_SELECT | SCE_CTRL_START;
	if ((raw & SCE_CTRL_SELECT) && (raw & SCE_CTRL_LTRIGGER))
		blocked |= SCE_CTRL_SELECT | SCE_CTRL_LTRIGGER;
	if ((raw & SCE_CTRL_SELECT) && (raw & SCE_CTRL_RTRIGGER))
		blocked |= SCE_CTRL_SELECT | SCE_CTRL_RTRIGGER;
	uint32_t game = b & ~blocked;

	// combo keys: a key with a combo acts as its two keys pressed together (instead of its own mapping)
	uint32_t add = 0;
	for (int k = 0; k < RBX_KEY_COUNT; k++)
		if (pr->combo[k] && (game & rbx_key_mask(k))) {
			game &= ~rbx_key_mask(k);
			for (int j = 0; j < RBX_KEY_COUNT; j++)
				if (pr->combo[k] & (1 << j))
					add |= rbx_key_mask(j);
		}
	game |= add;

	// turbo: while the turbo key is held, the chosen game button is pressed only during the first half of each period.
	// counting starts at the press frame, so the first press registers immediately
	static int turbo_t[2];
	int turbo_on = 0;
	const uint32_t tm = rbx_key_mask(pr->turbo_key);
	if (tm && (game & tm)) {
		const int period = rbx_turbo_period(pl);
		turbo_on = turbo_t[pl] < period / 2;
		turbo_t[pl] = (turbo_t[pl] + 1) % period;
	} else
		turbo_t[pl] = 0;
	const int turbo_act = rbx_turbo_act(pl);

	const int medal = rbx_medal_game();
	int on_act[RBX_ACT_COUNT] = { 0 };
	for (int a = 0; a < nact; a++) {
		// if the turbo key is also mapped as a normal key, turbo wins (its normal mapping is ignored)
		int on = ((game & ~tm) & rbx_act_bits(pr, a)) != 0 || stick_act[a];
		if (a == turbo_act)
			on |= turbo_on;
		if (a == RBX_ACT_COIN && medal) {   // (medal cabinets: one medal a press; held, the opto sensor saw a jam
			static int coin_t[2];            // and the game showed COIN ERROR)
			coin_t[pl] = on ? coin_t[pl] + 1 : 0;
			on = on && coin_t[pl] <= 6;
		}
		on_act[a] = on;
	}
	if (rbx_input_rot && re_pad_port(pl) == 0 && !re_pad_is_tv()) {
		// the second joystick turned like the first (Rotate Vertical L / R)
		int *r = on_act + RBX_ACT_R_UP;
		const int ru = r[0], rd = r[1], rl = r[2], rr = r[3];
		if (rbx_input_rot == 90) {
			r[0] = rr; r[1] = rl; r[2] = ru; r[3] = rd;
		} else {
			r[0] = rl; r[1] = rr; r[2] = rd; r[3] = ru;
		}
	}
	for (int a = 0; a < RBX_ACT_COUNT; a++)
		rbx_core_set_input(s_act_input[pl][a], on_act[a]);
}

// General Key Mapping (MAME's general inputs): the general inputs held on the players' devices, 1 << RBX_GEN_*. A key of
// two keys holds while both are held; then a key of one of those two keys alone (on that device) does not, and the two
// are kept from the player's own keys (gmask, SCE bits: SELECT+Triangle as the test switch inserts no coin while held).
// pd / bb: players 0..3's pads and buttons (with the touch buttons)
static uint32_t general_poll(const SceCtrlData *const *pd, const uint32_t *bb, uint32_t *gmask)
{
	uint32_t held = 0;
	for (int pl = 0; pl < 4; pl++) {
		gmask[pl] = 0;
		if (re_pad_port(pl) < 0)
			continue;
		const int prof = re_pad_player_profile(pl);
		const uint32_t keys = rbx_keys_of((bb[pl] & ~RBX_STICK_BITS) |
			rbx_stick_bits(pd[pl]->lx, pd[pl]->ly, pd[pl]->rx, pd[pl]->ry));
		uint32_t pair = 0;   // (the keys of the two-key ones held)
		for (int g = 0; g < RBX_GEN_COUNT * RBX_GEN_KEYS; g++) {   // (every key of every general input)
			const rbx_gen_key *k = &g_settings.gen[g / RBX_GEN_KEYS][g % RBX_GEN_KEYS];
			if (k->keys && k->prof == prof && (k->keys & (k->keys - 1)) && (keys & k->keys) == k->keys) {
				held |= 1u << (g / RBX_GEN_KEYS);
				pair |= k->keys;
			}
		}
		for (int g = 0; g < RBX_GEN_COUNT * RBX_GEN_KEYS; g++) {
			const rbx_gen_key *k = &g_settings.gen[g / RBX_GEN_KEYS][g % RBX_GEN_KEYS];
			if (k->keys && k->prof == prof && !(k->keys & (k->keys - 1)) && (keys & k->keys) && !(pair & k->keys))
				held |= 1u << (g / RBX_GEN_KEYS);
		}
		for (int i = 0; i < RBX_KEY_LANY; i++)
			if (pair & (1u << i))
				gmask[pl] |= rbx_key_mask(i);
	}
	return held;
}

uint32_t rbx_input_poll(void)
{
	// the players' controllers from the settings (loaded, or changed in the menu)
	static int src[2] = { -1, -1 };
	re_pad_set_added_players(g_settings.p2_added ? 2 : 0);   // (a 2P row added by hand takes its device)
	for (int p = 0; p < 2; p++)
		if (src[p] != g_settings.pad_src[p]) {
			src[p] = g_settings.pad_src[p];
			re_pad_set_source(p, src[p]);
		}
	re_pad_poll();
	SceCtrlData p1, p2, all;
	re_pad_read(0, &p1);
	re_pad_read(1, &p2);
	re_pad_read_all(&all);
	const uint32_t test = rbx_testpad_step(); // (log builds: the test input script)
	p1.buttons |= test;
	all.buttons |= test;
	uint32_t b = p1.buttons, b2 = p2.buttons;
	// touch buttons press their Vita keys (in the game only: the value returned is the pads alone), for the player on the
	// Vita's own controls (as ReST-V: 1P on a controller, 2P on the Vita: the touch is 2P's); 1P's when nobody reads it
	if (g_settings.touch_keys) {
		const uint32_t tb = rbx_touch_buttons();
		if (re_pad_port(0) != 0 && re_pad_port(1) == 0)
			b2 |= tb;
		else
			b |= tb;
	}
	// General Key Mapping: every player's device (3P / 4P read for these alone)
	SceCtrlData p3, p4;
	re_pad_read(2, &p3);
	re_pad_read(3, &p4);
	const SceCtrlData *pd[4] = { &p1, &p2, &p3, &p4 };
	const uint32_t bb[4] = { b, b2, p3.buttons, p4.buttons };
	uint32_t gmask[4];
	rbx_core_set_general(general_poll(pd, bb, gmask));
	map_player(0, &p1, p1.buttons, b, gmask[0]);
	map_player(1, &p2, p2.buttons, b2, gmask[1]);
	return all.buttons;
}

void rbx_input_release(void)
{
	rbx_core_set_general(0);
	for (int i = 0; i < RBX_IN_DIGITAL_COUNT; i++)
		rbx_core_set_input(i, 0);
	for (int p = 0; p < 2; p++) {
		rbx_set_analog(p, 0, 0);
		rbx_set_analog(p, 1, 0);
		rbx_set_rstick(p, 0, 0);
	}
}
