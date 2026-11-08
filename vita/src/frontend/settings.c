#include "frontend/settings.h"
#include "frontend/neon.h"
#include "common/re_pad.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <psp2/ctrl.h>
#include <psp2/io/dirent.h>
#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>

#include "frontend/touch.h"
#include "platform/log.h"

#define SETTINGS_PATH RBX_DATA_DIR "settings.cfg"
#define FAV_PATH RBX_DATA_DIR "favorites.cfg"
#define GAME_CFG_DIR RBX_DATA_DIR "cfg/"   // (the games' files: <set>.cfg, <set>_game.cfg, <set>_touch.cfg)

// Defaults (set 2026-10-28): FPS off, 4:3, Smooth, scanlines off (Medium when on), confirm X, art border, Graphite,
// attack □ jump △ coin SELECT start START, turbo off (button 1 when set), DIP 2 bytes (157, 247)
rbx_settings g_settings = {
	.show_fps = 0,
	.aspect = RBX_ASPECT_4_3,
	.filter = RBX_FILTER_LINEAR,
	.scan_type = RBX_SCAN_PX2,
	.scan_strength = RBX_SCAN_MEDIUM,
	.border = RBX_BORDER_ARTWORK,
	.border_color = RBX_BCOL_GRAY,
	.menu_confirm = RBX_CONFIRM_CROSS,
	.start_stage = 0,
	.port = {
		{ .keymap = { RBX_KEY_SQUARE, RBX_KEY_CROSS, RBX_KEY_SELECT, RBX_KEY_START, RBX_KEY_CIRCLE, RBX_KEY_TRIANGLE, RBX_KEY_L,
			RBX_KEY_R }, .turbo_key = RBX_KEY_NONE, .turbo_btn = 1, .turbo_speed = RBX_TURBO_FAST },
		{ .keymap = { RBX_KEY_SQUARE, RBX_KEY_CROSS, RBX_KEY_SELECT, RBX_KEY_START, RBX_KEY_CIRCLE, RBX_KEY_TRIANGLE, RBX_KEY_L,
			RBX_KEY_R }, .turbo_key = RBX_KEY_NONE, .turbo_btn = 1, .turbo_speed = RBX_TURBO_FAST },
	},
	.cheat_health = 0,
	.dip_count = 2,
	.dip = { 157, 247 },
	.rotate = RBX_ROT_AUTO,
	.touch_keys = 1,
	.touch_show = 1,
	.theme = RBX_THEME_PINK,
	.list_snap = 1,
	.neon = { 10, 2, -10, 10, 5, 60, 200 },   // (fixed: the owner's choice, 2026-11-07; not in the settings file nor the menu)
	.input_engine = RE_PAD_ENGINE_NATIVE,
};

static const char *const s_keymap_keys[RBX_ACT_COUNT] = { "key_attack", "key_jump", "key_coin", "key_start",
	"key_b3", "key_b4", "key_b5", "key_b6" };
static const char *const s_keymap2_keys[RBX_ACT_COUNT] = { "key2_attack", "key2_jump", "key2_coin", "key2_start",
	"key2_b3", "key2_b4", "key2_b5", "key2_b6" };

// a stick as the d-pad, or pressing nothing
void rbx_stick_preset(rbx_port *p, int st, int dpad)
{
	for (int d = 0; d < RBX_SDIR_COUNT; d++)
		p->smap[st][d] = dpad ? RBX_STGT_UP + d : RBX_STGT_NONE;
}

void rbx_settings_default_port(int port)
{
	if (port < 0 || port >= RBX_PROFILES) // (a profile: re_pad_profile)
		return;
	rbx_port *p = &g_settings.port[port];
	p->keymap[RBX_ACT_ATTACK] = RBX_KEY_SQUARE;
	p->keymap[RBX_ACT_JUMP] = RBX_KEY_CROSS;
	p->keymap[RBX_ACT_COIN] = RBX_KEY_SELECT;
	p->keymap[RBX_ACT_START] = RBX_KEY_START;
	p->keymap[RBX_ACT_B3] = RBX_KEY_CIRCLE;
	p->keymap[RBX_ACT_B4] = RBX_KEY_TRIANGLE;
	p->keymap[RBX_ACT_B5] = RBX_KEY_L;
	p->keymap[RBX_ACT_B6] = RBX_KEY_R;
	p->turbo_key = RBX_KEY_NONE;
	p->turbo_btn = 1;
	p->turbo_speed = RBX_TURBO_FAST;
	rbx_stick_preset(p, 0, 1);
	rbx_stick_preset(p, 1, 0);
	for (int a = 0; a < RBX_ACT_COUNT; a++)
		for (int x = 0; x < RBX_KEY_EXTRA; x++)
			p->keyx[a][x] = RBX_KEY_NONE;
	memset(p->combo, 0, sizeof(p->combo));
}

void rbx_settings_default_keys(void)
{
	for (int i = 0; i < RBX_PROFILES; i++)
		rbx_settings_default_port(i);
	g_settings.touch_keys = 1;
	g_settings.touch_show = 1;
}

// a player's controls: the profile of the device it reads now
static const rbx_port *port_of(int port) { return &g_settings.port[re_pad_player_profile(port)]; }

int rbx_turbo_period(int port)
{
	static const int p[RBX_TURBO_SPEED_COUNT] = { 4, 6, 8 }; // 15/10/7.5 per second at 60fps
	return p[port_of(port)->turbo_speed];
}

int rbx_turbo_act(int port)
{
	static const int a[RBX_TURBO_BUTTONS] = { RBX_ACT_ATTACK, RBX_ACT_JUMP, RBX_ACT_B3, RBX_ACT_B4, RBX_ACT_B5, RBX_ACT_B6 };
	const int b = port_of(port)->turbo_btn;
	return a[(b >= 1 && b <= RBX_TURBO_BUTTONS ? b : 1) - 1];
}

uint32_t rbx_key_mask(int key)
{
	static const uint32_t m[RBX_KEY_COUNT] = {
		SCE_CTRL_CROSS, SCE_CTRL_CIRCLE, SCE_CTRL_SQUARE, SCE_CTRL_TRIANGLE,
		SCE_CTRL_LTRIGGER, SCE_CTRL_RTRIGGER, SCE_CTRL_SELECT, SCE_CTRL_START,
		RE_PAD_L2, RE_PAD_R2, RE_PAD_L3, RE_PAD_R3,
		0x00020000u, 0x00040000u, 0x00400000u, 0x00800000u, 0x10000000u, 0x20000000u, 0x40000000u, 0x80000000u,
		RBX_STICK_BITS_L, RBX_STICK_BITS_R,
	};
	return (key >= 0 && key < RBX_KEY_COUNT) ? m[key] : 0;
}

uint32_t rbx_act_bits(const rbx_port *p, int act)
{
	uint32_t b = rbx_key_mask(p->keymap[act]);
	for (int x = 0; x < RBX_KEY_EXTRA; x++)
		b |= rbx_key_mask(p->keyx[act][x]);
	return b;
}

uint32_t rbx_stick_bits(int lx, int ly, int rx, int ry)
{
	uint32_t b = 0;
	if (ly < 80) b |= rbx_key_mask(RBX_KEY_LUP);
	if (ly > 176) b |= rbx_key_mask(RBX_KEY_LDOWN);
	if (lx < 80) b |= rbx_key_mask(RBX_KEY_LLEFT);
	if (lx > 176) b |= rbx_key_mask(RBX_KEY_LRIGHT);
	if (ry < 80) b |= rbx_key_mask(RBX_KEY_RUP);
	if (ry > 176) b |= rbx_key_mask(RBX_KEY_RDOWN);
	if (rx < 80) b |= rbx_key_mask(RBX_KEY_RLEFT);
	if (rx > 176) b |= rbx_key_mask(RBX_KEY_RRIGHT);
	return b;
}

const char *rbx_key_name(int key)
{
	static const char *const n[RBX_KEY_COUNT] = {
		"Cross", "Circle", "Square", "Triangle", "L", "R", "Select", "Start", "L2", "R2", "L3", "R3",
		"L-Stick Up", "L-Stick Down", "L-Stick Left", "L-Stick Right", "R-Stick Up", "R-Stick Down", "R-Stick Left",
		"R-Stick Right", "L-Stick", "R-Stick",
	};
	return (key >= 0 && key < RBX_KEY_COUNT) ? n[key] : "None";
}

static int clampi(int v, int lo, int hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}
// a key from a settings file: files before L2 / R2 / L3 / R3 (no keyver=2 line first) had None as 8
static int s_keyver;
static int key_val(int v, int hi) { return clampi((s_keyver < 2 && v == 8) || (s_keyver < 4 && v == 12) || (s_keyver < 5 && v == 20) ? RBX_KEY_NONE : v, 0, hi); }

static const char *const s_combo_keys[RBX_KEY_COUNT] = { "cross", "circle", "square", "triangle", "l", "r", NULL, NULL,
	"l2", "r2", "l3", "r3" };
// a combo value is two different keys of Cross..R
static int combo_ok(int v)
{
	int n = 0;
	for (int k = 0; k < RBX_KEY_COUNT; k++)
		if (v & (1 << k)) {
			if (k == RBX_KEY_SELECT || k == RBX_KEY_START) // (pairs of Cross..R, L2 / R2 / L3 / R3)
				return 0;
			n++;
		}
	return n == 2;
}

// Parse one "key=value" line (integer values, only last_game is text). Returns 1 on success
static int read_kv(FILE *f, char *key, int klen, char *val, int vlen)
{
	char line[96];
	if (!fgets(line, sizeof(line), f))
		return 0;
	char *eq = strchr(line, '=');
	if (!eq) {
		key[0] = 0;
		val[0] = 0;
		return 1;
	}
	*eq = 0;
	snprintf(key, klen, "%s", line);
	char *v = eq + 1;
	v[strcspn(v, "\r\n")] = 0;
	snprintf(val, vlen, "%s", v);
	return 1;
}

static void read_dips(const char *key, int val)
{
	if (!strcmp(key, "dip_count"))
		g_settings.dip_count = clampi(val, 0, RBX_DIP_BYTES_MAX);
	else if (!strncmp(key, "dip", 3) && key[3] >= '0' && key[3] <= '9') {
		int i = key[3] - '0';
		if (i < RBX_DIP_BYTES_MAX)
			g_settings.dip[i] = (uint8_t)val;
	}
}

// Per-game overridable items (global values kept in s_global)
typedef struct {
	int show_fps, aspect, filter, scan_type, scan_strength, border, border_color, rotate;
	rbx_port port[RBX_PROFILES];
	int touch_keys, touch_show;
} game_part;
static game_part s_global;   // values from the global settings file (written back to it while per-game settings are on)
static int s_game_on;        // per-game settings are on for the current game
static char s_game[16];      // current game's set name (per-game files <set>.cfg / <set>_game.cfg)
static void part_get(game_part *g)
{
	g->show_fps = g_settings.show_fps; g->aspect = g_settings.aspect; g->filter = g_settings.filter;
	g->scan_type = g_settings.scan_type; g->scan_strength = g_settings.scan_strength;
	g->border = g_settings.border; g->border_color = g_settings.border_color;
	memcpy(g->port, g_settings.port, sizeof(g->port));
	g->rotate = g_settings.rotate;
	g->touch_keys = g_settings.touch_keys; g->touch_show = g_settings.touch_show;
}
static void part_set(const game_part *g)
{
	g_settings.show_fps = g->show_fps; g_settings.aspect = g->aspect; g_settings.filter = g->filter;
	g_settings.scan_type = g->scan_type; g_settings.scan_strength = g->scan_strength;
	g_settings.border = g->border; g_settings.border_color = g->border_color;
	memcpy(g_settings.port, g->port, sizeof(g->port));
	g_settings.rotate = g->rotate;
	g_settings.touch_keys = g->touch_keys; g_settings.touch_show = g->touch_show;
}
// Old turbo lines (turbo_attack / turbo_jump, one key per button) seen in the file being read. If the file has no turbo_key,
// parse_end maps them to the single turbo key: attack -> button 1, else jump -> button 2
static int s_old_turbo[2], s_new_turbo;
static void parse_begin(void)
{
	s_old_turbo[0] = s_old_turbo[1] = -1;
	s_keyver = 0;
	s_new_turbo = 0;
}
// files before profiles per device (keyver < 3): 1P's controls go to every device
static void ports_spread(void)
{
	if (s_keyver >= 3)
		return;
	for (int i = 1; i < RBX_PROFILES; i++)
		g_settings.port[i] = g_settings.port[0];
}
static void parse_end_turbo(void);
static void parse_end(void)
{
	parse_end_turbo();
	ports_spread();
}
static void parse_end_turbo(void)
{
	if (s_new_turbo)
		return;
	// the old default (attack turbo on ×, jump off) was written to every file, chosen or not: turbo is now off by
	// default, so that pair means off
	if (s_old_turbo[0] == RBX_KEY_CROSS && (s_old_turbo[1] < 0 || s_old_turbo[1] == RBX_KEY_NONE))
		s_old_turbo[0] = RBX_KEY_NONE;
	if (s_old_turbo[0] >= 0 && s_old_turbo[0] != RBX_KEY_NONE) {
		g_settings.port[0].turbo_key = s_old_turbo[0];
		g_settings.port[0].turbo_btn = 1;
	} else if (s_old_turbo[1] >= 0 && s_old_turbo[1] != RBX_KEY_NONE) {
		g_settings.port[0].turbo_key = s_old_turbo[1];
		g_settings.port[0].turbo_btn = 2;
	} else if (s_old_turbo[0] >= 0 || s_old_turbo[1] >= 0)
		g_settings.port[0].turbo_key = RBX_KEY_NONE;
}
// One port line: Port 1 keeps the names from before ports (stick, turbo_*, combo_*, key_*), Port 2 has "p2_" before
// them (its keys: key2_*). Returns 1 if recognized
static int read_port(const char *key, int val)
{
	rbx_port *p = &g_settings.port[0];
	const char *dot = strchr(key, '.');
	if (dot) { // a device's profile: "<profile>.<item>"
		int i = 0;
		while (i < RBX_PROFILES && (strlen(re_pad_profile_key(i)) != (size_t)(dot - key) || strncmp(key, re_pad_profile_key(i), dot - key)))
			i++;
		if (i == RBX_PROFILES)
			return 0;
		p = &g_settings.port[i];
		key = dot + 1;
	} else if (!strncmp(key, "p2_", 3)) {
		p = &g_settings.port[1];
		key += 3;
	}
	if (!strcmp(key, "turbo_key")) {
		p->turbo_key = key_val(val, RBX_KEY_NONE);
		if (p == &g_settings.port[0])
			s_new_turbo = 1;
	} else if (!strcmp(key, "turbo_btn")) p->turbo_btn = clampi(val, 1, RBX_TURBO_BUTTONS);
	else if (!strcmp(key, "turbo_speed")) p->turbo_speed = clampi(val, 0, RBX_TURBO_SPEED_COUNT - 1);
	else if (!strcmp(key, "stick")) { // (files before each stick had its own map: 0 off, 1 left, 2 right, 3 both as the d-pad)
		rbx_stick_preset(p, 0, val == 1 || val == 3);
		rbx_stick_preset(p, 1, val == 2 || val == 3);
	} else if (!strncmp(key, "smap_", 5) && (key[5] == '0' || key[5] == '1') && key[6] >= '0' &&
		key[6] < '0' + RBX_SDIR_COUNT && !key[7])
		p->smap[key[5] - '0'][key[6] - '0'] = clampi(val, 0, RBX_STGT_COUNT - 1);
	else if (!strncmp(key, "keyx_", 5) && key[5] >= '0' && key[5] < '0' + RBX_ACT_COUNT && key[6] == '_' && key[7] >= '0' &&
		key[7] < '0' + RBX_KEY_EXTRA && !key[8])
		p->keyx[key[5] - '0'][key[7] - '0'] = key_val(val, RBX_KEY_NONE);
	else if (!strncmp(key, "combo_", 6)) { // combo_<key>=<mask of two keys>
		for (int k = 0; k < RBX_KEY_COUNT; k++)
			if (s_combo_keys[k] && !strcmp(key + 6, s_combo_keys[k])) {
				p->combo[k] = combo_ok(val) ? val : 0;
				return 1;
			}
		return 0;
	} else {
		for (int a = 0; a < RBX_ACT_COUNT; a++) {
			const int v = key_val(val, a >= RBX_ACT_B3 ? RBX_KEY_NONE : RBX_KEY_COUNT - 1);
			if (!strcmp(key, s_keymap_keys[a])) {
				p->keymap[a] = v;
				return 1;
			}
			if (!strcmp(key, s_keymap2_keys[a])) {
				g_settings.port[1].keymap[a] = v;
				return 1;
			}
		}
		return 0;
	}
	return 1;
}
// One overridable line (shared by global and per-game files). Returns 1 if recognized
static int read_part(const char *key, int val)
{
	if (!strcmp(key, "keyver")) s_keyver = val;
	else if (!strcmp(key, "show_fps")) g_settings.show_fps = val != 0;
	else if (!strcmp(key, "aspect")) g_settings.aspect = clampi(val, 0, RBX_ASPECT_COUNT - 1);
	else if (!strcmp(key, "filter")) g_settings.filter = clampi(val, 0, RBX_FILTER_COUNT - 1);
	else if (!strcmp(key, "scan_type")) g_settings.scan_type = clampi(val, 0, RBX_SCAN_COUNT - 1);
	else if (!strcmp(key, "scan_strength")) g_settings.scan_strength = clampi(val, 0, RBX_SCAN_STRENGTH_COUNT - 1);
	else if (!strcmp(key, "border")) g_settings.border = clampi(val, 0, RBX_BORDER_COUNT - 1);
	else if (!strcmp(key, "border_color")) g_settings.border_color = clampi(val, 0, RBX_BCOL_COUNT - 1);
	else if (!strcmp(key, "turbo_attack")) s_old_turbo[0] = key_val(val, RBX_KEY_NONE);
	else if (!strcmp(key, "turbo_jump")) s_old_turbo[1] = key_val(val, RBX_KEY_NONE);
	else if (!strcmp(key, "rotate")) g_settings.rotate = clampi(val, 0, RBX_ROT_COUNT - 1);
	else if (!strcmp(key, "touch_keys")) g_settings.touch_keys = val != 0;
	else if (!strcmp(key, "touch_show")) g_settings.touch_show = val != 0;
	else
		return read_port(key, val);
	return 1;
}
static void write_part(FILE *f, const game_part *g)
{
	fprintf(f, "keyver=5\n"); // (first: the keys below are read with it; 3: a profile per device, 4, 5: the sticks)
	fprintf(f, "show_fps=%d\naspect=%d\nfilter=%d\n", g->show_fps, g->aspect, g->filter);
	fprintf(f, "scan_type=%d\nscan_strength=%d\n", g->scan_type, g->scan_strength);
	fprintf(f, "border=%d\nborder_color=%d\n", g->border, g->border_color);
	for (int i = 0; i < RBX_PROFILES; i++) {
		const rbx_port *p = &g->port[i];
		char pre[16];
		snprintf(pre, sizeof(pre), "%s.", re_pad_profile_key(i));
		for (int a = 0; a < RBX_ACT_COUNT; a++)
			fprintf(f, "%s%s=%d\n", pre, s_keymap_keys[a], p->keymap[a]);
		for (int a = 0; a < RBX_ACT_COUNT; a++)
			for (int x = 0; x < RBX_KEY_EXTRA; x++)
				if (p->keyx[a][x] < RBX_KEY_COUNT)
					fprintf(f, "%skeyx_%d_%d=%d\n", pre, a, x, p->keyx[a][x]);
		for (int st = 0; st < 2; st++)
			for (int d = 0; d < RBX_SDIR_COUNT; d++)
				fprintf(f, "%ssmap_%d%d=%d\n", pre, st, d, p->smap[st][d]);
		fprintf(f, "%sturbo_key=%d\n%sturbo_btn=%d\n%sturbo_speed=%d\n", pre, p->turbo_key, pre, p->turbo_btn, pre,
			p->turbo_speed);
		for (int k = 0; k < RBX_KEY_COUNT; k++)
			if (s_combo_keys[k])
				fprintf(f, "%scombo_%s=%d\n", pre, s_combo_keys[k], p->combo[k]);
	}
	fprintf(f, "rotate=%d\ntouch_keys=%d\ntouch_show=%d\n", g->rotate, g->touch_keys, g->touch_show);
}

// Favorite games: set names in favorites.cfg, one per line
#define FAV_MAX 256
static char s_fav[FAV_MAX][16];
static int s_fav_n;

int rbx_fav_is(const char *set)
{
	for (int i = 0; i < s_fav_n; i++)
		if (!strcmp(s_fav[i], set))
			return 1;
	return 0;
}

static void fav_load(void)
{
	FILE *f = fopen(FAV_PATH, "r");
	if (!f)
		return;
	char line[64];
	s_fav_n = 0;
	while (s_fav_n < FAV_MAX && fgets(line, sizeof(line), f)) {
		const size_t len = strcspn(line, "\r\n");
		line[len] = 0;
		if (len && len < sizeof(s_fav[0]) && !rbx_fav_is(line))
			memcpy(s_fav[s_fav_n++], line, len + 1);
	}
	fclose(f);
}

void rbx_fav_toggle(const char *set)
{
	int i = 0;
	while (i < s_fav_n && strcmp(s_fav[i], set) != 0)
		i++;
	if (i < s_fav_n) {
		memmove(s_fav[i], s_fav[i + 1], (size_t)(s_fav_n - i - 1) * sizeof(s_fav[0]));
		s_fav_n--;
	} else if (s_fav_n < FAV_MAX && strlen(set) < sizeof(s_fav[0]))
		strcpy(s_fav[s_fav_n++], set);
	FILE *f = fopen(FAV_PATH, "w");
	if (!f) {
		RBX_LOG("settings: save failed %s", FAV_PATH);
		return;
	}
	for (i = 0; i < s_fav_n; i++)
		fprintf(f, "%s\n", s_fav[i]);
	fclose(f);
}

// the games' files, once in the data folder itself, to cfg/ (the app's own settings.cfg, favorites.cfg, touch.cfg stay)
void rbx_settings_move_game_files(void)
{
	sceIoMkdir(GAME_CFG_DIR, 0777);
	const SceUID d = sceIoDopen(RBX_DATA_DIR);
	if (d < 0)
		return;
	SceIoDirent de;
	int n = 0;
	for (;;) {
		memset(&de, 0, sizeof(de));
		if (sceIoDread(d, &de) <= 0)
			break;
		const size_t l = strlen(de.d_name);
		if (l < 5 || strcmp(de.d_name + l - 4, ".cfg") || SCE_S_ISDIR(de.d_stat.st_mode) || !strcmp(de.d_name, "settings.cfg") ||
		    !strcmp(de.d_name, "favorites.cfg") || !strcmp(de.d_name, "touch.cfg"))
			continue;
		char from[128], to[128];
		snprintf(from, sizeof(from), RBX_DATA_DIR "%s", de.d_name);
		snprintf(to, sizeof(to), GAME_CFG_DIR "%s", de.d_name);
		if (sceIoRename(from, to) >= 0)
			n++;
	}
	sceIoDclose(d);
	if (n)
		RBX_LOG("settings: %d game files moved to cfg/", n);
}

void rbx_settings_load(void)
{
	for (int i = 0; i < RBX_PROFILES; i++) // (every device's profile from the defaults first)
		rbx_settings_default_port(i);
	fav_load();
	// If missing in the new folder, read settings from the nbbatman-only era (the next save goes to the new folder)
	FILE *f = fopen(SETTINGS_PATH, "r");
	if (!f)
		f = fopen(RBX_OLD_DATA_DIR "settings.cfg", "r");
	if (!f)
		return;
	char key[32], sval[32];
	parse_begin();
	while (read_kv(f, key, sizeof(key), sval, sizeof(sval))) {
		const int val = atoi(sval);
		if (read_part(key, val)) continue;
		else if (!strcmp(key, "menu_confirm")) g_settings.menu_confirm = clampi(val, 0, RBX_CONFIRM_COUNT - 1);
		else if (!strcmp(key, "last_game")) snprintf(g_settings.last_game, sizeof(g_settings.last_game), "%s", sval);
		else if (!strcmp(key, "show_list")) g_settings.show_list = val != 0;
		else if (!strcmp(key, "list_filter")) g_settings.list_filter = clampi(val, 0, RBX_LIST_FILTER_COUNT - 1);
		else if (!strcmp(key, "list_group")) g_settings.list_group = clampi(val, 0, RBX_GROUP_COUNT - 1);
		else if (!strcmp(key, "list_snap")) g_settings.list_snap = val != 0;
		else if (!strcmp(key, "list_all")) g_settings.list_filter = val ? RBX_LIST_ALL : RBX_LIST_PLAYABLE; // (old two-state filter)
		else if (!strcmp(key, "theme")) g_settings.theme = clampi(val, 0, RBX_THEME_COUNT - 1);
		else if (!strcmp(key, "pad_p1")) g_settings.pad_src[0] = clampi(val, 0, RE_PAD_SRC_COUNT - 1);
		else if (!strcmp(key, "pad_p2")) g_settings.pad_src[1] = clampi(val, 0, RE_PAD_SRC_COUNT - 1);
		else if (!strcmp(key, "input_engine")) g_settings.input_engine = clampi(val, 0, RE_PAD_ENGINE_COUNT - 1);
		else if (!strncmp(key, "dip", 3)) read_dips(key, val); // (nbbatman-only era file: nbbatman's DIP)
	}
	fclose(f);
	parse_end();
	part_get(&s_global);
	g_settings.input_engine = re_pad_engine_fit(g_settings.input_engine); // (as the Vita is: VitaControl when a list loads it)
	RBX_LOG("settings: fps %d aspect %d filter %d scan %d keys %d/%d/%d/%d last %s",
		g_settings.show_fps, g_settings.aspect, g_settings.filter, g_settings.scan_type,
		g_settings.port[0].keymap[0], g_settings.port[0].keymap[1], g_settings.port[0].keymap[2],
		g_settings.port[0].keymap[3], g_settings.last_game);
}

static void game_path(char *buf, int len)
{
	snprintf(buf, len, GAME_CFG_DIR "%s.cfg", s_game);
}

void rbx_settings_touch_follow(int keep)
{
	char p[96];
	snprintf(p, sizeof(p), GAME_CFG_DIR "%s_touch.cfg", s_game);
	rbx_touch_select(s_game[0] && s_game_on ? p : NULL, keep);
}

void rbx_settings_load_game(const char *set)
{
	snprintf(s_game, sizeof(s_game), "%s", set);
	snprintf(g_settings.last_game, sizeof(g_settings.last_game), "%s", set);
	char p[96];
	// Per-game settings: if the previous game's were on, revert to global values, then apply this game's if present
	if (s_game_on) {
		part_set(&s_global);
		s_game_on = 0;
	} else
		part_get(&s_global);
	snprintf(p, sizeof(p), GAME_CFG_DIR "%s_game.cfg", s_game);
	FILE *f = fopen(p, "r");
	if (f) {
		char key[32], sval[32];
		parse_begin();
		while (read_kv(f, key, sizeof(key), sval, sizeof(sval)))
			read_part(key, atoi(sval));
		fclose(f);
		parse_end();
		s_game_on = 1;
		RBX_LOG("settings: game settings %s", p);
	}
	rbx_settings_touch_follow(0); // (the game's touch layout with its settings, else the global one)
	game_path(p, sizeof(p));
	f = fopen(p, "r");
	if (!f) {
		// No per-game file: nbbatman family keeps current values (default 157/247 or from the old settings file), other games use driver defaults
		if (strncmp(set, "nbbatman", 8) != 0)
			g_settings.dip_count = 0;
		return;
	}
	g_settings.dip_count = 0;
	char key[32], sval[32];
	while (read_kv(f, key, sizeof(key), sval, sizeof(sval)))
		read_dips(key, atoi(sval));
	fclose(f);
}

void rbx_settings_save(void)
{
	FILE *f = fopen(SETTINGS_PATH, "w");
	if (!f) {
		RBX_LOG("settings: save failed %s", SETTINGS_PATH);
		return;
	}
	// While per-game settings are on, those items go to the per-game file (below); the global file keeps the global values
	if (!s_game_on)
		part_get(&s_global);
	write_part(f, &s_global);
	fprintf(f, "menu_confirm=%d\n", g_settings.menu_confirm);
	// Cheats (start stage, infinite health) are not saved: every launch starts with them off
	if (g_settings.last_game[0])
		fprintf(f, "last_game=%s\n", g_settings.last_game);
	if (g_settings.show_list)
		fprintf(f, "show_list=1\n");
	fprintf(f, "list_filter=%d\nlist_group=%d\nlist_snap=%d\ntheme=%d\n", g_settings.list_filter, g_settings.list_group,
		g_settings.list_snap, g_settings.theme);
	fprintf(f, "pad_p1=%d\npad_p2=%d\ninput_engine=%d\n", g_settings.pad_src[0], g_settings.pad_src[1], g_settings.input_engine);
	fclose(f);
	if (!s_game[0])
		return;
	if (s_game_on)
		rbx_settings_save_game();
	char p[96];
	game_path(p, sizeof(p));
	f = fopen(p, "w");
	if (!f)
		return;
	fprintf(f, "dip_count=%d\n", g_settings.dip_count);
	for (int i = 0; i < g_settings.dip_count; i++)
		fprintf(f, "dip%d=%d\n", i, g_settings.dip[i]);
	fclose(f);
}

int rbx_settings_has_game(void)
{
	return s_game_on;
}

void rbx_settings_use_global(void)
{
	if (s_game_on) {
		part_set(&s_global);
		s_game_on = 0;
	}
	s_game[0] = 0; // (no per-game files written until the next rbx_settings_load_game)
	rbx_settings_touch_follow(0);
}

int rbx_settings_save_game(void)
{
	if (!s_game[0])
		return 0;
	char p[96];
	snprintf(p, sizeof(p), GAME_CFG_DIR "%s_game.cfg", s_game);
	FILE *f = fopen(p, "w");
	if (!f) {
		RBX_LOG("settings: game save failed %s", p);
		return 0;
	}
	game_part g;
	part_get(&g);
	write_part(f, &g);
	fclose(f);
	if (!s_game_on) {
		s_game_on = 1;
		rbx_settings_touch_follow(1); // (the touch layout now is the game's too)
	}
	return 1;
}

int rbx_settings_delete_game(void)
{
	if (!s_game[0] || !s_game_on)
		return 0;
	char p[96];
	snprintf(p, sizeof(p), GAME_CFG_DIR "%s_game.cfg", s_game);
	if (remove(p) != 0) {
		RBX_LOG("settings: game delete failed %s", p);
		return 0;
	}
	s_game_on = 0;
	snprintf(p, sizeof(p), GAME_CFG_DIR "%s_touch.cfg", s_game);
	remove(p);
	part_set(&s_global); // (straight to global values)
	rbx_settings_touch_follow(0);
	return 1;
}
