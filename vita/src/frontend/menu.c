// In-game menu, game list, loading screen, notices, FPS box, touch buttons and the PS button. Ported from ReSys12
// vita/src/frontend/menu.c (common UI of the Re series of Vita emulators): same layout, controls, animations and
// features (themes, choice windows, toggle switches, smooth scrolling, blinking focus, star favorites, touch layout
// editor, update window), with ReBOX's board tabs, DIP switches, cheats and per-game settings.
// Drawing is in ui.c (system default PGF font drawn to a CPU canvas, shown as vita2d overlay layers, video.h).
#include "frontend/menu.h"
#include "frontend/lang.h"
#include "frontend/neon.h"
#include "frontend/ime.h"
#include "frontend/gamelist.h"

#include <psp2/appmgr.h>
#include <psp2/ctrl.h>
#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>
#include <psp2/kernel/processmgr.h>
#include <psp2/power.h>
#include <psp2/rtc.h>
#include <psp2/shellutil.h>
#include <taihen.h>
#include <vita2d.h>
#include <psp2/display.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "common/re_pad.h"
#include "frontend/avsnap.h"
#include "frontend/present.h"
#include "frontend/settings.h"
#include "frontend/states.h"
#include "frontend/testpad.h"
#include "frontend/touch.h"
#include "frontend/ui.h"
#include "frontend/update.h"
#include "frontend/video.h"
#include "platform/log.h"
#include "platform/timer.h"
#include "frontend/ime.h"
#include "frontend/core_api.h"
#include "rbx_games.h"
#include "rbx_kctrl.h"
#include "rbx_version.h" // generated at build time (tools/git_version.cmake)
// Device choice list: each source with its device ("Controller 1: Switch Pro")
static char s_devtext[RE_PAD_SRC_COUNT][40];

#define MENU_OPEN_COMBO (SCE_CTRL_SELECT | SCE_CTRL_START)

#define SCREEN_W 960
#define SCREEN_H 544
#define PANEL_W  640
#define LINE_H   40 // menu rows (pages taller than the screen scroll)
#define ABOUT_LH 30 // About text lines
#define GL_LINE  44 // game list rows (the list scrolls)
#define TITLE_H  56
#define FOOT_H   48
#define VALUE_X  330 // from panel left edge to the value column

// Colour themes (Display > Theme; ReST-V's layout and shapes). Pink is the ReBOX default, Red ReSys12's, Blue ReST-V's. A theme
// is a gradient (top, bottom), the panel, the accent (selection), dim text, row text and the outline colour; the other
// colours are made from these
typedef struct { uint8_t top[3], bot[3], panel[3], sel[3], dim[3], item[3], edge[3]; } ThemeDef;
static const ThemeDef s_theme_def[RBX_THEME_COUNT] = {
	{ { 70, 6, 14 }, { 180, 22, 40 }, { 0x46, 0x0A, 0x12 }, { 0xFF, 0x7C, 0x7C }, { 0xD8, 0xA4, 0xAC }, { 0xFA, 0xDC, 0xE0 }, { 0xE0, 0x4C, 0x5C } },
	{ { 6, 22, 70 }, { 20, 86, 180 }, { 0x0A, 0x1C, 0x46 }, { 0x6C, 0xD0, 0xFF }, { 0x9C, 0xB0, 0xD4 }, { 0xD6, 0xE2, 0xFA }, { 0x4C, 0x8C, 0xE0 } },
	{ { 6, 50, 24 }, { 20, 136, 66 }, { 0x0A, 0x36, 0x1C }, { 0x7C, 0xF0, 0xA0 }, { 0xA0, 0xCC, 0xAC }, { 0xDA, 0xF6, 0xE0 }, { 0x4C, 0xC0, 0x78 } },
	{ { 40, 8, 70 }, { 110, 40, 180 }, { 0x2C, 0x0E, 0x4A }, { 0xD0, 0x9C, 0xFF }, { 0xC0, 0xA8, 0xD8 }, { 0xEC, 0xDE, 0xFA }, { 0xA0, 0x6C, 0xE0 } },
	{ { 70, 30, 4 }, { 186, 88, 20 }, { 0x48, 0x22, 0x08 }, { 0xFF, 0xC4, 0x6C }, { 0xDC, 0xBC, 0x9C }, { 0xFA, 0xEA, 0xD6 }, { 0xE8, 0x96, 0x40 } },
	{ { 24, 24, 28 }, { 70, 72, 82 }, { 0x26, 0x28, 0x2E }, { 0xE0, 0xE6, 0xF0 }, { 0xA8, 0xAC, 0xB4 }, { 0xE2, 0xE4, 0xE8 }, { 0x90, 0x94, 0xA0 } },
	{ { 72, 8, 46 }, { 196, 46, 120 }, { 0x48, 0x0C, 0x30 }, { 0xFF, 0x8C, 0xC8 }, { 0xE0, 0xA8, 0xC8 }, { 0xFC, 0xE0, 0xF0 }, { 0xF0, 0x5C, 0xA8 } },
};
static const char *const s_theme_name[RBX_THEME_COUNT] = { "Red", "Blue", "Green", "Purple", "Orange", "Graphite", "Pink" };
typedef struct {
	uint32_t bg, panel, item, dim, sel, hilite, edge, thumb, focus_still, focus_bright, focus_glow, shade, band, sep, toggle_on,
		toggle_off, grey, grey_sel, bar_track, bar_top, bar_bot, glow1, glow2, glow3, spinner;
	uint8_t panel_rgb[3];
} Theme;
static Theme s_th;
static int s_theme_cur = -1, s_theme_gen; // theme the colours were made for; counts changes (redraw what was kept)
static uint32_t rgb(const uint8_t *c, unsigned a) { return UI_RGBA(c[0], c[1], c[2], a); }
static uint32_t mix(const uint8_t *a, const uint8_t *b, int t, unsigned al) // a..b, t/256 towards b
{
	return UI_RGBA(a[0] + ((b[0] - a[0]) * t >> 8), a[1] + ((b[1] - a[1]) * t >> 8), a[2] + ((b[2] - a[2]) * t >> 8), al);
}
static void theme_make(int k)
{
	static const uint8_t white[3] = { 0xFF, 0xFF, 0xFF }, black[3] = { 0, 0, 0 };
	const ThemeDef *d = &s_theme_def[k];
	Theme *t = &s_th;
	t->bg = mix(d->top, black, 180, 0xC0);
	t->panel = rgb(d->panel, 0xFF); // opaque: filled without reading what is under it (fast)
	memcpy(t->panel_rgb, d->panel, 3);
	t->item = rgb(d->item, 0xFF);
	t->dim = rgb(d->dim, 0xFF);
	t->sel = rgb(d->sel, 0xFF);
	t->hilite = mix(d->sel, white, 40, 0x30);
	t->edge = rgb(d->edge, 0xFF);
	t->thumb = mix(d->sel, white, 50, 0xB0);
	t->focus_still = rgb(d->sel, 0xB0);
	t->focus_bright = mix(d->sel, white, 190, 0xFF);
	t->focus_glow = rgb(d->sel, 0x50);
	t->shade = mix(d->top, black, 200, 0x30);
	t->band = mix(d->top, black, 170, 0xFF);
	t->sep = mix(d->edge, white, 60, 0xFF);
	t->toggle_on = mix(d->edge, d->sel, 100, 0xFF);
	t->toggle_off = mix(d->dim, d->panel, 128, 0xFF);
	t->grey = mix(d->dim, d->panel, 70, 0xFF);
	t->grey_sel = mix(d->dim, d->item, 128, 0xFF);
	t->bar_track = mix(d->panel, black, 80, 0xE0);
	t->bar_top = mix(d->sel, white, 40, 0xFF);
	t->bar_bot = mix(d->edge, black, 40, 0xFF);
	t->glow1 = rgb(d->sel, 0x28);
	t->glow2 = mix(d->sel, white, 60, 0x60);
	t->glow3 = mix(d->sel, white, 220, 0xFF);
	t->spinner = mix(d->sel, white, 110, 0xFF);
	s_theme_cur = k;
	s_theme_gen++;
}
static const Theme *theme(void)
{
	const int k = g_settings.theme >= 0 && g_settings.theme < RBX_THEME_COUNT ? g_settings.theme : RBX_THEME_PINK;
	if (k != s_theme_cur)
		theme_make(k);
	return &s_th;
}
uint32_t rbx_menu_accent(void) { return theme()->edge; }
// "BOX": the theme's accent; Graphite (the default, its accent grey) has it yellow
static uint32_t brand_col(void)
{
	const uint32_t a = rbx_menu_accent();
	return s_theme_cur == RBX_THEME_GRAPHITE ? UI_RGBA(0xFF, 0xCC, 0x33, 0xFF) : s_theme_cur == RBX_THEME_PINK ? UI_RGBA(0xD8, 0x3C, 0xB0, 0xFF) : a;
}
#define COL_BG    (theme()->bg)
#define COL_PANEL (theme()->panel)
#define COL_TEXT  UI_RGBA(0xFF, 0xFF, 0xFF, 0xFF)
#define COL_ITEM  (theme()->item) // rows that are not selected
#define COL_DIM   (theme()->dim)
#define COL_SEL   (theme()->sel)
#define COL_BRAND brand_col()
#define COL_WARN  UI_RGBA(0xFF, 0xC0, 0x40, 0xFF)
#define COL_HILITE (theme()->hilite) // selected row bar
#define COL_EDGE  (theme()->edge)   // panel outline, separators
#define COL_THUMB (theme()->thumb)  // scroll bar

// button symbols inside text (drawn by draw_key)
#define G_TRI "\x10"
#define G_CIR "\x11"
#define G_CRO "\x12"
#define G_SQU "\x13"


// rounded rectangle drawn row by row (no overlapping parts, so translucent colours stay even)
static void rrect(UiCanvas *c, int x, int y, int w, int h, int r, uint32_t col)
{
	if (r * 2 > h) r = h / 2;
	if (r * 2 > w) r = w / 2;
	for (int yy = 0; yy < h; yy++) {
		int in = 0;
		const int d = yy < r ? r - yy : (yy >= h - r ? yy - (h - r - 1) : 0);
		if (d > 0) {
			const float dy = (float)d - 0.5f;
			in = r - (int)sqrtf((float)(r * r) - dy * dy);
		}
		ui_rect(c, x + in, y + yy, w - 2 * in, 1, col);
	}
}
// outline of a rounded rectangle, t px thick (rows of the outer shape minus the inner one: no overlap)
static void rrect_outline(UiCanvas *c, int x, int y, int w, int h, int r, int t, uint32_t col)
{
	if (r * 2 > h) r = h / 2;
	const int ri = r - t > 0 ? r - t : 0;
	for (int yy = 0; yy < h; yy++) {
		int in = 0;
		const int d = yy < r ? r - yy : (yy >= h - r ? yy - (h - r - 1) : 0);
		if (d > 0) {
			const float dy = (float)d - 0.5f;
			in = r - (int)sqrtf((float)(r * r) - dy * dy);
		}
		if (yy < t || yy >= h - t) {
			ui_rect(c, x + in, y + yy, w - 2 * in, 1, col);
			continue;
		}
		int ii = 0; // inner shape inset on this row (inner rect is t smaller on every side)
		const int yi = yy - t, hi = h - 2 * t, di = yi < ri ? ri - yi : (yi >= hi - ri ? yi - (hi - ri - 1) : 0);
		if (di > 0 && ri > 0) {
			const float dy = (float)di - 0.5f;
			ii = ri - (int)sqrtf((float)(ri * ri) - dy * dy);
		}
		const int l0 = x + in, l1 = x + t + ii, r0 = x + w - t - ii, r1 = x + w - in;
		if (l1 > l0) ui_rect(c, l0, y + yy, l1 - l0, 1, col);
		if (r1 > r0) ui_rect(c, r0, y + yy, r1 - r0, 1, col);
	}
}
// The focus outline blinks. It is a small layer of its own (slot 1, over the menu canvas), drawn once per position; the
// blink is that layer's opacity, set every frame (no CPU drawing). Only the topmost focus blinks; a focus under an open
// window keeps a steady outline
#define COL_FOCUS_STILL (theme()->focus_still)
static UiCanvas *s_focus_cv;
static int s_focus_x = -1, s_focus_y, s_focus_w, s_focus_h, s_focus_on, s_focus_want, s_focus_gen;
static int s_focus_cy0, s_focus_cy1 = SCREEN_H; // rows the focus layer may cover (the list / menu rows area)
static void focus_at(UiCanvas *c, int x, int y, int w, int h)
{
	enum { CW = SCREEN_W, CH = 56 + 80 };   // (the focused row grown to three lines: 114 + 4)
	if (y - 2 < s_focus_cy0 || y + h + 2 > s_focus_cy1) {
		// partly past the rows area (while scrolling): a steady outline in the canvas, cut there like the rows
		rrect_outline(c, x, y, w, h, 6, 2, COL_FOCUS_STILL);
		return;
	}
	s_focus_want = 1;
	if (!s_focus_cv)
		s_focus_cv = ui_canvas(1, 0, 0, CW, CH);
	if (!s_focus_cv || w + 4 > CW || h + 4 > CH)
		return;
	if (x == s_focus_x && y == s_focus_y && w == s_focus_w && h == s_focus_h && s_focus_on && s_focus_gen == s_theme_gen)
		return;
	if (w != s_focus_w || h != s_focus_h || s_focus_x < 0 || s_focus_gen != s_theme_gen) {
		ui_clear(s_focus_cv);
		rrect_outline(s_focus_cv, 2, 2, w, h, 6, 2, theme()->focus_bright);
		rrect_outline(s_focus_cv, 0, 0, w + 4, h + 4, 8, 2, theme()->focus_glow); // soft glow outside
		s_focus_gen = s_theme_gen;
	}
	s_focus_x = x;
	s_focus_y = y;
	s_focus_w = w;
	s_focus_h = h;
	ui_canvas_move(s_focus_cv, x - 2, y - 2);
	ui_commit(s_focus_cv);
	s_focus_on = 1;
}
static void focus_off(void)
{
	if (s_focus_on)
		ui_hide(s_focus_cv);
	s_focus_on = 0;
}
static void focus_begin(void)
{
	s_focus_want = 0;
	s_focus_cy0 = 0;
	s_focus_cy1 = SCREEN_H;
}
static void focus_end(void)
{
	if (!s_focus_want)
		focus_off();
}
static void focus_tick(void)
{
	if (!s_focus_on)
		return;
	const float p = 0.5f + 0.5f * sinf((float)(rbx_time_us() % 1000000) * 6.2832f / 1e6f);
	rbx_video_layer_alpha(1, 0.12f + 0.88f * p);
}
// favourites as a star: five points, edges smoothed (4 x 4 samples a pixel)
static void star(UiCanvas *c, float cx, float cy, float r, uint32_t rgb)
{
	float vx[10], vy[10];
	for (int k = 0; k < 10; k++) {
		const float a = -1.5708f + k * 3.14159f / 5.0f, rr = (k & 1) ? r * 0.42f : r;
		vx[k] = cx + rr * cosf(a);
		vy[k] = cy + rr * sinf(a);
	}
	for (int py = (int)(cy - r) - 1; py <= (int)(cy + r) + 1; py++)
		for (int px = (int)(cx - r) - 1; px <= (int)(cx + r) + 1; px++) {
			int hits = 0;
			for (int s = 0; s < 16; s++) {
				const float sx = px + 0.125f + 0.25f * (s & 3), sy = py + 0.125f + 0.25f * (s >> 2);
				int in = 0;
				for (int a = 0, b = 9; a < 10; b = a++)
					if ((vy[a] > sy) != (vy[b] > sy) && sx < (vx[b] - vx[a]) * (sy - vy[a]) / (vy[b] - vy[a]) + vx[a])
						in = !in;
				hits += in;
			}
			if (hits)
				ui_rect(c, px, py, 1, 1, (rgb & 0x00FFFFFFu) | ((uint32_t)(hits * 255 / 16) << 24));
		}
}
#define COL_STAR UI_RGBA(0xFF, 0xD2, 0x48, 0xFF)
// the settings button: a gear of eight teeth with a hole, edges smoothed as the star's
static void gear(UiCanvas *c, float cx, float cy, float r, uint32_t rgb)
{
	const float body = r * 0.72f, hole = r * 0.32f;
	for (int py = (int)(cy - r) - 1; py <= (int)(cy + r) + 1; py++)
		for (int px = (int)(cx - r) - 1; px <= (int)(cx + r) + 1; px++) {
			int hits = 0;
			for (int s = 0; s < 16; s++) {
				const float dx = px + 0.125f + 0.25f * (s & 3) - cx, dy = py + 0.125f + 0.25f * (s >> 2) - cy;
				const float d = sqrtf(dx * dx + dy * dy);
				if (d < hole || d > r)
					continue;
				hits += d <= body || cosf(8.0f * atan2f(dy, dx)) > 0.25f;   // (the teeth: the outer ring in 8 places)
			}
			if (hits)
				ui_rect(c, px, py, 1, 1, (rgb & 0x00FFFFFFu) | ((uint32_t)(hits * 255 / 16) << 24));
		}
}
// windows with slightly rounded corners and a soft shadow: drop shadow of a window (offset 4, 6, soft over 9 px), only
// outside it: straight edges as one-colour runs, the corners a pixel at a time
enum { SH_OX = 4, SH_OY = 6, SH_B = 9, SH_A = 0x58 };
static uint32_t sh_col(int dx, int dy)
{
	const int d2 = dx * dx + dy * dy;
	return d2 >= SH_B * SH_B ? 0 : UI_RGBA(0x04, 0x04, 0x06, SH_A * (SH_B * SH_B - d2) / (SH_B * SH_B));
}
static void sh_block(UiCanvas *c, int x0, int y0, int x1, int y1, int sx0, int sy0, int sx1, int sy1)
{
	for (int yy = y0; yy < y1; yy++)
		for (int xx = x0; xx < x1; xx++) {
			const uint32_t col = sh_col(xx < sx0 ? sx0 - xx : (xx >= sx1 ? xx - sx1 + 1 : 0), yy < sy0 ? sy0 - yy : (yy >= sy1 ? yy - sy1 + 1 : 0));
			if (col)
				ui_rect(c, xx, yy, 1, 1, col);
		}
}
static void window_box(UiCanvas *c, int x, int y, int w, int h, uint32_t edge, uint32_t fill)
{
	const int sx0 = x + SH_OX, sy0 = y + SH_OY, sx1 = x + w + SH_OX, sy1 = y + h + SH_OY, B = SH_B;
	const int ex = x + w + 1, ey = y + h + 1; // first column / row past the window and its outline
	for (int xx = ex; xx < sx1 + B; xx++) { // right edge (rows beside the window)
		const uint32_t col = sh_col(xx >= sx1 ? xx - sx1 + 1 : 0, 0);
		if (col)
			ui_rect(c, xx, sy0, 1, ey - sy0, col);
	}
	for (int xx = sx0 - B; xx < x - 1; xx++) { // left edge
		const uint32_t col = sh_col(sx0 - xx, 0);
		if (col)
			ui_rect(c, xx, sy0, 1, ey - sy0, col);
	}
	for (int yy = ey; yy < sy1 + B; yy++) { // bottom edge
		const uint32_t col = sh_col(0, yy >= sy1 ? yy - sy1 + 1 : 0);
		if (col)
			ui_rect(c, sx0, yy, ex - sx0, 1, col);
	}
	sh_block(c, ex, sy0 - B, sx1 + B, sy0, sx0, sy0, sx1, sy1); // corners
	sh_block(c, ex, ey, sx1 + B, sy1 + B, sx0, sy0, sx1, sy1);
	sh_block(c, sx0 - B, ey, sx0, sy1 + B, sx0, sy0, sx1, sy1);
	rrect_outline(c, x - 1, y - 1, w + 2, h + 2, 7, 1, edge);
	rrect(c, x, y, w, h, 6, fill);
}
// the menu background: one colour per row (vertical gradient; the list also gets its header and footer bands). Filled
// row by row: copying a whole-screen image took 8 ms on the device
static uint32_t blend_px(uint32_t d, uint32_t c)
{
	const unsigned a = c >> 24;
	uint32_t o = 0xFF000000u;
	for (int k = 0; k < 24; k += 8)
		o |= ((((d >> k) & 0xFF) * (255 - a) + ((c >> k) & 0xFF) * a + 127) / 255) << k;
	return o;
}
#define LIST_FOOT_H 40 // the list's footer band (one line of button help)
static const uint32_t *theme_rows(int bands)
{
	static uint32_t rows[2][SCREEN_H];
	static int made_gen = -1;
	const Theme *th = theme();
	if (made_gen != s_theme_gen) {
		made_gen = s_theme_gen;
		const ThemeDef *d = &s_theme_def[s_theme_cur];
		const uint32_t band = th->band & 0xFFFFFFu, sep = th->sep & 0xFFFFFFu;
		for (int y = 0; y < SCREEN_H; y++) {
			const int t = y * 256 / (SCREEN_H - 1);
			rows[0][y] = rows[1][y] = mix(d->top, d->bot, t, 0xFF); // deep -> bright
			if (y < TITLE_H - 8)
				rows[1][y] = blend_px(rows[1][y], band | 0x50000000u);
			else if (y == TITLE_H - 8 || y == SCREEN_H - LIST_FOOT_H)
				rows[1][y] = blend_px(rows[1][y], sep | 0x60000000u);
			else if (y > SCREEN_H - LIST_FOOT_H)
				rows[1][y] = blend_px(rows[1][y], band | 0x60000000u);
		}
	}
	return rows[bands ? 1 : 0];
}
// the list's background over a still preview (user 11-14): the Graphite theme's background under the picture (whatever the
// theme), the picture over it at 60% (video.c), and the list's own background fully see-through (alpha 0) so the text
// drawn over it stays opaque. theme_rows_see: the see-through rows for the canvas; s_under_cv: the graphite rows
static UiCanvas *s_under_cv;
// the theme's own background rows as an under layer (the list's canvas see-through always: no preview, the theme under
// it; a preview, Graphite and the picture or video. The switch needs no redraw of the canvas: no hitch, user 11-15)
static UiCanvas *s_under_theme_cv;
static const ReiTex *under_theme(void)
{
	static int made_gen = -1;
	const uint32_t *op = theme_rows(1);
	if (made_gen != s_theme_gen || !s_under_theme_cv) {
		if (!s_under_theme_cv)
			s_under_theme_cv = ui_canvas(-1, 0, 0, 8, SCREEN_H);
		if (!s_under_theme_cv)
			return NULL;
		ui_fill_rows(s_under_theme_cv, op, 0, 0, 8, SCREEN_H);
		ui_commit(s_under_theme_cv);
		made_gen = s_theme_gen;
	}
	return ui_tex(s_under_theme_cv);
}
static const uint32_t *theme_rows_see(void)
{
	static uint32_t rows[SCREEN_H];   // (all 0: see-through)
	static int made;
	if (!made) {
		static const uint8_t black[3] = { 0, 0, 0 };
		static const uint8_t white[3] = { 0xFF, 0xFF, 0xFF };
		const ThemeDef *d = &s_theme_def[RBX_THEME_GRAPHITE];
		static uint32_t op[SCREEN_H];
		const uint32_t band = mix(d->top, black, 170, 0xFF) & 0xFFFFFFu, sep = mix(d->edge, white, 60, 0xFF) & 0xFFFFFFu;
		for (int y = 0; y < SCREEN_H; y++) {   // (as theme_rows(1), in Graphite's colours)
			op[y] = mix(d->top, d->bot, y * 256 / (SCREEN_H - 1), 0xFF);
			if (y < TITLE_H - 8)
				op[y] = blend_px(op[y], band | 0x50000000u);
			else if (y == TITLE_H - 8 || y == SCREEN_H - LIST_FOOT_H)
				op[y] = blend_px(op[y], sep | 0x60000000u);
			else if (y > SCREEN_H - LIST_FOOT_H)
				op[y] = blend_px(op[y], band | 0x60000000u);
		}
		if (!s_under_cv)
			s_under_cv = ui_canvas(-1, 0, 0, 8, SCREEN_H);
		if (s_under_cv) {
			ui_fill_rows(s_under_cv, op, 0, 0, 8, SCREEN_H);
			ui_commit(s_under_cv);
			made = 1;
		}
	}
	return rows;
}
// premultiplied d towards premultiplied s by al / 255 (a fade to the see-through background: blending over it would
// leave the faded part opaque)
static uint32_t lerp_px(uint32_t d, uint32_t s, unsigned al)
{
	uint32_t o = 0;
	for (int k = 0; k < 32; k += 8)
		o |= ((((d >> k) & 0xFF) * (255 - al) + ((s >> k) & 0xFF) * al + 127) / 255) << k;
	return o;
}

// test (log builds): the file ux0:data/rebox/shot (shotload: the loading screen's) makes the menu / list / loading
// canvas an image (ux0:data/rebox/shot.ppm, the canvas over black) and is removed
static void shot_check_as(UiCanvas *c, const char *trigger)
{
#ifdef RBX_ENABLE_LOG
	static unsigned tick;
	SceIoStat st;
	// (rbx_testfile_seen, ADR-161: a watched file looked for here only once the watcher thread saw it)
	if (++tick % 15 == 0 && rbx_testfile_seen(trigger) && sceIoGetstat(trigger, &st) >= 0) {
		sceIoRemove(trigger);
		ui_save_ppm(c, RBX_DATA_DIR "shot.tmp", UI_RGBA(0, 0, 0, 0xFF));
		sceIoRemove(RBX_DATA_DIR "shot.ppm");
		sceIoRename(RBX_DATA_DIR "shot.tmp", RBX_DATA_DIR "shot.ppm"); // (complete when it appears)
		RBX_LOG("menu: shot.ppm written");
	}
	// shotfb: the screen as shown (the canvas with everything the GPU draws over it, such as the list's row strips)
	if (tick % 15 == 7 && rbx_testfile_seen(RBX_DATA_DIR "shotfb") && sceIoGetstat(RBX_DATA_DIR "shotfb", &st) >= 0) {
		sceIoRemove(RBX_DATA_DIR "shotfb");
		SceDisplayFrameBuf dfb;
		memset(&dfb, 0, sizeof(dfb));
		dfb.size = sizeof(dfb);
		const uint32_t *fb = sceDisplayGetFrameBuf(&dfb, SCE_DISPLAY_SETBUF_NEXTFRAME) >= 0 ? (const uint32_t *)dfb.base : NULL;
		const int pitch = fb ? (int)dfb.pitch : SCREEN_W;
		FILE *f = fb ? fopen(RBX_DATA_DIR "shot.tmp", "wb") : NULL;
		if (f) {
			static uint8_t row[SCREEN_W * 3];
			fprintf(f, "P6\n%d %d\n255\n", SCREEN_W, SCREEN_H);
			for (int y = 0; y < SCREEN_H; y++) {
				for (int x = 0; x < SCREEN_W; x++) {
					const uint32_t p = fb[y * pitch + x];
					row[x * 3] = (uint8_t)p;
					row[x * 3 + 1] = (uint8_t)(p >> 8);
					row[x * 3 + 2] = (uint8_t)(p >> 16);
				}
				fwrite(row, 1, sizeof(row), f);
			}
			fclose(f);
			sceIoRemove(RBX_DATA_DIR "shot.ppm");
			sceIoRename(RBX_DATA_DIR "shot.tmp", RBX_DATA_DIR "shot.ppm");
			RBX_LOG("menu: shot.ppm written (screen)");
		}
	}
#else
	(void)c;
	(void)trigger;
#endif
}
#define shot_check(c) shot_check_as(c, RBX_DATA_DIR "shot")

// ================================ menu state ================================
static char s_set[32], s_title[96];
static UiCanvas *s_cv, *s_osd_cv, *s_fps_cv;
static int s_changed; // settings changed since the menu opened (saved on close only then)
enum { PAGE_MAIN, PAGE_STATES, PAGE_DISPLAY, PAGE_CONTROLS, PAGE_DIPS, PAGE_CHEATS, PAGE_ABOUT, PAGE_COMBO, PAGE_TURBO,
	PAGE_TOUCH, PAGE_TEDIT, PAGE_PORT, PAGE_STICK, PAGE_SETTINGS };
enum { SET_LANG, SET_BACK, SET_COUNT };   // (the Settings page, user 11-22)
static int s_page;
static int s_mt_wait_up; // the finger that left the touch layout editor: not a tap on the page it came back to
// rows in groups (a gap before M_STATES, M_DISPLAY and M_ABOUT): game, states, settings, about / leave.
// M_REFRESH (read the ROM folders again) only in the game list options
enum { M_RESUME, M_RESET, M_STATES, M_DISPLAY, M_CONTROLS, M_DIPS, M_CHEATS, M_GAME_SAVE, M_GAME_DEL, M_ABOUT, M_SELECT_GAME,
	M_EXIT, M_BACK, M_REFRESH, M_GROUP, M_SETTINGS, M_COUNT };
#define GROUP_GAP 10
// game list options (START in the list): no game. Only Display / Controls / Refresh / Back, global settings only
static int s_list_mode;
// main menu rows -> items (Delete only if per-game settings exist, else Save only. Back only in list options)
static int main_list(int *k)
{
	int n = 0;
	if (s_list_mode) {
		k[n++] = M_DISPLAY;
		k[n++] = M_CONTROLS;
		k[n++] = M_SETTINGS;
		k[n++] = M_GROUP;
		k[n++] = M_REFRESH;
		k[n++] = M_ABOUT;
		k[n++] = M_EXIT;   // (user 11-15: the app can be closed from the list's options too; no Back: START / the
		                   // back button close them)
		return n;
	}
	for (int m = 0; m < M_BACK; m++) {
		if (m == (rbx_settings_has_game() ? M_GAME_SAVE : M_GAME_DEL))
			continue;
		k[n++] = m;
		if (m == M_CONTROLS)
			k[n++] = M_SETTINGS;
	}
	return n;
}
static int main_rows(void) { int k[M_COUNT]; return main_list(k); }
static int main_kind(int row)
{
	int k[M_COUNT];
	const int n = main_list(k);
	return row >= 0 && row < n ? k[row] : M_RESUME;
}
enum { S_SLOT, S_SAVE, S_LOAD, S_BACK, S_COUNT };

#define D_NEON_N 7   // (user 11-22: the list's corridor knobs, in the list's options for now; the games' stay fixed)
enum { D_FPS, D_FRAMEBUF, D_ASPECT, D_ROTATE, D_FILTER, D_SCAN, D_SCAN_STR, D_BORDER, D_BORDER_COL, D_THEME, D_NEON,
	D_NEONGAP = D_NEON + D_NEON_N, D_BACK, D_COUNT };   // (D_NEONGAP: the list corridor's walls apart, user 11-22)
// the rows shown: Border color only with a border that shows it (Color, Marquee)
static int disp_list(int *k)
{
	int n = 0;
	for (int d = 0; d < D_COUNT; d++)
		if ((d != D_BORDER_COL || g_settings.border == RBX_BORDER_COLOR || g_settings.border == RBX_BORDER_MARQUEE) &&
			(d < D_NEON || d > D_NEONGAP || s_list_mode))
			k[n++] = d;
	return n;
}
static int disp_kind(int row)
{
	int k[D_COUNT];
	const int n = disp_list(k);
	return row >= 0 && row < n ? k[row] : D_BACK;
}
// Controls (RetroArch style): Port 1 Controls, Port 2 Controls, Touch controls, menu confirm button, defaults (every
// port), back.
// Port N Controls (PAGE_PORT, s_kp = the port): device, analog stick, Set all controls, the game buttons (as many as the
// game uses, 6 in list options), coin, start, Turbo, Combo keys, reset this port, back. A key row: left / right step
// through the keys, confirm waits for the button to press (key binding).
// Turbo page: turbo key, turbo button, turbo speed, back (the port's). Touch page: on, show, edit, clear, back
enum { CK_ACT, CK_TURBO_KEY, CK_TURBO_BTN, CK_TURBO_SPEED, CK_TURBO, CK_STICK, CK_CONFIRM, CK_COMBO, CK_DEFAULTS, CK_BACK,
	CK_TOUCH, CK_T_ON, CK_T_SHOW, CK_T_EDIT, CK_T_CLEAR, CK_PORT1, CK_PORT2, CK_DEVICE, CK_BIND_ALL, CK_RESET_PORT, CK_ENGINE,
	CK_SMAP_SET, CK_SMAP_DIR, CK_ACT2 };
// Analog sticks page: per stick its row and the four directions; act = the row (stick = act / STICK_ROWS)
#define STICK_ROWS (1 + RBX_SDIR_COUNT)
#define STICK_PAGE_ROWS (2 * STICK_ROWS + 1)
static int s_nbuttons = RBX_TURBO_BUTTONS;
static int ctrl_buttons(void) { return s_list_mode ? RBX_TURBO_BUTTONS : s_nbuttons; }
// the port whose page is open (0 = Port 1 = 1P, 1 = Port 2 = 2P): its settings
static int s_kp;
// the controls the page shows: the profile of the device the player reads now (they belong to the device)
static rbx_port *prt(void) { return &g_settings.port[re_pad_player_profile(s_kp)]; }
// key binding (the rows that assign a key): confirm on the row waits for the button to use. Left / right step through
// the keys as before (no wait)
static int s_bind;          // waiting for a button
static int s_bind_all;      // Set all controls: on to the next key row after each button, until Start
static int s_bind_ready;    // every button was released since the wait began (not the confirm press itself)
static uint64_t s_bind_t;   // when the wait began: 5 s without a button cancels
#define BIND_US 5000000
// game button rows (button 1..6) -> RBX_ACT_*
static int button_act(int i) { return i < 2 ? i : RBX_ACT_B3 + (i - 2); }
// Controls page rows
static const int s_ctrl_list[] = { CK_PORT1, CK_PORT2, CK_ENGINE, CK_TOUCH, CK_CONFIRM, CK_DEFAULTS, CK_BACK };
#define CTRL_ROWS ((int)(sizeof(s_ctrl_list) / sizeof(s_ctrl_list[0])))
// the 2P Controller row: there with a second device to play with (re_pad_devices), or added by hand (Add on the 1P row,
// its device None until one is picked); only a hand-added one without a controller can be deleted (Delete on its row)
static int p2_shown(void) { return re_pad_player_shown(1) || g_settings.p2_added; }
static int p2_deletable(void) { return g_settings.p2_added && !re_pad_player_shown(1); }
// Controls rows now: Port 2 only while it is shown (p2_shown). k may be NULL
static int ctrl_list(int *k)
{
	int n = 0;
	for (int j = 0; j < (int)(sizeof(s_ctrl_list) / sizeof(s_ctrl_list[0])); j++)
		if (s_ctrl_list[j] != CK_PORT2 || p2_shown()) { // (hidden while 2P's controller is away and not added)
			if (k)
				k[n] = s_ctrl_list[j];
			n++;
		}
	return n;
}
static int ctrl_rows(void) { return ctrl_list(NULL); }
// Port page: rows before the buttons, and after the buttons, coin and start
static const int s_port_head[] = { CK_DEVICE, CK_STICK, CK_BIND_ALL };
static const int s_port_tail[] = { CK_TURBO, CK_COMBO, CK_RESET_PORT, CK_BACK };
#define PORT_HEAD ((int)(sizeof(s_port_head) / sizeof(s_port_head[0])))
#define PORT_TAIL ((int)(sizeof(s_port_tail) / sizeof(s_port_tail[0])))
// the player reads no device (Device None, or its controller away with nothing free): its page has only Device and Back
// (the controls belong to a device: with none there is nothing to set)
static int port_none(void) { return re_pad_port(s_kp) < 0; }
// an action's rows: its key, then one row for each more key (CK_ACT2: act = the action + (which more key + 1) * 256)
#define ACT2_ACT(v) ((v) & 0xFF)
#define ACT2_X(v) (((v) >> 8) - 1)
#define KEY_PENDING (RBX_KEY_NONE + 1) // (a more key waiting for its button)
static int act_extras(int a)
{
	int n = 0;
	while (n < RBX_KEY_EXTRA && prt()->keyx[a][n] != RBX_KEY_NONE)
		n++;
	return n;
}
static void act_remove(int a, int x) // (the ones after it move up)
{
	for (; x + 1 < RBX_KEY_EXTRA; x++)
		prt()->keyx[a][x] = prt()->keyx[a][x + 1];
	prt()->keyx[a][RBX_KEY_EXTRA - 1] = RBX_KEY_NONE;
}
static int act_of(int j) // the actions on the page: the game's buttons, coin, start
{
	const int b = ctrl_buttons();
	return j < b ? button_act(j) : j == b ? RBX_ACT_COIN : RBX_ACT_START;
}
static int act_rows(void)
{
	int n = 0;
	for (int j = 0; j < ctrl_buttons() + 2; j++)
		n += 1 + act_extras(act_of(j));
	return n;
}
static int port_rows(void) { return port_none() ? 2 : PORT_HEAD + act_rows() + PORT_TAIL; }
static int port_kind(int i, int *act)
{
	*act = -1;
	if (port_none())
		return i == 0 ? CK_DEVICE : CK_BACK;
	if (i < PORT_HEAD)
		return s_port_head[i];
	i -= PORT_HEAD;
	for (int j = 0; j < ctrl_buttons() + 2; j++) {
		const int a = act_of(j), ne = act_extras(a);
		if (i == 0) {
			*act = a;
			return CK_ACT;
		}
		if (i <= ne) {
			*act = a | i << 8;
			return CK_ACT2;
		}
		i -= 1 + ne;
	}
	return i < PORT_TAIL ? s_port_tail[i] : CK_BACK;
}
// row of a key action on the Port page
static int port_row_of_act(int act)
{
	int r = PORT_HEAD;
	for (int j = 0; j < ctrl_buttons() + 2; j++) {
		if (act_of(j) == act)
			break;
		r += 1 + act_extras(act_of(j));
	}
	return r;
}
// row of a kind on the Controls page (coming back from a sub page) or the Port page
static int ctrl_row_of(int kind)
{
	int k[CTRL_ROWS];
	const int n = ctrl_list(k);
	for (int j = 0; j < n; j++)
		if (k[j] == kind)
			return j;
	return 0;
}
static int port_row_of(int kind)
{
	for (int j = 0; j < PORT_HEAD; j++)
		if (s_port_head[j] == kind)
			return j;
	for (int j = 0; j < PORT_TAIL; j++)
		if (s_port_tail[j] == kind)
			return PORT_HEAD + act_rows() + j;
	return 0;
}
// current row's kind: Controls, Port, Turbo or Touch
static int row_kind(int i, int *act)
{
	*act = -1;
	if (s_page == PAGE_TURBO)
		return i < 3 ? CK_TURBO_KEY + i : CK_BACK;
	if (s_page == PAGE_STICK) {
		*act = i;
		return i >= 2 * STICK_ROWS ? CK_BACK : i % STICK_ROWS ? CK_SMAP_DIR : CK_SMAP_SET;
	}
	if (s_page == PAGE_PORT)
		return port_kind(i, act);
	if (s_page == PAGE_TOUCH) {
		static const int k[5] = { CK_T_ON, CK_T_SHOW, CK_T_EDIT, CK_T_CLEAR, CK_BACK };
		return i >= 0 && i < 5 ? k[i] : CK_BACK;
	}
	int k[CTRL_ROWS];
	const int n = ctrl_list(k);
	return i >= 0 && i < n ? k[i] : CK_BACK;
}
#define CTRL_PAGE (s_page == PAGE_CONTROLS || s_page == PAGE_TURBO || s_page == PAGE_TOUCH || s_page == PAGE_PORT || \
	s_page == PAGE_STICK)
static const char *act_name(int act)
{
	static const char *const n[RBX_ACT_COUNT] = { "Button 1", "Button 2", "Coin", "Start", "Button 3", "Button 4", "Button 5",
		"Button 6" };
	return n[act];
}

// combo keys in screen order (Square, Triangle, Cross, Circle, L, R) and the choices for one key: None, then every pair
static const int s_combo_order[RBX_COMBO_SRC] = { RBX_KEY_SQUARE, RBX_KEY_TRIANGLE, RBX_KEY_CROSS, RBX_KEY_CIRCLE,
	RBX_KEY_L, RBX_KEY_R, RBX_KEY_L2, RBX_KEY_R2, RBX_KEY_L3, RBX_KEY_R3 };
#define COMBO_CHOICES (1 + RBX_COMBO_SRC * (RBX_COMBO_SRC - 1) / 2)
#define COMBO_ROWS (RBX_COMBO_SRC + 1)
static int combo_choice(int i) // choice index -> key mask
{
	if (i <= 0)
		return 0;
	for (int a = 0, n = 1; a < RBX_COMBO_SRC; a++)
		for (int b = a + 1; b < RBX_COMBO_SRC; b++, n++)
			if (n == i)
				return (1 << s_combo_order[a]) | (1 << s_combo_order[b]);
	return 0;
}
static int combo_index(int mask) // key mask -> choice index (0 if not a pair)
{
	for (int i = 1; i < COMBO_CHOICES; i++)
		if (combo_choice(i) == mask)
			return i;
	return 0;
}

// an action among the game's buttons on the page
static int act_is_button(int a)
{
	for (int i = 0; i < ctrl_buttons(); i++)
		if (button_act(i) == a)
			return 1;
	return 0;
}

// ---- duplicate keys: one Vita key that drives two different things in the running game (list options: the 6-button
// layout). func = what the key presses
enum { KF_TURBO = 16, KF_COMBO = 32 };
typedef struct { int key, func; const char *what; } KeyUse;
static int key_uses(KeyUse *u)
{
	int n = 0;
	for (int i = 0; i < ctrl_buttons(); i++) {
		const int a = button_act(i);
		if (prt()->keymap[a] < RBX_KEY_COUNT)
			u[n++] = (KeyUse){ prt()->keymap[a], a, act_name(a) };
	}
	u[n++] = (KeyUse){ prt()->keymap[RBX_ACT_COIN], RBX_ACT_COIN, "Coin" };
	u[n++] = (KeyUse){ prt()->keymap[RBX_ACT_START], RBX_ACT_START, "Start" };
	for (int a = 0; a < RBX_ACT_COUNT; a++)
		for (int x = 0; x < RBX_KEY_EXTRA; x++)
			if (prt()->keyx[a][x] < RBX_KEY_COUNT && (a == RBX_ACT_COIN || a == RBX_ACT_START || act_is_button(a)))
				u[n++] = (KeyUse){ prt()->keyx[a][x], a, act_name(a) };
	if (prt()->turbo_key < RBX_KEY_COUNT)
		u[n++] = (KeyUse){ prt()->turbo_key, KF_TURBO, "Turbo key" };
	for (int k = 0; k < RBX_KEY_COUNT; k++)
		if (prt()->combo[k])
			u[n++] = (KeyUse){ k, KF_COMBO + k, "a combo key" };
	return n;
}
// another use of key that presses something else, or NULL
static const char *key_clash(int key, int func)
{
	KeyUse u[64];
	const int n = key >= 0 && key < RBX_KEY_COUNT ? key_uses(u) : 0;
	for (int j = 0; j < n; j++)
		if (u[j].key == key && u[j].func != func)
			return u[j].what;
	return NULL;
}
// key and func of a Controls / Turbo row that assigns a key (0 = not such a row)
static int row_key(int kind, int act, int *key, int *func)
{
	switch (kind) {
	case CK_ACT: *key = prt()->keymap[act]; *func = act; return 1;
	case CK_ACT2: *key = prt()->keyx[ACT2_ACT(act)][ACT2_X(act)]; *func = ACT2_ACT(act); return 1;
	case CK_TURBO_KEY: *key = prt()->turbo_key; *func = KF_TURBO; return 1;
	default: return 0;
	}
}
// set the key of such a row
static void row_set_key(int kind, int act, int key)
{
	if (kind == CK_ACT)
		prt()->keymap[act] = key;
	else if (kind == CK_ACT2) {
		if (key == RBX_KEY_NONE)
			act_remove(ACT2_ACT(act), ACT2_X(act));
		else
			prt()->keyx[ACT2_ACT(act)][ACT2_X(act)] = key;
	} else if (kind == CK_TURBO_KEY)
		prt()->turbo_key = key;
}
// name of such a row
static const char *row_key_name(int kind, int act) { return kind == CK_ACT ? act_name(act) : kind == CK_ACT2 ? act_name(ACT2_ACT(act)) : "Turbo key"; }
// notice after a key change that made a duplicate (not while the choice window steps to a value)
static int s_quiet;
static void warn_clash(int key, int func)
{
	(void)key; // (a duplicate is shown on its row, "! duplicate": no notice, notices are for what matters)
	(void)func;
}

// DIP switch page: the groups, Restore defaults, Back. Cheat page: (nbbatman family) two built-in cheats, the user cheat
// file's cheats (cheats/<set>.ini), Back
static int dip_items(void) { return rbx_core_dip_count() + 2; }
enum { HK_STAGE, HK_HEALTH, HK_FILE, HK_BACK };
static int cheat_rows(void) { return (rbx_core_has_cheats() ? 2 : 0) + rbx_core_cheat_count() + 1; }
static int cheat_kind(int row, int *idx)
{
	const int b = rbx_core_has_cheats() ? 2 : 0;
	if (row < b)
		return row == 0 ? HK_STAGE : HK_HEALTH;
	row -= b;
	if (row < rbx_core_cheat_count()) {
		*idx = row;
		return HK_FILE;
	}
	return HK_BACK;
}
static int any_cheats(void) { return rbx_core_has_cheats() || rbx_core_cheat_count() > 0; }
static void save_dips(void)
{
	g_settings.dip_count = rbx_core_dip_raw(g_settings.dip, RBX_DIP_BYTES_MAX);
}

static int s_sel, s_main_sel, s_reset_armed, s_state_slot, s_about_scroll, s_dirty = 1;
// the footer line on the Controls pages: what the selected row does (NULL: the page's own footer)
static const char *row_help(int kind)
{
	switch (kind) {
	case CK_PORT1: return "1P Controller: its device, buttons, analog stick, turbo and combo keys";
	case CK_PORT2: return "2P Controller: its device, buttons, analog stick, turbo and combo keys";
	case CK_ENGINE: return "Native: the Vita's own controls. RePad: Bluetooth controllers through the RePad plugin that comes "
		"with the app (a controller as 2P). VitaControl: the VitaControl plugin installed on the Vita (1P only)";
	case CK_TOUCH: return "On-screen touch buttons: on or off, shown or not, their layout";
	case CK_CONFIRM: return "The button that confirms in the menus (the other one goes back)";
	case CK_DEFAULTS: return "Every player's buttons, analog stick, turbo and combo keys back to the defaults";
	case CK_DEVICE: return "The device this player reads. Auto: the Vita for 1P, then the controllers in order";
	case CK_STICK: return "What each direction of the left and right analog stick presses";
	case CK_SMAP_SET: return "The whole stick as the d-pad, or off. Or set each direction below";
	case CK_SMAP_DIR: return "What this direction presses, in addition to the buttons";
	case CK_BIND_ALL: return "Press each button in turn, from Button 1 to Start";
	case CK_ACT: return "Confirm, then press the button or tilt a stick. Left / Right: all keys, a whole stick too. Triangle or +: one more key";
	case CK_ACT2: return "One more key for this action. Confirm: press another. Left / Right: all keys";
	case CK_TURBO: return "A key that repeats a game button while it is held";
	case CK_COMBO: return "Keys that press two keys together";
	case CK_RESET_PORT: return "This player's buttons, analog stick, turbo and combo keys back to the defaults";
	case CK_TURBO_KEY: return "The key that repeats the turbo button while held (None: off). Confirm, then press the key. Square: None";
	case CK_TURBO_BTN: return "The game button the turbo key repeats";
	case CK_TURBO_SPEED: return "How fast the turbo button repeats";
	case CK_T_ON: return "The touch buttons press their keys";
	case CK_T_SHOW: return "The touch buttons are drawn over the game";
	case CK_T_EDIT: return "Add, move and resize touch buttons and give them keys";
	case CK_T_CLEAR: return "Remove every touch button (press twice)";
	default: return NULL;
	}
}
// the other pages' rows
static const char *page_row_help(void)
{
	switch (s_page) {
	case PAGE_MAIN:
		switch (main_kind(s_sel)) {
		case M_RESUME: return "Back to the game";
		case M_RESET: return "Start the game again from power-on";
		case M_STATES: return "Save the game at this point or load it again";
		case M_DISPLAY: return "Screen size, rotation, filter, scanlines, border, FPS and menu colors";
		case M_SETTINGS: return "The app's language and its other settings";
		case M_CONTROLS: return "Controllers, input engine, buttons, analog stick, turbo, combo keys and touch buttons";
		case M_DIPS: return "The board's DIP switches: difficulty, lives, coins and the game's other settings";
		case M_CHEATS: return "Start stage, infinite health and the cheats of cheats/<set>.ini";
		case M_GAME_SAVE: return "Keep the display and control settings as they are now for this game only";
		case M_GAME_DEL: return "This game uses the global settings again";
		case M_ABOUT: return "Version, credits and licenses";
		case M_SELECT_GAME: return "Back to the game list";
		case M_EXIT: return "Close the app";
		case M_BACK: return "Close the options";
		case M_REFRESH: return "Read the ROM folders again (ROMs copied while the app runs)";
		case M_GROUP: return "How the game list is grouped: machine, year, genre, publisher, or all the games";
		}
		return NULL;
	case PAGE_DISPLAY:
		switch (disp_kind(s_sel)) {
		case D_FPS: return "Frames per second in a corner of the game picture";
		case D_FRAMEBUF: return "1 frame: the game may run a frame ahead to even out slow frames (input about 17 ms later). Auto: on for the games that need it";
		case D_ASPECT: return "4:3, the full screen, or the largest whole-number scale";
		case D_ROTATE: return "Auto: vertical games upright. Vertical L / R: hold the Vita turned, the controls turn with it";
		case D_FILTER: return "Smooth or sharp pixels";
		case D_SCAN: return "Scanlines over the game picture, as on an arcade monitor";
		case D_SCAN_STR: return "How dark the scanlines are";
		case D_BORDER: return "Beside the game picture (4:3, Integer): black, a graphic, a color, or the marquee on a color";
		case D_BORDER_COL: return "The border color (Border: Color, Marquee)";
		case D_THEME: return "The colors of the menus";
		}
		if (disp_kind(s_sel) == D_NEONGAP)
			return "How far apart the list's neon walls are (the middle of the screen between them)";
		if (disp_kind(s_sel) >= D_NEON && disp_kind(s_sel) < D_BACK)
			return "The game list's neon background (for now: to be fixed once set)";
		return NULL;
	case PAGE_SETTINGS:
		return s_sel == SET_LANG ? "The language of the menus and messages" : NULL;
	case PAGE_STATES:
		switch (s_sel) {
		case S_SLOT: return "The slot to save to or load from";
		case S_SAVE: return "Save the game in this slot (in the game: SELECT+L)";
		case S_LOAD: return "Load this slot (in the game: SELECT+R)";
		}
		return NULL;
	case PAGE_COMBO:
		return s_sel < RBX_COMBO_SRC ? "This key presses the two keys picked here together (None: it is itself). Square: None" : NULL;
	case PAGE_DIPS:
		if (s_sel < rbx_core_dip_count())
			return "A DIP switch of the board, as on the cabinet. Some settings take effect after Reset game";
		return s_sel == dip_items() - 2 ? "Every DIP switch back to the game's defaults" : NULL;
	case PAGE_CHEATS: {
		int ci = 0;
		switch (cheat_kind(s_sel, &ci)) {
		case HK_STAGE: return "The stage a new game starts at (it applies when a new game starts)";
		case HK_HEALTH: return "1P never loses health";
		case HK_FILE: return "A cheat of cheats/<set>.ini: off, or one of its values";
		default: return NULL;
		}
	}
	default:
		return NULL;
	}
}
static int s_help_scroll;   // the footer help is longer than the line: it scrolls (the menu is drawn every frame)
static int s_help_key = -1; // page and row the help belongs to (scrolling starts over on another row)
static uint64_t s_help_t0;
// VitaControl keeps RePad out: the window that offers to switch it off and restart the Vita
// a restart the chosen input engine needs, asked when Back leaves Controls (0 = none):
// 1 RePad chosen, VitaControl keeps it out: switch VitaControl off and restart
// 2 VitaControl chosen, its plugin list line is commented out: switch it on and restart
// 3 VitaControl chosen and listed, but not running since the last restart: restart
static int s_vc_ask;
static int engine_restart_ask(void)
{
	if (g_settings.input_engine == RE_PAD_ENGINE_REPAD)
		return !re_pad_plugin() && re_pad_vitacontrol_listed() ? 1 : 0;
	if (g_settings.input_engine == RE_PAD_ENGINE_VITACONTROL) {
		const int st = re_pad_vitacontrol_state();
		if (st == RE_PAD_VC_OFF)
			return 2;
		if (st == RE_PAD_VC_ON && !re_pad_vitacontrol_running())
			return 3;
	}
	return 0;
}
static int s_drawn_h; // panel height currently drawn on the menu canvas (0 = other content)
static uint32_t s_prev_buttons;
static uint32_t s_bind_prev; // the bound player's buttons last frame
static void bind_begin(int all)
{
	s_bind = 1;
	s_bind_all = all;
	s_bind_ready = 0;
	s_bind_t = rbx_time_us();
	s_bind_prev = ~0u; // (pressed only after a release)
	s_dirty = 1;
}
// one more key for an action: its row appears and waits for the button (cancelled: the row goes)
static void key_add(int act)
{
	const int x = act_extras(act);
	if (x >= RBX_KEY_EXTRA)
		return;
	prt()->keyx[act][x] = KEY_PENDING;
	s_sel = port_row_of_act(act) + 1 + x;
	s_changed = 1;
	bind_begin(0);
}

// About: vertically scrolling text (empty string = blank line, starting with '#' = heading line)
static const char *const s_about[] = {
	"#ReBOX for Vita",
	"Version " RBX_VERSION " (" RBX_VERSION_DATE ")",
	"Developed by Reyanon",
	"  https://github.com/Reyanon",
	"",
	"'Re', an emulator project for the PS Vita",
	"Games (c) their makers. ROM images are not included.",
	"Not affiliated with or endorsed by any of the makers.",
	"Free software: GNU GPL (see LICENSE).",
	"",
	"#Open source used",
	"MAME - board drivers, video and sound chips, and the CPU cores",
	"  (ARM, ARM7, HuC6280, HD6309, i386, 8080, Konami, M37710, 6502,",
	"  6800, 6805, 6809, M740, MCS-48 / 51, NEC V20-V35, PIC16C5x,",
	"  SH-2, SH-3, TLCS-90, TMS320C1x, TMS34010, TMS57002, uPD7810,",
	"  V60, ADSP-2100)",
	"  https://github.com/mamedev/mame",
	"Musashi - 68000 core (MIT)",
	"  https://github.com/kstenerud/Musashi",
	"superzazu/z80 - Z80 core (MIT)",
	"  https://github.com/superzazu/z80",
	"ymfm - Yamaha FM sound chips (BSD-3-Clause)",
	"  https://github.com/aaronsgiles/ymfm",
	"catver.ini - game genres (MAME_SupportFiles, AntoPISA)",
	"  https://github.com/AntoPISA/MAME_SupportFiles",
	"RePad - Bluetooth controllers (MIT)",
	"  https://github.com/Reyanon/RePad",
	"VitaSDK - toolchain, newlib, pthread, system stubs",
	"  https://github.com/vitasdk",
	"libvita2d - graphics",
	"  https://github.com/xerpi/libvita2d",
	"Mbed TLS - update check (HTTPS)",
	"  https://github.com/Mbed-TLS/mbedtls",
	"libpng (used by libvita2d)",
	"  https://github.com/pnggroup/libpng",
	"libjpeg-turbo (used by libvita2d)",
	"  https://github.com/libjpeg-turbo/libjpeg-turbo",
	"zlib - ROM zip reading",
	"  https://github.com/madler/zlib",
	"newlib - C library",
	"  https://sourceware.org/newlib",
	"",
	"License texts: LICENSE and the third-party notices in the source repository.",
};
#define ABOUT_LINES ((int)(sizeof(s_about) / sizeof(s_about[0])))
#define ABOUT_VISIBLE 12

static void ensure_canvases(void)
{
	ui_init();
	if (!s_cv)
		s_cv = ui_canvas(0, 0, 0, SCREEN_W, SCREEN_H);
	if (!s_osd_cv)
		s_osd_cv = ui_canvas(2, 0, 0, SCREEN_W, 40);
	if (!s_fps_cv)
		s_fps_cv = ui_canvas(3, SCREEN_W - 4 - 240, 4, 240, 32); // top right (rbx_fps_tick aligns the box right)
}

void rbx_menu_init(void)
{
	ensure_canvases();
}

void rbx_menu_fini(void)
{
	ui_fini();
}

void rbx_menu_set_game(const char *set, const char *title, int buttons)
{
	s_nbuttons = buttons < 2 ? 2 : buttons > RBX_TURBO_BUTTONS ? RBX_TURBO_BUTTONS : buttons;
	for (int i = 0; i < RBX_PROFILES; i++)
		if (g_settings.port[i].turbo_btn > s_nbuttons)
			g_settings.port[i].turbo_btn = 1;
	snprintf(s_set, sizeof(s_set), "%s", set);
	snprintf(s_title, sizeof(s_title), "%s", title ? title : set);
	s_page = PAGE_MAIN;
	s_sel = s_main_sel = 0;
	rbx_menu_states_changed();
}

// save state slots of the running game: used or not, and when (read when the game starts and after a save, not when
// the menu opens: ten file stats held the menu up; the
// emulation is stopped then)
static int s_slot_used[RBX_STATE_SLOTS];
static SceDateTime s_slot_time[RBX_STATE_SLOTS];

void rbx_menu_states_changed(void)
{
	for (int k = 0; k < RBX_STATE_SLOTS; k++) {
		char path[128];
		SceIoStat st;
		rbx_state_path(k, path, sizeof(path));
		s_slot_used[k] = s_set[0] && sceIoGetstat(path, &st) >= 0 && st.st_size > 0;
		if (s_slot_used[k])
			s_slot_time[k] = st.st_mtime;
	}
	s_dirty = 1;
}

static int s_keep_place, s_pop, s_pop_sel; // menu closed with SELECT+START (keep its page); choice window open, its row
// The PS button belongs to the app everywhere (locked for the system from the start). One press opens the menu (in a
// game) or the options (in the game list) and closes them again like SELECT+START; nothing while loading. Only a
// double press goes to the home screen: the app lets go of the lock and the kernel helper presses PS for the system
#define RBX_KCTRL_LOADED 0x8002D013u // taiLoadStartKernelModule: already loaded (device)
static int s_kctrl = -1;    // kernel helper usable (module/rbx_kctrl.skprx): 1 yes, -1 no
static int s_cpu3_hooked;   // (rbx_kctrl_cpu3: the kernel helper's CPU3 unlock made by this run)
static int s_ps_locked;     // the system's PS button handling locked by the app
static unsigned s_ps_one, s_ps_two; // helper counts already acted on (single, double presses)
static uint64_t s_ps_home_t; // when a double press sent the app home (lock taken again a while later)
static void ps_lock(int on)
{
	if (s_kctrl < 0 || on == s_ps_locked)
		return;
	const int r = on ? sceShellUtilLock(SCE_SHELL_UTIL_LOCK_TYPE_PS_BTN_2) : sceShellUtilUnlock(SCE_SHELL_UTIL_LOCK_TYPE_PS_BTN_2);
	RBX_LOG("menu: PS button %s (%08x)", on ? "locked" : "unlocked", (unsigned)r);
	(void)r;
	s_ps_locked = on;
	if (on) { // (presses made while unlocked are not acted on)
		const unsigned st = reiKCtrlPsState();
		s_ps_one = st & 0x7FFF;
		s_ps_two = (st >> 15) & 0x7FFF;
	}
}
// before the app exits or relaunches itself: give the PS button back to the system (a relaunch with the shell lock
// still held sometimes left the new process unable to start, error C1-2569-2)
void rbx_ps_release(void)
{
	ps_lock(0);
	if (s_kctrl > 0 && s_cpu3_hooked) {   // (the CPU3 unlock is the system's: given back too)
		reiKCtrlCpu3(0, NULL);
		s_cpu3_hooked = 0;
	}
}
// CPU3 for the app's threads (the CoreUnlocker80000H plugin's patch made by the kernel helper when the check's code is
// the verified one; else, or with RBX_DATA_DIR "nocpu3", CPU3 stays closed and the threads plan for three cores; the
// unlock is given back when the thread test still finds CPU3 closed). hook 0 (CPU3 already open: a
// plugin or another app's unlock): nothing hooked, only this module's unlock left by a run that did not exit is taken
// over, to be given back at exit
void rbx_kctrl_cpu3(int hook)
{
	if (s_kctrl <= 0) {
		RBX_LOG("cpu3: unlock not tried (no kernel helper)");
		return;
	}
	SceIoStat st;
	const int off = hook && sceIoGetstat(RBX_DATA_DIR "nocpu3", &st) >= 0;
	if (off)
		hook = 0;
	unsigned int info[4] = { 0, 0, 0, 0 };
	const int r = reiKCtrlCpu3(hook ? 1 : -1, info);
	s_cpu3_hooked = info[3] == 1;
	RBX_LOG("cpu3: unlock %d (%s; SceKernelThreadMgr NID %08x, code %08x %08x)", r,
	        off ? "off: nocpu3" : !hook ? (s_cpu3_hooked ? "already open, ours taken over" : "already open, not ours")
	        : r == 1 ? "hooked" : r == 2 ? "code differs from the verified one" : r == 0 ? "not hooked" : "failed", info[0],
	        info[1], info[2]);
}
// the unlock made but CPU3 still closed (the thread test after it): given back at once
void rbx_kctrl_cpu3_undo(void)
{
	if (s_kctrl <= 0 || !s_cpu3_hooked)
		return;
	reiKCtrlCpu3(0, NULL);
	s_cpu3_hooked = 0;
	RBX_LOG("cpu3: unlock released (CPU3 still closed after it)");
}
// Load the kernel helper at start (module/rbx_kctrl.skprx in the VPK). Its syscall is linked into this app only if the module was loaded before the app
// started (a call right after a first load crashed on the device), so a first load since boot restarts the app once
// (argument "kctrl"); the module then stays loaded until the Vita reboots
void rbx_kctrl_init(int restarted)
{
	const SceUID m = taiLoadStartKernelModule("ux0:app/RBOX00001/module/rbx_kctrl.skprx", 0, NULL, 0); // (the kernel cannot open app0:)
	// (the module name has a version, rbx_kctrl_v2: a module of an older build may still be loaded until reboot)
	if (m >= 0 && !restarted) {
		static char *const av[] = { "kctrl", NULL };
		sceAppMgrLoadExec("app0:eboot.bin", av, NULL);
	}
	s_kctrl = (unsigned)m == RBX_KCTRL_LOADED ? 1 : -1;
	RBX_LOG("kctrl: %08x (%s)", (unsigned)m, s_kctrl > 0 ? "ready" : "not available");
	if (s_kctrl > 0) {
		sceShellUtilInitEvents(0);
		ps_lock(1);
	}
}
int rbx_kctrl_ready(void) { return s_kctrl > 0; } // (the update installs through it)
// call every frame anywhere: 1 once per single PS press, when PS is let go (PS + START takes a screenshot, the menu must
// not cover it: START while PS is held drops that press); a double press goes home
static int s_ps_pending; // a single press counted, PS still held
static int ps_poll(void)
{
	if (s_kctrl < 0)
		return 0;
	if (!s_ps_locked) {
		if (rbx_time_us() - s_ps_home_t > 1500000)
			ps_lock(1); // (back from the home screen)
		return 0;
	}
	const unsigned st = reiKCtrlPsState(), one = st & 0x7FFF, two = (st >> 15) & 0x7FFF;
	if (two != s_ps_two) {
		s_ps_two = two;
		s_ps_one = one;
		s_ps_pending = 0;
		ps_lock(0);
		reiKCtrlGoHome();
		s_ps_home_t = rbx_time_us();
		return 0;
	}
	if (one != s_ps_one) {
		s_ps_one = one;
		s_ps_pending = 1;
	}
	if (!s_ps_pending)
		return 0;
	if (st & (1u << 30)) { // still held: START with it is the system's screenshot
		SceCtrlData pad;
		if (sceCtrlPeekBufferPositive(0, &pad, 1) >= 0 && (pad.buttons & SCE_CTRL_START))
			s_ps_pending = 0;
		return 0;
	}
	s_ps_pending = 0;
	return 1;
}

int rbx_menu_should_open(uint32_t buttons)
{
	const int combo = (buttons & MENU_OPEN_COMBO) == MENU_OPEN_COMBO;
	const int prev_combo = (s_prev_buttons & MENU_OPEN_COMBO) == MENU_OPEN_COMBO;
	const int ps = ps_poll();
	const int open = (combo && !prev_combo) || ps;
	s_prev_buttons = buttons; // so the same press does not act right away inside the menu
	if (open) {
		s_mt_wait_up = 1; // (a finger down now, e.g. on a touch Menu button, is not a tap in the menu; none: cleared at once)
		if (!s_keep_place) { // (closed with SELECT+START or PS: opens where it was, sub pages too)
			s_page = PAGE_MAIN;
			s_sel = 0;
		}
		s_keep_place = 0;
		s_pop = 0;
		s_bind = 0;
		s_reset_armed = 0;
		s_dirty = 1;
		s_changed = 0;
	}
	return open;
}

int rbx_menu_state_slot(void)
{
	return s_state_slot;
}

static int page_items(void)
{
	switch (s_page) {
	case PAGE_DISPLAY: { int k[D_COUNT]; return disp_list(k); }
	case PAGE_CONTROLS: return ctrl_rows();
	case PAGE_TURBO: return 4;
	case PAGE_STICK: return STICK_PAGE_ROWS;
	case PAGE_TOUCH: return 5;
	case PAGE_PORT: return port_rows();
	case PAGE_TEDIT: return 1;
	case PAGE_COMBO: return COMBO_ROWS;
	case PAGE_DIPS: return dip_items();
	case PAGE_CHEATS: return cheat_rows();
	case PAGE_STATES: return S_COUNT;
	case PAGE_SETTINGS: return SET_COUNT;
	case PAGE_ABOUT: return ABOUT_VISIBLE;
	default: return main_rows();
	}
}

static void go_page(int page)
{
	if (s_page == PAGE_MAIN)
		s_main_sel = s_sel;
	s_page = page;
	s_sel = (page == PAGE_MAIN) ? s_main_sel : 0;
	if (page == PAGE_MAIN && s_sel >= main_rows())
		s_sel = 0;
	s_reset_armed = 0;
}
// back one level: Turbo and Combo keys return to their row in the Port page; a Port page and Touch to their row in
// Controls; the touch editor to the Touch page; other pages to the main page
static int s_about_direct, s_close_now;   // (About opened from the list's footer: closing it goes back to the list)
static void page_back(void)
{
	const int from = s_page;
	if (from == PAGE_ABOUT && s_about_direct) {
		s_close_now = 1;
		return;
	}
	if (from == PAGE_TEDIT) {
		go_page(PAGE_TOUCH);
		s_sel = 2; // (Edit touch layout)
		s_mt_wait_up = 1;
		return;
	}
	if (from == PAGE_TURBO || from == PAGE_COMBO || from == PAGE_STICK) {
		go_page(PAGE_PORT);
		s_sel = port_row_of(from == PAGE_TURBO ? CK_TURBO : from == PAGE_COMBO ? CK_COMBO : CK_STICK);
		return;
	}
	if (from == PAGE_TOUCH || from == PAGE_PORT) {
		go_page(PAGE_CONTROLS);
		s_sel = ctrl_row_of(from == PAGE_PORT ? (s_kp ? CK_PORT2 : CK_PORT1) : CK_TOUCH);
		return;
	}
	go_page(PAGE_MAIN);
}

#define REPEAT_KEYS (SCE_CTRL_UP | SCE_CTRL_DOWN | SCE_CTRL_LEFT | SCE_CTRL_RIGHT)
// held directions repeat after 300 ms, then every 60 ms. Timed by the clock, not by loop passes: a slow redraw made a
// pass-counted repeat late and slow
static uint32_t key_repeat(uint32_t held, uint32_t pressed, uint64_t *next)
{
	const uint32_t dir = held & REPEAT_KEYS;
	const uint64_t now = rbx_time_us();
	if (!dir || (pressed & REPEAT_KEYS)) {
		*next = now + 300000;
		return pressed;
	}
	if (now >= *next) {
		*next = now + 60000;
		pressed |= dir;
	}
	return pressed;
}

static int wrap(int v, int n)
{
	return (v % n + n) % n;
}

// value names (items and the choice window)
static const char *const aspect[] = { "4:3", "Full screen", "Integer 2x" };
static const char *const filter[] = { "Smooth", "Sharp" };
static const char *const rotate[] = { "Auto", "Off", "Vertical L", "Vertical R" };
static const char *const framebuf[] = { "Auto", "Off", "1 frame" };
// Frame buffer's value as shown: Auto with what it means for the running game
static const char *framebuf_text(void)
{
	if (g_settings.frame_buffer != RBX_FRAMEBUF_AUTO)
		return framebuf[g_settings.frame_buffer];
	return s_list_mode ? "Auto" : rbx_present_auto_text();
}
static const char *const scan[] = { "Off", "CRT soft", "CRT sharp", "Every 2 px", "Every 3 px", "Every 4 px" };
static const char *const scan_str[] = { "Light", "Medium", "Heavy" };
static const char *const border[] = { "Black", "Graphic", "Color", "Marquee" };
static const char *const border_col[] = { "Navy", "Gray", "Maroon", "Green", "Purple", "Brick", "Black", "Charcoal",
	"Silver", "White", "Blue", "Sky", "Teal", "Olive", "Forest", "Brown", "Orange", "Gold", "Red", "Wine", "Pink", "Indigo" };
static const char *const stick_dir[RBX_SDIR_COUNT] = { "Up", "Down", "Left", "Right" };
static const char *const stick_tgt[RBX_STGT_COUNT] = { "None", "Up", "Down", "Left", "Right", "Button 1", "Button 2", "Button 3",
	"Button 4", "Button 5", "Button 6", "Coin", "Start" };
// a stick's row: the whole stick as the d-pad, off, or directions set one by one
static const char *stick_state(int st)
{
	int dpad = 1, off = 1;
	for (int d = 0; d < RBX_SDIR_COUNT; d++) {
		dpad &= prt()->smap[st][d] == RBX_STGT_UP + d;
		off &= prt()->smap[st][d] == RBX_STGT_NONE;
	}
	return dpad ? "D-pad" : off ? "Off" : "Custom";
}
static const char *const turbo_speed[] = { "Fast (15/s)", "Medium (10/s)", "Slow (7.5/s)" };
static const char *const stages[RBX_STAGE_COUNT] = {
	"1 Seattle", "2 San Francisco", "3 Las Vegas", "4 Texas", "5 Florida", "6 Chicago", "Final New York",
};

// Apply display settings to the game video (in the game list there is no game video yet: rbx_video_init applies them)
static void apply_video(void)
{
	if (!s_list_mode)
		rbx_video_apply_settings();
}

static void change(int dir);
static const char *const s_group_name[RBX_GROUP_COUNT] = { "Machine", "Year", "Genre", "Publisher", "All" };
static void engine_pick(int engine);
static int engine_list(int *e);
// an item with several values opens a window listing them all on the confirm button. The values of the selected item:
// names, count (0 = not such an item), current index
#define MAX_CHOICES 24
enum { CH_TEXT, CH_KEYS, CH_COMBO }; // how the choice window draws the values: names, button icons, key pairs
// the combo page's window: -1 = its first step (None or the first key), else the first key picked (the second step)
static int s_combo_first = -1, s_pop_keep;
// that window's key at row i: first step None (-1) then the ten keys, second step the keys but the first
static int combo_pick(int i)
{
	if (s_combo_first < 0)
		return i <= 0 ? -1 : s_combo_order[i - 1];
	for (int k = 0, n = 0; k < RBX_COMBO_SRC; k++)
		if (s_combo_order[k] != s_combo_first && n++ == i)
			return s_combo_order[k];
	return -1;
}
static int s_choice_kind; // the last item_choices() listed (drawn that way)
static int item_choices(const char **out, int *cur, const char **title)
{
	static char num[MAX_CHOICES][24];
	int n = 0;
	*cur = 0;
	*title = "";
	s_choice_kind = CH_TEXT;
	if (s_page == PAGE_MAIN && main_kind(s_sel) == M_GROUP) {
		for (int i = 0; i < RBX_GROUP_COUNT; i++)
			out[i] = s_group_name[i];
		*cur = g_settings.list_group;
		*title = "Group games by";
		return RBX_GROUP_COUNT;
	}
	if (s_page == PAGE_DISPLAY) {
		const char *const *v = NULL;
		switch (disp_kind(s_sel)) {
		case D_ASPECT: v = aspect; n = RBX_ASPECT_COUNT; *cur = g_settings.aspect; *title = "Screen"; break;
		case D_ROTATE: v = rotate; n = RBX_ROT_COUNT; *cur = g_settings.rotate; *title = "Rotate"; break;
		case D_FRAMEBUF: v = framebuf; n = RBX_FRAMEBUF_COUNT; *cur = g_settings.frame_buffer; *title = "Frame buffer"; break;
		case D_FILTER: v = filter; n = RBX_FILTER_COUNT; *cur = g_settings.filter; *title = "Filter"; break;
		case D_SCAN: v = scan; n = RBX_SCAN_COUNT; *cur = g_settings.scan_type; *title = "Scanlines"; break;
		case D_SCAN_STR: v = scan_str; n = RBX_SCAN_STRENGTH_COUNT; *cur = g_settings.scan_strength; *title = "Scanline strength"; break;
		case D_BORDER: v = border; n = RBX_BORDER_COUNT; *cur = g_settings.border; *title = "Border"; break;
		case D_BORDER_COL: v = border_col; n = RBX_BCOL_COUNT; *cur = g_settings.border_color; *title = "Border color"; break;
		case D_THEME: v = s_theme_name; n = RBX_THEME_COUNT; *cur = g_settings.theme; *title = "Theme"; break;
		default: return 0;
		}
		for (int i = 0; i < n; i++)
			out[i] = v[i];
	} else if (CTRL_PAGE) {
		int act;
		const int k = row_kind(s_sel, &act);
		int key, func;
		if (row_key(k, act, &key, &func)) {
			// buttons 3-6 and the turbo key can also be None
			const int with_none = k == CK_TURBO_KEY || (k == CK_ACT && act >= RBX_ACT_B3);
			n = RBX_KEY_COUNT + with_none;
			for (int i = 0; i < n; i++)
				out[i] = rbx_key_name(i);
			*cur = key;
			s_choice_kind = CH_KEYS;
			*title = row_key_name(k, act);
		} else if (k == CK_ENGINE) {
			int e[RE_PAD_ENGINE_COUNT];
			n = engine_list(e);
			for (int i = 0; i < n; i++) {
				out[i] = re_pad_engine_name(e[i]);
				if (e[i] == g_settings.input_engine)
					*cur = i;
			}
			*title = "Input engine";
		} else if (k == CK_DEVICE) {
			// the sources there are now (connected controllers only), with their devices
			int src[RE_PAD_SRC_COUNT];
			n = re_pad_sources(s_kp, src, cur);
			for (int i = 0; i < n; i++)
				out[i] = re_pad_source_text(src[i], s_devtext[i], sizeof(s_devtext[i]));
			*title = "Device";
		} else if (k == CK_TURBO_BTN) {
			n = ctrl_buttons();
			for (int i = 0; i < n; i++) {
				snprintf(num[i], sizeof(num[i]), T("Button %d"), i + 1);
				out[i] = num[i];
			}
			*cur = prt()->turbo_btn - 1;
			*title = "Turbo button";
		} else if (k == CK_TURBO_SPEED) {
			n = RBX_TURBO_SPEED_COUNT;
			for (int i = 0; i < n; i++)
				out[i] = turbo_speed[i];
			*cur = prt()->turbo_speed;
			*title = "Turbo speed";
		} else if (k == CK_SMAP_DIR) {
			static char tb[32];
			const int st = act / STICK_ROWS, d = act % STICK_ROWS - 1;
			n = RBX_STGT_COUNT;
			for (int i = 0; i < n; i++)
				out[i] = stick_tgt[i];
			*cur = prt()->smap[st][d];
			snprintf(tb, sizeof(tb), T("%s stick %s"), st ? "Right" : "Left", stick_dir[d]);
			*title = tb;
		}
	} else if (s_page == PAGE_STATES && s_sel == S_SLOT) {
		n = RBX_STATE_SLOTS < MAX_CHOICES ? RBX_STATE_SLOTS : MAX_CHOICES;
		for (int i = 0; i < n; i++) {
			snprintf(num[i], sizeof(num[i]), T("Slot %d%s"), i, s_slot_used[i] ? "" : "  (empty)");
			out[i] = num[i];
		}
		*cur = s_state_slot;
		*title = "Save slot";
	} else if (s_page == PAGE_COMBO && s_sel < RBX_COMBO_SRC) {
		// two steps (the pairs are too many for one list): None or the first key, then the second key
		const int v = prt()->combo[s_combo_order[s_sel]];
		n = s_combo_first < 0 ? 1 + RBX_COMBO_SRC : RBX_COMBO_SRC - 1;
		*cur = 0;
		for (int i = 0, found = 0; i < n; i++) {
			const int key = combo_pick(i);
			out[i] = key < 0 ? "None" : rbx_key_name(key);
			if (key >= 0 && (v & (1 << key)) && !found && (s_combo_first < 0 || (v & (1 << s_combo_first)))) {
				*cur = i;
				found = 1;
			}
		}
		s_choice_kind = CH_COMBO;
		*title = "Combo for";
	} else if (s_page == PAGE_DIPS && s_sel < rbx_core_dip_count()) {
		n = rbx_core_dip_options(s_sel);
		if (n > MAX_CHOICES)
			n = MAX_CHOICES;
		for (int i = 0; i < n; i++)
			out[i] = rbx_core_dip_option_name(s_sel, i);
		*cur = rbx_core_dip_get(s_sel) < 0 ? 0 : rbx_core_dip_get(s_sel);
		*title = rbx_core_dip_name(s_sel);
	} else if (s_page == PAGE_CHEATS) {
		int ci = 0;
		const int k = cheat_kind(s_sel, &ci);
		if (k == HK_STAGE) {
			n = RBX_STAGE_COUNT;
			for (int i = 0; i < n; i++)
				out[i] = stages[i];
			*cur = g_settings.start_stage;
			*title = "Start stage";
		} else if (k == HK_FILE) {
			n = rbx_core_cheat_options(ci);
			if (n > MAX_CHOICES)
				n = MAX_CHOICES;
			for (int i = 0; i < n; i++)
				out[i] = rbx_core_cheat_option_name(ci, i);
			*cur = rbx_core_cheat_get(ci);
			*title = rbx_core_cheat_name(ci);
		}
	}
	return n >= 3 ? n : 0; // two values: the button just flips them
}
static void set_choice(int idx)
{
	const char *names[MAX_CHOICES], *title;
	int cur, n = item_choices(names, &cur, &title);
	int eact;
	if (s_page == PAGE_COMBO && s_sel < RBX_COMBO_SRC) { // the combo window's two steps
		const int key = combo_pick(idx), src = s_combo_order[s_sel];
		if (s_combo_first < 0 && key >= 0) {
			s_combo_first = key; // on to the second key: the window stays
			s_pop_keep = 1;
			return;
		}
		prt()->combo[src] = key < 0 || s_combo_first < 0 ? 0 : (1 << s_combo_first) | (1 << key);
		s_combo_first = -1;
		return;
	}
	if (CTRL_PAGE && row_kind(s_sel, &eact) == CK_DEVICE) { // the source picked, set even when it is the one shown
		g_settings.pad_src[s_kp] = re_pad_pick_source(s_kp, idx);
		return;
	}
	if (CTRL_PAGE && row_kind(s_sel, &eact) == CK_ENGINE) {
		// the engine directly (stepping would pass RePad on the way and load it)
		int e[RE_PAD_ENGINE_COUNT];
		const int ne = engine_list(e);
		if (idx >= 0 && idx < ne && e[idx] != g_settings.input_engine)
			engine_pick(e[idx]);
		return;
	}
	s_quiet = 1; // (the steps between pass other values: warn once at the end)
	for (int guard = 0; n && cur != idx && guard < MAX_CHOICES + 2; guard++) {
		change(+1); // the same steps as the right button (wraps, applies)
		n = item_choices(names, &cur, &title);
	}
	s_quiet = 0;
	int act, key, func;
	if (CTRL_PAGE && row_key(row_kind(s_sel, &act), act, &key, &func))
		warn_clash(key, func);
	else if (s_page == PAGE_COMBO && s_sel < RBX_COMBO_SRC && prt()->combo[s_combo_order[s_sel]])
		warn_clash(s_combo_order[s_sel], KF_COMBO + s_combo_order[s_sel]);
}

// the input engine picked (left / right or its choice window): at once, either way. VitaControl keeping RePad out is
// asked about when Back leaves Controls (switch it off and restart, or back to Native)
static void engine_pick(int engine)
{
	g_settings.input_engine = engine;
	re_pad_engine_set(engine);
	if (engine == RE_PAD_ENGINE_REPAD && !re_pad_plugin() && !re_pad_vitacontrol_listed())
		rbx_osd_show("RePad could not start: another controller plugin is running");
	s_changed = 1;
}
// the engines offered: Native, RePad, VitaControl when a plugin list names it
static int engine_list(int *e)
{
	int n = 0;
	e[n++] = RE_PAD_ENGINE_NATIVE;
	e[n++] = RE_PAD_ENGINE_REPAD;
	if (re_pad_vitacontrol_state() != RE_PAD_VC_NONE)
		e[n++] = RE_PAD_ENGINE_VITACONTROL;
	return n;
}

static void change(int dir)
{
	switch (s_page) {
	case PAGE_MAIN:
		if (main_kind(s_sel) == M_GROUP) {
			g_settings.list_group = wrap(g_settings.list_group + dir, RBX_GROUP_COUNT);
			s_changed = 1;
		}
		break;
	case PAGE_DISPLAY:
		switch (disp_kind(s_sel)) {
		case D_FPS: g_settings.show_fps = !g_settings.show_fps; break;
		case D_ASPECT: g_settings.aspect = wrap(g_settings.aspect + dir, RBX_ASPECT_COUNT); break;
		case D_ROTATE: g_settings.rotate = wrap(g_settings.rotate + dir, RBX_ROT_COUNT); break;
		case D_FRAMEBUF: g_settings.frame_buffer = wrap(g_settings.frame_buffer + dir, RBX_FRAMEBUF_COUNT); break;
		case D_FILTER: g_settings.filter = wrap(g_settings.filter + dir, RBX_FILTER_COUNT); break;
		case D_SCAN: g_settings.scan_type = wrap(g_settings.scan_type + dir, RBX_SCAN_COUNT); break;
		case D_SCAN_STR: g_settings.scan_strength = wrap(g_settings.scan_strength + dir, RBX_SCAN_STRENGTH_COUNT); break;
		case D_BORDER: {   // (Border color comes and goes below it: the selection stays on a row)
			g_settings.border = wrap(g_settings.border + dir, RBX_BORDER_COUNT);
			int k[D_COUNT];
			const int n = disp_list(k);
			if (s_sel >= n)
				s_sel = n - 1;
			break;
		}
		case D_BORDER_COL: g_settings.border_color = wrap(g_settings.border_color + dir, RBX_BCOL_COUNT); break;
		case D_THEME:
			g_settings.theme = wrap(g_settings.theme + dir, RBX_THEME_COUNT);
			s_drawn_h = 0; // (every colour changes: the whole screen again)
			break;
		default: {
			const int dk = disp_kind(s_sel);
			if (dk == D_NEONGAP) {
				g_settings.neon_list_gap += dir * 20;
				g_settings.neon_list_gap = g_settings.neon_list_gap > 520 ? 40 : g_settings.neon_list_gap < 40 ? 520 : g_settings.neon_list_gap;
				break;
			}
			if (dk < D_NEON || dk >= D_BACK)
				return;
			const rbx_neon_param *p = &rbx_neon_params[dk - D_NEON];
			int *v = &g_settings.neon_list[dk - D_NEON];
			*v += dir * p->step;
			*v = *v > p->max ? p->min : *v < p->min ? p->max : *v;
			break;
		}
		}
		apply_video();
		s_changed = 1;
		break;
	case PAGE_CONTROLS:
	case PAGE_TURBO:
	case PAGE_STICK:
	case PAGE_TOUCH:
	case PAGE_PORT: {
		int act;
		const int k = row_kind(s_sel, &act);
		if (k == CK_ACT && act >= 0) // buttons 3-6 can also be None
			prt()->keymap[act] = wrap(prt()->keymap[act] + dir, act >= RBX_ACT_B3 ? RBX_KEY_COUNT + 1 : RBX_KEY_COUNT);
		else if (k == CK_ACT2) {
			int *v = &prt()->keyx[ACT2_ACT(act)][ACT2_X(act)];
			*v = wrap(*v + dir, RBX_KEY_COUNT);
		}
		else if (k == CK_ENGINE)
			engine_pick(re_pad_engine_step(g_settings.input_engine, dir)); // (VitaControl only when a list names it)
		else if (k == CK_DEVICE) // (through the sources there are now)
			g_settings.pad_src[s_kp] = re_pad_step_source(s_kp, dir);
		else if (k == CK_PORT1 || k == CK_PORT2) // (the player's device, as its Device row)
			g_settings.pad_src[k == CK_PORT2] = re_pad_step_source(k == CK_PORT2, dir);
		else if (k == CK_TURBO_KEY)
			prt()->turbo_key = wrap(prt()->turbo_key + dir, RBX_KEY_COUNT + 1); // + None (off)
		else if (k == CK_TURBO_BTN) // 1..number of game buttons
			prt()->turbo_btn = wrap(prt()->turbo_btn - 1 + dir, ctrl_buttons()) + 1;
		else if (k == CK_TURBO_SPEED)
			prt()->turbo_speed = wrap(prt()->turbo_speed + dir, RBX_TURBO_SPEED_COUNT);
		else if (k == CK_SMAP_SET)
			rbx_stick_preset(prt(), act / STICK_ROWS, strcmp(stick_state(act / STICK_ROWS), "D-pad"));
		else if (k == CK_SMAP_DIR) {
			int *v = &prt()->smap[act / STICK_ROWS][act % STICK_ROWS - 1];
			*v = wrap(*v + dir, RBX_STGT_COUNT);
		}
		else if (k == CK_T_ON)
			g_settings.touch_keys = !g_settings.touch_keys;
		else if (k == CK_T_SHOW)
			g_settings.touch_show = !g_settings.touch_show;
		else if (k == CK_CONFIRM)
			g_settings.menu_confirm = wrap(g_settings.menu_confirm + dir, RBX_CONFIRM_COUNT);
		else
			return;
		s_changed = 1;
		int key, func;
		if (row_key(k, act, &key, &func))
			warn_clash(key, func);
		break;
	}
	case PAGE_STATES:
		if (s_sel == S_SLOT)
			s_state_slot = wrap(s_state_slot + dir, RBX_STATE_SLOTS);
		break;
	case PAGE_SETTINGS:
		if (s_sel == SET_LANG && rbx_lang_count() > 0) {   // (the next language: every text drawn again in it)
			int i = rbx_lang_index(g_settings.language);
			i = wrap((i < 0 ? 0 : i) + dir, rbx_lang_count());
			snprintf(g_settings.language, sizeof(g_settings.language), "%s", rbx_lang_at(i)->code);
			rbx_lang_use(g_settings.language);
			s_theme_gen++;   // (the rows kept as textures: made again)
			s_drawn_h = 0;
			s_changed = 1;
		}
		break;
	case PAGE_COMBO: {
		if (s_sel >= RBX_COMBO_SRC)
			return;
		int *v = &prt()->combo[s_combo_order[s_sel]];
		*v = combo_choice(wrap(combo_index(*v) + dir, COMBO_CHOICES));
		s_changed = 1;
		if (*v)
			warn_clash(s_combo_order[s_sel], KF_COMBO + s_combo_order[s_sel]);
		break;
	}
	case PAGE_DIPS:
		if (s_sel < rbx_core_dip_count()) {
			const int n = rbx_core_dip_options(s_sel), cur = rbx_core_dip_get(s_sel);
			if (n > 0) {
				rbx_core_dip_set(s_sel, wrap((cur < 0 ? 0 : cur) + dir, n));
				save_dips();
				s_changed = 1;
			}
		}
		break;
	case PAGE_CHEATS: {
		int ci = 0;
		const int k = cheat_kind(s_sel, &ci);
		if (k == HK_STAGE) {
			g_settings.start_stage = wrap(g_settings.start_stage + dir, RBX_STAGE_COUNT);
			rbx_core_set_start_stage(g_settings.start_stage);
		} else if (k == HK_HEALTH) {
			g_settings.cheat_health = !g_settings.cheat_health;
			rbx_core_set_infinite_health(g_settings.cheat_health);
		} else if (k == HK_FILE && rbx_core_cheat_options(ci) > 0)
			rbx_core_cheat_set(ci, wrap(rbx_core_cheat_get(ci) + dir, rbx_core_cheat_options(ci)));
		break;
	}
	}
}

// ---- touch layout editor (Controls > Touch controls > Edit touch layout) ----
// Drag on an empty spot: a new button (then its key). Tap a button: its key, or Delete. Drag a button: move it; drag its
// lower right corner: its size. Square adds one in the middle, Triangle opens the selected one's keys (no touch needed).
// START, the back button or Done leave (the layout is saved then)
#define TE_MIN 42    // smallest button side (sizes go in TE_STEP steps)
#define TE_STEP 6
#define TE_GRAB 22   // an edge or corner is grabbed this far from it, inside or outside the button
#define TE_POP_N (RBX_TKEY_COUNT + 3) // the keys, Combo >, Duplicate, then Delete
#define TE_ROW_COMBO RBX_TKEY_COUNT
#define TE_ROW_DUP (RBX_TKEY_COUNT + 1)
#define TE_ROW_DEL (RBX_TKEY_COUNT + 2)
#define TE_CPOP_N (te_cpop == 2 ? RBX_COMBO_SRC - 1 : RBX_COMBO_SRC) // a pair: its first key, then its second (e.g. L, R)
#define TE_CPOP_RH 26
#define TE_POP_RH 28 // (with Duplicate and Menu the window still fits the screen: 16 rows)
enum { TE_IDLE, TE_NEW, TE_MOVE, TE_SIZE };
enum { TE_L = 1, TE_R = 2, TE_T = 4, TE_B = 8 }; // edges being dragged (TE_SIZE)
static int te_sel = -1, te_mode, te_prev_n, te_sx, te_sy, te_ox, te_oy, te_ow, te_oh, te_moved, te_pop = -1, te_pop_sel;
static int te_cpop, te_cpop_sel, te_cfirst; // the pair lists (from the key window's Combo row): 1 its first key, 2 its second
static int te_ckey(int i) // the pair list's key at row i (the second list leaves the first key out)
{
	for (int k = 0, n = 0; k < RBX_COMBO_SRC; k++)
		if ((te_cpop == 1 || s_combo_order[k] != te_cfirst) && n++ == i)
			return s_combo_order[k];
	return -1;
}
static int te_edges;
static int te_snap(int v) // a size in TE_STEP steps, at least TE_MIN
{
	v = (v + TE_STEP / 2) / TE_STEP * TE_STEP;
	return v < TE_MIN ? TE_MIN : v;
}
// the edges of button a a touch grabs (0: none). Within TE_GRAB of an edge, outside too; inside a small button at
// most a quarter of its side, so its middle still moves it
static int te_edges_at(const ReiTouchArea *a, int x, int y)
{
	const int gx = a->w / 4 < TE_GRAB ? a->w / 4 : TE_GRAB, gy = a->h / 4 < TE_GRAB ? a->h / 4 : TE_GRAB;
	if (x < a->x - TE_GRAB || x >= a->x + a->w + TE_GRAB || y < a->y - TE_GRAB || y >= a->y + a->h + TE_GRAB)
		return 0;
	int e = 0;
	if (x < a->x + gx)
		e |= TE_L;
	else if (x >= a->x + a->w - gx)
		e |= TE_R;
	if (y < a->y + gy)
		e |= TE_T;
	else if (y >= a->y + a->h - gy)
		e |= TE_B;
	return e;
}
static int te_nx, te_ny, te_nw, te_nh; // the button being drawn (TE_NEW)
static void tedit_open(void)
{
	te_sel = -1;
	te_mode = TE_IDLE;
	te_prev_n = 1; // (a finger already down does nothing until it is lifted)
	te_pop = -1;
	s_drawn_h = 0;
	s_dirty = 1;
}
static void te_pop_rect(int *x, int *y, int *w, int *h)
{
	*w = 320;
	*h = 64 + (te_cpop ? TE_CPOP_N * TE_CPOP_RH : TE_POP_N * TE_POP_RH) + 12;
	*x = (SCREEN_W - *w) / 2;
	*y = (SCREEN_H - *h) / 2;
}
static int te_done_hit(int x, int y) { return x >= SCREEN_W - 130 && y < 46; }
static void te_pop_open(int i)
{
	const ReiTouchArea *a = rbx_touch_area(i);
	if (!a)
		return;
	te_pop = i;
	te_cpop = 0;
	te_pop_sel = RBX_TKEY_IS_COMBO(a->key) ? TE_ROW_COMBO : a->key;
}
static void te_pop_apply(int row)
{
	ReiTouchArea *a = rbx_touch_area(te_pop);
	if (te_cpop) { // a key pair: the first key, then the second
		const int key = row >= 0 && row < TE_CPOP_N ? te_ckey(row) : -1;
		if (te_cpop == 1 && key >= 0) {
			te_cfirst = key;
			te_cpop = 2;
			te_cpop_sel = 0;
			return;
		}
		if (a && key >= 0) {
			a->key = (int16_t)(RBX_TKEY_COMBO + (1 << te_cfirst) + (1 << key));
			rbx_touch_changed();
		}
		te_cpop = 0;
		te_pop = -1;
		return;
	}
	if (row == TE_ROW_COMBO) { // the pair lists (entered from the key window)
		te_cpop = 1;
		te_cpop_sel = 0;
		for (int i = RBX_COMBO_SRC - 1; i >= 0; i--) // (on the button's first key, if it has a pair)
			if (a && RBX_TKEY_IS_COMBO(a->key) && ((a->key - RBX_TKEY_COMBO) & (1 << s_combo_order[i])))
				te_cpop_sel = i;
		return;
	}
	if (row == TE_ROW_DUP) { // a copy beside it (same size and key), then the copy's key
		if (a) {
			const int w = a->w, h = a->h, key = a->key;
			int nx = a->x + 24, ny = a->y + 24;
			if (nx + w > SCREEN_W)
				nx = a->x >= 24 ? a->x - 24 : 0;
			if (ny + h > SCREEN_H)
				ny = a->y >= 24 ? a->y - 24 : 0;
			const int i = rbx_touch_add(nx, ny, w, h, key);
			if (i >= 0) {
				te_sel = i;
				te_pop_open(i);
				return;
			}
		}
		te_pop = -1;
		return;
	}
	if (row == TE_ROW_DEL) { // Delete
		rbx_touch_remove(te_pop);
		te_sel = -1;
	} else if (a && row >= 0 && row < RBX_TKEY_COUNT) {
		a->key = (int8_t)row;
		rbx_touch_changed();
	}
	te_pop = -1;
}
static void tedit_leave(void)
{
	rbx_touch_save();
	te_pop = -1;
	page_back();
	s_drawn_h = 0;
	s_dirty = 1;
}
static void tedit_update(uint32_t pressed)
{
	const int o_ok = g_settings.menu_confirm == RBX_CONFIRM_CIRCLE;
	const uint32_t ok = o_ok ? SCE_CTRL_CIRCLE : SCE_CTRL_CROSS, back = o_ok ? SCE_CTRL_CROSS : SCE_CTRL_CIRCLE;
	int x[6], y[6];
	const int n = rbx_touch_read(x, y);
	const int down = n > 0 && te_prev_n == 0, held = n > 0, up = n == 0 && te_prev_n > 0;
	te_prev_n = n;
	if (te_pop >= 0) { // key window: touch a row, or up/down and confirm; back or a touch outside closes it
		int *sel = te_cpop ? &te_cpop_sel : &te_pop_sel;
		const int rows = te_cpop ? TE_CPOP_N : TE_POP_N, rh = te_cpop ? TE_CPOP_RH : TE_POP_RH;
		if (pressed & SCE_CTRL_UP) *sel = wrap(*sel - 1, rows);
		if (pressed & SCE_CTRL_DOWN) *sel = wrap(*sel + 1, rows);
		if (pressed & ok)
			te_pop_apply(*sel);
		else if (pressed & (back | SCE_CTRL_START)) {
			if (te_cpop)
				te_cpop--; // (back to the first key, or to the key window)
			else
				te_pop = -1;
		} else if (down) {
			int wx, wy, ww, wh;
			te_pop_rect(&wx, &wy, &ww, &wh);
			const int row = (y[0] - wy - 60) / rh;
			if (x[0] >= wx && x[0] < wx + ww && y[0] >= wy + 60 && row >= 0 && row < rows)
				te_pop_apply(row);
			else if (!(x[0] >= wx && x[0] < wx + ww && y[0] >= wy && y[0] < wy + wh)) {
				te_cpop = 0;
				te_pop = -1;
			}
		}
		if (pressed || down)
			s_dirty = 1;
		return;
	}
	if (pressed & (back | SCE_CTRL_START)) {
		tedit_leave();
		return;
	}
	if (pressed & SCE_CTRL_SQUARE) { // a new button in the middle, then its key
		const int i = rbx_touch_add(SCREEN_W / 2 - 50, SCREEN_H / 2 - 50, 100, 100, RBX_KEY_CROSS);
		if (i >= 0) {
			te_sel = i;
			te_pop_open(i);
		}
	}
	if ((pressed & SCE_CTRL_TRIANGLE) && te_sel >= 0)
		te_pop_open(te_sel);
	if (down) {
		if (te_done_hit(x[0], y[0])) {
			tedit_leave();
			return;
		}
		te_sx = x[0];
		te_sy = y[0];
		te_moved = 0;
		int hit = -1, edges = 0;
		// an edge or corner of the selected button first (its grab zone reaches outside it), then any button on top
		if (te_sel >= 0 && rbx_touch_area(te_sel)) {
			edges = te_edges_at(rbx_touch_area(te_sel), x[0], y[0]);
			if (edges)
				hit = te_sel;
		}
		for (int i = rbx_touch_count() - 1; i >= 0 && hit < 0; i--) { // (the last drawn is on top)
			const ReiTouchArea *a = rbx_touch_area(i);
			if (x[0] >= a->x && x[0] < a->x + a->w && y[0] >= a->y && y[0] < a->y + a->h) {
				hit = i;
				edges = te_edges_at(a, x[0], y[0]);
			}
		}
		if (hit >= 0) {
			const ReiTouchArea *a = rbx_touch_area(hit);
			te_sel = hit;
			te_edges = edges;
			te_mode = edges ? TE_SIZE : TE_MOVE;
			te_ox = a->x;
			te_oy = a->y;
			te_ow = a->w;
			te_oh = a->h;
		} else {
			te_mode = TE_NEW;
			te_sel = -1;
			te_nx = x[0];
			te_ny = y[0];
			te_nw = te_nh = 0;
		}
		s_dirty = 1;
	} else if (held && te_mode != TE_IDLE) {
		const int dx = x[0] - te_sx, dy = y[0] - te_sy;
		if (dx > 8 || dx < -8 || dy > 8 || dy < -8)
			te_moved = 1;
		ReiTouchArea *a = rbx_touch_area(te_sel);
		if (te_mode == TE_NEW) { // (from the first touch, in TE_STEP steps)
			te_nw = (dx < 0 ? -dx : dx) / TE_STEP * TE_STEP;
			te_nh = (dy < 0 ? -dy : dy) / TE_STEP * TE_STEP;
			te_nx = dx < 0 ? te_sx - te_nw : te_sx;
			te_ny = dy < 0 ? te_sy - te_nh : te_sy;
		} else if (a && te_moved && te_mode == TE_MOVE) { // (in TE_STEP steps too, as the size; as ReST-V)
			const int nx = te_ox + dx / TE_STEP * TE_STEP, ny = te_oy + dy / TE_STEP * TE_STEP;
			a->x = (int16_t)(nx < 0 ? 0 : nx > SCREEN_W - a->w ? SCREEN_W - a->w : nx);
			a->y = (int16_t)(ny < 0 ? 0 : ny > SCREEN_H - a->h ? SCREEN_H - a->h : ny);
			rbx_touch_changed();
		} else if (a && te_mode == TE_SIZE) { // the grabbed edges follow the finger, the others stay; TE_STEP steps
			int x0 = te_ox, x1 = te_ox + te_ow, y0 = te_oy, y1 = te_oy + te_oh;
			if (te_edges & TE_L) {
				const int lim = x0 + dx < 0 ? -x0 : dx;
				x0 = x1 - te_snap(te_ow - lim);
				if (x0 < 0)
					x0 = x1 - (x1 / TE_STEP * TE_STEP < TE_MIN ? TE_MIN : x1 / TE_STEP * TE_STEP);
			} else if (te_edges & TE_R) {
				x1 = x0 + te_snap(te_ow + dx);
				if (x1 > SCREEN_W)
					x1 = x0 + (SCREEN_W - x0) / TE_STEP * TE_STEP;
			}
			if (te_edges & TE_T) {
				const int lim = y0 + dy < 0 ? -y0 : dy;
				y0 = y1 - te_snap(te_oh - lim);
				if (y0 < 0)
					y0 = y1 - (y1 / TE_STEP * TE_STEP < TE_MIN ? TE_MIN : y1 / TE_STEP * TE_STEP);
			} else if (te_edges & TE_B) {
				y1 = y0 + te_snap(te_oh + dy);
				if (y1 > SCREEN_H)
					y1 = y0 + (SCREEN_H - y0) / TE_STEP * TE_STEP;
			}
			a->x = (int16_t)x0;
			a->y = (int16_t)y0;
			a->w = (int16_t)(x1 - x0);
			a->h = (int16_t)(y1 - y0);
			rbx_touch_changed();
		}
		s_dirty = 1;
	} else if (up) {
		if (te_mode == TE_NEW && te_nw >= TE_MIN && te_nh >= TE_MIN) {
			const int i = rbx_touch_add(te_nx, te_ny, te_nw, te_nh, RBX_KEY_CROSS);
			if (i >= 0) {
				te_sel = i;
				te_pop_open(i);
			}
		} else if (te_mode == TE_MOVE && !te_moved)
			te_pop_open(te_sel); // a tap: its key
		te_mode = TE_IDLE;
		s_dirty = 1;
	}
}

// Back out of a page (the back button or the Back row): leaving Controls itself, first the restart the chosen input
// engine needs, if any (its window: restart, or back to Native)
static void back_out(void)
{
	if (s_page == PAGE_CONTROLS && (s_vc_ask = engine_restart_ask()) != 0)
		s_drawn_h = 0;
	else
		page_back();
}

static rbx_menu_result confirm(void)
{
	switch (s_page) {
	case PAGE_MAIN:
		switch (main_kind(s_sel)) {
		case M_RESUME: return RBX_MENU_CLOSE;
		case M_RESET:
			if (!s_reset_armed) {
				s_reset_armed = 1;
				return RBX_MENU_STAY;
			}
			return RBX_MENU_RESET;
		case M_STATES: go_page(PAGE_STATES); break;
		case M_DISPLAY: go_page(PAGE_DISPLAY); break;
		case M_SETTINGS: go_page(PAGE_SETTINGS); break;
		case M_CONTROLS: go_page(PAGE_CONTROLS); break;
		case M_DIPS:
			if (rbx_core_dip_count() > 0)
				go_page(PAGE_DIPS);
			else
				rbx_osd_show(T("No DIP switches for %s"), s_set);
			break;
		case M_CHEATS:
			if (any_cheats())
				go_page(PAGE_CHEATS);
			else
				rbx_osd_show(T("No cheats: put %s.ini in cheats/"), s_set);
			break;
		case M_GAME_SAVE: // this game only (the global settings file is not written)
			if (rbx_settings_save_game())
				rbx_osd_show(T("Saved settings for %s"), s_set);
			else
				rbx_osd_show("Could not save game settings");
			break;
		case M_GAME_DEL:
			if (rbx_settings_delete_game()) {
				apply_video(); // (global values right away)
				rbx_osd_show(T("Deleted settings for %s"), s_set);
			} else
				rbx_osd_show("Could not delete game settings");
			break;
		case M_ABOUT:
			go_page(PAGE_ABOUT);
			s_about_scroll = 0;
			break;
		case M_SELECT_GAME: return RBX_MENU_SELECT_GAME;
		case M_REFRESH: return RBX_MENU_REFRESH;
		case M_GROUP: change(+1); break;
		case M_EXIT: return RBX_MENU_EXIT;
		case M_BACK: return RBX_MENU_CLOSE;
		}
		break;
	case PAGE_DISPLAY:
		if (disp_kind(s_sel) == D_BACK) go_page(PAGE_MAIN);
		else change(+1);
		break;
	case PAGE_CONTROLS:
	case PAGE_TURBO:
	case PAGE_STICK:
	case PAGE_TOUCH:
	case PAGE_PORT: {
		int act;
		const int k = row_kind(s_sel, &act);
		if (k == CK_PORT1 || k == CK_PORT2) {
			s_kp = k == CK_PORT2;
			go_page(PAGE_PORT);
			break;
		}
		if (k == CK_BIND_ALL) { // every key row in turn, from button 1
			s_sel = port_row_of_act(RBX_ACT_ATTACK);
			bind_begin(1);
			break;
		}
		if (k == CK_RESET_PORT) {
			rbx_settings_default_port(re_pad_player_profile(s_kp)); // (the device's controls)
			s_changed = 1;
			rbx_osd_show(T("%dP controls reset"), s_kp + 1);
			break;
		}
		if (k == CK_T_EDIT) {
			go_page(PAGE_TEDIT);
			tedit_open();
			break;
		}
		if (k == CK_T_CLEAR) {
			if (!s_reset_armed) {
				s_reset_armed = 1; // (press again to clear)
				break;
			}
			s_reset_armed = 0;
			rbx_touch_clear();
			rbx_touch_save();
			rbx_osd_show("Touch layout cleared");
			break;
		}
		if (k == CK_TOUCH) go_page(PAGE_TOUCH);
		else if (k == CK_BACK) back_out();
		else if (k == CK_TURBO) go_page(PAGE_TURBO);
		else if (k == CK_STICK) go_page(PAGE_STICK);
		else if (k == CK_COMBO) go_page(PAGE_COMBO);
		else if (k == CK_DEFAULTS) {
			rbx_settings_default_keys();
			s_changed = 1;
		} else change(+1);
		break;
	}
	case PAGE_DIPS:
		if (s_sel == dip_items() - 1) go_page(PAGE_MAIN);
		else if (s_sel == dip_items() - 2) {
			rbx_core_dip_defaults();
			save_dips();
			s_changed = 1;
		} else change(+1);
		break;
	case PAGE_CHEATS: {
		int ci = 0;
		if (cheat_kind(s_sel, &ci) == HK_BACK) go_page(PAGE_MAIN);
		else change(+1);
		break;
	}
	case PAGE_ABOUT:
		if (s_about_direct)
			s_close_now = 1;
		else
			go_page(PAGE_MAIN);
		break;
	case PAGE_COMBO:
		if (s_sel == COMBO_ROWS - 1) page_back();
		else change(+1);
		break;
	case PAGE_SETTINGS:
		if (s_sel == SET_BACK) go_page(PAGE_MAIN);
		else change(+1);
		break;
	case PAGE_STATES:
		if (s_sel == S_SAVE) return RBX_MENU_SAVE;
		if (s_sel == S_LOAD) {
			if (s_slot_used[s_state_slot])
				return RBX_MENU_LOAD;
			rbx_osd_show(T("Slot %d is empty"), s_state_slot);
		}
		if (s_sel == S_BACK) go_page(PAGE_MAIN);
		if (s_sel == S_SLOT) change(+1);
		break;
	}
	return RBX_MENU_STAY;
}

// the touch panel drops a report now and then near its top and bottom edges: there a finger counts as gone 60 ms after the
// last one (elsewhere at once: a tap acts when it leaves). s_touch_held: this read is such a kept point, not a real one
static int s_touch_held;
static int touch_read(int *x, int *y)
{
	static uint64_t seen;
	static int lx, ly;
	const int n = rbx_touch_read(x, y);
	const uint64_t now = rbx_time_us();
	s_touch_held = 0;
	if (n) {
		seen = now;
		lx = x[0];
		ly = y[0];
		return n;
	}
	if (seen && (ly < 40 || ly >= 490) && now - seen < 60000) {
		x[0] = lx;
		y[0] = ly;
		s_touch_held = 1;
		return 1;
	}
	return 0;
}
// a flick's speed at the finger's end, per frame: faster flicks go on more than in proportion
static float flick_boost(float v)
{
	const float a = fabsf(v);
	return v * (1.0f + (a > 1.5f ? 1.5f : a) * 1.2f);
}

// the touch effect in the game list and the menus: a soft light and a thin ring of the theme's colour under the finger,
// fading slowly when it leaves. Light added to the picture, from two small textures scaled by the video output: nothing is
// drawn again on the canvases
#define FX_GLOW 64
#define FX_RING 128
#define FX_HOLD 10 // frames
#define FX_FADE 20
static ReiTex *s_fx_glow, *s_fx_ring;
// a texture of light: f = strength by distance from the centre (0-1), white = how far towards white, by distance too
static void fx_fill(ReiTex *x, int size, uint32_t rgb, float (*f)(float), float (*white)(float))
{
	const unsigned r = rgb & 0xFF, g = rgb >> 8 & 0xFF, b = rgb >> 16 & 0xFF;
	for (int y0 = 0; y0 < size; y0++)
		for (int x0 = 0; x0 < size; x0++) {
			const float dx = (x0 + 0.5f) / (size / 2) - 1.0f, dy = (y0 + 0.5f) / (size / 2) - 1.0f, d = sqrtf(dx * dx + dy * dy);
			const float a = d < 1.0f ? f(d) : 0.0f, w = white(d);
			const float cr = r + (255 - r) * w, cg = g + (255 - g) * w, cb = b + (255 - b) * w;
			x->data[(size_t)y0 * x->stride + x0] = (uint32_t)(cr * a) | (uint32_t)(cg * a) << 8 | (uint32_t)(cb * a) << 16 | (uint32_t)(255 * a) << 24;   // (premultiplied: the corners add nothing)
		}
}
static float fx_glow_f(float d) // (no edge: a blur, brightest in the middle)
{
	const float a = 1.0f - d * d;
	return a * a * a;
}
static float fx_glow_w(float d) { return d < 0.5f ? 0.55f * (1.0f - 2.0f * d) : 0.0f; }
static float fx_ring_f(float d) // (a thin line, soft on both sides, and a faint trail inside it)
{
	float a = 1.0f - fabsf(d - 0.9f) / 0.07f;
	a = a < 0.0f ? 0.0f : a * a * (3.0f - 2.0f * a);
	const float tr = d < 0.9f ? 0.16f * d * d * d : 0.0f;
	return a > tr ? a : tr;
}
static float fx_ring_w(float d) { (void)d; return 0.35f; }
static void fx_make(void)
{
	static int gen = -1;
	theme();
	if (gen == s_theme_gen)
		return;
	if (!s_fx_glow)
		s_fx_glow = rbx_video_tex(FX_GLOW, FX_GLOW);
	if (!s_fx_ring)
		s_fx_ring = rbx_video_tex(FX_RING, FX_RING);
	if (!s_fx_glow || !s_fx_ring)
		return;
	gen = s_theme_gen;
	fx_fill(s_fx_glow, FX_GLOW, COL_SEL, fx_glow_f, fx_glow_w);
	fx_fill(s_fx_ring, FX_RING, COL_SEL, fx_ring_f, fx_ring_w);
}
static float ease_out(float t) { return t * (2.0f - t); }
// once a frame, with the touches now
static void touch_fx(int tn, int x, int y)
{
	static int down, age, fade;
	static float fx, fy;
	fx_make();
	if (tn) {
		if (!down)
			age = 0;
		down = 1;
		fade = 0;
		fx = (float)x;
		fy = (float)y;
		if (age < FX_HOLD)
			age++;
		const float e = ease_out((float)age / FX_HOLD);
		rbx_video_touch_fx(0, s_fx_glow, fx, fy, 24.0f + 14.0f * e, 0.3f + 0.3f * e);
		rbx_video_touch_fx(1, s_fx_ring, fx, fy, 19.0f + 10.0f * e, 0.45f * e);
		return;
	}
	if (down) {
		down = 0;
		fade = 1;
	}
	if (!fade)
		return;
	const float e = ease_out((float)age / FX_HOLD), l = 1.0f - (float)fade / FX_FADE; // (as it was, only fainter)
	rbx_video_touch_fx(0, s_fx_glow, fx, fy, 24.0f + 14.0f * e, (0.3f + 0.3f * e) * l);
	rbx_video_touch_fx(1, s_fx_ring, fx, fy, 19.0f + 10.0f * e, 0.45f * e * l);
	fade = fade < FX_FADE ? fade + 1 : 0;
}

// the options' touch: the last paint's panel, rows (text baselines) and choice window; s_mdrag: the rows scrolled by a
// drag (the selected row may leave the screen until a button)
#define MT_ROWS 128
static float s_mscr, s_mtgt; // menu rows scroll (px): drawn, target
static int s_mt_px, s_mt_py, s_mt_h, s_mt_ra, s_mt_rb, s_mt_lh, s_mt_n, s_mt_ty[MT_ROWS];
static int s_mt_wx, s_mt_wy, s_mt_ww, s_mt_wh, s_mt_plh, s_mt_nc;
static int s_mdrag;
static float s_mvel; // the rows' flick: px a frame
static uint64_t s_mt_block; // the finger that opened them is not read until this time
static int s_touch_add = -1; // a key row's Add was tapped: the action
// the 2P Controller row added (its device None) or deleted (back to Auto); the 1P row selected after a Delete
static void p2_add_del(int add)
{
	g_settings.p2_added = add;
	g_settings.pad_src[1] = add ? RE_PAD_SRC_NONE : RE_PAD_SRC_AUTO;
	re_pad_set_source(1, g_settings.pad_src[1]);   // (the device row reads it from here: None, and Left / Right pick another)
	re_pad_set_added_players(add ? 2 : 0);
	rbx_settings_save();
	s_sel = ctrl_row_of(add ? CK_PORT2 : CK_PORT1);
	s_dirty = 1;
	s_drawn_h = 0;
}
#define ROW_BTN_TOUCH 110 // (the touch area of a key row's button, from the row's end)
// a tap over the options: SCE_CTRL bits it stands for (and the row it selects); a drag scrolls a long page
static uint32_t menu_touch(int o_ok)
{
	static int down, x0, y0, last, moved, stopped;
	int tx[6], ty[6];
	const int tn = touch_read(tx, ty);
	const uint32_t ok = o_ok ? SCE_CTRL_CIRCLE : SCE_CTRL_CROSS, back = o_ok ? SCE_CTRL_CROSS : SCE_CTRL_CIRCLE;
	if (rbx_time_us() < s_mt_block) {
		down = 0;
		return 0;
	}
	if (s_mt_wait_up) { // (Done goes on the press: the same finger is still there)
		down = 0;
		s_mt_wait_up = tn != 0;
		return 0;
	}
	if (tn) {
		if (!down) {
			down = 1;
			x0 = tx[0];
			y0 = last = ty[0];
			moved = 0;
			stopped = s_mvel != 0.0f; // (a touch that stops the rows is not a tap)
			s_mvel = 0;
		} else {
			if (abs(ty[0] - y0) > 12 || abs(tx[0] - x0) > 12)
				moved = 1;
			if (moved && !s_pop && s_page == PAGE_ABOUT) {   // (About: a drag scrolls its lines, user 11-15)
				static float acc;
				acc -= (float)(ty[0] - last) / ABOUT_LH;
				const int max = ABOUT_LINES > ABOUT_VISIBLE ? ABOUT_LINES - ABOUT_VISIBLE : 0;
				while (acc >= 1.0f) {
					acc -= 1.0f;
					if (s_about_scroll < max)
						s_about_scroll++;
				}
				while (acc <= -1.0f) {
					acc += 1.0f;
					if (s_about_scroll > 0)
						s_about_scroll--;
				}
				s_dirty = 1;
			}
			if (moved && !s_pop && s_page != PAGE_ABOUT && x0 >= s_mt_px && x0 < s_mt_px + PANEL_W && y0 >= s_mt_ra && y0 < s_mt_rb) {
				const float d = (float)(ty[0] - last);
				s_mtgt -= d;
				if (!s_touch_held)
					s_mvel = s_mvel * 0.5f - d * 0.5f;
				s_mdrag = 1;
				s_dirty = 1;
			}
			last = ty[0];
		}
		return 0;
	}
	if (!down) {
		if (s_mvel != 0.0f) { // after a flick the rows go on, slowing down
			s_mtgt += s_mvel;
			s_mvel *= 0.95f;
			if (fabsf(s_mvel) < 0.6f)
				s_mvel = 0;
			s_dirty = 1;
		}
		return 0;
	}
	down = 0;
	if (moved) {
		s_mvel = flick_boost(s_mvel / LINE_H) * LINE_H;
		return 0;
	}
	if (stopped)
		return 0;
	s_dirty = 1;
	if (s_pop) { // the choice window: a value picks it, outside closes
		if (x0 < s_mt_wx || x0 >= s_mt_wx + s_mt_ww || y0 < s_mt_wy || y0 >= s_mt_wy + s_mt_wh)
			return back;
		const int i = s_mt_plh > 0 ? (y0 - (s_mt_wy + 60)) / s_mt_plh : -1;
		if (y0 >= s_mt_wy + 60 && i >= 0 && i < s_mt_nc) {
			s_pop_sel = i;
			return ok;
		}
		return 0;
	}
	if (x0 < s_mt_px || x0 >= s_mt_px + PANEL_W || y0 < s_mt_py || y0 >= s_mt_py + s_mt_h)
		return back; // (outside the panel: back, as the back button)
	if (s_page == PAGE_ABOUT)
		return back;
	if (y0 < s_mt_ra || y0 >= s_mt_rb)
		return 0;
	for (int i = 0; i < s_mt_n && i < MT_ROWS; i++) {
		const int top = s_mt_ty[i] - 7 - s_mt_lh / 2;
		if (y0 >= top && y0 < top + s_mt_lh) {
			s_mdrag = 0;
			int a;
			const int rk = (s_page == PAGE_PORT || s_page == PAGE_CONTROLS) && x0 >= s_mt_px + PANEL_W - ROW_BTN_TOUCH ?
			               row_kind(i, &a) : -1;
			if (s_page == PAGE_CONTROLS && ((rk == CK_PORT1 && !p2_shown()) || (rk == CK_PORT2 && p2_deletable()))) {
				p2_add_del(rk == CK_PORT1);
				return 0;
			}
			if (rk == CK_ACT && act_extras(a) < RBX_KEY_EXTRA) { // (the Add button at the row's right)
				s_sel = i;
				s_touch_add = a;
				return 0;
			}
			if (rk == CK_ACT2) { // (its Delete)
				act_remove(ACT2_ACT(a), ACT2_X(a));
				s_sel = port_row_of_act(ACT2_ACT(a));
				s_changed = 1;
				return 0;
			}
			if (i == s_sel)
				return ok; // (the selected row again: as the confirm button)
			s_sel = i;
			s_reset_armed = 0;
			return 0;
		}
	}
	return 0;
}

rbx_menu_result rbx_menu_update(uint32_t buttons)
{
	if (s_close_now) {   // (About from the list's footer closed)
		s_close_now = s_about_direct = 0;
		s_prev_buttons = buttons;
		return RBX_MENU_CLOSE;
	}
	const int ps_closed = ps_poll();
	if (ps_closed)
		s_pop = 0;
	static uint64_t rep_next;
	uint32_t pressed = key_repeat(buttons, buttons & ~s_prev_buttons, &rep_next);
	s_prev_buttons = buttons;
	if (pressed)
		s_dirty = 1;
	if (pressed)
		s_mdrag = 0;   // (the buttons bring the selected row back on screen)
	if (s_page != PAGE_TEDIT && !s_bind && !s_vc_ask)
		pressed |= menu_touch(g_settings.menu_confirm == RBX_CONFIRM_CIRCLE);
	if (s_touch_add >= 0) {
		const int a = s_touch_add;
		s_touch_add = -1;
		key_add(a);
		return RBX_MENU_STAY;
	}
	if (!s_list_mode) { // (the list's loop has its own: one tick a frame)
		int fxx[6], fxy[6];
		const int fxn = touch_read(fxx, fxy);
		touch_fx(fxn, fxx[0], fxy[0]);
	}
	if (re_pad_changed()) { // a controller came or went: the Port pages and the Device list show them, the whole panel again
		s_dirty = 1;
		s_drawn_h = 0;
		if (s_kp && !p2_shown() && (s_page == PAGE_PORT || s_page == PAGE_TURBO || s_page == PAGE_COMBO ||
			s_page == PAGE_STICK)) {
			// the device of this port went away (its row is gone): back to Controls
			s_pop = s_bind = 0;
			go_page(PAGE_CONTROLS);
			s_kp = 0;
		}
		if (s_sel >= page_items())
			s_sel = page_items() - 1;
	}
	rbx_menu_result r = RBX_MENU_STAY;
	if (s_page == PAGE_TEDIT && !ps_closed) { // touch layout editor: touch and its own buttons
		tedit_update(pressed);
		return RBX_MENU_STAY;
	}
	const int n = page_items();
	if (s_vc_ask) {
		// VitaControl off: confirm = its lines commented out in the plugin lists (copies kept), then the Vita restarts
		const int o_ok = g_settings.menu_confirm == RBX_CONFIRM_CIRCLE;
		const uint32_t ok = o_ok ? SCE_CTRL_CIRCLE : SCE_CTRL_CROSS, back = o_ok ? SCE_CTRL_CROSS : SCE_CTRL_CIRCLE;
		if ((pressed & ok) && !ps_closed) {
			const int kind = s_vc_ask;
			s_vc_ask = 0;
			const int r = kind == 1 ? re_pad_vitacontrol_off() : kind == 2 ? re_pad_vitacontrol_on() : 1;
			RBX_LOG("input engine restart %d: %d", kind, r);
			if (r > 0) {
				rbx_settings_save(); // (the chosen engine stays)
				scePowerRequestColdReset();
			} else
				rbx_osd_show(r < 0 ? "Could not change the plugin list (config.txt)" : "VitaControl is not in the plugin list");
		} else if ((pressed & (back | SCE_CTRL_START)) || ps_closed) {
			s_vc_ask = 0; // cancelled: Native again (the chosen engine cannot run without the restart), and on out of Controls
			g_settings.input_engine = RE_PAD_ENGINE_NATIVE;
			re_pad_engine_set(g_settings.input_engine);
			s_changed = 1;
			page_back();
		}
		if (!s_vc_ask) {
			s_dirty = 1;
			s_drawn_h = 0;
		}
		return RBX_MENU_STAY;
	}
	if (s_bind) {
		// key binding: wait until every button is up (the confirm press), then the first button pressed is the key.
		// The d-pad, the PS button or 5 s without a button cancel
		int act, key = -1, cancel = ps_closed || !CTRL_PAGE;
		const int k = row_kind(s_sel, &act);
		uint32_t keys = 0;
		for (int i = 0; i < RBX_KEY_LANY; i++) // (a whole stick is not a press)
			keys |= rbx_key_mask(i);
		// a player's own page: its own device alone (another controller's buttons are not taken). Keys are looked for
		// from the last: a controller's L2 / R2 is also L / R in the menus, the key is L2 / R2
		uint32_t bb = buttons, bp = pressed;
		SceCtrlData pd;
		if (s_page == PAGE_PORT || s_page == PAGE_TURBO) {
			re_pad_read(s_kp, &pd);
			bb = (pd.buttons & ~RBX_STICK_BITS) | rbx_stick_bits(pd.lx, pd.ly, pd.rx, pd.ry);
			bp = bb & ~s_bind_prev;
			s_bind_prev = bb;
		} else {
			re_pad_read(0, &pd);
			const uint32_t sb = rbx_stick_bits(pd.lx, pd.ly, pd.rx, pd.ry);
			bb = buttons | sb;
			bp = pressed | (sb & ~s_bind_prev);
			s_bind_prev = sb;
		}
		if (!s_bind_ready) {
			if (!((buttons | bb) & (keys | REPEAT_KEYS)))
				s_bind_ready = 1;
		} else {
			for (int i = RBX_KEY_LANY - 1; i >= 0 && key < 0; i--)
				if (bp & rbx_key_mask(i))
					key = i;
			if (key < 0 && ((pressed | bp) & REPEAT_KEYS))
				cancel = 1;
		}
		if (key < 0 && rbx_time_us() - s_bind_t > BIND_US)
			cancel = 1;
		int ck, cf;
		if (!row_key(k, act, &ck, &cf))
			cancel = 1;
		if (key >= 0 && !cancel) {
			row_set_key(k, act, key);
			s_changed = 1;
			warn_clash(key, cf);
			if (s_bind_all && k == CK_ACT && act != RBX_ACT_START) { // Set all controls: the next key row
				int a2;
				s_sel++;
				while (row_kind(s_sel, &a2) == CK_ACT2)
					s_sel++;
				s_bind_ready = 0;
				s_bind_t = rbx_time_us();
			} else
				s_bind = 0;
			s_dirty = 1;
		} else if (cancel) {
			if (k == CK_ACT2 && s_page == PAGE_PORT) { // a more key not given (or not given again): it goes
				act_remove(ACT2_ACT(act), ACT2_X(act));
				s_sel = port_row_of_act(ACT2_ACT(act));
				s_changed = 1;
			}
			s_bind = 0;
			s_dirty = 1;
		}
		return RBX_MENU_STAY;
	}
	{
		const int o_ok = g_settings.menu_confirm == RBX_CONFIRM_CIRCLE;
		const uint32_t ok = o_ok ? SCE_CTRL_CIRCLE : SCE_CTRL_CROSS, back = o_ok ? SCE_CTRL_CROSS : SCE_CTRL_CIRCLE;
		const char *names[MAX_CHOICES], *title;
		int cur;
		const int nc = item_choices(names, &cur, &title);
		if (s_pop && !ps_closed) { // choice window: up/down, confirm sets, back closes
			if (s_pop_sel >= nc) // (the Device list got shorter)
				s_pop_sel = nc > 0 ? nc - 1 : 0;
			if (!nc)
				s_pop = 0;
			else if (pressed & SCE_CTRL_UP)
				s_pop_sel = wrap(s_pop_sel - 1, nc);
			else if (pressed & SCE_CTRL_DOWN)
				s_pop_sel = wrap(s_pop_sel + 1, nc);
			else if (pressed & ok) {
				set_choice(s_pop_sel);
				if (s_pop_keep) { // (the combo window's second key next)
					s_pop_keep = 0;
					s_pop_sel = 0;
				} else
					s_pop = 0;
				if (s_page != PAGE_CHEATS) // (cheats are not saved)
					s_changed = 1;
			} else if (pressed & (back | SCE_CTRL_START)) {
				if (s_combo_first >= 0) { // the combo window: back to its first key
					for (int i = 0; i < RBX_COMBO_SRC; i++)
						if (s_combo_order[i] == s_combo_first)
							s_pop_sel = i + 1;
					s_combo_first = -1;
				} else
					s_pop = 0;
			}
			if (!s_pop) {
				s_drawn_h = 0; // the whole panel again (the window covered part of it)
				s_combo_first = -1;
			}
			return RBX_MENU_STAY;
		}
		if ((pressed & SCE_CTRL_SQUARE) && s_page == PAGE_COMBO && s_sel < RBX_COMBO_SRC && !ps_closed) { // Square: no combo
			prt()->combo[s_combo_order[s_sel]] = 0;
			s_changed = 1;
			s_dirty = 1;
			return RBX_MENU_STAY;
		}
		if ((pressed & SCE_CTRL_TRIANGLE) && s_page == PAGE_CONTROLS && !ps_closed) { // 1P row: Add a 2P; 2P row: Delete it
			int act;
			const int k = row_kind(s_sel, &act);
			if ((k == CK_PORT1 && !p2_shown()) || (k == CK_PORT2 && p2_deletable())) {
				p2_add_del(k == CK_PORT1);
				return RBX_MENU_STAY;
			}
		}
		if ((pressed & SCE_CTRL_TRIANGLE) && s_page == PAGE_PORT && !ps_closed) { // a key row: one more key
			int act;
			const int k = row_kind(s_sel, &act);
			if (k == CK_ACT) {
				key_add(act);
				return RBX_MENU_STAY;
			}
			if (k == CK_ACT2) { // (a more key: Triangle takes it away again)
				act_remove(ACT2_ACT(act), ACT2_X(act));
				s_sel = port_row_of_act(ACT2_ACT(act));
				s_changed = 1;
				s_dirty = 1;
				return RBX_MENU_STAY;
			}
		}
		if ((pressed & SCE_CTRL_SQUARE) && CTRL_PAGE && !ps_closed) { // a key row that can be none: Square sets None
			int act, ck, cf;
			const int k = row_kind(s_sel, &act);
			if (row_key(k, act, &ck, &cf) && (k == CK_TURBO_KEY || (k == CK_ACT && act >= RBX_ACT_B3))) {
				row_set_key(k, act, RBX_KEY_NONE);
				s_changed = 1;
				s_dirty = 1;
				return RBX_MENU_STAY;
			}
		}
		if ((pressed & ok) && CTRL_PAGE && !ps_closed) { // a key row: wait for the button (no choice window)
			int act, ck, cf;
			if (row_key(row_kind(s_sel, &act), act, &ck, &cf)) {
				bind_begin(0);
				return RBX_MENU_STAY;
			}
		}
		if ((pressed & ok) && nc && !ps_closed) {
			s_pop = 1;
			s_pop_sel = cur;
			s_drawn_h = 0;
			return RBX_MENU_STAY;
		}
	}
	if (s_page == PAGE_ABOUT) { // up/down scroll
		const int max = ABOUT_LINES > ABOUT_VISIBLE ? ABOUT_LINES - ABOUT_VISIBLE : 0;
		if ((pressed & SCE_CTRL_UP) && s_about_scroll > 0)
			s_about_scroll--;
		if ((pressed & SCE_CTRL_DOWN) && s_about_scroll < max)
			s_about_scroll++;
		pressed &= ~REPEAT_KEYS;
	}
	if (pressed & SCE_CTRL_UP) {
		s_sel = wrap(s_sel - 1, n);
		s_reset_armed = 0;
	}
	if (pressed & SCE_CTRL_DOWN) {
		s_sel = wrap(s_sel + 1, n);
		s_reset_armed = 0;
	}
	if (pressed & SCE_CTRL_LEFT)
		change(-1);
	if (pressed & SCE_CTRL_RIGHT)
		change(+1);
	// confirm/back follow the setting (Cross/Circle). If the setting changes during this input, use the setting at press time
	const int o_ok = g_settings.menu_confirm == RBX_CONFIRM_CIRCLE;
	const uint32_t ok = o_ok ? SCE_CTRL_CIRCLE : SCE_CTRL_CROSS, back = o_ok ? SCE_CTRL_CROSS : SCE_CTRL_CIRCLE;
	if (pressed & ok)
		r = confirm();
	else if (pressed & back) {
		if (s_page == PAGE_MAIN)
			r = RBX_MENU_CLOSE;
		else
			back_out();
	}
	if (pressed & SCE_CTRL_START) {
		r = RBX_MENU_CLOSE;
		s_keep_place = (buttons & MENU_OPEN_COMBO) == MENU_OPEN_COMBO && !s_list_mode;
	}
	if (ps_closed) { // PS closes the menu as SELECT+START does (opens again where it was)
		if (s_page == PAGE_TEDIT) {
			rbx_touch_save();
			page_back();
		}
		r = RBX_MENU_CLOSE;
		s_keep_place = !s_list_mode;
	}
	// memory card write only when the user changed something (no card I/O otherwise while a game is loaded)
	if (r != RBX_MENU_STAY && r != RBX_MENU_SAVE && s_changed) {
		rbx_settings_save();
		s_changed = 0;
	}
	return r;
}

// ================================ drawing ================================
// "Re" white + "BOX" in the brand colour (Re franchise naming rule, as the app art). Returns the width
static int draw_brand(UiCanvas *c, int x, int y, float scale)
{
	int w = ui_text(c, x, y, COL_TEXT, scale, "Re");
	w += ui_text(c, x + w, y, COL_BRAND, scale, "BOX");
	return w;
}

// one About line (heading lines start with '#'; the "ReBOX" heading uses the brand colours)
static void about_line(UiCanvas *c, int x, int y, const char *t)
{
	if (!t[0])
		return;
	if (!strncmp(t, "#ReBOX", 6)) {
		const int w = draw_brand(c, x, y, 1.0f);
		ui_text(c, x + w, y, COL_TEXT, 1.0f, t + 6);
		return;
	}
	const int head = t[0] == '#';
	ui_text(c, x, y, head ? COL_SEL : (t[0] == ' ' ? COL_DIM : COL_TEXT), head ? 1.0f : 0.85f, t + head);
}

#define VAL_ON "\x01on"   // item value: draw a toggle switch (two-state items only)
#define VAL_OFF "\x01off"
// the selected row: a soft bar with a shadow and a blinking outline (a steady one under the choice window)
static void sel_bar(UiCanvas *c, int x, int by, int w, int h)
{
	rrect(c, x, by + 3, w, h, 6, theme()->shade);
	rrect(c, x, by, w, h, 6, COL_HILITE);
	if (s_pop) // the choice window is over it
		rrect_outline(c, x, by, w, h, 6, 2, COL_FOCUS_STILL);
	else
		focus_at(c, x, by, w, h);
}
static void item(UiCanvas *c, int px, int ty, int i, int sel, const char *name, const char *value)
{
	const uint32_t col = (i == sel) ? COL_SEL : COL_TEXT;
	if (i == sel)
		sel_bar(c, px + 12, ty - 7 - (LINE_H - 6) / 2, PANEL_W - 40, LINE_H - 6); // bar centred on the text
	ui_text(c, px + 40, ty, i == sel ? COL_TEXT : COL_ITEM, 1.0f, name);
	if (value && value[0] == 1) {
		// two-state item as a toggle switch: a pill with the knob on the right (on, accent colour) or left (off, grey)
		const int on = value[2] == 'n';
		const float r = 10.0f, x0 = (float)(px + VALUE_X) + r, x1 = x0 + 26.0f, cy = (float)ty - 7.0f;
		const uint32_t track = on ? theme()->toggle_on : theme()->toggle_off;
		ui_circle(c, x0, cy, r, track);
		ui_circle(c, x1, cy, r, track);
		ui_rect(c, (int)x0, (int)(cy - r), (int)(x1 - x0), (int)(2 * r), track);
		ui_circle(c, on ? x1 : x0, cy, r - 3.0f, UI_RGBA(0xF4, 0xF4, 0xF4, 0xFF));
	} else if (value) {
		char buf[96];
		snprintf(buf, sizeof(buf), "< %s >", value);
		ui_text(c, px + VALUE_X, ty, col, 1.0f, buf);
	}
}

static void thick_line(UiCanvas *c, float x0, float y0, float x1, float y1, uint32_t col)
{
	ui_line(c, x0, y0, x1, y1, 2.4f, col);
}

// Draw a button symbol starting at (x, cy), vertically centered on cy, and return the width used. L, R, Select, Start, None as text
static int draw_key(UiCanvas *c, int x, int cy, int ty, int key, uint32_t text_col)
{
	const float r = 9.0f;
	const float cx = x + r + 1;
	switch (key) {
	case RBX_KEY_CROSS: {
		const uint32_t col = UI_RGBA(0x7C, 0xB4, 0xF0, 0xFF);
		thick_line(c, cx - r * 0.8f, cy - r * 0.8f, cx + r * 0.8f, cy + r * 0.8f, col);
		thick_line(c, cx - r * 0.8f, cy + r * 0.8f, cx + r * 0.8f, cy - r * 0.8f, col);
		return (int)(r * 2 + 2);
	}
	case RBX_KEY_CIRCLE:
		ui_circle(c, cx, cy, r, UI_RGBA(0xFF, 0x6E, 0x6E, 0xFF));
		ui_circle(c, cx, cy, r - 2.5f, COL_PANEL);
		return (int)(r * 2 + 2);
	case RBX_KEY_SQUARE: {
		const uint32_t col = UI_RGBA(0xF0, 0xA0, 0xD8, 0xFF);
		const float h = r * 0.85f;
		ui_rect(c, (int)(cx - h), (int)(cy - h), (int)(h * 2), 2, col);
		ui_rect(c, (int)(cx - h), (int)(cy + h - 2), (int)(h * 2), 2, col);
		ui_rect(c, (int)(cx - h), (int)(cy - h), 2, (int)(h * 2), col);
		ui_rect(c, (int)(cx + h - 2), (int)(cy - h), 2, (int)(h * 2), col);
		return (int)(r * 2 + 2);
	}
	case RBX_KEY_TRIANGLE: {
		const uint32_t col = UI_RGBA(0x50, 0xE0, 0xC0, 0xFF);
		const float top = cy - r, bot = cy + r * 0.75f, half = r;
		thick_line(c, cx, top, cx - half, bot, col);
		thick_line(c, cx, top, cx + half, bot, col);
		thick_line(c, cx - half, bot, cx + half, bot, col);
		return (int)(r * 2 + 2);
	}
	default: {
		const char *name = rbx_key_name(key);
		ui_text(c, x, ty, text_col, 1.0f, name);
		return ui_text_width(1.0f, name);
	}
	}
}

// a combo value: "None" or two key glyphs joined by "+". Returns the width
static int draw_combo(UiCanvas *c, int x, int ty, int mask, uint32_t col)
{
	if (!mask)
		return ui_text(c, x, ty, col, 1.0f, "None");
	const int x0 = x;
	int first = 1;
	for (int k = 0; k < RBX_COMBO_SRC; k++) {
		const int key = s_combo_order[k];
		if (!(mask & (1 << key)))
			continue;
		if (!first)
			x += ui_text(c, x + 4, ty, col, 1.0f, "+") + 10;
		x += draw_key(c, x, ty - 7, ty, key, col);
		first = 0;
	}
	return x - x0;
}
static int combo_width(int mask) // (draw_combo's width without drawing)
{
	int w = 0, n = 0;
	for (int k = 0; k < RBX_KEY_COUNT; k++)
		if (mask & (1 << k)) {
			w += k <= RBX_KEY_TRIANGLE ? 20 : ui_text_width(1.0f, rbx_key_name(k));
			n++;
		}
	return w + (n > 1 ? ui_text_width(1.0f, "+") + 10 : 0);
}

// a key row's button at its right. Selected row: no box of its own (the focus bar is the box), Triangle and the word in the
// row's colours. Others: a small soft key with a + or x, no outline
static void row_button(UiCanvas *c, int px, int ty, int selected, int del)
{
	const int right = px + PANEL_W - 36;
	if (selected) {
		const char *word = del ? "Delete" : "Add";
		const int ww = ui_text_width(0.85f, word), x = right - ww - 26;
		draw_key(c, x, ty - 7, ty, RBX_KEY_TRIANGLE, COL_SEL);
		ui_text(c, right - ww, ty, COL_SEL, 0.85f, word);
	} else {
		const int cx = right - 14, cy = ty - 7;
		rrect(c, cx - 15, cy - 13, 30, 26, 6, UI_RGBA(0xFF, 0xFF, 0xFF, 0x20));
		if (del)
			for (int k = -5; k < 5; k++) {
				ui_rect(c, cx + k, cy + k, 2, 1, COL_ITEM);
				ui_rect(c, cx - k - 1, cy + k, 2, 1, COL_ITEM);
			}
		else {
			ui_rect(c, cx - 6, cy - 1, 12, 2, COL_ITEM);
			ui_rect(c, cx - 1, cy - 6, 2, 12, COL_ITEM);
		}
	}
}

// clash: the key also does something else in this game (shown in the warning colour with "!")
static void item_key_w(UiCanvas *c, int px, int ty, int i, int sel, const char *name, int key, int clash)
{
	const uint32_t col = clash ? COL_WARN : (i == sel) ? COL_SEL : COL_TEXT;
	item(c, px, ty, i, sel, name, NULL);
	int x = px + VALUE_X;
	ui_text(c, x, ty, col, 1.0f, "<");
	x += ui_text_width(1.0f, "< ");
	x += draw_key(c, x, ty - 7, ty, key, col);
	x += 6 + ui_text(c, x + 6, ty, col, 1.0f, ">");
	if (clash)
		ui_text(c, x + 8, ty, COL_WARN, 1.0f, "! duplicate");
}

static void item_key(UiCanvas *c, int px, int ty, int i, int sel, const char *name, int key)
{
	item_key_w(c, px, ty, i, sel, name, key, 0);
}

// one line of text (size 0.8): symbol chars (0x10-0x13) drawn as button icons
// its width, as drawn below
static int keys_width(const char *s)
{
	char run[128];
	int k = 0, w = 0;
	for (const char *p = s;; p++) {
		const unsigned char ch = (unsigned char)*p;
		if ((ch >= 0x10 && ch <= 0x13) || !ch || k == (int)sizeof(run) - 1) {
			run[k] = 0;
			if (k)
				w += ui_text_width(0.8f, run);
			k = 0;
			if (!ch)
				return w;
			if (ch >= 0x10 && ch <= 0x13) {
				w += 22;
				continue;
			}
		}
		run[k++] = (char)ch;
	}
}
// a footer button: a pill of dark glass with a thin light edge
static void foot_button(UiCanvas *c, int x, int y, int w, int h)
{
	rrect(c, x, y, w, h, h / 2, UI_RGBA(0, 0, 0, 0x48));
	rrect(c, x + 1, y + 1, w - 2, h / 2 - 1, h / 2 - 1, UI_RGBA(0xFF, 0xFF, 0xFF, 0x0C));
	rrect_outline(c, x, y, w, h, h / 2, 1, UI_RGBA(0xFF, 0xFF, 0xFF, 0x38));
}
static int text_keys(UiCanvas *c, int x, int ty, uint32_t col, const char *s) // (the x after it)
{
	char run[128];
	int k = 0;
	for (const char *p = s;; p++) {
		const unsigned char ch = (unsigned char)*p;
		if ((ch >= 0x10 && ch <= 0x13) || !ch || k == (int)sizeof(run) - 1) {
			run[k] = 0;
			if (k) {
				ui_text(c, x, ty, col, 0.8f, run);
				x += ui_text_width(0.8f, run);
			}
			k = 0;
			if (!ch)
				break;
			if (ch >= 0x10 && ch <= 0x13) {
				static const int keys[4] = { RBX_KEY_TRIANGLE, RBX_KEY_CIRCLE, RBX_KEY_CROSS, RBX_KEY_SQUARE };
				x += draw_key(c, x, ty - 6, ty, keys[ch - 0x10], col) + 2;
				continue;
			}
		}
		run[k++] = (char)ch;
	}
	return x;
}

// Draw the menu panel to the canvas (caller uploads). overlay = over the game list (the dimmed list kept in s_under)
static int s_paint_x0, s_paint_y0, s_paint_x1, s_paint_y1;
static const uint32_t *s_under; // game list options: the dimmed list under the window (NULL: just drawn in full)
static int menu_anim(void) { return s_mscr != s_mtgt; } // (the menu is still moving: draw again)

// A footer help longer than its line scrolls: 1 s, then 60 px/s to its end, 1 s, from the start. Between paints of the
// menu only its band is drawn again: the band as the paint left it before the text (kept here), the text moved over it
// (painting and uploading the whole menu each frame took ~19 ms on the device)
#define HELP_BAND_H 40
static uint32_t s_help_bg[PANEL_W * HELP_BAND_H];
static int s_help_bx, s_help_by, s_help_bw, s_help_tw, s_help_off = -1;
static uint32_t s_help_col;
static const char *s_help_text;
static int help_off(int w, int avail)
{
	const int over = w - avail, hold = 1000, travel = over * 1000 / 60, ms = (int)((rbx_time_us() - s_help_t0) / 1000);
	const int tm = ms % (hold + travel + hold);
	return tm < hold ? 0 : tm < hold + travel ? (tm - hold) * 60 / 1000 : over;
}
static void help_band(UiCanvas *c, int keep)
{
	uint32_t *px = ui_pixels(c);
	const int cw = SCREEN_W;
	for (int y = 0; y < HELP_BAND_H; y++) {
		uint32_t *row = px + (size_t)(s_help_by + y) * cw + s_help_bx;
		if (keep)
			memcpy(s_help_bg + y * s_help_bw, row, (size_t)s_help_bw * 4);
		else
			memcpy(row, s_help_bg + y * s_help_bw, (size_t)s_help_bw * 4);
	}
	if (!keep)
		ui_touch_rect(c, s_help_bx, s_help_by, s_help_bw, HELP_BAND_H);
}
static void help_draw(UiCanvas *c, int off)
{
	ui_text_clip(c, s_help_bx - off, s_help_by + HELP_BAND_H - 16, s_help_col, 0.8f, s_help_text, s_help_bx, s_help_bx + s_help_bw);
	s_help_off = off;
}
static void tedit_paint(UiCanvas *c);
static void menu_paint(UiCanvas *c, int overlay)
{
	if (s_page == PAGE_TEDIT) {
		tedit_paint(c);
		return;
	}
	const int o_ok = g_settings.menu_confirm == RBX_CONFIRM_CIRCLE;
	const char *ok_name = o_ok ? G_CIR : G_CRO, *back_name = o_ok ? G_CRO : G_CIR;
	char foot_buf[128];
	const int n = page_items();
		if (s_mdrag)
			s_mscr = s_mtgt; // (the rows follow the finger)
	// group gaps on the main page (rows y move down by the gaps above them)
	int gap_at[M_COUNT + 1] = { 0 };
	if (s_page == PAGE_MAIN && !s_list_mode)
		for (int i = 1; i <= n && i <= M_COUNT; i++) {
			const int k = i < n ? main_kind(i) : -1;
			gap_at[i] = gap_at[i - 1] + ((k == M_STATES || k == M_DISPLAY || k == M_ABOUT) ? GROUP_GAP : 0);
		}
	// rows need not fit. The row area is at most the screen minus a margin; taller pages scroll smoothly (the selected
	// row kept a row away from the edges), fading at a cut edge, with a scroll bar
	const int lh = s_page == PAGE_ABOUT ? ABOUT_LH : LINE_H;
	const int gmax = s_page == PAGE_MAIN && !s_list_mode && n > 0 ? gap_at[n - 1] : 0;
	const int content = n * lh + gmax, maxv = SCREEN_H - 32 - TITLE_H - FOOT_H;
	const int view = content < maxv ? content : maxv;
	const int h = TITLE_H + view + FOOT_H;
	const int px = (SCREEN_W - PANEL_W) / 2, py = (SCREEN_H - h) / 2;
	{
		static int last_page = -1, last_n = -1;
		const float maxs = (float)(content - view);
		const int sy = s_sel * lh + (s_page == PAGE_MAIN && !s_list_mode && s_sel < n ? gap_at[s_sel] : 0);
		if (s_page == PAGE_ABOUT)
			s_mtgt = 0;
		else if (!s_mdrag) {
			if (sy - lh < s_mtgt)
				s_mtgt = (float)(sy - lh);
			if (sy + 2 * lh > s_mtgt + view)
				s_mtgt = (float)(sy + 2 * lh - view);
		}
		if (s_mtgt > maxs) s_mtgt = maxs;
		if (s_mtgt < 0) s_mtgt = 0;
		if (s_page != last_page || n != last_n)
			s_mscr = s_mtgt; // another page: no animation
		last_page = s_page;
		last_n = n;
		if (s_mdrag)
			s_mscr = s_mtgt;   // (the rows follow the finger)
		s_mscr += (s_mtgt - s_mscr) * 0.3f;
		if (fabsf(s_mtgt - s_mscr) < 0.5f)
			s_mscr = s_mtgt;
	}
	const int q0 = s_paint_x0, q1 = s_paint_y0, q2 = s_paint_x1, q3 = s_paint_y1; // what the last paint covered
	s_paint_x0 = px - 8; // area this paint covers (panel and shadow; the choice window adds its own below)
	s_paint_y0 = py - 8;
	s_paint_x1 = px + PANEL_W + 14;
	s_paint_y1 = py + h + 16;
	if (overlay) { // over the game list: it is already dimmed there
		s_drawn_h = 0;
		if (s_under) {
			// the last paint's area outside this window, from the kept list (the opaque window covers its inside;
			// its rounded corners come back too)
			const int ix0 = px + 8, iy0 = py + 8, ix1 = px + PANEL_W - 8, iy1 = py + h - 8;
			if (q1 < iy0) ui_blit_rect(c, s_under, q0, q1, q2 - q0, iy0 - q1);
			if (q3 > iy1) ui_blit_rect(c, s_under, q0, iy1, q2 - q0, q3 - iy1);
			const int m0 = q1 > iy0 ? q1 : iy0, m1 = q3 < iy1 ? q3 : iy1;
			if (m1 > m0) {
				if (q0 < ix0) ui_blit_rect(c, s_under, q0, m0, ix0 - q0, m1 - m0);
				if (q2 > ix1) ui_blit_rect(c, s_under, ix1, m0, q2 - ix1, m1 - m0);
			}
		}
	} else if (s_drawn_h == h) {
		// if the panel size is unchanged, redraw only the panel (do not recompose and copy the whole dimmed screen every
		// time), with the shadow and the choice window of the last paint: they are outside the panel
		const int x0 = q0 < s_paint_x0 ? q0 : s_paint_x0, y0 = q1 < s_paint_y0 ? q1 : s_paint_y0;
		const int x1 = q2 > s_paint_x1 ? q2 : s_paint_x1, y1 = q3 > s_paint_y1 ? q3 : s_paint_y1;
		ui_fill(c, x0, y0, x1 - x0, y1 - y0, COL_BG);
	} else {
		ui_fill(c, 0, 0, SCREEN_W, SCREEN_H, COL_BG); // (set, not blended: a blend reads the whole screen, ~10 ms)
		s_drawn_h = h;
	}
	window_box(c, px, py, PANEL_W, h, COL_EDGE, COL_PANEL); // rounded panel, outline and shadow

	const char *title = "OPTIONS";
	snprintf(foot_buf, sizeof(foot_buf), T("Up/Down: select   Left/Right/%s: change   %s: back"), ok_name, back_name);
	const char *foot = foot_buf;
	switch (s_page) {
	case PAGE_DISPLAY:
		title = "DISPLAY";
		if (s_list_mode)
			foot = "Left/Right: change   (global settings)";
		break;
	case PAGE_TURBO:
		title = s_kp ? "2P TURBO" : "1P TURBO";
		foot = "Hold the turbo key: the turbo button repeats";
		break;
	case PAGE_STICK:
		title = s_kp ? "2P ANALOG STICKS" : "1P ANALOG STICKS";
		foot = "Each stick is set on its own";
		break;
	case PAGE_TOUCH:
		title = "TOUCH CONTROLS";
		foot = s_reset_armed ? "Press again to clear every touch button" : "Touch buttons on the screen press the keys you give them";
		break;
	case PAGE_PORT: {
		// connected controllers
		int m = snprintf(foot_buf, sizeof(foot_buf), T("Connected: %s"), re_pad_port_name(0));
		for (int i = 1; i < RE_PAD_PORTS && m < (int)sizeof(foot_buf); i++)
			if (re_pad_connected(i))
				m += snprintf(foot_buf + m, sizeof(foot_buf) - m, ", %s (%d)", re_pad_port_name(i), i);
		{ // with the device whose controls these are
			static char tb[48];
			snprintf(tb, sizeof(tb), "%dP CONTROLLER  (%s)", s_kp + 1, port_none() ? "no device" :
				re_pad_profile_name(re_pad_player_profile(s_kp)));
			title = tb;
		}
		foot = foot_buf;
		break;
	}
	case PAGE_CONTROLS:
		title = "CONTROLS";
		foot = s_list_mode ? "Each player: its device, buttons, stick, turbo, combo keys   (global settings)"
			: "Each player: its device, buttons, stick, turbo, combo keys";
		break;
	case PAGE_COMBO:
		title = s_kp ? "2P COMBO KEYS" : "1P COMBO KEYS";
		snprintf(foot_buf, sizeof(foot_buf), "A key acts as two keys pressed together   Left/Right: change   %s: list   " G_SQU ": none",
			ok_name);
		break;
	case PAGE_DIPS:
		title = "DIP SWITCHES";
		foot = "Some settings take effect after Reset game";
		break;
	case PAGE_CHEATS:
		title = "CHEATS";
		foot = rbx_core_has_cheats() ? "Start stage applies when a new game starts" : "Left/Right: change   Cheat files: cheats/<set>.ini";
		break;
	case PAGE_SETTINGS:
		title = "SETTINGS";
		break;
	case PAGE_STATES:
		title = "SAVE / LOAD STATE";
		foot = "Quick save: SELECT+L   Quick load: SELECT+R";
		break;
	case PAGE_ABOUT:
		title = "ABOUT";
		snprintf(foot_buf, sizeof(foot_buf), T("Up/Down: scroll (%d/%d)   %s: back"), s_about_scroll + 1,
			ABOUT_LINES > ABOUT_VISIBLE ? ABOUT_LINES - ABOUT_VISIBLE + 1 : 1, back_name);
		break;
	default:
		if (!s_list_mode && rbx_settings_has_game())
			foot = "This game uses its own settings (Delete game settings to undo)";
		break;
	}
	if (s_bind) {
		int act;
		const int k = row_kind(s_sel, &act);
		snprintf(foot_buf, sizeof(foot_buf), T("%sPress the button for %s   (D-pad or 5 s: cancel)"),
			s_bind_all ? "Set all controls: " : "", row_key_name(k, act));
		foot = foot_buf;
	} else if (s_page == PAGE_PORT || s_page == PAGE_TURBO) {
		int act, ck, cf;
		if (row_key(row_kind(s_sel, &act), act, &ck, &cf)) {
			snprintf(foot_buf, sizeof(foot_buf), "%s: press a button to set it   " G_SQU ": none   Left/Right: change%s", ok_name,
				s_page == PAGE_PORT ? "   " G_TRI ": one more" : "");
			foot = foot_buf;
		}
	}
	{
		// title: cut with "..." when it overflows the panel (long game titles)
		char tbuf[128];
		snprintf(tbuf, sizeof(tbuf), "%s", s_page == PAGE_MAIN && !s_list_mode ? s_title : title);
		int len = (int)strlen(tbuf);
		while (len > 4 && ui_text_width(1.1f, tbuf) > PANEL_W - 48) {
			len--;
			memcpy(tbuf + len - 3, "...", 4);
		}
		ui_text(c, px + 24, py + 36, COL_TEXT, 1.1f, tbuf);
	}

	const int ra = py + TITLE_H, rb = ra + view, scr = (int)floorf(s_mscr + 0.5f);
	s_mt_px = px;
	s_mt_py = py;
	s_mt_h = h;
	s_mt_ra = ra;
	s_mt_rb = rb;
	s_mt_lh = lh;
	s_mt_n = n;
	s_mt_nc = 0;
	for (int i = 0; i < n && i < MT_ROWS; i++)
		s_mt_ty[i] = ra + i * lh + (s_page == PAGE_MAIN && !s_list_mode ? gap_at[i] : 0) - scr + lh / 2 + 7;
	s_mt_px = px;
	s_mt_py = py;
	s_mt_h = h;
	s_mt_ra = ra;
	s_mt_rb = rb;
	s_mt_lh = lh;
	s_mt_n = n;
	s_mt_nc = 0;
	for (int i = 0; i < n && i < MT_ROWS; i++)
		s_mt_ty[i] = ra + i * lh + (s_page == PAGE_MAIN && !s_list_mode ? gap_at[i] : 0) - scr + lh / 2 + 7;
	ui_clip_y(c, ra, rb);
	s_focus_cy0 = ra;
	s_focus_cy1 = rb;
	for (int i = 0; i < n; i++) {
		const int g = s_page == PAGE_MAIN && !s_list_mode ? gap_at[i] : 0;
		const int ty = ra + i * lh + g - scr + lh / 2 + 7;
		if (ty + lh < ra || ty - lh > rb)
			continue;
		switch (s_page) {
		case PAGE_MAIN: {
			static const char *const names[M_COUNT] = { "Resume game", "Reset game", "Save / Load state  >", "Display  >",
				"Controls  >", "DIP switches  >", "Cheats  >", "Save game settings", "Delete game settings", "About  >",
				"Select game", "Exit", "Back", "Refresh ROM list", "Group games by", "Settings  >" };
			const int k = main_kind(i);
			const char *name = names[k];
			char rbuf[64];
			if (k == M_RESET && s_reset_armed) {
				snprintf(rbuf, sizeof(rbuf), T("Reset game - press %s again"), o_ok ? "Circle" : "Cross");
				name = rbuf;
			}
			if (k == M_CHEATS && !any_cheats())
				name = "Cheats (none for this game)";
			if (k == M_DIPS && rbx_core_dip_count() <= 0)
				name = "DIP switches (none)";
			item(c, px, ty, i, s_sel, name, k == M_GROUP ? s_group_name[g_settings.list_group] : NULL);
			break;
		}
		case PAGE_DISPLAY: {
			static const char *const names[D_NEON] = { "Show FPS", "Frame buffer", "Screen", "Rotate", "Filter", "Scanlines",
				"Scanline strength", "Border (4:3 / 2x)", "Border color", "Theme" };
			const int d = disp_kind(i);
			static char nn[48];
			if (d == D_NEONGAP)
				snprintf(nn, sizeof(nn), "List neon: Gap");
			else if (d >= D_NEON && d < D_BACK)
				snprintf(nn, sizeof(nn), T("List neon: %s"), rbx_neon_params[d - D_NEON].name);
			const char *v = NULL, *name = d < D_NEON ? names[d] : d == D_BACK ? "Back" : nn;
			static char nv[16];
			if (d == D_NEONGAP) {
				snprintf(nv, sizeof(nv), T("%d px"), g_settings.neon_list_gap);
				v = nv;
			} else if (d >= D_NEON && d < D_BACK) {
				rbx_neon_value_text(d - D_NEON, g_settings.neon_list[d - D_NEON], nv, sizeof(nv));
				v = nv;
			}
			switch (d) {
			case D_FPS: v = g_settings.show_fps ? VAL_ON : VAL_OFF; break;
			case D_ASPECT: v = aspect[g_settings.aspect]; break;
			case D_ROTATE: v = rotate[g_settings.rotate]; break;
			case D_FRAMEBUF: v = framebuf_text(); break;
			case D_FILTER: v = filter[g_settings.filter]; break;
			case D_SCAN: v = scan[g_settings.scan_type]; break;
			case D_SCAN_STR: v = scan_str[g_settings.scan_strength]; break;
			case D_BORDER: v = border[g_settings.border]; break;
			case D_BORDER_COL: v = border_col[g_settings.border_color]; break;
			case D_THEME: v = s_theme_name[g_settings.theme]; break;
			}
			item(c, px, ty, i, s_sel, name, v);
			break;
		}
		case PAGE_CONTROLS:
		case PAGE_TURBO:
		case PAGE_STICK:
		case PAGE_TOUCH:
		case PAGE_PORT: {
			int act, rk, rf;
			const int k = row_kind(i, &act);
			const int is_key = row_key(k, act, &rk, &rf);
			const int clash = is_key && key_clash(rk, rf) != NULL;
			if (is_key && s_bind && i == s_sel)
				item(c, px, ty, i, s_sel, k == CK_ACT2 ? "    +" : row_key_name(k, act), "Press a button...");
			else if (k == CK_ACT) {
				item_key_w(c, px, ty, i, s_sel, act_name(act), prt()->keymap[act], clash);
				if (act_extras(act) < RBX_KEY_EXTRA)
					row_button(c, px, ty, i == s_sel, 0);
			} else if (k == CK_ACT2) {
				item_key_w(c, px, ty, i, s_sel, "    +", prt()->keyx[ACT2_ACT(act)][ACT2_X(act)], clash);
				row_button(c, px, ty, i == s_sel, 1);
			}
			else if (k == CK_PORT1 || k == CK_PORT2) {
				char pbuf[48], lbuf[24];
				snprintf(lbuf, sizeof(lbuf), T("%dP Controller  >"), k == CK_PORT2 ? 2 : 1);
				item(c, px, ty, i, s_sel, lbuf, re_pad_player_text(k == CK_PORT2, pbuf, sizeof(pbuf)));
				if (k == CK_PORT1 ? !p2_shown() : p2_deletable())   // (Add a 2P, or Delete the added one)
					row_button(c, px, ty, i == s_sel, k == CK_PORT2);
			} else if (k == CK_ENGINE)
				item(c, px, ty, i, s_sel, "Input engine", re_pad_engine_text(g_settings.input_engine));
			else if (k == CK_DEVICE) {
				char pbuf[48];
				item(c, px, ty, i, s_sel, "Device", re_pad_player_text(s_kp, pbuf, sizeof(pbuf)));
			} else if (k == CK_BIND_ALL)
				item(c, px, ty, i, s_sel, "Set all controls", NULL);
			else if (k == CK_RESET_PORT)
				item(c, px, ty, i, s_sel, "Reset to default controls", NULL);
			else if (k == CK_TURBO_KEY)
				item_key_w(c, px, ty, i, s_sel, "Turbo key", prt()->turbo_key, clash);
			else if (k == CK_TURBO)
				item(c, px, ty, i, s_sel, "Turbo  >", NULL);
			else if (k == CK_TOUCH)
				item(c, px, ty, i, s_sel, "Touch controls  >", NULL);
			else if (k == CK_T_ON)
				item(c, px, ty, i, s_sel, "Touch buttons", g_settings.touch_keys ? VAL_ON : VAL_OFF);
			else if (k == CK_T_SHOW)
				item(c, px, ty, i, s_sel, "Show touch buttons", g_settings.touch_show ? VAL_ON : VAL_OFF);
			else if (k == CK_T_EDIT) {
				char eb[48];
				snprintf(eb, sizeof(eb), T("Edit touch layout  >   (%d)"), rbx_touch_count());
				item(c, px, ty, i, s_sel, eb, NULL);
			} else if (k == CK_T_CLEAR)
				item(c, px, ty, i, s_sel, s_reset_armed && i == s_sel ? "Clear touch layout - press again" : "Clear touch layout", NULL);
			else if (k == CK_TURBO_BTN) {
				char bbuf[16];
				snprintf(bbuf, sizeof(bbuf), T("Button %d"), prt()->turbo_btn);
				item(c, px, ty, i, s_sel, "Turbo button", bbuf);
			} else if (k == CK_TURBO_SPEED)
				item(c, px, ty, i, s_sel, "Turbo speed", turbo_speed[prt()->turbo_speed]);
			else if (k == CK_STICK)
				item(c, px, ty, i, s_sel, "Analog sticks  >", NULL);
			else if (k == CK_SMAP_SET)
				item(c, px, ty, i, s_sel, act / STICK_ROWS ? "Right stick" : "Left stick", stick_state(act / STICK_ROWS));
			else if (k == CK_SMAP_DIR) {
				char sb[24];
				const int st = act / STICK_ROWS, d = act % STICK_ROWS - 1;
				const char *v = stick_tgt[prt()->smap[st][d]];
				for (int a = 0; a < RBX_ACT_COUNT; a++) // (set as a key on the player's page: that wins)
					if (rbx_act_bits(prt(), a) & rbx_key_mask(RBX_KEY_LUP + st * RBX_SDIR_COUNT + d))
						v = act_name(a);
				snprintf(sb, sizeof(sb), "    %s", stick_dir[d]);
				item(c, px, ty, i, s_sel, sb, v);
			}
			else if (k == CK_CONFIRM)
				item_key(c, px, ty, i, s_sel, "Menu confirm button", o_ok ? RBX_KEY_CIRCLE : RBX_KEY_CROSS);
			else if (k == CK_COMBO)
				item(c, px, ty, i, s_sel, "Combo keys  >", NULL);
			else
				item(c, px, ty, i, s_sel, k == CK_DEFAULTS ? "Restore defaults (every player)" : "Back", NULL);
			break;
		}
		case PAGE_COMBO:
			if (i == COMBO_ROWS - 1) {
				item(c, px, ty, i, s_sel, "Back", NULL);
			} else {
				// row: the key itself (glyph), value: < its combo >
				const int key = s_combo_order[i];
				const uint32_t col = (i == s_sel) ? COL_SEL : COL_TEXT;
				item(c, px, ty, i, s_sel, "", NULL);
				draw_key(c, px + 40, ty - 7, ty, key, i == s_sel ? COL_TEXT : COL_ITEM);
				int x = px + VALUE_X;
				x += ui_text(c, x, ty, col, 1.0f, "< ");
				x += draw_combo(c, x, ty, prt()->combo[key], col);
				ui_text(c, x + 6, ty, col, 1.0f, ">");
			}
			break;
		case PAGE_DIPS:
			if (i < rbx_core_dip_count()) {
				const int o = rbx_core_dip_get(i);
				item(c, px, ty, i, s_sel, rbx_core_dip_name(i), o < 0 ? "?" : rbx_core_dip_option_name(i, o));
			} else
				item(c, px, ty, i, s_sel, i == n - 2 ? "Restore defaults" : "Back", NULL);
			break;
		case PAGE_CHEATS: {
			int ci = 0;
			const int k = cheat_kind(i, &ci);
			if (k == HK_STAGE)
				item(c, px, ty, i, s_sel, "Start stage", stages[g_settings.start_stage]);
			else if (k == HK_HEALTH)
				item(c, px, ty, i, s_sel, "Infinite health (1P)", g_settings.cheat_health ? VAL_ON : VAL_OFF);
			else if (k == HK_FILE) {
				const char *v = rbx_core_cheat_option_name(ci, rbx_core_cheat_get(ci));
				item(c, px, ty, i, s_sel, rbx_core_cheat_name(ci), v ? v : "?");
			} else
				item(c, px, ty, i, s_sel, "Back", NULL);
			break;
		}
		case PAGE_STATES:
			if (i == S_SLOT) {
				char sbuf[64];
				const int k = s_state_slot;
				if (s_slot_used[k]) { // file times are UTC
					SceDateTime lt;
					SceRtcTick tick;
					sceRtcGetTick(&s_slot_time[k], &tick);
					sceRtcConvertUtcToLocalTime(&tick, &tick);
					sceRtcSetTick(&lt, &tick);
					snprintf(sbuf, sizeof(sbuf), "%d   used  %02d-%02d %02d:%02d", k, lt.month, lt.day, lt.hour, lt.minute);
				} else
					snprintf(sbuf, sizeof(sbuf), "%d   empty", k);
				item(c, px, ty, i, s_sel, "Slot", sbuf);
			} else {
				static const char *const names[S_COUNT] = { "", "Save state", "Load state", "Back" };
				item(c, px, ty, i, s_sel, names[i], NULL);
			}
			break;
		case PAGE_SETTINGS: {
			const int li = rbx_lang_index(g_settings.language);
			if (i == SET_LANG)
				item(c, px, ty, i, s_sel, "Language", li >= 0 ? rbx_lang_at(li)->name : "English");
			else
				item(c, px, ty, i, s_sel, "Back", NULL);
			break;
		}
		case PAGE_ABOUT: {
			const int k = s_about_scroll + i;
			if (k < ABOUT_LINES)
				about_line(c, px + 24, ty, s_about[k]);
			break;
		}
		}
	}
	if (content > view) { // fades where rows are cut (towards the panel colour)
		enum { FADE = 14 };
		for (int k = 0; k < FADE; k++) {
			const unsigned al = (unsigned)(0xE0 - k * 0xE0 / FADE);
			if (scr > 0)
				ui_rect(c, px + 2, ra + k, PANEL_W - 22, 1, UI_RGBA(theme()->panel_rgb[0], theme()->panel_rgb[1], theme()->panel_rgb[2], al));
			if (scr < content - view)
				ui_rect(c, px + 2, rb - 1 - k, PANEL_W - 22, 1, UI_RGBA(theme()->panel_rgb[0], theme()->panel_rgb[1], theme()->panel_rgb[2], al));
		}
	}
	ui_clip_y(c, 0, SCREEN_H);
	s_focus_cy0 = 0;
	s_focus_cy1 = SCREEN_H;
	if (s_page != PAGE_ABOUT) { // scroll bar (display only)
		const int tx = px + PANEL_W - 16, t0 = ra + 4, th = view - 8;
		int bh = content > 0 ? th * view / content : th, by = content > 0 ? t0 + (int)(th * s_mscr / content) : t0;
		if (bh < 16) bh = 16;
		if (by + bh > t0 + th) by = t0 + th - bh;
		rrect(c, tx, t0, 4, th, 2, UI_RGBA(0xFF, 0xFF, 0xFF, 0x18));
		rrect(c, tx, by, 4, bh, 2, COL_THUMB);
	}
	{
		// the footer: the selected row's help (it scrolls left when longer than the line: 1 s,
		// then 60 px/s to its end, 1 s, from the start), else the page's own line
		int act;
		const char *help = s_bind || s_pop || s_vc_ask || s_page == PAGE_ABOUT ? NULL : CTRL_PAGE ?
			row_help(row_kind(s_sel, &act)) : page_row_help();
		s_help_scroll = 0;
		if (help) {
			const int x0 = px + 24, avail = PANEL_W - 48, w = ui_text_width(0.8f, help), key = s_page * 256 + s_sel;
			const uint64_t now = rbx_time_us();
			if (key != s_help_key) {
				s_help_key = key;
				s_help_t0 = now;
			}
			if (w > avail && avail <= PANEL_W && py + h - 18 + 16 <= SCREEN_H && py + h - 18 + 16 - HELP_BAND_H >= 0) {
				s_help_scroll = 1;   // (rbx_menu_draw moves it between paints)
				s_help_bx = x0;
				s_help_by = py + h - 18 + 16 - HELP_BAND_H;
				s_help_bw = avail;
				s_help_tw = w;
				s_help_col = s_page == PAGE_DIPS ? COL_WARN : COL_DIM;
				s_help_text = help;
				help_band(c, 1);
				help_draw(c, help_off(w, avail));
			} else
				ui_text_clip(c, x0, py + h - 18, s_page == PAGE_DIPS ? COL_WARN : COL_DIM, 0.8f, help, x0, x0 + avail);
		} else
			text_keys(c, px + 24, py + h - 18, s_page == PAGE_DIPS ? COL_WARN : COL_DIM, foot);
	}
	if (s_vc_ask) { // the engine's restart? (over the panel)
		s_focus_want = 0; // (the row's blinking outline is a layer above the canvas: off while the window is up)
		const int ww = 560, wh = 170, wx = (SCREEN_W - ww) / 2, wy = (SCREEN_H - wh) / 2;
		ui_rect(c, px, py, PANEL_W, h, UI_RGBA(0x00, 0x00, 0x00, 0x80));
		window_box(c, wx, wy, ww, wh, COL_SEL, COL_PANEL);
		static const char *const head[3] = { "VitaControl keeps RePad out", "VitaControl is switched off",
			"VitaControl starts after a restart" };
		static const char *const line1[3] = { "Switch VitaControl off in the plugin list (config.txt, a copy is kept)",
			"Switch VitaControl on in the plugin list (config.txt, a copy is kept)", "It is in the plugin list but not running yet." };
		static const char *const act[3] = { "switch off and restart", "switch on and restart", "restart" };
		const int k = s_vc_ask - 1;
		ui_text(c, wx + 24, wy + 38, COL_TEXT, 1.0f, head[k]);
		ui_text(c, wx + 24, wy + 72, COL_TEXT, 0.8f, line1[k]);
		ui_text(c, wx + 24, wy + 96, COL_TEXT, 0.8f, k < 2 ? "and restart the Vita now?" : "Restart the Vita now?");
		char kb[80];
		snprintf(kb, sizeof(kb), T("%s: %s   %s: cancel (Native)"), ok_name, act[k], back_name);
		text_keys(c, wx + 24, wy + 140, COL_DIM, kb);
	}
	if (s_pop) { // choice window over the panel
		const char *names[MAX_CHOICES], *ptitle;
		int cur;
		const int nc = item_choices(names, &cur, &ptitle);
		if (nc) {
			// rows of LINE_H; many values (the combo pairs, long DIP lists) get shorter rows so the window fits the screen
			const int plh = 64 + nc * LINE_H + 12 <= SCREEN_H - 24 ? LINE_H : (SCREEN_H - 24 - 76) / nc;
			const int ww = 420, wh = 64 + nc * plh + 12, wx = (SCREEN_W - ww) / 2, wy = (SCREEN_H - wh) / 2;
			s_mt_wx = wx;
			s_mt_wy = wy;
			s_mt_ww = ww;
			s_mt_wh = wh;
			s_mt_plh = plh;
			s_mt_nc = nc;
			if (wy - 8 < s_paint_y0) s_paint_y0 = wy - 8;
			if (wy + wh + 16 > s_paint_y1) s_paint_y1 = wy + wh + 16;
			ui_rect(c, px, py, PANEL_W, h, UI_RGBA(0x00, 0x00, 0x00, 0x80)); // dim the panel under it
			window_box(c, wx, wy, ww, wh, COL_SEL, COL_PANEL);
			char tb[64];
			snprintf(tb, sizeof(tb), "%s", ptitle);
			int tl = (int)strlen(tb);
			while (tl > 4 && ui_text_width(1.0f, tb) > ww - 48) { // (long DIP / cheat names)
				tl--;
				memcpy(tb + tl - 3, "...", 4);
			}
			const int tw = ui_text(c, wx + 24, wy + 38, COL_TEXT, 1.0f, tb);
			if (s_choice_kind == CH_COMBO) // "Combo for" the key
			{
				int x = wx + 24 + tw + 8;
				x += draw_key(c, x, wy + 31, wy + 38, s_combo_order[s_sel], COL_TEXT);
				if (s_combo_first >= 0) { // (second step: "= first +")
					x += ui_text(c, x + 10, wy + 38, COL_TEXT, 1.0f, "=") + 20;
					x += draw_key(c, x, wy + 31, wy + 38, s_combo_first, COL_TEXT);
					ui_text(c, x + 6, wy + 38, COL_TEXT, 1.0f, "+");
				}
			}
			ui_rect(c, wx + 20, wy + 50, ww - 40, 1, COL_DIM);
			const float ts = plh < 30 ? 0.8f : 1.0f; // (short rows: smaller text)
			for (int i = 0; i < nc; i++) {
				const int ty = wy + 60 + i * plh + plh / 2 + 7, by = ty - 7 - (plh - 6) / 2;
				const uint32_t col = i == s_pop_sel ? COL_SEL : COL_TEXT;
				if (i == s_pop_sel) {
					rrect(c, wx + 12, by + 3, ww - 24, plh - 6, 6, theme()->shade);
					rrect(c, wx + 12, by, ww - 24, plh - 6, 6, COL_HILITE);
					focus_at(c, wx + 12, by, ww - 24, plh - 6);
				}
				if (s_choice_kind == CH_KEYS) // button icons as in the item, not their names
					draw_key(c, wx + 44, ty - 7, ty, i, col);
				else if (s_choice_kind == CH_COMBO) {
					const int key = combo_pick(i);
					if (key < 0)
						ui_text(c, wx + 44, ty, col, 1.0f, "None");
					else
						draw_key(c, wx + 44, ty - 7, ty, key, col);
				}
				else
					ui_text_clip(c, wx + 44, ty, col, ts, names[i] ? names[i] : "?", wx + 44, wx + ww - 16);
				if (i == cur)
					ui_circle(c, (float)(wx + 28), (float)(ty - 7), 5.0f, col);
			}
		}
	}
}

// a touch button's key: the PlayStation symbols as icons, the others as names, centred on (cx, cy)
static void tkey_label(UiCanvas *c, int cx, int cy, int key, uint32_t col)
{
	if (key >= RBX_KEY_CROSS && key <= RBX_KEY_TRIANGLE)
		draw_key(c, cx - 10, cy, cy + 7, key, col);
	else if (RBX_TKEY_IS_COMBO(key))
		draw_combo(c, cx - combo_width(key - RBX_TKEY_COMBO) / 2, cy + 7, key - RBX_TKEY_COMBO, col);
	else {
		const char *nm = rbx_touch_key_name(key);
		ui_text(c, cx - ui_text_width(1.0f, nm) / 2, cy + 7, col, 1.0f, nm);
	}
}
// one touch button: a translucent rounded box with its key (sel: the selected one in the editor)
static void tarea(UiCanvas *c, const ReiTouchArea *a, int sel, int edit)
{
	const int r = a->w < a->h ? a->w / 5 : a->h / 5;
	rrect(c, a->x, a->y, a->w, a->h, r, sel ? COL_HILITE : UI_RGBA(0xFF, 0xFF, 0xFF, edit ? 0x38 : 0x24));
	rrect_outline(c, a->x, a->y, a->w, a->h, r, 2, sel ? COL_SEL : UI_RGBA(0xFF, 0xFF, 0xFF, edit ? 0xB0 : 0x70));
	tkey_label(c, a->x + a->w / 2, a->y + a->h / 2, a->key, UI_RGBA(0xFF, 0xFF, 0xFF, edit ? 0xFF : 0xC0));
	if (edit) { // size handles: every corner, and the middle of every edge of the selected one
		const uint32_t hc = sel ? COL_SEL : UI_RGBA(0xFF, 0xFF, 0xFF, 0xB0);
		const int hs = sel ? 12 : 8;
		const int cx[3] = { a->x, a->x + a->w / 2, a->x + a->w }, cy[3] = { a->y, a->y + a->h / 2, a->y + a->h };
		for (int j = 0; j < 3; j++)
			for (int i = 0; i < 3; i++)
				if ((i != 1 || j != 1) && (sel || (i != 1 && j != 1)))
					ui_rect(c, cx[i] - hs / 2, cy[j] - hs / 2, hs, hs, hc);
		if (sel) { // its size (in TE_STEP steps) and place
			char sz[24];
			snprintf(sz, sizeof(sz), "%dx%d", a->w, a->h);
			ui_text(c, a->x + 4, a->y + a->h - 6, hc, 0.6f, sz);
			snprintf(sz, sizeof(sz), "%d,%d", a->x, a->y);
			ui_text(c, a->x + 4, a->y + 14, hc, 0.6f, sz);
		}
	}
}
static void tedit_paint(UiCanvas *c)
{
	// the whole screen: the dimmed list (game list options) or the game behind a light shade
	if (s_under)
		ui_blit(c, s_under);
	else
		ui_fill(c, 0, 0, SCREEN_W, SCREEN_H, UI_RGBA(0x00, 0x00, 0x00, 0x70));
	s_paint_x0 = s_paint_y0 = 0;
	s_paint_x1 = SCREEN_W;
	s_paint_y1 = SCREEN_H;
	s_drawn_h = 0;
	focus_off();
	for (int i = 0; i < rbx_touch_count(); i++)
		tarea(c, rbx_touch_area(i), i == te_sel, 1);
	if (te_mode == TE_NEW && te_nw > 2 && te_nh > 2)
		rrect_outline(c, te_nx, te_ny, te_nw, te_nh, 6, 2, te_nw >= TE_MIN && te_nh >= TE_MIN ? COL_SEL : COL_DIM);
	// header with Done, footer with the help
	ui_rect(c, 0, 0, SCREEN_W, 46, UI_RGBA(0x00, 0x00, 0x00, 0xA8));
	char t[64];
	snprintf(t, sizeof(t), T("TOUCH LAYOUT   (%d button%s)"), rbx_touch_count(), rbx_touch_count() == 1 || !rbx_lang_english() ? "" : "s");
	ui_text(c, 20, 31, COL_TEXT, 1.0f, t);
	rrect(c, SCREEN_W - 120, 7, 108, 32, 6, COL_SEL);
	ui_text(c, SCREEN_W - 120 + (108 - ui_text_width(0.9f, "Done")) / 2, 29, COL_PANEL, 0.9f, "Done");
	ui_rect(c, 0, SCREEN_H - 52, SCREEN_W, 52, UI_RGBA(0x00, 0x00, 0x00, 0xA8));
	text_keys(c, 20, SCREEN_H - 30, COL_DIM, "Drag on an empty spot: new button   Tap a button: its key, Duplicate or Delete   Drag it: move   Drag an edge or corner: size (6px steps)");
	text_keys(c, 20, SCREEN_H - 10, COL_DIM, G_SQU ": new button   " G_TRI ": key of the selected one   START: done");
	if (te_pop >= 0) {
		int wx, wy, ww, wh;
		te_pop_rect(&wx, &wy, &ww, &wh);
		const ReiTouchArea *a = rbx_touch_area(te_pop);
		window_box(c, wx, wy, ww, wh, COL_SEL, COL_PANEL);
		ui_text(c, wx + 24, wy + 38, COL_TEXT, 1.0f, te_cpop == 1 ? "Combo: first key" : te_cpop ? "Combo: second key" : "Button key");
		ui_rect(c, wx + 20, wy + 50, ww - 40, 1, COL_DIM);
		for (int i = 0; te_cpop && i < TE_CPOP_N; i++) {
			const int ty = wy + 60 + i * TE_CPOP_RH + TE_CPOP_RH / 2 + 7, by = ty - 7 - (TE_CPOP_RH - 4) / 2;
			const uint32_t col = i == te_cpop_sel ? COL_SEL : COL_TEXT;
			if (i == te_cpop_sel) {
				rrect(c, wx + 12, by, ww - 24, TE_CPOP_RH - 4, 6, COL_HILITE);
				rrect_outline(c, wx + 12, by, ww - 24, TE_CPOP_RH - 4, 6, 2, COL_FOCUS_STILL);
			}
			const int key = te_ckey(i);
			draw_key(c, wx + 44, ty - 7, ty, key, col);
			if (a && RBX_TKEY_IS_COMBO(a->key) && ((a->key - RBX_TKEY_COMBO) & (1 << key)) &&
				(te_cpop == 1 || ((a->key - RBX_TKEY_COMBO) & (1 << te_cfirst))))
				ui_circle(c, (float)(wx + 28), (float)(ty - 7), 5.0f, col);
		}
		for (int i = 0; !te_cpop && i < TE_POP_N; i++) {
			const int ty = wy + 60 + i * TE_POP_RH + TE_POP_RH / 2 + 7, by = ty - 7 - (TE_POP_RH - 6) / 2;
			const uint32_t col = i == te_pop_sel ? COL_SEL : COL_TEXT;
			if (i == te_pop_sel) {
				rrect(c, wx + 12, by, ww - 24, TE_POP_RH - 6, 6, COL_HILITE);
				rrect_outline(c, wx + 12, by, ww - 24, TE_POP_RH - 6, 6, 2, COL_FOCUS_STILL);
			}
			if (i == TE_ROW_DEL)
				ui_text(c, wx + 44, ty, COL_WARN, 1.0f, "Delete");
			else if (i == TE_ROW_DUP)
				ui_text(c, wx + 44, ty, col, 1.0f, "Duplicate");
			else if (i == TE_ROW_COMBO) { // the button's pair, if it has one
				int x = wx + 44;
				ui_text(c, x, ty, col, 1.0f, "Combo  >");
				x += ui_text_width(1.0f, "Combo  >  ");
				if (a && RBX_TKEY_IS_COMBO(a->key))
					draw_combo(c, x, ty, a->key - RBX_TKEY_COMBO, col);
			}
			else if (i <= RBX_KEY_TRIANGLE)
				draw_key(c, wx + 44, ty - 7, ty, i, col);
			else
				ui_text(c, wx + 44, ty, col, 1.0f, rbx_touch_key_name(i));
			if (a && (i == a->key || (i == TE_ROW_COMBO && RBX_TKEY_IS_COMBO(a->key))))
				ui_circle(c, (float)(wx + 28), (float)(ty - 7), 5.0f, col);
		}
	}
}

// ---- touch buttons over the game (slot 4, under the other layers): drawn again only when the layout changes ----
static UiCanvas *s_touch_cv;
static int s_touch_drawn = -1, s_touch_vis, s_touch_theme = -1;
static uint64_t s_touch_pm; // (the presses drawn)
void rbx_touch_overlay_update(int hidden)
{
	const int want = !hidden && g_settings.touch_keys && g_settings.touch_show && rbx_touch_count() > 0;
	if (!want) {
		if (s_touch_vis)
			ui_hide(s_touch_cv);
		s_touch_vis = 0;
		return;
	}
	// (as ReST-V) a button shows its press in the theme's selection colours, as the editor's selected one; a press change
	// redraws only the buttons it changed. A new theme or layout draws them all again
	theme();
	const uint64_t pm = rbx_touch_pressed();
	if (s_touch_vis && s_touch_drawn == rbx_touch_gen() && s_touch_theme == s_theme_gen) {
		const uint64_t ch = pm ^ s_touch_pm;
		if (!ch)
			return;
		for (int i = 0; i < rbx_touch_count() && i < 64; i++)
			if (ch & (1ull << i)) {
				const ReiTouchArea *a = rbx_touch_area(i);
				ui_clear_rect(s_touch_cv, a->x, a->y, a->w, a->h);
				tarea(s_touch_cv, a, (pm >> i) & 1, 0);
			}
		ui_commit(s_touch_cv);
		s_touch_pm = pm;
		return;
	}
	if (!s_touch_cv)
		s_touch_cv = ui_canvas(4, 0, 0, SCREEN_W, SCREEN_H);
	if (!s_touch_cv)
		return;
	ui_clear(s_touch_cv);
	for (int i = 0; i < rbx_touch_count(); i++)
		tarea(s_touch_cv, rbx_touch_area(i), i < 64 && ((pm >> i) & 1), 0);
	ui_commit(s_touch_cv);
	s_touch_drawn = rbx_touch_gen();
	s_touch_theme = s_theme_gen;
	s_touch_pm = pm;
	s_touch_vis = 1;
}

void rbx_menu_draw(void)
{
	focus_tick(); // (the blink: the focus layer's opacity only)
	if (s_cv)
		shot_check(s_cv);
	if (menu_anim())
		s_dirty = 1;
	if (!s_dirty && s_cv && s_help_scroll) {   // only the footer's help moving: its band alone
		const int off = help_off(s_help_tw, s_help_bw);
		if (off != s_help_off) {
			help_band(s_cv, 0);
			help_draw(s_cv, off);
			ui_commit(s_cv);
		}
		return;
	}
	if (!s_dirty || !s_cv)
		return;
	s_dirty = 0;
	focus_begin();
	menu_paint(s_cv, 0);
	focus_end();
	ui_commit(s_cv);
}

#ifdef RBX_ENABLE_LOG
// (log builds: with ux0:data/rebox/menubench present, the in-game menu opened and its focus moved 120 times; the paint
// and upload times logged, the whole draw as the main loop does)
void rbx_menu_bench(void)
{
	SceIoStat st;
	if (sceIoGetstat(RBX_DATA_DIR "menubench", &st) < 0)
		return;
	rbx_menu_should_open(MENU_OPEN_COMBO);
	// (as a key held down: the focus one row on every 6th frame, the frames between as the main loop draws them)
	uint64_t sum = 0, mx = 0, fsum = 0, fmx = 0, ssum = 0;
	int slow = 0, nf = 0, ns = 0;
	float last_scr = s_mscr;
	const int n = page_items();
	for (int k = 0; k < 240; k++) {
		if (k % 6 == 5) {
			s_sel = (s_sel + 1) % (n > 0 ? n : 1);
			s_dirty = 1;
		}
		const uint64_t t0 = sceKernelGetProcessTimeWide();
		rbx_menu_draw();
		const uint64_t dt = sceKernelGetProcessTimeWide() - t0;
		vita2d_start_drawing();
		vita2d_clear_screen();
		rbx_video_draw_layers();
		vita2d_end_drawing();
		vita2d_swap_buffers();
		sum += dt;
		if (dt > mx && k > 0)
			mx = dt;
		slow += dt > 12000;
		if (k % 6 == 5) {
			const int scrolled = s_mscr != last_scr;
			last_scr = s_mscr;
			if (scrolled) { ssum += dt; ns++; } else { fsum += dt; nf++; if (dt > fmx) fmx = dt; }
		}
	}
	RBX_LOG("menubench: menu draw %.2f ms avg, max %.2f ms (the first frame left out), %d of 240 frames over 12 ms; focus moves %d in view %.2f ms avg (max %.2f), %d scrolling %.2f ms avg",
		sum / 240000.0, mx / 1000.0, slow, nf, nf ? fsum / 1000.0 / nf : 0.0, fmx / 1000.0, ns, ns ? ssum / 1000.0 / ns : 0.0);
	rbx_menu_hide();
}
#endif

void rbx_menu_hide(void)
{
	focus_off();
	if (s_cv)
		ui_hide(s_cv);
	s_dirty = 1;
	s_drawn_h = 0;
}

// ---- loading screen: the app's background art, the game name, the stage with its percentage, a spinner and a moving
// bar (same look as ReSys12 / ReST-V) ----
static char s_loading_title[96];
static uint64_t s_loading_us; // time spent drawing (logged at the end)
static uint32_t s_loading_gap_max; // longest time between two shown frames
static int s_loading_gaps, s_loading_gap_at; // frames over 40 ms apart; progress at the longest
static uint32_t s_loading_gap_draw; // draw + present time of that frame
static uint32_t s_loading_ps_max, s_loading_present_max; // longest PS button poll, longest present
static int s_loading_draws;
// The background and the game name are drawn once (kept in s_load_base); each later draw puts back and redraws only the
// band with the stage, the spinner and the bar, so only that band is uploaded.
// The ROM load runs on its own thread and only sets the value (rbx_loading_set); the main thread draws every frame
// (rbx_loading_frame), so the spinner and the bar move at 60 fps whatever the loader is doing (owner: the screen
// stuttered when the loader drew between its steps)
static uint32_t *s_load_base;
static int s_load_base_ok;
// the app's background art 20% darker (80% brightness): decoded and drawn once (rbx_loading_prepare), then copied
static uint32_t *s_load_bgpx;
static void loading_bg(UiCanvas *c)
{
	if (s_load_bgpx) {
		memcpy(ui_pixels(c), s_load_bgpx, (size_t)SCREEN_W * SCREEN_H * 4);
		ui_touch(c);
		return;
	}
	ui_clear(c);
	int bg_w, bg_h;
	uint8_t *bg = rbx_video_load_bg(&bg_w, &bg_h);
	if (bg)
		ui_image(c, bg, bg_w, bg_h, 205);
	else
		ui_rect(c, 0, 0, SCREEN_W, SCREEN_H, UI_RGBA(0, 0, 0, 0xFF));
	free(bg);
	s_load_bgpx = (uint32_t *)malloc((size_t)SCREEN_W * SCREEN_H * 4);
	if (s_load_bgpx)
		memcpy(s_load_bgpx, ui_pixels(c), (size_t)SCREEN_W * SCREEN_H * 4);
}

void rbx_loading_prepare(void)
{
	ensure_canvases();
	loading_bg(s_cv);
}

void rbx_loading_title(const char *title)
{
	snprintf(s_loading_title, sizeof(s_loading_title), "%s", title ? title : "");
	s_load_base_ok = 0;
}

static volatile int s_loading_target; // set by any thread
static int s_loading_shown;           // drawn value, eases toward the target
void rbx_loading_set(int permille)
{
	if (permille < 0) permille = 0;
	if (permille > 1000) permille = 1000;
	__atomic_store_n(&s_loading_target, permille, __ATOMIC_RELEASE);
}
void rbx_loading_progress(int permille)
{
	rbx_loading_set(permille);
	s_loading_shown = s_loading_target; // (jump: called at the start and the end)
	rbx_loading_frame();
}
void rbx_loading_frame(void)
{
	const uint64_t t_ps = rbx_time_us();
	ps_poll(); // (loading: a single PS press does nothing, a double press still goes home)
	const uint64_t now = rbx_time_us();
	if (now - t_ps > s_loading_ps_max)
		s_loading_ps_max = (uint32_t)(now - t_ps);
	const int target = __atomic_load_n(&s_loading_target, __ATOMIC_ACQUIRE);
	if (s_loading_shown < target)
		s_loading_shown += (target - s_loading_shown + 3) / 4; // (the bar glides instead of jumping between steps)
	else
		s_loading_shown = target;
	const int permille = s_loading_shown;
	s_loading_draws++;
	ensure_canvases();
	UiCanvas *c = s_cv;
	// bar at the bottom with a small margin left, right and below; the title above it
	const int m = 36, h = 14, w = SCREEN_W - 2 * m, x = m, y = SCREEN_H - m - h;
	const int sb = y - 50, band = sb - 24; // stage text baseline; first row of the redrawn band
	s_drawn_h = 0;
	if (!s_load_base_ok) {
		loading_bg(c);
		const char *title = s_loading_title[0] ? s_loading_title : "Loading...";
		const int ttw = ui_text_width(0.9f, title), tx = ttw < w ? (SCREEN_W - ttw) / 2 : x;
		ui_text_clip(c, tx, sb - 18 - 20, COL_DIM, 0.9f, title, x, x + w);
		if (!s_load_base)
			s_load_base = (uint32_t *)malloc((size_t)SCREEN_W * SCREEN_H * 4);
		if (s_load_base) {
			memcpy(s_load_base, ui_pixels(c), (size_t)SCREEN_W * SCREEN_H * 4);
			s_load_base_ok = 1;
		}
	} else
		ui_blit_rect(c, s_load_base, 0, band, SCREEN_W, SCREEN_H - band);
	// capsule bar with a vertical gradient and gloss, stripes moving inside the filled part, a glowing head, a spinner and
	// the stage with its percentage
	const float r = h * 0.5f;
	const int fw = w * permille / 1000;
	ui_rect(c, x - 3, y - 3, w + 6, h + 6, UI_RGBA(0x00, 0x00, 0x00, 0x70)); // shade under the bar
	for (int yy = 0; yy < h; yy++) { // track and fill row by row (rounded ends)
		const float dy = yy + 0.5f - r, in = r - sqrtf(r * r - dy * dy);
		const int t0 = x + (int)in, t1 = x + w - (int)in;
		ui_rect(c, t0, y + yy, t1 - t0, 1, theme()->bar_track);
		if (fw > 2) {
			const int f1 = x + fw - (fw > 2 * r ? (int)in : 0);
			const int g = yy * 255 / (h - 1);
			const uint32_t ca = theme()->bar_top, cb = theme()->bar_bot;
			const uint32_t col = UI_RGBA((int)(ca & 0xFF) + (((int)(cb & 0xFF) - (int)(ca & 0xFF)) * g >> 8),
				(int)((ca >> 8) & 0xFF) + (((int)((cb >> 8) & 0xFF) - (int)((ca >> 8) & 0xFF)) * g >> 8),
				(int)((ca >> 16) & 0xFF) + (((int)((cb >> 16) & 0xFF) - (int)((ca >> 16) & 0xFF)) * g >> 8), 0xFF);
			if (f1 > t0)
				ui_rect(c, t0, y + yy, f1 - t0, 1, col);
			if (yy < h / 3 && f1 > t0) // gloss
				ui_rect(c, t0, y + yy, f1 - t0, 1, UI_RGBA(0xFF, 0xFF, 0xFF, 0x38));
		}
	}
	if (fw > 2 * r) { // stripes drifting to the right
		const int step = 22, off = (int)((now / 25000) % step);
		for (int sx = -h + off; sx < fw; sx += step) {
			const float x0 = (float)(x + sx), x1 = x0 + h - 4;
			if (x0 >= x + r && x1 <= x + fw - r)
				ui_line(c, x0, (float)(y + h - 2), x1, (float)(y + 2), 6.0f, UI_RGBA(0xFF, 0xFF, 0xFF, 0x30));
		}
	}
	if (fw > 0) { // glowing head
		const float hx = (float)(x + fw), hy = y + r;
		const float pulse = 0.5f + 0.5f * sinf((float)(now % 1000000) * 6.2832f / 1e6f);
		ui_circle(c, hx, hy, r + 7 + 2 * pulse, theme()->glow1);
		ui_circle(c, hx, hy, r + 3, theme()->glow2);
		ui_circle(c, hx, hy, r - 1, theme()->glow3);
	}
	// centred text with room around everything. From the bottom: the bar, 50 px, the stage with its percentage and the
	// loading icon after it (26 px apart, centred as one group), 18 px, the game name (centred, clipped to the bar)
	const char *stg = permille < 950 ? "Loading ROMs" : "Starting";
	char stage[80];
	snprintf(stage, sizeof(stage), "%s   %d%%", stg, permille / 10);
	const float icon_r = 11.0f; // dots on this circle, 3 px each: the icon is 28 px wide
	const int tw = ui_text_width(0.9f, stage), gap = 26, gw = tw + gap + 28, gx0 = (SCREEN_W - gw) / 2;
	ui_text(c, gx0, sb, COL_DIM, 0.9f, stage);
	const float scx = (float)(gx0 + tw + gap + 14), scy = (float)(sb - 5);
	const int ph = (int)((now / 70000) % 12);
	for (int i = 0; i < 12; i++) {
		const float a = i * 6.2832f / 12.0f;
		const int age = (ph - i + 12) % 12, al = 0xFF - age * 0x14;
		ui_circle(c, scx + icon_r * sinf(a), scy - icon_r * cosf(a), 3.0f, (theme()->spinner & 0xFFFFFFu) | ((uint32_t)(al < 0x20 ? 0x20 : al) << 24));
	}
	ui_commit(c);
	rbx_video_strips(NULL, 0, 0, 0); // (the game list's rows, left on the screen until this first frame)
	rbx_video_list_under(NULL);      // (and its background under a still preview)
	shot_check_as(c, RBX_DATA_DIR "shotload");
	if (s_loading_draws % 30 == 0)
		sceKernelPowerTick(SCE_KERNEL_POWER_TICK_DEFAULT);
	const uint64_t t_pr = rbx_time_us();
	s_loading_us += t_pr - now;
	rbx_video_present();
	static uint64_t last_end;
	const uint64_t end = rbx_time_us();
	if (end - t_pr > s_loading_present_max)
		s_loading_present_max = (uint32_t)(end - t_pr);
	if (s_loading_draws > 1 && end - last_end > 40000) {
		s_loading_gaps++;
		if (end - last_end > s_loading_gap_max) {
			s_loading_gap_max = (uint32_t)(end - last_end);
			s_loading_gap_at = permille;
			s_loading_gap_draw = (uint32_t)(end - now);
		}
	}
	last_end = end;
}

void rbx_loading_done(void)
{
	RBX_LOG("loading screen: %d draws, %llu ms drawing, %d gaps over 40 ms, longest %u us at %d (its draw %u us), "
		"PS poll max %u us, present max %u us", s_loading_draws, (unsigned long long)(s_loading_us / 1000), s_loading_gaps,
		(unsigned)s_loading_gap_max, s_loading_gap_at, (unsigned)s_loading_gap_draw, (unsigned)s_loading_ps_max,
		(unsigned)s_loading_present_max);
	s_loading_ps_max = s_loading_present_max = 0;
	s_loading_us = 0;
	s_loading_gap_max = 0;
	s_loading_gaps = 0;
	s_loading_draws = 0;
	s_load_base_ok = 0;
	s_loading_shown = s_loading_target = 0;
	if (s_cv)
		ui_hide(s_cv);
}

// ---- notice (bottom center, 2 s) ----
static uint64_t s_osd_until;
static int s_osd_on;

void rbx_osd_show(const char *fmt, ...)
{
	char msg[128];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);
	RBX_LOG("notice: %s", msg);
	ensure_canvases();
	UiCanvas *c = s_osd_cv;
	const int w = ui_text_width(1.0f, msg) + 40, h = 36;
	const int x = (SCREEN_W - w) / 2;
	ui_clear(c);
	ui_rect(c, x, 0, w, h, UI_RGBA(0x00, 0x00, 0x00, 0xC8));
	ui_rect(c, x, 0, w, 2, COL_SEL);
	ui_text(c, x + 20, 26, COL_TEXT, 1.0f, msg);
	ui_canvas_move(c, 0, SCREEN_H - h - 16);
	ui_commit(c);
	s_osd_until = rbx_time_us() + 2000000;
	s_osd_on = 1;
}

void rbx_osd_tick(void)
{
	if (s_osd_on && rbx_time_us() > s_osd_until) {
		ui_hide(s_osd_cv);
		s_osd_on = 0;
	}
}

// ---- FPS (box in the game picture's top-right corner; log builds with one decimal) ----
static int s_fps_shown;
static uint64_t s_fps_t0;
static int s_fps_frames;
static float s_fps;
static unsigned s_fps_count; // frames since the box was last drawn
// hidden while the menu is open (it covered Done of the touch layout editor); rbx_fps_tick draws it again
void rbx_fps_hide(void)
{
	if (s_fps_shown && s_fps_cv)
		ui_hide(s_fps_cv);
	s_fps_shown = 0;
}

static uint32_t s_fps_frame_no; // frames shown since the game started (test builds show it)
void rbx_fps_reset_frames(void) { s_fps_frame_no = 0; }
void rbx_fps_tick(void)
{
	// presented frames per second, over the last 0.5 s
	const uint64_t now = rbx_time_us();
	if (!s_fps_t0)
		s_fps_t0 = now;
	s_fps_frames++;
	s_fps_frame_no++;
	if (now - s_fps_t0 >= 500000) {
		s_fps = s_fps_frames * 1000000.0f / (float)(now - s_fps_t0);
		s_fps_frames = 0;
		s_fps_t0 = now;
	}
	if (!g_settings.show_fps) {
		rbx_fps_hide();
		return;
	}
	if (s_fps_shown && ++s_fps_count < 6) // (the box is drawn again every 0.1 s)
		return;
	s_fps_count = 0;
	const int was_shown = s_fps_shown;
	s_fps_shown = 1;
	ensure_canvases();
	UiCanvas *c = s_fps_cv;
	char buf[48];
#ifdef RBX_ENABLE_LOG
	snprintf(buf, sizeof(buf), "%.1f FPS  F%u", (double)s_fps, (unsigned)s_fps_frame_no); // (test builds: with the frame number)
#else
	snprintf(buf, sizeof(buf), T("%d FPS"), (int)(s_fps + 0.5f)); // release builds round (59.8 -> 60)
#endif
	const uint32_t col = s_fps >= 59.0f ? UI_RGBA(0x60, 0xFF, 0x60, 0xFF) : UI_RGBA(0xFF, 0xC0, 0x40, 0xFF);
	{   // (ADR-181) the same text, colour and place as the box shown: not drawn and sent again (release builds: "60 FPS"
		// most of the time; test builds' frame number changes it every time)
		static char last[48];
		static uint32_t last_col;
		static int last_k[6];
		int k[6] = { 0, 0, SCREEN_W, SCREEN_H, rbx_input_rot, 1 };
		if (!rbx_video_picture_rect(&k[0], &k[1], &k[2], &k[3]))
			k[5] = 0;
		if (was_shown && col == last_col && !strcmp(buf, last) && !memcmp(k, last_k, sizeof(k)))
			return;
		memcpy(last, buf, sizeof(last));
		last_col = col;
		memcpy(last_k, k, sizeof(k));
	}
	int w = ui_text_width(1.0f, buf) + 16;
	if (w > 240)
		w = 240;
	ui_clear(c);
	ui_rect(c, 240 - w, 0, w, 30, UI_RGBA(0x00, 0x00, 0x00, 0xA0));
	ui_text(c, 240 - w + 8, 22, col, 1.0f, buf);
	// flush in the game picture's top-right corner, no margin (the whole screen before the first frame); held upright (Vertical L / R)
	// the reader's top-right corner, turned with the picture (the menu stays as it is)
	int px, py, pw, ph;
	if (!rbx_video_picture_rect(&px, &py, &pw, &ph)) {
		px = py = 0;
		pw = SCREEN_W;
		ph = SCREEN_H;
	}
	int cx, cy;
	if (rbx_input_rot == 90) {
		rbx_video_layer_rot(3, 1);
		cx = px + pw - 32; // (turned: canvas row 0 is the right edge)
		cy = py + ph - 240;
	} else if (rbx_input_rot == 270) {
		rbx_video_layer_rot(3, 3);
		cx = px;
		cy = py;
	} else {
		rbx_video_layer_rot(3, 0);
		cx = px + pw - 240;
		cy = py;
	}
	cx = cx < 0 ? 0 : cx > SCREEN_W - 32 ? SCREEN_W - 32 : cx;
	cy = cy < 0 ? 0 : cy > SCREEN_H - 30 ? SCREEN_H - 30 : cy;
	ui_canvas_move(c, cx, cy);
	ui_commit(c);
}

// ================================ game list ================================
// As RetroArch's: the makers as tabs (L/R), their systems in a column at the left, the system's games at the right;
// Left / Right move the focus between the columns, touch picks any of them and scrolls. SELECT = filter (Playable: games
// with ROMs / All: everything, games without ROMs grey, picking one shows a notice / Favorites). Square = favorite on/off
// (star), Triangle = search, START, PS or the gear button = options (Display, Controls, Refresh ROM list; global
// settings). The cursor starts on the last game; nothing starts on its own
#define COL_GREY     (theme()->grey)
#define COL_GREY_SEL (theme()->grey_sel)
#define MAX_TABS 256
static rbx_game_found s_found[RBX_GAMES_MAX];
static int s_gl_n;
static int gl_playable(int i) { return s_found[i].present && s_found[i].supported; }
// (a game of a board not ported yet: game -1, its list entry has "port", the MAME driver; never started)
static int gl_unported(int i) { return s_found[i].game < 0; }
static const char *gl_e_rom(int i);
static const char *gl_set(int i) { return gl_unported(i) ? gl_e_rom(i) : rbx_core_game_set(s_found[i].game); }
// the list's data of a game (gamelist/*.json); NULL with no list: the core's own
static const rbx_list_entry *s_gl_e[RBX_GAMES_MAX];
static const char *gl_e_rom(int i) { return s_gl_e[i] ? s_gl_e[i]->rom : ""; }
static const char *gl_title(int i) { return s_gl_e[i] && s_gl_e[i]->title ? s_gl_e[i]->title : rbx_core_game_title(s_found[i].game); }
// the tab of a game: a board system ("Taito B System") has its own tab, the single-game boards share one
// the list's search (triangle): ROM name or title containing the text, any case; its games in one tab of their own
static char s_search[32];
static char s_search_tab[48];

static int contains_nocase(const char *h, const char *n)
{
	const size_t ln = strlen(n);
	for (; *h; h++)
		if (!strncasecmp(h, n, ln))
			return 1;
	return 0;
}

static int gl_search_match(int i);

// the game's group (the left column) and maker (the tabs: a core module; searching, one tab for the results; grouped by
// anything but the machine, one tab). Each game's groups are found once after a scan (s_gl_key)
static const char *s_gl_key[3][RBX_GAMES_MAX];   // machine, genre, publisher
#define MAX_PUBS 512
static char s_pub[MAX_PUBS][28];
static int s_npub;
static int gl_all(void) { return g_settings.list_group == RBX_GROUP_ALL; }
// the board's name without the maker's in front ("Konami TMNT": "TMNT"); Seta's boards have only their maker's name
static const char *board_name(int game)
{
	const char *b = rbx_core_game_board(game), *mk = rbx_module_name(rbx_game_module(game));
	const size_t n = strlen(mk);
	return !strncmp(b, mk, n) && b[n] == ' ' && strcmp(mk, "Seta") ? b + n + 1 : b;
}
// the publisher, without its licensee or partner ("Konami (Centuri license)", "Banpresto / Allumer"): each name once
static const char *pub_name(int game)
{
	char n[28];
	snprintf(n, sizeof(n), "%s", rbx_game_maker(game));
	n[strcspn(n, "(/")] = 0;
	for (int k = (int)strlen(n) - 1; k >= 0 && n[k] == ' '; k--)
		n[k] = 0;
	if (n[0] >= 'a' && n[0] <= 'z')   // ("bootleg")
		n[0] = (char)(n[0] - 32);
	for (int k = 0; k < s_npub; k++)
		if (!strcmp(s_pub[k], n))
			return s_pub[k];
	if (s_npub == MAX_PUBS)
		return "Other";
	memcpy(s_pub[s_npub], n, sizeof(n));
	return s_pub[s_npub++];
}
static void gl_keys(void)
{
	s_npub = 0;
	for (int i = 0; i < s_gl_n; i++) {
		const int g = s_found[i].game;
		const rbx_list_entry *e = s_gl_e[i];
		s_gl_key[0][i] = e && e->board ? e->board : board_name(g);
		s_gl_key[1][i] = e && e->genre ? e->genre : "Other";
		s_gl_key[2][i] = e && e->publisher ? e->publisher : pub_name(g);
	}
}
static const char *gl_board(int i)
{
	if (s_search[0])
		return gl_unported(i) ? rbx_gamelist_tab(s_gl_e[i]->tab) : rbx_module_name(rbx_game_module(s_found[i].game));
	switch (g_settings.list_group) {
	case RBX_GROUP_YEAR: return s_gl_e[i] && s_gl_e[i]->year ? s_gl_e[i]->year : gl_unported(i) ? "" : rbx_game_year(s_found[i].game);
	case RBX_GROUP_GENRE: return s_gl_key[1][i];
	case RBX_GROUP_PUBLISHER: return s_gl_key[2][i];
	case RBX_GROUP_ALL: return "All";
	default: return s_gl_key[0][i];
	}
}
static int gl_maker(int i)
{
	if (s_search[0])
		return -1;
	if (g_settings.list_group != RBX_GROUP_MACHINE)
		return -3;
	return s_gl_e[i] ? s_gl_e[i]->tab : rbx_game_module(s_found[i].game);
}
static const char *const s_gl_filter_name[RBX_LIST_FILTER_COUNT] = { "Playable", "All", "Favorites" };

// games visible under the current filter
static int gl_visible(int i)
{
	if (s_search[0] && !gl_search_match(i))
		return 0;
	if (g_settings.list_filter == RBX_LIST_ALL)
		return 1;
	if (g_settings.list_filter == RBX_LIST_FAVORITES)
		return rbx_fav_is(gl_set(i));
	return gl_playable(i);
}
static int s_ft_n, s_ft_x0[6], s_ft_x1[6]; // the footer's buttons on the screen (touch), and their pad keys
static uint32_t s_ft_key[6];
#define FT_ABOUT 0x40000000u   // (the About button: touch only, no pad key)
#define FT_INFO  0x20000000u   // (the Info button: touch only)

static const char *s_gl_tabs[MAX_TABS];
static int gl_search_match(int i)
{
	return contains_nocase(gl_set(i), s_search) || contains_nocase(gl_title(i), s_search);
}

// The list (as RetroArch's): the makers as tabs (L/R), their systems in the left column, the system's games on the
// right; Left / Right move the focus between the two columns (s_gl_col)
#define MAX_MAKERS 64
static int s_gl_mk[MAX_MAKERS], s_gl_nmk, s_gl_mkc;   // the tabs (module index; -1: the search results), the current one
static int s_gl_ntabs, s_gl_tab, s_gl_nvis;           // the systems of the current maker (s_gl_tabs), the current one
static int s_gl_col = 1;                              // focus: 0 the systems, 1 the games
static int s_tab_x0[MAX_MAKERS], s_tab_x1[MAX_MAKERS];   // the maker tabs on the screen (touch)
// the maker tab row slid left by s_tab_cur pixels, gliding to s_tab_tgt (a finger drags it; a new current tab out of
// view brings it in)
static float s_tab_cur, s_tab_tgt, s_tab_max;   // (s_tab_max: the row's widest slide, from its last drawing)
static float s_tab_vel;   // the tab row's flick: px a frame after the finger left (it goes on, slowing down, as the games)
static int s_tab_last = -1;
static const char *maker_label(int m)
{
	if (m == -3)
		return s_group_name[g_settings.list_group];
	return m < 0 ? s_search_tab : rbx_gamelist_tabs() ? rbx_gamelist_tab(m) : rbx_module_name(m);
}
// the game last chosen in each system (back there: at it again)
static struct { const char *sys; int mk, game; } s_gl_last[32];
static void gl_last_set(int game)
{
	const char *sys = gl_board(game);
	const int mk = gl_maker(game);
	int k = 0;
	while (k < 32 && s_gl_last[k].sys && !(s_gl_last[k].mk == mk && !strcmp(s_gl_last[k].sys, sys)))
		k++;
	if (k < 32) {
		s_gl_last[k].sys = sys;
		s_gl_last[k].mk = mk;
		s_gl_last[k].game = game;
	}
}
static int gl_last_get(int mk, const char *sys)
{
	for (int k = 0; k < 32 && s_gl_last[k].sys; k++)
		if (s_gl_last[k].mk == mk && !strcmp(s_gl_last[k].sys, sys))
			return gl_visible(s_gl_last[k].game) ? s_gl_last[k].game : -1;
	return -1;
}

static int cmp_title(const void *a, const void *b)
{
	const int x = *(const int *)a, y = *(const int *)b, c = strcasecmp(gl_title(x), gl_title(y));
	return c ? c : x - y;
}

// a system's slot in s_gl_tabs by its name (the build's hash: a maker with the boards not ported yet has hundreds)
#define SYS_HASH 1024
static short s_sys_hash[SYS_HASH];
static unsigned sys_hash(const char *n)
{
	unsigned h = 2166136261u;
	while (*n)
		h = (h ^ (unsigned char)*n++) * 16777619u;
	return h & (SYS_HASH - 1);
}

// the current system's games in idx, at want or the game last chosen there
static int gl_fill(int *idx, int want, int *sel)
{
	const int m = s_gl_nmk ? s_gl_mk[s_gl_mkc] : -2;
	if (want < 0 && s_gl_ntabs)
		want = gl_last_get(m, s_gl_tabs[s_gl_tab]);
	int n = 0;
	*sel = 0;
	for (int i = 0; i < s_gl_n && s_gl_ntabs; i++) {
		if (!gl_visible(i) || gl_maker(i) != m || strcmp(gl_board(i), s_gl_tabs[s_gl_tab]))
			continue;
		idx[n++] = i;
	}
	// (A-Z by title, also within a system: user 11-12. A sort, not insertion: a system of single-game boards has hundreds;
	// the same order, the list index breaking ties as the stable insertion did)
	qsort(idx, (size_t)n, sizeof(idx[0]), cmp_title);
	for (int k = 0; k < n; k++)
		if (idx[k] == want)
			*sel = k;
	return n;
}

// Rebuild the makers and systems from the games matching the filter and move to the maker / system / position of want
// (list index, may be -1: the maker and system kept). The current system's games go in idx
static int gl_build(int *idx, int want, int *sel)
{
	const int cur_mk = s_gl_mkc < s_gl_nmk ? s_gl_mk[s_gl_mkc] : -2;
	const char *cur = s_gl_tab < s_gl_ntabs ? s_gl_tabs[s_gl_tab] : NULL;
	if (want >= 0 && !gl_visible(want))
		want = -1;
	s_gl_nmk = 0;
	s_gl_nvis = 0;
	for (int i = 0; i < s_gl_n; i++) {
		if (!gl_visible(i))
			continue;
		s_gl_nvis++;
		const int m = gl_maker(i);
		int t = 0;
		while (t < s_gl_nmk && s_gl_mk[t] != m)
			t++;
		if (t == s_gl_nmk && s_gl_nmk < MAX_MAKERS)
			s_gl_mk[s_gl_nmk++] = m;
	}
	for (int a = 1; a < s_gl_nmk; a++)   // (the maker tabs by name, user 11-12)
		for (int b = a; b > 0 && strcasecmp(maker_label(s_gl_mk[b]), maker_label(s_gl_mk[b - 1])) < 0; b--) {
			const int k = s_gl_mk[b];
			s_gl_mk[b] = s_gl_mk[b - 1];
			s_gl_mk[b - 1] = k;
		}
	s_gl_mkc = 0;
	const int mk = want >= 0 ? gl_maker(want) : cur_mk;
	for (int t = 0; t < s_gl_nmk; t++)
		if (s_gl_mk[t] == mk)
			s_gl_mkc = t;
	const int m = s_gl_nmk ? s_gl_mk[s_gl_mkc] : -2;
	s_gl_ntabs = 0;
	// the systems, and their first and latest years, in one pass (it was a pass over every game for each system: 0.1 to
	// 0.35 s a tab change once the boards not ported yet made hundreds of systems, 11-22)
	static int y0s[MAX_TABS], y1s[MAX_TABS];
	memset(s_sys_hash, 0xff, sizeof(s_sys_hash));
	for (int i = 0; i < s_gl_n; i++) {
		if (!gl_visible(i) || gl_maker(i) != m)
			continue;
		const char *bd = gl_board(i);
		unsigned h = sys_hash(bd);
		int t;
		while ((t = s_sys_hash[h]) >= 0 && strcmp(s_gl_tabs[t], bd) != 0)
			h = (h + 1) & (SYS_HASH - 1);
		if (t < 0) {
			if (s_gl_ntabs >= MAX_TABS || s_gl_ntabs >= SYS_HASH / 2)
				continue;
			t = s_gl_ntabs++;
			s_gl_tabs[t] = bd;
			s_sys_hash[h] = (short)t;
			y0s[t] = 9999;
			y1s[t] = 0;
		}
		int y = atoi(s_gl_e[i] && s_gl_e[i]->year ? s_gl_e[i]->year : gl_unported(i) ? "" : rbx_game_year(s_found[i].game));
		if (y > 0 && y < 1000)
			y *= 10;   // ("198?": the 1980s)
		if (y && y < y0s[t]) y0s[t] = y;
		if (y > y1s[t]) y1s[t] = y;
	}
	if (g_settings.list_group == RBX_GROUP_GENRE || g_settings.list_group == RBX_GROUP_PUBLISHER) {   // (names: in order)
		for (int a = 1; a < s_gl_ntabs; a++)
			for (int b = a; b > 0 && strcmp(s_gl_tabs[b], s_gl_tabs[b - 1]) < 0; b--) {
				const char *nm = s_gl_tabs[b];
				s_gl_tabs[b] = s_gl_tabs[b - 1];
				s_gl_tabs[b - 1] = nm;
			}
	} else {	// the systems newest first: by the year their first game came out (the latest game's for a tie)
		static int key[MAX_TABS];
		for (int t = 0; t < s_gl_ntabs; t++) {
			const int y0 = y0s[t], y1 = y1s[t];
			// (the groups of single-game boards, Standalone then Classic, last)
			const int last = !strcmp(s_gl_tabs[t], "Standalone") ? 1 : !strcmp(s_gl_tabs[t], "Classic") ? 2 : 0;
			key[t] = last * 100000000 + (9999 - y0) * 10000 + (9999 - y1);
		}
		for (int a = 1; a < s_gl_ntabs; a++)   // (insertion sort: a few hundred systems at most, once a build)
			for (int b = a; b > 0 && key[b] < key[b - 1]; b--) {
				const int k = key[b];
				key[b] = key[b - 1];
				key[b - 1] = k;
				const char *nm = s_gl_tabs[b];
				s_gl_tabs[b] = s_gl_tabs[b - 1];
				s_gl_tabs[b - 1] = nm;
			}
	}
	s_gl_tab = 0;
	if (gl_all())
		s_gl_col = 1;
	const char *tb = want >= 0 ? gl_board(want) : cur;
	for (int t = 0; tb && t < s_gl_ntabs; t++)
		if (!strcmp(s_gl_tabs[t], tb))
			s_gl_tab = t;
	return gl_fill(idx, want, sel);
}
// another maker tab / system (the system's games, at the game last chosen there)
static int gl_goto(int *idx, int *sel, int mkc, int tab)
{
	if (mkc != s_gl_mkc) {
		s_gl_mkc = mkc;
		s_gl_tab = 0;
		s_gl_ntabs = 0;   // (the new maker's systems: its first)
		return gl_build(idx, -1, sel);
	}
	s_gl_tab = tab;   // (another system of the same maker: the columns stay, only its games are found)
	return gl_fill(idx, -1, sel);
}

// the listed games and which ROM zips are in the ROM folders (start, and Refresh ROM list: ROMs copied while the app
// runs). Returns the games that can run
static int gl_scan(const char *const *dirs, int ndirs)
{
	int present = 0;
	s_gl_n = rbx_games_scan(dirs, ndirs, s_found, RBX_GAMES_MAX, &present);
	if (s_gl_n < 0)
		s_gl_n = 0;
	{	// only the games of the boards that run, and with a list only those it names
		const int listed = rbx_gamelist_load();
		int m = 0;
		for (int i = 0; i < s_gl_n; i++) {
			const rbx_list_entry *e = listed ? rbx_gamelist_find(rbx_core_game_set(s_found[i].game)) : NULL;
			if (!s_found[i].supported || (listed && !e))
				continue;
			s_gl_e[m] = e;
			if (m != i)
				s_found[m] = s_found[i];
			m++;
		}
		s_gl_n = m;
		// the games of the boards not ported yet (user 11-20): every list entry with "port", greyed and never started
		for (int k = 0; listed && k < rbx_gamelist_count() && s_gl_n < RBX_GAMES_MAX; k++) {
			const rbx_list_entry *e = rbx_gamelist_at(k);
			if (!e->port)
				continue;
			memset(&s_found[s_gl_n], 0, sizeof(s_found[0]));
			s_found[s_gl_n].game = -1;
			s_gl_e[s_gl_n++] = e;
		}
	}
	int nplay = 0;
	for (int i = 0; i < s_gl_n; i++)
		nplay += gl_playable(i);
	gl_keys();
	RBX_LOG("games: %d supported, %d with ROM", s_gl_n, nplay);
	return nplay;
}

// one row of the game list (title at x + 48): the selected one with a soft shadow below, a bar and the blinking focus
// outline; favourites get a star; off = how far a long title has scrolled.
// selected: 0 no, 1 the focus (blinks), 2 the focus under an open window (steady outline), 3 the bar only (the focus
// placed by the caller), 4 the bar without its shadow (the focus in the systems column)
static void gl_row(UiCanvas *c, int px, int w, int ty, int g, int selected, int off)
{
	const int okg = gl_playable(g);
	const uint32_t col = selected ? (okg ? COL_TEXT : COL_GREY_SEL) : (okg ? COL_ITEM : COL_GREY);
	if (selected) {
		const int by = ty - 7 - (GL_LINE - 6) / 2; // bar centred on the text
		if (selected != 4) {
			rrect(c, px + 12, by + 3, w - 24, GL_LINE - 6, 6, theme()->shade); // shadow (below only)
			rrect(c, px + 13, by + 5, w - 26, GL_LINE - 6, 6, (theme()->shade & 0xFFFFFFu) | 0x20000000u);
		}
		rrect(c, px + 12, by, w - 24, GL_LINE - 6, 6, COL_HILITE);
		if (selected == 2)
			rrect_outline(c, px + 12, by, w - 24, GL_LINE - 6, 6, 2, COL_FOCUS_STILL);
		else if (selected == 1)
			focus_at(c, px + 12, by, w - 24, GL_LINE - 6);
		// (3: the bar only, the caller places the focus)
	}
	if (rbx_fav_is(gl_set(g)))
		star(c, (float)(px + 36), (float)(ty - 7), 7.0f, COL_STAR);
	ui_text_clip(c, px + 48 - off, ty, col, 1.0f, gl_title(g), px + 48, px + w - 24);
}

#define ROW_H 48      // row texture (games and systems): the bar, its shadow and the text
#define ROW_TY 28     // text baseline in it
// the columns: the systems at the left (SYS_W), the games right of them up to the scroll bar (GL_X .. GL_X + GL_W)
#define SYS_W    232
#define GL_X     (gl_all() ? 0 : SYS_W)
#define GL_W     (SCREEN_W - 16 - GL_X)
#define BAR_X    (SCREEN_W - 16)
// the settings button at the footer's right (touch; START too)
#define GEAR_CX  (SCREEN_W - 32)
#define GEAR_CY  (SCREEN_H - LIST_FOOT_H / 2)
#define GEAR_R   13
// The systems column looks and scrolls as the games do: the same rows (GL_LINE, the bar, its shadow, the font and
// colours), the same fades at its top and bottom, strips the GPU places (sys_tex). s_sys_scr, the first row shown,
// eases to s_sys_tgt, which keeps the current system a row inside the edges unless a touch drag moved the column
#define GL_LPAD 18   // the rows' top padding in both columns (the list's lpad)
static float s_sys_scr = -1, s_sys_tgt;
static int s_sys_drag;
static float s_sys_vel;   // the systems column's flick: rows a frame after the finger left (user 11-20, as the games)
static int s_sys_fling;
// a row's text baseline (the games' formula: GL_LINE rows from la + GL_LPAD)
static int sys_row_ty(int la, int t) { return la + GL_LPAD + (int)floorf(((float)t - s_sys_scr) * GL_LINE + 0.5f) + GL_LINE / 2 + 7; }
static void sys_ease(int la, int lb, int animate)
{
	const float rows_f = (float)(lb - la - 2 * GL_LPAD) / GL_LINE;
	float maxt = (float)s_gl_ntabs - rows_f;
	if (maxt < 0)
		maxt = 0;
	if (!s_sys_drag) {
		if (s_gl_tab < s_sys_tgt + 1)
			s_sys_tgt = (float)s_gl_tab - 1;
		if (s_gl_tab > s_sys_tgt + rows_f - 2)
			s_sys_tgt = (float)s_gl_tab - rows_f + 2;
	}
	if (animate && s_sys_fling && s_sys_vel != 0.0f) {   // (after a flick: it goes on, slowing down, and stops at its ends)
		s_sys_tgt += s_sys_vel;
		s_sys_vel *= 0.95f;
		if (fabsf(s_sys_vel) < 0.015f || s_sys_tgt <= 0.0f || s_sys_tgt >= maxt)
			s_sys_vel = 0;
	}
	if (s_sys_tgt > maxt) s_sys_tgt = maxt;
	if (s_sys_tgt < 0) s_sys_tgt = 0;
	if (s_sys_scr < 0 || !animate || s_sys_drag)
		s_sys_scr = s_sys_tgt;
	else if (s_sys_scr != s_sys_tgt) {
		s_sys_scr += (s_sys_tgt - s_sys_scr) * 0.3f;
		if (fabsf(s_sys_tgt - s_sys_scr) < 0.01f)
			s_sys_scr = s_sys_tgt;
	}
}
// one system row at text baseline ty, drawn as gl_row draws a game: var 0 plain, 1 the current system with the focus
// (bar and shadow), 4 the current system while the games have the focus (the bar without its shadow)
static void sys_row(UiCanvas *c, int ty, int t, int var)
{
	if (var) {
		const int by = ty - 7 - (GL_LINE - 6) / 2, w = SYS_W - 4;
		if (var != 4) {
			rrect(c, 12, by + 3, w - 24, GL_LINE - 6, 6, theme()->shade);
			rrect(c, 13, by + 5, w - 26, GL_LINE - 6, 6, (theme()->shade & 0xFFFFFFu) | 0x20000000u);
		}
		rrect(c, 12, by, w - 24, GL_LINE - 6, 6, COL_HILITE);
	}
	ui_text_clip(c, 28, ty, var ? COL_TEXT : COL_ITEM, 1.0f, s_gl_tabs[t], 28, SYS_W - 16);
}
static int sys_var(int t) { return t != s_gl_tab ? 0 : s_gl_col == 0 ? 1 : 4; }
// the systems column's scroll bar (display only), as the games' one, 5 px wide just left of the line between the columns;
// oy: the canvas's top row on the screen
#define SYS_X_BAR (SYS_W - 10)
static void sys_scrollbar(UiCanvas *c, float rows_f, int la, int lb, int oy)
{
	const int n = s_gl_ntabs, ty0 = la + 8 - oy, th = lb - la - 16;
	if (n <= 0)
		return;
	const float vis = rows_f < (float)n ? rows_f : (float)n;
	int bh = (int)(th * vis / n), by = ty0 + (int)(th * (s_sys_scr < 0 ? 0 : s_sys_scr) / n);
	if (bh < 16) bh = 16;
	if (by + bh > ty0 + th) by = ty0 + th - bh;
	rrect(c, 0, ty0, 5, th, 2, UI_RGBA(0xFF, 0xFF, 0xFF, 0x18));
	rrect(c, 0, by, 5, bh, 2, COL_THUMB);
}
// the focus's bar of a system row (the focus layer goes there)
static void sys_bar(int ty, int *by, int *w) { *by = ty - 7 - (GL_LINE - 6) / 2; *w = SYS_W - 4 - 24; }
// the column drawn in the list's canvas (a window open over the list: no strips), or only its divider (strips)
static void gl_systems(UiCanvas *c, int la, int lb, int rows_too)
{
	if (gl_all())
		return;
	ui_rect(c, SYS_W - 2, la + 8, 1, lb - la - 16, UI_RGBA(0xFF, 0xFF, 0xFF, 0x18));   // (the line between the columns)
	if (!rows_too)
		return;
	sys_ease(la, lb, 0);
	for (int t = 0; t < s_gl_ntabs; t++) {
		const int ty = sys_row_ty(la, t);
		int by, w;
		sys_bar(ty, &by, &w);
		if (by < la || by + GL_LINE > lb)
			continue;
		sys_row(c, ty, t, sys_var(t));
		if (t == s_gl_tab && s_gl_col == 0)
			rrect_outline(c, 12, by, w, GL_LINE - 6, 6, 2, COL_FOCUS_STILL);
	}
}
// a system row's texture (cached as the game rows')
#define SYS_CACHE 24
static struct { UiCanvas *cv; const char *name; int var, gen; unsigned used; } s_sysc[SYS_CACHE];
static unsigned s_sys_frame;
static const ReiTex *sys_tex(int t)
{
	const char *name = s_gl_tabs[t];
	const int var = sys_var(t);
	int k = -1;
	for (int i = 0; i < SYS_CACHE && k < 0; i++)
		if (s_sysc[i].cv && s_sysc[i].name == name && s_sysc[i].var == var && s_sysc[i].gen == s_theme_gen)
			k = i;
	if (k < 0) {
		for (int i = 0; i < SYS_CACHE; i++) {
			if (!s_sysc[i].cv) {
				k = i;
				break;
			}
			if (s_sysc[i].used != s_sys_frame && (k < 0 || s_sysc[i].used < s_sysc[k].used))
				k = i;
		}
		if (k < 0)
			return NULL;
		if (!s_sysc[k].cv)
			s_sysc[k].cv = ui_canvas(-1, 0, 0, SYS_W, ROW_H);
		if (!s_sysc[k].cv)
			return NULL;
		ui_clear(s_sysc[k].cv);
		sys_row(s_sysc[k].cv, ROW_TY, t, var);
		ui_commit(s_sysc[k].cv);
		s_sysc[k].name = name;
		s_sysc[k].var = var;
		s_sysc[k].gen = s_theme_gen;
	}
	s_sysc[k].used = s_sys_frame;
	return ui_tex(s_sysc[k].cv);
}

// The list rows while nothing is open over the list: each row is drawn once into a small texture of its own and the
// GPU places the rows (rbx_video_strips) as the list scrolls. Redrawing and copying the whole 960x544 canvas every
// frame took ~40 ms (the CPU reaches main memory at ~200 MB/s), so the list scrolled at 25 fps
#define ROW_CACHE 16  // more than the rows on screen at once
static struct { UiCanvas *cv; int g, var, off, fav, gen; unsigned used; } s_rowc_w[2][ROW_CACHE];
#define s_rowc s_rowc_w[gl_all()]   // (All: the rows are wider, canvases of their own)
static unsigned s_row_frame;
static const ReiTex *row_tex(int g, int var, int off, int w)
{
	const int fav = rbx_fav_is(gl_set(g)), gen = s_theme_gen;
	int k = -1;
	for (int i = 0; i < ROW_CACHE && k < 0; i++)
		if (s_rowc[i].cv && s_rowc[i].g == g && s_rowc[i].var == var)
			k = i;
	if (k < 0) { // a free entry, else the one shown longest ago (never one shown this frame)
		for (int i = 0; i < ROW_CACHE; i++) {
			if (!s_rowc[i].cv) {
				k = i;
				break;
			}
			if (s_rowc[i].used != s_row_frame && (k < 0 || s_rowc[i].used < s_rowc[k].used))
				k = i;
		}
		if (k < 0)
			return NULL;
		if (!s_rowc[k].cv)
			s_rowc[k].cv = ui_canvas(-1, 0, 0, w, ROW_H);
		if (!s_rowc[k].cv)
			return NULL;
		s_rowc[k].g = g;
		s_rowc[k].var = var;
		s_rowc[k].gen = -1; // (drawn below)
	}
	if (s_rowc[k].off != off || s_rowc[k].fav != fav || s_rowc[k].gen != gen) {
		ui_clear(s_rowc[k].cv);
		gl_row(s_rowc[k].cv, 0, w, ROW_TY, g, var, off);
		ui_commit(s_rowc[k].cv);
		s_rowc[k].off = off;
		s_rowc[k].fav = fav;
		s_rowc[k].gen = gen;
	}
	s_rowc[k].used = s_row_frame;
	return ui_tex(s_rowc[k].cv);
}

// the list's scroll bar (display only), in the 16 px right of the rows; oy: the canvas's top row on the screen
static void gl_scrollbar(UiCanvas *c, int n, float scr, float rows_f, int la, int lb, int ox, int oy)
{
	if (!n)
		return;
	const int tx = SCREEN_W - 14 - ox, ty0 = la + 8 - oy, th = lb - la - 16;
	const float vis = rows_f < (float)n ? rows_f : (float)n;
	int bh = (int)(th * vis / n), by = ty0 + (int)(th * scr / n);
	if (bh < 16) bh = 16;
	if (by + bh > ty0 + th) by = ty0 + th - bh;
	rrect(c, tx, ty0, 5, th, 2, UI_RGBA(0xFF, 0xFF, 0xFF, 0x18));
	rrect(c, tx, by, 5, bh, 2, COL_THUMB);
}

// the list's header: the app name in its colours, the version, "maker : system" (while searching "Search : the
// words"), the position in the list, the filter with its count (2026-11-04, user: the maker and the board in the title)
// the battery in the header (from ReST-V / ReSys12): the charge (0..100) | 0x100 while charging, -1 with none (PS TV, or
// a value out of range); read at most once a second
static int s_bat = -1;
static int bat_poll(void)
{
	static uint64_t t;
	static int read;
	const uint64_t now = rbx_time_us();
	if (read && now - t < 1000000)
		return s_bat;
	read = 1;
	t = now;
	if (re_pad_is_tv()) {
		s_bat = -1;
		return s_bat;
	}
	const int pct = scePowerGetBatteryLifePercent();
	s_bat = pct < 0 || pct > 100 ? -1 : pct | (scePowerIsBatteryCharging() ? 0x100 : 0);
	return s_bat;
}
// the battery icon (body 24 x 13 with its top at y, a cap at the right) filled to the charge, red at 20% or less; a bolt
// over it while charging
static void battery(UiCanvas *c, int x, int y, int pct, int chg)
{
	enum { BW = 24, BH = 13 };
	rrect_outline(c, x, y, BW, BH, 3, 2, COL_DIM);
	rrect(c, x + BW, y + 4, 3, BH - 8, 1, COL_DIM); // (the cap)
	const int iw = BW - 6, fw = pct > 0 ? (iw * pct + 50) / 100 : 0;
	if (fw > 0)
		ui_rect(c, x + 3, y + 3, fw < 1 ? 1 : fw, BH - 6, pct <= 20 ? UI_RGBA(0xF0, 0x48, 0x40, 0xFF) : COL_DIM);
	if (chg) { // a bolt of 7 rows (start, width), with a dark edge so it shows over the fill
		static const signed char bolt[7][2] = { { 4, 2 }, { 3, 2 }, { 2, 2 }, { 1, 5 }, { 3, 2 }, { 2, 2 }, { 1, 2 } };
		const int bx = x + (BW - 7) / 2, by = y + (BH - 7) / 2;
		for (int r = 0; r < 7; r++)
			ui_rect(c, bx + bolt[r][0] - 1, by + r, bolt[r][1] + 2, 1, UI_RGBA(0x00, 0x00, 0x00, 0xA0));
		for (int r = 0; r < 7; r++)
			ui_rect(c, bx + bolt[r][0], by + r, bolt[r][1], 1, COL_STAR);
	}
}
static void gl_header(UiCanvas *c, int n, int sel)
{
	char title[160];
	const int tx = 24 + draw_brand(c, 24, 32, 1.1f);   // (user 11-22: the text in the middle of the header band)
	const char *mk = s_gl_nmk ? maker_label(s_gl_mk[s_gl_mkc]) : NULL;
	const char *bd = s_gl_ntabs ? s_gl_tabs[s_gl_tab] : NULL;
	if (s_search[0])
		snprintf(title, sizeof(title), " (" RBX_VERSION ") - Search : %s (%d/%d)", s_search, n ? sel + 1 : 0, n);
	else if (gl_all())
		snprintf(title, sizeof(title), " (" RBX_VERSION ") - All games (%d/%d)", n ? sel + 1 : 0, n);
	else if (mk && bd && !strncasecmp(bd, mk, strlen(mk)))   // (a tab named with its maker: "Seta 1", once)
		snprintf(title, sizeof(title), " (" RBX_VERSION ") - %s (%d/%d)", bd, n ? sel + 1 : 0, n);
	else if (mk && bd)
		snprintf(title, sizeof(title), " (" RBX_VERSION ") - %s : %s (%d/%d)", mk, bd, n ? sel + 1 : 0, n);
	else
		snprintf(title, sizeof(title), " (" RBX_VERSION ") (%d/%d)", n ? sel + 1 : 0, n);
	// (as ReST-V / ReSys12: the battery at the right; the filter is in its footer pill now)
	int hx = SCREEN_W - 24;
	if (s_bat >= 0) {
		char pc[8];
		snprintf(pc, sizeof(pc), "%d%%", s_bat & 0xFF);
		hx -= ui_text_width(0.85f, pc);
		ui_text(c, hx, 30, COL_DIM, 0.85f, pc);
		hx -= 8 + 3 + 24;
		battery(c, hx, 18, s_bat & 0xFF, s_bat >> 8);
	}
	ui_text_clip(c, tx, 32, COL_TEXT, 1.1f, title, tx, hx - 16);
}

// ---- update window over the game list (Update / Close / Skip this version) ----
#define UPD_LINES 400
#define UPD_W 840
#define UPD_H 470
#define UPD_TEXT_W (UPD_W - 64)
#define UPD_VIS 13
static char s_upd_line[UPD_LINES][160];
static int s_upd_n, s_upd_scroll, s_upd_btn;
static void upd_wrap(const char *notes) // the notes as lines of the window's width (word wrap)
{
	s_upd_n = 0;
	const char *p = notes;
	while (*p && s_upd_n < UPD_LINES) {
		const char *e = strchr(p, '\n');
		const size_t len = e ? (size_t)(e - p) : strlen(p);
		char para[1024];
		snprintf(para, sizeof(para), "%.*s", (int)(len < sizeof(para) - 1 ? len : sizeof(para) - 1), p);
		if (!para[0])
			s_upd_line[s_upd_n++][0] = 0;
		for (char *q = para; *q && s_upd_n < UPD_LINES;) {
			char line[160];
			int take = 0, last_sp = -1;
			while (q[take] && take < (int)sizeof(line) - 1) {
				line[take] = q[take];
				line[take + 1] = 0;
				if (q[take] == ' ')
					last_sp = take;
				if (ui_text_width(0.8f, line) > UPD_TEXT_W - (para[0] == '#' ? 0 : 16)) {
					if (last_sp > 0)
						take = last_sp;
					break;
				}
				take++;
			}
			snprintf(s_upd_line[s_upd_n++], sizeof(s_upd_line[0]), "%s%.*s", q == para || para[0] == '#' ? "" : "  ", take, q);
			q += take;
			while (*q == ' ')
				q++;
		}
		p = e ? e + 1 : p + len;
	}
}
static void upd_draw(UiCanvas *c, int o_ok)
{
	const int mx = (SCREEN_W - UPD_W) / 2, my = (SCREEN_H - UPD_H) / 2;
	window_box(c, mx, my, UPD_W, UPD_H, COL_SEL, COL_PANEL);
	char t[96];
	snprintf(t, sizeof(t), T("Update available: %s  (this version %s)"), rbx_update_version(), RBX_VERSION);
	ui_text(c, mx + 24, my + 40, COL_TEXT, 1.0f, t);
	for (int i = 0; i < UPD_VIS && s_upd_scroll + i < s_upd_n; i++) {
		const char *l = s_upd_line[s_upd_scroll + i];
		const int head = l[0] == '#';
		ui_text(c, mx + 32, my + 80 + i * 24, head ? COL_SEL : COL_TEXT, head ? 0.9f : 0.8f, l + head);
	}
	if (s_upd_n > UPD_VIS) { // scroll bar
		const int th = UPD_VIS * 24, ty = my + 62;
		int bh = th * UPD_VIS / s_upd_n, by = ty + th * s_upd_scroll / s_upd_n;
		if (bh < 16) bh = 16;
		rrect(c, mx + UPD_W - 18, ty, 5, th, 2, UI_RGBA(0xFF, 0xFF, 0xFF, 0x18));
		rrect(c, mx + UPD_W - 18, by, 5, bh, 2, COL_THUMB);
	}
	static const char *const names[3] = { "Update", "Close", "Skip this version" };
	int bx = mx + 24;
	for (int i = 0; i < 3; i++) {
		const int bw = ui_text_width(0.9f, names[i]) + 40, by = my + UPD_H - 74;
		rrect(c, bx, by, bw, 36, 6, i == s_upd_btn ? COL_SEL : UI_RGBA(0xFF, 0xFF, 0xFF, 0x20));
		ui_text(c, bx + 20, by + 25, i == s_upd_btn ? COL_PANEL : COL_TEXT, 0.9f, names[i]);
		bx += bw + 16;
	}
	text_keys(c, mx + 24, my + UPD_H - 16, COL_DIM,
		o_ok ? "Up/Down: notes   Left/Right: choose   " G_CIR ": OK   " G_CRO ": close"
		     : "Up/Down: notes   Left/Right: choose   " G_CRO ": OK   " G_CIR ": close");
}
static void upd_progress(const char *stage, int pct) // (called while the update downloads)
{
	UiCanvas *c = s_cv;
	const int mw = 760, mh = 140, mx = (SCREEN_W - mw) / 2, my = (SCREEN_H - mh) / 2;
	// the screen under the window is kept and put back first (the window's shadow would get darker with every redraw)
	static uint32_t *under;
	if (!under) {
		under = (uint32_t *)malloc((size_t)SCREEN_W * SCREEN_H * 4);
		if (under)
			memcpy(under, ui_pixels(c), (size_t)SCREEN_W * SCREEN_H * 4);
	} else
		ui_blit(c, under);
	window_box(c, mx, my, mw, mh, COL_SEL, COL_PANEL);
	char t[96];
	if (pct == -2)
		snprintf(t, sizeof(t), "%s", stage);
	else if (pct >= 0)
		snprintf(t, sizeof(t), "%s %s  %d%%", stage, rbx_update_version(), pct);
	else
		snprintf(t, sizeof(t), "%s %s ...", stage, rbx_update_version());
	ui_text(c, mx + 24, my + 46, COL_TEXT, pct == -2 ? 0.9f : 1.0f, t);
	if (pct != -2)
		rrect(c, mx + 24, my + 74, mw - 48, 14, 6, UI_RGBA(0xFF, 0xFF, 0xFF, 0x20));
	if (pct > 0)
		rrect(c, mx + 24, my + 74, (mw - 48) * (pct > 100 ? 100 : pct) / 100, 14, 6, COL_SEL);
	ui_text(c, mx + 24, my + mh - 20, COL_DIM, 0.8f, "Do not turn the Vita off");
	ui_commit(c);
	rbx_video_present();
}

// ---- the game's Info (user 11-22): the list's rows hidden, the game's list entry faded in over them; the description,
// when longer than its space, moves up by the part not on screen (a pause at each end), then starts again
#define INFO_LINES 400
static char s_info_line[INFO_LINES][200];
static int s_info_n;
#define INFO_DESC_SC 0.85f
#define INFO_LINE_H 26
static void info_wrap(const char *text, int width)
{
	s_info_n = 0;
	for (const char *p = text; p && *p && s_info_n < INFO_LINES;) {
		const char *e = strchr(p, '\n');
		const size_t len = e ? (size_t)(e - p) : strlen(p);
		static char para[8192];
		snprintf(para, sizeof(para), "%.*s", (int)(len < sizeof(para) - 1 ? len : sizeof(para) - 1), p);
		if (!para[0])
			s_info_line[s_info_n++][0] = 0;
		for (char *q = para; *q && s_info_n < INFO_LINES;) {
			char line[200];
			int take = 0, last_sp = -1;
			while (q[take] && take < (int)sizeof(line) - 1) {
				line[take] = q[take];
				line[take + 1] = 0;
				if (q[take] == ' ')
					last_sp = take;
				if (ui_text_width(INFO_DESC_SC, line) > width) {
					if (last_sp > 0)
						take = last_sp;
					else
						while (take > 1 && ((unsigned char)q[take] & 0xC0) == 0x80)   // (not inside a UTF-8 letter)
							take--;
					break;
				}
				take++;
			}
			if (take < 1)
				take = 1;
			snprintf(s_info_line[s_info_n++], sizeof(s_info_line[0]), "%.*s", take, q);
			q += take;
			while (*q == ' ')
				q++;
		}
		p = e ? e + 1 : p + len;
	}
}

// ---- the focused row (user 11-22): resting on a game it grows to two lines (title; publisher and year), and when the
// game has a snap, to three with the snap at its left (the text pushed right), where the snap then plays
#define FOC_EXT2 30   // the bar's growth for two lines (px)
#define FOC_EXT3 76   // for three: three times the bar's height (38 -> 114)
static const char *gl_pub(int g)
{
	const rbx_list_entry *e = s_gl_e[g];
	return e && e->publisher ? e->publisher : gl_unported(g) ? "" : pub_name(s_found[g].game);
}
static const char *gl_board_name(int g)
{
	const rbx_list_entry *e = s_gl_e[g];
	return e && e->board ? e->board : gl_unported(g) ? "" : board_name(s_found[g].game);
}
static const char *gl_year(int g)
{
	const rbx_list_entry *e = s_gl_e[g];
	return e && e->year ? e->year : gl_unported(g) ? "" : rbx_game_year(s_found[g].game);
}
// the snap's box in a bar of growth ext (px, from the bar's left and top)
static void foc_snap_box(int ext, int *x, int *y, int *w, int *h)
{
	const int bh = GL_LINE - 6 + ext;
	*h = bh - 16;
	*w = *h * 4 / 3;
	*x = 12 + 8;
	*y = 8;
}
// a row as gl_row, its bar grown by ext px with the second line; shift (0..1) moves the text right of the snap's box
static void card_draw(UiCanvas *c, int px, int w, int ty, int g, int selected, int off, int ext, float shift)
{
	const int okg = gl_playable(g);
	const uint32_t col = okg ? COL_TEXT : COL_GREY_SEL;
	const int by = ty - 7 - (GL_LINE - 6) / 2, bh = GL_LINE - 6 + ext;
	if (selected != 4) {
		rrect(c, px + 12, by + 3, w - 24, bh, 6, theme()->shade);
		rrect(c, px + 13, by + 5, w - 26, bh, 6, (theme()->shade & 0xFFFFFFu) | 0x20000000u);
	}
	rrect(c, px + 12, by, w - 24, bh, 6, COL_HILITE);
	if (selected == 2)
		rrect_outline(c, px + 12, by, w - 24, bh, 6, 2, COL_FOCUS_STILL);
	else if (selected == 1)
		focus_at(c, px + 12, by, w - 24, bh);
	int sx, sy, sw, sh;
	foc_snap_box(FOC_EXT3, &sx, &sy, &sw, &sh);
	// (pushed past the snap's box: 14 px after its border; a favourite's star between them, 10 px either side of it,
	// only for a favourite: user 11-22)
	const int fav = rbx_fav_is(gl_set(g)), edge = sx + sw + 2;
	const int tx_to = fav ? edge + 10 + 14 + 10 : edge + 14, star_to = edge + 10 + 7;
	const int tx = px + 48 + (int)(shift * (float)(tx_to - 48) + 0.5f);
	const float star_x = (float)px + 36.0f + shift * (float)(star_to - 36);
	// (three lines: the two lines of text in the bar's middle)
	const int tyc = ty + (ext > FOC_EXT2 ? (ext - FOC_EXT2) / 2 : 0);
	if (fav)
		star(c, star_x, (float)(tyc - 7), 7.0f, COL_STAR);
	ui_text_clip(c, tx - off, tyc, col, 1.0f, gl_title(g), tx, px + w - 24);
	if (ext > 4) {   // (the second line comes in with the growth: publisher, year, board)
		const unsigned al = ext >= FOC_EXT2 ? 255u : (unsigned)(ext * 255 / FOC_EXT2);
		char l2[200];
		const char *pb = gl_pub(g), *yr = gl_year(g), *bd = gl_board_name(g);
		snprintf(l2, sizeof(l2), "%s%s%s%s%s", pb, pb[0] && yr[0] ? "   " : "", yr, (pb[0] || yr[0]) && bd[0] ? "   " : "", bd);
		ui_text_clip(c, tx, tyc + 30, (COL_DIM & 0xFFFFFFu) | (al << 24), 0.85f, l2, tx, px + w - 24);
	}
}
// the focused row's texture for the strips (one, redrawn when it changes)
static const ReiTex *card_tex(int g, int var, int off, int w, int ext, float shift)
{
	static UiCanvas *cv;
	static int kg = -1, kvar, koff, kext, kshift, kgen, kfav, kw;
	const int sh = (int)(shift * 100.0f + 0.5f), fav = rbx_fav_is(gl_set(g));
	if (!cv || kw != w) {
		cv = ui_canvas(-1, 0, 0, w, ROW_H + FOC_EXT3 + 8);
		kw = w;
		kg = -1;
	}
	if (!cv)
		return NULL;
	if (kg != g || kvar != var || koff != off || kext != ext || kshift != sh || kgen != s_theme_gen || kfav != fav) {
		ui_clear(cv);
		card_draw(cv, 0, w, ROW_TY, g, var, off, ext, shift);
		ui_commit(cv);
		kg = g, kvar = var, koff = off, kext = ext, kshift = sh, kgen = s_theme_gen, kfav = fav;
	}
	return ui_tex(cv);
}

// ---- the Info (user 11-22): the rows, the maker tabs and the systems fade out, the game's entry fades in over most of
// the screen: the snap (moved from the focused row, growing) with the title, year, publisher and ROM file under it at
// the left, the description at the right (moving up by its part not on screen)
#define INFO_SX (SCREEN_W - 16 - INFO_SW)   // (user 11-22: the snap at the right, the text at the left)
#define INFO_SY (TITLE_H + 10)
#define INFO_SW 500
#define INFO_SH (SCREEN_H - LIST_FOOT_H - 10 - INFO_SY)
#define INFO_DX 24
#define INFO_DX1 (INFO_SX - 24)   // the text's right edge
static UiCanvas *s_info_cv;   // the Info's texture (made with the list: making it as the Info opened cost a frame)
static int s_info_top;   // the description's top (below the title, year, publisher, board and ROM file)
static int info_desc_top(void) { return s_info_top ? s_info_top : INFO_SY; }
static void info_draw(UiCanvas *c, int g, int lb, int t, float scroll)
{
	const unsigned al = t >= 15 ? 255u : (unsigned)(t * 255 / 15);
	const uint32_t ct = (COL_TEXT & 0xFFFFFFu) | (al << 24), cd = (COL_DIM & 0xFFFFFFu) | (al << 24);
	const int x = INFO_DX, x1 = INFO_DX1;
	int y = INFO_SY + 26;
	{	// the title over as many lines as it needs (user 11-22), at the spaces where it can
		const char *tt = gl_title(g);
		for (int ln = 0; *tt && ln < 4; ln++) {
			char line[160];
			int take = 0, last_sp = -1;
			while (tt[take] && take < (int)sizeof(line) - 1) {
				line[take] = tt[take];
				line[take + 1] = 0;
				if (tt[take] == ' ')
					last_sp = take;
				if (ui_text_width(1.0f, line) > x1 - x) {
					if (last_sp > 0)
						take = last_sp;
					else
						while (take > 1 && ((unsigned char)tt[take] & 0xC0) == 0x80)
							take--;
					break;
				}
				take++;
			}
			if (take < 1)
				take = 1;
			snprintf(line, sizeof(line), "%.*s", take, tt);
			ui_text_clip(c, x, y, ct, 1.0f, line, x, x1);
			y += 26;
			tt += take;
			while (*tt == ' ')
				tt++;
		}
		y += 4;
	}
	const char *yr = gl_year(g), *pb = gl_pub(g), *bd = gl_board_name(g);
	if (yr[0]) { ui_text_clip(c, x, y, ct, 0.9f, yr, x, x1); y += 26; }
	if (pb[0]) { ui_text_clip(c, x, y, ct, 0.9f, pb, x, x1); y += 26; }
	if (bd[0]) { ui_text_clip(c, x, y, ct, 0.9f, bd, x, x1); y += 26; }
	{
		const rbx_list_entry *e = s_gl_e[g];
		char rf[160];
		if (e && e->rompath) {
			const char *b = e->rompath;
			for (const char *q = e->rompath; *q; q++)
				if (*q == '/' || *q == ':')
					b = q + 1;
			snprintf(rf, sizeof(rf), "%s", b);
		} else
			snprintf(rf, sizeof(rf), "%s.zip", gl_set(g));
		ui_text_clip(c, x, y, cd, 0.85f, rf, x, x1);
		y += 14;
	}
	s_info_top = y;
	const int top = info_desc_top(), bot = lb - 8;
	if (!s_info_n)
		return;
	ui_clip_y(c, top, bot);
	const int off = (int)floorf(scroll + 0.5f);
	for (int i = 0; i < s_info_n; i++) {
		const int ly = top + 18 + i * INFO_LINE_H - off;
		if (ly < top - INFO_LINE_H || ly > bot + INFO_LINE_H)
			continue;
		ui_text_clip(c, INFO_DX, ly, ct, INFO_DESC_SC, s_info_line[i], INFO_DX, x1);
	}
	ui_clip_y(c, 0, SCREEN_H);
}
// the description's part below its space (px; 0: it fits)
static int info_overflow(int lb)
{
	const int space = (lb - 8) - info_desc_top(), need = 18 + s_info_n * INFO_LINE_H;
	return need > space ? need - space : 0;
}

const rbx_game_found *rbx_game_select(const char *const *dirs, int ndirs, const char *rom_dir, const char *notice)
{
	ensure_canvases();
	UiCanvas *c = s_cv;
	int want = -1;
	// the ROM folders are read at the first list only (~0.3 s for 700 zips on the device: the app stopped that long on
	// each Select game). ROMs copied later: Refresh ROM list
	static int scanned, nplay_kept;
	if (!scanned || !s_gl_n) {
		nplay_kept = gl_scan(dirs, ndirs);
		scanned = 1;
	}
	const int nplay = nplay_kept;
	for (int i = 0; i < s_gl_n; i++)
		if (g_settings.last_game[0] && !strcmp(g_settings.last_game, gl_set(i)))
			want = i;
	for (int i = 0; i < s_gl_n && want < 0; i++) // (no last game: the first game with a ROM)
		if (gl_playable(i))
			want = i;
	int auto_all = 0; // the filter went to All only because no game could run
	if (!nplay && g_settings.list_filter == RBX_LIST_PLAYABLE) {
		g_settings.list_filter = RBX_LIST_ALL; // (show everything if no game can run)
		auto_all = 1;
	}
	if (want >= 0 && gl_playable(want) && !gl_visible(want))
		g_settings.list_filter = RBX_LIST_PLAYABLE; // (the cursor goes to the last game, so it must be on screen)
	static int idx[RBX_GAMES_MAX];
	int sel = 0, dirty = 1, upd = 0, group0 = g_settings.list_group;
	int info_on = 0, info_g = -1, info_t = 0;   // the Info shown: its game, frames since it opened
	float info_scr = 0.0f, info_from[4] = { 0 };   // (the description's scroll; the snap's box it grows from)
	int foc_g = -1, foc_t = 0, foc_req = 0;      // the focused row: its game, frames it rested there, its snap asked for
	float foc_ext = 0.0f, foc_shift = 0.0f, foc_snap = 0.0f;   // its growth (px), the text's push right, the snap's fade
	int foc_box[4] = { 0 };   // (the snap's box on the screen this frame, for the Info to grow from)
	rbx_video_info_overlay(NULL, 0, 0, 0.0f);
				rbx_video_list_hide(0.0f, 0, 0);
				rbx_video_snap_box(0, 0, 0, 0, 0, 0, 0, 0);   // (closed: the snap back to its row at once, user 11-22; a search
				                                             // dialog opened next drew it where the Info had it)
	if (!s_info_cv)
		s_info_cv = ui_canvas(-1, 0, 0, SCREEN_W, SCREEN_H);
	rbx_video_set_ui_hide(0.0f);
	static int upd_offered; // (once per run)
	uint64_t rep = 0;
	float scr = -1, tgt = 0; // list scroll (rows): drawn position easing to the target
	// touch: down at (t_x0, t_y0), last y, moved past a tap, began over the games; drag: the games scrolled by touch
	// (the cursor follows the rows on screen), vel: rows a frame after the finger left
	int t_down = 0, t_x0 = 0, t_y0 = 0, t_last = 0, t_moved = 0, t_list = 0, t_sys = 0, t_idle = 0, t_stop = 0, drag = 0;
	int t_tab = 0, t_lastx = 0;   // (the touch began on the maker tab row; its last x)
	float vel = 0;
	static uint32_t *under; // the dimmed list under the options window
	int under_ok = 0;
	int mq_sel = -1, mq_t = 0, mq_tw = 0; // scrolling title of the selected row (marquee)
	int fast_ok = 0, fast_key[14] = { 0 }, hdr_sel = -1, hdr_bat = -2, bar_n = -1, sync_left = 0; // rows as strips
	float bar_scr = -1;
	s_gl_ntabs = s_gl_tab = 0;
	s_list_mode = 0;
	int n = gl_build(idx, want, &sel);
	ui_clear(c); // whole canvas the first time (what loading/menu left there)
	char msg[256] = { 0 };
	if (notice)
		snprintf(msg, sizeof(msg), "%s", notice);
	uint32_t prev = 0xffffffffu; // ignore until released once, so the press that launched the app does not pick right away
	for (;;) {
		rbx_video_list_neon(1);   // (the list's background: the neon corridor, user 11-22; its moving parts on CPU1)
		rbx_neon_kick(NEON_LIST_Y, NEON_LIST_H, rbx_menu_accent());   // (the corridor turned: walls at the top and bottom)
#ifdef RBX_ENABLE_LOG
		const uint64_t lf_t0 = rbx_time_us();   // (a slow frame's parts, logged)
		const int lf_sel = sel;
#endif
#ifdef RBX_ENABLE_LOG
		{	// (log builds, the remote device tests: the list keeps the Vita awake as a game does. Left idle on the list
			// between tests the system went to standby, the screen off, and the remote tools lost it, 2026-11-02. The
			// release build leaves the list to the system's power saving)
			static unsigned tick;
			if (++tick % 30 == 0)
				sceKernelPowerTick(SCE_KERNEL_POWER_TICK_DEFAULT);
		}
#endif
		SceCtrlData pad;
		re_pad_poll();
		re_pad_read_all(&pad);
		uint32_t b = pad.buttons | rbx_testpad_step(); // (log builds: the test input script)
		if (pad.ly < 128 - 48) b |= SCE_CTRL_UP;
		if (pad.ly > 128 + 48) b |= SCE_CTRL_DOWN;
		if (pad.lx < 128 - 48) b |= SCE_CTRL_LEFT;
		if (pad.lx > 128 + 48) b |= SCE_CTRL_RIGHT;
		if (!s_list_mode && ps_poll() && !upd) { // the PS button opens the options here too (rbx_menu_update closes them)
			msg[0] = 0;
			rbx_settings_use_global();
			s_list_mode = 1;
			s_page = PAGE_MAIN;
			s_sel = s_main_sel = 0;
			s_pop = 0;
			s_changed = 0;
			s_prev_buttons = b;
			dirty = 1;
		}
		uint32_t pressed = key_repeat(b, b & ~prev, &rep);
		prev = b;
		if (pressed)
			dirty = 1;
		const int o_ok = g_settings.menu_confirm == RBX_CONFIRM_CIRCLE;
		const uint32_t ok = o_ok ? SCE_CTRL_CIRCLE : SCE_CTRL_CROSS, back = o_ok ? SCE_CTRL_CROSS : SCE_CTRL_CIRCLE;
		{	// touch: a tap picks a maker tab, a system, a game (tapped again: it starts) or the settings button, or closes a
			// notice; a drag over the games scrolls them (and goes on a little after the finger leaves)
			int tx[6], ty[6];
			const int tn = touch_read(tx, ty);
			touch_fx(tn, tx[0], ty[0]);
			const int la_t = TITLE_H + (s_gl_nmk > 1 ? 40 : 0), lb_t = SCREEN_H - LIST_FOOT_H, idle = !upd && !s_list_mode && !msg[0] && !info_on;
			if (tn && !t_down) {
				t_down = 1;
				t_x0 = tx[0];
				t_y0 = t_last = ty[0];
				t_moved = 0;
				t_stop = (drag && vel != 0.0f) || s_tab_vel != 0.0f || s_sys_vel != 0.0f; // (a touch that stops a glide is not a tap)
				vel = 0;
				s_tab_vel = 0;
				s_sys_vel = 0;
				s_sys_fling = 0;
				t_idle = idle; // (a finger that closed the options is not a tap on the list)
				t_list = idle && tx[0] >= GL_X && ty[0] >= la_t && ty[0] < lb_t;
				t_tab = idle && s_gl_nmk > 1 && ty[0] >= TITLE_H && ty[0] < la_t;
				t_lastx = tx[0];
				t_sys = idle && !gl_all() && tx[0] < SYS_W && ty[0] >= la_t && ty[0] < lb_t;
			} else if (tn) {
				if (abs(ty[0] - t_y0) > 12 || abs(tx[0] - t_x0) > 12)
					t_moved = 1;
				if (t_moved && t_list && n) {
					const float d = (float)(ty[0] - t_last) / GL_LINE;
					tgt -= d;
					if (!s_touch_held) // (the finger's speed over its last frames)
						vel = vel * 0.5f - d * 0.5f;
					drag = 1;
				}
				if (t_moved && t_tab) {   // (the tab row follows the finger, within its ends)
					const float dx = (float)(tx[0] - t_lastx);
					s_tab_tgt -= dx;
					if (!s_touch_held) // (the finger's speed over its last frames)
						s_tab_vel = s_tab_vel * 0.5f - dx * 0.5f;
					if (s_tab_tgt > s_tab_max) s_tab_tgt = s_tab_max;
					if (s_tab_tgt < 0.0f) s_tab_tgt = 0.0f;
					s_tab_cur = s_tab_tgt;
				}
				t_lastx = tx[0];
				if (t_moved && t_sys) {   // (the systems column moves with the finger; the current system is kept)
					const float sd = (float)(ty[0] - t_last) / GL_LINE;
					s_sys_tgt -= sd;
					if (!s_touch_held) // (the finger's speed over its last frames)
						s_sys_vel = s_sys_vel * 0.5f - sd * 0.5f;
					s_sys_drag = 1;
				}
				t_last = ty[0];
			} else if (t_down) {
				t_down = 0;
				if (t_moved && drag)
					vel = flick_boost(vel);
				if (t_moved && t_tab)   // (in rows of the list's height: the same boost as its flicks)
					s_tab_vel = flick_boost(s_tab_vel / GL_LINE) * GL_LINE;
				else
					s_tab_vel = 0;
				if (t_moved && t_sys) {
					s_sys_vel = flick_boost(s_sys_vel);
					s_sys_fling = 1;
				} else
					s_sys_vel = 0;
				if (!t_moved && t_stop)
					; // (it only stopped the list)
				else if (!t_moved && msg[0])
					pressed |= ok;   // (closes it)
				else if (!t_moved && info_on && t_y0 >= lb_t) {   // (the Info's footer buttons and the settings button)
					const int gdx = t_x0 - GEAR_CX, gdy = t_y0 - GEAR_CY;
					if (gdx * gdx + gdy * gdy < 30 * 30)
						pressed |= SCE_CTRL_START;
					for (int k = 0; k < s_ft_n; k++)
						if (t_x0 >= s_ft_x0[k] && t_x0 < s_ft_x1[k])
							pressed |= s_ft_key[k];
				} else if (!t_moved && idle && t_idle) {
					const int x = t_x0, y = t_y0;
					const int gdx = x - GEAR_CX, gdy = y - GEAR_CY;
					if (gdx * gdx + gdy * gdy < 30 * 30)
						pressed |= SCE_CTRL_START;
					else if (y < TITLE_H)
						; // (the header: the battery now; the filter is the footer's SELECT pill)
					else if (y >= lb_t) { // the footer's buttons
						for (int k = 0; k < s_ft_n; k++)
							if (x >= s_ft_x0[k] && x < s_ft_x1[k])
								pressed |= s_ft_key[k];
					} else if (y < la_t) {   // a maker tab
						for (int t = 0; t < s_gl_nmk; t++)
							if (x >= s_tab_x0[t] && x < s_tab_x1[t] && t != s_gl_mkc) {
								if (n)
									gl_last_set(idx[sel]);
								n = gl_goto(idx, &sel, t, 0);
								scr = -1;
							}
					} else if (y >= la_t && y < lb_t && x < SYS_W && !gl_all()) {   // a system
						const int t = (int)floorf((float)(y - la_t - GL_LPAD) / GL_LINE + s_sys_scr);
						if (t >= 0 && t < s_gl_ntabs) {
							if (t != s_gl_tab) {
								if (n)
									gl_last_set(idx[sel]);
								n = gl_goto(idx, &sel, s_gl_mkc, t);
								scr = -1;
							}
							s_gl_col = 0;
						}
					} else if (y >= la_t && y < lb_t && n) {   // a game
						// (the rows under the focused one sit lower by its growth)
						const int fx = (int)(foc_ext + 0.5f), ytop = la_t + 18 + (int)floorf(((float)sel - scr) * GL_LINE + 0.5f);
						int i = (int)floorf((float)(y - la_t - 18) / GL_LINE + scr);
						if (i > sel)
							i = y < ytop + GL_LINE + fx ? sel : (int)floorf((float)(y - la_t - 18 - fx) / GL_LINE + scr);
						if (i >= 0 && i < n) {
							if (x < GL_X + 44)
								pressed |= SCE_CTRL_SQUARE; // (its star)
							else if (i == sel && s_gl_col == 1)
								pressed |= ok;
							sel = i;
							s_gl_col = 1;
							drag = 0;
						}
					}
				}
			}
			if (tn || t_down)
				dirty = 1;
		}
		{	// the Info (user 11-22): the footer's button opens it; up / down another game, confirm starts it, cancel
			// closes it, search and favourite as in the list (the other keys do nothing under it)
			const int cur = n ? idx[sel] : -1;
			const int can = !upd && !s_list_mode && !msg[0];
			if (info_on && !can) {
				info_on = 0;
				rbx_video_info_overlay(NULL, 0, 0, 0.0f);
				rbx_video_list_hide(0.0f, 0, 0);
				rbx_video_snap_box(0, 0, 0, 0, 0, 0, 0, 0);   // (closed: the snap back to its row at once, user 11-22; a search
				                                             // dialog opened next drew it where the Info had it)
			} else if (info_on) {
				if (pressed & back) {
					info_on = 0;
					pressed = 0;
					rbx_video_info_overlay(NULL, 0, 0, 0.0f);
				rbx_video_list_hide(0.0f, 0, 0);
				rbx_video_snap_box(0, 0, 0, 0, 0, 0, 0, 0);   // (closed: the snap back to its row at once, user 11-22; a search
				                                             // dialog opened next drew it where the Info had it)
				} else if (pressed & (SCE_CTRL_UP | SCE_CTRL_DOWN | SCE_CTRL_LEFT | SCE_CTRL_RIGHT)) {   // (left / right too)
					const int ns = sel + ((pressed & (SCE_CTRL_UP | SCE_CTRL_LEFT)) ? -1 : 1);
					if (n && ns >= 0 && ns < n && ns != sel) {
						sel = ns;
						info_g = idx[sel];
						info_scr = -1.0f;   // (drawn again below)
						const rbx_list_entry *e = s_gl_e[info_g];
						info_wrap(e ? e->description : NULL, INFO_DX1 - INFO_DX);
						rbx_video_set_preview(gl_set(info_g), gl_unported(info_g) ? NULL : rbx_core_game_parent(s_found[info_g].game));
						foc_req = 1;
						if (info_t > 24)
							info_t = 24;   // (the description's scroll from its start; the snap stays in its place)
					}
					pressed = 0;
				} else if (pressed & (ok | SCE_CTRL_TRIANGLE)) {   // (the game started, or the search: as in the list)
					info_on = 0;
					rbx_video_info_overlay(NULL, 0, 0, 0.0f);
				rbx_video_list_hide(0.0f, 0, 0);
				rbx_video_snap_box(0, 0, 0, 0, 0, 0, 0, 0);   // (closed: the snap back to its row at once, user 11-22; a search
				                                             // dialog opened next drew it where the Info had it)
					pressed &= ok | SCE_CTRL_TRIANGLE;
				} else if (pressed & SCE_CTRL_START) {   // (the options: as in the list, the Info closed)
					info_on = 0;
					rbx_video_info_overlay(NULL, 0, 0, 0.0f);
					rbx_video_list_hide(0.0f, 0, 0);
					rbx_video_snap_box(0, 0, 0, 0, 0, 0, 0, 0);
					pressed &= SCE_CTRL_START;
				} else
					pressed &= SCE_CTRL_SQUARE;
			} else if (pressed & FT_INFO) {
				pressed &= ~FT_INFO;
				if (cur >= 0 && can) {
					s_gl_col = 1;
					info_on = 1;
					info_g = cur;
					info_t = 0;
					info_scr = -1.0f;   // (drawn below)
					const rbx_list_entry *e = s_gl_e[cur];
					info_wrap(e ? e->description : NULL, INFO_DX1 - INFO_DX);
					// (the snap grows from its box in the row, or from the row's left when it was not shown yet)
					for (int k = 0; k < 4; k++)
						info_from[k] = (float)foc_box[k];
					if (!foc_req) {
						rbx_video_set_preview(gl_set(cur), gl_unported(cur) ? NULL : rbx_core_game_parent(s_found[cur].game));
						foc_req = 1;
					}
				}
			}
			if (!info_on)
				info_g = -1;
			else {
				info_t++;
				// the description's overflow: 3 s still, up at 0.5 px a frame, 3 s still at the end, then from the top
				const int over = info_overflow(SCREEN_H - LIST_FOOT_H);
				const int move = over * 2, cyc = 180 + move + 180;
				const int ph = over > 0 ? (info_t > 20 ? (info_t - 20) % cyc : 0) : -1;
				const float ns = ph < 180 ? 0.0f : ph < 180 + move ? (float)(ph - 180) * 0.5f : (float)over;
				// the Info's own texture: drawn when it opens and when the description moves a pixel; its fade and the
				// snap's growth are the GPU's (the list under it goes on as strips: no full redraw of the canvas)
				static int igen = -1;
				const int lbi = SCREEN_H - LIST_FOOT_H;
				UiCanvas *const icv = s_info_cv;
				if (icv && (info_scr < 0.0f || igen != s_theme_gen ||
					(int)floorf(ns + 0.5f) != (int)floorf(info_scr + 0.5f))) {
					ui_clear(icv);   // (see-through: the neon corridor under it, user 11-22)
					info_draw(icv, info_g, lbi, 1000, ns);
					ui_commit(icv);
					igen = s_theme_gen;
				}
				info_scr = ns;
				const float fa = info_t >= 15 ? 1.0f : (float)info_t / 15.0f;
				rbx_video_info_overlay(icv ? ui_tex(icv) : NULL, TITLE_H, lbi, fa);
				rbx_video_list_hide(fa, TITLE_H, lbi);   // (the rows fade out as it comes)
			}
		}
		if (!upd && !upd_offered && !s_list_mode && !msg[0] && rbx_update_ready()) {
			upd = upd_offered = 1; // a newer release: Update / Close / Skip this version
			upd_wrap(rbx_update_notes());
			s_upd_scroll = s_upd_btn = 0;
			dirty = 1;
		}
		if (upd) {
			extern int rbx_update_auto;
			static int auto_step;
			if (rbx_update_auto && auto_step++ == 2) { // test: the window drawn, saved, then Update
				ui_save_ppm(c, RBX_DATA_DIR "upd.ppm", UI_RGBA(0, 0, 0, 0xFF));
				s_upd_btn = 0;
				pressed |= ok;
			}
			if ((pressed & SCE_CTRL_UP) && s_upd_scroll > 0) s_upd_scroll--;
			if ((pressed & SCE_CTRL_DOWN) && s_upd_scroll + UPD_VIS < s_upd_n) s_upd_scroll++;
			if (pressed & SCE_CTRL_LEFT) s_upd_btn = wrap(s_upd_btn - 1, 3);
			if (pressed & SCE_CTRL_RIGHT) s_upd_btn = wrap(s_upd_btn + 1, 3);
			if (pressed & back)
				upd = 0;
			else if (pressed & ok) {
				upd = 0;
				if (s_upd_btn == 2)
					rbx_update_skip();
				else if (s_upd_btn == 0) {
					focus_off();
					const char *err = rbx_update_install(upd_progress); // (returns only on failure)
					RBX_LOG("update: failed: %s", err);
					snprintf(msg, sizeof(msg), T("Update failed: %s\nThis version stays installed"), err);
				}
			}
		} else if (s_list_mode) { // options (Display / Controls / Refresh, global settings. Saved by rbx_menu_update on close)
			const rbx_menu_result mr = rbx_menu_update(b);
			// (the touch layout editor changes without a button press, and touching Done leaves it the same way; only the
			// editor's own changes were taken here, so after Done the editor stayed on the screen until a button)
			if (s_dirty) {
				s_dirty = 0;
				dirty = 1;
			}
			if (mr != RBX_MENU_STAY) {
				s_list_mode = 0;
				s_page = PAGE_MAIN;
				ui_clear(c);
				if (g_settings.list_group != group0) {   // (another grouping: the same game, in its group)
					group0 = g_settings.list_group;
					s_gl_tab = s_gl_mkc = 0;
					n = gl_build(idx, n ? idx[sel] : -1, &sel);
					scr = -1;
					s_sys_scr = -1;
					fast_ok = 0;
					dirty = 1;
				}
			}
			if (mr == RBX_MENU_EXIT) {   // (Exit in the list's options: no game, main closes the app)
				focus_off();
				rbx_video_set_preview(NULL, NULL);
				return NULL;
			}
			if (mr == RBX_MENU_REFRESH) { // ROMs copied while the app runs
				char cur[24] = "";
				if (n)
					snprintf(cur, sizeof(cur), "%s", gl_set(idx[sel]));
				const int np = gl_scan(dirs, ndirs);
				nplay_kept = np;
				if (auto_all && np && g_settings.list_filter == RBX_LIST_ALL) {
					g_settings.list_filter = RBX_LIST_PLAYABLE; // ROMs now there: back to the games that run
					auto_all = 0;
				}
				int w2 = -1;
				for (int i = 0; cur[0] && i < s_gl_n; i++)
					if (!strcmp(cur, gl_set(i)))
						w2 = i;
				n = gl_build(idx, w2, &sel);
				scr = -1; // (new list: no scroll animation)
				snprintf(msg, sizeof(msg), T("ROM list refreshed: %d game%s ready\nin %s"), np, np == 1 || !rbx_lang_english() ? "" : "s", rom_dir);
				dirty = 1;
			}
		} else if (msg[0]) { // notice box: any button closes it
			if (pressed & (ok | back | SCE_CTRL_START | SCE_CTRL_TRIANGLE))
				msg[0] = 0;
		} else {
			if (pressed) {
				drag = s_sys_drag = 0;   // (the buttons take the lists back from a touch scroll)
				s_sys_vel = 0;
				vel = 0;   // (and stop its glide: a speed left over made the next tap only "stop the list")
				s_tab_vel = 0;
			}
			if (s_gl_col == 0) {   // the systems: up/down another one (its games shown at once), right / OK to its games
				if (s_gl_ntabs > 1 && (pressed & (SCE_CTRL_UP | SCE_CTRL_DOWN))) {
					if (n)
						gl_last_set(idx[sel]);
					n = gl_goto(idx, &sel, s_gl_mkc, wrap(s_gl_tab + ((pressed & SCE_CTRL_UP) ? -1 : 1), s_gl_ntabs));
					scr = -1;
				}
				if (n && (pressed & (SCE_CTRL_RIGHT | ok))) {
					s_gl_col = 1;
					pressed &= ~ok;
				}
			} else {
				if (n && (pressed & SCE_CTRL_UP)) sel = wrap(sel - 1, n);
				if (n && (pressed & SCE_CTRL_DOWN)) sel = wrap(sel + 1, n);
				if (gl_all()) {   // (no groups: Left / Right a page)
					if (n && (pressed & SCE_CTRL_LEFT)) sel = sel >= 9 ? sel - 9 : 0;
					if (n && (pressed & SCE_CTRL_RIGHT)) sel = sel + 9 < n ? sel + 9 : n - 1;
				} else if (pressed & (SCE_CTRL_LEFT | back))
					s_gl_col = 0;
			}
			if (s_gl_nmk > 1 && (pressed & (SCE_CTRL_LTRIGGER | SCE_CTRL_RTRIGGER))) {   // L/R: another maker
				if (n)
					gl_last_set(idx[sel]);
				n = gl_goto(idx, &sel, wrap(s_gl_mkc + ((pressed & SCE_CTRL_LTRIGGER) ? -1 : 1), s_gl_nmk), 0);
				scr = -1;
			} else if (pressed & SCE_CTRL_SELECT) { // filter: Playable -> All -> Favorites (current game kept if visible)
				g_settings.list_filter = wrap(g_settings.list_filter + 1, RBX_LIST_FILTER_COUNT);
				auto_all = 0;
				rbx_settings_save();
				n = gl_build(idx, n ? idx[sel] : -1, &sel);
				scr = -1;
			} else if (n && (pressed & SCE_CTRL_SQUARE)) { // favorite on/off (if it drops out of Favorites the cursor moves on)
				const int cur = idx[sel];
				rbx_fav_toggle(gl_set(cur));
				int w2 = cur;
				if (g_settings.list_filter == RBX_LIST_FAVORITES && !gl_visible(cur))
					w2 = sel + 1 < n ? idx[sel + 1] : (sel > 0 ? idx[sel - 1] : -1);
				n = gl_build(idx, w2, &sel);
			} else if (pressed & SCE_CTRL_TRIANGLE) {   // search; again: the whole list, at the game chosen in the results
				const int cur = n ? idx[sel] : -1;
				if (s_search[0]) {
					s_search[0] = 0;
					n = gl_build(idx, cur, &sel);
					scr = -1;
				} else {
					char q[sizeof(s_search)];
					if (rbx_ime_input("Search: ROM name or title", "", q, sizeof(q)) && q[0]) {
						snprintf(s_search, sizeof(s_search), "%s", q);
						snprintf(s_search_tab, sizeof(s_search_tab), T("Search: %s"), q);
						n = gl_build(idx, -1, &sel);
						scr = -1;
						if (!n) {
							snprintf(msg, sizeof(msg), T("No game matches \"%s\""), q);
							s_search[0] = 0;
							n = gl_build(idx, cur, &sel);
						}
					}
					ui_clear(c);
					fast_ok = 0;   // (the canvas cleared: drawn again in full. A cancelled search kept the strips' key
					               // and left the background, the tabs and the footer black)
				}
				dirty = 1;
			} else if (pressed & FT_ABOUT) {   // the footer's About: the options' About page, closed straight back to the list
				s_mt_block = rbx_time_us() + 400000;
				rbx_settings_use_global();
				s_list_mode = 1;
				s_page = PAGE_MAIN;
				s_sel = s_main_sel = 0;
				go_page(PAGE_ABOUT);
				s_about_scroll = 0;
				s_about_direct = 1;
				s_pop = 0;
				s_changed = 0;
				s_prev_buttons = b;
				dirty = 1;
			} else if (pressed & SCE_CTRL_START) { // options: edit and save global settings only
				s_mt_block = rbx_time_us() + 400000; // (a tap on the settings button is not a tap on the options)
				rbx_settings_use_global();
				s_list_mode = 1;
				s_page = PAGE_MAIN;
				s_sel = s_main_sel = 0;
				s_pop = 0;
				s_changed = 0;
				s_prev_buttons = b; // (so the START press does not act inside options)
				dirty = 1;   // (drawn now: without it the options showed at the next input, a stick moved)
			} else if (n && s_gl_col == 1 && (pressed & ok)) {
				const int g = idx[sel];
				gl_last_set(g);
				if (gl_playable(g)) {
					// the list stays on the screen until the loading screen's first frame replaces it (it takes the
					// row strips away with it). Hiding it here showed a black screen while the game's settings were
					// read and the loading screen's background was decoded
					s_drawn_h = 0;
					focus_off();
					rbx_video_set_preview(NULL, NULL);
					rbx_video_snap_box(0, 0, 0, 0, 0, 0, 0, 0);
					rbx_video_info_overlay(NULL, 0, 0, 0.0f);
				rbx_video_list_hide(0.0f, 0, 0);
				rbx_video_snap_box(0, 0, 0, 0, 0, 0, 0, 0);   // (closed: the snap back to its row at once, user 11-22; a search
				                                             // dialog opened next drew it where the Info had it)
					rbx_video_list_neon(0);
					return &s_found[g];
				}
				if (gl_unported(g))
#ifdef RBX_ENABLE_LOG
					snprintf(msg, sizeof(msg), T("Not ported to ReBOX yet: %s\n(MAME driver %s.cpp)"), gl_title(g),
						s_gl_e[g]->port);
#else
					snprintf(msg, sizeof(msg), T("Not ported to ReBOX yet: %s\n(MAME driver %s)"), gl_title(g),
						s_gl_e[g]->port);
#endif
				else {
					const char *par = rbx_core_game_parent(s_found[g].game);
					snprintf(msg, sizeof(msg), "ROM not found: %s.zip%s%s%s\nPut the ROM zip in %s", gl_set(g),
						par ? "  (clone of " : "", par ? par : "", par ? ")" : "", rom_dir);
				}
			}
		}
		focus_tick();
		const int bat = bat_poll(); // (the header shows it; a new value redraws the header line)
		{	// the maker tab row gliding to its target (each frame: the fast list path redraws when its slide changes)
			if (!t_down && s_tab_vel != 0.0f) {   // (after a flick: it goes on, slowing down, and stops at its ends)
				s_tab_tgt += s_tab_vel;
				s_tab_vel *= 0.95f;
				if (fabsf(s_tab_vel) < 0.5f || s_tab_tgt <= 0.0f || s_tab_tgt >= s_tab_max)
					s_tab_vel = 0;
				if (s_tab_tgt > s_tab_max) s_tab_tgt = s_tab_max;
				if (s_tab_tgt < 0.0f) s_tab_tgt = 0.0f;
				s_tab_cur = s_tab_tgt;
				dirty = 1;
			}
			if (s_tab_tgt > s_tab_max) s_tab_tgt = s_tab_max;
			if (s_tab_tgt < 0.0f) s_tab_tgt = 0.0f;
			s_tab_cur += (s_tab_tgt - s_tab_cur) * 0.3f;
			if (fabsf(s_tab_tgt - s_tab_cur) < 0.5f)
				s_tab_cur = s_tab_tgt;
			if (!s_list_mode && (s_tab_cur != s_tab_tgt || s_sys_vel != 0.0f))   // (the systems' glide too)
				dirty = 1;
		}
		if (s_list_mode && menu_anim())
			dirty = 1; // the options window is still scrolling
		if (s_list_mode && !dirty && s_help_scroll) { // only its footer's help moving: that band alone
			const int off = help_off(s_help_tw, s_help_bw);
			if (off != s_help_off) {
				help_band(c, 0);
				help_draw(c, off);
				ui_commit(c);
			}
		}
		{ // another theme: the list under the options is drawn again in the new colours
			static int seen_gen = -1;
			theme();
			if (seen_gen != s_theme_gen) {
				seen_gen = s_theme_gen;
				under_ok = 0;
				dirty = 1;
			}
		}
		if (!s_list_mode && under_ok) { // options closed: the list is drawn again in full
			under_ok = 0;
			dirty = 1;
		}
		// the list scrolls smoothly and fades out at its top and bottom edges; rows need not fit
		const int tab_h = s_gl_nmk > 1 ? 40 : 0, la = TITLE_H + tab_h, lb = SCREEN_H - LIST_FOOT_H, lpad = 18, fade = 20;
		const float rows_f = (float)(lb - la - 2 * lpad) / GL_LINE;
		const float foc_rows = foc_ext / (float)GL_LINE;   // (the focused row's growth, in rows)
		{
			float maxt = (float)n + foc_rows - rows_f;
			if (maxt < 0)
				maxt = 0;
			if (drag && !t_down && vel != 0.0f) {   // (after a drag: it slows down)
				tgt += vel;
				vel *= 0.95f;
				if (fabsf(vel) < 0.015f)
					vel = 0;
			}
			if (!drag)   // the cursor in the middle of the rows (user 11-22; the list's ends excepted: clamped below)
				tgt = (float)sel + (1.0f + foc_rows) * 0.5f - rows_f * 0.5f + 1.0f;   // (one row above the middle, 11-22)
			if (tgt > maxt) { tgt = maxt; vel = 0; }
			if (tgt < 0) { tgt = 0; vel = 0; }
			if (drag)   // (the rows follow the finger)
				scr = tgt;
			if (drag && n) {   // the cursor follows them: the row in the middle (user 11-22), as the buttons keep it
				// (the cursor's row: one above the middle. The rows after the grown one sit lower by its growth: the row at
				// that line is counted with it, else a slow drag up from below it never moved the cursor, 11-22)
				int mid = (int)floorf(scr + rows_f * 0.5f - 1.0f);
				if (mid > sel) {
					mid = (int)floorf(scr + rows_f * 0.5f - 1.0f - foc_rows);
					if (mid < sel)
						mid = sel;
				}
				if (mid < 0) mid = 0;
				if (mid > n - 1) mid = n - 1;
				if (mid != sel) {
					// (the grown row goes back to one line: the rows after it move up by its growth; the list moved
					// down as much, so nothing jumps under the finger)
					if (foc_ext > 0.0f && mid > sel) {
						tgt -= foc_rows;
						scr = tgt;
					}
					sel = mid;
					dirty = 1;
				}
			}
			if (scr < 0)
				scr = tgt;
			else if (scr - tgt > 1.5f * rows_f || tgt - scr > 1.5f * rows_f) // long jumps: ease over the last half page only
				scr = scr < tgt ? tgt - 0.5f * rows_f : tgt + 0.5f * rows_f;
			if (scr != tgt && !s_list_mode) {
				scr += (tgt - scr) * 0.3f;
				if (fabsf(tgt - scr) < 0.01f)
					scr = tgt;
				dirty = 1;
			}
		}
		{	// the focused row (user 11-22): 0.25 s on a game, it grows to two lines; at 0.6 s its snap is asked for; at 1 s, if the
			// snap came, to three lines with the text pushed right, then the snap fades in and plays. Moving on: back at once
			// (nothing grows or loads while the cursor moves through the list)
			enum { FOC_WAIT = 15, FOC_LOAD = 36, FOC_SNAP = 60 };
			// (it stays grown while the list, the tabs or the systems are scrolled by touch: user 11-22)
			const int cur = n && s_gl_col == 1 && !s_list_mode && !msg[0] && !upd ? idx[sel] : -1;
			if (cur != foc_g) {
				if (foc_req && !info_on)
					rbx_video_set_preview(NULL, NULL);
				if (!info_on)
					foc_req = 0;
				foc_g = cur;
				foc_t = 0;
				if (foc_ext != 0.0f || foc_shift != 0.0f)
					dirty = 1;
				foc_ext = foc_shift = foc_snap = 0.0f;
			} else if (cur >= 0) {
				// (nothing grows while the list is dragged or still gliding: user 11-22)
				const int scrolling = (t_down && t_moved && t_list) || (drag && vel != 0.0f);
				if (foc_t < 100000 && !scrolling)
					foc_t++;
				if (foc_t == FOC_LOAD && !foc_req) {   // (0.6 s: a video opened for every row passed broke the decoder)
					rbx_video_set_preview(gl_set(cur), gl_unported(cur) ? NULL : rbx_core_game_parent(s_found[cur].game));
					foc_req = 1;
				}
				const int has = foc_t >= FOC_SNAP && rbx_snap_kind() != RBX_SNAP_NONE;
				const float ext_t = foc_t < FOC_WAIT ? 0.0f : has ? (float)FOC_EXT3 : (float)FOC_EXT2;
				const float sh_t = has ? 1.0f : 0.0f;
				const float e0 = foc_ext, s0 = foc_shift;
				foc_ext += (ext_t - foc_ext) * 0.22f;
				if (fabsf(ext_t - foc_ext) < 0.5f)
					foc_ext = ext_t;
				foc_shift += (sh_t - foc_shift) * 0.2f;
				if (fabsf(sh_t - foc_shift) < 0.01f)
					foc_shift = sh_t;
				if (has && foc_ext >= (float)FOC_EXT3 - 0.5f && foc_snap < 1.0f)
					foc_snap = foc_snap + 1.0f / 12.0f > 1.0f ? 1.0f : foc_snap + 1.0f / 12.0f;
				if (e0 != foc_ext || s0 != foc_shift)
					dirty = 1;
			}
		}
		{	// the snap's box: in the focused row (grown to three lines), or the Info's, grown to it from the row's over 20
			// frames; none else
			const int ty = la + lpad + (int)floorf(((float)sel - scr) * GL_LINE + 0.5f) + GL_LINE / 2 + 7;
			const int by = ty - 7 - (GL_LINE - 6) / 2;
			int bx, byo, bw, bh;
			foc_snap_box(FOC_EXT3, &bx, &byo, &bw, &bh);
			foc_box[0] = GL_X + bx;
			foc_box[1] = by + byo;
			foc_box[2] = bw;
			foc_box[3] = bh;
			if (info_on) {   // (0.4 s, eased in and out; the zoom of the row's box let go as it grows)
				float k = info_t >= 24 ? 1.0f : (float)info_t / 24.0f;
				k = k < 0.5f ? 4.0f * k * k * k : 1.0f - 4.0f * (1.0f - k) * (1.0f - k) * (1.0f - k);
				const float to[4] = { INFO_SX, INFO_SY, INFO_SW, (float)INFO_SH };
				float r[4];
				for (int q = 0; q < 4; q++)
					r[q] = info_from[q] + (to[q] - info_from[q]) * k;
				rbx_video_snap_box(r[0], r[1], r[2], r[3], 1.0f, 1.0f - k, TITLE_H, lb);
			} else if (foc_snap > 0.0f && n && !s_list_mode && !msg[0] && !upd)
				rbx_video_snap_box((float)foc_box[0], (float)foc_box[1], (float)bw, (float)bh, foc_snap, 1.0f, la, lb);
			else
				rbx_video_snap_box(0, 0, 0, 0, 0, 0, 0, 0);
		}
		// nothing open over the list: the rows are strips (row_tex) and the canvas is drawn again in full only when the
		// header, tabs or footer change; a cursor move redraws the header line, scrolling only the scroll bar
		const int fast = !msg[0] && !upd && !s_list_mode;
		int full = dirty && !fast;
		// a still preview: the canvas's background half see-through over it (the picture drawn under the canvas), so the
		// header, tabs, systems and footer text stay opaque. A video stays over the list (it fades the list away)
		const int see = fast && (theme_rows_see(), s_under_cv && ui_tex(s_under_cv)) && under_theme();
		if (see)   // (each frame: the theme under the list, or Graphite under a preview; the canvas the same)
			rbx_video_list_under(under_theme());
		if (fast) {
			theme();
			const int key[14] = { s_theme_gen, s_gl_tab, s_gl_ntabs, g_settings.list_filter, s_gl_nvis, n, o_ok, s_gl_col,
				s_gl_mkc, s_gl_nmk, g_settings.list_group, (int)floorf(s_tab_cur + 0.5f), see, info_on };   // (the Info's footer)   // (the tab row's slide too)
			if (!fast_ok || memcmp(key, fast_key, sizeof(key))) {
				memcpy(fast_key, key, sizeof(key));
				fast_ok = 1;
				full = 1;
			}
		} else if (dirty) {
			fast_ok = 0;
			rbx_video_strips(NULL, 0, 0, 0);
		}
		if (full) {
			dirty = 0;
			const int w = SCREEN_W;
			const int px = 0, py = 0;
			const uint32_t *bgr = theme_rows(1), *bgf = see ? theme_rows_see() : bgr;   // (bgf: the canvas's fill)
			if (!see)
				rbx_video_list_under(NULL);   // (the canvas opaque: nothing under it)
			s_drawn_h = 0;
			focus_begin();
			if (s_list_mode && under_ok) {
				// options over the list: menu_paint puts back the dimmed list kept when they opened, around the window
				s_under = under;
			} else {
				ui_fill_rows(c, bgf, 0, 0, SCREEN_W, SCREEN_H);
				const int rw = GL_W; // (the games' column, left of the scroll bar)
				const int still = s_list_mode || msg[0] || upd; // (no blinking focus under a window)
				s_focus_cy0 = la;
				s_focus_cy1 = lb;
				const int i0 = (int)floorf(scr - foc_ext / GL_LINE) - 1, i1 = (int)ceilf(scr + rows_f + 2.0f * lpad / GL_LINE);   // (i0: the
				// focused row above the area pushes the next ones down into it: they start earlier)
				const int fext = (int)(foc_ext + 0.5f);
				for (int i = i0 < 0 ? 0 : i0; !fast && i < n && i <= i1; i++) {
					const int ty = la + lpad + (int)floorf(((float)i - scr) * GL_LINE + 0.5f) + GL_LINE / 2 + 7 + (i > sel ? fext : 0);
					const int var = i == sel ? (s_gl_col == 0 ? 4 : info_on ? 3 : still ? 2 : 1) : 0;   // (no focus under the Info)
					if (i == sel && fext > 0)
						card_draw(c, GL_X, rw, ty, idx[i], var, 0, fext, foc_shift);
					else
						gl_row(c, GL_X, rw, ty, idx[i], var, 0);
				}
				s_focus_cy0 = 0;
				s_focus_cy1 = SCREEN_H;
				for (int k = 0; !fast && k < fade; k++) { // fade bands: the background over the rows, opaque at the edge
					const unsigned al = (unsigned)(255 - (k + 1) * 255 / (fade + 1));
					ui_rect(c, GL_X, la + k, rw, 1, (bgr[la + k] & 0xFFFFFFu) | (al << 24));
					ui_rect(c, GL_X, lb - 1 - k, rw, 1, (bgr[lb - 1 - k] & 0xFFFFFFu) | (al << 24));
				}
				ui_fill_rows(c, bgf, 0, 0, SCREEN_W, la); // rows that scrolled past the edges
				ui_fill_rows(c, bgf, 0, lb, SCREEN_W, SCREEN_H - lb);
				if (!fast && !info_on)
					gl_scrollbar(c, n, scr, rows_f, la, lb, 0, 0);
				gl_header(c, n, sel);
				{ // the maker tabs: the current one bright with an accent underline; a row wider than the screen slides so
				  // the current tab stays in view (arrows at the ends that have more)
					int tws[MAX_MAKERS], total = 0, cx0 = 0;
					for (int t = 0; t < s_gl_nmk; t++) {
						tws[t] = ui_text_width(0.95f, maker_label(s_gl_mk[t])) + 16;
						if (t < s_gl_mkc)
							cx0 += tws[t] + 6;
						total += tws[t] + (t + 1 < s_gl_nmk ? 6 : 0);
					}
					const int left = px + 16, avail = w - 32;
					const float maxoff = total > avail ? (float)(total - avail) : 0.0f;
					const int fw = 28;   // (the fades' width: a current tab is brought in clear of them)
					s_tab_max = maxoff;
					if (s_gl_nmk && s_gl_mkc != s_tab_last) {   // (a new current tab: brought into view, its glide stopped)
					s_tab_vel = 0;
						const int cx1 = cx0 + tws[s_gl_mkc < s_gl_nmk ? s_gl_mkc : 0];
						if ((float)cx1 - s_tab_tgt > (float)(avail - fw))
							s_tab_tgt = (float)(cx1 - avail + fw);
						if ((float)cx0 - s_tab_tgt < (float)fw)
							s_tab_tgt = (float)(cx0 - fw);
						if (s_tab_tgt > maxoff) s_tab_tgt = maxoff;
						if (s_tab_tgt < 0.0f) s_tab_tgt = 0.0f;
						s_tab_last = s_gl_mkc;
					}
					float cur = s_tab_cur;
					if (cur > maxoff) cur = maxoff;
					if (cur < 0.0f) cur = 0.0f;
					const int off = (int)floorf(cur + 0.5f);
					int tbx = left - off;
					for (int t = 0; s_gl_nmk > 1 && t < s_gl_nmk; t++) {
						const char *tn = maker_label(s_gl_mk[t]);
						const int tw = tws[t];
						const int vis = tbx + tw > left && tbx < left + avail;   // (at least partly in view: clipped)
						s_tab_x0[t] = vis ? (tbx < left ? left : tbx) : -1;
						s_tab_x1[t] = vis ? (tbx + tw + 6 > left + avail ? left + avail : tbx + tw + 6) : -1;
						if (vis) {
							if (t == s_gl_mkc) {
								int ux0 = tbx + 6, ux1 = tbx + tw - 6;
								if (ux0 < left) ux0 = left;
								if (ux1 > left + avail) ux1 = left + avail;
								if (ux1 > ux0)
									rrect(c, ux0, py + TITLE_H + 26, ux1 - ux0, 4, 2, COL_SEL);
							}
							ui_text_clip(c, tbx + 8, py + TITLE_H + 20, t == s_gl_mkc ? COL_TEXT : COL_DIM, 0.95f, tn, left, left + avail);
						}
						tbx += tw + 6;
					}
					// fades at the ends that have more tabs: the background over the row, opaque at the edge
					if (s_gl_nmk > 1 && (off > 0 || (float)off < maxoff)) {
						const int ty0 = py + TITLE_H, ty1 = py + TITLE_H + 40;
						for (int y = ty0; y < ty1; y++) {
							const uint32_t rgb = bgr[y] & 0xFFFFFFu;
							uint32_t *row = ui_pixels(c) + (size_t)y * SCREEN_W;   // (see: towards the see-through rows)
							for (int k = 0; k < fw; k++) {
								const unsigned al = (unsigned)(255 - k * 255 / fw);
								if (off > 0) {
									if (see)
										row[left + k] = lerp_px(row[left + k], bgf[y], al);
									else
										ui_rect(c, left + k, y, 1, 1, rgb | (al << 24));
								}
								if ((float)off < maxoff) {
									if (see)
										row[left + avail - 1 - k] = lerp_px(row[left + avail - 1 - k], bgf[y], al);
									else
										ui_rect(c, left + avail - 1 - k, y, 1, 1, rgb | (al << 24));
								}
							}
						}
						if (see)
							ui_touch_rect(c, left, ty0, avail, ty1 - ty0);
					}
				}
				s_focus_cy0 = la;
				s_focus_cy1 = lb;
				gl_systems(c, la, lb, !fast);
				s_focus_cy0 = 0;
				s_focus_cy1 = SCREEN_H;
				if (!n && g_settings.list_filter == RBX_LIST_FAVORITES)
					text_keys(c, GL_X + 48, la + 40, COL_DIM, "No favorites. " G_SQU ": add, SELECT: filter");
				else if (!n)
					ui_text(c, GL_X + 48, la + 40, COL_DIM, 1.0f, "No playable games. SELECT: show all, START: Refresh ROM list");
				{ // the footer: touch buttons (each also a pad key), the settings button at the right
					char f[6][48];
					uint32_t fk[6];
					int fn = 0;
					snprintf(f[fn], sizeof(f[fn]), T("%s  Play"), o_ok ? G_CIR : G_CRO);
					fk[fn++] = ok;
					snprintf(f[fn], sizeof(f[fn]), G_SQU "  Favorite");
					fk[fn++] = SCE_CTRL_SQUARE;
					snprintf(f[fn], sizeof(f[fn]), G_TRI "  %s", T(s_search[0] ? "All games" : "Search"));
					fk[fn++] = SCE_CTRL_TRIANGLE;
					if (info_on) {   // (the Info's keys: user 11-22)
						snprintf(f[fn], sizeof(f[fn]), T("%s  Close"), o_ok ? G_CRO : G_CIR);
						fk[fn++] = back;
						snprintf(f[fn], sizeof(f[fn]), "D-pad  Prev/Next");
						fk[fn++] = 0;
					} else {
						snprintf(f[fn], sizeof(f[fn]), T("SELECT  %s"), T(s_gl_filter_name[g_settings.list_filter]));   // (the filter shown now; a press steps on)
						fk[fn++] = SCE_CTRL_SELECT;
						snprintf(f[fn], sizeof(f[fn]), "Info");
						fk[fn++] = FT_INFO;
						snprintf(f[fn], sizeof(f[fn]), "About");
						fk[fn++] = FT_ABOUT;
					}
					const int by = SCREEN_H - LIST_FOOT_H + 7, bh = 27, bpad = 12;
					int fx = px + 16;
					s_ft_n = fn;
					for (int k = 0; k < fn; k++) {
						const int bw = keys_width(f[k]) + 2 * bpad;
						foot_button(c, fx, by, bw, bh);
						text_keys(c, fx + bpad, by + 19, COL_ITEM, f[k]);
						s_ft_x0[k] = fx;
						s_ft_x1[k] = fx + bw;
						s_ft_key[k] = fk[k];
						fx += bw + 8;
					}
					foot_button(c, GEAR_CX - 22, by, 44, bh);
					gear(c, (float)GEAR_CX, (float)GEAR_CY, (float)GEAR_R - 2, COL_TEXT);
				}
				if (msg[0]) { // notice box (over the list)
					const int mw = 760, mh = 150, mx = (SCREEN_W - mw) / 2, my = (SCREEN_H - mh) / 2;
					window_box(c, mx, my, mw, mh, COL_EDGE, COL_PANEL);
					char line[256];
					snprintf(line, sizeof(line), "%s", msg);
					char *nl = strchr(line, '\n');
					if (nl)
						*nl = 0;
					ui_text_clip(c, mx + 24, my + 44, COL_WARN, 1.0f, line, mx + 24, mx + mw - 16);
					if (nl)
						ui_text_clip(c, mx + 24, my + 84, COL_TEXT, 0.9f, nl + 1, mx + 24, mx + mw - 16);
					ui_text(c, mx + 24, my + mh - 20, COL_DIM, 0.8f, "Press any button");
				}
				if (upd)
					upd_draw(c, o_ok);
				s_under = NULL;
				if (s_list_mode) { // options just opened: dim the list once and keep it (the window redraws over it)
					ui_rect(c, 0, 0, SCREEN_W, SCREEN_H, COL_BG);
					if (!under)
						under = (uint32_t *)malloc((size_t)SCREEN_W * SCREEN_H * 4);
					if (under) {
						memcpy(under, ui_pixels(c), (size_t)SCREEN_W * SCREEN_H * 4);
						under_ok = 1;
					}
				}
			}
			if (s_list_mode)
				menu_paint(c, 1);
			if (!fast) {
				focus_end();
				ui_commit(c);
			}
		}
		if (fast) {
			dirty = 0;
			const int rw = GL_W;
			const uint32_t *bgr = theme_rows(1);
			int drawn = full;
			if (!full && (hdr_sel != sel || hdr_bat != bat)) { // the position or the battery in the header
				ui_fill_rows(c, see ? theme_rows_see() : bgr, 0, 0, SCREEN_W, TITLE_H - 8);   // (see is in the key: as drawn)
				gl_header(c, n, sel);
				drawn = 1;
			}
			static UiCanvas *bar_cv; // the scroll bar: a strip of its own (with the header in the canvas, one copy spanned both)
			if (!bar_cv)
				bar_cv = ui_canvas(-1, 0, 0, SCREEN_W - BAR_X, lb - TITLE_H);   // (the tabs may be hidden)
			if (bar_cv && (full || bar_scr != scr || bar_n != n)) {
				ui_clear(bar_cv);
				gl_scrollbar(bar_cv, n, scr, rows_f, la, lb, BAR_X, la);
				ui_commit(bar_cv);
			}
			hdr_sel = sel;
			hdr_bat = bat;
			bar_scr = scr;
			bar_n = n;
			// the fade bands at the top and bottom of the rows: the background over them, opaque at the edge
			static UiCanvas *fade_w[2][2], *fade_sys[2];   // (the games' column, the systems')
#define fade_cv fade_w[gl_all()]
			static int fade_gen = -1;
			if (fade_gen != s_theme_gen) {
				for (int e = 0; e < 4; e++) {
					UiCanvas **fc = e < 2 ? &fade_cv[e] : &fade_sys[e - 2];
					const int fw = e < 2 ? rw : SYS_W - 4, top = !(e & 1);
					if (!*fc)
						*fc = ui_canvas(-1, 0, 0, fw, fade);
					if (!*fc)
						continue;
					ui_clear(*fc);
					for (int k = 0; k < fade; k++) {
						const unsigned al = (unsigned)(255 - (k + 1) * 255 / (fade + 1));
						const int y = top ? k : fade - 1 - k;
						ui_rect(*fc, 0, y, fw, 1, (bgr[top ? la + k : lb - 1 - k] & 0xFFFFFFu) | (al << 24));
					}
					ui_commit(*fc);
				}
				fade_gen = s_theme_gen;
			}
			// rows on screen, the selected one with its focus (blinking while fully inside the rows area, else a
			// steady outline in the row) and its scrolling title
			s_row_frame++;
			ReiStrip st[RBX_NSTRIP];
			int ns = 0;
			focus_begin();
			const int i0 = (int)floorf(scr - foc_ext / GL_LINE) - 1, i1 = (int)ceilf(scr + rows_f + 2.0f * lpad / GL_LINE);   // (i0: the
				// focused row above the area pushes the next ones down into it: they start earlier)
			const int fext = (int)(foc_ext + 0.5f);
			for (int i = i0 < 0 ? 0 : i0; i < n && i <= i1 && ns < RBX_NSTRIP - 3; i++) {
				const int ty = la + lpad + (int)floorf(((float)i - scr) * GL_LINE + 0.5f) + GL_LINE / 2 + 7 + (i > sel ? fext : 0);
				int var = 0, off = 0;
				if (i == sel) {
					const int by = ty - 7 - (GL_LINE - 6) / 2, bh = GL_LINE - 6 + fext;
					const int inside = by - 2 >= la && by + bh + 2 <= lb;
					var = s_gl_col == 0 ? 4 : inside ? 3 : 2;   // (the systems focused: the bar only, no shadow)
					if (inside && s_gl_col == 1) {
						s_focus_cy0 = la;
						s_focus_cy1 = lb;
						focus_at(c, GL_X + 12, by, rw - 24, bh);
						s_focus_cy0 = 0;
						s_focus_cy1 = SCREEN_H;
					}
					if (mq_sel != idx[i]) {
						mq_sel = idx[i];
						mq_t = 0;
						mq_tw = ui_text_width(1.0f, gl_title(idx[i]));
					}
					const int avail = rw - 72, over = mq_tw - avail, hold = 60;
					if (over > 0) { // 1 s pause -> 60 px/s to the end -> 1 s pause -> from the start
						const int t = mq_t++ % (hold + over + hold);
						off = t < hold ? 0 : t < hold + over ? t - hold : over;
					}
				}
				const ReiTex *t = i == sel && fext > 0 ? card_tex(idx[i], var, off, rw, fext, foc_shift) : row_tex(idx[i], var, off, rw);
				if (t)
					st[ns++] = (ReiStrip){ t, GL_X, ty - ROW_TY };
			}
			if (!n)
				mq_sel = -1;
			for (int e = 0; e < 2; e++)
				if (!see && fade_cv[e] && ui_tex(fade_cv[e]))   // (over the neon corridor: no fade of the theme's colour, user 11-22)
					st[ns++] = (ReiStrip){ ui_tex(fade_cv[e]), GL_X, e ? lb - fade : la };
			if (bar_cv && ui_tex(bar_cv) && ns < RBX_NSTRIP)
				st[ns++] = (ReiStrip){ ui_tex(bar_cv), BAR_X, la };
			sys_ease(la, lb, 1);   // the systems column: its rows as strips, its focus while it has it
			{	// its scroll bar (user 11-20), left of the line between the columns, when its rows do not all fit
				static UiCanvas *sbar_cv;
				static float sbar_scr = -1;
				static int sbar_n = -1;
				const float srows = (float)(lb - la - 2 * GL_LPAD) / GL_LINE;
				const int show = !gl_all() && (float)s_gl_ntabs > srows;
				if (!sbar_cv)
					sbar_cv = ui_canvas(-1, 0, 0, 6, lb - TITLE_H);
				if (sbar_cv && show && (full || sbar_scr != s_sys_scr || sbar_n != s_gl_ntabs)) {
					ui_clear(sbar_cv);
					sys_scrollbar(sbar_cv, srows, la, lb, la);
					ui_commit(sbar_cv);
					sbar_scr = s_sys_scr;
					sbar_n = s_gl_ntabs;
				}
				if (show && sbar_cv && ui_tex(sbar_cv) && ns < RBX_NSTRIP)
					st[ns++] = (ReiStrip){ ui_tex(sbar_cv), SYS_X_BAR, la };
			}
			s_sys_frame++;
			for (int t = 0; !gl_all() && t < s_gl_ntabs && ns < RBX_NSTRIP - 2; t++) {
				const int ty = sys_row_ty(la, t);
				if (ty - ROW_TY + ROW_H <= la || ty - ROW_TY >= lb)
					continue;
				const ReiTex *tt = sys_tex(t);
				if (tt)
					st[ns++] = (ReiStrip){ tt, 0, ty - ROW_TY };
				int by, bw;
				sys_bar(ty, &by, &bw);
				if (t == s_gl_tab && s_gl_col == 0 && by - 2 >= la && by + GL_LINE - 6 + 2 <= lb) {
					s_focus_cy0 = la;
					s_focus_cy1 = lb;
					focus_at(c, 12, by, bw, GL_LINE - 6);
					s_focus_cy0 = 0;
					s_focus_cy1 = SCREEN_H;
				}
			}
			for (int e = 0; e < 2; e++)   // (the column's fades, as the games')
				if (!see && !gl_all() && fade_sys[e] && ui_tex(fade_sys[e]) && ns < RBX_NSTRIP)
					st[ns++] = (ReiStrip){ ui_tex(fade_sys[e]), 0, e ? lb - fade : la };
			rbx_video_strips(st, ns, la, lb);
			focus_end();
			// after a full redraw the other two textures still need the whole canvas: copied on frames with nothing
			// else to do (the screen is still then), not on the next cursor move
			if (drawn || sync_left > 0) {
				ui_commit(c);
				sync_left = full ? 2 : sync_left > 0 ? sync_left - 1 : 0;
			}
		}
		rbx_osd_tick(); // (notices from the options, such as a key used twice, time out here too)
#ifdef RBX_ENABLE_LOG
		const uint64_t lf_t1 = rbx_time_us();
#endif
		rbx_video_present();
#ifdef RBX_ENABLE_LOG
		{
			const uint64_t lf_t2 = rbx_time_us();
			if (lf_t2 - lf_t0 > 40000)
				RBX_LOG("list: slow frame %u us: work %u, present %u; sel %d->%d drag %d ext %.0f full %d snap %d",
					(unsigned)(lf_t2 - lf_t0), (unsigned)(lf_t1 - lf_t0), (unsigned)(lf_t2 - lf_t1), lf_sel, sel, drag,
					foc_ext, full, rbx_snap_kind());
		}
#endif
		shot_check(c);
		{ // keep the system from suspending the app after a while without input (as in the game, main.c)
			static unsigned pt;
			if (++pt % 30 == 0)
				sceKernelPowerTick(SCE_KERNEL_POWER_TICK_DEFAULT);
		}
#ifdef RBX_ENABLE_LOG
		static unsigned beat;
		static uint64_t beat_t;
		static uint64_t last_t;
		static uint32_t gap_max;
		const uint64_t now = rbx_time_us();
		if (last_t && now - last_t > gap_max)
			gap_max = (uint32_t)(now - last_t);
		last_t = now;
		if (++beat % 120 == 0) {
			RBX_LOG("list: %u frames, %.1f ms/frame, longest %u us", beat, (now - beat_t) / 120000.0, (unsigned)gap_max);
			beat_t = now;
			gap_max = 0;
		}
#endif
	}
}

// test (log builds): the shotfb screen capture while a game runs (called once per frame; "shotgame" never exists)
void rbx_menu_shot_poll(void)
{
	shot_check_as(s_cv, RBX_DATA_DIR "shotgame");
}
