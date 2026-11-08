/*  Konami's Z80 medal games. Ported from MAME src/mame/konami/konmedal.cpp (BSD-3-Clause, copyright-holders: R. Belmont;
    driver by R. Belmont, MetalliC), src/devices/machine/k053252.cpp (LGPL-2.1+, copyright-holders: Angelo Salese) and
    src/devices/machine/ticket.cpp (BSD-3-Clause, copyright-holders: Aaron Giles). Two families: the
    TMNT-based boards (Tsururin Kun, Slime Kun, Mario Roulette, Tsurikko Penta, Shuriken Boy, Fuusen Pentai: a Z80 at
    6 MHz, the K052109 / K051962 tilemaps, a K051649 and a uPD7759) and the GX-based boards (Dam Dam Boy, Buttobi Striker,
    Tsukande Toru Chicchi: a Z80 at 7.16 MHz, the K056832 tilemaps seen through an 8-bit window, a K053252, an OKIM6295
    and a K051649 or a YMZ280B). The palette is the PROMs'; no sprites. All the sound chips are the main CPU's and are
    rendered line by line on the main thread. Chusenoh (its keypad's i8251) is not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k051649.h"
#include "../../sound/okim6295.h"
#include "../../sound/upd7759.h"
#include "../../sound/ymz280b.h"
#include "../video/k052109.h"
#include "../video/k056832.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define Z80_TMNT 6000000u      // (24 MHz / 4)
#define Z80_GX 7159090u        // (14.318181 MHz / 2)
#define SCC_TMNT 3000000u      // (24 MHz / 8)
#define SCC_GX 3579545u        // (14.318181 MHz / 4)
#define OKI_CLOCK 1022727u     // (14.318181 MHz / 14, pin 7 high)
#define UPD_CLOCK 640000u
#define YMZ_CLOCK 16934400u
#define SND_RATE 48000
#define LINES_TMNT 264         // (set_raw(24 MHz / 4, 384, 16, 304, 264, 16, 240))
#define LINES_GX 256           // (set_refresh_hz(59.62), set_size(64 * 8, 32 * 8), set_vblank_time(0))
#define VIS_Y0 16
#define VIS_H 224
#define VIS_END (VIS_Y0 + VIS_H)
#define VIS_W_MAX 344
#define NPENS 256
#define RAM_SIZE 0x2000

enum { B_TMNT, B_DDBOY, B_TSUKANDE };

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t *s_prom;
static uint8_t *s_gfx;   // (the K056832's tiles: a byte a pixel)
static uint8_t *s_gfx_row;   // (each tile line: 0 all see-through, 1 some, 2 none)
static z80 s_z;
static const uint8_t *s_zmap[256];
static k052109_t s_k109;
static k056832_t s_k832;
static k051649_t s_scc;
static okim6295_t s_oki;
static upd7759_t s_upd;
static ymz280b_t s_ymz;
static int s_board, s_gx, s_has_nmi, s_lines, s_vis_x0, s_vis_w, s_nvsize;
static int s_port_dsw[2], s_port_in[2];
static int s_colorbase[4], s_order[4];
static uint32_t s_cyc_num, s_line_den, s_snd_num, s_hop_period;
static uint32_t s_snd_rate;
static uint8_t *s_pri;
static char s_nv_path[256];

static struct {
	uint8_t ram[RAM_SIZE];       // (the battery RAM: GX a000-bfff; TMNT 8000-87ff)
	uint8_t control, control2, bank, scc_on;
	uint8_t k53252[16];
	int32_t k53252_vc;
	int32_t ccu_int_time, ccu_int_time_count;
	int32_t irq;                 // (the Z80's IRQ line)
	uint8_t nmi, scan_on, nmi_left;
	uint8_t hop_power, hop_status;
	int32_t hop_left;            // (the hopper's timer, in lines)
	int32_t vpos;
	int32_t oki_hold;
	uint32_t oki_frac;
	uint64_t cyc_frac, z_end, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { k052109_t k109; k056832_t k832; } s_vs;
static k052109_view_t s_view;

static void capture(void)
{
	if (s_gx)
		memcpy(&s_vs.k832, &s_k832, offsetof(k056832_t, rom));
	else
		memcpy(&s_vs.k109, &s_k109, offsetof(k052109_t, state_end));
}

static void tile_cb(int layer, int bank, int *code, int *color, int *flags, int *priority)   // (shuriboy_tile_callback)
{
	const int c = *color;
	*code |= ((c & 0x0c) << 6) | (bank << 10);
	if (c & 0x02)
		*code |= 0x1000;
	*flags = (c & 0x01) ? K052109_TILE_FLIPX : 0;
	*priority = (c >> 4) & 1;
	*color = s_colorbase[layer] + ((c >> 5) & 7);
}

static inline int gx_tile(int codebits, int mode, int avac)   // (tile_callback's code)
{
	const int code = mode ? (((avac >> ((codebits >> 8) & 0x0c)) & 0x0f) << 10) | (codebits & 0x3ff) : codebits & 0xfff;
	// (bitswap<14>(code, 8, 9, 13, 12, 11, 10, 7, 6, 5, 4, 3, 2, 1, 0))
	return (code & 0xff) | (code >> 2 & 0x0f00) | (code & 0x0100) << 5 | (code & 0x0200) << 3;
}

// the K056832's layers (tilemap_draw_common, each page drawn by lines), line by line: a pixel's result does not depend on
// the other lines, so each line's tile pieces are found once for the two categories and the four layers drawn over each
// other. The CPU's window reaches only the code words, so the attribute words stay 0: no tile flips. The screen flips and
// the line map mode are not done.
typedef struct { const uint8_t *src; int16_t x; uint8_t n, kind; uint16_t pen; } gx_piece;
typedef struct {
	int rowspan, colspan, height, width, ay, scrollmode, dx_reg, dy;
	const uint16_t *sc;
} gx_layer;

static void gx_layer_set(const k056832_t *k, int layer, gx_layer *g)
{
	g->rowspan = k->h[layer] + 1;
	g->colspan = k->w[layer] + 1;
	g->height = g->rowspan * K056832_PAGE_HEIGHT;
	g->width = g->colspan * K056832_PAGE_WIDTH;
	g->dy = k->dy[layer];
	g->dx_reg = k->dx[layer];
	const int scrollbank = ((k->regs[0x18] >> 1) & 0x0c) | (k->regs[0x18] & 3);
	g->scrollmode = (k->regs[0x05] >> (k->lsram_page[layer][0] << 1)) & 3;
	g->sc = k->videoram + (scrollbank << 12) + (k->lsram_page[layer][1] >> 1);
	g->ay = (int)((unsigned)(g->dy - k->layer_offs[layer][1]) % (unsigned)g->height);
}

// the pieces of a layer's line, by category
static void gx_line(const k056832_t *k, int layer, const gx_layer *g, int y, int mode, int avac, gx_piece *list[2], int cnt[2])
{
	const int my = (y + g->ay) % g->height;
	const int r = my >> 8, py = my & 255;
	const int sline = g->rowspan > 1 ? my : g->dy + y;   // (the scroll table's line: the map's, one page counted from dy)
	int dx;
	if (g->scrollmode == 0) {
		const int o = (sline * 2) & 0x3ff;
		dx = (int)((uint32_t)g->sc[o] << 16 | g->sc[o + 1]);
	} else if (g->scrollmode == 2) {
		const int o = ((sline & ~7) * 2) & 0x3ff;
		dx = (int)((uint32_t)g->sc[o] << 16 | g->sc[o + 1]);
	} else
		dx = (uint16_t)g->dx_reg;
	dx -= k->layer_offs[layer][0];
	const int sx = g->colspan > 1 ? dx & (g->width - 1) : dx & 511;
	const int row = (py >> 3) * 64;
	const int pixrow = (py & 7) * 8, trow = py & 7;
	const int x0 = s_vis_x0, x1 = s_vis_x0 + s_vis_w - 1;
	for (int x = x0; x <= x1;) {
		const int mx = (x + sx) % g->width;
		int n = 8 - (mx & 7);
		if (n > x1 - x + 1)
			n = x1 - x + 1;
		const int page = (((k->y[layer] + r) & 3) << 2) | ((k->x[layer] + (mx >> 9)) & 3);
		const int owner = k->layer_assoc_with_page[page];
		if (k->layer_association ? owner == layer : owner != -1) {
			const int codebits = k->videoram[(page << 12) + ((row + ((mx & 511) >> 3)) << 1) + 1];
			const uint32_t tile = (uint32_t)gx_tile(codebits, mode, avac);
			const int kind = s_gfx_row[tile * 8 + (uint32_t)trow];
			if (kind) {   // (0: nothing to draw on this line of the tile)
				const int cat = (codebits >> 12) & 1;
				const int tl = k->layer_association || page ? layer : 0;   // (the layer active as the page's tiles are got)
				gx_piece *q = &list[cat][cnt[cat]++];
				q->src = s_gfx + tile * 64 + (uint32_t)pixrow + (uint32_t)(mx & 7);
				q->x = (int16_t)(x - x0);
				q->n = (uint8_t)n;
				q->kind = (uint8_t)kind;
				q->pen = (uint16_t)(((s_colorbase[tl] + ((codebits >> 13) & 7)) & 15) * 16);
			}
		}
		x += n;
	}
}

static void gx_render(const k056832_t *k)
{
	const int mode = k->regs[0x04 / 2] & 7, avac = k->regs[0x38 / 2];
	const int pitch = rbx_video.pitch;
	static gx_piece pieces[2][4][VIS_W_MAX / 8 + 2];
	gx_layer g[4];
	for (int l = 0; l < 4; l++)
		gx_layer_set(k, s_order[l], &g[l]);
	for (int y = VIS_Y0; y < VIS_END; y++) {
		int cnt[2][4];
		for (int l = 0; l < 4; l++) {
			gx_piece *list[2] = { pieces[0][l], pieces[1][l] };
			int c[2] = { 0, 0 };
			gx_line(k, s_order[l], &g[l], y, mode, avac, list, c);
			cnt[0][l] = c[0];
			cnt[1][l] = c[1];
		}
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * pitch;
		for (int cat = 0; cat < 2; cat++)
			for (int l = 0; l < 4; l++) {
				const gx_piece *q = pieces[cat][l];
				for (int j = cnt[cat][l]; j > 0; j--, q++) {
					const uint8_t *src = q->src;
					uint16_t *o = d + q->x;
					const int pen = q->pen, n = q->n;
					if (q->kind == 2)   // (no see-through pixel on this line of the tile)
						for (int i = 0; i < n; i++)
							o[i] = (uint16_t)(pen + src[i]);
					else
						for (int i = 0; i < n; i++)
							if (src[i])
								o[i] = (uint16_t)(pen + src[i]);
				}
			}
	}
}

static void km_render(void)   // (screen_update_konmedal, screen_update_shuriboy)
{
	const int pitch = rbx_video.pitch;
	for (int i = 0; i < NPENS; i++)   // (konmedal_palette: the 4-bit PROMs << 4)
		rbx_video.palette[i] = (uint32_t)((s_prom[i] << 4) & 0xff) << 16 | (uint32_t)((s_prom[0x100 + i] << 4) & 0xff) << 8 |
		                       (uint32_t)((s_prom[0x200 + i] << 4) & 0xff);
	for (int y = 0; y < VIS_H; y++)
		memset(rbx_video.bitmap + y * pitch, 0, (size_t)s_vis_w * 2);
	if (s_gx) {
		gx_render(&s_vs.k832);
		return;
	}
	k052109_t *k = &s_vs.k109;
	k->rom = s_k109.rom;
	k->rom_size = s_k109.rom_size;
	k->gfx = s_k109.gfx;
	k->ntiles = s_k109.ntiles;
	k->colors = s_k109.colors;
	k->cb = tile_cb;
	k052109_prepare(k, &s_view, s_vis_x0 + (s_vis_x0 + s_vis_w - 1) + 1, VIS_Y0 + (VIS_END - 1) + 1);
	for (int p = 0; p < 2; p++)
		for (int l = 0; l < 3; l++)
			k052109_draw(k, &s_view, s_order[l], p, 0, rbx_video.bitmap, s_pri, pitch, s_vis_x0, VIS_Y0, s_vis_x0, VIS_Y0,
			             s_vis_x0 + s_vis_w - 1, VIS_END - 1);
}

// ---- the board ----
static void bank_set(int b)
{
	t.bank = (uint8_t)b;
	const uint8_t *base = s_gx ? s_rom + b * 0x2000 : s_rom + 0x8000 + b * 0x2000;
	const int page = s_gx ? 0x80 : 0xa0;
	for (int i = 0; i < 0x20; i++)
		s_zmap[page + i] = base + i * 256;
}

static void hopper_motor(int state)   // (ticket_dispenser_device::motor_w, hopper type)
{
	if (state && !t.hop_power) {
		t.hop_left = (int32_t)s_hop_period;
		t.hop_power = 1;
		t.hop_status = 0;
	} else if (!state && t.hop_power) {
		if (!t.hop_status)
			t.hop_left = 0;
		t.hop_power = 0;
	}
	rbx_port_custom(s_port_in[1], 0x10, t.hop_status ? 0x10 : 0);
}

static void hopper_line(void)   // (update_output_state)
{
	if (t.hop_left && !--t.hop_left) {
		t.hop_status ^= 1;
		if (t.hop_power)
			t.hop_left = (int32_t)s_hop_period;
		rbx_port_custom(s_port_in[1], 0x10, t.hop_status ? 0x10 : 0);
	}
}

static void control2_w(uint8_t v)
{
	t.control2 = v;
	if (!s_gx) {   // (the board's direct reset line, then the OUT port's fields in the order of their bits)
		upd7759_reset_w(&s_upd, v & 2);
		upd7759_start_w(&s_upd, !(v & 1));
	}
	hopper_motor(v & 0x40);
}

static void scc_enable_w(uint8_t v) { t.scc_on = (v & 0x3f) == 0x3f; }

static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

// the K053252 (only the VCT read-back and the interrupts: the screen stays as the machine's configuration has it)
static uint8_t k53252_r(int o)
{
	const int v = t.vpos - t.k53252_vc;
	if (o == 0x0e)
		return (uint8_t)((v >> 8) & 1);
	if (o == 0x0f)
		return (uint8_t)v;
	return t.k53252[o];
}

static void k53252_w(int o, uint8_t v)
{
	t.k53252[o] = v;
	if (o == 0x08 || o == 0x09)
		t.k53252_vc = ((t.k53252[8] << 8 | t.k53252[9]) & 0x1ff) + 1;
	else if (o == 0x0d)
		t.ccu_int_time = v;
	else if (o == 0x0e)
		t.irq = 0;
	else if (o == 0x0f)
		t.nmi = 0;
}

static uint8_t vram_r(int o)
{
	if (!(t.control2 & 0x80))
		return k056832_ram_code_r(&s_k832, o >> 1, o & 1);
	return t.control == 0 ? k056832_konmedal_rom_r(&s_k832, (uint32_t)o) : 0;   // (the ROM readback)
}

static uint8_t gx_rd(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0xe000)
		return vram_r(a - 0xe000);
	if (a >= 0xc700 && a < 0xc704) {
		const int o = a & 3;
		if (o >= 2)
			return port(s_port_in[o - 2]);
		return port(s_port_dsw[s_board == B_DDBOY ? o : 1 - o]);   // (ddboy_main: DSW1, DSW2; medal_main: the other way round)
	}
	if (a >= 0xc800 && a < 0xc810)
		return k53252_r(a & 15);
	if (s_board == B_TSUKANDE) {
		if (a == 0xd000 || a == 0xd001)
			return ymz280b_read(&s_ymz, a & 1);
	} else if (a == 0xcc00)
		return oki_read(&s_oki);
	else if (a >= 0xd800 && t.scc_on)
		return k051649_scc_r(&s_scc, a & 0xff, 0);
	return 0;
}

static void gx_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000) {
		const int o = a - 0xe000;
		k056832_ram_code_w(&s_k832, o >> 1, o & 1, v);
	} else if (a >= 0xa000 && a < 0xc000)
		t.ram[a - 0xa000] = v;
	else if (a >= 0xc000 && a < 0xc040) {   // (k056832_w: offset ^ 1)
		const int o = (a - 0xc000) ^ 1;
		if (o & 1)
			k056832_word_w(&s_k832, o >> 1, v, 0x00ff);
		else
			k056832_word_w(&s_k832, o >> 1, (uint16_t)(v << 8), 0xff00);
	} else if (a >= 0xc600 && a < 0xc610) {
		const int o = (a - 0xc600) ^ 1;
		if (o & 1)
			k056832_b_word_w(&s_k832, o >> 1, v, 0x00ff);
		else
			k056832_b_word_w(&s_k832, o >> 1, (uint16_t)(v << 8), 0xff00);
	} else if (a == 0xc100)
		control2_w(v);
	else if (a == 0xc400) {
		bank_set(v >> 4);
		t.control = v & 0x0f;
	} else if (a >= 0xc800 && a < 0xc810)
		k53252_w(a & 15, v);
	else if (s_board == B_TSUKANDE) {
		if (a == 0xd000 || a == 0xd001)
			ymz280b_write(&s_ymz, a & 1, v);
	} else if (a == 0xcc00)
		oki_write(&s_oki, v);
	else if (a >= 0xd000 && a < 0xd800)
		scc_enable_w(v);
	else if (a >= 0xd800 && t.scc_on)
		k051649_scc_w(&s_scc, a & 0xff, v);
}

static uint8_t tm_rd(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0xc000)
		return k052109_read(&s_k109, a - 0xc000);
	switch (a) {
	case 0x8800: return port(s_port_in[1]);
	case 0x8801:
		rbx_port_custom(s_port_in[0], 0x80, upd7759_busy_r(&s_upd) ? 0x80 : 0);
		return port(s_port_in[0]);
	case 0x8802: return port(s_port_dsw[0]);
	case 0x8803: return port(s_port_dsw[1]);
	}
	if (a >= 0x9800 && a < 0xa000 && t.scc_on)
		return k051649_scc_r(&s_scc, a & 0xff, 0);
	return 0;
}

static void tm_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000) {
		k052109_write(&s_k109, a - 0xc000, v);
		t.irq = s_k109.irq_out;
	} else if (a >= 0x8000 && a < 0x8800)
		t.ram[a - 0x8000] = v;
	else if (a == 0x8900)
		control2_w(v);
	else if (a == 0x8c00) {   // (shuri_bank_w)
		k052109_set_rmrd_line(&s_k109, v & 0x40);
		bank_set(v & 3);
	} else if (a == 0x8d00)
		upd7759_port_w(&s_upd, v);
	else if (a >= 0x9000 && a < 0x9800)
		scc_enable_w(v);
	else if (a >= 0x9800 && a < 0xa000 && t.scc_on)
		k051649_scc_w(&s_scc, a & 0xff, v);
}

static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- the sound: a line's samples ----
static void sound_line(int n)
{
	if (n <= 0)
		return;
	if (n > 16)
		n = 16;
	if (s_board == B_TSUKANDE) {   // (the speakers 1.0 each)
		int32_t l[16], r[16];
		memset(l, 0, sizeof(l));
		memset(r, 0, sizeof(r));
		ymz280b_render(&s_ymz, l, r, n);
		for (int i = 0; i < n; i++)
			rbx_audio_put_stereo(l[i], r[i]);
		return;
	}
	int32_t sc[16], up[16];
	k051649_render(&s_scc, sc, n);
	if (s_board == B_TMNT)
		upd7759_render(&s_upd, up, n);
	for (int i = 0; i < n; i++) {
		int32_t m = sc[i] * 45 / 100;
		if (s_board == B_TMNT)
			m += up[i] * 60 / 100;
		else {
			t.oki_frac += s_oki.rate;
			while (t.oki_frac >= SND_RATE) {
				t.oki_frac -= SND_RATE;
				t.oki_hold = oki_sample(&s_oki);
			}
			m += t.oki_hold;
		}
		rbx_audio_put(m);
	}
}

// ---- init / frame ----
static void km_exit(void);
static void km_reset(void);

static void nvram_default(const char *m)   // (the machines' nvram handlers; tsururin: all 0)
{
	uint8_t *r = t.ram;
	if (s_gx) {   // (medal_nvram_init)
		r[0x303] = 0xff;
		r[0x309] = 0xaa;
	} else if (!strcmp(m, "shuriboy")) {
		static const struct { uint8_t o, v; } s[] = { { 0x08, 0x01 }, { 0x09, 0x02 }, { 0x0e, 0x04 }, { 0x17, 0x08 },
			{ 0x18, 0x10 }, { 0x21, 0x20 }, { 0x50, 0x40 }, { 0x52, 0x80 }, { 0x53, 0x55 }, { 0x54, 0xaa }, { 0x5d, 0x33 } };
		for (size_t i = 0; i < sizeof(s) / sizeof(s[0]); i++)
			r[s[i].o] = s[i].v;
	} else if (!strcmp(m, "fuusenpn")) {
		r[0x203] = 0xff;
		r[0x209] = 0xaa;
	} else if (!strcmp(m, "mariorou")) {
		r[0x502] = 0xff;
		r[0x506] = 0xaa;
		r[0x508] = 0x55;
	} else if (!strcmp(m, "tsupenta")) {
		r[0x503] = 0xff;
		r[0x506] = 0x00;
		r[0x509] = 0xaa;
		r[0x550] = 0x55;
	}
}

static void set_layers(const int *base, const int *order)
{
	for (int i = 0; i < 4; i++) {
		s_colorbase[i] = base[i];
		s_order[i] = order[i];
	}
}

static void driver_init(const char *set)   // (the GAME lines' init functions)
{
	static const int b_tmnt[4] = { 0, 8, 8, 0 }, b_gx[4] = { 0, 8, 0, 8 }, b_tsuka[4] = { 0, 8, 0, 0 };
	static const int o_mario[4] = { 0, 1, 2, 0 }, o_shuri[4] = { 2, 1, 0, 0 }, o_fuusen[4] = { 0, 2, 1, 0 };
	static const int o_ddboy[4] = { 3, 2, 1, 0 }, o_buttobi[4] = { 1, 0, 3, 2 }, o_tsuka[4] = { 3, 1, 0, 2 };
	if (!strcmp(set, "shuriboy"))
		set_layers(b_tmnt, o_shuri);
	else if (!strcmp(set, "fuusenpn"))
		set_layers(b_tmnt, o_fuusen);
	else if (!strcmp(set, "buttobi"))
		set_layers(b_gx, o_buttobi);
	else if (!strcmp(set, "tsukande"))
		set_layers(b_tsuka, o_tsuka);
	else if (s_gx)
		set_layers(b_gx, o_ddboy);
	else
		set_layers(b_tmnt, o_mario);
}

static int km_init(const rbx_game *g)
{
	uint32_t ps, ts, ss;
	s_board = !strcmp(g->machine, "tsukande") ? B_TSUKANDE : !strcmp(g->machine, "ddboy") ? B_DDBOY : B_TMNT;
	s_gx = s_board != B_TMNT;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region(s_gx ? "k056832" : "k052109", &ts);
	const uint8_t *snd = rbx_region(s_board == B_TSUKANDE ? "ymz" : s_gx ? "oki" : "upd", &ss);
	if (!s_rom || s_rom_size < (s_gx ? 0x20000u : 0x10000u) || !s_prom || ps < 0x300 || !tiles || !snd) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	driver_init(g->set);
	s_has_nmi = s_gx || (strcmp(g->machine, "tsupenta") && strcmp(g->machine, "tsururin"));
	s_lines = s_gx ? LINES_GX : LINES_TMNT;
	s_vis_x0 = s_gx ? 32 : 16;   // (GX: the K053252's res_change, set_offsets(32, 16) and 344 wide)
	s_vis_w = s_gx ? 344 : 288;
	s_nvsize = s_gx ? RAM_SIZE : 0x800;
	s_snd_rate = s_board == B_TSUKANDE ? YMZ_CLOCK / 192 : SND_RATE;
	if (s_gx) {   // (a line's time: 1 / (59.62 x 256))
		s_cyc_num = Z80_GX;
		s_line_den = 5962u * LINES_GX;
		s_snd_num = s_snd_rate * 100u;
		s_hop_period = 1526;   // (100 ms)
	} else {
		s_cyc_num = Z80_TMNT;
		s_line_den = 15625;
		s_snd_num = s_snd_rate;
		s_hop_period = 1562;
	}
	s_pri = NULL;
	s_gfx = NULL;
	if (s_gx) {
		const uint32_t n = ts / 32;   // (charlayout4: 8x8 of packed nibbles, the 32-bit words' bytes 1, 0, 3, 2)
		s_gfx = (uint8_t *)malloc((size_t)n * 64);
		s_gfx_row = (uint8_t *)malloc((size_t)n * 8);
		if (!s_gfx || !s_gfx_row) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		static const uint8_t sh[8] = { 4, 0, 4, 0, 4, 0, 4, 0 }, by[8] = { 1, 1, 0, 0, 3, 3, 2, 2 };
		for (uint32_t e = 0; e < n; e++)
			for (int y = 0; y < 8; y++)
				for (int x = 0; x < 8; x++)
					s_gfx[e * 64 + (uint32_t)(y * 8 + x)] = (tiles[e * 32 + (uint32_t)(y * 4 + by[x])] >> sh[x]) & 15;
		for (uint32_t e = 0; e < n * 8; e++) {
			int nz = 0;
			for (int x = 0; x < 8; x++)
				nz += s_gfx[e * 8 + (uint32_t)x] != 0;
			s_gfx_row[e] = (uint8_t)(nz == 0 ? 0 : nz == 8 ? 2 : 1);
		}
		k056832_init(&s_k832, tiles, ts, K056832_BPP_4, 1, 0);
	} else if (k052109_init(&s_k109, tiles, ts, NPENS, tile_cb)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (s_board == B_TSUKANDE)
		ymz280b_init(&s_ymz, YMZ_CLOCK, snd, ss, NULL, NULL);
	else {
		k051649_init(&s_scc, s_gx ? SCC_GX : SCC_TMNT);
		k051649_set_rate(&s_scc, SND_RATE);
		if (s_gx)
			oki_init(&s_oki, OKI_CLOCK, 1, snd, ss, 256);
		else
			upd7759_init(&s_upd, snd, ss, UPD_CLOCK, SND_RATE);
	}
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_in[0] = rbx_port_find("IN1");
	s_port_in[1] = rbx_port_find("IN2");
	if ((s_board == B_TSUKANDE ? rbx_audio_setup_stereo(s_snd_rate) : rbx_audio_setup(s_snd_rate)) ||
	    rbx_video_setup(s_vis_w, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		km_exit();
		return -1;
	}
	rbx_audio.frame_mhz = s_gx ? 59620 : (uint32_t)((uint64_t)Z80_TMNT * 1000 / (384 * LINES_TMNT));
	s_pri = (uint8_t *)malloc((size_t)rbx_video.pitch * VIS_H);
	if (!s_pri) {
		rbx_fail("%s: out of memory", g->set);
		km_exit();
		return -1;
	}
	nvram_default(g->machine);
	km_reset();
	s_nv_path[0] = 0;
	{   // (the battery RAM, kept between runs)
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(t.ram, 1, (size_t)s_nvsize, f) != (size_t)s_nvsize) {
					memset(t.ram, 0, sizeof(t.ram));
					nvram_default(g->machine);
				}
				fclose(f);
			}
		}
	}
	return 0;
}

static void km_exit(void)
{
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(t.ram, 1, (size_t)s_nvsize, f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	if (!s_gx)
		k052109_exit(&s_k109);
	free(s_gfx);
	free(s_gfx_row);
	s_gfx_row = NULL;
	free(s_pri);
	s_gfx = NULL;
	s_pri = NULL;
}

static void km_reset(void)
{
	static uint8_t nv[RAM_SIZE];   // (the battery RAM survives a reset; km_init set it first)
	memcpy(nv, t.ram, RAM_SIZE);
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memcpy(t.ram, nv, RAM_SIZE);
	t.ccu_int_time = 31;
	t.nmi_left = 24;   // (the K052109's NMI timer: from line 0)
	t.k53252[8] = 1;
	if (s_gx) {
		const uint8_t *rom = s_k832.rom;
		const uint32_t size = s_k832.rom_size;
		k056832_init(&s_k832, rom, size, K056832_BPP_4, 1, 0);
	} else {
		k052109_reset(&s_k109);
		upd7759_init(&s_upd, s_upd.rom, s_upd.rom_size, UPD_CLOCK, SND_RATE);
	}
	if (s_board == B_TSUKANDE)
		ymz280b_reset(&s_ymz);
	else
		k051649_reset(&s_scc);
	if (s_board == B_DDBOY)
		oki_reset(&s_oki);
	rbx_port_custom(s_port_in[1], 0x10, 0);
	z80_init(&s_z);
	s_z.read_byte = s_gx ? gx_rd : tm_rd;
	s_z.write_byte = s_gx ? gx_wr : tm_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = s_rom + p * 256;
	if (s_gx)
		for (int p = 0; p < 0x20; p++)
			s_zmap[0xa0 + p] = t.ram + p * 256;
	else
		for (int p = 0; p < 8; p++)
			s_zmap[0x80 + p] = t.ram + p * 256;
	bank_set(s_gx ? 4 : 0);
	s_z.read_map = s_zmap;
}

static void km_frame(void)
{
	for (int line = 0; line < s_lines; line++) {
		const int vpos = (VIS_END + line) % s_lines;   // (the frame from the vblank start, as the machine starts)
		t.vpos = vpos;
		if (vpos == VIS_END) {
			capture();
			rbx_video_frame_done();
			if (!s_gx) {
				k052109_vblank(&s_k109, 1);
				t.irq = s_k109.irq_out;
			}
		}
		if (s_gx) {   // (konmedal_scanline, from line 0)
			if (vpos == 0)
				t.scan_on = 1;
			if (t.scan_on) {
				if (vpos == 255)
					t.irq = 1;
				if (--t.ccu_int_time_count <= 0) {
					if (!t.nmi) {   // (ASSERT_LINE: an edge once)
						t.nmi = 1;
						z80_gen_nmi(&s_z);
					}
					t.ccu_int_time_count = t.ccu_int_time;
				}
			}
		} else {
			if (vpos == VIS_Y0)
				k052109_vblank(&s_k109, 0);
			if (!t.nmi_left) {   // (nmi_scanline: every 32 lines, free running)
				if (s_has_nmi && (s_k109.irq_control & 1) && !s_k109.nmi_out) {
					s_k109.nmi_out = 1;
					z80_gen_nmi(&s_z);
				}
				t.nmi_left = 32;
			}
			t.nmi_left--;
		}
		hopper_line();
		t.cyc_frac += (uint64_t)s_cyc_num * (s_gx ? 100 : 1);
		const unsigned long cyc = (unsigned long)(t.cyc_frac / s_line_den);
		t.cyc_frac -= (uint64_t)cyc * s_line_den;
		t.z_end += cyc;
		z80_run(&s_z, (unsigned long)t.z_end, (int *)&t.irq, 0);   // (a level: dropped by the writes that ack it)
		t.snd_frac += s_snd_num;
		const int ns = (int)(t.snd_frac / s_line_den);
		t.snd_frac -= (uint64_t)ns * s_line_den;
		sound_line(ns);
	}
	t.frame++;
}

static int km_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "km", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	if (s_gx)
		rc |= k056832_state(&s_k832, ctx, load);
	else {
		rc |= k052109_state(&s_k109, ctx, load, "km_k109");
		rc |= upd7759_state(&s_upd, ctx, load, "km_upd");
	}
	if (s_board == B_TSUKANDE)
		rc |= rbx_state_io(ctx, "km_ymz", &s_ymz, (uint32_t)YMZ280B_STATE_SIZE);
	else
		rc |= k051649_state(&s_scc, ctx, load, "km_scc");
	if (s_board == B_DDBOY)
		rc |= rbx_state_io(ctx, "km_oki", &s_oki, (uint32_t)offsetof(okim6295_t, rom));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "km_z80", &z, sizeof(z));
	if (!rc && load) {
		z.read_byte = s_gx ? gx_rd : tm_rd;
		z.write_byte = s_gx ? gx_wr : tm_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
		bank_set(t.bank);
		rbx_port_custom(s_port_in[1], 0x10, t.hop_status ? 0x10 : 0);
	}
	return rc ? -1 : 0;
}

static int km_save(void *ctx) { return km_state(ctx, 0); }
static int km_load(void *ctx) { return km_state(ctx, 1); }
static int km_supported(const char *machine)
{
	static const char *const s[] = { "tsururin", "tsupenta", "mariorou", "shuriboy", "fuusenpn", "ddboy", "tsukande" };
	for (size_t i = 0; i < sizeof(s) / sizeof(s[0]); i++)
		if (!strcmp(machine, s[i]))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_konmedal = { "Konami Medal", km_init, km_exit, km_reset, km_frame, km_save, km_load,
                                      km_supported, km_render, NULL };
