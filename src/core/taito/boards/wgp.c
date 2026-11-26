/*  WGP: Real Race Feeling / WGP 2, ported from MAME src/mame/taito/wgp.cpp (BSD-3-Clause, copyright-holders:
    David Graves; thanks to Richard Bush). Two 68000s sharing 16 KB (CPU A enables CPU B through its control port),
    TC0220IOC, ADC0809 (wheel, pedals), TC0100SCN, three "piv" 16x16 tilemaps with row scroll, row zoom and row colour
    banks (MAME piv_layer_draw), zoomed sprites built from 16x16 chunks through a RAM sprite map. The piv / sprite
    rotation is not emulated (as in MAME, where the game is marked not working). LAN not emulated.
    Sound board: Z80 + YM2610 + TC0140SYT (machine/taito_sound.c).
    Controls: the MAME "FAKE" port (digital steering by default: d-pad left / right / up / down, accel = button 1,
    brake = button 2, shift up / down = buttons 4 / 3). With the "Steering type" config set to analogue, or as soon as
    an analog x axis moves, the wheel follows rbx_analog(1, RBX_IPT_AD_STICK_X) (released d-pad / stick re-centres).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../../machine/adc0808.h"
#include "../video/tc0100scn.h"
#include "../../video/zoomgfx.h"

#include <stdlib.h>
#include <string.h>

#define LINES 256
#define SCREEN_W 320
#define VIS_Y0 16
#define VIS_Y1 255
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)
#define CPU_CLOCK 16000000
#define SLICES 2                  // per scanline and CPU (MAME quantum 30 kHz)

static int s_wgp2;
static uint8_t *s_rom, *s_sub_rom;
static uint32_t s_rom_size, s_sub_rom_size;
static uint8_t s_ram[0x10000], s_sub_ram[0x4000], s_shared[0x4000];
static uint16_t (*s_spr_hp)[0x8000];            // 0x400000-0x40ffff: sprite map (0x0000-0x5fff), sprite RAM (0x6000-)
#define s_spr (*s_spr_hp)
static uint16_t (*s_piv_hp)[0xc000];            // 0x500000-0x517fff (pivram = s_piv + 0x1000)
#define s_piv (*s_piv_hp)
static uint16_t s_pal[0x1000];
static uint16_t s_piv_ctrl[0x10];
static tc0100scn_t *s_scn, *s_snap_scn;
static zoomgfx_t s_spr_gfx, s_piv_gfx;
static adc0808_t s_adc;
static uint8_t s_ioc[8];
static int s_port[8], s_port_fake, s_port_steer, s_port_unknown;
static int s_steer_an = 0x80, s_analog_used;
static uint16_t s_cpua_ctrl, s_port_sel, s_rotate[8];
static uint16_t s_piv_scrollx[3], s_piv_scrolly[3], s_piv_ctrl_reg;
static int s_b_halted;                    // CPU B held in reset by CPU A
// CPU B (second Musashi context)
static m68k_bus_t s_bus_b;
static uint8_t s_ctx[2][2048];
static uint32_t s_irq[2];
static int s_cur;
static uint64_t s_frac[2], s_adc_frac;
static int32_t s_b_int6;

static struct {
	uint16_t spr[0x8000];
	uint16_t piv[0xc000];
	uint16_t pal[0x1000];
	uint16_t piv_ctrl[0x10];
	uint16_t scrollx[3], scrolly[3], ctrl_reg;
} *s_snap;
static uint8_t *s_pribuf, *s_scnpri;   // sprite priority (pitch 320), TC0100SCN (bitmap pitch, unused after)

static void cpu_switch(int n)
{
	if (n == s_cur)
		return;
	m68k_get_context(s_ctx[s_cur]);
	s_irq[s_cur] = m68k_bus_irq_pending();
	m68k_set_context(s_ctx[n]);
	m68k_bus_select(n ? &s_bus_b : NULL);
	m68k_bus_set_irq_pending(s_irq[n]);
	s_cur = n;
}

// ---- inputs ----
static uint8_t ioc_read(int o)
{
	switch (o) {
	case 0: case 1: case 2: case 3: case 7: return (uint8_t)rbx_port_read(s_port[o]);
	case 4: return s_ioc[4];
	}
	return 0xff;
}

static int fake_r(void) { return s_port_fake >= 0 ? (int)(rbx_port_read(s_port_fake) & 0xff) : 0; }

static uint8_t steer_r(void)
{
	int fake = fake_r();
	if (s_port_fake < 0)   // joystick versions: no wheel (MAME read_safe(0))
		return 0;
	if (!(fake & 0x10) || s_analog_used)   // analogue: reduce span to 0x80 (STEER is PORT_REVERSE)
		return (uint8_t)(((0xff - s_steer_an) * 0x80) / 0x100);
	int steer = 0x40;
	if (fake & 0x08) steer = 0x20;
	if (fake & 0x04) steer = 0x60;
	if (fake & 0x02) steer = 0x00;
	if (fake & 0x01) steer = 0x80;
	return (uint8_t)steer;
}

static uint8_t adc_input(int ch)
{
	switch (ch) {
	case 0: return (fake_r() & 0x40) ? 0xff : 0x00;   // accel
	case 1: return steer_r();
	case 2: return 0xc0;                               // steer offset
	case 3: return 0xbf;                               // accel offset
	case 4: return (fake_r() & 0x80) ? 0xcf : 0xff;   // brake
	case 5: return s_port_unknown >= 0 ? (uint8_t)rbx_port_read(s_port_unknown) : 0;
	}
	return 0xff;
}

static void update_controls(void)
{
	int a = rbx_analog(1, RBX_IPT_AD_STICK_X);
	if (a) {
		s_analog_used = 1;
		s_steer_an += a;
	} else {
		uint32_t in = rbx_player_input(1);
		if (in & (RBX_IN_LEFT | RBX_IN_RIGHT))
			s_steer_an += (in & RBX_IN_RIGHT) ? 25 : -25;   // MAME keydelta 25
		else if (s_steer_an > 0x80)
			s_steer_an = s_steer_an - 25 < 0x80 ? 0x80 : s_steer_an - 25;
		else if (s_steer_an < 0x80)
			s_steer_an = s_steer_an + 25 > 0x80 ? 0x80 : s_steer_an + 25;
	}
	s_steer_an = s_steer_an < 0 ? 0 : s_steer_an > 0xff ? 0xff : s_steer_an;
}

// ---- CPU A ----
static void piv_ctrl_w(int offset, uint16_t data, uint16_t mask)
{
	offset &= 0xf;
	s_piv_ctrl[offset] = (uint16_t)((s_piv_ctrl[offset] & ~mask) | (data & mask));
	data = s_piv_ctrl[offset];
	if (offset < 3) {   // kill bit 4 of the negated scroll
		uint16_t a = (uint16_t)-data;
		s_piv_scrollx[offset] = (uint16_t)((a & 0xf) | ((a & 0xffe0) >> 1));
	} else if (offset < 6) {
		s_piv_scrolly[offset - 3] = data;
	} else if (offset == 6) {
		s_piv_ctrl_reg = data;
	}
}

static void cpua_ctrl_w(uint16_t data)
{
	if ((data & 0xff00) && (data & 0xff) == 0)
		data = data >> 8;   // WGP
	s_cpua_ctrl = data;
	int halt = !(data & 1);   // parse_control: bit 0 enables CPU B (its reset line)
	if (halt != s_b_halted) {
		s_b_halted = halt;
		if (!halt) {   // reset released: CPU B starts from its reset vector
			int prev = s_cur;
			cpu_switch(1);
			m68k_bus_irq_reset();
			m68k_pulse_reset();
			cpu_switch(prev);
		}
	}
}

static uint16_t a_read16(uint32_t a)
{
	if (a >= 0x180000 && a <= 0x18000f) return (uint16_t)(ioc_read((int)((a >> 1) & 7)) << 8);
	if (a >= 0x200000 && a <= 0x20000f) { m68k_bus_set_line(6, 0); return adc0808_data_r(&s_adc); }
	if (a >= 0x320000 && a <= 0x32000f) return tc0100scn_ctrl_r(s_scn, (int)((a >> 1) & 7));
	if (a >= 0x520000 && a <= 0x52001f) return s_piv_ctrl[(a >> 1) & 0xf];
	return 0;
}

static uint8_t a_read8(uint32_t a)
{
	uint16_t v = a_read16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void a_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x180000 && a <= 0x18000f) {
		if (mask & 0xff00)
			s_ioc[(a >> 1) & 7] = (uint8_t)(v >> 8);   // 0 watchdog, 4 coin lockout / counters
		return;
	}
	if (a == 0x1c0000) { cpua_ctrl_w((uint16_t)((s_cpua_ctrl & ~mask) | (v & mask))); return; }
	if (a >= 0x200000 && a <= 0x20000f) { if (mask & 0xff) adc0808_start(&s_adc, (int)((a >> 1) & 7)); return; }
	if (a >= 0x320000 && a <= 0x32000f) { tc0100scn_ctrl_w(s_scn, (int)((a >> 1) & 7), v, mask); return; }
	if (a >= 0x520000 && a <= 0x52001f) { piv_ctrl_w((int)((a >> 1) & 0xf), v, mask); return; }
	if (a == 0x600000) { s_rotate[s_port_sel & 7] = v; return; }   // rotate_port_w (not emulated)
	if (a == 0x600002) { s_port_sel = v & 7; return; }
}

static void a_write16(uint32_t a, uint16_t v) { a_write16m(a, v, 0xffff); }
static void a_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		a_write16m(a & ~1u, v, 0x00ff);
	else
		a_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- CPU B ----
static uint16_t b_read16(uint32_t a)
{
	if (a == 0x200002) return tc0140syt_master_comm_r(tsnd_syt());
	if (a == 0x380000) return 0x4 << 8;   // lan_status_r
	return 0;
}

static uint8_t b_read8(uint32_t a)
{
	uint16_t v = b_read16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void b_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a == 0x200000 && (mask & 0xff)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)v); return; }
	if (a == 0x200002 && (mask & 0xff)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)v); return; }
}

static void b_write16(uint32_t a, uint16_t v) { b_write16m(a, v, 0xffff); }
static void b_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		b_write16m(a & ~1u, v, 0x00ff);
	else
		b_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- video (MAME wgp.cpp) ----
static const uint8_t xlookup[16] = { 0, 1, 0, 1, 2, 3, 2, 3, 0, 1, 0, 1, 2, 3, 2, 3 };
static const uint8_t ylookup[16] = { 0, 0, 1, 1, 0, 0, 1, 1, 2, 2, 3, 3, 2, 2, 3, 3 };

// the sprites on screen lines y0..y1 (bm: the bitmap's first row)
static void draw_sprites(uint16_t *bm, int pitch, int y0, int y1)
{
	static const uint32_t primasks[2] = { 0x0, 0xfffc };
	const uint16_t *map = s_snap->spr, *ram = s_snap->spr + 0x6000;
	const uint32_t tile_mask = s_spr_gfx.count - 1;
	zoomgfx_target t = { bm, s_pribuf, pitch, 0, y0, SCREEN_W - 1, y1, VIS_Y0, SCREEN_W };
	for (int offs = 0x1ff; offs >= 0; offs--) {
		int code = ram[0xe00 + offs];
		if (!code)
			continue;
		int i = (code << 3) & 0xfff;
		int x = ram[i], y = ram[i + 1];
		int bigsprite = ram[i + 2] & 0x3fff;
		if (ram[i + 4] == 0xfff6 && ram[i + 5] == 0)
			continue;
		int zoomx = (ram[i + 3] & 0x1ff) + 1;
		int zoomy = zoomx;
		y -= 4;
		y -= (0x40 - zoomy) / 4;
		if (x & 0x8000) x -= 0x10000;
		if (y & 0x8000) y -= 0x10000;
		int map_index = bigsprite << 1;
		int si = map[(map_index + 0xa) & 0x7fff], sj = map[(map_index + 0xc) & 0x7fff];
		int small_sprite = si > 0 && si <= 8 && sj > 0 && sj <= 8;
		int n = small_sprite ? 4 : 16, div = small_sprite ? 2 : 4;
		for (int k = 0; k < n; k++) {
			uint32_t tile = map[(map_index + (k << 1)) & 0x7fff] & tile_mask;
			uint16_t attr = map[(map_index + (k << 1) + 1) & 0x7fff];
			uint32_t col = attr & 0xf;
			int priority = (attr & 0x70) >> 4;
			int kx = xlookup[k], ky = ylookup[k];
			int curx = x + ((kx * zoomx) / div);
			int cury = y + ((ky * zoomy) / div);
			int zx = x + (((kx + 1) * zoomx) / div) - curx;
			int zy = y + (((ky + 1) * zoomy) / div) - cury;
			zoomgfx_draw(&s_spr_gfx, &t, tile, col * 16, 0, 0, curx, cury, (uint32_t)zx << 12, (uint32_t)zy << 12, 1,
			             primasks[(priority >> 1) & 1]);
		}
	}
}

// screen lines y0..y1 of the layer (bm: the bitmap's first row)
static void piv_layer_draw(uint16_t *bm, int pitch, int layer, int opaque, uint8_t priority, int y0, int y1)
{
	const uint16_t *pivram = s_snap->piv + 0x1000;
	const uint32_t zoomx = 0x10000;
	const int width_mask = 0x3ff;
	uint32_t zoomy = 0x10000 - (((s_snap->piv_ctrl[0x08 + layer] & 0xff) - 0x7f) * 512);
	int sx = (int)((uint32_t)s_snap->scrollx[layer] << 16) + 32 * (int)zoomx;
	int y_index = (int)(((uint32_t)s_snap->scrolly[layer] << 16) + (uint32_t)(16 + y0) * zoomy);
	for (int y = y0; y <= y1; y++, y_index += (int)zoomy) {
		int src_y = (y_index >> 16) & 0x3ff;
		uint16_t rc = pivram[src_y + layer * 0x400 + 0x3400];
		int row_zoom = rc & 0xff;
		uint16_t row_colbank = rc >> 8;
		row_colbank = (uint16_t)((((row_colbank & 0xf) << 1) | (row_colbank & 0xe0)) << 4);
		uint16_t row_scroll = pivram[src_y + layer * 0x1000 + 0x4000];
		row_scroll = (uint16_t)(((row_scroll & 0xf) | ((row_scroll & 0xffe0) >> 1)) & width_mask);
		int x_index = sx - (row_scroll << 16);
		int x_step = (int)zoomx;
		if (row_zoom > 0x7f)
			x_step -= ((row_zoom - 0x7f) << 8) & 0xffff;
		else if (row_zoom < 0x7f)
			x_step += ((0x7f - row_zoom) << 8) & 0xffff;
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		uint8_t *p = s_pribuf + (y - VIS_Y0) * SCREEN_W;
		const uint16_t *maprow = pivram + layer * 0x1000 + (src_y >> 4) * 64;
		int last_col = -1, fx = 0;
		const uint8_t *trow = NULL;
		uint16_t base = 0;
		for (int i = 0; i < SCREEN_W; i++, x_index += x_step) {
			int px = (x_index >> 16) & width_mask;
			if ((px >> 4) != last_col) {   // tile of this pixel (cached along the row)
				last_col = px >> 4;
				uint16_t code = maprow[last_col] & 0x3fff;
				uint16_t attr = maprow[last_col + 0x8000];
				fx = (attr & 0x40) ? 15 : 0;
				int fy = (attr & 0x80) ? 15 : 0;
				uint32_t c = rbx_wrap(code, s_piv_gfx.count);
				trow = s_piv_gfx.opaque[c] || opaque ? s_piv_gfx.pix + (size_t)c * 256 + (((src_y & 15) ^ fy) << 4) : NULL;
				base = (uint16_t)((attr & 0x3f) * 16 + row_colbank);
			}
			if (!trow)
				continue;
			uint8_t pen = trow[(px & 15) ^ fx];
			if (pen) {
				d[i] = (uint16_t)((base + pen) & 0xfff);
				p[i] |= priority;
			} else if (opaque) {
				d[i] = (uint16_t)(base & 0xfff);   // MAME's pixmap keeps the tile colour for pen 0 too
				p[i] |= priority;
			}
		}
	}
}

// the frame is drawn in bands of lines (wgp_help: by the threads that help the render thread too)
#define BANDS 8
#define BAND_H (VIS_H / BANDS)
static int s_layer[3];
static int s_job_next, s_job_left, s_job_active, s_helpers;

static void draw_band(int b)
{
	const int y0 = VIS_Y0 + b * BAND_H, y1 = y0 + BAND_H - 1;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = y0 - VIS_Y0; y <= y1 - VIS_Y0; y++)
		memset(bm + y * pitch, 0, SCREEN_W * 2);
	memset(s_pribuf + (y0 - VIS_Y0) * SCREEN_W, 0, SCREEN_W * BAND_H);
	piv_layer_draw(bm, pitch, s_layer[0], 1, 1, y0, y1);
	piv_layer_draw(bm, pitch, s_layer[1], 0, 2, y0, y1);
	piv_layer_draw(bm, pitch, s_layer[2], 0, 4, y0, y1);
	draw_sprites(bm, pitch, y0, y1);
	// TC0100SCN on top (its priority bits are not used afterwards)
	uint16_t *bb = bm + (y0 - VIS_Y0) * pitch;
	uint8_t *pb = s_scnpri + (y0 - VIS_Y0) * pitch;
	int bl = tc0100scn_bottomlayer(s_snap_scn);
	tc0100scn_draw(s_snap_scn, bl, bb, pb, pitch, y0, BAND_H, 0);
	tc0100scn_draw(s_snap_scn, bl ^ 1, bb, pb, pitch, y0, BAND_H, 0);
	tc0100scn_draw(s_snap_scn, 2, bb, pb, pitch, y0, BAND_H, 0);
}

// bands until none is left (one: a single band); the number drawn
static int draw_bands(int one)
{
	int n = 0;
	for (;;) {
		const int b = __atomic_fetch_add(&s_job_next, 1, __ATOMIC_ACQ_REL);
		if (b >= BANDS)
			return n;
		draw_band(b);
		__atomic_sub_fetch(&s_job_left, 1, __ATOMIC_ACQ_REL);
		n++;
		if (one)
			return n;
	}
}

static int wgp_help(int helper)
{
	if (!__atomic_load_n(&s_job_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_job_active, __ATOMIC_SEQ_CST))
		did = draw_bands(helper & 0x100) > 0;
	__atomic_sub_fetch(&s_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

static void wgp_render(void)
{
	for (int i = 0; i < 0x1000; i++) {   // RGBx_444
		uint16_t v = s_snap->pal[i];
		int r = (v >> 12) & 15, g = (v >> 8) & 15, b = (v >> 4) & 15;
		rbx_video.palette[i] = (uint32_t)((r * 17) << 16 | (g * 17) << 8 | (b * 17));
	}
	s_layer[0] = 0;
	s_layer[1] = s_snap->ctrl_reg == 0x2d ? 2 : 1;
	s_layer[2] = s_snap->ctrl_reg == 0x2d ? 1 : 2;
	s_job_next = 0;
	s_job_left = BANDS;
	__atomic_store_n(&s_job_active, 1, __ATOMIC_SEQ_CST);
	draw_bands(0);
	while (__atomic_load_n(&s_job_left, __ATOMIC_ACQUIRE)) {
	}
	__atomic_store_n(&s_job_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_helpers, __ATOMIC_SEQ_CST)) {   // (none may still take a band of the next frame)
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, s_spr, sizeof(s_spr));
	memcpy(s_snap->piv, s_piv, sizeof(s_piv));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->piv_ctrl, s_piv_ctrl, sizeof(s_piv_ctrl));
	memcpy(s_snap->scrollx, s_piv_scrollx, sizeof(s_piv_scrollx));
	memcpy(s_snap->scrolly, s_piv_scrolly, sizeof(s_piv_scrolly));
	s_snap->ctrl_reg = s_piv_ctrl_reg;
	tc0100scn_copy(s_snap_scn, s_scn);
	rbx_video_frame_done();
}

// ---- the CPUs' waits ----
// Each CPU waits for its interrupt in L: addq.w #q,(c).l or addi.w #i,(c).l / bra.s L (a counter in its RAM stepped
// until then). The passes that fit in the slice but the last are done here (the counter only: the last pass sets the
// flags), the rest by Musashi; the interrupts come between the slices
typedef struct { uint32_t l, b, c; uint16_t add; int cyc; } idle_loop;   // (b: the bra.s)
static idle_loop s_idle[2];   // per CPU (l = 0: none)

static void idle_find(int cpu)   // (that CPU's bus selected)
{
	idle_loop *w = &s_idle[cpu];
	memset(w, 0, sizeof(*w));
#ifndef __vita__
	if (getenv("RBX_NOIDLE"))   // (PC harness: Musashi only, to check the result)
		return;
#endif
	for (uint32_t l = 0x400; l < 0x40000; l += 2) {
		const uint16_t op = (uint16_t)m68k_read_memory_16(l);
		uint32_t c, len;
		uint16_t add;
		if ((op & 0xf1ff) == 0x5079) {   // addq.w #q,(xxx).l
			add = (op >> 9) & 7 ? (op >> 9) & 7 : 8;
			c = m68k_read_memory_32(l + 2);
			len = 6;
		} else if (op == 0x0679) {         // addi.w #i,(xxx).l
			add = (uint16_t)m68k_read_memory_16(l + 2);
			c = m68k_read_memory_32(l + 4);
			len = 8;
		} else
			continue;
		if (m68k_read_memory_16(l + len) != (uint16_t)(0x6000 | (uint8_t)(-(int)len - 2)) || c < 0x100000 || c > 0x13fffe ||
		    (c & 1))
			continue;
		w->l = l;
		w->b = l + len;
		w->c = c;
		w->add = add;
		rbx_log("wgp: CPU %c waits at %06x (counter %06x)", 'A' + cpu, l, c);
		return;
	}
}

// the passes of the current CPU's wait done here; the cycles used
static int idle_run(int want)
{
	idle_loop *w = &s_idle[s_cur];
	if (!w->l)
		return 0;
	uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	if (pc != w->l && pc != w->b)
		return 0;
	int used = pc == w->b ? m68k_execute(1) : 0;   // (to L)
	if (m68k_get_reg(NULL, M68K_REG_PC) != w->l)
		return used;
	if (!w->cyc) {   // (a pass's cycles, from Musashi once; the passes as Musashi would do them: within the slice)
		if (want - used < 200)
			return used;
		const int c = m68k_execute(1) + m68k_execute(1);
		used += c;
		if (m68k_get_reg(NULL, M68K_REG_PC) != w->l)
			return used;
		w->cyc = c;
	}
	const int k = (want - used - 1) / w->cyc - 1;   // (one pass and more left to Musashi)
	if (k <= 0)
		return used;
	m68k_write_memory_16(w->c, (uint16_t)(m68k_read_memory_16(w->c) + (uint32_t)k * w->add));
	return used + k * w->cyc;
}

static int exec(int cycles)
{
	int used = idle_run(cycles);
	if (used < cycles)
		used += m68k_execute(cycles - used);
	return used;
}

// ---- init / frame ----
static void wgp_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_sub_ram, 0, sizeof(s_sub_ram));
	memset(s_shared, 0, sizeof(s_shared));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_piv, 0, sizeof(s_piv));
	memset(s_piv_ctrl, 0, sizeof(s_piv_ctrl));
	memset(s_ioc, 0, sizeof(s_ioc));
	memset(s_rotate, 0, sizeof(s_rotate));
	memset(s_piv_scrollx, 0, sizeof(s_piv_scrollx));
	memset(s_piv_scrolly, 0, sizeof(s_piv_scrolly));
	s_piv_ctrl_reg = 0;
	s_cpua_ctrl = 0xff;   // machine_reset (CPU B running)
	s_port_sel = 0;
	s_b_halted = 0;
	tc0100scn_reset(s_scn);
	adc0808_reset(&s_adc);
	s_cur = 0;
	m68k_bus_select(&s_bus_b);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	m68k_get_context(s_ctx[1]);
	s_irq[1] = 0;
	idle_find(1);
	m68k_bus_select(NULL);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_irq[0] = 0;
	idle_find(0);
	tsnd_reset();
	s_frac[0] = s_frac[1] = s_adc_frac = 0;
	s_b_int6 = 0;
	s_steer_an = 0x80;
	s_analog_used = 0;
}

static void wgp_exit(void);

static int wgp_init(const rbx_game *g)
{
	if (!s_piv_hp && !(s_piv_hp = calloc(1, sizeof(*s_piv_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_spr_hp && !(s_spr_hp = calloc(1, sizeof(*s_spr_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_wgp2 = !strcmp(g->machine, "wgp2");
	uint32_t scn_size, spr_size, piv_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_sub_rom = rbx_region("sub", &s_sub_rom_size);
	uint8_t *scn_gfx = rbx_region("tc0100scn", &scn_size);
	uint8_t *spr = rbx_region("sprites", &spr_size);
	uint8_t *piv = rbx_region("piv", &piv_size);
	if (!s_rom || !s_sub_rom || !scn_gfx || !spr || !piv) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_bus_swap16(s_sub_rom, s_sub_rom_size);
	if (s_wgp2 && s_sub_rom_size > 0x8011) {   // init_wgp2: skip the failing memory checks
		*(uint16_t *)(s_sub_rom + 0x8008) = 0;
		*(uint16_t *)(s_sub_rom + 0x8010) = 0;
	}
	s_scn = (tc0100scn_t *)calloc(1, sizeof(tc0100scn_t));
	s_snap_scn = (tc0100scn_t *)malloc(sizeof(tc0100scn_t));
	s_snap = calloc(1, sizeof(*s_snap));
	s_pribuf = (uint8_t *)calloc(SCREEN_W, VIS_H);
	if (!s_scn || !s_snap_scn || !s_snap || !s_pribuf || tc0100scn_init(s_scn, scn_gfx, scn_size, TC0100SCN_4BPP) ||
	    zoomgfx_decode(&s_spr_gfx, spr, spr_size, ZGFX_16x16_PACKED_LSB) ||
	    zoomgfx_decode(&s_piv_gfx, piv, piv_size, ZGFX_16x16_PACKED_LSB)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		wgp_exit();
		return -1;
	}
	if (s_wgp2) {
		s_scn->x_offset = 4;
		s_scn->y_offset = 2;
	}
	s_scn->bg_colbank[0] = 0x80;   // set_colbanks(0x80, 0xc0, 0x40)
	s_scn->bg_colbank[1] = 0xc0;
	s_scn->tx_colbank = 0x40;
	s_scn->screen_h = VIS_Y0 + VIS_Y1 + 1;
	*s_snap_scn = *s_scn;
	static const char *const tags[8] = { "DSWA", "DSWB", "IN0", "IN1", 0, 0, 0, "IN2" };
	for (int i = 0; i < 8; i++)
		s_port[i] = tags[i] ? rbx_port_find(tags[i]) : -1;
	s_port_fake = rbx_port_find("FAKE");
	s_port_steer = rbx_port_find("STEER");
	s_port_unknown = rbx_port_find("UNKNOWN");
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_E000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	sc.ym.type = RBX_YM2610;
	sc.ym.clock = 8000000;
	sc.ym.gain[0] = 192;
	sc.ym.gain[1] = sc.ym.gain[2] = 128;   // (MAME's FM left and right at 1.0 each, averaged on one speaker)
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		wgp_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, VIS_H, 4096) || m68k_context_size() > sizeof(s_ctx[0]) ||
	    !(s_scnpri = (uint8_t *)calloc((size_t)rbx_video.pitch, VIS_H))) {
		wgp_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_select(NULL);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x100000, 0x10ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x140000, 0x143fff, s_shared, sizeof(s_shared), 1, 1);
	m68k_bus_map(0x300000, 0x30ffff, s_scn->ram, 0x10000, 1, 1);
	m68k_bus_map(0x400000, 0x40ffff, s_spr, sizeof(s_spr), 1, 1);
	m68k_bus_map(0x500000, 0x517fff, s_piv, sizeof(s_piv), 1, 1);
	m68k_bus_map(0x700000, 0x701fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = a_read8;
	m68k_bus.read16 = a_read16;
	m68k_bus.write8 = a_write8;
	m68k_bus.write16 = a_write16;
	m68k_bus_select(&s_bus_b);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x03ffff, s_sub_rom, s_sub_rom_size, 1, 0);
	m68k_bus_map(0x100000, 0x103fff, s_sub_ram, sizeof(s_sub_ram), 1, 1);
	m68k_bus_map(0x140000, 0x143fff, s_shared, sizeof(s_shared), 1, 1);
	s_bus_b.read8 = b_read8;
	s_bus_b.read16 = b_read16;
	s_bus_b.write8 = b_write8;
	s_bus_b.write16 = b_write16;
	m68k_bus_select(NULL);
	wgp_reset();
	return 0;
}

static void wgp_exit(void)
{
	m68k_bus_select(NULL);
	if (s_scn)
		tc0100scn_exit(s_scn);
	free(s_scn);
	free(s_snap_scn);
	s_scn = s_snap_scn = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_pribuf);
	free(s_scnpri);
	s_pribuf = s_scnpri = NULL;
	zoomgfx_free(&s_spr_gfx);
	zoomgfx_free(&s_piv_gfx);
	tsnd_exit();
	free(s_piv_hp);
	s_piv_hp = NULL;
	free(s_spr_hp);
	s_spr_hp = NULL;
}

static void run_b(int cycles)
{
	if (s_b_halted)
		return;
	cpu_switch(1);
	if (s_b_int6 > 0 && s_b_int6 <= cycles) {   // trigger_cpu_b_int6
		int done = exec(s_b_int6);
		m68k_bus_hold_irq(6);
		s_b_int6 = 0;
		if (cycles - done > 0)
			exec(cycles - done);
	} else {
		exec(cycles);
		if (s_b_int6 > 0)
			s_b_int6 -= cycles;
	}
}

static void wgp_frame(void)
{
	update_controls();
	for (int line = 0; line < LINES; line++) {
		if (line == 0) {   // vblank
			vblank_capture();
			cpu_switch(0);
			m68k_bus_hold_irq(4);
			if (!s_b_halted) {
				cpu_switch(1);
				m68k_bus_hold_irq(4);   // cpub_interrupt
				s_b_int6 = 200000 - 500;
			}
		}
		int ca = rbx_line_cycles(&s_frac[0], CPU_CLOCK, 60000, LINES);
		int cb = rbx_line_cycles(&s_frac[1], CPU_CLOCK, 60000, LINES);
		for (int s = 0; s < SLICES; s++) {
			int a = s ? ca - ca / SLICES : ca / SLICES, b = s ? cb - cb / SLICES : cb / SLICES;
			cpu_switch(0);
			exec(a);
			run_b(b);
		}
		cpu_switch(0);
		if (adc0808_advance(&s_adc, rbx_line_cycles(&s_adc_frac, 500000, 60000, LINES))) {
			s_adc.sar = adc_input(s_adc.addr);
			m68k_bus_set_line(6, 1);   // EOC flip-flop on IRQ6 of CPU A (level)
		}
		tsnd_line();
	}
	tsnd_frame_end();
}

static int wgp_state(void *ctx, int load)
{
	cpu_switch(0);
	if (!load) {
		m68k_get_context(s_ctx[0]);
		s_irq[0] = m68k_bus_irq_pending();
	}
	uint32_t csize = m68k_context_size();
	int misc[12] = { s_cpua_ctrl, s_port_sel, s_piv_ctrl_reg, s_b_halted, s_b_int6, s_steer_an, s_analog_used,
	                 s_piv_scrollx[0], s_piv_scrollx[1], s_piv_scrollx[2], 0, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", s_ctx[0], csize);
	rc |= rbx_state_io(ctx, "m68kb", s_ctx[1], csize);
	rc |= rbx_state_io(ctx, "irq", s_irq, sizeof(s_irq));
	rc |= rbx_state_io(ctx, "irqline", &m68k_bus.irq_asserted, sizeof(m68k_bus.irq_asserted));   // level lines (CPU A)
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "subram", s_sub_ram, sizeof(s_sub_ram));
	rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "piv", s_piv, sizeof(s_piv));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "pivctrl", s_piv_ctrl, sizeof(s_piv_ctrl));
	rc |= rbx_state_io(ctx, "scrolly", s_piv_scrolly, sizeof(s_piv_scrolly));
	rc |= rbx_state_io(ctx, "rotate", s_rotate, sizeof(s_rotate));
	rc |= rbx_state_io(ctx, "scn", s_scn->ram, sizeof(s_scn->ram));
	rc |= rbx_state_io(ctx, "scnc", s_scn->ctrl, sizeof(s_scn->ctrl));
	rc |= rbx_state_io(ctx, "ioc", s_ioc, sizeof(s_ioc));
	rc |= rbx_state_io(ctx, "adc", &s_adc, sizeof(s_adc));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "frac", s_frac, sizeof(s_frac));
	rc |= rbx_state_io(ctx, "afrac", &s_adc_frac, sizeof(s_adc_frac));
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(s_ctx[0]);
		m68k_bus_set_irq_pending(s_irq[0]);
		s_cpua_ctrl = (uint16_t)misc[0]; s_port_sel = (uint16_t)misc[1]; s_piv_ctrl_reg = (uint16_t)misc[2];
		s_b_halted = misc[3]; s_b_int6 = misc[4]; s_steer_an = misc[5]; s_analog_used = misc[6];
		for (int i = 0; i < 3; i++)
			s_piv_scrollx[i] = (uint16_t)misc[7 + i];
	}
	return 0;
}

static int wgp_save(void *ctx) { return wgp_state(ctx, 0); }
static int wgp_load(void *ctx) { return wgp_state(ctx, 1); }
static int wgp_supported(const char *machine) { return !strcmp(machine, "wgp") || !strcmp(machine, "wgp2"); }

const rbx_driver rbx_drv_wgp = { "World Grand Prix", wgp_init, wgp_exit, wgp_reset, wgp_frame, wgp_save, wgp_load,
                                 wgp_supported, wgp_render, wgp_help };
