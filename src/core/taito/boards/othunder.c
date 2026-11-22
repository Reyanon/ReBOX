/*  Operation Thunderbolt, ported from MAME src/mame/taito/othunder.cpp and othunder_v.cpp (BSD-3-Clause,
    copyright-holders: David Graves; thanks to Richard Bush). 68000 + TC0220IOC + EEPROM 93C46 + ADC0808 (light guns)
    + TC0100SCN + TC0110PCR palette + zoomed 16x8 sprite chunks built into 64x64 sprites through the sprite map ROM;
    sound board: Z80 + YM2610 + TC0140SYT (machine/taito_sound.c). The TC0310FAM panning is not mixed (mono).
    The screen is mirrored horizontally (MAME ORIENTATION_FLIP_X), here in render().
    Light gun: the crosshair of each player (MAME P1X / P1Y, 0-255 on the displayed screen) follows
    rbx_analog(player, RBX_IPT_LIGHTGUN_X / Y) (the d-pad when there is no pointing device); fire = button 1,
    rocket = button 2.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../../machine/eeprom93c46.h"
#include "../../machine/adc0808.h"
#include "../video/tc0100scn.h"
#include "../../video/zoomgfx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINES 262
#define RATE_MHZ 60056          // 26.686 MHz / 4 / 424 / 262
#define SCREEN_W 320
#define VIS_Y0 16
#define VIS_Y1 255
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)
#define VBLANK_LINE 256

static uint8_t *s_rom;
static uint32_t s_rom_size;
static const uint8_t *s_sprmap;
static uint32_t s_sprmap_words;
static uint8_t (*s_ram_hp)[0x10000];
#define s_ram (*s_ram_hp)
static uint16_t s_spr[0x800];          // 0x400000-0x4005ff (one 4KB page mapped)
static uint16_t s_pcr[0x1000];         // TC0110PCR RAM
static uint16_t s_pcr_addr;
static tc0100scn_t *s_scn, *s_snap_scn;
static zoomgfx_t s_gfx;
static eeprom93c46_t s_eeprom;
static char s_nv_path[512];
static adc0808_t s_adc;
static uint8_t s_ioc[8];
static int s_port[8];                  // IOC 0 DSWA, 1 DSWB, 2 IN0, 7 IN2
static int s_gun[4] = { 0x80, 0x80, 0x80, 0x80 };   // P1X P1Y P2X P2Y
static uint64_t s_cpu_frac, s_adc_frac;
static int s_port_p2_active = -1;   // the P2 crosshair is drawn once player 2 moved it

static struct {
	uint16_t spr[0x300];
	uint16_t pcr[0x1000];
	int gun[4], p2;
} *s_snap;
static uint8_t *s_pribuf;

typedef struct { uint32_t code, color; int flipx, flipy, x, y, zx, zy; uint32_t primask; } tempsprite_t;
static tempsprite_t (*s_list_hp)[0x2000];
#define s_list (*s_list_hp)

// ---- I/O ----
static uint8_t ioc_read(int o)
{
	switch (o) {
	case 0: return (uint8_t)rbx_port_read(s_port[0]);
	case 1: return (uint8_t)rbx_port_read(s_port[1]);
	case 2: return (uint8_t)rbx_port_read(s_port[2]);
	case 3: return (uint8_t)(eeprom_do(&s_eeprom) << 7);
	case 4: return s_ioc[4];
	case 7: return (uint8_t)rbx_port_read(s_port[7]);
	}
	return 0xff;
}

static void ioc_write(int o, uint8_t v)
{
	s_ioc[o & 7] = v;
	if (o == 3)   // eeprom_w: bit 6 data in, bit 5 clock, bit 4 chip select
		eeprom_write_lines(&s_eeprom, (v >> 4) & 1, (v >> 5) & 1, (v >> 6) & 1);
}

static void update_gun(void)
{
	for (int p = 0; p < 2; p++)
		for (int axis = 0; axis < 2; axis++) {
			int *g = &s_gun[p * 2 + axis];
			int v = *g + rbx_analog(p + 1, axis ? RBX_IPT_LIGHTGUN_Y : RBX_IPT_LIGHTGUN_X);
			*g = v < 0 ? 0 : v > 255 ? 255 : v;
			if (p == 1 && *g != 0x80)
				s_port_p2_active = 1;
		}
}

static uint16_t main_read16(uint32_t a)
{
	if (a >= 0x090000 && a <= 0x09000f) return ioc_read((int)((a >> 1) & 7));
	if (a >= 0x100000 && a <= 0x100007) return ((a >> 1) & 3) == 1 ? s_pcr[s_pcr_addr] : 0xff;
	if (a >= 0x220000 && a <= 0x22000f) return tc0100scn_ctrl_r(s_scn, (int)((a >> 1) & 7));
	if (a == 0x300002) return tc0140syt_master_comm_r(tsnd_syt());
	if (a >= 0x500000 && a <= 0x500007) return adc0808_data_r(&s_adc);
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	if (a & 1) {
		if (a == 0x300003) return tc0140syt_master_comm_r(tsnd_syt());
		return (uint8_t)main_read16(a & ~1u);
	}
	if ((a >= 0x090000 && a <= 0x09000f) || (a >= 0x500000 && a <= 0x500007) || (a >= 0x300000 && a <= 0x300003))
		return 0;
	return (uint8_t)(main_read16(a) >> 8);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x090000 && a <= 0x09000f) { if (mask & 0xff) ioc_write((int)((a >> 1) & 7), (uint8_t)v); return; }
	if (a >= 0x100000 && a <= 0x100007) {   // TC0110PCR, shift 0
		int o = (int)((a >> 1) & 3);
		if (o == 0) s_pcr_addr = (uint16_t)(v & 0xfff);
		else if (o == 1) s_pcr[s_pcr_addr] = v;
		return;
	}
	if (a >= 0x220000 && a <= 0x22000f) { tc0100scn_ctrl_w(s_scn, (int)((a >> 1) & 7), v, mask); return; }
	if (a == 0x300000 && (mask & 0xff)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)v); return; }
	if (a == 0x300002 && (mask & 0xff)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)v); return; }
	if (a >= 0x500000 && a <= 0x500007) { if (mask & 0xff) adc0808_start(&s_adc, (int)((a >> 1) & 3)); return; }
	if (a == 0x600000) { m68k_bus_set_line(5, 0); return; }   // irq_ack_w
	if (a == 0x600002) { m68k_bus_set_line(6, 0); return; }
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_write16m(a & ~1u, v, 0x00ff);
	else
		main_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- video (MAME othunder_v.cpp) ----
static void draw_sprites(uint16_t *bm, int pitch, const uint32_t *primasks, int y_offs)
{
	const uint16_t *sp = s_snap->spr;
	const uint32_t tile_mask = s_gfx.count - 1;
	tempsprite_t *s = s_list;
	for (int offs = 0x300 - 4; offs >= 0; offs -= 4) {
		uint16_t data = sp[offs + 0];
		int zoomy = (data & 0xfe00) >> 9;
		int y = data & 0x1ff;
		data = sp[offs + 1];
		int flipx = (data & 0x4000) >> 14;
		int priority = (data & 0x8000) >> 15;
		int x = data & 0x1ff;
		data = sp[offs + 2];
		uint32_t color = (data & 0xff00) >> 8;
		int zoomx = data & 0x7f;
		data = sp[offs + 3];
		uint32_t tilenum = data & 0x1fff;
		int flipy = (data & 0x8000) >> 15;
		if (!tilenum)
			continue;
		uint32_t map_offset = tilenum << 5;
		zoomx += 1;
		zoomy += 1;
		y += y_offs;
		if (x > 0x140) x -= 0x200;
		if (y > 0x140) y -= 0x200;
		for (int chunk = 0; chunk < 32; chunk++) {
			int k = chunk % 4, j = chunk / 4;
			int px = flipx ? 3 - k : k;
			int py = flipy ? 7 - j : j;
			uint32_t mo = map_offset + (uint32_t)px + ((uint32_t)py << 2);
			uint16_t code = mo < s_sprmap_words ? (uint16_t)(s_sprmap[mo * 2] | s_sprmap[mo * 2 + 1] << 8) : 0;
			code &= (uint16_t)tile_mask;
			int curx = x + ((k * zoomx) / 4);
			int cury = y + ((j * zoomy) / 8);
			int zx = x + (((k + 1) * zoomx) / 4) - curx;
			int zy = y + (((j + 1) * zoomy) / 8) - cury;
			if (s >= s_list + 0x2000)
				break;
			s->code = code; s->color = color; s->flipx = flipx; s->flipy = flipy;
			s->x = curx; s->y = cury; s->zx = zx << 12; s->zy = zy << 13;
			s->primask = primasks[priority];
			s++;
		}
	}
	zoomgfx_target t = { bm, s_pribuf, pitch, 0, VIS_Y0, SCREEN_W - 1, VIS_Y1, VIS_Y0 };
	while (s != s_list) {
		s--;
		zoomgfx_draw(&s_gfx, &t, s->code, s->color * 16, s->flipx, s->flipy, s->x, s->y, (uint32_t)s->zx, (uint32_t)s->zy,
		             1, s->primask);
	}
}

static void othunder_render(void)
{
	rbx_video.palette[4096] = 0xffffff;
	rbx_video.palette[4097] = 0x40ff40;
	for (int i = 0; i < 4096; i++) {   // color_xrgb555
		uint16_t d = s_snap->pcr[i];
		int r = (d >> 10) & 31, g = (d >> 5) & 31, b = d & 31;
		rbx_video.palette[i] = (uint32_t)(((r << 3) | (r >> 2)) << 16 | ((g << 3) | (g >> 2)) << 8 | ((b << 3) | (b >> 2)));
	}
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++)
		memset(bm + y * pitch, 0, SCREEN_W * 2);
	memset(s_pribuf, 0, (size_t)pitch * VIS_H);
	int layer0 = tc0100scn_bottomlayer(s_snap_scn);
	tc0100scn_draw_ex(s_snap_scn, layer0, bm, s_pribuf, pitch, VIS_Y0, VIS_H, 1, TC0100SCN_OPAQUE);
	tc0100scn_draw(s_snap_scn, layer0 ^ 1, bm, s_pribuf, pitch, VIS_Y0, VIS_H, 2);
	tc0100scn_draw(s_snap_scn, 2, bm, s_pribuf, pitch, VIS_Y0, VIS_H, 4);
	static const uint32_t primasks[2] = { 0xf0, 0xfc };
	draw_sprites(bm, pitch, primasks, 3);
	// ORIENTATION_FLIP_X
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *r = bm + y * pitch;
		for (int x = 0; x < SCREEN_W / 2; x++) {
			uint16_t t = r[x];
			r[x] = r[SCREEN_W - 1 - x];
			r[SCREEN_W - 1 - x] = t;
		}
	}
	// the game draws no crosshair: draw MAME's at the light gun positions (PORT_CROSSHAIR(Y, 1.0, -0.057, 0))
	for (int p = 0; p < 2; p++) {
		int cx = s_snap->gun[p * 2] * (SCREEN_W - 1) / 255;
		int cy = (int)((s_snap->gun[p * 2 + 1] / 255.0 - 0.057) * (VIS_H - 1));
		uint16_t col = (uint16_t)(4096 + p);
		for (int i = -6; i <= 6; i++) {
			if (i > -2 && i < 2)
				continue;
			if (cx + i >= 0 && cx + i < SCREEN_W && cy >= 0 && cy < VIS_H)
				bm[cy * pitch + cx + i] = col;
			if (cy + i >= 0 && cy + i < VIS_H && cx >= 0 && cx < SCREEN_W)
				bm[(cy + i) * pitch + cx] = col;
		}
		if (p == 0 && !s_snap->p2)
			break;
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, s_spr, sizeof(s_snap->spr));
	memcpy(s_snap->pcr, s_pcr, sizeof(s_snap->pcr));
	memcpy(s_snap->gun, s_gun, sizeof(s_gun));
	s_snap->p2 = s_port_p2_active > 0;
	tc0100scn_copy(s_snap_scn, s_scn);
	rbx_video_frame_done();
}

// ---- init / frame ----
static void othunder_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pcr, 0, sizeof(s_pcr));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_ioc, 0, sizeof(s_ioc));
	s_pcr_addr = 0;
	tc0100scn_reset(s_scn);
	adc0808_reset(&s_adc);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	tsnd_reset();
	s_cpu_frac = s_adc_frac = 0;
	for (int i = 0; i < 4; i++)
		s_gun[i] = 0x80;
	s_port_p2_active = -1;
}

static void othunder_exit(void);

static int othunder_init(const rbx_game *g)
{
	if (!s_list_hp && !(s_list_hp = calloc(1, sizeof(*s_list_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *scn_gfx = rbx_region("tc0100scn", &size);
	uint32_t ssize;
	uint8_t *spr = rbx_region("sprites", &ssize);
	uint32_t msize;
	s_sprmap = rbx_region("sprmap_rom", &msize);
	s_sprmap_words = msize / 2;
	if (!s_rom || !scn_gfx || !spr || !s_sprmap) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_scn = (tc0100scn_t *)calloc(1, sizeof(tc0100scn_t));
	s_snap_scn = (tc0100scn_t *)malloc(sizeof(tc0100scn_t));
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_scn || !s_snap_scn || !s_snap || tc0100scn_init(s_scn, scn_gfx, size, TC0100SCN_4BPP) ||
	    zoomgfx_decode(&s_gfx, spr, ssize, ZGFX_16x8_PLANAR64)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		othunder_exit();
		return -1;
	}
	s_scn->x_offset = 4;
	s_scn->y_offset = 0;
	s_scn->screen_h = VIS_Y0 + VIS_Y1 + 1;
	*s_snap_scn = *s_scn;
	static const char *const tags[8] = { "DSWA", "DSWB", "IN0", 0, 0, 0, 0, "IN2" };
	for (int i = 0; i < 8; i++)
		s_port[i] = tags[i] ? rbx_port_find(tags[i]) : -1;
	// EEPROM: factory contents from the set, then the saved NVRAM
	eeprom_init(&s_eeprom);
	uint32_t esize;
	uint8_t *e = rbx_region("eeprom", &esize);
	for (uint32_t i = 0; e && i < 64 && i * 2 + 1 < esize; i++)
		s_eeprom.data[i] = rbx_be16(e + i * 2);
	s_nv_path[0] = 0;
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	if (dir) {
		snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
		eeprom_load(&s_eeprom, s_nv_path);
	}
	// sound: YM2610 SSG 0.75, FM 1.0 (MAME routes before the TC0310FAM volumes)
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_E000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = RATE_MHZ;
	sc.lines = LINES;
	sc.ym.type = RBX_YM2610;
	sc.ym.clock = 8000000;
	sc.ym.gain[0] = 192;
	sc.ym.gain[1] = sc.ym.gain[2] = 256;
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		othunder_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, VIS_H, 8192)) {   // power of two: 4096 + crosshair colors
		othunder_exit();
		return -1;
	}
	s_pribuf = (uint8_t *)calloc((size_t)rbx_video.pitch, VIS_H);
	if (!s_pribuf) {
		othunder_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x07ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x080000, 0x08ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x200000, 0x20ffff, s_scn->ram, 0x10000, 1, 1);
	m68k_bus_map(0x400000, 0x400fff, s_spr, sizeof(s_spr), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	othunder_reset();
	return 0;
}

static void othunder_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	if (s_scn)
		tc0100scn_exit(s_scn);
	free(s_scn);
	free(s_snap_scn);
	s_scn = s_snap_scn = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_pribuf);
	s_pribuf = NULL;
	zoomgfx_free(&s_gfx);
	tsnd_exit();
	free(s_list_hp);
	s_list_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
}

static void othunder_frame(void)
{
	rbx_audio.frame_mhz = RATE_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	update_gun();
	for (int line = 0; line < LINES; line++) {
		if (line == VBLANK_LINE) {
			vblank_capture();
			m68k_bus_set_line(5, 1);   // vblank_w: ASSERT_LINE until irq_ack_w
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, 12000000, RATE_MHZ, LINES));
		if (adc0808_advance(&s_adc, rbx_line_cycles(&s_adc_frac, 500000, RATE_MHZ, LINES))) {
			// P1X / P2X are PORT_REVERSE: the port value falls as the crosshair moves right on the (mirrored) screen
			int ch = s_adc.addr & 3;
			s_adc.sar = (uint8_t)((ch & 1) ? s_gun[ch] : 255 - s_gun[ch]);
			m68k_bus_set_line(6, 1);   // adc_eoc_w
		}
		tsnd_line();
	}
	tsnd_frame_end();
}

static int othunder_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr[2] = { s_cpu_frac, s_adc_frac };
	int misc[6] = { s_gun[0], s_gun[1], s_gun[2], s_gun[3], s_pcr_addr, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "irqline", &m68k_bus.irq_asserted, sizeof(m68k_bus.irq_asserted));   // level lines (CPU A)
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "pcr", s_pcr, sizeof(s_pcr));
	rc |= rbx_state_io(ctx, "scn", s_scn->ram, sizeof(s_scn->ram));
	rc |= rbx_state_io(ctx, "scnc", s_scn->ctrl, sizeof(s_scn->ctrl));
	rc |= rbx_state_io(ctx, "ioc", s_ioc, sizeof(s_ioc));
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "adc", &s_adc, sizeof(s_adc));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_cpu_frac = fr[0];
		s_adc_frac = fr[1];
		for (int i = 0; i < 4; i++)
			s_gun[i] = misc[i];
		s_pcr_addr = (uint16_t)misc[4];
	}
	return 0;
}

static int othunder_save(void *ctx) { return othunder_state(ctx, 0); }
static int othunder_load(void *ctx) { return othunder_state(ctx, 1); }
static int othunder_supported(const char *machine) { return !strcmp(machine, "othunder"); }

const rbx_driver rbx_drv_othunder = { "Operation Thunderbolt", othunder_init, othunder_exit, othunder_reset,
                                      othunder_frame, othunder_save, othunder_load, othunder_supported, othunder_render };
