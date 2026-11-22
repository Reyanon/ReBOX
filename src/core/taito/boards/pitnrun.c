/*  Pit & Run hardware, ported from MAME src/mame/taito/pitnrun.cpp (BSD-3-Clause, copyright-holders: Tomasz Slanina,
    Pierpaolo Prazzoli).
    Z80 main (3.072 MHz), M68705P5 MCU (cpu/m6805; the Taito SJ style interface with access to the Z80 memory), Z80
    sound (2.5 MHz) with 2x AY-3-8910 (1.536 MHz). Video: 32x32 text layer, 128x32 scrolling background, 16x16 3bpp
    sprites and the spotlight mode (clip square + mask ROM) as in MAME's screen_update.
    Sound: the analog sound effects triggered through the noise latch ($a800-$a807) are not emulated in MAME either
    (silence here); the two AY-3-8910 are.
    The TILT input (MAME: master reset hack) and the watchdog are not emulated.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/m6805/m68705.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

// MAME set_raw(18.432 MHz / 3, 384, 0, 256, 264, 16, 240): the main CPU runs at half the pixel clock
#define LINES 264
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define MAIN_LINE 192
#define AUDIO_CLOCK 2500000
#define MCU_CLOCK 768000      // 3.072 MHz / 4
#define RATE_MHZ 60606        // 6.144 MHz / (384 x 264)
#define AY_CLOCK 1536000
#define SLICES 2

typedef struct {
	z80 z;
	int irq;
} zcpu;

static uint8_t *s_main_rom, *s_audio_rom, *s_spot, *s_prom;
static uint8_t s_ram[0x800], s_vram0[0x800], s_vram1[0x1000], s_spr[0x100], s_audio_ram[0x400];
static zcpu s_main, s_audio;
static m68705_t s_mcu;
static uint8_t s_mainlatch, s_nmi, s_nmi_line, s_h_heed, s_v_heed, s_ha, s_scrolly, s_soundlatch;
static uint16_t s_scroll;
static uint8_t s_fromz80, s_toz80, s_zaccept, s_zready, s_porta_in, s_porta_out;
static uint16_t s_address;
static uint64_t s_frac_audio, s_frac_mcu;
// video
static uint8_t *s_fg_tiles, *s_bg_tiles, *s_sprites, *s_spr_opaque;
static uint8_t (*s_spotmask_hp)[4][128 * 128];
#define s_spotmask (*s_spotmask_hp)
static uint32_t s_palette[96];
typedef struct {
	uint8_t vram0[0x800], vram1[0x1000], spr[0x100];
	uint8_t mainlatch, h_heed, v_heed, ha, scrolly;
	uint16_t scroll;
} snap_t;
static snap_t s_snap;
// ports
static int s_p_system, s_p_inputs, s_p_cocktail, s_p_dsw;
// sound
static ay8910_t s_ay[2];
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done;
static unsigned long s_audio_frame_start;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void snd_sync(void)
{
	uint32_t pos = (uint32_t)((uint64_t)(s_audio.z.cyc - s_audio_frame_start) * (AY_CLOCK / 1000) / (AUDIO_CLOCK / 1000));
	if (pos <= s_snd_done)
		return;
	int n = ay8910_render(&s_ay[0], pos - s_snd_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	ay8910_render(&s_ay[1], pos - s_snd_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	s_mix_n += n;
	s_snd_done = pos;
}
static uint8_t ay_latch_r(void *u) { (void)u; return s_soundlatch; }

// ---- main CPU ----
static void mainlatch_w(int bit, int state)
{
	s_mainlatch = (uint8_t)((s_mainlatch & ~(1 << bit)) | (state << bit));
	if (bit == 0) {   // NMION
		s_nmi = (uint8_t)state;
		if (!state)
			s_nmi_line = 0;
	}
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_main_rom[a];
	if (a < 0x8800) return s_ram[a - 0x8000];
	if (a < 0x9000) return s_vram0[a - 0x8800];
	if (a < 0xa000) return s_vram1[a - 0x9000];
	if (a < 0xa100) return s_spr[a - 0xa000];
	if (a == 0xa800) return port8(s_p_system);
	if (a == 0xb000) return port8(s_p_dsw);
	if (a == 0xb800) return (uint8_t)~port8((s_mainlatch & 0x40) ? s_p_cocktail : s_p_inputs);   // LS157 mux, inverted
	if (a == 0xd000) { s_zaccept = 1; return s_toz80; }
	if (a == 0xd800) return (uint8_t)~((s_zready << 1) | s_zaccept);
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a < 0x8800) { s_ram[a - 0x8000] = v; return; }
	if (a < 0x9000) { s_vram0[a - 0x8800] = v; return; }
	if (a < 0xa000) { s_vram1[a - 0x9000] = v; return; }
	if (a < 0xa100) { s_spr[a - 0xa000] = v; return; }
	if (a >= 0xa800 && a <= 0xa807) return;   // analog sound triggers (not emulated)
	if (a >= 0xb000 && a <= 0xb007) { mainlatch_w(a & 7, v & 1); return; }
	switch (a) {
	case 0xb800: s_soundlatch = v; return;
	case 0xc800: s_scroll = (uint16_t)((s_scroll & 0xff00) | v); return;
	case 0xc801: s_scroll = (uint16_t)((s_scroll & 0x00ff) | v << 8); return;
	case 0xc802: s_scrolly = v; return;
	case 0xc804:   // to the MCU
		s_zready = 1;
		m68705_set_irq_line(&s_mcu, 1);
		s_fromz80 = v;
		return;
	case 0xc805: s_h_heed = v; return;
	case 0xc806: s_v_heed = v; return;
	case 0xc807: s_ha = v; return;
	}
}

// ---- MCU ----
static uint8_t mcu_port_r(void *u, int n, uint8_t mask)
{
	(void)u; (void)mask;
	if (n == 0) return s_porta_in;
	if (n == 2) return (uint8_t)(s_zready | s_zaccept << 1);
	return 0xff;
}

static void mcu_port_w(void *u, int n, uint8_t data, uint8_t mask)
{
	(void)u; (void)mask;
	if (n == 0) {
		s_porta_out = data;
		return;
	}
	if (n != 1)
		return;
	if (~data & 0x02) {   // the 68705 reads the command of the Z80
		s_zready = 0;
		m68705_set_irq_line(&s_mcu, 0);
		s_porta_in = s_fromz80;
	}
	if (~data & 0x04) {   // the 68705 writes data for the Z80
		s_toz80 = s_porta_out;
		s_zaccept = 0;
	}
	if (~data & 0x10)
		main_write(NULL, s_address, s_porta_out);
	if (~data & 0x20)
		s_porta_in = main_read(NULL, s_address);
	if (~data & 0x40)
		s_address = (uint16_t)((s_address & 0xff00) | s_porta_out);
	if (~data & 0x80)
		s_address = (uint16_t)((s_address & 0x00ff) | s_porta_out << 8);
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000) return s_audio_rom[a];
	if (a >= 0x3800 && a < 0x3c00) return s_audio_ram[a - 0x3800];
	return 0;
}
static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x3800 && a < 0x3c00) s_audio_ram[a - 0x3800] = v;
}
static uint8_t audio_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p == 0x8f) { snd_sync(); return ay8910_data_r(&s_ay[0]); }
	return 0xff;
}
static void audio_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p == 0x00) s_soundlatch = 0;   // clear_w
	else if (p == 0x8c || p == 0x8d) { snd_sync(); ay8910_write(&s_ay[1], p & 1, v); }
	else if (p == 0x8e || p == 0x8f) { snd_sync(); ay8910_write(&s_ay[0], p & 1, v); }
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                      uint8_t (*in)(z80 *, uint8_t), void (*out)(z80 *, uint8_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = in;
	c->z.port_out = out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
	c->irq = 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE: acknowledged
	}
}

// ---- video ----
// MAME gfx_layout decode: plane / x / y bit offsets (MSB first), planes listed most significant first
static void decode_gfx(const uint8_t *src, uint32_t size, uint8_t *dst, uint8_t *opaque, int n, int w, int h, int planes,
                       const uint32_t *po, const uint32_t *xo, const uint32_t *yo, uint32_t inc)
{
	for (int t = 0; t < n; t++) {
		int any = 0;
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++) {
				int v = 0;
				for (int p = 0; p < planes; p++) {
					uint32_t b = (uint32_t)t * inc + po[p] + xo[x] + yo[y];
					v = v << 1 | ((b >> 3) < size && (src[b >> 3] & (0x80 >> (b & 7))) ? 1 : 0);
				}
				dst[(t * h + y) * w + x] = (uint8_t)v;
				any |= v;
			}
		if (opaque)
			opaque[t] = any != 0;
	}
}

static void pr_render(void)
{
	const snap_t *s = &s_snap;
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));   // (palette size 128: the core masks indices)
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int flipx = (s->mainlatch >> 6) & 1, flipy = (s->mainlatch >> 7) & 1;
	const int color_sel = (s->mainlatch >> 1) & 1, char_bank = (s->mainlatch >> 5) & 1;
	for (int y = 0; y < rbx_video.height; y++)
		memset(bm + y * pitch, 0, SCREEN_W * sizeof(uint16_t));
	// clip (screen coordinates)
	int cx0 = 0, cx1 = SCREEN_W - 1, cy0 = VIS_Y0, cy1 = VIS_Y1, dx = 0, dy = 0;
	if (s->ha & 4) {
		dx = 128 - s->h_heed + ((s->ha & 8) << 5) + 3;
		dy = 128 - s->v_heed + ((s->ha & 0x10) << 4);
		if (flipx)
			dx = 128 - dx + 16;
		if (flipy)
			dy = 128 - dy;
		if (dx > cx0) cx0 = dx;
		if (dx + 127 < cx1) cx1 = dx + 127;
		if (dy > cy0) cy0 = dy;
		if (dy + 127 < cy1) cy1 = dy + 127;
	}
	// background: 128 x 32 tiles, opaque
	const uint16_t bg_base = (uint16_t)(32 + color_sel * 16);
	for (int sy = cy0; sy <= cy1; sy++) {
		int v = (flipy ? 255 - sy + s->scrolly : sy + s->scrolly) & 255;
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		for (int sx = cx0; sx <= cx1; sx++) {
			int u = (flipx ? 255 - sx + s->scroll : sx + s->scroll) & 1023;
			int code = s->vram1[(v >> 3) * 128 + (u >> 3)] + (char_bank << 8);
			d[sx] = (uint16_t)(bg_base + s_bg_tiles[code * 64 + (v & 7) * 8 + (u & 7)]);
		}
	}
	// sprites
	for (int offs = 0; offs < 0x100; offs += 4) {
		const uint8_t *sp = s->spr + offs;
		int pal = sp[2] & 3;
		int sy = 256 - sp[0] - 16, sx = sp[3] + 1;
		int fy = (sp[1] & 0x80) >> 7, fx = (sp[1] & 0x40) >> 6;
		if (flipx) {
			sx = 242 - sx;
			fx = !fx;
		}
		if (flipy) {
			sy = 240 - sy;
			fy = !fy;
		}
		int code = (sp[1] & 0x3f) + ((sp[2] & 0x80) >> 1) + ((sp[2] & 0x40) << 1);
		if (!s_spr_opaque[code])
			continue;
		const uint8_t *src = s_sprites + code * 256;
		for (int y = 0; y < 16; y++) {
			int yy = sy + y;
			if (yy < cy0 || yy > cy1)
				continue;
			const uint8_t *row = src + (fy ? 15 - y : y) * 16;
			uint16_t *d = bm + (yy - VIS_Y0) * pitch;
			for (int x = 0; x < 16; x++) {
				int xx = sx + x;
				if (xx < cx0 || xx > cx1)
					continue;
				int c = row[fx ? 15 - x : x];
				if (c)
					d[xx] = (uint16_t)(pal * 8 + c);
			}
		}
	}
	// spotlight mask (pen 0 where the mask bit is 0)
	if (s->ha & 4) {
		const uint8_t *m = s_spotmask[s->ha & 3];
		for (int sy = cy0; sy <= cy1; sy++) {
			int my = sy - dy;
			if (flipy) my = 127 - my;
			uint16_t *d = bm + (sy - VIS_Y0) * pitch;
			for (int sx = cx0; sx <= cx1; sx++) {
				int mx = sx - dx;
				if (flipx) mx = 127 - mx;
				if (!m[my * 128 + mx])
					d[sx] = 0;
			}
		}
	}
	// text layer, transparent pen 0
	for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
		int v = flipy ? 255 - sy : sy;
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		for (int sx = 0; sx < SCREEN_W; sx++) {
			int u = flipx ? 255 - sx : sx;
			int c = s_fg_tiles[s->vram0[(v >> 3) * 32 + (u >> 3)] * 64 + (v & 7) * 8 + (u & 7)];
			if (c)
				d[sx] = (uint16_t)(64 + c);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.vram0, s_vram0, sizeof(s_vram0));
	memcpy(s_snap.vram1, s_vram1, sizeof(s_vram1));
	memcpy(s_snap.spr, s_spr, sizeof(s_spr));
	s_snap.mainlatch = s_mainlatch;
	s_snap.h_heed = s_h_heed;
	s_snap.v_heed = s_v_heed;
	s_snap.ha = s_ha;
	s_snap.scrolly = s_scrolly;
	s_snap.scroll = s_scroll;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void pr_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_vram0, 0, sizeof(s_vram0));
	memset(s_vram1, 0, sizeof(s_vram1));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write, no_in, no_out);
	zcpu_init(&s_audio, audio_read, audio_write, audio_in, audio_out);
	m68705_reset(&s_mcu);
	s_mainlatch = s_nmi = s_nmi_line = 0;
	s_h_heed = s_v_heed = s_ha = s_scrolly = s_soundlatch = 0;
	s_scroll = 0;
	s_fromz80 = s_toz80 = s_porta_in = s_porta_out = 0;
	s_zaccept = 1;
	s_zready = 0;
	s_address = 0;
	m68705_set_irq_line(&s_mcu, 0);
	s_frac_audio = s_frac_mcu = 0;
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	s_mix_n = 0;
	s_snd_done = 0;
}

static void pr_exit(void)
{
	free(s_fg_tiles);
	free(s_bg_tiles);
	free(s_sprites);
	free(s_spr_opaque);
	free(s_mix);
	s_fg_tiles = s_bg_tiles = s_sprites = s_spr_opaque = NULL;
	s_mix = NULL;
	free(s_spotmask_hp);
	s_spotmask_hp = NULL;
}

static int pr_init(const rbx_game *g)
{
	if (!s_spotmask_hp && !(s_spotmask_hp = calloc(1, sizeof(*s_spotmask_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t msize, asize, usize, ssize, bsize, fsize, spsize, psize;
	s_main_rom = rbx_region("maincpu", &msize);
	s_audio_rom = rbx_region("audiocpu", &asize);
	uint8_t *mcu = rbx_region("mcu", &usize);
	uint8_t *spr = rbx_region("sprites", &ssize);
	uint8_t *bg = rbx_region("bgtiles", &bsize);
	uint8_t *fg = rbx_region("fgtiles", &fsize);
	s_spot = rbx_region("spot", &spsize);
	s_prom = rbx_region("proms", &psize);
	if (!s_main_rom || msize < 0x8000 || !s_audio_rom || asize < 0x2000 || !mcu || !spr || ssize < 0x6000 || !bg ||
	    bsize < 0x4000 || !fg || fsize < 0x2000 || !s_spot || spsize < 0x2000 || !s_prom || psize < 0x60) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (m68705_init(&s_mcu, M68705_P5, mcu, usize)) {
		rbx_fail("%s: MCU ROM missing", g->set);
		return -1;
	}
	s_mcu.port_r[0] = mcu_port_r;
	s_mcu.port_r[1] = mcu_port_r;
	s_mcu.port_r[2] = mcu_port_r;
	s_mcu.port_w[0] = mcu_port_w;
	s_mcu.port_w[1] = mcu_port_w;
	s_fg_tiles = (uint8_t *)malloc(256 * 64);
	s_bg_tiles = (uint8_t *)malloc(512 * 64);
	s_sprites = (uint8_t *)malloc(256 * 256);
	s_spr_opaque = (uint8_t *)malloc(256);
	if (!s_fg_tiles || !s_bg_tiles || !s_sprites || !s_spr_opaque) {
		pr_exit();
		return -1;
	}
	// charlayout: planes { 1/2, 1/2 + 4, 0, 4 }, x { 0-3, 8-11 }, y step 16, 16 bytes a tile
	static const uint32_t cx[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
	static const uint32_t cy[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
	uint32_t cpf[4] = { fsize * 4, fsize * 4 + 4, 0, 4 }, cpb[4] = { bsize * 4, bsize * 4 + 4, 0, 4 };
	decode_gfx(fg, fsize, s_fg_tiles, NULL, 256, 8, 8, 4, cpf, cx, cy, 128);
	decode_gfx(bg, bsize, s_bg_tiles, NULL, 512, 8, 8, 4, cpb, cx, cy, 128);
	// spritelayout: 16x16, planes { 0, 1/3, 2/3 }, 32 bytes a sprite
	static const uint32_t sx[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 64, 65, 66, 67, 68, 69, 70, 71 };
	static const uint32_t sy[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 128, 136, 144, 152, 160, 168, 176, 184 };
	uint32_t sp[3] = { 0, ssize / 3 * 8, ssize / 3 * 16 };
	decode_gfx(spr, ssize, s_sprites, s_spr_opaque, 256, 16, 16, 3, sp, sx, sy, 256);
	// spotlight masks: 4 x 128 x 128 bits, MSB on the left
	for (int i = 0; i < 4; i++)
		for (int y = 0; y < 128; y++)
			for (int x = 0; x < 128; x++)
				s_spotmask[i][y * 128 + x] = (s_spot[128 * 16 * i + (x >> 3) + y * 16] >> (7 - (x & 7))) & 1;
	// palette: 3 PROMs of 32 bytes, then the darkened copy of the background colors (lightning) at 48-63
	for (int i = 0; i < 96; i++) {
		int c = s_prom[i];
		int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		int gr = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		int b = 0x47 * ((c >> 6) & 1) + 0x97 * ((c >> 7) & 1);
		s_palette[i] = (uint32_t)(r << 16 | gr << 8 | b);
	}
	for (int i = 48; i < 64; i++) {
		int c = s_prom[i - 16];
		int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		int gr = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		int b = 0x47 * ((c >> 6) & 1) + 0x97 * ((c >> 7) & 1);
		s_palette[i] = (uint32_t)((r / 3) << 16 | (gr / 3) << 8 | (b / 3));
	}
	s_p_system = rbx_port_find("SYSTEM");
	s_p_inputs = rbx_port_find("INPUTS");
	s_p_cocktail = rbx_port_find("COCKTAIL");
	s_p_dsw = rbx_port_find("DSW");
	// AY-3-8910 1.536 MHz, route 0.5: native clock / 8, 4 averaged a sample (48 kHz)
	for (int i = 0; i < 2; i++) {
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 128, 4, 1);
		s_ay[i].port_a_read = ay_latch_r;
		s_ay[i].port_b_read = ay_latch_r;
	}
	uint32_t rate = ay8910_rate(&s_ay[0]);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 128)) {
		pr_exit();
		return -1;
	}
	pr_reset();
	return 0;
}

static void pr_frame(void)
{
	rbx_audio.frame_mhz = RATE_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	s_audio_frame_start = s_audio.z.cyc;
	s_snd_done = 0;
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;   // the frame starts at vblank
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			if (s_nmi && !s_nmi_line) {   // NMI line asserted until NMION is cleared
				s_nmi_line = 1;
				s_main.z.nmi_pending = 1;
			}
			s_audio.irq = 1;
		}
		int ac = rbx_line_cycles(&s_frac_audio, AUDIO_CLOCK, RATE_MHZ, LINES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES);
		for (int s = 0; s < SLICES; s++) {
			int last = s == SLICES - 1;
			zcpu_run(&s_main, MAIN_LINE / SLICES);
			m68705_execute(&s_mcu, uc / SLICES + (last ? uc % SLICES : 0));
			zcpu_run(&s_audio, ac / SLICES + (last ? ac % SLICES : 0));
		}
	}
	snd_sync();
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int zcpu_state(void *ctx, const char *tag, zcpu *c, int load)
{
	zcpu t = *c;
	if (rbx_state_io(ctx, tag, &t, sizeof(t)))
		return -1;
	if (load) {
		t.z.read_byte = c->z.read_byte;
		t.z.write_byte = c->z.write_byte;
		t.z.port_in = c->z.port_in;
		t.z.port_out = c->z.port_out;
		t.z.userdata = c->z.userdata;
		*c = t;
	}
	return 0;
}

static int pr_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "vram0", s_vram0, sizeof(s_vram0));
	rc |= rbx_state_io(ctx, "vram1", s_vram1, sizeof(s_vram1));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "audram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "m68705", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay1", &s_ay[1], ay8910_state_size());
	uint8_t v[16] = { s_mainlatch, s_nmi, s_nmi_line, s_h_heed, s_v_heed, s_ha, s_scrolly, s_soundlatch,
		s_fromz80, s_toz80, s_zaccept, s_zready, s_porta_in, s_porta_out, 0, 0 };
	uint16_t w[2] = { s_scroll, s_address };
	rc |= rbx_state_io(ctx, "misc", v, sizeof(v));
	rc |= rbx_state_io(ctx, "misc16", w, sizeof(w));
	if (rc)
		return -1;
	if (load) {
		s_mainlatch = v[0]; s_nmi = v[1]; s_nmi_line = v[2]; s_h_heed = v[3]; s_v_heed = v[4]; s_ha = v[5];
		s_scrolly = v[6]; s_soundlatch = v[7]; s_fromz80 = v[8]; s_toz80 = v[9]; s_zaccept = v[10]; s_zready = v[11];
		s_porta_in = v[12]; s_porta_out = v[13];
		s_scroll = w[0]; s_address = w[1];
	}
	return 0;
}

static int pr_save(void *ctx) { return pr_state(ctx, 0); }
static int pr_load(void *ctx) { return pr_state(ctx, 1); }
// "pitnrun" (jumpkun, no MCU) is not in the game table (not a Taito set)
static int pr_supported(const char *machine) { return !strcmp(machine, "pitnrun_mcu"); }

const rbx_driver rbx_drv_pitnrun = { "Pit & Run", pr_init, pr_exit, pr_reset, pr_frame, pr_save, pr_load, pr_supported, pr_render };
