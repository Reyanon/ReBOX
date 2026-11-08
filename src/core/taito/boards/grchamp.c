/*  Taito Grand Champion, ported from MAME src/mame/taito/grchamp.cpp, grchamp_v.cpp, grchamp_a.cpp, grchamp.h
    (license BSD-3-Clause, copyright-holders: Ernesto Corvi, Phil Stroffolino, Hans Andersson) with
    compute_resistor_weights from MAME src/emu/video/resnet.cpp (BSD-3-Clause, Couriersud).
    Three Z80s: main (6.144 MHz / 2: objects, radar, player car, sound latch), game board (6.144 MHz / 2: the left,
    right and center scrolling tilemaps) and sound (3 MHz: three AY-3-8910 at 1.5 MHz, IRQ held at
    3 MHz / 163840 = 36.6 Hz, NMI = latch pending AND enable). 384 x 264 raster at 16 kHz lines (256 x 224 shown,
    lines 16-239); the main and game board IRQs are level lines raised at vblank while enabled and cleared by
    their OUT bit; watchdog of 8 vblanks.
    Video: screen_update of MAME done one scanline at a time on the emulation side, as the beam reaches it
    (MAME forces a partial update whenever the PC3259 collision latch is read): the background (center over
    left / right), the object line buffer (16 sprites + text through the object PROM), headlamp, /KILL, player
    car, rain, radar and the three collision checks. The finished frame (palette indices) is the snapshot; the
    headlamp brightening (+64 on each channel) uses a second bank of 64 pens. Fog is not drawn (MAME neither).
    The LED score board is not drawn: the default DIP ("Display Score" = On Screen) shows the score on screen.
    Controls: the wheel (IPT_DIAL, reversed) turns with rbx_analog(DIAL) or the d-pad left / right; the gas pedal
    (IPT_PEDAL, 0-255) is held with BUTTON1 (ramps up, released when let go) or rbx_analog(PEDAL); the gear
    shifter is the JOYSTICK_UP toggle of MAME.
    Sound: the three AY8910 (route 0.2 each) and the 8-bit DAC pair on AY #1's ports (DISCRETE_DAC_R1 ladders,
    port B inverted, output gain 5000 per volt) are emulated, as MAME's discrete netlist has only these two DACs;
    the engine / attack / fog analog sounds are not emulated (MAME: "missing engine noise"), nor the AY #3
    filter capacitors. One output sample per scanline (16 kHz).
    grchampa is flagged not working in MAME (different ports, bad dump?) and behaves the same here.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

#define PIXEL_CLOCK (18432000 / 3)
#define CPU_CLOCK (PIXEL_CLOCK / 2)
#define SOUND_CPU_CLOCK 3000000
#define AY_CLOCK 1500000
#define HTOTAL 384
#define VTOTAL 264
#define VBEND 16
#define VBSTART 240
#define VIS_W 256
#define VIS_H (VBSTART - VBEND)
#define RATE_MHZ 60606            // 6.144 MHz / (384 * 264)
#define LINE_RATE (PIXEL_CLOCK / HTOTAL)   // 16000
#define SND_IRQ_PERIOD 81920      // sound CPU clocks between IRQs (6 MHz / 4 / 16 / 16 / 10 / 16 = 36.6 Hz)
#define RGB_MAX 191

typedef struct {
	z80 z;
	int irq;      // level (main / sub) or held until acknowledged (sound)
} zcpu;

static zcpu s_main, s_sub, s_snd;
static uint8_t *s_main_rom, *s_sub_rom, *s_snd_rom;
static uint32_t s_main_size, s_sub_size, s_snd_size;
static const uint8_t *s_ame, *s_head, *s_pl, *s_prom;
static uint8_t s_ram[0x400], s_radar[0x400], s_vram[0x400], s_spr[0x100];
static uint8_t s_left[0x800], s_right[0x800], s_center[0x800], s_subram[0x400], s_sndram[0x400];
static uint8_t s_cpu0_out[16], s_cpu1_out[16], s_comm_latch, s_comm_latch2[4];
static uint8_t s_latch, s_latch_flag, s_nmi_en, s_nmi_out;
static uint16_t s_collide;
static int s_wd;
static uint8_t s_accel, s_wheel;
static uint64_t s_frac_main, s_frac_sub, s_frac_snd, s_frac_ay;
static uint32_t s_snd_irq_cnt;
static uint8_t s_dac_a, s_dac_b;
static int s_port_dswa, s_port_dswb, s_port_tilt;
// decoded graphics (1 byte per pixel)
static uint8_t *s_chars;    // 512 x 8x8 2bpp (gfx1)
static uint8_t *s_sprites;  // 128 x 16x16 2bpp (gfx1)
static uint8_t *s_tiles[3]; // left / right / center: 256 x 8x8 4bpp (gfx2..gfx4)
// frame being drawn (emulation side) and snapshot
// sound
static ay8910_t s_ay[3];
static int32_t s_aybuf[64];
static double s_dac_lut[256];
static int32_t s_hp_prev_in, s_hp_prev_out;

// ---- graphics decode (MAME gfx_layout semantics: bit offsets counted from the MSB of each byte) ----
static inline int gbit(const uint8_t *rom, uint32_t off) { return (rom[off >> 3] >> (7 - (off & 7))) & 1; }

static void gfx_decode(const uint8_t *rom, int count, int w, int h, int planes, const uint32_t *po, const uint32_t *xo,
                       const uint32_t *yo, uint32_t inc, uint8_t *out)
{
	for (int c = 0; c < count; c++)
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++) {
				int v = 0;
				for (int p = 0; p < planes; p++)
					v = (v << 1) | gbit(rom, c * inc + po[p] + yo[y] + xo[x]);
				out[(c * h + y) * w + x] = (uint8_t)v;
			}
}

// ---- sound ----
static void update_nmi(void)
{
	int out = s_nmi_en && s_latch_flag;
	if (out && !s_nmi_out)
		s_snd.z.nmi_pending = 1;
	s_nmi_out = (uint8_t)out;
}

static void ay1_port_a(void *u, uint8_t d) { (void)u; s_dac_a = d; }
static void ay1_port_b(void *u, uint8_t d) { (void)u; s_dac_b = (uint8_t)(255 - d); }
static void ay3_port_a(void *u, uint8_t d) { (void)u; (void)d; }   // filter caps / DAC attenuation: not emulated
static void ay3_port_b(void *u, uint8_t d) { (void)u; s_nmi_en = (uint8_t)(~d & 1); update_nmi(); }

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000) return a < s_snd_size ? s_snd_rom[a] : 0xff;
	if (a >= 0x4000 && a < 0x4400) return s_sndram[a - 0x4000];
	if (a >= 0x4800 && a < 0x5000) {
		switch (a & 7) {
		case 1: return ay8910_data_r(&s_ay[0]);
		case 3: return ay8910_data_r(&s_ay[1]);
		case 5: case 7: return ay8910_data_r(&s_ay[2]);
		}
		return 0xff;
	}
	if (a >= 0x5000 && a < 0x5800) {
		switch (a & 3) {
		case 0:
			s_latch_flag = 0;
			update_nmi();
			return s_latch;
		case 1:
			return (uint8_t)((s_latch_flag ? 8 : 0) | 3);
		}
	}
	return 0xff;
}

static void snd_write(void *u, uint16_t a, uint8_t d)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4400) { s_sndram[a - 0x4000] = d; return; }
	if (a >= 0x4800 && a < 0x5000) {
		switch (a & 7) {
		case 0: case 1: ay8910_write(&s_ay[0], a & 1, d); break;
		case 2: case 3: ay8910_write(&s_ay[1], a & 1, d); break;
		default: ay8910_write(&s_ay[2], a & 1, d); break;
		}
		return;
	}
	if (a >= 0x5000 && a < 0x5800 && (a & 3) == 0)
		s_latch &= 0x7f;   // soundlatch_clear7_w
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return a < s_main_size ? s_main_rom[a] : 0xff;
	switch (a & 0xf800) {
	case 0x4000: return s_ram[a & 0x3ff];
	case 0x4800: return s_radar[a & 0x3ff];
	case 0x5000: return s_vram[a & 0x3ff];
	case 0x5800: return s_spr[a & 0xff];
	}
	return 0xff;
}

static void main_write(void *u, uint16_t a, uint8_t d)
{
	(void)u;
	switch (a & 0xf800) {
	case 0x4000: s_ram[a & 0x3ff] = d; break;
	case 0x4800: s_radar[a & 0x3ff] = d; break;
	case 0x5000: s_vram[a & 0x3ff] = d; break;
	case 0x5800: s_spr[a & 0xff] = d; break;
	}
}

static uint8_t main_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p & 0x80)
		return 0xff;
	switch (p & 7) {
	case 0: return s_accel;
	case 1: {
		uint8_t bits = (uint8_t)((s_collide >> (((p >> 3) & 3) * 4)) & 0x0f);
		return (uint8_t)(bits | (bits << 4));
	}
	case 2: return s_comm_latch;
	case 3: return s_wheel;
	case 4: return (uint8_t)rbx_port_read(s_port_dswa);
	case 5: return (uint8_t)rbx_port_read(s_port_dswb);
	case 6: return (uint8_t)rbx_port_read(s_port_tilt);
	}
	return 0xff;
}

static void cpu0_outputs_w(int offset, uint8_t data)
{
	uint8_t diff = data ^ s_cpu0_out[offset];
	s_cpu0_out[offset] = data;
	switch (offset) {
	case 0x00:
		if ((diff & 0x01) && !(data & 0x01))
			s_main.irq = 0;
		if ((diff & 0x02) && !(data & 0x02))
			s_collide = 0;
		break;
	case 0x0d:
		s_wd = 0;
		break;
	case 0x0e:   // sound latch
		s_latch = data;
		s_latch_flag = 1;
		update_nmi();
		break;
	}
}

static void main_out(z80 *z, uint8_t p, uint8_t d)
{
	(void)z;
	if ((p & 0xb0) == 0x00)
		cpu0_outputs_w(p & 0x0f, d);
	else if ((p & 0xbc) == 0x10)
		s_comm_latch2[p & 3] = d;
	// 0x20-0x3f: LED board (not drawn)
}

// ---- game board CPU ----
static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000 || (a >= 0x5000 && a < 0x7000)) return a < s_sub_size ? s_sub_rom[a] : 0xff;
	if (a < 0x2800) return s_left[a - 0x2000];
	if (a < 0x3000) return s_right[a - 0x2800];
	if (a < 0x3800) return s_center[a - 0x3000];
	if (a >= 0x4000 && a < 0x4800) return s_subram[a & 0x3ff];
	return 0xff;
}

static void sub_write(void *u, uint16_t a, uint8_t d)
{
	(void)u;
	if (a >= 0x2000 && a < 0x2800) s_left[a - 0x2000] = d;
	else if (a >= 0x2800 && a < 0x3000) s_right[a - 0x2800] = d;
	else if (a >= 0x3000 && a < 0x3800) s_center[a - 0x3000] = d;
	else if (a >= 0x4000 && a < 0x4800) s_subram[a & 0x3ff] = d;
}

static uint8_t sub_in(z80 *z, uint8_t p)
{
	(void)z;
	return p < 4 ? s_comm_latch2[p] : 0xff;
}

static void sub_out(z80 *z, uint8_t p, uint8_t d)
{
	(void)z;
	if (p & 0x80)
		return;
	int offset = p & 0x0f;
	uint8_t diff = d ^ s_cpu1_out[offset];
	s_cpu1_out[offset] = d;
	if (offset == 4 && (diff & 0x01) && !(d & 0x01))
		s_sub.irq = 0;
	else if (offset == 8)
		s_comm_latch = d;
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t d) { (void)z; (void)p; (void)d; }

// ---- video: one scanline ----
static void draw_objects(int y, uint8_t *objdata)
{
	const uint8_t *prom = s_prom + 0x20;
	int change = (s_cpu0_out[0] & 0x20) << 3;
	memset(objdata, 0, 256);
	for (int num = 0; num < 16; num++) {
		int dataoffs = ((~num & 0x0e) << 1) | ((~num & 0x01) << 5);
		int sy = s_spr[0x40 + (dataoffs & ~0x20)];
		int dy = sy + ~y;
		if ((dy & 0xf0) == 0) {
			int codeflip = s_spr[0x41 + dataoffs];
			int code = (codeflip & 0x3f) + (change >> 2);
			int yflip = (codeflip & 0x80) ? 0x0f : 0x00;
			int xflip = (codeflip & 0x40) ? 0x0f : 0x00;
			const uint8_t *src = s_sprites + (code & 0x7f) * 256 + ((dy ^ yflip) & 15) * 16;
			int color = (s_spr[0x42 + (dataoffs & ~0x20)] & 0x07) << 2;
			int sx = s_spr[0x43 + dataoffs];
			for (int x = 0; x < 16; x++) {
				int dx = ~(x + sx) & 0xff;
				if (dx >= 0x08 && dx < 0xf8) {
					int pix = src[x ^ xflip];
					if (pix)
						objdata[dx] = (uint8_t)(pix | color);
				}
			}
		}
	}
	for (int num = 0; num < 32; num++) {
		int hprime = num ^ 0x1f;
		int dataoffs = hprime << 1;
		int sy = s_spr[0x00 + dataoffs];
		int dy = sy + ~y;
		int color = (s_spr[0x01 + dataoffs] & 0x07) << 2;
		int code = s_vram[(hprime | ((dy & 0xf8) << 2)) & 0x3ff] + change;
		const uint8_t *src = s_chars + (code & 0x1ff) * 64 + (dy & 7) * 8;
		for (int x = 0; x < 8; x++) {
			int pix = src[x ^ 7];
			uint8_t *o = &objdata[num * 8 + x];
			if (pix)
				*o = (uint8_t)((prom[pix | color] ^ 0x0f) | 0x10);
			else
				*o = (uint8_t)(prom[*o & 0x1f] ^ 0x0f);
		}
	}
}

// ---- lines: the collision on the emulation side, the pixels on the render thread ----
// A line's pixels (MAME's screen update of one scanline) need the latches, the tile rows and the objects as they are
// at that line: those are kept per line (two sets: the frame drawn into one while the render thread draws the last).
// The collision latch is all the CPUs read back: it is worked out here, on the pixels where it can come from (an
// object pixel with bit 3, or my car's 32 pixel window), in the same order and with the same tests as the full line
typedef struct {
	uint8_t out0[16], out1[16];
	uint8_t center[64], lr[64];   // the tile codes of the line's tile rows, 64 columns
	uint8_t radar[32];            // the radar bits of the line
	uint8_t obj[VIS_W];
} gc_line;
static gc_line *s_lines, *s_snap_lines;   // (s_lines: the set being drawn, of two at s_line_mem)
static gc_line *s_line_mem;
static int s_line_set;

static inline int tile_row_pixel(const uint8_t *codes, const uint8_t *gfx, int col, int row)
{
	return gfx[codes[(col >> 3) & 63] * 64 + (row & 7) * 8 + (col & 7)];
}

static void row_codes(uint8_t *codes, const uint8_t *ram, int row)
{
	const int tr = row >> 3;
	for (int tc = 0; tc < 64; tc++)
		codes[tc] = ram[((tc & 31) + tr * 32 + (tc >> 5) * 1024) & 0x7ff];
}

// the line's state for its pixels, and the collision latch
static void draw_line(int y)
{
	gc_line *L = &s_lines[y - VBEND];
	memcpy(L->out0, s_cpu0_out, 16);
	memcpy(L->out1, s_cpu1_out, 16);
	const uint8_t *lrram = y < 128 ? s_left : s_right;
	const int lyscroll = s_cpu1_out[2], ryscroll = s_cpu1_out[7], cyscroll = s_cpu1_out[11];
	const int lrrow = ((y < 128 ? lyscroll : ryscroll) + y - 16) & 0xff;
	const int crow = (cyscroll + y - 16) & 0xff;
	row_codes(L->center, s_center, crow);
	row_codes(L->lr, lrram, lrrow);
	memcpy(L->radar, s_radar + ((~y & 0x3e) << 4), 32);
	draw_objects(y, L->obj);
	if (s_collide & 0x1000)
		return;
	const int out0 = s_cpu0_out[0], out2 = s_cpu0_out[2];
	const int mydv = (s_cpu0_out[3] - (y - 16)) & 0xff;
	const int pl_row = (mydv & 0xe0) == 0 && (out0 & 0x02);   // (my car's collision can come on this line)
	const int plbase = s_cpu0_out[4] & 0x0f;
	const int lrxscroll = s_cpu1_out[0] + (s_cpu1_out[1] & 1) * 256;
	const int cxscroll = s_cpu1_out[9] + (s_cpu1_out[10] & 1) * 256;
	const int cxmask = (s_cpu1_out[3] & 0x20) ? 0xff : 0x1ff;
	const uint8_t *lrgfx = y < 128 ? s_tiles[0] : s_tiles[1];
	for (int x = 0; x < VIS_W; x++) {
		const int obj8 = L->obj[x] & 0x08;
		const int mydh = (out2 - x) & 0xff;
		const int mycar = pl_row && (mydh & 0xe0) == 0 &&
		                  ((s_pl[((mydh >> 2) | (mydv << 3) | (plbase << 8)) & 0xfff] >> (~mydh & 0x03)) & 0x01);
		if (!obj8 && !mycar)
			continue;
		int mvid = tile_row_pixel(L->center, s_tiles[2], (cxscroll + x + (384 - 336)) & cxmask, crow);
		if ((mvid & 0x0f) == 0)
			mvid = tile_row_pixel(L->lr, lrgfx, (lrxscroll + x + (384 - 336)) & 0x1ff, lrrow);
		if (mycar) {
			if (obj8)
				s_collide = (uint16_t)(0x1000 | 0x2000 | ((~y & 0x80) << 3) | ((~y & 0xf8) << 2) | ((~x & 0xf8) >> 3));
			else if ((mvid & 0x0f) != 0)
				s_collide = (uint16_t)(0x1000 | 0x4000 | ((~y & 0x80) << 3) | ((~y & 0xf8) << 2) | ((~x & 0xf8) >> 3));
		}
		if (!(s_collide & 0x1000) && obj8 && (mvid & 0x0f) != 0)
			s_collide = (uint16_t)(0x1000 | 0x8000 | ((~y & 0x80) << 3) | ((~y & 0xf8) << 2) | ((~x & 0xf8) >> 3));
		if (s_collide & 0x1000)
			return;
	}
}

// one line's pixels from its state (render thread)
static void line_pixels(const gc_line *L, int y, uint16_t *dest)
{
	const uint8_t *o0 = L->out0, *o1 = L->out1;
	const int bgcolor = o1[3] & 0x10;
	const int amebase = o0[4] >> 4;
	const int plbase = o0[4] & 0x0f;
	const int lrxscroll = o1[0] + (o1[1] & 1) * 256;
	const int lyscroll = o1[2], ryscroll = o1[7];
	const int cxscroll = o1[9] + (o1[10] & 1) * 256;
	const int cyscroll = o1[11];
	const int cxmask = (o1[3] & 0x20) ? 0xff : 0x1ff;
	const uint8_t *lrgfx = y < 128 ? s_tiles[0] : s_tiles[1];
	const int lrrow = ((y < 128 ? lyscroll : ryscroll) + y - 16) & 0xff;
	const int crow = (cyscroll + y - 16) & 0xff;
	const int out0 = o0[0], out2 = o0[2], out7 = o0[7], out8 = o0[8];
	const int head_on = (out0 & 0x10) != 0;
	const int mydv = (o0[3] - (y - 16)) & 0xff;
	const int head_row = head_on && ((mydv ^ (mydv >> 1)) & 0x40) == 0;
	const int head_v = ((mydv & 0x3f) << 3) | ((~mydv & 0x40) << 3) | ((out0 & 0x10) << 6);
	const int pl_row = (mydv & 0xe0) == 0;
	const int radar = y >= 192 && (out0 & 0x80);
	const int effy = (out7 - y) & 0x0f;
	for (int x = 0; x < VIS_W; x++) {
		int headbit = 0, kill = 0;
		int mvid = tile_row_pixel(L->center, s_tiles[2], (cxscroll + x + (384 - 336)) & cxmask, crow);
		if ((mvid & 0x0f) == 0)
			mvid = tile_row_pixel(L->lr, lrgfx, (lrxscroll + x + (384 - 336)) & 0x1ff, lrrow);
		int objpix = L->obj[x];
		const int mydh = (out2 - x) & 0xff;
		if (head_row && (mydh & 0xc0) == 0xc0) {
			int bits = s_head[(((mydh & 0x38) >> 3) | head_v) & 0x7ff];
			headbit = (bits >> (~mydh & 0x07)) & 0x01;
		}
		if (head_on && !headbit && !(objpix & 0x10)) {
			kill = 1;
			objpix &= ~7;
		}
		if (pl_row && (mydh & 0xe0) == 0) {
			int bits = s_pl[((mydh >> 2) | (mydv << 3) | (plbase << 8)) & 0xfff] >> (~mydh & 0x03);
			if (bits & 0x01)
				objpix |= 4;   // MYCAR(A)
			if (bits & 0x10)
				objpix |= 3;   // MYCAR(B)
		}
		if (amebase != 0) {
			int effx = (out8 + x) & 0x0f;
			if ((s_ame[(amebase << 5) | (effy << 1) | (effx >> 3)] >> (effx & 0x07)) & 0x01)
				objpix |= 7;
		}
		if (radar && ((L->radar[(~x & 0xf8) >> 3] >> (x & 0x07)) & 0x01))
			objpix |= 7;
		int pen;
		if ((objpix & 7) != 0)
			pen = 32 + (objpix & 7);
		else if (!kill)
			pen = (mvid | bgcolor) & 0x1f;
		else
			pen = 32;
		dest[x] = (uint16_t)(headbit ? pen + 64 : pen);
	}
}

static void gc_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VBEND; y < VBSTART; y++)
		line_pixels(&s_snap_lines[y - VBEND], y, bm + (y - VBEND) * pitch);
}

// ---- palette ----
static void resistor_weights(const int *res, int count, double *w)
{
	for (int n = 0; n < count; n++) {
		double r0 = 1.0 / 1e12, r1 = 1.0 / 100;
		for (int j = 0; j < count; j++) {
			if (j == n) r1 += 1.0 / res[j];
			else r0 += 1.0 / res[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		double v = RGB_MAX * r0 / (r1 + r0);
		w[n] = v < 0 ? 0 : v > RGB_MAX ? RGB_MAX : v;
	}
}

static uint32_t add_sat(uint32_t c, int a)
{
	int r = ((c >> 16) & 0xff) + a, g = ((c >> 8) & 0xff) + a, b = (c & 0xff) + a;
	if (r > 255) r = 255;
	if (g > 255) g = 255;
	if (b > 255) b = 255;
	return (uint32_t)(r << 16 | g << 8 | b);
}

static void setup_palette(void)
{
	static const int res[3] = { 100, 270, 470 };
	double w3[3], w2[2];
	resistor_weights(res, 3, w3);
	resistor_weights(res, 2, w2);
	double mx = w3[0] + w3[1] + w3[2];
	if (w2[0] + w2[1] > mx) mx = w2[0] + w2[1];
	double scale = RGB_MAX / mx;
	for (int i = 0; i < 3; i++) w3[i] *= scale;
	for (int i = 0; i < 2; i++) w2[i] *= scale;
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < 128; i++)
		pal[i] = 0;
	for (int i = 0; i < 32; i++) {
		uint8_t c = s_prom[i];
		int r = (int)(w3[0] * (c & 1) + w3[1] * ((c >> 1) & 1) + w3[2] * ((c >> 2) & 1) + 0.5);
		int g = (int)(w3[0] * ((c >> 3) & 1) + w3[1] * ((c >> 4) & 1) + w3[2] * ((c >> 5) & 1) + 0.5);
		int b = (int)(w2[0] * ((c >> 6) & 1) + w2[1] * ((c >> 7) & 1) + 0.5);
		pal[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	for (int i = 0; i < 8; i++)
		pal[32 + i] = (uint32_t)(((i & 4) ? RGB_MAX << 16 : 0) | ((i & 2) ? RGB_MAX << 8 : 0) | ((i & 1) ? RGB_MAX : 0));
	for (int i = 0; i < 40; i++)
		pal[64 + i] = add_sat(pal[i], 64);
}

// ---- machine ----
static void zinit(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
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

static void machine_reset(void)
{
	zinit(&s_main, main_read, main_write, main_in, main_out);
	zinit(&s_sub, sub_read, sub_write, sub_in, sub_out);
	zinit(&s_snd, snd_read, snd_write, no_in, no_out);
	for (int i = 0; i < 3; i++)
		ay8910_reset(&s_ay[i]);
	s_nmi_en = 0;   // sound NMI disabled
	s_nmi_out = 0;
	s_wd = 0;
}

static void gc_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_radar, 0, sizeof(s_radar));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_left, 0, sizeof(s_left));
	memset(s_right, 0, sizeof(s_right));
	memset(s_center, 0, sizeof(s_center));
	memset(s_subram, 0, sizeof(s_subram));
	memset(s_sndram, 0, sizeof(s_sndram));
	memset(s_cpu0_out, 0, sizeof(s_cpu0_out));
	memset(s_cpu1_out, 0, sizeof(s_cpu1_out));
	memset(s_comm_latch2, 0, sizeof(s_comm_latch2));
	s_comm_latch = 0;
	s_latch = 0;
	s_latch_flag = 0;
	s_collide = 0;
	s_accel = 0;
	s_wheel = 0;
	s_dac_a = 0;
	s_dac_b = 255;
	s_frac_main = s_frac_sub = s_frac_snd = s_frac_ay = 0;
	s_snd_irq_cnt = 0;
	s_hp_prev_in = s_hp_prev_out = 0;
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_sub, 0, sizeof(s_sub));
	memset(&s_snd, 0, sizeof(s_snd));
	machine_reset();
}

static void gc_exit(void)
{
	free(s_chars); s_chars = NULL;
	free(s_sprites); s_sprites = NULL;
	for (int i = 0; i < 3; i++) { free(s_tiles[i]); s_tiles[i] = NULL; }
	free(s_line_mem); s_line_mem = s_lines = s_snap_lines = NULL;
}

static int gc_init(const rbx_game *g)
{
	uint32_t sz[8];
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_sub_rom = rbx_region("sub", &s_sub_size);
	s_snd_rom = rbx_region("audiocpu", &s_snd_size);
	const uint8_t *gfx1 = rbx_region("gfx1", &sz[0]);
	const uint8_t *gfx[3] = { rbx_region("gfx2", &sz[1]), rbx_region("gfx3", &sz[2]), rbx_region("gfx4", &sz[3]) };
	s_ame = rbx_region("gfx5", &sz[4]);
	s_head = rbx_region("gfx6", &sz[5]);
	s_pl = rbx_region("gfx7", &sz[6]);
	s_prom = rbx_region("proms", &sz[7]);
	if (!s_main_rom || !s_sub_rom || !s_snd_rom || !gfx1 || sz[0] < 0x2000 || !gfx[0] || !gfx[1] || !gfx[2] ||
	    sz[1] < 0x2000 || sz[2] < 0x2000 || sz[3] < 0x2000 || !s_ame || sz[4] < 0x800 || !s_head || sz[5] < 0x800 ||
	    !s_pl || sz[6] < 0x1000 || !s_prom || sz[7] < 0x40) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_port_dswa = rbx_port_find("DSWA");
	s_port_dswb = rbx_port_find("DSWB");
	s_port_tilt = rbx_port_find("TILT");
	s_chars = (uint8_t *)malloc(512 * 64);
	s_sprites = (uint8_t *)malloc(128 * 256);
	for (int i = 0; i < 3; i++)
		s_tiles[i] = (uint8_t *)malloc(256 * 64);
	s_line_mem = (gc_line *)calloc(2 * VIS_H, sizeof(gc_line));
	s_line_set = 0;
	s_lines = s_line_mem;
	s_snap_lines = s_line_mem + VIS_H;
	if (!s_chars || !s_sprites || !s_tiles[0] || !s_tiles[1] || !s_tiles[2] || !s_line_mem) {
		gc_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{
		// gfx_8x8x2_planar: planes { RGN_FRAC(1,2), 0 }
		static const uint32_t po[2] = { 0x1000 * 8, 0 };
		uint32_t xo[8], yo[8];
		for (int i = 0; i < 8; i++) { xo[i] = (uint32_t)i; yo[i] = (uint32_t)i * 8; }
		gfx_decode(gfx1, 512, 8, 8, 2, po, xo, yo, 64, s_chars);
	}
	{
		// sprite_layout: 16x16, planes { RGN_FRAC(1,2), 0 }, x STEP8(0,1) STEP8(64,1), y STEP8(0,8) STEP8(128,8)
		static const uint32_t po[2] = { 0x1000 * 8, 0 };
		uint32_t xo[16], yo[16];
		for (int i = 0; i < 8; i++) {
			xo[i] = (uint32_t)i; xo[8 + i] = (uint32_t)(64 + i);
			yo[i] = (uint32_t)i * 8; yo[8 + i] = (uint32_t)(128 + i * 8);
		}
		gfx_decode(gfx1, 128, 16, 16, 2, po, xo, yo, 256, s_sprites);
	}
	{
		// tile_layout: 4 bpp, planes { 0, 4, RGN_FRAC(1,2), RGN_FRAC(1,2) + 4 }, x STEP4(0,1) STEP4(64,1)
		static const uint32_t po[4] = { 0, 4, 0x1000 * 8, 0x1000 * 8 + 4 };
		uint32_t xo[8], yo[8];
		for (int i = 0; i < 4; i++) { xo[i] = (uint32_t)i; xo[4 + i] = (uint32_t)(64 + i); }
		for (int i = 0; i < 8; i++) yo[i] = (uint32_t)i * 8;
		for (int k = 0; k < 3; k++)
			gfx_decode(gfx[k], 256, 8, 8, 4, po, xo, yo, 128, s_tiles[k]);
	}
	if (rbx_video_setup(VIS_W, VIS_H, 128) || rbx_audio_setup(LINE_RATE)) {
		gc_exit();
		return -1;
	}
	setup_palette();
	for (int i = 0; i < 3; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 51, 4, 1);   // route 0.2
	s_ay[0].port_a_write = ay1_port_a;
	s_ay[0].port_b_write = ay1_port_b;
	s_ay[2].port_a_write = ay3_port_a;
	s_ay[2].port_b_write = ay3_port_b;
	// DISCRETE_DAC_R1 (8 resistors, no bias / ground): V = 3.4 * sum(bit / R) / sum(1 / R)
	{
		static const double r[8] = { 680e3, 330e3, 150e3, 82e3, 39e3, 20e3, 10e3, 4.7e3 };
		double gsum = 0;
		for (int b = 0; b < 8; b++) gsum += 1.0 / r[b];
		for (int v = 0; v < 256; v++) {
			double gon = 0;
			for (int b = 0; b < 8; b++) if (v & (1 << b)) gon += 1.0 / r[b];
			s_dac_lut[v] = 3.4 * gon / gsum * 5000.0;
		}
	}
	gc_reset();
	return 0;
}

static void zrun(zcpu *c, int cycles, int hold)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (hold && c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE: acknowledged
	}
}

static void update_controls(void)
{
	uint32_t in = rbx_input(1);
	int d = rbx_analog(1, RBX_IPT_DIAL);
	if (!d) {
		if (in & RBX_IN_LEFT) d = -5;
		if (in & RBX_IN_RIGHT) d = 5;
	}
	s_wheel = (uint8_t)(s_wheel - d);   // PORT_REVERSE
	int p = rbx_analog(1, RBX_IPT_PEDAL);
	int a = s_accel;
	if (p)
		a += p;
	else if (in & (RBX_IN_B1 | RBX_IN_B2))
		a += 16;
	else
		a -= 16;
	s_accel = (uint8_t)(a < 0 ? 0 : a > 255 ? 255 : a);
}

static void gc_frame(void)
{
	rbx_audio.frame_mhz = RATE_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	update_controls();
	for (int line = 0; line < VTOTAL; line++) {
		if (line >= VBEND && line < VBSTART)
			draw_line(line);
		if (line == VBSTART) {
			rbx_video_wait_idle();
			s_snap_lines = s_lines;   // (its lines all set: the render draws them, the next frame sets the other set)
			s_line_set ^= 1;
			s_lines = s_line_mem + s_line_set * VIS_H;
			rbx_video_frame_done();
			if (s_cpu0_out[0] & 0x01) s_main.irq = 1;
			if (s_cpu1_out[4] & 0x01) s_sub.irq = 1;
			if (++s_wd > 8) {
				rbx_log("grchamp: watchdog reset");
				machine_reset();
			}
		}
		int mc = rbx_line_cycles(&s_frac_main, CPU_CLOCK, RATE_MHZ, VTOTAL);
		s_frac_sub = s_frac_main;
		int sc = rbx_line_cycles(&s_frac_snd, SOUND_CPU_CLOCK, RATE_MHZ, VTOTAL);
		// two slices per line (MAME quantum 1/6000 s is coarser)
		for (int h = 0; h < 2; h++) {
			zrun(&s_main, h ? mc - mc / 2 : mc / 2, 0);
			zrun(&s_sub, h ? mc - mc / 2 : mc / 2, 0);
			int scs = h ? sc - sc / 2 : sc / 2;
			s_snd_irq_cnt += (uint32_t)scs;
			if (s_snd_irq_cnt >= SND_IRQ_PERIOD) {
				s_snd_irq_cnt -= SND_IRQ_PERIOD;
				s_snd.irq = 1;
			}
			zrun(&s_snd, scs, 1);
		}
		// one output sample per line: AY averages + the DAC pair, high-passed (the DAC carries a DC level)
		int ac = rbx_line_cycles(&s_frac_ay, AY_CLOCK, RATE_MHZ, VTOTAL);
		int32_t sum = 0;
		for (int i = 0; i < 3; i++) {
			memset(s_aybuf, 0, sizeof(s_aybuf));
			int n = ay8910_render(&s_ay[i], (uint32_t)ac, s_aybuf, 64);
			int32_t acc = 0;
			for (int k = 0; k < n; k++) acc += s_aybuf[k];
			if (n) sum += acc / n;
		}
		int32_t dac = (int32_t)(s_dac_lut[s_dac_a] + s_dac_lut[s_dac_b]);
		// one-pole DC blocker on the DAC (x / 256 fixed point, pole 0.995)
		int32_t hp = dac - s_hp_prev_in + (s_hp_prev_out * 255) / 256;
		s_hp_prev_in = dac;
		s_hp_prev_out = hp;
		rbx_audio_put(sum + hp);
	}
}

static int zstate(void *ctx, const char *tag, zcpu *c, int load)
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

static int gc_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zstate(ctx, "main", &s_main, load);
	rc |= zstate(ctx, "sub", &s_sub, load);
	rc |= zstate(ctx, "snd", &s_snd, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "radar", s_radar, sizeof(s_radar));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "left", s_left, sizeof(s_left));
	rc |= rbx_state_io(ctx, "right", s_right, sizeof(s_right));
	rc |= rbx_state_io(ctx, "center", s_center, sizeof(s_center));
	rc |= rbx_state_io(ctx, "subram", s_subram, sizeof(s_subram));
	rc |= rbx_state_io(ctx, "sndram", s_sndram, sizeof(s_sndram));
	rc |= rbx_state_io(ctx, "out0", s_cpu0_out, sizeof(s_cpu0_out));
	rc |= rbx_state_io(ctx, "out1", s_cpu1_out, sizeof(s_cpu1_out));
	rc |= rbx_state_io(ctx, "comm2", s_comm_latch2, sizeof(s_comm_latch2));
	for (int i = 0; i < 3; i++)
		rc |= rbx_state_io(ctx, i == 0 ? "ay0" : i == 1 ? "ay1" : "ay2", &s_ay[i], ay8910_state_size());
	int32_t m[16] = { s_comm_latch, s_latch, s_latch_flag, s_nmi_en, s_nmi_out, s_collide, s_wd, s_accel, s_wheel,
	                  (int32_t)s_snd_irq_cnt, s_dac_a, s_dac_b, s_hp_prev_in, s_hp_prev_out, 0, 0 };
	rc |= rbx_state_io(ctx, "misc", m, sizeof(m));
	uint64_t f[4] = { s_frac_main, s_frac_sub, s_frac_snd, s_frac_ay };
	rc |= rbx_state_io(ctx, "frac", f, sizeof(f));
	if (rc)
		return -1;
	if (load) {
		s_comm_latch = (uint8_t)m[0]; s_latch = (uint8_t)m[1]; s_latch_flag = (uint8_t)m[2]; s_nmi_en = (uint8_t)m[3];
		s_nmi_out = (uint8_t)m[4]; s_collide = (uint16_t)m[5]; s_wd = m[6]; s_accel = (uint8_t)m[7];
		s_wheel = (uint8_t)m[8]; s_snd_irq_cnt = (uint32_t)m[9]; s_dac_a = (uint8_t)m[10]; s_dac_b = (uint8_t)m[11];
		s_hp_prev_in = m[12]; s_hp_prev_out = m[13];
		s_frac_main = f[0]; s_frac_sub = f[1]; s_frac_snd = f[2]; s_frac_ay = f[3];
	}
	return 0;
}

static int gc_save(void *ctx) { return gc_state(ctx, 0); }
static int gc_load(void *ctx) { return gc_state(ctx, 1); }
static int gc_supported(const char *machine) { return !strcmp(machine, "grchamp"); }

const rbx_driver rbx_drv_grchamp = { "Grand Champion", gc_init, gc_exit, gc_reset, gc_frame, gc_save, gc_load,
                                     gc_supported, gc_render };
