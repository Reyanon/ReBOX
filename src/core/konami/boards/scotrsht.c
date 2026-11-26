/*  Konami Scooter Shooter. Ported from MAME src/mame/konami/scotrsht.cpp (BSD-3-Clause, copyright-holders: Pierpaolo
    Prazzoli). A 6809E at 3.072 MHz, a 64x32 tilemap with column scroll, the 16x16 sprites (their transparent pens by
    the palette bank), the colours through the PROMs; the sound: a Z80 at 3.072 MHz and a YM2203, a frame behind on
    the worker. The tilemap's flip screen not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3072000u     // (18.432 MHz / 6; the Z80 and the YM2203 too)
#define SND_RATE 48000
#define LINES 256              // (set_refresh_hz(60), set_size(256, 256), set_vblank_time(0))
#define CPU_LINE (CPU_CLOCK / 60 / LINES)   // (200 clocks a line)
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x1000           // (16 * 8 * 16 * 2 pens over 256 colours)

static uint8_t *s_rom, *s_zrom, *s_prom;
static uint32_t s_rom_size, s_zrom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[256];
static m6809_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static int s_port_dsw[3], s_port_p[2], s_port_system;

static struct {
	uint8_t ram[0x2000];     // 0000-1fff (0000 the colour RAM, 0800 the video RAM, 1000-10bf the sprites)
	uint8_t scroll[0x20];    // 2000-201f
	uint8_t charbank, palette_bank, irq_enable, flip;
	uint8_t zram[0x400], soundlatch;
	int32_t zline;           // (the Z80's IRQ: the latch's data pending)
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t ram[0x10c0], scroll[0x20], charbank, palette_bank, flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.ram, t.ram, sizeof(s_vs.ram));
	memcpy(s_vs.scroll, t.scroll, sizeof(s_vs.scroll));
	s_vs.charbank = t.charbank;
	s_vs.palette_bank = t.palette_bank;
	s_vs.flip = t.flip;
}

static inline uint8_t ctab(int q)   // (the pen's colour: pen ((i & 0x100) << 3) | (j << 8) | (i & 0xff))
{
	const int i = (q & 0xff) | (((q >> 11) & 1) << 8), j = (q >> 8) & 7;
	return (uint8_t)(((~i & 0x100) >> 1) | (j << 4) | (s_prom[0x300 + i] & 0x0f));
}

static void ss_render(void)   // (screen_update)
{
	for (int q = 0; q < NPENS; q++)
		rbx_video.palette[q] = s_rgb[ctab(q)];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {   // (the tilemap, set_scroll_cols(64): the first 32 set)
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int col = 0; col < 32; col++) {
			const int ty = (y + s_vs.scroll[col]) & 255;
			const int idx = (ty >> 3) * 64 + col;
			const int attr = s_vs.ram[idx];
			const int code = s_vs.ram[0x800 + idx] + (s_vs.charbank << 9) + ((attr & 0x40) << 2);
			const int pen = ((attr & 0x0f) + s_vs.palette_bank * 16) * 16;
			const int fx = (attr & 0x10) != 0, fy = (attr & 0x20) != 0;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64 + (fy ? 7 - (ty & 7) : (ty & 7)) * 8;
			for (int i = 0; i < 8; i++)
				d[col * 8 + i] = (uint16_t)(pen + src[fx ? 7 - i : i]);
		}
	}
	const uint8_t *sr = s_vs.ram + 0x1000;
	const int tcol = s_vs.palette_bank * 16;
	for (int i = 0; i < 0xc0; i += 4) {   // (draw_sprites: transmask of the pens whose colour is palette_bank * 16)
		const int attr = sr[i + 1];
		const int code = sr[i] + ((attr & 0x40) << 2);
		const int pen = 16 * 16 * 8 + ((attr & 0x0f) + s_vs.palette_bank * 16) * 16;
		int flipx = (attr & 0x10) != 0, flipy = (attr & 0x20) != 0;
		int sx = sr[i + 2] - ((attr & 0x80) << 1), sy = sr[i + 3];
		if (s_vs.flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		uint8_t trans[16];
		for (int p = 0; p < 16; p++)
			trans[p] = ctab(pen + p) == tcol;
		const uint8_t *src = s_spr + (rbx_wrap((uint32_t)code, s_nspr)) * 256;
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
				continue;
			const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
			uint16_t *d = bm + (yy - VIS_Y0) * pitch;
			for (int k = 0; k < 16; k++) {
				const int xx = sx + k;
				if (xx < 0 || xx >= VIS_W)
					continue;
				const int p = s[flipx ? 15 - k : k];
				if (!trans[p])
					d[xx] = (uint16_t)(pen + p);
			}
		}
	}
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out[2048];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static void q_put(int kind, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

// ---- the main CPU's bus (main_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x2000)
		return t.ram[a];
	if (a < 0x2020)
		return t.scroll[a - 0x2000];
	switch (a) {
	case 0x3100: return port(s_port_dsw[1]);
	case 0x3200: return port(s_port_dsw[2]);
	case 0x3300: return port(s_port_system);
	case 0x3301: return port(s_port_p[0]);
	case 0x3302: return port(s_port_p[1]);
	case 0x3303: return port(s_port_dsw[0]);
	}
	if (a >= 0x4000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000)
		t.ram[a] = v;
	else if (a < 0x2020)
		t.scroll[a - 0x2000] = v;
	else
		switch (a) {
		case 0x2043: t.charbank = v & 0x01; break;
		case 0x2044:   // (ctrl_w)
			t.irq_enable = (v >> 1) & 1;
			t.flip = (v >> 3) & 1;
			break;
		case 0x3000: t.palette_bank = (v & 0x70) >> 4; break;   // (palettebank_w)
		case 0x3100: q_put(Q_LATCH, v); break;
		}
}

// ---- the sound board (sound_map, sound_portmap) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x4400)
		return t.zram[a - 0x4000];
	if (a == 0x8000) {
		t.zline = 0;   // (the latch read: its data pending cleared)
		return t.soundlatch;
	}
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4400)
		t.zram[a - 0x4000] = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	return p < 2 ? rbx_ym_read(0, p & 1) : 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p < 2)
		rbx_ym_write(0, p & 1, v);
}

static void sound_run(int n)   // (a line's samples: the Z80 (its IRQ a level), the YM2203 0.40; mono)
{
	if (n <= 0)
		return;
	t.z_frac += (uint64_t)CPU_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	t.z_end += cyc;
	z80_run(&s_z, (unsigned long)t.z_end, (int *)&t.zline, 0);
	if (n > 64)
		n = 64;
	int32_t ym[96];
	memset(ym, 0, sizeof(ym));
	t.ym_frac += (uint64_t)CPU_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance(0, yc, ym, 96);
	for (int i = 0; i < n && s_out_pos < 2048; i++)
		s_out[s_out_pos++] = yn > 0 ? ym[i * yn / n] : 0;
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *e = &s_q[b][i];
		switch (e->kind) {
		case Q_ADV: sound_run((int)e->v); break;
		case Q_LATCH:
			t.soundlatch = (uint8_t)e->v;
			t.zline = 1;
			break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put_stereo(s_out[i], s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void ss_exit(void);
static void ss_reset(void);

static int ss_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t ts, sps, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &sps);
	if (!s_rom || !s_zrom || !s_prom || ps < 0x500 || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (gfx_8x8x4_packed_msb; spritelayout: 16x16, the right half 32 bytes on, the lower 64; the first pixel in the top
	// nibble)
	s_nchr = ts / 32;
	s_nspr = sps / 128;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		ss_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int i = 0; i < 32; i++) {
			s_chr[e * 64 + (uint32_t)i * 2] = tiles[e * 32 + (uint32_t)i] >> 4;
			s_chr[e * 64 + (uint32_t)i * 2 + 1] = tiles[e * 32 + (uint32_t)i] & 15;
		}
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint8_t b = sprites[e * 128 + (uint32_t)((x >> 3) * 32 + (y >> 3) * 64 + (y & 7) * 4 + (x & 7) / 2)];
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = (x & 1) ? b & 15 : b >> 4;
			}
	for (int i = 0; i < 256; i++) {   // (pal4bit)
		const int r = (s_prom[i] & 15) * 0x11, gg = (s_prom[i | 0x100] & 15) * 0x11, b = (s_prom[i | 0x200] & 15) * 0x11;
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = CPU_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 102;   // (0.40)
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2203", g->set);
		ss_exit();
		return -1;
	}
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		ss_exit();
		return -1;
	}
	ss_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void ss_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_ym_exit(0);
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void ss_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	rbx_ym_reset(0);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x40; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 4; p++)
		s_zmap[0x40 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void ss_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.irq_enable)
				m6809_hold_irq(&s_cpu);
		}
		t.cpu_end += CPU_LINE;
		m6809_run_until(&s_cpu, t.cpu_end);
		t.snd_frac += SND_RATE;
		const int ns = (int)(t.snd_frac / (60u * LINES));
		t.snd_frac -= (uint64_t)ns * (60u * LINES);
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int ss_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "ss_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "ss", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "ss_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "ss_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "ss_z80", &z, sizeof(z));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int ss_save(void *ctx) { return ss_state(ctx, 0); }
static int ss_load(void *ctx) { return ss_state(ctx, 1); }
static int ss_supported(const char *machine) { return !strcmp(machine, "scotrsht"); }

const rbx_driver rbx_drv_scotrsht = { "Konami Scooter Shooter", ss_init, ss_exit, ss_reset, ss_frame, ss_save, ss_load,
                                      ss_supported, ss_render, NULL };
