/*  Konami Quick Pick 5 / Wai Wai Jockey. Ported from MAME src/mame/konami/quickpick5.cpp (BSD-3-Clause, copyright-holders:
    R. Belmont) and the devices it uses (k053244_k053245.cpp: David Haywood; k053252.cpp: David Haywood; ticket.cpp:
    Aaron Giles). A Z80 at 8 MHz, a 64x32 text tilemap in front of the K053245's sprites through konamigx_v.c's pre-GX
    mode, the K053252 CCU's two interrupts, a K051649 and an OKIM6295 on this thread.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k051649.h"
#include "../../sound/okim6295.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PIX_CLOCK 8000000u     // (32 MHz / 4)
#define HTOTAL 512
#define LINES 256              // (61.04 Hz)
#define CPU_LINE 512           // (the Z80 at the pixel clock)
#define HOPPER_LINES 1563      // (100 ms)
#define SND_RATE 48000

static int VIS_W, VIS_H, s_qp5, s_vpos;
static uint8_t *s_rom, *s_sprrom, *s_ttl;
static uint32_t s_rom_size, s_sprrom_size, s_ttl_size;
static uint8_t *s_chr;   // (a byte a pixel)
static z80 s_z;
static const uint8_t *s_map[256];
static k051649_t s_scc;
static okim6295_t s_oki;
static int s_port_in[3], s_port_dsw[2], s_port_sio[3];
static uint8_t s_ram[0x1c00];   // (c000-dbff: a part of it the battery RAM)
static char s_nv_path[512];

static struct {
	uint8_t vram[0x1000], pal8[0x800];
	uint8_t k244_regs[16], ccu[16];
	uint16_t k245_ram[0x400], k245_buffer[0x400];
	uint8_t control, bank;
	uint16_t scrollx;
	uint32_t ttlrom_offset;
	int32_t irq_line;
	uint8_t nmi_line, motor, hopper;
	int32_t ccu_int_time, ccu_int_time_count, hopper_count;
	uint16_t sio_out, sio_in0, sio_in1;
	uint8_t sio_prev, vbl;
	uint32_t oki_frac;
	int32_t oki_out;
	uint64_t z_end, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct {
	uint8_t vram[0x1000], k244_regs[16];
	uint16_t buffer[0x400], scrollx;
} s_vs;

static void capture(void)
{
	for (int i = 0; i < 1024; i++)   // (the palette: xBGR_555 of two bytes, the low one first)
		((uint16_t *)gx.palram)[i] = (uint16_t)(t.pal8[2 * i] | t.pal8[2 * i + 1] << 8);
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.k244_regs, t.k244_regs, sizeof(s_vs.k244_regs));
	memcpy(s_vs.buffer, t.k245_buffer, sizeof(s_vs.buffer));
	s_vs.scrollx = t.scrollx;
}

static void sprite_cb(int *code, int *color, int *primask)   // (sprite_callback)
{
	(void)code;
	*primask = (~*color >> 4) & 2;
}

static void draw(void)   // (screen_update)
{
	gx_pre_fill_pen(0);
	uint16_t *f = gx_pre_frame16();
	uint8_t *p = gx_pre_pri();
	const int sx = s_qp5 ? 80 : s_vs.scrollx + 104, sy = s_qp5 ? 28 : 16;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = f + y * VIS_W;
		uint8_t *q = p + y * VIS_W;
		const int ty = (y + sy) & 255;
		for (int x = 0; x < VIS_W; x++) {
			const int tx = (x + sx) & 511;
			const int idx = (ty >> 3) * 64 + (tx >> 3);
			const int attr = s_vs.vram[idx << 1 | 1];
			const int code = s_vs.vram[idx << 1] | (attr & 0x0f) << 8;
			const int pix = s_chr[(uint32_t)code * 64 + (ty & 7) * 8 + (tx & 7)];
			if (pix) {
				const int color = s_qp5 ? (attr >> 3) & 0x1e : attr >> 4;
				d[x] = (uint16_t)(color * 16 + pix);
				q[x] = 1;
			}
		}
	}
	gx_pre_sprites_053245(s_vs.buffer, s_vs.k244_regs, s_qp5 ? -28 : 0, s_qp5 ? 20 : 16, -1);
}

// ---- the K053252, the K053244 / K053245 ----
static uint8_t ccu_r(int o)
{
	if (o >= 0x0e) {   // (the VCT read back)
		const int v = s_vpos - (((t.ccu[8] << 8 | t.ccu[9]) & 0x1ff) + 1);
		return (uint8_t)(o == 0x0e ? (v >> 8) & 1 : v & 0xff);
	}
	return t.ccu[o];
}
static void ccu_w(int o, uint8_t v)
{
	t.ccu[o] = v;
	if (o == 0x0d)
		t.ccu_int_time = v;
	else if (o == 0x0e)
		t.irq_line = 0;   // (vbl_ack_w)
	else if (o == 0x0f)
		t.nmi_line = 0;   // (nmi_ack_w)
}
static uint8_t k244_r(int o)
{
	if (o == 6)
		memcpy(t.k245_buffer, t.k245_ram, sizeof(t.k245_buffer));
	else if (o == 7)
		for (int i = 0; i < 0x400; i += 8)
			t.k245_buffer[i] = 0;
	else if (o >= 0x0c) {
		const uint32_t addr = (uint32_t)(t.k244_regs[11] & 7) << 18 | (uint32_t)t.k244_regs[8] << 10 |
		                      (uint32_t)t.k244_regs[9] << 2 | (uint32_t)((o & 3) ^ 1);
		return s_sprrom[addr & (s_sprrom_size - 1)];
	}
	return 0;
}
static void k244_w(int o, uint8_t v)
{
	t.k244_regs[o] = v;
	if (o == 6)
		memcpy(t.k245_buffer, t.k245_ram, sizeof(t.k245_buffer));
	else if (o == 7)
		for (int i = 0; i < 0x400; i += 8)
			t.k245_buffer[i] = 0;
}

// ---- the serial I/O of quickp5 ----
static void serial_io_w(uint8_t v)
{
	if ((v & 2) && !(t.sio_prev & 2)) {
		switch (v & 0x1c) {
		case 0x10:
			t.sio_out = 0;
			for (int i = 0; i < 3; i++)
				if ((t.sio_in0 >> i) & 1)
					t.sio_out |= (uint16_t)rbx_port_read(s_port_sio[2 - i]);
			t.sio_in1 = (uint16_t)(t.sio_in1 << 1 | (v & 1));
			break;
		case 0x18: t.sio_out >>= 1; break;
		case 0x1c: t.sio_in0 = (uint16_t)(t.sio_in0 << 1 | (v & 1)); break;
		}
	}
	t.sio_prev = v;
}

static void control_w(uint8_t v)
{
	t.bank = v & 1;
	if ((t.control & 0x60) != 0x60 && (v & 0x60) == 0x60)
		t.ttlrom_offset = 0;
	t.control = v;
}

// ---- the Z80's bus (common_map) ----
static uint8_t in_port(int i)
{
	if (i == 0) {   // (IN1: the hopper's line)
		rbx_port_custom(s_port_in[0], s_qp5 ? 0x04 : 0x01, t.hopper ? (s_qp5 ? 0x04 : 0x01) : 0);
	} else if (i == 2) {   // (IN3: the serial line, the vblank)
		const uint32_t m = s_qp5 ? 0x81 : 0x80;
		rbx_port_custom(s_port_in[2], m, (t.vbl ? 0x80u : 0u) | (s_qp5 ? (t.sio_out & 1u) : 0u));
	}
	return (uint8_t)rbx_port_read(s_port_in[i]);
}

static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return s_rom[a];
	if (a < 0xc000)
		return s_rom[0x8000 + t.bank * 0x4000u + (a & 0x3fff)];
	if (a < 0xdc00)
		return s_ram[a - 0xc000];
	if (a < 0xdc10)
		return ccu_r(a & 0x0f);
	if (a >= 0xdc40 && a < 0xdc50)
		return k244_r((a & 0x0f) ^ 1);
	switch (a) {
	case 0xdc80: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0xdc81: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	case 0xdcc0: return in_port(0);
	case 0xdcc1: return in_port(1);
	case 0xdd40: return in_port(2);
	case 0xde40: return oki_read(&s_oki);
	}
	if (a >= 0xe000 && a < 0xf000) {
		if (t.control & 0x10)
			return k051649_scc_r(&s_scc, a & 0xff, 0);
		if ((t.control & 0x60) == 0x60) {   // (ttlrom_r)
			const uint8_t v = s_ttl[t.ttlrom_offset];
			t.ttlrom_offset = rbx_wrap((t.ttlrom_offset + 1), s_ttl_size);
			return v;
		}
		return t.vram[a - 0xe000];
	}
	if (a >= 0xf000 && a < 0xf800)
		return t.pal8[a - 0xf000];
	if (a >= 0xf800) {
		const int o = (a - 0xf800) ^ 1;
		const uint16_t w = t.k245_ram[o >> 1];
		return (uint8_t)((o & 1) ? w : w >> 8);
	}
	return 0;
}

static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000)
		return;
	if (a < 0xdc00)
		s_ram[a - 0xc000] = v;
	else if (a < 0xdc10)
		ccu_w(a & 0x0f, v);
	else if (a >= 0xdc40 && a < 0xdc50)
		k244_w((a & 0x0f) ^ 1, v);
	else if (a == 0xdd80)
		control_w(v);
	else if (a == 0xddc0) {
		if (s_qp5)
			serial_io_w(v);
	} else if (a == 0xde00)   // (out_w: the hopper's motor)
		t.motor = s_qp5 ? (v >> 1) & 1 : v & 1;
	else if (a == 0xde40)
		oki_write(&s_oki, v);
	else if (!s_qp5 && a == 0xde80)
		t.scrollx = (uint16_t)((t.scrollx & 0xff00) | v);
	else if (!s_qp5 && a == 0xdec0)
		t.scrollx = (uint16_t)((t.scrollx & 0x00ff) | v << 8);
	else if (a >= 0xe000 && a < 0xf000) {
		if (t.control & 0x10)
			k051649_scc_w(&s_scc, a & 0xff, v);
		else if ((t.control & 0x60) != 0x60)
			t.vram[a - 0xe000] = v;
	} else if (a >= 0xf000 && a < 0xf800)
		t.pal8[a - 0xf000] = v;
	else if (a >= 0xf800) {
		const int o = (a - 0xf800) ^ 1;
		uint16_t *w = &t.k245_ram[o >> 1];
		*w = (o & 1) ? (uint16_t)((*w & 0xff00) | v) : (uint16_t)((*w & 0x00ff) | v << 8);
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- init / frame ----
static void qp_exit(void);
static void qp_reset(void);

static uint8_t *nv_ptr(void) { return s_ram + (s_qp5 ? 0x1000 : 0); }

static int qp_init(const rbx_game *g)
{
	uint32_t os;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_sprrom = rbx_region("k053245", &s_sprrom_size);
	s_ttl = rbx_region("ttl", &s_ttl_size);
	const uint8_t *pcm = rbx_region("oki", &os);
	if (!s_rom || s_rom_size < 0x10000 || !s_sprrom || !s_ttl || !pcm) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_qp5 = !strcmp(g->machine, "quickpick5");
	VIS_W = s_qp5 ? 384 : 400;
	VIS_H = s_qp5 ? 232 : 226;
	memset(&gx, 0, sizeof(gx));
	memset(&t, 0, sizeof(t));
	memset(s_ram, 0, sizeof(s_ram));
	gx.pal_entries = 1024;
	gx.pal555 = 1;
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_sprcb = sprite_cb;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = HTOTAL;
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_053245, 0, 0);
	gx_video_set_area(0, VIS_W);
	gx_video_set_y0(0);
	gx_video_set_height(VIS_H);
	s_chr = (uint8_t *)malloc(4096 * 64);   // (charlayout: 4096 of 8x8, 4 bits a pixel, the high nibble first)
	if (!s_chr) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	for (uint32_t e = 0; e < 4096; e++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint8_t b = s_ttl[(e * 32 + (uint32_t)(y * 4 + (x >> 1))) % s_ttl_size];
				s_chr[e * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)((x & 1) ? (b & 0x0f) : (b >> 4));
			}
	k051649_init(&s_scc, 2000000);
	k051649_set_rate(&s_scc, SND_RATE);
	oki_init(&s_oki, 1333333, 0, pcm, os, 128);   // (PIN7_LOW, 0.5)
	static const char *const in[3] = { "IN1", "IN2", "IN3" }, *const sio[3] = { "SIO1", "SIO2", "SIO3" };
	for (int i = 0; i < 3; i++) {
		s_port_in[i] = rbx_port_find(in[i]);
		s_port_sio[i] = s_qp5 ? rbx_port_find(sio[i]) : -1;
	}
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		qp_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (61.04 Hz)
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	qp_reset();
	s_nv_path[0] = 0;
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(nv_ptr(), 1, 0x400, f) != 0x400)
					memset(nv_ptr(), 0, 0x400);
				fclose(f);
			}
		}
	}
	return 0;
}

static void qp_exit(void)
{
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(nv_ptr(), 1, 0x400, f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	gx_video_exit();
	free(s_chr);
	s_chr = NULL;
}

static void qp_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	k051649_reset(&s_scc);
	oki_reset(&s_oki);
	z80_init(&s_z);
	s_z.read_byte = rd;
	s_z.write_byte = wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	for (int p = 0; p < 0x80; p++)
		s_map[p] = s_rom + p * 256;
	for (int p = 0; p < 0x1c; p++)
		s_map[0xc0 + p] = s_ram + p * 256;
	s_z.read_map = s_map;
}

static void qp_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_H + line) % LINES;   // (the frame from the vblank start)
		s_vpos = vpos;
		if (line == 0) {   // (vblank: the update)
			gx_video_capture();
			t.vbl = 1;
		}
		if (vpos == 0)
			t.vbl = 0;
		if (vpos == 255)   // (scanline: the K053252's IRQ1)
			t.irq_line = 1;
		if (--t.ccu_int_time_count < 0) {   // (the K053252's INT_TIME on the IRQ2 pin)
			if (!t.nmi_line) {
				t.nmi_line = 1;
				z80_gen_nmi(&s_z);
			}
			t.ccu_int_time_count = t.ccu_int_time;
		}
		if (t.motor) {   // (the hopper: a medal each 100 ms)
			if (++t.hopper_count >= HOPPER_LINES) {
				t.hopper_count = 0;
				t.hopper ^= 1;
			}
		} else {
			t.hopper_count = 0;
			t.hopper = 0;
		}
		t.z_end += CPU_LINE;
		z80_run(&s_z, (unsigned long)t.z_end, &t.irq_line, 0);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		for (int i = 0; i < ns; i++) {
			t.oki_frac += s_oki.rate;
			while (t.oki_frac >= SND_RATE) {
				t.oki_frac -= SND_RATE;
				t.oki_out = oki_sample(&s_oki);
			}
			int32_t scc = 0;
			k051649_render(&s_scc, &scc, 1);
			rbx_audio_put(t.oki_out + scc / 2);
		}
	}
	t.frame++;
	gx.frame = t.frame;
}

static int qp_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "qp", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "qp_ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "qp_oki", &s_oki, sizeof(s_oki));
	rc |= k051649_state(&s_scc, ctx, load, "qp_scc");
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "qp_z80", &z, sizeof(z));
	if (!rc && load) {
		z.read_byte = rd;
		z.write_byte = wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_map;
		z.op_rom = NULL;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int qp_save(void *ctx) { return qp_state(ctx, 0); }
static int qp_load(void *ctx) { return qp_state(ctx, 1); }
static int qp_supported(const char *machine) { return !strcmp(machine, "quickpick5") || !strcmp(machine, "waijockey"); }

const rbx_driver rbx_drv_quickpick5 = { "Konami Quick Pick 5", qp_init, qp_exit, qp_reset, qp_frame, qp_save, qp_load,
                                        qp_supported, gx_render, NULL };
