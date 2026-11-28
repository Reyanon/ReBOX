/*  Super Qix hardware (Prebillian, Vs. Hot Smash, Super Qix and its bootlegs), ported from MAME
    src/mame/taito/superqix.cpp, superqix.h and superqix_v.cpp (BSD-3-Clause, copyright-holders: Mirko Buffoni,
    Nicola Salmoria, Tomasz Slanina). Memory / I/O maps, machine configurations, the MCU interfaces and the video
    follow MAME.
    Machines:
      pbillian    Z80 + MC68705P5 (cpu/m6805) through the port A/B/C multiplexer, AY-3-8910 + the sample player
                  (MAME SAMPLES device fed from the "samples" ROM: 8-bit unsigned PCM, a sample starts at
                  trigger * 128 and ends at the first 0xff byte, played at 12 MHz / 3072 with linear interpolation)
      pbillianb   pbillian without the MCU (inputs on the 0x0c00 ports)
      sqix        Z80 + i8751 (cpu/mcs51), two AY-3-8910 (single output), bitmap layer
      sqix_8031   sqix with the bootleg 8031 + external ROM and its port 1 multiplexer
      sqix_nomcu  sqix without the MCU (sqixb2; perestro / perestrof with MAME's init_perestro decryption)
    Interleave: 8 slices per scanline between the Z80 and the MCU (MAME synchronizes on every semaphore access and
    boosts the interleave for 250 us after a Z80 -> 68705 write).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/m6805/m68705.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

enum { M_PBILLIAN, M_PBILLIANB, M_SQIX, M_SQIX_8031, M_SQIX_NOMCU };

typedef struct {
	const char *name;
	int kind;
} machine_t;

static const machine_t s_machines[] = {
	{ "pbillian", M_PBILLIAN },
	{ "pbillianb", M_PBILLIANB },
	{ "sqix", M_SQIX },
	{ "sqix_8031", M_SQIX_8031 },
	{ "sqix_nomcu", M_SQIX_NOMCU },
};

// MAME screen: 60 Hz, 256 x 256, visible y 16-239 (no raw parameters: 256 lines of equal length)
#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define RATE_MHZ 60000
#define SLICES 8
#define Z80_CLOCK 6000000
#define AY_CLOCK 1500000
#define M68705_CLOCK 750000    // 12 MHz / 4 input, internal clock / 4
#define I8751_CLOCK 500000     // 6 MHz / 12
#define SQIX_VBL_LINES 38      // set_vblank_time(2500 us) = 38.4 lines

static const machine_t *s_m;
static int s_sqix, s_has_mcu, s_invert_p2_spinner;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x2000];          // e000-ffff (spriteram e000, videoram e800)
static uint8_t s_palram[0x200];
static uint8_t (*s_bitmap_hp)[2][0x7000];    // sqix bitmap RAM (I/O 0800-77ff, 8800-f7ff)
#define s_bitmap (*s_bitmap_hp)
static z80 s_z;
static int s_z80_reset;                // Z80 /RESET held by the 8751
static int s_nmi_line, s_nmi_mask;
static int s_bank, s_gfxbank, s_show_bitmap, s_flip;
// MCU interface (MAME superqix_state_base / superqix_state / hotsmash_state)
static int s_z80_has_written, s_mcu_has_written;
static uint8_t s_from_z80, s_from_mcu;
static uint8_t s_portb_out, s_portc_out, s_porta_in;
static uint8_t s_port2_raw, s_bl_port1, s_bl_fake_port2, s_bl_port3_out;
static int s_invert_coin_lockout;
static int s_dial_oldpos[2], s_dial_sign[2];
static int s_dial[2], s_plunger[2];
static m68705_t s_m68705;
static mcs51_t s_i8751;
static uint8_t *s_mcu_rom;
// sound
static ay8910_t s_ay[2];
static int s_nay;
static int16_t *s_smp_buf;
static uint32_t s_smp_size;
static uint32_t s_smp_start, s_smp_len, s_smp_pos, s_smp_step;   // pos in 16.16 source samples, len 0 = idle
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done, s_slice_start, s_slice_len;
static unsigned long s_cpu_start;
static int s_slice_cycles;
static uint64_t s_frac_z80, s_frac_ay, s_frac_mcu;
// video
static uint8_t *s_chars, *s_bgtiles, *s_sprites;   // 1 byte per pixel
static uint8_t *s_char_used, *s_bg_used, *s_spr_used;   // tile has a non-zero pen
static int s_nchars, s_nbg, s_nspr;
static uint8_t s_snap_ram[0x1000];     // e000-efff
static uint8_t s_snap_pal[0x200];
static uint8_t (*s_snap_bitmap_hp)[0x7000];
#define s_snap_bitmap (*s_snap_bitmap_hp)
static int s_snap_gfxbank, s_snap_flip;
static int s_line;
// ports
static int s_p_dsw1, s_p_dsw2, s_p_system, s_p_buttons, s_p_p1, s_p_p2, s_p_launch[2], s_p_controls, s_p_controls2;

static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0xff; }

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	int n = ay8910_render(&s_ay[0], pos - s_snd_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_nay > 1)
		ay8910_render(&s_ay[1], pos - s_snd_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_smp_len) {
		for (int i = 0; i < n && s_smp_len; i++) {
			uint32_t ip = s_smp_pos >> 16, fr = s_smp_pos & 0xffff;
			int32_t a = s_smp_buf[s_smp_start + ip], b = s_smp_buf[s_smp_start + rbx_wrap((ip + 1), s_smp_len)];
			int32_t v = (int32_t)((a * (int32_t)(0x10000 - fr) + b * (int32_t)fr) >> 16);
			s_mix[s_mix_n + i] += v / 2;   // MAME route 0.50
			s_smp_pos += s_smp_step;
			if ((s_smp_pos >> 16) >= s_smp_len)
				s_smp_len = 0;
		}
	}
	s_mix_n += n;
	s_snd_done = pos;
}

static void snd_sync(void)
{
	uint32_t done = (uint32_t)(s_z.cyc - s_cpu_start);
	if ((int)done > s_slice_cycles)
		done = (uint32_t)s_slice_cycles;
	snd_run_to(s_slice_start + (s_slice_cycles ? s_slice_len * done / (uint32_t)s_slice_cycles : 0));
}

static void sample_trigger_w(uint8_t data)
{
	snd_sync();
	uint32_t start = (uint32_t)data << 7, end = start;
	while (end < s_smp_size && (s_smp_buf[end] != (int16_t)(0x7f * 256)))   // raw 0xff = end marker
		end++;
	s_smp_start = start < s_smp_size ? start : 0;
	s_smp_len = start < s_smp_size ? end - start : 0;
	s_smp_pos = 0;
}

// ---- interrupts ----
static void nmi_set(int state)
{
	if (state && !s_nmi_line)
		z80_gen_nmi(&s_z);
	s_nmi_line = state;
}

static void z80_reset_regs(void);

static void z80_reset_line(int asserted)
{
	if (asserted && !s_z80_reset) {
		z80_reset_regs();
		s_nmi_line = 0;
	}
	s_z80_reset = asserted;
}

// ---- Super Qix MCU interface ----
static void mcu_port2_w(uint8_t data)
{
	uint8_t changed = s_port2_raw ^ data;
	s_port2_raw = data;
	// bit 0 = inverted clock of the 74LS174 @1J: the other bits are latched on its falling edge
	if ((changed & 1) && !(data & 1)) {
		s_flip = (data >> 4) & 1;             // (bits 1-3: coin counters and lockout)
		z80_reset_line(!((data >> 5) & 1));
		s_mcu_has_written = (data >> 6) & 1;
	}
	if ((changed & 0x80) && !(data & 0x80))
		s_z80_has_written = 0;
}

static uint8_t system_r(void)
{
	if (s_p_system >= 0 && s_sqix)
		rbx_port_custom(s_p_system, 0x80, s_z80_has_written ? 0x80 : 0);
	return port8(s_p_system);
}

static void bootleg_port1_w(uint8_t data)
{
	uint8_t changed = s_bl_port1 ^ data;
	s_bl_port1 = data;
	int bit = (data & 0x0e) >> 1;
	s_bl_fake_port2 = (uint8_t)((s_bl_fake_port2 & ~(1 << bit)) | ((data & 1) << bit));
	if ((changed & 0x80) && (data & 0x80))
		s_from_mcu = s_bl_port3_out;
	mcu_port2_w(s_bl_fake_port2);
}

static uint8_t bootleg_port3_r(void)
{
	if (!(s_bl_port1 & 0x10)) return port8(s_p_dsw1);
	if (!(s_bl_port1 & 0x20)) return system_r();
	if (!(s_bl_port1 & 0x40)) return s_from_z80;
	return 0xff;
}

static uint8_t i8751_in(void *u, int port)
{
	(void)u;
	if (s_m->kind == M_SQIX_8031)
		return port == 3 ? bootleg_port3_r() : 0xff;
	switch (port) {
	case 0: return system_r();
	case 1: return port8(s_p_dsw1);
	case 3: return s_from_z80;
	}
	return 0xff;
}

static void i8751_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (s_m->kind == M_SQIX_8031) {
		if (port == 1) bootleg_port1_w(data);
		else if (port == 3) s_bl_port3_out = data;
		return;
	}
	if (port == 2) mcu_port2_w(data);
	else if (port == 3) s_from_mcu = data;
}

// ---- Prebillian / Hot Smash MCU interface ----
static int read_inputs(int player)
{
	int newpos = s_dial[player] & 0xff;
	int launch = port8(s_p_launch[player]) & 1;
	if (newpos != s_dial_oldpos[player]) {
		s_dial_sign[player] = ((newpos - s_dial_oldpos[player]) & 0x80) >> 7;
		s_dial_oldpos[player] = newpos;
	}
	int pl = s_plunger[player];
	if (player == 0 || !s_invert_p2_spinner)
		return launch << 4 | (s_dial_oldpos[player] & 1) << 2 | s_dial_sign[player] << 3 | (pl & 2) | ((pl ^ (pl >> 1)) & 1);
	return launch << 4 | (s_dial_oldpos[player] & 1) << 3 | s_dial_sign[player] << 2 | (pl & 2) | ((pl ^ (pl >> 1)) & 1);
}

static uint8_t m68705_porta_r(void *u, int n, uint8_t mask) { (void)u; (void)n; (void)mask; return s_porta_in; }

static void m68705_port_w(void *u, int n, uint8_t data, uint8_t mask)
{
	(void)u;
	(void)mask;
	if (n == 1) {
		s_portb_out = data;
		return;
	}
	if (n != 2)
		return;
	uint8_t changed = s_portc_out ^ data;
	s_portc_out = data;
	if ((changed & 8) && !(data & 8)) {   // falling edge of the latch bit
		switch (data & 7) {
		case 0: s_porta_in = port8(s_p_dsw1); break;
		case 1: s_porta_in = port8(s_p_dsw2); break;
		case 3:
			m68705_set_irq_line(&s_m68705, 0);
			s_porta_in = s_from_z80;
			s_z80_has_written = 0;
			break;
		case 5:
			s_from_mcu = s_portb_out;
			s_mcu_has_written = 1;
			s_porta_in = 0xff;
			break;
		case 6:
		case 7: s_porta_in = (uint8_t)read_inputs(data & 1); break;
		default: s_porta_in = 0xff; break;
		}
	}
}

// ---- AY ports ----
static uint8_t ay1_a(void *u)
{
	(void)u;
	if (s_sqix)
		return port8(s_p_p1);
	if (s_has_mcu)
		rbx_port_custom(s_p_buttons, 0xc0, (s_z80_has_written ? 0 : 0x40) | (s_mcu_has_written ? 0 : 0x80));
	return port8(s_p_buttons);
}
static uint8_t ay1_b(void *u)
{
	(void)u;
	if (s_sqix) {
		rbx_port_custom(s_p_p2, 0xc0, (s_mcu_has_written ? 0x40 : 0) | (s_z80_has_written ? 0x80 : 0));
		return port8(s_p_p2);
	}
	return port8(s_p_system);
}
static uint8_t ay2_a(void *u) { (void)u; return port8(s_p_dsw2); }
static uint8_t ay2_b(void *u)
{
	(void)u;
	if (s_m->kind == M_SQIX_NOMCU) {   // bitswap<8>(DSW1, 0..7)
		uint8_t d = port8(s_p_dsw1), r = 0;
		for (int i = 0; i < 8; i++)
			r |= (uint8_t)(((d >> i) & 1) << (7 - i));
		return r;
	}
	return s_from_mcu;
}
static void ay2_b_w(void *u, uint8_t data)
{
	(void)u;
	if (s_m->kind != M_SQIX_NOMCU)
		s_from_z80 = data;
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_rom_size ? s_rom[a] : 0;
	if (a < 0xc000) {
		uint32_t o = 0x10000u + 0x4000u * (uint32_t)s_bank + (a - 0x8000u);
		return o < s_rom_size ? s_rom[o] : 0;
	}
	if (a >= 0xe000) return s_ram[a - 0xe000];
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000)
		s_ram[a - 0xe000] = v;
}

// full 16-bit I/O address: IN A,(n) / OUT (n),A put A on the high byte, the ED forms put B (OUTI and friends
// decrement B before the output; this Z80 core calls port_out first)
static uint16_t io_addr(z80 *z, uint8_t lo)
{
	uint8_t p2 = main_read(NULL, (uint16_t)(z->pc - 2)), p1 = main_read(NULL, (uint16_t)(z->pc - 1));
	if (p2 == 0xed) {
		if (p1 == 0xa3 || p1 == 0xab || p1 == 0xb3 || p1 == 0xbb)
			return (uint16_t)((uint8_t)(z->b - 1) << 8 | lo);
		return (uint16_t)(z->b << 8 | lo);
	}
	return (uint16_t)(z->a << 8 | lo);
}

static uint8_t nmi_ack_r(void)
{
	nmi_set(0);
	return s_has_mcu ? 0xff : system_r();
}

static uint8_t pb_in(uint16_t a)
{
	if (a < 0x0200) return s_palram[a];
	if (s_m->kind == M_PBILLIANB) {
		switch (a) {
		case 0x0c00: return port8(s_p_dsw1);
		case 0x0c01: return port8(s_p_dsw2);
		case 0x0c06: return port8(s_p_controls);
		case 0x0c07: return port8(s_p_controls2);
		}
	}
	switch (a) {
	case 0x0401: snd_sync(); return ay8910_data_r(&s_ay[0]);
	case 0x0408:
		if (!s_has_mcu)
			return 0;
		s_mcu_has_written = 0;
		return s_from_mcu;
	case 0x0418: return nmi_ack_r();
	}
	return 0;
}

static void pb_out(uint16_t a, uint8_t v)
{
	if (a < 0x0200) { s_palram[a] = v; return; }
	switch (a) {
	case 0x0402: snd_sync(); ay8910_data_w(&s_ay[0], v); return;      // MAME data_address_w: data at A0 = 0
	case 0x0403: snd_sync(); ay8910_address_w(&s_ay[0], v); return;
	case 0x0408:
		if (!s_has_mcu)
			return;
		s_from_z80 = v;
		s_z80_has_written = 1;
		m68705_set_irq_line(&s_m68705, 1);
		return;
	case 0x0410:
		s_bank = (v >> 3) & 1;
		s_nmi_mask = (v >> 4) & 1;
		if (!s_nmi_mask)
			nmi_set(0);
		s_flip = (v >> 5) & 1;
		return;
	case 0x041a: sample_trigger_w(v); return;
	}
}

static uint8_t sq_in(uint16_t a)
{
	if (a < 0x0100) return s_palram[a];
	if (a >= 0x0800 && a < 0x7800) return s_bitmap[0][a - 0x0800];
	if (a >= 0x8800 && a < 0xf800) return s_bitmap[1][a - 0x8800];
	switch (a) {
	case 0x0401: snd_sync(); return ay8910_data_r(&s_ay[0]);
	case 0x0405: snd_sync(); return ay8910_data_r(&s_ay[1]);
	case 0x0408:
		if (s_has_mcu)
			s_z80_has_written = 1;
		return 0;
	case 0x0418: return nmi_ack_r();
	}
	return 0;
}

static void sq_out(uint16_t a, uint8_t v)
{
	if (a < 0x0100) { s_palram[a] = v; return; }
	if (a >= 0x0800 && a < 0x7800) { s_bitmap[0][a - 0x0800] = v; return; }
	if (a >= 0x8800 && a < 0xf800) { s_bitmap[1][a - 0x8800] = v; return; }
	switch (a) {
	case 0x0402: snd_sync(); ay8910_data_w(&s_ay[0], v); return;
	case 0x0403: snd_sync(); ay8910_address_w(&s_ay[0], v); return;
	case 0x0406: snd_sync(); ay8910_data_w(&s_ay[1], v); return;
	case 0x0407: snd_sync(); ay8910_address_w(&s_ay[1], v); return;
	case 0x0410:
		s_gfxbank = v & 3;
		s_show_bitmap = (v >> 2) & 1;
		s_nmi_mask = (v >> 3) & 1;
		s_bank = (v >> 4) & 3;
		return;
	}
}

static uint8_t port_in(z80 *z, uint8_t p)
{
	uint16_t a = io_addr(z, p);
	return s_sqix ? sq_in(a) : pb_in(a);
}

static void port_out(z80 *z, uint8_t p, uint8_t v)
{
	uint16_t a = io_addr(z, p);
	if (s_sqix)
		sq_out(a, v);
	else
		pb_out(a, v);
}

static void z80_reset_regs(void)
{
	unsigned long cyc = s_z.cyc;
	z80_init(&s_z);
	s_z.cyc = cyc;
	s_z.read_byte = main_read;
	s_z.write_byte = main_write;
	s_z.port_in = port_in;
	s_z.port_out = port_out;
	s_z.int_data = 0xff;
}

// ---- video ----
static uint32_t pal_rgb(uint8_t raw)
{
	int i = raw & 3, r = (raw & 0x0c) | i, g = ((raw >> 2) & 0x0c) | i, b = ((raw >> 4) & 0x0c) | i;
	return (uint32_t)((r * 17) << 16 | (g * 17) << 8 | (b * 17));
}

// one 8x8 tile of the 32x32 background; mode 0 = opaque, 1 = pen 0 only, 2 = pens 1-15 only
static void draw_tile8(const uint8_t *src, int col, int row, uint16_t base, int flip, int mode)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	int sx = flip ? (31 - col) * 8 : col * 8, sy = flip ? (31 - row) * 8 : row * 8;
	for (int y = 0; y < 8; y++) {
		int dy = sy + y - VIS_Y0;
		if (dy < 0 || dy >= VIS_Y1 - VIS_Y0 + 1)
			continue;
		const uint8_t *r = src + (flip ? 7 - y : y) * 8;
		uint16_t *d = bm + dy * pitch + sx;
		for (int x = 0; x < 8; x++) {
			uint8_t p = r[flip ? 7 - x : x];
			if (mode == 0 || (mode == 1 ? p == 0 : p != 0))
				d[x] = (uint16_t)(base + p);
		}
	}
}

static void draw_sprite16(const uint8_t *src, int sx, int sy, uint16_t base, int fx, int fy)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < 16; y++) {
		int dy = sy + y - VIS_Y0;
		if (dy < 0 || dy > VIS_Y1 - VIS_Y0)
			continue;
		const uint8_t *r = src + (fy ? 15 - y : y) * 16;
		uint16_t *d = bm + dy * pitch;
		for (int x = 0; x < 16; x++) {
			int dx = sx + x;
			uint8_t p = r[fx ? 15 - x : x];
			if (p && dx >= 0 && dx < SCREEN_W)
				d[dx] = (uint16_t)(base + p);
		}
	}
}

static void sq_render(void)
{
	const int npal = s_sqix ? 256 : 512;
	for (int i = 0; i < npal; i++)
		rbx_video.palette[i] = pal_rgb(s_snap_pal[i]);
	const uint8_t *vram = s_snap_ram + 0x800, *spr = s_snap_ram;
	const int flip = s_snap_flip;
	if (!s_sqix) {
		// MAME screen_update_pbillian: opaque tilemap (colors 256-511), then the sprites
		for (int row = 2; row < 30; row++)
			for (int col = 0; col < 32; col++) {
				int i = row * 32 + col, attr = vram[i + 0x400];
				int code = rbx_wrap((vram[i] + 256 * (attr & 7)), s_nchars);
				draw_tile8(s_chars + code * 64, col, row, (uint16_t)(256 + ((attr & 0xf0) >> 4) * 16), flip, 0);
			}
		for (int offs = 0; offs < 0x100; offs += 4) {
			int attr = spr[offs + 3];
			int code = rbx_wrap((((spr[offs] & 0xfc) >> 2) + 64 * (attr & 0x0f)), s_nspr);
			int sx = spr[offs + 1] + 256 * (spr[offs] & 1), sy = spr[offs + 2];
			if (flip) {
				sx = 240 - sx;
				sy = 240 - sy;
			}
			if (s_spr_used[code])
				draw_sprite16(s_sprites + code * 256, sx, sy, (uint16_t)(((attr & 0xf0) >> 4) * 16), flip, flip);
		}
		return;
	}
	// MAME screen_update_superqix: tilemap back half, bitmap, sprites, tilemap front half
	for (int pass = 0; pass < 2; pass++) {
		for (int row = 2; row < 30; row++)
			for (int col = 0; col < 32; col++) {
				int i = row * 32 + col, attr = vram[i + 0x400];
				int code = vram[i] + 256 * (attr & 3), group = (attr >> 3) & 1;
				const uint8_t *src;
				const uint8_t *used;
				if (attr & 4) {
					code %= s_nchars;
					src = s_chars + code * 64;
					used = s_char_used + code;
				} else {
					code = rbx_wrap((code + 1024 * s_snap_gfxbank), s_nbg);
					src = s_bgtiles + code * 64;
					used = s_bg_used + code;
				}
				uint16_t base = (uint16_t)(((attr & 0xf0) >> 4) * 16);
				if (pass == 0)
					draw_tile8(src, col, row, base, flip, group ? 1 : 0);
				else if (group && *used)
					draw_tile8(src, col, row, base, flip, 2);
			}
		if (pass == 1)
			break;
		// bitmap layer (pen 0 transparent), flipped with the screen
		uint16_t *bm = rbx_video.bitmap;
		for (int dy = 0; dy <= VIS_Y1 - VIS_Y0; dy++) {
			int y = dy + VIS_Y0, sy = flip ? 255 - y : y;
			if (sy < 16 || sy >= 16 + 224)
				continue;
			const uint8_t *src = s_snap_bitmap + (sy - 16) * 128;
			uint16_t *d = bm + dy * rbx_video.pitch;
			for (int x = 0; x < SCREEN_W; x++) {
				int sx = flip ? 255 - x : x;
				uint8_t b = src[sx >> 1];
				uint8_t p = (sx & 1) ? (b & 0x0f) : (b >> 4);
				if (p)
					d[x] = p;
			}
		}
		for (int offs = 0; offs < 0x100; offs += 4) {
			int attr = spr[offs + 3];
			int code = rbx_wrap((spr[offs] + 256 * (attr & 1)), s_nspr);
			int fx = attr & 4, fy = attr & 8;
			int sx = spr[offs + 1], sy = spr[offs + 2];
			if (flip) {
				sx = 240 - sx;
				sy = 240 - sy;
				fx = !fx;
				fy = !fy;
			}
			if (s_spr_used[code])
				draw_sprite16(s_sprites + code * 256, sx, sy, (uint16_t)(((attr & 0xf0) >> 4) * 16), fx, fy);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_ram, s_ram, sizeof(s_snap_ram));
	memcpy(s_snap_pal, s_palram, sizeof(s_snap_pal));
	if (s_sqix)
		memcpy(s_snap_bitmap, s_bitmap[s_show_bitmap], sizeof(s_snap_bitmap));
	s_snap_gfxbank = s_gfxbank;
	s_snap_flip = s_flip;
	rbx_video_frame_done();
}

// ---- graphics decoding ----
// packed 4 bpp, high nibble first: 8x8 tiles of 32 bytes (gfx_8x8x4_packed_msb / pbillian_charlayout)
// MAME init_perestro: the program's address lines A0-A3 shuffled, the graphics' by bit swaps
static void unshuffle16(uint8_t *p, uint32_t len, const uint8_t *tab)
{
	uint8_t t[16];
	for (uint32_t i = 0; i + 16 <= len; i += 16) {
		memcpy(t, p + i, 16);
		for (int j = 0; j < 16; j++)
			p[i + j] = t[tab[j]];
	}
}
static void perestro_decrypt(uint8_t *rom, uint32_t rlen, uint8_t *g1, uint32_t l1, uint8_t *g2, uint32_t l2,
                             uint8_t *g3, uint32_t l3)
{
	static const uint8_t conv[16] = { 0xc, 0x9, 0xb, 0xa, 0x8, 0xd, 0xf, 0xe, 0x4, 0x1, 0x3, 0x2, 0x0, 0x5, 0x7, 0x6 };
	uint8_t t1[16], t2[16], t3[16];
	for (int j = 0; j < 16; j++) {
		int b0 = j & 1, b1 = j >> 1 & 1, b2 = j >> 2 & 1, b3 = j >> 3 & 1;
		t1[j] = (uint8_t)(b3 << 3 | b2 << 2 | b0 << 1 | b1);   // bitswap<8>(j, 7,6,5,4, 3,2,0,1)
		t2[j] = (uint8_t)(b0 << 3 | b1 << 2 | b2 << 1 | b3);   // bitswap<8>(j, 7,6,5,4, 0,1,2,3)
		t3[j] = (uint8_t)(b1 << 3 | b0 << 2 | b3 << 1 | b2);   // bitswap<8>(j, 7,6,5,4, 1,0,3,2)
	}
	unshuffle16(rom, rlen, conv);
	unshuffle16(g1, l1, t1);
	unshuffle16(g2, l2, t2);
	unshuffle16(g3, l3, t3);
}

static uint8_t *decode8(const uint8_t *src, int n, uint8_t **used)
{
	uint8_t *d = (uint8_t *)malloc((size_t)n * 64);
	*used = (uint8_t *)calloc((size_t)n, 1);
	if (!d || !*used)
		return d;
	for (int t = 0; t < n; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				uint8_t b = src[t * 32 + y * 4 + x / 2];
				uint8_t p = (x & 1) ? (b & 0x0f) : (b >> 4);
				d[t * 64 + y * 8 + x] = p;
				if (p)
					(*used)[t] = 1;
			}
	return d;
}

// spritelayout: four 8x8 packed blocks (TL, TR at +32, BL at +64, BR at +96), 128 bytes a sprite
static uint8_t *decode16(const uint8_t *src, int n, uint8_t **used)
{
	uint8_t *d = (uint8_t *)malloc((size_t)n * 256);
	*used = (uint8_t *)calloc((size_t)n, 1);
	if (!d || !*used)
		return d;
	for (int t = 0; t < n; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				uint8_t b = src[t * 128 + (y >= 8 ? 64 : 0) + (x >= 8 ? 32 : 0) + (y & 7) * 4 + (x & 7) / 2];
				uint8_t p = (x & 1) ? (b & 0x0f) : (b >> 4);
				d[t * 256 + y * 16 + x] = p;
				if (p)
					(*used)[t] = 1;
			}
	return d;
}

// ---- init / frame ----
static const machine_t *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
}

static void sq_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_slice_len = s_cpu_start = 0;
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_bitmap, 0, sizeof(s_bitmap));
	memset(&s_z, 0, sizeof(s_z));
	z80_reset_regs();
	s_z80_reset = 0;
	s_nmi_line = s_nmi_mask = 0;
	s_bank = s_gfxbank = s_show_bitmap = s_flip = 0;
	s_z80_has_written = s_mcu_has_written = 0;
	s_from_z80 = s_from_mcu = 0;
	s_portb_out = s_portc_out = 0;
	s_porta_in = 0;
	s_port2_raw = s_bl_port1 = s_bl_fake_port2 = s_bl_port3_out = 0;
	s_dial_oldpos[0] = s_dial_oldpos[1] = s_dial_sign[0] = s_dial_sign[1] = 0;
	if (s_has_mcu) {
		if (s_sqix)
			mcs51_reset(&s_i8751);
		else
			m68705_reset(&s_m68705);
	}
	for (int i = 0; i < s_nay; i++)
		ay8910_reset(&s_ay[i]);
	s_smp_len = 0;
	s_snd_done = 0;
	s_mix_n = 0;
	s_frac_z80 = s_frac_ay = s_frac_mcu = 0;
	s_line = 0;
}

static void sq_exit(void)
{
	free(s_chars); s_chars = NULL;
	free(s_bgtiles); s_bgtiles = NULL;
	free(s_sprites); s_sprites = NULL;
	free(s_char_used); s_char_used = NULL;
	free(s_bg_used); s_bg_used = NULL;
	free(s_spr_used); s_spr_used = NULL;
	free(s_smp_buf); s_smp_buf = NULL;
	free(s_mix); s_mix = NULL;
	free(s_bitmap_hp);
	s_bitmap_hp = NULL;
	free(s_snap_bitmap_hp);
	s_snap_bitmap_hp = NULL;
}

static int sq_init(const rbx_game *g)
{
	if (!s_bitmap_hp && !(s_bitmap_hp = calloc(1, sizeof(*s_bitmap_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_snap_bitmap_hp && !(s_snap_bitmap_hp = calloc(1, sizeof(*s_snap_bitmap_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_sqix = s_m->kind >= M_SQIX;
	s_has_mcu = s_m->kind == M_PBILLIAN || s_m->kind == M_SQIX || s_m->kind == M_SQIX_8031;
	s_invert_p2_spinner = !strcmp(g->set, "hotsmash");   // MAME init_hotsmash
	s_invert_coin_lockout = strcmp(g->set, "sqixr0") && strcmp(g->set, "sqixb1");
	(void)s_invert_coin_lockout;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint32_t g1 = 0, g2 = 0, g3 = 0, msize = 0;
	uint8_t *gfx1 = rbx_region("gfx1", &g1);
	if (!s_rom || !gfx1) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!strcmp(g->set, "perestro") || !strcmp(g->set, "perestrof")) {
		uint8_t *gfx2 = rbx_region("gfx2", &g2), *gfx3 = rbx_region("gfx3", &g3);
		if (!gfx2 || !gfx3) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		perestro_decrypt(s_rom, s_rom_size, gfx1, g1, gfx2, g2, gfx3, g3);
	}
	if (s_has_mcu) {
		s_mcu_rom = rbx_region("mcu", &msize);
		if (!s_mcu_rom || msize < (s_sqix ? 0x1000u : 0x800u)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
		if (s_sqix) {
			mcs51_init(&s_i8751, s_mcu_rom, 0x1000);
			s_i8751.port_in = i8751_in;
			s_i8751.port_out = i8751_out;
		} else {
			if (m68705_init(&s_m68705, M68705_P5, s_mcu_rom, 0x800)) {
				rbx_fail("%s: MCU ROM missing", g->set);
				return -1;
			}
			s_m68705.port_r[0] = m68705_porta_r;
			s_m68705.port_w[1] = m68705_port_w;
			s_m68705.port_w[2] = m68705_port_w;
		}
	}
	if (s_sqix) {
		uint8_t *gfx2 = rbx_region("gfx2", &g2), *gfx3 = rbx_region("gfx3", &g3);
		if (!gfx2 || !gfx3) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		s_nchars = (int)(g1 / 32);
		s_nbg = (int)(g2 / 32);
		s_nspr = (int)(g3 / 128);
		s_chars = decode8(gfx1, s_nchars, &s_char_used);
		s_bgtiles = decode8(gfx2, s_nbg, &s_bg_used);
		s_sprites = decode16(gfx3, s_nspr, &s_spr_used);
	} else {
		s_nchars = 0x800;   // pbillian_charlayout: 0x800 chars (does not use the whole ROM)
		s_nspr = (int)(g1 / 128);
		s_chars = decode8(gfx1, s_nchars, &s_char_used);
		s_sprites = decode16(gfx1, s_nspr, &s_spr_used);
		s_nbg = 1;
		s_bgtiles = NULL;
	}
	if (!s_chars || !s_sprites || !s_char_used || !s_spr_used || (s_sqix && (!s_bgtiles || !s_bg_used))) {
		sq_exit();
		rbx_fail("%s: out of memory (graphics)", g->set);
		return -1;
	}
	// samples: 8-bit unsigned to 16-bit signed (MAME init_pbillian)
	s_smp_size = 0;
	if (!s_sqix) {
		uint8_t *smp = rbx_region("samples", &s_smp_size);
		if (!smp) {
			sq_exit();
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		s_smp_buf = (int16_t *)malloc(sizeof(int16_t) * s_smp_size);
		if (!s_smp_buf) {
			sq_exit();
			return -1;
		}
		for (uint32_t i = 0; i < s_smp_size; i++)
			s_smp_buf[i] = (int16_t)((int8_t)(smp[i] ^ 0x80) * 256);
	}
	s_p_dsw1 = rbx_port_find("DSW1");
	s_p_dsw2 = rbx_port_find("DSW2");
	s_p_system = rbx_port_find("SYSTEM");
	s_p_buttons = rbx_port_find("BUTTONS");
	s_p_p1 = rbx_port_find("P1");
	s_p_p2 = rbx_port_find("P2");
	s_p_launch[0] = rbx_port_find("LAUNCH1");
	s_p_launch[1] = rbx_port_find("LAUNCH2");
	s_p_controls = rbx_port_find("CONTROLS");
	s_p_controls2 = rbx_port_find("CONTROLS2");
	// sound: pbillian AY (MAME default legacy output, route 0.30) + samples; sqix two AYs (single output, 0.25)
	if (s_sqix) {
		s_nay = 2;
		for (int i = 0; i < 2; i++)
			ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_SINGLE_OUTPUT, 64, 4, 1);
		s_ay[1].port_a_read = ay2_a;
		s_ay[1].port_b_read = ay2_b;
		s_ay[1].port_b_write = ay2_b_w;
	} else {
		s_nay = 1;
		ay8910_init(&s_ay[0], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 77, 4, 1);
	}
	s_ay[0].port_a_read = ay1_a;
	s_ay[0].port_b_read = ay1_b;
	uint32_t rate = ay8910_rate(&s_ay[0]);
	s_smp_step = (uint32_t)(((uint64_t)3906 << 16) / rate);   // 12 MHz / 3072 (MAME passes the integer 3906 Hz)
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, s_sqix ? 256 : 512)) {
		sq_exit();
		return -1;
	}
	s_dial[0] = s_dial[1] = s_plunger[0] = s_plunger[1] = 0;
	sq_reset();
	return 0;
}

static void sq_frame(void)
{
	for (int p = 0; p < 2; p++) {
		s_dial[p] = (s_dial[p] + rbx_analog_dpad(p + 1, RBX_IPT_DIAL, 3)) & 0xff;   // (d-pad: about MAME's key deltas)
		int d = rbx_analog(p + 1, RBX_IPT_PEDAL);
		if (!d)   // (no pedal: d-pad down pulls the plunger, let go it returns; key delta 16)
			d = (rbx_player_input(p + 1) & RBX_IN_DOWN) ? 16 : -16;
		int pl = s_plunger[p] + d;
		s_plunger[p] = pl < 0 ? 0 : pl > 0xff ? 0xff : pl;
	}
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			if (!s_sqix && s_nmi_mask)
				nmi_set(1);   // MAME vblank_irq
		}
		if (s_sqix && (line & 63) == 0 && s_nmi_mask)
			nmi_set(1);       // MAME sqix_timer_irq at 4 * 60 Hz
		if (s_sqix && s_p_p1 >= 0) {
			int vbl = line > VIS_Y1 || line < (VIS_Y1 + 1 + SQIX_VBL_LINES) - LINES;
			rbx_port_custom(s_p_p1, 0x40, vbl ? 0x40 : 0);
		}
		int zc = rbx_line_cycles(&s_frac_z80, Z80_CLOCK, RATE_MHZ, LINES);
		int ac = rbx_line_cycles(&s_frac_ay, AY_CLOCK, RATE_MHZ, LINES);
		int mc = rbx_line_cycles(&s_frac_mcu, s_sqix ? I8751_CLOCK : M68705_CLOCK, RATE_MHZ, LINES);
		uint32_t line_start = s_snd_done;
		for (int s = 0; s < SLICES; s++) {
			int z0 = zc * s / SLICES, z1 = zc * (s + 1) / SLICES;
			int a0 = ac * s / SLICES, a1 = ac * (s + 1) / SLICES;
			int m0 = mc * s / SLICES, m1 = mc * (s + 1) / SLICES;
			s_slice_start = line_start + (uint32_t)a0;
			s_slice_len = (uint32_t)(a1 - a0);
			s_slice_cycles = z1 - z0;
			s_cpu_start = s_z.cyc;
			unsigned long end = s_z.cyc + (unsigned long)(z1 - z0);
			if (s_z80_reset)
				s_z.cyc = end;
			while ((long)(s_z.cyc - end) < 0)
				z80_step(&s_z);
			if (s_has_mcu) {
				if (s_sqix)
					mcs51_execute(&s_i8751, m1 - m0);
				else
					m68705_execute(&s_m68705, m1 - m0);
			}
			snd_run_to(line_start + (uint32_t)a1);
		}
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_snd_done = 0;
}

static int sq_state(void *ctx, int load)
{
	z80 z = s_z;
	int rc = 0;
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_palram, sizeof(s_palram));
	if (s_sqix)
		rc |= rbx_state_io(ctx, "bitmap", s_bitmap, sizeof(s_bitmap));
	for (int i = 0; i < s_nay; i++)
		rc |= rbx_state_io(ctx, i ? "ay2" : "ay1", &s_ay[i], ay8910_state_size());
	if (s_has_mcu) {
		if (s_sqix)
			rc |= rbx_state_io(ctx, "i8751", &s_i8751.st, sizeof(s_i8751.st));
		else
			rc |= rbx_state_io(ctx, "m68705", &s_m68705.st, sizeof(s_m68705.st));
	}
	int32_t misc[32] = {
		s_z80_reset, s_nmi_line, s_nmi_mask, s_bank, s_gfxbank, s_show_bitmap, s_flip, s_z80_has_written,
		s_mcu_has_written, s_from_z80, s_from_mcu, s_portb_out, s_portc_out, s_porta_in, s_port2_raw, s_bl_port1,
		s_bl_fake_port2, s_bl_port3_out, s_dial_oldpos[0], s_dial_oldpos[1], s_dial_sign[0], s_dial_sign[1],
		s_dial[0], s_dial[1], s_plunger[0], s_plunger[1], (int32_t)s_smp_start, (int32_t)s_smp_len,
		(int32_t)s_smp_pos, 0, 0, 0
	};
	uint64_t fr[3] = { s_frac_z80, s_frac_ay, s_frac_mcu };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	if (rc)
		return -1;
	if (load) {
		z.read_byte = main_read;
		z.write_byte = main_write;
		z.port_in = port_in;
		z.port_out = port_out;
		z.userdata = NULL;
		s_z = z;
		s_z80_reset = misc[0]; s_nmi_line = misc[1]; s_nmi_mask = misc[2]; s_bank = misc[3]; s_gfxbank = misc[4];
		s_show_bitmap = misc[5]; s_flip = misc[6]; s_z80_has_written = misc[7]; s_mcu_has_written = misc[8];
		s_from_z80 = (uint8_t)misc[9]; s_from_mcu = (uint8_t)misc[10]; s_portb_out = (uint8_t)misc[11];
		s_portc_out = (uint8_t)misc[12]; s_porta_in = (uint8_t)misc[13]; s_port2_raw = (uint8_t)misc[14];
		s_bl_port1 = (uint8_t)misc[15]; s_bl_fake_port2 = (uint8_t)misc[16]; s_bl_port3_out = (uint8_t)misc[17];
		s_dial_oldpos[0] = misc[18]; s_dial_oldpos[1] = misc[19]; s_dial_sign[0] = misc[20]; s_dial_sign[1] = misc[21];
		s_dial[0] = misc[22]; s_dial[1] = misc[23]; s_plunger[0] = misc[24]; s_plunger[1] = misc[25];
		s_smp_start = (uint32_t)misc[26]; s_smp_len = (uint32_t)misc[27]; s_smp_pos = (uint32_t)misc[28];
		s_frac_z80 = fr[0]; s_frac_ay = fr[1]; s_frac_mcu = fr[2];
	}
	return 0;
}

static int sq_save(void *ctx) { return sq_state(ctx, 0); }
static int sq_load(void *ctx) { return sq_state(ctx, 1); }
static int sq_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_superqix = { "Super Qix", sq_init, sq_exit, sq_reset, sq_frame, sq_save, sq_load, sq_supported, sq_render };
