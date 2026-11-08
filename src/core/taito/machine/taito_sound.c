/*  The Taito sound board (taito_sound.h). Memory maps from MAME taito_b.cpp / taito_x.cpp (GPL-2.0+, Jarek Burczynski,
    Howie Cohen, Yochizo and others) and taitosnd.cpp (BSD-3-Clause, Philip Bennett).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "taito_sound.h"
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static tsnd_config s_cfg;
static z80 s_z80;
static tc0140syt_t s_syt;
static uint8_t *s_rom;
static uint32_t s_rom_size, s_banks;
static uint8_t s_ram[0x2000];
static int s_bank, s_reset_line, s_ym_irq;
static okim6295_t s_oki;
static int s_has_oki;
static uint32_t s_ym_done, s_ym_line;
static int s_line_target;
static unsigned long s_line_start;
static int32_t *s_mix;
static int32_t *s_mix_r, *s_mix_t;   // (stereo: the right side; ext_mix's scratch)
static int s_mix_cap, s_mix_n;
static uint32_t s_oki_acc;
static int16_t s_oki_hold;
static uint64_t s_z80_frac, s_ym_frac;

int tsnd_line_cycles(uint64_t *frac, uint32_t clock)
{
	// clock / (rate * lines) per line, rate in milli-Hz
	uint64_t den = (uint64_t)s_cfg.frame_rate_mhz * (uint64_t)s_cfg.lines;
	*frac += (uint64_t)clock * 1000;
	uint64_t n = rbx_div64_small(*frac, den);
	*frac -= n * den;
	return (int)n;
}

static void nmi_cb(int state)
{
	if (state)
		z80_gen_nmi(&s_z80);
}

static uint8_t z80_read(void *u, uint16_t a);
static void z80_write(void *u, uint16_t a, uint8_t v);
static void sync(void);
static uint8_t z80_in(z80 *z, uint8_t p)
{
	(void)z;
	if (!s_cfg.port_in)
		return 0xff;
	sync();
	return s_cfg.port_in(p);
}
static void z80_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (s_cfg.port_out) {
		sync();
		s_cfg.port_out(p, v);
	}
}

// Z80 read map: ROM, the banked ROM window and RAM read without a z80_read call (the same bytes)
static const uint8_t *s_map[256];
static int s_map_bank = -1;   // bank in the map's 4000-7fff window

static void map_range(uint32_t start, uint32_t end, const uint8_t *mem)
{
	for (uint32_t a = start; a < end; a += 0x100)
		s_map[a >> 8] = mem ? mem + (a - start) : NULL;
}

static void map_bank(void)
{
	if (s_cfg.rom_flat || s_cfg.map == TSND_MAP_8800 || s_bank == s_map_bank)
		return;
	uint32_t o = s_cfg.bank_base + (uint32_t)s_bank * 0x4000;
	if (s_bank == 0 && s_cfg.bank0_offset)
		o = s_cfg.bank0_offset;
	map_range(0x4000, 0x8000, o + 0x4000 <= s_rom_size ? s_rom + o : NULL);
	s_map_bank = s_bank;
}

static void update_map(void)
{
	memset(s_map, 0, sizeof(s_map));
	map_range(0x0000, 0x4000, s_rom);
	if ((s_cfg.rom_flat || s_cfg.map == TSND_MAP_8800) && s_rom_size >= 0x8000)
		map_range(0x4000, 0x8000, s_rom + 0x4000);
	s_map_bank = -1;
	map_bank();
	if (s_cfg.map == TSND_MAP_8800)
		map_range(0x8000, 0x8800, s_ram);
	else if (s_cfg.map == TSND_MAP_9000)
		map_range(0x8000, 0x9000, s_ram);
	else
		map_range(0xc000, 0xe000, s_ram);
	s_z80.read_map = s_map;
}

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	update_map();
}

static void reset_cb(int state)
{
	if (state && !s_reset_line) {
		unsigned long cyc = s_z80.cyc;
		uint16_t sp = s_z80.sp;   // MAME z80 device_reset keeps SP (volfied's NMI can come before its SP setup)
		z80_start();
		s_z80.cyc = cyc;
		s_z80.sp = sp;
	}
	s_reset_line = state;
}

static void ym_irq(int state) { s_ym_irq = state; }
static void ym_port_a(uint8_t data) { s_bank = data & 3; map_bank(); }

// run the chips up to the Z80's position in the current line
static void sync(void)
{
	uint32_t done = (uint32_t)(s_z80.cyc - s_line_start);
	uint32_t target = s_line_target ? rbx_muldiv(s_ym_line, done, (uint32_t)s_line_target) : 0;
	if (target > s_ym_line)
		target = s_ym_line;
	if (target <= s_ym_done)
		return;
	int n = s_cfg.stereo ? rbx_ym_advance_lr(0, target - s_ym_done, s_mix + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n)
	                     : rbx_ym_advance(0, target - s_ym_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const int i0 = s_mix_n;   // (the mono additions below go to the right side too)
	if (s_has_oki) {
		uint32_t yr = rbx_ym_rate(0);
		for (int i = 0; i < n; i++) {
			s_oki_acc += s_oki.rate;
			while (s_oki_acc >= yr) {
				s_oki_acc -= yr;
				s_oki_hold = (int16_t)oki_sample(&s_oki);
			}
			s_mix[s_mix_n + i] += s_oki_hold;
			if (s_cfg.stereo)
				s_mix_r[s_mix_n + i] += s_oki_hold;
		}
	}
	if (s_cfg.ext_mix && n > 0) {
		if (s_cfg.stereo) {   // (mono chips: their part, the difference, added to the right too)
			for (int i = 0; i < n; i++)
				s_mix_t[i] = s_mix[i0 + i];
			s_cfg.ext_mix(s_mix + s_mix_n, n, rbx_ym_rate(0));
			for (int i = 0; i < n; i++)
				s_mix_r[i0 + i] += s_mix[i0 + i] - s_mix_t[i];
		} else
			s_cfg.ext_mix(s_mix + s_mix_n, n, rbx_ym_rate(0));
	}
	s_mix_n += n;
	s_ym_done = target;
}

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000)
		return s_rom[a];
	if (a < 0x8000) {
		if (s_cfg.rom_flat || s_cfg.map == TSND_MAP_8800)
			return a < s_rom_size ? s_rom[a] : 0xff;
		uint32_t o = s_cfg.bank_base + (uint32_t)s_bank * 0x4000 + (a - 0x4000);
		if (s_bank == 0 && s_cfg.bank0_offset)
			o = s_cfg.bank0_offset + (a - 0x4000);
		return o < s_rom_size ? s_rom[o] : 0xff;
	}
	if (s_cfg.map == TSND_MAP_8800) {
		if (a < 0x8800) return s_ram[a & 0x7ff];
		if (a == 0x8801) return tc0140syt_slave_comm_r(&s_syt);
		if (a == 0x9000 || a == 0x9001) { sync(); return rbx_ym_read(0, a & 1); }
		return 0xff;
	}
	if (s_cfg.map == TSND_MAP_9000) {
		if (a < 0x9000) return s_ram[a & 0xfff];
		if (a < 0x9002) { sync(); return rbx_ym_read(0, a & 1); }
		if (a == 0xa001) return tc0140syt_slave_comm_r(&s_syt);
		if (s_has_oki && (a == 0xb000 || a == 0xb001)) { sync(); return oki_read(&s_oki); }
		return 0xff;
	}
	if (a >= 0xc000 && a < 0xe000) return s_ram[a & 0x1fff];
	if (a >= 0xe000 && a <= 0xe003) { sync(); return rbx_ym_read(0, a & 3); }
	if (a == 0xe201) return tc0140syt_slave_comm_r(&s_syt);
	return 0xff;
}

static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_cfg.map == TSND_MAP_8800) {
		if (a >= 0x8000 && a < 0x8800) { s_ram[a & 0x7ff] = v; return; }
		if (a == 0x8800) { tc0140syt_slave_port_w(&s_syt, v); return; }
		if (a == 0x8801) { tc0140syt_slave_comm_w(&s_syt, v); return; }
		if (a == 0x9000 || a == 0x9001) { sync(); rbx_ym_write(0, a & 1, v); return; }
		return;
	}
	if (s_cfg.map == TSND_MAP_9000) {
		if (a >= 0x8000 && a < 0x9000) { s_ram[a & 0xfff] = v; return; }
		if (a >= 0x9000 && a < 0x9002) { sync(); rbx_ym_write(0, a & 1, v); return; }
		if (a == 0xa000) { tc0140syt_slave_port_w(&s_syt, v); return; }
		if (a == 0xa001) { tc0140syt_slave_comm_w(&s_syt, v); return; }
		if (s_has_oki && (a == 0xb000 || a == 0xb001)) { sync(); oki_write(&s_oki, v); return; }
		if (s_cfg.ext_write && !s_has_oki && a >= 0xb000) { sync(); s_cfg.ext_write(a, v); return; }
		return;
	}
	if (a >= 0xc000 && a < 0xe000) { s_ram[a & 0x1fff] = v; return; }
	if (a >= 0xe000 && a <= 0xe003) { sync(); rbx_ym_write(0, a & 3, v); return; }
	if (a == 0xe200) { tc0140syt_slave_port_w(&s_syt, v); return; }
	if (a == 0xe201) { tc0140syt_slave_comm_w(&s_syt, v); return; }
	if (a == 0xf200) { s_bank = (int)((v & (s_cfg.bank_mask ? s_cfg.bank_mask : 3)) % (s_banks ? s_banks : 1)); map_bank(); return; }
	if (s_cfg.ext_write && a >= 0xe000) { sync(); s_cfg.ext_write(a, v); }   // board latches (ninjaw pan e400)
}

int tsnd_init(const tsnd_config *cfg)
{
	s_cfg = *cfg;
	s_rom = rbx_region("audiocpu", &s_rom_size);
	if (!s_rom)
		return -1;
	s_banks = s_rom_size / 0x4000;
	rbx_ym_config yc = cfg->ym;
	yc.irq = ym_irq;
	if (yc.type == RBX_YM2203 || cfg->ym_port_bank)
		yc.port_a_write = ym_port_a;
	if (yc.type == RBX_YM2610 || yc.type == RBX_YM2610B) {
		yc.adpcma = rbx_region("ymsnd:adpcma", &yc.adpcma_size);
		yc.adpcmb = rbx_region("ymsnd:adpcmb", &yc.adpcmb_size);
		if (!yc.adpcmb) {   // one sample ROM: ADPCM-B reads the ADPCM-A region (MAME)
			yc.adpcmb = yc.adpcma;
			yc.adpcmb_size = yc.adpcma_size;
		}
	}
	if (rbx_ym_init(0, &yc))
		return -1;
	// (mono boards: the YM's samples made on the worker, the timers and the reads here: the same sound, a third or
	// more of the emulation thread's time in play)
	if (rbx_ym_set_async(0, cfg->stereo ? 2 : 1))
		return -1;
	s_has_oki = cfg->oki_clock != 0;
	if (s_has_oki) {
		uint32_t osize;
		uint8_t *orom = rbx_region("oki", &osize);
		oki_init(&s_oki, cfg->oki_clock, cfg->oki_pin7_high, orom, osize, cfg->oki_gain);
	}
	s_syt.nmi = nmi_cb;
	s_syt.reset = reset_cb;
	s_syt.sync = NULL;
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (cfg->stereo) {
		s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_mix_t = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	}
	if (!s_mix || (cfg->stereo && (!s_mix_r || !s_mix_t)) ||
	    (cfg->stereo ? rbx_audio_setup_stereo(rbx_ym_rate(0)) : rbx_audio_setup(rbx_ym_rate(0))))
		return -1;
	tsnd_reset();
	return 0;
}

void tsnd_exit(void)
{
	free(s_mix);
	free(s_mix_r);
	free(s_mix_t);
	s_mix = s_mix_r = s_mix_t = NULL;
	rbx_ym_exit(0);
}

void tsnd_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	tc0140syt_reset(&s_syt);
	z80_start();
	s_reset_line = 0;
	s_bank = 0;
	update_map();
	s_ym_irq = 0;
	rbx_ym_reset(0);
	if (s_has_oki)
		oki_reset(&s_oki);
	s_z80_frac = s_ym_frac = 0;
	s_mix_n = 0;
	s_oki_acc = 0;
	s_oki_hold = 0;
}

tc0140syt_t *tsnd_syt(void) { return &s_syt; }
void tsnd_sync(void) { sync(); }

void tsnd_line(void)
{
	s_ym_line = (uint32_t)tsnd_line_cycles(&s_ym_frac, s_cfg.ym.clock);
	s_ym_done = 0;
	int cycles = tsnd_line_cycles(&s_z80_frac, s_cfg.z80_clock);
	s_line_target = cycles;
	s_line_start = s_z80.cyc;
	if (s_reset_line) {
		s_z80.cyc += (unsigned long)cycles;
	} else {
		unsigned long end = s_z80.cyc + (unsigned long)cycles;
		z80_run(&s_z80, end, &s_ym_irq, 0);   // level-triggered INT from the YM
	}
	sync();
}

void tsnd_frame_end(void)
{
	if (s_cfg.stereo) {
		rbx_ym_async_take_lr(0, s_mix, s_mix_r, s_mix_n);
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put_stereo(s_mix[i], s_mix_r[i]);
		memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	} else {
		rbx_ym_async_take(0, s_mix, s_mix_n);
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put(s_mix[i]);
	}
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

int tsnd_state(void *ctx, int load)
{
	z80 z = s_z80;
	int misc[6] = { s_bank, s_reset_line, s_ym_irq, (int)s_oki_acc, s_oki_hold, 0 };
	uint64_t fr[2] = { s_z80_frac, s_ym_frac };
	int rc = 0;
	rc |= rbx_state_io(ctx, "syt", &s_syt, offsetof(tc0140syt_t, nmi));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "zmisc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "zfrac", fr, sizeof(fr));
	if (s_has_oki)
		rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	if (rc)
		return -1;
	if (load) {
		z.read_byte = z80_read;
		z.write_byte = z80_write;
		z.port_in = z80_in;
		z.port_out = z80_out;
		z.userdata = NULL;
		s_z80 = z;
		s_bank = misc[0]; s_reset_line = misc[1]; s_ym_irq = misc[2];
		s_oki_acc = (uint32_t)misc[3]; s_oki_hold = (int16_t)misc[4];
		s_z80_frac = fr[0]; s_ym_frac = fr[1];
		update_map();
	}
	return 0;
}
