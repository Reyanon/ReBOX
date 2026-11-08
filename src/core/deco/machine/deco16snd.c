/*  The Data East 16-bit boards' sound board (deco16snd.h). After MAME's sound_map of the DECO 16 drivers
    (src/mame/dataeast/supbtime.cpp etc., BSD-3-Clause, copyright-holders: Bryan McPhail, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "deco16snd.h"
#include "../../rbx.h"
#include "../../cpu/h6280/h6280.h"
#include "../../sound/rbx_ym.h"

#include <stdlib.h>
#include <string.h>

static deco16snd_cfg s_cfg;
static h6280_t s_cpu;
static const uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x2000];
static okim6295_t s_oki[2];
static int s_nokis;
static uint8_t s_latch, s_latch_pending;
static uint32_t s_frame_mhz;
static int s_lines;
static uint64_t s_cpu_frac, s_ym_frac;
static uint32_t s_out_rate, s_oki_phase[2], s_ym_line_clocks;
static int32_t s_oki_last[2];
static int32_t *s_mix_l, *s_mix_r, *s_ym2;
static int s_ym2_cap;   // (a frame of the YM2203's samples: 5800 at its 335 kHz)
static int s_mix_cap, s_mix_n;
static uint64_t s_ym2_frac;
static int s_ym2_n;   // the YM2203's samples this frame (made on the worker, taken at the frame's end)

static void ym_irq(int state) { h6280_set_irq(&s_cpu, H6280_IRQ2, state); }

static uint8_t snd_r(void *ctx, uint32_t pa)
{
	(void)ctx;
	if (pa < 0x10000) return s_rom[pa & (s_rom_size - 1)];
	if ((pa & ~1u) == 0x110000) return rbx_ym_read(0, pa & 1);
	if ((pa & ~1u) == 0x100000) return s_cfg.ym2203_clock ? rbx_ym_read(1, pa & 1) : 0;
	if ((pa & ~1u) == 0x120000) return oki_read(&s_oki[0]);
	if ((pa & ~1u) == 0x130000) return s_nokis > 1 ? oki_read(&s_oki[1]) : 0;
	if ((pa & ~1u) == 0x140000) {
		if (s_cfg.latch_r)
			return s_cfg.latch_r();
		s_latch_pending = 0;   // (generic latch: read clears its interrupt)
		h6280_set_irq(&s_cpu, H6280_IRQ1, 0);
		return s_latch;
	}
	if (pa >= 0x1f0000 && pa < 0x1f2000) return s_ram[pa & 0x1fff];
	return 0;
}
static void snd_w(void *ctx, uint32_t pa, uint8_t v)
{
	(void)ctx;
	if ((pa & ~1u) == 0x110000) rbx_ym_write(0, pa & 1, v);
	else if ((pa & ~1u) == 0x100000) { if (s_cfg.ym2203_clock) rbx_ym_write(1, pa & 1, v); }
	else if ((pa & ~1u) == 0x120000) oki_write(&s_oki[0], v);
	else if ((pa & ~1u) == 0x130000) { if (s_nokis > 1) oki_write(&s_oki[1], v); }
	else if (pa >= 0x1f0000 && pa < 0x1f2000) s_ram[pa & 0x1fff] = v;
}

void deco16snd_latch_w(uint8_t v)
{
	s_latch = v;
	s_latch_pending = 1;
	h6280_set_irq(&s_cpu, H6280_IRQ1, 1);
}
void deco16snd_latch_irq(int state) { h6280_set_irq(&s_cpu, H6280_IRQ1, state); }
okim6295_t *deco16snd_oki(int i) { return &s_oki[i]; }

int deco16snd_init(const uint8_t *rom, uint32_t rom_size, const deco16snd_cfg *cfg, uint32_t frame_mhz, int lines)
{
	s_cfg = *cfg;
	s_rom = rom;
	s_rom_size = rom_size;
	s_frame_mhz = frame_mhz;
	s_lines = lines;
	s_nokis = cfg->oki_rom[1] ? 2 : 1;
	for (int i = 0; i < s_nokis; i++)
		oki_init(&s_oki[i], cfg->oki_clock[i], 1, cfg->oki_rom[i], cfg->oki_size[i], cfg->oki_gain[i]);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = cfg->ym_clock;
	yc.gain[0] = cfg->ym_gain[0];
	yc.gain[1] = cfg->ym_gain[1];
	yc.irq = ym_irq;
	yc.port_a_write = cfg->ym_port;
	// (the YM2151's samples made on the worker: rbx_ym_async_take_lr at the frame's end)
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2))
		return -1;
	if (cfg->ym2203_clock) {
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM2203;
		yc.clock = cfg->ym2203_clock;
		for (int i = 0; i < 4; i++)
			yc.gain[i] = cfg->ym2203_gain[i];
		// (its samples on the worker too: rbx_ym_async_take at the frame's end)
		if (rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1))
			return -1;
	}
	s_out_rate = rbx_ym_rate(0);
	s_mix_cap = (int)(s_out_rate / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_ym2_cap = cfg->ym2203_clock ? (int)(rbx_ym_rate(1) / 50) + 256 : 1;
	s_ym2 = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || !s_ym2 || (cfg->stereo ? rbx_audio_setup_stereo(s_out_rate) : rbx_audio_setup(s_out_rate))) {
		deco16snd_exit();
		return -1;
	}
	h6280_init(&s_cpu);
	for (uint32_t a = 0; a < 0x10000 && a < rom_size; a += 0x2000)
		s_cpu.rpage[a >> 13] = (uint8_t *)rom + a;
	s_cpu.rpage[0x1f0000 >> 13] = s_cpu.wpage[0x1f0000 >> 13] = s_ram;
	s_cpu.read = snd_r;
	s_cpu.write = snd_w;
	if (cfg->timer_scale)
		s_cpu.timer_scale = cfg->timer_scale;
	return 0;
}

void deco16snd_exit(void)
{
	rbx_ym_exit(0);
	if (s_cfg.ym2203_clock)
		rbx_ym_exit(1);
	free(s_mix_l);
	free(s_mix_r);
	free(s_ym2);
	s_mix_l = s_mix_r = s_ym2 = NULL;
}

void deco16snd_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	for (int i = 0; i < s_nokis; i++) {
		oki_reset(&s_oki[i]);
		s_oki[i].bank = 0;
		s_oki_phase[i] = 0;
		s_oki_last[i] = 0;
	}
	s_latch = s_latch_pending = 0;
	rbx_ym_reset(0);
	if (s_cfg.ym2203_clock)
		rbx_ym_reset(1);
	h6280_reset(&s_cpu);
	s_cpu_frac = s_ym_frac = s_ym2_frac = 0;
	s_ym2_n = 0;
	s_mix_n = 0;
}

void deco16snd_line(void)
{
	h6280_run_until(&s_cpu, s_cpu.cycles + rbx_line_cycles(&s_cpu_frac, s_cfg.cpu_clock, s_frame_mhz, s_lines));
	h6280_break = 0;
	s_ym_line_clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, s_cfg.ym_clock, s_frame_mhz, s_lines);
	const int n = rbx_ym_advance_lr(0, s_ym_line_clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	if (s_cfg.ym2203_clock) {   // (the YM2203's samples of the frame, mixed in at its end)
		const uint32_t c2 = (uint32_t)rbx_line_cycles(&s_ym2_frac, s_cfg.ym2203_clock, s_frame_mhz, s_lines);
		s_ym2_n += rbx_ym_advance(1, c2, s_ym2 + s_ym2_n, s_ym2_cap - s_ym2_n);
	}
	for (int i = 0; i < n; i++) {   // the OKIs (to both sides) at their own rates, held
		int32_t o = 0;
		for (int k = 0; k < s_nokis; k++) {
			s_oki_phase[k] += s_oki[k].rate;
			while (s_oki_phase[k] >= s_out_rate) {
				s_oki_phase[k] -= s_out_rate;
				s_oki_last[k] = oki_sample(&s_oki[k]);
			}
			o += s_oki_last[k];
		}
		s_mix_l[s_mix_n + i] += o;
		if (s_cfg.stereo)   // (one speaker: the sides are added at the end, the OKIs once)
			s_mix_r[s_mix_n + i] += o;
	}
	s_mix_n += n;
}

void deco16snd_frame_end(void)
{
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	if (s_cfg.ym2203_clock)
		rbx_ym_async_take(1, s_ym2, s_ym2_n);
	if (s_cfg.ym2203_clock && s_ym2_n > 0) {   // the YM2203's frame held at the YM2151's rate (both sides)
		for (int i = 0; i < s_mix_n; i++) {
			const int32_t v = s_ym2[(int)((int64_t)i * s_ym2_n / s_mix_n)];
			s_mix_l[i] += v;
			if (s_cfg.stereo)
				s_mix_r[i] += v;
		}
		memset(s_ym2, 0, (size_t)s_ym2_n * sizeof(int32_t));   // (rbx_ym_advance adds)
	}
	s_ym2_n = 0;
	if (s_cfg.stereo)
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	else
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put(s_mix_l[i] + s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

int deco16snd_state(void *ctx, int load)
{
	uint8_t cpu[H6280_STATE_SIZE];
	if (!load)
		memcpy(cpu, &s_cpu, H6280_STATE_SIZE);
	uint64_t fr[3] = { s_cpu_frac, s_ym_frac, s_ym2_frac };
	int32_t misc[8] = { s_latch | s_latch_pending << 8, (int32_t)s_oki[0].bank, (int32_t)s_oki[1].bank, (int32_t)s_oki_phase[0],
	                    (int32_t)s_oki_phase[1], s_oki_last[0], s_oki_last[1], 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "h6280", cpu, H6280_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_ram, sizeof(s_ram));
	for (int i = 0; i < s_nokis; i++)
		rc |= rbx_state_io(ctx, i ? "oki2" : "oki1", s_oki[i].v, sizeof(s_oki[i].v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	if (s_cfg.ym2203_clock)
		rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "sfrac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "smisc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		memcpy(&s_cpu, cpu, H6280_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_ym_frac = fr[1];
		s_ym2_frac = fr[2];
		s_latch = (uint8_t)misc[0];
		s_latch_pending = (uint8_t)(misc[0] >> 8);
		s_oki[0].bank = (uint32_t)misc[1];
		s_oki[1].bank = (uint32_t)misc[2];
		s_oki_phase[0] = (uint32_t)misc[3];
		s_oki_phase[1] = (uint32_t)misc[4];
		s_oki_last[0] = misc[5];
		s_oki_last[1] = misc[6];
	}
	return 0;
}
