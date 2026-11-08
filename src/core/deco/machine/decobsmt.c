/*  Data East Pinball BSMT2000 sound board, after MAME src/mame/shared/decobsmt.cpp and
    src/devices/sound/bsmt2000.cpp (BSD-3-Clause). See decobsmt.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "decobsmt.h"
#include "../../rbx.h"

#include <string.h>

#define MASTER 24000000
#define FIRQ_HZ 489

static void set_irq(decobsmt_t *d, int state)
{
	d->s.irq = (uint8_t)state;
	m6809_set_irq(&d->cpu, state);
}

static void bsmt_reset(decobsmt_t *d)
{
	tms320c1x_reset(&d->tms);
}

// ---- the 6809 ----
static uint8_t cpu_r(void *ctx, uint16_t a)
{
	decobsmt_t *d = (decobsmt_t *)ctx;
	if (a < 0x2000) return d->ram[a];
	if (a == 0x2002 || a == 0x2003) return d->s.comms;
	if (a == 0x2006 || a == 0x2007) return (uint8_t)((d->s.write_pending ? 0 : 1) << 7);
	return d->rom[a];
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	decobsmt_t *d = (decobsmt_t *)ctx;
	if (a < 0x2000) {
		d->ram[a] = v;
	} else if (a == 0x2000 || a == 0x2001) {   // bsmt_reset_w: the BSMT reset on bit 7 going low
		const uint8_t diff = v ^ d->s.reset;
		d->s.reset = v;
		if ((diff & 0x80) && !(v & 0x80))
			bsmt_reset(d);
	} else if (a == 0x6000) {
		d->s.latch = v;
	} else if (a >= 0xa000 && a < 0xa100) {   // bsmt1_w: the register (offset ^ 0xff) and the data word
		d->s.reg_select = (uint16_t)((a & 0xff) ^ 0xff);
		d->s.write_data = (uint16_t)(d->s.latch << 8 | v);
		d->s.write_pending = 1;
		set_irq(d, 0);   // (the BSMT is not ready)
	}
}

// ---- the TMS320C15 ----
static uint16_t tms_in(void *user, int port)
{
	decobsmt_t *d = (decobsmt_t *)user;
	switch (port) {
	case 0: return d->s.reg_select;
	case 1:   // tms_data_r: the word taken, the 6809 told
		d->s.write_pending = 0;
		set_irq(d, 1);
		return d->s.write_data;
	case 2: {   // tms_rom_r: the sample byte, signed, high
		const uint32_t a = ((uint32_t)d->s.rom_bank << 16) + d->s.rom_address;
		const uint8_t b = a < d->samples_size ? d->samples[a] : 0;
		return (uint16_t)(int16_t)(b << 8);
	}
	}
	return 0;
}
static void tms_out(void *user, int port, uint16_t data)
{
	decobsmt_t *d = (decobsmt_t *)user;
	switch (port) {
	case 0: d->s.rom_address = data; break;
	case 1: d->s.rom_bank = data; break;
	case 3: d->s.left = (int16_t)data; break;
	case 7: d->s.right = (int16_t)data; break;
	}
}
static int tms_bio(void *user) { return ((decobsmt_t *)user)->s.write_pending; }

void decobsmt_init(decobsmt_t *d, const uint8_t *rom, const uint8_t *prog, const uint8_t *samples, uint32_t samples_size)
{
	memset(d, 0, sizeof(*d));
	d->rom = rom;
	d->samples = samples;
	d->samples_size = samples_size;
	for (int i = 0; i < 0x1000; i++)
		d->prog[i] = (uint16_t)(prog[2 * i] << 8 | prog[2 * i + 1]);
	m6809_init(&d->cpu);
	d->cpu.read = cpu_r;
	d->cpu.write = cpu_w;
	d->cpu.ctx = d;
	for (int p = 0; p < 0x20; p++)
		d->cpu.rpage[p] = d->cpu.wpage[p] = d->ram + p * 256;
	for (int p = 0x21; p < 0x100; p++)   // (0x2000 .. 0x20ff: the registers; 0x6000, 0xa0xx are writes only)
		d->cpu.rpage[p] = d->rom + p * 256;
	tms320c1x_init(&d->tms);
	d->tms.rom = d->prog;
	d->tms.rom_mask = 0xfff;
	d->tms.in = tms_in;
	d->tms.out = tms_out;
	d->tms.bio = tms_bio;
	d->tms.user = d;
}

void decobsmt_reset(decobsmt_t *d)
{
	memset(&d->s, 0, sizeof(d->s));
	memset(d->ram, 0, sizeof(d->ram));
	m6809_reset(&d->cpu);
	m6809_set_irq(&d->cpu, 0);
	m6809_set_firq(&d->cpu, 0);
	bsmt_reset(d);
}

void decobsmt_comms_w(decobsmt_t *d, uint8_t data) { d->s.comms = data; }

void decobsmt_reset_line(decobsmt_t *d, int state)
{
	if (d->s.cpu_reset && !state) {   // (held in reset: running again from its vector; its clock kept)
		const int64_t cyc = d->cpu.cycles;
		m6809_reset(&d->cpu);
		d->cpu.cycles = cyc;
		m6809_set_irq(&d->cpu, d->s.irq);
		m6809_set_firq(&d->cpu, d->s.firq);
	}
	d->s.cpu_reset = (uint8_t)(state != 0);
}

// one 187.5 kHz sample's time (128 master clocks): the 6809 and the TMS to it
static void run_slice(decobsmt_t *d)
{
	d->s.clock += 128;
	d->s.firq_frac += 128 * FIRQ_HZ;
	if (d->s.firq_frac >= MASTER) {   // (the periodic FIRQ, HOLD_LINE)
		d->s.firq_frac -= MASTER;
		d->s.firq = 1;
		m6809_set_firq(&d->cpu, 1);
	}
	const int64_t t6809 = d->s.clock / 12;
	if (d->s.cpu_reset) {
		d->cpu.cycles = t6809;
	} else {
		if (d->s.firq && !(d->cpu.cc & 0x40)) {   // (taken at the next instruction: the line dropped after it)
			m6809_step(&d->cpu);
			d->s.firq = 0;
			m6809_set_firq(&d->cpu, 0);
		}
		m6809_run_until(&d->cpu, t6809);
	}
	const int64_t ttms = d->s.clock / 4;
	if (d->tms.cycles < ttms)
		tms320c1x_execute(&d->tms, (int)(ttms - d->tms.cycles));
}

int decobsmt_run(decobsmt_t *d, uint32_t clocks, int32_t *out_l, int32_t *out_r, int max)
{
	int n = 0;
	const int64_t end = d->s.clock + d->s.carry + clocks;   // (with the last call's remainder)
	while (d->s.clock + 128 <= end) {
		run_slice(d);
		d->s.acc_l += d->s.left;
		d->s.acc_r += d->s.right;
		if (++d->s.acc_n == 4) {
			if (n < max) {
				out_l[n] += d->s.acc_l / 4;
				out_r[n] += d->s.acc_r / 4;
				n++;
			}
			d->s.acc_l = d->s.acc_r = 0;
			d->s.acc_n = 0;
		}
	}
	d->s.carry = (uint32_t)(end - d->s.clock);
	return n;
}

int decobsmt_state(decobsmt_t *d, void *ctx, int load)
{
	uint8_t cpu[M6809_STATE_SIZE], tms[TMS320C1X_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &d->cpu, M6809_STATE_SIZE);
		memcpy(tms, &d->tms, TMS320C1X_STATE_SIZE);
	}
	int rc = 0;
	rc |= rbx_state_io(ctx, "bsmt_cpu", cpu, sizeof(cpu));
	rc |= rbx_state_io(ctx, "bsmt_tms", tms, sizeof(tms));
	rc |= rbx_state_io(ctx, "bsmt_s", &d->s, sizeof(d->s));
	rc |= rbx_state_io(ctx, "bsmt_ram", d->ram, sizeof(d->ram));
	if (!rc && load) {
		memcpy(&d->cpu, cpu, M6809_STATE_SIZE);
		memcpy(&d->tms, tms, TMS320C1X_STATE_SIZE);
	}
	return rc ? -1 : 0;
}
