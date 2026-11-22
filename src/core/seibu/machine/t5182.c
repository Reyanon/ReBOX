/*  Seibu / Toshiba T5182 sound MCU (see t5182.h).
    Ported from MAME 0.289 src/mame/seibu/t5182.cpp (BSD-3-Clause, copyright-holders: Jonathan Gevaryahu).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "t5182.h"
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"

#include <stddef.h>
#include <string.h>

enum { YM2151_ASSERT, YM2151_CLEAR, YM2151_ACK, CPU_ASSERT, CPU_CLEAR };

static const uint8_t *s_cpu, *s_ext;
static uint32_t s_ext_size;
static int s_coin_port;
static z80 s_z;
static int s_ym_on;

static struct {
	uint8_t ram[0x800], shared[0x100];
	uint8_t irqstate, semaphore_main, semaphore_snd;
	int line;
	uint8_t state_end;
} s;

static void setirq(int param)
{
	switch (param) {
	case YM2151_ASSERT: s.irqstate |= 1 | 4; break;
	case YM2151_CLEAR: s.irqstate &= ~1; break;
	case YM2151_ACK: s.irqstate &= ~4; break;
	case CPU_ASSERT: s.irqstate |= 2; break;
	case CPU_CLEAR: s.irqstate &= ~2; break;
	}
	s.line = s.irqstate != 0;
}

static void ym_irq(int state) { setirq(state ? YM2151_ASSERT : YM2151_CLEAR); }

static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000) return s_cpu[a];
	if (a < 0x4000) return s.ram[a & 0x7ff];
	if (a < 0x8000) return s.shared[a & 0xff];
	a -= 0x8000;
	return a < s_ext_size ? s_ext[a] : 0xff;
}

static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x2000 && a < 0x4000) s.ram[a & 0x7ff] = v;
	else if (a >= 0x4000 && a < 0x8000) s.shared[a & 0xff] = v;
}

static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (p) {
	case 0x00: case 0x01: return rbx_ym_read(0, p & 1);
	case 0x20: return (uint8_t)((s.semaphore_main ? 1 : 0) | (s.irqstate & 2));
	case 0x30: return (uint8_t)rbx_port_read(s_coin_port);
	default: return 0xff;
	}
}

static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p) {
	case 0x00: case 0x01: rbx_ym_write(0, p & 1, v); break;
	case 0x10: s.semaphore_snd = 1; break;
	case 0x11: s.semaphore_snd = 0; break;
	case 0x12: setirq(YM2151_ACK); break;
	case 0x13: setirq(CPU_CLEAR); break;
	default: break;
	}
}

static void z_setup(z80 *z)
{
	z->read_byte = z_read;
	z->write_byte = z_write;
	z->port_in = z_in;
	z->port_out = z_out;
	z->userdata = NULL;
	z->read_map = NULL;
	z->op_rom = NULL;
}

int t5182_init(const uint8_t *cpu, const uint8_t *ext, uint32_t ext_size, int coin_port, int ym_gain)
{
	s_cpu = cpu;
	s_ext = ext;
	s_ext_size = ext ? ext_size : 0;
	s_coin_port = coin_port;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = T5182_CLOCK;
	yc.gain[0] = yc.gain[1] = ym_gain;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc))
		return -1;
	s_ym_on = 1;
	return 0;
}

void t5182_exit(void)
{
	if (s_ym_on)
		rbx_ym_exit(0);
	s_ym_on = 0;
}

void t5182_reset(void)
{
	memset(&s, 0, offsetof(__typeof__(s), state_end));
	z80_init(&s_z);
	z_setup(&s_z);
	rbx_ym_reset(0);
}

uint8_t t5182_sharedram_r(int offset) { return s.shared[offset & 0xff]; }
void t5182_sharedram_w(int offset, uint8_t data) { s.shared[offset & 0xff] = data; }
void t5182_sound_irq_w(void) { setirq(CPU_ASSERT); }
uint8_t t5182_semaphore_snd_r(void) { return s.semaphore_snd ? 1 : 0; }
void t5182_semaphore_main_acquire_w(void) { s.semaphore_main = 1; }
void t5182_semaphore_main_release_w(void) { s.semaphore_main = 0; }

void t5182_run(int cycles)
{
	s_z.int_data = 0xff;
	z80_run(&s_z, s_z.cyc + (unsigned long)cycles, &s.line, 0);
}

int t5182_sound(uint32_t clocks, int32_t *out, int max) { return rbx_ym_advance(0, clocks, out, max); }

int t5182_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "t5182", &s, (uint32_t)offsetof(__typeof__(s), state_end));
	z80 c = s_z;
	rc |= rbx_state_io(ctx, "t5182_z80", &c, sizeof(c));
	if (!rc && load) {
		z_setup(&c);
		s_z = c;
	}
	rc |= rbx_ym_state(0, ctx, load);
	return rc;
}
