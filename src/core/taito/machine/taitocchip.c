/*  Taito TC0030CMD "C-Chip", ported from MAME src/mame/taito/taitocchip.cpp (BSD-3-Clause, copyright-holders:
    Jonathan Gevaryahu, David Haywood). See taitocchip.h.
    MCU map (MAME cchip_map): 0000-0fff internal ROM, 1000-13ff SRAM bank, 1400-17ff ASIC, 2000-3fff EPROM,
    ff00-ffff internal RAM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "taitocchip.h"
#include "../../rbx.h"
#include "../../cpu/upd7810/upd7810.h"

#include <string.h>

static cchip_config s_cfg;
static const uint8_t *s_eprom;
static uint32_t s_eprom_size;
static struct {
	uint8_t sram[0x2000];      // uPD4464
	uint8_t asic_ram[4];
	uint8_t bank, bank68;      // 1 KB SRAM windows of the MCU and of the 68000
	uint8_t pad[2];
} s;

static uint8_t asic_r(uint32_t offset)
{
	if (offset < 0x200)   // 400-5ff is asic 'ram'
		return s.asic_ram[offset & 3];
	return 0x00;          // 600-7ff is write-only(?) asic banking reg
}

static void asic_w(uint32_t offset, uint8_t data)
{
	if (offset == 0x200)
		s.bank = data & 7;
	else
		s.asic_ram[offset & 3] = data;
}

static uint8_t mcu_read(uint16_t a)
{
	if (a >= 0x1000 && a < 0x1400) return s.sram[s.bank * 0x400 + (a & 0x3ff)];
	if (a >= 0x1400 && a < 0x1800) return asic_r(a - 0x1400u);
	if (a >= 0x2000 && a < 0x4000) return (uint32_t)(a - 0x2000) < s_eprom_size ? s_eprom[a - 0x2000] : 0xff;
	return 0xff;
}

static void mcu_write(uint16_t a, uint8_t v)
{
	if (a >= 0x1000 && a < 0x1400) { s.sram[s.bank * 0x400 + (a & 0x3ff)] = v; return; }
	if (a >= 0x1400 && a < 0x1800) { asic_w(a - 0x1400u, v); return; }
}

static uint8_t port_in(int port)
{
	uint8_t (*f)(void) = NULL;
	switch (port) {
	case UPD7810_PORTA: f = s_cfg.in_pa; break;
	case UPD7810_PORTB: f = s_cfg.in_pb; break;
	case UPD7810_PORTC: f = s_cfg.in_pc; break;
	default: return 0xff;   // port D / F not connected (MAME default 0xff)
	}
	return f ? f() : 0;
}

static void port_out(int port, uint8_t v)
{
	switch (port) {
	case UPD7810_PORTA: if (s_cfg.out_pa) s_cfg.out_pa(v); break;
	case UPD7810_PORTB: if (s_cfg.out_pb) s_cfg.out_pb(v); break;
	case UPD7810_PORTC: if (s_cfg.out_pc) s_cfg.out_pc(v); break;
	default: break;         // port F: internal? (MAME logs it)
	}
}

static uint8_t an_in(int n)
{
	uint8_t v = s_cfg.in_ad ? s_cfg.in_ad() : 0;
	return ((v >> n) & 1) ? 0xff : 0;
}

int cchip_init(const cchip_config *cfg)
{
	uint32_t isize;
	const uint8_t *irom = rbx_region("cchip:upd7811", &isize);
	s_eprom = rbx_region("cchip:cchip_eprom", &s_eprom_size);
	if (!irom || !s_eprom)
		return -1;
	s_cfg = *cfg;
	upd7810_config uc;
	memset(&uc, 0, sizeof(uc));
	uc.irom = irom;
	uc.irom_size = isize < 0x1000 ? isize : 0x1000;
	uc.read = mcu_read;
	uc.write = mcu_write;
	uc.port_in = port_in;
	uc.in_ports = 1 << UPD7810_PORTA | 1 << UPD7810_PORTB | 1 << UPD7810_PORTC;
	uc.port_out = port_out;
	uc.an_in = an_in;
	upd7810_init(&uc);
	memset(&s, 0, sizeof(s));
	cchip_reset();
	return 0;
}

void cchip_reset(void)
{
	// (MAME device_start clears the ASIC RAM and selects bank 0; the SRAM keeps its contents)
	upd7810_reset();
}

void cchip_run_clocks(uint64_t *frac, uint32_t rate_mhz, int lines)
{
	// MCU states per scanline: clock / 3 (MAME execute_clocks_to_cycles)
	int n = rbx_line_cycles(frac, s_cfg.clock / 3, rate_mhz, lines);
	upd7810_execute(n);
}

void cchip_ext_interrupt(int state) { upd7810_set_input(UPD7810_INTF1, state); }

uint8_t cchip_mem68_r(uint32_t offset) { return s.sram[s.bank68 * 0x400 + (offset & 0x3ff)]; }
void cchip_mem68_w(uint32_t offset, uint8_t data) { s.sram[s.bank68 * 0x400 + (offset & 0x3ff)] = data; }
uint8_t cchip_asic_r(uint32_t offset) { return asic_r(offset & 0x3ff); }

void cchip_asic68_w(uint32_t offset, uint8_t data)
{
	offset &= 0x3ff;
	if (offset == 0x200)
		s.bank68 = data & 7;
	else
		s.asic_ram[offset & 3] = data;
}

int cchip_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "cchip", &s, sizeof(s));
	rc |= upd7810_state(ctx, load);
	return rc;
}
