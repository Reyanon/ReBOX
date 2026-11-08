/*  Taito TC0030CMD "C-Chip", ported from MAME src/mame/taito/taitocchip.cpp (BSD-3-Clause, copyright-holders:
    Jonathan Gevaryahu, David Haywood): uPD78C11 MCU (cpu/upd7810) with its 4 KB mask ROM ("cchip:upd7811"),
    the game's 8 KB EPROM ("cchip:cchip_eprom"), 8 KB banked SRAM shared with the 68000 and the ASIC registers.
    The board runs it alongside its main CPU (cchip_run per scanline) and pulses cchip_ext_interrupt at vblank.
    Used by Superman (taito_x), Rainbow Islands, Operation Wolf, Volfied, Bonze Adventure and many F2 games.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TAITOCCHIP_H
#define RBX_TAITOCCHIP_H

#include <stdint.h>

typedef struct {
	uint32_t clock;                  // input clock (12 MHz on the Taito boards); the MCU runs clock / 3 states
	uint8_t (*in_pa)(void);          // NULL = reads 0 (MAME default)
	uint8_t (*in_pb)(void);
	uint8_t (*in_pc)(void);
	uint8_t (*in_ad)(void);          // AN0..AN7 as bits (0xff / 0 per input)
	void (*out_pa)(uint8_t v);
	void (*out_pb)(uint8_t v);
	void (*out_pc)(uint8_t v);
} cchip_config;

int cchip_init(const cchip_config *cfg);   // regions "cchip:upd7811" / "cchip:cchip_eprom"; 0 = ok
void cchip_reset(void);
// run the MCU for clock * num / den input clocks (fractions carried); call once per scanline with num = 1,
// den = lines * frame rate
void cchip_run_clocks(uint64_t *frac, uint32_t rate_mhz, int lines);
void cchip_ext_interrupt(int state);       // INT1 (vblank)

// 68000 side (byte offsets inside the 0x000-0x3ff / 0x400-0x7ff windows, MAME mem68_r / asic_r ...)
uint8_t cchip_mem68_r(uint32_t offset);
void cchip_mem68_w(uint32_t offset, uint8_t data);
uint8_t cchip_asic_r(uint32_t offset);
void cchip_asic68_w(uint32_t offset, uint8_t data);

int cchip_state(void *ctx, int load);

#endif
