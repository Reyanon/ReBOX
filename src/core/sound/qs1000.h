/*  QDSP QS1000 wavetable synthesizer (32 voices, PCM / OKI ADPCM, an 8052 core reading commands from its serial port),
    ported to C from MAME 0.289 src/devices/sound/qs1000.cpp and qs1000.h (BSD-3-Clause, copyright-holders: Philip
    Bennett), the ADPCM decoder from okiadpcm.cpp (BSD-3-Clause, copyright-holders: Andrew Gardner, Aaron Giles).
    As in MAME: the voices play from their sound ROM headers, looping and envelopes are not emulated (MAME's TODO),
    the internal program ROM is not dumped (the boards use an external one). MAME renders at clock / 32 (750 kHz):
    here at clock / 512 (46875 Hz) with the voices stepping 16 of MAME's samples per output sample (point sampled).
    The serial link from the board's sound CPU arrives a byte at a time (qs1000_serial_in).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_QS1000_H
#define RBX_QS1000_H

#include <stdint.h>
#include "../cpu/mcs51/mcs51.h"

#define QS1000_CHANNELS 32
#define QS1000_RATE_DIV 512   // output rate = clock / QS1000_RATE_DIV
#define QS1000_STEP 16        // MAME's samples (clock / 32) per output sample

typedef struct {
	uint32_t acc;
	int32_t adpcm_signal;
	uint32_t start, addr, adpcm_addr, loop_start, loop_end;
	uint16_t freq, flags;
	uint8_t regs[16];
	int32_t adpcm_sig, adpcm_step;   // oki_adpcm_state
} qs1000_channel;

typedef struct {
	/* ---- saved (st and the CPU's state) ---- */
	struct {
		uint8_t wave_regs[18];
		uint8_t pad[2];
		qs1000_channel ch[QS1000_CHANNELS];
		uint8_t ram[256];        // the 8052's external data RAM (0x0000-0x00ff)
	} st;
	mcs51_t cpu;
	/* ---- configuration ---- */
	const uint8_t *rom;          // the sample ROM (24-bit addresses)
	uint32_t rom_size;
	uint32_t clock;
	uint8_t (*p1_in)(void *user);   // NULL: 0
	uint8_t (*p3_in)(void *user);   // NULL: 0
	void *user;
} qs1000_t;

// cpu_rom: the 8052's program (external ROM); sample ROM and its size
void qs1000_init(qs1000_t *q, uint32_t clock, const uint8_t *cpu_rom, uint32_t cpu_rom_size, const uint8_t *rom,
                 uint32_t rom_size);
void qs1000_reset(qs1000_t *q);
// run the 8052 for `cycles` machine cycles (clock / 12)
int qs1000_run(qs1000_t *q, int cycles);
// a byte from the board's sound CPU on the serial line
void qs1000_serial_in(qs1000_t *q, int data);
// MAME set_irq: the 8052's INT1
void qs1000_set_irq(qs1000_t *q, int state);
// mix n output samples (clock / QS1000_RATE_DIV) into l / r (added, 16-bit scale)
void qs1000_render(qs1000_t *q, int32_t *l, int32_t *r, int n);
int qs1000_state(qs1000_t *q, void *ctx, int load);

#endif
