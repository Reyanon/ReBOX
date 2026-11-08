/*  The sound board of Irem M72 / M81 / M82 / M84 and M90: a Z80, a YM2151 and a DAC fed by the Z80 from sample ROM (or by
    the protection MCU). Ported from MAME src/mame/irem/m72.cpp, m72_a.cpp, m90.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria, Bryan McPhail, and for rstbuf.cpp: AJR).
    A frame behind on the workers as irem_snd.c: rbx_aux2 runs the Z80, rbx_aux the YM2151's samples.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef IREM_ZSND_H
#define IREM_ZSND_H

#include <stdint.h>

enum { ZP_RTYPE, ZP_M72, ZP_PROT, ZP_RTYPE2, ZP_M99, ZP_DYNA };   // the Z80's port maps
enum { ZN_NONE, ZN_PULSE, ZN_FAKE, ZN_FAKE80, ZN_IRQHOLD };       // the periodic interrupt

typedef struct {
	int ports, nmi;
	int nmi_lines;            // lines between two
	const uint8_t *samples;   // may be NULL
	uint32_t samples_size;
	uint8_t *ram;             // the Z80's 64K of RAM, written by the main CPU; NULL: ROM region "soundcpu" + RAM at f000
	int ym_gain, dac_gain;    // 1/256
	uint32_t frame_mhz;
	int lines;
	void (*mcu_latch)(uint8_t v, int pending);   // ZP_PROT: the MCU latch (at the same line a frame later)
} zsnd_cfg;

int zsnd_init(const zsnd_cfg *cfg);
void zsnd_exit(void);
void zsnd_reset(int held);            // held: the Z80 kept in reset
void zsnd_latch_w(uint8_t v);         // soundlatch
void zsnd_latch2_w(uint8_t v);        // soundlatch2: the Z80's NMI
void zsnd_reset_line(int asserted);
void zsnd_sample_start(uint32_t a);   // set_sample_start
void zsnd_dac_main(int line, uint8_t v);   // the DAC written on the main side (the MCU)
void zsnd_line(int line);
void zsnd_frame(void);
int zsnd_state(void *ctx, int load);

#endif
