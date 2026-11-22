/*  The Williams / Midway CVSD and ADPCM sound boards of the Y- and T-unit, ported to C from MAME
    src/mame/shared/williamssound.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    CVSD: 6809E, YM2151, 6821 PIA, MC1408 DAC, HC-55516. ADPCM: 6809E, YM2151, AD7524 DAC, OKI6295 (banked).
    Mono output at the YM2151's rate (3.579545 MHz / 64), the route gains of MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_WILLIAMS_SND_H
#define RBX_WILLIAMS_SND_H
#include <stdint.h>

enum { WMS_SND_CVSD, WMS_SND_ADPCM };

// type, the CPU region ("cvsd:cpu" / "adpcm:cpu"), the OKI region ("adpcm:oki") of the ADPCM board
int wms_snd_init(int type, uint8_t *cpu_rom, uint32_t cpu_size, uint8_t *oki_rom, uint32_t oki_size);
void wms_snd_exit(void);
void wms_snd_reset(void);              // the machine's reset (all the board's chips)
void wms_snd_write(uint16_t data);    // the input latch (CVSD: bits 0-7, CB1 = bit 8, CB2 = bit 9)
void wms_snd_reset_write(int state);  // the reset line: 1 holds the CPU, 0 restarts it
int wms_snd_irq_read(void);           // the ADPCM board's sound IRQ state
int wms_snd_irq_known0(void);         // 1: it is 0 and stays 0 until the next wms_snd_write (no sync needed)
uint32_t wms_snd_rate(void);
// protection RAM of the 6809 [start, end] (install_hidden_ram), or (CVSD small) writes into bank 0 of the ROM
void wms_snd_hidden_ram(uint16_t start, uint16_t end);
void wms_snd_rom_protection(uint16_t start, uint16_t end);
// run the board until `t` seconds * 8 MHz (its time in 8 MHz ticks), its samples put with rbx_audio_put
void wms_snd_run(uint64_t ticks8m);
int wms_snd_state(void *ctx, int load);
// the board runs on the worker thread (rbx_aux, same results): at the frame's end, after the last wms_snd_run (its
// samples done)
void wms_snd_frame_end(void);

#endif
