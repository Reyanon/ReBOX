/*  The Midway DCS sound board, rev 1 (ADSP-2105, ROM-based mono: DCS_AUDIO_2K of the T-unit, DCS_AUDIO_8K of the
    Wolf unit), ported to C from MAME src/mame/shared/dcs.cpp, dcs.h (BSD-3-Clause, copyright-holders: Aaron Giles)
    with the DMA DAC of src/devices/sound/dmadac.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    The board's time is in ADSP cycles (10 MHz) from the machine's reset; the output is mono at the DAC's rate.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DCS_H
#define RBX_DCS_H
#include <stdint.h>

enum { DCS_2K, DCS_8K };
#define DCS_CLOCK 10000000u

// the "dcs" region (16-bit words, little-endian)
int dcs_init(int type, uint8_t *rom, uint32_t rom_size);
void dcs_exit(void);
void dcs_reset(void);              // the machine's reset (device_reset; the board's time restarts at 0)
void dcs_reset_w(int state);       // the host's reset line: 0 holds the ADSP and reboots it, 1 runs it
void dcs_data_w(uint16_t data);
uint16_t dcs_data_r(int *empty);   // *empty: the output latch was empty (the host eats 4444 cycles)
int dcs_control_r(void);
// run the board until ADSP cycle `cycles`, its samples put with rbx_audio_put
void dcs_run(uint64_t cycles);
uint32_t dcs_rate(void);
// the board on the worker thread (rbx_aux, same results): set after dcs_init; dcs_frame_end at the frame's end (its
// samples done)
void dcs_set_async(int on);
void dcs_frame_end(void);
int dcs_state(void *ctx, int load);

#endif
