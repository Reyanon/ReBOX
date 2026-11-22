/*  ADSP-21xx series emulator (the ADSP-2105 of the Midway DCS sound boards), ported to C from MAME
    src/devices/cpu/adsp2100/adsp2100.cpp, adsp2100.h, 2100ops.hxx (BSD-3-Clause, copyright-holders: Aaron Giles).
    One instance. The address spaces are page tables of 1K words, set by the board.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ADSP2100_H
#define RBX_ADSP2100_H
#include <stdint.h>

// input lines (ADSP-2105)
enum { ADSP2105_IRQ0 = 0, ADSP2105_SPORT1_RX = 0, ADSP2105_IRQ1 = 1, ADSP2105_SPORT1_TX = 1, ADSP2105_IRQ2 = 2,
       ADSP2105_TIMER = 5 };

typedef struct {
	void (*sport_tx)(int port, uint32_t data);   // TX0 (0) / TX1 (1) register writes
	void (*timer_fired)(int state);              // MSTAT timer enable changes
} adsp_callbacks;

void adsp2105_init(const adsp_callbacks *cb);
void adsp_reset(void);                    // device_reset
void adsp_exit(void);                     // frees the predecoded instruction cache
// data space: page = address >> 10 (0-15)
void adsp_map_data_mem(int page, uint16_t *mem, int writable);    // 16-bit words
void adsp_map_data_hi(int page, uint32_t *mem);                   // the upper 16 bits of 24-bit program words
void adsp_map_data_io(int page, uint16_t (*r)(uint32_t a), void (*w)(uint32_t a, uint16_t d));
void adsp_map_data_none(int page);
// program space (24-bit words in uint32)
void adsp_map_prog_mem(int page, uint32_t *mem);
void adsp_map_prog_io(int page, uint32_t (*r)(uint32_t a), void (*w)(uint32_t a, uint32_t d));
void adsp_map_prog_none(int page);
uint16_t adsp_data_read(uint32_t a);
uint32_t adsp_prog_read(uint32_t a);

int adsp_execute(int cycles);             // returns the cycles run
void adsp_abort(void);                    // end the current adsp_execute (a timer set from a handler)
void adsp_skip(int cycles);               // the time of a held CPU
void adsp_state_time_reset(void);        // the time restarts at 0
uint64_t adsp_total_cycles(void);
uint32_t adsp_pc(void);
uint64_t adsp_insns(void);                // (profiling: the instructions executed so far, not the loops skipped)
void adsp_set_input(int line, int state);
uint32_t adsp_get_i(int n);
void adsp_set_i(int n, uint32_t v);       // set_state_int(ADSP2100_I0 + n): with update_i
int32_t adsp_get_m(int n);
uint32_t adsp_get_l(int n);
uint32_t adsp_get_ibase(int n);
void adsp_load_boot_data(const uint8_t *src, uint32_t *dst);
int adsp_state(void *ctx, int load);

#endif
