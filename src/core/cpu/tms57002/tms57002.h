/*  Texas Instruments TMS57002 "DASP" (digital audio signal processor), ported to C from MAME
    src/devices/cpu/tms57002/tms57002.cpp, tms57kdec.cpp, tmsops.cpp and the generated tms57002.hxx (MAME 0.289,
    BSD-3-Clause, copyright-holders: Olivier Galibert).
    This file is part of ReBOX. GPL-2.0-or-later.

    One program pass per sample: tms57002_sample() takes the 4 inputs, gives the 4 outputs (those the program wrote
    during the previous pass) and runs the DSP for the cycles of one sample period (MAME: the synchronous stream's
    update then sync_w, the CPU running until its idle instruction).
*/
#ifndef RBX_TMS57002_H
#define RBX_TMS57002_H

#include <stdint.h>

// the decoded-instruction cache (MAME tms57002_device::cd)
enum { TMS57002_IBS = 8192, TMS57002_HBS = 4096 };
typedef struct { uint16_t op; int16_t next; uint8_t param; } tms57002_icd;
typedef struct { uint32_t st1; int16_t ipc; int16_t next; } tms57002_hcd;
typedef struct {
	int16_t hashbase[256];
	tms57002_hcd hashnode[TMS57002_HBS];
	tms57002_icd inst[TMS57002_IBS];
	int hused, iused;
} tms57002_cd;

// the threaded form of cache.inst[] (not saved: rebuilt from it after a state load): the handler's address in the
// interpreter, the next micro-op (next -1: the sentinel, whose handler decodes at pc) and the parameter
typedef struct tms57002_xop {
	const void *lbl;
	const struct tms57002_xop *next;
	uint32_t param;
} tms57002_xop;
typedef struct { const tms57002_xop *ip; uint32_t key; } tms57002_memo;   // (a pc's decoding for st1's key)

// the interpreter's data used at every instruction, first: near the state (the offsets of cmem and dmem0 then
// ARM immediates, 128 and 384 words); not saved
typedef struct {
	uint8_t cm_slow[256];      // (get_cmem's fast test, see tms57002.c)
	uint16_t cm_key;
	int32_t xm_n;              // (during a run: the memory transfer, see tms57002.c)
	uint32_t xm_adr0, xm_tx0, xm_final, xm_mask, xm_shift;
	uint32_t gen;              // the instruction cache's generation (new at each flush and state load: the JIT)
} tms57002_hot;

typedef struct {
	tms57002_hot h;
	uint8_t h_pad[488 - sizeof(tms57002_hot)];
	// (from macc up to `data`: the saved state; the member names are MAME's: tms57002.hxx uses them)
	int64_t macc, macc_read, macc_write;
	uint32_t cmem[256];
	uint32_t dmem0[256];
	uint32_t dmem1[32];
	uint32_t si[4], so[4];
	uint32_t st0, st1, sti;
	uint32_t aacc, xoa, xba, xwr, xrd, txrd, creg;
	uint8_t pc, hpc, ca, id, ba0, ba1, rptc, rptc_next, sa;
	uint32_t xm_adr;
	uint8_t host[4], hidx, allow_update;
	uint32_t update[16];
	uint8_t update_counter_head, update_counter_tail;
	uint8_t reset_held;        // the reset line asserted: the program not run
	int32_t icount;
	uint32_t program[256];     // the internal program RAM (MAME AS_PROGRAM, 256 x 32 bits)
	tms57002_cd cache;
	// the data space (AS_DATA, 8 bits wide, 20-bit addresses): RAM at 0, unmapped above data_size (reads 0)
	uint8_t *data;
	uint32_t data_size;
	tms57002_memo memo[256];
	tms57002_xop xsent, xop[TMS57002_IBS];
	uint8_t xadr[TMS57002_IBS];   // (an end's instruction address)
	uint8_t xspec[TMS57002_IBS];  // (an end's instruction has an op of CINTRPSPECIAL)
} tms57002_t;

// data: the external RAM (MAME's data map), data_size bytes. Then as at power on (device_start + device_reset)
void tms57002_init(tms57002_t *t, uint8_t *data, uint32_t data_size);
void tms57002_reset(tms57002_t *t);                     // device_reset
// the reset input line: asserted holds the DSP; released after being asserted resets it (MAME execute interface)
void tms57002_set_reset_line(tms57002_t *t, int asserted);

// the host interface
uint8_t tms57002_data_r(tms57002_t *t);
void tms57002_data_w(tms57002_t *t, uint8_t data);
void tms57002_pload_w(tms57002_t *t, int state);
void tms57002_cload_w(tms57002_t *t, int state);
int tms57002_dready_r(const tms57002_t *t);
int tms57002_pc0_r(const tms57002_t *t);
int tms57002_empty_r(const tms57002_t *t);

// one sample: in[] the 4 inputs as fractions of full scale x 65536 (MAME's float input x 65536), out[] the 4
// outputs as fractions of full scale x 2^31; then the DSP runs `cycles` cycles (its clock / the sample rate)
void tms57002_sample(tms57002_t *t, const int32_t in[4], int32_t out[4], int cycles);

int tms57002_state(tms57002_t *t, void *ctx, int load);   // through rbx_state_io

// the compiled program (ARM, rbx_jit memory) into place: the emulation thread, at a frame start (core.c)
void tms57002_jit_install(void);

#endif
