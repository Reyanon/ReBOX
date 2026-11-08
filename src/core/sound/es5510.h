/*  Ensoniq ES5510 (ESP) effects DSP, ported from MAME src/devices/cpu/es5510/es5510.cpp (BSD-3-Clause,
    copyright-holders: Christian Brunschen): host interface, the pipelined ALU / multiplier / delay-line RAM
    execution, and run_once() as the ES5505/5510 pump (sound/esqpump.cpp) uses it once per sample.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ES5510_H
#define RBX_ES5510_H

#include <stdint.h>

#define ES5510_DRAM_SIZE (1 << 20)

typedef struct {
	uint8_t aReg, bReg, src, dst, op, update_ccr, write_result, pad;
	int32_t aValue, bValue, result;
} es5510_alu;

typedef struct {
	uint8_t cReg, dReg, src, dst, accumulate, write_result, pad[2];
	int32_t cValue, dValue;
	int64_t product, result;
} es5510_mac;

typedef struct {
	int32_t address;
	uint8_t io, cycle, pad[2];
} es5510_ram;

typedef struct {
	uint8_t halt_asserted, pc, state, mac_overflow;
	int16_t ser0r, ser0l, ser1r, ser1l, ser2r, ser2l, ser3r, ser3l;
	int64_t machl;
	int16_t dil;
	int8_t memshift, ccr, cmr, pad;
	int32_t memsiz, memmask, memincrement, dlength, abase, bbase, dbase, sigreg, mulshift;
	int16_t dol[2];
	int32_t dol_count;
	int32_t gpr[0xc0];
	uint64_t instr[0xa0];
	int32_t dol_latch, dil_latch, gpr_latch;
	uint32_t dadr_latch;
	uint64_t instr_latch;
	uint8_t ram_sel, host_control, host_serial, pad2;
	es5510_alu alu;
	es5510_mac mulacc;
	es5510_ram ram, ram_p, ram_pp;
	int16_t *dram;                 // ES5510_DRAM_SIZE words (not part of the state struct copy)
} es5510_t;

int es5510_init(es5510_t *e);      // allocates the delay RAM
void es5510_exit(es5510_t *e);
void es5510_reset(es5510_t *e);
uint8_t es5510_host_r(es5510_t *e, int offset);
void es5510_host_w(es5510_t *e, int offset, uint8_t data);
// the general registers (< 0xc0) an instruction word may write: its ALU and MAC result registers, -1 = none
void es5510_instr_dests(uint64_t instr, int *alu_reg, int *mac_reg);
// one sample: serial inputs ser0..ser2 (L, R), run the program once, serial outputs ser3 (L, R)
void es5510_run_once(es5510_t *e);
// the program words were replaced from outside (state load)
void es5510_invalidate(es5510_t *e);
// the compiled program (ARM, rbx_jit memory) into place: the emulation thread, at a frame start
void es5510_jit_install(void);

#endif
