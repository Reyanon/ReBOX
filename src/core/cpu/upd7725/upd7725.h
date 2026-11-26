/*  NEC uPD96050 (uPD7725 family) DSP, ported to C from MAME src/devices/cpu/upd7725/upd7725.cpp and upd7725.h
    (BSD-3-Clause, copyright-holders: R. Belmont, byuu; original by byuu in the public domain).
    The uPD96050 only (14-bit program addresses, 2048 words of data RAM), one instruction a clock as MAME counts it.
    The program and data ROMs are big-endian (MAME's ROM_REGION32_BE / ROM_REGION16_BE). No serial port, no DMA,
    no interrupt line (the boards here leave them unconnected).
    Idle loops: a backward jump that comes back to the same place with every register unchanged and nothing written
    in between is a fixed point; the whole iterations left in the slice are skipped (the state they would end in is
    the one it is in), the rest run (exactly MAME's results, at a fraction of the cost).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_UPD7725_H
#define RBX_UPD7725_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	uint8_t s1, s0, c, z, ov1, ov0;
} necdsp_flag;

typedef struct {
	uint16_t pc;
	uint16_t stack[16];
	uint16_t rp, dp;
	uint8_t sp, pad0;
	int16_t k, l, m, n, a, b;
	necdsp_flag flaga, flagb;
	uint16_t tr, trb;
	uint16_t sr;          // (the Status bits as MAME's operator unsigned packs them)
	uint16_t dr, si, so, idb;
	uint8_t siack, soack;
} necdsp_regs;

typedef struct {
	necdsp_regs regs;
	uint16_t dataRAM[2048];
	int irq_firing;
	// (not saved below this) the ROMs
	const uint8_t *prg;   // 4 bytes an instruction, big-endian (the top 24 bits)
	uint32_t prg_words;
	const uint8_t *data;  // 2 bytes a word, big-endian
	uint32_t data_words;
	uint32_t writes;      // (RAM / DR / SR writes: the idle loop check)
	uint16_t idle_pc;     // (the last backward jump's target and the state there)
	int64_t idle_at;
	uint32_t idle_writes;
	necdsp_regs idle_regs;
	int64_t total;        // instructions run
} upd96050_t;
#define UPD96050_STATE_SIZE offsetof(upd96050_t, prg)

void upd96050_init(upd96050_t *d, const uint8_t *prg, uint32_t prg_bytes, const uint8_t *data, uint32_t data_bytes);
void upd96050_reset(upd96050_t *d);
void upd96050_run(upd96050_t *d, int64_t cycles);   // (instructions)
uint8_t upd96050_status_r(upd96050_t *d);
uint8_t upd96050_data_r(upd96050_t *d);
void upd96050_data_w(upd96050_t *d, uint8_t data);
static inline uint16_t upd96050_dataram_r(const upd96050_t *d, uint16_t addr) { return d->dataRAM[addr & 0x07ff]; }
static inline void upd96050_dataram_w(upd96050_t *d, uint16_t addr, uint16_t data, uint16_t mask)
{
	uint16_t *p = &d->dataRAM[addr & 0x07ff];
	*p = (uint16_t)((*p & ~mask) | (data & mask));
	d->writes++;
}

#endif
