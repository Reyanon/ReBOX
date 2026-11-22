/*  IDT7200-series FIFO (IDT7201: 512 x 9), ported from MAME 0.289 src/devices/machine/7200fifo.cpp, 7200fifo.h
    (BSD-3-Clause, copyright-holders: hap). The flag lines are read as the pins (active low).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_FIFO7200_H
#define RBX_FIFO7200_H

#include <stdint.h>

#define FIFO7200_MAX 0x400

typedef struct {
	uint16_t buffer[FIFO7200_MAX];
	int32_t read_ptr, write_ptr;
	uint8_t ef, ff, hf;   // empty, full, half-full (1 = set)
	int32_t ram_size;
} fifo7200_t;

void fifo7200_init(fifo7200_t *f, int ram_size);   // 0x200: IDT7201
void fifo7200_reset(fifo7200_t *f);
void fifo7200_write(fifo7200_t *f, uint16_t data);
uint16_t fifo7200_read(fifo7200_t *f);
static inline int fifo7200_ef_r(const fifo7200_t *f) { return !f->ef; }   // _EF
static inline int fifo7200_ff_r(const fifo7200_t *f) { return !f->ff; }   // _FF
static inline int fifo7200_hf_r(const fifo7200_t *f) { return !f->hf; }   // _HF

#endif
