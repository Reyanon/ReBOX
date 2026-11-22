/*  NEC uPD7759 ADPCM speech processor (the stand alone mode: its own ROM). Ported from MAME
    src/devices/sound/upd7759.cpp (BSD-3-Clause, copyright-holders: Juergen Buchmueller, Mike Balfour, Howie Cohen,
    Olivier Galibert, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_UPD7759_H
#define RBX_UPD7759_H

#include <stdint.h>

typedef struct {
	// ---- saved ----
	uint32_t pos, step;
	uint8_t fifo_in, reset, start, drq;
	int32_t state, clocks_left;
	uint16_t nibbles_left;
	uint8_t repeat_count;
	int32_t post_drq_state, post_drq_clocks;
	uint8_t req_sample, last_sample, block_header, sample_rate, first_valid_header;
	uint32_t offset, repeat_offset;
	int8_t adpcm_state;
	uint8_t adpcm_data;
	int16_t sample;
	uint8_t state_end;
	// ---- the slave mode (saved apart: upd7759_slave_state) ----
	uint8_t md, slave, pad[2];
	int32_t timer_left;         // (the chip's clocks to its next step; < 0: stopped)
	uint8_t slave_end;
	// ---- the configuration ----
	const uint8_t *rom;
	uint32_t rom_size, clock;
} upd7759_t;

void upd7759_init(upd7759_t *u, const uint8_t *rom, uint32_t rom_size, uint32_t clock, uint32_t rate);
void upd7759_reset(upd7759_t *u);         // (device_reset)
void upd7759_reset_w(upd7759_t *u, int state);
void upd7759_start_w(upd7759_t *u, int state);
void upd7759_port_w(upd7759_t *u, uint8_t data);
int upd7759_busy_r(const upd7759_t *u);   // (/BUSY: 1 when idle)
// n samples at the rate given at init: the held 9-bit sample x 128 (MAME's sample x 128 / 32768 of a 16-bit range)
void upd7759_render(upd7759_t *u, int32_t *out, int n);
int upd7759_state(upd7759_t *u, void *ctx, int load, const char *tag);
// the slave mode (/MD low: the data written to the port at each DRQ, upd7759_device::drq_update): md_w starts it;
// slave_advance runs the chip's clocks and returns how many times DRQ rose (the board's NMI)
void upd7759_md_w(upd7759_t *u, int state);
int upd7759_slave_advance(upd7759_t *u, int clocks);
int upd7759_slave_state(upd7759_t *u, void *ctx, const char *tag);

#endif
