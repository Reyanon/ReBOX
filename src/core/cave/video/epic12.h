/*  Cave CV1000 blitter (EPIC12 / "cv1k_blitter", an Altera Cyclone FPGA), ported to C from MAME 0.289
    src/mame/cave/cv1k_v.cpp, cv1k_v.h, cv1k_v_blit*.cpp, cv1k_v_in.ipp, cv1k_v_pixel.ipp (= MAME 0.260
    src/devices/video/epic12*; BSD-3-Clause, copyright-holders: David Haywood, Luca Elia, MetalliC).
    MAME runs the operation list on a worker thread from a copy of it taken at the exec write. Here the list is read
    at the exec write (its busy time, clips and parameters, the uploaded pixels) and the drawing is queued: drawn in
    two lanes of destination rows by jobs (epic12_drain_lane) on worker threads, or at the write without a queue
    (epic12.c, the drawing queue: how the lanes keep each pixel's order). Only the blitter writes the VRAM, so
    the VRAM is the same at every look as when drawn at the write: the screen copy and the state wait for the queue
    (epic12_sync_screen, epic12_sync). The busy flag is MAME's delay estimate (buffi's research: VRAM / SRAM clocks
    per operation) as a CPU clock stamp, from the list alone.
    VRAM: 8192 x 4096 pixels of 16 bits, ARGB1555 as uploaded (MAME keeps 32-bit pens --t- ---- rrrr r--- gggg g---
    bbbb b---, the same information).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_EPIC12_H
#define RBX_EPIC12_H

#include <stddef.h>
#include <stdint.h>
#include "../../rbx_prof.h"

#define EPIC12_W 0x2000
#define EPIC12_H 0x1000
// the VRAM's rows in memory: EPIC12_STRIDE pixels apart (one 32-byte cache line more than a row: rows 16 KB apart would
// all fall in the same cache sets, ADR-145); EPIC12_PIXELS the buffer (a pixel x, y at y * EPIC12_STRIDE + x)
#define EPIC12_STRIDE (EPIC12_W + 16)

// the sampled activity (rbx_prof.h, ADR-155): the emulation thread's blitter work (the driver names the ids, cv1k.c):
// the operation list read at the exec write, and the queue's syncs (the emulation thread drawing or waiting for the
// lanes: the ring full, the state, the screen)
enum { EPIC12_PA_LIST = RBX_PA_DRV + 1, EPIC12_PA_SYNC = RBX_PA_DRV + 2 };
#define EPIC12_PIXELS ((size_t)EPIC12_STRIDE * EPIC12_H)

typedef struct {
	/* saved */
	uint32_t gfx_addr, scroll_x, scroll_y, clip_x, clip_y;
	uint32_t idle_op_bytes;  // MAME m_blit_idle_op_bytes (kept from one list to the next)
	int64_t busy_until;      // the CPU clock at which the blitter becomes ready (MAME m_blitter_delay_timer)
	/* not saved */
	uint16_t *vram;          // EPIC12_W x EPIC12_H, rows EPIC12_STRIDE apart (EPIC12_PIXELS)
	const uint8_t *ram;      // main RAM, host-order 32-bit words (the SH-3 core's layout)
	uint32_t rammask;
	uint32_t (*dsw)(void *user);
	void *user;
	uint64_t cpu_hz;
	// the drawing queue (epic12_queue; NULL: drawn at the exec write): a ring of 32-bit words written by the emulation
	// thread (qw, published up to qhead; qseq: the entries' numbers) and drawn in two lanes (ADR-120): lane 1 draws the
	// destination rows in the 16-row blocks b with b % split == split - 1, lane 0 the others; each lane passes every
	// entry in order, an entry waits for the other lane where their rows meet (the per-pixel order kept)
	uint32_t *q;
	uint32_t qwords, qw, qhead, qseq;
	uint32_t qlim;     // (ADR-170: qw may go up to here without looking at the lanes again: the slower lane's position
	                   // + qwords when last looked at; the lanes only go on)
	int qold;          // (RBX_CV1K_OLDLIST=1: the list read as before ADR-170)
	uint32_t lpos[2], ldone[2];   // (each lane's read position, words, and the number of the last entry it passed)
	int lhold[2];                 // (who draws the lane now: 0 nobody, 1 + EPIC12_WHO_*)
	int lreq[2], lwant[2];        // (the lane asked for: by the emulation thread / by lane 0's worker; by its own worker)
	int split;
	uint8_t blane[EPIC12_H >> 4];   // (each 16-row block's lane: no division per block, the Cortex-A9 has none)
	uint8_t bend[EPIC12_H >> 4];    // (ADR-170: the last block of the run of blocks of the same lane from each block)
	uint32_t *bw, *br;   // (per 16-row block: the last entry drawing there, the last entry whose other lane read it)
	void (*q_kick)(int lane);
	void (*q_wait)(int lane, int emu);   // (a moment's wait for the lane's progress: on the emulation thread, emu = 1)
	uint32_t st_exec, st_draw, st_up, st_full, st_sync, st_skip, st_solo;   // (counts for RBX_EPIC12_STAT)
	uint32_t st_ent[3][2], st_block[3], st_help[3], st_take;
	uint64_t st_px[3][2];   // (pixels drawn: by who, lane)
} epic12_t;

#define EPIC12_STATE_SIZE ((uint32_t)offsetof(epic12_t, vram))

void epic12_init_tables(void);
void epic12_reset(epic12_t *e);
// the 0x18000000-0x18000057 window: offset = (address - base) / 4, mask = the big-endian lanes; now = CPU clock
uint32_t epic12_r(epic12_t *e, uint32_t offset, uint32_t mask, int64_t now);
void epic12_w(epic12_t *e, uint32_t offset, uint32_t data, uint32_t mask, int64_t now);
// the drawing queued (words: a power of two, at least 0x4000; split: lane 1's share of the rows, 1 in split blocks,
// 0 = none): kick(lane) after queuing (the lane's worker starts on it), wait(lane, emu) a moment while waiting; 0 = ok
int epic12_queue(epic12_t *e, uint32_t words, int split, void (*kick)(int lane), void (*wait)(int lane, int emu));
void epic12_queue_free(epic12_t *e);
enum { EPIC12_WHO_W0, EPIC12_WHO_W1, EPIC12_WHO_EMU };
// a lane's worker (who: EPIC12_WHO_W0 / W1 for lane 0 / 1): draws its lane as far as queued (helping the other lane
// where it waits for it, and when its own is done)
void epic12_drain_lane(epic12_t *e, int lane);
void epic12_sync(epic12_t *e);    // all the queued drawing done
// the queued drawing done as far as the w x h screen at the scroll registers needs it (the rest may still run)
void epic12_sync_screen(epic12_t *e, int w, int h);
// copy the 320 x 240 screen (scrolled, wrapping) as ARGB1555
void epic12_screen(const epic12_t *e, uint16_t *dst, int w, int h);
// the screen copy queued after the drawing so far (ADR-150): copied into dst by lane 0 when it gets there; returns the
// entry's number for epic12_shot_wait (0: copied now, no queue). epic12_shot_wait: until it is copied (emu = 1 on the
// emulation thread: draws there as epic12_sync does; 0 on another thread: waits for lane 0)
uint32_t epic12_queue_shot(epic12_t *e, uint16_t *dst, int w, int h);
void epic12_shot_wait(epic12_t *e, uint32_t k, int emu);

#endif
