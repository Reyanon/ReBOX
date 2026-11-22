/*  Texas Instruments TMS34010 graphics processor, ported to C from MAME src/devices/cpu/tms34010 (tms34010.cpp,
    tms34010.h, 34010ops.h, 34010ops.hxx, 34010fld.hxx, 34010gfx.hxx, 34010tbl.hxx; BSD-3-Clause,
    copyright-holders: Alex Pasadyn, Zsolt Vasvari; parts based on code by Aaron Giles).
    The instruction set, the field / pixel / PIXBLT code and the cycle counts are MAME's (the opcode files are converted
    by a script, the 34020 parts left out). One CPU per machine: its state is file-scope. The bus is bit-addressed as
    MAME's (32-bit addresses, 16-bit words): 1 MB-of-bits pages (4096) read / written directly when a page has memory,
    else through the board's handlers. The I/O registers (0xc0000000-0xc00001ff), the display timing (scanline timer,
    display interrupt, DPYADR) and the screen it configures follow MAME's tms340x0_device and screen_device.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TMS34010_H
#define RBX_TMS34010_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t vcount;              // most recent VCOUNT
	uint16_t veblnk, vsblnk;      // start / end of VBLANK
	uint16_t heblnk, hsblnk;      // start / end of HBLANK
	uint16_t rowaddr, coladdr;    // row / column addresses
	uint8_t yoffset;              // y offset from addresses
	uint8_t enabled;              // video enabled
} tms34010_display_params;

typedef struct {
	uint32_t clock;               // input clock (the CPU runs at clock / 8)
	uint32_t pixclock;            // the pixel clock
	int pixperclock;
	// the screen's raw configuration (MAME set_raw: pixclock * pixperclock, htotal, hbend, hbstart, vtotal, vbend, vbstart)
	int htotal, hbend, hbstart, vtotal, vbend, vbstart;
	// the bus outside the pages with memory: a = bit address of the word
	uint16_t (*read)(uint32_t a);
	void (*write)(uint32_t a, uint16_t v);
	// the polling loop skip (may be NULL): 1 when reading word `a` through `read` has no effect and gives the same value
	// while the CPU runs (no other device, no time); `write` calls tms34010_unmapped() when a write had no effect
	int (*pure_read)(uint32_t a);
	void (*to_shiftreg)(uint32_t address, uint16_t *shiftreg);
	void (*from_shiftreg)(uint32_t address, uint16_t *shiftreg);
	// draw screen line `scanline` into dest (the whole screen line, x from 0) from the display parameters
	void (*scanline)(int scanline, const tms34010_display_params *params, uint16_t *dest);
	uint16_t black_pen;           // the blanked parts of a line
	uint16_t *bitmap;             // the visible area is copied here, line by line (pitch in pixels)
	int pitch;
	void (*output_int)(int state);   // may be NULL
} tms34010_config;

void tms34010_init(const tms34010_config *cfg);
void tms34010_release(void);   // at the game's end: the JIT's block table given back (made again at the next run, ADR-200)
// map bit addresses [start, end] (whole 1 MB-of-bits pages) to memory: words little-endian; write NULL = writes go to
// the handler
void tms34010_map(uint32_t start, uint32_t end, uint16_t *read, uint16_t *write);
// pages (1 MB) of mapped program ROM that nothing writes: the JIT makes blocks of their code (ARM hosts)
void tms34010_code_rom(uint32_t start, uint32_t end);
// a write to bit addresses [start, end] of a page with memory is stored, then fn is called (a MAME write tap: umk3's
// palette hack) without taking the whole page off the direct path; one range, fn NULL = none
void tms34010_unmapped(void);
void tms34010_write_watch(uint32_t start, uint32_t end, void (*fn)(void));
void tms34010_reset(void);
// run about `cycles` CPU cycles (clock / 8); returns the cycles run (an instruction can overrun)
int tms34010_execute(int cycles);
// end the current tms34010_execute after this instruction (MAME abort_timeslice: a timer set from a handler)
void tms34010_abort(void);
// MAME eat_cycles: the rest of the timeslice (up to `cycles`) is spent
void tms34010_eat(int cycles);
int tms34010_executing(void);
uint32_t tms34010_pc(void);
uint64_t tms34010_total_cycles(void);
void tms34010_set_irq(int line, int state);   // external interrupt 1 (line 0) / 2 (line 1)
void tms34010_get_display_params(tms34010_display_params *params);
uint16_t tms34010_host_r(int reg);
void tms34010_host_w(int reg, uint16_t data);

// ---- the screen (MAME screen_device with the scanline timer of the TMS34010) ----
typedef struct {
	int width, height;                      // in pixels / lines
	int min_x, max_x, min_y, max_y;         // visible area
	int vpos;                               // the line about to start
} tms34010_screen_t;
extern tms34010_screen_t tms34010_screen;
// the start of line tms34010_screen.vpos: the scanline timer if it is due; returns the line's length in pixels
int tms34010_line_start(void);
// the end of the line: 1 when the next line starts VBLANK (a frame is done: the remaining partial update drawn)
int tms34010_line_end(void);

int tms34010_state(void *ctx, int load);   // rbx_state_io blocks

#ifdef __cplusplus
}
#endif
#endif
