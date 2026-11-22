/*  Idle loops of SH-2 games found at run time and skipped in whole rounds through the core's idle hook (a speed-up
    with the interpreter's exact results, MAME's eat_cycles in effect). The forms (the Psikyo games' waits for the
    vblank interrupt):
      count:  L: MOV.L @Rc,Rx / ADD #1,Rx / MOV.L Rx,@Rc and MOV.L @(disp,PC),Ra / MOV.L @Ra,Ry (in some order) /
              TST Ry,Ry / BT L, Rc a register set before the loop or loaded first in it by MOV.L @(disp,PC),Rc:
              rounds are skipped while the flag at Ra is 0, the counter (and Rx) raised by them
      spin:   L: BRA L / NOP (the work done by the interrupts)
      compare: L: MOV.L @(disp,PC),Ra / MOV.L @(disp,PC),Rb / MOV.L @Ra,Ra / MOV.L @Rb,Rb / CMP/EQ / BT 1f / BRA out /
              NOP / 1: BRA L / NOP (Fever Soccer's vblank wait): rounds are skipped while the two words are equal (the
              round then the same: its loads give the same values), measured only across rounds that saw them equal
    Inside a slice nothing else changes what the loop reads (interrupts come between the driver's slices, the CPU's
    own events end slices), so the rounds left out are those the interpreter would run; a round's cycles are measured
    between two rounds of one slice. Memory read by a loop must be plain pages (host memory both ways).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SH2IDLE_H
#define RBX_SH2IDLE_H

#include "sh2.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t top;
	int kind;            // 0 count, 1 spin, 2 compare (cnt, flag: the two words)
	int rc;              // the counter's register, -1: at cnt
	uint32_t cnt, flag;  // addresses
	int rx;
	int32_t iter, last_icount;
	int64_t last_end;
} sh2_idle_loop;

typedef struct {
	sh2_idle_loop l[4];  // (the core's idle_pc[4])
	int next, off;
	int64_t skipped;     // cycles left out (statistics)
} sh2_idle_t;

// forget the loops (at reset); off: never look for any (a comparison run)
void sh2_idle_reset(sh2_idle_t *s, sh2_t *c, int off);
// look at the code around the CPU's pc (between slices) and keep a loop of one of the forms
void sh2_idle_learn(sh2_idle_t *s, sh2_t *c);

#ifdef __cplusplus
}
#endif

#endif
