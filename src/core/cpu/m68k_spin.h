/*  Polling loops of a 68000 skipped within a slice (hashes unchanged). Within a slice nothing else runs and the
    interrupt lines stay as they are (boards that change them only between slices), so a loop that only reads RAM / ROM
    and writes back what is already there repeats itself exactly: the same instructions, cycles, registers and memory.
    A turn of such a loop, run an instruction at a time from a pc where a slice started, is kept as that pc's template:
    the registers it touches and the words it reads or writes (as they were), its cycles, and the counters it steps
    (registers that only an addq / subq #k,Dn or a dbf changes, their flags never looked at). A later slice that starts
    at that pc with the same registers, status register and words skips as many whole turns as fit (counters advanced
    by as many, kept from wrapping) and runs the rest with Musashi, ending where running every turn ends.
    Templates hold only these instructions, memory operands on mapped RAM / ROM pages of m68k_bus_cur (no device
    behind them) and code on pages nothing writes: nop, tst, tas, btst / bclr / bset / bchg, move to Dn or memory,
    moveq, clr, cmpi, cmp to Dn, andi / ori / eori to Dn, addq / subq to Dn, Bcc / bra, DBcc. Opcodes are read as the
    CPU fetches them (m68k_bus_cur->op: encrypted CPUs); code that reads the same but runs differently (the FD1094's
    state) has its own key.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_M68K_SPIN_H
#define RBX_M68K_SPIN_H

#include <stdint.h>

struct spin_tpl;
typedef struct { struct spin_tpl *t; } m68k_spin_t;   // one per CPU (its templates, allocated on first use)
void m68k_spin_clear(m68k_spin_t *s);   // forget the templates (reset)
void m68k_spin_free(m68k_spin_t *s);    // (exit)
// m68k_execute(budget) for the selected CPU with its polling loops skipped; key tells apart code that reads the
// same at a pc but runs differently (the FD1094's state), 0 otherwise. Returns the cycles run as m68k_execute does
int m68k_spin_execute(m68k_spin_t *s, int budget, uint16_t key);

#endif
