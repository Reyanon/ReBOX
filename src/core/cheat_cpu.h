/*  The CPUs' program spaces for the cheats (cheat.c): each CPU core offers its space when a CPU is set up; the first one
    offered for a game is taken as MAME's "maincpu" (the drivers set their main CPU up first), unless a driver names
    its main CPU's kind with rbx_cheat_prefer. A 68000 on the shared bus (m68k_bus) is the main CPU before any offered one.
    rd / wr: a byte at a cheat address; big: the CPU's byte order (a word's first byte is its high byte); unit: the
    address step of a byte (8 for the TMS34010's bit addresses, 1 for the others).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_CHEAT_CPU_H
#define RBX_CHEAT_CPU_H

#include <stdint.h>

typedef uint8_t (*rbx_cheat_rd)(void *ctx, uint32_t a);
typedef void (*rbx_cheat_wr)(void *ctx, uint32_t a, uint8_t v);
void rbx_cheat_offer(const char *cpu, rbx_cheat_rd rd, rbx_cheat_wr wr, void *ctx, int big, int unit);
// a driver whose first CPU set up is not its main one names the main one's kind ("z80", "nec" ...: the first of that
// kind offered is taken; a 68000 on the shared bus is then not used), before or after setting the CPUs up
void rbx_cheat_prefer(const char *cpu);
void rbx_cheat_prefer_cpu(const void *ctx);   // (the same, by the CPU's context: two CPUs of one kind)
// a driver whose main 68000 has a bus of its own (m68k_bus_select) names it
void rbx_cheat_m68k_bus(void *bus);   // (an m68k_bus_t)
const char *rbx_cheat_cpu_name(void);   // (the main CPU the cheats reach: "68000", "z80" ..., "" none)
uint64_t rbx_cheat_read(uint32_t a, int size);   // (testing: the main CPU's memory as a cheat reads it)
void rbx_cheat_cpu_reset(void);         // (rbx_init, before the driver sets its CPUs up)

#endif
