#ifndef Z80_Z80_H_
#define Z80_Z80_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct z80 z80;
struct z80 {
  uint8_t (*read_byte)(void*, uint16_t);
  void (*write_byte)(void*, uint16_t, uint8_t);
  uint8_t (*port_in)(z80*, uint8_t);
  void (*port_out)(z80*, uint8_t, uint8_t);
  void* userdata;

  unsigned long cyc; // cycle count (t-states)

  uint16_t pc, sp, ix, iy; // special purpose registers
  uint16_t mem_ptr; // "wz" register (ReBOX: holds the 16-bit port address while port_in runs)
  uint8_t a, b, c, d, e, h, l; // main registers
  uint8_t a_, b_, c_, d_, e_, h_, l_, f_; // alternate registers
  uint8_t i, r; // interrupt vector, memory refresh

  // flags: sign, zero, yf, half-carry, xf, parity/overflow, negative, carry
  // (ReBOX: plain bools, not 1-bit fields: a flag write is a byte store instead of a read-modify-write)
  bool sf, zf, yf, hf, xf, pf, nf, cf;

  uint8_t iff_delay;
  uint8_t interrupt_mode;
  uint8_t int_data;
  bool iff1, iff2;
  bool halted;
  bool int_pending, nmi_pending;

  // ReBOX: optional read map, 256 pages of 256 bytes. A non-NULL page is read directly (no read_byte call):
  // only for memory whose reads have no side effects. The owner restores it after a state load (a pointer).
  const uint8_t* const* read_map;
  // ReBOX: MAME's AS_OPCODES: the opcode fetches (M1, prefixes' second bytes too) from op_rom[pc] (64 KB); NULL: none
  const uint8_t* op_rom;
};

void z80_init(z80* const z);
void z80_step(z80* const z);
void z80_run(z80* const z, unsigned long end, int* const line, int hold);
int z80_halt_skip(z80* const z, unsigned long end, int int_line);
extern int z80_break;   // ReBOX: z80_run ends after the instruction that sets it
void z80_debug_output(z80* const z);
void z80_gen_nmi(z80* const z);
void z80_gen_int(z80* const z, uint8_t data);

#endif // Z80_Z80_H_
