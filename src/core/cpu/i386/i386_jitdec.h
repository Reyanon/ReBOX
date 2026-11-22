/*  i386 JIT (ADR-042): an instruction's parts (prefixes, opcode, ModRM, SIB, displacement, immediate) and length, read
    from the code bytes the way the interpreter (i386_decode_opcode and the handlers' FETCH calls) reads them.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_I386_JITDEC_H
#define RBX_I386_JITDEC_H

#include <stdint.h>
#include <string.h>

typedef struct {
	uint8_t len;          // bytes, 0: not decoded (an opcode the JIT leaves to the interpreter's step)
	uint8_t op32, addr32; // operand / address size after the prefixes
	uint8_t seg;          // segment override + 1 (0: none)
	uint8_t rep;          // 0, 0xf2 or 0xf3
	uint8_t lock;
	uint8_t two;          // 1: 0x0f xx
	uint8_t op;           // the opcode byte (after 0x0f when two)
	uint8_t has_modrm, modrm, has_sib, sib;
	uint8_t disp_len, imm_len;
	int32_t disp;
	uint32_t imm;
	uint8_t imm2_len;     // a second immediate (ENTER's byte)
	uint32_t imm2;
} i386_dinsn;

// the bytes from p (at least 16 readable); def32: the code segment's default size (CS.d). Returns the length, 0 when
// the opcode is not one this decoder knows (then nothing is promised about it)
static inline int i386_jit_decode(const uint8_t *p, int def32, i386_dinsn *d)
{
	memset(d, 0, sizeof(*d));
	int n = 0, opsz = def32, adsz = def32;
	for (;;) {
		const uint8_t b = p[n];
		if (b == 0x66) opsz = !def32;
		else if (b == 0x67) adsz = !def32;
		else if (b == 0x26) d->seg = 1;   // ES
		else if (b == 0x2e) d->seg = 2;   // CS
		else if (b == 0x36) d->seg = 3;   // SS
		else if (b == 0x3e) d->seg = 4;   // DS
		else if (b == 0x64) d->seg = 5;   // FS
		else if (b == 0x65) d->seg = 6;   // GS
		else if (b == 0xf0) d->lock = 1;
		else if (b == 0xf2 || b == 0xf3) d->rep = b;
		else break;
		if (++n > 14)
			return 0;
	}
	d->op32 = (uint8_t)opsz;
	d->addr32 = (uint8_t)adsz;
	uint8_t op = p[n++];
	int modrm = 0, imm = 0;   // imm: bytes, or -1 the operand size (2 / 4)
	if (op == 0x0f) {
		d->two = 1;
		op = p[n++];
		d->op = op;
		if ((op & 0xf0) == 0x80) imm = -1;                           // jcc rel16/32
		else if ((op & 0xf0) == 0x90) modrm = 1;                     // setcc
		else if (op == 0xa3 || op == 0xab || op == 0xb3 || op == 0xbb || op == 0xaf || op == 0xbc || op == 0xbd ||
		         op == 0xb6 || op == 0xb7 || op == 0xbe || op == 0xbf || op == 0xa5 || op == 0xad || op == 0xb0 ||
		         op == 0xb1 || op == 0xc0 || op == 0xc1 || op == 0x02 || op == 0x03 || op == 0x00 || op == 0x01) modrm = 1;
		else if (op == 0xa4 || op == 0xac || op == 0xba) { modrm = 1; imm = 1; }
		else if (op == 0xa0 || op == 0xa1 || op == 0xa8 || op == 0xa9 || op == 0xa2 || (op >= 0xc8 && op <= 0xcf) ||
		         op == 0x06 || op == 0x08 || op == 0x09 || op == 0x31) ;
		else
			return 0;
	} else {
		d->op = op;
		if (op < 0x40) {
			if ((op & 7) < 4) modrm = 1;
			else if ((op & 7) == 4) imm = 1;
			else if ((op & 7) == 5) imm = -1;
			// (6, 7: push / pop seg, daa / das / aaa / aas; the prefixes 0x26 ... were taken above)
		} else if (op < 0x60) ;                                       // inc / dec / push / pop r
		else if (op == 0x60 || op == 0x61) ;
		else if (op == 0x62 || op == 0x63) modrm = 1;
		else if (op == 0x68) imm = -1;
		else if (op == 0x69) { modrm = 1; imm = -1; }
		else if (op == 0x6a) imm = 1;
		else if (op == 0x6b) { modrm = 1; imm = 1; }
		else if (op >= 0x6c && op <= 0x6f) ;
		else if (op >= 0x70 && op <= 0x7f) imm = 1;
		else if (op == 0x80 || op == 0x82 || op == 0x83) { modrm = 1; imm = 1; }
		else if (op == 0x81) { modrm = 1; imm = -1; }
		else if (op >= 0x84 && op <= 0x8f) modrm = 1;
		else if (op >= 0x90 && op <= 0x99) ;
		else if (op == 0x9a) { imm = -1; d->imm2_len = 2; }          // call far ptr16:16/32
		else if (op >= 0x9b && op <= 0x9f) ;
		else if (op >= 0xa0 && op <= 0xa3) imm = adsz ? 4 : 2;       // mov moffs
		else if (op >= 0xa4 && op <= 0xa7) ;
		else if (op == 0xa8) imm = 1;
		else if (op == 0xa9) imm = -1;
		else if (op >= 0xaa && op <= 0xaf) ;
		else if (op >= 0xb0 && op <= 0xb7) imm = 1;
		else if (op >= 0xb8 && op <= 0xbf) imm = -1;
		else if (op == 0xc0 || op == 0xc1) { modrm = 1; imm = 1; }
		else if (op == 0xc2 || op == 0xca) imm = 2;
		else if (op == 0xc3 || op == 0xcb || op == 0xcc || op == 0xce || op == 0xcf || op == 0xc9) ;
		else if (op == 0xc4 || op == 0xc5) modrm = 1;
		else if (op == 0xc6) { modrm = 1; imm = 1; }
		else if (op == 0xc7) { modrm = 1; imm = -1; }
		else if (op == 0xc8) { imm = 2; d->imm2_len = 1; }
		else if (op == 0xcd) imm = 1;
		else if (op >= 0xd0 && op <= 0xd3) modrm = 1;
		else if (op == 0xd4 || op == 0xd5) imm = 1;
		else if (op == 0xd6 || op == 0xd7) ;
		else if (op >= 0xd8 && op <= 0xdf) modrm = 1;                // FPU
		else if (op >= 0xe0 && op <= 0xe3) imm = 1;                  // loop / jcxz
		else if (op >= 0xe4 && op <= 0xe7) imm = 1;
		else if (op == 0xe8 || op == 0xe9) imm = -1;
		else if (op == 0xea) { imm = -1; d->imm2_len = 2; }
		else if (op == 0xeb) imm = 1;
		else if (op >= 0xec && op <= 0xef) ;
		else if (op == 0xf4 || op == 0xf5 || (op >= 0xf8 && op <= 0xfd)) ;
		else if (op == 0xf6) { modrm = 1; imm = -2; }                // test r/m8, imm8 for reg 0 / 1
		else if (op == 0xf7) { modrm = 1; imm = -3; }                // test r/m, imm for reg 0 / 1
		else if (op == 0xfe || op == 0xff) modrm = 1;
		else
			return 0;
	}
	if (modrm) {
		const uint8_t m = p[n++];
		d->has_modrm = 1;
		d->modrm = m;
		const int mod = m >> 6, rm = m & 7;
		if (mod != 3) {
			if (adsz) {
				int base = rm;
				if (rm == 4) {
					d->has_sib = 1;
					d->sib = p[n++];
					base = d->sib & 7;
				}
				if (mod == 1) d->disp_len = 1;
				else if (mod == 2) d->disp_len = 4;
				else if (base == 5) d->disp_len = 4;
			} else {
				if (mod == 1) d->disp_len = 1;
				else if (mod == 2) d->disp_len = 2;
				else if (rm == 6) d->disp_len = 2;
			}
		}
		if (d->disp_len == 1) d->disp = (int8_t)p[n];
		else if (d->disp_len == 2) d->disp = (int16_t)(p[n] | p[n + 1] << 8);
		else if (d->disp_len == 4) d->disp = (int32_t)(p[n] | p[n + 1] << 8 | p[n + 2] << 16 | (uint32_t)p[n + 3] << 24);
		n += d->disp_len;
		const int reg = (m >> 3) & 7;
		if (imm == -2) imm = reg < 2 ? 1 : 0;
		else if (imm == -3) imm = reg < 2 ? -1 : 0;
	}
	if (imm == -1) imm = opsz ? 4 : 2;
	d->imm_len = (uint8_t)imm;
	if (imm == 1) d->imm = p[n];
	else if (imm == 2) d->imm = (uint32_t)(p[n] | p[n + 1] << 8);
	else if (imm == 4) d->imm = (uint32_t)(p[n] | p[n + 1] << 8 | p[n + 2] << 16 | (uint32_t)p[n + 3] << 24);
	n += imm;
	if (d->imm2_len == 1) d->imm2 = p[n];
	else if (d->imm2_len == 2) d->imm2 = (uint32_t)(p[n] | p[n + 1] << 8);
	n += d->imm2_len;
	d->len = (uint8_t)n;
	return n;
}

#endif
