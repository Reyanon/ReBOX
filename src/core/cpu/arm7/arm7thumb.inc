/*  ARM7TDMI Thumb instructions, ported to C from MAME src/devices/cpu/arm7/arm7thmb.cpp (+ arm7help.h flag macros;
    MAME 0.289; BSD-3-Clause, copyright-holders: Steve Ellenoff, R. Belmont, Ryan Holtz).
    Included by arm7.c. One case per MAME tgXX_Y handler (thumb_handler[(op & 0xffc0) >> 6]), same semantics:
    R15 is the address of the executing halfword, the handlers add 2 / 4 themselves; every Thumb instruction costs
    MAME's 3 cycles (the -3 in arm7_run; MAME's thumbCycles table is unused). Flag quirks kept: ADC / SBC flags are
    computed without the carry in, hi-register ADD / MOV / CMP / BX read PC as MAME does (+4 / +2), the ARMv5 BLX forms
    run, POP {PC} stays in Thumb (archRev 4). The encodings MAME stops on with fatalerror (0x4400-0x443f
    tg04_01_00, 0x47c0-0x47ff tg04_01_33, 0xb1xx-0xb3xx, 0xb6xx-0xbbxx, 0xbexx-0xbfxx, 0xdexx) only advance R15 by 2.
    This file is part of ReBOX. GPL-2.0-or-later.
*/

#define THUMB_ADD_FLAGS(rd, rn, op2) \
	set_cpsr(c, (CPSR & ~(N_MASK | Z_MASK | V_MASK | C_MASK)) \
		| ((uint32_t)((!DIFFER(rn, op2)) && DIFFER(rn, rd)) << V_BIT) \
		| ((uint32_t)((~(rn)) < (op2)) << C_BIT) \
		| NZ_FLAGS(rd)); \
	R15 += 2;

#define THUMB_SUB_FLAGS(rd, rn, op2) \
	set_cpsr(c, sub_flags(CPSR, rd, rn, op2)); \
	R15 += 2;

#define SET_NZ(v) set_cpsr(c, (CPSR & ~(Z_MASK | N_MASK)) | NZ_FLAGS(v))
#define SET_C(cond) set_cpsr(c, (cond) ? (CPSR | C_MASK) : (CPSR & ~C_MASK))

// conditional branch condition (tg0d_0 .. tg0d_d)
A7_INLINE int thumb_cond(uint32_t cpsr, uint32_t cond)
{
	const int n = (cpsr & N_MASK) != 0, z = (cpsr & Z_MASK) != 0, cf = (cpsr & C_MASK) != 0, v = (cpsr & V_MASK) != 0;
	switch (cond) {
	case 0x0: return z;
	case 0x1: return !z;
	case 0x2: return cf;
	case 0x3: return !cf;
	case 0x4: return n;
	case 0x5: return !n;
	case 0x6: return v;
	case 0x7: return !v;
	case 0x8: return cf && !z;
	case 0x9: return !cf || z;
	case 0xa: return n == v;
	case 0xb: return n != v;
	case 0xc: return !z && n == v;
	default:  return z || n != v; // 0xd
	}
}

// tg04_00_xx: ALU operations Rd, Rs
static void thumb_alu(arm7_t *c, uint32_t op)
{
	const uint32_t rs = (op >> 3) & 7;
	const uint32_t rd = op & 7;
	switch ((op >> 6) & 0xf) {
	case 0x0: // AND
		c->r[rd] &= c->r[rs];
		SET_NZ(c->r[rd]);
		R15 += 2;
		break;
	case 0x1: // EOR
		c->r[rd] ^= c->r[rs];
		SET_NZ(c->r[rd]);
		R15 += 2;
		break;
	case 0x2: { // LSL Rd, Rs
		const uint32_t rrd = c->r[rd];
		const uint32_t offs = c->r[rs] & 0xff;
		if (offs > 0) {
			if (offs < 32) {
				c->r[rd] = rrd << offs;
				SET_C(rrd & (1u << (31 - (offs - 1))));
			} else if (offs == 32) {
				c->r[rd] = 0;
				SET_C(rrd & 1);
			} else {
				c->r[rd] = 0;
				SET_C(0);
			}
		}
		SET_NZ(c->r[rd]);
		R15 += 2;
		break;
	}
	case 0x3: { // LSR Rd, Rs
		const uint32_t rrd = c->r[rd];
		const uint32_t offs = c->r[rs] & 0xff;
		if (offs > 0) {
			if (offs < 32) {
				c->r[rd] = rrd >> offs;
				SET_C(rrd & (1u << (offs - 1)));
			} else if (offs == 32) {
				c->r[rd] = 0;
				SET_C(rrd & 0x80000000u);
			} else {
				c->r[rd] = 0;
				SET_C(0);
			}
		}
		SET_NZ(c->r[rd]);
		R15 += 2;
		break;
	}
	case 0x4: { // ASR Rd, Rs
		const uint32_t rrs = c->r[rs] & 0xff;
		const uint32_t rrd = c->r[rd];
		if (rrs != 0) {
			if (rrs >= 32) {
				SET_C(rrd >> 31);
				c->r[rd] = (rrd & 0x80000000u) ? 0xffffffffu : 0;
			} else {
				SET_C((rrd >> (rrs - 1)) & 1);
				c->r[rd] = (rrd & 0x80000000u) ? ((0xffffffffu << (32 - rrs)) | (rrd >> rrs)) : (rrd >> rrs);
			}
		}
		SET_NZ(c->r[rd]);
		R15 += 2;
		break;
	}
	case 0x5: { // ADC Rd, Rs (flags without the carry in, as MAME)
		const uint32_t op2 = (CPSR & C_MASK) ? 1 : 0;
		const uint32_t a = c->r[rd], b = c->r[rs];
		const uint32_t rn = a + b + op2;
		THUMB_ADD_FLAGS(rn, a, b);
		c->r[rd] = rn;
		break;
	}
	case 0x6: { // SBC Rd, Rs
		const uint32_t op2 = (CPSR & C_MASK) ? 0 : 1;
		const uint32_t a = c->r[rd], b = c->r[rs];
		const uint32_t rn = a - b - op2;
		THUMB_SUB_FLAGS(rn, a, b);
		c->r[rd] = rn;
		break;
	}
	case 0x7: { // ROR Rd, Rs
		const uint32_t rrd = c->r[rd];
		const uint32_t imm = c->r[rs] & 0xff;
		if (imm > 0) {
			c->r[rd] = rotr32(rrd, imm);
			SET_C(rrd & (1u << ((imm - 1) & 0x1f)));
		}
		SET_NZ(c->r[rd]);
		R15 += 2;
		break;
	}
	case 0x8: // TST
		SET_NZ(c->r[rd] & c->r[rs]);
		R15 += 2;
		break;
	case 0x9: { // NEG Rd, Rs
		const uint32_t rrs = c->r[rs];
		c->r[rd] = 0 - rrs;
		THUMB_SUB_FLAGS(c->r[rd], 0u, rrs);
		break;
	}
	case 0xa: { // CMP Rd, Rs
		const uint32_t a = c->r[rd], b = c->r[rs];
		THUMB_SUB_FLAGS(a - b, a, b);
		break;
	}
	case 0xb: { // CMN Rd, Rs
		const uint32_t a = c->r[rd], b = c->r[rs];
		THUMB_ADD_FLAGS(a + b, a, b);
		break;
	}
	case 0xc: // ORR
		c->r[rd] |= c->r[rs];
		SET_NZ(c->r[rd]);
		R15 += 2;
		break;
	case 0xd: { // MUL
		const uint32_t rn = c->r[rd] * c->r[rs];
		c->r[rd] = rn;
		SET_NZ(rn);
		R15 += 2;
		break;
	}
	case 0xe: // BIC
		c->r[rd] &= ~c->r[rs];
		SET_NZ(c->r[rd]);
		R15 += 2;
		break;
	default: // MVN
		c->r[rd] = ~c->r[rs];
		SET_NZ(c->r[rd]);
		R15 += 2;
		break;
	}
}

// tg04_01_xx: hi register operations / BX
static void thumb_hireg(arm7_t *c, uint32_t op)
{
	const uint32_t rs = (op >> 3) & 7;
	const uint32_t rd = op & 7;
	switch ((op >> 6) & 0xf) {
	case 0x1: // ADD Rd, HRs
		c->r[rd] = c->r[rd] + c->r[rs + 8];
		if (rs == 7) // emulate the effects of pre-fetch
			c->r[rd] = c->r[rd] + 4;
		R15 += 2;
		break;
	case 0x2: // ADD HRd, Rs
		c->r[rd + 8] = c->r[rd + 8] + c->r[rs];
		if (rd == 7)
			R15 += 2;
		R15 += 2;
		break;
	case 0x3: // ADD HRd, HRs
		c->r[rd + 8] = c->r[rd + 8] + c->r[rs + 8];
		if (rs == 7)
			c->r[rd + 8] = c->r[rd + 8] + 4;
		if (rd == 7)
			R15 += 2;
		R15 += 2;
		break;
	case 0x4: { // CMP Rd, Rs
		const uint32_t a = c->r[rd], b = c->r[rs];
		THUMB_SUB_FLAGS(a - b, a, b);
		break;
	}
	case 0x5: { // CMP Rd, Hs
		const uint32_t a = c->r[rd], b = c->r[rs + 8];
		THUMB_SUB_FLAGS(a - b, a, b);
		break;
	}
	case 0x6: { // CMP Hd, Rs
		const uint32_t a = c->r[rd + 8], b = c->r[rs];
		THUMB_SUB_FLAGS(a - b, a, b);
		break;
	}
	case 0x7: { // CMP Hd, Hs
		const uint32_t a = c->r[rd + 8], b = c->r[rs + 8];
		THUMB_SUB_FLAGS(a - b, a, b);
		break;
	}
	case 0x8: // MOV Rd, Rs (undefined form, MAME runs it)
		c->r[rd] = c->r[rs];
		R15 += 2;
		break;
	case 0x9: // MOV Rd, Hs
		c->r[rd] = c->r[rs + 8];
		if (rs == 7)
			c->r[rd] = c->r[rd] + 4;
		R15 += 2;
		break;
	case 0xa: // MOV Hd, Rs
		c->r[rd + 8] = c->r[rs];
		if (rd != 7)
			R15 += 2;
		else
			R15 &= ~1u;
		break;
	case 0xb: // MOV Hd, Hs
		if (rs == 7)
			c->r[rd + 8] = c->r[rs + 8] + 4;
		else
			c->r[rd + 8] = c->r[rs + 8];
		if (rd != 7)
			R15 += 2;
		else
			R15 &= ~1u;
		break;
	case 0xc: { // BX Rs
		uint32_t addr = c->r[rs];
		if (addr & 1) {
			addr &= ~1u;
		} else {
			set_cpsr(c, CPSR & ~T_MASK);
			if (addr & 2)
				addr += 2;
		}
		R15 = addr;
		break;
	}
	case 0xd: { // BX Hs
		uint32_t addr = c->r[rs + 8];
		if (rs == 7)
			addr += 2;
		if (addr & 1) {
			addr &= ~1u;
		} else {
			set_cpsr(c, CPSR & ~T_MASK);
			if (addr & 2)
				addr += 2;
		}
		R15 = addr;
		break;
	}
	case 0xe: { // BLX Rs (ARMv5, MAME runs it)
		uint32_t addr = c->r[rs];
		c->r[14] = (R15 + 2) | 1;
		if (!(addr & 1)) {
			set_cpsr(c, CPSR & ~T_MASK);
			if (addr & 2)
				addr += 2;
		} else {
			addr &= ~1u;
		}
		R15 = addr;
		break;
	}
	default: // 0x0 (G4-1-0), 0xf (G4-3): MAME fatalerror
		R15 += 2;
		break;
	}
}

static void thumb_exec(arm7_t *c, uint32_t op)
{
	switch (op >> 12) {
	case 0x0: { // tg00: LSL / LSR #imm
		const uint32_t rs = (op >> 3) & 7, rd = op & 7;
		const uint32_t rrs = c->r[rs];
		const uint32_t offs = (op >> 6) & 0x1f;
		if (!(op & 0x0800)) {
			// LSL (MAME clears N and Z first; they are set again below)
			set_cpsr(c, CPSR & ~(N_MASK | Z_MASK));
			if (offs != 0) {
				c->r[rd] = rrs << offs;
				SET_C(rrs & (1u << (31 - (offs - 1))));
			} else {
				c->r[rd] = rrs;
			}
		} else {
			// LSR
			if (offs != 0) {
				c->r[rd] = rrs >> offs;
				SET_C(rrs & (1u << (offs - 1)));
			} else {
				c->r[rd] = 0;
				SET_C(rrs & 0x80000000u);
			}
		}
		SET_NZ(c->r[rd]);
		R15 += 2;
		break;
	}
	case 0x1:
		if (!(op & 0x0800)) {
			// tg01_0: ASR #imm
			const uint32_t rs = (op >> 3) & 7, rd = op & 7;
			const uint32_t rrs = c->r[rs];
			uint32_t offs = (op >> 6) & 0x1f;
			if (offs == 0)
				offs = 32;
			if (offs >= 32) {
				SET_C(rrs >> 31);
				c->r[rd] = (rrs & 0x80000000u) ? 0xffffffffu : 0;
			} else {
				SET_C((rrs >> (offs - 1)) & 1);
				c->r[rd] = (rrs & 0x80000000u) ? ((0xffffffffu << (32 - offs)) | (rrs >> offs)) : (rrs >> offs);
			}
			SET_NZ(c->r[rd]);
			R15 += 2;
		} else {
			// tg01_1x: ADD / SUB Rd, Rs, Rn / #imm3
			const uint32_t n = (op >> 6) & 7;
			const uint32_t rs = c->r[(op >> 3) & 7];
			const uint32_t rd = op & 7;
			const uint32_t opnd = (op & 0x0400) ? n : c->r[n];
			if (!(op & 0x0200)) {
				c->r[rd] = rs + opnd;
				THUMB_ADD_FLAGS(c->r[rd], rs, opnd);
			} else {
				c->r[rd] = rs - opnd;
				THUMB_SUB_FLAGS(c->r[rd], rs, opnd);
			}
		}
		break;
	case 0x2: {
		const uint32_t rd = (op >> 8) & 7;
		const uint32_t op2 = op & 0xff;
		if (!(op & 0x0800)) {
			// tg02_0: MOV Rd, #imm
			c->r[rd] = op2;
			SET_NZ(c->r[rd]);
			R15 += 2;
		} else {
			// tg02_1: CMP Rd, #imm
			const uint32_t rn = c->r[rd];
			THUMB_SUB_FLAGS(rn - op2, rn, op2);
		}
		break;
	}
	case 0x3: {
		const uint32_t rdn = (op >> 8) & 7;
		const uint32_t rn = c->r[rdn];
		const uint32_t op2 = op & 0xff;
		if (!(op & 0x0800)) {
			// tg03_0: ADD Rd, #imm
			const uint32_t rd = rn + op2;
			c->r[rdn] = rd;
			THUMB_ADD_FLAGS(rd, rn, op2);
		} else {
			// tg03_1: SUB Rd, #imm
			const uint32_t rd = rn - op2;
			c->r[rdn] = rd;
			THUMB_SUB_FLAGS(rd, rn, op2);
		}
		break;
	}
	case 0x4:
		if (op & 0x0800) {
			// tg04_0203: LDR Rd, [PC, #imm]
			const uint32_t readword = RD32(c, (R15 & ~2u) + 4 + ((op & 0xff) << 2));
			c->r[(op >> 8) & 7] = readword;
			R15 += 2;
		} else if (op & 0x0400) {
			thumb_hireg(c, op);
		} else {
			thumb_alu(c, op);
		}
		break;
	case 0x5: { // tg05: LDR / STR with register offset
		const uint32_t rm = (op >> 6) & 7, rn = (op >> 3) & 7, rd = op & 7;
		const uint32_t addr = c->r[rn] + c->r[rm];
		switch ((op >> 9) & 7) {
		case 0: WR32(c, addr, c->r[rd]); break;                         // STR
		case 1: WR16(c, addr, c->r[rd]); break;                         // STRH
		case 2: WR8(c, addr, c->r[rd]); break;                          // STRB
		case 3: c->r[rd] = (uint32_t)(int32_t)(int8_t)RD8(c, addr); break; // LDRSB
		case 4: c->r[rd] = RD32(c, addr); break;                        // LDR
		case 5: c->r[rd] = RD16(c, addr); break;                        // LDRH
		case 6: c->r[rd] = RD8(c, addr); break;                         // LDRB
		default: {                                                      // LDRSH
			int32_t op2 = (int32_t)(int16_t)(uint16_t)RD16(c, addr & ~1u);
			if ((addr & 1) && !c->arch5)
				op2 >>= 8;
			c->r[rd] = (uint32_t)op2;
			break;
		}
		}
		R15 += 2;
		break;
	}
	case 0x6: { // tg06: STR / LDR Rd, [Rn, #imm5 * 4]
		const uint32_t rn = (op >> 3) & 7, rd = op & 7;
		const uint32_t offs = ((op >> 6) & 0x1f) << 2;
		if (!(op & 0x0800))
			WR32(c, c->r[rn] + offs, c->r[rd]);
		else
			c->r[rd] = RD32(c, c->r[rn] + offs);
		R15 += 2;
		break;
	}
	case 0x7: { // tg07: STRB / LDRB Rd, [Rn, #imm5]
		const uint32_t rn = (op >> 3) & 7, rd = op & 7;
		const uint32_t offs = (op >> 6) & 0x1f;
		if (!(op & 0x0800))
			WR8(c, c->r[rn] + offs, c->r[rd]);
		else
			c->r[rd] = RD8(c, c->r[rn] + offs);
		R15 += 2;
		break;
	}
	case 0x8: { // tg08: STRH / LDRH Rd, [Rs, #imm5 * 2]
		const uint32_t imm = (op >> 6) & 0x1f;
		const uint32_t rs = (op >> 3) & 7, rd = op & 7;
		if (!(op & 0x0800))
			WR16(c, c->r[rs] + (imm << 1), c->r[rd]);
		else
			c->r[rd] = RD16(c, c->r[rs] + (imm << 1));
		R15 += 2;
		break;
	}
	case 0x9: { // tg09: STR / LDR Rd, [SP, #imm8 * 4]
		const uint32_t rd = (op >> 8) & 7;
		const uint32_t offs = op & 0xff;
		if (!(op & 0x0800))
			WR32(c, c->r[13] + (offs << 2), c->r[rd]);
		else
			c->r[rd] = RD32(c, (c->r[13] + (offs << 2)) & ~3u);
		R15 += 2;
		break;
	}
	case 0xa: { // tg0a: ADD Rd, PC / SP, #imm8 * 4
		const uint32_t rd = (op >> 8) & 7;
		const uint32_t offs = (op & 0xff) << 2;
		if (!(op & 0x0800))
			c->r[rd] = ((R15 + 4) & ~2u) + offs;
		else
			c->r[rd] = c->r[13] + offs;
		R15 += 2;
		break;
	}
	case 0xb:
		switch ((op >> 8) & 0xf) {
		case 0x0: { // tg0b_0: ADD SP, #+-imm7 * 4
			const uint32_t addr = op & 0x7f;
			c->r[13] = c->r[13] + ((op & 0x80) ? 0u - (addr << 2) : (addr << 2));
			R15 += 2;
			break;
		}
		case 0x4: // tg0b_4: PUSH {Rlist}
		case 0x5: // tg0b_5: PUSH {Rlist, LR}
			if (op & 0x0100) {
				c->r[13] -= 4;
				WR32(c, c->r[13], c->r[14]);
			}
			for (int offs = 7; offs >= 0; offs--) {
				if (op & (1u << offs)) {
					c->r[13] -= 4;
					WR32(c, c->r[13], c->r[offs]);
				}
			}
			R15 += 2;
			break;
		case 0xc: // tg0b_c: POP {Rlist}
		case 0xd: // tg0b_d: POP {Rlist, PC}
			for (uint32_t offs = 0; offs < 8; offs++) {
				if (op & (1u << offs)) {
					c->r[offs] = RD32(c, c->r[13] & ~3u);
					c->r[13] += 4;
				}
			}
			if (op & 0x0100) {
				uint32_t addr = RD32(c, c->r[13] & ~3u);
				if (!c->arch5)
					R15 = addr & ~1u;   // archRev 4: stays in Thumb
				else {
					// archRev 5: bit 0 clear leaves Thumb (MAME then adds 2 when bit 1 is set)
					if (addr & 1)
						addr &= ~1u;
					else {
						set_cpsr(c, CPSR & ~T_MASK);
						if (addr & 2)
							addr += 2;
					}
					R15 = addr;
				}
				c->r[13] += 4;
			} else {
				R15 += 2;
			}
			break;
		default: // MAME fatalerror
			R15 += 2;
			break;
		}
		break;
	case 0xc: { // tg0c: STMIA / LDMIA Rb!, {Rlist}
		const uint32_t rd = (op >> 8) & 7;
		uint32_t ld_st_address = c->r[rd];
		if (!(op & 0x0800)) {
			for (uint32_t offs = 0; offs < 8; offs++) {
				if (op & (1u << offs)) {
					WR32(c, ld_st_address & ~3u, c->r[offs]);
					ld_st_address += 4;
				}
			}
			c->r[rd] = ld_st_address;
		} else {
			const uint32_t rd_in_list = op & (1u << rd);
			for (uint32_t offs = 0; offs < 8; offs++) {
				if (op & (1u << offs)) {
					c->r[offs] = RD32(c, ld_st_address & ~3u);
					ld_st_address += 4;
				}
			}
			if (!rd_in_list)
				c->r[rd] = ld_st_address;
		}
		R15 += 2;
		break;
	}
	case 0xd: { // tg0d: Bcond / SWI
		const uint32_t cond = (op >> 8) & 0xf;
		if (cond == 0xf) {
			// SWI
			c->pending_swi = 1;
			update_irq_state(c);
			check_irq_state(c);
		} else if (cond == 0xe) {
			// MAME fatalerror (ARM9 reserved)
			R15 += 2;
		} else {
			const int32_t offs = (int8_t)(op & 0xff);
			if (thumb_cond(CPSR, cond))
				R15 += 4 + (uint32_t)(offs * 2);
			else
				R15 += 2;
		}
		break;
	}
	case 0xe:
		if (!(op & 0x0800)) {
			// tg0e_0: B #offs
			uint32_t offs = (op & 0x07ff) << 1;
			if (offs & 0x800)
				offs |= 0xfffff000u;
			R15 += 4 + offs;
		} else {
			// tg0e_1: BLX (LO) (ARMv5, MAME runs it)
			uint32_t addr = c->r[14];
			addr += (op & 0x07ff) << 1;
			addr &= 0xfffffffcu;
			c->r[14] = (R15 + 2) | 1;
			R15 = addr;
			set_cpsr(c, CPSR & ~T_MASK);
		}
		break;
	default:
		if (!(op & 0x0800)) {
			// tg0f_0: BL (HI)
			uint32_t addr = (op & 0x07ff) << 12;
			if (addr & 0x400000)
				addr |= 0xff800000u;
			addr += R15 + 4;
			c->r[14] = addr;
			R15 += 2;
		} else {
			// tg0f_1: BL (LO)
			uint32_t addr = c->r[14] & ~1u;
			addr += (op & 0x07ff) << 1;
			c->r[14] = (R15 + 2) | 1;
			R15 = addr;
		}
		break;
	}
}

#undef THUMB_ADD_FLAGS
#undef THUMB_SUB_FLAGS
#undef SET_NZ
#undef SET_C
