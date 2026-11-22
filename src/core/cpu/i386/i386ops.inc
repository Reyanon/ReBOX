// license:BSD-3-Clause
// copyright-holders:Ville Linde, Barry Rodewald, Carl, Philip Bennett
/*  Converted to C from MAME 0.289 src/devices/cpu/i386/i386ops.hxx (BSD-3-Clause,
    copyright-holders: Ville Linde, Barry Rodewald, Carl, Philip Bennett).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
static uint8_t i386_shift_rotate8(uint8_t modrm, uint32_t value, uint8_t shift)
{
	uint32_t src = value & 0xff;
	uint8_t dst = value;

	if( shift == 0 ) {
		CYCLES_RM(modrm, 3, 7);
	} else if( shift == 1 ) {
		switch( (modrm >> 3) & 0x7 )
		{
			case 0:         /* ROL rm8, 1 */
				I.CF = (src & 0x80) ? 1 : 0;
				dst = (src << 1) + I.CF;
				I.OF = ((src ^ dst) & 0x80) ? 1 : 0;
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 1:         /* ROR rm8, 1 */
				I.CF = (src & 0x1) ? 1 : 0;
				dst = (I.CF << 7) | (src >> 1);
				I.OF = ((src ^ dst) & 0x80) ? 1 : 0;
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 2:         /* RCL rm8, 1 */
				dst = (src << 1) + I.CF;
				I.CF = (src & 0x80) ? 1 : 0;
				I.OF = ((src ^ dst) & 0x80) ? 1 : 0;
				CYCLES_RM(modrm, CYCLES_ROTATE_CARRY_REG, CYCLES_ROTATE_CARRY_MEM);
				break;
			case 3:         /* RCR rm8, 1 */
				dst = (I.CF << 7) | (src >> 1);
				I.CF = src & 0x1;
				I.OF = ((src ^ dst) & 0x80) ? 1 : 0;
				CYCLES_RM(modrm, CYCLES_ROTATE_CARRY_REG, CYCLES_ROTATE_CARRY_MEM);
				break;
			case 4:         /* SHL/SAL rm8, 1 */
			case 6:
				dst = src << 1;
				I.CF = (src & 0x80) ? 1 : 0;
				I.OF = (((I.CF << 7) ^ dst) & 0x80) ? 1 : 0;
				SetSZPF8(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 5:         /* SHR rm8, 1 */
				dst = src >> 1;
				I.CF = src & 0x1;
				I.OF = (dst & 0x80) ? 1 : 0;
				SetSZPF8(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 7:         /* SAR rm8, 1 */
				dst = (int8_t)(src) >> 1;
				I.CF = src & 0x1;
				I.OF = 0;
				SetSZPF8(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
		}

	} else {
		switch( (modrm >> 3) & 0x7 )
		{
			case 0:         /* ROL rm8, i8 */
				if(!(shift & 7))
				{
					if(shift & 0x18)
					{
						I.CF = src & 1;
						I.OF = (src & 1) ^ ((src >> 7) & 1);
					}
					break;
				}
				shift &= 7;
				dst = ((src & ((uint8_t)0xff >> shift)) << shift) |
						((src & ((uint8_t)0xff << (8-shift))) >> (8-shift));
				I.CF = dst & 0x1;
				I.OF = (dst & 1) ^ (dst >> 7);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 1:         /* ROR rm8, i8 */
				if(!(shift & 7))
				{
					if(shift & 0x18)
					{
						I.CF = (src >> 7) & 1;
						I.OF = ((src >> 7) & 1) ^ ((src >> 6) & 1);
					}
					break;
				}
				shift &= 7;
				dst = ((src & ((uint8_t)0xff << shift)) >> shift) |
						((src & ((uint8_t)0xff >> (8-shift))) << (8-shift));
				I.CF = (dst >> 7) & 1;
				I.OF = ((dst >> 7) ^ (dst >> 6)) & 1;
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 2:         /* RCL rm8, i8 */
				shift %= 9;
				dst = ((src & ((uint8_t)0xff >> shift)) << shift) |
						((src & ((uint8_t)0xff << (9-shift))) >> (9-shift)) |
						(I.CF << (shift-1));
				if(shift) I.CF = (src >> (8-shift)) & 0x1;
				I.OF = I.CF ^ ((dst >> 7) & 1);
				CYCLES_RM(modrm, CYCLES_ROTATE_CARRY_REG, CYCLES_ROTATE_CARRY_MEM);
				break;
			case 3:         /* RCR rm8, i8 */
				shift %= 9;
				dst = ((src & ((uint8_t)0xff << shift)) >> shift) |
						((src & ((uint8_t)0xff >> (8-shift))) << (9-shift)) |
						(I.CF << (8-shift));
				if(shift) I.CF = (src >> (shift-1)) & 0x1;
				I.OF = ((dst >> 7) ^ (dst >> 6)) & 1;
				CYCLES_RM(modrm, CYCLES_ROTATE_CARRY_REG, CYCLES_ROTATE_CARRY_MEM);
				break;
			case 4:         /* SHL/SAL rm8, i8 */
			case 6:
				shift &= 31;
				dst = src << shift;
				I.CF = (shift <= 8) && ((src >> (8 - shift)) & 1);
				SetSZPF8(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 5:         /* SHR rm8, i8 */
				shift &= 31;
				dst = src >> shift;
				I.CF = (src & (1 << (shift-1))) ? 1 : 0;
				SetSZPF8(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 7:         /* SAR rm8, i8 */
				shift &= 31;
				dst = (int8_t)src >> shift;
				I.CF = (src & (1 << (shift-1))) ? 1 : 0;
				SetSZPF8(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
		}
	}

	return dst;
}



static void i386_adc_rm8_r8(void)        // Opcode 0x10
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG8(modrm);
		dst = LOAD_RM8(modrm);
		dst = ADC8(dst, src, I.CF);
		STORE_RM8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG8(modrm);
		dst = READ8(ea);
		dst = ADC8(dst, src, I.CF);
		WRITE8(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_adc_r8_rm8(void)        // Opcode 0x12
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM8(modrm);
		dst = LOAD_REG8(modrm);
		dst = ADC8(dst, src, I.CF);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ8(ea);
		dst = LOAD_REG8(modrm);
		dst = ADC8(dst, src, I.CF);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_adc_al_i8(void)     // Opcode 0x14
{
	uint8_t src, dst;
	src = FETCH();
	dst = REG8(AL);
	dst = ADC8(dst, src, I.CF);
	REG8(AL) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_add_rm8_r8(void)        // Opcode 0x00
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG8(modrm);
		dst = LOAD_RM8(modrm);
		dst = ADD8(dst, src);
		STORE_RM8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG8(modrm);
		dst = READ8(ea);
		dst = ADD8(dst, src);
		WRITE8(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_add_r8_rm8(void)        // Opcode 0x02
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM8(modrm);
		dst = LOAD_REG8(modrm);
		dst = ADD8(dst, src);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ8(ea);
		dst = LOAD_REG8(modrm);
		dst = ADD8(dst, src);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_add_al_i8(void)     // Opcode 0x04
{
	uint8_t src, dst;
	src = FETCH();
	dst = REG8(AL);
	dst = ADD8(dst, src);
	REG8(AL) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_and_rm8_r8(void)        // Opcode 0x20
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG8(modrm);
		dst = LOAD_RM8(modrm);
		dst = AND8(dst, src);
		STORE_RM8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG8(modrm);
		dst = READ8(ea);
		dst = AND8(dst, src);
		WRITE8(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_and_r8_rm8(void)        // Opcode 0x22
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM8(modrm);
		dst = LOAD_REG8(modrm);
		dst = AND8(dst, src);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ8(ea);
		dst = LOAD_REG8(modrm);
		dst = AND8(dst, src);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_and_al_i8(void)         // Opcode 0x24
{
	uint8_t src, dst;
	src = FETCH();
	dst = REG8(AL);
	dst = AND8(dst, src);
	REG8(AL) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_clc(void)               // Opcode 0xf8
{
	I.CF = 0;
	CYCLES(CYCLES_CLC);
}

static void i386_cld(void)               // Opcode 0xfc
{
	I.DF = 0;
	CYCLES(CYCLES_CLD);
}

static void i386_cli(void)               // Opcode 0xfa
{
	if(PROTECTED_MODE)
	{
		uint8_t IOPL = I.IOP1 | (I.IOP2 << 1);
		if(I.CPL > IOPL)
			FAULT(FAULT_GP,0);
	}
	I.IF = 0;
	CYCLES(CYCLES_CLI);
}

static void i386_cmc(void)               // Opcode 0xf5
{
	I.CF ^= 1;
	CYCLES(CYCLES_CMC);
}

static void i386_cmp_rm8_r8(void)        // Opcode 0x38
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG8(modrm);
		dst = LOAD_RM8(modrm);
		SUB8(dst, src);
		CYCLES(CYCLES_CMP_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = LOAD_REG8(modrm);
		dst = READ8(ea);
		SUB8(dst, src);
		CYCLES(CYCLES_CMP_REG_MEM);
	}
}

static void i386_cmp_r8_rm8(void)        // Opcode 0x3a
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM8(modrm);
		dst = LOAD_REG8(modrm);
		SUB8(dst, src);
		CYCLES(CYCLES_CMP_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ8(ea);
		dst = LOAD_REG8(modrm);
		SUB8(dst, src);
		CYCLES(CYCLES_CMP_MEM_REG);
	}
}

static void i386_cmp_al_i8(void)         // Opcode 0x3c
{
	uint8_t src, dst;
	src = FETCH();
	dst = REG8(AL);
	SUB8(dst, src);
	CYCLES(CYCLES_CMP_IMM_ACC);
}

static void i386_cmpsb(void)             // Opcode 0xa6
{
	uint32_t eas, ead;
	uint8_t src, dst;
	if( I.segment_prefix ) {
		eas = i386_translate(I.segment_override, I.address_size ? REG32(ESI) : REG16(SI), 0 );
	} else {
		eas = i386_translate(DS, I.address_size ? REG32(ESI) : REG16(SI), 0 );
	}
	ead = i386_translate(ES, I.address_size ? REG32(EDI) : REG16(DI), 0 );
	src = READ8(eas);
	dst = READ8(ead);
	SUB8(src, dst);
	BUMP_SI(1);
	BUMP_DI(1);
	CYCLES(CYCLES_CMPS);
}

static void i386_in_al_i8(void)          // Opcode 0xe4
{
	uint16_t port = FETCH();
	uint8_t data = READPORT8(port);
	REG8(AL) = data;
	CYCLES(CYCLES_IN_VAR);
}

static void i386_in_al_dx(void)          // Opcode 0xec
{
	uint16_t port = REG16(DX);
	uint8_t data = READPORT8(port);
	REG8(AL) = data;
	CYCLES(CYCLES_IN);
}

static void i386_ja_rel8(void)           // Opcode 0x77
{
	int8_t disp = FETCH();
	if( I.CF == 0 && I.ZF == 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jbe_rel8(void)          // Opcode 0x76
{
	int8_t disp = FETCH();
	if( I.CF != 0 || I.ZF != 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jc_rel8(void)           // Opcode 0x72
{
	int8_t disp = FETCH();
	if( I.CF != 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jg_rel8(void)           // Opcode 0x7f
{
	int8_t disp = FETCH();
	if( I.ZF == 0 && (I.SF == I.OF) ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jge_rel8(void)          // Opcode 0x7d
{
	int8_t disp = FETCH();
	if(I.SF == I.OF) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jl_rel8(void)           // Opcode 0x7c
{
	int8_t disp = FETCH();
	if( (I.SF != I.OF) ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jle_rel8(void)      // Opcode 0x7e
{
	int8_t disp = FETCH();
	if( I.ZF != 0 || (I.SF != I.OF) ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jnc_rel8(void)          // Opcode 0x73
{
	int8_t disp = FETCH();
	if( I.CF == 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jno_rel8(void)          // Opcode 0x71
{
	int8_t disp = FETCH();
	if( I.OF == 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jnp_rel8(void)          // Opcode 0x7b
{
	int8_t disp = FETCH();
	if( I.PF == 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jns_rel8(void)          // Opcode 0x79
{
	int8_t disp = FETCH();
	if( I.SF == 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jnz_rel8(void)          // Opcode 0x75
{
	int8_t disp = FETCH();
	if( I.ZF == 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jo_rel8(void)           // Opcode 0x70
{
	int8_t disp = FETCH();
	if( I.OF != 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jp_rel8(void)           // Opcode 0x7a
{
	int8_t disp = FETCH();
	if( I.PF != 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_js_rel8(void)           // Opcode 0x78
{
	int8_t disp = FETCH();
	if( I.SF != 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jz_rel8(void)           // Opcode 0x74
{
	int8_t disp = FETCH();
	if( I.ZF != 0 ) {
		NEAR_BRANCH(disp);
		CYCLES(CYCLES_JCC_DISP8);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_DISP8_NOBRANCH);
	}
}

static void i386_jmp_rel8(void)          // Opcode 0xeb
{
	int8_t disp = FETCH();
	NEAR_BRANCH(disp);
	CYCLES(CYCLES_JMP_SHORT);      /* TODO: Timing = 7 + m */
}

static void i386_lahf(void)              // Opcode 0x9f
{
	REG8(AH) = get_flags() & 0xd7;
	CYCLES(CYCLES_LAHF);
}

static void i386_lodsb(void)             // Opcode 0xac
{
	uint32_t eas;
	if( I.segment_prefix ) {
		eas = i386_translate(I.segment_override, I.address_size ? REG32(ESI) : REG16(SI), 0 );
	} else {
		eas = i386_translate(DS, I.address_size ? REG32(ESI) : REG16(SI), 0 );
	}
	REG8(AL) = READ8(eas);
	BUMP_SI(1);
	CYCLES(CYCLES_LODS);
}

static void i386_mov_rm8_r8(void)        // Opcode 0x88
{
	uint8_t src;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG8(modrm);
		STORE_RM8(modrm, src);
		CYCLES(CYCLES_MOV_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG8(modrm);
		WRITE8(ea, src);
		CYCLES(CYCLES_MOV_REG_MEM);
	}
}

static void i386_mov_r8_rm8(void)        // Opcode 0x8a
{
	uint8_t src;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM8(modrm);
		STORE_REG8(modrm, src);
		CYCLES(CYCLES_MOV_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ8(ea);
		STORE_REG8(modrm, src);
		CYCLES(CYCLES_MOV_MEM_REG);
	}
}

static void i386_mov_rm8_i8(void)        // Opcode 0xc6
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint8_t value = FETCH();
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_MOV_IMM_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		uint8_t value = FETCH();
		WRITE8(ea, value);
		CYCLES(CYCLES_MOV_IMM_MEM);
	}
}

static void i386_mov_r32_cr(void)        // Opcode 0x0f 20
{
	if(PROTECTED_MODE && I.CPL)
		FAULT(FAULT_GP, 0);
	uint8_t modrm = FETCH();
	uint8_t cr = (modrm >> 3) & 0x7;

	STORE_RM32(modrm, I.cr[cr]);
	CYCLES(CYCLES_MOV_CR_REG);
}

static void i386_mov_r32_dr(void)        // Opcode 0x0f 21
{
	if(PROTECTED_MODE && I.CPL)
		FAULT(FAULT_GP, 0);
	uint8_t modrm = FETCH();
	uint8_t dr = (modrm >> 3) & 0x7;

	STORE_RM32(modrm, I.dr[dr]);
	switch(dr)
	{
		case 0:
		case 1:
		case 2:
		case 3:
			CYCLES(CYCLES_MOV_REG_DR0_3);
			break;
		case 6:
		case 7:
			CYCLES(CYCLES_MOV_REG_DR6_7);
			break;
	}
}

static void i386_mov_cr_r32(void)        // Opcode 0x0f 22
{
	if(PROTECTED_MODE && I.CPL)
		FAULT(FAULT_GP, 0);
	uint8_t modrm = FETCH();
	uint8_t cr = (modrm >> 3) & 0x7;
	uint32_t data = LOAD_RM32(modrm);
	switch(cr)
	{
		case 0:
			data &= 0xfffeffff; // wp not supported on 386
			CYCLES(CYCLES_MOV_REG_CR0);
			if (PROTECTED_MODE != BIT(data, 0))
				debugger_privilege_hook();
			break;
		case 2: CYCLES(CYCLES_MOV_REG_CR2); break;
		case 3:
			CYCLES(CYCLES_MOV_REG_CR3);
			vtlb_flush_dynamic();
			break;
		case 4: CYCLES(1); break; // TODO
		default:
			logerror("i386: mov_cr_r32 CR%d!\n", cr);
			return;
	}
	I.cr[cr] = data;
}

static void i386_mov_dr_r32(void)        // Opcode 0x0f 23
{
	if(PROTECTED_MODE && I.CPL)
		FAULT(FAULT_GP, 0);
	uint8_t modrm = FETCH();
	uint8_t dr = (modrm >> 3) & 0x7;

	uint32_t rm32 = LOAD_RM32(modrm);
	switch(dr)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			I.dr[dr] = rm32;
			dri_changed();
			CYCLES(CYCLES_MOV_DR0_3_REG);
			break;
		}
		case 6: CYCLES(CYCLES_MOV_DR6_7_REG); I.dr[dr] = LOAD_RM32(modrm); break;
		case 7:
		{
			uint32_t old_dr7 = I.dr[7];
			I.dr[dr] = rm32;
			dr7_changed(old_dr7, I.dr[7]);
			CYCLES(CYCLES_MOV_DR6_7_REG);
			break;
		}
		default:
			logerror("i386: mov_dr_r32 DR%d!\n", dr);
			return;
	}
}

static void i386_mov_al_m8(void)         // Opcode 0xa0
{
	uint32_t offset, ea;
	if( I.address_size ) {
		offset = FETCH32();
	} else {
		offset = FETCH16();
	}
	/* TODO: Not sure if this is correct... */
	if( I.segment_prefix ) {
		ea = i386_translate(I.segment_override, offset, 0 );
	} else {
		ea = i386_translate(DS, offset, 0 );
	}
	REG8(AL) = READ8(ea);
	CYCLES(CYCLES_MOV_IMM_MEM);
}

static void i386_mov_m8_al(void)         // Opcode 0xa2
{
	uint32_t offset, ea;
	if( I.address_size ) {
		offset = FETCH32();
	} else {
		offset = FETCH16();
	}
	/* TODO: Not sure if this is correct... */
	if( I.segment_prefix ) {
		ea = i386_translate(I.segment_override, offset, 1 );
	} else {
		ea = i386_translate(DS, offset, 1 );
	}
	WRITE8(ea, REG8(AL) );
	CYCLES(CYCLES_MOV_MEM_ACC);
}

static void i386_mov_rm16_sreg(void)     // Opcode 0x8c
{
	uint8_t modrm = FETCH();
	int s = (modrm >> 3) & 0x7;

	if( modrm >= 0xc0 ) {
		if(I.operand_size)
			STORE_RM32(modrm, I.sreg[s].selector);
		else
			STORE_RM16(modrm, I.sreg[s].selector);
		CYCLES(CYCLES_MOV_SREG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE16(ea, I.sreg[s].selector);
		CYCLES(CYCLES_MOV_SREG_MEM);
	}
}

static void i386_mov_sreg_rm16(void)     // Opcode 0x8e
{
	uint16_t selector;
	uint8_t modrm = FETCH();
	bool fault;
	int s = (modrm >> 3) & 0x7;

	if( modrm >= 0xc0 ) {
		selector = LOAD_RM16(modrm);
		CYCLES(CYCLES_MOV_REG_SREG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		selector = READ16(ea);
		CYCLES(CYCLES_MOV_MEM_SREG);
	}

	i386_sreg_load(selector,s,&fault);
	if((s == SS) && !fault)
	{
		if(I.IF != 0) // if external interrupts are enabled
		{
			I.IF = 0;  // reset IF for the next instruction
			I.delayed_interrupt_enable = 1;
		}
	}
}

static void i386_mov_al_i8(void)         // Opcode 0xb0
{
	REG8(AL) = FETCH();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_cl_i8(void)         // Opcode 0xb1
{
	REG8(CL) = FETCH();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_dl_i8(void)         // Opcode 0xb2
{
	REG8(DL) = FETCH();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_bl_i8(void)         // Opcode 0xb3
{
	REG8(BL) = FETCH();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_ah_i8(void)         // Opcode 0xb4
{
	REG8(AH) = FETCH();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_ch_i8(void)         // Opcode 0xb5
{
	REG8(CH) = FETCH();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_dh_i8(void)         // Opcode 0xb6
{
	REG8(DH) = FETCH();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_bh_i8(void)         // Opcode 0xb7
{
	REG8(BH) = FETCH();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_movsb(void)             // Opcode 0xa4
{
	uint32_t eas, ead;
	uint8_t v;
	if( I.segment_prefix ) {
		eas = i386_translate(I.segment_override, I.address_size ? REG32(ESI) : REG16(SI), 0 );
	} else {
		eas = i386_translate(DS, I.address_size ? REG32(ESI) : REG16(SI), 0 );
	}
	ead = i386_translate(ES, I.address_size ? REG32(EDI) : REG16(DI), 1 );
	v = READ8(eas);
	WRITE8(ead, v);
	BUMP_SI(1);
	BUMP_DI(1);
	CYCLES(CYCLES_MOVS);
}

static void i386_or_rm8_r8(void)         // Opcode 0x08
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG8(modrm);
		dst = LOAD_RM8(modrm);
		dst = OR8(dst, src);
		STORE_RM8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG8(modrm);
		dst = READ8(ea);
		dst = OR8(dst, src);
		WRITE8(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_or_r8_rm8(void)         // Opcode 0x0a
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM8(modrm);
		dst = LOAD_REG8(modrm);
		dst = OR8(dst, src);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ8(ea);
		dst = LOAD_REG8(modrm);
		dst = OR8(dst, src);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_or_al_i8(void)          // Opcode 0x0c
{
	uint8_t src, dst;
	src = FETCH();
	dst = REG8(AL);
	dst = OR8(dst, src);
	REG8(EAX) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_out_al_i8(void)         // Opcode 0xe6
{
	uint16_t port = FETCH();
	uint8_t data = REG8(AL);
	WRITEPORT8(port, data);
	CYCLES(CYCLES_OUT_VAR);
}

static void i386_out_al_dx(void)         // Opcode 0xee
{
	uint16_t port = REG16(DX);
	uint8_t data = REG8(AL);
	WRITEPORT8(port, data);
	CYCLES(CYCLES_OUT);
}


static void i386_arpl(void)           // Opcode 0x63
{
	uint16_t src, dst;
	uint8_t modrm = FETCH();
	uint8_t flag = 0;

	if(PROTECTED_MODE && !V8086_MODE)
	{
			if( modrm >= 0xc0 ) {
			src = LOAD_REG16(modrm);
			dst = LOAD_RM16(modrm);
			if( (dst&0x3) < (src&0x3) ) {
				dst = (dst&0xfffc) | (src&0x3);
				flag = 1;
				STORE_RM16(modrm, dst);
			}
		} else {
			uint32_t ea = GetEA(modrm,1);
			src = LOAD_REG16(modrm);
			dst = READ16(ea);
			if( (dst&0x3) < (src&0x3) ) {
				dst = (dst&0xfffc) | (src&0x3);
				flag = 1;
				WRITE16(ea, dst);
			}
		}
		SetZF(flag);
	}
	else
		i386_trap(6, 0);  // invalid opcode in real mode or v8086 mode
}

static void i386_push_i8(void)           // Opcode 0x6a
{
	uint8_t value = FETCH();
	PUSH8(value);
	CYCLES(CYCLES_PUSH_IMM);
}

static void i386_ins_generic(int size)
{
	uint32_t ead;
	uint8_t vb;
	uint16_t vw;
	uint32_t vd;

	ead = i386_translate(ES, I.address_size ? REG32(EDI) : REG16(DI), 1 );

	switch(size) {
	case 1:
		vb = READPORT8(REG16(DX));
		WRITE8(ead, vb);
		break;
	case 2:
		vw = READPORT16(REG16(DX));
		WRITE16(ead, vw);
		break;
	case 4:
		vd = READPORT32(REG16(DX));
		WRITE32(ead, vd);
		break;
	}

	if(I.address_size)
		REG32(EDI) += ((I.DF) ? -1 : 1) * size;
	else
		REG16(DI) += ((I.DF) ? -1 : 1) * size;
	CYCLES(CYCLES_INS);    // TODO: Confirm this value
}

static void i386_insb(void)              // Opcode 0x6c
{
	i386_ins_generic(1);
}

static void i386_insw(void)              // Opcode 0x6d
{
	i386_ins_generic(2);
}

static void i386_insd(void)              // Opcode 0x6d
{
	i386_ins_generic(4);
}

static void i386_outs_generic(int size)
{
	uint32_t eas;
	uint8_t vb;
	uint16_t vw;
	uint32_t vd;

	if( I.segment_prefix ) {
		eas = i386_translate(I.segment_override, I.address_size ? REG32(ESI) : REG16(SI), 0 );
	} else {
		eas = i386_translate(DS, I.address_size ? REG32(ESI) : REG16(SI), 0 );
	}

	switch(size) {
	case 1:
		vb = READ8(eas);
		WRITEPORT8(REG16(DX), vb);
		break;
	case 2:
		vw = READ16(eas);
		WRITEPORT16(REG16(DX), vw);
		break;
	case 4:
		vd = READ32(eas);
		WRITEPORT32(REG16(DX), vd);
		break;
	}

	if(I.address_size)
		REG32(ESI) += ((I.DF) ? -1 : 1) * size;
	else
		REG16(SI) += ((I.DF) ? -1 : 1) * size;
	CYCLES(CYCLES_OUTS);   // TODO: Confirm this value
}

static void i386_outsb(void)             // Opcode 0x6e
{
	i386_outs_generic(1);
}

static void i386_outsw(void)             // Opcode 0x6f
{
	i386_outs_generic(2);
}

static void i386_outsd(void)             // Opcode 0x6f
{
	i386_outs_generic(4);
}

static void i386_repeat(int invert_flag)
{
	uint32_t repeated_eip = I.eip;
	uint32_t repeated_pc = I.pc;
	uint8_t opcode; // = FETCH();
//  uint32_t eas, ead;
	uint32_t count;
	int32_t cycle_base = 0, cycle_adjustment = 0;
	uint8_t prefix_flag=1;
	uint8_t *flag = NULL;


	do {
	repeated_eip = I.eip;
	repeated_pc = I.pc;
	opcode = FETCH();
	switch(opcode) {
		case 0x0f:
		if (invert_flag == 0)
			i386_decode_three_bytef3(); // sse f3 0f
		else
			i386_decode_three_bytef2(); // sse f2 0f
		return;
		case 0x26:
		I.segment_override=ES;
		I.segment_prefix=1;
		break;
		case 0x2e:
		I.segment_override=CS;
		I.segment_prefix=1;
		break;
		case 0x36:
		I.segment_override=SS;
		I.segment_prefix=1;
		break;
		case 0x3e:
		I.segment_override=DS;
		I.segment_prefix=1;
		break;
		case 0x64:
		I.segment_override=FS;
		I.segment_prefix=1;
		break;
		case 0x65:
		I.segment_override=GS;
		I.segment_prefix=1;
		break;
		case 0x66:
		if(!I.operand_prefix)
		{
			I.operand_size ^= 1;
			I.xmm_operand_size ^= 1;
			I.operand_prefix = 1;
		}
		break;
		case 0x67:
		if(!I.address_prefix)
		{
			I.address_size ^= 1;
			I.address_prefix = 1;
		}
		break;
		default:
		prefix_flag=0;
	}
	} while (prefix_flag);


	if( I.segment_prefix ) {
		// FIXME: the following does not work if both address override and segment override are used
		i386_translate(I.segment_override, I.sreg[I.segment_prefix].d ? REG32(ESI) : REG16(SI), -1 );
	} else {
		//eas =
		i386_translate(DS, I.address_size ? REG32(ESI) : REG16(SI), -1 );
	}
	i386_translate(ES, I.address_size ? REG32(EDI) : REG16(DI), -1 );

	switch(opcode)
	{
		case 0x6c:
		case 0x6d:
			/* INSB, INSW, INSD */
			// TODO: cycle count
			cycle_base = 8;
			cycle_adjustment = -4;
			flag = NULL;
			break;

		case 0x6e:
		case 0x6f:
			/* OUTSB, OUTSW, OUTSD */
			// TODO: cycle count
			cycle_base = 8;
			cycle_adjustment = -4;
			flag = NULL;
			break;

		case 0xa4:
		case 0xa5:
			/* MOVSB, MOVSW, MOVSD */
			cycle_base = 8;
			cycle_adjustment = -4;
			flag = NULL;
			break;

		case 0xa6:
		case 0xa7:
			/* CMPSB, CMPSW, CMPSD */
			cycle_base = 5;
			cycle_adjustment = -1;
			flag = &I.ZF;
			break;

		case 0xac:
		case 0xad:
			/* LODSB, LODSW, LODSD */
			cycle_base = 5;
			cycle_adjustment = 1;
			flag = NULL;
			break;

		case 0xaa:
		case 0xab:
			/* STOSB, STOSW, STOSD */
			cycle_base = 5;
			cycle_adjustment = 0;
			flag = NULL;
			break;

		case 0xae:
		case 0xaf:
			/* SCASB, SCASW, SCASD */
			cycle_base = 5;
			cycle_adjustment = 0;
			flag = &I.ZF;
			break;

		case 0x90:
			CYCLES(CYCLES_NOP);
			return;

		case 0xc2: // sigh
		case 0xc3:
			I.pc--;
			return;

		default:
			logerror("i386: Invalid REP/opcode %02X combination at %08x\n",opcode, I.pc - 2);
			I.pc--;
			return;
	}

	if( I.address_size ) {
		if( REG32(ECX) == 0 )
			return;
	} else {
		if( REG16(CX) == 0 )
			return;
	}

	/* now actually perform the repeat */
	CYCLES_NUM(cycle_base);
	do
	{
		I.eip = repeated_eip;
		I.pc = repeated_pc;
		{ I386_TRY_BEGIN
	if (!setjmp(jb_)) {
			i386_decode_opcode();
			s_jmp = prev_;
	} else {
	const uint64_t e = s_exc;
	s_jmp = prev_;
	(void)e;
			I.eip = I.prev_eip;
			i386_throw(e);
		}}

		CYCLES_NUM(cycle_adjustment);

		if (I.address_size)
			count = --REG32(ECX);
		else
			count = --REG16(CX);
		if (count && (I.cycles <= 0))
			goto outofcycles;
	}
	while( count && (!flag || (invert_flag ? !*flag : *flag)) );
	return;

outofcycles:
	/* if we run out of cycles to execute, and we are still in the repeat, we need
	 * to exit this instruction in such a way to go right back into it when we have
	 * time to execute cycles */
	if(flag && (invert_flag ? *flag : !*flag))
		return;
	I.eip = I.prev_eip;
	CHANGE_PC(I.eip);
	CYCLES_NUM(-cycle_base);
}

static void i386_rep(void)               // Opcode 0xf3
{
	i386_repeat(0);
}

static void i386_repne(void)             // Opcode 0xf2
{
	i386_repeat(1);
}

static void i386_sahf(void)              // Opcode 0x9e
{
	set_flags((get_flags() & 0xffffff00) | (REG8(AH) & 0xd7) );
	CYCLES(CYCLES_SAHF);
}

static void i386_sbb_rm8_r8(void)        // Opcode 0x18
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG8(modrm);
		dst = LOAD_RM8(modrm);
		dst = SBB8(dst, src, I.CF);
		STORE_RM8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG8(modrm);
		dst = READ8(ea);
		dst = SBB8(dst, src, I.CF);
		WRITE8(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_sbb_r8_rm8(void)        // Opcode 0x1a
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM8(modrm);
		dst = LOAD_REG8(modrm);
		dst = SBB8(dst, src, I.CF);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ8(ea);
		dst = LOAD_REG8(modrm);
		dst = SBB8(dst, src, I.CF);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_sbb_al_i8(void)         // Opcode 0x1c
{
	uint8_t src, dst;
	src = FETCH();
	dst = REG8(AL);
	dst = SBB8(dst, src, I.CF);
	REG8(EAX) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_scasb(void)             // Opcode 0xae
{
	uint32_t eas;
	uint8_t src, dst;
	eas = i386_translate(ES, I.address_size ? REG32(EDI) : REG16(DI), 0 );
	src = READ8(eas);
	dst = REG8(AL);
	SUB8(dst, src);
	BUMP_DI(1);
	CYCLES(CYCLES_SCAS);
}

static void i386_setalc(void)            // Opcode 0xd6 (undocumented)
{
	if( I.CF ) {
		REG8(AL) = 0xff;
	} else {
		REG8(AL) = 0;
	}
	CYCLES(3);
}

static void i386_seta_rm8(void)          // Opcode 0x0f 97
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.CF == 0 && I.ZF == 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setbe_rm8(void)         // Opcode 0x0f 96
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.CF != 0 || I.ZF != 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setc_rm8(void)          // Opcode 0x0f 92
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.CF != 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setg_rm8(void)          // Opcode 0x0f 9f
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.ZF == 0 && (I.SF == I.OF) ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setge_rm8(void)         // Opcode 0x0f 9d
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if(I.SF == I.OF) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setl_rm8(void)          // Opcode 0x0f 9c
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.SF != I.OF ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setle_rm8(void)         // Opcode 0x0f 9e
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.ZF != 0 || (I.SF != I.OF) ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setnc_rm8(void)         // Opcode 0x0f 93
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.CF == 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setno_rm8(void)         // Opcode 0x0f 91
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.OF == 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setnp_rm8(void)         // Opcode 0x0f 9b
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.PF == 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setns_rm8(void)         // Opcode 0x0f 99
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.SF == 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setnz_rm8(void)         // Opcode 0x0f 95
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.ZF == 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_seto_rm8(void)          // Opcode 0x0f 90
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.OF != 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setp_rm8(void)          // Opcode 0x0f 9a
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.PF != 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_sets_rm8(void)          // Opcode 0x0f 98
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.SF != 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_setz_rm8(void)          // Opcode 0x0f 94
{
	uint8_t modrm = FETCH();
	uint8_t value = 0;
	if( I.ZF != 0 ) {
		value = 1;
	}
	if( modrm >= 0xc0 ) {
		STORE_RM8(modrm, value);
		CYCLES(CYCLES_SETCC_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		WRITE8(ea, value);
		CYCLES(CYCLES_SETCC_MEM);
	}
}

static void i386_stc(void)               // Opcode 0xf9
{
	I.CF = 1;
	CYCLES(CYCLES_STC);
}

static void i386_std(void)               // Opcode 0xfd
{
	I.DF = 1;
	CYCLES(CYCLES_STD);
}

static void i386_sti(void)               // Opcode 0xfb
{
	if(PROTECTED_MODE)
	{
		uint8_t IOPL = I.IOP1 | (I.IOP2 << 1);
		if(I.CPL > IOPL)
			FAULT(FAULT_GP,0);
	}
	I.delayed_interrupt_enable = 1;  // IF is set after the next instruction.
	CYCLES(CYCLES_STI);
}

static void i386_stosb(void)             // Opcode 0xaa
{
	uint32_t ead;
	ead = i386_translate(ES, I.address_size ? REG32(EDI) : REG16(DI), 1 );
	WRITE8(ead, REG8(AL));
	BUMP_DI(1);
	CYCLES(CYCLES_STOS);
}

static void i386_sub_rm8_r8(void)        // Opcode 0x28
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG8(modrm);
		dst = LOAD_RM8(modrm);
		dst = SUB8(dst, src);
		STORE_RM8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG8(modrm);
		dst = READ8(ea);
		dst = SUB8(dst, src);
		WRITE8(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_sub_r8_rm8(void)        // Opcode 0x2a
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM8(modrm);
		dst = LOAD_REG8(modrm);
		dst = SUB8(dst, src);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ8(ea);
		dst = LOAD_REG8(modrm);
		dst = SUB8(dst, src);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_sub_al_i8(void)         // Opcode 0x2c
{
	uint8_t src, dst;
	src = FETCH();
	dst = REG8(EAX);
	dst = SUB8(dst, src);
	REG8(EAX) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_test_al_i8(void)        // Opcode 0xa8
{
	uint8_t src = FETCH();
	uint8_t dst = REG8(AL);
	dst = src & dst;
	SetSZPF8(dst);
	I.CF = 0;
	I.OF = 0;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_test_rm8_r8(void)       // Opcode 0x84
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG8(modrm);
		dst = LOAD_RM8(modrm);
		dst = src & dst;
		SetSZPF8(dst);
		I.CF = 0;
		I.OF = 0;
		CYCLES(CYCLES_TEST_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = LOAD_REG8(modrm);
		dst = READ8(ea);
		dst = src & dst;
		SetSZPF8(dst);
		I.CF = 0;
		I.OF = 0;
		CYCLES(CYCLES_TEST_REG_MEM);
	}
}

static void i386_xchg_r8_rm8(void)       // Opcode 0x86
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint8_t src = LOAD_RM8(modrm);
		uint8_t dst = LOAD_REG8(modrm);
		STORE_REG8(modrm, src);
		STORE_RM8(modrm, dst);
		CYCLES(CYCLES_XCHG_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		uint8_t src = READ8(ea);
		uint8_t dst = LOAD_REG8(modrm);
		WRITE8(ea, dst);
		STORE_REG8(modrm, src);
		CYCLES(CYCLES_XCHG_REG_MEM);
	}
}

static void i386_xor_rm8_r8(void)        // Opcode 0x30
{
	uint8_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG8(modrm);
		dst = LOAD_RM8(modrm);
		dst = XOR8(dst, src);
		STORE_RM8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG8(modrm);
		dst = READ8(ea);
		dst = XOR8(dst, src);
		WRITE8(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_xor_r8_rm8(void)        // Opcode 0x32
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM8(modrm);
		dst = LOAD_REG8(modrm);
		dst = XOR8(dst, src);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ8(ea);
		dst = LOAD_REG8(modrm);
		dst = XOR8(dst, src);
		STORE_REG8(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_xor_al_i8(void)         // Opcode 0x34
{
	uint8_t src, dst;
	src = FETCH();
	dst = REG8(AL);
	dst = XOR8(dst, src);
	REG8(AL) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}



static void i386_group80_8(void)         // Opcode 0x80
{
	uint32_t ea;
	uint8_t src, dst;
	uint8_t modrm = FETCH();

	switch( (modrm >> 3) & 0x7 )
	{
		case 0:     // ADD Rm8, i8
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM8(modrm);
				src = FETCH();
				dst = ADD8(dst, src);
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,0);
				dst = READ8(ea);
				src = FETCH();
				dst = ADD8(dst, src);
				WRITE8(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 1:     // OR Rm8, i8
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM8(modrm);
				src = FETCH();
				dst = OR8(dst, src);
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ8(ea);
				src = FETCH();
				dst = OR8(dst, src);
				WRITE8(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 2:     // ADC Rm8, i8
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM8(modrm);
				src = FETCH();
				dst = ADC8(dst, src, I.CF);
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ8(ea);
				src = FETCH();
				dst = ADC8(dst, src, I.CF);
				WRITE8(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 3:     // SBB Rm8, i8
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM8(modrm);
				src = FETCH();
				dst = SBB8(dst, src, I.CF);
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ8(ea);
				src = FETCH();
				dst = SBB8(dst, src, I.CF);
				WRITE8(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 4:     // AND Rm8, i8
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM8(modrm);
				src = FETCH();
				dst = AND8(dst, src);
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ8(ea);
				src = FETCH();
				dst = AND8(dst, src);
				WRITE8(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 5:     // SUB Rm8, i8
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM8(modrm);
				src = FETCH();
				dst = SUB8(dst, src);
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ8(ea);
				src = FETCH();
				dst = SUB8(dst, src);
				WRITE8(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 6:     // XOR Rm8, i8
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM8(modrm);
				src = FETCH();
				dst = XOR8(dst, src);
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ8(ea);
				src = FETCH();
				dst = XOR8(dst, src);
				WRITE8(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 7:     // CMP Rm8, i8
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM8(modrm);
				src = FETCH();
				SUB8(dst, src);
				CYCLES(CYCLES_CMP_REG_REG);
			} else {
				ea = GetEA(modrm,0);
				dst = READ8(ea);
				src = FETCH();
				SUB8(dst, src);
				CYCLES(CYCLES_CMP_REG_MEM);
			}
			break;
	}
}

static void i386_groupC0_8(void)         // Opcode 0xc0
{
	uint8_t dst;
	uint8_t modrm = FETCH();
	uint8_t shift;

	if( modrm >= 0xc0 ) {
		dst = LOAD_RM8(modrm);
		shift = FETCH() & 0x1f;
		dst = i386_shift_rotate8(modrm, dst, shift);
		STORE_RM8(modrm, dst);
	} else {
		uint32_t ea = GetEA(modrm,1);
		dst = READ8(ea);
		shift = FETCH() & 0x1f;
		dst = i386_shift_rotate8(modrm, dst, shift);
		WRITE8(ea, dst);
	}
}

static void i386_groupD0_8(void)         // Opcode 0xd0
{
	uint8_t dst;
	uint8_t modrm = FETCH();

	if( modrm >= 0xc0 ) {
		dst = LOAD_RM8(modrm);
		dst = i386_shift_rotate8(modrm, dst, 1);
		STORE_RM8(modrm, dst);
	} else {
		uint32_t ea = GetEA(modrm,1);
		dst = READ8(ea);
		dst = i386_shift_rotate8(modrm, dst, 1);
		WRITE8(ea, dst);
	}
}

static void i386_groupD2_8(void)         // Opcode 0xd2
{
	uint8_t dst;
	uint8_t modrm = FETCH();

	if( modrm >= 0xc0 ) {
		dst = LOAD_RM8(modrm);
		dst = i386_shift_rotate8(modrm, dst, REG8(CL));
		STORE_RM8(modrm, dst);
	} else {
		uint32_t ea = GetEA(modrm,1);
		dst = READ8(ea);
		dst = i386_shift_rotate8(modrm, dst, REG8(CL));
		WRITE8(ea, dst);
	}
}

static void i386_groupF6_8(void)         // Opcode 0xf6
{
	uint8_t modrm = FETCH();

	switch( (modrm >> 3) & 0x7 )
	{
		case 0:         /* TEST Rm8, i8 */
			if( modrm >= 0xc0 ) {
				uint8_t dst = LOAD_RM8(modrm);
				uint8_t src = FETCH();
				dst &= src;
				I.CF = I.OF = I.AF = 0;
				SetSZPF8(dst);
				CYCLES(CYCLES_TEST_IMM_REG);
			} else {
				uint32_t ea = GetEA(modrm,0);
				uint8_t dst = READ8(ea);
				uint8_t src = FETCH();
				dst &= src;
				I.CF = I.OF = I.AF = 0;
				SetSZPF8(dst);
				CYCLES(CYCLES_TEST_IMM_MEM);
			}
			break;
		case 2:         /* NOT Rm8 */
			if( modrm >= 0xc0 ) {
				uint8_t dst = LOAD_RM8(modrm);
				dst = ~dst;
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_NOT_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint8_t dst = READ8(ea);
				dst = ~dst;
				WRITE8(ea, dst);
				CYCLES(CYCLES_NOT_MEM);
			}
			break;
		case 3:         /* NEG Rm8 */
			if( modrm >= 0xc0 ) {
				uint8_t dst = LOAD_RM8(modrm);
				dst = SUB8(0, dst );
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_NEG_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint8_t dst = READ8(ea);
				dst = SUB8(0, dst );
				WRITE8(ea, dst);
				CYCLES(CYCLES_NEG_MEM);
			}
			break;
		case 4:         /* MUL AL, Rm8 */
			{
				uint16_t result;
				uint8_t src, dst;
				if( modrm >= 0xc0 ) {
					src = LOAD_RM8(modrm);
					CYCLES(CYCLES_MUL8_ACC_REG);       /* TODO: Correct multiply timing */
				} else {
					uint32_t ea = GetEA(modrm,0);
					src = READ8(ea);
					CYCLES(CYCLES_MUL8_ACC_MEM);       /* TODO: Correct multiply timing */
				}

				dst = REG8(AL);
				result = (uint16_t)src * (uint16_t)dst;
				REG16(AX) = (uint16_t)result;

				I.CF = I.OF = (REG16(AX) > 0xff);
			}
			break;
		case 5:         /* IMUL AL, Rm8 */
			{
				int16_t result;
				int16_t src, dst;
				if( modrm >= 0xc0 ) {
					src = (int16_t)(int8_t)LOAD_RM8(modrm);
					CYCLES(CYCLES_IMUL8_ACC_REG);      /* TODO: Correct multiply timing */
				} else {
					uint32_t ea = GetEA(modrm,0);
					src = (int16_t)(int8_t)READ8(ea);
					CYCLES(CYCLES_IMUL8_ACC_MEM);      /* TODO: Correct multiply timing */
				}

				dst = (int16_t)(int8_t)REG8(AL);
				result = src * dst;

				REG16(AX) = (uint16_t)result;

				I.CF = I.OF = !(result == (int16_t)(int8_t)result);
			}
			break;
		case 6:         /* DIV AL, Rm8 */
			{
				uint16_t quotient, remainder, result;
				uint8_t src;
				if( modrm >= 0xc0 ) {
					src = LOAD_RM8(modrm);
					CYCLES(CYCLES_DIV8_ACC_REG);
				} else {
					uint32_t ea = GetEA(modrm,0);
					src = READ8(ea);
					CYCLES(CYCLES_DIV8_ACC_MEM);
				}

				quotient = (uint16_t)REG16(AX);
				if( src ) {
					remainder = quotient % (uint16_t)src;
					result = quotient / (uint16_t)src;
					if( result > 0xff ) {
						/* TODO: Divide error */
					} else {
						REG8(AH) = (uint8_t)remainder & 0xff;
						REG8(AL) = (uint8_t)result & 0xff;

						// this flag is actually undefined, enable on non-cyrix
						if (I.cpuid_id0 != 0x69727943)
							I.CF = 1;
					}
				} else {
					i386_trap(0, 0);
				}
			}
			break;
		case 7:         /* IDIV AL, Rm8 */
			{
				int16_t quotient, remainder, result;
				uint8_t src;
				if( modrm >= 0xc0 ) {
					src = LOAD_RM8(modrm);
					CYCLES(CYCLES_IDIV8_ACC_REG);
				} else {
					uint32_t ea = GetEA(modrm,0);
					src = READ8(ea);
					CYCLES(CYCLES_IDIV8_ACC_MEM);
				}

				quotient = (int16_t)REG16(AX);
				if( src ) {
					remainder = quotient % (int16_t)(int8_t)src;
					result = quotient / (int16_t)(int8_t)src;
					if( result > 0x7f || result < -0x80 ) {
						/* TODO: Divide error */
					} else {
						REG8(AH) = (uint8_t)remainder & 0xff;
						REG8(AL) = (uint8_t)result & 0xff;

						// this flag is actually undefined, enable on non-cyrix
						if (I.cpuid_id0 != 0x69727943)
							I.CF = 1;
					}
				} else {
					i386_trap(0, 0);
				}
			}
			break;
	}
}

static void i386_groupFE_8(void)         // Opcode 0xfe
{
	uint8_t modrm = FETCH();

	switch( (modrm >> 3) & 0x7 )
	{
		case 0:         /* INC Rm8 */
			if( modrm >= 0xc0 ) {
				uint8_t dst = LOAD_RM8(modrm);
				dst = INC8(dst);
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_INC_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint8_t dst = READ8(ea);
				dst = INC8(dst);
				WRITE8(ea, dst);
				CYCLES(CYCLES_INC_MEM);
			}
			break;
		case 1:         /* DEC Rm8 */
			if( modrm >= 0xc0 ) {
				uint8_t dst = LOAD_RM8(modrm);
				dst = DEC8(dst);
				STORE_RM8(modrm, dst);
				CYCLES(CYCLES_DEC_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint8_t dst = READ8(ea);
				dst = DEC8(dst);
				WRITE8(ea, dst);
				CYCLES(CYCLES_DEC_MEM);
			}
			break;
		case 6:         /* PUSH Rm8*/
			{
				uint8_t value;
				if( modrm >= 0xc0 ) {
					value = LOAD_RM8(modrm);
				} else {
					uint32_t ea = GetEA(modrm,0);
					value = READ8(ea);
				}
				if( I.operand_size ) {
					PUSH32(value);
				} else {
					PUSH16(value);
				}
				CYCLES(CYCLES_PUSH_RM);
			}
			break;
		default:
			report_invalid_modrm("groupFE_8", modrm);
			break;
	}
}



static void i386_segment_CS(void)        // Opcode 0x2e
{
	I.segment_prefix = 1;
	I.segment_override = CS;

	i386_decode_opcode();
}

static void i386_segment_DS(void)        // Opcode 0x3e
{
	I.segment_prefix = 1;
	I.segment_override = DS;
	CYCLES(0); // TODO: Specify cycle count
	i386_decode_opcode();
}

static void i386_segment_ES(void)        // Opcode 0x26
{
	I.segment_prefix = 1;
	I.segment_override = ES;
	CYCLES(0); // TODO: Specify cycle count
	i386_decode_opcode();
}

static void i386_segment_FS(void)        // Opcode 0x64
{
	I.segment_prefix = 1;
	I.segment_override = FS;
	CYCLES(1); // TODO: Specify cycle count
	i386_decode_opcode();
}

static void i386_segment_GS(void)        // Opcode 0x65
{
	I.segment_prefix = 1;
	I.segment_override = GS;
	CYCLES(1); // TODO: Specify cycle count
	i386_decode_opcode();
}

static void i386_segment_SS(void)        // Opcode 0x36
{
	I.segment_prefix = 1;
	I.segment_override = SS;
	CYCLES(0); // TODO: Specify cycle count
	i386_decode_opcode();
}

static void i386_operand_size(void)      // Opcode prefix 0x66
{
	if(I.operand_prefix == 0)
	{
		I.operand_size ^= 1;
		I.xmm_operand_size ^= 1;
		I.operand_prefix = 1;
	}
	I.opcode = FETCH();
	if (I.opcode == 0x0f)
		i386_decode_three_byte66();
	else
	{
		if( I.operand_size )
			I.opcode_table1_32[I.opcode]();
		else
			I.opcode_table1_16[I.opcode]();
	}
}

static void i386_address_size(void)      // Opcode 0x67
{
	if(I.address_prefix == 0)
	{
		I.address_size ^= 1;
		I.address_prefix = 1;
	}
	i386_decode_opcode();
}

static void i386_nop(void)               // Opcode 0x90
{
	CYCLES(CYCLES_NOP);
}

static void i386_int3(void)              // Opcode 0xcc
{
	CYCLES(CYCLES_INT3);
	I.ext = 0; // not an external interrupt
	i386_trap(3, 1);
	I.ext = 1;
}

static void i386_int(void)               // Opcode 0xcd
{
	int interrupt = FETCH();
	CYCLES(CYCLES_INT);
	I.ext = 0; // not an external interrupt
	i386_trap(interrupt, 1);
	I.ext = 1;
}

static void i386_into(void)              // Opcode 0xce
{
	if( I.OF ) {
		I.ext = 0;
		i386_trap(4, 1);
		I.ext = 1;
		CYCLES(CYCLES_INTO_OF1);
	}
	else
	{
		CYCLES(CYCLES_INTO_OF0);
	}
}

static uint32_t i386_escape_ea;   // hack around GCC 4.6 error because we need the side effects of GetEA()
static void i386_escape(void)            // Opcodes 0xd8 - 0xdf
{
	uint8_t modrm = FETCH();
	if(modrm < 0xc0)
	{
		i386_escape_ea = GetEA(modrm,0);
	}
	CYCLES(3); // TODO: confirm this
	(void) LOAD_RM8(modrm);
}

static void i386_hlt(void)               // Opcode 0xf4
{
	if(PROTECTED_MODE && I.CPL != 0)
		FAULT(FAULT_GP,0);
	I.halted = 1;
	CYCLES(CYCLES_HLT);
	if (I.cycles > 0)
		I.cycles = 0;
}

static void i386_decimal_adjust(int direction)
{
	uint8_t tmpAL = REG8(AL);
	uint8_t tmpCF = I.CF;

	if (I.AF || ((REG8(AL) & 0xf) > 9))
	{
		uint16_t t= (uint16_t)REG8(AL) + (direction * 0x06);
		REG8(AL) = (uint8_t)t&0xff;
		I.AF = 1;
		if (t & 0x100)
			I.CF = 1;
		if (direction > 0)
			tmpAL = REG8(AL);
	}

	if (tmpCF || (tmpAL > 0x99))
	{
		REG8(AL) += (direction * 0x60);
		I.CF = 1;
	}

	SetSZPF8(REG8(AL));
}

static void i386_daa(void)               // Opcode 0x27
{
	i386_decimal_adjust(+1);
	CYCLES(CYCLES_DAA);
}

static void i386_das(void)               // Opcode 0x2f
{
	i386_decimal_adjust(-1);
	CYCLES(CYCLES_DAS);
}

static void i386_aaa(void)               // Opcode 0x37
{
	if( ( (REG8(AL) & 0x0f) > 9) || (I.AF != 0) ) {
		REG16(AX) = REG16(AX) + 6;
		REG8(AH) = REG8(AH) + 1;
		I.AF = 1;
		I.CF = 1;
	} else {
		I.AF = 0;
		I.CF = 0;
	}
	REG8(AL) = REG8(AL) & 0x0f;
	CYCLES(CYCLES_AAA);
}

static void i386_aas(void)               // Opcode 0x3f
{
	if (I.AF || ((REG8(AL) & 0xf) > 9))
	{
		REG16(AX) -= 6;
		REG8(AH) -= 1;
		I.AF = 1;
		I.CF = 1;
	}
	else
	{
		I.AF = 0;
		I.CF = 0;
	}
	REG8(AL) &= 0x0f;
	CYCLES(CYCLES_AAS);
}

static void i386_aad(void)               // Opcode 0xd5
{
	uint8_t tempAL = REG8(AL);
	uint8_t tempAH = REG8(AH);
	uint8_t i = FETCH();

	REG8(AL) = (tempAL + (tempAH * i)) & 0xff;
	REG8(AH) = 0;
	SetSZPF8( REG8(AL) );
	CYCLES(CYCLES_AAD);
}

static void i386_aam(void)               // Opcode 0xd4
{
	uint8_t tempAL = REG8(AL);
	uint8_t i = FETCH();

	if(!i)
	{
		i386_trap(0, 0);
		return;
	}
	REG8(AH) = tempAL / i;
	REG8(AL) = tempAL % i;
	SetSZPF8( REG8(AL) );
	CYCLES(CYCLES_AAM);
}

static void i386_clts(void)              // Opcode 0x0f 0x06
{
	// Privileged instruction, CPL must be zero.  Can be used in real or v86 mode.
	if(PROTECTED_MODE && I.CPL != 0)
		FAULT(FAULT_GP,0)
	I.cr[0] &= ~CR0_TS;   /* clear TS bit */
	CYCLES(CYCLES_CLTS);
}

static void i386_wait(void)              // Opcode 0x9B
{
	if ((I.cr[0] & (CR0_TS | CR0_MP)) == (CR0_TS | CR0_MP))
	{
		i386_trap(FAULT_NM, 0);
		return;
	}
	// TODO
}

static void i386_lock(void)              // Opcode 0xf0
{
	// lock doesn't depend on iopl on 386
	I.lock = true;
	CYCLES(CYCLES_LOCK);       // TODO: Determine correct cycle count
	i386_decode_opcode();
}

static void i386_mov_r32_tr(void)        // Opcode 0x0f 24
{
	FETCH();
	CYCLES(1);     // TODO: correct cycle count
}

static void i386_mov_tr_r32(void)        // Opcode 0x0f 26
{
	FETCH();
	CYCLES(1);     // TODO: correct cycle count
}

static void i386_loadall(void)       // Opcode 0x0f 0x07 (0x0f 0x05 on 80286), undocumented
{
	if(PROTECTED_MODE && (I.CPL != 0))
		FAULT(FAULT_GP,0)
	uint32_t ea = i386_translate(ES, REG32(EDI), 0);
	uint32_t old_dr7 = I.dr[7];
	I.cr[0] = READ32(ea) & 0xfffeffff; // wp not supported on 386
	set_flags(READ32(ea + 0x04));
	I.eip = READ32(ea + 0x08);
	REG32(EDI) = READ32(ea + 0x0c);
	REG32(ESI) = READ32(ea + 0x10);
	REG32(EBP) = READ32(ea + 0x14);
	REG32(ESP) = READ32(ea + 0x18);
	REG32(EBX) = READ32(ea + 0x1c);
	REG32(EDX) = READ32(ea + 0x20);
	REG32(ECX) = READ32(ea + 0x24);
	REG32(EAX) = READ32(ea + 0x28);
	I.dr[6] = READ32(ea + 0x2c);
	I.dr[7] = READ32(ea + 0x30);
	I.task.segment = READ16(ea + 0x34);
	I.ldtr.segment = READ16(ea + 0x38);
	I.sreg[GS].selector = READ16(ea + 0x3c);
	I.sreg[FS].selector = READ16(ea + 0x40);
	I.sreg[DS].selector = READ16(ea + 0x44);
	I.sreg[SS].selector = READ16(ea + 0x48);
	I.sreg[CS].selector = READ16(ea + 0x4c);
	I.sreg[ES].selector = READ16(ea + 0x50);
	I.task.flags = READ32(ea + 0x54) >> 8;
	I.task.base = READ32(ea + 0x58);
	I.task.limit = READ32(ea + 0x5c);
	I.idtr.base = READ32(ea + 0x64);
	I.idtr.limit = READ32(ea + 0x68);
	I.gdtr.base = READ32(ea + 0x70);
	I.gdtr.limit = READ32(ea + 0x74);
	I.ldtr.flags = READ32(ea + 0x78) >> 8;
	I.ldtr.base = READ32(ea + 0x7c);
	I.ldtr.limit = READ32(ea + 0x80);
	I.sreg[GS].flags = READ32(ea + 0x84) >> 8;
	I.sreg[GS].base = READ32(ea + 0x88);
	I.sreg[GS].limit = READ32(ea + 0x8c);
	I.sreg[FS].flags = READ32(ea + 0x90) >> 8;
	I.sreg[FS].base = READ32(ea + 0x94);
	I.sreg[FS].limit = READ32(ea + 0x98);
	I.sreg[DS].flags = READ32(ea + 0x9c) >> 8;
	I.sreg[DS].base = READ32(ea + 0xa0);
	I.sreg[DS].limit = READ32(ea + 0xa4);
	I.sreg[SS].flags = READ32(ea + 0xa8) >> 8;
	I.sreg[SS].base = READ32(ea + 0xac);
	I.sreg[SS].limit = READ32(ea + 0xb0);
	I.sreg[CS].flags = READ32(ea + 0xb4) >> 8;
	I.sreg[CS].base = READ32(ea + 0xb8);
	I.sreg[CS].limit = READ32(ea + 0xbc);
	I.sreg[ES].flags = READ32(ea + 0xc0) >> 8;
	I.sreg[ES].base = READ32(ea + 0xc4);
	I.sreg[ES].limit = READ32(ea + 0xc8);
	I.CPL = (I.sreg[SS].flags >> 5) & 3; // cpl == dpl of ss

	for(int i = 0; i <= GS; i++)
	{
		I.sreg[i].valid = (I.sreg[i].flags & 0x80) ? true : false;
		I.sreg[i].d = (I.sreg[i].flags & 0x4000) ? 1 : 0;
	}

	dr7_changed(old_dr7, I.dr[7]);
	CHANGE_PC(I.eip);
}

static void i386_invalid(void)
{
	report_invalid_opcode();
	i386_trap(6, 0);
}

static void i386_xlat(void)          // Opcode 0xd7
{
	uint32_t ea;
	if( I.segment_prefix ) {
		if(!I.address_size)
		{
			ea = i386_translate(I.segment_override, REG16(BX) + REG8(AL), 0 );
		}
		else
		{
			ea = i386_translate(I.segment_override, REG32(EBX) + REG8(AL), 0 );
		}
	} else {
		if(!I.address_size)
		{
			ea = i386_translate(DS, REG16(BX) + REG8(AL), 0 );
		}
		else
		{
			ea = i386_translate(DS, REG32(EBX) + REG8(AL), 0 );
		}
	}
	REG8(AL) = READ8(ea);
	CYCLES(CYCLES_XLAT);
}
