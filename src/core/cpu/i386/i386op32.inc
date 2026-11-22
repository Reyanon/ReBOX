// license:BSD-3-Clause
// copyright-holders:Ville Linde, Barry Rodewald, Carl, Philip Bennett
/*  Converted to C from MAME 0.289 src/devices/cpu/i386/i386op32.hxx (BSD-3-Clause,
    copyright-holders: Ville Linde, Barry Rodewald, Carl, Philip Bennett).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
static uint32_t i386_shift_rotate32(uint8_t modrm, uint32_t value, uint8_t shift)
{
	uint32_t dst, src;
	dst = value;
	src = value;

	if( shift == 0 ) {
		CYCLES_RM(modrm, 3, 7);
	} else if( shift == 1 ) {
		switch( (modrm >> 3) & 0x7 )
		{
			case 0:         /* ROL rm32, 1 */
				I.CF = (src & 0x80000000) ? 1 : 0;
				dst = (src << 1) + I.CF;
				I.OF = ((src ^ dst) & 0x80000000) ? 1 : 0;
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 1:         /* ROR rm32, 1 */
				I.CF = (src & 0x1) ? 1 : 0;
				dst = (I.CF << 31) | (src >> 1);
				I.OF = ((src ^ dst) & 0x80000000) ? 1 : 0;
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 2:         /* RCL rm32, 1 */
				dst = (src << 1) + I.CF;
				I.CF = (src & 0x80000000) ? 1 : 0;
				I.OF = ((src ^ dst) & 0x80000000) ? 1 : 0;
				CYCLES_RM(modrm, CYCLES_ROTATE_CARRY_REG, CYCLES_ROTATE_CARRY_MEM);
				break;
			case 3:         /* RCR rm32, 1 */
				dst = (I.CF << 31) | (src >> 1);
				I.CF = src & 0x1;
				I.OF = ((src ^ dst) & 0x80000000) ? 1 : 0;
				CYCLES_RM(modrm, CYCLES_ROTATE_CARRY_REG, CYCLES_ROTATE_CARRY_MEM);
				break;
			case 4:         /* SHL/SAL rm32, 1 */
			case 6:
				dst = src << 1;
				I.CF = (src & 0x80000000) ? 1 : 0;
				I.OF = (((I.CF << 31) ^ dst) & 0x80000000) ? 1 : 0;
				SetSZPF32(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 5:         /* SHR rm32, 1 */
				dst = src >> 1;
				I.CF = src & 0x1;
				I.OF = (src & 0x80000000) ? 1 : 0;
				SetSZPF32(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 7:         /* SAR rm32, 1 */
				dst = (int32_t)(src) >> 1;
				I.CF = src & 0x1;
				I.OF = 0;
				SetSZPF32(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
		}

	} else {
		shift &= 31;
		switch( (modrm >> 3) & 0x7 )
		{
			case 0:         /* ROL rm32, i8 */
				dst = rotl_32(src, shift);
				I.CF = dst & 0x1;
				I.OF = (dst & 1) ^ (dst >> 31);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 1:         /* ROR rm32, i8 */
				dst = rotr_32(src, shift);
				I.CF = (dst >> 31) & 0x1;
				I.OF = ((dst >> 31) ^ (dst >> 30)) & 1;
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 2:         /* RCL rm32, i8 */
				dst = ((src & ((uint32_t)0xffffffff >> shift)) << shift) |
						((src & ((uint32_t)0xffffffff << (33-shift))) >> (33-shift)) |
						(I.CF << (shift-1));
				I.CF = (src >> (32-shift)) & 0x1;
				I.OF = I.CF ^ ((dst >> 31) & 1);
				CYCLES_RM(modrm, CYCLES_ROTATE_CARRY_REG, CYCLES_ROTATE_CARRY_MEM);
				break;
			case 3:         /* RCR rm32, i8 */
				dst = ((src & ((uint32_t)0xffffffff << shift)) >> shift) |
						((src & ((uint32_t)0xffffffff >> (32-shift))) << (33-shift)) |
						(I.CF << (32-shift));
				I.CF = (src >> (shift-1)) & 0x1;
				I.OF = ((dst >> 31) ^ (dst >> 30)) & 1;
				CYCLES_RM(modrm, CYCLES_ROTATE_CARRY_REG, CYCLES_ROTATE_CARRY_MEM);
				break;
			case 4:         /* SHL/SAL rm32, i8 */
			case 6:
				dst = src << shift;
				I.CF = (src & (1 << (32-shift))) ? 1 : 0;
				SetSZPF32(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 5:         /* SHR rm32, i8 */
				dst = src >> shift;
				I.CF = (src & (1 << (shift-1))) ? 1 : 0;
				SetSZPF32(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
			case 7:         /* SAR rm32, i8 */
				dst = (int32_t)src >> shift;
				I.CF = (src & (1 << (shift-1))) ? 1 : 0;
				SetSZPF32(dst);
				CYCLES_RM(modrm, CYCLES_ROTATE_REG, CYCLES_ROTATE_MEM);
				break;
		}

	}
	return dst;
}



static void i386_adc_rm32_r32(void)      // Opcode 0x11
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG32(modrm);
		dst = LOAD_RM32(modrm);
		dst = ADC32(dst, src, I.CF);
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG32(modrm);
		dst = READ32(ea);
		dst = ADC32(dst, src, I.CF);
		WRITE32(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_adc_r32_rm32(void)      // Opcode 0x13
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
		dst = LOAD_REG32(modrm);
		dst = ADC32(dst, src, I.CF);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
		dst = LOAD_REG32(modrm);
		dst = ADC32(dst, src, I.CF);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_adc_eax_i32(void)       // Opcode 0x15
{
	uint32_t src, dst;
	src = FETCH32();
	dst = REG32(EAX);
	dst = ADC32(dst, src, I.CF);
	REG32(EAX) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_add_rm32_r32(void)      // Opcode 0x01
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG32(modrm);
		dst = LOAD_RM32(modrm);
		dst = ADD32(dst, src);
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG32(modrm);
		dst = READ32(ea);
		dst = ADD32(dst, src);
		WRITE32(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_add_r32_rm32(void)      // Opcode 0x03
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
		dst = LOAD_REG32(modrm);
		dst = ADD32(dst, src);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
		dst = LOAD_REG32(modrm);
		dst = ADD32(dst, src);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_add_eax_i32(void)       // Opcode 0x05
{
	uint32_t src, dst;
	src = FETCH32();
	dst = REG32(EAX);
	dst = ADD32(dst, src);
	REG32(EAX) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_and_rm32_r32(void)      // Opcode 0x21
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG32(modrm);
		dst = LOAD_RM32(modrm);
		dst = AND32(dst, src);
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG32(modrm);
		dst = READ32(ea);
		dst = AND32(dst, src);
		WRITE32(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_and_r32_rm32(void)      // Opcode 0x23
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
		dst = LOAD_REG32(modrm);
		dst = AND32(dst, src);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
		dst = LOAD_REG32(modrm);
		dst = AND32(dst, src);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_and_eax_i32(void)       // Opcode 0x25
{
	uint32_t src, dst;
	src = FETCH32();
	dst = REG32(EAX);
	dst = AND32(dst, src);
	REG32(EAX) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_bsf_r32_rm32(void)      // Opcode 0x0f bc
{
	uint32_t src, dst, temp;
	uint8_t modrm = FETCH();

	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
	}

	dst = 0;

	if( src == 0 ) {
		I.ZF = 1;
	} else {
		I.ZF = 0;
		temp = 0;
		while( (src & (1 << temp)) == 0 ) {
			temp++;
			dst = temp;
			CYCLES(CYCLES_BSF);
		}
		STORE_REG32(modrm, dst);
	}
	CYCLES(CYCLES_BSF_BASE);
}

static void i386_bsr_r32_rm32(void)      // Opcode 0x0f bd
{
	uint32_t src, dst, temp;
	uint8_t modrm = FETCH();

	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
	}

	dst = 0;

	if( src == 0 ) {
		I.ZF = 1;
	} else {
		I.ZF = 0;
		dst = temp = 31;
		while( (src & (1U << temp)) == 0 ) {
			temp--;
			dst = temp;
			CYCLES(CYCLES_BSR);
		}
		STORE_REG32(modrm, dst);
	}
	CYCLES(CYCLES_BSR_BASE);
}

static void i386_bt_rm32_r32(void)       // Opcode 0x0f a3
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t dst = LOAD_RM32(modrm);
		uint32_t bit = LOAD_REG32(modrm);

		if( dst & (1 << bit) )
			I.CF = 1;
		else
			I.CF = 0;

		CYCLES(CYCLES_BT_REG_REG);
	} else {
		uint8_t segment;
		uint32_t ea = GetNonTranslatedEA(modrm,&segment);
		uint32_t bit = LOAD_REG32(modrm);
		ea += 4*(bit/32);
		ea = i386_translate(segment,(I.address_size)?ea:(ea&0xffff),0);
		bit %= 32;
		uint32_t dst = READ32(ea);

		if( dst & (1 << bit) )
			I.CF = 1;
		else
			I.CF = 0;

		CYCLES(CYCLES_BT_REG_MEM);
	}
}

static void i386_btc_rm32_r32(void)      // Opcode 0x0f bb
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t dst = LOAD_RM32(modrm);
		uint32_t bit = LOAD_REG32(modrm);

		if( dst & (1 << bit) )
			I.CF = 1;
		else
			I.CF = 0;
		dst ^= (1 << bit);

		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_BTC_REG_REG);
	} else {
		uint8_t segment;
		uint32_t ea = GetNonTranslatedEA(modrm,&segment);
		uint32_t bit = LOAD_REG32(modrm);
		ea += 4*(bit/32);
		ea = i386_translate(segment,(I.address_size)?ea:(ea&0xffff),1);
		bit %= 32;
		uint32_t dst = READ32(ea);

		if( dst & (1 << bit) )
			I.CF = 1;
		else
			I.CF = 0;
		dst ^= (1 << bit);

		WRITE32(ea, dst);
		CYCLES(CYCLES_BTC_REG_MEM);
	}
}

static void i386_btr_rm32_r32(void)      // Opcode 0x0f b3
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t dst = LOAD_RM32(modrm);
		uint32_t bit = LOAD_REG32(modrm);

		if( dst & (1 << bit) )
			I.CF = 1;
		else
			I.CF = 0;
		dst &= ~(1 << bit);

		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_BTR_REG_REG);
	} else {
		uint8_t segment;
		uint32_t ea = GetNonTranslatedEA(modrm,&segment);
		uint32_t bit = LOAD_REG32(modrm);
		ea += 4*(bit/32);
		ea = i386_translate(segment,(I.address_size)?ea:(ea&0xffff),1);
		bit %= 32;
		uint32_t dst = READ32(ea);

		if( dst & (1 << bit) )
			I.CF = 1;
		else
			I.CF = 0;
		dst &= ~(1 << bit);

		WRITE32(ea, dst);
		CYCLES(CYCLES_BTR_REG_MEM);
	}
}

static void i386_bts_rm32_r32(void)      // Opcode 0x0f ab
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t dst = LOAD_RM32(modrm);
		uint32_t bit = LOAD_REG32(modrm);

		if( dst & (1 << bit) )
			I.CF = 1;
		else
			I.CF = 0;
		dst |= (1 << bit);

		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_BTS_REG_REG);
	} else {
		uint8_t segment;
		uint32_t ea = GetNonTranslatedEA(modrm,&segment);
		uint32_t bit = LOAD_REG32(modrm);
		ea += 4*(bit/32);
		ea = i386_translate(segment,(I.address_size)?ea:(ea&0xffff),1);
		bit %= 32;
		uint32_t dst = READ32(ea);

		if( dst & (1 << bit) )
			I.CF = 1;
		else
			I.CF = 0;
		dst |= (1 << bit);

		WRITE32(ea, dst);
		CYCLES(CYCLES_BTS_REG_MEM);
	}
}

static void i386_call_abs32(void)        // Opcode 0x9a
{
	uint32_t offset = FETCH32();
	uint16_t ptr = FETCH16();

	if(PROTECTED_MODE && !V8086_MODE)
	{
		i386_protected_mode_call(ptr,offset,0,1);
	}
	else
	{
		PUSH32SEG(I.sreg[CS].selector );
		PUSH32(I.eip );
		I.sreg[CS].selector = ptr;
		I.performed_intersegment_jump = 1;
		I.eip = offset;
		i386_load_segment_descriptor(CS);
	}
	CYCLES(CYCLES_CALL_INTERSEG);
	CHANGE_PC(I.eip);
}

static void i386_call_rel32(void)        // Opcode 0xe8
{
	int32_t disp = FETCH32();
	PUSH32(I.eip );
	I.eip += disp;
	CHANGE_PC(I.eip);
	CYCLES(CYCLES_CALL);       /* TODO: Timing = 7 + m */
}

static void i386_cdq(void)               // Opcode 0x99
{
	if( REG32(EAX) & 0x80000000 ) {
		REG32(EDX) = 0xffffffff;
	} else {
		REG32(EDX) = 0x00000000;
	}
	CYCLES(CYCLES_CWD);
}

static void i386_cmp_rm32_r32(void)      // Opcode 0x39
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG32(modrm);
		dst = LOAD_RM32(modrm);
		SUB32(dst, src);
		CYCLES(CYCLES_CMP_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = LOAD_REG32(modrm);
		dst = READ32(ea);
		SUB32(dst, src);
		CYCLES(CYCLES_CMP_REG_MEM);
	}
}

static void i386_cmp_r32_rm32(void)      // Opcode 0x3b
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
		dst = LOAD_REG32(modrm);
		SUB32(dst, src);
		CYCLES(CYCLES_CMP_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
		dst = LOAD_REG32(modrm);
		SUB32(dst, src);
		CYCLES(CYCLES_CMP_MEM_REG);
	}
}

static void i386_cmp_eax_i32(void)       // Opcode 0x3d
{
	uint32_t src, dst;
	src = FETCH32();
	dst = REG32(EAX);
	SUB32(dst, src);
	CYCLES(CYCLES_CMP_IMM_ACC);
}

static void i386_cmpsd(void)             // Opcode 0xa7
{
	uint32_t eas, ead, src, dst;
	if( I.segment_prefix ) {
		eas = i386_translate(I.segment_override, I.address_size ? REG32(ESI) : REG16(SI), 0 );
	} else {
		eas = i386_translate(DS, I.address_size ? REG32(ESI) : REG16(SI), 0 );
	}
	ead = i386_translate(ES, I.address_size ? REG32(EDI) : REG16(DI), 0 );
	src = READ32(eas);
	dst = READ32(ead);
	SUB32(src,dst);
	BUMP_SI(4);
	BUMP_DI(4);
	CYCLES(CYCLES_CMPS);
}

static void i386_cwde(void)              // Opcode 0x98
{
	REG32(EAX) = (int32_t)((int16_t)REG16(AX));
	CYCLES(CYCLES_CBW);
}

static void i386_dec_eax(void)           // Opcode 0x48
{
	REG32(EAX) = DEC32(REG32(EAX) );
	CYCLES(CYCLES_DEC_REG);
}

static void i386_dec_ecx(void)           // Opcode 0x49
{
	REG32(ECX) = DEC32(REG32(ECX) );
	CYCLES(CYCLES_DEC_REG);
}

static void i386_dec_edx(void)           // Opcode 0x4a
{
	REG32(EDX) = DEC32(REG32(EDX) );
	CYCLES(CYCLES_DEC_REG);
}

static void i386_dec_ebx(void)           // Opcode 0x4b
{
	REG32(EBX) = DEC32(REG32(EBX) );
	CYCLES(CYCLES_DEC_REG);
}

static void i386_dec_esp(void)           // Opcode 0x4c
{
	REG32(ESP) = DEC32(REG32(ESP) );
	CYCLES(CYCLES_DEC_REG);
}

static void i386_dec_ebp(void)           // Opcode 0x4d
{
	REG32(EBP) = DEC32(REG32(EBP) );
	CYCLES(CYCLES_DEC_REG);
}

static void i386_dec_esi(void)           // Opcode 0x4e
{
	REG32(ESI) = DEC32(REG32(ESI) );
	CYCLES(CYCLES_DEC_REG);
}

static void i386_dec_edi(void)           // Opcode 0x4f
{
	REG32(EDI) = DEC32(REG32(EDI) );
	CYCLES(CYCLES_DEC_REG);
}

static void i386_imul_r32_rm32(void)     // Opcode 0x0f af
{
	uint8_t modrm = FETCH();
	int64_t result;
	int64_t src, dst;
	if( modrm >= 0xc0 ) {
		src = (int64_t)(int32_t)LOAD_RM32(modrm);
		CYCLES(CYCLES_IMUL32_REG_REG);     /* TODO: Correct multiply timing */
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = (int64_t)(int32_t)READ32(ea);
		CYCLES(CYCLES_IMUL32_REG_REG);     /* TODO: Correct multiply timing */
	}

	dst = (int64_t)(int32_t)LOAD_REG32(modrm);
	result = src * dst;

	STORE_REG32(modrm, (uint32_t)result);

	I.CF = I.OF = !(result == (int64_t)(int32_t)result);
}

static void i386_imul_r32_rm32_i32(void) // Opcode 0x69
{
	uint8_t modrm = FETCH();
	int64_t result;
	int64_t src, dst;
	if( modrm >= 0xc0 ) {
		dst = (int64_t)(int32_t)LOAD_RM32(modrm);
		CYCLES(CYCLES_IMUL32_REG_IMM_REG);     /* TODO: Correct multiply timing */
	} else {
		uint32_t ea = GetEA(modrm,0);
		dst = (int64_t)(int32_t)READ32(ea);
		CYCLES(CYCLES_IMUL32_MEM_IMM_REG);     /* TODO: Correct multiply timing */
	}

	src = (int64_t)(int32_t)FETCH32();
	result = src * dst;

	STORE_REG32(modrm, (uint32_t)result);

	I.CF = I.OF = !(result == (int64_t)(int32_t)result);
}

static void i386_imul_r32_rm32_i8(void)  // Opcode 0x6b
{
	uint8_t modrm = FETCH();
	int64_t result;
	int64_t src, dst;
	if( modrm >= 0xc0 ) {
		dst = (int64_t)(int32_t)LOAD_RM32(modrm);
		CYCLES(CYCLES_IMUL32_REG_IMM_REG);     /* TODO: Correct multiply timing */
	} else {
		uint32_t ea = GetEA(modrm,0);
		dst = (int64_t)(int32_t)READ32(ea);
		CYCLES(CYCLES_IMUL32_MEM_IMM_REG);     /* TODO: Correct multiply timing */
	}

	src = (int64_t)(int8_t)FETCH();
	result = src * dst;

	STORE_REG32(modrm, (uint32_t)result);

	I.CF = I.OF = !(result == (int64_t)(int32_t)result);
}

static void i386_in_eax_i8(void)         // Opcode 0xe5
{
	uint16_t port = FETCH();
	uint32_t data = READPORT32(port);
	REG32(EAX) = data;
	CYCLES(CYCLES_IN_VAR);
}

static void i386_in_eax_dx(void)         // Opcode 0xed
{
	uint16_t port = REG16(DX);
	uint32_t data = READPORT32(port);
	REG32(EAX) = data;
	CYCLES(CYCLES_IN);
}

static void i386_inc_eax(void)           // Opcode 0x40
{
	REG32(EAX) = INC32(REG32(EAX) );
	CYCLES(CYCLES_INC_REG);
}

static void i386_inc_ecx(void)           // Opcode 0x41
{
	REG32(ECX) = INC32(REG32(ECX) );
	CYCLES(CYCLES_INC_REG);
}

static void i386_inc_edx(void)           // Opcode 0x42
{
	REG32(EDX) = INC32(REG32(EDX) );
	CYCLES(CYCLES_INC_REG);
}

static void i386_inc_ebx(void)           // Opcode 0x43
{
	REG32(EBX) = INC32(REG32(EBX) );
	CYCLES(CYCLES_INC_REG);
}

static void i386_inc_esp(void)           // Opcode 0x44
{
	REG32(ESP) = INC32(REG32(ESP) );
	CYCLES(CYCLES_INC_REG);
}

static void i386_inc_ebp(void)           // Opcode 0x45
{
	REG32(EBP) = INC32(REG32(EBP) );
	CYCLES(CYCLES_INC_REG);
}

static void i386_inc_esi(void)           // Opcode 0x46
{
	REG32(ESI) = INC32(REG32(ESI) );
	CYCLES(CYCLES_INC_REG);
}

static void i386_inc_edi(void)           // Opcode 0x47
{
	REG32(EDI) = INC32(REG32(EDI) );
	CYCLES(CYCLES_INC_REG);
}

static void i386_iret32(void)            // Opcode 0xcf
{
	if( PROTECTED_MODE )
	{
		i386_protected_mode_iret(1);
	}
	else
	{
		/* TODO: #SS(0) exception */
		/* TODO: #GP(0) exception */
		I.eip = POP32();
		I.sreg[CS].selector = POP32() & 0xffff;
		set_flags(POP32() );
		i386_load_segment_descriptor(CS);
		CHANGE_PC(I.eip);
	}
	I.auto_clear_RF = false;
	CYCLES(CYCLES_IRET);
}

static void i386_ja_rel32(void)          // Opcode 0x0f 87
{
	int32_t disp = FETCH32();
	if( I.CF == 0 && I.ZF == 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jbe_rel32(void)         // Opcode 0x0f 86
{
	int32_t disp = FETCH32();
	if( I.CF != 0 || I.ZF != 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jc_rel32(void)          // Opcode 0x0f 82
{
	int32_t disp = FETCH32();
	if( I.CF != 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jg_rel32(void)          // Opcode 0x0f 8f
{
	int32_t disp = FETCH32();
	if( I.ZF == 0 && (I.SF == I.OF) ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jge_rel32(void)         // Opcode 0x0f 8d
{
	int32_t disp = FETCH32();
	if(I.SF == I.OF) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jl_rel32(void)          // Opcode 0x0f 8c
{
	int32_t disp = FETCH32();
	if( (I.SF != I.OF) ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jle_rel32(void)         // Opcode 0x0f 8e
{
	int32_t disp = FETCH32();
	if( I.ZF != 0 || (I.SF != I.OF) ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jnc_rel32(void)         // Opcode 0x0f 83
{
	int32_t disp = FETCH32();
	if( I.CF == 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jno_rel32(void)         // Opcode 0x0f 81
{
	int32_t disp = FETCH32();
	if( I.OF == 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jnp_rel32(void)         // Opcode 0x0f 8b
{
	int32_t disp = FETCH32();
	if( I.PF == 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jns_rel32(void)         // Opcode 0x0f 89
{
	int32_t disp = FETCH32();
	if( I.SF == 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jnz_rel32(void)         // Opcode 0x0f 85
{
	int32_t disp = FETCH32();
	if( I.ZF == 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jo_rel32(void)          // Opcode 0x0f 80
{
	int32_t disp = FETCH32();
	if( I.OF != 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jp_rel32(void)          // Opcode 0x0f 8a
{
	int32_t disp = FETCH32();
	if( I.PF != 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_js_rel32(void)          // Opcode 0x0f 88
{
	int32_t disp = FETCH32();
	if( I.SF != 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jz_rel32(void)          // Opcode 0x0f 84
{
	int32_t disp = FETCH32();
	if( I.ZF != 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCC_FULL_DISP);      /* TODO: Timing = 7 + m */
	} else {
		CYCLES(CYCLES_JCC_FULL_DISP_NOBRANCH);
	}
}

static void i386_jcxz32(void)            // Opcode 0xe3
{
	int8_t disp = FETCH();
	int val = (I.address_size)?(REG32(ECX) == 0):(REG16(CX) == 0);
	if( val ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
		CYCLES(CYCLES_JCXZ);       /* TODO: Timing = 9 + m */
	} else {
		CYCLES(CYCLES_JCXZ_NOBRANCH);
	}
}

static void i386_jmp_rel32(void)         // Opcode 0xe9
{
	uint32_t disp = FETCH32();
	/* TODO: Segment limit */
	I.eip += disp;
	CHANGE_PC(I.eip);
	CYCLES(CYCLES_JMP);        /* TODO: Timing = 7 + m */
}

static void i386_jmp_abs32(void)         // Opcode 0xea
{
	uint32_t address = FETCH32();
	uint16_t segment = FETCH16();

	if( PROTECTED_MODE && !V8086_MODE)
	{
		i386_protected_mode_jump(segment,address,0,1);
	}
	else
	{
		I.eip = address;
		I.sreg[CS].selector = segment;
		I.performed_intersegment_jump = 1;
		i386_load_segment_descriptor(CS);
		CHANGE_PC(I.eip);
	}
	CYCLES(CYCLES_JMP_INTERSEG);
}

static void i386_lea32(void)             // Opcode 0x8d
{
	uint8_t modrm = FETCH();
	uint32_t ea = GetNonTranslatedEA(modrm,NULL);
	if (!I.address_size)
	{
		ea &= 0xffff;
	}
	STORE_REG32(modrm, ea);
	CYCLES(CYCLES_LEA);
}

static void i386_enter32(void)           // Opcode 0xc8
{
	uint16_t framesize = FETCH16();
	uint8_t level = FETCH() % 32;
	uint8_t x;
	uint32_t frameptr;
	PUSH32(REG32(EBP));
	if(!STACK_32BIT)
		frameptr = REG16(SP);
	else
		frameptr = REG32(ESP);

	if(level > 0)
	{
		for(x=1;x<=level-1;x++)
		{
			uint32_t addr;
			if(!STACK_32BIT)
			{
				REG16(BP) -= 4;
				addr = REG16(BP);
			}
			else
			{
				REG32(EBP) -= 4;
				addr = REG32(EBP);
			}
			PUSH32(READ32(i386_translate(SS, addr, 0)));
		}
		PUSH32(frameptr);
	}
	REG32(EBP) = frameptr;
	if(!STACK_32BIT)
		REG16(SP) -= framesize;
	else
		REG32(ESP) -= framesize;
	CYCLES(CYCLES_ENTER);
}

static void i386_leave32(void)           // Opcode 0xc9
{
	if(!STACK_32BIT)
		REG16(SP) = REG16(BP);
	else
		REG32(ESP) = REG32(EBP);
	REG32(EBP) = POP32();
	CYCLES(CYCLES_LEAVE);
}

static void i386_lodsd(void)             // Opcode 0xad
{
	uint32_t eas;
	if( I.segment_prefix ) {
		eas = i386_translate(I.segment_override, I.address_size ? REG32(ESI) : REG16(SI), 0, 4 );
	} else {
		eas = i386_translate(DS, I.address_size ? REG32(ESI) : REG16(SI), 0, 4 );
	}
	REG32(EAX) = READ32(eas);
	BUMP_SI(4);
	CYCLES(CYCLES_LODS);
}

static void i386_loop32(void)            // Opcode 0xe2
{
	int8_t disp = FETCH();
	int32_t reg = (I.address_size)?--REG32(ECX):--REG16(CX);
	if( reg != 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
	}
	CYCLES(CYCLES_LOOP);       /* TODO: Timing = 11 + m */
}

static void i386_loopne32(void)          // Opcode 0xe0
{
	int8_t disp = FETCH();
	int32_t reg = (I.address_size)?--REG32(ECX):--REG16(CX);
	if( reg != 0 && I.ZF == 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
	}
	CYCLES(CYCLES_LOOPNZ);     /* TODO: Timing = 11 + m */
}

static void i386_loopz32(void)           // Opcode 0xe1
{
	int8_t disp = FETCH();
	int32_t reg = (I.address_size)?--REG32(ECX):--REG16(CX);
	if( reg != 0 && I.ZF != 0 ) {
		I.eip += disp;
		CHANGE_PC(I.eip);
	}
	CYCLES(CYCLES_LOOPZ);      /* TODO: Timing = 11 + m */
}

static void i386_mov_rm32_r32(void)      // Opcode 0x89
{
	uint32_t src;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG32(modrm);
		STORE_RM32(modrm, src);
		CYCLES(CYCLES_MOV_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG32(modrm);
		WRITE32(ea, src);
		CYCLES(CYCLES_MOV_REG_MEM);
	}
}

static void i386_mov_r32_rm32(void)      // Opcode 0x8b
{
	uint32_t src;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_MOV_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_MOV_MEM_REG);
	}
}

static void i386_mov_rm32_i32(void)      // Opcode 0xc7
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t value = FETCH32();
		STORE_RM32(modrm, value);
		CYCLES(CYCLES_MOV_IMM_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		uint32_t value = FETCH32();
		WRITE32(ea, value);
		CYCLES(CYCLES_MOV_IMM_MEM);
	}
}

static void i386_mov_eax_m32(void)       // Opcode 0xa1
{
	uint32_t offset, ea;
	if( I.address_size ) {
		offset = FETCH32();
	} else {
		offset = FETCH16();
	}
	if( I.segment_prefix ) {
		ea = i386_translate(I.segment_override, offset, 0 );
	} else {
		ea = i386_translate(DS, offset, 0 );
	}
	REG32(EAX) = READ32(ea);
	CYCLES(CYCLES_MOV_MEM_ACC);
}

static void i386_mov_m32_eax(void)       // Opcode 0xa3
{
	uint32_t offset, ea;
	if( I.address_size ) {
		offset = FETCH32();
	} else {
		offset = FETCH16();
	}
	if( I.segment_prefix ) {
		ea = i386_translate(I.segment_override, offset, 1 );
	} else {
		ea = i386_translate(DS, offset, 1 );
	}
	WRITE32(ea, REG32(EAX) );
	CYCLES(CYCLES_MOV_ACC_MEM);
}

static void i386_mov_eax_i32(void)       // Opcode 0xb8
{
	REG32(EAX) = FETCH32();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_ecx_i32(void)       // Opcode 0xb9
{
	REG32(ECX) = FETCH32();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_edx_i32(void)       // Opcode 0xba
{
	REG32(EDX) = FETCH32();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_ebx_i32(void)       // Opcode 0xbb
{
	REG32(EBX) = FETCH32();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_esp_i32(void)       // Opcode 0xbc
{
	REG32(ESP) = FETCH32();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_ebp_i32(void)       // Opcode 0xbd
{
	REG32(EBP) = FETCH32();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_esi_i32(void)       // Opcode 0xbe
{
	REG32(ESI) = FETCH32();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_mov_edi_i32(void)       // Opcode 0xbf
{
	REG32(EDI) = FETCH32();
	CYCLES(CYCLES_MOV_IMM_REG);
}

static void i386_movsd(void)             // Opcode 0xa5
{
	uint32_t eas, ead, v;
	if( I.segment_prefix ) {
		eas = i386_translate(I.segment_override, I.address_size ? REG32(ESI) : REG16(SI), 0, 4 );
	} else {
		eas = i386_translate(DS, I.address_size ? REG32(ESI) : REG16(SI), 0, 4 );
	}
	ead = i386_translate(ES, I.address_size ? REG32(EDI) : REG16(DI), 1, 4 );
	v = READ32(eas);
	WRITE32(ead, v);
	BUMP_SI(4);
	BUMP_DI(4);
	CYCLES(CYCLES_MOVS);
}

static void i386_movsx_r32_rm8(void)     // Opcode 0x0f be
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		int32_t src = (int8_t)LOAD_RM8(modrm);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_MOVSX_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		int32_t src = (int8_t)READ8(ea);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_MOVSX_MEM_REG);
	}
}

static void i386_movsx_r32_rm16(void)    // Opcode 0x0f bf
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		int32_t src = (int16_t)LOAD_RM16(modrm);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_MOVSX_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		int32_t src = (int16_t)READ16(ea);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_MOVSX_MEM_REG);
	}
}

static void i386_movzx_r32_rm8(void)     // Opcode 0x0f b6
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t src = (uint8_t)LOAD_RM8(modrm);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_MOVZX_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		uint32_t src = (uint8_t)READ8(ea);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_MOVZX_MEM_REG);
	}
}

static void i386_movzx_r32_rm16(void)    // Opcode 0x0f b7
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t src = (uint16_t)LOAD_RM16(modrm);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_MOVZX_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		uint32_t src = (uint16_t)READ16(ea);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_MOVZX_MEM_REG);
	}
}

static void i386_or_rm32_r32(void)       // Opcode 0x09
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG32(modrm);
		dst = LOAD_RM32(modrm);
		dst = OR32(dst, src);
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG32(modrm);
		dst = READ32(ea);
		dst = OR32(dst, src);
		WRITE32(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_or_r32_rm32(void)       // Opcode 0x0b
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
		dst = LOAD_REG32(modrm);
		dst = OR32(dst, src);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
		dst = LOAD_REG32(modrm);
		dst = OR32(dst, src);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_or_eax_i32(void)        // Opcode 0x0d
{
	uint32_t src, dst;
	src = FETCH32();
	dst = REG32(EAX);
	dst = OR32(dst, src);
	REG32(EAX) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_out_eax_i8(void)        // Opcode 0xe7
{
	uint16_t port = FETCH();
	uint32_t data = REG32(EAX);
	WRITEPORT32(port, data);
	CYCLES(CYCLES_OUT_VAR);
}

static void i386_out_eax_dx(void)        // Opcode 0xef
{
	uint16_t port = REG16(DX);
	uint32_t data = REG32(EAX);
	WRITEPORT32(port, data);
	CYCLES(CYCLES_OUT);
}

static void i386_pop_eax(void)           // Opcode 0x58
{
	uint32_t offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	if(i386_limit_check(SS,offset+3) == 0)
		REG32(EAX) = POP32();
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POP_REG_SHORT);
}

static void i386_pop_ecx(void)           // Opcode 0x59
{
	uint32_t offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	if(i386_limit_check(SS,offset+3) == 0)
		REG32(ECX) = POP32();
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POP_REG_SHORT);
}

static void i386_pop_edx(void)           // Opcode 0x5a
{
	uint32_t offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	if(i386_limit_check(SS,offset+3) == 0)
		REG32(EDX) = POP32();
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POP_REG_SHORT);
}

static void i386_pop_ebx(void)           // Opcode 0x5b
{
	uint32_t offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	if(i386_limit_check(SS,offset+3) == 0)
		REG32(EBX) = POP32();
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POP_REG_SHORT);
}

static void i386_pop_esp(void)           // Opcode 0x5c
{
	uint32_t offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	if(i386_limit_check(SS,offset+3) == 0)
		REG32(ESP) = POP32();
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POP_REG_SHORT);
}

static void i386_pop_ebp(void)           // Opcode 0x5d
{
	uint32_t offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	if(i386_limit_check(SS,offset+3) == 0)
		REG32(EBP) = POP32();
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POP_REG_SHORT);
}

static void i386_pop_esi(void)           // Opcode 0x5e
{
	uint32_t offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	if(i386_limit_check(SS,offset+3) == 0)
		REG32(ESI) = POP32();
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POP_REG_SHORT);
}

static void i386_pop_edi(void)           // Opcode 0x5f
{
	uint32_t offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	if(i386_limit_check(SS,offset+3) == 0)
		REG32(EDI) = POP32();
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POP_REG_SHORT);
}

static bool i386_pop_seg32(int segment)
{
	uint32_t ea, offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	uint32_t value;
	bool fault;
	if(i386_limit_check(SS,offset+3) == 0)
	{
		ea = i386_translate(SS, offset, 0);
		value = READ32(ea);
		i386_sreg_load(value, segment, &fault);
		if(fault) return false;
		if(STACK_32BIT)
			REG32(ESP) = offset + 4;
		else
			REG16(SP) = offset + 4;
	}
	else
	{
		I.ext = 1;
		i386_trap_with_error(FAULT_SS,0,0,0);
		return false;
	}
	CYCLES(CYCLES_POP_SREG);
	return true;
}

static void i386_pop_ds32(void)          // Opcode 0x1f
{
	i386_pop_seg32(DS);
}

static void i386_pop_es32(void)          // Opcode 0x07
{
	i386_pop_seg32(ES);
}

static void i386_pop_fs32(void)          // Opcode 0x0f a1
{
	i386_pop_seg32(FS);
}

static void i386_pop_gs32(void)          // Opcode 0x0f a9
{
	i386_pop_seg32(GS);
}

static void i386_pop_ss32(void)          // Opcode 0x17
{
	if(!i386_pop_seg32(SS)) return;
	if(I.IF != 0) // if external interrupts are enabled
	{
		I.IF = 0;  // reset IF for the next instruction
		I.delayed_interrupt_enable = 1;
	}
}

static void i386_pop_rm32(void)          // Opcode 0x8f
{
	uint8_t modrm = FETCH();
	uint32_t value;
	uint32_t ea, offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	if(i386_limit_check(SS,offset+3) == 0)
	{
		// be careful here, if the write references the esp register
		// it expects the post-pop value but esp must be wound back
		// if the write faults
		uint32_t temp_sp = REG32(ESP);
		value = POP32();

		if( modrm >= 0xc0 ) {
			STORE_RM32(modrm, value);
		} else {
			{ I386_TRY_BEGIN
	if (!setjmp(jb_)) {
				ea = GetEA(modrm,1);
				WRITE32(ea, value);
				s_jmp = prev_;
	} else {
	const uint64_t e = s_exc;
	s_jmp = prev_;
	(void)e;
				REG32(ESP) = temp_sp;
				i386_throw(e);
			}}
		}
	}
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POP_RM);
}

static void i386_popad(void)             // Opcode 0x61
{
	uint32_t offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));
	if(i386_limit_check(SS,offset+31) == 0)
	{
		REG32(EDI) = POP32();
		REG32(ESI) = POP32();
		REG32(EBP) = POP32();
		REG32(ESP) += 4;
		REG32(EBX) = POP32();
		REG32(EDX) = POP32();
		REG32(ECX) = POP32();
		REG32(EAX) = POP32();
	}
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POPA);
}

static void i386_popfd(void)             // Opcode 0x9d
{
	uint32_t value;
	uint32_t current = get_flags();
	uint8_t IOPL = (current >> 12) & 0x03;
	uint32_t mask = 0x00257fd5;  // VM, VIP and VIF cannot be set by POPF/POPFD
	uint32_t offset = (STACK_32BIT ? REG32(ESP) : REG16(SP));

	// IOPL can only change if CPL is 0
	if(I.CPL != 0)
		mask &= ~0x00003000;

	// IF can only change if CPL is at least as privileged as IOPL
	if(I.CPL > IOPL)
		mask &= ~0x00000200;

	if(V8086_MODE)
	{
		if(IOPL < 3)
		{
			LOGMASKED(LOG_PM_FAULT_GP, "POPFD(%08x): IOPL < 3 while in V86 mode.\n",I.pc);
			FAULT(FAULT_GP,0)  // #GP(0)
		}
		mask &= ~0x00003000;  // IOPL cannot be changed while in V8086 mode
	}

	if(i386_limit_check(SS,offset+3) == 0)
	{
		value = POP32();
		value &= ~0x00010000;  // RF will always return zero
		set_flags((current & ~mask) | (value & mask));  // mask out reserved bits
	}
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_POPF);
}

static void i386_push_eax(void)          // Opcode 0x50
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32(REG32(EAX) );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_REG_SHORT);
}

static void i386_push_ecx(void)          // Opcode 0x51
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32(REG32(ECX) );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_REG_SHORT);
}

static void i386_push_edx(void)          // Opcode 0x52
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32(REG32(EDX) );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_REG_SHORT);
}

static void i386_push_ebx(void)          // Opcode 0x53
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32(REG32(EBX) );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_REG_SHORT);
}

static void i386_push_esp(void)          // Opcode 0x54
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32(REG32(ESP) );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_REG_SHORT);
}

static void i386_push_ebp(void)          // Opcode 0x55
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32(REG32(EBP) );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_REG_SHORT);
}

static void i386_push_esi(void)          // Opcode 0x56
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32(REG32(ESI) );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_REG_SHORT);
}

static void i386_push_edi(void)          // Opcode 0x57
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32(REG32(EDI) );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_REG_SHORT);
}

static void i386_push_cs32(void)         // Opcode 0x0e
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32SEG(I.sreg[CS].selector );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_SREG);
}

static void i386_push_ds32(void)         // Opcode 0x1e
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32SEG(I.sreg[DS].selector );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_SREG);
}

static void i386_push_es32(void)         // Opcode 0x06
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32SEG(I.sreg[ES].selector );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_SREG);
}

static void i386_push_fs32(void)         // Opcode 0x0f a0
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32SEG(I.sreg[FS].selector );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_SREG);
}

static void i386_push_gs32(void)         // Opcode 0x0f a8
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32SEG(I.sreg[GS].selector );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_SREG);
}

static void i386_push_ss32(void)         // Opcode 0x16
{
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32SEG(I.sreg[SS].selector );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_SREG);
}

static void i386_push_i32(void)          // Opcode 0x68
{
	uint32_t value = FETCH32();
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 4;
	else
		offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32(value);
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSH_IMM);
}

static void i386_pushad(void)            // Opcode 0x60
{
	uint32_t temp = REG32(ESP);
	uint32_t offset;
	if(STACK_32BIT)
		offset = REG32(ESP) - 32;
	else
		offset = (REG16(SP) - 32) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
	{
		PUSH32(REG32(EAX) );
		PUSH32(REG32(ECX) );
		PUSH32(REG32(EDX) );
		PUSH32(REG32(EBX) );
		PUSH32(temp );
		PUSH32(REG32(EBP) );
		PUSH32(REG32(ESI) );
		PUSH32(REG32(EDI) );
	}
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSHA);
}

static void i386_pushfd(void)            // Opcode 0x9c
{
	if(!I.IOP1 && !I.IOP2 && V8086_MODE)
		FAULT(FAULT_GP,0)
		uint32_t offset;
		if(STACK_32BIT)
			offset = REG32(ESP) - 4;
		else
			offset = (REG16(SP) - 4) & 0xffff;
	if(i386_limit_check(SS,offset) == 0)
		PUSH32(get_flags() & 0x00fcffff );
	else
		FAULT(FAULT_SS,0)
	CYCLES(CYCLES_PUSHF);
}

static void i386_ret_near32_i16(void)    // Opcode 0xc2
{
	int16_t disp = FETCH16();
	I.eip = POP32();
	REG32(ESP) += disp;
	CHANGE_PC(I.eip);
	CYCLES(CYCLES_RET_IMM);        /* TODO: Timing = 10 + m */
}

static void i386_ret_near32(void)        // Opcode 0xc3
{
	I.eip = POP32();
	CHANGE_PC(I.eip);
	CYCLES(CYCLES_RET);        /* TODO: Timing = 10 + m */
}

static void i386_sbb_rm32_r32(void)      // Opcode 0x19
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG32(modrm);
		dst = LOAD_RM32(modrm);
		dst = SBB32(dst, src, I.CF);
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG32(modrm);
		dst = READ32(ea);
		dst = SBB32(dst, src, I.CF);
		WRITE32(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_sbb_r32_rm32(void)      // Opcode 0x1b
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
		dst = LOAD_REG32(modrm);
		dst = SBB32(dst, src, I.CF);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
		dst = LOAD_REG32(modrm);
		dst = SBB32(dst, src, I.CF);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_sbb_eax_i32(void)       // Opcode 0x1d
{
	uint32_t src, dst;
	src = FETCH32();
	dst = REG32(EAX);
	dst = SBB32(dst, src, I.CF);
	REG32(EAX) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_scasd(void)             // Opcode 0xaf
{
	uint32_t eas, src, dst;
	eas = i386_translate(ES, I.address_size ? REG32(EDI) : REG16(DI), 0 );
	src = READ32(eas);
	dst = REG32(EAX);
	SUB32(dst, src);
	BUMP_DI(4);
	CYCLES(CYCLES_SCAS);
}

static void i386_shld32_i8(void)         // Opcode 0x0f a4
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t dst = LOAD_RM32(modrm);
		uint32_t upper = LOAD_REG32(modrm);
		uint8_t shift = FETCH();
		shift &= 31;
		if( shift == 0 ) {
		} else {
			I.CF = (dst & (1 << (32-shift))) ? 1 : 0;
			dst = (dst << shift) | (upper >> (32-shift));
			I.OF = I.CF ^ (dst >> 31);
			SetSZPF32(dst);
		}
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_SHLD_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		uint32_t dst = READ32(ea);
		uint32_t upper = LOAD_REG32(modrm);
		uint8_t shift = FETCH();
		shift &= 31;
		if( shift == 0 ) {
		} else {
			I.CF = (dst & (1 << (32-shift))) ? 1 : 0;
			dst = (dst << shift) | (upper >> (32-shift));
			I.OF = I.CF ^ (dst >> 31);
			SetSZPF32(dst);
		}
		WRITE32(ea, dst);
		CYCLES(CYCLES_SHLD_MEM);
	}
}

static void i386_shld32_cl(void)         // Opcode 0x0f a5
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t dst = LOAD_RM32(modrm);
		uint32_t upper = LOAD_REG32(modrm);
		uint8_t shift = REG8(CL);
		shift &= 31;
		if( shift == 0 ) {
		} else {
			I.CF = (dst & (1 << (32-shift))) ? 1 : 0;
			dst = (dst << shift) | (upper >> (32-shift));
			I.OF = I.CF ^ (dst >> 31);
			SetSZPF32(dst);
		}
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_SHLD_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		uint32_t dst = READ32(ea);
		uint32_t upper = LOAD_REG32(modrm);
		uint8_t shift = REG8(CL);
		shift &= 31;
		if( shift == 0 ) {
		} else {
			I.CF = (dst & (1 << (32-shift))) ? 1 : 0;
			dst = (dst << shift) | (upper >> (32-shift));
			I.OF = I.CF ^ (dst >> 31);
			SetSZPF32(dst);
		}
		WRITE32(ea, dst);
		CYCLES(CYCLES_SHLD_MEM);
	}
}

static void i386_shrd32_i8(void)         // Opcode 0x0f ac
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t dst = LOAD_RM32(modrm);
		uint32_t upper = LOAD_REG32(modrm);
		uint8_t shift = FETCH();
		shift &= 31;
		if( shift == 0 ) {
		} else {
			I.CF = (dst & (1 << (shift-1))) ? 1 : 0;
			dst = (dst >> shift) | (upper << (32-shift));
			I.OF = ((dst >> 31) ^ (dst >> 30)) & 1;
			SetSZPF32(dst);
		}
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_SHRD_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		uint32_t dst = READ32(ea);
		uint32_t upper = LOAD_REG32(modrm);
		uint8_t shift = FETCH();
		shift &= 31;
		if( shift == 0 ) {
		} else {
			I.CF = (dst & (1 << (shift-1))) ? 1 : 0;
			dst = (dst >> shift) | (upper << (32-shift));
			I.OF = ((dst >> 31) ^ (dst >> 30)) & 1;
			SetSZPF32(dst);
		}
		WRITE32(ea, dst);
		CYCLES(CYCLES_SHRD_MEM);
	}
}

static void i386_shrd32_cl(void)         // Opcode 0x0f ad
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t dst = LOAD_RM32(modrm);
		uint32_t upper = LOAD_REG32(modrm);
		uint8_t shift = REG8(CL);
		shift &= 31;
		if( shift == 0 ) {
		} else {
			I.CF = (dst & (1 << (shift-1))) ? 1 : 0;
			dst = (dst >> shift) | (upper << (32-shift));
			I.OF = ((dst >> 31) ^ (dst >> 30)) & 1;
			SetSZPF32(dst);
		}
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_SHRD_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		uint32_t dst = READ32(ea);
		uint32_t upper = LOAD_REG32(modrm);
		uint8_t shift = REG8(CL);
		shift &= 31;
		if( shift == 0 ) {
		} else {
			I.CF = (dst & (1 << (shift-1))) ? 1 : 0;
			dst = (dst >> shift) | (upper << (32-shift));
			I.OF = ((dst >> 31) ^ (dst >> 30)) & 1;
			SetSZPF32(dst);
		}
		WRITE32(ea, dst);
		CYCLES(CYCLES_SHRD_MEM);
	}
}

static void i386_stosd(void)             // Opcode 0xab
{
	uint32_t eas = i386_translate(ES, I.address_size ? REG32(EDI) : REG16(DI), 1 );
	WRITE32(eas, REG32(EAX));
	BUMP_DI(4);
	CYCLES(CYCLES_STOS);
}

static void i386_sub_rm32_r32(void)      // Opcode 0x29
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG32(modrm);
		dst = LOAD_RM32(modrm);
		dst = SUB32(dst, src);
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG32(modrm);
		dst = READ32(ea);
		dst = SUB32(dst, src);
		WRITE32(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_sub_r32_rm32(void)      // Opcode 0x2b
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
		dst = LOAD_REG32(modrm);
		dst = SUB32(dst, src);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
		dst = LOAD_REG32(modrm);
		dst = SUB32(dst, src);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_sub_eax_i32(void)       // Opcode 0x2d
{
	uint32_t src, dst;
	src = FETCH32();
	dst = REG32(EAX);
	dst = SUB32(dst, src);
	REG32(EAX) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}

static void i386_test_eax_i32(void)      // Opcode 0xa9
{
	uint32_t src = FETCH32();
	uint32_t dst = REG32(EAX);
	dst = src & dst;
	SetSZPF32(dst);
	I.CF = 0;
	I.OF = 0;
	CYCLES(CYCLES_TEST_IMM_ACC);
}

static void i386_test_rm32_r32(void)     // Opcode 0x85
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG32(modrm);
		dst = LOAD_RM32(modrm);
		dst = src & dst;
		SetSZPF32(dst);
		I.CF = 0;
		I.OF = 0;
		CYCLES(CYCLES_TEST_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = LOAD_REG32(modrm);
		dst = READ32(ea);
		dst = src & dst;
		SetSZPF32(dst);
		I.CF = 0;
		I.OF = 0;
		CYCLES(CYCLES_TEST_REG_MEM);
	}
}

static void i386_xchg_eax_ecx(void)      // Opcode 0x91
{
	uint32_t temp;
	temp = REG32(EAX);
	REG32(EAX) = REG32(ECX);
	REG32(ECX) = temp;
	CYCLES(CYCLES_XCHG_REG_REG);
}

static void i386_xchg_eax_edx(void)      // Opcode 0x92
{
	uint32_t temp;
	temp = REG32(EAX);
	REG32(EAX) = REG32(EDX);
	REG32(EDX) = temp;
	CYCLES(CYCLES_XCHG_REG_REG);
}

static void i386_xchg_eax_ebx(void)      // Opcode 0x93
{
	uint32_t temp;
	temp = REG32(EAX);
	REG32(EAX) = REG32(EBX);
	REG32(EBX) = temp;
	CYCLES(CYCLES_XCHG_REG_REG);
}

static void i386_xchg_eax_esp(void)      // Opcode 0x94
{
	uint32_t temp;
	temp = REG32(EAX);
	REG32(EAX) = REG32(ESP);
	REG32(ESP) = temp;
	CYCLES(CYCLES_XCHG_REG_REG);
}

static void i386_xchg_eax_ebp(void)      // Opcode 0x95
{
	uint32_t temp;
	temp = REG32(EAX);
	REG32(EAX) = REG32(EBP);
	REG32(EBP) = temp;
	CYCLES(CYCLES_XCHG_REG_REG);
}

static void i386_xchg_eax_esi(void)      // Opcode 0x96
{
	uint32_t temp;
	temp = REG32(EAX);
	REG32(EAX) = REG32(ESI);
	REG32(ESI) = temp;
	CYCLES(CYCLES_XCHG_REG_REG);
}

static void i386_xchg_eax_edi(void)      // Opcode 0x97
{
	uint32_t temp;
	temp = REG32(EAX);
	REG32(EAX) = REG32(EDI);
	REG32(EDI) = temp;
	CYCLES(CYCLES_XCHG_REG_REG);
}

static void i386_xchg_r32_rm32(void)     // Opcode 0x87
{
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		uint32_t src = LOAD_RM32(modrm);
		uint32_t dst = LOAD_REG32(modrm);
		STORE_REG32(modrm, src);
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_XCHG_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		uint32_t src = READ32(ea);
		uint32_t dst = LOAD_REG32(modrm);
		WRITE32(ea, dst);
		STORE_REG32(modrm, src);
		CYCLES(CYCLES_XCHG_REG_MEM);
	}
}

static void i386_xor_rm32_r32(void)      // Opcode 0x31
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_REG32(modrm);
		dst = LOAD_RM32(modrm);
		dst = XOR32(dst, src);
		STORE_RM32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,1);
		src = LOAD_REG32(modrm);
		dst = READ32(ea);
		dst = XOR32(dst, src);
		WRITE32(ea, dst);
		CYCLES(CYCLES_ALU_REG_MEM);
	}
}

static void i386_xor_r32_rm32(void)      // Opcode 0x33
{
	uint32_t src, dst;
	uint8_t modrm = FETCH();
	if( modrm >= 0xc0 ) {
		src = LOAD_RM32(modrm);
		dst = LOAD_REG32(modrm);
		dst = XOR32(dst, src);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_REG_REG);
	} else {
		uint32_t ea = GetEA(modrm,0);
		src = READ32(ea);
		dst = LOAD_REG32(modrm);
		dst = XOR32(dst, src);
		STORE_REG32(modrm, dst);
		CYCLES(CYCLES_ALU_MEM_REG);
	}
}

static void i386_xor_eax_i32(void)       // Opcode 0x35
{
	uint32_t src, dst;
	src = FETCH32();
	dst = REG32(EAX);
	dst = XOR32(dst, src);
	REG32(EAX) = dst;
	CYCLES(CYCLES_ALU_IMM_ACC);
}



static void i386_group81_32(void)        // Opcode 0x81
{
	uint32_t ea;
	uint32_t src, dst;
	uint8_t modrm = FETCH();

	switch( (modrm >> 3) & 0x7 )
	{
		case 0:     // ADD Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = FETCH32();
				dst = ADD32(dst, src);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = FETCH32();
				dst = ADD32(dst, src);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 1:     // OR Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = FETCH32();
				dst = OR32(dst, src);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = FETCH32();
				dst = OR32(dst, src);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 2:     // ADC Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = FETCH32();
				dst = ADC32(dst, src, I.CF);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = FETCH32();
				dst = ADC32(dst, src, I.CF);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 3:     // SBB Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = FETCH32();
				dst = SBB32(dst, src, I.CF);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = FETCH32();
				dst = SBB32(dst, src, I.CF);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 4:     // AND Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = FETCH32();
				dst = AND32(dst, src);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = FETCH32();
				dst = AND32(dst, src);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 5:     // SUB Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = FETCH32();
				dst = SUB32(dst, src);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = FETCH32();
				dst = SUB32(dst, src);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 6:     // XOR Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = FETCH32();
				dst = XOR32(dst, src);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = FETCH32();
				dst = XOR32(dst, src);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 7:     // CMP Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = FETCH32();
				SUB32(dst, src);
				CYCLES(CYCLES_CMP_REG_REG);
			} else {
				ea = GetEA(modrm,0);
				dst = READ32(ea);
				src = FETCH32();
				SUB32(dst, src);
				CYCLES(CYCLES_CMP_REG_MEM);
			}
			break;
	}
}

static void i386_group83_32(void)        // Opcode 0x83
{
	uint32_t ea;
	uint32_t src, dst;
	uint8_t modrm = FETCH();

	switch( (modrm >> 3) & 0x7 )
	{
		case 0:     // ADD Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = ADD32(dst, src);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = ADD32(dst, src);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 1:     // OR Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = OR32(dst, src);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = OR32(dst, src);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 2:     // ADC Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = ADC32(dst, src, I.CF);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = ADC32(dst, src, I.CF);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 3:     // SBB Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = ((uint32_t)(int32_t)(int8_t)FETCH());
				dst = SBB32(dst, src, I.CF);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = ((uint32_t)(int32_t)(int8_t)FETCH());
				dst = SBB32(dst, src, I.CF);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 4:     // AND Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = AND32(dst, src);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = AND32(dst, src);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 5:     // SUB Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = SUB32(dst, src);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = SUB32(dst, src);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 6:     // XOR Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = XOR32(dst, src);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_ALU_REG_REG);
			} else {
				ea = GetEA(modrm,1);
				dst = READ32(ea);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				dst = XOR32(dst, src);
				WRITE32(ea, dst);
				CYCLES(CYCLES_ALU_REG_MEM);
			}
			break;
		case 7:     // CMP Rm32, i32
			if( modrm >= 0xc0 ) {
				dst = LOAD_RM32(modrm);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				SUB32(dst, src);
				CYCLES(CYCLES_CMP_REG_REG);
			} else {
				ea = GetEA(modrm,0);
				dst = READ32(ea);
				src = (uint32_t)(int32_t)(int8_t)FETCH();
				SUB32(dst, src);
				CYCLES(CYCLES_CMP_REG_MEM);
			}
			break;
	}
}

static void i386_groupC1_32(void)        // Opcode 0xc1
{
	uint32_t dst;
	uint8_t modrm = FETCH();
	uint8_t shift;

	if( modrm >= 0xc0 ) {
		dst = LOAD_RM32(modrm);
		shift = FETCH() & 0x1f;
		dst = i386_shift_rotate32(modrm, dst, shift);
		STORE_RM32(modrm, dst);
	} else {
		uint32_t ea = GetEA(modrm,1);
		dst = READ32(ea);
		shift = FETCH() & 0x1f;
		dst = i386_shift_rotate32(modrm, dst, shift);
		WRITE32(ea, dst);
	}
}

static void i386_groupD1_32(void)        // Opcode 0xd1
{
	uint32_t dst;
	uint8_t modrm = FETCH();

	if( modrm >= 0xc0 ) {
		dst = LOAD_RM32(modrm);
		dst = i386_shift_rotate32(modrm, dst, 1);
		STORE_RM32(modrm, dst);
	} else {
		uint32_t ea = GetEA(modrm,1);
		dst = READ32(ea);
		dst = i386_shift_rotate32(modrm, dst, 1);
		WRITE32(ea, dst);
	}
}

static void i386_groupD3_32(void)        // Opcode 0xd3
{
	uint32_t dst;
	uint8_t modrm = FETCH();

	if( modrm >= 0xc0 ) {
		dst = LOAD_RM32(modrm);
		dst = i386_shift_rotate32(modrm, dst, REG8(CL));
		STORE_RM32(modrm, dst);
	} else {
		uint32_t ea = GetEA(modrm,1);
		dst = READ32(ea);
		dst = i386_shift_rotate32(modrm, dst, REG8(CL));
		WRITE32(ea, dst);
	}
}

static void i386_groupF7_32(void)        // Opcode 0xf7
{
	uint8_t modrm = FETCH();

	switch( (modrm >> 3) & 0x7 )
	{
		case 0:         /* TEST Rm32, i32 */
			if( modrm >= 0xc0 ) {
				uint32_t dst = LOAD_RM32(modrm);
				uint32_t src = FETCH32();
				dst &= src;
				I.CF = I.OF = I.AF = 0;
				SetSZPF32(dst);
				CYCLES(CYCLES_TEST_IMM_REG);
			} else {
				uint32_t ea = GetEA(modrm,0);
				uint32_t dst = READ32(ea);
				uint32_t src = FETCH32();
				dst &= src;
				I.CF = I.OF = I.AF = 0;
				SetSZPF32(dst);
				CYCLES(CYCLES_TEST_IMM_MEM);
			}
			break;
		case 2:         /* NOT Rm32 */
			if( modrm >= 0xc0 ) {
				uint32_t dst = LOAD_RM32(modrm);
				dst = ~dst;
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_NOT_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint32_t dst = READ32(ea);
				dst = ~dst;
				WRITE32(ea, dst);
				CYCLES(CYCLES_NOT_MEM);
			}
			break;
		case 3:         /* NEG Rm32 */
			if( modrm >= 0xc0 ) {
				uint32_t dst = LOAD_RM32(modrm);
				dst = SUB32(0, dst );
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_NEG_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint32_t dst = READ32(ea);
				dst = SUB32(0, dst );
				WRITE32(ea, dst);
				CYCLES(CYCLES_NEG_MEM);
			}
			break;
		case 4:         /* MUL EAX, Rm32 */
			{
				uint64_t result;
				uint32_t src, dst;
				if( modrm >= 0xc0 ) {
					src = LOAD_RM32(modrm);
					CYCLES(CYCLES_MUL32_ACC_REG);      /* TODO: Correct multiply timing */
				} else {
					uint32_t ea = GetEA(modrm,0);
					src = READ32(ea);
					CYCLES(CYCLES_MUL32_ACC_MEM);      /* TODO: Correct multiply timing */
				}

				dst = REG32(EAX);
				result = (uint64_t)src * (uint64_t)dst;
				REG32(EDX) = (uint32_t)(result >> 32);
				REG32(EAX) = (uint32_t)result;

				I.CF = I.OF = (REG32(EDX) != 0);
			}
			break;
		case 5:         /* IMUL EAX, Rm32 */
			{
				int64_t result;
				int64_t src, dst;
				if( modrm >= 0xc0 ) {
					src = (int64_t)(int32_t)LOAD_RM32(modrm);
					CYCLES(CYCLES_IMUL32_ACC_REG);     /* TODO: Correct multiply timing */
				} else {
					uint32_t ea = GetEA(modrm,0);
					src = (int64_t)(int32_t)READ32(ea);
					CYCLES(CYCLES_IMUL32_ACC_MEM);     /* TODO: Correct multiply timing */
				}

				dst = (int64_t)(int32_t)REG32(EAX);
				result = src * dst;

				REG32(EDX) = (uint32_t)(result >> 32);
				REG32(EAX) = (uint32_t)result;

				I.CF = I.OF = !(result == (int64_t)(int32_t)result);
			}
			break;
		case 6:         /* DIV EAX, Rm32 */
			{
				uint64_t quotient, remainder, result;
				uint32_t src;
				if( modrm >= 0xc0 ) {
					src = LOAD_RM32(modrm);
					CYCLES(CYCLES_DIV32_ACC_REG);
				} else {
					uint32_t ea = GetEA(modrm,0);
					src = READ32(ea);
					CYCLES(CYCLES_DIV32_ACC_MEM);
				}

				quotient = ((uint64_t)(REG32(EDX)) << 32) | (uint64_t)(REG32(EAX));
				if( src ) {
					remainder = quotient % (uint64_t)src;
					result = quotient / (uint64_t)src;
					if( result > 0xffffffffULL ) {
						/* TODO: Divide error */
					} else {
						REG32(EDX) = (uint32_t)remainder;
						REG32(EAX) = (uint32_t)result;
					}
				} else {
					i386_trap(0, 0);
				}
			}
			break;
		case 7:         /* IDIV EAX, Rm32 */
			{
				int64_t quotient, remainder, result;
				uint32_t src;
				if( modrm >= 0xc0 ) {
					src = LOAD_RM32(modrm);
					CYCLES(CYCLES_IDIV32_ACC_REG);
				} else {
					uint32_t ea = GetEA(modrm,0);
					src = READ32(ea);
					CYCLES(CYCLES_IDIV32_ACC_MEM);
				}

				quotient = (((int64_t)REG32(EDX)) << 32) | ((uint64_t)REG32(EAX));
				if( src ) {
					remainder = quotient % (int64_t)(int32_t)src;
					result = quotient / (int64_t)(int32_t)src;
					if( result > 0x7fffffffLL || result < -0x80000000LL ) {
						/* TODO: Divide error */
					} else {
						REG32(EDX) = (uint32_t)remainder;
						REG32(EAX) = (uint32_t)result;
					}
				} else {
					i386_trap(0, 0);
				}
			}
			break;
	}
}

static void i386_groupFF_32(void)        // Opcode 0xff
{
	uint8_t modrm = FETCH();

	switch( (modrm >> 3) & 0x7 )
	{
		case 0:         /* INC Rm32 */
			if( modrm >= 0xc0 ) {
				uint32_t dst = LOAD_RM32(modrm);
				dst = INC32(dst);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_INC_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint32_t dst = READ32(ea);
				dst = INC32(dst);
				WRITE32(ea, dst);
				CYCLES(CYCLES_INC_MEM);
			}
			break;
		case 1:         /* DEC Rm32 */
			if( modrm >= 0xc0 ) {
				uint32_t dst = LOAD_RM32(modrm);
				dst = DEC32(dst);
				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_DEC_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint32_t dst = READ32(ea);
				dst = DEC32(dst);
				WRITE32(ea, dst);
				CYCLES(CYCLES_DEC_MEM);
			}
			break;
		case 2:         /* CALL Rm32 */
			{
				uint32_t address;
				if( modrm >= 0xc0 ) {
					address = LOAD_RM32(modrm);
					CYCLES(CYCLES_CALL_REG);       /* TODO: Timing = 7 + m */
				} else {
					uint32_t ea = GetEA(modrm,0);
					address = READ32(ea);
					CYCLES(CYCLES_CALL_MEM);       /* TODO: Timing = 10 + m */
				}
				PUSH32(I.eip );
				I.eip = address;
				CHANGE_PC(I.eip);
			}
			break;
		case 3:         /* CALL FAR Rm32 */
			{
				uint16_t selector;
				uint32_t address;

				if( modrm >= 0xc0 )
				{
					report_invalid_modrm("groupFF_32", modrm);
				}
				else
				{
					uint32_t ea = GetEA(modrm,0);
					address = READ32(ea + 0);
					selector = READ16(ea + 4);
					CYCLES(CYCLES_CALL_MEM_INTERSEG);      /* TODO: Timing = 10 + m */
					if(PROTECTED_MODE && !V8086_MODE)
					{
						i386_protected_mode_call(selector,address,1,1);
					}
					else
					{
						PUSH32SEG(I.sreg[CS].selector );
						PUSH32(I.eip );
						I.sreg[CS].selector = selector;
						I.performed_intersegment_jump = 1;
						i386_load_segment_descriptor(CS );
						I.eip = address;
						CHANGE_PC(I.eip);
					}
				}
			}
			break;
		case 4:         /* JMP Rm32 */
			{
				uint32_t address;
				if( modrm >= 0xc0 ) {
					address = LOAD_RM32(modrm);
					CYCLES(CYCLES_JMP_REG);        /* TODO: Timing = 7 + m */
				} else {
					uint32_t ea = GetEA(modrm,0);
					address = READ32(ea);
					CYCLES(CYCLES_JMP_MEM);        /* TODO: Timing = 10 + m */
				}
				I.eip = address;
				CHANGE_PC(I.eip);
			}
			break;
		case 5:         /* JMP FAR Rm32 */
			{
				uint16_t selector;
				uint32_t address;

				if( modrm >= 0xc0 )
				{
					report_invalid_modrm("groupFF_32", modrm);
				}
				else
				{
					uint32_t ea = GetEA(modrm,0);
					address = READ32(ea + 0);
					selector = READ16(ea + 4);
					CYCLES(CYCLES_JMP_MEM_INTERSEG);       /* TODO: Timing = 10 + m */
					if(PROTECTED_MODE && !V8086_MODE)
					{
						i386_protected_mode_jump(selector,address,1,1);
					}
					else
					{
						I.sreg[CS].selector = selector;
						I.performed_intersegment_jump = 1;
						i386_load_segment_descriptor(CS );
						I.eip = address;
						CHANGE_PC(I.eip);
					}
				}
			}
			break;
		case 6:         /* PUSH Rm32 */
			{
				uint32_t value;
				if( modrm >= 0xc0 ) {
					value = LOAD_RM32(modrm);
				} else {
					uint32_t ea = GetEA(modrm,0);
					value = READ32(ea);
				}
				PUSH32(value);
				CYCLES(CYCLES_PUSH_RM);
			}
			break;
		default:
			report_invalid_modrm("groupFF_32", modrm);
			break;
	}
}

static void i386_group0F00_32(void)          // Opcode 0x0f 00
{
	uint32_t address, ea;
	uint8_t modrm = FETCH();
	I386_SREG seg;
	uint8_t result;

	switch( (modrm >> 3) & 0x7 )
	{
		case 0:         /* SLDT */
			if ( PROTECTED_MODE && !V8086_MODE )
			{
				if( modrm >= 0xc0 ) {
					STORE_RM32(modrm, I.ldtr.segment);
					CYCLES(CYCLES_SLDT_REG);
				} else {
					ea = GetEA(modrm,1);
					WRITE16(ea, I.ldtr.segment);
					CYCLES(CYCLES_SLDT_MEM);
				}
			}
			else
			{
				i386_trap(6, 0);
			}
			break;
		case 1:         /* STR */
			if ( PROTECTED_MODE && !V8086_MODE )
			{
				if( modrm >= 0xc0 ) {
					STORE_RM32(modrm, I.task.segment);
					CYCLES(CYCLES_STR_REG);
				} else {
					ea = GetEA(modrm,1);
					WRITE16(ea, I.task.segment);
					CYCLES(CYCLES_STR_MEM);
				}
			}
			else
			{
				i386_trap(6, 0);
			}
			break;
		case 2:         /* LLDT */
			if ( PROTECTED_MODE && !V8086_MODE )
			{
				if(I.CPL)
					FAULT(FAULT_GP,0)
				if( modrm >= 0xc0 ) {
					address = LOAD_RM32(modrm);
					I.ldtr.segment = address;
					CYCLES(CYCLES_LLDT_REG);
				} else {
					ea = GetEA(modrm,0);
					I.ldtr.segment = READ32(ea);
					CYCLES(CYCLES_LLDT_MEM);
				}
				memset(&seg, 0, sizeof(seg));
				seg.selector = I.ldtr.segment;
				i386_load_protected_mode_segment(&seg,NULL);
				I.ldtr.limit = seg.limit;
				I.ldtr.base = seg.base;
				I.ldtr.flags = seg.flags;
			}
			else
			{
				i386_trap(6, 0);
			}
			break;

		case 3:         /* LTR */
			if ( PROTECTED_MODE && !V8086_MODE )
			{
				if(I.CPL)
					FAULT(FAULT_GP,0)
				if( modrm >= 0xc0 ) {
					address = LOAD_RM32(modrm);
					I.task.segment = address;
					CYCLES(CYCLES_LTR_REG);
				} else {
					ea = GetEA(modrm,0);
					I.task.segment = READ32(ea);
					CYCLES(CYCLES_LTR_MEM);
				}
				memset(&seg, 0, sizeof(seg));
				seg.selector = I.task.segment;
				i386_load_protected_mode_segment(&seg,NULL);

				offs_t addr = ((seg.selector & 4) ? I.ldtr.base : I.gdtr.base) + (seg.selector & ~7) + 5;
				i386_translate_address(TR_READ, false, &addr, NULL);
				prog_w8(addr, (seg.flags & 0xff) | 2);

				I.task.limit = seg.limit;
				I.task.base = seg.base;
				I.task.flags = seg.flags | 2;
			}
			else
			{
				i386_trap(6, 0);
			}
			break;

		case 4:  /* VERR */
			if ( PROTECTED_MODE && !V8086_MODE )
			{
				if( modrm >= 0xc0 ) {
					address = LOAD_RM32(modrm);
					CYCLES(CYCLES_VERR_REG);
				} else {
					ea = GetEA(modrm,0);
					address = READ32(ea);
					CYCLES(CYCLES_VERR_MEM);
				}
				memset(&seg, 0, sizeof(seg));
				seg.selector = address;
				result = i386_load_protected_mode_segment(&seg,NULL);
				// check if the segment is a code or data segment (not a special segment type, like a TSS, gate, LDT...)
				if(!(seg.flags & 0x10))
					result = 0;
				// check that the segment is readable
				if(seg.flags & 0x10)  // is code or data segment
				{
					if(seg.flags & 0x08)  // is code segment, so check if it's readable
					{
						if(!(seg.flags & 0x02))
						{
							result = 0;
						}
						else
						{  // check if conforming, these are always readable, regardless of privilege
							if(!(seg.flags & 0x04))
							{
								// if not conforming, then we must check privilege levels
								if(((seg.flags >> 5) & 0x03) < I386_MAX(I.CPL, (uint8_t)(address & 0x03)))
									result = 0;
							}
						}
					}
					else
					{
						if(((seg.flags >> 5) & 0x03) < I386_MAX(I.CPL, (uint8_t)(address & 0x03)))
							result = 0;
					}
				}
				SetZF(result);
			}
			else
			{
				i386_trap(6, 0);
				LOGMASKED(LOG_PM_EVENTS, "i386: VERR: Exception - Running in real mode or virtual 8086 mode.\n");
			}
			break;

		case 5:  /* VERW */
			if ( PROTECTED_MODE && !V8086_MODE )
			{
				if( modrm >= 0xc0 ) {
					address = LOAD_RM16(modrm);
					CYCLES(CYCLES_VERW_REG);
				} else {
					ea = GetEA(modrm,0);
					address = READ16(ea);
					CYCLES(CYCLES_VERW_MEM);
				}
				memset(&seg, 0, sizeof(seg));
				seg.selector = address;
				result = i386_load_protected_mode_segment(&seg,NULL);
				// check if the segment is a code or data segment (not a special segment type, like a TSS, gate, LDT...)
				if(!(seg.flags & 0x10))
					result = 0;
				// check that the segment is writable
				if(seg.flags & 0x10)  // is code or data segment
				{
					if(seg.flags & 0x08)  // is code segment (and thus, not writable)
					{
						result = 0;
					}
					else
					{  // is data segment
						if(!(seg.flags & 0x02))
							result = 0;
					}
				}
				if(((seg.flags >> 5) & 0x03) < I386_MAX(I.CPL, (uint8_t)(address & 0x03)))
					result = 0;
				SetZF(result);
			}
			else
			{
				i386_trap(6, 0);
				LOGMASKED(LOG_PM_EVENTS, "i386: VERW: Exception - Running in real mode or virtual 8086 mode.\n");
			}
			break;

		default:
			report_invalid_modrm("group0F00_32", modrm);
			break;
	}
}

static void i386_group0F01_32(void)      // Opcode 0x0f 01
{
	uint8_t modrm = FETCH();
	uint32_t address, ea;

	switch( (modrm >> 3) & 0x7 )
	{
		case 0:         /* SGDT */
			{
				if( modrm >= 0xc0 ) {
					address = LOAD_RM32(modrm);
					ea = i386_translate(CS, address, 1 );
				} else {
					ea = GetEA(modrm,1);
				}
				WRITE16(ea, I.gdtr.limit);
				WRITE32(ea + 2, I.gdtr.base);
				CYCLES(CYCLES_SGDT);
				break;
			}
		case 1:         /* SIDT */
			{
				if (modrm >= 0xc0)
				{
					address = LOAD_RM32(modrm);
					ea = i386_translate(CS, address, 1 );
				}
				else
				{
					ea = GetEA(modrm,1);
				}
				WRITE16(ea, I.idtr.limit);
				WRITE32(ea + 2, I.idtr.base);
				CYCLES(CYCLES_SIDT);
				break;
			}
		case 2:         /* LGDT */
			{
				if(PROTECTED_MODE && I.CPL)
					FAULT(FAULT_GP,0)
				if( modrm >= 0xc0 ) {
					address = LOAD_RM32(modrm);
					ea = i386_translate(CS, address, 0 );
				} else {
					ea = GetEA(modrm,0);
				}
				I.gdtr.limit = READ16(ea);
				I.gdtr.base = READ32(ea + 2);
				CYCLES(CYCLES_LGDT);
				break;
			}
		case 3:         /* LIDT */
			{
				if(PROTECTED_MODE && I.CPL)
					FAULT(FAULT_GP,0)
				if( modrm >= 0xc0 ) {
					address = LOAD_RM32(modrm);
					ea = i386_translate(CS, address, 0 );
				} else {
					ea = GetEA(modrm,0);
				}
				I.idtr.limit = READ16(ea);
				I.idtr.base = READ32(ea + 2);
				CYCLES(CYCLES_LIDT);
				break;
			}
		case 4:         /* SMSW */
			{
				if( modrm >= 0xc0 ) {
					// smsw stores all of cr0 into register
					STORE_RM32(modrm, I.cr[0]);
					CYCLES(CYCLES_SMSW_REG);
				} else {
					/* always 16-bit memory operand */
					ea = GetEA(modrm,1);
					WRITE16(ea, I.cr[0]);
					CYCLES(CYCLES_SMSW_MEM);
				}
				break;
			}
		case 6:         /* LMSW */
			{
				if(PROTECTED_MODE && I.CPL)
					FAULT(FAULT_GP,0)
				uint16_t b;
				if( modrm >= 0xc0 ) {
					b = LOAD_RM16(modrm);
					CYCLES(CYCLES_LMSW_REG);
				} else {
					ea = GetEA(modrm,0);
					CYCLES(CYCLES_LMSW_MEM);
				b = READ16(ea);
				}
				if(PROTECTED_MODE)
					b |= 0x0001;  // cannot return to real mode using this instruction.
				I.cr[0] &= ~0x0000000f;
				I.cr[0] |= b & 0x0000000f;
				break;
			}
		default:
			report_invalid_modrm("group0F01_32", modrm);
			break;
	}
}

static void i386_group0FBA_32(void)      // Opcode 0x0f ba
{
	uint8_t modrm = FETCH();

	switch( (modrm >> 3) & 0x7 )
	{
		case 4:         /* BT Rm32, i8 */
			if( modrm >= 0xc0 ) {
				uint32_t dst = LOAD_RM32(modrm);
				uint8_t bit = FETCH();

				if( dst & (1 << bit) )
					I.CF = 1;
				else
					I.CF = 0;

				CYCLES(CYCLES_BT_IMM_REG);
			} else {
				uint32_t ea = GetEA(modrm,0);
				uint32_t dst = READ32(ea);
				uint8_t bit = FETCH();

				if( dst & (1 << bit) )
					I.CF = 1;
				else
					I.CF = 0;

				CYCLES(CYCLES_BT_IMM_MEM);
			}
			break;
		case 5:         /* BTS Rm32, i8 */
			if( modrm >= 0xc0 ) {
				uint32_t dst = LOAD_RM32(modrm);
				uint8_t bit = FETCH();

				if( dst & (1 << bit) )
					I.CF = 1;
				else
					I.CF = 0;
				dst |= (1 << bit);

				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_BTS_IMM_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint32_t dst = READ32(ea);
				uint8_t bit = FETCH();

				if( dst & (1 << bit) )
					I.CF = 1;
				else
					I.CF = 0;
				dst |= (1 << bit);

				WRITE32(ea, dst);
				CYCLES(CYCLES_BTS_IMM_MEM);
			}
			break;
		case 6:         /* BTR Rm32, i8 */
			if( modrm >= 0xc0 ) {
				uint32_t dst = LOAD_RM32(modrm);
				uint8_t bit = FETCH();

				if( dst & (1 << bit) )
					I.CF = 1;
				else
					I.CF = 0;
				dst &= ~(1 << bit);

				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_BTR_IMM_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint32_t dst = READ32(ea);
				uint8_t bit = FETCH();

				if( dst & (1 << bit) )
					I.CF = 1;
				else
					I.CF = 0;
				dst &= ~(1 << bit);

				WRITE32(ea, dst);
				CYCLES(CYCLES_BTR_IMM_MEM);
			}
			break;
		case 7:         /* BTC Rm32, i8 */
			if( modrm >= 0xc0 ) {
				uint32_t dst = LOAD_RM32(modrm);
				uint8_t bit = FETCH();

				if( dst & (1 << bit) )
					I.CF = 1;
				else
					I.CF = 0;
				dst ^= (1 << bit);

				STORE_RM32(modrm, dst);
				CYCLES(CYCLES_BTC_IMM_REG);
			} else {
				uint32_t ea = GetEA(modrm,1);
				uint32_t dst = READ32(ea);
				uint8_t bit = FETCH();

				if( dst & (1 << bit) )
					I.CF = 1;
				else
					I.CF = 0;
				dst ^= (1 << bit);

				WRITE32(ea, dst);
				CYCLES(CYCLES_BTC_IMM_MEM);
			}
			break;
		default:
			report_invalid_modrm("group0FBA_32", modrm);
			break;
	}
}

static void i386_lar_r32_rm32(void)  // Opcode 0x0f 0x02
{
	uint8_t modrm = FETCH();
	I386_SREG seg;
	uint8_t type;

	if(PROTECTED_MODE && !V8086_MODE)
	{
		memset(&seg,0,sizeof(seg));
		if(modrm >= 0xc0)
		{
			seg.selector = LOAD_RM32(modrm);
			CYCLES(CYCLES_LAR_REG);
		}
		else
		{
			uint32_t ea = GetEA(modrm,0);
			seg.selector = READ32(ea);
			CYCLES(CYCLES_LAR_MEM);
		}
		if(seg.selector == 0)
		{
			SetZF(0);  // not a valid segment
		}
		else
		{
			uint64_t desc;
			if(!i386_load_protected_mode_segment(&seg,&desc))
			{
				SetZF(0);
				return;
			}
			uint8_t DPL = (seg.flags >> 5) & 3;
			if(((DPL < I.CPL) || (DPL < (seg.selector & 3))) && ((seg.flags & 0x1c) != 0x1c))
			{
				SetZF(0);
				return;
			}
			if(!(seg.flags & 0x10))  // special segment
			{
				// check for invalid segment types
				type = seg.flags & 0x000f;
				if(type == 0x00 || type == 0x08 || type == 0x0a || type == 0x0d)
				{
					SetZF(0);  // invalid segment type
				}
				else
				{
					STORE_REG32(modrm,(desc>>32) & 0x00ffff00);
					SetZF(1);
				}
			}
			else
			{
				STORE_REG32(modrm,(desc>>32) & 0x00ffff00);
				SetZF(1);
			}
		}
	}
	else
	{
		// illegal opcode
		i386_trap(6,0);
		LOGMASKED(LOG_PM_EVENTS, "i386: LAR: Exception - running in real mode or virtual 8086 mode.\n");
	}
}

static void i386_lsl_r32_rm32(void)  // Opcode 0x0f 0x03
{
	uint8_t modrm = FETCH();
	uint32_t limit;
	I386_SREG seg;

	if(PROTECTED_MODE && !V8086_MODE)
	{
		memset(&seg, 0, sizeof(seg));
		if(modrm >= 0xc0)
		{
			seg.selector = LOAD_RM32(modrm);
		}
		else
		{
			uint32_t ea = GetEA(modrm,0);
			seg.selector = READ32(ea);
		}
		if(seg.selector == 0)
		{
			SetZF(0);  // not a valid segment
		}
		else
		{
			uint8_t type;
			if(!i386_load_protected_mode_segment(&seg,NULL))
			{
				SetZF(0);
				return;
			}
			uint8_t DPL = (seg.flags >> 5) & 3;
			if(((DPL < I.CPL) || (DPL < (seg.selector & 3))) && ((seg.flags & 0x1c) != 0x1c))
			{
				SetZF(0);
				return;
			}
			type = seg.flags & 0x1f;
			switch(type)
			{
			case 0:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 10:
			case 12:
			case 13:
			case 14:
			case 15:
				SetZF(0);
				return;
			default:
				limit = seg.limit;
				STORE_REG32(modrm,limit);
				SetZF(1);
			}
		}
	}
	else
		i386_trap(6, 0);
}

static void i386_bound_r32_m32_m32(void) // Opcode 0x62
{
	uint8_t modrm;
	int32_t val, low, high;

	modrm = FETCH();

	if (modrm >= 0xc0)
	{
		low = high = LOAD_RM32(modrm);
	}
	else
	{
		uint32_t ea = GetEA(modrm,0);
		low = READ32(ea + 0);
		high = READ32(ea + 4);
	}
	val = LOAD_REG32(modrm);

	if ((val < low) || (val > high))
	{
		CYCLES(CYCLES_BOUND_OUT_RANGE);
		i386_trap(5, 0);
	}
	else
	{
		CYCLES(CYCLES_BOUND_IN_RANGE);
	}
}

static void i386_retf32(void)            // Opcode 0xcb
{
	if(PROTECTED_MODE && !V8086_MODE)
	{
		i386_protected_mode_retf(0,1);
	}
	else
	{
		I.eip = POP32();
		I.sreg[CS].selector = POP32();
		i386_load_segment_descriptor(CS );
		CHANGE_PC(I.eip);
	}

	CYCLES(CYCLES_RET_INTERSEG);
}

static void i386_retf_i32(void)          // Opcode 0xca
{
	uint16_t count = FETCH16();

	if(PROTECTED_MODE && !V8086_MODE)
	{
		i386_protected_mode_retf(count,1);
	}
	else
	{
		I.eip = POP32();
		I.sreg[CS].selector = POP32();
		i386_load_segment_descriptor(CS );
		CHANGE_PC(I.eip);
		REG32(ESP) += count;
	}

	CYCLES(CYCLES_RET_IMM_INTERSEG);
}

static bool i386_load_far_pointer32(int s)
{
	uint8_t modrm = FETCH();
	uint16_t selector;
	bool fault = false;

	if( modrm >= 0xc0 ) {
		report_invalid_modrm("load_far_pointer32", modrm);
		return false;
	} else {
		uint32_t ea = GetEA(modrm,0);
		uint32_t val = READ32(ea + 0);
		selector = READ16(ea + 4);
		i386_sreg_load(selector,s,&fault);
		if(!fault)
			STORE_REG32(modrm, val);
	}
	return !fault;
}

static void i386_lds32(void)             // Opcode 0xc5
{
	if(i386_load_far_pointer32(DS))
		CYCLES(CYCLES_LDS);
}

static void i386_lss32(void)             // Opcode 0x0f 0xb2
{
	if(i386_load_far_pointer32(SS))
		CYCLES(CYCLES_LSS);
}

static void i386_les32(void)             // Opcode 0xc4
{
	if(i386_load_far_pointer32(ES))
		CYCLES(CYCLES_LES);
}

static void i386_lfs32(void)             // Opcode 0x0f 0xb4
{
	if(i386_load_far_pointer32(FS))
		CYCLES(CYCLES_LFS);
}

static void i386_lgs32(void)             // Opcode 0x0f 0xb5
{
	if(i386_load_far_pointer32(GS))
		CYCLES(CYCLES_LGS);
}
