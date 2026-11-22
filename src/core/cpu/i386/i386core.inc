// license:BSD-3-Clause
// copyright-holders:Ville Linde, Barry Rodewald, Carl, Philip Bennett
/*  Converted to C from MAME 0.289 src/devices/cpu/i386/i386.cpp (the i386_device helpers: address translation,
    fetches, memory and port accesses, the ALU helpers, the stack, flags, the effective address, the IRQ check,
    the opcode decoders) (BSD-3-Clause, copyright-holders: Ville Linde, Barry Rodewald, Carl, Philip Bennett).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
static uint32_t i386_translate_sz(int segment, uint32_t ip, int rwn, int size)
{
	// TODO: segment limit access size, execution permission, handle exception thrown from exception handler
	if (PROTECTED_MODE && !V8086_MODE && (rwn != -1))
	{
		// (ReBOX: a valid flat segment, limit 4 GB and not expand-down data: the limit check cannot fail, only the
		// read / write rights below are checked)
		const int flat = I.sreg[segment].valid && I.sreg[segment].limit == 0xffffffffu && (I.sreg[segment].flags & 0x1c) != 0x14;
		if (!flat && !(I.sreg[segment].valid))
			FAULT_THROW((segment == SS) ? FAULT_SS : FAULT_GP, 0);
		if (!flat) {   // (i386_limit_check_sz's tests here: no call; expand-down data segments through it)
			const uint32_t fl = I.sreg[segment].flags;
			if ((fl & 0x0018) == 0x0010 && (fl & 0x0004)) {
				if (i386_limit_check(segment, ip, size))
					FAULT_THROW((segment == SS) ? FAULT_SS : FAULT_GP, 0);
			} else if ((uint32_t)(ip + size - 1) > I.sreg[segment].limit)
				FAULT_THROW((segment == SS) ? FAULT_SS : FAULT_GP, 0);
		}
		if ((rwn == 0) && ((I.sreg[segment].flags & 8) && !(I.sreg[segment].flags & 2)))
			FAULT_THROW(FAULT_GP, 0);
		if ((rwn == 1) && ((I.sreg[segment].flags & 8) || !(I.sreg[segment].flags & 2)))
			FAULT_THROW(FAULT_GP, 0);
	}
	return I.sreg[segment].base + ip;
}

static vtlb_entry get_permissions(uint32_t pte, int wp)
{
	vtlb_entry ret = READ_ALLOWED | ((pte & 4) ? USER_READ_ALLOWED : 0);
	if (!wp)
		ret |= WRITE_ALLOWED;
	if (pte & 2)
		ret |= WRITE_ALLOWED | ((pte & 4) ? USER_WRITE_ALLOWED : 0);
	return ret;
}

static bool i386_translate_address(int intention, bool debug, offs_t *address, vtlb_entry *entry)
{
	offs_t a = *address;
	offs_t pdbr = I.cr[3] & 0xfffff000;
	offs_t directory = (a >> 22) & 0x3ff;
	offs_t table = (a >> 12) & 0x3ff;
	vtlb_entry perm = 0;
	bool ret;
	bool user = (intention & TR_USER) ? true : false;
	bool write = (intention & TR_WRITE) ? true : false;

	if (!(I.cr[0] & CR0_PG))
	{
		if (entry)
			*entry = 0x77;
		return true;
	}

	uint32_t page_dir = prog_r32(pdbr + directory * 4);
	if (page_dir & 1)
	{
		if ((page_dir & 0x80) && (I.cr[4] & CR4_PSE))
		{
			a = (page_dir & 0xffc00000) | (a & 0x003fffff);
			if (debug)
			{
				*address = a;
				return true;
			}
			perm = get_permissions(page_dir, WP);
			if (write && (!(perm & WRITE_ALLOWED) || (user && !(perm & USER_WRITE_ALLOWED))))
				ret = false;
			else if (user && !(perm & USER_READ_ALLOWED))
				ret = false;
			else
			{
				if (write)
					perm |= FLAG_DIRTY;
				if (!(page_dir & 0x40) && write)
					prog_w32(pdbr + directory * 4, page_dir | 0x60);
				else if (!(page_dir & 0x20))
					prog_w32(pdbr + directory * 4, page_dir | 0x20);
				ret = true;
			}
		}
		else
		{
			uint32_t page_entry = prog_r32((page_dir & 0xfffff000) + (table * 4));
			if (!(page_entry & 1))
				ret = false;
			else
			{
				a = (page_entry & 0xfffff000) | (a & 0xfff);
				if (debug)
				{
					*address = a;
					return true;
				}
				perm = get_permissions(page_entry, WP);
				if (write && (!(perm & WRITE_ALLOWED) || (user && !(perm & USER_WRITE_ALLOWED))))
					ret = false;
				else if (user && !(perm & USER_READ_ALLOWED))
					ret = false;
				else
				{
					if (write)
						perm |= FLAG_DIRTY;
					if (!(page_dir & 0x20))
						prog_w32(pdbr + directory * 4, page_dir | 0x20);
					if (!(page_entry & 0x40) && write)
						prog_w32((page_dir & 0xfffff000) + (table * 4), page_entry | 0x60);
					else if (!(page_entry & 0x20))
						prog_w32((page_dir & 0xfffff000) + (table * 4), page_entry | 0x20);
					ret = true;
				}
			}
		}
	}
	else
		ret = false;
	if (entry)
		*entry = perm;
	if (ret)
		*address = a;
	return ret;
}

static void CHANGE_PC(uint32_t pc)
{
	I.pc = i386_translate(CS, pc, -1 );
}

static void NEAR_BRANCH(int32_t offs)
{
	/* TODO: limit */
	I.eip += offs;
	I.pc += offs;
}

static uint8_t FETCH_slow(void);
static inline uint8_t FETCH(void)
{
	if (!(I.cr[0] & CR0_PG)) {   // (ReBOX: no paging and a direct page: the byte without a call)
		const offs_t a = I.pc & I.a20_mask;
		const uint8_t *p = s_rmap[a >> 16];
		if (p) {
			I.eip++;
			I.pc++;
			return p[a & 0xffff];
		}
	}
	return FETCH_slow();
}

static uint8_t FETCH_slow(void)
{
	uint8_t value;
	offs_t address = I.pc;
	uint32_t error;

	if(!translate_address(I.CPL,TR_FETCH,&address,&error))
		PF_THROW(error);

	value = prog_r8(address & I.a20_mask);
#ifdef DEBUG_MISSING_OPCODE
	I.opcode_bytes[I.opcode_bytes_length] = value;
	I.opcode_bytes_length = (I.opcode_bytes_length + 1) & 15;
#endif
	I.eip++;
	I.pc++;
	return value;
}

static uint16_t FETCH16(void)
{
	uint16_t value;
	offs_t address = I.pc;
	uint32_t error;

	if( !WORD_ALIGNED(address) ) {       /* Unaligned read */
		value = (FETCH() << 0);
		value |= (FETCH() << 8);
	} else {
		if(!translate_address(I.CPL,TR_FETCH,&address,&error))
			PF_THROW(error);
		address &= I.a20_mask;
		value = prog_r16(address);
		I.eip += 2;
		I.pc += 2;
	}
	return value;
}

static uint32_t FETCH32(void)
{
	uint32_t value;
	offs_t address = I.pc;
	uint32_t error;

	if( !DWORD_ALIGNED(I.pc) ) {      /* Unaligned read */
		value = (FETCH() << 0);
		value |= (FETCH() << 8);
		value |= (FETCH() << 16);
		value |= (FETCH() << 24);
	} else {
		if(!translate_address(I.CPL,TR_FETCH,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		value = prog_r32(address);
		I.eip += 4;
		I.pc += 4;
	}
	return value;
}

static uint8_t READ8PL(uint32_t ea, uint8_t privilege)
{
	offs_t address = ea;
	uint32_t error;

	if(!translate_address(privilege,TR_READ,&address,&error))
		PF_THROW(error);

	address &= I.a20_mask;
	return prog_r8(address);
}

static uint16_t READ16PL(uint32_t ea, uint8_t privilege)
{
	uint16_t value;
	offs_t address = ea;
	uint32_t error;

	switch (ea & 3)
	{
	case 0:
	case 2:
	default:
		if(!translate_address(privilege,TR_READ,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		value = prog_r16(address);
		break;

	case 1:
		if(!translate_address(privilege,TR_READ,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		value = prog_r32_mask(address - 1, 0x00ffff00) >> 8;
		break;

	case 3:
		value = READ8PL(ea, privilege);
		value |= READ8PL(ea + 1, privilege) << 8;
		break;
	}

	return value;
}

static uint32_t READ32PL(uint32_t ea, uint8_t privilege)
{
	uint32_t value;
	offs_t address = ea;
	uint32_t error;

	switch (ea & 3)
	{
	case 0:
	default:
		if(!translate_address(privilege,TR_READ,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		value = prog_r32(address);
		break;

	case 1:
		if(!translate_address(privilege,TR_READ,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		value = prog_r32_mask(address - 1, 0xffffff00) >> 8;
		value |= READ8PL(ea + 3, privilege) << 24;
		break;

	case 2:
		value = READ16PL(ea, privilege);
		value |= READ16PL(ea + 2, privilege) << 16;
		break;

	case 3:
		value = READ8PL(ea, privilege);

		address = ea + 1;
		if(!translate_address(privilege,TR_READ,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		value |= prog_r32_mask(address, 0x00ffffff) << 8;
		break;
	}

	return value;
}

static uint64_t READ64PL(uint32_t ea, uint8_t privilege)
{
	uint64_t value;
	offs_t address = ea;
	uint32_t error;

	switch (ea & 3)
	{
	case 0:
	default:
		value = READ32PL(ea, privilege);
		value |= ((uint64_t)(READ32PL(ea + 4, privilege))) << 32;
		break;

	case 1:
		if(!translate_address(privilege,TR_READ,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		value = prog_r32_mask(address - 1, 0xffffff00) >> 8;
		value |= ((uint64_t)(READ32PL(ea + 3, privilege))) << 24;
		value |= ((uint64_t)(READ8PL(ea + 7, privilege))) << 56;
		break;

	case 2:
		value = READ16PL(ea, privilege);
		value |= ((uint64_t)(READ32PL(ea + 2, privilege))) << 16;
		value |= ((uint64_t)(READ16PL(ea + 6, privilege))) << 48;
		break;

	case 3:
		value = READ8PL(ea, privilege);
		value |= ((uint64_t)(READ32PL(ea + 1, privilege))) << 8;

		address = ea + 5;
		if(!translate_address(privilege,TR_READ,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		value |= ((uint64_t)(prog_r32_mask(address, 0x00ffffff))) << 40;
		break;
	}

	return value;
}

static void WRITE_TEST(uint32_t ea)
{
	offs_t address = ea;
	uint32_t error;

	if(!translate_address(I.CPL,TR_WRITE,&address,&error))
		PF_THROW(error);
}

static void WRITE8PL(uint32_t ea, uint8_t privilege, uint8_t value)
{
	offs_t address = ea;
	uint32_t error;

	if(!translate_address(privilege,TR_WRITE,&address,&error))
		PF_THROW(error);

	address &= I.a20_mask;
	prog_w8(address, value);
}

static void WRITE16PL(uint32_t ea, uint8_t privilege, uint16_t value)
{
	offs_t address = ea;
	uint32_t error;

	switch(ea & 3)
	{
	case 0:
	case 2:
		if(!translate_address(privilege,TR_WRITE,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		prog_w16(address, value);
		break;

	case 1:
		if(!translate_address(privilege,TR_WRITE,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		prog_w32_mask(address - 1, value << 8, 0x00ffff00);
		break;

	case 3:
		WRITE8PL(ea, privilege, value & 0xff);
		WRITE8PL(ea + 1, privilege, (value >> 8) & 0xff);
		break;
	}
}

static void WRITE32PL(uint32_t ea, uint8_t privilege, uint32_t value)
{
	offs_t address = ea;
	uint32_t error;

	switch(ea & 3)
	{
	case 0:
		if(!translate_address(privilege,TR_WRITE,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		prog_w32(address, value);
		break;

	case 1:
		if(!translate_address(privilege,TR_WRITE,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		prog_w32_mask(address - 1, (value << 8) & 0xffffff00, 0xffffff00);
		WRITE8PL(ea + 3, privilege, (value >> 24) & 0xff);
		break;

	case 2:
		WRITE16PL(ea, privilege, value & 0xffff);
		WRITE16PL(ea + 2, privilege, (value >> 16) & 0xffff);
		break;

	case 3:
		WRITE8PL(ea, privilege, value & 0xff);

		address = ea + 1;
		if(!translate_address(privilege,TR_WRITE,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		prog_w32_mask(address, value >> 8, 0x00ffffff);
		break;
	}
}

static void WRITE64PL(uint32_t ea, uint8_t privilege, uint64_t value)
{
	offs_t address = ea;
	uint32_t error;

	switch(ea & 3)
	{
	case 0:
		WRITE32PL(ea, privilege, value & 0xffffffff);
		WRITE32PL(ea + 4, privilege, (value >> 32) & 0xffffffff);
		break;

	case 1:
		if(!translate_address(privilege,TR_WRITE,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		prog_w32_mask(address - 1, value << 8, 0xffffff00);
		WRITE32PL(ea + 3, privilege, (value >> 24) & 0xffffffff);
		WRITE8PL(ea + 7, privilege, (value >> 56) & 0xff );
		break;

	case 2:
		WRITE16PL(ea, privilege, value & 0xffff);
		WRITE32PL(ea + 2, privilege, (value >> 16) & 0xffffffff);
		WRITE16PL(ea + 6, privilege, (value >> 48) & 0xffff);
		break;

	case 3:
		WRITE8PL(ea, privilege, value & 0xff);
		WRITE32PL(ea + 1, privilege, (value >> 8) & 0xffffffff);

		address = ea + 5;
		if(!translate_address(privilege,TR_WRITE,&address,&error))
			PF_THROW(error);

		address &= I.a20_mask;
		prog_w32_mask(address, (value >> 40) & 0x00ffffff, 0x00ffffff);
		break;
	}
}

static uint8_t OR8(uint8_t dst, uint8_t src)
{
	uint8_t res = dst | src;
	I.CF = I.OF = 0;
	SetSZPF8(res);
	return res;
}

static uint16_t OR16(uint16_t dst, uint16_t src)
{
	uint16_t res = dst | src;
	I.CF = I.OF = 0;
	SetSZPF16(res);
	return res;
}

static uint32_t OR32(uint32_t dst, uint32_t src)
{
	uint32_t res = dst | src;
	I.CF = I.OF = 0;
	SetSZPF32(res);
	return res;
}

static uint8_t AND8(uint8_t dst, uint8_t src)
{
	uint8_t res = dst & src;
	I.CF = I.OF = 0;
	SetSZPF8(res);
	return res;
}

static uint16_t AND16(uint16_t dst, uint16_t src)
{
	uint16_t res = dst & src;
	I.CF = I.OF = 0;
	SetSZPF16(res);
	return res;
}

static uint32_t AND32(uint32_t dst, uint32_t src)
{
	uint32_t res = dst & src;
	I.CF = I.OF = 0;
	SetSZPF32(res);
	return res;
}

static uint8_t XOR8(uint8_t dst, uint8_t src)
{
	uint8_t res = dst ^ src;
	I.CF = I.OF = 0;
	SetSZPF8(res);
	return res;
}

static uint16_t XOR16(uint16_t dst, uint16_t src)
{
	uint16_t res = dst ^ src;
	I.CF = I.OF = 0;
	SetSZPF16(res);
	return res;
}

static uint32_t XOR32(uint32_t dst, uint32_t src)
{
	uint32_t res = dst ^ src;
	I.CF = I.OF = 0;
	SetSZPF32(res);
	return res;
}

static uint8_t SBB8(uint8_t dst, uint8_t src, uint8_t b)
{
	uint16_t res = (uint16_t)dst - (uint16_t)src - (uint8_t)b;
	SetCF8(res);
	SetOF_Sub8(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF8(res);
	return (uint8_t)res;
}

static uint16_t SBB16(uint16_t dst, uint16_t src, uint16_t b)
{
	uint32_t res = (uint32_t)dst - (uint32_t)src - (uint32_t)b;
	SetCF16(res);
	SetOF_Sub16(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF16(res);
	return (uint16_t)res;
}

static uint32_t SBB32(uint32_t dst, uint32_t src, uint32_t b)
{
	uint64_t res = (uint64_t)dst - (uint64_t)src - (uint64_t) b;
	SetCF32(res);
	SetOF_Sub32(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF32(res);
	return (uint32_t)res;
}

static uint8_t ADC8(uint8_t dst, uint8_t src, uint8_t c)
{
	uint16_t res = (uint16_t)dst + (uint16_t)src + (uint16_t)c;
	SetCF8(res);
	SetOF_Add8(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF8(res);
	return (uint8_t)res;
}

static uint16_t ADC16(uint16_t dst, uint16_t src, uint8_t c)
{
	uint32_t res = (uint32_t)dst + (uint32_t)src + (uint32_t)c;
	SetCF16(res);
	SetOF_Add16(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF16(res);
	return (uint16_t)res;
}

static uint32_t ADC32(uint32_t dst, uint32_t src, uint32_t c)
{
	uint64_t res = (uint64_t)dst + (uint64_t)src + (uint64_t) c;
	SetCF32(res);
	SetOF_Add32(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF32(res);
	return (uint32_t)res;
}

static uint8_t INC8(uint8_t dst)
{
	uint16_t res = (uint16_t)dst + 1;
	SetOF_Add8(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF8(res);
	return (uint8_t)res;
}

static uint16_t INC16(uint16_t dst)
{
	uint32_t res = (uint32_t)dst + 1;
	SetOF_Add16(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF16(res);
	return (uint16_t)res;
}

static uint32_t INC32(uint32_t dst)
{
	uint64_t res = (uint64_t)dst + 1;
	SetOF_Add32(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF32(res);
	return (uint32_t)res;
}

static uint8_t DEC8(uint8_t dst)
{
	uint16_t res = (uint16_t)dst - 1;
	SetOF_Sub8(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF8(res);
	return (uint8_t)res;
}

static uint16_t DEC16(uint16_t dst)
{
	uint32_t res = (uint32_t)dst - 1;
	SetOF_Sub16(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF16(res);
	return (uint16_t)res;
}

static uint32_t DEC32(uint32_t dst)
{
	uint64_t res = (uint64_t)dst - 1;
	SetOF_Sub32(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF32(res);
	return (uint32_t)res;
}

static void PUSH16(uint16_t value)
{
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) - 2;
		ea = i386_translate(SS, new_esp, 1);
		WRITE16(ea, value );
		REG32(ESP) = new_esp;
	} else {
		new_esp = (REG16(SP) - 2) & 0xffff;
		ea = i386_translate(SS, new_esp, 1);
		WRITE16(ea, value );
		REG16(SP) = new_esp;
	}
}

static void PUSH32(uint32_t value)
{
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) - 4;
		ea = i386_translate(SS, new_esp, 1);
		WRITE32(ea, value );
		REG32(ESP) = new_esp;
	} else {
		new_esp = (REG16(SP) - 4) & 0xffff;
		ea = i386_translate(SS, new_esp, 1);
		WRITE32(ea, value );
		REG16(SP) = new_esp;
	}
}

static void PUSH32SEG(uint32_t value)
{
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) - 4;
		ea = i386_translate(SS, new_esp, 1);
		((I.cpu_version & 0xf00) == 0x300) ? WRITE16(ea, value) : WRITE32(ea, value ); // 486 also?
		REG32(ESP) = new_esp;
	} else {
		new_esp = (REG16(SP) - 4) & 0xffff;
		ea = i386_translate(SS, new_esp, 1);
		((I.cpu_version & 0xf00) == 0x300) ? WRITE16(ea, value) : WRITE32(ea, value );
		REG16(SP) = new_esp;
	}
}

static void PUSH8(uint8_t value)
{
	if( I.operand_size ) {
		PUSH32((int32_t)(int8_t)value);
	} else {
		PUSH16((int16_t)(int8_t)value);
	}
}

static uint8_t POP8(void)
{
	uint8_t value;
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) + 1;
		ea = i386_translate(SS, new_esp - 1, 0);
		value = READ8(ea );
		REG32(ESP) = new_esp;
	} else {
		new_esp = REG16(SP) + 1;
		ea = i386_translate(SS, (new_esp - 1) & 0xffff, 0);
		value = READ8(ea );
		REG16(SP) = new_esp;
	}
	return value;
}

static uint16_t POP16(void)
{
	uint16_t value;
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) + 2;
		ea = i386_translate(SS, new_esp - 2, 0);
		value = READ16(ea );
		REG32(ESP) = new_esp;
	} else {
		new_esp = REG16(SP) + 2;
		ea = i386_translate(SS, (new_esp - 2) & 0xffff, 0);
		value = READ16(ea );
		REG16(SP) = new_esp;
	}
	return value;
}

static uint32_t POP32(void)
{
	uint32_t value;
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) + 4;
		ea = i386_translate(SS, new_esp - 4, 0);
		value = READ32(ea );
		REG32(ESP) = new_esp;
	} else {
		new_esp = REG16(SP) + 4;
		ea = i386_translate(SS, (new_esp - 4) & 0xffff, 0);
		value = READ32(ea );
		REG16(SP) = new_esp;
	}
	return value;
}

static void BUMP_SI(int adjustment)
{
	if ( I.address_size )
		REG32(ESI) += ((I.DF) ? -adjustment : +adjustment);
	else
		REG16(SI) += ((I.DF) ? -adjustment : +adjustment);
}

static void BUMP_DI(int adjustment)
{
	if ( I.address_size )
		REG32(EDI) += ((I.DF) ? -adjustment : +adjustment);
	else
		REG16(DI) += ((I.DF) ? -adjustment : +adjustment);
}

static inline __attribute__((always_inline)) void CYCLES(int x)
{
	if (PROTECTED_MODE)
	{
		I.cycles -= I.cycle_table_pm[x];
	}
	else
	{
		I.cycles -= I.cycle_table_rm[x];
	}
}

static void CYCLES_RM(int modrm, int r, int m)
{
	if (modrm >= 0xc0)
	{
		if (PROTECTED_MODE)
		{
			I.cycles -= I.cycle_table_pm[r];
		}
		else
		{
			I.cycles -= I.cycle_table_rm[r];
		}
	}
	else
	{
		if (PROTECTED_MODE)
		{
			I.cycles -= I.cycle_table_pm[m];
		}
		else
		{
			I.cycles -= I.cycle_table_rm[m];
		}
	}
}

static void check_ioperm(offs_t port, uint8_t mask)
{
	uint8_t IOPL, map;
	uint16_t IOPB;
	uint32_t address;

	if(!PROTECTED_MODE)
		return;

	IOPL = I.IOP1 | (I.IOP2 << 1);
	if(!V8086_MODE && (I.CPL <= IOPL))
		return;

	if((I.task.limit < 0x67) || ((I.task.flags & 0xd) != 9))
		FAULT_THROW(FAULT_GP,0);

	address = I.task.base;
	IOPB = READ16PL(address+0x66,0);
	if((IOPB+(port/8)) > I.task.limit)
		FAULT_THROW(FAULT_GP,0);

	map = READ8PL(address+IOPB+(port/8),0);
	map >>= (port%8);
	if(map & mask)
		FAULT_THROW(FAULT_GP,0);
}

static uint8_t READPORT8(offs_t port)
{
	check_ioperm(port, 1);
	return io_r8(port);
}

static void WRITEPORT8(offs_t port, uint8_t value)
{
	check_ioperm(port, 1);
	io_w8(port, value);
}

static uint16_t READPORT16(offs_t port)
{
	uint16_t value;

	switch (port & 3)
	{
	case 0:
	case 2:
	default:
		check_ioperm(port, 3);
		value = io_r16(port);
		break;

	case 1:
		check_ioperm(port, 3);
		value = io_r32_mask(port - 1, 0x00ffff00) >> 8;
		break;

	case 3:
		value = READPORT8(port);
		value |= (READPORT8(port + 1) << 8);
		break;
	}

	return value;
}

static void WRITEPORT16(offs_t port, uint16_t value)
{
	switch (port & 3)
	{
	case 0:
	case 2:
		check_ioperm(port, 3);
		io_w16(port, value);
		break;

	case 1:
		check_ioperm(port, 3);
		io_w32_mask(port - 1, value << 8, 0x00ffff00);
		break;

	case 3:
		WRITEPORT8(port, value & 0xff);
		WRITEPORT8(port + 1, (value >> 8) & 0xff);
		break;
	}
}

static uint32_t READPORT32(offs_t port)
{
	uint32_t value;

	switch (port & 3)
	{
	case 0:
	default:
		check_ioperm(port, 0xf);
		value = io_r32(port);
		break;

	case 1:
		check_ioperm(port, 7);
		value = io_r32_mask(port - 1, 0xffffff00) >> 8;
		value |= READPORT8(port + 3) << 24;
		break;

	case 2:
		value = READPORT16(port);
		value |= READPORT16(port + 2) << 16;
		break;

	case 3:
		value = READPORT8(port);
		check_ioperm(port + 1, 7);
		value |= io_r32_mask(port + 1, 0x00ffffff) << 8;
		break;
	}

	return value;
}

static void WRITEPORT32(offs_t port, uint32_t value)
{
	switch (port & 3)
	{
	case 0:
		check_ioperm(port, 0xf);
		io_w32(port, value);
		break;

	case 1:
		check_ioperm(port, 7);
		io_w32_mask(port - 1, value << 8, 0xffffff00);
		WRITEPORT8(port + 3, (value >> 24) & 0xff);
		break;

	case 2:
		WRITEPORT16(port, value & 0xffff);
		WRITEPORT16(port + 2, (value >> 16) & 0xffff);
		break;

	case 3:
		WRITEPORT8(port, value & 0xff);
		check_ioperm(port + 1, 7);
		io_w32_mask(port + 1, value >> 8, 0x00ffffff);
		break;
	}
}

static uint32_t get_flags(void)
{
	uint32_t f = 0x2;
	f |= I.CF;
	f |= I.PF << 2;
	f |= I.AF << 4;
	f |= I.ZF << 6;
	f |= I.SF << 7;
	f |= I.TF << 8;
	f |= I.IF << 9;
	f |= I.DF << 10;
	f |= I.OF << 11;
	f |= I.IOP1 << 12;
	f |= I.IOP2 << 13;
	f |= I.NT << 14;
	f |= I.RF << 16;
	f |= I.VM << 17;
	f |= I.AC << 18;
	f |= I.VIF << 19;
	f |= I.VIP << 20;
	f |= I.ID << 21;
	return (I.eflags & ~I.eflags_mask) | (f & I.eflags_mask);
}

static void set_flags(uint32_t f)
{
	f &= I.eflags_mask;
	I.CF = (f & 0x1) ? 1 : 0;
	I.PF = (f & 0x4) ? 1 : 0;
	I.AF = (f & 0x10) ? 1 : 0;
	I.ZF = (f & 0x40) ? 1 : 0;
	I.SF = (f & 0x80) ? 1 : 0;
	I.TF = (f & 0x100) ? 1 : 0;
	I.IF = (f & 0x200) ? 1 : 0;
	I.DF = (f & 0x400) ? 1 : 0;
	I.OF = (f & 0x800) ? 1 : 0;
	I.IOP1 = (f & 0x1000) ? 1 : 0;
	I.IOP2 = (f & 0x2000) ? 1 : 0;
	I.NT = (f & 0x4000) ? 1 : 0;
	I.RF = (f & 0x10000) ? 1 : 0;
	I.VM = (f & 0x20000) ? 1 : 0;
	I.AC = (f & 0x40000) ? 1 : 0;
	I.VIF = (f & 0x80000) ? 1 : 0;
	I.VIP = (f & 0x100000) ? 1 : 0;
	I.ID = (f & 0x200000) ? 1 : 0;
	I.eflags = f;
}

static void sib_byte(uint8_t mod, uint32_t* out_ea, uint8_t* out_segment)
{
	uint32_t ea = 0;
	uint8_t segment = 0;
	uint8_t scale, i, base;
	uint8_t sib = FETCH();
	scale = (sib >> 6) & 0x3;
	i = (sib >> 3) & 0x7;
	base = sib & 0x7;

	switch( base )
	{
		case 0: ea = REG32(EAX); segment = DS; break;
		case 1: ea = REG32(ECX); segment = DS; break;
		case 2: ea = REG32(EDX); segment = DS; break;
		case 3: ea = REG32(EBX); segment = DS; break;
		case 4: ea = REG32(ESP); segment = SS; break;
		case 5:
			if( mod == 0 ) {
				ea = FETCH32();
				segment = DS;
			} else if( mod == 1 ) {
				ea = REG32(EBP);
				segment = SS;
			} else if( mod == 2 ) {
				ea = REG32(EBP);
				segment = SS;
			}
			break;
		case 6: ea = REG32(ESI); segment = DS; break;
		case 7: ea = REG32(EDI); segment = DS; break;
	}
	switch( i )
	{
		case 0: ea += REG32(EAX) * (1 << scale); break;
		case 1: ea += REG32(ECX) * (1 << scale); break;
		case 2: ea += REG32(EDX) * (1 << scale); break;
		case 3: ea += REG32(EBX) * (1 << scale); break;
		case 4: break;
		case 5: ea += REG32(EBP) * (1 << scale); break;
		case 6: ea += REG32(ESI) * (1 << scale); break;
		case 7: ea += REG32(EDI) * (1 << scale); break;
	}
	*out_ea = ea;
	*out_segment = segment;
}

static void modrm_to_EA(uint8_t mod_rm, uint32_t* out_ea, uint8_t* out_segment)
{
	int8_t disp8;
	int16_t disp16;
	int32_t disp32;
	uint8_t mod = (mod_rm >> 6) & 0x3;
	uint8_t rm = mod_rm & 0x7;
	uint32_t ea;
	uint8_t segment;

	if( mod_rm >= 0xc0 )
		fatalerror("i386: Called modrm_to_EA with modrm value %02X!\n",mod_rm);


	if( I.address_size ) {
		switch( rm )
		{
			default:
			case 0: ea = REG32(EAX); segment = DS; break;
			case 1: ea = REG32(ECX); segment = DS; break;
			case 2: ea = REG32(EDX); segment = DS; break;
			case 3: ea = REG32(EBX); segment = DS; break;
			case 4: sib_byte(mod, &ea, &segment ); break;
			case 5:
				if( mod == 0 ) {
					ea = FETCH32(); segment = DS;
				} else {
					ea = REG32(EBP); segment = SS;
				}
				break;
			case 6: ea = REG32(ESI); segment = DS; break;
			case 7: ea = REG32(EDI); segment = DS; break;
		}
		if( mod == 1 ) {
			disp8 = FETCH();
			ea += (int32_t)disp8;
		} else if( mod == 2 ) {
			disp32 = FETCH32();
			ea += disp32;
		}

		if( I.segment_prefix )
			segment = I.segment_override;

		*out_ea = ea;
		*out_segment = segment;

	} else {
		switch( rm )
		{
			default:
			case 0: ea = REG16(BX) + REG16(SI); segment = DS; break;
			case 1: ea = REG16(BX) + REG16(DI); segment = DS; break;
			case 2: ea = REG16(BP) + REG16(SI); segment = SS; break;
			case 3: ea = REG16(BP) + REG16(DI); segment = SS; break;
			case 4: ea = REG16(SI); segment = DS; break;
			case 5: ea = REG16(DI); segment = DS; break;
			case 6:
				if( mod == 0 ) {
					ea = FETCH16(); segment = DS;
				} else {
					ea = REG16(BP); segment = SS;
				}
				break;
			case 7: ea = REG16(BX); segment = DS; break;
		}
		if( mod == 1 ) {
			disp8 = FETCH();
			ea += (int32_t)disp8;
		} else if( mod == 2 ) {
			disp16 = FETCH16();
			ea += (int32_t)disp16;
		}

		if( I.segment_prefix )
			segment = I.segment_override;

		*out_ea = ea & 0xffff;
		*out_segment = segment;
	}
}

static uint32_t GetNonTranslatedEA(uint8_t modrm,uint8_t *seg)
{
	uint8_t segment;
	uint32_t ea;
	modrm_to_EA(modrm, &ea, &segment );
	if(seg) *seg = segment;
	return ea;
}

static uint32_t GetEA(uint8_t modrm, int rwn)
{
	uint8_t segment;
	uint32_t ea;
	modrm_to_EA(modrm, &ea, &segment );
	return i386_translate(segment, ea, rwn );
}

static void i386_check_irq_line(void)
{
	if(!I.smm && I.smi)
	{
		enter_smm();
		return;
	}

	/* Check if the interrupts are enabled */
	if ( (I.irq_state) && I.IF )
	{
		I.cycles -= 2;
		i386_trap(I386_IRQ_ACK(), 1);
	}
}

static void report_invalid_opcode(void)
{
#ifndef DEBUG_MISSING_OPCODE
	LOGMASKED(LOG_INVALID_OPCODE, "i386: Invalid opcode %02X at %08X %s\n", I.opcode, I.pc - 1, I.lock ? "with lock" : "");
#else
	logerror("Invalid opcode");
	for (int a = 0; a < I.opcode_bytes_length; a++)
		logerror(" %02X", I.opcode_bytes[a]);
	logerror(" at %08X %s\n", I.opcode_pc, I.lock ? "with lock" : "");
	logerror("Backtrace:\n");
	for (uint32_t i = 1; i < 16; i++)
		logerror("  %08X\n", I.opcode_addrs[(I.opcode_addrs_index - i) & 15]);
#endif
}

static void report_invalid_modrm(const char* opcode, uint8_t modrm)
{
#ifndef DEBUG_MISSING_OPCODE
	LOGMASKED(LOG_INVALID_OPCODE, "i386: Invalid %s modrm %01X at %08X\n", opcode, modrm, I.pc - 2);
#else
	logerror("Invalid %s modrm %01X", opcode, modrm);
	for (int a = 0; a < I.opcode_bytes_length; a++)
		logerror(" %02X", I.opcode_bytes[a]);
	logerror(" at %08X %s\n", I.opcode_pc, I.lock ? "with lock" : "");
	logerror("Backtrace:\n");
	for (uint32_t i = 1; i < 16; i++)
		logerror("  %08X\n", I.opcode_addrs[(I.opcode_addrs_index - i) & 15]);
#endif
	i386_trap(6, 0);
}

static void i386_decode_opcode(void)
{
	I.opcode = FETCH();

	if(I.lock && !I.lock_table[0][I.opcode])
		return i386_invalid();

	if( I.operand_size )
		I.opcode_table1_32[I.opcode]();
	else
		I.opcode_table1_16[I.opcode]();
}

static void i386_decode_two_byte(void)
{
	I.opcode = FETCH();

	if(I.lock && !I.lock_table[1][I.opcode])
		return i386_invalid();

	if( I.operand_size )
		I.opcode_table2_32[I.opcode]();
	else
		I.opcode_table2_16[I.opcode]();
}
