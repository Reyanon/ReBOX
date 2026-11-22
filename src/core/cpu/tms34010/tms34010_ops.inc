/*  TMS34010: 34010ops.h, 34010fld.hxx, 34010tbl.hxx (the 34010 table), 34010ops.hxx (without the 34020 part) and the inline helpers / pixel functions / tables of tms34010.cpp, from MAME src/devices/cpu/tms34010 (BSD-3-Clause, copyright-holders: Alex Pasadyn,
    Zsolt Vasvari; parts based on code by Aaron Giles), put into C by a conversion script (member functions to static
    functions). Do not edit by hand.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
/* ---- 34010ops.h ---- */
// license:BSD-3-Clause
// copyright-holders:Alex Pasadyn,Zsolt Vasvari
/***************************************************************************

    TMS34010: Portable Texas Instruments TMS34010 emulator

    Copyright Alex Pasadyn/Zsolt Vasvari
    Parts based on code by Aaron Giles

***************************************************************************/

#ifndef MAME_CPU_TMS34010_34010OPS_H
#define MAME_CPU_TMS34010_34010OPS_H

#pragma once




/***************************************************************************
    MEMORY I/O MACROS
***************************************************************************/

/* IO registers accessor */
#define IOREG(reg)                (m_IOregs[reg])
#define SMART_IOREG(reg)          (m_IOregs[m_is_34020 ? (int)REG020_##reg : (int)REG_##reg])
#define PBH()                     (IOREG(REG_CONTROL) & 0x0100)
#define PBV()                     (IOREG(REG_CONTROL) & 0x0200)



/***************************************************************************
    FIELD WRITE MACROS
***************************************************************************/

#define WFIELDMAC(MASK,MAX)                                                         \
	uint32_t shift = offset & 0x0f;                                                 \
	uint32_t masked_data = data & (MASK);                                           \
	uint32_t old;                                                                   \
																					\
	offset = offset & 0xfffffff0;                                                   \
																					\
	if (shift >= MAX)                                                               \
	{                                                                               \
		old = (uint32_t)TMS34010_RDMEM_DWORD(offset) & ~((MASK) << shift);          \
		TMS34010_WRMEM_DWORD(offset, (masked_data << shift) | old);                 \
	}                                                                               \
	else                                                                            \
	{                                                                               \
		old = (uint32_t)TMS34010_RDMEM_WORD(offset) & ~((MASK) << shift);           \
		TMS34010_WRMEM_WORD(offset, ((masked_data & (MASK)) << shift) | old);       \
	}

#define WFIELDMAC_BIG(MASK,MAX)                                                     \
	uint32_t shift = offset & 0x0f;                                                 \
	uint32_t masked_data = data & (MASK);                                           \
	uint32_t old;                                                                   \
																					\
	offset = offset & 0xfffffff0;                                                   \
																					\
	old = (uint32_t)TMS34010_RDMEM_DWORD(offset) & ~(uint32_t)((MASK) << shift);    \
	TMS34010_WRMEM_DWORD(offset, (uint32_t)(masked_data << shift) | old);           \
	if (shift >= MAX)                                                               \
	{                                                                               \
		shift = 32 - shift;                                                         \
		old = (uint32_t)TMS34010_RDMEM_WORD(offset + 0x20) & ~((MASK) >> shift);    \
		TMS34010_WRMEM_WORD(offset, (masked_data >> shift) | old);                  \
	}

#define WFIELDMAC_8()                                                               \
	if (1)                                                                       \
	{                                                                               \
		WFIELDMAC(0xff,9);                                                          \
	}

#define RFIELDMAC_8()                                                               \
	if (1)                                                                       \
	{                                                                               \
		RFIELDMAC(0xff,9);                                                          \
	}

#define WFIELDMAC_32()                                                              \
	if (offset & 0x0f)                                                              \
	{                                                                               \
		uint32_t shift = offset&0x0f;                                               \
		uint32_t old;                                                               \
		uint32_t hiword;                                                            \
		offset &= 0xfffffff0;                                                       \
		old =    ((uint32_t) TMS34010_RDMEM_DWORD (offset     )&(0xffffffff>>(0x20-shift)));   \
		hiword = ((uint32_t) TMS34010_RDMEM_DWORD (offset+0x20)&(0xffffffff<<shift));      \
		TMS34010_WRMEM_DWORD(offset     ,(data<<      shift) |old);                 \
		TMS34010_WRMEM_DWORD(offset+0x20,(data>>(0x20-shift))|hiword);              \
	}                                                                               \
	else                                                                            \
		TMS34010_WRMEM_DWORD(offset,data);


/***************************************************************************
    FIELD READ MACROS
***************************************************************************/

#define RFIELDMAC(MASK,MAX)                                                         \
	uint32_t shift = offset & 0x0f;                                                 \
	offset = offset & 0xfffffff0;                                                   \
																					\
	if (shift >= MAX)                                                               \
		ret = (TMS34010_RDMEM_DWORD(offset) >> shift) & (MASK);                     \
	else                                                                            \
		ret = (TMS34010_RDMEM_WORD(offset) >> shift) & (MASK);

#define RFIELDMAC_BIG(MASK,MAX)                                                     \
	uint32_t shift = offset & 0x0f;                                                 \
	offset = offset & 0xfffffff0;                                                   \
																					\
	ret = (uint32_t)TMS34010_RDMEM_DWORD(offset) >> shift;                          \
	if (shift >= MAX)                                                               \
		ret |= (TMS34010_RDMEM_WORD(offset + 0x20) << (32 - shift));                \
	ret &= MASK;

#define RFIELDMAC_32()                                                              \
	if (offset&0x0f)                                                                \
	{                                                                               \
		uint32_t shift = offset&0x0f;                                               \
		offset &= 0xfffffff0;                                                       \
		return (((uint32_t)TMS34010_RDMEM_DWORD (offset     )>>      shift) |       \
						  (TMS34010_RDMEM_DWORD (offset+0x20)<<(0x20-shift)));      \
	}                                                                               \
	else                                                                            \
		return TMS34010_RDMEM_DWORD(offset);


#endif // MAME_CPU_TMS34010_34010OPS_H



/***************************************************************************
    MACROS
***************************************************************************/

/* status register definitions */
#define STBIT_N         (1 << 31)
#define STBIT_C         (1 << 30)
#define STBIT_Z         (1 << 29)
#define STBIT_V         (1 << 28)
#define STBIT_P         (1 << 25)
#define STBIT_IE        (1 << 21)
#define STBIT_FE1       (1 << 11)
#define STBITS_F1       (0x1f << 6)
#define STBIT_FE0       (1 << 5)
#define STBITS_F0       (0x1f << 0)

/* register definitions and shortcuts */
#define N_FLAG()       (m_st & STBIT_N)
#define Z_FLAG()       (m_st & STBIT_Z)
#define C_FLAG()       (m_st & STBIT_C)
#define V_FLAG()       (m_st & STBIT_V)
#define P_FLAG()       (m_st & STBIT_P)
#define IE_FLAG()      (m_st & STBIT_IE)
#define FE0_FLAG()     (m_st & STBIT_FE0)
#define FE1_FLAG()     (m_st & STBIT_FE1)

/* register file access */
#define AREG(i)       (m_regs[i].reg)
#define AREG_XY(i)    (m_regs[i].xy)
#define AREG_X(i)     (m_regs[i].xy.x)
#define AREG_Y(i)     (m_regs[i].xy.y)
#define BREG(i)       (m_regs[30 - (i)].reg)
#define BREG_XY(i)    (m_regs[30 - (i)].xy)
#define BREG_X(i)     (m_regs[30 - (i)].xy.x)
#define BREG_Y(i)     (m_regs[30 - (i)].xy.y)
#define SP()          AREG(15)
#define FW(i)         ((m_st >> (i ? 6 : 0)) & 0x1f)
#define FWEX(i)       ((m_st >> (i ? 6 : 0)) & 0x3f)

/* opcode decode helpers */
#define SRCREG(O)       (((O) >> 5) & 0x0f)
#define DSTREG(O)       ((O) & 0x0f)
#define SKIP_WORD()     (m_pc += (2 << 3))
#define SKIP_LONG()     (m_pc += (4 << 3))
#define PARAM_K(O)      (((O) >> 5) & 0x1f)
#define PARAM_N(O)      ((O) & 0x1f)
#define PARAM_REL8(O)   ((int8_t)(O))

/* memory I/O */
#define WFIELD0(a,b)  (s_wfield_functions[FW(0)])(a,b)
#define WFIELD1(a,b)  (s_wfield_functions[FW(1)])(a,b)
#define RFIELD0(a)    (s_rfield_functions[FWEX(0)])(a)
#define RFIELD1(a)    (s_rfield_functions[FWEX(1)])(a)
#define WPIXEL(a,b)   (m_pixel_write)(a,b)
#define RPIXEL(a)     (m_pixel_read)(a)

/* Implied Operands */
#define SADDR()        BREG(0)
#define SADDR_X()      BREG_X(0)
#define SADDR_Y()      BREG_Y(0)
#define SADDR_XY()     BREG_XY(0)
#define SPTCH()        BREG(1)
#define DADDR()        BREG(2)
#define DADDR_X()      BREG_X(2)
#define DADDR_Y()      BREG_Y(2)
#define DADDR_XY()     BREG_XY(2)
#define DPTCH()        BREG(3)
#define OFFSET()       BREG(4)
#define WSTART_X()     BREG_X(5)
#define WSTART_Y()     BREG_Y(5)
#define WSTART_XY()    BREG_XY(5)
#define WEND_X()       BREG_X(6)
#define WEND_Y()       BREG_Y(6)
#define WEND_XY()      BREG_XY(6)
#define DYDX_X()       BREG_X(7)
#define DYDX_Y()       BREG_Y(7)
#define DYDX_XY()      BREG_XY(7)
#define COLOR0()       BREG(8)
#define COLOR1()       BREG(9)
#define COUNT()        BREG(10)
#define INC1_X()       BREG_X(11)
#define INC1_Y()       BREG_Y(11)
#define INC2_X()       BREG_X(12)
#define INC2_Y()       BREG_Y(12)
#define PATTRN()       BREG(13)
#define TEMP()         BREG(14)

/* I/O registers */
#define WINDOW_CHECKING()  ((IOREG(REG_CONTROL) >> 6) & 0x03)



/***************************************************************************
    INLINE SHORTCUTS
***************************************************************************/

/* Break up Status Register into indiviual flags */
static inline void SET_ST(uint32_t st)
{
	m_st = st;
	/* interrupts might have been enabled, check it */
	check_interrupt();
}

/* Intialize Status to 0x0010 */
static inline void RESET_ST()
{
	SET_ST(0x00000010);
}

/* shortcuts for reading opcodes */



















/* read memory byte */
static inline uint32_t RBYTE(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_8();
	return ret;
}

/* write memory byte */
static inline void WBYTE(offs_t offset, uint32_t data)
{
	WFIELDMAC_8();
}

/* read memory long */
static inline uint32_t RLONG(offs_t offset)
{
	RFIELDMAC_32();
}

/* write memory long */
static inline void WLONG(offs_t offset, uint32_t data)
{
	WFIELDMAC_32();
}

/* pushes/pops a value from the stack */
static inline void PUSH(uint32_t data)
{
	SP() -= 0x20;
	WLONG(SP(), data);
}

static inline int32_t POP()
{
	int32_t ret = RLONG(SP());
	SP() += 0x20;
	return ret;
}



/***************************************************************************
    PIXEL READS
***************************************************************************/

#define RP(m1,m2)                                           \
	/* TODO: Plane masking */                               \
	return (TMS34010_RDMEM_WORD(offset & 0xfffffff0) >> (offset & m1)) & m2;

static uint32_t read_pixel_1(offs_t offset) { RP(0x0f,0x01) }
static uint32_t read_pixel_2(offs_t offset) { RP(0x0e,0x03) }
static uint32_t read_pixel_4(offs_t offset) { RP(0x0c,0x0f) }
static uint32_t read_pixel_8(offs_t offset) { RP(0x08,0xff) }
static uint32_t read_pixel_16(offs_t offset)
{
	/* TODO: Plane masking */
	return TMS34010_RDMEM_WORD(offset & 0xfffffff0);
}
static uint32_t read_pixel_32(offs_t offset)
{
	/* TODO: Plane masking */
	return TMS34010_RDMEM_DWORD(offset & 0xffffffe0);
}

/* Shift register read */
static uint32_t read_pixel_shiftreg(offs_t offset)
{
	if (m_to_shiftreg_cb)
		m_to_shiftreg_cb(offset, &m_shiftreg[0]);
	else
		fatalerror("To ShiftReg function not set. PC = %08X\n", m_pc);
	return m_shiftreg[0];
}



/***************************************************************************
    PIXEL WRITES
***************************************************************************/

/* No Raster Op + No Transparency */
#define WP(m1,m2)                                                                           \
	uint32_t a = offset & 0xfffffff0;                                                       \
	uint32_t pix = TMS34010_RDMEM_WORD(a);                                                  \
	uint32_t shiftcount = offset & m1;                                                      \
																							\
	/* TODO: plane masking */                                                               \
	data &= m2;                                                                             \
	pix = (pix & ~(m2 << shiftcount)) | (data << shiftcount);                               \
	TMS34010_WRMEM_WORD(a, pix);

/* No Raster Op + Transparency */
#define WP_T(m1,m2)                                                                         \
	/* TODO: plane masking */                                                               \
	data &= m2;                                                                             \
	if (data)                                                                               \
	{                                                                                       \
		uint32_t a = offset & 0xfffffff0;                                                   \
		uint32_t pix = TMS34010_RDMEM_WORD(a);                                              \
		uint32_t shiftcount = offset & m1;                                                  \
																							\
		/* TODO: plane masking */                                                           \
		pix = (pix & ~(m2 << shiftcount)) | (data << shiftcount);                           \
		TMS34010_WRMEM_WORD(a, pix);                                                        \
	}
/* Raster Op + No Transparency */
#define WP_R(m1,m2)                                                                         \
	uint32_t a = offset & 0xfffffff0;                                                       \
	uint32_t pix = TMS34010_RDMEM_WORD(a);                                                  \
	uint32_t shiftcount = offset & m1;                                                      \
																							\
	/* TODO: plane masking */                                                               \
	data = (m_raster_op)(data & m2, (pix >> shiftcount) & m2) & m2;                  \
	pix = (pix & ~(m2 << shiftcount)) | (data << shiftcount);                               \
	TMS34010_WRMEM_WORD(a, pix);

/* Raster Op + Transparency */
#define WP_R_T(m1,m2)                                                                       \
	uint32_t a = offset & 0xfffffff0;                                                       \
	uint32_t pix = TMS34010_RDMEM_WORD(a);                                                  \
	uint32_t shiftcount = offset & m1;                                                      \
																							\
	/* TODO: plane masking */                                                               \
	data = (m_raster_op)(data & m2, (pix >> shiftcount) & m2) & m2;                  \
	if (data)                                                                               \
	{                                                                                       \
		pix = (pix & ~(m2 << shiftcount)) | (data << shiftcount);                           \
		TMS34010_WRMEM_WORD(a, pix);                                                        \
	}

/* No Raster Op + No Transparency */
static void write_pixel_1(offs_t offset, uint32_t data) { WP(0x0f, 0x01); }
static void write_pixel_2(offs_t offset, uint32_t data) { WP(0x0e, 0x03); }
static void write_pixel_4(offs_t offset, uint32_t data) { WP(0x0c, 0x0f); }
static void write_pixel_8(offs_t offset, uint32_t data) { WP(0x08, 0xff); }
static void write_pixel_16(offs_t offset, uint32_t data)
{
	/* TODO: plane masking */
	TMS34010_WRMEM_WORD(offset & 0xfffffff0, data);
}
static void write_pixel_32(offs_t offset, uint32_t data)
{
	/* TODO: plane masking */
	TMS34010_WRMEM_DWORD(offset & 0xffffffe0, data);
}

/* No Raster Op + Transparency */
static void write_pixel_t_1(offs_t offset, uint32_t data) { WP_T(0x0f, 0x01); }
static void write_pixel_t_2(offs_t offset, uint32_t data) { WP_T(0x0e, 0x03); }
static void write_pixel_t_4(offs_t offset, uint32_t data) { WP_T(0x0c, 0x0f); }
static void write_pixel_t_8(offs_t offset, uint32_t data) { WP_T(0x08, 0xff); }
static void write_pixel_t_16(offs_t offset, uint32_t data)
{
	/* TODO: plane masking */
	if (data)
		TMS34010_WRMEM_WORD(offset & 0xfffffff0, data);
}
static void write_pixel_t_32(offs_t offset, uint32_t data)
{
	/* TODO: plane masking */
	if (data)
		TMS34010_WRMEM_DWORD(offset & 0xffffffe0, data);
}

/* Raster Op + No Transparency */
static void write_pixel_r_1(offs_t offset, uint32_t data) { WP_R(0x0f, 0x01); }
static void write_pixel_r_2(offs_t offset, uint32_t data) { WP_R(0x0e, 0x03); }
static void write_pixel_r_4(offs_t offset, uint32_t data) { WP_R(0x0c, 0x0f); }
static void write_pixel_r_8(offs_t offset, uint32_t data) { WP_R(0x08, 0xff); }
static void write_pixel_r_16(offs_t offset, uint32_t data)
{
	/* TODO: plane masking */
	uint32_t a = offset & 0xfffffff0;
	TMS34010_WRMEM_WORD(a, (m_raster_op)(data, TMS34010_RDMEM_WORD(a)));
}
static void write_pixel_r_32(offs_t offset, uint32_t data)
{
	/* TODO: plane masking */
	uint32_t a = offset & 0xffffffe0;
	TMS34010_WRMEM_DWORD(a, (m_raster_op)(data, TMS34010_RDMEM_DWORD(a)));
}

/* Raster Op + Transparency */
static void write_pixel_r_t_1(offs_t offset, uint32_t data) { WP_R_T(0x0f,0x01); }
static void write_pixel_r_t_2(offs_t offset, uint32_t data) { WP_R_T(0x0e,0x03); }
static void write_pixel_r_t_4(offs_t offset, uint32_t data) { WP_R_T(0x0c,0x0f); }
static void write_pixel_r_t_8(offs_t offset, uint32_t data) { WP_R_T(0x08,0xff); }
static void write_pixel_r_t_16(offs_t offset, uint32_t data)
{
	/* TODO: plane masking */
	uint32_t a = offset & 0xfffffff0;
	data = (m_raster_op)(data, TMS34010_RDMEM_WORD(a));

	if (data)
		TMS34010_WRMEM_WORD(a, data);
}
static void write_pixel_r_t_32(offs_t offset, uint32_t data)
{
	/* TODO: plane masking */
	uint32_t a = offset & 0xffffffe0;
	data = (m_raster_op)(data, TMS34010_RDMEM_DWORD(a));

	if (data)
		TMS34010_WRMEM_DWORD(a, data);
}

/* Shift register write */
static void write_pixel_shiftreg(offs_t offset, uint32_t data)
{
	if (m_from_shiftreg_cb)
		m_from_shiftreg_cb(offset, &m_shiftreg[0]);
	else
		fatalerror("From ShiftReg function not set. PC = %08X\n", m_pc);
}



/***************************************************************************
    RASTER OPS
***************************************************************************/

/* Raster operations */
static uint32_t raster_op_1(uint32_t newpix, uint32_t oldpix)  { return newpix & oldpix; }
static uint32_t raster_op_2(uint32_t newpix, uint32_t oldpix)  { return newpix & ~oldpix; }
static uint32_t raster_op_3(uint32_t newpix, uint32_t oldpix)  { return 0; }
static uint32_t raster_op_4(uint32_t newpix, uint32_t oldpix)  { return newpix | ~oldpix; }
static uint32_t raster_op_5(uint32_t newpix, uint32_t oldpix)  { return ~(newpix ^ oldpix); }
static uint32_t raster_op_6(uint32_t newpix, uint32_t oldpix)  { return ~oldpix; }
static uint32_t raster_op_7(uint32_t newpix, uint32_t oldpix)  { return ~(newpix | oldpix); }
static uint32_t raster_op_8(uint32_t newpix, uint32_t oldpix)  { return newpix | oldpix; }
static uint32_t raster_op_9(uint32_t newpix, uint32_t oldpix)  { return oldpix; }
static uint32_t raster_op_10(uint32_t newpix, uint32_t oldpix) { return newpix ^ oldpix; }
static uint32_t raster_op_11(uint32_t newpix, uint32_t oldpix) { return ~newpix & oldpix; }
static uint32_t raster_op_12(uint32_t newpix, uint32_t oldpix) { return 0xffff; }
static uint32_t raster_op_13(uint32_t newpix, uint32_t oldpix) { return ~newpix | oldpix; }
static uint32_t raster_op_14(uint32_t newpix, uint32_t oldpix) { return ~(newpix & oldpix); }
static uint32_t raster_op_15(uint32_t newpix, uint32_t oldpix) { return ~newpix; }
static uint32_t raster_op_16(uint32_t newpix, uint32_t oldpix) { return newpix + oldpix; }
static uint32_t raster_op_17(uint32_t newpix, uint32_t oldpix)
{
	uint32_t max = (uint32_t)0xffffffff >> (32 - IOREG(REG_PSIZE));
	uint32_t res = newpix + oldpix;
	return (res > max) ? max : res;
}
static uint32_t raster_op_18(uint32_t newpix, uint32_t oldpix) { return oldpix - newpix; }
static uint32_t raster_op_19(uint32_t newpix, uint32_t oldpix) { return (oldpix > newpix) ? oldpix - newpix : 0; }
static uint32_t raster_op_20(uint32_t newpix, uint32_t oldpix) { return (oldpix > newpix) ? oldpix : newpix; }
static uint32_t raster_op_21(uint32_t newpix, uint32_t oldpix) { return (oldpix > newpix) ? newpix : oldpix; }



/***************************************************************************
    OPCODE TABLE & IMPLEMENTATIONS
***************************************************************************/

/* ---- 34010fld.hxx ---- */
// license:BSD-3-Clause
// copyright-holders:Alex Pasadyn,Zsolt Vasvari
/***************************************************************************

    TMS34010: Portable Texas Instruments TMS34010 emulator

    Copyright Alex Pasadyn/Zsolt Vasvari
    Parts based on code by Aaron Giles

***************************************************************************/



/***************************************************************************
    FIELD WRITE FUNCTIONS
***************************************************************************/

static void wfield_01(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x01,16);
}

static void wfield_02(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x03,15);
}

static void wfield_03(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x07,14);
}

static void wfield_04(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x0f,13);
}

static void wfield_05(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x1f,12);
}

static void wfield_06(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x3f,11);
}

static void wfield_07(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x7f,10);
}

static void wfield_08(offs_t offset, uint32_t data)
{
	WFIELDMAC_8();
}

static void wfield_09(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x1ff,8);
}

static void wfield_10(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x3ff,7);
}

static void wfield_11(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x7ff,6);
}

static void wfield_12(offs_t offset, uint32_t data)
{
	WFIELDMAC(0xfff,5);
}

static void wfield_13(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x1fff,4);
}

static void wfield_14(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x3fff,3);
}

static void wfield_15(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x7fff,2);
}

static void wfield_16(offs_t offset, uint32_t data)
{
	if (offset & 0x0f)
	{
		WFIELDMAC(0xffff,1);
	}
	else
	{
		TMS34010_WRMEM_WORD(offset,data);
	}
}

static void wfield_17(offs_t offset, uint32_t data)
{
	WFIELDMAC(0x1ffff,0);
}

static void wfield_18(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x3ffff,15);
}

static void wfield_19(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x7ffff,14);
}

static void wfield_20(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0xfffff,13);
}

static void wfield_21(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x1fffff,12);
}

static void wfield_22(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x3fffff,11);
}

static void wfield_23(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x7fffff,10);
}

static void wfield_24(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0xffffff,9);
}

static void wfield_25(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x1ffffff,8);
}

static void wfield_26(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x3ffffff,7);
}

static void wfield_27(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x7ffffff,6);
}

static void wfield_28(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0xfffffff,5);
}

static void wfield_29(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x1fffffff,4);
}

static void wfield_30(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x3fffffff,3);
}

static void wfield_31(offs_t offset, uint32_t data)
{
	WFIELDMAC_BIG(0x7fffffff,2);
}

static void wfield_32(offs_t offset, uint32_t data)
{
	WFIELDMAC_32();
}


static const wfield_func s_wfield_functions[32] =
{
	wfield_32, wfield_01, wfield_02, wfield_03, wfield_04, wfield_05,
	wfield_06, wfield_07, wfield_08, wfield_09, wfield_10, wfield_11,
	wfield_12, wfield_13, wfield_14, wfield_15, wfield_16, wfield_17,
	wfield_18, wfield_19, wfield_20, wfield_21, wfield_22, wfield_23,
	wfield_24, wfield_25, wfield_26, wfield_27, wfield_28, wfield_29,
	wfield_30, wfield_31
};



/***************************************************************************
    FIELD READ FUNCTIONS (ZERO-EXTEND)
***************************************************************************/

static uint32_t rfield_z_01(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x01,16);
	return ret;
}

static uint32_t rfield_z_02(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x03,15);
	return ret;
}

static uint32_t rfield_z_03(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x07,14);
	return ret;
}

static uint32_t rfield_z_04(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x0f,13);
	return ret;
}

static uint32_t rfield_z_05(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x1f,12);
	return ret;
}

static uint32_t rfield_z_06(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x3f,11);
	return ret;
}

static uint32_t rfield_z_07(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x7f,10);
	return ret;
}

static uint32_t rfield_z_08(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_8();
	return ret;
}

static uint32_t rfield_z_09(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x1ff,8);
	return ret;
}

static uint32_t rfield_z_10(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x3ff,7);
	return ret;
}

static uint32_t rfield_z_11(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x7ff,6);
	return ret;
}

static uint32_t rfield_z_12(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0xfff,5);
	return ret;
}

static uint32_t rfield_z_13(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x1fff,4);
	return ret;
}

static uint32_t rfield_z_14(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x3fff,3);
	return ret;
}

static uint32_t rfield_z_15(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x7fff,2);
	return ret;
}

static uint32_t rfield_z_16(offs_t offset)
{
	uint32_t ret;
	if (offset & 0x0f)
	{
		RFIELDMAC(0xffff,1);
	}

	else
		ret = TMS34010_RDMEM_WORD(offset);
	return ret;
}

static uint32_t rfield_z_17(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x1ffff,0);
	return ret;
}

static uint32_t rfield_z_18(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x3ffff,15);
	return ret;
}

static uint32_t rfield_z_19(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x7ffff,14);
	return ret;
}

static uint32_t rfield_z_20(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0xfffff,13);
	return ret;
}

static uint32_t rfield_z_21(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x1fffff,12);
	return ret;
}

static uint32_t rfield_z_22(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x3fffff,11);
	return ret;
}

static uint32_t rfield_z_23(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x7fffff,10);
	return ret;
}

static uint32_t rfield_z_24(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0xffffff,9);
	return ret;
}

static uint32_t rfield_z_25(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x1ffffff,8);
	return ret;
}

static uint32_t rfield_z_26(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x3ffffff,7);
	return ret;
}

static uint32_t rfield_z_27(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x7ffffff,6);
	return ret;
}

static uint32_t rfield_z_28(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0xfffffff,5);
	return ret;
}

static uint32_t rfield_z_29(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x1fffffff,4);
	return ret;
}

static uint32_t rfield_z_30(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x3fffffff,3);
	return ret;
}

static uint32_t rfield_z_31(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x7fffffff,2);
	return ret;
}

static uint32_t rfield_32(offs_t offset)
{
	RFIELDMAC_32();
}


/***************************************************************************
    FIELD READ FUNCTIONS (SIGN-EXTEND)
***************************************************************************/

static uint32_t rfield_s_01(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x01,16);
	return TMS_SEXT(ret, 1);
}

static uint32_t rfield_s_02(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x03,15);
	return TMS_SEXT(ret, 2);
}

static uint32_t rfield_s_03(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x07,14);
	return TMS_SEXT(ret, 3);
}

static uint32_t rfield_s_04(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x0f,13);
	return TMS_SEXT(ret, 4);
}

static uint32_t rfield_s_05(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x1f,12);
	return TMS_SEXT(ret, 5);
}

static uint32_t rfield_s_06(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x3f,11);
	return TMS_SEXT(ret, 6);
}

static uint32_t rfield_s_07(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x7f,10);
	return TMS_SEXT(ret, 7);
}

static uint32_t rfield_s_08(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_8();
	return (int32_t)(int8_t)ret;
}

static uint32_t rfield_s_09(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x1ff,8);
	return TMS_SEXT(ret, 9);
}

static uint32_t rfield_s_10(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x3ff,7);
	return TMS_SEXT(ret, 10);
}

static uint32_t rfield_s_11(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x7ff,6);
	return TMS_SEXT(ret, 11);
}

static uint32_t rfield_s_12(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0xfff,5);
	return TMS_SEXT(ret, 12);
}

static uint32_t rfield_s_13(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x1fff,4);
	return TMS_SEXT(ret, 13);
}

static uint32_t rfield_s_14(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x3fff,3);
	return TMS_SEXT(ret, 14);
}

static uint32_t rfield_s_15(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x7fff,2);
	return TMS_SEXT(ret, 15);
}

static uint32_t rfield_s_16(offs_t offset)
{
	uint32_t ret;
	if (offset & 0x0f)
	{
		RFIELDMAC(0xffff,1);
	}

	else
	{
		ret = TMS34010_RDMEM_WORD(offset);
	}

	return (int32_t)(int16_t)ret;
}

static uint32_t rfield_s_17(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC(0x1ffff,0);
	return TMS_SEXT(ret, 17);
}

static uint32_t rfield_s_18(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x3ffff,15);
	return TMS_SEXT(ret, 18);
}

static uint32_t rfield_s_19(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x7ffff,14);
	return TMS_SEXT(ret, 19);
}

static uint32_t rfield_s_20(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0xfffff,13);
	return TMS_SEXT(ret, 20);
}

static uint32_t rfield_s_21(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x1fffff,12);
	return TMS_SEXT(ret, 21);
}

static uint32_t rfield_s_22(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x3fffff,11);
	return TMS_SEXT(ret, 22);
}

static uint32_t rfield_s_23(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x7fffff,10);
	return TMS_SEXT(ret, 23);
}

static uint32_t rfield_s_24(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0xffffff,9);
	return TMS_SEXT(ret, 24);
}

static uint32_t rfield_s_25(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x1ffffff,8);
	return TMS_SEXT(ret, 25);
}

static uint32_t rfield_s_26(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x3ffffff,7);
	return TMS_SEXT(ret, 26);
}

static uint32_t rfield_s_27(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x7ffffff,6);
	return TMS_SEXT(ret, 27);
}

static uint32_t rfield_s_28(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0xfffffff,5);
	return TMS_SEXT(ret, 28);
}

static uint32_t rfield_s_29(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x1fffffff,4);
	return TMS_SEXT(ret, 29);
}

static uint32_t rfield_s_30(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x3fffffff,3);
	return TMS_SEXT(ret, 30);
}

static uint32_t rfield_s_31(offs_t offset)
{
	uint32_t ret;
	RFIELDMAC_BIG(0x7fffffff,2);
	return TMS_SEXT(ret, 31);
}

static const rfield_func s_rfield_functions[64] =
{
	rfield_32  , rfield_z_01, rfield_z_02, rfield_z_03, rfield_z_04, rfield_z_05,
	rfield_z_06, rfield_z_07, rfield_z_08, rfield_z_09, rfield_z_10, rfield_z_11,
	rfield_z_12, rfield_z_13, rfield_z_14, rfield_z_15, rfield_z_16, rfield_z_17,
	rfield_z_18, rfield_z_19, rfield_z_20, rfield_z_21, rfield_z_22, rfield_z_23,
	rfield_z_24, rfield_z_25, rfield_z_26, rfield_z_27, rfield_z_28, rfield_z_29,
	rfield_z_30, rfield_z_31,
	rfield_32  , rfield_s_01, rfield_s_02, rfield_s_03, rfield_s_04, rfield_s_05,
	rfield_s_06, rfield_s_07, rfield_s_08, rfield_s_09, rfield_s_10, rfield_s_11,
	rfield_s_12, rfield_s_13, rfield_s_14, rfield_s_15, rfield_s_16, rfield_s_17,
	rfield_s_18, rfield_s_19, rfield_s_20, rfield_s_21, rfield_s_22, rfield_s_23,
	rfield_s_24, rfield_s_25, rfield_s_26, rfield_s_27, rfield_s_28, rfield_s_29,
	rfield_s_30, rfield_s_31
};


/* includes the static function prototypes and the master opcode table */
/* ---- 34010tbl.hxx ---- */
// license:BSD-3-Clause
// copyright-holders:Alex Pasadyn,Zsolt Vasvari
/*** TMS34010: Portable TMS34010 emulator ***********************************

    Copyright Alex Pasadyn/Zsolt Vasvari

    Opcode Table

*****************************************************************************/

// 34010 Opcode Table
static const opcode_func s_opcode_table[65536 >> 4] =
{
	// 0x0000 0x0010 0x0020 0x0030 ... 0x00f0
	unimpl,     unimpl,     rev_a,      rev_b,      unimpl,     unimpl,     unimpl,     unimpl,
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	// 0x0100
	emu,        unimpl,     exgpc_a,    exgpc_b,    getpc_a,    getpc_b,    jump_a,     jump_b,
	getst_a,    getst_b,    putst_a,    putst_b,    popst,      unimpl,     pushst,     unimpl,
	// 0x0200
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x0300
	nop,        unimpl,     clrc,       unimpl,     movb_aa,    unimpl,     dint,       unimpl,
	abs_a,      abs_b,      neg_a,      neg_b,      negb_a,     negb_b,     not_a,      not_b,
	// 0x0400
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x0500
	sext0_a,    sext0_b,    zext0_a,    zext0_b,    setf0,      setf0,      setf0,      setf0,
	move0_ra_a, move0_ra_b, move0_ar_a, move0_ar_b, move0_aa,   unimpl,     movb_ra_a,  movb_ra_b,
	// 0x0600
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	// 0x0700
	sext1_a,    sext1_b,    zext1_a,    zext1_b,    setf1,      setf1,      setf1,      setf1,
	move1_ra_a, move1_ra_b, move1_ar_a, move1_ar_b, move1_aa,   unimpl,     movb_ar_a,  movb_ar_b,
	// 0x0800
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x0900
	trap,       trap,       call_a,     call_b,     reti,       unimpl,     rets,       rets,
	mmtm_a,     mmtm_b,     mmfm_a,     mmfm_b,     movi_w_a,   movi_w_b,   movi_l_a,   movi_l_b,
	// 0x0a00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x0b00
	addi_w_a,   addi_w_b,   addi_l_a,   addi_l_b,   cmpi_w_a,   cmpi_w_b,   cmpi_l_a,   cmpi_l_b,
	andi_a,     andi_b,     ori_a,      ori_b,      xori_a,     xori_b,     subi_w_a,   subi_w_b,
	// 0x0c00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x0d00
	subi_l_a,   subi_l_b,   unimpl,     callr,      unimpl,     calla,      eint,       unimpl,
	dsj_a,      dsj_b,      dsjeq_a,    dsjeq_b,    dsjne_a,    dsjne_b,    setc,       unimpl,
	// 0x0e00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x0f00
	pixblt_l_l, unimpl,     pixblt_l_xy,unimpl,     pixblt_xy_l,unimpl,     pixblt_xy_xy,unimpl,
	pixblt_b_l, unimpl,     pixblt_b_xy,unimpl,     fill_l,     unimpl,     fill_xy,    unimpl,
	// 0x1000
	addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,
	addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,
	// 0x1100
	addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,
	addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,
	// 0x1200
	addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,
	addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,
	// 0x1300
	addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,
	addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,     addk_a,     addk_b,
	// 0x1400
	subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,
	subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,
	// 0x1500
	subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,
	subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,
	// 0x1600
	subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,
	subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,
	// 0x1700
	subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,
	subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,     subk_a,     subk_b,
	// 0x1800
	movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,
	movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,
	// 0x1900
	movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,
	movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,
	// 0x1a00
	movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,
	movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,
	// 0x1b00
	movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,
	movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,     movk_a,     movk_b,
	// 0x1c00
	btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,
	btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,
	// 0x1d00
	btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,
	btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,
	// 0x1e00
	btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,
	btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,
	// 0x1f00
	btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,
	btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,   btst_k_a,   btst_k_b,
	// 0x2000
	sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,
	sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,
	// 0x2100
	sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,
	sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,
	// 0x2200
	sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,
	sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,
	// 0x2300
	sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,
	sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,    sla_k_a,    sla_k_b,
	// 0x2400
	sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,
	sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,
	// 0x2500
	sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,
	sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,
	// 0x2600
	sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,
	sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,
	// 0x2700
	sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,
	sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,    sll_k_a,    sll_k_b,
	// 0x2800
	sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,
	sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,
	// 0x2900
	sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,
	sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,
	// 0x2a00
	sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,
	sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,
	// 0x2b00
	sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,
	sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,    sra_k_a,    sra_k_b,
	// 0x2c00
	srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,
	srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,
	// 0x2d00
	srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,
	srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,
	// 0x2e00
	srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,
	srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,
	// 0x2f00
	srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,
	srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,    srl_k_a,    srl_k_b,
	// 0x3000
	rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,
	rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,
	// 0x3100
	rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,
	rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,
	// 0x3200
	rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,
	rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,
	// 0x3300
	rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,
	rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,     rl_k_a,     rl_k_b,
	// 0x3400
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x3500
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x3600
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x3700
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x3800
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	// 0x3900
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	// 0x3a00
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	// 0x3b00
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	// 0x3c00
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	// 0x3d00
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	// 0x3e00
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	// 0x3f00
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,     dsjs_a,     dsjs_b,
	// 0x4000
	add_a,      add_b,      add_a,      add_b,      add_a,      add_b,      add_a,      add_b,
	add_a,      add_b,      add_a,      add_b,      add_a,      add_b,      add_a,      add_b,
	// 0x4100
	add_a,      add_b,      add_a,      add_b,      add_a,      add_b,      add_a,      add_b,
	add_a,      add_b,      add_a,      add_b,      add_a,      add_b,      add_a,      add_b,
	// 0x4200
	addc_a,     addc_b,     addc_a,     addc_b,     addc_a,     addc_b,     addc_a,     addc_b,
	addc_a,     addc_b,     addc_a,     addc_b,     addc_a,     addc_b,     addc_a,     addc_b,
	// 0x4300
	addc_a,     addc_b,     addc_a,     addc_b,     addc_a,     addc_b,     addc_a,     addc_b,
	addc_a,     addc_b,     addc_a,     addc_b,     addc_a,     addc_b,     addc_a,     addc_b,
	// 0x4400
	sub_a,      sub_b,      sub_a,      sub_b,      sub_a,      sub_b,      sub_a,      sub_b,
	sub_a,      sub_b,      sub_a,      sub_b,      sub_a,      sub_b,      sub_a,      sub_b,
	// 0x4500
	sub_a,      sub_b,      sub_a,      sub_b,      sub_a,      sub_b,      sub_a,      sub_b,
	sub_a,      sub_b,      sub_a,      sub_b,      sub_a,      sub_b,      sub_a,      sub_b,
	// 0x4600
	subb_a,     subb_b,     subb_a,     subb_b,     subb_a,     subb_b,     subb_a,     subb_b,
	subb_a,     subb_b,     subb_a,     subb_b,     subb_a,     subb_b,     subb_a,     subb_b,
	// 0x4700
	subb_a,     subb_b,     subb_a,     subb_b,     subb_a,     subb_b,     subb_a,     subb_b,
	subb_a,     subb_b,     subb_a,     subb_b,     subb_a,     subb_b,     subb_a,     subb_b,
	// 0x4800
	cmp_a,      cmp_b,      cmp_a,      cmp_b,      cmp_a,      cmp_b,      cmp_a,      cmp_b,
	cmp_a,      cmp_b,      cmp_a,      cmp_b,      cmp_a,      cmp_b,      cmp_a,      cmp_b,
	// 0x4900
	cmp_a,      cmp_b,      cmp_a,      cmp_b,      cmp_a,      cmp_b,      cmp_a,      cmp_b,
	cmp_a,      cmp_b,      cmp_a,      cmp_b,      cmp_a,      cmp_b,      cmp_a,      cmp_b,
	// 0x4a00
	btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,
	btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,
	// 0x4b00
	btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,
	btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,   btst_r_a,   btst_r_b,
	// 0x4c00
	move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,
	move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,
	// 0x4d00
	move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,
	move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,  move_rr_a,  move_rr_b,
	// 0x4e00
	move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx,
	move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx,
	// 0x4f00
	move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx,
	move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx, move_rr_ax, move_rr_bx,
	// 0x5000
	and_a,      and_b,      and_a,      and_b,      and_a,      and_b,      and_a,      and_b,
	and_a,      and_b,      and_a,      and_b,      and_a,      and_b,      and_a,      and_b,
	// 0x5100
	and_a,      and_b,      and_a,      and_b,      and_a,      and_b,      and_a,      and_b,
	and_a,      and_b,      and_a,      and_b,      and_a,      and_b,      and_a,      and_b,
	// 0x5200
	andn_a,     andn_b,     andn_a,     andn_b,     andn_a,     andn_b,     andn_a,     andn_b,
	andn_a,     andn_b,     andn_a,     andn_b,     andn_a,     andn_b,     andn_a,     andn_b,
	// 0x5300
	andn_a,     andn_b,     andn_a,     andn_b,     andn_a,     andn_b,     andn_a,     andn_b,
	andn_a,     andn_b,     andn_a,     andn_b,     andn_a,     andn_b,     andn_a,     andn_b,
	// 0x5400
	or_a,       or_b,       or_a,       or_b,       or_a,       or_b,       or_a,       or_b,
	or_a,       or_b,       or_a,       or_b,       or_a,       or_b,       or_a,       or_b,
	// 0x5500
	or_a,       or_b,       or_a,       or_b,       or_a,       or_b,       or_a,       or_b,
	or_a,       or_b,       or_a,       or_b,       or_a,       or_b,       or_a,       or_b,
	// 0x5600
	xor_a,      xor_b,      xor_a,      xor_b,      xor_a,      xor_b,      xor_a,      xor_b,
	xor_a,      xor_b,      xor_a,      xor_b,      xor_a,      xor_b,      xor_a,      xor_b,
	// 0x5700
	xor_a,      xor_b,      xor_a,      xor_b,      xor_a,      xor_b,      xor_a,      xor_b,
	xor_a,      xor_b,      xor_a,      xor_b,      xor_a,      xor_b,      xor_a,      xor_b,
	// 0x5800
	divs_a,     divs_b,     divs_a,     divs_b,     divs_a,     divs_b,     divs_a,     divs_b,
	divs_a,     divs_b,     divs_a,     divs_b,     divs_a,     divs_b,     divs_a,     divs_b,
	// 0x5900
	divs_a,     divs_b,     divs_a,     divs_b,     divs_a,     divs_b,     divs_a,     divs_b,
	divs_a,     divs_b,     divs_a,     divs_b,     divs_a,     divs_b,     divs_a,     divs_b,
	// 0x5a00
	divu_a,     divu_b,     divu_a,     divu_b,     divu_a,     divu_b,     divu_a,     divu_b,
	divu_a,     divu_b,     divu_a,     divu_b,     divu_a,     divu_b,     divu_a,     divu_b,
	// 0x5b00
	divu_a,     divu_b,     divu_a,     divu_b,     divu_a,     divu_b,     divu_a,     divu_b,
	divu_a,     divu_b,     divu_a,     divu_b,     divu_a,     divu_b,     divu_a,     divu_b,
	// 0x5c00
	mpys_a,     mpys_b,     mpys_a,     mpys_b,     mpys_a,     mpys_b,     mpys_a,     mpys_b,
	mpys_a,     mpys_b,     mpys_a,     mpys_b,     mpys_a,     mpys_b,     mpys_a,     mpys_b,
	// 0x5d00
	mpys_a,     mpys_b,     mpys_a,     mpys_b,     mpys_a,     mpys_b,     mpys_a,     mpys_b,
	mpys_a,     mpys_b,     mpys_a,     mpys_b,     mpys_a,     mpys_b,     mpys_a,     mpys_b,
	// 0x5e00
	mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,
	mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,
	// 0x5f00
	mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,
	mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,     mpyu_a,     mpyu_b,
	// 0x6000
	sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,
	sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,
	// 0x6100
	sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,
	sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,    sla_r_a,    sla_r_b,
	// 0x6200
	sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,
	sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,
	// 0x6300
	sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,
	sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,    sll_r_a,    sll_r_b,
	// 0x6400
	sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,
	sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,
	// 0x6500
	sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,
	sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,    sra_r_a,    sra_r_b,
	// 0x6600
	srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,
	srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,
	// 0x6700
	srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,
	srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,    srl_r_a,    srl_r_b,
	// 0x6800
	rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,
	rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,
	// 0x6900
	rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,
	rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,     rl_r_a,     rl_r_b,
	// 0x6a00
	lmo_a,      lmo_b,      lmo_a,      lmo_b,      lmo_a,      lmo_b,      lmo_a,      lmo_b,
	lmo_a,      lmo_b,      lmo_a,      lmo_b,      lmo_a,      lmo_b,      lmo_a,      lmo_b,
	// 0x6b00
	lmo_a,      lmo_b,      lmo_a,      lmo_b,      lmo_a,      lmo_b,      lmo_a,      lmo_b,
	lmo_a,      lmo_b,      lmo_a,      lmo_b,      lmo_a,      lmo_b,      lmo_a,      lmo_b,
	// 0x6c00
	mods_a,     mods_b,     mods_a,     mods_b,     mods_a,     mods_b,     mods_a,     mods_b,
	mods_a,     mods_b,     mods_a,     mods_b,     mods_a,     mods_b,     mods_a,     mods_b,
	// 0x6d00
	mods_a,     mods_b,     mods_a,     mods_b,     mods_a,     mods_b,     mods_a,     mods_b,
	mods_a,     mods_b,     mods_a,     mods_b,     mods_a,     mods_b,     mods_a,     mods_b,
	// 0x6e00
	modu_a,     modu_b,     modu_a,     modu_b,     modu_a,     modu_b,     modu_a,     modu_b,
	modu_a,     modu_b,     modu_a,     modu_b,     modu_a,     modu_b,     modu_a,     modu_b,
	// 0x6f00
	modu_a,     modu_b,     modu_a,     modu_b,     modu_a,     modu_b,     modu_a,     modu_b,
	modu_a,     modu_b,     modu_a,     modu_b,     modu_a,     modu_b,     modu_a,     modu_b,
	// 0x7000
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7100
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7200
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7300
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7400
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7500
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7600
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7700
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7800
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7900
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7a00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7b00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7c00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7d00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7e00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x7f00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x8000
	move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b,
	move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b,
	// 0x8100
	move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b,
	move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b, move0_rn_a, move0_rn_b,
	// 0x8200
	move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b,
	move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b,
	// 0x8300
	move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b,
	move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b, move1_rn_a, move1_rn_b,
	// 0x8400
	move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b,
	move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b,
	// 0x8500
	move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b,
	move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b, move0_nr_a, move0_nr_b,
	// 0x8600
	move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b,
	move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b,
	// 0x8700
	move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b,
	move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b, move1_nr_a, move1_nr_b,
	// 0x8800
	move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b,
	move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b,
	// 0x8900
	move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b,
	move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b, move0_nn_a, move0_nn_b,
	// 0x8a00
	move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b,
	move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b,
	// 0x8b00
	move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b,
	move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b, move1_nn_a, move1_nn_b,
	// 0x8c00
	movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,
	movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,
	// 0x8d00
	movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,
	movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,  movb_rn_a,  movb_rn_b,
	// 0x8e00
	movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,
	movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,
	// 0x8f00
	movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,
	movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,  movb_nr_a,  movb_nr_b,
	// 0x9000
	move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,
	move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,
	// 0x9100
	move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,
	move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,   move0_r_ni_a,   move0_r_ni_b,
	// 0x9200
	move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,
	move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,
	// 0x9300
	move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,
	move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,   move1_r_ni_a,   move1_r_ni_b,
	// 0x9400
	move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,
	move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,
	// 0x9500
	move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,
	move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,   move0_ni_r_a,   move0_ni_r_b,
	// 0x9600
	move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,
	move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,
	// 0x9700
	move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,
	move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,   move1_ni_r_a,   move1_ni_r_b,
	// 0x9800
	move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,
	move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,
	// 0x9900
	move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,
	move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,  move0_ni_ni_a,  move0_ni_ni_b,
	// 0x9a00
	move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,
	move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,
	// 0x9b00
	move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,
	move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,  move1_ni_ni_a,  move1_ni_ni_b,
	// 0x9c00
	movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,
	movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,
	// 0x9d00
	movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,
	movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,  movb_nn_a,  movb_nn_b,
	// 0x9e00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0x9f00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xa000
	move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,
	move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,
	// 0xa100
	move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,
	move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,   move0_r_dn_a,   move0_r_dn_b,
	// 0xa200
	move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,
	move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,
	// 0xa300
	move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,
	move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,   move1_r_dn_a,   move1_r_dn_b,
	// 0xa400
	move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,
	move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,
	// 0xa500
	move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,
	move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,   move0_dn_r_a,   move0_dn_r_b,
	// 0xa600
	move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,
	move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,
	// 0xa700
	move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,
	move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,   move1_dn_r_a,   move1_dn_r_b,
	// 0xa800
	move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,
	move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,
	// 0xa900
	move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,
	move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,  move0_dn_dn_a,  move0_dn_dn_b,
	// 0xaa00
	move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,
	move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,
	// 0xab00
	move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,
	move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,  move1_dn_dn_a,  move1_dn_dn_b,
	// 0xac00
	movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,
	movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,
	// 0xad00
	movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,
	movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,    movb_r_no_a,    movb_r_no_b,
	// 0xae00
	movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,
	movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,
	// 0xaf00
	movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,
	movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,    movb_no_r_a,    movb_no_r_b,
	// 0xb000
	move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,
	move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,
	// 0xb100
	move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,
	move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,   move0_r_no_a,   move0_r_no_b,
	// 0xb200
	move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,
	move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,
	// 0xb300
	move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,
	move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,   move1_r_no_a,   move1_r_no_b,
	// 0xb400
	move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,
	move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,
	// 0xb500
	move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,
	move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,   move0_no_r_a,   move0_no_r_b,
	// 0xb600
	move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,
	move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,
	// 0xb700
	move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,
	move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,   move1_no_r_a,   move1_no_r_b,
	// 0xb800
	move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,
	move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,
	// 0xb900
	move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,
	move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,  move0_no_no_a,  move0_no_no_b,
	// 0xba00
	move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,
	move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,
	// 0xbb00
	move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,
	move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,  move1_no_no_a,  move1_no_no_b,
	// 0xbc00
	movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,
	movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,
	// 0xbd00
	movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,
	movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,   movb_no_no_a,   movb_no_no_b,
	// 0xbe00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xbf00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xc000
	j_UC_0,     j_UC_x,     j_UC_x,     j_UC_x,     j_UC_x,     j_UC_x,     j_UC_x,     j_UC_x,
	j_UC_8,     j_UC_x,     j_UC_x,     j_UC_x,     j_UC_x,     j_UC_x,     j_UC_x,     j_UC_x,
	// 0xc100
	j_P_0,      j_P_x,      j_P_x,      j_P_x,      j_P_x,      j_P_x,      j_P_x,      j_P_x,
	j_P_8,      j_P_x,      j_P_x,      j_P_x,      j_P_x,      j_P_x,      j_P_x,      j_P_x,
	// 0xc200
	j_LS_0,     j_LS_x,     j_LS_x,     j_LS_x,     j_LS_x,     j_LS_x,     j_LS_x,     j_LS_x,
	j_LS_8,     j_LS_x,     j_LS_x,     j_LS_x,     j_LS_x,     j_LS_x,     j_LS_x,     j_LS_x,
	// 0xc300
	j_HI_0,     j_HI_x,     j_HI_x,     j_HI_x,     j_HI_x,     j_HI_x,     j_HI_x,     j_HI_x,
	j_HI_8,     j_HI_x,     j_HI_x,     j_HI_x,     j_HI_x,     j_HI_x,     j_HI_x,     j_HI_x,
	// 0xc400
	j_LT_0,     j_LT_x,     j_LT_x,     j_LT_x,     j_LT_x,     j_LT_x,     j_LT_x,     j_LT_x,
	j_LT_8,     j_LT_x,     j_LT_x,     j_LT_x,     j_LT_x,     j_LT_x,     j_LT_x,     j_LT_x,
	// 0xc500
	j_GE_0,     j_GE_x,     j_GE_x,     j_GE_x,     j_GE_x,     j_GE_x,     j_GE_x,     j_GE_x,
	j_GE_8,     j_GE_x,     j_GE_x,     j_GE_x,     j_GE_x,     j_GE_x,     j_GE_x,     j_GE_x,
	// 0xc600
	j_LE_0,     j_LE_x,     j_LE_x,     j_LE_x,     j_LE_x,     j_LE_x,     j_LE_x,     j_LE_x,
	j_LE_8,     j_LE_x,     j_LE_x,     j_LE_x,     j_LE_x,     j_LE_x,     j_LE_x,     j_LE_x,
	// 0xc700
	j_GT_0,     j_GT_x,     j_GT_x,     j_GT_x,     j_GT_x,     j_GT_x,     j_GT_x,     j_GT_x,
	j_GT_8,     j_GT_x,     j_GT_x,     j_GT_x,     j_GT_x,     j_GT_x,     j_GT_x,     j_GT_x,
	// 0xc800
	j_C_0,      j_C_x,      j_C_x,      j_C_x,      j_C_x,      j_C_x,      j_C_x,      j_C_x,
	j_C_8,      j_C_x,      j_C_x,      j_C_x,      j_C_x,      j_C_x,      j_C_x,      j_C_x,
	// 0xc900
	j_NC_0,     j_NC_x,     j_NC_x,     j_NC_x,     j_NC_x,     j_NC_x,     j_NC_x,     j_NC_x,
	j_NC_8,     j_NC_x,     j_NC_x,     j_NC_x,     j_NC_x,     j_NC_x,     j_NC_x,     j_NC_x,
	// 0xca00
	j_EQ_0,     j_EQ_x,     j_EQ_x,     j_EQ_x,     j_EQ_x,     j_EQ_x,     j_EQ_x,     j_EQ_x,
	j_EQ_8,     j_EQ_x,     j_EQ_x,     j_EQ_x,     j_EQ_x,     j_EQ_x,     j_EQ_x,     j_EQ_x,
	// 0xcb00
	j_NE_0,     j_NE_x,     j_NE_x,     j_NE_x,     j_NE_x,     j_NE_x,     j_NE_x,     j_NE_x,
	j_NE_8,     j_NE_x,     j_NE_x,     j_NE_x,     j_NE_x,     j_NE_x,     j_NE_x,     j_NE_x,
	// 0xcc00
	j_V_0,      j_V_x,      j_V_x,      j_V_x,      j_V_x,      j_V_x,      j_V_x,      j_V_x,
	j_V_8,      j_V_x,      j_V_x,      j_V_x,      j_V_x,      j_V_x,      j_V_x,      j_V_x,
	// 0xcd00
	j_NV_0,     j_NV_x,     j_NV_x,     j_NV_x,     j_NV_x,     j_NV_x,     j_NV_x,     j_NV_x,
	j_NV_8,     j_NV_x,     j_NV_x,     j_NV_x,     j_NV_x,     j_NV_x,     j_NV_x,     j_NV_x,
	// 0xce00
	j_N_0,      j_N_x,      j_N_x,      j_N_x,      j_N_x,      j_N_x,      j_N_x,      j_N_x,
	j_N_8,      j_N_x,      j_N_x,      j_N_x,      j_N_x,      j_N_x,      j_N_x,      j_N_x,
	// 0xcf00
	j_NN_0,     j_NN_x,     j_NN_x,     j_NN_x,     j_NN_x,     j_NN_x,     j_NN_x,     j_NN_x,
	j_NN_8,     j_NN_x,     j_NN_x,     j_NN_x,     j_NN_x,     j_NN_x,     j_NN_x,     j_NN_x,
	// 0xd000
	move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,
	move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,
	// 0xd100
	move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,
	move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,  move0_no_ni_a,  move0_no_ni_b,
	// 0xd200
	move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,
	move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,
	// 0xd300
	move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,
	move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,  move1_no_ni_a,  move1_no_ni_b,
	// 0xd400
	move0_a_ni_a,move0_a_ni_b,unimpl,   unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	// 0xd500
	exgf0_a,    exgf0_b,    unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	// 0xd600
	move1_a_ni_a,move1_a_ni_b,unimpl,   unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	// 0xd700
	exgf1_a,    exgf1_b,    unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	// 0xd800
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xd900
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xda00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xdb00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xdc00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xdd00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xde00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xdf00
	unimpl,     line,       unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	unimpl,     line,       unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	// 0xe000
	add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,
	add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,
	// 0xe100
	add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,
	add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,   add_xy_a,   add_xy_b,
	// 0xe200
	sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,
	sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,
	// 0xe300
	sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,
	sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,   sub_xy_a,   sub_xy_b,
	// 0xe400
	cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,
	cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,
	// 0xe500
	cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,
	cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,   cmp_xy_a,   cmp_xy_b,
	// 0xe600
	cpw_a,      cpw_b,      cpw_a,      cpw_b,      cpw_a,      cpw_b,      cpw_a,      cpw_b,
	cpw_a,      cpw_b,      cpw_a,      cpw_b,      cpw_a,      cpw_b,      cpw_a,      cpw_b,
	// 0xe700
	cpw_a,      cpw_b,      cpw_a,      cpw_b,      cpw_a,      cpw_b,      cpw_a,      cpw_b,
	cpw_a,      cpw_b,      cpw_a,      cpw_b,      cpw_a,      cpw_b,      cpw_a,      cpw_b,
	// 0xe800
	cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,
	cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,
	// 0xe900
	cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,
	cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,    cvxyl_a,    cvxyl_b,
	// 0xea00
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	// 0xeb00
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,     unimpl,
	// 0xec00
	movx_a,     movx_b,     movx_a,     movx_b,     movx_a,     movx_b,     movx_a,     movx_b,
	movx_a,     movx_b,     movx_a,     movx_b,     movx_a,     movx_b,     movx_a,     movx_b,
	// 0xed00
	movx_a,     movx_b,     movx_a,     movx_b,     movx_a,     movx_b,     movx_a,     movx_b,
	movx_a,     movx_b,     movx_a,     movx_b,     movx_a,     movx_b,     movx_a,     movx_b,
	// 0xee00
	movy_a,     movy_b,     movy_a,     movy_b,     movy_a,     movy_b,     movy_a,     movy_b,
	movy_a,     movy_b,     movy_a,     movy_b,     movy_a,     movy_b,     movy_a,     movy_b,
	// 0xef00
	movy_a,     movy_b,     movy_a,     movy_b,     movy_a,     movy_b,     movy_a,     movy_b,
	movy_a,     movy_b,     movy_a,     movy_b,     movy_a,     movy_b,     movy_a,     movy_b,
	// 0xf000
	pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,
	pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,
	// 0xf100
	pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,
	pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,    pixt_rixy_a,    pixt_rixy_b,
	// 0xf200
	pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,
	pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,
	// 0xf300
	pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,
	pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,    pixt_ixyr_a,    pixt_ixyr_b,
	// 0xf400
	pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,
	pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,
	// 0xf500
	pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,
	pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,  pixt_ixyixy_a,  pixt_ixyixy_b,
	// 0xf600
	drav_a,     drav_b,     drav_a,     drav_b,     drav_a,     drav_b,     drav_a,     drav_b,
	drav_a,     drav_b,     drav_a,     drav_b,     drav_a,     drav_b,     drav_a,     drav_b,
	// 0xf700
	drav_a,     drav_b,     drav_a,     drav_b,     drav_a,     drav_b,     drav_a,     drav_b,
	drav_a,     drav_b,     drav_a,     drav_b,     drav_a,     drav_b,     drav_a,     drav_b,
	// 0xf800
	pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,
	pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,
	// 0xf900
	pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,
	pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,  pixt_ri_a,  pixt_ri_b,
	// 0xfa00
	pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,
	pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,
	// 0xfb00
	pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,
	pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,  pixt_ir_a,  pixt_ir_b,
	// 0xfc00
	pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,
	pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,
	// 0xfd00
	pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,
	pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,  pixt_ii_a,  pixt_ii_b,
	// 0xfe00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	// 0xff00
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop,
	illop,      illop,      illop,      illop,      illop,      illop,      illop,    illop
};

/* includes the actual opcode implementations */
/* ---- 34010ops.hxx ---- */
// license:BSD-3-Clause
// copyright-holders:Alex Pasadyn,Zsolt Vasvari
/***************************************************************************

    TMS34010: Portable Texas Instruments TMS34010 emulator

    Copyright Alex Pasadyn/Zsolt Vasvari
    Parts based on code by Aaron Giles

***************************************************************************/



/***************************************************************************
    MISC MACROS
***************************************************************************/

#define ZEXTEND(val,width) if (width) (val) &= ((uint32_t)0xffffffff >> (32 - (width)))
#define SEXTEND(val,width) if (width) (val) = (int32_t)((val) << (32 - (width))) >> (32 - (width))

#define SXYTOL(val)   ((((int16_t)(val).y * m_convsp) + ((int16_t)(val).x << m_pixelshift)) + OFFSET())
#define DXYTOL(val)   ((((int16_t)(val).y * m_convdp) + ((int16_t)(val).x << m_pixelshift)) + OFFSET())
#define MXYTOL(val)   ((((int16_t)(val).y * m_convmp) + ((int16_t)(val).x << m_pixelshift)) + OFFSET())

#define COUNT_CYCLES(x)   m_icount -= x
#define COUNT_UNKNOWN_CYCLES(x) COUNT_CYCLES(x)

#define CORRECT_ODD_PC(x) do { if (m_pc & 0x0f) TMS_LOGERR("%s to PC=%08X\n", x, m_pc); m_pc &= ~0x0f; } while (0)



/***************************************************************************
    FLAG HANDLING MACROS
***************************************************************************/

#define SIGN(val)           ((val) & 0x80000000)

#define CLR_Z()                m_st &= ~STBIT_Z
#define CLR_V()                m_st &= ~STBIT_V
#define CLR_C()                m_st &= ~STBIT_C
#define CLR_N()                m_st &= ~STBIT_N
#define CLR_NZ()               m_st &= ~(STBIT_N | STBIT_Z)
#define CLR_CZ()               m_st &= ~(STBIT_C | STBIT_Z)
#define CLR_ZV()               m_st &= ~(STBIT_Z | STBIT_V)
#define CLR_NZV()              m_st &= ~(STBIT_N | STBIT_Z | STBIT_V)
#define CLR_NCZ()              m_st &= ~(STBIT_N | STBIT_C | STBIT_Z)
#define CLR_NCZV()             m_st &= ~(STBIT_N | STBIT_C | STBIT_Z | STBIT_V)

#define SET_V_BIT_LO(val,bit) m_st |= ((val) << (28 - (bit))) & STBIT_V
#define SET_V_BIT_HI(val,bit) m_st |= ((val) >> ((bit) - 28)) & STBIT_V
#define SET_V_LOG(val)        m_st |= (val) << 28
#define SET_Z_BIT_LO(val,bit) m_st |= ((val) << (29 - (bit))) & STBIT_Z
#define SET_Z_BIT_HI(val,bit) m_st |= ((val) >> ((bit) - 29)) & STBIT_Z
#define SET_Z_LOG(val)        m_st |= (val) << 29
#define SET_C_BIT_LO(val,bit) m_st |= ((val) << (30 - (bit))) & STBIT_C
#define SET_C_BIT_HI(val,bit) m_st |= ((val) >> ((bit) - 30)) & STBIT_C
#define SET_C_LOG(val)        m_st |= (val) << 30
#define SET_N_BIT(val,bit)    m_st |= ((val) << (31 - (bit))) & STBIT_N
#define SET_N_LOG(val)        m_st |= (val) << 31

#define SET_Z_VAL(val)        SET_Z_LOG((val) == 0)
#define SET_N_VAL(val)        SET_N_BIT(val, 31)
#define SET_NZ_VAL(val)       SET_Z_VAL(val); SET_N_VAL(val)
#define SET_V_SUB(a,b,r)      SET_V_BIT_HI(((a) ^ (b)) & ((a) ^ (r)), 31)
#define SET_V_ADD(a,b,r)      SET_V_BIT_HI(~((a) ^ (b)) & ((a) ^ (r)), 31)
#define SET_C_SUB(a,b)        SET_C_LOG((uint32_t)(b) > (uint32_t)(a))
#define SET_C_ADD(a,b)        SET_C_LOG((uint32_t)~(a) < (uint32_t)(b))
#define SET_NZV_SUB(a,b,r)    SET_NZ_VAL(r); SET_V_SUB(a,b,r)
#define SET_NZCV_SUB(a,b,r)   SET_NZV_SUB(a,b,r); SET_C_SUB(a,b)
#define SET_NZCV_ADD(a,b,r)   SET_NZ_VAL(r); SET_V_ADD(a,b,r); SET_C_ADD(a,b)

static const uint8_t fw_inc[32] = { 32,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 };


/***************************************************************************
    UNIMPLEMENTED INSTRUCTION
***************************************************************************/

static void unimpl(uint16_t op)
{
	// Not all illegal opcodes cause TRAP 30 exceptions on TMS34010, and some games only work because of this:

	// 9 Ball Shootout calls to FFDF7468, expecting it
	// to execute the next instruction from FFDF7470
	// but the instruction at FFDF7460 is an 0x0001

	// Super High Impact executes 0x0007 at various entry points

	TMS_LOGERR("TMS34010 reserved opcode %04Xh encountered at %08x\n", op, m_ppc);
	COUNT_CYCLES(1);
}

static void illop(uint16_t op)
{
	PUSH(m_pc);
	PUSH(m_st);
	RESET_ST();
	m_pc = RLONG(0xfffffc20);
	COUNT_UNKNOWN_CYCLES(16);

	/* extra check to prevent bad things */
#if 0
	if (m_pc == 0 || s_opcode_table[space(AS_PROGRAM).read_word(m_pc) >> 4] == unimpl)
	{
		set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		
	}
#endif
}


/***************************************************************************
    X/Y OPERATIONS
***************************************************************************/

#define ADD_XY(R)                               \
{                                               \
	XY  a =  R##REG_XY(SRCREG(op));                 \
	XY *b = &R##REG_XY(DSTREG(op));                 \
	CLR_NCZV();                                  \
	b->x += a.x;                                \
	b->y += a.y;                                \
	SET_N_LOG(b->x == 0);                      \
	SET_C_BIT_LO(b->y, 15);                        \
	SET_Z_LOG(b->y == 0);                      \
	SET_V_BIT_LO(b->x, 15);                        \
	COUNT_CYCLES(1);                            \
}
static void add_xy_a(uint16_t op) { ADD_XY(A); }
static void add_xy_b(uint16_t op) { ADD_XY(B); }

#define SUB_XY(R)                               \
{                                               \
	XY  a =  R##REG_XY(SRCREG(op));                 \
	XY *b = &R##REG_XY(DSTREG(op));                 \
	CLR_NCZV();                                  \
	SET_N_LOG(a.x == b->x);                        \
	SET_C_LOG(a.y > b->y);                     \
	SET_Z_LOG(a.y == b->y);                        \
	SET_V_LOG(a.x > b->x);                     \
	b->x -= a.x;                                \
	b->y -= a.y;                                \
	COUNT_CYCLES(1);                            \
}
static void sub_xy_a(uint16_t op) { SUB_XY(A); }
static void sub_xy_b(uint16_t op) { SUB_XY(B); }

#define CMP_XY(R)                               \
{                                               \
	int16_t res;                                  \
	XY a = R##REG_XY(DSTREG(op));                   \
	XY b = R##REG_XY(SRCREG(op));                   \
	CLR_NCZV();                                  \
	res = a.x-b.x;                              \
	SET_N_LOG(res == 0);                       \
	SET_V_BIT_LO(res, 15);                     \
	res = a.y-b.y;                              \
	SET_Z_LOG(res == 0);                       \
	SET_C_BIT_LO(res, 15);                     \
	COUNT_CYCLES(1);                            \
}
static void cmp_xy_a(uint16_t op) { CMP_XY(A); }
static void cmp_xy_b(uint16_t op) { CMP_XY(B); }

#define CPW(R)                                  \
{                                               \
	int32_t res = 0;                              \
	int16_t x = R##REG_X(SRCREG(op));                 \
	int16_t y = R##REG_Y(SRCREG(op));                 \
												\
	CLR_V();                                     \
	res |= ((WSTART_X() > x) ? 0x20  : 0);       \
	res |= ((x > WEND_X())   ? 0x40  : 0);       \
	res |= ((WSTART_Y() > y) ? 0x80  : 0);       \
	res |= ((y > WEND_Y())   ? 0x100 : 0);       \
	R##REG(DSTREG(op)) = res;                       \
	SET_V_LOG(res != 0);                       \
	COUNT_CYCLES(1);                            \
}
static void cpw_a(uint16_t op) { CPW(A); }
static void cpw_b(uint16_t op) { CPW(B); }

// TODO: Handle cycles properly, currently we assume power of two
#define CVXYL(R)                                    \
{                                                   \
	R##REG(DSTREG(op)) = DXYTOL(R##REG_XY(SRCREG(op)));     \
	COUNT_CYCLES(3);                                \
}
static void cvxyl_a(uint16_t op) { CVXYL(A); }
static void cvxyl_b(uint16_t op) { CVXYL(B); }

#define MOVX(R)                                     \
{                                                   \
	R##REG(DSTREG(op)) = (R##REG(DSTREG(op)) & 0xffff0000) | (uint16_t)R##REG(SRCREG(op));    \
	COUNT_CYCLES(1);                                                                    \
}
static void movx_a(uint16_t op) { MOVX(A); }
static void movx_b(uint16_t op) { MOVX(B); }

#define MOVY(R)                                     \
{                                                   \
	R##REG(DSTREG(op)) = (R##REG(SRCREG(op)) & 0xffff0000) | (uint16_t)R##REG(DSTREG(op));    \
	COUNT_CYCLES(1);                                                                    \
}
static void movy_a(uint16_t op) { MOVY(A); }
static void movy_b(uint16_t op) { MOVY(B); }



/***************************************************************************
    PIXEL TRANSFER OPERATIONS
***************************************************************************/

#define PIXT_RI(R)                                  \
{                                                   \
	WPIXEL(R##REG(DSTREG(op)),R##REG(SRCREG(op)));  \
	COUNT_UNKNOWN_CYCLES(2);                        \
}
static void pixt_ri_a(uint16_t op) { PIXT_RI(A); }
static void pixt_ri_b(uint16_t op) { PIXT_RI(B); }

#define PIXT_RIXY(R)                                                                \
{                                                                                   \
	if (WINDOW_CHECKING() != 0)                                                      \
	{                                                                               \
		CLR_V();                                                                     \
		if (R##REG_X(DSTREG(op)) < WSTART_X() || R##REG_X(DSTREG(op)) > WEND_X() ||               \
			R##REG_Y(DSTREG(op)) < WSTART_Y() || R##REG_Y(DSTREG(op)) > WEND_Y())             \
		{                                                                           \
			SET_V_LOG(1);                                                          \
			goto skip;                                                              \
		}                                                                           \
		if (WINDOW_CHECKING() == 1) goto skip;                                       \
	}                                                                               \
	WPIXEL(DXYTOL(R##REG_XY(DSTREG(op))),R##REG(SRCREG(op)));                               \
skip:                                                                               \
	COUNT_UNKNOWN_CYCLES(4);                                                        \
}
static void pixt_rixy_a(uint16_t op) { PIXT_RIXY(A); }
static void pixt_rixy_b(uint16_t op) { PIXT_RIXY(B); }

#define PIXT_IR(R)                                  \
{                                                   \
	int32_t temp = RPIXEL(R##REG(SRCREG(op)));            \
	CLR_V();                                         \
	R##REG(DSTREG(op)) = temp;                          \
	SET_V_LOG(temp != 0);                          \
	COUNT_CYCLES(4);                                \
}
static void pixt_ir_a(uint16_t op) { PIXT_IR(A); }
static void pixt_ir_b(uint16_t op) { PIXT_IR(B); }

#define PIXT_II(R)                                  \
{                                                   \
	WPIXEL(R##REG(DSTREG(op)),RPIXEL(R##REG(SRCREG(op))));  \
	COUNT_UNKNOWN_CYCLES(4);                        \
}
static void pixt_ii_a(uint16_t op) { PIXT_II(A); }
static void pixt_ii_b(uint16_t op) { PIXT_II(B); }

#define PIXT_IXYR(R)                                \
{                                                   \
	int32_t temp = RPIXEL(SXYTOL(R##REG_XY(SRCREG(op)))); \
	CLR_V();                                         \
	R##REG(DSTREG(op)) = temp;                          \
	SET_V_LOG(temp != 0);                          \
	COUNT_CYCLES(6);                                \
}
static void pixt_ixyr_a(uint16_t op) { PIXT_IXYR(A); }
static void pixt_ixyr_b(uint16_t op) { PIXT_IXYR(B); }

#define PIXT_IXYIXY(R)                                                              \
{                                                                                   \
	if (WINDOW_CHECKING() != 0)                                                      \
	{                                                                               \
		CLR_V();                                                                     \
		if (R##REG_X(DSTREG(op)) < WSTART_X() || R##REG_X(DSTREG(op)) > WEND_X() ||               \
			R##REG_Y(DSTREG(op)) < WSTART_Y() || R##REG_Y(DSTREG(op)) > WEND_Y())             \
		{                                                                           \
			SET_V_LOG(1);                                                          \
			goto skip;                                                              \
		}                                                                           \
		if (WINDOW_CHECKING() == 1) goto skip;                                       \
	}                                                                               \
	WPIXEL(DXYTOL(R##REG_XY(DSTREG(op))),RPIXEL(SXYTOL(R##REG_XY(SRCREG(op)))));            \
skip:                                                                               \
	COUNT_UNKNOWN_CYCLES(7);                                                        \
}
static void pixt_ixyixy_a(uint16_t op) { PIXT_IXYIXY(A); }
static void pixt_ixyixy_b(uint16_t op) { PIXT_IXYIXY(B); }

#define DRAV(R)                                                                     \
{                                                                                   \
	if (WINDOW_CHECKING() != 0)                                                      \
	{                                                                               \
		CLR_V();                                                                     \
		if (R##REG_X(DSTREG(op)) < WSTART_X() || R##REG_X(DSTREG(op)) > WEND_X() ||               \
			R##REG_Y(DSTREG(op)) < WSTART_Y() || R##REG_Y(DSTREG(op)) > WEND_Y())             \
		{                                                                           \
			SET_V_LOG(1);                                                          \
			goto skip;                                                              \
		}                                                                           \
		if (WINDOW_CHECKING() == 1) goto skip;                                       \
	}                                                                               \
	WPIXEL(DXYTOL(R##REG_XY(DSTREG(op))),COLOR1());                                      \
skip:                                                                               \
	R##REG_X(DSTREG(op)) += R##REG_X(SRCREG(op));                                           \
	R##REG_Y(DSTREG(op)) += R##REG_Y(SRCREG(op));                                           \
	COUNT_UNKNOWN_CYCLES(4);                                                        \
}
static void drav_a(uint16_t op) { DRAV(A); }
static void drav_b(uint16_t op) { DRAV(B); }



/***************************************************************************
    ARITHMETIC OPERATIONS
***************************************************************************/

#define ABS(R)                                              \
{                                                           \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	int32_t r = 0 - *rd;                                      \
	CLR_NZV();                                               \
	if (r > 0) *rd = r;                                     \
	SET_NZ_VAL(r);                                         \
	SET_V_LOG(r == (int32_t)0x80000000);                     \
	COUNT_CYCLES(1);                                        \
}
static void abs_a(uint16_t op) { ABS(A); }
static void abs_b(uint16_t op) { ABS(B); }

#define ADD(R)                                              \
{                                                           \
	int32_t a = R##REG(SRCREG(op));                               \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	int32_t b = *rd;                                          \
	int32_t r = a + b;                                        \
	CLR_NCZV();                                              \
	*rd = r;                                                \
	SET_NZCV_ADD(a,b,r);                                    \
	COUNT_CYCLES(1);                                        \
}
static void add_a(uint16_t op) { ADD(A); }
static void add_b(uint16_t op) { ADD(B); }

#define ADDC(R)                                             \
{                                                           \
	/* I'm not sure to which side the carry is added to, should */  \
	/* verify it against the examples */                    \
	int32_t a = R##REG(SRCREG(op));                               \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	int32_t b = *rd;                                          \
	int32_t r = a + b + (C_FLAG() ? 1 : 0);                        \
	CLR_NCZV();                                              \
	*rd = r;                                                \
	SET_NZCV_ADD(a,b,r);                                    \
	COUNT_CYCLES(1);                                        \
}
static void addc_a(uint16_t op) { ADDC(A); }
static void addc_b(uint16_t op) { ADDC(B); }

#define ADDI_W(R)                                           \
{                                                           \
	int32_t a = PARAM_WORD();                              \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	int32_t b = *rd;                                          \
	int32_t r = a + b;                                        \
	CLR_NCZV();                                              \
	*rd = r;                                                \
	SET_NZCV_ADD(a,b,r);                                    \
	COUNT_CYCLES(2);                                        \
}
static void addi_w_a(uint16_t op) { ADDI_W(A); }
static void addi_w_b(uint16_t op) { ADDI_W(B); }

#define ADDI_L(R)                                           \
{                                                           \
	int32_t a = PARAM_LONG();                              \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	int32_t b = *rd;                                          \
	int32_t r = a + b;                                        \
	CLR_NCZV();                                              \
	*rd = r;                                                \
	SET_NZCV_ADD(a,b,r);                                    \
	COUNT_CYCLES(3);                                        \
}
static void addi_l_a(uint16_t op) { ADDI_L(A); }
static void addi_l_b(uint16_t op) { ADDI_L(B); }

#define ADDK(R)                                             \
{                                                           \
	int32_t a = fw_inc[PARAM_K(op)];                              \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	int32_t b = *rd;                                          \
	int32_t r = a + b;                                        \
	CLR_NCZV();                                              \
	*rd = r;                                                \
	SET_NZCV_ADD(a,b,r);                                    \
	COUNT_CYCLES(1);                                        \
}
static void addk_a(uint16_t op) { ADDK(A); }
static void addk_b(uint16_t op) { ADDK(B); }

#define AND(R)                                              \
{                                                           \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	CLR_Z();                                                 \
	*rd &= R##REG(SRCREG(op));                                  \
	SET_Z_VAL(*rd);                                            \
	COUNT_CYCLES(1);                                        \
}
static void and_a(uint16_t op) { AND(A); }
static void and_b(uint16_t op) { AND(B); }

#define ANDI(R)                                             \
{                                                           \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	CLR_Z();                                                 \
	*rd &= ~PARAM_LONG();                                \
	SET_Z_VAL(*rd);                                            \
	COUNT_CYCLES(3);                                        \
}
static void andi_a(uint16_t op) { ANDI(A); }
static void andi_b(uint16_t op) { ANDI(B); }

#define ANDN(R)                                             \
{                                                           \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	CLR_Z();                                                 \
	*rd &= ~R##REG(SRCREG(op));                                 \
	SET_Z_VAL(*rd);                                            \
	COUNT_CYCLES(1);                                        \
}
static void andn_a(uint16_t op) { ANDN(A); }
static void andn_b(uint16_t op) { ANDN(B); }

#define BTST_K(R)                                           \
{                                                           \
	int bit = 31 - PARAM_K(op);                                 \
	CLR_Z();                                                 \
	if (bit <= 29)                                          \
		SET_Z_BIT_LO(~R##REG(DSTREG(op)), bit);                    \
	else                                                    \
		SET_Z_BIT_HI(~R##REG(DSTREG(op)), bit);                    \
	COUNT_CYCLES(1);                                        \
}
static void btst_k_a(uint16_t op) { BTST_K(A); }
static void btst_k_b(uint16_t op) { BTST_K(B); }

#define BTST_R(R)                                           \
{                                                           \
	int bit = R##REG(SRCREG(op)) & 0x1f;                        \
	CLR_Z();                                                 \
	if (bit <= 29)                                          \
		SET_Z_BIT_LO(~R##REG(DSTREG(op)), bit);                    \
	else                                                    \
		SET_Z_BIT_HI(~R##REG(DSTREG(op)), bit);                    \
	COUNT_CYCLES(2);                                        \
}
static void btst_r_a(uint16_t op) { BTST_R(A); }
static void btst_r_b(uint16_t op) { BTST_R(B); }

static void clrc(uint16_t op)
{
	CLR_C();
	COUNT_CYCLES(1);
}

#define CMP(R)                                              \
{                                                           \
	int32_t *rs = &R##REG(SRCREG(op));                            \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	int32_t r = *rd - *rs;                                    \
	CLR_NCZV();                                              \
	SET_NZCV_SUB(*rd,*rs,r);                                \
	COUNT_CYCLES(1);                                        \
}
static void cmp_a(uint16_t op) { CMP(A); }
static void cmp_b(uint16_t op) { CMP(B); }

#define CMPI_W(R)                                           \
{                                                           \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	int32_t t = (int16_t)~PARAM_WORD();                      \
	int32_t r = *rd - t;                                      \
	CLR_NCZV();                                              \
	SET_NZCV_SUB(*rd,t,r);                                  \
	COUNT_CYCLES(2);                                        \
}
static void cmpi_w_a(uint16_t op) { CMPI_W(A); }
static void cmpi_w_b(uint16_t op) { CMPI_W(B); }

#define CMPI_L(R)                                           \
{                                                           \
	int32_t *rd = &R##REG(DSTREG(op));                            \
	int32_t t = ~PARAM_LONG();                             \
	int32_t r = *rd - t;                                      \
	CLR_NCZV();                                              \
	SET_NZCV_SUB(*rd,t,r);                                  \
	COUNT_CYCLES(3);                                        \
}
static void cmpi_l_a(uint16_t op) { CMPI_L(A); }
static void cmpi_l_b(uint16_t op) { CMPI_L(B); }

static void dint(uint16_t op)
{
	m_st &= ~STBIT_IE;
	COUNT_CYCLES(3);
}

#define DIVS(R)                                             \
{                                                           \
	int32_t *rs  = &R##REG(SRCREG(op));                           \
	int32_t *rd1 = &R##REG(DSTREG(op));                           \
	CLR_NZV();                                               \
	if (!(DSTREG(op) & 1))                                      \
	{                                                       \
		if (!*rs)                                           \
		{                                                   \
			SET_V_LOG(1);                                  \
		}                                                   \
		else                                                \
		{                                                   \
			int32_t *rd2 = &R##REG(DSTREG(op)+1);                 \
			int64_t dividend = ((uint64_t)*rd1 << 32) | (uint32_t)*rd2; \
			int64_t quotient = dividend / *rs;                \
			int32_t remainder = dividend % *rs;               \
			if ((int64_t)(int32_t)quotient != quotient)     \
			{                                               \
				SET_V_LOG(1);                              \
			}                                               \
			else                                            \
			{                                               \
				*rd1 = quotient;                            \
				*rd2 = remainder;                           \
				SET_NZ_VAL(*rd1);                          \
			}                                               \
		}                                                   \
		COUNT_CYCLES(40);                                   \
	}                                                       \
	else                                                    \
	{                                                       \
		if (!*rs)                                           \
		{                                                   \
			SET_V_LOG(1);                                  \
		}                                                   \
		else                                                \
		{                                                   \
			*rd1 /= *rs;                                    \
			SET_NZ_VAL(*rd1);                              \
		}                                                   \
		COUNT_CYCLES(39);                                   \
	}                                                       \
}
static void divs_a(uint16_t op) { DIVS(A); }
static void divs_b(uint16_t op) { DIVS(B); }

#define DIVU(R)                                             \
{                                                           \
	int32_t *rs  = &R##REG(SRCREG(op));                           \
	int32_t *rd1 = &R##REG(DSTREG(op));                           \
	CLR_ZV();                                                    \
	if (!(DSTREG(op) & 1))                                      \
	{                                                       \
		if (!*rs)                                           \
		{                                                   \
			SET_V_LOG(1);                                  \
		}                                                   \
		else                                                \
		{                                                   \
			int32_t *rd2 = &R##REG(DSTREG(op)+1);                 \
			uint64_t dividend  = ((uint64_t)*rd1 << 32) | (uint32_t)*rd2; \
			uint64_t quotient  = dividend / (uint32_t)*rs;      \
			uint32_t remainder = dividend % (uint32_t)*rs;      \
			if (quotient > 0xffffffff)                      \
			{                                               \
				SET_V_LOG(1);                              \
			}                                               \
			else                                            \
			{                                               \
				*rd1 = quotient;                            \
				*rd2 = remainder;                           \
				SET_Z_VAL(*rd1);                           \
			}                                               \
		}                                                   \
	}                                                       \
	else                                                    \
	{                                                       \
		if (!*rs)                                           \
		{                                                   \
			SET_V_LOG(1);                                  \
		}                                                   \
		else                                                \
		{                                                   \
			*rd1 = (uint32_t)*rd1 / (uint32_t)*rs;              \
			SET_Z_VAL(*rd1);                               \
		}                                                   \
	}                                                       \
	COUNT_CYCLES(37);                                       \
}
static void divu_a(uint16_t op) { DIVU(A); }
static void divu_b(uint16_t op) { DIVU(B); }

static void eint(uint16_t op)
{
	m_st |= STBIT_IE;
	check_interrupt();
	COUNT_CYCLES(3);
}

#define EXGF(F,R)                                               \
{                                                               \
	uint8_t shift = F ? 6 : 0;                                    \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	uint32_t temp = (m_st >> shift) & 0x3f;                    \
	m_st &= ~(0x3f << shift);                                \
	m_st |= (*rd & 0x3f) << shift;                           \
	*rd = temp;                                                 \
	COUNT_CYCLES(1);                                            \
}
static void exgf0_a(uint16_t op) { EXGF(0,A); }
static void exgf0_b(uint16_t op) { EXGF(0,B); }
static void exgf1_a(uint16_t op) { EXGF(1,A); }
static void exgf1_b(uint16_t op) { EXGF(1,B); }

#define LMO(R)                                                  \
{                                                               \
	uint32_t res = 0;                                             \
	uint32_t rs  = R##REG(SRCREG(op));                                \
		int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_Z();                                                     \
	SET_Z_VAL(rs);                                             \
	if (rs)                                                     \
	{                                                           \
		while (!(rs & 0x80000000))                              \
		{                                                       \
			res++;                                              \
			rs <<= 1;                                           \
		}                                                       \
	}                                                           \
	*rd = res;                                                  \
	COUNT_CYCLES(1);                                            \
}
static void lmo_a(uint16_t op) { LMO(A); }
static void lmo_b(uint16_t op) { LMO(B); }

/* (ReBOX, ADR-057: the same loads straight from a memory page when the block is word aligned and within one page:
   RDMEM_WORD reads such a page with no side effect) */
static inline int mm_page_words(uint32_t a, unsigned n, uint16_t *const *pages, uint16_t **w)
{
	uint16_t *p = pages[a >> 20];
	if ((a & 0x0f) || !p || (a & 0xfffff) + n * 0x20u > 0x100000u)
		return 0;
	*w = p + ((a & 0xfffff) >> 4);
	return 1;
}
#define MMFM(R)                                                 \
{                                                               \
	int32_t i;                                                    \
	uint16_t l = (uint16_t) PARAM_WORD();                        \
	COUNT_CYCLES(3);                                            \
	{                                                           \
		int32_t rd = DSTREG(op);                                      \
		uint16_t *w;                                            \
		if (s_mm_fast && !((l >> rd) & 1) && mm_page_words((uint32_t)R##REG(rd), (unsigned)__builtin_popcount(l), s_rpage, &w)) \
		{                                                       \
			for (i = 15; i >= 0 ; i--)                          \
			{                                                   \
				if (l & 0x8000)                                 \
				{                                               \
					R##REG(i) = (int32_t)(w[0] | ((uint32_t)w[1] << 16)); \
					w += 2;                                     \
					R##REG(rd) += 0x20;                         \
					COUNT_CYCLES(4);                            \
				}                                               \
				l <<= 1;                                        \
			}                                                   \
		}                                                       \
		else                                                    \
		for (i = 15; i >= 0 ; i--)                              \
		{                                                       \
			if (l & 0x8000)                                     \
			{                                                   \
				R##REG(i) = RLONG(R##REG(rd));                 \
				R##REG(rd) += 0x20;                             \
				COUNT_CYCLES(4);                                \
			}                                                   \
			l <<= 1;                                            \
		}                                                       \
	}                                                           \
}
static void mmfm_a(uint16_t op) { MMFM(A); }
static void mmfm_b(uint16_t op) { MMFM(B); }

#define MMTM(R)                                                 \
{                                                               \
	uint32_t i;                                                   \
	uint16_t l = (uint16_t) PARAM_WORD();                        \
	COUNT_CYCLES(2);                                            \
	{                                                           \
		int32_t rd = DSTREG(op);                                      \
		if (m_is_34020)                                      \
		{                                                       \
			CLR_N();                                             \
			SET_N_VAL(R##REG(rd) ^ 0x80000000);                    \
		}                                                       \
		const unsigned n = (unsigned)__builtin_popcount(l);     \
		const uint32_t lo = (uint32_t)R##REG(rd) - n * 0x20u;   \
		uint16_t *w;                                            \
		/* (ReBOX: the same stores straight to a page, outside the write watch: as TMS34010_WRMEM_WORD) */ \
		if (s_mm_fast && mm_page_words(lo, n, s_wpage, &w) &&   \
			!((uint64_t)lo < (uint64_t)s_wlo + s_wlen && (uint64_t)s_wlo < (uint64_t)lo + n * 0x20u)) \
		{                                                       \
			w += 2 * n;                                         \
			for (i = 0; i  < 16; i++)                           \
			{                                                   \
				if (l & 0x8000)                                 \
				{                                               \
					R##REG(rd) -= 0x20;                         \
					w -= 2;                                     \
					w[0] = (uint16_t)R##REG(i);                 \
					w[1] = (uint16_t)((uint32_t)R##REG(i) >> 16); \
					COUNT_CYCLES(4);                            \
				}                                               \
				l <<= 1;                                        \
			}                                                   \
		}                                                       \
		else                                                    \
		for (i = 0; i  < 16; i++)                               \
		{                                                       \
			if (l & 0x8000)                                     \
			{                                                   \
				R##REG(rd) -= 0x20;                             \
				WLONG(R##REG(rd),R##REG(i));                   \
				COUNT_CYCLES(4);                                \
			}                                                   \
			l <<= 1;                                            \
		}                                                       \
	}                                                           \
}
static void mmtm_a(uint16_t op) { MMTM(A); }
static void mmtm_b(uint16_t op) { MMTM(B); }

#define MODS(R)                                                 \
{                                                               \
	int32_t *rs = &R##REG(SRCREG(op));                                \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_NZV();                                                   \
	if (*rs != 0)                                               \
	{                                                           \
		*rd %= *rs;                                             \
		SET_NZ_VAL(*rd);                                       \
	}                                                           \
	else                                                        \
		SET_V_LOG(1);                                          \
	COUNT_CYCLES(40);                                           \
}
static void mods_a(uint16_t op) { MODS(A); }
static void mods_b(uint16_t op) { MODS(B); }

#define MODU(R)                                                 \
{                                                               \
	int32_t *rs = &R##REG(SRCREG(op));                                \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_ZV();                                                        \
	if (*rs != 0)                                               \
	{                                                           \
		*rd = (uint32_t)*rd % (uint32_t)*rs;                        \
		SET_Z_VAL(*rd);                                            \
	}                                                           \
	else                                                        \
		SET_V_LOG(1);                                          \
	COUNT_CYCLES(35);                                           \
}
static void modu_a(uint16_t op) { MODU(A); }
static void modu_b(uint16_t op) { MODU(B); }

#define MPYS(R)                                                 \
{                                                               \
	int32_t *rd1 = &R##REG(DSTREG(op));                               \
	int32_t m1 = R##REG(SRCREG(op));                                  \
	int64_t product;                                              \
																\
	SEXTEND(m1, FW(1));                                         \
	CLR_NZ();                                                        \
	product = mul_32x32(m1, *rd1);                          \
	SET_Z_LOG(product == 0);                                   \
	SET_N_BIT(product >> 32, 31);                              \
																\
	*rd1             = (int32_t)(product >> 32);                \
	R##REG(DSTREG(op)|1) = product & 0xffffffff;                \
																\
	COUNT_CYCLES(20);                                           \
}
static void mpys_a(uint16_t op) { MPYS(A); }
static void mpys_b(uint16_t op) { MPYS(B); }

#define MPYU(R)                                                 \
{                                                               \
	int32_t *rd1 = &R##REG(DSTREG(op));                               \
	uint32_t m1 = R##REG(SRCREG(op));                                 \
	uint64_t product;                                             \
																\
	ZEXTEND(m1, FW(1));                                         \
	CLR_Z();                                                     \
	product = mulu_32x32(m1, *rd1);                     \
	SET_Z_LOG(product == 0);                                   \
																\
	*rd1             = (int32_t)(product >> 32);                \
	R##REG(DSTREG(op)|1) = product & 0xffffffff;                \
																\
	COUNT_CYCLES(21);                                           \
}
static void mpyu_a(uint16_t op) { MPYU(A); }
static void mpyu_b(uint16_t op) { MPYU(B); }

#define NEG(R)                                                  \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t r = 0 - *rd;                                          \
	CLR_NCZV();                                                  \
	SET_NZCV_SUB(0,*rd,r);                                      \
	*rd = r;                                                    \
	COUNT_CYCLES(1);                                            \
}
static void neg_a(uint16_t op) { NEG(A); }
static void neg_b(uint16_t op) { NEG(B); }

#define NEGB(R)                                                 \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t t = *rd + (C_FLAG() ? 1 : 0);                          \
	int32_t r = 0 - t;                                            \
	CLR_NCZV();                                                  \
	SET_NZCV_SUB(0,t,r);                                        \
	*rd = r;                                                    \
	COUNT_CYCLES(1);                                            \
}
static void negb_a(uint16_t op) { NEGB(A); }
static void negb_b(uint16_t op) { NEGB(B); }

static void nop(uint16_t op)
{
	COUNT_CYCLES(1);
}

#define NOT(R)                                                  \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_Z();                                                     \
	*rd = ~(*rd);                                               \
	SET_Z_VAL(*rd);                                                \
	COUNT_CYCLES(1);                                            \
}
static void not_a(uint16_t op) { NOT(A); }
static void not_b(uint16_t op) { NOT(B); }

#define OR(R)                                                   \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_Z();                                                     \
	*rd |= R##REG(SRCREG(op));                                      \
	SET_Z_VAL(*rd);                                                \
	COUNT_CYCLES(1);                                            \
}
static void or_a(uint16_t op) { OR(A); }
static void or_b(uint16_t op) { OR(B); }

#define ORI(R)                                                  \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_Z();                                                     \
	*rd |= PARAM_LONG();                                     \
	SET_Z_VAL(*rd);                                                \
	COUNT_CYCLES(3);                                            \
}
static void ori_a(uint16_t op) { ORI(A); }
static void ori_b(uint16_t op) { ORI(B); }

static void setc(uint16_t op)
{
	SET_C_LOG(1);
	COUNT_CYCLES(1);
}

#define SETF(F)                                                 \
{                                                               \
	uint8_t shift = F ? 6 : 0;                                    \
	m_st &= ~(0x3f << shift);                                \
	m_st |= (op & 0x3f) << shift;                        \
	COUNT_CYCLES(1+F);                                          \
}
static void setf0(uint16_t op) { SETF(0); }
static void setf1(uint16_t op) { SETF(1); }

#define SEXT(F,R)                                               \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_NZ();                                                        \
	SEXTEND(*rd,FW(F));                                         \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(3);                                            \
}
static void sext0_a(uint16_t op) { SEXT(0,A); }
static void sext0_b(uint16_t op) { SEXT(0,B); }
static void sext1_a(uint16_t op) { SEXT(1,A); }
static void sext1_b(uint16_t op) { SEXT(1,B); }

#define RL(R,K)                                                 \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t res = *rd;                                            \
	int32_t k = (K);                                              \
	CLR_CZ();                                                        \
	if (k)                                                      \
	{                                                           \
		res<<=(k-1);                                            \
		SET_C_BIT_HI(res, 31);                                 \
		res<<=1;                                                \
		res |= (((uint32_t)*rd)>>((-k)&0x1f));                    \
		*rd = res;                                              \
	}                                                           \
	SET_Z_VAL(res);                                                \
	COUNT_CYCLES(1);                                            \
}
static void rl_k_a(uint16_t op) { RL(A,PARAM_K(op)); }
static void rl_k_b(uint16_t op) { RL(B,PARAM_K(op)); }
static void rl_r_a(uint16_t op) { RL(A,AREG(SRCREG(op))&0x1f); }
static void rl_r_b(uint16_t op) { RL(B,BREG(SRCREG(op))&0x1f); }

#define SLA(R,K)                                                \
{                                                               \
		int32_t *rd = &R##REG(DSTREG(op));                                \
	uint32_t res = *rd;                                           \
		int32_t k = K;                                                \
	CLR_NCZV();                                                  \
	if (k)                                                      \
	{                                                           \
		uint32_t mask = (0xffffffff<<(31-k))&0x7fffffff;          \
		uint32_t res2 = SIGN(res) ? res^mask : res;               \
		SET_V_LOG((res2 & mask) != 0);                         \
																\
		res<<=(k-1);                                            \
		SET_C_BIT_HI(res, 31);                                 \
		res<<=1;                                                \
		*rd = res;                                              \
	}                                                           \
	SET_NZ_VAL(res);                                           \
	COUNT_CYCLES(3);                                            \
}
static void sla_k_a(uint16_t op) { SLA(A,PARAM_K(op)); }
static void sla_k_b(uint16_t op) { SLA(B,PARAM_K(op)); }
static void sla_r_a(uint16_t op) { SLA(A,AREG(SRCREG(op))&0x1f); }
static void sla_r_b(uint16_t op) { SLA(B,BREG(SRCREG(op))&0x1f); }

#define SLL(R,K)                                                \
{                                                               \
		int32_t *rd = &R##REG(DSTREG(op));                                \
	uint32_t res = *rd;                                           \
		int32_t k = K;                                                \
	CLR_CZ();                                                        \
	if (k)                                                      \
	{                                                           \
		res<<=(k-1);                                            \
		SET_C_BIT_HI(res, 31);                                 \
		res<<=1;                                                \
		*rd = res;                                              \
	}                                                           \
	SET_Z_VAL(res);                                                \
	COUNT_CYCLES(1);                                            \
}
static void sll_k_a(uint16_t op) { SLL(A,PARAM_K(op)); }
static void sll_k_b(uint16_t op) { SLL(B,PARAM_K(op)); }
static void sll_r_a(uint16_t op) { SLL(A,AREG(SRCREG(op))&0x1f); }
static void sll_r_b(uint16_t op) { SLL(B,BREG(SRCREG(op))&0x1f); }

#define SRA(R,K)                                                \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t res = *rd;                                            \
	int32_t k = (-(K)) & 0x1f;                                    \
	CLR_NCZ();                                                   \
	if (k)                                                      \
	{                                                           \
		res>>=(k-1);                                            \
		SET_C_BIT_LO(res, 0);                                  \
		res>>=1;                                                \
		*rd = res;                                              \
	}                                                           \
	SET_NZ_VAL(res);                                           \
	COUNT_CYCLES(1);                                            \
}
static void sra_k_a(uint16_t op) { SRA(A,PARAM_K(op)); }
static void sra_k_b(uint16_t op) { SRA(B,PARAM_K(op)); }
static void sra_r_a(uint16_t op) { SRA(A,AREG(SRCREG(op))); }
static void sra_r_b(uint16_t op) { SRA(B,BREG(SRCREG(op))); }

#define SRL(R,K)                                                \
{                                                               \
		int32_t *rd = &R##REG(DSTREG(op));                                \
	uint32_t res = *rd;                                           \
		int32_t k = (-(K)) & 0x1f;                                    \
	CLR_CZ();                                                        \
	if (k)                                                      \
	{                                                           \
		res>>=(k-1);                                            \
		SET_C_BIT_LO(res, 0);                                  \
		res>>=1;                                                \
		*rd = res;                                              \
	}                                                           \
	SET_Z_VAL(res);                                                \
	COUNT_CYCLES(1);                                            \
}
static void srl_k_a(uint16_t op) { SRL(A,PARAM_K(op)); }
static void srl_k_b(uint16_t op) { SRL(B,PARAM_K(op)); }
static void srl_r_a(uint16_t op) { SRL(A,AREG(SRCREG(op))); }
static void srl_r_b(uint16_t op) { SRL(B,BREG(SRCREG(op))); }

#define SUB(R)                                                  \
{                                                               \
	int32_t *rs = &R##REG(SRCREG(op));                                \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t r = *rd - *rs;                                        \
	CLR_NCZV();                                                  \
	SET_NZCV_SUB(*rd,*rs,r);                                    \
	*rd = r;                                                    \
	COUNT_CYCLES(1);                                            \
}
static void sub_a(uint16_t op) { SUB(A); }
static void sub_b(uint16_t op) { SUB(B); }

#define SUBB(R)                                                 \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t t = R##REG(SRCREG(op));                                   \
	int32_t r = *rd - t - (C_FLAG() ? 1 : 0);                      \
	CLR_NCZV();                                                  \
	SET_NZCV_SUB(*rd,t,r);                                      \
	*rd = r;                                                    \
	COUNT_CYCLES(1);                                            \
}
static void subb_a(uint16_t op) { SUBB(A); }
static void subb_b(uint16_t op) { SUBB(B); }

#define SUBI_W(R)                                               \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t r;                                                    \
	int32_t t = ~PARAM_WORD();                                 \
	CLR_NCZV();                                                  \
	r = *rd - t;                                                \
	SET_NZCV_SUB(*rd,t,r);                                      \
	*rd = r;                                                    \
	COUNT_CYCLES(2);                                            \
}
static void subi_w_a(uint16_t op) { SUBI_W(A); }
static void subi_w_b(uint16_t op) { SUBI_W(B); }

#define SUBI_L(R)                                               \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t t = ~PARAM_LONG();                                 \
	int32_t r = *rd - t;                                          \
	CLR_NCZV();                                                  \
	SET_NZCV_SUB(*rd,t,r);                                      \
	*rd = r;                                                    \
	COUNT_CYCLES(3);                                            \
}
static void subi_l_a(uint16_t op) { SUBI_L(A); }
static void subi_l_b(uint16_t op) { SUBI_L(B); }

#define SUBK(R)                                                 \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t t = fw_inc[PARAM_K(op)];                                  \
	int32_t r = *rd - t;                                          \
	CLR_NCZV();                                                  \
	SET_NZCV_SUB(*rd,t,r);                                      \
	*rd = r;                                                    \
	COUNT_CYCLES(1);                                            \
}
static void subk_a(uint16_t op) { SUBK(A); }
static void subk_b(uint16_t op) { SUBK(B); }

#define XOR(R)                                                  \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_Z();                                                     \
	*rd ^= R##REG(SRCREG(op));                                      \
	SET_Z_VAL(*rd);                                                \
	COUNT_CYCLES(1);                                            \
}
static void xor_a(uint16_t op) { XOR(A); }
static void xor_b(uint16_t op) { XOR(B); }

#define XORI(R)                                                 \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_Z();                                                     \
	*rd ^= PARAM_LONG();                                     \
	SET_Z_VAL(*rd);                                                \
	COUNT_CYCLES(3);                                            \
}
static void xori_a(uint16_t op) { XORI(A); }
static void xori_b(uint16_t op) { XORI(B); }

#define ZEXT(F,R)                                               \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_Z();                                                     \
	ZEXTEND(*rd,FW(F));                                         \
	SET_Z_VAL(*rd);                                                \
	COUNT_CYCLES(1);                                            \
}
static void zext0_a(uint16_t op) { ZEXT(0,A); }
static void zext0_b(uint16_t op) { ZEXT(0,B); }
static void zext1_a(uint16_t op) { ZEXT(1,A); }
static void zext1_b(uint16_t op) { ZEXT(1,B); }



/***************************************************************************
    MOVE INSTRUCTIONS
***************************************************************************/

#define MOVI_W(R)                                               \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_NZV();                                                   \
	*rd=PARAM_WORD();                                        \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(2);                                            \
}
static void movi_w_a(uint16_t op) { MOVI_W(A); }
static void movi_w_b(uint16_t op) { MOVI_W(B); }

#define MOVI_L(R)                                               \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_NZV();                                                   \
	*rd=PARAM_LONG();                                        \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(3);                                            \
}
static void movi_l_a(uint16_t op) { MOVI_L(A); }
static void movi_l_b(uint16_t op) { MOVI_L(B); }

#define MOVK(R)                                                 \
{                                                               \
	int32_t k = PARAM_K(op); if (!k) k = 32;                          \
	R##REG(DSTREG(op)) = k;                                         \
	COUNT_CYCLES(1);                                            \
}
static void movk_a(uint16_t op) { MOVK(A); }
static void movk_b(uint16_t op) { MOVK(B); }

#define MOVB_RN(R)                                              \
{                                                               \
	WBYTE(R##REG(DSTREG(op)),R##REG(SRCREG(op)));                      \
	COUNT_CYCLES(1);                                            \
}
static void movb_rn_a(uint16_t op) { MOVB_RN(A); }
static void movb_rn_b(uint16_t op) { MOVB_RN(B); }

#define MOVB_NR(R)                                              \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_NZV();                                                   \
	*rd = (int8_t)RBYTE(R##REG(SRCREG(op)));                     \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(3);                                            \
}
static void movb_nr_a(uint16_t op) { MOVB_NR(A); }
static void movb_nr_b(uint16_t op) { MOVB_NR(B); }

#define MOVB_NN(R)                                              \
{                                                               \
	WBYTE(R##REG(DSTREG(op)),(uint32_t)(uint8_t)RBYTE(R##REG(SRCREG(op))));\
	COUNT_CYCLES(3);                                            \
}
static void movb_nn_a(uint16_t op) { MOVB_NN(A); }
static void movb_nn_b(uint16_t op) { MOVB_NN(B); }

#define MOVB_R_NO(R)                                            \
{                                                               \
	int32_t o = PARAM_WORD();                                  \
	WBYTE(R##REG(DSTREG(op))+o,R##REG(SRCREG(op)));                        \
	COUNT_CYCLES(3);                                            \
}
static void movb_r_no_a(uint16_t op) { MOVB_R_NO(A); }
static void movb_r_no_b(uint16_t op) { MOVB_R_NO(B); }

#define MOVB_NO_R(R)                                            \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t o = PARAM_WORD();                                  \
	CLR_NZV();                                                   \
	*rd = (int8_t)RBYTE(R##REG(SRCREG(op))+o);                   \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(5);                                            \
}
static void movb_no_r_a(uint16_t op) { MOVB_NO_R(A); }
static void movb_no_r_b(uint16_t op) { MOVB_NO_R(B); }

#define MOVB_NO_NO(R)                                           \
{                                                               \
	int32_t o1 = PARAM_WORD();                                 \
	int32_t o2 = PARAM_WORD();                                 \
	WBYTE(R##REG(DSTREG(op))+o2,(uint32_t)(uint8_t)RBYTE(R##REG(SRCREG(op))+o1)); \
	COUNT_CYCLES(5);                                            \
}
static void movb_no_no_a(uint16_t op) { MOVB_NO_NO(A); }
static void movb_no_no_b(uint16_t op) { MOVB_NO_NO(B); }

#define MOVB_RA(R)                                              \
{                                                               \
	WBYTE(PARAM_LONG(),R##REG(DSTREG(op)));                     \
	COUNT_CYCLES(1);                                            \
}
static void movb_ra_a(uint16_t op) { MOVB_RA(A); }
static void movb_ra_b(uint16_t op) { MOVB_RA(B); }

#define MOVB_AR(R)                                              \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_NZV();                                                   \
	*rd = (int8_t)RBYTE(PARAM_LONG());                    \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(5);                                            \
}
static void movb_ar_a(uint16_t op) { MOVB_AR(A); }
static void movb_ar_b(uint16_t op) { MOVB_AR(B); }

static void movb_aa(uint16_t op)
{
	uint32_t bitaddrs=PARAM_LONG();
	WBYTE(PARAM_LONG(),(uint32_t)(uint8_t)RBYTE(bitaddrs));
	COUNT_CYCLES(6);
}

#define MOVE_RR(RS,RD)                                          \
{                                                               \
	int32_t *rd = &RD##REG(DSTREG(op));                               \
	CLR_NZV();                                                   \
	*rd = RS##REG(SRCREG(op));                                      \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(1);                                            \
}
static void move_rr_a (uint16_t op) { MOVE_RR(A,A); }
static void move_rr_b (uint16_t op) { MOVE_RR(B,B); }
static void move_rr_ax(uint16_t op) { MOVE_RR(A,B); }
static void move_rr_bx(uint16_t op) { MOVE_RR(B,A); }

#define MOVE_RN(F,R)                                            \
{                                                               \
	WFIELD##F(R##REG(DSTREG(op)),R##REG(SRCREG(op)));                   \
	COUNT_CYCLES(1);                                            \
}
static void move0_rn_a (uint16_t op) { MOVE_RN(0,A); }
static void move0_rn_b (uint16_t op) { MOVE_RN(0,B); }
static void move1_rn_a (uint16_t op) { MOVE_RN(1,A); }
static void move1_rn_b (uint16_t op) { MOVE_RN(1,B); }

#define MOVE_R_DN(F,R)                                          \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	*rd-=fw_inc[FW(F)];                                         \
	WFIELD##F(*rd,R##REG(SRCREG(op)));                              \
	COUNT_CYCLES(2);                                            \
}
static void move0_r_dn_a (uint16_t op) { MOVE_R_DN(0,A); }
static void move0_r_dn_b (uint16_t op) { MOVE_R_DN(0,B); }
static void move1_r_dn_a (uint16_t op) { MOVE_R_DN(1,A); }
static void move1_r_dn_b (uint16_t op) { MOVE_R_DN(1,B); }

#define MOVE_R_NI(F,R)                                          \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	WFIELD##F(*rd,R##REG(SRCREG(op)));                              \
	*rd+=fw_inc[FW(F)];                                         \
	COUNT_CYCLES(1);                                            \
}
static void move0_r_ni_a (uint16_t op) { MOVE_R_NI(0,A); }
static void move0_r_ni_b (uint16_t op) { MOVE_R_NI(0,B); }
static void move1_r_ni_a (uint16_t op) { MOVE_R_NI(1,A); }
static void move1_r_ni_b (uint16_t op) { MOVE_R_NI(1,B); }

#define MOVE_NR(F,R)                                            \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_NZV();                                                   \
	*rd = RFIELD##F(R##REG(SRCREG(op)));                            \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(3);                                            \
}
static void move0_nr_a (uint16_t op) { MOVE_NR(0,A); }
static void move0_nr_b (uint16_t op) { MOVE_NR(0,B); }
static void move1_nr_a (uint16_t op) { MOVE_NR(1,A); }
static void move1_nr_b (uint16_t op) { MOVE_NR(1,B); }

#define MOVE_DN_R(F,R)                                          \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t *rs = &R##REG(SRCREG(op));                                \
	CLR_NZV();                                                   \
	*rs-=fw_inc[FW(F)];                                         \
	*rd = RFIELD##F(*rs);                                       \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(4);                                            \
}
static void move0_dn_r_a (uint16_t op) { MOVE_DN_R(0,A); }
static void move0_dn_r_b (uint16_t op) { MOVE_DN_R(0,B); }
static void move1_dn_r_a (uint16_t op) { MOVE_DN_R(1,A); }
static void move1_dn_r_b (uint16_t op) { MOVE_DN_R(1,B); }

#define MOVE_NI_R(F,R)                                          \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t *rs = &R##REG(SRCREG(op));                                \
	int32_t data = RFIELD##F(*rs);                                \
	CLR_NZV();                                                   \
	*rs+=fw_inc[FW(F)];                                         \
	*rd = data;                                                 \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(3);                                            \
}
static void move0_ni_r_a (uint16_t op) { MOVE_NI_R(0,A); }
static void move0_ni_r_b (uint16_t op) { MOVE_NI_R(0,B); }
static void move1_ni_r_a (uint16_t op) { MOVE_NI_R(1,A); }
static void move1_ni_r_b (uint16_t op) { MOVE_NI_R(1,B); }

#define MOVE_NN(F,R)                                            \
{                                                               \
	WFIELD##F(R##REG(DSTREG(op)),RFIELD##F(R##REG(SRCREG(op))));        \
	COUNT_CYCLES(3);                                            \
}
static void move0_nn_a (uint16_t op) { MOVE_NN(0,A); }
static void move0_nn_b (uint16_t op) { MOVE_NN(0,B); }
static void move1_nn_a (uint16_t op) { MOVE_NN(1,A); }
static void move1_nn_b (uint16_t op) { MOVE_NN(1,B); }

#define MOVE_DN_DN(F,R)                                         \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t *rs = &R##REG(SRCREG(op));                                \
	int32_t data;                                                 \
	*rs-=fw_inc[FW(F)];                                         \
	data = RFIELD##F(*rs);                                      \
	*rd-=fw_inc[FW(F)];                                         \
	WFIELD##F(*rd,data);                                        \
	COUNT_CYCLES(4);                                            \
}
static void move0_dn_dn_a (uint16_t op) { MOVE_DN_DN(0,A); }
static void move0_dn_dn_b (uint16_t op) { MOVE_DN_DN(0,B); }
static void move1_dn_dn_a (uint16_t op) { MOVE_DN_DN(1,A); }
static void move1_dn_dn_b (uint16_t op) { MOVE_DN_DN(1,B); }

#define MOVE_NI_NI(F,R)                                         \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t *rs = &R##REG(SRCREG(op));                                \
	int32_t data = RFIELD##F(*rs);                                \
	*rs+=fw_inc[FW(F)];                                         \
	WFIELD##F(*rd,data);                                        \
	*rd+=fw_inc[FW(F)];                                         \
	COUNT_CYCLES(4);                                            \
}
static void move0_ni_ni_a (uint16_t op) { MOVE_NI_NI(0,A); }
static void move0_ni_ni_b (uint16_t op) { MOVE_NI_NI(0,B); }
static void move1_ni_ni_a (uint16_t op) { MOVE_NI_NI(1,A); }
static void move1_ni_ni_b (uint16_t op) { MOVE_NI_NI(1,B); }

#define MOVE_R_NO(F,R)                                          \
{                                                               \
	int32_t o = PARAM_WORD();                                  \
	WFIELD##F(R##REG(DSTREG(op))+o,R##REG(SRCREG(op)));                 \
	COUNT_CYCLES(3);                                            \
}
static void move0_r_no_a (uint16_t op) { MOVE_R_NO(0,A); }
static void move0_r_no_b (uint16_t op) { MOVE_R_NO(0,B); }
static void move1_r_no_a (uint16_t op) { MOVE_R_NO(1,A); }
static void move1_r_no_b (uint16_t op) { MOVE_R_NO(1,B); }

#define MOVE_NO_R(F,R)                                          \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t o = PARAM_WORD();                                  \
	CLR_NZV();                                                   \
	*rd = RFIELD##F(R##REG(SRCREG(op))+o);                          \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(5);                                            \
}
static void move0_no_r_a (uint16_t op) { MOVE_NO_R(0,A); }
static void move0_no_r_b (uint16_t op) { MOVE_NO_R(0,B); }
static void move1_no_r_a (uint16_t op) { MOVE_NO_R(1,A); }
static void move1_no_r_b (uint16_t op) { MOVE_NO_R(1,B); }

#define MOVE_NO_NI(F,R)                                         \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t o = PARAM_WORD();                                  \
	int32_t data = RFIELD##F(R##REG(SRCREG(op))+o);                   \
	WFIELD##F(*rd,data);                                        \
	*rd+=fw_inc[FW(F)];                                         \
	COUNT_CYCLES(5);                                            \
}
static void move0_no_ni_a (uint16_t op) { MOVE_NO_NI(0,A); }
static void move0_no_ni_b (uint16_t op) { MOVE_NO_NI(0,B); }
static void move1_no_ni_a (uint16_t op) { MOVE_NO_NI(1,A); }
static void move1_no_ni_b (uint16_t op) { MOVE_NO_NI(1,B); }

#define MOVE_NO_NO(F,R)                                         \
{                                                               \
	int32_t o1 = PARAM_WORD();                                 \
	int32_t o2 = PARAM_WORD();                                 \
	int32_t data = RFIELD##F(R##REG(SRCREG(op))+o1);                  \
	WFIELD##F(R##REG(DSTREG(op))+o2,data);                          \
	COUNT_CYCLES(5);                                            \
}
static void move0_no_no_a (uint16_t op) { MOVE_NO_NO(0,A); }
static void move0_no_no_b (uint16_t op) { MOVE_NO_NO(0,B); }
static void move1_no_no_a (uint16_t op) { MOVE_NO_NO(1,A); }
static void move1_no_no_b (uint16_t op) { MOVE_NO_NO(1,B); }

#define MOVE_RA(F,R)                                            \
{                                                               \
	WFIELD##F(PARAM_LONG(),R##REG(DSTREG(op)));                  \
	COUNT_CYCLES(3);                                            \
}
static void move0_ra_a (uint16_t op) { MOVE_RA(0,A); }
static void move0_ra_b (uint16_t op) { MOVE_RA(0,B); }
static void move1_ra_a (uint16_t op) { MOVE_RA(1,A); }
static void move1_ra_b (uint16_t op) { MOVE_RA(1,B); }

#define MOVE_AR(F,R)                                            \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	CLR_NZV();                                                   \
	*rd = RFIELD##F(PARAM_LONG());                           \
	SET_NZ_VAL(*rd);                                           \
	COUNT_CYCLES(5);                                            \
}
static void move0_ar_a (uint16_t op) { MOVE_AR(0,A); }
static void move0_ar_b (uint16_t op) { MOVE_AR(0,B); }
static void move1_ar_a (uint16_t op) { MOVE_AR(1,A); }
static void move1_ar_b (uint16_t op) { MOVE_AR(1,B); }

#define MOVE_A_NI(F,R)                                          \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	WFIELD##F(*rd,RFIELD##F(PARAM_LONG()));                  \
	*rd+=fw_inc[FW(F)];                                         \
	COUNT_CYCLES(5);                                            \
}
static void move0_a_ni_a (uint16_t op) { MOVE_A_NI(0,A); }
static void move0_a_ni_b (uint16_t op) { MOVE_A_NI(0,B); }
static void move1_a_ni_a (uint16_t op) { MOVE_A_NI(1,A); }
static void move1_a_ni_b (uint16_t op) { MOVE_A_NI(1,B); }

#define MOVE_AA(F)                                              \
{                                                               \
	uint32_t bitaddrs=PARAM_LONG();                            \
	WFIELD##F(PARAM_LONG(),RFIELD##F(bitaddrs));             \
	COUNT_CYCLES(7);                                            \
}
static void move0_aa (uint16_t op) { MOVE_AA(0); }
static void move1_aa (uint16_t op) { MOVE_AA(1); }



/***************************************************************************
    PROGRAM CONTROL INSTRUCTIONS
***************************************************************************/

#define CALL(R)                                                 \
{                                                               \
	PUSH(m_pc);                                                 \
	m_pc = R##REG(DSTREG(op));                                       \
	CORRECT_ODD_PC("CALL");                                     \
	COUNT_CYCLES(3);                                            \
}
static void call_a (uint16_t op) { CALL(A); }
static void call_b (uint16_t op) { CALL(B); }

static void callr(uint16_t op)
{
	PUSH(m_pc+0x10);
	m_pc += (PARAM_WORD_NO_INC()<<4)+0x10;
	COUNT_CYCLES(3);
}

static void calla(uint16_t op)
{
	PUSH(m_pc+0x20);
	m_pc = PARAM_LONG_NO_INC();
	CORRECT_ODD_PC("CALLA");
	COUNT_CYCLES(4);
}

#define DSJ(R)                                                  \
{                                                               \
	if (--R##REG(DSTREG(op)))                                       \
	{                                                           \
		m_pc += (PARAM_WORD_NO_INC()<<4)+0x10;                    \
		COUNT_CYCLES(3);                                        \
	}                                                           \
	else                                                        \
	{                                                           \
		SKIP_WORD();                                             \
		COUNT_CYCLES(2);                                        \
	}                                                           \
}
static void dsj_a (uint16_t op) { DSJ(A); }
static void dsj_b (uint16_t op) { DSJ(B); }

#define DSJEQ(R)                                                \
{                                                               \
	if (Z_FLAG())                                                    \
	{                                                           \
		if (--R##REG(DSTREG(op)))                                   \
		{                                                       \
			m_pc += (PARAM_WORD_NO_INC()<<4)+0x10;                \
			COUNT_CYCLES(3);                                    \
		}                                                       \
		else                                                    \
		{                                                       \
			SKIP_WORD();                                         \
			COUNT_CYCLES(2);                                    \
		}                                                       \
	}                                                           \
	else                                                        \
	{                                                           \
		SKIP_WORD();                                             \
		COUNT_CYCLES(2);                                        \
	}                                                           \
}
static void dsjeq_a (uint16_t op) { DSJEQ(A); }
static void dsjeq_b (uint16_t op) { DSJEQ(B); }

#define DSJNE(R)                                                \
{                                                               \
	if (!Z_FLAG())                                               \
	{                                                           \
		if (--R##REG(DSTREG(op)))                                   \
		{                                                       \
			m_pc += (PARAM_WORD_NO_INC()<<4)+0x10;                \
			COUNT_CYCLES(3);                                    \
		}                                                       \
		else                                                    \
		{                                                       \
			SKIP_WORD();                                         \
			COUNT_CYCLES(2);                                    \
		}                                                       \
	}                                                           \
	else                                                        \
	{                                                           \
		SKIP_WORD();                                             \
		COUNT_CYCLES(2);                                        \
	}                                                           \
}
static void dsjne_a (uint16_t op) { DSJNE(A); }
static void dsjne_b (uint16_t op) { DSJNE(B); }

#define DSJS(R)                                                 \
{                                                               \
	if (op & 0x0400)                                        \
	{                                                           \
		if (--R##REG(DSTREG(op)))                                   \
		{                                                       \
			m_pc -= ((PARAM_K(op))<<4);                              \
			COUNT_CYCLES(2);                                    \
		}                                                       \
		else                                                    \
			COUNT_CYCLES(3);                                    \
	}                                                           \
	else                                                        \
	{                                                           \
		if (--R##REG(DSTREG(op)))                                   \
		{                                                       \
			m_pc += ((PARAM_K(op))<<4);                              \
			COUNT_CYCLES(2);                                    \
		}                                                       \
		else                                                    \
			COUNT_CYCLES(3);                                    \
	}                                                           \
}
static void dsjs_a (uint16_t op) { DSJS(A); }
static void dsjs_b (uint16_t op) { DSJS(B); }

static void emu(uint16_t op)
{
	/* in RUN state, this instruction is a NOP */
	COUNT_CYCLES(6);
}

#define EXGPC(R)                                                \
{                                                               \
	int32_t *rd = &R##REG(DSTREG(op));                                \
	int32_t temppc = *rd;                                         \
	*rd = m_pc;                                                  \
	m_pc = temppc;                                               \
	CORRECT_ODD_PC("EXGPC");                                    \
	COUNT_CYCLES(2);                                            \
}
static void exgpc_a (uint16_t op) { EXGPC(A); }
static void exgpc_b (uint16_t op) { EXGPC(B); }

#define GETPC(R)                                                \
{                                                               \
	R##REG(DSTREG(op)) = m_pc;                                       \
	COUNT_CYCLES(1);                                            \
}
static void getpc_a (uint16_t op) { GETPC(A); }
static void getpc_b (uint16_t op) { GETPC(B); }

#define GETST(R)                                                \
{                                                               \
	R##REG(DSTREG(op)) = m_st;                               \
	COUNT_CYCLES(1);                                            \
}
static void getst_a (uint16_t op) { GETST(A); }
static void getst_b (uint16_t op) { GETST(B); }

#define j_xx_8(TAKE)                                            \
{                                                               \
	if (DSTREG(op))                                                 \
	{                                                           \
		if (TAKE)                                               \
		{                                                       \
			m_pc += (PARAM_REL8(op) << 4);                           \
			COUNT_CYCLES(2);                                    \
		}                                                       \
		else                                                    \
			COUNT_CYCLES(1);                                    \
	}                                                           \
	else                                                        \
	{                                                           \
		if (TAKE)                                               \
		{                                                       \
			m_pc = PARAM_LONG_NO_INC();                       \
			CORRECT_ODD_PC("J_XX_8");                           \
			COUNT_CYCLES(3);                                    \
		}                                                       \
		else                                                    \
		{                                                       \
			SKIP_LONG();                                         \
			COUNT_CYCLES(4);                                    \
		}                                                       \
	}                                                           \
}

#define j_xx_0(TAKE)                                            \
{                                                               \
	if (DSTREG(op))                                             \
	{                                                           \
		if (TAKE)                                               \
		{                                                       \
			m_pc += (PARAM_REL8(op) << 4);                           \
			COUNT_CYCLES(2);                                    \
		}                                                       \
		else                                                    \
			COUNT_CYCLES(1);                                    \
	}                                                           \
	else                                                        \
	{                                                           \
		if (TAKE)                                               \
		{                                                       \
			m_pc += (PARAM_WORD_NO_INC()<<4)+0x10;                \
			COUNT_CYCLES(3);                                    \
		}                                                       \
		else                                                    \
		{                                                       \
			SKIP_WORD();                                         \
			COUNT_CYCLES(2);                                    \
		}                                                       \
	}                                                           \
}

#define j_xx_x(TAKE)                                            \
{                                                               \
	if (TAKE)                                                   \
	{                                                           \
		m_pc += (PARAM_REL8(op) << 4);                               \
		COUNT_CYCLES(2);                                        \
	}                                                           \
	else                                                        \
		COUNT_CYCLES(1);                                        \
}

static void j_UC_0(uint16_t op)
{
	j_xx_0(1);
}
static void j_UC_8(uint16_t op)
{
	j_xx_8(1);
}
static void j_UC_x(uint16_t op)
{
	j_xx_x(1);
}
static void j_P_0(uint16_t op)
{
	j_xx_0(!N_FLAG() && !Z_FLAG());
}
static void j_P_8(uint16_t op)
{
	j_xx_8(!N_FLAG() && !Z_FLAG());
}
static void j_P_x(uint16_t op)
{
	j_xx_x(!N_FLAG() && !Z_FLAG());
}
static void j_LS_0(uint16_t op)
{
	j_xx_0(C_FLAG() || Z_FLAG());
}
static void j_LS_8(uint16_t op)
{
	j_xx_8(C_FLAG() || Z_FLAG());
}
static void j_LS_x(uint16_t op)
{
	j_xx_x(C_FLAG() || Z_FLAG());
}
static void j_HI_0(uint16_t op)
{
	j_xx_0(!C_FLAG() && !Z_FLAG());
}
static void j_HI_8(uint16_t op)
{
	j_xx_8(!C_FLAG() && !Z_FLAG());
}
static void j_HI_x(uint16_t op)
{
	j_xx_x(!C_FLAG() && !Z_FLAG());
}
static void j_LT_0(uint16_t op)
{
	j_xx_0((N_FLAG() && !V_FLAG()) || (!N_FLAG() && V_FLAG()));
}
static void j_LT_8(uint16_t op)
{
	j_xx_8((N_FLAG() && !V_FLAG()) || (!N_FLAG() && V_FLAG()));
}
static void j_LT_x(uint16_t op)
{
	j_xx_x((N_FLAG() && !V_FLAG()) || (!N_FLAG() && V_FLAG()));
}
static void j_GE_0(uint16_t op)
{
	j_xx_0((N_FLAG() && V_FLAG()) || (!N_FLAG() && !V_FLAG()));
}
static void j_GE_8(uint16_t op)
{
	j_xx_8((N_FLAG() && V_FLAG()) || (!N_FLAG() && !V_FLAG()));
}
static void j_GE_x(uint16_t op)
{
	j_xx_x((N_FLAG() && V_FLAG()) || (!N_FLAG() && !V_FLAG()));
}
static void j_LE_0(uint16_t op)
{
	j_xx_0((N_FLAG() && !V_FLAG()) || (!N_FLAG() && V_FLAG()) || Z_FLAG());
}
static void j_LE_8(uint16_t op)
{
	j_xx_8((N_FLAG() && !V_FLAG()) || (!N_FLAG() && V_FLAG()) || Z_FLAG());
}
static void j_LE_x(uint16_t op)
{
	j_xx_x((N_FLAG() && !V_FLAG()) || (!N_FLAG() && V_FLAG()) || Z_FLAG());
}
static void j_GT_0(uint16_t op)
{
	j_xx_0((N_FLAG() && V_FLAG() && !Z_FLAG()) || (!N_FLAG() && !V_FLAG() && !Z_FLAG()));
}
static void j_GT_8(uint16_t op)
{
	j_xx_8((N_FLAG() && V_FLAG() && !Z_FLAG()) || (!N_FLAG() && !V_FLAG() && !Z_FLAG()));
}
static void j_GT_x(uint16_t op)
{
	j_xx_x((N_FLAG() && V_FLAG() && !Z_FLAG()) || (!N_FLAG() && !V_FLAG() && !Z_FLAG()));
}
static void j_C_0(uint16_t op)
{
	j_xx_0(C_FLAG());
}
static void j_C_8(uint16_t op)
{
	j_xx_8(C_FLAG());
}
static void j_C_x(uint16_t op)
{
	j_xx_x(C_FLAG());
}
static void j_NC_0(uint16_t op)
{
	j_xx_0(!C_FLAG());
}
static void j_NC_8(uint16_t op)
{
	j_xx_8(!C_FLAG());
}
static void j_NC_x(uint16_t op)
{
	j_xx_x(!C_FLAG());
}
static void j_EQ_0(uint16_t op)
{
	j_xx_0(Z_FLAG());
}
static void j_EQ_8(uint16_t op)
{
	j_xx_8(Z_FLAG());
}
static void j_EQ_x(uint16_t op)
{
	j_xx_x(Z_FLAG());
}
static void j_NE_0(uint16_t op)
{
	j_xx_0(!Z_FLAG());
}
static void j_NE_8(uint16_t op)
{
	j_xx_8(!Z_FLAG());
}
static void j_NE_x(uint16_t op)
{
	j_xx_x(!Z_FLAG());
}
static void j_V_0(uint16_t op)
{
	j_xx_0(V_FLAG());
}
static void j_V_8(uint16_t op)
{
	j_xx_8(V_FLAG());
}
static void j_V_x(uint16_t op)
{
	j_xx_x(V_FLAG());
}
static void j_NV_0(uint16_t op)
{
	j_xx_0(!V_FLAG());
}
static void j_NV_8(uint16_t op)
{
	j_xx_8(!V_FLAG());
}
static void j_NV_x(uint16_t op)
{
	j_xx_x(!V_FLAG());
}
static void j_N_0(uint16_t op)
{
	j_xx_0(N_FLAG());
}
static void j_N_8(uint16_t op)
{
	j_xx_8(N_FLAG());
}
static void j_N_x(uint16_t op)
{
	j_xx_x(N_FLAG());
}
static void j_NN_0(uint16_t op)
{
	j_xx_0(!N_FLAG());
}
static void j_NN_8(uint16_t op)
{
	j_xx_8(!N_FLAG());
}
static void j_NN_x(uint16_t op)
{
	j_xx_x(!N_FLAG());
}

#define JUMP(R)                                                 \
{                                                               \
	m_pc = R##REG(DSTREG(op));                                       \
	CORRECT_ODD_PC("JUMP");                                     \
	COUNT_CYCLES(2);                                            \
}
static void jump_a (uint16_t op) { JUMP(A); }
static void jump_b (uint16_t op) { JUMP(B); }

static void popst(uint16_t op)
{
	SET_ST(POP());
	COUNT_CYCLES(8);
}

static void pushst(uint16_t op)
{
	PUSH(m_st);
	COUNT_CYCLES(2);
}

#define PUTST(R)                                                \
{                                                               \
	SET_ST(R##REG(DSTREG(op)));                                \
	COUNT_CYCLES(3);                                            \
}
static void putst_a (uint16_t op) { PUTST(A); }
static void putst_b (uint16_t op) { PUTST(B); }

static void reti(uint16_t op)
{
	int32_t st = POP();
	m_pc = POP();
	CORRECT_ODD_PC("RETI");
	SET_ST(st);
	COUNT_CYCLES(11);
}

static void rets(uint16_t op)
{
	uint32_t offs;
	m_pc = POP();
	CORRECT_ODD_PC("RETS");
	offs = PARAM_N(op);
	if (offs)
	{
		SP()+=(offs<<4);
	}
	COUNT_CYCLES(7);
}

#define REV(R)                                                  \
{                                                               \
	R##REG(DSTREG(op)) = 0x0008;                                    \
	COUNT_CYCLES(1);                                            \
}
static void rev_a (uint16_t op) { REV(A); }
static void rev_b (uint16_t op) { REV(B); }

static void trap(uint16_t op)
{
	uint32_t t = PARAM_N(op);
	if (t)
	{
		PUSH(m_pc);
		PUSH(m_st);
	}
	RESET_ST();
	m_pc = RLONG(0xffffffe0-(t<<5));
	CORRECT_ODD_PC("TRAP");
	COUNT_CYCLES(16);
}



#include "tms34010_gfx.inc"



/***************************************************************************
    Internal interrupt check
****************************************************************************/

/* Generate pending interrupts. */
static void check_interrupt()
{
	int vector = 0;
	int irqline = -1;
	int irq;

	/* if we're not actively executing, skip it */
	if (!m_executing)
		return;

	/* check for NMI first */
	if (IOREG(REG_HSTCTLH) & 0x0100)
	{
		TMS_LOGERR("TMS34010 takes NMI\n");

		/* ack the NMI */
		IOREG(REG_HSTCTLH) &= ~0x0100;

		/* handle NMI mode bit */
		if (!(IOREG(REG_HSTCTLH) & 0x0200))
		{
			PUSH(m_pc);
			PUSH(m_st);
		}

		/* leap to the vector */
		RESET_ST();
		m_pc = RLONG(0xfffffee0);
		COUNT_CYCLES(16);
		return;
	}

	/* early out if everything else is disabled */
	irq = IOREG(REG_INTPEND) & IOREG(REG_INTENB);
	if (!IE_FLAG() || !irq)
		return;

	/* host interrupt */
	if (irq & TMS34010_HI)
	{
		TMS_LOGERR("TMS34010 takes HI\n");
		vector = 0xfffffec0;
	}

	/* display interrupt */
	else if (irq & TMS34010_DI)
	{
		TMS_LOGERR("TMS34010 takes DI\n");
		vector = 0xfffffea0;
	}

	/* window violation interrupt */
	else if (irq & TMS34010_WV)
	{
		TMS_LOGERR("TMS34010 takes WV\n");
		vector = 0xfffffe80;
	}

	/* external 1 interrupt */
	else if (irq & TMS34010_INT1)
	{
		TMS_LOGERR("TMS34010 takes INT1\n");
		vector = 0xffffffc0;
		irqline = 0;
	}

	/* external 2 interrupt */
	else if (irq & TMS34010_INT2)
	{
		TMS_LOGERR("TMS34010 takes INT2\n");
		vector = 0xffffffa0;
		irqline = 1;
	}

	/* if we took something, generate it */
	if (vector)
	{
		/* call the callback for externals */
		if (irqline >= 0)
			standard_irq_callback(irqline, m_pc);

		PUSH(m_pc);
		PUSH(m_st);
		RESET_ST();
		m_pc = RLONG(vector);
		COUNT_CYCLES(16);
	}
}



/***************************************************************************
    Reset the CPU emulation
***************************************************************************/






/***************************************************************************
    Set IRQ line state
***************************************************************************/




/***************************************************************************
    Generate internal interrupt
***************************************************************************/




/***************************************************************************
    Execute
***************************************************************************/






/***************************************************************************
    PIXEL OPS
***************************************************************************/

static const pixel_write_func s_pixel_write_ops[4][6] =
{
	{ write_pixel_1,     write_pixel_2,     write_pixel_4,     write_pixel_8,     write_pixel_16,     write_pixel_32     },
	{ write_pixel_r_1,   write_pixel_r_2,   write_pixel_r_4,   write_pixel_r_8,   write_pixel_r_16,   write_pixel_r_32   },
	{ write_pixel_t_1,   write_pixel_t_2,   write_pixel_t_4,   write_pixel_t_8,   write_pixel_t_16,   write_pixel_t_32   },
	{ write_pixel_r_t_1, write_pixel_r_t_2, write_pixel_r_t_4, write_pixel_r_t_8, write_pixel_r_t_16, write_pixel_r_t_32 }
};

static const pixel_read_func s_pixel_read_ops[6] =
{
	read_pixel_1,        read_pixel_2,      read_pixel_4,      read_pixel_8,      read_pixel_16,      read_pixel_32
};


static void set_pixel_function()
{
	uint32_t i1,i2;

	if (IOREG(REG_DPYCTL) & 0x0800)
	{
		/* Shift Register Transfer */
		m_pixel_write = write_pixel_shiftreg;
		m_pixel_read  = read_pixel_shiftreg;
		return;
	}

	switch (IOREG(REG_PSIZE))
	{
		default:
		case 0x01: i2 = 0; break;
		case 0x02: i2 = 1; break;
		case 0x04: i2 = 2; break;
		case 0x08: i2 = 3; break;
		case 0x10: i2 = 4; break;
		case 0x20: i2 = 5; break;
	}

	if (IOREG(REG_CONTROL) & 0x20)
		i1 = m_raster_op ? 3 : 2;
	else
		i1 = m_raster_op ? 1 : 0;

	m_pixel_write = s_pixel_write_ops[i1][i2];
	m_pixel_read  = s_pixel_read_ops [i2];
}



/***************************************************************************
    RASTER OPS
***************************************************************************/

static const raster_op_func s_raster_ops[32] =
{
	NULL,                        raster_op_1,  raster_op_2,  raster_op_3,
	raster_op_4,  raster_op_5,  raster_op_6,  raster_op_7,
	raster_op_8,  raster_op_9,  raster_op_10, raster_op_11,
	raster_op_12, raster_op_13, raster_op_14, raster_op_15,
	raster_op_16, raster_op_17, raster_op_18, raster_op_19,
	raster_op_20, raster_op_21, NULL,                        NULL,
	NULL,                        NULL,                        NULL,                        NULL,
	NULL,                        NULL,                        NULL,                        NULL,
};


static void set_raster_op()
{
	m_raster_op = s_raster_ops[(IOREG(REG_CONTROL) >> 10) & 0x1f];
}


