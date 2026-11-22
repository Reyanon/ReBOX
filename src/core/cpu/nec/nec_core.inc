/*  The instruction set of nec.c, included once per chip (NEC_CORE_V33, NEC_CORE_V35: one or none). The body of the
    instructions is MAME src/devices/cpu/nec/necinstr.hxx, necmacro.h and v25instr.hxx (BSD-3-Clause, copyright-holders:
    Bryan McPhail, Alex W. Jackson) put in one switch.
    This file is part of ReBOX. GPL-2.0-or-later.
*/

#if NEC_CORE_V33
#define CLKS(v20, v30, v33) CLK(v33)
#define CLKW(v20o, v30o, v33o, v20e, v30e, v33e, addr) CLK(((addr) & 1) ? (v33o) : (v33e))
#define CLKM(v20, v30, v33, v20m, v30m, v33m) CLK(ModRM >= 0xc0 ? (v33) : (v33m))
#define CLKR(v20o, v30o, v33o, v20e, v30e, v33e, vall, addr) CLK(ModRM >= 0xc0 ? (vall) : ((addr) & 1) ? (v33o) : (v33e))
#define NEC_IS_V33 1
#define NEC_DIV_QUIRK 0
#define PCS 0    // (prefetch cycles 1 << PCS)
#define R (S->iram + 0x70)
#else
#define CLKS(v20, v30, v33) CLK(v30)
#define CLKW(v20o, v30o, v33o, v20e, v30e, v33e, addr) CLK(((addr) & 1) ? (v30o) : (v30e))
#define CLKM(v20, v30, v33, v20m, v30m, v33m) CLK(ModRM >= 0xc0 ? (v30) : (v30m))
#define CLKR(v20o, v30o, v33o, v20e, v30e, v33e, vall, addr) CLK(ModRM >= 0xc0 ? (vall) : ((addr) & 1) ? (v30o) : (v30e))
#define NEC_IS_V33 0
#define NEC_DIV_QUIRK 1
#define PCS 1
#if NEC_CORE_V35
#define R (S->rp)
#else
#define R (S->iram + 0x70)
#endif
#endif
#define RBYTES ((uint8_t *)(R))
#define Wreg(x) R[x]
#define Sreg(x) R[x]
#define Breg(x) RBYTES[x]
#define PF (nec_parity[(uint8_t)S->ParityVal])
#define RB (S->rbw >> 4)

#if NEC_CORE_V35
#define CompressFlags() (uint16_t)((int)CF | (S->IBRK << 1) | ((int)PF << 2) | (S->F0 << 3) | ((int)AF << 4) | \
	(S->F1 << 5) | ((int)ZF << 6) | ((int)SF << 7) | (S->TF << 8) | (S->IF << 9) | (S->DF << 10) | ((int)OF << 11) | \
	(RB << 12) | (S->MF << 15))
#define ExpandFlags(f) { \
	S->CarryVal = (f) & 0x0001; S->IBRK = ((f) & 0x0002) == 0x0002; S->ParityVal = !((f) & 0x0004); \
	S->F0 = ((f) & 0x0008) == 0x0008; S->AuxVal = (f) & 0x0010; S->F1 = ((f) & 0x0020) == 0x0020; \
	S->ZeroVal = !((f) & 0x0040); S->SignVal = (f) & 0x0080 ? -1 : 0; S->TF = ((f) & 0x0100) == 0x0100; \
	S->IF = ((f) & 0x0200) == 0x0200; S->DF = ((f) & 0x0400) == 0x0400; S->OverVal = (f) & 0x0800; \
	S->MF = ((f) & 0x8000) == 0x8000; }
#else
#define CompressFlags() (uint16_t)((int)CF | 0x02 | ((int)PF << 2) | ((int)AF << 4) | ((int)ZF << 6) | ((int)SF << 7) | \
	(S->TF << 8) | (S->IF << 9) | (S->DF << 10) | ((int)OF << 11) | 0x7000 | (S->MF << 15))
#define ExpandFlags(f) { \
	S->CarryVal = (f) & 0x0001; S->ParityVal = !((f) & 0x0004); S->AuxVal = (f) & 0x0010; \
	S->ZeroVal = !((f) & 0x0040); S->SignVal = (f) & 0x0080 ? -1 : 0; S->TF = ((f) & 0x0100) == 0x0100; \
	S->IF = ((f) & 0x0200) == 0x0200; S->DF = ((f) & 0x0400) == 0x0400; S->OverVal = (f) & 0x0800; }
#endif

// ---- memory ----
static inline uint8_t PFX(rd8)(nec_t *S, uint32_t a)
{
	a = ADDR(S, a);
	const uint8_t *p = S->rmap[a >> 12];
	return likely(p != NULL) ? p[a & 0xfff] : SLOW_R8(S, a);
}
static inline uint16_t PFX(rd16)(nec_t *S, uint32_t a)
{
	if (unlikely(a & 1))
		return (uint16_t)(PFX(rd8)(S, a) | PFX(rd8)(S, a + 1) << 8);
	a = ADDR(S, a);
	const uint8_t *p = S->rmap[a >> 12];
	return likely(p != NULL) ? *(const u16a *)(p + (a & 0xfff)) : SLOW_R16(S, a);
}
static inline void PFX(wr8)(nec_t *S, uint32_t a, uint32_t v)
{
	a = ADDR(S, a);
	uint8_t *p = S->wmap[a >> 12];
	if (likely(p != NULL))
		p[a & 0xfff] = (uint8_t)v;
	else
		SLOW_W8(S, a, (uint8_t)v);
}
static inline void PFX(wr16)(nec_t *S, uint32_t a, uint32_t v)
{
	if (unlikely(a & 1)) {
		PFX(wr8)(S, a, v);
		PFX(wr8)(S, a + 1, v >> 8);
		return;
	}
	a = ADDR(S, a);
	uint8_t *p = S->wmap[a >> 12];
	if (likely(p != NULL))
		*(u16a *)(p + (a & 0xfff)) = (uint16_t)v;
	else
		SLOW_W16(S, a, (uint16_t)v);
}
#define read_mem_byte(a) PFX(rd8)(S, a)
#define read_mem_word(a) PFX(rd16)(S, a)
#define write_mem_byte(a, d) PFX(wr8)(S, a, d)
#define write_mem_word(a, d) PFX(wr16)(S, a, d)

#if NEC_CORE_V33
static uint32_t v33_in(nec_t *S, uint32_t p, int word)
{
	if (p >= 0xff00) {   // (v33_internal_port_map)
		if (p < 0xff80) {
			const uint16_t v = S->transtable[((p & 0x7f) >> 1) & 63];
			return word ? v : (p & 1) ? (uint32_t)(v >> 8) : (uint32_t)(v & 0xff);
		}
		return (p & ~1u) == 0xff80 && !(p & 1) ? S->xa : 0;
	}
	return word ? S->in16(p) : S->in8(p);
}
static void v33_out(nec_t *S, uint32_t p, uint32_t v, int word)
{
	if (p >= 0xff00) {
		if (p < 0xff80) {
			uint16_t *t = &S->transtable[((p & 0x7f) >> 1) & 63];
			if (word)
				*t = (uint16_t)v;
			else
				*t = (p & 1) ? (uint16_t)((*t & 0xff) | v << 8) : (uint16_t)((*t & 0xff00) | (v & 0xff));
		}
		return;
	}
	if (word)
		S->out16(p, (uint16_t)v);
	else
		S->out8(p, (uint8_t)v);
}
#define IN8(p) ((uint8_t)v33_in(S, (uint16_t)(p), 0))
#define OUT8(p, d) v33_out(S, (uint16_t)(p), (d), 0)
static inline uint16_t PFX(in16)(nec_t *S, uint32_t p)
{
	return (p & 1) ? (uint16_t)(v33_in(S, p, 0) | v33_in(S, (p + 1) & 0xffff, 0) << 8) : (uint16_t)v33_in(S, p, 1);
}
static inline void PFX(out16)(nec_t *S, uint32_t p, uint32_t d)
{
	if (p & 1) {
		v33_out(S, p, d & 0xff, 0);
		v33_out(S, (p + 1) & 0xffff, (d >> 8) & 0xff, 0);
	} else
		v33_out(S, p, d, 1);
}
#else
#define IN8(p) S->in8((uint16_t)(p))
#define OUT8(p, d) S->out8((uint16_t)(p), (uint8_t)(d))
static inline uint16_t PFX(in16)(nec_t *S, uint32_t p)
{
	return (p & 1) ? (uint16_t)(S->in8(p) | S->in8((p + 1) & 0xffff) << 8) : S->in16(p);
}
static inline void PFX(out16)(nec_t *S, uint32_t p, uint32_t d)
{
	if (p & 1) {
		S->out8(p, (uint8_t)d);
		S->out8((p + 1) & 0xffff, (uint8_t)(d >> 8));
	} else
		S->out16(p, (uint16_t)d);
}
#endif
#define IN16(p) PFX(in16)(S, (uint16_t)(p))
#define OUT16(p, d) PFX(out16)(S, (uint16_t)(p), (d))

#define SegBase(Seg) ((uint32_t)Sreg(Seg) << 4)
#define DefaultBase(Seg) ((S->seg_prefix && ((Seg) == DS0 || (Seg) == SS)) ? S->prefix_base : (uint32_t)Sreg(Seg) << 4)
#define GetMemB(Seg, Off) (read_mem_byte(DefaultBase(Seg) + (Off)))
#define GetMemW(Seg, Off) (read_mem_word(DefaultBase(Seg) + (Off)))
#define PutMemB(Seg, Off, x) { write_mem_byte(DefaultBase(Seg) + (Off), (x)); }
#define PutMemW(Seg, Off, x) { write_mem_word(DefaultBase(Seg) + (Off), (x)); }
#define PUSH(val) { Wreg(SP) -= 2; write_mem_word(((uint32_t)Sreg(SS) << 4) + Wreg(SP), (val)); }
#define POP(var) { Wreg(SP) += 2; var = read_mem_word(((uint32_t)Sreg(SS) << 4) + ((Wreg(SP) - 2) & 0xffff)); }
#define POP_IP() { uint16_t tip_; POP(tip_); SETIP(tip_); }
#define POPF_BODY { uint32_t tflags; POP(tflags); ExpandFlags(tflags); CLKS(12,8,5); }

// ---- addresses of the instruction (necea.h): fp past the displacement back ----
static const uint8_t *PFX(get_ea)(nec_t *S, uint32_t m, const uint8_t *fp)
{
	uint32_t eo, base;
	const uint32_t mod = m >> 6;
	switch (m & 7) {
	case 0: eo = (uint32_t)Wreg(BW) + Wreg(IX); base = DefaultBase(DS0); break;
	case 1: eo = (uint32_t)Wreg(BW) + Wreg(IY); base = DefaultBase(DS0); break;
	case 2: eo = (uint32_t)Wreg(BP) + Wreg(IX); base = DefaultBase(SS); break;
	case 3: eo = (uint32_t)Wreg(BP) + Wreg(IY); base = DefaultBase(SS); break;
	case 4: eo = Wreg(IX); base = DefaultBase(DS0); break;
	case 5: eo = Wreg(IY); base = DefaultBase(DS0); break;
	case 6:
		if (mod == 0) {
			eo = (uint32_t)fp[0] | (uint32_t)fp[1] << 8;
			fp += 2;
			base = DefaultBase(DS0);
			S->EO = (uint16_t)eo;
			S->EA = base + S->EO;
			return fp;
		}
		eo = Wreg(BP);
		base = DefaultBase(SS);
		break;
	default: eo = Wreg(BW); base = DefaultBase(DS0); break;
	}
	if (mod == 1) {
		eo += (uint32_t)(int32_t)(int8_t)fp[0];
		fp++;
	} else if (mod == 2) {
		S->E16 = (uint16_t)(fp[0] | fp[1] << 8);
		fp += 2;
		eo += (uint32_t)(int32_t)(int16_t)S->E16;
	}
	S->EO = (uint16_t)eo;
	S->EA = base + S->EO;
	return fp;
}

// ---- interrupts ----
#define SCLK(n) do { S->ic -= (n); S->cur_cycles += (n); } while (0)
#if NEC_CORE_V33
#define SCLKS(v30, v33) SCLK(v33)
#else
#define SCLKS(v30, v33) SCLK(v30)
#endif

static void PFX(interrupt)(nec_t *S, unsigned int_num, int source)
{
	uint32_t dest_seg, dest_off;
	uint16_t f;

	S->rep_params = 0;
	f = CompressFlags();
	PUSH(f);
	SCLKS(8, 3);
	S->TF = S->IF = 0;
#if NEC_CORE_V35
	S->MF = S->mode_state;
	if (source == BRKN)
		S->MF = 1;
	else if (source == BRKS) {
		if (S->dec)
			S->MF = 0;
	}
#else
	S->MF = 1;
#endif
	if (source == INT_IRQ)   // (get vector)
		int_num = S->irq_ack ? (unsigned)S->irq_ack() : 0xff;
	dest_off = read_mem_word(int_num * 4);
	dest_seg = read_mem_word(int_num * 4 + 2);
	PUSH(Sreg(PS));
	PUSH(S->ip);
	S->prev_ip = S->ip = (uint16_t)dest_off;
	Sreg(PS) = (uint16_t)dest_seg;
	S->prefetch_reset = 1;
}

#if NEC_CORE_V35
static void PFX(bankswitch)(nec_t *S, unsigned bank_num)
{
	const uint16_t tmp = CompressFlags();
	S->TF = S->IF = 0;
	S->MF = S->mode_state;
	V35_SETRB(S, bank_num);
	Wreg(PSW_SAVE) = tmp;
	Wreg(PC_SAVE) = S->ip;
	S->prev_ip = S->ip = Wreg(VECTOR_PC);
	S->prefetch_reset = 1;
}

static void PFX(external_int)(nec_t *S)
{
	enum { SOURCES = INTTU0 | INTTU1 | INTTU2 | INTD0 | INTD1 | INTP0 | INTP1 | INTP2 |
	                 INTSER0 | INTSR0 | INTST0 | INTSER1 | INTSR1 | INTST1 | INTTB };
	const uint32_t pending = S->pending_irq & S->unmasked_irq;

	if (pending & NMI_IRQ) {
		S->pending_irq &= ~(uint32_t)NMI_IRQ;
		PFX(interrupt)(S, NEC_NMI_VECTOR, NMI_IRQ);
	} else if (pending & SOURCES) {
		int i = -1;
		uint32_t source = 0;
		uint8_t vector = 0;
		while (++i < 8) {
			if (S->ISPR & (1 << i))
				break;
			if (S->priority_inttu == i) {
				if (pending & INTTU0) { source = INTTU0; vector = NEC_INTTU0_VECTOR; break; }
				if (pending & INTTU1) { source = INTTU1; vector = NEC_INTTU1_VECTOR; break; }
				if (pending & INTTU2) { source = INTTU2; vector = NEC_INTTU2_VECTOR; break; }
			}
			if (S->priority_intd == i) {
				if (pending & INTD0) { source = INTD0; vector = NEC_INTD0_VECTOR; break; }
				if (pending & INTD1) { source = INTD1; vector = NEC_INTD1_VECTOR; break; }
			}
			if (S->priority_intp == i) {
				if (pending & INTP0) { source = INTP0; vector = NEC_INTP0_VECTOR; break; }
				if (pending & INTP1) { source = INTP1; vector = NEC_INTP1_VECTOR; break; }
				if (pending & INTP2) { source = INTP2; vector = NEC_INTP2_VECTOR; break; }
			}
			if (S->priority_ints0 == i) {
				if (pending & INTSER0) { source = INTSER0; vector = NEC_INTSER0_VECTOR; break; }
				if (pending & INTSR0) { source = INTSR0; vector = NEC_INTSR0_VECTOR; break; }
				if (pending & INTST0) { source = INTST0; vector = NEC_INTST0_VECTOR; break; }
			}
			if (S->priority_ints1 == i) {
				if (pending & INTSER1) { source = INTSER1; vector = NEC_INTSER1_VECTOR; break; }
				if (pending & INTSR1) { source = INTSR1; vector = NEC_INTSR1_VECTOR; break; }
				if (pending & INTST1) { source = INTST1; vector = NEC_INTST1_VECTOR; break; }
			}
			if (i == 7 && (pending & INTTB)) { source = INTTB; vector = NEC_INTTB_VECTOR; break; }
		}
		if (source != 0) {
			S->pending_irq &= ~source;
			if (!(S->macro_service & source)) {   // (macro service not done, as MAME)
				S->IRQS = vector;
				S->ISPR |= (uint8_t)(1 << i);
				if (S->bankswitch_irq & source)
					PFX(bankswitch)(S, (unsigned)i);
				else
					PFX(interrupt)(S, vector, (int)source);
			}
		}
	} else if (pending & INT_IRQ) {
		S->irq_state = 0;
		S->pending_irq &= ~(uint32_t)INT_IRQ;
		PFX(interrupt)(S, (unsigned)-1, INT_IRQ);
	}
}
#else
static void PFX(external_int)(nec_t *S)
{
	if (S->pending_irq & NMI_IRQ) {
		S->pending_irq &= ~(uint32_t)NMI_IRQ;
		PFX(interrupt)(S, NEC_NMI_VECTOR, NMI_IRQ);
	} else if (S->pending_irq) {
		// (the line taken down first: the acknowledge may raise it again, for after this instruction)
		S->irq_state = 0;
		S->pending_irq &= ~(uint32_t)INT_IRQ;
		PFX(interrupt)(S, (unsigned)-1, INT_IRQ);
	}
}
#endif

#define CHANGE_PC do { S->prefetch_reset = 1; } while (0)
#define CLK(n) do { const int cyc_ = (n); cc += cyc_; ic -= cyc_; } while (0)
#define IPNOW ((uint16_t)(ip0 + (uint32_t)(fp - fs)))
#define SETIP(x) (ip0 = (uint32_t)(uint16_t)(x) - (uint32_t)(fp - fs))
#define FETCH() (*fp++)
#define FETCHW() (fp += 2, (uint16_t)(fp[-2] | fp[-1] << 8))
#if NEC_CORE_V35
#define FETCHOP() (S->MF ? *fp++ : S->dec ? S->dec[*fp++] : *fp++)
#else
#define FETCHOP() (*fp++)
#endif
#define GET_EA(m) (fp = PFX(get_ea)(S, (m), fp), S->EA)
#define INTERRUPT(num, src) do { const unsigned inum_ = (unsigned)(num); S->ip = IPNOW; S->ic = ic; S->cur_cycles = cc; \
		PFX(interrupt)(S, inum_, (src)); ic = S->ic; cc = S->cur_cycles; SETIP(S->ip); } while (0)
#define NEC_BRK(n) do { const uint32_t bn_ = (n); SETIP(read_mem_word(bn_ * 4)); Sreg(PS) = read_mem_word(bn_ * 4 + 2); CHANGE_PC; } while (0)
#if NEC_CORE_V33
#define BRKXA(x) { NEC_BRK(FETCH()); S->xa = (x); }
#define BRKEM
#else
#define BRKXA(x)
#define BRKEM         // (the V30's 8080 mode not done: no Irem game uses it)
#endif

#define RegWord(m) Wreg(15 - (((m) >> 3) & 7))
#define RegByte(m) Breg(0x1e - 2 * (((m) >> 3) & 3) + (((m) >> 5) & 1))
#define RMW(m) (15 - ((m) & 7))
#define RMB(m) (0x1e - 2 * ((m) & 3) + (((m) >> 2) & 1))
#define GetRMWord(m) ((m) >= 0xc0 ? Wreg(RMW(m)) : ((void)GET_EA(m), read_mem_word(S->EA)))
#define PutbackRMWord(m, val) { if ((m) >= 0xc0) Wreg(RMW(m)) = (val); else write_mem_word(S->EA, (val)); }
#define GetnextRMWord read_mem_word((S->EA & 0xf0000) | ((S->EA + 2) & 0xffff))
#define PutRMWord(m, val) { if ((m) >= 0xc0) Wreg(RMW(m)) = (val); else { (void)GET_EA(m); write_mem_word(S->EA, (val)); } }
#define PutImmRMWord(m) { if ((m) >= 0xc0) Wreg(RMW(m)) = FETCHW(); else { (void)GET_EA(m); const uint16_t imv_ = FETCHW(); write_mem_word(S->EA, imv_); } }
#define GetRMByte(m) ((m) >= 0xc0 ? Breg(RMB(m)) : read_mem_byte(GET_EA(m)))
#define PutRMByte(m, val) { if ((m) >= 0xc0) Breg(RMB(m)) = (val); else write_mem_byte(GET_EA(m), (val)); }
#define PutImmRMByte(m) { if ((m) >= 0xc0) Breg(RMB(m)) = FETCH(); else { (void)GET_EA(m); const uint32_t imv_ = FETCH(); write_mem_byte(S->EA, imv_); } }
#define PutbackRMByte(m, val) { if ((m) >= 0xc0) Breg(RMB(m)) = (val); else write_mem_byte(S->EA, (val)); }
#define DEF_br8 uint32_t ModRM = FETCH(), src, dst; src = RegByte(ModRM); dst = GetRMByte(ModRM)
#define DEF_wr16 uint32_t ModRM = FETCH(), src, dst; src = RegWord(ModRM); dst = GetRMWord(ModRM)
#define DEF_r8b uint32_t ModRM = FETCH(), src, dst; dst = RegByte(ModRM); src = GetRMByte(ModRM)
#define DEF_r16w uint32_t ModRM = FETCH(), src, dst; dst = RegWord(ModRM); src = GetRMWord(ModRM)
#define DEF_ald8 uint32_t src = FETCH(); uint32_t dst = Breg(AL)
#define DEF_axd16 uint32_t src = FETCH(); uint32_t dst = Wreg(AW); src += ((uint32_t)FETCH() << 8)
#define GetModRM uint32_t ModRM = FETCH()

#if NEC_CORE_V35
#define GetRB ModRM = FETCH(); tmp = ModRM >= 0xc0 ? (uint32_t)(Wreg(RMW(ModRM)) & 7) : 0;
#define RETRBI tmp = (Wreg(PSW_SAVE) & 0x7000) >> 12; SETIP(Wreg(PC_SAVE)); { const uint32_t fl_ = Wreg(PSW_SAVE); ExpandFlags(fl_); } V35_SETRB(S, tmp); CHANGE_PC
#define TSKSW Wreg(PSW_SAVE) = CompressFlags(); Wreg(PC_SAVE) = IPNOW; V35_SETRB(S, tmp); SETIP(Wreg(PC_SAVE)); { const uint32_t fl_ = Wreg(PSW_SAVE); ExpandFlags(fl_); } CHANGE_PC
#define MOVSPA tmp = (Wreg(PSW_SAVE) & 0x7000) >> 8; Sreg(SS) = S->iram[tmp + SS]; Wreg(SP) = S->iram[tmp + SP]
#define MOVSPB tmp <<= 4; S->iram[tmp + SS] = Sreg(SS); S->iram[tmp + SP] = Wreg(SP)
#define FINT for (tmp = 1; tmp < 0x100; tmp <<= 1) { if (S->ISPR & tmp) { S->ISPR &= (uint8_t)~tmp; break; } }
#endif

// (generated once from MAME necmacro.h, then edited: see nec.c)
#define SetTF(x)        (S->TF = (x))
#define SetIF(x)        (S->IF = (x))
#define SetDF(x)        (S->DF = (x))
#define SetMD(x)        (S->MF = (x))   /* OB [19.07.99] Mode Flag V30 */

#define SetCFB(x)       (S->CarryVal = (x) & 0x100)
#define SetCFW(x)       (S->CarryVal = (x) & 0x10000)
#define SetAF(x,y,z)    (S->AuxVal = ((x) ^ ((y) ^ (z))) & 0x10)
#define SetSF(x)        (S->SignVal = (x))
#define SetZF(x)        (S->ZeroVal = (x))
#define SetPF(x)        (S->ParityVal = (x))

#define SetSZPF_Byte(x) (S->SignVal=S->ZeroVal=S->ParityVal=(int8_t)(x))
#define SetSZPF_Word(x) (S->SignVal=S->ZeroVal=S->ParityVal=(int16_t)(x))

#define SetOFW_Add(x,y,z)   (S->OverVal = ((x) ^ (y)) & ((x) ^ (z)) & 0x8000)
#define SetOFB_Add(x,y,z)   (S->OverVal = ((x) ^ (y)) & ((x) ^ (z)) & 0x80)
#define SetOFW_Sub(x,y,z)   (S->OverVal = ((z) ^ (y)) & ((z) ^ (x)) & 0x8000)
#define SetOFB_Sub(x,y,z)   (S->OverVal = ((z) ^ (y)) & ((z) ^ (x)) & 0x80)

#define ADDB { uint32_t res=dst+src; SetCFB(res); SetOFB_Add(res,src,dst); SetAF(res,src,dst); SetSZPF_Byte(res); dst=(BYTE)res; }
#define ADDW { uint32_t res=dst+src; SetCFW(res); SetOFW_Add(res,src,dst); SetAF(res,src,dst); SetSZPF_Word(res); dst=(WORD)res; }

#define ADDCB { uint32_t res=dst+src+CF; SetCFB(res); SetOFB_Add(res,src,dst); SetAF(res,src,dst); SetSZPF_Byte(res); dst=(BYTE)res; }
#define ADDCW { uint32_t res=dst+src+CF; SetCFW(res); SetOFW_Add(res,src,dst); SetAF(res,src,dst); SetSZPF_Word(res); dst=(WORD)res; }

#define SUBB { uint32_t res=dst-src; SetCFB(res); SetOFB_Sub(res,src,dst); SetAF(res,src,dst); SetSZPF_Byte(res); dst=(BYTE)res; }
#define SUBW { uint32_t res=dst-src; SetCFW(res); SetOFW_Sub(res,src,dst); SetAF(res,src,dst); SetSZPF_Word(res); dst=(WORD)res; }

#define SUBCB { uint32_t res=dst-src-CF; SetCFB(res); SetOFB_Sub(res,src,dst); SetAF(res,src,dst); SetSZPF_Byte(res); dst=(BYTE)res; }
#define SUBCW { uint32_t res=dst-src-CF; SetCFW(res); SetOFW_Sub(res,src,dst); SetAF(res,src,dst); SetSZPF_Word(res); dst=(WORD)res; }

#define ORB dst|=src; S->CarryVal=S->OverVal=S->AuxVal=0; SetSZPF_Byte(dst)
#define ORW dst|=src; S->CarryVal=S->OverVal=S->AuxVal=0; SetSZPF_Word(dst)

#define ANDB dst&=src; S->CarryVal=S->OverVal=S->AuxVal=0; SetSZPF_Byte(dst)
#define ANDW dst&=src; S->CarryVal=S->OverVal=S->AuxVal=0; SetSZPF_Word(dst)

#define XORB dst^=src; S->CarryVal=S->OverVal=S->AuxVal=0; SetSZPF_Byte(dst)
#define XORW dst^=src; S->CarryVal=S->OverVal=S->AuxVal=0; SetSZPF_Word(dst)

#define IncWordReg(Reg)                     \
	unsigned tmp = (unsigned)Wreg(Reg);     \
	unsigned tmp1 = tmp+1;                  \
	S->OverVal = (tmp == 0x7fff);            \
	SetAF(tmp1,tmp,1);                      \
	SetSZPF_Word(tmp1);                     \
	Wreg(Reg)=tmp1

#define DecWordReg(Reg)                     \
	unsigned tmp = (unsigned)Wreg(Reg);     \
	unsigned tmp1 = tmp-1;                  \
	S->OverVal = (tmp == 0x8000);            \
	SetAF(tmp1,tmp,1);                      \
	SetSZPF_Word(tmp1);                     \
	Wreg(Reg)=tmp1

#define IncByteReg(Reg)                     \
	unsigned tmp = (unsigned)Breg(Reg);     \
	unsigned tmp1 = tmp+1;                  \
	S->OverVal = (tmp == 0x7f);              \
	SetAF(tmp1,tmp,1);                      \
	SetSZPF_Byte(tmp1);                     \
	Breg(Reg)=tmp1

#define DecByteReg(Reg)                     \
	unsigned tmp = (unsigned)Breg(Reg);     \
	unsigned tmp1 = tmp-1;                  \
	S->OverVal = (tmp == 0x80);              \
	SetAF(tmp1,tmp,1);                      \
	SetSZPF_Byte(tmp1);                     \
	Breg(Reg)=tmp1

#define JMP(flag)                           \
	int tmp;                                \
	S->prefetch_reset = 1;                       \
	tmp = (int)((int8_t)FETCH());           \
	if (flag)                               \
	{                                       \
		 \
		SETIP(IPNOW + tmp);          \
		if (unlikely(S->idle_on) && tmp < 0)   \
			nec_idle_see(S, IPNOW);         \
		CLKS(10,10,3);          \
		CHANGE_PC;                          \
		break;                             \
	}

#define ADJ4(param1,param2)                 \
	if (AF || ((Breg(AL) & 0xf) > 9))       \
	{                                       \
		uint16_t tmp;                       \
		tmp = Breg(AL) + param1;            \
		Breg(AL) = tmp;                     \
		S->AuxVal = 1;                       \
		S->CarryVal |= tmp & 0x100;          \
	}                                       \
	if (CF || (Breg(AL)>0x9f))              \
	{                                       \
		Breg(AL) += param2;                 \
		S->CarryVal = 1;                     \
	}                                       \
	SetSZPF_Byte(Breg(AL))

#define ADJB(param1,param2)                 \
	if (AF || ((Breg(AL) & 0xf) > 9))       \
	{                                       \
		Breg(AL) += param1;                 \
		Breg(AH) += param2;                 \
		S->AuxVal = 1;                       \
		S->CarryVal = 1;                     \
	}                                       \
	else                                    \
	{                                       \
		S->AuxVal = 0;                       \
		S->CarryVal = 0;                     \
	}                                       \
	Breg(AL) &= 0x0F

#define BITOP_BYTE                          \
	ModRM = FETCH();                        \
	if (ModRM >= 0xc0) {                    \
		tmp=Breg(RMB(ModRM));       \
	}                                       \
	else {                                  \
		GET_EA(ModRM);          \
		tmp=read_mem_byte(S->EA);            \
	}

#define BITOP_WORD                          \
	ModRM = FETCH();                        \
	if (ModRM >= 0xc0) {                    \
		tmp=Wreg(RMW(ModRM));       \
	}                                       \
	else {                                  \
		GET_EA(ModRM);          \
		tmp=read_mem_word(S->EA);            \
	}

#define BIT_NOT                             \
	if (tmp & (1<<tmp2))                    \
		tmp &= ~(1<<tmp2);                  \
	else                                    \
		tmp |= (1<<tmp2)

#define XchgAWReg(Reg)                      \
	WORD tmp;                               \
	tmp = Wreg(Reg);                        \
	Wreg(Reg) = Wreg(AW);                   \
	Wreg(AW) = tmp

#define ROL_BYTE S->CarryVal = dst & 0x80; dst = (dst << 1)+CF
#define ROL_WORD S->CarryVal = dst & 0x8000; dst = (dst << 1)+CF
#define ROR_BYTE S->CarryVal = dst & 0x1; dst = (dst >> 1)+(CF<<7)
#define ROR_WORD S->CarryVal = dst & 0x1; dst = (dst >> 1)+(CF<<15)
#define ROLC_BYTE dst = (dst << 1) + CF; SetCFB(dst)
#define ROLC_WORD dst = (dst << 1) + CF; SetCFW(dst)
#define RORC_BYTE dst = (CF<<8)+dst; S->CarryVal = dst & 0x01; dst >>= 1
#define RORC_WORD dst = (CF<<16)+dst; S->CarryVal = dst & 0x01; dst >>= 1
#define SHL_BYTE(c) CLK(c); dst <<= c;    SetCFB(dst); SetSZPF_Byte(dst); PutbackRMByte(ModRM,(BYTE)dst)
#define SHL_WORD(c) CLK(c); dst <<= c;    SetCFW(dst); SetSZPF_Word(dst); PutbackRMWord(ModRM,(WORD)dst)
#define SHR_BYTE(c) CLK(c); dst >>= c-1; S->CarryVal = dst & 0x1; dst >>= 1; SetSZPF_Byte(dst); PutbackRMByte(ModRM,(BYTE)dst)
#define SHR_WORD(c) CLK(c); dst >>= c-1; S->CarryVal = dst & 0x1; dst >>= 1; SetSZPF_Word(dst); PutbackRMWord(ModRM,(WORD)dst)
#define SHRA_BYTE(c) CLK(c); dst = ((int8_t)dst) >> (c-1);  S->CarryVal = dst & 0x1; dst = ((int8_t)((BYTE)dst)) >> 1; SetSZPF_Byte(dst); PutbackRMByte(ModRM,(BYTE)dst)
#define SHRA_WORD(c) CLK(c); dst = ((int16_t)dst) >> (c-1); S->CarryVal = dst & 0x1; dst = ((int16_t)((WORD)dst)) >> 1; SetSZPF_Word(dst); PutbackRMWord(ModRM,(WORD)dst)

// After DIV/DIVU overflow, registers are confirmed to be unchanged on V30/V35.
// V20/V25 untested, but probably same as V30.
// From testing by wickerwaka, V33 appears to store an intermediate result, it's not
// entirely clear how that works, so let's just store the truncated result for now.

#define DIVUB {                                          \
	uresult = Wreg(AW);                                  \
	uresult2 = uresult % tmp;                            \
	uresult /= tmp;                                      \
	int overflow = uresult > 0xff;                      \
	if (NEC_DIV_QUIRK)                                 \
		S->CarryVal = S->OverVal = !overflow;              \
	if (overflow)                                        \
		INTERRUPT(NEC_DIVIDE_VECTOR, BRK);           \
	if (!overflow || NEC_IS_V33) {          \
		Breg(AL) = uresult;                              \
		Breg(AH) = uresult2;                             \
	}                                                    \
}

#define DIVB {                                           \
	result = (int16_t)Wreg(AW);                          \
	result2 = result % (int16_t)((int8_t)tmp);           \
	result /= (int16_t)((int8_t)tmp);                    \
	int overflow = result > 0x7f || result < -0x7f;     \
	if (NEC_DIV_QUIRK)                                 \
		S->CarryVal = S->OverVal = !overflow;              \
	if (overflow)                                        \
		INTERRUPT(NEC_DIVIDE_VECTOR, BRK);           \
	if (!overflow || NEC_IS_V33) {          \
		Breg(AL) = result;                               \
		Breg(AH) = result2;                              \
	}                                                    \
}

#define DIVUW {                                          \
	uresult = (((uint32_t)Wreg(DW)) << 16) | Wreg(AW);   \
	uresult2 = uresult % tmp;                            \
	uresult /= tmp;                                      \
	int overflow = uresult > 0xffff;                    \
	if (NEC_DIV_QUIRK)                                 \
		S->CarryVal = S->OverVal = !overflow;              \
	if (overflow)                                        \
		INTERRUPT(NEC_DIVIDE_VECTOR, BRK);           \
	if (!overflow || NEC_IS_V33) {          \
		Wreg(AW) = uresult;                              \
		Wreg(DW) = uresult2;                             \
	}                                                    \
}

#define DIVW {                                           \
	result = ((uint32_t)Wreg(DW) << 16) + Wreg(AW);      \
	result2 = result % (int32_t)((int16_t)tmp);          \
	result /= (int32_t)((int16_t)tmp);                   \
	int overflow = result > 0x7fff || result < -0x7fff; \
	if (NEC_DIV_QUIRK)                                 \
		S->CarryVal = S->OverVal = !overflow;              \
	if (overflow)                                        \
		INTERRUPT(NEC_DIVIDE_VECTOR, BRK);           \
	if (!overflow || NEC_IS_V33) {          \
		Wreg(AW) = result;                               \
		Wreg(DW) = result2;                              \
	}                                                    \
}

#define ADD4S {                                  \
	int i,v1,v2,result;                          \
	int count = (Breg(CL)+1)/2;                  \
	unsigned di = Wreg(IY);                      \
	unsigned si = Wreg(IX);                      \
	    \
	\
	S->ZeroVal = S->CarryVal = 0;                  \
	for (i=0;i<count;i++) {                      \
		CLKS(19,19,18);               \
		tmp = GetMemB(DS0, si);                  \
		tmp2 = GetMemB(DS1, di);                 \
		v1 = (tmp>>4)*10 + (tmp&0xf);            \
		v2 = (tmp2>>4)*10 + (tmp2&0xf);          \
		result = v1+v2+S->CarryVal;               \
		S->CarryVal = result > 99 ? 1 : 0;        \
		result = result % 100;                   \
		v1 = ((result/10)<<4) | (result % 10);   \
		PutMemB(DS1, di,v1);                     \
		if (v1) S->ZeroVal = 1;                   \
		si++;                                    \
		di++;                                    \
	}                                            \
}

#define SUB4S {                                  \
	int count = (Breg(CL)+1)/2;                  \
	int i,v1,v2,result;                          \
	unsigned di = Wreg(IY);                      \
	unsigned si = Wreg(IX);                      \
	    \
	\
	S->ZeroVal = S->CarryVal = 0;                  \
	for (i=0;i<count;i++) {                      \
		CLKS(19,19,18);               \
		tmp = GetMemB(DS1, di);                  \
		tmp2 = GetMemB(DS0, si);                 \
		v1 = (tmp>>4)*10 + (tmp&0xf);            \
		v2 = (tmp2>>4)*10 + (tmp2&0xf);          \
		if (v1 < (v2+S->CarryVal)) {              \
			v1+=100;                             \
			result = v1-(v2+S->CarryVal);         \
			S->CarryVal = 1;                      \
		} else {                                 \
			result = v1-(v2+S->CarryVal);         \
			S->CarryVal = 0;                      \
		}                                        \
		v1 = ((result/10)<<4) | (result % 10);   \
		PutMemB(DS1, di,v1);                     \
		if (v1) S->ZeroVal = 1;                   \
		si++;                                    \
		di++;                                    \
	}                                            \
}

#define CMP4S {                                  \
	int count = (Breg(CL)+1)/2;                  \
	int i,v1,v2,result;                          \
	unsigned di = Wreg(IY);                      \
	unsigned si = Wreg(IX);                      \
	    \
	\
	S->ZeroVal = S->CarryVal = 0;                  \
	for (i=0;i<count;i++) {                      \
		CLKS(19,19,14);               \
		tmp = GetMemB(DS1, di);                  \
		tmp2 = GetMemB(DS0, si);                 \
		v1 = (tmp>>4)*10 + (tmp&0xf);            \
		v2 = (tmp2>>4)*10 + (tmp2&0xf);          \
		if (v1 < (v2+S->CarryVal)) {              \
			v1+=100;                             \
			result = v1-(v2+S->CarryVal);         \
			S->CarryVal = 1;                      \
		} else {                                 \
			result = v1-(v2+S->CarryVal);         \
			S->CarryVal = 0;                      \
		}                                        \
		v1 = ((result/10)<<4) | (result % 10);   \
		if (v1) S->ZeroVal = 1;                   \
		si++;                                    \
		di++;                                    \
	}                                            \
}


#define STR_insb { PutMemB(DS1,Wreg(IY),IN8(Wreg(DW))); Wreg(IY)+= -2 * S->DF + 1; CLK(8); }
#define STR_insw { PutMemW(DS1,Wreg(IY),IN16(Wreg(DW))); Wreg(IY)+= -4 * S->DF + 2; CLKS(18,10,8); }
#define STR_outsb { OUT8(Wreg(DW),GetMemB(DS0,Wreg(IX))); Wreg(IX)+= -2 * S->DF + 1; CLK(8); }
#define STR_outsw { OUT16(Wreg(DW),GetMemW(DS0,Wreg(IX))); Wreg(IX)+= -4 * S->DF + 2; CLKS(18,10,8); }
#define STR_movsb { uint32_t tmp = GetMemB(DS0,Wreg(IX)); PutMemB(DS1,Wreg(IY), tmp); Wreg(IY) += -2 * S->DF + 1; Wreg(IX) += -2 * S->DF + 1; CLKS(8,8,6); }
#define STR_movsw { uint32_t tmp = GetMemW(DS0,Wreg(IX)); PutMemW(DS1,Wreg(IY), tmp); Wreg(IY) += -4 * S->DF + 2; Wreg(IX) += -4 * S->DF + 2; CLKS(16,16,10); }
#define STR_cmpsb { uint32_t src = GetMemB(DS1, Wreg(IY)); uint32_t dst = GetMemB(DS0, Wreg(IX)); SUBB; Wreg(IY) += -2 * S->DF + 1; Wreg(IX) += -2 * S->DF + 1; CLKS(14,14,14); }
#define STR_cmpsw { uint32_t src = GetMemW(DS1, Wreg(IY)); uint32_t dst = GetMemW(DS0, Wreg(IX)); SUBW; Wreg(IY) += -4 * S->DF + 2; Wreg(IX) += -4 * S->DF + 2; CLKS(14,14,14); }
#define STR_stosb { PutMemB(DS1,Wreg(IY),Breg(AL));  Wreg(IY) += -2 * S->DF + 1; CLKS(4,4,3); }
#define STR_stosw { PutMemW(DS1,Wreg(IY),Wreg(AW));  Wreg(IY) += -4 * S->DF + 2; CLKW(8,8,5,8,4,3,Wreg(IY)); }
#define STR_lodsb { Breg(AL) = GetMemB(DS0,Wreg(IX)); Wreg(IX) += -2 * S->DF + 1; CLKS(4,4,3); }
#define STR_lodsw { Wreg(AW) = GetMemW(DS0,Wreg(IX)); Wreg(IX) += -4 * S->DF + 2; CLKW(8,8,5,8,4,3,Wreg(IX)); }
#define STR_scasb { uint32_t src = GetMemB(DS1, Wreg(IY)); uint32_t dst = Breg(AL); SUBB; Wreg(IY) += -2 * S->DF + 1; CLKS(4,4,3); }
#define STR_scasw { uint32_t src = GetMemW(DS1, Wreg(IY)); uint32_t dst = Wreg(AW); SUBW; Wreg(IY) += -4 * S->DF + 2; CLKW(8,8,5,8,4,3,Wreg(IY)); }



static int PFX(run)(nec_t *S, int cycles)
{
	static const void *const ops[256] = {
		&&op_00, &&op_01, &&op_02, &&op_03, &&op_04, &&op_05, &&op_06, &&op_07,
		&&op_08, &&op_09, &&op_0a, &&op_0b, &&op_0c, &&op_0d, &&op_0e, &&op_0f,
		&&op_10, &&op_11, &&op_12, &&op_13, &&op_14, &&op_15, &&op_16, &&op_17,
		&&op_18, &&op_19, &&op_1a, &&op_1b, &&op_1c, &&op_1d, &&op_1e, &&op_1f,
		&&op_20, &&op_21, &&op_22, &&op_23, &&op_24, &&op_25, &&op_26, &&op_27,
		&&op_28, &&op_29, &&op_2a, &&op_2b, &&op_2c, &&op_2d, &&op_2e, &&op_2f,
		&&op_30, &&op_31, &&op_32, &&op_33, &&op_34, &&op_35, &&op_36, &&op_37,
		&&op_38, &&op_39, &&op_3a, &&op_3b, &&op_3c, &&op_3d, &&op_3e, &&op_3f,
		&&op_40, &&op_41, &&op_42, &&op_43, &&op_44, &&op_45, &&op_46, &&op_47,
		&&op_48, &&op_49, &&op_4a, &&op_4b, &&op_4c, &&op_4d, &&op_4e, &&op_4f,
		&&op_50, &&op_51, &&op_52, &&op_53, &&op_54, &&op_55, &&op_56, &&op_57,
		&&op_58, &&op_59, &&op_5a, &&op_5b, &&op_5c, &&op_5d, &&op_5e, &&op_5f,
		&&op_60, &&op_61, &&op_62, &&op_63, &&op_64, &&op_65, &&op_66, &&op_67,
		&&op_68, &&op_69, &&op_6a, &&op_6b, &&op_6c, &&op_6d, &&op_6e, &&op_6f,
		&&op_70, &&op_71, &&op_72, &&op_73, &&op_74, &&op_75, &&op_76, &&op_77,
		&&op_78, &&op_79, &&op_7a, &&op_7b, &&op_7c, &&op_7d, &&op_7e, &&op_7f,
		&&op_80, &&op_81, &&op_82, &&op_83, &&op_84, &&op_85, &&op_86, &&op_87,
		&&op_88, &&op_89, &&op_8a, &&op_8b, &&op_8c, &&op_8d, &&op_8e, &&op_8f,
		&&op_90, &&op_91, &&op_92, &&op_93, &&op_94, &&op_95, &&op_96, &&op_97,
		&&op_98, &&op_99, &&op_9a, &&op_9b, &&op_9c, &&op_9d, &&op_9e, &&op_9f,
		&&op_a0, &&op_a1, &&op_a2, &&op_a3, &&op_a4, &&op_a5, &&op_a6, &&op_a7,
		&&op_a8, &&op_a9, &&op_aa, &&op_ab, &&op_ac, &&op_ad, &&op_ae, &&op_af,
		&&op_b0, &&op_b1, &&op_b2, &&op_b3, &&op_b4, &&op_b5, &&op_b6, &&op_b7,
		&&op_b8, &&op_b9, &&op_ba, &&op_bb, &&op_bc, &&op_bd, &&op_be, &&op_bf,
		&&op_c0, &&op_c1, &&op_c2, &&op_c3, &&op_c4, &&op_c5, &&op_c6, &&op_c7,
		&&op_c8, &&op_c9, &&op_ca, &&op_cb, &&op_cc, &&op_cd, &&op_ce, &&op_cf,
		&&op_d0, &&op_d1, &&op_d2, &&op_d3, &&op_d4, &&op_d5, &&op_d6, &&op_d7,
		&&op_d8, &&op_d9, &&op_da, &&op_db, &&op_dc, &&op_dd, &&op_de, &&op_df,
		&&op_e0, &&op_e1, &&op_e2, &&op_e3, &&op_e4, &&op_e5, &&op_e6, &&op_e7,
		&&op_e8, &&op_e9, &&op_ea, &&op_eb, &&op_ec, &&op_ed, &&op_ee, &&op_ef,
		&&op_f0, &&op_f1, &&op_f2, &&op_f3, &&op_f4, &&op_f5, &&op_f6, &&op_f7,
		&&op_f8, &&op_f9, &&op_fa, &&op_fb, &&op_fc, &&op_fd, &&op_fe, &&op_ff,
	};
	int ic = cycles, cc = 0;
	int pf = S->prefetch_count;
	const uint8_t *fp = S->fbuf, *fs = S->fbuf;
	uint32_t ip0 = S->ip;
	uint32_t op;
	int rep_kind = 0;

#if NEC_CORE_V35
	V35_WAKE(S);
#endif
	if (S->halted)
		return cycles;

	while (ic > 0) {
		ip0 = S->ip;
		S->prev_ip = (uint16_t)ip0;
		S->ic = ic;
		cc = 0;
		if (unlikely((PENDING(S) | S->no_interrupt | S->rep_params) != 0)) {   // (the rare cases in one test)
			if (PENDING(S) != 0 && S->no_interrupt == 0) {
				if ((S->pending_irq & NMI_IRQ) || S->IF) {
					S->cur_cycles = 0;
					PFX(external_int)(S);
					ic = S->ic;
				}
			}
			if (S->no_interrupt)
				S->no_interrupt--;
			ip0 = S->ip;
			if (S->rep_params) {
				const uint32_t rp_ = S->rep_params;
				fs = fp = S->fbuf;
				ip0 = S->rep_ip;
				S->seg_prefix = (uint8_t)((rp_ >> 2) & 1);
				op = (rp_ >> 8) & 0xff;
				rep_kind = (int)(rp_ & 3);
				S->rep_params = 0;
				goto do_rep;
			}
		}
		{
			const uint32_t pc = (((uint32_t)Sreg(PS) << 4) + ip0) & 0xfffff;
			const uint8_t *p = S->omap[pc >> 12];
			if (likely(p != NULL && (pc & 0xfff) <= 0xfe0 && ip0 <= 0xffe0 && !XA_ON(S)))
				fp = p + (pc & 0xfff);
			else {   // (a page's end, the IP's wrap, no direct page: the bytes one by one)
				for (int i = 0; i < 32; i++)
					S->fbuf[i] = PFX(rd_op)(S, ((uint32_t)Sreg(PS) << 4) + ((ip0 + (uint32_t)i) & 0xffff));
				fp = S->fbuf;
			}
			fs = fp;
		}
		op = FETCHOP();
dispatch:
		goto *ops[op];   // (the cases through a table of their labels: no range test)
		switch (op) {
		case 0x00: op_00: { DEF_br8;   ADDB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7); } break;
		case 0x01: op_01: { DEF_wr16;  ADDW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,S->EA); } break;
		case 0x02: op_02: { DEF_r8b;   ADDB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6); } break;
		case 0x03: op_03: { DEF_r16w;  ADDW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,S->EA); } break;
		case 0x04: op_04: { DEF_ald8;  ADDB;   Breg(AL)=dst;           CLKS(4,4,2); } break;
		case 0x05: op_05: { DEF_axd16; ADDW;   Wreg(AW)=dst;           CLKS(4,4,2); } break;
		case 0x06: op_06: { PUSH(Sreg(DS1));   CLKS(12,8,3); } break;
		case 0x07: op_07: { POP(Sreg(DS1));    CLKS(12,8,5); } break;
		case 0x08: op_08: { DEF_br8;   ORB;    PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7); } break;
		case 0x09: op_09: { DEF_wr16;  ORW;    PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,S->EA); } break;
		case 0x0a: op_0a: { DEF_r8b;   ORB;    RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6); } break;
		case 0x0b: op_0b: { DEF_r16w;  ORW;    RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,S->EA); } break;
		case 0x0c: op_0c: { DEF_ald8;  ORB;    Breg(AL)=dst;           CLKS(4,4,2); } break;
		case 0x0d: op_0d: { DEF_axd16; ORW;    Wreg(AW)=dst;           CLKS(4,4,2); } break;
		case 0x0e: op_0e: { PUSH(Sreg(PS));    CLKS(12,8,3); } break;
		case 0x0f: op_0f: { uint32_t ModRM, tmp, tmp2;
	switch (FETCH()) {
		case 0x10 : BITOP_BYTE; CLKS(3,3,4); tmp2 = Breg(CL) & 0x7; S->ZeroVal = (tmp & (1<<tmp2)) ? 1 : 0; S->CarryVal=S->OverVal=0; break; /* Test */
		case 0x11 : BITOP_WORD; CLKS(3,3,4); tmp2 = Breg(CL) & 0xf; S->ZeroVal = (tmp & (1<<tmp2)) ? 1 : 0; S->CarryVal=S->OverVal=0; break; /* Test */
		case 0x12 : BITOP_BYTE; CLKS(5,5,4); tmp2 = Breg(CL) & 0x7; tmp &= ~(1<<tmp2);  PutbackRMByte(ModRM,tmp);   break; /* Clr */
		case 0x13 : BITOP_WORD; CLKS(5,5,4); tmp2 = Breg(CL) & 0xf; tmp &= ~(1<<tmp2);  PutbackRMWord(ModRM,tmp);   break; /* Clr */
		case 0x14 : BITOP_BYTE; CLKS(4,4,4); tmp2 = Breg(CL) & 0x7; tmp |= (1<<tmp2);   PutbackRMByte(ModRM,tmp);   break; /* Set */
		case 0x15 : BITOP_WORD; CLKS(4,4,4); tmp2 = Breg(CL) & 0xf; tmp |= (1<<tmp2);   PutbackRMWord(ModRM,tmp);   break; /* Set */
		case 0x16 : BITOP_BYTE; CLKS(4,4,4); tmp2 = Breg(CL) & 0x7; BIT_NOT;            PutbackRMByte(ModRM,tmp);   break; /* Not */
		case 0x17 : BITOP_WORD; CLKS(4,4,4); tmp2 = Breg(CL) & 0xf; BIT_NOT;            PutbackRMWord(ModRM,tmp);   break; /* Not */

		case 0x18 : BITOP_BYTE; CLKS(4,4,4); tmp2 = (FETCH()) & 0x7;    S->ZeroVal = (tmp & (1<<tmp2)) ? 1 : 0; S->CarryVal=S->OverVal=0; break; /* Test */
		case 0x19 : BITOP_WORD; CLKS(4,4,4); tmp2 = (FETCH()) & 0xf;    S->ZeroVal = (tmp & (1<<tmp2)) ? 1 : 0; S->CarryVal=S->OverVal=0; break; /* Test */
		case 0x1a : BITOP_BYTE; CLKS(6,6,4); tmp2 = (FETCH()) & 0x7;    tmp &= ~(1<<tmp2);      PutbackRMByte(ModRM,tmp);   break; /* Clr */
		case 0x1b : BITOP_WORD; CLKS(6,6,4); tmp2 = (FETCH()) & 0xf;    tmp &= ~(1<<tmp2);      PutbackRMWord(ModRM,tmp);   break; /* Clr */
		case 0x1c : BITOP_BYTE; CLKS(5,5,4); tmp2 = (FETCH()) & 0x7;    tmp |= (1<<tmp2);       PutbackRMByte(ModRM,tmp);   break; /* Set */
		case 0x1d : BITOP_WORD; CLKS(5,5,4); tmp2 = (FETCH()) & 0xf;    tmp |= (1<<tmp2);       PutbackRMWord(ModRM,tmp);   break; /* Set */
		case 0x1e : BITOP_BYTE; CLKS(5,5,4); tmp2 = (FETCH()) & 0x7;    BIT_NOT;                PutbackRMByte(ModRM,tmp);   break; /* Not */
		case 0x1f : BITOP_WORD; CLKS(5,5,4); tmp2 = (FETCH()) & 0xf;    BIT_NOT;                PutbackRMWord(ModRM,tmp);   break; /* Not */

		case 0x20 : ADD4S; CLKS(7,7,2); break;
		case 0x22 : SUB4S; CLKS(7,7,2); break;
		case 0x26 : CMP4S; CLKS(7,7,2); break;
		case 0x28 : ModRM = FETCH(); tmp = GetRMByte(ModRM); tmp <<= 4; tmp |= Breg(AL) & 0xf; Breg(AL) = (Breg(AL) & 0xf0) | ((tmp>>8)&0xf); tmp &= 0xff; PutbackRMByte(ModRM,tmp); CLKM(13,13,9,28,28,15); break;
		case 0x2a : ModRM = FETCH(); tmp = GetRMByte(ModRM); tmp2 = (Breg(AL) & 0xf)<<4; Breg(AL) = (Breg(AL) & 0xf0) | (tmp&0xf); tmp = tmp2 | (tmp>>4);   PutbackRMByte(ModRM,tmp); CLKM(17,17,13,32,32,19); break;
		#if NEC_CORE_V35
		case 0x2d : GetRB;   // BRKCS: nec_bankswitch inside an instruction (the saved PC the next one's, the jump kept)
			{ const uint16_t fl_ = CompressFlags(); S->TF = S->IF = 0; S->MF = S->mode_state; V35_SETRB(S, tmp);
			  Wreg(PSW_SAVE) = fl_; Wreg(PC_SAVE) = IPNOW; SETIP(Wreg(VECTOR_PC)); CHANGE_PC; }
			CLK(15); break;
		case 0x31 : ModRM = FETCH(); ModRM = 0; break;
		case 0x33 : ModRM = FETCH(); ModRM = 0; break;
		case 0x25 : MOVSPA; CLK(16); break;
		case 0x91 : RETRBI; CLK(12); break;
		case 0x92 : FINT; CLK(2); S->no_interrupt = 1; break;
		case 0x94 : GetRB; TSKSW; CLK(20); break;
		case 0x95 : GetRB; MOVSPB; CLK(11); break;
		case 0x9e : ic = 0; break;
#else
case 0x31 : // INS reg1,reg2
			ModRM = FETCH();
			tmp = GetRMByte(0xc0 | (ModRM & 7)) & 0x0f;
			tmp2 = (GetRMByte(0xc0 | ((ModRM >> 3) & 7)) & 0x0f) + 1;
			PutMemW(DS1, Wreg(IY), (GetMemW(DS1, Wreg(IY)) & ~(((1 << tmp2) - 1) << tmp)) | ((Wreg(AW) & ((1 << tmp2) - 1)) << tmp));
			if (tmp + tmp2 > 15) {
				Wreg(IY) += 2;
				PutMemW(DS1, Wreg(IY), (GetMemW(DS1, Wreg(IY)) & ~((1 << (tmp2 - (16 - tmp))) - 1)) | ((Wreg(AW) >> (16 - tmp)) & ((1 << (tmp2 - (16 - tmp))) - 1)));
			}
			PutRMByte(ModRM, (tmp + tmp2) & 0x0f);
			// When and how many extra cycles are taken is not documented:
			// V20: 35-113 cycles
			// V30: 35-133 cycles, odd addresses
			// V30: 31-117 cycles, even addresses
			// V33: 39-77 cycles, odd addresses
			// V33: 37-69 cycles, even addresses
			if (Wreg(IY) & 1) {
				CLKS(113, 113, 77);
			} else {
				CLKS(113, 117, 69);
			}
			break;
		case 0x33 : // EXT reg1,reg2
			ModRM = FETCH();
			tmp = GetRMByte(0xc0 | (ModRM & 7)) & 0x0f;
			tmp2 = (GetRMByte(0xc0 | ((ModRM >> 3) & 7)) & 0x0f) + 1;
			Wreg(AW) = GetMemW(DS0, Wreg(IX)) >> tmp;
			if (tmp + tmp2 > 15) {
				Wreg(IX) += 2;
				Wreg(AW) |= GetMemW(DS0, Wreg(IX)) << (16 - tmp);
			}
			Wreg(AW) &= ((1 << tmp2) - 1);
			PutRMByte(ModRM, (tmp + tmp2) & 0x0f);
			// When and how many extra cycles are taken is not documented:
			// V20: 34-59 cycles
			// V30: 34-59 cycles, odd addresses
			// V30: 26-55 cycles, even addresses
			// V33: 33-63 cycles, odd addresses
			// V33: 29-61 cycles, even addresses
			if (Wreg(IX) & 1) {
				CLKS(59, 59, 63);
			} else {
				CLKS(59, 55, 62);
			}
			break;
		case 0x39 : // INS reg,imm4
			ModRM = FETCH();
			tmp = GetRMByte(ModRM) & 0x0f;
			tmp2 = (FETCH() & 0x0f) + 1;
			PutMemW(DS1, Wreg(IY), (GetMemW(DS1, Wreg(IY)) & ~(((1 << tmp2) - 1) << tmp)) | ((Wreg(AW) & ((1 << tmp2) - 1)) << tmp));
			if (tmp + tmp2 > 15) {
				Wreg(IY) += 2;
				PutMemW(DS1, Wreg(IY), (GetMemW(DS1, Wreg(IY)) & ~((1 << (tmp2 - (16 - tmp))) - 1)) | ((Wreg(AW) >> (16 - tmp)) & ((1 << (tmp2 - (16 - tmp))) - 1)));
			}
			PutRMByte(ModRM, (tmp + tmp2) & 0x0f);
			// When and how many extra cycles are taken is not documented:
			// V20: 75-103 cycles
			// V30: 75-103 cycles, odd addresses
			// V30: 67-87 cycles, even addresses
			// V33: 39-77 cycles, odd addresses
			// V33: 37-69 cycles, even addresses
			if (Wreg(IY) & 1) {
				CLKS(103, 103, 77);
			} else {
				CLKS(103, 87, 69);
			}
			break;
		case 0x3b : // EXT reg,imm4
			ModRM = FETCH();
			tmp = GetRMByte(ModRM) & 0x0f;
			tmp2 = (FETCH() & 0x0f) + 1;
			Wreg(AW) = GetMemW(DS0, Wreg(IX)) >> tmp;
			if (tmp + tmp2 > 15) {
				Wreg(IX) += 2;
				Wreg(AW) |= GetMemW(DS0, Wreg(IX)) << (16 - tmp);
			}
			Wreg(AW) &= ((1 << tmp2) - 1);
			PutRMByte(ModRM, (tmp + tmp2) & 0x0f);
			// When and how many extra cycles are taken is not documented:
			// V20: 25-52 cycles
			// V30: 25-52 cycles, odd addresses
			// V30: 21-44 cycles, even addresses
			// V33: 33-63 cycles, odd addresses
			// V33: 29-61 cycles, even addresses
			if (Wreg(IX) & 1) {
				CLKS(52, 52, 63);
			} else {
				CLKS(52, 44, 62);
			}
			break;
		case 0xe0 : BRKXA(1); CLK(12); break;
		case 0xf0 : BRKXA(0); CLK(12); break;
		case 0xff : BRKEM; CLK(50); break;
		#endif
		default:    ; break;
	}
 } break;
		case 0x10: op_10: { DEF_br8;   ADDCB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7); } break;
		case 0x11: op_11: { DEF_wr16;  ADDCW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,S->EA); } break;
		case 0x12: op_12: { DEF_r8b;   ADDCB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6); } break;
		case 0x13: op_13: { DEF_r16w;  ADDCW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,S->EA); } break;
		case 0x14: op_14: { DEF_ald8;  ADDCB;   Breg(AL)=dst;           CLKS(4,4,2); } break;
		case 0x15: op_15: { DEF_axd16; ADDCW;   Wreg(AW)=dst;           CLKS(4,4,2); } break;
		case 0x16: op_16: { PUSH(Sreg(SS));        CLKS(12,8,3); } break;
		case 0x17: op_17: { POP(Sreg(SS));     CLKS(12,8,5);   S->no_interrupt=1; } break;
		case 0x18: op_18: { DEF_br8;   SUBCB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7); } break;
		case 0x19: op_19: { DEF_wr16;  SUBCW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,S->EA); } break;
		case 0x1a: op_1a: { DEF_r8b;   SUBCB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6); } break;
		case 0x1b: op_1b: { DEF_r16w;  SUBCW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,S->EA); } break;
		case 0x1c: op_1c: { DEF_ald8;  SUBCB;   Breg(AL)=dst;           CLKS(4,4,2); } break;
		case 0x1d: op_1d: { DEF_axd16; SUBCW;   Wreg(AW)=dst;           CLKS(4,4,2); } break;
		case 0x1e: op_1e: { PUSH(Sreg(DS0));       CLKS(12,8,3); } break;
		case 0x1f: op_1f: { POP(Sreg(DS0));        CLKS(12,8,5); } break;
		case 0x20: op_20: { DEF_br8;   ANDB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7); } break;
		case 0x21: op_21: { DEF_wr16;  ANDW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,S->EA); } break;
		case 0x22: op_22: { DEF_r8b;   ANDB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6); } break;
		case 0x23: op_23: { DEF_r16w;  ANDW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,S->EA); } break;
		case 0x24: op_24: { DEF_ald8;  ANDB;   Breg(AL)=dst;           CLKS(4,4,2); } break;
		case 0x25: op_25: { DEF_axd16; ANDW;   Wreg(AW)=dst;           CLKS(4,4,2); } break;
		case 0x26: op_26: S->seg_prefix = 1; S->prefix_base = (uint32_t)Sreg(DS1) << 4; CLK(2); op = FETCHOP(); goto dispatch;
		case 0x27: op_27: { ADJ4(6,0x60);                                  CLKS(3,3,2); } break;
		case 0x28: op_28: { DEF_br8;   SUBB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7); } break;
		case 0x29: op_29: { DEF_wr16;  SUBW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,S->EA); } break;
		case 0x2a: op_2a: { DEF_r8b;   SUBB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6); } break;
		case 0x2b: op_2b: { DEF_r16w;  SUBW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,S->EA); } break;
		case 0x2c: op_2c: { DEF_ald8;  SUBB;   Breg(AL)=dst;           CLKS(4,4,2); } break;
		case 0x2d: op_2d: { DEF_axd16; SUBW;   Wreg(AW)=dst;           CLKS(4,4,2); } break;
		case 0x2e: op_2e: S->seg_prefix = 1; S->prefix_base = (uint32_t)Sreg(PS) << 4; CLK(2); op = FETCHOP(); goto dispatch;
		case 0x2f: op_2f: { ADJ4(-6,-0x60);                                CLKS(3,3,2); } break;
		case 0x30: op_30: { DEF_br8;   XORB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7); } break;
		case 0x31: op_31: { DEF_wr16;  XORW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,S->EA); } break;
		case 0x32: op_32: { DEF_r8b;   XORB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6); } break;
		case 0x33: op_33: { DEF_r16w;  XORW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,S->EA); } break;
		case 0x34: op_34: { DEF_ald8;  XORB;   Breg(AL)=dst;           CLKS(4,4,2); } break;
		case 0x35: op_35: { DEF_axd16; XORW;   Wreg(AW)=dst;           CLKS(4,4,2); } break;
		case 0x36: op_36: S->seg_prefix = 1; S->prefix_base = (uint32_t)Sreg(SS) << 4; CLK(2); op = FETCHOP(); goto dispatch;
		case 0x37: op_37: { ADJB(6, (Breg(AL) > 0xf9) ? 2 : 1);        CLKS(7,7,4); } break;
		case 0x38: op_38: { DEF_br8;   SUBB;                   CLKM(2,2,2,11,11,6); } break;
		case 0x39: op_39: { DEF_wr16;  SUBW;                   CLKR(15,15,8,15,11,6,2,S->EA); } break;
		case 0x3a: op_3a: { DEF_r8b;   SUBB;                   CLKM(2,2,2,11,11,6); } break;
		case 0x3b: op_3b: { DEF_r16w;  SUBW;                   CLKR(15,15,8,15,11,6,2,S->EA); } break;
		case 0x3c: op_3c: { DEF_ald8;  SUBB;                   CLKS(4,4,2); } break;
		case 0x3d: op_3d: { DEF_axd16; SUBW;                   CLKS(4,4,2); } break;
		case 0x3e: op_3e: S->seg_prefix = 1; S->prefix_base = (uint32_t)Sreg(DS0) << 4; CLK(2); op = FETCHOP(); goto dispatch;
		case 0x3f: op_3f: { ADJB(-6, (Breg(AL) < 6) ? -2 : -1);        CLKS(7,7,4); } break;
		case 0x40: op_40: { IncWordReg(AW);                     CLK(2); } break;
		case 0x41: op_41: { IncWordReg(CW);                     CLK(2); } break;
		case 0x42: op_42: { IncWordReg(DW);                     CLK(2); } break;
		case 0x43: op_43: { IncWordReg(BW);                     CLK(2); } break;
		case 0x44: op_44: { IncWordReg(SP);                     CLK(2); } break;
		case 0x45: op_45: { IncWordReg(BP);                     CLK(2); } break;
		case 0x46: op_46: { IncWordReg(IX);                     CLK(2); } break;
		case 0x47: op_47: { IncWordReg(IY);                     CLK(2); } break;
		case 0x48: op_48: { DecWordReg(AW);                     CLK(2); } break;
		case 0x49: op_49: { DecWordReg(CW);                     CLK(2); } break;
		case 0x4a: op_4a: { DecWordReg(DW);                     CLK(2); } break;
		case 0x4b: op_4b: { DecWordReg(BW);                     CLK(2); } break;
		case 0x4c: op_4c: { DecWordReg(SP);                     CLK(2); } break;
		case 0x4d: op_4d: { DecWordReg(BP);                     CLK(2); } break;
		case 0x4e: op_4e: { DecWordReg(IX);                     CLK(2); } break;
		case 0x4f: op_4f: { DecWordReg(IY);                     CLK(2); } break;
		case 0x50: op_50: { PUSH(Wreg(AW));                 CLKS(12,8,3); } break;
		case 0x51: op_51: { PUSH(Wreg(CW));                 CLKS(12,8,3); } break;
		case 0x52: op_52: { PUSH(Wreg(DW));                 CLKS(12,8,3); } break;
		case 0x53: op_53: { PUSH(Wreg(BW));                 CLKS(12,8,3); } break;
		case 0x54: op_54: { PUSH(Wreg(SP));                 CLKS(12,8,3); } break;
		case 0x55: op_55: { PUSH(Wreg(BP));                 CLKS(12,8,3); } break;
		case 0x56: op_56: { PUSH(Wreg(IX));                 CLKS(12,8,3); } break;
		case 0x57: op_57: { PUSH(Wreg(IY));                 CLKS(12,8,3); } break;
		case 0x58: op_58: { POP(Wreg(AW));                  CLKS(12,8,5); } break;
		case 0x59: op_59: { POP(Wreg(CW));                  CLKS(12,8,5); } break;
		case 0x5a: op_5a: { POP(Wreg(DW));                  CLKS(12,8,5); } break;
		case 0x5b: op_5b: { POP(Wreg(BW));                  CLKS(12,8,5); } break;
		case 0x5c: op_5c: { POP(Wreg(SP));                  CLKS(12,8,5); } break;
		case 0x5d: op_5d: { POP(Wreg(BP));                  CLKS(12,8,5); } break;
		case 0x5e: op_5e: { POP(Wreg(IX));                  CLKS(12,8,5); } break;
		case 0x5f: op_5f: { POP(Wreg(IY));                  CLKS(12,8,5); } break;
		case 0x60: op_60: { // i_pusha
			unsigned tmp=Wreg(SP);
	PUSH(Wreg(AW));
	PUSH(Wreg(CW));
	PUSH(Wreg(DW));
	PUSH(Wreg(BW));
	PUSH(tmp);
	PUSH(Wreg(BP));
	PUSH(Wreg(IX));
	PUSH(Wreg(IY));
	CLKS(67,35,20);
			break; }
		case 0x61: op_61: { // i_popa
			POP(Wreg(IY));
	POP(Wreg(IX));
	POP(Wreg(BP));
	POP(S->popa_tmp);
	POP(Wreg(BW));
	POP(Wreg(DW));
	POP(Wreg(CW));
	POP(Wreg(AW));
	CLKS(75,43,22);
			break; }
		case 0x62: op_62: { // i_chkind
			uint32_t low,high,tmp;
	GetModRM;
	low = GetRMWord(ModRM);
	high= GetnextRMWord;
	tmp= RegWord(ModRM);
	if (tmp<low || tmp>high) {
		INTERRUPT(NEC_CHKIND_VECTOR, BRK);
	}
	CLK(20);
	;
			break; }
		case 0x64: op_64: rep_kind = 0; goto start_rep;
		case 0x65: op_65: rep_kind = 1; goto start_rep;
		case 0x68: op_68: { uint32_t tmp; tmp = FETCHW(); PUSH(tmp); CLKW(12,12,5,12,8,5,Wreg(SP)); } break;
		case 0x69: op_69: { uint32_t tmp; DEF_r16w; tmp = FETCHW(); dst = (int32_t)((int16_t)src)*(int32_t)((int16_t)tmp); S->CarryVal = S->OverVal = (((int32_t)dst) >> 15 != 0) && (((int32_t)dst) >> 15 != -1); RegWord(ModRM)=(WORD)dst; CLKM(42,42,12,52,48,16); } break;
		case 0x6a: op_6a: { uint32_t tmp = (WORD)((int16_t)((int8_t)FETCH())); PUSH(tmp); CLKW(11,11,5,11,7,3,Wreg(SP)); } break;
		case 0x6b: op_6b: { uint32_t src2; DEF_r16w; src2= (WORD)((int16_t)((int8_t)FETCH())); dst = (int32_t)((int16_t)src)*(int32_t)((int16_t)src2); S->CarryVal = S->OverVal = (((int32_t)dst) >> 15 != 0) && (((int32_t)dst) >> 15 != -1); RegWord(ModRM)=(WORD)dst; CLKM(34,34,12,44,40,16); } break;
		case 0x6c: op_6c: STR_insb break;
		case 0x6d: op_6d: STR_insw break;
		case 0x6e: op_6e: STR_outsb break;
		case 0x6f: op_6f: STR_outsw break;
		case 0x70: op_70: { JMP( OF);               CLKS(4,4,3); } break;
		case 0x71: op_71: { JMP(!OF);               CLKS(4,4,3); } break;
		case 0x72: op_72: { JMP( CF);               CLKS(4,4,3); } break;
		case 0x73: op_73: { JMP(!CF);               CLKS(4,4,3); } break;
		case 0x74: op_74: { JMP( ZF);               CLKS(4,4,3); } break;
		case 0x75: op_75: { JMP(!ZF);               CLKS(4,4,3); } break;
		case 0x76: op_76: { JMP(CF || ZF);          CLKS(4,4,3); } break;
		case 0x77: op_77: { JMP(!(CF || ZF));       CLKS(4,4,3); } break;
		case 0x78: op_78: { JMP( SF);               CLKS(4,4,3); } break;
		case 0x79: op_79: { JMP(!SF);               CLKS(4,4,3); } break;
		case 0x7a: op_7a: { JMP( PF);               CLKS(4,4,3); } break;
		case 0x7b: op_7b: { JMP(!PF);               CLKS(4,4,3); } break;
		case 0x7c: op_7c: { JMP((SF!=OF)&&(!ZF));   CLKS(4,4,3); } break;
		case 0x7d: op_7d: { JMP((ZF)||(SF==OF));    CLKS(4,4,3); } break;
		case 0x7e: op_7e: { JMP((ZF)||(SF!=OF));    CLKS(4,4,3); } break;
		case 0x7f: op_7f: { JMP((SF==OF)&&(!ZF));   CLKS(4,4,3); } break;
		case 0x80: op_80: { // i_80pre
			uint32_t dst, src; GetModRM; dst = GetRMByte(ModRM); src = FETCH();
	if (ModRM >=0xc0 ) CLKS(4,4,2); else if ((ModRM & 0x38)==0x38) CLKS(13,13,6); else CLKS(18,18,7);
	switch (ModRM & 0x38) {
		case 0x00: ADDB;            PutbackRMByte(ModRM,dst);   break;
		case 0x08: ORB;             PutbackRMByte(ModRM,dst);   break;
		case 0x10: ADDCB;           PutbackRMByte(ModRM,dst);   break;
		case 0x18: SUBCB;           PutbackRMByte(ModRM,dst);   break;
		case 0x20: ANDB;            PutbackRMByte(ModRM,dst);   break;
		case 0x28: SUBB;            PutbackRMByte(ModRM,dst);   break;
		case 0x30: XORB;            PutbackRMByte(ModRM,dst);   break;
		case 0x38: SUBB;            break;  /* CMP */
	}
			break; }
		case 0x81: op_81: { // i_81pre
			uint32_t dst, src; GetModRM; dst = GetRMWord(ModRM); src = FETCH(); src+= (FETCH() << 8);
	if (ModRM >=0xc0 ) CLKS(4,4,2); else if ((ModRM & 0x38)==0x38) CLKW(17,17,8,17,13,6,S->EA); else CLKW(26,26,11,26,18,7,S->EA);
	switch (ModRM & 0x38) {
		case 0x00: ADDW;            PutbackRMWord(ModRM,dst);   break;
		case 0x08: ORW;             PutbackRMWord(ModRM,dst);   break;
		case 0x10: ADDCW;           PutbackRMWord(ModRM,dst);   break;
		case 0x18: SUBCW;           PutbackRMWord(ModRM,dst);   break;
		case 0x20: ANDW;            PutbackRMWord(ModRM,dst);   break;
		case 0x28: SUBW;            PutbackRMWord(ModRM,dst);   break;
		case 0x30: XORW;            PutbackRMWord(ModRM,dst);   break;
		case 0x38: SUBW;            break;  /* CMP */
	}
			break; }
		case 0x82: op_82: { // i_82pre
			uint32_t dst, src; GetModRM; dst = GetRMByte(ModRM); src = (BYTE)((int8_t)FETCH());
	if (ModRM >=0xc0 ) CLKS(4,4,2); else if ((ModRM & 0x38)==0x38) CLKS(13,13,6); else CLKS(18,18,7);
	switch (ModRM & 0x38) {
		case 0x00: ADDB;            PutbackRMByte(ModRM,dst);   break;
		case 0x08: ORB;             PutbackRMByte(ModRM,dst);   break;
		case 0x10: ADDCB;           PutbackRMByte(ModRM,dst);   break;
		case 0x18: SUBCB;           PutbackRMByte(ModRM,dst);   break;
		case 0x20: ANDB;            PutbackRMByte(ModRM,dst);   break;
		case 0x28: SUBB;            PutbackRMByte(ModRM,dst);   break;
		case 0x30: XORB;            PutbackRMByte(ModRM,dst);   break;
		case 0x38: SUBB;            break;  /* CMP */
	}
			break; }
		case 0x83: op_83: { // i_83pre
			uint32_t dst, src; GetModRM; dst = GetRMWord(ModRM); src = (WORD)((int16_t)((int8_t)FETCH()));
	if (ModRM >=0xc0 ) CLKS(4,4,2); else if ((ModRM & 0x38)==0x38) CLKW(17,17,8,17,13,6,S->EA); else CLKW(26,26,11,26,18,7,S->EA);
	switch (ModRM & 0x38) {
		case 0x00: ADDW;            PutbackRMWord(ModRM,dst);   break;
		case 0x08: ORW;             PutbackRMWord(ModRM,dst);   break;
		case 0x10: ADDCW;           PutbackRMWord(ModRM,dst);   break;
		case 0x18: SUBCW;           PutbackRMWord(ModRM,dst);   break;
		case 0x20: ANDW;            PutbackRMWord(ModRM,dst);   break;
		case 0x28: SUBW;            PutbackRMWord(ModRM,dst);   break;
		case 0x30: XORW;            PutbackRMWord(ModRM,dst);   break;
		case 0x38: SUBW;            break;  /* CMP */
	}
			break; }
		case 0x84: op_84: { DEF_br8;  ANDB;   CLKM(2,2,2,10,10,6); } break;
		case 0x85: op_85: { DEF_wr16; ANDW;   CLKR(14,14,8,14,10,6,2,S->EA); } break;
		case 0x86: op_86: { DEF_br8;  RegByte(ModRM)=dst; PutbackRMByte(ModRM,src); CLKM(3,3,3,16,18,8); } break;
		case 0x87: op_87: { DEF_wr16; RegWord(ModRM)=dst; PutbackRMWord(ModRM,src); CLKR(24,24,12,24,16,8,3,S->EA); } break;
		case 0x88: op_88: { uint8_t  src; GetModRM; src = RegByte(ModRM);   PutRMByte(ModRM,src);   CLKM(2,2,2,9,9,3); } break;
		case 0x89: op_89: { uint16_t src; GetModRM; src = RegWord(ModRM);   PutRMWord(ModRM,src);   CLKR(13,13,5,13,9,3,2,S->EA); } break;
		case 0x8a: op_8a: { uint8_t  src; GetModRM; src = GetRMByte(ModRM); RegByte(ModRM)=src;     CLKM(2,2,2,11,11,5); } break;
		case 0x8b: op_8b: { uint16_t src; GetModRM; src = GetRMWord(ModRM); RegWord(ModRM)=src;     CLKR(15,15,7,15,11,5,2,S->EA); } break;
		case 0x8c: op_8c: { // i_mov_wsreg
			GetModRM;
	switch (ModRM & 0x38) {
		case 0x00: PutRMWord(ModRM,Sreg(DS1)); CLKR(14,14,5,14,10,3,2,S->EA); break;
		case 0x08: PutRMWord(ModRM,Sreg(PS)); CLKR(14,14,5,14,10,3,2,S->EA); break;
		case 0x10: PutRMWord(ModRM,Sreg(SS)); CLKR(14,14,5,14,10,3,2,S->EA); break;
		case 0x18: PutRMWord(ModRM,Sreg(DS0)); CLKR(14,14,5,14,10,3,2,S->EA); break;
		default:   ;
	}
			break; }
		case 0x8d: op_8d: { uint32_t ModRM = FETCH(); if (ModRM < 0xc0) { (void)GET_EA(ModRM); RegWord(ModRM)=S->EO; }  CLKS(4,4,2); } break;
		case 0x8e: op_8e: { // i_mov_sregw
			uint16_t src; GetModRM; src = GetRMWord(ModRM); CLKR(15,15,7,15,11,5,2,S->EA);
	switch (ModRM & 0x38) {
		case 0x00: Sreg(DS1) = src; break; /* mov es,ew */
		case 0x08: Sreg(PS) = src; break; /* mov cs,ew */
		case 0x10: Sreg(SS) = src; break; /* mov ss,ew */
		case 0x18: Sreg(DS0) = src; break; /* mov ds,ew */
		default:   ;
	}
	S->no_interrupt=1;
			break; }
		case 0x8f: op_8f: { uint16_t tmp; GetModRM; POP(tmp); PutRMWord(ModRM,tmp); CLK(21); } break;
		case 0x90: op_90: { CLK(3); /* { if (S->MF == 0) printf("90 -> %06x: \n",PC()); }  */ } break;
		case 0x91: op_91: { XchgAWReg(CW); CLK(3); } break;
		case 0x92: op_92: { XchgAWReg(DW); CLK(3); } break;
		case 0x93: op_93: { XchgAWReg(BW); CLK(3); } break;
		case 0x94: op_94: { XchgAWReg(SP); CLK(3); } break;
		case 0x95: op_95: { XchgAWReg(BP); CLK(3); } break;
		case 0x96: op_96: { XchgAWReg(IX); CLK(3); } break;
		case 0x97: op_97: { XchgAWReg(IY); CLK(3); } break;
		case 0x98: op_98: { Breg(AH) = (Breg(AL) & 0x80) ? 0xff : 0;      CLK(2); } break;
		case 0x99: op_99: { Wreg(DW) = (Breg(AH) & 0x80) ? 0xffff : 0;    CLK(4); } break;
		case 0x9a: op_9a: { uint32_t tmp, tmp2; tmp = FETCHW(); tmp2 = FETCHW(); PUSH(Sreg(PS)); PUSH(IPNOW); SETIP(tmp); Sreg(PS) = (WORD)tmp2; CHANGE_PC; CLKW(29,29,13,29,21,9,Wreg(SP)); } break;
		case 0x9b: op_9b: { if (!S->poll_state) SETIP(IPNOW - 1); CLK(5); } break;
		case 0x9c: op_9c: { uint16_t tmp = CompressFlags(); PUSH( tmp ); CLKS(12,8,3); } break;
		case 0x9d: op_9d: { uint32_t tmp; POP(tmp); ExpandFlags(tmp); CLKS(12,8,5);  } break;
		case 0x9e: op_9e: { uint32_t tmp = (CompressFlags() & 0xff00) | (Breg(AH) & 0xd5); ExpandFlags(tmp); CLKS(3,3,2); } break;
		case 0x9f: op_9f: { Breg(AH) = CompressFlags() & 0xff; CLKS(3,3,2); } break;
		case 0xa0: op_a0: { uint32_t addr; addr = FETCHW(); Breg(AL) = GetMemB(DS0, addr); CLKS(10,10,5); } break;
		case 0xa1: op_a1: { uint32_t addr; addr = FETCHW(); Wreg(AW) = GetMemW(DS0, addr); CLKW(14,14,7,14,10,5,addr); } break;
		case 0xa2: op_a2: { uint32_t addr; addr = FETCHW(); PutMemB(DS0, addr, Breg(AL));  CLKS(9,9,3); } break;
		case 0xa3: op_a3: { uint32_t addr; addr = FETCHW(); PutMemW(DS0, addr, Wreg(AW));  CLKW(13,13,5,13,9,3,addr); } break;
		case 0xa4: op_a4: STR_movsb break;
		case 0xa5: op_a5: STR_movsw break;
		case 0xa6: op_a6: STR_cmpsb break;
		case 0xa7: op_a7: STR_cmpsw break;
		case 0xa8: op_a8: { DEF_ald8;  ANDB; CLKS(4,4,2); } break;
		case 0xa9: op_a9: { DEF_axd16; ANDW; CLKS(4,4,2); } break;
		case 0xaa: op_aa: STR_stosb break;
		case 0xab: op_ab: STR_stosw break;
		case 0xac: op_ac: STR_lodsb break;
		case 0xad: op_ad: STR_lodsw break;
		case 0xae: op_ae: STR_scasb break;
		case 0xaf: op_af: STR_scasw break;
		case 0xb0: op_b0: { Breg(AL) = FETCH();   CLKS(4,4,2); } break;
		case 0xb1: op_b1: { Breg(CL) = FETCH(); CLKS(4,4,2); } break;
		case 0xb2: op_b2: { Breg(DL) = FETCH(); CLKS(4,4,2); } break;
		case 0xb3: op_b3: { Breg(BL) = FETCH(); CLKS(4,4,2); } break;
		case 0xb4: op_b4: { Breg(AH) = FETCH(); CLKS(4,4,2); } break;
		case 0xb5: op_b5: { Breg(CH) = FETCH(); CLKS(4,4,2); } break;
		case 0xb6: op_b6: { Breg(DH) = FETCH(); CLKS(4,4,2); } break;
		case 0xb7: op_b7: { Breg(BH) = FETCH();   CLKS(4,4,2); } break;
		case 0xb8: op_b8: { Breg(AL) = FETCH();    Breg(AH) = FETCH();    CLKS(4,4,2); } break;
		case 0xb9: op_b9: { Breg(CL) = FETCH();    Breg(CH) = FETCH();    CLKS(4,4,2); } break;
		case 0xba: op_ba: { Breg(DL) = FETCH();    Breg(DH) = FETCH();    CLKS(4,4,2); } break;
		case 0xbb: op_bb: { Breg(BL) = FETCH();    Breg(BH) = FETCH();    CLKS(4,4,2); } break;
		case 0xbc: op_bc: { Wreg(SP) = FETCHW();   CLKS(4,4,2); } break;
		case 0xbd: op_bd: { Wreg(BP) = FETCHW();   CLKS(4,4,2); } break;
		case 0xbe: op_be: { Wreg(IX) = FETCHW();   CLKS(4,4,2); } break;
		case 0xbf: op_bf: { Wreg(IY) = FETCHW();   CLKS(4,4,2); } break;
		case 0xc0: op_c0: { // i_rotshft_bd8
			uint32_t src, dst; uint8_t c;
	GetModRM; src = (unsigned)GetRMByte(ModRM); dst=src;
	c=FETCH();
	CLKM(7,7,2,19,19,6);
	if (c) switch (ModRM & 0x38) {
		case 0x00: do { ROL_BYTE;  c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x08: do { ROR_BYTE;  c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x10: do { ROLC_BYTE; c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x18: do { RORC_BYTE; c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x20: SHL_BYTE(c); break;
		case 0x28: SHR_BYTE(c); break;
		case 0x30: ; break;
		case 0x38: SHRA_BYTE(c); break;
	}
			break; }
		case 0xc1: op_c1: { // i_rotshft_wd8
			uint32_t src, dst;  uint8_t c;
	GetModRM; src = (unsigned)GetRMWord(ModRM); dst=src;
	c=FETCH();
	CLKM(7,7,2,27,19,6);
	if (c) switch (ModRM & 0x38) {
		case 0x00: do { ROL_WORD;  c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x08: do { ROR_WORD;  c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x10: do { ROLC_WORD; c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x18: do { RORC_WORD; c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x20: SHL_WORD(c); break;
		case 0x28: SHR_WORD(c); break;
		case 0x30: ; break;
		case 0x38: SHRA_WORD(c); break;
	}
			break; }
		case 0xc2: op_c2: { uint32_t count = FETCH(); count += FETCH() << 8; POP_IP(); Wreg(SP)+=count; CHANGE_PC; CLKS(24,24,10); } break;
		case 0xc3: op_c3: { POP_IP(); CHANGE_PC; CLKS(19,19,10); } break;
		case 0xc4: op_c4: { GetModRM; WORD tmp = GetRMWord(ModRM); RegWord(ModRM)=tmp; Sreg(DS1) = GetnextRMWord; CLKW(26,26,14,26,18,10,S->EA); } break;
		case 0xc5: op_c5: { GetModRM; WORD tmp = GetRMWord(ModRM); RegWord(ModRM)=tmp; Sreg(DS0) = GetnextRMWord; CLKW(26,26,14,26,18,10,S->EA); } break;
		case 0xc6: op_c6: { GetModRM; PutImmRMByte(ModRM); CLK((ModRM >=0xc0) ? 4 : 11); } break;
		case 0xc7: op_c7: { GetModRM; PutImmRMWord(ModRM); CLK((ModRM >=0xc0) ? 4 : 15); } break;
		case 0xc8: op_c8: { // i_enter
			uint32_t nb = FETCH();
	uint32_t i,level;

	CLK(23);
	nb += FETCH() << 8;
	level = FETCH();
	PUSH(Wreg(BP));
	Wreg(BP)=Wreg(SP);
	Wreg(SP) -= nb;
	for (i=1;i<level;i++) {
		PUSH(GetMemW(SS,Wreg(BP)-i*2));
		CLK(16);
	}
	if (level) PUSH(Wreg(BP));
			break; }
		case 0xc9: op_c9: { // i_leave
			Wreg(SP)=Wreg(BP);
	POP(Wreg(BP));
	CLK(8);
			break; }
		case 0xca: op_ca: { uint32_t count = FETCH(); count += FETCH() << 8; POP_IP(); POP(Sreg(PS)); Wreg(SP)+=count; CHANGE_PC; CLKS(32,32,16); } break;
		case 0xcb: op_cb: { POP_IP(); POP(Sreg(PS)); CHANGE_PC; CLKS(29,29,16); } break;
		case 0xcc: op_cc: { INTERRUPT(3, BRK); CLKS(50,50,24); } break;
		case 0xcd: op_cd: { INTERRUPT(FETCH(), BRK); CLKS(50,50,24); } break;
		case 0xce: op_ce: { if (OF) { INTERRUPT(NEC_BRKV_VECTOR, BRK); CLKS(52,52,26); } else CLK(3); } break;
		case 0xcf: op_cf: { POP_IP(); POP(Sreg(PS)); POPF_BODY; CHANGE_PC; CLKS(39,39,19); } break;
		case 0xd0: op_d0: { // i_rotshft_b
			uint32_t src, dst; GetModRM; src = (uint32_t)GetRMByte(ModRM); dst=src;
	CLKM(6,6,2,16,16,7);
	switch (ModRM & 0x38) {
		case 0x00: ROL_BYTE;  PutbackRMByte(ModRM,(BYTE)dst); S->OverVal = (src^dst)&0x80; break;
		case 0x08: ROR_BYTE;  PutbackRMByte(ModRM,(BYTE)dst); S->OverVal = (src^dst)&0x80; break;
		case 0x10: ROLC_BYTE; PutbackRMByte(ModRM,(BYTE)dst); S->OverVal = (src^dst)&0x80; break;
		case 0x18: RORC_BYTE; PutbackRMByte(ModRM,(BYTE)dst); S->OverVal = (src^dst)&0x80; break;
		case 0x20: SHL_BYTE(1); S->OverVal = (src^dst)&0x80; break;
		case 0x28: SHR_BYTE(1); S->OverVal = (src^dst)&0x80; break;
		case 0x30: ; break;
		case 0x38: SHRA_BYTE(1); S->OverVal = 0; break;
	}
			break; }
		case 0xd1: op_d1: { // i_rotshft_w
			uint32_t src, dst; GetModRM; src = (uint32_t)GetRMWord(ModRM); dst=src;
	CLKM(6,6,2,24,16,7);
	switch (ModRM & 0x38) {
		case 0x00: ROL_WORD;  PutbackRMWord(ModRM,(WORD)dst); S->OverVal = (src^dst)&0x8000; break;
		case 0x08: ROR_WORD;  PutbackRMWord(ModRM,(WORD)dst); S->OverVal = (src^dst)&0x8000; break;
		case 0x10: ROLC_WORD; PutbackRMWord(ModRM,(WORD)dst); S->OverVal = (src^dst)&0x8000; break;
		case 0x18: RORC_WORD; PutbackRMWord(ModRM,(WORD)dst); S->OverVal = (src^dst)&0x8000; break;
		case 0x20: SHL_WORD(1); S->OverVal = (src^dst)&0x8000;  break;
		case 0x28: SHR_WORD(1); S->OverVal = (src^dst)&0x8000;  break;
		case 0x30: ; break;
		case 0x38: SHRA_WORD(1); S->OverVal = 0; break;
	}
			break; }
		case 0xd2: op_d2: { // i_rotshft_bcl
			uint32_t src, dst; uint8_t c; GetModRM; src = (uint32_t)GetRMByte(ModRM); dst=src;
	c=Breg(CL);
	CLKM(7,7,2,19,19,6);
	if (c) switch (ModRM & 0x38) {
		case 0x00: do { ROL_BYTE;  c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x08: do { ROR_BYTE;  c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x10: do { ROLC_BYTE; c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x18: do { RORC_BYTE; c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x20: SHL_BYTE(c); break;
		case 0x28: SHR_BYTE(c); break;
		case 0x30: ; break;
		case 0x38: SHRA_BYTE(c); break;
	}
			break; }
		case 0xd3: op_d3: { // i_rotshft_wcl
			uint32_t src, dst; uint8_t c; GetModRM; src = (uint32_t)GetRMWord(ModRM); dst=src;
	c=Breg(CL);
	CLKM(7,7,2,27,19,6);
	if (c) switch (ModRM & 0x38) {
		case 0x00: do { ROL_WORD;  c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x08: do { ROR_WORD;  c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x10: do { ROLC_WORD; c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x18: do { RORC_WORD; c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x20: SHL_WORD(c); break;
		case 0x28: SHR_WORD(c); break;
		case 0x30: ; break;
		case 0x38: SHRA_WORD(c); break;
	}
			break; }
		case 0xd4: op_d4: { FETCH(); Breg(AH) = Breg(AL) / 10; Breg(AL) %= 10; SetSZPF_Word(Wreg(AW)); CLKS(15,15,12); } break;
		case 0xd5: op_d5: { FETCH(); Breg(AL) = Breg(AH) * 10 + Breg(AL); Breg(AH) = 0; SetSZPF_Byte(Breg(AL)); CLKS(7,7,8); } break;
		case 0xd6: op_d6: { Breg(AL) = (CF)?0xff:0x00; CLK(3); ; } break;
		case 0xd7: op_d7: { uint32_t dest = (Wreg(BW)+Breg(AL))&0xffff; Breg(AL) = GetMemB(DS0, dest); CLKS(9,9,5); } break;
		case 0xd9: op_d9: case 0xda: op_da: case 0xdb: op_db: case 0xdc: op_dc: case 0xdd: op_dd: case 0xde: op_de: case 0xdf: op_df:
		case 0xd8: op_d8: { GetModRM; GetRMByte(ModRM); CLK(2);  ; } break;
		case 0xe0: op_e0: { int8_t disp = (int8_t)FETCH(); Wreg(CW)--; if (!ZF && Wreg(CW)) { SETIP(IPNOW + disp); /*CHANGE_PC;*/ CLKS(14,14,6); } else CLKS(5,5,3); } break;
		case 0xe1: op_e1: { int8_t disp = (int8_t)FETCH(); Wreg(CW)--; if ( ZF && Wreg(CW)) { SETIP(IPNOW + disp); /*CHANGE_PC;*/ CLKS(14,14,6); } else CLKS(5,5,3); } break;
		case 0xe2: op_e2: { int8_t disp = (int8_t)FETCH(); Wreg(CW)--; if (Wreg(CW)) { SETIP(IPNOW + disp); /*CHANGE_PC;*/ CLKS(13,13,6); } else CLKS(5,5,3); } break;
		case 0xe3: op_e3: { int8_t disp = (int8_t)FETCH(); if (Wreg(CW) == 0) { SETIP(IPNOW + disp); /*CHANGE_PC;*/ CLKS(13,13,6); } else CLKS(5,5,3); } break;
		case 0xe4: op_e4: { uint8_t port = FETCH(); Breg(AL) = IN8(port); CLKS(9,9,5); } break;
		case 0xe5: op_e5: { uint8_t port = FETCH(); Wreg(AW) = IN16(port); CLKW(13,13,7,13,9,5,port); } break;
		case 0xe6: op_e6: { uint8_t port = FETCH(); OUT8(port, Breg(AL)); CLKS(8,8,3); } break;
		case 0xe7: op_e7: { uint8_t port = FETCH(); OUT16(port, Wreg(AW)); CLKW(12,12,5,12,8,3,port); } break;
		case 0xe8: op_e8: { uint32_t tmp; tmp = FETCHW(); PUSH(IPNOW); SETIP(IPNOW + (int16_t)tmp); CHANGE_PC; CLK(24); } break;
		case 0xe9: op_e9: { uint32_t tmp; tmp = FETCHW(); SETIP(IPNOW + (int16_t)tmp); CHANGE_PC; CLK(15); } break;
		case 0xea: op_ea: { uint32_t tmp,tmp1; tmp = FETCHW(); tmp1 = FETCHW(); Sreg(PS) = (WORD)tmp1;     SETIP(tmp); CHANGE_PC; CLK(27); } break;
		case 0xeb: op_eb: { int tmp = (int)((int8_t)FETCH()); CLK(12); SETIP(IPNOW + tmp); if (unlikely(S->idle_on) && tmp < 0) nec_idle_see(S, IPNOW); } break;
		case 0xec: op_ec: { Breg(AL) = IN8(Wreg(DW)); CLKS(8,8,5); } break;
		case 0xed: op_ed: { Wreg(AW) = IN16(Wreg(DW)); CLKW(12,12,7,12,8,5,Wreg(DW)); } break;
		case 0xee: op_ee: { OUT8(Wreg(DW), Breg(AL)); CLKS(8,8,3); } break;
		case 0xef: op_ef: { OUT16(Wreg(DW), Wreg(AW)); CLKW(12,12,5,12,8,3,Wreg(DW)); } break;
		case 0xf0: op_f0: { S->no_interrupt=1; CLK(2); } break;
		case 0xf2: op_f2: rep_kind = 2; goto start_rep;
		case 0xf3: op_f3: rep_kind = 3; goto start_rep;
		case 0xf4: op_f4: { S->halted=1; ic=0; } break;
		case 0xf5: op_f5: { S->CarryVal = !CF; CLK(2); } break;
		case 0xf6: op_f6: { // i_f6pre
			uint32_t tmp; uint32_t uresult,uresult2; int32_t result,result2;
	GetModRM; tmp = GetRMByte(ModRM);
	switch (ModRM & 0x38) {
		case 0x00: tmp &= FETCH(); S->CarryVal = S->OverVal = 0; SetSZPF_Byte(tmp); CLK((ModRM >= 0xc0) ? 4 : 11); break; /* TEST */
		case 0x08: ; break;
		case 0x10: PutbackRMByte(ModRM,~tmp); CLK((ModRM >= 0xc0) ? 2 : 16); break; /* NOT */
		case 0x18: S->CarryVal=(tmp!=0); tmp=(~tmp)+1; SetSZPF_Byte(tmp); PutbackRMByte(ModRM,tmp&0xff); CLK((ModRM >= 0xc0) ? 2 : 16); break; /* NEG */
		case 0x20: uresult = Breg(AL)*tmp; Wreg(AW)=(WORD)uresult; S->CarryVal=S->OverVal=(Breg(AH)!=0); CLKM(30,30,8,36,36,12); break; /* MULU */
		case 0x28: result = (int16_t)((int8_t)Breg(AL))*(int16_t)((int8_t)tmp); Wreg(AW)=(WORD)result; S->CarryVal=S->OverVal=(Breg(AH)!=0); CLKM(47,47,8,57,53,12); break; /* MUL */
		case 0x30:
			if (tmp)
				DIVUB
			else
			{
				S->CarryVal = S->OverVal = 0;
				INTERRUPT(NEC_DIVIDE_VECTOR, BRK);
			}

			CLKM(25,25,11,35,31,15);
			break;
		case 0x38:
			if (tmp)
				DIVB
			else
			{
				S->CarryVal = S->OverVal = 0;
				INTERRUPT(NEC_DIVIDE_VECTOR, BRK);
			}

			CLKM(43,43,17,53,49,20);
			break;
	}
			break; }
		case 0xf7: op_f7: { // i_f7pre
			uint32_t tmp,tmp2; uint32_t uresult,uresult2; int32_t result,result2;
	GetModRM; tmp = GetRMWord(ModRM);
	switch (ModRM & 0x38) {
		case 0x00: tmp2 = FETCHW(); tmp &= tmp2; S->CarryVal = S->OverVal = 0; SetSZPF_Word(tmp); CLK((ModRM >= 0xc0) ? 4 : 11); break; /* TEST */
		case 0x08: ; break;
		case 0x10: PutbackRMWord(ModRM,~tmp); CLK((ModRM >= 0xc0) ? 2 : 16); break; /* NOT */
		case 0x18: S->CarryVal=(tmp!=0); tmp=(~tmp)+1; SetSZPF_Word(tmp); PutbackRMWord(ModRM,tmp&0xffff); CLK((ModRM >= 0xc0) ? 2 : 16); break; /* NEG */
		case 0x20: uresult = Wreg(AW)*tmp; Wreg(AW)=uresult&0xffff; Wreg(DW)=((uint32_t)uresult)>>16; S->CarryVal=S->OverVal=(Wreg(DW)!=0); CLKM(30,30,12,36,36,16); break; /* MULU */
		case 0x28: result = (int32_t)((int16_t)Wreg(AW))*(int32_t)((int16_t)tmp); Wreg(AW)=result&0xffff; Wreg(DW)=result>>16; S->CarryVal=S->OverVal=(Wreg(DW)!=0); CLKM(47,47,12,57,53,16); break; /* MUL */
		case 0x30:
			if (tmp)
				DIVUW
			else
			{
				S->CarryVal = S->OverVal = 0;
				INTERRUPT(NEC_DIVIDE_VECTOR, BRK);
			}

			CLKM(25,25,19,35,31,23);
			break;
		case 0x38:
			if (tmp)
				DIVW
			else
			{
				S->CarryVal = S->OverVal = 0;
				INTERRUPT(NEC_DIVIDE_VECTOR, BRK);
			}
			CLKM(43,43,24,53,49,28);
			break;
	}
			break; }
		case 0xf8: op_f8: { S->CarryVal = 0;  CLK(2); } break;
		case 0xf9: op_f9: { S->CarryVal = 1;  CLK(2); } break;
		case 0xfa: op_fa: { SetIF(0);         CLK(2); } break;
		case 0xfb: op_fb: { SetIF(1);         CLK(2); } break;
		case 0xfc: op_fc: { SetDF(0);         CLK(2); } break;
		case 0xfd: op_fd: { SetDF(1);         CLK(2); } break;
		case 0xfe: op_fe: { // i_fepre
			uint32_t tmp, tmp1; GetModRM; tmp=GetRMByte(ModRM);
	switch(ModRM & 0x38) {
		case 0x00: tmp1 = tmp+1; S->OverVal = (tmp==0x7f); SetAF(tmp1,tmp,1); SetSZPF_Byte(tmp1); PutbackRMByte(ModRM,(BYTE)tmp1); CLKM(2,2,2,16,16,7); break; /* INC */
		case 0x08: tmp1 = tmp-1; S->OverVal = (tmp==0x80); SetAF(tmp1,tmp,1); SetSZPF_Byte(tmp1); PutbackRMByte(ModRM,(BYTE)tmp1); CLKM(2,2,2,16,16,7); break; /* DEC */
		default:   ;
	}
			break; }
		case 0xff: op_ff: { // i_ffpre
			uint32_t tmp, tmp1; GetModRM; tmp=GetRMWord(ModRM);
	switch(ModRM & 0x38) {
		case 0x00: tmp1 = tmp+1; S->OverVal = (tmp==0x7fff); SetAF(tmp1,tmp,1); SetSZPF_Word(tmp1); PutbackRMWord(ModRM,(WORD)tmp1); CLKM(2,2,2,24,16,7); break; /* INC */
		case 0x08: tmp1 = tmp-1; S->OverVal = (tmp==0x8000); SetAF(tmp1,tmp,1); SetSZPF_Word(tmp1); PutbackRMWord(ModRM,(WORD)tmp1); CLKM(2,2,2,24,16,7); break; /* DEC */
		case 0x10: PUSH(IPNOW); SETIP(tmp); CHANGE_PC; CLK((ModRM >= 0xc0) ? 16 : 20); break; /* CALL */
		case 0x18: tmp1 = Sreg(PS); Sreg(PS) = GetnextRMWord; PUSH(tmp1); PUSH(IPNOW); SETIP(tmp); CHANGE_PC; CLK((ModRM >= 0xc0) ? 16 : 26); break; /* CALL FAR */
		case 0x20: SETIP(tmp); CHANGE_PC; CLK(13); break; /* JMP */
		case 0x28: SETIP(tmp); Sreg(PS) = GetnextRMWord; CHANGE_PC; CLK(15); break; /* JMP FAR */
		case 0x30: PUSH(tmp); CLK(4); break;
		default:   ;
	}
			break; }

		default: CLK(10); break;
		case 0x66: op_66: case 0x67: op_67: CLK(10); break;
#if NEC_CORE_V35
		case 0x63: op_63: { INTERRUPT(FETCH(), BRKN); CLKS(50,50,24); } break;
		case 0xf1: op_f1: { INTERRUPT(FETCH(), BRKS); CLKS(50,50,24); } break;
#else
		case 0x63: op_63: case 0xf1: op_f1: CLK(10); break;
#endif
		}
		goto op_end;

start_rep:
		op = FETCHOP();
		switch (op) {
		case 0x26: S->seg_prefix = 1; S->prefix_base = (uint32_t)Sreg(DS1) << 4; op = FETCHOP(); CLK(2); break;
		case 0x2e: S->seg_prefix = 1; S->prefix_base = (uint32_t)Sreg(PS) << 4; op = FETCHOP(); CLK(2); break;
		case 0x36: S->seg_prefix = 1; S->prefix_base = (uint32_t)Sreg(SS) << 4; op = FETCHOP(); CLK(2); break;
		case 0x3e: S->seg_prefix = 1; S->prefix_base = (uint32_t)Sreg(DS0) << 4; op = FETCHOP(); CLK(2); break;
		}
do_rep:
		{
			uint32_t cw = Wreg(CW);
			// (a slice's end ends the loop: see rep_q)
			int q_floor = 0;
			if (unlikely(S->rep_q)) {
				const int done = (int)(((uint32_t)S->total + (uint32_t)(cycles - ic)) % (uint32_t)S->rep_q);
				q_floor = ic - (S->rep_q - done);
				if (q_floor < 0)
					q_floor = 0;
			}
#define REPF (rep_kind == 0 ? !CF : rep_kind == 1 ? CF : 1)
#define REPZ (rep_kind == 0 ? !CF : rep_kind == 1 ? CF : rep_kind == 2 ? !ZF : ZF)
#define REP1(code, name, test) case code: CLK(2); if (cw) do { STR_##name; cw--; } while (cw > 0 && (test) && ic > q_floor); Wreg(CW) = (uint16_t)cw; break;
			switch (op) {
			REP1(0x6c, insb, REPF)
			REP1(0x6d, insw, REPF)
			REP1(0x6e, outsb, REPF)
			REP1(0x6f, outsw, REPF)
			REP1(0xa4, movsb, REPF)
			REP1(0xa5, movsw, REPF)
			REP1(0xa6, cmpsb, REPZ)
			REP1(0xa7, cmpsw, REPZ)
			REP1(0xaa, stosb, REPF)
			REP1(0xab, stosw, REPF)
			REP1(0xac, lodsb, REPF)
			REP1(0xad, lodsw, REPF)
			REP1(0xae, scasb, REPZ)
			REP1(0xaf, scasw, REPZ)
			default: goto dispatch;   // (invalid: the instruction alone)
			}
			const int z86 = (op & 0x86) == 0x86;
			const int more = rep_kind == 0 ? !CF : rep_kind == 1 ? CF : rep_kind == 2 ? !(ZF && z86) : !(!ZF && z86);
			if (cw && more) {
				S->rep_ip = IPNOW;
				SETIP(S->prev_ip);
				S->rep_params = op << 8 | (uint32_t)rep_kind | (S->seg_prefix ? 4u : 0u);
			}
		}
op_end:
		S->ip = IPNOW;
		S->seg_prefix = 0;
		// (do_prefetch: a fetch is one byte of the queue; MAME's loops as a few shifts)
		pf -= (int)(fp - fs);
		if (pf < 0) {
			const int n = -pf;
			int k = cc > 0 ? (cc - 1) >> PCS : 0;
			if (k > n)
				k = n;
			cc -= k << PCS;
			ic -= (n - k) << PCS;
			pf = 0;
		}
		if (S->prefetch_reset) {
			pf = 0;
			S->prefetch_reset = 0;
		} else {   // (cc >= 0 and pf <= 6: the same as only when cc >= 1 << PCS and pf < 6)
			pf += cc >> PCS;
			if (pf > 6)
				pf = 6;
		}
		if (unlikely(S->stop)) {   // (nec_stop: MAME abort_timeslice, the instruction finished)
			cycles -= ic;
			ic = 0;
		}
	}
	S->prefetch_count = pf;
	return cycles - ic;
}

#undef CLKS
#undef CLKW
#undef CLKM
#undef CLKR
#undef NEC_IS_V33
#undef NEC_DIV_QUIRK
#undef PCS
#undef R
#undef RBYTES
#undef Wreg
#undef Sreg
#undef Breg
#undef PF
#undef RB
#undef CompressFlags
#undef ExpandFlags
#undef read_mem_byte
#undef read_mem_word
#undef write_mem_byte
#undef write_mem_word
#undef IN8
#undef OUT8
#undef IN16
#undef OUT16
#undef SegBase
#undef DefaultBase
#undef GetMemB
#undef GetMemW
#undef PutMemB
#undef PutMemW
#undef PUSH
#undef POP
#undef POP_IP
#undef POPF_BODY
#undef SCLK
#undef SCLKS
#undef CHANGE_PC
#undef CLK
#undef IPNOW
#undef SETIP
#undef FETCH
#undef FETCHW
#undef FETCHOP
#undef GET_EA
#undef INTERRUPT
#undef NEC_BRK
#undef BRKXA
#undef BRKEM
#undef RegWord
#undef RegByte
#undef RMW
#undef RMB
#undef GetRMWord
#undef PutbackRMWord
#undef GetnextRMWord
#undef PutRMWord
#undef PutImmRMWord
#undef GetRMByte
#undef PutRMByte
#undef PutImmRMByte
#undef PutbackRMByte
#undef DEF_br8
#undef DEF_wr16
#undef DEF_r8b
#undef DEF_r16w
#undef DEF_ald8
#undef DEF_axd16
#undef GetModRM
#undef REPF
#undef REPZ
#undef REP1
