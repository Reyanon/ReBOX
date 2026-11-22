/*  TMS34010 JIT (ADR-049 first stage, ADR-059 second stage): the guest code in the program ROM as ARM blocks
    (included by tms34010.c; ARM hosts with executable memory from the frontend, rbx_jit_alloc). The instruction
    semantics are MAME's (tms34010_ops.inc / tms34010_gfx.inc, BSD-3-Clause, copyright-holders: Alex Pasadyn,
    Zsolt Vasvari; parts based on code by Aaron Giles): the hot register / immediate / jump instructions written out
    in ARM with the same results and flags, the rest by a call of the handler.
    This file is part of ReBOX. GPL-2.0-or-later.

    A block is decoded from a pc in a code ROM page (tms34010_code_rom, never written) by the instruction lengths
    (s_tj_len), up to TJ_MAXI instructions, past conditional jumps (the taken side leaves), until an unconditional
    transfer or the page's end. ROM immediates are constants. Every instruction ends as the interpreter's step does:
    m_icount lowered by the handler's cycles, then the block left when it is <= 0 (m_pc / m_ppc stored as the
    interpreter has them). A handler call gets m_ppc = pc, m_pc = pc + 16 (the handler reads its words from there);
    when m_pc is not the next instruction afterwards (a transfer, an interrupt taken inside) the block goes on at the
    block of m_pc (s_tj_tab) or leaves. Native instructions keep m_regs / m_st in memory (flags materialised at once)
    and touch no memory, so nothing outside the CPU can see a difference. With the polling loop skip on, a taken
    backward native jump calls spin_back as the sw_ wrappers do. A static exit has a slot (nop) patched to a branch
    into the next block when the run loop finds it (block chaining). Off: RBX_TMS_NOJIT=1; RBX_TMS_JITSTAT=1: counts.
    Host registers in a block: r4 = m_regs, r5 = &m_icount, r6 = &m_st, r7 = &m_pc, r8 = &m_ppc, r9 = s_tj_tab,
    r10 = s_tj_flag. */

#if defined(__arm__)
#define TJ_CODE_SIZE (2u << 20)
#define TJ_TAB_BITS 14
#define TJ_MAXI 48
#define TJ_RESERVE 6144u   // words (the largest block)
typedef uint32_t (*tj_enter_fn)(uint32_t code);
static uint32_t *s_tj_code;            // executable (NULL: none yet)
static unsigned s_tj_used, s_tj_size;  // words
static int s_tj_none;
static int s_tj_on = -1;               // (-1: RBX_TMS_NOJIT not read yet)
static struct tj_ent { uint32_t pc, fn; } *s_tj_tab;
static uint8_t s_tj_rom[4096];         // the code ROM pages (by 1 MB)
static opcode_func const *s_tj_optab;  // the handler table the blocks call
static uint32_t s_tj_flag[32];         // ARM NZCV -> ST N C Z V (16..31: C inverted, after a subtraction)
static uint32_t s_tj_enter, s_tj_exit0, s_tj_exitr, s_tj_disp;   // the shared routines (code addresses)
static uint8_t s_tj_flushed;
static uint32_t s_tj_buf[TJ_RESERVE];
static unsigned s_tj_n;
static uint64_t s_tj_runs, s_tj_steps, s_tj_made, s_tj_chained, s_tj_flushes;   // (RBX_TMS_JITSTAT=1: printed)
static int s_tj_nochain;
static void spin_back(void);

// ---- the decode tables ----
enum {
	TK_CALL = 0, TK_END, TK_J, TK_DSJS, TK_MOVIW, TK_MOVIL, TK_MOVK, TK_MOVRR, TK_CMP, TK_CMPIW, TK_CMPIL, TK_ADD,
	TK_SUB, TK_ADDK, TK_SUBK, TK_ADDIW, TK_ADDIL, TK_SUBIW, TK_SUBIL, TK_AND, TK_ANDN, TK_ANDI, TK_OR, TK_ORI, TK_XOR,
	TK_XORI, TK_NOT, TK_BTSTK, TK_RNR, TK_RNOR, TK_RAR
};
static uint8_t s_tj_kind[4096], s_tj_len[4096], s_tj_file[4096], s_tj_cc[4096];   // file: bit 0 dst B, bit 1 src B, bit 2 field 1
static uint16_t s_tj_cmask[16];   // per condition: taken for the ST >> 28 values
static uint8_t s_tj_init;

static void tj_tables(void)
{
	static const struct { opcode_func f; uint8_t k, file, len; } nat[] = {
		{ movi_w_a, TK_MOVIW, 0, 2 }, { movi_w_b, TK_MOVIW, 1, 2 }, { movi_l_a, TK_MOVIL, 0, 3 }, { movi_l_b, TK_MOVIL, 1, 3 },
		{ movk_a, TK_MOVK, 0, 1 }, { movk_b, TK_MOVK, 1, 1 },
		{ move_rr_a, TK_MOVRR, 0, 1 }, { move_rr_b, TK_MOVRR, 3, 1 }, { move_rr_ax, TK_MOVRR, 1, 1 }, { move_rr_bx, TK_MOVRR, 2, 1 },
		{ cmp_a, TK_CMP, 0, 1 }, { cmp_b, TK_CMP, 3, 1 }, { cmpi_w_a, TK_CMPIW, 0, 2 }, { cmpi_w_b, TK_CMPIW, 1, 2 },
		{ cmpi_l_a, TK_CMPIL, 0, 3 }, { cmpi_l_b, TK_CMPIL, 1, 3 },
		{ add_a, TK_ADD, 0, 1 }, { add_b, TK_ADD, 3, 1 }, { sub_a, TK_SUB, 0, 1 }, { sub_b, TK_SUB, 3, 1 },
		{ addk_a, TK_ADDK, 0, 1 }, { addk_b, TK_ADDK, 1, 1 }, { subk_a, TK_SUBK, 0, 1 }, { subk_b, TK_SUBK, 1, 1 },
		{ addi_w_a, TK_ADDIW, 0, 2 }, { addi_w_b, TK_ADDIW, 1, 2 }, { addi_l_a, TK_ADDIL, 0, 3 }, { addi_l_b, TK_ADDIL, 1, 3 },
		{ subi_w_a, TK_SUBIW, 0, 2 }, { subi_w_b, TK_SUBIW, 1, 2 }, { subi_l_a, TK_SUBIL, 0, 3 }, { subi_l_b, TK_SUBIL, 1, 3 },
		{ and_a, TK_AND, 0, 1 }, { and_b, TK_AND, 3, 1 }, { andn_a, TK_ANDN, 0, 1 }, { andn_b, TK_ANDN, 3, 1 },
		{ andi_a, TK_ANDI, 0, 3 }, { andi_b, TK_ANDI, 1, 3 }, { or_a, TK_OR, 0, 1 }, { or_b, TK_OR, 3, 1 },
		{ ori_a, TK_ORI, 0, 3 }, { ori_b, TK_ORI, 1, 3 }, { xor_a, TK_XOR, 0, 1 }, { xor_b, TK_XOR, 3, 1 },
		{ xori_a, TK_XORI, 0, 3 }, { xori_b, TK_XORI, 1, 3 }, { not_a, TK_NOT, 0, 1 }, { not_b, TK_NOT, 1, 1 },
		{ btst_k_a, TK_BTSTK, 0, 1 }, { btst_k_b, TK_BTSTK, 1, 1 }, { dsjs_a, TK_DSJS, 0, 1 }, { dsjs_b, TK_DSJS, 1, 1 },
		// handler calls: the lengths (a wrong one only costs speed: m_pc is checked after the call) and the ends
		{ mmtm_a, TK_CALL, 0, 2 }, { mmtm_b, TK_CALL, 0, 2 }, { mmfm_a, TK_CALL, 0, 2 }, { mmfm_b, TK_CALL, 0, 2 },
		{ dsj_a, TK_CALL, 0, 2 }, { dsj_b, TK_CALL, 0, 2 }, { dsjeq_a, TK_CALL, 0, 2 }, { dsjeq_b, TK_CALL, 0, 2 },
		{ dsjne_a, TK_CALL, 0, 2 }, { dsjne_b, TK_CALL, 0, 2 },
		{ move0_nr_a, TK_RNR, 0, 1 }, { move0_nr_b, TK_RNR, 3, 1 }, { move1_nr_a, TK_RNR, 4, 1 }, { move1_nr_b, TK_RNR, 7, 1 },
		{ move0_no_r_a, TK_RNOR, 0, 2 }, { move0_no_r_b, TK_RNOR, 3, 2 }, { move1_no_r_a, TK_RNOR, 4, 2 }, { move1_no_r_b, TK_RNOR, 7, 2 },
		{ move0_r_no_a, TK_CALL, 0, 2 }, { move0_r_no_b, TK_CALL, 0, 2 }, { move1_r_no_a, TK_CALL, 0, 2 }, { move1_r_no_b, TK_CALL, 0, 2 },
		{ move0_no_ni_a, TK_CALL, 0, 2 }, { move0_no_ni_b, TK_CALL, 0, 2 }, { move1_no_ni_a, TK_CALL, 0, 2 }, { move1_no_ni_b, TK_CALL, 0, 2 },
		{ move0_no_no_a, TK_CALL, 0, 3 }, { move0_no_no_b, TK_CALL, 0, 3 }, { move1_no_no_a, TK_CALL, 0, 3 }, { move1_no_no_b, TK_CALL, 0, 3 },
		{ move0_ra_a, TK_CALL, 0, 3 }, { move0_ra_b, TK_CALL, 0, 3 }, { move1_ra_a, TK_CALL, 0, 3 }, { move1_ra_b, TK_CALL, 0, 3 },
		{ move0_ar_a, TK_RAR, 0, 3 }, { move0_ar_b, TK_RAR, 1, 3 }, { move1_ar_a, TK_RAR, 4, 3 }, { move1_ar_b, TK_RAR, 5, 3 },
		{ move0_a_ni_a, TK_CALL, 0, 3 }, { move0_a_ni_b, TK_CALL, 0, 3 }, { move1_a_ni_a, TK_CALL, 0, 3 }, { move1_a_ni_b, TK_CALL, 0, 3 },
		{ move0_aa, TK_CALL, 0, 5 }, { move1_aa, TK_CALL, 0, 5 },
		{ movb_no_r_a, TK_CALL, 0, 2 }, { movb_no_r_b, TK_CALL, 0, 2 }, { movb_r_no_a, TK_CALL, 0, 2 }, { movb_r_no_b, TK_CALL, 0, 2 },
		{ movb_no_no_a, TK_CALL, 0, 3 }, { movb_no_no_b, TK_CALL, 0, 3 }, { movb_ra_a, TK_CALL, 0, 3 }, { movb_ra_b, TK_CALL, 0, 3 },
		{ movb_ar_a, TK_CALL, 0, 3 }, { movb_ar_b, TK_CALL, 0, 3 }, { movb_aa, TK_CALL, 0, 5 },
		{ callr, TK_END, 0, 2 }, { calla, TK_END, 0, 3 }, { call_a, TK_END, 0, 1 }, { call_b, TK_END, 0, 1 },
		{ rets, TK_END, 0, 1 }, { reti, TK_END, 0, 1 }, { trap, TK_END, 0, 1 }, { jump_a, TK_END, 0, 1 },
		{ jump_b, TK_END, 0, 1 }, { exgpc_a, TK_END, 0, 1 }, { exgpc_b, TK_END, 0, 1 } };
	static const opcode_func jcc[16][3] = {
		{ j_UC_0, j_UC_8, j_UC_x }, { j_P_0, j_P_8, j_P_x }, { j_LS_0, j_LS_8, j_LS_x }, { j_HI_0, j_HI_8, j_HI_x },
		{ j_LT_0, j_LT_8, j_LT_x }, { j_GE_0, j_GE_8, j_GE_x }, { j_LE_0, j_LE_8, j_LE_x }, { j_GT_0, j_GT_8, j_GT_x },
		{ j_C_0, j_C_8, j_C_x }, { j_NC_0, j_NC_8, j_NC_x }, { j_EQ_0, j_EQ_8, j_EQ_x }, { j_NE_0, j_NE_8, j_NE_x },
		{ j_V_0, j_V_8, j_V_x }, { j_NV_0, j_NV_8, j_NV_x }, { j_N_0, j_N_8, j_N_x }, { j_NN_0, j_NN_8, j_NN_x } };
	for (int i = 0; i < 4096; i++) {
		const opcode_func f = s_opcode_table[i];
		s_tj_kind[i] = TK_CALL;
		s_tj_len[i] = 1;
		s_tj_file[i] = 0;
		for (size_t k = 0; k < sizeof(nat) / sizeof(nat[0]); k++)
			if (f == nat[k].f) {
				s_tj_kind[i] = nat[k].k;
				s_tj_len[i] = nat[k].len;
				s_tj_file[i] = nat[k].file;
			}
		for (int c = 0; c < 16; c++)
			for (int v = 0; v < 3; v++)
				if (f == jcc[c][v]) {
					s_tj_kind[i] = TK_J;
					s_tj_cc[i] = (uint8_t)(c << 2 | v);
				}
	}
	const char *off = getenv("RBX_TMS_JITOFF");   // (a mask of TK_ kinds made handler calls: for finding a bad one)
	const uint32_t offm = off ? (uint32_t)strtoul(off, NULL, 16) : 0;
	for (int i = 0; i < 4096; i++)
		if (offm >> s_tj_kind[i] & 1)
			s_tj_kind[i] = TK_CALL;
	// the conditions from the handlers themselves: taken for which N C Z V
	const uint32_t pc = m_pc, st = m_st;
	const int ic = m_icount;
	for (int c = 0; c < 16; c++) {
		s_tj_cmask[c] = 0;
		for (int v = 0; v < 16; v++) {
			m_st = (uint32_t)v << 28;
			m_pc = 0x1000;
			jcc[c][2](0x0001);
			if (m_pc != 0x1000)
				s_tj_cmask[c] |= (uint16_t)(1u << v);
		}
	}
	m_pc = pc;
	m_st = st;
	m_icount = ic;
	for (int v = 0; v < 16; v++) {
		const uint32_t n = v & 8 ? STBIT_N : 0, z = v & 4 ? STBIT_Z : 0, c = v & 2 ? STBIT_C : 0, o = v & 1 ? STBIT_V : 0;
		s_tj_flag[v] = n | z | c | o;
		s_tj_flag[16 + v] = n | z | (c ^ STBIT_C) | o;
	}
	s_tj_init = 1;
}

static void tj_stat(void)
{
	fprintf(stderr, "tms34010 jit: %llu block runs, %llu chained, %llu interpreted steps, %llu blocks made, %llu flushes\n",
		(unsigned long long)s_tj_runs, (unsigned long long)s_tj_chained, (unsigned long long)s_tj_steps,
		(unsigned long long)s_tj_made, (unsigned long long)s_tj_flushes);
}

// ---- the ARM emitter ----
static void TE(uint32_t w) { s_tj_buf[s_tj_n++] = w; }
static void te_imm(int rd, uint32_t v)   // movw (+ movt)
{
	TE(0xe3000000u | (v & 0xf000u) << 4 | (uint32_t)rd << 12 | (v & 0xfffu));
	if (v >> 16)
		TE(0xe3400000u | (v >> 16 & 0xf000u) << 4 | (uint32_t)rd << 12 | (v >> 16 & 0xfffu));
}
static void te_ld(int rt, int rn, int off) { TE(0xe5900000u | (uint32_t)rn << 16 | (uint32_t)rt << 12 | (uint32_t)off); }
static void te_st(int rt, int rn, int off) { TE(0xe5800000u | (uint32_t)rn << 16 | (uint32_t)rt << 12 | (uint32_t)off); }
static int te_rot(uint32_t v)   // the ARM immediate field of v (-1: none)
{
	for (int r = 0; r < 16; r++) {
		const uint32_t x = r ? (v << (2 * r)) | (v >> (32 - 2 * r)) : v;
		if (x < 256)
			return r << 8 | (int)x;
	}
	return -1;
}
// data processing with an immediate: op = the opcode bits (cond, S, rn, rd set by the caller)
static void te_dpi(uint32_t base, uint32_t v) { TE(base | 0x02000000u | (uint32_t)te_rot(v)); }
#define TJ_N 0x80000000u
#define TJ_Z 0x20000000u
#define TJ_NZV 0xb0000000u
#define TJ_NCZV 0xf0000000u
static void te_ldreg(int rt, int idx) { te_ld(rt, 4, idx * 4); }
static void te_streg(int rt, int idx) { te_st(rt, 4, idx * 4); }

// cold code after the block: exits
enum { TX_PLAIN, TX_SLOT, TX_TAKEN };
static struct tj_cold { unsigned at; uint8_t kind, cyc, spin; uint32_t ppc, pc; } s_tj_cold[TJ_MAXI * 3];
static unsigned s_tj_nc;
static void te_cold(uint32_t cond, uint8_t kind, uint32_t ppc, uint32_t pc, uint8_t cyc, uint8_t spin)
{
	s_tj_cold[s_tj_nc].at = s_tj_n;
	s_tj_cold[s_tj_nc].kind = kind;
	s_tj_cold[s_tj_nc].ppc = ppc;
	s_tj_cold[s_tj_nc].pc = pc;
	s_tj_cold[s_tj_nc].cyc = cyc;
	s_tj_cold[s_tj_nc++].spin = spin;
	TE(cond << 28 | 0x0a000000u);
}
static void te_b(uint32_t cond, uint32_t *base, uint32_t target)   // b<cond> to a code address
{
	const uint32_t here = (uint32_t)(uintptr_t)(base + s_tj_n);
	TE(cond << 28 | 0x0a000000u | (((target - here - 8) >> 2) & 0x00ffffffu));
}
static void te_pcs(uint32_t ppc, uint32_t pc)   // m_ppc, m_pc
{
	te_imm(0, ppc);
	te_st(0, 8, 0);
	te_imm(0, pc);
	te_st(0, 7, 0);
}
// m_icount -= c, then the exit (ppc, pc) when it is <= 0
static void te_cyc(int c, uint32_t ppc, uint32_t pc)
{
	te_ld(0, 5, 0);
	TE(0xe2500000u | (uint32_t)c);   // subs r0, r0, #c
	te_st(0, 5, 0);
	te_cold(0xd, TX_PLAIN, ppc, pc, 0, 0);   // ble
}
// ST's N C Z V from the ARM flags (sub: C inverted)
static void te_nczv(int sub)
{
	TE(0xe10fc000u);   // mrs r12, apsr
	TE(0xe1a0ce2cu);   // lsr r12, r12, #28
	if (sub)
		TE(0xe38cc010u);   // orr r12, r12, #16
	TE(0xe79ac10cu);   // ldr r12, [r10, r12, lsl #2]
	te_ld(1, 6, 0);
	te_dpi(0xe1c01000u | 1u << 16, TJ_NCZV);   // bic r1, r1, #NCZV
	TE(0xe181100cu);   // orr r1, r1, r12
	te_st(1, 6, 0);
}
// ST's N and Z from r0 (V cleared, C kept)
static void te_nz0(void)
{
	te_ld(1, 6, 0);
	te_dpi(0xe1c01000u | 1u << 16, TJ_NZV);   // bic r1, r1, #NZV
	TE(0xe3500000u);                           // cmp r0, #0
	te_dpi(0x41801000u | 1u << 16, TJ_N);      // orrmi r1, r1, #N
	te_dpi(0x01801000u | 1u << 16, TJ_Z);      // orreq r1, r1, #Z
	te_st(1, 6, 0);
}
// ST's Z from the ARM Z flag (the rest kept)
static void te_z(void)
{
	te_ld(1, 6, 0);
	te_dpi(0xe1c01000u | 1u << 16, TJ_Z);   // bic r1, r1, #Z
	te_dpi(0x01801000u | 1u << 16, TJ_Z);   // orreq r1, r1, #Z
	te_st(1, 6, 0);
}
// N and Z of a constant put in r0 (V cleared, C kept)
static void te_nzk(uint32_t v)
{
	te_ld(1, 6, 0);
	te_dpi(0xe1c01000u | 1u << 16, TJ_NZV);
	const uint32_t f = (v & 0x80000000u ? TJ_N : 0) | (v ? 0 : TJ_Z);
	if (f)
		te_dpi(0xe1801000u | 1u << 16, f);
	te_st(1, 6, 0);
}

static inline uint32_t tj_find(uint32_t pc)
{
	const struct tj_ent *e = &s_tj_tab[(pc >> 4) & ((1u << TJ_TAB_BITS) - 1)];
	return e->pc == pc ? e->fn : 0;
}

// the shared routines at the start of the code memory
static void tj_shared(void)
{
	uint32_t *base = s_tj_code;
	s_tj_n = 0;
	s_tj_enter = (uint32_t)(uintptr_t)(base + s_tj_n);
	TE(0xe92d4ff8u);   // push {r3-r11, lr}
	te_imm(4, (uint32_t)(uintptr_t)m_regs);
	te_imm(5, (uint32_t)(uintptr_t)&m_icount);
	te_imm(6, (uint32_t)(uintptr_t)&m_st);
	te_imm(7, (uint32_t)(uintptr_t)&m_pc);
	te_imm(8, (uint32_t)(uintptr_t)&m_ppc);
	te_imm(9, (uint32_t)(uintptr_t)s_tj_tab);
	te_imm(10, (uint32_t)(uintptr_t)s_tj_flag);
	TE(0xe12fff10u);   // bx r0
	s_tj_exit0 = (uint32_t)(uintptr_t)(base + s_tj_n);
	TE(0xe3a00000u);   // mov r0, #0
	s_tj_exitr = (uint32_t)(uintptr_t)(base + s_tj_n);
	TE(0xe8bd8ff8u);   // pop {r3-r11, pc}
	// the block of m_pc when there is one and m_icount > 0
	s_tj_disp = (uint32_t)(uintptr_t)(base + s_tj_n);
	te_ld(0, 5, 0);
	TE(0xe3500000u);   // cmp r0, #0
	te_b(0xd, base, s_tj_exit0);
	te_ld(0, 7, 0);
	TE(0xe7ed1250u);   // ubfx r1, r0, #4, #14
	TE(0xe0891181u);   // add r1, r9, r1, lsl #3
	te_ld(2, 1, 0);
	TE(0xe1520000u);   // cmp r2, r0
	TE(0x0591f004u);   // ldreq pc, [r1, #4]
	te_b(0xe, base, s_tj_exit0);
	rbx_jit_write_begin();
	memcpy(base, s_tj_buf, s_tj_n * 4);
	rbx_jit_write_end(base, s_tj_n * 4);
	s_tj_used = s_tj_n;
}

// all blocks forgotten; no code written here: tj_flush is called by tms34010_init / _map / _code_rom / _reset, which
// the frontend may run on another thread (the Vita's loader thread for the next game, where the VM domain is not
// open: a write into the code memory there faults), so the shared routines are made again by the emulation thread
// at its next block (tj_need_shared, called from tj_make)
static uint8_t s_tj_noshared = 1;   // (the shared routines are to be made before the next block)
static void tj_flush(void)
{
	if (s_tj_tab)
		for (unsigned i = 0; i < 1u << TJ_TAB_BITS; i++) {
			s_tj_tab[i].pc = 1;   // (never a pc)
			s_tj_tab[i].fn = 0;
		}
	s_tj_used = 0;
	s_tj_flushed = 1;
	s_tj_flushes++;
	s_tj_noshared = 1;
}
static inline void tj_need_shared(void)
{
	if (s_tj_noshared) {
		tj_shared();
		s_tj_noshared = 0;
	}
}

// at the game's end (tms34010_release): the block table given back, ADR-200
static void tj_release(void)
{
	free(s_tj_tab);
	s_tj_tab = NULL;
}

static int tj_ready(void)
{
	if (s_tj_code && s_tj_tab)
		return 1;
	if (s_tj_none)
		return 0;
	// (ADR-200: the table of the last game was given back; made again here. The shared routines, which hold its
	// address, are made again before the next block, as after any flush)
	if (s_tj_code) {
		s_tj_tab = (struct tj_ent *)calloc((size_t)1 << TJ_TAB_BITS, sizeof(*s_tj_tab));
		if (!s_tj_tab)
			return 0;   // (the interpreter, this game)
		tj_flush();
		return 1;
	}
	s_tj_tab = (struct tj_ent *)calloc((size_t)1 << TJ_TAB_BITS, sizeof(*s_tj_tab));
	for (unsigned sz = TJ_CODE_SIZE; sz >= (1u << 20) && !s_tj_code; sz -= 1u << 20)
		if ((s_tj_code = (uint32_t *)rbx_jit_alloc(sz)) != NULL)
			s_tj_size = sz / 4;
	if (getenv("RBX_TMS_JITKB") && (unsigned)atoi(getenv("RBX_TMS_JITKB")) * 256u < s_tj_size)   // (a smaller code memory: tests)
		s_tj_size = (unsigned)atoi(getenv("RBX_TMS_JITKB")) * 256u;
	if (!s_tj_tab || !s_tj_code) {
		s_tj_none = 1;
		s_tj_code = NULL;
		return 0;
	}
	tj_flush();
	return 1;
}

// ---- the block compiler ----
static inline int tj_idx(int b, int r) { return b ? 30 - r : r; }

// the block from pc0 (a code ROM page, aligned); its code address
static uint32_t tj_make(uint32_t pc0)
{
	if (s_tj_used + TJ_RESERVE > s_tj_size)
		tj_flush();
	tj_need_shared();
	uint32_t *const base = s_tj_code + s_tj_used;
	const uint16_t *const page = s_rpage[pc0 >> 20];
	const int spin = s_tj_optab != s_opcode_table;
	s_tj_n = 0;
	s_tj_nc = 0;
	uint32_t pc = pc0, ppc = pc0;
	int open = 1, ni = 0;   // (open: the flow goes on after the last instruction)
	for (int i = 0; i < TJ_MAXI && open; i++, ni++) {
		if ((pc ^ pc0) >> 20)   // (the next page: another block)
			break;
		const uint32_t ix = (pc & 0xfffff) >> 4;
		const uint16_t op = page[ix];
		const int len = s_tj_kind[op >> 4] == TK_J ? ((s_tj_cc[op >> 4] & 3) == 2 || (op & 15) ? 1 :
			(s_tj_cc[op >> 4] & 3) == 0 ? 2 : 3) : s_tj_len[op >> 4];
		if (ix + (unsigned)len > 0x10000u)   // (the page's end)
			break;
		const uint32_t w1 = page[ix + 1 < 0x10000u ? ix + 1 : ix], w2 = page[ix + 2 < 0x10000u ? ix + 2 : ix];
		const uint32_t lng = w1 | w2 << 16;
		const uint32_t np = pc + (uint32_t)len * 16;
		const int fl = s_tj_file[op >> 4];
		const int d = tj_idx(fl & 1, op & 15), s = tj_idx(fl >> 1 & 1, (op >> 5) & 15);
		const int k = (op >> 5) & 31, kk = k ? k : 32;
		ppc = pc;
		switch (s_tj_kind[op >> 4]) {
		case TK_MOVIW:
		case TK_MOVIL: {
			const uint32_t v = s_tj_kind[op >> 4] == TK_MOVIW ? (uint32_t)(int32_t)(int16_t)w1 : lng;
			te_imm(0, v);
			te_streg(0, d);
			te_nzk(v);
			te_cyc(len, pc, np);
			break;
		}
		case TK_MOVK:
			TE(0xe3a00000u | (uint32_t)kk);   // mov r0, #k
			te_streg(0, d);
			te_cyc(1, pc, np);
			break;
		case TK_MOVRR:
			te_ldreg(0, s);
			te_streg(0, d);
			te_nz0();
			te_cyc(1, pc, np);
			break;
		case TK_CMP:
			te_ldreg(0, d);
			te_ldreg(1, s);
			TE(0xe1500001u);   // cmp r0, r1
			te_nczv(1);
			te_cyc(1, pc, np);
			break;
		case TK_CMPIW:
		case TK_CMPIL:
			te_ldreg(0, d);
			te_imm(1, s_tj_kind[op >> 4] == TK_CMPIW ? ~(uint32_t)(int32_t)(int16_t)w1 : ~lng);
			TE(0xe1500001u);   // cmp r0, r1
			te_nczv(1);
			te_cyc(len, pc, np);
			break;
		case TK_ADD:
		case TK_SUB:
		case TK_ADDK:
		case TK_SUBK:
		case TK_ADDIW:
		case TK_ADDIL:
		case TK_SUBIW:
		case TK_SUBIL: {
			const int kd = s_tj_kind[op >> 4];
			const int sub = kd == TK_SUB || kd == TK_SUBK || kd == TK_SUBIW || kd == TK_SUBIL;
			te_ldreg(0, d);
			if (kd == TK_ADD || kd == TK_SUB)
				te_ldreg(1, s);
			else
				te_imm(1, kd == TK_ADDK || kd == TK_SUBK ? (uint32_t)kk : kd == TK_ADDIW ? (uint32_t)(int32_t)(int16_t)w1 :
					kd == TK_ADDIL ? lng : kd == TK_SUBIW ? ~(uint32_t)(int32_t)(int16_t)w1 : ~lng);
			TE(sub ? 0xe0500001u : 0xe0900001u);   // subs / adds r0, r0, r1
			te_streg(0, d);
			te_nczv(sub);
			te_cyc(len == 1 ? 1 : len, pc, np);
			break;
		}
		case TK_AND:
		case TK_ANDN:
		case TK_OR:
		case TK_XOR:
		case TK_ANDI:
		case TK_ORI:
		case TK_XORI:
		case TK_NOT: {
			const int kd = s_tj_kind[op >> 4];
			te_ldreg(0, d);
			if (kd == TK_AND || kd == TK_ANDN || kd == TK_OR || kd == TK_XOR)
				te_ldreg(1, s);
			else if (kd != TK_NOT)
				te_imm(1, kd == TK_ANDI ? ~lng : lng);
			TE(kd == TK_AND || kd == TK_ANDI ? 0xe0100001u : kd == TK_ANDN ? 0xe1d00001u : kd == TK_OR || kd == TK_ORI ?
				0xe1900001u : kd == TK_NOT ? 0xe1f00000u : 0xe0300001u);   // ands / bics / orrs / mvns / eors r0, r0(, r1)
			te_streg(0, d);
			te_z();
			te_cyc(len, pc, np);
			break;
		}
		case TK_BTSTK:
			te_ldreg(0, d);
			te_imm(1, 1u << (31 - k));   // (MAME: bit = 31 - K)
			TE(0xe1100001u);   // tst r0, r1
			te_z();
			te_cyc(1, pc, np);
			break;
		case TK_RNR:
		case TK_RNOR:
		case TK_RAR: {   // (a field read: the function of the field size and extension in ST, as RFIELD0 / RFIELD1)
			const int kd = s_tj_kind[op >> 4];
			if (kd == TK_RAR)
				te_imm(0, lng);
			else {
				te_ldreg(0, s);
				if (kd == TK_RNOR) {
					te_imm(1, (uint32_t)(int32_t)(int16_t)w1);
					TE(0xe0800001u);   // add r0, r0, r1
				}
			}
			te_ld(1, 6, 0);
			TE(0xe7e51051u | (fl & 4 ? 6u : 0u) << 7);   // ubfx r1, r1, #(0 / 6), #6
			te_imm(12, (uint32_t)(uintptr_t)s_rfield_functions);
			TE(0xe79cc101u);   // ldr r12, [r12, r1, lsl #2]
			TE(0xe12fff3cu);   // blx r12
			te_streg(0, d);
			te_nz0();
			te_cyc(kd == TK_RNR ? 3 : 5, pc, np);
			break;
		}
		case TK_DSJS: {
			const uint32_t t = op & 0x0400 ? pc + 16 - (uint32_t)k * 16 : pc + 16 + (uint32_t)k * 16;
			te_ldreg(0, d);
			TE(0xe2500001u);   // subs r0, r0, #1
			te_streg(0, d);
			te_cold(0x1, TX_TAKEN, pc, t, 2, (uint8_t)(spin && t <= pc));   // bne
			te_cyc(3, pc, np);
			break;
		}
		case TK_J: {
			const int c = s_tj_cc[op >> 4] >> 2, v = s_tj_cc[op >> 4] & 3;
			uint32_t t;
			int ct, cn;
			if (v == 2 || (op & 15)) {
				t = pc + 16 + (uint32_t)((int32_t)(int8_t)(op & 0xff) * 16);
				ct = 2; cn = 1;
			} else if (v == 0) {
				t = pc + 32 + (uint32_t)((int32_t)(int16_t)w1 * 16);
				ct = 3; cn = 2;
			} else {
				t = lng & ~15u;
				ct = 3; cn = 4;
			}
			const uint16_t m = s_tj_cmask[c];
			if (m == 0xffff) {   // (always)
				te_cold(0xe, TX_TAKEN, pc, t, (uint8_t)ct, (uint8_t)(spin && t <= pc));
				open = 0;
				break;
			}
			if (m) {
				te_ld(0, 6, 0);
				TE(0xe1a00e20u);   // lsr r0, r0, #28
				te_imm(1, m);
				TE(0xe1a01031u);   // lsr r1, r1, r0
				TE(0xe3110001u);   // tst r1, #1
				te_cold(0x1, TX_TAKEN, pc, t, (uint8_t)ct, (uint8_t)(spin && t <= pc));   // bne
			}
			te_cyc(cn, pc, np);
			break;
		}
		default: {   // TK_CALL, TK_END: the handler
			te_imm(0, pc);
			te_st(0, 8, 0);
			TE(0xe2800010u);   // add r0, r0, #16
			te_st(0, 7, 0);
			te_imm(0, op);
			te_imm(12, (uint32_t)(uintptr_t)s_tj_optab[op >> 4]);
			TE(0xe12fff3cu);   // blx r12
			if (s_tj_kind[op >> 4] == TK_END) {
				te_b(0xe, base, s_tj_disp);
				open = 0;
				break;
			}
			te_ld(0, 7, 0);
			te_imm(1, np);
			TE(0xe1500001u);   // cmp r0, r1
			te_b(0x1, base, s_tj_disp);   // bne: the flow went elsewhere
			te_ld(0, 5, 0);
			TE(0xe3500000u);   // cmp r0, #0
			te_b(0xd, base, s_tj_exit0);
			break;
		}
		}
		pc = np;
	}
	if (!ni)
		return 0;
	if (open) {   // (the next instruction: a static exit)
		te_cold(0xe, TX_SLOT, ppc, pc, 0, 0);
	}
	// the cold code
	for (unsigned j = 0; j < s_tj_nc; j++) {
		struct tj_cold *x = &s_tj_cold[j];
		s_tj_buf[x->at] |= (s_tj_n - (x->at + 2)) & 0x00ffffffu;
		if (x->kind == TX_PLAIN) {
			te_pcs(x->ppc, x->pc);
			te_b(0xe, base, s_tj_exit0);
			continue;
		}
		if (x->kind == TX_TAKEN) {
			te_ld(0, 5, 0);
			TE(0xe2500000u | x->cyc);   // subs r0, r0, #c
			te_st(0, 5, 0);
			if (x->spin) {
				te_pcs(x->ppc, x->pc);
				te_imm(12, (uint32_t)(uintptr_t)spin_back);
				TE(0xe12fff3cu);   // blx r12
				te_ld(0, 5, 0);
				TE(0xe3500000u);   // cmp r0, #0
				te_b(0xd, base, s_tj_exit0);
			} else {
				const unsigned over = s_tj_n;
				TE(0xca000000u);   // bgt (over the exit)
				te_pcs(x->ppc, x->pc);
				te_b(0xe, base, s_tj_exit0);
				s_tj_buf[over] |= (s_tj_n - (over + 2)) & 0x00ffffffu;
			}
		}
		// the slot: a branch into the next block once it is known
		const unsigned slot = s_tj_n;
		TE(0xe1a00000u);   // nop
		te_pcs(x->ppc, x->pc);
		TE(0xe24f0000u | (8u + 4u * (s_tj_n - slot)));   // sub r0, pc, #(to the slot)
		te_b(0xe, base, s_tj_exitr);
	}
	rbx_jit_write_begin();
	memcpy(base, s_tj_buf, s_tj_n * 4);
	rbx_jit_write_end(base, s_tj_n * 4);
	s_tj_used += s_tj_n;
	struct tj_ent *e = &s_tj_tab[(pc0 >> 4) & ((1u << TJ_TAB_BITS) - 1)];
	e->pc = pc0;
	e->fn = (uint32_t)(uintptr_t)base;
	s_tj_made++;
	return e->fn;
}

// the run loop with blocks; the interpreter's step where there is none
static void tj_run(opcode_func const *tab)
{
	if (tab != s_tj_optab) {
		s_tj_optab = tab;
		tj_flush();
	}
	uint32_t *slot = NULL;
	do {
		const uint32_t pc = m_pc;
		uint32_t f = tj_find(pc);
		if (!f && s_tj_rom[pc >> 20] && !(pc & 15))
			f = tj_make(pc);
		if (s_tj_flushed) {   // (by tj_make or by a handler in the last block: its slot is gone with the code)
			s_tj_flushed = 0;
			slot = NULL;
		}
		if (f) {
			if (slot && *slot == 0xe1a00000u && !s_tj_nochain) {
				rbx_jit_write_begin();
				*slot = 0xea000000u | (((f - (uint32_t)(uintptr_t)slot - 8) >> 2) & 0x00ffffffu);
				rbx_jit_write_end(slot, 4);
				s_tj_chained++;
			}
			s_tj_runs++;
			slot = (uint32_t *)(uintptr_t)((tj_enter_fn)(uintptr_t)s_tj_enter)(f);
			continue;
		}
		slot = NULL;
		s_tj_steps++;
		m_ppc = pc;
		const uint16_t op = ROPCODE();
		tab[op >> 4](op);
	} while (m_icount > 0);
}

static int tj_enabled(void)
{
	if (s_tj_on < 0)
	{
		s_tj_on = !(getenv("RBX_TMS_NOJIT") && atoi(getenv("RBX_TMS_NOJIT")));
		s_tj_nochain = getenv("RBX_TMS_JITNOCHAIN") != NULL;
		if (getenv("RBX_TMS_JITSTAT") && atoi(getenv("RBX_TMS_JITSTAT")))
			atexit(tj_stat);
	}
	if (!s_tj_init)
		tj_tables();
	return s_tj_on && tj_ready();
}
#else
static void tj_flush(void) {}
static void tj_release(void) {}
static uint8_t s_tj_rom[4096];
static int tj_enabled(void) { return 0; }
static void tj_run(opcode_func const *tab) { (void)tab; }
#endif
