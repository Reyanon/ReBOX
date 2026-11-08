/*  TMS57002 program pass compiled to ARM code (included by tms57002.c).
    This file is part of ReBOX. GPL-2.0-or-later.

    A sample's run starts at pc 0 (sync_w) with ca, id 0 and the instruction cache's decoding of the program for st1;
    when nothing else is in flight (no repeat, branch or memory transfer pending, the cycles those of the steady
    state), every step of execute_run is known from the cache but the data: the op chain (blocks, their links, the
    decoding at the slow ends from the cache's hash), ca and id, the memory transfer's steps (their count from st0),
    st1's decoding bits (only the ops set them, to constants), the instruction count and where the run ends. tj_walk
    follows execute_run's control flow over the cache into a list of the run's ops and transfer ends (tj_ir), and
    tj_emit turns it into ARM code: the macc / macc_write / macc_read copies at each instruction start are register
    renaming (three register pairs), ca / id / pc / icount and the static st1 / sti bits are stored at the end, the
    ops inline, get_cmem's slow path (the update queue, crm) and the overflow handling out of line, memory transfers
    whole at their last step (nothing else reads or writes the data space or the transfer's registers in between).
    A run that is not such a pass, or a program with an op not handled here (branches on data, repeats, ...), runs
    in the interpreter, which stays the reference (RBX_TMSCHK=N: every Nth compiled run compared with it).

    The code is made by the thread that runs the DSP in ordinary memory and copied into the executable memory by the
    emulation thread at a frame start (tms57002_jit_install, see es5510_jit.inc: ReST-V ADR-110); that memory is only
    ever added to. */

#include <stdio.h>
#include <stdlib.h>

// ---- the walk: a pass of execute_run over the cache, its steps in order ----
enum { TJ_OP, TJ_ISTART, TJ_XMDONE };
typedef struct {
	uint8_t kind;
	uint8_t param;      // TJ_OP: the op's param; TJ_XMDONE: 1 read, 0 write
	uint8_t ca, id;     // (TJ_OP: at the op)
	uint16_t op;
} tj_ir;

enum { TJ_MAXIR = 4096 };
typedef struct {
	tj_ir ir[TJ_MAXIR];
	int n;
	// the state at the end
	uint8_t pc, ca, id;
	int32_t icount;
	uint32_t st1c;      // st1 & ST1_CACHE
	uint32_t sti;       // the static sti bits (S_READ | S_WRITE | S_IDLE | S_BRANCH)
	int xm;             // the transfer at the end: 0 none, -1 not stepped yet, > 0 steps left
	int xm_used;        // a transfer was started in the pass
	int xm_n;           // a transfer's steps (from st0)
	const char *why;    // (not compiled: the reason)
} tj_walk_t;

// the ops handled (the generated op numbers, see tms57002.hxx)
static int tj_op_ok(int op)
{
	switch (op) {
	case 436: case 437: case 849: case 856: case 857: case 866: case 893:
	case 911: case 923: case 931: case 939: case 947: case 1003: case 1090:
	case 1103: case 1104: case 1110: case 1111: case 1117:
	case 1236: case 1240: case 1268: case 1380: case 1412: case 1432: case 1433: case 1434:
	case 1437: case 1441: case 1445: case 1449:
	case 1489: case 1529: case 1569: case 1570: case 1609: case 1610:
	case 1630: case 1631: case 1632: case 1633: case 1634: case 1635: case 1640: case 1642:
	case 1646: case 1648: case 1651:
		return 1;
	}
	return 0;
}

// a transfer's steps when it starts (xm_init's address: its low bits 0, see xm_steps)
static int tj_xm_steps(uint32_t st0v)
{
	switch (st0v & (ST0_WORD | ST0_SEL)) {
	case ST0_WORD | ST0_SEL: return 3;
	case ST0_WORD:           return 6;
	case ST0_SEL:            return 2;
	default:                 return 4;
	}
}

static int tj_walk(const tms57002_t *s, int32_t icount, tj_walk_t *w)
{
	const tms57002_cd *c = &s->cache;
	uint32_t st1c = s->st1 & ST1_CACHE;
	uint8_t pc = 0, ca = 0, id = 0;
	int idle = 0, branch = 0, xm = 0, xm_read = 0;
	int16_t pnode;
	int ipc = get_hash((tms57002_t *)s, 0, st1c, &pnode);
	w->n = 0;
	w->xm_used = 0;
	w->xm_n = tj_xm_steps(s->st0);
	w->why = NULL;
#define TJ_FAIL(m) do { w->why = (m); return 0; } while (0)
#define TJ_PUT(k, o, p) do { if (w->n >= TJ_MAXIR) TJ_FAIL("long"); \
		tj_ir *e_ = &w->ir[w->n++]; e_->kind = (k); e_->op = (uint16_t)(o); e_->param = (uint8_t)(p); e_->ca = ca; \
		e_->id = id; } while (0)
	if (icount <= 0 || icount > 1024)
		TJ_FAIL("icount");
	for (;;) {
		if (ipc < 0)
			TJ_FAIL("decode");   // (the interpreter would decode here)
		// the instruction's start (inst_start / INST_END): the transfer's step, the macc copies
		if (xm) {
			if (xm < 0)
				xm = w->xm_n;
			if (!--xm)
				TJ_PUT(TJ_XMDONE, 0, xm_read);
		}
		TJ_PUT(TJ_ISTART, 0, 0);
		// its ops
		while (c->inst[ipc].op >= 4) {
			const int op = c->inst[ipc].op, p = c->inst[ipc].param;
			if (op >= OP_COUNT)
				TJ_FAIL("unknown op");
			if (!tj_op_ok(op)) {
				static char m[32];
				snprintf(m, sizeof(m), "op %d", op);
				TJ_FAIL(m);
			}
			switch (op) {
			case 1103: case 1104: case 1110: case 1111:   // (wre / rde: none while a transfer is on)
				if (xm)
					break;
				TJ_PUT(TJ_OP, op, p);
				xm = -1;
				xm_read = op >= 1110;
				w->xm_used = 1;
				break;
			default:
				TJ_PUT(TJ_OP, op, p);
				break;
			}
			switch (op) {   // (their static effects)
			case 1630: case 1631: case 1632: case 1633: st1c = (st1c & ~(uint32_t)ST1_SFMA) | (uint32_t)(op - 1630) << ST1_SFMA_SHIFT; break;
			case 1634: case 1635: st1c = (st1c & ~(uint32_t)ST1_SFMO) | (uint32_t)(op - 1634) << ST1_SFMO_SHIFT; break;
			case 1640: st1c = (st1c & ~(uint32_t)ST1_RND) | 2u << ST1_RND_SHIFT; break;
			case 1642: st1c = (st1c & ~(uint32_t)ST1_RND) | 4u << ST1_RND_SHIFT; break;
			case 1646: idle = 1; break;
			case 1648: ca = (uint8_t)p; break;
			case 1651: pc = (uint8_t)p; branch = 1; break;
			}
			ipc = c->inst[ipc].next;
			if (ipc < 0)
				TJ_FAIL("chain");
		}
		// its end
		const tms57002_icd *e = &c->inst[ipc];
		if (e->op & INC_CA)
			ca++;
		if (e->op & INC_ID)
			id++;
		--icount;
		if (e->next != -1 && !s->xspec[ipc] && icount > 0) {
			ipc = e->next;
			continue;
		}
		// (inst_slow: no repeat)
		if (branch) {
			branch = 0;
			ipc = -1;
		} else {
			pc = (uint8_t)(s->xadr[ipc] + 1);
			ipc = e->next;
		}
		if (icount <= 0 || idle)
			break;
		if (ipc < 0)
			ipc = get_hash((tms57002_t *)s, pc, st1c, &pnode);
	}
#undef TJ_PUT
#undef TJ_FAIL
	w->pc = pc;
	w->ca = ca;
	w->id = id;
	w->icount = icount > 0 ? 0 : icount;
	w->st1c = st1c;
	w->xm = xm;
	w->sti = (idle ? S_IDLE : 0) | (xm ? (xm_read ? S_READ : S_WRITE) : 0);
	return 1;
}

// (the PC harness: RBX_TMSWALK=1 prints the walk of the first steady run)
static __attribute__((unused)) void tj_walk_print(const tms57002_t *s, const tj_walk_t *w)
{
	int ops = 0, insts = 0, xms = 0;
	for (int i = 0; i < w->n; i++) {
		ops += w->ir[i].kind == TJ_OP;
		insts += w->ir[i].kind == TJ_ISTART;
		xms += w->ir[i].kind == TJ_XMDONE;
	}
	fprintf(stderr, "TMSWALK st0 %06x st1 %06x: %s%s, %d instructions %d ops %d transfers, end pc %d ca %d id %d icount %d "
		"st1c %06x sti %x xm %d\n", s->st0, s->st1, w->why ? "not compiled: " : "ok", w->why ? w->why : "", insts, ops, xms,
		w->pc, w->ca, w->id, w->icount, w->st1c, w->sti, w->xm);
}

#if defined(__arm__)
// ---- ARM code ----
enum { R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, SP = 13, LR = 14, PC = 15 };
enum { C_EQ = 0, C_NE = 1, C_VS = 6, C_HI = 8, C_AL = 14 };
enum { DP_AND = 0, DP_EOR = 1, DP_SUB = 2, DP_RSB = 3, DP_ADD = 4, DP_ADC = 5, DP_TST = 8, DP_CMP = 10, DP_ORR = 12,
	DP_MOV = 13, DP_BIC = 14, DP_MVN = 15 };
enum { SH_LSL = 0, SH_LSR = 1, SH_ASR = 2 };
// registers: r11 = &s->dmem0 (the state's fields at signed 12-bit offsets from it), r10 = ba0, r4-r9 the three macc
// pairs (lo, hi); r0-r3, r12, lr scratch
#define TO(f) ((int)offsetof(tms57002_t, f) - (int)offsetof(tms57002_t, dmem0))

#define TJ_STAGE_WORDS 0x4000
#define TJ_VM_SIZE 0x200000
static uint32_t s_tj_stage[TJ_STAGE_WORDS];
static unsigned s_tj_n;    // (words emitted)
static int s_tj_fail;

static void tw(uint32_t op)
{
	if (s_tj_n < TJ_STAGE_WORDS)
		s_tj_stage[s_tj_n++] = op;
	else
		s_tj_fail = 1;
}

static int t_imm(uint32_t v, uint32_t *enc)
{
	for (uint32_t r = 0; r < 16; r++) {
		const uint32_t x = r ? (v << (2 * r)) | (v >> (32 - 2 * r)) : v;   // (v rotated left by 2r)
		if (x < 256) {
			*enc = r << 8 | x;
			return 1;
		}
	}
	return 0;
}

// data processing, register operand shifted by an immediate (sh 0: none)
static void t_dp(int cond, int opc, int sf, int rd, int rn, int rm, int shtype, int sh)
{
	tw((uint32_t)cond << 28 | (uint32_t)opc << 21 | (uint32_t)sf << 20 | (uint32_t)rn << 16 | (uint32_t)rd << 12 |
		(uint32_t)sh << 7 | (uint32_t)shtype << 5 | (uint32_t)rm);
}
static void t_dpi(int cond, int opc, int sf, int rd, int rn, uint32_t v)
{
	uint32_t enc;
	if (!t_imm(v, &enc)) {
		s_tj_fail = 1;
		return;
	}
	tw((uint32_t)cond << 28 | 0x02000000u | (uint32_t)opc << 21 | (uint32_t)sf << 20 | (uint32_t)rn << 16 |
		(uint32_t)rd << 12 | enc);
}
#define T_MOV(rd, rm)             t_dp(C_AL, DP_MOV, 0, rd, 0, rm, SH_LSL, 0)
#define T_SHIFT(rd, rm, type, n)  t_dp(C_AL, DP_MOV, 0, rd, 0, rm, type, n)
static void t_mov32c(int cond, int rd, uint32_t v)
{
	tw((uint32_t)cond << 28 | 0x03000000u | (v >> 12 & 0xf) << 16 | (uint32_t)rd << 12 | (v & 0xfff));   // movw
	if (v >> 16)
		tw((uint32_t)cond << 28 | 0x03400000u | (v >> 28 & 0xf) << 16 | (uint32_t)rd << 12 | (v >> 16 & 0xfff));   // movt
}
static void t_mov32(int rd, uint32_t v)
{
	uint32_t enc;
	if (t_imm(v, &enc))
		t_dpi(C_AL, DP_MOV, 0, rd, 0, v);
	else if (t_imm(~v, &enc))
		t_dpi(C_AL, DP_MVN, 0, rd, 0, ~v);
	else
		t_mov32c(C_AL, rd, v);
}
// ldr / str / ldrb / strb rt, [rn, #off] (off signed, 12 bits)
static void t_mem(int load, int byte, int rt, int rn, int off)
{
	if (off < -4095 || off > 4095) {
		s_tj_fail = 1;
		return;
	}
	tw(0xe5000000u | (off >= 0 ? 1u << 23 : 0) | (uint32_t)byte << 22 | (uint32_t)load << 20 | (uint32_t)rn << 16 |
		(uint32_t)rt << 12 | (uint32_t)(off >= 0 ? off : -off));
}
#define T_LDR(rt, f)   t_mem(1, 0, rt, R11, f)
#define T_STR(rt, f)   t_mem(0, 0, rt, R11, f)
#define T_LDRB(rt, f)  t_mem(1, 1, rt, R11, f)
#define T_STRB(rt, f)  t_mem(0, 1, rt, R11, f)
// ldr / str rt, [rn, rm, lsl #2]
static void t_memr(int load, int rt, int rn, int rm)
{
	tw(0xe7800100u | (uint32_t)load << 20 | (uint32_t)rn << 16 | (uint32_t)rt << 12 | (uint32_t)rm);
}
static void t_ubfx(int rd, int rn, int lsb, int width)
{
	tw(0xe7e00050u | (uint32_t)(width - 1) << 16 | (uint32_t)rd << 12 | (uint32_t)lsb << 7 | (uint32_t)rn);
}
static void t_smull(int lo, int hi, int rn, int rm) { tw(0xe0c00090u | (uint32_t)hi << 16 | (uint32_t)lo << 12 | (uint32_t)rm << 8 | (uint32_t)rn); }
static unsigned t_b(int cond) { tw((uint32_t)cond << 28 | 0x0a000000u); return s_tj_n - 1; }   // (patched by t_patch)
static void t_patch(unsigned at, unsigned to)
{
	if (at < TJ_STAGE_WORDS)
		s_tj_stage[at] = (s_tj_stage[at] & 0xff000000u) | ((uint32_t)((int)to - (int)at - 2) & 0x00ffffffu);
}
static void t_call(const void *f)
{
	t_mov32c(C_AL, R12, (uint32_t)(uintptr_t)f);
	tw(0xe12fff3cu);   // blx r12
}

// the out of line parts, emitted after the pass: their branch from the inline code and back
enum { ST_CMEM, ST_MOUT, ST_SUB };
typedef struct { uint8_t kind, cond; uint8_t a; uint8_t sat; uint32_t vneg, vpos; unsigned from, back; } tj_stub;
enum { TJ_MAXSTUB = 1024 };
static tj_stub s_tj_stub[TJ_MAXSTUB];
static int s_tj_nstub;
static void t_stub(int kind, int cond, int a, int sat, uint32_t vneg, uint32_t vpos)
{
	if (s_tj_nstub >= TJ_MAXSTUB) {
		s_tj_fail = 1;
		return;
	}
	tj_stub *st = &s_tj_stub[s_tj_nstub++];
	st->kind = (uint8_t)kind;
	st->cond = (uint8_t)cond;
	st->a = (uint8_t)a;
	st->sat = (uint8_t)sat;
	st->vneg = vneg;
	st->vpos = vpos;
	st->from = t_b(cond);
	st->back = s_tj_n;
}

// r0 = &dmem0[(k + ba0) & 0xff]'s index (k static: param or id); returns the register holding it
static int t_dadr(int rd, int k)
{
	if (!k)
		return R10;   // (ba0 < 256)
	t_dpi(C_AL, DP_ADD, 0, rd, R10, (uint32_t)k);
	t_dpi(C_AL, DP_AND, 0, rd, rd, 0xff);
	return rd;
}
static void t_dload(int rt, int k) { const int r = t_dadr(rt, k); t_memr(1, rt, R11, r); }
static void t_dstore(int rt, int k, int rtmp) { const int r = t_dadr(rtmp, k); t_memr(0, rt, R11, r); }

// r1 = get_cmem(a) (with dmem's load into r2 in between when dk >= 0: the slow path keeps r2, r3)
static void t_cmem(int a, int dk)
{
	T_LDRB(R0, TO(h.cm_slow) + a);
	T_LDR(R1, TO(cmem) + 4 * a);
	if (dk >= 0)
		t_dload(R2, dk);
	t_dpi(C_AL, DP_CMP, 1, 0, R0, 0);
	t_stub(ST_CMEM, C_NE, a, 0, 0, 0);
}

// the macc register pairs
typedef struct { int m, w, r; } tj_regs;   // (pair numbers 0-2: macc, macc_write, macc_read)
#define PL(p) (R4 + 2 * (p))
#define PH(p) (R5 + 2 * (p))
static int t_free_pair(const tj_regs *g, int keep_w)
{
	for (int p = 0; p < 3; p++)
		if (p != g->r && (!keep_w || p != g->w))
			return p;
	return 0;
}

// macc_read's output (macc_to_output: lo the first overflow test's low bit, 0 none; shift; rounding; rmask's low
// word; sat) as (m >> 24) & 0xffffff in r0
static void t_mout(const tj_regs *g, int lo, int shift, uint32_t round, uint32_t rmask_lo, int sat)
{
	const int ml = PL(g->r), mh = PH(g->r);
	int l = ml, h = mh;
	if (lo) {   // r12 != 0: bits lo..51 not all the same
		T_SHIFT(R0, mh, SH_LSL, 12);
		T_SHIFT(R1, R0, SH_ASR, 31);
		t_dp(C_AL, DP_SUB, 0, R12, R1, R0, SH_ASR, lo - 20);
	}
	if (shift > 0) {
		T_SHIFT(R3, mh, SH_LSL, shift);
		t_dp(C_AL, DP_ORR, 0, R3, R3, ml, SH_LSR, 32 - shift);
		T_SHIFT(R2, ml, SH_LSL, shift);
		l = R2; h = R3;
	} else if (shift < 0) {
		T_SHIFT(R2, ml, SH_LSR, -shift);
		t_dp(C_AL, DP_ORR, 0, R2, R2, mh, SH_LSL, 32 + shift);
		T_SHIFT(R3, mh, SH_ASR, -shift);
		l = R2; h = R3;
	}
	if (round) {
		t_dpi(C_AL, DP_ADD, 1, R2, l, round);
		t_dpi(C_AL, DP_ADC, 0, R3, h, 0);
		l = R2; h = R3;
	}
	if (rmask_lo != 0xffffffffu) {
		int k = 0;
		while (k < 32 && !(rmask_lo >> k & 1))
			k++;
		if (k >= 32)
			t_dpi(C_AL, DP_MOV, 0, R2, 0, 0);
		else if (k > 0) {
			T_SHIFT(R2, l, SH_LSR, k);
			T_SHIFT(R2, R2, SH_LSL, k);
		} else if (l != R2)
			T_MOV(R2, l);
		if (h != R3)
			T_MOV(R3, h);
		l = R2; h = R3;
	}
	// the second test (bits 47..51) and the value
	T_SHIFT(R0, h, SH_LSL, 12);
	T_SHIFT(R1, R0, SH_ASR, 31);
	t_dp(C_AL, DP_SUB, 1, R1, R1, R0, SH_ASR, 27);
	if (lo)
		t_dp(C_AL, DP_ORR, 1, R1, R1, R12, SH_LSL, 0);
	T_SHIFT(R0, l, SH_LSR, 24);
	t_dp(C_AL, DP_ORR, 0, R0, R0, h, SH_LSL, 8);
	t_dpi(C_AL, DP_BIC, 0, R0, R0, 0xff000000u);
	if (h != R3)
		T_MOV(R3, h);   // (the stub's sign)
	t_stub(ST_MOUT, C_NE, 0, sat, 0x800000, 0x7fffff);
}

// check_macc_overflow(lo, sat)'s value: out 0 (m >> 24) & 0xffff00, 1 (m >> 8) & 0xffffff, in r0
static void t_mchk(const tj_regs *g, int lo, int sat, int out)
{
	const int ml = PL(g->r), mh = PH(g->r);
	if (lo) {
		T_SHIFT(R0, mh, SH_LSL, 12);
		T_SHIFT(R1, R0, SH_ASR, 31);
		t_dp(C_AL, DP_SUB, 1, R1, R1, R0, SH_ASR, lo - 20);
	}
	if (out == 0) {
		T_SHIFT(R0, mh, SH_LSL, 16);
		T_SHIFT(R0, R0, SH_LSR, 8);
	} else
		t_ubfx(R0, ml, 8, 24);
	if (lo) {
		if (mh != R3)
			T_MOV(R3, mh);
		const int64_t sn = (int64_t)0xffff800000000000ULL, sp = 0x00007fffffffffffLL;
		const uint32_t vneg = out == 0 ? (uint32_t)((sn >> 24) & 0xffff00) : (uint32_t)((sn >> 8) & 0xffffff);
		const uint32_t vpos = out == 0 ? (uint32_t)((sp >> 24) & 0xffff00) : (uint32_t)((sp >> 8) & 0xffffff);
		t_stub(ST_MOUT, C_NE, 0, sat, vneg, vpos);
	}
}

// the product (c in r1, d in r2) >> 7 in r0 (lo), r3 (hi); unsigned: d unsigned
static void t_prod(int unsig)
{
	if (!unsig) {   // (d | 0xff000000 when its bit 23 is set)
		t_dpi(C_AL, DP_TST, 1, 0, R2, 0x800000);
		t_dpi(C_NE, DP_ORR, 0, R2, R2, 0xff000000u);
	}
	t_smull(R0, R3, R1, R2);
	if (unsig) {
		t_dpi(C_AL, DP_TST, 1, 0, R2, 0x80000000u);
		t_dp(C_NE, DP_ADD, 0, R3, R3, R1, SH_LSL, 0);
	}
	T_SHIFT(R0, R0, SH_LSR, 7);
	t_dp(C_AL, DP_ORR, 0, R0, R0, R3, SH_LSL, 25);
	T_SHIFT(R3, R3, SH_ASR, 7);
}

// a transfer's end, whole (TJ_XMDONE)
static void tj_xm_done_c(tms57002_t *s, uint32_t rd)
{
	s->sti = (s->sti & ~(uint32_t)(S_READ | S_WRITE)) | (rd ? S_READ : S_WRITE);
	xm_first(s);
	xm_last(s);
}
// a transfer in flight at the end, stepped `done` times: as the interpreter, done at its first step, then xm_sync
static void tj_xm_tail_c(tms57002_t *s, uint32_t done)
{
	const int n = xm_first(s);
	xm_sync(s, n - (int)done);
}

static int tj_emit(const tms57002_t *s, const tj_walk_t *w)
{
	const int fast_xm = (s->st0 & (ST0_WORD | ST0_SEL)) == (ST0_WORD | ST0_SEL);
	uint32_t mask = 0;
	switch (s->st0 & ST0_M) {
	case ST0_M_64K:  mask = 0x0ffff; break;
	case ST0_M_256K: mask = 0x3ffff; break;
	case ST0_M_1M:   mask = 0xfffff; break;
	}
	const int shift = (s->st0 & ST0_WORD ? 2 : 1) + (s->st0 & ST0_SEL ? 0 : 1);
	const int inline_xm = fast_xm && (mask & ~3u) + 2 < s->data_size;
	int creg_last = -1;   // (the last op writing creg: only it stores it, nothing reads it in a pass)
	for (int i = 0; i < w->n; i++)
		if (w->ir[i].kind == TJ_OP && ((w->ir[i].op >= 911 && w->ir[i].op <= 947) || w->ir[i].op == 1003))
			creg_last = i;

	s_tj_n = 0;
	s_tj_fail = 0;
	s_tj_nstub = 0;
	tw(0xe92d5ff0u);                            // push {r4-r12, lr}
	t_dpi(C_AL, DP_ADD, 0, R11, R0, (uint32_t)offsetof(tms57002_t, dmem0));
	T_LDRB(R10, TO(ba0));
	tj_regs g = { 0, 1, 2 };
	T_LDR(PL(0), TO(macc)); T_LDR(PH(0), TO(macc) + 4);
	T_LDR(PL(1), TO(macc_write)); T_LDR(PH(1), TO(macc_write) + 4);
	T_LDR(PL(2), TO(macc_read)); T_LDR(PH(2), TO(macc_read) + 4);

	for (int i = 0; i < w->n && !s_tj_fail; i++) {
		const tj_ir *e = &w->ir[i];
		if (e->kind == TJ_ISTART) {
			g.r = g.w;
			g.w = g.m;
			continue;
		}
		if (e->kind == TJ_XMDONE) {
			if (!inline_xm) {
				t_dpi(C_AL, DP_SUB, 0, R0, R11, (uint32_t)offsetof(tms57002_t, dmem0));
				t_dpi(C_AL, DP_MOV, 0, R1, 0, e->param);
				t_call((const void *)tj_xm_done_c);
			} else {
				T_LDR(R0, TO(xm_adr));
				t_mov32c(C_AL, R1, (uint32_t)(uintptr_t)s->data);
				t_dp(C_AL, DP_ADD, 0, R0, R1, R0, SH_LSL, 0);
				if (e->param) {   // read: (txrd & 0xff000000) | the 3 bytes
					t_mem(1, 1, R1, R0, 0);
					t_mem(1, 1, R2, R0, 1);
					t_mem(1, 1, R3, R0, 2);
					T_LDR(R12, TO(txrd));
					t_dpi(C_AL, DP_AND, 0, R12, R12, 0xff000000u);
					t_dp(C_AL, DP_ORR, 0, R12, R12, R1, SH_LSL, 16);
					t_dp(C_AL, DP_ORR, 0, R12, R12, R2, SH_LSL, 8);
					t_dp(C_AL, DP_ORR, 0, R12, R12, R3, SH_LSL, 0);
					T_STR(R12, TO(txrd));
					T_STR(R12, TO(xrd));
				} else {
					T_LDR(R1, TO(xwr));
					T_SHIFT(R2, R1, SH_LSR, 16);
					t_mem(0, 1, R2, R0, 0);
					T_SHIFT(R2, R1, SH_LSR, 8);
					t_mem(0, 1, R2, R0, 1);
					t_mem(0, 1, R1, R0, 2);
				}
			}
			continue;
		}
		const int op = e->op, p = e->param, ca = e->ca, id = e->id;
		switch (op) {
		case 436: case 437:   // sub: aacc = c - aacc (overflow: AOV, AOVM saturates)
			t_cmem(op == 436 ? p : ca, -1);
			T_LDR(R2, TO(aacc));
			t_dp(C_AL, DP_SUB, 1, R0, R1, R2, SH_LSL, 0);
			t_stub(ST_SUB, C_VS, 0, 0, 0, 0);
			T_STR(R0, TO(aacc));
			break;
		case 849:   // lacd: aacc = dmem0[id] << 8
			t_dload(R0, id);
			T_SHIFT(R0, R0, SH_LSL, 8);
			T_STR(R0, TO(aacc));
			break;
		case 856: case 857:   // lacc
			t_cmem(op == 856 ? p : ca, -1);
			T_STR(R1, TO(aacc));
			break;
		case 866: case 893:   // and (cmode 0), or (cmode 1)
			t_cmem(op == 866 ? p : ca, -1);
			T_LDR(R0, TO(aacc));
			t_dp(C_AL, op == 866 ? DP_AND : DP_ORR, 0, R0, R0, R1, SH_LSL, 0);
			T_STR(R0, TO(aacc));
			break;
		case 911: case 923: case 931: case 939: case 947: case 1003: {   // mpy, mac (sfma 0-3), mpyu
			t_cmem(ca, p);
			if (i == creg_last)
				T_STR(R1, TO(creg));
			t_prod(op == 1003);
			const int o = g.m, n = t_free_pair(&g, 1);
			if (op == 911 || op == 1003) {
				T_MOV(PL(n), R0);
				T_MOV(PH(n), R3);
			} else if (op == 923) {
				t_dp(C_AL, DP_ADD, 1, PL(n), PL(o), R0, SH_LSL, 0);
				t_dp(C_AL, DP_ADC, 0, PH(n), PH(o), R3, SH_LSL, 0);
			} else if (op == 931 || op == 939) {   // (macc << 2 / 4) + r
				const int k = op == 931 ? 2 : 4;
				T_SHIFT(R12, PH(o), SH_LSL, k);
				t_dp(C_AL, DP_ORR, 0, R12, R12, PL(o), SH_LSR, 32 - k);
				t_dp(C_AL, DP_ADD, 1, PL(n), R0, PL(o), SH_LSL, k);
				t_dp(C_AL, DP_ADC, 0, PH(n), R3, R12, SH_LSL, 0);
			} else {   // (macc >> 16) + r
				T_SHIFT(R12, PL(o), SH_LSR, 16);
				t_dp(C_AL, DP_ORR, 0, R12, R12, PH(o), SH_LSL, 16);
				t_dp(C_AL, DP_ADD, 1, PL(n), R0, R12, SH_LSL, 0);
				t_dp(C_AL, DP_ADC, 0, PH(n), R3, PH(o), SH_ASR, 16);
			}
			g.m = n;
			break;
		}
		case 1090: {   // lmhd: macc_write = macc = (int64)(int32)(dmem0[p] << 8) << 16
			t_dload(R0, p);
			T_SHIFT(R0, R0, SH_LSL, 8);
			const int n = t_free_pair(&g, 0);
			T_SHIFT(PH(n), R0, SH_ASR, 16);
			T_SHIFT(PL(n), R0, SH_LSL, 16);
			g.m = g.w = n;
			break;
		}
		case 1103: case 1104: case 1110: case 1111:   // wre, rde (not busy: the walk drops the others)
			if (op == 1103 || op == 1104) {
				t_dload(R0, op == 1103 ? p : id);
				T_STR(R0, TO(xwr));
			}
			t_cmem(op == 1103 || op == 1111 ? ca : p, -1);
			T_STR(R1, TO(xoa));
			T_LDR(R0, TO(xba));
			t_dp(C_AL, DP_ADD, 0, R0, R0, R1, SH_LSL, 0);
			if (!mask)
				t_dpi(C_AL, DP_MOV, 0, R0, 0, 0);
			else {
				int bits = 0;
				while (mask >> bits & 1)
					bits++;
				T_SHIFT(R0, R0, SH_LSL, shift);
				t_ubfx(R0, R0, 0, bits);
			}
			T_STR(R0, TO(xm_adr));
			break;
		case 1117:   // sacd: dmem0[id] = aacc >> 8
			T_LDR(R0, TO(aacc));
			T_SHIFT(R0, R0, SH_LSR, 8);
			t_dstore(R0, id, R1);
			break;
		case 1236: case 1240: case 1268:   // smhd: dmem0[p] = macc_to_output(...) >> 24 & 0xffffff
			if (op == 1236)
				t_mout(&g, 47, 0, 0x800000, 0xff000000u, 1);
			else if (op == 1240)
				t_mout(&g, 45, 2, 0x800000, 0xff000000u, 1);
			else
				t_mout(&g, 47, 0, 0x80000000u, 0, 1);
			t_dstore(R0, p, R1);
			break;
		case 1380: case 1412:   // slmh, slml (sfmo 0, movm)
			t_mchk(&g, 47, 1, op == 1380 ? 0 : 1);
			t_dstore(R0, p, R1);
			break;
		case 1432:   // ref
			break;
		case 1433: case 1434:   // srbd
			T_LDR(R0, TO(xrd));
			t_dstore(R0, op == 1433 ? p : id, R1);
			break;
		case 1437: case 1441: case 1445: case 1449:   // dis
			T_LDR(R0, TO(si) + 4 * ((op - 1437) / 4));
			t_dstore(R0, p, R1);
			break;
		case 1489: case 1529: case 1569: case 1570: case 1609: case 1610: {   // domh: so[k] (rnd 4, movm)
			const int k = op == 1489 ? 0 : op == 1529 ? 1 : op <= 1570 ? 2 : 3;
			const int sf1 = op == 1570 || op == 1610;
			t_mout(&g, sf1 ? 45 : 47, sf1 ? 2 : 0, 0x80000000u, 0, 1);
			T_STR(R0, TO(so) + 4 * k);
			break;
		}
		case 1630: case 1631: case 1632: case 1633: case 1634: case 1635: case 1640: case 1642:
		case 1646: case 1648: case 1651:   // (static: st1's decoding bits, idle, ca, pc at the end)
			break;
		default:
			s_tj_fail = 1;
			break;
		}
	}

	// the end: the macc pairs, st1's decoding bits, sti's static bits, pc, ca, id, icount, the transfer
	T_STR(PL(g.m), TO(macc)); T_STR(PH(g.m), TO(macc) + 4);
	T_STR(PL(g.w), TO(macc_write)); T_STR(PH(g.w), TO(macc_write) + 4);
	T_STR(PL(g.r), TO(macc_read)); T_STR(PH(g.r), TO(macc_read) + 4);
	const uint32_t st1x = (s->st1 & ST1_CACHE) ^ w->st1c;
	if (st1x) {
		T_LDR(R0, TO(st1));
		t_mov32(R1, st1x);
		t_dp(C_AL, DP_EOR, 0, R0, R0, R1, SH_LSL, 0);
		T_STR(R0, TO(st1));
	}
	if (w->sti) {
		T_LDR(R0, TO(sti));
		t_dpi(C_AL, DP_ORR, 0, R0, R0, w->sti);
		T_STR(R0, TO(sti));
	}
	t_dpi(C_AL, DP_MOV, 0, R0, 0, w->pc); T_STRB(R0, TO(pc));
	t_dpi(C_AL, DP_MOV, 0, R0, 0, w->ca); T_STRB(R0, TO(ca));
	t_dpi(C_AL, DP_MOV, 0, R0, 0, w->id); T_STRB(R0, TO(id));
	t_mov32(R0, (uint32_t)w->icount); T_STR(R0, TO(icount));
	if (w->xm_used && !w->xm) {
		t_dpi(C_AL, DP_MOV, 0, R0, 0, 0);
		T_STR(R0, TO(xm_adr));
	} else if (w->xm > 0) {
		t_dpi(C_AL, DP_SUB, 0, R0, R11, (uint32_t)offsetof(tms57002_t, dmem0));
		t_dpi(C_AL, DP_MOV, 0, R1, 0, (uint32_t)(w->xm_n - w->xm));
		t_call((const void *)tj_xm_tail_c);
	}
	tw(0xe8bd9ff0u);                            // pop {r4-r12, pc}

	// the out of line parts
	for (int k = 0; k < s_tj_nstub && !s_tj_fail; k++) {
		const tj_stub *st = &s_tj_stub[k];
		t_patch(st->from, s_tj_n);
		switch (st->kind) {
		case ST_CMEM:   // r1 = get_cmem_slow(s, a), r2 / r3 kept
			tw(0xe92d000cu);                    // push {r2, r3}
			t_dpi(C_AL, DP_SUB, 0, R0, R11, (uint32_t)offsetof(tms57002_t, dmem0));
			t_dpi(C_AL, DP_MOV, 0, R1, 0, st->a);
			t_call((const void *)get_cmem_slow);
			T_MOV(R1, R0);
			tw(0xe8bd000cu);                    // pop {r2, r3}
			break;
		case ST_MOUT:   // overflow: st1 MOV; saturated: the value by the sign (r3 bit 19: macc bit 51)
			T_LDR(R1, TO(st1));
			t_dpi(C_AL, DP_ORR, 0, R1, R1, ST1_MOV);
			T_STR(R1, TO(st1));
			if (st->sat) {
				t_dpi(C_AL, DP_TST, 1, 0, R3, 0x80000);
				t_mov32c(C_NE, R0, st->vneg);
				t_mov32c(C_EQ, R0, st->vpos);
			}
			break;
		case ST_SUB:   // AOV; AOVM: the limit by the wrapped result's sign
			T_LDR(R1, TO(st1));
			t_dpi(C_AL, DP_ORR, 0, R1, R1, ST1_AOV);
			T_STR(R1, TO(st1));
			t_dpi(C_AL, DP_TST, 1, 0, R1, ST1_AOVM);
			t_dp(C_NE, DP_MOV, 0, R1, 0, R0, SH_ASR, 31);
			t_dpi(C_NE, DP_EOR, 0, R0, R1, 0x80000000u);
			break;
		}
		t_patch(t_b(C_AL), st->back);
	}
	return !s_tj_fail;
}

// ---- the compiled passes ----
// a pass compiled for the cache's generation, st1's decoding bits, st0 and the run's cycles
typedef struct {
	const tms57002_t *s;
	const uint8_t *data;
	uint32_t gen, key, st0;
	int32_t icount;
	int state;          // TV_*
	unsigned ver;       // (TV_WAIT: the stage's build)
	void (*code)(tms57002_t *s);
} tj_var;
enum { TV_EMPTY, TV_FAIL, TV_WAIT, TV_LIVE };
enum { TJ_VARS = 8 };
static tj_var s_tv[TJ_VARS];
static int s_tv_next;
enum { JS_NONE, JS_BUILDING, JS_READY, JS_COPYING };
static int s_tj_stage_state;
static unsigned s_tj_stage_len, s_tj_stage_ver, s_tj_ver;
static uint32_t *s_tj_vm;
static unsigned s_tj_vm_used;
static int s_tj_vm_none;
static void *volatile s_tj_live;
static volatile unsigned s_tj_live_ver;

static int tj_compile(tms57002_t *s, tj_var *v)
{
	static tj_walk_t w;
	if (!tj_walk(s, s->icount, &w)) {
#if !defined(__vita__)
		if (getenv("RBX_TMSWALK"))
			tj_walk_print(s, &w);
#endif
		return 0;
	}
#if !defined(__vita__)
	if (getenv("RBX_TMSWALK"))
		tj_walk_print(s, &w);
#endif
	if (!tj_emit(s, &w)) {
#if !defined(__vita__)
		if (getenv("RBX_TMSWALK"))
			fprintf(stderr, "TMSJIT not emitted (%u words)\n", s_tj_n);
#endif
		return 0;
	}
	s_tj_stage_len = s_tj_n * 4;
#if !defined(__vita__)
	if (getenv("RBX_TMSWALK")) {
		unsigned nb = 0;
		for (unsigned k = 0; k < s_tj_n; k++)
			nb += s_tj_stage[k] == 0xe12fff3cu;
		fprintf(stderr, "TMSJIT %u words, %d out of line parts, %u calls\n", s_tj_n, s_tj_nstub, nb);
	}
#endif
	(void)v;
	return 1;
}

// the run by the compiled pass when it can (1 = done)
static int tj_run(tms57002_t *s)
{
	if (s_tj_vm_none)
		return 0;
	const uint32_t key = s->st1 & ST1_CACHE;
	if (s->icount <= 0 || (s->sti & (S_IDLE | IN_PLOAD | S_READ | S_WRITE | S_BRANCH)) || s->pc || s->ca || s->id ||
			s->rptc || s->rptc_next)
		return 0;
	tj_var *v = NULL;
	for (int i = 0; i < TJ_VARS; i++) {
		tj_var *x = &s_tv[i];
		if (x->state != TV_EMPTY && x->s == s && x->gen == s->h.gen && x->key == key && x->st0 == s->st0 &&
				x->icount == s->icount && x->data == s->data) {
			v = x;
			break;
		}
	}
	if (!v) {
		// (not while a build waits to be put in place; the pc 0 decoding first: the interpreter's)
		for (int i = 0; i < TJ_VARS; i++)
			if (s_tv[i].state == TV_WAIT)
				return 0;
		int16_t pnode;
		if (get_hash(s, 0, s->st1, &pnode) < 0)
			return 0;
		int st = __atomic_load_n(&s_tj_stage_state, __ATOMIC_ACQUIRE);
		if (st != JS_NONE || !__atomic_compare_exchange_n(&s_tj_stage_state, &st, JS_BUILDING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
			return 0;
		v = &s_tv[s_tv_next];
		s_tv_next = (s_tv_next + 1) % TJ_VARS;
		v->s = s;
		v->data = s->data;
		v->gen = s->h.gen;
		v->key = key;
		v->st0 = s->st0;
		v->icount = s->icount;
		v->code = NULL;
		if (tj_compile(s, v)) {
			v->state = TV_WAIT;
			v->ver = s_tj_stage_ver = s_tj_ver = s_tj_ver + 1 + (s_tj_ver + 1 == 0);
			__atomic_store_n(&s_tj_stage_state, JS_READY, __ATOMIC_RELEASE);
		} else {
			v->state = TV_FAIL;
			__atomic_store_n(&s_tj_stage_state, JS_NONE, __ATOMIC_RELEASE);
		}
		return 0;
	}
	if (v->state == TV_WAIT) {
		if (__atomic_load_n(&s_tj_live_ver, __ATOMIC_ACQUIRE) != v->ver)
			return 0;
		v->code = (void (*)(tms57002_t *))(uintptr_t)s_tj_live;
		v->state = v->code ? TV_LIVE : TV_FAIL;
	}
	if (v->state != TV_LIVE)
		return 0;
	cm_set(s);   // (as execute_run's start)
	v->code(s);
	return 1;
}

void tms57002_jit_install(void)
{
	int st = JS_READY;
	if (!__atomic_compare_exchange_n(&s_tj_stage_state, &st, JS_COPYING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return;
	if (!s_tj_vm && !s_tj_vm_none) {
		s_tj_vm = (uint32_t *)rbx_jit_alloc(TJ_VM_SIZE);
		s_tj_vm_none = !s_tj_vm;
	}
	const unsigned len = (s_tj_stage_len + 63) & ~63u;
	void *code = NULL;
	// (the same code again, e.g. after a state load or the game again: the copy made before)
	enum { TJ_BLOBS = 256 };
	static struct { const uint32_t *p; unsigned len; uint32_t h; } blob[TJ_BLOBS];
	static int nblob;
	uint32_t h = 2166136261u;
	for (unsigned k = 0; k < s_tj_stage_len / 4; k++)
		h = (h ^ s_tj_stage[k]) * 16777619u;
	for (int k = 0; k < nblob && !code; k++)
		if (blob[k].h == h && blob[k].len == s_tj_stage_len && !memcmp(blob[k].p, s_tj_stage, s_tj_stage_len))
			code = (void *)(uintptr_t)blob[k].p;
	if (!code && s_tj_vm && s_tj_vm_used + len <= TJ_VM_SIZE) {
		uint32_t *dst = s_tj_vm + s_tj_vm_used / 4;
		s_tj_vm_used += len;
		rbx_jit_write_begin();
		memcpy(dst, s_tj_stage, s_tj_stage_len);
		rbx_jit_write_end(dst, s_tj_stage_len);
		code = dst;
		if (nblob < TJ_BLOBS) {
			blob[nblob].p = dst;
			blob[nblob].len = s_tj_stage_len;
			blob[nblob++].h = h;
		}
#if !defined(__vita__)
		if (getenv("RBX_TMSWALK"))
			fprintf(stderr, "TMSJIT installed at %p (%u bytes)\n", (void *)dst, s_tj_stage_len);
#endif
	}
	s_tj_live = code;   // (NULL: the memory used up, the pass interpreted)
	__atomic_store_n(&s_tj_live_ver, s_tj_stage_ver, __ATOMIC_RELEASE);
	__atomic_store_n(&s_tj_stage_state, JS_NONE, __ATOMIC_RELEASE);
}

#if !defined(__vita__)
static uint32_t tj_rand(void)
{
	static uint32_t x;
	if (!x) {
		const char *e = getenv("RBX_TMSSEED");
		x = e && atoi(e) ? (uint32_t)atoi(e) * 2654435761u | 1 : 0x12345678;
	}
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}
static uint32_t tj_rval(void)   // (24-bit values mostly, with large and full 32-bit ones)
{
	const uint32_t r = tj_rand(), k = tj_rand() & 7;
	return k < 4 ? r & 0xffffff : k < 6 ? (r & 1 ? 0x7fff00 | (r >> 24) : 0x800000 | (r >> 24)) : r;
}
static int64_t tj_rmacc(void)   // (around the overflow limits of the output shifts)
{
	const int sh = 40 + (int)(tj_rand() % 14);
	int64_t v = (int64_t)(((uint64_t)tj_rand() << 32 | tj_rand()) & (((uint64_t)1 << sh) - 1));
	if (tj_rand() & 1)
		v = -v;
	return v;
}
// RBX_TMSFUZZ: the state made random first (values near the limits: the overflow paths; cmem updates queued: get_cmem's
// slow path; AOVM), both runs from it, then the true state back and run
static void tj_fuzz(tms57002_t *s)
{
	for (int i = 0; i < 256; i++) {
		s->dmem0[i] = tj_rval();
		s->cmem[i] = tj_rand() & 3 ? tj_rval() : tj_rand();
	}
	for (int i = 0; i < 32; i++)
		s->dmem1[i] = tj_rval();
	s->macc = tj_rmacc();
	s->macc_read = tj_rmacc();
	s->macc_write = tj_rmacc();
	s->aacc = tj_rand();
	s->creg = tj_rand();
	s->xrd = tj_rand();
	s->txrd = tj_rand();
	s->xwr = tj_rand();
	s->xoa = tj_rand();
	s->xba = tj_rand() & 0x7ffff;
	s->ba0 = (uint8_t)tj_rand();
	s->ba1 = (uint8_t)tj_rand();
	for (int k = 0; k < 4; k++)
		s->si[k] = tj_rval();
	s->st1 = (s->st1 & ~(uint32_t)(ST1_AOVM | ST1_AOV | ST1_MOV)) | (tj_rand() & 1 ? ST1_AOVM : 0);
	if (tj_rand() & 1) {   // (an update queue: get_cmem's slow path)
		s->update_counter_tail = (uint8_t)(tj_rand() & 15);
		s->update_counter_head = (uint8_t)((s->update_counter_tail + 1 + (tj_rand() % 15)) & 15);
		for (int i = 0; i < 16; i++)
			s->update[i] = tj_rand();
		s->sa = (uint8_t)tj_rand();
		if (tj_rand() & 1)
			s->sti |= S_UPDATE;
		else
			s->sti &= ~(uint32_t)S_UPDATE;
	}
	for (uint32_t i = 0; i < s->data_size; i += 1 + (tj_rand() & 63))
		s->data[i] = (uint8_t)tj_rand();
}

// PC harness check (ARM build, RBX_TMSCHK=N): every Nth compiled run also runs the interpreter from the same state,
// the saved state, the data space and get_cmem's fast test compared; the interpreter's result is kept
static unsigned long long s_chk_n, s_chk_bad;
static int tj_check(tms57002_t *s, int fuzz)
{
	static tms57002_t before, jres, orig;
	static uint8_t *bd, *jd, *od;
#define n s_chk_n
#define bad s_chk_bad
	const size_t a = offsetof(tms57002_t, macc), b = offsetof(tms57002_t, data);
	if (!bd) {
		bd = (uint8_t *)malloc(s->data_size);
		jd = (uint8_t *)malloc(s->data_size);
		od = (uint8_t *)malloc(s->data_size);
	}
	if (fuzz) {
		memcpy(&orig, s, sizeof(orig));
		memcpy(od, s->data, s->data_size);
		tj_fuzz(s);
	}
	memcpy(&before, s, sizeof(before));
	memcpy(bd, s->data, s->data_size);
	if (!tj_run(s)) {
		if (fuzz) {
			memcpy(s, &orig, sizeof(orig));
			memcpy(s->data, od, s->data_size);
		}
		return 0;
	}
	memcpy(&jres, s, sizeof(jres));
	memcpy(jd, s->data, s->data_size);
	memcpy((uint8_t *)s + a, (uint8_t *)&before + a, b - a);
	memcpy(&s->h, &before.h, sizeof(s->h));
	memcpy(s->data, bd, s->data_size);
	execute_run(s);
	n++;
	if (memcmp((uint8_t *)s + a, (uint8_t *)&jres + a, b - a) || memcmp(s->data, jd, s->data_size) ||
			memcmp(s->h.cm_slow, jres.h.cm_slow, sizeof(s->h.cm_slow)) || s->h.cm_key != jres.h.cm_key) {
		if (bad++ < 8) {
			const uint8_t *p = (const uint8_t *)s, *q = (const uint8_t *)&jres;
			size_t i = a;
			while (i < b && p[i] == q[i])
				i++;
			fprintf(stderr, "TMSCHK check %llu differs at state byte %u (macc at %u, cmem %u, dmem0 %u, dmem1 %u, si %u, "
				"st0 %u, pc %u, update %u, icount %u, cache %u)%s\n", n, (unsigned)i, (unsigned)a,
				(unsigned)offsetof(tms57002_t, cmem), (unsigned)offsetof(tms57002_t, dmem0),
				(unsigned)offsetof(tms57002_t, dmem1), (unsigned)offsetof(tms57002_t, si),
				(unsigned)offsetof(tms57002_t, st0), (unsigned)offsetof(tms57002_t, pc),
				(unsigned)offsetof(tms57002_t, update), (unsigned)offsetof(tms57002_t, icount),
				(unsigned)offsetof(tms57002_t, cache), i == b ? " (data or cm)" : "");
			if (i < b && i >= offsetof(tms57002_t, dmem0) && i < offsetof(tms57002_t, dmem1))
				fprintf(stderr, "  dmem0[%u] interp %06x jit %06x\n", (unsigned)(i - offsetof(tms57002_t, dmem0)) / 4,
					s->dmem0[(i - offsetof(tms57002_t, dmem0)) / 4], jres.dmem0[(i - offsetof(tms57002_t, dmem0)) / 4]);
		}
	} else if (n % 10000 == 0)
		fprintf(stderr, "TMSCHK %llu checks%s, %llu differ\n", n, fuzz ? " (random states)" : "", bad);
	if (fuzz) {   // (the true state's run)
		memcpy(s, &orig, sizeof(orig));
		memcpy(s->data, od, s->data_size);
		if (!tj_run(s))
			execute_run(s);
	}
	return 1;
#undef n
#undef bad
}

// RBX_TMSSYN=N (ARM harness): N random programs of the ops handled here (rejection sampled from random instruction
// words, decoded for random st0 / st1 modes; with lcak, b, ends by idle or by the cycles), each run from random
// states by the compiled code and the interpreter, compared; then the process exits
static uint32_t tj_syn_word(tms57002_t *s, uint32_t *st1v, int len)
{
	static const uint16_t T[] = { 436, 437, 849, 856, 857, 866, 893, 911, 923, 931, 939, 947, 1003, 1090, 1103, 1104, 1110,
		1111, 1117, 1236, 1240, 1268, 1380, 1412, 1432, 1433, 1434, 1437, 1441, 1445, 1449, 1489, 1529, 1569, 1570, 1609,
		1610, 1630, 1631, 1632, 1633, 1634, 1635, 1640, 1642 };
	const int target = T[tj_rand() % (sizeof(T) / sizeof(T[0]))];   // (an op wanted, when the modes allow it)
	const uint32_t k = tj_rand() % 64;   // (or lcak, b, idle)
	for (int tries = 0; tries < 30000; tries++) {
		uint32_t w = tj_rand() & 0xffffff;
		if (k < 2)
			w = 0xfcc000 | (tj_rand() & 0xff);                       // lcak
		else if (k < 3)
			w = 0xfc0000 | 0x48 << 11 | (tj_rand() % (uint32_t)len); // b
		else if (k < 4 && !(tj_rand() % 4))
			w = 0xfc4000;                                            // idle
		uint16_t o[3] = { 0, 0, 0 };
		cstate cs;
		memset(&cs, 0, sizeof(cs));
		s->st1 = *st1v;
		if ((w & 0xfc0000) == 0xfc0000)
			decode_cat3(s, w, &o[0], &cs);
		else {
			decode_cat2_pre(s, w, &o[0], &cs);
			decode_cat1(s, w, &o[1], &cs);
			decode_cat2_post(s, w, &o[2], &cs);
		}
		if ((!o[0] || tj_op_ok(o[0])) && (!o[1] || tj_op_ok(o[1])) && (!o[2] || tj_op_ok(o[2])) &&
				(k < 4 || tries >= 20000 || o[0] == target || o[1] == target || o[2] == target)) {
			*st1v = s->st1;
			return w;
		}
	}
	s->st1 = *st1v;
	return 0;
}

static void tj_syn_test(int progs)
{
	static tms57002_t t;
	static uint8_t dat[0x40000];
	unsigned long long runs = 0, notc = 0;
	for (int p = 0; p < progs; p++) {
		tms57002_init(&t, dat, sizeof(dat));
		static const uint32_t M[4] = { ST0_M_64K, ST0_M_256K, ST0_M_1M, ST0_M };
		t.st0 = M[tj_rand() & 3] | (tj_rand() & 1 ? ST0_WORD : 0) | (tj_rand() & 1 ? ST0_SEL : 0);
		if (tj_rand() & 1)
			t.st0 = ST0_M_256K | ST0_WORD | ST0_SEL;   // (the games' mode)
		uint32_t key = (tj_rand() & 3) << ST1_SFMA_SHIFT | (tj_rand() & 3) << ST1_SFMO_SHIFT | (tj_rand() % 5) << ST1_RND_SHIFT |
			(tj_rand() & 1 ? ST1_MOVM : 0) | (tj_rand() % 8 == 0 ? (1 + tj_rand() % 2) << ST1_CRM_SHIFT : 0);
		if (tj_rand() & 1)   // (the games' modes: their output ops)
			key = (tj_rand() & 3) << ST1_SFMA_SHIFT | (tj_rand() & 1) << ST1_SFMO_SHIFT | (2 + 2 * (tj_rand() & 1)) << ST1_RND_SHIFT |
				ST1_MOVM;
		const int len = 4 + (int)(tj_rand() % 160);
		uint32_t cur = key;
		for (int i = 0; i < len - 1; i++)
			t.program[i] = tj_syn_word(&t, &cur, len);
		t.program[len - 1] = 0xfc4000;
		const int32_t cycles = len + (tj_rand() % 3 ? 0 : (int32_t)(tj_rand() % 64)) - (tj_rand() % 4 ? 0 : (int32_t)(tj_rand() % 4));
		int ran = 0;
		for (int r = 0; r < 64; r++) {
			t.pc = t.ca = t.id = 0;
			t.sti &= ~(uint32_t)(S_IDLE | S_READ | S_WRITE | S_BRANCH | IN_PLOAD | IN_CLOAD);
			t.rptc = t.rptc_next = 0;
			t.icount = cycles > 0 ? cycles : 1;
			t.st1 = key;
			if (tj_check(&t, 1)) {
				ran++;
				runs++;
				continue;
			}
			t.pc = t.ca = t.id = 0;
			t.sti &= ~(uint32_t)(S_IDLE | S_READ | S_WRITE | S_BRANCH);
			t.icount = cycles > 0 ? cycles : 1;
			t.st1 = key;
			execute_run(&t);   // (the decoding, as the interpreter's first run)
			tms57002_jit_install();
		}
		static unsigned hist[OP_COUNT];
		if (ran) {   // (the ops covered)
			static tj_walk_t w;
			t.pc = t.ca = t.id = 0;
			t.sti &= ~(uint32_t)(S_IDLE | S_READ | S_WRITE | S_BRANCH);
			t.st1 = key;
			if (tj_walk(&t, cycles > 0 ? cycles : 1, &w))
				for (int i = 0; i < w.n; i++)
					if (w.ir[i].kind == TJ_OP)
						hist[w.ir[i].op]++;
		}
		if (p == progs - 1) {
			fprintf(stderr, "TMSSYN ops:");
			for (int op = 4; op < OP_COUNT; op++)
				if (tj_op_ok(op))
					fprintf(stderr, " %d:%u", op, hist[op]);
			fprintf(stderr, "\n");
		}
		if (!ran) {
			notc++;
			if (notc <= 10) {
				static tj_walk_t w;
				t.pc = t.ca = t.id = 0;
				t.st1 = key;
				tj_walk(&t, cycles > 0 ? cycles : 1, &w);
				fprintf(stderr, "TMSSYN program %d (%d words) not compiled: %s\n", p, len, w.why ? w.why : "?");
			}
		}
	}
	fprintf(stderr, "TMSSYN seed %s, %d programs (%llu not compiled), %llu runs, %llu checks, %llu differ\n",
		getenv("RBX_TMSSEED") ? getenv("RBX_TMSSEED") : "-", progs, notc, runs,
		s_chk_n, s_chk_bad);
	exit(s_chk_bad ? 1 : 0);
}
#endif

static int tms_jit_run(tms57002_t *s)
{
#if !defined(__vita__)
	static int chk = -1, fuzz;
	static unsigned cnt;
	if (chk < 0) {
		const char *c = getenv("RBX_TMSCHK"), *f = getenv("RBX_TMSFUZZ");
		chk = c ? atoi(c) : f ? atoi(f) : 0;
		fuzz = f != NULL;
	}
	if (chk > 0 && ++cnt % (unsigned)chk == 0)
		return tj_check(s, fuzz);
#endif
	return tj_run(s);
}
#else
void tms57002_jit_install(void) {}
static int tms_jit_run(tms57002_t *s)
{
	// (the PC harness: RBX_TMSWALK=1 prints the walks as the ARM build would compile them, one per generation, key,
	// st0 and cycles; RBX_TMSWALK=2 also counts the runs that would not be compiled)
	static int on = -1;
	static uint32_t lgen, lkey, lst0;
	static int32_t licount;
	static unsigned long long n, no;
	if (on < 0)
		on = getenv("RBX_TMSWALK") ? atoi(getenv("RBX_TMSWALK")) : 0;
	if (!on)
		return 0;
	n++;
	if (s->icount > 0 && !(s->sti & (S_IDLE | IN_PLOAD | S_READ | S_WRITE | S_BRANCH)) && !s->pc && !s->ca && !s->id &&
			!s->rptc && !s->rptc_next) {
		int16_t pnode;
		if (get_hash(s, 0, s->st1, &pnode) >= 0 && (s->h.gen != lgen || (s->st1 & ST1_CACHE) != lkey ||
				s->st0 != lst0 || s->icount != licount)) {
			static tj_walk_t w;
			tj_walk(s, s->icount, &w);
			tj_walk_print(s, &w);
			lgen = s->h.gen;
			lkey = s->st1 & ST1_CACHE;
			lst0 = s->st0;
			licount = s->icount;
		}
	} else if (!(s->sti & (S_IDLE | IN_PLOAD)) && s->icount > 0) {
		if (on == 2 && no++ < 20)
			fprintf(stderr, "TMSWALK run %llu not a pass: sti %x pc %d ca %d id %d rptc %d/%d icount %d\n", n, s->sti, s->pc,
				s->ca, s->id, s->rptc, s->rptc_next, s->icount);
	}
	return 0;
}
#endif
