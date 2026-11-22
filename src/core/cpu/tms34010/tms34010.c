/*  Texas Instruments TMS34010, ported to C from MAME src/devices/cpu/tms34010/tms34010.cpp (BSD-3-Clause,
    copyright-holders: Alex Pasadyn, Zsolt Vasvari; parts based on code by Aaron Giles) and the screen timing of
    src/emu/screen.cpp (BSD-3-Clause, copyright-holders: Aaron Giles) it drives.
    tms34010_ops.inc / tms34010_gfx.inc / tms34010_proto.inc are MAME's 34010ops.h, 34010fld.hxx, 34010tbl.hxx,
    34010ops.hxx and 34010gfx.hxx put into C by a script (member functions to static functions, the 34020 parts out).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tms34010.h"
#include "../../rbx.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif

typedef uint32_t offs_t;
typedef uint16_t u16;
typedef struct { int16_t x, y; } XY;   // (little-endian hosts: the low half of the register is x)

enum {
	REG_HESYNC = 0, REG_HEBLNK, REG_HSBLNK, REG_HTOTAL, REG_VESYNC, REG_VEBLNK, REG_VSBLNK, REG_VTOTAL,
	REG_DPYCTL, REG_DPYSTRT, REG_DPYINT, REG_CONTROL, REG_HSTDATA, REG_HSTADRL, REG_HSTADRH, REG_HSTCTLL,
	REG_HSTCTLH, REG_INTENB, REG_INTPEND, REG_CONVSP, REG_CONVDP, REG_PSIZE, REG_PMASK, REG_UNK23,
	REG_UNK24, REG_UNK25, REG_UNK26, REG_DPYTAP, REG_HCOUNT, REG_VCOUNT, REG_DPYADR, REG_REFCNT
};
// (the 34020's register numbers: SMART_IOREG names them, m_is_34020 is 0)
enum {
	REG020_VESYNC, REG020_HESYNC, REG020_VEBLNK, REG020_HEBLNK, REG020_VSBLNK, REG020_HSBLNK, REG020_VTOTAL,
	REG020_HTOTAL, REG020_DPYCTL, REG020_DPYSTRT, REG020_DPYINT, REG020_CONTROL, REG020_HSTDATA, REG020_HSTADRL,
	REG020_HSTADRH, REG020_HSTCTLL, REG020_HSTCTLH, REG020_INTENB, REG020_INTPEND, REG020_CONVSP, REG020_CONVDP,
	REG020_PSIZE, REG020_PMASKL, REG020_PMASKH, REG020_CONVMP, REG020_CONTROL2, REG020_CONFIG, REG020_DPYTAP,
	REG020_VCOUNT, REG020_HCOUNT, REG020_DPYADR, REG020_REFADR, REG020_DPYSTL, REG020_DPYSTH, REG020_DPYNXL,
	REG020_DPYNXH, REG020_DINCL, REG020_DINCH
};

#define TMS34010_INT1 0x0002
#define TMS34010_INT2 0x0004
#define TMS34010_HI   0x0200
#define TMS34010_DI   0x0400
#define TMS34010_WV   0x0800

#define TMS_LOGERR(...) ((void)0)
#define fatalerror(...) ((void)0)
#define osd_printf_debug(...) ((void)0)
static inline int64_t mul_32x32(int32_t a, int32_t b) { return (int64_t)a * (int64_t)b; }
static inline uint64_t mulu_32x32(uint32_t a, uint32_t b) { return (uint64_t)a * (uint64_t)b; }
#define TMS_SEXT(v, n) ((int32_t)((uint32_t)(v) << (32 - (n))) >> (32 - (n)))
#define standard_irq_callback(l, pc) ((void)0)

typedef void (*pixel_write_func)(offs_t offset, uint32_t data);
typedef uint32_t (*pixel_read_func)(offs_t offset);
typedef uint32_t (*raster_op_func)(uint32_t newpix, uint32_t oldpix);
typedef void (*wfield_func)(offs_t offset, uint32_t data);
typedef uint32_t (*rfield_func)(offs_t offset);
typedef uint32_t (*pixel_op_func)(uint32_t, uint32_t, uint32_t);
typedef void (*pixblt_op_func)(int, int);
typedef void (*pixblt_b_op_func)(int);
typedef void (*word_write_func)(offs_t offset, uint16_t data);
typedef uint16_t (*word_read_func)(offs_t offset);
typedef void (*opcode_func)(uint16_t op);

// ---- state (MAME's members) ----
static uint32_t m_pc, m_ppc, m_st;
static pixel_write_func m_pixel_write;
static pixel_read_func m_pixel_read;
static raster_op_func m_raster_op;
static pixel_op_func m_pixel_op;
static uint32_t m_pixel_op_timing;
static uint32_t m_convsp, m_convdp, m_convmp;
static int32_t m_gfxcycles;
static uint8_t m_pixelshift;
static const bool m_is_34020 = false;
static bool m_reset_deferred, m_halt_on_reset;
static uint8_t m_hblank_stable, m_external_host_access, m_executing;
static int m_icount;
static union { int32_t reg; XY xy; } m_regs[31];
static uint16_t m_IOregs[64];
static uint16_t m_shiftreg[(8 * 512 * sizeof(uint16_t)) / 2];
static void (*m_to_shiftreg_cb)(uint32_t address, uint16_t *shiftreg);
static void (*m_from_shiftreg_cb)(uint32_t address, uint16_t *shiftreg);

static tms34010_config s_cfg;
static uint16_t *s_rpage[4096], *s_wpage[4096];
static uint64_t s_total;       // cycles of the finished tms34010_execute calls
static int s_req, s_stolen;    // the running call's request, cycles taken away by tms34010_abort

static void io_register_w(offs_t offset, u16 data, u16 mem_mask);
static u16 io_register_r(offs_t offset);
static void check_interrupt(void);

// the polling loop skip (not in MAME, exact: see spin_back): while a loop's iteration is being checked (s_probe),
// anything that is not a pure read or a write that leaves the memory as it was marks it (s_dirty)
static uint8_t s_probe, s_dirty, s_unmapped;
static int s_spin_on = -1;   // (-1: RBX_TMS_NOSPIN not read yet)
static void spin_init(void);
// (a loop found not to be one: not checked again for 2-256 arrivals, more after each failure)
static struct { uint32_t head; uint16_t wait; uint8_t level; } s_spin_bad[256];

// ---- the bus (MAME memory_access<32, 1, 3, LITTLE>: bit addresses, 16-bit words) ----
static uint16_t slow_read(offs_t a)
{
	if ((a & 0xfffffe00) == 0xc0000000) {
		s_dirty |= s_probe;
		return io_register_r((a >> 4) & 0x1f);
	}
	if (s_probe && !(s_cfg.pure_read && s_cfg.pure_read(a & 0xfffffff0)))
		s_dirty = 1;
	return s_cfg.read(a & 0xfffffff0);
}
static void slow_write(offs_t a, uint16_t v)
{
	if ((a & 0xfffffe00) == 0xc0000000) {
		s_dirty |= s_probe;
		io_register_w((a >> 4) & 0x1f, v, 0xffff);
		return;
	}
	if (s_probe) {
		s_unmapped = 0;
		s_cfg.write(a & 0xfffffff0, v);
		if (!s_unmapped)
			s_dirty = 1;
		return;
	}
	s_cfg.write(a & 0xfffffff0, v);
}
void tms34010_unmapped(void) { s_unmapped = 1; }
static inline uint32_t TMS34010_RDMEM_WORD(offs_t A)
{
	const uint16_t *p = s_rpage[A >> 20];
	return p ? p[(A & 0xfffff) >> 4] : slow_read(A);
}
static inline uint32_t TMS34010_RDMEM_DWORD(offs_t A)
{
	uint32_t result = TMS34010_RDMEM_WORD(A);
	return result | (TMS34010_RDMEM_WORD(A + 16) << 16);
}
static uint32_t s_watch_lo, s_watch_len = 0;   // (tms34010_write_watch: none)
static void (*s_watch_fn)(void);
static uint32_t s_wlo, s_wlen;   // (the writes to memory that take watched_write: the watch, all while checking a loop)
static void __attribute__((noinline)) watched_write(offs_t A, uint16_t *w, uint16_t V)
{
	if (s_probe && *w != V)
		s_dirty = 1;
	*w = V;
	if (A - s_watch_lo < s_watch_len) {
		s_dirty |= s_probe;
		s_watch_fn();
	}
}
static inline void TMS34010_WRMEM_WORD(offs_t A, uint32_t V)
{
	uint16_t *p = s_wpage[A >> 20];
	if (p) {
		uint16_t *w = &p[(A & 0xfffff) >> 4];
		if (__builtin_expect(A - s_wlo < s_wlen, 0))
			watched_write(A, w, (uint16_t)V);
		else
			*w = (uint16_t)V;
	} else
		slow_write(A, (uint16_t)V);
}
static inline void TMS34010_WRMEM_DWORD(offs_t A, uint32_t V)
{
	TMS34010_WRMEM_WORD(A, V);
	TMS34010_WRMEM_WORD(A + 16, V >> 16);
}
static inline uint16_t ROPCODE(void)
{
	uint32_t pc = m_pc;
	m_pc += 2 << 3;
	return (uint16_t)TMS34010_RDMEM_WORD(pc);
}
static inline int16_t PARAM_WORD(void)
{
	uint32_t pc = m_pc;
	m_pc += 2 << 3;
	return (int16_t)TMS34010_RDMEM_WORD(pc);
}
static inline int32_t PARAM_LONG(void)
{
	uint32_t pc = m_pc;
	m_pc += 4 << 3;
	return (int32_t)((uint16_t)TMS34010_RDMEM_WORD(pc) | (TMS34010_RDMEM_WORD(pc + 16) << 16));
}
static inline int16_t PARAM_WORD_NO_INC(void) { return (int16_t)TMS34010_RDMEM_WORD(m_pc); }
static inline int32_t PARAM_LONG_NO_INC(void)
{
	uint32_t pc = m_pc;
	return (int32_t)((uint16_t)TMS34010_RDMEM_WORD(pc) | (TMS34010_RDMEM_WORD(pc + 16) << 16));
}

static int s_mm_fast = 1;   // (MMFM / MMTM straight to a memory page; RBX_TMS_NOMMFAST=1: off)
#include "tms34010_proto.inc"
#include "tms34010_ops.inc"
#include "tms34010_jit.inc"

void tms34010_release(void) { tj_release(); }

// ---- reset / run (MAME device_reset, execute_set_input, execute_run) ----
void tms34010_init(const tms34010_config *cfg)
{
	s_cfg = *cfg;
	m_to_shiftreg_cb = cfg->to_shiftreg;
	m_from_shiftreg_cb = cfg->from_shiftreg;
	memset(s_rpage, 0, sizeof(s_rpage));
	memset(s_wpage, 0, sizeof(s_wpage));
	memset(s_tj_rom, 0, sizeof(s_tj_rom));
	tj_flush();
	m_halt_on_reset = false;
	s_watch_lo = s_watch_len = 0;
	s_watch_fn = NULL;
	s_wlo = s_wlen = 0;
	s_mm_fast = !(getenv("RBX_TMS_NOMMFAST") && atoi(getenv("RBX_TMS_NOMMFAST")));
	if (s_spin_on < 0)
		spin_init();
	memset(s_spin_bad, 0, sizeof(s_spin_bad));
}

void tms34010_write_watch(uint32_t start, uint32_t end, void (*fn)(void))
{
	s_watch_lo = start;
	s_watch_len = fn ? end - start + 1 : 0;
	s_watch_fn = fn;
	s_wlo = s_watch_lo;
	s_wlen = s_watch_len;
}

void tms34010_map(uint32_t start, uint32_t end, uint16_t *read, uint16_t *write)
{
	for (uint32_t p = start >> 20; p <= end >> 20; p++) {
		uint32_t o = ((p << 20) - start) >> 4;   // words from start
		s_rpage[p] = read ? read + o : NULL;
		s_wpage[p] = write ? write + o : NULL;
		s_tj_rom[p] = 0;
	}
	tj_flush();
}

static void screen_reset(void);

void tms34010_reset(void)
{
	tj_flush();
	m_ppc = 0;
	m_st = 0;
	m_pixel_write = NULL;
	m_pixel_read = NULL;
	m_raster_op = NULL;
	m_pixel_op = NULL;
	m_pixel_op_timing = 0;
	m_convsp = 0;
	m_convdp = 0;
	m_convmp = 0;
	m_gfxcycles = 0;
	m_pixelshift = 0;
	m_hblank_stable = 0;
	m_external_host_access = 0;
	m_executing = 0;
	m_icount = 0;
	memset(m_regs, 0, sizeof(m_regs));
	memset(m_IOregs, 0, sizeof(m_IOregs));
	memset(m_shiftreg, 0, sizeof(m_shiftreg));
	s_total = 0;
	s_req = s_stolen = 0;

	m_pc = RLONG(0xffffffe0) & 0xfffffff0;
	RESET_ST();
	m_reset_deferred = m_halt_on_reset;
	if (m_reset_deferred)
		io_register_w(REG_HSTCTLH, 0x8000, 0xffff);
	screen_reset();
}

void tms34010_set_irq(int line, int state)
{
	const uint16_t bit = line ? TMS34010_INT2 : TMS34010_INT1;
	if (state)
		IOREG(REG_INTPEND) |= bit;
	else
		IOREG(REG_INTPEND) &= ~bit;
}

// ---- the polling loop skip ----
// A backward transfer (a taken jump to pc <= its own) starts a check of the loop from its target: the registers and ST
// are kept, then the next time the same transfer is taken, the iteration between is the same forever if it ran only
// the instructions of s_spin_kind (no calls, stack, interrupts, graphics), read only memory or pure board addresses
// (tms34010_config.pure_read), wrote nothing that changed the memory (unmapped board addresses, the same value) and
// ended with the same registers and ST: nothing changes the memory or the CPU within tms34010_execute but the CPU. The
// whole iterations that fit in the slice but one are then counted as run (m_icount), the rest run as before.
// A delay loop (the same but one register one less, from a DSJ / DSJS, with only nop / dsj / movi / movk / move to
// an absolute address in the loop) is skipped the same way, the register lowered, at least two iterations left.
// RBX_TMS_NOSPIN=1: off.
static uint8_t s_spin_kind[4096];   // bit 0: may be in a polling loop, bit 1: may be in a delay loop
static uint8_t s_pkind;
static uint32_t s_phead, s_ptail, s_pst;
static int s_picount;
static int32_t s_pregs[31];
static uint64_t s_spin_skipped, s_spin_delay;

static void spin_stat(void)
{
	fprintf(stderr, "tms34010 spin: %llu polling, %llu delay iterations skipped\n", (unsigned long long)s_spin_skipped,
		(unsigned long long)s_spin_delay);
}

static void spin_init(void)
{
	static const opcode_func poll[] = {
		j_UC_0, j_UC_8, j_UC_x, j_P_0, j_P_8, j_P_x, j_LS_0, j_LS_8, j_LS_x, j_HI_0, j_HI_8, j_HI_x, j_LT_0, j_LT_8, j_LT_x,
		j_GE_0, j_GE_8, j_GE_x, j_LE_0, j_LE_8, j_LE_x, j_GT_0, j_GT_8, j_GT_x, j_C_0, j_C_8, j_C_x, j_NC_0, j_NC_8, j_NC_x,
		j_EQ_0, j_EQ_8, j_EQ_x, j_NE_0, j_NE_8, j_NE_x, j_V_0, j_V_8, j_V_x, j_NV_0, j_NV_8, j_NV_x, j_N_0, j_N_8, j_N_x,
		j_NN_0, j_NN_8, j_NN_x, jump_a, jump_b,
		add_xy_a, add_xy_b, sub_xy_a, sub_xy_b, cmp_xy_a, cmp_xy_b, movx_a, movx_b, movy_a, movy_b,
		abs_a, abs_b, add_a, add_b, addc_a, addc_b, addi_w_a, addi_w_b, addi_l_a, addi_l_b, addk_a, addk_b,
		and_a, and_b, andi_a, andi_b, andn_a, andn_b, btst_k_a, btst_k_b, btst_r_a, btst_r_b, clrc, setc,
		cmp_a, cmp_b, cmpi_w_a, cmpi_w_b, cmpi_l_a, cmpi_l_b, neg_a, neg_b, negb_a, negb_b, not_a, not_b,
		or_a, or_b, ori_a, ori_b, sext0_a, sext0_b, sext1_a, sext1_b, zext0_a, zext0_b, zext1_a, zext1_b,
		rl_k_a, rl_k_b, rl_r_a, rl_r_b, sla_k_a, sla_k_b, sla_r_a, sla_r_b, sll_k_a, sll_k_b, sll_r_a, sll_r_b,
		sra_k_a, sra_k_b, sra_r_a, sra_r_b, srl_k_a, srl_k_b, srl_r_a, srl_r_b, sub_a, sub_b, subb_a, subb_b,
		subi_w_a, subi_w_b, subi_l_a, subi_l_b, subk_a, subk_b, xor_a, xor_b, xori_a, xori_b,
		movb_rn_a, movb_rn_b, movb_nr_a, movb_nr_b, movb_nn_a, movb_nn_b, movb_r_no_a, movb_r_no_b,
		movb_no_r_a, movb_no_r_b, movb_no_no_a, movb_no_no_b, movb_ar_a, movb_ar_b, movb_aa,
		move_rr_a, move_rr_b, move_rr_ax, move_rr_bx,
		move0_rn_a, move0_rn_b, move1_rn_a, move1_rn_b, move0_r_dn_a, move0_r_dn_b, move1_r_dn_a, move1_r_dn_b,
		move0_r_ni_a, move0_r_ni_b, move1_r_ni_a, move1_r_ni_b, move0_nr_a, move0_nr_b, move1_nr_a, move1_nr_b,
		move0_dn_r_a, move0_dn_r_b, move1_dn_r_a, move1_dn_r_b, move0_ni_r_a, move0_ni_r_b, move1_ni_r_a, move1_ni_r_b,
		move0_nn_a, move0_nn_b, move1_nn_a, move1_nn_b, move0_dn_dn_a, move0_dn_dn_b, move1_dn_dn_a, move1_dn_dn_b,
		move0_ni_ni_a, move0_ni_ni_b, move1_ni_ni_a, move1_ni_ni_b, move0_r_no_a, move0_r_no_b, move1_r_no_a, move1_r_no_b,
		move0_no_r_a, move0_no_r_b, move1_no_r_a, move1_no_r_b, move0_no_ni_a, move0_no_ni_b, move1_no_ni_a, move1_no_ni_b,
		move0_no_no_a, move0_no_no_b, move1_no_no_a, move1_no_no_b, move0_ar_a, move0_ar_b, move1_ar_a, move1_ar_b,
		move0_a_ni_a, move0_a_ni_b, move1_a_ni_a, move1_a_ni_b, move0_aa, move1_aa };
	static const opcode_func delay[] = { nop, dsj_a, dsj_b, dsjeq_a, dsjeq_b, dsjne_a, dsjne_b, dsjs_a, dsjs_b,
		movi_w_a, movi_w_b, movi_l_a, movi_l_b, movk_a, movk_b, movb_ra_a, movb_ra_b, move0_ra_a, move0_ra_b,
		move1_ra_a, move1_ra_b };
	memset(s_spin_kind, 0, sizeof(s_spin_kind));
	for (int i = 0; i < 4096; i++) {
		for (size_t k = 0; k < sizeof(poll) / sizeof(poll[0]); k++)
			if (s_opcode_table[i] == poll[k])
				s_spin_kind[i] = 1;
		for (size_t k = 0; k < sizeof(delay) / sizeof(delay[0]); k++)
			if (s_opcode_table[i] == delay[k])
				s_spin_kind[i] = 3;
	}
	s_spin_on = !(getenv("RBX_TMS_NOSPIN") && atoi(getenv("RBX_TMS_NOSPIN")));
	if (getenv("RBX_TMS_SPINSTAT"))
		atexit(spin_stat);
}


static void spin_fail(uint32_t head)
{
	unsigned h = (head >> 4) & 255;
	if (s_spin_bad[h].head != head) {
		s_spin_bad[h].head = head;
		s_spin_bad[h].level = 0;
	}
	if (s_spin_bad[h].level < 8)
		s_spin_bad[h].level++;
	s_spin_bad[h].wait = (uint16_t)(1u << s_spin_bad[h].level);
}

static inline void spin_check(void)
{
	if (s_probe && (m_pc != s_phead || m_ppc != s_ptail))
		spin_fail(s_phead);   // (another loop within: the checked one is not a polling loop)
	if (s_probe && m_pc == s_phead && m_ppc == s_ptail) {
		int ok = !s_dirty && m_st == s_pst;
		int d = -1;   // (the one register one less)
		for (int i = 0; ok && i < 31; i++)
			if (m_regs[i].reg != s_pregs[i]) {
				if (d >= 0 || (uint32_t)m_regs[i].reg != (uint32_t)s_pregs[i] - 1u)
					ok = 0;
				d = i;
			}
		if (ok && !(s_pkind & (d < 0 ? 1 : 2)))
			ok = 0;
		if (!ok) {
			spin_fail(m_pc);
			s_probe = 0;
			return;
		}
		const int c = s_picount - m_icount;   // (the iteration's cycles)
		int n = c > 0 ? m_icount / c - 1 : 0;
		if (d < 0 && n > 0) {
			m_icount -= n * c;
			s_spin_skipped += (uint64_t)n;
		} else if (d >= 0 && n > 0 && (uint32_t)m_regs[d].reg >= 2u) {
			uint32_t r = (uint32_t)m_regs[d].reg;
			if ((uint32_t)n > r - 2u)
				n = (int)(r - 2u);
			if (n > 0) {
				m_icount -= n * c;
				m_regs[d].reg = (int32_t)(r - (uint32_t)n);
				s_spin_delay += (uint64_t)n;
			}
		}
	} else {
		unsigned h = (m_pc >> 4) & 255;
		if (s_spin_bad[h].head == m_pc && s_spin_bad[h].wait) {
			s_spin_bad[h].wait--;
			s_probe = 0;
			return;
		}
	}
	if (m_ppc - m_pc > 64 * 16) {   // (a loop of up to 64 words)
		s_probe = 0;
		return;
	}
	s_probe = 1;
	s_dirty = 0;
	s_pkind = 3;
	s_phead = m_pc;
	s_ptail = m_ppc;
	s_pst = m_st;
	s_picount = m_icount;
	memcpy(s_pregs, m_regs, sizeof(s_pregs));
}

// the run loop changes when a check starts or ends (the one with s_pkind while checking): m_icount is put aside
static int s_stash;
static uint8_t s_switch;
static void __attribute__((noinline)) spin_back(void)
{
	const uint8_t was = s_probe;
	spin_check();
	if (s_probe != was) {
		s_wlo = s_probe ? 0 : s_watch_lo;
		s_wlen = s_probe ? 0xffffffffu : s_watch_len;
		s_stash = m_icount;
		m_icount = 0;
		s_switch = 1;
	}
}

// the opcode table with the jumps (the instructions that close a loop) followed by the backward transfer test
#define SPIN_WRAP(f) static void sw_##f(uint16_t op) { f(op); if (m_pc <= m_ppc) spin_back(); }
#define SPIN_J(c) SPIN_WRAP(j_##c##_0) SPIN_WRAP(j_##c##_8) SPIN_WRAP(j_##c##_x)
SPIN_J(UC) SPIN_J(P) SPIN_J(LS) SPIN_J(HI) SPIN_J(LT) SPIN_J(GE) SPIN_J(LE) SPIN_J(GT) SPIN_J(C) SPIN_J(NC) SPIN_J(EQ)
SPIN_J(NE) SPIN_J(V) SPIN_J(NV) SPIN_J(N) SPIN_J(NN)
SPIN_WRAP(jump_a) SPIN_WRAP(jump_b) SPIN_WRAP(dsj_a) SPIN_WRAP(dsj_b) SPIN_WRAP(dsjeq_a) SPIN_WRAP(dsjeq_b)
SPIN_WRAP(dsjne_a) SPIN_WRAP(dsjne_b) SPIN_WRAP(dsjs_a) SPIN_WRAP(dsjs_b)
static opcode_func s_spin_table[4096];
static void spin_table(void)
{
#define SPIN_P(f) { f, sw_##f },
#define SPIN_PJ(c) SPIN_P(j_##c##_0) SPIN_P(j_##c##_8) SPIN_P(j_##c##_x)
	static const struct { opcode_func f, w; } pairs[] = {
		SPIN_PJ(UC) SPIN_PJ(P) SPIN_PJ(LS) SPIN_PJ(HI) SPIN_PJ(LT) SPIN_PJ(GE) SPIN_PJ(LE) SPIN_PJ(GT) SPIN_PJ(C)
		SPIN_PJ(NC) SPIN_PJ(EQ) SPIN_PJ(NE) SPIN_PJ(V) SPIN_PJ(NV) SPIN_PJ(N) SPIN_PJ(NN)
		SPIN_P(jump_a) SPIN_P(jump_b) SPIN_P(dsj_a) SPIN_P(dsj_b) SPIN_P(dsjeq_a) SPIN_P(dsjeq_b) SPIN_P(dsjne_a)
		SPIN_P(dsjne_b) SPIN_P(dsjs_a) SPIN_P(dsjs_b) };
	for (int i = 0; i < 4096; i++) {
		s_spin_table[i] = s_opcode_table[i];
		for (size_t k = 0; k < sizeof(pairs) / sizeof(pairs[0]); k++)
			if (s_opcode_table[i] == pairs[k].f)
				s_spin_table[i] = pairs[k].w;
	}
}

void tms34010_code_rom(uint32_t start, uint32_t end)
{
	for (uint32_t p = start >> 20; p <= end >> 20; p++)
		s_tj_rom[p] = s_rpage[p] && !s_wpage[p];
	tj_flush();
}

int tms34010_execute(int cycles)
{
	s_req = cycles;
	s_stolen = 0;
	m_icount = cycles;
	if (IOREG(REG_HSTCTLH) & 0x8000) {
		m_icount = 0;
		s_total += (uint64_t)cycles;
		return cycles;
	}
	if (m_reset_deferred) {
		m_reset_deferred = false;
		m_pc = RLONG(0xffffffe0);
	}
	m_executing = true;
	check_interrupt();
	if (s_spin_on > 0) {
		if (!s_spin_table[0])
			spin_table();
		s_probe = 0;
		for (;;) {
			if (s_probe)
				do {
					m_ppc = m_pc;
					uint16_t op = ROPCODE();
					s_pkind &= s_spin_kind[op >> 4];
					s_spin_table[op >> 4](op);
				} while (m_icount > 0);
			else if (tj_enabled())
				tj_run(s_spin_table);
			else
				do {
					m_ppc = m_pc;
					uint16_t op = ROPCODE();
					s_spin_table[op >> 4](op);
				} while (m_icount > 0);
			if (!s_switch)
				break;
			s_switch = 0;
			m_icount = s_stash;
			if (m_icount <= 0)
				break;
		}
		s_probe = 0;
		s_wlo = s_watch_lo;
		s_wlen = s_watch_len;
	} else if (tj_enabled())
		tj_run(s_opcode_table);
	else {
		do {
			m_ppc = m_pc;
			uint16_t op = ROPCODE();
			s_opcode_table[op >> 4](op);
		} while (m_icount > 0);
	}
	m_executing = false;
	int ran = cycles - m_icount - s_stolen;
	s_total += (uint64_t)(int64_t)ran;
	s_req = 0;
	m_icount = 0;
	s_stolen = 0;
	return ran;
}

void tms34010_abort(void)
{
	if (m_executing && m_icount > 0) {
		s_stolen += m_icount;
		m_icount = 0;
	}
}

void tms34010_eat(int cycles)
{
	if (m_executing)
		m_icount = cycles > m_icount ? 0 : m_icount - cycles;
}

int tms34010_executing(void) { return m_executing; }
uint32_t tms34010_pc(void) { return m_pc; }
uint64_t tms34010_total_cycles(void)
{
	return s_total + (uint64_t)(int64_t)(m_executing ? s_req - m_icount - s_stolen : 0);
}

// ---- the screen (MAME screen_device: raw configuration, configure(), vpos, partial updates) ----
tms34010_screen_t tms34010_screen;
static int s_cb_pending, s_cb_line, s_cb_param;   // the scanline timer: its line (-1: at once) and parameter
static int s_last_partial;
static uint32_t s_ticks_since_vblank;              // pixels since the last VBLANK start, at the current line's start
static uint64_t s_line_tick;                       // pixels since the reset at the current line's start
static uint16_t s_linebuf[2048];

static void screen_reset(void)
{
	tms34010_screen_t *s = &tms34010_screen;
	s->width = s_cfg.htotal;
	s->height = s_cfg.vtotal;
	s->min_x = s_cfg.hbend;
	s->max_x = s_cfg.hbstart - 1;
	s->min_y = s_cfg.vbend;
	s->max_y = s_cfg.vbstart - 1;
	s->vpos = s->max_y + 1;   // (MAME's screens start at the beginning of VBLANK)
	s_ticks_since_vblank = 0;
	s_cb_pending = 1;          // (device_start: m_scantimer->adjust(attotime::zero))
	s_cb_line = -1;
	s_cb_param = 0;
	s_last_partial = 0;
	s_line_tick = 0;
}

static void screen_configure(int width, int height, int min_x, int max_x, int min_y, int max_y)
{
	tms34010_screen_t *s = &tms34010_screen;
	if (width > (int)(sizeof(s_linebuf) / 2))
		width = sizeof(s_linebuf) / 2;
	s->width = width;
	s->height = height;
	s->min_x = min_x;
	s->max_x = max_x;
	s->min_y = min_y;
	s->max_y = max_y;
	// the beam position from the time since the last VBLANK start (screen_device::vpos)
	s->vpos = (int)((uint32_t)(max_y + 1) + s_ticks_since_vblank / (uint32_t)width) % height;
}

static void get_display_params(tms34010_display_params *params)
{
	params->enabled = ((SMART_IOREG(DPYCTL) & 0x8000) != 0);
	params->vcount = SMART_IOREG(VCOUNT);
	params->veblnk = SMART_IOREG(VEBLNK);
	params->vsblnk = SMART_IOREG(VSBLNK);
	params->heblnk = SMART_IOREG(HEBLNK) * s_cfg.pixperclock;
	params->hsblnk = SMART_IOREG(HSBLNK) * s_cfg.pixperclock;
	uint16_t dpyadr = IOREG(REG_DPYADR);
	if (!(IOREG(REG_DPYCTL) & 0x0400))
		dpyadr ^= 0xfffc;
	params->rowaddr = dpyadr >> 4;
	params->coladdr = ((dpyadr & 0x007c) << 4) | (IOREG(REG_DPYTAP) & 0x3fff);
	params->yoffset = (IOREG(REG_DPYSTRT) - IOREG(REG_DPYADR)) & 3;
}
void tms34010_get_display_params(tms34010_display_params *params) { get_display_params(params); }

// tms340x0_ind16 for one line
static void draw_line(int y)
{
	tms34010_screen_t *s = &tms34010_screen;
	tms34010_display_params params;
	get_display_params(&params);
	uint16_t *dest = s_linebuf;
	if (params.enabled)
		s_cfg.scanline(y, &params, dest);
	else
		params.heblnk = params.hsblnk = s->max_x + 1;
	for (int x = s->min_x; x < params.heblnk && x <= s->max_x; x++)
		dest[x] = s_cfg.black_pen;
	for (int x = params.hsblnk < s->min_x ? s->min_x : params.hsblnk; x <= s->max_x; x++)
		dest[x] = s_cfg.black_pen;
	if (s_cfg.bitmap)
		memcpy(s_cfg.bitmap + (size_t)(y - s->min_y) * s_cfg.pitch, dest + s->min_x, (size_t)(s->max_x - s->min_x + 1) * 2);
}

static void update_partial(int scanline)
{
	tms34010_screen_t *s = &tms34010_screen;
	if (scanline < s_last_partial)
		return;
	int top = s->min_y > s_last_partial ? s->min_y : s_last_partial;
	int bottom = s->max_y < scanline ? s->max_y : scanline;
	if (top > bottom)
		return;
	draw_line(top);   // (the update draws the first line of its range only, as tms340x0_ind16 does)
	s_last_partial = scanline + 1;
}

// MAME scanline_callback
static void scanline_callback(int vcount)
{
	tms34010_screen_t *s = &tms34010_screen;
	int enabled = SMART_IOREG(DPYCTL) & 0x8000;
	int master = (m_is_34020 || (SMART_IOREG(DPYCTL) & 0x2000));
	int vsblnk = SMART_IOREG(VSBLNK);
	int veblnk = SMART_IOREG(VEBLNK);
	int vtotal = SMART_IOREG(VTOTAL);
	if (!master) {
		vtotal = s->height - 1 < vtotal ? s->height - 1 : vtotal;
		vcount = s->vpos;
	}
	SMART_IOREG(VCOUNT) = vcount;
	if (enabled && vcount == SMART_IOREG(DPYINT))
		IOREG(REG_INTPEND) |= TMS34010_DI;   // (internal_interrupt_callback)
	if (vcount == vsblnk)
		IOREG(REG_DPYADR) = IOREG(REG_DPYSTRT);
	if (vcount == vtotal && master) {
		int htotal = SMART_IOREG(HTOTAL);
		if (htotal > 0 && vtotal > 0) {
			int width = (htotal + 1) * s_cfg.pixperclock;
			int height = vtotal + 1;
			int min_x = SMART_IOREG(HEBLNK) * s_cfg.pixperclock;
			int max_x = SMART_IOREG(HSBLNK) * s_cfg.pixperclock - 1;
			int min_y = veblnk;
			int max_y = vsblnk - 1;
			if (min_x < max_x && max_x <= width && min_y < max_y && max_y <= height) {
				if (width != s->width || height != s->height || min_y != s->min_y || max_y != s->max_y ||
					(m_hblank_stable > 2 && (min_x != s->min_x || max_x != s->max_x)))
					screen_configure(width, height, min_x, max_x, min_y, max_y);
				m_hblank_stable++;
			}
		}
	}
	if (vcount >= s->min_y && vcount <= s->max_y)
		update_partial(vcount);
	if (vcount >= veblnk && vcount < vsblnk) {
		uint16_t dpyadr = IOREG(REG_DPYADR);
		if ((dpyadr & 3) == 0)
			dpyadr = ((dpyadr & 0xfffc) - (IOREG(REG_DPYCTL) & 0x03fc)) | (IOREG(REG_DPYSTRT) & 0x0003);
		else
			dpyadr = (dpyadr & 0xfffc) | ((dpyadr - 1) & 3);
		IOREG(REG_DPYADR) = dpyadr;
	}
	vcount++;
	if (vcount > vtotal)
		vcount = 0;
	// m_scantimer->adjust(screen().time_until_pos(vcount), vcount): the line's start, a frame later when it is this one
	s_cb_pending = 1;
	s_cb_line = s->height ? vcount % s->height : 0;
	s_cb_param = vcount;
}

int tms34010_line_start(void)
{
	if (s_cb_pending && (s_cb_line < 0 || s_cb_line == tms34010_screen.vpos)) {
		s_cb_pending = 0;
		scanline_callback(s_cb_param);
	}
	return tms34010_screen.width;
}

int tms34010_line_end(void)
{
	tms34010_screen_t *s = &tms34010_screen;
	s_ticks_since_vblank += (uint32_t)s->width;
	s_line_tick += (uint64_t)s->width;
	if (++s->vpos >= s->height)
		s->vpos = 0;
	if (s->vpos == s->max_y + 1) {
		update_partial(s->max_y);
		s_last_partial = 0;
		s_ticks_since_vblank = 0;
		return 1;
	}
	return 0;
}

// the beam's x within the current line, from the CPU's time (the board runs it at clock / 8 against the pixel clock)
static int screen_hpos(void)
{
	const uint64_t rate = (uint64_t)s_cfg.pixclock * (uint64_t)s_cfg.pixperclock;
	const uint64_t cpu = s_cfg.clock / 8;
	uint64_t px = (tms34010_total_cycles() * rate * 2 + cpu) / (2 * cpu);   // (rounded to the nearest pixel, as MAME)
	px = px > s_line_tick ? px - s_line_tick : 0;
	return px >= (uint64_t)tms34010_screen.width ? tms34010_screen.width - 1 : (int)px;
}

// ---- I/O registers (MAME tms34010_device::io_register_w / _r) ----
static void io_register_w(offs_t offset, u16 data, u16 mem_mask)
{
	int oldreg, newreg;
	oldreg = IOREG(offset);
	IOREG(offset) = data;
	switch (offset) {
	case REG_CONTROL:
		set_raster_op();
		set_pixel_function();
		break;
	case REG_PSIZE:
		set_pixel_function();
		switch (data) {
		default:
		case 0x01: m_pixelshift = 0; break;
		case 0x02: m_pixelshift = 1; break;
		case 0x04: m_pixelshift = 2; break;
		case 0x08: m_pixelshift = 3; break;
		case 0x10: m_pixelshift = 4; break;
		}
		break;
	case REG_PMASK:
		break;
	case REG_DPYCTL:
		set_pixel_function();
		break;
	case REG_HSTCTLH:
		if (mem_mask & 0xff00) {
			if ((data & 0x8000) && !m_external_host_access)
				tms34010_abort();
			// NMI issued? (a scheduler sync: the interrupt is checked when the CPU runs next)
			if (data & 0x0100)
				tms34010_abort();
		}
		break;
	case REG_HSTCTLL:
		if (mem_mask & 0x00ff) {
			if (!m_external_host_access) {
				newreg = (oldreg & 0xff8f) | (data & 0x0070);
				newreg |= data & 0x0080;
				newreg &= data | ~0x0008;
			} else {
				newreg = (oldreg & 0xfff8) | (data & 0x0007);
				newreg &= data | ~0x0080;
				newreg |= data & 0x0008;
			}
			IOREG(offset) = (uint16_t)newreg;
			if (!(oldreg & 0x0080) && (newreg & 0x0080)) {
				if (s_cfg.output_int) s_cfg.output_int(1);
			} else if ((oldreg & 0x0080) && !(newreg & 0x0080)) {
				if (s_cfg.output_int) s_cfg.output_int(0);
			}
			if (!(oldreg & 0x0008) && (newreg & 0x0008)) {
				IOREG(REG_INTPEND) |= TMS34010_HI;
				tms34010_abort();
			} else if ((oldreg & 0x0008) && !(newreg & 0x0008))
				IOREG(REG_INTPEND) &= ~TMS34010_HI;
		}
		break;
	case REG_CONVSP:
		m_convsp = 1u << (~data & 0x1f);
		break;
	case REG_CONVDP:
		m_convdp = 1u << (~data & 0x1f);
		break;
	case REG_INTENB:
		check_interrupt();
		break;
	case REG_INTPEND:
		IOREG(REG_INTPEND) = (uint16_t)oldreg;
		if (!(data & TMS34010_WV))
			IOREG(REG_INTPEND) &= ~TMS34010_WV;
		if (!(data & TMS34010_DI))
			IOREG(REG_INTPEND) &= ~TMS34010_DI;
		break;
	case REG_HEBLNK:
	case REG_HSBLNK:
		if (oldreg != data)
			m_hblank_stable = 0;
		break;
	}
}

static u16 io_register_r(offs_t offset)
{
	int result, total;
	switch (offset) {
	case REG_HCOUNT:
		result = screen_hpos();
		total = IOREG(REG_HTOTAL) + 1;
		result = result * total / tms34010_screen.width;
		result += IOREG(REG_HEBLNK);
		if (result > total)
			result -= total;
		return (u16)result;
	case REG_REFCNT:
		return (u16)(((tms34010_total_cycles()) / 16) & 0xfffc);
	case REG_INTPEND:
		result = IOREG(offset);
		// (Cool Pool: DI shown a little early, when the scanline timer is less than 3 / (40 MHz / 8) away)
		if (SMART_IOREG(VCOUNT) + 1 == SMART_IOREG(DPYINT) && s_cb_pending && s_cb_line == (tms34010_screen.vpos + 1) % tms34010_screen.height) {
			int left = tms34010_screen.width - screen_hpos();
			// left pixels at pixclock * pixperclock vs 0.6 us
			if ((uint64_t)left * 5000000u < (uint64_t)s_cfg.pixclock * (uint64_t)s_cfg.pixperclock * 3u)
				result |= TMS34010_DI;
		}
		return (u16)result;
	}
	return IOREG(offset);
}

// ---- host interface (MAME host_w / host_r) ----
uint16_t tms34010_host_r(int reg)
{
	uint32_t addr;
	int result = 0;
	switch (reg) {
	case 1: result = IOREG(REG_HSTADRH); break;
	case 0: result = IOREG(REG_HSTADRL); break;
	case 2:
		addr = ((uint32_t)IOREG(REG_HSTADRH) << 16) | IOREG(REG_HSTADRL);
		result = (int)TMS34010_RDMEM_WORD(addr & 0xfffffff0);
		if (IOREG(REG_HSTCTLH) & 0x1000) {
			addr += 0x10;
			IOREG(REG_HSTADRH) = (uint16_t)(addr >> 16);
			IOREG(REG_HSTADRL) = (uint16_t)addr;
		}
		break;
	case 3: result = (IOREG(REG_HSTCTLH) & 0xff00) | (IOREG(REG_HSTCTLL) & 0x00ff); break;
	}
	return (uint16_t)result;
}

void tms34010_host_w(int reg, uint16_t data)
{
	uint32_t addr;
	switch (reg) {
	case 1: IOREG(REG_HSTADRH) = data; break;
	case 0: IOREG(REG_HSTADRL) = data; break;
	case 2:
		addr = ((uint32_t)IOREG(REG_HSTADRH) << 16) | IOREG(REG_HSTADRL);
		TMS34010_WRMEM_WORD(addr & 0xfffffff0, data);
		if (IOREG(REG_HSTCTLH) & 0x0800) {
			addr += 0x10;
			IOREG(REG_HSTADRH) = (uint16_t)(addr >> 16);
			IOREG(REG_HSTADRL) = (uint16_t)addr;
		}
		break;
	case 3:
		m_external_host_access = true;
		io_register_w(REG_HSTCTLH, data & 0xff00, 0xff00);
		io_register_w(REG_HSTCTLL, data & 0x00ff, 0x00ff);
		m_external_host_access = false;
		break;
	}
}

// ---- save state ----
int tms34010_state(void *ctx, int load)
{
	struct {
		uint32_t pc, ppc, st, convsp, convdp, convmp;
		int32_t gfxcycles;
		uint8_t pixelshift, reset_deferred, hblank_stable, pad;
		int32_t regs[31];
		int32_t cb_pending, cb_line, cb_param, last_partial;
		uint32_t ticks;
		uint64_t total, line_tick;
		tms34010_screen_t scr;
	} st;
	memset(&st, 0, sizeof(st));   // (the padding at its end: no stack bytes in the state)
	if (!load) {
		st.pc = m_pc; st.ppc = m_ppc; st.st = m_st; st.convsp = m_convsp; st.convdp = m_convdp; st.convmp = m_convmp;
		st.gfxcycles = m_gfxcycles; st.pixelshift = m_pixelshift; st.reset_deferred = m_reset_deferred;
		st.hblank_stable = m_hblank_stable; st.pad = 0;
		for (int i = 0; i < 31; i++) st.regs[i] = m_regs[i].reg;
		st.cb_pending = s_cb_pending; st.cb_line = s_cb_line; st.cb_param = s_cb_param; st.last_partial = s_last_partial;
		st.ticks = s_ticks_since_vblank; st.total = s_total; st.line_tick = s_line_tick; st.scr = tms34010_screen;
	}
	if (rbx_state_io(ctx, "tms0", &st, sizeof(st)) || rbx_state_io(ctx, "tmsio", m_IOregs, sizeof(m_IOregs)) ||
		rbx_state_io(ctx, "tmssr", m_shiftreg, sizeof(m_shiftreg)))
		return -1;
	if (load) {
		m_pc = st.pc; m_ppc = st.ppc; m_st = st.st; m_convsp = st.convsp; m_convdp = st.convdp; m_convmp = st.convmp;
		m_gfxcycles = st.gfxcycles; m_pixelshift = st.pixelshift; m_reset_deferred = st.reset_deferred;
		m_hblank_stable = st.hblank_stable;
		for (int i = 0; i < 31; i++) m_regs[i].reg = st.regs[i];
		s_cb_pending = st.cb_pending; s_cb_line = st.cb_line; s_cb_param = st.cb_param; s_last_partial = st.last_partial;
		s_ticks_since_vblank = st.ticks; s_total = st.total; s_line_tick = st.line_tick; tms34010_screen = st.scr;
		set_raster_op();
		set_pixel_function();
	}
	return 0;
}
