/*  NEC uPD7810 / 78C10 / 78C11 CPU, ported from MAME src/devices/cpu/upd7810/upd7810.cpp (BSD-3-Clause,
    copyright-holders: Juergen Buchmueller). The opcode functions, tables and peripherals are converted from the
    MAME sources by tools/port_upd7810.py into upd7810_gen.inc; this file holds the memory interface, the
    execute loop (MAME execute_run) and the C interface.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "upd7810.h"
#include "../../rbx.h"

#include <stddef.h>
#include <string.h>

typedef union {
	struct { uint8_t l, h, h2, h3; } b;
	struct { uint16_t l, h; } w;
	struct { int8_t l, h, h2, h3; } sb;
	struct { int16_t l, h; } sw;
	uint32_t d;
	int32_t sd;
} PAIR;   // little-endian hosts (PC, Vita, PSP)

#define ASSERT_LINE 1
#define CLEAR_LINE 0
#define BIT(x, n) (((x) >> (n)) & 1)
#define UPD_LOG(...) ((void)0)
#define PAR7(n) ((((n)>>6)^((n)>>5)^((n)>>4)^((n)>>3)^((n)>>2)^((n)>>1)^((n)))&1)
#define PAR8(n) ((((n)>>7)^((n)>>6)^((n)>>5)^((n)>>4)^((n)>>3)^((n)>>2)^((n)>>1)^((n)))&1)
typedef uint32_t offs_t;

static upd7810_config s_cfg;
static uint8_t s_iram[256];   // internal RAM at 0xff00 (enabled by MM bit 3)

static inline uint8_t upd_rm(uint16_t a);
static inline void upd_wm(uint16_t a, uint8_t v);
#define RDOP(O)     O = upd_rm((uint16_t)PCD); PC++
#define RDOPARG(A)  A = upd_rm((uint16_t)PCD); PC++
#define RM(A)       upd_rm((uint16_t)(A))
#define WM(A,V)     upd_wm((uint16_t)(A), (uint8_t)(V))

#include "upd7810_gen.inc"

static inline uint8_t upd_rm(uint16_t a)
{
	if (a < s_cfg.irom_size)
		return s_cfg.irom[a];
	if (a >= 0xff00 && m_ram_enabled)
		return s_iram[a & 0xff];
	return s_cfg.read ? s_cfg.read(a) : 0xff;
}

static inline void upd_wm(uint16_t a, uint8_t v)
{
	if (a < s_cfg.irom_size)
		return;
	if (a >= 0xff00 && m_ram_enabled) {
		s_iram[a & 0xff] = v;
		return;
	}
	if (s_cfg.write)
		s_cfg.write(a, v);
}

void upd7810_init(const upd7810_config *cfg)
{
	s_cfg = *cfg;
	// configure_ops (uPD7810 family)
	m_opXX = s_opXX_7810;
	m_op48 = s_op48;
	m_op4C = s_op4C;
	m_op4D = s_op4D;
	m_op60 = s_op60;
	m_op64 = s_op64;
	m_op70 = s_op70;
	m_op74 = s_op74;
	memset(&S, 0, sizeof(S));
	m_pa_pullups = m_pb_pullups = m_pc_pullups = m_pd_pullups = m_pf_pullups = 0xff;
	// device_start
	MM = 8;
	m_ram_enabled = 1;
	memset(s_iram, 0, sizeof(s_iram));
	upd7810_reset();
}

void upd7810_reset(void)
{
	int icount = m_icount;
	device_reset();
	m_ram_enabled = (MM & 8) ? 1 : 0;
	m_icount = icount;
}

uint16_t upd7810_pc(void) { return PC; }

void upd7810_set_input(int line, int state) { execute_set_input(line, state); }

// MAME execute_run
int upd7810_execute(int cycles)
{
	int start;
	m_icount += cycles;
	start = m_icount;
	while (m_icount > 0) {
		int cc;
		if (m_halt) {
			// continue running peripherals until a pending (internal or external) interrupt
			m_icount--;
			handle_timers(1);
			upd7810_take_irq();
			continue;
		}

		PPC = PC;
		RDOP(OP);

		/* clear L0 and/or L1 flags for all opcodes except L0 for "MVI L,xx" or "LXI H,xxxx", L1 for "MVI A,xx" */
		PSW &= ~m_opXX[OP].mask_l0_l1;

		/* skip flag set and not SOFTI opcode? */
		if ((PSW & SK) && (OP != 0x72)) {
			if (m_opXX[OP].cycles) {
				cc = m_opXX[OP].cycles_skip;
				PC += m_opXX[OP].oplen - 1;
			} else {
				RDOP(OP2);
				switch (OP) {
				case 0x48: cc = m_op48[OP2].cycles_skip; PC += m_op48[OP2].oplen - 2; break;
				case 0x4c: cc = m_op4C[OP2].cycles_skip; PC += m_op4C[OP2].oplen - 2; break;
				case 0x4d: cc = m_op4D[OP2].cycles_skip; PC += m_op4D[OP2].oplen - 2; break;
				case 0x60: cc = m_op60[OP2].cycles_skip; PC += m_op60[OP2].oplen - 2; break;
				case 0x64: cc = m_op64[OP2].cycles_skip; PC += m_op64[OP2].oplen - 2; break;
				case 0x70: cc = m_op70[OP2].cycles_skip; PC += m_op70[OP2].oplen - 2; break;
				case 0x74: cc = m_op74[OP2].cycles_skip; PC += m_op74[OP2].oplen - 2; break;
				default: cc = 4; break;   // (MAME: fatal error)
				}
			}
			PSW &= ~SK;
			handle_timers(cc);
		} else {
			cc = m_opXX[OP].cycles;
			handle_timers(cc);
			m_opXX[OP].opfunc();
		}
		m_icount -= cc;
		upd7810_take_irq();
		m_iff = m_iff_pending;
	}
	return start - m_icount;
}

int upd7810_state(void *ctx, int load)
{
	(void)load;
	int rc = rbx_state_io(ctx, "upd7810", &S, sizeof(S));
	rc |= rbx_state_io(ctx, "upd7810ram", s_iram, sizeof(s_iram));
	return rc;
}
