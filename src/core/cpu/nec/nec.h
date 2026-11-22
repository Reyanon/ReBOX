/*  NEC V30 / V33 / V35 interpreter. Ported from MAME src/devices/cpu/nec (nec.cpp, necinstr.hxx, necmacro.h, necea.h,
    necmodrm.h, v25.cpp, v25instr.hxx, v25sfr.cpp; BSD-3-Clause, copyright-holders: Bryan McPhail, Alex W. Jackson), the
    cycle counts and the prefetch model as MAME's. Opcode decryption tables: irem_cpu.h. See nec.c.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef NEC_H
#define NEC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { NEC_V30, NEC_V33, NEC_V35 };

// lines of nec_set_input (MAME execute_set_input)
enum { NEC_LINE_IRQ, NEC_LINE_NMI, NEC_LINE_INTP0, NEC_LINE_INTP1, NEC_LINE_INTP2, NEC_LINE_POLL };

typedef struct nec_t {
	// V25 / V35: the internal RAM with the eight register banks; V30 / V33: the registers, bank 7 of the same layout
	uint16_t iram[128];
	uint16_t ip, prev_ip, rep_ip;
	uint16_t EO, E16;
	uint8_t TF, IF, DF, MF;
	int32_t SignVal;
	uint32_t AuxVal, OverVal, ZeroVal, CarryVal, ParityVal;
	uint32_t pending_irq, nmi_state, irq_state, poll_state;
	uint8_t no_interrupt, halted, prefetch_reset, seg_prefix;
	uint32_t rep_params, prefix_base, EA;
	int32_t prefetch_count, cur_cycles;
	uint32_t popa_tmp;
	int32_t ic;                // (at the start of the running instruction; nec_now)
	int32_t slice;             // cycles of the running slice
	int64_t total;             // cycles run; on the V35 the clocks run (cycles x PCK)
	// V33
	uint16_t transtable[64];
	uint8_t xa;
	// V25 / V35
	uint8_t IBRK, F0, F1, mode_state, intm, RAMEN, TB, PCK, RFM, TMC0, TMC1;
	uint8_t priority_inttu, priority_intd, priority_intp, priority_ints0, priority_ints1;
	uint8_t ems[3], srms[2], stms[2], tmms[3], scm[2], scc[2], brg[2], sce[2], dmac[2], dmam[2];
	uint8_t IRQS, ISPR;
	uint8_t intp_state[3];
	uint16_t rbw;              // bank in words (RBW of MAME)
	uint16_t TM0, MD0, TM1, MD1, WTC;
	uint32_t IDB, unmasked_irq, macro_service, bankswitch_irq;
	int64_t tm_next[4], tm_period[4];   // the four timers in clocks (0 = stopped)
	uint32_t tm_param[4];
	uint8_t state_end;

	int chip;
	// set from a handler (V30 / V33): the running nec_run_until returns after this instruction (MAME abort_timeslice)
	uint8_t stop;
	// MAME's scheduler slices of this many cycles (0: none): a REP loop stops at a slice's end and pays its 2 cycles again
	// (the M72 boards' add_quantum for 25 seconds: their boot timing)
	int rep_q;
	uint16_t *rp;              // the current bank
	const uint8_t *dec;        // V35: opcode decryption table (secure mode), NULL = none

	// pages of 4K: NULL = through the handlers; omap for opcode fetches
	const uint8_t *omap[256], *rmap[256];
	uint8_t *wmap[256];
	const uint8_t *idb_r[2];   // (V35: what the internal area's pages hide)
	uint8_t *idb_w[2];
	uint8_t idb_page[2];

	uint8_t (*read8)(uint32_t a);
	uint16_t (*read16)(uint32_t a);     // even addresses; odd ones are two read8
	void (*write8)(uint32_t a, uint8_t v);
	void (*write16)(uint32_t a, uint16_t v);
	uint8_t (*in8)(uint32_t p);
	uint16_t (*in16)(uint32_t p);
	void (*out8)(uint32_t p, uint8_t v);
	void (*out16)(uint32_t p, uint16_t v);
	int (*irq_ack)(void);               // the vector; NULL = 0xff
	// V35 ports (NULL: reads 0xff, writes nothing)
	uint8_t (*port_in)(int port);       // 0..2 P0..P2, 3 PT
	void (*port_out)(int port, uint8_t v);
	uint8_t fbuf[64];          // (the bytes of an instruction at a page's end)
	// the exact skipping of polling loops inside a run (nec_idle_enable; V30 / V33). Not state: only where a run is
	// split, never its result
	uint8_t idle_on;
	uint8_t idle_badn[64];     // (the probes failed at the target idle_bad[] of a slot)
	uint32_t idle_bad[64];
	uint32_t idle_tgt;         // (the last taken backward branch's target and how often it came again)
	int idle_cnt;
	int (*idle_pure)(uint32_t a);
} nec_t;

#define NEC_STATE_SIZE ((uint32_t)offsetof(nec_t, state_end))

// the pointers and pages are left alone: map first, then reset. dec only for the V35
void nec_init(nec_t *c, int chip, const uint8_t *dec);
void nec_reset(nec_t *c);
// a range of pages (start, end at 4K boundaries) for fetches and reads, and writes; NULL = the handlers
void nec_map(nec_t *c, uint32_t start, uint32_t end, const uint8_t *r, uint8_t *w);
// run until total reaches end: V30 / V33 in CPU cycles, V35 in its input clocks (timers included)
void nec_run_until(nec_t *c, int64_t end);
int64_t nec_now(const nec_t *c);        // total, inside handlers too
void nec_set_input(nec_t *c, int line, int state);
void nec_after_load(nec_t *c);          // state bytes back: the bank pointer, the internal area's pages
// Exact skipping of polling loops inside nec_run_until (V30 / V33; after the board's nec_map). A taken backward branch
// seen a third time in a row ends the slice; one iteration of the loop is then run an instruction at a time with the
// writes and the handler reads caught. When it comes back to its first instruction with every register as before,
// every byte it wrote back to its old value, no write through a handler, no I/O port, no handler read but of the
// addresses `pure` accepts (side-effect free, their value changed only by this CPU within a run; NULL: none), no
// interrupt pending and no REP instruction, the next iterations are the same: the whole ones left before the run's
// end are counted without being run. Rules for the board: nothing but this CPU changes its direct pages or the pure
// addresses inside nec_run_until, and an abort (stop = 1) comes only from a handler. One CPU probing at a time
// (static state: the emulation thread's CPUs only).
void nec_idle_enable(nec_t *c, int (*pure)(uint32_t a));

#ifdef __cplusplus
}
#endif

#endif
