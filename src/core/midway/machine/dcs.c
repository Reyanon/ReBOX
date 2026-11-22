/*  The Midway DCS sound board, rev 1 (ADSP-2105, ROM-based mono: DCS_AUDIO_2K, DCS_AUDIO_8K), ported to C from MAME
    src/mame/shared/dcs.cpp, dcs.h (BSD-3-Clause, copyright-holders: Aaron Giles) with the DMA DAC of
    src/devices/sound/dmadac.cpp, dmadac.h (BSD-3-Clause, copyright-holders: Aaron Giles).
    MAME's timers are events in ADSP cycles (the sample timer in 16.16 cycles); its synchronize() callbacks are applied
    at once (the board runs behind the host and is brought up to the host's time before each access).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "dcs.h"
#include "../../rbx.h"
#include "../../cpu/adsp2100/adsp2100.h"

#include <stdlib.h>
#include <string.h>

#define FIFO_SIZE 32768            // dmadac BUFFER_SIZE
#define LCTRL_OUTPUT_EMPTY 0x400
#define LCTRL_INPUT_EMPTY  0x800

enum {
	IDMA_CONTROL_REG = 0, BDMA_INT_ADDR_REG, BDMA_EXT_ADDR_REG, BDMA_CONTROL_REG, BDMA_WORD_COUNT_REG,
	PROG_FLAG_DATA_REG, PROG_FLAG_CONTROL_REG,
	S1_AUTOBUF_REG = 15, S1_RFSDIV_REG, S1_SCLKDIV_REG, S1_CONTROL_REG, S0_AUTOBUF_REG, S0_RFSDIV_REG, S0_SCLKDIV_REG,
	S0_CONTROL_REG, S0_MCTXLO_REG, S0_MCTXHI_REG, S0_MCRXLO_REG, S0_MCRXHI_REG, TIMER_SCALE_REG, TIMER_COUNT_REG,
	TIMER_PERIOD_REG, WAITSTATES_REG, SYSCONTROL_REG
};

static int s_type;
static uint16_t *s_rom;            // the "dcs" region (boot ROM = sound data)
static uint32_t s_rom_words, s_banks;
static uint32_t *s_pint;           // internal program RAM (0x400 words)
static uint32_t *s_pext;           // external program RAM (2K: 0x800, 8K: 0x1800 words)
static uint16_t *s_dlow;           // 8K: data RAM 0x0000-0x07ff
static uint16_t *s_diram;          // data RAM 0x3800-0x39ff (a page)
static int16_t *s_fifo;            // the DAC's buffer
static uint32_t s_pext_words;

// state (saved)
static struct {
	uint16_t control_regs[32];
	uint32_t sounddata_bank;
	int32_t size, incs, ireg;
	uint32_t ireg_base;
	uint16_t latch_control, input_data, output_data, pre_output_data, output_control, progflags;
	uint64_t output_control_cycles;
	uint8_t halted, cpu_reset_pending, irq1_clear, dac_enabled, timer_enable, timer_ignore, reg_on, int_on;
	uint64_t timer_start_cycles, timers_fired;
	uint32_t timer_start_count, timer_scale, timer_period;
	uint64_t reg_at, reg_period;   // the sample timer (dcs_irq), 16.16 cycles
	uint64_t int_at;               // the internal timer
	uint32_t cps;                  // the DAC: cycles per sample
	uint64_t stream_at;            // the next output sample
	uint32_t bufin, bufout;
} D;

static void dcs_sync(void);

#define IS_OUTPUT_EMPTY()  (D.latch_control & LCTRL_OUTPUT_EMPTY)
#define SET_OUTPUT_EMPTY() (D.latch_control |= LCTRL_OUTPUT_EMPTY)
#define SET_OUTPUT_FULL()  (D.latch_control &= ~LCTRL_OUTPUT_EMPTY)
#define SET_INPUT_EMPTY()  (D.latch_control |= LCTRL_INPUT_EMPTY)
#define SET_INPUT_FULL()   (D.latch_control &= ~LCTRL_INPUT_EMPTY)

// ---- the DMA DAC ----
static void dac_enable(int enable)
{
	D.dac_enabled = (uint8_t)enable;
	if (!enable)
		D.bufin = D.bufout = 0;
}

static void dac_set_rate(uint32_t cps)
{
	D.cps = cps;
	rbx_audio.rate = (DCS_CLOCK + cps / 2) / cps;
}

// ---- events (a timer set from a handler ends the ADSP's timeslice, as MAME's new first timer) ----
static void set_int_timer(uint64_t at)
{
	D.int_on = 1;
	D.int_at = at;
	adsp_abort();
}

static void set_reg_timer(uint64_t period16)
{
	D.reg_period = period16;
	D.reg_at = ((uint64_t)adsp_total_cycles() << 16) + period16;
	D.reg_on = period16 != 0;
	adsp_abort();
}

// ---- memory banking and boot ----
static void set_data_bank(void)
{
	uint16_t *base = s_rom + (size_t)(D.sounddata_bank % s_banks) * 0x1000;
	for (int p = 0; p < 4; p++)
		adsp_map_data_mem(8 + p, base + p * 0x400, 0);
}

static void dcs_boot(void)
{
	// rev 1: use the last set data bank to boot from
	const uint16_t *base = &s_rom[(D.sounddata_bank * 0x1000) % s_rom_words];
	uint8_t buffer[0x1000];
	for (int i = 0; i < 0x1000; i++)
		buffer[i] = (uint8_t)base[i];
	adsp_load_boot_data(buffer, s_pint);
}

// MAME dcs_reset (the timer callback)
static void dcs_reset_board(void)
{
	D.sounddata_bank = 0;
	set_data_bank();
	D.size = 0;
	D.incs = 0;
	D.ireg = 0;
	memset(D.control_regs, 0, sizeof(D.control_regs));
	adsp_set_input(ADSP2105_IRQ0, 0);
	adsp_set_input(ADSP2105_IRQ1, 0);
	adsp_set_input(ADSP2105_IRQ2, 0);
	SET_INPUT_EMPTY();
	SET_OUTPUT_EMPTY();
	dcs_boot();
	D.timer_ignore = 0;
	D.timer_enable = 0;
	D.timer_scale = 1;
	D.int_on = 0;
}

// ---- the internal timer ----
static void update_timer_count(void)
{
	if (!D.timer_enable)
		return;
	uint64_t elapsed_cycles = adsp_total_cycles() - D.timer_start_cycles;
	uint64_t elapsed_clocks = elapsed_cycles / D.timer_scale;
	if (elapsed_clocks < (uint64_t)D.timer_start_count + 1) {
		D.timer_start_count -= (uint32_t)elapsed_clocks;
		D.control_regs[TIMER_COUNT_REG] = (uint16_t)D.timer_start_count;
	} else {
		elapsed_clocks -= (uint64_t)D.timer_start_count + 1;
		uint64_t periods_since_start = elapsed_clocks / ((uint64_t)D.timer_period + 1);
		elapsed_clocks -= periods_since_start * ((uint64_t)D.timer_period + 1);
		D.timer_start_count = D.timer_period - (uint32_t)elapsed_clocks;
		D.control_regs[TIMER_COUNT_REG] = (uint16_t)D.timer_start_count;
	}
}

static void internal_timer_callback(void)
{
	D.timers_fired++;
	int64_t target_cycles = (int64_t)(D.timer_start_cycles + (uint64_t)D.timer_scale *
		((uint64_t)D.timer_start_count + 1 + D.timers_fired * ((uint64_t)D.timer_period + 1)));
	uint64_t now = adsp_total_cycles();
	if (!D.timer_ignore && (D.timer_period > 10 || D.timer_scale > 1))
		set_int_timer(target_cycles > (int64_t)now ? (uint64_t)target_cycles : now);
	else
		D.int_on = 0;
	// the IRQ line is edge triggered
	adsp_set_input(ADSP2105_TIMER, 1);
	adsp_set_input(ADSP2105_TIMER, 0);
}

static void reset_timer(void)
{
	if (!D.timer_enable)
		return;
	D.timer_start_cycles = adsp_total_cycles();
	D.timers_fired = 0;
	// the IRQ routine is only the DRAM refresh stub: the timer need not fire
	if (!D.timer_ignore &&
		adsp_prog_read(0x18) == 0x0c0030 &&      // ENA SEC_REG
		adsp_prog_read(0x19) == 0x804828 &&      // SI = DM($0482)
		adsp_prog_read(0x1a) == 0x904828 &&      // DM($0482) = SI
		adsp_prog_read(0x1b) == 0x0c0020 &&      // DIS SEC_REG
		adsp_prog_read(0x1c) == 0x0a001f)        // RTI
		D.timer_ignore = 1;
	if (!D.timer_ignore)
		set_int_timer(D.timer_start_cycles + (uint64_t)D.timer_scale * ((uint64_t)D.timer_start_count + 1));
}

static void timer_enable_callback(int state)
{
	if (state) {
		D.timer_enable = (uint8_t)state;
		reset_timer();
	} else {
		update_timer_count();
		D.timer_enable = 0;
		D.int_on = 0;
	}
}

// ---- the sample output (SPORT1 autobuffer) ----
static void dcs_irq(void)
{
	int reg = (int)adsp_get_i(D.ireg);
	int count = D.size / (2 * (D.incs ? D.incs : 1));
	if (count > 0x800)
		count = 0x800;
	uint32_t maxin = (D.bufout + FIFO_SIZE - 1) % FIFO_SIZE;
	for (int i = 0; i < count; i++) {
		int16_t v = (int16_t)adsp_data_read((uint32_t)reg);
		reg += D.incs;
		if (D.dac_enabled && D.bufin != maxin) {
			s_fifo[D.bufin] = v;
			D.bufin = (D.bufin + 1) % FIFO_SIZE;
		}
	}
	D.ireg_base = adsp_get_ibase(D.ireg);
	if ((uint32_t)reg >= D.ireg_base + (uint32_t)D.size) {
		reg = (int)D.ireg_base;
		// the (internal) IRQ, a pulse
		adsp_set_input(ADSP2105_IRQ1, 1);
		D.irq1_clear = 1;
	}
	adsp_set_i(D.ireg, (uint32_t)reg);
}

static void recompute_sample_rate(void)
{
	uint32_t cps;
	if (D.control_regs[S1_CONTROL_REG] & 0x4000)   // the internal clock for SPORT1
		cps = 2u * (D.control_regs[S1_SCLKDIV_REG] + 1u) * 16u;
	else                                             // the external 31.25 kHz sample clock
		cps = DCS_CLOCK / 31250u;
	if (cps != D.cps)
		dac_set_rate(cps);
	D.dac_enabled = 1;
	if (D.incs)
		set_reg_timer(((uint64_t)cps * (uint32_t)D.size << 16) / (uint64_t)(2 * D.incs));
}

static void sound_tx_callback(int port, uint32_t data)
{
	(void)data;
	if (port != 1)
		return;
	if ((D.control_regs[SYSCONTROL_REG] & 0x0800) && (D.control_regs[S1_AUTOBUF_REG] & 0x0002)) {
		D.ireg = (D.control_regs[S1_AUTOBUF_REG] >> 9) & 7;
		int mreg = (D.control_regs[S1_AUTOBUF_REG] >> 7) & 3;
		mreg |= D.ireg & 0x04;
		int lreg = D.ireg;
		uint16_t source = (uint16_t)adsp_get_i(D.ireg);
		D.incs = adsp_get_m(mreg);
		D.size = (int32_t)adsp_get_l(lreg);
		source &= ~0xf;
		adsp_set_i(D.ireg, source);
		D.ireg_base = source;
		recompute_sample_rate();
		return;
	}
	dac_enable(0);
	D.reg_on = 0;
}

// ---- the ADSP's control registers ----
static uint16_t adsp_control_r(uint32_t offset)
{
	uint16_t result;
	switch (offset) {
	case PROG_FLAG_DATA_REG:
		result = (uint16_t)((D.control_regs[PROG_FLAG_CONTROL_REG] & D.control_regs[PROG_FLAG_DATA_REG]) |
		                    (D.progflags & ~D.control_regs[PROG_FLAG_CONTROL_REG]));
		D.progflags ^= 0x6;
		break;
	case IDMA_CONTROL_REG:
		result = 0xffff;
		break;
	case TIMER_COUNT_REG:
		update_timer_count();
		result = D.control_regs[offset];
		break;
	default:
		result = D.control_regs[offset];
		break;
	}
	return result;
}

static void adsp_control_w(uint32_t offset, uint16_t data)
{
	D.control_regs[offset] = data;
	switch (offset) {
	case SYSCONTROL_REG:
		// bit 9 forces a reset
		if (data & 0x0200) {
			D.cpu_reset_pending = 1;   // (pulse_input_line(INPUT_LINE_RESET): at the end of the timeslice)
			adsp_abort();
			dcs_boot();
			D.control_regs[SYSCONTROL_REG] = 0;
		}
		// see if SPORT1 got disabled
		if ((data & 0x0800) == 0) {
			dac_enable(0);
			D.reg_on = 0;
		}
		break;
	case S1_AUTOBUF_REG:
		// autobuffer off: nuke the timer, and disable the DAC
		if ((data & 0x0002) == 0) {
			dac_enable(0);
			D.reg_on = 0;
		}
		break;
	case TIMER_SCALE_REG: {
		uint32_t d = (data & 0xff) + 1u;
		if (d != D.timer_scale) {
			update_timer_count();
			D.timer_scale = d;
			reset_timer();
		}
		break;
	}
	case TIMER_COUNT_REG:
		D.timer_start_count = data;
		reset_timer();
		break;
	case TIMER_PERIOD_REG:
		if (data != D.timer_period) {
			update_timer_count();
			D.timer_period = data;
			reset_timer();
		}
		break;
	}
}

// ---- the ADSP's buses ----
static uint16_t input_latch_r(void)
{
	// (auto ack)
	SET_INPUT_EMPTY();
	adsp_set_input(ADSP2105_IRQ2, 0);
	return D.input_data;
}

static void output_latch_w(uint16_t data)
{
	D.pre_output_data = data;
	SET_OUTPUT_FULL();
	D.output_data = D.pre_output_data;
}

static uint16_t dio12_r(uint32_t a) { (void)a; return 0; }
static void dio12_w(uint32_t a, uint16_t d)
{
	if (s_type == DCS_8K && a != 0x3000)
		return;
	D.sounddata_bank = d & 0x7ff;   // (bit 11: the board's LED)
	set_data_bank();
}
static uint16_t dio13_r(uint32_t a)
{
	if (s_type == DCS_8K && a > 0x3403)
		return 0;
	return input_latch_r();
}
static void dio13_w(uint32_t a, uint16_t d)
{
	if (s_type == DCS_8K && a > 0x3403)
		return;
	output_latch_w(d);
}
static uint16_t dio15_r(uint32_t a) { return a >= 0x3fe0 ? adsp_control_r(a - 0x3fe0) : 0; }
static void dio15_w(uint32_t a, uint16_t d) { if (a >= 0x3fe0) adsp_control_w(a - 0x3fe0, d); }
// 8K: the latch in program space 0x3000-0x3003
static uint32_t pio12_r(uint32_t a) { return a <= 0x3003 ? (uint32_t)input_latch_r() << 8 : 0; }
static void pio12_w(uint32_t a, uint32_t d)
{
	if (a > 0x3003)
		return;
	D.pre_output_data = (uint16_t)(d >> 8);
	SET_OUTPUT_FULL();
	D.output_data = D.pre_output_data;
}

static void map_memory(void)
{
	for (int p = 0; p < 16; p++) {
		adsp_map_prog_none(p);
		adsp_map_data_none(p);
	}
	adsp_map_prog_mem(0, s_pint);
	if (s_type == DCS_2K) {
		for (int p = 2; p < 8; p++)
			adsp_map_prog_mem(p, s_pext + (p & 1) * 0x400);
		for (int p = 0; p < 8; p++)   // 0x0000-0x07ff mirror 0x1800: the external program RAM's upper 16 bits
			adsp_map_data_hi(p, s_pext + (p & 1) * 0x400);
	} else {
		for (int p = 2; p < 8; p++)
			adsp_map_prog_mem(p, s_pext + (p - 2) * 0x400);
		adsp_map_prog_io(12, pio12_r, pio12_w);
		adsp_map_data_mem(0, s_dlow, 1);
		adsp_map_data_mem(1, s_dlow + 0x400, 1);
		for (int p = 2; p < 8; p++)
			adsp_map_data_hi(p, s_pext + (p - 2) * 0x400);
	}
	set_data_bank();
	adsp_map_data_io(12, dio12_r, dio12_w);
	adsp_map_data_io(13, dio13_r, dio13_w);
	adsp_map_data_mem(14, s_diram, 1);
	adsp_map_data_io(15, dio15_r, dio15_w);
}

// ---- the host's interface ----
int dcs_init(int type, uint8_t *rom, uint32_t rom_size)
{
	s_type = type;
	s_rom = (uint16_t *)rom;
	s_rom_words = rom_size / 2;
	s_banks = s_rom_words / 0x1000;
	if (!rom || !s_banks)
		return -1;
	s_pext_words = type == DCS_2K ? 0x800 : 0x1800;
	s_pint = (uint32_t *)calloc(0x400, 4);
	s_pext = (uint32_t *)calloc(s_pext_words, 4);
	s_dlow = (uint16_t *)calloc(0x800, 2);
	s_diram = (uint16_t *)calloc(0x400, 2);
	s_fifo = (int16_t *)calloc(FIFO_SIZE, 2);
	if (!s_pint || !s_pext || !s_dlow || !s_diram || !s_fifo) {
		dcs_exit();
		return -1;
	}
	static const adsp_callbacks cb = { sound_tx_callback, timer_enable_callback };
	adsp2105_init(&cb);
	memset(&D, 0, sizeof(D));
	map_memory();
	return 0;
}

void dcs_exit(void)
{
	dcs_set_async(0);
	adsp_exit();
	free(s_pint); s_pint = NULL;
	free(s_pext); s_pext = NULL;
	free(s_dlow); s_dlow = NULL;
	free(s_diram); s_diram = NULL;
	free(s_fifo); s_fifo = NULL;
}

uint32_t dcs_rate(void) { return 31250; }

void dcs_reset(void)
{
	dcs_sync();
	// (the board's time restarts with the machine's)
	adsp_state_time_reset();
	memset(&D, 0, sizeof(D));
	D.cps = DCS_CLOCK / 31250u;
	rbx_audio.rate = 31250;
	D.stream_at = D.cps;
	// device_reset: dcs_reset; the ADSP's device_reset
	dcs_reset_board();
	adsp_reset();
}

static void reset_w_now(int state)
{
	if (!state) {
		// going low halts the CPU: run through the init code again
		dcs_reset_board();
		D.halted = 1;
	} else if (D.halted) {
		// going high resets and reactivates the CPU
		adsp_reset();
		D.halted = 0;
	}
}

static void data_w_now(uint16_t data)
{
	// (rev 1: dcs_delayed_data_w by synchronize)
	adsp_set_input(ADSP2105_IRQ2, 1);
	SET_INPUT_FULL();
	D.input_data = data;
}

static void run_now(uint64_t target);

/*  The board on the worker thread (ReBOX, rbx_aux; RBX_DCS_NOTHREAD=1 = off): the host's runs, data writes and reset
    line changes are queued in their order and done by the job, so the board does exactly what it does inline, the
    host's CPU meanwhile running on. Its reads (data, control), the state, the reset and the frame's end (the samples)
    wait for the queue to be done first (dcs_sync). Without a worker thread the kick runs the job at once. */
enum { Q_RUN, Q_DATA, Q_RESET };
#define QSIZE 1024
static struct { uint8_t kind; uint16_t v; uint64_t t; } s_q[QSIZE];
static unsigned s_qhead, s_qtail;   // (the job's / the host's: atomics)
static int s_async;

static void dcs_job(void)
{
	unsigned h = __atomic_load_n(&s_qhead, __ATOMIC_ACQUIRE);
	const unsigned t = __atomic_load_n(&s_qtail, __ATOMIC_ACQUIRE);
	for (; h != t; h = (h + 1) % QSIZE) {
		switch (s_q[h].kind) {
		case Q_RUN: run_now(s_q[h].t); break;
		case Q_DATA: data_w_now(s_q[h].v); break;
		case Q_RESET: reset_w_now(s_q[h].v); break;
		}
		__atomic_store_n(&s_qhead, (h + 1) % QSIZE, __ATOMIC_RELEASE);
	}
}

// (profiling, the log every 300 frames: the host's syncs by kind, those with work still queued; no clock reads)
enum { SY_CONTROL, SY_DATA, SY_FRAME, SY_FULL, SY_OTHER };
static unsigned s_sy[5], s_sy_q;
extern uint64_t rbx_aux_sync_us[3];

static void dcs_sync_k(int kind)
{
	if (s_async) {
		s_sy[kind]++;
		if (__atomic_load_n(&s_qhead, __ATOMIC_ACQUIRE) != s_qtail)
			s_sy_q++;
		rbx_aux_sync();
	}
}
static void dcs_sync(void) { dcs_sync_k(SY_OTHER); }

static void dcs_stat(void)
{
	static unsigned f;
	static uint64_t insns0, wait0;
	if (++f % 300)
		return;
	const uint64_t insns = adsp_insns(), wait = rbx_aux_sync_us[0];
	rbx_log("dcs: adsp %.0f insns/frame; host syncs/frame: control %.1f data %.1f frame %.1f full %.1f (work queued %.1f); "
	        "sync wait %.2f ms/frame", (insns - insns0) / 300.0, s_sy[SY_CONTROL] / 300.0, s_sy[SY_DATA] / 300.0,
	        s_sy[SY_FRAME] / 300.0, s_sy[SY_FULL] / 300.0, s_sy_q / 300.0, (wait - wait0) / 300000.0);
	insns0 = insns;
	wait0 = wait;
	memset(s_sy, 0, sizeof(s_sy));
	s_sy_q = 0;
}

static void q_put(int kind, uint16_t v, uint64_t t)
{
	const unsigned tail = s_qtail, next = (tail + 1) % QSIZE;
	if (next == __atomic_load_n(&s_qhead, __ATOMIC_ACQUIRE))
		dcs_sync_k(SY_FULL);   // (full: the job catches up)
	s_q[tail].kind = (uint8_t)kind;
	s_q[tail].v = v;
	s_q[tail].t = t;
	__atomic_store_n(&s_qtail, next, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

void dcs_set_async(int on)
{
	if (s_async) {
		rbx_aux_set_job(NULL);   // (syncs first)
		s_async = 0;
	}
	s_qhead = s_qtail = 0;
	if (on && !getenv("RBX_DCS_NOTHREAD")) {
		s_async = 1;
		rbx_aux_set_job(dcs_job);
		rbx_aux_set_spin(300);   // (a job every scanline)
	}
}

void dcs_frame_end(void)
{
	dcs_sync_k(SY_FRAME);
	dcs_stat();
}

void dcs_reset_w(int state)
{
	if (s_async)
		q_put(Q_RESET, (uint16_t)(state != 0), 0);
	else
		reset_w_now(state);
}

void dcs_data_w(uint16_t data)
{
	if (s_async)
		q_put(Q_DATA, data, 0);
	else
		data_w_now(data);
}

void dcs_run(uint64_t target)
{
	if (s_async)
		q_put(Q_RUN, 0, target);
	else
		run_now(target);
}

uint16_t dcs_data_r(int *empty)
{
	dcs_sync_k(SY_DATA);
	*empty = IS_OUTPUT_EMPTY() != 0;
	SET_OUTPUT_EMPTY();   // (auto ack)
	return D.output_data;
}

int dcs_control_r(void)
{
	dcs_sync_k(SY_CONTROL);
	return D.latch_control;
}

static void run_now(uint64_t target)
{
	for (;;) {
		uint64_t now = adsp_total_cycles();
		if (D.reg_on && ((D.reg_at + 0xffff) >> 16) <= now) {
			D.reg_at += D.reg_period;
			dcs_irq();
			continue;
		}
		if (D.int_on && D.int_at <= now) {
			internal_timer_callback();
			continue;
		}
		if (now >= target)
			break;
		uint64_t lim = target;
		if (D.reg_on && ((D.reg_at + 0xffff) >> 16) < lim)
			lim = (D.reg_at + 0xffff) >> 16;
		if (D.int_on && D.int_at < lim)
			lim = D.int_at;
		if (D.halted)
			adsp_skip((int)(lim - now));
		else
			adsp_execute((int)(lim - now));
		if (D.irq1_clear) {
			adsp_set_input(ADSP2105_IRQ1, 0);
			D.irq1_clear = 0;
		}
		if (D.cpu_reset_pending) {
			D.cpu_reset_pending = 0;
			adsp_reset();
		}
	}
	// the DAC's stream: silence when its buffer is empty
	while (D.stream_at <= target) {
		int16_t s = 0;
		if (D.bufout != D.bufin) {
			s = s_fifo[D.bufout];
			D.bufout = (D.bufout + 1) % FIFO_SIZE;
		}
		rbx_audio_put(s);
		D.stream_at += D.cps;
	}
}

int dcs_state(void *ctx, int load)
{
	dcs_sync();
	if (rbx_state_io(ctx, "dcsD", &D, sizeof(D)) || rbx_state_io(ctx, "dcspi", s_pint, 0x400 * 4) ||
		rbx_state_io(ctx, "dcspe", s_pext, s_pext_words * 4) || rbx_state_io(ctx, "dcsdl", s_dlow, 0x800 * 2) ||
		rbx_state_io(ctx, "dcsdi", s_diram, 0x400 * 2) || rbx_state_io(ctx, "dcsfifo", s_fifo, FIFO_SIZE * 2) ||
		adsp_state(ctx, load))
		return -1;
	if (load) {
		set_data_bank();
		if (D.cps)
			rbx_audio.rate = (DCS_CLOCK + D.cps / 2) / D.cps;
	}
	return 0;
}
