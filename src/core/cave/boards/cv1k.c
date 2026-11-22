/*  Cave CV1000-B / CV1000-D (Mushihime-Sama, Ibara, Espgaluda II, Mushihime-Tama, Pink Sweets, Mushihime-Sama
    Futari, Muchi Muchi Pork!, Deathsmiles, DoDonPachi Dai-Fukkatsu ...). Ported from MAME 0.289
    src/mame/cave/cv1k.cpp (BSD-3-Clause, copyright-holders: David Haywood, Luca Elia, MetalliC), with the Samsung
    K9F1G08U0M NAND of src/devices/machine/nandflash.cpp (BSD-3-Clause, copyright-holders: Raphael Nabet)
    and the Epson RTC-9701 of src/devices/machine/rtc9701.cpp (BSD-3-Clause, copyright-holders: Angelo Salese, David
    Haywood).
    An SH-3 (SH7709S) at 102.4 MHz, 8 MB (CV1000-D: 16 MB) of work RAM, the program in a 4 MB flash, the graphics
    in a 132 MB NAND read into RAM by the program and uploaded to the blitter's 8192 x 4096 VRAM, a YMZ770C.
    The screen is the blitter's VRAM seen through its scroll registers, 320 x 240 at 60.024 Hz (512 lines, the
    vblank interrupt (IRQ2, held) at line 240).
    The NAND is read from the ROM file kept outside memory (rbx_driver.file_regions, through a cache of 8-page lines);
    its pages written stay in memory (MAME keeps them in its nvram); the RTC's EEPROM is kept in <set>.nv; the clock
    starts at 2000-01-01 00:00:00 (MAME: the host's time). The first vblank comes one frame after the reset (MAME:
    set_vblank_time(0)).
    MAME's speed-up (a read of the idle loop's variable spins the CPU for 10 us) is kept: it sets the timing.
    The idle loop's and the blitter status polling's rounds that would come back the same are left out by the CPU
    core with the results of running them (sh3_spin_round, sh3.c idle rounds; RBX_CV1K_NOIDLE=1 runs them, ADR-044).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/sh3/sh3.h"
#include "../video/epic12.h"
#include "../sound/ymz770.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_HZ 102400000u
#define VIS_W 320
#define VIS_H 240
#define LINES 512
// a line in CPU clocks: CPU_HZ / (60.024 * 512) = LINE_NUM / LINE_DEN
#define LINE_NUM 102400000000ull
#define LINE_DEN 30732288ull
#define SND_CYC (CPU_HZ / YMZ770_RATE)   // 6400 clocks a sample
#define SND_CAP 2048
#define FLASH_PAGE (2048 + 64)

enum { SM_M_INIT, SM_M_READ, SM_M_PROGRAM, SM_M_ERASE, SM_M_READSTATUS, SM_M_READID, SM_M_30, SM_M_RANDOM_DATA_INPUT,
       SM_M_RANDOM_DATA_OUTPUT };
enum { SM_PM_A, SM_PM_B, SM_PM_C };

static struct cv1k_state {
	// NAND (nand_device)
	int32_t nd_mode, nd_pointer_mode, nd_addr_load_ptr, nd_program_byte_count, nd_mode_3065;
	uint32_t nd_page_addr, nd_byte_addr, nd_status, nd_accumulated_status;
	uint8_t nd_pagereg[FLASH_PAGE];
	// rtc9701
	int32_t rtc_latch, rtc_reset_line, rtc_clock_line, rtc_state, rtc_cmd_pos, rtc_cur_cmd;
	int32_t rtc_addr_pos, rtc_cur_addr, rtc_cur_data, rtc_data_pos;
	uint16_t rtc_data[0x100];
	uint8_t rtc_sec, rtc_min, rtc_hour, rtc_day, rtc_wday, rtc_month, rtc_year, pad0;
	int64_t rtc_next;       // the clock of the next second
	// the EEPROMOUT port's line writers (MAME dynamic_field m_oldval)
	uint8_t eo_old[3], pad1;
	// timing
	uint64_t line_frac;
	int64_t line_end;
	int64_t snd_pos;        // samples made since reset
	epic12_t blit;
	ymz770_t ymz;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

// the sampled activity (rbx_prof.h, ADR-155): the driver's ids (EPIC12_PA_LIST / SYNC among them, epic12.h)
enum {
	CV_PA_FRAME = RBX_PA_DRV,   // the frame's own work (cv1k_frame, run_lines)
	CV_PA_BLIT_LIST_ = EPIC12_PA_LIST,
	CV_PA_BLIT_SYNC_ = EPIC12_PA_SYNC,
	CV_PA_VBLANK,               // the vblank: a free snapshot, the screen copy queued (or waited for and copied)
	CV_PA_YMZ,                  // the YMZ770's writes and sample runs queued (or made here)
	CV_PA_YMZ_SYNC,             // the frame end: the YMZ770 queue's rest done here
	CV_PA_NAND,                 // the NAND flash: commands, data reads (the DMA's block reads)
	CV_PA_IO,                   // the other board reads / writes: RTC, EEPROM, ports, ROM writes
	CV_PA_RAM,                  // the idle variable's page through the handlers
	CV_PA_IDLE,                 // the idle loop's read: the speed-up check (sh3_spin_round's own mark inside)
	CV_PA_BLIT_REG,             // the blitter's register reads (the busy flag)
	CV_PA_KICK,                 // the workers woken (the blitter lanes, the sound worker)
	CV_PA_AUDIO,                // the frame's samples to the core
	CV_PA_END
};
static const char *const s_cv_pa_names[CV_PA_END - RBX_PA_DRV] = {
	"cv1k frame", "blit list", "blit sync", "vblank", "ymz", "ymz sync", "nand", "cv1k io", "ram page", "idle check",
	"blit reg", "kick", "audio put",
};

static sh3_t *s_cpu;
static uint8_t *s_ram;          // work RAM (host-order 32-bit words)
static uint32_t s_ram_size;
static uint16_t *s_vram;
static uint16_t *s_snap;        // the screen at vblank (ARGB1555), two (ADR-150: slot rbx_video_post_seq() & 1)
static int16_t *s_snd;
static int s_snd_n;
static uint32_t s_rows;          // NAND pages
static uint8_t **s_nand_w;       // the pages written (programmed or erased): their own copy, NULL = as in the ROM file
static const uint8_t *s_pg;      // the page last looked up
static uint32_t s_pg_no;
static uint32_t s_idle_off, s_idle_pc;
static int s_port_c, s_port_d, s_port_f, s_port_l, s_port_dsw;
static char s_nv_path[300];
static int s_565;
static int s_trace;
static int s_tr_from, s_tr_to, s_frame;
static uint32_t s_vrz_n;     // the packed VRAM size of frame s_vrz_frame (-1: none)
static int s_vrz_frame = -1;
static void tr_cb(sh3_t *c, uint32_t pc) { rbx_log("%lld %08X\n", (long long)sh3_total_cycles(c), pc); }

// RBX_CV1K_PROF=frame: from that frame on, the CPU clocks and runs of each pc and the board reads by pc (the
// interpreter, traced; printed at exit; RBX_CV1K_PROFDUMP=addr,len: the code there too)
static int s_prof_from;
#define PROF_N (1u << 16)
static struct prof_e { uint32_t pc, n, rd; uint64_t cyc; } *s_prof;
static uint32_t s_prof_pc;
static int64_t s_prof_t;
static struct prof_e *prof_at(uint32_t pc)
{
	uint32_t h = (pc * 2654435761u) >> 16;
	while (s_prof[h].pc != pc && s_prof[h].pc)
		h = (h + 1) & (PROF_N - 1);
	s_prof[h].pc = pc;
	return &s_prof[h];
}
static void prof_cb(sh3_t *c, uint32_t pc)
{
	const int64_t t = sh3_total_cycles(c);
	if (s_prof_pc)
		prof_at(s_prof_pc)->cyc += (uint64_t)(t - s_prof_t);
	prof_at(pc)->n++;
	s_prof_pc = pc;
	s_prof_t = t;
}
static void prof_report(void)
{
	if (!s_prof)
		return;
	uint64_t tot = 0;
	for (uint32_t i = 0; i < PROF_N; i++)
		tot += s_prof[i].cyc;
	rbx_log("cv1k prof: %llu cycles\n", (unsigned long long)tot);
	for (int k = 0; k < 60; k++) {
		uint32_t best = 0;
		for (uint32_t i = 1; i < PROF_N; i++)
			if (s_prof[i].cyc > s_prof[best].cyc)
				best = i;
		if (!s_prof[best].cyc)
			break;
		rbx_log("  %08x %04x %5.2f%% n %u cyc %llu\n", s_prof[best].pc, sh3_read16(s_cpu, s_prof[best].pc),
		        s_prof[best].cyc * 100.0 / (double)tot, s_prof[best].n, (unsigned long long)s_prof[best].cyc);
		s_prof[best].cyc = 0;
	}
	for (int k = 0; k < 20; k++) {
		uint32_t best = 0;
		for (uint32_t i = 1; i < PROF_N; i++)
			if (s_prof[i].rd > s_prof[best].rd)
				best = i;
		if (!s_prof[best].rd)
			break;
		rbx_log("  board reads at pc %08x: %u\n", s_prof[best].pc, s_prof[best].rd);
		s_prof[best].rd = 0;
	}
	if (getenv("RBX_CV1K_PROFDUMP")) {
		unsigned pa = 0, pl = 0;
		sscanf(getenv("RBX_CV1K_PROFDUMP"), "%x,%x", &pa, &pl);
		for (unsigned k = 0; k < pl; k += 2)
			rbx_log("  op %08x %04x\n", pa + k, sh3_read16(s_cpu, pa + k));
	}
	free(s_prof);
	s_prof = NULL;
}

/***************************************************************************
    NAND (MAME nand_device as the Samsung K9F1G08U0M: 2048 + 64 byte pages, 64 pages a block, 2 + 2 address cycles)
***************************************************************************/

#define NAND_PAGES (64 * 1024)
#define NC_PAGES 8   // a cache chunk: 8 pages (16.5 KB), one card read
#define NC_PEND 64   // chunks being read ahead at most

/* The NAND read cache (ADR-160): s_nc_n slots of a chunk each, any chunk in any slot (s_nc_map: chunk -> slot + 1),
   the least recently used one given up. The game reads the NAND almost only forwards (mushisam, 3600 frames: of 23.5k
   page lookups 23.4k the page after the last one, 0.4% of the pages looked up again), so the size does little; reading
   ahead does: with the io worker on (rbx_io_on), each new chunk looked up queues the next s_nc_ahead chunks not cached,
   read by the worker into their slots. A chunk needed while still queued is taken back and read here, one being read is
   waited for: the bytes are the file's either way. RBX_NAND_SLOTS, RBX_NAND_AHEAD (0: none ahead), RBX_NAND_OLD=1: 64
   slots, none ahead (the cache before ADR-160 was direct-mapped: 0.2% more reads) */
enum { NC_FREE, NC_READY, NC_PENDING };
typedef struct { uint32_t chunk, use; int16_t req; uint8_t state; } nc_slot;
static uint8_t *s_nc;           // the slots' bytes
static nc_slot *s_ncs;
static uint32_t *s_nc_map;      // chunk -> slot + 1 (0: not cached)
static uint32_t s_nc_n, s_nc_chunks, s_nc_ahead, s_nc_use, s_nc_last;   // (s_nc_last: the chunk last looked up + 1)
static uint32_t s_nc_pend[NC_PEND], s_nc_npend;   // the slots being read ahead

// (testing: RBX_NAND_STAT=1 the card reads per frame at exit; RBX_NAND_TRACE=file the pages looked up, as frame and page
// 32-bit pairs, for trying cache shapes off line)
static struct nand_stat { int on; FILE *tr; uint64_t miss, bytes, ahead, late, dropped, here_us; uint32_t fmiss, fmax, fmaxb, fbytes, frames_hit, nfr; } s_nst;
static FILE *s_nst_ff;   // (RBX_NAND_FRAMES=file: a line a frame with reads: frame, card reads here, their bytes, waits)
static uint64_t s_nst_late0;
static void nand_stat_frame(void)
{
	if (!s_nst.on)
		return;
	if (s_nst_ff && (s_nst.fmiss || s_nst.late != s_nst_late0))
		fprintf(s_nst_ff, "%u %u %u %llu\n", s_nst.nfr, s_nst.fmiss, s_nst.fbytes, (unsigned long long)(s_nst.late - s_nst_late0));
	s_nst_late0 = s_nst.late;
	s_nst.nfr++;
	if (s_nst.fmiss)
		s_nst.frames_hit++;
	if (s_nst.fmiss > s_nst.fmax)
		s_nst.fmax = s_nst.fmiss;
	if (s_nst.fbytes > s_nst.fmaxb)
		s_nst.fmaxb = s_nst.fbytes;
	s_nst.fmiss = s_nst.fbytes = 0;
}

static uint32_t nc_pages(uint32_t c) { return s_rows - c * NC_PAGES < NC_PAGES ? s_rows - c * NC_PAGES : NC_PAGES; }
static uint8_t *nc_data(uint32_t s) { return s_nc + (size_t)s * NC_PAGES * FLASH_PAGE; }

static void nc_read_here(uint32_t s, uint32_t c)   // (the emulation thread's card read, as before ADR-160)
{
	const uint32_t n = nc_pages(c);
	const uint64_t t0 = s_nst.on ? rbx_clock_us() : 0;
	if (rbx_region_read("nand", c * NC_PAGES * FLASH_PAGE, nc_data(s), n * FLASH_PAGE))
		memset(nc_data(s), 0xff, (size_t)n * FLASH_PAGE);
	if (s_nst.on)
		s_nst.here_us += rbx_clock_us() - t0;
	s_nst.miss++, s_nst.fmiss++, s_nst.bytes += n * FLASH_PAGE, s_nst.fbytes += n * FLASH_PAGE;
}

static void nc_unpend(uint32_t s)
{
	for (uint32_t i = 0; i < s_nc_npend; i++)
		if (s_nc_pend[i] == s) {
			s_nc_pend[i] = s_nc_pend[--s_nc_npend];
			return;
		}
}

// a slot read ahead, its bytes in place: taken back and read here if the worker has not started it, waited for if it
// is reading it (a failed read: read here again, 0xff if that fails too, as before)
static void nc_settle(uint32_t s)
{
	nc_slot *e = &s_ncs[s];
	if (s_nst.on) {   // (statistics: not started is a card read here, started a wait)
		if (rbx_io_queued(e->req)) {
			const uint32_t n = nc_pages(e->chunk) * FLASH_PAGE;
			s_nst.miss++, s_nst.fmiss++, s_nst.bytes += n, s_nst.fbytes += n;
		} else if (!rbx_io_ready(e->req))
			s_nst.late++;
	}
	const uint64_t t0 = s_nst.on ? rbx_clock_us() : 0;
	const int rc = rbx_io_finish(e->req);
	if (s_nst.on)
		s_nst.here_us += rbx_clock_us() - t0;
	if (rc)
		nc_read_here(s, e->chunk);
	e->req = -1;
	e->state = NC_READY;
	nc_unpend(s);
}

// the reads ahead the worker finished made ready; with the worker gone, the ones it never started given up
static void nc_reap(void)
{
	const int on = rbx_io_on();
	for (uint32_t i = s_nc_npend; i-- > 0;) {
		const uint32_t s = s_nc_pend[i];
		nc_slot *e = &s_ncs[s];
		if (rbx_io_ready(e->req))
			nc_settle(s);
		else if (!on && rbx_io_queued(e->req)) {
			rbx_io_drop(e->req);
			s_nc_map[e->chunk] = 0;
			e->req = -1;
			e->state = NC_FREE;
			nc_unpend(s);
		}
	}
}

// a slot to fill: a free one, else the least recently used ready one but keep (given up); ~0u: none
static uint32_t nc_victim(uint32_t keep)
{
	uint32_t best = ~0u, bu = 0;
	for (uint32_t s = 0; s < s_nc_n; s++) {
		const nc_slot *e = &s_ncs[s];
		if (e->state == NC_FREE)
			return s;
		if (e->state == NC_READY && s != keep && (best == ~0u || (int32_t)(e->use - bu) < 0))
			best = s, bu = e->use;
	}
	if (best != ~0u) {
		const uint32_t c = s_ncs[best].chunk;
		s_nc_map[c] = 0;
		s_ncs[best].state = NC_FREE;
		if (s_pg_no != ~0u && s_pg_no / NC_PAGES == c && !s_nand_w[s_pg_no])
			s_pg_no = ~0u;   // (the page last looked up was there)
	}
	return best;
}

// the chunks after c not cached queued for the worker (c's slot kept)
static void nc_ahead(uint32_t c, uint32_t keep)
{
	nc_reap();
	if (!rbx_io_on())
		return;
	for (uint32_t k = 1; k <= s_nc_ahead && c + k < s_nc_chunks; k++) {
		const uint32_t cc = c + k;
		if (s_nc_map[cc])
			continue;
		if (s_nc_npend >= NC_PEND)
			break;
		const uint32_t s = nc_victim(keep);
		if (s == ~0u)
			break;
		const int id = rbx_io_submit("nand", cc * NC_PAGES * FLASH_PAGE, nc_data(s), nc_pages(cc) * FLASH_PAGE);
		if (id < 0)
			break;   // (the slot stays free)
		s_ncs[s].chunk = cc;
		s_ncs[s].use = s_nc_use;
		s_ncs[s].req = (int16_t)id;
		s_ncs[s].state = NC_PENDING;
		s_nc_map[cc] = s + 1;
		s_nc_pend[s_nc_npend++] = s;
		s_nst.ahead++;
	}
}

/* (ADR-177) A jump (a chunk looked up that is not the last one's next): the reads queued ahead of the old place that
   the worker has not started are given up (the game reads runs of about 63 pages, 8 chunks, then jumps: the 16 queued
   ahead of a run are mostly not needed), so the worker goes on to the new place at once. Those in the new place's
   window stay. Only their caching changes: a chunk given up is read again when it is looked up. RBX_NAND_DROP=0: off */
static int s_nc_drop;
static void nc_drop_stale(uint32_t c)
{
	for (uint32_t i = s_nc_npend; i-- > 0;) {
		const uint32_t s = s_nc_pend[i];
		nc_slot *e = &s_ncs[s];
		if (e->chunk > c && e->chunk <= c + s_nc_ahead)
			continue;
		if (!rbx_io_cancel(e->req))
			continue;   // (started, or done: kept)
		s_nc_map[e->chunk] = 0;
		e->req = -1;
		e->state = NC_FREE;
		nc_unpend(s);
		s_nst.dropped++;
	}
}

static const uint8_t *nand_page(uint32_t p)   // (p < s_rows)
{
	if (p == s_pg_no)
		return s_pg;
	const uint8_t *r = s_nand_w[p];
	if (!r && s_nst.tr) {
		const uint32_t e[2] = { (uint32_t)s_frame, p };
		fwrite(e, sizeof(e), 1, s_nst.tr);
	}
	if (!r) {
		const uint32_t c = p / NC_PAGES;
		uint32_t s = s_nc_map[c];
		if (s) {
			s--;
			if (s_ncs[s].state == NC_PENDING)
				nc_settle(s);
		} else {
			s = nc_victim(~0u);   // (always one: at most NC_PEND of more slots are pending)
			nc_read_here(s, c);
			s_ncs[s].chunk = c;
			s_ncs[s].req = -1;
			s_ncs[s].state = NC_READY;
			s_nc_map[c] = s + 1;
		}
		s_ncs[s].use = ++s_nc_use;
		r = nc_data(s) + (p % NC_PAGES) * FLASH_PAGE;
		if (c + 1 != s_nc_last) {
			if (s_nc_drop && c != s_nc_last && s_nc_npend)   // (a jump: ADR-177)
				nc_drop_stale(c);
			s_nc_last = c + 1;
			if (s_nc_ahead)
				nc_ahead(c, s);
		}
	}
	s_pg_no = p;
	s_pg = r;
	return r;
}

static uint8_t *nand_page_w(uint32_t p)   // a page to be written (its own copy; NULL: out of memory)
{
	if (!s_nand_w[p]) {
		uint8_t *c = (uint8_t *)malloc(FLASH_PAGE);
		if (c)
			memcpy(c, nand_page(p), FLASH_PAGE);
		s_nand_w[p] = c;
	}
	s_pg_no = ~0u;
	return s_nand_w[p];
}

static void nand_free(void)
{
	for (uint32_t i = 0; i < s_nc_npend; i++)   // (the reads ahead given up: the worker no longer writes there)
		rbx_io_drop(s_ncs[s_nc_pend[i]].req);
	s_nc_npend = 0;
	for (uint32_t p = 0; s_nand_w && p < s_rows; p++)
		free(s_nand_w[p]);
	free(s_nand_w);
	s_nand_w = NULL;
	free(s_nc);
	s_nc = NULL;
	free(s_ncs);
	s_ncs = NULL;
	free(s_nc_map);
	s_nc_map = NULL;
}

static void nand_reset(void)
{
	b.nd_mode = SM_M_INIT;
	b.nd_pointer_mode = SM_PM_A;
	b.nd_page_addr = 0;
	b.nd_byte_addr = 0;
	b.nd_accumulated_status = 0;
	b.nd_mode_3065 = 0;
	b.nd_status = 0xc0;
	memset(b.nd_pagereg, 0, sizeof(b.nd_pagereg));
}

static void nand_command_w(uint8_t data)
{
	switch (data) {
	case 0xff:   // reset
		b.nd_mode = SM_M_INIT;
		b.nd_pointer_mode = SM_PM_A;
		b.nd_status = (b.nd_status & 0x80) | 0x40;
		b.nd_accumulated_status = 0;
		b.nd_mode_3065 = 0;
		break;
	case 0x00:   // read (1st cycle)
		b.nd_mode = SM_M_READ;
		b.nd_pointer_mode = SM_PM_A;
		b.nd_addr_load_ptr = 0;
		break;
	case 0x01:
	case 0x50:   // (512-byte page chips only)
		b.nd_mode = SM_M_INIT;
		break;
	case 0x80:   // page program (1st cycle)
		b.nd_mode = SM_M_PROGRAM;
		b.nd_addr_load_ptr = 0;
		b.nd_program_byte_count = 0;
		memset(b.nd_pagereg, 0xff, FLASH_PAGE);
		break;
	case 0x10:   // page program (2nd cycle)
	case 0x15:
		if (b.nd_mode != SM_M_PROGRAM && b.nd_mode != SM_M_RANDOM_DATA_INPUT)
			b.nd_mode = SM_M_INIT;
		else {
			b.nd_status = (b.nd_status & 0x80) | b.nd_accumulated_status;
			uint8_t *w = b.nd_page_addr < s_rows ? nand_page_w(b.nd_page_addr) : NULL;
			for (int i = 0; w && i < FLASH_PAGE; i++)
				w[i] &= b.nd_pagereg[i];
			b.nd_status |= 0x40;
			b.nd_accumulated_status = data == 0x15 ? (b.nd_status & 0x1f) : 0;
			b.nd_mode = SM_M_INIT;
		}
		break;
	case 0x60:   // block erase (1st cycle)
		b.nd_mode = SM_M_ERASE;
		b.nd_page_addr = 0;
		b.nd_addr_load_ptr = 0;
		break;
	case 0xd0:   // block erase (2nd cycle)
		if (b.nd_mode != SM_M_ERASE)
			b.nd_mode = SM_M_INIT;
		else {
			b.nd_status &= 0x80;
			const uint32_t first = b.nd_page_addr & ~63u;
			for (uint32_t p = first; p < first + 64 && p < s_rows; p++) {
				uint8_t *w = nand_page_w(p);
				if (w)
					memset(w, 0xff, FLASH_PAGE);
			}
			b.nd_status |= 0x40;
			b.nd_mode = SM_M_INIT;
			if (b.nd_pointer_mode == SM_PM_B)
				b.nd_pointer_mode = SM_PM_A;
		}
		break;
	case 0x70: b.nd_mode = SM_M_READSTATUS; break;
	case 0x90: b.nd_mode = SM_M_READID; b.nd_addr_load_ptr = 0; break;
	case 0x30:   // read (2nd cycle)
		if (b.nd_mode != SM_M_READ || b.nd_addr_load_ptr < 4)
			b.nd_mode = SM_M_INIT;
		break;
	case 0x65:
		if (b.nd_mode != SM_M_30) b.nd_mode = SM_M_INIT;
		else b.nd_mode_3065 = 1;
		break;
	case 0x05:   // random data output (1st cycle)
		if (b.nd_mode != SM_M_READ && b.nd_mode != SM_M_RANDOM_DATA_OUTPUT)
			b.nd_mode = SM_M_INIT;
		else {
			b.nd_mode = SM_M_RANDOM_DATA_OUTPUT;
			b.nd_addr_load_ptr = 0;
		}
		break;
	case 0xe0:
		if (b.nd_mode != SM_M_RANDOM_DATA_OUTPUT) b.nd_mode = SM_M_INIT;
		break;
	case 0x85:   // random data input
		if (b.nd_mode != SM_M_PROGRAM && b.nd_mode != SM_M_RANDOM_DATA_INPUT)
			b.nd_mode = SM_M_INIT;
		else {
			b.nd_mode = SM_M_RANDOM_DATA_INPUT;
			b.nd_addr_load_ptr = 0;
			b.nd_program_byte_count = 0;
		}
		break;
	default:
		b.nd_mode = SM_M_INIT;
		break;
	}
}

static void nand_address_w(uint8_t data)
{
	switch (b.nd_mode) {
	case SM_M_READ:
	case SM_M_PROGRAM:
		if (b.nd_addr_load_ptr == 0)
			b.nd_page_addr = 0;
		if (b.nd_addr_load_ptr < 2) {
			b.nd_byte_addr &= ~(0xffu << (b.nd_addr_load_ptr * 8));
			b.nd_byte_addr |= (uint32_t)data << (b.nd_addr_load_ptr * 8);
		} else if (b.nd_addr_load_ptr < 4) {
			b.nd_page_addr &= ~(0xffu << ((b.nd_addr_load_ptr - 2) * 8));
			b.nd_page_addr |= (uint32_t)data << ((b.nd_addr_load_ptr - 2) * 8);
		}
		b.nd_addr_load_ptr++;
		break;
	case SM_M_ERASE:
		if (b.nd_addr_load_ptr < 2) {
			b.nd_page_addr &= ~(0xffu << (b.nd_addr_load_ptr * 8));
			b.nd_page_addr |= (uint32_t)data << (b.nd_addr_load_ptr * 8);
		}
		b.nd_addr_load_ptr++;
		break;
	case SM_M_RANDOM_DATA_INPUT:
	case SM_M_RANDOM_DATA_OUTPUT:
		if (b.nd_addr_load_ptr < 2) {
			b.nd_byte_addr &= ~(0xffu << (b.nd_addr_load_ptr * 8));
			b.nd_byte_addr |= (uint32_t)data << (b.nd_addr_load_ptr * 8);
		}
		b.nd_addr_load_ptr++;
		break;
	case SM_M_READID:
		if (b.nd_addr_load_ptr == 0)
			b.nd_byte_addr = data;
		b.nd_addr_load_ptr++;
		break;
	default:
		break;
	}
}

static uint8_t nand_data_r(void)
{
	static const uint8_t id[4] = { 0xec, 0xf1, 0x00, 0x15 };
	uint8_t reply = 0;
	switch (b.nd_mode) {
	case SM_M_READ:
	case SM_M_RANDOM_DATA_OUTPUT:
		if (!b.nd_mode_3065) {
			if (b.nd_byte_addr < FLASH_PAGE)
				reply = b.nd_page_addr < s_rows ? nand_page(b.nd_page_addr)[b.nd_byte_addr] : 0xff;
			else
				reply = 0xff;
		} else
			reply = 0xff;
		b.nd_byte_addr++;
		break;
	case SM_M_READSTATUS:
		reply = b.nd_status & 0xc1;
		break;
	case SM_M_READID:
		reply = b.nd_byte_addr < 4 ? id[b.nd_byte_addr] : 0;
		b.nd_byte_addr++;
		break;
	default:
		break;
	}
	return reply;
}

// n reads of the data port in a row (sh3.c read_block8, the DMA's NAND to RAM copies): the page read modes only
static uint32_t nand_data_block(uint8_t *buf, uint32_t n)
{
	if ((b.nd_mode != SM_M_READ && b.nd_mode != SM_M_RANDOM_DATA_OUTPUT) || b.nd_mode_3065)
		return 0;
	uint32_t i = 0;
	if (b.nd_byte_addr < FLASH_PAGE && b.nd_page_addr < s_rows) {
		i = FLASH_PAGE - b.nd_byte_addr < n ? FLASH_PAGE - b.nd_byte_addr : n;
		memcpy(buf, nand_page(b.nd_page_addr) + b.nd_byte_addr, i);
	}
	memset(buf + i, 0xff, n - i);
	b.nd_byte_addr += n;
	return n;
}

static void nand_data_w(uint8_t data)
{
	if (b.nd_mode == SM_M_PROGRAM || b.nd_mode == SM_M_RANDOM_DATA_INPUT) {
		if (b.nd_program_byte_count++ < FLASH_PAGE)
			b.nd_pagereg[b.nd_byte_addr % FLASH_PAGE] = data;
		b.nd_byte_addr++;
		if (b.nd_byte_addr == FLASH_PAGE)
			b.nd_byte_addr = b.nd_pointer_mode != SM_PM_C ? 0 : 2048;
	}
}

/***************************************************************************
    RTC-9701 (serial RTC + EEPROM)
***************************************************************************/

enum { RTC_CMD_WAIT, RTC_WRITE, RTC_READ, EEPROM_READ, EEPROM_WRITE, AFTER_WRITE_ENABLE };

static void rtc_tick(void)
{
	static const uint8_t dpm[12] = { 0x31, 0x28, 0x31, 0x30, 0x31, 0x30, 0x31, 0x31, 0x30, 0x31, 0x30, 0x31 };
	b.rtc_sec++;
	if ((b.rtc_sec & 0x0f) >= 0x0a) { b.rtc_sec += 0x10; b.rtc_sec &= 0xf0; }
	if ((b.rtc_sec & 0xf0) >= 0x60) { b.rtc_min++; b.rtc_sec = 0; }
	if ((b.rtc_min & 0x0f) >= 0x0a) { b.rtc_min += 0x10; b.rtc_min &= 0xf0; }
	if ((b.rtc_min & 0xf0) >= 0x60) { b.rtc_hour++; b.rtc_min = 0; }
	if ((b.rtc_hour & 0x0f) >= 0x0a) { b.rtc_hour += 0x10; b.rtc_hour &= 0xf0; }
	if ((b.rtc_hour & 0xff) >= 0x24) { b.rtc_day++; b.rtc_wday <<= 1; b.rtc_hour = 0; }
	if (b.rtc_wday & 0x80) b.rtc_wday = 1;
	if ((b.rtc_day & 0x0f) >= 0x0a) { b.rtc_day += 0x10; b.rtc_day &= 0xf0; }
	int dpm_count = (b.rtc_month & 0xf) + (((b.rtc_month & 0x10) >> 4) * 10) - 1;
	if (dpm_count < 0) dpm_count = 0;
	if (dpm_count > 11) dpm_count = 11;
	if (((b.rtc_year % 4) == 0) && b.rtc_month == 2) {
		if ((b.rtc_day & 0xff) >= dpm[dpm_count] + 1 + 1) { b.rtc_month++; b.rtc_day = 0x01; }
	} else if ((b.rtc_day & 0xff) >= dpm[dpm_count] + 1) { b.rtc_month++; b.rtc_day = 0x01; }
	if ((b.rtc_month & 0x0f) >= 0x0a) b.rtc_month = 0x10;
	if (b.rtc_month >= 0x13) { b.rtc_year++; b.rtc_month = 1; }
	if ((b.rtc_year & 0x0f) >= 0x0a) { b.rtc_year += 0x10; b.rtc_year &= 0xf0; }
	if ((b.rtc_year & 0xf0) >= 0xa0) b.rtc_year = 0;
}

static void rtc_catch_up(int64_t now)
{
	while (b.rtc_next <= now) {
		rtc_tick();
		b.rtc_next += CPU_HZ;
	}
}

static uint8_t rtc_read(uint8_t offset)
{
	switch (offset) {
	case 0: return b.rtc_sec;
	case 1: return b.rtc_min;
	case 2: return b.rtc_hour;
	case 3: return b.rtc_wday;
	case 4: return b.rtc_day;
	case 5: return b.rtc_month;
	case 6: return b.rtc_year;
	case 7: return 0x20;
	default: return 0;
	}
}

static void rtc_write(uint8_t offset, uint8_t data)
{
	switch (offset) {
	case 0: b.rtc_sec = data; break;
	case 1: b.rtc_min = data; break;
	case 2: b.rtc_hour = data; break;
	case 3: b.rtc_wday = data; break;
	case 4: b.rtc_day = data; break;
	case 5: b.rtc_month = data; break;
	case 6: b.rtc_year = data; break;
	default: break;
	}
}

static int rtc_read_bit(void)
{
	if (b.rtc_state == RTC_READ || b.rtc_state == EEPROM_READ)
		return (b.rtc_cur_data >> (b.rtc_data_pos - 1)) & 1;
	return 0;
}

static void rtc_set_cs_line(int state)
{
	b.rtc_reset_line = state;
	if (state) {
		b.rtc_state = RTC_CMD_WAIT;
		b.rtc_cmd_pos = 0;
		b.rtc_cur_cmd = 0;
		b.rtc_addr_pos = 0;
		b.rtc_cur_addr = 0;
		b.rtc_cur_data = 0;
		b.rtc_data_pos = 0;
	}
}

static void rtc_begin(int st)
{
	b.rtc_state = st;
	b.rtc_cmd_pos = 0;
	b.rtc_addr_pos = 0;
	b.rtc_cur_addr = 0;
	b.rtc_data_pos = 0;
	b.rtc_cur_data = 0;
}

static void rtc_set_clock_line(int state)
{
	if (b.rtc_reset_line || state != 1)
		return;
	switch (b.rtc_state) {
	case RTC_CMD_WAIT:
		b.rtc_cur_cmd = (b.rtc_cur_cmd << 1) | (b.rtc_latch & 1);
		b.rtc_cmd_pos++;
		if (b.rtc_cmd_pos == 4) {
			b.rtc_cmd_pos = 0;
			if (b.rtc_cur_cmd == 0x00) rtc_begin(RTC_WRITE);
			else if (b.rtc_cur_cmd == 0x02) rtc_begin(EEPROM_WRITE);
			else if (b.rtc_cur_cmd == 0x06) { b.rtc_state = AFTER_WRITE_ENABLE; b.rtc_cmd_pos = 0; }
			else if (b.rtc_cur_cmd == 0x08) rtc_begin(RTC_READ);
			else if (b.rtc_cur_cmd == 0x0a) rtc_begin(EEPROM_READ);
			b.rtc_cur_cmd = 0;
		}
		break;
	case AFTER_WRITE_ENABLE:
		b.rtc_cmd_pos++;
		if (b.rtc_cmd_pos == 12) {
			b.rtc_cmd_pos = 0;
			b.rtc_state = RTC_CMD_WAIT;
		}
		break;
	case RTC_WRITE:
		b.rtc_cmd_pos++;
		if (b.rtc_cmd_pos <= 4) {
			b.rtc_addr_pos++;
			b.rtc_cur_addr = (b.rtc_cur_addr << 1) | (b.rtc_latch & 1);
		}
		if (b.rtc_cmd_pos > 4) {
			b.rtc_data_pos++;
			b.rtc_cur_data = (b.rtc_cur_data << 1) | (b.rtc_latch & 1);
		}
		if (b.rtc_cmd_pos == 12) {
			b.rtc_cmd_pos = 0;
			rtc_write((uint8_t)b.rtc_cur_addr, (uint8_t)b.rtc_cur_data);
			b.rtc_state = RTC_CMD_WAIT;
		}
		break;
	case EEPROM_READ:
		b.rtc_cmd_pos++;
		if (b.rtc_cmd_pos <= 12) {
			b.rtc_addr_pos++;
			b.rtc_cur_addr = (b.rtc_cur_addr << 1) | (b.rtc_latch & 1);
			if (b.rtc_cmd_pos == 12) {
				b.rtc_cur_data = b.rtc_data[(b.rtc_cur_addr >> 1) & 0xff];
				b.rtc_data_pos = 16;
			}
		}
		if (b.rtc_cmd_pos > 12)
			b.rtc_data_pos--;
		if (b.rtc_cmd_pos == 28)
			b.rtc_cmd_pos = 0;
		break;
	case EEPROM_WRITE:
		b.rtc_cmd_pos++;
		if (b.rtc_cmd_pos <= 12) {
			b.rtc_addr_pos++;
			b.rtc_cur_addr = (b.rtc_cur_addr << 1) | (b.rtc_latch & 1);
		}
		if (b.rtc_cmd_pos > 12) {
			b.rtc_data_pos++;
			b.rtc_cur_data = (b.rtc_cur_data << 1) | (b.rtc_latch & 1);
		}
		if (b.rtc_cmd_pos == 28) {
			b.rtc_cmd_pos = 0;
			b.rtc_data[(b.rtc_cur_addr >> 1) & 0xff] = (uint16_t)b.rtc_cur_data;
			b.rtc_state = RTC_CMD_WAIT;
		}
		break;
	case RTC_READ:
		b.rtc_cmd_pos++;
		if (b.rtc_cmd_pos <= 4) {
			b.rtc_addr_pos++;
			b.rtc_cur_addr = (b.rtc_cur_addr << 1) | (b.rtc_latch & 1);
			if (b.rtc_cmd_pos == 4) {
				b.rtc_cur_data = rtc_read((uint8_t)b.rtc_cur_addr);
				b.rtc_data_pos = 8;
			}
		}
		if (b.rtc_cmd_pos > 4)
			b.rtc_data_pos--;
		if (b.rtc_cmd_pos == 12)
			b.rtc_cmd_pos = 0;
		break;
	default:
		break;
	}
}

// EEPROMOUT: bit 0 data (write_bit), bit 1 clock, bit 2 CS (active low); a line writer runs when its bit changes
static void eepromout_w(uint8_t data)
{
	const uint8_t v[3] = { (uint8_t)(data & 1), (uint8_t)((data >> 1) & 1), (uint8_t)(((data ^ 4) >> 2) & 1) };
	if (v[0] != b.eo_old[0]) { b.eo_old[0] = v[0]; b.rtc_latch = v[0]; }
	if (v[1] != b.eo_old[1]) { b.eo_old[1] = v[1]; rtc_set_clock_line(v[1]); }
	if (v[2] != b.eo_old[2]) { b.eo_old[2] = v[2]; rtc_set_cs_line(v[2]); }
}

/***************************************************************************
    sound
***************************************************************************/

// The YMZ770 on the first worker (ADR-086; RBX_CV1K_NOSNDQ=1: here). The CPU only writes it, so its writes and the
// sample runs (n samples into s_snd at a given place, as many as here) are queued in order and done by rbx_aux's job
// while the CPU runs on; the frame end, the state and the reset wait for it (snd_sync). The output is made sample by
// sample, so the runs' lengths do not change it.
// The entries are taken one at a time under a lock by the worker or by the emulation thread (ADR-095): at the frame
// end the emulation thread does what is left itself instead of waiting for a worker that may not have its core (the
// Vita: 8~28 ms waits a frame with the worker on CPU1). One entry at a time, in order, on one thread: the same output
typedef struct { int32_t n; uint16_t at; uint8_t off, data; } snd_ev_t;   // (n > 0: n samples at s_snd + at; else a write)
#define SQ_N 1024
static snd_ev_t *s_sq;
static unsigned s_sq_head, s_sq_tail;   // (head: queued by the emulation thread; tail: done)
static int s_sq_on, s_sq_lock;          // (lock: an entry being done, by either thread)
static int s_sq_take;                   // (the emulation thread is taking the rest: the worker leaves at its next entry)
// (ADR-176) the frame's last lines (s_sq_hold_lines, RBX_CV1K_SNDHOLD; 0 = off): the worker starts no entry from
// their start to the frame end, where the emulation thread does what is left. The worker's entry then ends during
// those lines instead of at the frame end: no wait there for a worker that lost its core in an entry (the Vita: the
// render thread wakes at the frame end on its core)
static int s_sq_hold, s_sq_hold_lines;
// RBX_CV1K_SNDSTAT=1: the YMZ770's work (writes and samples), the emulation thread's frame-end time for it (its own
// work there and the waits for the entry the worker is on), us
static int s_sndstat;
static uint64_t s_snd_work_us, s_snd_wait_us, s_snd_wait_max, s_snd_own_us;
static unsigned s_snd_frames;
static uint64_t s_snd_pend_e, s_snd_pend_s;   // (the entries and samples left at the frame ends)

// the queued entries in order, one at a time under the lock. The worker (own = 0) leaves when the emulation thread is
// taking them; the emulation thread (own = 1) waits for the worker's entry and goes on until the queue is empty
static void snd_drain(int own)
{
	uint64_t work = 0;
	if (own)
		__atomic_store_n(&s_sq_take, 1, __ATOMIC_SEQ_CST);
	for (int spin = 0;;) {
		if (__atomic_load_n(&s_sq_tail, __ATOMIC_ACQUIRE) == __atomic_load_n(&s_sq_head, __ATOMIC_ACQUIRE))
			break;
		if (!own && (__atomic_load_n(&s_sq_take, __ATOMIC_ACQUIRE) || __atomic_load_n(&s_sq_hold, __ATOMIC_ACQUIRE)))
			break;
		int free = 0;
		if (!__atomic_compare_exchange_n(&s_sq_lock, &free, 1, 0, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
			if (!own)
				break;
			if (++spin >= 4000)   // (the worker preempted on its entry: a sleep)
				rbx_wait();
			continue;
		}
		spin = 0;
		const uint64_t t0 = s_sndstat ? rbx_clock_us() : 0;
		unsigned t = __atomic_load_n(&s_sq_tail, __ATOMIC_RELAXED);
		if (t != __atomic_load_n(&s_sq_head, __ATOMIC_ACQUIRE)) {
			const snd_ev_t *e = &s_sq[t % SQ_N];
			if (e->n > 0)
				ymz770_render(&b.ymz, NULL, s_snd + e->at, e->n);
			else
				ymz770_write(&b.ymz, e->off, e->data);
			__atomic_store_n(&s_sq_tail, t + 1, __ATOMIC_RELEASE);
		}
		if (s_sndstat)
			work += rbx_clock_us() - t0;
		__atomic_store_n(&s_sq_lock, 0, __ATOMIC_RELEASE);
	}
	if (own)
		__atomic_store_n(&s_sq_take, 0, __ATOMIC_RELEASE);
	if (s_sndstat) {   // (the two threads' sums apart: no shared adds)
		if (own)
			s_snd_own_us += work;
		else
			s_snd_work_us += work;
	}
}

static void snd_job(void) { snd_drain(0); }

static void snd_ev(int n, uint32_t at, uint8_t off, uint8_t data)
{
	if (!s_sq_on) {
		const uint64_t t0 = s_sndstat ? rbx_clock_us() : 0;
		if (n > 0)
			ymz770_render(&b.ymz, NULL, s_snd + at, n);   // (only the right output is connected)
		else
			ymz770_write(&b.ymz, off, data);
		if (s_sndstat)
			s_snd_own_us += rbx_clock_us() - t0;
		return;
	}
	if (s_sq_head - __atomic_load_n(&s_sq_tail, __ATOMIC_ACQUIRE) >= SQ_N)
		snd_drain(1);
	snd_ev_t *e = &s_sq[s_sq_head % SQ_N];
	e->n = n;
	e->at = (uint16_t)at;
	e->off = off;
	e->data = data;
	__atomic_store_n(&s_sq_head, s_sq_head + 1, __ATOMIC_RELEASE);
}

static void snd_sync(void)
{
	if (s_sq_on)
		snd_drain(1);
}

static void snd_update(int64_t now)
{
	const int64_t want = now / SND_CYC;
	while (b.snd_pos < want) {
		int n = (int)(want - b.snd_pos);
		if (n > SND_CAP - s_snd_n) n = SND_CAP - s_snd_n;
		if (n <= 0) {   // (the buffer is full: the frame flushes it)
			b.snd_pos = want;
			break;
		}
		snd_ev(n, (uint32_t)s_snd_n, 0, 0);
		s_snd_n += n;
		b.snd_pos += n;
	}
}

/***************************************************************************
    memory
***************************************************************************/

static inline uint32_t ram_r32(uint32_t off) { return *(const uint32_t *)(s_ram + (off & ~3u)); }

static inline uint16_t ram_r16(uint32_t off) { return *(const uint16_t *)(s_ram + ((off & ~1u) ^ 2)); }

static int s_noidle;

/* The SH-3's calibrated timing model (sh3.c tpenalty, ADR-076): the clocks of a cacheable access class (r / w +
   address >> 26 in hex) per miss of the direct mapped tag array, of a not cacheable one per access, as MAME's cache
   and bus model charged them over 11400 frames of play (the harness: --auto with continues, RBX_SH3_TSTAT). By
   parent set; a set not listed takes the first. RBX_CV1K_EXACT=1: MAME's model; RBX_CV1K_TCOST="r03:26 ...": these */
static const struct { const char *set, *costs; } s_tcal[] = {
#include "cv1k_tcal.inc"
};

static void timing_setup(const rbx_game *g)
{
	if (getenv("RBX_CV1K_EXACT") && atoi(getenv("RBX_CV1K_EXACT"))) {
		sh3_set_timing(s_cpu, NULL, NULL);
		return;
	}
	const char *costs = s_tcal[0].costs, *name = g->parent ? g->parent : g->set;
	for (size_t i = 0; i < sizeof(s_tcal) / sizeof(s_tcal[0]); i++)
		if (!strcmp(s_tcal[i].set, name) || !strcmp(s_tcal[i].set, g->set))
			costs = s_tcal[i].costs;
	if (getenv("RBX_CV1K_TCOST"))
		costs = getenv("RBX_CV1K_TCOST");
	uint16_t t[2][64];
	memset(t, 0, sizeof(t));
	for (const char *p = costs; *p;) {
		char k = 0;
		unsigned cl = 0, v = 0;
		int n = 0;
		while (*p == ' ')
			p++;
		if (!*p)
			break;
		if (sscanf(p, "%c%2x:%u%n", &k, &cl, &v, &n) == 3 && (k == 'r' || k == 'w') && cl < 64)
			t[k == 'w'][cl] = (uint16_t)v;
		else
			break;
		p += n;
	}
	sh3_set_timing(s_cpu, t[0], t[1]);
}

// RBX_CV1K_LAG=1 (testing, the slowdown): the idle loop's reads that leave it (the main loop's rounds done), the
// clocks spent in it (spins), by frame; a frame with no round done is a lag frame. RBX_CV1K_LAGF=file: one line a frame
static struct { int on, from; FILE *f; uint32_t exits, reads, lag, frames, tot_exits; int64_t skip0, wait, wait_tot; } s_lag;
// RBX_CV1K_IDLEF=file (testing): the polling loops' calls by frame (sh3.c idle rounds and sh3_spin_round's counters,
// the differences), one line a frame
static FILE *s_idlef;
static uint32_t s_idlef_last[10];

static void idle_check(uint32_t a, int size, uint32_t v)
{
	// MAME speedup_r: the idle loop's read spins the CPU for 10 us
	const uint32_t pc = s_cpu->pc;
	if (pc != s_idle_pc && pc != s_idle_pc + 2)
		return;
	if (s_lag.on) {
		s_lag.reads++;
		if (!(s_cpu->r[4] > v - s_cpu->r[1]))
			s_lag.exits++;
	}
	// the loop at the idle pc: SUB R1,R0; CMP/HI R0,R4; BT/S (back to it); MOV.L @R2,R0 (this read, in the slot).
	// When it goes round again (R4 > v - R1), the round up to the next read is this read only: no write, no other
	// device (sh3.c: the rounds that would come back the same are left out)
	const uint32_t o = s_idle_pc - 0x0c000000u;
	if (!s_noidle && pc == s_idle_pc && size == 4 && o + 8 <= s_ram_size && ram_r16(o) == 0x3018 && ram_r16(o + 2) == 0x3406 &&
	    ram_r16(o + 4) == 0x8dfc && ram_r16(o + 6) == 0x6022 && (s_cpu->r[2] & 0x1fffffffu) == a &&
	    s_cpu->r[4] > v - s_cpu->r[1])
		sh3_spin_round(s_cpu, CPU_HZ / 100000, v);
	else
		sh3_spin_until(s_cpu, sh3_total_cycles(s_cpu) + CPU_HZ / 100000);
}

static uint8_t io_r8(uint32_t a)
{
	switch (a >> 20) {
	case 0x100:   // 0x10000000 flash
		return (a & 7) == 0 ? nand_data_r() : 0xff;
	case 0x10c:   // 0x10c00000 RTC
		if ((a & 7) == 1) {
			rtc_catch_up(sh3_total_cycles(s_cpu));
			return (uint8_t)(0xfe | rtc_read_bit());
		}
		return 0;
	default:
		return 0;
	}
}

static void io_w8(uint32_t a, uint8_t d)
{
	switch (a >> 20) {
	case 0x100:
		switch (a & 7) {
		case 0: nand_data_w(d); break;
		case 1: nand_command_w(d); break;
		case 2: nand_address_w(d); break;
		default: break;
		}
		break;
	case 0x104:   // 0x10400000 YMZ770
		if (a < 0x10400008) {
			if (a & 1)
				snd_update(sh3_total_cycles(s_cpu));
			snd_ev(0, 0, (uint8_t)(a & 7), d);
			if (s_trace > 2)
				rbx_log("ymz %08x %02x\n", a, d);
		}
		break;
	case 0x10c:
		if ((a & 7) == 1) {
			rtc_catch_up(sh3_total_cycles(s_cpu));
			eepromout_w(d);
		}   // (3: the NAND's CE pin)
		break;
	default:
		break;
	}
}

static int in_idle(uint32_t a, uint32_t size)
{
	const uint32_t i = 0x0c000000u + s_idle_off;
	return s_idle_off && a + size > i && a < i + 8;
}

static int in_ram(uint32_t a) { return a - 0x0c000000u < s_ram_size; }
static int in_blit(uint32_t a) { return a - 0x18000000u < 0x58; }

static uint32_t rd(uint32_t a, int size)
{
	if (s_prof && s_cpu->trace)
		prof_at(s_cpu->pc)->rd++;
	if (in_ram(a)) {   // (the idle variable's page)
		const unsigned pa = rbx_prof_enter(CV_PA_RAM);
		const uint32_t o = a - 0x0c000000u;
		const uint32_t v = size == 4 ? ram_r32(o) : size == 2 ? ram_r16(o) : s_ram[o ^ 3];
		if (in_idle(a, (uint32_t)size)) {
			rbx_prof_set(CV_PA_IDLE);
			idle_check(a, size, v);
		}
		rbx_prof_leave(pa);
		return v;
	}
	if (in_blit(a)) {
		const unsigned pa = rbx_prof_enter(CV_PA_BLIT_REG);
		const uint32_t lane = a & 3;
		const uint32_t mask = size == 4 ? 0xffffffffu : size == 2 ? (lane ? 0xffffu : 0xffff0000u) : 0xffu << ((3 - lane) * 8);
		uint32_t v = epic12_r(&b.blit, (a - 0x18000000u) >> 2, mask, sh3_total_cycles(s_cpu)) & mask;
		rbx_prof_leave(pa);
		return size == 4 ? v : size == 2 ? (lane ? v : v >> 16) : v >> ((3 - lane) * 8);
	}
	if (a >= 0x10000000u && a < 0x11000000u) {   // (8-bit devices: the bytes, big-endian)
		const unsigned pa = rbx_prof_enter((a >> 20) == 0x100 ? CV_PA_NAND : CV_PA_IO);
		uint32_t v = 0;
		for (int i = 0; i < size; i++)
			v = (v << 8) | io_r8(a + (uint32_t)i);
		rbx_prof_leave(pa);
		return v;
	}
	return 0;
}

static void wr(uint32_t a, uint32_t d, int size)
{
	if (in_ram(a)) {   // (a write through the handlers: none, the idle page's writes are direct)
		const uint32_t o = a - 0x0c000000u;
		if (size == 4) *(uint32_t *)(s_ram + (o & ~3u)) = d;
		else if (size == 2) *(uint16_t *)(s_ram + ((o & ~1u) ^ 2)) = (uint16_t)d;
		else s_ram[o ^ 3] = (uint8_t)d;
		return;
	}
	if (in_blit(a)) {
		const uint32_t lane = a & 3;
		const int sh = size == 4 ? 0 : size == 2 ? (lane ? 0 : 16) : (int)(3 - lane) * 8;
		const uint32_t mask = (size == 4 ? 0xffffffffu : size == 2 ? 0xffffu : 0xffu) << sh;
		epic12_w(&b.blit, (a - 0x18000000u) >> 2, d << sh, mask, sh3_total_cycles(s_cpu));
		return;
	}
	if (a >= 0x10000000u && a < 0x11000000u) {
		const unsigned pa = rbx_prof_enter((a >> 20) == 0x100 ? CV_PA_NAND : (a >> 20) == 0x104 ? CV_PA_YMZ : CV_PA_IO);
		for (int i = 0; i < size; i++)
			io_w8(a + (uint32_t)i, (uint8_t)(d >> (8 * (size - 1 - i))));
		rbx_prof_leave(pa);
		return;
	}
	// (ROM writes and the rest: ignored)
}

// (sh3.c idle rounds) the blitter's registers read as a function of the clock: the status is busy up to busy_until,
// then ready until the next blit (a write); the rest constants or the DIP switches (set between frames only)
static int64_t blit_until(void *u, uint32_t a, int size)
{
	(void)u; (void)size;
	const int64_t now = sh3_total_cycles(s_cpu);
	return (a & ~3u) == 0x18000010u && now < b.blit.busy_until ? b.blit.busy_until : SH3_NEVER;
}

static uint8_t r8(void *u, uint32_t a) { (void)u; return (uint8_t)rd(a, 1); }
static uint32_t rblk8(void *u, uint32_t a, uint8_t *buf, uint32_t n)   // (sh3.c read_block8: the NAND data port)
{
	(void)u;
	if ((a >> 20) != 0x100 || (a & 7) != 0 || s_prof)
		return 0;
	const unsigned pa = rbx_prof_enter(CV_PA_NAND);
	const uint32_t r = nand_data_block(buf, n);
	rbx_prof_leave(pa);
	return r;
}
static uint16_t r16(void *u, uint32_t a) { (void)u; return (uint16_t)rd(a, 2); }
static uint32_t r32(void *u, uint32_t a) { (void)u; return rd(a, 4); }
static void w8(void *u, uint32_t a, uint8_t d) { (void)u; wr(a, d, 1); }
static void w16(void *u, uint32_t a, uint16_t d) { (void)u; wr(a, d, 2); }
static void w32(void *u, uint32_t a, uint32_t d) { (void)u; wr(a, d, 4); }

static uint8_t port_r(void *u, int port)
{
	(void)u;
	rbx_prof_set(CV_PA_IO);   // (from sh3.c slow_r: its caller's mark put back after)
	switch (port) {
	case SH3_PORT_C: {
		uint8_t v = (uint8_t)rbx_port_read(s_port_c);
		if (s_trace > 3 && v != 0xff) rbx_log("portc %02x f%d\n", v, s_frame);
		return v;
	}
	case SH3_PORT_D: return (uint8_t)rbx_port_read(s_port_d);
	case SH3_PORT_E: return (uint8_t)(((b.nd_status & 0x40) ? 0x20 : 0x00) | 0xdf);   // (the NAND's ready line)
	case SH3_PORT_F: return (uint8_t)rbx_port_read(s_port_f);
	case SH3_PORT_L: return (uint8_t)rbx_port_read(s_port_l);
	case SH3_PORT_J: return 0xff;          // (the FPGA)
	default: return 0;
	}
}

static void port_w(void *u, int port, uint8_t d)
{
	(void)u; (void)port; (void)d;   // (port J: the FPGA's configuration stream, only checksummed by MAME)
}

static uint32_t dsw_r(void *u) { (void)u; return rbx_port_read(s_port_dsw); }

static void map_mem(void)
{
	sh3_map(s_cpu, 0x00000000, 0x003fffff, rbx_region("maincpu", NULL), 0, 1, 0);
	sh3_map(s_cpu, 0x0c000000, 0x0c000000 + s_ram_size - 1, s_ram, 0, 1, 1);
	if (s_idle_off) {   // (the idle variable's page: reads through the callback, writes direct)
		const uint32_t pg = s_idle_off & ~0xfffu;
		sh3_map(s_cpu, 0x0c000000 + pg, 0x0c000000 + pg + 0xfff, NULL, 0, 1, 1);
		sh3_map(s_cpu, 0x0c000000 + pg, 0x0c000000 + pg + 0xfff, s_ram + pg, 0x1000, 0, 1);
	}
}

/***************************************************************************
    video
***************************************************************************/

// RBX_CV1K_BLITSTAT=1: the drawing's time (on the workers) and the emulation thread's waits for it by kind (0: in a
// list, the ring full; 1: the vblank screen copy; 2: the rest), us. RBX_CV1K_BLITSLOW=n / RBX_CV1K_BLITSLOW3=n
// (testing, timing only): lane 0's / lane 1's worker idles n times its drawing time after each run (a slow core)
static int s_bstat, s_bslow, s_bslow3;
static uint64_t s_bjob_us, s_bjob3_us, s_bwait_us[3], s_bjob_prev, s_bwait_prev, s_bjob_max, s_bwait_max;
static unsigned s_bwaits[3], s_bframes, s_bwait_frames;

// the blitter's two lanes (ADR-120): lane 0 on the second worker (CPU2 on the Vita), lane 1 on the third (CPU3);
// without the third, lane 0's worker and the emulation thread draw lane 1 too
static void blit_kick(int lane)   // (the emulation thread)
{
	const unsigned pa = rbx_prof_enter(CV_PA_KICK);
	if (lane)
		rbx_aux3_nudge();
	else
		rbx_aux2_kick();
	rbx_prof_leave(pa);
}

static void blit_wait(int lane, int emu)
{
	if (emu)
		rbx_aux_wait_for(lane ? 2 : 1);
	else
		rbx_worker_wait();
}

static void blit_synced(int kind, uint64_t t0)
{
	if (!s_bstat)
		return;
	s_bwait_us[kind] += rbx_clock_us() - t0;
	s_bwaits[kind]++;
}

// the screen copy queued after the frame's drawing (ADR-150, default): the lanes copy it into the snapshot when they
// get there and render waits for that (on the render thread; without it, within rbx_video_frame_done here), the
// emulation thread only for a free snapshot of the two (rbx_video_set_depth(2)). RBX_CV1K_SHOTWAIT=1 (the Vita's data
// folder "e12shotwait"): the vblank waits for the drawing that reaches the screen and copies it, as before
static int s_shotq;
static uint32_t s_shot_k[2];   // (each snapshot's copy entry, epic12_queue_shot)

static void vblank_capture(void)
{
	const unsigned pa = rbx_prof_enter(CV_PA_VBLANK);
	if (s_shotq) {
		const uint64_t t0 = s_bstat ? rbx_clock_us() : 0;
		rbx_video_wait_slot();   // (a free snapshot: the frame before this one may still be being copied and drawn)
		blit_synced(1, t0);
		const unsigned slot = rbx_video_post_seq() & 1;
		s_shot_k[slot] = epic12_queue_shot(&b.blit, s_snap + slot * (VIS_W * VIS_H), VIS_W, VIS_H);
	} else {
		rbx_video_wait_idle();
		const uint64_t t0 = s_bstat ? rbx_clock_us() : 0;
		epic12_sync_screen(&b.blit, VIS_W, VIS_H);   // (the queued drawing that reaches the screen done)
		blit_synced(1, t0);
		epic12_screen(&b.blit, s_snap, VIS_W, VIS_H);
	}
	rbx_video_frame_done();
	if (s_bstat) {
		if (s_bframes == 60) {   // (from the second second on)
			__atomic_store_n(&s_bjob_us, 0, __ATOMIC_RELAXED);
			__atomic_store_n(&s_bjob3_us, 0, __ATOMIC_RELAXED);
			s_bwait_us[0] = s_bwait_us[1] = s_bwait_us[2] = 0;
			s_bwaits[0] = s_bwaits[1] = s_bwaits[2] = 0;
			s_bjob_prev = s_bwait_prev = 0;
		}
		const uint64_t j = __atomic_load_n(&s_bjob_us, __ATOMIC_RELAXED), w = s_bwait_us[0] + s_bwait_us[1];
		if (s_bframes++ > 60) {
			if (j - s_bjob_prev > s_bjob_max) s_bjob_max = j - s_bjob_prev;
			if (w - s_bwait_prev > s_bwait_max) s_bwait_max = w - s_bwait_prev;
			s_bwait_frames += w != s_bwait_prev;
		}
		s_bjob_prev = j;
		s_bwait_prev = w;
	}
	rbx_prof_leave(pa);
}

static void blit_lane_job(int lane)
{
	const int slow = lane ? s_bslow3 : s_bslow;
	if (!s_bstat && !slow) {
		epic12_drain_lane(&b.blit, lane);
		return;
	}
	const uint64_t t0 = rbx_clock_us();
	epic12_drain_lane(&b.blit, lane);
	const uint64_t t1 = rbx_clock_us();
	__atomic_add_fetch(lane ? &s_bjob3_us : &s_bjob_us, t1 - t0, __ATOMIC_RELAXED);
	if (slow)
		while (rbx_clock_us() - t1 < (t1 - t0) * (uint64_t)slow) {}
}
static void blit_job(void) { blit_lane_job(0); }
static void blit_job3(void) { blit_lane_job(1); }

static void cv1k_render(void)
{
	const uint16_t *snap = s_snap;
	if (s_shotq) {   // (the copy of this snapshot done: by the lanes, or drawn here on the emulation thread)
		const unsigned slot = rbx_video_render_seq() & 1;
		if (rbx_video_threaded())
			epic12_shot_wait(&b.blit, s_shot_k[slot], 0);
		else {
			const uint64_t t0 = s_bstat ? rbx_clock_us() : 0;
			epic12_shot_wait(&b.blit, s_shot_k[slot], 1);
			blit_synced(1, t0);
		}
		snap += slot * (VIS_W * VIS_H);
	}
	if (!rbx_video.out)
		return;
	s_565 =rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f && rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	for (int y = 0; y < VIS_H; y++) {
		const uint16_t *s = snap + y * VIS_W;
		uint8_t *o = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
		if (s_565) {
			for (int x = 0; x < VIS_W; x++) {
				const uint32_t p = s[x];
				((uint16_t *)o)[x] = (uint16_t)(((p & 0x7c00) << 1) | ((p & 0x03e0) << 1) | (p & 0x001f));
			}
		} else {
			for (int x = 0; x < VIS_W; x++) {
				const uint32_t p = s[x];
				const int r = (int)((p >> 10) & 0x1f) << 3, g = (int)((p >> 5) & 0x1f) << 3, bb = (int)(p & 0x1f) << 3;
				if (rbx_video.out_bpp == 4) ((uint32_t *)o)[x] = rbx_video.out_color(r, g, bb);
				else ((uint16_t *)o)[x] = (uint16_t)rbx_video.out_color(r, g, bb);
			}
		}
	}
	rbx_video.out_done = 1;
}

/***************************************************************************
    frame
***************************************************************************/

static int s_shash;
static uint32_t s_dump_a, s_dump_n;
static void run_lines(int n)
{
	b.line_frac += (uint64_t)n * LINE_NUM;
	const uint64_t cyc = b.line_frac / LINE_DEN;
	b.line_frac -= cyc * LINE_DEN;
	b.line_end += (int64_t)cyc;
	s_cpu->target = b.line_end;
	sh3_execute(s_cpu, 0);
	if (s_shash) {   // (debugging, RBX_CV1K_SHASH=frame: from that frame, the CPU state and RAM hashes after every slice)
		if (s_frame >= s_shash) {
			uint32_t hc = 2166136261u, hr = 2166136261u;
			const uint8_t *p = (const uint8_t *)s_cpu;
			for (uint32_t k = 0; k < SH3_STATE_SIZE; k++)
				hc = (hc ^ p[k]) * 16777619u;
			for (uint32_t k = 0; k < s_ram_size; k += 4)
				hr = (hr ^ *(const uint32_t *)(s_ram + k)) * 16777619u;
			fprintf(stderr, "S %d %lld %08x %08x %08x\n", s_frame, (long long)sh3_total_cycles(s_cpu), s_cpu->pc, hc, hr);
		}
	}
	if (s_sq_on) {   // (the sound so far made by the worker while the CPU runs on)
		rbx_prof_set(CV_PA_YMZ);
		snd_update(sh3_total_cycles(s_cpu));
		rbx_prof_set(CV_PA_KICK);
		rbx_aux_kick();
		rbx_prof_set(CV_PA_FRAME);
	}
	if (s_trace > 1)
		rbx_log("cv1k slice pc %08x\n", s_cpu->pc);
}

static void cv1k_frame(void)
{
	rbx_prof_set(CV_PA_FRAME);   // (the core's mark put back after the frame)
	nand_stat_frame();
	s_frame++;
	s_cpu->trace = (s_frame > s_tr_from && s_frame <= s_tr_to) ? tr_cb : NULL;
	if (s_prof_from && s_frame >= s_prof_from) {
		if (!s_prof)
			s_prof = calloc(PROF_N, sizeof(*s_prof));
		if (s_prof)
			s_cpu->trace = prof_cb;
	}
	rbx_audio.frame_mhz = 60024;
	// (MAME: the vblank starts the frame (set_vblank_time(0)): the first one comes one frame after the reset)
	for (int i = 0; i < LINES; i += 16) {
		if (s_sq_on && s_sq_hold_lines && i == LINES - s_sq_hold_lines)
			__atomic_store_n(&s_sq_hold, 1, __ATOMIC_RELEASE);
		run_lines(16);
	}
	vblank_capture();
	sh3_set_irq_line(s_cpu, 2, SH3_HOLD);   // MAME irq2_line_hold at the vblank
	rbx_prof_set(CV_PA_YMZ);
	snd_update(sh3_total_cycles(s_cpu));
	rbx_prof_set(CV_PA_IO);
	rtc_catch_up(sh3_total_cycles(s_cpu));
	rbx_prof_set(CV_PA_YMZ_SYNC);
	if (s_trace)
		rbx_log("cv1k pc %08x r15 %08x\n", s_cpu->pc, s_cpu->r[15]);
	if (s_sndstat) {
		const uint64_t t0 = rbx_clock_us();
		for (unsigned k = __atomic_load_n(&s_sq_tail, __ATOMIC_ACQUIRE); k != s_sq_head; k++) {   // (left at the frame end)
			s_snd_pend_e++;
			if (s_sq[k % SQ_N].n > 0)
				s_snd_pend_s += (unsigned)s_sq[k % SQ_N].n;
		}
		snd_sync();
		const uint64_t w = rbx_clock_us() - t0;
		s_snd_wait_us += w;
		if (w > s_snd_wait_max)
			s_snd_wait_max = w;
		s_snd_frames++;
	} else
		snd_sync();
	__atomic_store_n(&s_sq_hold, 0, __ATOMIC_RELEASE);
	rbx_prof_set(CV_PA_AUDIO);
	for (int i = 0; i < s_snd_n; i++)
		rbx_audio_put(s_snd[i]);
	s_snd_n = 0;
	rbx_prof_set(CV_PA_FRAME);
	sh3_tstat_frame(s_cpu, s_frame);
	if (s_dump_n) {   // (debugging, RBX_CV1K_DUMP=offset,bytes: those RAM words after every frame they changed in)
		static uint32_t last = 0;
		uint32_t h = 2166136261u;
		for (uint32_t k = 0; k < s_dump_n; k += 4)
			h = (h ^ ram_r32(s_dump_a + k)) * 16777619u;
		if (h != last) {
			last = h;
			fprintf(stderr, "D %d", s_frame);
			for (uint32_t k = 0; k < s_dump_n; k += 4)
				fprintf(stderr, " %08x", ram_r32(s_dump_a + k));
			fputc(10, stderr);
		}
	}
	if (s_idlef) {
		const struct sh3_idle *I = &s_cpu->idle;
		const struct sh3_spinr *S = &s_cpu->spin;
		const uint32_t v[10] = { I->ncall, I->nsnap, I->nsame, I->nrec, I->nskip, I->nfail, S->ncall, S->nfirst, S->nskip,
		                         s_cpu->nslow };
		fprintf(s_idlef, "%d", s_frame);
		for (int i = 0; i < 10; i++) {
			fprintf(s_idlef, " %u", v[i] - s_idlef_last[i]);
			s_idlef_last[i] = v[i];
		}
		fputc(10, s_idlef);
	}
	if (s_lag.on) {
		const int64_t wait = (int64_t)s_lag.reads * (CPU_HZ / 100000) + s_cpu->spin.skipped - s_lag.skip0;
		s_lag.skip0 = s_cpu->spin.skipped;
		if (s_frame > s_lag.from) {
			s_lag.frames++;
			s_lag.lag += s_lag.exits == 0;
			s_lag.tot_exits += s_lag.exits;
			s_lag.wait_tot += wait;
		}
		if (s_lag.f)
			fprintf(s_lag.f, "%d %u %lld\n", s_frame, s_lag.exits, (long long)wait);
		s_lag.exits = s_lag.reads = 0;
	}
}

static void cv1k_reset(void)
{
	rbx_video_wait_idle();
	nand_reset();
	epic12_reset(&b.blit);
	snd_sync();
	ymz770_reset(&b.ymz);
	sh3_reset(s_cpu);
	b.line_frac = 0;
	b.line_end = sh3_total_cycles(s_cpu);
	b.snd_pos = 0;
	b.rtc_next = CPU_HZ;
	s_snd_n = 0;
}

static int cv1k_supported(const char *machine)
{
	return !strncmp(machine, "cv1k:", 5) || !strncmp(machine, "cv1k_d:", 7);
}

static void nv_save(void)
{
	if (!s_nv_path[0] || !s_bp)
		return;
	FILE *f = fopen(s_nv_path, "wb");
	if (!f)
		return;
	fwrite(b.rtc_data, 1, sizeof(b.rtc_data), f);
	fclose(f);
}

static void nv_load(const char *set)
{
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	s_nv_path[0] = 0;
	for (int i = 0; i < 0x100; i++)
		b.rtc_data[i] = 0xffff;   // (MAME nvram_default)
	if (!dir || !set)
		return;
	snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, set);
	FILE *f = fopen(s_nv_path, "rb");
	if (!f)
		return;
	uint16_t e[0x100];
	if (fread(e, 1, sizeof(e), f) == sizeof(e))
		memcpy(b.rtc_data, e, sizeof(e));
	fclose(f);
}

// (ADR-200) the VRAM (64 MB) and the work RAM made by the core before the ROM regions: the lowest holes of the heap
static int cv1k_early(const rbx_game *g, size_t *sizes)
{
	sizes[0] = (size_t)EPIC12_PIXELS * 2;
	sizes[1] = !strncmp(g->machine, "cv1k_d:", 7) ? 0x1000000 : 0x800000;
	return 2;
}
static int s_vram_early, s_ram_early;   // (s_vram, s_ram: the core's blocks, not freed here)

static void cv1k_exit(void)
{
	rbx_video_wait_idle();
	if (s_bp && b.blit.q) {
		epic12_sync(&b.blit);     // (the queued drawing done)
		rbx_aux3_set_job(NULL);
		rbx_aux2_set_job(NULL);
		if (getenv("RBX_EPIC12_STAT")) {
			const epic12_t *q = &b.blit;
			rbx_log("epic12: %u lists, %u draws, %u uploads queued; ring full %u; screen syncs %u, not needed %u; "
			        "split %d, solo %u\n", q->st_exec, q->st_draw, q->st_up, q->st_full, q->st_sync, q->st_skip, q->split,
			        q->st_solo);
			static const char *const who[3] = { "lane 0 worker", "lane 1 worker", "emulation thread" };
			for (int w = 0; w < 3; w++)
				rbx_log("epic12: %s: lane 0 %u entries %llu px, lane 1 %u entries %llu px; helped %u, blocked %u\n",
				        who[w], q->st_ent[w][0], (unsigned long long)q->st_px[w][0], q->st_ent[w][1],
				        (unsigned long long)q->st_px[w][1], q->st_help[w], q->st_block[w]);
			rbx_log("epic12: lane 1 taken by the emulation thread %u times\n", q->st_take);
		}
		if (s_bstat && s_bframes > 60) {
			const double n = (double)(s_bframes - 60) * 1000.0;
			rbx_log("cv1k blitter: drawing lane 0 worker %.3f, lane 1 worker %.3f ms/frame (lane 0 max %.3f); waited: "
			        "ring full %.3f (%u), screen %.3f (%u), rest %.3f (%u) ms/frame; max %.3f, frames waiting %u of %u\n",
			        s_bjob_us / n, s_bjob3_us / n, s_bjob_max / 1000.0, s_bwait_us[0] / n, s_bwaits[0], s_bwait_us[1] / n,
			        s_bwaits[1], s_bwait_us[2] / n, s_bwaits[2], s_bwait_max / 1000.0, s_bwait_frames, s_bframes - 60);
		}
		epic12_queue_free(&b.blit);
	}
	if (s_sq_on) {
		rbx_aux_set_job(NULL);   // (syncs first)
		s_sq_on = 0;
	}
	if (s_sndstat && s_snd_frames)
		rbx_log("cv1k ymz770: work %.3f ms/frame (worker %.3f, emulation thread %.3f), frame-end %.3f ms/frame (max %.3f) "
		        "over %u frames\n", (s_snd_work_us + s_snd_own_us) / 1000.0 / s_snd_frames,
		        s_snd_work_us / 1000.0 / s_snd_frames, s_snd_own_us / 1000.0 / s_snd_frames,
		        s_snd_wait_us / 1000.0 / s_snd_frames, s_snd_wait_max / 1000.0, s_snd_frames);
	if (s_sndstat && s_snd_frames)
		rbx_log("cv1k ymz770: left at the frame end %.2f entries, %.2f samples a frame\n",
		        (double)s_snd_pend_e / s_snd_frames, (double)s_snd_pend_s / s_snd_frames);
	free(s_sq); s_sq = NULL;
	prof_report();
	if (s_cpu && getenv("RBX_CV1K_IDLESTAT"))
		rbx_log("cv1k idle rounds: %lld cycles skipped in %u skips, %u recordings, %u failed; idle loop %lld cycles in %u\n",
		        (long long)s_cpu->idle.skipped, s_cpu->idle.nskip, s_cpu->idle.nrec, s_cpu->idle.nfail,
		        (long long)s_cpu->spin.skipped, s_cpu->spin.nskip);
	if (s_cpu && getenv("RBX_CV1K_IDLESTAT"))
		rbx_log("cv1k idle calls: idle rounds %u reads, %u snapshots, %u compares, %u recordings proven pure; idle loop %u "
		        "rounds, %u first\n", s_cpu->idle.ncall, s_cpu->idle.nsnap, s_cpu->idle.nsame, s_cpu->idle.npure,
		        s_cpu->spin.ncall, s_cpu->spin.nfirst);
	if (s_idlef) {
		fclose(s_idlef);
		s_idlef = NULL;
	}
	if (s_lag.on) {
		rbx_log("cv1k lag: %u frames after %d, %u lag frames, %u main loop rounds, idle %.2f%%", s_lag.frames, s_lag.from,
		        s_lag.lag, s_lag.tot_exits, s_lag.frames ? 100.0 * (double)s_lag.wait_tot * 60.024 / CPU_HZ / s_lag.frames : 0.0);
		if (s_lag.f)
			fclose(s_lag.f);
		s_lag.f = NULL;
	}
	if (s_nst.on && s_nst.nfr)
		rbx_log("cv1k nand: %llu card reads, %llu bytes over %u frames (%.3f reads, %.0f bytes a frame); frames with reads %u, "
		        "most in a frame %u reads %u bytes\n", (unsigned long long)s_nst.miss, (unsigned long long)s_nst.bytes,
		        s_nst.nfr, (double)s_nst.miss / s_nst.nfr, (double)s_nst.bytes / s_nst.nfr, s_nst.frames_hit, s_nst.fmax,
		        s_nst.fmaxb);
	if (s_nst.on) {
		uint64_t wk, tk, wt, wus;
		rbx_io_stat(&wk, &tk, &wt, &wus);
		rbx_log("cv1k nand: %u slots, %u ahead; %llu chunks queued ahead, %llu read by the worker in %llu reads, %llu taken "
		        "back, %llu waited for (%.1f ms), %llu given up at jumps\n", s_nc_n, s_nc_ahead,
		        (unsigned long long)s_nst.ahead, (unsigned long long)wk, (unsigned long long)rbx_io_nreads(),
		        (unsigned long long)tk, (unsigned long long)wt, wus / 1000.0, (unsigned long long)s_nst.dropped);
		rbx_log("cv1k nand: the emulation thread on card reads (its own and waits) %.1f ms\n", s_nst.here_us / 1000.0);
	}
	if (s_nst.tr)
		fclose(s_nst.tr);
	if (s_nst_ff)
		fclose(s_nst_ff);
	s_nst_ff = NULL;
	memset(&s_nst, 0, sizeof(s_nst));
	nv_save();
	s_nv_path[0] = 0;
	sh3_release();   // (the JIT's tables: ADR-185)
	free(s_cpu); s_cpu = NULL;
	if (!s_ram_early)
		free(s_ram);
	if (!s_vram_early)
		free(s_vram);
	s_ram = NULL;
	s_vram = NULL;
	s_ram_early = s_vram_early = 0;
	free(s_snap); s_snap = NULL;
	free(s_snd); s_snd = NULL;
	free(s_bp); s_bp = NULL;
	nand_free();
}

static int cv1k_init(const rbx_game *g)
{
	rbx_prof_name(RBX_PA_DRV, s_cv_pa_names, CV_PA_END - RBX_PA_DRV);
	if (!cv1k_supported(g->machine)) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	const int type_d = !strncmp(g->machine, "cv1k_d:", 7);
	const char *init = strchr(g->machine, ':') + 1;
	// MAME install_speedups (idle RAM offset, idle loop pc)
	if (!strcmp(init, "mushisam")) s_idle_off = 0x024d8, s_idle_pc = 0xc04a2aa;
	else if (!strcmp(init, "ibara")) s_idle_off = 0x022f0, s_idle_pc = 0xc04a0aa;
	else if (!strcmp(init, "espgal2")) s_idle_off = 0x02310, s_idle_pc = 0xc05177a;
	else if (!strcmp(init, "mushitam")) s_idle_off = 0x022f0, s_idle_pc = 0xc04a0da;
	else if (!strcmp(init, "pinkswts")) s_idle_off = 0x02310, s_idle_pc = 0xc05176a;
	else if (!strcmp(init, "deathsml")) s_idle_off = 0x02310, s_idle_pc = 0xc0519a2;
	else if (!strcmp(init, "ddpdfk")) s_idle_off = 0x02310, s_idle_pc = 0xc1d1346;
	else s_idle_off = 0, s_idle_pc = 1;
	uint32_t prg_size = 0, ymz_size = 0;
	uint8_t *prg = rbx_region("maincpu", &prg_size);
	const uint32_t nand_size = rbx_region_file("nand");
	uint8_t *ymz = rbx_region("ymz770", &ymz_size);
	if (!prg || prg_size < 0x400000 || !nand_size || !ymz) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_rows = nand_size / FLASH_PAGE;
	s_pg_no = ~0u;
	// (the cache, ADR-160: 256 chunks (4.3 MB), 16 read ahead; RBX_NAND_OLD=1: 64 (1 MB), none ahead)
	const int old = getenv("RBX_NAND_OLD") && atoi(getenv("RBX_NAND_OLD"));
	s_nc_n = old ? 64 : getenv("RBX_NAND_SLOTS") ? (uint32_t)atoi(getenv("RBX_NAND_SLOTS")) : 256;
	s_nc_ahead = old ? 0 : getenv("RBX_NAND_AHEAD") ? (uint32_t)atoi(getenv("RBX_NAND_AHEAD")) : 16;
	s_nc_drop = !old && !(getenv("RBX_NAND_DROP") && !atoi(getenv("RBX_NAND_DROP")));
	if (s_nc_ahead > NC_PEND)
		s_nc_ahead = NC_PEND;
	if (s_nc_ahead && s_nc_n < NC_PEND + 8)   // (slots for the chunks looked up besides those being read ahead)
		s_nc_n = NC_PEND + 8;
	if (s_nc_n < 4)
		s_nc_n = 4;
	s_nc_chunks = (s_rows + NC_PAGES - 1) / NC_PAGES;
	s_nc_use = s_nc_last = s_nc_npend = 0;
	s_ram_size = type_d ? 0x1000000 : 0x800000;
	// (ADR-185: the largest first, the 64 MB VRAM at the lowest hole: a fixed-size heap (newlib's, the Vita) keeps a
	// block for the app's life wherever it was made, and the next game's buffers must fit around it. ADR-200: the VRAM
	// and the RAM are the core's blocks made before the ROM regions (cv1k_early), zeroed as calloc's)
	size_t esz = 0;
	if ((s_vram = rbx_early(0, &esz)) != NULL && esz == (size_t)EPIC12_PIXELS * 2)
		s_vram_early = 1;
	else
		s_vram = NULL;
	if ((s_ram = rbx_early(1, &esz)) != NULL && esz == s_ram_size)
		s_ram_early = 1;
	else
		s_ram = NULL;
	if ((!s_vram && !(s_vram = calloc(EPIC12_PIXELS, 2))) || (!s_ram && !(s_ram = calloc(s_ram_size, 1))) ||
	    !(s_nc = malloc((size_t)s_nc_n * NC_PAGES * FLASH_PAGE)) || !(s_cpu = calloc(1, sizeof(*s_cpu))) ||
	    !(s_bp = calloc(1, sizeof(*s_bp))) ||
	    !(s_snap = calloc(2 * VIS_W * VIS_H, 2)) || !(s_snd = calloc(SND_CAP, sizeof(int16_t))) ||
	    !(s_nand_w = calloc(s_rows, sizeof(*s_nand_w))) ||
	    !(s_ncs = calloc(s_nc_n, sizeof(*s_ncs))) || !(s_nc_map = calloc(s_nc_chunks, sizeof(*s_nc_map)))) {
		rbx_fail("%s: out of memory (VRAM %s, RAM %s, NAND cache %s)", g->set, s_vram ? (s_vram_early ? "early" : "ok") : "no",
		         s_ram ? (s_ram_early ? "early" : "ok") : "no", s_nc ? "ok" : "no");
		cv1k_exit();
		return -1;
	}
	sh3_swap32(prg, 0x400000);   // (the big-endian image in host-order 32-bit words)
	s_port_c = rbx_port_find("PORT_C");
	s_port_d = rbx_port_find("PORT_D");
	s_port_f = rbx_port_find("PORT_F");
	s_port_l = rbx_port_find("PORT_L");
	s_port_dsw = rbx_port_find("DSW");
	s_trace = getenv("RBX_CV1K_TRACE") ? atoi(getenv("RBX_CV1K_TRACE")) : 0;
	s_tr_from = getenv("RBX_CV1K_ITRACE") ? atoi(getenv("RBX_CV1K_ITRACE")) : 0;   // (frame n + 1: an instruction trace)
	s_tr_to = s_tr_from ? s_tr_from + (getenv("RBX_CV1K_ITRACE_N") ? atoi(getenv("RBX_CV1K_ITRACE_N")) : 1) : 0;
	s_frame = 0;
	s_vrz_frame = -1;
	memset(&s_nst, 0, sizeof(s_nst));
	s_nst.on = getenv("RBX_NAND_STAT") && atoi(getenv("RBX_NAND_STAT"));
	s_nst.tr = getenv("RBX_NAND_TRACE") ? fopen(getenv("RBX_NAND_TRACE"), "wb") : NULL;
	s_nst_ff = s_nst.on && getenv("RBX_NAND_FRAMES") ? fopen(getenv("RBX_NAND_FRAMES"), "w") : NULL;
	s_nst_late0 = 0;
	s_shash =getenv("RBX_CV1K_SHASH") ? atoi(getenv("RBX_CV1K_SHASH")) : 0;
	s_dump_n = 0;
	if (getenv("RBX_CV1K_DUMP"))
		sscanf(getenv("RBX_CV1K_DUMP"), "%x,%x", &s_dump_a, &s_dump_n);
	s_prof_from = getenv("RBX_CV1K_PROF") ? atoi(getenv("RBX_CV1K_PROF")) : 0;
	s_prof_pc = 0;
	memset(&s_lag, 0, sizeof(s_lag));
	s_lag.on = getenv("RBX_CV1K_LAG") != NULL;
	s_lag.from = s_lag.on ? atoi(getenv("RBX_CV1K_LAG")) : 0;
	if (s_lag.on && getenv("RBX_CV1K_LAGF"))
		s_lag.f = fopen(getenv("RBX_CV1K_LAGF"), "w");
	memset(s_idlef_last, 0, sizeof(s_idlef_last));
	s_idlef = getenv("RBX_CV1K_IDLEF") ? fopen(getenv("RBX_CV1K_IDLEF"), "w") : NULL;
	nv_load(g->set);
	// the RTC (MAME: the host's time): 2000-01-01, a Saturday
	b.rtc_year = 0x00; b.rtc_month = 0x01; b.rtc_day = 0x01; b.rtc_wday = 1 << 6;
	b.rtc_hour = b.rtc_min = b.rtc_sec = 0;
	if (getenv("RBX_CV1K_RTC"))   // (testing: hhmmss in BCD)
		{ unsigned v = (unsigned)strtoul(getenv("RBX_CV1K_RTC"), NULL, 16); b.rtc_hour = (uint8_t)(v >> 16); b.rtc_min = (uint8_t)(v >> 8); b.rtc_sec = (uint8_t)v; }
	b.eo_old[0] = 0; b.eo_old[1] = 0; b.eo_old[2] = 1;   // (the fields' default values)
	b.blit.vram = s_vram;
	b.blit.ram = s_ram;
	b.blit.rammask = s_ram_size - 1;
	b.blit.dsw = dsw_r;
	b.blit.cpu_hz = CPU_HZ;
	// the drawing on the second worker (ADR-085; RBX_CV1K_NOBLITQ=1: drawn at the exec write; RBX_CV1K_BLITQ=words:
	// the ring's size, 512 KB, for testing)
	const uint32_t qw = getenv("RBX_CV1K_BLITQ") ? (uint32_t)strtoul(getenv("RBX_CV1K_BLITQ"), NULL, 0) : 1u << 17;
	s_bstat = getenv("RBX_CV1K_BLITSTAT") && atoi(getenv("RBX_CV1K_BLITSTAT"));
	s_bslow = getenv("RBX_CV1K_BLITSLOW") ? atoi(getenv("RBX_CV1K_BLITSLOW")) : 0;
	s_bslow3 = getenv("RBX_CV1K_BLITSLOW3") ? atoi(getenv("RBX_CV1K_BLITSLOW3")) : 0;
	// (RBX_CV1K_SPLIT=n: lane 1 draws 1 in n 16-row blocks, 0 = one lane; ADR-120)
	const int split = getenv("RBX_CV1K_SPLIT") ? atoi(getenv("RBX_CV1K_SPLIT")) : 3;
	s_bjob3_us = 0;
	s_bjob_us = s_bjob_prev = s_bwait_prev = s_bjob_max = s_bwait_max = 0;
	memset(s_bwait_us, 0, sizeof(s_bwait_us));
	memset(s_bwaits, 0, sizeof(s_bwaits));
	s_bframes = s_bwait_frames = 0;
	if (!(getenv("RBX_CV1K_NOBLITQ") && atoi(getenv("RBX_CV1K_NOBLITQ"))) &&
	    !epic12_queue(&b.blit, qw, split == 1 || split < 0 ? 3 : split, blit_kick, blit_wait)) {
		rbx_aux2_set_job(blit_job);
		rbx_aux2_set_alone(RBX_AUX2_BY_RENDER);   // (CPU2 on the Vita, a whole core: CPU3's half fell behind, ADR-105)
		if (b.blit.split) {
			rbx_aux3_set_job(blit_job3);
			rbx_aux3_set_alone(1);   // (CPU3 on the Vita, alone, low: a lane the others take over)
		}
	}
	// (ADR-150: the screen copy in the queue, the render thread one frame behind; RBX_CV1K_SHOTWAIT=1 as before)
	s_shotq = !(getenv("RBX_CV1K_SHOTWAIT") && atoi(getenv("RBX_CV1K_SHOTWAIT")));
	s_shot_k[0] = s_shot_k[1] = 0;
	if (s_shotq)
		rbx_video_set_depth(2);
	ymz770_init(&b.ymz, ymz, ymz_size);
	s_sq_head = s_sq_tail = 0;
	s_sndstat = getenv("RBX_CV1K_SNDSTAT") && atoi(getenv("RBX_CV1K_SNDSTAT"));
	s_snd_work_us = s_snd_wait_us = s_snd_wait_max = s_snd_own_us = 0;
	s_snd_pend_e = s_snd_pend_s = 0;
	s_sq_lock = s_sq_take = s_sq_hold = 0;
	s_sq_hold_lines = getenv("RBX_CV1K_SNDHOLD") ? atoi(getenv("RBX_CV1K_SNDHOLD")) & ~15 : 16;
	if (s_sq_hold_lines < 0 || s_sq_hold_lines > LINES)
		s_sq_hold_lines = 0;
	s_snd_frames = 0;
	s_sq_on = !(getenv("RBX_CV1K_NOSNDQ") && atoi(getenv("RBX_CV1K_NOSNDQ"))) &&
	          (s_sq = (snd_ev_t *)calloc(SQ_N, sizeof(snd_ev_t))) != NULL;
	if (s_sq_on) {
		rbx_aux_set_job(snd_job);
		rbx_aux_set_by_render(1);   // (CPU2 on the Vita, beside the light render thread: CPU1 has the sound output)
	}
	sh3_init(s_cpu);
	s_cpu->read8 = r8;
	s_cpu->read16 = r16;
	s_cpu->read32 = r32;
	s_cpu->write8 = w8;
	s_cpu->write16 = w16;
	s_cpu->write32 = w32;
	s_cpu->read_block8 = rblk8;
	s_cpu->port_r = port_r;
	s_cpu->port_w = port_w;
	map_mem();
	s_cpu->flush_pc = s_idle_off ? s_idle_pc + 2 : 1;   // (MAME sh2drc_add_pcflush(idlepc + 2))
	s_cpu->fetch_wr = 1;   // (the idle variable's page: rd() returns the RAM, the speed-up only at the variable)
	timing_setup(g);
	// RBX_CV1K_NOIDLE=1: the polling loops run round by round (sh3.c idle rounds: the blitter status; sh3_spin_round:
	// the idle loop)
	s_noidle = getenv("RBX_CV1K_NOIDLE") && atoi(getenv("RBX_CV1K_NOIDLE"));
	if (!s_noidle) {
		s_cpu->read_until = blit_until;
		s_cpu->ru_lo = 0x18000000u;
		s_cpu->ru_hi = 0x18000057u;
	}
	if (rbx_audio_setup(YMZ770_RATE) || rbx_video_setup(VIS_W, VIS_H, 1)) {
		rbx_fail("%s: out of memory", g->set);
		cv1k_exit();
		return -1;
	}
	cv1k_reset();
	return 0;
}

/* the VRAM in a save state, packed (64 MB, mostly runs): 16-bit little-endian words, 0x8000 | n then a pixel
   repeated n times, or n (1 ... 0x7fff) then n pixels, row after row (each EPIC12_W pixels, in memory EPIC12_STRIDE
   apart). out NULL: the bytes only */
#define VPX(v, i) (v)[(size_t)((i) / EPIC12_W) * EPIC12_STRIDE + ((i) & (EPIC12_W - 1))]
static uint32_t vram_pack(const uint16_t *v, uint8_t *out)
{
	const uint32_t n = (uint32_t)EPIC12_W * EPIC12_H;
	uint32_t i = 0, w = 0, lit = 0, cnt = 0;
#define PUT(at, x) do { if (out) { out[2 * (at)] = (uint8_t)(x); out[2 * (at) + 1] = (uint8_t)((x) >> 8); } } while (0)
	while (i < n) {
		uint32_t j = i + 1;
		const uint16_t vi = VPX(v, i);
		while (j < n && VPX(v, j) == vi && j - i < 0x7fff)
			j++;
		if (j - i >= 3) {
			PUT(w, 0x8000 | (j - i));
			PUT(w + 1, vi);
			w += 2;
			i = j;
			cnt = 0;
		} else {
			if (!cnt)
				lit = w++;
			PUT(w, vi);
			w++;
			i++;
			cnt++;
			PUT(lit, cnt);
			if (cnt == 0x7fff)
				cnt = 0;
		}
	}
#undef PUT
	return w * 2;
}

static int vram_unpack(const uint8_t *in, uint32_t bytes, uint16_t *v)
{
	const uint32_t n = (uint32_t)EPIC12_W * EPIC12_H, words = bytes / 2;
	uint32_t i = 0, w = 0;
	while (w < words && i < n) {
		const uint32_t t = in[2 * w] | (uint32_t)in[2 * w + 1] << 8, c = t & 0x7fff;
		w++;
		if (!c || c > n - i || w + ((t & 0x8000) ? 1 : c) > words)
			return -1;
		if (t & 0x8000) {
			const uint16_t px = (uint16_t)(in[2 * w] | in[2 * w + 1] << 8);
			w++;
			for (uint32_t k = 0; k < c; k++)
				{ VPX(v, i) = px; i++; }
		} else
			for (uint32_t k = 0; k < c; k++, w++)
				{ VPX(v, i) = (uint16_t)(in[2 * w] | in[2 * w + 1] << 8); i++; }
	}
	return i == n && w == words ? 0 : -1;
}
#undef VPX

static int cv1k_state(void *ctx, int load)
{
	if (load)
		rbx_video_wait_idle();
	const uint64_t t0 = s_bstat ? rbx_clock_us() : 0;
	epic12_sync(&b.blit);   // (the VRAM as drawn)
	blit_synced(2, t0);
	snd_sync();             // (the YMZ770 up to now)
	int rc = 0;
	rc |= rbx_state_io(ctx, "sh3", s_cpu, SH3_STATE_SIZE);
	rc |= rbx_state_io(ctx, "cv1k", s_bp, (uint32_t)offsetof(struct cv1k_state, blit));
	rc |= rbx_state_io(ctx, "epic12", &b.blit, EPIC12_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ymz770", &b.ymz, YMZ770_STATE_SIZE);
	rc |= rbx_state_io(ctx, "cv1k_ram", s_ram, s_ram_size);
	// the packed VRAM's size: not in the load's check walk (read from the file), and once per save (the sizing pass's,
	// reused by the writing pass of the same frame: the 64 MB scan took about 1.5 s on the Vita each time)
	uint32_t vn = 0;
	if (!load && !rbx_state_checking(ctx)) {
		if (s_vrz_frame != s_frame) {
			s_vrz_n = vram_pack(s_vram, NULL);
			s_vrz_frame = s_frame;
		}
		vn = s_vrz_n;
	}
	uint8_t *vz = (uint8_t *)rbx_state_var(ctx, "cv1k_vrz", &vn, load);
	if (load) {
		rc |= !vz || vram_unpack(vz, vn, s_vram);
		s_vrz_frame = -1;
	} else if (vz) {
		vram_pack(s_vram, vz);
		s_vrz_frame = -1;
	}
	if (!rc && load) {
		sh3_post_load(s_cpu);
		s_snd_n = 0;
	}
	return rc ? -1 : 0;
}

static int cv1k_save(void *ctx) { return cv1k_state(ctx, 0); }
static int cv1k_load(void *ctx) { return cv1k_state(ctx, 1); }

static const char *const cv1k_file_regions[] = { "nand", NULL };
const rbx_driver rbx_drv_cv1k = { "Cave CV1000", cv1k_init, cv1k_exit, cv1k_reset, cv1k_frame, cv1k_save, cv1k_load,
                                  cv1k_supported, cv1k_render, NULL, cv1k_file_regions, cv1k_early };
