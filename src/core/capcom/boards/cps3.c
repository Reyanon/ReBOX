/*  Capcom CPS-3 (Red Earth / Warzard, Street Fighter III, 2nd Impact, 3rd Strike, JoJo's Venture, JoJo's Bizarre
    Adventure). Ported from MAME 0.289 src/mame/capcom/cps3.cpp, cps3.h (BSD-3-Clause, copyright-holders: David Haywood,
    Andreas Naive, Tomasz Slanina, ElSemi), cps3_a.cpp, cps3_a.h (BSD-3-Clause, copyright-holders: Philip Bennett) and
    the Fujitsu 29F016A part of src/devices/machine/intelfsh.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    An encrypted SH-2 (SH7604) at 25 MHz; the program in two SIMMs of four 29F016A flash chips, the graphics and
    samples in up to four SIMMs of eight; the CPS3 PPU (zoomed sprites and tilemap lines drawn through a
    1024x448 palette index render buffer with framebuffer zoom, a text layer, 0x20000 colours with fades and
    shadow / highlight bits), character RAM filled by a compressed DMA, the CPS3 custom 16-voice PCM.
    The BIOS is decrypted in place; the program flash is decrypted into an image the instructions are fetched from
    (the data reads see the same image, except on sfiii2's dead cart where they see the plain one); the cache RAM at
    0xc0000000 is fetched through a decrypted copy. The graphics flash is read where it is (the SIMM regions) instead
    of MAME's interleaved copy (user5): an erased sector is seen at once (MAME: only at the next write of a word).
    The CD sets (the BIOS and a CHD) are not emulated, only the NO CD ones.
    Drawing: the vblank snapshot holds the sprite list, the tilemap words, the colours, the text RAM and the
    registers; the character RAM is not copied, the emulation waits for the drawing before writing it.
    Speed-up (exact): the task scheduler's idle rounds are recorded once and skipped while nothing they read changes
    (see "idle rounds"; RBX_CPS3_NOIDLE turns it off, RBX_CPS3_STAT logs the cycles skipped); the CPU runs in slices of
    64 lines (the vblank is the only interrupt; same results as one slice a line). The character DMA runs on a worker
    thread (rbx_aux) while the emulation goes on, synced where its results are used (ADR-125; RBX_CPS3_DMA_INLINE
    keeps it on the emulation thread, RBX_CPS3_PROF=us logs the slow frames).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/sh2/sh2.h"
#include "../../video/decoband.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define XTAL 42954545u
#define CPU_CLOCK 25000000u      // 6.25 MHz x 4
#define LINES 264
#define VIS_W 384
#define VIS_H 224
#define HTOTAL 546               // (454 + 1) * 6 / 5 pixels of XTAL / 5
#define LINE_NUM ((uint64_t)CPU_CLOCK * HTOTAL * 5)   // CPU cycles of a line x XTAL
#define SND_DEN ((uint64_t)CPU_CLOCK * 3 * 384)       // CPU cycles of a sample x XTAL
#define SND_CAP 4096
#define RB_W 1024
#define RB_H 448
#define NEVER INT64_MAX

// ---- ROMs, configuration ----
static uint32_t s_key1, s_key2;
static int s_alt;                // MAME m_altEncryption
static uint8_t *s_chip[7][8];    // the flash chips (2 MB each, the SIMM regions), NULL = not fitted
static int s_nsimm_prg;          // program SIMMs fitted (1 or 2)
static uint8_t *s_gpair[2][16];  // the graphics chip pairs in user5 order
static int s_ngpair;
static uint32_t *s_dec;          // the program flash decrypted (host-order words: fetches, and data unless s_alt)
static uint32_t *s_raw;          // the plain program flash (s_alt only: the data reads)
static int s_port_in, s_port_extra;
static sh2_t s_cpu;
static char s_nv_path[300];

enum { FM_NORMAL, FM_READID, FM_READSTATUS, FM_WRITEPART1, FM_CLEARPART1, FM_SETMASTER, FM_READAMDID1, FM_READAMDID2,
       FM_READAMDID3, FM_ERASEAMD1, FM_ERASEAMD2, FM_ERASEAMD3, FM_ERASEAMD4, FM_BYTEPROGRAM, FM_FAST_RESET };

typedef struct {
	uint8_t mode, status, master_lock, fast_mode;
	uint32_t erase_sector;
	int64_t timer;   // MAME m_timer (CPU clock, NEVER = off)
} flash_t;

typedef struct {
	uint32_t regs[8];
	uint32_t pos, frac;
} voice_t;

static struct cps3_state {   // (on the heap while the game runs, ADR-040)
	uint32_t mainram[0x80000 / 4];
	uint32_t spriteram[0x80000 / 4];
	uint16_t colourram[0x20000];
	uint32_t spritelist[0x80000 / 4 + 0x800];   // (+ room for MAME's reads past the end)
	uint8_t ss_ram[0x8000];
	uint32_t fram[0x400 / 4];
	uint32_t cache[0x400 / 4];
	uint32_t cache_dec[0x400 / 4];
	uint32_t eeprom[0x80 / 4];
	uint32_t gscroll[8], tmap_regs[16], crtc_zoom[8], gscroll_buff[8];
	flash_t flash[7][8];
	voice_t voice[16];
	uint16_t key;
	int16_t ss_hscroll, ss_vscroll;
	uint8_t ss_pal_base;
	uint16_t dma_status, spritelist_dma, current_eeprom_read;
	uint32_t cram_gfxflash_bank, cram_bank;
	uint32_t paldma_source, paldma_realsource, paldma_dest, paldma_fade, paldma_other2, paldma_length;
	uint32_t chardma_source, chardma_other, current_table_address;
	int32_t rle_length, last_normal_byte;
	uint16_t lastb, lastb2;
	int64_t dma_timer, spr_timer;
	int64_t line_end;
	uint64_t line_frac, snd_frac;
	int64_t snd_last;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static uint32_t *s_char_ram;     // 8 MB (saved apart)
// The character RAM as the emulation sees it, by 4 KB page: the RAM itself, or (written while the frame drawn last is
// being drawn from the RAM) a copy taken at the first write, put back at the next vblank when the drawing is over
#define COW_PAGES 512
static uint32_t *s_cram_ev[0x800000 / 4096];
static uint32_t *s_cow_pool;
static uint16_t s_cow_pg[COW_PAGES];
static int s_cow_n, s_cowtest;
static int s_cram_drawing;       // the drawing may read the character RAM (set at the vblank, cleared by the drawing)

#define UNLIKELY_COW(x) __builtin_expect(!!(x), 0)
static inline uint32_t *cram_live(uint32_t p) { return s_char_ram + (size_t)p * 1024; }

// the copies back into the RAM (the drawing is over); drop: they are forgotten (the RAM was loaded)
static void cram_merge(int drop)
{
	for (int i = 0; i < s_cow_n; i++) {
		const uint32_t p = s_cow_pg[i];
		if (!drop)
			memcpy(cram_live(p), s_cram_ev[p], 4096);
		s_cram_ev[p] = cram_live(p);
	}
	s_cow_n = 0;
}

static inline uint32_t *cram_rd(uint32_t idx) { return s_cram_ev[idx >> 10] + (idx & 1023); }

static uint32_t *cram_wr(uint32_t idx)
{
	const uint32_t p = idx >> 10;
	if (s_cram_ev[p] == cram_live(p) && (s_cowtest || __atomic_load_n(&s_cram_drawing, __ATOMIC_ACQUIRE))) {
		if (s_cow_n == COW_PAGES) {   // (no room: wait for the drawing)
			rbx_video_wait_idle();
			cram_merge(0);
		} else {
			uint32_t *cp = s_cow_pool + (size_t)s_cow_n * 1024;
			memcpy(cp, cram_live(p), 4096);
			s_cow_pg[s_cow_n++] = (uint16_t)p;
			s_cram_ev[p] = cp;
		}
	}
	return s_cram_ev[p] + (idx & 1023);
}

// a byte of the character RAM written by the character DMA (a: byte address, MAME's host order: the guest byte ^ 3)
static int s_dma_drawing;   // (s_cram_drawing when the DMA started: the drawing may end meanwhile, not start)
static inline void cram_wb(uint32_t a, uint8_t v)
{
	const uint32_t p = (a >> 12) & 0x7ff;
	if (UNLIKELY_COW(s_cram_ev[p] == cram_live(p) && (s_dma_drawing || s_cowtest)) && cram_wr(p << 10) == cram_live(p))
		s_dma_drawing = 0;   // (no copy made: the drawing is over)
	((uint8_t *)s_cram_ev[p])[a & 0xfff] = v;
}
// RBX_CPS3_PROF=us: a line per frame taking longer (the time in the character DMA, the CPU cycles run)
static struct { uint64_t out_bytes, dma_us, dma_bytes, dma_n, hit_n, dec_us, wait_us; int on; uint32_t thr; } s_prof;
static int32_t *s_snd;           // the frame's samples (left, right)
static void rnd_forget(void);
static int s_snd_n;

// ---- encryption ----
static inline uint16_t rotate_left(uint16_t value, int n)
{
	const int aux = value >> (16 - n);
	return (uint16_t)(((value << n) | aux) % 0x10000);
}

static inline uint16_t rotxor(uint16_t val, uint16_t xorval)
{
	uint16_t res = (uint16_t)(val + rotate_left(val, 2));
	res = (uint16_t)(rotate_left(res, 4) ^ (res & (val ^ xorval)));
	return res;
}

static uint32_t cps3_mask(uint32_t address, uint32_t key1, uint32_t key2)
{
	if (s_alt == 2)
		return 0;
	address ^= key1;
	uint16_t val = (uint16_t)((address & 0xffff) ^ 0xffff);
	val = rotxor(val, (uint16_t)(key2 & 0xffff));
	val ^= (uint16_t)((address >> 16) ^ 0xffff);
	val = rotxor(val, (uint16_t)(key2 >> 16));
	val ^= (uint16_t)((address & 0xffff) ^ (key2 & 0xffff));
	return val | ((uint32_t)val << 16);
}
#define MASK(a) cps3_mask((a), s_key1, s_key2)

// ---- user5: the graphics / sample flash in MAME's interleaved layout, read from the chips ----
// host byte a of MAME's m_user5 (a little-endian host): pair a >> 22, chip (a & 1) ^ 1, chip byte (a >> 1) & 0x1fffff
static inline uint8_t u5(uint32_t a)
{
	const uint32_t p = a >> 22;
	if (p >= (uint32_t)s_ngpair)
		return 0;
	return s_gpair[(a & 1) ^ 1][p][(a >> 1) & 0x1fffff];
}
// MAME sourcedata[DMA_XOR(a)]
static inline uint8_t u5_dma(uint32_t a) { return u5(a ^ 1); }

// ---- flash (intelfsh 29F016A: maker 0x04, device 0xad, 8-bit, 64 KB sectors) ----
static void flash_tick(flash_t *f)
{
	if (f->timer == NEVER || sh2_total_cycles(&s_cpu) < f->timer)
		return;
	f->timer = NEVER;
	switch (f->mode) {
	case FM_READSTATUS:
		f->status = 0x80;
		break;
	case FM_ERASEAMD4:
		f->mode = FM_NORMAL;
		f->status = 0x80;
		break;
	}
}

static void flash_timer(flash_t *f, uint32_t ms) { f->timer = sh2_total_cycles(&s_cpu) + (int64_t)CPU_CLOCK / 1000 * ms; }

static uint8_t flash_read(flash_t *f, const uint8_t *data, uint32_t address)
{
	flash_tick(f);
	switch (f->mode) {
	default:
	case FM_NORMAL:
		return data[address];
	case FM_READSTATUS:
		return f->status;
	case FM_READAMDID3:
		switch (address & 0xff) {
		case 0: return 0x04;
		case 1: return 0xad;
		default: return 0;
		}
	case FM_READID:
		switch (address & 0xff) {
		case 0: return 0x04;
		case 1: return 0xad;
		case 3: return f->master_lock ? 1 : 0;
		default: return 0;
		}
	case FM_ERASEAMD4:   // (the Fujitsu 0xad: every read polls the status)
		f->status ^= (1 << 6) | (1 << 2);
		return f->status;
	}
}

static void flash_write(flash_t *f, uint8_t *data, uint32_t address, uint8_t v)
{
	flash_tick(f);
	switch (f->mode) {
	case FM_NORMAL:
	case FM_READSTATUS:
	case FM_READID:
	case FM_READAMDID3:
		switch (v) {
		case 0xf0:
		case 0xff:
			f->mode = FM_NORMAL;
			break;
		case 0x90:
			f->mode = f->fast_mode ? FM_FAST_RESET : FM_READID;
			break;
		case 0x40:
		case 0x10:
			f->mode = FM_WRITEPART1;
			break;
		case 0x50:
			f->status = 0x80;
			f->mode = FM_READSTATUS;
			break;
		case 0x20:
			f->mode = FM_CLEARPART1;
			break;
		case 0x60:
			f->mode = FM_SETMASTER;
			break;
		case 0x70:
			f->mode = FM_READSTATUS;
			break;
		case 0xa0:
			if (f->fast_mode)
				f->mode = FM_BYTEPROGRAM;
			break;
		case 0xaa:
			if ((address & 0xfff) == 0x555 || (address & 0xfff) == 0xaaa)
				f->mode = FM_READAMDID1;
			break;
		}
		break;
	case FM_READAMDID1:
		if (((address & 0xffff) == 0x2aa || (address & 0xffff) == 0x2aaa || (address & 0xfff) == 0x555) && v == 0x55)
			f->mode = FM_READAMDID2;
		else
			f->mode = FM_NORMAL;
		break;
	case FM_READAMDID2: {
		const int a1 = (address & 0xffff) == 0x555, a2 = (address & 0xffff) == 0x5555, a3 = (address & 0xfff) == 0xaaa;
		if ((a1 || a2 || a3) && v == 0x90)
			f->mode = FM_READAMDID3;
		else if ((a1 || a2 || a3) && v == 0x80)
			f->mode = FM_ERASEAMD1;
		else if ((a1 || a2 || a3) && v == 0xa0)
			f->mode = FM_BYTEPROGRAM;
		else if ((a1 || a2 || a3) && v == 0xf0)
			f->mode = FM_NORMAL;
		else if (((address & 0xfff) == 0xaaa || (address & 0xfff) == 0x555) && v == 0x20) {
			f->mode = FM_NORMAL;
			f->fast_mode = 1;
		} else
			f->mode = FM_NORMAL;
		break;
	}
	case FM_ERASEAMD1:
		if (((address & 0xfff) == 0x555 || (address & 0xfff) == 0xaaa) && v == 0xaa)
			f->mode = FM_ERASEAMD2;
		break;
	case FM_ERASEAMD2:
		if (((address & 0xffff) == 0x2aa || (address & 0xffff) == 0x2aaa || (address & 0xfff) == 0x555) && v == 0x55)
			f->mode = FM_ERASEAMD3;
		break;
	case FM_ERASEAMD3:
		if (((address & 0xfff) == 0x555 || (address & 0xfff) == 0xaaa) && v == 0x10) {   // chip erase
			memset(data, 0xff, 0x200000);
			f->status = 1 << 3;
			f->mode = FM_ERASEAMD4;
			flash_timer(f, 16000);
		} else if (v == 0x30) {   // sector erase
			memset(data + (address & ~0xffffu), 0xff, 0x10000);
			f->erase_sector = address & ~0xffffu;
			flash_timer(f, 1000);
			f->status = 1 << 3;
			f->mode = FM_ERASEAMD4;
		}
		break;
	case FM_BYTEPROGRAM:
		data[address] = v;
		f->mode = FM_NORMAL;
		break;
	case FM_WRITEPART1:
		data[address] = v;
		f->status = 0x80;
		f->mode = FM_READSTATUS;
		break;
	case FM_CLEARPART1:
		if (v == 0xd0) {
			memset(data + (address & ~0xffffu), 0xff, 0x10000);
			flash_timer(f, 1000);
		}
		f->status = 0x00;
		f->mode = FM_READSTATUS;
		break;
	case FM_SETMASTER:
		if (v == 0xf1)
			f->master_lock = 1;
		else if (v == 0xd0)
			f->master_lock = 0;
		f->mode = FM_READSTATUS;
		break;
	case FM_FAST_RESET:
		if (v == 0xf0 || v == 0) {
			f->fast_mode = 0;
			f->mode = FM_NORMAL;
		}
		break;
	}
}

// ---- the program flash images ----
static uint32_t prg_raw_word(int which, uint32_t i)
{
	return (uint32_t)s_chip[which][0][i] << 24 | (uint32_t)s_chip[which][1][i] << 16 | (uint32_t)s_chip[which][2][i] << 8 |
	       s_chip[which][3][i];
}

static void prg_map(int which)
{
	const uint32_t base = 0x06000000 + (uint32_t)which * 0x800000;
	int normal = 1;
	for (int i = 0; i < 4; i++) {
		flash_tick(&b.flash[which][i]);
		normal &= b.flash[which][i].mode == FM_NORMAL;
	}
	uint32_t *img = s_alt ? s_raw : s_dec;
	sh2_map(&s_cpu, base, base + 0x7fffff, normal ? img + (uint32_t)which * 0x200000 : NULL, 0x800000, 1, 0);
}

// MAME copy_from_nvram (the program part)
static void copy_from_nvram(void)
{
	for (int which = 0; which < s_nsimm_prg; which++)
		for (uint32_t i = 0; i < 0x200000; i++) {
			const uint32_t data = prg_raw_word(which, i);
			s_dec[which * 0x200000 + i] = data ^ MASK(0x6000000 + which * 0x800000 + i * 4);
			if (s_alt)
				s_raw[which * 0x200000 + i] = data;
		}
}

static uint32_t flashmain_r(int which, uint32_t offset, uint32_t mask)
{
	uint32_t result = 0;
	if (which >= s_nsimm_prg)
		return 0xffffffff;
	for (int l = 0; l < 4; l++)
		if (mask & (0xffu << (24 - 8 * l)))
			result |= (uint32_t)flash_read(&b.flash[which][l], s_chip[which][l], offset) << (24 - 8 * l);
	return result;
}

static void flashmain_w(int which, uint32_t offset, uint32_t data, uint32_t mask)
{
	if (which >= s_nsimm_prg)
		return;
	for (int l = 0; l < 4; l++)
		if (mask & (0xffu << (24 - 8 * l)))
			flash_write(&b.flash[which][l], s_chip[which][l], offset, (uint8_t)(data >> (24 - 8 * l)));
	const uint32_t newdata = prg_raw_word(which, offset);
	s_dec[which * 0x200000 + offset] = newdata ^ MASK(0x6000000 + which * 0x800000 + offset * 4);
	if (s_alt)
		s_raw[which * 0x200000 + offset] = newdata;
	prg_map(which);
}

// the graphics flash pair of cram_gfxflash_bank (chip0, chip1), 0 = not fitted
static int gfx_chips(int *simm, int *c0)
{
	const uint32_t bank = b.cram_gfxflash_bank;
	if (bank / 8 >= 5)
		return 0;
	*simm = 2 + (int)(bank / 8);
	*c0 = (int)((bank % 8) & ~1u);
	return s_chip[*simm][*c0] && s_chip[*simm][*c0 | 1];
}

static uint32_t gfxflash_r(uint32_t offset, uint32_t mask)
{
	uint32_t result = 0;
	if (b.cram_gfxflash_bank & 1)
		offset += 0x200000 / 4;
	int s, c0;
	if (!gfx_chips(&s, &c0))
		return 0xffffffff;
	flash_t *f0 = &b.flash[s][c0], *f1 = &b.flash[s][c0 | 1];
	const uint8_t *d0 = s_chip[s][c0], *d1 = s_chip[s][c0 | 1];
	if (mask & 0xff000000) result |= (uint32_t)flash_read(f0, d0, offset << 1) << 24;
	if (mask & 0x00ff0000) result |= (uint32_t)flash_read(f1, d1, offset << 1) << 16;
	if (mask & 0x0000ff00) result |= (uint32_t)flash_read(f0, d0, (offset << 1) + 1) << 8;
	if (mask & 0x000000ff) result |= (uint32_t)flash_read(f1, d1, (offset << 1) + 1);
	return result;
}

static void cdc_flush(void);
static inline void cdq_sync(void);
static void gfxflash_w(uint32_t offset, uint32_t data, uint32_t mask)
{
	cdq_sync();   // (the queued transfers read the flash)
	cdc_flush();   // (the character DMA's source may change)
	if (b.cram_gfxflash_bank & 1)
		offset += 0x200000 / 4;
	int s, c0;
	if (!gfx_chips(&s, &c0))
		return;
	flash_t *f0 = &b.flash[s][c0], *f1 = &b.flash[s][c0 | 1];
	uint8_t *d0 = s_chip[s][c0], *d1 = s_chip[s][c0 | 1];
	if (mask & 0xff000000) flash_write(f0, d0, offset << 1, (uint8_t)(data >> 24));
	if (mask & 0x00ff0000) flash_write(f1, d1, offset << 1, (uint8_t)(data >> 16));
	if (mask & 0x0000ff00) flash_write(f0, d0, (offset << 1) + 1, (uint8_t)(data >> 8));
	if (mask & 0x000000ff) flash_write(f1, d1, (offset << 1) + 1, (uint8_t)data);
}

// ---- sound (cps3_sound_device) ----
static void snd_render(int n)
{
	if (n <= 0)
		return;
	if (s_snd_n + n > SND_CAP)
		n = SND_CAP - s_snd_n;
	int32_t *out = s_snd + 2 * s_snd_n;
	memset(out, 0, sizeof(int32_t) * 2 * (size_t)n);
	for (int i = 0; i < 16; i++) {
		if (!(b.key & (1 << i)))
			continue;
		voice_t *vptr = &b.voice[i];
		uint32_t start = (vptr->regs[1] >> 16 & 0x0000ffff) | (vptr->regs[1] << 16 & 0xffff0000);
		uint32_t end = (vptr->regs[5] >> 16 & 0x0000ffff) | (vptr->regs[5] << 16 & 0xffff0000);
		uint32_t loop = (vptr->regs[3] & 0x0000ffff) | (vptr->regs[4] << 16 & 0xffff0000);
		const int loop_enable = vptr->regs[2] & 1;
		const uint32_t step = vptr->regs[3] >> 16 & 0xffff;
		const int16_t vol_l = (int16_t)(vptr->regs[7] & 0xffff);
		const int16_t vol_r = (int16_t)(vptr->regs[7] >> 16 & 0xffff);
		uint32_t pos = vptr->pos, frac = vptr->frac;
		start -= 0x400000;
		end -= 0x400000;
		loop -= 0x400000;
		for (int j = 0; j < n; j++) {
			pos += frac >> 12;
			frac &= 0xfff;
			if (start + pos >= end) {
				if (loop_enable)
					pos = (pos + loop) - end;
				else
					break;
			}
			const int32_t sample = (int8_t)u5(start + pos);
			frac += step;
			out[2 * j] += sample * vol_r;       // (the stream's output 1 goes left)
			out[2 * j + 1] += sample * vol_l;
		}
		vptr->pos = pos;
		vptr->frac = frac;
	}
	s_snd_n += n;
}

// m_stream->update(): the samples up to the CPU's clock
static void snd_update(void)
{
	const int64_t now = sh2_total_cycles(&s_cpu);
	if (now <= b.snd_last)
		return;
	b.snd_frac += (uint64_t)(now - b.snd_last) * XTAL;
	b.snd_last = now;
	const uint32_t n = rbx_div64_small(b.snd_frac, SND_DEN);
	b.snd_frac -= (uint64_t)n * SND_DEN;
	snd_render((int)n);
}

static void sound_w(uint32_t offset, uint32_t data, uint32_t mask)
{
	snd_update();
	if (offset < 0x80) {
		uint32_t *r = &b.voice[offset / 8].regs[offset & 7];
		*r = (*r & ~mask) | (data & mask);
	} else if (offset == 0x80) {
		const uint16_t key = (uint16_t)(data >> 16);
		for (int i = 0; i < 16; i++)
			if ((key & (1 << i)) && !(b.key & (1 << i))) {
				b.voice[i].frac = 0;
				b.voice[i].pos = 0;
			}
		b.key = key;
	}
}

static uint32_t sound_r(uint32_t offset, uint32_t mask)
{
	snd_update();
	if (offset < 0x80)
		return b.voice[offset / 8].regs[offset & 7] & mask;
	if (offset == 0x80)
		return (uint32_t)b.key << 16;
	return 0;
}

// ---- video registers, DMA ----
static void arm_dma_timer(void)
{
	b.dma_timer = sh2_total_cycles(&s_cpu) + CPU_CLOCK / 10000;   // 100 us
	sh2_end_timeslice(&s_cpu);
}

static inline uint16_t get_fade(int c, int f)
{
	if (f & 0x40)
		c = (f & 0x20) ? ((((c ^ 0x1f) * (~f & 0x1f)) >> 5) ^ 0x1f) : (c * (f & 0x1f) >> 5);
	return (uint16_t)c;
}

// (the colour's RGB is taken from colourram when drawing: MAME's m_mame_colours is always its r, g, b << 3)
static uint8_t s_pal_dirty[0x20000 / 256];
static void set_colour(int colournum, uint16_t data, uint32_t fadeval)
{
	int r = data & 0x1f, g = (data >> 5) & 0x1f, bl = (data >> 10) & 0x1f;
	if (fadeval & 0x40400040) {
		r = get_fade(r, (int)((fadeval & 0x7f000000) >> 24));
		g = get_fade(g, (int)((fadeval & 0x007f0000) >> 16));
		bl = get_fade(bl, (int)(fadeval & 0x0000007f));
		data = (uint16_t)((data & 0x8000) | r | (g << 5) | (bl << 10));
	}
	colournum &= 0x1ffff;
	b.colourram[colournum] = data;
	s_pal_dirty[colournum >> 8] = 1;
}

static void palettedma_w(uint32_t offset, uint32_t data, uint32_t mask)
{
	if (offset == 0) {
		b.paldma_source = (b.paldma_source & ~mask) | (data & mask);
		b.paldma_realsource = (b.paldma_source << 1) - 0x400000;
	} else if (offset == 1)
		b.paldma_dest = (b.paldma_dest & ~mask) | (data & mask);
	else if (offset == 2)
		b.paldma_fade = (b.paldma_fade & ~mask) | (data & mask);
	else if (offset == 3) {
		b.paldma_other2 = (b.paldma_other2 & ~mask) | (data & mask);
		if (mask & 0xff000000)
			b.paldma_length = (data & 0xffff0000) >> 16;
		if ((mask & 0x000000ff) && (data & 0x0002)) {
			const uint32_t dmalen = b.paldma_length | ((data & 1) << 16);
			for (uint32_t i = 0; i < dmalen; i++) {
				const uint32_t h = ((b.paldma_realsource >> 1) + i) ^ 1;   // (u16 index, BYTE_XOR_BE)
				const uint16_t coldata = (uint16_t)(u5(2 * h) | u5(2 * h + 1) << 8);
				set_colour((int)((b.paldma_dest + i) ^ 1), coldata, b.paldma_fade);
			}
			b.dma_status |= 4;
			arm_dma_timer();
		}
	}
}

static inline uint32_t process_byte(uint8_t real_byte, uint32_t destination, int max_length)
{
	destination &= 0x7fffff;
	if (real_byte & 0x40) {   // (MAME's loop with the run length and the byte kept in locals)
		uint32_t tranfercount = 0;
		int32_t rle = (real_byte & 0x3f) + 1;
		const uint8_t v = (uint8_t)(b.last_normal_byte & 0x3f);
		while (rle) {
			cram_wb(((destination + tranfercount) & 0x7fffff) ^ 3, v);
			tranfercount++;
			rle--;
			max_length--;
			if ((destination + tranfercount) > 0x7fffff) {
				b.rle_length = rle;
				return (uint32_t)max_length;
			}
		}
		b.rle_length = 0;
		return tranfercount;
	}
	cram_wb((destination & 0x7fffff) ^ 3, real_byte);
	b.last_normal_byte = real_byte;
	return 1;
}

// the page of the character RAM the DMA writes (cram_wb for a whole page)
static inline uint8_t *cram_page_w(uint32_t p)
{
	if (UNLIKELY_COW(s_cram_ev[p] == cram_live(p) && (s_dma_drawing || s_cowtest)) && cram_wr(p << 10) == cram_live(p))
		s_dma_drawing = 0;
	return (uint8_t *)s_cram_ev[p];
}

// do_char_dma when the destination cannot pass the end of the RAM (a run may write past the length, by 64 at most):
// MAME's loop with its state in locals, the table's 256 bytes read once, the destination page looked up once a page
static uint32_t char_dma_fast(uint32_t src, uint32_t d, uint32_t len)
{
	const uint32_t d_start = d;
	uint8_t tab[256];
	for (uint32_t i = 0; i < 256; i++)
		tab[i] = u5_dma(b.current_table_address + i);
	int32_t last = 0, rem = (int32_t)len;
	// the destination: base[d ^ 3] is the host byte of d inside the page ending at pend (looked up when d reaches it)
	uint8_t *base = NULL;
	uint32_t pend = d;
	// the source read ahead in blocks (a plain read: past its end it does no harm)
	uint8_t sb[256];
	uint32_t sbi = sizeof(sb);
#define CDMA_PAGE() do { if (d == pend) { const uint32_t pa_ = d & ~0xfffu; base = cram_page_w(pa_ >> 12) - pa_; 	pend = pa_ + 0x1000; } } while (0)
#define CDMA_PUT(v) do { CDMA_PAGE(); base[d ^ 3] = (v); d++; } while (0)
#define CDMA_EMIT(rb_) do { const uint8_t rb = (rb_); 	if (rb & 0x40) { 		int n = (rb & 0x3f) + 1; 		const uint8_t v = (uint8_t)(last & 0x3f); 		rem -= n; 		while (n && (d & 3)) { CDMA_PUT(v); n--; } 		const uint32_t w = v * 0x01010101u;   /* (the aligned longs: the same byte, the order inside no matter) */ 		while (n >= 4) { CDMA_PAGE(); *(uint32_t *)(base + d) = w; d += 4; n -= 4; } 		while (n) { CDMA_PUT(v); n--; } 	} else { CDMA_PUT(rb); last = rb; rem--; } 	if (rem <= 0) goto done; } while (0)
	for (;;) {
		if (sbi == sizeof(sb)) {
			const uint32_t p = src >> 22;
			if (p < (uint32_t)s_ngpair && ((src + (uint32_t)sizeof(sb) - 1) >> 22) == p && !(src & 1)) {
				// (u5_dma(a): chip a & 1 of the pair a >> 22, byte (a >> 1) & 0x1fffff)
				const uint8_t *c0 = s_gpair[0][p] + ((src >> 1) & 0x1fffff), *c1 = s_gpair[1][p] + ((src >> 1) & 0x1fffff);
				for (uint32_t k = 0; k < sizeof(sb) / 2; k++) {
					sb[2 * k] = c0[k];
					sb[2 * k + 1] = c1[k];
				}
			} else
				for (uint32_t k = 0; k < sizeof(sb); k++)
					sb[k] = u5_dma(src + k);
			src += sizeof(sb);
			sbi = 0;
		}
		const uint8_t c = sb[sbi++];
		if (c & 0x80) {
			CDMA_EMIT(tab[(c & 0x7f) * 2]);
			CDMA_EMIT(tab[(c & 0x7f) * 2 + 1]);
		} else
			CDMA_EMIT(c);
	}
done:
	b.last_normal_byte = last;
	b.rle_length = 0;
	return d - d_start;   // (the bytes written)
#undef CDMA_EMIT
#undef CDMA_PUT
#undef CDMA_PAGE
}

typedef uint32_t u32u __attribute__((aligned(1), may_alias));   // (an unaligned long)
static int s_dma_new;   // RBX_CPS3_NEWDMA: char_dma_fast2 (a test: fewer mispredicted branches, more instructions)

// [lo, hi) of the page at pa from guest-order bytes (buf[k]: the guest byte pa + k): the page's host-order longs
static void cdma_put_page(uint32_t pa, const uint8_t *buf, uint32_t lo, uint32_t hi)
{
	uint8_t *pg = cram_page_w(pa >> 12);
	uint32_t k = lo;
	for (; k < hi && (k & 3); k++)
		pg[k ^ 3] = buf[k];
	for (; k + 4 <= hi; k += 4)
		*(uint32_t *)(pg + k) = __builtin_bswap32(*(const uint32_t *)(buf + k));
	for (; k < hi; k++)
		pg[k ^ 3] = buf[k];
}

// char_dma_fast decoding into a page of guest-order bytes (a run: 8-byte stores, past its end into what comes next),
// put into the character RAM a page at a time. The same bytes written, the same state left
static uint32_t char_dma_fast2(uint32_t src, uint32_t d, uint32_t len)
{
	const uint32_t d_start = d;
	uint8_t tab[256];
	for (uint32_t i = 0; i < 256; i++)
		tab[i] = u5_dma(b.current_table_address + i);
	// an input byte's tokens: the first, the second (bit 16: there is one)
	uint32_t pt[256];
	for (uint32_t c = 0; c < 256; c++)
		pt[c] = c & 0x80 ? (uint32_t)tab[(c & 0x7f) * 2] | (uint32_t)tab[(c & 0x7f) * 2 + 1] << 8 | 1u << 16 : c;
	uint32_t pbuf[(4096 + 256) / 4];
	uint8_t *const pb = (uint8_t *)pbuf, *const pend = pb + 4096;
	uint32_t pa = d & ~0xfffu, lo = d & 0xfff;
	uint8_t *o = pb + lo;
	int32_t rem = (int32_t)len;
	uint32_t last = 0;
	uint8_t sb[256];
	uint32_t sbi = sizeof(sb);
	// (a token, without branches on its kind: 16 bytes stored, of the byte or of the run's; the length checked after
	// each, as MAME. en = 0: no token, nothing changes)
#define CDMA2_TOK(t_, en_) do { const uint32_t t = (t_), en = (en_), isr = t & 0x40; 	uint32_t n = isr ? (t & 0x3f) + 1 : 1; 	n = en ? n : 0; 	const uint32_t w = (isr ? (last & 0x3f) : t) * 0x01010101u; 	((u32u *)o)[0] = w; 	((u32u *)o)[1] = w; 	((u32u *)o)[2] = w; 	((u32u *)o)[3] = w; 	if (UNLIKELY_COW(n > 16)) 		for (uint32_t i_ = 16; i_ < n; i_ += 8) { ((u32u *)(o + i_))[0] = w; ((u32u *)(o + i_))[1] = w; } 	last = en && !isr ? t : last; 	o += n; 	rem -= (int32_t)n; 	if (UNLIKELY_COW(rem <= 0)) 		goto done; } while (0)
	for (;;) {
		if (UNLIKELY_COW(sbi == sizeof(sb))) {
			const uint32_t p = src >> 22;
			if (p < (uint32_t)s_ngpair && ((src + (uint32_t)sizeof(sb) - 1) >> 22) == p && !(src & 1)) {
				const uint8_t *c0 = s_gpair[0][p] + ((src >> 1) & 0x1fffff), *c1 = s_gpair[1][p] + ((src >> 1) & 0x1fffff);
				for (uint32_t k = 0; k < sizeof(sb) / 2; k++) {
					sb[2 * k] = c0[k];
					sb[2 * k + 1] = c1[k];
				}
			} else
				for (uint32_t k = 0; k < sizeof(sb); k++)
					sb[k] = u5_dma(src + k);
			src += sizeof(sb);
			sbi = 0;
		}
		const uint32_t e = pt[sb[sbi++]];
		CDMA2_TOK(e & 0xff, 1);
		CDMA2_TOK((e >> 8) & 0xff, e >> 16);
		if (UNLIKELY_COW(o >= pend)) {   // (the page is full: put, what went past it to the start of the next)
			cdma_put_page(pa, pb, lo, 4096);
			const uint32_t over = (uint32_t)(o - pend);
			for (uint32_t k = 0; k < over; k++)
				pb[k] = pend[k];
			o = pb + over;
			pa += 4096;
			lo = 0;
		}
	}
done:
	{
		uint32_t hi = (uint32_t)(o - pb);
		if (hi > 4096) {   // (the last token went past the page)
			cdma_put_page(pa, pb, lo, 4096);
			hi -= 4096;
			for (uint32_t k = 0; k < hi; k++)
				pb[k] = pend[k];
			pa += 4096;
			lo = 0;
		}
		if (hi > lo)
			cdma_put_page(pa, pb, lo, hi);
		b.last_normal_byte = (int32_t)last;
		b.rle_length = 0;
		return pa + hi - d_start;   // (the bytes written)
	}
#undef CDMA2_TOK
}

// The character DMA's output is a function of its source, length and table (the flash being unchanged): the large
// ones are kept (by recency, up to CDC_BYTES) and written again from the copy when the game repeats one (a new round
// loads the same characters, sometimes to another place)
#define CDC_N 32
#define CDC_MIN 0x10000
#define CDC_BYTES (6u << 20)
typedef struct { uint32_t src, len, tab, written, use; int32_t last; uint8_t *data; } cdc_t;
static cdc_t s_cdc[CDC_N];
static uint32_t s_cdc_bytes, s_cdc_clock;

static void cdc_flush(void)
{
	for (int i = 0; i < CDC_N; i++) {
		free(s_cdc[i].data);
		s_cdc[i].data = NULL;
	}
	s_cdc_bytes = 0;
}

// [d, d + n) of the character RAM (d on a long) from / to host-order longs (the bytes past n in the last long kept)
static void cdc_copy(uint32_t d, uint8_t *buf, uint32_t n, int to_ram)
{
	for (uint32_t o = 0; o < n;) {
		const uint32_t a = d + o, p = a >> 12;
		uint32_t m = 0x1000 - (a & 0xfff);
		if (m > n - o)
			m = n - o;
		uint8_t *pg = to_ram ? cram_page_w(p) : (uint8_t *)s_cram_ev[p];
		const uint32_t whole = m & ~3u;
		if (to_ram)
			memcpy(pg + (a & 0xfff), buf + o, whole);
		else
			memcpy(buf + o, pg + (a & 0xfff), (m + 3) & ~3u);
		if (to_ram)
			for (uint32_t k = whole; k < m; k++)   // (the last long's first bytes)
				pg[(a & 0xfff) + (k ^ 3)] = buf[o + (k ^ 3)];
		o += m;
	}
}

static void char_dma_cached(uint32_t src, uint32_t dst, uint32_t len)
{
	const uint32_t tab = b.current_table_address;
	for (int i = 0; i < CDC_N; i++) {
		cdc_t *e = &s_cdc[i];
		if (e->data && e->src == src && e->len == len && e->tab == tab) {
			cdc_copy(dst, e->data, e->written, 1);
			b.last_normal_byte = e->last;
			b.rle_length = 0;
			e->use = ++s_cdc_clock;
			s_prof.hit_n++;
			return;
		}
	}
	const uint64_t pf_a = s_prof.on ? rbx_clock_us() : 0;
	const uint32_t written = s_dma_new ? char_dma_fast2(src, dst, len) : char_dma_fast(src, dst, len);
	if (s_prof.on)
		s_prof.dec_us += rbx_clock_us() - pf_a, s_prof.out_bytes += written;
	if (len < CDC_MIN)
		return;
	const uint32_t size = (written + 3) & ~3u;
	int v = -1;
	for (;;) {   // (room: the least recently used ones out)
		int old = -1, nused = 0;
		for (int i = 0; i < CDC_N; i++) {
			if (!s_cdc[i].data) {
				if (v < 0)
					v = i;
				continue;
			}
			nused++;
			if (old < 0 || s_cdc[i].use < s_cdc[old].use)
				old = i;
		}
		if (v >= 0 && s_cdc_bytes + size <= CDC_BYTES)
			break;
		if (old < 0)
			return;
		s_cdc_bytes -= (s_cdc[old].written + 3) & ~3u;
		free(s_cdc[old].data);
		s_cdc[old].data = NULL;
		if (v < 0)
			v = old;
	}
	cdc_t *e = &s_cdc[v];
	if (size > CDC_BYTES || !(e->data = malloc(size)))
		return;
	cdc_copy(dst, e->data, written, 0);
	e->src = src, e->len = len, e->tab = tab, e->written = written, e->last = b.last_normal_byte, e->use = ++s_cdc_clock;
	s_cdc_bytes += size;
}

static void do_char_dma(uint32_t real_source, uint32_t real_destination, uint32_t real_length)
{
	if (real_destination < 0x800000 && real_length < 0x800000 && real_destination + real_length + 256 <= 0x800000) {
		char_dma_cached(real_source, real_destination, real_length);
		return;
	}
	b.last_normal_byte = 0;
	b.rle_length = 0;
	int length_remaining = (int)real_length;
	while (length_remaining) {
		uint8_t current_byte = u5_dma(real_source);
		real_source++;
		if (current_byte & 0x80) {
			current_byte &= 0x7f;
			uint8_t real_byte = u5_dma(b.current_table_address + current_byte * 2 + 0);
			uint32_t length_processed = process_byte(real_byte, real_destination, length_remaining);
			length_remaining -= (int)length_processed;
			real_destination += length_processed;
			if (real_destination > 0x7fffff || length_remaining <= 0)
				return;
			real_byte = u5_dma(b.current_table_address + current_byte * 2 + 1);
			length_processed = process_byte(real_byte, real_destination, length_remaining);
			length_remaining -= (int)length_processed;
			real_destination += length_processed;
			if (real_destination > 0x7fffff || length_remaining <= 0)
				return;
		} else {
			const uint32_t length_processed = process_byte(current_byte, real_destination, length_remaining);
			length_remaining -= (int)length_processed;
			real_destination += length_processed;
			if (real_destination > 0x7fffff || length_remaining <= 0)
				return;
		}
	}
}

static uint32_t process_byte8(uint8_t bt, uint32_t dst_offset)
{
	uint32_t l = 0;
	if (b.lastb == b.lastb2) {   // rle
		const int rle = (bt + 1) & 0xff;
		for (int i = 0; i < rle; ++i) {
			cram_wb((dst_offset & 0x7fffff) ^ 3, (uint8_t)b.lastb);
			dst_offset++;
			++l;
		}
		b.lastb2 = 0xffff;
		return l;
	}
	b.lastb2 = b.lastb;
	b.lastb = bt;
	cram_wb((dst_offset & 0x7fffff) ^ 3, bt);
	return 1;
}

static void do_alt_char_dma(uint32_t src, uint32_t real_dest, uint32_t real_length)
{
	const uint32_t start = real_dest;
	uint32_t ds = real_dest;
	b.lastb = 0xfffe;
	b.lastb2 = 0xffff;
	for (;;) {
		uint8_t ctrl = u5_dma(src);
		++src;
		for (int i = 0; i < 8; ++i) {
			uint8_t p = u5_dma(src);
			if (ctrl & 0x80) {
				p &= 0x7f;
				ds += process_byte8(u5_dma(b.current_table_address + p * 2 + 0), ds);
				ds += process_byte8(u5_dma(b.current_table_address + p * 2 + 1), ds);
			} else
				ds += process_byte8(p, ds);
			++src;
			ctrl = (uint8_t)(ctrl << 1);
			if ((ds - start) >= real_length)
				return;
		}
	}
}

static void char_dma_run(uint32_t address)
{
	const uint64_t pf_t = s_prof.on ? rbx_clock_us() : 0;
	s_dma_drawing = __atomic_load_n(&s_cram_drawing, __ATOMIC_ACQUIRE);   // (the drawing reads the character RAM)
	for (int i = 0; i < 0x1000; i += 3) {
		const uint32_t dat1 = *cram_rd((i + 0 + address) & 0x1fffff);
		const uint32_t dat2 = *cram_rd((i + 1 + address) & 0x1fffff);
		const uint32_t dat3 = *cram_rd((i + 2 + address) & 0x1fffff);
		if (dat1 & 0x01000000)
			break;
		const uint32_t dma_command = (dat1 & 0x00e00000) >> 21;
		uint32_t real_length = ((dat1 & 0x001fffff) + 1) << 3;
		uint32_t real_destination = dat2 << 3;
		uint32_t real_source = (dat3 << 1) - 0x400000;
		switch (dma_command) {
		case 4:
			b.current_table_address = real_source;
			break;
		case 2:
			s_prof.dma_bytes += real_length, s_prof.dma_n++;
			do_char_dma(real_source, real_destination, real_length);
			break;
		case 3:
			do_alt_char_dma(real_source, real_destination, real_length);
			break;
		case 0:
			for (; real_length; --real_length) {
				cram_wb((real_destination & 0x7fffff) ^ 3, u5_dma(real_source));
				real_source++;
				real_destination++;
			}
			break;
		default:
			break;
		}
	}
	if (s_prof.on)
		s_prof.dma_us += rbx_clock_us() - pf_t;
}

// The character DMA on the worker (rbx_aux, CPU2 beside the drawing on the Vita; RBX_CPS3_DMA_INLINE: on the
// emulation thread): the emulation queues the transfer (its status and end interrupt do not depend on what it writes)
// and goes on; it waits for the queue to be done before it touches what the transfers read or write: the character
// RAM (the CPU's accesses, the vblank's merge of the copies), the graphics flash (a write), the DMA's registers kept in
// the state (a save, a load, a reset). The worker writes the character RAM as the emulation would (through the copies
// while the frame is being drawn), one transfer after another in order: the results are the same
#define CDQ_N 16
static uint32_t s_cdq[CDQ_N];
static unsigned s_cdq_w, s_cdq_r;   // (written by the emulation / by the job)
static int s_cdq_on;

static void cdq_job(void)
{
	unsigned r = s_cdq_r;
	while (r != __atomic_load_n(&s_cdq_w, __ATOMIC_ACQUIRE)) {
		char_dma_run(s_cdq[r % CDQ_N]);
		__atomic_store_n(&s_cdq_r, ++r, __ATOMIC_RELEASE);
	}
}

// the queued transfers done (before the emulation touches what they use)
static inline void cdq_sync(void)
{
	if (s_cdq_on && __atomic_load_n(&s_cdq_r, __ATOMIC_ACQUIRE) != s_cdq_w) {
		const uint64_t t0 = s_prof.on ? rbx_clock_us() : 0;
		rbx_aux_sync();
		if (s_prof.on)
			s_prof.wait_us += rbx_clock_us() - t0;
	}
}

static void process_character_dma(uint32_t address)
{
	if (s_cdq_on) {
		if (s_cdq_w - __atomic_load_n(&s_cdq_r, __ATOMIC_ACQUIRE) >= CDQ_N)
			rbx_aux_sync();
		s_cdq[s_cdq_w % CDQ_N] = address;
		__atomic_store_n(&s_cdq_w, s_cdq_w + 1, __ATOMIC_RELEASE);
		rbx_aux_kick();
	} else
		char_dma_run(address);
	b.dma_status |= 2;
	arm_dma_timer();
}

static void characterdma_w(uint32_t offset, uint32_t data, uint32_t mask)
{
	if (offset == 0) {
		if (mask & 0x000000ff)
			b.chardma_source = data & 0x0000ffff;
	} else if (offset == 1) {
		b.chardma_other = (b.chardma_other & ~mask) | (data & mask);
		if ((mask & 0xff000000) && ((data >> 16) & 0x0040))
			process_character_dma(b.chardma_source | (b.chardma_other & 0x003f0000));
	}
}

static void spritedma_w(uint16_t data, uint16_t mask)
{
	const uint16_t prev = b.spritelist_dma;
	b.spritelist_dma = (uint16_t)((b.spritelist_dma & ~mask) | (data & mask));
	if (!(b.dma_status & 1) && (b.spritelist_dma & 9) == 8 && (prev & 9) == 9) {
		for (int i = 0; i < 0x2000 / 4; i += 4) {
			memcpy(&b.spritelist[i], &b.spriteram[i], 4 * sizeof(uint32_t));
			const uint32_t dat = b.spriteram[i];
			if (dat & 0x80000000)
				break;
			const uint32_t offs = (dat & 0x00007fff) << 2;
			uint32_t length = (dat & 0x01ff0000) >> 16;
			if (offs + length * 4 > 0x20000)   // (MAME copies past the end of the sprite RAM)
				length = (0x20000 - offs) / 4;
			memcpy(&b.spritelist[offs], &b.spriteram[offs], length * 4 * sizeof(uint32_t));
		}
		memcpy(b.gscroll_buff, b.gscroll, sizeof(b.gscroll_buff));
		b.dma_status |= 1;
		b.spr_timer = sh2_total_cycles(&s_cpu) + CPU_CLOCK / 250000;   // 4 us
		sh2_end_timeslice(&s_cpu);
	}
}

// ---- EEPROM ----
static uint32_t eeprom_r(uint32_t offset, uint32_t mask)
{
	const uint32_t addr = offset * 4;
	if (addr >= 0x100 && addr <= 0x17f) {
		const uint32_t v = b.eeprom[offset - 0x100 / 4];
		b.current_eeprom_read = (uint16_t)((mask & 0xff000000) ? v >> 16 : v);
		return 0;
	}
	if (addr == 0x200)
		return (mask & 0xff000000) ? 0 : b.current_eeprom_read;
	return 0;
}

static void eeprom_w(uint32_t offset, uint32_t data, uint32_t mask)
{
	const uint32_t addr = offset * 4;
	if (addr >= 0x080 && addr <= 0x0ff) {
		uint32_t *e = &b.eeprom[offset - 0x80 / 4];
		*e = (*e & ~mask) | (data & mask);
	}
}

static void ssregs_w(uint32_t offset, uint8_t data)
{
	switch (offset) {
	case 0x07: b.ss_hscroll = (int16_t)((b.ss_hscroll & 0xff00) | data); break;
	case 0x08: b.ss_hscroll = (int16_t)((b.ss_hscroll & 0xff) | (data << 8)); break;
	case 0x10: b.ss_vscroll = (int16_t)((b.ss_vscroll & 0xff00) | data); break;
	case 0x11: b.ss_vscroll = (int16_t)((b.ss_vscroll & 0xff) | (data << 8)); break;
	case 0x12: b.ss_pal_base = data; break;
	default: break;
	}
}

// ---- bus (the accesses missing the page table): a long at a (aligned) with MAME's mem_mask ----
static uint32_t rd32(uint32_t a, uint32_t mask)
{
	if (a >= 0xc0000000) {
		if (a < 0xc0000400)
			return b.cache[(a & 0x3ff) >> 2];
		return 0;
	}
	if (a - 0x06000000 < 0x1000000) {   // program flash (out of read mode, or not fitted)
		const int which = (a >> 23) & 1;
		uint32_t v = flashmain_r(which, (a & 0x7fffff) >> 2, mask);
		if (!s_alt)
			v ^= MASK(a & ~3u);
		return v;
	}
	if (a - 0x04080000 < 0x40000) {   // colour RAM (16-bit)
		const uint32_t o = (a - 0x04080000) >> 1;
		return (uint32_t)b.colourram[o] << 16 | b.colourram[o + 1];
	}
	if (a - 0x04100000 < 0x100000)
		return cdq_sync(), *cram_rd(((b.cram_bank & 7) * 0x100000 / 4 + ((a - 0x04100000) >> 2)) & 0x1fffff);
	if (a - 0x04200000 < 0x200000)
		return gfxflash_r((a - 0x04200000) >> 2, mask);
	if (a - 0x040e0000 < 0x300)
		return sound_r((a - 0x040e0000) >> 2, mask);
	if (a - 0x05040000 < 0x10000) {   // SS RAM (umask 0x00ff00ff)
		const uint32_t o = (a - 0x05040000) >> 1;
		return (uint32_t)b.ss_ram[o & 0x7fff] << 16 | b.ss_ram[(o + 1) & 0x7fff];
	}
	if (a == 0x040c000c)
		return (uint32_t)b.dma_status << 16;
	if (a == 0x05000000)
		return rbx_port_read(s_port_in);
	if (a == 0x05000004)
		return rbx_port_read(s_port_extra);
	if (a - 0x05000a00 < 0x20)
		return 0xffffffff;
	if (a - 0x05001000 < 0x204)
		return eeprom_r((a - 0x05001000) >> 2, mask);
	if (a - 0x03000000 < 0x400)
		return b.fram[(a & 0x3ff) >> 2];
	return 0;
}

static void wr32(uint32_t a, uint32_t data, uint32_t mask)
{
	if (a >= 0xc0000000) {
		if (a < 0xc0000400) {
			const uint32_t o = (a & 0x3ff) >> 2;
			b.cache[o] = (b.cache[o] & ~mask) | (data & mask);
			b.cache_dec[o] = b.cache[o] ^ MASK(o * 4 + 0xc0000000);
			rnd_forget();
		}
		return;
	}
	if (a - 0x06000000 < 0x1000000) {
		flashmain_w((a >> 23) & 1, (a & 0x7fffff) >> 2, data, mask);
		rnd_forget();
		return;
	}
	if (a - 0x04080000 < 0x40000) {
		const uint32_t o = (a - 0x04080000) >> 1;
		if (mask & 0xffff0000) {
			const uint16_t m = (uint16_t)(mask >> 16);
			const uint16_t v = (uint16_t)((b.colourram[o] & ~m) | ((data >> 16) & m));
			b.colourram[o] = v;
			set_colour((int)o, v, 0);
		}
		if (mask & 0x0000ffff) {
			const uint16_t m = (uint16_t)mask;
			const uint16_t v = (uint16_t)((b.colourram[o + 1] & ~m) | (data & m));
			b.colourram[o + 1] = v;
			set_colour((int)o + 1, v, 0);
		}
		return;
	}
	if (a - 0x04100000 < 0x100000) {
		cdq_sync();
		uint32_t *p = cram_wr(((b.cram_bank & 7) * 0x100000 / 4 + ((a - 0x04100000) >> 2)) & 0x1fffff);
		*p = (*p & ~mask) | (data & mask);
		return;
	}
	if (a - 0x04200000 < 0x200000) {
		gfxflash_w((a - 0x04200000) >> 2, data, mask);
		return;
	}
	if (a - 0x040e0000 < 0x300) {
		sound_w((a - 0x040e0000) >> 2, data, mask);
		return;
	}
	if (a - 0x05040000 < 0x10000) {
		const uint32_t o = (a - 0x05040000) >> 1;
		if (mask & 0x00ff0000)
			b.ss_ram[o & 0x7fff] = (uint8_t)(data >> 16);
		if (mask & 0x000000ff)
			b.ss_ram[(o + 1) & 0x7fff] = (uint8_t)data;
		return;
	}
	if (a - 0x05050000 < 0x2c) {
		const uint32_t o = (a - 0x05050000) >> 1;
		if (mask & 0x00ff0000)
			ssregs_w(o, (uint8_t)(data >> 16));
		if (mask & 0x000000ff)
			ssregs_w(o + 1, (uint8_t)data);
		return;
	}
	if (a - 0x040c0000 < 0x20) {
		uint32_t *r = &b.gscroll[(a >> 2) & 7];
		*r = (*r & ~mask) | (data & mask);
		return;
	}
	if (a - 0x040c0020 < 0x40) {
		uint32_t *r = &b.tmap_regs[((a - 0x040c0020) >> 2) & 15];
		*r = (*r & ~mask) | (data & mask);
		return;
	}
	if (a - 0x040c0060 < 0x20) {
		uint32_t *r = &b.crtc_zoom[((a - 0x040c0060) >> 2) & 7];
		*r = (*r & ~mask) | (data & mask);
		return;
	}
	if (a == 0x040c0080) {
		if (mask & 0xffff)
			spritedma_w((uint16_t)data, (uint16_t)mask);
		return;
	}
	if (a == 0x040c0084) {
		if (mask & 0xff)
			b.cram_bank = data;   // (MAME: the whole data when bits 0-7 are written)
		return;
	}
	if (a == 0x040c0088) {
		if (mask & 0xff000000)
			b.cram_gfxflash_bank = ((data & 0xffff0000) >> 16) - 2;
		return;
	}
	if (a - 0x040c0094 < 8) {
		characterdma_w((a - 0x040c0094) >> 2, data, mask);
		return;
	}
	if (a - 0x040c00a0 < 0x10) {
		palettedma_w((a - 0x040c00a0) >> 2, data, mask);
		return;
	}
	if (a - 0x05001000 < 0x204) {
		eeprom_w((a - 0x05001000) >> 2, data, mask);
		return;
	}
	if (a == 0x05100000) { sh2_set_irq_line(&s_cpu, 12, SH2_CLEAR); return; }
	if (a == 0x05110000) { sh2_set_irq_line(&s_cpu, 10, SH2_CLEAR); return; }
	if (a == 0x05120000) { sh2_set_irq_line(&s_cpu, 14, SH2_CLEAR); return; }
	if (a == 0x05130000) { sh2_set_irq_line(&s_cpu, 6, SH2_CLEAR); return; }
	if (a - 0x03000000 < 0x400) {
		uint32_t *r = &b.fram[(a & 0x3ff) >> 2];
		*r = (*r & ~mask) | (data & mask);
		return;
	}
}

// ---- idle rounds (a speed-up with the interpreter's exact results) ----
// The games' task scheduler spins through its task slots while every task waits for the vblank (most of the CPU's
// cycles). A round of it (from a backward BT / BF target back to the same CPU state) is recorded once with the main
// and sprite RAM unmapped, so that every RAM long it touches goes through the callbacks, and the instructions it runs
// traced. A round that touches nothing else (no I/O, no on-chip register, no DMA, no interrupt) and leaves the
// registers and every long it touched as they were is a fixed point. A long it raises by one is allowed when it is an
// idle counter: read as a long into a register that, along the traced instructions, is only raised by one (ADD #1),
// stored back as a long, tested for zero (TST Rx,Rx, its value being nonzero) and then written over, never read: the
// round does not depend on its value (nor on more than its being nonzero).
// While the registers, the interrupt lines and the other longs are unchanged, the rounds the slice has room for are
// skipped: their cycles go by, the counters are raised by their number, nothing else changes.
#define RND_SLOTS 4
#define RND_TOUCH 1024
#define RND_TRACE 8192
#define RND_CNT 8
#define RND_REC_MAX 8000   // cycles a recorded round may take
#define RND_HASH 4096
typedef struct {
	uint32_t regs[27];     // r0-r15, sr, gbr, vbr, mach, macl, pr, pending_irq, pending_nmi, internal_irq_level, pc, delay
	int32_t iter;          // cycles of a round (0 = not verified)
	int ntouch, ncnt;
	uint32_t taddr[RND_TOUCH], tval[RND_TOUCH];   // RAM long index, its value (before the round)
	uint8_t tcnt[RND_TOUCH];                      // 1 = an idle counter, 2 = one tested for zero
	int fails, miss, okn, longf;   // okn: rounds verified, longf: recordings that did not come back
	uint32_t hits;
} rnd_t;
static struct rnd_state {
	rnd_t l[RND_SLOTS];
	uint32_t top[RND_SLOTS];
	int next;
	// recording
	int rec;               // slot recorded, -1 = none
	int bad, calls;
	int32_t rec_icount;
	int64_t rec_end;
	uint32_t rec_onchip;
	int ntr;
	uint32_t tr[RND_TRACE];                       // the instructions run (addresses)
	uint8_t nrd[RND_TOUCH], nwr[RND_TOUCH], all32[RND_TOUCH], rdzero[RND_TOUCH], wfirst[RND_TOUCH];   // accesses of each touched long
	int rdi[RND_TOUCH][4], wri[RND_TOUCH];        // their instructions (trace indices)
	uint32_t hkey[RND_HASH];                      // touched longs: RAM long index (~0 = free) -> touch number
	int16_t hval[RND_HASH], hslot[RND_TOUCH];
	uint32_t dead[16];           // loops found not to be idle rounds (a counting loop, I/O ...): not learnt again
	uint32_t good[16];           // loops found to be idle rounds: never put among the dead
	int ndead, ngood;
	int64_t skipped, recorded;   // (statistics: cycles left out, cycles run while recording)
	int nrec;
} *s_rnd;
static int s_rnd_off;

static int rnd_good(uint32_t top)
{
	for (int i = 0; i < 16; i++)
		if (s_rnd->good[i] == top)
			return 1;
	return 0;
}

static void rnd_dead_add(int n)
{
	if (s_rnd->l[n].okn || rnd_good(s_rnd->top[n]))   // (it was an idle round: kept)
		return;
	s_rnd->dead[s_rnd->ndead++ & 15] = s_rnd->top[n];
	s_rnd->top[n] = s_cpu.idle_pc[n] = 1;
	s_rnd->l[n].iter = 0;
}

static void rnd_stop(int ok);
static void rnd_stop_dead(void)   // (a recording of a loop whose state does not come back)
{
	const int n = s_rnd->rec;
	rnd_stop(0);
	if (++s_rnd->l[n].longf >= 4)
		rnd_dead_add(n);
}

static void rnd_regs(const sh2_t *c, uint32_t *r)
{
	memcpy(r, c->r, 16 * sizeof(uint32_t));
	r[16] = c->sr, r[17] = c->gbr, r[18] = c->vbr, r[19] = c->mach, r[20] = c->macl, r[21] = c->pr;
	r[22] = c->pending_irq, r[23] = c->pending_nmi, r[24] = (uint32_t)c->internal_irq_level;
	r[25] = c->pc, r[26] = c->delay;
}

static inline uint32_t *rnd_long(uint32_t i) { return i < 0x80000 / 4 ? &b.mainram[i] : &b.spriteram[i - 0x80000 / 4]; }

// an instruction word as the CPU fetches it (0 = not code we know)
static uint16_t op16(uint32_t a)
{
	a &= ~1u;
	if (a >= 0xc0000000)
		return a < 0xc0000400 ? (uint16_t)(b.cache_dec[(a & 0x3ff) >> 2] >> (16 - 8 * (a & 2))) : 0;
	a &= 0x07ffffff;
	if (a - 0x06000000 < (uint32_t)s_nsimm_prg * 0x800000)
		return (uint16_t)(s_dec[(a - 0x06000000) >> 2] >> (16 - 8 * (a & 2)));
	if (a < 0x80000)
		return sh2_read16(&s_cpu, a);
	if (a - 0x02000000 < 0x80000)
		return (uint16_t)(b.mainram[(a - 0x02000000) >> 2] >> (16 - 8 * (a & 2)));
	return 0;
}

// (before each instruction while recording: a round gone wrong or on too long ends the recording at once)
static void rnd_trace(void *u, uint32_t pc)
{
	(void)u;
	if (s_rnd->bad)
		rnd_stop(0);
	else if (s_rnd->ntr == RND_TRACE || s_rnd->rec_icount - s_cpu.icount > RND_REC_MAX)
		rnd_stop_dead();
	else
		s_rnd->tr[s_rnd->ntr++] = pc;
}

static void rnd_map(int on)
{
	sh2_map(&s_cpu, 0x02000000, 0x0207ffff, on ? b.mainram : NULL, sizeof(b.mainram), 1, 1);
	sh2_map(&s_cpu, 0x04000000, 0x0407ffff, on ? b.spriteram : NULL, sizeof(b.spriteram), 1, 1);
	s_cpu.fetch_trace = on ? NULL : rnd_trace;
}

static void rnd_stop(int ok)
{
	rnd_t *l = &s_rnd->l[s_rnd->rec];
#ifdef RND_DEBUG
	fprintf(stderr, "@%lld stop %d top %08x bad %d calls %d touch %d trace %d\n", (long long)sh2_total_cycles(&s_cpu), ok, s_rnd->top[s_rnd->rec], s_rnd->bad,
	        s_rnd->calls, l->ntouch, s_rnd->ntr);
	for (int i = 0; i < l->ntouch; i++)
		if (*rnd_long(l->taddr[i]) != l->tval[i])
			fprintf(stderr, "  diff %05x %08x -> %08x r%d w%d\n", l->taddr[i] * 4, l->tval[i], *rnd_long(l->taddr[i]),
			        s_rnd->nrd[i], s_rnd->nwr[i]);
#endif
	for (int i = 0; i < l->ntouch; i++)
		s_rnd->hkey[s_rnd->hslot[i]] = ~0u;
	if (!ok) {
		l->iter = 0;
		l->fails++;
	}
	s_rnd->rec = -1;
	s_rnd->recorded += s_rnd->rec_icount - s_cpu.icount;
	s_rnd->nrec++;
	rnd_map(1);
}

// a RAM access while recording (size 1, 2, 4; write); the long it touches, NULL when a is not RAM
static uint32_t *rnd_touch(uint32_t a, int size, int write)
{
	uint32_t i;
	if (a - 0x02000000 < 0x80000)
		i = (a - 0x02000000) >> 2;
	else if (a - 0x04000000 < 0x80000)
		i = 0x80000 / 4 + ((a - 0x04000000) >> 2);
	else if (!write && a - 0x040c0000 < 8)   // (reads 0 always: Red Earth reads it)
		return NULL;
	else {
#ifdef RND_DEBUG
		fprintf(stderr, "  io %c %08x at %08x\n", write ? 'w' : 'r', a, s_rnd->ntr ? s_rnd->tr[s_rnd->ntr - 1] : 0);
#endif
		s_rnd->bad = 1;
		return NULL;
	}
	uint32_t *p = rnd_long(i);
	rnd_t *l = &s_rnd->l[s_rnd->rec];
	int t;
	uint32_t h = (i * 2654435761u) >> 20;
	while (s_rnd->hkey[h] != i && s_rnd->hkey[h] != ~0u)
		h = (h + 1) & (RND_HASH - 1);
	if (s_rnd->hkey[h] == i)
		t = s_rnd->hval[h];
	else {
		if (l->ntouch == RND_TOUCH) {
			s_rnd->bad = 1;
			return p;
		}
		t = l->ntouch++;
		s_rnd->hkey[h] = i;
		s_rnd->hval[h] = (int16_t)t;
		s_rnd->hslot[t] = (int16_t)h;
		l->taddr[t] = i;
		l->tval[t] = *p;
		s_rnd->nrd[t] = s_rnd->nwr[t] = s_rnd->rdzero[t] = 0;
		s_rnd->all32[t] = 1;
		s_rnd->wfirst[t] = write && size == 4;   // (its value before the round not read)
	}
	const int at = s_rnd->ntr - 1;   // (the instruction making the access: the last one fetched)
	if (write) {
		s_rnd->wri[t] = at;
		if (s_rnd->nwr[t] < 255) s_rnd->nwr[t]++;
	} else {
		if (s_rnd->nrd[t] < 4)
			s_rnd->rdi[t][s_rnd->nrd[t]] = at;
		if (s_rnd->nrd[t] < 255) s_rnd->nrd[t]++;
		s_rnd->rdzero[t] |= *p == 0;
	}
	if (size != 4)
		s_rnd->all32[t] = 0;
	return p;
}

// the general registers an instruction reads and writes (bit n = Rn); 0 = it runs, -1 = an instruction not known
// here (SLEEP, TRAPA, illegal)
static int sh2_op_regs(uint16_t op, uint32_t *rd, uint32_t *wr)
{
	const uint32_t n = 1u << ((op >> 8) & 15), m = 1u << ((op >> 4) & 15), r0 = 1;
	*rd = *wr = 0;
	switch (op >> 12) {
	case 0x0:
		switch (op & 0xf) {
		case 2: if ((op & 0xf0) <= 0x20) { *wr = n; return 0; } return -1;   // STC SR / GBR / VBR,Rn
		case 3: if ((op & 0xf0) == 0x00 || (op & 0xf0) == 0x20) { *rd = n; return 0; } return -1;   // BSRF BRAF
		case 4: case 5: case 6: *rd = m | n | r0; return 0;                  // MOV.x Rm,@(R0,Rn)
		case 7: *rd = m | n; return 0;                                       // MUL.L
		case 8: return (op & 0xfcf) == 0x008 && (op & 0x30) != 0x30 ? 0 : -1;   // CLRT SETT CLRMAC
		case 9: if ((op & 0xfef) == 0x009) return 0;                         // NOP DIV0U
			if ((op & 0xf0) == 0x20) { *wr = n; return 0; } return -1;   // MOVT
		case 0xa: if ((op & 0xf0) <= 0x20) { *wr = n; return 0; } return -1; // STS MACH / MACL / PR,Rn
		case 0xb: return (op & 0xfff) == 0x00b || (op & 0xfff) == 0x02b ? 0 : -1;   // RTS RTE
		case 0xc: case 0xd: case 0xe: *rd = m | r0; *wr = n; return 0;       // MOV.x @(R0,Rm),Rn
		case 0xf: *rd = *wr = m | n; return 0;                               // MAC.L
		default: return -1;
		}
	case 0x1: *rd = m | n; return 0;                                             // MOV.L Rm,@(disp,Rn)
	case 0x2:
		switch (op & 0xf) {
		case 0: case 1: case 2: case 7: case 8: case 0xc: case 0xe: case 0xf: *rd = m | n; return 0;
		case 4: case 5: case 6: case 9: case 0xa: case 0xb: case 0xd: *rd = m | n; *wr = n; return 0;
		default: return -1;
		}
	case 0x3:
		switch (op & 0xf) {
		case 0: case 2: case 3: case 5: case 6: case 7: case 0xd: *rd = m | n; return 0;
		case 4: case 8: case 0xa: case 0xb: case 0xc: case 0xe: case 0xf: *rd = m | n; *wr = n; return 0;
		default: return -1;
		}
	case 0x4:
		switch (op & 0xff) {
		case 0x00: case 0x01: case 0x04: case 0x05: case 0x08: case 0x09: case 0x18: case 0x19: case 0x28: case 0x29:
		case 0x20: case 0x21: case 0x24: case 0x25: case 0x10:
		case 0x02: case 0x12: case 0x22: case 0x03: case 0x13: case 0x23:
		case 0x06: case 0x16: case 0x26: case 0x07: case 0x17: case 0x27:
			*rd = *wr = n; return 0;
		case 0x11: case 0x15: case 0x0a: case 0x1a: case 0x2a: case 0x0e: case 0x1e: case 0x2e: case 0x1b:
		case 0x0b: case 0x2b:                                                 // JSR JMP @Rn
			*rd = n; return 0;
		default:
			if ((op & 0xf) == 0xf) { *rd = *wr = m | n; return 0; }           // MAC.W
			return -1;
		}
	case 0x5: *rd = m; *wr = n; return 0;                                        // MOV.L @(disp,Rm),Rn
	case 0x6:
		if ((op & 0xf) >= 4 && (op & 0xf) <= 6) { *rd = m; *wr = m | n; return 0; }   // MOV.x @Rm+,Rn
		*rd = m; *wr = n; return 0;
	case 0x7: *rd = *wr = n; return 0;                                           // ADD #imm,Rn
	case 0x8:
		switch ((op >> 8) & 15) {
		case 0x0: case 0x1: *rd = r0 | m; return 0;                           // MOV.x R0,@(disp,Rn) (n in bits 4-7)
		case 0x4: case 0x5: *rd = m; *wr = r0; return 0;                      // MOV.x @(disp,Rm),R0
		case 0x8: *rd = r0; return 0;                                         // CMP/EQ #imm,R0
		case 0x9: case 0xb: case 0xd: case 0xf: return 0;                     // BT BF BT/S BF/S
		default: return -1;
		}
	case 0x9: *wr = n; return 0;                                                 // MOV.W @(disp,PC),Rn
	case 0xa: case 0xb: return 0;                                                // BRA, BSR
	case 0xc:
		switch ((op >> 8) & 15) {
		case 0x0: case 0x1: case 0x2: case 0x8: case 0xc: case 0xd: case 0xe: case 0xf: *rd = r0; return 0;
		case 0x4: case 0x5: case 0x6: case 0x7: *wr = r0; return 0;
		case 0x9: case 0xa: case 0xb: *rd = *wr = r0; return 0;
		default: return -1;                                                   // TRAPA
		}
	case 0xd: case 0xe: *wr = n; return 0;                                       // MOV.L @(disp,PC),Rn; MOV #imm,Rn
	default: return -1;
	}
}

// the register a long load writes (MOV.L @Rm,Rn / @Rm+,Rn / @(disp,Rm),Rn / @(R0,Rm),Rn / @(disp,GBR),R0), -1 = none
static int load_dest(uint16_t op)
{
	if ((op & 0xf00f) == 0x6002 || (op & 0xf000) == 0x5000 || (op & 0xf00f) == 0x000e)
		return (op >> 8) & 15;
	if ((op & 0xf00f) == 0x6006 && ((op >> 8) & 15) != ((op >> 4) & 15))
		return (op >> 8) & 15;
	if ((op & 0xff00) == 0xc600)
		return 0;
	return -1;
}

// a long store of Rx (MOV.L Rx,@Rn / @-Rn / @(disp,Rn) / @(R0,Rn) / @(disp,GBR) for R0), its address not from Rx
static int store_of(uint16_t op, int x)
{
	const int n = (op >> 8) & 15, m = (op >> 4) & 15;
	if ((op & 0xf00f) == 0x2002 || (op & 0xf00f) == 0x2006 || (op & 0xf000) == 0x1000)
		return m == x && n != x;
	if ((op & 0xf00f) == 0x0006)
		return m == x && n != x && x != 0;
	return (op & 0xff00) == 0xc200 && x == 0;
}

// the value loaded at trace index i (into Rx) along the trace: 1 = raised by one and stored back at wr only,
// 2 = tested for zero only, 0 = anything else (used otherwise, or still live when the round ends)
static int rnd_follow(int i, int wr)
{
	const int x = load_dest(op16(s_rnd->tr[i]));
	if (x < 0)
		return 0;
	const uint32_t bit = 1u << x;
	int inc = 0, st = 0, tst = 0;
	for (int j = i + 1; j < s_rnd->ntr; j++) {
		const uint16_t op = op16(s_rnd->tr[j]);
		if (j == wr) {   // (the counter's store)
			if (!inc || st || !store_of(op, x))
				return 0;
			st = 1;
			continue;
		}
		if (op == (0x7001 | x << 8) && !inc && !st && !tst) {
			inc = 1;
			continue;
		}
		if (op == (0x2008 | x << 8 | x << 4) && !inc) {
			tst = 1;
			continue;
		}
		uint32_t rd, wrm;
		if (sh2_op_regs(op, &rd, &wrm) < 0 || (rd & bit))
			return 0;
		if (wrm & bit) {
			if (inc)
				return st ? 1 : 0;
			return tst ? 2 : 0;
		}
	}
	return 0;
}

// a touched long left one higher: an idle counter? 1 = yes, 2 = yes, also tested for zero
static int rnd_counter(int t, uint32_t now)
{
	const rnd_t *l = &s_rnd->l[s_rnd->rec];
	const int nrd = s_rnd->nrd[t];
	if (now != l->tval[t] + 1 || nrd < 1 || nrd > 4 || s_rnd->nwr[t] != 1 || !s_rnd->all32[t] || s_rnd->rdzero[t])
		return 0;
	int inc = 0, tst = 0;
	for (int r = 0; r < nrd; r++) {
		const int k = rnd_follow(s_rnd->rdi[t][r], s_rnd->wri[t]);
		if (k == 1 && !inc)
			inc = 1;
		else if (k == 2)
			tst = 1;
		else
			return 0;
	}
	return inc ? 1 + tst : 0;
}

static int rnd_same_mem(const rnd_t *l)
{
	for (int i = 0; i < l->ntouch; i++)
		if (!l->tcnt[i] && *rnd_long(l->taddr[i]) != l->tval[i])
			return 0;
	return 1;
}

static void rnd_cb(sh2_t *c)
{
	// (a BT / BF taken: pc is the top; a BRA: delay is, pc the slot's address)
	const uint32_t at = c->delay ? c->delay : c->pc;
	int n = 0;
	while (s_rnd->top[n] != at)
		if (++n == RND_SLOTS)
			return;
	rnd_t *l = &s_rnd->l[n];
	uint32_t regs[27];
	if (s_rnd->rec >= 0) {
		if (s_rnd->rec != n) {
			if (s_rnd->rec_icount - c->icount > RND_REC_MAX)   // (another loop's round runs long: drop it)
				rnd_stop_dead();
			return;
		}
		if (s_rnd->bad || c->onchip_n != s_rnd->rec_onchip || c->slice_end != s_rnd->rec_end ||
		    c->dma_active[0] || c->dma_active[1]) {
			rnd_stop(0);
			return;
		}
		rnd_regs(c, regs);
		if (memcmp(regs, l->regs, sizeof(regs))) {
			if (++s_rnd->calls > 64 || s_rnd->rec_icount - c->icount > RND_REC_MAX)
				rnd_stop_dead();
			return;
		}
		int ok = s_rnd->rec_icount > c->icount;
		l->ncnt = 0;
		for (int i = 0; i < l->ntouch && ok; i++) {
			const uint32_t now = *rnd_long(l->taddr[i]);
			l->tcnt[i] = 0;
			if (now == l->tval[i])
				continue;
			if (s_rnd->wfirst[i]) {   // (a scratch long: the round leaves it at what it writes, whatever it was)
				l->tval[i] = now;
				continue;
			}
			if (l->ncnt < RND_CNT && (l->tcnt[i] = (uint8_t)rnd_counter(i, now)) != 0) {
				l->ncnt++;
			} else
				ok = 0;
		}
		if (ok) {
			l->iter = s_rnd->rec_icount - c->icount;
			if (!l->okn++ && !rnd_good(s_rnd->top[n]))
				s_rnd->good[s_rnd->ngood++ & 15] = s_rnd->top[n];
		}
		rnd_stop(ok);
		if (!ok)
			return;
	} else {
		if (c->dma_active[0] || c->dma_active[1])
			return;
		rnd_regs(c, regs);
		const int same = !memcmp(regs, l->regs, sizeof(regs));
		if (l->iter && !same && ++l->miss < 32)   // (the round's state comes back in a few calls)
			return;
		if (!l->iter || !same || !rnd_same_mem(l)) {
			// record a round from here (a candidate failing often is tried less and less)
			if (l->fails > 8 && (++l->hits & ((1u << (l->fails > 20 ? 12 : l->fails - 8)) - 1)))
				return;
			if (c->icount < 4000)
				return;
			memcpy(l->regs, regs, sizeof(regs));
			l->iter = 0;
			l->ntouch = 0;
			s_rnd->rec = n;
			s_rnd->bad = 0;
			s_rnd->calls = 0;
			s_rnd->ntr = 0;
			s_rnd->rec_icount = c->icount;
			s_rnd->rec_end = c->slice_end;
			s_rnd->rec_onchip = c->onchip_n;
			rnd_map(0);
			return;
		}
	}
	l->fails = 0;
	l->miss = 0;
	int32_t k = (c->icount - l->iter - 2) / l->iter;
	for (int i = 0; i < l->ntouch && k > 0; i++)
		if (l->tcnt[i] == 2) {   // (tested for zero: it stays nonzero)
			const uint32_t v = *rnd_long(l->taddr[i]);
			if (!v || v + (uint32_t)k < v)
				k = 0;
		}
	if (k > 0) {
		c->icount -= k * l->iter;
		s_rnd->skipped += (int64_t)k * l->iter;
		for (int i = 0; i < l->ntouch; i++)
			if (l->tcnt[i])
				*rnd_long(l->taddr[i]) += (uint32_t)k;
	}
}

// between slices: a recording left unfinished is dropped; look for a loop around the CPU's pc
static void rnd_learn(int look)
{
	if (s_rnd_off)
		return;
	if (s_rnd->rec >= 0)
		rnd_stop(0);
	if (!look || s_cpu.delay)
		return;
	const uint32_t pc = s_cpu.pc;
	if ((pc >> 24) != 0x06)
		return;
	for (int i = 0; i < RND_SLOTS; i++)
		if (pc - s_rnd->top[i] < 0x100)
			return;
	// the first backward BT / BF / BRA after pc whose target is at or before pc
	for (uint32_t a = pc; a < pc + 0x200; a += 2) {
		const uint16_t op = op16(a);
		uint32_t t;
		if ((op & 0xfd80) == 0x8980)
			t = a + 4 + (uint32_t)(int8_t)(op & 0xff) * 2;
		else if ((op & 0xf800) == 0xa800)
			t = a + 4 + (uint32_t)(((int32_t)(op & 0xfff) << 20) >> 19);
		else
			continue;
		if (t > pc)
			continue;
		for (int i = 0; i < 16; i++)
			if (s_rnd->dead[i] == t)
				return;
		// (the slot of a loop never verified, the one failing most; else one not verified now; else in turn)
		int n = -1;
		for (int i = 0; i < RND_SLOTS; i++)
			if (!s_rnd->l[i].okn && !rnd_good(s_rnd->top[i]) && (n < 0 || s_rnd->l[i].fails > s_rnd->l[n].fails))
				n = i;
		for (int i = 0; i < RND_SLOTS && n < 0; i++)
			if (!s_rnd->l[i].iter)
				n = i;
		if (n < 0) {
			n = s_rnd->next;
			s_rnd->next = (n + 1) % RND_SLOTS;
		}
		if (s_rnd->l[n].fails > 8 && s_rnd->top[n] != 1)   // (a loop failing often: not again)
			rnd_dead_add(n);
		memset(&s_rnd->l[n], 0, sizeof(s_rnd->l[n]));
		s_rnd->top[n] = t;
		s_cpu.idle_pc[n] = t;
		return;
	}
}

static void rnd_reset(void)
{
	s_rnd_off = getenv("RBX_CPS3_NOIDLE") != NULL;
	memset(s_rnd, 0, sizeof(*s_rnd));
	memset(s_rnd->hkey, 0xff, sizeof(s_rnd->hkey));
	s_rnd->rec = -1;
	for (int i = 0; i < RND_SLOTS; i++)
		s_rnd->top[i] = s_cpu.idle_pc[i] = 1;
	s_cpu.idle = rnd_cb;
	s_cpu.idle_user = NULL;
}

// the code or a constant a round may have read changed (a flash or cache RAM write)
static void rnd_forget(void)
{
	if (s_rnd)
		for (int i = 0; i < RND_SLOTS; i++)
			s_rnd->l[i].iter = 0;
}

#define RND_REC (s_rnd->rec >= 0)
#define RND_R(a, T, size, shift) do { if (RND_REC) { uint32_t *p_ = rnd_touch(a, size, 0); \
	if (p_) return (T)(*p_ >> (shift)); } } while (0)

static uint8_t r8(void *u, uint32_t a)
{
	(void)u;
	RND_R(a, uint8_t, 1, 24 - 8 * (a & 3));
	const int sh = 24 - 8 * (int)(a & 3);
	return (uint8_t)(rd32(a & ~3u, 0xffu << sh) >> sh);
}
static uint16_t r16(void *u, uint32_t a)
{
	(void)u;
	RND_R(a, uint16_t, 2, 16 - 8 * (a & 2));
	const int sh = 16 - 8 * (int)(a & 2);
	return (uint16_t)(rd32(a & ~3u, 0xffffu << sh) >> sh);
}
static uint32_t r32(void *u, uint32_t a)
{
	(void)u;
	RND_R(a, uint32_t, 4, 0);
	return rd32(a, 0xffffffffu);
}
static void wr32_rec(uint32_t a, uint32_t data, uint32_t mask, int size)
{
	if (RND_REC) {
		uint32_t *p = rnd_touch(a, size, 1);
		if (p) {
			*p = (*p & ~mask) | (data & mask);
			return;
		}
	}
	wr32(a, data, mask);
}
static void w8(void *u, uint32_t a, uint8_t d)
{
	(void)u;
	const int sh = 24 - 8 * (int)(a & 3);
	wr32_rec(a & ~3u, (uint32_t)d << sh, 0xffu << sh, 1);
}
static void w16(void *u, uint32_t a, uint16_t d)
{
	(void)u;
	const int sh = 16 - 8 * (int)(a & 2);
	wr32_rec(a & ~3u, (uint32_t)d << sh, 0xffffu << sh, 2);
}
static void w32(void *u, uint32_t a, uint32_t d)
{
	(void)u;
	wr32_rec(a, d, 0xffffffffu, 4);
}

// MAME dma_callback (the SH-2 DMA bypasses the encryption)
static uint32_t dma_kludge(void *u, uint32_t src, uint32_t dst, uint32_t data, int size)
{
	(void)u;
	(void)dst;
	if (size == 0)
		data <<= 24 - 8 * (src & 3);
	if (src < 0x80000)
		data ^= MASK(src & ~3u);
	else if (src >= 0x6000000 && src < 0x7000000) {
		if (!s_alt)
			data ^= MASK(src & ~3u);
	}
	if (size == 0)
		data = (data >> (24 - 8 * (src & 3))) & 0xff;
	return data;
}

static void map_mem(uint8_t *bios)
{
	sh2_map(&s_cpu, 0x00000000, 0x0007ffff, bios, 0x80000, 1, 0);
	sh2_map(&s_cpu, 0x02000000, 0x0207ffff, b.mainram, sizeof(b.mainram), 1, 1);
	sh2_map(&s_cpu, 0x04000000, 0x0407ffff, b.spriteram, sizeof(b.spriteram), 1, 1);
	for (int w = 0; w < s_nsimm_prg; w++)
		prg_map(w);
	s_cpu.op_hi = (const uint8_t *)b.cache_dec;
	s_cpu.op_hi_base = 0xc0000000;
	s_cpu.op_hi_size = 0x400;
	s_cpu.op_lo = (const uint8_t *)s_dec;
	s_cpu.op_lo_base = 0x06000000;
	s_cpu.op_lo_size = (uint32_t)s_nsimm_prg * 0x800000;
}

// ---- video ----
enum { T_PEN_INDEX, T_PEN_INDEX_BLEND };

static struct snap {
	uint32_t spritelist[0x80000 / 4 + 0x800];
	uint32_t sprram[0x80000 / 4 + 0x1400];   // (the tilemaps' words; + room for MAME's reads past the end)
	uint32_t pens[0x20000];                  // 0x00RRGGBB
	uint8_t ss_ram[0x8000];
	uint32_t gscroll_buff[8], tmap_regs[16], crtc_zoom[8];
	int16_t ss_vscroll;
	uint8_t ss_pal_base;
} *s_snap;
static uint32_t *s_rb;            // the render buffer (RB_W x RB_H palette indices)
static int s_rb_l, s_rb_r, s_rb_t, s_rb_b;   // its clip this frame
static uint32_t s_fszx, s_fszy;
static int s_zoomed, s_565, s_warned;

typedef struct { int l, r, t, bt; } clip_t;

// MAME cps3_drawgfxzoom for the 16x16 8 bpp characters (gfx 1) into the render buffer, PEN_INDEX(_BLEND), pen 0 clear
static void drawgfxzoom16(const clip_t *clip, uint32_t code, uint32_t color, int gran, int flipx, int flipy, int sx, int sy,
                          int transparency, int scalex, int scaley)
{
	if (!scalex || !scaley)
		return;
	clip_t c = *clip;
	if (c.l < 0) c.l = 0;
	if (c.t < 0) c.t = 0;
	if (c.r > RB_W - 1) c.r = RB_W - 1;
	if (c.bt > RB_H - 1) c.bt = RB_H - 1;
	const uint32_t palbase = ((uint32_t)gran * color) & 0x1ffff;
	const uint8_t *source_base = (const uint8_t *)s_char_ram + (size_t)(code & 0x7fff) * 256;
	const int sprite_screen_height = (scaley * 16 + 0x8000) >> 16;
	const int sprite_screen_width = (scalex * 16 + 0x8000) >> 16;
	if (!sprite_screen_width || !sprite_screen_height)
		return;
	int dx = (16 << 16) / sprite_screen_width;
	int dy = (16 << 16) / sprite_screen_height;
	int ex = sx + sprite_screen_width, ey = sy + sprite_screen_height;
	int x_index_base, y_index;
	if (flipx) {
		x_index_base = (sprite_screen_width - 1) * dx;
		dx = -dx;
	} else
		x_index_base = 0;
	if (flipy) {
		y_index = (sprite_screen_height - 1) * dy;
		dy = -dy;
	} else
		y_index = 0;
	if (sx < c.l) {
		const int pixels = c.l - sx;
		sx += pixels;
		x_index_base += pixels * dx;
	}
	if (sy < c.t) {
		const int pixels = c.t - sy;
		sy += pixels;
		y_index += pixels * dy;
	}
	if (ex > c.r + 1)
		ex = c.r + 1;
	if (ey > c.bt + 1)
		ey = c.bt + 1;
	if (ex <= sx)
		return;
	for (int y = sy; y < ey; y++) {
		const uint8_t *source = source_base + (y_index >> 16) * 16;
		uint32_t *dest = s_rb + (size_t)y * RB_W;
		int x_index = x_index_base;
		if (transparency == T_PEN_INDEX) {
			for (int x = sx; x < ex; x++) {
				const int px = source[(x_index >> 16) ^ 3];
				if (px)
					dest[x] = (uint32_t)px | palbase;
				x_index += dx;
			}
		} else if (gran == 64) {
			for (int x = sx; x < ex; x++) {
				const int px = source[(x_index >> 16) ^ 3];
				if (px)
					dest[x] |= (uint32_t)(px & 0xf) << 13;
				x_index += dx;
			}
		} else {
			for (int x = sx; x < ex; x++) {
				const int px = source[(x_index >> 16) ^ 3];
				if (px)
					dest[x] |= (uint32_t)((px & 1) << 15) | ((color & 1) << 16);
				x_index += dx;
			}
		}
		y_index += dy;
	}
}

static void draw_tilemapsprite_line(const uint32_t *regs, int drawline, const clip_t *cliprect)
{
	if (!(regs[1] & 0x00008000))
		return;
	int scrollx = (int)((regs[0] & 0xffff0000) >> 16);
	int scrolly = (int)(regs[0] & 0x0000ffff);
	const int linescroll_enable = (regs[1] & 0x00004000) != 0;
	const int global_flip_x = (regs[1] & 0x00000800) != 0;
	const int global_flip_y = (regs[1] & 0x00000400) != 0;
	const uint32_t linebase = ((regs[2] & 0x7f000000) >> 24) << 10;
	const uint32_t mapbase = ((regs[2] & 0x007f0000) >> 16) << 10;
	scrolly += 4;
	int line = (drawline + scrolly) & 0x3ff;
	if (global_flip_y)
		line ^= 0x3ff;
	const int xflip_mask = global_flip_x ? 0x3f : 0;
	const int tileline = (line / 16) + 1;
	const int tilesubline = line % 16;
	if (linescroll_enable)
		scrollx += (int)((s_snap->sprram[linebase + ((line + 16) & 0x3ff)] >> 16) & 0x3ff);
	const clip_t clip = { cliprect->l, cliprect->r, drawline, drawline };
	for (int x = cliprect->l / 16; x < (cliprect->r / 16) + 2; x++) {
		const uint32_t dat = s_snap->sprram[mapbase + ((tileline & 63) * 64) + (((x + scrollx / 16) & 63) ^ xflip_mask)];
		const uint32_t tileno = (dat & 0xfffe0000) >> 17;
		int xflip = (dat & 0x00001000) >> 12;
		const int yflip = (dat & 0x00000800) >> 11;
		const int alpha = (dat & 0x00000400) >> 10;
		const int bpp = (dat & 0x00000200) >> 9;
		const uint32_t colour = dat & 0x000001ff;
		xflip ^= xflip_mask & 1;
		drawgfxzoom16(&clip, tileno, colour, bpp ? 64 : 256, xflip, yflip, (x * 16) - scrollx % 16, drawline - tilesubline,
		              alpha ? T_PEN_INDEX_BLEND : T_PEN_INDEX, 0x10000, 0x10000);
	}
}

// the sprites of screen_update into render buffer rows [t, bt] (fill, then the list in order)
static void draw_sprites(int t, int bt)
{
	for (int y = t; y <= bt; y++)
		memset(s_rb + (size_t)y * RB_W + s_rb_l, 0, sizeof(uint32_t) * (size_t)(s_rb_r - s_rb_l + 1));
	const clip_t clip = { s_rb_l, s_rb_r, t, bt };
	const uint32_t *sl = s_snap->spritelist;
	for (int i = 0; i < 0x2000 / 4; i += 4) {
		if (sl[i + 0] & 0x80000000)
			break;
		const int gscroll = (int)((sl[i + 0] & 0x70000000) >> 28);
		const uint32_t length = (sl[i + 0] & 0x01ff0000) >> 16;
		uint32_t start = (sl[i + 0] & 0x00007ff0) >> 4;
		const int xpos = (int)((sl[i + 1] & 0x03ff0000) >> 16);
		const int ypos = (int)(sl[i + 1] & 0x000003ff);
		const int whichbpp = (sl[i + 2] & 0x40000000) != 0;
		const int whichpal = (sl[i + 2] & 0x20000000) != 0;
		const int global_xflip = (sl[i + 2] & 0x10000000) != 0;
		const int global_yflip = (sl[i + 2] & 0x08000000) != 0;
		const int global_alpha = (sl[i + 2] & 0x04000000) != 0;
		const int global_bpp = (sl[i + 2] & 0x02000000) != 0;
		const uint32_t global_pal = (sl[i + 2] & 0x01ff0000) >> 16;
		const int gscrollx = (int)((s_snap->gscroll_buff[gscroll] & 0x03ff0000) >> 16);
		const int gscrolly = (int)(s_snap->gscroll_buff[gscroll] & 0x000003ff);
		start = (start * 0x100) >> 2;
		for (uint32_t j = 0; j < length * 4; j += 4) {
			const uint32_t value1 = sl[start + j + 0];
			const uint32_t value2 = sl[start + j + 1];
			const uint32_t value3 = sl[start + j + 2];
			static const int tilestable[4] = { 8, 1, 2, 4 };
			const uint32_t tileno = (value1 & 0xfffe0000) >> 17;
			int flipx = (value1 & 0x00001000) >> 12;
			int flipy = (value1 & 0x00000800) >> 11;
			const int alpha = (value1 & 0x00000400) >> 10;
			const int bpp = (value1 & 0x00000200) >> 9;
			const uint32_t pal = value1 & 0x000001ff;
			int xpos2 = (int)((value2 & 0x03ff0000) >> 16);
			int ypos2 = (int)(value2 & 0x000003ff);
			const uint32_t ysizedraw2 = ((value3 & 0x7f000000) >> 24) + 1;
			const uint32_t xsizedraw2 = ((value3 & 0x007f0000) >> 16) + 1;
			int ysize2 = (int)((value3 & 0x0000000c) >> 2);
			int xsize2 = (int)(value3 & 0x00000003);
			if (ysize2 == 0)
				continue;
			if (xsize2 == 0) {   // a tilemap
				const int tilemapnum = (int)((value3 & 0x00000030) >> 4);
				const uint32_t *regs = &s_snap->tmap_regs[tilemapnum * 4];
				for (uint32_t yy = 0; yy < ysizedraw2; yy++) {
					int cury_pos = ypos2 + gscrolly - (int)yy;
					cury_pos = ~cury_pos;
					cury_pos -= 18;
					cury_pos &= 0x3ff;
					if (cury_pos >= t && cury_pos <= bt)
						draw_tilemapsprite_line(regs, cury_pos, &clip);
				}
				continue;
			}
			ysize2 = tilestable[ysize2];
			xsize2 = tilestable[xsize2];
			const uint32_t xinc = (xsizedraw2 << 16) / (uint32_t)xsize2;
			const uint32_t yinc = (ysizedraw2 << 16) / (uint32_t)ysize2;
			uint32_t xscale = xinc / 16, yscale = yinc / 16;
			if (xscale & 0xffff) xscale += (1 << 16) / 16;
			if (yscale & 0xffff) yscale += (1 << 16) / 16;
			xsize2 -= 1;
			ysize2 -= 1;
			flipx ^= global_xflip;
			flipy ^= global_yflip;
			if (!flipx) xpos2 += (int)(xsizedraw2 / 2);
			else xpos2 -= (int)(xsizedraw2 / 2);
			ypos2 += (int)(ysizedraw2 / 2);
			if (!flipx) xpos2 -= (int)(((uint32_t)(xsize2 + 1) * xinc) >> 16);
			else xpos2 += (int)(((uint32_t)xsize2 * xinc) >> 16);
			if (flipy) ypos2 -= (int)(((uint32_t)ysize2 * yinc) >> 16);
			const uint32_t actualpal = whichpal ? global_pal : pal;
			const int gran = (whichbpp ? global_bpp : bpp) ? 64 : 256;
			const int trans = (global_alpha || alpha) ? T_PEN_INDEX_BLEND : T_PEN_INDEX;
			int count = 0;
			for (int xx = 0; xx < xsize2 + 1; xx++) {
				int current_xpos;
				if (!flipx) current_xpos = (xpos + xpos2 + (int)(((uint32_t)xx * xinc) >> 16));
				else current_xpos = (xpos + xpos2 - (int)(((uint32_t)xx * xinc) >> 16));
				current_xpos += gscrollx;
				current_xpos += 1;
				current_xpos &= 0x3ff;
				if (current_xpos & 0x200) current_xpos -= 0x400;
				for (int yy = 0; yy < ysize2 + 1; yy++) {
					int current_ypos;
					if (flipy) current_ypos = (ypos + ypos2 + (int)(((uint32_t)yy * yinc) >> 16));
					else current_ypos = (ypos + ypos2 - (int)(((uint32_t)yy * yinc) >> 16));
					current_ypos += gscrolly;
					current_ypos = 0x3ff - current_ypos;
					current_ypos -= 17;
					current_ypos &= 0x3ff;
					if (current_ypos & 0x200) current_ypos -= 0x400;
					drawgfxzoom16(&clip, tileno + (uint32_t)count, actualpal, gran, flipx, flipy, current_xpos, current_ypos,
					              trans, (int)xscale, (int)yscale);
					count++;
				}
			}
		}
	}
}

// the text layer of a line (MAME draw_fg_layer, 8x8 4 bpp tiles of the SS RAM, pen 0 clear)
static void draw_fg_line(int line, uint32_t *d)
{
	const uint8_t *ss = s_snap->ss_ram;
	const int scrolly = (-s_snap->ss_vscroll) & 0x100;
	const int y = line / 8;
	int offset = ((line + scrolly) / 8 * 128) & 0x1fff;
	const int rowscroll = ss[((line + scrolly - 1) & 0x1ff) * 2 + 0x2000];
	const int sy = y * 8;
	for (int x = 0; x < 64; x++) {
		const uint16_t data = (uint16_t)(ss[offset] | (ss[offset + 1] << 8));
		const uint32_t tile = data & 0x01ff;
		int pal = (data & 0x3e00) >> 9;
		const int flipy = (data & 0x4000) >> 14;
		const int flipx = (data & 0x8000) >> 15;
		pal += s_snap->ss_pal_base << 5;
		const uint32_t *pens = s_snap->pens + (((uint32_t)pal * 16) & 0x1ffff);
		const int row = flipy ? 7 - (line - sy) : line - sy;
		const uint8_t *src = ss + 0x4000 + tile * 32 + row * 4;
		if (src[0] | src[1] | src[2] | src[3]) {
			for (int k = 0; k < 2; k++) {
				const int sx = k * 512 + (x * 8) - rowscroll;
				for (int cx = 0; cx < 8; cx++) {
					const int dx = sx + cx;
					if (dx < 0 || dx >= VIS_W)
						continue;
					const int px = flipx ? 7 - cx : cx;
					const int c = (src[px >> 1] >> ((px & 1) * 4)) & 0xf;
					if (c)
						d[dx] = pens[c];
				}
			}
		}
		offset += 2;
	}
}

#define BANDS 8
static void band_sprites(int band)
{
	const int n = s_rb_b - s_rb_t + 1;
	const int t = s_rb_t + n * band / BANDS, bt = s_rb_t + n * (band + 1) / BANDS - 1;
	if (bt >= t)
		draw_sprites(t, bt);
}

static void band_out(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS;
	const uint32_t *pens = s_snap->pens;
	for (int y = r0; y < r1; y++) {
		uint32_t d[VIS_W];
		if (!s_zoomed) {
			const uint32_t *src = s_rb + (size_t)y * RB_W;
			for (int x = 0; x < VIS_W; x++)
				d[x] = pens[src[x] & 0x1ffff];
		} else {
			const uint32_t *src = s_rb + (size_t)((uint32_t)y * s_fszy >> 16) * RB_W;
			uint32_t srcx = 0;
			for (int x = 0; x < VIS_W; x++) {
				d[x] = pens[src[srcx >> 16] & 0x1ffff];
				srcx += s_fszx;
			}
		}
		draw_fg_line(y, d);
		uint8_t *o = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
		if (s_565)
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)o)[x] = (uint16_t)((d[x] >> 8 & 0xf800) | (d[x] >> 5 & 0x07e0) | (d[x] >> 3 & 0x001f));
		else if (rbx_video.out_bpp == 4)
			for (int x = 0; x < VIS_W; x++)
				((uint32_t *)o)[x] = rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
		else
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)o)[x] = (uint16_t)rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
	}
}

static decoband_t s_bands_spr = { 0, 0, 0, 0, BANDS, band_sprites };
static decoband_t s_bands_out = { 0, 0, 0, 0, BANDS, band_out };
static int cps3_help(int helper) { return decoband_help(&s_bands_spr, helper) || decoband_help(&s_bands_out, helper); }

static void cps3_render(void)
{
	if (!rbx_video.out) {
		__atomic_store_n(&s_cram_drawing, 0, __ATOMIC_SEQ_CST);
		return;
	}
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f && rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	const uint32_t *crtc = s_snap->crtc_zoom;
	const int width = (int)((crtc[1] & 0xffff0000) >> 16) - (int)(crtc[0] & 0xffff);
	if (width > 0 && width != VIS_W && !s_warned) {
		s_warned = 1;
		rbx_log("cps3: screen width %d not emulated (drawn at %d)\n", width, VIS_W);
	}
	uint32_t zx = crtc[3] & 0xff, zy = crtc[7] & 0xff;
	if (zx > 0x80) zx = 0x80;
	if (zy > 0x80) zy = 0x80;
	s_fszx = (zx << 16) / 0x40;
	s_fszy = (zy << 16) / 0x40;
	s_zoomed = !(zx == 0x40 && zy == 0x40);
	if (!s_zoomed) {
		s_rb_l = 0, s_rb_r = VIS_W - 1, s_rb_t = 0, s_rb_b = VIS_H - 1;
	} else {
		s_rb_l = 0;
		s_rb_r = (int)((((uint32_t)VIS_W * s_fszx + 0x8000) >> 16)) - 1;
		s_rb_t = 0;
		s_rb_b = (int)((((uint32_t)VIS_H * s_fszy + 0x8000) >> 16)) - 1;
	}
	decoband_run(&s_bands_spr);
	__atomic_store_n(&s_cram_drawing, 0, __ATOMIC_SEQ_CST);   // (the character RAM is not read any more this frame)
	decoband_run(&s_bands_out);
	rbx_video.out_done = 1;
}

static void snap_range(uint32_t from, uint32_t n)
{
	if (from >= 0x20000)
		return;
	if (from + n > 0x20000)
		n = 0x20000 - from;
	memcpy(&s_snap->sprram[from], &b.spriteram[from], n * sizeof(uint32_t));
}

static void vblank_capture(void)
{
	cdq_sync();
	rbx_video_wait_idle();
	cram_merge(0);
	// the sprite list as screen_update reads it: the main list and the sublists it points at
	const uint32_t *sl = b.spritelist;
	for (int i = 0; i < 0x2000 / 4; i += 4) {
		memcpy(&s_snap->spritelist[i], &sl[i], 4 * sizeof(uint32_t));
		if (sl[i] & 0x80000000)
			break;
		const uint32_t start = ((sl[i] & 0x00007ff0) >> 4) * 0x100 >> 2;
		const uint32_t length = (sl[i] & 0x01ff0000) >> 16;
		memcpy(&s_snap->spritelist[start], &sl[start], length * 4 * sizeof(uint32_t));
	}
	memcpy(s_snap->tmap_regs, b.tmap_regs, sizeof(b.tmap_regs));
	for (int n = 0; n < 4; n++) {
		const uint32_t *regs = &b.tmap_regs[n * 4];
		if (!(regs[1] & 0x8000))
			continue;
		snap_range(((regs[2] & 0x007f0000) >> 16) << 10, 64 * 64);
		snap_range(((regs[2] & 0x7f000000) >> 24) << 10, 0x400);
	}
	for (int k = 0; k < 0x20000 / 256; k++) {
		if (!s_pal_dirty[k])
			continue;
		s_pal_dirty[k] = 0;
		for (int i = k * 256; i < k * 256 + 256; i++) {
			const uint16_t c = b.colourram[i];
			s_snap->pens[i] = (uint32_t)(c & 0x1f) << 19 | (uint32_t)((c >> 5) & 0x1f) << 11 | (uint32_t)((c >> 10) & 0x1f) << 3;
		}
	}
	memcpy(s_snap->ss_ram, b.ss_ram, sizeof(b.ss_ram));
	memcpy(s_snap->gscroll_buff, b.gscroll_buff, sizeof(b.gscroll_buff));
	memcpy(s_snap->crtc_zoom, b.crtc_zoom, sizeof(b.crtc_zoom));
	s_snap->ss_vscroll = b.ss_vscroll;
	s_snap->ss_pal_base = b.ss_pal_base;
	__atomic_store_n(&s_cram_drawing, 1, __ATOMIC_SEQ_CST);
	rbx_video_frame_done();
}

// ---- frame ----
static void exec_to(int64_t t)
{
	s_cpu.target = t;
	sh2_execute(&s_cpu, 0);
}

static void run_until(int64_t t)
{
	for (;;) {
		int64_t now = sh2_total_cycles(&s_cpu);
		if (b.dma_timer <= now) {
			b.dma_timer = NEVER;
			b.dma_status &= (uint16_t)~6;
			sh2_set_irq_line(&s_cpu, 10, SH2_ASSERT);
		}
		if (b.spr_timer <= now) {
			b.spr_timer = NEVER;
			b.dma_status &= (uint16_t)~1;
		}
		if (now >= t)
			break;
		int64_t stop = t;
		if (b.dma_timer < stop) stop = b.dma_timer;
		if (b.spr_timer < stop) stop = b.spr_timer;
		exec_to(stop);
		rnd_learn(0);
	}
}


static void cps3_frame(void)
{
	uint64_t pf_t0 = 0;
	int64_t pf_c0 = 0, pf_s0 = 0, pf_r0 = 0;
	if (s_prof.on) {
		pf_t0 = rbx_clock_us();
		pf_c0 = sh2_total_cycles(&s_cpu);
		pf_s0 = s_rnd->skipped;
		pf_r0 = s_rnd->recorded;
		s_prof.dma_us = s_prof.out_bytes = s_prof.dma_bytes = s_prof.dma_n = s_prof.hit_n = s_prof.dec_us = s_prof.wait_us = 0;
	}
	rbx_audio.frame_mhz = 59600;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H) {
			vblank_capture();
			sh2_set_irq_line(&s_cpu, 12, SH2_ASSERT);
		}
		b.line_frac += LINE_NUM;
		const uint32_t cyc = rbx_div64_small(b.line_frac, XTAL);
		b.line_frac -= (uint64_t)cyc * XTAL;
		b.line_end += cyc;
		if ((line & 63) == 63 || line == VIS_H - 1 || line == LINES - 1) {   // (the only interrupt comes at VIS_H)
			run_until(b.line_end);
			rnd_learn(1);
		}
	}
	snd_update();
	for (int i = 0; i < s_snd_n; i++)
		rbx_audio_put_stereo(s_snd[2 * i] >> 8, s_snd[2 * i + 1] >> 8);
	s_snd_n = 0;
	if (s_prof.on) {
		static int pf_f;
		const uint64_t us = rbx_clock_us() - pf_t0;
		pf_f++;
		if (us >= s_prof.thr)
			fprintf(stderr, "cps3prof f%d %llu us: dma %llu us %llu B %llu (hit %llu) decode %llu us out %llu B, cpu run %lld of %lld rec %lld, waited for the DMA %llu us\n", pf_f,
			        (unsigned long long)us, (unsigned long long)s_prof.dma_us, (unsigned long long)s_prof.dma_bytes,
			        (unsigned long long)s_prof.dma_n, (unsigned long long)s_prof.hit_n, (unsigned long long)s_prof.dec_us, (unsigned long long)s_prof.out_bytes,
			        (long long)((sh2_total_cycles(&s_cpu) - pf_c0) - (s_rnd->skipped - pf_s0)),
			        (long long)(sh2_total_cycles(&s_cpu) - pf_c0), (long long)(s_rnd->recorded - pf_r0), (unsigned long long)s_prof.wait_us);
	}
}

static void cps3_reset(void)
{
	cdq_sync();
	rbx_video_wait_idle();
	cram_merge(0);
	// (MAME machine_reset: the DMA state and the program images; the RAMs keep their contents over a reset as in
	// MAME, but a new game starts from zero: cps3_init clears the state)
	b.current_table_address = (uint32_t)-1;
	b.dma_status = 0;
	b.spritelist_dma = 0;
	b.dma_timer = b.spr_timer = NEVER;
	copy_from_nvram();
	for (int w = 0; w < s_nsimm_prg; w++)
		prg_map(w);
	sh2_reset(&s_cpu);
	rnd_reset();
	b.line_end = sh2_total_cycles(&s_cpu);
	b.snd_last = b.line_end;
	b.snd_frac = 0;
	s_snd_n = 0;
}

static int cps3_supported(const char *machine)
{
	return !strncmp(machine, "redearth:", 9) || !strncmp(machine, "sfiii:", 6) || !strncmp(machine, "sfiii2:", 7) ||
	       !strncmp(machine, "jojo:", 5) || !strncmp(machine, "sfiii3:", 7) || !strncmp(machine, "jojoba:", 7);
}

static void nv_save(void)
{
	if (!s_nv_path[0] || !s_bp)
		return;
	FILE *f = fopen(s_nv_path, "wb");
	if (!f)
		return;
	fwrite(b.eeprom, 1, sizeof(b.eeprom), f);
	fclose(f);
}

static void nv_load(const char *set)
{
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	s_nv_path[0] = 0;
	if (!dir || !set)
		return;
	snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, set);
	FILE *f = fopen(s_nv_path, "rb");
	if (!f)
		return;
	uint32_t e[0x80 / 4];
	if (fread(e, 1, sizeof(e), f) == sizeof(e))
		memcpy(b.eeprom, e, sizeof(e));
	fclose(f);
}

static void cps3_exit(void)
{
	if (s_cdq_on)
		rbx_aux_set_job(NULL);   // (syncs first)
	s_cdq_on = 0;
	rbx_video_wait_idle();
	nv_save();
	s_nv_path[0] = 0;
	if (s_rnd && getenv("RBX_CPS3_STAT"))
		rbx_log("cps3: idle rounds skipped %lld cycles of %lld, %d recordings %lld cycles\n", (long long)s_rnd->skipped,
		        (long long)sh2_total_cycles(&s_cpu), s_rnd->nrec, (long long)s_rnd->recorded);
	free(s_snap);
	s_snap = NULL;
	free(s_rb);
	s_rb = NULL;
	free(s_snd);
	s_snd = NULL;
	free(s_dec);
	s_dec = NULL;
	free(s_raw);
	s_raw = NULL;
	free(s_char_ram);
	s_char_ram = NULL;
	free(s_cow_pool);
	s_cow_pool = NULL;
	cdc_flush();
	s_cow_n = 0;
	free(s_bp);
	s_bp = NULL;
	free(s_rnd);
	s_rnd = NULL;
}

static int cps3_init(const rbx_game *g)
{
	if (!cps3_supported(g->machine)) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	if (!rbx_region_opt("simm1.0", NULL)) {
		rbx_fail("%s: the CD sets are not emulated (use a NO CD set)", g->set);
		return -1;
	}
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	s_alt = 0;
	if (!strcmp(init, "redearth")) s_key1 = 0x9e300ab1, s_key2 = 0xa175b82c;
	else if (!strcmp(init, "sfiii")) s_key1 = 0xb5fe053e, s_key2 = 0xfc03925a;
	else if (!strcmp(init, "sfiii2")) s_key1 = 0, s_key2 = 0, s_alt = 1;
	else if (!strcmp(init, "jojo")) s_key1 = 0x02203ee3, s_key2 = 0x01301972;
	else if (!strcmp(init, "sfiii3")) s_key1 = 0xa55432b4, s_key2 = 0x0c129981;
	else if (!strcmp(init, "jojoba")) s_key1 = 0x23323ee3, s_key2 = 0x03021972;
	else {
		rbx_fail("%s: %s not emulated", g->set, init);
		return -1;
	}
	uint32_t bios_size = 0;
	uint8_t *bios = rbx_region("bios", &bios_size);
	if (!bios || bios_size < 0x80000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(s_chip, 0, sizeof(s_chip));
	for (int s = 0; s < 7; s++)
		for (int i = 0; i < 8; i++) {
			char name[16];
			snprintf(name, sizeof(name), "simm%d.%d", s + 1, i);
			uint32_t sz = 0;
			uint8_t *p = rbx_region_opt(name, &sz);
			s_chip[s][i] = (p && sz >= 0x200000) ? p : NULL;
		}
	s_nsimm_prg = 0;
	for (int w = 0; w < 2; w++) {
		int ok = 1;
		for (int i = 0; i < 4; i++)
			ok &= s_chip[w][i] != NULL;
		if (!ok)
			break;
		s_nsimm_prg = w + 1;
	}
	if (!s_nsimm_prg) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_ngpair = 0;   // (MAME copy_from_nvram: the pairs in order, up to the first missing)
	for (int fn = 0; fn < 40 && s_ngpair < 16; fn += 2) {
		const int s = 2 + fn / 8, i = fn % 8;
		if (s >= 7 || !s_chip[s][i] || !s_chip[s][i + 1])
			break;
		s_gpair[0][s_ngpair] = s_chip[s][i];
		s_gpair[1][s_ngpair] = s_chip[s][i + 1];
		s_ngpair++;
	}
	if (!(s_bp = calloc(1, sizeof(*s_bp))) || !(s_char_ram = calloc(0x800000, 1)) || !(s_snap = calloc(1, sizeof(*s_snap))) ||
	    !(s_rb = calloc((size_t)RB_W * RB_H, sizeof(uint32_t))) || !(s_snd = calloc(2 * SND_CAP, sizeof(int32_t))) || !(s_rnd = calloc(1, sizeof(*s_rnd))) || !(s_cow_pool = malloc((size_t)COW_PAGES * 4096)) ||
	    !(s_dec = calloc((size_t)s_nsimm_prg * 0x800000, 1)) || (s_alt && !(s_raw = calloc((size_t)s_nsimm_prg * 0x800000, 1)))) {
		rbx_fail("%s: out of memory", g->set);
		cps3_exit();
		return -1;
	}
	for (uint32_t p = 0; p < 0x800000 / 4096; p++)
		s_cram_ev[p] = cram_live(p);
	s_cow_n = 0;
	s_cram_drawing = 0;
	s_cowtest = getenv("RBX_CPS3_COWTEST") != NULL;   // (a test: every character RAM write of a frame through the copies)
	s_prof.on = getenv("RBX_CPS3_PROF") != NULL;
	s_dma_new = getenv("RBX_CPS3_NEWDMA") != NULL;
	s_cdq_w = s_cdq_r = 0;
	// (off by default since 11-20: on the device the worker DMA added 1.1-1.4 s stalls in jojoban/redearthn play that
	// the inline path has not; RBX_CPS3_DMA_WORKER turns it on for tests)
	s_cdq_on = getenv("RBX_CPS3_DMA_WORKER") != NULL && getenv("RBX_CPS3_DMA_INLINE") == NULL;
	if (s_cdq_on) {
		rbx_aux_set_by_render(1);
		rbx_aux_set_job(cdq_job);
	}
	s_prof.thr = s_prof.on ? (uint32_t)atoi(getenv("RBX_CPS3_PROF")) : 0;
	if (s_prof.on)
		memset(s_char_ram, 0, 0x800000);   // (the host's pages touched now)
	// MAME decrypt_bios (the region in host-order words, as MAME's 32-bit big-endian region on a little-endian host)
	sh2_swap32(bios, 0x80000);
	for (uint32_t i = 0; i < 0x80000; i += 4)
		((uint32_t *)bios)[i / 4] ^= MASK(i);
	for (int s = 0; s < 7; s++)
		for (int i = 0; i < 8; i++)
			b.flash[s][i].timer = NEVER;
	memset(s_pal_dirty, 1, sizeof(s_pal_dirty));
	s_warned = 0;
	s_snd_n = 0;
	s_snap->spritelist[0] = 0x80000000;
	b.spritelist[0] = 0x80000000;
	s_port_in = rbx_port_find("INPUTS");
	s_port_extra = rbx_port_find("EXTRA");
	nv_load(g->set);
	sh2_init(&s_cpu);
	s_cpu.read8 = r8;
	s_cpu.read16 = r16;
	s_cpu.read32 = r32;
	s_cpu.write8 = w8;
	s_cpu.write16 = w16;
	s_cpu.write32 = w32;
	s_cpu.dma_kludge = dma_kludge;
	map_mem(bios);
	if (rbx_audio_setup_stereo(37287) || rbx_video_setup(VIS_W, VIS_H, 1)) {
		rbx_fail("%s: out of memory", g->set);
		cps3_exit();
		return -1;
	}
	cps3_reset();
	return 0;
}

static int cps3_state(void *ctx, int load)
{
	cdq_sync();
	rbx_video_wait_idle();
	cram_merge(load);
	if (load)
		cdc_flush();
	int rc = 0;
	rc |= rbx_state_io(ctx, "sh2", &s_cpu, SH2_STATE_SIZE);
	rc |= rbx_state_io(ctx, "cps3", s_bp, (uint32_t)offsetof(struct cps3_state, state_end));
	rc |= rbx_state_io(ctx, "cps3_cram", s_char_ram, 0x800000);
	if (!rc && load) {
		sh2_post_load(&s_cpu);
		copy_from_nvram();
		for (int w = 0; w < s_nsimm_prg; w++)
			prg_map(w);
		memset(s_pal_dirty, 1, sizeof(s_pal_dirty));
		s_snd_n = 0;
		rnd_forget();
	}
	return rc ? -1 : 0;
}

static int cps3_save(void *ctx) { return cps3_state(ctx, 0); }
static int cps3_load(void *ctx) { return cps3_state(ctx, 1); }

const rbx_driver rbx_drv_cps3 = { "Capcom CPS3", cps3_init, cps3_exit, cps3_reset, cps3_frame, cps3_save, cps3_load,
                                  cps3_supported, cps3_render, cps3_help };
