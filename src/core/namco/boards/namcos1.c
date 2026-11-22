/*  Namco System 1: three MC6809E (main, sub, sound) and an HD63701 MCU, the C117 MMU, the C116 palette / clip, the C123
    tilemaps, the CUS39/CUS35 sprites, YM2151 + CUS30 + two 8-bit DACs, and the key custom chips.
    Ported from MAME 0.289 src/mame/namco/namcos1.cpp, namcos1_m.cpp, namcos1_v.cpp, namcos1_sprite.cpp, c117.cpp,
    namco_c116.cpp, namco_c123tmap.cpp (BSD-3-Clause, copyright-holders: Ernesto Corvi, Manuel Abadia, Nicola Salmoria,
    Alex W. Jackson).
    This file is part of ReBOX. GPL-2.0-or-later.

    Not done: the flip screen (cocktail), the 3D glasses output, the watchdog.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6800/m6801.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/namco_wsg.h"

#include <stdlib.h>
#include <string.h>

#define VIS_X0 73
#define VIS_W 288
#define VIS_Y0 16
#define VIS_H 224
#define LINES 264
#define SLICES 2                // (MAME: a 38400 Hz quantum, 2.4 a line; 2 keeps every MAME comparison, ADR-037)
#define FRAME_MHZ 60606         // 6.144 MHz / (384 x 264)
#define CPU_CLOCK 1536000       // 49.152 MHz / 32 (the MCU's E clock: / 8 / 4)
#define YM_CLOCK 3579545
#define PAL_BLACK 0x2000

enum { KEY_NONE, KEY_1, KEY_2, KEY_3 };
enum { IN_STD, IN_BERABOHM, IN_FACEOFF, IN_QUESTER };

// ---- the configuration of the set ----
static uint8_t *s_mainrom, *s_sndrom, *s_mcurom, *s_voice, *s_chr, *s_mask, *s_sprrom;
static uint32_t s_mainrom_size, s_sndrom_size, s_mcurom_size, s_voice_size, s_chr_size, s_mask_size, s_sprrom_size;
static uint8_t *s_sprpix;       // the sprites, a byte a pixel (32x32 a tile)
static uint32_t s_spr_count, s_chr_count;
static int s_key_type, s_key_id, s_key_reg, s_key_rng, s_key_swap4_arg, s_key_swap4, s_key_bottom4, s_key_top4;
static int s_inputs;
static int s_port_ctl[2], s_port_dsw, s_port_coin, s_port_in[4], s_port_paddle[2];

static m6809_t s_main, s_sub, s_snd;
static m6801 s_mcu;
static namco_wsg_t s_wsg;

// ---- the board's state (saved) ----
static struct {
	uint32_t offsets[2][8];     // the C117's banks (23-bit virtual addresses)
	uint8_t subres, reset, held, mcu_patch;
	uint8_t copy_sprites, flip, soundbank, mcubank;
	uint8_t dac[2], dac_gain, strobe;
	uint8_t strobe_count, input_count, stored_input[2];
	uint8_t key[8];
	uint16_t key_quotient, key_reminder, key_hi;
	uint16_t c116_regs[8];
	uint32_t rng;
	uint8_t paddle[2], pad[2];
	int64_t t_main, t_sub, t_snd;
	uint64_t cpu_frac, ym_frac;
	uint8_t control[0x20];
	uint8_t ram[0x8000];        // 0x300000
	uint8_t scratch[0x800];     // 0x2fc000
	uint8_t spr[0x800];         // 0x2fc800
	uint8_t triram[0x800];      // 0x2ff000
	uint8_t vram[0x8000];       // 0x2f0000 (the C123's words, big-endian)
	uint8_t pal_r[0x2000], pal_g[0x2000], pal_b[0x2000];
	uint8_t sndram[0x2000];
	uint8_t nvram[0x800];
} b;

// ---- the drawing's copy (taken at vblank) ----
typedef struct {
	uint8_t control[0x20];
	uint8_t spr[0x800];
	uint8_t vram[0x8000];
	uint8_t pal_r[0x2000], pal_g[0x2000], pal_b[0x2000];
	uint16_t c116_regs[8];
	int flip, pal_dirty;
} snap_t;
static int s_pal_dirty;   // (the palette RAM written since the drawing's copy)
static snap_t *s_snap;
static uint8_t *s_nspri;   // (the priority bitmap, VIS_W x VIS_H)

static int32_t *s_mix_l, *s_mix_r;
static int s_mix_n, s_mix_cap;
static uint32_t s_ym_rate;

// ---- the key custom chips (namcos1_m.cpp) ----
static uint8_t key_r(uint32_t off)
{
	switch (s_key_type) {
	case KEY_1:
		if (off < 3) {
			const int d = b.key[0], n = (b.key[1] << 8) | b.key[2];
			int q = 0xffff, r = 0;
			if (d) {
				q = n / d;
				r = n % d;
			}
			if (off == 0) return (uint8_t)r;
			if (off == 1) return (uint8_t)(q >> 8);
			return (uint8_t)q;
		}
		return off == 3 ? (uint8_t)s_key_id : 0;
	case KEY_2:
		b.key_hi = 0;
		if (off == 0) return (uint8_t)(b.key_reminder >> 8);
		if (off == 1) return (uint8_t)b.key_reminder;
		if (off == 2) return (uint8_t)(b.key_quotient >> 8);
		if (off == 3) return (uint8_t)b.key_quotient;
		return off == 4 ? (uint8_t)s_key_id : 0;
	case KEY_3: {
		const int op = (int)((off & 0x70) >> 4);
		const uint8_t k = s_key_swap4_arg >= 0 ? b.key[s_key_swap4_arg] : 0;
		if (op == s_key_reg) return (uint8_t)s_key_id;
		if (op == s_key_rng) {   // (machine().rand())
			b.rng = b.rng * 1103515245u + 12345u;
			return (uint8_t)(b.rng >> 16);
		}
		if (op == s_key_swap4) return (uint8_t)((k << 4) | (k >> 4));
		if (op == s_key_bottom4) return (uint8_t)((off << 4) | (k & 0x0f));
		if (op == s_key_top4) return (uint8_t)((off << 4) | (k >> 4));
		return 0;
	}
	default:
		return 0;
	}
}

static void key_w(uint32_t off, uint8_t v)
{
	switch (s_key_type) {
	case KEY_1:
		if (off < 4) b.key[off] = v;
		break;
	case KEY_2:
		if (off < 5) {
			b.key[off] = v;
			if (off == 3) {
				const uint32_t d = (uint32_t)(b.key[0] << 8 | b.key[1]);
				const uint32_t n = ((uint32_t)b.key_hi << 16) | (uint32_t)(b.key[2] << 8) | b.key[3];
				if (d) {
					b.key_quotient = (uint16_t)(n / d);
					b.key_reminder = (uint16_t)(n % d);
				} else {
					b.key_quotient = 0xffff;
					b.key_reminder = 0;
				}
				b.key_hi = (uint16_t)(b.key[2] << 8 | b.key[3]);
			}
		}
		break;
	case KEY_3:
		b.key[(off & 0x70) >> 4] = v;
		break;
	default:
		break;
	}
}

// ---- the virtual space of the C117 (virtual_map) ----
static uint8_t c116_r(uint32_t off)
{
	off &= 0x7fff;
	const uint32_t c = ((off & 0x6000) >> 2) | (off & 0x7ff);
	switch (off & 0x1800) {
	case 0x0000: return b.pal_r[c];
	case 0x0800: return b.pal_g[c];
	case 0x1000: return b.pal_b[c];
	default: {
		const int reg = (int)((off & 0xf) >> 1);
		return (off & 1) ? (uint8_t)b.c116_regs[reg] : (uint8_t)(b.c116_regs[reg] >> 8);
	}
	}
}

static void c116_w(uint32_t off, uint8_t v)
{
	off &= 0x7fff;
	const uint32_t c = ((off & 0x6000) >> 2) | (off & 0x7ff);
	switch (off & 0x1800) {
	case 0x0000: b.pal_r[c] = v; s_pal_dirty = 1; break;
	case 0x0800: b.pal_g[c] = v; s_pal_dirty = 1; break;
	case 0x1000: b.pal_b[c] = v; s_pal_dirty = 1; break;
	default: {
		const int reg = (int)((off & 0xf) >> 1);
		if (off & 1)
			b.c116_regs[reg] = (uint16_t)((b.c116_regs[reg] & 0xff00) | v);
		else
			b.c116_regs[reg] = (uint16_t)((b.c116_regs[reg] & 0x00ff) | (v << 8));
		break;
	}
	}
}

static uint8_t virt_r(uint32_t a)
{
	a &= 0x7fffff;
	if (a >= 0x400000) return s_mainrom[a - 0x400000];
	if (a >= 0x300000) return a < 0x308000 ? b.ram[a - 0x300000] : 0;
	if (a >= 0x2ff000) return b.triram[a & 0x7ff];
	if (a >= 0x2fe000) return namco_cus30_amap_r(&s_wsg, (int)(a & 0x3ff));
	if (a >= 0x2fd000) return b.control[a & 0x1f];
	if (a >= 0x2fc800) return b.spr[a - 0x2fc800];
	if (a >= 0x2fc000) return b.scratch[a - 0x2fc000];
	if (a >= 0x2f8000) return a < 0x2fa000 ? key_r(a & 0x1fff) : 0;
	if (a >= 0x2f0000) return b.vram[a - 0x2f0000];
	if (a >= 0x2e0000) return a < 0x2e8000 ? c116_r(a - 0x2e0000) : 0;
	return 0;
}

static void virt_w(uint32_t a, uint8_t v)
{
	a &= 0x7fffff;
	if (a >= 0x400000) return;
	if (a >= 0x300000) {
		if (a < 0x308000) b.ram[a - 0x300000] = v;
	} else if (a >= 0x2ff000)
		b.triram[a & 0x7ff] = v;
	else if (a >= 0x2fe000)
		namco_cus30_amap_w(&s_wsg, (int)(a & 0x3ff), v);
	else if (a >= 0x2fd000)
		b.control[a & 0x1f] = v;   // (control8_w)
	else if (a >= 0x2fc800) {      // spriteram_w
		const uint32_t o = a - 0x2fc800;
		b.spr[o] = v;
		if (o == 0x7f2)
			b.copy_sprites = 1;
		if (o == 0x7f6 && (v & 1) != b.flip)   // (MAME: the bool set from the whole byte)
			b.flip = v != 0;
	} else if (a >= 0x2fc000)
		b.scratch[a - 0x2fc000] = v;
	else if (a >= 0x2f8000) {
		if (a < 0x2fa000) key_w(a & 0x1fff, v);
	} else if (a >= 0x2f0000)
		b.vram[a - 0x2f0000] = v;
	else if (a >= 0x2e0000) {
		if (a < 0x2e8000) c116_w(a - 0x2e0000, v);
	}
	// (0x2c0000: the 3D glasses)
}

// the pages of a bank read / written directly where the virtual space is plain memory
static void bank_pages(int cpu, int bank)
{
	m6809_t *c = cpu ? &s_sub : &s_main;
	const uint32_t base = b.offsets[cpu][bank] & 0x7fe000;
	for (int p = 0; p < 32; p++) {
		const uint32_t a = base + (uint32_t)p * 256;
		const uint8_t *r = NULL;
		uint8_t *w = NULL;
		if (a >= 0x400000)
			r = s_mainrom + (a - 0x400000);
		else if (a >= 0x300000) {
			if (a < 0x308000) r = w = b.ram + (a - 0x300000);
		} else if (a >= 0x2ff000)
			r = w = b.triram + (a & 0x700);
		else if (a >= 0x2fe000) {
		} else if (a >= 0x2fd000) {
		} else if (a >= 0x2fc800) {
			r = b.spr + (a - 0x2fc800);
			if (a < 0x2fcf00) w = (uint8_t *)r;
		} else if (a >= 0x2fc000)
			r = w = b.scratch + (a - 0x2fc000);
		else if (a >= 0x2f8000) {
		} else if (a >= 0x2f0000)
			r = w = b.vram + (a - 0x2f0000);
		const int page = bank * 32 + p;
		c->rpage[page] = r;
		c->wpage[page] = bank == 7 ? NULL : w;   // (0xe000-0xffff written: the C117's registers)
	}
}

static void sub_reset_line(int state);

static void c117_w(int cpu, uint16_t a, uint8_t v)
{
	const int reg = (a >> 9) & 0xf;
	switch (reg) {
	case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: {
		uint32_t *bank = &b.offsets[cpu][reg];
		const uint32_t old = *bank;
		if (!(a & 1))
			*bank = (*bank & 0x1fe000) | ((uint32_t)(v & 3) * 0x200000);
		else
			*bank = (*bank & 0x600000) | ((uint32_t)v * 0x2000);
		if (*bank != old)   // (the same bank written again: its pages as they are)
			bank_pages(cpu, reg);
		break;
	}
	case 8:   // SUBRES
		if (cpu == 0) {
			b.subres = v & 1;
			sub_reset_line(!b.subres);
		}
		break;
	case 11: m6809_set_irq(cpu ? &s_sub : &s_main, 0); break;
	case 12: m6809_set_firq(cpu ? &s_sub : &s_main, 0); break;
	case 13: if (cpu == 0) m6809_set_firq(&s_sub, 1); break;
	case 14:
		if (cpu == 0) {
			b.offsets[1][7] = 0x600000 | ((uint32_t)v * 0x2000);
			bank_pages(1, 7);
		}
		break;
	default: break;
	}
}

static uint8_t main_r(void *ctx, uint16_t a)
{
	const int cpu = ctx != NULL;
	return virt_r(b.offsets[cpu][a >> 13] | (a & 0x1fff));
}

static void main_w(void *ctx, uint16_t a, uint8_t v)
{
	const int cpu = ctx != NULL;
	if (a < 0xe000)
		virt_w(b.offsets[cpu][a >> 13] | (a & 0x1fff), v);
	else
		c117_w(cpu, a, v);
}

// ---- the sound CPU (sound_map) ----
static void snd_bank(void)
{
	const uint32_t base = (uint32_t)b.soundbank * 0x4000;
	for (int p = 0; p < 0x40; p++) {
		const uint32_t o = base + (uint32_t)p * 256;
		s_snd.rpage[p] = o + 256 <= s_sndrom_size ? s_sndrom + o : NULL;
	}
}

static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x4000) return 0;   // (beyond the ROM)
	if (a < 0x5000) return a < 0x4002 ? rbx_ym_read(0, 1) : 0;
	if (a < 0x6000) return namco_cus30_amap_r(&s_wsg, a & 0x3ff);
	return 0;
}

static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x4000 || a == 0x4001)
		rbx_ym_write(0, a & 1, v);
	else if (a >= 0x5000 && a < 0x6000)
		namco_cus30_amap_w(&s_wsg, a & 0x3ff, v);
	else if (a == 0xc000 || a == 0xc001) {
		b.soundbank = (v & 0x70) >> 4;
		snd_bank();
	} else if (a == 0xe000)
		m6809_set_irq(&s_snd, 0);
	// (0xd001: the watchdog)
}

static void ym_irq(int state) { m6809_set_firq(&s_snd, state); }

// ---- the CPUs' wait loops, skipped exactly ----
// A CPU's slice runs alone: no other CPU writes memory and no interrupt line changes until it ends. So a loop that
// only reads plain memory (or the CUS30's RAM), found at its top with its exit condition false and no interrupt
// taken there, runs to the slice's end unchanged: its iterations are added at once (and the counter of the counting
// form), leaving the last ones to run as before. Forms (opcode fetch at the loop's top):
//   96 dd 27 fc     LDA <dd ; BEQ *            (7 clocks)
//   96 dd 26 fc     LDA <dd ; BNE *            (7)
//   0c cc 96 dd 26 fa   INC <cc ; LDA <dd ; BNE  (13)
//   b7 hh ll 96 dd 26 f9   STA $hhll (the C117's watchdog) ; LDA <dd ; BNE  (12)
//   b6 hh ll 27 fb  LDA $hhll ; BEQ *          (8)
static int64_t s_tgt[3];
static uint32_t *s_pcprof;   // (debug: NS1_PCPROF=1 counts the opcode fetches by PC, listed at exit)
static int s_noidle;

static inline uint8_t op_byte(m6809_t *c, uint16_t a)
{
	const uint8_t *p = c->rpage[a >> 8];
	return p ? p[a & 0xff] : c->read(c->ctx, a);
}

// a side-effect free read of plain memory: -1 when the address is not
static int plain_r(m6809_t *c, int cpu, uint16_t a)
{
	const uint8_t *p = c->rpage[a >> 8];
	if (p)
		return p[a & 0xff];
	if (cpu == 2 && a >= 0x5000 && a < 0x6000 && (a & 0x3ff) >= 0x140)
		return namco_cus30_amap_r(&s_wsg, a & 0x3ff);
	return -1;
}

// the length of the loop at pc (0: none, or its exit condition true now); *inc: the counting form's counter
static int idle_loop(m6809_t *c, int cpu, uint16_t pc, uint8_t **inc)
{
	const uint8_t op = op_byte(c, pc);
	int len = 0, v;
	*inc = NULL;
	const uint16_t dpb = (uint16_t)(c->dp << 8);
	if (op == 0x96) {
		const uint8_t b1 = op_byte(c, (uint16_t)(pc + 1)), b2 = op_byte(c, (uint16_t)(pc + 2)), b3 = op_byte(c, (uint16_t)(pc + 3));
		if (b3 != 0xfc || (b2 != 0x27 && b2 != 0x26) || (v = plain_r(c, cpu, (uint16_t)(dpb | b1))) < 0)
			return 0;
		if ((b2 == 0x27) != (v == 0))
			return 0;
		len = 7;
	} else if (op == 0x0c && op_byte(c, (uint16_t)(pc + 2)) == 0x96 && op_byte(c, (uint16_t)(pc + 4)) == 0x9a &&
	           op_byte(c, (uint16_t)(pc + 6)) == 0x26 && op_byte(c, (uint16_t)(pc + 7)) == 0xf8) {   // INC ; LDA ; ORA ; BNE
		const uint8_t cc = op_byte(c, (uint16_t)(pc + 1)), dd = op_byte(c, (uint16_t)(pc + 3)), ee = op_byte(c, (uint16_t)(pc + 5));
		const uint16_t ca = (uint16_t)(dpb | cc);
		int v2;
		if (cc == dd || cc == ee || !c->wpage[ca >> 8] || (v = plain_r(c, cpu, (uint16_t)(dpb | dd))) < 0 ||
		    (v2 = plain_r(c, cpu, (uint16_t)(dpb | ee))) < 0 || !(v | v2))
			return 0;
		*inc = c->wpage[ca >> 8] + (ca & 0xff);
		len = 17;
	} else if (op == 0x0c) {
		if (op_byte(c, (uint16_t)(pc + 2)) != 0x96 || op_byte(c, (uint16_t)(pc + 4)) != 0x26 || op_byte(c, (uint16_t)(pc + 5)) != 0xfa)
			return 0;
		const uint8_t cc = op_byte(c, (uint16_t)(pc + 1)), dd = op_byte(c, (uint16_t)(pc + 3));
		const uint16_t ca = (uint16_t)(dpb | cc);
		if (cc == dd || !c->wpage[ca >> 8] || (v = plain_r(c, cpu, (uint16_t)(dpb | dd))) <= 0)
			return 0;
		*inc = c->wpage[ca >> 8] + (ca & 0xff);
		len = 13;
	} else if (op == 0xb7) {
		const uint16_t sa = (uint16_t)(op_byte(c, (uint16_t)(pc + 1)) << 8 | op_byte(c, (uint16_t)(pc + 2)));
		if (cpu == 2 || sa < 0xe000 || ((sa >> 9) & 0xf) != 9 || op_byte(c, (uint16_t)(pc + 3)) != 0x96 ||
		    op_byte(c, (uint16_t)(pc + 5)) != 0x26 || op_byte(c, (uint16_t)(pc + 6)) != 0xf9)
			return 0;
		if ((v = plain_r(c, cpu, (uint16_t)(dpb | op_byte(c, (uint16_t)(pc + 4))))) <= 0)
			return 0;
		len = 12;
	} else if (op == 0xb6) {
		if (op_byte(c, (uint16_t)(pc + 3)) != 0x27 || op_byte(c, (uint16_t)(pc + 4)) != 0xfb)
			return 0;
		const uint16_t la = (uint16_t)(op_byte(c, (uint16_t)(pc + 1)) << 8 | op_byte(c, (uint16_t)(pc + 2)));
		if ((v = plain_r(c, cpu, la)) != 0)
			return 0;
		len = 8;
	} else
		return 0;
	return len;
}

// the core's hook at a known loop top (before the opcode fetch)
// the forms found at the CPUs' loop tops, kept while the code's pages and DP stay the same (the hook then only reads
// the loop's memory): kind 0 none, 1 v == 0 loops, 2 v != 0 loops, 3 (v | v2) != 0 loops
typedef struct {
	const uint8_t *page, *page2;
	uint16_t pc, a1, a2, ca;
	uint8_t dp, kind, len, valid;
} idle_form_t;
static idle_form_t s_iform[3][2];

static void idle_form(m6809_t *c, int cpu, uint16_t pc, idle_form_t *f)
{
	const uint8_t op = op_byte(c, pc);
	const uint16_t dpb = (uint16_t)(c->dp << 8);
	f->pc = pc;
	f->page = c->rpage[pc >> 8];
	f->page2 = c->rpage[(uint16_t)(pc + 7) >> 8];
	f->dp = c->dp;
	f->kind = 0;
	f->valid = f->page && f->page2;
	f->ca = 0;
	(void)cpu;
	if (op == 0x96) {
		const uint8_t b1 = op_byte(c, (uint16_t)(pc + 1)), b2 = op_byte(c, (uint16_t)(pc + 2)), b3 = op_byte(c, (uint16_t)(pc + 3));
		if (b3 == 0xfc && (b2 == 0x27 || b2 == 0x26)) {
			f->a1 = (uint16_t)(dpb | b1);
			f->kind = b2 == 0x27 ? 1 : 2;
			f->len = 7;
		}
	} else if (op == 0x0c && op_byte(c, (uint16_t)(pc + 2)) == 0x96 && op_byte(c, (uint16_t)(pc + 4)) == 0x9a &&
	           op_byte(c, (uint16_t)(pc + 6)) == 0x26 && op_byte(c, (uint16_t)(pc + 7)) == 0xf8) {
		const uint8_t cc = op_byte(c, (uint16_t)(pc + 1)), dd = op_byte(c, (uint16_t)(pc + 3)), ee = op_byte(c, (uint16_t)(pc + 5));
		if (cc != dd && cc != ee) {
			f->ca = (uint16_t)(dpb | cc);
			f->a1 = (uint16_t)(dpb | dd);
			f->a2 = (uint16_t)(dpb | ee);
			f->kind = 3;
			f->len = 17;
		}
	} else if (op == 0x0c) {
		if (op_byte(c, (uint16_t)(pc + 2)) == 0x96 && op_byte(c, (uint16_t)(pc + 4)) == 0x26 && op_byte(c, (uint16_t)(pc + 5)) == 0xfa) {
			const uint8_t cc = op_byte(c, (uint16_t)(pc + 1)), dd = op_byte(c, (uint16_t)(pc + 3));
			if (cc != dd) {
				f->ca = (uint16_t)(dpb | cc);
				f->a1 = (uint16_t)(dpb | dd);
				f->kind = 2;
				f->len = 13;
			}
		}
	} else if (op == 0xb7) {
		const uint16_t sa = (uint16_t)(op_byte(c, (uint16_t)(pc + 1)) << 8 | op_byte(c, (uint16_t)(pc + 2)));
		if (cpu != 2 && sa >= 0xe000 && ((sa >> 9) & 0xf) == 9 && op_byte(c, (uint16_t)(pc + 3)) == 0x96 &&
		    op_byte(c, (uint16_t)(pc + 5)) == 0x26 && op_byte(c, (uint16_t)(pc + 6)) == 0xf9) {
			f->a1 = (uint16_t)(dpb | op_byte(c, (uint16_t)(pc + 4)));
			f->kind = 2;
			f->len = 12;
		}
	} else if (op == 0xb6) {
		if (op_byte(c, (uint16_t)(pc + 3)) == 0x27 && op_byte(c, (uint16_t)(pc + 4)) == 0xfb) {
			f->a1 = (uint16_t)(op_byte(c, (uint16_t)(pc + 1)) << 8 | op_byte(c, (uint16_t)(pc + 2)));
			f->kind = 1;
			f->len = 8;
		}
	}
}

static void idle_hook(m6809_t *c)
{
	const int cpu = c == &s_main ? 0 : c == &s_sub ? 1 : 2;
	const int64_t left = s_tgt[cpu] - 1 - c->cycles;
	if (left <= 0)
		return;
	idle_form_t *f = &s_iform[cpu][c->pc == c->idle_pc[0] ? 0 : 1];
	if (!f->valid || f->pc != c->pc || f->dp != c->dp || f->page != c->rpage[c->pc >> 8] ||
	    f->page2 != c->rpage[(uint16_t)(c->pc + 7) >> 8])
		idle_form(c, cpu, c->pc, f);
	if (!f->kind)
		return;
	const int v = plain_r(c, cpu, f->a1);
	if (v < 0)
		return;
	if (f->kind == 1 ? v != 0 : f->kind == 2 ? v == 0 : 0)
		return;
	if (f->kind == 3) {
		const int v2 = plain_r(c, cpu, f->a2);
		if (v2 < 0 || !(v | v2))
			return;
	}
	uint8_t *inc = NULL;
	if (f->ca) {
		if (!c->wpage[f->ca >> 8])
			return;
		inc = c->wpage[f->ca >> 8] + (f->ca & 0xff);
	}
	const int len = f->len;
	const int64_t n = left / len;
	if (n <= 0)
		return;
	c->cycles += n * len;
	if (inc)
		*inc = (uint8_t)(*inc + n);
}

// after a slice: a CPU stopped inside a wait loop has its top remembered (two per CPU, the newest first)
static void idle_learn(m6809_t *c, int cpu)
{
	static const int8_t back[7] = { 0, -2, -3, -4, -5, -6, -7 };
	uint8_t *inc;
	for (int k = 0; k < 7; k++) {
		const uint16_t top = (uint16_t)(c->pc + back[k]);
		if (top == c->idle_pc[0] || top == c->idle_pc[1])
			return;
		const uint8_t op = op_byte(c, top);
		if (op != 0x96 && op != 0x0c && op != 0xb7 && op != 0xb6)
			continue;
		const int ora = op == 0x0c && op_byte(c, (uint16_t)(top + 4)) == 0x9a;
		const int len = op == 0x96 ? 4 : op == 0x0c ? (ora ? 8 : 6) : op == 0xb7 ? 7 : 5;
		if (c->pc - top >= len)
			continue;
		uint8_t *dummy;
		// (the form only: its condition may be true now)
		const uint8_t b = op_byte(c, (uint16_t)(top + len - 1));
		if (!((op == 0x96 && b == 0xfc) || (op == 0x0c && b == (ora ? 0xf8 : 0xfa)) || (op == 0xb7 && b == 0xf9) || (op == 0xb6 && b == 0xfb)))
			continue;
		(void)dummy;
		(void)cpu;
		c->idle_pc[1] = c->idle_pc[0];
		c->idle_pc[0] = top;
		return;
	}
}

static uint8_t idle_op(void *ctx, uint16_t a)
{
	m6809_t *c = ctx == NULL ? &s_main : ctx == (void *)&s_sub ? &s_sub : &s_snd;
	const int cpu = c == &s_main ? 0 : c == &s_sub ? 1 : 2;
	s_pcprof[cpu * 0x10000 + a]++;
	return op_byte(c, a);
}

// ---- the MCU (mcu_map) ----
static uint8_t in_ctl(int i) { return (uint8_t)rbx_port_read(s_port_ctl[i]); }
static uint8_t in_port(int i) { return (uint8_t)rbx_port_read(s_port_in[i]); }

static uint8_t berabohm_r(int off)
{
	if (off == 0) {
		const int inp = b.input_count;
		if (inp == 4) return in_ctl(0);
		int res = in_port(inp);
		if (res & 1) res = 0x7f;
		else if (res & 2) res = 0x48;
		else if (res & 4) res = 0x40;
		return (uint8_t)res;
	}
	int res = in_ctl(1) & 0x8f;
	uint8_t strobe_count = (uint8_t)(b.strobe_count + 1), strobe = b.strobe, input_count = b.input_count;
	if (strobe_count > 4) {
		strobe_count = 0;
		strobe ^= 0x40;
		if (!strobe)
			input_count = (uint8_t)((input_count + 1) % 5);
	}
	if (input_count == 3) res |= 0x10;
	res |= strobe;
	b.strobe_count = strobe_count;
	b.strobe = strobe;
	b.input_count = input_count;
	return (uint8_t)res;
}

static uint8_t faceoff_r(int off)
{
	if (off == 0)
		return (uint8_t)((in_ctl(0) & 0x80) | b.stored_input[0]);
	int res = in_ctl(1) & 0x80;
	uint8_t strobe_count = (uint8_t)(b.strobe_count + 1);
	if (strobe_count > 8) {
		strobe_count = 0;
		res |= b.input_count;
		switch (b.input_count) {
		case 0:
			b.stored_input[0] = in_port(0) & 0x1f;
			b.stored_input[1] = (uint8_t)((in_port(3) & 0x07) << 3);
			break;
		case 3:
			b.stored_input[0] = in_port(2) & 0x1f;
			break;
		case 4:
			b.stored_input[0] = in_port(1) & 0x1f;
			b.stored_input[1] = in_port(3) & 0x18;
			break;
		default:
			b.stored_input[0] = 0x1f;
			b.stored_input[1] = 0x1f;
			break;
		}
		b.input_count = (b.input_count + 1) & 7;
	} else
		res |= 0x40 | b.stored_input[1];
	b.strobe_count = strobe_count;
	return (uint8_t)res;
}

static uint8_t quester_r(int off)
{
	int ret;
	if (off == 0) {
		ret = (in_ctl(0) & 0x90) | (b.strobe & 0x40) | (b.paddle[(b.strobe & 0x20) ? 1 : 0] & 0x0f);
		b.strobe ^= 0x40;
	} else {
		if (!(b.strobe & 0x20))
			ret = (in_ctl(1) & 0x90) | 0x00 | (b.paddle[0] >> 4);
		else
			ret = (in_ctl(1) & 0x90) | 0x20 | (b.paddle[1] >> 4);
		if (!(b.strobe & 0x40))
			b.strobe ^= 0x20;
	}
	return (uint8_t)ret;
}

static uint8_t mcu_r(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x4000 && a < 0xc000) {   // (the voice ROM: the samples)
		const uint32_t o = (uint32_t)b.mcubank * 0x8000 + (a - 0x4000);
		return s_voice ? s_voice[rbx_wrap(o, s_voice_size)] : 0xff;
	}
	if (a >= 0xc000 && a < 0xc800) return b.triram[a & 0x7ff];
	if (a >= 0x1000 && a < 0x1004) {   // dsw_r: the LS157's nibble (offset bit 1), bits reversed
		const uint8_t d = (uint8_t)rbx_port_read(s_port_dsw);
		const uint8_t o = (a & 2) ? (uint8_t)(d >> 4) : (uint8_t)(d & 0xf);
		return (uint8_t)(0xf0 | ((o & 1) << 3) | ((o & 2) << 1) | ((o & 4) >> 1) | ((o & 8) >> 3));
	}
	if (a == 0x1400 || a == 0x1401) {
		switch (s_inputs) {
		case IN_BERABOHM: return berabohm_r(a & 1);
		case IN_FACEOFF: return faceoff_r(a & 1);
		case IN_QUESTER: return quester_r(a & 1);
		default: return in_ctl(a & 1);
		}
	}
	if (a >= 0x4000 && a < 0xc000) {
		const uint32_t o = (uint32_t)b.mcubank * 0x8000 + (a - 0x4000);
		return s_voice ? s_voice[rbx_wrap(o, s_voice_size)] : 0xff;
	}
	if (a >= 0xc000 && a < 0xc800) return b.triram[a & 0x7ff];
	if (a >= 0xc800 && a < 0xd000) return b.nvram[a & 0x7ff];
	if (a >= 0xf000) return s_mcurom[a & 0xfff];
	return 0;
}

static void mcu_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a == 0xc000) {   // mcu_patch_w (MAME's kludge)
		if (b.mcu_patch == 0xa6) return;
		b.mcu_patch = v;
		b.triram[0] = v;
	} else if (a > 0xc000 && a < 0xc800)
		b.triram[a & 0x7ff] = v;
	else if (a >= 0xc800 && a < 0xd000)
		b.nvram[a & 0x7ff] = v;
	else if (a == 0xd000)
		b.dac[0] = v;
	else if (a == 0xd400)
		b.dac[1] = v;
	else if (a == 0xd800) {   // mcu_bankswitch_w
		int bank;
		switch (v & 0xfc) {
		case 0xf8: bank = 0; v ^= 2; break;
		case 0xf4: bank = 4; break;
		case 0xec: bank = 8; break;
		case 0xdc: bank = 12; break;
		case 0xbc: bank = 16; break;
		case 0x7c: bank = 20; break;
		default: bank = 0; break;
		}
		b.mcubank = (uint8_t)(bank + (v & 3));
	} else if (a == 0xf000)
		m6801_set_irq_line(&s_mcu, M6801_IRQ1_LINE, 0);
}

static uint8_t mcu_port_r(void *u, int port)
{
	(void)u;
	return port == 1 ? (uint8_t)rbx_port_read(s_port_coin) : 0xff;
}

static void mcu_port_w(void *u, int port, uint8_t data, uint8_t ddr)
{
	(void)u;
	if (port == 2)
		b.dac_gain = (uint8_t)((data & ddr) | (ddr ^ 0xff));
}

// the C117's SUBRES: the sub CPU, the sound CPU and the MCU held in reset (subres_w)
static void sub_reset_line(int state)
{
	if (state != b.reset) {
		b.mcu_patch = 0;
		b.reset = (uint8_t)state;
	}
	if (state)
		b.held = 1;
	else if (b.held) {   // (released: they start from their reset vectors)
		b.held = 0;
		m6809_reset(&s_sub);
		m6809_reset(&s_snd);
		s_sub.cycles = b.t_sub;
		s_snd.cycles = b.t_snd;
	}
	m6801_set_reset(&s_mcu, state);
}

// ---- drawing ----
static const int s_adj[4] = { 4, 2, 1, 0 };

static void draw_layer(const snap_t *sn, int i, int vy, int x0, int x1, uint16_t *d, uint8_t *pri, int p)
{
	const uint8_t *vram = sn->vram;
	const uint16_t cbase = (uint16_t)(0x800 + (sn->control[0x18 + i] & 7) * 0x100);
	int ty, sx, cols, base;
	if (i < 4) {
		const int scrollx = (sn->control[i * 4] << 8) | sn->control[i * 4 + 1];
		const int scrolly = (sn->control[i * 4 + 2] << 8) | sn->control[i * 4 + 3];
		if (sn->flip) {   // (init_scroll(flip): the scroll negated; the tilemaps flipped against MAME's extents)
			ty = (-25 - vy - scrolly) & (i == 3 ? 0xff : 0x1ff);
			sx = -45 - s_adj[i] - scrollx;
		} else {
			ty = (vy + scrolly + 24) & (i == 3 ? 0xff : 0x1ff);
			sx = scrollx + 44 + s_adj[i];
		}
		cols = 64;
		base = i * 0x1000;
	} else {
		ty = sn->flip ? 223 - vy : vy;
		sx = sn->flip ? 287 : 0;
		cols = 36;
		base = i == 4 ? 0x3808 : 0x3c08;
	}
	const int row = (ty >> 3) * cols, fy = ty & 7;
	if (sn->flip) {   // (the tilemap's x going down: a tile's pixels from fx to 0)
		const int wm = i < 4 ? 0x1ff : 0x3ff;
		for (int x = x0; x < x1;) {
			const int tx = (sx - x) & wm;
			const int w = base + row + (tx >> 3);
			const uint32_t tile = rbx_wrap((uint32_t)((vram[w * 2] << 8) | vram[w * 2 + 1]) & 0x3fff, s_chr_count);
			const uint8_t m = s_mask[tile * 8 + (uint32_t)fy];
			int fx = tx & 7;
			if (!m) {
				x += fx + 1;
				continue;
			}
			const uint8_t *src = s_chr + tile * 64 + (uint32_t)fy * 8;
			for (; fx >= 0 && x < x1; fx--, x++)
				if (m & (0x80 >> fx)) {
					d[x] = (uint16_t)(cbase + src[fx]);
					pri[x] = (uint8_t)p;
				}
		}
		return;
	}
	for (int x = x0; x < x1;) {
		const int tx = i < 4 ? (x + sx) & 0x1ff : x;
		const int w = base + row + (tx >> 3);
		const uint32_t tile = rbx_wrap((uint32_t)((vram[w * 2] << 8) | vram[w * 2 + 1]) & 0x3fff, s_chr_count);
		const uint8_t m = s_mask[tile * 8 + (uint32_t)fy];
		const uint8_t *src = s_chr + tile * 64 + (uint32_t)fy * 8;
		int fx = tx & 7;
		if (!m) {   // (a transparent row of the tile)
			x += 8 - fx;
			continue;
		}
		if (m == 0xff && fx == 0 && x + 8 <= x1) {   // (an opaque one)
			for (int k = 0; k < 8; k++) {
				d[x + k] = (uint16_t)(cbase + src[k]);
				pri[x + k] = (uint8_t)p;
			}
			x += 8;
			continue;
		}
		for (; fx < 8 && x < x1; fx++, x++)
			if (m & (0x80 >> fx)) {
				d[x] = (uint16_t)(cbase + src[fx]);
				pri[x] = (uint8_t)p;
			}
	}
}

static void draw_sprites(const snap_t *sn, int cx0, int cx1, int cy0, int cy1)
{
	static const int ssz[4] = { 16, 8, 32, 4 };
	const uint8_t *spr = sn->spr;
	const int xoffs = spr[0x7f5] + ((spr[0x7f4] & 1) << 8), yoffs = spr[0x7f7];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int o = 0x7e0; o >= 0; o -= 0x10) {
		const uint8_t *s = spr + o;
		const uint8_t a1 = s[10], a2 = s[14];
		int color = s[12];
		const int fx = (a1 >> 5) & 1, fy = a2 & 1;
		const int w = ssz[a1 >> 6], h = ssz[(a2 >> 1) & 3];
		const int tx = (a1 & 0x18) & ~(w - 1), ty = (a2 & 0x18) & ~(h - 1);
		int sx = s[13] + ((color & 1) << 8), sy = -s[15] - h;
		const uint32_t code = rbx_wrap((uint32_t)s[11] + ((uint32_t)(a1 & 7) << 8), s_spr_count);
		const uint32_t pmask = ((0xffu << (((a2 & 0xe0) >> 5) + 1)) & 0xff) | 0x80000000u;
		color >>= 1;
		sx += xoffs;
		sy -= yoffs;
		int fx2 = fx, fy2 = fy;
		if (sn->flip) {
			sx = -sx - w;
			sy = -sy - h;
			fx2 = !fx;
			fy2 = !fy;
		}
		sy++;
		const int x0 = sx & 0x1ff, y0 = ((sy + 16) & 0xff) - 16;
		const uint8_t *tile = s_sprpix + code * 1024;
		for (int yy = 0; yy < h; yy++) {
			const int ay = y0 + yy;
			if (ay < cy0 || ay >= cy1)
				continue;
			const uint8_t *src = tile + (uint32_t)(ty + (fy2 ? h - 1 - yy : yy)) * 32 + (uint32_t)tx;
			uint16_t *d = bm + (ay - VIS_Y0) * pitch - VIS_X0;
			uint8_t *pri = s_nspri + (ay - VIS_Y0) * VIS_W - VIS_X0;
			for (int xx = 0; xx < w; xx++) {
				const int ax = x0 + xx;
				if (ax < cx0 || ax >= cx1)
					continue;
				const uint8_t pen = src[fx2 ? w - 1 - xx : xx];
				if (pen == 15)
					continue;
				if (!((1u << pri[ax]) & pmask)) {
					if (color == 0x7f) {   // the shadow: the tilemaps' colours to their shadow page
						const uint16_t c = d[ax];
						if (c >= 0x800 && c < 0x1000)
							d[ax] = (uint16_t)(c + 0x800);
					} else
						d[ax] = (uint16_t)(color * 16 + pen);
				}
				pri[ax] = 31;
			}
		}
	}
}

// the frame in bands: the render thread and the helpers (render_help) take them in turn
#define BANDS 8
static int s_band_next, s_band_left, s_band_active, s_band_helpers;
static int s_cx0, s_cx1, s_cy0, s_cy1;   // (the C116's window this frame; s_cx0 > s_cx1: nothing)
static int s_pal_valid;

static void run_band(int band)
{
	const snap_t *sn = s_snap;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = PAL_BLACK;
	}
	int cy0 = s_cy0 - VIS_Y0, cy1 = s_cy1 + 1 - VIS_Y0;
	if (cy0 < y0) cy0 = y0;
	if (cy1 > y1) cy1 = y1;
	if (s_cx0 > s_cx1 || cy0 >= cy1)
		return;
	int order[6], n = 0;   // the enabled layers by priority (MAME: priority 0..7, layer 0..5 within one)
	for (int p = 0; p < 8; p++)
		for (int i = 0; i < 6; i++) {
			const uint8_t c = sn->control[0x10 + i];
			if ((c & 7) == p && !(c & 8))
				order[n++] = i;
		}
	memset(s_nspri + cy0 * VIS_W, 0, (size_t)(cy1 - cy0) * VIS_W);
	for (int vy = cy0; vy < cy1; vy++) {
		uint16_t *d = bm + vy * pitch;
		uint8_t *pri = s_nspri + vy * VIS_W;
		for (int k = 0; k < n; k++)
			draw_layer(sn, order[k], vy, s_cx0 - VIS_X0, s_cx1 + 1 - VIS_X0, d, pri, sn->control[0x10 + order[k]] & 7);
	}
	draw_sprites(sn, s_cx0, s_cx1 + 1, cy0 + VIS_Y0, cy1 + VIS_Y0);
}

static void ns1_render(void)
{
	snap_t *sn = s_snap;
	if (sn->pal_dirty || !s_pal_valid) {
		uint32_t *pal = rbx_video.palette;
		for (int i = 0; i < 0x2000; i++)
			pal[i] = (uint32_t)(sn->pal_r[i] << 16 | sn->pal_g[i] << 8 | sn->pal_b[i]);
		pal[PAL_BLACK] = 0;
		sn->pal_dirty = 0;
		s_pal_valid = 1;
	}
	// the C116's window (berabohm's iris), in the screen's coordinates
	s_cx0 = sn->c116_regs[0] - 1;
	s_cx1 = sn->c116_regs[1] - 1 - 1;
	s_cy0 = sn->c116_regs[2] - 0x11;
	s_cy1 = sn->c116_regs[3] - 0x11 - 1;
	if (s_cx0 < VIS_X0) s_cx0 = VIS_X0;
	if (s_cx1 > VIS_X0 + VIS_W - 1) s_cx1 = VIS_X0 + VIS_W - 1;
	if (s_cy0 < VIS_Y0) s_cy0 = VIS_Y0;
	if (s_cy1 > VIS_Y0 + VIS_H - 1) s_cy1 = VIS_Y0 + VIS_H - 1;
	__atomic_store_n(&s_band_next, 0, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_left, BANDS, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_active, 1, __ATOMIC_SEQ_CST);
	for (;;) {
		const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
		if (bnd >= BANDS)
			break;
		run_band(bnd);
		__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
	}
	while (__atomic_load_n(&s_band_left, __ATOMIC_ACQUIRE) > 0)
		;
	__atomic_store_n(&s_band_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_band_helpers, __ATOMIC_ACQUIRE) > 0)
		;
}

static int ns1_render_help(int helper)
{
	const int one = helper & 0x100;
	helper &= 0xff;
	if (helper < 1 || helper > 2 || !__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		for (;;) {
			const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
			if (bnd >= BANDS)
				break;
			run_band(bnd);
			__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
			did = 1;
			if (one)
				break;
		}
	__atomic_sub_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->control, b.control, sizeof(b.control));
	memcpy(s_snap->spr, b.spr, sizeof(b.spr));
	memcpy(s_snap->vram, b.vram, sizeof(b.vram));
	if (s_pal_dirty) {   // (written since the last copy)
		memcpy(s_snap->pal_r, b.pal_r, sizeof(b.pal_r));
		memcpy(s_snap->pal_g, b.pal_g, sizeof(b.pal_g));
		memcpy(s_snap->pal_b, b.pal_b, sizeof(b.pal_b));
		s_snap->pal_dirty = 1;
		s_pal_dirty = 0;
	}
	memcpy(s_snap->c116_regs, b.c116_regs, sizeof(b.c116_regs));
	s_snap->flip = b.flip;
	rbx_video_frame_done();
}

// ---- frame ----
static void ns1_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	s_mix_n = 0;
	for (int p = 0; p < 2; p++)
		if (s_port_paddle[p] >= 0)
			b.paddle[p] = (uint8_t)(b.paddle[p] + rbx_analog_dpad(p + 1, RBX_IPT_DIAL, 15));
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0) {   // the end of vblank (screen_vblank 0)
			m6809_set_irq(&s_snd, 1);
			m6801_set_irq_line(&s_mcu, M6801_IRQ1_LINE, 1);
		}
		if (line == VIS_Y0 + VIS_H) {   // vblank: the frame drawn, then the sprites copied and the IRQs
			vblank_capture();
			if (b.copy_sprites) {
				for (int i = 0; i < 0x800; i += 16)
					for (int j = 10; j < 16; j++)
						b.spr[i + j] = b.spr[i + j - 6];
				b.copy_sprites = 0;
			}
			m6809_set_irq(&s_main, 1);
			m6809_set_irq(&s_sub, 1);
		}
		for (int sl = 0; sl < SLICES; sl++) {
			const int n = rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES * SLICES);
			b.t_main += n;
			b.t_sub += n;
			b.t_snd += n;
			s_tgt[0] = b.t_main;
			s_tgt[1] = b.t_sub;
			s_tgt[2] = b.t_snd;
			m6809_run_until(&s_main, b.t_main);
			if (b.held) {
				s_sub.cycles = b.t_sub;
				s_snd.cycles = b.t_snd;
			} else {
				m6809_run_until(&s_sub, b.t_sub);
				m6809_run_until(&s_snd, b.t_snd);
			}
			if (s_main.idle_hook && sl == SLICES - 1) {   // (a loop's top found where a CPU stopped, once a line)
				idle_learn(&s_main, 0);
				if (!b.held) {
					idle_learn(&s_sub, 1);
					idle_learn(&s_snd, 2);
				}
			}
			m6801_execute(&s_mcu, n);
			if (s_pcprof)
				s_pcprof[3 * 0x10000 + s_mcu.pc.w.l]++;
			// the sound of this slice
			const uint32_t yc = (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, FRAME_MHZ, LINES * SLICES);
			const int k = rbx_ym_advance_lr(0, yc, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
			if (k > 0) {
				namco_wsg_render(&s_wsg, s_mix_l + s_mix_n, s_mix_r + s_mix_n, k, s_ym_rate);
				const int g0 = ((b.dac_gain >> 1) & 2) | (b.dac_gain & 1), g1 = ((b.dac_gain >> 3) & 2) | ((b.dac_gain >> 3) & 1);
				const int32_t dv = ((b.dac[0] * 2 - 255) * 4096 * (g0 + 1) + (b.dac[1] * 2 - 255) * 4096 * (g1 + 1)) / 255;
				for (int j = 0; j < k; j++) {
					s_mix_l[s_mix_n + j] += dv;
					s_mix_r[s_mix_n + j] += dv;
				}
				s_mix_n += k;
			}
		}
	}
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	for (int i = 0; i < s_mix_n; i++) {
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
		s_mix_l[i] = s_mix_r[i] = 0;
	}
}

// ---- init / reset ----
static void all_pages(void)
{
	for (int c = 0; c < 2; c++)
		for (int k = 0; k < 8; k++)
			bank_pages(c, k);
	snd_bank();
}

static void ns1_reset(void)
{
	memset(&b.offsets, 0, sizeof(b.offsets));
	b.offsets[0][0] = b.offsets[0][1] = 0x0180 * 0x2000;
	b.offsets[0][7] = 0x03ff * 0x2000;
	b.offsets[1][0] = 0x0180 * 0x2000;
	b.offsets[1][7] = 0x03ff * 0x2000;
	b.subres = 0;
	b.copy_sprites = b.flip = 0;
	b.soundbank = b.mcubank = 0;
	b.dac[0] = b.dac[1] = 0;
	b.dac_gain = 0xff;
	b.strobe = b.strobe_count = b.input_count = 0;
	b.stored_input[0] = b.stored_input[1] = 0;
	memset(b.key, 0, sizeof(b.key));
	b.key_quotient = b.key_reminder = b.key_hi = 0;
	b.mcu_patch = 0;
	b.rng = 0x12345678;
	b.t_main = b.t_sub = b.t_snd = 0;
	memset(s_iform, 0, sizeof(s_iform));
	b.cpu_frac = b.ym_frac = 0;
	all_pages();
	m6809_reset(&s_main);
	s_main.cycles = 0;
	m6809_reset(&s_sub);
	m6809_reset(&s_snd);
	s_sub.cycles = s_snd.cycles = 0;
	m6801_reset(&s_mcu);
	b.reset = 0;
	b.held = 0;
	sub_reset_line(1);   // (the C117's reset: SUBRES asserted)
	rbx_ym_reset(0);
	namco_wsg_init(&s_wsg, NAMCO_WSG_TYPE_CUS30, 49152000 / 2048 / 2, 1, NULL, 128);
}

static const struct {
	const char *init;
	int type, id, reg, rng, swap4_arg, swap4, bottom4, top4;
} s_keys[] = {
	{ "dspirit", KEY_1, 0x36, 0, 0, 0, 0, 0, 0 },
	{ "wldcourt", KEY_1, 0x35, 0, 0, 0, 0, 0, 0 },
	{ "blazer", KEY_1, 0x13, 0, 0, 0, 0, 0, 0 },
	{ "puzlclub", KEY_1, 0x35, 0, 0, 0, 0, 0, 0 },
	{ "pacmania", KEY_2, 0x12, 0, 0, 0, 0, 0, 0 },
	{ "alice", KEY_2, 0x25, 0, 0, 0, 0, 0, 0 },
	{ "galaga88", KEY_2, 0x31, 0, 0, 0, 0, 0, 0 },
	{ "ws", KEY_2, 0x07, 0, 0, 0, 0, 0, 0 },
	{ "bakutotu", KEY_2, 0x22, 0, 0, 0, 0, 0, 0 },
	{ "splatter", KEY_3, 181, 3, 4, -1, -1, -1, -1 },
	{ "rompers", KEY_3, 182, 7, -1, -1, -1, -1, -1 },
	{ "blastoff", KEY_3, 183, 0, 7, 3, 5, -1, -1 },
	{ "ws89", KEY_3, 184, 2, -1, -1, -1, -1, -1 },
	{ "tankfrce", KEY_3, 185, 5, -1, 1, -1, 2, -1 },
	{ "tankfrc4", KEY_3, 185, 5, -1, 1, -1, 2, -1 },
	{ "dangseed", KEY_3, 308, 6, -1, 5, -1, 0, 4 },
	{ "pistoldm", KEY_3, 309, 1, 2, 0, -1, 4, -1 },
	{ "ws90", KEY_3, 310, 4, -1, 7, -1, 3, -1 },
	{ "soukobdx", KEY_3, 311, 2, 3, 0, -1, 4, -1 },
};

static void ns1_exit(void);

static int ns1_init(const rbx_game *g)
{
	s_mainrom = rbx_region("mainrom", &s_mainrom_size);
	s_sndrom = rbx_region("audiocpu", &s_sndrom_size);
	s_mcurom = rbx_region("mcu", &s_mcurom_size);
	s_voice = rbx_region("voice", &s_voice_size);
	s_chr = rbx_region("c123tmap", &s_chr_size);
	s_mask = rbx_region("c123tmap:mask", &s_mask_size);
	s_sprrom = rbx_region("sprite", &s_sprrom_size);
	if (!s_mainrom || s_mainrom_size < 0x400000 || !s_sndrom || s_sndrom_size < 0x4000 || !s_mcurom || s_mcurom_size < 0x1000 ||
	    !s_chr || !s_mask || !s_sprrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// driver_init: bit 16 of the address inverted for PRG7
	for (uint32_t i = 0x380000; i < 0x400000; i++)
		if (!(i & 0x10000)) {
			const uint8_t t = s_mainrom[i];
			s_mainrom[i] = s_mainrom[i + 0x10000];
			s_mainrom[i + 0x10000] = t;
		}
	s_chr_count = s_chr_size / 64;
	if (s_mask_size / 8 < s_chr_count)
		s_chr_count = s_mask_size / 8;
	s_spr_count = s_sprrom_size / 512;
	s_sprpix = (uint8_t *)malloc((size_t)s_spr_count * 1024 + 1);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	s_nspri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	if (!s_sprpix || !s_snap || !s_nspri) {
		ns1_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	// the 32x32 4bpp sprites: 16x16 quarters (x at +128 bytes, y at +256), a row of 16 pixels in 8 bytes
	for (uint32_t t = 0; t < s_spr_count; t++)
		for (int y = 0; y < 32; y++)
			for (int x = 0; x < 32; x++) {
				const uint8_t v = s_sprrom[t * 512 + (uint32_t)((y >> 4) * 256 + (y & 15) * 8 + (x >> 4) * 128 + (x & 15) / 2)];
				s_sprpix[t * 1024 + (uint32_t)(y * 32 + x)] = (x & 1) ? (v & 15) : (v >> 4);
			}
	// the set's key chip and inputs
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	s_key_type = KEY_NONE;
	s_key_id = s_key_reg = s_key_rng = s_key_swap4_arg = s_key_swap4 = s_key_bottom4 = s_key_top4 = 0;
	for (size_t i = 0; i < sizeof(s_keys) / sizeof(s_keys[0]); i++)
		if (!strcmp(init, s_keys[i].init)) {
			s_key_type = s_keys[i].type;
			s_key_id = s_keys[i].id;
			s_key_reg = s_keys[i].reg;
			s_key_rng = s_keys[i].rng;
			s_key_swap4_arg = s_keys[i].swap4_arg;
			s_key_swap4 = s_keys[i].swap4;
			s_key_bottom4 = s_keys[i].bottom4;
			s_key_top4 = s_keys[i].top4;
		}
	s_inputs = !strcmp(init, "berabohm") ? IN_BERABOHM
	         : (!strcmp(init, "faceoff") || !strcmp(init, "tankfrc4")) ? IN_FACEOFF
	         : !strcmp(init, "quester") ? IN_QUESTER : IN_STD;
	s_port_ctl[0] = rbx_port_find("CONTROL0");
	s_port_ctl[1] = rbx_port_find("CONTROL1");
	s_port_dsw = rbx_port_find("DIPSW");
	s_port_coin = rbx_port_find("COIN");
	static const char *const in_tags[4] = { "IN0", "IN1", "IN2", "IN3" };
	for (int i = 0; i < 4; i++)
		s_port_in[i] = rbx_port_find(in_tags[i]);
	s_port_paddle[0] = rbx_port_find("PADDLE0");
	s_port_paddle[1] = rbx_port_find("PADDLE1");

	memset(&b, 0, sizeof(b));
	// the CPUs
	m6809_init(&s_main);
	s_main.read = main_r;
	s_main.write = main_w;
	s_main.ctx = NULL;
	m6809_init(&s_sub);
	s_sub.read = main_r;
	s_sub.write = main_w;
	s_sub.ctx = &s_sub;   // (non-NULL: the sub CPU)
	m6809_init(&s_snd);
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	for (int p = 0x70; p < 0x80; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = b.triram + (p & 7) * 256;
	for (int p = 0x80; p < 0xa0; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = b.sndram + (p - 0x80) * 256;
	for (int p = 0xc0; p < 0x100; p++)
		s_snd.rpage[p] = (uint32_t)(p - 0xc0) * 256 + 256 <= s_sndrom_size ? s_sndrom + (p - 0xc0) * 256 : NULL;
	s_snd.ctx = &s_snd;
	s_noidle = getenv("NS1_NOIDLE") != NULL;
	if (getenv("NS1_PCPROF"))
		s_pcprof = (uint32_t *)calloc(4 * 0x10000, sizeof(uint32_t));
	if (s_pcprof)
		s_main.read_op = s_sub.read_op = s_snd.read_op = idle_op;
	else if (!s_noidle)
		s_main.idle_hook = s_sub.idle_hook = s_snd.idle_hook = idle_hook;
	memset(&s_mcu, 0, sizeof(s_mcu));
	s_mcu.read = mcu_r;
	s_mcu.write = mcu_w;
	s_mcu.port_r = mcu_port_r;
	s_mcu.port_w = mcu_port_w;
	s_mcu.rom = s_mcurom;
	s_mcu.rom_base = 0xf000;
	s_mcu.rom_size = 0x1000;
	s_mcu.type = M6801_TYPE_HD63701V0;
	m6801_init(&s_mcu);

	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 128;
	yc.irq = ym_irq;
	int rc = rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2);   // (its samples made on the worker)
	s_ym_rate = rc ? 0 : rbx_ym_rate(0);
	s_mix_cap = (int)(s_ym_rate / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	rc = rc || !s_mix_l || !s_mix_r || rbx_audio_setup_stereo(s_ym_rate) || rbx_video_setup(VIS_W, VIS_H, 0x2001);
	if (rc) {
		ns1_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_pal_valid = 0;
	s_pal_dirty = 1;
	ns1_reset();
	return 0;
}

static void ns1_exit(void)
{
	if (s_pcprof) {
		for (int cpu = 0; cpu < 4; cpu++) {
			uint64_t tot = 0;
			for (int i = 0; i < 0x10000; i++) tot += s_pcprof[cpu * 0x10000 + i];
			for (int k = 0; k < 6; k++) {
				int best = 0;
				for (int i = 1; i < 0x10000; i++)
					if (s_pcprof[cpu * 0x10000 + i] > s_pcprof[cpu * 0x10000 + best]) best = i;
				m6809_t *c = cpu == 0 ? &s_main : cpu == 1 ? &s_sub : &s_snd;
				uint8_t by[6];
				for (int j = 0; j < 6; j++) {
					const uint16_t ad = (uint16_t)(best + j);
					by[j] = cpu == 3 ? (ad >= 0xf000 ? s_mcurom[ad & 0xfff] : 0) : c->rpage[ad >> 8] ? c->rpage[ad >> 8][ad & 0xff] : c->read(c->ctx, ad);
				}
				rbx_log("PC cpu%d %04x %u (%.1f%%) %02x %02x %02x %02x %02x %02x dp %02x", cpu, best, s_pcprof[cpu * 0x10000 + best],
				        tot ? 100.0 * s_pcprof[cpu * 0x10000 + best] / (double)tot : 0.0, by[0], by[1], by[2], by[3], by[4], by[5], c->dp);
				s_pcprof[cpu * 0x10000 + best] = 0;
			}
		}
		free(s_pcprof);
		s_pcprof = NULL;
	}
	rbx_ym_exit(0);
	free(s_sprpix);
	free(s_snap);
	free(s_nspri);
	free(s_mix_l);
	free(s_mix_r);
	s_sprpix = NULL;
	s_snap = NULL;
	s_nspri = NULL;
	s_mix_l = s_mix_r = NULL;
}

static int ns1_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "board", &b, sizeof(b));
	rc |= rbx_state_io(ctx, "main", &s_main, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sub", &s_sub, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "snd", &s_snd, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "mcu", m6801_state_data(&s_mcu), m6801_state_size());
	rc |= rbx_state_io(ctx, "wsg", &s_wsg, (uint32_t)NAMCO_WSG_STATE_SIZE);
	rc |= rbx_ym_state(0, ctx, load);
	if (load && !rc) {
		all_pages();
		s_pal_dirty = 1;
	}
	return rc;
}

static int ns1_save(void *ctx) { return ns1_state(ctx, 0); }
static int ns1_load(void *ctx) { return ns1_state(ctx, 1); }
static int ns1_supported(const char *machine) { return !strncmp(machine, "ns1:", 4) || !strncmp(machine, "quester:", 8); }

const rbx_driver rbx_drv_namcos1 = { "Namco System 1", ns1_init, ns1_exit, ns1_reset, ns1_frame, ns1_save, ns1_load,
                                     ns1_supported, ns1_render, ns1_render_help };
