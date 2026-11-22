/*  Data East Astro Fighter, Space Fighter Mark II and Tomahawk 777. Ported from MAME src/mame/dataeast/astrof.cpp
    (BSD-3-Clause, copyright-holders: Lee Taylor).
    A 6502 at 662 kHz (an IRQ at the vblank start held until a read of 0xa002, the coins on its NMI), a 256x256 1bpp
    bitmap with a colour per 8x2 cell latched at each write, the pens from a PROM (a palette bank, a red gun override).
    MAME's partial updates on the video control writes: lines drawn as the beam passes them.
    The bootlegs: the ROMs decrypted through a PROM or inverted, a protection read and a coin counter.
    Not emulated: the sound (Astro Fighter plays MAME samples, Tomahawk an SN76477; Space Fighter has none in MAME).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"

#include <stdlib.h>
#include <string.h>

#define CLOCK (10595000 / 16)
#define FRAME_MHZ 56308          // set_raw(10.595 MHz / 2, 0x150, 0, 0x100, 0x118, 0, 0x100)
#define LINES 0x118
#define VBSTART 0x100

enum { M_ASTROF, M_ABATTLE, M_SPFGHMK2, M_TOMAHAWK };
enum { PROT_NONE, PROT_ABATTLE, PROT_AFIRE };

static int s_machine, s_prot;
static uint8_t *s_rom, *s_prom;
static uint32_t s_rom_size, s_prom_size;
static uint8_t s_ram[0x400], s_vram[0x2000], s_cram[0x1000];
static uint8_t s_color, s_tprot, s_flip, s_screen_off, s_bank, s_red_on;
static uint16_t s_abattle_count;
static int s_coin_nmi, s_irq;
static int s_port_in, s_port_dsw, s_port_p1, s_port_p2, s_port_cab, s_port_coin, s_port_fake;
static int s_line;               // (from the vblank start, as MAME's screen at time 0)
static uint32_t s_frame;
static int64_t s_target;
static uint64_t s_frac;
static uint32_t s_rand;
static m6502_t s_cpu;
static uint8_t (*s_lines_hp)[256 * 256];   // pens drawn as the beam passes (64 = black)
#define s_lines (*s_lines_hp)
static uint8_t *s_snap;

static int vblank(void) { return s_frame && s_line < LINES - VBSTART; }

// ---- inputs ----
static uint8_t in_r(void)
{
	const uint32_t p1 = rbx_port_read(s_port_p1);
	if (s_machine == M_TOMAHAWK) {   // tomahawk_controls_r: the two sets multiplexed by the flip
		rbx_port_custom(s_port_in, 0x1f, (s_flip ? rbx_port_read(s_port_p2) : p1) & 0x1f);
	} else {   // astrof_p1 / p2_controls_r: an upright cabinet's one set on both
		const uint32_t p2 = rbx_port_read(s_port_cab) & 1 ? rbx_port_read(s_port_p2) : p1;
		rbx_port_custom(s_port_in, 0xfc, ((p1 & 7) << 2) | ((p2 & 7) << 5));
	}
	return (uint8_t)rbx_port_read(s_port_in);
}
static uint8_t dsw_r(void)
{
	rbx_port_custom(s_port_dsw, 0x80, vblank() ? 0x80 : 0);
	return (uint8_t)rbx_port_read(s_port_dsw);
}

// the coins on the NMI (coin_inserted: newval ? ASSERT : CLEAR)
static void coins(void)
{
	const int nmi = (rbx_port_read(s_port_coin) & 3) != 0;
	if (nmi != s_coin_nmi)
		m6502_set_nmi(&s_cpu, nmi);
	s_coin_nmi = nmi;
}

// ---- bus ----
static uint8_t main_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x2000) return s_ram[a & 0x3ff];
	if (a >= 0x4000 && a < 0x6000) return s_vram[a - 0x4000];
	if (a >= 0xc000) return s_rom[a];
	if (a >= 0xa000 && a < 0xc000)
		switch (a & 7) {
		case 0: return in_r();
		case 1: return dsw_r();
		case 2:   // irq_clear_r
			s_irq = 0;
			m6502_set_irq(&s_cpu, 0);
			return 0;
		case 3:
			if (s_machine == M_TOMAHAWK) {   // tomahawk_protection_r: the byte reversed
				uint8_t r = 0;
				for (int i = 0; i < 8; i++)
					r |= (uint8_t)(((s_tprot >> i) & 1) << (7 - i));
				return r;
			}
			if (s_prot != PROT_NONE) {   // shoot_r: machine().rand() & 8
				s_rand = s_rand * 1103515245u + 12345u;
				return (uint8_t)((s_rand >> 16) & 8);
			}
			return 0;
		case 4:
			if (s_prot == PROT_ABATTLE) {   // abattle_coin_prot_r
				s_abattle_count = (uint16_t)((s_abattle_count + 1) % 0x0101);
				return s_abattle_count ? 0x07 : 0x00;
			}
			if (s_prot == PROT_AFIRE) {   // afire_coin_prot_r
				s_abattle_count ^= 1;
				return s_abattle_count ? 0x07 : 0x00;
			}
			return 0;
		}
	return 0;
}

static void main_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000) {
		s_ram[a & 0x3ff] = v;
	} else if (a >= 0x4000 && a < 0x6000) {   // the colour latched for the 8x2 cell (A0 not on the colour RAM)
		s_vram[a - 0x4000] = v;
		s_cram[(a - 0x4000) >> 1] = (uint8_t)(s_machine == M_TOMAHAWK ? (s_color & 0x0e) | ((s_color & 1) << 4)
		                                                            : s_color & 0x0e);
	} else if (a >= 0x8000 && a < 0xa000) {
		switch (a & 7) {
		case 3: s_color = v; break;
		case 4:   // video_control_1_w: the flip (a cocktail cabinet only), the shift registers' clear
			s_flip = (uint8_t)(v & 1 & rbx_port_read(s_port_cab));
			s_screen_off = (v >> 1) & 1;
			break;
		case 5:   // video_control_2_w: D2 the palette bank (not Tomahawk), D3 the red gun (not Space Fighter)
			if (s_machine != M_TOMAHAWK)
				s_bank = (v >> 2) & 1;
			if (s_machine != M_SPFGHMK2)
				s_red_on = (v >> 3) & 1;
			break;
		case 7:
			if (s_machine == M_TOMAHAWK)
				s_tprot = v;
			break;
		}
	}
}

// ---- video ----
// the pen of a cell: astrof_get_pens / tomahawk_get_pens with the PROM wire mod ("FAKE"), then the red override
static uint8_t pen(uint8_t color, int fg)
{
	const int config = s_port_fake >= 0 ? (int)(rbx_port_read(s_port_fake) & 3) : 0;
	int i;
	if (s_machine == M_TOMAHAWK) {
		i = (color & 0x1f) | fg;
		i = config == 0 ? i : config == 1 ? i ^ 0x10 : config == 2 ? i & 0x0f : i | 0x10;
	} else {
		int bank = s_bank ? 0x10 : 0;
		bank = config == 0 ? bank : config == 1 ? bank ^ 0x10 : config == 2 ? 0 : 0x10;
		i = bank | (color & 0x0e) | fg;
	}
	return (uint8_t)(i | (s_red_on ? 32 : 0));
}

// one screen line (video_update_common; its first and last lines are never drawn)
static void draw_line(int y)
{
	uint8_t *d = s_lines + y * 256;
	if (y == 0 || y == 255)
		return;
	const uint8_t lo = (uint8_t)(s_flip ? y : ~y);
	for (int c = 0; c < 32; c++) {
		const int offs = c << 8 | lo;
		const uint8_t color = s_cram[offs >> 1];
		const uint8_t back = pen(color, 0), fore = pen(color, 1);
		uint8_t data = s_screen_off ? 0 : s_vram[offs];
		for (int i = 0; i < 8; i++, data >>= 1) {
			const int x = c * 8 + i;
			d[s_flip ? 255 - x : x] = (data & 1) ? fore : back;
		}
	}
}

static void af_render(void)
{
	for (int i = 0; i < 64; i++) {   // make_pen: D0-1 red, D2-3 green, D4-5 blue (0xc0 and 0x3f), the red on
		const uint8_t p = (uint32_t)(i & 31) < s_prom_size ? s_prom[i & 31] : 0;
		const int red = i >= 32;
		const int r = (red || (p & 1) ? 0xc0 : 0) + (red || (p & 2) ? 0x3f : 0);
		const int g = ((p >> 2) & 1) * 0xc0 + ((p >> 3) & 1) * 0x3f;
		const int b = ((p >> 4) & 1) * 0xc0 + ((p >> 5) & 1) * 0x3f;
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	rbx_video.palette[64] = 0;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < 256; y++)
		for (int x = 0; x < 256; x++)
			bm[y * pitch + x] = s_snap[y * 256 + x];
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap, s_lines, sizeof(s_lines));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void af_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_cram, 0, sizeof(s_cram));
	memset(s_lines, 64, sizeof(s_lines));
	s_color = s_tprot = s_flip = s_screen_off = 0;
	s_bank = s_machine != M_TOMAHAWK;   // (the 74175 outputs all high: set_video_control_2(0xff))
	s_red_on = s_machine != M_SPFGHMK2;
	s_abattle_count = 0;
	s_coin_nmi = s_irq = 0;
	s_rand = 0;
	m6502_reset(&s_cpu);
	m6502_set_irq(&s_cpu, 0);
	m6502_set_nmi(&s_cpu, 0);
	s_line = 0;
	s_frame = 0;
	s_target = 0;
	s_frac = 0;
}

static void af_exit(void);

static int af_init(const rbx_game *g)
{
	if (!s_lines_hp && !(s_lines_hp = calloc(1, sizeof(*s_lines_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine, *s = g->set;
	s_machine = !strcmp(m, "tomahawk") ? M_TOMAHAWK : !strcmp(m, "spfghmk2") ? M_SPFGHMK2 :
	            !strcmp(m, "abattle") ? M_ABATTLE : M_ASTROF;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_prom = rbx_region("proms", &s_prom_size);
	if (!s_rom || s_rom_size < 0x10000 || !s_prom) {
		rbx_fail("%s: ROM regions missing", s);
		return -1;
	}
	// the drivers' inits (once: the regions are loaded afresh for each game)
	s_prot = PROT_NONE;
	if (s_machine == M_ABATTLE) {
		uint32_t us;
		const uint8_t *u = rbx_region("user1", &us);
		if (!strcmp(s, "abattle") || !strcmp(s, "abattle2") || !strcmp(s, "acombat4")) {   // init_abattle
			if (!u || us < 0x100) {
				rbx_fail("%s: ROM regions missing", s);
				return -1;
			}
			for (int i = 0xd000; i < 0x10000; i++)
				s_rom[i] = u[s_rom[i]];
			s_prot = PROT_ABATTLE;
		} else if (!strcmp(s, "asterion")) {   // init_asterion
			if (!u || us < 0x200) {
				rbx_fail("%s: ROM regions missing", s);
				return -1;
			}
			for (int i = 0xd000; i < 0x10000; i++)
				s_rom[i] = u[(s_rom[i] & 0x1f) + ((s_rom[i] & 0xe0) << 1)];
			s_prot = PROT_ABATTLE;
		} else if (!strcmp(s, "afire") || !strcmp(s, "acombat") || !strcmp(s, "acombato")) {   // init_afire
			for (int i = 0xd000; i < 0x10000; i++)
				s_rom[i] = (uint8_t)~s_rom[i];
			s_prot = PROT_AFIRE;
		} else if (!strcmp(s, "sstarbtl")) {   // init_sstarbtl
			for (int i = 0xd000; i < 0x10000; i++)
				s_rom[i] = (uint8_t)~s_rom[i];
			s_prot = PROT_ABATTLE;
		} else {   // init_acombat3 (acombat3, strfight)
			s_prot = PROT_ABATTLE;
		}
	}
	s_snap = (uint8_t *)malloc(sizeof(s_lines));
	if (!s_snap || rbx_audio_setup(48000) || rbx_video_setup(256, 256, 65)) {
		rbx_fail("%s: out of memory", s);
		af_exit();
		return -1;
	}
	memset(s_snap, 64, sizeof(s_lines));
	m6502_init(&s_cpu, M6502_NMOS);
	s_cpu.read = main_r;
	s_cpu.write = main_w;
	for (int p = 0xc0; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + (p & 3) * 256;
	for (int p = 0x40; p < 0x60; p++)
		s_cpu.rpage[p] = s_vram + (p - 0x40) * 256;
	s_port_in = rbx_port_find("IN");
	s_port_dsw = rbx_port_find("DSW");
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_cab = rbx_port_find("CAB");
	s_port_coin = rbx_port_find("COIN");
	s_port_fake = rbx_port_find("FAKE");
	af_reset();
	return 0;
}

static void af_exit(void)
{
	free(s_snap);
	s_snap = NULL;
	free(s_lines_hp);
	s_lines_hp = NULL;
}

static void af_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coins();
	for (int i = 0; i < LINES; i++) {   // (from line 0x100, the vblank start)
		s_line = i;
		const int y = (i + VBSTART) % LINES;
		if (i == 0 && s_frame) {   // the scanline timer at VBSTART (its first at the next frame)
			s_irq = 1;
			m6502_set_irq(&s_cpu, 1);
		}
		s_target += rbx_line_cycles(&s_frac, CLOCK, FRAME_MHZ, LINES);
		m6502_run_until(&s_cpu, s_target);
		if (y < 256)
			draw_line(y);
	}
	const int n = (int)(48000000 / FRAME_MHZ);   // (silence)
	for (int k = 0; k < n; k++)
		rbx_audio_put(0);
	vblank_capture();
	s_frame++;
}

static int af_state(void *ctx, int load)
{
	uint8_t cpu[M6502_STATE_SIZE];
	if (!load)
		memcpy(cpu, &s_cpu, M6502_STATE_SIZE);
	int64_t tg = s_target;
	uint64_t fr = s_frac;
	int32_t misc[12] = { (int32_t)s_frame, s_color, s_tprot, s_flip, s_screen_off, s_bank, s_red_on, s_abattle_count,
	                     s_coin_nmi, s_irq, (int32_t)s_rand, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "cram", s_cram, sizeof(s_cram));
	rc |= rbx_state_io(ctx, "lines", s_lines, sizeof(s_lines));
	rc |= rbx_state_io(ctx, "tg", &tg, sizeof(tg));
	rc |= rbx_state_io(ctx, "frac", &fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6502_STATE_SIZE);
		s_target = tg;
		s_frac = fr;
		s_frame = (uint32_t)misc[0];
		s_color = (uint8_t)misc[1];
		s_tprot = (uint8_t)misc[2];
		s_flip = (uint8_t)misc[3];
		s_screen_off = (uint8_t)misc[4];
		s_bank = (uint8_t)misc[5];
		s_red_on = (uint8_t)misc[6];
		s_abattle_count = (uint16_t)misc[7];
		s_coin_nmi = misc[8];
		s_irq = misc[9];
		s_rand = (uint32_t)misc[10];
	}
	return rc ? -1 : 0;
}

static int af_save(void *ctx) { return af_state(ctx, 0); }
static int af_load(void *ctx) { return af_state(ctx, 1); }
static int af_supported(const char *machine)
{
	return !strcmp(machine, "astrof") || !strcmp(machine, "abattle") || !strcmp(machine, "spfghmk2") ||
	       !strcmp(machine, "tomahawk");
}

const rbx_driver rbx_drv_astrof = { "DECO Astro Fighter", af_init, af_exit, af_reset, af_frame, af_save, af_load,
                                    af_supported, af_render, NULL };
