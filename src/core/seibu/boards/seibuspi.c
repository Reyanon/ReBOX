/*  Seibu SPI hardware: the SPI motherboard with its cartridges (Senkyu / Battle Balls, Viper Phase 1, E-Jan High
    School, Raiden Fighters, Raiden Fighters 2, Raiden Fighters Jet), the single boards SXX2E / SXX2F / SXX2G, the
    SYS386I (two OKI MSM6295) and the SYS386F (E-Jan Sakurasou: sprites only, YMZ280B, mahjong panel).
    An Intel 386DX (cpu/i386), the Seibu CRTC's four tile layers fed by DMA from the main RAM, the SEI252 / RISE10 /
    RISE11 sprites (encrypted ROMs, seibuspi_m), alpha blending; the sound: a Z80 with a YMF271 behind two IDT7201
    FIFOs (the SPI board's Z80 program and the YMF271's samples are uploaded by the 386: the program into the Z80's
    RAM, the samples into two E28F008SA flash chips through the YMF271's external port), the DS2404 RTC, a 93C46
    EEPROM on the later boards.
    Ported from MAME 0.289 src/mame/seibu/seibuspi.cpp, seibuspi.h, seibuspi_v.cpp (BSD-3-Clause, copyright-holders:
    Ville Linde, hap, Nicola Salmoria), src/mame/seibu/sei25x_rise1x_spr.cpp (BSD-3-Clause, copyright-holders:
    Olivier Galibert, Angelo Salese, David Haywood, Tomasz Slanina, Nicola Salmoria, Ville Linde, hap) and
    src/mame/seibu/seibu_crtc.cpp (BSD-3-Clause, copyright-holders: Angelo Salese).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved by sched.h (MAME's 12 kHz maximum quantum on the Z80 boards). The screen is drawn whole at
    vblank from a copy (no raster effects); the blending is done in RGB, so the driver writes the frontend's frame
    itself (rbx_video.out). MAME's idle-loop speedups (ENABLE_SPEEDUP_HACKS) are kept.
    Not done: the flip screen, the coin counters; the sound flash, the DS2404 SRAM and the RTC are not kept between
    runs (MAME's nvram: an SPI board writes its cartridge's samples to the flash at every start, as on MAME's first),
    the RTC starts at its reference date rather than at the host's time.
*/
#include "../../rbx.h"
#include "../../cpu/i386/i386.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ymf271.h"
#include "../../sound/okim6295.h"
#include "../../sound/ymz280b.h"
#include "../../machine/eepromser.h"
#include "../../machine/fifo7200.h"
#include "../../machine/intelfsh.h"
#include "../../machine/ds2404.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"
#include "../machine/sched.h"
#include "../machine/seibuspi_m.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VIS_W 320
#define VIS_H 240

// machine configurations
enum { M_SPI, M_EJANHS, M_RDFT2, M_SXX2E, M_SXX2F, M_SXX2G, M_SYS386I, M_SYS386F };
// the init_* functions
enum { I_SENKYU, I_SENKYUA, I_BATLBALL, I_VIPRP1, I_VIPRP1O, I_EJANHS, I_RDFT, I_RDFT2, I_RFJET, I_RDFT22KC,
       I_RFJET2KC, I_SYS386F };

static int s_m, s_init;
static int s_cart;           // seibuspi_state: the SPI motherboard (flash, two FIFOs, the Z80 program upload)
static int s_z80;            // seibuspi_z80_state: the Z80 + YMF271 sound
static int s_tilemap;        // seibuspi_tilemap_state
static int s_rise;           // rise_map (RISE10/11) or sei252_map
static int s_has_rtc, s_has_eeprom;
static uint32_t s_cpu_clock, s_z80_clock, s_snd_rate;
static int64_t s_frame_as, s_vblank_as;
static uint32_t s_fps_mhz;

static uint8_t *s_rom;               // maincpu (0x200000)
static uint8_t *s_zrom;              // audiocpu (0x40000; the SPI board's: written by the 386)
static uint32_t s_zrom_size, s_zrom_mask;   // (mask: size - 1 for a power of two, else 0)
static uint8_t *s_sound01;           // SPI: the cartridge's sound data at 0xa00000
static uint32_t s_sound01_size;
static const uint8_t *s_ymf_rom;     // single boards: the YMF271's ROM
static uint32_t s_ymf_rom_size;
static uint8_t *s_flash_data;        // SPI: the two E28F008SA (2 x 1 MB)
static char s_flash_nv[256];         // (its nvram file)
static int s_flash_dirty;
static int s_flash_quiet;            // (frames since the last flash write: saved 2 s after the updater ends, as MAME keeps it)
static void flash_nv_save(void);
static const uint8_t *s_flash1_region;
static uint32_t s_flash1_region_size;
static rbx_gfx s_tiles, s_chars, s_spr;
static uint32_t s_bg_fore_layer_position;
static uint8_t s_alpha_table[0x2000];
static uint32_t s_palette_size, s_palette_ram_size, s_sprite_ram_size;
static int s_port_inputs, s_port_exch, s_port_system, s_port_coin, s_port_jumpers, s_port_special, s_port_key[5];

// the speedup handlers (MAME's *_speedup_r): the dword read, the idle loop's pc
static uint32_t s_speedup_addr;
static const uint32_t *s_speedup_pcs;
static int s_speedup_n;

static z80 s_z;
static ymf271_t *s_ymf;
static okim6295_t s_oki[2];
static const uint8_t *s_oki_rom[2];
static uint32_t s_oki_size[2];
static ymz280b_t s_ymz;
static eepser_t s_eeprom;
static intelfsh_t s_flash[2];

typedef struct {
	uint32_t ram[0x40000 / 4];        // mainram (the I/O at 0x400-0x7ff apart)
	uint32_t palette_ram[0x4000 / 4];
	uint16_t sprite_ram[0x2000 / 2];
	uint32_t tilemap_ram[0x4000 / 4];
	uint32_t pal[8192];               // (the palette's colours, 0x00RRGGBB)
	uint16_t crtc[0x40];              // (the CRTC's register RAM, by word)
	uint32_t video_dma_length, video_dma_address;
	uint16_t layer_enable, layer_bank, reg_1a;
	uint8_t rf2_layer_bank, rowscroll_enable;
	uint16_t scrollram[6];
	uint32_t midl_layer_offset, fore_layer_offset, text_layer_offset;
	uint32_t fore_layer_d13, back_layer_d14, midl_layer_d14, fore_layer_d14;
	uint8_t input_select;             // (sys386f)
	uint8_t irq_line;                 // (the 386's INT, held until acknowledged)
	// the sound side
	uint8_t zram[0x2000];
	int32_t z80_lastbank;
	uint8_t sb_coin_latch;
	uint8_t z80_reset;                // (the SPI board's Z80 held in reset)
	uint8_t z80_irq;
	uint8_t oki_bank;
	uint32_t z80_prg_transfer_pos;
	fifo7200_t fifo[2];
	ds2404_t rtc;
	int64_t snd_as;                   // (the time of the next sound sample in the frame)
	int64_t cpu_total;                // (the 386's cycles before the running slice: the EEPROM's time)
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

typedef struct {
	uint32_t tilemap_ram[0x4000 / 4];
	uint16_t sprite_ram[0x2000 / 2];
	uint32_t pal[8192];
	uint16_t scroll[6], layer_enable;
	uint8_t rowscroll_enable;
	uint32_t midl_layer_offset, fore_layer_offset, text_layer_offset;
	uint32_t fore_layer_d13, back_layer_d14, midl_layer_d14, fore_layer_d14;
} snap_t;
static snap_t *s_snapp;   // (calloc at the init: ADR-040)
#define s_snap (*s_snapp)
static uint16_t *s_sprbmp;   // (m_sprite_bitmap)
static int32_t *s_mix;       // (the YMF271's 4 outputs of a frame)
static int s_mix_cap, s_mix_n;

static inline void combine32(uint32_t *p, uint32_t data, uint32_t mask) { *p = (*p & ~mask) | (data & mask); }
static inline void combine16(uint16_t *p, uint16_t data, uint16_t mask) { *p = (uint16_t)((*p & ~mask) | (data & mask)); }
static inline uint32_t pal5bit(uint32_t v) { v &= 0x1f; return (v << 3) | (v >> 2); }

// ---- the scheduler's time ----
static int64_t cpu_now(void) { return b.cpu_total + i386_done(); }

// ---- the sound streams: rendered up to now before a chip is touched (MAME's stream update) ----
static void snd_render(int n);
static void snd_sync(void)
{
	if (!s_snd_rate)
		return;
	const int64_t now = sched_now(), step = SCHED_AS_PER_SEC / s_snd_rate;
	int n = 0;
	while (b.snd_as <= now) {
		b.snd_as += step;
		n++;
	}
	if (n)
		snd_render(n);
}

static void snd_render(int n)
{
	if (s_ymf) {
		if (s_mix_n + n > s_mix_cap)
			n = s_mix_cap - s_mix_n;
		if (n > 0) {
			ymf271_render(s_ymf, s_mix + s_mix_n * 4, n);
			s_mix_n += n;
		}
	} else if (s_m == M_SYS386F) {
		int32_t l[256], r[256];
		while (n > 0) {
			const int k = n > 256 ? 256 : n;
			memset(l, 0, sizeof(l));
			memset(r, 0, sizeof(r));
			ymz280b_render(&s_ymz, l, r, k);
			for (int i = 0; i < k; i++)   // (ALL_OUTPUTS to the mono speaker)
				rbx_audio_put(l[i] + r[i]);
			n -= k;
		}
	} else if (s_m == M_SYS386I) {
		for (int i = 0; i < n; i++)
			rbx_audio_put(oki_sample(&s_oki[0]) + oki_sample(&s_oki[1]));
	}
}

// the YMF271's samples made on the rbx_aux worker (single boards: ymf271_set_async)
// (two workers: the even groups on rbx_aux, the odd ones on rbx_aux2 on a core of its own)
static void ymf_job(void) { ymf271_async_job(s_ymf, 0); }
static void ymf_job2(void) { ymf271_async_job(s_ymf, 1); }
static void ymf_kick(void *u) { (void)u; rbx_aux_kick(); rbx_aux2_kick(); }
static void ymf_sync(void *u) { (void)u; rbx_aux_sync(); rbx_aux2_sync(); }

// the YMF271's samples of the frame to the output (the SPI board's outputs 0 / 1 to the stereo speaker, the single
// boards' all four to the mono one)
static void ymf_flush(void)
{
	if (s_ymf->wk)
		ymf271_async_take(s_ymf, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++) {
		const int32_t *m = s_mix + i * 4;
		if (s_cart)
			rbx_audio_put_stereo(m[0] / 4, m[1] / 4);
		else
			rbx_audio_put((m[0] + m[1] + m[2] + m[3]) / 4);
	}
	s_mix_n = 0;
}

// ---- timers ----
enum { T_VBLANK = 1, T_YMF_A, T_YMF_B, T_RTC, T_FLASH0, T_FLASH1 };

static void ymf_timer(void *user, int which, int64_t clocks)
{
	(void)user;
	if (clocks < 0)
		sched_timer_off(T_YMF_A + which);
	else
		sched_timer_set(T_YMF_A + which, sched_now() + sched_ticks(clocks, s_ymf->master_clock), 0);
}

static void flash_delay(void *user, int seconds)
{
	const int n = (int)(intptr_t)user;
	sched_timer_set(T_FLASH0 + n, sched_now() + (int64_t)seconds * SCHED_AS_PER_SEC, 0);
}

static void ymf_irq(void *user, int state)
{
	(void)user;
	b.z80_irq = (uint8_t)(state ? 1 : 0);
}

// the YMF271's external memory: the SPI board's flash (spi_ymf271_map), the single boards' ROM
static uint8_t ymf_mem_r(void *user, uint32_t addr)
{
	(void)user;
	if (s_cart) {
		addr &= 0x1fffff;
		return intelfsh_read(&s_flash[addr >> 20], addr & 0xfffff);
	}
	if (!s_ymf_rom_size)
		return 0;
	if ((s_ymf_rom_size & (s_ymf_rom_size - 1)) == 0)
		return s_ymf_rom[addr & (s_ymf_rom_size - 1)];
	return addr < s_ymf_rom_size ? s_ymf_rom[addr] : 0;
}

static void ymf_sync(void *u);
// the flash about to change (a write, its timer): the workers first make the samples queued before it (they read it)
static void flash_change(void)
{
	if (s_ymf && s_ymf->wk)
		ymf_sync(NULL);
}
// the flash read straight by the YMF271 while both chips read their array (ymf_mem_r's reads: the same bytes), else
// through ymf_mem_r; made again after the flash changed (the workers idle then)
static void flash_rom_update(void)
{
	if (s_cart && s_ymf && s_flash_data)
		ymf271_set_rom(s_ymf, intelfsh_reads_data(&s_flash[0]) && intelfsh_reads_data(&s_flash[1]) ? s_flash_data : NULL,
		               0x200000);
}

static void ymf_mem_w(void *user, uint32_t addr, uint8_t data)
{
	(void)user;
	if (s_cart) {
		flash_change();
		addr &= 0x1fffff;
		intelfsh_write(&s_flash[addr >> 20], addr & 0xfffff, data);
		flash_rom_update();
		s_flash_dirty = 1;
		s_flash_quiet = 0;
	}
}

// ---- video (seibuspi_v.cpp) ----
static void set_layer_offsets(void)
{
	if (b.rowscroll_enable) {
		b.fore_layer_offset = 0x1000 / 4;
		b.midl_layer_offset = 0x2000 / 4;
		b.text_layer_offset = 0x3000 / 4;
	} else {
		b.fore_layer_offset = 0x1000 / 4 / 2;
		b.midl_layer_offset = 0x2000 / 4 / 2;
		b.text_layer_offset = 0x3000 / 4 / 2;
	}
	b.fore_layer_d13 = (uint32_t)b.layer_bank << 2 & 0x2000;
	b.back_layer_d14 = (uint32_t)b.rf2_layer_bank << 14 & 0x4000;
	b.midl_layer_d14 = (uint32_t)b.rf2_layer_bank << 13 & 0x4000;
	b.fore_layer_d14 = (uint32_t)b.rf2_layer_bank << 12 & 0x4000;
}

static void layer_bank_w(uint16_t data, uint16_t mask)
{
	// r000f000 0010100a
	combine16(&b.layer_bank, data, mask);
	b.rowscroll_enable = (b.layer_bank >> 15) & 1;
	set_layer_offsets();
}

static void rf2_layer_bank_w(uint8_t data)
{
	// 00000fmb
	b.rf2_layer_bank = data;
	set_layer_offsets();
}

// the CRTC (seibu_crtc_vregs) by word: its byte register is word * 2
static void crtc_w(int word, uint16_t data, uint16_t mask)
{
	const int reg = word * 2;
	if (reg == 0x14)   // decrypt_key_w (the tile keys: MAME logs them only)
		return;
	if (reg == 0x1a) {
		combine16(&b.reg_1a, data, mask);
		layer_bank_w(data, mask);
		return;
	}
	if (reg == 0x1c) {
		combine16(&b.layer_enable, data, mask);
		return;
	}
	if (reg >= 0x20 && reg < 0x2c) {
		combine16(&b.scrollram[(reg - 0x20) / 2], data, mask);
		return;
	}
	if (reg >= 0x2c && reg < 0x3c)   // layer_scroll_base_w (no callback on SPI)
		return;
	combine16(&b.crtc[word & 0x3f], data, mask);
}

static uint16_t crtc_r(int word)
{
	if (word * 2 == 0x1a)
		return b.reg_1a;
	return b.crtc[word & 0x3f];
}

static void tilemap_dma_start_w(void)
{
	const int dma_length_user = b.rowscroll_enable ? 0x4000 : 0x2800;
	(void)dma_length_user;
	uint32_t index = b.video_dma_address / 4;
#define RAMW(i) b.ram[(i) & 0xffff]
	/* back layer */
	for (int i = 0; i < 0x800 / 4; i++)
		b.tilemap_ram[i] = RAMW(index++);
	/* back layer row scroll */
	if (b.rowscroll_enable)
		for (int i = 0; i < 0x800 / 4; i++)
			b.tilemap_ram[0x800 / 4 + i] = RAMW(index++);
	/* fore layer */
	for (int i = 0; i < 0x800 / 4; i++)
		b.tilemap_ram[(i + b.fore_layer_offset) & 0xfff] = RAMW(index++);
	/* fore layer row scroll */
	if (b.rowscroll_enable)
		for (int i = 0; i < 0x800 / 4; i++)
			b.tilemap_ram[0x2800 / 4 + i] = RAMW(index++);
	/* middle layer */
	for (int i = 0; i < 0x800 / 4; i++)
		b.tilemap_ram[(i + b.midl_layer_offset) & 0xfff] = RAMW(index++);
	/* middle layer row scroll */
	if (b.rowscroll_enable)
		for (int i = 0; i < 0x800 / 4; i++)
			b.tilemap_ram[0x1800 / 4 + i] = RAMW(index++);
	/* text layer */
	for (int i = 0; i < 0x1000 / 4; i++)
		b.tilemap_ram[(i + b.text_layer_offset) & 0xfff] = RAMW(index++);
#undef RAMW
}

static void palette_dma_start_w(void)
{
	const uint32_t dma_length = (b.video_dma_length + 1) * 2;
	for (uint32_t i = 0; i < dma_length / 4 && i < s_palette_ram_size / 4; i++) {
		const uint32_t color = b.ram[(b.video_dma_address / 4 + i) & 0xffff];
		if (b.palette_ram[i] != color) {
			b.palette_ram[i] = color;
			b.pal[i * 2] = pal5bit(color >> 0) << 16 | pal5bit(color >> 5) << 8 | pal5bit(color >> 10);
			b.pal[i * 2 + 1] = pal5bit(color >> 16) << 16 | pal5bit(color >> 21) << 8 | pal5bit(color >> 26);
		}
	}
}

static void sprite_dma_start_w(void)
{
	for (uint32_t i = 0; i < s_sprite_ram_size / 4; i++) {
		const uint32_t v = b.ram[(b.video_dma_address / 4 + i) & 0xffff];
		b.sprite_ram[i << 1] = (uint16_t)(v & 0xffff);
		b.sprite_ram[(i << 1) | 1] = (uint16_t)(v >> 16);
	}
}

// ---- the EEPROM (eeprom_w) ----
static void eeprom_w(uint8_t data)
{
	const int64_t now = cpu_now();
	eepser_di_write(&s_eeprom, (data >> 7) & 1);
	eepser_clk_write(&s_eeprom, (data >> 6) & 1, now);
	eepser_cs_write(&s_eeprom, (data >> 5) & 1, now);
}

static uint32_t port(int p) { return p >= 0 ? rbx_port_read(p) : 0xffffffffu; }

// ejanhs_encode<N>
static uint32_t ejanhs_encode(int n)
{
	const uint32_t state = port(s_port_key[n]);
	for (int bit = 0; bit < 6; bit++)
		if (!((state >> bit) & 1))
			return (uint32_t)bit + 2;
	return 0;
}

static uint32_t read_system(void)
{
	if (s_has_eeprom && s_m != M_SYS386F)   // (sxx2f: the EEPROM's data out)
		rbx_port_custom(s_port_system, 0x40, eepser_do_read_93cxx(&s_eeprom, cpu_now()) ? 0x40 : 0);
	return port(s_port_system);
}

static uint32_t read_inputs(void)
{
	if (s_m == M_EJANHS) {
		const uint32_t v = ejanhs_encode(0) | ejanhs_encode(1) << 3 | ejanhs_encode(2) << 8 | ejanhs_encode(3) << 11;
		rbx_port_custom(s_port_inputs, 0x3f3f, v);
	}
	return port(s_port_inputs);
}

// ---- the 386's I/O (base_map, sei252_map / rise_map, spi_map, rdft2_map, sxx2e_map, sxx2f_map, sys386i_map) ----
static uint8_t sound_fifo_status_r(void)
{
	// d0: fifo full flag (z80)
	// d1: fifo empty flag (main)
	const uint8_t d1 = s_cart ? (uint8_t)(fifo7200_ef_r(&b.fifo[1]) << 1) : 0;
	return (uint8_t)(d1 | fifo7200_ff_r(&b.fifo[0]));
}

static void z80_hold(int held)
{
	if (held) {
		// (the reset line asserted: the CPU reset and suspended)
		const z80 cb = s_z;
		z80_init(&s_z);
		s_z.read_byte = cb.read_byte;
		s_z.write_byte = cb.write_byte;
		s_z.port_in = cb.port_in;
		s_z.port_out = cb.port_out;
		s_z.int_data = 0xd7;   // (audio_vector_r: IM0, RST 10h)
	}
	b.z80_reset = (uint8_t)(held ? 1 : 0);
}

static uint32_t io_r(uint32_t a, uint32_t mask)
{
	uint32_t v = b.ram[a >> 2];   // (the lanes without a handler: the RAM under the I/O)
	if (s_m == M_SYS386F) {
		switch (a) {
		case 0x010:
			if (mask & 0xff)
				v = (v & ~0xffu) | 0x01;   // spi_status_r
			break;
		case 0x400: return read_system();
		case 0x600: case 0x604:   // ymz read (umask32 0xff)
			if (mask & 0xff) {
				snd_sync();
				v = (v & ~0xffu) | ymz280b_read(&s_ymz, (a >> 2) & 1);
			}
			break;
		case 0x60c: {   // input_mux_r
			if (s_port_special >= 0)
				rbx_port_custom(s_port_special, 0x4000, eepser_do_read_93cxx(&s_eeprom, cpu_now()) ? 0x4000 : 0);
			uint32_t ret = port(s_port_special);
			if (b.input_select & 0x01) ret &= port(s_port_key[3]);
			if (b.input_select & 0x02) ret &= port(s_port_key[4]);
			if (b.input_select & 0x04) ret &= port(s_port_key[2]);
			if (b.input_select & 0x08) ret &= port(s_port_key[0]);
			if (b.input_select & 0x10) ret &= port(s_port_key[1]);
			uint32_t sw = 0;
			for (int i = 0; i < 5; i++)   // bitswap<5>(ret, 0, 1, 2, 3, 4)
				sw |= ((ret >> i) & 1) << (4 - i);
			return (ret & ~0x1fu) | sw;
		}
		default: break;
		}
		return v;
	}
	if (a >= 0x400 && a < 0x440) {   // the CRTC (16-bit)
		const int w = (int)(a - 0x400) / 2;
		v = 0;
		if (mask & 0xffff) v |= crtc_r(w);
		if (mask & 0xffff0000u) v |= (uint32_t)crtc_r(w + 1) << 16;
		return v;
	}
	switch (a) {
	case 0x600:
		if (mask & 0xff)
			v = (v & ~0xffu) | 0x01;   // spi_status_r: d0, video / DMA status?
		break;
	case 0x604: return read_inputs();
	case 0x608: return port(s_port_exch);
	case 0x60c: return read_system();
	case 0x680:
		if (!(mask & 0xff))
			break;
		if (s_cart)
			v = (v & ~0xffu) | (fifo7200_read(&b.fifo[1]) & 0xff);
		else if (s_z80) {   // sb_coin_r
			v = (v & ~0xffu) | b.sb_coin_latch;
			b.sb_coin_latch = 0;
		}
		break;
	case 0x684:
		if ((mask & 0xff) && s_z80)
			v = (v & ~0xffu) | sound_fifo_status_r();
		break;
	case 0x688:
		if (s_m == M_SXX2E || s_m == M_SXX2F || s_m == M_SXX2G)   // (noprw)
			v = 0;
		break;
	case 0x6dc:
		if (s_has_rtc) {
			if (mask & 0xff)
				v = (v & ~0xffu) | ds2404_data_r(&b.rtc);
			if (mask & 0xff00)
				v &= ~0xff00u;   // spi_ds2404_unknown_r: waits for d0-d2 cleared
		}
		break;
	default: break;
	}
	return v;
}

static void io_w(uint32_t a, uint32_t data, uint32_t mask)
{
	if (s_m == M_SYS386F) {
		switch (a) {
		case 0x400: b.input_select = (uint8_t)data; return;   // input_select_w
		case 0x404: if (mask & 0xff) eeprom_w((uint8_t)data); return;
		case 0x408: case 0x40c:   // ymz write (umask32 0xff)
			if (mask & 0xff) {
				snd_sync();
				ymz280b_write(&s_ymz, (a >> 2) & 1, (uint8_t)data);
			}
			return;
		case 0x484: palette_dma_start_w(); return;
		case 0x490: combine32(&b.video_dma_length, data, mask); return;
		case 0x494: combine32(&b.video_dma_address, data, mask); return;
		case 0x54c: return;   // RISE10/11 sprite decryption key
		case 0x560: if (mask & 0xffff0000u) sprite_dma_start_w(); return;
		default: break;
		}
		combine32(&b.ram[a >> 2], data, mask);
		return;
	}
	if (a >= 0x400 && a < 0x440) {
		const int w = (int)(a - 0x400) / 2;
		if (mask & 0xffff) crtc_w(w, (uint16_t)data, (uint16_t)mask);
		if (mask & 0xffff0000u) crtc_w(w + 1, (uint16_t)(data >> 16), (uint16_t)(mask >> 16));
		return;
	}
	switch (a) {
	case 0x480: tilemap_dma_start_w(); return;
	case 0x484: palette_dma_start_w(); return;
	case 0x490: combine32(&b.video_dma_length, data, mask); return;
	case 0x494: combine32(&b.video_dma_address, data, mask); return;
	case 0x498: return;   // ? dma address high bits? (always writes 0)
	default: break;
	}
	if (!s_rise) {
		if (a == 0x50c && (mask & 0xffff0000u)) { sprite_dma_start_w(); return; }
		if (a == 0x524 || a == 0x528 || a == 0x530 || a == 0x534 || a == 0x53c) return;   // SEI252 sprite decryption
	} else {
		if (a == 0x54c) return;   // RISE10/11 sprite decryption key
		if (a == 0x560 && (mask & 0xffff0000u)) { sprite_dma_start_w(); return; }
	}
	if (s_cart) {
		switch (a) {
		case 0x600: return;   // ?
		case 0x680:
			if (mask & 0xff)
				fifo7200_write(&b.fifo[0], (uint8_t)data);
			return;
		case 0x688:
			if ((mask & 0xff) && b.z80_prg_transfer_pos < s_zrom_size)   // z80_prg_transfer_w
				s_zrom[b.z80_prg_transfer_pos++] = (uint8_t)data;
			return;
		case 0x68c:
			if (mask & 0xff) {   // z80_enable_w: d0: reset z80
				b.z80_prg_transfer_pos = 0;
				z80_hold(!(data & 1));
			}
			if (mask & 0xff0000)
				rf2_layer_bank_w((uint8_t)(data >> 16));
			return;
		default: break;
		}
	} else if (s_z80) {
		switch (a) {
		case 0x680:
			if (mask & 0xff)
				fifo7200_write(&b.fifo[0], (uint8_t)data);
			return;
		case 0x688: return;   // ?
		case 0x68c:
			if (s_m == M_SXX2E)   // (nopw)
				return;
			if (mask & 0xff0000) {   // spi_layerbanks_eeprom_w
				rf2_layer_bank_w((uint8_t)(data >> 16));
				eeprom_w((uint8_t)(data >> 16));
			}
			return;
		case 0x690: return;   // ?
		default: break;
		}
	} else if (s_m == M_SYS386I && a == 0x68c) {
		if (mask & 0xff0000) {
			rf2_layer_bank_w((uint8_t)(data >> 16));
			eeprom_w((uint8_t)(data >> 16));
		}
		if (mask & 0xff000000u) {   // oki_bank_w
			b.oki_bank = (uint8_t)(data >> 24);
			s_oki[1].rom = s_oki_rom[1] + (((b.oki_bank >> 2) & 1) ? 0x40000 : 0);
		}
		return;
	}
	if (s_has_rtc) {
		switch (a) {
		case 0x6d0: if (mask & 0xff) ds2404_1w_reset_w(&b.rtc); return;
		case 0x6d4: if (mask & 0xff) ds2404_data_w(&b.rtc, (uint8_t)data); return;
		case 0x6d8: if (mask & 0xff) ds2404_clk_w(&b.rtc); return;
		default: break;
		}
	}
	combine32(&b.ram[a >> 2], data, mask);
}

// ---- the 386's program space ----
static uint32_t speedup_r(void)
{
	const uint32_t pc = i386_pc();
	for (int i = 0; i < s_speedup_n; i++)
		if (pc == s_speedup_pcs[i]) {
			i386_spin_until_interrupt(); // idle
			if (s_init == I_RFJET && pc == 0x0205b39) {   // rfjetus: hack to enter test mode
				const uint32_t r = b.ram[0x002894c / 4] & ~0x400u;
				return r | (((read_system() ^ 0xff) << 8) & 0x400);
			}
			break;
		}
	return b.ram[s_speedup_addr / 4];
}

static uint32_t cpu_r32(uint32_t a, uint32_t mask)
{
	if (a < 0x40000) {
		if (a < 0x800)
			return io_r(a, mask);
		if (s_speedup_n && a == s_speedup_addr)
			return speedup_r();
		return b.ram[a >> 2];
	}
	if (s_m == M_SYS386I && (a == 0x1200000 || a == 0x1200004)) {
		uint32_t v = 0;
		if (mask & 0xff) {
			snd_sync();
			v = oki_read(&s_oki[(a >> 2) & 1]);
		}
		return v;
	}
	return 0;   // (unmapped; the ROMs are read directly)
}

// the handler reads without side effects whose values only the CPU changes within its run (the RAM, but the speedup
// dword read from an idle loop's pc; the status and the sound FIFO's flags: the Z80 runs between the CPU's runs; the
// unmapped space), the writes without any effect (the cart boards' 0x600): the polling loops' probe (i386_idle_enable)
static int cpu_pure(uint32_t a, int write)
{
	if (write)
		return s_cart && (a & ~3u) == 0x600;
	if (a < 0x800)
		return s_m != M_SYS386F && ((a & ~3u) == 0x600 || ((a & ~3u) == 0x684 && s_z80));
	if (a < 0x40000) {
		if (s_speedup_n && (a & ~3u) == s_speedup_addr) {
			const uint32_t pc = i386_pc();
			for (int i = 0; i < s_speedup_n; i++)
				if (pc == s_speedup_pcs[i])
					return 0;
		}
		return 1;
	}
	return !(s_m == M_SYS386I && (a & ~7u) == 0x1200000);
}

static void cpu_w32(uint32_t a, uint32_t data, uint32_t mask)
{
	if (a < 0x40000) {
		if (a < 0x800)
			io_w(a, data, mask);
		else
			combine32(&b.ram[a >> 2], data, mask);
		return;
	}
	if (s_m == M_SYS386I && (a == 0x1200000 || a == 0x1200004)) {
		if (mask & 0xff) {
			snd_sync();
			oki_write(&s_oki[(a >> 2) & 1], (uint8_t)data);
		}
	}
}

// irq_callback: the vector; the vblank's HOLD_LINE released
static int irq_ack(void)
{
	b.irq_line = 0;
	i386_set_irq(0);
	return 0x20;
}

// ---- the Z80 (sxx2e_soundmap, spi_soundmap) ----
static uint8_t z80_soundfifo_status_r(void)
{
	// d0: fifo full flag (main)
	// d1: fifo empty flag (z80)
	const uint8_t d0 = s_cart ? (uint8_t)fifo7200_ff_r(&b.fifo[1]) : 0;
	return (uint8_t)(d0 | fifo7200_ef_r(&b.fifo[0]) << 1);
}

// ---- the sound Z80's idle passes ----
// The sound program's main loop polls (its channels' counters, the FIFO's status) until an interrupt or the main CPU
// gives it work. Within a slice nothing it reads changes but by its own hand (the interrupt line, the FIFO and the
// YMF271's timers change between slices). A pass of the loop (from its start back to it) is noted: the registers at
// each of its instructions' starts with the cycles and R counts to there, each byte it read before writing it (with
// its value; reads with effects or values that move within a slice, the FIFO's data and the YMF271, spoil it, as does
// an interrupt to take) and its writes in order (RAM only: its calls' return addresses). A pass is fixed by its
// registers at its start and those bytes. If it ended with the registers it began with (but the cycle count and R) and
// the bytes it read still as it read them, it is kept: the passes after it run the same, over and over, while those
// bytes stay and no interrupt is to be taken. The Z80 at the kept pass's start with its registers, or where a skip
// left it in the last slice, with those bytes and no interrupt, is then moved straight to the first instruction start
// at or past the slice's end: its registers there, the cycles and R counts added, the writes on the way made (the same
// as running there). The loop's start: the target of the last backward JP / JR. RBX_SPI_NOIDLE=1: off
#define ZI_RD 128
#define ZI_WR 16
#define ZI_IN 96   // (instructions a pass at most)
#define ZI_REGS (offsetof(z80, read_map) - offsetof(z80, pc))
#define ZI_ROFF (offsetof(z80, r) - offsetof(z80, pc))
typedef struct {
	int32_t bank;                    // (the ROM bank: the single boards' ROM reads are not noted)
	int n;                           // its instructions
	uint8_t regs[ZI_IN + 1][ZI_REGS];   // the registers at each one's start ([n]: at its end)
	uint16_t off[ZI_IN + 1];         // the cycles from its start to there ([n]: its length)
	uint8_t ro[ZI_IN + 1];           // the R count to there
	int nrd;
	uint16_t ra[ZI_RD];              // the bytes it read before writing them, their values
	uint8_t rv[ZI_RD];
	int nwr;
	uint8_t wi[ZI_WR];               // its writes in order: the instruction, the address, the value
	uint16_t wa[ZI_WR];
	uint8_t wv[ZI_WR];
} zi_pass_t;
static struct {
	int on;                // (0: not read yet, 1: on, 2: off)
	int head;              // (-1: none yet)
	int rec, bad;          // a pass being noted (into cur) from rec_head, from this cycle count and R
	int rec_head;
	unsigned long c0;
	uint8_t r0;
	zi_pass_t cur;
	int have;              // a kept pass (at keep_head)
	int keep_head;
	zi_pass_t keep;
	int at;                // (the Z80 left by a skip at this instruction of the kept pass; -1 none)
} s_zi;   // (spi_reset: nothing learnt)

static uint8_t z_read_raw(uint16_t a);

// a read while a pass is noted: the first read of a byte not written yet kept with its value
static void zi_note_read(uint16_t a, uint8_t v)
{
	if (a == 0x4008 || (a >= 0x6000 && a < 0x6010)) {
		s_zi.bad = 1;   // (a read with an effect, or one whose value moves within a slice: the YMF271's status)
		return;
	}
	if (!s_cart && (a < 0x2000 || a >= 0x8000))
		return;   // (a ROM: the same at the same bank. The SPI board's is RAM the 386 writes)
	zi_pass_t *p = &s_zi.cur;
	for (int i = 0; i < p->nwr; i++)
		if (p->wa[i] == a)
			return;
	for (int i = 0; i < p->nrd; i++)
		if (p->ra[i] == a) {
			if (p->rv[i] != v)
				s_zi.bad = 1;   // (changed between slices while noted)
			return;
		}
	if (p->nrd >= ZI_RD) {
		s_zi.bad = 1;
		return;
	}
	p->ra[p->nrd] = a;
	p->rv[p->nrd++] = v;
}

// a byte of code without side effects (-1: not memory)
static int z_peek(uint16_t a)
{
	if (a < 0x2000)
		return s_zrom[a];
	if (a < 0x4000)
		return b.zram[a - 0x2000];
	if (a >= 0x8000) {
		const uint32_t o = (uint32_t)(b.z80_lastbank & 7) * 0x8000 + (a - 0x8000);
		return s_zrom[s_zrom_mask ? o & s_zrom_mask : o % s_zrom_size];
	}
	return -1;
}

// the Z80's direct reads (z80 read_map): its ROM (on the SPI board RAM the 386 fills), RAM and the bank at 0x8000, as
// z_read_raw reads them; the bank's pages made again when it changes. Not while a pass is noted (its reads are noted)
static const uint8_t *s_zmap[256];
static int s_zmap_bank = -1;
static void zmap_bank(void)
{
	s_zmap_bank = b.z80_lastbank;
	for (int p = 0; p < 0x20; p++) {
		s_zmap[p] = s_zrom + p * 256;
		s_zmap[0x20 + p] = b.zram + p * 256;
	}
	for (int p = 0x40; p < 0x80; p++)
		s_zmap[p] = NULL;
	for (int p = 0x80; p < 0x100; p++) {
		const uint32_t o = (uint32_t)(b.z80_lastbank & 7) * 0x8000 + (uint32_t)(p - 0x80) * 256;
		const uint32_t r = s_zrom_mask ? o & s_zrom_mask : o % s_zrom_size;
		// (a page that would wrap inside the ROM: read through z_read_raw)
		s_zmap[p] = (s_zrom_mask || r + 256 <= s_zrom_size) ? s_zrom + r : NULL;
	}
}

static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t v = z_read_raw(a);
	if (s_zi.rec)
		zi_note_read(a, v);
	return v;
}

static uint8_t z_read_raw(uint16_t a)
{
	if (a < 0x2000)
		return s_zrom[a];
	if (a < 0x4000)
		return b.zram[a - 0x2000];
	if (a >= 0x8000) {
		const uint32_t o = (uint32_t)(b.z80_lastbank & 7) * 0x8000 + (a - 0x8000);
		return s_zrom[s_zrom_mask ? o & s_zrom_mask : o % s_zrom_size];   // (no division for a power of two)
	}
	if (a >= 0x6000 && a < 0x6010) {
		snd_sync();
		return ymf271_read(s_ymf, a & 0xf);
	}
	switch (a) {
	case 0x4008: return (uint8_t)fifo7200_read(&b.fifo[0]);
	case 0x4009: return z80_soundfifo_status_r();
	case 0x400a: return s_cart ? (uint8_t)port(s_port_jumpers) : 0;
	case 0x4013: return (uint8_t)port(s_port_coin);
	default: return 0;
	}
}

static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_zi.rec) {   // (a pass noted: its RAM writes, nothing else)
		zi_pass_t *p = &s_zi.cur;
		if (a >= 0x2000 && a < 0x4000 && p->nwr < ZI_WR && p->n > 0) {
			p->wi[p->nwr] = (uint8_t)(p->n - 1);
			p->wa[p->nwr] = a;
			p->wv[p->nwr++] = v;
		} else
			s_zi.bad = 1;
	}
	if (a >= 0x2000 && a < 0x4000) {
		b.zram[a - 0x2000] = v;
		return;
	}
	if (a >= 0x6000 && a < 0x6010) {
		snd_sync();
		ymf271_write(s_ymf, a & 0xf, v);
		return;
	}
	switch (a) {
	case 0x4004:   // spi_coin_w (the coin counters apart): the coin latch of the single boards
		b.sb_coin_latch = v ? (uint8_t)(0xa0 | v) : 0;
		break;
	case 0x4008:
		if (s_cart)
			fifo7200_write(&b.fifo[1], v);
		break;
	case 0x401b:   // z80_bank_w: d0-d2: bank @ 8000, d3: watchdog?
		b.z80_lastbank = v & 7;
		break;
	default: break;
	}
}

static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- the scheduler's devices ----
static int cpu_run(int n)
{
	const int done = i386_run(n);
	b.cpu_total += done;
	return done;
}
static int64_t cpu_done(void) { return i386_done(); }
static void cpu_abort(void) { i386_abort(); }

static unsigned long s_zrun0;
static int s_zabort;

// the registers (from pc on, as noted) the same but R
static int zi_same_regs(const uint8_t *x, const uint8_t *y)
{
	return !memcmp(x, y, ZI_ROFF) && !memcmp(x + ZI_ROFF + 1, y + ZI_ROFF + 1, ZI_REGS - ZI_ROFF - 1);
}

// the kept pass's inputs as it read them, no interrupt to take
static int zi_inputs_same(void)
{
	const zi_pass_t *p = &s_zi.keep;
	if (b.z80_irq || b.z80_lastbank != p->bank)
		return 0;
	if (b.z80_lastbank != s_zmap_bank)
		zmap_bank();
	for (int i = 0; i < p->nrd; i++) {   // (through the direct map where z_read_raw reads memory: the same bytes)
		const uint16_t a = p->ra[i];
		const uint8_t *pg = s_zmap[a >> 8];
		if ((pg ? pg[a & 0xff] : z_read_raw(a)) != p->rv[i])
			return 0;
	}
	return 1;
}

// the Z80 at instruction j of the kept pass (its registers, the inputs the same) moved to the first instruction start
// at or past the end as the passes would run there
static void zi_skip(int j, unsigned long end)
{
	const zi_pass_t *p = &s_zi.keep;
	const unsigned long dc = p->off[p->n], base = s_z.cyc - p->off[j], rel = end - base;
	unsigned long m = rel / dc;
	const unsigned long rest = rel - m * dc;
	int i = 0;
	while (p->off[i] < rest)
		i++;
	if (i == p->n) {   // (the next pass's start)
		m++;
		i = 0;
	}
	// the writes on the way: the rest of this pass's, whole passes', the last one's to instruction i
	for (int k = 0; k < p->nwr; k++)
		if (p->wi[k] >= j && (m || p->wi[k] < i))
			b.zram[p->wa[k] - 0x2000] = p->wv[k];
	if (m >= 2)
		for (int k = 0; k < p->nwr; k++)
			b.zram[p->wa[k] - 0x2000] = p->wv[k];
	if (m)
		for (int k = 0; k < p->nwr; k++)
			if (p->wi[k] < i)
				b.zram[p->wa[k] - 0x2000] = p->wv[k];
	const unsigned r0 = (unsigned)(s_z.r - p->ro[j]), r7 = s_z.r & 0x80;   // (R at this pass's start)
	memcpy(&s_z.pc, p->regs[i], ZI_REGS);
	s_z.r = (uint8_t)(r7 | ((r0 + m * p->ro[p->n] + p->ro[i]) & 0x7f));
	s_z.cyc = base + m * dc + p->off[i];
	s_zi.at = i;
}

// at the loop's start: a kept pass to skip from here, a noted pass ended here (kept if idle), or one noted from here
static void zi_head(unsigned long end)
{
	zi_pass_t *p = &s_zi.cur;
	if (s_zi.rec && s_z.pc != s_zi.rec_head)
		return;   // (a pass noted from elsewhere goes on)
	if (s_zi.rec) {   // (back at the start of the pass noted: kept if it ended as it began)
		s_zi.rec = 0;
		const int n = p->n;
		memcpy(p->regs[n], &s_z.pc, ZI_REGS);
		p->off[n] = (uint16_t)(s_z.cyc - s_zi.c0);
		p->ro[n] = (uint8_t)((s_z.r - s_zi.r0) & 0x7f);
		int same = !s_zi.bad && n > 0 && zi_same_regs(p->regs[n], p->regs[0]) && b.z80_lastbank == p->bank;
		// (the bytes it read still as it read them: a pass noted over slices saw no change of its inputs; it ran as
		// any pass beginning with these registers and bytes runs)
		for (int i = 0; same && i < p->nrd; i++)
			same = z_read_raw(p->ra[i]) == p->rv[i];
		if (same) {
			s_zi.keep = *p;
			s_zi.keep_head = s_z.pc;
			s_zi.have = 1;
		}
	}
	if (b.z80_irq || s_z.halted)   // (an interrupt to take: not idle)
		return;
	if (s_zi.have && s_zi.keep_head == s_z.pc) {
		uint8_t now[ZI_REGS];
		memcpy(now, &s_z.pc, ZI_REGS);
		if (zi_same_regs(now, s_zi.keep.regs[0]) && zi_inputs_same()) {
			zi_skip(0, end);
			return;
		}
	}
	if (s_z.pc != s_zi.head)
		return;   // (passes are noted from the loop's start found last)
	s_zi.rec = 1;   // (a pass noted from here)
	s_zi.rec_head = s_z.pc;
	s_zi.bad = 0;
	s_zi.c0 = s_z.cyc;
	s_zi.r0 = s_z.r;
	p->bank = b.z80_lastbank;
	p->n = 0;
	p->nrd = 0;
	p->nwr = 0;
}

static int z_run(int n)
{
	if (b.z80_reset)
		return n;
	if (!s_zi.on)
		s_zi.on = getenv("RBX_SPI_NOIDLE") ? 2 : 1;
	s_zrun0 = s_z.cyc;
	s_zabort = 0;
	const unsigned long end = s_zrun0 + (unsigned long)n;
	// (left inside the kept pass by the last slice's skip: on from there while its inputs stay)
	if (s_zi.at >= 0 && n > 0 && zi_inputs_same()) {
		uint8_t now[ZI_REGS];
		memcpy(now, &s_z.pc, ZI_REGS);
		if (zi_same_regs(now, s_zi.keep.regs[s_zi.at]))
			zi_skip(s_zi.at, end);
		else
			s_zi.at = -1;
	} else
		s_zi.at = -1;
	while ((long)(s_z.cyc - end) < 0 && !s_zabort) {
		s_z.int_pending = b.z80_irq;
		if (s_z.halted && z80_halt_skip(&s_z, end, b.z80_irq))
			break;
		if ((s_z.pc == s_zi.head || (s_zi.have && s_z.pc == s_zi.keep_head)) && s_zi.on == 1) {
			zi_head(end);
			if ((long)(s_z.cyc - end) >= 0)
				break;   // (skipped to the end)
		}
		const uint16_t pc0 = s_z.pc;
		if (s_zi.rec) {   // (an interrupt to take, code reading or setting R, or run from the devices, or a long pass:
			              // not noted)
			const int op = z_peek(pc0);
			zi_pass_t *p = &s_zi.cur;
			if (b.z80_irq || s_z.halted || op < 0 || p->n >= ZI_IN ||
			    (op == 0xed && ((z_peek((uint16_t)(pc0 + 1)) | 0x10) == 0x5f))) {
				s_zi.bad = 1;
				s_zi.rec = 0;
			} else {
				memcpy(p->regs[p->n], &s_z.pc, ZI_REGS);
				p->off[p->n] = (uint16_t)(s_z.cyc - s_zi.c0);
				p->ro[p->n] = (uint8_t)((s_z.r - s_zi.r0) & 0x7f);
				p->n++;
			}
		}
		if (s_zi.rec)
			s_z.read_map = NULL;
		else {
			if (b.z80_lastbank != s_zmap_bank)
				zmap_bank();
			s_z.read_map = s_zmap;
		}
		z80_step(&s_z);
		if (s_z.pc < pc0 && !s_zi.rec && s_zi.on == 1) {   // (a backward JP nn / JR e: a loop's start)
			const int op = z_peek(pc0);
			if (op == 0xc3 || op == 0x18)
				s_zi.head = s_z.pc;
		}
	}
	return (int)(s_z.cyc - s_zrun0);
}
static int64_t z_done(void) { return b.z80_reset ? 0 : (int64_t)(s_z.cyc - s_zrun0); }
static void z_abort(void) { s_zabort = 1; }

// ---- the screen ----
static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.tilemap_ram, b.tilemap_ram, sizeof(s_snap.tilemap_ram));
	memcpy(s_snap.sprite_ram, b.sprite_ram, sizeof(s_snap.sprite_ram));
	memcpy(s_snap.pal, b.pal, sizeof(s_snap.pal));
	memcpy(s_snap.scroll, b.scrollram, sizeof(s_snap.scroll));
	s_snap.layer_enable = b.layer_enable;
	s_snap.rowscroll_enable = b.rowscroll_enable;
	s_snap.midl_layer_offset = b.midl_layer_offset;
	s_snap.fore_layer_offset = b.fore_layer_offset;
	s_snap.text_layer_offset = b.text_layer_offset;
	s_snap.fore_layer_d13 = b.fore_layer_d13;
	s_snap.back_layer_d14 = b.back_layer_d14;
	s_snap.midl_layer_d14 = b.midl_layer_d14;
	s_snap.fore_layer_d14 = b.fore_layer_d14;
	rbx_video_frame_done();
}

static void fire(int id, int32_t param)
{
	(void)param;
	switch (id) {
	case T_VBLANK:   // the screen, the 386's IRQ (interrupt: HOLD_LINE)
		capture();
		b.irq_line = 1;
		i386_set_irq(1);
		sched_timer_set(T_VBLANK, sched_now() + s_frame_as, 0);
		break;
	case T_YMF_A: case T_YMF_B:
		snd_sync();
		ymf271_timer_expired(s_ymf, id - T_YMF_A);
		break;
	case T_RTC:
		ds2404_tick(&b.rtc);
		sched_timer_set(T_RTC, sched_now() + SCHED_AS_PER_SEC / 256, 0);
		break;
	case T_FLASH0: case T_FLASH1:
		flash_change();
		intelfsh_delay_tick(&s_flash[id - T_FLASH0]);
		flash_rom_update();
		break;
	default: break;
	}
}

static void spi_frame(void)
{
	if (s_cart && s_flash_dirty && ++s_flash_quiet == 120)
		flash_nv_save();   // (the updater done: kept at once, not only at exit)
	rbx_audio.frame_mhz = s_fps_mhz;
	sched_run(T_VBLANK, fire);
	// the frame's sound to the vblank, then everything a frame back
	const int64_t end = sched_now();
	if (s_snd_rate) {
		const int64_t step = SCHED_AS_PER_SEC / s_snd_rate;
		int n = 0;
		while (b.snd_as <= end) {
			b.snd_as += step;
			n++;
		}
		if (n)
			snd_render(n);
		b.snd_as -= s_frame_as;
	}
	if (s_ymf)
		ymf_flush();
	sched_rebase(s_frame_as);   // (the 386's cycle count, the EEPROM's time, runs on)
}

// ---- the drawing (screen_update_spi, screen_update_sys386f) ----
// The frame is drawn in bands of lines (render_help: the threads helping the render thread take bands too); a band
// draws its own lines of the sprite bitmap (each sprite clipped to the band) and composes them, so every pixel gets
// the same writes in the same order as the whole-screen drawing. The layers go a tile row at a time.
#define BANDS 8
static int s_565;                 // (out_color is RGB565 by its values: the line converted inline)
static uint8_t s_sprpri[VIS_H];   // (the sprites of a line: bit n = a pixel of priority n)
static int16_t s_sprx0[VIS_H], s_sprx1[VIS_H];   // (the columns [x0, x1) a line's sprite tiles cover)
static uint8_t s_alpha32[0x2000 / 32];   // (any blended colour in a group of 32)
static uint16_t *s_tile_tr, *s_char_tr;  // (per element: bit n = row n all the transparent pen; calloc at the init)

// an element's rows that are all the transparent pen (bit n: row n), for a set of 16 or 8 pixels wide
static uint16_t *rows_transparent(const rbx_gfx *g, uint8_t tp)
{
	uint16_t *m = (uint16_t *)calloc(g->count ? g->count : 1, sizeof(uint16_t));
	if (!m)
		return NULL;
	const size_t w = (size_t)g->width, h = (size_t)g->height;
	for (uint32_t e = 0; e < g->count; e++) {
		const uint8_t *p = g->pix + (size_t)e * w * h;
		uint16_t bits = 0;
		for (size_t r = 0; r < h; r++, p += w) {
			size_t i = 0;
			while (i < w && p[i] == tp)
				i++;
			if (i == w)
				bits |= (uint16_t)(1u << r);
		}
		m[e] = bits;
	}
	return m;
}

static inline uint32_t alpha_blend_r32(uint32_t d, uint32_t s, uint32_t level)
{
	return ((((s & 0x0000ff) * level + (d & 0x0000ff) * (256 - level)) >> 8)) |
	       ((((s & 0x00ff00) * level + (d & 0x00ff00) * (256 - level)) >> 8) & 0x00ff00) |
	       ((((s & 0xff0000) * level + (d & 0xff0000) * (256 - level)) >> 8) & 0xff0000);
}

static inline void blend_pixel(uint32_t *dest, uint16_t pen)
{
	const uint32_t c = s_snap.pal[pen & 0x1fff];
	*dest = s_alpha_table[pen & 0x1fff] ? alpha_blend_r32(*dest, c, 0x7f) : c;
}

// sei25x_rise1x_device::draw_sprites (raw: tilemap boards; to RGB: sys386f), the lines [y0, y1) only
static void draw_sprites(uint32_t *rgb, int y0, int y1)
{
	const uint16_t *spriteram = s_snap.sprite_ram;
	const int size = (int)(s_sprite_ram_size / 2);
	const int raw = rgb == NULL;
	const uint32_t elements = s_spr.count;
	if (!elements)
		return;
	for (int i = size - 4; i != -4; i -= 4) {
		uint32_t code = spriteram[i + 1];
		if (rbx_wrap(code, elements) == 0)
			continue;
		const int flipy = (spriteram[i + 0] >> 15) & 1;
		const int sizey = ((spriteram[i + 0] >> 12) & 7) + 1;
		const int flipx = (spriteram[i + 0] >> 11) & 1;
		const int sizex = ((spriteram[i + 0] >> 8) & 7) + 1;
		const uint32_t pri = (spriteram[i + 0] >> 6) & 3;
		uint32_t color = spriteram[i + 0] & 0x3f;
		const int ext = (spriteram[i + 2] >> 12) & 1;
		int x = spriteram[i + 2] & 0x1ff;
		int y = spriteram[i + 3] & 0x1ff;
		if (x >= 0x180) x -= 0x200;
		if (y >= 0x180) y -= 0x200;
		if (y >= y1 || y + 16 * sizey <= y0)
			continue;
		if (raw)
			color = (color << 6) | (pri << 14);   // (pix_raw_shift 6, pri_raw_shift 14)
		if (s_tilemap)   // gfxbank_callback
			code |= ((elements > 0x10000) && ext) ? 0x10000 : 0;
		for (int ax = 0; ax < sizex; ax++) {
			const int sx = flipx ? (x + 16 * (sizex - ax - 1)) : (x + 16 * ax);
			if (sx <= -16 || sx >= VIS_W) {
				code += (uint32_t)sizey;
				continue;
			}
			const int px0 = sx < 0 ? -sx : 0, px1 = sx > VIS_W - 16 ? VIS_W - sx : 16;
			for (int ay = 0; ay < sizey; ay++) {
				const int sy = flipy ? (y + 16 * (sizey - ay - 1)) : (y + 16 * ay);
				const uint32_t c = code++;
				if (sy <= -16 || sy >= VIS_H || sy >= y1 || sy + 16 <= y0)
					continue;
				const uint8_t *src = s_spr.pix + (size_t)rbx_wrap(c, elements) * 256;
				const int py0 = sy < y0 ? y0 - sy : 0, py1 = sy + 16 > y1 ? y1 - sy : 16;
				for (int py = py0; py < py1; py++) {
					const int dy = sy + py;
					const uint8_t *row = src + (flipy ? 15 - py : py) * 16;
					if (raw) {
						uint16_t *d = s_sprbmp + dy * VIS_W + sx;
						int any = 0;
						if (flipx) {
							for (int px = px0; px < px1; px++) {
								const uint8_t p = row[15 - px];
								if (p != 63) {   // (transpen 63)
									d[px] = (uint16_t)(color + p);
									any = 1;
								}
							}
						} else {
							for (int px = px0; px < px1; px++) {
								const uint8_t p = row[px];
								if (p != 63) {
									d[px] = (uint16_t)(color + p);
									any = 1;
								}
							}
						}
						if (any) {
							s_sprpri[dy] |= (uint8_t)(1u << pri);
							if (s_sprx0[dy] > sx + px0)
								s_sprx0[dy] = (int16_t)(sx + px0);
							if (s_sprx1[dy] < sx + px1)
								s_sprx1[dy] = (int16_t)(sx + px1);
						}
					} else {
						uint32_t *d = rgb + dy * VIS_W + sx;
						const uint32_t *pal = s_snap.pal + (((color % 32) * 256) & 0x1fff);
						for (int px = px0; px < px1; px++) {
							const uint8_t p = row[flipx ? 15 - px : px];
							if (p != 255)   // (transpen 255; the colour % 32 colours of 256)
								d[px] = pal[p];
						}
					}
				}
			}
		}
	}
}

// a tile row's n pixels over dest: colours cb + pen (pen tp transparent unless opaque), blended by the alpha table
static inline void tile_span(uint32_t *dest, const uint8_t *src, int n, uint32_t cb, int tp, int opaque, int any_alpha)
{
	const uint32_t *pal = s_snap.pal + cb;
	if (!any_alpha) {
		if (opaque)
			for (int i = 0; i < n; i++)
				dest[i] = pal[src[i]];
		else
			for (int i = 0; i < n; i++) {
				const uint8_t p = src[i];
				if (p != tp)
					dest[i] = pal[p];
			}
		return;
	}
	const uint8_t *at = s_alpha_table + cb;
	for (int i = 0; i < n; i++) {
		const uint8_t p = src[i];
		if (!opaque && p == tp)
			continue;
		dest[i] = at[p] ? alpha_blend_r32(dest[i], pal[p], 0x7f) : pal[p];
	}
}

// combine_tilemap on one line, a tile row at a time
static void combine_line(uint32_t *dest, int y, int layer, int sx, int sy, int opaque, const int16_t *rowscroll)
{
	const uint32_t *ram = s_snap.tilemap_ram;
	if (layer == 3) {   // text: 8x8, 64x32, scan rows (not scrolled)
		const int my = (y + sy) & 255;
		const uint32_t rowi = (uint32_t)(my >> 3) * 64;
		const size_t roff = (size_t)(my & 7) * 8;
		int mx = sx & 511;
		for (int x = 0; x < VIS_W;) {
			const int fx = mx & 7;
			int n = 8 - fx;
			if (n > VIS_W - x)
				n = VIS_W - x;
			const uint32_t tile_index = rowi + (uint32_t)(mx >> 3);
			const uint32_t offs = tile_index / 2 + s_snap.text_layer_offset;
			const int t = (int)(ram[offs & 0xfff] >> ((tile_index & 1) ? 16 : 0)) & 0xffff;
			const uint32_t code = rbx_wrap((uint32_t)t & 0xfff, s_chars.count);
			const uint32_t cb = 5632 + (uint32_t)((t >> 12) & 0xf) * 32;
			if (!opaque && ((s_char_tr[code] >> (my & 7)) & 1)) {   // (a row of pen 31 only)
				x += n;
				mx = (mx + n) & 511;
				continue;
			}
			tile_span(dest + x, s_chars.pix + (size_t)code * 64 + roff + (size_t)fx, n, cb, 31, opaque, s_alpha32[cb >> 5]);
			x += n;
			mx = (mx + n) & 511;
		}
		return;
	}
	int rx = sx;
	if (rowscroll)
		rx += rowscroll[(y + 19) & 511]; // adder value probably not hardcoded but came from CRTC
	const int my = (y + sy) & 511;
	const size_t roff = (size_t)(my & 15) * 16;
	uint32_t loff, cadd, codeor;
	switch (layer) {
	case 0: loff = 0; cadd = 0; codeor = s_snap.back_layer_d14; break;   // back
	case 1: loff = s_snap.midl_layer_offset; cadd = 16; codeor = 0x2000 | s_snap.midl_layer_d14; break;   // middle
	default:   // fore
		loff = s_snap.fore_layer_offset;
		cadd = 8;
		codeor = s_bg_fore_layer_position | s_snap.fore_layer_d13 | s_snap.fore_layer_d14;
		break;
	}
	int mx = rx & 511;
	for (int x = 0; x < VIS_W;) {
		const int fx = mx & 15;
		int n = 16 - fx;
		if (n > VIS_W - x)
			n = VIS_W - x;
		const uint32_t tile_index = (uint32_t)(mx >> 4) * 32 + (uint32_t)(my >> 4);   // 16x16, 32x32, scan cols
		const uint32_t offs = tile_index / 2 + loff;
		const int t = (int)(ram[offs & 0xfff] >> ((tile_index & 1) ? 16 : 0)) & 0xffff;
		const uint32_t code = rbx_wrap(((uint32_t)t & 0x1fff) | codeor, s_tiles.count);
		const uint32_t cb = 4096 + (((uint32_t)(t >> 13) & 7) + cadd) * 64;
		if (!opaque && ((s_tile_tr[code] >> (my & 15)) & 1)) {   // (a row of pen 63 only)
			x += n;
			mx = (mx + n) & 511;
			continue;
		}
		tile_span(dest + x, s_tiles.pix + (size_t)code * 256 + roff + (size_t)fx, n, cb, 63, opaque,
		          s_alpha32[cb >> 5] | s_alpha32[(cb >> 5) + 1]);
		x += n;
		mx = (mx + n) & 511;
	}
}

static void blend_sprite_line(uint32_t *dest, int y, int pri)
{
	if (s_snap.layer_enable & 0x10)
		return;
	if (!((s_sprpri[y] >> pri) & 1))   // (no sprite pixel of that priority on the line)
		return;
	const uint16_t pr = (uint16_t)(pri << 14);
	const uint16_t *src = s_sprbmp + y * VIS_W;
	for (int x = s_sprx0[y]; x < s_sprx1[y]; x++) {   // (within the line's sprite tiles' columns)
		const uint16_t pen = src[x];
		if ((pen & 0xc000) == pr && pen != 0xffff)
			blend_pixel(&dest[x], (uint16_t)(pen & ~0xc000));
	}
}

static void out_line(int y, const uint32_t *d)
{
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

static void spi_band(int band)
{
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	if (!s_tilemap) {   // screen_update_sys386f
		uint32_t *rgb = (uint32_t *)s_sprbmp;   // (VIS_W * VIS_H * 2 words: room for an RGB frame)
		memset(rgb + y0 * VIS_W, 0, (size_t)VIS_W * (size_t)(y1 - y0) * 4);
		draw_sprites(rgb, y0, y1);
		for (int y = y0; y < y1; y++)
			out_line(y, rgb + y * VIS_W);
		return;
	}
	uint32_t d[VIS_W] = { 0 };
	const uint16_t le = s_snap.layer_enable;
	const int16_t *back_rowscroll = NULL, *midl_rowscroll = NULL, *fore_rowscroll = NULL;
	if (s_snap.rowscroll_enable) {
		back_rowscroll = (const int16_t *)&s_snap.tilemap_ram[0x200];
		midl_rowscroll = (const int16_t *)&s_snap.tilemap_ram[0x600];
		fore_rowscroll = (const int16_t *)&s_snap.tilemap_ram[0xa00];
	}
	if (!(le & 0x10)) {
		for (int i = y0 * VIS_W; i < y1 * VIS_W; i++)
			s_sprbmp[i] = 0xffff;
		for (int y = y0; y < y1; y++) {
			s_sprpri[y] = 0;
			s_sprx0[y] = VIS_W;
			s_sprx1[y] = 0;
		}
		draw_sprites(NULL, y0, y1);
	}
	const int16_t *sc = (const int16_t *)s_snap.scroll;
	for (int y = y0; y < y1; y++) {
		// (the back layer's opaque line: its colours, 0x1000-0x11ff, are never blended (spi_init), so it doesn't
		// read what the line held before)
		if (le & 1)
			memset(d, 0, sizeof(d));
		else
			combine_line(d, y, 0, sc[0], sc[1], 1, back_rowscroll);
		blend_sprite_line(d, y, 0);
		// if fore layer is enabled, draw priority 0 sprites behind back layer
		if ((le & 0x15) == 0)
			combine_line(d, y, 0, sc[0], sc[1], 0, back_rowscroll);
		// if fore layer is enabled, draw priority 1 sprites behind middle layer
		if (~le & 4)
			blend_sprite_line(d, y, 1);
		if (~le & 2)
			combine_line(d, y, 1, sc[2], sc[3], 0, midl_rowscroll);
		// if fore layer is disabled, draw priority 1 sprites above middle layer
		if (le & 4)
			blend_sprite_line(d, y, 1);
		blend_sprite_line(d, y, 2);
		if (~le & 4)
			combine_line(d, y, 2, sc[4], sc[5], 0, fore_rowscroll);
		blend_sprite_line(d, y, 3);
		if (~le & 8)
			combine_line(d, y, 3, 0, 0, 0, NULL);
		out_line(y, d);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, spi_band };
static int spi_help(int helper) { return decoband_help(&s_bands, helper); }

static void spi_render(void)
{
	if (!rbx_video.out)
		return;
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f && rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	decoband_run(&s_bands);
	rbx_video.out_done = 1;
}

// ---- machine_reset ----
static void spi_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(board_t, state_end));
	s_zi.head = -1;   // (the sound Z80's idle passes: nothing learnt)
	s_zi.rec = 0;
	s_zi.have = 0;
	s_zi.at = -1;
	s_zi.on = 0;
	set_layer_offsets();
	if (s_has_rtc)
		ds2404_init(&b.rtc);
	fifo7200_init(&b.fifo[0], 0x200);
	fifo7200_init(&b.fifo[1], 0x200);
	if (s_cart) {
		memset(s_zrom, 0, s_zrom_size);   // (ROMREGION_ERASE00: the Z80's program RAM)
		intelfsh_reset(&s_flash[0]);
		intelfsh_reset(&s_flash[1]);
		flash_rom_update();
		// Hack: fix the magic ID byte so users can't "brick" the machine
		s_flash_data[0] = s_flash1_region[0];
	}
	i386_reset();
	b.irq_line = 0;
	i386_set_irq(0);
	g_sched.base = 0;
	for (int i = 0; i < g_sched.nex; i++) {
		g_sched.ex[i].local = 0;
		g_sched.ex[i].suspended = 0;
	}
	for (int i = 0; i < SCHED_TIMERS; i++)
		g_sched.tm[i].id = -1;
	sched_timer_set(T_VBLANK, s_vblank_as, 0);
	if (s_has_rtc)
		sched_timer_set(T_RTC, SCHED_AS_PER_SEC / 256, 0);
	s_mix_n = 0;
	if (s_z80) {
		z80_hold(1);
		b.z80_reset = (uint8_t)s_cart;   // (the SPI board's Z80 waits for the 386; the single boards' runs)
		ymf271_reset(s_ymf);
	}
	if (s_m == M_SYS386I) {
		oki_reset(&s_oki[0]);
		oki_reset(&s_oki[1]);
		s_oki[1].rom = s_oki_rom[1];
	}
	if (s_m == M_SYS386F)
		ymz280b_reset(&s_ymz);
}

// ---- the sound flash kept between runs (MAME's nvram: <set>_flash.nv in the NVRAM folder, both chips) ----
static void flash_nv_load(const char *set)
{
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	// nvram_default: the soundflash1 region, the second chip erased
	memset(s_flash_data, 0xff, 0x200000);
	memcpy(s_flash_data, s_flash1_region, s_flash1_region_size < 0x100000 ? s_flash1_region_size : 0x100000);
	s_flash_nv[0] = 0;
	s_flash_dirty = 0;
	s_flash_quiet = 0;
	if (!dir)
		return;
	snprintf(s_flash_nv, sizeof(s_flash_nv), "%s%s_flash.nv", dir, set);
	FILE *f = fopen(s_flash_nv, "rb");
	if (!f)
		return;
	uint8_t *buf = (uint8_t *)malloc(0x200000);
	if (buf && fread(buf, 1, 0x200000, f) == 0x200000)
		memcpy(s_flash_data, buf, 0x200000);
	free(buf);
	fclose(f);
}

static void flash_nv_save(void)
{
	if (!s_flash_nv[0] || !s_flash_dirty || !s_flash_data)
		return;
	FILE *f = fopen(s_flash_nv, "wb");
	if (!f)
		return;
	fwrite(s_flash_data, 1, 0x200000, f);
	fclose(f);
	s_flash_dirty = 0;
}

static void spi_exit(void)
{
	rbx_video_wait_idle();
	if (s_has_eeprom)
		eepser_nv_save(&s_eeprom);
	if (s_cart)
		flash_nv_save();
	s_has_eeprom = 0;
	i386_exit();
	if (s_ymf && s_ymf->wk) {
		rbx_aux_set_job(NULL);
		rbx_aux2_set_job(NULL);
	}
	if (s_ymf)
		ymf271_destroy(s_ymf);
	s_ymf = NULL;
	rbx_gfx_free(&s_tiles);
	rbx_gfx_free(&s_chars);
	rbx_gfx_free(&s_spr);
	free(s_tile_tr);
	free(s_char_tr);
	s_tile_tr = NULL;
	s_char_tr = NULL;
	free(s_flash_data);
	s_flash_data = NULL;
	free(s_sprbmp);
	s_sprbmp = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
}

// ---- the speedups (init_*) ----
static const uint32_t pc_senkyu[] = { 0x00305bb2 };
static const uint32_t pc_senkyua[] = { 0x30582e };
static const uint32_t pc_batlball[] = { 0x00305996, 0x003058aa };
static const uint32_t pc_viprp1[] = { 0x0202769, 0x02027e9, 0x02026bd };
static const uint32_t pc_viprp1o[] = { 0x0201f99 };
static const uint32_t pc_rdft[] = { 0x0203f06, 0x0203f0a, 0x0203f16, 0x0203f22, 0x0203f46, 0x0203f3a, 0x0203f6e };
static const uint32_t pc_rf2[] = { 0x0203926, 0x0204372, 0x020420e, 0x0204366 };
static const uint32_t pc_rfjet[] = { 0x0206082, 0x0205b39, 0x0205f2e };
#define PCS(a) a, (int)(sizeof(a) / sizeof(a[0]))

static const struct { const char *name; int init; uint32_t addr; const uint32_t *pcs; int n; } s_inits[] = {
	{ "senkyu", I_SENKYU, 0x0018cb4, PCS(pc_senkyu) },
	{ "senkyua", I_SENKYUA, 0x0018c9c, PCS(pc_senkyua) },
	{ "batlball", I_BATLBALL, 0x0018db4, PCS(pc_batlball) },
	{ "viprp1", I_VIPRP1, 0x001e2e0, PCS(pc_viprp1) },
	{ "viprp1o", I_VIPRP1O, 0x001d49c, PCS(pc_viprp1o) },
	{ "ejanhs", I_EJANHS, 0, NULL, 0 },   // (idle skip doesn't work properly? MAME's is off)
	{ "rdft", I_RDFT, 0x00298d0, PCS(pc_rdft) },
	{ "rdft2", I_RDFT2, 0x00282ac, PCS(pc_rf2) },
	{ "rfjet", I_RFJET, 0x002894c, PCS(pc_rfjet) },
	{ "rdft22kc", I_RDFT22KC, 0x00282ac, PCS(pc_rf2) },
	{ "rfjet2kc", I_RFJET2KC, 0x002894c, PCS(pc_rfjet) },
	{ "sys386f", I_SYS386F, 0, NULL, 0 },
};

static const struct { const char *name; int m; } s_machines[] = {
	{ "spi", M_SPI }, { "ejanhs", M_EJANHS }, { "rdft2", M_RDFT2 }, { "sxx2e", M_SXX2E }, { "sxx2f", M_SXX2F },
	{ "sxx2g", M_SXX2G }, { "sys386i", M_SYS386I }, { "sys386f", M_SYS386F },
};

static int parse_machine(const char *mc, int *m, int *init)
{
	const char *colon = strchr(mc, ':');
	if (!colon)
		return -1;
	const size_t len = (size_t)(colon - mc);
	*m = -1;
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (strlen(s_machines[i].name) == len && !strncmp(mc, s_machines[i].name, len))
			*m = s_machines[i].m;
	*init = -1;
	for (size_t i = 0; i < sizeof(s_inits) / sizeof(s_inits[0]); i++)
		if (!strcmp(colon + 1, s_inits[i].name))
			*init = (int)i;
	return (*m < 0 || *init < 0) ? -1 : 0;
}

// the gfx layouts (spi_tilelayout, spi_charlayout, spi_spritelayout, sys386f_spritelayout)
static const uint32_t tl_planes[6] = { 0, 4, 8, 12, 16, 20 };
static const uint32_t tl_x[16] = { 3, 2, 1, 0, 27, 26, 25, 24, 51, 50, 49, 48, 75, 74, 73, 72 };
static const uint32_t tl_y[16] = { 0, 96, 192, 288, 384, 480, 576, 672, 768, 864, 960, 1056, 1152, 1248, 1344, 1440 };
static const uint32_t ch_planes[5] = { 4, 8, 12, 16, 20 };
static const uint32_t ch_x[8] = { 3, 2, 1, 0, 27, 26, 25, 24 };
static const uint32_t ch_y[8] = { 0, 48, 96, 144, 192, 240, 288, 336 };
static const uint32_t sp_x[16] = { 7, 6, 5, 4, 3, 2, 1, 0, 23, 22, 21, 20, 19, 18, 17, 16 };
static const uint32_t sp_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };

static int fail(const rbx_game *g, const char *why)
{
	spi_exit();
	rbx_fail("%s: %s", g->set, why);
	return -1;
}

static int spi_init(const rbx_game *g)
{
	int ii;
	if (parse_machine(g->machine, &s_m, &ii)) {
		rbx_fail("%s: machine %s not supported", g->set, g->machine);
		return -1;
	}
	s_init = s_inits[ii].init;
	s_speedup_addr = s_inits[ii].addr;
	s_speedup_pcs = s_inits[ii].pcs;
	s_speedup_n = s_inits[ii].n;
	s_cart = s_m == M_SPI || s_m == M_EJANHS || s_m == M_RDFT2;
	s_z80 = s_cart || s_m == M_SXX2E || s_m == M_SXX2F || s_m == M_SXX2G;
	s_tilemap = s_m != M_SYS386F;
	s_rise = !(s_m == M_SPI || s_m == M_EJANHS || s_m == M_SXX2E);
	s_has_rtc = s_cart || s_m == M_SXX2E;
	s_has_eeprom = s_m == M_SXX2F || s_m == M_SXX2G || s_m == M_SYS386I || s_m == M_SYS386F;
	s_cpu_clock = s_m == M_SXX2G ? 28636363 : s_m == M_SYS386I ? 40000000 : 25000000;
	s_z80_clock = s_m == M_SXX2G ? 4951200 : 7159090;
	s_palette_size = s_tilemap ? 6144 : 8192;
	s_palette_ram_size = s_tilemap ? 0x3000 : 0x4000;
	s_sprite_ram_size = s_tilemap ? 0x1000 : 0x2000;
	s_ymf = NULL;
	s_snd_rate = 0;

	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	s_sprbmp = calloc((size_t)VIS_W * VIS_H, 4);
	if (!s_bp || !s_snapp || !s_sprbmp)
		return fail(g, "out of memory");

	uint32_t rs = 0, ss = 0;
	s_rom = rbx_region("maincpu", &rs);
	uint8_t *sp = rbx_region("sprites", &ss);
	if (!s_rom || rs < 0x200000 || !sp)
		return fail(g, "ROM regions missing");

	// the graphics: decryption (init_*), then the layouts
	if (s_tilemap) {
		uint32_t cs = 0, ts = 0;
		uint8_t *ch = rbx_region("chars", &cs), *tl = rbx_region("tiles", &ts);
		if (!ch || !tl || cs < 0x30000)
			return fail(g, "ROM regions missing");
		if (s_init == I_RDFT2 || s_init == I_RDFT22KC) {
			seibuspi_text_decrypt(ch, 0x823146, 0x4de2f8, 0x157adc);
			seibuspi_bg_decrypt(tl, (int)ts, 0x823146, 0x4de2f8, 0x157adc);
			seibuspi_rise10_sprite_decrypt(sp, 0x600000);
		} else if (s_init == I_RFJET || s_init == I_RFJET2KC) {
			seibuspi_text_decrypt(ch, 0xaea754, 0xfe8530, 0xccb666);
			seibuspi_bg_decrypt(tl, (int)ts, 0xaea754, 0xfe8530, 0xccb666);
			seibuspi_rise11_sprite_decrypt_rfjet(sp, 0x800000);
		} else {   // init_sei252
			seibuspi_text_decrypt(ch, 0x5a3845, 0x77cf5b, 0x1378df);
			seibuspi_bg_decrypt(tl, (int)ts, 0x5a3845, 0x77cf5b, 0x1378df);
			seibuspi_sprite_decrypt(sp, 0x400000);
		}
		s_bg_fore_layer_position = ts <= 0x300000 ? 0x2000 : ts <= 0x600000 ? 0x4000 : 0x8000;
		const rbx_gfx_layout tll = { 16, 16, 6, tl_planes, tl_x, tl_y, 6 * 16 * 16 };
		const rbx_gfx_layout chl = { 8, 8, 5, ch_planes, ch_x, ch_y, 6 * 8 * 8 };
		const uint32_t f = ss / 3 * 8;
		const uint32_t spp[6] = { 0, 8, f + 0, f + 8, 2 * f + 0, 2 * f + 8 };
		const rbx_gfx_layout spl = { 16, 16, 6, spp, sp_x, sp_y, 16 * 32 };
		if (rbx_gfx_decode(&s_tiles, &tll, tl, ts) || rbx_gfx_decode(&s_chars, &chl, ch, cs) ||
		    rbx_gfx_decode(&s_spr, &spl, sp, ss / 3))
			return fail(g, "out of memory");
		s_tile_tr = rows_transparent(&s_tiles, 63);
		s_char_tr = rows_transparent(&s_chars, 31);
		if (!s_tile_tr || !s_char_tr)
			return fail(g, "out of memory");
		// alpha blending (preliminary; TODO : Differs per games?)
		memset(s_alpha_table, 0, sizeof(s_alpha_table));
		if (s_m != M_EJANHS) {
			memset(s_alpha_table + 0x730, 1, 0x10);
			memset(s_alpha_table + 0x780, 1, 0x20);
			memset(s_alpha_table + 0xfc0, 1, 0x40);
			memset(s_alpha_table + 0x1200 + 0x160, 1, 0x20);
			memset(s_alpha_table + 0x1200 + 0x1b0, 1, 0x10);
			memset(s_alpha_table + 0x1200 + 0x1f0, 1, 0x10);
			memset(s_alpha_table + 0x1400 + 0x1b0, 1, 0x10);
			memset(s_alpha_table + 0x1400 + 0x1f0, 1, 0x10);
			memset(s_alpha_table + 0x1600 + 0x170, 1, 0x10);
			memset(s_alpha_table + 0x1600 + 0x1f0, 1, 0x10);
		}
	} else {   // init_sys386f: sprite_reorder() only
		uint16_t *src = (uint16_t *)sp, tmp[0x40 / 2];
		for (uint32_t i = 0; i < ss / 0x40; i++) {
			memcpy(tmp, src, 0x40);
			for (int j = 0; j < 0x40 / 2; j++) {
				const uint16_t offset = (uint16_t)((j >> 1) | (j << 4 & 0x10));
				*src++ = tmp[offset];
			}
		}
		const uint32_t f = ss / 4 * 8;
		const uint32_t spp[8] = { 0, 8, f + 0, f + 8, 2 * f + 0, 2 * f + 8, 3 * f + 0, 3 * f + 8 };
		const rbx_gfx_layout spl = { 16, 16, 8, spp, sp_x, sp_y, 16 * 32 };
		if (rbx_gfx_decode(&s_spr, &spl, sp, ss / 4))
			return fail(g, "out of memory");
		memset(s_alpha_table, 0, sizeof(s_alpha_table));   // no alpha blending
	}
	for (int i = 0; i < 0x2000 / 32; i++) {
		s_alpha32[i] = 0;
		for (int p = 0; p < 32; p++)
			s_alpha32[i] |= s_alpha_table[i * 32 + p];
	}

	// the screen: base_video (raw 7.159 MHz, 448 x 296, vblank at line 240) / sys386f (57.59 Hz)
	if (s_tilemap) {
		const double pixel_clock = 28636363.0 / 4;
		s_frame_as = (int64_t)(1e18 * 448 * 296 / pixel_clock);
		s_vblank_as = (int64_t)(1e18 * 448 * 240 / pixel_clock);
		s_fps_mhz = (uint32_t)(pixel_clock * 1000 / (448 * 296));
	} else {
		s_frame_as = (int64_t)(1e18 / 57.59);
		s_vblank_as = s_frame_as * 240 / 256;
		s_fps_mhz = 57590;
	}
	if (rbx_video_setup(VIS_W, VIS_H, (int)s_palette_size))
		return fail(g, "out of memory");

	// the ports
	s_port_inputs = rbx_port_find("INPUTS");
	s_port_exch = rbx_port_find("EXCH");
	s_port_system = rbx_port_find("SYSTEM");
	s_port_coin = rbx_port_find("COIN");
	s_port_jumpers = rbx_port_find("JUMPERS");
	s_port_special = rbx_port_find("SPECIAL");
	for (int i = 0; i < 5; i++) {
		char tag[8] = "KEY0";
		tag[3] = (char)('0' + i);
		s_port_key[i] = rbx_port_find(tag);
	}

	// the scheduler, the CPUs
	sched_init(s_z80 ? SCHED_AS_PER_SEC / 12000 : SCHED_AS_PER_SEC / 60);
	(void)sched_add_exec(s_cpu_clock, cpu_run, cpu_done, cpu_abort);
	const i386_bus bus = { cpu_r32, cpu_w32, NULL, NULL, irq_ack };
	if (i386_init(&bus))
		return fail(g, "out of memory");
	i386_map(0x10000, 0x40000, (uint8_t *)s_bp->ram + 0x10000, 1);
	if (s_speedup_n)   // (the speedup's page through the handlers)
		i386_map(s_speedup_addr & ~0xffffu, (s_speedup_addr & ~0xffffu) + 0x10000, NULL, 0);
	// (the RAM of the pages left to the handlers read and written straight in 4 KB parts: all of them but the I/O's
	// (below 0x800) and the speedup dword's; cpu_r32 / cpu_w32 read and write them as plain RAM)
	for (uint32_t a = 0; a < 0x40000; a += 0x1000) {
		const int handlers = a < 0x1000 || (s_speedup_n && a == (s_speedup_addr & ~0xfffu));
		const int unmapped = a < 0x10000 || (s_speedup_n && (a >> 16) == (s_speedup_addr >> 16));
		if (unmapped && !handlers)
			i386_map_split(a, a + 0x1000, (uint8_t *)s_bp->ram + a, 1);
	}
	i386_map(0x00200000, 0x00400000, s_rom, 0);
	i386_map(0xffe00000, 0x00000000, s_rom, 0); // ROM location in real-mode
	if (!getenv("RBX_SEIBU_NOIDLE"))   // (the polling loops: the same result, RBX_SEIBU_NOIDLE=1 to compare)
		i386_idle_enable(cpu_pure);

	// the sound
	if (s_z80) {
		s_zrom = rbx_region("audiocpu", &s_zrom_size);
		if (!s_zrom || s_zrom_size < 0x40000)
			return fail(g, "ROM regions missing");
		const uint32_t ymf_clock = s_m == M_SXX2G ? 16384000 : 16934400;
		s_ymf = ymf271_create(ymf_clock, ymf_mem_r, ymf_mem_w, ymf_irq, ymf_timer, NULL);
		if (!s_ymf)
			return fail(g, "out of memory");
		s_snd_rate = ymf271_rate(s_ymf);
		s_mix_cap = (int)(s_snd_rate / 30) + 64;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap * 4, sizeof(int32_t));
		if (!s_mix || (s_cart ? rbx_audio_setup_stereo(s_snd_rate) : rbx_audio_setup(s_snd_rate)))
			return fail(g, "out of memory");
		if (s_cart) {
			s_sound01 = rbx_region("sound01", &s_sound01_size);
			s_flash1_region = rbx_region("soundflash1", &s_flash1_region_size);
			s_flash_data = (uint8_t *)malloc(0x200000);
			if (!s_sound01 || !s_flash1_region || !s_flash1_region_size || !s_flash_data)
				return fail(g, "ROM regions missing");
			i386_map(0x00a00000, 0x00a00000 + (s_sound01_size & ~0xffffu), s_sound01, 0);
			intelfsh_init(&s_flash[0], s_flash_data, 0x100000, 0x89, 0xa2, flash_delay, (void *)(intptr_t)0);
			intelfsh_init(&s_flash[1], s_flash_data + 0x100000, 0x100000, 0x89, 0xa2, flash_delay, (void *)(intptr_t)1);
			flash_nv_load(g->set);
			flash_rom_update();
		} else {
			s_ymf_rom = rbx_region("ymf", &s_ymf_rom_size);
			if (!s_ymf_rom)
				return fail(g, "ROM regions missing");
			ymf271_set_rom(s_ymf, s_ymf_rom, s_ymf_rom_size);   // (ymf_mem_r's reads, no call a sample)
		}
		// the samples on the rbx_aux workers (they read the ROM, or the flash, which changes only after flash_change)
		if (!ymf271_set_async(s_ymf, 2, ymf_kick, ymf_sync, NULL)) {
			rbx_aux_set_job(ymf_job);
			rbx_aux2_set_job(ymf_job2);
			rbx_aux2_set_alone(1);
		}
		s_zrom_mask = (s_zrom_size & (s_zrom_size - 1)) ? 0 : s_zrom_size - 1;
		z80_init(&s_z);
		s_zmap_bank = -1;   // (the map made again for this game's ROM)
		s_z.read_byte = z_read;
		s_z.write_byte = z_write;
		s_z.port_in = z_in;
		s_z.port_out = z_out;
		(void)sched_add_exec(s_z80_clock, z_run, z_done, z_abort);
	} else if (s_m == M_SYS386I) {
		for (int i = 0; i < 2; i++) {
			s_oki_rom[i] = rbx_region(i ? "oki2" : "oki1", &s_oki_size[i]);
			if (!s_oki_rom[i])
				return fail(g, "ROM regions missing");
			oki_init(&s_oki[i], 28636363 / 20, 1, s_oki_rom[i], i ? 0x40000 : s_oki_size[i], 128);   // (0.50)
		}
		s_snd_rate = s_oki[0].rate;
		if (rbx_audio_setup(s_snd_rate))
			return fail(g, "out of memory");
	} else {   // sys386f
		uint32_t zs = 0;
		const uint8_t *zr = rbx_region("ymz", &zs);
		if (!zr)
			return fail(g, "ROM regions missing");
		ymz280b_init(&s_ymz, 16384000, zr, zs, NULL, NULL);
		s_snd_rate = ymz280b_rate(&s_ymz);
		if (rbx_audio_setup(s_snd_rate))
			return fail(g, "out of memory");
	}
	if (s_has_eeprom) {   // (EEPROM_93C46_16BIT)
		uint32_t es = 0;
		const uint8_t *e = rbx_region_opt("eeprom", &es);
		eepser_init(&s_eeprom, EEPSER_93CXX, 64, 16, s_cpu_clock);
		if (e)
			eepser_default(&s_eeprom, e, es);
		eepser_nv_load(&s_eeprom, g->set);
	}
	spi_reset();
	return 0;
}

static int spi_state(void *ctx, int load)
{
	if (load) {
		rbx_video_wait_idle();
		s_zi.rec = 0;   // (the sound Z80's pass being noted or skipped into: of the run before)
		s_zi.at = -1;
	}
	int rc = rbx_state_io(ctx, "seibuspi", s_bp, (uint32_t)offsetof(board_t, state_end));
	rc |= i386_state(ctx, load);
	if (s_z80) {
		z80 zc = s_z;
		rc |= rbx_state_io(ctx, "seibuspi_z80", &zc, (uint32_t)sizeof(zc));
		if (load && !rc) {
			zc.read_byte = z_read;
			zc.write_byte = z_write;
			zc.port_in = z_in;
			zc.port_out = z_out;
			zc.read_map = NULL;
			zc.op_rom = NULL;
			s_z = zc;
		}
		if (!load)
			ymf271_state_save_prep(s_ymf);
		rc |= rbx_state_io(ctx, "seibuspi_ymf", s_ymf, (uint32_t)YMF271_STATE_SIZE);
		if (load)
			ymf271_state_loaded(s_ymf);
		if (s_cart) {
			rc |= rbx_state_io(ctx, "seibuspi_zrom", s_zrom, s_zrom_size);
			rc |= rbx_state_io(ctx, "seibuspi_flash", s_flash_data, 0x200000);
			rc |= rbx_state_io(ctx, "seibuspi_fsh0", &s_flash[0], (uint32_t)INTELFSH_STATE_SIZE);
			rc |= rbx_state_io(ctx, "seibuspi_fsh1", &s_flash[1], (uint32_t)INTELFSH_STATE_SIZE);
			if (load)
				flash_rom_update();
		}
	}
	if (s_m == M_SYS386I)
		for (int i = 0; i < 2; i++)
			rc |= rbx_state_io(ctx, i ? "seibuspi_oki2" : "seibuspi_oki1", &s_oki[i], (uint32_t)offsetof(okim6295_t, rom));
	if (s_m == M_SYS386F)
		rc |= rbx_state_io(ctx, "seibuspi_ymz", &s_ymz, (uint32_t)YMZ280B_STATE_SIZE);
	if (s_has_eeprom)
		rc |= eepser_state(&s_eeprom, ctx, load, "seibuspi_eeprom");
	rc |= sched_state(ctx, load);
	if (load && !rc && s_m == M_SYS386I)
		s_oki[1].rom = s_oki_rom[1] + (((b.oki_bank >> 2) & 1) ? 0x40000 : 0);
	return rc ? -1 : 0;
}

static int spi_save(void *ctx) { return spi_state(ctx, 0); }
static int spi_load(void *ctx) { return spi_state(ctx, 1); }
static int spi_supported(const char *machine)
{
	int m, i;
	return parse_machine(machine, &m, &i) == 0;
}

const rbx_driver rbx_drv_seibuspi = { "Seibu SPI", spi_init, spi_exit, spi_reset, spi_frame, spi_save, spi_load,
                                      spi_supported, spi_render, spi_help };
