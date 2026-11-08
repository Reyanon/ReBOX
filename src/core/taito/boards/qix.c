/*  Taito Qix hardware, ported from MAME src/mame/taito/qix.cpp, qix_m.cpp, qix_v.cpp, qix_a.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles, Zsolt Vasvari; qix_a.cpp: Derrick Renaud, Couriersud;
    thanks-to: John Butler, Ed Mueller).
    Data CPU and video CPU: MC6809E at 20 MHz/16 (cpu/m6809), sound CPU: M6802 at 7.3728 MHz/2 (internal /4,
    cpu/m6800), six 6821 PIAs (machine/pia6821), MC6845 CRTC (video/mc6845), 256x256x8 frame buffer with a
    1024-entry RRGGBBII palette. Memory maps, PIA wiring, FIRQ / IRQ lines and the CRTC-driven scanline latch and
    VSYNC follow MAME. MAME runs the machine with a perfect quantum on the data CPU: here the video and sound CPUs
    catch up after every data CPU instruction.
    Sound: MAME's discrete graph (qix_discrete) as plain code: DAC (data * 128 - 16384), per-channel 4-bit
    attenuation (parallel 22k/10k/5.6k/3.3k against 10k, full volume when no resistor is selected), 1.5k / 1uF
    CR high-pass; the two channels are averaged to mono at 50 kHz (box-filtered DAC).
    Machines: qix (qix, qixa, qixb, qixo, qix2, complexx), zookeepbl (video ROM bank), kram3 (krame: opcode
    decryption on the LIC cycle, MAME's partial tables). Sets with the 68705 coin MCU (mcu / zookeep machines)
    and Slither (SN76489 sound) are not emulated yet.
    The emulation runs 1/60 s per frame (the CRTC frame is ~55.9 Hz); the picture is captured at VSYNC.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6800/m6800.h"
#include "../../machine/pia6821.h"
#include "../../video/mc6845.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "qix_kram3.h"

enum { M_QIX, M_ZOOKEEPBL, M_KRAM3 };

typedef struct {
	const char *name;
	int kind;
} machine_t;

static const machine_t s_machines[] = {
	{ "qix", M_QIX },
	{ "zookeepbl", M_ZOOKEEPBL },
	{ "kram3", M_KRAM3 },
};

#define MAIN_CLOCK   1250000   // 20 MHz / 4 / 4
#define SOUND_CLOCK  921600    // 7.3728 MHz / 2, 6802 internal / 4
#define SND_RATE     50000     // output rate: MAIN_CLOCK / 25
#define SND_DIV      25
#define SCREEN_W     256
#define SCREEN_H     256
#define PEN_BLACK    1024      // (the core masks pens with palette_size - 1: palette of 2048)

static const machine_t *s_m;
static uint8_t *s_mrom, *s_vrom, *s_arom;
static uint32_t s_vrom_size;
static uint8_t *s_main_dec, *s_video_dec;   // kram3 decrypted opcodes (0xa000-0xffff)

static m6809_t s_main, s_video;
static m6800_t s_audio;
static pia6821_t s_pia[3], s_sndpia[3];
static mc6845_t s_crtc;

// memory
static uint8_t s_shared[0x400], s_mainram[0x400], s_nvram[0x400], s_palram[0x400];
static uint8_t s_audram[0x100];
static uint8_t *s_vram;   // 64 KB frame buffer

// machine state (saved)
typedef struct {
	uint8_t vram_addr[2], scanline_latch, palette_bank, leds, flip, videobank, main_irq_a, main_irq_b;
	uint8_t snd_irq_a, snd_irq_b, dac, vol, captured, pad_[2];
	int64_t frame_end, next_line, snd_t;
	uint64_t frame_frac;
	int32_t snd_acc;
	int32_t hp_cap;        // high-pass capacitor voltage, 16.16
	int32_t level;         // current mixed (mono) level before the filter
} qstate_t;
static qstate_t s_st;

static int s_iobase;      // data CPU I/O base: 0x8000 (0x0000 on Zoo Keeper)
static int s_port_p1, s_port_coin, s_port_spare, s_port_in0, s_port_p2;
static char s_nv_path[512];

// render snapshot
typedef struct {
	uint8_t vram[0x10000];
	uint8_t pal[0x400];
	uint8_t bank, flip, valid, hdisp, vdisp, max_ras;
	uint16_t start;
} snap_t;
static snap_t *s_snap;

static int32_t s_att[16];   // 4-bit attenuation, Q15

// ---- sound (qix_discrete) ----
static void snd_update(int64_t t)
{
	if (t <= s_st.snd_t)
		return;
	while (s_st.snd_t < t) {
		int64_t next = (s_st.snd_t / SND_DIV + 1) * SND_DIV;
		int64_t to = next < t ? next : t;
		s_st.snd_acc += s_st.level * (int32_t)(to - s_st.snd_t);
		s_st.snd_t = to;
		if (to == next) {
			int32_t x = s_st.snd_acc / SND_DIV;
			s_st.snd_acc = 0;
			// CR filter, 1.5k * 1uF at 50 kHz: exponent 1 - exp(-1 / 75) = 0.013245
			int32_t y = x - (s_st.hp_cap >> 16);
			s_st.hp_cap += (int32_t)(((((int64_t)x << 16) - s_st.hp_cap) * 868) >> 16);
			rbx_audio_put(y);
		}
	}
}

static void snd_level(void)
{
	int32_t d = s_st.dac * 128 - 128 * 128;
	int l = (s_st.vol >> 4) & 15, r = s_st.vol & 15;
	int32_t vl = l ? (d * s_att[l]) >> 15 : d;
	int32_t vr = r ? (d * s_att[r]) >> 15 : d;
	s_st.level = (vl + vr) / 2;
}

static int64_t audio_to_main(int64_t c) { return c * 100000 / 73728; }

static void dac_w(void *ctx, uint8_t data)
{
	(void)ctx;
	int64_t t = audio_to_main(s_audio.cycles);
	if (t < s_st.snd_t)
		t = s_st.snd_t;
	snd_update(t);
	s_st.dac = data;
	snd_level();
}

static void vol_w(void *ctx, uint8_t data)
{
	(void)ctx;
	snd_update(s_main.cycles);
	s_st.vol = data;
	snd_level();
}

// ---- PIA wiring ----
static uint8_t in_p1(void *ctx) { (void)ctx; return (uint8_t)rbx_port_read(s_port_p1); }
// First power on (no NVRAM yet): the game asks the operator for its language and settings, stepped through with the
// Test Advance switch, then wants a reset. That is done here as an operator would: Test Advance pressed for each of the
// six screens (their defaults kept), then a reset; the NVRAM then holds the settings and the game starts as usual
#define SETUP_PRESSES 6
#define SETUP_GAP 120   // frames between presses
static int s_setup;     // frames into the first power on's setup, 0 = none
static uint8_t in_coin(void *ctx)
{
	(void)ctx;
	uint8_t v = (uint8_t)rbx_port_read(s_port_coin);
	if (s_setup && s_setup % SETUP_GAP < 10 && s_setup / SETUP_GAP >= 1 && s_setup / SETUP_GAP <= SETUP_PRESSES)
		v &= (uint8_t)~0x01;   // Test Advance (active low)
	return v;
}
static uint8_t in_spare(void *ctx) { (void)ctx; return (uint8_t)rbx_port_read(s_port_spare); }
static uint8_t in_in0(void *ctx) { (void)ctx; return (uint8_t)rbx_port_read(s_port_in0); }
static uint8_t in_p2(void *ctx) { (void)ctx; return (uint8_t)rbx_port_read(s_port_p2); }
static void coinctr_w(void *ctx, uint8_t data) { (void)ctx; (void)data; }

static void sndpia0_out_a(void *ctx, uint8_t d) { (void)ctx; pia6821_porta_w(&s_sndpia[1], d); }
static void sndpia0_ca2(void *ctx, int s) { (void)ctx; pia6821_ca1_w(&s_sndpia[1], s); }
static void sndpia0_cb2(void *ctx, int s) { (void)ctx; s_st.flip = (uint8_t)(s != 0); }
static void main_irq_update(void) { m6809_set_irq(&s_main, s_st.main_irq_a | s_st.main_irq_b); }
static void sndpia0_irqa(void *ctx, int s) { (void)ctx; s_st.main_irq_a = (uint8_t)s; main_irq_update(); }
static void sndpia0_irqb(void *ctx, int s) { (void)ctx; s_st.main_irq_b = (uint8_t)s; main_irq_update(); }
static void sndpia1_out_a(void *ctx, uint8_t d) { (void)ctx; pia6821_porta_w(&s_sndpia[0], d); }
static void sndpia1_ca2(void *ctx, int s) { (void)ctx; pia6821_ca1_w(&s_sndpia[0], s); }
static void snd_irq_update(void) { m6800_set_irq(&s_audio, s_st.snd_irq_a | s_st.snd_irq_b); }
static void sndpia1_irqa(void *ctx, int s) { (void)ctx; s_st.snd_irq_a = (uint8_t)s; snd_irq_update(); }
static void sndpia1_irqb(void *ctx, int s) { (void)ctx; s_st.snd_irq_b = (uint8_t)s; snd_irq_update(); }

static void pia_setup(void)
{
	for (int i = 0; i < 3; i++) {
		pia6821_config(&s_pia[i]);
		pia6821_config(&s_sndpia[i]);
		s_pia[i].a_input_overrides_output_mask = 0xff;
	}
	s_pia[0].in_a_cb = in_p1;
	s_pia[0].in_b_cb = in_coin;
	s_pia[1].in_a_cb = in_spare;
	s_pia[1].in_b_cb = in_in0;
	s_pia[2].in_a_cb = in_p2;
	s_pia[2].out_b_cb = coinctr_w;
	s_pia[2].ts_b = 0;
	s_sndpia[0].out_a_cb = sndpia0_out_a;
	s_sndpia[0].out_b_cb = vol_w;
	s_sndpia[0].ts_b = 0xff;
	s_sndpia[0].ca2_cb = sndpia0_ca2;
	s_sndpia[0].cb2_cb = sndpia0_cb2;
	s_sndpia[0].irqa_cb = sndpia0_irqa;
	s_sndpia[0].irqb_cb = sndpia0_irqb;
	s_sndpia[1].out_a_cb = sndpia1_out_a;
	s_sndpia[1].out_b_cb = dac_w;
	s_sndpia[1].ts_b = 0;
	s_sndpia[1].ca2_cb = sndpia1_ca2;
	s_sndpia[1].irqa_cb = sndpia1_irqa;
	s_sndpia[1].irqb_cb = sndpia1_irqb;
	// sndpia[2] (TMS5220 interface) is not used by any game
}

// ---- data CPU ----
static uint8_t main_read(void *ctx, uint16_t a)
{
	(void)ctx;
	int o = a - s_iobase;
	if (o < 0 || o >= 0x2000)
		return 0;
	switch (o >> 10) {
	case 2: return 0;   // 6850 ACIA (diagnostics only)
	case 3:
		if (o & 1) m6809_set_firq(&s_main, 0);    // data FIRQ acknowledge
		else m6809_set_firq(&s_video, 1);          // assert FIRQ on the video CPU
		return 0;
	case 4: return pia6821_read(&s_sndpia[0], o);
	case 5: return pia6821_read(&s_pia[0], o);
	case 6: return pia6821_read(&s_pia[1], o);
	case 7: return pia6821_read(&s_pia[2], o);
	}
	return 0;
}

static void main_write(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	int o = a - s_iobase;
	if (o < 0 || o >= 0x2000)
		return;
	switch (o >> 10) {
	case 3:
		if (o & 1) m6809_set_firq(&s_main, 0);
		else m6809_set_firq(&s_video, 1);
		break;
	case 4: pia6821_write(&s_sndpia[0], o, v); break;
	case 5: pia6821_write(&s_pia[0], o, v); break;
	case 6: pia6821_write(&s_pia[1], o, v); break;
	case 7: pia6821_write(&s_pia[2], o, v); break;
	}
}

static uint8_t main_read_op(void *ctx, uint16_t a)
{
	if (a >= 0xa000)
		return s_main_dec[a - 0xa000];
	const uint8_t *p = s_main.rpage[a >> 8];
	return p ? p[a & 0xff] : main_read(ctx, a);
}

// ---- video CPU ----
static void map_vram(void)
{
	uint8_t *base = s_vram + ((s_st.vram_addr[0] & 0x80) << 8);
	for (int i = 0; i < 0x80; i++) {
		s_video.rpage[i] = base + i * 256;
		s_video.wpage[i] = base + i * 256;
	}
}

static void map_videobank(void)
{
	if (s_m->kind != M_ZOOKEEPBL)
		return;
	const uint8_t *base = s_st.videobank ? s_vrom + 0x10000 : s_vrom + 0xa000;
	for (int i = 0; i < 0x20; i++)
		s_video.rpage[0xa0 + i] = base + i * 256;
}

static void palettebank_w(uint8_t data)
{
	s_st.palette_bank = data & 3;
	s_st.leds = (uint8_t)(~data & 0xfc);
}

static uint8_t video_read(void *ctx, uint16_t a)
{
	(void)ctx;
	switch ((a >> 10) & 0x3f) {
	case 0x23:   // 0x8c00
		if (a & 1) m6809_set_firq(&s_video, 0);    // video FIRQ acknowledge
		else m6809_set_firq(&s_main, 1);           // assert FIRQ on the data CPU
		return 0;
	case 0x25:   // 0x9400: video RAM at the latched address
		if ((a & 3) == 0)
			return s_vram[s_st.vram_addr[0] << 8 | s_st.vram_addr[1]];
		return 0;
	case 0x26: return s_st.scanline_latch;
	case 0x27: return (a & 1) ? mc6845_register_r(&s_crtc) : 0;
	}
	return 0;
}

static void video_write(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	switch ((a >> 10) & 0x3f) {
	case 0x22:   // 0x8800
		if (s_m->kind == M_ZOOKEEPBL && (a & 1)) {
			s_st.videobank = (v >> 2) & 1;
			map_videobank();
		}
		palettebank_w(v);
		break;
	case 0x23:
		if (a & 1) m6809_set_firq(&s_video, 0);
		else m6809_set_firq(&s_main, 1);
		break;
	case 0x25:
		switch (a & 3) {
		case 0: s_vram[s_st.vram_addr[0] << 8 | s_st.vram_addr[1]] = v; break;
		case 2: {
			int old = s_st.vram_addr[0] & 0x80;
			s_st.vram_addr[0] = v;
			if ((v & 0x80) != old)
				map_vram();
			break;
		}
		case 3: s_st.vram_addr[1] = v; break;
		}
		break;
	case 0x27:
		if (a & 1) mc6845_register_w(&s_crtc, v);
		else mc6845_address_w(&s_crtc, v);
		break;
	}
}

static uint8_t video_read_op(void *ctx, uint16_t a)
{
	if (a >= 0xa000)
		return s_video_dec[a - 0xa000];
	const uint8_t *p = s_video.rpage[a >> 8];
	return p ? p[a & 0xff] : video_read(ctx, a);
}

// ---- sound CPU ----
static uint8_t audio_read(void *ctx, uint16_t a)
{
	(void)ctx;
	if (!(a & 0x8000)) {
		if (a & 0x4000) return pia6821_read(&s_sndpia[1], a);
		if (a & 0x2000) return pia6821_read(&s_sndpia[2], a);
	}
	return 0;
}

static void audio_write(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (!(a & 0x8000)) {
		if (a & 0x4000) pia6821_write(&s_sndpia[1], a, v);
		else if (a & 0x2000) pia6821_write(&s_sndpia[2], a, v);
	}
}

// ---- kram3 opcode decryption (MAME init_kram3) ----
static inline int bit(int v, int n) { return (v >> n) & 1; }

static int kram3_permut1(int idx, int v)
{
	switch (idx) {
	case 1: return (v & 0xf0) | bit(v, 0) << 3 | bit(v, 3) << 2 | bit(v, 2) << 1 | bit(v, 1);
	case 2: return (v & 0xf0) | bit(v, 1) << 3 | bit(v, 0) << 2 | bit(v, 3) << 1 | bit(v, 2);
	case 3: return (v & 0xf0) | bit(v, 2) << 3 | bit(v, 3) << 2 | bit(v, 0) << 1 | bit(v, 1);
	}
	return v;
}

static int kram3_permut2(int tbl_index, int idx, const uint8_t *xor_table)
{
	int x = 0;
	if (idx == 0 || idx == 3) {
		x = xor_table[tbl_index];
		if (x == 99)
			return x;
	}
	x ^= 0x02;
	if (idx == 3)
		x = (x & 0xf0) | bit(x, 0) << 3 | bit(x, 2) << 2 | bit(x, 3) << 1 | bit(x, 1);
	return x;
}

static int kram3_decrypt(int address, int value)
{
	int indx1 = (bit(address, 1) << 1) | bit(address, 5);
	int indx2 = (bit(address, 7) << 1) | bit(address, 3);
	int bits1 = ((value & 0x10) >> 1) | (value & 0x07);
	int bits2 = ((value & 0xe0) >> 4) | ((value & 0x08) >> 3);
	int tbl_index = ((address & 0x7f00) >> 4) | (bit(address, 6) << 3) | (bit(address, 4) << 2) |
	                (bit(address, 2) << 1) | bit(address, 0);
	int xor1 = kram3_permut2(tbl_index, indx1, xor1_table);
	int xor2 = kram3_permut2(tbl_index, indx2, xor2_table);
	if (xor1 == 99 || xor2 == 99)
		return 99;
	bits1 = kram3_permut1(indx1, bits1) ^ xor1;
	bits2 = kram3_permut1(indx2, bits2) ^ xor2;
	return ((bits2 & 0xe) << 4) | ((bits1 & 0x8) << 1) | ((bits2 & 0x1) << 3) | (bits1 & 0x7);
}

// ---- video ----
static void qx_render(void)
{
	const snap_t *s = s_snap;
	for (int i = 0; i < 1024; i++) {   // qix_R2G2B2I2
		static const uint8_t table[16] = {
			0x00, 0x12, 0x24, 0x49, 0x12, 0x24, 0x49, 0x92, 0x5b, 0x6d, 0x92, 0xdb, 0x7f, 0x91, 0xb6, 0xff };
		uint8_t raw = s->pal[i];
		int in = raw & 3;
		int r = table[((raw >> 6) & 3) << 2 | in], g = table[((raw >> 4) & 3) << 2 | in], b = table[((raw >> 2) & 3) << 2 | in];
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	rbx_video.palette[PEN_BLACK] = 0;
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	int ch = s->max_ras + 1;
	int vis_h = s->valid ? s->vdisp * ch : 0, vis_w = s->hdisp * 8;
	if (vis_w > SCREEN_W)
		vis_w = SCREEN_W;
	uint16_t pens = (uint16_t)(s->bank << 8);
	uint16_t xr = s->flip ? 0xffff : 0;
	uint16_t ma = s->start;
	for (int y = 0; y < SCREEN_H; y++) {
		uint16_t *d = bm + y * pitch;
		if (y >= vis_h) {
			for (int x = 0; x < SCREEN_W; x++)
				d[x] = PEN_BLACK;
			continue;
		}
		int ra = y % ch;
		uint16_t offs = (uint16_t)(((ma << 6) & 0xf800) | ((ra << 8) & 0x0700));
		int x = 0;
		for (; x < vis_w; x++)
			d[x] = pens | s->vram[(uint16_t)((offs + x) ^ xr)];
		for (; x < SCREEN_W; x++)
			d[x] = PEN_BLACK;
		if (ra == ch - 1)
			ma = (uint16_t)((ma + s->hdisp) & 0x3fff);
	}
}

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_vram, 0x10000);
	memcpy(s_snap->pal, s_palram, sizeof(s_palram));
	s_snap->bank = s_st.palette_bank;
	s_snap->flip = s_st.flip;
	s_snap->valid = s_crtc.valid;
	s_snap->hdisp = s_crtc.hdisp;
	s_snap->vdisp = s_crtc.vdisp;
	s_snap->max_ras = s_crtc.max_ras;
	s_snap->start = s_crtc.start_addr;
	rbx_video_frame_done();
	s_st.captured = 1;
}

static int line_cycles(void)
{
	// one scanline = (R0 + 1) characters of 8 pixels at 5 MHz = 2 * (R0 + 1) data CPU clocks
	return s_crtc.valid ? 2 * (s_crtc.htotal + 1) : 0x148 / 4;
}

static void scanline(void)
{
	if (!s_crtc.valid)
		return;
	int old_vsync = s_crtc.vsync;
	mc6845_line(&s_crtc);
	if (s_crtc.de)   // display enable rising edge: RA0-RA2 -> D0-D2, MA5-MA9 -> D3-D7
		s_st.scanline_latch = (uint8_t)(((s_crtc.line_address >> 2) & 0xf8) | (s_crtc.raster & 0x07));
	if (s_crtc.vsync != old_vsync) {
		pia6821_cb1_w(&s_sndpia[0], s_crtc.vsync);
		if (s_crtc.vsync && !s_st.captured)
			capture();
	}
}

// ---- init / frame ----
static const machine_t *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
}

static void map_cpus(void)
{
	int k = s_m->kind;
	// data CPU
	m6809_init(&s_main);
	s_main.read = main_read;
	s_main.write = main_write;
	int b = s_iobase >> 8;
	for (int i = 0; i < 4; i++) {
		s_main.rpage[b + i] = s_main.wpage[b + i] = s_shared + i * 256;
		s_main.rpage[b + 4 + i] = s_main.wpage[b + 4 + i] = s_mainram + i * 256;
	}
	for (int i = k == M_ZOOKEEPBL ? 0x80 : 0xa0; i < 0x100; i++)
		s_main.rpage[i] = s_mrom + i * 256;
	if (k == M_KRAM3)
		s_main.read_op = main_read_op;
	// video CPU
	m6809_init(&s_video);
	s_video.read = video_read;
	s_video.write = video_write;
	for (int i = 0; i < 4; i++) {
		s_video.rpage[0x80 + i] = s_video.wpage[0x80 + i] = s_shared + i * 256;
		s_video.rpage[0x84 + i] = s_video.wpage[0x84 + i] = s_nvram + i * 256;
		s_video.rpage[0x90 + i] = s_video.wpage[0x90 + i] = s_palram + i * 256;
	}
	for (int i = 0xa0; i < 0x100; i++)
		s_video.rpage[i] = s_vrom + i * 256;
	if (k == M_KRAM3)
		s_video.read_op = video_read_op;
	map_vram();
	map_videobank();
	// sound CPU
	m6800_init(&s_audio);
	s_audio.read = audio_read;
	s_audio.write = audio_write;
	s_audio.rpage[0] = s_audio.wpage[0] = s_audram;   // 6802 internal RAM (0x00-0x7f)
	for (int i = 0xd0; i < 0x100; i++)
		s_audio.rpage[i] = s_arom + i * 256;
}

static void qx_reset(void)
{
	s_main.cycles = s_video.cycles = s_audio.cycles = 0;
	memset(s_shared, 0, sizeof(s_shared));
	memset(s_mainram, 0, sizeof(s_mainram));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_audram, 0, sizeof(s_audram));
	memset(s_vram, 0, 0x10000);
	memset(&s_st, 0, sizeof(s_st));
	s_st.dac = 0x80;
	s_st.vol = 0xff;
	mc6845_reset(&s_crtc);
	map_vram();
	map_videobank();
	for (int i = 0; i < 3; i++) {
		pia6821_reset(&s_pia[i]);
		pia6821_reset(&s_sndpia[i]);
	}
	snd_level();
	s_main.firq_line = s_main.irq_line = 0;
	s_video.firq_line = s_video.irq_line = 0;
	m6809_reset(&s_main);
	m6809_reset(&s_video);
	m6800_reset(&s_audio);
	s_st.next_line = line_cycles();
}

static void qx_exit(void);

static int qx_init(const rbx_game *g)
{
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t ms, as;
	s_mrom = rbx_region("maincpu", &ms);
	s_vrom = rbx_region("videocpu", &s_vrom_size);
	s_arom = rbx_region("audiocpu", &as);
	if (!s_mrom || !s_vrom || !s_arom || ms < 0x10000 || s_vrom_size < 0x10000 || as < 0x10000 ||
	    (s_m->kind == M_ZOOKEEPBL && s_vrom_size < 0x12000)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_iobase = s_m->kind == M_ZOOKEEPBL ? 0x0000 : 0x8000;
	s_vram = (uint8_t *)calloc(1, 0x10000);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	if (!s_vram || !s_snap) {
		rbx_fail("%s: out of memory", g->set);
		qx_exit();
		return -1;
	}
	if (s_m->kind == M_KRAM3) {
		s_main_dec = (uint8_t *)malloc(0x6000);
		s_video_dec = (uint8_t *)malloc(0x6000);
		if (!s_main_dec || !s_video_dec) {
			rbx_fail("%s: out of memory", g->set);
			qx_exit();
			return -1;
		}
		for (int i = 0xa000; i < 0x10000; i++) {
			s_main_dec[i - 0xa000] = (uint8_t)kram3_decrypt(i, s_mrom[i]);
			s_video_dec[i - 0xa000] = (uint8_t)kram3_decrypt(i, s_vrom[i]);
		}
	}
	// attenuation: parallel resistance of the selected resistors (+250 ohm each) against 10k
	static const double res[4] = { 22000 + 250, 10000 + 250, 5600 + 250, 3300 + 250 };
	s_att[0] = 32768;
	for (int v = 1; v < 16; v++) {
		double g_ = 0;
		for (int i = 0; i < 4; i++)
			if (v & (1 << i))
				g_ += 1.0 / res[i];
		double rp = 1.0 / g_;
		s_att[v] = (int32_t)(rp / (rp + 10000) * 32768 + 0.5);
	}
	s_port_p1 = rbx_port_find("P1");
	s_port_coin = rbx_port_find("COIN");
	s_port_spare = rbx_port_find("SPARE");
	s_port_in0 = rbx_port_find("IN0");
	s_port_p2 = rbx_port_find("P2");
	// NVRAM (video CPU 0x8400-0x87ff; MAME default all 0)
	memset(s_nvram, 0, sizeof(s_nvram));
	s_nv_path[0] = 0;
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	if (dir) {
		snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
		FILE *f = fopen(s_nv_path, "rb");
		if (f) {
			if (fread(s_nvram, 1, sizeof(s_nvram), f) != sizeof(s_nvram))
				memset(s_nvram, 0, sizeof(s_nvram));
			fclose(f);
		}
	}
	if (rbx_video_setup(SCREEN_W, SCREEN_H, 2048) || rbx_audio_setup(SND_RATE)) {
		qx_exit();
		return -1;
	}
	s_setup = 0;
	{
		int blank = 1;
		for (size_t i = 0; i < sizeof(s_nvram); i++)
			blank &= s_nvram[i] == 0;
		s_setup = blank;   // (no settings yet: the operator's setup, in_coin)
	}
	pia_setup();
	map_cpus();
	qx_reset();
	return 0;
}

static void qx_exit(void)
{
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(s_nvram, 1, sizeof(s_nvram), f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	free(s_vram);
	free(s_snap);
	free(s_main_dec);
	free(s_video_dec);
	s_vram = NULL;
	s_snap = NULL;
	s_main_dec = s_video_dec = NULL;
}

static inline void sync_others(int64_t t)
{
	m6809_run_until(&s_video, t);
	m6800_run_until(&s_audio, t * 73728 / 100000);
}

static void qx_frame(void)
{
	if (s_setup && ++s_setup > (SETUP_PRESSES + 1) * SETUP_GAP) {   // (the setup done: the reset it asks for)
		s_setup = 0;
		qx_reset();
	}
	s_st.captured = 0;
	s_st.frame_frac += MAIN_CLOCK;
	int64_t n = (int64_t)(s_st.frame_frac / 60);
	s_st.frame_frac -= (uint64_t)n * 60;
	s_st.frame_end += n;
	while (s_main.cycles < s_st.frame_end) {
		int64_t lim = s_st.next_line < s_st.frame_end ? s_st.next_line : s_st.frame_end;
		while (s_main.cycles < lim) {
			m6809_step(&s_main);
			sync_others(s_main.cycles);
		}
		if (s_main.cycles >= s_st.next_line) {
			scanline();
			s_st.next_line += line_cycles();
		}
	}
	snd_update(s_main.cycles);
	if (!s_st.captured) {   // no VSYNC in this 1/60 s: show the last picture again
		rbx_video_wait_idle();
		rbx_video_frame_done();
	}
}

static int qx_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "main", &s_main, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "video", &s_video, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "audio", &s_audio, M6800_STATE_SIZE);
	char tag[8];
	for (int i = 0; i < 3; i++) {
		snprintf(tag, sizeof(tag), "pia%d", i);
		rc |= rbx_state_io(ctx, tag, &s_pia[i], PIA6821_STATE_SIZE);
		snprintf(tag, sizeof(tag), "spia%d", i);
		rc |= rbx_state_io(ctx, tag, &s_sndpia[i], PIA6821_STATE_SIZE);
	}
	rc |= rbx_state_io(ctx, "crtc", &s_crtc, sizeof(s_crtc));
	rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "mainram", s_mainram, sizeof(s_mainram));
	rc |= rbx_state_io(ctx, "nvram", s_nvram, sizeof(s_nvram));
	rc |= rbx_state_io(ctx, "pal", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "audram", s_audram, sizeof(s_audram));
	rc |= rbx_state_io(ctx, "vram", s_vram, 0x10000);
	rc |= rbx_state_io(ctx, "st", &s_st, sizeof(s_st));
	if (rc)
		return -1;
	if (load) {
		map_vram();
		map_videobank();
	}
	return 0;
}

static int qx_save(void *ctx) { return qx_state(ctx, 0); }
static int qx_load(void *ctx) { return qx_state(ctx, 1); }
static int qx_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_qix = { "Qix", qx_init, qx_exit, qx_reset, qx_frame, qx_save, qx_load, qx_supported, qx_render };
