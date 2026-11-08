/*  Konami Juno First. Ported from MAME src/mame/konami/junofrst.cpp (BSD-3-Clause, copyright-holders: Chris Hardy),
    the video of src/mame/konami/tutankhm_v.cpp (BSD-3-Clause, copyright-holders: Mirko Buffoni) and the palette's
    resistor networks of src/emu/video/resnet.cpp (BSD-3-Clause, copyright-holders: Couriersud). A KONAMI1 at
    1.536 MHz with a 4 KB ROM bank, a 256x256 4-bit bitmap (the left 192 columns scrolled) with a 16x16 blitter from
    its own ROMs, 16 colours in RAM, on a 768-wide screen (3 pixels a bitmap column; the stars never enabled); the
    sound: a Z80 at 1.79 MHz with an AY-3-8910 and an I8039 at 8 MHz on a DAC, a frame behind on the worker (the AY's
    RC filters, port B).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../sound/filter_rc.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12)
#define Z80_CLOCK 1789772u     // (14.318 MHz / 8; the AY too)
#define MCS_CYCLES 533333u     // (the I8039 at 8 MHz: 15 clocks a machine cycle)
#define SND_RATE 48000
#define PIX_CLOCK 18432000u    // (set_raw(GALAXIAN_PIXEL_CLOCK, 1152, 0, 768, 264, 16, 240))
#define HTOTAL 1152
#define LINES 264
#define VIS_W 768
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x20             // (0-15 the palette RAM, 16 the black of the fill)

static uint8_t *s_rom, *s_zrom, *s_drom, *s_blit;
static uint32_t s_rom_size, s_zrom_size, s_drom_size, s_blit_size;
static uint32_t s_raw_rgb[256];
static m6809_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static mcs48 s_mcs;
static ay8910_t s_ay;
static int s_port_dsw[2], s_port_p[2], s_port_system;

static struct {
	uint8_t vram[0x8000];    // 0000-7fff
	uint8_t pal[16];         // 8000-800f
	uint8_t ram[0xf00];      // 8100-8fff
	uint8_t blitter[4];      // 8070-8073
	uint8_t latch, bank, irq_enable, irq_toggle, last_irq;
	uint8_t zram[0x400], soundlatch, soundlatch2, dac, i8039_status, z80_irq;
	int32_t mcs_left, hp_x, hp_y;   // (the output's high-pass: the DAC idles at 0)
	filter_rc_t flt[3];             // (the AY channels' RC filters, port B)
	int32_t div_acc, div_n;         // (the filtered native samples averaged four to one)
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, mcs_frac, ay_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x8000], pal[16], scroll, flipx, flipy; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	s_vs.scroll = 0;   // (no scroll register on this board: m_scroll not found)
	s_vs.flipx = (t.latch >> 4) & 1;
	s_vs.flipy = (t.latch >> 5) & 1;
}

static void jf_render(void)   // (screen_update_scramble, the stars off)
{
	for (int i = 0; i < 16; i++)
		rbx_video.palette[i] = s_raw_rgb[s_vs.pal[i]];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int xorx = s_vs.flipx ? 255 : 0, xory = s_vs.flipy ? 255 : 0;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		const uint8_t effy = (uint8_t)(y ^ xory);
		for (int x = 0; x < 256; x++) {
			const int effx = x ^ xorx;
			const uint8_t v = (uint8_t)(s_vs.vram[effy * 128 + effx / 2] >> (4 * (effx & 1)) & 0x0f);
			d[3 * x] = d[3 * x + 1] = d[3 * x + 2] = v;
		}
	}
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch and the IRQ queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;
static int32_t s_out[2048];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static void q_put(int kind, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

// ---- the main CPU's bus (main_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static void set_bank(void)
{
	const uint32_t base = 0x10000 + (uint32_t)t.bank * 0x1000;
	for (int p = 0; p < 16; p++)
		s_cpu.rpage[0x90 + p] = s_rom + (base + (uint32_t)p * 256) % s_rom_size;
}

static void blitter_w(int offset, uint8_t v)   // (a 16x16 nibble block from the blitter ROMs: copied, or cleared)
{
	t.blitter[offset] = v;
	if (offset != 3)
		return;
	uint32_t src = (((uint32_t)t.blitter[2] << 8) | t.blitter[3]) & 0xfffc;
	uint32_t dest = ((uint32_t)t.blitter[0] << 8) | t.blitter[1];
	const int copy = t.blitter[3] & 1;
	for (int i = 0; i < 16; i++) {
		for (int j = 0; j < 16; j++) {
			const uint32_t so = src >> 1;
			const uint8_t b = so < s_blit_size ? s_blit[so] : 0;
			uint8_t data = (src & 1) ? (b & 0x0f) : (uint8_t)(b >> 4);
			src++;
			if (data) {
				if (!copy)
					data = 0;
				const uint32_t o = dest >> 1;
				if (o < sizeof(t.vram)) {
					if (dest & 1)
						t.vram[o] = (uint8_t)((t.vram[o] & 0x0f) | (data << 4));
					else
						t.vram[o] = (uint8_t)((t.vram[o] & 0xf0) | data);
				}
			}
			dest++;
		}
		dest += 240;
	}
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x8000)
		return t.vram[a];
	if (a < 0x8010)
		return t.pal[a & 0x0f];
	switch (a) {
	case 0x8010: return port(s_port_dsw[1]);
	case 0x8020: return port(s_port_system);
	case 0x8024: return port(s_port_p[0]);
	case 0x8028: return port(s_port_p[1]);
	case 0x802c: return port(s_port_dsw[0]);
	}
	if (a >= 0x8100 && a < 0x9000)
		return t.ram[a - 0x8100];
	if (a >= 0x9000 && a < 0xa000)
		return s_rom[(0x10000 + (uint32_t)t.bank * 0x1000 + (a & 0xfff)) % s_rom_size];
	if (a >= 0xa000)
		return s_rom[a % s_rom_size];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x8000)
		t.vram[a] = v;
	else if (a < 0x8010)
		t.pal[a & 0x0f] = v;
	else if (a >= 0x8030 && a < 0x8038) {   // (the LS259: Q0 the IRQ enable, Q4 / Q5 the flips)
		const int bit = a & 7, now = v & 1;
		t.latch = (uint8_t)((t.latch & ~(1 << bit)) | (now << bit));
		if (bit == 0) {
			t.irq_enable = (uint8_t)now;
			if (!now)
				m6809_set_irq(&s_cpu, 0);
		}
	} else if (a == 0x8040) {   // (sh_irqtrigger_w: 0 then 1)
		if (t.last_irq == 0 && v == 1)
			q_put(Q_IRQ, 0);
		t.last_irq = v;
	} else if (a == 0x8050)
		q_put(Q_LATCH, v);
	else if (a == 0x8060) {
		t.bank = v & 0x0f;
		set_bank();
	} else if (a >= 0x8070 && a < 0x8074)
		blitter_w(a & 3, v);
	else if (a >= 0x8100 && a < 0x9000)
		t.ram[a - 0x8100] = v;
}

// ---- the sound board (audio_map) ----
// filter_w (port B): each channel's capacitors, 2 bits a channel (0.047 uF and 0.22 uF), at the AY's native rate
static void ay_port_b(void *user, uint8_t v)
{
	(void)user;
	for (int i = 0; i < 3; i++, v >>= 2) {
		const double c = ((v & 1) ? 47000e-12 : 0) + ((v & 2) ? 220000e-12 : 0);
		filter_rc_lowpass_3r(&t.flt[i], 1000, 2200, 200, c, Z80_CLOCK / 8.0);
	}
}

static uint8_t ay_port_a(void *u)   // (portA_r: a timer from the Z80's clock, the I8039's status)
{
	(void)u;
	return (uint8_t)((((s_z.cyc / (1024 / 2)) & 0x0f) << 4) | t.i8039_status);
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x1000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a >= 0x2000 && a < 0x2400)
		return t.zram[a - 0x2000];
	if (a == 0x3000)
		return t.soundlatch;
	if (a == 0x4001)
		return ay8910_data_r(&s_ay);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x2000 && a < 0x2400)
		t.zram[a - 0x2000] = v;
	else if (a == 0x4000)
		ay8910_address_w(&s_ay, v);
	else if (a == 0x4002)
		ay8910_data_w(&s_ay, v);
	else if (a == 0x5000)
		t.soundlatch2 = v;
	else if (a == 0x6000)
		mcs48_set_irq(&s_mcs, 1);   // (i8039_irq_w: ASSERT_LINE)
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- the I8039 (every I/O read the second latch; P1 the DAC, P2 bit 7 low clears the IRQ, bits 4-6 the status) ----
static uint8_t mcs_ext_r(void *u, uint8_t a) { (void)u; (void)a; return t.soundlatch2; }
static void mcs_port_w(void *u, int p, uint8_t v)
{
	(void)u;
	if (p == 1)
		t.dac = v;
	else if (p == 2) {
		if (!(v & 0x80))
			mcs48_set_irq(&s_mcs, 0);
		t.i8039_status = (v & 0x70) >> 4;
	}
}

static void sound_run(int n)   // (a line's samples: the Z80, the I8039, the AY and the DAC)
{
	if (n <= 0)
		return;
	t.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	int line = t.z80_irq;
	t.z_end += cyc;
	z80_run(&s_z, (unsigned long)t.z_end, &line, 1);
	t.z80_irq = (uint8_t)line;
	t.mcs_frac += (uint64_t)MCS_CYCLES * (uint64_t)n;
	const int mc = (int)(t.mcs_frac / SND_RATE);
	t.mcs_frac -= (uint64_t)mc * SND_RATE;
	t.mcs_left += mc;
	if (t.mcs_left > 0)
		t.mcs_left -= mcs48_execute(&s_mcs, t.mcs_left);
	if (n > 64)
		n = 64;
	int32_t a1[96];
	memset(a1, 0, sizeof(a1));
	t.ay_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const uint32_t ac = (uint32_t)(t.ay_frac / SND_RATE);
	t.ay_frac -= (uint64_t)ac * SND_RATE;
	// (each channel at 0.30 through its filter at the AY's own rate, then four samples to one as before)
	static int16_t ch[3 * 512];
	const int nn = ay8910_render_ch(&s_ay, ac, ch, 512);
	int n1 = 0;
	for (int i = 0; i < nn; i++) {
		t.div_acc += (filter_rc_step(&t.flt[0], ch[3 * i]) + filter_rc_step(&t.flt[1], ch[3 * i + 1]) +
		              filter_rc_step(&t.flt[2], ch[3 * i + 2])) * 77 >> 8;
		if (++t.div_n == 4) {
			if (n1 < 96)
				a1[n1++] = t.div_acc / 4;
			t.div_acc = 0;
			t.div_n = 0;
		}
	}
	const int32_t dac = ((int32_t)t.dac - 128) * 256 / 2;   // (the DAC's range -1 to 1, 0.25; as Mega Zone's levels)
	for (int i = 0; i < n && s_out_pos < 2048; i++) {
		const int32_t x = (n1 > 0 ? a1[i * n1 / n] : 0) + dac;
		t.hp_y = x - t.hp_x + t.hp_y - t.hp_y / 225;
		t.hp_x = x;
		s_out[s_out_pos++] = t.hp_y;
	}
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *e = &s_q[b][i];
		switch (e->kind) {
		case Q_ADV: sound_run((int)e->v); break;
		case Q_LATCH: t.soundlatch = (uint8_t)e->v; break;
		case Q_IRQ: t.z80_irq = 1; break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put(s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void jf_exit(void);
static void jf_reset(void);

// MAME compute_resistor_weights(0, 224, -1.0, ...): each resistor's output alone, all the networks scaled so that
// the one with the greatest output gives 224 (RGB_MAXIMUM)
static double raw_weights(const int *res, int n, int pulldown, double *w)
{
	double sum = 0;
	for (int k = 0; k < n; k++) {
		double r0 = pulldown ? 1.0 / pulldown : 1.0 / 1e12, r1 = 1.0 / 1e12;
		for (int j = 0; j < n; j++) {
			if (j == k)
				r1 += 1.0 / res[j];
			else
				r0 += 1.0 / res[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		w[k] = 255.0 * r0 / (r1 + r0);
		sum += w[k];
	}
	return sum;
}

static void mcs_setup(void)
{
	s_mcs.rom = s_drom;
	s_mcs.rom_mask = (uint16_t)(s_drom_size - 1);
	s_mcs.userdata = NULL;
	s_mcs.ext_r = mcs_ext_r;
	s_mcs.port_w = mcs_port_w;
}

static int jf_init(const rbx_game *g)
{
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_drom = rbx_region("mcu", &s_drom_size);
	s_blit = rbx_region("blitrom", &s_blit_size);
	if (!s_rom || s_rom_size < 0x20000 || !s_zrom || !s_drom || !s_blit) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (raw_to_rgb_func: the red and green 1k / 470 / 220, the blue 470 / 220, all on a 470 pull-down)
	static const int res_rg[3] = { 1000, 470, 220 }, res_b[2] = { 470, 220 };
	double wr[3], wb[2];
	const double srg = raw_weights(res_rg, 3, 470, wr), sb = raw_weights(res_b, 2, 470, wb);
	const double scale = 224.0 / (srg > sb ? srg : sb);
	for (int i = 0; i < 256; i++) {
		const int r = (int)(scale * (wr[0] * (i & 1) + wr[1] * ((i >> 1) & 1) + wr[2] * ((i >> 2) & 1)) + 0.5);
		const int gg = (int)(scale * (wr[0] * ((i >> 3) & 1) + wr[1] * ((i >> 4) & 1) + wr[2] * ((i >> 5) & 1)) + 0.5);
		const int b = (int)(scale * (wb[0] * ((i >> 6) & 1) + wb[1] * ((i >> 7) & 1)) + 0.5);
		s_raw_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, Z80_CLOCK, AY8910_LEGACY_OUTPUT, 77, 4, 0);   // (0.30: render_ch, filtered)
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		jf_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	jf_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void jf_exit(void) { rbx_aux_set_job(NULL); }

static void jf_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	ay8910_reset(&s_ay);
	for (int i = 0; i < 3; i++)
		t.flt[i].k = 0x10000;
	s_ay.port_a_read = ay_port_a;
	s_ay.port_b_write = ay_port_b;
	s_ay.user = NULL;
	mcs48_init(&s_mcs, MCS48_I8039);
	mcs_setup();
	mcs48_reset(&s_mcs);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.konami1 = 1;
	for (int p = 0; p < 0x80; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.vram + p * 256;
	for (int p = 0x81; p < 0x90; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x81) * 256;
	set_bank();
	for (int p = 0xa0; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x10; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 4; p++)
		s_zmap[0x20 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void jf_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, _30hz_irq every other frame)
			capture();
			rbx_video_frame_done();
			t.irq_toggle ^= 1;
			if (t.irq_toggle && t.irq_enable)
				m6809_set_irq(&s_cpu, 1);
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		t.cpu_end += cyc;
		m6809_run_until(&s_cpu, t.cpu_end);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int jf_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "jf_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "jf", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "jf_mcs", (uint8_t *)&s_mcs + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	rc |= rbx_state_io(ctx, "jf_ay", &s_ay, ay8910_state_size());
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "jf_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "jf_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "jf_z80", &z, sizeof(z));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		set_bank();
		mcs_setup();
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int jf_save(void *ctx) { return jf_state(ctx, 0); }
static int jf_load(void *ctx) { return jf_state(ctx, 1); }
static int jf_supported(const char *machine) { return !strcmp(machine, "junofrst"); }

const rbx_driver rbx_drv_junofrst = { "Konami Juno First", jf_init, jf_exit, jf_reset, jf_frame, jf_save, jf_load,
                                      jf_supported, jf_render, NULL };
