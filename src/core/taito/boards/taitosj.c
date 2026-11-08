/*  Taito SJ System, ported from MAME src/mame/taito/taitosj.cpp, taitosj_m.cpp, taitosj_v.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria) and taitosjsec.cpp (BSD-3-Clause, copyright-holders: Vas Crabb).
    Memory maps, the machine configurations (nomcu / mcu / kikstart), the security MCU interface (M68705P5 with
    bus request), the protection hacks (spacecr, alpine, alpinea, junglehbr), the layer / sprite drawing, the
    priority PROM and the hardware collision detection follow MAME. Sound: 4 x AY-3-8910 + 8-bit DAC with the
    resistor-ladder volume (dacvol), the "sound stop" bit of the bank register mutes everything as in MAME.
    The collision registers are computed on the main side at vblank (game logic depends on them); drawing reads
    only the snapshot.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/m6805/m68705.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

enum { SJ_NOMCU, SJ_MCU, SJ_KIKSTART };

// MAME set_raw(12 MHz / 2, 384, 0, 256, 264, 16, 240)
#define LINES 264
#define VIS_Y0 16
#define VIS_Y1 239
#define MAIN_LINE 256      // 4 MHz
#define AUDIO_LINE 192     // 3 MHz
#define MCU_LINE 48        // 3 MHz / 4
#define AY_LINE 96         // 1.5 MHz
#define SLICES 4
#define AUDIO_IRQ_PERIOD 81920   // 3 MHz / (12 MHz / (2*4*16*16*10*16))
#define TRANSPARENT 0x40

typedef struct {
	z80 z;
	int irq;
} zcpu;

static int s_kind, s_alpine, s_spacecr, s_junglhbr;
static uint8_t *s_main_rom, *s_audio_rom, *s_gfxrom, *s_prom;
static uint32_t s_main_size, s_audio_size, s_gfx_size;
static zcpu s_main, s_audio;
static int s_bank;
// memory
static uint8_t s_ram[0x800], s_charram[0x3000], s_c000[0x1000], s_d000[0x100], s_spriteram[0x100], s_palram[0x80];
static uint8_t s_kikscroll[0x800], s_audio_ram[0x400];
static uint8_t s_scroll[6], s_colorbank[2], s_gfxptr[2], s_collision[4], s_video_mode, s_priority;
static uint8_t s_chardirty[2][256 / 8], s_sprdirty[2][64 / 8];
// sound / misc
static uint8_t s_soundlatch, s_input4_f0, s_gears[2], s_spacecr_prot, s_protection;
static int s_soundlatch_flag, s_sem2, s_nmi_mask_in, s_nmi_line, s_sound_on;
static uint32_t s_audio_irq_count;
static ay8910_t s_ay[4];
static uint8_t s_dac_data, s_dac_vol;
static int32_t s_dac_out;
// security MCU (taitosjsec, int_mode LATCH)
static m68705_t s_mcu;
static uint16_t s_addr;
static uint8_t s_mcu_data, s_host_data, s_read_data, s_pa_val, s_pb_val;
static int s_zaccept, s_zready, s_busak, s_busrq;
// video
static int s_draw_order[32][4];
static uint8_t *s_dec_char, *s_dec_spr;   // render side: [bank][code] pixels, one byte each
static uint8_t s_snap_charram[0x3000], s_snap_vram[0xc00], s_snap_colscroll[0x60], s_snap_spr[0x100], s_snap_pal[0x80];
static uint8_t s_snap_kik[0x200], s_snap_scroll[6], s_snap_colorbank[2], s_snap_mode, s_snap_prio;
static uint8_t s_snap_chardirty[2][256 / 8], s_snap_sprdirty[2][64 / 8];
// ports
static int s_p_in[5], s_p_dsw1, s_p_dsw2, s_p_dsw3, s_p_gear[2];
// audio mix
static int32_t *s_mix, *s_tmp;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done, s_slice_start;
static unsigned long s_cpu_start;
static int s_cpu_is_main;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	uint32_t clk = pos - s_snd_done;
	int room = s_mix_cap - s_mix_n;
	if (room <= 0) {
		s_snd_done = pos;
		return;
	}
	memset(s_tmp, 0, sizeof(int32_t) * (size_t)(ay8910_out_max(&s_ay[0], clk) < room ? ay8910_out_max(&s_ay[0], clk) : room));
	int n = ay8910_render(&s_ay[0], clk, s_tmp, room);
	for (int i = 1; i < 4; i++)
		ay8910_render(&s_ay[i], clk, s_tmp, room);
	int32_t *o = s_mix + s_mix_n;
	if (s_sound_on)
		for (int i = 0; i < n; i++)
			o[i] = s_tmp[i] + s_dac_out;
	else
		memset(o, 0, sizeof(int32_t) * (size_t)n);
	s_mix_n += n;
	s_snd_done = pos;
}

static void snd_sync(void)
{
	const zcpu *c = s_cpu_is_main ? &s_main : &s_audio;
	uint32_t len = (s_cpu_is_main ? MAIN_LINE : AUDIO_LINE) / SLICES;
	uint32_t done = (uint32_t)(c->z.cyc - s_cpu_start);
	if (done > len)
		done = len;
	snd_run_to(s_slice_start + (uint32_t)(AY_LINE / SLICES) * done / len);
}

static void update_dac(void)
{
	// DISCRETE_DAC_R1 ladder (680k .. 4.7k) driven by the inverted port B: volume 0..1 (3.4 V * 9637 / 32768)
	static const double r[8] = { 680e3, 330e3, 150e3, 82e3, 39e3, 20e3, 10e3, 4.7e3 };
	double on = 0, all = 0;
	for (int i = 0; i < 8; i++) {
		all += 1.0 / r[i];
		if ((s_dac_vol >> i) & 1)
			on += 1.0 / r[i];
	}
	double vol = 3.4 * (on / all) * 9637.0 / 32768.0;
	// DAC_8BIT_R2R with the range -vol .. +vol, route 0.15
	s_dac_out = (int32_t)(32767.0 * 0.15 * vol * (2.0 * s_dac_data / 255.0 - 1.0));
}

static void update_nmi(void)
{
	int m0 = s_nmi_mask_in && s_soundlatch_flag;   // INPUT_MERGER_ALL_HIGH
	int line = m0 || s_sem2;                        // INPUT_MERGER_ANY_HIGH
	if (line && !s_nmi_line)
		s_audio.z.nmi_pending = 1;
	s_nmi_line = line;
}

static uint8_t ay0_pa(void *u) { (void)u; return port8(s_p_dsw2); }
static uint8_t ay0_pb(void *u) { (void)u; return port8(s_p_dsw3); }
static void ay1_pa_w(void *u, uint8_t d) { (void)u; snd_sync(); s_dac_data = d; update_dac(); }
static void ay1_pb_w(void *u, uint8_t d) { (void)u; snd_sync(); s_dac_vol = (uint8_t)(d ^ 0xff); update_dac(); }   // 7416 inverter
static void ay2_pa_w(void *u, uint8_t d) { (void)u; s_input4_f0 = (uint8_t)(d >> 4); }
static void ay3_pb_w(void *u, uint8_t d) { (void)u; s_nmi_mask_in = (~d) & 1; update_nmi(); }

// ---- security MCU (taito_sj_security_mcu_device) ----
static uint8_t main_read(void *u, uint16_t a);
static void map_bank(void);
static void main_write(void *u, uint16_t a, uint8_t v);

static uint8_t bus_val(void)
{
	return (uint8_t)(((~s_pb_val & 0x02) ? s_host_data : 0xff) & s_pa_val & ((s_busak && (~s_pb_val & 0x20)) ? s_read_data : 0xff));
}

static uint8_t mcu_port_r(void *u, int n, uint8_t mask)
{
	(void)u; (void)mask;
	if (n == 0)
		return bus_val();
	if (n == 2)
		return (uint8_t)((s_zready ? 1 : 0) | (s_zaccept ? 2 : 0) | (s_busak ? 0 : 4));
	return 0xff;
}

static void mcu_port_w(void *u, int n, uint8_t data, uint8_t mask)
{
	(void)u; (void)mask;
	if (n == 0) {
		s_pa_val = data;
		if (~s_pb_val & 0x40)
			s_addr = (uint16_t)((s_addr & 0xff00) | bus_val());
		return;
	}
	if (n != 1)
		return;
	int inc = 0;
	uint8_t diff = s_pb_val ^ data;
	// (68INTRQ, bit 0: not supported by MAME either)
	uint8_t bv = bus_val();
	if (diff & data & 0x02) {   // 68LRD
		s_zready = 0;
		m68705_set_irq_line(&s_mcu, 0);
	}
	if (diff & data & 0x04) {   // 68LWR
		s_mcu_data = bv;
		s_zaccept = 0;
	}
	if (diff & 0x08) {          // BUSRQ (the Z80 is between instructions here: BUSAK follows at once)
		s_busrq = !(data & 0x08);
		s_busak = s_busrq;
	}
	if (diff & 0x10) {          // 68WRITE
		if (~data & 0x10)
			main_write(NULL, s_addr, bv);
		else if (data & 0x20)
			inc = 1;
	}
	if (diff & 0x20) {          // 68READ
		if (~data & 0x20)
			s_read_data = main_read(NULL, s_addr);
		else if (data & 0x10)
			inc = 1;
	}
	if (~data & 0x40)
		s_addr = (uint16_t)((s_addr & 0xff00) | bv);
	else if (inc)
		s_addr = (uint16_t)((s_addr & 0xff00) | ((s_addr + 1) & 0xff));
	if (~data & 0x80)
		s_addr = (uint16_t)((s_addr & 0x00ff) | (bv << 8));
	s_pb_val = data;
}

static uint8_t sec_data_r(int offset)
{
	if (offset & 1)
		return (uint8_t)((s_zaccept ? 0 : 2) | (s_zready ? 0 : 1));   // ZLSTATUS (unmapped bits read 0)
	s_zaccept = 1;   // ZLREAD
	return s_mcu_data;
}

static void sec_data_w(int offset, uint8_t data)
{
	if (offset & 1)
		return;   // ZINTRQ: int_mode WRITE only
	s_host_data = data;   // ZLWRITE
	s_zready = 1;
	m68705_set_irq_line(&s_mcu, 1);
}

// ---- main CPU ----
static void bankswitch_w(uint8_t data)
{
	snd_sync();
	s_sound_on = (data & 2) != 0;
	s_bank = (data >> 7) & 1;
	map_bank();
}

static uint8_t gfxrom_r(void)
{
	uint32_t offs = (uint32_t)(s_gfxptr[0] | s_gfxptr[1] << 8);
	uint8_t ret = (offs < 0x8000 && offs < s_gfx_size) ? s_gfxrom[offs] : 0;
	offs++;
	s_gfxptr[0] = (uint8_t)offs;
	s_gfxptr[1] = (uint8_t)(offs >> 8);
	return ret;
}

static void characterram_w(uint32_t offset, uint8_t data)
{
	if (s_junglhbr)
		data ^= 0xfc;
	if (s_charram[offset] == data)
		return;
	s_charram[offset] = data;
	int bank = offset >= 0x1800;
	int c = (offset / 8) & 0xff, sp = (offset / 32) & 0x3f;
	s_chardirty[bank][c >> 3] |= (uint8_t)(1 << (c & 7));
	s_sprdirty[bank][sp >> 3] |= (uint8_t)(1 << (sp & 7));
}

static uint8_t io_d4_r(int o)
{
	switch (o) {
	case 0: case 1: case 2: case 3: return s_collision[o];
	case 4: case 5: case 6: case 7: return gfxrom_r();
	case 8: return port8(s_p_in[0]);
	case 9: return port8(s_p_in[1]);
	case 10: return port8(s_p_dsw1);
	case 11: return (uint8_t)(port8(s_p_in[2]) | (s_alpine ? s_protection : 0));
	case 12: return port8(s_p_in[3]);
	case 13:
		rbx_port_custom(s_p_in[4], 0xf0, (uint32_t)s_input4_f0 << 4);
		return port8(s_p_in[4]);
	case 15: snd_sync(); return ay8910_data_r(&s_ay[0]);
	}
	return 0;
}

static void alpine_protection_w(uint8_t data)
{
	switch (data) {
	case 0x05: s_protection = 0x18; break;
	case 0x07: case 0x0c: case 0x0f: s_protection = 0x00; break;
	case 0x16: s_protection = 0x08; break;
	case 0x1d: s_protection = 0x18; break;
	default: s_protection = data; break;
	}
}

static void io_d5_w(int o, uint8_t v)
{
	if (o < 6) { s_scroll[o] = v; return; }
	switch (o) {
	case 6: case 7: s_colorbank[o - 6] = v; return;
	case 8: memset(s_collision, 0, 4); return;
	case 9: case 10: s_gfxptr[o - 9] = v; return;
	case 11: s_soundlatch = v; s_soundlatch_flag = 1; update_nmi(); return;
	case 12: s_sem2 = v & 1; update_nmi(); return;
	case 13: return;   // watchdog
	case 14:
		bankswitch_w(v);
		if (s_alpine == 2)
			s_protection = v >> 2;
		return;
	case 15:
		if (s_alpine == 1)
			alpine_protection_w(v);
		return;
	}
}

static uint8_t kik_gear(int p)
{
	uint8_t g = port8(s_p_gear[p]);
	if (g & 1) s_gears[p] = 2;
	if (g & 2) s_gears[p] = 3;
	if (g & 4) s_gears[p] = 1;
	return s_gears[p];
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x6000) return s_main_rom[a];
	if (a < 0x8000) {
		uint32_t o = s_bank ? 0x10000u + (a - 0x6000u) : a;
		return o < s_main_size ? s_main_rom[o] : 0xff;
	}
	if (a < 0x8800) return s_ram[a & 0x7ff];
	if (a < 0x9000) {
		if (s_kind == SJ_KIKSTART) {
			if (a < 0x8802) return sec_data_r(a & 1);
			return 0;
		}
		if (s_kind == SJ_MCU) return sec_data_r(a & 1);
		return (a & 1) ? 0xff : 0;   // fake_status_r / fake_data_r
	}
	if (a < 0xc000) return 0;   // character RAM is write only
	if (a < 0xd000) return s_c000[a & 0xfff];
	if (s_kind == SJ_KIKSTART) {
		if (a >= 0xd800 && a < 0xe000) return s_kikscroll[a & 0x7ff];
		if (a >= 0xd100 && a < 0xd200) return s_spriteram[a & 0xff];
		if (a >= 0xd200 && a < 0xd280) return s_palram[a & 0x7f];
		if (a >= 0xd400 && a < 0xd404) return s_collision[a & 3];
		if (a == 0xd404) return gfxrom_r();
		if ((a & 0xff00) == 0xd400 && (a & 0x0f) >= 8 && (a & 0x0f) <= 13) {
			int o = a & 0x0f;
			if (o == 11) return port8(s_p_in[2]);
			if (o == 12 || o == 13) {
				int p = o - 12;
				int pi = s_p_in[3 + p];
				if (port8(s_p_dsw3) & 0x08)
					rbx_port_custom(pi, 0x03, kik_gear(p));
				else
					rbx_port_custom(pi, 0x03, 0);   // (active-low unknown bits: inverted to 1)
				return port8(pi);
			}
			return io_d4_r(o);
		}
		if (a == 0xd40f) { snd_sync(); return ay8910_data_r(&s_ay[0]); }
		if (a >= 0xe000 && a < 0xf000) return a < s_main_size ? s_main_rom[a] : 0xff;
		return 0;
	}
	if (a < 0xd100) return s_d000[a & 0xff];
	if (a < 0xd200) return s_spriteram[a & 0xff];
	if (a < 0xd300) return s_palram[a & 0x7f];
	if (a < 0xd400) return 0;
	if (a < 0xd500) {
		if (s_spacecr && a == 0xd48b) {
			s_spacecr_prot ^= 0xff;
			return s_spacecr_prot;
		}
		return io_d4_r(a & 0x0f);
	}
	if (a < 0xe000) return 0;
	return a < s_main_size ? s_main_rom[a] : 0xff;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a < 0x8800) { s_ram[a & 0x7ff] = v; return; }
	if (a < 0x9000) {
		if (s_kind == SJ_KIKSTART) {
			if (a < 0x8802) sec_data_w(a & 1, v);
			return;   // (8a00-8a5f: column scroll, unused by the kikstart drawing)
		}
		if (s_kind == SJ_MCU) sec_data_w(a & 1, v);
		return;
	}
	if (a < 0xc000) { characterram_w(a - 0x9000u, v); return; }
	if (a < 0xd000) { s_c000[a & 0xfff] = v; return; }
	if (s_kind == SJ_KIKSTART) {
		if (a < 0xd002) { s_colorbank[a & 1] = v; return; }
		if (a < 0xd008) { s_scroll[a - 0xd002] = v; return; }
		if (a >= 0xd100 && a < 0xd200) { s_spriteram[a & 0xff] = v; return; }
		if (a >= 0xd200 && a < 0xd280) { s_palram[a & 0x7f] = v; return; }
		if (a == 0xd300) { s_priority = v; return; }
		if (a == 0xd40e || a == 0xd40f) { snd_sync(); ay8910_write(&s_ay[0], a & 1, v); return; }
		if (a >= 0xd508 && a <= 0xd50e) { io_d5_w(a & 0x0f, v); return; }
		if (a == 0xd600) { s_video_mode = v; return; }
		if (a >= 0xd800 && a < 0xe000) { s_kikscroll[a & 0x7ff] = v; return; }
		return;
	}
	if (a < 0xd100) { s_d000[a & 0xff] = v; return; }
	if (a < 0xd200) { s_spriteram[a & 0xff] = v; return; }
	if (a < 0xd300) { s_palram[a & 0x7f] = v; return; }
	if (a < 0xd400) { s_priority = v; return; }
	if (a < 0xd500) {
		if ((a & 0x0e) == 0x0e) { snd_sync(); ay8910_write(&s_ay[0], a & 1, v); }
		return;
	}
	if (a < 0xd600) { io_d5_w(a & 0x0f, v); return; }
	if (a < 0xd700) { s_video_mode = v; return; }
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (a < 0x4800) return a < 0x4400 ? s_audio_ram[a & 0x3ff] : 0;
	if (a < 0x5000) {
		int o = a & 7;
		if (o == 1) { snd_sync(); return ay8910_data_r(&s_ay[1]); }
		if (o == 3) { snd_sync(); return ay8910_data_r(&s_ay[2]); }
		if ((a & 5) == 5) { snd_sync(); return ay8910_data_r(&s_ay[3]); }
		return 0;
	}
	if (a < 0x5800) {
		switch (a & 3) {
		case 0:
			s_soundlatch_flag = 0;
			update_nmi();
			return s_soundlatch;
		case 1:
			return (uint8_t)((s_soundlatch_flag ? 8 : 0) | (s_sem2 ? 4 : 0) | 3);
		}
		return 0;
	}
	if (a >= 0xe000 && a < 0xf000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x4000) return;
	if (a < 0x4800) { if (a < 0x4400) s_audio_ram[a & 0x3ff] = v; return; }
	if (a < 0x5000) {
		int o = a & 7;
		snd_sync();
		if (o < 2) ay8910_write(&s_ay[1], o & 1, v);
		else if (o < 4) ay8910_write(&s_ay[2], o & 1, v);
		else ay8910_write(&s_ay[3], o & 1, v);
		return;
	}
	if (a < 0x5800) {
		switch (a & 3) {
		case 0: s_soundlatch &= 0x7f; break;
		case 1: s_sem2 = 0; update_nmi(); break;
		}
	}
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = no_in;
	c->z.port_out = no_out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
	c->irq = 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	z80_run(&c->z, c->z.cyc + (unsigned long)cycles, &c->irq, 1);   // (HOLD_LINE: the line cleared when taken)
}

// ---- Z80 read maps: ROM / RAM pages read straight (the same bytes the read handlers return) ----
static const uint8_t *s_main_map[256], *s_audio_map[256];

static void map_bank(void)   // the main CPU's 6000-7fff window
{
	for (uint32_t a = 0x6000; a < 0x8000; a += 0x100) {
		const uint32_t o = s_bank ? 0x10000u + (a - 0x6000u) : a;
		s_main_map[a >> 8] = o + 0x100 <= s_main_size ? s_main_rom + o : NULL;
	}
}

static void map_build(void)
{
	memset(s_main_map, 0, sizeof(s_main_map));
	memset(s_audio_map, 0, sizeof(s_audio_map));
	for (uint32_t a = 0; a < 0x6000; a += 0x100)
		s_main_map[a >> 8] = s_main_rom + a;
	map_bank();
	for (uint32_t a = 0x8000; a < 0x8800; a += 0x100)
		s_main_map[a >> 8] = s_ram + (a & 0x7ff);
	for (uint32_t a = 0xc000; a < 0xd000; a += 0x100)
		s_main_map[a >> 8] = s_c000 + (a & 0xfff);
	for (uint32_t a = 0; a < 0x4000 && a + 0x100 <= s_audio_size; a += 0x100)
		s_audio_map[a >> 8] = s_audio_rom + a;
	for (uint32_t a = 0x4000; a < 0x4400; a += 0x100)
		s_audio_map[a >> 8] = s_audio_ram + (a & 0x3ff);
	s_main.z.read_map = s_main_map;
	s_audio.z.read_map = s_audio_map;
}

// ---- pixels (shared by the collision check and the drawing) ----
static inline int char_pen(const uint8_t *cr, int bank, int code, int x, int y)
{
	const uint8_t *p = cr + bank * 0x1800 + code * 8 + y;
	return ((p[0x1000] >> x) & 1) << 2 | ((p[0x800] >> x) & 1) << 1 | ((p[0] >> x) & 1);
}
static inline int spr_pen(const uint8_t *cr, int bank, int code, int x, int y)
{
	const uint8_t *p = cr + bank * 0x1800 + code * 32 + (y & 7) + ((y & 8) ? 16 : 0) + ((x & 8) ? 8 : 0);
	x &= 7;
	return ((p[0x1000] >> x) & 1) << 2 | ((p[0x800] >> x) & 1) << 1 | ((p[0] >> x) & 1);
}

typedef struct {
	const uint8_t *charram, *vram, *colscroll, *spr, *kik, *scroll, *colorbank;
	uint8_t mode;
	int kikstart;
} vstate;

// scroll of a layer: per column (normal) or per row (kikstart)
typedef struct {
	int sx, sy;
	int col[32];
	int row[256];
} lscroll;

static void layer_scroll(const vstate *v, int which, lscroll *ls)
{
	static const int fudge1[3] = { 3, 1, -1 };
	static const int fudge2[3] = { 8, 10, 12 };
	const int fx = v->mode & 1, fy = v->mode & 2;
	if (!v->kikstart) {
		int sx = v->scroll[2 * which];
		if (fx)
			sx = (sx & 0xf8) + ((sx + fudge1[which]) & 7) + fudge2[which];
		else
			sx = -(sx & 0xf8) + ((sx + fudge1[which]) & 7) + fudge2[which];
		ls->sx = sx;
		for (int i = 0; i < 32; i++) {
			if (fy)
				ls->col[31 - i] = v->colscroll[32 * which + i] + v->scroll[2 * which + 1];
			else
				ls->col[i] = -v->colscroll[32 * which + i] - v->scroll[2 * which + 1];
		}
	} else {
		ls->row[0] = 0;   // (left unset by MAME, row 0 is not visible)
		for (int i = 1; i < 256; i++) {
			if (fy) {
				switch (which) {
				case 0: ls->row[256 - i] = 0; break;
				case 1: ls->row[256 - i] = v->kik[i] + ((v->scroll[2] + 0x0a) & 0xff); break;
				default: ls->row[256 - i] = v->kik[0x100 + i] + ((v->scroll[4] + 0x0c) & 0xff); break;
				}
			} else {
				switch (which) {
				case 0: ls->row[i] = 0; break;
				case 1: ls->row[i] = 0xff - v->kik[i - 1] - ((v->scroll[2] - 0x10) & 0xff); break;
				default: ls->row[i] = 0xff - v->kik[0x100 + i - 1] - ((v->scroll[4] - 0x12) & 0xff); break;
				}
			}
		}
		ls->sy = v->scroll[2 * which + 1];
	}
}

static inline void layer_bank_color(const vstate *v, int which, int *bank, int *color)
{
	switch (which) {
	case 0: *bank = (v->colorbank[0] & 0x08) != 0; *color = v->colorbank[0] & 7; break;
	case 1: *bank = (v->colorbank[0] & 0x80) != 0; *color = (v->colorbank[0] >> 4) & 7; break;
	default: *bank = (v->colorbank[1] & 0x08) != 0; *color = v->colorbank[1] & 7; break;
	}
}

// source coordinates in the 256x256 layer bitmap of screen pixel (x, y)
static inline void layer_src(const vstate *v, const lscroll *ls, int x, int y, int *lx, int *ly)
{
	if (!v->kikstart) {
		*lx = (x - ls->sx) & 0xff;
		*ly = (y - ls->col[*lx >> 3]) & 0xff;
	} else {
		*ly = (y - ls->sy) & 0xff;
		*lx = (x - ls->row[*ly]) & 0xff;
	}
}

// pen (0 = transparent) of the layer bitmap (drawn with the global flip)
static inline int layer_pen_raw(const vstate *v, int which, int bank, int lx, int ly)
{
	int u = (v->mode & 1) ? 255 - lx : lx, w = (v->mode & 2) ? 255 - ly : ly;
	int code = v->vram[which * 0x400 + (w >> 3) * 32 + (u >> 3)];
	return char_pen(v->charram, bank, code, u & 7, w & 7);
}

static inline int sprite_xy(const uint8_t *spr, int which, uint8_t *sx, uint8_t *sy)
{
	*sx = (uint8_t)(spr[which * 4] - 1);
	*sy = (uint8_t)(240 - spr[which * 4 + 1]);
	return *sy < 240;
}

// ---- collision detection (main side, MAME check_collision) ----
static int sprite_pix(const vstate *v, const uint8_t *s, int x, int y, int flipx, int flipy)
{
	int bank = (s[3] & 0x40) != 0;
	return spr_pen(v->charram, bank, s[3] & 0x3f, flipx ? 15 - x : x, flipy ? 15 - y : y);
}

static int sprite_sprite_bitpattern(const vstate *v, int sx1, int sy1, int w1, int sx2, int sy2, int w2)
{
	int minx, miny;
	if (sx1 < sx2) { sx2 -= sx1; sx1 = 0; minx = sx2; } else { sx1 -= sx2; sx2 = 0; minx = sx1; }
	if (sy1 < sy2) { sy2 -= sy1; sy1 = 0; miny = sy2; } else { sy1 -= sy2; sy2 = 0; miny = sy1; }
	const uint8_t *s1 = v->spr + w1 * 4, *s2 = v->spr + w2 * 4;
	for (int y = miny; y < 16; y++)
		for (int x = minx; x < 16; x++) {
			int x1 = x - sx1, y1 = y - sy1, x2 = x - sx2, y2 = y - sy2;
			if (x1 < 0 || x1 > 15 || y1 < 0 || y1 > 15 || x2 < 0 || x2 > 15 || y2 < 0 || y2 > 15)
				continue;
			if (sprite_pix(v, s1, x1, y1, s1[2] & 1, s1[2] & 2) && sprite_pix(v, s2, x2, y2, s2[2] & 1, s2[2] & 2))
				return 1;
		}
	return 0;
}

static void check_collision(void)
{
	vstate v = { s_charram, s_c000 + 0x400, s_d000, s_spriteram + ((s_video_mode & 4) ? 0x80 : 0), s_kikscroll, s_scroll,
	             s_colorbank, s_video_mode, s_kind == SJ_KIKSTART };
	if (!(v.mode & 0x80))
		return;
	// sprite / sprite
	for (int w1 = 0; w1 < 0x20; w1++) {
		uint8_t sx1, sy1;
		if (w1 >= 0x10 && w1 <= 0x17) continue;
		if (!sprite_xy(v.spr, w1, &sx1, &sy1)) continue;
		for (int w2 = w1 + 1; w2 < 0x20; w2++) {
			uint8_t sx2, sy2;
			if (w2 >= 0x10 && w2 <= 0x17) continue;
			if (!sprite_xy(v.spr, w2, &sx2, &sy2)) continue;
			if (abs((int8_t)sx1 - (int8_t)sx2) < 16 && abs((int8_t)sy1 - (int8_t)sy2) < 16) {
				if (!sprite_sprite_bitpattern(&v, sx1, sy1, w1, sx2, sy2, w2)) continue;
				int w = w2 == 0x1f ? w1 : w2, reg = w >> 3;
				if (reg == 3) reg = 2;
				s_collision[reg] |= (uint8_t)(1 << (w & 7));
			}
		}
	}
	// sprite / layer
	lscroll ls[3];
	int bank[3], color[3], en[3];
	for (int l = 0; l < 3; l++) {
		en[l] = (v.mode & (0x10 << l)) != 0;
		if (en[l]) {
			layer_scroll(&v, l, &ls[l]);
			layer_bank_color(&v, l, &bank[l], &color[l]);
		}
	}
	if (!(en[0] | en[1] | en[2]))
		return;
	for (int w = 0; w < 0x20; w++) {
		uint8_t sx, sy;
		if (w >= 0x10 && w <= 0x17) continue;
		if (!sprite_xy(v.spr, w, &sx, &sy)) continue;
		if (v.mode & 1) sx = (uint8_t)(238 - sx);
		if (v.mode & 2) sy = (uint8_t)(242 - sy);
		int minx = sx, miny = sy, maxx = minx + 15, maxy = miny + 15;
		if (maxx > 255) maxx = 255;
		if (maxy > 255) maxy = 255;
		const uint8_t *s = v.spr + w * 4;
		int fx = (s[2] & 1) ^ (v.mode & 1), fy = (s[2] & 2) ^ (v.mode & 2);
		int result = 0;
		for (int y = miny; y <= maxy; y++)
			for (int x = minx; x <= maxx; x++) {
				if (!sprite_pix(&v, s, x - minx, y - miny, fx, fy))
					continue;
				for (int l = 0; l < 3; l++) {
					if (!en[l] || (result & (1 << l)))
						continue;
					int lx, ly;
					layer_src(&v, &ls[l], x, y, &lx, &ly);
					if (layer_pen_raw(&v, l, bank[l], lx, ly))
						result |= 1 << l;
				}
			}
		s_collision[3] |= (uint8_t)result;
	}
}

// ---- drawing (render side, snapshot only) ----
static void decode_dirty(void)
{
	for (int b = 0; b < 2; b++) {
		for (int c = 0; c < 256; c++) {
			if (!(s_snap_chardirty[b][c >> 3] & (1 << (c & 7))))
				continue;
			uint8_t *d = s_dec_char + (b * 256 + c) * 64;
			for (int y = 0; y < 8; y++)
				for (int x = 0; x < 8; x++)
					d[y * 8 + x] = (uint8_t)char_pen(s_snap_charram, b, c, x, y);
		}
		for (int c = 0; c < 64; c++) {
			if (!(s_snap_sprdirty[b][c >> 3] & (1 << (c & 7))))
				continue;
			uint8_t *d = s_dec_spr + (b * 64 + c) * 256;
			for (int y = 0; y < 16; y++)
				for (int x = 0; x < 16; x++)
					d[y * 16 + x] = (uint8_t)spr_pen(s_snap_charram, b, c, x, y);
		}
	}
}

// the layer on the screen lines y0..y1
static void draw_layer(const vstate *v, int which, int y0, int y1)
{
	if (!(v->mode & (0x10 << which)))
		return;
	lscroll ls;
	int bank, color;
	layer_scroll(v, which, &ls);
	layer_bank_color(v, which, &bank, &color);
	const uint8_t *tiles = s_dec_char + bank * 256 * 64;
	const uint8_t *vram = v->vram + which * 0x400;
	const int fx = v->mode & 1, fy = v->mode & 2;
	const uint16_t base = (uint16_t)(color * 8);
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = y0; y <= y1; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int x = 0; x < 256; x++) {
			int lx, ly;
			layer_src(v, &ls, x, y, &lx, &ly);
			int u = fx ? 255 - lx : lx, w = fy ? 255 - ly : ly;
			uint8_t p = tiles[vram[(w >> 3) * 32 + (u >> 3)] * 64 + (w & 7) * 8 + (u & 7)];
			if (p)
				d[x] = (uint16_t)(base + p);
		}
	}
}

// the sprites on the screen lines y0..y1
static void draw_sprites(const vstate *v, int y0, int y1)
{
	if (!(v->mode & 0x80))
		return;
	const int gfx = v->mode & 1, gfy = v->mode & 2;
	const int cx0 = gfx ? 1 : 3, cx1 = gfx ? 252 : 254;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int sprite = 0x1f; sprite >= 0; sprite--) {
		int which = (sprite - 1) & 0x1f;
		if (which >= 0x10 && which <= 0x17) continue;
		uint8_t sx, sy;
		if (!sprite_xy(v->spr, which, &sx, &sy)) continue;
		const uint8_t *s = v->spr + which * 4;
		int code = s[3] & 0x3f, bank = (s[3] & 0x40) != 0;
		int color = 2 * ((v->colorbank[1] >> 4) & 3) + ((s[2] >> 2) & 1);
		int flipx = s[2] & 1, flipy = s[2] & 2;
		if (gfx) { sx = (uint8_t)(238 - sx); flipx = !flipx; }
		if (gfy) { sy = (uint8_t)(242 - sy); flipy = !flipy; }
		const uint8_t *src = s_dec_spr + (bank * 64 + code) * 256;
		const uint16_t base = (uint16_t)(color * 8);
		for (int rep = 0; rep < 2; rep++) {
			int ox = rep ? sx - 0x100 : sx;
			for (int ty = 0; ty < 16; ty++) {
				int y = sy + ty;
				if (y < y0 || y > y1) continue;
				const uint8_t *row = src + (flipy ? 15 - ty : ty) * 16;
				uint16_t *d = bm + (y - VIS_Y0) * pitch;
				for (int tx = 0; tx < 16; tx++) {
					int x = ox + tx;
					if (x < cx0 || x > cx1) continue;
					uint8_t p = row[flipx ? 15 - tx : tx];
					if (p)
						d[x] = (uint16_t)(base + p);
				}
			}
		}
	}
}

// the screen in bands of lines, by the threads that help the render thread too (sj_help): each line is the background,
// then the layers and sprites in the priority order, as the whole screen is
#define BANDS 8
static int s_job_next, s_job_left, s_job_active, s_helpers;
static vstate s_band_v;

static void draw_band(int band)
{
	const int h = VIS_Y1 - VIS_Y0 + 1, r0 = h * band / BANDS, r1 = h * (band + 1) / BANDS;
	const vstate *v = &s_band_v;
	uint16_t *bm = rbx_video.bitmap;
	const uint16_t bg = (uint16_t)(8 * (v->colorbank[1] & 7));
	for (int y = r0; y < r1; y++)
		for (int x = 0; x < 256; x++)
			bm[y * rbx_video.pitch + x] = bg;
	for (int i = 0; i < 4; i++) {
		int which = s_draw_order[s_snap_prio & 0x1f][i];
		if (which == 0)
			draw_sprites(v, VIS_Y0 + r0, VIS_Y0 + r1 - 1);
		else
			draw_layer(v, which - 1, VIS_Y0 + r0, VIS_Y0 + r1 - 1);
	}
}

// bands until none is left (one: a single band); the number drawn
static int draw_bands(int one)
{
	int n = 0;
	for (;;) {
		const int b = __atomic_fetch_add(&s_job_next, 1, __ATOMIC_ACQ_REL);
		if (b >= BANDS)
			return n;
		draw_band(b);
		__atomic_sub_fetch(&s_job_left, 1, __ATOMIC_ACQ_REL);
		n++;
		if (one)
			return n;
	}
}

static int sj_help(int helper)
{
	if (!__atomic_load_n(&s_job_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_job_active, __ATOMIC_SEQ_CST))
		did = draw_bands(helper & 0x100) > 0;
	__atomic_sub_fetch(&s_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

static void sj_render(void)
{
	// palette: 9 bits, inverted, 1k / 470 / 270 ohm weights
	static const double wt[3] = { 255.0 * (1 / 1000.0) / (1 / 1000.0 + 1 / 470.0 + 1 / 270.0),
	                              255.0 * (1 / 470.0) / (1 / 1000.0 + 1 / 470.0 + 1 / 270.0),
	                              255.0 * (1 / 270.0) / (1 / 1000.0 + 1 / 470.0 + 1 / 270.0) };
	for (int i = 0; i < 0x40; i++) {
		int v1 = s_snap_pal[(i << 1) | 1], v0 = s_snap_pal[i << 1];
		int r = (int)(wt[0] * ((~v1 >> 6) & 1) + wt[1] * ((~v1 >> 7) & 1) + wt[2] * ((~v0) & 1) + 0.5);
		int g = (int)(wt[0] * ((~v1 >> 3) & 1) + wt[1] * ((~v1 >> 4) & 1) + wt[2] * ((~v1 >> 5) & 1) + 0.5);
		int b = (int)(wt[0] * ((~v1) & 1) + wt[1] * ((~v1 >> 1) & 1) + wt[2] * ((~v1 >> 2) & 1) + 0.5);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	decode_dirty();
	s_band_v = (vstate){ s_snap_charram, s_snap_vram, s_snap_colscroll, s_snap_spr + ((s_snap_mode & 4) ? 0x80 : 0),
	                     s_snap_kik, s_snap_scroll, s_snap_colorbank, s_snap_mode, s_kind == SJ_KIKSTART };
	s_job_next = 0;
	s_job_left = BANDS;
	__atomic_store_n(&s_job_active, 1, __ATOMIC_SEQ_CST);
	draw_bands(0);
	while (__atomic_load_n(&s_job_left, __ATOMIC_ACQUIRE)) {
	}
	__atomic_store_n(&s_job_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_helpers, __ATOMIC_SEQ_CST)) {   // (none may still take a band of the next frame)
	}
}

static void vblank_capture(void)
{
	check_collision();
	rbx_video_wait_idle();
	memcpy(s_snap_charram, s_charram, sizeof(s_charram));
	memcpy(s_snap_vram, s_c000 + 0x400, sizeof(s_snap_vram));
	memcpy(s_snap_colscroll, s_d000, sizeof(s_snap_colscroll));
	memcpy(s_snap_spr, s_spriteram, sizeof(s_snap_spr));
	memcpy(s_snap_pal, s_palram, sizeof(s_snap_pal));
	memcpy(s_snap_kik, s_kikscroll, sizeof(s_snap_kik));
	memcpy(s_snap_scroll, s_scroll, sizeof(s_scroll));
	memcpy(s_snap_colorbank, s_colorbank, sizeof(s_colorbank));
	s_snap_mode = s_video_mode;
	s_snap_prio = s_priority;
	for (int b = 0; b < 2; b++) {
		for (int i = 0; i < 32; i++) s_snap_chardirty[b][i] |= s_chardirty[b][i];
		for (int i = 0; i < 8; i++) s_snap_sprdirty[b][i] |= s_sprdirty[b][i];
	}
	memset(s_chardirty, 0, sizeof(s_chardirty));
	memset(s_sprdirty, 0, sizeof(s_sprdirty));
	rbx_video_frame_done();
}

// render clears the dirty marks it consumed (it runs before the next capture, which waits for it)
static void sj_render_entry(void)
{
	sj_render();
	memset(s_snap_chardirty, 0, sizeof(s_snap_chardirty));
	memset(s_snap_sprdirty, 0, sizeof(s_snap_sprdirty));
}

// ---- init / frame ----
static void mark_all_dirty(void)
{
	memset(s_chardirty, 0xff, sizeof(s_chardirty));
	memset(s_sprdirty, 0xff, sizeof(s_sprdirty));
}

static void sj_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_cpu_start = 0;
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_charram, 0, sizeof(s_charram));
	memset(s_c000, 0, sizeof(s_c000));
	memset(s_d000, 0, sizeof(s_d000));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_kikscroll, 0, sizeof(s_kikscroll));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(s_scroll, 0, sizeof(s_scroll));
	memset(s_colorbank, 0, sizeof(s_colorbank));
	memset(s_gfxptr, 0, sizeof(s_gfxptr));
	memset(s_collision, 0, sizeof(s_collision));
	s_video_mode = s_priority = 0;
	mark_all_dirty();
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_audio, audio_read, audio_write);
	map_build();
	for (int i = 0; i < 4; i++)
		ay8910_reset(&s_ay[i]);
	// reset_common
	s_sem2 = 0;
	s_soundlatch = 0xff;
	s_soundlatch_flag = 0;
	s_nmi_mask_in = 0;
	s_nmi_line = 0;
	s_input4_f0 = 0;
	s_gears[0] = s_gears[1] = 2;
	s_dac_data = 0;
	s_dac_vol = 0xff;
	update_dac();
	s_spacecr_prot = 0;
	s_protection = 0;
	s_audio_irq_count = 0;
	bankswitch_w(0);
	// security MCU (device_start + device_reset)
	s_addr = 0xffff;
	s_mcu_data = s_host_data = s_read_data = 0xff;
	s_pb_val = 0xff;
	s_pa_val = 0xff;   // port A is open drain with pull-ups: high while the DDR is 0 (MAME starts at 0)
	s_busak = s_busrq = 0;
	s_zaccept = 1;
	s_zready = 0;
	if (s_kind != SJ_NOMCU) {
		m68705_reset(&s_mcu);
		m68705_set_irq_line(&s_mcu, 0);
	}
	s_snd_done = 0;
	s_mix_n = 0;
}

static void sj_exit(void)
{
	free(s_dec_char);
	free(s_dec_spr);
	s_dec_char = s_dec_spr = NULL;
	free(s_mix);
	free(s_tmp);
	s_mix = s_tmp = NULL;
}

static int kind_of(const char *machine)
{
	if (!strcmp(machine, "nomcu")) return SJ_NOMCU;
	if (!strcmp(machine, "mcu")) return SJ_MCU;
	if (!strcmp(machine, "kikstart")) return SJ_KIKSTART;
	return -1;
}

static void compute_draw_order(void)
{
	for (int i = 0; i < 32; i++) {
		int mask = 0;
		for (int j = 3; j >= 0; j--) {
			int data = s_prom[0x10 * (i & 0x0f) + mask] & 0x0f;
			data = (i & 0x10) ? data >> 2 : data & 3;
			mask |= 1 << data;
			s_draw_order[i][j] = data;
		}
	}
}

static int sj_init(const rbx_game *g)
{
	s_kind = kind_of(g->machine);
	if (s_kind < 0) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_spacecr = !strcmp(g->set, "spacecr");
	s_alpine = !strcmp(g->set, "alpine") ? 1 : !strcmp(g->set, "alpinea") ? 2 : 0;
	s_junglhbr = !strcmp(g->set, "junglehbr");
	uint32_t psize = 0;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	s_gfxrom = rbx_region("gfx", &s_gfx_size);
	s_prom = rbx_region("proms", &psize);
	if (!s_main_rom || s_main_size < 0x8000 || !s_audio_rom || !s_prom || psize < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!s_gfxrom)
		s_gfx_size = 0;
	if (s_kind != SJ_NOMCU) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("bmcu:mcu", &msize);
		memset(&s_mcu, 0, sizeof(s_mcu));
		if (!mrom || msize < 0x800 || m68705_init(&s_mcu, M68705_P5, mrom, 0x800)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
		s_mcu.port_r[0] = mcu_port_r;
		s_mcu.port_r[2] = mcu_port_r;
		s_mcu.port_w[0] = mcu_port_w;
		s_mcu.port_w[1] = mcu_port_w;
	}
	compute_draw_order();
	static const char *const in[5] = { "IN0", "IN1", "IN2", "IN3", "IN4" };
	for (int i = 0; i < 5; i++)
		s_p_in[i] = rbx_port_find(in[i]);
	s_p_dsw1 = rbx_port_find("DSW1");
	s_p_dsw2 = rbx_port_find("DSW2");
	s_p_dsw3 = rbx_port_find("DSW3");
	s_p_gear[0] = rbx_port_find("GEARP1");
	s_p_gear[1] = rbx_port_find("GEARP2");

	s_dec_char = (uint8_t *)calloc(2 * 256 * 64, 1);
	s_dec_spr = (uint8_t *)calloc(2 * 64 * 256, 1);
	// AY #0: default (legacy) output, route 0.15; #1 / #2 0.5, #3 1.0 with AY8910_SINGLE_OUTPUT
	ay8910_init(&s_ay[0], AY8910_TYPE_AY8910, 1500000, AY8910_LEGACY_OUTPUT, 38, 4, 1);
	ay8910_init(&s_ay[1], AY8910_TYPE_AY8910, 1500000, AY8910_SINGLE_OUTPUT, 128, 4, 1);
	ay8910_init(&s_ay[2], AY8910_TYPE_AY8910, 1500000, AY8910_SINGLE_OUTPUT, 128, 4, 1);
	ay8910_init(&s_ay[3], AY8910_TYPE_AY8910, 1500000, AY8910_SINGLE_OUTPUT, 256, 4, 1);
	s_ay[0].port_a_read = ay0_pa;
	s_ay[0].port_b_read = ay0_pb;
	s_ay[1].port_a_write = ay1_pa_w;
	s_ay[1].port_b_write = ay1_pb_w;
	s_ay[2].port_a_write = ay2_pa_w;
	s_ay[3].port_b_write = ay3_pb_w;
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_tmp = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_dec_char || !s_dec_spr || !s_mix || !s_tmp || rbx_audio_setup(ay8910_rate(&s_ay[0])) ||
	    rbx_video_setup(256, VIS_Y1 - VIS_Y0 + 1, 64)) {
		sj_exit();
		return -1;
	}
	sj_reset();
	return 0;
}

static void sj_frame(void)
{
	// the frame starts at vblank (MAME updates the inputs there)
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;   // screen_vblank -> IRQ0 HOLD_LINE
		}
		for (int s = 0; s < SLICES; s++) {
			s_slice_start = s_snd_done;
			// main CPU (stalled while the MCU holds the bus)
			s_cpu_is_main = 1;
			s_cpu_start = s_main.z.cyc;
			if (s_busrq)
				s_main.z.cyc += MAIN_LINE / SLICES;
			else
				zcpu_run(&s_main, MAIN_LINE / SLICES);
			// sound CPU with its periodic IRQ
			s_cpu_is_main = 0;
			s_cpu_start = s_audio.z.cyc;
			s_audio_irq_count += AUDIO_LINE / SLICES;
			if (s_audio_irq_count >= AUDIO_IRQ_PERIOD) {
				s_audio_irq_count -= AUDIO_IRQ_PERIOD;
				s_audio.irq = 1;
			}
			zcpu_run(&s_audio, AUDIO_LINE / SLICES);
			if (s_kind != SJ_NOMCU)
				m68705_execute(&s_mcu, MCU_LINE / SLICES);
			snd_run_to(s_slice_start + AY_LINE / SLICES);
		}
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	s_mix_n = 0;
	s_snd_done = 0;
}

static int zcpu_state(void *ctx, const char *tag, zcpu *c, int load)
{
	zcpu t = *c;
	if (rbx_state_io(ctx, tag, &t, sizeof(t)))
		return -1;
	if (load) {
		t.z.read_byte = c->z.read_byte;
		t.z.write_byte = c->z.write_byte;
		t.z.port_in = c->z.port_in;
		t.z.port_out = c->z.port_out;
		t.z.userdata = c->z.userdata;
		t.z.read_map = c->z.read_map;
		*c = t;
	}
	return 0;
}

static int sj_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "charram", s_charram, sizeof(s_charram));
	rc |= rbx_state_io(ctx, "c000", s_c000, sizeof(s_c000));
	rc |= rbx_state_io(ctx, "d000", s_d000, sizeof(s_d000));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "kik", s_kikscroll, sizeof(s_kikscroll));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "scroll", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "cbank", s_colorbank, sizeof(s_colorbank));
	rc |= rbx_state_io(ctx, "gfxptr", s_gfxptr, sizeof(s_gfxptr));
	rc |= rbx_state_io(ctx, "coll", s_collision, sizeof(s_collision));
	for (int i = 0; i < 4; i++) {
		static const char *const tag[4] = { "ay0", "ay1", "ay2", "ay3" };
		rc |= rbx_state_io(ctx, tag[i], &s_ay[i], ay8910_state_size());
	}
	if (s_kind != SJ_NOMCU)
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
	int32_t misc[28] = { s_bank, s_video_mode, s_priority, s_soundlatch, s_input4_f0, s_gears[0], s_gears[1],
		s_spacecr_prot, s_protection, s_soundlatch_flag, s_sem2, s_nmi_mask_in, s_nmi_line, s_sound_on,
		(int32_t)s_audio_irq_count, s_dac_data, s_dac_vol, s_addr, s_mcu_data, s_host_data, s_read_data, s_pa_val,
		s_pb_val, s_zaccept, s_zready, s_busak, s_busrq, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_bank = misc[0]; map_bank(); s_video_mode = (uint8_t)misc[1]; s_priority = (uint8_t)misc[2]; s_soundlatch = (uint8_t)misc[3];
		s_input4_f0 = (uint8_t)misc[4]; s_gears[0] = (uint8_t)misc[5]; s_gears[1] = (uint8_t)misc[6];
		s_spacecr_prot = (uint8_t)misc[7]; s_protection = (uint8_t)misc[8]; s_soundlatch_flag = misc[9];
		s_sem2 = misc[10]; s_nmi_mask_in = misc[11]; s_nmi_line = misc[12]; s_sound_on = misc[13];
		s_audio_irq_count = (uint32_t)misc[14]; s_dac_data = (uint8_t)misc[15]; s_dac_vol = (uint8_t)misc[16];
		s_addr = (uint16_t)misc[17]; s_mcu_data = (uint8_t)misc[18]; s_host_data = (uint8_t)misc[19];
		s_read_data = (uint8_t)misc[20]; s_pa_val = (uint8_t)misc[21]; s_pb_val = (uint8_t)misc[22];
		s_zaccept = misc[23]; s_zready = misc[24]; s_busak = misc[25]; s_busrq = misc[26];
		update_dac();
		mark_all_dirty();
	}
	return 0;
}

static int sj_save(void *ctx) { return sj_state(ctx, 0); }
static int sj_load(void *ctx) { return sj_state(ctx, 1); }
static int sj_supported(const char *machine) { return kind_of(machine) >= 0; }

const rbx_driver rbx_drv_taitosj = { "Taito SJ System", sj_init, sj_exit, sj_reset, sj_frame, sj_save, sj_load, sj_supported, sj_render_entry,
                                      sj_help };
