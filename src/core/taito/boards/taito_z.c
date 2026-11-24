/*  Taito Z System (Continental Circus, Chase H.Q., Enforce, Battle Shark, S.C.I., Night Striker, Aqua Jack,
    Space Gun, Double Axle, Racing Beat), ported from MAME src/mame/taito/taito_z.cpp, taito_z_v.cpp and
    taito_z.h (BSD-3-Clause, copyright-holders: David Graves; thanks to Richard Bush), with the TC0040IOC /
    TC0220IOC / TC0510NIO I/O chips of taitoio.cpp, TC0110PCR (tc0110pcr.cpp) and the ADC0809 of
    src/devices/machine/adc0808.cpp (BSD-3-Clause).
    Two 68000s (one Musashi core switched with m68k_get_context / m68k_set_context, one m68k_bus_t per CPU) and,
    on most games, the Z80 + YM2610 sound board (machine/taito_sound.c); bshark and spacegun drive the YM2610
    from the second 68000. Video: TC0100SCN or TC0480SCP tilemaps, TC0150ROD road, zoomed sprites built from
    spritemap ROM chunks (MAME taito_z_v.cpp draw routines).
    Analog controls (MAME paddle / AD stick / pedal ports) come from rbx_analog(player, RBX_IPT_LIGHTGUN_X / Y),
    counts per frame like the light gun (the core turns the d-pad into 4 counts per frame when there is no pointing
    device): the steering wheel or stick position moves by them and springs back to centre when they stop (MAME
    centerdelta; Space Gun's crosshair stays). Analog pedals: B1 / up = gas, B2 / down = brake.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../../machine/eeprom93c46.h"
#include "../../sound/rbx_ym.h"
#include "../video/tc0100scn.h"
#include "../video/tc0480scp.h"
#include "../video/tc0150rod.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---- address maps (MAME *_map / *_cpub_map) ----
enum {
	D_END,
	D_ROM, D_RAM, D_SHARE, D_SPR, D_SCN, D_ROD, D_PAL, D_SCP,                  // direct memory
	D_SCNC, D_SCPC, D_PCR, D_IOC40B, D_IOC40, D_IOC220, D_NIO8, D_NIOSW,       // handlers
	D_STEER_SCI, D_STEER_DBL, D_CTRL, D_SYT, D_CCOUT, D_ADC, D_YM, D_PAN, D_NOP,
	D_MOTOR, D_SPRFRAME, D_GUNOUT, D_FF,
};
typedef struct { uint32_t start, end; uint8_t dev; } range_t;
#define END { 0, 0, D_END }

static const range_t map_contcirc_a[] = {
	{ 0x000000, 0x03ffff, D_ROM }, { 0x080000, 0x083fff, D_RAM }, { 0x084000, 0x087fff, D_SHARE },
	{ 0x090000, 0x090001, D_CCOUT }, { 0x100000, 0x100007, D_PCR }, { 0x200000, 0x20ffff, D_SCN },
	{ 0x220000, 0x22000f, D_SCNC }, { 0x300000, 0x301fff, D_ROD }, { 0x400000, 0x400fff, D_SPR }, END };
static const range_t map_contcirc_b[] = {
	{ 0x000000, 0x03ffff, D_ROM }, { 0x080000, 0x083fff, D_RAM }, { 0x084000, 0x087fff, D_SHARE },
	{ 0x100000, 0x100003, D_IOC40B }, { 0x200000, 0x200003, D_SYT }, END };
static const range_t map_chasehq_a[] = {
	{ 0x000000, 0x07ffff, D_ROM }, { 0x100000, 0x10ffff, D_RAM }, { 0x108000, 0x10bfff, D_SHARE },
	{ 0x400000, 0x400003, D_IOC40B }, { 0x800000, 0x800001, D_CTRL }, { 0x820000, 0x820003, D_SYT },
	{ 0xa00000, 0xa00007, D_PCR }, { 0xc00000, 0xc0ffff, D_SCN }, { 0xc20000, 0xc2000f, D_SCNC },
	{ 0xd00000, 0xd00fff, D_SPR }, { 0xe00000, 0xe003ff, D_MOTOR }, END };
static const range_t map_chasehq_b[] = {
	{ 0x000000, 0x01ffff, D_ROM }, { 0x100000, 0x103fff, D_RAM }, { 0x108000, 0x10bfff, D_SHARE },
	{ 0x800000, 0x801fff, D_ROD }, END };
static const range_t map_enforce_a[] = {
	{ 0x000000, 0x03ffff, D_ROM }, { 0x100000, 0x103fff, D_RAM }, { 0x104000, 0x107fff, D_SHARE },
	{ 0x200000, 0x200001, D_CCOUT }, { 0x300000, 0x300fff, D_SPR }, { 0x400000, 0x401fff, D_ROD },
	{ 0x500000, 0x500007, D_PCR }, { 0x600000, 0x60ffff, D_SCN }, { 0x620000, 0x62000f, D_SCNC }, END };
static const range_t map_enforce_b[] = {
	{ 0x000000, 0x03ffff, D_ROM }, { 0x100000, 0x103fff, D_RAM }, { 0x104000, 0x107fff, D_SHARE },
	{ 0x200000, 0x200003, D_SYT }, { 0x300000, 0x300003, D_IOC40 }, END };
static const range_t map_bshark_a[] = {
	{ 0x000000, 0x07ffff, D_ROM }, { 0x100000, 0x10ffff, D_RAM }, { 0x110000, 0x113fff, D_SHARE },
	{ 0x400000, 0x40000f, D_IOC220 }, { 0x600000, 0x600001, D_CTRL }, { 0x800000, 0x80000f, D_ADC },
	{ 0xa00000, 0xa01fff, D_PAL }, { 0xc00000, 0xc00fff, D_SPR }, { 0xd00000, 0xd0ffff, D_SCN },
	{ 0xd20000, 0xd2000f, D_SCNC }, END };
static const range_t map_bsharkjjs_a[] = {
	{ 0x000000, 0x07ffff, D_ROM }, { 0x100000, 0x10ffff, D_RAM }, { 0x110000, 0x113fff, D_SHARE },
	{ 0x400000, 0x40000f, D_IOC220 }, { 0x600000, 0x600001, D_CTRL },
	{ 0xa00000, 0xa01fff, D_PAL }, { 0xc00000, 0xc00fff, D_SPR }, { 0xd00000, 0xd0ffff, D_SCN },
	{ 0xd20000, 0xd2000f, D_SCNC }, END };
static const range_t map_bshark_b[] = {
	{ 0x000000, 0x07ffff, D_ROM }, { 0x108000, 0x10bfff, D_RAM }, { 0x110000, 0x113fff, D_SHARE },
	{ 0x400000, 0x400007, D_PAN }, { 0x600000, 0x600007, D_YM }, { 0x60000c, 0x60000f, D_NOP },
	{ 0x800000, 0x801fff, D_ROD }, END };
static const range_t map_sci_a[] = {
	{ 0x000000, 0x07ffff, D_ROM }, { 0x100000, 0x10ffff, D_RAM }, { 0x108000, 0x10bfff, D_SHARE },
	{ 0x200000, 0x20000f, D_IOC220 }, { 0x200010, 0x20001f, D_STEER_SCI }, { 0x400000, 0x400001, D_CTRL },
	{ 0x420000, 0x420003, D_SYT }, { 0x800000, 0x801fff, D_PAL }, { 0xa00000, 0xa0ffff, D_SCN },
	{ 0xa20000, 0xa2000f, D_SCNC }, { 0xc00000, 0xc03fff, D_SPR }, { 0xc08000, 0xc08001, D_SPRFRAME }, END };
static const range_t map_sci_b[] = {
	{ 0x000000, 0x01ffff, D_ROM }, { 0x200000, 0x203fff, D_RAM }, { 0x208000, 0x20bfff, D_SHARE },
	{ 0xa00000, 0xa01fff, D_ROD }, END };
static const range_t map_nightstr_a[] = {
	{ 0x000000, 0x07ffff, D_ROM }, { 0x100000, 0x10ffff, D_RAM }, { 0x110000, 0x113fff, D_SHARE },
	{ 0x400000, 0x40000f, D_IOC220 }, { 0x800000, 0x800001, D_CTRL }, { 0x820000, 0x820003, D_SYT },
	{ 0xa00000, 0xa00007, D_PCR }, { 0xc00000, 0xc0ffff, D_SCN }, { 0xc20000, 0xc2000f, D_SCNC },
	{ 0xd00000, 0xd00fff, D_SPR }, { 0xe00000, 0xe00011, D_NOP }, { 0xe40000, 0xe4000f, D_ADC }, END };
static const range_t map_nightstr_b[] = {
	{ 0x000000, 0x03ffff, D_ROM }, { 0x100000, 0x103fff, D_RAM }, { 0x104000, 0x107fff, D_SHARE },
	{ 0x800000, 0x801fff, D_ROD }, END };
static const range_t map_aquajack_a[] = {
	{ 0x000000, 0x03ffff, D_ROM }, { 0x100000, 0x103fff, D_RAM }, { 0x104000, 0x107fff, D_SHARE },
	{ 0x200000, 0x200001, D_CTRL }, { 0x300000, 0x300007, D_PCR }, { 0x800000, 0x801fff, D_ROD },
	{ 0xa00000, 0xa0ffff, D_SCN }, { 0xa20000, 0xa2000f, D_SCNC }, { 0xc40000, 0xc40fff, D_SPR }, END };
static const range_t map_aquajack_b[] = {
	{ 0x000000, 0x03ffff, D_ROM }, { 0x100000, 0x103fff, D_RAM }, { 0x104000, 0x107fff, D_SHARE },
	{ 0x200000, 0x20000f, D_IOC220 }, { 0x300000, 0x300003, D_SYT }, { 0x800800, 0x80083f, D_FF }, END };
static const range_t map_spacegun_a[] = {
	{ 0x000000, 0x07ffff, D_ROM }, { 0x30c000, 0x30ffff, D_RAM }, { 0x310000, 0x31ffff, D_SHARE },
	{ 0x500000, 0x500fff, D_SPR }, { 0x900000, 0x90ffff, D_SCN }, { 0x920000, 0x92000f, D_SCNC },
	{ 0xb00000, 0xb00007, D_PCR }, END };
static const range_t map_spacegun_b[] = {
	{ 0x000000, 0x03ffff, D_ROM }, { 0x20c000, 0x20ffff, D_RAM }, { 0x210000, 0x21ffff, D_SHARE },
	{ 0x800000, 0x80000f, D_NIO8 }, { 0xc00000, 0xc00007, D_YM }, { 0xc0000c, 0xc0000f, D_NOP },
	{ 0xc20000, 0xc20007, D_PAN }, { 0xe00000, 0xe00001, D_GUNOUT }, { 0xf00000, 0xf0000f, D_ADC }, END };
static const range_t map_dblaxle_a[] = {
	{ 0x000000, 0x07ffff, D_ROM }, { 0x200000, 0x203fff, D_RAM }, { 0x210000, 0x21ffff, D_SHARE },
	{ 0x400000, 0x40000f, D_NIOSW }, { 0x400010, 0x40001f, D_STEER_DBL }, { 0x600000, 0x600001, D_CTRL },
	{ 0x620000, 0x620003, D_SYT }, { 0x800000, 0x801fff, D_PAL }, { 0x900000, 0x90ffff, D_SCP },
	{ 0xa00000, 0xa0ffff, D_SCP }, { 0xa30000, 0xa3002f, D_SCPC }, { 0xc00000, 0xc03fff, D_SPR }, END };
static const range_t map_dblaxle_b[] = {
	{ 0x000000, 0x03ffff, D_ROM }, { 0x100000, 0x103fff, D_RAM }, { 0x110000, 0x11ffff, D_SHARE },
	{ 0x300000, 0x301fff, D_ROD }, { 0x500000, 0x503fff, D_RAM }, END };
static const range_t map_racingb_a[] = {
	{ 0x000000, 0x07ffff, D_ROM }, { 0x100000, 0x103fff, D_RAM }, { 0x110000, 0x11ffff, D_SHARE },
	{ 0x300000, 0x30000f, D_NIOSW }, { 0x300010, 0x30001f, D_STEER_DBL }, { 0x500002, 0x500003, D_CTRL },
	{ 0x520000, 0x520003, D_SYT }, { 0x700000, 0x701fff, D_PAL }, { 0x900000, 0x90ffff, D_SCP },
	{ 0x930000, 0x93002f, D_SCPC }, { 0xb00000, 0xb03fff, D_SPR }, { 0xb08000, 0xb08001, D_SPRFRAME }, END };
static const range_t map_racingb_b[] = {
	{ 0x000000, 0x03ffff, D_ROM }, { 0x400000, 0x403fff, D_RAM }, { 0x410000, 0x41ffff, D_SHARE },
	{ 0xa00000, 0xa01fff, D_ROD }, { 0xd00000, 0xd03fff, D_RAM }, END };

// ---- machine configurations (MAME taitoz_state::contcirc ...) ----
enum { MZ_CONTCIRC, MZ_CHASEHQ, MZ_ENFORCE, MZ_BSHARK, MZ_SCI, MZ_NIGHTSTR, MZ_AQUAJACK, MZ_SPACEGUN, MZ_DBLAXLE,
       MZ_RACINGB };
enum { SPR_CONTCIRC, SPR_CHASEHQ, SPR_BSHARK, SPR_SCI, SPR_AQUAJACK, SPR_SPACEGUN };
enum { PAL_PCR_XRGB, PAL_PCR_XBGR, PAL_RAM };
enum { SCR_SCN_ROAD, SCR_SPACEGUN, SCR_DBLAXLE, SCR_RACINGB };
enum { IO_TC0040IOC, IO_TC0220IOC, IO_TC0510NIO };
enum { STEER_NONE, STEER_CONTCIRC, STEER_CHASEHQ, STEER_SCI, STEER_DBLAXLE, STEER_RACINGB };

typedef struct {
	const char *name;
	int kind;
	const range_t *map[2];
	uint32_t clock;
	int irq_a, irq_b;           // vblank interrupt levels (irqN_line_hold)
	int vis_y0, vis_h;          // MAME visible lines vdisp_start .. vdisp_end - 1
	int slices;                 // CPU slices per scanline (MAME maximum quantum)
	int spr, spr_bytes, spr_yoffs;
	int pal, scr, io, steer;
	int road_yoffs, road_paloffs, road_type, road_trans;   // road_paloffs < 0: contcirc road palette bank
	int z80;                    // sound board present (else YM2610 on the sub CPU)
	int gain_ssg, gain_fm;      // YM2610 route gains in 1/256 (mono)
	int adc_cpu, adc_irq;       // ADC0809 end of conversion interrupt (CPU 0 / 1, level), 0 = no ADC
	int flipx;                  // ORIENTATION_FLIP_X
	int scn_xoffs;
} machine_t;

static const machine_t s_machines[] = {
	{ "contcirc", MZ_CONTCIRC, { map_contcirc_a, map_contcirc_b }, 12000000, 6, 6, 24, 224, 1,
	  SPR_CONTCIRC, 0x700, 5, PAL_PCR_XRGB, SCR_SCN_ROAD, IO_TC0040IOC, STEER_CONTCIRC, -3, -1, 1, 0, 1, 154, 128,
	  0, 0, 0, 0 },
	{ "chasehq", MZ_CHASEHQ, { map_chasehq_a, map_chasehq_b }, 12000000, 4, 4, 16, 240, 1,
	  SPR_CHASEHQ, 0x800, 7, PAL_PCR_XBGR, SCR_SCN_ROAD, IO_TC0040IOC, STEER_CHASEHQ, -1, 0xc0, 0, 0, 1, 154, 256,
	  0, 0, 0, 0 },
	{ "enforce", MZ_ENFORCE, { map_enforce_a, map_enforce_b }, 12000000, 6, 6, 24, 224, 1,
	  SPR_CONTCIRC, 0x700, 5, PAL_PCR_XRGB, SCR_SCN_ROAD, IO_TC0040IOC, STEER_NONE, -3, -1, 1, 0, 1, 192, 256,
	  0, 0, 0, 0 },
	{ "bshark", MZ_BSHARK, { map_bshark_a, map_bshark_b }, 12000000, 4, 4, 16, 240, 2,
	  SPR_BSHARK, 0x1000, 8, PAL_RAM, SCR_SCN_ROAD, IO_TC0220IOC, STEER_NONE, -1, 0xc0, 0, 1, 0, 192, 7168,
	  0, 6, 1, 0 },
	{ "bsharkjjs", MZ_BSHARK, { map_bsharkjjs_a, map_bshark_b }, 12000000, 4, 4, 16, 240, 2,
	  SPR_BSHARK, 0x1000, 8, PAL_RAM, SCR_SCN_ROAD, IO_TC0220IOC, STEER_NONE, -1, 0xc0, 0, 1, 0, 192, 7168,
	  0, 0, 1, 0 },
	{ "sci", MZ_SCI, { map_sci_a, map_sci_b }, 16000000, 4, 4, 16, 240, 1,
	  SPR_SCI, 0x4000, 6, PAL_RAM, SCR_SCN_ROAD, IO_TC0220IOC, STEER_SCI, -1, 0xc0, 0, 0, 1, 192, 256,
	  0, 0, 0, 0 },
	{ "nightstr", MZ_NIGHTSTR, { map_nightstr_a, map_nightstr_b }, 12000000, 4, 4, 16, 240, 2,
	  SPR_CHASEHQ, 0x800, 7, PAL_PCR_XBGR, SCR_SCN_ROAD, IO_TC0220IOC, STEER_NONE, -1, 0xc0, 0, 0, 1, 154, 192,
	  0, 6, 0, 0 },
	{ "aquajack", MZ_AQUAJACK, { map_aquajack_a, map_aquajack_b }, 12000000, 4, 4, 16, 240, 2,
	  SPR_AQUAJACK, 0x400, 3, PAL_PCR_XBGR, SCR_SCN_ROAD, IO_TC0220IOC, STEER_NONE, -1, 0, 2, 1, 1, 192, 256,
	  0, 0, 0, 0 },
	{ "spacegun", MZ_SPACEGUN, { map_spacegun_a, map_spacegun_b }, 12000000, 4, 4, 16, 240, 1,
	  SPR_SPACEGUN, 0x600, 4, PAL_PCR_XRGB, SCR_SPACEGUN, IO_TC0510NIO, STEER_NONE, 0, 0, 0, 0, 0, 192, 2048,
	  1, 5, 1, 4 },
	{ "dblaxle", MZ_DBLAXLE, { map_dblaxle_a, map_dblaxle_b }, 16000000, 4, 4, 16, 240, 1,
	  SPR_BSHARK, 0x4000, 7, PAL_RAM, SCR_DBLAXLE, IO_TC0510NIO, STEER_DBLAXLE, -1, 0xc0, 0, 0, 1, 192, 320,
	  0, 0, 0, 0 },
	{ "racingb", MZ_RACINGB, { map_racingb_a, map_racingb_b }, 16000000, 4, 4, 16, 240, 1,
	  SPR_SCI, 0x4000, 7, PAL_RAM, SCR_RACINGB, IO_TC0510NIO, STEER_RACINGB, -1, 0xc0, 0, 0, 1, 192, 256,
	  0, 0, 0, 0 },
};

// screen: 26.686 MHz / 4 pixel clock, 424 x 262 -> 60.056 Hz
#define LINES 262
#define FRAME_MHZ 60056
#define SCREEN_W 320
#define PAL_ENTRIES 4096
#define MAX_RAMS 8

static const machine_t *s_m;
static uint8_t *s_rom[2];
static uint32_t s_rom_size[2];
static uint8_t *s_ram[MAX_RAMS];
static uint32_t s_ram_size[MAX_RAMS];
static int s_nram;
static uint8_t (*s_share_hp)[0x10000];
#define s_share (*s_share_hp)
static uint16_t (*s_spr_hp)[0x2000];          // sprite RAM (0x4000 bytes at most)
#define s_spr (*s_spr_hp)
static uint16_t s_palram[0x1000];       // palette RAM or TC0110PCR RAM
static uint16_t s_pcr_addr;
static tc0100scn_t *s_scn;
static tc0480scp_t *s_scp;
static tc0150rod_t *s_rod;
static m68k_bus_t (*s_bus_hp)[2];
#define s_bus (*s_bus_hp)
static uint8_t s_ctx[2][1024];
static int s_cur;                        // CPU whose context is in Musashi
static int s_sub_reset;                  // sub CPU held in reset (cpua_ctrl bit 0 low)
static uint16_t s_cpua_ctrl;
static uint64_t s_frac[2], s_ym_frac;
static uint8_t s_io_regs[8], s_ioc_port;
static int s_port_io[8], s_port_unk[4], s_port_adj[2];
static int s_road_palbank, s_sci_spriteframe, s_sci_int6;
static int32_t s_int6_cycles;            // sci: cycles until the delayed IRQ 6 (< 0 = none)
static int s_adc_addr, s_adc_sar, s_adc_eoc;
static int32_t s_adc_cycles;             // cycles until the end of conversion (< 0 = idle)
static int s_wheel, s_dial;              // steering wheel position -128..127, dial count
static int s_stick[2][2];                // AD stick positions of players 1 / 2, -128..127
static int s_enforceja;
static eeprom93c46_t s_eeprom;
static int s_has_eeprom;
static char s_nv_path[512];
static int s_pan[4];
// direct YM2610 (bshark, spacegun)
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ym_pending;

// drawing snapshot
static struct {
	uint16_t spr[0x2000];
	uint16_t pal[0x1000];
	int road_palbank, spriteframe;
} *s_snap;
static tc0100scn_t *s_snap_scn;
static tc0480scp_t *s_snap_scp;
static tc0150rod_t *s_snap_rod;
static uint8_t *s_pribuf;

// sprite graphics: 16x8 or 16x16 tiles, one byte per pixel
typedef struct { uint8_t *pix, *used; uint32_t n; int h; } sprgfx_t;
static sprgfx_t s_gfx[2];

// ---- CPU switching ----
static void cpu_switch(int n)
{
	if (n == s_cur)
		return;
	m68k_get_context(s_ctx[s_cur]);
	m68k_set_context(s_ctx[n]);
	m68k_bus_select(&s_bus[n]);
	s_cur = n;
	m68k_bus_update_irq();
}

static void cpu_hold(int n, int level)
{
	s_bus[n].irq_pending |= 1u << level;
	if (n == s_cur)
		m68k_bus_update_irq();
}

static void cpu_line(int n, int level, int state)
{
	if (state)
		s_bus[n].irq_asserted |= 1u << level;
	else
		s_bus[n].irq_asserted &= ~(1u << level);
	if (n == s_cur)
		m68k_bus_update_irq();
}

static void cpua_ctrl_w(uint16_t data, uint16_t mask)
{
	if (mask == 0xff00)
		data >>= 8;
	s_cpua_ctrl = data & 0xff;
	int hold = !(s_cpua_ctrl & 1);   // bit 0 enables CPU B
	if (s_sub_reset && !hold) {      // reset released: the CPU restarts from its vectors
		int was = s_cur;
		cpu_switch(1);
		m68k_pulse_reset();
		cpu_switch(was);
	}
	s_sub_reset = hold;
}

// ---- analog inputs ----
static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

// move a -128..127 position by this frame's counts, back to centre by 16 per frame when idle (if centre)
static int move_axis(int pos, int delta, int centre)
{
	if (delta)
		return clampi(pos + delta * 4, -128, 127);
	if (!centre)
		return pos;
	if (pos > 16) return pos - 16;
	if (pos < -16) return pos + 16;
	return 0;
}

static void read_analog(void)
{
	int dx = rbx_analog(1, RBX_IPT_LIGHTGUN_X), dy = rbx_analog(1, RBX_IPT_LIGHTGUN_Y);
	int centre = s_m->kind != MZ_SPACEGUN;
	if (s_m->steer != STEER_NONE) {
		s_wheel = move_axis(s_wheel, dx, 1);
		s_dial += s_wheel / 8;
	}
	s_stick[0][0] = move_axis(s_stick[0][0], dx, centre);
	s_stick[0][1] = move_axis(s_stick[0][1], dy, centre);
	if (s_m->kind == MZ_SPACEGUN) {
		s_stick[1][0] = move_axis(s_stick[1][0], rbx_analog(2, RBX_IPT_LIGHTGUN_X), 0);
		s_stick[1][1] = move_axis(s_stick[1][1], rbx_analog(2, RBX_IPT_LIGHTGUN_Y), 0);
	}
}

// AD stick / paddle value from a -128..127 position over MAME PORT_MINMAX, PORT_REVERSE
static int axis_value(int pos, int lo, int hi, int reverse)
{
	int v = pos < 0 ? 0x80 + pos * (0x80 - lo) / 128 : 0x80 + pos * (hi - 0x80) / 127;
	return reverse ? lo + hi - v : v;
}

static uint8_t dsw(int i) { return (uint8_t)rbx_port_read(s_port_io[i]); }

// MAME STEER port: paddle (0x80 centre) or dial (16-bit count) by the cabinet DIP
static uint16_t steer_value(void)
{
	int v;
	switch (s_m->steer) {
	case STEER_CONTCIRC:
		v = (dsw(1) & 0x10) ? -s_dial : axis_value(s_wheel, 0x20, 0xe0, 1);
		break;
	case STEER_CHASEHQ:
		v = (dsw(0) & 3) == 2 ? s_dial : axis_value(s_wheel, 0x20, 0xe0, 0);
		break;
	case STEER_SCI: v = axis_value(s_wheel, 0x20, 0xe0, 0); break;
	case STEER_DBLAXLE: v = axis_value(s_wheel, 0x40, 0xc0, 0); break;
	case STEER_RACINGB: v = (dsw(1) & 0x08) ? axis_value(s_wheel, 0x00, 0xff, 0) : s_dial; break;
	default: v = 0x80; break;
	}
	return (uint16_t)(0xff80 + v);
}

static const uint8_t s_gray[8] = { 0, 1, 3, 2, 6, 7, 5, 4 };   // MAME gas_pedal_r / brake_pedal_r / adstick_r

// I/O chip input ports with the IPT_CUSTOM bits of MAME (pedals, enforceja stick)
static uint8_t in_port(int i)
{
	uint8_t v = (uint8_t)rbx_port_read(s_port_io[i]);
	if (i != 2 && i != 3)
		return v;
	int dy = 0;
	switch (s_m->kind) {
	case MZ_CONTCIRC:
		if (dsw(0) & 1) {   // cockpit: analog pedals; the gas / brake switch bits tell the buttons
			int on = (v & 0xe0) != 0xe0;
			dy = rbx_analog(1, RBX_IPT_LIGHTGUN_Y);
			on |= i == 2 ? dy < 0 : dy > 0;
			v = (uint8_t)((v & 0x1f) | s_gray[on ? 7 : 0] << 5);
		}
		break;
	case MZ_CHASEHQ:
		if (!(dsw(0) & 2)) {   // analog pedals: IN0 brake, IN1 gas
			int on = !(v & 0x20);
			dy = rbx_analog(1, RBX_IPT_LIGHTGUN_Y);
			on |= i == 3 ? dy < 0 : dy > 0;
			v = (uint8_t)((v & 0x1f) | s_gray[on ? 7 : 0] << 5);
		}
		break;
	case MZ_ENFORCE:
		if (s_enforceja) {
			int raw = (axis_value(s_stick[0][i - 2], 0, 0xff, 0) >> 5) & 7;
			v = (uint8_t)((v & 0x1f) | s_gray[raw] << 5);
		}
		break;
	}
	return v;
}

static uint8_t adc_input(int ch)
{
	switch (s_m->kind) {
	case MZ_BSHARK:
		switch (ch) {
		case 0: return (uint8_t)axis_value(s_stick[0][0], 0x35, 0xcc, 1);
		case 1: return (uint8_t)rbx_port_read(s_port_adj[0]);
		case 2: return (uint8_t)axis_value(s_stick[0][1], 0x32, 0xd5, 0);
		case 3: return (uint8_t)rbx_port_read(s_port_adj[1]);
		}
		break;
	case MZ_NIGHTSTR:
		switch (ch) {
		case 0: return (uint8_t)axis_value(s_stick[0][0], 0, 0xff, 0);
		case 1: return (uint8_t)axis_value(s_stick[0][1], 0, 0xff, 1);
		case 2: return (uint8_t)rbx_port_read(s_port_adj[0]);
		case 3: return (uint8_t)rbx_port_read(s_port_adj[1]);
		}
		break;
	case MZ_SPACEGUN:
		if (ch < 4)
			return (uint8_t)axis_value(s_stick[ch >> 1][ch & 1], 0, 0xff, !(ch & 1));
		break;
	}
	return 0;
}

// ---- I/O chips ----
static uint8_t ioc_portreg(int port)
{
	switch (port) {
	case 0: case 1: case 2: case 3: case 7: return in_port(port);
	case 4: return s_io_regs[4];
	}
	return 0xff;
}

// MAME contcirc_input_bypass_r / chasehq_input_bypass_r
static uint8_t ioc_bypass(void)
{
	uint16_t steer = steer_value();
	if (s_m->kind == MZ_CONTCIRC) {
		if (s_ioc_port == 8) return (uint8_t)steer;
		if (s_ioc_port == 9) return (uint8_t)(steer >> 8);
	} else {
		if (s_ioc_port >= 8 && s_ioc_port <= 0x0b) return (uint8_t)rbx_port_read(s_port_unk[s_ioc_port - 8]);
		if (s_ioc_port == 0x0c) return (uint8_t)steer;
		if (s_ioc_port == 0x0d) return (uint8_t)(steer >> 8);
	}
	return ioc_portreg(s_ioc_port);
}

static void ioc_portreg_w(uint8_t v)
{
	if (s_ioc_port < 8)
		s_io_regs[s_ioc_port] = v;   // 4 = coin lockout / counters
}

static uint8_t io_read(int offset)   // TC0220IOC / TC0510NIO
{
	offset &= 7;
	if (offset == 3 && s_has_eeprom)   // spacegun: EEPROM data out on bit 7
		return (uint8_t)(eeprom_do(&s_eeprom) << 7);
	return ioc_portreg(offset);
}

static void io_write(int offset, uint8_t v)
{
	offset &= 7;
	s_io_regs[offset] = v;   // 0 = watchdog, 4 = coin lockout / counters
	if (offset == 3 && s_has_eeprom)   // EEPROMOUT: 0x10 cs, 0x20 clk, 0x40 di
		eeprom_write_lines(&s_eeprom, (v >> 4) & 1, (v >> 5) & 1, (v >> 6) & 1);
}

// ---- ADC0809 (MAME adc0808_device: address_offset_start_w / data_r) ----
static void adc_start(int ch)
{
	s_adc_addr = ch & 7;
	// start + 1 clock, then 64 clocks of conversion at 500 kHz
	s_adc_cycles = (int32_t)((uint64_t)s_m->clock * 66 / 500000);
	if (s_adc_eoc)
		s_adc_eoc = 0;
}

static uint8_t adc_read(void)
{
	cpu_line(s_m->adc_cpu, s_m->adc_irq, 0);   // OE clears the EOC flip-flop
	return (uint8_t)s_adc_sar;
}

static void adc_advance(int cycles)
{
	if (s_adc_cycles < 0)
		return;
	s_adc_cycles -= cycles;
	if (s_adc_cycles <= 0) {
		s_adc_cycles = -1;
		s_adc_sar = adc_input(s_adc_addr);
		s_adc_eoc = 1;
		cpu_line(s_m->adc_cpu, s_m->adc_irq, 1);
	}
}

// ---- YM2610 on the sub CPU ----
static void ym_sync(void)
{
	if (s_ym_pending) {
		s_mix_n += rbx_ym_advance(0, s_ym_pending, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		s_ym_pending = 0;
	}
}

static void pan_w(int offset, uint8_t data)
{
	// MAME pancontrol_w: filter gain data / 255 on 2610.1.r, 2610.1.l, 2610.2.r, 2610.2.l; mono = their average
	s_pan[offset & 3] = data;
	if (!s_m->z80) {
		ym_sync();
		rbx_ym_set_volume(0, (s_pan[0] + s_pan[1] + s_pan[2] + s_pan[3] + 2) / 4);
	}
}

// ---- bus handlers ----
static const range_t *find_range(uint32_t a)
{
	for (const range_t *r = s_m->map[s_cur]; r->dev; r++)
		if (a >= r->start && a <= r->end)
			return r;
	return NULL;
}

static uint16_t dev_read16(uint32_t a)
{
	const range_t *r = find_range(a);
	if (!r)
		return 0;
	uint32_t off = (a - r->start) >> 1;
	switch (r->dev) {
	case D_SCNC: return tc0100scn_ctrl_r(s_scn, (int)off);
	case D_SCPC: return tc0480scp_ctrl_r(s_scp, (int)off);
	case D_PCR: return (off & 3) == 1 ? s_palram[s_pcr_addr] : 0xff;
	case D_IOC40B: return off & 1 ? 0 : ioc_bypass();          // 1 = port / watchdog
	case D_IOC40: return off & 1 ? 0 : ioc_portreg(s_ioc_port);
	case D_IOC220: case D_NIO8: return io_read((int)off);
	case D_NIOSW: return io_read((int)(off ^ 1));
	case D_STEER_SCI: {
		uint16_t s = steer_value();
		return off == 4 ? (s & 0xff) : off == 5 ? (s >> 8) : 0xff;
	}
	case D_STEER_DBL: {
		uint16_t s = steer_value();
		return off == 4 ? (s >> 8) : off == 5 ? (s & 0xff) : 0;
	}
	case D_SYT: return off & 1 ? tc0140syt_master_comm_r(tsnd_syt()) : 0;
	case D_ADC: return adc_read();
	case D_YM: ym_sync(); return rbx_ym_read(0, off & 3);
	case D_MOTOR:   // MAME chasehq_motor_r: random motor status, 0x55 motor CPU status
		return off == 0 ? (uint16_t)(rand() & 0xff) : off == 0x101 ? 0x55 : 0;
	case D_SPRFRAME: return (uint16_t)(s_sci_spriteframe << 8);
	case D_FF: return 0xff;
	}
	return 0;
}

static void dev_write16(uint32_t a, uint16_t v, uint16_t mask)
{
	const range_t *r = find_range(a);
	if (!r)
		return;
	uint32_t off = (a - r->start) >> 1;
	uint8_t lo = (uint8_t)v;
	int lo_lane = mask & 0x00ff;
	switch (r->dev) {
	case D_SCNC: tc0100scn_ctrl_w(s_scn, (int)off, v, mask); break;
	case D_SCPC: tc0480scp_ctrl_w(s_scp, (int)off, v, mask); break;
	case D_PCR:   // TC0110PCR (shift 0): 0 = address, 1 = data
		if ((off & 3) == 0)
			s_pcr_addr = v & 0xfff;
		else if ((off & 3) == 1)
			s_palram[s_pcr_addr] = v;
		break;
	case D_IOC40B:
	case D_IOC40:
		if (!lo_lane)
			break;
		if (off & 1)
			s_ioc_port = lo;
		else
			ioc_portreg_w(lo);
		break;
	case D_IOC220: case D_NIO8: if (lo_lane) io_write((int)off, lo); break;
	case D_NIOSW: io_write((int)(off ^ 1), lo_lane ? lo : (uint8_t)(v >> 8)); break;   // halfword_w
	case D_CTRL: cpua_ctrl_w(v, mask); break;
	case D_SYT:
		if (!lo_lane)
			break;
		if (off & 1)
			tc0140syt_master_comm_w(tsnd_syt(), lo);
		else
			tc0140syt_master_port_w(tsnd_syt(), lo);
		break;
	case D_CCOUT:   // MAME contcirc_out_w
		if (!lo_lane)
			break;
		tsnd_syt()->reset(!(lo & 1));   // bit 0: sound CPU reset
		s_road_palbank = (lo & 0xc0) >> 6;
		break;
	case D_ADC: if (lo_lane) adc_start((int)off); break;
	case D_YM: if (lo_lane) { ym_sync(); rbx_ym_write(0, off & 3, lo); } break;
	case D_PAN: if (lo_lane) pan_w((int)off, lo); break;
	case D_SPRFRAME: if (mask & 0xff00) s_sci_spriteframe = (v >> 8) & 0xff; break;
	}
}

static uint16_t bus_read16(uint32_t a) { return dev_read16(a & ~1u); }
static uint8_t bus_read8(uint32_t a)
{
	uint16_t v = dev_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void bus_write16(uint32_t a, uint16_t v) { dev_write16(a & ~1u, v, 0xffff); }
static void bus_write8(uint32_t a, uint8_t v)
{
	dev_write16(a & ~1u, (uint16_t)(v << 8 | v), (a & 1) ? 0x00ff : 0xff00);
}

// ---- sprites (MAME taito_z_v.cpp draw routines) ----
static int decode_sprites(sprgfx_t *g, const char *region, int h)
{
	uint32_t size;
	const uint8_t *src = rbx_region(region, &size);
	if (!src)
		return -1;
	uint32_t n = size / (8 * h);   // 16 pixels x 4 planes = 8 bytes per row
	g->pix = (uint8_t *)malloc((size_t)n * 16 * h + n);
	if (!g->pix)
		return -1;
	g->used = g->pix + (size_t)n * 16 * h;
	g->n = n;
	g->h = h;
	for (uint32_t i = 0; i < n; i++) {
		uint8_t *d = g->pix + (size_t)i * 16 * h, any = 0;
		const uint8_t *s = src + (size_t)i * 8 * h;
		for (int y = 0; y < h; y++, s += 8)
			for (int x = 0; x < 16; x++) {
				int bit = 7 - (x & 7), b = x >> 3;   // planes at bit offsets 0, 16, 32, 48 (MSB first)
				uint8_t p = (uint8_t)(((s[b] >> bit) & 1) << 3 | ((s[2 + b] >> bit) & 1) << 2 |
				                      ((s[4 + b] >> bit) & 1) << 1 | ((s[6 + b] >> bit) & 1));
				d[y * 16 + x] = p;
				any |= p;
			}
		g->used[i] = any != 0;
	}
	return 0;
}

static int s_vy0, s_vy1;   // visible screen lines

// gfx_element::prio_zoom_transpen with pmask | 1 << 31; drawn pixels set the priority to 31. Clipped to the screen
// lines cy0..cy1 (bm: the line s_vy0)
static void draw_chunk(uint16_t *bm, int pitch, const sprgfx_t *g, uint32_t code, uint32_t color, int flipx, int flipy,
                       int destx, int desty, int dstw, int dsth, uint32_t pmask, int cy0, int cy1)
{
	if (dstw < 1 || dsth < 1)
		return;
	int ex = destx + dstw - 1, ey = desty + dsth - 1;
	if (destx > SCREEN_W - 1 || ex < 0 || desty > cy1 || ey < cy0)   // (first: no divide for a chunk off these lines)
		return;
	if (code >= g->n)
		code %= g->n;
	if (!g->used[code])
		return;
	int dx = (16 << 16) / dstw, dy = (g->h << 16) / dsth;
	int srcx = 0, srcy = 0;
	if (destx < 0) { srcx = -destx * dx; destx = 0; }
	if (ex > SCREEN_W - 1) ex = SCREEN_W - 1;
	if (desty < cy0) { srcy = (cy0 - desty) * dy; desty = cy0; }
	if (ey > cy1) ey = cy1;
	if (flipx) { srcx = (dstw - 1) * dx - srcx; dx = -dx; }
	if (flipy) { srcy = (dsth - 1) * dy - srcy; dy = -dy; }
	pmask |= 1u << 31;
	const uint8_t *data = g->pix + (size_t)code * 16 * g->h;
	uint16_t base = (uint16_t)((color & 0xff) << 4);
	for (int y = desty; y <= ey; y++, srcy += dy) {
		const uint8_t *s = data + (srcy >> 16) * 16;
		uint16_t *d = bm + (y - s_vy0) * pitch;
		uint8_t *p = s_pribuf + (y - s_vy0) * pitch;
		int sx = srcx;
		for (int x = destx; x <= ex; x++, sx += dx) {
			uint8_t c = s[sx >> 16];
			if (c) {
				if (!((1u << (p[x] & 0x1f)) & pmask))
					d[x] = (uint16_t)(base + c);
				p[x] = 31;
			}
		}
	}
}

// a big sprite of cols x rows chunks from the spritemap ROM
static void draw_big(uint16_t *bm, int pitch, const sprgfx_t *g, const uint16_t *map, int cols, int rows, int x, int y,
                     int zoomx, int zoomy, int flipx, int flipy, uint32_t color, uint32_t pmask, int cy0, int cy1)
{
	// (the chunks lie within x.., y.. by zoomx, zoomy: a sprite off these lines is skipped whole, a row of chunks too.
	// cols and rows are powers of two and the products not negative: the divides are shifts)
	if (y > cy1 || y + zoomy - 1 < cy0 || x > SCREEN_W - 1 || x + zoomx - 1 < 0)
		return;
	const int cs = __builtin_ctz((unsigned)cols), rs = __builtin_ctz((unsigned)rows);
	for (int j = 0; j < rows; j++) {
		int cury = y + ((j * zoomy) >> rs);
		int zy = y + (((j + 1) * zoomy) >> rs) - cury;
		if (zy < 1 || cury > cy1 || cury + zy - 1 < cy0)
			continue;
		for (int k = 0; k < cols; k++) {
			int px = flipx ? cols - 1 - k : k, py = flipy ? rows - 1 - j : j;
			uint16_t code = map[px + py * cols];
			int curx = x + ((k * zoomx) >> cs);
			int zx = x + (((k + 1) * zoomx) >> cs) - curx;
			draw_chunk(bm, pitch, g, code, color, flipx, flipy, curx, cury, zx, zy, pmask, cy0, cy1);
		}
	}
}

// the sprites on screen lines cy0..cy1 (bm: the line s_vy0)
static void draw_sprites(uint16_t *bm, int pitch, int cy0, int cy1)
{
	static const uint32_t primasks[2] = { 0xf0, 0xfc };
	const uint16_t *spr = s_snap->spr;
	uint32_t map_size;
	const uint16_t *spritemap = (const uint16_t *)rbx_region("spritemap", &map_size);
	if (!spritemap)
		return;
	uint32_t map_words = map_size / 2;
	int words = s_m->spr_bytes / 2, y_offs = s_m->spr_yoffs;
	int first, last, step;
	switch (s_m->spr) {
	case SPR_CONTCIRC: case SPR_AQUAJACK: first = 0; last = words - 4; step = 4; break;
	case SPR_SPACEGUN: first = 0; last = words - 8; step = 4; break;
	case SPR_SCI: {
		int start = 0x800 - (s_snap->spriteframe & 1) * 0x800;
		first = start + 0x800 - 4; last = start; step = -4;
		break;
	}
	default: first = words - 4; last = 0; step = -4; break;
	}
	for (int offs = first; step > 0 ? offs <= last : offs >= last; offs += step) {
		const uint16_t *s = spr + offs;
		int zoomx, zoomy, x, y, flipx, flipy;
		uint32_t tilenum, color, priority;
		switch (s_m->spr) {
		case SPR_CONTCIRC:
			zoomy = (s[0] & 0xfe00) >> 9; y = s[0] & 0x1ff;
			tilenum = s[1] & 0x7ff;
			priority = (s[2] & 0x8000) >> 15; flipx = (s[2] & 0x4000) >> 14; flipy = (s[2] & 0x2000) >> 13;
			x = s[2] & 0x1ff;
			color = (s[3] & 0xff00) >> 8; zoomx = s[3] & 0x7f;
			break;
		case SPR_CHASEHQ:
			zoomy = (s[0] & 0xfe00) >> 9; y = s[0] & 0x1ff;
			priority = (s[1] & 0x8000) >> 15; color = (s[1] & 0x7f80) >> 7; zoomx = s[1] & 0x7f;
			flipy = (s[2] & 0x8000) >> 15; flipx = (s[2] & 0x4000) >> 14; x = s[2] & 0x1ff;
			tilenum = s[3] & 0x7ff;
			break;
		case SPR_BSHARK: case SPR_SCI:
			zoomy = (s[0] & 0x7e00) >> 9; y = s[0] & 0x1ff;
			priority = (s[1] & 0x8000) >> 15; color = (s[1] & 0x7f80) >> 7; zoomx = s[1] & 0x3f;
			flipy = (s[2] & 0x8000) >> 15; flipx = (s[2] & 0x4000) >> 14; x = s[2] & 0x1ff;
			tilenum = s[3] & 0x1fff;
			break;
		case SPR_AQUAJACK:
			zoomy = (s[0] & 0x7e00) >> 9; y = s[0] & 0x1ff;
			priority = (s[1] & 0x8000) >> 15; flipx = (s[1] & 0x4000) >> 14; x = s[1] & 0x1ff;
			color = (s[2] & 0xff00) >> 8; zoomx = s[2] & 0x3f;
			flipy = (s[3] & 0x8000) >> 15; tilenum = s[3] & 0x1fff;
			break;
		default:   // SPR_SPACEGUN
			zoomy = (s[0] & 0xfe00) >> 9; y = s[0] & 0x1ff;
			priority = (s[1] & 0x8000) >> 15; flipx = (s[1] & 0x4000) >> 14; x = s[1] & 0x1ff;
			color = (s[2] & 0xff00) >> 8; zoomx = s[2] & 0x7f;
			flipy = (s[3] & 0x8000) >> 15; tilenum = s[3] & 0x1fff;
			break;
		}
		if (!tilenum)
			continue;
		zoomx += 1;
		zoomy += 1;
		y += y_offs;
		uint32_t pm = primasks[priority];
		const sprgfx_t *g = &s_gfx[0];
		uint32_t map_offset;
		int cols, rows;
		switch (s_m->spr) {
		case SPR_CONTCIRC: y += 128 - zoomy; map_offset = tilenum << 7; cols = 8; rows = 16; break;
		case SPR_CHASEHQ:
			y += 128 - zoomy;
			if ((zoomx - 1) & 0x40) { map_offset = tilenum << 6; cols = 8; rows = 8; }   // 128x128, OBJA
			else if ((zoomx - 1) & 0x20) { map_offset = (tilenum << 5) + 0x20000; cols = 4; rows = 8; g = &s_gfx[1]; }
			else { map_offset = (tilenum << 4) + 0x30000; cols = 2; rows = 8; g = &s_gfx[1]; }    // 32x128
			break;
		case SPR_BSHARK: case SPR_SCI: y += 64 - zoomy; map_offset = tilenum << 5; cols = 4; rows = 8; break;
		default: map_offset = tilenum << 5; cols = 4; rows = 8; break;
		}
		if (x > 0x140) x -= 0x200;   // treat coords as signed
		if (y > 0x140) y -= 0x200;
		if (map_offset + (uint32_t)(cols * rows) > map_words || !g->pix)
			continue;
		draw_big(bm, pitch, g, spritemap + map_offset, cols, rows, x, y, zoomx, zoomy, flipx, flipy, color, pm, cy0, cy1);
	}
}

// ---- video (MAME screen_update_contcirc / chasehq / bshark / sci / aquajack / spacegun / dblaxle / racingb) ----
static void palette_update(void)
{
	const uint16_t *p = s_snap->pal;
	for (int i = 0; i < PAL_ENTRIES; i++) {
		int c = p[i], r, g, b;
		if (s_m->pal == PAL_PCR_XRGB) { r = (c >> 10) & 31; g = (c >> 5) & 31; b = c & 31; }
		else { r = c & 31; g = (c >> 5) & 31; b = (c >> 10) & 31; }
		r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
}

// the frame is drawn in bands of lines (tz_help: by the threads that help the render thread too)
#define BANDS 8
static int s_job_next, s_job_left, s_job_active, s_helpers;

static void draw_band(int band)
{
	const int h = s_m->vis_h, vy0 = s_m->vis_y0, pitch = rbx_video.pitch;
	const int r0 = h * band / BANDS, n = h * (band + 1) / BANDS - r0, y0 = vy0 + r0;   // (rows r0.., screen line y0..)
	uint16_t *bm = rbx_video.bitmap + r0 * pitch;
	uint8_t *pb = s_pribuf + r0 * pitch;
	for (int y = 0; y < n; y++)
		memset(bm + y * pitch, 0, SCREEN_W * 2);
	memset(pb, 0, (size_t)pitch * n);
	if (s_m->scr == SCR_SCN_ROAD || s_m->scr == SCR_SPACEGUN) {
		tc0100scn_t *scn = s_snap_scn;
		int bl = tc0100scn_bottomlayer(scn);
		int sg = s_m->scr == SCR_SPACEGUN;
		tc0100scn_draw(scn, bl, bm, pb, pitch, y0, n, (uint8_t)(sg ? 1 : 0));
		tc0100scn_draw(scn, bl ^ 1, bm, pb, pitch, y0, n, (uint8_t)(sg ? 2 : 1));
		if (!sg) {
			int paloffs = s_m->road_paloffs < 0 ? s_snap->road_palbank << 6 : s_m->road_paloffs;
			tc0150rod_draw(s_snap_rod, bm, pb, pitch, SCREEN_W, y0, n, s_m->road_yoffs, paloffs,
			               s_m->road_type, s_m->road_trans, 1, 2);
		}
		tc0100scn_draw(scn, 2, bm, pb, pitch, y0, n, 4);
		draw_sprites(rbx_video.bitmap, pitch, y0, y0 + n - 1);
	} else {
		tc0480scp_t *scp = s_snap_scp;
		int priority = tc0480scp_bg_priority(scp);
		int layer[4] = { (priority >> 12) & 15, (priority >> 8) & 15, (priority >> 4) & 15, priority & 15 };
		int rac = s_m->scr == SCR_RACINGB;
		tc0480scp_draw(scp, layer[0], bm, pb, pitch, y0, n, 0);
		tc0480scp_draw(scp, layer[1], bm, pb, pitch, y0, n, 0);
		tc0480scp_draw(scp, layer[2], bm, pb, pitch, y0, n, (uint8_t)(rac ? 2 : 1));
		if (rac)
			tc0480scp_draw(scp, layer[3], bm, pb, pitch, y0, n, 2);
		tc0150rod_draw(s_snap_rod, bm, pb, pitch, SCREEN_W, y0, n, -1, 0xc0, 0, 0, 1, 2);
		draw_sprites(rbx_video.bitmap, pitch, y0, y0 + n - 1);
		if (!rac)
			tc0480scp_draw(scp, layer[3], bm, pb, pitch, y0, n, 4);   // big numeric displays
		tc0480scp_draw(scp, 4, bm, pb, pitch, y0, n, (uint8_t)(rac ? 4 : 0));   // text layer
	}
	if (s_m->flipx)   // ORIENTATION_FLIP_X
		for (int y = 0; y < n; y++) {
			uint16_t *row = bm + y * pitch;
			for (int a = 0, b = SCREEN_W - 1; a < b; a++, b--) {
				uint16_t t = row[a];
				row[a] = row[b];
				row[b] = t;
			}
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

static int tz_help(int helper)
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

static void tz_render(void)
{
	palette_update();
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
	rbx_video_wait_idle();
	memcpy(s_snap->spr, s_spr, sizeof(s_snap->spr));
	memcpy(s_snap->pal, s_palram, sizeof(s_snap->pal));
	s_snap->road_palbank = s_road_palbank;
	s_snap->spriteframe = s_sci_spriteframe;
	if (s_scn)
		tc0100scn_copy(s_snap_scn, s_scn);
	if (s_scp)
		tc0480scp_copy(s_snap_scp, s_scp);
	tc0150rod_copy(s_snap_rod, s_rod);
	rbx_video_frame_done();
}

// ---- init / reset ----
static const machine_t *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
}

static int map_cpu(int n)
{
	m68k_bus_select(&s_bus[n]);
	m68k_bus_reset_map();
	for (const range_t *r = s_m->map[n]; r->dev; r++) {
		uint32_t len = r->end - r->start + 1;
		switch (r->dev) {
		case D_ROM: m68k_bus_map(r->start, r->end, s_rom[n], s_rom_size[n], 1, 0); break;
		case D_RAM:
			if (s_nram >= MAX_RAMS)
				return -1;
			s_ram[s_nram] = (uint8_t *)calloc(1, len);
			if (!s_ram[s_nram])
				return -1;
			s_ram_size[s_nram] = len;
			m68k_bus_map(r->start, r->end, s_ram[s_nram], len, 1, 1);
			s_nram++;
			break;
		case D_SHARE: m68k_bus_map(r->start, r->end, s_share, sizeof(s_share), 1, 1); break;
		case D_SPR: m68k_bus_map(r->start, r->end, s_spr, sizeof(s_spr), 1, 1); break;
		case D_SCN: m68k_bus_map(r->start, r->end, s_scn->ram, 0x10000, 1, 1); break;
		case D_ROD: m68k_bus_map(r->start, r->end, s_rod->ram, TC0150ROD_RAM_BYTES, 1, 1); break;
		case D_PAL: m68k_bus_map(r->start, r->end, s_palram, sizeof(s_palram), 1, 1); break;
		case D_SCP: m68k_bus_map(r->start, r->end, s_scp->ram, TC0480SCP_RAM_BYTES, 1, 1); break;
		}
	}
	m68k_bus_cur->read8 = bus_read8;
	m68k_bus_cur->read16 = bus_read16;
	m68k_bus_cur->write8 = bus_write8;
	m68k_bus_cur->write16 = bus_write16;
	return 0;
}

static void delay_find(int cpu);

static void tz_reset(void)
{
	for (int i = 0; i < s_nram; i++)
		memset(s_ram[i], 0, s_ram_size[i]);
	memset(s_share, 0, sizeof(s_share));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_io_regs, 0, sizeof(s_io_regs));
	s_ioc_port = 0;
	s_pcr_addr = 0;
	if (s_scn)
		tc0100scn_reset(s_scn);
	if (s_scp)
		tc0480scp_reset(s_scp);
	tc0150rod_reset(s_rod);
	s_road_palbank = 3;   // contcirc video_start
	s_sci_spriteframe = s_sci_int6 = 0;
	s_int6_cycles = -1;
	s_adc_addr = s_adc_sar = s_adc_eoc = 0;
	s_adc_cycles = -1;
	s_wheel = s_dial = 0;
	memset(s_stick, 0, sizeof(s_stick));
	for (int i = 0; i < 4; i++)
		s_pan[i] = 0x80;
	s_cpua_ctrl = 0xff;   // machine_reset: CPU B running
	s_sub_reset = 0;
	for (int n = 1; n >= 0; n--) {
		m68k_set_context(s_ctx[n]);
		m68k_bus_select(&s_bus[n]);
		s_cur = n;
		m68k_bus_irq_reset();
		m68k_pulse_reset();
		delay_find(n);
		m68k_get_context(s_ctx[n]);
	}
	if (s_m->z80)
		tsnd_reset();
	else {
		rbx_ym_reset(0);
		rbx_ym_set_volume(0, 0x80);
		s_ym_pending = 0;
		s_mix_n = 0;
	}
	s_frac[0] = s_frac[1] = s_ym_frac = 0;
}

static void tz_exit(void);

static int tz_init(const rbx_game *g)
{
	if (!s_bus_hp && !(s_bus_hp = calloc(1, sizeof(*s_bus_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	rbx_cheat_m68k_bus(&s_bus[0]);   // (the cheats: the main 68000's bus, CPU A)
	if (!s_share_hp && !(s_share_hp = calloc(1, sizeof(*s_share_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_spr_hp && !(s_spr_hp = calloc(1, sizeof(*s_spr_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_rom[0] = rbx_region("maincpu", &s_rom_size[0]);
	s_rom[1] = rbx_region("sub", &s_rom_size[1]);
	if (!s_rom[0] || !s_rom[1]) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom[0], s_rom_size[0]);
	m68k_bus_swap16(s_rom[1], s_rom_size[1]);
	s_vy0 = s_m->vis_y0;
	s_vy1 = s_m->vis_y0 + s_m->vis_h - 1;
	s_snap = calloc(1, sizeof(*s_snap));
	s_rod = (tc0150rod_t *)calloc(1, sizeof(tc0150rod_t));
	s_snap_rod = (tc0150rod_t *)calloc(1, sizeof(tc0150rod_t));
	if (!s_snap || !s_rod || !s_snap_rod)
		goto oom;
	uint32_t size;
	uint8_t *rodgfx = rbx_region("tc0150rod", &size);
	tc0150rod_init(s_rod, rodgfx, rodgfx ? size : 0);
	if (s_m->scr == SCR_DBLAXLE || s_m->scr == SCR_RACINGB) {
		uint8_t *gfx = rbx_region("tc0480scp", &size);
		s_scp = (tc0480scp_t *)calloc(1, sizeof(tc0480scp_t));
		s_snap_scp = (tc0480scp_t *)calloc(1, sizeof(tc0480scp_t));
		if (!s_scp || !s_snap_scp || tc0480scp_init(s_scp, gfx, gfx ? size : 0))
			goto oom;
		s_scp->x_offset = 0x1f;   // set_offsets(0x1f, 0x08)
		s_scp->y_offset = 0x08;
		*s_snap_scp = *s_scp;
	} else {
		uint8_t *gfx = rbx_region("tc0100scn", &size);
		s_scn = (tc0100scn_t *)calloc(1, sizeof(tc0100scn_t));
		s_snap_scn = (tc0100scn_t *)calloc(1, sizeof(tc0100scn_t));
		if (!s_scn || !s_snap_scn || tc0100scn_init(s_scn, gfx, gfx ? size : 0, TC0100SCN_4BPP))
			goto oom;
		s_scn->x_offset = s_m->scn_xoffs;
		s_scn->screen_w = SCREEN_W;
		s_scn->screen_h = s_vy1 + s_vy0 + 1;   // MAME flip mirror: visible bottom + top + 1
		*s_snap_scn = *s_scn;
	}
	if (s_m->spr == SPR_CHASEHQ) {
		if (decode_sprites(&s_gfx[0], "sprites", 16) || decode_sprites(&s_gfx[1], "sprites2", 16))
			goto oom;
	} else if (decode_sprites(&s_gfx[0], "sprites", 8))
		goto oom;
	static const char *const io_tags[8] = { "DSWA", "DSWB", "IN0", "IN1", 0, 0, 0, "IN2" };
	for (int i = 0; i < 8; i++)
		s_port_io[i] = io_tags[i] ? rbx_port_find(io_tags[i]) : -1;
	for (int i = 0; i < 4; i++) {
		char tag[8];
		snprintf(tag, sizeof(tag), "UNK%d", i + 1);
		s_port_unk[i] = rbx_port_find(tag);
	}
	s_port_adj[0] = rbx_port_find("X_ADJUST");
	s_port_adj[1] = rbx_port_find("Y_ADJUST");
	s_enforceja = s_m->kind == MZ_ENFORCE && rbx_port_find("STICKX") >= 0;
	// sound
	if (s_m->z80) {
		tsnd_config sc;
		memset(&sc, 0, sizeof(sc));
		sc.map = TSND_MAP_E000;
		sc.z80_clock = 4000000;
		sc.frame_rate_mhz = FRAME_MHZ;
		sc.lines = LINES;
		sc.bank_mask = 7;   // MAME sound_bankswitch_w: data & 7
		sc.ym.type = RBX_YM2610;
		sc.ym.clock = 8000000;
		sc.ym.gain[0] = s_m->gain_ssg;
		sc.ym.gain[1] = sc.ym.gain[2] = s_m->gain_fm;
		if (tsnd_init(&sc)) {
			rbx_fail("%s: sound init failed", g->set);
			tz_exit();
			return -1;
		}
	} else {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM2610;
		yc.clock = 8000000;
		yc.gain[0] = s_m->gain_ssg;
		yc.gain[1] = yc.gain[2] = s_m->gain_fm;
		yc.adpcma = rbx_region("ymsnd:adpcma", &yc.adpcma_size);
		yc.adpcmb = rbx_region("ymsnd:adpcmb", &yc.adpcmb_size);
		if (!yc.adpcmb) {
			yc.adpcmb = yc.adpcma;
			yc.adpcmb_size = yc.adpcma_size;
		}
		if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1))   // (its samples made on the worker)
			goto oom;
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 64;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)))
			goto oom;
	}
	// EEPROM (spacegun): default contents from the set, then the saved file
	s_has_eeprom = s_m->kind == MZ_SPACEGUN;
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	if (s_has_eeprom) {
		uint8_t *ee = rbx_region("eeprom", &size);
		for (uint32_t i = 0; ee && i < 64 && i * 2 + 1 < size; i++)
			s_eeprom.data[i] = rbx_be16(ee + i * 2);
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			eeprom_load(&s_eeprom, s_nv_path);
		}
	}
	if (rbx_video_setup(SCREEN_W, s_m->vis_h, PAL_ENTRIES))
		goto oom;
	s_pribuf = (uint8_t *)malloc((size_t)rbx_video.pitch * s_m->vis_h);
	if (!s_pribuf || m68k_context_size() > sizeof(s_ctx[0]))
		goto oom;
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_get_context(s_ctx[0]);
	memcpy(s_ctx[1], s_ctx[0], sizeof(s_ctx[0]));
	memset(s_bus, 0, sizeof(s_bus));
	s_nram = 0;
	if (map_cpu(0) || map_cpu(1))
		goto oom;
	tz_reset();
	cpu_switch(0);
	return 0;
oom:
	rbx_fail("%s: out of memory", g->set);
	tz_exit();
	return -1;
}

static void tz_exit(void)
{
	if (s_has_eeprom && s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_has_eeprom = 0;
	if (s_scn) tc0100scn_exit(s_scn);
	if (s_scp) tc0480scp_exit(s_scp);
	free(s_scn); free(s_snap_scn); free(s_scp); free(s_snap_scp); free(s_rod); free(s_snap_rod);
	s_scn = s_snap_scn = NULL;
	s_scp = s_snap_scp = NULL;
	s_rod = s_snap_rod = NULL;
	free(s_snap);
	s_snap = NULL;
	for (int i = 0; i < 2; i++) {
		free(s_gfx[i].pix);
		memset(&s_gfx[i], 0, sizeof(s_gfx[i]));
	}
	for (int i = 0; i < s_nram; i++)
		free(s_ram[i]);
	s_nram = 0;
	free(s_pribuf);
	s_pribuf = NULL;
	if (s_m && s_m->z80)
		tsnd_exit();
	else
		rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	m68k_bus_select(NULL);   // other boards use the default bus
	s_m = NULL;
	free(s_bus_hp);
	s_bus_hp = NULL;
	free(s_share_hp);
	s_share_hp = NULL;
	free(s_spr_hp);
	s_spr_hp = NULL;
}

// ---- frame ----
// ---- delay loops ----
// "addq.w #1,dN; cmp.w (aM),dN / cmpi.w #i,dN; bne" counts dN up to a value (spacegun's two CPUs wait in them, 62% of
// their instructions). At a slice's start a CPU at the addq does the passes Musashi would do in the slice at once: dN up
// by k, k passes' cycles. Within m68k_execute no interrupt comes (they come between slices) and the value compared is
// memory (not I/O): the passes are the same. The flags are set again by the next pass's addq / cmp before the bne
typedef struct { uint32_t pc; int dn, am, imm, cyc; } delay_loop;   // (am < 0: cmpi with imm)
#define DELAY_MAX 8
static delay_loop s_delay[2][DELAY_MAX];
static int s_ndelay[2];

static void delay_find(int cpu)   // (that CPU's bus selected)
{
	s_ndelay[cpu] = 0;
#ifndef __vita__
	if (getenv("RBX_NOIDLE"))   // (PC harness: Musashi only, to check the result)
		return;
#endif
	for (uint32_t a = 0; s_rom[cpu] && a + 8 <= s_rom_size[cpu] && s_ndelay[cpu] < DELAY_MAX; a += 2) {   // (through the bus: the ROM is kept in host order)
		const uint16_t op = (uint16_t)m68k_read_memory_16(a);
		if ((op & 0xfff8) != 0x5240)   // addq.w #1,dN
			continue;
		const int dn = op & 7;
		const uint16_t c = (uint16_t)m68k_read_memory_16(a + 2);
		delay_loop d = { a, dn, -1, 0, 0 };
		uint32_t bne;
		if ((c & 0xfff8) == (0xb050 | dn << 9)) {   // cmp.w (aM),dN; bne.s -6
			d.am = c & 7;
			bne = a + 4;
		} else if (c == (0x0c40 | dn)) {             // cmpi.w #i,dN; bne.s -8
			d.imm = (uint16_t)m68k_read_memory_16(a + 4);
			bne = a + 6;
		} else
			continue;
		if ((uint16_t)m68k_read_memory_16(bne) != (uint16_t)(0x6600 | (uint8_t)(a - (bne + 2))))
			continue;
		s_delay[cpu][s_ndelay[cpu]++] = d;
		rbx_log("taito_z: CPU %c delay loop at %06x", 'A' + cpu, a);
	}
}

// the passes of the current CPU's delay loop done here; the cycles used
static int delay_run(int cpu, int want)
{
	if (!s_ndelay[cpu])
		return 0;
	uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	delay_loop *d = NULL;
	for (int i = 0; i < s_ndelay[cpu]; i++)
		if (pc >= s_delay[cpu][i].pc && pc <= s_delay[cpu][i].pc + (s_delay[cpu][i].am >= 0 ? 4u : 6u))
			d = &s_delay[cpu][i];
	if (!d)
		return 0;
	int used = 0;
	while (pc != d->pc && used < want) {   // (in the loop past the addq: its cmp / bne by Musashi, back to the addq)
		used += m68k_execute(1);
		pc = m68k_get_reg(NULL, M68K_REG_PC);
		if (pc < d->pc || pc > d->pc + 6)
			return used;
	}
	if (pc != d->pc)
		return used;
	uint16_t target;
	if (d->am >= 0) {
		const uint32_t ea = m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_A0 + d->am)) & 0xfffffe;
		if (!m68k_bus_cur->rd[ea >> M68K_PAGE_BITS])   // (an I/O read: not skipped)
			return 0;
		target = (uint16_t)m68k_read_memory_16(ea);
	} else
		target = (uint16_t)d->imm;
	if (!d->cyc) {   // (a pass's cycles, from Musashi once)
		if (want - used < 200)
			return used;
		used += d->cyc = m68k_execute(1) + m68k_execute(1) + m68k_execute(1);
		if (m68k_get_reg(NULL, M68K_REG_PC) != d->pc) {   // (left the loop: measured again next time)
			d->cyc = 0;
			return used;
		}
	}
	const uint32_t dv = m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_D0 + d->dn));
	const uint32_t left = (uint16_t)(target - (uint16_t)dv) ? (uint16_t)(target - (uint16_t)dv) : 0x10000;   // (passes to the exit)
	int k = (want - used - 1) / d->cyc - 1;   // (one pass and more left to Musashi)
	if ((uint32_t)k > left - 1)
		k = (int)(left - 1);
	if (k <= 0)
		return used;
	m68k_set_reg((m68k_register_t)(M68K_REG_D0 + d->dn), (dv & 0xffff0000u) | (uint16_t)(dv + (uint32_t)k));
	return used + k * d->cyc;
}

static void exec(int cpu, int cycles)
{
	const int used = delay_run(cpu, cycles);
	if (used < cycles)
		m68k_execute(cycles - used);
}

static void run_cpus(int slice_lines)
{
	int cyc = rbx_line_cycles(&s_frac[0], s_m->clock, FRAME_MHZ, LINES * slice_lines);
	int cycb = rbx_line_cycles(&s_frac[1], s_m->clock, FRAME_MHZ, LINES * slice_lines);
	cpu_switch(0);
	exec(0, cyc);
	if (!s_sub_reset) {
		cpu_switch(1);
		exec(1, cycb);
	}
	if (s_m->adc_irq)
		adc_advance(cyc);
	if (s_int6_cycles >= 0) {
		s_int6_cycles -= cyc;
		if (s_int6_cycles < 0)
			cpu_hold(0, 6);   // MAME sci_state::trigger_int6
	}
}

static void tz_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	read_analog();
	int vblank = s_m->vis_y0 + s_m->vis_h;   // first line after the visible area
	for (int line = 0; line < LINES; line++) {
		if (line == vblank) {
			vblank_capture();
			if (s_m->kind == MZ_SCI || s_m->kind == MZ_RACINGB) {   // MAME sci_interrupt
				s_sci_int6 = !s_sci_int6;
				if (s_sci_int6)
					s_int6_cycles = 200000 - 500;
			}
			cpu_hold(0, s_m->irq_a);
			if (!s_sub_reset)
				cpu_hold(1, s_m->irq_b);
		}
		for (int s = 0; s < s_m->slices; s++)
			run_cpus(s_m->slices);
		if (s_m->z80)
			tsnd_line();
		else {
			s_ym_pending += (uint32_t)rbx_line_cycles(&s_ym_frac, 8000000, FRAME_MHZ, LINES);
			if ((line & 15) == 15)   // (the samples so far to the worker: made during the frame, not all at its end)
				ym_sync();
		}
	}
	cpu_switch(0);
	if (s_m->z80)
		tsnd_frame_end();
	else {
		ym_sync();
		rbx_ym_async_take(0, s_mix, s_mix_n);
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put(s_mix[i]);
		memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
		s_mix_n = 0;
	}
}

// ---- save states ----
static int tz_state(void *ctx, int load)
{
	cpu_switch(0);
	m68k_get_context(s_ctx[0]);
	uint32_t csize = m68k_context_size();
	uint32_t irq[4] = { s_bus[0].irq_pending, s_bus[0].irq_asserted, s_bus[1].irq_pending, s_bus[1].irq_asserted };
	int32_t misc[24] = { s_sub_reset, s_cpua_ctrl, s_ioc_port, s_pcr_addr, s_road_palbank, s_sci_spriteframe,
	                     s_sci_int6, s_int6_cycles, s_adc_addr, s_adc_sar, s_adc_eoc, s_adc_cycles, s_wheel, s_dial,
	                     s_stick[0][0], s_stick[0][1], s_stick[1][0], s_stick[1][1], s_pan[0], s_pan[1], s_pan[2],
	                     s_pan[3], (int32_t)s_ym_pending, 0 };
	uint64_t fr[3] = { s_frac[0], s_frac[1], s_ym_frac };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k_a", s_ctx[0], csize);
	rc |= rbx_state_io(ctx, "m68k_b", s_ctx[1], csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "ioregs", s_io_regs, sizeof(s_io_regs));
	for (int i = 0; i < s_nram; i++)
		rc |= rbx_state_io(ctx, "ram", s_ram[i], s_ram_size[i]);
	rc |= rbx_state_io(ctx, "share", s_share, sizeof(s_share));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "pal", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "rod", s_rod->ram, sizeof(s_rod->ram));
	if (s_scn) {
		rc |= rbx_state_io(ctx, "scn", s_scn->ram, sizeof(s_scn->ram));
		rc |= rbx_state_io(ctx, "scnc", s_scn->ctrl, sizeof(s_scn->ctrl));
	}
	if (s_scp) {
		rc |= rbx_state_io(ctx, "scp", s_scp->ram, sizeof(s_scp->ram));
		rc |= rbx_state_io(ctx, "scpc", s_scp->ctrl, sizeof(s_scp->ctrl));
		rc |= rbx_state_io(ctx, "scpp", &s_scp->pri_reg, sizeof(s_scp->pri_reg));
	}
	if (s_has_eeprom)
		rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	if (s_m->z80)
		rc |= tsnd_state(ctx, load);
	else
		rc |= rbx_ym_state(0, ctx, load);
	if (rc)
		return -1;
	if (load) {
		s_sub_reset = misc[0]; s_cpua_ctrl = (uint16_t)misc[1]; s_ioc_port = (uint8_t)misc[2];
		s_pcr_addr = (uint16_t)misc[3]; s_road_palbank = misc[4]; s_sci_spriteframe = misc[5]; s_sci_int6 = misc[6];
		s_int6_cycles = misc[7]; s_adc_addr = misc[8]; s_adc_sar = misc[9]; s_adc_eoc = misc[10];
		s_adc_cycles = misc[11]; s_wheel = misc[12]; s_dial = misc[13];
		s_stick[0][0] = misc[14]; s_stick[0][1] = misc[15]; s_stick[1][0] = misc[16]; s_stick[1][1] = misc[17];
		for (int i = 0; i < 4; i++)
			s_pan[i] = misc[18 + i];
		s_ym_pending = (uint32_t)misc[22];
		s_frac[0] = fr[0]; s_frac[1] = fr[1]; s_ym_frac = fr[2];
		s_bus[0].irq_pending = irq[0]; s_bus[0].irq_asserted = irq[1];
		s_bus[1].irq_pending = irq[2]; s_bus[1].irq_asserted = irq[3];
		if (s_scp)
			tc0480scp_post_load(s_scp);
		if (!s_m->z80)
			rbx_ym_set_volume(0, (s_pan[0] + s_pan[1] + s_pan[2] + s_pan[3] + 2) / 4);
		m68k_set_context(s_ctx[0]);
		m68k_bus_select(&s_bus[0]);
		s_cur = 0;
		m68k_bus_update_irq();
	}
	return 0;
}

static int tz_save(void *ctx) { return tz_state(ctx, 0); }
static int tz_load(void *ctx) { return tz_state(ctx, 1); }
static int tz_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_taito_z = { "Taito Z System", tz_init, tz_exit, tz_reset, tz_frame, tz_save, tz_load,
                                     tz_supported, tz_render, tz_help };
