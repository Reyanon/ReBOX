/*  The DECO Cassette System. Ported from MAME src/mame/dataeast/decocass.cpp, decocass_m.cpp, decocass_v.cpp,
    decocass_tape.cpp (GPL-2.0+, copyright-holders: Juergen Buchmueller, David Haywood).
    A DECO 222 at 750 kHz runs the game its BIOS loads from the cassette into RAM (an i8041 at 6 MHz reads the tape:
    blocks of 256 bytes with a CRC, the clock and data on its P2, the motor on its P1; the main CPU talks to it
    through its UPI host port), a 6502 at 500 kHz with two AY-3-8910 (an NMI every 16 lines once enabled), and the
    protection dongle of each game on the same E5xx window (type 1: a 32-byte PROM with latched and passed bits,
    type 2: a PROM paged by a command, type 3: a 4K ROM read by a counter with the status bits swapped by a PAL,
    type 4: a 32K ROM, type 5: a fixed value, the multi-game / DarkSoft boards: a 1 MB ROM).
    Video: 1024 RAM characters (3bpp, 8x8) and 8 RAM sprites (16x16) from the same 24K, two 16x16 background
    "edge" tilemaps (the left and right halves of the road, sharing the tile RAM), a 4-line center stripe, 16
    missiles, a 64x64x2 object (the headlight / tunnel) lighting pen bit 4 up, 32 colours from a palette RAM.
    The treasure island and explorer boards add a DE-0091 ROM daughterboard banked over 6000-afff.
    Not emulated: the watchdog (as games feed it), the old board's tape timing differences.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

#define MASTER 12000000
#define FRAME_MHZ 57444          // set_raw(6 MHz, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define MCU_RATE (MASTER / 2 / 15)   // the i8041's machine cycles per second

enum { D_NONE, D_TYPE1, D_TYPE2, D_TYPE3, D_TYPE4, D_TYPE5, D_NODONG, D_WIDEL, D_DARKSOFT };
enum { T1PROM = 1, T1DIRECT = 2, T1LATCH = 4, T1LATCHINV = 8 };
enum { SWAP_NONE, SWAP_01, SWAP_12, SWAP_13, SWAP_24, SWAP_25, SWAP_34_0, SWAP_34_7, SWAP_45, SWAP_23_56, SWAP_56,
       SWAP_67 };

// ---- tape (decocass_tape.cpp) ----
#define TAPE_CLOCKRATE 4800
#define MS_CLOCKS(x) ((x) * TAPE_CLOCKRATE / 1000)
#define LEADER_END MS_CLOCKS(1000)
#define LEADER_GAP_END (LEADER_END + MS_CLOCKS(1500))
#define BOT_END (LEADER_GAP_END + 12)            // (2.5 ms: 12 clocks)
#define BOT_GAP_END (BOT_END + MS_CLOCKS(300))
enum { R_LEADER, R_LEADER_GAP, R_BOT, R_BOT_GAP, R_DATA0, R_EOT_GAP = R_DATA0 + 256, R_EOT, R_TRAILER_GAP, R_TRAILER };
enum { B_PRE_GAP0 = 0, B_LEADIN = 34, B_HEADER, B_DATA0, B_CRC_MSB = B_DATA0 + 256, B_CRC_LSB, B_TRAILER, B_LEADOUT,
       B_LONGCLOCK, B_POSTGAP0, B_TOTAL = B_POSTGAP0 + 34 };

static const uint16_t s_tile_offset[32 * 32] = {
#define R8(b) b+0,b+1,b+2,b+3,b+4,b+5,b+6,b+7
#define F8(b) b+7,b+6,b+5,b+4,b+3,b+2,b+1,b+0
#define ROW(o) R8(0x078 - (o)), F8(0x0f8 - (o)), R8(0x278 - (o)), F8(0x2f8 - (o))
#define ROWB(o) R8(0x178 - (o)), F8(0x1f8 - (o)), R8(0x378 - (o)), F8(0x3f8 - (o))
	ROW(0x00), ROW(0x08), ROW(0x10), ROW(0x18), ROW(0x20), ROW(0x28), ROW(0x30), ROW(0x38),
	ROW(0x40), ROW(0x48), ROW(0x50), ROW(0x58), ROW(0x60), ROW(0x68), ROW(0x70), ROW(0x78),
	ROWB(0x00), ROWB(0x08), ROWB(0x10), ROWB(0x18), ROWB(0x20), ROWB(0x28), ROWB(0x30), ROWB(0x38),
	ROWB(0x40), ROWB(0x48), ROWB(0x50), ROWB(0x58), ROWB(0x60), ROWB(0x68), ROWB(0x70), ROWB(0x78),
#undef R8
#undef F8
#undef ROW
#undef ROWB
};

static uint8_t *s_rom, *s_srom, *s_mrom, *s_dongle, *s_tape, *s_user3;
static uint32_t s_rom_size, s_srom_size, s_mrom_size, s_dongle_size, s_tape_size, s_user3_size;
static int s_dongle_type, s_crom, s_telejan;
static const uint8_t *s_t1map;
static uint32_t s_t1in, s_t1out;
static int s_t3swap;

static uint8_t s_ram[0x6000], s_chr[0x6000], s_fg[0x400], s_col[0x400], s_tile[0x800], s_obj[0x400], s_pal[0x100];
static uint8_t s_sram[0x1000];
// machine state (saved as one block)
static struct {
	uint8_t watchdog_flip, color_missiles, color_center_bot, mode_set, back_h_shift, back_vl_shift, back_vr_shift,
	        part_h_shift, part_v_shift, center_h_shift_space, center_v_shift;
	uint8_t sound_ack, audio_nmi_enabled, audio_nmi_state, decocass_reset, quad[4], i8041_p1, i8041_p2, mux_data;
	uint8_t latch, latch2, firsttime, latch1, main_nmi, coin_last, snd_reset, mcu_reset, rom_bank;
	uint8_t t2_d2, t2_xx, t2_addr, t3_d0, t3_pal19, t4_latch, t5_latch, widel_latch;
	uint8_t palc[32];   // the 32 colours (a palette RAM write: colour (offset & 31) ^ 16)
	uint32_t t3_ctrs, t4_ctrs, widel_ctrs, dark_addr;
	// tape
	int32_t tape_speed, tape_region, tape_byte, tape_bit, pad0;
	uint32_t tape_clockpos;
	uint64_t tape_frac, mcu_time;
} s_m;
static uint32_t s_tape_numclocks;
static uint16_t s_crc16[256];
static int s_port_in[3], s_port_an[4], s_port_dsw[2], s_port_mp[2][4];
static int s_line;               // (from the vblank start, as MAME's screen at time 0)
static uint32_t s_frame;
static int64_t s_cpu_target, s_snd_target;
static uint64_t s_cpu_frac, s_snd_frac, s_mcu_frac, s_ay_frac;
static int s_mcu_avail;
static uint64_t s_exec_base;
static int s_exec_req;
static m6502_t s_cpu, s_snd;
static mcs48 s_mcu;
static ay8910_t s_ay[2];
static int32_t *s_mix;
static int s_mix_cap;
static struct {
	uint8_t chr[0x6000], fg[0x400], col[0x400], tile[0x800], obj[0x400], pal[0x100];
	uint8_t watchdog_flip, color_missiles, color_center_bot, mode_set, back_h_shift, back_vl_shift, back_vr_shift,
	        part_h_shift, part_v_shift, center_h_shift_space, center_v_shift, flip_dsw;
} *s_snap;

static int vblank(void) { return s_frame && s_line < LINES - VIS_H; }

// ---------------------------------------------------------------- tape
static uint16_t crc16_byte(uint16_t crc, uint8_t data)
{
	for (int bit = 0; bit < 8; bit++) {
		crc = (uint16_t)((crc >> 1) | (crc << 15));
		crc ^= (data << 7) & 0x80;
		if (crc & 0x80)
			crc ^= 0x0120;
		data >>= 1;
	}
	return crc;
}

static void tape_init(void)
{
	int offs;
	for (offs = (int)s_tape_size - 1; offs >= 0; offs--)
		if (s_tape[offs] != 0)
			break;
	int numblocks = ((offs | 0xff) + 1) / 256;
	if (numblocks > 255) numblocks = 255;
	s_tape_numclocks = (uint32_t)(BOT_GAP_END + numblocks * B_TOTAL * 16 + BOT_GAP_END);
	// the pair of bytes bringing each block's CRC to 0 (MAME's brute force: the lowest; the CRC is linear, so
	// crc(crc(c, hi), lo) = crc(crc(c, hi), 0) ^ crc(0, lo) and lo comes from the inverse of crc(0, .))
	static int inv[0x10000];   // (not on the stack: 256K)
	for (int i = 0; i < 0x10000; i++)
		inv[i] = -1;
	for (int lo = 255; lo >= 0; lo--)
		inv[crc16_byte(0, (uint8_t)lo)] = lo;
	for (int b = 0; b < numblocks; b++) {
		uint16_t crc = 0;
		for (int o = 256 * b; o < 256 * b + 256; o++)
			crc = crc16_byte(crc, (uint32_t)o < s_tape_size ? s_tape[o] : 0);
		int found = 0x10000;
		for (int hi = 0; hi < 256 && found == 0x10000; hi++) {
			const int lo = inv[crc16_byte(crc16_byte(crc, (uint8_t)hi), 0)];
			if (lo >= 0)
				found = hi << 8 | lo;
		}
		s_crc16[b] = (uint16_t)found;
	}
}

static void tape_region_update(void)
{
	const uint32_t p = s_m.tape_clockpos, n = s_tape_numclocks;
	if (p < LEADER_END) s_m.tape_region = R_LEADER;
	else if (p < LEADER_GAP_END) s_m.tape_region = R_LEADER_GAP;
	else if (p < BOT_END) s_m.tape_region = R_BOT;
	else if (p < BOT_GAP_END) s_m.tape_region = R_BOT_GAP;
	else if (p >= n - LEADER_END) s_m.tape_region = R_TRAILER;
	else if (p >= n - LEADER_GAP_END) s_m.tape_region = R_TRAILER_GAP;
	else if (p >= n - BOT_END) s_m.tape_region = R_EOT;
	else if (p >= n - BOT_GAP_END) s_m.tape_region = R_EOT_GAP;
	else {
		uint32_t dc = p - BOT_GAP_END;
		const uint32_t blk = dc / (16 * B_TOTAL);
		s_m.tape_region = (int32_t)(R_DATA0 + blk);
		dc -= blk * 16 * B_TOTAL;
		s_m.tape_byte = (int32_t)(dc / 16);
		dc -= (uint32_t)s_m.tape_byte * 16;
		s_m.tape_bit = (int32_t)(dc / 2);
	}
}

// the tape timer up to the i8041's time 'now' (in its machine cycles)
static void tape_advance(uint64_t now)
{
	if (now <= s_m.mcu_time)
		return;
	const uint64_t dt = now - s_m.mcu_time;
	s_m.mcu_time = now;
	if (!s_m.tape_speed)
		return;
	const int sp = s_m.tape_speed < 0 ? -s_m.tape_speed : s_m.tape_speed;
	s_m.tape_frac += dt * (uint64_t)(TAPE_CLOCKRATE * sp);
	while (s_m.tape_frac >= MCU_RATE) {
		s_m.tape_frac -= MCU_RATE;
		if (s_m.tape_speed < 0 && s_m.tape_clockpos > 0)
			s_m.tape_clockpos--;
		else if (s_m.tape_speed > 0 && s_m.tape_clockpos < s_tape_numclocks)
			s_m.tape_clockpos++;
		tape_region_update();
	}
}

static uint8_t tape_status(void)
{
	uint8_t bits = 0;
	const int r = s_m.tape_region;
	if (r == R_LEADER || r == R_BOT || r == R_EOT || r == R_TRAILER)
		bits |= 0x20;
	if (r >= R_DATA0 && r < R_DATA0 + 256) {
		const int blk = r - R_DATA0, by = s_m.tape_byte;
		uint8_t v = 0;
		if (by >= B_LEADIN && by <= B_LEADOUT)
			bits |= ((s_m.tape_clockpos - BOT_GAP_END) & 1) ? 0x00 : 0x40;
		else if (by == B_LONGCLOCK)
			bits |= 0x40;
		if (by == B_HEADER || by == B_TRAILER)
			v = 0xaa;
		else if (by >= B_DATA0 && by < B_DATA0 + 256) {
			const uint32_t o = (uint32_t)blk * 256 + (uint32_t)(by - B_DATA0);
			v = o < s_tape_size ? s_tape[o] : 0;
		} else if (by == B_CRC_MSB)
			v = (uint8_t)(s_crc16[blk] >> 8);
		else if (by == B_CRC_LSB)
			v = (uint8_t)s_crc16[blk];
		if ((v >> s_m.tape_bit) & 1)
			bits |= 0x80;
	}
	return bits;
}

static void tape_change_speed(int sp)
{
	if (s_m.tape_speed == sp)
		return;
	s_m.tape_speed = sp;
	s_m.tape_frac = 0;   // (the timer adjusted: the first clock a period from now)
}

// ---------------------------------------------------------------- i8041
static uint64_t mcu_now(void) { return s_exec_base + (uint64_t)(s_exec_req - s_mcu.icount); }

static uint8_t mcu_port_r(void *u, int port)
{
	(void)u;
	if (port == 1)
		return s_m.i8041_p1;
	tape_advance(mcu_now());
	return (uint8_t)((s_m.i8041_p2 & ~0xe0) | tape_status());
}
static void mcu_port_w(void *u, int port, uint8_t v)
{
	(void)u;
	if (port == 1) {
		if ((v ^ s_m.i8041_p1) & 0x34) {   // the FAST / REW / FWD signals
			int sp = 0;
			if ((v & 0x30) == 0x20)
				sp = (v & 0x04) ? -1 : -7;
			else if ((v & 0x30) == 0x10)
				sp = (v & 0x04) ? 1 : 7;
			tape_advance(mcu_now());
			tape_change_speed(sp);
		}
		s_m.i8041_p1 = v;
	} else
		s_m.i8041_p2 = (uint8_t)((s_m.i8041_p2 & 0xe0) | (v & ~0xe0));
}

// ---------------------------------------------------------------- dongles
#define BIT(v, b) (((v) >> (b)) & 1)
#define T1MAP(x, m) (((m) >> ((x) * 3)) & 7)
static uint32_t make_map(int m0, int m1, int m2, int m3, int m4, int m5, int m6, int m7)
{
	return (uint32_t)(m0 | (m1 << 3) | (m2 << 6) | (m3 << 9) | (m4 << 12) | (m5 << 15) | (m6 << 18) | (m7 << 21));
}

static uint8_t mcu_r(int offset) { return (offset & 2) ? 0xff : upi41_master_r(&s_mcu, offset & 1); }

static uint8_t type1_r(int offset)
{
	if (!s_t1map)
		return 0;
	uint8_t data;
	if (offset & 1) {
		data = mcu_r(1);
		return (uint8_t)((BIT(data, 0) << 0) | (BIT(data, 1) << 1) | 0x7c);
	}
	if (s_m.firsttime) {
		s_m.firsttime = 0;
		s_m.latch1 = 0;
	}
	data = mcu_r(0);
	const uint8_t save = data;
	uint32_t promaddr = 0;
	int shift = 0;
	for (int i = 0; i < 8; i++)
		if (s_t1map[i] == T1PROM)
			promaddr |= (uint32_t)(((data >> T1MAP(i, s_t1in)) & 1) << shift++);
	data = 0;
	shift = 0;
	const uint8_t p = promaddr < s_dongle_size ? s_dongle[promaddr] : 0;
	for (int i = 0; i < 8; i++) {
		if (s_t1map[i] == T1PROM) data |= (uint8_t)(((p >> shift++) & 1) << T1MAP(i, s_t1out));
		if (s_t1map[i] == T1LATCHINV) data |= (uint8_t)((1 - ((s_m.latch1 >> T1MAP(i, s_t1in)) & 1)) << T1MAP(i, s_t1out));
		if (s_t1map[i] == T1LATCH) data |= (uint8_t)(((s_m.latch1 >> T1MAP(i, s_t1in)) & 1) << T1MAP(i, s_t1out));
		if (s_t1map[i] == T1DIRECT) data |= (uint8_t)(((save >> T1MAP(i, s_t1in)) & 1) << T1MAP(i, s_t1out));
	}
	s_m.latch1 = save;
	return data;
}

static uint8_t type2_r(int offset)
{
	if (s_m.t2_xx) {
		if (offset & 1) {
			const uint32_t a = 256u * s_m.t2_d2 + s_m.t2_addr;
			return a < s_dongle_size ? s_dongle[a] : 0;
		}
		return 0xff;
	}
	return (offset & 2) ? (uint8_t)offset : upi41_master_r(&s_mcu, offset & 1);
}
static void type2_w(int offset, uint8_t data)
{
	if (s_m.t2_xx && !(offset & 1)) {
		s_m.t2_addr = data;
		return;
	}
	if ((offset & 1) && (data & 0xf0) == 0xc0) {
		s_m.t2_xx = 1;
		s_m.t2_d2 = (data & 0x04) ? 1 : 0;
	}
	upi41_master_w(&s_mcu, offset & 1, data);
}

static uint8_t type3_swap(uint8_t s)
{
	const uint8_t d0 = s_m.t3_d0;
	switch (s_t3swap) {
	case SWAP_01: return (uint8_t)(BIT(s, 1) | d0 << 1 | (s & 0xfc));
	case SWAP_12: return (uint8_t)(d0 | BIT(s, 2) << 1 | BIT(s, 1) << 2 | (s & 0xf8));
	case SWAP_13: return (uint8_t)(d0 | BIT(s, 3) << 1 | BIT(s, 2) << 2 | BIT(s, 1) << 3 | (s & 0xf0));
	case SWAP_24: return (uint8_t)(d0 | BIT(s, 1) << 1 | BIT(s, 4) << 2 | BIT(s, 3) << 3 | BIT(s, 2) << 4 | (s & 0xe0));
	case SWAP_25: return (uint8_t)(d0 | BIT(s, 1) << 1 | BIT(s, 5) << 2 | BIT(s, 3) << 3 | BIT(s, 4) << 4 |
	                               BIT(s, 2) << 5 | (s & 0xc0));
	case SWAP_34_0: return (uint8_t)(d0 | (s & 0x06) | BIT(s, 3) << 4 | BIT(s, 4) << 3 | (s & 0xe0));
	case SWAP_34_7: return (uint8_t)(BIT(s, 7) | (s & 0x06) | BIT(s, 4) << 3 | BIT(s, 3) << 4 | (s & 0x60) | d0 << 7);
	case SWAP_45: return (uint8_t)(d0 | (s & 0x0e) | BIT(s, 5) << 4 | BIT(s, 4) << 5 | (s & 0xc0));
	case SWAP_23_56: return (uint8_t)(d0 | BIT(s, 1) << 1 | BIT(s, 3) << 2 | BIT(s, 2) << 3 | BIT(s, 4) << 4 |
	                                  BIT(s, 6) << 5 | BIT(s, 5) << 6 | (s & 0x80));
	case SWAP_56: return (uint8_t)(d0 | (s & 0x1e) | BIT(s, 6) << 5 | BIT(s, 5) << 6 | (s & 0x80));
	case SWAP_67: return (uint8_t)(d0 | (s & 0x3e) | BIT(s, 7) << 6 | BIT(s, 6) << 7);
	}
	return (uint8_t)(d0 | (s & 0xfe));
}
static uint8_t type3_r(int offset)
{
	if (offset & 1) {
		if (s_m.t3_pal19) {
			const uint8_t d = s_m.t3_ctrs < s_dongle_size ? s_dongle[s_m.t3_ctrs] : 0;
			if (++s_m.t3_ctrs == 4096)
				s_m.t3_ctrs = 0;
			return d;
		}
		return mcu_r(1);
	}
	if (s_m.t3_pal19)
		return 0xff;
	uint8_t data;
	if (!(offset & 2)) {
		const uint8_t save = upi41_master_r(&s_mcu, 0);
		data = type3_swap(save);
		s_m.t3_d0 = save & 1;
	} else {   // (open bus: bits 6 and 7 swapped)
		data = (uint8_t)(s_m.t3_d0 | 0x3e | 0xc0);
		s_m.t3_d0 = 1;
	}
	return data;
}
static void type3_w(int offset, uint8_t data)
{
	if (offset & 1) {
		if (s_m.t3_pal19) {
			s_m.t3_ctrs = (uint32_t)data << 4;
			return;
		}
		if ((data & 0xf0) == 0xc0)
			s_m.t3_pal19 = 1;
	} else if (s_m.t3_pal19)
		return;
	upi41_master_w(&s_mcu, offset & 1, data);
}

static uint8_t type4_r(int offset)
{
	if (offset & 1)
		return mcu_r(1);
	if (s_m.t4_latch) {
		const uint8_t d = s_m.t4_ctrs < s_dongle_size ? s_dongle[s_m.t4_ctrs] : 0;
		s_m.t4_ctrs = (s_m.t4_ctrs + 1) & 0x7fff;
		return d;
	}
	return mcu_r(0);
}
static void type4_w(int offset, uint8_t data)
{
	if (offset & 1) {
		if (s_m.t4_latch) {
			s_m.t4_ctrs = (s_m.t4_ctrs & 0x00ff) | ((uint32_t)(data & 0x7f) << 8);
			return;
		}
		if ((data & 0xf0) == 0xc0)
			s_m.t4_latch = 1;
	} else if (s_m.t4_latch) {
		s_m.t4_ctrs = (s_m.t4_ctrs & 0xff00) | data;
		return;
	}
	upi41_master_w(&s_mcu, offset & 1, data);
}

static uint8_t type5_r(int offset)
{
	if (offset & 1)
		return mcu_r(1);
	return s_m.t5_latch ? 0x55 : mcu_r(0);
}
static void type5_w(int offset, uint8_t data)
{
	if (offset & 1) {
		if (s_m.t5_latch)
			return;
		if ((data & 0xf0) == 0xc0)
			s_m.t5_latch = 1;
	} else if (s_m.t5_latch)
		return;
	upi41_master_w(&s_mcu, offset & 1, data);
}

static uint8_t widel_r(int offset)
{
	if (offset & 1) {
		if (!(offset & 2)) {
			if (s_m.widel_latch)
				s_m.widel_ctrs = (s_m.widel_ctrs + 0x100) & 0xfffff;
			return upi41_master_r(&s_mcu, 1);
		}
		return 0xff;
	}
	if (s_m.widel_latch) {
		const uint8_t d = s_m.widel_ctrs < s_dongle_size ? s_dongle[s_m.widel_ctrs] : 0;
		s_m.widel_ctrs = (s_m.widel_ctrs + 1) & 0xfffff;
		return d;
	}
	return mcu_r(0);
}
static void widel_w(int offset, uint8_t data)
{
	if (offset & 1) {
		if (s_m.widel_latch) {
			s_m.widel_ctrs = 0;
			return;
		}
		if ((data & 0xf0) == 0xc0)
			s_m.widel_latch = 1;
	} else if (s_m.widel_latch) {
		s_m.widel_ctrs = (s_m.widel_ctrs & 0xfff00) | data;
		return;
	}
	upi41_master_w(&s_mcu, offset & 1, data);
}

static uint8_t darksoft_r(int offset)
{
	const uint32_t a = s_m.dark_addr & 0xfffff;
	const uint8_t d = a < s_dongle_size ? s_dongle[a] : 0;
	if (offset == 0) {
		s_m.dark_addr++;
		return d;
	}
	if (offset == 1) {
		s_m.dark_addr += 0x100;
		return d;
	}
	return 0xff;
}

static uint8_t e5xx_r(int offset)
{
	if ((offset & 2) == 2) {   // the 8041's port bits, the tape's BOT/EOT, the cassette present
		const uint8_t bot_eot = (tape_status() >> 5) & 1;
		return (uint8_t)(BIT(s_m.i8041_p1, 7) | BIT(s_m.i8041_p2, 0) << 1 | BIT(s_m.i8041_p2, 1) << 2 |
		                 BIT(s_m.i8041_p2, 2) << 3 | bot_eot << 4 | 0x60 | (s_tape ? 0 : 0x80));
	}
	switch (s_dongle_type) {
	case D_TYPE1: return type1_r(offset);
	case D_TYPE2: return type2_r(offset);
	case D_TYPE3: return type3_r(offset);
	case D_TYPE4: return type4_r(offset);
	case D_TYPE5: return type5_r(offset);
	case D_NODONG: return mcu_r(offset);
	case D_WIDEL: return widel_r(offset);
	case D_DARKSOFT: return darksoft_r(offset);
	}
	return 0xff;
}
static void e5xx_w(int offset, uint8_t data)
{
	switch (s_dongle_type) {
	case D_TYPE2: type2_w(offset, data); return;
	case D_TYPE3: type3_w(offset, data); return;
	case D_TYPE4: type4_w(offset, data); return;
	case D_TYPE5: type5_w(offset, data); return;
	case D_WIDEL: widel_w(offset, data); return;
	case D_DARKSOFT:
		if (!(offset & 2))
			s_m.dark_addr = data;
		return;
	}
	if (!(offset & 2))
		upi41_master_w(&s_mcu, offset & 1, data);
}

// ---------------------------------------------------------------- sound CPU
static void audio_nmi_update(void)
{
	m6502_set_nmi(&s_snd, s_m.audio_nmi_enabled && s_m.audio_nmi_state);
}

static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x1000) return s_sram[a];
	if (a < 0x1800) {   // sound_nmi_enable_r
		s_m.audio_nmi_enabled = 1;
		audio_nmi_update();
		return 0xff;
	}
	if (a < 0x2000) {   // sound_data_ack_reset_r
		s_m.sound_ack &= (uint8_t)~0x40;
		return 0xff;
	}
	if (a >= 0xa000 && a < 0xb000) {   // sound_command_r
		m6502_set_irq(&s_snd, 0);
		s_m.sound_ack &= (uint8_t)~0x80;
		return s_m.latch;
	}
	if (a >= 0xf800) return s_srom[a];
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x1000) s_sram[a] = v;
	else if (a < 0x1800) {
		s_m.audio_nmi_enabled = 1;
		audio_nmi_update();
	} else if (a < 0x2000) s_m.sound_ack &= (uint8_t)~0x40;
	else if (a >= 0x2000 && a < 0x3000) ay8910_data_w(&s_ay[0], v);
	else if (a >= 0x4000 && a < 0x5000) ay8910_address_w(&s_ay[0], v);
	else if (a >= 0x6000 && a < 0x7000) ay8910_data_w(&s_ay[1], v);
	else if (a >= 0x8000 && a < 0x9000) ay8910_address_w(&s_ay[1], v);
	else if (a >= 0xc000 && a < 0xd000) {   // sound_data_w
		s_m.latch2 = v;
		s_m.sound_ack |= 0x40;
	}
}

// ---------------------------------------------------------------- main CPU
static void reset_w(uint8_t v)
{
	s_m.decocass_reset = v;
	if ((v & 1) && !s_m.snd_reset) {   // the sound CPU's active high reset (the audio NMI disabled too)
		m6502_reset(&s_snd);
		m6502_set_irq(&s_snd, 0);
	}
	s_m.snd_reset = v & 1;
	if (v & 1) {
		s_m.audio_nmi_enabled = 0;
		audio_nmi_update();
	}
	if ((v & 8) && !s_m.mcu_reset)   // the 8041's reset
		mcs48_reset(&s_mcu);
	s_m.mcu_reset = (v >> 3) & 1;
}

static uint8_t input_r(int offset)
{
	if (s_telejan && !(offset & 6))   // the DS Telejan mahjong panel
		return s_port_mp[offset & 1][s_m.mux_data] >= 0 ? (uint8_t)rbx_port_read(s_port_mp[offset & 1][s_m.mux_data])
		                                              : 0xff;
	switch (offset & 7) {
	case 0: case 1: case 2: return (uint8_t)rbx_port_read(s_port_in[offset & 7]);
	case 3: case 4: case 5: case 6: return s_m.quad[(offset & 7) - 3];
	}
	return 0xff;
}

static uint8_t main_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x6000) return s_ram[a];
	if (a < 0xc000) {
		if (s_crom && a < 0xb000 && s_m.rom_bank) {   // the DE-0091 ROM board
			const uint32_t o = (uint32_t)(s_m.rom_bank - 1) * 0x5000 + (uint32_t)(a - 0x6000);
			return o < s_user3_size ? s_user3[o] : 0xff;
		}
		return s_chr[a - 0x6000];
	}
	if (a < 0xc400) return s_fg[a - 0xc000];
	if (a < 0xc800) return s_col[a - 0xc400];
	if (a < 0xd000) {   // the swizzled mirrors
		const uint32_t o = ((a >> 5) & 0x1f) | ((a & 0x1f) << 5);
		return a < 0xcc00 ? s_fg[o] : s_col[o];
	}
	if (a < 0xd800) return s_tile[a - 0xd000];
	if (a < 0xdc00) return s_obj[a - 0xd800];
	if (a >= 0xf000) return s_rom[a];
	if (a >= 0xe000 && a < 0xe100) return s_pal[a - 0xe000];
	if (a == 0xe300) {
		rbx_port_custom(s_port_dsw[0], 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_dsw[0]);
	}
	if (a == 0xe301) return (uint8_t)rbx_port_read(s_port_dsw[1]);
	if (a == 0xe414) return 0xc0;   // sound_command_main_r
	if (a >= 0xe500 && a < 0xe600) return e5xx_r(a & 0xff);
	if (a >= 0xe600 && a < 0xe700) return input_r(a & 0xff);
	if (a == 0xe700) return s_m.latch2;
	if (a == 0xe701) return s_m.sound_ack;
	return 0;
}

static void main_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x6000) { s_ram[a] = v; return; }
	if (a < 0xc000) { s_chr[a - 0x6000] = v; return; }
	if (a < 0xc400) { s_fg[a - 0xc000] = v; return; }
	if (a < 0xc800) { s_col[a - 0xc400] = v; return; }
	if (a < 0xd000) {
		const uint32_t o = ((a >> 5) & 0x1f) | ((a & 0x1f) << 5);
		if (a < 0xcc00) s_fg[o] = v; else s_col[o] = v;
		return;
	}
	if (a < 0xd800) { s_tile[a - 0xd000] = v; return; }
	if (a < 0xdc00) { s_obj[a - 0xd800] = v; return; }
	if (a >= 0xe000 && a < 0xe100) {
		s_pal[a - 0xe000] = v;
		s_m.palc[(a & 31) ^ 16] = v;
		return;
	}
	switch (a) {
	case 0xe301: s_m.watchdog_flip = v; return;
	case 0xe302: s_m.color_missiles = v & 0x77; return;
	case 0xe400: reset_w(v); return;
	case 0xe402: s_m.mode_set = v; return;
	case 0xe403: s_m.back_h_shift = v; return;
	case 0xe404: s_m.back_vl_shift = v; return;
	case 0xe405: s_m.back_vr_shift = v; return;
	case 0xe406: s_m.part_h_shift = v; return;
	case 0xe407: s_m.part_v_shift = v; return;
	case 0xe410: s_m.color_center_bot = v; return;
	case 0xe411: s_m.center_h_shift_space = v; return;
	case 0xe412: s_m.center_v_shift = v; return;
	case 0xe413:
		if (s_telejan)
			s_m.mux_data = (v & 0xc) >> 2;
		return;
	case 0xe414:   // sound_command_w
		s_m.latch = v;
		s_m.sound_ack |= 0x80;
		s_m.sound_ack &= (uint8_t)~0x40;
		m6502_set_irq(&s_snd, 1);
		return;
	case 0xe415: case 0xe416:   // the quadrature decoders latched
		for (int i = 0; i < 4; i++)
			s_m.quad[i] = s_port_an[i] >= 0 ? (uint8_t)rbx_port_read(s_port_an[i]) : 0x80;
		return;
	case 0xe417:   // nmi_reset_w
		s_m.main_nmi = 0;
		m6502_set_nmi(&s_cpu, 0);
		return;
	case 0xe900:
		if (s_crom && (v & 3) != 3)
			s_m.rom_bank = v & 3;
		return;
	}
	if (a >= 0xe500 && a < 0xe600)
		e5xx_w(a & 0xff, v);
}

// the coins on the NMI (coin_inserted: asserted on the press, cleared by e417)
static void coins(void)
{
	const uint8_t c = (uint8_t)(rbx_port_read(s_port_in[2]) & 0xc0);
	if ((s_m.coin_last & ~c) & 0xc0) {
		s_m.main_nmi = 1;
		m6502_set_nmi(&s_cpu, 1);
	}
	s_m.coin_last = c;
}

// ---------------------------------------------------------------- video
static uint8_t s_scr[256 * 256], s_pri[256 * 256];
static uint8_t s_objpix[2][64 * 64];

static inline int char_pix(const uint8_t *chr, int code, int x, int y)
{
	const int o = code * 8 + y, b = 7 - x;
	return ((chr[0x4000 + o] >> b) & 1) << 2 | ((chr[0x2000 + o] >> b) & 1) << 1 | ((chr[o] >> b) & 1);
}
static inline int sprite_pix(const uint8_t *chr, int code, int x, int y)
{
	const int o = code * 32 + (x < 8 ? 16 : 0) + y, b = 7 - (x & 7);
	return ((chr[0x4000 + o] >> b) & 1) << 2 | ((chr[0x2000 + o] >> b) & 1) << 1 | ((chr[o] >> b) & 1);
}
// a background tile pixel (tilelayout: the planes in the nibbles of 0x400 and 0, columns in four byte groups)
static inline int tile_pix(const uint8_t *tile, int code, int x, int y)
{
	const int o = code * 64 + (3 - (x >> 2)) * 16 + y, xb = x & 3;
	return ((tile[0x400 + o] >> (3 - xb)) & 1) << 2 | ((tile[0x400 + o] >> (7 - xb)) & 1) << 1 |
	       ((tile[o] >> (3 - xb)) & 1);
}

// the pixmap of the left (which 0) or right background tilemap (512x512, the global flip applied to it)
static inline int bg_pix(int which, int px, int py, int flip)
{
	if (flip) {
		px = 511 - px;
		py = 511 - py;
	}
	const int col = px >> 4, row = py >> 4;
	const int idx = s_tile_offset[col * 32 + row];
	const int base = (((s_snap->color_center_bot >> 7) & 1) * 4 + 1) * 8;
	if (which == 0 ? (idx & 0x80) : !(idx & 0x80))
		return base;
	const int tx = px & 15, ty = which ? 15 - (py & 15) : py & 15;   // (the right map's tiles: TILE_FLIPY)
	return base + tile_pix(s_snap->tile, s_snap->tile[idx] >> 4, tx, ty);
}

static void draw_edge(int which, int opaque, int flip)
{
	int scrolly_l = s_snap->back_vl_shift, scrolly_r = 256 - s_snap->back_vr_shift;
	if (!(s_snap->mode_set & 0x04))
		scrolly_r += 256;
	else
		scrolly_l += 256;
	const int scrollx = 256 - s_snap->back_h_shift, scrolly = which ? scrolly_r : scrolly_l;
	const int y0 = which ? 128 : VIS_Y0, y1 = which ? VIS_Y0 + VIS_H - 1 : 127;
	for (int y = y0; y <= y1; y++) {
		const int srcline = (y + scrolly) & 0x1ff;
		uint8_t *d = s_scr + y * 256;
		for (int x = 0; x < 256; x++) {
			int srccol = 0;
			switch (s_snap->mode_set & 3) {
			case 0: srccol = (x + scrollx) & 0xff; break;
			case 1: srccol = (x + scrollx + 0x100) & 0x1ff; break;
			case 2: srccol = ((x + scrollx) & 0xff) + 0x100; break;
			case 3: srccol = (x + scrollx) & 0x1ff; break;
			}
			const int pix = bg_pix(which, srccol, srcline, flip);
			if ((pix & 3) || opaque)
				d[x] = (uint8_t)pix;
		}
	}
}

static void draw_center(void)
{
	const uint8_t ccb = s_snap->color_center_bot, chs = s_snap->center_h_shift_space;
	int color = 0;
	if (ccb & 0x10) color |= 4;
	if (ccb & 0x20) color |= 2;
	if (ccb & 0x40) color |= 1;
	if (ccb & 0x80) color = (color & 4) + ((color << 1) & 2) + ((color >> 1) & 1);
	const int sy = s_snap->center_v_shift, sx = (chs >> 2) & 0x3c;
	for (int y = 0; y < 4; y++) {
		const int yy = sy + y;
		if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
			continue;
		if ((yy & ccb & 3) == (sy & ccb & 3))
			for (int x = 0; x < 256; x++)
				if ((x & 16) || (chs & 1))
					s_scr[yy * 256 + ((sx + x) & 255)] = (uint8_t)color;
	}
}

static void draw_sprite(int code, int color, int flipx, int flipy, int sx, int sy)
{
	for (int y = 0; y < 16; y++) {
		const int yy = sy + y;
		if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
			continue;
		for (int x = 0; x < 16; x++) {
			const int xx = sx + x;
			if (xx < 0 || xx > 255)
				continue;
			const int p = sprite_pix(s_snap->chr, code, flipx ? 15 - x : x, flipy ? 15 - y : y);
			if (!p)
				continue;
			uint8_t *pr = &s_pri[yy * 256 + xx];
			if (!((1u << (*pr & 0x1f)) & 2))
				s_scr[yy * 256 + xx] = (uint8_t)(color * 8 + p);
			*pr = 31;
		}
	}
}

static void dc_render(void)
{
	// the palette: 32 colours (RGB inverted, A4 inverted), pens 32-63 with bits 1 and 2 of the index swapped
	uint32_t c[32];
	for (int i = 0; i < 32; i++) {
		const uint8_t d = (uint8_t)~s_snap->pal[i];
		const int r = d & 7, g = (d >> 3) & 7, b = (d >> 6) & 3;
		c[i] = (uint32_t)(((r << 5) | (r << 2) | (r >> 1)) << 16 | ((g << 5) | (g << 2) | (g >> 1)) << 8 | (b * 0x55));
	}
	for (int i = 0; i < 32; i++) {
		rbx_video.palette[i] = c[i];
		rbx_video.palette[32 + i] = c[(i & ~6) | ((i >> 1) & 2) | ((i << 1) & 4)];
	}
	const int flip = s_snap->watchdog_flip & s_snap->flip_dsw;
	memset(s_scr, 8, sizeof(s_scr));
	memset(s_pri, 0, sizeof(s_pri));
	const uint8_t mode = s_snap->mode_set, ccb = s_snap->color_center_bot;
	if (mode & 0x08) {
		draw_edge(0, 1, flip);
		draw_edge(1, 1, flip);
	}
	draw_center();
	if (!(mode & 0x20) && (mode & 0x08)) {
		draw_edge(0, 0, flip);
		draw_edge(1, 0, flip);
	}
	// the foreground characters (pen 0 transparent)
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		const int ty = flip ? 255 - y : y;
		for (int x = 0; x < 256; x++) {
			const int tx = flip ? 255 - x : x;
			const int idx = (31 - (tx >> 3)) * 32 + (ty >> 3);
			const int code = 256 * (s_snap->col[idx] & 3) + s_snap->fg[idx];
			const int p = char_pix(s_snap->chr, code, tx & 7, ty & 7);
			if (p)
				s_scr[y * 256 + x] = (uint8_t)((ccb & 1) * 8 + p);
		}
	}
	// the sprites (in the foreground RAM, 0x20 apart)
	for (int i = 0, offs = 28 * 0x20; i < 8; i++, offs -= 4 * 0x20) {
		const uint8_t *r = s_snap->fg;
		if (!(r[offs] & 1))
			continue;
		int sx = 240 - r[offs + 3 * 0x20], sy = 240 - r[offs + 2 * 0x20];
		int flipx = r[offs] & 4, flipy = r[offs] & 2;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		const int code = r[offs + 0x20], color = (ccb >> 1) & 1;
		draw_sprite(code, color, flipx, flipy, sx, sy);
		draw_sprite(code, color, flipx, flipy, sx, sy + (flip ? -256 : 256));
	}
	// the missiles (in the colour RAM)
	for (int i = 0, offs = 0; i < 8; i++, offs += 4 * 0x20) {
		const uint8_t *r = s_snap->col;
		for (int k = 0; k < 2; k++) {
			int sy = 255 - r[offs + k * 0x20], sx = 255 - r[offs + (2 + k) * 0x20];
			if (flip) {
				sx = 240 - sx;
				sy = 240 - sy;
			}
			sy -= 1;
			if (sy < VIS_Y0 || sy >= VIS_Y0 + VIS_H)
				continue;
			for (int x = 0; x < 4; x++, sx++)
				if (sx >= 0 && sx <= 255) {
					s_scr[sy * 256 + sx] = (uint8_t)(((s_snap->color_missiles >> (4 * k)) & 7) | 8);
					s_pri[sy * 256 + sx] |= (uint8_t)(4 << k);
				}
		}
	}
	// the object (headlight / tunnel) and the pen bit 4 lighting
	const int crossing = mode & 3;
	if (!((crossing == 0 || (mode & 0x40)) && !(mode & 0x20))) {
		for (int o = 0; o < 2; o++)   // objlayout: 64x64 1bpp, rows 128 bits apart bottom up, columns right to left
			for (int y = 0; y < 64; y++)
				for (int x = 0; x < 64; x++) {
					const uint32_t bit = (uint32_t)(o * 64 + (63 - y) * 128 + (7 - (x >> 3)) * 8 + (x & 7));
					s_objpix[o][y * 64 + x] = (s_snap->obj[bit >> 3] >> (7 - (bit & 7))) & 1;
				}
		// (bitswap<8>(ccb, 0, 1, 7, 2, 3, 4, 5, 6) & 0x27) | 0x08
		const int color = ((ccb >> 7) & 1) << 5 | ((ccb >> 4) & 1) << 2 | ((ccb >> 5) & 1) << 1 | ((ccb >> 6) & 1) | 0x08;
		int sy = 64 - s_snap->part_v_shift + 1;
		if (sy < 0) sy += 256;
		const int sx = s_snap->part_h_shift - 128, scroll = s_snap->back_h_shift;
		for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
			const int dy = y - sy;
			for (int x = 0; x < 256; x++) {
				const int dx = x - sx;
				int pri2 = crossing == 0 ? 1 : crossing == 1 ? x >= scroll : crossing == 3 ? x < scroll : 0;
				if (mode & 0x80) {
					const uint8_t *o0 = s_objpix[0], *o1 = s_objpix[1];
					if ((dy >= -64 && dy < 0 && dx >= 64 && dx < 128 && o0[(-1 - dy) * 64 + dx - 64]) ||
					    (dy >= 0 && dy < 64 && dx >= 64 && dx < 128 && o0[dy * 64 + dx - 64]) ||
					    (dy >= -64 && dy < 0 && dx >= 0 && dx < 64 && o1[(-1 - dy) * 64 + dx]) ||
					    (dy >= 0 && dy < 64 && dx >= 0 && dx < 64 && o1[dy * 64 + dx])) {
						pri2 = 1;
						if ((mode & 0x20) && s_pri[y * 256 + x] == 0)
							s_scr[y * 256 + x] = (uint8_t)(color & 0x3f);
					}
				}
				if (!pri2)
					s_scr[y * 256 + x] |= 0x10;
			}
		}
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < 256; x++)
			bm[y * pitch + x] = s_scr[(y + VIS_Y0) * 256 + x];
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->chr, s_chr, sizeof(s_chr));
	memcpy(s_snap->fg, s_fg, sizeof(s_fg));
	memcpy(s_snap->col, s_col, sizeof(s_col));
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->obj, s_obj, sizeof(s_obj));
	memcpy(s_snap->pal, s_m.palc, sizeof(s_m.palc));
	s_snap->watchdog_flip = s_m.watchdog_flip;
	s_snap->color_missiles = s_m.color_missiles;
	s_snap->color_center_bot = s_m.color_center_bot;
	s_snap->mode_set = s_m.mode_set;
	s_snap->back_h_shift = s_m.back_h_shift;
	s_snap->back_vl_shift = s_m.back_vl_shift;
	s_snap->back_vr_shift = s_m.back_vr_shift;
	s_snap->part_h_shift = s_m.part_h_shift;
	s_snap->part_v_shift = s_m.part_v_shift;
	s_snap->center_h_shift_space = s_m.center_h_shift_space;
	s_snap->center_v_shift = s_m.center_v_shift;
	s_snap->flip_dsw = (rbx_port_read(s_port_dsw[0]) >> 6) & 1;
	rbx_video_frame_done();
}

// ---------------------------------------------------------------- init / frame
static const uint8_t t1_latch_26_pass_3_inv_2[8] = { T1PROM, T1PROM, T1LATCHINV, T1DIRECT, T1PROM, T1PROM, T1LATCH, T1PROM };
static const uint8_t t1_ctisland3[8] = { T1LATCHINV, T1PROM, T1PROM, T1DIRECT, T1PROM, T1PROM, T1LATCH, T1PROM };
static const uint8_t t1_pass_136[8] = { T1PROM, T1DIRECT, T1PROM, T1DIRECT, T1PROM, T1PROM, T1DIRECT, T1PROM };
static const uint8_t t1_nebula[8] = { T1PROM, T1PROM, T1DIRECT, T1DIRECT, T1PROM, T1DIRECT, T1PROM, T1PROM };
static const uint8_t t1_latch_xab_pass_x54[8] = { T1PROM, T1PROM, T1DIRECT, T1PROM, T1DIRECT, T1PROM, T1DIRECT, T1PROM };
static const uint8_t t1_latch_27_pass_3_inv_2[8] = { T1PROM, T1PROM, T1LATCHINV, T1DIRECT, T1PROM, T1PROM, T1PROM, T1LATCH };
static const uint8_t t1_latch_26_pass_5_inv_2[8] = { T1PROM, T1PROM, T1LATCHINV, T1PROM, T1PROM, T1DIRECT, T1LATCH, T1PROM };
static const uint8_t t1_latch_16_pass_3_inv_1[8] = { T1PROM, T1LATCHINV, T1PROM, T1DIRECT, T1PROM, T1PROM, T1LATCH, T1PROM };
static const uint8_t t1_map1100[8] = { T1PROM, T1PROM, T1LATCHINV, T1PROM, T1DIRECT, T1PROM, T1LATCH, T1PROM };
static const uint8_t t1_map1120[8] = { T1PROM, T1PROM, T1LATCHINV, T1DIRECT, T1PROM, T1LATCH, T1PROM, T1PROM };
static const uint8_t t1_clocknchj[8] = { T1PROM, T1PROM, T1DIRECT, T1LATCHINV, T1PROM, T1PROM, T1LATCH, T1PROM };

// the dongle of a set: its driver class (the set) and its MACHINE_RESET (the machine configuration)
static void select_dongle(const rbx_game *g)
{
	static const char *const type2[] = { "cdiscon1", "csweetht", "ctornado", "cmissnx", "cptennis", "cptennisj", NULL };
	static const char *const type4[] = { "cscrtry", "cscrtry2", "coozumou", NULL };
	static const struct { const char *m; const uint8_t *map; int a, b, c, d; } t1[] = {
		{ "ctsttape", t1_pass_136, 0, 1, 2, 3 }, { "cnebula", t1_nebula, 0, 1, 2, 3 },
		{ "chwy", t1_latch_27_pass_3_inv_2, 0, 1, 2, 3 }, { "cdsteljn", t1_latch_27_pass_3_inv_2, 0, 1, 2, 3 },
		{ "cterrani", t1_latch_26_pass_3_inv_2, 0, 1, 2, 3 }, { "castfant", t1_latch_16_pass_3_inv_1, 0, 1, 2, 3 },
		{ "csuperas", t1_latch_26_pass_3_inv_2, 0, 1, 2, 3 }, { "cmanhat", t1_latch_xab_pass_x54, 0, 1, 2, 3 },
		{ "clocknch", t1_latch_26_pass_3_inv_2, 0, 1, 2, 3 }, { "cprogolf", t1_latch_26_pass_3_inv_2, 0, 1, 2, 3 },
		{ "cprogolfj", t1_latch_26_pass_3_inv_2, 0, 1, 2, 3 }, { "cluckypo", t1_latch_26_pass_3_inv_2, 0, 1, 2, 3 },
		{ "ctisland", t1_latch_26_pass_3_inv_2, 0, 1, 2, 3 }, { "ctisland3", t1_ctisland3, 0, 1, 2, 3 },
		{ "cexplore", t1_latch_26_pass_5_inv_2, 0, 1, 2, 3 }, { "cocean1a", t1_map1100, 0, 1, 2, 3 },
		{ "cfboy0a1", t1_map1120, 0, 1, 2, 3 }, { "clocknchj", t1_clocknchj, 0, 1, 2, 3 },
		{ NULL, NULL, 0, 0, 0, 0 } };
	static const struct { const char *m; int swap; } t3[] = {
		{ "cfishing", SWAP_01 }, { "cbtime", SWAP_12 }, { "cburnrub", SWAP_67 }, { "cgraplop", SWAP_56 },
		{ "cgraplop2", SWAP_67 }, { "clapapa", SWAP_34_7 }, { "cskater", SWAP_45 }, { "cprobowl", SWAP_34_0 },
		{ "cnightst", SWAP_13 }, { "cpsoccer", SWAP_24 }, { "csdtenis", SWAP_23_56 }, { "czeroize", SWAP_23_56 },
		{ "cppicf", SWAP_01 }, { "cfghtice", SWAP_25 }, { NULL, 0 } };
	s_dongle_type = D_NONE;
	s_t1map = NULL;
	s_t1in = s_t1out = make_map(0, 1, 2, 3, 4, 5, 6, 7);
	s_t3swap = SWAP_NONE;
	for (int i = 0; type2[i]; i++)
		if (!strcmp(g->set, type2[i])) s_dongle_type = D_TYPE2;
	for (int i = 0; type4[i]; i++)
		if (!strcmp(g->set, type4[i])) s_dongle_type = D_TYPE4;
	if (!strcmp(g->set, "cbdash")) s_dongle_type = D_TYPE5;
	if (!strcmp(g->set, "cflyball")) s_dongle_type = D_NODONG;
	if (!strcmp(g->set, "decomult")) s_dongle_type = D_WIDEL;
	if (!strncmp(g->set, "decodark", 8)) s_dongle_type = D_DARKSOFT;
	if (s_dongle_type != D_NONE)
		return;
	for (int i = 0; t3[i].m; i++)
		if (!strcmp(g->machine, t3[i].m)) {
			s_dongle_type = D_TYPE3;
			s_t3swap = t3[i].swap;
			return;
		}
	for (int i = 0; t1[i].m; i++)
		if (!strcmp(g->machine, t1[i].m)) {
			s_dongle_type = D_TYPE1;
			s_t1map = t1[i].map;
			break;
		}
	// (the flipped address bits of the DE-0061 variants)
	const char *m = g->machine;
	if (!strcmp(m, "csuperas")) s_t1in = s_t1out = make_map(0, 1, 2, 3, 5, 4, 6, 7);
	else if (!strcmp(m, "clocknch")) s_t1in = s_t1out = make_map(0, 1, 3, 2, 4, 5, 6, 7);
	else if (!strcmp(m, "cprogolf") || !strcmp(m, "cprogolfj")) s_t1in = s_t1out = make_map(1, 0, 2, 3, 4, 5, 6, 7);
	else if (!strcmp(m, "cluckypo")) s_t1in = s_t1out = make_map(0, 3, 2, 1, 4, 5, 6, 7);
	else if (!strcmp(m, "ctisland")) s_t1in = s_t1out = make_map(2, 1, 0, 3, 4, 5, 6, 7);
}

static void dc_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_chr, 0, sizeof(s_chr));
	memset(s_fg, 0, sizeof(s_fg));
	memset(s_col, 0, sizeof(s_col));
	memset(s_tile, 0, sizeof(s_tile));
	memset(s_obj, 0, sizeof(s_obj));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_sram, 0, sizeof(s_sram));
	memset(&s_m, 0, sizeof(s_m));
	s_m.firsttime = 1;
	s_m.i8041_p1 = s_m.i8041_p2 = 0xff;
	s_m.coin_last = 0xc0;
	tape_region_update();
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	m6502_reset(&s_cpu);
	m6502_set_irq(&s_cpu, 0);
	m6502_set_nmi(&s_cpu, 0);
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	mcs48_reset(&s_mcu);
	s_line = 0;
	s_frame = 0;
	s_cpu_target = s_snd_target = 0;
	s_cpu_frac = s_snd_frac = s_mcu_frac = s_ay_frac = 0;
	s_mcu_avail = 0;
}

static void dc_exit(void);

static int dc_init(const rbx_game *g)
{
	uint32_t ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	s_mrom = rbx_region("mcu", &s_mrom_size);
	s_dongle = rbx_region("dongle", &s_dongle_size);
	s_tape = rbx_region("cassette", &s_tape_size);
	s_user3 = rbx_region("user3", &s_user3_size);
	const uint8_t *prom = rbx_region("proms", &ps);
	(void)prom;
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || s_srom_size < 0x10000 || !s_mrom || s_mrom_size < 0x400) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!s_dongle)
		s_dongle_size = 0;
	if (!s_tape)
		s_tape_size = 0;
	select_dongle(g);
	s_crom = !strcmp(g->set, "ctisland") || !strcmp(g->set, "ctisland2") || !strcmp(g->set, "ctisland3") ||
	         !strcmp(g->set, "cexplore");
	if (s_crom && !s_user3)
		s_user3_size = 0;
	s_telejan = !strcmp(g->set, "cdsteljn");
	if (!strcmp(g->machine, "czeroize") && s_dongle && s_dongle_size >= 0x1000) {   // (MAME's hack: the missing ROM)
		memset(s_dongle, 0, 0x1000);
		s_dongle[0x08a0] = 0x18;
		s_dongle[0x08a1] = 0xf7;
	}
	if (s_tape)
		tape_init();
	s_snap = calloc(1, sizeof(*s_snap));
	// AY-3-8910 x2 at 1.5 MHz, 0.40 each
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, MASTER / 8, AY8910_LEGACY_OUTPUT, 102, 4, 0);
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_snap || !s_mix || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(256, VIS_H, 64)) {
		rbx_fail("%s: out of memory", g->set);
		dc_exit();
		return -1;
	}
	m6502_init(&s_cpu, M6502_NMOS);
	s_cpu.deco222 = 1;
	s_cpu.read = main_r;
	s_cpu.write = main_w;
	for (int p = 0xf0; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	for (int p = 0; p < 0x60; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	m6502_init(&s_snd, M6502_NMOS);
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	for (int p = 0xf8; p < 0x100; p++)
		s_snd.rpage[p] = s_srom + p * 256;
	for (int p = 0; p < 0x10; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	mcs48_init(&s_mcu, MCS48_I8041A);
	s_mcu.rom = s_mrom;
	s_mcu.rom_mask = 0x3ff;
	s_mcu.port_r = mcu_port_r;
	s_mcu.port_w = mcu_port_w;
	static const char *const in[3] = { "IN0", "IN1", "IN2" }, *const an[4] = { "AN0", "AN1", "AN2", "AN3" };
	for (int i = 0; i < 3; i++)
		s_port_in[i] = rbx_port_find(in[i]);
	for (int i = 0; i < 4; i++) {
		char t[8] = "P1_MP0";
		s_port_an[i] = rbx_port_find(an[i]);
		t[1] = '1';
		t[5] = (char)('0' + i);
		s_port_mp[0][i] = rbx_port_find(t);
		t[1] = '2';
		s_port_mp[1][i] = rbx_port_find(t);
	}
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	dc_reset();
	return 0;
}

static void dc_exit(void)
{
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
}

static void dc_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coins();
	for (int i = 0; i < LINES; i++) {   // (from line 248, the vblank start)
		s_line = i;
		const int y = (i + VIS_Y0 + VIS_H) % LINES;
		if (!(y & 7) && (s_frame || i >= LINES - VIS_H)) {   // the audio NMI timer: every 8 lines from line 0, its
		                                                       // state bit 3 of the line
			s_m.audio_nmi_state = y & 8;
			audio_nmi_update();
		}
		s_cpu_target += rbx_line_cycles(&s_cpu_frac, MASTER / 16, FRAME_MHZ, LINES);
		m6502_run_until(&s_cpu, s_cpu_target);
		s_snd_target += rbx_line_cycles(&s_snd_frac, MASTER / 24, FRAME_MHZ, LINES);
		if (s_m.snd_reset)
			s_snd.cycles = s_snd_target;
		else
			m6502_run_until(&s_snd, s_snd_target);
		const int mc = rbx_line_cycles(&s_mcu_frac, MCU_RATE, FRAME_MHZ, LINES);
		const uint64_t line_end = s_m.mcu_time + (uint64_t)mc;
		if (s_m.mcu_reset) {
			s_mcu_avail = 0;
		} else {
			s_mcu_avail += mc;
			uint64_t t = s_m.mcu_time;
			while (s_mcu_avail > 0) {
				s_exec_base = t;
				s_exec_req = s_mcu_avail;
				const int ran = mcs48_execute(&s_mcu, s_mcu_avail);
				s_mcu_avail -= ran;
				t += (uint64_t)ran;
			}
		}
		tape_advance(line_end);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ay_frac, MASTER / 8, FRAME_MHZ, LINES);
		const int n0 = ay8910_render(&s_ay[0], clocks, s_mix, s_mix_cap);
		const int n1 = ay8910_render(&s_ay[1], clocks, s_mix, s_mix_cap);
		const int n = n0 > n1 ? n0 : n1;
		for (int k = 0; k < n; k++) {
			rbx_audio_put(s_mix[k]);
			s_mix[k] = 0;
		}
	}
	vblank_capture();
	s_frame++;
}

static int dc_state(void *ctx, int load)
{
	uint8_t cpu[M6502_STATE_SIZE], snd[M6502_STATE_SIZE], ay[2][256];
	uint8_t mcu[MCS48_STATE_SIZE];
	const uint32_t ays = ay8910_state_size();
	if (!load) {
		memcpy(cpu, &s_cpu, M6502_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
		memcpy(mcu, (uint8_t *)&s_mcu + MCS48_STATE_OFFSET, MCS48_STATE_SIZE);
		for (int i = 0; i < 2; i++)
			memcpy(ay[i], &s_ay[i], ays);
	}
	int64_t tg[2] = { s_cpu_target, s_snd_target };
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_mcu_frac, s_ay_frac };
	int32_t misc[4] = { (int32_t)s_frame, s_mcu_avail, 0, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "snd", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "mcu", mcu, MCS48_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ay", ay, sizeof(ay));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "chr", s_chr, sizeof(s_chr));
	rc |= rbx_state_io(ctx, "fg", s_fg, sizeof(s_fg));
	rc |= rbx_state_io(ctx, "col", s_col, sizeof(s_col));
	rc |= rbx_state_io(ctx, "tile", s_tile, sizeof(s_tile));
	rc |= rbx_state_io(ctx, "obj", s_obj, sizeof(s_obj));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "m", &s_m, sizeof(s_m));
	rc |= rbx_state_io(ctx, "tg", tg, sizeof(tg));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6502_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		memcpy((uint8_t *)&s_mcu + MCS48_STATE_OFFSET, mcu, MCS48_STATE_SIZE);
		for (int i = 0; i < 2; i++)
			memcpy(&s_ay[i], ay[i], ays);
		s_cpu_target = tg[0];
		s_snd_target = tg[1];
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_mcu_frac = fr[2];
		s_ay_frac = fr[3];
		s_frame = (uint32_t)misc[0];
		s_mcu_avail = misc[1];
	}
	return rc ? -1 : 0;
}

static int dc_save(void *ctx) { return dc_state(ctx, 0); }
static int dc_load(void *ctx) { return dc_state(ctx, 1); }
static int dc_supported(const char *machine) { (void)machine; return 1; }

const rbx_driver rbx_drv_decocass = { "DECO Cassette System", dc_init, dc_exit, dc_reset, dc_frame, dc_save, dc_load,
                                      dc_supported, dc_render, NULL };
