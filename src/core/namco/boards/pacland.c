/*  Namco Pac-Land: an MC6809E, an HD63701 (sound and inputs), two 64 x 32 tilemaps (the foreground with a per-row
    scroll and two priority categories), the Mappy-type sprites with the CUS29 priority / transparency through the
    lookup PROM, a bank-switched palette and the CUS30.
    Ported from MAME 0.289 src/mame/namco/pacland.cpp (BSD-3-Clause, copyright-holders: Ernesto Corvi).
    This file is part of ReBOX. GPL-2.0-or-later.

    Not done: the LEDs, the coin counters / lockout, the watchdog.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6800/m6801.h"
#include "../../sound/namco_wsg.h"

#include <stdlib.h>
#include <string.h>

#define VIS_W 288
#define VIS_X0 24               // (the visible area: 24-311 x 16-239 of 384 x 264)
#define VIS_Y0 16
#define VIS_H 224
#define FRAME_MHZ 60606         // 6.144 MHz / (384 x 264)
#define FRAME_CYCLES 25344      // (the CPUs' clocks a frame: 96 a line)
#define CPU_CLOCK 1536000       // 49.152 MHz / 32 (and the MCU's E clock: / 8 / 4)
#define SND_RATE 48000
#define NPENS 0xc00             // fg chars 256 x 4, bg chars 256 x 4, sprites 64 x 16

// ---- the set's configuration ----
static uint8_t *s_cpu1, *s_mcurom, *s_mcusub, *s_fgrom, *s_bgrom, *s_sprrom, *s_proms;
static uint32_t s_cpu1_size, s_mcurom_size, s_mcusub_size, s_fgrom_size, s_bgrom_size, s_sprrom_size, s_proms_size;
static uint8_t *s_fgpix, *s_bgpix, *s_sprpix;   // decoded, a byte a pixel
static uint32_t s_fg_count, s_bg_count, s_spr_count;
static uint32_t (*s_pal_hp)[4][NPENS];   // (the four palette banks)
#define s_pal (*s_pal_hp)
static uint8_t s_fg_trans[256];    // a fg colour's transparent pens (lookup 0x7f / 0xff)
static uint16_t s_spr_mask[3][64]; // m_transmask[0..2]
static int s_port_in[5];           // DSWA DSWB IN0 IN1 IN2

static m6809_t s_cpu;
static m6801 s_mcu;
static namco_wsg_t s_wsg;

// ---- the board's state (saved) ----
static struct {
	uint8_t flip, rombank, palbank, main_irq_mask, mcu_irq_mask, mcu_reset, pad[2];
	uint16_t scroll[2];
	int64_t t;          // (the scheduler's base time)
	int64_t mcu_req;    // (the MCU's clocks asked for; its time is mcu_req - its icount)
	uint64_t snd_frac;
	uint8_t vram[2][0x1000];
	uint8_t spr[0x1800];
	uint8_t mcuram[0x800];
} b;

// ---- the drawing's copy ----
typedef struct {
	uint8_t vram[2][0x1000];
	uint8_t spr[0x1800];
	uint16_t scroll[2];
	uint8_t flip, palbank;
} snap_t;
static snap_t *s_snap;
static uint8_t *s_pri;     // (the screen priority bitmap: 31 under a high priority sprite pixel)
static uint16_t *s_sbm;    // (m_sprite_bitmap)
static int32_t s_mix[16];  // (a timeslice's samples)

static int s_sync;           // (the main CPU asked for a sync: its timeslice ends after this instruction)
static int32_t s_mcu_stolen; // (the MCU asked for one: the clocks taken from its timeslice)
static int s_pend_reset;     // (the MCU's RESET line change, applied at the sync point: -1 none)

// ---- the main CPU (main_map) ----
static void rom_bank(void)
{
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x40 + p] = s_cpu1 + 0x10000 + (uint32_t)b.rombank * 0x2000 + (uint32_t)p * 256;
}

static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x6800 && a < 0x6c00) return namco_cus30_amap_r(&s_wsg, a & 0x3ff);
	return 0;
}

static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x3800 || a == 0x3801)
		b.scroll[0] = (uint16_t)(v + 256 * (a & 1));
	else if (a == 0x3a00 || a == 0x3a01)
		b.scroll[1] = (uint16_t)(v + 256 * (a & 1));
	else if (a == 0x3c00) {   // bankswitch_w
		b.rombank = v & 7;
		b.palbank = (v & 0x18) >> 3;
		rom_bank();
	} else if (a >= 0x6800 && a < 0x6c00)
		namco_cus30_amap_w(&s_wsg, a & 0x3ff, v);
	else if (a >= 0x7000 && a < 0x8000) {   // maincpu_irq_ctrl_w
		b.main_irq_mask = !(a & 0x800);
		if (!b.main_irq_mask) {   // (a synced input line change: the timeslice ends here)
			m6809_set_irq(&s_cpu, 0);
			s_sync = 1;
		}
	} else if (a >= 0x8000 && a < 0x9000) {   // subreset_w (synced: applied when the MCU has caught up)
		s_pend_reset = (a & 0x800) != 0;
		s_sync = 1;
	} else if (a >= 0x9000 && a < 0xa000)   // flipscreen_w
		b.flip = !(a & 0x800);
}

// ---- the MCU (mcu_map) ----
static uint8_t mcu_r(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x1000 && a < 0x1400) return namco_cus30_amap_r(&s_wsg, a & 0x3ff);
	if (a >= 0x8000 && a < 0xa000) return s_mcusub[rbx_wrap(a - 0x8000, s_mcusub_size)];
	if (a >= 0xc000 && a < 0xc800) return b.mcuram[a - 0xc000];
	if (a >= 0xd000 && a < 0xd004) {   // input_r
		const int shift = 4 * (a & 1), port = a & 2;
		return (uint8_t)(((rbx_port_read(s_port_in[port]) << shift) & 0xf0) |
		                 ((rbx_port_read(s_port_in[port + 1]) >> (4 - shift)) & 0x0f));
	}
	if (a >= 0xf000) return s_mcurom[a & 0xfff];
	return 0;
}

static void mcu_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x1000 && a < 0x1400)
		namco_cus30_amap_w(&s_wsg, a & 0x3ff, v);
	else if (a >= 0x4000 && a < 0x8000) {   // mcu_irq_ctrl_w
		b.mcu_irq_mask = !(a & 0x2000);
		if (!b.mcu_irq_mask) {   // (synced: the MCU's timeslice ends after this instruction, as abort_timeslice)
			m6801_set_irq_line(&s_mcu, M6801_IRQ1_LINE, 0);
			s_mcu_stolen += s_mcu.icount;
			s_mcu.icount = 0;
		}
	} else if (a >= 0xc000 && a < 0xc800)
		b.mcuram[a - 0xc000] = v;
}

static uint8_t mcu_port_r(void *u, int port)
{
	(void)u;
	return port == 1 ? (uint8_t)rbx_port_read(s_port_in[4]) : 0xff;
}

// ---- drawing ----
// a tilemap's pixel at the logical position (x 0-511, y 0-255): the pen index, cat: the category
static inline int tile_pixel(const uint8_t *vram, const uint8_t *pix, uint32_t count, int fg, int x, int y, int *cat, int *opaque)
{
	const int offs = ((y >> 3) * 64 + (x >> 3)) * 2;
	const int attr = vram[offs + 1];
	const int code = vram[offs] + ((attr & 1) << 8);
	const int color = fg ? ((attr & 0x1e) >> 1) + ((code & 0x1e0) >> 1) : ((attr & 0x3e) >> 1) + ((code & 0x1c0) >> 1);
	const int px = (attr & 0x40) ? 7 - (x & 7) : x & 7, py = (attr & 0x80) ? 7 - (y & 7) : y & 7;
	const int pen = pix[rbx_wrap((uint32_t)code, count) * 64 + (uint32_t)(py * 8 + px)];
	if (fg) {
		*cat = (attr & 0x20) ? 1 : 0;
		*opaque = !((s_fg_trans[color] >> pen) & 1);
	}
	return color * 4 + pen;
}

// a tile row of a tilemap line, looked up once a tile column (tile_pixel's result for every x of the column)
typedef struct {
	int col;               // (the tile column cached, -1 none)
	const uint8_t *row;    // the tile's row of pixels
	int flipx, color, cat;
	uint16_t trans;        // (fg: the colour's transparent pens)
} tile_cache;

static inline void tile_fetch(tile_cache *tc, const uint8_t *vram, const uint8_t *pix, uint32_t count, int fg, int col, int y)
{
	const int offs = ((y >> 3) * 64 + col) * 2;
	const int attr = vram[offs + 1];
	const int code = vram[offs] + ((attr & 1) << 8);
	tc->col = col;
	tc->color = fg ? ((attr & 0x1e) >> 1) + ((code & 0x1e0) >> 1) : ((attr & 0x3e) >> 1) + ((code & 0x1c0) >> 1);
	tc->flipx = (attr & 0x40) != 0;
	const int py = (attr & 0x80) ? 7 - (y & 7) : y & 7;
	tc->row = pix + rbx_wrap((uint32_t)code, count) * 64 + (uint32_t)(py * 8);
	tc->cat = (attr & 0x20) ? 1 : 0;
	tc->trans = fg ? s_fg_trans[tc->color] : 0;
}

static void draw_bg(const snap_t *sn, int vy, uint16_t *d)
{
	const int flip = sn->flip, sy = vy + VIS_Y0;
	const int ly = flip ? 255 - ((sy - 8) & 0xff) : sy & 0xff;
	tile_cache tc;
	tc.col = -1;
	for (int vx = 0; vx < VIS_W; vx++) {
		const int sx = vx + VIS_X0;
		const int lx = flip ? 511 - ((sx + 468 - sn->scroll[1]) & 0x1ff) : (sx - 3 + sn->scroll[1]) & 0x1ff;
		if ((lx >> 3) != tc.col)
			tile_fetch(&tc, sn->vram[1], s_bgpix, s_bg_count, 0, lx >> 3, ly);
		const int pen = tc.row[tc.flipx ? 7 - (lx & 7) : lx & 7];
		d[vx] = (uint16_t)(0x400 + tc.color * 4 + pen);
	}
}

// draw_fg: the category's opaque pixels where no high priority sprite pixel is
static void draw_fg(const snap_t *sn, int vy, uint16_t *d, const uint8_t *pri, int category)
{
	const int flip = sn->flip, sy = vy + VIS_Y0;
	const int ly = flip ? 255 - ((sy - 8) & 0xff) : sy & 0xff;
	const int row = ly >> 3;
	const int rs = (row >= 5 && row < 29) ? sn->scroll[0] - (flip ? 7 : 0) : 0;
	tile_cache tc;
	tc.col = -1;
	for (int vx = 0; vx < VIS_W; vx++) {
		const int sx = vx + VIS_X0;
		const int lx = flip ? 511 - ((sx + 464 - rs) & 0x1ff) : (sx + rs) & 0x1ff;
		if ((lx >> 3) != tc.col)
			tile_fetch(&tc, sn->vram[0], s_fgpix, s_fg_count, 1, lx >> 3, ly);
		if (tc.cat != category || pri[vx])
			continue;
		const int pen = tc.row[tc.flipx ? 7 - (lx & 7) : lx & 7];
		if (!((tc.trans >> pen) & 1))
			d[vx] = (uint16_t)(tc.color * 4 + pen);
	}
}

// whichmask 0: only the high priority pixels, marking the priority bitmap; 1, 2: drawn into dst
static void draw_sprites(const snap_t *sn, uint16_t *dst, int pitch, int whichmask)
{
	static const int gfx_offs[2][2] = { { 0, 1 }, { 2, 3 } };
	const uint8_t *s1 = sn->spr + 0x780, *s2 = sn->spr + 0xf80, *s3 = sn->spr + 0x1780;
	for (int offs = 0; offs < 0x80; offs += 2) {
		int sprite = s1[offs] + ((s3[offs] & 0x80) << 1);
		const int color = s1[offs + 1] & 0x3f;
		const int sx = s2[offs + 1] + 0x100 * (s3[offs + 1] & 1) - 47;
		int sy = 256 - s2[offs] + 9;
		int flipx = s3[offs] & 1, flipy = (s3[offs] & 2) >> 1;
		const int sizex = (s3[offs] & 4) >> 2, sizey = (s3[offs] & 8) >> 3;
		sprite &= ~sizex;
		sprite &= ~(sizey << 1);
		if (sn->flip) {
			flipx ^= 1;
			flipy ^= 1;
		}
		sy -= 16 * (sizey + 1);
		sy = (sy & 0xff) - 16;
		const uint16_t trans = s_spr_mask[whichmask][color];
		const uint16_t cbase = (uint16_t)(0x800 + color * 16);
		for (int yy = 0; yy <= sizey; yy++)
			for (int xx = 0; xx <= sizex; xx++) {
				const uint32_t code = rbx_wrap((uint32_t)(sprite + gfx_offs[yy ^ (sizey * flipy)][xx ^ (sizex * flipx)]), s_spr_count);
				const uint8_t *tile = s_sprpix + code * 256;
				const int x0 = sx + 16 * xx - VIS_X0, y0 = sy + 16 * yy - VIS_Y0;
				for (int py = 0; py < 16; py++) {
					const int ay = y0 + py;
					if (ay < 0 || ay >= VIS_H)
						continue;
					const uint8_t *src = tile + (flipy ? 15 - py : py) * 16;
					uint16_t *d = dst + ay * pitch;
					uint8_t *pri = s_pri + ay * VIS_W;
					for (int px = 0; px < 16; px++) {
						const int ax = x0 + px;
						if (ax < 0 || ax >= VIS_W)
							continue;
						const uint8_t pen = src[flipx ? 15 - px : px];
						if ((trans >> pen) & 1)
							continue;
						if (whichmask == 0)
							pri[ax] = 31;
						else
							d[ax] = (uint16_t)(cbase + pen);
					}
				}
			}
	}
}

static void pl_render(void)
{
	const snap_t *sn = s_snap;
	memcpy(rbx_video.palette, s_pal[sn->palbank & 3], sizeof(s_pal[0]));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	memset(s_pri, 0, VIS_W * VIS_H);
	draw_sprites(sn, NULL, 0, 0);
	for (int y = 0; y < VIS_H; y++) {
		draw_bg(sn, y, bm + y * pitch);
		draw_fg(sn, y, bm + y * pitch, s_pri + y * VIS_W, 0);
	}
	draw_sprites(sn, bm, pitch, 1);
	memset(s_sbm, 0, VIS_W * VIS_H * sizeof(uint16_t));
	draw_sprites(sn, s_sbm, VIS_W, 2);
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *spr = s_sbm + y * VIS_W;
		const uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++)
			if (spr[x] != 0 && spr[x] != d[x])
				spr[x] = 0;
	}
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		const uint16_t *spr = s_sbm + y * VIS_W;
		draw_fg(sn, y, d, s_pri + y * VIS_W, 1);
		for (int x = 0; x < VIS_W; x++)
			if (spr[x])
				d[x] = spr[x];
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, b.vram, sizeof(b.vram));
	memcpy(s_snap->spr, b.spr, sizeof(b.spr));
	memcpy(s_snap->scroll, b.scroll, sizeof(b.scroll));
	s_snap->flip = b.flip;
	s_snap->palbank = b.palbank;
	rbx_video_frame_done();
}

// ---- frame ----
// MAME's scheduling: timeslices of the 6 kHz quantum (256 clocks) from the last sync point, cut by the screen's timers
// (line 0, the vblank's end at line 16, its start at line 240); a synced input line change ends the running CPU's
// timeslice where it happens, and the later CPUs then run only up to that time. The main CPU runs first, then the
// MCU. A frame here runs from the vblank's start to the next (MAME starts the machine there, as for Baraduke).
#define QUANTUM 256
static const int s_timer_at[3] = { 24 * 96, 40 * 96, FRAME_CYCLES };   // (from the vblank's start)

static void run_mcu(int64_t target, int64_t *mcu_time)
{
	const int n = (int)(target - b.mcu_req);
	b.mcu_req = target;
	s_mcu_stolen = 0;
	m6801_execute(&s_mcu, n);
	b.mcu_req -= s_mcu_stolen;
	*mcu_time = b.mcu_req - s_mcu.icount;
}

static void pl_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	const int64_t start = b.t, end = b.t + FRAME_CYCLES;
	while (b.t < end) {
		int64_t target = b.t + QUANTUM;
		for (int i = 0; i < 3; i++)
			if (start + s_timer_at[i] > b.t && start + s_timer_at[i] < target)
				target = start + s_timer_at[i];
		s_sync = 0;
		s_pend_reset = -1;
		while (s_cpu.cycles < target) {
			m6809_step(&s_cpu);
			if (s_sync) {
				if (s_cpu.cycles < target)
					target = s_cpu.cycles;
				break;
			}
		}
		int64_t mcu_time = b.mcu_req - s_mcu.icount;
		if (mcu_time < target) {
			run_mcu(target, &mcu_time);
			if (mcu_time < target)
				target = mcu_time;
		}
		if (s_pend_reset >= 0 && s_pend_reset != b.mcu_reset) {
			b.mcu_reset = (uint8_t)s_pend_reset;
			m6801_set_reset(&s_mcu, s_pend_reset);
		}
		b.snd_frac += (uint64_t)(target - b.t);
		b.t = target;
		const int k = (int)(b.snd_frac / (CPU_CLOCK / SND_RATE));   // (48 kHz: a sample every 32 clocks)
		if (k > 0) {
			b.snd_frac -= (uint64_t)k * (CPU_CLOCK / SND_RATE);
			memset(s_mix, 0, (size_t)k * sizeof(s_mix[0]));
			namco_wsg_render(&s_wsg, s_mix, NULL, k, SND_RATE);
			for (int i = 0; i < k; i++)
				rbx_audio_put(s_mix[i]);
		}
	}
	// vblank: the frame drawn, then the IRQs
	vblank_capture();
	if (b.main_irq_mask)
		m6809_set_irq(&s_cpu, 1);
	if (b.mcu_irq_mask)
		m6801_set_irq_line(&s_mcu, M6801_IRQ1_LINE, 1);
}

// ---- init / reset ----
static void pl_reset(void)
{
	b.flip = b.rombank = b.palbank = 0;
	b.main_irq_mask = b.mcu_irq_mask = b.mcu_reset = 0;
	b.scroll[0] = b.scroll[1] = 0;
	b.t = 0;
	b.mcu_req = 0;
	b.snd_frac = 0;
	s_sync = 0;
	s_mcu_stolen = 0;
	s_pend_reset = -1;
	rom_bank();
	m6809_reset(&s_cpu);
	s_cpu.cycles = 4;   // (MAME: the reset vector fetch, INTERRUPT_VECTOR, takes 4 clocks)
	m6801_set_reset(&s_mcu, 0);
	m6801_reset(&s_mcu);
	s_mcu.icount = 0;
	namco_wsg_init(&s_wsg, NAMCO_WSG_TYPE_CUS30, 49152000 / 2048, 0, NULL, 256);
}

static void pl_exit(void);

static void map_pages(m6809_t *c, int first, int n, uint8_t *mem)
{
	for (int p = 0; p < n; p++)
		c->rpage[first + p] = c->wpage[first + p] = mem + p * 256;
}

// charlayout: 8x8, 2bpp, planes at bits 0 / 4 (MSB first), pixels 0-3 from byte 8, 4-7 from byte 0
static void decode_chars(const uint8_t *rom, uint32_t count, uint8_t *pix)
{
	for (uint32_t t = 0; t < count; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint8_t v = rom[t * 16 + (uint32_t)((x < 4 ? 8 : 0) + y)];
				const int k = x & 3;
				pix[t * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)((((v >> (7 - k)) & 1) << 1) | ((v >> (3 - k)) & 1));
			}
}

static int pl_init(const rbx_game *g)
{
	if (!s_pal_hp && !(s_pal_hp = calloc(1, sizeof(*s_pal_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_cpu1 = rbx_region("maincpu", &s_cpu1_size);
	s_mcurom = rbx_region("mcu", &s_mcurom_size);
	s_mcusub = rbx_region("mcusub", &s_mcusub_size);
	s_fgrom = rbx_region("fg_chars", &s_fgrom_size);
	s_bgrom = rbx_region("bg_chars", &s_bgrom_size);
	s_sprrom = rbx_region("sprites", &s_sprrom_size);
	s_proms = rbx_region("proms", &s_proms_size);
	if (!s_cpu1 || s_cpu1_size < 0x20000 || !s_mcurom || s_mcurom_size < 0x1000 || !s_mcusub || !s_mcusub_size || !s_fgrom ||
	    s_fgrom_size < 16 || !s_bgrom || s_bgrom_size < 16 || !s_sprrom || s_sprrom_size < 128 || !s_proms || s_proms_size < 0x1400) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// the palette: four banks of 256 colours (resistor weights 0x0e 0x1f 0x43 0x8f), three 1024-entry lookup PROMs
	for (int bank = 0; bank < 4; bank++) {
		uint32_t rgb[256];
		for (int i = 0; i < 256; i++) {
			const uint8_t rg = s_proms[bank * 256 + i], bl = s_proms[0x400 + bank * 256 + i];
			const int r = 0x0e * (rg & 1) + 0x1f * ((rg >> 1) & 1) + 0x43 * ((rg >> 2) & 1) + 0x8f * ((rg >> 3) & 1);
			const int gr = 0x0e * ((rg >> 4) & 1) + 0x1f * ((rg >> 5) & 1) + 0x43 * ((rg >> 6) & 1) + 0x8f * ((rg >> 7) & 1);
			const int bb = 0x0e * (bl & 1) + 0x1f * ((bl >> 1) & 1) + 0x43 * ((bl >> 2) & 1) + 0x8f * ((bl >> 3) & 1);
			rgb[i] = (uint32_t)r << 16 | (uint32_t)gr << 8 | (uint32_t)bb;
		}
		for (int i = 0; i < NPENS; i++)
			s_pal[bank][i] = rgb[s_proms[0x800 + i]];
	}
	for (int c = 0; c < 256; c++) {
		s_fg_trans[c] = 0;
		for (int p = 0; p < 4; p++) {
			const uint8_t e = s_proms[0x800 + c * 4 + p];
			if (e == 0x7f || e == 0xff)
				s_fg_trans[c] |= (uint8_t)(1 << p);
		}
	}
	for (int c = 0; c < 64; c++) {
		s_spr_mask[0][c] = s_spr_mask[1][c] = s_spr_mask[2][c] = 0;
		for (int p = 0; p < 16; p++) {
			const uint8_t e = s_proms[0x1000 + c * 16 + p];
			if (e >= 0x80) s_spr_mask[0][c] |= (uint16_t)(1 << p);
			if ((e & 0x7f) == 0x7f) s_spr_mask[1][c] |= (uint16_t)(1 << p);
			if (e < 0xf0 || e == 0xff) s_spr_mask[2][c] |= (uint16_t)(1 << p);
		}
	}
	s_fg_count = s_fgrom_size / 16;
	s_bg_count = s_bgrom_size / 16;
	s_spr_count = s_sprrom_size / 2 / 64;
	s_fgpix = (uint8_t *)malloc((size_t)s_fg_count * 64);
	s_bgpix = (uint8_t *)malloc((size_t)s_bg_count * 64);
	s_sprpix = (uint8_t *)malloc((size_t)s_spr_count * 256);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	s_sbm = (uint16_t *)calloc(1, VIS_W * VIS_H * sizeof(uint16_t));
	if (!s_fgpix || !s_bgpix || !s_sprpix || !s_snap || !s_pri || !s_sbm) {
		pl_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	decode_chars(s_fgrom, s_fg_count, s_fgpix);
	decode_chars(s_bgrom, s_bg_count, s_bgpix);
	{   // spritelayout: 16x16, 4bpp, planes { 0, 4, half + 0, half + 4 }, 4-pixel columns of 8 bytes
		const uint32_t half = s_sprrom_size / 2;
		for (uint32_t t = 0; t < s_spr_count; t++)
			for (int y = 0; y < 16; y++)
				for (int x = 0; x < 16; x++) {
					const uint32_t o = t * 64 + (uint32_t)((x >> 2) * 8 + (y & 7) + (y >> 3) * 32);
					const int k = x & 3;
					const uint8_t v = s_sprrom[o], h = s_sprrom[half + o];
					s_sprpix[t * 256 + (uint32_t)(y * 16 + x)] =
						(uint8_t)((((v >> (7 - k)) & 1) << 3) | (((v >> (3 - k)) & 1) << 2) | (((h >> (7 - k)) & 1) << 1) |
						          ((h >> (3 - k)) & 1));
				}
	}
	s_port_in[0] = rbx_port_find("DSWA");
	s_port_in[1] = rbx_port_find("DSWB");
	s_port_in[2] = rbx_port_find("IN0");
	s_port_in[3] = rbx_port_find("IN1");
	s_port_in[4] = rbx_port_find("IN2");

	memset(&b, 0, sizeof(b));
	m6809_init(&s_cpu);
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	map_pages(&s_cpu, 0x00, 0x10, b.vram[0]);
	map_pages(&s_cpu, 0x10, 0x10, b.vram[1]);
	map_pages(&s_cpu, 0x20, 0x18, b.spr);
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_cpu1 + p * 256;
	memset(&s_mcu, 0, sizeof(s_mcu));
	s_mcu.read = mcu_r;
	s_mcu.write = mcu_w;
	s_mcu.port_r = mcu_port_r;
	s_mcu.rom = s_mcurom;
	s_mcu.rom_base = 0xf000;
	s_mcu.rom_size = 0x1000;
	s_mcu.type = M6801_TYPE_HD63701V0;
	m6801_init(&s_mcu);
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		pl_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	pl_reset();
	return 0;
}

static void pl_exit(void)
{
	free(s_fgpix);
	free(s_bgpix);
	free(s_sprpix);
	free(s_snap);
	free(s_pri);
	free(s_sbm);
	s_fgpix = s_bgpix = s_sprpix = NULL;
	s_snap = NULL;
	s_pri = NULL;
	s_sbm = NULL;
	free(s_pal_hp);
	s_pal_hp = NULL;
}

static int pl_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "board", &b, sizeof(b));
	rc |= rbx_state_io(ctx, "cpu", &s_cpu, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "mcu", m6801_state_data(&s_mcu), m6801_state_size());
	rc |= rbx_state_io(ctx, "wsg", &s_wsg, (uint32_t)NAMCO_WSG_STATE_SIZE);
	if (load && !rc)
		rom_bank();
	return rc;
}

static int pl_save(void *ctx) { return pl_state(ctx, 0); }
static int pl_load(void *ctx) { return pl_state(ctx, 1); }
static int pl_supported(const char *machine) { return !strcmp(machine, "pacland"); }

const rbx_driver rbx_drv_pacland = { "Namco Pac-Land", pl_init, pl_exit, pl_reset, pl_frame, pl_save, pl_load,
                                     pl_supported, pl_render, NULL };
