/*  ReBOX core: game table, ROM loading, MAME-style input ports and DIP switches, video
    conversion, audio resampling and save states. Board drivers live in boards/.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "rbx.h"
#include "rbx_core.h"
#include "rbx_module.h"
#include "zip.h"
#include "cpu/tms57002/tms57002.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <zlib.h>

// ---- the sampled activity (rbx_prof.h, ADR-155) ----
volatile uint8_t rbx_prof_cur;
#ifdef RBX_PROF_CNT
unsigned long long rbx_prof_cnt;
#endif
static const char *s_prof_names[RBX_PROF_IDS] = {
	"other", "front", "draw", "swap", "input", "driver", "video out", "audio out", "wait", "rom read", "io wait",
	"fe pad", "fe audio", "fe ticks",
};
void rbx_prof_name(unsigned first, const char *const *names, unsigned n)
{
	for (unsigned i = 0; i < n && first + i < RBX_PROF_IDS; i++)
		s_prof_names[first + i] = names[i];
}
const char *rbx_prof_get_name(unsigned id)
{
	return id < RBX_PROF_IDS && s_prof_names[id] ? s_prof_names[id] : "";
}

// ---- game tables: the modules' boards, in this order (rbx_module.h) ----
#ifdef RBX_MOD_TAITO
extern const rbx_module rbx_mod_taito;
#endif
#ifdef RBX_MOD_DECO
extern const rbx_module rbx_mod_deco;
#endif
#ifdef RBX_MOD_KONAMI
extern const rbx_module rbx_mod_konami;
#endif
#ifdef RBX_MOD_SETA
extern const rbx_module rbx_mod_seta;
#endif
#ifdef RBX_MOD_IREM
extern const rbx_module rbx_mod_irem;
#endif
#ifdef RBX_MOD_IGS
extern const rbx_module rbx_mod_igs;
#endif
#ifdef RBX_MOD_CAVE
extern const rbx_module rbx_mod_cave;
#endif
#ifdef RBX_MOD_TOAPLAN
extern const rbx_module rbx_mod_toaplan;
#endif
#ifdef RBX_MOD_CAPCOM
extern const rbx_module rbx_mod_capcom;
#endif
#ifdef RBX_MOD_SNK
extern const rbx_module rbx_mod_snk;
#endif
#ifdef RBX_MOD_SEGA
extern const rbx_module rbx_mod_sega;
#endif
#ifdef RBX_MOD_PSIKYO
extern const rbx_module rbx_mod_psikyo;
#endif
#ifdef RBX_MOD_NMK
extern const rbx_module rbx_mod_nmk;
#endif
#ifdef RBX_MOD_NAMCO
extern const rbx_module rbx_mod_namco;
#endif
#ifdef RBX_MOD_KANEKO
extern const rbx_module rbx_mod_kaneko;
#endif
#ifdef RBX_MOD_VSYSTEM
extern const rbx_module rbx_mod_vsystem;
#endif
#ifdef RBX_MOD_JALECO
extern const rbx_module rbx_mod_jaleco;
#endif
#ifdef RBX_MOD_SEIBU
extern const rbx_module rbx_mod_seibu;
#endif
#ifdef RBX_MOD_TECHNOS
extern const rbx_module rbx_mod_technos;
#endif
#ifdef RBX_MOD_ATARI
extern const rbx_module rbx_mod_atari;
#endif
#ifdef RBX_MOD_TECMO
extern const rbx_module rbx_mod_tecmo;
#endif
#ifdef RBX_MOD_MIDWAY
extern const rbx_module rbx_mod_midway;
#endif
static const rbx_module *const s_modules[] = {
#ifdef RBX_MOD_TAITO
	&rbx_mod_taito,
#endif
#ifdef RBX_MOD_DECO
	&rbx_mod_deco,
#endif
#ifdef RBX_MOD_KONAMI
	&rbx_mod_konami,
#endif
#ifdef RBX_MOD_SETA
	&rbx_mod_seta,
#endif
#ifdef RBX_MOD_IREM
	&rbx_mod_irem,
#endif
#ifdef RBX_MOD_IGS
	&rbx_mod_igs,
#endif
#ifdef RBX_MOD_CAVE
	&rbx_mod_cave,
#endif
#ifdef RBX_MOD_TOAPLAN
	&rbx_mod_toaplan,
#endif
#ifdef RBX_MOD_CAPCOM
	&rbx_mod_capcom,
#endif
#ifdef RBX_MOD_SNK
	&rbx_mod_snk,
#endif
#ifdef RBX_MOD_SEGA
	&rbx_mod_sega,
#endif
#ifdef RBX_MOD_PSIKYO
	&rbx_mod_psikyo,
#endif
#ifdef RBX_MOD_NMK
	&rbx_mod_nmk,
#endif
#ifdef RBX_MOD_NAMCO
	&rbx_mod_namco,
#endif
#ifdef RBX_MOD_KANEKO
	&rbx_mod_kaneko,
#endif
#ifdef RBX_MOD_VSYSTEM
	&rbx_mod_vsystem,
#endif
#ifdef RBX_MOD_JALECO
	&rbx_mod_jaleco,
#endif
#ifdef RBX_MOD_SEIBU
	&rbx_mod_seibu,
#endif
#ifdef RBX_MOD_TECHNOS
	&rbx_mod_technos,
#endif
#ifdef RBX_MOD_ATARI
	&rbx_mod_atari,
#endif
#ifdef RBX_MOD_TECMO
	&rbx_mod_tecmo,
#endif
#ifdef RBX_MOD_MIDWAY
	&rbx_mod_midway,
#endif
};
#define NMODULES ((int)(sizeof(s_modules) / sizeof(s_modules[0])))

static const rbx_game *game_at_mod(int i, const rbx_driver **drv, int *mod)
{
	for (int m = 0; m < NMODULES; m++)
		for (int b = 0; b < s_modules[m]->nboards; b++) {
			const rbx_board_tab *t = &s_modules[m]->boards[b];
			if (i < t->count) {
				if (drv)
					*drv = t->drv;
				if (mod)
					*mod = m;
				return &t->games[i];
			}
			i -= t->count;
		}
	return NULL;
}
static const rbx_game *game_at(int i, const rbx_driver **drv) { return game_at_mod(i, drv, NULL); }

int rbx_game_count(void)
{
	int n = 0;
	for (int m = 0; m < NMODULES; m++)
		for (int b = 0; b < s_modules[m]->nboards; b++)
			n += s_modules[m]->boards[b].count;
	return n;
}

int rbx_module_count(void) { return NMODULES; }
const char *rbx_module_id(int m) { return m >= 0 && m < NMODULES ? s_modules[m]->id : NULL; }
const char *rbx_module_name(int m) { return m >= 0 && m < NMODULES ? s_modules[m]->name : NULL; }
int rbx_game_module(int i)
{
	int m = -1;
	game_at_mod(i, NULL, &m);
	return m;
}
// the game's tab (the module's group) and its place in the tab order (modules in order, then their groups)
const char *rbx_game_group(int i)
{
	int m = -1;
	const rbx_game *g = game_at_mod(i, NULL, &m);
	return g ? s_modules[m]->groups[s_modules[m]->group(g)] : NULL;
}
int rbx_game_group_order(int i)
{
	int m = -1;
	const rbx_game *g = game_at_mod(i, NULL, &m);
	return g ? m * 100 + s_modules[m]->group(g) : 100 * NMODULES;
}
int rbx_board_order(const char *board)
{
	for (int m = 0; m < NMODULES; m++) {   // (the first module that knows the board; modules in order)
		const int o = s_modules[m]->board_order(board);
		if (o < 1000)
			return m * 10000 + o;
	}
	return 10000 * NMODULES + 1000;
}
const char *rbx_game_set(int i) { const rbx_game *g = game_at(i, NULL); return g ? g->set : NULL; }
const char *rbx_game_parent(int i) { const rbx_game *g = game_at(i, NULL); return g ? g->parent : NULL; }
const char *rbx_game_title(int i) { const rbx_game *g = game_at(i, NULL); return g ? g->title : NULL; }
const char *rbx_game_year(int i) { const rbx_game *g = game_at(i, NULL); return g ? g->year : NULL; }
const char *rbx_game_maker(int i) { const rbx_game *g = game_at(i, NULL); return g ? g->maker : NULL; }
const char *rbx_game_board(int i) { const rbx_game *g = game_at(i, NULL); return g ? g->board : NULL; }
int rbx_game_rot(int i) { const rbx_game *g = game_at(i, NULL); return g ? g->rot : 0; }
int rbx_game_flags(int i)
{
	const rbx_driver *drv;
	const rbx_game *g = game_at(i, &drv);
	if (!g)
		return 0;
	return g->flags | (drv->supported && !drv->supported(g->machine) ? RBX_GF_UNSUPPORTED : 0);
}
int rbx_game_find(const char *set)
{
	int n = rbx_game_count();
	for (int i = 0; i < n; i++)
		if (!strcmp(rbx_game_set(i), set))
			return i;
	return -1;
}

static void game_controls(const rbx_game *g, int *players, int *buttons)
{
	int pl = 0, bt = 0;
	for (const rbx_ipt_port *p = g->ports; p->tag; p++)
		for (int f = p->first; f < p->first + p->count; f++) {
			const rbx_ipt_field *fd = &g->fields[f];
			if (fd->type >= RBX_IPT_JOYSTICK_UP && fd->type <= RBX_IPT_AD_STICK_Y) {
				if (fd->player > pl)
					pl = fd->player;
				if (fd->player == 1 && fd->type >= RBX_IPT_BUTTON1 && fd->type <= RBX_IPT_BUTTON6 &&
				    fd->type - RBX_IPT_BUTTON1 + 1 > bt)
					bt = fd->type - RBX_IPT_BUTTON1 + 1;
			}
		}
	*players = pl;
	*buttons = bt;
}
int rbx_game_players(int i) { int p = 0, b = 0; const rbx_game *g = game_at(i, NULL); if (g) game_controls(g, &p, &b); return p; }
int rbx_game_buttons(int i) { int p = 0, b = 0; const rbx_game *g = game_at(i, NULL); if (g) game_controls(g, &p, &b); return b; }

// ---- logging / errors ----
static rbx_config s_cfg;
static char s_error[256];

uint64_t rbx_clock_us(void) { return s_cfg.time_us ? s_cfg.time_us() : 0; }
// executable memory: one block from the frontend for the app's life, its parts given out once each (the code in
// them is only ever added to, never rewritten: ADR-012); the emulation thread
#define RBX_JIT_BLOCK 0x1000000
void *rbx_jit_alloc(unsigned size)
{
	static uint8_t *base;
	static unsigned used;
	static int none;
	if (!base && !none) {
		base = s_cfg.jit_alloc ? (uint8_t *)s_cfg.jit_alloc(RBX_JIT_BLOCK) : NULL;
		none = !base;
	}
	size = (size + 63) & ~63u;
	if (!base || size > RBX_JIT_BLOCK - used)
		return NULL;
	used += size;
	return base + used - size;
}
void rbx_jit_write_begin(void) { if (s_cfg.jit_write_begin) s_cfg.jit_write_begin(); }
void rbx_jit_write_end(void *code, unsigned size) { if (s_cfg.jit_write_end) s_cfg.jit_write_end(code, size); }

void rbx_log(const char *fmt, ...)
{
	char buf[512];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (s_cfg.log)
		s_cfg.log(buf);
	else
		fprintf(stderr, "%s\n", buf);
}

static int s_fail_sticky;   // (a missing ROM reported by rbx_region: the driver's own message after it kept out)
void rbx_fail(const char *fmt, ...)
{
	if (s_fail_sticky)
		return;
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(s_error, sizeof(s_error), fmt, ap);
	va_end(ap);
	rbx_log("error: %s", s_error);
}
const char *rbx_error(void) { return s_error; }
const char *rbx_nvram_dir(void) { return s_cfg.nvram_dir; }
const char *rbx_rom_dir(int i) { return i >= 0 && i < s_cfg.rom_dir_count ? s_cfg.rom_dirs[i] : NULL; }

// ---- running game state ----
static const rbx_game *s_game;
static const rbx_driver *s_drv;
static int s_game_index = -1;

// ROM regions
#define MAX_REGIONS 48   // (CPS3: the BIOS and up to 40 flash chips)
static uint8_t *s_reg_data[MAX_REGIONS];
static uint32_t s_reg_size[MAX_REGIONS];
// a region's first missing file: reported when the driver asks for the region (a region it never uses, a PROM or PLD
// of a later MAME's set, does not keep the game from running)
static const rbx_rom_load *s_reg_missing[MAX_REGIONS];
static int s_nreg;
// rbx_driver.file_regions: 1 = to be kept in a file, s_reg_fp its open cache file (NULL: the region is in memory)
static uint8_t s_reg_isfile[MAX_REGIONS];
static FILE *s_reg_fp[MAX_REGIONS];
// a file region loaded with ROM_LOAD16_WORD_SWAP: rbx_region_read gives the file's bytes (each pair not swapped)
static uint8_t s_reg_swap16[MAX_REGIONS];

// a region the driver can do without (a newer dump than the user's ROM set): NULL when it is missing, no error
uint8_t *rbx_region_opt(const char *name, uint32_t *size)
{
	for (int i = 0; s_game && i < s_nreg; i++)
		if (!strcmp(s_game->regions[i].name, name) && s_reg_missing[i]) {
			if (size)
				*size = 0;
			return NULL;
		}
	return rbx_region(name, size);
}

static int region_index(const char *name)
{
	for (int i = 0; s_game && i < s_nreg; i++)
		if (!strcmp(s_game->regions[i].name, name))
			return i;
	return -1;
}

uint32_t rbx_region_file(const char *name)
{
	const int i = region_index(name);
	if (i < 0 || !s_reg_isfile[i])
		return 0;
	if (s_reg_missing[i]) {
		rbx_fail("%s: missing %s (crc %08x)", s_game->set, s_reg_missing[i]->file, (unsigned)s_reg_missing[i]->crc);
		s_fail_sticky = 1;
		return 0;
	}
	return s_reg_size[i];
}

int rbx_region_file_swap16(const char *name)
{
	for (int i = 0; s_game && i < s_nreg; i++)
		if (!strcmp(s_game->regions[i].name, name))
			return s_reg_isfile[i] && s_reg_swap16[i];
	return 0;
}

/* (ADR-178) A file region's cache file is read without stdio's buffer: newlib (the Vita) reads a buffered stream
   BUFSIZ = 1 KB a system call (a 16.5 KB NAND chunk = 17 sceIoRead, and fseek reads a block too); unbuffered, fread
   is one read of the whole length into the caller's memory. The bytes are the same. RBX_IO_STDBUF=1: buffered as
   before (set before the first open) */
static void region_unbuffered(FILE *f)
{
	static int keep = -1;
	if (keep < 0)
		keep = getenv("RBX_IO_STDBUF") && atoi(getenv("RBX_IO_STDBUF"));
	if (f && !keep)
		setvbuf(f, NULL, _IONBF, 0);
}

/* (testing, ADR-177: RBX_IO_SLOW=us,us_per_KB: every card read of a file region, here and on the io worker, waits that
   long after it (us a system call: with RBX_IO_STDBUF=1 one per KB and one for the seek), as a slow memory card would
   take; the bytes are the same) */
static int s_io_slow_set;
static uint32_t s_io_slow_us, s_io_slow_kb;
static void io_slow(uint32_t len)
{
	if (!s_io_slow_set) {
		const char *e = getenv("RBX_IO_SLOW");
		if (e)
			sscanf(e, "%u,%u", &s_io_slow_us, &s_io_slow_kb);
		s_io_slow_set = 1;
	}
	// (a buffered stream: a system call each BUFSIZ and one for the fseek, ADR-178)
	const uint32_t calls = getenv("RBX_IO_STDBUF") && atoi(getenv("RBX_IO_STDBUF")) ? (len + 1023) / 1024 + 1 : 1;
	const uint64_t us = (uint64_t)s_io_slow_us * calls + (uint64_t)s_io_slow_kb * len / 1024;
	if (us && s_cfg.time_us) {
		const uint64_t t0 = rbx_clock_us();
		while (rbx_clock_us() - t0 < us)
			;
	}
}

int rbx_region_read(const char *name, uint32_t offset, void *dst, uint32_t len)
{
	const int i = region_index(name);
	if (i < 0 || s_reg_missing[i] || offset > s_reg_size[i] || len > s_reg_size[i] - offset)
		return -1;
	const unsigned pa = rbx_prof_enter(RBX_PA_ROMREAD);
	int rc = 0;
	if (s_reg_data[i])
		memcpy(dst, s_reg_data[i] + offset, len);
	else if (!s_reg_fp[i] || fseek(s_reg_fp[i], 16 + (long)offset, SEEK_SET))
		rc = -1;
	else
		rc = fread(dst, 1, len, s_reg_fp[i]) == len ? 0 : -1;
	if (!s_reg_data[i])
		io_slow(len);
	rbx_prof_leave(pa);
	return rc;
}

// ---- reads ahead of a file region on a worker thread (ADR-160) ----
// The emulation thread queues reads it will soon need (rbx_io_submit); the worker reads them with its own FILE of the
// region's cache file, in order. The emulation thread needing one takes it back unread (and reads it itself) when the
// worker has not started it, or waits for it when it has: the bytes are the file's either way.
// Requests live here (not in the driver), so a worker still holding an old id never touches freed memory.
#define IO_N 64
#define IO_RING 256
enum { IO_FREE, IO_QUEUED, IO_READING, IO_DONE, IO_FAILED };
static struct io_req { volatile int state; int reg; uint32_t off, len; void *dst; } s_io[IO_N];
static int s_io_ring[IO_RING];
static volatile unsigned s_io_head, s_io_tail;   // pushed by the emulation thread / popped by the worker
static volatile int s_io_on;
static void (*volatile s_io_wake)(void);
static char *s_reg_path[MAX_REGIONS];   // a file region's cache file (the worker opens its own)
static char *path_dup(const char *p)
{
	const size_t n = strlen(p) + 1;
	char *d = (char *)malloc(n);
	if (d)
		memcpy(d, p, n);
	return d;
}
static volatile int s_io_close;         // the regions released: the worker closes its file
static struct { uint64_t worker, taken, waits, wait_us, reads; } s_io_st;   // (the driver's statistics: rbx_io_stat)
static void wait_some(void);

int rbx_io_on(void) { return __atomic_load_n(&s_io_on, __ATOMIC_ACQUIRE); }
void rbx_set_io_thread(int on) { __atomic_store_n(&s_io_on, on, __ATOMIC_RELEASE); }
void rbx_set_io_wake(void (*wake)(void)) { s_io_wake = wake; }
int rbx_io_wanted(void)
{
	for (int i = 0; i < s_nreg; i++)
		if (s_reg_fp[i] && s_reg_path[i])
			return 1;
	return 0;
}

int rbx_io_submit(const char *name, uint32_t offset, void *dst, uint32_t len)
{
	if (!rbx_io_on())
		return -1;
	const int i = region_index(name);
	if (i < 0 || !s_reg_fp[i] || !s_reg_path[i] || offset > s_reg_size[i] || len > s_reg_size[i] - offset)
		return -1;
	const unsigned head = s_io_head;
	if (head - __atomic_load_n(&s_io_tail, __ATOMIC_ACQUIRE) >= IO_RING)
		return -1;
	int id = -1;
	for (int k = 0; k < IO_N && id < 0; k++)
		if (__atomic_load_n(&s_io[k].state, __ATOMIC_ACQUIRE) == IO_FREE)
			id = k;
	if (id < 0)
		return -1;
	struct io_req *r = &s_io[id];
	r->reg = i;
	r->off = offset;
	r->len = len;
	r->dst = dst;
	__atomic_store_n(&r->state, IO_QUEUED, __ATOMIC_RELEASE);
	s_io_ring[head % IO_RING] = id;
	__atomic_store_n(&s_io_head, head + 1, __ATOMIC_RELEASE);
	void (*wake)(void) = s_io_wake;
	if (wake)
		wake();
	return id;
}

int rbx_io_ready(int id)   // read by the worker (or failed there): rbx_io_finish will not wait
{
	const int st = __atomic_load_n(&s_io[id].state, __ATOMIC_ACQUIRE);
	return st == IO_DONE || st == IO_FAILED;
}

int rbx_io_queued(int id) { return __atomic_load_n(&s_io[id].state, __ATOMIC_ACQUIRE) == IO_QUEUED; }

// the request's bytes in place (0) or not readable (-1); the id is free again after
int rbx_io_finish(int id)
{
	struct io_req *r = &s_io[id];
	int st = IO_QUEUED;
	if (__atomic_compare_exchange_n(&r->state, &st, IO_FREE, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
		s_io_st.taken++;   // (not started: read here)
		return rbx_region_read(s_game->regions[r->reg].name, r->off, r->dst, r->len);
	}
	if (st == IO_READING) {
		const unsigned pa = rbx_prof_enter(RBX_PA_IOWAIT);
		const uint64_t t0 = rbx_clock_us();
		s_io_st.waits++;
		for (int spin = 0; __atomic_load_n(&r->state, __ATOMIC_ACQUIRE) == IO_READING; spin++)
			if (spin >= 200)
				wait_some();
		s_io_st.wait_us += rbx_clock_us() - t0;
		rbx_prof_leave(pa);
	}
	const int rc = __atomic_load_n(&r->state, __ATOMIC_ACQUIRE) == IO_DONE ? 0 : -1;
	__atomic_store_n(&r->state, IO_FREE, __ATOMIC_RELEASE);
	return rc;
}

// not needed any more: taken back unread, or waited for while the worker reads into it
void rbx_io_drop(int id)
{
	struct io_req *r = &s_io[id];
	int st = IO_QUEUED;
	if (__atomic_compare_exchange_n(&r->state, &st, IO_FREE, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return;
	if (__atomic_load_n(&r->state, __ATOMIC_ACQUIRE) == IO_READING) {
		const unsigned pa = rbx_prof_enter(RBX_PA_IOWAIT);
		while (__atomic_load_n(&r->state, __ATOMIC_ACQUIRE) == IO_READING)
			wait_some();
		rbx_prof_leave(pa);
	}
	__atomic_store_n(&r->state, IO_FREE, __ATOMIC_RELEASE);
}

int rbx_io_cancel(int id)
{
	int st = IO_QUEUED;
	return __atomic_compare_exchange_n(&s_io[id].state, &st, IO_FREE, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

/* (ADR-177) The worker reads requests that follow each other in the file (the queued ones right after it in the ring,
   the same region, each starting where the one before ends) with one read of up to s_io_merge bytes: the card's
   time per read is mostly its own, not the bytes (a burst of the CV1000's 16.5 KB chunks). Into their buffers when
   those follow each other in memory too, else into the worker's buffer and copied. The bytes are the file's either
   way. RBX_IO_MERGE = bytes (0: one request a read, as before) */
#define IO_MERGE_MAX (256 * 1024)
static uint32_t s_io_merge = 64 * 1024;
static int s_io_merge_set;

// the worker: the queued requests read in order; the number read (0: none queued, the thread may sleep)
int rbx_io_run(void)
{
	static FILE *fp;
	static char path[512];
	static uint8_t *stage;
	int n = 0;
	if (!s_io_merge_set) {
		const char *e = getenv("RBX_IO_MERGE");
		if (e)
			s_io_merge = (uint32_t)atoi(e) < IO_MERGE_MAX ? (uint32_t)atoi(e) : IO_MERGE_MAX;
		s_io_merge_set = 1;
	}
	if (fp && __atomic_load_n(&s_io_close, __ATOMIC_ACQUIRE)) {   // (the last game's file: closed before this one's reads)
		fclose(fp);
		fp = NULL;
		__atomic_store_n(&s_io_close, 0, __ATOMIC_RELEASE);
	}
	for (;;) {
		const unsigned tail = s_io_tail;
		if (tail == __atomic_load_n(&s_io_head, __ATOMIC_ACQUIRE))
			break;
		const int id = s_io_ring[tail % IO_RING];
		__atomic_store_n(&s_io_tail, tail + 1, __ATOMIC_RELEASE);
		struct io_req *r = &s_io[id];
		int st = IO_QUEUED;
		if (!__atomic_compare_exchange_n(&r->state, &st, IO_READING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
			continue;   // (taken back, or a second entry of an id queued again)
		const char *p = s_reg_path[r->reg];
		if (fp && (!p || strcmp(p, path))) {
			fclose(fp);
			fp = NULL;
		}
		if (!fp && p && strlen(p) < sizeof(path)) {
			strcpy(path, p);
			fp = fopen(path, "rb");
			region_unbuffered(fp);
		}
		// (the requests after it that follow it in the file, taken now: reading)
		int ids[IO_N], k = 1, direct = 1;
		uint32_t len = r->len;
		ids[0] = id;
		if (s_io_merge && !stage)
			stage = (uint8_t *)malloc(IO_MERGE_MAX);
		for (unsigned t = tail + 1; stage && k < IO_N && t != __atomic_load_n(&s_io_head, __ATOMIC_ACQUIRE); t++) {
			const int j = s_io_ring[t % IO_RING];
			struct io_req *q = &s_io[j];
			if (__atomic_load_n(&q->state, __ATOMIC_ACQUIRE) != IO_QUEUED || q->reg != r->reg || q->off != r->off + len ||
			    len + q->len > s_io_merge)
				break;
			int qs = IO_QUEUED;
			if (!__atomic_compare_exchange_n(&q->state, &qs, IO_READING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
				break;   // (taken back just now)
			if ((uint8_t *)q->dst != (uint8_t *)s_io[ids[k - 1]].dst + s_io[ids[k - 1]].len)
				direct = 0;
			ids[k++] = j;
			len += q->len;
		}
		int ok = fp && !fseek(fp, 16 + (long)r->off, SEEK_SET);
		if (k == 1 || direct)
			ok = ok && fread(r->dst, 1, len, fp) == len;
		else {
			ok = ok && fread(stage, 1, len, fp) == len;
			for (int i = 0, at = 0; ok && i < k; at += (int)s_io[ids[i]].len, i++)
				memcpy(s_io[ids[i]].dst, stage + at, s_io[ids[i]].len);
		}
		io_slow(len);
		s_io_st.worker += (uint64_t)k;
		s_io_st.reads++;
		for (int i = 0; i < k; i++)
			__atomic_store_n(&s_io[ids[i]].state, ok ? IO_DONE : IO_FAILED, __ATOMIC_RELEASE);
		n += k;
	}
	if (fp && __atomic_load_n(&s_io_close, __ATOMIC_ACQUIRE)) {
		fclose(fp);
		fp = NULL;
		__atomic_store_n(&s_io_close, 0, __ATOMIC_RELEASE);
	}
	return n;
}

uint64_t rbx_io_nreads(void) { return s_io_st.reads; }

void rbx_io_stat(uint64_t *worker, uint64_t *taken, uint64_t *waits, uint64_t *wait_us)
{
	*worker = s_io_st.worker, *taken = s_io_st.taken, *waits = s_io_st.waits, *wait_us = s_io_st.wait_us;
}

uint8_t *rbx_region(const char *name, uint32_t *size)
{
	if (!s_game)
		return NULL;
	for (int i = 0; i < s_nreg; i++)
		if (!strcmp(s_game->regions[i].name, name)) {
			if (s_reg_missing[i]) {
				rbx_fail("%s: missing %s (crc %08x)", s_game->set, s_reg_missing[i]->file, (unsigned)s_reg_missing[i]->crc);
				s_fail_sticky = 1;
				if (size)
					*size = 0;
				return NULL;
			}
			if (size)
				*size = s_reg_size[i];
			return s_reg_data[i];
		}
	if (size)
		*size = 0;
	return NULL;
}

static void free_regions(void)
{
	for (int i = 0; i < s_nreg; i++) {
		free(s_reg_data[i]);
		s_reg_data[i] = NULL;
		if (s_reg_fp[i])
			fclose(s_reg_fp[i]);
		s_reg_fp[i] = NULL;
		s_reg_isfile[i] = 0;
		free(s_reg_path[i]);
		s_reg_path[i] = NULL;
		__atomic_store_n(&s_io_close, 1, __ATOMIC_RELEASE);
	}
	s_nreg = 0;
	memset(&s_io_st, 0, sizeof(s_io_st));
}

// zips of the set and its parents, in lookup order
#define MAX_ZIPS 4
static rbx_zip *s_zips[MAX_ZIPS];
static int s_nzips;

static rbx_zip *open_set_zip(const char *set)
{
	char path[512];
	for (int d = 0; d < s_cfg.rom_dir_count; d++) {
		snprintf(path, sizeof(path), "%s%s.zip", s_cfg.rom_dirs[d], set);
		rbx_zip *z = rbx_zip_open(path);
		if (z)
			return z;
	}
	return NULL;
}

static const rbx_game *find_game(const char *set)
{
	int i = rbx_game_find(set);
	return i >= 0 ? game_at(i, NULL) : NULL;
}

static uint8_t *read_rom_file(const rbx_rom_load *l, uint32_t *size)
{
	for (int z = 0; z < s_nzips; z++) {
		int i = rbx_zip_find(s_zips[z], l->file, l->crc, l->length);
		if (i < 0)
			continue;
		const rbx_zip_entry *e = rbx_zip_get(s_zips[z], i);
		uint8_t *buf = (uint8_t *)malloc(e->size ? e->size : 1);
		if (!buf || rbx_zip_read(s_zips[z], i, buf)) {
			free(buf);
			return NULL;
		}
		*size = e->size;
		return buf;
	}
	return NULL;
}

// a file region (rbx_driver.file_regions): the ROM file kept in <cache_dir>rbx_<region>.cache (a 16-byte header:
// "RBXC", CRC, size, 0; written last), extracted again when the header is not this file's. Without the cache folder or
// on a write error the file goes to memory as other regions do (streamed: no second copy). 0 = ok, -2 = no memory
typedef struct { FILE *fp; uint8_t *mem; uint32_t pos, size, crc; int done, total; } extract_ctx;

static int extract_cb(void *user, const uint8_t *data, uint32_t len)
{
	extract_ctx *x = (extract_ctx *)user;
	if (x->fp && fwrite(data, 1, len, x->fp) != len)
		return -1;
	if (x->mem)
		memcpy(x->mem + x->pos, data, len);
	x->crc = (uint32_t)crc32(x->crc, data, len);
	x->pos += len;
	if (s_cfg.progress && x->total && x->size)
		s_cfg.progress((int)(((uint64_t)x->done * x->size + x->pos) * 900 / ((uint64_t)x->total * x->size)), s_cfg.progress_user);
	return 0;
}

static uint32_t rd_le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

// a region's loads done (MAME rom_load_manager::region_post_process): ROMREGION_INVERT
static void region_post(const rbx_game *g, int reg)
{
	if (reg >= s_nreg || !s_reg_data[reg] || !(g->regions[reg].flags & RBX_REGION_INVERT))
		return;
	uint8_t *d = s_reg_data[reg];
	for (uint32_t i = 0; i < s_reg_size[reg]; i++)
		d[i] = (uint8_t)~d[i];
}

static int file_region_load(int reg, const rbx_rom_load *l, int done, int total)
{
	rbx_zip *z = NULL;
	int zi = -1;
	for (int k = 0; k < s_nzips && zi < 0; k++)
		if ((zi = rbx_zip_find(s_zips[k], l->file, l->crc, l->length)) >= 0)
			z = s_zips[k];
	if (zi < 0)
		return -1;
	const rbx_zip_entry *e = rbx_zip_get(z, zi);
	if (e->size != s_reg_size[reg])
		return -1;
	if (s_cfg.cache_dir) {
		char nm[64], path[512], tmp[540];
		int n = 0;
		for (const char *c = s_game->regions[reg].name; *c && n < 63; c++)
			nm[n++] = (char)((*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9') ? *c : '_');
		nm[n] = 0;
		snprintf(path, sizeof(path), "%srbx_%s.cache", s_cfg.cache_dir, nm);
		FILE *f = fopen(path, "rb");
		region_unbuffered(f);
		uint8_t h[16];
		if (f && fread(h, 1, 16, f) == 16 && !memcmp(h, "RBXC", 4) && rd_le32(h + 4) == e->crc && rd_le32(h + 8) == e->size &&
		    !fseek(f, 0, SEEK_END) && ftell(f) == 16 + (long)e->size) {
			s_reg_fp[reg] = f;
			s_reg_path[reg] = path_dup(path);
			rbx_log("%s: %s from %s", s_game->set, s_game->regions[reg].name, path);
			return 0;
		}
		if (f)
			fclose(f);
		// (a name of this run: another process may be making the same cache)
		int stack_mark;
		snprintf(tmp, sizeof(tmp), "%s.%08x.tmp", path,
		         (unsigned)((uintptr_t)&stack_mark ^ (uintptr_t)time(NULL) ^ (uintptr_t)clock()));
		extract_ctx x = { fopen(tmp, "wb"), NULL, 0, e->size, 0, done, total };
		static const uint8_t blank[16] = { 0 };
		int ok = x.fp && fwrite(blank, 1, 16, x.fp) == 16 && !rbx_zip_read_cb(z, zi, extract_cb, &x) && x.crc == e->crc;
		if (ok) {
			const uint8_t hd[16] = { 'R', 'B', 'X', 'C', (uint8_t)e->crc, (uint8_t)(e->crc >> 8), (uint8_t)(e->crc >> 16),
			                         (uint8_t)(e->crc >> 24), (uint8_t)e->size, (uint8_t)(e->size >> 8), (uint8_t)(e->size >> 16),
			                         (uint8_t)(e->size >> 24), 0, 0, 0, 0 };
			ok = !fseek(x.fp, 0, SEEK_SET) && fwrite(hd, 1, 16, x.fp) == 16;
		}
		if (x.fp && fclose(x.fp))
			ok = 0;
		if (ok) {
			remove(path);
			ok = !rename(tmp, path) && (s_reg_fp[reg] = fopen(path, "rb")) != NULL;
			if (ok) {
				region_unbuffered(s_reg_fp[reg]);
				s_reg_path[reg] = path_dup(path);
			}
		}
		if (ok) {
			rbx_log("%s: %s extracted to %s", s_game->set, s_game->regions[reg].name, path);
			return 0;
		}
		remove(tmp);
		rbx_log("%s: %s could not be written to %s, kept in memory", s_game->set, s_game->regions[reg].name, path);
	}
	if (!(s_reg_data[reg] = (uint8_t *)malloc(s_reg_size[reg]))) {
		rbx_fail("out of memory (%s, %u bytes)", s_game->regions[reg].name, (unsigned)s_reg_size[reg]);
		return -2;
	}
	extract_ctx x = { NULL, s_reg_data[reg], 0, e->size, 0, done, total };
	return rbx_zip_read_cb(z, zi, extract_cb, &x) || x.crc != e->crc ? -1 : 0;
}

// copy file bytes [*pos, *pos+length) into a region following the ROMX_LOAD layout
static void place(uint8_t *reg, uint32_t regsize, uint32_t offset, const uint8_t *src, uint32_t srclen,
                  uint32_t *pos, uint32_t length, int group, int skip, int reverse)
{
	if (group < 1)
		group = 1;
	uint32_t d = offset;
	if (reverse >= 2) {   // ROM_NIBBLE: each file byte's low 4 bits into the low (2) or high (3) nibble
		const int sh = reverse == 3 ? 4 : 0;
		const uint8_t mask = (uint8_t)(0x0f << sh);
		for (uint32_t n = 0; n < length; n++, d += 1 + (uint32_t)skip, (*pos)++)
			if (d < regsize && *pos < srclen)
				reg[d] = (uint8_t)((reg[d] & ~mask) | ((src[*pos] << sh) & mask));
		return;
	}
	for (uint32_t n = 0; n < length; n += group) {
		for (int b = 0; b < group; b++) {
			int sb = reverse ? group - 1 - b : b;
			uint32_t sp = *pos + sb;
			if (d + b < regsize && sp < srclen && n + sb < length)
				reg[d + b] = src[sp];
		}
		*pos += group;
		d += group + skip;
	}
}

// ReBOX: a ROM file placed while it is unpacked (no buffer of the whole file): the bytes place() would put, in groups
typedef struct {
	uint8_t *reg;
	uint32_t regsize, d, n, length;
	int group, skip, reverse, ncarry;
	uint8_t carry[8];
} stream_place;

static void stream_group(stream_place *p, int have)
{
	for (int b = 0; b < p->group; b++) {
		const int sb = p->reverse ? p->group - 1 - b : b;
		if (p->d + b < p->regsize && sb < have && p->n + sb < p->length)
			p->reg[p->d + b] = p->carry[sb];
	}
	p->n += (uint32_t)p->group;
	p->d += (uint32_t)(p->group + p->skip);
}

static int stream_cb(void *user, const uint8_t *data, uint32_t len)
{
	stream_place *p = (stream_place *)user;
	if (p->group == 1 && !p->skip) {   // (plain: a copy)
		if (p->n < p->length && p->d < p->regsize) {
			uint32_t k = len;
			if (k > p->length - p->n) k = p->length - p->n;
			if (k > p->regsize - p->d) k = p->regsize - p->d;
			memcpy(p->reg + p->d, data, k);
		}
		p->n += len;
		p->d += len;
		return 0;
	}
	for (uint32_t i = 0; i < len; i++) {
		p->carry[p->ncarry++] = data[i];
		if (p->ncarry == p->group) {
			stream_group(p, p->group);
			p->ncarry = 0;
		}
	}
	return 0;
}

// 0 = placed, -1 = not found / not readable
static int stream_rom_file(const rbx_rom_load *l, uint8_t *reg, uint32_t rsize)
{
	for (int z = 0; z < s_nzips; z++) {
		const int i = rbx_zip_find(s_zips[z], l->file, l->crc, l->length);
		if (i < 0)
			continue;
		stream_place p;
		memset(&p, 0, sizeof(p));
		p.reg = reg;
		p.regsize = rsize;
		p.d = l->offset;
		p.length = l->length;
		p.group = l->group ? l->group : 1;
		p.skip = l->skip;
		p.reverse = l->reverse;
		if (p.group > 8)
			return -2;
		if (rbx_zip_read_cb(s_zips[z], i, stream_cb, &p))
			return -1;
		if (p.ncarry)
			stream_group(&p, p.ncarry);
		return 0;
	}
	return -1;
}

static int load_roms(const rbx_game *g, const rbx_driver *drv)
{
	// zips: the set, then its parent chain
	s_nzips = 0;
	for (const rbx_game *p = g; p && s_nzips < MAX_ZIPS; p = p->parent ? find_game(p->parent) : NULL) {
		rbx_zip *z = open_set_zip(p->set);
		if (z)
			s_zips[s_nzips++] = z;
	}
	if (!s_nzips) {
		rbx_fail("%s.zip not found", g->set);
		return -1;
	}
	s_nreg = 0;
	s_fail_sticky = 0;
	memset(s_reg_missing, 0, sizeof(s_reg_missing));
	for (const rbx_rom_region *r = g->regions; r->name && s_nreg < MAX_REGIONS; r++) {
		// a file region: one whole ROM file, nothing else loaded into it
		int isfile = 0, loads = 0;
		s_reg_swap16[s_nreg] = 0;
		for (const char *const *f = drv ? drv->file_regions : NULL; f && *f && !isfile; f++)
			isfile = !strcmp(*f, r->name);
		for (const rbx_rom_load *l = g->loads; isfile && l->op != RBX_ROM_END; l++)
			if (l->region == s_nreg && (++loads > 1 || l->op != RBX_ROM_LOAD || l->offset || l->length != r->size ||
			                            l->skip || ((l->group > 1 || l->reverse) && !(l->group == 2 && l->reverse == 1))))
				isfile = 0;
			else if (l->region == s_nreg)
				s_reg_swap16[s_nreg] = l->group == 2;   // (ROM_LOAD16_WORD_SWAP: the file's bytes, the driver swaps)
		if (!isfile)
			s_reg_swap16[s_nreg] = 0;
		s_reg_isfile[s_nreg] = (uint8_t)(isfile && loads == 1 && !(r->flags & RBX_REGION_INVERT));
		s_reg_fp[s_nreg] = NULL;
		if (s_reg_isfile[s_nreg]) {
			s_reg_data[s_nreg] = NULL;
			s_reg_size[s_nreg] = r->size;
			s_nreg++;
			continue;
		}
		s_reg_data[s_nreg] = (uint8_t *)malloc(r->size);
		if (!s_reg_data[s_nreg]) {
			rbx_fail("out of memory (%s, %u bytes)", r->name, (unsigned)r->size);
			return -1;
		}
		memset(s_reg_data[s_nreg], r->fill, r->size);
		s_reg_size[s_nreg] = r->size;
		s_nreg++;
	}
	int total = 0, done = 0;
	for (const rbx_rom_load *l = g->loads; l->op != RBX_ROM_END; l++)
		total++;
	uint8_t *file = NULL;
	uint32_t flen = 0, fpos = 0;
	const rbx_rom_load *last = NULL;
	int rc = 0, fin = 0;   // (the regions before fin are done: their post-processing applied, as MAME region_post_process)
	for (const rbx_rom_load *l = g->loads; l->op != RBX_ROM_END; l++, done++) {
		for (; fin < l->region; fin++)
			region_post(g, fin);
		uint8_t *reg = s_reg_data[l->region];
		uint32_t rsize = s_reg_size[l->region];
		if (s_reg_isfile[l->region]) {   // (its one RBX_ROM_LOAD)
			const int fr = file_region_load(l->region, l, done, total);
			if (fr == -2) {
				rc = -1;
				goto out;
			}
			if (fr) {
				rbx_log("%s: missing %s (crc %08x)", g->set, l->file, (unsigned)l->crc);
				if (!s_reg_missing[l->region])
					s_reg_missing[l->region] = l;
			}
			last = NULL;
			continue;
		}
		switch (l->op) {
		case RBX_ROM_LOAD:
			free(file);
			file = NULL;
			// (not used again by a CONTINUE / RELOAD after it: unpacked straight into the region)
			if (l[1].op != RBX_ROM_CONTINUE && l[1].op != RBX_ROM_RELOAD && l->reverse < 2) {
				const int sr = stream_rom_file(l, reg, rsize);
				if (!sr) {
					last = NULL;
					break;
				}
				if (sr == -1) {
					rbx_log("%s: missing %s (crc %08x)", g->set, l->file, (unsigned)l->crc);
					if (!s_reg_missing[l->region])
						s_reg_missing[l->region] = l;
					last = NULL;
					break;
				}
			}
			file = read_rom_file(l, &flen);
			if (!file) {   // (reported if the driver asks for this region: rbx_region)
				rbx_log("%s: missing %s (crc %08x)", g->set, l->file, (unsigned)l->crc);
				if (!s_reg_missing[l->region])
					s_reg_missing[l->region] = l;
				last = NULL;
				break;
			}
			last = l;
			fpos = 0;
			place(reg, rsize, l->offset, file, flen, &fpos, l->length, l->group, l->skip, l->reverse);
			break;
		case RBX_ROM_CONTINUE:
			if (file && last)
				place(reg, rsize, l->offset, file, flen, &fpos, l->length, last->group, last->skip, last->reverse);
			break;
		case RBX_ROM_RELOAD:
			if (file && last) {
				fpos = 0;
				place(reg, rsize, l->offset, file, flen, &fpos, l->length, last->group, last->skip, last->reverse);
			}
			break;
		case RBX_ROM_FILL:
			if (l->offset < rsize)
				memset(reg + l->offset, (int)l->value, l->offset + l->length <= rsize ? l->length : rsize - l->offset);
			break;
		case RBX_ROM_COPY: {
			int sr = (int)(l->value >> 24);
			uint32_t so = l->value & 0xffffff;
			if (so + l->length <= s_reg_size[sr] && l->offset + l->length <= rsize)
				memmove(reg + l->offset, s_reg_data[sr] + so, l->length);
			break;
		}
		}
		if (s_cfg.progress && total)
			s_cfg.progress(done * 900 / total, s_cfg.progress_user);
	}
	for (; fin < s_nreg; fin++)
		region_post(g, fin);
out:
	free(file);
	for (int z = 0; z < s_nzips; z++)
		rbx_zip_close(s_zips[z]);
	s_nzips = 0;
	return rc;
}

// ---- input ports ----
#define MAX_PORTS 32
#define MAX_FIELDS 256
static int s_nports;
static uint32_t s_port_val[MAX_PORTS];
static uint32_t s_custom_mask[MAX_PORTS], s_custom_val[MAX_PORTS];
static uint32_t s_custom_inv[MAX_PORTS];   // active-low IPT_CUSTOM bits (the driver gives line levels: 1 = asserted)
static uint32_t s_dip_val[MAX_FIELDS];      // current value of each DIP / config field
static uint8_t s_impulse[MAX_FIELDS], s_prev_on[MAX_FIELDS];
static uint32_t s_in_player[4], s_in_sys;
static int s_analog[4][2];

int rbx_port_find(const char *tag)
{
	for (int i = 0; s_game && s_game->ports[i].tag; i++)
		if (!strcmp(s_game->ports[i].tag, tag))
			return i;
	return -1;
}

uint32_t rbx_port_read(int port)
{
	if (port < 0 || port >= s_nports)
		return 0xffffffff;
	return (s_port_val[port] & ~s_custom_mask[port]) | ((s_custom_val[port] ^ s_custom_inv[port]) & s_custom_mask[port]);
}

void rbx_port_custom(int port, uint32_t mask, uint32_t value)
{
	if (port < 0 || port >= s_nports)
		return;
	s_custom_mask[port] |= mask;
	s_custom_val[port] = (s_custom_val[port] & ~mask) | (value & mask);
}

int rbx_analog(int player, int type)
{
	if (player < 1 || player > 4)
		return 0;
	int y = type == RBX_IPT_TRACKBALL_Y || type == RBX_IPT_AD_STICK_Y || type == RBX_IPT_LIGHTGUN_Y;
	int v = s_analog[player - 1][y];
	if (type == RBX_IPT_PEDAL)   // pedals: stick up = pressed further
		v = -s_analog[player - 1][1];
	if (!v && (type == RBX_IPT_LIGHTGUN_X || type == RBX_IPT_LIGHTGUN_Y)) {
		// no pointing device: the d-pad moves the light gun crosshair (4 counts per frame; boards with paddles or
		// trackballs do their own d-pad fallback)
		uint32_t in = s_in_player[player - 1];
		if (y)
			v = ((in & RBX_IN_DOWN) ? 4 : 0) - ((in & RBX_IN_UP) ? 4 : 0);
		else
			v = ((in & RBX_IN_RIGHT) ? 4 : 0) - ((in & RBX_IN_LEFT) ? 4 : 0);
	}
	return v;
}

int rbx_analog_dpad(int player, int type, int step)
{
	const int v = rbx_analog(player, type);
	if (v || player < 1 || player > 4)
		return v;
	const uint32_t in = s_in_player[player - 1];
	if (type == RBX_IPT_TRACKBALL_Y || type == RBX_IPT_AD_STICK_Y || type == RBX_IPT_LIGHTGUN_Y)
		return ((in & RBX_IN_DOWN) ? step : 0) - ((in & RBX_IN_UP) ? step : 0);
	return ((in & RBX_IN_RIGHT) ? step : 0) - ((in & RBX_IN_LEFT) ? step : 0);
}

uint32_t rbx_player_input(int player) { return player >= 1 && player <= 4 ? s_in_player[player - 1] : 0; }

uint32_t rbx_input(int player) { return player >= 1 && player <= 4 ? s_in_player[player - 1] : 0; }
void rbx_set_input(int player, uint32_t bits) { if (player >= 0 && player < 4) s_in_player[player] = bits; }
uint32_t rbx_input_bits(int player) { return player >= 1 && player <= 4 ? s_in_player[player - 1] : 0; }
void rbx_set_system(uint32_t bits) { s_in_sys = bits; }
void rbx_set_analog(int player, int axis, int delta) { if (player >= 0 && player < 4 && axis >= 0 && axis < 2) s_analog[player][axis] = delta; }

static int field_pressed(const rbx_ipt_field *f)
{
	int p = f->player >= 1 && f->player <= 4 ? f->player - 1 : 0;
	uint32_t in = s_in_player[p];
	switch (f->type) {
	case RBX_IPT_JOYSTICK_UP: return !!(in & RBX_IN_UP);
	case RBX_IPT_JOYSTICK_DOWN: return !!(in & RBX_IN_DOWN);
	case RBX_IPT_JOYSTICK_LEFT: return !!(in & RBX_IN_LEFT);
	case RBX_IPT_JOYSTICK_RIGHT: return !!(in & RBX_IN_RIGHT);
	case RBX_IPT_BUTTON1: case RBX_IPT_BUTTON2: case RBX_IPT_BUTTON3:
	case RBX_IPT_BUTTON4: case RBX_IPT_BUTTON5: case RBX_IPT_BUTTON6:
		return !!(in & (RBX_IN_B1 << (f->type - RBX_IPT_BUTTON1)));
	case RBX_IPT_PEDAL: return !!(in & RBX_IN_B1);   // no analog pedal: button 1 = full throttle
	case RBX_IPT_START1: case RBX_IPT_START2: case RBX_IPT_START3: case RBX_IPT_START4:
		return !!(s_in_player[f->type - RBX_IPT_START1] & RBX_IN_START);
	case RBX_IPT_COIN1: case RBX_IPT_COIN2: case RBX_IPT_COIN3: case RBX_IPT_COIN4:
		return !!(s_in_player[f->type - RBX_IPT_COIN1] & RBX_IN_COIN);
	case RBX_IPT_SERVICE1: case RBX_IPT_SERVICE2: case RBX_IPT_SERVICE3: case RBX_IPT_SERVICE4:
		return !!(s_in_player[f->type - RBX_IPT_SERVICE1] & RBX_IN_SERVICE) ||
		       (f->type == RBX_IPT_SERVICE1 && (s_in_sys & RBX_SYS_SERVICE));
	case RBX_IPT_SERVICE: return !!(s_in_sys & RBX_SYS_TEST);
	case RBX_IPT_TILT: return !!(s_in_sys & RBX_SYS_TILT);
	default: return 0;
	}
}

// PORT_CONDITION: a field counts while its port's DIP settings say so (the condition's port as the DIP switches set it;
// the fields sharing a bit with one of the other layout otherwise undid a press: qcrayon2's start)
static int field_active(const rbx_ipt_field *f)
{
	if (f->cond_port == 255 || f->cond_port >= s_nports)
		return 1;
	const rbx_ipt_port *cp = &s_game->ports[f->cond_port];
	uint32_t v = 0;
	for (int i = cp->first; i < cp->first + cp->count; i++) {
		const rbx_ipt_field *c = &s_game->fields[i];
		if (c->type == RBX_IPT_DIPSWITCH || c->type == RBX_IPT_CONFIG)
			v |= s_dip_val[i] & c->mask;
		else
			v |= c->def & c->mask;
	}
	const int eq = (v & f->cond_mask) == f->cond_value;
	return f->cond_type == 1 ? eq : f->cond_type == 2 ? !eq : 1;
}

static void update_ports(void)
{
	for (int p = 0; p < s_nports; p++) {
		const rbx_ipt_port *pt = &s_game->ports[p];
		uint32_t v = 0;
		uint32_t covered = 0;
		for (int i = pt->first; i < pt->first + pt->count; i++) {
			const rbx_ipt_field *f = &s_game->fields[i];
			if (!field_active(f))   // (PORT_CONDITION: a field of the other layout, as MAME leaves it out)
				continue;
			covered |= f->mask;
			uint32_t fv = f->def;
			if (f->type == RBX_IPT_DIPSWITCH || f->type == RBX_IPT_CONFIG) {
				fv = s_dip_val[i];
			} else if (f->type >= RBX_IPT_SERVICE && f->type <= RBX_IPT_BUTTON6) {
				int on = field_pressed(f);
				if (f->impulse) {
					if (on && !s_prev_on[i])
						s_impulse[i] = f->impulse;
					s_prev_on[i] = (uint8_t)on;
					on = s_impulse[i] > 0;
					if (s_impulse[i])
						s_impulse[i]--;
				}
				if (on)
					fv = f->def ^ f->mask;
			}
			v |= fv & f->mask;
		}
		// bits no field covers read as 1 (MAME default for unmapped bits is the port default; Taito ports are active low)
		s_port_val[p] = v | (~covered & 0xffff);
	}
}

// ---- DIP switches (fields with settings, excluding Unused / Unknown) ----
static int s_dips[MAX_FIELDS], s_ndips;

static void dips_init(void)
{
	s_ndips = 0;
	for (int p = 0; s_game->ports[p].tag; p++)
		for (int i = s_game->ports[p].first; i < s_game->ports[p].first + s_game->ports[p].count && i < MAX_FIELDS; i++) {
			const rbx_ipt_field *f = &s_game->fields[i];
			s_dip_val[i] = f->def;
			if ((f->type == RBX_IPT_DIPSWITCH || f->type == RBX_IPT_CONFIG) && f->set_count > 1 &&
			    strcmp(f->name, "Unused") && strcmp(f->name, "Unknown"))
				s_dips[s_ndips++] = i;
		}
}

int rbx_dip_count(void) { return s_game ? s_ndips : 0; }
const char *rbx_dip_name(int d) { return d >= 0 && d < s_ndips ? s_game->fields[s_dips[d]].name : NULL; }
int rbx_dip_options(int d) { return d >= 0 && d < s_ndips ? s_game->fields[s_dips[d]].set_count : 0; }
const char *rbx_dip_option_name(int d, int o)
{
	if (d < 0 || d >= s_ndips || o < 0 || o >= rbx_dip_options(d))
		return NULL;
	return s_game->settings[s_game->fields[s_dips[d]].set_first + o].name;
}
int rbx_dip_get(int d)
{
	if (d < 0 || d >= s_ndips)
		return -1;
	const rbx_ipt_field *f = &s_game->fields[s_dips[d]];
	for (int o = 0; o < f->set_count; o++)
		if (s_game->settings[f->set_first + o].value == s_dip_val[s_dips[d]])
			return o;
	return -1;
}
void rbx_dip_set(int d, int o)
{
	if (d < 0 || d >= s_ndips || o < 0 || o >= rbx_dip_options(d))
		return;
	const rbx_ipt_field *f = &s_game->fields[s_dips[d]];
	s_dip_val[s_dips[d]] = s_game->settings[f->set_first + o].value;
}
void rbx_dip_defaults(void)
{
	for (int d = 0; d < s_ndips; d++)
		s_dip_val[s_dips[d]] = s_game->fields[s_dips[d]].def;
}
int rbx_dip_raw(uint8_t *out, int max)
{
	int n = 0;
	for (int d = 0; d < s_ndips && n < max; d++)
		out[n++] = (uint8_t)(rbx_dip_get(d) < 0 ? 0xff : rbx_dip_get(d));
	return n;
}
void rbx_dip_set_raw(const uint8_t *in, int n)
{
	for (int d = 0; d < s_ndips && d < n; d++)
		if (in[d] != 0xff)
			rbx_dip_set(d, in[d]);
}

// ---- video ----
rbx_video_t rbx_video;
static uint32_t *s_lut;       // palette in frame buffer format; second half: the RGB888 entries it was made from
static rbx_color_fn s_lut_conv;   // conversion the table was made with (NULL = table invalid)
static int s_lut_bpp;

int rbx_video_setup(int width, int height, int palette_size)
{
	free(rbx_video.bitmap);
	free(rbx_video.palette);
	free(s_lut);
	// the conversion masks pens with palette_size - 1: a size that is not a power of two (Taito O 528, 40, 96 ...) is
	// made the next one up (the entries above the board's stay black). Before, pens past the mask folded onto low ones
	// (Taito O: every colour bank shown in bank 0's greys)
	int p2 = 1;
	while (p2 < palette_size)
		p2 <<= 1;
	palette_size = p2;
	rbx_video.bitmap = (uint16_t *)calloc((size_t)width * height, 2);
	rbx_video.palette = (uint32_t *)calloc(palette_size, 4);
	s_lut = (uint32_t *)calloc((size_t)palette_size * 2, 4);
	s_lut_conv = NULL;
	rbx_video.pitch = width;
	rbx_video.width = width;
	rbx_video.height = height;
	rbx_video.palette_size = palette_size;
	rbx_video.blank = 0;
	rbx_video.palette_used = 0;
	return rbx_video.bitmap && rbx_video.palette && s_lut ? 0 : -1;
}

static uint32_t default_color(int r, int g, int b)
{
	if (s_cfg.bytes_per_pixel == 2)
		return (uint32_t)((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3));
	return (uint32_t)(r << 16 | g << 8 | b);
}

static void video_out(void *fb, int pitch)
{
	rbx_color_fn conv = s_cfg.color ? s_cfg.color : default_color;
	int w = rbx_video.width, h = rbx_video.height;
	if (rbx_video.blank) {
		uint32_t c = conv(0, 0, 0);
		for (int y = 0; y < h; y++) {
			if (s_cfg.bytes_per_pixel == 2) {
				uint16_t *d = (uint16_t *)((uint8_t *)fb + y * pitch);
				for (int x = 0; x < w; x++) d[x] = (uint16_t)c;
			} else {
				uint32_t *d = (uint32_t *)((uint8_t *)fb + y * pitch);
				for (int x = 0; x < w; x++) d[x] = c;
			}
		}
		return;
	}
	// convert the palette entries that changed (large palettes of the multi-screen boards: 16384 entries), only
	// the ones the frame uses when the board says so (F3)
	int npal = rbx_video.palette_used > 0 && rbx_video.palette_used < rbx_video.palette_size ? rbx_video.palette_used : rbx_video.palette_size;
	uint32_t *src = s_lut + rbx_video.palette_size;
	int all = s_lut_conv != conv || s_lut_bpp != s_cfg.bytes_per_pixel;
	for (int i = 0; i < npal; i++) {
		uint32_t c = rbx_video.palette[i];
		if (!all && src[i] == c)
			continue;
		src[i] = c;
		s_lut[i] = conv((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff);
	}
	s_lut_conv = conv;
	s_lut_bpp = s_cfg.bytes_per_pixel;
	int mask = rbx_video.palette_size - 1;
	for (int y = 0; y < h; y++) {
		const uint16_t *s = rbx_video.bitmap + y * rbx_video.pitch;
		if (s_cfg.bytes_per_pixel == 2) {
			uint16_t *d = (uint16_t *)((uint8_t *)fb + y * pitch);
			for (int x = 0; x < w; x++) d[x] = (uint16_t)s_lut[s[x] & mask];
		} else {
			uint32_t *d = (uint32_t *)((uint8_t *)fb + y * pitch);
			for (int x = 0; x < w; x++) d[x] = s_lut[s[x] & mask];
		}
	}
}

#ifdef RBX_OUT_PROF   // -DRBX_OUT_PROF: the driver's render and video_out, ms a frame drawn, printed at exit
static double s_vo_ms, s_dr_ms;
static int s_dr_n;
static double prof_ms(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e3 + t.tv_nsec / 1e6; }
static void vo_print(void)
{
	const int n = s_dr_n ? s_dr_n : 1;
	fprintf(stderr, "OUTPROF %d frames render %.3f video_out %.3f total %.3f ms\n", s_dr_n, s_dr_ms / n, s_vo_ms / n, (s_dr_ms + s_vo_ms) / n);
}
static void video_out_timed(void *fb, int pitch)
{
	const double a = prof_ms();
	video_out(fb, pitch);
	s_vo_ms += prof_ms() - a;
}
#define video_out video_out_timed
static void drv_render(void)
{
	const double a = prof_ms();
	s_drv->render();
	s_dr_ms += prof_ms() - a;
	if (!s_dr_n++)
		atexit(vo_print);
}
#else
#define drv_render() s_drv->render()
#endif

void rbx_screen(int *w, int *h)
{
	if (w) *w = rbx_video.width;
	if (h) *h = rbx_video.height;
}
double rbx_fps(void) { return 60.0; }

// the driver's render with the frontend's frame on offer (rbx_video.out), then the conversion if it did not write it
static void out_offer(void *fb, int pitch)
{
	rbx_video.out = fb;
	rbx_video.out_pitch = pitch;
	rbx_video.out_bpp = s_cfg.bytes_per_pixel;
	rbx_video.out_color = s_cfg.color ? s_cfg.color : default_color;
	rbx_video.out_done = 0;
}

static void render_with_out(void *fb, int pitch)
{
	out_offer(fb, pitch);
	if (s_drv && s_drv->render)
		drv_render();
	rbx_video.out = NULL;
	if (fb && !rbx_video.out_done)
		video_out(fb, pitch);
}

// ---- render thread ----
static volatile int s_rt_on, s_rt_stop;
static volatile unsigned s_rt_posted, s_rt_done;   // frames captured / drawn

static volatile int s_wait_site;   // (the emulation thread's mostly: profiling only)
int rbx_wait_site(void) { return s_wait_site; }

static void wait_site(int site)   // (the emulation thread only)
{
	s_wait_site = site;
	const unsigned pa = rbx_prof_enter(RBX_PA_WAIT);
	if (s_cfg.wait_hook)
		s_cfg.wait_hook();
	rbx_prof_leave(pa);
	s_wait_site = 0;
}

static void wait_some(void)   // (the workers' waits: the site left as it is)
{
	if (s_cfg.wait_hook)
		s_cfg.wait_hook();
}

void rbx_video_wait_idle(void)
{
	if (!s_rt_on)
		return;
	while (__atomic_load_n(&s_rt_done, __ATOMIC_ACQUIRE) != __atomic_load_n(&s_rt_posted, __ATOMIC_ACQUIRE) &&
	       !__atomic_load_n(&s_rt_stop, __ATOMIC_ACQUIRE))
		if (!(s_drv && s_drv->render_help && s_drv->render_help(1)))   // (help the render thread instead of waiting)
			wait_site(1);
}

// A driver with two frame snapshots (rbx_video_set_depth(2)) lets the emulation run one frame ahead of the render
// thread: the snapshot of frame N + 1 is taken while frame N is still being drawn. Its vblank waits for a free
// snapshot (rbx_video_wait_slot), takes it into slot rbx_video_post_seq() & 1 and posts it; render draws slot
// rbx_video_render_seq() & 1. Both counts start at 0 together (rbx_set_render_thread), and without the render thread
// both stay 0 (the frame is drawn in rbx_video_frame_done): the frames drawn are the same.
static int s_rt_depth = 1;
void rbx_video_set_depth(int depth) { s_rt_depth = depth >= 2 ? 2 : 1; }
int rbx_video_depth(void) { return s_rt_depth; }
int rbx_video_threaded(void) { return s_rt_on; }   // (the frames drawn by the render thread: set only while idle)
unsigned rbx_video_post_seq(void) { return s_rt_on ? __atomic_load_n(&s_rt_posted, __ATOMIC_ACQUIRE) : 0; }
unsigned rbx_video_render_seq(void) { return s_rt_on ? __atomic_load_n(&s_rt_done, __ATOMIC_ACQUIRE) : 0; }

void rbx_video_wait_slot(void)
{
	if (!s_rt_on)
		return;
	while (__atomic_load_n(&s_rt_posted, __ATOMIC_ACQUIRE) - __atomic_load_n(&s_rt_done, __ATOMIC_ACQUIRE) >= (unsigned)s_rt_depth &&
	       !__atomic_load_n(&s_rt_stop, __ATOMIC_ACQUIRE))
		if (!(s_drv && s_drv->render_help && s_drv->render_help(1)))   // (help the render thread instead of waiting)
			wait_site(1);
}

void rbx_video_frame_done(void)
{
	if (s_rt_on)
		__atomic_add_fetch(&s_rt_posted, 1, __ATOMIC_RELEASE);
	else if (s_drv && s_drv->render) {   // (here on the emulation thread)
		const unsigned pa = rbx_prof_enter(RBX_PA_VIDEO_OUT);
		drv_render();
		rbx_prof_leave(pa);
	}
}

void rbx_set_render_thread(int on)
{
	if (on == s_rt_on)
		return;
	rbx_video_wait_idle();
	s_rt_posted = s_rt_done = 0;
	s_rt_stop = 0;
	s_rt_on = on;
}

// a frontend thread on a spare core draws part of the frame being drawn (0 = nothing to do now)
static volatile int s_aux_idle_help;   // (rbx_aux_set_idle_help: then no frontend helper thread)
int rbx_render_can_help(void) { return s_drv && s_drv->render_help && !s_aux_idle_help; }

int rbx_render_help_one(int helper)
{
	return s_rt_on && s_drv && s_drv->render_help ? s_drv->render_help(helper | 0x100) : 0;
}

int rbx_render_help(void)
{
	return s_rt_on && s_drv && s_drv->render_help ? s_drv->render_help(2) : 0;
}

int rbx_render_frame(void *fb, int pitch)
{
	for (;;) {
		if (__atomic_load_n(&s_rt_stop, __ATOMIC_ACQUIRE))
			return 0;
		if (__atomic_load_n(&s_rt_posted, __ATOMIC_ACQUIRE) != __atomic_load_n(&s_rt_done, __ATOMIC_ACQUIRE))
			break;
		wait_some();
	}
	render_with_out(fb, pitch);
	__atomic_add_fetch(&s_rt_done, 1, __ATOMIC_RELEASE);
	return 1;
}

void rbx_render_stop(void)
{
	__atomic_store_n(&s_rt_stop, 1, __ATOMIC_RELEASE);
}

// ---- worker threads (a driver's deferred jobs: rbx_aux_* on the first, rbx_aux2_* on the second, rbx_aux3_*) ----
typedef struct {
	void (*volatile job)(void);
	volatile int on, stop;
	volatile unsigned posted, done;   // requests / requests served
	volatile int busy;                // the job running (on the worker, or on a waiting thread that took it: aux_sync)
	volatile int spin_us;             // after a job, look for the next one this long before sleeping (rbx_aux_set_spin)
} aux_worker;
static aux_worker s_aux[3];
uint64_t rbx_aux_sync_us[3];   // (profiling: the time waited in each worker's sync, spins and sleeps)

// the job once, by whoever holds busy: the requests seen before it ran are then served (0: someone else is running it)
static int aux_claim_run(aux_worker *w)
{
	int free = 0;
	if (!__atomic_compare_exchange_n(&w->busy, &free, 1, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return 0;
	const unsigned posted = __atomic_load_n(&w->posted, __ATOMIC_ACQUIRE);
	void (*job)(void) = w->job;
	if (job)
		job();
	__atomic_store_n(&w->done, posted, __ATOMIC_RELEASE);
	__atomic_store_n(&w->busy, 0, __ATOMIC_RELEASE);
	return 1;
}

static void aux_kick(aux_worker *w)
{
	if (w->on)
		__atomic_add_fetch(&w->posted, 1, __ATOMIC_RELEASE);
	else if (w->job)
		w->job();   // (no worker: the work is done here)
}

static void aux_sync(aux_worker *w)
{
	void (*job)(void) = w->job;
	if (!job)
		return;
	if (!w->on) {
		job();
		return;
	}
	const unsigned want = __atomic_add_fetch(&w->posted, 1, __ATOMIC_RELEASE);
	if ((int)(__atomic_load_n(&w->done, __ATOMIC_ACQUIRE) - want) >= 0)
		return;
	const uint64_t t0 = rbx_clock_us();
	for (int spin = 0; (int)(__atomic_load_n(&w->done, __ATOMIC_ACQUIRE) - want) < 0; spin++) {
		if (__atomic_load_n(&w->stop, __ATOMIC_ACQUIRE)) {   // (the worker is gone: do the work here)
			job();
			return;
		}
		// the worker not on it (asleep, or not woken yet): the job here at once. A thread waiting on the ESP for each
		// of its register reads lost ~0.8 ms a read to the worker's waking (kirameki: 25 reads a frame, 30 ms frames)
		if (aux_claim_run(w))
			continue;
		if (spin >= 4000)   // (a short wait spins: a sleep is ~100 us on the Vita)
			wait_site(2 + (int)(w - s_aux));
	}
	rbx_aux_sync_us[w - s_aux] += rbx_clock_us() - t0;
}

static void aux_set_thread(aux_worker *w, int on)
{
	if (on == w->on)
		return;
	aux_sync(w);
	w->posted = w->done = 0;
	w->stop = 0;
	w->on = on;
}

// the first worker, idle, draws bands of the frame (render_help(helper | 0x100)): konamigx, whose emulation thread is
// busy and whose CPU3 runs the DASP
void rbx_aux_set_idle_help(int helper) { __atomic_store_n(&s_aux_idle_help, helper, __ATOMIC_RELEASE); }

static int aux_run(aux_worker *w)
{
	for (;;) {
		if (__atomic_load_n(&w->stop, __ATOMIC_ACQUIRE))
			return 0;
		if (__atomic_load_n(&w->posted, __ATOMIC_ACQUIRE) != __atomic_load_n(&w->done, __ATOMIC_ACQUIRE))
			break;
		const int help = __atomic_load_n(&s_aux_idle_help, __ATOMIC_ACQUIRE);
		if (help && w == &s_aux[0] && s_rt_on && s_drv && s_drv->render_help && s_drv->render_help(help | 0x100))
			continue;
		if (w->spin_us) {   // (jobs a scanline apart: a sleep would make the next one wait ~100 us)
			const uint64_t t0 = rbx_clock_us();
			int got = 0;
			for (int k = 0; !got; k++) {
				got = __atomic_load_n(&w->posted, __ATOMIC_ACQUIRE) != __atomic_load_n(&w->done, __ATOMIC_ACQUIRE) ||
				      __atomic_load_n(&w->stop, __ATOMIC_ACQUIRE);
				if ((k & 63) == 63 && rbx_clock_us() - t0 > (uint64_t)w->spin_us)
					break;
			}
			if (got)
				continue;
		}
		wait_some();
	}
	if (!aux_claim_run(w) && !w->spin_us)   // (a waiting thread has it: look again; spinning, at once)
		wait_some();
	return 1;
}

void rbx_aux_kick(void) { aux_kick(&s_aux[0]); }
void rbx_aux_sync(void) { aux_sync(&s_aux[0]); }
int rbx_aux_has_job(void) { return s_aux[0].job != NULL; }
static int s_aux_by_render;
void rbx_aux_set_job(void (*job)(void)) { aux_sync(&s_aux[0]); s_aux[0].job = job; if (!job) { s_aux_by_render = 0; s_aux[0].spin_us = 0; } }
void rbx_aux_set_by_render(int on) { s_aux_by_render = on; }
int rbx_aux_by_render(void) { return s_aux_by_render; }
void rbx_aux_set_spin(int us) { s_aux[0].spin_us = us; }
void rbx_set_aux_thread(int on) { aux_set_thread(&s_aux[0], on); }
int rbx_aux_run(void) { return aux_run(&s_aux[0]); }
void rbx_aux_stop(void) { __atomic_store_n(&s_aux[0].stop, 1, __ATOMIC_RELEASE); }

void rbx_aux2_kick(void) { aux_kick(&s_aux[1]); }
void rbx_aux2_sync(void) { aux_sync(&s_aux[1]); }
int rbx_aux2_has_job(void) { return s_aux[1].job != NULL; }
static int s_aux2_alone;
void rbx_aux2_set_job(void (*job)(void)) { aux_sync(&s_aux[1]); s_aux[1].job = job; if (!job) { s_aux2_alone = 0; s_aux[1].spin_us = 0; } }
void rbx_aux2_set_spin(int us) { s_aux[1].spin_us = us; }
void rbx_aux2_set_alone(int on) { s_aux2_alone = on; }
int rbx_aux2_alone(void) { return s_aux2_alone; }
void rbx_set_aux2_thread(int on) { aux_set_thread(&s_aux[1], on); }
int rbx_aux2_run(void) { return aux_run(&s_aux[1]); }
void rbx_aux2_stop(void) { __atomic_store_n(&s_aux[1].stop, 1, __ATOMIC_RELEASE); }

static void (*volatile s_aux3_wake)(void);
void rbx_aux3_kick(void)
{
	aux_kick(&s_aux[2]);
	void (*wake)(void) = s_aux3_wake;
	if (s_aux[2].on && wake)
		wake();
}
void rbx_set_aux3_wake(void (*wake)(void)) { s_aux3_wake = wake; }
void rbx_aux3_nudge(void)
{
	if (s_aux[2].on)
		rbx_aux3_kick();
}
int rbx_aux3_try(void)
{
	aux_worker *w = &s_aux[2];
	if (__atomic_load_n(&w->posted, __ATOMIC_ACQUIRE) == __atomic_load_n(&w->done, __ATOMIC_ACQUIRE))
		return 0;
	return aux_claim_run(w);
}
void rbx_aux3_sync(void) { aux_sync(&s_aux[2]); }
int rbx_aux3_has_job(void) { return s_aux[2].job != NULL; }
static int s_aux3_alone;
void rbx_aux3_set_job(void (*job)(void)) { aux_sync(&s_aux[2]); s_aux[2].job = job; if (!job) s_aux3_alone = 0; }
void rbx_aux3_set_alone(int on) { s_aux3_alone = on; }
int rbx_aux3_alone(void) { return s_aux3_alone; }
void rbx_aux_wait_for(int w) { wait_site(2 + (w & 3)); }
void rbx_worker_wait(void) { wait_some(); }
void rbx_set_aux3_thread(int on) { aux_set_thread(&s_aux[2], on); }
int rbx_aux3_run(void) { return aux_run(&s_aux[2]); }
void rbx_aux3_stop(void) { __atomic_store_n(&s_aux[2].stop, 1, __ATOMIC_RELEASE); }
void rbx_wait(void) { wait_site(5); }

// ---- audio ----
rbx_audio_t rbx_audio;
static uint32_t s_rs_pos;      // 16.16 position into the current frame's input
static uint32_t s_auto_frames;          // (frames and samples the driver gave since rbx_audio_setup: its frame rate when it
static uint64_t s_auto_samples;         // sets none, see audio_out)
static int16_t s_rs_prev, s_rs_prev_r;  // last input samples of the previous frame

int rbx_audio_setup(uint32_t rate)
{
	free(rbx_audio.buf);
	free(rbx_audio.buf_r);
	rbx_audio.buf_r = NULL;
	rbx_audio.stereo = 0;
	rbx_audio.rate = rate;
	// (room for several frames: a board whose sound comes from a worker a frame or two late, F3's ESP, puts out up to
	// three frames' samples in one; a cap of 1.3 frames dropped the rest, the sound short by up to 0.4%: underruns)
	rbx_audio.cap = (int)(rate / 10) + 64;
	rbx_audio.buf = (int16_t *)calloc(rbx_audio.cap, 2);
	rbx_audio.count = 0;
	s_rs_pos = 0;
	s_rs_prev = 0;
	s_auto_frames = 0;
	s_auto_samples = 0;
	return rbx_audio.buf ? 0 : -1;
}

int rbx_audio_setup_stereo(uint32_t rate)
{
	if (rbx_audio_setup(rate))
		return -1;
	rbx_audio.buf_r = (int16_t *)calloc(rbx_audio.cap, 2);
	rbx_audio.stereo = 1;
	s_rs_prev_r = 0;
	return rbx_audio.buf_r ? 0 : -1;
}

// (the output volume is the frontend's: the Vita frontend doubles it twice with a soft limit, ADR-009)
static inline int16_t out_gain(int v)
{
	return (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
}

static int audio_out(int16_t *out, int max)
{
	if (!rbx_audio.rate || !s_cfg.sample_rate) {
		rbx_audio.count = 0;
		return 0;
	}
	// linear interpolation; position 0 is the previous frame's last sample
	// a frame of the board (1 / its frame rate) is played in a 60 Hz frame, as its picture: otherwise a board slower
	// than 60 Hz (F3 58.94 Hz) makes more sound than the output takes and the frontend's buffer fills up
	// A board that does not say its frame rate (rbx_audio.frame_mhz): the rate its own samples show, so a board of
	// 59.18 Hz makes no more sound than the output takes (338 sets, Konami's and Taito's: the buffer overflowed, samples
	// dropped) and one of 60.61 Hz no less (underruns). Within 0.3 % of 60 Hz it is 60 Hz.
	uint32_t fmhz = rbx_audio.frame_mhz;
	s_auto_frames++;
	s_auto_samples += (uint32_t)(rbx_audio.count > 0 ? rbx_audio.count : 0);
	if (!fmhz) {
		fmhz = 60000;
		if (s_auto_frames >= 60 && s_auto_samples) {
			const uint64_t est = (uint64_t)rbx_audio.rate * 1000u * s_auto_frames / s_auto_samples;
			if (est >= 40000 && est <= 80000 && (est < 59820 || est > 60180))
				fmhz = (uint32_t)est;
		}
	}
	uint32_t step = (uint32_t)(((uint64_t)rbx_audio.rate << 16) * 60000 / ((uint64_t)s_cfg.sample_rate * fmhz));
	int n = 0;
	int cnt = rbx_audio.count;
	while (n < max) {
		uint32_t i = s_rs_pos >> 16;
		if ((int)i >= cnt)
			break;
		int a = i ? rbx_audio.buf[i - 1] : s_rs_prev;
		int b = rbx_audio.buf[i];
		int f = (int)(s_rs_pos & 0xffff);
		int v = a + (((b - a) * f) >> 16);
		out[2 * n] = out[2 * n + 1] = out_gain(v);
		if (rbx_audio.stereo) {
			int ar = i ? rbx_audio.buf_r[i - 1] : s_rs_prev_r;
			int br = rbx_audio.buf_r[i];
			out[2 * n + 1] = out_gain(ar + (((br - ar) * f) >> 16));
		}
		n++;
		s_rs_pos += step;
	}
	if (cnt) {
		s_rs_prev = rbx_audio.buf[cnt - 1];
		if (rbx_audio.stereo)
			s_rs_prev_r = rbx_audio.buf_r[cnt - 1];
		s_rs_pos -= (uint32_t)cnt << 16;
		if ((int32_t)s_rs_pos < 0)
			s_rs_pos = 0;
	}
	rbx_audio.count = 0;
	return n;
}

// ---- init / exit / frame ----
// (ADR-200) the driver's largest buffers, made before the ROM regions (rbx_driver.early)
static void *s_early[RBX_EARLY_MAX];
static size_t s_early_n[RBX_EARLY_MAX];
void *rbx_early(int i, size_t *size)
{
	if (i < 0 || i >= RBX_EARLY_MAX)
		return NULL;
	if (size)
		*size = s_early_n[i];
	return s_early[i];
}
static void early_free(void)
{
	for (int i = 0; i < RBX_EARLY_MAX; i++) {
		free(s_early[i]);
		s_early[i] = NULL;
		s_early_n[i] = 0;
	}
}
static int early_make(const rbx_game *g, const rbx_driver *drv)
{
	if (!drv->early || getenv("RBX_EARLY_OFF"))   // (RBX_EARLY_OFF=1: the driver makes them itself, measurements)
		return 0;
	size_t sz[RBX_EARLY_MAX] = { 0 };
	int n = drv->early(g, sz);
	if (n > RBX_EARLY_MAX)
		n = RBX_EARLY_MAX;
	for (int i = 0; i < n; i++) {
		if (!sz[i])
			continue;
		if (!(s_early[i] = calloc(1, sz[i]))) {
			rbx_fail("%s: out of memory (a block of %u KB)", g->set, (unsigned)(sz[i] >> 10));
			early_free();
			return -1;
		}
		s_early_n[i] = sz[i];
	}
	return 0;
}

static void release(void)
{
	free_regions();
	early_free();
	free(rbx_video.bitmap);
	free(rbx_video.palette);
	free(s_lut);
	s_lut = NULL;
	s_lut_conv = NULL;
	memset(&rbx_video, 0, sizeof(rbx_video));
	free(rbx_audio.buf);
	free(rbx_audio.buf_r);
	memset(&rbx_audio, 0, sizeof(rbx_audio));
}

// ---- every game from a fresh core (user 11-23: "always as if just started") ----
// The link (tools/rbx_state.ld) gathers the core library's writable static data (.data, .bss) into two ranges. The
// first rbx_init copies them; every later rbx_init puts that copy back first, so no static state of the last game (a
// driver's, a chip's, a CPU core's) is left for the next: what a driver's init or exit forgets cannot leak into the
// next game. (Heap memory a driver did not free stays lost; the frontend logs the heap after each game.)
// Without the link script the symbols are missing (weak: NULL) and nothing is copied.
extern char __rbx_data_start[] __attribute__((weak)), __rbx_data_end[] __attribute__((weak));
extern char __rbx_bss_start[] __attribute__((weak)), __rbx_bss_end[] __attribute__((weak));
// The copy: .data whole, then of .bss (mostly zero) only its chunks that are not zero, each as {offset, bytes}
#define FRESH_CHUNK 1024
static unsigned char *s_fresh;   // (itself in the copied range: the copy holds this same pointer)
static size_t s_fresh_size;

static int chunk_zero(const unsigned char *p, size_t n)
{
	for (size_t i = 0; i < n; i++)
		if (p[i])
			return 0;
	return 1;
}

static void fresh_core(void)
{
	if (!__rbx_data_start || !__rbx_bss_start)
		return;
	const size_t nd = (size_t)(__rbx_data_end - __rbx_data_start), nb = (size_t)(__rbx_bss_end - __rbx_bss_start);
	if (!s_fresh) {   // (the first game: the state the app started with)
		size_t nz = 0;
		for (size_t o = 0; o < nb; o += FRESH_CHUNK)
			nz += !chunk_zero((const unsigned char *)__rbx_bss_start + o, nb - o < FRESH_CHUNK ? nb - o : FRESH_CHUNK);
		const size_t size = nd + nz * (sizeof(uint32_t) + FRESH_CHUNK) + sizeof(uint32_t);
		unsigned char *b = (unsigned char *)malloc(size);
		if (!b)
			return;
		memcpy(b, __rbx_data_start, nd);
		unsigned char *w = b + nd;
		for (size_t o = 0; o < nb; o += FRESH_CHUNK) {
			const size_t n = nb - o < FRESH_CHUNK ? nb - o : FRESH_CHUNK;
			if (chunk_zero((const unsigned char *)__rbx_bss_start + o, n))
				continue;
			const uint32_t off = (uint32_t)o;
			memcpy(w, &off, sizeof(off));
			memcpy(w + sizeof(off), __rbx_bss_start + o, n);
			w += sizeof(off) + FRESH_CHUNK;
		}
		const uint32_t end = 0xffffffffu;
		memcpy(w, &end, sizeof(end));
		s_fresh = b;   // (after the copy: set before, their chunk was counted zero and then copied)
		s_fresh_size = size;
		return;
	}
	unsigned char *const b = s_fresh;
	const size_t size = s_fresh_size;
	memcpy(__rbx_data_start, b, nd);
	memset(__rbx_bss_start, 0, nb);
	for (const unsigned char *r = b + nd;; r += sizeof(uint32_t) + FRESH_CHUNK) {
		uint32_t off;
		memcpy(&off, r, sizeof(off));
		if (off == 0xffffffffu)
			break;
		memcpy(__rbx_bss_start + off, r + sizeof(off), nb - off < FRESH_CHUNK ? nb - off : FRESH_CHUNK);
	}
	// (the compiler takes the copies for writes to other objects than s_fresh: without this it kept s_fresh in a
	// register and dropped the stores below, s_fresh was 0 after the second game)
	__asm__ __volatile__("" ::: "memory");
	s_fresh = b;   // (themselves in the ranges: their values at the copy were not yet set)
	s_fresh_size = size;
}

// the copy's bytes, the ranges' bytes
size_t rbx_fresh_size(void) { return s_fresh_size; }
size_t rbx_fresh_range(void)
{
	return __rbx_data_start && __rbx_bss_start ? (size_t)(__rbx_data_end - __rbx_data_start) + (size_t)(__rbx_bss_end - __rbx_bss_start) : 0;
}

int rbx_init(const rbx_config *cfg)
{
	fresh_core();
	s_cfg = *cfg;
	for (unsigned i = RBX_PA_CPU; i < RBX_PROF_IDS; i++)   // (the last game's CPU and driver names gone)
		s_prof_names[i] = NULL;
	if (s_cfg.bytes_per_pixel != 2)
		s_cfg.bytes_per_pixel = 4;
	s_error[0] = 0;
	int idx = rbx_game_find(cfg->set);
	if (idx < 0) {
		rbx_fail("unknown set %s", cfg->set);
		return -1;
	}
	const rbx_driver *drv;
	const rbx_game *g = game_at(idx, &drv);
	s_game = g;
	const uint64_t tl0 = rbx_clock_us();
	if (early_make(g, drv) || load_roms(g, drv)) {
		release();
		s_game = NULL;
		return -1;
	}
	s_nports = 0;
	while (g->ports[s_nports].tag && s_nports < MAX_PORTS)
		s_nports++;
	memset(s_custom_mask, 0, sizeof(s_custom_mask));
	memset(s_custom_inv, 0, sizeof(s_custom_inv));
	for (int p = 0; p < s_nports; p++)
		for (int i = g->ports[p].first; i < g->ports[p].first + g->ports[p].count; i++)
			if (g->fields[i].type == RBX_IPT_CUSTOM)
				s_custom_inv[p] |= g->fields[i].def & g->fields[i].mask;
	memset(s_impulse, 0, sizeof(s_impulse));
	memset(s_prev_on, 0, sizeof(s_prev_on));
	memset(s_in_player, 0, sizeof(s_in_player));
	s_in_sys = 0;
	dips_init();
	update_ports();
	const uint64_t tl1 = rbx_clock_us();
	s_rt_depth = 1;
	if (!drv->init) {   // (a board without its init (scumimon, bkungfu): an error, not a crash)
		rbx_fail("%s: board not emulated", g->set);
		release();
		s_game = NULL;
		return -1;
	}
	const int drv_rc = drv->init(g);
	rbx_log("load: roms %u ms, driver init %u ms", (unsigned)((tl1 - tl0) / 1000), (unsigned)((rbx_clock_us() - tl1) / 1000));
	if (drv_rc) {
		if (!s_error[0])
			rbx_fail("%s: board init failed", g->set);
		release();
		s_game = NULL;
		return -1;
	}
	s_drv = drv;
	s_game_index = idx;
	if (s_cfg.progress)
		s_cfg.progress(1000, s_cfg.progress_user);
	return 0;
}

void rbx_exit(void)
{
	rbx_render_stop();
	s_rt_on = 0;
	if (s_drv)
		s_drv->exit();
	s_drv = NULL;
	release();
	s_game = NULL;
	s_game_index = -1;
}

int rbx_running_game(void) { return s_game_index; }

void rbx_reset(void)
{
	rbx_video_wait_idle();
	if (s_drv)
		s_drv->reset();
}

int rbx_run_frame(void *fb, int pitch, int16_t *audio, int max_samples)
{
	if (!s_drv)
		return 0;
	const unsigned pa = rbx_prof_enter(RBX_PA_INPUT);   // (the frontend's mark put back at the end)
	tms57002_jit_install();   // (a DSP program compiled by the sound thread into place, see tms57002_jit.inc)
	update_ports();
	const int direct = !s_rt_on;   // (without the render thread the frame is drawn within run_frame: the fb on offer)
	if (direct)
		out_offer(fb, pitch);
	rbx_prof_set(RBX_PA_DRIVER);
	s_drv->run_frame();
	if (direct) {
		rbx_prof_set(RBX_PA_VIDEO_OUT);
		rbx_video.out = NULL;
		if (fb && !rbx_video.out_done)
			video_out(fb, pitch);
	}
	int n = 0;
	rbx_prof_set(RBX_PA_AUDIO_OUT);
	if (audio)
		n = audio_out(audio, max_samples);
	else
		rbx_audio.count = 0;
	rbx_prof_leave(pa);
	return n;
}

// ---- save states ----
typedef struct { uint8_t *buf; uint32_t size, pos; int load, err, check; } state_ctx;   // check: headers compared only

int rbx_state_io(void *vctx, const char *tag, void *data, uint32_t size)
{
	state_ctx *c = (state_ctx *)vctx;
	uint8_t hdr[12] = { 0 };
	strncpy((char *)hdr, tag, 8);
	hdr[8] = (uint8_t)size; hdr[9] = (uint8_t)(size >> 8); hdr[10] = (uint8_t)(size >> 16); hdr[11] = (uint8_t)(size >> 24);
	if (!c->buf) {           // size pass
		c->pos += 12 + size;
		return 0;
	}
	if (c->pos + 12 + size > c->size) {
		c->err = 1;
		return -1;
	}
	if (c->load || c->check) {
		if (memcmp(c->buf + c->pos, hdr, 12)) {
			rbx_log("state: block %.8s size %u does not match the file's %.8s %u", tag, size, (const char *)c->buf + c->pos,
			        (uint32_t)c->buf[c->pos + 8] | (uint32_t)c->buf[c->pos + 9] << 8 | (uint32_t)c->buf[c->pos + 10] << 16 |
			            (uint32_t)c->buf[c->pos + 11] << 24);
			c->err = 1;
			return -1;
		}
		if (c->load)
			memcpy(data, c->buf + c->pos + 12, size);
	} else {
		memcpy(c->buf + c->pos, hdr, 12);
		memcpy(c->buf + c->pos + 12, data, size);
	}
	c->pos += 12 + size;
	return 0;
}

int rbx_state_checking(void *vctx) { return ((state_ctx *)vctx)->check; }

void *rbx_state_var(void *vctx, const char *tag, uint32_t *size, int load)
{
	state_ctx *c = (state_ctx *)vctx;
	uint8_t hdr[12] = { 0 };
	strncpy((char *)hdr, tag, 8);
	if (!load && !c->buf) {   // size pass
		c->pos += 12 + *size;
		return NULL;
	}
	if (c->pos + 12 > c->size) {
		c->err = 1;
		return NULL;
	}
	uint8_t *p = c->buf + c->pos;
	if (load || c->check) {
		const uint32_t n = (uint32_t)p[8] | (uint32_t)p[9] << 8 | (uint32_t)p[10] << 16 | (uint32_t)p[11] << 24;
		if (memcmp(p, hdr, 8) || n > c->size - c->pos - 12) {
			rbx_log("state: block %.8s does not match the file's %.8s", tag, (const char *)p);
			c->err = 1;
			return NULL;
		}
		c->pos += 12 + n;
		if (!load)
			return NULL;
		*size = n;
		return p + 12;
	}
	if (*size > c->size - c->pos - 12) {
		c->err = 1;
		return NULL;
	}
	memcpy(p, hdr, 8);
	p[8] = (uint8_t)*size; p[9] = (uint8_t)(*size >> 8); p[10] = (uint8_t)(*size >> 16); p[11] = (uint8_t)(*size >> 24);
	c->pos += 12 + *size;
	return p + 12;
}

static int state_run(state_ctx *c)
{
	uint32_t magic[2] = { 0x58425252, 1 };  // "RRBX", version
	char set[16] = { 0 };
	strncpy(set, s_game->set, 15);
	rbx_state_io(c, "rbxstate", magic, sizeof(magic));
	rbx_state_io(c, "set", set, sizeof(set));
	if ((c->load || c->check) && (magic[0] != 0x58425252 || magic[1] != 1 || strcmp(set, s_game->set)))
		return -1;
	if (c->load ? s_drv->state_load(c) : s_drv->state_save(c))   // (check: the save walk, nothing written)
		return -1;
	return c->err ? -1 : 0;
}

uint32_t rbx_state_size(void)
{
	rbx_video_wait_idle();
	if (!s_drv || !s_drv->state_save)
		return 0;
	state_ctx c = { NULL, 0, 0, 0, 0, 0 };
	state_run(&c);
	return c.pos;
}

int rbx_state_write(void *buf, uint32_t size)
{
	rbx_video_wait_idle();
	if (!s_drv || !s_drv->state_save)
		return -1;
	state_ctx c = { (uint8_t *)buf, size, 0, 0, 0, 0 };
	return state_run(&c);
}

int rbx_state_read(const void *buf, uint32_t size)
{
	rbx_video_wait_idle();
	if (!s_drv || !s_drv->state_load)
		return -1;
	// (a state of another size is from an older build: refused whole. Loaded part way, it left the machine broken and
	// the game hung, e.g. an F3 state from before the sound board format change)
	const uint64_t t0 = rbx_clock_us();
	// every part's name and size checked first, nothing loaded (it logs the first block that differs): a state of an
	// older layout (a part changed but the same total size) loaded part way and then refused left the machine broken.
	// The parts must fill the state exactly (rbx_state_var blocks: their sizes as stored)
	{
		state_ctx k = { (uint8_t *)buf, size, 0, 0, 0, 1 };
		if (state_run(&k) || k.pos != size) {
			rbx_log("state: size %u, this build's layout %u", size, k.pos);
			return -1;
		}
	}
	const uint64_t t1 = rbx_clock_us();
	rbx_state_load_us[2] = (uint32_t)(t1 - t0);
	state_ctx c = { (uint8_t *)buf, size, 0, 1, 0, 0 };
	const int rc = state_run(&c);
	rbx_state_load_us[3] = (uint32_t)(rbx_clock_us() - t1);
	return rc;
}

int rbx_state_save(const char *path)
{
	uint32_t n = rbx_state_size();
	if (!n)
		return -1;
	uint8_t *buf = (uint8_t *)malloc(n);
	int rc = -1;
	if (buf && !rbx_state_write(buf, n)) {
		FILE *fp = fopen(path, "wb");
		if (fp) {
			rc = fwrite(buf, 1, n, fp) == n ? 0 : -1;
			fclose(fp);
		}
	}
	free(buf);
	return rc;
}

uint32_t rbx_state_load_us[4];   // the last load's parts: open, read, size + check walks, the load walk

int rbx_state_load(const char *path)
{
	const uint64_t t0 = rbx_clock_us();
	FILE *fp = fopen(path, "rb");
	rbx_state_load_us[0] = (uint32_t)(rbx_clock_us() - t0);
	if (!fp)
		return -1;
	fseek(fp, 0, SEEK_END);
	long n = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	uint8_t *buf = (uint8_t *)malloc(n > 0 ? n : 1);
	int rc = -1;
	const uint64_t t1 = rbx_clock_us();
	if (buf && fread(buf, 1, n, fp) == (size_t)n) {
		rbx_state_load_us[1] = (uint32_t)(rbx_clock_us() - t1);
		rc = rbx_state_read(buf, (uint32_t)n);
	}
	fclose(fp);
	free(buf);
	return rc;
}

// ---- cheats: provided by the driver through the main CPU space ----
uint8_t rbx_board_peek(uint32_t addr);
void rbx_board_poke(uint32_t addr, uint8_t v);
uint8_t rbx_peek(uint32_t addr) { return s_drv ? rbx_board_peek(addr) : 0; }
void rbx_poke(uint32_t addr, uint8_t v) { if (s_drv) rbx_board_poke(addr, v); }
