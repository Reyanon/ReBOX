/*  Texas Instruments TMS57002 "DASP", ported to C from MAME src/devices/cpu/tms57002/tms57002.cpp, tms57kdec.cpp
    and tmsops.cpp (MAME 0.289, BSD-3-Clause, copyright-holders: Olivier Galibert). The instruction decoders and
    interpreters are MAME's generated tms57002.hxx, included here with macros mapping the device's members onto the
    C state (s->...). See tms57002.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tms57002.h"
#include "../../rbx.h"

#include <stddef.h>
#include <string.h>
#if !defined(__vita__) && defined(__linux__)
#include <time.h>
#endif

typedef tms57002_xop icd;   // (the generated interpreters take the threaded form: they only read i->param)

#if defined(__GNUC__)
#define TMS_INLINE inline __attribute__((always_inline))
#define TMS_NOINLINE __attribute__((noinline))
#define TMS_LIKELY(x) __builtin_expect(!!(x), 1)
#define TMS_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define TMS_INLINE inline
#define TMS_NOINLINE
#define TMS_LIKELY(x) (x)
#define TMS_UNLIKELY(x) (x)
#endif

enum {
	IN_PLOAD = 0x00000001,
	IN_CLOAD = 0x00000002,
	SU_CVAL  = 0x00000004,
	SU_MASK  = 0x00000018, SU_ST0 = 0x00, SU_ST1 = 0x08, SU_PRG = 0x10,
	S_IDLE   = 0x00000020,
	S_READ   = 0x00000040,
	S_WRITE  = 0x00000080,
	S_BRANCH = 0x00000100,
	S_HOST   = 0x00000200,
	S_UPDATE = 0x00000400
};

enum {
	ST0_INCS = 0x000001,
	ST0_DIRI = 0x000002,
	ST0_FI   = 0x000004,
	ST0_SIM  = 0x000008,
	ST0_PLRI = 0x000020,
	ST0_PBCI = 0x000040,
	ST0_DIRO = 0x000080,
	ST0_FO   = 0x000100,
	ST0_SOM  = 0x000600,
	ST0_PLRO = 0x000800,
	ST0_PBCO = 0x001000,
	ST0_CNS  = 0x002000,
	ST0_WORD = 0x004000,
	ST0_SEL  = 0x008000,
	ST0_M    = 0x030000, ST0_M_64K = 0x000000, ST0_M_256K = 0x010000, ST0_M_1M = 0x020000,
	ST0_SRAM = 0x200000,

	ST1_AOV  = 0x000001,
	ST1_SFAI = 0x000002,
	ST1_SFAO = 0x000004,
	ST1_AOVM = 0x000008, // undocumented!
	ST1_MOVM = 0x000020,
	ST1_MOV  = 0x000040,
	ST1_SFMA = 0x000180, ST1_SFMA_SHIFT = 7,
	ST1_SFMO = 0x001800, ST1_SFMO_SHIFT = 11,
	ST1_RND  = 0x038000, ST1_RND_SHIFT = 15,
	ST1_CRM  = 0x0C0000, ST1_CRM_SHIFT = 18, ST1_CRM_32 = 0x000000, ST1_CRM_16H = 0x040000, ST1_CRM_16L = 0x080000,
	ST1_DBP  = 0x100000,
	ST1_CAS  = 0x200000,

	ST1_CACHE = ST1_SFAI|ST1_SFAO|ST1_MOVM|ST1_SFMA|ST1_SFMO|ST1_RND|ST1_CRM|ST1_DBP
};

enum { BR_UB, BR_CB, BR_IDLE };   // (as MAME: BR_UB is 0, so a block's decoding goes on past a "b")

enum { IBS = TMS57002_IBS, HBS = TMS57002_HBS };

enum { INC_CA = 1, INC_ID = 2 };

typedef struct {
	int branch;
	int inc;
	int16_t hnode;
	int16_t ipc;
	int spec;   // (ReBOX: an op of the instruction in CINTRPSPECIAL)
} cstate;

_Static_assert(offsetof(tms57002_t, macc) == 488 && offsetof(tms57002_t, cmem) == 128 * 4 &&
		offsetof(tms57002_t, dmem0) == 384 * 4, "tms57002_t's layout");

// ---- the data space ----
static inline uint8_t data_read(const tms57002_t *s, uint32_t adr)
{
	return adr < s->data_size ? s->data[adr] : 0;
}
static inline void data_write(tms57002_t *s, uint32_t adr, uint8_t v)
{
	if (adr < s->data_size)
		s->data[adr] = v;
}

// ---- the host interface ----
static void update_pc0(tms57002_t *s) { (void)s; }      // (no pc0 / dready / empty callbacks on the GX board)
static void update_dready(tms57002_t *s) { (void)s; }
static void update_empty(tms57002_t *s) { (void)s; }
static void cache_flush(tms57002_t *s);

void tms57002_pload_w(tms57002_t *s, int state)
{
	const uint8_t olds = (uint8_t)s->sti;
	if (state)
		s->sti &= ~(uint32_t)IN_PLOAD;
	else
		s->sti |= IN_PLOAD;
	if (olds ^ (uint8_t)s->sti) {
		if (s->sti & IN_PLOAD) {
			s->hidx = 0;
			s->pc = 0;
			s->ca = 0;
			s->sti &= ~(uint32_t)SU_MASK;
		}
	}
}

void tms57002_cload_w(tms57002_t *s, int state)
{
	const uint8_t olds = (uint8_t)s->sti;
	if (state)
		s->sti &= ~(uint32_t)IN_CLOAD;
	else
		s->sti |= IN_CLOAD;
	if (olds ^ (uint8_t)s->sti) {
		if (s->sti & IN_CLOAD) {
			s->hidx = 0;
			//ca = 0; // Seems extremely dubious
		}
	}
}

void tms57002_reset(tms57002_t *s)
{
	s->sti = (s->sti & ~(uint32_t)(SU_MASK|S_READ|S_WRITE|S_BRANCH|S_HOST|S_UPDATE)) | (SU_ST0|S_IDLE);
	s->pc = 0;
	s->ca = 0;
	s->hidx = 0;
	s->id = 0;
	s->ba0 = 0;
	s->ba1 = 0;
	s->sa = 0;
	s->rptc = 0;
	s->rptc_next = 0;
	s->update_counter_tail = 0;
	s->update_counter_head = 0;
	s->st0 &= ~(uint32_t)(ST0_INCS | ST0_DIRI | ST0_FI | ST0_SIM | ST0_PLRI |
			ST0_PBCI | ST0_DIRO | ST0_FO | ST0_SOM | ST0_PLRO |
			ST0_PBCO | ST0_CNS);
	s->st1 &= ~(uint32_t)(ST1_AOV | ST1_SFAI | ST1_SFAO | ST1_AOVM | ST1_MOVM | ST1_MOV |
			ST1_SFMA | ST1_SFMO | ST1_RND | ST1_CRM | ST1_DBP);
	update_dready(s);
	update_pc0(s);
	update_empty(s);

	s->xba = 0;
	s->xoa = 0;
	cache_flush(s);
}

void tms57002_set_reset_line(tms57002_t *s, int asserted)
{
	if (asserted)
		s->reset_held = 1;
	else {
		if (s->reset_held)   // (coming out of reset: the device reset)
			tms57002_reset(s);
		s->reset_held = 0;
	}
}

void tms57002_data_w(tms57002_t *s, uint8_t data)
{
	switch (s->sti & (IN_PLOAD|IN_CLOAD)) {
	case 0:
		s->hidx = 0;
		s->sti &= ~(uint32_t)SU_CVAL;
		break;
	case IN_PLOAD:
		s->host[s->hidx++] = data;
		if (s->hidx >= 3) {
			const uint32_t val = (uint32_t)s->host[0]<<16 | (uint32_t)s->host[1]<<8 | s->host[2];
			s->hidx = 0;

			switch (s->sti & SU_MASK) {
			case SU_ST0:
				s->st0 = val;
				s->sti = (s->sti & ~(uint32_t)SU_MASK) | SU_ST1;
				break;
			case SU_ST1:
				s->st1 = val;
				s->sti = (s->sti & ~(uint32_t)SU_MASK) | SU_PRG;
				break;
			case SU_PRG:
				s->program[s->pc++] = val;
				update_pc0(s);
				break;
			}
		}
		break;
	case IN_CLOAD:
		if (s->sti & SU_CVAL) {
			s->host[s->hidx++] = data;
			if (s->hidx >= 4) {
				const uint32_t val = (uint32_t)s->host[0]<<24 | (uint32_t)s->host[1]<<16 | (uint32_t)s->host[2]<<8 | s->host[3];
				s->sti &= ~(uint32_t)SU_CVAL;
				s->update[s->update_counter_head] = val;
				s->update_counter_head = (s->update_counter_head + 1) & 0x0f;
				s->hidx = 1; // the write shouldn't really happen until CLOAD is high though
				update_empty(s);
			}
		} else {
			s->sa = data;
			s->hidx = 0;
			s->sti |= SU_CVAL;
		}
		break;
	case IN_PLOAD|IN_CLOAD:
		s->host[s->hidx++] = data;
		if (s->hidx >= 4) {
			const uint32_t val = (uint32_t)s->host[0]<<24 | (uint32_t)s->host[1]<<16 | (uint32_t)s->host[2]<<8 | s->host[3];
			s->hidx = 0;
			s->cmem[s->ca++] = val;
		}
		break;
	}
}

uint8_t tms57002_data_r(tms57002_t *s)
{
	uint8_t res;
	if (!(s->sti & S_HOST))
		return 0xff;

	res = s->host[s->hidx];
	s->hidx++;
	if (s->hidx == 4) {
		s->hidx = 0;
		s->sti &= ~(uint32_t)S_HOST;
		update_dready(s);
	}
	return res;
}

int tms57002_dready_r(const tms57002_t *s) { return s->sti & S_HOST ? 0 : 1; }
int tms57002_pc0_r(const tms57002_t *s) { return s->pc == 0 ? 0 : 1; }
int tms57002_empty_r(const tms57002_t *s) { return s->update_counter_head == s->update_counter_tail; }

static void sync_w(tms57002_t *s)
{
	if (s->sti & (IN_PLOAD /*| IN_CLOAD*/))
		return;

	s->allow_update = 1;
	s->pc = 0;
	s->ca = 0;
	s->id = 0;
	if (!(s->st0 & ST0_INCS)) {
		s->ba0--;
		s->ba1++;
	}
	s->xba = (s->xba-1) & 0x7ffff;
	s->st1 &= ~(uint32_t)(ST1_AOV | ST1_MOV);
	s->sti &= ~(uint32_t)S_IDLE;
}

// ---- the external memory ----
// (MAME xm_init's address mask and shift from st0, set at a run's start: st0 is the host's)
static void xm_mode(tms57002_t *s)
{
	uint32_t mask = 0;

	switch (s->st0 & ST0_M) {
	case ST0_M_64K:  mask = 0x0ffff; break;
	case ST0_M_256K: mask = 0x3ffff; break;
	case ST0_M_1M:   mask = 0xfffff; break;
	}
	s->h.xm_mask = mask;
	s->h.xm_shift = (s->st0 & ST0_WORD ? 2 : 1) + (s->st0 & ST0_SEL ? 0 : 1);
}

static TMS_INLINE void xm_init_(tms57002_t *s)
{
	s->xm_adr = ((s->xoa + s->xba) << s->h.xm_shift) & s->h.xm_mask;
}

// A step of a transfer (MAME xm_step_read / xm_step_write, one at each instruction's start), at adr: 1 at the last
static TMS_INLINE int xm_rd1(const tms57002_t *s, uint32_t adr, uint32_t *tx)
{
	const uint32_t v = data_read(s, adr);
	int done;
	if (s->st0 & ST0_WORD) {
		if (s->st0 & ST0_SEL) {
			const int off = 16 - ((adr & 3) << 3);
			*tx = (*tx & ~(0xffu << off)) | (v << off);
			done = off == 0;
		} else {
			const int off = 20 - ((adr & 7) << 2);
			*tx = (*tx & ~(0xfu << off)) | ((v & 0xf) << off);
			done = off == 0;
		}
	} else {
		if (s->st0 & ST0_SEL) {
			const int off = 16 - ((adr & 1) << 3);
			*tx = (*tx & ~(0xffu << off)) | (v << off);
			done = off == 8;
			if (done)
				*tx &= 0xffff00;
		} else {
			const int off = 20 - ((adr & 3) << 2);
			*tx = (*tx & ~(0xfu << off)) | ((v & 0xf) << off);
			done = off == 8;
			if (done)
				*tx &= 0xffff00;
		}
	}
	return done;
}

static TMS_INLINE int xm_wr1(tms57002_t *s, uint32_t adr)
{
	uint8_t v;
	int done;
	if (s->st0 & ST0_WORD) {
		if (s->st0 & ST0_SEL) {
			const int off = 16 - ((adr & 3) << 3);
			v = (uint8_t)(s->xwr >> off);
			done = off == 0;
		} else {
			const int off = 20 - ((adr & 7) << 2);
			v = (s->xwr >> off) & 0xf;
			done = off == 0;
		}
	} else {
		if (s->st0 & ST0_SEL) {
			const int off = 16 - ((adr & 1) << 3);
			v = (uint8_t)(s->xwr >> off);
			done = off == 8;
		} else {
			const int off = 20 - ((adr & 3) << 2);
			v = (s->xwr >> off) & 0xf;
			done = off == 8;
		}
	}
	data_write(s, adr, v);
	return done;
}

// A transfer's steps to its end, this one included (the done tests above: the address's low bits)
static int xm_steps(const tms57002_t *s)
{
	const uint32_t adr = s->xm_adr;
	switch (s->st0 & (ST0_WORD | ST0_SEL)) {
	case ST0_WORD | ST0_SEL: return (int)((2 - adr) & 3) + 1;   // (off 16 - (adr & 3) * 8 == 0)
	case ST0_WORD:           return (int)((5 - adr) & 7) + 1;   // (off 20 - (adr & 7) * 4 == 0)
	case ST0_SEL:            return (int)((1 - adr) & 1) + 1;   // (off 16 - (adr & 1) * 8 == 8)
	default:                 return (int)((3 - adr) & 3) + 1;   // (off 20 - (adr & 3) * 4 == 8)
	}
}

// Here a transfer is done whole at its first step and only its end (xrd, S_READ / S_WRITE cleared) waits for the
// last one's: until then the steps change only the transfer's state (txrd, xm_adr) and, a write, the data space,
// which only a later read transfer reads (one at a time). The interpreter counts the steps left (xm; -1: the next
// is the first, from xm_init or a run's start); at a run's end, xm_sync puts the transfer's state back as MAME's.
// xm_first gives the steps.
static TMS_NOINLINE int xm_first(tms57002_t *s)
{
	const uint32_t adr = s->xm_adr;
	int n;
	s->h.xm_adr0 = adr;
	s->h.xm_tx0 = s->txrd;
	if ((s->st0 & (ST0_WORD | ST0_SEL)) == (ST0_WORD | ST0_SEL) && !(adr & 3) && adr + 2 < s->data_size) {
		// (the bytes at off 16, 8 and 0, the last at adr & 3 == 2)
		uint8_t *d = s->data + adr;
		n = 3;
		if (s->sti & S_READ)
			s->h.xm_final = (s->txrd & 0xff000000) | (uint32_t)d[0] << 16 | (uint32_t)d[1] << 8 | d[2];
		else {
			d[0] = (uint8_t)(s->xwr >> 16);
			d[1] = (uint8_t)(s->xwr >> 8);
			d[2] = (uint8_t)s->xwr;
		}
	} else {
		n = xm_steps(s);
		if (s->sti & S_READ) {
			uint32_t tx = s->txrd;
			for (int k = 0; k < n; k++)
				xm_rd1(s, adr + (uint32_t)k, &tx);
			s->h.xm_final = tx;
		} else
			for (int k = 0; k < n; k++)
				xm_wr1(s, adr + (uint32_t)k);
	}
	s->h.xm_n = n;
	return n;
}

static TMS_INLINE void xm_last(tms57002_t *s)
{
	if (s->sti & S_READ) {
		s->txrd = s->xrd = s->h.xm_final;
		s->sti &= ~(uint32_t)S_READ;
	} else
		s->sti &= ~(uint32_t)S_WRITE;
	s->xm_adr = 0;
}

// (a transfer's step at an instruction's start)
#define XM_STEP() do { \
		if (xm) { \
			if (xm < 0) \
				xm = xm_first(s); \
			if (!--xm) \
				xm_last(s); \
		} \
	} while (0)

static void xm_sync(tms57002_t *s, int xm)
{
	if (xm > 0) {
		const int done = s->h.xm_n - xm;   // (none the last)
		if (s->sti & S_READ) {
			uint32_t tx = s->h.xm_tx0;
			for (int k = 0; k < done; k++)
				xm_rd1(s, s->h.xm_adr0 + (uint32_t)k, &tx);
			s->txrd = tx;
		}
		s->xm_adr = s->h.xm_adr0 + (uint32_t)done;
	}
}

// ---- the multiplier accumulator's outputs (MAME macc_to_output_N[s] / check_macc_overflow_N[s]) ----
// MAME's overflow tests are m & mask neither 0 nor the mask, the mask the bits lo..51 (lo 47, 45 or 43): those bits
// not all the same, t + 1 (t the bits) not 0 nor 1 in w = 52 - lo bits. The bits are in m's high word.
static TMS_INLINE int macc_over(int64_t m, int lo)
{
	const uint32_t h = (uint32_t)((uint64_t)m >> 32);
	return (((h >> (lo - 32)) + 1) & ((2u << (51 - lo)) - 2)) != 0;
}

static TMS_INLINE int64_t macc_sat(int64_t m)
{
	return m & 0x8000000000000LL ? (int64_t)0xffff800000000000ULL : 0x00007fffffffffffLL;
}

// lo: the shift's overflow mask (0: none), shift: <0 right; sat: saturate on overflow
static TMS_INLINE int64_t macc_to_output(tms57002_t *s, int64_t rounding, uint64_t rmask, int lo, int shift, int sat)
{
	int64_t m = s->macc_read;
	int over = lo ? macc_over(m, lo) : 0;   // (overflow detection and shifting)

	if (shift > 0)
		m = (int64_t)((uint64_t)m << shift);
	else if (shift < 0)
		m >>= -shift;

	m = (int64_t)((uint64_t)(m + rounding) & rmask);

	over |= macc_over(m, 47);   // (second overflow detection, 0xf800000000000)

	// Overflow handling
	if (over) {
		s->st1 |= ST1_MOV;
		if (sat)
			m = macc_sat(m);
	}
	return m;
}

static TMS_INLINE int64_t check_macc_overflow(tms57002_t *s, int lo, int sat)
{
	int64_t m = s->macc_read;

	// Overflow detection
	if (lo && macc_over(m, lo)) {
		s->st1 |= ST1_MOV;
		if (sat)
			m = macc_sat(m);
	}
	return m;
}

// get_cmem's fast test: cm_slow[addr] when the update's part may be taken or crm is not 0 (cm_key: all, 0x200;
// queued, the address sa; none, 0x100). Kept by cm_set after what changes them: the host's writes, the updates
// (get_cmem_slow), st1 (the decoding, the crm ops: cm_all before their write)
enum { CM_NONE = 0x100, CM_ALL = 0x200 };

static void cm_all(tms57002_t *s)
{
	if (s->h.cm_key != CM_ALL) {
		memset(s->h.cm_slow, 1, sizeof(s->h.cm_slow));
		s->h.cm_key = CM_ALL;
	}
}

static void cm_set(tms57002_t *s)
{
	const uint16_t key = (s->sti & S_UPDATE) || (s->st1 & ST1_CRM) ? CM_ALL :
		s->update_counter_head != s->update_counter_tail ? s->sa : CM_NONE;
	if (key == s->h.cm_key)
		return;
	if (s->h.cm_key == CM_ALL)
		memset(s->h.cm_slow, 0, sizeof(s->h.cm_slow));
	else if (s->h.cm_key < CM_NONE)
		s->h.cm_slow[s->h.cm_key] = 0;
	if (key == CM_ALL)
		memset(s->h.cm_slow, 1, sizeof(s->h.cm_slow));
	else if (key < CM_NONE)
		s->h.cm_slow[key] = 1;
	s->h.cm_key = key;
}

static uint32_t get_cmem_crm(const tms57002_t *s, uint32_t cvar)
{
	const int crm = (s->st1 & ST1_CRM) >> ST1_CRM_SHIFT;
	if (crm == 1)
		return (cvar & 0xffff0000);
	else if (crm == 2)
		return (cvar << 16);
	return cvar;
}

static TMS_NOINLINE uint32_t get_cmem_slow(tms57002_t *s, uint8_t addr)
{
	uint32_t v;
	if (s->sa == addr && s->update_counter_head != s->update_counter_tail)
		s->sti |= S_UPDATE;

	if (s->sti & S_UPDATE) {
		s->cmem[addr] = s->update[s->update_counter_tail];
		s->update_counter_tail = (s->update_counter_tail + 1) & 0x0f;
		update_empty(s);

		if (s->update_counter_head == s->update_counter_tail)
			s->sti &= ~(uint32_t)S_UPDATE;

		v = s->cmem[addr]; // The value of crm is ignored during an update.
	} else
		v = get_cmem_crm(s, s->cmem[addr]);
	cm_set(s);
	return v;
}

// (the usual case inline: the update's part not taken, crm 0)
static TMS_INLINE uint32_t get_cmem_(tms57002_t *s, uint8_t addr)
{
	if (TMS_LIKELY(!s->h.cm_slow[addr]))
		return s->cmem[addr];
	return get_cmem_slow(s, addr);
}

// ---- the generated interpreters' names (MAME tmsops.cpp; their state's in execute_run, where they are) ----
#define TMS_MAX(a, b) ((a) > (b) ? (a) : (b))
#define TMS_MIN(a, b) ((a) < (b) ? (a) : (b))
#define int64_t(x) ((int64_t)(x))   // (C++'s functional cast)

#define get_cmem(a)               get_cmem_(s, (uint8_t)(a))
#define xm_init()                 xm_init_(s)
#define update_dready()           update_dready(s)
// (the overflow masks 0xf800000000000, 0xfe00000000000, 0xff80000000000: bits 47, 45, 43 to 51)
#define macc_to_output_0(r, m)    macc_to_output(s, r, m, 47, 0, 0)
#define macc_to_output_1(r, m)    macc_to_output(s, r, m, 45, 2, 0)
#define macc_to_output_2(r, m)    macc_to_output(s, r, m, 43, 4, 0)
#define macc_to_output_3(r, m)    macc_to_output(s, r, m, 0, -8, 0)
#define macc_to_output_0s(r, m)   macc_to_output(s, r, m, 47, 0, 1)
#define macc_to_output_1s(r, m)   macc_to_output(s, r, m, 45, 2, 1)
#define macc_to_output_2s(r, m)   macc_to_output(s, r, m, 43, 4, 1)
#define macc_to_output_3s(r, m)   macc_to_output(s, r, m, 0, -8, 1)
#define check_macc_overflow_0()   check_macc_overflow(s, 47, 0)
#define check_macc_overflow_1()   check_macc_overflow(s, 45, 0)
#define check_macc_overflow_2()   check_macc_overflow(s, 43, 0)
#define check_macc_overflow_3()   check_macc_overflow(s, 0, 0)
#define check_macc_overflow_0s()  check_macc_overflow(s, 47, 1)
#define check_macc_overflow_1s()  check_macc_overflow(s, 45, 1)
#define check_macc_overflow_2s()  check_macc_overflow(s, 43, 1)
#define check_macc_overflow_3s()  check_macc_overflow(s, 0, 1)

// ---- the decoders (MAME tms57kdec.cpp) ----
#define st1         (s->st1)
static inline int xmode(uint32_t opcode, char type, cstate *cs)
{
	if (((opcode & 0x400) && (type == 'c')) || (!(opcode & 0x400) && (type == 'd'))) {
		if (opcode & 0x100)
			return 0;
		else if (opcode & 0x80)
			cs->inc |= type == 'c' ? INC_CA : INC_ID;

		return 1;
	} else if (opcode & 0x200)
		cs->inc |= type == 'c' ? INC_CA : INC_ID;

	return 1;
}

static inline int sfao(uint32_t v) { return v & ST1_SFAO ? 1 : 0; }
static inline int dbp(uint32_t v) { return v & ST1_DBP ? 1 : 0; }
static inline int sfai(uint32_t v) { return v & ST1_SFAI ? 1 : 0; }
static inline int sfmo(uint32_t v) { return (v & ST1_SFMO) >> ST1_SFMO_SHIFT; }
static inline int rnd(uint32_t v)
{
	const int r = (v & ST1_RND) >> ST1_RND_SHIFT;
	return r <= 4 ? r : 0;
}
static inline int movm(uint32_t v) { return v & ST1_MOVM ? 1 : 0; }
static inline int sfma(uint32_t v) { return (v & ST1_SFMA) >> ST1_SFMA_SHIFT; }

// (the unhandled opcodes ignored, as MAME after its one-time message)
static void decode_cat1(tms57002_t *s, uint32_t opcode, uint16_t *op, cstate *cs)
{
	switch (opcode >> 18) {
	case 0x00: // nop
		break;

#define CDEC1
#include "tms57002.hxx"
#undef CDEC1

	default:
		break;
	}
}

static void decode_cat2_pre(tms57002_t *s, uint32_t opcode, uint16_t *op, cstate *cs)
{
	switch ((opcode >> 11) & 0x7f) {
	case 0x00: // nop
		break;

#define CDEC2A
#include "tms57002.hxx"
#undef CDEC2A

	default:
		break;
	}
}

static void decode_cat2_post(tms57002_t *s, uint32_t opcode, uint16_t *op, cstate *cs)
{
	switch ((opcode >> 11) & 0x7f) {
	case 0x00: // nop
		break;

#define CDEC2B
#include "tms57002.hxx"
#undef CDEC2B

	default:
		break;
	}
}

static void decode_cat3(tms57002_t *s, uint32_t opcode, uint16_t *op, cstate *cs)
{
	switch ((opcode >> 11) & 0x7f) {
	case 0x00: // nop
		break;

#define CDEC3
#include "tms57002.hxx"
#undef CDEC3

	default:
		break;
	}
}

#undef st1

// ---- the instruction cache ----
static uint32_t s_gen;   // (the generations: unique across chips and games)

static void cache_flush(tms57002_t *s)
{
	int i;
	tms57002_cd *c = &s->cache;
	s->h.gen = ++s_gen;
	c->hused = c->iused = 0;
	memset(s->memo, 0, sizeof(s->memo));
	for (i = 0; i != 256; i++)
		c->hashbase[i] = -1;
	for (i = 0; i != HBS; i++) {
		c->hashnode[i].st1 = 0;
		c->hashnode[i].ipc = -1;
		c->hashnode[i].next = -1;
	}
	for (i = 0; i != IBS; i++) {
		c->inst[i].op = 0;
		c->inst[i].next = -1;
		c->inst[i].param = 0;
	}
}

// the interpreter's handlers by op (0-3: the instruction's end with its increments, then the generated ops),
// OP_DECODE the sentinel's, OP_EEND a block's last end (next -1). An end's param: its instruction's address, XP_END
// when the generated op before it ends the instruction itself (not the last end, and no id increment) and its
// increments XP_CA / XP_ID
enum { OP_COUNT = 1656, OP_DECODE = OP_COUNT, OP_EEND, XP_END = 0x100, XP_CA_SHIFT = 16, XP_CA = 1 << XP_CA_SHIFT,
	XP_ID = 0x1000000 };
static const void *const *s_ops;
static uint8_t s_spec[OP_COUNT];   // (the ops of CINTRPSPECIAL: their instruction's end is a slow one)

static void xop_set(tms57002_t *s, int ipc)
{
	const tms57002_icd *c = &s->cache.inst[ipc];
	tms57002_xop *x = &s->xop[ipc];
	const int op = c->op < OP_COUNT ? c->op : 0;   // (an unknown op: as the original's default, the end)
	x->next = c->next == -1 ? &s->xsent : &s->xop[c->next];
	if (op < 4) {
		const int slow = c->next == -1 || s->xspec[ipc];
		x->lbl = s_ops[slow ? OP_EEND : op];
		x->param = s->xadr[ipc] | (slow || (op & INC_ID) ? 0 : XP_END) | (op & INC_CA ? XP_CA : 0) |
			(op & INC_ID ? XP_ID : 0);
	} else {
		x->lbl = s_ops[op];
		x->param = c->param;
	}
}

// an end's increments (ca in execute_run's local)
#define END_INCS(param) do { \
		if ((param) & XP_CA) \
			++lca; \
		if ((param) & XP_ID) \
			++s->id; \
	} while (0)

static void set_next(tms57002_t *s, int16_t ipc, int16_t next)
{
	s->cache.inst[ipc].next = next;
	xop_set(s, ipc);
}

// the threaded cache from cache.inst (after a state load): the ends' addresses from the hash (an instruction's
// micro-ops consecutive, its end the last)
static void xop_rebuild(tms57002_t *s)
{
	const tms57002_cd *c = &s->cache;
	for (int adr = 0; adr < 256; adr++)
		for (int16_t h = c->hashbase[adr]; h != -1; h = c->hashnode[h].next) {
			int ipc = c->hashnode[h].ipc;
			if (ipc < 0)
				continue;
			int spec = 0;
			while (ipc < c->iused - 1 && c->inst[ipc].op >= 4 && c->inst[ipc].op < OP_COUNT)
				spec |= s_spec[c->inst[ipc++].op];
			s->xadr[ipc] = (uint8_t)adr;
			s->xspec[ipc] = (uint8_t)spec;
		}
	for (int ipc = 0; ipc < c->iused; ipc++)
		xop_set(s, ipc);
	memset(s->memo, 0, sizeof(s->memo));
	s->h.gen = ++s_gen;
}

static void add_one(tms57002_t *s, cstate *cs, uint16_t op, uint8_t param)
{
	tms57002_cd *c = &s->cache;
	const int16_t ipc = (int16_t)c->iused++;
	c->inst[ipc].op = op;
	c->inst[ipc].param = param;
	c->inst[ipc].next = -1;
	if (op < 4)
		s->xspec[ipc] = (uint8_t)cs->spec;
	else if (op < OP_COUNT)
		cs->spec |= s_spec[op];
	xop_set(s, ipc);
	if (cs->ipc != -1)
		set_next(s, cs->ipc, ipc);
	cs->ipc = ipc;
	if (cs->hnode != -1) {
		c->hashnode[cs->hnode].ipc = ipc;
		cs->hnode = -1;
	}
}

typedef void (*decode_fn)(tms57002_t *s, uint32_t opcode, uint16_t *op, cstate *cs);

static void decode_one(tms57002_t *s, uint32_t opcode, cstate *cs, decode_fn dec)
{
	uint16_t op = 0;
	dec(s, opcode, &op, cs);
	if (!op)
		return;
	add_one(s, cs, op, opcode & 0xff);
}

static int16_t get_hash(tms57002_t *s, uint8_t adr, uint32_t st1, int16_t *pnode)
{
	int16_t hnode;
	st1 &= ST1_CACHE;
	*pnode = -1;
	hnode = s->cache.hashbase[adr];
	while (hnode != -1) {
		if (s->cache.hashnode[hnode].st1 == st1)
			return s->cache.hashnode[hnode].ipc;
		*pnode = hnode;
		hnode = s->cache.hashnode[hnode].next;
	}
	return -1;
}

static int16_t get_hashnode(tms57002_t *s, uint8_t adr, uint32_t st1, int16_t pnode)
{
	const int16_t hnode = (int16_t)s->cache.hused++;
	s->cache.hashnode[hnode].st1 = st1 & ST1_CACHE;
	s->cache.hashnode[hnode].ipc = -1;
	s->cache.hashnode[hnode].next = -1;
	if (pnode == -1)
		s->cache.hashbase[adr] = hnode;
	else
		s->cache.hashnode[pnode].next = hnode;
	return hnode;
}

static int decode_get_pc(tms57002_t *s)
{
	int16_t pnode, res;
	cstate cs;
	uint8_t adr = s->pc;

	res = get_hash(s, adr, s->st1, &pnode);
	if (res != -1)
		return res;

	if (HBS - s->cache.hused < 256 || IBS - s->cache.iused < 256*3) {
		cache_flush(s);
		pnode = -1;
	}

	cs.hnode = res = get_hashnode(s, adr, s->st1, pnode);
	cs.ipc = -1;
	cs.branch = 0;

	for (;;) {
		int16_t ipc;
		const uint32_t opcode = s->program[adr];

		cs.inc = 0;
		cs.spec = 0;

		if ((opcode & 0xfc0000) == 0xfc0000)
			decode_one(s, opcode, &cs, decode_cat3);
		else {
			decode_one(s, opcode, &cs, decode_cat2_pre);
			decode_one(s, opcode, &cs, decode_cat1);
			decode_one(s, opcode, &cs, decode_cat2_post);
		}
		s->xadr[s->cache.iused] = adr;
		add_one(s, &cs, (uint16_t)cs.inc, 0);

		if (cs.branch)
			break;

		adr++;
		ipc = get_hash(s, adr, s->st1, &pnode);
		if (ipc != -1) {
			set_next(s, cs.ipc, ipc);
			break;
		}
		cs.hnode = get_hashnode(s, adr, s->st1, pnode);
	}

	return s->cache.hashnode[res].ipc;
}

// MAME's execute_run, threaded (GCC labels as values): each micro-op's handler jumps to the next one's. An
// instruction's last micro-op is always its end, its increments (ops 0-3: the decoder adds one to every
// instruction); the generated op before it does the end itself (INST_END), one dispatch an instruction. The end's
// usual case inline (icount--, the memory transfer's step, the macc copies), the rest (repeat, branch, idle, a
// block's end, the cycles' end) in inst_slow / inst_start, as MAME's loop. The decoding (ipc -1: the sentinel's
// handler) comes after the step and the copies at an instruction's start, which it does not touch. With s NULL:
// the handlers' table.
#if !defined(__GNUC__)
#error "tms57002.c: the interpreter needs GCC's labels as values"
#endif
#if !defined(__clang__)
__attribute__((optimize("no-gcse", "no-crossjumping")))   // (GCC: keeps the computed gotos apart)
#endif
#if defined(__arm__) && defined(__thumb__)
__attribute__((target("arm")))   // (ARM state: the dispatch one ldr pc, the conditions without IT)
#endif
static const void *const *execute_run(tms57002_t *s)
{
#define OP(n) [n] = &&l_##n,
	static const void *const ops[OP_EEND + 1] = {
		[0] = &&l_end0, [1] = &&l_end1, [2] = &&l_end, [3] = &&l_end,
#define CINTRPLIST
#include "tms57002.hxx"
#undef CINTRPLIST
		[OP_DECODE] = &&l_decode, [OP_EEND] = &&l_eend
	};
#undef OP
	const icd *ip, *iip;
	int icount;
	uint32_t endp = 0;                       // (the end's param, into inst_slow)
	int icount_held = 0;                     // (while repeating or branching: icount, the local then 1)
	int xm = 0;                              // (a memory transfer's steps left, see xm_first)
	// (macc and ca in locals during the run; the rest in s)
	int64_t lmacc;
	uint8_t lca;

	// the generated interpreters (GCC nested functions, inlined: the locals in their scope)
#define macc        lmacc
#define ca          lca
#define ba0         (s->ba0)
#define macc_write  (s->macc_write)
#define cmem        (s->cmem)
#define dmem0       (s->dmem0)
#define dmem1       (s->dmem1)
#define si          (s->si)
#define so          (s->so)
#define st0         (s->st0)
#define st1         (s->st1)
#define sti         (s->sti)
#define aacc        (s->aacc)
#define xoa         (s->xoa)
#define xwr         (s->xwr)
#define xrd         (s->xrd)
#define creg        (s->creg)
#define pc          (s->pc)
#define id          (s->id)
#define ba1         (s->ba1)
#define rptc_next   (s->rptc_next)
#define host        (s->host)
#define hidx        (s->hidx)
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#define TMS_EX inline __attribute__((always_inline))
#undef xm_init
#define xm_init() (xm_init_(s), xm = -1)
// (the crm ops: get_cmem's fast test off before st1's write, then cm_set at their instruction's end)
#define ST1_CRM_SHIFT (cm_all(s), ST1_CRM_SHIFT)
#define CINTRP
#include "tms57002.hxx"
#undef CINTRP
#undef ST1_CRM_SHIFT
#undef TMS_EX
#undef xm_init
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
#undef macc
#undef ca
#undef ba0
#undef macc_write
#undef cmem
#undef dmem0
#undef dmem1
#undef si
#undef so
#undef st0
#undef st1
#undef sti
#undef aacc
#undef xoa
#undef xwr
#undef xrd
#undef creg
#undef pc
#undef id
#undef ba1
#undef rptc_next
#undef host
#undef hidx

	if (!s)
		return ops;

	lmacc = s->macc;
	lca = s->ca;
	xm_mode(s);
	xm = s->sti & (S_READ | S_WRITE) ? -1 : 0;   // (a transfer from the last run: from its state, as MAME's)
	icount = s->icount;
	if (icount <= 0 || (s->sti & (S_IDLE | IN_PLOAD /*| IN_CLOAD*/)))   // (IN_PLOAD: not changed in a run)
		goto done;
	cm_set(s);
	ip = &s->xsent;   // (ipc -1)
	goto inst_start;

	// the end of an instruction, ip the end: the increments, icount--; the usual case here, the rest in inst_slow:
	// the ends of a block (pc needed for the decoding) and of the instructions of a CINTRPSPECIAL op (branch, idle,
	// repeat, crm), always (l_eend); while repeating or branching (icount held) and at the cycles' end. pc
	// is kept by the slow ends only: the others are followed by the instruction at pc + 1, and nothing reads it in
	// between.
#define INST_END(p, incs) do { \
		const icd *nx = ip->next; \
		const uint32_t ep = (p); \
		incs; \
		if (TMS_UNLIKELY(--icount <= 0)) { \
			endp = ep; \
			ip = nx; \
			goto inst_slow; \
		} \
		XM_STEP(); \
		s->macc_read = s->macc_write; \
		s->macc_write = lmacc; \
		ip = nx; \
		goto *ip->lbl; \
	} while (0)

l_end0:   // (an instruction of no generated op)
	INST_END(ip->param, (void)0);
l_end1:
	INST_END(ip->param, ++lca);
l_end:   // (or of an id increment)
	INST_END(ip->param, END_INCS(ep));

l_eend:
	endp = ip->param;
	END_INCS(endp);
	ip = ip->next;
	--icount;
	goto inst_slow;

	// a generated op; when the end follows, it is done here (one dispatch less, and one per op)
#define OP(n) l_##n: \
	ex_##n(s, ip); \
	ip = ip->next; \
	if (ip->param & XP_END) \
		INST_END(ip->param, lca += (ep >> XP_CA_SHIFT) & 1); \
	goto *ip->lbl;
#define CINTRPLIST
#include "tms57002.hxx"
#undef CINTRPLIST
#undef OP
#undef INST_END

l_decode: {
		// (decode_get_pc's result for a pc and st1 stays until the cache's flush)
		const uint32_t key = s->st1 & ST1_CACHE;
		tms57002_memo *m = &s->memo[s->pc];
		if (!m->ip || m->key != key) {
			m->ip = &s->xop[decode_get_pc(s)];   // (a flush there clears the memo first)
			m->key = key;
			cm_set(s);   // (the decoding sets crm, MAME)
		}
		ip = iip = m->ip;
		goto *ip->lbl;
	}

inst_slow:   // (icount decremented, ip the next micro-op, endp the end's param)
	if (icount_held) {
		icount = icount_held - 1;
		icount_held = 0;
	}
	if (s->rptc) {
		s->rptc--;
		ip = iip;
	} else if (s->sti & S_BRANCH) {
		s->sti &= ~(uint32_t)S_BRANCH;
		ip = &s->xsent;
	} else {
		s->pc = (uint8_t)(endp + 1); // (pc++) Wraps if it reaches 256, next wraps too
		update_pc0(s);
	}

	if (s->rptc_next) {
		s->rptc = s->rptc_next;
		s->rptc_next = 0;
	}

	if (icount <= 0 || (s->sti & S_IDLE))
		goto done;

inst_start:   // (an instruction's start; iip, for a repeat, kept here only: the ends while repeating are slow)
	if (s->rptc || (s->sti & S_BRANCH)) {   // (the end then slow)
		icount_held = icount;
		icount = 1;
	}
	if (s->h.cm_key == CM_ALL)   // (after a crm op, cm_all)
		cm_set(s);
	XM_STEP();
	s->macc_read = s->macc_write;
	s->macc_write = lmacc;
	iip = ip;
	goto *ip->lbl;

done:
	xm_sync(s, xm);
	s->macc = lmacc;
	s->ca = lca;
	s->icount = icount > 0 ? 0 : icount;
	return ops;
}

#include "tms57002_jit.inc"

// ---- the stream (MAME sound_stream_update, one sample) ----
void tms57002_sample(tms57002_t *s, const int32_t in[4], int32_t out[4], int cycles)
{
	// (MAME: s32(input * 32768 * (SIM ? 256 : 1)), the input a float of full scale 1; in[] is that x 65536)
	for (int k = 0; k < 4; k++) {
		const int32_t v = (s->st0 & ST0_SIM) ? in[k] * 128 : in[k] / 2;
		s->si[k] = (uint32_t)v & 0xffffff;
	}
	for (int k = 0; k < 4; k++)
		out[k] = (int32_t)(s->so[k] << 8);

	sync_w(s);

	if (!s->reset_held) {
		s->icount += cycles;
#if !defined(__vita__) && defined(__linux__)
		// (PC harness, RBX_TMSTIME=1: the time in the runs, printed every 48000 samples)
		static int tm = -1;
		if (tm < 0)
			tm = getenv("RBX_TMSTIME") != NULL;
		if (tm) {
			static unsigned long long ns, n, jn;
			struct timespec t0, t1;
			clock_gettime(CLOCK_MONOTONIC, &t0);
			const int j = tms_jit_run(s);
			if (!j)
				execute_run(s);
			clock_gettime(CLOCK_MONOTONIC, &t1);
			ns += (unsigned long long)((t1.tv_sec - t0.tv_sec) * 1000000000LL + (t1.tv_nsec - t0.tv_nsec));
			jn += (unsigned)j;
			if (++n % 48000 == 0)
				fprintf(stderr, "TMSTIME %llu samples %.3f s (%.2f us/sample), compiled %llu\n", n, ns / 1e9,
					ns / 1e3 / (double)n, jn);
			return;
		}
#endif
		if (!tms_jit_run(s))
			execute_run(s);
	} else
		s->icount = 0;
}

void tms57002_init(tms57002_t *s, uint8_t *data, uint32_t data_size)
{
	memset(s, 0, sizeof(*s));
	s->data = data;
	s->data_size = data_size;
	s->sti = S_IDLE;   // (device_start)
	if (!s_ops) {
		s_ops = execute_run(NULL);
#define SP(n) s_spec[n] = 1;
#define CINTRPSPECIAL
#include "tms57002.hxx"
#undef CINTRPSPECIAL
#undef SP
	}
	s->xsent.lbl = s_ops[OP_DECODE];
	s->xsent.next = &s->xsent;
	s->h.cm_key = CM_NONE;   // (cm_slow all 0)
	tms57002_reset(s);
#if defined(__arm__) && !defined(__vita__)
	static int syn;   // (the compiled code's test: tms57002_jit.inc)
	if (!syn && getenv("RBX_TMSSYN")) {
		syn = 1;
		tj_syn_test(atoi(getenv("RBX_TMSSYN")));
	}
#endif
}

int tms57002_state(tms57002_t *s, void *ctx, int load)
{
	const int rc = rbx_state_io(ctx, "tms57002", &s->macc, (uint32_t)(offsetof(tms57002_t, data) - offsetof(tms57002_t, macc)));
	if (load && !rc)   // (the threaded cache from the loaded one)
		xop_rebuild(s);
	return rc;
}
