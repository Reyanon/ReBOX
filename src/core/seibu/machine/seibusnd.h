/*  Seibu Sound System: the Z80 (its ROM optionally SEI80BU-encrypted, a 32 KB bank at 0x8000), the main CPU's latches
    and the RST 10 / RST 18 interrupt logic, the FM chip (YM3812 or YM2151 through rbx_ym) and the OKI MSM6295, mixed to
    one mono stream at the FM chip's rate.
    Ported from MAME 0.289 src/mame/shared/seibusound.cpp / seibusound.h (BSD-3-Clause, copyright-holders: Bryan
    McPhail, R. Belmont) and src/mame/seibu/sei80bu.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail, R. Belmont).
    This file is part of ReBOX. GPL-2.0-or-later.

    One instance (the board's). The Z80 runs as a device of the board's scheduler (sched.h).
*/
#ifndef RBX_SEIBUSND_H
#define RBX_SEIBUSND_H

#include <stdint.h>

typedef struct {
	const uint8_t *rom;        // the "audiocpu" region (0x0000-0x7fff fixed, banks of 0x8000 from 0x10000)
	uint32_t rom_size;
	int encrypted;             // SEI80BU (MAME sei80bu_device: data and opcodes decrypted apart)
	int ym_type;               // RBX_YM3812 / RBX_YM2151
	uint32_t ym_clock;
	int ym_gain;               // 1/256
	uint32_t z80_clock;
	const uint8_t *oki_rom;    // NULL: none
	uint32_t oki_size, oki_clock;
	int oki_pin7, oki_gain;
	int coin_port;             // the "COIN" port index (-1: none)
	// Cabal's map (the SEI80BU below 0x2000 only, no bank, no OKI, two ADPCM interfaces); predecrypted: the opcodes
	// below 0x2000 from the ROM at 0x2000 (cabalbl2), none decrypted
	int cabal, predecrypted;
	const uint8_t *adpcm[2];   // (their lines swapped here)
	uint32_t adpcm_size[2];
	int adpcm_gain;            // 1/256, each MSM5205
	int oki_bank_port;         // Godzilla: the Z80's port 0 banks the OKI ROM (256 KB a bank)
	int ym2;                   // Dead Angle: a second FM chip (the same type) at 0x6008, the bank and 0x4000 mapped
	// the Raiden II boards' maps (MAME raiden2.cpp raiden2_sound_map / zeroteam_sound_map): the bank at 0x401a, 0x4007
	// unmapped; Raiden II's second OKI at 0x6002 (the same clock and pin 7)
	int r2map;
	const uint8_t *oki2_rom;   // NULL: none
	uint32_t oki2_size;
	int enc_low;               // Air Raid: the SEI80BU below 0x2000 only, the ROM at 0x8000 plain (the normal map)
	// the FM chips' samples made on the rbx_aux worker (rbx_ym_set_async), added at the frame's end: the same sound.
	// For boards that use rbx_aux for nothing else
	int ym_async;
} seibusnd_config;

// the board's scheduler set up with its CPUs first (sched_init, sched_add_exec): the Z80 added after them. 0 = ok
// (sets rbx_audio up at the FM rate)
int seibusnd_init(const seibusnd_config *cfg);
void seibusnd_exit(void);
void seibusnd_reset(void);
// the main CPU's side (seibu_sound_device main_r / main_w, offsets 0..6)
uint8_t seibusnd_main_r(int offset);
void seibusnd_main_w(int offset, uint8_t data);
// the scheduler's timers of ids SEIBUSND_TIMER_* go here (1 = taken)
#define SEIBUSND_TIMER_SYNC 100
#define SEIBUSND_TIMER_YM 101    // (and 102)
#define SEIBUSND_TIMER_VCK 103   // (Cabal)
#define SEIBUSND_TIMER_CAP 104   // (and 105)
#define SEIBUSND_TIMER_YM2 106   // (and 107: Dead Angle's second FM chip)
int seibusnd_fire(int id, int32_t param);
// the frame's end (frame_as attoseconds long, the board's scheduler at it): the sound made handed to rbx_audio
void seibusnd_frame_end(int64_t frame_as);
int seibusnd_state(void *ctx, int load);
// lazy synchronization (after seibusnd_init; seibusnd_exit ends it): the Z80 runs ahead of the scheduler, below the
// first timer and below peer() (the attoseconds the board's CPU has really run to: it must stop itself before any
// access to the latches), the scheduler's slices replayed from its records: the same results as the tight run
// mq (NULL: none): the peer's own run-ahead queue (scheduler device 0, this Z80 device 1): its z80_run iterations'
// cycles from f[h & mask] to f[t & mask], pend their sum, consumed as its run would (while the cycles done are below the
// slice's; it refills itself when emptied). With it the scheduler's slices are replayed here while both have records.
typedef struct {
	uint8_t *f;
	uint32_t h, t, mask;
	unsigned long pend;
} seibusnd_mainq;
void seibusnd_set_lazy(int on, int64_t (*peer)(void), seibusnd_mainq *mq);
int seibusnd_lazy_pending(void);   // (records not replayed: none at a frame's end)
// the SEI80BU's decryption of a Z80 image from address 0 (sei80bu_device data_r / opcode_r) for a CPU other than the
// sound one (Mustache Boy's main Z80): data / op may be NULL, either may be src
void seibusnd_sei80bu_decrypt(const uint8_t *src, uint8_t *data, uint8_t *op, uint32_t len);

// (-DRBX_FG_PROF: Fail Gate's per-frame work counters, written by airraid.c to RBX_FG_PROF=file; none otherwise)
#ifdef RBX_FG_PROF
extern unsigned long rbx_fgprof[20];
#define FGP(i, n) (rbx_fgprof[i] += (unsigned long)(n))
unsigned long long rbx_fgp_ins(void);
extern unsigned rbx_fgp_mpc[65536], rbx_fgp_spc[65536];   // (the frame's PCs run ahead: main, sound)
   // (x86 Linux: this thread's instructions, 0 elsewhere)
#else
#define FGP(i, n) ((void)0)
#endif

#endif
