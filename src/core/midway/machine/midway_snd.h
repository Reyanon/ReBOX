/*  The Bally Midway sound boards of the MCR boards, ported to C from MAME src/mame/bally/midway_sound.cpp (SSIO, Sounds
    Good, Turbo Cheap Squeak; BSD-3-Clause, copyright-holders: Aaron Giles), src/mame/bally/csd.cpp (Cheap Squeak
    Deluxe; BSD-3-Clause, copyright-holders: Aaron Giles) and src/mame/shared/ballysound.cpp (Squawk & Talk;
    BSD-3-Clause, copyright-holders: Aaron Giles), their DAC filters as src/devices/sound/flt_biquad.cpp (BSD-3-Clause,
    copyright-holders: K.Wilkins, Couriersud, Derrick Renaud, Frank Palazzolo, Jonathan Gevaryahu).
      SSIO: Z80 at 2 MHz, two AY-8910 at 2 MHz whose voices are scaled by the duty cycle of the PROM counters, the
            14024's /SINT, four input latches and the status latch; the main board's I/O ports are the board's.
      TCS:  MC6809E at 2 MHz, 6821 PIA, 10-bit DAC behind the three LM359 filters.
      SG:   68000 at 8 MHz (the cpu/m68k_snd instance), 6821 PIA, the same DAC and filters.
      CSD:  68000 at 8 MHz (the cpu/m68k_snd instance), 6821 PIA, the same DAC and filters.
      S&T:  M6802 at 3.579545 MHz (internal / 4), two PIAs, AD558 DAC (high pass), TMS5200 (sound/tms5220).
      BG:   the Williams D-11581 background board of src/mame/shared/s11c_bg.cpp (BSD-3-Clause, copyright-holders:
            Barry Rodewald, Jonathan Gevaryahu): MC6809E at 2 MHz, YM2151, PIA, MC1408 DAC, HC-55516.
    Time base: each board runs to a time given in its own CPU's clocks (msnd_*_run), the main board calling once per
    scanline (MAME's synchronize()d writes land at the board's current time, the scanline's start). Every board puts
    its output at MSND_RATE into its own FIFO; msnd_mix takes what all of them have.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_MIDWAY_SND_H
#define RBX_MIDWAY_SND_H

#include <stdint.h>

#define MSND_RATE 50000
#define MSND_FIFO 64

typedef struct {
	int32_t n, pad_;
	int32_t l[MSND_FIFO], r[MSND_FIFO];
} msnd_fifo_t;

// ---- SSIO ----
int msnd_ssio_init(const uint8_t *cpu_rom, uint32_t cpu_size, const uint8_t *prom, float gain);
void msnd_ssio_reset(void);
void msnd_ssio_write(int offset, uint8_t data);   // the input latches (main I/O 0x1c-0x1f)
uint8_t msnd_ssio_status(void);                  // (main I/O 0x07)
void msnd_ssio_reset_write(int state);
void msnd_ssio_suspend(int on);                  // (MAME suspend_cpu: Demolition Derby's SSIO has no program)
void msnd_ssio_set_dip(uint8_t v);              // the board's DIP switches ("ssio:DIP")
void msnd_ssio_run(int64_t cycles);              // up to that many sound CPU clocks since reset
int64_t msnd_ssio_now(void);
msnd_fifo_t *msnd_ssio_fifo(void);

// ---- Turbo Cheap Squeak ----
int msnd_tcs_init(const uint8_t *cpu_rom, uint32_t cpu_size, float gain);
void msnd_tcs_reset(void);
void msnd_tcs_write(uint8_t data);
uint8_t msnd_tcs_read(void);
void msnd_tcs_reset_write(int state);
void msnd_tcs_run(int64_t cycles);
int64_t msnd_tcs_now(void);
msnd_fifo_t *msnd_tcs_fifo(void);

// ---- Sounds Good / Cheap Squeak Deluxe (one 68000 board at a time) ----
enum { MSND_68K_SG, MSND_68K_CSD };
int msnd_68k_init(int type, const uint8_t *cpu_rom, uint32_t cpu_size, float gain);   // cpu_rom: the region's bytes
void msnd_68k_exit(void);
void msnd_68k_reset(void);
void msnd_68k_write(uint8_t data);          // SG: the 5-bit latch (MAME write)
void msnd_68k_sr_w(uint8_t data);           // CSD: sr_w (4 bits) and sirq_w
void msnd_68k_sirq_w(int state);
uint8_t msnd_68k_read(void);                // SG read / CSD stat_r
void msnd_68k_reset_write(int state);
void msnd_68k_suspend(int on);
void msnd_68k_run(int64_t cycles);
int64_t msnd_68k_now(void);
msnd_fifo_t *msnd_68k_fifo(void);

// ---- Squawk & Talk ----
int msnd_snt_init(const uint8_t *cpu_rom, uint32_t cpu_size, float gain);
void msnd_snt_reset(void);
void msnd_snt_select(uint8_t data);
void msnd_snt_int(int state);
void msnd_snt_run(int64_t cycles);
int64_t msnd_snt_now(void);
msnd_fifo_t *msnd_snt_fifo(void);

// ---- Williams D-11581 background board (MCR-68k: Arch Rivals, Pigskin, Tri-Sports) ----
enum { MSND_BG_S11, MSND_BG_S11C };
int msnd_bg_init(int type, const uint8_t *cpu_rom, uint32_t cpu_size, float gain);   // (rbx_ym chip 0)
void msnd_bg_exit(void);
void msnd_bg_reset(void);
void msnd_bg_data_w(uint8_t data);
void msnd_bg_ctrl_w(int state);
void msnd_bg_extra_w(int state);
void msnd_bg_resetq_w(int state);
void msnd_bg_run(int64_t cycles);
int64_t msnd_bg_now(void);

// the boards in use (MSND_B_*): their FIFOs mixed, a DC block, out to rbx_audio (stereo)
#define MSND_B_SSIO 1
#define MSND_B_TCS  2
#define MSND_B_68K  4
#define MSND_B_SNT  8
#define MSND_B_BG   16
void msnd_mix(int boards);
void msnd_mix_reset(void);
int msnd_state(int boards, void *ctx, int load);

#endif
