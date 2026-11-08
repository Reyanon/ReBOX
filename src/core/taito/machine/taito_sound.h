/*  The Taito sound board shared by the B / X / F2 / Z and many other drivers: Z80 with banked ROM, one Yamaha FM chip,
    TC0140SYT / PC060HA communication and an optional OKI M6295. Memory maps follow MAME (taito_b.cpp sound_map,
    masterw_sound_map, viofight_sound_map; taito_x.cpp sound_map, daisenpu_sound_map ...).
    The board runs it one scanline at a time after its main CPU (tsnd_line); chips advance with the Z80 so their
    timers and IRQ follow the program, and output is mixed to mono at the chip rate.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TAITO_SOUND_H
#define RBX_TAITO_SOUND_H

#include <stdint.h>
#include "taitosnd.h"
#include "../../sound/rbx_ym.h"

enum {
	TSND_MAP_E000,       // c000-dfff RAM, e000 YM (4 ports YM2610, 2 ports YM2151 / YM2203), e200/e201 comm, f200 bank
	TSND_MAP_9000,       // 8000-8fff RAM, 9000 YM2203, a000/a001 comm, b000 OKI; bank from YM2203 port A
	TSND_MAP_8800,       // volfied.cpp z80_map: 0000-7fff ROM, 8000-87ff RAM, 8800/8801 comm, 9000 YM (no bank)
};

typedef struct {
	int map;
	uint32_t z80_clock;
	rbx_ym_config ym;          // irq / port_a_write are set by the sound board
	uint32_t oki_clock;        // 0 = none
	int oki_pin7_high, oki_gain;
	uint32_t frame_rate_mhz;   // video refresh in milli-Hz (60000 = 60 Hz)
	int lines;                 // scanlines per frame
	// optional (zero = off), for the rastan / opwolf / rbisland family (TSND_MAP_9000 with a YM2151):
	int ym_port_bank;          // ROM bank from the YM port (YM2151 CT1/CT2 pins), as YM2203 port A
	uint32_t bank_base;        // offset of bank 0 in the audiocpu region (rbisland: 0xc000)
	void (*ext_write)(uint16_t a, uint8_t v);   // Z80 writes at b000-ffff (map 9000, no OKI) or other e000-ffff
	                                            // writes (map E000); chips synced first
	void (*ext_mix)(int32_t *buf, int n, uint32_t rate);   // add other chips to the n samples just produced
	int bank_mask;             // f200 bank write mask (0 = 3; the F2 sound board decodes 3 bits)
	int rom_flat;              // 1 = 0000-7fff plain ROM without banking (F2 cameltrya)
	uint32_t bank0_offset;     // nonzero: audiocpu offset of bank 0 (topspeed: the erased area after the ROM)
	uint8_t (*port_in)(uint8_t port);            // Z80 I/O space (NULL: reads 0xff, writes ignored); chips synced first
	void (*port_out)(uint8_t port, uint8_t v);
	int stereo;                // 1 = the YM's two sides kept apart (MAME stereo routes; the OKI and ext_mix to both)
} tsnd_config;
// let the chips catch up with the Z80 (for ext_write side effects on volume etc.)
void tsnd_sync(void);

int tsnd_init(const tsnd_config *cfg);   // audiocpu / ymsnd:adpcma / adpcmb / oki regions from the set
void tsnd_exit(void);
void tsnd_reset(void);
tc0140syt_t *tsnd_syt(void);             // main side of the communication chip
void tsnd_line(void);                    // run one scanline of Z80 and chips
void tsnd_frame_end(void);               // hand the frame's samples to the core
int tsnd_state(void *ctx, int load);
// clocks of a CPU for the next scanline (fractions carried); frac belongs to the caller, zeroed at reset
int tsnd_line_cycles(uint64_t *frac, uint32_t clock);

#endif
