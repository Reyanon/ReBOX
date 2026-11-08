/*  Taito MC68705 MCU interfaces, ported from MAME src/mame/shared/taito68705.cpp/.h
    (BSD-3-Clause, copyright-holders: David Haywood, Vas Crabb).
    TAITO68705_STD   taito68705_mcu_device (M68705P5; PB1/PB2 latch control, PC0/PC1 semaphores): Tokio ...
    TAITO68705_TIGER taito68705_mcu_tiger_device (status bits on port C inverted)
    TAITO68705_ARK_P3 / _P5 arkanoid_68705p3/p5_device (latch control on PC2/PC3, port B inputs from the host)
    Save state: `st` and `mcu.st` are plain data (memcpy).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TAITO68705_H
#define RBX_TAITO68705_H

#include "../../cpu/m6805/m68705.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { TAITO68705_STD, TAITO68705_TIGER, TAITO68705_ARK_P3, TAITO68705_ARK_P5 };

typedef struct {
	uint8_t latch_driven, reset_input, host_flag, mcu_flag;
	uint8_t host_latch, mcu_latch, pa_output, pb_output, pc_output;
} taito68705_state;

typedef struct {
	m68705_t mcu;
	taito68705_state st;
	int kind;
	// host side callbacks (may be NULL)
	void *user;
	void (*semaphore_cb)(void *user, int state);              // MCU semaphore line (ARK: semaphore_cb)
	void (*aux_out_cb)(void *user, int n, int state);          // STD/TIGER: PB2+n level changes (n = 0..5)
	void (*aux_strobe_cb)(void *user, int n, uint8_t data);    // STD/TIGER: falling edge on PB2+n, data = port A value
	uint8_t (*portb_r_cb)(void *user);                         // ARK: port B inputs (default 0xff)
} taito68705_t;

// rom: the MCU EPROM image (0x800 bytes), must stay valid
int taito68705_init(taito68705_t *t, int kind, uint8_t *rom, uint32_t size);
void taito68705_reset(taito68705_t *t);                 // device reset (interface, then the MCU)
uint8_t taito68705_data_r(taito68705_t *t);             // host reads the MCU latch, clears the MCU semaphore
void taito68705_data_w(taito68705_t *t, uint8_t data);  // host writes the latch, sets the host semaphore and /INT
void taito68705_reset_w(taito68705_t *t, int state);    // host drives the MCU RESET line (1 = held)
int taito68705_host_semaphore_r(const taito68705_t *t);
int taito68705_mcu_semaphore_r(const taito68705_t *t);
// internal cycles (clock / 4); returns cycles spent
static inline int taito68705_execute(taito68705_t *t, int cycles) { return m68705_execute(&t->mcu, cycles); }

#ifdef __cplusplus
}
#endif

#endif
