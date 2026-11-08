/*  Taito MC68705 MCU interfaces, ported from MAME src/mame/shared/taito68705.cpp
    (BSD-3-Clause, copyright-holders: David Haywood, Vas Crabb).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "taito68705.h"

#include <string.h>

static int is_ark(const taito68705_t *t) { return t->kind == TAITO68705_ARK_P3 || t->kind == TAITO68705_ARK_P5; }

static uint8_t pa_value(const taito68705_t *t)
{
	return (uint8_t)(t->st.pa_output & (t->st.latch_driven ? t->st.host_latch : 0xff));
}

static void semaphore(taito68705_t *t, int state)
{
	if (t->semaphore_cb)
		t->semaphore_cb(t->user, state);
}

uint8_t taito68705_data_r(taito68705_t *t)
{
	uint8_t r = t->st.mcu_latch;
	t->st.mcu_flag = 0;
	semaphore(t, 0);
	return r;
}

void taito68705_data_w(taito68705_t *t, uint8_t data)
{
	if (!t->st.reset_input)
		t->st.host_flag = 1;
	t->st.host_latch = data;
	if (t->st.latch_driven)
		m68705_port_input(&t->mcu, 0, data);
	m68705_set_irq_line(&t->mcu, t->st.host_flag);
}

void taito68705_reset_w(taito68705_t *t, int state)
{
	t->st.reset_input = state ? 1 : 0;
	if (state) {
		t->st.host_flag = 0;
		t->st.mcu_flag = 0;
		m68705_set_irq_line(&t->mcu, 0);
	}
	m68705_set_reset_line(&t->mcu, state);
}

int taito68705_host_semaphore_r(const taito68705_t *t) { return t->st.host_flag ? 1 : 0; }
int taito68705_mcu_semaphore_r(const taito68705_t *t) { return t->st.mcu_flag ? 1 : 0; }

static void latch_control(taito68705_t *t, uint8_t data, uint8_t *value, int host_bit, int mcu_bit)
{
	// save this here to simulate latch propagation delays
	uint8_t old_pa_value = pa_value(t);

	// rising edge clears the host semaphore flag
	if ((data >> host_bit) & 1) {
		t->st.latch_driven = 0;
		m68705_port_input(&t->mcu, 0, 0xff);
		if (!((*value >> host_bit) & 1)) {
			t->st.host_flag = 0;
			m68705_set_irq_line(&t->mcu, 0);
		}
	} else {
		t->st.latch_driven = 1;
		m68705_port_input(&t->mcu, 0, t->st.host_latch);
	}

	// the MCU bit sets the MCU semaphore when low
	if (!((data >> mcu_bit) & 1)) {
		if (!t->st.reset_input)
			t->st.mcu_flag = 1;
		// data is latched on falling edge
		if ((*value >> mcu_bit) & 1)
			t->st.mcu_latch = old_pa_value;
	}

	*value = data;
	if (!((data >> mcu_bit) & 1) && !t->st.reset_input)
		semaphore(t, 1);
}

// ---- MCU port callbacks ----
static uint8_t mcu_port_r(void *user, int n, uint8_t mask)
{
	taito68705_t *t = (taito68705_t *)user;
	(void)mask;
	if (n == 2) {
		// PC0 host semaphore flag (active high), PC1 MCU semaphore flag (active low)
		uint8_t v = (uint8_t)((t->st.host_flag ? 0x01 : 0x00) | (t->st.mcu_flag ? 0x00 : 0x02) | 0xfc);
		if (t->kind == TAITO68705_TIGER)
			v ^= 0x03;
		return v;
	}
	if (n == 1 && is_ark(t))
		return t->portb_r_cb ? t->portb_r_cb(t->user) : 0xff;
	return 0xff;
}

static void mcu_port_w(void *user, int n, uint8_t data, uint8_t mask)
{
	taito68705_t *t = (taito68705_t *)user;
	if (n == 0) {
		t->st.pa_output = data;
	} else if (n == 1 && !is_ark(t)) {
		uint8_t old_pa_value = pa_value(t);
		uint8_t aux_changed = (uint8_t)((data ^ t->st.pb_output) >> 2);
		uint8_t aux_strobes = (uint8_t)((mask & ~data & t->st.pb_output) >> 2);
		// rising edge on PB1 clears the host semaphore flag, PB2 sets the MCU semaphore when low
		latch_control(t, data, &t->st.pb_output, 1, 2);
		for (int i = 0; i < 6; i++) {
			if (((aux_changed >> i) & 1) && t->aux_out_cb)
				t->aux_out_cb(t->user, i, (data >> (i + 2)) & 1);
			if (((aux_strobes >> i) & 1) && t->aux_strobe_cb)
				t->aux_strobe_cb(t->user, i, old_pa_value);
		}
	} else if (n == 2 && is_ark(t)) {
		// rising edge on PC2 clears the host semaphore flag, PC3 sets the MCU semaphore when low
		latch_control(t, data, &t->st.pc_output, 2, 3);
	}
}

int taito68705_init(taito68705_t *t, int kind, uint8_t *rom, uint32_t size)
{
	void *user = t->user;
	void (*sem)(void *, int) = t->semaphore_cb;
	void (*aux)(void *, int, int) = t->aux_out_cb;
	void (*strobe)(void *, int, uint8_t) = t->aux_strobe_cb;
	uint8_t (*pb)(void *) = t->portb_r_cb;
	memset(t, 0, sizeof(*t));
	t->user = user;
	t->semaphore_cb = sem;
	t->aux_out_cb = aux;
	t->aux_strobe_cb = strobe;
	t->portb_r_cb = pb;
	t->kind = kind;
	if (m68705_init(&t->mcu, kind == TAITO68705_ARK_P3 ? M68705_P3 : M68705_P5, rom, size))
		return -1;
	t->mcu.user = t;
	t->mcu.port_w[0] = mcu_port_w;
	t->mcu.port_r[2] = mcu_port_r;
	if (is_ark(t)) {
		t->mcu.port_r[1] = mcu_port_r;
		t->mcu.port_w[2] = mcu_port_w;
	} else {
		t->mcu.port_w[1] = mcu_port_w;
	}
	// device_start
	t->st.host_latch = 0xff;
	t->st.mcu_latch = 0xff;
	t->st.pa_output = 0xff;
	t->st.pb_output = 0xff;
	t->st.pc_output = 0xff;
	return 0;
}

void taito68705_reset(taito68705_t *t)
{
	// device_reset of the interface, then the MCU
	t->st.host_flag = 0;
	t->st.mcu_flag = 0;
	m68705_set_irq_line(&t->mcu, 0);
	semaphore(t, 0);
	m68705_reset(&t->mcu);
}
