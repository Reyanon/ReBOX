/*  TI TMS5220 / TMS5220C LPC speech synthesizer, ported to C from MAME src/devices/sound/tms5220.cpp
    (BSD-3-Clause, copyright-holders: Frank Palazzolo, Aaron Giles, Jonathan Gevaryahu, Raphael Nabet, Couriersud,
    Michael Zapf) and its coefficient table from src/devices/sound/tms5110r.hxx (BSD-3-Clause, copyright-holders:
    Frank Palazzolo, Couriersud, Jonathan Gevaryahu).
    MAME's build options kept: INTERP_SHIFT as a shift, FAST_START_HACK on, FORCE_DIGITAL 0,
    FORCE_SUBC_RELOAD 1, no TMS5220_PERFECT_INTERPOLATION_HACK, no ALLOW_4_LSB.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tms5220.h"
#include "../rbx.h"

#include <stddef.h>
#include <string.h>

#define FORCE_DIGITAL     0
#define FORCE_SUBC_RELOAD 1
#define SAMPLE_CLOCKS     80
#define NOCOMMAND         0xff

#define HAS_RATE_CONTROL(t) ((t)->variant == TMS5220_VARIANT_5220C)

// sample count reload for 5220c only; 5220 always reloads with 0 (its c_variant_rate stays 0)
static const uint8_t reload_table[4] = { 0, 2, 4, 6 };

// ---- tms5220_coeff of tms5110r.hxx (TMS5220 and TMS5220C share it) ----
#define NUM_K       10
#define ENERGY_BITS 4
#define PITCH_BITS  6
static const int kbits[NUM_K] = { 5, 5, 4, 4, 4, 4, 4, 3, 3, 3 };
// TI_028X_LATER_ENERGY
static const uint16_t energytable[16] = {
	0,  1,  2,  3,  4,  6,  8, 11,
	16, 23, 33, 47, 63, 85, 114, 0
};
// TI_5220_PITCH
static const uint16_t pitchtable[64] = {
	0,  15,  16,  17,  18,  19,  20,  21,
	22,  23,  24,  25,  26,  27,  28,  29,
	30,  31,  32,  33,  34,  35,  36,  37,
	38,  39,  40,  41,  42,  44,  46,  48,
	50,  52,  53,  56,  58,  60,  62,  65,
	68,  70,  72,  76,  78,  80,  84,  86,
	91,  94,  98, 101, 105, 109, 114, 118,
	122, 127, 132, 137, 142, 148, 153, 159
};
// TI_5110_5220_LPC
static const int16_t ktable[NUM_K][32] = {
	// K1
	{ -501, -498, -497, -495, -493, -491, -488, -482,
	  -478, -474, -469, -464, -459, -452, -445, -437,
	  -412, -380, -339, -288, -227, -158,  -81,   -1,
	    80,  157,  226,  287,  337,  379,  411,  436 },
	// K2
	{ -328, -303, -274, -244, -211, -175, -138,  -99,
	   -59,  -18,   24,   64,  105,  143,  180,  215,
	   248,  278,  306,  331,  354,  374,  392,  408,
	   422,  435,  445,  455,  463,  470,  476,  506 },
	// K3
	{ -441, -387, -333, -279, -225, -171, -117,  -63,
	    -9,   45,   98,  152,  206,  260,  314,  368 },
	// K4
	{ -328, -273, -217, -161, -106,  -50,    5,   61,
	   116,  172,  228,  283,  339,  394,  450,  506 },
	// K5
	{ -328, -282, -235, -189, -142,  -96,  -50,   -3,
	    43,   90,  136,  182,  229,  275,  322,  368 },
	// K6
	{ -256, -212, -168, -123,  -79,  -35,   10,   54,
	    98,  143,  187,  232,  276,  320,  365,  409 },
	// K7
	{ -308, -260, -212, -164, -117,  -69,  -21,   27,
	    75,  122,  170,  218,  266,  314,  361,  409 },
	// K8
	{ -256, -161,  -66,   29,  124,  219,  314,  409 },
	// K9
	{ -256, -176,  -96,  -15,   65,  146,  226,  307 },
	// K10
	{ -205, -132,  -59,   14,   87,  160,  234,  307 },
};
// T0285_2501E_coeff (TMS5200 / CD2501E): TI_2501E_PITCH and TI_2801_2501E_LPC; energy, chirp, interpolation as above
static const uint16_t pitchtable_5200[64] = {
	0,  14,  15,  16,  17,  18,  19,  20,
	21,  22,  23,  24,  25,  26,  27,  28,
	29,  30,  31,  32,  34,  36,  38,  40,
	41,  43,  45,  48,  49,  51,  54,  55,
	57,  60,  62,  64,  68,  72,  74,  76,
	81,  85,  87,  90,  96,  99, 103, 107,
	112, 117, 122, 127, 133, 139, 145, 151,
	157, 164, 171, 178, 186, 194, 202, 211
};
static const int16_t ktable_5200[NUM_K][32] = {
	// K1
	{ -501, -498, -495, -490, -485, -478, -469, -459,
	  -446, -431, -412, -389, -362, -331, -295, -253,
	  -207, -156, -102,  -45,   13,   70,  126,  179,
	   228,  272,  311,  345,  374,  399,  420,  437 },
	// K2
	{ -376, -357, -335, -312, -286, -258, -227, -195,
	  -161, -124,  -87,  -49,  -10,   29,   68,  106,
	   143,  178,  212,  243,  272,  299,  324,  346,
	   366,  384,  400,  414,  427,  438,  448,  506 },
	// K3
	{ -407, -381, -349, -311, -268, -218, -162, -102,
	   -39,   25,   89,  149,  206,  257,  302,  341 },
	// K4
	{ -290, -252, -209, -163, -114,  -62,   -9,   44,
	    97,  147,  194,  238,  278,  313,  344,  371 },
	// K5
	{ -318, -283, -245, -202, -156, -107,  -56,   -3,
	    49,  101,  150,  196,  239,  278,  313,  344 },
	// K6
	{ -193, -152, -109,  -65,  -20,   26,   71,  115,
	   158,  198,  235,  270,  301,  330,  355,  377 },
	// K7
	{ -254, -218, -180, -140,  -97,  -53,   -8,   36,
	    81,  124,  165,  204,  240,  274,  304,  332 },
	// K8
	{ -205, -112,  -10,   92,  187,  269,  336,  387 },
	// K9
	{ -249, -183, -110,  -32,   48,  126,  198,  261 },
	// K10
	{ -190, -133,  -73,  -10,   53,  115,  173,  227 },
};
// TI_LATER_CHIRP
static const int8_t chirptable[52] = {
	0x00, 0x03, 0x0f, 0x28, 0x4c, 0x6c, 0x71, 0x50,
	0x25, 0x26, 0x4c, 0x44, 0x1a, 0x32, 0x3b, 0x13,
	0x37, 0x1a, 0x25, 0x1f, 0x1d, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};
// TI_INTERP
static const int8_t interp_coeff[8] = { 0, 3, 3, 3, 2, 2, 1, 1 };

static void process_command(tms5220_t *t, uint8_t cmd);
static void device_reset(tms5220_t *t);

static inline int talk_status(const tms5220_t *t) { return t->SPEN || t->TALKD; }
static inline int new_frame_stop_flag(const tms5220_t *t) { return t->new_frame_energy_idx == 0x0f; }
static inline int new_frame_silence_flag(const tms5220_t *t) { return t->new_frame_energy_idx == 0; }
static inline int new_frame_unvoiced_flag(const tms5220_t *t) { return t->new_frame_pitch_idx == 0; }

// m_timer_io_ready->adjust(clocks, param): replaces a pending one
static void io_timer_adjust(tms5220_t *t, uint32_t clocks, uint8_t param)
{
	t->io_pending = 1;
	t->io_clocks = clocks;
	t->io_param = param;
}

static void set_interrupt_state(tms5220_t *t, int state)
{
	if (state != t->irq_pin) {
		t->irq_pin = (uint8_t)state;
		if (t->irq_cb)
			t->irq_cb(t->cb_ctx, !state);
	}
}

static void update_ready_state(tms5220_t *t)
{
	const uint8_t state = t->io_ready;
	if (t->ready_pin != state) {
		if (t->ready_cb)
			t->ready_cb(t->cb_ctx, !state);
		t->ready_pin = state;
	}
}

// (no VSM attached: the dummy read only clears its flag)
static void perform_dummy_read(tms5220_t *t) { t->schedule_dummy_read = 0; }

static uint8_t status_read(tms5220_t *t, int clear_int)
{
	if (t->RDB_flag) {
		t->RDB_flag = 0;
		return t->read_byte_register;
	}
	if (clear_int)
		set_interrupt_state(t, 0);
	return (uint8_t)((talk_status(t) << 7) | (t->buffer_low << 6) | (t->buffer_empty << 5));
}

static void update_fifo_status_and_ints(tms5220_t *t)
{
	update_ready_state(t);

	if (t->fifo_count <= 8) {
		if (!t->buffer_low) {
			t->buffer_low = 1;
			set_interrupt_state(t, 1);
		}
	} else
		t->buffer_low = 0;

	if (t->fifo_count == 0) {
		if (!t->buffer_empty) {
			t->buffer_empty = 1;
			set_interrupt_state(t, 1);
		}
		if (t->DDIS)
			t->TALK = t->SPEN = 0;
	} else
		t->buffer_empty = 0;

	if (t->previous_talk_status && !talk_status(t)) {
		set_interrupt_state(t, 1);
		t->DDIS = 0;
		t->previous_talk_status = 0;
		if (t->command_register != NOCOMMAND) {
			process_command(t, t->command_register);
			if (t->data_latched)
				io_timer_adjust(t, 16, 1);
			else
				t->io_ready = 1;
			update_ready_state(t);
		}
	}
	t->previous_talk_status = (uint8_t)talk_status(t);

	if (t->rs_ws == 0x01 && !t->RDB_flag)
		t->read_latch = status_read(t, 0);
}

static void data_write(tms5220_t *t, int data)
{
	const int old_buffer_low = t->buffer_low;
	if (t->DDIS) {
		if (t->fifo_count < TMS5220_FIFO_SIZE) {
			t->fifo[t->fifo_tail] = (uint8_t)data;
			t->fifo_tail = (uint8_t)((t->fifo_tail + 1) % TMS5220_FIFO_SIZE);
			t->fifo_count++;
			update_fifo_status_and_ints(t);
			if (!t->SPEN && (old_buffer_low && !t->buffer_low)) {
				t->zpar = 1;
				t->uv_zpar = 1;
				t->OLDE = 1;
				t->OLDP = 1;
				t->SPEN = 1;
				t->TALK = 1;   // (FAST_START_HACK)
				t->new_frame_energy_idx = 0;
				t->new_frame_pitch_idx = 0;
				for (int i = 0; i < 4; i++)
					t->new_frame_k_idx[i] = 0;
				for (int i = 4; i < 7; i++)
					t->new_frame_k_idx[i] = 0xf;
				for (int i = 7; i < NUM_K; i++)
					t->new_frame_k_idx[i] = 0x7;
			}
		}
		t->data_latched = 0;
	} else
		process_command(t, (uint8_t)data);

	if (!t->data_latched)
		t->io_ready = 1;
}

static int read_bits(tms5220_t *t, int count)
{
	int val = 0;
	if (t->DDIS) {
		while (count--) {
			val = (val << 1) | ((t->fifo[t->fifo_head] >> t->fifo_bits_taken) & 1);
			t->fifo_bits_taken++;
			if (t->fifo_bits_taken >= 8) {
				t->fifo_count--;
				t->fifo[t->fifo_head] = 0;
				t->fifo_head = (uint8_t)((t->fifo_head + 1) % TMS5220_FIFO_SIZE);
				t->fifo_bits_taken = 0;
				update_fifo_status_and_ints(t);
			}
		}
	} else
		val = (1 << count) - 1;   // (no VSM: the input floats high)
	return val;
}

static void zero_new_frame(tms5220_t *t)
{
	t->zpar = 1;
	t->uv_zpar = 1;
	t->OLDE = 1;
	t->OLDP = 1;
	t->new_frame_energy_idx = 0;
	t->new_frame_pitch_idx = 0;
	for (int i = 0; i < 4; i++)
		t->new_frame_k_idx[i] = 0;
	for (int i = 4; i < 7; i++)
		t->new_frame_k_idx[i] = 0xf;
	for (int i = 7; i < NUM_K; i++)
		t->new_frame_k_idx[i] = 0x7;
}

static void process_command(tms5220_t *t, uint8_t cmd)
{
	t->command_register = cmd;
	switch (cmd & 0x70) {
	case 0x10:   // read byte
		if (!talk_status(t)) {
			if (t->schedule_dummy_read) {
				t->schedule_dummy_read = 0;
				perform_dummy_read(t);
			}
			t->read_byte_register = (uint8_t)read_bits(t, 8);
			t->RDB_flag = 1;
			t->command_register = NOCOMMAND;
		}
		break;

	case 0x00:
	case 0x20:   // set rate (5220C), otherwise NOP
		if (HAS_RATE_CONTROL(t))
			t->c_variant_rate = cmd & 0x0f;
		t->command_register = NOCOMMAND;
		break;

	case 0x30:   // read and branch
		if (!talk_status(t)) {
			t->RDB_flag = 0;
			if (t->schedule_dummy_read) {
				t->schedule_dummy_read = 0;
				perform_dummy_read(t);
			}
			t->command_register = NOCOMMAND;
		}
		break;

	case 0x40:   // load address
		if (!talk_status(t)) {
			t->schedule_dummy_read = 1;
			t->command_register = NOCOMMAND;
		}
		break;

	case 0x50:   // speak
		if (t->schedule_dummy_read) {
			t->schedule_dummy_read = 0;
			perform_dummy_read(t);
		}
		t->SPEN = 1;
		t->TALK = 1;   // (FAST_START_HACK)
		t->DDIS = 0;
		zero_new_frame(t);
		t->command_register = NOCOMMAND;
		break;

	case 0x60:   // speak external
		memset(t->fifo, 0, sizeof(t->fifo));
		t->fifo_head = t->fifo_tail = t->fifo_count = t->fifo_bits_taken = 0;
		t->DDIS = 1;
		zero_new_frame(t);
		t->RDB_flag = 0;
		t->command_register = NOCOMMAND;
		break;

	case 0x70:   // reset
		if (t->schedule_dummy_read) {
			t->schedule_dummy_read = 0;
			perform_dummy_read(t);
		}
		device_reset(t);
		t->command_register = NOCOMMAND;
		break;
	}
	update_fifo_status_and_ints(t);
}

static void parse_frame(tms5220_t *t)
{
	int i, rep_flag;
	t->uv_zpar = t->zpar = 0;

	if (HAS_RATE_CONTROL(t) && (t->c_variant_rate & 0x04)) {
		i = read_bits(t, 2);
		t->IP = reload_table[i];
	} else
		t->IP = reload_table[t->c_variant_rate & 0x3];

	update_fifo_status_and_ints(t);
	if (t->DDIS && t->buffer_empty)
		return;

	t->new_frame_energy_idx = (uint8_t)read_bits(t, ENERGY_BITS);
	update_fifo_status_and_ints(t);
	if (t->DDIS && t->buffer_empty)
		return;
	if (t->new_frame_energy_idx == 0 || t->new_frame_energy_idx == 15)
		return;

	rep_flag = read_bits(t, 1);
	t->new_frame_pitch_idx = (uint8_t)read_bits(t, PITCH_BITS);
	t->uv_zpar = (uint8_t)new_frame_unvoiced_flag(t);
	update_fifo_status_and_ints(t);
	if (t->DDIS && t->buffer_empty)
		return;
	if (rep_flag)
		return;

	for (i = 0; i < 4; i++) {
		t->new_frame_k_idx[i] = (uint8_t)read_bits(t, kbits[i]);
		update_fifo_status_and_ints(t);
		if (t->DDIS && t->buffer_empty)
			return;
	}
	if (t->new_frame_pitch_idx == 0)
		return;
	for (i = 4; i < NUM_K; i++) {
		t->new_frame_k_idx[i] = (uint8_t)read_bits(t, kbits[i]);
		update_fifo_status_and_ints(t);
		if (t->DDIS && t->buffer_empty)
			return;
	}
}

static int16_t clip_analog(int16_t cliptemp)
{
	if (cliptemp > 2047)
		cliptemp = 2047;
	else if (cliptemp < -2048)
		cliptemp = -2048;
	cliptemp &= ~0xf;
	return (int16_t)((cliptemp << 4) | ((cliptemp & 0x7f0) >> 3) | ((cliptemp & 0x400) >> 10));
}

static inline int32_t matrix_multiply(int32_t a, int32_t b)
{
	while (a > 511) a -= 1024;
	while (a < -512) a += 1024;
	while (b > 16383) b -= 32768;
	while (b < -16384) b += 32768;
	return (a * b) >> 9;
}

static int32_t lattice_filter(tms5220_t *t)
{
	int32_t *u = t->u, *x = t->x;
	const int16_t *k = t->current_k;
	u[10] = matrix_multiply(t->previous_energy, t->excitation_data * 64);
	u[9] = u[10] - matrix_multiply(k[9], x[9]);
	u[8] = u[9] - matrix_multiply(k[8], x[8]);
	u[7] = u[8] - matrix_multiply(k[7], x[7]);
	u[6] = u[7] - matrix_multiply(k[6], x[6]);
	u[5] = u[6] - matrix_multiply(k[5], x[5]);
	u[4] = u[5] - matrix_multiply(k[4], x[4]);
	u[3] = u[4] - matrix_multiply(k[3], x[3]);
	u[2] = u[3] - matrix_multiply(k[2], x[2]);
	u[1] = u[2] - matrix_multiply(k[1], x[1]);
	u[0] = u[1] - matrix_multiply(k[0], x[0]);
	x[9] = x[8] + matrix_multiply(k[8], u[8]);
	x[8] = x[7] + matrix_multiply(k[7], u[7]);
	x[7] = x[6] + matrix_multiply(k[6], u[6]);
	x[6] = x[5] + matrix_multiply(k[5], u[5]);
	x[5] = x[4] + matrix_multiply(k[4], u[4]);
	x[4] = x[3] + matrix_multiply(k[3], u[3]);
	x[3] = x[2] + matrix_multiply(k[2], u[2]);
	x[2] = x[1] + matrix_multiply(k[1], u[1]);
	x[1] = x[0] + matrix_multiply(k[0], u[0]);
	x[0] = u[0];
	t->previous_energy = (uint16_t)t->current_energy;
	return u[0];
}

// one sample of MAME's process()
static int16_t gen_sample(tms5220_t *t)
{
	int16_t out;
	if (t->TALKD) {
		if (t->IP == 0 && t->PC == 12 && t->subcycle == 1) {
			t->IP = reload_table[t->c_variant_rate & 0x3];
			parse_frame(t);
			if (new_frame_stop_flag(t)) {
				t->TALK = t->SPEN = 0;
				update_fifo_status_and_ints(t);
			}
			if ((!t->OLDP && new_frame_unvoiced_flag(t)) || (t->OLDP && !new_frame_unvoiced_flag(t)) ||
			    (t->OLDE && !new_frame_silence_flag(t)) || (t->OLDP && new_frame_silence_flag(t)))
				t->inhibit = 1;
			else
				t->inhibit = 0;
		} else {
			const int inhibit_state = t->inhibit && t->IP != 0;
			if (t->subcycle == 2) {
				const int sh = interp_coeff[t->IP];
				switch (t->PC) {
				case 0:
					if (t->IP == 0)
						t->pitch_zero = 0;
					t->current_energy = (int16_t)((t->current_energy +
					    (((energytable[t->new_frame_energy_idx] - t->current_energy) * (1 - inhibit_state)) >> sh)) *
					    (1 - t->zpar));
					break;
				case 1:
					t->current_pitch = (int16_t)((t->current_pitch +
					    (((t->pitch_tab[t->new_frame_pitch_idx] - t->current_pitch) * (1 - inhibit_state)) >> sh)) *
					    (1 - t->zpar));
					break;
				case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9: case 10: case 11: {
					const int i = t->PC - 2;
					t->current_k[i] = (int16_t)((t->current_k[i] +
					    (((t->k_tab[i][t->new_frame_k_idx[i]] - t->current_k[i]) * (1 - inhibit_state)) >> sh)) *
					    (1 - (i < 4 ? t->zpar : t->uv_zpar)));
					break;
				}
				default:
					break;
				}
			}
		}

		if (t->OLDP) {
			if (t->RNG & 1)
				t->excitation_data = ~0x3f;
			else
				t->excitation_data = 0x40;
		} else {
			if (t->pitch_count >= 51)
				t->excitation_data = chirptable[51];
			else
				t->excitation_data = chirptable[t->pitch_count];
		}

		for (int i = 0; i < 20; i++) {
			const int bitout = ((t->RNG >> 12) & 1) ^ ((t->RNG >> 3) & 1) ^ ((t->RNG >> 2) & 1) ^ (t->RNG & 1);
			t->RNG = (uint16_t)((t->RNG << 1) | bitout);
		}
		int32_t this_sample = lattice_filter(t);

		while (this_sample > 16383) this_sample -= 32768;
		while (this_sample < -16384) this_sample += 32768;
		if (t->digital_select == 0)
			out = clip_analog((int16_t)this_sample);
		else {
			this_sample &= ~0xf;
			out = (int16_t)((this_sample << 1) | ((this_sample & 0x3e00) >> 9));
		}

		t->subcycle++;
		if (t->subcycle == 2 && t->PC == 12) {
			if (t->IP == 7 && t->inhibit)
				t->pitch_zero = 1;
			if (t->IP == 7) {
				t->OLDE = (uint8_t)new_frame_silence_flag(t);
				t->OLDP = (uint8_t)new_frame_unvoiced_flag(t);
				t->TALKD = t->TALK;
				update_fifo_status_and_ints(t);
				if (!t->TALK && t->SPEN)
					t->TALK = 1;
			}
			t->subcycle = t->subc_reload;
			t->PC = 0;
			t->IP = (t->IP + 1) & 0x7;
		} else if (t->subcycle == 3) {
			t->subcycle = t->subc_reload;
			t->PC++;
		}
		t->pitch_count++;
		if (t->pitch_count >= t->current_pitch || t->pitch_zero)
			t->pitch_count = 0;
		t->pitch_count &= 0x1ff;
	} else {
		t->subcycle++;
		if (t->subcycle == 2 && t->PC == 12) {
			if (t->IP == 7) {
				t->TALKD = t->TALK;
				update_fifo_status_and_ints(t);
				if (!t->TALK && t->SPEN)
					t->TALK = 1;
			}
			t->subcycle = t->subc_reload;
			t->PC = 0;
			t->IP = (t->IP + 1) & 0x7;
		} else if (t->subcycle == 3) {
			t->subcycle = t->subc_reload;
			t->PC++;
		}
		out = -1;
	}
	return out;
}

// ---- reset / timer ----
static void device_reset(tms5220_t *t)
{
	t->digital_select = FORCE_DIGITAL;
	memset(t->fifo, 0, sizeof(t->fifo));
	t->fifo_head = t->fifo_tail = t->fifo_count = t->fifo_bits_taken = 0;

	t->SPEN = t->DDIS = t->TALK = t->TALKD = t->previous_talk_status = t->irq_pin = t->ready_pin = 0;
	set_interrupt_state(t, 0);
	update_ready_state(t);
	t->buffer_empty = t->buffer_low = 1;

	t->command_register = NOCOMMAND;
	t->data_latched = 0;
	t->RDB_flag = 0;

	t->new_frame_energy_idx = 0;
	t->current_energy = 0;
	t->previous_energy = 0;
	t->new_frame_pitch_idx = 0;
	t->current_pitch = 0;
	t->zpar = t->uv_zpar = 0;
	memset(t->new_frame_k_idx, 0, sizeof(t->new_frame_k_idx));
	memset(t->current_k, 0, sizeof(t->current_k));

	t->inhibit = 1;
	t->subcycle = t->c_variant_rate = 0;
	t->pitch_count = 0;
	t->PC = 0;
	t->subc_reload = FORCE_SUBC_RELOAD;
	t->OLDE = t->OLDP = 1;
	t->IP = reload_table[t->c_variant_rate & 0x3];
	t->RNG = 0x1fff;
	memset(t->u, 0, sizeof(t->u));
	memset(t->x, 0, sizeof(t->x));

	perform_dummy_read(t);

	t->PDC = 0;
	t->CTL_pins = 0;
	t->state = 0;
	t->address = 0;
	t->next_is_address = 0;
	t->addr_bit = 0;
	t->CTL_buffer = 0;
}

static void set_io_ready(tms5220_t *t, int param)
{
	if (param) {
		switch (t->rs_ws) {
		case 0x02:   // write
			if (t->fifo_count >= TMS5220_FIFO_SIZE && t->DDIS) {
				io_timer_adjust(t, 16, 1);
				break;
			}
			if (t->command_register != NOCOMMAND) {
				// the command register is still busy: keep /READY inactive and the data latch
			} else {
				t->data_latched = 0;
				data_write(t, t->write_latch);
			}
			break;
		case 0x01:   // read
			t->read_latch = status_read(t, 1);
			t->io_ready = 1;
			break;
		case 0x03:   // high impedance
		case 0x00:   // illegal
			t->io_ready = 1;
			break;
		}
	}
	update_ready_state(t);
}

// ---- the time base ----
int tms5220_advance(tms5220_t *t, uint32_t clocks, int16_t *out, int max)
{
	int n = 0;
	for (;;) {
		uint32_t m = SAMPLE_CLOCKS - t->pos;
		if (m > clocks)
			m = clocks;
		const int fire = t->io_pending && t->io_clocks <= m;
		if (fire)
			m = t->io_clocks;
		else if (!clocks)
			break;
		else if (t->io_pending)
			t->io_clocks -= m;
		t->pos += m;
		clocks -= m;
		if (fire) {
			t->io_pending = 0;
			t->io_clocks = 0;
		}
		if (t->pos >= SAMPLE_CLOCKS) {
			// (MAME's set_io_ready brings the stream up to date first: the sample ending now comes before it)
			const int16_t s = gen_sample(t);
			if (n < max)
				out[n++] = s;
			t->pos = 0;
		}
		if (fire)
			set_io_ready(t, t->io_param);
	}
	return n;
}

void tms5220_render(tms5220_t *t, int16_t *out, int n)
{
	if (n <= 0)
		return;
	tms5220_advance(t, (SAMPLE_CLOCKS - t->pos) + (uint32_t)(n - 1) * SAMPLE_CLOCKS, out, n);
}

uint32_t tms5220_rate(const tms5220_t *t) { return t->clock / SAMPLE_CLOCKS; }

// ---- init ----
void tms5220_init(tms5220_t *t, int variant, uint32_t clock)
{
	memset(t, 0, sizeof(*t));
	t->variant = variant == TMS5220_VARIANT_5220C ? TMS5220_VARIANT_5220C : TMS5220_VARIANT_5220;
	t->pitch_tab = variant == TMS5220_VARIANT_5200 ? pitchtable_5200 : pitchtable;   // (the 5200: a 5220 otherwise)
	t->k_tab = variant == TMS5220_VARIANT_5200 ? ktable_5200 : ktable;
	t->clock = clock;
	// (device_start)
	t->io_ready = 1;
	t->true_timing = 0;
	t->rs_ws = 0x03;
	t->write_latch = 0;
	device_reset(t);
}

void tms5220_reset(tms5220_t *t) { device_reset(t); }

void tms5220_set_callbacks(tms5220_t *t, void (*irq_cb)(void *, int), void (*ready_cb)(void *, int), void *ctx)
{
	t->irq_cb = irq_cb;
	t->ready_cb = ready_cb;
	t->cb_ctx = ctx;
}

void tms5220_set_clock(tms5220_t *t, uint32_t clock)
{
	if (!clock || clock == t->clock)
		return;
	// a pending io_ready keeps its absolute time
	if (t->io_pending && t->clock)
		t->io_clocks = (uint32_t)(((uint64_t)t->io_clocks * clock + t->clock / 2) / t->clock);
	t->clock = clock;
}

// ---- pins ----
void tms5220_rsq_w(tms5220_t *t, int state)
{
	t->true_timing = 1;
	state &= 0x01;
	const uint8_t new_val = (uint8_t)((t->rs_ws & 0x01) | (state << 1));
	if (new_val == t->rs_ws)
		return;
	t->rs_ws = new_val;
	if (new_val == 0) {
		if (HAS_RATE_CONTROL(t))
			device_reset(t);
		return;
	}
	if (new_val == 3) {
		t->read_latch = 0xff;
		return;
	}
	if (!state) {
		// high to low: /READY goes inactive at once, data valid ~13 us later
		t->io_ready = 0;
		update_ready_state(t);
		io_timer_adjust(t, (uint32_t)(((uint64_t)t->clock * 13 + 500000) / 1000000), 1);
	}
}

void tms5220_wsq_w(tms5220_t *t, int state)
{
	t->true_timing = 1;
	state &= 0x01;
	const uint8_t new_val = (uint8_t)((t->rs_ws & 0x02) | state);
	if (new_val == t->rs_ws)
		return;
	t->rs_ws = new_val;
	if (new_val == 0) {
		if (HAS_RATE_CONTROL(t))
			device_reset(t);
		return;
	}
	if (new_val == 3) {
		t->read_latch = 0xff;
		return;
	}
	if (!state) {
		// high to low: /READY inactive, 16 clocks to complete (MAME assumes 16 for every command)
		t->io_ready = 0;
		update_ready_state(t);
		io_timer_adjust(t, 16, 1);
	}
}

void tms5220_data_w(tms5220_t *t, uint8_t data)
{
	t->write_latch = data;
	t->data_latched = 1;
	if (!t->true_timing)
		data_write(t, t->write_latch);
}

uint8_t tms5220_status_r(tms5220_t *t)
{
	if (!t->true_timing)
		return status_read(t, 1);
	if (t->rs_ws == 0x01)
		return t->read_latch;
	return 0xff;
}

int tms5220_readyq_r(tms5220_t *t)
{
	int ready;
	if (!t->true_timing)
		ready = (t->fifo_count < TMS5220_FIFO_SIZE || !t->DDIS) && t->io_ready;
	else
		ready = t->io_ready;
	return !ready;
}

int tms5220_intq_r(tms5220_t *t) { return !t->irq_pin; }

int tms5220_state(tms5220_t *t, void *ctx, int load, const char *tag)
{
	(void)load;
	return rbx_state_io(ctx, tag, t, (uint32_t)offsetof(tms5220_t, state_end));
}
