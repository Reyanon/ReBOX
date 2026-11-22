// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, R. Belmont, hap
/*
    Yamaha YMF271-F "OPX" emulator v0.1
    By R. Belmont.
    Based in part on YMF278B emulator by R. Belmont and O. Galibert.
    12June04 update by Toshiaki Nijiura
    Copyright R. Belmont.

    Ported to C from MAME 0.289 src/devices/sound/ymf271.cpp (BSD-3-Clause, copyright-holders: Olivier Galibert,
    R. Belmont, hap): the device's members are the chip struct's (c->x for m_x), its timers and stream are the
    owner's (ymf271.h).
    This file is part of ReBOX. GPL-2.0-or-later.

    TODO (MAME's):
    - A/L bit (alternate loop)
    - EN and EXT Out bits
    - Src B and Src NOTE bits
    - statusreg Busy flag
    - PFM (FM using external PCM waveform)
    - detune (should be same as on other Yamaha chips)
    - Acc On bit (some sound effects in viprp1?). The documentation says
      "determines if slot output is accumulated(1), or output directly(0)"
    - Is memory handling 100% correct? At the moment, seibuspi.cpp is the only
      hardware currently emulated that uses external handlers.
    - *16 multiplier for timer B is free-running like other yamaha FM chips?
*/

#include "ymf271.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define logerror(...) ((void)0)
#define popmessage(...) ((void)0)
#define fatalerror(...) ((void)0)

static void irq_set(ymf271_t *c, int state)
{
	if (c->irq)
		c->irq(c->user, state);
}

#define STD_CLOCK       (16934400)

#define MAXOUT          (+32767)
#define MINOUT          (-32768)

#define SIN_BITS        10
#define SIN_LEN         (1<<SIN_BITS)
#define SIN_MASK        (SIN_LEN-1)

#define LFO_LENGTH      256
#define LFO_SHIFT       8
#define PLFO_MAX        (+1.0)
#define PLFO_MIN        (-1.0)
#define ALFO_MAX        (+65536)
#define ALFO_MIN        (0)

#define ENV_ATTACK      0
#define ENV_DECAY1      1
#define ENV_DECAY2      2
#define ENV_RELEASE     3

#define OP_INPUT_FEEDBACK   -1
#define OP_INPUT_NONE       -2

#define ENV_VOLUME_SHIFT    16

#define INF     -1.0

static const double ARTime[64] =
{
	INF,        INF,        INF,        INF,        6188.12,    4980.68,    4144.76,    3541.04,
	3094.06,    2490.34,    2072.38,    1770.52,    1547.03,    1245.17,    1036.19,    885.26,
	773.51,     622.59,     518.10,     441.63,     386.76,     311.29,     259.05,     221.32,
	193.38,     155.65,     129.52,     110.66,     96.69,      77.82,      64.76,      55.33,
	48.34,      38.91,      32.38,      27.66,      24.17,      19.46,      16.19,      13.83,
	12.09,      9.73,       8.10,       6.92,       6.04,       4.86,       4.05,       3.46,
	3.02,       2.47,       2.14,       1.88,       1.70,       1.38,       1.16,       1.02,
	0.88,       0.70,       0.57,       0.48,       0.43,       0.43,       0.43,       0.07
};

static const double DCTime[64] =
{
	INF,        INF,        INF,        INF,        93599.64,   74837.91,   62392.02,   53475.56,
	46799.82,   37418.96,   31196.01,   26737.78,   23399.91,   18709.48,   15598.00,   13368.89,
	11699.95,   9354.74,    7799.00,    6684.44,    5849.98,    4677.37,    3899.50,    3342.22,
	2924.99,    2338.68,    1949.75,    1671.11,    1462.49,    1169.34,    974.88,     835.56,
	731.25,     584.67,     487.44,     417.78,     365.62,     292.34,     243.72,     208.89,
	182.81,     146.17,     121.86,     104.44,     91.41,      73.08,      60.93,      52.22,
	45.69,      36.55,      33.85,      26.09,      22.83,      18.28,      15.22,      13.03,
	11.41,      9.12,       7.60,       6.51,       5.69,       5.69,       5.69,       5.69
};

/* Notes about the LFO Frequency Table below:

    There are 2 known errors in the LFO table listed in the original manual.

    Both 201 & 202 are listed as 3.74490.  202 has been computed/corrected to 3.91513
    232 was listed as 13.35547 but has been replaced with the correct value of 14.35547.

  Corrections are computed values based on formulas by Olivier Galibert & Nicola Salmoria listed below:

LFO period seems easy to compute:

Olivier Galibert's version                       Nicola Salmoria's version

int lfo_period(int entry)             or         int calc_lfo_period(int entry)
{                                                {
  int ma, ex;                                      entry = 256 - entry;
  entry = 256-entry;
  ma = entry & 15;                                 if (entry < 16)
                                                   {
  ex = entry >> 4;                                    return (entry & 0x0f) << 7;
  if(ex)                                           }
    return (ma | 16) << (ex+6);                    else
  else                                             {
    return ma << 7;                                   int shift = 6 + (entry >> 4);
}                                                     return (0x10 + (entry & 0x0f)) << shift;
                                                   }
lfo_freq = 44100 / lfo_period                    }

*/

static const double LFO_frequency_table[256] =
{
	0.00066,    0.00068,    0.00070,    0.00073,    0.00075,    0.00078,    0.00081,    0.00084,
	0.00088,    0.00091,    0.00096,    0.00100,    0.00105,    0.00111,    0.00117,    0.00124,
	0.00131,    0.00136,    0.00140,    0.00145,    0.00150,    0.00156,    0.00162,    0.00168,
	0.00175,    0.00183,    0.00191,    0.00200,    0.00210,    0.00221,    0.00234,    0.00247,
	0.00263,    0.00271,    0.00280,    0.00290,    0.00300,    0.00312,    0.00324,    0.00336,
	0.00350,    0.00366,    0.00382,    0.00401,    0.00421,    0.00443,    0.00467,    0.00495,
	0.00526,    0.00543,    0.00561,    0.00580,    0.00601,    0.00623,    0.00647,    0.00673,
	0.00701,    0.00731,    0.00765,    0.00801,    0.00841,    0.00885,    0.00935,    0.00990,
	0.01051,    0.01085,    0.01122,    0.01160,    0.01202,    0.01246,    0.01294,    0.01346,
	0.01402,    0.01463,    0.01529,    0.01602,    0.01682,    0.01771,    0.01869,    0.01979,
	0.02103,    0.02171,    0.02243,    0.02320,    0.02403,    0.02492,    0.02588,    0.02692,
	0.02804,    0.02926,    0.03059,    0.03204,    0.03365,    0.03542,    0.03738,    0.03958,
	0.04206,    0.04341,    0.04486,    0.04641,    0.04807,    0.04985,    0.05176,    0.05383,
	0.05608,    0.05851,    0.06117,    0.06409,    0.06729,    0.07083,    0.07477,    0.07917,
	0.08411,    0.08683,    0.08972,    0.09282,    0.09613,    0.09969,    0.10353,    0.10767,
	0.11215,    0.11703,    0.12235,    0.12817,    0.13458,    0.14167,    0.14954,    0.15833,
	0.16823,    0.17365,    0.17944,    0.18563,    0.19226,    0.19938,    0.20705,    0.21533,
	0.22430,    0.23406,    0.24470,    0.25635,    0.26917,    0.28333,    0.29907,    0.31666,
	0.33646,    0.34731,    0.35889,    0.37126,    0.38452,    0.39876,    0.41410,    0.43066,
	0.44861,    0.46811,    0.48939,    0.51270,    0.53833,    0.56666,    0.59814,    0.63333,
	0.67291,    0.69462,    0.71777,    0.74252,    0.76904,    0.79753,    0.82820,    0.86133,
	0.89722,    0.93623,    0.97878,    1.02539,    1.07666,    1.13333,    1.19629,    1.26666,
	1.34583,    1.38924,    1.43555,    1.48505,    1.53809,    1.59509,    1.65640,    1.72266,
	1.79443,    1.87245,    1.95756,    2.05078,    2.15332,    2.26665,    2.39258,    2.53332,
	2.69165,    2.77848,    2.87109,    2.97010,    3.07617,    3.19010,    3.31280,    3.44531,
	3.58887,    3.74490,    3.91513,    4.10156,    4.30664,    4.53331,    4.78516,    5.06664,
	5.38330,    5.55696,    5.74219,    5.94019,    6.15234,    6.38021,    6.62560,    6.89062,
	7.17773,    7.48981,    7.83026,    8.20312,    8.61328,    9.06661,    9.57031,    10.13327,
	10.76660,   11.11391,   11.48438,   11.88039,   12.30469,   12.76042,   13.25120,   13.78125,
	14.35547,   14.97962,   15.66051,   16.40625,   17.22656,   18.13322,   19.14062,   20.26654,
	21.53320,   22.96875,   24.60938,   26.50240,   28.71094,   31.32102,   34.45312,   38.28125,
	43.06641,   49.21875,   57.42188,   68.90625,   86.13281,   114.84375,  172.26562,  344.53125
};

static const int RKS_Table[32][8] =
{
	{  0,  0,  0,  0,  0,  2,  4,  8 },
	{  0,  0,  0,  0,  1,  3,  5,  9 },
	{  0,  0,  0,  1,  2,  4,  6, 10 },
	{  0,  0,  0,  1,  3,  5,  7, 11 },
	{  0,  0,  1,  2,  4,  6,  8, 12 },
	{  0,  0,  1,  2,  5,  7,  9, 13 },
	{  0,  0,  1,  3,  6,  8, 10, 14 },
	{  0,  0,  1,  3,  7,  9, 11, 15 },
	{  0,  1,  2,  4,  8, 10, 12, 16 },
	{  0,  1,  2,  4,  9, 11, 13, 17 },
	{  0,  1,  2,  5, 10, 12, 14, 18 },
	{  0,  1,  2,  5, 11, 13, 15, 19 },
	{  0,  1,  3,  6, 12, 14, 16, 20 },
	{  0,  1,  3,  6, 13, 15, 17, 21 },
	{  0,  1,  3,  7, 14, 16, 18, 22 },
	{  0,  1,  3,  7, 15, 17, 19, 23 },
	{  0,  2,  4,  8, 16, 18, 20, 24 },
	{  0,  2,  4,  8, 17, 19, 21, 25 },
	{  0,  2,  4,  9, 18, 20, 22, 26 },
	{  0,  2,  4,  9, 19, 21, 23, 27 },
	{  0,  2,  5, 10, 20, 22, 24, 28 },
	{  0,  2,  5, 10, 21, 23, 25, 29 },
	{  0,  2,  5, 11, 22, 24, 26, 30 },
	{  0,  2,  5, 11, 23, 25, 27, 31 },
	{  0,  3,  6, 12, 24, 26, 28, 31 },
	{  0,  3,  6, 12, 25, 27, 29, 31 },
	{  0,  3,  6, 13, 26, 28, 30, 31 },
	{  0,  3,  6, 13, 27, 29, 31, 31 },
	{  0,  3,  7, 14, 28, 30, 31, 31 },
	{  0,  3,  7, 14, 29, 31, 31, 31 },
	{  0,  3,  7, 15, 30, 31, 31, 31 },
	{  0,  3,  7, 15, 31, 31, 31, 31 },
};

static const double multiple_table[16] = { 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

static const double pow_table[16] = { 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 0.5, 1, 2, 4, 8, 16, 32, 64 };

static const double fs_frequency[4] = { 1.0/1.0, 1.0/2.0, 1.0/4.0, 1.0/8.0 };

static const double channel_attenuation_table[16] =
{
	0.0, 2.5, 6.0, 8.5, 12.0, 14.5, 18.1, 20.6, 24.1, 26.6, 30.1, 32.6, 36.1, 96.1, 96.1, 96.1
};

static const int modulation_level[8] = { 16, 8, 4, 2, 1, 32, 64, 128 };

// feedback_level * 16
static const int feedback_level[8] = { 0, 1, 2, 4, 8, 16, 32, 64 };

// slot mapping assists
static const int fm_tab[16] = { 0, 1, 2, -1, 3, 4, 5, -1, 6, 7, 8, -1, 9, 10, 11, -1 };
static const int pcm_tab[16] = { 0, 4, 8, -1, 12, 16, 20, -1, 24, 28, 32, -1, 36, 40, 44, -1 };


/*****************************************************************************/

static void calculate_step(ymf271_t *c, YMF271Slot *slot)
{
	double st;

	if (slot->waveform == 7)
	{
		// external waveform (PCM)
		st = (double)(2 * (slot->fns | 2048)) * pow_table[slot->block] * fs_frequency[slot->fs];
		st = st * multiple_table[slot->multiple];

		// LFO phase modulation
		st *= slot->lfo_phasemod;

		st /= (double)(524288/65536); // pre-multiply with 65536

		slot->step = (uint32_t)st;
	}
	else
	{
		// internal waveform (FM)
		st = (double)(2 * slot->fns) * pow_table[slot->block];
		st = st * multiple_table[slot->multiple] * (double)(SIN_LEN);

		// LFO phase modulation
		st *= slot->lfo_phasemod;

		st /= (double)(536870912/65536); // pre-multiply with 65536

		slot->step = (uint32_t)st;
	}
}

static inline bool check_envelope_end(ymf271_t *c, YMF271Slot *slot)
{
	if (slot->volume <= 0)
	{
		slot->active = 0;
		slot->volume = 0;
		return true;
	}
	return false;
}

// calculate status end disable/enable (Desert War shots relies on this)
static inline void calculate_status_end(ymf271_t *c, int slotnum, bool state)
{
	// guess: don't enable/disable if slot isn't a multiple of 4
	if (slotnum & 3)
		return;

	/*
	bit scheme is kinda twisted
	status1 Busy  End36 End24 End12 End0  ----  TimB  TimA
	status2 End44 End32 End20 End8  End40 End28 End16 End4
	*/
	uint8_t subbit = slotnum / 12;
	uint8_t bankbit = ((slotnum % 12) >> 2);

	if (state == false)
		c->end_status &= ~(1 << (subbit + bankbit * 4));
	else
		c->end_status |= (1 << (subbit + bankbit * 4));

}

// (the per-sample helpers inlined into update_pcm / calculate_op: no calls a sample)
#define YMF_INLINE static inline __attribute__((always_inline))

// a byte of the external memory (the owner's ROM straight when set: ymf271_set_rom)
YMF_INLINE uint8_t ext_read(ymf271_t *c, uint32_t addr)
{
	if (c->rom)   // (a power-of-two size: masked; else outside the ROM reads 0, as the owners' read())
		return c->rom_mask ? c->rom[addr & c->rom_mask] : addr < c->rom_size ? c->rom[addr] : 0;
	return c->read(c->user, addr);
}

YMF_INLINE void update_envelope(ymf271_t *c, YMF271Slot *slot)
{
	switch (slot->env_state)
	{
		case ENV_ATTACK:
		{
			slot->volume += slot->env_attack_step;

			if (slot->volume >= (255 << ENV_VOLUME_SHIFT))
			{
				slot->volume = (255 << ENV_VOLUME_SHIFT);
				slot->env_state = ENV_DECAY1;
			}
			break;
		}

		case ENV_DECAY1:
		{
			int decay_level = 255 - (slot->decay1lvl << 4);
			slot->volume -= slot->env_decay1_step;

			if (!check_envelope_end(c, slot) && (slot->volume >> ENV_VOLUME_SHIFT) <= decay_level)
			{
				slot->env_state = ENV_DECAY2;
			}
			break;
		}

		case ENV_DECAY2:
		{
			slot->volume -= slot->env_decay2_step;
			check_envelope_end(c, slot);
			break;
		}

		case ENV_RELEASE:
		{
			slot->volume -= slot->env_release_step;
			check_envelope_end(c, slot);
			break;
		}
	}
}

static inline int get_keyscaled_rate(ymf271_t *c, int rate, int keycode, int keyscale)
{
	int newrate = rate + RKS_Table[keycode][keyscale];

	if (newrate > 63)
	{
		newrate = 63;
	}
	if (newrate < 0)
	{
		newrate = 0;
	}
	return newrate;
}

static inline int get_internal_keycode(ymf271_t *c, int block, int fns)
{
	int n43;
	if (fns < 0x780)
	{
		n43 = 0;
	}
	else if (fns < 0x900)
	{
		n43 = 1;
	}
	else if (fns < 0xa80)
	{
		n43 = 2;
	}
	else
	{
		n43 = 3;
	}

	return ((block & 7) * 4) + n43;
}

static inline int get_external_keycode(ymf271_t *c, int block, int fns)
{
	int n43;
	if (fns < 0x100)
	{
		n43 = 0;
	}
	else if (fns < 0x300)
	{
		n43 = 1;
	}
	else if (fns < 0x500)
	{
		n43 = 2;
	}
	else
	{
		n43 = 3;
	}

	return ((block & 7) * 4) + n43;
}

static void init_envelope(ymf271_t *c, YMF271Slot *slot)
{
	int keycode, rate;
	int decay_level = 255 - (slot->decay1lvl << 4);

	if (slot->waveform != 7)
	{
		keycode = get_internal_keycode(c, slot->block, slot->fns);
	}
	else
	{
		keycode = get_external_keycode(c, slot->block, slot->fns & 0x7ff);
		/* keycode = (keycode + slot->srcb * 4 + slot->srcnote) / 2; */ // not sure
	}

	// init attack state
	rate = get_keyscaled_rate(c, slot->ar * 2, keycode, slot->keyscale);
	slot->env_attack_step = (rate < 4) ? 0 : (int)(((double)(255-0) / c->lut_ar[rate]) * 65536.0);

	// init decay1 state
	rate = get_keyscaled_rate(c, slot->decay1rate * 2, keycode, slot->keyscale);
	slot->env_decay1_step = (rate < 4) ? 0 : (int)(((double)(255-decay_level) / c->lut_dc[rate]) * 65536.0);

	// init decay2 state
	rate = get_keyscaled_rate(c, slot->decay2rate * 2, keycode, slot->keyscale);
	slot->env_decay2_step = (rate < 4) ? 0 : (int)(((double)(255-0) / c->lut_dc[rate]) * 65536.0);

	// init release state
	rate = get_keyscaled_rate(c, slot->relrate * 4, keycode, slot->keyscale);
	slot->env_release_step = (rate < 4) ? 0 : (int)(((double)(255-0) / c->lut_dc[rate]) * 65536.0);

	slot->volume = (255-160) << ENV_VOLUME_SHIFT; // -60db
	slot->env_state = ENV_ATTACK;
}

static void init_lfo(ymf271_t *c, YMF271Slot *slot)
{
	slot->lfo_phase = 0;
	slot->lfo_amplitude = 0;
	slot->lfo_phasemod = 0;

	slot->lfo_step = (int)((((double)LFO_LENGTH * c->lut_lfo[slot->lfoFreq]) / 44100.0) * 256.0);
}

YMF_INLINE void update_lfo(ymf271_t *c, YMF271Slot *slot)
{
	slot->lfo_phase += slot->lfo_step;

	slot->lfo_amplitude = c->lut_alfo[slot->lfowave][(slot->lfo_phase >> LFO_SHIFT) & (LFO_LENGTH-1)];
	const double pm = c->lut_plfo[slot->lfowave][slot->pms][(slot->lfo_phase >> LFO_SHIFT) & (LFO_LENGTH-1)];

	// (the step depends on the phase modulation and on registers whose writes compute it again: the same modulation
	// gives the same step, so it is made again only when the modulation changed or after any register write, reset or
	// state load (step_gen))
	const int sn = (int)(slot - c->slots);
	if (pm != slot->lfo_phasemod || c->step_gen_slot[sn] != c->step_gen)
	{
		slot->lfo_phasemod = pm;
		calculate_step(c, slot);
		c->step_gen_slot[sn] = c->step_gen;
	}
}

// (lfo_volume and env_volume fit 32 bits, their products not: 32 x 32 -> 64-bit multiplies, the same values)
YMF_INLINE int64_t calculate_slot_volume(ymf271_t *c, YMF271Slot *slot)
{
	int64_t volume;
	int32_t env_volume;
	int32_t lfo_volume = 65536;

	switch (slot->ams)
	{
		case 0: lfo_volume = 65536; break;  // 0dB
		case 1: lfo_volume = 65536 - ((slot->lfo_amplitude * 33124) >> 16); break;  // 5.90625dB
		case 2: lfo_volume = 65536 - ((slot->lfo_amplitude * 16742) >> 16); break;  // 11.8125dB
		case 3: lfo_volume = 65536 - ((slot->lfo_amplitude * 4277) >> 16); break;   // 23.625dB
	}

	// (no amplitude LFO: times 65536 >> 16, the table's value itself)
	env_volume = slot->ams == 0 ? c->lut_env_volume[255 - (slot->volume >> ENV_VOLUME_SHIFT)]
	                            : (int32_t)(((int64_t)c->lut_env_volume[255 - (slot->volume >> ENV_VOLUME_SHIFT)] * lfo_volume) >> 16);

	volume = ((int64_t)env_volume * c->lut_total_level[slot->tl]) >> 16;

	return volume;
}

static void update_pcm(ymf271_t *c, int slotnum, int32_t *mixp, int length)
{
	int i;
	int64_t final_volume;
	int16_t sample;
	int64_t ch0_vol, ch1_vol, ch2_vol, ch3_vol;

	YMF271Slot *slot = &c->slots[slotnum];

	if (!slot->active)
	{
		return;
	}

	if (slot->waveform != 7)
	{
		fatalerror("Waveform %d in update_pcm!!!\n", slot->waveform);
	}

	// (the output levels do not change during a render: their attenuations read once)
	const int32_t att0 = c->lut_attenuation[slot->ch0_level], att1 = c->lut_attenuation[slot->ch1_level];
	const int32_t att2 = c->lut_attenuation[slot->ch2_level], att3 = c->lut_attenuation[slot->ch3_level];

	for (i = 0; i < length; i++)
	{
		// loop
		if ((slot->stepptr>>16) > slot->endaddr)
		{
			slot->stepptr = slot->stepptr - ((uint64_t)slot->endaddr<<16) + ((uint64_t)slot->loopaddr<<16);
			calculate_status_end(c, slotnum,true);
			if ((slot->stepptr>>16) > slot->endaddr)
			{
				// overflow
				slot->stepptr &= 0xffff;
				slot->stepptr |= ((uint64_t)slot->loopaddr<<16);
				if ((slot->stepptr>>16) > slot->endaddr)
				{
					// still overflow? (triggers in rdft2, rarely)
					slot->stepptr &= 0xffff;
					slot->stepptr |= ((uint64_t)slot->endaddr<<16);
				}
			}
		}

		if (slot->bits == 8)
		{
			// 8bit
			sample = ext_read(c, (uint32_t)(slot->startaddr + (slot->stepptr>>16)))<<8;
		}
		else
		{
			// 12bit
			if (slot->stepptr & 0x10000)
				sample = ext_read(c, (uint32_t)(slot->startaddr + (slot->stepptr>>17)*3 + 2))<<8 | ((ext_read(c, (uint32_t)(slot->startaddr + (slot->stepptr>>17)*3 + 1)) << 4) & 0xf0);
			else
				sample = ext_read(c, (uint32_t)(slot->startaddr + (slot->stepptr>>17)*3))<<8 | (ext_read(c, (uint32_t)(slot->startaddr + (slot->stepptr>>17)*3 + 1)) & 0xf0);
		}

		update_envelope(c, slot);
		update_lfo(c, slot);

		// (final_volume and the levels fit 32 bits: 32 x 32 -> 64-bit products; a sample times a level (at most
		// 65536) fits 32 bits)
		final_volume = calculate_slot_volume(c, slot);
		const int32_t fv = (int32_t)final_volume;

		ch0_vol = ((int64_t)fv * att0) >> 16;
		ch1_vol = ((int64_t)fv * att1) >> 16;
		ch2_vol = ((int64_t)fv * att2) >> 16;
		ch3_vol = ((int64_t)fv * att3) >> 16;

		if (ch0_vol > 65536) ch0_vol = 65536;
		if (ch1_vol > 65536) ch1_vol = 65536;
		if (ch2_vol > 65536) ch2_vol = 65536;
		if (ch3_vol > 65536) ch3_vol = 65536;

		*mixp++ += (sample * (int32_t)ch0_vol) >> 16;
		*mixp++ += (sample * (int32_t)ch1_vol) >> 16;
		*mixp++ += (sample * (int32_t)ch2_vol) >> 16;
		*mixp++ += (sample * (int32_t)ch3_vol) >> 16;

		// go to next step
		slot->stepptr += slot->step;
	}
}

// calculates the output of one FM operator
YMF_INLINE int64_t calculate_op(ymf271_t *c, int slotnum, int64_t inp)
{
	YMF271Slot *slot = &c->slots[slotnum];
	int64_t env, slot_output, slot_input = 0;

	update_envelope(c, slot);
	update_lfo(c, slot);
	env = calculate_slot_volume(c, slot);

	if (inp == OP_INPUT_FEEDBACK)
	{
		// from own feedback
		slot_input = (slot->feedback_modulation0 + slot->feedback_modulation1) / 2;
		slot->feedback_modulation0 = slot->feedback_modulation1;
	}
	else if (inp != OP_INPUT_NONE)
	{
		// from previous slot output
		slot_input = ((inp << (SIN_BITS-2)) * modulation_level[slot->feedback]);
	}

	// (the phase's bits 16-25 only: the low 32 bits of the sum; the wave (16 bits) times env (below 2^17): a
	// 32 x 32 -> 64-bit product)
	slot_output = c->lut_waves[slot->waveform][(((uint32_t)slot->stepptr + (uint32_t)slot_input) >> 16) & SIN_MASK];
	slot_output = ((int64_t)(int32_t)slot_output * (int32_t)env) >> 16;
	slot->stepptr += slot->step;

	return slot_output;
}

static void set_feedback(ymf271_t *c, int slotnum, int64_t inp)
{
	YMF271Slot *slot = &c->slots[slotnum];
	slot->feedback_modulation1 = (((inp << (SIN_BITS-2)) * feedback_level[slot->feedback]) / 16);
}

//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

// (the slots' output levels do not change during a render: their four attenuations read once a group; an output
// (below 2^17) times one: a 32 x 32 -> 64-bit product)
#define ATT4(a, s) const int32_t a[4] = { c->lut_attenuation[c->slots[s].ch0_level], \
	c->lut_attenuation[c->slots[s].ch1_level], c->lut_attenuation[c->slots[s].ch2_level], \
	c->lut_attenuation[c->slots[s].ch3_level] }
#define MULA(o, a) ((int64_t)(int32_t)(o) * (a))

// the groups g0, g0 + gstep, ... below 12 (each group's slots are its own: groups made apart and their outputs added
// give the same sums)
static void render_groups(ymf271_t *c, int32_t *mix, int samples, int g0, int gstep)
{
	int32_t *mixp;

	memset(mix, 0, sizeof(int32_t) * 4 * (size_t)samples);

	for (int j = g0; j < 12; j += gstep)
	{
		YMF271Group *slot_group = &c->groups[j];
		mixp = mix;

		if (slot_group->pfm && slot_group->sync != 3)
		{
			popmessage("ymf271 PFM, contact MAMEdev");
			logerror("ymf271 Group %d: PFM, Sync = %d, Waveform Slot1 = %d, Slot2 = %d, Slot3 = %d, Slot4 = %d\n",
				j, slot_group->sync, c->slots[j+0].waveform, c->slots[j+12].waveform, c->slots[j+24].waveform, c->slots[j+36].waveform);
		}

		switch (slot_group->sync)
		{
			// 4 operator FM
			case 0:
			{
				int slot1 = j + (0*12);
				int slot2 = j + (1*12);
				int slot3 = j + (2*12);
				int slot4 = j + (3*12);
				mixp = mix;

				if (c->slots[slot1].active)
				{
					ATT4(a1, slot1);
					ATT4(a2, slot2);
					ATT4(a3, slot3);
					ATT4(a4, slot4);
					for (int i = 0; i < samples; i++)
					{
						int64_t output1 = 0, output2 = 0, output3 = 0, output4 = 0;
						int64_t phase_mod1, phase_mod2, phase_mod3;
						switch (c->slots[slot1].algorithm)
						{
							// <--------|
							// +--[S1]--|--+--[S3]--+--[S2]--+--[S4]-->
							case 0:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								phase_mod3 = calculate_op(c, slot3, phase_mod1);
								phase_mod2 = calculate_op(c, slot2, phase_mod3);
								output4 = calculate_op(c, slot4, phase_mod2);
								break;

							// <-----------------|
							// +--[S1]--+--[S3]--|--+--[S2]--+--[S4]-->
							case 1:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(c, slot3, phase_mod1);
								set_feedback(c, slot1, phase_mod3);
								phase_mod2 = calculate_op(c, slot2, phase_mod3);
								output4 = calculate_op(c, slot4, phase_mod2);
								break;

							// <--------|
							// +--[S1]--|
							//          |
							//  --[S3]--+--[S2]--+--[S4]-->
							case 2:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								phase_mod3 = calculate_op(c, slot3, OP_INPUT_NONE);
								phase_mod2 = calculate_op(c, slot2, (phase_mod1 + phase_mod3) / 1);
								output4 = calculate_op(c, slot4, phase_mod2);
								break;

							//          <--------|
							//          +--[S1]--|
							//                   |
							//  --[S3]--+--[S2]--+--[S4]-->
							case 3:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								phase_mod3 = calculate_op(c, slot3, OP_INPUT_NONE);
								phase_mod2 = calculate_op(c, slot2, phase_mod3);
								output4 = calculate_op(c, slot4, (phase_mod1 + phase_mod2) / 1);
								break;

							//              --[S2]--|
							// <--------|           |
							// +--[S1]--|--+--[S3]--+--[S4]-->
							case 4:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								phase_mod3 = calculate_op(c, slot3, phase_mod1);
								phase_mod2 = calculate_op(c, slot2, OP_INPUT_NONE);
								output4 = calculate_op(c, slot4, (phase_mod3 + phase_mod2) / 1);
								break;

							//           --[S2]-----|
							// <-----------------|  |
							// +--[S1]--+--[S3]--|--+--[S4]-->
							case 5:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(c, slot3, phase_mod1);
								set_feedback(c, slot1, phase_mod3);
								phase_mod2 = calculate_op(c, slot2, OP_INPUT_NONE);
								output4 = calculate_op(c, slot4, (phase_mod3 + phase_mod2) / 1);
								break;

							//  --[S2]-----+--[S4]--|
							//                      |
							// <--------|           |
							// +--[S1]--|--+--[S3]--+-->
							case 6:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output3 = calculate_op(c, slot3, phase_mod1);
								phase_mod2 = calculate_op(c, slot2, OP_INPUT_NONE);
								output4 = calculate_op(c, slot4, phase_mod2);
								break;

							//  --[S2]--+--[S4]-----|
							//                      |
							// <-----------------|  |
							// +--[S1]--+--[S3]--|--+-->
							case 7:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(c, slot3, phase_mod1);
								set_feedback(c, slot1, phase_mod3);
								output3 = phase_mod3;
								phase_mod2 = calculate_op(c, slot2, OP_INPUT_NONE);
								output4 = calculate_op(c, slot4, phase_mod2);
								break;

							//  --[S3]--+--[S2]--+--[S4]--|
							//                            |
							// <--------|                 |
							// +--[S1]--|-----------------+-->
							case 8:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output1 = phase_mod1;
								phase_mod3 = calculate_op(c, slot3, OP_INPUT_NONE);
								phase_mod2 = calculate_op(c, slot2, phase_mod3);
								output4 = calculate_op(c, slot4, phase_mod2);
								break;

							//          <--------|
							//          +--[S1]--|
							//                   |
							//  --[S3]--|        |
							//  --[S2]--+--[S4]--+-->
							case 9:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output1 = phase_mod1;
								phase_mod3 = calculate_op(c, slot3, OP_INPUT_NONE);
								phase_mod2 = calculate_op(c, slot2, OP_INPUT_NONE);
								output4 = calculate_op(c, slot4, (phase_mod3 + phase_mod2) / 1);
								break;

							//              --[S4]--|
							//              --[S2]--|
							// <--------|           |
							// +--[S1]--|--+--[S3]--+-->
							case 10:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output3 = calculate_op(c, slot3, phase_mod1);
								output2 = calculate_op(c, slot2, OP_INPUT_NONE);
								output4 = calculate_op(c, slot4, OP_INPUT_NONE);
								break;

							//           --[S4]-----|
							//           --[S2]-----|
							// <-----------------|  |
							// +--[S1]--+--[S3]--|--+-->
							case 11:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(c, slot3, phase_mod1);
								set_feedback(c, slot1, phase_mod3);
								output3 = phase_mod3;
								output2 = calculate_op(c, slot2, OP_INPUT_NONE);
								output4 = calculate_op(c, slot4, OP_INPUT_NONE);
								break;

							//             |--+--[S4]--|
							// <--------|  |--+--[S3]--|
							// +--[S1]--|--|--+--[S2]--+-->
							case 12:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output3 = calculate_op(c, slot3, phase_mod1);
								output2 = calculate_op(c, slot2, phase_mod1);
								output4 = calculate_op(c, slot4, phase_mod1);
								break;

							//  --[S3]--+--[S2]--|
							//                   |
							//  --[S4]-----------|
							// <--------|        |
							// +--[S1]--|--------+-->
							case 13:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output1 = phase_mod1;
								phase_mod3 = calculate_op(c, slot3, OP_INPUT_NONE);
								output2 = calculate_op(c, slot2, phase_mod3);
								output4 = calculate_op(c, slot4, OP_INPUT_NONE);
								break;

							//  --[S2]-----+--[S4]--|
							//                      |
							// <--------|  +--[S3]--|
							// +--[S1]--|--|--------+-->
							case 14:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output1 = phase_mod1;
								output3 = calculate_op(c, slot3, phase_mod1);
								phase_mod2 = calculate_op(c, slot2, OP_INPUT_NONE);
								output4 = calculate_op(c, slot4, phase_mod2);
								break;

							//  --[S4]-----|
							//  --[S2]-----|
							//  --[S3]-----|
							// <--------|  |
							// +--[S1]--|--+-->
							case 15:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output1 = phase_mod1;
								output3 = calculate_op(c, slot3, OP_INPUT_NONE);
								output2 = calculate_op(c, slot2, OP_INPUT_NONE);
								output4 = calculate_op(c, slot4, OP_INPUT_NONE);
								break;
						}

						for (int k = 0; k < 4; k++)
							*mixp++ += (int32_t)((MULA(output1, a1[k]) + MULA(output2, a2[k]) + MULA(output3, a3[k]) +
							                      MULA(output4, a4[k])) >> 16);
					}
				}
				break;
			}

			// 2x 2 operator FM
			case 1:
			{
				for (int op = 0; op < 2; op++)
				{
					int slot1 = j + ((op + 0) * 12);
					int slot3 = j + ((op + 2) * 12);

					mixp = mix;
					if (c->slots[slot1].active)
					{
						ATT4(a1, slot1);
						ATT4(a3, slot3);
						for (int i = 0; i < samples; i++)
						{
							int64_t output1 = 0, output3 = 0;
							int64_t phase_mod1, phase_mod3;
							switch (c->slots[slot1].algorithm & 3)
							{
								// <--------|
								// +--[S1]--|--+--[S3]-->
								case 0:
									phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
									set_feedback(c, slot1, phase_mod1);
									output3 = calculate_op(c, slot3, phase_mod1);
									break;

								// <-----------------|
								// +--[S1]--+--[S3]--|-->
								case 1:
									phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
									phase_mod3 = calculate_op(c, slot3, phase_mod1);
									set_feedback(c, slot1, phase_mod3);
									output3 = phase_mod3;
									break;

								//  --[S3]-----|
								// <--------|  |
								// +--[S1]--|--+-->
								case 2:
									phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
									set_feedback(c, slot1, phase_mod1);
									output1 = phase_mod1;
									output3 = calculate_op(c, slot3, OP_INPUT_NONE);
									break;
								//
								// <--------|  +--[S3]--|
								// +--[S1]--|--|--------+-->
								case 3:
									phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
									set_feedback(c, slot1, phase_mod1);
									output1 = phase_mod1;
									output3 = calculate_op(c, slot3, phase_mod1);
									break;
							}

							for (int k = 0; k < 4; k++)
								*mixp++ += (int32_t)((MULA(output1, a1[k]) + MULA(output3, a3[k])) >> 16);
						}
					}
				}
				break;
			}

			// 3 operator FM + PCM
			case 2:
			{
				int slot1 = j + (0*12);
				int slot2 = j + (1*12);
				int slot3 = j + (2*12);
				mixp = mix;

				if (c->slots[slot1].active)
				{
					ATT4(a1, slot1);
					ATT4(a2, slot2);
					ATT4(a3, slot3);
					for (int i = 0; i < samples; i++)
					{
						int64_t output1 = 0, output2 = 0, output3 = 0;
						int64_t phase_mod1, phase_mod3;
						switch (c->slots[slot1].algorithm & 7)
						{
							// <--------|
							// +--[S1]--|--+--[S3]--+--[S2]-->
							case 0:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								phase_mod3 = calculate_op(c, slot3, phase_mod1);
								output2 = calculate_op(c, slot2, phase_mod3);
								break;

							// <-----------------|
							// +--[S1]--+--[S3]--|--+--[S2]-->
							case 1:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(c, slot3, phase_mod1);
								set_feedback(c, slot1, phase_mod3);
								output2 = calculate_op(c, slot2, phase_mod3);
								break;

							//  --[S3]-----|
							// <--------|  |
							// +--[S1]--|--+--[S2]-->
							case 2:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								phase_mod3 = calculate_op(c, slot3, OP_INPUT_NONE);
								output2 = calculate_op(c, slot2, (phase_mod1 + phase_mod3) / 1);
								break;

							//  --[S3]--+--[S2]--|
							// <--------|        |
							// +--[S1]--|--------+-->
							case 3:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output1 = phase_mod1;
								phase_mod3 = calculate_op(c, slot3, OP_INPUT_NONE);
								output2 = calculate_op(c, slot2, phase_mod3);
								break;

							//              --[S2]--|
							// <--------|           |
							// +--[S1]--|--+--[S3]--+-->
							case 4:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output3 = calculate_op(c, slot3, phase_mod1);
								output2 = calculate_op(c, slot2, OP_INPUT_NONE);
								break;

							//              --[S2]--|
							// <-----------------|  |
							// +--[S1]--+--[S3]--|--+-->
							case 5:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(c, slot3, phase_mod1);
								set_feedback(c, slot1, phase_mod3);
								output3 = phase_mod3;
								output2 = calculate_op(c, slot2, OP_INPUT_NONE);
								break;

							//  --[S2]-----|
							//  --[S3]-----|
							// <--------|  |
							// +--[S1]--|--+-->
							case 6:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output1 = phase_mod1;
								output3 = calculate_op(c, slot3, OP_INPUT_NONE);
								output2 = calculate_op(c, slot2, OP_INPUT_NONE);
								break;

							//              --[S2]--|
							// <--------|  +--[S3]--|
							// +--[S1]--|--|--------+-->
							case 7:
								phase_mod1 = calculate_op(c, slot1, OP_INPUT_FEEDBACK);
								set_feedback(c, slot1, phase_mod1);
								output1 = phase_mod1;
								output3 = calculate_op(c, slot3, phase_mod1);
								output2 = calculate_op(c, slot2, OP_INPUT_NONE);
								break;
						}

						for (int k = 0; k < 4; k++)
							*mixp++ += (int32_t)((MULA(output1, a1[k]) + MULA(output2, a2[k]) + MULA(output3, a3[k])) >> 16);
					}
				}

				mixp = mix;
				update_pcm(c, j + (3*12), mixp, samples);
				break;
			}

			// PCM
			case 3:
			{
				update_pcm(c, j + (0*12), mixp, samples);
				update_pcm(c, j + (1*12), mixp, samples);
				update_pcm(c, j + (2*12), mixp, samples);
				update_pcm(c, j + (3*12), mixp, samples);
				break;
			}
		}
	}

}

static void write_register(ymf271_t *c, int slotnum, int reg, uint8_t data)
{
	YMF271Slot *slot = &c->slots[slotnum];

	switch (reg)
	{
		case 0x0:
			slot->ext_en = (data & 0x80) ? 1 : 0;
			slot->ext_out = (data >> 3) & 0xf;

			if (data & 1)
			{
				// key on
				slot->step = 0;
				slot->stepptr = 0;

				slot->active = 1;

				calculate_step(c, slot);
				calculate_status_end(c, slotnum,false);
				init_envelope(c, slot);
				init_lfo(c, slot);
				slot->feedback_modulation0 = 0;
				slot->feedback_modulation1 = 0;
			}
			else
			{
				if (slot->active)
				{
					//calculate_status_end(c, slotnum,true); status changes if keyoff? verify this from real hardware.
					slot->env_state = ENV_RELEASE;
				}
			}
			break;

		case 0x1:
			slot->lfoFreq = data;
			break;

		case 0x2:
			slot->lfowave = data & 3;
			slot->pms = (data >> 3) & 0x7;
			slot->ams = (data >> 6) & 0x3;
			break;

		case 0x3:
			slot->multiple = data & 0xf;
			slot->detune = (data >> 4) & 0x7;
			break;

		case 0x4:
			slot->tl = data & 0x7f;
			break;

		case 0x5:
			slot->ar = data & 0x1f;
			slot->keyscale = (data >> 5) & 0x7;
			break;

		case 0x6:
			slot->decay1rate = data & 0x1f;
			break;

		case 0x7:
			slot->decay2rate = data & 0x1f;
			break;

		case 0x8:
			slot->relrate = data & 0xf;
			slot->decay1lvl = (data >> 4) & 0xf;
			break;

		case 0x9:
			// write frequency and block here
			slot->fns = (slot->fns_hi << 8 & 0x0f00) | data;
			slot->block = slot->fns_hi >> 4 & 0xf;
			break;

		case 0xa:
			slot->fns_hi = data;
			break;

		case 0xb:
			slot->waveform = data & 0x7;
			slot->feedback = (data >> 4) & 0x7;
			slot->accon = (data & 0x80) ? 1 : 0;
			break;

		case 0xc:
			slot->algorithm = data & 0xf;
			break;

		case 0xd:
			slot->ch0_level = data >> 4;
			slot->ch1_level = data & 0xf;
			break;

		case 0xe:
			slot->ch2_level = data >> 4;
			slot->ch3_level = data & 0xf;
			break;

		default:
			break;
	}
}

static void ymf271_write_fm(ymf271_t *c, int bank, uint8_t address, uint8_t data)
{
	int groupnum = fm_tab[address & 0xf];
	if (groupnum == -1)
	{
		logerror("ymf271_write_fm invalid group %02X %02X\n", address, data);
		return;
	}

	int reg = (address >> 4) & 0xf;

	// check if the register is a synchronized register
	int sync_reg = 0;
	switch (reg)
	{
		case 0:
		case 9:
		case 10:
		case 12:
		case 13:
		case 14:
			sync_reg = 1;
			break;

		default:
			break;
	}

	// check if the slot is key on slot for synchronizing
	int sync_mode = 0;
	switch (c->groups[groupnum].sync)
	{
		// 4 slot mode
		case 0:
			if (bank == 0)
				sync_mode = 1;
			break;

		// 2x 2 slot mode
		case 1:
			if (bank == 0 || bank == 1)
				sync_mode = 1;
			break;

		// 3 slot + 1 slot mode
		case 2:
			if (bank == 0)
				sync_mode = 1;
			break;

		default:
			break;
	}

	// key-on slot & synced register
	if (sync_mode && sync_reg)
	{
		switch (c->groups[groupnum].sync)
		{
			// 4 slot mode
			case 0:
				write_register(c, (12 * 0) + groupnum, reg, data);
				write_register(c, (12 * 1) + groupnum, reg, data);
				write_register(c, (12 * 2) + groupnum, reg, data);
				write_register(c, (12 * 3) + groupnum, reg, data);
				break;

			// 2x 2 slot mode
			case 1:
				if (bank == 0)
				{
					// Slot 1 - Slot 3
					write_register(c, (12 * 0) + groupnum, reg, data);
					write_register(c, (12 * 2) + groupnum, reg, data);
				}
				else
				{
					// Slot 2 - Slot 4
					write_register(c, (12 * 1) + groupnum, reg, data);
					write_register(c, (12 * 3) + groupnum, reg, data);
				}
				break;

			// 3 slot + 1 slot mode (1 slot is handled normally)
			case 2:
				write_register(c, (12 * 0) + groupnum, reg, data);
				write_register(c, (12 * 1) + groupnum, reg, data);
				write_register(c, (12 * 2) + groupnum, reg, data);
				break;
		}
	}
	else
	{
		// write register normally
		write_register(c, (12 * bank) + groupnum, reg, data);
	}
}

static void ymf271_write_pcm(ymf271_t *c, uint8_t address, uint8_t data)
{
	int slotnum = pcm_tab[address & 0xf];
	if (slotnum == -1)
	{
		logerror("ymf271_write_pcm invalid slot %02X %02X\n", address, data);
		return;
	}
	YMF271Slot *slot = &c->slots[slotnum];

	switch (address >> 4 & 0xf)
	{
		case 0x0:
			slot->startaddr &= ~0xff;
			slot->startaddr |= data;
			break;

		case 0x1:
			slot->startaddr &= ~0xff00;
			slot->startaddr |= data<<8;
			break;

		case 0x2:
			slot->startaddr &= ~0xff0000;
			slot->startaddr |= (data & 0x7f)<<16;
			slot->altloop = (data & 0x80) ? 1 : 0;
			if (slot->altloop)
				popmessage("ymf271 A/L, contact MAMEdev");
			break;

		case 0x3:
			slot->endaddr &= ~0xff;
			slot->endaddr |= data;
			break;

		case 0x4:
			slot->endaddr &= ~0xff00;
			slot->endaddr |= data<<8;
			break;

		case 0x5:
			slot->endaddr &= ~0xff0000;
			slot->endaddr |= (data & 0x7f)<<16;
			break;

		case 0x6:
			slot->loopaddr &= ~0xff;
			slot->loopaddr |= data;
			break;

		case 0x7:
			slot->loopaddr &= ~0xff00;
			slot->loopaddr |= data<<8;
			break;

		case 0x8:
			slot->loopaddr &= ~0xff0000;
			slot->loopaddr |= (data & 0x7f)<<16;
			break;

		case 0x9:
			slot->fs = data & 0x3;
			slot->bits = (data & 0x4) ? 12 : 8;
			slot->srcnote = (data >> 3) & 0x3;
			slot->srcb = (data >> 5) & 0x7;
			break;

		default:
			break;
	}
}

static void timer_a_expired(ymf271_t *c)
{
	c->status |= 1;

	// assert IRQ
	if (c->enable & 4)
	{
		c->irqstate |= 1;
		irq_set(c, 1);
	}

	// reload timer
	c->timer(c->user, 0, 384 * (1024 - c->timerA));
}

static void timer_b_expired(ymf271_t *c)
{
	c->status |= 2;

	// assert IRQ
	if (c->enable & 8)
	{
		c->irqstate |= 2;
		irq_set(c, 1);
	}

	// reload timer
	c->timer(c->user, 1, 384 * 16 * (256 - c->timerB));
}

static void ymf271_write_timer(ymf271_t *c, uint8_t address, uint8_t data)
{
	if ((address & 0xf0) == 0)
	{
		int groupnum = fm_tab[address & 0xf];
		if (groupnum == -1)
		{
			logerror("ymf271_write_timer invalid group %02X %02X\n", address, data);
			return;
		}
		YMF271Group *group = &c->groups[groupnum];

		group->sync = data & 0x3;
		group->pfm = data >> 7;
	}
	else
	{
		switch (address)
		{
			case 0x10:
				c->timerA = (c->timerA & 0x003) | (data << 2); // High 8 bit of Timer A period
				break;

			case 0x11:
				// Timer A is 10 bit, split high 8 bit and low 2 bit like other Yamaha FM chips
				// unlike Yamaha's documentation; it says 0x11 writes timer A upper 2 bits.
				c->timerA = (c->timerA & 0x3fc) | (data & 0x03); // Low 2 bit of Timer A period
				break;

			case 0x12:
				c->timerB = data;
				break;

			case 0x13:
				// timer A load
				if (~c->enable & data & 1)
				{
					const int64_t period = 384 * (1024 - c->timerA);
					c->timer(c->user, 0, (data & 1) ? period : -1);
				}

				// timer B load
				if (~c->enable & data & 2)
				{
					const int64_t period = 384 * 16 * (256 - c->timerB);
					c->timer(c->user, 1, (data & 2) ? period : -1);
				}

				// timer A reset
				if (data & 0x10)
				{
					c->irqstate &= ~1;
					c->status &= ~1;

					if (~c->irqstate & 2)
						irq_set(c, 0);
				}

				// timer B reset
				if (data & 0x20)
				{
					c->irqstate &= ~2;
					c->status &= ~2;

					if (~c->irqstate & 1)
						irq_set(c, 0);
				}

				c->enable = data;
				break;

			case 0x14:
				c->ext_address &= ~0xff;
				c->ext_address |= data;
				break;

			case 0x15:
				c->ext_address &= ~0xff00;
				c->ext_address |= data << 8;
				break;

			case 0x16:
				c->ext_address &= ~0xff0000;
				c->ext_address |= (data & 0x7f) << 16;
				c->ext_rw = (data & 0x80) ? 1 : 0;
				break;

			case 0x17:
				c->ext_address = (c->ext_address + 1) & 0x7fffff;
				if (!c->ext_rw)
					c->write(c->user, c->ext_address, data);
				break;

			case 0x20:
			case 0x21:
			case 0x22:
				// test
				break;

			default:
				break;
		}
	}
}

static void render_full(ymf271_t *c, int32_t *mix, int samples) { render_groups(c, mix, samples, 0, 1); }

static void write_core(ymf271_t *c, int offset, uint8_t data)
{
	c->step_gen++;

	offset &= 0xf;
	c->regs_main[offset] = data;

	switch (offset)
	{
		case 0x0:
		case 0x2:
		case 0x4:
		case 0x6:
		case 0x8:
		case 0xc:
			// address regs
			break;

		case 0x1:
			ymf271_write_fm(c, 0, c->regs_main[0x0], data);
			break;

		case 0x3:
			ymf271_write_fm(c, 1, c->regs_main[0x2], data);
			break;

		case 0x5:
			ymf271_write_fm(c, 2, c->regs_main[0x4], data);
			break;

		case 0x7:
			ymf271_write_fm(c, 3, c->regs_main[0x6], data);
			break;

		case 0x9:
			ymf271_write_pcm(c, c->regs_main[0x8], data);
			break;

		case 0xd:
			ymf271_write_timer(c, c->regs_main[0xc], data);
			break;

		default:
			break;
	}
}

uint8_t ymf271_read(ymf271_t *c, int offset)
{
	offset &= 0xf;

	// (MAME updates the stream before reading the status registers: the owner renders up to now first)

	switch (offset)
	{
		case 0x0:
			return c->status | ((c->end_status & 0xf) << 3);

		case 0x1:
			// statusreg 2
			return c->end_status >> 4;

		case 0x2:
		{
			if (!c->ext_rw)
				return 0xff;

			uint8_t ret = c->ext_readlatch;
			c->ext_address = (c->ext_address + 1) & 0x7fffff;
			c->ext_readlatch = c->read(c->user, c->ext_address);
			return ret;
		}

		default:
			break;
	}

	return 0xff;
}

static void init_tables(ymf271_t *c)
{
	for (int i = 0; i < SIN_LEN; i++)
	{
		double m = sin( ((i*2)+1) * M_PI / SIN_LEN );
		double m2 = sin( ((i*4)+1) * M_PI / SIN_LEN );

		// Waveform 0: sin(wt)    (0 <= wt <= 2PI)
		c->lut_waves[0][i] = (int16_t)(m * MAXOUT);

		// Waveform 1: sin?(wt)   (0 <= wt <= PI)     -sin?(wt)  (PI <= wt <= 2PI)
		c->lut_waves[1][i] = (i < (SIN_LEN/2)) ? (int16_t)((m * m) * MAXOUT) : (int16_t)((m * m) * MINOUT);

		// Waveform 2: sin(wt)    (0 <= wt <= PI)     -sin(wt)   (PI <= wt <= 2PI)
		c->lut_waves[2][i] = (i < (SIN_LEN/2)) ? (int16_t)(m * MAXOUT) : (int16_t)(-m * MAXOUT);

		// Waveform 3: sin(wt)    (0 <= wt <= PI)     0
		c->lut_waves[3][i] = (i < (SIN_LEN/2)) ? (int16_t)(m * MAXOUT) : 0;

		// Waveform 4: sin(2wt)   (0 <= wt <= PI)     0
		c->lut_waves[4][i] = (i < (SIN_LEN/2)) ? (int16_t)(m2 * MAXOUT) : 0;

		// Waveform 5: |sin(2wt)| (0 <= wt <= PI)     0
		c->lut_waves[5][i] = (i < (SIN_LEN/2)) ? (int16_t)(fabs(m2) * MAXOUT) : 0;

		// Waveform 6:     1      (0 <= wt <= 2PI)
		c->lut_waves[6][i] = (int16_t)(1 * MAXOUT);

		c->lut_waves[7][i] = 0;
	}

	for (int i = 0; i < LFO_LENGTH; i++)
	{
		int tri_wave;
		double ftri_wave, fsaw_wave;
		double plfo[4];

		// LFO phase modulation
		plfo[0] = 0;

		fsaw_wave = ((i % (LFO_LENGTH/2)) * PLFO_MAX) / (double)((LFO_LENGTH/2)-1);
		plfo[1] = (i < (LFO_LENGTH/2)) ? fsaw_wave : fsaw_wave - PLFO_MAX;

		plfo[2] = (i < (LFO_LENGTH/2)) ? PLFO_MAX : PLFO_MIN;

		ftri_wave = ((i % (LFO_LENGTH/4)) * PLFO_MAX) / (double)(LFO_LENGTH/4);
		switch (i / (LFO_LENGTH/4))
		{
			case 0: plfo[3] = ftri_wave; break;
			case 1: plfo[3] = PLFO_MAX - ftri_wave; break;
			case 2: plfo[3] = 0 - ftri_wave; break;
			case 3: plfo[3] = 0 - (PLFO_MAX - ftri_wave); break;
			default: plfo[3] = 0; break;
		}

		for (int j = 0; j < 4; j++)
		{
			c->lut_plfo[j][0][i] = pow(2.0, 0.0);
			c->lut_plfo[j][1][i] = pow(2.0, (3.378 * plfo[j]) / 1200.0);
			c->lut_plfo[j][2][i] = pow(2.0, (5.0646 * plfo[j]) / 1200.0);
			c->lut_plfo[j][3][i] = pow(2.0, (6.7495 * plfo[j]) / 1200.0);
			c->lut_plfo[j][4][i] = pow(2.0, (10.1143 * plfo[j]) / 1200.0);
			c->lut_plfo[j][5][i] = pow(2.0, (20.1699 * plfo[j]) / 1200.0);
			c->lut_plfo[j][6][i] = pow(2.0, (40.1076 * plfo[j]) / 1200.0);
			c->lut_plfo[j][7][i] = pow(2.0, (79.307 * plfo[j]) / 1200.0);
		}

		// LFO amplitude modulation
		c->lut_alfo[0][i] = 0;

		c->lut_alfo[1][i] = ALFO_MAX - ((i * ALFO_MAX) / LFO_LENGTH);

		c->lut_alfo[2][i] = (i < (LFO_LENGTH/2)) ? ALFO_MAX : ALFO_MIN;

		tri_wave = ((i % (LFO_LENGTH/2)) * ALFO_MAX) / (LFO_LENGTH/2);
		c->lut_alfo[3][i] = (i < (LFO_LENGTH/2)) ? ALFO_MAX-tri_wave : tri_wave;
	}

	for (int i = 0; i < 256; i++)
	{
		c->lut_env_volume[i] = (int)(65536.0 / pow(10.0, ((double)i / (256.0 / 96.0)) / 20.0));
	}

	for (int i = 0; i < 16; i++)
	{
		c->lut_attenuation[i] = (int)(65536.0 / pow(10.0, channel_attenuation_table[i] / 20.0));
	}
	for (int i = 0; i < 128; i++)
	{
		double db = 0.75 * (double)i;
		c->lut_total_level[i] = (int)(65536.0 / pow(10.0, db / 20.0));
	}
}

static void calculate_clock_correction(ymf271_t *c)
{
	// timing may use a non-standard XTAL
	double clock_correction = (c->master_clock != 0) ? (double)(STD_CLOCK) / (double)c->master_clock : 0.0;
	for (int i = 0; i < 256; i++)
	{
		c->lut_lfo[i] = LFO_frequency_table[i] * clock_correction;
	}

	for (int i = 0; i < 64; i++)
	{
		// attack rate in number of samples
		c->lut_ar[i] = (ARTime[i] * clock_correction * 44100.0) / 1000.0;
	}
	for (int i = 0; i < 64; i++)
	{
		// decay/release rate in number of samples
		c->lut_dc[i] = (DCTime[i] * clock_correction * 44100.0) / 1000.0;
	}
}


// timer_a_expired / timer_b_expired
void ymf271_timer_expired(ymf271_t *c, int which)
{
	if (which)
		timer_b_expired(c);
	else
		timer_a_expired(c);
}

// device_start (the stream and timers are the owner's) with device_clock_changed's clock correction
ymf271_t *ymf271_create(uint32_t clock, uint8_t (*read)(void *user, uint32_t addr),
	void (*write)(void *user, uint32_t addr, uint8_t data), void (*irq)(void *user, int state),
	void (*timer)(void *user, int which, int64_t clocks), void *user)
{
	ymf271_t *c = (ymf271_t *)calloc(1, sizeof(ymf271_t));
	if (!c)
		return NULL;
	c->read = read;
	c->write = write;
	c->irq = irq;
	c->timer = timer;
	c->user = user;
	c->master_clock = clock;
	init_tables(c);
	calculate_clock_correction(c);
	return c;
}

void ymf271_destroy(ymf271_t *c)
{
	if (c && c->wk) {
		for (int i = 0; i < 2; i++) {
			free(c->wq[i].w);
			free(c->wq[i].out);
		}
		free(c->q);
	}
	free(c);
}

void ymf271_set_rom(ymf271_t *c, const uint8_t *rom, uint32_t size)
{
	const int ok = rom && size;
	c->rom = ok ? rom : NULL;
	c->rom_mask = ok && !(size & (size - 1)) ? size - 1 : 0;
	c->rom_size = ok ? size : 0;
	for (int i = 0; i < c->nwk; i++)   // (the workers' copies too: the owner calls it with the workers idle)
		ymf271_set_rom(c->wq[i].w, rom, size);
}

// the constructor's zeroed state, device_reset
void ymf271_reset(ymf271_t *c)
{
	const uint32_t clock = c->master_clock;
	memset(c, 0, YMF271_STATE_SIZE);
	c->step_gen++;
	c->master_clock = clock;

	for (int i = 0; i < 48; i++)
	{
		c->slots[i].active = 0;
		c->slots[i].volume = 0;
	}

	// reset timers and IRQ
	if (c->timer)
	{
		c->timer(c->user, 0, -1);
		c->timer(c->user, 1, -1);
	}

	c->irqstate = 0;
	c->status = 0;
	c->enable = 0;

	irq_set(c, 0);
	if (c->wk) {   // (the workers' copies reset too, after what they were given before)
		c->sync(c->sync_user);
		for (int i = 0; i < c->nwk; i++) {
			memcpy(c->wq[i].w, c, YMF271_STATE_SIZE);
			c->wq[i].w->step_gen++;
		}
	}
}

// ---- async: the samples made by the worker's copy of the chip (ymf271_set_async) ----
// The owner's chip takes every write (its timers, status and IRQ stay exact) and, for the samples, runs only what the
// status reads can see: the PCM slots' positions, loop ends (the end flags), envelopes (a slot going quiet stops its
// steps) and LFOs (they change a PCM slot's step). The worker's copy is given the same writes and sample counts in
// order and makes the full samples (FM, the PCM samples, levels, mixing); the timers' only effects are on the status
// and IRQ, so its copy has none. A save first gives the owner's chip the worker's slots (the full state).
enum { Q_RENDER = 0, Q_WRITE = 1 };
static void timer_none(void *user, int which, int64_t clocks) { (void)user; (void)which; (void)clocks; }
static void write_none(void *user, uint32_t addr, uint8_t data) { (void)user; (void)addr; (void)data; }

// update_lfo without the amplitude (the level's part: the owner makes no samples; a save takes the workers' slots)
YMF_INLINE void update_lfo_step(ymf271_t *c, YMF271Slot *slot)
{
	slot->lfo_phase += slot->lfo_step;
	const double pm = c->lut_plfo[slot->lfowave][slot->pms][(slot->lfo_phase >> LFO_SHIFT) & (LFO_LENGTH-1)];
	const int sn = (int)(slot - c->slots);
	if (pm != slot->lfo_phasemod || c->step_gen_slot[sn] != c->step_gen)
	{
		slot->lfo_phasemod = pm;
		calculate_step(c, slot);
		c->step_gen_slot[sn] = c->step_gen;
	}
}

// update_pcm without the sample, its level and the mixing: the same steps, loops, end flags, envelope and LFO step
static void pcm_light(ymf271_t *c, int slotnum, int length)
{
	YMF271Slot *slot = &c->slots[slotnum];
	if (!slot->active)
		return;
	for (int i = 0; i < length; i++)
	{
		if ((slot->stepptr>>16) > slot->endaddr)
		{
			slot->stepptr = slot->stepptr - ((uint64_t)slot->endaddr<<16) + ((uint64_t)slot->loopaddr<<16);
			calculate_status_end(c, slotnum,true);
			if ((slot->stepptr>>16) > slot->endaddr)
			{
				slot->stepptr &= 0xffff;
				slot->stepptr |= ((uint64_t)slot->loopaddr<<16);
				if ((slot->stepptr>>16) > slot->endaddr)
				{
					slot->stepptr &= 0xffff;
					slot->stepptr |= ((uint64_t)slot->endaddr<<16);
				}
			}
		}
		update_envelope(c, slot);
		update_lfo_step(c, slot);
		slot->stepptr += slot->step;
	}
}

static void q_push(ymf271_t *c, uint32_t v)
{
	const uint32_t h = c->qh;
	if (h - __atomic_load_n(&c->wq[0].qt, __ATOMIC_ACQUIRE) >= c->qcap ||
	    (c->nwk > 1 && h - __atomic_load_n(&c->wq[1].qt, __ATOMIC_ACQUIRE) >= c->qcap))
		c->sync(c->sync_user);   // (full: the workers empty it)
	c->q[h & (c->qcap - 1)] = v;
	__atomic_store_n(&c->qh, h + 1, __ATOMIC_RELEASE);
}

static ymf271_t *worker_copy(const ymf271_t *c)
{
	ymf271_t *w = (ymf271_t *)malloc(sizeof(ymf271_t));
	if (!w)
		return NULL;
	memcpy(w, c, sizeof(*w));
	w->wk = NULL;
	w->nwk = 0;
	w->q = NULL;
	for (int i = 0; i < 2; i++) {
		w->wq[i].w = NULL;
		w->wq[i].out = NULL;
	}
	w->timer = timer_none;
	w->irq = NULL;
	w->write = write_none;
	w->step_gen++;
	return w;
}

int ymf271_set_async(ymf271_t *c, int workers, void (*kick)(void *), void (*sync)(void *), void *user)
{
	if (c->wk)
		return 0;   // (the workers read the PCM samples: from the ROM, or read() which the owner makes safe there)
	if (workers != 2)
		workers = 1;
	c->qcap = 1 << 14;
	c->q = (uint32_t *)malloc(sizeof(uint32_t) * c->qcap);
	c->out_cap = 1 << 13;
	int ok = c->q != NULL;
	for (int i = 0; i < workers; i++) {
		c->wq[i].w = ok ? worker_copy(c) : NULL;
		c->wq[i].out = (int32_t *)malloc(sizeof(int32_t) * 4 * (size_t)c->out_cap);
		ok = ok && c->wq[i].w && c->wq[i].out;
		c->wq[i].qt = 0;
		c->wq[i].out_n = 0;
		c->wq[i].g0 = i;   // (two workers: the even groups and the odd ones, about the same load)
		c->wq[i].g1 = workers;   // (the step)
	}
	if (!ok) {
		for (int i = 0; i < 2; i++) {
			free(c->wq[i].w);
			free(c->wq[i].out);
			c->wq[i].w = NULL;
			c->wq[i].out = NULL;
		}
		free(c->q);
		c->q = NULL;
		return -1;
	}
	c->qh = 0;
	c->kick = kick;
	c->sync = sync;
	c->sync_user = user;
	c->nwk = workers;
	c->wk = c->wq[0].w;
	return 0;
}

void ymf271_async_job(ymf271_t *c, int which)
{
	ymf271_wq *q = &c->wq[which];
	uint32_t t = q->qt;
	const uint32_t h = __atomic_load_n(&c->qh, __ATOMIC_ACQUIRE);
	while (t != h) {
		const uint32_t v = c->q[t & (c->qcap - 1)];
		if ((v >> 30) == Q_WRITE)
			write_core(q->w, (int)(v >> 8 & 0xf), (uint8_t)v);
		else {
			int n = (int)(v & 0xffffff);
			if (q->out_n + n > c->out_cap)
				n = c->out_cap - q->out_n;
			if (n > 0) {
				render_groups(q->w, q->out + (size_t)q->out_n * 4, n, q->g0, q->g1);
				q->out_n += n;
			}
		}
		t++;
		__atomic_store_n(&q->qt, t, __ATOMIC_RELEASE);
	}
}

int ymf271_async_take(ymf271_t *c, int32_t *mix, int samples)
{
	c->sync(c->sync_user);
	const int n = c->wq[0].out_n < samples ? c->wq[0].out_n : samples;
	memcpy(mix, c->wq[0].out, sizeof(int32_t) * 4 * (size_t)n);
	if (c->nwk > 1)   // (the second worker's groups added: the same sums as one pass over all)
		for (int i = 0; i < n * 4; i++)
			mix[i] += c->wq[1].out[i];
	if (n < samples)
		memset(mix + (size_t)n * 4, 0, sizeof(int32_t) * 4 * (size_t)(samples - n));
	for (int i = 0; i < c->nwk; i++)
		c->wq[i].out_n = 0;
	return n;
}

void ymf271_render(ymf271_t *c, int32_t *mix, int samples)
{
	if (!c->wk) {
		render_full(c, mix, samples);
		return;
	}
	for (int j = 0; j < 12; j++) {   // (the PCM slots as render_full runs them)
		if (c->groups[j].sync == 2)
			pcm_light(c, j + 3 * 12, samples);
		else if (c->groups[j].sync == 3)
			for (int k = 0; k < 4; k++)
				pcm_light(c, j + k * 12, samples);
	}
	q_push(c, (uint32_t)Q_RENDER << 30 | (uint32_t)samples);
	c->kick(c->sync_user);
}

void ymf271_write(ymf271_t *c, int offset, uint8_t data)
{
	write_core(c, offset, data);
	if (c->wk)
		q_push(c, (uint32_t)Q_WRITE << 30 | (uint32_t)(offset & 0xf) << 8 | data);
}

void ymf271_state_save_prep(ymf271_t *c)
{
	if (!c->wk)
		return;
	c->sync(c->sync_user);
	for (int i = 0; i < c->nwk; i++)   // (each worker's own groups' slots: the full state)
		for (int j = c->wq[i].g0; j < 12; j += c->wq[i].g1)
			for (int k = 0; k < 4; k++)
				c->slots[j + k * 12] = c->wq[i].w->slots[j + k * 12];
	memcpy(c->groups, c->wk->groups, sizeof(c->groups));
	c->step_gen++;
}

void ymf271_state_loaded(ymf271_t *c)
{
	c->step_gen++;
	if (c->wk) {
		c->sync(c->sync_user);
		for (int i = 0; i < c->nwk; i++) {
			memcpy(c->wq[i].w, c, YMF271_STATE_SIZE);
			c->wq[i].w->step_gen++;
		}
	}
}
