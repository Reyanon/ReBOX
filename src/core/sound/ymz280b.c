/*  Yamaha YMZ280B PCMD8 (8-voice ADPCM / PCM8 / PCM16 player), ported from MAME 0.289
    src/devices/sound/ymz280b.cpp / ymz280b.h (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "ymz280b.h"

#include <string.h>

#define FRAC_BITS 9
#define FRAC_ONE  (1 << FRAC_BITS)

// output samples per internal block: one block needs at most CHUNK + 1 decoded samples
// (output_step <= FRAC_ONE); MAME splits its updates at arbitrary points as well
#define CHUNK 1024

// step size index shift table
static const int s_index_scale[8] = { 0x0e6, 0x0e6, 0x0e6, 0x0e6, 0x133, 0x199, 0x200, 0x266 };

// MAME compute_tables(): (nib & 7) * 2 + 1, negative when bit 3 is set
static const int s_diff_lookup[16] = {
	1, 3, 5, 7, 9, 11, 13, 15, -1, -3, -5, -7, -9, -11, -13, -15
};

// x / 7 for 0 <= x <= 255 * 7 (exact in that range; no divide on the Cortex-A9)
static inline int div7(int x)
{
	return (x * 9363) >> 16;
}

static inline uint8_t read_byte(const ymz280b_t *c, uint32_t a)
{
	a &= 0xffffff & c->rom_mask;
	return a < c->rom_size ? c->rom[a] : 0;
}

static void update_irq_state(ymz280b_t *c)
{
	int irq_bits = c->status_register & c->irq_mask;

	// always off if the enable is off
	if (!c->irq_enable)
		irq_bits = 0;

	if (irq_bits && !c->irq_state) {
		c->irq_state = 1;
		if (c->irq)
			c->irq(c->user, 1);
	} else if (!irq_bits && c->irq_state) {
		c->irq_state = 0;
		if (c->irq)
			c->irq(c->user, 0);
	}
}

static void update_step(ymz280b_voice_t *voice)
{
	int frequency;
	if (voice->mode == 1)
		frequency = voice->fnum & 0x0ff;
	else
		frequency = voice->fnum & 0x1ff;
	voice->output_step = frequency + 1;   // ((fnum + 1) * (input clock / 384)) / 256
}

static void update_volumes(ymz280b_voice_t *voice)
{
	if (voice->pan == 8) {
		voice->output_left = voice->level;
		voice->output_right = voice->level;
	} else if (voice->pan < 8) {
		voice->output_left = voice->level;
		// pan 1 is hard-left, what's pan 0? MAME: assume same as pan 1
		voice->output_right = (voice->pan == 0) ? 0 : div7(voice->level * (voice->pan - 1));
	} else {
		voice->output_left = div7(voice->level * (15 - voice->pan));
		voice->output_right = voice->level;
	}
}

// MAME update_irq_state_timer_common
static void irq_timer(ymz280b_t *c, int v)
{
	ymz280b_voice_t *voice = &c->v[v];
	if (!voice->irq_schedule)
		return;
	voice->playing = 0;
	c->status_register |= 1 << v;
	update_irq_state(c);
	voice->irq_schedule = 0;
}

/* ---- decoders: return the number of samples not generated ---- */

static inline int adpcm_nibble(const ymz280b_t *c, uint32_t position, int *signal, int *step)
{
	int val = read_byte(c, position >> 1) >> ((~position & 1) << 2);
	int d = *step * s_diff_lookup[val & 15];
	int s = *signal + (d >= 0 ? d >> 3 : -((-d) >> 3));   // C division by 8 (truncates)

	if (s > 32767)
		s = 32767;
	else if (s < -32768)
		s = -32768;

	int st = (*step * s_index_scale[val & 7]) >> 8;
	if (st > 0x6000)
		st = 0x6000;
	else if (st < 0x7f)
		st = 0x7f;

	*signal = s;
	*step = st;
	return s;
}

static int generate_adpcm(const ymz280b_t *c, ymz280b_voice_t *voice, int16_t *buffer, int samples)
{
	uint32_t position = voice->position;
	int signal = voice->signal;
	int step = voice->step;

	if (!voice->looping) {
		while (samples) {
			*buffer++ = (int16_t)adpcm_nibble(c, position, &signal, &step);
			samples--;
			position++;
			if (position >= voice->stop) {
				voice->ended = 1;
				break;
			}
		}
	} else {
		while (samples) {
			*buffer++ = (int16_t)adpcm_nibble(c, position, &signal, &step);
			samples--;
			position++;
			if (position == voice->loop_start && voice->loop_count == 0) {
				voice->loop_signal = signal;
				voice->loop_step = step;
			}
			if (position >= voice->loop_end) {
				if (voice->keyon) {
					position = voice->loop_start;
					signal = voice->loop_signal;
					step = voice->loop_step;
					voice->loop_count++;
				}
			}
			if (position >= voice->stop) {
				voice->ended = 1;
				break;
			}
		}
	}

	voice->position = position;
	voice->signal = signal;
	voice->step = step;
	return samples;
}

static int generate_pcm8(const ymz280b_t *c, ymz280b_voice_t *voice, int16_t *buffer, int samples)
{
	uint32_t position = voice->position;

	while (samples) {
		*buffer++ = (int16_t)((int8_t)read_byte(c, position >> 1) * 256);
		samples--;
		position += 2;
		if (voice->looping && position >= voice->loop_end) {
			if (voice->keyon)
				position = voice->loop_start;
		}
		if (position >= voice->stop) {
			voice->ended = 1;
			break;
		}
	}

	voice->position = position;
	return samples;
}

static int generate_pcm16(const ymz280b_t *c, ymz280b_voice_t *voice, int16_t *buffer, int samples)
{
	uint32_t position = voice->position;

	while (samples) {
		uint32_t a = position >> 1;
		*buffer++ = (int16_t)((read_byte(c, a + 1) << 8) + read_byte(c, a));
		samples--;
		position += 4;
		if (voice->looping && position >= voice->loop_end) {
			if (voice->keyon)
				position = voice->loop_start;
		}
		if (position >= voice->stop) {
			voice->ended = 1;
			break;
		}
	}

	voice->position = position;
	return samples;
}

/* ---- MAME sound_stream_update, for one voice and up to CHUNK output samples ---- */

static inline void out(int32_t *l, int32_t *r, int s, int lvol, int rvol)
{
	// MAME: add_int(interp * vol / 2, 32768 * 256) -> 16-bit scale is (interp * vol / 2) / 256
	*l += (s * lvol / 2) >> 8;
	*r += (s * rvol / 2) >> 8;
}

static void update_voice(ymz280b_t *c, int v, int32_t *left, int32_t *right, int remaining, int16_t *scratch)
{
	ymz280b_voice_t *voice = &c->v[v];
	int prev = voice->last_sample;
	int curr = voice->curr_sample;
	const int16_t *curr_data = scratch;
	int lvol = voice->output_left;
	int rvol = voice->output_right;
	const int ostep = voice->output_step;
	int pos;

	// quick out if we're not playing and we're at 0
	if (!voice->playing && curr == 0 && prev == 0) {
		// make sure next sound plays immediately
		voice->output_pos = FRAC_ONE;
		return;
	}

	pos = voice->output_pos;

	// finish off the current sample
	while (remaining > 0 && pos < FRAC_ONE) {
		int s = (prev * (FRAC_ONE - pos) + curr * pos) >> FRAC_BITS;
		out(left++, right++, s, lvol, rvol);
		pos += ostep;
		remaining--;
	}

	// if we're over, continue; otherwise, we're done
	if (pos >= FRAC_ONE)
		pos -= FRAC_ONE;
	else {
		voice->output_pos = pos;
		return;
	}

	// compute how many new samples we need
	uint32_t final_pos = (uint32_t)pos + (uint32_t)remaining * (uint32_t)ostep;
	int new_samples = (int)((final_pos + FRAC_ONE) >> FRAC_BITS);
	int samples_left;

	// generate them into our buffer
	switch (voice->playing << 7 | voice->mode) {
	case 0x81: samples_left = generate_adpcm(c, voice, scratch, new_samples); break;
	case 0x82: samples_left = generate_pcm8(c, voice, scratch, new_samples); break;
	case 0x83: samples_left = generate_pcm16(c, voice, scratch, new_samples); break;
	default:   samples_left = 0; memset(scratch, 0, (size_t)new_samples * sizeof(scratch[0])); break;
	}

	if (samples_left || voice->ended) {
		voice->ended = 0;

		// if there are leftovers, ramp back to 0
		int base = new_samples - samples_left;
		int t = (base == 0) ? curr : scratch[base - 1];
		for (int i = 0; i < samples_left; i++) {
			if (t < 0)
				t = -((-t * 15) >> 4);
			else if (t > 0)
				t = (t * 15) >> 4;
			scratch[base + i] = (int16_t)t;
		}

		// if we hit the end and IRQs are enabled, signal it
		if (base != 0) {
			voice->playing = 0;
			voice->irq_schedule = 1;   // latched by irq_timer() after this update
		}
	}

	// advance forward one sample
	prev = curr;
	curr = *curr_data++;

	// then sample-rate convert with linear interpolation
	while (remaining > 0) {
		while (remaining > 0 && pos < FRAC_ONE) {
			int s = (prev * (FRAC_ONE - pos) + curr * pos) >> FRAC_BITS;
			out(left++, right++, s, lvol, rvol);
			pos += ostep;
			remaining--;
		}
		// if we're over, grab the next samples
		if (pos >= FRAC_ONE) {
			pos -= FRAC_ONE;
			prev = curr;
			curr = *curr_data++;
		}
	}

	voice->output_pos = pos;
	voice->last_sample = (int16_t)prev;
	voice->curr_sample = (int16_t)curr;
}

int ymz280b_render(ymz280b_t *c, int32_t *left, int32_t *right, int n)
{
	int16_t scratch[CHUNK + 2];
	int done = 0;

	while (done < n) {
		int len = n - done;
		if (len > CHUNK)
			len = CHUNK;
		for (int v = 0; v < 8; v++)
			update_voice(c, v, left + done, right + done, len, scratch);
		done += len;
	}

	// MAME: the zero-time IRQ timers fire after the stream update
	for (int v = 0; v < 8; v++)
		if (c->v[v].irq_schedule)
			irq_timer(c, v);
	return n;
}

/* ---- registers ---- */

static void write_to_register(ymz280b_t *c, int data)
{
	if (c->current_register < 0x80) {
		ymz280b_voice_t *voice = &c->v[(c->current_register >> 2) & 7];

		switch (c->current_register & 0xe3) {
		case 0x00:      // pitch low 8 bits
			voice->fnum = (uint16_t)((voice->fnum & 0x100) | (data & 0xff));
			update_step(voice);
			break;

		case 0x01:      // pitch upper 1 bit, loop, key on, mode
			voice->fnum = (uint16_t)((voice->fnum & 0xff) | ((data & 0x01) << 8));
			voice->looping = (uint8_t)((data & 0x10) >> 4);
			if ((data & 0x60) == 0)
				data &= 0x7f;   // ignore mode setting and set to same state as KON=0
			else
				voice->mode = (uint8_t)((data & 0x60) >> 5);
			if (!voice->keyon && (data & 0x80) && c->keyon_enable) {
				voice->playing = 1;
				voice->position = voice->start;
				voice->signal = voice->loop_signal = 0;
				voice->step = voice->loop_step = 0x7f;
				voice->loop_count = 0;
				voice->irq_schedule = 0;   // cancel a pending end-of-sample IRQ
			} else if (voice->keyon && !(data & 0x80)) {
				voice->playing = 0;
				voice->irq_schedule = 0;
			}
			voice->keyon = (uint8_t)((data & 0x80) >> 7);
			update_step(voice);
			break;

		case 0x02:      // total level
			voice->level = (uint8_t)data;
			update_volumes(voice);
			break;

		case 0x03:      // pan
			voice->pan = (uint8_t)(data & 0x0f);
			update_volumes(voice);
			break;

		case 0x20: voice->start      = (voice->start      & (0x00ffffu << 1)) | ((uint32_t)data << 17); break;
		case 0x21: voice->loop_start = (voice->loop_start & (0x00ffffu << 1)) | ((uint32_t)data << 17); break;
		case 0x22: voice->loop_end   = (voice->loop_end   & (0x00ffffu << 1)) | ((uint32_t)data << 17); break;
		case 0x23: voice->stop       = (voice->stop       & (0x00ffffu << 1)) | ((uint32_t)data << 17); break;

		case 0x40: voice->start      = (voice->start      & (0xff00ffu << 1)) | ((uint32_t)data << 9); break;
		case 0x41: voice->loop_start = (voice->loop_start & (0xff00ffu << 1)) | ((uint32_t)data << 9); break;
		case 0x42: voice->loop_end   = (voice->loop_end   & (0xff00ffu << 1)) | ((uint32_t)data << 9); break;
		case 0x43: voice->stop       = (voice->stop       & (0xff00ffu << 1)) | ((uint32_t)data << 9); break;

		case 0x60: voice->start      = (voice->start      & (0xffff00u << 1)) | ((uint32_t)data << 1); break;
		case 0x61: voice->loop_start = (voice->loop_start & (0xffff00u << 1)) | ((uint32_t)data << 1); break;
		case 0x62: voice->loop_end   = (voice->loop_end   & (0xffff00u << 1)) | ((uint32_t)data << 1); break;
		case 0x63: voice->stop       = (voice->stop       & (0xffff00u << 1)) | ((uint32_t)data << 1); break;

		default:
			break;
		}
	} else {
		switch (c->current_register) {
		case 0x80:      // DSP related (not implemented in MAME)
		case 0x81:
		case 0x82:
			break;

		case 0x84:      // ROM readback / RAM write (high)
			c->ext_mem_address_hi = (uint32_t)data << 16;
			break;

		case 0x85:      // ROM readback / RAM write (middle)
			c->ext_mem_address_mid = (uint32_t)data << 8;
			break;

		case 0x86:      // ROM readback / RAM write (low) -> update latch
			c->ext_mem_address = c->ext_mem_address_hi | c->ext_mem_address_mid | (uint32_t)data;
			if (c->ext_mem_enable)
				c->ext_readlatch = read_byte(c, c->ext_mem_address);
			break;

		case 0x87:      // RAM write (the sample memory is ROM here: only the address moves)
			if (c->ext_mem_enable)
				c->ext_mem_address = (c->ext_mem_address + 1) & 0xffffff;
			break;

		case 0xfe:      // IRQ mask
			c->irq_mask = (uint8_t)data;
			update_irq_state(c);
			break;

		case 0xff:      // IRQ enable, test, etc
			c->ext_mem_enable = (uint8_t)((data & 0x40) >> 6);
			c->irq_enable = (uint8_t)((data & 0x10) >> 4);
			update_irq_state(c);

			if (c->keyon_enable && !(data & 0x80)) {
				for (int i = 0; i < 8; i++) {
					c->v[i].playing = 0;
					c->v[i].irq_schedule = 0;
				}
			} else if (!c->keyon_enable && (data & 0x80)) {
				for (int i = 0; i < 8; i++)
					if (c->v[i].keyon && c->v[i].looping)
						c->v[i].playing = 1;
			}
			c->keyon_enable = (uint8_t)((data & 0x80) >> 7);
			break;

		default:
			break;
		}
	}
}

void ymz280b_write(ymz280b_t *c, int offset, uint8_t v)
{
	if ((offset & 1) == 0)
		c->current_register = v;
	else
		write_to_register(c, v);
}

uint8_t ymz280b_read(ymz280b_t *c, int offset)
{
	if ((offset & 1) == 0) {
		if (!c->ext_mem_enable)
			return 0xff;
		// read from external memory
		uint8_t ret = c->ext_readlatch;
		c->ext_readlatch = read_byte(c, c->ext_mem_address);
		c->ext_mem_address = (c->ext_mem_address + 1) & 0xffffff;
		return ret;
	} else {
		// compute_status: read, then clear the IRQ state
		uint8_t result = c->status_register;
		c->status_register = 0;
		update_irq_state(c);
		return result;
	}
}

uint32_t ymz280b_rate(const ymz280b_t *c)
{
	return c->clock / 192;   // MAME INTERNAL_SAMPLE_RATE = (clock / 384) * 2
}

void ymz280b_reset(ymz280b_t *c)
{
	// initial clear registers
	for (int i = 0xff; i >= 0; i--) {
		c->current_register = (uint8_t)i;
		write_to_register(c, 0);
	}

	c->current_register = 0;
	c->status_register = 0;
	c->ext_mem_address = 0;

	for (int i = 0; i < 8; i++) {
		ymz280b_voice_t *voice = &c->v[i];
		voice->curr_sample = 0;
		voice->last_sample = 0;
		voice->output_pos = FRAC_ONE;
		voice->playing = 0;
	}
}

void ymz280b_init(ymz280b_t *c, uint32_t clock, const uint8_t *rom, uint32_t rom_size,
	void (*irq)(void *user, int state), void *user)
{
	memset(c, 0, sizeof(*c));
	c->clock = clock;
	c->rom = rom;
	c->rom_size = rom ? rom_size : 0;
	// device_rom_interface: a region smaller than the 24-bit space mirrors on its power-of-two size
	uint32_t m = c->rom_size ? c->rom_size - 1 : 0;
	m |= m >> 1; m |= m >> 2; m |= m >> 4; m |= m >> 8; m |= m >> 16;
	c->rom_mask = m;
	c->irq = irq;
	c->user = user;
	ymz280b_reset(c);
}
