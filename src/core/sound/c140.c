/*  Namco C140 / C219 PCM sound chips. Ported from MAME 0.289 src/devices/sound/c140.cpp (BSD-3-Clause,
    copyright-holders: R. Belmont) and src/mame/namco/namcos2_m.cpp c140_rom_r (BSD-3-Clause,
    copyright-holders: K.Wilkins, R. Belmont). See c140.h for the approximations.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "c140.h"
#include <stdlib.h>
#include <string.h>

#define MAX_VOICE 24
#define CHUNK 512

void c140_init(c140_t *t, uint32_t clock, const uint8_t *rom, uint32_t rom_size, int gain256,
               c140_irq_cb irq, void *irq_user)
{
	memset(t, 0, sizeof(*t));
	t->clock = clock;
	t->rom = rom;
	t->rom_size = rom_size;
	t->gain = gain256;
	t->irq = irq;
	t->irq_user = irq_user;
	// decompress pcm table (device_start)
	for (int i = 0; i < 256; i++) {
		int j = (int8_t)i;
		int s1 = j & 7;
		int s2 = abs(j >> 3) & 31;
		int v = (0x80 << s1) & 0xff00;
		v += s2 << (s1 ? s1 + 3 : 4);
		if (j < 0)
			v = -v;
		t->pcmtbl[i] = (int16_t)v;
	}
	c140_reset(t);
}

void c219_init(c140_t *t, uint32_t clock, const uint8_t *mem, uint32_t mem_size, int gain256)
{
	c140_init(t, clock, NULL, 0, gain256, NULL, NULL);
	t->c219 = 1;
	t->mem = mem;
	t->mem_size = mem_size;
	// c219_device::device_start: the mu-law table (same as c352.cpp)
	int j = 0;
	for (int i = 0; i < 128; i++) {
		t->pcmtbl[i] = (int16_t)(j << 5);
		if (i < 16)
			j += 1;
		else if (i < 24)
			j += 2;
		else if (i < 48)
			j += 4;
		else if (i < 100)
			j += 8;
		else
			j += 16;
	}
	for (int i = 0; i < 128; i++)
		t->pcmtbl[i + 128] = (int16_t)((~t->pcmtbl[i]) & 0xffe0);
	t->lfsr = 0x1234;
}

uint8_t c219_r(c140_t *t, int offset)
{
	offset &= 0x1ff;
	uint8_t data = t->reg[offset];
	if ((offset & 0xf) == 0x5 && offset < 0x100)
		data = (uint8_t)((t->v[offset >> 4].key ? 0x40 : 0x00) | (t->reg[offset] & 0x3f)); // keyon_status_read
	return data;
}

void c219_w(c140_t *t, int offset, uint8_t data)
{
	offset &= 0x1ff;
	// the bank registers mirrored (bkrtmaq, xday2)
	if (offset >= 0x1f8 && (offset & 1))
		offset -= 8;
	t->reg[offset] = data;
	if (offset < 0x100 && (offset & 0xf) == 0x5) {   // (16 voices)
		c140_voice_t *v = &t->v[offset >> 4];
		if (data & 0x80) {
			const uint8_t *r = &t->reg[offset & 0x1f0];
			v->key = 1;
			v->ptoffset = 0;
			v->pos = 0;
			v->lastdt = v->prevdt = v->dltdt = 0;
			v->bank = r[4];
			v->mode = data;
			// on the 219, addresses are in words
			v->sample_loop = ((r[10] << 8) | r[11]) << 1;
			v->sample_start = ((r[6] << 8) | r[7]) << 1;
			v->sample_end = ((r[8] << 8) | r[9]) << 1;
		} else {
			v->key = 0;
		}
	}
}

void c140_reset(c140_t *t)
{
	memset(t->reg, 0, sizeof(t->reg));
	memset(t->v, 0, sizeof(t->v));
	t->timer_on = 0;
	t->timer_left = 0;
	t->frac = 0;
	t->cur_l = t->cur_r = 0;
	if (t->c219)
		t->lfsr = 0x1234;   // (device_start's value: MAME keeps it through a reset)
}

static uint8_t rom_byte(const c140_t *t, uint32_t a)
{
	return a < t->rom_size ? t->rom[a] : 0;
}

uint16_t c140_rom_r(const c140_t *t, uint32_t offset)
{
	if (!t->rom)
		return 0;
	const uint32_t romsel = (offset >> 21) & 1;
	const int lsb_en = !((offset >> 20) & 1);
	const int lsb_swap = !((offset >> 19) & 1);
	offset &= 0x7ffff;
	uint16_t ret = (uint16_t)(rom_byte(t, (romsel << 19 | offset) * 2) << 8); // voice1 or voice2
	if (lsb_en) {
		uint8_t lsb = rom_byte(t, offset * 2 + 1); // voice0
		if (lsb_swap)
			lsb = (uint8_t)(lsb << 4); // D0-D3
		ret |= lsb & 0xf0;
	}
	return ret;
}

uint8_t c140_r(c140_t *t, int offset)
{
	offset &= 0x1ff;
	uint8_t data = t->reg[offset];
	if ((offset & 0xf) == 0x5 && offset < 0x180)
		data = (uint8_t)((t->v[offset >> 4].key ? 0x40 : 0x00) | (t->reg[offset] & 0x3f)); // keyon_status_read
	else if (offset == 0x1f8)
		data++; // timer reload value = written reg data + 1
	return data;
}

static void set_irq(c140_t *t, int state)
{
	if (t->irq)
		t->irq(t->irq_user, state);
}

void c140_w(c140_t *t, int offset, uint8_t data)
{
	offset &= 0x1ff;
	t->reg[offset] = data;
	if (offset < 0x180) {
		c140_voice_t *v = &t->v[offset >> 4];
		if ((offset & 0xf) == 0x5) {
			if ((data & 0x80) || ((data & 0x40) && v->key)) {
				const uint8_t *r = &t->reg[offset & 0x1f0];
				v->key = 1;
				v->ptoffset = 0;
				v->pos = 0;
				v->lastdt = v->prevdt = v->dltdt = 0;
				v->bank = r[4];
				v->mode = data;
				v->sample_loop = (r[10] << 8) | r[11];
				v->sample_start = (r[6] << 8) | r[7];
				v->sample_end = (r[8] << 8) | r[9];
			} else {
				v->key = 0;
			}
		}
	} else switch (offset) {
	case 0x1fa: // set INT1 timer
		set_irq(t, 0);
		if (t->reg[0x1fe] & 1) {
			t->timer_on = 1;
			t->timer_left = ((uint64_t)(t->reg[0x1f8] + 1) * 2) << 16;
		}
		break;
	case 0x1fe: // enable INT1 timer
		if (data & 1) {
			// kyukaidk and marvlandj want the first interrupt to happen immediately (MAME: expire().is_never())
			if (!t->timer_on)
				set_irq(t, 1);
		} else {
			set_irq(t, 0);
			t->timer_on = 0;
		}
		break;
	default:
		break;
	}
}

void c140_timer_advance(c140_t *t, uint64_t ticks65536)
{
	if (!t->timer_on)
		return;
	if (ticks65536 >= t->timer_left) {
		t->timer_left = 0;
		t->timer_on = 0; // one-shot: expire() becomes never after firing
		set_irq(t, 1);
	} else {
		t->timer_left -= ticks65536;
	}
}

// sound_stream_update for <= CHUNK chip samples into MAME's s16 mixer buffers (which wrap)
static void gen(c140_t *t, int16_t *lmixb, int16_t *rmixb, int samples)
{
	memset(lmixb, 0, samples * sizeof(int16_t));
	memset(rmixb, 0, samples * sizeof(int16_t));
	for (int i = 0; i < MAX_VOICE; i++) {
		c140_voice_t *v = &t->v[i];
		const uint8_t *r = &t->reg[i * 16];
		if (!v->key)
			continue;
		const int frequency = (r[2] << 8) | r[3];
		if (frequency == 0)
			continue;
		const int delta = frequency * 2;
		const int lvol = (r[1] * 32) / MAX_VOICE; // 32ch -> 24ch
		const int rvol = (r[0] * 32) / MAX_VOICE;
		int16_t *lmix = lmixb, *rmix = rmixb;
		const int st = v->sample_start;
		const int sz = v->sample_end - st;
		const int sampleData = (v->bank << 16) + st; // find_sample
		int offset = v->ptoffset, pos = v->pos;
		int32_t lastdt = v->lastdt, prevdt = v->prevdt, dltdt = v->dltdt;
		const int looped = (v->mode >> 4) & 1, mulaw = (v->mode >> 3) & 1;
		for (int j = 0; j < samples; j++) {
			offset += delta;
			const int cnt = (offset >> 16) & 0x7fff;
			offset &= 0xffff;
			pos += cnt;
			if (pos >= sz) {
				if (looped) {
					pos = v->sample_loop - st;
				} else {
					v->key = 0;
					break;
				}
			}
			if (cnt) {
				// read_word((sampleData + pos) << 1): the 0x800000-byte address map -> word offset
				const uint16_t sample = c140_rom_r(t, (uint32_t)(sampleData + pos) & 0x3fffff) & 0xfff0;
				prevdt = lastdt;
				lastdt = (mulaw ? t->pcmtbl[(sample >> 8) & 0xff] : (int16_t)sample) >> 4;
				dltdt = lastdt - prevdt;
			}
			const int32_t dt = ((dltdt * offset) >> 16) + prevdt;
			*lmix = (int16_t)(*lmix + ((dt * lvol) >> (5 + 4)));
			*rmix = (int16_t)(*rmix + ((dt * rvol) >> (5 + 4)));
			lmix++;
			rmix++;
		}
		v->ptoffset = offset;
		v->pos = pos;
		v->lastdt = lastdt;
		v->prevdt = prevdt;
		v->dltdt = dltdt;
	}
}

// c219_device::sound_stream_update
static void gen219(c140_t *t, int16_t *lmixb, int16_t *rmixb, int samples)
{
	static const int16_t banks[4] = { 0x1f7, 0x1f1, 0x1f3, 0x1f5 };
	memset(lmixb, 0, samples * sizeof(int16_t));
	memset(rmixb, 0, samples * sizeof(int16_t));
	for (int i = 0; i < 16; i++) {
		c140_voice_t *v = &t->v[i];
		const uint8_t *r = &t->reg[i * 16];
		if (!v->key)
			continue;
		const int frequency = (r[2] << 8) | r[3];
		if (frequency == 0)
			continue;
		const int delta = frequency * 2;
		const int lvol = (r[1] * 32) / MAX_VOICE;
		const int rvol = (r[0] * 32) / MAX_VOICE;
		int16_t *lmix = lmixb, *rmix = rmixb;
		const int st = v->sample_start;
		const int sz = v->sample_end - st;
		const int sampleData = (t->reg[banks[i / 4]] & 0x3) * 0x20000 + (v->bank << 16) + st; // find_sample
		int offset = v->ptoffset, pos = v->pos;
		int32_t lastdt = v->lastdt, prevdt = v->prevdt, dltdt = v->dltdt;
		const int looped = (v->mode >> 4) & 1, noise = (v->mode >> 2) & 1, mulaw = v->mode & 1;
		const int inv_lout = (v->mode >> 3) & 1, inv_sign = (v->mode >> 6) & 1;
		const int shift = noise ? 8 : 3;
		for (int j = 0; j < samples; j++) {
			offset += delta;
			const int cnt = (offset >> 16) & 0x7fff;
			offset &= 0xffff;
			pos += cnt;
			if (pos >= sz) {
				if (looped || noise) {
					pos = v->sample_loop - st;
				} else {
					v->key = 0;
					break;
				}
			}
			if (cnt) {
				prevdt = lastdt;
				if (noise) {
					t->lfsr = (uint16_t)((t->lfsr >> 1) ^ ((-(t->lfsr & 1)) & 0xfff6));
					lastdt = (int16_t)t->lfsr;
				} else {
					const uint32_t a = (uint32_t)(sampleData + pos) & 0x1ffffff;
					lastdt = (int8_t)(a < t->mem_size ? t->mem[a ^ 1] : 0);
					if (mulaw)   // 11 bit mu-law
						lastdt = t->pcmtbl[lastdt & 0xff] >> 5;
					else
						lastdt *= 8;   // scale as 11 bit
				}
				if (inv_sign)
					lastdt = -lastdt;
				dltdt = lastdt - prevdt;
			}
			const int32_t dt = ((dltdt * offset) >> 16) + prevdt;
			*lmix = (int16_t)(*lmix + ((inv_lout ? -(dt * lvol) : (dt * lvol)) >> (5 + shift)));
			*rmix = (int16_t)(*rmix + ((dt * rvol) >> (5 + shift)));
			lmix++;
			rmix++;
		}
		v->ptoffset = offset;
		v->pos = pos;
		v->lastdt = lastdt;
		v->prevdt = prevdt;
		v->dltdt = dltdt;
	}
}

// put_int_clamp(m, 32768 / 8) at 16-bit scale, gain applied
static int32_t outv(const c140_t *t, int32_t m)
{
	if (m > 4096) m = 4096;
	if (m < -4096) m = -4096;
	int32_t s = m * 8;
	if (s > 32767) s = 32767;
	return (s * t->gain) >> 8;
}

void c140_render(c140_t *t, int32_t *left, int32_t *right, int n, uint32_t out_rate)
{
	static int16_t lm[CHUNK], rm[CHUNK];
	if (out_rate == t->clock) {
		while (n > 0) {
			int k = n < CHUNK ? n : CHUNK;
			if (t->c219)
				gen219(t, lm, rm, k);
			else
				gen(t, lm, rm, k);
			for (int i = 0; i < k; i++) {
				*left++ += outv(t, lm[i]);
				*right++ += outv(t, rm[i]);
			}
			n -= k;
		}
		return;
	}
	// box filter; time in units of 1 / (clock * out_rate)
	while (n > 0) {
		int k = n < 64 ? n : 64;
		uint64_t end = t->frac + (uint64_t)k * t->clock;
		int need = (int)(end / out_rate);
		if (need > CHUNK) {
			k = (int)((uint64_t)(CHUNK - 1) * out_rate / t->clock);
			if (k < 1) k = 1;
			end = t->frac + (uint64_t)k * t->clock;
			need = (int)(end / out_rate);
		}
		if (t->c219)
			gen219(t, lm, rm, need);
		else
			gen(t, lm, rm, need);
		uint64_t acc = t->frac;
		int idx = 0;
		for (int i = 0; i < k; i++) {
			acc += t->clock;
			int c = 0;
			int32_t sl = 0, sr = 0;
			while (idx < need && (uint64_t)(idx + 1) * out_rate <= acc) {
				sl += outv(t, lm[idx]);
				sr += outv(t, rm[idx]);
				idx++;
				c++;
			}
			if (c) {
				t->cur_l = sl / c;
				t->cur_r = sr / c;
			}
			*left++ += t->cur_l;
			*right++ += t->cur_r;
		}
		t->frac = end - (uint64_t)need * out_rate;
		n -= k;
	}
}
