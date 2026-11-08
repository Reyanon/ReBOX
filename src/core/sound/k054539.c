/*  Konami 054539 PCM sound chip, ported to C from MAME src/devices/sound/k054539.cpp (BSD-3-Clause,
    copyright-holders: Olivier Galibert). See k054539.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k054539.h"
#include "../rbx.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Registers:
   00..ff: 20 bytes/channel, 8 channels
     00..02: pitch (lsb, mid, msb)
         03: volume (0=max, 0x40=-36dB)
         04: reverb volume (idem)
     05: pan (1-f right, 10 middle, 11-1f left)
     06..07: reverb delay (0=max, current computation non-trusted)
     08..0a: loop (lsb, mid, msb)
     0c..0e: start (lsb, mid, msb) (and current position ?)
   100.1ff: effects?
     13f: pan of the analog input (1-1f)
   200..20f: 2 bytes/channel, 8 channels
     00: type (b2-3), reverse (b5)
     01: loop (b0)
   214: Key on (b0-7 = channel 0-7)
   215: Key off
   227: Timer frequency
   22c: Channel active? (b0-7 = channel 0-7)
   22d: Data read/write port
   22e: ROM/RAM select (00..7f == ROM banks, 80 = Reverb RAM)
   22f: Global control:
        .......x - Enable PCM
        ...x.... - Enable ROM/RAM readback from 0x22d
        ..x..... - Timer output enable?
        x....... - Disable register RAM updates
*/

static inline uint8_t read_byte(const k054539_t *k, uint32_t a)
{
	a &= 0xffffff;
	return a < k->rom_size ? k->rom[a] : 0;
}

static inline int regupdate(const k054539_t *k) { return !(k->regs[0x22f] & 0x80); }
static inline void keyon(k054539_t *k, int ch) { if (regupdate(k)) k->regs[0x22c] |= (uint8_t)(1 << ch); }
static inline void keyoff(k054539_t *k, int ch) { if (regupdate(k)) k->regs[0x22c] &= (uint8_t)~(1 << ch); }

static void init_chip(k054539_t *k)
{
	memset(k->regs, 0, sizeof(k->regs));
	memset(k->posreg_latch, 0, sizeof(k->posreg_latch));
	k->flags |= K054539_UPDATE_AT_KEYON;   // (make it default until proven otherwise)
	k->reverb_pos = 0;
	k->cur_ptr = 0;
	k->rom_addr = 0;
	k->timer_state = 0;
	memset(k->ram, 0, sizeof(k->ram));
	memset(k->ch, 0, sizeof(k->ch));
}

void k054539_init(k054539_t *k, const uint8_t *rom, uint32_t rom_size)
{
	memset(k, 0, sizeof(*k));
	k->rom = rom;
	k->rom_size = rom ? rom_size : 0;
	for (int i = 0; i < 8; i++)
		k->gain[i] = 1.0;
	// vol=0 -> no attenuation, vol=0x40 -> -36dB; the 1/4 for the number of channels
	for (int i = 0; i < 256; i++)
		k->voltab[i] = pow(10.0, (-36.0 * (double)i / (double)0x40) / 20.0) / 4.0;
	// the left channel's pan (the right: the index inverted): pan[i]**2+pan[0xe-i]**2 = 1, pan[0xe] = 1
	for (int i = 0; i < 0xf; i++)
		k->pantab[i] = sqrt((double)i) / sqrt((double)0xe);
	init_chip(k);
}

void k054539_set_gain(k054539_t *k, int channel, double gain)
{
	if (gain >= 0)
		k->gain[channel] = gain;
}

void k054539_reset(k054539_t *k)
{
	k->regs[0x22c] = 0;
	k->regs[0x22f] = 0;
	memset(k->ram, 0, sizeof(k->ram));
	k->timer_div = 0;   // (timer adjust never)
	k->timer_acc = 0;
}

static void timer_out(k054539_t *k, int state)
{
	if (k->timer_cb)
		k->timer_cb(k->user, state);
}

void k054539_write(k054539_t *k, uint32_t offset, uint8_t data)
{
	if (offset >= 0x230)
		return;
	const int latch = (k->flags & K054539_UPDATE_AT_KEYON) && (k->regs[0x22f] & 1);
	if (latch && offset < 0x100) {
		const int offs = (int)(offset & 0x1f) - 0xc, ch = (int)(offset >> 5);
		if (offs >= 0 && offs <= 2) {   // (latch writes to the position index registers)
			k->posreg_latch[ch][offs] = data;
			return;
		}
	} else {
		switch (offset) {
		case 0x13f:   // (the analog input's pan: no analog input here)
			break;
		case 0x214:
			for (int ch = 0; ch < 8; ch++)
				if (data & (1 << ch)) {
					if (latch) {   // (update the chip at key-on)
						uint8_t *reg = k->regs + (ch << 5) + 0xc;
						reg[0] = k->posreg_latch[ch][0];
						reg[1] = k->posreg_latch[ch][1];
						reg[2] = k->posreg_latch[ch][2];
					}
					keyon(k, ch);
				}
			break;
		case 0x215:
			for (int ch = 0; ch < 8; ch++)
				if (data & (1 << ch))
					keyoff(k, ch);
			break;
		case 0x227:   // period: 1 / ((38 + data) * clock / 384 / 14400) / 2 = 7200 / (38 + data) samples
			k->timer_div = 38u + data;
			k->timer_acc = 0;
			k->timer_state = 0;
			timer_out(k, 0);
			break;
		case 0x22d:
			if (k->rom_addr == 0x80) {
				const uint32_t addr = (uint32_t)(k->cur_ptr & 0x3fff) | ((uint32_t)(k->cur_ptr & 0x10000) >> 2);
				k->ram[addr] = data;
			}
			k->cur_ptr = (k->cur_ptr + 1) & 0x1ffff;
			break;
		case 0x22e:
			k->rom_addr = data;
			k->cur_ptr = 0;
			break;
		case 0x22f:
			if (!(data & 0x20)) {   // (disable timer output?)
				k->timer_state = 0;
				timer_out(k, 0);
			}
			break;
		default:
			break;
		}
	}
	k->regs[offset] = data;
}

uint8_t k054539_read(k054539_t *k, uint32_t offset)
{
	if (offset >= 0x230)
		return 0;
	if (offset == 0x22d) {
		if (k->regs[0x22f] & 0x10) {
			const uint32_t addr = (uint32_t)(k->cur_ptr & 0x3fff) | ((uint32_t)(k->cur_ptr & 0x10000) >> 2);
			const uint8_t res = k->rom_addr == 0x80 ? k->ram[addr] : read_byte(k, 0x20000 * k->rom_addr + (uint32_t)k->cur_ptr);
			k->cur_ptr = (k->cur_ptr + 1) & 0x1ffff;
			return res;
		}
		return 0;
	}
	return k->regs[offset];
}

int k054539_timer_due(const k054539_t *k)
{
	if (!k->timer_div)
		return -1;
	return (int)((7200 - k->timer_acc + k->timer_div - 1) / k->timer_div);
}

static const int16_t dpcm[16] = {
	0 * 0x100,   1 * 0x100,   2 * 0x100,   4 * 0x100,  8 * 0x100, 16 * 0x100, 32 * 0x100, 64 * 0x100,
	0 * 0x100, -64 * 0x100, -32 * 0x100, -16 * 0x100, -8 * 0x100, -4 * 0x100, -2 * 0x100, -1 * 0x100
};

void k054539_render(k054539_t *k, int32_t *outl, int32_t *outr, int n)
{
	static const double VOL_CAP = 1.80;
	int16_t *rbase = (int16_t *)k->ram;
	for (int sample = 0; sample < n; sample++) {
		if (k->regs[0x22f] & 1) {
			double lval, rval;
			if (!(k->flags & K054539_DISABLE_REVERB))
				lval = rval = rbase[k->reverb_pos];
			else
				lval = rval = 0;
			rbase[k->reverb_pos] = 0;
			for (int ch = 0; ch < 8; ch++) {
				if (!((k->regs[0x22c] >> ch) & 1))
					continue;
				uint8_t *base1 = k->regs + 0x20 * ch, *base2 = k->regs + 0x200 + 0x2 * ch;
				int delta = base1[0x00] | (base1[0x01] << 8) | (base1[0x02] << 16);
				const int vol = base1[0x03];
				int bval = vol + base1[0x04];
				if (bval > 255)
					bval = 255;
				int pan = base1[0x05];
				if (pan >= 0x81 && pan <= 0x8f)   // (DJ Main: 81-87 right, 88 middle, 89-8f left)
					pan -= 0x81;
				else if (pan >= 0x11 && pan <= 0x1f)
					pan -= 0x11;
				else
					pan = 0x18 - 0x11;
				const double cur_gain = k->gain[ch];
				double lvol = k->voltab[vol] * k->pantab[pan] * cur_gain;
				if (lvol > VOL_CAP)
					lvol = VOL_CAP;
				double rvol = k->voltab[vol] * k->pantab[0xe - pan] * cur_gain;
				if (rvol > VOL_CAP)
					rvol = VOL_CAP;
				double rbvol = k->voltab[bval] * cur_gain / 2;
				if (rbvol > VOL_CAP)
					rbvol = VOL_CAP;
				int rdelta = (base1[6] | (base1[7] << 8)) >> 3;
				rdelta = (rdelta + k->reverb_pos) & 0x3fff;
				int cur_pos = base1[0x0c] | (base1[0x0d] << 8) | (base1[0x0e] << 16);
				int fdelta, pdelta;
				if (base2[0] & 0x20) {
					delta = -delta;
					fdelta = +0x10000;
					pdelta = -1;
				} else {
					fdelta = -0x10000;
					pdelta = +1;
				}
				int cur_pfrac, cur_val, cur_pval;
				if ((uint32_t)cur_pos != k->ch[ch].pos) {
					k->ch[ch].pos = (uint32_t)cur_pos;
					cur_pfrac = 0;
					cur_val = 0;
					cur_pval = 0;
				} else {
					cur_pfrac = (int)k->ch[ch].pfrac;
					cur_val = k->ch[ch].val;
					cur_pval = k->ch[ch].pval;
				}
				switch (base2[0] & 0xc) {
				case 0x0:   // 8bit pcm
					cur_pfrac += delta;
					while (cur_pfrac & ~0xffff) {
						cur_pfrac += fdelta;
						cur_pos += pdelta;
						cur_pval = cur_val;
						cur_val = (int16_t)(read_byte(k, (uint32_t)cur_pos) << 8);
						if (cur_val == (int16_t)0x8000 && (base2[1] & 1)) {
							cur_pos = base1[0x08] | (base1[0x09] << 8) | (base1[0x0a] << 16);
							cur_val = (int16_t)(read_byte(k, (uint32_t)cur_pos) << 8);
						}
						if (cur_val == (int16_t)0x8000) {
							keyoff(k, ch);
							cur_val = 0;
							break;
						}
					}
					break;
				case 0x4:   // 16bit pcm lsb first
					pdelta <<= 1;
					cur_pfrac += delta;
					while (cur_pfrac & ~0xffff) {
						cur_pfrac += fdelta;
						cur_pos += pdelta;
						cur_pval = cur_val;
						cur_val = (int16_t)(read_byte(k, (uint32_t)cur_pos) | read_byte(k, (uint32_t)cur_pos + 1) << 8);
						if (cur_val == (int16_t)0x8000 && (base2[1] & 1)) {
							cur_pos = base1[0x08] | (base1[0x09] << 8) | (base1[0x0a] << 16);
							cur_val = (int16_t)(read_byte(k, (uint32_t)cur_pos) | read_byte(k, (uint32_t)cur_pos + 1) << 8);
						}
						if (cur_val == (int16_t)0x8000) {
							keyoff(k, ch);
							cur_val = 0;
							break;
						}
					}
					break;
				case 0x8:   // 4bit dpcm
					cur_pos <<= 1;
					cur_pfrac <<= 1;
					if (cur_pfrac & 0x10000) {
						cur_pfrac &= 0xffff;
						cur_pos |= 1;
					}
					cur_pfrac += delta;
					while (cur_pfrac & ~0xffff) {
						cur_pfrac += fdelta;
						cur_pos += pdelta;
						cur_pval = cur_val;
						cur_val = read_byte(k, (uint32_t)cur_pos >> 1);
						if (cur_val == 0x88 && (base2[1] & 1)) {
							cur_pos = (base1[0x08] | (base1[0x09] << 8) | (base1[0x0a] << 16)) << 1;
							cur_val = read_byte(k, (uint32_t)cur_pos >> 1);
						}
						if (cur_val == 0x88) {
							keyoff(k, ch);
							cur_val = 0;
							break;
						}
						if (cur_pos & 1)
							cur_val >>= 4;
						else
							cur_val &= 15;
						cur_val = cur_pval + dpcm[cur_val];
						if (cur_val < -32768)
							cur_val = -32768;
						else if (cur_val > 32767)
							cur_val = 32767;
					}
					cur_pfrac >>= 1;
					if (cur_pos & 1)
						cur_pfrac |= 0x8000;
					cur_pos >>= 1;
					break;
				default:   // (unknown sample type)
					break;
				}
				lval += cur_val * lvol;
				rval += cur_val * rvol;
				rbase[(rdelta + k->reverb_pos) & 0x1fff] += (int16_t)(cur_val * rbvol);
				k->ch[ch].pos = (uint32_t)cur_pos;
				k->ch[ch].pfrac = (uint32_t)cur_pfrac;
				k->ch[ch].pval = cur_pval;
				k->ch[ch].val = cur_val;
				if (regupdate(k)) {
					base1[0x0c] = (uint8_t)(cur_pos & 0xff);
					base1[0x0d] = (uint8_t)(cur_pos >> 8 & 0xff);
					base1[0x0e] = (uint8_t)(cur_pos >> 16 & 0xff);
				}
			}
			k->reverb_pos = (k->reverb_pos + 1) & 0x1fff;
			// (MAME put_int(.., 32768): the sum as 16-bit full scale)
			int32_t li = (int32_t)lval, ri = (int32_t)rval;
			if (k->flags & K054539_REVERSE_STEREO) {
				const int32_t t = li;
				li = ri;
				ri = t;
			}
			outl[sample] += li;
			outr[sample] += ri;
		}
		if (k->timer_div) {   // (the timer runs whether or not the PCM is enabled)
			k->timer_acc += k->timer_div;
			if (k->timer_acc >= 7200) {
				k->timer_acc -= 7200;
				if (k->regs[0x22f] & 0x20) {
					k->timer_state ^= 1;
					timer_out(k, (int)k->timer_state);
				}
			}
		}
	}
}

int k054539_state(k054539_t *k, void *ctx, int load, const char *prefix)
{
	char t[48];
	int rc = 0;
#define IO(name, p, n) (snprintf(t, sizeof(t), "%s%s", prefix, name), rbx_state_io(ctx, t, p, n))
	rc |= IO("regs", k->regs, sizeof(k->regs));
	rc |= IO("latch", k->posreg_latch, sizeof(k->posreg_latch));
	rc |= IO("ch", k->ch, sizeof(k->ch));
	rc |= IO("ram", k->ram, sizeof(k->ram));
	int32_t v[7] = { k->flags, k->reverb_pos, k->cur_ptr, (int32_t)k->rom_addr, (int32_t)k->timer_state,
	                 (int32_t)k->timer_div, (int32_t)k->timer_acc };
	rc |= IO("v", v, sizeof(v));
#undef IO
	if (load && !rc) {
		k->flags = v[0];
		k->reverb_pos = v[1];
		k->cur_ptr = v[2];
		k->rom_addr = (uint32_t)v[3];
		k->timer_state = (uint32_t)v[4];
		k->timer_div = (uint32_t)v[5];
		k->timer_acc = (uint32_t)v[6];
	}
	return rc;
}
