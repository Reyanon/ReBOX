/*  Yamaha YMZ770C "AMMS-A" and YMZ774 "AMMS2C", ported from MAME 0.289 src/devices/sound/ymz770.cpp / ymz770.h
    (BSD-3-Clause, copyright-holders: Olivier Galibert, R. Belmont, MetalliC).
    Emulation by R. Belmont and MetalliC, AMM decode by Olivier Galibert.
    MAME TODO (not emulated there either): Simple Access mode, the configurable sample clock divider, 770
    sequencer triggers, 774 4-channel output, equalizer, pan delayed transition, sequencer off trigger.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "ymz774.h"
#include "../rbx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define YMZ_PAGE 4096u
#define YMZ_PAGES 64u
#define YMZ_WINDOW 4096u

// ReBOX: a page of the ROM read through fetch (rom NULL)
static const uint8_t *rom_page(ymz77x_t *c, uint32_t page)
{
	uint8_t *p = c->pages + (page % YMZ_PAGES) * YMZ_PAGE;
	if (c->page_tag[page % YMZ_PAGES] != page + 1) {
		const uint32_t off = page * YMZ_PAGE;
		const uint32_t len = c->rom_size - off < YMZ_PAGE ? c->rom_size - off : YMZ_PAGE;
		if (c->fetch(c->fetch_user, off, p, len))
			memset(p, 0xff, len);
		if (len < YMZ_PAGE)
			memset(p + len, 0xff, YMZ_PAGE - len);
		if (c->fetch_swap16)
			for (uint32_t i = 0; i < YMZ_PAGE; i += 2) {
				const uint8_t t = p[i];
				p[i] = p[i + 1];
				p[i + 1] = t;
			}
		c->page_tag[page % YMZ_PAGES] = page + 1;
		c->page_misses++;
	}
	return p;
}

static inline uint8_t get_rom_byte(const ymz77x_t *c, uint32_t offset)
{
	offset %= c->rom_size;
	if (c->rom)
		return c->rom[offset];
	return rom_page((ymz77x_t *)c, offset / YMZ_PAGE)[offset % YMZ_PAGE];
}

// MAME indexes m_rom directly here; the modulo only matters for offsets past the region
static uint32_t get_phrase_offs(const ymz77x_t *c, int phrase)
{
	uint32_t p = 4u * (uint32_t)phrase;
	if (c->type == YMZ_TYPE_774)
		return ((uint32_t)(get_rom_byte(c, p) & 0x0f) << 24 | (uint32_t)get_rom_byte(c, p + 1) << 16 |
			(uint32_t)get_rom_byte(c, p + 2) << 8 | get_rom_byte(c, p + 3)) * 2;
	return (uint32_t)get_rom_byte(c, p + 1) << 16 | (uint32_t)get_rom_byte(c, p + 2) << 8 | get_rom_byte(c, p + 3);
}

static uint32_t get_seq_offs(const ymz77x_t *c, int sqn)
{
	if (c->type == YMZ_TYPE_774) {
		uint32_t s = 4u * (uint32_t)sqn + 0x2000;
		return ((uint32_t)(get_rom_byte(c, s) & 0x0f) << 24 | (uint32_t)get_rom_byte(c, s + 1) << 16 |
			(uint32_t)get_rom_byte(c, s + 2) << 8 | get_rom_byte(c, s + 3)) * 2;
	}
	uint32_t s = 4u * (uint32_t)sqn + 0x400;
	return (uint32_t)get_rom_byte(c, s + 1) << 16 | (uint32_t)get_rom_byte(c, s + 2) << 8 | get_rom_byte(c, s + 3);
}

static uint32_t get_sqc_offs(const ymz77x_t *c, int sqc)
{
	uint32_t s = 4u * (uint32_t)sqc + 0x6000;
	return ((uint32_t)(get_rom_byte(c, s) & 0x0f) << 24 | (uint32_t)get_rom_byte(c, s + 1) << 16 |
		(uint32_t)get_rom_byte(c, s + 2) << 8 | get_rom_byte(c, s + 3)) * 2;
}

int ymz77x_init_fetch(ymz77x_t *c, int type, uint32_t clock, uint32_t rom_size,
                      int (*fetch)(void *user, uint32_t offset, void *dst, uint32_t len), void *user, int swap16)
{
	memset(c, 0, sizeof(*c));
	c->fetch = fetch;
	c->fetch_user = user;
	c->fetch_swap16 = swap16;
	c->pages = (uint8_t *)malloc(YMZ_PAGES * YMZ_PAGE);
	c->window = (uint8_t *)malloc(YMZ_WINDOW);
	if (!c->pages || !c->window) {
		free(c->pages);
		free(c->window);
		c->pages = c->window = NULL;
		return -1;
	}
	return ymz77x_init(c, type, clock, NULL, rom_size);
}

int ymz77x_init(ymz77x_t *c, int type, uint32_t clock, const uint8_t *rom, uint32_t rom_size)
{
	if (rom)
		memset(c, 0, sizeof(*c));
	else {   // (ymz77x_init_fetch: its fields kept)
		ymz77x_t keep = *c;
		memset(c, 0, sizeof(*c));
		c->fetch = keep.fetch;
		c->fetch_user = keep.fetch_user;
		c->fetch_swap16 = keep.fetch_swap16;
		c->pages = keep.pages;
		c->window = keep.window;
	}
	c->type = type;
	c->clock = clock;
	c->divider = type == YMZ_TYPE_774 ? 512 : 1024;
	c->rom = rom;
	c->rom_size = rom_size;

	// calculate volume increments, fixed point values, fractions of 0x20000
	for (uint32_t i = 0; i < 256; i++) {
		if (i < 0x20)
			c->volinc[i] = i;
		else
			c->volinc[i] = (0x20 | (i & 0x1f)) << ((i >> 5) - 1);
	}

	c->output_data = (int16_t *)calloc(16 * YMZ770_OUTPUT_SIZE, sizeof(int16_t));
	c->dec = (rbx_mpeg_audio_t *)calloc(16, sizeof(rbx_mpeg_audio_t));
	if (!c->output_data || !c->dec) {
		ymz77x_exit(c);
		return -1;
	}

	// device_start
	c->sclock = clock / c->divider;
	for (int i = 0; i < 16; i++) {
		c->ch[i].is_playing = 0;
		rbx_mpeg_audio_init(&c->dec[i], rom ? (const void *)rom : (const void *)c->window, RBX_MPEG_AMM, 0, 0);
	}
	for (int i = 0; i < 8; i++) {
		c->seq[i].is_playing = 0;
		c->sqc[i].is_playing = 0;
	}

	ymz77x_reset(c);
	return 0;
}

void ymz77x_exit(ymz77x_t *c)
{
	free(c->output_data);
	free(c->dec);
	free(c->pages);
	free(c->window);
	c->output_data = NULL;
	c->dec = NULL;
	c->pages = c->window = NULL;
}

void ymz77x_reset(ymz77x_t *c)
{
	for (int i = 0; i < 16; i++) {
		ymz_channel_t *ch = &c->ch[i];
		ch->latch.pan = 64;
		ch->latch.pan_delay = 0;
		ch->latch.pan1 = 64;
		ch->latch.pan1_delay = 0;
		ch->latch.volume_target = 0;
		ch->latch.volume_delay = 0;
		ch->latch.volume2 = 0;

		ch->current = ch->latch;

		ch->phrase = 0;
		ch->volume = 0;
		ch->loop = 0;

		ch->pending = 0;
		ch->is_playing = 0;
		ch->is_paused = 0;
		ch->last_block = 0;
		ch->output_remaining = 0;
		rbx_mpeg_audio_clear(&c->dec[i]);
	}
	for (int i = 0; i < 8; i++) {
		ymz_sequence_t *s = &c->seq[i];
		s->delay = 0;
		s->sequence = 0;
		s->timer = 0;
		s->stopchan = 0;
		s->loop = 0;
		s->offset = 0;
		s->bank = 0;
		s->is_playing = 0;
		s->is_paused = 0;
	}
	for (int i = 0; i < 8; i++) {
		ymz_sqc_t *q = &c->sqc[i];
		q->is_playing = 0;
		q->loop = 0;
		q->offset = 0;
		q->is_waiting = 0;
	}
}

//-------------------------------------------------
//  register writes
//-------------------------------------------------

static void internal_reg_write_770(ymz77x_t *c, uint8_t reg, uint8_t data)
{
	// global registers
	if (reg < 0x40) {
		switch (reg) {
		case 0x00:
			c->mute = data & 1;
			c->doen = data >> 1 & 1;
			break;

		case 0x01:
			c->vlma = data;
			break;

		case 0x02:
			c->bsl = data & 7;
			c->cpl = data >> 4 & 7;
			break;

		default: // unused
			break;
		}
	}

	// playback registers
	else if (reg < 0x60) {
		ymz_channel_t *ch = &c->ch[reg >> 2 & 0x07];

		switch (reg & 0x03) {
		case 0:
			ch->phrase = data;
			break;

		case 1:
			ch->volume = 128 << 17;
			ch->latch.volume2 = data;
			break;

		case 2:
			ch->latch.pan = (uint8_t)(data << 3);
			break;

		case 3:
			if ((data & 6) == 2 || ((data & 6) == 6 && !ch->is_playing)) { // both KON bits is 1 = "Keep Playing", do not restart channel in this case
				ch->pending = 1;
				ch->is_playing = 1;
				ch->last_block = 0;
			} else if ((data & 6) == 0)
				ch->is_playing = 0;

			ch->loop = (data & 1) ? 255 : 0;
			break;
		}
	}

	// sequencer registers
	else if (reg >= 0x80) {
		ymz_sequence_t *s = &c->seq[reg >> 4 & 0x07];

		switch (reg & 0x0f) {
		case 0: // SQSN
			s->sequence = data;
			break;
		case 1: // SQON SQLP
			if ((data & 6) == 2 || ((data & 6) == 6 && !s->is_playing)) { // both KON bits 1 is "Keep Playing"
				s->offset = get_seq_offs(c, s->sequence);
				s->delay = 0;
				s->is_playing = 1;
			} else if ((data & 6) == 0 && s->is_playing) {
				s->is_playing = 0;
				for (int i = 0; i < 8; i++)
					if (s->stopchan & (1 << i))
						c->ch[i].is_playing = 0;
			}

			s->loop = data & 1;
			break;
		case 2: // TMRH
			s->timer = (uint16_t)((s->timer & 0x00ff) | (data << 8));
			break;
		case 3: // TMRL
			s->timer = (uint16_t)((s->timer & 0xff00) | data);
			break;
		case 6: // SQOF
			s->stopchan = data;
			break;
		default: // TGST, TGEN, 7, 8: not implemented (MAME logs)
			break;
		}
	}
	// 0x60-0x7f: not implemented (MAME logs)
}

static void internal_reg_write_774(ymz77x_t *c, uint8_t reg, uint8_t data)
{
	// playback registers
	if (reg < 0x10) { // phrase num H and L
		ymz_channel_t *ch = &c->ch[((reg >> 1) & 7) + c->bank * 8];
		if (reg & 1)
			ch->phrase = (uint16_t)((ch->phrase & 0xff00) | data);
		else
			ch->phrase = (uint16_t)((ch->phrase & 0x00ff) | ((data & 7) << 8));
	} else if (reg < 0x60) {
		ymz_channel_t *ch = &c->ch[(reg & 7) + c->bank * 8];
		switch (reg & 0xf8) {
		case 0x10: // Volume 1
			ch->latch.volume_target = data;
			break;
		case 0x18: // Volume 1 delayed transition
			ch->latch.volume_delay = data;
			break;
		case 0x20: // Volume 2
			ch->latch.volume2 = data;
			break;
		case 0x28: // Pan L/R
			ch->latch.pan = data;
			break;
		case 0x30: // Pan L/R delayed transition (not implemented)
			ch->latch.pan_delay = data;
			break;
		case 0x38: // Pan T/B
			ch->latch.pan1 = data;
			break;
		case 0x40: // Pan T/B delayed transition (not implemented)
			ch->latch.pan1_delay = data;
			break;
		case 0x48: // Loop
			ch->loop = data;
			break;
		case 0x50: // Start / Stop
			if (data) {
				ch->pending = 1;
				ch->is_playing = 1;
				ch->is_paused = 0; // checkme, might be not needed
				ch->last_block = 0;
			} else
				ch->is_playing = 0;
			break;
		case 0x58: // Pause / Resume
			ch->is_paused = data ? 1 : 0;
			break;
		}
	} else if (reg < 0xd0) {
		if (c->bank == 0) {
			int sq = reg & 7;
			switch (reg & 0xf8) {
			case 0x60: // sequence num H and L
			case 0x68:
				sq = (reg >> 1) & 7;
				if (reg & 1)
					c->seq[sq].sequence = (uint16_t)((c->seq[sq].sequence & 0xff00) | data);
				else
					c->seq[sq].sequence = (uint16_t)((c->seq[sq].sequence & 0x00ff) | ((data & 0x07) << 8));
				break;
			case 0x70: // Start / Stop
				if (data) {
					c->seq[sq].offset = get_seq_offs(c, c->seq[sq].sequence);
					c->seq[sq].delay = 0;
					c->seq[sq].is_playing = 1;
					c->seq[sq].is_paused = 0; // checkme, might be not needed
				} else {
					if (c->seq[sq].is_playing)
						for (int ch = 0; ch < 16; ch++)
							if (c->seq[sq].stopchan & (1 << ch))
								c->ch[ch].is_playing = 0;
					c->seq[sq].is_playing = 0;
				}
				break;
			case 0x78: // Pause / Resume
				c->seq[sq].is_paused = data ? 1 : 0;
				break;
			case 0x80: // Loop count, 0 = off, 255 - infinite
				c->seq[sq].loop = data;
				break;
			case 0x88: // timer H and L
			case 0x90:
				sq = (reg - 0x88) >> 1;
				if (reg & 1)
					c->seq[sq].timer = (uint16_t)((c->seq[sq].timer & 0xff00) | data);
				else
					c->seq[sq].timer = (uint16_t)((c->seq[sq].timer & 0x00ff) | (data << 8));
				break;
			case 0x98: // Off trigger (not implemented)
				break;
			case 0xa0: // stop channel mask H and L (when sequence stopped)
			case 0xa8:
				sq = (reg >> 1) & 7;
				if (reg & 1)
					c->seq[sq].stopchan = (uint16_t)((c->seq[sq].stopchan & 0xff00) | data);
				else
					c->seq[sq].stopchan = (uint16_t)((c->seq[sq].stopchan & 0x00ff) | (data << 8));
				break;
			case 0xb0: // SQC number
				c->sqc[sq].sqc = data;
				break;
			case 0xb8: // SQC start / stop
				if (data) {
					c->sqc[sq].offset = get_sqc_offs(c, c->sqc[sq].sqc);
					c->sqc[sq].is_playing = 1;
					c->sqc[sq].is_waiting = 0;
				} else {
					c->sqc[sq].is_playing = 0;
					// stop SEQ too, and stop channels
					if (c->seq[sq].is_playing)
						for (int ch = 0; ch < 16; ch++)
							if (c->seq[sq].stopchan & (1 << ch))
								c->ch[ch].is_playing = 0;
					c->seq[sq].is_playing = 0;
				}
				break;
			case 0xc0: // SQC loop (255 = infinite)
				c->sqc[sq].loop = data;
				break;
			default:
				break;
			}
		}
		// else bank1 - Equalizer control
	}
	// global registers
	else {
		switch (reg) {
		case 0xd0: // Total Volume L0/R0
			c->vlma = data;
			break;
		case 0xd1: // Total Volume L1/R1
			c->vlma1 = data;
			break;
		case 0xd2: // Clip limit
			c->cpl = data;
			break;
		case 0xf0:
			c->bank = data & 1;
			break;
		default: // 0xd3-0xde: digital / PWM output, digital audio IF, GPIO (not implemented)
			break;
		}
	}
}

static inline void internal_reg_write(ymz77x_t *c, uint8_t reg, uint8_t data)
{
	if (c->type == YMZ_TYPE_774)
		internal_reg_write_774(c, reg, data);
	else
		internal_reg_write_770(c, reg, data);
}

void ymz77x_write(ymz77x_t *c, uint32_t offset, uint8_t data)
{
	if (offset & 1)
		internal_reg_write(c, c->cur_reg, data);
	else
		c->cur_reg = data;
}

uint8_t ymz774_read(ymz77x_t *c, uint32_t offset)
{
	if (offset & 1) {
		if (c->cur_reg == 0xe3 || c->cur_reg == 0xe4) {
			uint8_t res = 0;
			int bank = (c->cur_reg == 0xe3) ? 8 : 0;
			for (int i = 0; i < 8; i++)
				if (c->ch[i + bank].is_playing)
					res |= (uint8_t)(1 << i);
			return res;
		}
	}
	return 0;
}

//-------------------------------------------------
//  sequencers (MAME calls them once per output sample)
//-------------------------------------------------

static void sequencer_770(ymz77x_t *c)
{
	for (int i = 0; i < 8; i++) {
		ymz_sequence_t *s = &c->seq[i];
		if (!s->is_playing)
			continue;
		if (s->delay > 0)
			s->delay--;
		else {
			int reg = get_rom_byte(c, s->offset++);
			uint8_t data = get_rom_byte(c, s->offset++);
			switch (reg) {
			case 0x0f:
				for (int ch = 0; ch < 8; ch++) // might be wrong, ie not needed in case of loop
					if (s->stopchan & (1 << ch))
						c->ch[ch].is_playing = 0;
				if (s->loop)
					s->offset = get_seq_offs(c, s->sequence); // loop sequence
				else
					s->is_playing = 0;
				break;
			case 0x0e:
				s->delay = (uint32_t)s->timer * 32 + 32 - 1;
				break;
			default:
				internal_reg_write_770(c, (uint8_t)reg, data);
				break;
			}
		}
	}
}

static void sequencer_774(ymz77x_t *c)
{
	for (int i = 0; i < 16; i++) {
		ymz_channel_t *ch = &c->ch[i];
		if (ch->is_playing && !ch->is_paused && (ch->volume >> 17) != ch->current.volume_target) {
			if (ch->current.volume_delay) {
				if ((ch->volume >> 17) < ch->current.volume_target)
					ch->volume += (int32_t)c->volinc[ch->current.volume_delay];
				else
					ch->volume -= (int32_t)c->volinc[ch->current.volume_delay];
			} else
				ch->volume = (int32_t)ch->current.volume_target << 17;
		}
	}

	for (int i = 0; i < 8; i++) {
		ymz_sqc_t *sqc = &c->sqc[i];
		ymz_sequence_t *sequence = &c->seq[i];

		if (sqc->is_playing && !sqc->is_waiting) {
			// SQC consists of 4 byte records: SEQ num H, SEQ num L, SEQ Loop count, End flag (0xff)
			sequence->sequence = (uint16_t)(((get_rom_byte(c, sqc->offset) << 8) | get_rom_byte(c, sqc->offset + 1)) & 0x7ff);
			sqc->offset += 2;
			sequence->loop = get_rom_byte(c, sqc->offset++);
			sequence->offset = get_seq_offs(c, sequence->sequence);
			sequence->delay = 0;
			sequence->is_playing = 1;
			sequence->is_paused = 0; // checkme, might be not needed
			sqc->is_waiting = 1;
			if (get_rom_byte(c, sqc->offset++) == 0xff) {
				if (sqc->loop) {
					if (sqc->loop != 255)
						--sqc->loop;
					sqc->offset = get_sqc_offs(c, sqc->sqc);
				} else
					sqc->is_playing = 0;
			}
		}

		if (sequence->is_playing && !sequence->is_paused) {
			if (sequence->delay > 0)
				--sequence->delay;
			else {
				int reg = get_rom_byte(c, sequence->offset++);
				uint8_t data = get_rom_byte(c, sequence->offset++);
				switch (reg) {
				case 0xff: // end
					for (int ch = 0; ch < 16; ch++) // might be wrong, ie not needed in case of loop
						if (sequence->stopchan & (1 << ch))
							c->ch[ch].is_playing = 0;
					if (sequence->loop) {
						if (sequence->loop != 255)
							--sequence->loop;
						sequence->offset = get_seq_offs(c, sequence->sequence);
					} else {
						sequence->is_playing = 0;
						sqc->is_waiting = 0;
					}
					break;
				case 0xfe: // timer delay
					sequence->delay = (uint32_t)sequence->timer * 32 + 32 - 1;
					break;
				case 0xf0:
					sequence->bank = data & 1;
					break;
				default: {
					uint8_t temp = c->bank;
					c->bank = sequence->bank;
					if (c->bank == 0 && reg >= 0x60 && reg < 0xb0) { // if we hit SEQ registers need to add this sequence offset
						int sqn = i;
						if (reg < 0x70 || (reg >= 0x88 && reg < 0x98) || reg >= 0xa0)
							sqn = i * 2;
						internal_reg_write_774(c, (uint8_t)(reg + sqn), data);
					} else
						internal_reg_write_774(c, (uint8_t)reg, data);
					c->bank = temp;
					break;
				}
				}
			}
		}
	}
}

//-------------------------------------------------
//  sound_stream_update
//-------------------------------------------------

#define CLIP_MAX3 (32768 * 75 / 100)
#define CLIP_MAX2 (32768 * 875 / 1000)

static inline int32_t clamp32(int32_t v, int32_t lo, int32_t hi)
{
	return v < lo ? lo : v > hi ? hi : v;
}

void ymz77x_render(ymz77x_t *c, int16_t *out_lr, int frames)
{
	const int is774 = c->type == YMZ_TYPE_774;
	const int limit = (int)(c->rom_size * 8);

	for (int i = 0; i < frames; i++) {
		if (is774)
			sequencer_774(c);
		else
			sequencer_770(c);

		// process channels
		int32_t mixl = 0;
		int32_t mixr = 0;

		for (int n = 0; n < 16; n++) {
			ymz_channel_t *ch = &c->ch[n];
			rbx_mpeg_audio_t *dec = &c->dec[n];
			int16_t *out = c->output_data + n * YMZ770_OUTPUT_SIZE;

			if (ch->output_remaining == 0 && ch->is_playing && !ch->is_paused) {
				for (;;) { // MAME: retry
					if (ch->last_block) {
						if (ch->loop) {
							if (ch->loop != 255)
								--ch->loop;
							// loop sample
							ch->pending = 1;
						} else {
							ch->is_playing = 0;
							ch->output_remaining = 0;
							rbx_mpeg_audio_clear(dec);
						}
					}

					if (!ch->is_playing)
						break;

					// new phrase
					if (ch->pending) {
						int phrase = ch->phrase;
						ch->atbl = get_rom_byte(c, 4u * (uint32_t)phrase) >> 4 & 7;
						ch->pptr = (int)(8 * get_phrase_offs(c, phrase));

						// phrase transition is seamless most of the time, but AMM buffer 'contamination' should not be noticeable
						if (!ch->last_block && ch->latch.volume2 != ch->current.volume2)
							rbx_mpeg_audio_clear(dec);

						ch->pending = 0;
					}

					// next block
					int sample_rate = (int)c->sclock, channel_count;
					int ok;
					if (c->rom)
						ok = rbx_mpeg_audio_decode_buffer(dec, &ch->pptr, limit, out, &ch->output_remaining,
							&sample_rate, &channel_count, ch->atbl);
					else {
						// (ReBOX: the bytes from the position on in the window, the position relative to it)
						const uint32_t start = ((uint32_t)ch->pptr >> 3) & ~3u;
						for (uint32_t k = 0; k < YMZ_WINDOW; k += 4) {
							const uint32_t o = start + k;
							if (o >= c->rom_size) {
								memset(c->window + k, 0xff, YMZ_WINDOW - k);
								break;
							}
							memcpy(c->window + k, rom_page(c, o / YMZ_PAGE) + o % YMZ_PAGE, 4);
						}
						int wpos = ch->pptr - (int)(start * 8);
						int wlim = limit - (int)(start * 8);
						if (wlim > (int)(YMZ_WINDOW * 8))
							wlim = (int)(YMZ_WINDOW * 8);
						ok = rbx_mpeg_audio_decode_buffer(dec, &wpos, wlim, out, &ch->output_remaining,
							&sample_rate, &channel_count, ch->atbl);
						ch->pptr = wpos + (int)(start * 8);
					}
					if (!ok || ch->output_remaining == 0) {
						ch->is_playing = !ch->last_block; // detect infinite retry loop
						ch->last_block = 1;
						ch->output_remaining = 0;
						continue;
					}

					// it's not clear how exactly clock divider selection works, so far we simply set sample rate as
					// per AMM header (MAME: the stream rate follows after a synchronize; here ymz77x_rate() changes)
					if ((uint32_t)sample_rate != c->sclock)
						c->sclock = (uint32_t)sample_rate;

					ch->last_block = ch->output_remaining < 1152;
					ch->output_ptr = 0;
					ch->current = ch->latch;
					break;
				}
			}

			if (ch->output_remaining > 0) {
				// force finish current block
				int32_t smpl = out[ch->output_ptr++];

				// volume is linear, 0 - 128 (100%)
				smpl = (smpl * (ch->volume >> 17)) >> 7;
				smpl = (smpl * ch->current.volume2) >> 7;

				// pan seems linear, 0 - 128, where 0 = 100% left, 128 = 100% right, 64 = 50% left 50% right
				mixr += (smpl * ch->current.pan) >> 7;
				mixl += (smpl * (128 - ch->current.pan)) >> 7;

				ch->output_remaining--;
				if (ch->output_remaining == 0 && !ch->is_playing)
					rbx_mpeg_audio_clear(dec);
			}
		}

		if (c->mute)
			mixr = mixl = 0;
		else {
			// main volume is linear, 0 - 255, where 128 = 100%
			mixr *= c->vlma;
			mixl *= c->vlma;
			mixr >>= 7 - c->bsl;
			mixl >>= 7 - c->bsl;

			// Clip limiter: 0 - off, 1 - 6.02 dB (100%), 2 - 4.86 dB (87.5%), 3 - 3.52 dB (75%)
			switch (c->cpl) {
			case 3:
				mixl = clamp32(mixl, -CLIP_MAX3, CLIP_MAX3);
				mixr = clamp32(mixr, -CLIP_MAX3, CLIP_MAX3);
				break;
			case 2:
				mixl = clamp32(mixl, -CLIP_MAX2, CLIP_MAX2);
				mixr = clamp32(mixr, -CLIP_MAX2, CLIP_MAX2);
				break;
			case 1:
				mixl = clamp32(mixl, -32768, 32767);
				mixr = clamp32(mixr, -32768, 32767);
				break;
			}
		}

		// MAME: put_int(.., 32768) as float; the final mix clamps to full scale
		out_lr[2 * i] = (int16_t)clamp32(mixl, -32768, 32767);
		out_lr[2 * i + 1] = (int16_t)clamp32(mixr, -32768, 32767);
	}
}

uint32_t ymz77x_rate(const ymz77x_t *c)
{
	return c->sclock;
}

int ymz77x_state(ymz77x_t *c, void *ctx, int load, const char *prefix)
{
	char t[48];
	int rc = 0;
	(void)load;
#define IO(name, p, n) (snprintf(t, sizeof(t), "%s%s", prefix, name), rbx_state_io(ctx, t, p, (uint32_t)(n)))
	rc |= IO("ymz", c, YMZ77X_STATE_SIZE);
	rc |= IO("ymzout", c->output_data, 16 * YMZ770_OUTPUT_SIZE * sizeof(int16_t));
	for (int i = 0; i < 16; i++) {
		char name[24];
		snprintf(name, sizeof(name), "ymzdec%d", i);
		rc |= IO(name, &c->dec[i], RBX_MPEG_AUDIO_STATE_SIZE);
	}
#undef IO
	return rc;
}
