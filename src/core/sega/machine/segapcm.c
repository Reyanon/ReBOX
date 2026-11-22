/*  Sega 315-5218 SegaPCM (segapcm.h). Ported from MAME src/devices/sound/segapcm.cpp (BSD-3-Clause,
    copyright-holders: Hiromitsu Shioya, Olivier Galibert).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "segapcm.h"

uint8_t segapcm_r(const segapcm_t *p, uint8_t o)
{
	const pcm_voice *v = &p->v[(o >> 3) & 15];
	switch (o & 0x87) {
	case 0x02: return v->lvol;
	case 0x03: return v->rvol;
	case 0x04: return (uint8_t)v->loop;
	case 0x05: return (uint8_t)(v->loop >> 8);
	case 0x06: return v->end;
	case 0x07: return v->freq;
	case 0x84: return (uint8_t)(v->addr >> 8);
	case 0x85: return (uint8_t)(v->addr >> 16);
	case 0x86: return v->ctrl;
	}
	return p->ram[o];
}
void segapcm_w(segapcm_t *p, uint8_t o, uint8_t d)
{
	pcm_voice *v = &p->v[(o >> 3) & 15];
	switch (o & 0x87) {
	case 0x02: v->lvol = d; break;
	case 0x03: v->rvol = d; break;
	case 0x04: v->loop = (uint16_t)((v->loop & 0xff00) | d); break;
	case 0x05: v->loop = (uint16_t)((v->loop & 0x00ff) | d << 8); break;
	case 0x06: v->end = d; break;
	case 0x07: v->freq = d; break;
	case 0x84: v->addr = (v->addr & ~0xff00u) | (uint32_t)d << 8; break;
	case 0x85: v->addr = (v->addr & ~0xff0000u) | (uint32_t)d << 16; break;
	case 0x86: v->ctrl = d; break;
	default: p->ram[o] = d; break;
	}
}
void segapcm_tick(segapcm_t *p, int32_t *l, int32_t *r)
{
	int32_t lo = 0, ro = 0;
	for (int i = 0; i < 16; i++) {
		pcm_voice *v = &p->v[i];
		if (!(v->ctrl & 1)) {
			if ((v->addr >> 16) == v->end) {
				if (v->ctrl & 2) {
					v->ctrl |= 1;
					continue;
				}
				v->addr = (uint32_t)v->loop << 8;
			}
			const uint32_t bank = (uint32_t)(v->ctrl & p->bankmask) << p->bankshift;
			const int8_t s = (int8_t)(p->rom ? p->rom[(bank + (v->addr >> 8)) & p->mask] - 0x80 : 0);
			lo += s * (v->lvol & 0x7f);
			ro += s * (v->rvol & 0x7f);
			v->addr = (v->addr + v->freq) & 0xffffff;
		} else
			v->addr &= 0xffff00;
	}
	*l = lo;
	*r = ro;
}

