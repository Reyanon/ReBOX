/*  NEC uPD96050 DSP, ported to C from MAME src/devices/cpu/upd7725/upd7725.cpp (BSD-3-Clause, copyright-holders:
    R. Belmont, byuu; original by byuu in the public domain). See upd7725.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "upd7725.h"

#include <string.h>

// the Status register's bits (MAME's struct Status, packed as its operator unsigned)
#define SR_RQM  0x8000
#define SR_DRS  0x1000
#define SR_DRC  0x0400
#define SR_EI   0x0080
#define SR_MASK 0xff83

static inline unsigned flag_pack(const necdsp_flag *f)
{
	return (unsigned)(f->s1 << 5 | f->s0 << 4 | f->c << 3 | f->z << 2 | f->ov1 << 1 | f->ov0);
}

void upd96050_init(upd96050_t *d, const uint8_t *prg, uint32_t prg_bytes, const uint8_t *data, uint32_t data_bytes)
{
	memset(d, 0, sizeof(*d));
	d->prg = prg;
	d->prg_words = prg_bytes / 4;
	d->data = data;
	d->data_words = data_bytes / 2;
	// device_start: the registers the /RESET line leaves (all 0 here: dataRAM, rp, dp, sp, k .. idb)
	upd96050_reset(d);
}

void upd96050_reset(upd96050_t *d)
{
	necdsp_regs *r = &d->regs;
	r->pc = 0;
	r->sr = 0;
	memset(&r->flaga, 0, sizeof(r->flaga));
	memset(&r->flagb, 0, sizeof(r->flagb));
	r->siack = 0;
	r->soack = 0;
	d->irq_firing = 0;
	d->writes++;
}

static inline uint32_t prg_read(const upd96050_t *d, uint16_t pc)
{
	const uint32_t a = pc & 0x3fff;
	if (a >= d->prg_words)
		return 0;
	const uint8_t *p = d->prg + a * 4;
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

static inline uint16_t data_read(const upd96050_t *d, uint16_t a)
{
	a &= 0x0fff;
	if (a >= 0x800 || a >= d->data_words)
		return 0;
	const uint8_t *p = d->data + a * 2;
	return (uint16_t)(p[0] << 8 | p[1]);
}

static void exec_ld(upd96050_t *d, uint32_t opcode)
{
	necdsp_regs *r = &d->regs;
	const uint16_t id = (uint16_t)(opcode >> 6);
	const uint8_t dst = opcode & 0xf;
	r->idb = id;
	switch (dst) {
	case 0: break;
	case 1: r->a = (int16_t)id; break;
	case 2: r->b = (int16_t)id; break;
	case 3: r->tr = id; break;
	case 4: r->dp = id; break;
	case 5: r->rp = id; break;
	case 6: r->dr = id; r->sr |= SR_RQM; d->writes++; break;
	case 7: r->sr = (uint16_t)(((r->sr & 0x907c) | (id & ~0x907c)) & SR_MASK); d->writes++; break;
	case 8: {   // (LSB first output)
		uint16_t v = 0;
		for (int i = 0; i < 16; i++)
			v |= (uint16_t)(((id >> i) & 1) << (15 - i));
		r->so = v;
		break;
	}
	case 9: r->so = id; break;
	case 10: r->k = (int16_t)id; break;
	case 11: r->k = (int16_t)id; r->l = (int16_t)data_read(d, r->rp); break;
	case 12: r->l = (int16_t)id; r->k = (int16_t)d->dataRAM[(r->dp & 0x7ff) | 0x40]; break;
	case 13: r->l = (int16_t)id; break;
	case 14: r->trb = id; break;
	case 15: d->dataRAM[r->dp & 0x7ff] = id; d->writes++; break;
	}
}

static void exec_op(upd96050_t *d, uint32_t opcode)
{
	necdsp_regs *r = &d->regs;
	const uint8_t pselect = (opcode >> 20) & 0x3;
	const uint8_t alu = (opcode >> 16) & 0xf;
	const uint8_t asl = (opcode >> 15) & 0x1;
	const uint8_t dpl = (opcode >> 13) & 0x3;
	const uint8_t dphm = (opcode >> 9) & 0xf;
	const uint8_t rpdcr = (opcode >> 8) & 0x1;
	const uint8_t src = (opcode >> 4) & 0xf;
	const uint8_t dst = opcode & 0xf;

	switch (src) {
	case 0: r->idb = r->trb; break;
	case 1: r->idb = (uint16_t)r->a; break;
	case 2: r->idb = (uint16_t)r->b; break;
	case 3: r->idb = r->tr; break;
	case 4: r->idb = r->dp; break;
	case 5: r->idb = r->rp; break;
	case 6: r->idb = data_read(d, r->rp); break;
	case 7: r->idb = (uint16_t)(0x8000 - r->flaga.s1); break;
	case 8: r->idb = r->dr; r->sr |= SR_RQM; d->writes++; break;
	case 9: r->idb = r->dr; break;
	case 10: r->idb = r->sr; break;
	case 11: r->idb = r->si; break;
	case 12: {
		uint16_t v = 0;
		for (int i = 0; i < 16; i++)
			v |= (uint16_t)(((r->si >> i) & 1) << (15 - i));
		r->idb = v;
		break;
	}
	case 13: r->idb = (uint16_t)r->k; break;
	case 14: r->idb = (uint16_t)r->l; break;
	case 15: r->idb = d->dataRAM[r->dp & 0x07ff]; break;
	}

	if (alu) {
		uint16_t p = 0, q = 0, res = 0;
		necdsp_flag flag;
		int c = 0;
		switch (pselect) {
		case 0: p = d->dataRAM[r->dp & 0x07ff]; break;
		case 1: p = r->idb; break;
		case 2: p = (uint16_t)r->m; break;
		case 3: p = (uint16_t)r->n; break;
		}
		if (!asl) { q = (uint16_t)r->a; flag = r->flaga; c = r->flagb.c; }
		else { q = (uint16_t)r->b; flag = r->flagb; c = r->flaga.c; }
		switch (alu) {
		case 1: res = q | p; break;
		case 2: res = q & p; break;
		case 3: res = q ^ p; break;
		case 4: res = (uint16_t)(q - p); break;
		case 5: res = (uint16_t)(q + p); break;
		case 6: res = (uint16_t)(q - p - c); break;
		case 7: res = (uint16_t)(q + p + c); break;
		case 8: res = (uint16_t)(q - 1); p = 1; break;
		case 9: res = (uint16_t)(q + 1); p = 1; break;
		case 10: res = (uint16_t)~q; break;
		case 11: res = (uint16_t)((q >> 1) | (q & 0x8000)); break;
		case 12: res = (uint16_t)((q << 1) | (c ? 1 : 0)); break;
		case 13: res = (uint16_t)((q << 2) | 3); break;
		case 14: res = (uint16_t)((q << 4) | 15); break;
		case 15: res = (uint16_t)((q << 8) | (q >> 8)); break;
		}
		flag.s0 = (res & 0x8000) != 0;
		flag.z = res == 0;
		if (!flag.ov1)
			flag.s1 = flag.s0;
		switch (alu) {
		case 1: case 2: case 3: case 10: case 13: case 14: case 15:
			flag.c = 0;
			flag.ov0 = flag.ov1 = 0;
			break;
		case 4: case 5: case 6: case 7: case 8: case 9:
			if (alu & 1) {
				flag.ov0 = ((q ^ res) & ~(q ^ p) & 0x8000) != 0;
				flag.c = res < q;
			} else {
				flag.ov0 = ((q ^ res) & (q ^ p) & 0x8000) != 0;
				flag.c = res > q;
			}
			flag.ov1 = (flag.ov0 & flag.ov1) ? (flag.s1 == flag.s0) : (flag.ov0 | flag.ov1);
			break;
		case 11:
			flag.c = q & 1;
			flag.ov0 = flag.ov1 = 0;
			break;
		case 12:
			flag.c = q >> 15;
			flag.ov0 = flag.ov1 = 0;
			break;
		}
		if (!asl) { r->a = (int16_t)res; r->flaga = flag; }
		else { r->b = (int16_t)res; r->flagb = flag; }
	}

	exec_ld(d, ((uint32_t)r->idb << 6) + dst);

	if (dst != 4) {
		switch (dpl) {
		case 1: r->dp = (uint16_t)((r->dp & 0xf0) + ((r->dp + 1) & 0x0f)); break;
		case 2: r->dp = (uint16_t)((r->dp & 0xf0) + ((r->dp - 1) & 0x0f)); break;
		case 3: r->dp = (uint16_t)(r->dp & 0xf0); break;
		}
		r->dp ^= (uint16_t)(dphm << 4);
	}
	if (rpdcr && dst != 5)
		r->rp--;
}

static void exec_rt(upd96050_t *d, uint32_t opcode)
{
	necdsp_regs *r = &d->regs;
	exec_op(d, opcode);
	r->pc = r->stack[--r->sp & 0xf];
	r->sp &= 0xf;
}

static void exec_jp(upd96050_t *d, uint32_t opcode)
{
	necdsp_regs *r = &d->regs;
	const uint16_t brch = (opcode >> 13) & 0x1ff;
	const uint16_t na = (opcode >> 2) & 0x7ff;
	const uint16_t bank = opcode & 0x3;
	const uint16_t jps = (uint16_t)((r->pc & 0x2000) | (bank << 11) | na);
	const uint16_t jpl = (uint16_t)((bank << 11) | na);
	const necdsp_flag *fa = &r->flaga, *fb = &r->flagb;
	int take = 0;
	switch (brch) {
	case 0x000: r->pc = r->so; return;
	case 0x080: take = fa->c == 0; break;
	case 0x082: take = fa->c == 1; break;
	case 0x084: take = fb->c == 0; break;
	case 0x086: take = fb->c == 1; break;
	case 0x088: take = fa->z == 0; break;
	case 0x08a: take = fa->z == 1; break;
	case 0x08c: take = fb->z == 0; break;
	case 0x08e: take = fb->z == 1; break;
	case 0x090: take = fa->ov0 == 0; break;
	case 0x092: take = fa->ov0 == 1; break;
	case 0x094: take = fb->ov0 == 0; break;
	case 0x096: take = fb->ov0 == 1; break;
	case 0x098: take = fa->ov1 == 0; break;
	case 0x09a: take = fa->ov1 == 1; break;
	case 0x09c: take = fb->ov1 == 0; break;
	case 0x09e: take = fb->ov1 == 1; break;
	case 0x0a0: take = fa->s0 == 0; break;
	case 0x0a2: take = fa->s0 == 1; break;
	case 0x0a4: take = fb->s0 == 0; break;
	case 0x0a6: take = fb->s0 == 1; break;
	case 0x0a8: take = fa->s1 == 0; break;
	case 0x0aa: take = fa->s1 == 1; break;
	case 0x0ac: take = fb->s1 == 0; break;
	case 0x0ae: take = fb->s1 == 1; break;
	case 0x0b0: take = (r->dp & 0x0f) == 0x00; break;
	case 0x0b1: take = (r->dp & 0x0f) != 0x00; break;
	case 0x0b2: take = (r->dp & 0x0f) == 0x0f; break;
	case 0x0b3: take = (r->dp & 0x0f) != 0x0f; break;
	case 0x0b4: take = r->siack == 0; break;
	case 0x0b6: take = r->siack == 1; break;
	case 0x0b8: take = r->soack == 0; break;
	case 0x0ba: take = r->soack == 1; break;
	case 0x0bc: take = (r->sr & SR_RQM) == 0; break;
	case 0x0be: take = (r->sr & SR_RQM) != 0; break;
	case 0x100: r->pc = jpl; return;
	case 0x101: r->pc = (uint16_t)(0x2000 | jpl); return;
	case 0x140: r->stack[r->sp++ & 0xf] = r->pc; r->pc = jpl; r->sp &= 0xf; return;
	case 0x141: r->stack[r->sp++ & 0xf] = r->pc; r->pc = (uint16_t)(0x2000 | jpl); r->sp &= 0xf; return;
	}
	if (take)
		r->pc = jps;
}

void upd96050_run(upd96050_t *d, int64_t cycles)
{
	necdsp_regs *r = &d->regs;
	while (cycles > 0) {
		uint32_t opcode;
		const uint16_t pc0 = r->pc;
		if (d->irq_firing == 0) {
			opcode = prg_read(d, r->pc) >> 8;
			r->pc++;
		} else if (d->irq_firing == 1) {
			opcode = 0x000000;
			d->irq_firing = 2;
		} else {
			opcode = 0xA80400;
			d->irq_firing = 0;
		}
		switch (opcode >> 22) {
		case 0: exec_op(d, opcode); break;
		case 1: exec_rt(d, opcode); break;
		case 2: exec_jp(d, opcode); break;
		case 3: exec_ld(d, opcode); break;
		}
		const int32_t result = (int32_t)r->k * r->l;
		r->m = (int16_t)(result >> 15);
		r->n = (int16_t)(result << 1);
		d->total++;
		cycles--;
		// the idle loop check at a backward jump taken
		if ((opcode >> 22) == 2 && r->pc <= pc0 && !d->irq_firing) {
			if (r->pc == d->idle_pc && d->idle_writes == d->writes && d->total > d->idle_at &&
			    !memcmp(&d->idle_regs, r, sizeof(*r))) {
				const int64_t period = d->total - d->idle_at;
				const int64_t skip = cycles / period * period;
				cycles -= skip;
				d->total += skip;
				d->idle_at = d->total;
			} else {
				d->idle_pc = r->pc;
				d->idle_at = d->total;
				d->idle_writes = d->writes;
				memcpy(&d->idle_regs, r, sizeof(*r));
			}
		}
	}
}

uint8_t upd96050_status_r(upd96050_t *d)
{
	return (uint8_t)(d->regs.sr >> 8);
}

uint8_t upd96050_data_r(upd96050_t *d)
{
	necdsp_regs *r = &d->regs;
	d->writes++;
	if (!(r->sr & SR_DRC)) {
		if (!(r->sr & SR_DRS)) {
			r->sr |= SR_DRS;
			return (uint8_t)r->dr;
		}
		r->sr &= ~(SR_RQM | SR_DRS);
		return (uint8_t)(r->dr >> 8);
	}
	r->sr &= ~SR_RQM;
	return (uint8_t)r->dr;
}

void upd96050_data_w(upd96050_t *d, uint8_t data)
{
	necdsp_regs *r = &d->regs;
	d->writes++;
	if (!(r->sr & SR_DRC)) {
		if (!(r->sr & SR_DRS)) {
			r->sr |= SR_DRS;
			r->dr = (uint16_t)((r->dr & 0xff00) | data);
		} else {
			r->sr &= ~(SR_RQM | SR_DRS);
			r->dr = (uint16_t)((data << 8) | (r->dr & 0x00ff));
		}
	} else {
		r->sr &= ~SR_RQM;
		r->dr = (uint16_t)((r->dr & 0xff00) | data);
	}
}
