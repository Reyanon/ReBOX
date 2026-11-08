/*  Taito TC0140SYT / PC060HA main <-> sound CPU communication, ported from MAME
    src/mame/shared/taitosnd.cpp (BSD-3-Clause, copyright-holders: Philip Bennett).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "taitosnd.h"

#include <string.h>

#define PORT01_FULL        0x01
#define PORT23_FULL        0x02
#define PORT01_FULL_MASTER 0x04
#define PORT23_FULL_MASTER 0x08

void tc0140syt_reset(tc0140syt_t *t)
{
	memset(t->slavedata, 0, 4);
	memset(t->masterdata, 0, 4);
	t->mainmode = t->submode = t->status = t->nmi_enabled = 0;
	t->nmi_line = 0;
}

static void update_nmi(tc0140syt_t *t)
{
	int line = (t->status & (PORT23_FULL | PORT01_FULL)) && t->nmi_enabled;
	if (line != t->nmi_line) {
		t->nmi_line = (uint8_t)line;
		if (t->nmi)
			t->nmi(line);
	}
}

void tc0140syt_master_port_w(tc0140syt_t *t, uint8_t data)
{
	t->mainmode = data & 0x0f;
}

void tc0140syt_master_comm_w(tc0140syt_t *t, uint8_t data)
{
	if (t->sync)
		t->sync();
	data &= 0x0f;
	switch (t->mainmode) {
	case 0: t->slavedata[t->mainmode++] = data; break;
	case 1: t->slavedata[t->mainmode++] = data; t->status |= PORT01_FULL; update_nmi(t); break;
	case 2: t->slavedata[t->mainmode++] = data; break;
	case 3: t->slavedata[t->mainmode++] = data; t->status |= PORT23_FULL; update_nmi(t); break;
	case 4:
		t->reset_line = data ? 1 : 0;
		if (t->reset)
			t->reset(t->reset_line);
		break;
	}
}

uint8_t tc0140syt_master_comm_r(tc0140syt_t *t)
{
	if (t->sync)
		t->sync();
	uint8_t res = 0;
	switch (t->mainmode) {
	case 0: res = t->masterdata[t->mainmode++]; break;
	case 1: res = t->masterdata[t->mainmode]; t->status &= ~PORT01_FULL_MASTER; t->mainmode++; break;
	case 2: res = t->masterdata[t->mainmode++]; break;
	case 3: res = t->masterdata[t->mainmode]; t->status &= ~PORT23_FULL_MASTER; t->mainmode++; break;
	case 4: res = t->status; break;
	}
	return res;
}

void tc0140syt_slave_port_w(tc0140syt_t *t, uint8_t data)
{
	t->submode = data & 0x0f;
}

void tc0140syt_slave_comm_w(tc0140syt_t *t, uint8_t data)
{
	data &= 0x0f;
	switch (t->submode) {
	case 0: t->masterdata[t->submode++] = data; break;
	case 1: t->masterdata[t->submode++] = data; t->status |= PORT01_FULL_MASTER; break;
	case 2: t->masterdata[t->submode++] = data; break;
	case 3: t->masterdata[t->submode++] = data; t->status |= PORT23_FULL_MASTER; break;
	case 4: break;
	case 5: t->nmi_enabled = 0; update_nmi(t); break;
	case 6: t->nmi_enabled = 1; update_nmi(t); break;
	}
}

uint8_t tc0140syt_slave_comm_r(tc0140syt_t *t)
{
	uint8_t res = 0;
	switch (t->submode) {
	case 0: res = t->slavedata[t->submode++]; break;
	case 1: res = t->slavedata[t->submode]; t->status &= ~PORT01_FULL; t->submode++; update_nmi(t); break;
	case 2: res = t->slavedata[t->submode++]; break;
	case 3: res = t->slavedata[t->submode]; t->status &= ~PORT23_FULL; t->submode++; update_nmi(t); break;
	case 4: res = t->status; break;
	}
	return res;
}
