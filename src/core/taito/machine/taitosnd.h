/*  Taito TC0140SYT / PC060HA main <-> sound CPU communication, ported from MAME
    src/mame/shared/taitosnd.cpp (BSD-3-Clause, copyright-holders: Philip Bennett).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TAITOSND_H
#define RBX_TAITOSND_H

#include <stdint.h>

typedef struct {
	uint8_t slavedata[4], masterdata[4];
	uint8_t mainmode, submode, status, nmi_enabled;
	uint8_t nmi_line, reset_line;
	void (*nmi)(int state);      // sound CPU NMI line
	void (*reset)(int state);    // sound CPU reset line
	void (*sync)(void);          // called before main-side accesses (let the sound CPU catch up)
} tc0140syt_t;

void tc0140syt_reset(tc0140syt_t *t);
void tc0140syt_master_port_w(tc0140syt_t *t, uint8_t data);
void tc0140syt_master_comm_w(tc0140syt_t *t, uint8_t data);
uint8_t tc0140syt_master_comm_r(tc0140syt_t *t);
void tc0140syt_slave_port_w(tc0140syt_t *t, uint8_t data);
void tc0140syt_slave_comm_w(tc0140syt_t *t, uint8_t data);
uint8_t tc0140syt_slave_comm_r(tc0140syt_t *t);

#endif
