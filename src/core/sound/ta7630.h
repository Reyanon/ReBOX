/*  Toshiba TA7630P volume control, ported from MAME src/devices/sound/ta7630.cpp
    (BSD-3-Clause, copyright-holders: Angelo Salese): only the volume table (as MAME, no balance / tone filters).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TA7630_H
#define RBX_TA7630_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// vol[0..15] in 1/256 units (MAME m_vol_ctrl: 1.5 dB steps growing by 0.125 dB, 15 = 1.0)
static inline void ta7630_table(int vol[16])
{
	double db = 0.0, step = 1.50;
	for (int i = 0; i < 16; i++) {
		double max = 100.0 / pow(10.0, db / 20.0);
		vol[15 - i] = (int)(max / 100.0 * 256.0 + 0.5);
		db += step;
		step += 0.125;
	}
}

#ifdef __cplusplus
}
#endif

#endif
