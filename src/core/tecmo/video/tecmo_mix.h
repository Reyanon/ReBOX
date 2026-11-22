/*  Tecmo's layer mixer with additive blending. Ported from MAME src/mame/tecmo/tecmo_mix.cpp (BSD-3-Clause,
    copyright-holders: David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TECMO_MIX_H
#define RBX_TECMO_MIX_H

#include <stdint.h>

typedef struct {
	int sprpri_shift, sprbln_shift, sprcol_shift;
	int spblend_source, fgblend_source;
	int bgblend_comp, fgblend_comp, txblend_comp, spblend_comp;
	int bgregular_comp, fgregular_comp, txregular_comp, spregular_comp;
	int bgpen, bgpen_blend;
	int revspritetile;   // set_revspritetile: 3
	uint32_t rand;       // (machine().rand() for the cases MAME leaves undone)
} tecmo_mix_t;

// mix_bitmaps for one line: the layers' pixels as MAME's ind16 bitmaps (tx may be NULL), pal the pens as 0x00RRGGBB
void tecmo_mix_line(tecmo_mix_t *m, const uint32_t *pal, const uint16_t *bg, const uint16_t *fg, const uint16_t *tx,
                    const uint16_t *sp, uint32_t *out, int n);

#endif
