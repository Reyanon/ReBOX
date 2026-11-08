/*  Seta Hardware (MAME src/mame/seta/seta.cpp, BSD-3-Clause, copyright-holders: Luca Elia): one game table, two drivers:
    seta_sp.c the machines with the X1-001 sprites alone, seta_tm.c those with X1-012 tilemaps. This picks by machine.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"

extern const rbx_driver rbx_drv_seta_sp, rbx_drv_seta_tm;
static const rbx_driver *s_d;

static int d_init(const rbx_game *g)
{
	s_d = rbx_drv_seta_tm.supported(g->machine) ? &rbx_drv_seta_tm : &rbx_drv_seta_sp;
	return s_d->init(g);
}
static void d_exit(void) { s_d->exit(); }
static void d_reset(void) { s_d->reset(); }
static void d_frame(void) { s_d->run_frame(); }
static int d_save(void *ctx) { return s_d->state_save(ctx); }
static int d_load(void *ctx) { return s_d->state_load(ctx); }
static void d_render(void) { s_d->render(); }
static int d_supported(const char *machine)
{
	return rbx_drv_seta_sp.supported(machine) || rbx_drv_seta_tm.supported(machine);
}

const rbx_driver rbx_drv_seta = { "Seta Hardware", d_init, d_exit, d_reset, d_frame, d_save, d_load, d_supported, d_render, NULL };
