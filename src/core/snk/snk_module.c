/*  The SNK module (rbx_module.h): the Neo Geo MVS, its driver and game table, and the game list's tab.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/neogeo.inc"
#include "gen/snk68.inc"
#include "gen/prehisle.inc"
#include "gen/alpha68k.inc"
#include "gen/alpha68k_i.inc"
#include "gen/alpha68k_n.inc"

extern const rbx_driver rbx_drv_neogeo;
extern const rbx_driver rbx_drv_snk68;
extern const rbx_driver rbx_drv_prehisle;
extern const rbx_driver rbx_drv_alpha68k;
extern const rbx_driver rbx_drv_alpha68k_n;

static const rbx_board_tab s_boards[] = {
	{ neogeo_games, NEOGEO_GAME_COUNT, &rbx_drv_neogeo },
	{ snk68_games, SNK68_GAME_COUNT, &rbx_drv_snk68 },
	{ prehisle_games, PREHISLE_GAME_COUNT, &rbx_drv_prehisle },
	{ alpha68k_games, ALPHA68K_GAME_COUNT, &rbx_drv_alpha68k },
	{ alpha68k_i_games, ALPHA68K_I_GAME_COUNT, &rbx_drv_alpha68k_n },
	{ alpha68k_n_games, ALPHA68K_N_GAME_COUNT, &rbx_drv_alpha68k_n },
};

static const char *const s_groups[] = { "Neo Geo", "SNK 68000", "Prehistoric Isle", "Alpha 68K", "Alpha 68K I", "Alpha 68K N" };

static int group(const rbx_game *g)
{
	return !strcmp(g->board, "SNK 68000") ? 1 : !strcmp(g->board, "SNK Prehistoric Isle") ? 2
	     : !strcmp(g->board, "SNK Alpha 68K") ? 3 : !strcmp(g->board, "SNK Alpha 68K I") ? 4
	     : !strcmp(g->board, "SNK Alpha 68K N") ? 5 : 0;
}

static int board_order(const char *board)
{
	(void)board;
	return 0;
}

const rbx_module rbx_mod_snk = {
	"snk", "SNK",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
