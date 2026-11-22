/*  The Tecmo module (rbx_module.h): the Tecmo boards, their drivers and game tables, and the game list's tab.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/gaiden.inc"
#include "gen/tecmo16.inc"
#include "gen/spbactn.inc"

extern const rbx_driver rbx_drv_gaiden;
extern const rbx_driver rbx_drv_tecmo16;
extern const rbx_driver rbx_drv_spbactn;

static const rbx_board_tab s_boards[] = {
	{ gaiden_games, GAIDEN_GAME_COUNT, &rbx_drv_gaiden },
	{ tecmo16_games, TECMO16_GAME_COUNT, &rbx_drv_tecmo16 },
	{ spbactn_games, SPBACTN_GAME_COUNT, &rbx_drv_spbactn },
};

static const char *const s_groups[] = { "Ninja Gaiden", "Tecmo 16", "Super Pinball Action" };

static int group(const rbx_game *g)
{
	if (!strcmp(g->board, "Tecmo 16"))
		return 1;
	if (!strcmp(g->board, "Tecmo Super Pinball Action"))
		return 2;
	return 0;
}

static int board_order(const char *board)
{
	(void)board;
	return 0;
}

const rbx_module rbx_mod_tecmo = {
	"tecmo", "Tecmo",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
