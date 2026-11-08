/*  The Seta module (rbx_module.h): Seta boards, their drivers and game tables, and the game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include "gen/seta.inc"

extern const rbx_driver rbx_drv_seta;

static const rbx_board_tab s_boards[] = {
	{ seta_games, SETA_GAME_COUNT, &rbx_drv_seta },
};

static const char *const s_groups[] = { "Seta Hardware" };

static int group(const rbx_game *g)
{
	(void)g;
	return 0;
}

static int board_order(const char *board)
{
	(void)board;
	return 0;
}

const rbx_module rbx_mod_seta = {
	"seta", "Seta",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
