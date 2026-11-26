/*  The Eolith module (rbx_module.h): the Eolith Hyperstone boards (Gradation 2D, the 16-bit board), their drivers
    and game tables, and the game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/eolith.inc"
#include "gen/eolith16.inc"

extern const rbx_driver rbx_drv_eolith;
extern const rbx_driver rbx_drv_eolith16;

static const rbx_board_tab s_boards[] = {
	{ eolith_games, EOLITH_GAME_COUNT, &rbx_drv_eolith },
	{ eolith16_games, EOLITH16_GAME_COUNT, &rbx_drv_eolith16 },
};

static const char *const s_groups[] = { "Eolith" };

static int group(const rbx_game *g)
{
	(void)g;
	return 0;
}

static int board_order(const char *board)
{
	return !strcmp(board, "Eolith Gradation 2D") ? 0 : !strcmp(board, "Eolith 16-bit") ? 1 : 1000;
}

const rbx_module rbx_mod_eolith = {
	"eolith", "Eolith",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
