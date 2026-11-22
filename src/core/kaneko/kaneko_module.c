/*  The Kaneko module (rbx_module.h): the Kaneko 16-bit boards, their driver and game table, and the game list's tab.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include "gen/kaneko16.inc"

extern const rbx_driver rbx_drv_kaneko16;

static const rbx_board_tab s_boards[] = {
	{ kaneko16_games, KANEKO16_GAME_COUNT, &rbx_drv_kaneko16 },
};

static const char *const s_groups[] = { "Kaneko16" };

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

const rbx_module rbx_mod_kaneko = {
	"kaneko", "Kaneko",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
