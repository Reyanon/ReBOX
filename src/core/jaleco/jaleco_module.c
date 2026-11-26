/*  The Jaleco module (rbx_module.h): the Mega System 1 and Mega System 32 boards, their drivers and game tables, and the
    game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/megasys1.inc"
#include "gen/ms32.inc"

extern const rbx_driver rbx_drv_megasys1;
extern const rbx_driver rbx_drv_ms32;

static const rbx_board_tab s_boards[] = {
	{ megasys1_games, MEGASYS1_GAME_COUNT, &rbx_drv_megasys1 },
	{ ms32_games, MS32_GAME_COUNT, &rbx_drv_ms32 },
};

static const char *const s_groups[] = { "Mega System 1", "MS32" };

static int group(const rbx_game *g)
{
	return strcmp(g->board, "Jaleco Mega System 32") ? 0 : 1;
}

static int board_order(const char *board)
{
	return strcmp(board, "Jaleco Mega System 32") ? 0 : 1;
}

const rbx_module rbx_mod_jaleco = {
	"jaleco", "Jaleco",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
