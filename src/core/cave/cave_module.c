/*  The Cave module (rbx_module.h): Cave's 68000 boards (MAME keeps the driver with Atlus) and the CV1000 (SH-3),
    their drivers and game tables, and the game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/cave.inc"
#include "gen/cv1k.inc"

extern const rbx_driver rbx_drv_cave;
extern const rbx_driver rbx_drv_cv1k;

static const rbx_board_tab s_boards[] = {
	{ cave_games, CAVE_GAME_COUNT, &rbx_drv_cave },
	{ cv1k_games, CV1K_GAME_COUNT, &rbx_drv_cv1k },
};

static const char *const s_groups[] = { "Cave", "CV1000" };

static int group(const rbx_game *g)
{
	return !strcmp(g->board, "Cave CV1000") ? 1 : 0;
}

static int board_order(const char *board)
{
	return !strcmp(board, "Cave CV1000") ? 1 : 0;
}

const rbx_module rbx_mod_cave = {
	"cave", "Cave",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
