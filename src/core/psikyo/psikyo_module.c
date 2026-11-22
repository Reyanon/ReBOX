/*  The Psikyo module (rbx_module.h): the 68EC020 and SH-2 boards, their drivers and game table, and the game list's tab.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/psikyo.inc"
#include "gen/psikyosh.inc"
#include "gen/psikyo4.inc"

extern const rbx_driver rbx_drv_psikyo;
extern const rbx_driver rbx_drv_psikyosh;
extern const rbx_driver rbx_drv_psikyo4;

static const rbx_board_tab s_boards[] = {
	{ psikyo_games, PSIKYO_GAME_COUNT, &rbx_drv_psikyo },
	{ psikyosh_games, PSIKYOSH_GAME_COUNT, &rbx_drv_psikyosh },
	{ psikyo4_games, PSIKYO4_GAME_COUNT, &rbx_drv_psikyo4 },
};

static const char *const s_groups[] = { "68EC020", "PS3/PS5", "PS4" };

static int group(const rbx_game *g)
{
	return !strcmp(g->board, "Psikyo PS3/PS5") ? 1 : !strcmp(g->board, "Psikyo PS4") ? 2 : 0;
}

static int board_order(const char *board)
{
	(void)board;
	return 0;
}

const rbx_module rbx_mod_psikyo = {
	"psikyo", "Psikyo",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
