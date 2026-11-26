/*  The Seta module (rbx_module.h): Seta boards (Seta Hardware, Seta 2), their drivers and game tables, and the game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/seta.inc"
#include "gen/seta2.inc"

extern const rbx_driver rbx_drv_seta;
extern const rbx_driver rbx_drv_seta2;

static const rbx_board_tab s_boards[] = {
	{ seta_games, SETA_GAME_COUNT, &rbx_drv_seta },
	{ seta2_games, SETA2_GAME_COUNT, &rbx_drv_seta2 },
};

static const char *const s_groups[] = { "Seta Hardware", "Seta 2" };

static int group(const rbx_game *g)
{
	return strcmp(g->board, "Seta 2") == 0 ? 1 : 0;
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
