/*  The IGS module (rbx_module.h): IGS boards (PGM), their drivers and game tables, and the game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/pgm.inc"
#include "gen/pgm2.inc"

extern const rbx_driver rbx_drv_pgm;
extern const rbx_driver rbx_drv_pgm2;

static const rbx_board_tab s_boards[] = {
	{ pgm_games, PGM_GAME_COUNT, &rbx_drv_pgm },
	{ pgm2_games, PGM2_GAME_COUNT, &rbx_drv_pgm2 },
};

static const char *const s_groups[] = { "PGM", "PGM2" };

static int group(const rbx_game *g)
{
	return !strcmp(g->board, "IGS PGM2");
}

static int board_order(const char *board)
{
	return !strcmp(board, "IGS PGM2");
}

const rbx_module rbx_mod_igs = {
	"igs", "IGS",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
