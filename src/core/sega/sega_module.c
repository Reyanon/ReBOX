/*  The Sega module (rbx_module.h): Sega's boards (System 1 / 2, System 16A / 16B / 18, Out Run, X / Y-Board, System 24, System 32), their drivers and game tables, and the game list's tab.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/segas16a.inc"
#include "gen/segas16b.inc"
#include "gen/segas18.inc"
#include "gen/segaxbd.inc"
#include "gen/segaybd.inc"
#include "gen/segaorun.inc"
#include "gen/segas24.inc"
#include "gen/segas32.inc"
#include "gen/system1.inc"

extern const rbx_driver rbx_drv_segas16a;
extern const rbx_driver rbx_drv_segas16b;
extern const rbx_driver rbx_drv_segas18;
extern const rbx_driver rbx_drv_segaxbd;
extern const rbx_driver rbx_drv_segaybd;
extern const rbx_driver rbx_drv_segaorun;
extern const rbx_driver rbx_drv_segas24;
extern const rbx_driver rbx_drv_segas32;
extern const rbx_driver rbx_drv_system1;

static const rbx_board_tab s_boards[] = {
	{ segas16a_games, SEGAS16A_GAME_COUNT, &rbx_drv_segas16a },
	{ segas16b_games, SEGAS16B_GAME_COUNT, &rbx_drv_segas16b },
	{ segas18_games, SEGAS18_GAME_COUNT, &rbx_drv_segas18 },
	{ segaxbd_games, SEGAXBD_GAME_COUNT, &rbx_drv_segaxbd },
	{ segaybd_games, SEGAYBD_GAME_COUNT, &rbx_drv_segaybd },
	{ segaorun_games, SEGAORUN_GAME_COUNT, &rbx_drv_segaorun },
	{ segas24_games, SEGAS24_GAME_COUNT, &rbx_drv_segas24 },
	{ segas32_games, SEGAS32_GAME_COUNT, &rbx_drv_segas32 },
	{ system1_games, SYSTEM1_GAME_COUNT, &rbx_drv_system1 },
};

static const char *const s_groups[] = { "System 16", "X-Board", "Y-Board", "System 24", "System 32", "Out Run", "System 1" };

static int group(const rbx_game *g)
{
	return strcmp(g->board, "Sega X-Board") == 0 ? 1 : strcmp(g->board, "Sega Y-Board") == 0 ? 2
	     : strcmp(g->board, "Sega System 24") == 0 ? 3 : strcmp(g->board, "Sega System 32") == 0 ? 4
	     : strcmp(g->board, "Sega Out Run") == 0 ? 5 : strcmp(g->board, "Sega System 1") == 0 ? 6 : 0;
}

static int board_order(const char *board)
{
	return strcmp(board, "Sega System 16A") == 0 ? 0 : strcmp(board, "Sega System 16B") == 0 ? 1
	     : strcmp(board, "Sega System 18") == 0 ? 2 : strcmp(board, "Sega System 32") == 0 ? 4
	     : strcmp(board, "Sega System 1") == 0 ? 5 : 3;
}

const rbx_module rbx_mod_sega = {
	"sega", "Sega",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
