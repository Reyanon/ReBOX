/*  The Video System module (rbx_module.h): the boards of MAME's vsystem/pspikes.cpp and aerofgt.cpp, their drivers and
    game tables, and the game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/pspikes.inc"
#include "gen/aerofgt.inc"

extern const rbx_driver rbx_drv_pspikes, rbx_drv_aerofgt;

static const rbx_board_tab s_boards[] = {
	{ pspikes_games, PSPIKES_GAME_COUNT, &rbx_drv_pspikes },
	{ aerofgt_games, AEROFGT_GAME_COUNT, &rbx_drv_aerofgt },
};

static const char *const s_groups[] = { "Power Spikes", "Aero Fighters" };

static int group(const rbx_game *g)
{
	return strcmp(g->board, "Video System Aero Fighters") ? 0 : 1;
}

static int board_order(const char *board)
{
	return strcmp(board, "Video System Aero Fighters") ? 0 : 1;
}

const rbx_module rbx_mod_vsystem = {
	"vsystem", "Video System",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
