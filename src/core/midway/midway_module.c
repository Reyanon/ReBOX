/*  The Midway module (rbx_module.h): the Williams / Midway TMS34010 boards (Y-unit, T-unit, Wolf unit), their drivers and game
    tables, and the game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#define RBX_IPT_BILL1 RBX_IPT_COIN4   // (a bill validator: a coin input here)
#include "gen/midyunit.inc"
#include "gen/midtunit.inc"
#include "gen/midwunit.inc"

extern const rbx_driver rbx_drv_midyunit;
extern const rbx_driver rbx_drv_midtunit;
extern const rbx_driver rbx_drv_midwunit;

static const rbx_board_tab s_boards[] = {
	{ midyunit_games, MIDYUNIT_GAME_COUNT, &rbx_drv_midyunit },
	{ midtunit_games, MIDTUNIT_GAME_COUNT, &rbx_drv_midtunit },
	{ midwunit_games, MIDWUNIT_GAME_COUNT, &rbx_drv_midwunit },
};

static const char *const s_groups[] = { "Y-Unit", "T-Unit", "Wolf Unit" };

static int group(const rbx_game *g)
{
	return !strcmp(g->board, "Midway T-Unit") ? 1 : !strcmp(g->board, "Midway Wolf Unit") ? 2 : 0;
}

static int board_order(const char *board)
{
	return !strcmp(board, "Midway Y-Unit") ? 0 : !strcmp(board, "Midway T-Unit") ? 1 :
	       !strcmp(board, "Midway Wolf Unit") ? 2 : 1000;
}

const rbx_module rbx_mod_midway = {
	"midway", "Midway",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
