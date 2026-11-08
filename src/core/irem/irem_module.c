/*  The Irem module (rbx_module.h): Irem boards, their drivers and game tables, and the game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <stddef.h>
#include <string.h>

#include "gen/m107.inc"
#include "gen/m92.inc"
#include "gen/m90.inc"
#include "gen/m72.inc"
#include "gen/m63.inc"
#include "gen/m62.inc"
#include "gen/m62_bkungfu.inc"
#include "gen/m58.inc"
#include "gen/m57.inc"
#include "gen/m52.inc"
#include "gen/vigilant.inc"
#include "gen/travrusa.inc"
#include "gen/spartanxtec.inc"
#include "gen/olibochu.inc"
#include "gen/redalert.inc"
#include "gen/m10.inc"
#include "gen/m14.inc"
#include "gen/m102.inc"
#include "gen/m119.inc"

// (a board not ported yet: its games are in no list)
static int none_supported(const char *machine) { (void)machine; return 0; }
static const rbx_driver rbx_drv_irem_none = { "Irem", NULL, NULL, NULL, NULL, NULL, NULL, none_supported, NULL, NULL };

extern const rbx_driver rbx_drv_m52;
extern const rbx_driver rbx_drv_m57;
extern const rbx_driver rbx_drv_m58;
extern const rbx_driver rbx_drv_m62;
extern const rbx_driver rbx_drv_m63;
extern const rbx_driver rbx_drv_travrusa;
extern const rbx_driver rbx_drv_vigilant;
extern const rbx_driver rbx_drv_spartanxtec;
extern const rbx_driver rbx_drv_olibochu;
extern const rbx_driver rbx_drv_m10;
extern const rbx_driver rbx_drv_m14;
extern const rbx_driver rbx_drv_redalert;
extern const rbx_driver rbx_drv_m92, rbx_drv_m107, rbx_drv_m72, rbx_drv_m90;

static const rbx_board_tab s_boards[] = {
	{ m107_games, M107_GAME_COUNT, &rbx_drv_m107 },
	{ m92_games, M92_GAME_COUNT, &rbx_drv_m92 },
	{ m90_games, M90_GAME_COUNT, &rbx_drv_m90 },
	{ m72_games, M72_GAME_COUNT, &rbx_drv_m72 },
	{ m63_games, M63_GAME_COUNT, &rbx_drv_m63 },
	{ m62_games, M62_GAME_COUNT, &rbx_drv_m62 },
	{ m62_bkungfu_games, M62_BKUNGFU_GAME_COUNT, &rbx_drv_irem_none },
	{ m58_games, M58_GAME_COUNT, &rbx_drv_m58 },
	{ m57_games, M57_GAME_COUNT, &rbx_drv_m57 },
	{ m52_games, M52_GAME_COUNT, &rbx_drv_m52 },
	{ vigilant_games, VIGILANT_GAME_COUNT, &rbx_drv_vigilant },
	{ travrusa_games, TRAVRUSA_GAME_COUNT, &rbx_drv_travrusa },
	{ spartanxtec_games, SPARTANXTEC_GAME_COUNT, &rbx_drv_spartanxtec },
	{ olibochu_games, OLIBOCHU_GAME_COUNT, &rbx_drv_olibochu },
	{ redalert_games, REDALERT_GAME_COUNT, &rbx_drv_redalert },
	{ m10_games, M10_GAME_COUNT, &rbx_drv_m10 },
	{ m14_games, M14_GAME_COUNT, &rbx_drv_m14 },
	{ m102_games, M102_GAME_COUNT, &rbx_drv_irem_none },
	{ m119_games, M119_GAME_COUNT, &rbx_drv_irem_none },
};

static const char *const s_groups[] = { "M107", "M92", "M90", "M72", "M63", "M62", "M58", "M57", "M52", "Classic" };

static int group(const rbx_game *g)
{
	for (int i = 0; i < 9; i++)
		if (!strcmp(g->board + 5, s_groups[i]))   // ("Irem M72")
			return i;
	return !strcmp(g->board, "Irem M62 Battle Kung Fu") ? 5 : 9;
}

static int board_order(const char *board)
{
	for (int i = 0; i < (int)(sizeof(s_boards) / sizeof(s_boards[0])); i++)
		if (s_boards[i].games[0].board && !strcmp(s_boards[i].games[0].board, board))
			return i;
	return 1000;
}

const rbx_module rbx_mod_irem = {
	"irem", "Irem",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
