/*  The Toaplan module (rbx_module.h): Toaplan's boards (and Raizing's on the same hardware), their drivers and game
    tables, and the game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/truxton2.inc"
#include "gen/snowbro2.inc"
#include "gen/enmadaio.inc"
#include "gen/vfive.inc"
#include "gen/kbash.inc"
#include "gen/batsugun.inc"
#include "gen/dogyuun.inc"
#include "gen/fixeight.inc"
#include "gen/raizing.inc"
#include "gen/batrider.inc"
#include "gen/toaplan1.inc"
#include "gen/twincobr.inc"
#include "gen/wardner.inc"
#include "gen/snowbros.inc"

extern const rbx_driver rbx_drv_toaplan2;
extern const rbx_driver rbx_drv_toaplan1;
extern const rbx_driver rbx_drv_twincobr;
extern const rbx_driver rbx_drv_wardner;
extern const rbx_driver rbx_drv_snowbros;

static const rbx_board_tab s_boards[] = {
	{ truxton2_games, TRUXTON2_GAME_COUNT, &rbx_drv_toaplan2 },
	{ snowbro2_games, SNOWBRO2_GAME_COUNT, &rbx_drv_toaplan2 },
	{ enmadaio_games, ENMADAIO_GAME_COUNT, &rbx_drv_toaplan2 },
	{ vfive_games, VFIVE_GAME_COUNT, &rbx_drv_toaplan2 },
	{ kbash_games, KBASH_GAME_COUNT, &rbx_drv_toaplan2 },
	{ batsugun_games, BATSUGUN_GAME_COUNT, &rbx_drv_toaplan2 },
	{ dogyuun_games, DOGYUUN_GAME_COUNT, &rbx_drv_toaplan2 },
	{ fixeight_games, FIXEIGHT_GAME_COUNT, &rbx_drv_toaplan2 },
	{ raizing_games, RAIZING_GAME_COUNT, &rbx_drv_toaplan2 },
	{ batrider_games, BATRIDER_GAME_COUNT, &rbx_drv_toaplan2 },
	{ toaplan1_games, TOAPLAN1_GAME_COUNT, &rbx_drv_toaplan1 },
	{ twincobr_games, TWINCOBR_GAME_COUNT, &rbx_drv_twincobr },
	{ wardner_games, WARDNER_GAME_COUNT, &rbx_drv_wardner },
	{ snowbros_games, SNOWBROS_GAME_COUNT, &rbx_drv_snowbros },
};

static const char *const s_groups[] = { "GP9001", "Toaplan 1", "Twin Cobra", "Wardner", "Snow Bros" };

static int group(const rbx_game *g)
{
	if (!strcmp(g->board, "Toaplan Twin Cobra"))
		return 2;
	if (!strcmp(g->board, "Toaplan Wardner"))
		return 3;
	if (!strcmp(g->board, "Snow Bros"))
		return 4;
	return !strcmp(g->board, "Toaplan 1") ? 1 : 0;
}

static int board_order(const char *board)
{
	(void)board;
	return 0;
}

const rbx_module rbx_mod_toaplan = {
	"toaplan", "Toaplan",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
