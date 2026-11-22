/*  The Namco module (rbx_module.h): the Namco boards (System 1, 2, 86, NA-1 / NA-2, NB-1 / NB-2, Baraduke, Sky Kid, Pac-Land, Mappy), their driver and game table, and the game list's tab.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/namcos1.inc"
#include "gen/namcos2.inc"
#include "gen/namcos86.inc"
#include "gen/baraduke.inc"
#include "gen/skykid.inc"
#include "gen/pacland.inc"
#include "gen/mappy.inc"
#include "gen/namcona1.inc"
#include "gen/namconb1.inc"

extern const rbx_driver rbx_drv_namcos1;
extern const rbx_driver rbx_drv_namcos2;
extern const rbx_driver rbx_drv_namcos86;
extern const rbx_driver rbx_drv_baraduke;
extern const rbx_driver rbx_drv_skykid;
extern const rbx_driver rbx_drv_pacland;
extern const rbx_driver rbx_drv_mappy;
extern const rbx_driver rbx_drv_namcona1;
extern const rbx_driver rbx_drv_namconb1;

static const rbx_board_tab s_boards[] = {
	{ namcos1_games, NAMCOS1_GAME_COUNT, &rbx_drv_namcos1 },
	{ namcos2_games, NAMCOS2_GAME_COUNT, &rbx_drv_namcos2 },
	{ namcos86_games, NAMCOS86_GAME_COUNT, &rbx_drv_namcos86 },
	{ baraduke_games, BARADUKE_GAME_COUNT, &rbx_drv_baraduke },
	{ skykid_games, SKYKID_GAME_COUNT, &rbx_drv_skykid },
	{ pacland_games, PACLAND_GAME_COUNT, &rbx_drv_pacland },
	{ mappy_games, MAPPY_GAME_COUNT, &rbx_drv_mappy },
	{ namcona1_games, NAMCONA1_GAME_COUNT, &rbx_drv_namcona1 },
	{ namconb1_games, NAMCONB1_GAME_COUNT, &rbx_drv_namconb1 },
};

static const char *const s_groups[] = { "Namco System 1", "Namco System 2", "Namco System 86", "Namco NA-1/NA-2", "Namco NB-1/NB-2", "Namco 8-bit" };

static int group(const rbx_game *g)
{
	return !strcmp(g->board, "Namco System 1") ? 0 : !strcmp(g->board, "Namco System 2") ? 1
	     : !strcmp(g->board, "Namco System 86") ? 2 : !strcmp(g->board, "Namco NA-1/NA-2") ? 3
	     : !strcmp(g->board, "Namco NB-1/NB-2") ? 4 : 5;   // (5: Baraduke, Sky Kid, Pac-Land, Mappy)
}

static int board_order(const char *board)
{
	(void)board;
	return 0;
}

const rbx_module rbx_mod_namco = {
	"namco", "Namco",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
