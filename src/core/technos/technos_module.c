/*  The Technos module (rbx_module.h): the Technos boards, their drivers and game table, and the game list's tab.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/ddragon3.inc"
#include "gen/ddragon.inc"
#include "gen/chinagat.inc"
#include "gen/wwfsstar.inc"
#include "gen/shadfrce.inc"
#include "gen/blockout.inc"
#include "gen/renegade.inc"
#include "gen/xain.inc"
#include "gen/spdodgeb.inc"
#include "gen/vball.inc"
#include "gen/mystston.inc"
#include "gen/matmania.inc"
#include "gen/battlane.inc"
#include "gen/bogeyman.inc"
#include "gen/dogfgt.inc"
#include "gen/ssozumo.inc"
#include "gen/tagteam.inc"

extern const rbx_driver rbx_drv_ddragon3;
extern const rbx_driver rbx_drv_ddragon;
extern const rbx_driver rbx_drv_wwfsstar;
extern const rbx_driver rbx_drv_shadfrce;
extern const rbx_driver rbx_drv_blockout;
extern const rbx_driver rbx_drv_renegade;
extern const rbx_driver rbx_drv_xain;
extern const rbx_driver rbx_drv_spdodgeb;
extern const rbx_driver rbx_drv_vball;
extern const rbx_driver rbx_drv_mystston;
extern const rbx_driver rbx_drv_matmania;
extern const rbx_driver rbx_drv_battlane;
extern const rbx_driver rbx_drv_bogeyman;
extern const rbx_driver rbx_drv_dogfgt;
extern const rbx_driver rbx_drv_ssozumo;
extern const rbx_driver rbx_drv_tagteam;

static const rbx_board_tab s_boards[] = {
	{ ddragon3_games, DDRAGON3_GAME_COUNT, &rbx_drv_ddragon3 },
	{ ddragon_games, DDRAGON_GAME_COUNT, &rbx_drv_ddragon },
	{ chinagat_games, CHINAGAT_GAME_COUNT, &rbx_drv_ddragon },
	{ wwfsstar_games, WWFSSTAR_GAME_COUNT, &rbx_drv_wwfsstar },
	{ shadfrce_games, SHADFRCE_GAME_COUNT, &rbx_drv_shadfrce },
	{ blockout_games, BLOCKOUT_GAME_COUNT, &rbx_drv_blockout },
	{ renegade_games, RENEGADE_GAME_COUNT, &rbx_drv_renegade },
	{ xain_games, XAIN_GAME_COUNT, &rbx_drv_xain },
	{ spdodgeb_games, SPDODGEB_GAME_COUNT, &rbx_drv_spdodgeb },
	{ vball_games, VBALL_GAME_COUNT, &rbx_drv_vball },
	{ mystston_games, MYSTSTON_GAME_COUNT, &rbx_drv_mystston },
	{ matmania_games, MATMANIA_GAME_COUNT, &rbx_drv_matmania },
	{ battlane_games, BATTLANE_GAME_COUNT, &rbx_drv_battlane },
	{ bogeyman_games, BOGEYMAN_GAME_COUNT, &rbx_drv_bogeyman },
	{ dogfgt_games, DOGFGT_GAME_COUNT, &rbx_drv_dogfgt },
	{ ssozumo_games, SSOZUMO_GAME_COUNT, &rbx_drv_ssozumo },
	{ tagteam_games, TAGTEAM_GAME_COUNT, &rbx_drv_tagteam },
};

static const char *const s_groups[] = { "Double Dragon 3", "WWF Superstars", "Double Dragon", "China Gate", "Shadow Force", "Block Out", "Renegade", "Xain'd Sleena", "Super Dodge Ball", "V'ball", "Mysterious Stones", "Mat Mania", "Battle Lane", "Bogey Manor", "Dog Fight", "Syusse Oozumou", "Tag Team Wrestling" };

static int group(const rbx_game *g)
{
	if (!strcmp(g->board, "Technos WWF Superstars"))
		return 1;
	if (!strcmp(g->board, "Technos Double Dragon"))
		return 2;
	if (!strcmp(g->board, "Technos China Gate"))
		return 3;
	if (!strcmp(g->board, "Technos Shadow Force"))
		return 4;
	if (!strcmp(g->board, "Technos Block Out"))
		return 5;
	if (!strcmp(g->board, "Technos Renegade"))
		return 6;
	if (!strcmp(g->board, "Technos Xain'd Sleena"))
		return 7;
	if (!strcmp(g->board, "Technos Super Dodge Ball"))
		return 8;
	if (!strcmp(g->board, "Technos U.S. Championship V'ball"))
		return 9;
	if (!strcmp(g->board, "Technos Mysterious Stones"))
		return 10;
	if (!strcmp(g->board, "Technos Mat Mania"))
		return 11;
	if (!strcmp(g->board, "Technos Battle Lane"))
		return 12;
	if (!strcmp(g->board, "Technos Bogey Manor"))
		return 13;
	if (!strcmp(g->board, "Technos Dog Fight"))
		return 14;
	if (!strcmp(g->board, "Technos Syusse Oozumou"))
		return 15;
	if (!strcmp(g->board, "Technos Tag Team Wrestling"))
		return 16;
	return 0;
}

static int board_order(const char *board)
{
	(void)board;
	return 0;
}

const rbx_module rbx_mod_technos = {
	"technos", "Technos",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
