/*  The Seibu module (rbx_module.h): the Seibu Kaihatsu boards, their drivers and game tables, and the game list's tab.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/raiden.inc"
#include "gen/dcon.inc"
#include "gen/bloodbro.inc"
#include "gen/dynduke.inc"
#include "gen/cabal.inc"
#include "gen/deadang.inc"
#include "gen/legionna.inc"
#include "gen/raiden2.inc"
#include "gen/r2dx_v33.inc"
#include "gen/toki.inc"
#include "gen/wiz.inc"
#include "gen/stfight.inc"
#include "gen/kncljoe.inc"
#include "gen/mustache.inc"
#include "gen/darkmist.inc"
#include "gen/goal92.inc"
#include "gen/goodejan.inc"
#include "gen/sengokmj.inc"
#include "gen/panicr.inc"
#include "gen/airraid.inc"
#include "gen/metlfrzr.inc"
#include "gen/feversoc.inc"
#include "gen/seicupbl.inc"
#include "gen/seibuspi.inc"

extern const rbx_driver rbx_drv_raiden;
extern const rbx_driver rbx_drv_dcon;
extern const rbx_driver rbx_drv_bloodbro;
extern const rbx_driver rbx_drv_dynduke;
extern const rbx_driver rbx_drv_cabal;
extern const rbx_driver rbx_drv_deadang;
extern const rbx_driver rbx_drv_legionna;
extern const rbx_driver rbx_drv_raiden2;
extern const rbx_driver rbx_drv_r2dx_v33;
extern const rbx_driver rbx_drv_toki;
extern const rbx_driver rbx_drv_wiz;
extern const rbx_driver rbx_drv_stfight;
extern const rbx_driver rbx_drv_kncljoe;
extern const rbx_driver rbx_drv_mustache;
extern const rbx_driver rbx_drv_darkmist;
extern const rbx_driver rbx_drv_goal92;
extern const rbx_driver rbx_drv_goodejan;
extern const rbx_driver rbx_drv_sengokmj;
extern const rbx_driver rbx_drv_panicr;
extern const rbx_driver rbx_drv_airraid;
extern const rbx_driver rbx_drv_metlfrzr;
extern const rbx_driver rbx_drv_feversoc;
extern const rbx_driver rbx_drv_seicupbl;
extern const rbx_driver rbx_drv_seibuspi;

static const rbx_board_tab s_boards[] = {
	{ raiden_games, RAIDEN_GAME_COUNT, &rbx_drv_raiden },
	{ dcon_games, DCON_GAME_COUNT, &rbx_drv_dcon },
	{ bloodbro_games, BLOODBRO_GAME_COUNT, &rbx_drv_bloodbro },
	{ dynduke_games, DYNDUKE_GAME_COUNT, &rbx_drv_dynduke },
	{ cabal_games, CABAL_GAME_COUNT, &rbx_drv_cabal },
	{ deadang_games, DEADANG_GAME_COUNT, &rbx_drv_deadang },
	{ legionna_games, LEGIONNA_GAME_COUNT, &rbx_drv_legionna },
	{ raiden2_games, RAIDEN2_GAME_COUNT, &rbx_drv_raiden2 },
	{ r2dx_v33_games, R2DX_V33_GAME_COUNT, &rbx_drv_r2dx_v33 },
	{ toki_games, TOKI_GAME_COUNT, &rbx_drv_toki },
	{ wiz_games, WIZ_GAME_COUNT, &rbx_drv_wiz },
	{ stfight_games, STFIGHT_GAME_COUNT, &rbx_drv_stfight },
	{ kncljoe_games, KNCLJOE_GAME_COUNT, &rbx_drv_kncljoe },
	{ mustache_games, MUSTACHE_GAME_COUNT, &rbx_drv_mustache },
	{ darkmist_games, DARKMIST_GAME_COUNT, &rbx_drv_darkmist },
	{ goal92_games, GOAL92_GAME_COUNT, &rbx_drv_goal92 },
	{ goodejan_games, GOODEJAN_GAME_COUNT, &rbx_drv_goodejan },
	{ sengokmj_games, SENGOKMJ_GAME_COUNT, &rbx_drv_sengokmj },
	{ panicr_games, PANICR_GAME_COUNT, &rbx_drv_panicr },
	{ airraid_games, AIRRAID_GAME_COUNT, &rbx_drv_airraid },
	{ metlfrzr_games, METLFRZR_GAME_COUNT, &rbx_drv_metlfrzr },
	{ feversoc_games, FEVERSOC_GAME_COUNT, &rbx_drv_feversoc },
	{ seicupbl_games, SEICUPBL_GAME_COUNT, &rbx_drv_seicupbl },
	{ seibuspi_games, SEIBUSPI_GAME_COUNT, &rbx_drv_seibuspi },
};

// (SPI; the COP boards; the other 16-bit and 8-bit boards, which the game list shows by board or as Classic/Standalone)
static const char *const s_groups[] = { "SPI", "COP", "16-bit", "8-bit" };

static int group(const rbx_game *g)
{
	static const char *const cop[] = { "Seibu Legionnaire", "Seibu Raiden II", "Seibu Raiden II V33", "Seibu Cup Soccer Bootleg" };
	static const char *const bit8[] = { "Seibu Air Raid", "Seibu Darkmist", "Seibu Knuckle Joe", "Seibu Metal Freezer",
	                                    "Seibu Mustache Boy", "Seibu Panic Road", "Seibu Street Fight", "Seibu Wiz" };
	if (!strcmp(g->board, "Seibu SPI"))
		return 0;
	for (unsigned i = 0; i < sizeof(cop) / sizeof(cop[0]); i++)
		if (!strcmp(g->board, cop[i]))
			return 1;
	for (unsigned i = 0; i < sizeof(bit8) / sizeof(bit8[0]); i++)
		if (!strcmp(g->board, bit8[i]))
			return 3;
	return 2;
}

static int board_order(const char *board)
{
	(void)board;
	return 0;
}

const rbx_module rbx_mod_seibu = {
	"seibu", "Seibu",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
