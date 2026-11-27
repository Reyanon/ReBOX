/*  The Capcom module (rbx_module.h): Capcom's boards (CPS1, CPS2, CPS3), their drivers and game tables, and the game list's tabs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/cps1.inc"
#include "gen/cps2.inc"
#include "cps2_wide.inc"
#include "gen/cps3.inc"
#include "gen/lastduel.inc"
#include "gen/bionicc.inc"
#include "gen/tigeroad.inc"
#include "gen/sf.inc"
#include "gen/c1942.inc"
#include "gen/c1943.inc"
#include "gen/commando.inc"
#include "gen/gng.inc"
#include "gen/lwings.inc"
#include "gen/mitchell.inc"
#include "gen/higemaru.inc"
#include "gen/vulgus.inc"
#include "gen/exedexes.inc"
#include "gen/gunsmoke.inc"
#include "gen/sonson.inc"
#include "gen/cbasebal.inc"
#include "gen/blktiger.inc"
#include "gen/srumbler.inc"
#include "gen/sidearms.inc"

extern const rbx_driver rbx_drv_cps1;
extern const rbx_driver rbx_drv_cps3;
extern const rbx_driver rbx_drv_lastduel;
extern const rbx_driver rbx_drv_bionicc;
extern const rbx_driver rbx_drv_tigeroad;
extern const rbx_driver rbx_drv_sf;
extern const rbx_driver rbx_drv_c1942;
extern const rbx_driver rbx_drv_c1943;
extern const rbx_driver rbx_drv_commando;
extern const rbx_driver rbx_drv_gng;
extern const rbx_driver rbx_drv_lwings;
extern const rbx_driver rbx_drv_mitchell;
extern const rbx_driver rbx_drv_higemaru;
extern const rbx_driver rbx_drv_vulgus;
extern const rbx_driver rbx_drv_exedexes;

extern const rbx_driver rbx_drv_gunsmoke;
extern const rbx_driver rbx_drv_sonson;
extern const rbx_driver rbx_drv_cbasebal;
extern const rbx_driver rbx_drv_blktiger;
extern const rbx_driver rbx_drv_srumbler;
extern const rbx_driver rbx_drv_sidearms;
static const rbx_board_tab s_boards[] = {
	{ cps1_games, CPS1_GAME_COUNT, &rbx_drv_cps1 },
	{ cps2_games, CPS2_GAME_COUNT, &rbx_drv_cps1 },   // (one driver: CPS2 shares the CPS-A / CPS-B video)
	{ cps2_wide_games, CPS2_WIDE_GAME_COUNT, &rbx_drv_cps1 },   // (CPS-2 WIDE v1: Vampire Saved)
	{ cps3_games, CPS3_GAME_COUNT, &rbx_drv_cps3 },
	{ lastduel_games, LASTDUEL_GAME_COUNT, &rbx_drv_lastduel },
	{ bionicc_games, BIONICC_GAME_COUNT, &rbx_drv_bionicc },
	{ tigeroad_games, TIGEROAD_GAME_COUNT, &rbx_drv_tigeroad },
	{ sf_games, SF_GAME_COUNT, &rbx_drv_sf },
	{ c1942_games, C1942_GAME_COUNT, &rbx_drv_c1942 },
	{ c1943_games, C1943_GAME_COUNT, &rbx_drv_c1943 },
	{ commando_games, COMMANDO_GAME_COUNT, &rbx_drv_commando },
	{ gng_games, GNG_GAME_COUNT, &rbx_drv_gng },
	{ lwings_games, LWINGS_GAME_COUNT, &rbx_drv_lwings },
	{ mitchell_games, MITCHELL_GAME_COUNT, &rbx_drv_mitchell },
	{ higemaru_games, HIGEMARU_GAME_COUNT, &rbx_drv_higemaru },
	{ vulgus_games, VULGUS_GAME_COUNT, &rbx_drv_vulgus },
	{ exedexes_games, EXEDEXES_GAME_COUNT, &rbx_drv_exedexes },
	{ gunsmoke_games, GUNSMOKE_GAME_COUNT, &rbx_drv_gunsmoke },
	{ sonson_games, SONSON_GAME_COUNT, &rbx_drv_sonson },
	{ cbasebal_games, CBASEBAL_GAME_COUNT, &rbx_drv_cbasebal },
	{ blktiger_games, BLKTIGER_GAME_COUNT, &rbx_drv_blktiger },
	{ srumbler_games, SRUMBLER_GAME_COUNT, &rbx_drv_srumbler },
	{ sidearms_games, SIDEARMS_GAME_COUNT, &rbx_drv_sidearms },
};

// (the CPS boards each a group; the earlier boards, one driver each, together as Classic)
static const char *const s_groups[] = { "CPS1", "CPS2", "CPS3", "Classic" };

static int group(const rbx_game *g)
{
	if (!strcmp(g->board, "Capcom CPS1"))
		return 0;
	if (!strcmp(g->board, "Capcom CPS2"))
		return 1;
	if (!strcmp(g->board, "Capcom CPS3"))
		return 2;
	return 3;
}

static int board_order(const char *board)
{
	return !strcmp(board, "Capcom CPS2") ? 1 : 0;
}

const rbx_module rbx_mod_capcom = {
	"capcom", "Capcom",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
