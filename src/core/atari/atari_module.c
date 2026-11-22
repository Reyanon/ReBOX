/*  The Atari module (rbx_module.h): the Atari Games 68000 boards (Gauntlet, System 1), their drivers and game tables, and the
    game list's tab.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/gauntlet.inc"
#include "gen/atarisy1.inc"
#include "gen/klax.inc"
#include "gen/rampart.inc"
#include "gen/xybots.inc"
#include "gen/blstroid.inc"
#include "gen/eprom.inc"
#include "gen/skullxbo.inc"
#include "gen/toobin.inc"
#include "gen/cyberbal.inc"

extern const rbx_driver rbx_drv_gauntlet;
extern const rbx_driver rbx_drv_atarisy1;
extern const rbx_driver rbx_drv_klax;
extern const rbx_driver rbx_drv_rampart;
extern const rbx_driver rbx_drv_xybots;
extern const rbx_driver rbx_drv_blstroid;
extern const rbx_driver rbx_drv_eprom;
extern const rbx_driver rbx_drv_skullxbo;
extern const rbx_driver rbx_drv_toobin;
extern const rbx_driver rbx_drv_cyberbal;

static const rbx_board_tab s_boards[] = {
	{ gauntlet_games, GAUNTLET_GAME_COUNT, &rbx_drv_gauntlet },
	{ atarisy1_games, ATARISY1_GAME_COUNT, &rbx_drv_atarisy1 },
	{ klax_games, KLAX_GAME_COUNT, &rbx_drv_klax },
	{ rampart_games, RAMPART_GAME_COUNT, &rbx_drv_rampart },
	{ xybots_games, XYBOTS_GAME_COUNT, &rbx_drv_xybots },
	{ blstroid_games, BLSTROID_GAME_COUNT, &rbx_drv_blstroid },
	{ eprom_games, EPROM_GAME_COUNT, &rbx_drv_eprom },
	{ skullxbo_games, SKULLXBO_GAME_COUNT, &rbx_drv_skullxbo },
	{ toobin_games, TOOBIN_GAME_COUNT, &rbx_drv_toobin },
	{ cyberbal_games, CYBERBAL_GAME_COUNT, &rbx_drv_cyberbal },
};

static const char *const s_groups[] = { "Gauntlet", "System 1", "Klax", "Rampart", "Xybots", "Blasteroids", "Escape", "Skull & Crossbones", "Toobin'", "Cyberball" };
// the board name of each group (s_groups order)
static const char *const s_group_boards[] = { "Atari Gauntlet", "Atari System 1", "Atari Klax", "Atari Rampart", "Atari Xybots", "Atari Blasteroids", "Atari Escape", "Atari Skull & Crossbones", "Atari Toobin'", "Atari Cyberball" };

static int group(const rbx_game *g)
{
	for (int i = 0; i < (int)(sizeof(s_group_boards) / sizeof(s_group_boards[0])); i++)
		if (!strcmp(g->board, s_group_boards[i]))
			return i;
	return 0;
}

static int board_order(const char *board)
{
	(void)board;
	return 0;
}

const rbx_module rbx_mod_atari = {
	"atari", "Atari",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
