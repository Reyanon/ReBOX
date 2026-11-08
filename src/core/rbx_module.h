/*  A core module: one maker's boards (src/core/<id>/: drivers, their chips, the game tables made by
    tools/gen_games.py) and how the game list groups their games. The shared parts (CPUs, sound chips,
    common devices, ROM loading, input, video and sound services) stay in src/core.
    A module is built in with RBX_MOD_<ID> (src/core/<id>/<id>.cmake) and listed in core.c s_modules.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_MODULE_H
#define RBX_MODULE_H
#include "rbx.h"

// a board: its driver and its games (a gen/<board>.inc table)
typedef struct {
	const rbx_game *games;
	int count;
	const rbx_driver *drv;
} rbx_board_tab;

typedef struct {
	const char *id;                    // "taito": the folder, the frontend's core switch
	const char *name;                  // "Taito"
	const rbx_board_tab *boards;
	int nboards;
	const char *const *groups;         // the game list's tabs, in their order
	int ngroups;
	int (*group)(const rbx_game *g);   // the game's tab: index in groups
	int (*board_order)(const char *board);   // the board's place among the module's boards (unknown: 1000 or more)
} rbx_module;

#endif
