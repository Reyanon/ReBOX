/*  Find supported games in ROM folders (shared by the Vita and PSP frontends).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_GAMES_H
#define RBX_GAMES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int game;          // rbx_game_* index
	char zip[160];     // <set>.zip path
	char parent[160];  // parent zip path for clones (empty if not found)
	int present;       // ROM zip exists (if not, zip is empty: greyed out in the list)
	int supported;     // board runs on this platform
} rbx_game_found;

// Fill out with all games in board order (table order), look up "<set>.zip" in dirs (first folder found, names
// case-insensitive). Up to 16 folders. *npresent = games with ROMs
#define RBX_GAMES_MAX 8192   // (1179 sets on 11-03: the list cut the last boards at 1024; 4117 on 11-09 cut at 4096)
int rbx_games_scan(const char *const *dirs, int ndirs, rbx_game_found *out, int max, int *npresent);
// (the tabs: rbx_core.h rbx_game_group)

#ifdef __cplusplus
}
#endif

#endif
