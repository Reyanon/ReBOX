// The game list's data: ux0:data/rebox/gamelist/*.json. A file is a tab (maker); files of the same tab name are one tab.
// The base lists (<maker>.json) and, from each release on, the games it added (v<version>/<maker>.json), so a new
// version's games reach a list folder whose files the user may have edited. Shipped in the VPK (copied when missing)
// and beside it in the release zip. GPL-2.0-or-later.
#ifndef RBX_FRONTEND_GAMELIST_H
#define RBX_FRONTEND_GAMELIST_H

// An entry: { "rompath": "/roms/shuuz.zip", "title", "board", "publisher", "year", "genre", "marquee": "/marquee/shuuz.png",
// "snap": "/snap/shuuz.png", "video": "/video/shuuz.mp4", "description": null }. A path that starts with "/" is in the
// data folder (ux0:data/rebox/), one with a device ("ux0:...") is used as it is. The set is the ROM file's name without
// ".zip" (older lists: "rom": "<set>").
typedef struct {
	const char *rom, *title, *board, *publisher, *year, *genre;
	const char *rompath, *marquee, *snap, *video, *description;   // (NULL: not given)
	const char *port;   // a game of a board not ported yet: its MAME driver (shown greyed, not started), else NULL
	int tab;
	int seq;   // (load order: of two entries of the same set the ported one is kept, then the earlier one)
} rbx_list_entry;

int rbx_gamelist_load(void);   // (once) the entries read; 0: no list, the core's own data is shown
const rbx_list_entry *rbx_gamelist_find(const char *rom);
int rbx_gamelist_tabs(void);
int rbx_gamelist_count(void);
const rbx_list_entry *rbx_gamelist_at(int i);
const char *rbx_gamelist_tab(int t);
// a list path made a device path: "/x" -> ux0:data/rebox/x; out NULL or "" when p is NULL
const char *rbx_gamelist_path(const char *p, char *out, int size);
// a set's marquee (kind 0), snap (1) or video (2): the list's path, else marquee/<set>.png, snap/<set>.png,
// video/<set>.mp4 in the data folder
const char *rbx_gamelist_media(const char *set, int kind, char *out, int size);
// the folders of the entries' ROM files that are not one of dirs (at most max, each ROM_DIR-like with a trailing "/")
int rbx_gamelist_rom_dirs(const char *const *dirs, int ndirs, const char **out, int max);

#endif
