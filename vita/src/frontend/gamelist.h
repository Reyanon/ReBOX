// The game list's data: ux0:data/rebox/gamelist/*.json, one file a tab (maker), shipped beside the VPK in the release zip
// (not in the app, so it can be edited or added to). GPL-2.0-or-later.
#ifndef RBX_FRONTEND_GAMELIST_H
#define RBX_FRONTEND_GAMELIST_H

typedef struct {
	const char *rom, *title, *board, *publisher, *year, *genre;
	int tab;
} rbx_list_entry;

int rbx_gamelist_load(void);   // (once) the entries read; 0: no list, the core's own data is shown
const rbx_list_entry *rbx_gamelist_find(const char *rom);
int rbx_gamelist_tabs(void);
const char *rbx_gamelist_tab(int t);

#endif
