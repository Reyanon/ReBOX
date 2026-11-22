/*  Find supported games in ROM folders (rbx_games.h). Each folder's file list is read once and sorted
    (trying fopen per game takes seconds on a memory card).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "rbx_games.h"
#include "rbx_core.h"
#include "rbx.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>

#define NAME_LEN 24
#define MAX_NAMES 8192
#define MAX_DIRS 16

typedef struct {
	char (*names)[NAME_LEN];
	int count;
} dir_list;

static int cmp_name(const void *a, const void *b) { return strcmp((const char *)a, (const char *)b); }

static void read_dir(const char *path, dir_list *out)
{
	out->names = NULL;
	out->count = 0;
	struct stat st;   // (stat first: some missing devices pass opendir and hang in readdir)
	if (stat(path, &st) != 0 || !S_ISDIR(st.st_mode))
		return;
	DIR *d = opendir(path);
	if (!d)
		return;
	int cap = 0;
	struct dirent *e;
	while ((e = readdir(d)) != NULL) {
		const char *n = e->d_name;
		const size_t len = strlen(n);
		if (len < 5 || len - 4 >= NAME_LEN)
			continue;
		const char *ext = n + len - 4;
		if (!(ext[0] == '.' && (ext[1] | 0x20) == 'z' && (ext[2] | 0x20) == 'i' && (ext[3] | 0x20) == 'p'))
			continue;
		if (out->count == cap) {
			if (cap >= MAX_NAMES)
				break;
			cap = cap ? cap * 2 : 64;
			void *p = realloc(out->names, (size_t)cap * NAME_LEN);
			if (!p)
				break;
			out->names = (char (*)[NAME_LEN])p;
		}
		char *dst = out->names[out->count++];
		for (size_t i = 0; i < len - 4; i++)
			dst[i] = (n[i] >= 'A' && n[i] <= 'Z') ? (char)(n[i] | 0x20) : n[i];
		dst[len - 4] = 0;
	}
	closedir(d);
	if (out->count)
		qsort(out->names, out->count, NAME_LEN, cmp_name);
}

static int has(const dir_list *l, const char *set)
{
	char key[NAME_LEN];
	snprintf(key, sizeof(key), "%s", set);
	return l->count && bsearch(key, l->names, l->count, NAME_LEN, cmp_name) != NULL;
}

static int find_zip(const char *const *dirs, const dir_list *lists, int ndirs, int first, const char *set, char *out, int n)
{
	for (int k = 0; k < ndirs; k++) {
		const int i = (first + k) % ndirs;
		if (has(&lists[i], set)) {
			snprintf(out, n, "%s%s.zip", dirs[i], set);
			return i;
		}
	}
	out[0] = 0;
	return -1;
}

// the tab of a game in the list, its order: the modules' groups (core.c rbx_game_group_order)
static int tab_group(int g) { return rbx_game_group_order(g); }

static int cmp_found(const void *a, const void *b)
{
	const rbx_game_found *x = (const rbx_game_found *)a, *y = (const rbx_game_found *)b;
	int bx = tab_group(x->game), by = tab_group(y->game);
	if (bx != by)
		return bx < by ? -1 : 1;
	// within a tab: by game title, then table order
	int t = strcasecmp(rbx_game_title(x->game), rbx_game_title(y->game));
	return t ? t : x->game - y->game;
}

int rbx_games_scan(const char *const *dirs, int ndirs, rbx_game_found *out, int max, int *npresent)
{
	dir_list lists[MAX_DIRS];
	if (ndirs > MAX_DIRS)
		ndirs = MAX_DIRS;
	for (int i = 0; i < ndirs; i++)
		read_dir(dirs[i], &lists[i]);
	int n = 0, present = 0;
	for (int g = 0; g < rbx_game_count() && n < max; g++) {
		// (a game of a board not emulated (yet) is listed too, unplayable: the list shows every game of the modules)
		rbx_game_found *f = &out[n++];
		const int d = find_zip(dirs, lists, ndirs, 0, rbx_game_set(g), f->zip, sizeof(f->zip));
		f->game = g;
		f->parent[0] = 0;
		f->present = d >= 0;
		f->supported = !(rbx_game_flags(g) & RBX_GF_UNSUPPORTED);
		if (d < 0)
			continue;
		present++;
		const char *p = rbx_game_parent(g);
		if (p)
			find_zip(dirs, lists, ndirs, d, p, f->parent, sizeof(f->parent));
	}
	if (n < rbx_game_count())   // (the table outgrew max: the last modules' games left out)
		rbx_log("games: %d of %d listed (RBX_GAMES_MAX)", n, rbx_game_count());
	for (int i = 0; i < ndirs; i++)
		free(lists[i].names);
	qsort(out, (size_t)n, sizeof(out[0]), cmp_found);
	if (npresent)
		*npresent = present;
	return n;
}
