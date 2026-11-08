// The game list's data, gamelist/*.json. See gamelist.h. GPL-2.0-or-later.
//   { "name": "Konami", "order": 3, "games": [ { "rom": "gyruss", "title": "Gyruss", "board": "Gyruss", "publisher": "Konami",
//     "year": "1983", "genre": "Shooter" }, ... ] }
// A flat reader: strings in place, an object in "games" is an entry; other keys are skipped, so a list may carry more.
#include "frontend/gamelist.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <psp2/io/dirent.h>
#include <psp2/io/stat.h>

#include "frontend/settings.h"
#include "platform/log.h"

#define MAX_TABS 16
#define MAX_ENTRIES 6000
#define USER_DIR RBX_DATA_DIR "gamelist/"

static rbx_list_entry *s_e;
static int s_n;
static struct { const char *name; int order; } s_tab[MAX_TABS];
static int s_ntab;

static char *read_all(const char *path)
{
	FILE *f = fopen(path, "rb");
	if (!f)
		return NULL;
	fseek(f, 0, SEEK_END);
	const long n = ftell(f);
	fseek(f, 0, SEEK_SET);
	char *b = n > 0 ? (char *)malloc((size_t)n + 1) : NULL;
	if (b) {
		if (fread(b, 1, (size_t)n, f) == (size_t)n)
			b[n] = 0;
		else {
			free(b);
			b = NULL;
		}
	}
	fclose(f);
	return b;
}

// the string starting after the " at p, closed and unescaped in place; returns what follows it
static char *json_str(char *p)
{
	char *w = p;
	while (*p && *p != '"') {
		if (*p == '\\' && p[1])
			p++;
		*w++ = *p++;
	}
	const int more = *p != 0;
	*w = 0;
	return more ? p + 1 : p;
}

static void parse(char *p)
{
	const int tab = s_ntab;
	const char *name = NULL;
	int order = 1000, in_games = 0;
	rbx_list_entry e;
	memset(&e, 0, sizeof(e));
	for (; *p; p++) {
		if (*p == '{' && in_games)
			memset(&e, 0, sizeof(e));
		else if (*p == '}' && in_games && e.rom && s_n < MAX_ENTRIES) {
			e.tab = tab;
			s_e[s_n++] = e;
			e.rom = NULL;
		} else if (*p == '"') {
			const char *k = p + 1;
			p = json_str(p + 1);
			while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
				p++;
			if (*p != ':') {
				p--;
				continue;
			}
			p++;
			while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
				p++;
			if (!strcmp(k, "games")) {
				in_games = 1;
				p--;
			} else if (*p == '"') {
				const char *v = p + 1;
				p = json_str(p + 1) - 1;
				if (!in_games) {
					if (!strcmp(k, "name")) name = v;
				} else if (!strcmp(k, "rom")) e.rom = v;
				else if (!strcmp(k, "title")) e.title = v;
				else if (!strcmp(k, "board")) e.board = v;
				else if (!strcmp(k, "publisher")) e.publisher = v;
				else if (!strcmp(k, "year")) e.year = v;
				else if (!strcmp(k, "genre")) e.genre = v;
			} else {
				if (!in_games && !strcmp(k, "order"))
					order = atoi(p);
				p--;
			}
		}
	}
	if (name && s_ntab < MAX_TABS) {
		s_tab[s_ntab].name = name;
		s_tab[s_ntab++].order = order;
	}
}

static int cmp_entry(const void *a, const void *b)
{
	return strcmp(((const rbx_list_entry *)a)->rom, ((const rbx_list_entry *)b)->rom);
}

int rbx_gamelist_load(void)
{
	if (s_e)
		return s_n;
	s_e = (rbx_list_entry *)malloc(sizeof(rbx_list_entry) * MAX_ENTRIES);
	if (!s_e)
		return 0;
	// the lists are files beside the app (the release zip's data/rebox/gamelist/), not in it: the user may edit or add them
	{
		const char *base = USER_DIR;
		const SceUID d = sceIoDopen(base);
		SceIoDirent de;
		for (int more = d >= 0; more && s_ntab < MAX_TABS;) {
			memset(&de, 0, sizeof(de));
			more = sceIoDread(d, &de) > 0;
			const size_t l = strlen(de.d_name);
			if (!more || l < 6 || strcmp(de.d_name + l - 5, ".json"))
				continue;
			char path[128];
			snprintf(path, sizeof(path), "%s%s", base, de.d_name);
			char *b = read_all(path);
			if (b)
				parse(b);   // (kept: the entries point into it)
		}
		if (d >= 0)
			sceIoDclose(d);
	}
	// the tabs by their order; the entries' numbers follow
	int map[MAX_TABS];
	for (int i = 0; i < s_ntab; i++)
		map[i] = i;
	for (int a = 1; a < s_ntab; a++)
		for (int b = a; b > 0 && s_tab[map[b]].order < s_tab[map[b - 1]].order; b--) {
			const int k = map[b];
			map[b] = map[b - 1];
			map[b - 1] = k;
		}
	int pos[MAX_TABS];
	__typeof__(s_tab) sorted;
	for (int i = 0; i < s_ntab; i++) {
		pos[map[i]] = i;
		sorted[i] = s_tab[map[i]];
	}
	memcpy(s_tab, sorted, sizeof(s_tab));
	for (int i = 0; i < s_n; i++)
		s_e[i].tab = pos[s_e[i].tab];
	qsort(s_e, (size_t)s_n, sizeof(s_e[0]), cmp_entry);
	RBX_LOG("gamelist: %d games, %d tabs", s_n, s_ntab);
	return s_n;
}

const rbx_list_entry *rbx_gamelist_find(const char *rom)
{
	rbx_list_entry k;
	memset(&k, 0, sizeof(k));
	k.rom = rom;
	return s_n ? (const rbx_list_entry *)bsearch(&k, s_e, (size_t)s_n, sizeof(s_e[0]), cmp_entry) : NULL;
}

int rbx_gamelist_tabs(void) { return s_ntab; }
const char *rbx_gamelist_tab(int t) { return t >= 0 && t < s_ntab ? s_tab[t].name : ""; }
