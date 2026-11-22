// The game list's data, gamelist/*.json. See gamelist.h. GPL-2.0-or-later.
//   { "name": "Konami", "order": 3, "games": [ { "rom": "gyruss", "title": "Gyruss", "board": "Gyruss", "publisher": "Konami",
//     "year": "1983", "genre": "Shooter" }, ... ] }
// A flat reader: strings in place, an object in "games" is an entry; other keys are skipped, so a list may carry more.
#include "frontend/gamelist.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <psp2/io/dirent.h>
#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>

#include "frontend/settings.h"
#include "platform/log.h"

#define MAX_TABS 64     // (the makers: one file each; 22 in 11-12)
#define MAX_ENTRIES 40000   // (the boards not ported yet too: MAME's other games, 11-20)
#define USER_DIR RBX_DATA_DIR "gamelist/"
#define APP_DIR "app0:gamelist/"     // (the lists shipped in the VPK)

static int list_dir(const char *dir, char (*names)[64], int max, int want_dirs);

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
		if (*p == '\\' && p[1]) {   // (JSON escapes: \n, \t, \uXXXX as UTF-8; any other letter as itself)
			p++;
			const char k = *p;
			if (k == 'n') { *w++ = '\n'; p++; continue; }
			if (k == 't') { *w++ = '\t'; p++; continue; }
			if (k == 'r') { p++; continue; }
			if (k == 'u' && p[1] && p[2] && p[3] && p[4]) {
				char hex[5] = { p[1], p[2], p[3], p[4], 0 };
				unsigned u = (unsigned)strtoul(hex, NULL, 16);
				p += 5;
				if (u >= 0xD800 && u < 0xDC00 && p[0] == '\\' && p[1] == 'u' && p[2] && p[3] && p[4] && p[5]) {   // (a pair)
					char h2[5] = { p[2], p[3], p[4], p[5], 0 };
					const unsigned lo = (unsigned)strtoul(h2, NULL, 16);
					if (lo >= 0xDC00 && lo < 0xE000) {
						u = 0x10000 + ((u - 0xD800) << 10) + (lo - 0xDC00);
						p += 6;
					}
				}
				// (the UTF-8 is never longer than the escape it replaces: written in place)
				if (u < 0x80)
					*w++ = (char)u;
				else if (u < 0x800) {
					*w++ = (char)(0xC0 | (u >> 6));
					*w++ = (char)(0x80 | (u & 0x3F));
				} else if (u < 0x10000) {
					*w++ = (char)(0xE0 | (u >> 12));
					*w++ = (char)(0x80 | ((u >> 6) & 0x3F));
					*w++ = (char)(0x80 | (u & 0x3F));
				} else {
					*w++ = (char)(0xF0 | (u >> 18));
					*w++ = (char)(0x80 | ((u >> 12) & 0x3F));
					*w++ = (char)(0x80 | ((u >> 6) & 0x3F));
					*w++ = (char)(0x80 | (u & 0x3F));
				}
				continue;
			}
		}
		*w++ = *p++;
	}
	const int more = *p != 0;
	*w = 0;
	return more ? p + 1 : p;
}

// the set of a ROM path: its file name without ".zip" (a copy)
static const char *set_of(const char *path)
{
	const char *b = path;
	for (const char *q = path; *q; q++)
		if (*q == '/' || *q == ':')
			b = q + 1;
	size_t l = strlen(b);
	if (l > 4 && !strcasecmp(b + l - 4, ".zip"))
		l -= 4;
	char *r = (char *)malloc(l + 1);
	if (r) {
		memcpy(r, b, l);
		r[l] = 0;
	}
	return r;
}

static void parse(char *p)
{
	const int tab = s_ntab, first = s_n;
	const char *name = NULL;
	int order = 1000, in_games = 0;
	rbx_list_entry e;
	memset(&e, 0, sizeof(e));
	for (; *p; p++) {
		if (*p == '{' && in_games)
			memset(&e, 0, sizeof(e));
		else if (*p == '}' && in_games && !e.rom && e.rompath)
			e.rom = set_of(e.rompath);
		if (*p == '}' && in_games && e.rom && s_n < MAX_ENTRIES) {
			e.tab = tab;
#ifndef RBX_ENABLE_LOG
			if (e.port && e.board) {   // (release: a board not ported shown as "galaga", not "galaga.cpp"; user 11-20)
				char *bd = (char *)e.board;
				const size_t bl = strlen(bd);
				if (bl > 4 && !strcmp(bd + bl - 4, ".cpp"))
					bd[bl - 4] = 0;
			}
#endif
			e.seq = s_n;
			s_e[s_n++] = e;
			memset(&e, 0, sizeof(e));   // (the closing "}" of the file must not add it again)
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
				else if (!strcmp(k, "rompath")) e.rompath = v;
				else if (!strcmp(k, "marquee")) e.marquee = v;
				else if (!strcmp(k, "snap")) e.snap = v;
				else if (!strcmp(k, "video")) e.video = v;
				else if (!strcmp(k, "description")) e.description = v;
				else if (!strcmp(k, "title")) e.title = v;
				else if (!strcmp(k, "board")) e.board = v;
				else if (!strcmp(k, "publisher")) e.publisher = v;
				else if (!strcmp(k, "year")) e.year = v;
				else if (!strcmp(k, "genre")) e.genre = v;
				else if (!strcmp(k, "port")) e.port = v;
			} else {
				if (!in_games && !strcmp(k, "order"))
					order = atoi(p);
				p--;
			}
		}
	}
	if (!name)
		return;
	for (int t = 0; t < s_ntab; t++)   // (a tab already read: a version's file adds to it)
		if (!strcasecmp(s_tab[t].name, name)) {
			for (int i = first; i < s_n; i++)
				s_e[i].tab = t;
			return;
		}
	if (s_ntab < MAX_TABS) {
		s_tab[s_ntab].name = name;
		s_tab[s_ntab++].order = order;
	}
}

static int cmp_version(const char *a, const char *b)
{
	while (*a && *b) {
		if (*a >= '0' && *a <= '9' && *b >= '0' && *b <= '9') {
			long x = strtol(a, (char **)&a, 10), y = strtol(b, (char **)&b, 10);
			if (x != y)
				return x < y ? -1 : 1;
		} else {
			if (*a != *b)
				return (unsigned char)*a < (unsigned char)*b ? -1 : 1;
			a++, b++;
		}
	}
	return *a ? 1 : *b ? -1 : 0;
}

static int cmp_name(const void *a, const void *b) { return strcasecmp((const char *)a, (const char *)b); }
static int cmp_dir(const void *a, const void *b) { return cmp_version((const char *)a, (const char *)b); }

static void read_dir(const char *sub)
{
	static char names[128][64];
	char dir[160];
	snprintf(dir, sizeof(dir), "%s%s", USER_DIR, sub);
	const int nn = list_dir(dir, names, 128, 0);
	qsort(names, (size_t)nn, sizeof(names[0]), cmp_name);
	for (int i = 0; i < nn; i++) {
		char path[256];
		snprintf(path, sizeof(path), "%s%s", dir, names[i]);
		char *b = read_all(path);
		if (b)
			parse(b);   // (kept: the entries point into it)
	}
}

static int cmp_entry(const void *a, const void *b)
{
	return strcmp(((const rbx_list_entry *)a)->rom, ((const rbx_list_entry *)b)->rom);
}

// the sort before the duplicates are dropped: by set, then the ported entry, then the earlier file's
static int cmp_entry_keep(const void *a, const void *b)
{
	const rbx_list_entry *x = (const rbx_list_entry *)a, *y = (const rbx_list_entry *)b;
	const int c = strcmp(x->rom, y->rom);
	if (c)
		return c;
	if (!x->port != !y->port)
		return x->port ? 1 : -1;
	return x->seq - y->seq;
}

static int is_json(const char *n)
{
	const size_t l = strlen(n);
	return l >= 6 && !strcmp(n + l - 5, ".json");
}

// the .json files, or the folders, of a folder (names, at most max). Every folder of gamelist/ is read (user 11-22: the
// releases' "v<version>" folders, and any other the user makes)
static int list_dir(const char *dir, char (*names)[64], int max, int want_dirs)
{
	int n = 0;
	const SceUID d = sceIoDopen(dir);
	if (d < 0)
		return 0;
	SceIoDirent de;
	while (n < max) {
		memset(&de, 0, sizeof(de));
		if (sceIoDread(d, &de) <= 0)
			break;
		const int isdir = SCE_S_ISDIR(de.d_stat.st_mode);
		if (strlen(de.d_name) >= 64 || de.d_name[0] == '.' || (want_dirs ? !isdir : (isdir || !is_json(de.d_name))))
			continue;
		strcpy(names[n++], de.d_name);
	}
	sceIoDclose(d);
	return n;
}

// the lists of one folder of the VPK copied to the same folder of the data folder when not there
static int install_dir(const char *sub)
{
	static char names[128][64];
	char src_dir[160], dst_dir[160];
	snprintf(src_dir, sizeof(src_dir), "%s%s", APP_DIR, sub);
	snprintf(dst_dir, sizeof(dst_dir), "%s%s", USER_DIR, sub);
	const int nn = list_dir(src_dir, names, 128, 0);
	if (nn)
		sceIoMkdir(dst_dir, 0777);
	int n = 0;
	for (int i = 0; i < nn; i++) {
		char src[256], dst[256];
		snprintf(src, sizeof(src), "%s%s", src_dir, names[i]);
		snprintf(dst, sizeof(dst), "%s%s", dst_dir, names[i]);
		SceIoStat st;
		if (sceIoGetstat(dst, &st) >= 0)
			continue;
		char *b = read_all(src);
		if (!b)
			continue;
		const SceUID f = sceIoOpen(dst, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
		if (f >= 0) {
			const int len = (int)strlen(b);
			const int ok = sceIoWrite(f, b, (SceSize)len) == len;
			sceIoClose(f);
			if (!ok)
				sceIoRemove(dst);   // (a part file would be read as a broken list: tried again next start)
			else
				n++;
		}
		free(b);
	}
	return n;
}

// the lists of the VPK (app0:gamelist/ and its version folders) copied to the data folder when a file of that name is
// not there (user 11-22: a VPK installed without the release zip's data folder still gets them; a list already there,
// maybe edited, is never overwritten)
static void install_missing(void)
{
	static char dirs[64][64];
	sceIoMkdir(RBX_DATA_DIR, 0777);
	sceIoMkdir(USER_DIR, 0777);
	int n = install_dir("");
	const int nd = list_dir(APP_DIR, dirs, 64, 1);
	for (int i = 0; i < nd; i++) {
		char sub[80];
		snprintf(sub, sizeof(sub), "%s/", dirs[i]);
		n += install_dir(sub);
	}
	if (n)
		RBX_LOG("gamelist: %d lists installed from the app", n);
}

int rbx_gamelist_load(void)
{
	if (s_e)
		return s_n;
	s_e = (rbx_list_entry *)malloc(sizeof(rbx_list_entry) * MAX_ENTRIES);
	if (!s_e)
		return 0;
	// the lists are files beside the app (the release zip's data/rebox/gamelist/): the user may edit or add them. The
	// VPK's copies fill in the missing ones
	install_missing();
	{	// gamelist/*.json first, then every folder's *.json, the folders in version order ("v0.13.95" < "v0.13.100"; which
		// entry is kept does not depend on the directory's order)
		static char dirs[64][64];
		read_dir("");
		const int nd = list_dir(USER_DIR, dirs, 64, 1);
		qsort(dirs, (size_t)nd, sizeof(dirs[0]), cmp_dir);
		for (int i = 0; i < nd; i++) {
			char sub[80];
			snprintf(sub, sizeof(sub), "%s/", dirs[i]);
			read_dir(sub);
		}
	}
	// the tabs by name (user 11-12; the files' "order" no longer used); the entries' numbers follow
	int map[MAX_TABS];
	for (int i = 0; i < s_ntab; i++)
		map[i] = i;
	for (int a = 1; a < s_ntab; a++)
		for (int b = a; b > 0 && strcasecmp(s_tab[map[b]].name, s_tab[map[b - 1]].name) < 0; b--) {
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
	qsort(s_e, (size_t)s_n, sizeof(s_e[0]), cmp_entry_keep);
	int m = 0;
	for (int i = 0; i < s_n; i++)   // (one entry a set: the first after the sort)
		if (!m || strcmp(s_e[i].rom, s_e[m - 1].rom))
			s_e[m++] = s_e[i];
	const int dup = s_n - m;
	s_n = m;
	RBX_LOG("gamelist: %d games, %d tabs (%d duplicates dropped)", s_n, s_ntab, dup);
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
int rbx_gamelist_count(void) { return s_n; }
const rbx_list_entry *rbx_gamelist_at(int i) { return i >= 0 && i < s_n ? &s_e[i] : NULL; }
const char *rbx_gamelist_tab(int t) { return t >= 0 && t < s_ntab ? s_tab[t].name : ""; }

const char *rbx_gamelist_path(const char *p, char *out, int size)
{
	if (!p || !p[0]) {
		out[0] = 0;
		return NULL;
	}
	if (strchr(p, ':'))
		snprintf(out, (size_t)size, "%s", p);
	else
		snprintf(out, (size_t)size, RBX_DATA_DIR "%s", p[0] == '/' ? p + 1 : p);
	return out;
}

int rbx_gamelist_rom_dirs(const char *const *dirs, int ndirs, const char **out, int max)
{
	static char held[16][128];
	int n = 0;
	for (int i = 0; i < s_n && n < max && n < 16; i++) {
		if (!s_e[i].rompath || s_e[i].port)
			continue;
		char d[128];
		rbx_gamelist_path(s_e[i].rompath, d, sizeof(d));
		char *sl = strrchr(d, '/');
		if (!sl)
			continue;
		sl[1] = 0;
		int known = 0;
		for (int k = 0; k < ndirs && !known; k++)
			known = !strcasecmp(dirs[k], d);
		for (int k = 0; k < n && !known; k++)
			known = !strcasecmp(out[k], d);
		if (known)
			continue;
		memcpy(held[n], d, strlen(d) + 1);
		out[n] = held[n];
		n++;
	}
	return n;
}

const char *rbx_gamelist_media(const char *set, int kind, char *out, int size)
{
	static const char *const dir[3] = { "marquee", "snap", "video" }, *const ext[3] = { "png", "png", "mp4" };
	if (!set || !set[0] || kind < 0 || kind > 2) {
		out[0] = 0;
		return NULL;
	}
	const rbx_list_entry *e = rbx_gamelist_find(set);
	const char *p = !e ? NULL : kind == 0 ? e->marquee : kind == 1 ? e->snap : e->video;
	if (p && p[0])
		return rbx_gamelist_path(p, out, size);
	snprintf(out, (size_t)size, RBX_DATA_DIR "%s/%s.%s", dir[kind], set, ext[kind]);
	return out;
}
