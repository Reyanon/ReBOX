// The folders chosen in Settings (gamelist.h, user 11-24; any number of each kind, user 11-28): ROM folders, snap
// folders, marquee folders, each a list in the order the user gave. Kept in folders.cfg as "<key><n>=<folder>" lines:
//   rom1=ux0:roms/        rom2=...      rom3=...
//   snap=ux0:snaps/       snap2=...     (the first without its number, as before 11-28)
//   marquee=...           marquee2=...
// A file of the earlier versions (rom1, rom2, snap, marquee; a folder not set written as "rom2=") reads the same, and
// those versions still read the first folders of a file written here. This file is part of ReBOX. GPL-2.0-or-later.
#include "frontend/gamelist.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "frontend/settings.h"
#include "platform/log.h"

#ifndef FOLDERS_PATH
#define FOLDERS_PATH RBX_DATA_DIR "folders.cfg"
#endif
#define FOLDER_LEN 160

typedef struct {
	char (*p)[FOLDER_LEN];
	int n, cap;
} folder_list;
static folder_list s_fl[RBX_FOLDER_KINDS];
static int s_rom_changed;
static const char *const k_key[RBX_FOLDER_KINDS] = { "rom", "snap", "marquee" };

// a folder ends in "/" (as the ROM folders), or is a device ("ux0:")
static void folder_norm(char *out, const char *path)
{
	snprintf(out, FOLDER_LEN, "%s", path ? path : "");
	const size_t l = strlen(out);
	if (l && out[l - 1] != '/' && out[l - 1] != ':' && l + 1 < FOLDER_LEN) {
		out[l] = '/';
		out[l + 1] = 0;
	}
}

static int folder_find(int kind, const char *p)
{
	for (int i = 0; i < s_fl[kind].n; i++)
		if (!strcmp(s_fl[kind].p[i], p))
			return i;
	return -1;
}

static int folder_append(int kind, const char *p)
{
	folder_list *l = &s_fl[kind];
	if (l->n == l->cap) {
		const int cap = l->cap ? l->cap * 2 : 4;
		void *q = realloc(l->p, (size_t)cap * FOLDER_LEN);
		if (!q)
			return 0;
		l->p = (char (*)[FOLDER_LEN])q;
		l->cap = cap;
	}
	memcpy(l->p[l->n++], p, FOLDER_LEN);
	return 1;
}

// "rom3" -> kind ROM, number 3; "snap" -> SNAP, 1. -1: not a folder key
static int folder_key(const char *k, int *num)
{
	for (int kind = 0; kind < RBX_FOLDER_KINDS; kind++) {
		const size_t l = strlen(k_key[kind]);
		if (strncmp(k, k_key[kind], l))
			continue;
		const char *d = k + l;
		if (!*d) {
			*num = 1;
			return kind;
		}
		int v = 0;
		for (; *d >= '0' && *d <= '9' && v < 100000000; d++)
			v = v * 10 + (*d - '0');
		if (*d || d == k + l)
			continue;
		*num = v;
		return kind;
	}
	return -1;
}

typedef struct { int kind, num, seq; char p[FOLDER_LEN]; } folder_line;
static int cmp_line(const void *a, const void *b)
{
	const folder_line *x = (const folder_line *)a, *y = (const folder_line *)b;
	if (x->kind != y->kind)
		return x->kind - y->kind;
	if (x->num != y->num)
		return x->num < y->num ? -1 : 1;
	return x->seq - y->seq;   // (two lines of one number: in the file's order)
}

void rbx_folders_load(void)
{
	for (int k = 0; k < RBX_FOLDER_KINDS; k++)
		s_fl[k].n = 0;
	FILE *f = fopen(FOLDERS_PATH, "r");
	if (!f)
		return;
	folder_line *v = NULL;
	int n = 0, cap = 0;
	char line[512];
	while (fgets(line, sizeof(line), f)) {
		line[strcspn(line, "\r\n")] = 0;
		char *eq = strchr(line, '=');
		if (!eq)
			continue;
		*eq = 0;
		int num = 0;
		const int kind = folder_key(line, &num);
		if (kind < 0 || !eq[1])   // (another key, or a folder not set)
			continue;
		if (n == cap) {
			const int c = cap ? cap * 2 : 8;
			void *q = realloc(v, (size_t)c * sizeof(*v));
			if (!q)
				break;
			v = (folder_line *)q;
			cap = c;
		}
		v[n].kind = kind;
		v[n].num = num;
		v[n].seq = n;
		folder_norm(v[n].p, eq + 1);
		n++;
	}
	fclose(f);
	if (n)
		qsort(v, (size_t)n, sizeof(*v), cmp_line);
	for (int i = 0; i < n; i++)
		if (folder_find(v[i].kind, v[i].p) < 0 && folder_append(v[i].kind, v[i].p))
			RBX_LOG("folders: %s %d %s", k_key[v[i].kind], s_fl[v[i].kind].n, v[i].p);
	free(v);
}

static void folders_save(void)
{
	FILE *f = fopen(FOLDERS_PATH, "w");
	if (!f) {
		RBX_LOG("folders: save failed %s", FOLDERS_PATH);
		return;
	}
	for (int k = 0; k < RBX_FOLDER_KINDS; k++)
		for (int i = 0; i < s_fl[k].n; i++) {
			if (k == RBX_FOLDER_ROM || i)
				fprintf(f, "%s%d=%s\n", k_key[k], i + 1, s_fl[k].p[i]);
			else
				fprintf(f, "%s=%s\n", k_key[k], s_fl[k].p[i]);
		}
	fclose(f);
}

int rbx_folder_count(int kind) { return kind >= 0 && kind < RBX_FOLDER_KINDS ? s_fl[kind].n : 0; }

const char *rbx_folder(int kind, int i)
{
	return kind >= 0 && kind < RBX_FOLDER_KINDS && i >= 0 && i < s_fl[kind].n ? s_fl[kind].p[i] : "";
}

void rbx_folder_remove(int kind, int i)
{
	if (kind < 0 || kind >= RBX_FOLDER_KINDS || i < 0 || i >= s_fl[kind].n)
		return;
	folder_list *l = &s_fl[kind];
	RBX_LOG("folders: %s %d removed (%s)", k_key[kind], i + 1, l->p[i]);
	memmove(l->p[i], l->p[i + 1], (size_t)(l->n - i - 1) * FOLDER_LEN);
	l->n--;
	if (kind == RBX_FOLDER_ROM)
		s_rom_changed = 1;
	folders_save();
}

int rbx_folder_set(int kind, int i, const char *path)
{
	if (kind < 0 || kind >= RBX_FOLDER_KINDS || i < 0 || i > s_fl[kind].n)
		return -1;
	char p[FOLDER_LEN];
	folder_norm(p, path);
	if (!p[0]) {   // (none: the folder taken away)
		rbx_folder_remove(kind, i);
		return -1;
	}
	const int at = folder_find(kind, p);
	if (at >= 0) {   // (already in the list: kept where it is; the row changed to it goes)
		if (i < s_fl[kind].n && i != at)
			rbx_folder_remove(kind, i);
		return folder_find(kind, p);
	}
	if (i == s_fl[kind].n) {
		if (!folder_append(kind, p))
			return -1;
	} else
		memcpy(s_fl[kind].p[i], p, FOLDER_LEN);
	if (kind == RBX_FOLDER_ROM)
		s_rom_changed = 1;
	folders_save();
	RBX_LOG("folders: %s %d %s", k_key[kind], i + 1, p);
	return i;
}

int rbx_folders_rom_changed(void)
{
	const int c = s_rom_changed;
	s_rom_changed = 0;
	return c;
}
