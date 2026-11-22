// The menus' language (user 11-22). See lang.h. GPL-2.0-or-later.
//   ux0:data/rebox/lang/<code>.json         { "code": "kor", "name": "한국어", "english": "Korean",
//                                              "strings": { "<English text>": "<its translation>", ... } }
//   ux0:data/rebox/lang/v<version>-<code>.json   a release's changes: its strings replace or add to the earlier ones
// The VPK carries them (app0:lang/); the missing ones are copied to the data folder at start, the ones there are kept.
#include "frontend/lang.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <psp2/io/dirent.h>
#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>

#include "frontend/settings.h"
#include "frontend/ui.h"
#include "platform/log.h"

#define LANG_DIR RBX_DATA_DIR "lang/"
#define APP_LANG_DIR "app0:lang/"
#define MAX_LANGS 32
#define MAP_N 2048   // (open addressing: about 350 strings)

static rbx_lang s_langs[MAX_LANGS];
static int s_nlangs;
static struct { const char *k, *v; } s_map[MAP_N];
static int s_on;   // a translation is in use (English: none)

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

// the JSON string starting after the " at p, unescaped in place (\n \t \" \\ \/ \uXXXX); returns what follows it
static char *json_str(char *p)
{
	char *w = p;
	while (*p && *p != '"') {
		if (*p == '\\' && p[1]) {
			p++;
			const char k = *p;
			if (k == 'n') { *w++ = '\n'; p++; continue; }
			if (k == 't') { *w++ = '\t'; p++; continue; }
			if (k == 'r') { p++; continue; }
			if (k == 'u' && p[1] && p[2] && p[3] && p[4]) {
				char hex[5] = { p[1], p[2], p[3], p[4], 0 };
				unsigned u = (unsigned)strtoul(hex, NULL, 16);
				p += 5;
				if (u >= 0xD800 && u < 0xDC00 && p[0] == '\\' && p[1] == 'u' && p[2] && p[3] && p[4] && p[5]) {
					char h2[5] = { p[2], p[3], p[4], p[5], 0 };
					const unsigned lo = (unsigned)strtoul(h2, NULL, 16);
					if (lo >= 0xDC00 && lo < 0xE000) {
						u = 0x10000 + ((u - 0xD800) << 10) + (lo - 0xDC00);
						p += 6;
					}
				}
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

static unsigned hash(const char *s)
{
	unsigned h = 2166136261u;
	while (*s)
		h = (h ^ (unsigned char)*s++) * 16777619u;
	return h;
}
static void map_put(const char *k, const char *v)
{
	unsigned h = hash(k) & (MAP_N - 1);
	for (int n = 0; n < MAP_N; n++, h = (h + 1) & (MAP_N - 1)) {
		if (!s_map[h].k || !strcmp(s_map[h].k, k)) {
			s_map[h].k = k;
			s_map[h].v = v;
			return;
		}
	}
}

// a file's header fields and (map) its strings into the table; the buffer is kept (the table points into it)
static int parse(char *b, rbx_lang *head, int map)
{
	char *p = b;
	int in_strings = 0, depth = 0;
	while (*p) {
		if (*p == '{') { depth++; p++; continue; }
		if (*p == '}') { depth--; if (in_strings && depth == 1) in_strings = 0; p++; continue; }
		if (*p != '"') { p++; continue; }
		char *k = p + 1;
		p = json_str(k);
		while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
			p++;
		if (*p != ':')
			continue;
		p++;
		while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
			p++;
		if (depth == 1 && !strcmp(k, "strings") && *p == '{') {
			in_strings = 1;
			continue;
		}
		if (*p != '"')
			continue;
		char *v = p + 1;
		p = json_str(v);
		if (in_strings && depth == 2) {
			if (map)
				map_put(k, v);
		} else if (depth == 1 && head) {
			if (!strcmp(k, "code")) snprintf(head->code, sizeof(head->code), "%s", v);
			else if (!strcmp(k, "name")) snprintf(head->name, sizeof(head->name), "%s", v);
			else if (!strcmp(k, "english")) snprintf(head->english, sizeof(head->english), "%s", v);
		}
	}
	return 1;
}

static int is_json(const char *n)
{
	const size_t l = strlen(n);
	return l >= 6 && !strcmp(n + l - 5, ".json");
}
static int is_version(const char *n) { return n[0] == 'v' && n[1] >= '0' && n[1] <= '9'; }

// the VPK's files copied to the data folder when not there (one there, maybe edited, is kept)
static void install_missing(void)
{
	const SceUID d = sceIoDopen(APP_LANG_DIR);
	if (d < 0)
		return;
	sceIoMkdir(RBX_DATA_DIR, 0777);
	sceIoMkdir(LANG_DIR, 0777);
	SceIoDirent de;
	int n = 0;
	for (;;) {
		memset(&de, 0, sizeof(de));
		if (sceIoDread(d, &de) <= 0)
			break;
		if (!is_json(de.d_name))
			continue;
		char src[160], dst[160];
		snprintf(src, sizeof(src), APP_LANG_DIR "%s", de.d_name);
		snprintf(dst, sizeof(dst), LANG_DIR "%s", de.d_name);
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
				sceIoRemove(dst);
			else
				n++;
		}
		free(b);
	}
	sceIoDclose(d);
	if (n)
		RBX_LOG("lang: %d files installed from the app", n);
}

// "1.2.10" after "1.2.9": the numbers compared as numbers
static int cmp_version(const char *a, const char *b)
{
	while (*a && *b) {
		if (*a >= '0' && *a <= '9' && *b >= '0' && *b <= '9') {
			const long x = strtol(a, (char **)&a, 10), y = strtol(b, (char **)&b, 10);
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
static int cmp_name(const void *a, const void *b) { return cmp_version((const char *)a, (const char *)b); }
static int cmp_lang(const void *a, const void *b)
{
	return strcasecmp(((const rbx_lang *)a)->english, ((const rbx_lang *)b)->english);
}

void rbx_lang_init(void)
{
	install_missing();
	// the languages: the files without a version, by their English name
	s_nlangs = 0;
	const SceUID d = sceIoDopen(LANG_DIR);
	SceIoDirent de;
	for (int more = d >= 0; more && s_nlangs < MAX_LANGS;) {
		memset(&de, 0, sizeof(de));
		more = sceIoDread(d, &de) > 0;
		if (!more || !is_json(de.d_name) || is_version(de.d_name))
			continue;
		char path[160];
		snprintf(path, sizeof(path), LANG_DIR "%s", de.d_name);
		char *b = read_all(path);
		if (!b)
			continue;
		rbx_lang *l = &s_langs[s_nlangs];
		memset(l, 0, sizeof(*l));
		parse(b, l, 0);
		free(b);
		if (l->code[0] && l->name[0]) {
			if (!l->english[0])
				snprintf(l->english, sizeof(l->english), "%s", l->name);
			s_nlangs++;
		}
	}
	if (d >= 0)
		sceIoDclose(d);
	qsort(s_langs, (size_t)s_nlangs, sizeof(s_langs[0]), cmp_lang);
	RBX_LOG("lang: %d languages", s_nlangs);
	rbx_lang_use(g_settings.language);
}

void rbx_lang_use(const char *code)
{
	static char *bufs[64];
	static int nbufs;
	for (int i = 0; i < nbufs; i++)
		free(bufs[i]);
	nbufs = 0;
	memset(s_map, 0, sizeof(s_map));
	s_on = 0;
	if (!code || !code[0] || !strcmp(code, "eng"))
		return;
	// the base file, then the releases' files of this language in version order (a later one replaces or adds)
	static char names[64][64];
	int nn = 0;
	snprintf(names[nn++], sizeof(names[0]), "%s.json", code);
	char suffix[24];
	snprintf(suffix, sizeof(suffix), "-%s.json", code);
	const size_t sl = strlen(suffix);
	const SceUID d = sceIoDopen(LANG_DIR);
	SceIoDirent de;
	for (int more = d >= 0; more && nn < 64;) {
		memset(&de, 0, sizeof(de));
		more = sceIoDread(d, &de) > 0;
		const size_t l = strlen(de.d_name);
		if (!more || !is_version(de.d_name) || l <= sl || l >= sizeof(names[0]) || strcmp(de.d_name + l - sl, suffix))
			continue;
		memcpy(names[nn++], de.d_name, l + 1);
	}
	if (d >= 0)
		sceIoDclose(d);
	qsort(names + 1, (size_t)(nn - 1), sizeof(names[0]), cmp_name);
	int strings = 0;
	for (int i = 0; i < nn && nbufs < 64; i++) {
		char path[160];
		snprintf(path, sizeof(path), LANG_DIR "%s", names[i]);
		char *b = read_all(path);
		if (!b)
			continue;
		bufs[nbufs++] = b;
		parse(b, NULL, 1);
		strings++;
	}
	s_on = strings > 0;
	RBX_LOG("lang: %s, %d files", code, strings);
}

int rbx_lang_count(void) { return s_nlangs; }
int rbx_lang_english(void) { return !s_on; }
const rbx_lang *rbx_lang_at(int i) { return i >= 0 && i < s_nlangs ? &s_langs[i] : NULL; }
int rbx_lang_index(const char *code)
{
	for (int i = 0; i < s_nlangs; i++)
		if (!strcmp(s_langs[i].code, code))
			return i;
	return -1;
}

const char *ui_tr(const char *s)
{
	if (!s_on || !s || !s[0])
		return s;
	unsigned h = hash(s) & (MAP_N - 1);
	for (int n = 0; n < MAP_N; n++, h = (h + 1) & (MAP_N - 1)) {
		if (!s_map[h].k)
			return s;
		if (!strcmp(s_map[h].k, s))
			return s_map[h].v && s_map[h].v[0] ? s_map[h].v : s;
	}
	return s;
}
