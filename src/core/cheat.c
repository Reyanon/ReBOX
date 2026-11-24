/*  ReBOX cheats: MAME's cheat XML (cheat.7z / <set>.xml, "mamecheat" version 1) run as MAME's cheat engine does
    (frontend/cheat.cpp): a cheat's scripts "on" (when it is turned on), "run" (every frame while on), "change" (its
    value changed) and "off"; their actions are MAME debugger expressions (debug/express.cpp): numbers in hex unless
    written otherwise, temp0..temp9 and param per cheat, frame, memory as [cpu.]{p,d,i,o,r,m}{b,w,d,q}@address, the C
    operators and their word forms (AND, OR, LT, ...), assignments. Memory: the main CPU's space when it is a 68000 on
    the shared bus (m68k_bus); "r" / "o" / "m" accesses write the mapped memory itself (ROM patches), "p" goes through
    the bus as the CPU would. A cheat that names another CPU, or a set whose main CPU is not one, is left out.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "rbx_core.h"
#include "rbx.h"
#include "cpu/m68k_bus.h"
#include "cpu/m68k/m68k.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define CHEAT_MAX 1024
#define ITEM_MAX  256

enum { SC_ON, SC_RUN, SC_CHANGE, SC_OFF, SC_N };
typedef struct { const char *cond, *expr; } cheat_action;
typedef struct {
	const char *desc;
	int nact[SC_N], act0[SC_N];      // a script's actions: s_act[act0 .. act0 + nact)
	int has_param, pmin, pmax, pstep;
	int nitem, item0;                // the parameter's items: s_item[item0 ..]
	int opt;                         // 0 off, else the option chosen
	uint64_t param, temp[10];
} cheat_t;
typedef struct { uint64_t value; const char *text; } cheat_item;

static char *s_xml;   // (the file, its strings cut in place)
static cheat_t *s_cheat;
static int s_n;
static cheat_action *s_act;
static int s_nact, s_act_cap;
static cheat_item *s_item;
static int s_nitem, s_item_cap;
static uint64_t s_frame;
static cheat_t *s_cur;   // (the cheat whose expression runs: its temps and param)

// ---- memory: the main 68000's space ----
static int m68k_main(void)
{
	for (int i = 0; i < M68K_PAGES; i++)
		if (m68k_bus.rd[i] || m68k_bus.wr[i])
			return 1;
	return m68k_bus.nparts > 0;
}

static uint8_t mem_rd8(uint32_t a)
{
	m68k_bus_t *const was = m68k_bus_cur;
	m68k_bus_cur = &m68k_bus;
	const uint8_t v = (uint8_t)m68k_read_memory_8(a & 0xffffff);
	m68k_bus_cur = was;
	return v;
}

static void mem_wr8(uint32_t a, uint8_t v, int direct)
{
	a &= 0xffffff;
	m68k_bus_t *const was = m68k_bus_cur;
	m68k_bus_cur = &m68k_bus;
	uint8_t *p = direct ? m68k_bus.rd[a >> M68K_PAGE_BITS] : NULL;
	if (p) {   // (the memory itself: a ROM patch too; the decrypted opcodes as well when the CPU fetches from them)
		p[(a & ((1u << M68K_PAGE_BITS) - 1)) ^ M68K_BYTE_XOR] = v;
		if (m68k_bus.op && a < m68k_bus.op_size)
			((uint8_t *)m68k_bus.op)[a ^ M68K_BYTE_XOR] = v;
	} else
		m68k_write_memory_8(a, v);
	m68k_bus_cur = was;
}

static uint64_t mem_read(uint32_t a, int size)
{
	uint64_t v = 0;
	for (int i = 0; i < size; i++)
		v = v << 8 | mem_rd8(a + (uint32_t)i);
	return v;
}

static void mem_write(uint32_t a, int size, uint64_t v, int direct)
{
	for (int i = size - 1; i >= 0; i--, v >>= 8)
		mem_wr8(a + (uint32_t)i, (uint8_t)v, direct);
}

// ---- expressions (MAME debugger syntax) ----
typedef struct { const char *p; int err, run; } ex_t;   // run: assignments and writes done (0: only checking)

static void ws(ex_t *e) { while (*e->p && isspace((unsigned char)*e->p)) e->p++; }

// a word operator (AND, OR, ...) at p, not followed by more of a name
static int word(ex_t *e, const char *w)
{
	ws(e);
	const size_t l = strlen(w);
	if (strncasecmp(e->p, w, l) || isalnum((unsigned char)e->p[l]) || e->p[l] == '_' || e->p[l] == '.')
		return 0;
	e->p += l;
	return 1;
}

// a symbol operator at p (not the start of a longer one given in `not`)
static int sym(ex_t *e, const char *s, const char *not)
{
	ws(e);
	const size_t l = strlen(s);
	if (strncmp(e->p, s, l))
		return 0;
	if (not && e->p[l] && strchr(not, e->p[l]))
		return 0;
	e->p += l;
	return 1;
}

static uint64_t ex_assign(ex_t *e);
static uint64_t ex_unary(ex_t *e);

// a name: letters, digits, '_' and '.'
static int name(ex_t *e, char *out, int max)
{
	ws(e);
	int n = 0;
	while (isalnum((unsigned char)e->p[n]) || e->p[n] == '_' || e->p[n] == '.') {
		if (n < max - 1)
			out[n] = e->p[n];
		n++;
	}
	out[n < max - 1 ? n : max - 1] = 0;
	return n;
}

// an lvalue: a variable (kind 1: temp n / 2: param) or memory (3: address, size, direct); 0 = not one
typedef struct { int kind, idx, size, direct; uint32_t addr; } lval_t;

// [cpu.][type]size@ : the memory reference's size (1..8) and type, or 0
static int memref(const char *nm, int *direct)
{
	const char *dot = strrchr(nm, '.');
	const char *spec = dot ? dot + 1 : nm;
	if (dot && ((size_t)(dot - nm) != 7 || strncasecmp(nm, "maincpu", 7)))
		return -1;   // (another CPU: not reached)
	char t = 'p', s;
	if (strlen(spec) == 2) {
		t = (char)tolower((unsigned char)spec[0]);
		s = (char)tolower((unsigned char)spec[1]);
	} else if (strlen(spec) == 1)
		s = (char)tolower((unsigned char)spec[0]);
	else
		return 0;
	if (!strchr("pdio3rm", t))
		return 0;
	*direct = t == 'r' || t == 'o' || t == 'm';
	return s == 'b' ? 1 : s == 'w' ? 2 : s == 'd' ? 4 : s == 'q' ? 8 : 0;
}

static uint64_t lval_get(const lval_t *l)
{
	if (l->kind == 1) return s_cur->temp[l->idx];
	if (l->kind == 2) return s_cur->param;
	return mem_read(l->addr, l->size);
}

static void lval_set(ex_t *e, const lval_t *l, uint64_t v)
{
	if (!e->run)
		return;
	if (l->kind == 1) s_cur->temp[l->idx] = v;
	else if (l->kind == 2) s_cur->param = v;
	else if (l->kind == 3) mem_write(l->addr, l->size, v, l->direct);
}

// a variable or memory reference at p (consumed), else 0 and p as it was
static int lvalue(ex_t *e, lval_t *l)
{
	const char *save = e->p;
	char nm[48];
	const int n = name(e, nm, sizeof(nm));
	memset(l, 0, sizeof(*l));
	if (!n)
		return 0;
	if (e->p[n] == '@') {
		int direct = 0;
		const int size = memref(nm, &direct);
		if (size < 0) {
			e->err = 2;   // (another CPU)
			return 0;
		}
		if (size) {
			e->p += n + 1;
			l->kind = 3;
			l->size = size;
			l->direct = direct;
			l->addr = (uint32_t)ex_unary(e);
			return 1;
		}
	}
	if (!strncasecmp(nm, "temp", 4) && isdigit((unsigned char)nm[4]) && !nm[5]) {
		e->p += n;
		l->kind = 1;
		l->idx = nm[4] - '0';
		return 1;
	}
	if (!strcasecmp(nm, "param")) {
		e->p += n;
		l->kind = 2;
		return 1;
	}
	e->p = save;
	return 0;
}

static uint64_t number(ex_t *e)
{
	ws(e);
	int base = 16;
	if (e->p[0] == '0' && (e->p[1] == 'x' || e->p[1] == 'X')) e->p += 2;
	else if (e->p[0] == '$') e->p++;
	else if (e->p[0] == '#') { base = 10; e->p++; }
	else if (e->p[0] == '0' && (e->p[1] == 'o' || e->p[1] == 'O')) { base = 8; e->p += 2; }
	uint64_t v = 0;
	int any = 0;
	for (;; e->p++) {
		const int c = tolower((unsigned char)*e->p);
		const int d = isdigit(c) ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : 99;
		if (d >= base)
			break;
		v = v * (uint64_t)base + (uint64_t)d;
		any = 1;
	}
	if (!any || isalpha((unsigned char)*e->p) || *e->p == '_')
		e->err = 1;
	return v;
}

static uint64_t primary(ex_t *e)
{
	ws(e);
	if (*e->p == '(') {
		e->p++;
		const uint64_t v = ex_assign(e);
		if (!sym(e, ")", NULL))
			e->err = 1;
		return v;
	}
	lval_t l;
	if (lvalue(e, &l))
		return lval_get(&l);
	if (e->err)
		return 0;
	char nm[48];
	const int n = name(e, nm, sizeof(nm));
	if (n && !strcasecmp(nm, "frame")) {
		e->p += n;
		return s_frame;
	}
	return number(e);
}

static uint64_t ex_unary(ex_t *e)
{
	if (sym(e, "!", "=") || word(e, "not")) return !ex_unary(e);
	if (sym(e, "~", NULL)) return ~ex_unary(e);
	if (sym(e, "-", "-=")) return (uint64_t)-(int64_t)ex_unary(e);
	if (sym(e, "+", "+=")) return ex_unary(e);
	return primary(e);
}

static uint64_t ex_mul(ex_t *e)
{
	uint64_t v = ex_unary(e);
	for (;;) {
		if (sym(e, "*", "=")) v *= ex_unary(e);
		else if (sym(e, "/", "=")) { const uint64_t d = ex_unary(e); v = d ? v / d : 0; }
		else if (sym(e, "%", "=")) { const uint64_t d = ex_unary(e); v = d ? v % d : 0; }
		else return v;
	}
}
static uint64_t ex_add(ex_t *e)
{
	uint64_t v = ex_mul(e);
	for (;;) {
		if (sym(e, "+", "=")) v += ex_mul(e);
		else if (sym(e, "-", "=")) v -= ex_mul(e);
		else return v;
	}
}
static uint64_t ex_shift(ex_t *e)
{
	uint64_t v = ex_add(e);
	for (;;) {
		if (sym(e, "<<", "=") || word(e, "lshift")) v <<= ex_add(e) & 63;
		else if (sym(e, ">>", "=") || word(e, "rshift")) v >>= ex_add(e) & 63;
		else return v;
	}
}
static uint64_t ex_rel(ex_t *e)
{
	uint64_t v = ex_shift(e);
	for (;;) {
		if (sym(e, "<=", NULL) || word(e, "le")) v = v <= ex_shift(e);
		else if (sym(e, ">=", NULL) || word(e, "ge")) v = v >= ex_shift(e);
		else if (sym(e, "<", "<=") || word(e, "lt")) v = v < ex_shift(e);
		else if (sym(e, ">", ">=") || word(e, "gt")) v = v > ex_shift(e);
		else return v;
	}
}
static uint64_t ex_eq(ex_t *e)
{
	uint64_t v = ex_rel(e);
	for (;;) {
		if (sym(e, "==", NULL) || word(e, "eq")) v = v == ex_rel(e);
		else if (sym(e, "!=", NULL) || word(e, "ne")) v = v != ex_rel(e);
		else return v;
	}
}
static uint64_t ex_band(ex_t *e)
{
	uint64_t v = ex_eq(e);
	while (sym(e, "&", "&=") || word(e, "band"))
		v &= ex_eq(e);
	return v;
}
static uint64_t ex_bxor(ex_t *e)
{
	uint64_t v = ex_band(e);
	while (sym(e, "^", "=") || word(e, "bxor"))
		v ^= ex_band(e);
	return v;
}
static uint64_t ex_bor(ex_t *e)
{
	uint64_t v = ex_bxor(e);
	while (sym(e, "|", "|=") || word(e, "bor"))
		v |= ex_bxor(e);
	return v;
}
static uint64_t ex_land(ex_t *e)
{
	uint64_t v = ex_bor(e);
	while (sym(e, "&&", NULL) || word(e, "and")) {
		const uint64_t r = ex_bor(e);   // (both sides read, as MAME: no side effects in conditions)
		v = v && r;
	}
	return v;
}
static uint64_t ex_lor(ex_t *e)
{
	uint64_t v = ex_land(e);
	while (sym(e, "||", NULL) || word(e, "or")) {
		const uint64_t r = ex_land(e);
		v = v || r;
	}
	return v;
}

static uint64_t ex_assign(ex_t *e)
{
	const char *save = e->p;
	lval_t l;
	const int saved_run = e->run;
	e->run = 0;   // (the address is read again below: nothing written while looking)
	const int is_l = lvalue(e, &l);
	e->run = saved_run;
	if (is_l) {
		static const char *const ops[] = { "<<=", ">>=", "+=", "-=", "*=", "/=", "%=", "&=", "^=", "|=", "=" };
		for (int k = 0; k < 11; k++) {
			ws(e);
			const size_t ol = strlen(ops[k]);
			if (strncmp(e->p, ops[k], ol) || (k == 10 && e->p[1] == '='))
				continue;
			e->p += ol;
			const uint64_t r = ex_assign(e);
			uint64_t v = k == 10 ? r : lval_get(&l);
			switch (k) {
			case 0: v <<= r & 63; break;
			case 1: v >>= r & 63; break;
			case 2: v += r; break;
			case 3: v -= r; break;
			case 4: v *= r; break;
			case 5: v = r ? v / r : 0; break;
			case 6: v = r ? v % r : 0; break;
			case 7: v &= r; break;
			case 8: v ^= r; break;
			case 9: v |= r; break;
			}
			lval_set(e, &l, v);
			return v;
		}
	}
	if (e->err)
		return 0;
	e->p = save;
	return ex_lor(e);
}

// run (or, run 0, only check) an expression; statements separated by ','. -1 = not understood, -2 = another CPU
static int64_t ex_run(const char *s, int run, uint64_t *out)
{
	ex_t e = { s, 0, run };
	uint64_t v = 0;
	do
		v = ex_assign(&e);
	while (!e.err && sym(&e, ",", NULL));
	ws(&e);
	if (e.err == 2)
		return -2;
	if (e.err || *e.p)
		return -1;
	if (out)
		*out = v;
	return 0;
}

// ---- the XML ----
static void unescape(char *s)
{
	static const struct { const char *e; char c; } ent[] = { { "&lt;", '<' }, { "&gt;", '>' }, { "&amp;", '&' },
		{ "&quot;", '"' }, { "&apos;", '\'' } };
	char *o = s;
	for (const char *p = s; *p;) {
		int k = -1;
		if (*p == '&')
			for (int i = 0; i < 5 && k < 0; i++)
				if (!strncmp(p, ent[i].e, strlen(ent[i].e)))
					k = i;
		if (k >= 0) {
			*o++ = ent[k].c;
			p += strlen(ent[k].e);
		} else
			*o++ = *p++;
	}
	*o = 0;
}

// the attribute's value in the tag [t, end) (cut and unescaped in place: the tag is not read again), else NULL
static char *attr(char *t, char *end, const char *a)
{
	const size_t l = strlen(a);
	for (char *p = t; p + l + 2 < end; p++) {
		if (strncmp(p, a, l) || p[l] != '=' || (p[l + 1] != '"' && p[l + 1] != '\'') || !isspace((unsigned char)p[-1]))
			continue;
		const char q = p[l + 1];
		char *v = p + l + 2, *e = v;
		while (e < end && *e != q)
			e++;
		if (e >= end)
			return NULL;
		*e = 0;
		unescape(v);
		return v;
	}
	return NULL;
}

static long attr_int(char *t, char *end, const char *a, long def)
{
	const char *v = attr(t, end, a);
	return v ? strtol(v, NULL, 0) : def;
}

static int add_action(const char *cond, const char *expr)
{
	if (s_nact == s_act_cap) {
		const int cap = s_act_cap ? s_act_cap * 2 : 256;
		cheat_action *n = (cheat_action *)realloc(s_act, (size_t)cap * sizeof(*n));
		if (!n)
			return -1;
		s_act = n;
		s_act_cap = cap;
	}
	s_act[s_nact].cond = cond;
	s_act[s_nact].expr = expr;
	return s_nact++;
}

static int add_item(uint64_t v, const char *text)
{
	if (s_nitem == s_item_cap) {
		const int cap = s_item_cap ? s_item_cap * 2 : 256;
		cheat_item *n = (cheat_item *)realloc(s_item, (size_t)cap * sizeof(*n));
		if (!n)
			return -1;
		s_item = n;
		s_item_cap = cap;
	}
	s_item[s_nitem].value = v;
	s_item[s_nitem].text = text;
	return s_nitem++;
}

void rbx_cheat_clear(void)
{
	free(s_xml);
	free(s_cheat);
	free(s_act);
	free(s_item);
	s_xml = NULL;
	s_cheat = NULL;
	s_act = NULL;
	s_item = NULL;
	s_n = s_nact = s_act_cap = s_nitem = s_item_cap = 0;
	s_frame = 0;
}

// a script's actions all understood (checked once, nothing run); -2: one names another CPU
static int script_ok(cheat_t *c)
{
	s_cur = c;
	for (int k = 0; k < SC_N; k++)
		for (int i = 0; i < c->nact[k]; i++) {
			const cheat_action *a = &s_act[c->act0[k] + i];
			const int64_t r1 = a->cond ? ex_run(a->cond, 0, NULL) : 0, r2 = ex_run(a->expr, 0, NULL);
			if (r1 || r2)
				return r1 == -2 || r2 == -2 ? -2 : -1;
		}
	return 0;
}

int rbx_cheat_load_xml(const char *xml, size_t len)
{
	rbx_cheat_clear();
	if (!xml || !len)
		return 0;
	if (!m68k_main()) {
		rbx_log("cheats: the main CPU of this board is not reached yet (68000 boards only)");
		return 0;
	}
	s_xml = (char *)malloc(len + 1);
	s_cheat = (cheat_t *)calloc(CHEAT_MAX, sizeof(cheat_t));
	if (!s_xml || !s_cheat) {
		rbx_cheat_clear();
		return 0;
	}
	memcpy(s_xml, xml, len);
	s_xml[len] = 0;
	int skipped = 0, other_cpu = 0;
	cheat_t *c = NULL;
	int script = -1, in_param = 0;
	for (char *p = s_xml; (p = strchr(p, '<')) != NULL;) {
		char *end = strchr(p, '>');
		if (!end)
			break;
		*end = 0;   // (the tag's text ends here: attributes read within it)
		char *t = p + 1;
		char *next = end + 1;
		if (!strncmp(t, "!--", 3)) {   // (a comment: to its end)
			*end = '>';
			char *ce = strstr(p, "-->");
			p = ce ? ce + 3 : next;
			continue;
		}
		if (!strncmp(t, "cheat", 5) && (isspace((unsigned char)t[5]) || !t[5] || t[5] == '/')) {
			c = s_n < CHEAT_MAX ? &s_cheat[s_n] : NULL;
			if (c) {
				memset(c, 0, sizeof(*c));
				c->desc = attr(t, end, "desc");
				for (int k = 0; k < SC_N; k++)
					c->act0[k] = -1;
			}
			script = -1;
			in_param = 0;
			if (end > t && end[-1] == '/') {   // (<cheat desc=""/>: a separator)
				c = NULL;
			}
		} else if (!strncmp(t, "/cheat", 6)) {
			if (c) {
				int any = 0;
				for (int k = 0; k < SC_N; k++)
					any |= c->nact[k] > 0;
				const int ok = any ? script_ok(c) : -1;
				if (any && !ok && c->desc && c->desc[0])
					s_n++;
				else if (any) {
					skipped++;
					other_cpu += ok == -2;
				}
			}
			c = NULL;
		} else if (c && !strncmp(t, "parameter", 9)) {
			c->has_param = 1;
			c->pmin = (int)attr_int(t, end, "min", 0);
			c->pmax = (int)attr_int(t, end, "max", 0);
			c->pstep = (int)attr_int(t, end, "step", 1);
			if (c->pstep <= 0)
				c->pstep = 1;
			c->item0 = s_nitem;
			in_param = !(end > t && end[-1] == '/');
		} else if (c && !strncmp(t, "/parameter", 10)) {
			in_param = 0;
		} else if (c && in_param && !strncmp(t, "item", 4)) {
			const long v = attr_int(t, end, "value", 0);
			char *te = strchr(next, '<');
			if (te) {
				*te = 0;   // (the item's text; the closing tag after it is found from te + 1)
				unescape(next);
				if (c->nitem < ITEM_MAX && add_item((uint64_t)v, next) >= 0)
					c->nitem++;
				next = te + 1;   // (the closing tag: passed over as text up to the next '<')
			}
		} else if (c && !strncmp(t, "script", 6)) {
			const char *st = attr(t, end, "state");
			script = !st || !strcmp(st, "run") ? SC_RUN : !strcmp(st, "on") ? SC_ON : !strcmp(st, "off") ? SC_OFF
			         : !strcmp(st, "change") ? SC_CHANGE : -1;
			if (end > t && end[-1] == '/')
				script = -1;
		} else if (c && !strncmp(t, "/script", 7)) {
			script = -1;
		} else if (c && script >= 0 && !strncmp(t, "action", 6)) {
			const char *cond = attr(t, end, "condition");
			char *te = strchr(next, '<');
			if (te) {
				*te = 0;
				unescape(next);
				// (a script's actions are together in s_act: one script after another per cheat)
				if (c->act0[script] < 0 || c->act0[script] + c->nact[script] == s_nact) {
					const int i = add_action(cond, next);
					if (i >= 0) {
						if (c->act0[script] < 0)
							c->act0[script] = i;
						c->nact[script]++;
					}
				}
				next = te + 1;
			}
		}
		p = next;
	}
	if (s_n == 0)
		rbx_cheat_clear();
	rbx_log("cheats: %d (%d left out%s)", s_n, skipped, other_cpu ? ", some for another CPU" : "");
	return s_n;
}

static void run_script(cheat_t *c, int k)
{
	s_cur = c;
	for (int i = 0; i < c->nact[k]; i++) {
		const cheat_action *a = &s_act[c->act0[k] + i];
		uint64_t ok = 1;
		if (a->cond && (ex_run(a->cond, 0, &ok) || !ok))
			continue;
		ex_run(a->expr, 1, NULL);
	}
}

int rbx_cheat_count(void) { return s_n; }
const char *rbx_cheat_name(int i) { return i >= 0 && i < s_n ? s_cheat[i].desc : NULL; }

// options: 0 off, then the items, or the range's values, or (no parameter) on
int rbx_cheat_options(int i)
{
	if (i < 0 || i >= s_n)
		return 0;
	const cheat_t *c = &s_cheat[i];
	if (!c->has_param)
		return 2;
	if (c->nitem)
		return 1 + c->nitem;
	const int n = c->pmax >= c->pmin ? (c->pmax - c->pmin) / c->pstep + 1 : 1;
	return 1 + (n > 4096 ? 4096 : n);
}

static uint64_t opt_value(const cheat_t *c, int o)
{
	if (!c->has_param || o <= 0)
		return 0;
	return c->nitem ? s_item[c->item0 + o - 1].value : (uint64_t)(c->pmin + (o - 1) * c->pstep);
}

const char *rbx_cheat_option_name(int i, int o)
{
	static char buf[4][24];
	static int k;
	if (i < 0 || i >= s_n || o < 0 || o >= rbx_cheat_options(i))
		return NULL;
	const cheat_t *c = &s_cheat[i];
	if (o == 0)
		return "Off";
	if (!c->has_param)
		return "On";
	if (c->nitem)
		return s_item[c->item0 + o - 1].text;
	k = (k + 1) & 3;
	snprintf(buf[k], sizeof(buf[k]), "%d", (int)opt_value(c, o));
	return buf[k];
}

int rbx_cheat_get(int i) { return i >= 0 && i < s_n ? s_cheat[i].opt : 0; }

void rbx_cheat_set(int i, int o)
{
	if (i < 0 || i >= s_n || o < 0 || o >= rbx_cheat_options(i))
		return;
	cheat_t *c = &s_cheat[i];
	if (o == c->opt)
		return;
	const int was = c->opt;
	c->opt = o;
	c->param = opt_value(c, o);
	if (!o)
		run_script(c, SC_OFF);
	else if (!was)
		run_script(c, SC_ON);
	else
		run_script(c, c->nact[SC_CHANGE] ? SC_CHANGE : SC_ON);
}

// once a frame, after it ran (the core's rbx_run_frame)
void rbx_cheat_frame(void)
{
	for (int i = 0; i < s_n; i++)
		if (s_cheat[i].opt)
			run_script(&s_cheat[i], SC_RUN);
	s_frame++;
}
