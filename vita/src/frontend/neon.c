// Neon corridor: the moving background beside the game picture. A corridor seen down its length: its walls' depth lines
// (rails) run from the screen's edges in to the picture's sides, rings on the walls flow out to the screen's edges, light
// pulses run out along the rails and dust drifts by the walls. Nothing shows inside the picture's columns (it covers them)
// The background and the rails do not move: made into vertices once (again when the picture's width or the theme changes)
// and kept; the rings, pulses and dust are made each frame into vita2d's pool. Lines are thin quads, added (blend add).
// This file is part of ReBOX. GPL-2.0-or-later.
#include "frontend/neon.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <psp2/gxm.h>
#include <psp2/io/stat.h>
#include <psp2/kernel/cpu.h>
#include <psp2/kernel/sysmem.h>
#include <psp2/kernel/threadmgr.h>
#include <vita2d.h>

#include "frontend/settings.h"
#include "platform/log.h"
#include "platform/timer.h"

// speed: the HTML's speed; density: rings between the screen's edge and the picture; hue: degrees, below 0 the theme's;
// glow, bright: tenths (the halo's strength, the lines'); pulses, dust: how many
const rbx_neon_param rbx_neon_params[RBX_NEON_COUNT] = {
	{ "neon_speed", "Artwork speed", 0, 10, 1, 10 },
	{ "neon_density", "Artwork rings", 2, 20, 1, 2 },
	{ "neon_hue", "Artwork hue", -10, 350, 10, -10 },
	{ "neon_glow", "Artwork glow", 0, 30, 1, 10 },
	{ "neon_bright", "Artwork brightness", 1, 20, 1, 5 },
	{ "neon_pulses", "Artwork pulses", 0, 60, 2, 60 },
	{ "neon_dust", "Artwork dust", 0, 300, 20, 200 },
};

void rbx_neon_value_text(int k, int v, char *out, int size)
{
	if (k == RBX_NEON_HUE && v < 0)
		snprintf(out, size, "Theme");
	else if (k == RBX_NEON_GLOW || k == RBX_NEON_BRIGHT)
		snprintf(out, size, "%d.%d", v / 10, v % 10);
	else
		snprintf(out, size, "%d", v);
}

// (turned, user 11-22: the game list's corridor runs up and down, its walls at the top and bottom. Made upright on a
// 544 x 960 screen and each vertex's x and y swapped)
static int s_rot, s_lay_rot = -1;
#define SW (s_lay_rot == 1 ? 544 : 960)
#define SH (s_lay_rot == 1 ? 960 : 544)
void rbx_neon_rotate(int on) { s_rot = on != 0; }
#define ZM 40.0f       // the depth whose walls meet the picture's sides
#define KN (s_lay_rot == 1 ? g_settings.neon_list : g_settings.neon)   // (the list's own knobs, user 11-22)
#define DENSITY ((float)KN[RBX_NEON_DENSITY])
#define SPEED ((float)KN[RBX_NEON_SPEED])
#define GLOW (KN[RBX_NEON_GLOW] * 0.1f)
#define BRIGHT (KN[RBX_NEON_BRIGHT] * 0.1f)
#define NPULSE_MAX 60
#define NDUST_MAX 300
#define NPULSE (KN[RBX_NEON_PULSES])
#define NDUST (KN[RBX_NEON_DUST])
#define RAIL_SEG 16
#define STATIC_BYTES (512 * 1024)
#define DYN_MAX 9000   // vertices a frame

typedef vita2d_color_vertex Vtx;
static void swap_xy(Vtx *v, int n)
{
	for (int i = 0; i < n; i++) {
		const float t = v[i].x;
		v[i].x = v[i].y;
		v[i].y = t;
	}
}

static float F, HW, HH, ZN, ZF, RING, cx, cy, edge;
static float s_lay_x = -1, s_lay_w = -1;
static uint32_t s_lay_acc;
static int s_lay_knobs[RBX_NEON_COUNT];

// colours: the HTML's hsla(hue + dh, 85%, l%), the hue the theme accent's
static float s_hue, s_sat = 0.85f;   // (saturation 0: a grey theme's lines grey and white)
static uint32_t rgb_hsl(float h, float l)
{
	h = fmodf(h + 360.0f, 360.0f) / 60.0f;
	const float s = s_sat, c = (1.0f - fabsf(2.0f * l - 1.0f)) * s, x = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f)), m = l - c / 2;
	float r = 0, g = 0, b = 0;
	switch ((int)h) {
	case 0: r = c; g = x; break;
	case 1: r = x; g = c; break;
	case 2: g = c; b = x; break;
	case 3: g = x; b = c; break;
	case 4: r = x; b = c; break;
	default: r = c; b = x; break;
	}
	return (uint32_t)((r + m) * 255.0f + 0.5f) | (uint32_t)((g + m) * 255.0f + 0.5f) << 8 | (uint32_t)((b + m) * 255.0f + 0.5f) << 16;
}
static int s_add;   // (drawing added: vita2d's blend add takes the colour whole, so it is the colour times alpha)
static uint32_t col(uint32_t rgb, float a)
{
	if (s_add)
		a *= BRIGHT;
	if (a > 1.0f)
		a = 1.0f;
	if (a < 0.0f)
		a = 0.0f;
	if (s_add) {
		const unsigned k = (unsigned)(a * 256.0f);
		return ((rgb & 0xFF) * k >> 8) | (((rgb >> 8) & 0xFF) * k >> 8) << 8 | (((rgb >> 16) & 0xFF) * k >> 8) << 16 | 0xFF000000u;
	}
	return rgb | (uint32_t)(a * 255.0f + 0.5f) << 24;
}
static uint32_t C55, C62, C76, C82, C50, P65, P88, D80;   // (lightness, the pulses' hue +25, the dust's -20)

static float hue_of(uint32_t acc)
{
	const float r = (acc & 0xFF) / 255.0f, g = ((acc >> 8) & 0xFF) / 255.0f, b = ((acc >> 16) & 0xFF) / 255.0f;
	const float mx = fmaxf(r, fmaxf(g, b)), mn = fminf(r, fminf(g, b)), d = mx - mn;
	if (d < 0.12f)
		return -1.0f;   // (a grey theme: no hue, grey lines)
	float h = mx == r ? fmodf((g - b) / d, 6.0f) : mx == g ? (b - r) / d + 2.0f : (r - g) / d + 4.0f;
	h *= 60.0f;
	return h < 0 ? h + 360.0f : h;   // (0-360: below 0 means grey)
}

static void proj(float x, float y, float z, float *px, float *py)
{
	*px = cx + x * F / z;
	*py = cy + y * F / z;
}
static float smooth(float a, float b, float v)
{
	float t = (v - a) / (b - a);
	t = t < 0 ? 0 : t > 1 ? 1 : t;
	return t * t * (3 - 2 * t);
}
// fades out into the picture's columns
static float side(float sx) { return smooth(edge * 0.7f, edge * 1.02f, fabsf(sx - cx)); }
static float z_at(float t) { return 1.0f / (1.0f / ZN + (1.0f / ZF - 1.0f / ZN) * t); }

static Vtx *s_v;
static int s_n, s_max;
static void tri(float x0, float y0, uint32_t c0, float x1, float y1, uint32_t c1, float x2, float y2, uint32_t c2)
{
	if (s_n + 3 > s_max)
		return;
	Vtx *v = s_v + s_n;
	v[0] = (Vtx){ x0, y0, 0.5f, c0 };
	v[1] = (Vtx){ x1, y1, 0.5f, c1 };
	v[2] = (Vtx){ x2, y2, 0.5f, c2 };
	s_n += 3;
}
// a line a-b as a quad, its widths and colours at each end (thinner than a pixel: a pixel wide and fainter)
static void seg(float ax, float ay, float bx, float by, float wa, float wb, uint32_t rgb, float aa, float ab)
{
	if (wa < 1.0f) {
		aa *= wa;
		wa = 1.0f;
	}
	if (wb < 1.0f) {
		ab *= wb;
		wb = 1.0f;
	}
	float dx = bx - ax, dy = by - ay;
	const float len = sqrtf(dx * dx + dy * dy);
	if (len < 0.01f)
		return;
	dx /= len;
	dy /= len;
	const float nax = -dy * wa * 0.5f, nay = dx * wa * 0.5f, nbx = -dy * wb * 0.5f, nby = dx * wb * 0.5f;
	const uint32_t ca = col(rgb, aa), cb = col(rgb, ab);
	tri(ax + nax, ay + nay, ca, ax - nax, ay - nay, ca, bx + nbx, by + nby, cb);
	tri(bx + nbx, by + nby, cb, ax - nax, ay - nay, ca, bx - nbx, by - nby, cb);
}
static void rect(float x0, float y0, float x1, float y1, uint32_t c0, uint32_t c1)   // (colours left, right)
{
	tri(x0, y0, c0, x0, y1, c0, x1, y0, c1);
	tri(x1, y0, c1, x0, y1, c0, x1, y1, c1);
}

static uint32_t s_rand = 0x2545F491;
static float rnd(void)
{
	s_rand = s_rand * 1664525u + 1013904223u;
	return (s_rand >> 8) * (1.0f / 16777216.0f);
}

// rails (depth lines): x, y in corridor halves, major
#define NRAIL (33 * 2 + 8 * 4)
static float s_rail[NRAIL][3];
static void rails_make(void)
{
	int n = 0;
	for (int i = 0; i <= 32; i++) {
		const float fy = -1.0f + i / 16.0f;
		for (int s = -1; s <= 1; s += 2) {
			s_rail[n][0] = (float)s;
			s_rail[n][1] = fy;
			s_rail[n++][2] = i % 4 == 0;
		}
	}
	for (int i = 0; i < 8; i++)
		for (int s = -1; s <= 1; s += 2)
			for (int f = -1; f <= 1; f += 2) {
				s_rail[n][0] = s * (1.0f - i * 0.06f);
				s_rail[n][1] = (float)f;
				s_rail[n++][2] = i == 0;
			}
}

static struct { float z, v, len; int r; } s_pulse[NPULSE_MAX];
static struct { float sx, fx, fy, t; } s_dust[NDUST_MAX];
static float s_travel, s_time;
static uint64_t s_last;

static SceUID s_blk = -1;
static Vtx *s_static;
static int s_nbg, s_nstatic;

static void layout(float gx, float gw)
{
	float c = gw / SW;
	c = c < 0.2f ? 0.2f : c > 0.96f ? 0.96f : c;
	F = (SW < SH ? SW : SH) * 0.9f;
	edge = gw * 0.5f;
	cx = gx + gw * 0.5f;
	cy = SH * 0.5f;
	HW = edge * ZM / F;
	HH = (SH / 2) * 1.05f * ZM * 1.6f / F;
	ZN = ZM * c * 0.9f;
	ZF = ZM * 1.6f;
	RING = (ZM - ZM * c) / DENSITY;
	s_travel = fmodf(s_travel, RING * 5.0f);
}

// the background and the rails into the kept vertices
static void static_make(void)
{
	if (s_blk < 0) {
		s_blk = sceKernelAllocMemBlock("rbx_neon", SCE_KERNEL_MEMBLOCK_TYPE_USER_RW_UNCACHE, STATIC_BYTES, NULL);
		if (s_blk < 0)
			return;
		void *p;
		sceKernelGetMemBlockBase(s_blk, &p);
		if (sceGxmMapMemory(p, STATIC_BYTES, SCE_GXM_MEMORY_ATTRIB_READ) < 0) {
			sceKernelFreeMemBlock(s_blk);
			s_blk = -1;
			return;
		}
		s_static = (Vtx *)p;
		rails_make();
	} else
		vita2d_wait_rendering_done();   // (the GPU may be reading them)
	s_v = s_static;
	s_n = 0;
	s_max = STATIC_BYTES / sizeof(Vtx);
	// the ground, the bands darkest at the screen's edges and melting into it towards the picture, a faint glow in each
	const uint32_t ground = 0xFF26211C, deep = 0xFF130F0A, mid = 0xFF1F1A14, clear = 0x0026211C;
	rect(0, 0, SW, SH, ground, ground);
	const float band = SW * 0.5f - edge;
	for (int s = -1; s <= 1; s += 2) {
		const float x0 = s < 0 ? 0 : SW, x1 = cx + s * edge * 0.85f, xm = x0 + (x1 - x0) * 0.7f;
		if (s < 0) {
			rect(x0, 0, xm, SH, deep, mid);
			rect(xm, 0, x1, SH, mid, clear);
		} else {
			rect(xm, 0, x0, SH, mid, deep);
			rect(x1, 0, xm, SH, clear, mid);
		}
		const float gx = cx + s * (edge + band * 0.45f), r = fmaxf(band * 1.6f, SH * 0.45f);
		const uint32_t c0 = col(C50, 0.10f * GLOW), c1 = col(C50, 0);
		for (int k = 0; k < 32; k++) {
			const float a0 = k * (6.2831853f / 32), a1 = (k + 1) * (6.2831853f / 32);
			tri(gx, cy, c0, gx + cosf(a0) * r, cy + sinf(a0) * r, c1, gx + cosf(a1) * r, cy + sinf(a1) * r, c1);
		}
	}
	s_nbg = s_n;
	s_add = 1;
	for (int i = 0; i < NRAIL; i++) {
		const float x = s_rail[i][0] * HW, y = s_rail[i][1] * HH, k8 = s_rail[i][2] ? 1.0f : 0.5f;
		float px[RAIL_SEG + 1], py[RAIL_SEG + 1], a[RAIL_SEG + 1], w[RAIL_SEG + 1];
		for (int k = 0; k <= RAIL_SEG; k++) {
			const float z = z_at((float)k / RAIL_SEG);
			proj(x, y, z, &px[k], &py[k]);
			a[k] = side(px[k]) * k8;
			w[k] = fmaxf(0.6f, F / z * HW * 0.0009f);
		}
		for (int k = 0; k < RAIL_SEG; k++) {
			if (a[k] < 0.01f && a[k + 1] < 0.01f)
				continue;
			if ((py[k] < -8 && py[k + 1] < -8) || (py[k] > SH + 8 && py[k + 1] > SH + 8))
				continue;
			if (GLOW > 0)
				seg(px[k], py[k], px[k + 1], py[k + 1], w[k] * 5, w[k + 1] * 5, C55, a[k] * 0.07f * GLOW, a[k + 1] * 0.07f * GLOW);
			seg(px[k], py[k], px[k + 1], py[k + 1], w[k], w[k + 1], s_rail[i][2] ? C76 : C62, a[k] * 0.7f, a[k + 1] * 0.7f);
		}
	}
	s_nstatic = s_n;
	s_add = 0;
	if (s_lay_rot == 1)
		swap_xy(s_static, s_nstatic);
}

static void pulse_new(int i, float z)
{
	s_pulse[i].r = (int)(rnd() * NRAIL) % NRAIL;
	s_pulse[i].z = z;
	s_pulse[i].v = 0.6f + rnd() * 1.2f;
	s_pulse[i].len = 0.8f + rnd() * 2.0f;
}

static void draw_list(const Vtx *v, int n)
{
	for (int i = 0; i < n; i += 6000)   // (vita2d's linear indices)
		vita2d_draw_array(SCE_GXM_PRIMITIVE_TRIANGLES, v + i, n - i < 6000 ? n - i : 6000);
}

static int s_dust_init;
static int lay_changed(float game_x, float game_w, uint32_t accent)
{
	return game_x != s_lay_x || game_w != s_lay_w || accent != s_lay_acc || s_blk < 0 || s_rot != s_lay_rot ||
		memcmp(s_lay_knobs, s_rot ? g_settings.neon_list : g_settings.neon, sizeof(s_lay_knobs));
}

// a new picture width, theme or knob: the colours, the layout and the kept vertices again
static void lay_make(float game_x, float game_w, uint32_t accent)
{
	const int first = s_lay_w < 0;
	memcpy(s_lay_knobs, s_rot ? g_settings.neon_list : g_settings.neon, sizeof(s_lay_knobs));
	s_lay_x = game_x;
	s_lay_w = game_w;
	s_lay_rot = s_rot;
	s_lay_acc = accent;
	s_hue = KN[RBX_NEON_HUE] < 0 ? hue_of(accent) : (float)KN[RBX_NEON_HUE];
	s_sat = s_hue < 0 ? 0.0f : 0.85f;
	if (s_hue < 0)
		s_hue = 0;
	C50 = rgb_hsl(s_hue, 0.50f);
	C55 = rgb_hsl(s_hue, 0.55f);
	C62 = rgb_hsl(s_hue, 0.62f);
	C76 = rgb_hsl(s_hue, 0.76f);
	C82 = rgb_hsl(s_hue, 0.82f);
	P65 = rgb_hsl(s_hue + 25, 0.65f);
	P88 = rgb_hsl(s_hue + 25, 0.88f);
	D80 = rgb_hsl(s_hue - 20, 0.80f);
	layout(game_x, game_w);
	if (first)
		for (int i = 0; i < NPULSE_MAX; i++)
			pulse_new(i, ZN + rnd() * (ZF - ZN));
	static_make();
}

// the moving parts (rings, pulses, dust) at the clock now into v, up to max vertices (v NULL: none made and, as the
// loops below stop at once, the pulses and dust stay where they are); returns the vertices made
static int dyn_make(uint64_t now, Vtx *v, int max)
{
	float dt = s_last ? (now - s_last) / 1e6f : 0;
	s_last = now;
	if (dt > 0.05f)
		dt = 0.05f;
	s_travel = fmodf(s_travel + SPEED * dt * 0.22f, RING * 5.0f);
	s_time += dt;

	s_add = 1;
	s_v = v;
	s_n = 0;
	s_max = s_v ? max : 0;
	// rings: upright on the walls, flowing out to the screen's edges
	int idx = (int)ceilf((ZF + s_travel) / RING);
	for (float z = idx * RING - s_travel; z > ZN * 0.98f && s_v; z -= RING, idx--) {
		if (z > ZF)
			continue;
		const int major = idx % 5 == 0;
		for (int s = -1; s <= 1; s += 2) {
			float tx, ty, bx, by, ix0, iy0, ix1, iy1;
			proj(s * HW, -HH, z, &tx, &ty);
			proj(s * HW, HH, z, &bx, &by);
			proj(s * HW * 0.8f, -HH, z, &ix0, &iy0);
			proj(s * HW * 0.8f, HH, z, &ix1, &iy1);
			const float w8 = side(tx);
			if (w8 < 0.01f)
				continue;
			const float w = fmaxf(0.6f, F / z * HW * (major ? 0.0018f : 0.0009f));
			const float ga = w8 * (major ? 0.14f : 0.05f) * GLOW, ca = w8 * (major ? 0.95f : 0.5f);
			const uint32_t cc = major ? C82 : C62;
			if (ty > 0) {
				seg(ix0, iy0, tx, ty, w * 6, w * 6, C55, ga, ga);
				seg(ix0, iy0, tx, ty, w, w, cc, ca, ca);
			}
			seg(tx, fmaxf(ty, -8), bx, fminf(by, SH + 8), w * 6, w * 6, C55, ga, ga);
			seg(tx, fmaxf(ty, -8), bx, fminf(by, SH + 8), w, w, cc, ca, ca);
			if (by < SH) {
				seg(bx, by, ix1, iy1, w * 6, w * 6, C55, ga, ga);
				seg(bx, by, ix1, iy1, w, w, cc, ca, ca);
			}
		}
	}
	// pulses: light running out along the rails
	for (int i = 0; i < NPULSE && s_v; i++) {
		s_pulse[i].z -= SPEED * s_pulse[i].v * dt * 0.5f;
		if (s_pulse[i].z < ZN * 0.9f)
			pulse_new(i, ZF);
		const float x = s_rail[s_pulse[i].r][0] * HW, y = s_rail[s_pulse[i].r][1] * HH;
		for (int k = 0; k < 8; k++) {
			const float z0 = s_pulse[i].z + s_pulse[i].len * k / 8, z1 = s_pulse[i].z + s_pulse[i].len * (k + 1) / 8;
			float ax, ay, bx, by;
			proj(x, y, z0, &ax, &ay);
			proj(x, y, z1, &bx, &by);
			const float w8 = side(ax) * (1.0f - k / 8.0f);
			if (w8 < 0.01f || ay < -8 || ay > SH + 8)
				continue;
			const float w = fmaxf(1.0f, F / z0 * HW * 0.0025f);
			seg(ax, ay, bx, by, w * 4, w * 4, P65, w8 * 0.2f * (0.3f + GLOW), w8 * 0.2f * (0.3f + GLOW));
			seg(ax, ay, bx, by, w, w, P88, w8, w8);
		}
	}
	// dust drifting out by the walls
	for (int i = 0; i < NDUST && s_v; i++) {
		s_dust[i].t -= dt * SPEED * 0.012f;
		if (s_dust[i].t < 0) {
			s_dust[i].t = 1;
			s_dust[i].fy = rnd() * 2 - 1;
			s_dust[i].fx = 0.75f + rnd() * 0.25f;
		}
		const float z = ZN + (ZM * 1.15f - ZN) * s_dust[i].t;
		float sx, sy;
		proj(s_dust[i].sx * s_dust[i].fx * HW, s_dust[i].fy * HH + sinf(s_time + s_dust[i].fx * 20) * HH * 0.02f, z, &sx, &sy);
		float a = side(sx) * 0.7f;
		if (a < 0.01f || sy < -4 || sy > SH + 4)
			continue;
		float r = fmaxf(0.5f, F / z * HW * 0.0012f);
		if (r < 0.75f) {
			a *= r / 0.75f;
			r = 0.75f;
		}
		const uint32_t c = col(D80, a);
		rect(sx - r, sy - r, sx + r, sy + r, c, c);
	}
	s_add = 0;
	if (s_lay_rot == 1 && s_v)
		swap_xy(s_v, s_n);
	return s_n;
}

// ---- (ADR-181) the moving parts made on a worker on CPU1 while the emulation thread runs the frame ----
// rbx_neon_kick (the frame's start, main.c) posts the frame's job with the clock read then; rbx_neon_draw takes the
// vertices it made. The same code and order of the animation's state (the pulses, the dust, the random numbers) as
// before; the clock is read at the frame's start, not at its drawing (the same intervals). A job the worker has not
// started is done by the drawing itself (as before ADR-181); one it is doing is waited for. A new layout at the
// drawing (the picture's width, the theme, a knob) puts the state back as before the job and makes them there, as
// before. Three vertex buffers of its own (GPU-read): the GPU may still read the last two frames'. The data folder's
// "neonsync": all on the emulation thread as before
#define DYN_BYTES ((3 * DYN_MAX * (int)sizeof(Vtx) + 4095) & ~4095)
static SceUID s_dyn_blk = -1, s_wth = -1, s_wsema = -1;
static Vtx *s_dyn[3];
static int s_dyn_k, s_mode = -1;   // (s_mode: -1 not tried, 0 on the emulation thread, 1 on the worker)
static int s_job;                  // 0 none, 1 posted, 2 being made, 3 made (atomic)
static uint64_t s_job_now;
static int s_job_n;
static Vtx *s_job_v;
static struct {
	__typeof__(s_pulse) pulse;
	__typeof__(s_dust) dust;
	float travel, time;
	uint64_t last;
	uint32_t rand;
} s_snap;

static void job_run(void)
{
	s_job_n = dyn_make(s_job_now, s_job_v, DYN_MAX);
	__atomic_store_n(&s_job, 3, __ATOMIC_RELEASE);
}
static int job_take(int from, int to)
{
	return __atomic_compare_exchange_n(&s_job, &from, to, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}
static int neon_worker(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	for (;;) {   // (lives as long as the app)
		if (sceKernelWaitSema(s_wsema, 1, NULL) < 0)
			return sceKernelExitDeleteThread(0);
		if (job_take(1, 2))
			job_run();
	}
}

#define NEON_PRIORITY 120   // (CPU1: below the sound threads 70-100 and the io worker 72, above the test files' 190)
static int worker_start(void)
{
	SceIoStat st;
	if (sceIoGetstat(RBX_DATA_DIR "neonsync", &st) >= 0) {
		RBX_LOG("neon: the corridor on the emulation thread (neonsync)");
		return 0;
	}
	s_dyn_blk = sceKernelAllocMemBlock("rbx_neon_dyn", SCE_KERNEL_MEMBLOCK_TYPE_USER_RW_UNCACHE, DYN_BYTES, NULL);
	void *p = NULL;
	if (s_dyn_blk < 0 || sceKernelGetMemBlockBase(s_dyn_blk, &p) < 0 || sceGxmMapMemory(p, DYN_BYTES, SCE_GXM_MEMORY_ATTRIB_READ) < 0) {
		RBX_LOG("neon: vertex buffers failed 0x%08x (the corridor on the emulation thread)", s_dyn_blk);
		if (s_dyn_blk >= 0)
			sceKernelFreeMemBlock(s_dyn_blk);
		s_dyn_blk = -1;
		return 0;
	}
	for (int k = 0; k < 3; k++)
		s_dyn[k] = (Vtx *)p + k * DYN_MAX;
	s_wsema = sceKernelCreateSema("rbx_neon", 0, 0, 1, NULL);
	s_wth = s_wsema < 0 ? -1 : sceKernelCreateThread("rbx_neon", neon_worker, NEON_PRIORITY, 0x4000, 0, SCE_KERNEL_CPU_MASK_USER_1, NULL);
	if (s_wth < 0 || sceKernelStartThread(s_wth, 0, NULL) < 0) {
		RBX_LOG("neon: worker failed 0x%08x (the corridor on the emulation thread)", s_wth);
		return 0;   // (the buffers stay unused)
	}
	RBX_LOG("neon: the corridor's moving parts on CPU1, priority %d", NEON_PRIORITY);
	return 1;
}

void rbx_neon_kick(float game_x, float game_w, uint32_t accent)
{
	if (s_mode == 0 || !s_static || !s_dust_init || __atomic_load_n(&s_job, __ATOMIC_ACQUIRE) != 0 ||
		lay_changed(game_x, game_w, accent))
		return;
	if (s_mode < 0 && !(s_mode = worker_start()))
		return;
	memcpy(s_snap.pulse, s_pulse, sizeof(s_pulse));
	memcpy(s_snap.dust, s_dust, sizeof(s_dust));
	s_snap.travel = s_travel;
	s_snap.time = s_time;
	s_snap.last = s_last;
	s_snap.rand = s_rand;
	s_job_now = rbx_time_us();
	s_dyn_k = (s_dyn_k + 1) % 3;
	s_job_v = s_dyn[s_dyn_k];
	__atomic_store_n(&s_job, 1, __ATOMIC_RELEASE);
	sceKernelSignalSema(s_wsema, 1);
}

void rbx_neon_draw(float game_x, float game_w, uint32_t accent)
{
	if (!s_dust_init) {
		s_dust_init = 1;
		for (int i = 0; i < NDUST_MAX; i++) {
			s_dust[i].sx = rnd() < 0.5f ? -1.0f : 1.0f;
			s_dust[i].fx = 0.75f + rnd() * 0.25f;
			s_dust[i].fy = rnd() * 2 - 1;
			s_dust[i].t = rnd();
		}
	}
	int have = 0, redo = 0;   // (redo: the job undone for a new layout; made again at its clock)
	const Vtx *dv = NULL;
	int dn = 0;
	if (__atomic_load_n(&s_job, __ATOMIC_ACQUIRE) != 0) {   // (this frame's job: made here if the worker has not begun it)
		if (job_take(1, 2))
			job_run();
		while (__atomic_load_n(&s_job, __ATOMIC_ACQUIRE) != 3)
			;
		have = 1;
		dv = s_job_v;
		dn = s_job_n;
		__atomic_store_n(&s_job, 0, __ATOMIC_RELEASE);
	}
	if (lay_changed(game_x, game_w, accent)) {
		if (have) {   // (the state as before the job: the frame made here after the new layout, as before)
			memcpy(s_pulse, s_snap.pulse, sizeof(s_pulse));
			memcpy(s_dust, s_snap.dust, sizeof(s_dust));
			s_travel = s_snap.travel;
			s_time = s_snap.time;
			s_last = s_snap.last;
			s_rand = s_snap.rand;
			have = 0;
			redo = 1;
		}
		lay_make(game_x, game_w, accent);
	}
	if (!s_static)
		return;
	if (!have) {
		Vtx *v = (Vtx *)vita2d_pool_memalign(DYN_MAX * sizeof(Vtx), sizeof(Vtx));
		dv = v;
		dn = dyn_make(redo ? s_job_now : rbx_time_us(), v, DYN_MAX);
	}

	draw_list(s_static, s_nbg);
	vita2d_set_blend_mode_add(1);
	draw_list(s_static + s_nbg, s_nstatic - s_nbg);
	if (dn)
		draw_list(dv, dn);
	vita2d_set_blend_mode_add(0);
}
