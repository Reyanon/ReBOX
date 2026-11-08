#include "frontend/video.h"
#include "frontend/avsnap.h"

#include <math.h>
#include <png.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <psp2/kernel/cpu.h>
#include <psp2/kernel/threadmgr.h>
#include <vita2d.h>

#include "frontend/menu.h"
#include "frontend/neon.h"
#include "frontend/settings.h"
#include "frontend/core_api.h"
#include "platform/log.h"
#include "frontend/threads.h"
#include "platform/perf.h"

#define SCREEN_W 960
#define SCREEN_H 544
#define NTEX     3

#define RENDER_PRIORITY   80
#define RENDER_STACK_SIZE (256 * 1024)

static vita2d_texture *s_tex[NTEX];
// Game draw buffer size and orientation (M92 320x240, other boards rbx_core_screen). Vertical games show the buffer rotated
static int s_gw = RBX_SCREEN_W, s_gh = RBX_SCREEN_H, s_gflags;
// quarter turns clockwise the buffer is shown with: the game's monitor (vertical, flipped) and Display > Rotate
static int s_rot;
int rbx_input_rot;
#define DISP_W (s_rot & 1 ? s_gh : s_gw) // visible game screen (after rotation)
#define DISP_H (s_rot & 1 ? s_gw : s_gh)
static void update_rotation(void)
{
	const int vert = (s_gflags & RBX_SCREEN_VERTICAL) != 0, flip = (s_gflags & RBX_SCREEN_FLIPPED) != 0;
	// vertical 90 degrees counter-clockwise, flipped 180, both 90 clockwise
	const int base = vert ? (flip ? 1 : 3) : (flip ? 2 : 0);
	rbx_input_rot = 0;
	switch (g_settings.rotate) {
	case RBX_ROT_OFF: s_rot = flip ? 2 : 0; break; // (the monitor's quarter turn left out, its flip kept)
	case RBX_ROT_VL: s_rot = (base + 1) & 3; rbx_input_rot = 90; break;
	case RBX_ROT_VR: s_rot = (base + 3) & 3; rbx_input_rot = 270; break;
	default: s_rot = base; break;
	}
}

void rbx_video_set_screen(int w, int h, int flags)
{
	s_gw = w;
	s_gh = h;
	s_gflags = flags;
}
static void free_art(void);
static int s_pic_ok; // a game picture was drawn (rbx_video_picture_rect)
static float s_pic_x, s_pic_y, s_pic_w, s_pic_h;
static int s_cur; // texture to draw this time in single-thread mode

static SceUID s_render = -1;
static volatile int s_render_running;
static uint64_t s_rendered; // frames fully drawn by the render thread (updated atomically)

int rbx_video_init(void)
{
	for (int i = 0; i < NTEX; i++) {
		s_tex[i] = vita2d_create_empty_texture_format(s_gw, s_gh,
			SCE_GXM_TEXTURE_FORMAT_U5U6U5_RGB);
		if (!s_tex[i])
			return -1;
	}
	rbx_video_apply_settings();
	s_cur = 0;
	return 0;
}

// Scanlines: an 8px-wide translucent texture overlaid on the game screen.
// - CRT Soft : averages, over the span each screen line covers, a cos curve darkest at game line borders.
//              Even with one game line = 2.27 screen pixels, soft borders hide thickness differences
// - CRT Sharp: computes the position within the game line per screen line and darkens the lower half (dot filter)
// - 2/3/4px  : fixed spacing in screen pixels (independent of game lines, fully uniform)
static vita2d_texture *s_scan;
static int s_scan_h, s_scan_type = -1, s_scan_strength = -1, s_scan_dest_h;

// Integer 2x: if 2x does not fit (vertical games), the largest integer scale that fits
static int int_scale(void)
{
	int k = 2;
	while (k > 1 && (DISP_W * k > SCREEN_W || DISP_H * k > SCREEN_H))
		k--;
	return k;
}

static int dest_height(void)
{
	return g_settings.aspect == RBX_ASPECT_INT2X ? DISP_H * int_scale() : SCREEN_H;
}

static void build_scanlines(void)
{
	const int type = (s_rot & 1) ? RBX_SCAN_OFF : g_settings.scan_type, strength = g_settings.scan_strength, dh = dest_height();
	if (type == s_scan_type && strength == s_scan_strength && dh == s_scan_dest_h)
		return;
	if (s_scan) {
		vita2d_wait_rendering_done(); // the GPU may still be reading
		vita2d_free_texture(s_scan);
		s_scan = NULL;
	}
	s_scan_type = type;
	s_scan_strength = strength;
	s_scan_dest_h = dh;
	if (type == RBX_SCAN_OFF) // (a picture turned a quarter has vertical game lines: no overlay, see type above)
		return;

	static const float alpha[RBX_SCAN_STRENGTH_COUNT] = { 70.0f, 120.0f, 175.0f };
	const float a = alpha[strength];
	const int h = dh;
	s_scan = vita2d_create_empty_texture(8, h);
	if (!s_scan)
		return;
	vita2d_texture_set_filters(s_scan, SCE_GXM_TEXTURE_FILTER_POINT, SCE_GXM_TEXTURE_FILTER_POINT);
	s_scan_h = h;

	uint32_t *p = (uint32_t *)vita2d_texture_get_datap(s_scan);
	const unsigned stride = vita2d_texture_get_stride(s_scan) / 4;
	const float pi2 = 6.2831853f;
	for (int y = 0; y < h; y++) {
		float dark = 0.0f; // 0~1
		switch (type) {
		case RBX_SCAN_CRT_SOFT: {
			// Darkness at game line position u: d(u) = (cos(2πu)+1)/2 (1 at line border, 0 at center).
			// Average over [u0,u1] covered by screen line y = (F(u1)-F(u0))/(u1-u0), F(u) = u/2 + sin(2πu)/(4π)
			const float u0 = (float)y * DISP_H / h, u1 = (float)(y + 1) * DISP_H / h;
			const float F0 = u0 * 0.5f + sinf(pi2 * u0) / (2.0f * pi2);
			const float F1 = u1 * 0.5f + sinf(pi2 * u1) / (2.0f * pi2);
			dark = (F1 - F0) / (u1 - u0);
			break;
		}
		case RBX_SCAN_CRT_SHARP: {
			float pos = (y + 0.5f) * DISP_H / h;
			dark = (pos - (int)pos) >= 0.5f ? 1.0f : 0.0f;
			break;
		}
		case RBX_SCAN_PX2: dark = (y % 2) == 1; break;
		case RBX_SCAN_PX3: dark = (y % 3) == 2; break;
		case RBX_SCAN_PX4: dark = (y % 4) == 3; break;
		}
		const uint32_t c = RGBA8(0, 0, 0, (uint8_t)(a * dark + 0.5f));
		for (int x = 0; x < 8; x++)
			p[y * stride + x] = c;
	}
}

void rbx_video_apply_settings(void)
{
	update_rotation();
	SceGxmTextureFilter f = (g_settings.filter == RBX_FILTER_POINT)
		? SCE_GXM_TEXTURE_FILTER_POINT : SCE_GXM_TEXTURE_FILTER_LINEAR;
	for (int i = 0; i < NTEX; i++)
		if (s_tex[i])
			vita2d_texture_set_filters(s_tex[i], f, f);
	build_scanlines();
}

void rbx_video_fini(void)
{
	rbx_video_stop_render_thread();
	vita2d_wait_rendering_done();
	for (int i = 0; i < NTEX; i++) {
		if (s_tex[i])
			vita2d_free_texture(s_tex[i]);
		s_tex[i] = NULL;
	}
	if (s_scan)
		vita2d_free_texture(s_scan);
	s_scan = NULL;
	s_scan_type = s_scan_strength = s_scan_dest_h = -1;
	free_art();
	s_pic_ok = 0;
}

// --- Border outside the screen (Display > Border) ---
// Artwork: user-supplied full-screen background ux0:data/rebox/artwork/<game>.png (.jpg, .bmp), stretched to the screen.
// <game> = ROM set name, else the parent set name. No artwork ships with the app (copyright). Without it, solid Border Color
static vita2d_texture *s_art;

static vita2d_texture *load_art_file(const char *name)
{
	char path[160];
	snprintf(path, sizeof(path), RBX_DATA_DIR "artwork/%s.png", name);
	vita2d_texture *t = vita2d_load_PNG_file(path);
	if (!t) {
		snprintf(path, sizeof(path), RBX_DATA_DIR "artwork/%s.jpg", name);
		t = vita2d_load_JPEG_file(path);
	}
	if (!t) {
		snprintf(path, sizeof(path), RBX_DATA_DIR "artwork/%s.bmp", name);
		t = vita2d_load_BMP_file(path);
	}
	return t;
}

static uint8_t *load_png_rgba(const char *path, int *w, int *h)
{
	png_image img;
	memset(&img, 0, sizeof(img));
	img.version = PNG_IMAGE_VERSION;
	if (!png_image_begin_read_from_file(&img, path))
		return NULL;
	img.format = PNG_FORMAT_RGBA;
	uint8_t *buf = (uint8_t *)malloc(PNG_IMAGE_SIZE(img));
	if (!buf || !png_image_finish_read(&img, NULL, buf, 0, NULL)) {
		free(buf);
		png_image_free(&img);
		return NULL;
	}
	*w = (int)img.width;
	*h = (int)img.height;
	return buf;
}

// an image of a folder of the data dir, the game's or else its parent's
static vita2d_texture *load_named(const char *dir, const char *name)
{
	char path[160];
	snprintf(path, sizeof(path), RBX_DATA_DIR "%s/%s.png", dir, name);
	vita2d_texture *t = vita2d_load_PNG_file(path);
	if (!t) {
		snprintf(path, sizeof(path), RBX_DATA_DIR "%s/%s.jpg", dir, name);
		t = vita2d_load_JPEG_file(path);
	}
	return t;
}
static vita2d_texture *load_game_image(const char *dir, const char *game, const char *parent)
{
	vita2d_texture *t = game ? load_named(dir, game) : NULL;
	if (!t && game && parent)
		t = load_named(dir, parent);
	if (t)
		vita2d_texture_set_filters(t, SCE_GXM_TEXTURE_FILTER_LINEAR, SCE_GXM_TEXTURE_FILTER_LINEAR);
	return t;
}
static vita2d_texture *s_marquee;
void rbx_video_set_marquee(const char *game, const char *parent)
{
	if (s_marquee) {
		vita2d_wait_rendering_done();
		vita2d_free_texture(s_marquee);
	}
	s_marquee = load_game_image("marquees", game, parent);
}
void rbx_video_set_preview(const char *game, const char *parent) { rbx_snap_request(game, parent); }
int rbx_video_preview_shown(void) { return rbx_snap_kind() != RBX_SNAP_NONE && rbx_snap_fade() > 0.0f; }
// the marquee at both sides of the picture, turned along them: its top towards the picture
static void draw_marquee(void)
{
	if (!s_marquee || s_pic_x < 32)
		return;
	const float w = (float)vita2d_texture_get_width(s_marquee), h = (float)vita2d_texture_get_height(s_marquee);
	const float col = (float)s_pic_x * 0.9f, sa = col / h, sb = (float)SCREEN_H * 0.9f / w, sc = sa < sb ? sa : sb;
	const float xr = (float)(s_pic_x + s_pic_w), cl = (float)s_pic_x * 0.5f, cr = xr + ((float)SCREEN_W - xr) * 0.5f;
	vita2d_draw_texture_scale_rotate(s_marquee, cl, SCREEN_H * 0.5f, sc, sc, 1.5707963f);
	vita2d_draw_texture_scale_rotate(s_marquee, cr, SCREEN_H * 0.5f, sc, sc, -1.5707963f);
}

void rbx_video_set_art(const char *game, const char *parent)
{
	free_art();
	s_art = load_art_file(game);
	if (!s_art && parent)
		s_art = load_art_file(parent);
	if (s_art)
		vita2d_texture_set_filters(s_art, SCE_GXM_TEXTURE_FILTER_LINEAR, SCE_GXM_TEXTURE_FILTER_LINEAR);
	if (!s_art)
		RBX_LOG("video: %s artwork missing (neon corridor)", game);
}

static void free_art(void)
{
	if (s_art) {
		vita2d_wait_rendering_done();
		vita2d_free_texture(s_art);
	}
	s_art = NULL;
}

static unsigned int border_color(void)
{
	static const unsigned int c[RBX_BCOL_COUNT] = {
		RGBA8(0x14, 0x18, 0x3C, 0xFF), RGBA8(0x28, 0x28, 0x2C, 0xFF), RGBA8(0x48, 0x10, 0x14, 0xFF),
		RGBA8(0x10, 0x34, 0x1C, 0xFF), RGBA8(0x2C, 0x14, 0x48, 0xFF), RGBA8(0x5A, 0x24, 0x18, 0xFF),
		RGBA8(0x00, 0x00, 0x00, 0xFF), RGBA8(0x18, 0x18, 0x1A, 0xFF), RGBA8(0x80, 0x84, 0x8C, 0xFF),   // black, charcoal, silver
		RGBA8(0xE8, 0xE8, 0xE8, 0xFF), RGBA8(0x14, 0x30, 0x80, 0xFF), RGBA8(0x30, 0x70, 0xA8, 0xFF),   // white, blue, sky
		RGBA8(0x0C, 0x4C, 0x50, 0xFF), RGBA8(0x48, 0x48, 0x14, 0xFF), RGBA8(0x0C, 0x24, 0x10, 0xFF),   // teal, olive, forest
		RGBA8(0x40, 0x28, 0x14, 0xFF), RGBA8(0x90, 0x48, 0x10, 0xFF), RGBA8(0x8C, 0x70, 0x18, 0xFF),   // brown, orange, gold
		RGBA8(0x80, 0x10, 0x10, 0xFF), RGBA8(0x30, 0x08, 0x14, 0xFF), RGBA8(0x9C, 0x40, 0x6C, 0xFF),   // red, wine, pink
		RGBA8(0x20, 0x10, 0x58, 0xFF),                                                                  // indigo
	};
	return c[g_settings.border_color];
}

// Fill outside the game area. Full screen has no outside
static void draw_border(void)
{
	if (g_settings.border == RBX_BORDER_BLACK || g_settings.aspect == RBX_ASPECT_FULL)
		return;
	if (g_settings.border == RBX_BORDER_ARTWORK && s_art)   // (Graphic: the game's artwork, else the moving corridor)
		vita2d_draw_texture_scale(s_art, 0, 0, (float)SCREEN_W / vita2d_texture_get_width(s_art),
			(float)SCREEN_H / vita2d_texture_get_height(s_art));
	else if (g_settings.border == RBX_BORDER_ARTWORK)
		rbx_neon_draw(s_pic_x, s_pic_w, rbx_menu_accent()); // (no artwork of its own: the moving corridor)
	else
		vita2d_draw_rectangle(0, 0, SCREEN_W, SCREEN_H, border_color());
	if (g_settings.border == RBX_BORDER_MARQUEE)   // (the marquee on the border color; Graphic and Color without it)
		draw_marquee();
}

static void draw_scaled(vita2d_texture *tex)
{
	// Fit the visible game screen (DISP_W x DISP_H, after rotation) to w x h per the aspect setting
	float w, h;
	switch (g_settings.aspect) {
	case RBX_ASPECT_FULL: // stretch to full screen
		w = SCREEN_W;
		h = SCREEN_H;
		break;
	case RBX_ASPECT_INT2X: // integer scale (usually 2x, 640x480)
		w = (float)DISP_W * int_scale();
		h = (float)DISP_H * int_scale();
		break;
	default: // fit height 544 and keep 4:3 (3:4 when turned a quarter) -> 725x544 / 408x544
		h = SCREEN_H;
		w = h * ((s_rot & 1) ? 3.0f / 4.0f : 4.0f / 3.0f);
		break;
	}
	const float x = (SCREEN_W - w) * 0.5f, y = (SCREEN_H - h) * 0.5f;
	s_pic_x = x;
	s_pic_y = y;
	s_pic_w = w;
	s_pic_h = h;
	s_pic_ok = 1;
	draw_border();
	if (!s_rot) {
		vita2d_draw_texture_part_scale(tex, x, y, 0, 0, s_gw, s_gh, w / s_gw, h / s_gh);
	} else {
		// turned clockwise by s_rot quarters around the picture centre; turned a quarter, the buffer width is the
		// screen height
		const float rad = 1.5707963f * s_rot;
		const float sx = (s_rot & 1) ? h / s_gw : w / s_gw, sy = (s_rot & 1) ? w / s_gh : h / s_gh;
		vita2d_draw_texture_part_scale_rotate(tex, x + w * 0.5f, y + h * 0.5f, 0, 0, s_gw, s_gh, sx, sy, rad);
	}
	if (s_scan)
		vita2d_draw_texture_scale(s_scan, x, y, w / 8.0f, h / (float)s_scan_h);
}

int rbx_video_picture_rect(int *x, int *y, int *w, int *h)
{
	if (!s_pic_ok)
		return 0;
	*x = (int)s_pic_x;
	*y = (int)s_pic_y;
	*w = (int)s_pic_w;
	*h = (int)s_pic_h;
	return 1;
}

void *rbx_video_target(int *pitch_bytes)
{
	*pitch_bytes = (int)vita2d_texture_get_stride(s_tex[s_cur]);
	return vita2d_texture_get_datap(s_tex[s_cur]);
}

void rbx_video_draw(void)
{
	draw_scaled(s_tex[s_cur]);
	s_cur = (s_cur + 1) % 2;
}

void rbx_video_redraw(void)
{
	draw_scaled(s_tex[(s_cur + 1) % 2]);
}

static int render_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	uint64_t k = 0;
	extern SceUID g_render_tid;
	extern volatile uint32_t g_core_wait_us[3];
	g_render_tid = sceKernelGetThreadId();
	while (s_render_running) {
		vita2d_texture *tex = s_tex[k % NTEX];
		uint64_t t0 = rbx_time_us();
		// Wait for frame k recorded by main, then draw it to the texture (including wait time)
		if (!rbx_core_render_frame(vita2d_texture_get_datap(tex), (int)vita2d_texture_get_stride(tex)))
			break;
		{   // busy time: without the wait for the frame
			const uint32_t w = g_core_wait_us[1], all = (uint32_t)(rbx_time_us() - t0);
			g_core_wait_us[1] = 0;
			rbx_perf_record(RBX_PERF_RENDER, all > w ? all - w : 0);
		}
		k++;
		__atomic_store_n(&s_rendered, k, __ATOMIC_RELEASE);
	}
	return sceKernelExitDeleteThread(0);
}

// CPU3 (shared with the system, about half a core for applications: ReST-V ADR-158) helps draw the frames of boards
// that split their drawing (rbx_render_help); it polls while there is nothing to do. The lowest user priority: at the
// render thread's priority it held the system off CPU3 and the whole Vita froze (2026-10-31)
#define CPU_MASK_USER_3 0x00080000
#define HELPER_PRIORITY 191
static SceUID s_helper = -1;
static volatile int s_helper_running, s_helper_paused;
static int helper_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	while (s_helper_running)
		if (s_helper_paused || !rbx_render_help())
			sceKernelDelayThread(500);
	return sceKernelExitDeleteThread(0);
}

// (another thread of CPU3 does the helping: threads.c rbx_aux3)
void rbx_video_pause_helper(int paused) { s_helper_paused = paused; }

int rbx_video_start_render_thread(void)
{
	rbx_set_render_thread(0); // (clears a stop and the frame counts of the last run)
	rbx_set_render_thread(1);
	s_render_running = 1;
	s_render = sceKernelCreateThread("rbx_render", render_thread, RENDER_PRIORITY, RENDER_STACK_SIZE,
		0, SCE_KERNEL_CPU_MASK_USER_2, NULL);
	if (s_render < 0) {
		RBX_LOG("video: render thread creation failed 0x%08x", s_render);
		s_render_running = 0;
		return -1;
	}
	sceKernelStartThread(s_render, 0, NULL);
	RBX_LOG("video: render thread started on CPU2");
	if (!rbx_render_can_help())
		return 0;
	// (CPU3 closed (a stock system): no helper. On CPU1 below the sound threads the bands it took came late and the
	// render thread waited for them: moomesa 60 -> 45 fps; the render thread and the waiting emulation draw them)
	if (!rbx_cpu3_usable()) {
		RBX_LOG("video: drawing helper off (CPU3 closed: the bands by the render thread and the waiting emulation)");
		return 0;
	}
	s_helper_running = 1;
	s_helper = sceKernelCreateThread("rbx_rhelp", helper_thread, HELPER_PRIORITY, 256 * 1024, 0, CPU_MASK_USER_3, NULL);
	if (s_helper >= 0) {
		sceKernelStartThread(s_helper, 0, NULL);
		RBX_LOG("video: drawing helper started on CPU3");
	} else {
		RBX_LOG("video: drawing helper creation failed 0x%08x", s_helper);
		s_helper_running = 0;
	}
	return 0;
}

void rbx_video_stop_render_thread(void)
{
	if (s_render < 0)
		return;
	if (s_helper >= 0) {
		s_helper_running = 0;
		sceKernelWaitThreadEnd(s_helper, NULL, NULL);
		s_helper = -1;
	}
	s_render_running = 0;
	rbx_core_stop();
	sceKernelWaitThreadEnd(s_render, NULL, NULL);
	s_render = -1;
	rbx_set_render_thread(0);
}

// the last frame the render thread drew, as a PPM (log builds: switchtest play check)
void rbx_video_dump_rendered(const char *path)
{
	const uint64_t n = __atomic_load_n(&s_rendered, __ATOMIC_ACQUIRE);
	if (!n)
		return;
	vita2d_texture *t = s_tex[(n - 1) % NTEX];
	const uint16_t *px = (const uint16_t *)vita2d_texture_get_datap(t);
	const int stride = (int)vita2d_texture_get_stride(t) / 2;
	FILE *f = fopen(path, "wb");
	if (!f)
		return;
	fprintf(f, "P6 %d %d 255 ", s_gw, s_gh);
	uint8_t *row = (uint8_t *)malloc((size_t)s_gw * 3);
	for (int y = 0; row && y < s_gh; y++) {
		for (int x = 0; x < s_gw; x++) {
			const uint16_t c = px[y * stride + x];
			row[x * 3] = (uint8_t)((c >> 11) << 3);
			row[x * 3 + 1] = (uint8_t)(((c >> 5) & 63) << 2);
			row[x * 3 + 2] = (uint8_t)((c & 31) << 3);
		}
		fwrite(row, 3, (size_t)s_gw, f);
	}
	free(row);
	fclose(f);
}

void rbx_video_reset_rendered(void)
{
	__atomic_store_n(&s_rendered, 0, __ATOMIC_RELEASE);
}

void rbx_video_draw_rendered(uint64_t frame_n)
{
	if (frame_n == 0)
		return;
	// Wait until frame N-1 (= render sequence N-1) finishes. Usually already done
	while (__atomic_load_n(&s_rendered, __ATOMIC_ACQUIRE) < frame_n && s_render_running)
		;
	draw_scaled(s_tex[(frame_n - 1) % NTEX]);
}

uint32_t rbx_video_color(int r, int g, int b)
{
	return ((uint32_t)(r >> 3) << 11) | ((uint32_t)(g >> 2) << 5) | (uint32_t)(b >> 3);
}

// --- overlay layers ---
ReiTex *rbx_video_tex(int w, int h)
{
	vita2d_texture *t = vita2d_create_empty_texture(w, h);
	if (!t)
		return NULL;
	vita2d_texture_set_filters(t, SCE_GXM_TEXTURE_FILTER_POINT, SCE_GXM_TEXTURE_FILTER_POINT);
	ReiTex *r = (ReiTex *)calloc(1, sizeof(ReiTex));
	if (!r) {
		vita2d_free_texture(t);
		return NULL;
	}
	r->tex = t;
	r->data = (uint32_t *)vita2d_texture_get_datap(t);
	r->stride = (int)vita2d_texture_get_stride(t) / 4;
	r->w = w;
	r->h = h;
	memset(r->data, 0, (size_t)r->stride * h * 4);
	return r;
}

static struct { ReiTex *t; int x, y, w, h, rot; float a; } s_layer[RBX_NLAYER] = { { .a = 1 }, { .a = 1 }, { .a = 1 }, { .a = 1 },
	{ .a = 1 }, { .a = 1 }, { .a = 1 } };

// The canvases' textures hold premultiplied colours (ui.c paints that way), drawn with blend (ONE, ONE_MINUS_SRC_ALPHA):
// vita2d's texture programs again with that blend, put in place of vita2d's while the layers are drawn. (Before, ui_commit
// divided each pixel by its alpha for vita2d's straight-alpha blend: ~38 ms for a whole canvas on the device, the menu
// stuttered)
extern SceGxmFragmentProgram *_vita2d_textureFragmentProgram, *_vita2d_textureTintFragmentProgram;
extern const SceGxmProgram texture_f_gxp_start, texture_tint_f_gxp_start, texture_v_gxp_start;
static SceGxmFragmentProgram *s_pm_tex, *s_pm_tint, *s_keep_tex, *s_keep_tint;
static int s_pm_tried;
static void premul_begin(void)
{
	if (!s_pm_tried) {
		s_pm_tried = 1;
		SceGxmShaderPatcher *sp = vita2d_get_shader_patcher();
		SceGxmShaderPatcherId id_tex, id_tint;
		const SceGxmBlendInfo b = { .colorMask = SCE_GXM_COLOR_MASK_ALL, .colorFunc = SCE_GXM_BLEND_FUNC_ADD,
			.alphaFunc = SCE_GXM_BLEND_FUNC_ADD, .colorSrc = SCE_GXM_BLEND_FACTOR_ONE,
			.colorDst = SCE_GXM_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, .alphaSrc = SCE_GXM_BLEND_FACTOR_ONE,
			.alphaDst = SCE_GXM_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA };
		if (sceGxmShaderPatcherRegisterProgram(sp, &texture_f_gxp_start, &id_tex) < 0 ||
			sceGxmShaderPatcherRegisterProgram(sp, &texture_tint_f_gxp_start, &id_tint) < 0 ||
			sceGxmShaderPatcherCreateFragmentProgram(sp, id_tex, SCE_GXM_OUTPUT_REGISTER_FORMAT_UCHAR4,
				SCE_GXM_MULTISAMPLE_NONE, &b, &texture_v_gxp_start, &s_pm_tex) < 0 ||
			sceGxmShaderPatcherCreateFragmentProgram(sp, id_tint, SCE_GXM_OUTPUT_REGISTER_FORMAT_UCHAR4,
				SCE_GXM_MULTISAMPLE_NONE, &b, &texture_v_gxp_start, &s_pm_tint) < 0) {
			s_pm_tex = s_pm_tint = NULL;
			RBX_LOG("video: premultiplied programs failed");
		}
	}
	s_keep_tex = _vita2d_textureFragmentProgram;
	s_keep_tint = _vita2d_textureTintFragmentProgram;
	if (s_pm_tex) {
		_vita2d_textureFragmentProgram = s_pm_tex;
		_vita2d_textureTintFragmentProgram = s_pm_tint;
	}
}
static void premul_end(void)
{
	_vita2d_textureFragmentProgram = s_keep_tex;
	_vita2d_textureTintFragmentProgram = s_keep_tint;
}
int rbx_video_premul(void)
{
	if (!s_pm_tried) { // (made at the first upload: every upload depends on them)
		premul_begin();
		premul_end();
	}
	return s_pm_tex != NULL;
}

void rbx_video_layer(int slot, ReiTex *t, int x, int y, int w, int h)
{
	if (slot < 0 || slot >= RBX_NLAYER)
		return;
	s_layer[slot].t = t;
	s_layer[slot].x = x;
	s_layer[slot].y = y;
	s_layer[slot].w = w;
	s_layer[slot].h = h;
}

void rbx_video_layer_alpha(int slot, float a)
{
	if (slot >= 0 && slot < RBX_NLAYER)
		s_layer[slot].a = a;
}

void rbx_video_layer_rot(int slot, int rot)
{
	if (slot >= 0 && slot < RBX_NLAYER)
		s_layer[slot].rot = rot & 3;
}

static ReiStrip s_strip[RBX_NSTRIP];
static int s_nstrip, s_strip_y0, s_strip_y1;
void rbx_video_strips(const ReiStrip *s, int n, int y0, int y1)
{
	if (n > RBX_NSTRIP)
		n = RBX_NSTRIP;
	for (int i = 0; i < n; i++)
		s_strip[i] = s[i];
	s_nstrip = n;
	s_strip_y0 = y0;
	s_strip_y1 = y1;
}

static void draw_strips(void)
{
	// (over a preview the rows are see-through: 80% over a video, 90% over a picture)
	const int a8 = rbx_video_preview_shown() ? 255 - (int)((rbx_snap_kind() == RBX_SNAP_VIDEO ? 51.0f : 26.0f) * rbx_snap_fade()) : 255;
	for (int i = 0; i < s_nstrip; i++) {
		const ReiStrip *s = &s_strip[i];
		const int y0 = s->y > s_strip_y0 ? s->y : s_strip_y0, y1 = s->y + s->t->h < s_strip_y1 ? s->y + s->t->h : s_strip_y1;
		if (y1 <= y0)
			continue;
		if (a8 < 255)
			vita2d_draw_texture_tint_part((vita2d_texture *)s->t->tex, s->x, y0, 0, y0 - s->y, s->t->w, y1 - y0,
				RGBA8(a8, a8, a8, a8));
		else
			vita2d_draw_texture_part((vita2d_texture *)s->t->tex, s->x, y0, 0, y0 - s->y, s->t->w, y1 - y0);
	}
}

static struct { ReiTex *t; float x, y, r, a; } s_fx[RBX_FX_LAYERS];
static int s_fx_ttl; // frames left: the menu calling this once a frame keeps it (a closed menu leaves nothing behind)
void rbx_video_touch_fx(int slot, ReiTex *t, float x, float y, float r, float a)
{
	if (slot < 0 || slot >= RBX_FX_LAYERS)
		return;
	s_fx[slot].t = t;
	s_fx[slot].x = x;
	s_fx[slot].y = y;
	s_fx[slot].r = r;
	s_fx[slot].a = a;
	s_fx_ttl = 3;
}
// the touch effect, over everything: light added to the picture (ONE, ONE of a premultiplied texture: its corners add
// nothing). vita2d's own additive mode goes by the texture's alpha and the tint program does not follow it
static SceGxmFragmentProgram *s_add_tint;
static void draw_touch_fx(void)
{
	if (s_fx_ttl <= 0 || !--s_fx_ttl)
		return;
	static int tried;
	if (!tried) {
		tried = 1;
		SceGxmShaderPatcher *sp = vita2d_get_shader_patcher();
		SceGxmShaderPatcherId id;
		const SceGxmBlendInfo b = { .colorMask = SCE_GXM_COLOR_MASK_ALL, .colorFunc = SCE_GXM_BLEND_FUNC_ADD,
			.alphaFunc = SCE_GXM_BLEND_FUNC_ADD, .colorSrc = SCE_GXM_BLEND_FACTOR_ONE, .colorDst = SCE_GXM_BLEND_FACTOR_ONE,
			.alphaSrc = SCE_GXM_BLEND_FACTOR_ZERO, .alphaDst = SCE_GXM_BLEND_FACTOR_ONE };
		if (sceGxmShaderPatcherRegisterProgram(sp, &texture_tint_f_gxp_start, &id) < 0 ||
			sceGxmShaderPatcherCreateFragmentProgram(sp, id, SCE_GXM_OUTPUT_REGISTER_FORMAT_UCHAR4, SCE_GXM_MULTISAMPLE_NONE, &b,
				&texture_v_gxp_start, &s_add_tint) < 0) {
			s_add_tint = NULL;
			RBX_LOG("video: additive program failed");
		}
	}
	if (!s_add_tint)
		return;
	SceGxmFragmentProgram *keep = _vita2d_textureTintFragmentProgram;
	_vita2d_textureTintFragmentProgram = s_add_tint;
	for (int k = 0; k < RBX_FX_LAYERS; k++) {
		if (!s_fx[k].t || s_fx[k].a <= 0.0f)
			continue;
		vita2d_texture *tex = (vita2d_texture *)s_fx[k].t->tex;
		const float sc = 2.0f * s_fx[k].r / (float)s_fx[k].t->w;
		const int a8 = (int)(s_fx[k].a * 255.0f + 0.5f);
		vita2d_draw_texture_tint_scale(tex, s_fx[k].x - s_fx[k].r, s_fx[k].y - s_fx[k].r, sc, sc, RGBA8(a8, a8, a8, a8));
	}
	_vita2d_textureTintFragmentProgram = keep;
}

void rbx_video_draw_layers(void)
{
	static const int order[RBX_NLAYER] = { 5, 6, 4, 0, 1, 2, 3 }; // (medal panels and touch buttons under the menu layers)
	premul_begin();
	for (int o = 0; o < RBX_NLAYER; o++) {
		const int k = order[o];
		vita2d_texture *pv = o == 4 ? rbx_snap_frame() : NULL;
		if (pv) {   // (the list's preview, over its background, under its rows)
			// (video 60%, picture 50%; premultiplied: the tint scales the colour too)
			const int a8 = (int)((rbx_snap_kind() == RBX_SNAP_VIDEO ? 153.0f : 128.0f) * rbx_snap_fade());
			vita2d_draw_texture_tint_scale(pv, 0, 0, (float)SCREEN_W / vita2d_texture_get_width(pv),
				(float)SCREEN_H / vita2d_texture_get_height(pv), RGBA8(a8, a8, a8, a8));
		}
		if (o == 4)
			draw_strips(); // (over slot 0, under the focus outline)
		const ReiTex *t = s_layer[k].t;
		if (!t)
			continue;
		vita2d_texture *tex = (vita2d_texture *)t->tex;
		const float a = s_layer[k].a;
		const int a8 = (int)(a * 255.0f + 0.5f);
		const unsigned int col = RGBA8(a8, a8, a8, a8); // (premultiplied: the tint scales the colour too)
		const int rot = s_layer[k].rot, w = s_layer[k].w, h = s_layer[k].h;
		if (rot) { // turned: on screen h x w after a quarter turn, from x, y
			const float ow = (float)((rot & 1) ? h : w), oh = (float)((rot & 1) ? w : h);
			vita2d_draw_texture_part_tint_scale_rotate(tex, s_layer[k].x + ow * 0.5f, s_layer[k].y + oh * 0.5f, 0, 0, w, h,
				1.0f, 1.0f, 1.5707963f * rot, col);
		} else if (a < 1.0f)
			vita2d_draw_texture_tint_part(tex, s_layer[k].x, s_layer[k].y, 0, 0, w, h, col);
		else
			vita2d_draw_texture_part(tex, s_layer[k].x, s_layer[k].y, 0, 0, w, h);
	}
	premul_end();
	draw_touch_fx();
}

void rbx_video_present(void)
{
	vita2d_start_drawing();
	vita2d_clear_screen();
	rbx_video_draw_layers();
	vita2d_end_drawing();
	vita2d_swap_buffers();
}

void rbx_video_present_dialog(void)
{
	vita2d_start_drawing();
	vita2d_clear_screen();
	rbx_video_draw_layers();
	vita2d_end_drawing();
	vita2d_common_dialog_update();
	vita2d_swap_buffers();
}

uint8_t *rbx_video_load_bg(int *w, int *h)
{
	return load_png_rgba("app0:sce_sys/livearea/contents/bg.png", w, h);
}
