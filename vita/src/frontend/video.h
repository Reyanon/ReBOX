#ifndef RBX_FRONTEND_VIDEO_H
#define RBX_FRONTEND_VIDEO_H

#include <stdint.h>

// Game screen texture (RGB565, 320x240). The renderer draws directly into texture memory (docs/RULES.md ch.3)
//
// Render thread mode (ADR-007): the CPU2 thread draws frame k into texture k%3, and main
// emulates frame N then shows the finished frame N-1 (display latency +1 frame).
// Single-thread mode: alternates two textures, and the core draws directly inside run_frame.
//
// Overlay layers (menu, notices, FPS box, touch buttons; ui.c canvases) are vita2d textures drawn over the game
// picture, or over an empty screen in the game list and the loading screen (rbx_video_present).

// Game draw buffer size and orientation (rbx_core_screen values, RBX_SCREEN_VERTICAL/FLIPPED). Before rbx_video_init. Default 320x240
void rbx_video_set_screen(int w, int h, int flags);
// the part of the buffer shown when a board narrows its screen while running (<= the size given above)
void rbx_video_set_visible(int w, int h);
int rbx_video_init(void);   // per game: the game textures
void rbx_video_fini(void);  // per game: game textures, scanlines, the game's artwork (the overlay layers stay)

// --- single-thread mode ---
// Texture memory and stride (bytes) the core draws into this frame
void *rbx_video_target(int *pitch_bytes);
// Draw the just-rendered texture and advance to the next (between start_drawing and end_drawing)
void rbx_video_draw(void);
// Redraw the last rendered texture (menu background while paused)
void rbx_video_redraw(void);

// --- render thread mode ---
int rbx_video_start_render_thread(void);
void rbx_video_pause_helper(int paused);   // (the CPU3 drawing helper thread idles: another thread helps)
void rbx_video_stop_render_thread(void);
// Call after main finishes frame N: waits until frame N-1 is drawn, then draws that texture
// (between start_drawing and end_drawing). Draws nothing if N == 0.
void rbx_video_draw_rendered(uint64_t frame_n);
// the same wait asleep (the present thread, ADR-195: it shares CPU0 with the emulation thread above it)
void rbx_video_wait_rendered(uint64_t frame_n);
// Call after a game reset before restarting the render thread (frame numbers restart at 0)
void rbx_video_reset_rendered(void);
void rbx_video_dump_rendered(const char *path);

// Apply option changes (aspect ratio, filter, scanlines, rotation) to the textures
void rbx_video_apply_settings(void);
// Outside-screen image (Border = Artwork): user artwork/<game>.png/.jpg/.bmp (else parent set name), else the moving neon
// corridor (neon.c)
void rbx_video_set_art(const char *game, const char *parent);
// The game's marquee (the list's "marquee", else ux0:data/rebox/marquee/<set>.png or .jpg; or the parent's): at both sides of the picture while it runs
void rbx_video_set_marquee(const char *game, const char *parent);
// The game list's preview (ux0:data/rebox/snap/<set>.mp4, .png, .jpg, in that order, or the parent's): full screen over the
// background at 30%, under the rows. NULL: none
void rbx_video_set_preview(const char *game, const char *parent);
int rbx_video_preview_shown(void);
int rbx_video_preview_is_video(void);   // (the preview is a movie, not a still picture)
// the game list hidden over its preview: 0 shown .. 1 gone (the preview then fully opaque)
void rbx_video_set_ui_hide(float h);
// where the game picture was drawn last (screen pixels); 0 before the first game frame
int rbx_video_picture_rect(int *x, int *y, int *w, int *h);
// The touch effect over everything, light added to the picture: in each of RBX_FX_LAYERS slots texture t centred at x, y,
// r from centre to edge, strength a (0: none). Called once a frame while it shows: it goes by itself a few frames after the
// last call
#define RBX_FX_LAYERS 2
struct ReiTex;
void rbx_video_touch_fx(int slot, struct ReiTex *t, float x, float y, float r, float a);
// Vita held turned for the game (Display > Rotate Vertical L / R): game direction = screen direction turned back by this
// (0, 90 or 270 degrees clockwise)
extern int rbx_input_rot;

// Core color conversion callback (r,g,b 0-255 -> RGB565)
uint32_t rbx_video_color(int r, int g, int b);

// --- overlay layers ---
// A texture the CPU writes (RGBA, 0xAABBGGRR, straight alpha): stride in pixels
typedef struct ReiTex {
	void *tex;      // vita2d_texture
	uint32_t *data;
	int stride, w, h;
} ReiTex;
ReiTex *rbx_video_tex(int w, int h);
// 1 when the layers are drawn premultiplied (ui_commit then copies the canvases as they are)
int rbx_video_premul(void);
#define RBX_NLAYER 7   // 0 menu, 1 focus, 2 notices, 3 FPS, 4 touch buttons, 5 / 6 medal cabinet panels (left / right)
// slot 0-4 shows texture t (NULL hides it) at x,y, w x h of it. Slot 4 (touch buttons) is drawn first, then 0-3
// (higher number on top)
void rbx_video_layer(int slot, ReiTex *t, int x, int y, int w, int h);
void rbx_video_layer_alpha(int slot, float a); // opacity (the blinking menu focus)
void rbx_video_layer_rot(int slot, int rot);   // turned clockwise by rot * 90 degrees (the FPS box)
// strips drawn right over slot 0 (the game list rows: drawn once each, moved by the GPU while the list scrolls), cut to
// rows y0..y1 of the screen. n 0 removes them
#define RBX_NSTRIP 48
typedef struct { const ReiTex *t; int x, y; } ReiStrip;
void rbx_video_strips(const ReiStrip *s, int n, int y0, int y1);
void rbx_video_strips_keep(int save);   // save: the strips set aside; 0: put back (a screen over the list or the menu)
// the game list's opaque background under slot 0, the list's still preview between them (t: SCREEN_H rows, stretched
// across). Set while slot 0 holds the list with a see-through background (its text stays opaque); NULL: the preview over
// slot 0 as before
void rbx_video_list_under(const ReiTex *t);
void rbx_video_draw_layers(void);              // between start_drawing and end_drawing
void rbx_video_kick_border(void);               // (ADR-181) the frame's start: the corridor's moving parts on a worker
// one frame of only the layers (game list, loading screen): waits for vblank
void rbx_video_present(void);
// rbx_video_present with a system dialog (the on-screen keyboard) drawn over it
void rbx_video_present_dialog(void);
// the app's LiveArea background as RGBA pixels (malloc'd, NULL if missing): the loading screen
uint8_t *rbx_video_load_bg(int *w, int *h);
void rbx_video_splash(void);
// the game list's snap in a box (screen px), a its opacity, zoom 1 filling the box / 0 all of it shown, cut to rows
// clip_y0..clip_y1; w <= 0: none
void rbx_video_snap_box(float x, float y, float w, float h, float a, float zoom, int clip_y0, int clip_y1);
// the game list's Info: texture t (full screen) shown between rows y0 and y1 at opacity a over everything, the snap's box
// over it; t NULL or a 0: none
void rbx_video_info_overlay(const ReiTex *t, int y0, int y1, float a);
// the game list: the neon corridor under its background (1) or not
void rbx_video_list_neon(int on);
// the game list's rows (the canvas between y0 and y1, the strips, the focus) faded out by h (0..1): the Info over them
void rbx_video_list_hide(float h, int y0, int y1);
// the list's corridor: its walls' rows (turned: up and down), the gap in the middle of the screen (settings)
#define NEON_LIST_H ((float)g_settings.neon_list_gap)
#define NEON_LIST_Y (272.0f - NEON_LIST_H * 0.5f)   // the LiveArea background as the start splash (after vita2d_init)

#endif
