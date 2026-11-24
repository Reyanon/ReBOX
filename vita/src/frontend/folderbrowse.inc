// The folder browser (Settings' ROM / snap / marquee folder rows, user 11-24): a screen of its own over the menu. Included
// by menu.c (its theme, shapes, keys and touch). GPL-2.0-or-later.
//
// The folders are rows drawn once each into small textures and placed by the GPU (rbx_video_strips, as the game list),
// so the list moves at the display's rate: the view eases to keep the cursor in sight, the cursor's bar glides to its
// row, a drag scrolls with the finger and goes on after it with inertia (a soft stop past the ends), the left stick
// scrolls at the speed it is pushed, and a folder opened or left slides in from its side. The header shows the path
// (each part can be tapped) and what the folder holds; the footer's buttons can be tapped too.

#define FB_ROW_H   56
#define FB_X       40
#define FB_W       (SCREEN_W - 2 * FB_X)
#define FB_Y0      108                        // the rows area
#define FB_Y1      (SCREEN_H - 52)
#define FB_VIEW    (FB_Y1 - FB_Y0)
#define FB_NAMES   1024
#define FB_CACHE   24

static char s_fbn[FB_NAMES][64];
static struct { UiCanvas *cv; int idx, var, gen; unsigned used; } s_fbc[FB_CACHE];
static UiCanvas *s_fb_bar, *s_fb_thumb;
static unsigned s_fb_frame;

static const char *fb_device_name(const char *d)
{
	static const char *const dev[][2] = { { "ux0:", "Memory card" }, { "uma0:", "USB / SD2Vita" }, { "ur0:", "System" },
		{ "imc0:", "Internal storage" }, { "xmc0:", "Memory card" }, { "grw0:", "Game card" } };
	for (unsigned i = 0; i < sizeof(dev) / sizeof(dev[0]); i++)
		if (!strcmp(d, dev[i][0]))
			return dev[i][1];
	return "";
}

// a folder (tab and body) or, for a device, a chip with pins
static void fb_icon(UiCanvas *c, int x, int cy, int device, uint32_t col)
{
	if (device) {
		rrect(c, x + 3, cy - 12, 26, 24, 5, col);
		for (int i = 0; i < 4; i++)
			ui_rect(c, x + 7 + i * 5, cy - 16, 2, 4, col);
		rrect(c, x + 9, cy - 6, 14, 12, 3, UI_RGBA(0, 0, 0, 0x50));
		return;
	}
	rrect(c, x, cy - 12, 14, 8, 3, col);
	rrect(c, x, cy - 8, 32, 21, 4, col);
	ui_rect(c, x + 2, cy - 4, 28, 1, UI_RGBA(0xFF, 0xFF, 0xFF, 0x40));
}

// row i's texture: var 0 the row, 1 the row under the cursor
static const ReiTex *fb_row(int i, int var, int device, int gen)
{
	int k = -1;
	for (int j = 0; j < FB_CACHE && k < 0; j++)
		if (s_fbc[j].cv && s_fbc[j].idx == i && s_fbc[j].var == var && s_fbc[j].gen == gen)
			k = j;
	if (k < 0) {
		for (int j = 0; j < FB_CACHE; j++) {
			if (!s_fbc[j].cv) {
				k = j;
				break;
			}
			if (s_fbc[j].used != s_fb_frame && (k < 0 || s_fbc[j].used < s_fbc[k].used))
				k = j;
		}
		if (k < 0)
			return NULL;
		if (!s_fbc[k].cv)
			s_fbc[k].cv = ui_canvas(-1, 0, 0, FB_W, FB_ROW_H);
		if (!s_fbc[k].cv)
			return NULL;
		UiCanvas *c = s_fbc[k].cv;
		const Theme *th = theme();
		ui_clear(c);
		const int cy = FB_ROW_H / 2, ty = cy + 7;
		fb_icon(c, 26, cy, device, var ? COL_SEL : th->toggle_on);
		const uint32_t tc = var ? COL_TEXT : COL_ITEM;
		const int nx = 76;
		int x = nx + ui_text_clip(c, nx, ty, tc, 1.0f, s_fbn[i], nx, FB_W - 70);
		if (device) {
			const char *dn = fb_device_name(s_fbn[i]);
			if (dn[0])
				ui_text_clip(c, x + 16, ty, COL_DIM, 0.85f, dn, x + 16, FB_W - 70);
		}
		// the chevron: this row opens
		const float ax = (float)(FB_W - 40), ay = (float)cy;
		const uint32_t ac = var ? COL_SEL : COL_DIM;
		ui_line(c, ax, ay - 8.0f, ax + 7.0f, ay, 2.4f, ac);
		ui_line(c, ax + 7.0f, ay, ax, ay + 8.0f, 2.4f, ac);
		if (!var)
			ui_rect(c, nx, FB_ROW_H - 1, FB_W - nx - 24, 1, (th->sep & 0x00FFFFFFu) | 0x24000000u);
		ui_commit(c);
		s_fbc[k].idx = i;
		s_fbc[k].var = var;
		s_fbc[k].gen = gen;
	}
	s_fbc[k].used = s_fb_frame;
	return ui_tex(s_fbc[k].cv);
}

typedef struct { int x0, x1, len; } FbCrumb;   // a part of the path in the header (tap: go there), len: of the path
typedef struct { int x0, x1, act; } FbButton;  // a footer button
enum { FBA_OPEN = 1, FBA_BACK, FBA_USE, FBA_NONE };

// the screen under the rows: background, header (title, path, what is here), footer buttons; the empty folder's note
static void fb_paint(UiCanvas *c, const char *title, const char *path, int n, int files, int kind, FbCrumb *cr, int *ncr,
                     FbButton *bt, int *nbt, int ok_key, int back_key)
{
	const Theme *th = theme();
	ui_fill_rows(c, theme_rows(0), 0, 0, SCREEN_W, SCREEN_H);
	ui_rect(c, 0, 0, SCREEN_W, FB_Y0 - 12, (th->band & 0x00FFFFFFu) | 0x90000000u);
	ui_rect(c, 0, FB_Y0 - 12, SCREEN_W, 2, (th->edge & 0x00FFFFFFu) | 0xB0000000u);
	ui_rect(c, 0, FB_Y1 + 8, SCREEN_W, SCREEN_H - FB_Y1 - 8, (th->band & 0x00FFFFFFu) | 0x90000000u);
	ui_rect(c, 0, FB_Y1 + 6, SCREEN_W, 2, (th->edge & 0x00FFFFFFu) | 0xB0000000u);
	// title, and what the folder holds at the right
	fb_icon(c, FB_X, 30, 0, COL_SEL);
	ui_text(c, FB_X + 46, 38, COL_TEXT, 1.0f, title);
	char info[96];
	if (!path[0])
		snprintf(info, sizeof(info), "%s", T("Choose a storage"));
	else if (kind == RBX_FOLDER_ROM1 || kind == RBX_FOLDER_ROM2)
		snprintf(info, sizeof(info), T("%d folders, %d zip files"), n, files);
	else
		snprintf(info, sizeof(info), T("%d folders, %d pictures"), n, files);
	const int iw = ui_text_width(0.85f, info);
	ui_text(c, SCREEN_W - FB_X - iw, 38, files ? COL_SEL : COL_DIM, 0.85f, info);
	// the path as parts: the device, then each folder ("..." for those left out at the start when it is long)
	*ncr = 0;
	{
		int pos[32], len[32], np = 0;
		const char *p = path;
		const char *colon = strchr(path, ':');
		if (colon) {
			pos[np] = 0;
			len[np++] = (int)(colon - path + 1);
			p = colon + 1;
		}
		while (*p && np < 32) {
			const char *e = strchr(p, '/');
			const int l = e ? (int)(e - p) : (int)strlen(p);
			if (l > 0) {
				pos[np] = (int)(p - path);
				len[np++] = l;
			}
			p += l + (e ? 1 : 0);
		}
		const int y = 80, sep = 26, avail = SCREEN_W - 2 * FB_X - 40;
		char part[72];
		int first = np > 0 ? 1 : 0, total;
		for (;;) {   // (from the first part shown to the end: within the width)
			total = 0;
			for (int i = 0; i < np; i++) {
				if (i > 0 && i < first)
					continue;
				snprintf(part, sizeof(part), "%.*s", len[i], path + pos[i]);
				total += ui_text_width(0.9f, part) + (i ? sep : 0) + (i == 0 ? 24 : 0);
			}
			if (first > 1)
				total += ui_text_width(0.9f, "...") + sep;
			if (total <= avail || first >= np - 1)
				break;
			first = first ? first + 1 : 2;
		}
		int x = FB_X;
		for (int i = 0; i < np; i++) {
			if (i > 0 && i < first) {
				if (i == 1) {
					x += ui_text(c, x + sep, y, COL_DIM, 0.9f, "...") + sep;
				}
				continue;
			}
			if (i) {   // the separator: a small chevron
				const float cx = (float)(x + sep / 2 - 3), cy = (float)(y - 6);
				ui_line(c, cx, cy - 5.0f, cx + 5.0f, cy, 1.8f, COL_DIM);
				ui_line(c, cx + 5.0f, cy, cx, cy + 5.0f, 1.8f, COL_DIM);
				x += sep;
			}
			snprintf(part, sizeof(part), "%.*s", len[i], path + pos[i]);
			const int w = ui_text_width(0.9f, part) + (i == 0 ? 24 : 0);
			const int last = i == np - 1;
			if (i == 0)   // (the device as a chip)
				rrect(c, x - 2, y - 21, w, 28, 14, (th->edge & 0x00FFFFFFu) | 0x50000000u);
			ui_text(c, x + (i == 0 ? 10 : 0), y, last ? COL_TEXT : COL_ITEM, 0.9f, part);
			if (*ncr < 32)
				cr[(*ncr)++] = (FbCrumb){ x - 6, x + w + 6, pos[i] + len[i] };
			x += w;
		}
		if (!np)
			ui_text(c, FB_X, y, COL_DIM, 0.9f, T("Storages"));
	}
	if (!n) {   // an empty folder
		const char *m1 = path[0] ? T("No folders here") : T("No storage found");
		ui_text(c, (SCREEN_W - ui_text_width(1.0f, m1)) / 2, FB_Y0 + FB_VIEW / 2 - 10, COL_DIM, 1.0f, m1);
		if (path[0]) {
			char m2[96];
			snprintf(m2, sizeof(m2), T("%c Use this folder"), 0x10);
			const int w2 = ui_text_width(0.85f, m2) + 24;
			text_keys(c, (SCREEN_W - w2) / 2, FB_Y0 + FB_VIEW / 2 + 26, COL_ITEM, m2);
		}
	}
	// footer buttons
	*nbt = 0;
	{
		struct { int key, act; const char *label; } b[4] = {
			{ ok_key, FBA_OPEN, "Open" }, { back_key, FBA_BACK, path[0] ? "Up" : "Cancel" },
			{ 0x10, FBA_USE, "Use this folder" }, { 0x13, FBA_NONE, "Not set" } };
		// (user 11-25: the same gap above and below: the pills centred in the band under its line, the text on their
		// middle; a 0.8 line's middle is 6 px above its baseline)
		const int band0 = FB_Y1 + 8, h = 28, top = band0 + (SCREEN_H - band0 - h) / 2, y = top + h / 2 + 6;
		int x = FB_X - 8;
		for (int i = 0; i < 4; i++) {
			if (b[i].act == FBA_USE && !path[0])
				continue;
			// (the button's icon, then its label: the pill as wide as both with the same margin at each end)
			static const int keys[4] = { RBX_KEY_TRIANGLE, RBX_KEY_CIRCLE, RBX_KEY_CROSS, RBX_KEY_SQUARE };
			const char *label = T(b[i].label);
			const int iw = 20, gap = 6, pad = 12, w = pad + iw + gap + ui_text_width(0.8f, label) + pad;
			foot_button(c, x, top, w, h);
			draw_key(c, x + pad, y - 6, y, keys[b[i].key - 0x10], COL_TEXT);
			ui_text(c, x + pad + iw + gap, y, COL_TEXT, 0.8f, label);
			bt[(*nbt)++] = (FbButton){ x, x + w, b[i].act };
			x += w + 10;
		}
		const char *pg = "L / R  Page";
		ui_text(c, SCREEN_W - FB_X - ui_text_width(0.8f, pg), y, COL_DIM, 0.8f, pg);
	}
	// the scroll bar's track
	if (n * FB_ROW_H > FB_VIEW)
		rrect(c, SCREEN_W - 18, FB_Y0 + 6, 6, FB_VIEW - 12, 3, th->bar_track);
}

// the browser: start at the folder k has (else ux0:). 1 = k set (a folder or none), 0 = left as it was
static int folder_browse(int k)
{
	static const char *const zips[] = { "zip", "7z", NULL }, *const pics[] = { "png", "jpg", "jpeg", NULL };
	static const char *const titles[RBX_FOLDERS] = { "ROM folder 1", "ROM folder 2", "Snap folder", "Marquee folder" };
	const char *const *exts = k == RBX_FOLDER_ROM1 || k == RBX_FOLDER_ROM2 ? zips : pics;
	const int o_ok = g_settings.menu_confirm == RBX_CONFIRM_CIRCLE;
	const uint32_t ok = o_ok ? SCE_CTRL_CIRCLE : SCE_CTRL_CROSS, back = o_ok ? SCE_CTRL_CROSS : SCE_CTRL_CIRCLE;
	const int ok_key = o_ok ? 0x11 : 0x12, back_key = o_ok ? 0x12 : 0x11;
	char path[160];
	snprintf(path, sizeof(path), "%s", rbx_folder(k)[0] ? rbx_folder(k) : "ux0:");
	for (;;) {   // (a folder no longer there: the nearest one above it)
		const SceUID d = path[0] ? sceIoDopen(path) : 0;
		if (d >= 0) {
			if (path[0])
				sceIoDclose(d);
			break;
		}
		size_t l = strlen(path);
		if (l && path[l - 1] == '/')
			path[--l] = 0;
		while (l && path[l - 1] != '/' && path[l - 1] != ':')
			path[--l] = 0;
		if (l && path[l - 1] == ':') {   // (the device: there, else the device list)
			const SceUID dd = sceIoDopen(path);
			if (dd < 0)
				path[0] = 0;
			else
				sceIoDclose(dd);
			break;
		}
	}
	ensure_canvases();
	UiCanvas *c = s_cv;
	focus_off();
	rbx_video_strips_keep(1);
	if (!s_fb_bar)
		s_fb_bar = ui_canvas(-1, 0, 0, FB_W, FB_ROW_H);
	if (!s_fb_thumb)
		s_fb_thumb = ui_canvas(-1, 0, 0, 6, FB_VIEW);
	int n = 0, files = 0, sel = 0, gen = 0, result = 0, painted = 0, bar_gen = -1, thumb_h = -1;
	float pos = 0, hy = 0, vel = 0, slide = 0, stick = 0;
	int follow = 1, t_down = 0, t_moved = 0, t_y0 = 0, t_x0 = 0, t_last = 0, dragging = 0;
	float t_pos0 = 0;
	uint64_t t_time = 0, rep = 0, held_since = 0;
	FbCrumb cr[32];
	FbButton bt[4];
	int ncr = 0, nbt = 0;
	char came_from[64] = "";
	uint32_t prev = 0xffffffffu;   // (the press that opened it: not taken)
	int relist = 1, done = 0;
	while (!done) {
		if (relist) {
			files = 0;
			n = rbx_folder_list(path, s_fbn, FB_NAMES, exts, &files);
			sel = 0;
			for (int i = 0; came_from[0] && i < n; i++)
				if (!strcasecmp(s_fbn[i], came_from))
					sel = i;
			const float maxp = n * FB_ROW_H > FB_VIEW ? (float)(n * FB_ROW_H - FB_VIEW) : 0.0f;
			pos = (float)(sel * FB_ROW_H) - FB_VIEW / 2 + FB_ROW_H / 2;
			pos = pos < 0 ? 0 : pos > maxp ? maxp : pos;
			hy = (float)(sel * FB_ROW_H);
			vel = 0;
			follow = 1;
			gen++;
			relist = 0;
			painted = 0;
		}
		if (!painted) {
			fb_paint(c, T(titles[k]), path, n, files, k, cr, &ncr, bt, &nbt, ok_key, back_key);
			ui_commit(c);
			painted = 1;
		}
		s_fb_frame++;
		// ---- input
		SceCtrlData pad;
		re_pad_poll();
		re_pad_read_all(&pad);
		uint32_t b = pad.buttons | rbx_testpad_step();
		const int dy_stick = (int)pad.ly - 128;
		const uint32_t pressed_raw = b & ~prev;
		uint32_t pressed = key_repeat(b, pressed_raw, &rep);
		if (!(b & (SCE_CTRL_UP | SCE_CTRL_DOWN)))
			held_since = 0;
		else if (!held_since)
			held_since = rbx_time_us();
		prev = b;
		int move = 0;
		const int fast = held_since && rbx_time_us() - held_since > 1200000;   // (held over 1.2 s: 3 rows a step)
		if (pressed & SCE_CTRL_UP) move -= fast ? 3 : 1;
		if (pressed & SCE_CTRL_DOWN) move += fast ? 3 : 1;
		const int page = FB_VIEW / FB_ROW_H - 1;
		if (pressed & (SCE_CTRL_LTRIGGER | SCE_CTRL_LEFT)) move -= page;
		if (pressed & (SCE_CTRL_RTRIGGER | SCE_CTRL_RIGHT)) move += page;
		if (dy_stick < -40 || dy_stick > 40) {   // the stick: rows a frame as far as it is pushed
			const float f = (float)(abs(dy_stick) - 40) / 88.0f;
			stick += (dy_stick < 0 ? -1.0f : 1.0f) * (0.08f + f * f * 0.9f);
			while (stick >= 1.0f) { move++; stick -= 1.0f; }
			while (stick <= -1.0f) { move--; stick += 1.0f; }
		} else
			stick = 0;
		int act = 0;
		if (pressed_raw & ok) act = FBA_OPEN;
		else if (pressed_raw & back) act = FBA_BACK;
		else if (pressed_raw & SCE_CTRL_TRIANGLE) act = FBA_USE;
		else if (pressed_raw & SCE_CTRL_SQUARE) act = FBA_NONE;
		else if (pressed_raw & SCE_CTRL_START) done = 1;
		if (move && n) {
			if (!follow) {   // (after a drag the cursor may be out of sight: from the row nearest it)
				const int top = (int)ceilf(pos / FB_ROW_H), bot = (int)floorf((pos + FB_VIEW) / FB_ROW_H) - 1;
				if (sel < top) sel = top;
				if (sel > bot) sel = bot;
			}
			sel += move;
			sel = sel < 0 ? 0 : sel >= n ? n - 1 : sel;
			follow = 1;
			vel = 0;
		}
		// touch: rows (drag with inertia, tap picks, tap on the picked row opens), the path's parts, the buttons
		{
			int tx[6], ty[6];
			const int tn = touch_read(tx, ty);
			if (tn) {
				if (!t_down) {
					t_down = 1;
					t_moved = 0;
					t_x0 = tx[0];
					t_y0 = t_last = ty[0];
					t_pos0 = pos;
					t_time = rbx_time_us();
					dragging = ty[0] >= FB_Y0 && ty[0] < FB_Y1;
					if (dragging)
						vel = 0;
				} else {
					if (abs(ty[0] - t_y0) > 12 || abs(tx[0] - t_x0) > 12)
						t_moved = 1;
					if (dragging && t_moved) {
						follow = 0;
						const float d = (float)(t_last - ty[0]);
						vel = vel * 0.4f + d * 0.6f;   // (the finger's speed, smoothed: kept when it leaves)
						pos = t_pos0 + (float)(t_y0 - ty[0]);
					}
					t_last = ty[0];
				}
			} else if (t_down) {
				t_down = 0;
				if (!t_moved && rbx_time_us() - t_time < 500000) {
					if (t_y0 >= FB_Y0 && t_y0 < FB_Y1 && n) {
						const int i = (int)floorf(((float)(t_y0 - FB_Y0) + pos) / FB_ROW_H);
						if (i >= 0 && i < n) {
							if (i == sel)
								act = FBA_OPEN;
							sel = i;
							follow = 1;
							vel = 0;
						}
					} else if (t_y0 < FB_Y0 - 12 && t_y0 > 50) {
						for (int i = 0; i < ncr; i++)
							if (t_x0 >= cr[i].x0 && t_x0 < cr[i].x1 && cr[i].len < (int)strlen(path)) {
								// (a part of the path: there, the folder below it picked)
								const char *rest = path + cr[i].len;
								while (*rest == '/') rest++;
								const char *e = strchr(rest, '/');
								snprintf(came_from, sizeof(came_from), "%.*s", e ? (int)(e - rest) : (int)strlen(rest), rest);
								path[cr[i].len] = 0;
								if (path[cr[i].len - 1] != ':' && path[cr[i].len - 1] != '/')
									strcat(path, "/");
								slide = -60.0f;
								relist = 1;
							}
					} else if (t_y0 >= FB_Y1 + 8) {
						for (int i = 0; i < nbt; i++)
							if (t_x0 >= bt[i].x0 && t_x0 < bt[i].x1)
								act = bt[i].act;
					}
				}
				dragging = 0;
			}
		}
		// ---- actions
		if (act == FBA_OPEN && n && sel < n) {
			const size_t l = strlen(path);
			snprintf(path + l, sizeof(path) - l, l ? "%s/" : "%s", s_fbn[sel]);
			came_from[0] = 0;
			slide = 60.0f;
			relist = 1;
		} else if (act == FBA_BACK) {
			size_t l = strlen(path);
			if (!l)
				done = 1;
			else {
				if (path[l - 1] == ':') {
					snprintf(came_from, sizeof(came_from), "%s", path);
					path[0] = 0;
				} else {
					if (path[l - 1] == '/')
						path[--l] = 0;
					while (l && path[l - 1] != '/' && path[l - 1] != ':')
						l--;
					snprintf(came_from, sizeof(came_from), "%s", path + l);
					path[l] = 0;
				}
				slide = -60.0f;
				relist = 1;
			}
		} else if (act == FBA_USE && path[0]) {
			rbx_folder_set(k, path);
			result = 1;
			done = 1;
		} else if (act == FBA_NONE) {
			rbx_folder_set(k, "");
			result = 1;
			done = 1;
		}
		if (relist || done)
			continue;
		// ---- motion
		const float maxp = n * FB_ROW_H > FB_VIEW ? (float)(n * FB_ROW_H - FB_VIEW) : 0.0f;
		if (follow) {   // the view eases to keep the cursor a row away from the edges
			const float sy = (float)(sel * FB_ROW_H), margin = FB_ROW_H * 1.2f;
			float tgt = pos;
			if (sy - margin < tgt) tgt = sy - margin;
			if (sy + FB_ROW_H + margin > tgt + FB_VIEW) tgt = sy + FB_ROW_H + margin - FB_VIEW;
			tgt = tgt < 0 ? 0 : tgt > maxp ? maxp : tgt;
			pos += (tgt - pos) * 0.25f;
			if (fabsf(tgt - pos) < 0.3f)
				pos = tgt;
		} else if (!t_down) {   // inertia, then a soft stop back inside the ends
			pos += vel;
			vel *= 0.95f;
			if (fabsf(vel) < 0.05f)
				vel = 0;
			if (pos < 0) { pos += (0 - pos) * 0.25f; vel *= 0.5f; }
			if (pos > maxp) { pos += (maxp - pos) * 0.25f; vel *= 0.5f; }
		} else {   // (dragged past an end: it gives way at half the finger's speed)
			if (pos < 0) pos *= 0.5f;
			if (pos > maxp) pos = maxp + (pos - maxp) * 0.5f;
		}
		hy += ((float)(sel * FB_ROW_H) - hy) * 0.35f;
		if (fabsf((float)(sel * FB_ROW_H) - hy) < 0.3f)
			hy = (float)(sel * FB_ROW_H);
		slide *= 0.78f;
		if (fabsf(slide) < 0.5f)
			slide = 0;
		// ---- the strips: the cursor's bar, the rows on screen, the scroll bar's thumb
		ReiStrip st[RBX_NSTRIP];
		int ns = 0;
		const int xo = FB_X + (int)slide;
		if (n && s_fb_bar) {
			if (bar_gen != s_theme_gen) {
				const Theme *th = theme();
				ui_clear(s_fb_bar);
				rrect(s_fb_bar, 4, 6, FB_W - 8, FB_ROW_H - 8, 10, th->shade);
				rrect(s_fb_bar, 0, 3, FB_W - 8, FB_ROW_H - 8, 10, COL_HILITE);
				rrect_outline(s_fb_bar, 0, 3, FB_W - 8, FB_ROW_H - 8, 10, 1, (th->sel & 0x00FFFFFFu) | 0x70000000u);
				rrect(s_fb_bar, 6, 14, 4, FB_ROW_H - 30, 2, COL_SEL);
				ui_commit(s_fb_bar);
				bar_gen = s_theme_gen;
			}
			st[ns++] = (ReiStrip){ ui_tex(s_fb_bar), xo, FB_Y0 + (int)floorf(hy - pos + 0.5f) };
		}
		const int i0 = (int)floorf(pos / FB_ROW_H) - 1, i1 = (int)ceilf((pos + FB_VIEW) / FB_ROW_H) + 1;
		for (int i = i0 < 0 ? 0 : i0; i < n && i <= i1 && ns < RBX_NSTRIP - 1; i++) {
			const ReiTex *t = fb_row(i, i == sel, !path[0], gen + s_theme_gen * 4096);
			if (t)
				st[ns++] = (ReiStrip){ t, xo, FB_Y0 + (int)floorf((float)(i * FB_ROW_H) - pos + 0.5f) };
		}
		if (maxp > 0 && s_fb_thumb) {
			const int th_h = (int)((float)(FB_VIEW - 12) * FB_VIEW / (float)(n * FB_ROW_H)) < 24 ? 24
			                 : (int)((float)(FB_VIEW - 12) * FB_VIEW / (float)(n * FB_ROW_H));
			if (th_h != thumb_h) {
				ui_clear(s_fb_thumb);
				rrect(s_fb_thumb, 0, 0, 6, th_h, 3, COL_THUMB);
				ui_commit(s_fb_thumb);
				thumb_h = th_h;
			}
			const float pp = pos < 0 ? 0 : pos > maxp ? maxp : pos;
			st[ns++] = (ReiStrip){ ui_tex(s_fb_thumb), SCREEN_W - 18,
				FB_Y0 + 6 + (int)((float)(FB_VIEW - 12 - th_h) * pp / maxp) };
		}
		rbx_video_strips(st, ns, FB_Y0, FB_Y1);
		shot_check(c);   // (log builds: vita.py shot)
		rbx_video_present();
		if ((s_fb_frame & 31) == 0)
			sceKernelPowerTick(SCE_KERNEL_POWER_TICK_DEFAULT);
	}
	// back to the menu: its strips and its whole panel again
	rbx_video_strips(NULL, 0, 0, 0);
	rbx_video_strips_keep(0);
	ui_clear(c);
	s_drawn_h = 0;
	s_dirty = 1;
	s_mt_block = rbx_time_us() + 300000;   // (the finger that tapped a button: not a tap on the menu)
	s_prev_buttons = 0xffffffffu;   // (the button that closed it, still down: not a press in the menu, START closed it too)
	return result;
}
