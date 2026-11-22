// The game list's preview: ux0:data/rebox/snap/<set>.mp4 (AvPlayer, looped, with its sound), .png or .jpg, or the parent's.
// A thread of its own opens, closes and plays the sound, so the list never waits; the frames are copied to two buffers of
// ours, so the GPU never reads one the player is writing. GPL-2.0-or-later.
#include "frontend/gamelist.h"
#include "frontend/avsnap.h"

#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include <psp2/audioout.h>
#include <psp2/avplayer.h>
#include <psp2/gxm.h>
#include <psp2/io/stat.h>
#include <psp2/kernel/sysmem.h>
#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/threadmgr.h>
#include <psp2/sysmodule.h>

#include "frontend/settings.h"
#include "platform/log.h"

#define FADE_FRAMES 40
#define FRAME_BLOCK 0x40000u   // (a copied frame's memory: 320 x 240 and more)

static SceUID s_thread = -1, s_sema = -1;
static volatile unsigned s_req;       // the newest request's number
static char s_req_game[32], s_req_parent[32];
static SceAvPlayerHandle s_player;    // (the thread's)
static int s_loaded;
static SceUInt64 s_open_us;   // (when the player was opened)
static volatile int s_had_frame;   // (the open player gave a frame)
#define BAD_MAX 64
static char s_bad[BAD_MAX][48], s_open_path[48];   // the videos that gave no frame this run, the one open
static uint8_t s_bad_n[BAD_MAX];                      // (their failures)
static int s_nbad;
static int is_bad(const char *path)
{
	for (int i = 0; i < s_nbad; i++)
		if (!strcmp(s_bad[i], path))
			return s_bad_n[i] >= 2;   // (11-22: twice; once was the decoder's state as often as the file)
	return 0;
}
static void bad_note(const char *path)
{
	for (int i = 0; i < s_nbad; i++)
		if (!strcmp(s_bad[i], path)) {
			s_bad_n[i]++;
			return;
		}
	if (s_nbad < BAD_MAX) {
		snprintf(s_bad[s_nbad], sizeof(s_bad[0]), "%s", path);
		s_bad_n[s_nbad++] = 1;
	}
}
static volatile int s_kind;           // RBX_SNAP_*: what is shown now
static vita2d_texture *volatile s_img;
static vita2d_texture s_tex;          // the video's picture
static void *s_buf[2];
static int s_cur, s_have, s_fade_n;
static int s_port = -1, s_port_grain, s_port_rate, s_port_mode;

// the player's memory (user 11-22): its big blocks in memory blocks of their own, not the app's heap. With the heap
// nearly full (0.6 MB free after a while in the list) the decoder got no memory and gave no frame, every video after
#define BIG_ALLOC 0x10000u
static void *mem_alloc(void *arg, uint32_t alignment, uint32_t size)
{
	(void)arg;
	if (size >= BIG_ALLOC && alignment <= 4096) {
		const SceUID uid = sceKernelAllocMemBlock("rbx_avsnap_mem", SCE_KERNEL_MEMBLOCK_TYPE_USER_RW,
			(size + 0xfffu) & ~0xfffu, NULL);
		void *p = NULL;
		if (uid >= 0 && sceKernelGetMemBlockBase(uid, &p) >= 0)
			return p;
		if (uid >= 0)
			sceKernelFreeMemBlock(uid);
		RBX_LOG("avsnap: player memory %u in a block failed (%08x)", (unsigned)size, (unsigned)uid);
	}
	return memalign(alignment < 64 ? 64 : alignment, size);
}
static void mem_free(void *arg, void *p)
{
	(void)arg;
	if (!p)
		return;
	const SceUID uid = sceKernelFindMemBlockByAddr(p, 0);
	SceKernelMemBlockInfo bi;
	bi.size = sizeof(bi);
	if (uid >= 0 && sceKernelGetMemBlockInfoByAddr(p, &bi) >= 0 && bi.mappedBase == p) {   // (one of ours: its block)
		sceKernelFreeMemBlock(uid);
		return;
	}
	free(p);
}
// memory the GPU reads: CDRAM, else physically contiguous main memory (user 11-23: with the list's layers in CDRAM
// the decoder's frames did not fit, no frame came; the contiguous memory was unused). *uid: its block
static void *gpu_alloc(uint32_t size, SceUID *uid_out)
{
	*uid_out = -1;
	SceUID uid = sceKernelAllocMemBlock("rbx_avsnap", SCE_KERNEL_MEMBLOCK_TYPE_USER_CDRAM_RW, (size + 0x3ffff) & ~0x3ffffu, NULL);
	uint32_t got = (size + 0x3ffff) & ~0x3ffffu;
	if (uid < 0) {
		got = (size + 0xfffff) & ~0xfffffu;
		uid = sceKernelAllocMemBlock("rbx_avsnap_pc", SCE_KERNEL_MEMBLOCK_TYPE_USER_MAIN_PHYCONT_NC_RW, got, NULL);
	}
	void *p = NULL;
	if (uid < 0)
		return NULL;
	if (sceKernelGetMemBlockBase(uid, &p) < 0 || sceGxmMapMemory(p, got, SCE_GXM_MEMORY_ATTRIB_READ) < 0) {
		sceKernelFreeMemBlock(uid);
		return NULL;
	}
	*uid_out = uid;
	return p;
}
// the player's frame blocks, kept so that those it does not give back are freed after its close: a video the
// decoder cannot play (no frame ever) left its blocks, and after a few the next videos found no memory (user 11-15)
#define FRAME_BLOCKS 32
static void *s_fb[FRAME_BLOCKS];
static SceUID s_fb_uid[FRAME_BLOCKS];   // (each block freed by its own handle: looked up by its address, the blocks
                                         // left by a player that gave no frame stayed taken, 5 MB of CDRAM each, user 11-23)
static SceKernelLwMutexWork s_fb_lock;
static int s_fb_lock_ok;
static void fb_lock(void) { if (s_fb_lock_ok) sceKernelLockLwMutex(&s_fb_lock, 1, NULL); }
static void fb_unlock(void) { if (s_fb_lock_ok) sceKernelUnlockLwMutex(&s_fb_lock, 1); }
static void gpu_free(void *p, SceUID uid)
{
	if (uid < 0)
		uid = sceKernelFindMemBlockByAddr(p, 0);
	if (uid < 0) {
		RBX_LOG("avsnap: frame block %p not found", p);
		return;
	}
	sceGxmUnmapMemory(p);
	const int rc = sceKernelFreeMemBlock(uid);
	if (rc < 0)
		RBX_LOG("avsnap: frame block %p free %08x", p, (unsigned)rc);
}
static void *frame_alloc(void *arg, uint32_t alignment, uint32_t size)
{
	(void)arg;
	(void)alignment;
	SceUID uid;
	void *p = gpu_alloc(size, &uid);
	if (!p) {
		RBX_LOG("avsnap: frame memory %u failed", (unsigned)size);
		return NULL;
	}
	fb_lock();
	int kept = 0;
	for (int i = 0; i < FRAME_BLOCKS; i++)
		if (!s_fb[i]) {
			s_fb[i] = p;
			s_fb_uid[i] = uid;
			kept = 1;
			break;
		}
	fb_unlock();
	if (!kept)
		RBX_LOG("avsnap: more than %d frame blocks", FRAME_BLOCKS);
	return p;
}
static void frame_free(void *arg, void *p)
{
	(void)arg;
	SceUID uid = -1;
	fb_lock();
	for (int i = 0; i < FRAME_BLOCKS; i++)
		if (s_fb[i] == p) {
			uid = s_fb_uid[i];
			s_fb[i] = NULL;
		}
	fb_unlock();
	gpu_free(p, uid);
}

static void mem_note(const char *what);
static int open_video(const char *path)
{
	if (!s_fb_lock_ok)
		s_fb_lock_ok = sceKernelCreateLwMutex(&s_fb_lock, "rbx_avsnap_fb", 0, 0, NULL) >= 0;
	if (!s_loaded) {
		const int m = sceSysmoduleLoadModule(SCE_SYSMODULE_AVPLAYER);
		if (m < 0) {
			RBX_LOG("avsnap: AvPlayer module %08x", (unsigned)m);
			return 0;
		}
		s_loaded = 1;
	}
	SceAvPlayerInitData d;
	memset(&d, 0, sizeof(d));
	d.memoryReplacement.allocate = mem_alloc;
	d.memoryReplacement.deallocate = mem_free;
	d.memoryReplacement.allocateTexture = frame_alloc;
	d.memoryReplacement.deallocateTexture = frame_free;
	d.basePriority = 0xA0;
	d.numOutputVideoFrameBuffers = 2;
	d.autoStart = SCE_TRUE;
	s_player = sceAvPlayerInit(&d);
	if (!s_player || ((uint32_t)s_player >> 16) == 0x806A) {   // (the handle is an address; errors are 0x806Axxxx)
		RBX_LOG("avsnap: init failed %08x", (unsigned)s_player);
		s_player = 0;
		return 0;
	}
	const int rc = sceAvPlayerAddSource(s_player, path);
	if (rc < 0) {
		RBX_LOG("avsnap: %s: %08x", path, (unsigned)rc);
		sceAvPlayerClose(s_player);
		s_player = 0;
		return 0;
	}
	sceAvPlayerSetLooping(s_player, SCE_TRUE);
	s_had_frame = 0;
	s_open_us = sceKernelGetProcessTimeWide();
	snprintf(s_open_path, sizeof(s_open_path), "%s", path);
	RBX_LOG("avsnap: %s opened", path);
	mem_note("opened");
	return 1;
}

// (the memory free now: logged with each open and each video that gave no frame, user 11-22: videos stopped coming after
// a while in the list)
static void mem_note(const char *what)
{
	SceKernelFreeMemorySizeInfo fm;
	fm.size = sizeof(fm);
	if (sceKernelGetFreeMemorySize(&fm) < 0)
		fm.size_user = fm.size_cdram = fm.size_phycont = 0;
	const struct mallinfo mi = mallinfo();
	RBX_LOG("avsnap: %s: free user %u KB cdram %u KB phycont %u KB, heap in use %u KB free %u KB", what,
		fm.size_user >> 10, fm.size_cdram >> 10, fm.size_phycont >> 10, (unsigned)mi.uordblks >> 10, (unsigned)mi.fordblks >> 10);
}

static void close_all(void)
{
	s_kind = RBX_SNAP_NONE;
	sceKernelDelayThread(40000);   // (the frames being drawn are done with what was shown)
	if (s_player) {
		// (11-22: a player closed while it was still starting (the cursor moved on within a moment) left the decoder
		// taken: every video after it gave no frame. It is kept 0.8 s at least; this thread waits, not the list)
		for (int i = 0; i < 40 && !s_had_frame && sceAvPlayerIsActive(s_player) &&
			sceKernelGetProcessTimeWide() - s_open_us < 800000; i++)
			sceKernelDelayThread(20000);
		// (no frame and the player stopped by itself: it could not decode the file; one left before its first frame
		// only because the cursor moved on is not that)
		const int failed = !s_had_frame && !sceAvPlayerIsActive(s_player);
		sceAvPlayerStop(s_player);
		for (int i = 0; i < 50 && sceAvPlayerIsActive(s_player); i++)   // (its threads stopped first)
			sceKernelDelayThread(2000);
		sceAvPlayerClose(s_player);
		s_player = 0;
		fb_lock();   // (the blocks it kept: freed now)
		int left = 0;
		for (int i = 0; i < FRAME_BLOCKS; i++)
			if (s_fb[i]) {
				gpu_free(s_fb[i], s_fb_uid[i]);
				s_fb[i] = NULL;
				left++;
			}
		fb_unlock();
		if (left)
			RBX_LOG("avsnap: %d frame blocks left by the player freed", left);
		// (a video that gave no frame (some files the player cannot decode: it stops on them): not opened again this
		// run, its picture shown instead. Opened a second time, no video played any more until the Vita restarted (a
		// video opened next may fail once too and is left out with it))
		if (failed && s_open_path[0]) {
			bad_note(s_open_path);
			RBX_LOG("avsnap: %s gave no frame%s", s_open_path, is_bad(s_open_path) ? ": not opened again" : "");
			mem_note("no frame");
			sceKernelDelayThread(500000);   // (the next video opened at once failed too: the decoder's time back)
			// (11-22: the module is not loaded again after it: that left 5 MB of CDRAM behind each time)
		}
		s_open_path[0] = 0;
		// (user 11-15: after many previews no video came any more, each player then leaking its frame blocks: one
		// opened right after a close found the decoder still taken. Some time for it to be given back)
		sceKernelDelayThread(100000);
	}
	if (s_img) {
		vita2d_free_texture(s_img);
		s_img = NULL;
	}
	s_have = 0;
	s_fade_n = 0;
}

// an image file: the path as given, or with .png and .jpg swapped
static vita2d_texture *load_any(const char *path)
{
	char alt[256];
	snprintf(alt, sizeof(alt), "%s", path);
	const size_t l = strlen(alt);
	const int jpg = l > 4 && (!strcasecmp(alt + l - 4, ".jpg") || (l > 5 && !strcasecmp(alt + l - 5, ".jpeg")));
	vita2d_texture *t = jpg ? vita2d_load_JPEG_file(alt) : vita2d_load_PNG_file(alt);
	if (!t && l > 4 && alt[l - 4] == '.') {
		strcpy(alt + l - 3, jpg ? "png" : "jpg");
		t = jpg ? vita2d_load_PNG_file(alt) : vita2d_load_JPEG_file(alt);
	}
	return t;
}

static vita2d_texture *load_image(const char *name)
{
	char path[256];
	vita2d_texture *t = rbx_gamelist_media(name, 1, path, sizeof(path)) ? load_any(path) : NULL;
	if (t)
		vita2d_texture_set_filters(t, SCE_GXM_TEXTURE_FILTER_LINEAR, SCE_GXM_TEXTURE_FILTER_LINEAR);
	return t;
}

static void open_game(const char *game, const char *parent)
{
	const char *names[2] = { game, parent };
	for (int k = 0; k < 2; k++) {   // (a video first, the set's or else its parent's: the list's "video", else video/<set>.mp4)
		char path[256];
		SceIoStat st;
		if (!names[k][0] || !rbx_gamelist_media(names[k], 2, path, sizeof(path)))
			continue;
		if (sceIoGetstat(path, &st) < 0)   // (where the videos were before 11-22: snap/<set>.mp4)
			snprintf(path, sizeof(path), RBX_DATA_DIR "snap/%s.mp4", names[k]);
		if (sceIoGetstat(path, &st) >= 0 && !is_bad(path) && open_video(path)) {
			s_kind = RBX_SNAP_VIDEO;
			return;
		}
	}
	for (int k = 0; k < 2; k++)
		if (names[k][0] && (s_img = load_image(names[k])) != NULL) {
			s_kind = RBX_SNAP_IMAGE;
			return;
		}
}

// its sound at full volume, at its own speed: this is also what keeps the player's clock right
static void play_sound(void)
{
	SceAvPlayerFrameInfo f;
	memset(&f, 0, sizeof(f));
	if (!sceAvPlayerGetAudioData(s_player, &f) || !f.pData || !f.details.audio.sampleRate) {
		sceKernelDelayThread(4000);
		return;
	}
	const int ch = f.details.audio.channelCount, grain = (int)(f.details.audio.size / (2u * (ch ? ch : 1)));
	const int rate = (int)f.details.audio.sampleRate;
	const int mode = ch == 1 ? SCE_AUDIO_OUT_MODE_MONO : SCE_AUDIO_OUT_MODE_STEREO;
	if (s_port < 0) {
		s_port = sceAudioOutOpenPort(SCE_AUDIO_OUT_PORT_TYPE_BGM, grain, rate, mode);
		s_port_grain = grain;
		s_port_rate = rate;
		s_port_mode = mode;
		if (s_port >= 0) {
			int vol[2] = { SCE_AUDIO_VOLUME_0DB, SCE_AUDIO_VOLUME_0DB };
			sceAudioOutSetVolume(s_port, SCE_AUDIO_VOLUME_FLAG_L_CH | SCE_AUDIO_VOLUME_FLAG_R_CH, vol);
		}
	} else if (grain != s_port_grain || rate != s_port_rate || mode != s_port_mode) {
		sceAudioOutSetConfig(s_port, grain, rate, mode);
		s_port_grain = grain;
		s_port_rate = rate;
		s_port_mode = mode;
	}
	if (s_port >= 0)
		sceAudioOutOutput(s_port, f.pData);
}

static int snap_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	unsigned done = 0;
	for (;;) {
		if (done == s_req && !s_player)
			sceKernelWaitSema(s_sema, 1, NULL);
		if (done != s_req) {
			done = s_req;
			char game[32], parent[32];
			memcpy(game, s_req_game, sizeof(game));
			memcpy(parent, s_req_parent, sizeof(parent));
			close_all();
			if (game[0] && done == s_req)
				open_game(game, parent);
			continue;
		}
		if (s_player)
			play_sound();
	}
	return 0;
}

void rbx_snap_request(const char *game, const char *parent)
{
	s_kind = RBX_SNAP_NONE;   // (gone at once; the thread closes it)
	if (s_thread < 0) {
		s_sema = sceKernelCreateSema("rbx_avsnap", 0, 0, 1, NULL);
		// (CPU2, user 11-22: high priority for the sound, so on the list's core a picture decoded in it stopped the list's
		// frames 80-100 ms. CPU2 draws the games, not the list)
		s_thread = sceKernelCreateThread("rbx_avsnap", snap_thread, 0x60, 0x40000, 0, SCE_KERNEL_CPU_MASK_USER_2, NULL);
		if (s_thread < 0)
			return;
		sceKernelStartThread(s_thread, 0, NULL);
	}
	snprintf(s_req_game, sizeof(s_req_game), "%s", game ? game : "");
	snprintf(s_req_parent, sizeof(s_req_parent), "%s", parent ? parent : "");
	s_req++;
	sceKernelSignalSema(s_sema, 1);
}

int rbx_snap_kind(void) { return s_have ? s_kind : RBX_SNAP_NONE; }
float rbx_snap_fade(void) { return (float)s_fade_n / FADE_FRAMES; }

vita2d_texture *rbx_snap_frame(void)
{
	vita2d_texture *t = NULL;
	if (s_kind == RBX_SNAP_IMAGE) {
		t = s_img;
		s_have = t != NULL;
	} else if (s_kind == RBX_SNAP_VIDEO && s_player) {
		SceAvPlayerFrameInfo f;
		memset(&f, 0, sizeof(f));
		if (sceAvPlayerIsActive(s_player) && sceAvPlayerGetVideoData(s_player, &f) && f.pData) {
			const uint32_t w = f.details.video.width, h = f.details.video.height, size = w * h * 3 / 2;
			if (!s_have)
				RBX_LOG("avsnap: first frame %ux%u", (unsigned)w, (unsigned)h);
			s_had_frame = 1;
			if (size > FRAME_BLOCK) {
				static unsigned big;
				if (big++ % 120 == 0)
					RBX_LOG("avsnap: frame %ux%u too big for our copy (%u bytes)", (unsigned)w, (unsigned)h, (unsigned)size);
			}
			if (size <= FRAME_BLOCK) {
				const int next = !s_cur;
				if (!s_buf[next]) {
					SceUID uid;   // (kept for the run)
					s_buf[next] = gpu_alloc(FRAME_BLOCK, &uid);
				}
				if (s_buf[next]) {
					memcpy(s_buf[next], f.pData, size);
					sceGxmTextureInitLinear(&s_tex.gxm_tex, s_buf[next], SCE_GXM_TEXTURE_FORMAT_YVU420P2_CSC1, w, h, 0);
					sceGxmTextureSetMinFilter(&s_tex.gxm_tex, SCE_GXM_TEXTURE_FILTER_LINEAR);
					sceGxmTextureSetMagFilter(&s_tex.gxm_tex, SCE_GXM_TEXTURE_FILTER_LINEAR);
					s_cur = next;
					s_have = 1;
				}
			}
		}
		t = s_have ? &s_tex : NULL;
	}
	if (t && s_fade_n < FADE_FRAMES)
		s_fade_n++;
	return t;
}
