// The game list's preview (snap/<set>.mp4, .png, .jpg). GPL-2.0-or-later.
#ifndef RBX_FRONTEND_AVSNAP_H
#define RBX_FRONTEND_AVSNAP_H

#include <vita2d.h>

enum { RBX_SNAP_NONE, RBX_SNAP_IMAGE, RBX_SNAP_VIDEO };

void rbx_snap_request(const char *game, const char *parent);   // (NULL: none) what was shown goes at once
vita2d_texture *rbx_snap_frame(void);   // once a frame: the picture to draw, NULL while there is none
int rbx_snap_kind(void);                // what is on the screen: RBX_SNAP_*
float rbx_snap_fade(void);              // how far it has faded in, 0 to 1

#endif
