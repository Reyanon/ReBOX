// Neon corridor: the moving background beside the game picture (Border = Artwork, games without their own artwork)
#pragma once

#include <stdint.h>

// the corridor's knobs (settings.cfg neon_*, global; on the Display page of log builds only)
enum { RBX_NEON_SPEED, RBX_NEON_DENSITY, RBX_NEON_HUE, RBX_NEON_GLOW, RBX_NEON_BRIGHT, RBX_NEON_PULSES, RBX_NEON_DUST,
	RBX_NEON_COUNT };
typedef struct {
	const char *key, *name;   // settings.cfg key, menu name
	int min, max, step, def;
} rbx_neon_param;
extern const rbx_neon_param rbx_neon_params[RBX_NEON_COUNT];
// the value as the menu shows it
void rbx_neon_value_text(int k, int v, char *out, int size);

// draws the whole screen (the picture covers the middle). game_x / game_w: the picture's columns; accent: the theme's
// colour (ABGR) the lines take their hue from
void rbx_neon_draw(float game_x, float game_w, uint32_t accent);
// (ADR-181) at the frame's start, with the last drawing's picture columns: the frame's moving parts made on a worker
// (CPU1) while the frame runs; rbx_neon_draw then only draws them. Nothing when the layout would change or "neonsync"
void rbx_neon_kick(float game_x, float game_w, uint32_t accent);
// the corridor turned a quarter (the game list: walls at the top and bottom; game_x / game_w then rows on the screen)
void rbx_neon_rotate(int on);
