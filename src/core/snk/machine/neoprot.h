/*  Neo Geo cartridge protections and encryptions (MAME src/devices/bus/neogeo/prot_*.cpp, BSD-3-Clause).
    The 68000 ROMs are in MAME's layout on a little-endian host (host-order words).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_NEOPROT_H
#define RBX_NEOPROT_H

#include <stdint.h>

// CMC42 / CMC50 (the sprites, then the S data from their end; the CMC50's M1: audiocrypt 0x80000 to the audio CPU's
// region, 0x90000)
int neo_cmc42_gfx_decrypt(uint8_t *rom, uint32_t rom_size, int extra_xor);
int neo_cmc50_gfx_decrypt(uint8_t *rom, uint32_t rom_size, int extra_xor);
void neo_sfix_decrypt(const uint8_t *rom, uint32_t rom_size, uint8_t *fixed, uint32_t fixed_size);
int neo_cmc50_m1_decrypt(uint8_t *romcrypt, uint32_t romcrypt_size, uint8_t *romaudio, uint32_t romaudio_size);
#define NEO_KOF99_GFX_KEY 0x00
#define NEO_GAROU_GFX_KEY 0x06
#define NEO_MSLUG3_GFX_KEY 0xad
#define NEO_ZUPAPA_GFX_KEY 0xbd
#define NEO_GANRYU_GFX_KEY 0x07
#define NEO_S1945P_GFX_KEY 0x05
#define NEO_PREISLE2_GFX_KEY 0x9f
#define NEO_BANGBEAD_GFX_KEY 0xf8
#define NEO_NITD_GFX_KEY 0xff
#define NEO_SENGOKU3_GFX_KEY 0xfe
#define NEO_KOF2000_GFX_KEY 0x00
#define NEO_KOF2001_GFX_KEY 0x1e
#define NEO_MSLUG4_GFX_KEY 0x31
#define NEO_ROTD_GFX_KEY 0x3f
#define NEO_PNYAA_GFX_KEY 0x2e
#define NEO_KOF2002_GFX_KEY 0xec
#define NEO_MATRIM_GFX_KEY 0x6a
#define NEO_SAMSHO5_GFX_KEY 0x0f
#define NEO_SAMSHO5SP_GFX_KEY 0x0d
#define NEO_MSLUG5_GFX_KEY 0x19
#define NEO_SVC_GFX_KEY 0x57
#define NEO_KOF2003_GFX_KEY 0x9d
#define NEO_JOCKEYGP_GFX_KEY 0xac

// SMA (the P ROM region, 0x900000)
void neo_sma_kof99_decrypt_68k(uint8_t *base);
void neo_sma_garou_decrypt_68k(uint8_t *base);
void neo_sma_garouh_decrypt_68k(uint8_t *base);
void neo_sma_mslug3_decrypt_68k(uint8_t *base);
void neo_sma_mslug3a_decrypt_68k(uint8_t *base);
void neo_sma_kof2000_decrypt_68k(uint8_t *base);
uint32_t neo_sma_kof99_bank_base(uint16_t sel);
uint32_t neo_sma_garou_bank_base(uint16_t sel);
uint32_t neo_sma_garouh_bank_base(uint16_t sel);
uint32_t neo_sma_mslug3_bank_base(uint16_t sel);
uint32_t neo_sma_mslug3a_bank_base(uint16_t sel);
uint32_t neo_sma_kof2000_bank_base(uint16_t sel);
uint16_t neo_sma_random(uint16_t *rng);   // (the state starts at 0x2345)

// NEO-PCM2 (the V ROMs)
void neo_pcm2_decrypt(uint8_t *ymrom, uint32_t ymsize, int value);
int neo_pcm2_swap(uint8_t *ymrom, uint32_t ymsize, int value);

// KOF2002-type, PVC, KOF98 (the P ROMs)
int neo_kof2002_decrypt_68k(uint8_t *cpurom, uint32_t cpurom_size);
int neo_samsho5_decrypt_68k(uint8_t *cpurom, uint32_t cpurom_size);
int neo_samsh5sp_decrypt_68k(uint8_t *cpurom, uint32_t cpurom_size);
int neo_mslug5_decrypt_68k(uint8_t *rom, uint32_t size);
int neo_svc_px_decrypt(uint8_t *rom, uint32_t size);
int neo_kof2003_decrypt_68k(uint8_t *rom, uint32_t size);
int neo_kof2003h_decrypt_68k(uint8_t *rom, uint32_t size);
int neo_kof98_decrypt_68k(uint8_t *cpurom, uint32_t cpurom_size, uint16_t *default_rom);

// the PVC's cartridge RAM (0x1000 words at 0x2fe000)
void neo_pvc_w(uint16_t *ram, int offset, uint16_t data, uint16_t mask);
uint32_t neo_pvc_bank_base(uint16_t *ram);

// Metal Slug X
typedef struct { uint16_t command, counter; } neo_mslugx_t;
void neo_mslugx_w(neo_mslugx_t *p, int offset, uint16_t data);
uint16_t neo_mslugx_r(neo_mslugx_t *p, uint8_t (*read_byte)(uint32_t), uint16_t (*read_word)(uint32_t));

// Fatal Fury 2: the PRO-CT0
typedef struct {
	uint32_t c, sr, old_sr;
	uint8_t clk, load, even, h, old_even, old_h, gad, gbd;
} neo_proct0_t;
void neo_proct0_reset(neo_proct0_t *c);
uint16_t neo_fatfury2_r(neo_proct0_t *c, uint32_t offset);
void neo_fatfury2_w(neo_proct0_t *c, uint32_t offset, uint16_t data);

#endif
