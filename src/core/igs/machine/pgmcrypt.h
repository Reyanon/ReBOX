/*
 * PGM 68000 ROM decryption (IGS025/IGS027A protected PGM sets).
 * Ported from MAME 0.289 src/mame/igs/pgmcrypt.cpp (the PGM functions),
 * plus pgm_drgw2_decrypt from pgmprot_igs025_igs012.cpp and
 * pgm_dw3_decrypt / pgm_killbld_decrypt from pgmprot_igs025_igs022.cpp.
 * MAME license: BSD-3-Clause. Copyright-holders: David Haywood, ElSemi.
 *
 * This file is part of ReBOX. GPL-2.0-or-later.
 */

#ifndef RBX_IGS_PGMCRYPT_H
#define RBX_IGS_PGMCRYPT_H

#include <stdint.h>

/* region: host-order 16-bit words, size in bytes. */
void pgm_kov_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_kovsh_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_photoy2k_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_pstar_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_dfront_decrypt(uint8_t *user1, uint32_t size);
void pgm_ddp2_decrypt(uint8_t *user1, uint32_t size);
void pgm_mm_decrypt(uint8_t *user1, uint32_t size);
void pgm_kov2_decrypt(uint8_t *user1, uint32_t size);
void pgm_kov2p_decrypt(uint8_t *user1, uint32_t size);
void pgm_puzzli2_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_theglad_decrypt(uint8_t *user1, uint32_t size);
void pgm_oldsplus_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_kovshp_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_killbldp_decrypt(uint8_t *user1, uint32_t size);
void pgm_svg_decrypt(uint8_t *user1, uint32_t size);
void pgm_svgpcb_decrypt(uint8_t *user1, uint32_t size);
void pgm_py2k2_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_ket_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_espgal_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_happy6_decrypt(uint8_t *user1, uint32_t size);
void pgm_decrypt_pgm3in1(uint8_t *maincpu, uint32_t size);
void pgm_dwpc_decrypt(uint8_t *user1, uint32_t size);
void pgm_drgw2_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_dw3_decrypt(uint8_t *maincpu, uint32_t size);
void pgm_killbld_decrypt(uint8_t *maincpu, uint32_t size);

#endif
