# The Atari module (src/core/atari: rbx_mod_atari, the Atari Games 68000 boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_ATARI_DIR ${RBX_CORE_DIR}/atari)
list(APPEND RBX_CORE_SOURCES
  ${RBX_ATARI_DIR}/atari_module.c
  ${RBX_ATARI_DIR}/boards/gauntlet.c
  ${RBX_ATARI_DIR}/boards/atarisy1.c
  ${RBX_ATARI_DIR}/boards/klax.c
  ${RBX_ATARI_DIR}/boards/rampart.c
  ${RBX_ATARI_DIR}/boards/xybots.c
  ${RBX_ATARI_DIR}/boards/blstroid.c
  ${RBX_ATARI_DIR}/boards/eprom.c
  ${RBX_ATARI_DIR}/boards/skullxbo.c
  ${RBX_ATARI_DIR}/boards/toobin.c
  ${RBX_ATARI_DIR}/boards/cyberbal.c
  ${RBX_ATARI_DIR}/video/atarimo.c
  ${RBX_ATARI_DIR}/machine/slapstic.c
  ${RBX_ATARI_DIR}/machine/atarijsa.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_ATARI)
