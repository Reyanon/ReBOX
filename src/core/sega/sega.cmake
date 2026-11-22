# The Sega module (src/core/sega: rbx_mod_sega). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_SEGA_DIR ${RBX_CORE_DIR}/sega)
list(APPEND RBX_CORE_SOURCES
  ${RBX_SEGA_DIR}/sega_module.c
  ${RBX_SEGA_DIR}/boards/segas16a.c
  ${RBX_SEGA_DIR}/boards/segas16b.c
  ${RBX_SEGA_DIR}/boards/segas18.c
  ${RBX_SEGA_DIR}/boards/segaxbd.c
  ${RBX_SEGA_DIR}/boards/segaybd.c
  ${RBX_SEGA_DIR}/boards/segas24.c
  ${RBX_SEGA_DIR}/boards/segas32.c
  ${RBX_SEGA_DIR}/video/s315_5313.c
  ${RBX_SEGA_DIR}/machine/fd1094.c
  ${RBX_SEGA_DIR}/machine/fd1089.c
  ${RBX_SEGA_DIR}/machine/mc8123.c
  ${RBX_SEGA_DIR}/machine/segaic16m.c
  ${RBX_SEGA_DIR}/machine/segapcm.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_SEGA)
