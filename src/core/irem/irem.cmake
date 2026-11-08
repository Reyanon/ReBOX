# The Irem module (src/core/irem: rbx_mod_irem, Irem boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_IREM_DIR ${RBX_CORE_DIR}/irem)
list(APPEND RBX_CORE_SOURCES
  ${RBX_IREM_DIR}/irem_module.c
  ${RBX_IREM_DIR}/machine/irem_audio.c
  ${RBX_IREM_DIR}/boards/m52.c
  ${RBX_IREM_DIR}/boards/m57.c
  ${RBX_IREM_DIR}/boards/m58.c
  ${RBX_IREM_DIR}/boards/m62.c
  ${RBX_IREM_DIR}/boards/m63.c
  ${RBX_IREM_DIR}/boards/travrusa.c
  ${RBX_IREM_DIR}/boards/vigilant.c
  ${RBX_IREM_DIR}/boards/spartanxtec.c
  ${RBX_IREM_DIR}/boards/olibochu.c
  ${RBX_IREM_DIR}/boards/m10.c
  ${RBX_IREM_DIR}/boards/m14.c
  ${RBX_IREM_DIR}/boards/redalert.c
  ${RBX_IREM_DIR}/machine/pic8259.c
  ${RBX_IREM_DIR}/machine/irem_snd.c
  ${RBX_IREM_DIR}/video/irem_gfx.c
  ${RBX_IREM_DIR}/boards/m92.c
  ${RBX_IREM_DIR}/boards/m107.c
  ${RBX_IREM_DIR}/boards/m72.c
  ${RBX_IREM_DIR}/boards/m90.c
  ${RBX_IREM_DIR}/machine/irem_zsnd.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_IREM)
