# The Video System module (src/core/vsystem: rbx_mod_vsystem, Video System's 68000 boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_VSYSTEM_DIR ${RBX_CORE_DIR}/vsystem)
list(APPEND RBX_CORE_SOURCES
  ${RBX_VSYSTEM_DIR}/vsystem_module.c
  ${RBX_VSYSTEM_DIR}/boards/pspikes.c
  ${RBX_VSYSTEM_DIR}/boards/aerofgt.c
  ${RBX_VSYSTEM_DIR}/video/vsystem_gfx.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_VSYSTEM)
