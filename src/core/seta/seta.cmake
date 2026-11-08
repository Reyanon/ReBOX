# The Seta module (src/core/seta: rbx_mod_seta, Seta boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_SETA_DIR ${RBX_CORE_DIR}/seta)
list(APPEND RBX_CORE_SOURCES
  ${RBX_SETA_DIR}/seta_module.c
  ${RBX_SETA_DIR}/boards/seta_disp.c
  ${RBX_SETA_DIR}/boards/seta_sp.c
  ${RBX_SETA_DIR}/boards/seta_tm.c
  ${RBX_SETA_DIR}/video/x1_012.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_SETA)
