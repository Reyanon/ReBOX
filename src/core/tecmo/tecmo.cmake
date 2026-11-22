# The Tecmo module (src/core/tecmo: rbx_mod_tecmo). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_TECMO_DIR ${RBX_CORE_DIR}/tecmo)
list(APPEND RBX_CORE_SOURCES
  ${RBX_TECMO_DIR}/tecmo_module.c
  ${RBX_TECMO_DIR}/video/tecmo_mix.c
  ${RBX_TECMO_DIR}/video/tecmo_spr.c
  ${RBX_TECMO_DIR}/boards/gaiden.c
  ${RBX_TECMO_DIR}/boards/tecmo16.c
  ${RBX_TECMO_DIR}/boards/spbactn.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_TECMO)
