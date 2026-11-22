# The Toaplan module (src/core/toaplan: rbx_mod_toaplan). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_TOAPLAN_DIR ${RBX_CORE_DIR}/toaplan)
list(APPEND RBX_CORE_SOURCES
  ${RBX_TOAPLAN_DIR}/toaplan_module.c
  ${RBX_TOAPLAN_DIR}/boards/toaplan2.c
  ${RBX_TOAPLAN_DIR}/boards/toaplan1.c
  ${RBX_TOAPLAN_DIR}/boards/twincobr.c
  ${RBX_TOAPLAN_DIR}/video/gp9001.c
  ${RBX_TOAPLAN_DIR}/video/txtilemap.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_TOAPLAN)
