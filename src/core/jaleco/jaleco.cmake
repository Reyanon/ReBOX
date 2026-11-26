# The Jaleco module (src/core/jaleco: rbx_mod_jaleco, Mega System 1, Mega System 32). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_JALECO_DIR ${RBX_CORE_DIR}/jaleco)
list(APPEND RBX_CORE_SOURCES
  ${RBX_JALECO_DIR}/jaleco_module.c
  ${RBX_JALECO_DIR}/boards/megasys1.c
  ${RBX_JALECO_DIR}/boards/ms32.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_JALECO)
