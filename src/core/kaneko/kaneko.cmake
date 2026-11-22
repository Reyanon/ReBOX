# The Kaneko module (src/core/kaneko: rbx_mod_kaneko, the Kaneko 16-bit boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_KANEKO_DIR ${RBX_CORE_DIR}/kaneko)
list(APPEND RBX_CORE_SOURCES
  ${RBX_KANEKO_DIR}/kaneko_module.c
  ${RBX_KANEKO_DIR}/boards/kaneko16.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_KANEKO)
