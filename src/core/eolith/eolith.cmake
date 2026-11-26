# The Eolith module (src/core/eolith: rbx_mod_eolith, the Hyperstone E1 boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_EOLITH_DIR ${RBX_CORE_DIR}/eolith)
list(APPEND RBX_CORE_SOURCES
  ${RBX_EOLITH_DIR}/eolith_module.c
  ${RBX_EOLITH_DIR}/boards/eolith.c
  ${RBX_EOLITH_DIR}/boards/eolith16.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_EOLITH)
