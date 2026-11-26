# The Metro module (src/core/metro: rbx_mod_metro, the Metro / Imagetek boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_METRO_DIR ${RBX_CORE_DIR}/metro)
list(APPEND RBX_CORE_SOURCES
  ${RBX_METRO_DIR}/metro_module.c
  ${RBX_METRO_DIR}/boards/metro.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_METRO)
