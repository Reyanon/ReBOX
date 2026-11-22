# The SNK module (src/core/snk: rbx_mod_snk, the Neo Geo). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_SNK_DIR ${RBX_CORE_DIR}/snk)
list(APPEND RBX_CORE_SOURCES
  ${RBX_SNK_DIR}/snk_module.c
  ${RBX_SNK_DIR}/boards/neogeo.c
  ${RBX_SNK_DIR}/boards/snk68.c
  ${RBX_SNK_DIR}/boards/prehisle.c
  ${RBX_SNK_DIR}/boards/alpha68k.c
  ${RBX_SNK_DIR}/boards/alpha68k_n.c
  ${RBX_SNK_DIR}/machine/neoprot.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_SNK)
