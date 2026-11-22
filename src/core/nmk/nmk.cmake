# The NMK module (src/core/nmk: rbx_mod_nmk, the NMK16 boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_NMK_DIR ${RBX_CORE_DIR}/nmk)
list(APPEND RBX_CORE_SOURCES
  ${RBX_NMK_DIR}/nmk_module.c
  ${RBX_NMK_DIR}/boards/nmk16.c
  ${RBX_NMK_DIR}/machine/nmk214.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_NMK)
