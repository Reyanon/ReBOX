# The Psikyo module (src/core/psikyo: rbx_mod_psikyo, the 68EC020 boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_PSIKYO_DIR ${RBX_CORE_DIR}/psikyo)
list(APPEND RBX_CORE_SOURCES
  ${RBX_PSIKYO_DIR}/psikyo_module.c
  ${RBX_PSIKYO_DIR}/boards/psikyo.c
  ${RBX_PSIKYO_DIR}/boards/psikyosh.c
  ${RBX_PSIKYO_DIR}/boards/psikyo4.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_PSIKYO)
