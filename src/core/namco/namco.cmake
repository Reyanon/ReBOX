# The Namco module (src/core/namco: rbx_mod_namco, the Namco boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_NAMCO_DIR ${RBX_CORE_DIR}/namco)
list(APPEND RBX_CORE_SOURCES
  ${RBX_NAMCO_DIR}/namco_module.c
  ${RBX_NAMCO_DIR}/boards/namcos1.c
  ${RBX_NAMCO_DIR}/boards/namcos2.c
  ${RBX_NAMCO_DIR}/boards/namcos86.c
  ${RBX_NAMCO_DIR}/boards/baraduke.c
  ${RBX_NAMCO_DIR}/boards/skykid.c
  ${RBX_NAMCO_DIR}/boards/pacland.c
  ${RBX_NAMCO_DIR}/boards/mappy.c
  ${RBX_NAMCO_DIR}/boards/namcona1.c
  ${RBX_NAMCO_DIR}/boards/namconb1.c
  ${RBX_NAMCO_DIR}/machine/namcoio.c
  ${RBX_NAMCO_DIR}/video/c355spr.c
  ${RBX_NAMCO_DIR}/video/c169roz.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_NAMCO)
