# The Midway module (src/core/midway: rbx_mod_midway, Y-unit / T-unit / Wolf unit, Williams 6809, MCR). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_MIDWAY_DIR ${RBX_CORE_DIR}/midway)
list(APPEND RBX_CORE_SOURCES
  ${RBX_MIDWAY_DIR}/midway_module.c
  ${RBX_MIDWAY_DIR}/boards/midyunit.c
  ${RBX_MIDWAY_DIR}/boards/midtunit.c
  ${RBX_MIDWAY_DIR}/boards/williams.c
  ${RBX_MIDWAY_DIR}/boards/mcr.c
  ${RBX_MIDWAY_DIR}/boards/mcr68.c
  ${RBX_MIDWAY_DIR}/machine/midway_snd.c
  ${RBX_MIDWAY_DIR}/machine/williams_snd.c
  ${RBX_MIDWAY_DIR}/machine/dcs.c
  ${RBX_CORE_DIR}/cpu/adsp2100/adsp2100.c
  ${RBX_CORE_DIR}/cpu/tms34010/tms34010.c
  ${RBX_CORE_DIR}/cpu/pic16c5x/pic16c5x.c
  ${RBX_CORE_DIR}/sound/hc55516.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_MIDWAY)
