# The Technos module (src/core/technos: rbx_mod_technos, Double Dragon 3). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_TECHNOS_DIR ${RBX_CORE_DIR}/technos)
list(APPEND RBX_CORE_SOURCES
  ${RBX_TECHNOS_DIR}/technos_module.c
  ${RBX_TECHNOS_DIR}/boards/ddragon3.c
  ${RBX_TECHNOS_DIR}/boards/ddragon.c
  ${RBX_TECHNOS_DIR}/boards/wwfsstar.c
  ${RBX_TECHNOS_DIR}/boards/shadfrce.c
  ${RBX_TECHNOS_DIR}/boards/blockout.c
  ${RBX_TECHNOS_DIR}/boards/renegade.c
  ${RBX_TECHNOS_DIR}/boards/xain.c
  ${RBX_TECHNOS_DIR}/boards/spdodgeb.c
  ${RBX_TECHNOS_DIR}/boards/vball.c
  ${RBX_TECHNOS_DIR}/boards/mystston.c
  ${RBX_TECHNOS_DIR}/boards/matmania.c
  ${RBX_TECHNOS_DIR}/boards/battlane.c
  ${RBX_TECHNOS_DIR}/boards/bogeyman.c
  ${RBX_TECHNOS_DIR}/boards/dogfgt.c
  ${RBX_TECHNOS_DIR}/boards/ssozumo.c
  ${RBX_TECHNOS_DIR}/boards/tagteam.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_TECHNOS)
