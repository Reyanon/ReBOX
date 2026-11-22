# The Capcom module (src/core/capcom: rbx_mod_capcom). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_CAPCOM_DIR ${RBX_CORE_DIR}/capcom)
list(APPEND RBX_CORE_SOURCES
  ${RBX_CAPCOM_DIR}/capcom_module.c
  ${RBX_CAPCOM_DIR}/boards/cps1.c
  ${RBX_CAPCOM_DIR}/boards/cps3.c
  ${RBX_CAPCOM_DIR}/boards/lastduel.c
  ${RBX_CAPCOM_DIR}/boards/bionicc.c
  ${RBX_CAPCOM_DIR}/boards/tigeroad.c
  ${RBX_CAPCOM_DIR}/boards/sf.c
  ${RBX_CAPCOM_DIR}/boards/c1942.c
  ${RBX_CAPCOM_DIR}/boards/c1943.c
  ${RBX_CAPCOM_DIR}/boards/commando.c
  ${RBX_CAPCOM_DIR}/boards/gng.c
  ${RBX_CAPCOM_DIR}/boards/lwings.c
  ${RBX_CAPCOM_DIR}/boards/mitchell.c
  ${RBX_CAPCOM_DIR}/boards/higemaru.c
  ${RBX_CAPCOM_DIR}/boards/vulgus.c
  ${RBX_CAPCOM_DIR}/boards/exedexes.c
  ${RBX_CAPCOM_DIR}/boards/gunsmoke.c
  ${RBX_CAPCOM_DIR}/boards/sonson.c
  ${RBX_CAPCOM_DIR}/boards/cbasebal.c
  ${RBX_CAPCOM_DIR}/boards/blktiger.c
  ${RBX_CAPCOM_DIR}/boards/srumbler.c
  ${RBX_CAPCOM_DIR}/boards/sidearms.c
  ${RBX_CAPCOM_DIR}/machine/cps2crypt.c
  ${RBX_CAPCOM_DIR}/machine/kabuki.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_CAPCOM)
