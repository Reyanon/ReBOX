# The Seibu module (src/core/seibu: rbx_mod_seibu, the Seibu Kaihatsu boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_SEIBU_DIR ${RBX_CORE_DIR}/seibu)
list(APPEND RBX_CORE_SOURCES
  ${RBX_SEIBU_DIR}/seibu_module.c
  ${RBX_SEIBU_DIR}/machine/seibusnd.c
  ${RBX_SEIBU_DIR}/machine/sched.c
  ${RBX_SEIBU_DIR}/boards/raiden.c
  ${RBX_SEIBU_DIR}/boards/dcon.c
  ${RBX_SEIBU_DIR}/boards/bloodbro.c
  ${RBX_SEIBU_DIR}/boards/dynduke.c
  ${RBX_SEIBU_DIR}/boards/cabal.c
  ${RBX_SEIBU_DIR}/boards/deadang.c
  ${RBX_SEIBU_DIR}/machine/seibucop.c
  ${RBX_SEIBU_DIR}/boards/legionna.c
  ${RBX_SEIBU_DIR}/machine/r2crypt.c
  ${RBX_SEIBU_DIR}/boards/raiden2.c
  ${RBX_SEIBU_DIR}/boards/toki.c
  ${RBX_SEIBU_DIR}/boards/wiz.c
  ${RBX_SEIBU_DIR}/boards/stfight.c
  ${RBX_SEIBU_DIR}/boards/kncljoe.c
  ${RBX_SEIBU_DIR}/machine/t5182.c
  ${RBX_SEIBU_DIR}/boards/mustache.c
  ${RBX_SEIBU_DIR}/boards/darkmist.c
  ${RBX_SEIBU_DIR}/boards/goal92.c
  ${RBX_SEIBU_DIR}/boards/goodejan.c
  ${RBX_SEIBU_DIR}/boards/sengokmj.c
  ${RBX_SEIBU_DIR}/boards/panicr.c
  ${RBX_SEIBU_DIR}/boards/airraid.c
  ${RBX_SEIBU_DIR}/boards/metlfrzr.c
  ${RBX_SEIBU_DIR}/boards/feversoc.c
  ${RBX_SEIBU_DIR}/machine/seicopbl.c
  ${RBX_SEIBU_DIR}/boards/seicupbl.c
  ${RBX_SEIBU_DIR}/machine/seibuspi_m.c
  ${RBX_SEIBU_DIR}/boards/seibuspi.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_SEIBU)
