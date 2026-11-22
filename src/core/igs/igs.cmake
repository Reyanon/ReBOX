# The IGS module (src/core/igs: rbx_mod_igs, IGS boards). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_IGS_DIR ${RBX_CORE_DIR}/igs)
list(APPEND RBX_CORE_SOURCES
  ${RBX_IGS_DIR}/igs_module.c
  ${RBX_IGS_DIR}/boards/pgm.c
  ${RBX_IGS_DIR}/boards/pgm2.c
  ${RBX_IGS_DIR}/boards/pgm_prot_asic3.c
  ${RBX_IGS_DIR}/boards/pgm_prot_025.c
  ${RBX_IGS_DIR}/boards/pgm_prot_arm2.c
  ${RBX_IGS_DIR}/boards/pgm_prot_arm1.c
  ${RBX_IGS_DIR}/boards/pgm_prot_arm3.c
  ${RBX_CORE_DIR}/cpu/arm7/arm7.c
  ${RBX_IGS_DIR}/machine/pgmcrypt.c
  ${RBX_IGS_DIR}/machine/igs036crypt.c
  ${RBX_IGS_DIR}/video/igs023.c
  ${RBX_CORE_DIR}/sound/ics2115.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_IGS)
