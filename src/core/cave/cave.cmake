# The Cave module (src/core/cave: rbx_mod_cave, Cave's 68000 boards and the CV1000). Included by core.cmake.
# This file is part of ReBOX. GPL-2.0-or-later.
set(RBX_CAVE_DIR ${RBX_CORE_DIR}/cave)
list(APPEND RBX_CORE_SOURCES
  ${RBX_CAVE_DIR}/cave_module.c
  ${RBX_CAVE_DIR}/boards/cave.c
  ${RBX_CAVE_DIR}/boards/cv1k.c
  ${RBX_CAVE_DIR}/video/epic12.c
  ${RBX_CAVE_DIR}/sound/ymz770.c
  ${RBX_CORE_DIR}/cpu/sh3/sh3.c
)
list(APPEND RBX_CORE_MODULE_DEFS RBX_MOD_CAVE)
# (the SH-3 core's computed-goto dispatch table: GCC's LTO partitions lose its local labels on the Vita link)
set_source_files_properties(${RBX_CORE_DIR}/cpu/sh3/sh3.c PROPERTIES COMPILE_OPTIONS "-fno-lto")
