# ReBOX emulation core: shared by the Vita and PSP builds and the PC harness.
# Usage: include(${RBX_ROOT}/src/core/core.cmake), then link rbx_core.

set(RBX_CORE_DIR ${CMAKE_CURRENT_LIST_DIR})

set(RBX_CORE_SOURCES
  ${RBX_CORE_DIR}/core.c
  ${RBX_CORE_DIR}/zip.c
  ${RBX_CORE_DIR}/rbx_games.c
  ${RBX_CORE_DIR}/cpu/m68k_bus.c
  ${RBX_CORE_DIR}/cpu/m68k_spin.c
  ${RBX_CORE_DIR}/cpu/m68k/m68kcpu.c
  ${RBX_CORE_DIR}/cpu/m68k/m68kops.c
  ${RBX_CORE_DIR}/cpu/m68k/m68kopac.c
  ${RBX_CORE_DIR}/cpu/m68k/m68kopdm.c
  ${RBX_CORE_DIR}/cpu/m68k/m68kopnz.c
  ${RBX_CORE_DIR}/cpu/z80/z80.c
  ${RBX_CORE_DIR}/cpu/nec/nec.c
  ${RBX_CORE_DIR}/cpu/nec/irem_cpu.c
  ${RBX_CORE_DIR}/cpu/i8080/i8080.c
  ${RBX_CORE_DIR}/cpu/upd7810/upd7810.c
  ${RBX_CORE_DIR}/cpu/m6809/m6809.c
  ${RBX_CORE_DIR}/cpu/hd6309/hd6309.c
  ${RBX_CORE_DIR}/cpu/konami/konami.c
  ${RBX_CORE_DIR}/cpu/m6502/m6502.c
  ${RBX_CORE_DIR}/cpu/m740/m740.c
  ${RBX_CORE_DIR}/cpu/m37710/m37710.c
  ${RBX_CORE_DIR}/cpu/arm/arm.c
  ${RBX_CORE_DIR}/cpu/m6800/m6800.c
  ${RBX_CORE_DIR}/cpu/v60/v60.c
  ${RBX_CORE_DIR}/sound/rbx_ym.cpp
  ${RBX_CORE_DIR}/sound/okim6295.c
  ${RBX_CORE_DIR}/sound/qsound.c
  ${RBX_CORE_DIR}/sound/rf5c68.c
  ${RBX_CORE_DIR}/sound/multipcm.c
  ${RBX_CORE_DIR}/sound/namco_wsg.c
  ${RBX_CORE_DIR}/sound/c140.c
  ${RBX_CORE_DIR}/sound/c352.c
  ${RBX_CORE_DIR}/sound/iremga20.c
  ${RBX_CORE_DIR}/sound/msm5205.c
  ${RBX_CORE_DIR}/sound/msm5205u.c
  ${RBX_CORE_DIR}/sound/c6280.c
  ${RBX_CORE_DIR}/sound/k054539.c
  ${RBX_CORE_DIR}/sound/k053260.c
  ${RBX_CORE_DIR}/sound/k007232.c
  ${RBX_CORE_DIR}/sound/k005289.c
  ${RBX_CORE_DIR}/sound/k051649.c
  ${RBX_CORE_DIR}/sound/x1_010.c
  ${RBX_CORE_DIR}/sound/upd7759.c
  ${RBX_CORE_DIR}/sound/vlm5030.c
  ${RBX_CORE_DIR}/sound/pokey.c
  ${RBX_CORE_DIR}/sound/tms5220.c
  ${RBX_CORE_DIR}/cpu/tms320c1x/tms320c1x.c
  ${RBX_CORE_DIR}/sound/ymfm/ymfm_opn.cpp
  ${RBX_CORE_DIR}/sound/ymfm/ymfm_opm.cpp
  ${RBX_CORE_DIR}/sound/ymfm/ymfm_adpcm.cpp
  ${RBX_CORE_DIR}/sound/ymfm/ymfm_ssg.cpp
  ${RBX_CORE_DIR}/sound/ymfm/ymfm_opl.cpp
  ${RBX_CORE_DIR}/sound/ymfm/ymfm_pcm.cpp
  ${RBX_CORE_DIR}/machine/eeprom93c46.c
  ${RBX_CORE_DIR}/machine/eepromser.c
  ${RBX_CORE_DIR}/machine/i8255.c
  ${RBX_CORE_DIR}/machine/taito68705.c
  ${RBX_CORE_DIR}/video/x1_001.c
  ${RBX_CORE_DIR}/sound/ay8910.c
  ${RBX_CORE_DIR}/cpu/m6800/m6801.c
  ${RBX_CORE_DIR}/cpu/m6805/m68705.c
  ${RBX_CORE_DIR}/cpu/mcs48/mcs48.c
  ${RBX_CORE_DIR}/machine/pia6821.c
  ${RBX_CORE_DIR}/video/mc6845.c
  ${RBX_CORE_DIR}/cpu/m68k_multi.c
  ${RBX_CORE_DIR}/video/gfxlayout.c
  ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68kcpu.c
  ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68kops.c
  ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68kopac.c
  ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68kopdm.c
  ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68kopnz.c
  ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68k_bus.c
  ${RBX_CORE_DIR}/sound/es5505.c
  ${RBX_CORE_DIR}/sound/es5510.c
  ${RBX_CORE_DIR}/video/zoomgfx.c
  ${RBX_CORE_DIR}/sound/msm5232.c
  ${RBX_CORE_DIR}/cpu/mcs51/mcs51.c
  ${RBX_CORE_DIR}/cpu/tlcs90/tlcs90.c
  ${RBX_CORE_DIR}/sound/sn76496.c
  ${RBX_CORE_DIR}/sound/ymz280b.c
  ${RBX_CORE_DIR}/sound/mpeg_audio.c
  ${RBX_CORE_DIR}/sound/ymz774.c
  ${RBX_CORE_DIR}/cpu/h6280/h6280.c
  ${RBX_CORE_DIR}/cpu/tms57002/tms57002.c
  ${RBX_CORE_DIR}/cpu/sh2/sh2.c
  ${RBX_CORE_DIR}/cpu/sh2/sh2idle.c
  ${RBX_CORE_DIR}/cpu/i386/i386.c
  ${RBX_CORE_DIR}/sound/ymf271.c
  ${RBX_CORE_DIR}/machine/fifo7200.c
  ${RBX_CORE_DIR}/machine/intelfsh.c
  ${RBX_CORE_DIR}/machine/ds2404.c
)

# the core modules (rbx_module.h): each adds its sources to RBX_CORE_SOURCES and its define to RBX_CORE_MODULE_DEFS
set(RBX_CORE_MODULE_DEFS)
include(${RBX_CORE_DIR}/taito/taito.cmake)
include(${RBX_CORE_DIR}/deco/deco.cmake)
include(${RBX_CORE_DIR}/konami/konami.cmake)
include(${RBX_CORE_DIR}/seta/seta.cmake)
include(${RBX_CORE_DIR}/irem/irem.cmake)
include(${RBX_CORE_DIR}/igs/igs.cmake)
include(${RBX_CORE_DIR}/cave/cave.cmake)
include(${RBX_CORE_DIR}/toaplan/toaplan.cmake)
include(${RBX_CORE_DIR}/capcom/capcom.cmake)
include(${RBX_CORE_DIR}/snk/snk.cmake)
include(${RBX_CORE_DIR}/sega/sega.cmake)
include(${RBX_CORE_DIR}/psikyo/psikyo.cmake)
include(${RBX_CORE_DIR}/nmk/nmk.cmake)
include(${RBX_CORE_DIR}/namco/namco.cmake)
include(${RBX_CORE_DIR}/kaneko/kaneko.cmake)
include(${RBX_CORE_DIR}/vsystem/vsystem.cmake)
include(${RBX_CORE_DIR}/jaleco/jaleco.cmake)
include(${RBX_CORE_DIR}/seibu/seibu.cmake)
include(${RBX_CORE_DIR}/technos/technos.cmake)
include(${RBX_CORE_DIR}/atari/atari.cmake)
include(${RBX_CORE_DIR}/tecmo/tecmo.cmake)
include(${RBX_CORE_DIR}/midway/midway.cmake)

add_library(rbx_core STATIC ${RBX_CORE_SOURCES})
target_compile_definitions(rbx_core PRIVATE ${RBX_CORE_MODULE_DEFS})
target_include_directories(rbx_core PUBLIC ${RBX_CORE_DIR})
set_target_properties(rbx_core PROPERTIES C_STANDARD 99 CXX_STANDARD 17)
# third-party cores keep their original code: no warning fixes there
set_source_files_properties(
  ${RBX_CORE_DIR}/cpu/m68k/m68kcpu.c ${RBX_CORE_DIR}/cpu/m68k/m68kops.c ${RBX_CORE_DIR}/cpu/m68k/m68kopac.c
  ${RBX_CORE_DIR}/cpu/m68k/m68kopdm.c ${RBX_CORE_DIR}/cpu/m68k/m68kopnz.c ${RBX_CORE_DIR}/cpu/z80/z80.c
  ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68kcpu.c ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68kops.c
  ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68kopac.c ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68kopdm.c
  ${RBX_CORE_DIR}/cpu/m68k_snd/snd_m68kopnz.c
  ${RBX_CORE_DIR}/sound/ymfm/ymfm_opn.cpp ${RBX_CORE_DIR}/sound/ymfm/ymfm_opm.cpp
  ${RBX_CORE_DIR}/sound/ymfm/ymfm_adpcm.cpp ${RBX_CORE_DIR}/sound/ymfm/ymfm_ssg.cpp
  ${RBX_CORE_DIR}/sound/ymfm/ymfm_opl.cpp ${RBX_CORE_DIR}/sound/ymfm/ymfm_pcm.cpp
  PROPERTIES COMPILE_OPTIONS "-w")
target_compile_options(rbx_core PRIVATE -fno-strict-aliasing $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>)
