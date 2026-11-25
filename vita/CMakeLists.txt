cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(DEFINED ENV{VITASDK})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VITASDK}/share/vita.toolchain.cmake" CACHE PATH "toolchain file")
  else()
    message(FATAL_ERROR "VITASDK environment variable is not set. Install VitaSDK and set VITASDK.")
  endif()
endif()

project(rebox C CXX)
# Vita build. Uses the shared core (src/core) at the repo root. Build: tools/build.sh vita (-> build-vita/)
set(RBX_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
include("$ENV{VITASDK}/share/vita.cmake" REQUIRED)

set(VITA_APP_NAME "ReBOX")
set(VITA_TITLEID  "RBOX00001")
set(VITA_VERSION  "00.01")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(RBX_ARCH_FLAGS "-mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${RBX_ARCH_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RBX_ARCH_FLAGS} -Wall")
set(CMAKE_C_FLAGS_RELEASE   "-O3 -ffast-math -flto")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math -flto")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-O3 -flto=auto")   # (the link on all the cores: minutes less)
set(CMAKE_C_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

# UDP logger: ON while developing (OFF for release VPKs: -DRBX_LOG=OFF)
option(RBX_LOG "Include UDP logger" ON)
set(RBX_LOG_HOST "" CACHE STRING "PC IP to receive logs (empty = broadcast)")
set(RBX_ARM_MHZ 444 CACHE STRING "ARM clock requested at startup")

include(${RBX_ROOT}/src/core/core.cmake)
target_link_libraries(rbx_core PUBLIC z)
# every game from a fresh core (core.c fresh_core): the link gathers the core's static data by its library, which
# link-time optimization would hide (its objects are made at the link), so the core is compiled without it
target_compile_options(rbx_core PRIVATE -fno-lto)

add_executable(${PROJECT_NAME}
  src/frontend/main.c
  src/frontend/lang.c
  src/frontend/core_api.c
  src/frontend/medal.c
  src/frontend/neon.c
  src/frontend/video.c
  src/frontend/audio.c
  src/frontend/resample.c
  src/frontend/input.c
  src/frontend/threads.c
  src/frontend/settings.c
  src/frontend/menu.c
  src/frontend/states.c
  src/frontend/ui.c
  src/frontend/touch.c
  src/frontend/ime.c
  src/frontend/gamelist.c
  src/frontend/avsnap.c
  src/common/re_pad.c
  src/frontend/update.c
  src/frontend/testpad.c
  src/frontend/sampler.c
  src/frontend/present.c
  src/platform/clock.c
  src/platform/log.c
  src/platform/perf.c
  src/platform/hud.c
)

target_include_directories(${PROJECT_NAME} PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})
# Version (options About): rbx_version.h is regenerated on every build (tools/git_version.cmake)
add_custom_target(rbx_version
  COMMAND ${CMAKE_COMMAND} -DSRC=${RBX_ROOT} -DOUT=${CMAKE_CURRENT_BINARY_DIR}/rbx_version.h -P ${RBX_ROOT}/tools/git_version.cmake
  BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/rbx_version.h
  VERBATIM)
add_dependencies(${PROJECT_NAME} rbx_version)
target_compile_definitions(${PROJECT_NAME} PRIVATE RBX_ARM_MHZ=${RBX_ARM_MHZ})
set(RBX_TEXT_PAD 0 CACHE STRING "Padding at the end of the code segment (tools/build.sh, vita-elf-create overlap)")
target_compile_definitions(${PROJECT_NAME} PRIVATE RBX_TEXT_PAD=${RBX_TEXT_PAD})

if(RBX_LOG)
  target_compile_definitions(${PROJECT_NAME} PRIVATE RBX_ENABLE_LOG)
  if(RBX_LOG_HOST)
    target_compile_definitions(${PROJECT_NAME} PRIVATE RBX_LOG_HOST="${RBX_LOG_HOST}")
  endif()
endif()

# kernel helper module (PS button, update install), in the VPK as module/rbx_kctrl.skprx. Its user stubs are made here
# directly (vita_create_stubs runs make on an unquoted path, and this tree's path has a space)
add_executable(rbx_kctrl kmod/rbx_kctrl.c)
set_target_properties(rbx_kctrl PROPERTIES LINK_FLAGS "-nostdlib -fno-lto")
target_compile_options(rbx_kctrl PRIVATE -fno-builtin -fno-lto)
target_link_libraries(rbx_kctrl SceCtrlForDriver_stub SceThreadmgrForDriver_stub SceIofilemgrForDriver_stub SceSysmemForDriver_stub
  taihenForKernel_stub taihenModuleUtils_stub)
add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/rbx_kctrl.skprx"
  COMMAND vita-elf-create -e "${CMAKE_SOURCE_DIR}/kmod/exports.yml" $<TARGET_FILE:rbx_kctrl> "${CMAKE_BINARY_DIR}/rbx_kctrl.velf"
  COMMAND vita-make-fself -c "${CMAKE_BINARY_DIR}/rbx_kctrl.velf" "${CMAKE_BINARY_DIR}/rbx_kctrl.skprx"
  DEPENDS rbx_kctrl "${CMAKE_SOURCE_DIR}/kmod/exports.yml"
  VERBATIM)
add_custom_target(kctrl_skprx ALL DEPENDS "${CMAKE_BINARY_DIR}/rbx_kctrl.skprx")
set(RBX_KSTUB_DIR "${CMAKE_BINARY_DIR}/kctrl_stubs")
set(RBX_KSTUB "${RBX_KSTUB_DIR}/librbx_kctrl_v2_stub_weak.a")
add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/kctrl.yml"
  COMMAND vita-elf-export kernel $<TARGET_FILE:rbx_kctrl> "${CMAKE_SOURCE_DIR}/kmod/exports.yml" "${CMAKE_BINARY_DIR}/kctrl.yml"
  DEPENDS rbx_kctrl "${CMAKE_SOURCE_DIR}/kmod/exports.yml"
  VERBATIM)
add_custom_command(OUTPUT "${RBX_KSTUB}"
  COMMAND vita-libs-gen "${CMAKE_BINARY_DIR}/kctrl.yml" "${RBX_KSTUB_DIR}"
  COMMAND make -C "${RBX_KSTUB_DIR}"
  DEPENDS "${CMAKE_BINARY_DIR}/kctrl.yml"
  VERBATIM)
add_custom_target(kctrl_stubs DEPENDS "${RBX_KSTUB}")

# RePad: Bluetooth controllers (kernel plugin, MIT; vita/repad = the RePad project's files, keep them the same), in the
# VPK as module/repad.skprx. The app loads it when it is not running yet (re_pad_load_plugin)
add_executable(repad repad/src/main.c repad/src/bt.c repad/src/ctrl.c repad/src/pads.c)
set_target_properties(repad PROPERTIES LINK_FLAGS "-nostdlib -fno-lto")
target_compile_options(repad PRIVATE -fno-builtin -std=gnu11 -fno-lto)
target_link_libraries(repad SceBtForDriver_stub SceCtrlForDriver_stub SceKernelSuspendForDriver_stub SceSysclibForDriver_stub
  SceThreadmgrForDriver_stub taihenForKernel_stub taihenModuleUtils_stub SceSblAIMgrForDriver_stub)
add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/repad.skprx"
  COMMAND vita-elf-create -e "${CMAKE_SOURCE_DIR}/repad/repad.yml" $<TARGET_FILE:repad> "${CMAKE_BINARY_DIR}/repad.velf"
  COMMAND vita-make-fself -c "${CMAKE_BINARY_DIR}/repad.velf" "${CMAKE_BINARY_DIR}/repad.skprx"
  DEPENDS repad "${CMAKE_SOURCE_DIR}/repad/repad.yml"
  VERBATIM)
add_custom_target(repad_skprx ALL DEPENDS "${CMAKE_BINARY_DIR}/repad.skprx")
add_dependencies(${PROJECT_NAME} kctrl_stubs)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/kmod)
# room after the code segment for the SCE module data vita-elf-create appends (the data segment starts at the next
# 64 KB; when the code ended within ~3.6 KB of it, vita-elf-create failed: "segment 1 overlaps")
target_link_options(${PROJECT_NAME} PRIVATE "-Wl,--defsym,__sce_headroom=0x2000")
target_link_options(${PROJECT_NAME} PRIVATE "-Wl,-T,${RBX_ROOT}/tools/rbx_state.ld")
target_link_options(${PROJECT_NAME} PRIVATE "-Wl,-T,${RBX_ROOT}/tools/vita_room.ld")

target_link_libraries(${PROJECT_NAME}
  "${RBX_KSTUB}"
  rbx_core
  vita2d
  SceAudio_stub
  SceDisplay_stub
  SceGxm_stub
  SceSysmodule_stub
  SceAvPlayer_stub
  SceCtrl_stub
  ScePgf_stub
  ScePvf_stub
  SceCommonDialog_stub
  SceIme_stub
  SceAppMgr_stub
  SceAppUtil_stub
  ScePower_stub
  SceNet_stub
  SceNetCtl_stub
  SceShellSvc_stub
  SceRtc_stub
  SceTouch_stub
  SceLibKernel_stub
  taihen_stub
  mbedtls
  mbedx509
  mbedcrypto
  pthread
  png
  jpeg
  z
  m
  c
)

# keep the unstripped .elf as artifacts/<commit-hash>.elf (core dump analysis)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -DELF=$<TARGET_FILE:${PROJECT_NAME}> -DSRC=${RBX_ROOT}
          -P ${RBX_ROOT}/tools/archive_elf.cmake
  VERBATIM)

vita_create_self(eboot.bin ${PROJECT_NAME} UNSAFE)
# VPK: vita_create_vpk does not quote paths (this tree has a space), so the same steps are written out with VERBATIM.
set(RBX_SCE_SYS_FILES
  sce_sys/icon0.png
  sce_sys/pic0.png
  sce_sys/livearea/contents/bg.png
  sce_sys/livearea/contents/startup.png
  sce_sys/livearea/contents/template.xml
)
set(RBX_VPK_ADD_ARGS -a "${CMAKE_BINARY_DIR}/rbx_kctrl.skprx=module/rbx_kctrl.skprx"
  -a "${CMAKE_BINARY_DIR}/repad.skprx=module/repad.skprx"
  -a "${CMAKE_SOURCE_DIR}/res/cacert.pem=cacert.pem"
  -a "${CMAKE_SOURCE_DIR}/res/loading.png=loading.png")
set(RBX_VPK_DEPS "${CMAKE_BINARY_DIR}/rbx_kctrl.skprx" "${CMAKE_BINARY_DIR}/repad.skprx"
  "${CMAKE_SOURCE_DIR}/res/cacert.pem" "${CMAKE_SOURCE_DIR}/res/loading.png")
# the menus' languages (lang/<code>.json and the releases' v<version>-<code>.json): copied to the data folder when missing
file(GLOB RBX_LANGS CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/res/lang/*.json")
foreach(f ${RBX_LANGS})
  get_filename_component(n "${f}" NAME)
  list(APPEND RBX_VPK_ADD_ARGS -a "${f}=lang/${n}")
  list(APPEND RBX_VPK_DEPS "${f}")
endforeach()
# the game lists, also beside the VPK in the release zip: the app copies the missing ones to ux0:data/rebox/gamelist/
file(GLOB_RECURSE RBX_GAMELISTS CONFIGURE_DEPENDS RELATIVE "${CMAKE_SOURCE_DIR}/res/gamelist"
  "${CMAKE_SOURCE_DIR}/res/gamelist/*.json")
foreach(n ${RBX_GAMELISTS})
  set(f "${CMAKE_SOURCE_DIR}/res/gamelist/${n}")
  list(APPEND RBX_VPK_ADD_ARGS -a "${f}=gamelist/${n}")
  list(APPEND RBX_VPK_DEPS "${f}")
endforeach()
foreach(f ${RBX_SCE_SYS_FILES})
  list(APPEND RBX_VPK_ADD_ARGS -a "${CMAKE_SOURCE_DIR}/${f}=${f}")
  list(APPEND RBX_VPK_DEPS "${CMAKE_SOURCE_DIR}/${f}")
endforeach()

set(RBX_SFO "${CMAKE_BINARY_DIR}/param.sfo")
set(RBX_VPK "${CMAKE_BINARY_DIR}/rebox.vpk")
add_custom_command(OUTPUT "${RBX_SFO}"
  # ATTRIBUTE2 12: the enlarged memory mode (~109 MB more for the app, ADR-222: kof98umh's sprite chunks)
  COMMAND vita-mksfoex -s APP_VER=${VITA_VERSION} -s TITLE_ID=${VITA_TITLEID} -d ATTRIBUTE2=12
          "${VITA_APP_NAME}" "${RBX_SFO}"
  VERBATIM)
add_custom_command(OUTPUT "${RBX_VPK}"
  COMMAND vita-pack-vpk -s "${RBX_SFO}" -b "${CMAKE_BINARY_DIR}/eboot.bin"
          ${RBX_VPK_ADD_ARGS} "${RBX_VPK}"
  DEPENDS "${RBX_SFO}" "${CMAKE_BINARY_DIR}/eboot.bin" ${RBX_VPK_DEPS}
  VERBATIM)
add_custom_target(vpk ALL DEPENDS "${RBX_VPK}")
