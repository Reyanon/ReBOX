#!/usr/bin/env bash
# Build helper (Linux/WSL).
#   tools/build.sh vita [Release|Debug] [cmake args...]  -> build-vita/ (or BUILD_DIR)
#   tools/build.sh pc                                   -> build-pc/harness
# On Windows: wsl -d Ubuntu-24.04 -- bash tools/build.sh vita
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export VITASDK="${VITASDK:-$HOME/vitasdk}"
export PATH="$VITASDK/bin:$PATH"
GEN=()
command -v ninja >/dev/null 2>&1 && GEN=(-G Ninja)
case "${1:-}" in
vita)
	TYPE="${2:-Release}"
	shift $(( $# >= 2 ? 2 : 1 ))
	DIR="${BUILD_DIR:-build-vita}"
	# a failed build must not leave the previous eboot.bin to be deployed
	[ -f "$DIR/eboot.bin" ] && mv -f "$DIR/eboot.bin" "$DIR/eboot.bin.prev"
	cmake -S vita -B "$DIR" "${GEN[@]}" -DCMAKE_TOOLCHAIN_FILE="$VITASDK/share/vita.toolchain.cmake" -DCMAKE_BUILD_TYPE="$TYPE" "$@"
	if ! cmake --build "$DIR" -j 2>&1 | tee "$DIR/build.log"; then
		# vita-elf-create needs ~4KB free after the code segment, which ends anywhere in the 64KB before the data
		# segment: move its end by 8KB (or back) and build again (as ReSys12)
		grep -q "segment 1 overlaps" "$DIR/build.log" || exit 1
		PAD=$(sed -n 's/^RBX_TEXT_PAD:STRING=//p' "$DIR/CMakeCache.txt")
		[ "${PAD:-0}" = 0 ] && PAD=8192 || PAD=0
		echo "build.sh: no room for the SCE data after the code segment, padding $PAD"
		cmake -S vita -B "$DIR" -DRBX_TEXT_PAD="$PAD"
		cmake --build "$DIR" -j
	fi
	;;
inst)
	cmake -S vita/installer -B build-inst "${GEN[@]}" -DCMAKE_TOOLCHAIN_FILE="$VITASDK/share/vita.toolchain.cmake"
	cmake --build build-inst -j
	;;
pc)
	cmake -S tools/harness -B build-pc "${GEN[@]}"
	cmake --build build-pc -j
	;;
*)
	sed -n '2,6p' "$0" >&2
	exit 2
	;;
esac
