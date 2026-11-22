#!/usr/bin/env bash
# measure mushisam 3600 frames on the Vita: measure_cv.sh <eboot or -> <label> [switch files...]
# (needs the UDP logger writing out/udp_s24p2.txt; takes the device lock, restores the app and unlocks after)
cd "/e/Dev/Vita Port/ReBOX" || exit 1
EB="$1"; LABEL="$2"; shift 2
python vita/tools/vita.py lock 20 || exit 3
for f in "$@"; do python vita/tools/vita.py put out/empty.txt "ux0:/data/rebox/$f" >/dev/null; done
python vita/tools/vita.py put ${ST:-out/st_cv_long} ux0:/data/rebox/switchtest >/dev/null
: > out/udp_s24p2.txt
if [ "$EB" != "-" ]; then python vita/tools/vita.py deploy "$EB"; else python vita/tools/vita.py cmd "launch RBOX00001" >/dev/null; fi
sleep 25
for i in $(seq 1 80); do grep -A99999 "game 1/1" out/udp_s24p2.txt 2>/dev/null | grep -q "list:" && break; sleep 10; done
echo "== $LABEL"
python out/perfsum_play.py out/udp_s24p2.txt | grep steady
grep -E "prof:" out/udp_s24p2.txt | sed -n '3p;8p' | cut -c1-260
cp out/udp_s24p2.txt "out/udp_cv_$LABEL.txt"
for f in "$@"; do python vita/tools/vita.py rm "ux0:/data/rebox/$f" >/dev/null; done
python vita/tools/vita.py rm ux0:/data/rebox/switchtest >/dev/null
python vita/tools/vita.py cmd "launch RBOX00001" >/dev/null
python vita/tools/vita.py unlock
