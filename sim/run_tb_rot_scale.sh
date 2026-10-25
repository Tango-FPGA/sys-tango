#!/usr/bin/env bash
# run_tb_rot_scale.sh -- the arcade Scale entry (Normal / V-Integer only). sim/tb_rot_scale.v runs
# tango_video_scale_arc.vh + tango_video_rot_geo.vh (the functions tango_video includes) into the REAL tango_video_geo:
# arcade status 2/3 -> exactly the Normal window, 0/1 the recorded Normal / V-Integer windows (Galaga rotated: Normal
# 547x720 x 366..912, V-Integer 438x576 x 421..858 y 72..647), console raw 2/3 still the HV-Integer windows.
#   -> TB_RESULT PASS, "TB_ROT_SCALE: 28 checks, 0 mismatches"
# Run from the sys-tango checkout (seconds, a few MB; memory-capped by build_tools/tango_capped.sh). Output: sim/obj_rot_scale/{build,run}.log.
set -eu
cd "$(dirname "$0")/.."
ROOT="$(pwd)"
CAP="$ROOT/build_tools/tango_capped.sh"; export TANGO_MEM_CAP=${TANGO_MEM_CAP:-2G}
V="-Wall -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-UNUSEDPARAM
   -Wno-BLKSEQ --binary --timing --timescale 1ns/1ps -O2 -I. -Isim"
OBJ=sim/obj_rot_scale; rm -rf "$OBJ"; mkdir -p "$OBJ"
nice -n 10 "$CAP" verilator $V --Mdir "$OBJ" --top-module tb_rot_scale tango_video_geo.v sim/tb_rot_scale.v -o Vtb \
    > "$OBJ/build.log" 2>&1 || { tail -30 "$OBJ/build.log"; exit 2; }
( cd "$OBJ" && timeout 600 "$CAP" ./Vtb ) > "$OBJ/run.log" 2>&1 || true
grep -E '^(TB_ROT_SCALE|TB_RESULT)' "$OBJ/run.log"
grep -q "TB_RESULT PASS" "$OBJ/run.log"
