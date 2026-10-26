#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
# run_tb_tango_ascal_words.sh -- tango_ascal_words against an ascal-exact master and a stalling word memory
# (sim/tb_tango_ascal_words.v), in four configurations; every one must print TB_RESULT PASS.
#   base       ascal's traffic (32-beat bursts, all byte enables); the memory takes 60 % of clocks, reads 2..17 clocks
#   be_len     + random byte enables and burst lengths 1..32 (legal Avalon beyond what ascal does)
#   slow       the memory takes 15 % of clocks, reads 8..23 clocks late
#   fast       the memory takes every clock it is not stalled (100 %), reads 2..17 clocks
# Run from the sys-tango checkout (or a core's sys/).  Memory-capped by build_tools/tango_capped.sh.
set -eu
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; R=0
for cfg in "base" "be_len -DRANDOM_BE -DRANDOM_LEN" "slow -DWTAKE_PCT=15 -DRLAT_MIN=8" "fast -DWTAKE_PCT=100"; do
    set -- $cfg; NAME=$1; shift
    OBJ=sim/obj_ascal_words_$NAME; mkdir -p "$OBJ"
    TANGO_MEM_CAP=${TANGO_MEM_CAP:-3G} nice -n 10 "$ROOT/build_tools/tango_capped.sh" \
        verilator --binary --timing -O2 -Wall -Wno-fatal "$@" \
        -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-BLKSEQ \
        --Mdir "$OBJ" --top-module tb_tango_ascal_words tango_ascal_words.v sim/tb_tango_ascal_words.v \
        -o Vtb > "$OBJ/build.log" 2>&1 || { tail -20 "$OBJ/build.log"; exit 1; }
    "$OBJ/Vtb" > "$OBJ/run.log" 2>&1 || true
    echo "== $NAME: $(grep -E '^TB ' "$OBJ/run.log" | tr '\n' ' ')"
    grep -h "TB cycle" "$OBJ/run.log" | head -5
    grep -q "TB_RESULT PASS" "$OBJ/run.log" && echo "   $NAME: PASS" || { echo "   $NAME: FAIL"; R=1; }
done
exit $R
