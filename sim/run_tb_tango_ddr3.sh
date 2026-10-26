#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
# run_tb_tango_ddr3.sh -- tango_ddr3 (V port, core port, arbiter) on the IP model (sim/tb_tango_ddr3.v), in four
# configurations; each must print TB_RESULT PASS:
#   base        random V (ascal-exact) and C (MiSTer DDRAM, bursts 1..64, gaps) traffic, Q_CORE 8
#   adv8        the core streaming flat out (255-beat bursts back to back), Q_CORE 8
#   adv4 / adv16  the same with Q_CORE 4 / 16
#   precal      base, with the core out of reset and issuing long before the IP has calibrated: DDRAM_BUSY must hold it
# Run from the sys-tango checkout; memory-capped by build_tools/tango_capped.sh.
set -eu
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; R=0
CAP="$ROOT/build_tools/tango_capped.sh"; export TANGO_MEM_CAP=${TANGO_MEM_CAP:-4G}
SRC="sim/tango_ddr3_model.v tango_cdc.v tango_ddr3_fifo.v tango_ddr3_vport.v tango_ddr3_cport.v tango_ddr3_arb.v
     tango_ddr3.v sim/tb_tango_ddr3.v"
for cfg in "base" "adv8 -DADVERSARIAL" "adv4 -DADVERSARIAL -DQ_CORE=4" "adv16 -DADVERSARIAL -DQ_CORE=16" "precal -DPRECAL"; do
    set -- $cfg; NAME=$1; shift
    OBJ=sim/obj_tango_ddr3_$NAME; mkdir -p "$OBJ"
    nice -n 10 "$CAP" verilator --binary --timing --timescale 1ns/1ps -O2 -Isim -Wall -Wno-fatal -DTANGO_DDRAM "$@" \
        -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-BLKSEQ \
        -Wno-PINCONNECTEMPTY --Mdir "$OBJ" --top-module tb_tango_ddr3 $SRC -o Vtb > "$OBJ/build.log" 2>&1 \
        || { tail -30 "$OBJ/build.log"; exit 1; }
    ( cd "$OBJ" && timeout 3600 "$CAP" ./Vtb ) > "$OBJ/run.log" 2>&1 || true
    echo "== $NAME"; grep -E '^TB ' "$OBJ/run.log" | head -14
    grep -q "TB_RESULT PASS" "$OBJ/run.log" && echo "   $NAME: PASS" || { echo "   $NAME: FAIL"; R=1; }
done
exit $R
