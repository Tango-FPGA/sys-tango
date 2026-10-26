#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
# run_tb_pad_decode.sh -- unit bench of tango_pad_decode.v: replays sim/pads/pad_vectors.txt, which the firmware's (menu-tango's)
# bl616/tango/test/pd_vectors.c writes from its C reference decode, plus hand-written scenarios.  Prints "PAD_DECODE PASS".
# Small (one module, ~20 s to build; memory-capped by build_tools/tango_capped.sh).  Run from the sys-tango checkout (or a core's sys/).
#   regenerate the vectors:  make -C <menu-tango>/bl616/tango/test pd_vectors && <menu-tango>/bl616/tango/test/pd_vectors > sim/pads/pad_vectors.txt
set -eu
cd "$(dirname "$0")/../.."
ROOT="$(pwd)"; OBJ=sim/obj_pad_decode
mkdir -p "$OBJ"
TANGO_MEM_CAP=${TANGO_MEM_CAP:-2G} nice -n 10 "$ROOT/build_tools/tango_capped.sh" \
    verilator --cc --exe --build -O2 -Wall -Wno-fatal -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
    -Mdir "$OBJ" --top-module tango_pad_decode "$ROOT/tango_pad_decode.v" "$ROOT/sim/pads/tb_pad_decode.cpp" -o Vpad_decode > "$OBJ/build.log" 2>&1 \
    || { tail -30 "$OBJ/build.log"; exit 1; }
"$OBJ/Vpad_decode" sim/pads/pad_vectors.txt | tee "$OBJ/run.log"
grep -q "PAD_DECODE PASS" "$OBJ/run.log"

# second bench: the engine with the built-in DS4 table against the hardwired decode of tango_ds4_joy.v, same reports
OBJ2=sim/obj_ds4_equiv; mkdir -p "$OBJ2"
TANGO_MEM_CAP=${TANGO_MEM_CAP:-2G} nice -n 10 "$ROOT/build_tools/tango_capped.sh" \
    verilator --cc --exe --build -O2 -Wall -Wno-fatal -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDPARAM \
    --public-flat-rw -Mdir "$OBJ2" --top-module tb_ds4_equiv "$ROOT/tango_pad_decode.v" "$ROOT/tango_ds4_joy.v" "$ROOT/sim/pads/tb_ds4_equiv.v" \
    "$ROOT/sim/pads/tb_ds4_equiv.cpp" -o Vds4_equiv > "$OBJ2/build.log" 2>&1 || { tail -30 "$OBJ2/build.log"; exit 1; }
"$OBJ2/Vds4_equiv" sim/pads/pad_vectors.txt | tee "$OBJ2/run.log"
grep -q "DS4_EQUIV PASS" "$OBJ2/run.log"
