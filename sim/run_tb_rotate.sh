#!/usr/bin/env bash
# run_tb_rotate.sh -- the rotation client's benches (the three below, plus rotate_galaga / rotate_geo / rot_scale):
#   tb_rotate_beat    the allocator/beat FSM (no cport)           -> TB_RESULT PASS, bank/overwrite violations 0
#   tb_rotate_golden  pixel-exact address algebra (no cport)      -> TB_RESULT PASS, live per-write 0 mismatches
#   tb_rotate_arb     REAL tango_ddr3 (arbiter + cport) + the DDR3 IP model + 60 Template frames of V load
#                     -> TB_RESULT PASS; its numbers (writes, stall, occupancy, obs counters) must not move
#                     when only tango_ddr3_cport.v changes cycle-exactly
# Run from the sys-tango checkout (memory-capped by build_tools/tango_capped.sh).  Output goes to sim/obj_rotate_<bench>/{build,run}.log.
set -eu
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; R=0
CAP="$ROOT/build_tools/tango_capped.sh"; export TANGO_MEM_CAP=${TANGO_MEM_CAP:-4G}
V="-Wall -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-UNUSEDPARAM
   -Wno-BLKSEQ -Wno-MULTIDRIVEN -Wno-UNOPTFLAT -Wno-SYNCASYNCNET -Wno-CASEINCOMPLETE -Wno-PINMISSING -Wno-BLKANDNBLK
   -Wno-INITIALDLY -Wno-UNDRIVEN --binary --timing --timescale 1ns/1ps -O2 -Isim"
run() {   # bench top files...
    local B=$1; shift; local OBJ=sim/obj_$B; rm -rf "$OBJ"; mkdir -p "$OBJ"
    nice -n 10 "$CAP" verilator $V -DTANGO_DDRAM --Mdir "$OBJ" --top-module "tb_$B" "$@" -o Vtb > "$OBJ/build.log" 2>&1 \
        || { tail -30 "$OBJ/build.log"; return 2; }
    ( cd "$OBJ" && timeout 3000 "$CAP" ./Vtb ) > "$OBJ/run.log" 2>&1 || true
    grep -E '^(TB_ROTATE|TB_RESULT)' "$OBJ/run.log"
    grep -q "TB_RESULT PASS" "$OBJ/run.log"
}
for B in beat golden; do
    echo "== rotate_$B"; run "rotate_$B" tango_cdc.v tango_video_rotate_ddr3.v "sim/tb_rotate_$B.v" && echo "   $B: PASS" || { echo "   $B: FAIL"; R=1; }
done
echo "== rotate_arb"
run rotate_arb sim/tango_ddr3_model.v tango_cdc.v tango_ddr3_fifo.v tango_ddr3_vport.v tango_ddr3_cport.v tango_ddr3_arb.v \
    tango_ddr3.v tango_video_rotate_ddr3.v sim/tb_rotate_arb.v && echo "   arb: PASS" || { echo "   arb: FAIL"; R=1; }
grep -E 'TANGO CONTRACT' sim/obj_rotate_arb/run.log | sed 's/^/   /' | head -4
# Galaga's real raster through the real arbiter, every write's position checked; and the
# geometry (aspect/size) swap tango_video applies while the rotated framebuffer is shown
echo "== rotate_galaga"
run rotate_galaga sim/tango_ddr3_model.v tango_cdc.v tango_ddr3_fifo.v tango_ddr3_vport.v tango_ddr3_cport.v tango_ddr3_arb.v \
    tango_ddr3.v tango_video_rotate_ddr3.v sim/tb_rotate_galaga.v && echo "   galaga: PASS" || { echo "   galaga: FAIL"; R=1; }
echo "== rotate_geo"
run rotate_geo -I. tango_video_geo.v sim/tb_rotate_geo.v && echo "   geo: PASS" || { echo "   geo: FAIL"; R=1; }
# the arcade Scale entry (Normal / V-Integer; a stale 2/3 = Normal) -- also sim/run_tb_rot_scale.sh
echo "== rot_scale"
run rot_scale -I. tango_video_geo.v sim/tb_rot_scale.v && echo "   rot_scale: PASS" || { echo "   rot_scale: FAIL"; R=1; }
exit $R
