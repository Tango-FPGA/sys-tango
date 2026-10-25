#!/usr/bin/env bash
# run_tb_ddr3_test.sh -- the DDR3 bring-up test's statistics RAM alone (sim/tb_ddr3_rmw.v: back-to-back
# read-modify-writes against a model), then the test's logic on the IP model (sim/tb_ddr3_test.v) in four configurations, each must print
# TB_RESULT PASS; then the base run's displayed bank through the results screen and the UART (sim/tb_ddr3_ui.v), and
# both must decode (ddr3/test/decode_d1.py) to that bank, all 114 slots.  A design file Verilator finds driven from
# two blocks (MULTIDRIVEN: GowinSynthesis stops on it, EX2000) fails the run.
#   base      the model as configured in the bench (RD_LAT 18, refresh 28 clocks every 780, 32-command FIFO)
#   fault     the model flips bit 37 of word 700 on every read: the checker must report exactly that
#   jitter    read latency +0..8, an executor stall of 300 clocks every 5000, pll_stop low 1 us every 200 us
#   utimeout  the UART's requests kept from clk_x1 for 1.6 ms: a line times out with the copy it has, a later one is
#             fresh, every line equals its copy
# Run from the sys-tango checkout; memory-capped by build_tools/tango_capped.sh.
set -eu
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; R=0
VFLAGS="--binary --timing --timescale 1ns/1ps -O2 -Isim -Wall -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME
        -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-UNOPTFLAT"
CAP="$ROOT/build_tools/tango_capped.sh"
export TANGO_MEM_CAP=${TANGO_MEM_CAP:-4G}
SRC="sim/tango_ddr3_model.v ddr3/test/tango_ddr3_rmw.v ddr3/test/tango_ddr3_tester.v ddr3/test/tango_ddr3_ov.v
     ddr3/test/tango_ddr3_uart.v ddr3/test/tango_ddr3_test_core.v sim/tb_ddr3_test.v"
multidriven() {   # $1 = a Verilator build log: MULTIDRIVEN in a ddr3/test file fails
    if grep -A2 'MULTIDRIVEN' "$1" | grep -q 'ddr3/test/'; then
        echo "   MULTIDRIVEN in the design:"; grep -A2 'MULTIDRIVEN' "$1" | grep 'ddr3/test/' | head; R=1
    fi
}
# the statistics RAM alone
OBJ=sim/obj_ddr3_rmw; mkdir -p "$OBJ"
nice -n 10 "$CAP" verilator $VFLAGS --Mdir "$OBJ" --top-module tb_ddr3_rmw ddr3/test/tango_ddr3_rmw.v sim/tb_ddr3_rmw.v \
    -o Vtb > "$OBJ/build.log" 2>&1 || { tail -30 "$OBJ/build.log"; exit 1; }
multidriven "$OBJ/build.log"
( cd "$OBJ" && timeout 600 "$CAP" ./Vtb ) > "$OBJ/run.log" 2>&1 || true
echo "== rmw"; grep -E '^TB ' "$OBJ/run.log" | head -12
if grep -q "TB_RESULT PASS" "$OBJ/run.log"; then echo "   rmw: PASS"; else echo "   rmw: FAIL"; tail -5 "$OBJ/run.log"; R=1; fi
for cfg in "base" "fault -DFAULT" "jitter -DJITTER -DPSTOP" "utimeout -DUTIMEOUT"; do
    set -- $cfg; NAME=$1; shift
    OBJ=sim/obj_ddr3_test_$NAME; mkdir -p "$OBJ"
    nice -n 10 "$CAP" verilator $VFLAGS "$@" --Mdir "$OBJ" --top-module tb_ddr3_test $SRC -o Vtb \
        > "$OBJ/build.log" 2>&1 || { tail -30 "$OBJ/build.log"; exit 1; }
    multidriven "$OBJ/build.log"
    ( cd "$OBJ" && timeout 3600 "$CAP" ./Vtb ) > "$OBJ/run.log" 2>&1 || true
    echo "== $NAME"; grep -E '^TB (FAIL|loops|calibrated)' "$OBJ/run.log" || true
    if grep -q "TB_RESULT PASS" "$OBJ/run.log"; then echo "   $NAME: PASS"; else echo "   $NAME: FAIL"; tail -5 "$OBJ/run.log"; R=1; fi
done
# the screen and the UART from the base run's bank
OBJ=sim/obj_ddr3_ui; mkdir -p "$OBJ"
BANK="$ROOT/sim/obj_ddr3_test_base/bank.txt"
if [ -f "$BANK" ]; then
    nice -n 10 "$CAP" verilator $VFLAGS --Mdir "$OBJ" --top-module tb_ddr3_ui ddr3/test/tango_ddr3_ov.v \
        ddr3/test/tango_ddr3_uart.v sim/tb_ddr3_ui.v -o Vtb > "$OBJ/build.log" 2>&1 || { tail -30 "$OBJ/build.log"; exit 1; }
    ( cd "$OBJ" && timeout 1800 "$CAP" ./Vtb +bank="$BANK" ) > "$OBJ/run.log" 2>&1 || true
    grep -q "TB_RESULT PASS" "$OBJ/run.log" || { echo "== ui: FAIL"; tail -5 "$OBJ/run.log"; exit 1; }
    python3 ddr3/test/decode_d1.py --json "$OBJ/frame.ppm" > "$OBJ/frame.json" || { echo "   frame: NO DECODE"; R=1; }
    python3 ddr3/test/decode_d1.py --json --uart "$OBJ/uart.txt" > "$OBJ/uart.json" || { echo "   uart: NO DECODE"; R=1; }
    python3 - "$OBJ" "$BANK" <<'PY' || R=1
import json, sys
obj, bankf = sys.argv[1], sys.argv[2]
bank = [int(l, 16) for l in open(bankf) if l.strip()]
ok = True
for name in ('frame', 'uart'):
    try:
        recs = json.load(open(f'{obj}/{name}.json'))
    except Exception:
        recs = []
    got = recs[0]['slots'] if recs else []
    diff = [i for i in range(len(bank)) if i >= len(got) or got[i] != bank[i]]
    print(f'== ui {name} decode vs bank: ' + ('EQUAL (114 slots)' if not diff else f'{len(diff)} slots differ: {diff[:10]}') +
          (f"  (offset {recs[0].get('offset')})" if recs and 'offset' in recs[0] else ''))
    ok &= not diff
sys.exit(0 if ok else 1)
PY
    python3 ddr3/test/check_capture_decode.py "$OBJ/frame.ppm" "$BANK" || R=1
    python3 ddr3/test/decode_d1.py --report "$OBJ/frame.ppm" | sed -n '/^  flags/,$p'
else
    echo "== ui: no bank from the base run"; R=1
fi
echo "run_tb_ddr3_test: $([ $R -eq 0 ] && echo PASS || echo FAIL)"
exit $R
