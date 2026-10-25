#!/usr/bin/env bash
# run_tb_ul_lim_equiv.sh -- build and run sim/tb_ul_lim_equiv.v against the REAL tango_ul_lim module in
# tango_link.v, then run the mutant set (each mutant is a python edit of a COPY of tango_link.v; the
# repo file is never modified) and require every non-equivalent mutant to be caught (bench exit != 0).
#   usage: sim/run_tb_ul_lim_equiv.sh [objdir]      (default: a fresh mktemp dir; RANDN=<n> to resize PART 4)
# Light job (one CPU thread, ~15 s per build+run).
set -eu
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; OBJ="${1:-$(mktemp -d)}"; mkdir -p "$OBJ"
RANDN="${RANDN:-20000000}"; MUT_RANDN="${MUT_RANDN:-2000000}"   # PART 4 size: full for the baseline, smaller per mutant
VFLAGS="--binary --timing -Wno-fatal -Wno-WIDTH -Wno-UNSIGNED -Wno-LITENDIAN -DTANGO_IOCTL_UPLOAD --top-module tb_ul_lim_equiv"
build_run () {   # <name> <tango_link.v path> <RANDN>
    local d="$OBJ/$1"; mkdir -p "$d"
    verilator $VFLAGS +define+RANDN=$3 -I"$ROOT" -Mdir "$d" "$ROOT/sim/tb_ul_lim_equiv.v" "$2" -o eq >"$d/build.log" 2>&1 || { echo "BUILD FAIL $1 (see $d/build.log)"; return 99; }
    # Verilator --binary does not propagate $finish(n) as the exit status: the verdict is the printed line
    "$d/eq" >"$d/run.log" 2>&1 || true
    grep -q '^EQUIV SWEEP: PASS' "$d/run.log" || return 1
}
echo "== baseline (the repo's tango_link.v) =="
set +e; build_run base "$ROOT/tango_link.v" "$RANDN"; RC=$?; set -e
grep -E "^(PART|ul_lim VALUE|EQUIV)" "$OBJ/base/run.log"
[ $RC -eq 0 ] || { echo "BASELINE FAIL (rc $RC) -- $OBJ/base/run.log"; exit 1; }
FAIL=0
mutate () {   # <name> <expect: catch|survive> <python-expr on s>
    mkdir -p "$OBJ/mut_$1"
    python3 - "$ROOT/tango_link.v" "$OBJ/mut_$1/tango_link.v" "$3" <<'PY'
import sys
s = open(sys.argv[1]).read(); code = sys.argv[3]
t = eval(code, {'s': s})
assert t != s, 'mutation did not change the file: ' + code
open(sys.argv[2], 'w').write(t)
PY
    set +e; build_run "mut_$1" "$OBJ/mut_$1/tango_link.v" "$MUT_RANDN"; local rc=$?; set -e
    local why; why=$(grep -E "^EQUIV" "$OBJ/mut_$1/run.log" || true)
    if [ "$2" = catch ]; then
        if [ $rc -ne 0 ] && [ $rc -ne 99 ]; then echo "mutant $1: CAUGHT   ($why)"; else echo "mutant $1: *** SURVIVED (should be caught) rc=$rc"; FAIL=1; fi
    else
        if [ $rc -eq 0 ]; then echo "mutant $1: survives, as expected (equivalent mutant)"; else echo "mutant $1: *** unexpectedly caught/failed rc=$rc"; FAIL=1; fi
    fi
}
echo "== mutants =="
mutate sz4_plus3        catch   "s.replace(\"ul_sz4 <= ul_size_o + 24'd4;\", \"ul_sz4 <= ul_size_o + 24'd3;\")"
mutate sz4_plus5        catch   "s.replace(\"ul_sz4 <= ul_size_o + 24'd4;\", \"ul_sz4 <= ul_size_o + 24'd5;\")"
mutate sz4_23bit        catch   "s.replace(\"ul_sz4 <= ul_size_o + 24'd4;\", \"ul_sz4 <= {1'b0, ul_size_o[22:0] + 23'd4};\")"
mutate no_clamp         catch   "s.replace(\"ul_lim <= ul_sub[24] ? 24'd0 : ul_sub[23:0];\", \"ul_lim <= ul_sub[23:0];\")"
mutate clamp_to_1       catch   "s.replace(\"ul_sub[24] ? 24'd0 :\", \"ul_sub[24] ? 24'd1 :\")"
mutate borrow_bit23     catch   "s.replace(\"ul_lim <= ul_sub[24] ?\", \"ul_lim <= ul_sub[23] ?\")"
mutate operands_swapped catch   "s.replace(\"{1'b0, ul_sz4} - {1'b0, ul_off}\", \"{1'b0, ul_off} - {1'b0, ul_sz4}\")"
mutate clamp_at_diff1   catch   "s.replace(\"ul_lim <= ul_sub[24] ?\", \"ul_lim <= (ul_sub[24] || ul_sub[23:0] == 24'd1) ?\")"
mutate lag3_extra_stage catch   "s.replace(\"ul_sz4 <= ul_size_o + 24'd4;\", \"ul_sz4p <= ul_size_o; ul_sz4 <= ul_sz4p + 24'd4;\").replace(\"reg  [23:0] ul_sz4 = 24'd4;\", \"reg  [23:0] ul_sz4 = 24'd4; reg [23:0] ul_sz4p = 24'd0;\")"
mutate lag1_stage_dropped catch "s.replace(\"{1'b0, ul_sz4} - {1'b0, ul_off}\", \"{1'b0, ul_size_o + 24'd4} - {1'b0, ul_off}\")"
mutate clamp_at_diff0   survive "s.replace(\"ul_lim <= ul_sub[24] ?\", \"ul_lim <= (ul_sub[24] || ul_sub[23:0] == 24'd0) ?\")"
[ $FAIL -eq 0 ] && echo "UL_LIM EQUIV BENCH + MUTANTS: PASS" || { echo "UL_LIM EQUIV BENCH + MUTANTS: FAIL"; exit 1; }
