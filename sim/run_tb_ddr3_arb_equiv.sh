#!/usr/bin/env bash
# run_tb_ddr3_arb_equiv.sh [mutants] -- the lockstep equivalence bench (sim/tb_ddr3_arb_equiv.v) of tango_ddr3_arb against
# sim/ddr3_arb_ref (the arbiter before its timing fix).  Without an argument: the configurations must all
# print TB_RESULT PASS.  With `mutants`: each mutant of the tree's arbiter / queue (made on a copy) MUST print FAIL
# (a mutant that passes is a hole in the bench; the script exits 1).  Run from the sys-tango checkout
# (memory-capped by build_tools/tango_capped.sh).
set -eu
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; R=0
CAP="$ROOT/build_tools/tango_capped.sh"; export TANGO_MEM_CAP=${TANGO_MEM_CAP:-4G}
OPTS="--binary --timing --timescale 1ns/1ps -O2 -Isim -Wall -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME
      -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-PINCONNECTEMPTY -Wno-INITIALDLY -Wno-LATCH -Wno-CASEINCOMPLETE
      --top-module tb_ddr3_arb_equiv"
REF="sim/ddr3_arb_ref/tango_ddr3_skid_ref.v sim/ddr3_arb_ref/tango_ddr3_arb_ref.v"
build_run() {   # name dir defs...
    local NAME=$1 D=$2; shift 2
    local OBJ=sim/obj_arb_equiv_$NAME; rm -rf "$OBJ"; mkdir -p "$OBJ"
    nice -n 10 "$CAP" verilator $OPTS "$@" --Mdir "$OBJ" sim/tango_ddr3_model.v $D/tango_ddr3_fifo.v $D/tango_ddr3_arb.v $REF \
        sim/tb_ddr3_arb_equiv.v -o Vtb > "$OBJ/build.log" 2>&1 || { tail -30 "$OBJ/build.log"; return 2; }
    ( cd "$OBJ" && timeout 3000 "$CAP" ./Vtb ) > "$OBJ/run.log" 2>&1 || true
    grep -E '^(TB|EQUIV|TANGO CONTRACT)' "$OBJ/run.log" | grep -v "OK:" | head -12
    grep -c "TANGO CONTRACT OK" "$OBJ/run.log" | sed 's/^/   contract checks that finished clean: /'
    grep -q "TB_RESULT PASS" "$OBJ/run.log"
}
if [ "${1:-}" != mutants ]; then
    for cfg in "nc1 -DNC=1" "nc2 -DNC=2 -DSEED=7" "q4 -DQ_CORE=4 -DSEED=3" "g2tag4 -DGV=2 -DTAGL=4 -DSEED=5 -DFENCEW=16" "nc2q6 -DNC=2 -DQ_CORE=6 -DGV=3 -DSEED=11"; do
        set -- $cfg; NAME=$1; shift
        echo "== $NAME"
        build_run "$NAME" . "$@" && echo "   $NAME: PASS" || { echo "   $NAME: FAIL"; R=1; }
    done
    exit $R
fi
# ---- mutants of the fix: each must be caught
MD=sim/obj_arb_equiv_mut_src; rm -rf "$MD"; mkdir -p "$MD"
python3 - "$MD" <<'PY'
import sys,re
d=sys.argv[1]
arb=open('tango_ddr3_arb.v').read(); fifo=open('tango_ddr3_fifo.v').read()
M={}
def mut(name,src,a,b):
    assert src.count(a)==1,(name,a)
    M[name]=(src.replace(a,b),src is arb)
mut('swap_inflight_candidates',arb,"wire [15:0]   inflight_n = gc ? infl_p1[15:0] : inflight_r;","wire [15:0]   inflight_n = gc ? inflight_r : infl_p1[15:0];")
mut('inflight_candidate_without_return',arb,"{1'b0, inflight_r} + 17'd1;","{1'b0, inflight} + 17'd1;")
mut('stale_gc_selects_inflight',arb,"wire [15:0]   inflight_n = gc ? infl_p1[15:0] : inflight_r;","reg gc_q = 0; always @(posedge clk) gc_q <= gc;\n    wire [15:0]   inflight_n = gc_q ? infl_p1[15:0] : inflight_r;")
mut('pend_candidates_swapped',arb,"wire [7:0]    ret_n  = gc ? pend_p1 : pend;","wire [7:0]    ret_n  = gc ? pend : pend_p1;")
mut('room_gc_off_by_one',arb,"(infl_p1 < Q_CORE);","(infl_p1 <= Q_CORE);")
mut('room_ngc_uses_inflight',arb,"({1'b0, inflight_r} < Q_CORE);","({1'b0, inflight} < Q_CORE);")
mut('vlim_inc_uses_hold',arb,"(vs_p1 >= G);","(vstreak >= G);")
mut('vlim_clear_dropped',arb,"vlim <= gc ? lim_clr : gv ? (cany ? lim_inc : lim_clr) : lim_hold;","vlim <= gc ? lim_hold : gv ? (cany ? lim_inc : lim_clr) : lim_hold;")
mut('wb_pop_stay_swapped',fifo,"out_wb <= out_pop ? wb_pop : wb_stay;","out_wb <= out_pop ? wb_stay : wb_pop;")
mut('wb_empty_keeps_head_flag',fifo,"wire wb_stay /* synthesis syn_keep = 1 */ = (n == 2'd0) ? in_data[WB] : out_wb;","wire wb_stay /* synthesis syn_keep = 1 */ = out_wb;")
mut('wb_pop_of_two_takes_pushed',fifo,"= (n == 2'd2) ? out_wo : in_data[WB];","= in_data[WB];")
mut('wb_pop_of_one_keeps_head',fifo,"= (n == 2'd2) ? out_wo : in_data[WB];","= (n == 2'd2) ? out_wo : out_wb;")
mut('wb_second_flag_never_set',fifo,"if (n == 2'd1) out_wo <= in_data[WB];","")
mut('wb_second_flag_set_when_empty',fifo,"if (n == 2'd1) out_wo <= in_data[WB];","if (n == 2'd0) out_wo <= in_data[WB];")
mut('wb_second_flag_set_when_two',fifo,"if (n == 2'd1) out_wo <= in_data[WB];","if (n == 2'd2) out_wo <= in_data[WB];")
# a queue mutant must also be caught by the LOCKSTEP compare, not only by the queue's own contract check: that check is
# switched off in its copy
for n in list(M):
    t,isarb=M[n]
    if not isarb: M[n]=(t.replace("if (out_valid && out_wb !== out_data[WB]) begin","if (1'b0) begin"),isarb)
for n,(t,isarb) in M.items():
    open(f'{d}/{n}.arb.v','w').write(t if isarb else arb)
    open(f'{d}/{n}.fifo.v','w').write(fifo if isarb else t)
open(f'{d}/list','w').write('\n'.join(M))
PY
for n in $(cat "$MD/list"); do
    mkdir -p "$MD/$n"; cp "$MD/$n.arb.v" "$MD/$n/tango_ddr3_arb.v"; cp "$MD/$n.fifo.v" "$MD/$n/tango_ddr3_fifo.v"
    echo "== mutant $n"
    rc=0; build_run "mut_$n" "$MD/$n" -DNC=1 -DSEED=2 -DNCLK=200000 > "$MD/$n.out" 2>&1 || rc=$?
    if [ $rc = 0 ]; then echo "   $n: SURVIVED (PASS) -- a hole in the bench"; R=1
    elif [ $rc = 2 ]; then echo "   $n: BUILD ERROR (not a verdict)"; R=1
    else echo "   $n: caught: $(grep -E 'EQUIV MISMATCH|CONTRACT: |TB coverage' "$MD/$n.out" | head -1)"; fi
done
exit $R
