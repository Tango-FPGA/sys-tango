#!/usr/bin/env bash
# run_tb_cport_equiv.sh [selfcheck|mutants] -- the lockstep equivalence bench (sim/tb_cport_equiv.v) of tango_ddr3_cport
# against sim/cport_ref (the core port before its timing fix).
#   (no argument)  the configurations (core clock slower / equal / faster than clk_ddr, other seeds): all must print TB_RESULT PASS
#   selfcheck      the bench against itself (the reference as the DUT): must PASS -- proves the bench's own stimulus is sane
#   mutants        each mutant of the tree's cport (made on a copy, its own contract checks switched off so the LOCKSTEP compare has
#                  to catch it) MUST be caught (a mutant that passes is a hole in the bench; the script exits 1)
# Run from the sys-tango checkout; memory-capped by build_tools/tango_capped.sh.
set -eu
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; R=0
CAP="$ROOT/build_tools/tango_capped.sh"; export TANGO_MEM_CAP=${TANGO_MEM_CAP:-4G}
OPTS="--binary --timing --timescale 1ns/1ps -O2 -Isim -Wall -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME
      -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-PINCONNECTEMPTY -Wno-INITIALDLY -Wno-LATCH -Wno-CASEINCOMPLETE
      -Wno-SIDEEFFECT -Wno-MULTIDRIVEN -Wno-UNOPTFLAT --top-module tb_cport_equiv"
REF="sim/cport_ref/tango_ddr3_cport_ref.v"
build_run() {   # name dut_file defs...
    local NAME=$1 DUT=$2; shift 2
    local OBJ=sim/obj_cport_equiv_$NAME; rm -rf "$OBJ"; mkdir -p "$OBJ"
    nice -n 10 "$CAP" verilator $OPTS "$@" --Mdir "$OBJ" tango_cdc.v "$DUT" $REF sim/tb_cport_equiv.v -o Vtb > "$OBJ/build.log" 2>&1 \
        || { tail -30 "$OBJ/build.log"; return 2; }
    ( cd "$OBJ" && timeout 3000 "$CAP" ./Vtb ) > "$OBJ/run.log" 2>&1 || true
    grep -E '^(TB|TANGO CONTRACT:)' "$OBJ/run.log" | head -14
    grep -c "TANGO CONTRACT OK" "$OBJ/run.log" | sed 's/^/   contract checks that finished clean: /'
    grep -q "TB_RESULT PASS" "$OBJ/run.log"
}
CFGS="c74 -DCORE_HP=6.734 -DSEED=1
c212 -DCORE_HP=2.35 -DSEED=2
c28 -DCORE_HP=18 -DSEED=3
c100 -DCORE_HP=5 -DSEED=4
c55 -DCORE_HP=9.1 -DSEED=5 -DNCLK=900000"
echo "$CFGS" > sim/obj_cport_cfgs.txt
if [ "${1:-}" = selfcheck ]; then
    # the reference's own text under the DUT's module name
    sed 's/^module tango_ddr3_cport_ref #(/module tango_ddr3_cport #(/' "$REF" > sim/obj_cport_selfcheck_dut.v
    while read -r cfg; do
        set -- $cfg; NAME=$1; shift
        echo "== selfcheck $NAME"
        build_run "self_$NAME" sim/obj_cport_selfcheck_dut.v "$@" && echo "   $NAME: PASS" || { echo "   $NAME: FAIL"; R=1; }
    done < sim/obj_cport_cfgs.txt
    exit $R
fi
if [ "${1:-}" != mutants ]; then
    while read -r cfg; do
        set -- $cfg; NAME=$1; shift
        echo "== $NAME"
        build_run "$NAME" tango_ddr3_cport.v "$@" && echo "   $NAME: PASS" || { echo "   $NAME: FAIL"; R=1; }
    done < sim/obj_cport_cfgs.txt
    exit $R
fi
# ---- mutants of the change: each must be caught
MD=sim/obj_cport_mut_src; rm -rf "$MD"; mkdir -p "$MD"
python3 - "$MD" <<'PY'
import sys,re
d=sys.argv[1]
src=open('tango_ddr3_cport.v').read()
# the contract checks off: the lockstep compare must catch each mutant by itself
src=re.sub(r"    /\* synthesis translate_off \*/.*?/\* synthesis translate_on \*/\n", "", src, flags=re.S)
M={}
def mut(name,a,b,cnt=1):
    assert src.count(a)==cnt,(name,a,src.count(a))
    M[name]=src.replace(a,b)
mut('ri_write_next_slot',"ri[ri_wp[4:0]] <= {l0, l1};","ri[ri_wp[4:0] + 5'd1] <= {l0, l1};")
mut('ri_write_also_next_slot',"ri[ri_wp[4:0]] <= {l0, l1};","ri[ri_wp[4:0]] <= {l0, l1};\n        ri[ri_wp[4:0] + 5'd1] <= {l0, l0};")   # overwrites the OLDEST held entry when 31 are held: what ri_ok must prevent
mut('ri_write_wrong_lanes',"ri[ri_wp[4:0]] <= {l0, l1};","ri[ri_wp[4:0]] <= {l0, l0};")
mut('ri_ok_threshold_plus1_all',"< 6'd31);","< 6'd32);",3)
mut('ri_ok_threshold_plus1_idle',"wire       ric00 /* synthesis syn_keep = 1 */ = ((ri_wp - ri_rp) < 6'd31);","wire       ric00 /* synthesis syn_keep = 1 */ = ((ri_wp - ri_rp) < 6'd32);")
mut('ri_ok_candidates_swapped',"ri_ok  <= r_go ? (r_valid ? ric00 : ric10) : (r_valid ? ric01 : ric00);","ri_ok  <= r_go ? (r_valid ? ric00 : ric01) : (r_valid ? ric10 : ric00);")
mut('ri_ok_ignores_r_valid',"ri_ok  <= r_go ? (r_valid ? ric00 : ric10) : (r_valid ? ric01 : ric00);","ri_ok  <= r_go ? ric10 : ric00;")
mut('rf_res_candidates_swapped',"rf_res <= r_go ? rfr_g : rfr_n;","rf_res <= r_go ? rfr_n : rfr_g;")
mut('rf_res_ignores_return',"wire [RF_LOG2+1:0] rfr_g /* synthesis syn_keep = 1 */ = rf_res + 1'b1 - (rf_we ? 1'b1 : 1'b0);","wire [RF_LOG2+1:0] rfr_g /* synthesis syn_keep = 1 */ = rf_res + 1'b1;")
mut('ract_not_cleared_on_last',"if (lastw) ract <= 1'b0;","")
mut('wact_not_cleared_on_last',"if (lastw) wact <= 1'b0;","")
mut('ract_set_ignores_count0',"ract <= !cf_q[33] && nz_c;","ract <= !cf_q[33];")
mut('wact_set_ignores_count0',"wact <= cf_q[33] && nz_c;","wact <= cf_q[33];")
mut('acc_full_not_updated_on_pop',"acc_full <= ({6'd0, acc_n} + 9'd1 == nb);","")
mut('acc_full_pop_compares_acc_n',"acc_full <= ({6'd0, acc_n} + 9'd1 == nb);","acc_full <= ({6'd0, acc_n} == nb);")
mut('last_after_go_lt',"wire       last_m = (sn_m <= 9'd4);","wire       last_m = (sn_m < 9'd4);")
mut('next_word_max_3',"wire [8:0] nb_m   = (sn_m < 9'd4) ? sn_m : 9'd4;","wire [8:0] nb_m   = (sn_m < 9'd3) ? sn_m : 9'd3;")
mut('last_after_cmd_lt',"wire       last_c = (n_c <= 9'd4 - {7'd0, lo_c});","wire       last_c = (n_c < 9'd4 - {7'd0, lo_c});")
mut('last_after_cmd_ignores_lane',"wire       last_c = (n_c <= 9'd4 - {7'd0, lo_c});","wire       last_c = (n_c <= 9'd4);")
mut('sa_step_two_bits',"wire [24:0] sa_m  = sa + {22'd0, nb[2:0]};","wire [24:0] sa_m  = sa + {23'd0, nb[1:0]};")
mut('pop_needs_full_skid',"assign pop_b = wact && !acc_full && (skn != 0);","assign pop_b = wact && !acc_full && (skn == 2'd2);")
mut('w_go_ignores_q_free',"wire w_go    = wact && acc_full && q_free;","wire w_go    = wact && acc_full;")
mut('r_go_ignores_rf_ok',"wire r_go    = ract && q_free && rf_ok && ri_ok;","wire r_go    = ract && q_free && ri_ok;")
mut('busy_cleared_when_not_last',"if (lastw) busy <= 1'b0;","if (!lastw) busy <= 1'b0;")
mut('lastw_not_updated_on_go',"sa <= sa_m; sn <= sn_m; nb <= nb_m; lastw <= last_m;","sa <= sa_m; sn <= sn_m; nb <= nb_m;")
for n,t in M.items(): open(f"{d}/{n}.v","w").write(t)
print(len(M),"mutants")
PY
LOGM=sim/obj_cport_mut_log.txt; : > "$LOGM"
for f in $(ls "$MD"/*.v | sort); do
    N=$(basename "$f" .v); CAUGHT=0
    while read -r cfg; do
        set -- $cfg; NAME=$1; shift
        OBJ=sim/obj_cport_equiv_m_$N; rm -rf "$OBJ"; mkdir -p "$OBJ"
        nice -n 10 "$CAP" verilator $OPTS "$@" -DNCLK=250000 --Mdir "$OBJ" tango_cdc.v "$f" $REF sim/tb_cport_equiv.v -o Vtb > "$OBJ/build.log" 2>&1 \
            || { echo "   $N: BUILD FAILED"; tail -5 "$OBJ/build.log"; CAUGHT=-1; break; }
        ( cd "$OBJ" && timeout 1200 "$CAP" ./Vtb ) > "$OBJ/run.log" 2>&1 || true
        if grep -qE "TB EQUIV MISMATCH|TB FUNC" "$OBJ/run.log"; then CAUGHT=1; echo "   $N: caught by $NAME ($(grep -m1 'TB EQUIV MISMATCH' "$OBJ/run.log" | cut -c1-60))" | tee -a "$LOGM"; break; fi
        rm -rf "$OBJ"
    done < sim/obj_cport_cfgs.txt
    [ $CAUGHT -eq 1 ] || { echo "   $N: NOT CAUGHT (a hole in the bench, or an equivalent mutant)" | tee -a "$LOGM"; R=1; }
done
echo "mutants: $(grep -c caught "$LOGM") caught"
exit $R
