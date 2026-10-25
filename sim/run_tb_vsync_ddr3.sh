#!/usr/bin/env bash
# run_tb_vsync_ddr3.sh [delay ...] -- ascal at a native rate (build-time raster totals): sim/tb_ascal_ddr3.v (the
# REAL ascal netlist, tango_ddr3 on the calibrated IP model) with TB_VSYNC_CORE (sim/tb_vsync_core.v: a progressive
# source of exactly LINES x 4719 clk_pix a frame -- the TG16's frame at the exact clock) and the raster at HT x VT
# (tango_cfg.vh TANGO_HDMI_HTOTAL / _VTOTAL; default 1657 x 749, the TG16's 263-line totals: 4 clk_pix a frame shorter
# than the source, +3.2 ppm).  One run per start delay of the source (default: 6 phases across a frame); gate per run:
# MON_RESULT PASS (data, protocol, the read pattern, and every judged output frame showing exactly ONE new input frame)
# and "TANGO VIDEO OK" (ascal aligned once, every later output VS on target, period HT x VT).
# NEG=1: the negative controls instead -- (a) the bench's beam at 1650 x 750 while tango_sys runs HT x VT (the alignment
# rule must report), (b) a source with CE every 11 clk_pix (363 x 11 x 263 = 1,050,159 clk_pix a frame, 18 % faster, its VS
# intact): dropped frames must be counted (MON input frames ... 2+ (drop) > 0); both must FAIL.
# STAGE2=1 (the runtime raster totals): TANGO_VSYNC_ADJUST (+ TANGO_AUDIO_FILTER for the link tap) with the fixed
# raster 1650 x 750: the source starts at 262 lines and switches to 263 at its frame SW (default 30); tango_vsync must
# measure 1,236,378 then 1,241,097 clk_pix, plan 1664 x 743 then 1657 x 749, and every output frame of a native raster
# (after its first two) must show exactly one new input frame (sim/check_vsync_stage2.py; NF default 64).
# STAGE1=1 [PLL=745875] (the build-time HDMI PLL presets): STAGE2's run at the retuned pixel clock (tango_cfg.vh
# TANGO_HDMI_PLL_<f>: clk_pix at exactly 74.5875 MHz) with the source on its OWN clock (TB_CORE_CLK: the TG16's 945/11 MHz,
# 1365 master clocks a line -- no longer a whole number of clk_pix) and the fixed raster 1662 x 748 (59.99754 Hz): tango_vsync
# must measure 1,241,998 then 1,246,738 clk_pix (+-1; exact 1241997.9 / 1246738.35) and plan 1656 x 750 then 1669 x 747
# (-1.69 / -3.73 ppm), the same frame accounting (OBJ default sim/obj_vsync_s1).
# PAUSE=<line> [PAUSE_US=18000] (the retune at core start, the clock-pause design): STAGE1's build (74.5875 MHz,
# the source on its own clock) with a constant 263-line source; at output frame 26 clk_pix stops at beam line <line> for
# PAUSE_US and resumes (tb_ascal_ddr3.v TB_PAUSE): the plan 1669 x 747 and every judged frame's accounting must hold
# (CHECK_VSYNC_STAGE2 with the paused frames skipped).  730 (vertical blanking, no read in flight) must PASS; an active line
# (PAUSE=300 PAUSE_X=1400: just after ascal starts the next line's reads at HS) is the negative control and must FAIL (ascal's
# read handshake across a stopped o_clk).  OBJ sim/obj_vsync_p<line>.
# Env: HT VT LINES NF (default 16) OBJ (default sim/obj_vsync_ddr3).  Heavy job: memory-capped by build_tools/tango_capped.sh.
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; OBJ=${OBJ:-sim/obj_vsync_ddr3}; NF=${NF:-16}; HT=${HT:-1657}; VT=${VT:-749}; LINES=${LINES:-263}
S2DEFS=""; S2FILES=""
WANT="1664x743,1236378 1657x749,1241097"
[ -n "${PAUSE:-}" ] && { STAGE1=1; OBJ_S1=${OBJ_P:-sim/obj_vsync_p$PAUSE}; NF_S2=${NF_S2:-44}; }
# STAGE2C=1: STAGE2 (74.25 MHz, fixed raster 1650 x 750) with the source on its OWN 945/11 MHz clock (TB_CORE_CLK) -- the control
# for STAGE1's async-source runs (the same source timing at the old pixel clock; the expected P are the same integers)
[ "${STAGE2C:-0}" = 1 ] && { STAGE2=1; S2C="-DTB_CORE_CLK"; OBJ_S2=${OBJ_S2:-sim/obj_vsync_s2c}; }
if [ "${STAGE2:-0}" = 1 ] || [ "${STAGE1:-0}" = 1 ]; then
    HT=1650; VT=750; NF=${NF_S2:-64}; OBJ=${OBJ_S2:-sim/obj_vsync_s2}
    S2DEFS="-DTANGO_VSYNC_ADJUST -DTANGO_AUDIO_FILTER -DTB_VSYNC_STAGE2 ${S2C:-}"
    S2FILES="tango_vsync.v tango_beam_lead_rt.v tango_vproc_regs.v tango_audio_filter.v"
    if [ "${STAGE1:-0}" = 1 ]; then
        PLL=${PLL:-745875}; HT=1662; VT=748; OBJ=${OBJ_S1:-sim/obj_vsync_s1}; STAGE2=1
        S2DEFS="$S2DEFS -DTANGO_HDMI_PLL_$PLL -DTB_CORE_CLK"
        case "$PLL" in 745875) WANT="1656x750,1241998 1669x747,1246738";;
            *) echo "run_tb_vsync_ddr3: STAGE1 knows PLL=745875 only"; exit 2;; esac
    fi
fi
mkdir -p "$OBJ"; OBJ="$(cd "$OBJ" && pwd)"
build() {   # build <dir> <extra defines...>
    local D="$1"; shift
    mkdir -p "$D"
    TANGO_MEM_CAP=${TANGO_MEM_CAP:-6G} nice -n 10 "$ROOT/build_tools/tango_capped.sh" \
        verilator --binary --timing -O2 -Wall -Wno-fatal -Isim \
        -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
        -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-INITIALDLY -Wno-PINMISSING -Wno-MULTIDRIVEN -Wno-UNOPTFLAT \
        -DTANGO_VIDEO_ASCAL -DTANGO_VIDEO_OPTS -DTANGO_VIDEO_DDR3 -DTANGO_VIDEO_F1 -DTB_VSYNC_CORE \
        -DTANGO_HDMI_HTOTAL=$HT -DTANGO_HDMI_VTOTAL=$VT $S2DEFS "$@" \
        -DTANGO_VIDEO_ASCAL_MAXW=384 '-DBUILD_DATE="sim"' -DNFRAMES_DEF=$NF \
        --Mdir "$D" --top-module tb_ascal_ddr3 -I"$ROOT" \
        tango_sys.v tango_io.v tango_cdc.v tango_conf_rom.v tango_menu_overlay.v tango_link.v tango_link_dbg_view.v \
        tango_usb_port.v tango_ds4_joy.v tango_pad_decode.v tango_desc_ram.v tango_beam_lead.v usb/usb_fs_host.v usb/usb_fs_phy.v usb/usb_ls_detect.v \
        tango_video.v tango_video_geo.v ascal_wrap.v ascal_ghdl/ascal_shim_ddr3.v ascal_ghdl/ascal_fixed_ddr3.v \
        tango_ddr3.v tango_ddr3_arb.v tango_ddr3_vport.v tango_ddr3_cport.v tango_ddr3_fifo.v sim/tango_ddr3_model.v \
        $S2FILES sim/tb_vsync_core.v sim/tb_ascal_ddr3.v -o Vtb_ascal_ddr3 > "$D/build.log" 2>&1 \
        || { tail -30 "$D/build.log"; echo "run_tb_vsync_ddr3: build FAILED ($D)"; exit 1; }
}
run() {     # run <bin dir> <run dir> <plusargs...>
    local B="$1" D="$2"; shift 2
    rm -rf "$D"; mkdir -p "$D"
    ( cd "$D" && nice -n 10 "$ROOT/build_tools/tango_capped.sh" "$B/Vtb_ascal_ddr3" +seed=${SEED:-1} "$@" > run.log 2>&1 )
    grep -E '^MON (input|frames|avalon)|^MON_RESULT|^TANGO VIDEO' "$D/run.log" | sed "s/^/  $(basename "$D"): /"
}
R=0
if [ "${STAGE2:-0}" = 1 ]; then
    build "$OBJ/bin"
    DELAYS=("$@"); [ ${#DELAYS[@]} -eq 0 ] && DELAYS=(0)
    for d in "${DELAYS[@]}"; do
        D="$OBJ/d$d"
        if [ -n "${PAUSE:-}" ]; then
            run "$OBJ/bin" "$D" +core_lines=263 +core_delay=$d +pause_frame=26 +pause_line=$PAUSE +pause_x=${PAUSE_X:-0} +pause_us=${PAUSE_US:-18000}
            PF=$(sed -nE 's/^PAUSE 1: clk_pix stopped at beam \([0-9]+, [0-9]+\), MON frame ([0-9]+).*/\1/p' "$D/run.log" | head -1)
            SKIP=""; [ -n "$PF" ] && SKIP="--skip $PF,$((PF + 1)),$((PF + 2))"
            grep -E '^(RASTER|VSYNC measured|PAUSE)' "$D/run.log" | sed "s/^/  d$d: /"
            python3 sim/check_vsync_stage2.py "$D/run.log" $SKIP --want 1669x747,1246738 > "$D/check.log" 2>&1
        else
        run "$OBJ/bin" "$D" +core_lines=262 +core_lines2=263 +core_switch=${SW:-30} +core_delay=$d
        grep -E '^(RASTER|VSYNC measured|tb_vsync_core: from)' "$D/run.log" | sed "s/^/  d$d: /"
        python3 sim/check_vsync_stage2.py "$D/run.log" --want $WANT > "$D/check.log" 2>&1
        fi
        sed "s/^/  d$d: /" "$D/check.log" | tail -8
        if grep -q '^CHECK_VSYNC_STAGE2 PASS' "$D/check.log"; then echo "stage2 delay $d: PASS"; else echo "stage2 delay $d: FAIL"; R=1; fi
    done
    echo "run_tb_vsync_ddr3 STAGE2${PLL:+ (STAGE1, PLL $PLL)}: $([ $R = 0 ] && echo PASS || echo FAIL)"; exit $R
fi
if [ "${NEG:-0}" = 1 ]; then
    build "$OBJ/neg_beam" -DTB_BEAM_FW=1650 -DTB_BEAM_FH=750
    run "$OBJ/neg_beam" "$OBJ/neg_beam/run" +core_lines=$LINES
    if grep -q '^MON_RESULT PASS' "$OBJ/neg_beam/run/run.log" && grep -q '^TANGO VIDEO OK' "$OBJ/neg_beam/run/run.log"; then
        echo "neg_beam: PASSED -- the checks did not see the raster mismatch (BAD)"; R=1
    else echo "neg_beam: FAIL as it must (the raster mismatch is seen)"; fi
    build "$OBJ/bin"
    run "$OBJ/bin" "$OBJ/neg_rate" +core_lines=263 +core_ce=11
    if grep -q '^MON_RESULT PASS' "$OBJ/neg_rate/run.log"; then echo "neg_rate: PASSED -- dropped frames not seen (BAD)"; R=1
    elif ! grep -qE '^MON input frames: .*2\+ \(drop\) [1-9]' "$OBJ/neg_rate/run.log"; then
        echo "neg_rate: failed WITHOUT counting a drop -- not a valid negative control (BAD)"; R=1
    else echo "neg_rate: FAIL as it must (dropped frames are counted)"; fi
    echo "run_tb_vsync_ddr3 NEG: $([ $R = 0 ] && echo PASS || echo FAIL)"; exit $R
fi
DELAYS=("$@"); [ ${#DELAYS[@]} -eq 0 ] && DELAYS=(0 206850 413700 620550 827400 1034250)
build "$OBJ/bin"
for d in "${DELAYS[@]}"; do
    D="$OBJ/d$d"
    run "$OBJ/bin" "$D" +core_lines=$LINES +core_delay=$d
    if grep -q '^MON_RESULT PASS' "$D/run.log" && grep -q '^TANGO VIDEO OK' "$D/run.log"; then echo "delay $d: PASS"
    else echo "delay $d: FAIL"; R=1; fi
done
echo "run_tb_vsync_ddr3 ${HT}x${VT} source ${LINES} lines: $([ $R = 0 ] && echo PASS || echo FAIL)"
exit $R
