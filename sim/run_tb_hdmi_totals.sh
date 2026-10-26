#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
# run_tb_hdmi_totals.sh [H:V:VIC ...] -- hdmi.sv alone at raster totals (sim/tb_hdmi_totals.sv; the
# build-time raster totals).  Default set: 1650:750:4 (the defaults; with ORIG_REV set, also compared clock by clock with the ORIGINAL
# hdmi.sv from git, see below), the TG16 totals 1657:749:4 (263 lines,
# exact clock), 1664:743:4 (262 lines, exact clock), 1652:751:4 / 1659:745:4 (263 / 262 lines at the old 42.96875 MHz
# clock), 1657:749:0 (VIC 0), the range ends 1626:740:4 / 1720:790:4, and "runtime" (TANGO_VSYNC_ADJUST, the runtime
# totals: they change every frame along tb_hdmi_totals.sv's schedule, 9 frames).  Gate: "HDMI_TOTALS PASS" for each.
# Small (hdmi.sv alone, ~5 frames): still capped by build_tools/tango_capped.sh.  OBJ=<dir> (default sim/obj_hdmi_totals).
# PLL=745875|75600 (the build-time HDMI PLL presets): every config at that preset's exact clk_pix with its audio
# accumulator and ACR N / CTS (tango_cfg.vh TANGO_HDMI_PLL_<f>; no TB_ORIG comparison); default set then
# 1669:747:4 1656:750:4 1662:748:4 runtime (the 74.5875 MHz preset's 263 / 262-line totals and its fixed raster), OBJ default sim/obj_hdmi_pll<f>.
# ORIG_REV=<git rev> (optional): a revision of this repository whose hdmi/hdmi.sv predates the build-time totals
# (hdmi.sv with the fixed 1650 x 750 raster, no frame_width / frame_height parameters).  Its hdmi.sv runs beside the
# current one at 1650:750:4 (TB_ORIG) and every clock of the two TMDS streams must match.  Unset (e.g. a clone without
# that history), 1650:750:4 runs without the comparison and the script says so; a revision that has no such hdmi.sv
# stops the script (exit 2).
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; OBJ=${OBJ:-sim/obj_hdmi_totals}; ORIG_REV=${ORIG_REV:-}; PLL=${PLL:-}
PDEF=""; [ -n "$PLL" ] && { PDEF="-DTANGO_HDMI_PLL_$PLL"; OBJ=${OBJ_PLL:-sim/obj_hdmi_pll$PLL}; }
mkdir -p "$OBJ"; OBJ="$(cd "$OBJ" && pwd)"
CFGS=("$@"); [ ${#CFGS[@]} -eq 0 ] && CFGS=(1650:750:4 1657:749:4 1664:743:4 1652:751:4 1659:745:4 1657:749:0 1626:740:4 1720:790:4 runtime)
[ -n "$PLL" ] && [ $# -eq 0 ] && CFGS=(1669:747:4 1656:750:4 1662:748:4 runtime)
CMP_ORIG=0
for c in "${CFGS[@]}"; do [ "$c" = 1650:750:4 ] && [ -z "$PLL" ] && CMP_ORIG=1; done
if [ $CMP_ORIG = 1 ] && [ -n "$ORIG_REV" ]; then
    git show "$ORIG_REV:hdmi/hdmi.sv" | sed '0,/^module hdmi$/s//module hdmi_orig/' > "$OBJ/hdmi_orig.sv"
    grep -q '^module hdmi_orig$' "$OBJ/hdmi_orig.sv" || { echo "run_tb_hdmi_totals: could not make hdmi_orig from $ORIG_REV"; exit 2; }
elif [ $CMP_ORIG = 1 ]; then
    CMP_ORIG=0
    echo "run_tb_hdmi_totals: ORIG_REV not set: 1650:750:4 runs WITHOUT the clock-by-clock comparison with the original"
    echo "  hdmi.sv (set ORIG_REV=<rev> to a revision whose hdmi/hdmi.sv predates the build-time totals to include it)"
fi
R=0
for c in "${CFGS[@]}"; do
    IFS=: read -r H V VIC <<< "$c"
    D="$OBJ/${H}x${V}_vic${VIC}"; EXTRA=""
    if [ "$c" = runtime ]; then H=1650; V=750; VIC=4; D="$OBJ/runtime"; EXTRA="-DTANGO_VSYNC_ADJUST -DTB_NF=9"; fi
    rm -rf "$D"; mkdir -p "$D"
    FILES="hdmi/hdmi.sv"
    if [ "$c" = "1650:750:4" ] && [ $CMP_ORIG = 1 ]; then EXTRA="-DTB_ORIG"; FILES="$FILES $OBJ/hdmi_orig.sv"; fi
    EXTRA="$EXTRA $PDEF"
    TANGO_MEM_CAP=${TANGO_MEM_CAP:-3G} nice -n 10 "$ROOT/build_tools/tango_capped.sh" \
        verilator --binary --timing -O2 -Wall -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME \
        -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-MULTIDRIVEN -Wno-UNOPTFLAT \
        -Wno-SYNCASYNCNET -Wno-CASEINCOMPLETE -Wno-PINMISSING -Wno-BLKANDNBLK \
        -DTB_FW=$H -DTB_FH=$V -DTB_VIC=$VIC $EXTRA --Mdir "$D" --top-module tb_hdmi_totals -Ihdmi -I"$ROOT" \
        $FILES hdmi/packet_picker.sv hdmi/packet_assembler.sv hdmi/tmds_channel.sv \
        sim/hdmi_acr_sim.sv hdmi/audio_info_frame.sv hdmi/audio_sample_packet.sv \
        hdmi/auxiliary_video_information_info_frame.sv sim/tb_hdmi_totals.sv -o Vtb > "$D/build.log" 2>&1 \
        || { tail -20 "$D/build.log"; echo "$c: build FAILED"; R=1; continue; }
    ( cd "$D" && nice -n 10 "$ROOT/build_tools/tango_capped.sh" ./Vtb > run.log 2>&1 )
    grep -E '^HDMI_TOTALS|^VSYNC rise|^ACR BAD' "$D/run.log" | tail -5 | sed "s/^/  $c: /"
    if grep -q '^HDMI_TOTALS PASS' "$D/run.log"; then echo "$c: PASS"; else echo "$c: FAIL"; R=1; fi
done
echo "run_tb_hdmi_totals: $([ $R = 0 ] && echo PASS || echo FAIL)$([ -z "$PLL" ] && [ $CMP_ORIG = 0 ] && echo " (no original-hdmi.sv comparison)")"
exit $R
