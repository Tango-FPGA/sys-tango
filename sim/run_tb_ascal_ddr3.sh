#!/usr/bin/env bash
# run_tb_ascal_ddr3.sh [config ...] -- the DDR3 real-netlist gate: TANGO_VIDEO_DDR3 + TANGO_VIDEO_F1
# through the REAL ascal netlist (N_DW 128) and tango_ddr3 on the calibrated IP model (sim/tb_ascal_ddr3.v, its core
# sim/tb_ascal_ddr3_core.v), then sim/check_ascal_ext.py on the frames.  Configurations (default: all):
#   base   the calibrated model
#   wst    + cmd_ready low 350 ns (the DDR3 bring-up test's soak maximum) every 10 us on average
#   ref8   + 8 refreshes back to back every 100 us on average (a stress case NOT seen on silicon)
# Options as run_tb_ascal_ext.sh: Scanlines 50 % from the start; at output frame 8 the core goes progressive.  Gate per
# configuration: MON_RESULT PASS (data exact, protocol, the read pattern) and CHECK_ASCAL_EXT PASS (frames 5-8 woven
# with no scanlines, 12-13 progressive with scanlines).  OBJ=<dir> (default sim/obj_ascal_ddr3).
# Memory-capped by build_tools/tango_capped.sh.
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; OBJ=${OBJ:-sim/obj_ascal_ddr3}; NF=${NF:-14}
mkdir -p "$OBJ"; OBJ="$(cd "$OBJ" && pwd)"
CFGS=("$@"); [ ${#CFGS[@]} -eq 0 ] && CFGS=(base wst ref8)
if [ "${BUILD:-1}" = 1 ]; then
TANGO_MEM_CAP=${TANGO_MEM_CAP:-6G} nice -n 10 "$ROOT/build_tools/tango_capped.sh" \
    verilator --binary --timing -O2 -Wall -Wno-fatal -Isim \
    -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
    -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-INITIALDLY -Wno-PINMISSING -Wno-MULTIDRIVEN -Wno-UNOPTFLAT \
    -DTANGO_VIDEO_ASCAL -DTANGO_VIDEO_OPTS -DTANGO_VIDEO_DDR3 -DTANGO_VIDEO_F1 \
    -DTANGO_VIDEO_ASCAL_MAXW=384 '-DBUILD_DATE="sim"' -DNFRAMES_DEF=$NF \
    --Mdir "$OBJ" --top-module tb_ascal_ddr3 -I"$ROOT" \
    tango_sys.v tango_io.v tango_cdc.v tango_conf_rom.v tango_menu_overlay.v tango_link.v tango_link_dbg_view.v \
    tango_usb_port.v tango_ds4_joy.v tango_pad_decode.v tango_desc_ram.v tango_beam_lead.v usb/usb_fs_host.v usb/usb_fs_phy.v usb/usb_ls_detect.v \
    tango_video.v tango_video_geo.v ascal_wrap.v ascal_ghdl/ascal_shim_ddr3.v ascal_ghdl/ascal_fixed_ddr3.v \
    tango_ddr3.v tango_ddr3_arb.v tango_ddr3_vport.v tango_ddr3_cport.v tango_ddr3_fifo.v sim/tango_ddr3_model.v \
    sim/tb_ascal_ddr3_core.v sim/tb_ascal_ddr3.v -o Vtb_ascal_ddr3 > "$OBJ/build.log" 2>&1 \
    || { tail -30 "$OBJ/build.log"; echo "run_tb_ascal_ddr3: build FAILED"; exit 1; }
fi
S1=08000000000000000000000000000000
S2=08000000000000000000000000000002
R=0
for c in "${CFGS[@]}"; do
    case $c in
        base) A="";;
        wst)  A="+stall35_us=10";;
        ref8) A="+ref8_us=100";;
        *) echo "unknown configuration $c"; exit 2;;
    esac
    D="$OBJ/$c"; rm -rf "$D"; mkdir -p "$D"
    ( cd "$D" && nice -n 10 "$ROOT/build_tools/tango_capped.sh" "$OBJ/Vtb_ascal_ddr3" +status=$S1 +status2=$S2 \
        +switch_frame=8 +seed=${SEED:-1} $A > run.log 2>&1 )
    python3 sim/check_ascal_ext.py "$D" --woven 5-8 --prog 12-13 > "$D/check.log" 2>&1
    grep -E '^MON (avalon|frames|model)|^MON_RESULT' "$D/run.log" | sed "s/^/  $c: /"
    tail -1 "$D/check.log" | sed "s/^/  $c: /"
    if grep -q 'MON_RESULT PASS' "$D/run.log" && grep -q 'CHECK_ASCAL_EXT PASS' "$D/check.log"; then echo "$c: PASS"
    else echo "$c: FAIL"; R=1; fi
done
echo "run_tb_ascal_ddr3: $([ $R = 0 ] && echo PASS || echo FAIL)"
exit $R
