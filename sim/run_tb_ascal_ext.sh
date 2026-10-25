#!/usr/bin/env bash
# run_tb_ascal_ext.sh [nframes] -- TANGO_ASCAL_EXT + TANGO_VIDEO_F1 through the REAL ascal (sim/tb_ascal_ext.v: tango_sys
# + the sim-only core sim/tb_ascal_ext_core.v), then sim/check_ascal_ext.py on the frames.
# Run from the sys-tango checkout (or a core's sys/): sim/run_tb_ascal_ext.sh.  OBJ=<dir> (default sim/obj_ascal_ext).
# The option words: Scanlines 50 % from the start (status[123:122] = 2); at output frame 8 the core goes progressive
# (status[1] = 1).  Gate: frames 5-8 woven with no scanlines (the first frames show ascal starting up: a field alone
# until its interlace detection and both fields are in one buffer), frames 12-13 progressive with scanlines.
# Memory-capped by build_tools/tango_capped.sh.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"; OBJ=${OBJ:-sim/obj_ascal_ext}; NF=${1:-14}
mkdir -p "$OBJ"; rm -f "$OBJ"/out_f*.ppm
TANGO_MEM_CAP=${TANGO_MEM_CAP:-6G} nice -n 10 "$ROOT/build_tools/tango_capped.sh" \
    verilator --binary --timing -O2 -Wall -Wno-fatal \
    -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
    -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-INITIALDLY -Wno-PINMISSING -Wno-MULTIDRIVEN -Wno-UNOPTFLAT \
    -DTANGO_VIDEO_ASCAL -DTANGO_VIDEO_OPTS -DTANGO_SDRAM -DTANGO_ASCAL_EXT -DTANGO_VIDEO_F1 \
    -DTANGO_VIDEO_ASCAL_MAXW=384 '-DBUILD_DATE="sim"' -DNFRAMES_DEF=$NF \
    --Mdir "$OBJ" --top-module tb_ascal_ext -I"$ROOT" \
    tango_sys.v tango_io.v tango_cdc.v tango_conf_rom.v tango_menu_overlay.v tango_link.v tango_link_dbg_view.v \
    tango_usb_port.v tango_ds4_joy.v tango_pad_decode.v tango_desc_ram.v tango_beam_lead.v usb/usb_fs_host.v usb/usb_fs_phy.v usb/usb_ls_detect.v \
    tango_video.v tango_video_geo.v ascal_wrap.v ascal_ghdl/ascal_shim.v ascal_ghdl/ascal_fixed.v tango_ascal_words.v \
    sim/tb_ascal_ext_core.v sim/tb_ascal_ext.v -o Vtb_ascal_ext
# the option words (32 hex digits, bit 127 first): Scanlines 50 % = status[123:122] = 2 (0x08 in the top byte);
# then + status[1] (the core goes progressive)
S1=08000000000000000000000000000000
S2=08000000000000000000000000000002
( cd "$OBJ" && nice -n 10 "$ROOT/build_tools/tango_capped.sh" ./Vtb_ascal_ext +status=$S1 +status2=$S2 +switch_frame=8 ) | tee "$OBJ/run.log"
python3 sim/check_ascal_ext.py "$OBJ" --woven 5-8 --prog 12-13 | tee "$OBJ/check.log"
grep -q "CHECK_ASCAL_EXT PASS" "$OBJ/check.log"
