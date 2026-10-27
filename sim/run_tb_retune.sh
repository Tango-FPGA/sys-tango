#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
# run_tb_retune.sh [PLL=745875] -- vsync_adjust's HDMI PLL retune at core start, at the clock level (sim/tb_retune.v + sim/check_retune.py): the HDMI
# PLL (a behavioural model of the Gowin primitive, sim/gw_pll_hdmi_sim.v, with Gowin's own PLL_INIT) switched at run time by
# SYS 0x70 through tango_vsync and tango_pll_retune, hdmi.sv's raster / ACR / audio clock checked at both clocks.
# Small (no core, no DDR3): memory-capped by build_tools/tango_capped.sh. OBJ default sim/obj_retune.
# PLL_INIT is Gowin's pll_init.v from the user's Gowin installation: GOWIN_HOME (the directory that holds IDE/; default
# ~/gowin), IDE/ipcore/PLL_ADV/data/PLL/pll_init.v, as board/console138k/build_core.sh.
set -uo pipefail
cd "$(dirname "$0")/.."
PLL_INIT_V="${GOWIN_HOME:-$HOME/gowin}/IDE/ipcore/PLL_ADV/data/PLL/pll_init.v"
[ -f "$PLL_INIT_V" ] || { echo "run_tb_retune: $PLL_INIT_V not found: set GOWIN_HOME to the Gowin installation (the directory that holds IDE/)"; exit 2; }
ROOT="$(pwd)"; OBJ=${OBJ:-sim/obj_retune}; PLL=${PLL:-745875}
case "$PLL" in 745875) FPIX=74587500; FIX=1662x748; PLAN=1669x747;; *) echo "run_tb_retune: PLL=745875 only"; exit 2;; esac
rm -rf "$OBJ"; mkdir -p "$OBJ"
TANGO_MEM_CAP=${TANGO_MEM_CAP:-3G} nice -n 10 "$ROOT/build_tools/tango_capped.sh" \
    verilator --binary --timing -O2 -Wall -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-DECLFILENAME \
    -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-MULTIDRIVEN -Wno-UNOPTFLAT \
    -Wno-SYNCASYNCNET -Wno-CASEINCOMPLETE -Wno-PINMISSING -Wno-BLKANDNBLK -Wno-INITIALDLY -Wno-UNDRIVEN \
    -DTANGO_VSYNC_ADJUST -DTANGO_HDMI_RETUNE -DTANGO_HDMI_PLL_$PLL -DTB_HT=${FIX%x*} -DTB_VT=${FIX#*x} \
    --Mdir "$OBJ" --top-module tb_retune -Ihdmi -I"$ROOT" \
    sim/gw_pll_hdmi_sim.v sim/gw_dce_sim.v "$PLL_INIT_V" board/console138k/pll_c1485_mod.v board/console138k/pll_c1485.v \
    tango_pll_retune.v tango_vsync.v hdmi/hdmi.sv hdmi/packet_picker.sv hdmi/packet_assembler.sv hdmi/tmds_channel.sv \
    sim/hdmi_acr_sim.sv hdmi/audio_info_frame.sv hdmi/audio_sample_packet.sv hdmi/auxiliary_video_information_info_frame.sv \
    sim/tb_retune.v -o Vtb > "$OBJ/build.log" 2>&1 || { tail -30 "$OBJ/build.log"; echo "run_tb_retune: build FAILED"; exit 1; }
( cd "$OBJ" && nice -n 10 "$ROOT/build_tools/tango_capped.sh" ./Vtb > run.log 2>&1 )
grep -E "^phase|^SYS 0x70|^AUDIO|^TB_RETUNE|^FAIL|^PLL MODEL|^ACR BAD|^PAUSE" "$OBJ/run.log" | head -40
python3 sim/check_retune.py "$OBJ/run.log" --fpix $FPIX --fixed $FIX --plan $PLAN
