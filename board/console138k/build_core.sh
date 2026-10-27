#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
# build_core.sh -- build a Tango core for the Tang Console 138K and archive the bitstream.
#   sys/board/console138k/build_core.sh <name> <core.sdc or -> <core HDL files...>
#     <name>      output / archive name, e.g. tango_template138 (debug builds: add TANGO_DBG=1)
#     <core.sdc>  the core's extra clocks (appended to the board SDC), or - for none
#     files       the core's HDL (paths relative to the checkout or absolute); one defines tango_core
#   e.g. sys/board/console138k/build_core.sh tango_menu138 - fpga/rtl/tango_core_menu.v fpga/rtl/tango_test_pattern.v
# Output: $TANGO_OUT (default fpga/build/out, relative to the checkout that holds sys/)/<name>/impl/pnr/<name>.fs;
# STA gate (sta_gate.py, every clock); archive $TANGO_ARCHIVE (default $TANGO_OUT/archive)/
# <name>_<YYYYMMDD-HHMM>_g<git>.{fs,bin} (TANGO_NO_ARCHIVE=1: none -- core repos archive themselves).
# The build helpers (tango_capped.sh, sta_gate.py, fs2bin.py) come from sys/build_tools/ when it exists,
# else from the checkout's harness/ and tools/ (an older layout).
# TANGO_SDRAM=1: the core has the SDRAM ports (tango_core_ports.vh `ifdef TANGO_SDRAM): the build gets
# `define TANGO_SDRAM (a generated tango_defines.v, the first file of the build) and the board cst
# plus sys/board/console138k/tango_console138k_sdram.cst.
# TANGO_SD=1 [TANGO_SD_VDNUM=n]: the core has MiSTer's block-device ports (sys/tango_sd_ports.vh, tango_sd.v).
# TANGO_IOCTL_UPLOAD=1: the core has MiSTer's hps_io upload direction (tango_cfg.vh: ioctl_upload,
# ioctl_upload_req, ioctl_upload_index, ioctl_din -- hps_io.sv 142-150, the reverse of ioctl_download).
# TANGO_VIDEO_FB=1 [TANGO_VIDEO_MAXW= _MAXH= _BPC= _NBUF= ; TANGO_VIDEO_IDX=1 TANGO_VIDEO_IDXW=]: the core outputs
# MiSTer's emu video ports and sys/tango_video.v scales them (README "Build options"); not the default yet.
# TANGO_VIDEO_ASCAL=1: ascal.vhd + SDRAM replaces the BSRAM frame buffer (README "Video").
# The framework owns the SDRAM controller (tango_sdram.v); the core must NOT define TANGO_SDRAM.
# TANGO_VIDEO_ASCAL_MAXW= _MAXH= _RAMBASE= _RAMSIZE= override the tango_cfg.vh defaults.
# TANGO_ASCAL_EXT=1 (with TANGO_VIDEO_ASCAL=1 and TANGO_SDRAM=1): the core owns the SDRAM and its controller serves
# ascal's Avalon port (tango_cfg.vh, README "A core that owns the SDRAM"): no tango_sdram, no SDRAM PLL, no clk_sdram
# SDC; the core's SDC declares its memory clock.  TANGO_VIDEO_F1=1 (with TANGO_VIDEO_ASCAL=1): the core drives
# VGA_F1 (tango_cfg.vh).
# TANGO_VIDEO_OPTS=1 (with TANGO_VIDEO_ASCAL=1): the core opts in to the framework's Video options
# (sys/tango_video_opts.vh: Aspect ratio / Scale / Scanlines = status[127:120], decoded by sys/tango_video.v).
# TANGO_VIDEO_ROTATE=1 (README "Rotation"): needs TANGO_VIDEO_DDR3=1, not just TANGO_VIDEO_ASCAL=1 -- the rotated
# picture is written through tango_ddr3's C0 port (sys/tango_video_rotate_ddr3.v), so this also defines
# `TANGO_DDRAM` (the C0 port's own build option) automatically. rotate_mode is CDC'd from
# status[`TANGO_VO_B_ROTATE] (tango_cdc_word), not a Verilog constant -- a literal 2'b00 would let synthesis fold
# the block away. tango_video_opts.vh's `TANGO_VIDEO_CONF`/`TANGO_VIDEO_CONF_P` add the "Rotate" CONF_STR entry,
# but ONLY when this flag (TANGO_VIDEO_ROTATE) is itself defined for the build -- a core that defines
# TANGO_VIDEO_OPTS without this flag still gets no Rotate entry and status[119:118] stays at its reset default
# (Normal). Not set: tango_sys.v does not instantiate the block at all -- the core's netlist is unchanged.
# Limitation: this claims the whole of C0 for the rotation client. A core that also wants MiSTer-style DDRAM_*
# traffic of its own, together with TANGO_VIDEO_ROTATE, would need a second core port (NC=2) and is not supported
# by this flag.
# TANGO_VIDEO_PROC=1 (with TANGO_VIDEO_ASCAL=1): MiSTer's video processing -- polyphase filters, gamma, shadow mask
# -- and its audio filter (README "Video and audio processing"); TANGO_AUDIO_FILTER=1: the audio filter alone (any core).
# TANGO_VIDEO_DDR3=1 (with TANGO_VIDEO_ASCAL=1; not with TANGO_ASCAL_EXT, TANGO_DBG): ascal's frames
# in the DDR3 (README "The DDR3 path"; tango_cfg.vh): the DDR3 IP generated into the build (ddr3/tango_ddr3_ip.tcl),
# its pins and placement (tango_console138k_ddr3.cst), its clocks memory_clk / clk_x1 (tango_console138k_ddr3.sdc, each
# a group of its own in the build's one set_clock_groups line); no tango_sdram, no SDRAM PLL, no framework SDRAM pins --
# except with TANGO_NES_SDRAM=1: tango_sdram, its PLL, pins and clk_sdram SDC stay, for the NES's ports A/B (tango_cfg.vh).
# TANGO_BUILD_DATE=<yymmdd>: the BUILD_DATE the core shows (default today); for a rebuild that must equal an old
# build bit for bit (e.g. the byte-identity check of an opt-in feature).
# TANGO_PRESET_STATUS=<32 hex digits>: the option word at power-up (until the firmware's first SET_STATUS).
# TANGO_VPROC_PRESET=<settings> (with TANGO_VIDEO_PROC or TANGO_AUDIO_FILTER): a compiled-in load of filters / gamma /
# mask / audio filter, replayed after reset (build_tools/vproc_preset.py; the firmware's loads replace it).
# <settings> = the arguments for vproc_preset.py, e.g. "--preset 'Scanlines - Soft.ini' --afilter 'General LPF/LPF
# 20khz 3rd BW.txt'" (files from TANGO_FILTERS_ROOT, which must be set: the folder with MiSTer's filter files,
# README "Video and audio processing"); its --scanlines comes from TANGO_PRESET_STATUS bits 123:122 unless given.
# For JTAG tests without the BL616 firmware.
# TANGO_HDMI_HTOTAL=<1626..1720> TANGO_HDMI_VTOTAL=<740..790> [TANGO_HDMI_VIC=<0|4>]: the HDMI raster's totals and AVI
# VIC (tango_cfg.vh; README "Native-rate video (vsync_adjust)"): the output frame rate becomes
# 74.25e6 / (HTOTAL x VTOTAL).
# TANGO_VSYNC_ADJUST=1 [TANGO_VSYNC_DEFAULT=0|1] (with TANGO_VIDEO_ASCAL=1 and TANGO_VIDEO_PROC=1 or TANGO_AUDIO_FILTER=1):
# native-rate output, the totals planned at runtime from the core's measured frame rate (tango_vsync.v);
# the fixed raster is then TANGO_HDMI_HTOTAL x _VTOTAL (Tango.ini VSYNC_ADJUST=0, or before the first plan).
# TANGO_HDMI_PLL=745875|75600: the HDMI pixel clock retuned at build time (tango_cfg.vh TANGO_HDMI_PLL_<f>):
# 74.5875 / 75.6 MHz; the SDC's clk_pix / clk_ser periods are rewritten here, the audio accumulator and the
# ACR's N / CTS follow in RTL.  Not set: 74.25 MHz, the board SDC as it is.
# TANGO_HDMI_RETUNE=1 (with TANGO_VSYNC_ADJUST=1 and TANGO_HDMI_PLL=<f>; tango_cfg.vh, tango_pll_retune.v): the
# PLL switched at run time between the preset (native) and 74.25 MHz (fixed: exactly 60.000 Hz) by SYS 0x70.
# (copied, cmp, chmod 555; .bin via tools/fs2bin.py).  A failing gate is archived but printed as
# do-not-flash.  Headless Gowin environment (gw_sh from GOWIN_HOME, below) + memory cap, through
# build_tools/tango_capped.sh.
# Heavy job: run one build or simulation at a time, and check that no other Gowin or Verilator job runs:
# `ps -eo pid,etime,comm | grep -E ' (Vtb|V[a-z_]+|verilator|gw_sh|GowinSynthesis|gw_syn)'`
# must be empty before starting.  Never flashes anything.
# GOWIN_HOME=<dir>: the Gowin EDA installation, the directory that holds IDE/ (default ~/gowin).  The build runs its
# gw_sh and takes two things from it that this repository does not ship: PLL_INIT (IDE/ipcore/PLL_ADV/data/PLL/
# pll_init.v, the start-up calibration every Console 138K PLL needs; added to every build) and, in TANGO_VIDEO_DDR3
# builds, the DDR3 IP (generated by ddr3/tango_ddr3_ip.tcl).
set -eu
[ $# -ge 3 ] || { sed -n 2,8p "$0"; exit 2; }
GOWIN_HOME="${GOWIN_HOME:-$HOME/gowin}"
GW_SH="$GOWIN_HOME/IDE/bin/gw_sh"
PLL_INIT_V="$GOWIN_HOME/IDE/ipcore/PLL_ADV/data/PLL/pll_init.v"
[ -x "$GW_SH" ] || { echo "build_core.sh: no Gowin IDE at GOWIN_HOME=$GOWIN_HOME ($GW_SH not found): set GOWIN_HOME to the installation directory (the one that holds IDE/)"; exit 2; }
[ -f "$PLL_INIT_V" ] || { echo "build_core.sh: $PLL_INIT_V not found: the Gowin IDE at GOWIN_HOME=$GOWIN_HOME has no PLL_ADV IP core (PLL_INIT)"; exit 2; }
echo "build_core.sh: PLL_INIT from $PLL_INIT_V (sha256 $(sha256sum "$PLL_INIT_V" | cut -c1-16))"
NAME=$1; CSDC=$2; shift 2
cd "$(dirname "$0")/../../.."
ROOT="$(pwd)"
FILES=""; for f in "$@"; do FILES="$FILES $(readlink -f "$f")"; done
BRD=sys/board/console138k
DBGSFX=""; [ "${TANGO_DBG:-0}" = 1 ] && DBGSFX=_dbg
if [ -d sys/build_tools ]; then CAPPED=sys/build_tools/tango_capped.sh; STAG=sys/build_tools/sta_gate.py; FS2BIN=sys/build_tools/fs2bin.py
else CAPPED=harness/tango_capped.sh; STAG=harness/sta_gate.py; FS2BIN=tools/fs2bin.py; fi
OUTD="${TANGO_OUT:-fpga/build/out}"; mkdir -p "$OUTD"; OUTD="$(cd "$OUTD" && pwd)"
SDC="$OUTD/${NAME}.sdc"
cat "$BRD/tango_console138k${DBGSFX}.sdc" > "$SDC"
# the build's own include dir: tango_cfg.vh (TANGO_SDRAM or not) found before sys/'s default one
INC="$OUTD/${NAME}_inc"; rm -rf "$INC"; mkdir -p "$INC"; cp sys/*.vh "$INC/"   # sys/tango_cfg.vh: no TANGO_SDRAM
CST="$OUTD/${NAME}.cst"; cat "$BRD/tango_console138k${DBGSFX}.cst" > "$CST"
# build options as `defines in a source file compiled FIRST (Gowin compiles every file as one unit, so the
# define reaches sys/ and the core; its include search finds sys/tango_cfg.vh before any include path, so a
# generated tango_cfg.vh cannot override it -- measured on the first NES build)
DEFS="$INC/tango_defines.v"; echo "// generated by build_core.sh: the build's options" > "$DEFS"
printf '`define BUILD_DATE "v%s"\n' "${TANGO_BUILD_DATE:-$(date +%y%m%d)}" >> "$DEFS"   # TANGO_BUILD_DATE=yymmdd: rebuild an old day's bits
if [ "${TANGO_SD:-0}" = 1 ]; then          # MiSTer's block device (tango_sd.v): sd_* / img_* ports
    printf '`define TANGO_SD\n`define TANGO_SD_VDNUM %d\n' "${TANGO_SD_VDNUM:-1}" >> "$DEFS"
fi
if [ "${TANGO_VIDEO_FB:-0}" = 1 ]; then    # the earlier BSRAM frame buffer + scaler path (README "Build options")
    printf '`define TANGO_VIDEO_FB\n' >> "$DEFS"
    for v in MAXW MAXH BPC NBUF IDXW; do eval "x=\${TANGO_VIDEO_$v:-}"; [ -n "$x" ] && printf '`define TANGO_VIDEO_%s %d\n' "$v" "$x" >> "$DEFS"; done
    [ "${TANGO_VIDEO_IDX:-0}" = 1 ] && printf '`define TANGO_VIDEO_IDX\n' >> "$DEFS"
fi
if [ "${TANGO_SDRAM:-0}" = 1 ]; then
    printf '`define TANGO_SDRAM\n' >> "$DEFS"
    cat "$BRD/tango_console138k_sdram.cst" >> "$CST"
fi
# MiSTer's hps_io upload direction (tango_cfg.vh; ioctl_upload/ioctl_din/ioctl_upload_req/_index): the Gowin
# build's define, as the simulations' +define+TANGO_IOCTL_UPLOAD.
if [ "${TANGO_IOCTL_UPLOAD:-0}" = 1 ]; then
    printf '`define TANGO_IOCTL_UPLOAD\n' >> "$DEFS"
fi
if [ "${TANGO_NES_SDRAM:-0}" = 1 ]; then
    printf '`define TANGO_NES_SDRAM\n' >> "$DEFS"
fi
if [ "${TANGO_ASCAL_EXT:-0}" = 1 ]; then
    [ "${TANGO_VIDEO_ASCAL:-0}" = 1 ] && [ "${TANGO_SDRAM:-0}" = 1 ] || { echo "build_core.sh: TANGO_ASCAL_EXT=1 needs TANGO_VIDEO_ASCAL=1 and TANGO_SDRAM=1 (sys/tango_cfg.vh)"; exit 2; }
    [ "${TANGO_NES_SDRAM:-0}" = 1 ] && { echo "build_core.sh: TANGO_ASCAL_EXT=1 excludes TANGO_NES_SDRAM=1 (tango_sdram is not built)"; exit 2; }
    printf '`define TANGO_ASCAL_EXT\n' >> "$DEFS"
fi
if [ "${TANGO_VIDEO_F1:-0}" = 1 ]; then
    [ "${TANGO_VIDEO_ASCAL:-0}" = 1 ] || { echo "build_core.sh: TANGO_VIDEO_F1=1 needs TANGO_VIDEO_ASCAL=1 (sys/tango_cfg.vh)"; exit 2; }
    printf '`define TANGO_VIDEO_F1\n' >> "$DEFS"
fi
DDR3=0
if [ "${TANGO_VIDEO_DDR3:-0}" = 1 ]; then
    [ "${TANGO_VIDEO_ASCAL:-0}" = 1 ] || { echo "build_core.sh: TANGO_VIDEO_DDR3=1 needs TANGO_VIDEO_ASCAL=1 (sys/tango_cfg.vh)"; exit 2; }
    [ "${TANGO_ASCAL_EXT:-0}" = 1 ] && { echo "build_core.sh: TANGO_VIDEO_DDR3=1 excludes TANGO_ASCAL_EXT=1 (ascal's port goes to the DDR3)"; exit 2; }
    [ -n "$DBGSFX" ] && { echo "build_core.sh: TANGO_VIDEO_DDR3=1 has no debug top yet (TANGO_DBG=1)"; exit 2; }
    printf '`define TANGO_VIDEO_DDR3\n' >> "$DEFS"
    cat "$BRD/tango_console138k_ddr3.cst" >> "$CST"
    DDR3=1
fi
if [ "${TANGO_VIDEO_ASCAL:-0}" = 1 ]; then
    # the GHDL netlist (sys/ascal_ghdl/, default ascal source) has RAMBASE/RAMSIZE baked in: refuse others
    RB0="32\'h0140_0000"; [ $DDR3 = 1 ] && RB0="32\'h2000_0000"
    if [ "${TANGO_ASCAL_SRC:-ghdl}" = ghdl ]; then
        for v in RAMBASE:$RB0 RAMSIZE:32\'h0040_0000; do
            eval "x=\${TANGO_VIDEO_ASCAL_${v%%:*}:-}"
            [ -z "$x" ] || [ "$x" = "${v#*:}" ] || { echo "build_core.sh: TANGO_VIDEO_ASCAL_${v%%:*}=$x differs from sys/ascal_ghdl (${v#*:}): edit ascal_fixed.vhd + rerun gen.sh, or TANGO_ASCAL_SRC=vhdl"; exit 2; }
        done
    fi
    printf '`define TANGO_VIDEO_ASCAL\n' >> "$DEFS"
    for v in MAXW MAXH; do eval "x=\${TANGO_VIDEO_ASCAL_$v:-}"; [ -n "$x" ] && printf '`define TANGO_VIDEO_ASCAL_%s %d\n' "$v" "$x" >> "$DEFS"; done
    for v in RAMBASE RAMSIZE; do eval "x=\${TANGO_VIDEO_ASCAL_$v:-}"; [ -n "$x" ] && printf '`define TANGO_VIDEO_ASCAL_%s %s\n' "$v" "$x" >> "$DEFS"; done
    # tango_sdram's pins and clk_sdram (TANGO_FW_SDRAM, tango_cfg.vh): not with TANGO_ASCAL_EXT (the pins came with
    # TANGO_SDRAM; no clk_sdram), nor with TANGO_VIDEO_DDR3 unless TANGO_NES_SDRAM (ports A/B)
    if [ "${TANGO_ASCAL_EXT:-0}" != 1 ] && { [ $DDR3 = 0 ] || [ "${TANGO_NES_SDRAM:-0}" = 1 ]; }; then
    cat "$BRD/tango_console138k_sdram.cst" >> "$CST"   # SDRAM pin constraints (same as TANGO_SDRAM)
    # clk_sdram SDC: create_clock 162 MHz + set_clock_groups (debug builds prefix nets with u/)
    if [ -n "$DBGSFX" ]; then sed 's#get_nets {#get_nets {u/#g' "$BRD/tango_console138k_sdram108.sdc" >> "$SDC"
    else cat "$BRD/tango_console138k_sdram108.sdc" >> "$SDC"; fi
    fi
fi
if [ "${TANGO_VIDEO_OPTS:-0}" = 1 ]; then   # the framework's Video options (sys/tango_video_opts.vh)
    [ "${TANGO_VIDEO_ASCAL:-0}" = 1 ] || { echo "build_core.sh: TANGO_VIDEO_OPTS=1 needs TANGO_VIDEO_ASCAL=1 (sys/tango_cfg.vh)"; exit 2; }
    printf '`define TANGO_VIDEO_OPTS\n' >> "$DEFS"
fi
if [ "${TANGO_VIDEO_ROTATE:-0}" = 1 ]; then   # the DDR3-direct rotation client (sys/tango_video_rotate_ddr3.v, README "Rotation")
    [ "$DDR3" = 1 ] || { echo "build_core.sh: TANGO_VIDEO_ROTATE=1 needs TANGO_VIDEO_DDR3=1 (the rotation client writes its frames through tango_ddr3's C0 port)"; exit 2; }
    printf '`define TANGO_VIDEO_ROTATE\n`define TANGO_DDRAM\n' >> "$DEFS"
fi
VPROC=0; AFILT=0
if [ "${TANGO_VIDEO_PROC:-0}" = 1 ]; then   # MiSTer's video + audio processing (README "Video and audio processing")
    [ "${TANGO_VIDEO_ASCAL:-0}" = 1 ] || { echo "build_core.sh: TANGO_VIDEO_PROC=1 needs TANGO_VIDEO_ASCAL=1 (sys/tango_cfg.vh)"; exit 2; }
    printf '`define TANGO_VIDEO_PROC\n' >> "$DEFS"; VPROC=1; AFILT=1
fi
if [ "${TANGO_AUDIO_FILTER:-0}" = 1 ]; then printf '`define TANGO_AUDIO_FILTER\n' >> "$DEFS"; AFILT=1; fi
# TANGO_HDMI_HTOTAL / TANGO_HDMI_VTOTAL / TANGO_HDMI_VIC: the HDMI raster's totals and AVI VIC (tango_cfg.vh;
# README "Native-rate video (vsync_adjust)"): both totals or neither, in tango_cfg.vh's range (the board top refuses
# others too)
if [ -n "${TANGO_HDMI_HTOTAL:-}${TANGO_HDMI_VTOTAL:-}" ]; then
    echo "${TANGO_HDMI_HTOTAL:-}" | grep -qE '^[0-9]+$' && echo "${TANGO_HDMI_VTOTAL:-}" | grep -qE '^[0-9]+$' || \
        { echo "build_core.sh: TANGO_HDMI_HTOTAL and TANGO_HDMI_VTOTAL go together, both decimal"; exit 2; }
    [ "$TANGO_HDMI_HTOTAL" -ge 1626 ] && [ "$TANGO_HDMI_HTOTAL" -le 1720 ] && [ "$TANGO_HDMI_VTOTAL" -ge 740 ] && \
        [ "$TANGO_HDMI_VTOTAL" -le 790 ] || { echo "build_core.sh: TANGO_HDMI_HTOTAL 1626..1720, TANGO_HDMI_VTOTAL 740..790 (tango_cfg.vh)"; exit 2; }
    printf '`define TANGO_HDMI_HTOTAL %d\n`define TANGO_HDMI_VTOTAL %d\n' "$TANGO_HDMI_HTOTAL" "$TANGO_HDMI_VTOTAL" >> "$DEFS"
fi
if [ -n "${TANGO_HDMI_VIC:-}" ]; then
    case "$TANGO_HDMI_VIC" in 0|4) ;; *) echo "build_core.sh: TANGO_HDMI_VIC 0 or 4 (tango_cfg.vh)"; exit 2;; esac
    printf '`define TANGO_HDMI_VIC %d\n' "$TANGO_HDMI_VIC" >> "$DEFS"
fi
# TANGO_HDMI_PLL=<f>: the preset's `define and its exact clk_pix / clk_ser periods (ns, rounded DOWN = never looser) in the
# board SDC's two create_clock lines (the dbg SDC's too: the -name is the same)
if [ -n "${TANGO_HDMI_PLL:-}" ]; then
    case "$TANGO_HDMI_PLL" in
        745875) PPIX=13.407; PSER=2.681;;     # 1e9 / 74587500 = 13.40707, / 372937500 = 2.68141
        75600)  PPIX=13.227; PSER=2.645;;     # 1e9 / 75600000 = 13.22751, / 378000000 = 2.64550
        *) echo "build_core.sh: TANGO_HDMI_PLL 745875 or 75600 (tango_cfg.vh)"; exit 2;;
    esac
    printf '`define TANGO_HDMI_PLL_%s\n' "$TANGO_HDMI_PLL" >> "$DEFS"
    grep -qE '^create_clock -name clk_pix +-period 13\.468 ' "$SDC" && grep -qE '^create_clock -name clk_ser +-period 2\.694 ' "$SDC" || \
        { echo "build_core.sh: TANGO_HDMI_PLL: the board SDC's clk_pix / clk_ser lines are not the expected 13.468 / 2.694"; exit 2; }
    sed -i -E "s#^(create_clock -name clk_pix +-period )13\.468 (.*)// 720p: 74\.25 MHz#\1$PPIX \2// TANGO_HDMI_PLL=$TANGO_HDMI_PLL#; s#^(create_clock -name clk_ser +-period )2\.694 (.*)// 720p: 371\.25 MHz#\1$PSER \2// TANGO_HDMI_PLL=$TANGO_HDMI_PLL x 5#" "$SDC"
    grep -qE "^create_clock -name clk_pix +-period $PPIX " "$SDC" && grep -qE "^create_clock -name clk_ser +-period $PSER " "$SDC" || \
        { echo "build_core.sh: TANGO_HDMI_PLL: the SDC rewrite failed"; exit 2; }
    echo "build_core.sh: TANGO_HDMI_PLL=$TANGO_HDMI_PLL: $(grep -E '^create_clock -name clk_(pix|ser) ' "$SDC" | tr -s ' ' | tr '\n' ';')"
fi
if [ "${TANGO_HDMI_RETUNE:-0}" = 1 ]; then
    [ -n "${TANGO_HDMI_PLL:-}" ] || { echo "build_core.sh: TANGO_HDMI_RETUNE=1 needs TANGO_HDMI_PLL=<f> (the native preset)"; exit 2; }
    [ "${TANGO_VSYNC_ADJUST:-0}" = 1 ] || { echo "build_core.sh: TANGO_HDMI_RETUNE=1 needs TANGO_VSYNC_ADJUST=1"; exit 2; }
    printf '`define TANGO_HDMI_RETUNE\n' >> "$DEFS"
    # the PLL's raw CLKOUT1 (before the DCE that pauses clk_pix through a retune) clocks the DCE's enable flops: the same
    # period, in clk_pix's group
    RAWNET=clk_pix_raw; [ -n "$DBGSFX" ] && RAWNET=u/clk_pix_raw
    PRAW=$(sed -nE 's#^create_clock -name clk_pix +-period ([0-9.]+) .*#\1#p' "$SDC" | head -1)
    sed -i "/^create_clock -name clk_pix /a create_clock -name clk_pix_raw -period $PRAW [get_nets {$RAWNET}]   // TANGO_HDMI_RETUNE: before the DCE" "$SDC"
    echo "build_core.sh: TANGO_HDMI_RETUNE: $(grep -E '^create_clock -name clk_pix_raw' "$SDC" | tr -s ' ')"
    RETUNE_SDC=1                                  # clk_pix_raw joins clk_pix's group once the build's group line is final
fi
# TANGO_VSYNC_ADJUST=1 [TANGO_VSYNC_DEFAULT=0|1]: native-rate output (tango_cfg.vh; tango_vsync.v): needs
# TANGO_VIDEO_ASCAL and the link tap (TANGO_VIDEO_PROC or TANGO_AUDIO_FILTER); tango_sys refuses it otherwise
if [ "${TANGO_VSYNC_ADJUST:-0}" = 1 ]; then
    [ "${TANGO_VIDEO_ASCAL:-0}" = 1 ] || { echo "build_core.sh: TANGO_VSYNC_ADJUST=1 needs TANGO_VIDEO_ASCAL=1 (tango_cfg.vh)"; exit 2; }
    [ $AFILT = 1 ] || { echo "build_core.sh: TANGO_VSYNC_ADJUST=1 needs TANGO_VIDEO_PROC=1 or TANGO_AUDIO_FILTER=1 (the link tap)"; exit 2; }
    printf '`define TANGO_VSYNC_ADJUST\n' >> "$DEFS"
    case "${TANGO_VSYNC_DEFAULT:-1}" in 0|1) printf '`define TANGO_VSYNC_DEFAULT %d\n' "${TANGO_VSYNC_DEFAULT:-1}" >> "$DEFS";;
        *) echo "build_core.sh: TANGO_VSYNC_DEFAULT 0 or 1"; exit 2;; esac
fi
if [ -n "${TANGO_PRESET_STATUS:-}" ]; then   # the option word at power-up (tango_cfg.vh)
    echo "$TANGO_PRESET_STATUS" | grep -qE '^[0-9A-Fa-f]{1,32}$' || { echo "build_core.sh: TANGO_PRESET_STATUS must be up to 32 hex digits"; exit 2; }
    printf '`define TANGO_PRESET_STATUS 128%sh%s\n' "'" "$TANGO_PRESET_STATUS" >> "$DEFS"
fi
if [ -n "${TANGO_VPROC_PRESET:-}" ]; then    # a compiled-in load (tango_vproc_regs replays it after reset)
    [ $AFILT = 1 ] || { echo "build_core.sh: TANGO_VPROC_PRESET needs TANGO_VIDEO_PROC=1 or TANGO_AUDIO_FILTER=1"; exit 2; }
    [ -n "${TANGO_FILTERS_ROOT:-}" ] || { echo "build_core.sh: TANGO_VPROC_PRESET needs TANGO_FILTERS_ROOT, the folder with MiSTer's filter files (README)"; exit 2; }
    PSTAT=""; [ -n "${TANGO_PRESET_STATUS:-}" ] && PSTAT="--status $TANGO_PRESET_STATUS"
    eval "python3 sys/build_tools/vproc_preset.py --root \"\$TANGO_FILTERS_ROOT\" $PSTAT $TANGO_VPROC_PRESET --vh \"\$INC/tango_vproc_preset.vh\" --json \"\$OUTD/\${NAME}_vproc_preset.json\"" \
        || { echo "build_core.sh: vproc_preset.py failed"; exit 2; }
    printf '`define TANGO_VPROC_PRESET\n' >> "$DEFS"
fi
if [ "$CSDC" != - ]; then
    # Gowin STA: a set_clock_groups with two or more -group makes every clock it does not list asynchronous to
    # all clocks, even to another unlisted clock, and a later set_clock_groups cannot undo that
    # (README "Timing constraints").  The board's line lists only the board clocks, so it would cut the core's own
    # related clocks apart (nes_clk <-> nes_fclk: 0 paths timed).  A core SDC therefore carries the ONE
    # set_clock_groups line of the build (every board group + its own), and the board's line is dropped here.
    grep -q '^set_clock_groups' "$CSDC" || { echo "build_core.sh: $CSDC has no set_clock_groups line (README \"Timing constraints\")"; exit 2; }
    grep -v '^set_clock_groups' "$SDC" > "$SDC.tmp" && mv "$SDC.tmp" "$SDC"
    if [ -n "$DBGSFX" ]; then sed 's#get_nets {#get_nets {u/#g' "$CSDC" >> "$SDC"; else cat "$CSDC" >> "$SDC"; fi
fi
if [ "${RETUNE_SDC:-0}" = 1 ]; then
    grep -q '^set_clock_groups.*get_clocks {clk_ser clk_pix}' "$SDC" || \
        { echo "build_core.sh: TANGO_HDMI_RETUNE: the build's set_clock_groups has no {clk_ser clk_pix} group"; exit 2; }
    sed -i '/^set_clock_groups/s#get_clocks {clk_ser clk_pix}#get_clocks {clk_ser clk_pix clk_pix_raw}#' "$SDC"
fi
if [ $DDR3 = 1 ]; then
    # the DDR3 clocks, and each its own group in the build's one set_clock_groups line (unless a core SDC listed them)
    cat "$BRD/tango_console138k_ddr3.sdc" >> "$SDC"
    for c in memory_clk clk_x1; do
        grep '^set_clock_groups' "$SDC" | grep -q "get_clocks {$c}" || \
            sed -i "s#^\(set_clock_groups -asynchronous .*\)\$#\1 -group [get_clocks {$c}]#" "$SDC"
    done
    [ "$(grep -c '^set_clock_groups' "$SDC")" = 1 ] || { echo "build_core.sh: TANGO_VIDEO_DDR3: want one set_clock_groups line"; exit 2; }
fi
# TANGO_EXTRA_CST=<file>: a core's placement constraints (INS_LOC lines; never GROUP -- the Gowin GROUP memory bomb),
# appended to the board cst (experiment: the Genesis DDR3 DBG build's bus-data latch region)
if [ -n "${TANGO_EXTRA_CST:-}" ]; then
    [ -f "$TANGO_EXTRA_CST" ] || { echo "build_core.sh: TANGO_EXTRA_CST=$TANGO_EXTRA_CST: no such file"; exit 2; }
    # INS_LOC lines, comments and blank lines only: GROUP / GRP_LOC (the memory bomb) and anything else (an IO_LOC
    # would override the board's pins) are refused
    BAD=$(grep -nvE '^[[:space:]]*(INS_LOC[[:space:]]|//|$)' "$TANGO_EXTRA_CST" | head -3)
    [ -z "$BAD" ] || { echo "build_core.sh: TANGO_EXTRA_CST: INS_LOC lines only; refused: $BAD"; exit 2; }
    printf '\n// TANGO_EXTRA_CST %s\n' "$TANGO_EXTRA_CST" >> "$CST"; cat "$TANGO_EXTRA_CST" >> "$CST"
fi
RUN_START=$(mktemp); trap 'rm -f "$RUN_START"' EXIT
SHA=$(git rev-parse --short HEAD 2>/dev/null || echo nogit)
DTAG=""; [ -n "$(git status --porcelain --untracked-files=no -- sys fpga template harness 2>/dev/null)" ] && DTAG="-dirty"   # build inputs only
TANGO_ROOT="$ROOT" TANGO_NAME="$NAME" TANGO_CORE_FILES="$FILES" TANGO_SDC="$SDC" TANGO_DBG="${TANGO_DBG:-0}" \
TANGO_OUT_ABS="$OUTD" TANGO_INC="$INC" TANGO_CST="$CST" TANGO_DEFS="$DEFS" TANGO_PLL_INIT="$PLL_INIT_V" \
TANGO_VIDEO_ASCAL="${TANGO_VIDEO_ASCAL:-0}" TANGO_ASCAL_SRC="${TANGO_ASCAL_SRC:-ghdl}" TANGO_VIDEO_DDR3="$DDR3" \
TANGO_VIDEO_ROTATE="${TANGO_VIDEO_ROTATE:-0}" \
TANGO_VIDEO_PROC_FILES="$VPROC" TANGO_AUDIO_FILTER_FILES="$AFILT" TANGO_VSYNC_ADJUST="${TANGO_VSYNC_ADJUST:-0}" \
TANGO_NES_SDRAM="${TANGO_NES_SDRAM:-0}" TANGO_ASCAL_EXT="${TANGO_ASCAL_EXT:-0}" \
TANGO_MEM_CAP="${TANGO_MEM_CAP:-4G}" \
LD_LIBRARY_PATH="$GOWIN_HOME/IDE/lib" QT_QPA_PLATFORM=minimal \
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libfreetype.so.6 \
timeout 3600 nice -n 10 "$ROOT/$CAPPED" "$GW_SH" "$BRD/build_core.tcl"

IMPL=$OUTD/$NAME/impl/pnr
RPT=$IMPL/$NAME.rpt.txt
FS=$IMPL/$NAME.fs
echo "=== resources ==="
grep -E "^\s*(Logic|CLS|Register|BSRAM|DSP)\s*\|" "$RPT" | head -8 || true
if [ -f "$FS" ] && [ "$FS" -nt "$RUN_START" ]; then
  # Collect --relax directives embedded in the build's generated SDC as comments of the form:
  #   // ... pass --relax NAME:FREQ to sta_gate.py
  # (see tango_console138k_sdram108.sdc: PLL overconstrained for routing but relaxed for STA).
  STAG_RELAX=""
  while IFS= read -r rl; do
    STAG_RELAX="$STAG_RELAX --relax $rl"
  done < <(grep -oP '(?<=--relax )\S+(?= to sta_gate\.py)' "$SDC" 2>/dev/null || true)
  echo "=== sta gate ==="; set +e; python3 "$STAG" "$IMPL" $STAG_RELAX; SG=$?; set -e
  [ -n "${TANGO_NO_ARCHIVE:-}" ] && { echo "fresh bitstream (archive skipped: TANGO_NO_ARCHIVE)"; exit 0; }
  ARCH="${TANGO_ARCHIVE:-$OUTD/archive}"; mkdir -p "$ARCH"
  STAMP="$ARCH/${NAME}_$(date +%Y%m%d-%H%M)_g${SHA}${DTAG}"
  cp "$FS" "$STAMP.fs" && cmp "$FS" "$STAMP.fs" && chmod 555 "$STAMP.fs"
  python3 "$FS2BIN" "$STAMP.fs" "$STAMP.bin" && chmod 555 "$STAMP.bin"
  ls -l "$STAMP.fs" "$STAMP.bin"
  [ $SG -ne 0 ] && echo "WARNING: STA GATE FAIL -- do NOT flash"
  exit 0
else
  echo "NO FRESH BITSTREAM (missing, or older than this run = stale) -- check syn/pnr errors"; exit 1
fi
