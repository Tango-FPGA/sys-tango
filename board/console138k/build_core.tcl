# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
# build_core.tcl -- build a Tango core for the Tang Console 138K: the framework (sys/) + the core's
# files.  Driven by build_core.sh through environment variables:
#   TANGO_ROOT        the core's checkout (sys/ lives here)
#   TANGO_NAME        project / output name, e.g. tango_template138
#   TANGO_CORE_FILES  the core's HDL files (absolute paths, space separated); one defines module tango_core
#   TANGO_SDC         the SDC to use (board SDC, or the board SDC + the core's clocks)
#   TANGO_DBG         1 = debug build (link-debug blocks on screen; top tango_sys_top138_dbg)
# Device and options: GW5AST-LV138PG484AC1/I0, device version B, IOB registers, plus
# -use_jtag_as_gpio 1 for the BL616 link on the JTAG balls, as the A2600Nano Console 138K build.
set root $env(TANGO_ROOT)
set sys  $root/sys
set brd  $sys/board/console138k
set dbg  [expr {[info exists env(TANGO_DBG)] && $env(TANGO_DBG) == "1"}]
create_project -name $env(TANGO_NAME) -dir $env(TANGO_OUT_ABS) -pn GW5AST-LV138PG484AC1/I0 -device_version B -force
add_file $env(TANGO_DEFS)          ;# the build's `defines (TANGO_SDRAM), first: build_core.sh
foreach f [list \
    $brd/pll_init.v $brd/pll_c27_mod.v $brd/pll_c27.v $brd/pll_c1485_mod.v $brd/pll_c1485.v \
    $brd/pll_c48_mod.v $brd/pll_c48.v $brd/tango_tmds_ser.v $brd/tango_sys_top138.v \
    $sys/usb/usb_ls_detect.v $sys/usb/usb_fs_phy.v $sys/usb/usb_fs_host.v \
    $sys/tango_beam_lead.v $sys/tango_menu_overlay.v $sys/tango_link.v $sys/tango_link_dbg_view.v \
    $sys/tango_usb_port.v $sys/tango_ds4_joy.v $sys/tango_pad_decode.v $sys/tango_desc_ram.v $sys/tango_cdc.v $sys/tango_io.v $sys/tango_conf_rom.v $sys/tango_sd.v \
    $sys/tango_video.v $sys/tango_video_geo.v \
    $sys/tango_sys.v] { add_file $f }
set ddr3 [expr {[info exists env(TANGO_VIDEO_DDR3)] && $env(TANGO_VIDEO_DDR3) == "1"}]
if {$ddr3} {
    # TANGO_VIDEO_DDR3 (README "The DDR3 path"): the Gowin DDR3 IP generated into the build (never committed), its PLL, and
    # tango_ddr3 (the scaler port, the arbiter, the core port's stub)
    source $sys/ddr3/tango_ddr3_ip.tcl
    tango_ddr3_gen_ip $env(TANGO_OUT_ABS)/$env(TANGO_NAME)_ip
    foreach f [list $brd/pll_ddr3_mod.v $brd/pll_ddr3.v $sys/tango_ddr3.v $sys/tango_ddr3_arb.v $sys/tango_ddr3_vport.v \
               $sys/tango_ddr3_cport.v $sys/tango_ddr3_fifo.v] { add_file $f }
}
if {[info exists env(TANGO_VIDEO_ASCAL)] && $env(TANGO_VIDEO_ASCAL) == "1"} {
    if {$ddr3} {
        add_file $sys/ascal_wrap.v
        # TANGO_VIDEO_DDR3 + TANGO_NES_SDRAM: tango_sdram for the NES's ports A/B (tango_cfg.vh TANGO_FW_SDRAM)
        if {[info exists env(TANGO_NES_SDRAM)] && $env(TANGO_NES_SDRAM) == "1"} {
            foreach f [list $brd/gowin_pll_sdram.v $sys/tango_sdram.v $sys/tango_ab_slot.v] { add_file $f }
        }
    } elseif {[info exists env(TANGO_ASCAL_EXT)] && $env(TANGO_ASCAL_EXT) == "1"} {
        # TANGO_ASCAL_EXT: the core's controller serves ascal (tango_cfg.vh); tango_ascal_words.v is its helper
        foreach f [list $sys/ascal_wrap.v $sys/tango_ascal_words.v] { add_file $f }
    } else {
    foreach f [list $brd/gowin_pll_sdram.v $sys/tango_sdram.v $sys/tango_ab_slot.v $sys/ascal_wrap.v] { add_file $f }
    }
    # ascal: GHDL's netlist of ascal.vhd (sys/ascal_ghdl/, the logic the sims run) unless TANGO_ASCAL_SRC=vhdl;
    # GowinSynthesis's own VHDL build of ascal.vhd squeezed the picture to 3/4 width on silicon.
    if {[info exists env(TANGO_ASCAL_SRC)] && $env(TANGO_ASCAL_SRC) == "vhdl"} {
        add_file $sys/ascal.vhd
    } elseif {[info exists env(TANGO_VIDEO_PROC_FILES)] && $env(TANGO_VIDEO_PROC_FILES) == "1"} {
        # TANGO_VIDEO_PROC: MiSTer's FRAC 8, the coefficient memories in block RAM (ascal_ghdl/gen.sh 8; DDR3: 8d)
        set d [expr {$ddr3 ? "_ddr3" : ""}]
        foreach f [list $sys/ascal_ghdl/ascal_shim8$d.v $sys/ascal_ghdl/ascal_fixed8$d.v] { add_file $f }
    } else {
        set d [expr {$ddr3 ? "_ddr3" : ""}]
        foreach f [list $sys/ascal_ghdl/ascal_shim$d.v $sys/ascal_ghdl/ascal_fixed$d.v] { add_file $f }
    }
}
# TANGO_VIDEO_ROTATE (README "Rotation"): the DDR3-direct rotation client, writing through
# tango_ddr3's C0 port (wired in tango_sys.v, gated by the same `define; build_core.sh also sets
# `TANGO_DDRAM` for this flag). Not set: this file is never added -- the core's file list, and so its
# netlist, is exactly as before this option existed.
if {[info exists env(TANGO_VIDEO_ROTATE)] && $env(TANGO_VIDEO_ROTATE) == "1"} {
    add_file $sys/tango_video_rotate_ddr3.v
}
# MiSTer's video / audio processing (README "Video and audio processing"): build_core.sh sets these for TANGO_VIDEO_PROC /
# TANGO_AUDIO_FILTER builds only, so no other build's file list changes
if {[info exists env(TANGO_VSYNC_ADJUST)] && $env(TANGO_VSYNC_ADJUST) == "1"} {
    # native-rate output (README "Native-rate video (vsync_adjust)"): the planner, the runtime beam lead and the PLL retune
    foreach f [list $sys/tango_vsync.v $sys/tango_beam_lead_rt.v $sys/tango_pll_retune.v] { add_file $f }
}
if {[info exists env(TANGO_AUDIO_FILTER_FILES)] && $env(TANGO_AUDIO_FILTER_FILES) == "1"} {
    foreach f [list $sys/tango_vproc_regs.v $sys/tango_audio_filter.v] { add_file $f }
}
if {[info exists env(TANGO_VIDEO_PROC_FILES)] && $env(TANGO_VIDEO_PROC_FILES) == "1"} {
    foreach f [list $sys/tango_gamma.v $sys/tango_shmask.v] { add_file $f }
}
if {$dbg} { add_file $brd/tango_sys_top138_dbg.v }
foreach f [lsort [glob $sys/hdmi/*.sv]] { add_file $f }
foreach f $env(TANGO_CORE_FILES) { add_file $f }
add_file $env(TANGO_CST)          ;# the board cst (+ the SDRAM pins for TANGO_SDRAM=1), made by build_core.sh
add_file $env(TANGO_SDC)
set_option -top_module [expr {$dbg ? "tango_sys_top138_dbg" : "tango_sys_top138"}]
set_option -include_path $env(TANGO_INC)   ;# tango_cfg.vh + copies of sys/*.vh (build_core.sh): one directory
set_option -verilog_std sysv2017
set_option -vhdl_std vhd2008          ;# cores may bring VHDL (A2600: A2600Nano's A2601 uses VHDL-2008, as its own build)
# tango_conf_rom's generate loop runs 32 x ceil(len / 32) times: 4096 for the longest CONF_STR (4095 bytes,
# 12-bit conf_addr; README "Using it in a core").  GowinSynthesis stops a loop at 2000 by default (SUG100-4.4.2E
# p.104 "-looplimit"), so any CONF_STR over 2000 bytes failed with EX3934 (NES_MiSTer fit check, 2276 bytes).
set_option -looplimit 4096
set_option -use_jtag_as_gpio 1
set_option -ireg_in_iob 1
set_option -oreg_in_iob 1
set_option -ioreg_in_iob 1
# TANGO_SYN_RETIME (opt-in, default off -- every existing core/build is unaffected unless it explicitly
# sets this): Gowin's own native retiming/timing-driven synthesis + max-effort place&route + automatic
# hold-fix options (gw_sh `set_option -h`: -retiming, -timing_driven, -place_option, -route_option,
# -correct_hold_violation), as opposed to a hand-written RTL retiming -- lower risk, since none of it
# changes what is computed, only how the vendor tool synthesizes/places/routes it. For a core whose timing
# closure is tight: on one core a manual RTL retiming made setup timing worse (22 -> 170 violated
# endpoints), so the vendor tool's own techniques are offered first.
if {[info exists env(TANGO_SYN_RETIME)] && $env(TANGO_SYN_RETIME) == "1"} {
    set_option -retiming 1
    set_option -timing_driven 1
    set_option -place_option 4
    set_option -route_option 2
    set_option -correct_hold_violation 1
}
run syn
run pnr
