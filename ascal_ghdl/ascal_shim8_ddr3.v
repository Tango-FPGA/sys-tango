// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// ascal_shim8_ddr3 -- FRAC 8: the TANGO_VIDEO_DDR3 variant of ascal_shim8.v (Avalon N_DW 128, N_AW 28, RAMBASE 0x2000_0000:
// the DDR3 scaler port), around the GHDL netlist ascal_fixed8_ddr3.v (gen.sh 8d).
// Everything else is ascal_shim8.v's; its header follows.
// ascal_shim8 -- `module ascal` (FRAC 8) for Verilator AND Gowin builds: the REAL TEMLIB ascal.vhd with the Tango
// patch ascal_poly_bram.patch, as the GHDL netlist of ascal_fixed8_ddr3.vhd (ascal_fixed8.v, made by gen.sh 8), behind the parameter
// and port names sys/ascal_wrap.v uses, so ascal_wrap.v compiles unmodified.  Gowin's VHDL front
// end builds ascal.vhd wrongly (on silicon: picture squeezed to 3/4 width while this netlist
// simulates pixel-exact), so the board build uses this netlist too (build_core.sh TANGO_ASCAL_SRC).
// Build with: sys/ascal_wrap.v + this file + ascal_fixed8_ddr3.v  (instead of ascal.vhd).
// THIS FILE is the FRAC 8 variant (ascal_fixed8_ddr3.vhd, `gen.sh 8`): the TANGO_VIDEO_PROC builds, whose
// polyphase filters use MiSTer's FRAC (README "Video and audio processing").  Every other build uses ascal_shim.v
// + ascal_fixed.v (FRAC 4), unchanged.  A build compiles one shim, never both (both define `module ascal`).
//
// Parameters are accepted and ignored: their values are fixed in ascal_fixed.vhd and must
// match; the guard below stops the simulation at time 0 if an instance asks for others.
// Port widths are the ones those generics give (N_DW=128, N_AW=28, FRAC=8).
// VHDL port defaults are not modelled: connect every input (ascal_wrap.v does).
`default_nettype none
module ascal #(
    parameter [7:0]   MASK         = 8'hFF,
    parameter [31:0]  RAMBASE      = 32'h2000_0000,
    parameter [31:0]  RAMSIZE      = 32'h0040_0000,
    parameter         INTER        = 1'b1,
    parameter         HEADER       = 1'b0,
    parameter         DOWNSCALE    = 1'b0,
    parameter         BYTESWAP     = 1'b1,
    parameter         PALETTE      = 1'b0,
    parameter         PALETTE2     = 1'b0,
    parameter         ADAPTIVE     = 1'b1,
    parameter         DOWNSCALE_NN = 1'b0,
    parameter integer FRAC         = 8,
    parameter integer OHRES        = 2048,
    parameter integer IHRES        = 512,
    parameter integer N_DW         = 128,
    parameter integer N_AW         = 28,
    parameter integer N_BURST      = 256
) (
    // input video
    input  wire [7:0]  i_r, i_g, i_b,
    input  wire        i_hs, i_vs, i_fl, i_de, i_ce, i_clk,
    // output video
    output wire [7:0]  o_r, o_g, o_b,
    output wire        o_hs, o_vs, o_de, o_vbl, o_brd,
    input  wire        o_ce, o_clk,
    input  wire [23:0] o_border,
    // framebuffer mode
    input  wire        o_fb_ena,
    input  wire [11:0] o_fb_hsize, o_fb_vsize,    // VHDL natural 0..4095
    input  wire [5:0]  o_fb_format,
    input  wire [31:0] o_fb_base,
    input  wire [13:0] o_fb_stride,
    // palette (PALETTE=false: pal1_dr/pal2_dr read 0)
    input  wire        pal1_clk,
    input  wire [47:0] pal1_dw,
    output wire [47:0] pal1_dr,
    input  wire [6:0]  pal1_a,
    input  wire        pal1_wr,
    input  wire        pal_n,
    input  wire        pal2_clk,
    input  wire [23:0] pal2_dw,
    output wire [23:0] pal2_dr,
    input  wire [7:0]  pal2_a,
    input  wire        pal2_wr,
    // low lag PLL tuning
    output wire [15:0] o_lltune,
    // input video parameters (VHDL natural 0..4095 -> 12 bits)
    input  wire        iauto,
    input  wire [11:0] himin, himax, vimin, vimax,
    output wire [11:0] i_hdmax, i_vdmax,
    // output video parameters (VHDL natural 0..4095 -> 12 bits)
    input  wire        run, freeze,
    input  wire [4:0]  mode,
    input  wire [11:0] htotal, hsstart, hsend, hdisp, hmin, hmax,
    input  wire [11:0] vtotal, vsstart, vsend, vdisp, vmin, vmax,
    input  wire        vrr,
    input  wire [11:0] vrrmax,
    input  wire [1:0]  format,
    // polyphase coefficients
    input  wire        poly_clk,
    input  wire [9:0]  poly_dw,
    input  wire [11:0] poly_a,                    // FRAC+3:0
    input  wire        poly_wr,
    // Avalon
    input  wire        avl_clk,
    input  wire        avl_waitrequest,
    input  wire [127:0] avl_readdata,              // N_DW-1:0
    input  wire        avl_readdatavalid,
    output wire [7:0]  avl_burstcount,
    output wire [127:0] avl_writedata,             // N_DW-1:0
    output wire [27:0] avl_address,               // N_AW-1:0
    output wire        avl_write,
    output wire        avl_read,
    output wire [15:0] avl_byteenable,            // N_DW/8-1:0
    input  wire        reset_na
);

    // Checked at time 0, not at elaboration: Verilator 5.020 demotes elaboration-time
    // $error/$fatal to warnings under -Wno-fatal, which the sim scripts pass.
    localparam GENERICS_MATCH =
        MASK == 8'hFF && RAMBASE == 32'h2000_0000 && RAMSIZE == 32'h0040_0000 &&
        INTER == 1 && HEADER == 0 && DOWNSCALE == 0 && BYTESWAP == 1 && PALETTE == 0 &&
        PALETTE2 == 0 && ADAPTIVE == 1 && DOWNSCALE_NN == 0 && FRAC == 8 &&
        OHRES == 2048 && IHRES == 512 && N_DW == 128 && N_AW == 28 && N_BURST == 256;
    // synthesis translate_off
    initial if (!GENERICS_MATCH)
        $fatal(1, "ascal_shim8_ddr3: ascal parameters differ from the generics baked into ascal_fixed8.v; edit sys/ascal_ghdl/ascal_fixed8_ddr3.vhd (and GENERICS_MATCH here), then rerun sys/ascal_ghdl/gen.sh 8");
    // synthesis translate_on

    ascal_fixed8_ddr3 u_ascal_fixed (
        .i_r(i_r), .i_g(i_g), .i_b(i_b),
        .i_hs(i_hs), .i_vs(i_vs), .i_fl(i_fl), .i_de(i_de), .i_ce(i_ce), .i_clk(i_clk),
        .o_r(o_r), .o_g(o_g), .o_b(o_b),
        .o_hs(o_hs), .o_vs(o_vs), .o_de(o_de), .o_vbl(o_vbl), .o_brd(o_brd),
        .o_ce(o_ce), .o_clk(o_clk),
        .o_border(o_border),
        .o_fb_ena(o_fb_ena), .o_fb_hsize(o_fb_hsize), .o_fb_vsize(o_fb_vsize),
        .o_fb_format(o_fb_format), .o_fb_base(o_fb_base), .o_fb_stride(o_fb_stride),
        .pal1_clk(pal1_clk), .pal1_dw(pal1_dw), .pal1_dr(pal1_dr),
        .pal1_a(pal1_a), .pal1_wr(pal1_wr),
        .pal_n(pal_n),
        .pal2_clk(pal2_clk), .pal2_dw(pal2_dw), .pal2_dr(pal2_dr),
        .pal2_a(pal2_a), .pal2_wr(pal2_wr),
        .o_lltune(o_lltune),
        .iauto(iauto),
        .himin(himin), .himax(himax), .vimin(vimin), .vimax(vimax),
        .i_hdmax(i_hdmax), .i_vdmax(i_vdmax),
        .run(run), .freeze(freeze), .mode(mode),
        .htotal(htotal), .hsstart(hsstart), .hsend(hsend), .hdisp(hdisp),
        .hmin(hmin), .hmax(hmax),
        .vtotal(vtotal), .vsstart(vsstart), .vsend(vsend), .vdisp(vdisp),
        .vmin(vmin), .vmax(vmax),
        .vrr(vrr), .vrrmax(vrrmax),
        .format(format),
        .poly_clk(poly_clk), .poly_dw(poly_dw), .poly_a(poly_a), .poly_wr(poly_wr),
        .avl_clk(avl_clk),
        .avl_waitrequest(avl_waitrequest),
        .avl_readdata(avl_readdata),
        .avl_readdatavalid(avl_readdatavalid),
        .avl_burstcount(avl_burstcount),
        .avl_writedata(avl_writedata),
        .avl_address(avl_address),
        .avl_write(avl_write),
        .avl_read(avl_read),
        .avl_byteenable(avl_byteenable),
        .reset_na(reset_na)
    );

endmodule
`default_nettype wire
