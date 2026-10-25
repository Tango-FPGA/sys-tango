// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// ascal_wrap -- Verilog wrapper for TEMLIB's ascal.vhd (AVALON SCALER).
// Parameters: N_DW=64, N_AW=29 (tango_sdram port C; TANGO_VIDEO_DDR3: 128 / 28, the DDR3 scaler port and
// ascal_ghdl/ascal_shim{,8}_ddr3.v), INTER=true, BYTESWAP=true, HEADER=false,
//             DOWNSCALE=false, FRAC=4, OHRES=2048, IHRES=512, PALETTE=false.
// `ascal` is either the VHDL entity (Gowin mixed-language flow, TANGO_ASCAL_SRC=vhdl) or, by default,
// sys/ascal_ghdl/ascal_shim.v around GHDL's netlist of the same file, whose generics are fixed there.
// TANGO_VIDEO_PROC (README "Video and audio processing"): the polyphase coefficient port poly_* is driven, in o_clk
// (= clk_pix, the link's clock: no crossing), by tango_vproc_regs, and ascal has MiSTer's FRAC 8 (tango_cfg.vh
// TANGO_VPROC_FRAC; Template_MiSTer sys_top.v 735): 256 phases per set, the netlist ascal_fixed8.v with
// ascal_shim8.v (its coefficient memories in block RAM: sys/ascal_ghdl/ascal_poly_bram.patch).  Without it:
// FRAC 4, ascal_fixed.v + ascal_shim.v, poly_* tied off, as before.
//==============================================================================
`ifdef TANGO_VIDEO_PROC
`include "tango_cfg.vh"                 // TANGO_VPROC_FRAC
`endif
`default_nettype none
module ascal_wrap #(
    // RAMBASE and RAMSIZE are runtime-critical generics: pass as parameters
    parameter [31:0] RAMBASE = 32'h0140_0000,  // 20 MB mark: 3 x 4 MB buffers end at 32 MB (tango_cfg.vh)
    parameter [31:0] RAMSIZE = 32'h0040_0000   // 4 MB per frame buffer
) (
    // framebuffer mode (TANGO_VIDEO_ROTATE): when fb_ena is high, ascal's own output-read engine
    // reads from fb_base/fb_hsize/fb_vsize/fb_format/fb_stride instead of its own normal triple-buffer
    // bookkeeping (ascal.vhd 1922-1925, 1941-1942, 2063-2067). Default tied off below (ascal_wrap's own
    // instantiation, unchanged for every build that does not pass these) -- this is additive, not a
    // behavior change for any existing core.
    input  wire        fb_ena,
    input  wire [11:0] fb_hsize,
    input  wire [11:0] fb_vsize,
    input  wire [5:0]  fb_format,
    input  wire [31:0] fb_base,
    input  wire [13:0] fb_stride,
    output wire        fb_vbl,             // ascal's own o_vbl: the FB_VBL-equivalent for the writer's swap
    // core video input (i_clk domain)
    input  wire        i_clk,
    input  wire        i_ce,
    input  wire [7:0]  i_r, i_g, i_b,
    input  wire        i_hs, i_vs, i_de,
    input  wire        i_fl,          // field (0 for progressive)
    // output video (o_clk domain)
    input  wire        o_clk,         // = clk_pix (74.25 MHz)
    input  wire        o_ce,          // tango_video's sweep-alignment enable (low once, then 1)
    output wire [7:0]  o_r, o_g, o_b,
    output wire        o_hs, o_vs, o_de,
    // geometry / config (quasi-static, o_clk domain, updated at frame boundary)
    input  wire [11:0] hmin, hmax,    // output window on 720p canvas
    input  wire [10:0] vmin, vmax,
    // output timing (must match 720p generator)
    input  wire [11:0] htotal, hsstart, hsend, hdisp,
    input  wire [11:0] vtotal, vsstart, vsend, vdisp,
    // ascal mode (ascal.vhd header): [2:0] filter (000 nearest), [3] triple buffering, [4] unused
    input  wire [4:0]  mode,
`ifdef TANGO_VIDEO_PROC
    // polyphase coefficients (o_clk): ascal.vhd 230-238, 2408-2437; poly_a = {set[1:0], phase[FRAC-1:0], tap[1:0]}
    input  wire        poly_wr,
    input  wire [`TANGO_VPROC_FRAC+3:0] poly_a,
    input  wire [9:0]  poly_dw,
`endif
`ifdef TANGO_VIDEO_DDR3
    // Avalon MM to the DDR3 scaler port (avl_clk = clk_ddr, the DDR3 IP's 100 MHz): N_DW 128,
    // N_AW 28 (16-byte words, byte_addr = {avl_address, 4'b0}), MiSTer's widths
    input  wire        avl_clk,
    output wire [27:0] avl_address,
    output wire        avl_write,
    output wire [127:0] avl_writedata,
    output wire        avl_read,
    output wire [7:0]  avl_burstcount,
    output wire [15:0] avl_byteenable,
    input  wire [127:0] avl_readdata,
`else
    // Avalon MM to SDRAM (avl_clk domain = clk_sdram = 96 MHz)
    input  wire        avl_clk,
    output wire [28:0] avl_address,  // 29-bit word address (N_AW=29, byte_addr = {avl_address,3'b0})
    output wire        avl_write,
    output wire [63:0] avl_writedata,
    output wire        avl_read,
    output wire [7:0]  avl_burstcount,
    output wire [7:0]  avl_byteenable,
    input  wire [63:0] avl_readdata,
`endif
    input  wire        avl_readdatavalid,
    input  wire        avl_waitrequest,
    // active-low async reset
    input  wire        reset_na,
    // DEBUG OUTPUTS (avl_clk): Avalon handshake monitoring (optional, tied off if not used)
    output reg  [31:0] dbg_read_count,
    output reg  [31:0] dbg_readdata_valid_count,
    output reg  [31:0] dbg_write_count,          // write beats accepted
    output reg  [28:0] dbg_last_address,
    output reg  [63:0] dbg_last_readdata,
    output reg         dbg_waitrequest_stuck,
    output wire        dbg_avl_read,
    output wire        dbg_avl_readdatavalid,
    output wire        dbg_avl_waitrequest
);

    // Avalon buses are std_logic_vector in the VHDL entity; wire directly.
    // VHDL unsigned(7 DOWNTO 0) ports map to plain wire [7:0] in Verilog
    // (Gowin's mixed-language flow handles the type mapping).

    // ========== DEBUG INSTRUMENTATION: Avalon handshake monitor ==========
    // Capture key signals to verify ascal can read from SDRAM frame buffer

    always @(posedge avl_clk or negedge reset_na) begin
        if (!reset_na) begin
            dbg_read_count <= 32'd0;
            dbg_readdata_valid_count <= 32'd0;
            dbg_write_count <= 32'd0;
            dbg_last_address <= 29'd0;
            dbg_last_readdata <= 64'd0;
        end else begin
            // Count avl_read strobes (ascal trying to read)
            if (avl_read && !avl_waitrequest)
                dbg_read_count <= dbg_read_count + 32'd1;

            // Count avl_readdatavalid pulses (SDRAM responding)
            if (avl_readdatavalid)
                dbg_readdata_valid_count <= dbg_readdata_valid_count + 32'd1;

            if (avl_write && !avl_waitrequest)
                dbg_write_count <= dbg_write_count + 32'd1;

            // Capture last address and data
            if (avl_read)
                dbg_last_address <= avl_address;
            if (avl_readdatavalid)
                dbg_last_readdata <= avl_readdata[63:0];
        end
    end

    // Flag: if waitrequest stuck high for >100 cycles, ascal is stalled
    reg [7:0] waitrequest_holdoff;
    always @(posedge avl_clk or negedge reset_na) begin
        if (!reset_na) begin
            waitrequest_holdoff <= 8'd0;
            dbg_waitrequest_stuck <= 1'b0;
        end else begin
            if (avl_waitrequest)
                waitrequest_holdoff <= (waitrequest_holdoff < 8'd100) ? waitrequest_holdoff + 8'd1 : 8'd100;
            else
                waitrequest_holdoff <= 8'd0;
            dbg_waitrequest_stuck <= (waitrequest_holdoff >= 8'd100);
        end
    end
    // =====================================================================

    ascal #(
        .MASK        (8'hFF),
        .RAMBASE     (RAMBASE),
        .RAMSIZE     (RAMSIZE),
        .INTER       (1'b1),
        .HEADER      (1'b0),
        .DOWNSCALE   (1'b0),
        .BYTESWAP    (1'b1),
        .PALETTE     (1'b0),
        .PALETTE2    (1'b0),
        .ADAPTIVE    (1'b1),
        .DOWNSCALE_NN(1'b0),
`ifdef TANGO_VIDEO_PROC
        .FRAC        (`TANGO_VPROC_FRAC),   // 8: ascal_shim8.v / ascal_fixed8.v
`else
        .FRAC        (4),
`endif
        .OHRES       (2048),
        .IHRES       (512),
`ifdef TANGO_VIDEO_DDR3
        .N_DW        (128),
        .N_AW        (28),  // the DDR3 scaler port: N_DW=128, N_AW=28, byte_addr={avl_address,4'b0}
`else
        .N_DW        (64),
        .N_AW        (29),  // match tango_sdram: N_DW=64, N_AW=29, byte_addr={avl_address,3'b0}
`endif
        .N_BURST     (256)
    ) u_ascal (
        // input video
        .i_r    (i_r),
        .i_g    (i_g),
        .i_b    (i_b),
        .i_hs   (i_hs),
        .i_vs   (i_vs),
        .i_fl   (i_fl),
        .i_de   (i_de),
        .i_ce   (i_ce),
        .i_clk  (i_clk),
        // output video
        .o_r    (o_r),
        .o_g    (o_g),
        .o_b    (o_b),
        .o_hs   (o_hs),
        .o_vs   (o_vs),
        .o_de   (o_de),
        .o_vbl  (fb_vbl),
        .o_brd  (),
        .o_ce   (o_ce),
        .o_clk  (o_clk),
        // border colour
        .o_border   (24'h000000),
        // framebuffer mode (TANGO_VIDEO_ROTATE): the fb_* ports above, real wires, not a
        // hardwired tie-off -- every build that does not pass them (fb_ena tied to 0 by its caller) is
        // unchanged: ascal takes the "disabled (direct-input mode)" path exactly as before.
        .o_fb_ena    (fb_ena),
        .o_fb_hsize  (fb_hsize),
        .o_fb_vsize  (fb_vsize),
        .o_fb_format (fb_format),
        .o_fb_base   (fb_base),
        .o_fb_stride (fb_stride),
        // palette: unused (PALETTE=false)
        .pal1_clk (1'b0),
        .pal1_dw  (48'd0),
        .pal1_a   (7'd0),
        .pal1_wr  (1'b0),
        .pal_n    (1'b0),
        .pal2_clk (1'b0),
        .pal2_dw  (24'd0),
        .pal2_a   (8'd0),
        .pal2_wr  (1'b0),
        // ll tune output: unused
        .o_lltune (),
        // input video parameters
        .iauto  (1'b1),
        .himin  (12'd0),
        .himax  (12'd0),
        .vimin  (12'd0),
        .vimax  (12'd0),
        .i_hdmax(),
        .i_vdmax(),
        // output video parameters
        .run    (1'b1),
        .freeze (1'b0),
        .mode   (mode),
        .htotal  (htotal),
        .hsstart (hsstart),
        .hsend   (hsend),
        .hdisp   (hdisp),
        .hmin    (hmin),
        .hmax    (hmax),
        .vtotal  (vtotal),
        .vsstart (vsstart),
        .vsend   (vsend),
        .vdisp   (vdisp),
        .vmin    (vmin),
        .vmax    (vmax),
        .vrr     (1'b0),
        .vrrmax  (12'd0),
        // format: 24bpp
        .format  (2'b01),
`ifdef TANGO_VIDEO_PROC
        // polyphase coefficients from the link (tango_vproc_regs), in o_clk
        .poly_clk (o_clk),
        .poly_dw  (poly_dw),
        .poly_a   (poly_a),           // FRAC+3 DOWNTO 0 = 11:0 for FRAC=8
        .poly_wr  (poly_wr),
`else
        // polyphase: unused (bilinear fallback)
        .poly_clk (1'b0),
        .poly_dw  (10'd0),
        .poly_a   ({(4+4){1'b0}}),   // FRAC+3 DOWNTO 0 = 7:0 for FRAC=4
        .poly_wr  (1'b0),
`endif
        // Avalon
        .avl_clk           (avl_clk),
        .avl_waitrequest   (avl_waitrequest),
        .avl_readdata      (avl_readdata),
        .avl_readdatavalid (avl_readdatavalid),
        .avl_burstcount    (avl_burstcount),
        .avl_writedata     (avl_writedata),
        .avl_address       (avl_address),
        .avl_write         (avl_write),
        .avl_read          (avl_read),
        .avl_byteenable    (avl_byteenable),
        // reset
        .reset_na (reset_na)
    );

    // Wire passthrough debug outputs
    assign dbg_avl_read = avl_read;
    assign dbg_avl_readdatavalid = avl_readdatavalid;
    assign dbg_avl_waitrequest = avl_waitrequest;

endmodule
`default_nettype wire
