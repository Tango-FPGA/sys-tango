// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_video -- Option B: ascal.vhd + SDRAM (TANGO_VIDEO_ASCAL builds).
// The core outputs its video on MiSTer emu ports (CLK_VIDEO / CE_PIXEL / VGA_*);
// ascal_wrap captures it in direct-input mode, writes frames to SDRAM via an
// Avalon MM 64-bit bus (port C of tango_sdram), and outputs scaled 720p60 video
// aligned to clk_pix.  The BSRAM frame buffer (E1..E11 pipeline) is gone.
//
// External interface is backward-compatible with the BSRAM version; BSRAM-only
// outputs (show_seq, show_valid, show_take, dbg_lost, dbg_frames) are tied to
// zero so existing callers need no changes.
//
// VLAT = 2: ascal's output is one registered stage inside this module, plus one
// register in tango_sys.v (bg_q).  tango_sys must be built with VLAT = 2 in the
// TANGO_VIDEO_ASCAL block.
//
// TANGO_VIDEO_OPTS (a core opts in; tango_video_opts.vh): the Video options are sys's own status bits, read here
// from the option word (port status): Aspect ratio picks VIDEO_ARX/ARY (Original = the core's), Scale is the
// scaler's SCALE and Scanlines the scanline level; the core's VIDEO_SCALE and VGA_SL are not used ("Video options").
//
// TANGO_VIDEO_F1 (tango_cfg.vh): the core's interlace field (MiSTer's VGA_F1) is ascal's i_fl, so ascal detects interlaced
// video and weaves the two fields (ascal.vhd 1170-1177, 1268-1280; MiSTer's sys_top.v connects VGA_F1 the same way), and
// the framework's Scanlines are off while the input is interlaced (MiSTer cores mask VGA_SL in interlace themselves,
// e.g. Genesis_MiSTer Genesis.sv 697; here sys owns Scanlines).  Interlaced = the field changed at a CE_PIXEL clock within
// the last three input frames, as ascal decides it (i_intercnt); taken with each input frame's record, so it applies at
// a frame boundary like the core's own options.  Without TANGO_VIDEO_F1, i_fl is 0 and nothing here exists.
//
// TANGO_VIDEO_PROC (README "Video and audio processing"; registers from tango_vproc_regs, all clk_pix): MiSTer's video
// processing.  Gamma (tango_gamma) on the core's pixels before ascal, in clk_video; ascal's polyphase coefficients
// (poly_*) and mode[2] (flt_mode[0]: polyphase on, else nearest); the shadow mask (tango_shmask) after the scanline
// dimming, which flt_mode = 11 (polyphase on, the V set is the Scan filter) turns off.  The mask adds MASK_LAT + 1
// pipeline clocks: VLAT = 3 + MASK_LAT = 9 (tango_sys leads this module's beam by LAT + VLAT - 2, so the picture
// stays put).  sl_level reports the scanline level in use (FLTNUM's flags).
//==============================================================================
`include "tango_cfg.vh"
`include "tango_video_opts.vh"
`default_nettype none
module tango_video #(
    parameter integer MAXW  = 256,
    parameter integer MAXH  = 240,
    parameter integer BPC   = 8,
    parameter integer IDXW  = 0,          // unused in ascal path (always RGB)
    parameter integer NBUF  = 3,          // unused (ascal handles buffering)
    parameter integer FSWAP = 725,        // kept for port compatibility
    parameter integer LSET  = 1284,       // kept for port compatibility
    parameter integer FW = 1650, FH = 750, OW = 1280, OH = 720,
    // ascal RAMBASE / RAMSIZE must agree with tango_cfg.vh macros
    parameter [31:0] ASCAL_RAMBASE = 32'h01C0_0000,
    parameter [31:0] ASCAL_RAMSIZE = 32'h0040_0000
) (
    // ---- core video (clk_video), MiSTer emu ports
    input  wire        clk_video,
    input  wire        ce_pixel,
    input  wire [7:0]  vga_r, vga_g, vga_b,
    input  wire        vga_hs,
    input  wire        vga_vs,
    input  wire        vga_de,
    input  wire        vga_idx,        // index mode (unused in ascal path)
    input  wire [1:0]  vga_sl,         // scanlines
    input  wire [12:0] video_arx,
    input  wire [12:0] video_ary,
    input  wire [2:0]  video_scale,
    input  wire [11:0] video_crop_size,
    input  wire [4:0]  video_crop_off,
    input  wire        hdmi_freeze,
`ifdef TANGO_VIDEO_F1
    input  wire        vga_f1,         // the core's interlace field (MiSTer VGA_F1), clk_video
`endif
`ifdef TANGO_VIDEO_OPTS
    input  wire [127:0] status,        // clk_pix: the option word as tango_link holds it; sys's bits decoded below
`endif
`ifdef TANGO_VIDEO_PROC
    // MiSTer's video processing (header), clk_pix
    input  wire        poly_wr,
    input  wire [`TANGO_VPROC_FRAC+3:0] poly_a,     // {set, phase, tap}: ascal FRAC+3..0 (tango_cfg.vh)
    input  wire [9:0]  poly_dw,
    input  wire [1:0]  flt_mode,       // bit0 polyphase on, bit1 the V set is the Scan filter
    input  wire        gam_en,
    input  wire        gam_wr,
    input  wire [9:0]  gam_a,
    input  wire [7:0]  gam_d,
    input  wire        mask_wr,
    input  wire [15:0] mask_cmd,
    output wire [1:0]  sl_level,       // the scanline level in use
`endif
    input  wire        pal_wr,         // palette (unused)
    input  wire        pal_a,
    input  wire [23:0] pal_d,
`ifdef TANGO_VIDEO_ROTATE
`ifdef TANGO_VIDEO_DDR3
    // framebuffer mode (TANGO_VIDEO_ROTATE): tango_video_rotate_ddr3's own o_fb_* outputs, already
    // CDC'd into clk_pix there (ascal_wrap.v's own "quasi-static, o_clk domain" discipline). Passed straight
    // through to ascal_wrap -- this module makes no decision about rotation, it only carries the wires.
    input  wire        rot_fb_ena,
    input  wire [11:0] rot_fb_hsize,
    input  wire [11:0] rot_fb_vsize,
    input  wire [5:0]  rot_fb_format,
    input  wire [31:0] rot_fb_base,
    input  wire [13:0] rot_fb_stride,
    output wire        rot_fb_vbl,       // ascal's own o_vbl, back out for the writer's buffer-swap CDC
`endif
`endif
    // ---- 720p side (clk_pix)
    input  wire        clk_pix,
    input  wire        rst_pix,
    input  wire [11:0] cx,
    input  wire [10:0] cy,
`ifdef TANGO_VSYNC_ADJUST
    input  wire [11:0] vt_nxt_w,       // TANGO_VSYNC_ADJUST: the raster's totals for the NEXT frame (tango_vsync: they
    input  wire [10:0] vt_nxt_h,       // change only mid-frame), taken at this module's own frame end (below)
`endif
    output reg  [23:0] rgb = 24'd0,
    // ---- backward-compatible debug outputs (tied to 0)
    output wire [15:0] show_seq,
    output wire        show_valid,
    output wire        show_take,
    output wire [15:0] dbg_lost,
    output wire [15:0] dbg_frames,
    output wire [15:0] rseq,    // frame-locked seq matching rbank; only non-zero under TANGO_SIM
    // ---- ascal Avalon MM (to tango_sdram port C)
`ifdef TANGO_VIDEO_DDR3
    // TANGO_VIDEO_DDR3: to tango_ddr3's scaler port, 128-bit (ascal_wrap.v)
    output wire [27:0] avl_address,
    output wire        avl_write,
    output wire [127:0] avl_writedata,
    output wire        avl_read,
    output wire [7:0]  avl_burstcount,
    output wire [15:0] avl_byteenable,
    input  wire [127:0] avl_readdata,
`else
    output wire [28:0] avl_address,
    output wire        avl_write,
    output wire [63:0] avl_writedata,
    output wire        avl_read,
    output wire [7:0]  avl_burstcount,
    output wire [7:0]  avl_byteenable,
    input  wire [63:0] avl_readdata,
`endif
    input  wire        avl_readdatavalid,
    input  wire        avl_waitrequest,
    input  wire        avl_clk,        // 96 MHz SDRAM clock (TANGO_VIDEO_DDR3: clk_ddr, 100 MHz)
    // ---- ascal Avalon instruments (avl_clk domain)
    output wire [31:0] dbg_arc,        // read commands accepted
    output wire [31:0] dbg_arv,        // read data beats returned
    output wire [31:0] dbg_awc,        // write beats accepted
    output wire        dbg_aws,        // waitrequest held > 100 cycles
    output wire [63:0] dbg_ald,        // last read data beat
    // ---- scaler config (quasi-static, clk_pix domain)
    input  wire [4:0]  video_mode,     // {~lowlat, filter[1:0], 2'b00}
    // ---- output timing for ascal (720p fixed)
    input  wire [11:0] o_htotal, o_hsstart, o_hsend, o_hdisp,
    input  wire [11:0] o_vtotal, o_vsstart, o_vsend, o_vdisp
);
`ifdef TANGO_VIDEO_PROC
    localparam integer MASK_LAT = 6;   // tango_shmask (its dout is then registered once more into rgb)
    localparam integer VLAT = 3 + MASK_LAT;   // tango_sys.v uses the same number
`else
    localparam integer VLAT = 2;
`endif

    // ---- debug outputs: real signals under TANGO_SIM, otherwise tied to 0
`ifndef TANGO_SIM
    assign show_seq   = 16'd0;
    assign show_valid = 1'b0;
    assign show_take  = 1'b0;
    assign dbg_lost   = 16'd0;
    assign dbg_frames = 16'd0;
    assign rseq       = 16'd0;
`endif
`ifdef TANGO_VIDEO_ROTATE
`ifdef TANGO_VIDEO_DDR3
`ifdef TANGO_SIM
    // ascal_wrap_sim has no framebuffer-mode model yet (rotation is simulated against the
    // real GHDL netlist, not this behavioural stand-in) -- tied off so a TANGO_SIM build still elaborates
    assign rot_fb_vbl = 1'b0;
`endif
`endif
`endif

    //==================================================================== gamma (TANGO_VIDEO_PROC, header)
    // v_* = the core's video as the writer and ascal take it: through the gamma tables, or the core's own.
    wire [7:0] v_r, v_g, v_b;
    wire       v_hs, v_vs, v_de;
`ifdef TANGO_VIDEO_PROC
    tango_gamma u_gamma (
        .clk_w(clk_pix), .wr(gam_wr), .wa(gam_a), .wd(gam_d), .en_w(gam_en),
        .clk(clk_video), .ce(ce_pixel), .r_in(vga_r), .g_in(vga_g), .b_in(vga_b),
        .hs_in(vga_hs), .vs_in(vga_vs), .de_in(vga_de),
        .r_out(v_r), .g_out(v_g), .b_out(v_b), .hs_out(v_hs), .vs_out(v_vs), .de_out(v_de));
`else
    assign v_r = vga_r;  assign v_g = vga_g;  assign v_b = vga_b;
    assign v_hs = vga_hs;  assign v_vs = vga_vs;  assign v_de = vga_de;
`endif

    //==================================================================== writer: frame metadata CDC
    // Capture the core's quasi-static option fields every frame (same logic as BSRAM version).
    localparam integer OPTW = 48;
    reg [OPTW-1:0] opt_s0 = {OPTW{1'b0}}, opt_s1 = {OPTW{1'b0}};
    wire [OPTW-1:0] opt_in = {vga_sl, video_arx, video_ary, video_scale, video_crop_size, video_crop_off};
    reg  ce_r = 1'b0, de_r = 1'b0, vs_r = 1'b0;
    always @(posedge clk_video) begin
        ce_r <= ce_pixel; de_r <= v_de; vs_r <= v_vs;
        opt_s0 <= opt_in; opt_s1 <= opt_s0;
    end

    // ---- count frame dimensions (first line width and line count), same as BSRAM writer
    reg old_de = 1'b0, old_vs = 1'b0, started = 1'b0;
    reg [11:0] hcnt = 12'd0, hs_first = 12'd0, row = 12'd0;
    reg [OPTW-1:0] m_opt = {OPTW{1'b0}};
    reg [11:0]     m_hs  = 12'd0, m_vt = 12'd0;
    reg            fd_tog = 1'b0;
    wire de_fall = ce_r && !de_r && old_de;
    wire vs_rise = ce_r && vs_r && !old_vs;
    always @(posedge clk_video) begin
        if (ce_r) begin
            old_de <= de_r; old_vs <= vs_r;
            if (de_r) begin
                if (hcnt != 12'hFFF) hcnt <= hcnt + 12'd1;
            end
            if (de_fall) begin
                if (row == 12'd0) hs_first <= (hcnt > MAXW[11:0]) ? MAXW[11:0] : hcnt;
                hcnt <= 12'd0;
                if (row < MAXH[11:0]) row <= row + 12'd1;
            end
            if (vs_rise) begin
                if (started && row != 12'd0 && !hdmi_freeze) begin
                    m_hs  <= (row == 12'd0) ? ((hcnt > MAXW[11:0]) ? MAXW[11:0] : hcnt) : hs_first;
                    m_vt  <= (row > MAXH[11:0]) ? MAXH[11:0] : row;
                    m_opt <= opt_s1;
                    fd_tog <= ~fd_tog;
                end
                started <= 1'b1;
                row <= 12'd0; hcnt <= 12'd0;
            end
        end
    end

`ifdef TANGO_VIDEO_F1
    //==================================================================== TANGO_VIDEO_F1: interlaced? (header)
    // f1_r follows vga_f1 as de_r / vs_r follow the core's DE / VS; at CE clocks a change of the field arms il_cnt = 3
    // and every VS rise counts it down (ascal.vhd 1170-1177); m_il is taken with the frame record (m_opt, above).
    reg       f1_r = 1'b0, f1_pre = 1'b0, m_il = 1'b0;
    reg [1:0] il_cnt = 2'd0;
    always @(posedge clk_video) begin
        f1_r <= vga_f1;
        if (ce_r) begin
            f1_pre <= f1_r;
            if (f1_r != f1_pre) il_cnt <= 2'd3;
            else if (vs_rise && il_cnt != 2'd0) il_cnt <= il_cnt - 2'd1;
            if (vs_rise && started && row != 12'd0 && !hdmi_freeze) m_il <= (il_cnt != 2'd0);
        end
    end
`endif

    //==================================================================== CE_PIXEL contract (simulation only)
    // The rule (tango_core_ports.vh, emu video ports): CE_PIXEL must keep pulsing through horizontal and vertical
    // blanking; VGA_HS / VGA_VS / VGA_DE are sampled only on CE.  ascal reads i_vs / i_de only at i_ce clocks
    // (ascal.vhd 1162-1166: i_vs_pre / i_de_pre), starts an input frame on that VS rise (1180) and ends a line on that
    // DE fall (1220, 1427); the writer above measures the frame the same way (123-124).  MiSTer's sys/video_mixer.sv
    // (Genesis_MiSTer adc0c42) changes VGA_VS / VGA_HS / VGA_DE only at CE_PIXEL clocks (188, 206-215).  nes-tango once
    // pulsed CE only inside the picture: ascal never started a frame and the silicon showed its no-input
    // picture (since fixed).
    // Checked per input frame, from one VGA_VS rise on CLK_VIDEO to the next (sampled every clock, CE or not):
    //   (a) CE_PIXEL pulses with VGA_DE low: at least one;
    //   (b) the VGA_VS rise is seen at a CE_PIXEL clock (VGA_VS high there, low at the CE clock before);
    //   (c) every VGA_DE fall is seen at a CE_PIXEL clock before the next VGA_DE fall: once per line.
    // A violation prints "TANGO CONTRACT: CE_PIXEL ..." and stops the simulation with $fatal (Verilator aborts, so the
    // sim gates' exit-code checks fail); the end of a run prints "TANGO CONTRACT OK: CE_PIXEL ..." with the count.
    // Simulation only: GowinSynthesis skips the code between the two pragma comments below (SUG550-2.0.1E section
    // 5.19, the Verilog block-comment form); Verilator has no such pragma and compiles the block (Verilator 5.020
    // FAQ, faq.html: use the preprocessor instead).  No netlist change.  (Keep the pragma words out of other comments.)
    /* synthesis translate_off */
    reg     cc_vs_q = 1'b0, cc_de_q = 1'b0;     // VGA_VS / VGA_DE at the previous CLK_VIDEO clock
    reg     cc_vs_s = 1'b0, cc_de_s = 1'b0;     // VGA_VS / VGA_DE at the last CE_PIXEL clock: what ascal sees
    reg     cc_pend = 1'b0;                     // a VGA_DE fall not yet seen at a CE_PIXEL clock
    reg     cc_arm  = 1'b0;                     // a VGA_VS rise has started a frame
    integer cc_frames = 0;                      // frames checked
    integer cc_ce = 0, cc_blank = 0, cc_vsr = 0, cc_lines = 0, cc_lsee = 0, cc_miss = 0;   // the frame so far
    integer cc_ok_blank = 0, cc_ok_ce = 0, cc_ok_lines = 0;                                 // the last good frame
    wire    cc_vs_rise = vga_vs && !cc_vs_q;                 // frame boundary (every clock)
    wire    cc_de_fall = !vga_de && cc_de_q;                 // a line ends (every clock)
    wire    cc_ce_vsr  = ce_pixel && vga_vs && !cc_vs_s;     // ... as seen at CE_PIXEL clocks
    wire    cc_ce_def  = ce_pixel && !vga_de && cc_de_s;
    wire    cc_ce_blk  = ce_pixel && !vga_de;
    always @(posedge clk_video) begin
        cc_vs_q <= vga_vs;
        cc_de_q <= vga_de;
        if (ce_pixel) begin cc_vs_s <= vga_vs; cc_de_s <= vga_de; end
        cc_pend <= cc_de_fall ? !cc_ce_def : (cc_ce_def ? 1'b0 : cc_pend);
        if (cc_vs_rise) begin
            if (cc_arm) begin
                if (cc_blank == 0 || cc_vsr == 0 || cc_miss != 0) begin
                    $display("TANGO CONTRACT: CE_PIXEL broken in input frame %0d (VGA_VS rise to VGA_VS rise on CLK_VIDEO, ending at %0t, %m):",
                             cc_frames + 1, $time);
                    $display("TANGO CONTRACT: CE_PIXEL   (a) pulses with VGA_DE low: %0d of %0d CE_PIXEL pulses  (need > 0)%s",
                             cc_blank, cc_ce, cc_blank == 0 ? "  <-- FAIL" : "");
                    $display("TANGO CONTRACT: CE_PIXEL   (b) VGA_VS rises seen at a CE_PIXEL clock: %0d  (need 1)%s",
                             cc_vsr, cc_vsr == 0 ? "  <-- FAIL" : "");
                    $display("TANGO CONTRACT: CE_PIXEL   (c) VGA_DE falls (lines): %0d, seen at a CE_PIXEL clock: %0d, missed: %0d  (need 0 missed)%s",
                             cc_lines, cc_lsee, cc_miss, cc_miss != 0 ? "  <-- FAIL" : "");
                    $display("TANGO CONTRACT: CE_PIXEL must keep pulsing through horizontal and vertical blanking; VGA_HS / VGA_VS / VGA_DE are sampled only on CE (sys/tango_core_ports.vh)");
                    $fatal(1, "TANGO CONTRACT: CE_PIXEL violated (see the lines above)");
                end
                cc_frames   <= cc_frames + 1;
                cc_ok_blank <= cc_blank;
                cc_ok_ce    <= cc_ce;
                cc_ok_lines <= cc_lines;
            end
            cc_arm   <= 1'b1;
            cc_ce    <= ce_pixel  ? 1 : 0;
            cc_blank <= cc_ce_blk ? 1 : 0;
            cc_vsr   <= cc_ce_vsr ? 1 : 0;
            cc_lines <= cc_de_fall ? 1 : 0;
            cc_lsee  <= cc_ce_def ? 1 : 0;
            cc_miss  <= (cc_de_fall && cc_pend) ? 1 : 0;
        end else begin
            cc_ce    <= cc_ce    + (ce_pixel  ? 1 : 0);
            cc_blank <= cc_blank + (cc_ce_blk ? 1 : 0);
            cc_vsr   <= cc_vsr   + (cc_ce_vsr ? 1 : 0);
            cc_lines <= cc_lines + (cc_de_fall ? 1 : 0);
            cc_lsee  <= cc_lsee  + (cc_ce_def ? 1 : 0);
            cc_miss  <= cc_miss  + ((cc_de_fall && cc_pend) ? 1 : 0);
        end
    end
    final
        $display("TANGO CONTRACT OK: CE_PIXEL held in %0d input frames (%m; last: %0d CE_PIXEL pulses, %0d with VGA_DE low, %0d lines)",
                 cc_frames, cc_ok_ce, cc_ok_blank, cc_ok_lines);
    /* synthesis translate_on */
`ifdef TANGO_VIDEO_OPTS
    //==================================================================== Video options rule (simulation only)
    // With TANGO_VIDEO_OPTS sys reads Scale and Scanlines from its own status bits and does not use VIDEO_SCALE or
    // VGA_SL (tango_core_ports.vh).  A core that still drives either has per-core Video wiring left over: the kind
    // of wiring that once read the wrong status bits.  Checked for every input frame the writer above takes
    // (the values it latches into m_opt at a VS rise): both must be 0, else "TANGO CONTRACT: VIDEO_OPTS ..." and
    // $fatal, as the CE_PIXEL check.  No netlist change (the same pragma pair as the CE_PIXEL check).
    /* synthesis translate_off */
    integer vo_frames = 0;
    always @(posedge clk_video)
        if (vs_rise && started && !hdmi_freeze) begin
            if (opt_s1[47:46] != 2'd0 || opt_s1[19:17] != 3'd0) begin
                $display("TANGO CONTRACT: VIDEO_OPTS: the core drives VGA_SL = %0d and VIDEO_SCALE = %0d (%m, %0t); with TANGO_VIDEO_OPTS sys owns Scanlines and Scale (sys/tango_video_opts.vh) and does not use them: tie both to 0",
                         opt_s1[47:46], opt_s1[19:17], $time);
                $fatal(1, "TANGO CONTRACT: VIDEO_OPTS violated (see the line above)");
            end
            vo_frames <= vo_frames + 1;
        end
    final
        $display("TANGO CONTRACT OK: VIDEO_OPTS: VGA_SL = 0 and VIDEO_SCALE = 0 in %0d input frames (%m)", vo_frames);
    /* synthesis translate_on */
`endif

    //==================================================================== geometry (clk_pix)
    // Two-flop sync of fd_tog; capture metadata when toggle changes.
    reg [2:0]  fd_s    = 3'b000;
    reg        fd_pend = 1'b0;
    reg [11:0] p_hs    = 12'd0, p_vt = 12'd0;
    reg [OPTW-1:0] p_opt = {OPTW{1'b0}};
    reg [11:0] l_hs    = 12'd0, l_vt = 12'd0;
    reg [OPTW-1:0] l_opt = {OPTW{1'b0}};
    reg        have    = 1'b0;
`ifdef TANGO_VIDEO_OPTS
    // the Video options (tango_video_opts.vh) in place of the core's fields of l_opt, taken with it (below)
    reg [12:0] vo_arx  = 13'd0, vo_ary = 13'd0;
    reg [2:0]  vo_sc   = 3'd0;
    reg [1:0]  vo_sl   = 2'd0;
    wire [1:0] vo_ar_s = status[`TANGO_VO_B_ASPECT];
`ifdef TANGO_VIDEO_ROTATE
`include "tango_video_scale_arc.vh"   // scale_arc(): arcade Scale is Normal / V-Integer only (tango_video_opts.vh)
`endif
`endif
    // ---- rotation: while ascal shows the rotated framebuffer, the geometry engine gets
    // the framebuffer's size and the core's aspect swapped (tango_video_rot_geo.vh). rot_fb_* are clk_pix already
    // (tango_video_rotate_ddr3's own CDC) and rot_fb_ena is the same signal that switches ascal to the framebuffer,
    // so the window and the source switch together (taken at the frame hand-over, as everything below is).
`ifdef TANGO_VIDEO_ROTATE
`ifdef TANGO_VIDEO_DDR3
`include "tango_video_rot_geo.vh"
    wire [71:0] p_geo_in = rot_geo_in(rot_fb_ena, p_hs, p_vt, rot_fb_hsize, rot_fb_vsize, p_opt);
`else
    wire [71:0] p_geo_in = {p_hs, p_vt, p_opt};
`endif
`else
    wire [71:0] p_geo_in = {p_hs, p_vt, p_opt};
`endif

    always @(posedge clk_pix) begin
        fd_s <= {fd_s[1:0], fd_tog};
        if (fd_s[2] != fd_s[1]) begin
            fd_pend <= 1'b1; p_hs <= m_hs; p_vt <= m_vt; p_opt <= m_opt;
        end
        if (rst_pix) begin
            have <= 1'b0; fd_pend <= 1'b0;
        end else if (fd_pend) begin
            fd_pend <= 1'b0;
            l_hs <= p_geo_in[71:60]; l_vt <= p_geo_in[59:48]; l_opt <= p_geo_in[47:0];   // = p_hs/p_vt/p_opt unless rotated
`ifdef TANGO_VIDEO_OPTS
            vo_arx <= (vo_ar_s == 2'd0) ? p_geo_in[45:33] : (vo_ar_s == 2'd2) ? 13'd4 : (vo_ar_s == 2'd3) ? 13'd16 : 13'd0;
            vo_ary <= (vo_ar_s == 2'd0) ? p_geo_in[32:20] : (vo_ar_s == 2'd2) ? 13'd3 : (vo_ar_s == 2'd3) ? 13'd9  : 13'd0;
`ifdef TANGO_VIDEO_ROTATE
            vo_sc  <= {1'b0, scale_arc(status[`TANGO_VO_B_SCALE])};   // arcade: 2/3 (stale CFG) = Normal (tango_video_scale_arc.vh)
`else
            vo_sc  <= {1'b0, status[`TANGO_VO_B_SCALE]};
`endif
            vo_sl  <= status[`TANGO_VO_B_SL];
`endif
            have <= 1'b1;
        end
    end

`ifdef TANGO_VIDEO_F1
    // the interlace flag with the frame record (as p_opt / l_opt)
    reg p_il = 1'b0, l_il = 1'b0;
    always @(posedge clk_pix) begin
        if (fd_s[2] != fd_s[1]) p_il <= m_il;
        if (!rst_pix && fd_pend) l_il <= p_il;
    end
`endif

`ifdef TANGO_VIDEO_OPTS
    // ---- the Video options (above): status is a register of clk_pix (tango_link, written whole when a SET_STATUS
    // transfer ends), so it needs no synchroniser here.  vo_* are taken at the frame hand-over, with the core's own
    // fields (l_opt), so a change applies at an input frame boundary as a core's own option does (a frame sooner only
    // when it lands within the few clocks of that path's synchronisers).  Registered, so the geometry tag compare
    // and tango_video_geo's inputs see flops, as they see l_opt.
    //   Aspect ratio: 0 Original = the core's VIDEO_ARX/ARY; 1 Full Screen = 0:0 (Template_MiSTer Template.sv 54-55:
    //                 ar 1 -> 0:0; tango_video_geo P_SYS: the whole screen); 2 = 4:3; 3 = 16:9
    //   Scale:        video_freak's SCALE 0-3 (tango_video_geo.v); an arcade build (TANGO_VIDEO_ROTATE) offers 0/1 only
    //                 and decodes a stale 2/3 as 0, as the BL616 menu shows it (tango_video_scale_arc.vh)
    //   Scanlines:    the VGA_SL level 0-3 (dim() below)
`endif

    // geo engine to compute ascal output window
`ifdef TANGO_VIDEO_OPTS
    wire [69:0] l_geo = {l_hs, l_vt, vo_arx, vo_ary, vo_sc, l_opt[16:0]};
`else
    wire [69:0] l_geo = {l_hs, l_vt, l_opt[45:0]};
`endif
    reg  [69:0] geo_tag  = 70'd0;
    reg         geo_go   = 1'b0, geo_ok = 1'b0, geo_have = 1'b0;
    wire        g_busy, g_valid;
    wire [11:0] g_x0, g_x1, g_voff, g_ih_crop, g_hq0, g_hq;
    wire [10:0] g_y0, g_y1;
    wire [12:0] g_ha0, g_hr, g_vq0, g_vq, g_va0, g_vr;
    wire signed [14:0] g_hdn, g_vdn;
    wire [12:0] g_fx, g_fy;
    wire [11:0] g_ow, g_oh;
    wire [11:0] g_hmin, g_hmax;
    wire [10:0] g_vmin, g_vmax;
    tango_video_geo #(.OW(OW), .OH(OH)) u_geo (
        .clk(clk_pix), .go(geo_go),
`ifdef TANGO_VIDEO_OPTS
        .hs(l_hs), .vt(l_vt), .arx(vo_arx), .ary(vo_ary), .scale(vo_sc),
`else
        .hs(l_hs), .vt(l_vt), .arx(l_opt[45:33]), .ary(l_opt[32:20]), .scale(l_opt[19:17]),
`endif
        .crop_size(l_opt[16:5]), .crop_off(l_opt[4:0]),
        .busy(g_busy), .valid(g_valid), .x0(g_x0), .x1(g_x1), .y0(g_y0), .y1(g_y1),
        .voff(g_voff), .ih_crop(g_ih_crop),
        .hq0(g_hq0), .hq(g_hq), .ha0(g_ha0), .hr(g_hr), .hdn(g_hdn),
        .vq0(g_vq0), .vq(g_vq), .va0(g_va0), .vr(g_vr), .vdn(g_vdn),
        .fx(g_fx), .fy(g_fy), .ow_o(g_ow), .oh_o(g_oh),
        .hmin(g_hmin), .hmax(g_hmax), .vmin(g_vmin), .vmax(g_vmax));

    always @(posedge clk_pix) begin
        geo_go <= 1'b0;
        if (rst_pix) begin
            geo_ok <= 1'b0; geo_have <= 1'b0;
        end else if (have && !geo_ok && !geo_go && !g_busy) begin
            if (geo_have && l_geo == geo_tag) begin
                geo_ok <= 1'b1;
            end else begin geo_go <= 1'b1; geo_tag <= l_geo; geo_have <= 1'b1; end
        end else if (have && !geo_ok && geo_have && !g_busy) begin
            geo_ok <= 1'b1;
        end else if (geo_ok && l_geo != geo_tag) begin
            geo_ok <= 1'b0;   // opts changed: invalidate so engine re-fires
        end
    end

    // live geometry: updated at each new frame when geo is ready
    // (ascal resamples quasi-statically; we update when fd_pend fires or geo_ok transitions)
    reg [11:0] v_hmin = 12'd0, v_hmax = 12'd640;
    reg [10:0] v_vmin = 11'd0, v_vmax = 11'd359;
    reg [1:0]  v_sl   = 2'd0;
    reg [11:0] v_ih   = 12'd0, v_voff = 12'd0;

    always @(posedge clk_pix) begin
        if (geo_ok && !g_busy) begin
            v_hmin <= g_hmin; v_hmax <= g_hmax;
            v_vmin <= g_vmin; v_vmax <= g_vmax;
`ifdef TANGO_VIDEO_OPTS
`ifdef TANGO_VIDEO_F1
            v_sl   <= l_il ? 2'd0 : vo_sl;     // TANGO_VIDEO_F1: no scanlines on interlaced video (header)
`else
            v_sl   <= vo_sl;
`endif
`else
            v_sl   <= l_opt[47:46];
`endif
            v_ih   <= g_ih_crop;
            v_voff <= g_voff;
        end
    end

    //==================================================================== ascal_wrap / ascal_wrap_sim
    wire [7:0] a_or, a_og, a_ob;
    wire       a_ohs, a_ovs, a_ode;
`ifdef TANGO_VSYNC_ADJUST
    // ---- TANGO_VSYNC_ADJUST: the raster's totals at runtime.  This module's beam leads
    // hdmi.sv's by tango_sys's lead, so it takes the next totals at ITS OWN frame end (cw/ch: the frame its beam is
    // in; the lead itself works from hdmi.sv's current totals).  ascal's output sweep runs ~13 clocks AHEAD of this beam
    // (o_vs = o_vsv(11) + one register: ascal.vhd 2677-2684, 2852-2854), and it re-reads htotal / vtotal every clock
    // (ascal.vhd 1838-1845): its line ends at o_hcpt + 1 >= o_htotal (2650) and its frame where o_vcpt_pre3, three lines
    // ahead of the displayed line, reaches o_vtotal (2659).  So the new htotal must arrive inside line 0 of ascal's new
    // frame and vtotal between its two pre3 wraps: both are taken when this beam is at (64, 0) of the new frame, ascal
    // then being near x 77 of the same line.  vs_at (the target of the alignment below) is the NEXT frame's, FRAME the
    // current one's.  The sim-only rule below checks that ascal's VS stays on target through every change.
    reg  [11:0] cw = FW, cw_m1 = FW - 1;
    reg  [10:0] ch = FH, ch_m1 = FH - 1;
    reg  [11:0] a_ht = FW, a_vt = FH;
    reg  [20:0] frame_q = FW * FH, vs_at_q = 21'd0;
    always @(posedge clk_pix) begin
        if (rst_pix || (cx == cw_m1 && cy == ch_m1)) begin
            cw <= vt_nxt_w;  cw_m1 <= vt_nxt_w - 12'd1;  ch <= vt_nxt_h;  ch_m1 <= vt_nxt_h - 11'd1;
        end
        if (rst_pix) begin a_ht <= vt_nxt_w; a_vt <= {1'b0, vt_nxt_h}; end
        else if (cx == 12'd64 && cy == 11'd0) begin a_ht <= cw; a_vt <= {1'b0, ch}; end
        frame_q <= cw * ch;
        vs_at_q <= o_vsstart * vt_nxt_w + o_hsstart;
    end
`endif

`ifndef TANGO_SIM
    // ---- ascal's output sweep onto this module's beam.  ascal.vhd's OSWEEP counters have no reset: they
    // count from configuration, so ascal's frame lands at an arbitrary offset from hdmi.sv's beam (on silicon,
    // e.g. 613 pixels, 45 lines, wrapped).  rgb at clock t is for the beam (cx, cy) of t-VLAT and is
    // made from ascal's output of t-2, so ascal's output must be for the beam of the same clock: its vsync
    // rises at (o_hsstart, o_vsstart).  A rise anywhere else holds o_ce low for the difference (at most one
    // frame, once); every later rise is on target.
    // to_vs = (vs_at - the linear position of (cx, cy)) mod FRAME: the clocks until the beam reaches the target, kept
    // in step with the beam (vs_at at its wrap, one down every clock), so the alignment loads it as it is.  (It used
    // to compare and subtract the 21-bit position in the loading clock: 24 LUT levels at 74.25 MHz in the A2600
    // build, -1.3 ns.)  The value loaded is the one before: (position < vs_at) ? vs_at - position : FRAME - position
    // + vs_at, and to_vs != 0 is position != vs_at.
`ifdef TANGO_VSYNC_ADJUST
    wire [20:0] FRAME = frame_q;               // the current frame's (above)
    wire [20:0] vs_at = vs_at_q;               // the next frame's target, loaded at this beam's frame end
`else
    localparam [20:0] FRAME = FW * FH;
    wire [20:0] vs_at = o_vsstart * FW + o_hsstart;
`endif
    reg  [20:0] to_vs = 21'd0;
    reg         bsync = 1'b0;                 // to_vs is valid: the beam has wrapped since reset
    reg  [20:0] ce_hold = 21'd0;
    reg         ce_ascal = 1'b1, a_ovs_q = 1'b0;
    always @(posedge clk_pix) begin
        a_ovs_q <= a_ovs;
`ifdef TANGO_VSYNC_ADJUST
        if (cx == cw_m1 && cy == ch_m1) begin to_vs <= vs_at;  bsync <= 1'b1; end
`else
        if (cx == FW - 1 && cy == FH - 1) begin to_vs <= vs_at;  bsync <= 1'b1; end
`endif
        else to_vs <= (to_vs == 21'd0) ? FRAME - 21'd1 : to_vs - 21'd1;
        if (rst_pix) begin
            bsync <= 1'b0;  ce_hold <= 21'd0;  ce_ascal <= 1'b1;
        end else if (ce_hold != 21'd0) begin
            ce_hold  <= ce_hold - 21'd1;
            ce_ascal <= (ce_hold == 21'd1);
        end else if (bsync && a_ovs && !a_ovs_q && to_vs != 21'd0) begin
            ce_hold  <= to_vs;
            ce_ascal <= 1'b0;
        end
    end
    // ---- the alignment rule (simulation only; the pragma pair of the CE_PIXEL check above, no netlist change).  The beam
    // this module sees (FW x FH) and ascal's output raster (o_htotal x o_vtotal) must be the same raster: then ascal is
    // aligned ONCE after reset (one ce_hold load, above) and every later output VS rise is on target (to_vs == 0), one
    // FRAME = FW x FH clocks after the one before.  A second load, an off-target rise or another period means two
    // consumers of the raster disagree: printed as "TANGO VIDEO: ..."; the end of a run
    // prints the counts ("TANGO VIDEO OK: ..." only with one load and every period FRAME).
    /* synthesis translate_off */
    integer al_loads = 0, al_rises = 0, al_on = 0, al_per_bad = 0, al_changes = 0;
    reg     al_have = 1'b0;
    reg [31:0] al_clk = 32'd0, al_t = 32'd0;
    // the expected period between two on-target rises: the earlier frame's total - its VS position + this frame's
    // (= FRAME while the totals do not change; TANGO_VSYNC_ADJUST: the frame of each rise is cw x ch)
`ifdef TANGO_VSYNC_ADJUST
    wire [31:0] al_vpos = o_vsstart * cw + o_hsstart, al_ftot = cw * ch;
`else
    wire [31:0] al_vpos = o_vsstart * FW + o_hsstart, al_ftot = FRAME;
`endif
    reg  [31:0] al_vprev = 32'd0, al_fprev = 32'd0;
    always @(posedge clk_pix) begin
        al_clk <= al_clk + 32'd1;
        if (!rst_pix && ce_hold == 21'd0 && bsync && a_ovs && !a_ovs_q) begin
            al_rises <= al_rises + 1;
            if (to_vs != 21'd0) begin
                al_loads <= al_loads + 1;
                al_have  <= 1'b0;
                if (al_loads >= 1)
                    $display("TANGO VIDEO: ascal's output VS %0d clocks off target after the first alignment (load %0d, %m, %0t): the raster's consumers disagree (FW x FH = %0d x %0d, o_htotal x o_vtotal = %0d x %0d)",
                             to_vs, al_loads + 1, $time, FW, FH, o_htotal, o_vtotal);
            end else begin
                al_on <= al_on + 1;
                if (al_have && al_clk - al_t != al_fprev - al_vprev + al_vpos) begin
                    al_per_bad <= al_per_bad + 1;
                    $display("TANGO VIDEO: ascal's output VS period %0d clocks, want %0d (%m, %0t)", al_clk - al_t,
                             al_fprev - al_vprev + al_vpos, $time);
                end
                if (al_have && al_fprev != al_ftot) al_changes <= al_changes + 1;
                al_have  <= 1'b1;
                al_t     <= al_clk;
                al_fprev <= al_ftot;
                al_vprev <= al_vpos;
            end
        end
    end
    final
        $display("TANGO VIDEO%0s: ascal output alignment: %0d load(s) after reset, %0d VS rises, %0d on target, %0d periods != the raster's, %0d raster changes, last raster %0d clk_pix (%m)",
                 (al_loads == 1 && al_per_bad == 0 && al_on > 0) ? " OK" : "", al_loads, al_rises, al_on, al_per_bad,
                 al_changes, al_fprev);
    /* synthesis translate_on */
`else
    wire ce_ascal = 1'b1;                     // ascal_wrap_sim runs its own beam
`endif

    // Common port connections used by both real and sim instantiations.
    // Under TANGO_SIM: ascal_wrap_sim gets an extra parameter block (.SIMW/.SIMH)
    // and exposes four sim-only debug ports wired back to the module's debug outputs.

`ifdef TANGO_SIM
    // Intermediate wires for sim debug ports (output wires can't be driven directly
    // from a port connection in all Verilog tools).
    wire [15:0] _sim_frames, _sim_seq, _sim_rseq;
    wire        _sim_valid, _sim_take, _sim_dl_parity, _sim_frame_end;
    assign dbg_frames = _sim_frames;
    assign show_seq   = _sim_seq;    // live CDC (rs_s1): correct for T-log staleness check
    assign rseq       = _sim_rseq;  // frame-locked (rseq_r): correct for F-log f_seq (matches rbank)
    assign show_valid = _sim_valid && geo_ok;  // only valid once geometry window settled
    assign show_take  = _sim_take;
    assign dbg_lost   = 16'd0;

    ascal_wrap_sim #(
        .SIMW    (MAXW),
        .SIMH    (MAXH),
        .RAMBASE (ASCAL_RAMBASE),
        .RAMSIZE (ASCAL_RAMSIZE)
    ) u_ascal (
`else
    ascal_wrap #(
        .RAMBASE (ASCAL_RAMBASE),
        .RAMSIZE (ASCAL_RAMSIZE)
    ) u_ascal (
`endif
`ifndef TANGO_SIM
`ifdef TANGO_VIDEO_ROTATE
`ifdef TANGO_VIDEO_DDR3
        .fb_ena    (rot_fb_ena),
        .fb_hsize  (rot_fb_hsize),
        .fb_vsize  (rot_fb_vsize),
        .fb_format (rot_fb_format),
        .fb_base   (rot_fb_base),
        .fb_stride (rot_fb_stride),
        .fb_vbl    (rot_fb_vbl),
`else
        // ascal_wrap's fb_* ports exist unconditionally; every build
        // without TANGO_VIDEO_ROTATE+TANGO_VIDEO_DDR3 must still tie fb_ena=0 explicitly -- an unconnected
        // Verilog input is undefined ('z'), not a safe implicit 0, and an option a core does not use must
        // change nothing for that core.
        .fb_ena    (1'b0), .fb_hsize(12'd0), .fb_vsize(12'd0), .fb_format(6'b000100), .fb_base(32'd0),
        .fb_stride (14'd0), .fb_vbl(),
`endif
`else
        .fb_ena    (1'b0), .fb_hsize(12'd0), .fb_vsize(12'd0), .fb_format(6'b000100), .fb_base(32'd0),
        .fb_stride (14'd0), .fb_vbl(),
`endif
`endif
        // input video (clk_video domain)
        .i_clk  (clk_video),
        .i_ce   (ce_pixel),
        .i_r    (v_r),
        .i_g    (v_g),
        .i_b    (v_b),
        .i_hs   (v_hs),
        .i_vs   (v_vs),
        .i_de   (v_de),
`ifdef TANGO_VIDEO_F1
        .i_fl   (vga_f1),                     // TANGO_VIDEO_F1 (header)
`else
        .i_fl   (1'b0),
`endif
        // output video (clk_pix domain)
        .o_clk  (clk_pix),
        .o_ce   (ce_ascal),
        .o_r    (a_or),
        .o_g    (a_og),
        .o_b    (a_ob),
        .o_hs   (a_ohs),
        .o_vs   (a_ovs),
        .o_de   (a_ode),
        // geometry window
        .hmin   (v_hmin),
        .hmax   (v_hmax),
        .vmin   (v_vmin),
        .vmax   (v_vmax),
        // output timing (720p60; TANGO_VSYNC_ADJUST: the totals at runtime, above)
`ifdef TANGO_VSYNC_ADJUST
        .htotal  (a_ht),
`else
        .htotal  (o_htotal),
`endif
        .hsstart (o_hsstart),
        .hsend   (o_hsend),
        .hdisp   (o_hdisp),
`ifdef TANGO_VSYNC_ADJUST
        .vtotal  (a_vt),
`else
        .vtotal  (o_vtotal),
`endif
        .vsstart (o_vsstart),
        .vsend   (o_vsend),
        .vdisp   (o_vdisp),
        // scaler mode (TANGO_VIDEO_PROC: [2:0] = 100 polyphase while flt_mode[0], else the caller's)
`ifdef TANGO_VIDEO_PROC
        .mode   ({video_mode[4:3], flt_mode[0], video_mode[1:0]}),
`else
        .mode   (video_mode),
`endif
        // Avalon (avl_clk = 96 MHz SDRAM clock)
        .avl_clk          (avl_clk),
        .avl_address      (avl_address),
        .avl_write        (avl_write),
        .avl_writedata    (avl_writedata),
        .avl_read         (avl_read),
        .avl_burstcount   (avl_burstcount),
        .avl_byteenable   (avl_byteenable),
        .avl_readdata     (avl_readdata),
        .avl_readdatavalid(avl_readdatavalid),
        .avl_waitrequest  (avl_waitrequest),
`ifdef TANGO_VIDEO_PROC
`ifndef TANGO_SIM
        // polyphase coefficients (tango_vproc_regs; ascal_wrap_sim has no polyphase filter)
        .poly_wr  (poly_wr), .poly_a (poly_a), .poly_dw (poly_dw),
`endif
`endif
        // reset (active-low async)
        .reset_na (~rst_pix),
        // debug outputs: Avalon handshake monitoring
        .dbg_read_count       (dbg_arc),
        .dbg_readdata_valid_count (dbg_arv),
        .dbg_write_count      (dbg_awc),
        .dbg_last_address     (),
        .dbg_last_readdata    (dbg_ald),
        .dbg_waitrequest_stuck(dbg_aws),
        .dbg_avl_read         (),
        .dbg_avl_readdatavalid(),
        .dbg_avl_waitrequest  ()
`ifdef TANGO_SIM
        // sim-only ports
        , .i_freeze       (hdmi_freeze)
        , .sim_dbg_frames  (_sim_frames)
        , .sim_show_seq    (_sim_seq)
        , .sim_show_rseq   (_sim_rseq)
        , .sim_show_valid  (_sim_valid)
        , .sim_show_take   (_sim_take)
        , .sim_dl_parity   (_sim_dl_parity)
        , .sim_frame_end   (_sim_frame_end)
        , .sim_cx         (cx)
        , .sim_cy         (cy)
        , .i_voff         (v_voff)
        , .i_ih           (v_ih)
`endif
    );

    //==================================================================== output register + scanlines
    // ascal outputs are already registered internally; add one more stage here (VLAT = 2 total).
    // Scanlines: dim() matching MiSTer scanlines.v 37-53.
    function [7:0] dim(input [7:0] v, input [1:0] s);
        case (s)
            2'd1:    dim = {1'b0, v[7:1]} + {2'b00, v[7:2]};  // 75 %
            2'd2:    dim = {1'b0, v[7:1]};                     // 50 %
            2'd3:    dim = {2'b00, v[7:2]};                    // 25 %
            default: dim = v;
        endcase
    endfunction

    // BPC expansion: keep top BPC bits of v, replicate them to fill 8 bits.
    // Matches MiSTer's expand() used in video_mixer / sys_top output path.
    function automatic [7:0] bpc_expand;
        input [7:0] v;
        reg [7:0] vm, o;
        integer have;
        begin
            vm   = (v >> (8 - BPC)) << (8 - BPC);
            o    = 8'd0;
            have = 0;
            while (have < 8) begin
                o    = o | (vm >> have);
                have = have + BPC;
            end
            bpc_expand = o;
        end
    endfunction

    reg [23:0] pix_r = 24'd0;
    reg        de_out = 1'b0;
    // ---- the picture gate, the same in the board build and under TANGO_SIM: black until the core's first frame has
    // been measured and its geometry computed (geo_ok), opened at an output VS rise.  ascal draws its output window
    // from the moment it runs, frame or no frame: a core that sends no picture (e.g. the NES before a ROM is loaded)
    // showed ascal's unwritten buffer in the reset window above (v_hmin 0..640, v_vmin 0..359) on silicon, a flat
    // rectangle at the top left of a black screen, while every sim was black (this gate existed only under TANGO_SIM,
    // as geo_ok_frame).  The VS rise is in the vertical blanking, and ascal changes its output buffer at the VS fall
    // that follows (ascal.vhd 1881-1893) to the one its input finished at the input VS that also let the writer above
    // measure the frame (1207-1208, 1866-1871): the first picture is a whole frame of the core's, no torn frame.  Once
    // open the gate stays open until rst_pix (a later geometry change is ascal's, as before; geo_ok_frame took geo_ok
    // again at every VS rise, which could black one frame on a change).  The OSD (tango_menu_overlay in tango_sys.v)
    // draws over rgb, so the in-core menu shows over the black (README "Black until the core's first frame").
    reg a_ovs_r = 1'b0, show_ok = 1'b0;
    always @(posedge clk_pix) begin
        a_ovs_r <= a_ovs;
        if (rst_pix)                         show_ok <= 1'b0;
        else if (a_ovs && !a_ovs_r && geo_ok) show_ok <= 1'b1;
    end
`ifdef TANGO_SIM
    reg [1:0] v_sl_frame = 2'd0;   // v_sl latched at Fix Q (last pixel) so geo settles before scan
    always @(posedge clk_pix)
        if (_sim_frame_end)
            v_sl_frame <= v_sl;
    // dl_par_r: registered parity of the scaler's doubled-line index.  Aligned with pix_r
    // (both latch at the same stage), then used for scanline dimming one stage later.
    reg dl_par_r = 1'b0;
    always @(posedge clk_pix) begin
        de_out   <= a_ode && show_ok;
        pix_r    <= {bpc_expand(a_or), bpc_expand(a_og), bpc_expand(a_ob)};
        dl_par_r <= _sim_dl_parity;
    end
`else
    always @(posedge clk_pix) begin
        de_out <= a_ode && show_ok;
        pix_r  <= {bpc_expand(a_or), bpc_expand(a_og), bpc_expand(a_ob)};
    end
`endif
    // sl_parity: which output rows get scanline dimming.
    // In sim: use scaler's dl[0] (matches golden's (dl & 1) criterion).
    // In hardware: use output beam row cy[0] (simpler, matches ascal's own pipeline).
`ifdef TANGO_SIM
    wire sl_parity = dl_par_r;
`else
    wire sl_parity = cy[0];
`endif
    // rgb: scanline dim on alternating lines, black outside active area.
    // In sim: use v_sl_frame (latched at VS) so opts changes mid-frame don't shift the dimming pattern.
    // In hardware: use live v_sl (geo settles long before display; the real ascal has its own pipeline).
`ifdef TANGO_SIM
    wire [1:0] sl_use = v_sl_frame;
`else
    wire [1:0] sl_use = v_sl;
`endif
`ifdef TANGO_VIDEO_PROC
    // the Scan filter draws the scanlines (header): no dimming while it is the V set and polyphase is on
    wire [1:0] sl_eff = (flt_mode == 2'b11) ? 2'd0 : sl_use;
    assign sl_level = v_sl;
    reg  [23:0] rgb_d = 24'd0;                 // the dimmed pixel, into the shadow mask
    always @(posedge clk_pix) begin
        if (!de_out) rgb_d <= 24'd0;
        else if (sl_eff != 2'd0 && sl_parity)
            rgb_d <= {dim(pix_r[23:16], sl_eff), dim(pix_r[15:8], sl_eff), dim(pix_r[7:0], sl_eff)};
        else
            rgb_d <= pix_r;
    end
    // the shadow mask: rgb_d at t is for the beam (cx, cy) of t-2 (header "VLAT")
    reg  [11:0] mcx1 = 12'd0, mcx2 = 12'd0;  reg [10:0] mcy1 = 11'd0, mcy2 = 11'd0;
    always @(posedge clk_pix) begin mcx1 <= cx; mcx2 <= mcx1; mcy1 <= cy; mcy2 <= mcy1; end
    wire [23:0] rgb_m;
    tango_shmask #(.FW(FW)) u_mask (
        .clk(clk_pix), .cmd_wr(mask_wr), .cmd(mask_cmd), .cx(mcx2), .cy(mcy2),
        .hmin(v_hmin), .vmin(v_vmin), .din(rgb_d), .dout(rgb_m));
    always @(posedge clk_pix) rgb <= rgb_m;
`else
    always @(posedge clk_pix) begin
        if (!de_out) rgb <= 24'd0;
        else if (sl_use != 2'd0 && sl_parity)
            rgb <= {dim(pix_r[23:16], sl_use), dim(pix_r[15:8], sl_use), dim(pix_r[7:0], sl_use)};
        else
            rgb <= pix_r;
    end
`endif

endmodule
`default_nettype wire
