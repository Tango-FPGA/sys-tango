// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_hdmi_totals -- hdmi.sv alone at a build's raster totals (tango_cfg.vh
// TANGO_HDMI_HTOTAL / _VTOTAL / _VIC).  No other sim instantiates hdmi.sv (every tango_sys bench makes its own beam),
// so this is the one place the HDMI raster itself is checked.  clk_pix 74.25 MHz, clk_audio as the board top
// (+16 mod 12375 on clk_pix), reset for 16 clocks, then TB_NF frames.  Checked at EVERY clock after reset, from cx / cy
// two clocks earlier (cx2 / cy2: hdmi.sv registers its windows from cx, then mode / control_data from those):
//   mode 1 (video data)        <=> cx2 < 1280 and cy2 < 720
//   mode 2 (video guard)       <=> cx2 in {FW-2, FW-1} on a line before a picture line (cy2 = FH-1 or cy2 < 719)
//   mode 3/4 (data island)     <=> cx2 in [1292, 1616)        (every line: 10 packets)
//   video preamble (CTL0 only) <=> cx2 in [FW-10, FW-2) on such a line
//   island preamble (CTL0+2)   <=> cx2 in [1284, 1292)
// i.e. the picture, the islands and the guards stay where they were and the extra clocks are back porch, the video
// preamble never touches an island (FW-10 >= 1616).  Per frame: HSYNC rises every FW clocks at cx 1390; VSYNC rises
// every FW x FH clocks, at (1390, 724), and falls at (1390, 729); 1280 x 720 video-data clocks; at least one AVI
// InfoFrame sent, its PB4 = the VIC.  TB_ORIG=1 also runs the ORIGINAL hdmi.sv (sim/obj_*/hdmi_orig.sv, made by the run
// script from git) beside it and requires the two TMDS streams to be identical at every clock (the defaults only).
// TANGO_VSYNC_ADJUST (hdmi.sv's runtime totals): next_frame_width / _height change mid-frame every frame along
// a schedule (1650x750, 1657x749, 1664x743, 1720x790, 1626x740, 1657x749, ...); each frame must run the totals present
// at its start (the latch is checked at every wrap) and every check above uses the CURRENT frame's totals (frame_width /
// frame_height, two clocks late for the windows): HSYNC every W of the line it rose on; VSYNC W_k x (V_k - 724) +
// 724 x W_k+1 after the one before.  Prints "HDMI_TOTALS PASS/FAIL" with the counts.
// TANGO_HDMI_PLL_<f> (the build-time HDMI PLL presets; run_tb_hdmi_totals.sh PLL=<f>): clk_pix at the preset's EXACT
// frequency (tango_cfg.vh TANGO_HDMI_FPIX: half periods of 5e11 / f ps, the fraction by a Bresenham accumulator) and
// clk_audio from the preset's accumulator, as the board top; also checked: every ACR packet carries the preset's N / CTS
// (tango_cfg.vh TANGO_HDMI_ACR_N / _CTS), f_pix x N == 128 x 48000 x CTS exactly (the sink regenerates exactly 48 kHz), and
// the clk_audio rate measured over the run is 48 kHz within the run's resolution (+-1 sample).  The defaults also print
// the ACR pair (6144 / 74250).
//==============================================================================
`default_nettype none
`timescale 1ps/1ps
`include "tango_cfg.vh"
module tb_hdmi_totals;
`ifndef TB_FW
`define TB_FW 1650
`endif
`ifndef TB_FH
`define TB_FH 750
`endif
`ifndef TB_VIC
`define TB_VIC 4
`endif
`ifndef TB_NF
`define TB_NF 4
`endif
    localparam integer FW = `TB_FW, FH = `TB_FH, NF = `TB_NF;
    reg clk_pix = 1'b0;
`ifdef TANGO_HDMI_PLL_SET
    // the preset's exact clk_pix: half period 5e11 / f ps = HPI + HPR / f
    localparam longint FPIX = `TANGO_HDMI_FPIX, HPI = 64'd500000000000 / FPIX, HPR = 64'd500000000000 % FPIX;
    longint hacc = 0;
    always begin
        hacc = hacc + HPR;
        if (hacc >= FPIX) begin hacc = hacc - FPIX; #(HPI + 1); end else #(HPI);
        clk_pix = ~clk_pix;
    end
`else
    localparam longint FPIX = 74250000;
    always #6734 clk_pix = ~clk_pix;
`endif
    reg [4:0] rcnt = 5'd0;
    wire rst = (rcnt != 5'd16);
    always @(posedge clk_pix) if (rst) rcnt <= rcnt + 5'd1;
    // clk_audio as tango_sys_top138.v (tango_cfg.vh TANGO_HDMI_AUD_*: +16 mod 12375 at 74.25 MHz)
    localparam integer AW = `TANGO_HDMI_AUD_W;
    localparam [AW-1:0] AINC = `TANGO_HDMI_AUD_INC, AMOD = `TANGO_HDMI_AUD_MOD;
    reg [AW-1:0] aacc = {AW{1'b0}};
    reg clk_audio = 1'b0;
    always @(posedge clk_pix)
        if (aacc >= AMOD - AINC) begin aacc <= aacc + AINC - AMOD; clk_audio <= ~clk_audio; end
        else aacc <= aacc + AINC;
    reg [15:0] smp = 16'd0;
    always @(posedge clk_audio) smp <= smp + 16'd1237;
    wire [15:0] aw [1:0];
    assign aw[0] = smp;  assign aw[1] = ~smp;
    wire [11:0] cx;  wire [10:0] cy;
    reg  [23:0] rgb = 24'd0;
    always @(posedge clk_pix) rgb <= {cx[7:0], cy[7:0], cx[11:4]};
    wire [9:0] t0, t1, t2;
    wire [11:0] fw_o;  wire [10:0] fh_o;
`ifdef TANGO_VSYNC_ADJUST
    // the schedule: the next frame's totals, changed mid-frame (cy 360), as tango_vsync does
    reg [11:0] nw = FW;  reg [10:0] nh = FH;
    integer    sched = 0;
    always @(posedge clk_pix) if (!rst && cx == 12'd0 && cy == 11'd360) begin
        sched = sched + 1;
        case (sched % 6)
        1: begin nw <= 12'd1657; nh <= 11'd749; end
        2: begin nw <= 12'd1664; nh <= 11'd743; end
        3: begin nw <= 12'd1720; nh <= 11'd790; end
        4: begin nw <= 12'd1626; nh <= 11'd740; end
        5: begin nw <= 12'd1657; nh <= 11'd749; end
        default: begin nw <= 12'd1650; nh <= 11'd750; end
        endcase
    end
    // the latch: at every wrap the new frame's totals are the ones present at the wrap
    integer n_latch_bad = 0;
    reg [11:0] nw_q = 12'd0;  reg [10:0] nh_q = 11'd0;  reg wrap_seen = 1'b0;
    always @(posedge clk_pix) begin
        nw_q <= nw;  nh_q <= nh;
        wrap_seen <= !rst && cx == fw_o - 12'd1 && cy == fh_o - 11'd1;
        if (wrap_seen && (fw_o != nw_q || fh_o != nh_q)) begin
            n_latch_bad = n_latch_bad + 1;
            $display("LATCH BAD: frame started with %0d x %0d, next was %0d x %0d", fw_o, fh_o, nw_q, nh_q);
        end
    end
`endif
    hdmi #(.AUDIO_RATE(48000), .AUDIO_BIT_WIDTH(16), .IT_CONTENT(1'b1),
`ifdef TANGO_HDMI_PLL_SET
           .ACR_N(`TANGO_HDMI_ACR_N), .ACR_CTS(`TANGO_HDMI_ACR_CTS),
`endif
           .FRAME_WIDTH(FW), .FRAME_HEIGHT(FH), .VIC(`TB_VIC)) u_hdmi (
`ifdef TANGO_VSYNC_ADJUST
        .next_frame_width(nw), .next_frame_height(nh),
`endif
        .frame_height_adj(4'd0), .clk_pixel_x5(1'b0), .clk_pixel(clk_pix), .clk_audio(clk_audio), .reset(rst),
        .rgb(rgb), .audio_sample_word(aw), .tmds_ch0(t0), .tmds_ch1(t1), .tmds_ch2(t2), .cx(cx), .cy(cy),
        .frame_width(fw_o), .frame_height(fh_o), .screen_width(), .screen_height());
`ifdef TB_ORIG
    wire [9:0] o0, o1, o2;  wire [11:0] ocx;  wire [10:0] ocy;
    hdmi_orig #(.AUDIO_RATE(48000), .AUDIO_BIT_WIDTH(16), .IT_CONTENT(1'b1)) u_orig (
        .frame_height_adj(4'd0), .clk_pixel_x5(1'b0), .clk_pixel(clk_pix), .clk_audio(clk_audio), .reset(rst),
        .rgb(rgb), .audio_sample_word(aw), .tmds_ch0(o0), .tmds_ch1(o1), .tmds_ch2(o2), .cx(ocx), .cy(ocy),
        .frame_width(), .frame_height(), .screen_width(), .screen_height());
    integer n_diff = 0, n_cmp = 0;
    always @(posedge clk_pix) if (!rst) begin
        n_cmp = n_cmp + 1;
        if ({t0, t1, t2, cx, cy} !== {o0, o1, o2, ocx, ocy}) begin
            n_diff = n_diff + 1;
            if (n_diff <= 5) $display("ORIG DIFF at cx %0d cy %0d: tmds %h %h %h vs original %h %h %h", cx, cy, t0, t1, t2, o0, o1, o2);
        end
    end
`endif
    // ---- the ACR packet and the audio rate (the PLL presets)
`ifdef TANGO_HDMI_PLL_SET
    localparam longint WANT_N = `TANGO_HDMI_ACR_N, WANT_CTS = `TANGO_HDMI_ACR_CTS;
`else
    localparam longint WANT_N = 6144, WANT_CTS = 74250;
`endif
    integer n_acr = 0, n_acr_bad = 0, n_aud = 0;
    reg [7:0] pt_a = 8'd0;
    wire [55:0] acr = u_hdmi.packet_picker.sub_acr[0];
    wire [19:0] acr_n = {acr[35:32], acr[47:40], acr[55:48]}, acr_cts = {acr[11:8], acr[23:16], acr[31:24]};
    always @(posedge clk_pix) begin
        pt_a <= u_hdmi.packet_picker.packet_type;
        if (!rst && u_hdmi.packet_picker.packet_type == 8'h01 && pt_a != 8'h01) begin
            n_acr = n_acr + 1;
            if (acr_n != 20'(WANT_N) || acr_cts != 20'(WANT_CTS)) begin
                n_acr_bad = n_acr_bad + 1;
                if (n_acr_bad <= 3) $display("ACR BAD: N %0d CTS %0d (want %0d / %0d)", acr_n, acr_cts, WANT_N, WANT_CTS);
            end
        end
    end
    longint t_aud0 = 0;  integer n_aud0 = 0;  reg aud_started = 1'b0;
    always @(posedge clk_audio) if (!rst) begin        // rises from the first one after reset: n = floor(t x 48000) + 1
        if (!aud_started) begin aud_started <= 1'b1; t_aud0 = $time; n_aud0 = n_aud; end
        n_aud = n_aud + 1;
    end
    // ---- the checks
    reg  [11:0] cx1 = 12'd0, cx2 = 12'd0;  reg [10:0] cy1 = 11'd0, cy2 = 11'd0;
    reg  [11:0] fw1 = 12'd0, fw2 = 12'd0;  reg [10:0] fh1 = 11'd0, fh2 = 11'd0;   // the totals of cx2 / cy2's frame
    reg  [1:0]  vld = 2'd0;
    always @(posedge clk_pix) begin
        cx1 <= cx; cx2 <= cx1; cy1 <= cy; cy2 <= cy1;
        fw1 <= fw_o; fw2 <= fw1; fh1 <= fh_o; fh2 <= fh1;
        vld <= rst ? 2'd0 : {vld[0], 1'b1};
    end
    wire        pre_line = (cy2 == fh2 - 11'd1) || (cy2 < 719);
    wire        e_video  = (cx2 < 1280) && (cy2 < 720);
    wire        e_vguard = (cx2 >= fw2 - 12'd2) && (cx2 < fw2) && pre_line;
    wire        e_isl    = (cx2 >= 1292) && (cx2 < 1616);
    wire        e_vpre   = (cx2 >= fw2 - 12'd10) && (cx2 < fw2 - 12'd2) && pre_line;
    wire        e_ipre   = (cx2 >= 1284) && (cx2 < 1292);
    wire [2:0]  m  = u_hdmi.mode;
    wire [5:0]  cd = u_hdmi.control_data;
    wire        hs = u_hdmi.hsync, vs = u_hdmi.vsync;
    integer n_mode = 0, n_pre = 0, n_vid = 0, n_frames = 0, n_hs_bad = 0, n_vs_bad = 0, n_vid_bad = 0, n_avi = 0;
    integer n_avi_bad = 0, n_hs = 0, n_vs = 0, n_avi_frames_missing = 0, avi_in_frame = 0;
    reg     hs_q = 1'b0, vs_q = 1'b0;
    longint clk = 0, hs_t = -1, vs_t = -1, hs_w = 0, vs_w = 0, vs_h = 0;
    reg [7:0] pt_q = 8'd0;
    always @(posedge clk_pix) begin
        clk = clk + 1;
        hs_q <= hs;  vs_q <= vs;
        pt_q <= u_hdmi.packet_picker.packet_type;
        if (!rst && vld[1]) begin
            // the windows
            if ((m == 3'd1) != e_video || (m == 3'd2) != e_vguard || ((m == 3'd3) || (m == 3'd4)) != e_isl) begin
                n_mode = n_mode + 1;
                if (n_mode <= 5) $display("MODE BAD: mode %0d at cx2 %0d cy2 %0d (video %0d guard %0d island %0d)", m, cx2, cy2,
                                          e_video, e_vguard, e_isl);
            end
            if (m == 3'd0 && ((cd[2] && !cd[4]) != e_vpre || cd[4] != e_ipre)) begin
                n_pre = n_pre + 1;
                if (n_pre <= 5) $display("PREAMBLE BAD: ctl %b at cx2 %0d cy2 %0d (video preamble %0d island preamble %0d)",
                                         cd[5:2], cx2, cy2, e_vpre, e_ipre);
            end
            if (m == 3'd1) n_vid = n_vid + 1;
        end
        if (!rst) begin
            // HSYNC: rises at cx 1390, every FW clocks
            if (hs && !hs_q) begin
                n_hs = n_hs + 1;
                if (cx != 12'd1390 || (hs_t >= 0 && clk - hs_t != hs_w)) begin
                    n_hs_bad = n_hs_bad + 1;
                    if (n_hs_bad <= 5) $display("HSYNC BAD: rise at cx %0d, %0d clocks after the last (want %0d)", cx, clk - hs_t, hs_w);
                end
                hs_t = clk;  hs_w = fw_o;
            end
            // VSYNC: rises at (1390, 724), falls at (1390, 729), every FW x FH clocks
            if (vs && !vs_q) begin
                n_vs = n_vs + 1;
                if (cx != 12'd1390 || cy != 11'd724 || (vs_t >= 0 && clk - vs_t != vs_w * (vs_h - 724) + 724 * fw_o)) begin
                    n_vs_bad = n_vs_bad + 1;
                    $display("VSYNC BAD: rise at (%0d, %0d), %0d clocks after the last (want %0d)", cx, cy, clk - vs_t,
                             vs_w * (vs_h - 724) + 724 * fw_o);
                end
                if (vs_t >= 0) $display("VSYNC rise at (%0d, %0d): period %0d clk_pix (frame before %0d x %0d, this %0d x %0d) -> %.6f Hz at %.4f MHz",
                                        cx, cy, clk - vs_t, vs_w, vs_h, fw_o, fh_o, real'(FPIX) / (clk - vs_t), real'(FPIX) / 1e6);
                vs_t = clk;  vs_w = fw_o;  vs_h = fh_o;
            end
            if (!vs && vs_q && (cx != 12'd1390 || cy != 11'd729)) begin
                n_vs_bad = n_vs_bad + 1;
                $display("VSYNC BAD: fall at (%0d, %0d)", cx, cy);
            end
            // the AVI InfoFrame: picked at least once per frame, PB4 = the VIC
            if (u_hdmi.packet_picker.packet_type == 8'h82 && pt_q != 8'h82) begin
                n_avi = n_avi + 1;  avi_in_frame = avi_in_frame + 1;
                if (u_hdmi.packet_picker.sub_avi[0][39:32] != 8'(`TB_VIC)) n_avi_bad = n_avi_bad + 1;
            end
            // a frame ends at the raster wrap
            if (cx == fw_o - 12'd1 && cy == fh_o - 11'd1) begin
                if (n_frames >= 1) begin
                    if (n_vid != 1280 * 720) begin
                        n_vid_bad = n_vid_bad + 1;
                        $display("VIDEO BAD: frame %0d has %0d video-data clocks (want %0d)", n_frames, n_vid, 1280 * 720);
                    end
                    if (avi_in_frame == 0) n_avi_frames_missing = n_avi_frames_missing + 1;
                end
                n_vid = 0;  avi_in_frame = 0;
                n_frames = n_frames + 1;
                if (n_frames == NF + 1) begin
                    $display("HDMI_TOTALS frame_width %0d frame_height %0d (outputs) VIC %0d: %0d frames, VSYNC rises %0d (bad %0d), HSYNC rises %0d (bad %0d), mode errors %0d, preamble errors %0d, video-data frames bad %0d, AVI InfoFrames %0d (PB4 != VIC %0d, frames without one %0d)",
                             fw_o, fh_o, `TB_VIC, NF, n_vs, n_vs_bad, n_hs, n_hs_bad, n_mode, n_pre, n_vid_bad, n_avi,
                             n_avi_bad, n_avi_frames_missing);
`ifdef TB_ORIG
                    $display("HDMI_TOTALS original hdmi.sv: %0d clocks compared, %0d differ", n_cmp, n_diff);
`endif
`ifdef TANGO_VSYNC_ADJUST
                    $display("HDMI_TOTALS runtime: %0d frames started with the wrong totals", n_latch_bad);
`endif
                    $display("HDMI_TOTALS f_pix %0d Hz: ACR packets %0d (N / CTS != %0d / %0d: %0d); f_pix x N %s 128 x 48000 x CTS; clk_audio %0d rises in %.6f s = %.3f Hz (48 kHz %s within +-1 sample)",
                             FPIX, n_acr, WANT_N, WANT_CTS, n_acr_bad, (FPIX * WANT_N == 64'd128 * 64'd48000 * WANT_CTS) ? "==" : "!=",
                             n_aud - n_aud0, real'($time - t_aud0) * 1e-12, real'(n_aud - n_aud0) / (real'($time - t_aud0) * 1e-12),
                             ((n_aud - n_aud0) - 48000.0 * real'($time - t_aud0) * 1e-12 <= 1.0 &&
                              (n_aud - n_aud0) - 48000.0 * real'($time - t_aud0) * 1e-12 >= -1.0) ? "yes" : "NO");
                    $display("HDMI_TOTALS %0s", (n_acr > 0 && n_acr_bad == 0 && FPIX * WANT_N == 64'd128 * 64'd48000 * WANT_CTS &&
                             (n_aud - n_aud0) - 48000.0 * real'($time - t_aud0) * 1e-12 <= 1.0 &&
                             (n_aud - n_aud0) - 48000.0 * real'($time - t_aud0) * 1e-12 >= -1.0 &&
`ifdef TANGO_VSYNC_ADJUST
                             n_latch_bad == 0 &&
`else
                             fw_o == FW && fh_o == FH &&
`endif
                             n_vs >= NF - 1 && n_vs_bad == 0 && n_hs_bad == 0 &&
                             n_mode == 0 && n_pre == 0 && n_vid_bad == 0 && n_avi > 0 && n_avi_bad == 0 &&
                             n_avi_frames_missing == 0
`ifdef TB_ORIG
                             && n_diff == 0 && n_cmp > 0
`endif
                             ) ? "PASS" : "FAIL");
                    $finish;
                end
            end
        end
    end
endmodule
`default_nettype wire
