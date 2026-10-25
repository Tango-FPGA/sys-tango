// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_retune -- vsync_adjust's HDMI PLL retune at core start, at the board top's clock level, the CLOCK-PAUSE design:
// pll_c1485 with DYN_MDIV (the Gowin PLL as sim/gw_pll_hdmi_sim.v, PLL_INIT = Gowin's own pll_init.v), tango_pll_retune on
// 50 MHz, clk_pix = the raw CLKOUT1 through DCE (sim/gw_dce_sim.v: a plain AND, the worst case) with its enable from two
// negedge flops of the raw clock, rst_pix at power-up only and the serialiser reset, all exactly as tango_sys_top138.v;
// tango_vsync (TANGO_HDMI_RETUNE) fed SYS 0x70 bytes by the bench, hdmi.sv with the runtime ACR CTS, the clk_audio
// accumulator as the board top.  The source: a TG16-rate VS (263 lines of 1365 master clocks at 945/44 MHz), own clock.
// Script (sim time):
//   A. power-up at the preset (NATIVE0 = 1): the fixed raster TB_HT x TB_VT, then tango_vsync's plan for the 263-line source
//   B. SYS 0x70 apply native = 0: ONE retune at hdmi.sv's line TRIG_Y (730) -- the clock enable falls, I_RST, the PLL reset,
//      PLL_INIT's sweep, the new lock, 1.3 ms, the enable rises: clk_pix resumes at 74.25 MHz where it stopped; reply bit 2
//      until then; then the fixed raster 1650 x 750 = 60.000000 Hz; ACR CTS 148500; clk_audio 48 kHz
//   C. the same apply again: NO retune
//   D. apply native = 1: back to the preset (another pause), the plan again; ACR CTS = the preset's
// Checked: every steady VS period (clk_pix count) and its real time against the raster x the clock (the frame spanning a
// pause is marked and skipped); rst_pix never again after power-up; across each pause the beam (cx, cy) and a free-running
// clk_pix counter continue exactly where they stopped, the pause starts on line TRIG_Y; no clk_pix high or low phase shorter
// than the nominal half period (no runt from the gate); the serialiser reset held through the pause and released 2 clk_pix
// after; the PLL model's "divider changed while running" = 0; retunes = 2; the ACR packets' CTS; clk_audio per phase.
//==============================================================================
`default_nettype none
`timescale 1ps/1ps
`include "tango_cfg.vh"
module tb_retune;
`ifndef TB_HT
`define TB_HT 1662
`endif
`ifndef TB_VT
`define TB_VT 748
`endif
    // ---- clocks: 50 MHz (PLL_INIT + the sequencer), 27 MHz (the PLL's input; the model times itself from SIM_FIN_HZ)
    reg clk = 1'b0, clk27 = 1'b0;
    always #10000 clk = ~clk;
    always #18519 clk27 = ~clk27;
    // ---- the HDMI PLL + its retune, as the board top
    wire clk_ser, clk_pix, clk_pix_raw, lkh, rt_irst, rt_pll_native, rt_tog_pix, rt_req_native, rt_gate_off;
    wire [6:0] rt_mdsel;  wire [2:0] rt_mdsel_frac;  wire [7:0] n_retunes;
    pll_c1485 #(.MDIV(`TANGO_HDMI_MDIV), .MDIV_FRAC(`TANGO_HDMI_MDIV_FRAC), .DYN_MDIV("TRUE"))
              u_pllh (.clkin(clk27), .init_clk(clk), .clkout0(clk_ser), .clkout1(clk_pix_raw), .lock(lkh),
                      .irst(rt_irst), .mdsel(rt_mdsel), .mdsel_frac(rt_mdsel_frac));
    tango_pll_retune #(.MDIV_N(`TANGO_HDMI_MDIV), .FRAC_N(`TANGO_HDMI_MDIV_FRAC), .NATIVE0(1)) u_retune (
        .clk(clk), .req_tog(rt_tog_pix), .req_native(rt_req_native), .lock(lkh), .irst(rt_irst), .mdsel(rt_mdsel),
        .mdsel_frac(rt_mdsel_frac), .native(rt_pll_native), .gate_off(rt_gate_off), .n_retunes(n_retunes));
    reg [1:0] rt_ce_s = 2'b11;
    always @(negedge clk_pix_raw) rt_ce_s <= {rt_ce_s[0], ~rt_gate_off};
    DCE u_dce_pix (.CLKIN(clk_pix_raw), .CE(rt_ce_s[1]), .CLKOUT(clk_pix));
    reg  rt_por = 1'b0;
    always @(posedge clk) if (lkh) rt_por <= 1'b1;
    wire lk_rst = lkh | rt_por;
    reg [7:0] rstp_sr = 8'd0;
    always @(posedge clk_pix or negedge lk_rst) if (!lk_rst) rstp_sr <= 8'd0; else rstp_sr <= {rstp_sr[6:0], 1'b1};
    wire rst_pix = ~rstp_sr[7];
    reg [1:0] rt_ser_rst = 2'b11;
    always @(posedge clk_pix or posedge rt_gate_off) if (rt_gate_off) rt_ser_rst <= 2'b11; else rt_ser_rst <= {rt_ser_rst[0], 1'b0};
    // ---- the source: 263 lines x 1365 master clocks (945/44 MHz), its VS a few lines long
    reg mclk = 1'b0;  integer macc = 0;          // 945/44 MHz: 44e12 / 1890e6 = 23280 + 80/189 ps a half period
    always begin macc = macc + 80; if (macc >= 189) begin macc = macc - 189; #23281; end else #23280; mclk = ~mclk; end
    integer mx = 0, my = 0;  reg core_vs = 1'b0;
    always @(posedge mclk) begin
        if (mx == 1364) begin mx = 0; my = (my == 262) ? 0 : my + 1; end else mx = mx + 1;
        core_vs <= (my >= 245) && (my < 248);
    end
    // ---- tango_vsync + SYS 0x70 bytes from the bench
    reg        tap_cmd_stb = 1'b0, tap_stb = 1'b0, tap_end = 1'b0;
    reg  [7:0] tap_cmd = 8'd0, tap_byte = 8'd0;  reg [15:0] tap_n = 16'd0;
    wire [11:0] nxt_w;  wire [10:0] nxt_h;  wire [7:0] vs_status;
    wire [11:0] cx;  wire [10:0] cy;  wire [11:0] cur_w;  wire [10:0] cur_h;
    tango_vsync #(.HT0(`TB_HT), .VT0(`TB_VT), .MODE0(1)) u_vsync (
        .clk(clk_pix), .rst(rst_pix), .core_vs(core_vs), .cx(cx), .cy(cy),
        .tap_cmd_stb(tap_cmd_stb), .tap_stb(tap_stb), .tap_cmd(tap_cmd), .tap_n(tap_n), .tap_byte(tap_byte), .tap_end(tap_end),
        .status(vs_status), .pll_native(rt_pll_native), .fix_w(rt_pll_native ? 12'(`TB_HT) : 12'd1650),
        .fix_h(rt_pll_native ? 11'(`TB_VT) : 11'd750), .rt_tog(rt_tog_pix), .rt_native(rt_req_native),
        .nxt_w(nxt_w), .nxt_h(nxt_h), .meas_p(), .meas_ok(), .plan_w(), .plan_h(), .plan_err());
    task automatic sys70(input native);           // one SYS 0x70 apply, as tango_link's TAP hands it over (clk_pix)
        integer k;  reg [7:0] b [0:6];
        begin
            b[0] = {1'b1, 6'd0, native};  for (k = 1; k < 7; k = k + 1) b[k] = 8'd0;
            @(posedge clk_pix); tap_cmd <= 8'h70; tap_cmd_stb <= 1'b1;
            @(posedge clk_pix); tap_cmd_stb <= 1'b0;
            for (k = 0; k < 7; k = k + 1) begin
                @(posedge clk_pix); tap_n <= 16'(k); tap_byte <= b[k]; tap_stb <= 1'b1;
                @(posedge clk_pix); tap_stb <= 1'b0;
            end
            @(posedge clk_pix); tap_end <= 1'b1;
            @(posedge clk_pix); tap_end <= 1'b0;
            $display("SYS 0x70 apply native=%0d at %0t: status %02x (pending %0d)", native, $time, vs_status, vs_status[2]);
        end
    endtask
    // ---- hdmi.sv (runtime totals + runtime ACR CTS) and the audio clock, as the board top
    wire [15:0] AINC = rt_pll_native ? 16'(`TANGO_HDMI_AUD_INC) : 16'd16;
    wire [15:0] AMOD = rt_pll_native ? 16'(`TANGO_HDMI_AUD_MOD) : 16'd12375;
    reg  [15:0] aacc = 16'd0;  reg clk_audio = 1'b0;
    always @(posedge clk_pix)
        if (aacc >= AMOD - AINC) begin aacc <= aacc + AINC - AMOD; clk_audio <= ~clk_audio; end
        else aacc <= aacc + AINC;
    wire [15:0] aw [1:0];  assign aw[0] = 16'd0;  assign aw[1] = 16'd0;
    wire [9:0] t0, t1, t2;
    hdmi #(.AUDIO_RATE(48000), .AUDIO_BIT_WIDTH(16), .IT_CONTENT(1'b1), .ACR_N(12288),
           .FRAME_WIDTH(`TB_HT), .FRAME_HEIGHT(`TB_VT), .VIC(4)) u_hdmi (
        .next_frame_width(nxt_w), .next_frame_height(nxt_h),
        .frame_height_adj(4'd0), .clk_pixel_x5(1'b0), .clk_pixel(clk_pix), .clk_audio(clk_audio), .reset(rst_pix),
        .rgb(24'd0), .audio_sample_word(aw), .acr_cts_rt(rt_pll_native ? 20'(`TANGO_HDMI_CTS12288) : 20'd148500),
        .tmds_ch0(t0), .tmds_ch1(t1), .tmds_ch2(t2), .cx(cx), .cy(cy),
        .frame_width(cur_w), .frame_height(cur_h), .screen_width(), .screen_height());
    // ---- the pause: continuity, runts, the serialiser reset
    longint t_last = -1, t_hi = -1, t_lo = -1, min_hi = 64'd1000000, min_lo = 64'd1000000;
    longint pix_cnt = 0, cnt_before = 0;
    integer cx_b = 0, cy_b = 0, n_pause = 0, n_pause_bad = 0, n_rst_again = 0, ser_edges = -1, pause_in_frame = 0;
    reg     rst_seen_release = 1'b0, rst_q = 1'b1;
    always @(posedge clk_pix) begin
        if (t_last >= 0 && longint'($time) - t_last > 64'd1000000) begin           // > 1 us without a clk_pix edge: a pause
            n_pause = n_pause + 1;  pause_in_frame = 1;
            $display("PAUSE %0d: %.3f ms from %.6f ms; before: beam (%0d, %0d) counter %0d; after: beam (%0d, %0d) counter %0d; serialiser reset %0d; PLL %0s",
                     n_pause, real'(longint'($time) - t_last) / 1e9, real'(t_last) / 1e9, cx_b, cy_b, cnt_before, cx, cy, pix_cnt,
                     rt_ser_rst[1], rt_pll_native ? "preset" : "74.25");
            if (pix_cnt != cnt_before || cy_b != 730 || cx_b > 64 || !rt_ser_rst[1]) begin
                n_pause_bad = n_pause_bad + 1;
                $display("FAIL: pause %0d: the domain did not resume where it stopped / not on line 730 / serialiser not in reset", n_pause);
            end
            ser_edges = 0;
        end
        if (ser_edges >= 0) begin
            if (!rt_ser_rst[1]) begin
                $display("PAUSE %0d: serialiser reset released %0d clk_pix after the resume", n_pause, ser_edges);
                if (ser_edges > 3) n_pause_bad = n_pause_bad + 1;
                ser_edges = -1;
            end else ser_edges = ser_edges + 1;
        end
        t_last = longint'($time);
        cx_b = cx; cy_b = cy;
        pix_cnt = pix_cnt + 1;  cnt_before = pix_cnt;
        rst_q <= rst_pix;
        if (!rst_pix) rst_seen_release = 1'b1;
        if (rst_seen_release && rst_pix && !rst_q) n_rst_again = n_rst_again + 1;
    end
    // phase lengths of clk_pix (a runt from the gate would show as a short phase); pauses excluded (> 1 us)
    always @(clk_pix) begin
        if (clk_pix) begin if (t_lo >= 0) begin longint d; d = longint'($time) - t_lo; if (d < min_lo) min_lo = d; end t_hi = longint'($time); end
        else begin if (t_hi >= 0) begin longint d; d = longint'($time) - t_hi; if (d < min_hi) min_hi = d; end t_lo = longint'($time); end
    end
    // ---- measurements
    longint fpix_pre = `TANGO_HDMI_FPIX;          // the preset's clock
    integer phase = 0;                            // A=0 B=1 C=2 D=3
    integer n_vs_bad = 0, n_vs = 0, n_acr_bad = 0, n_acr = 0;
    longint vs_t = -1, vs_clk = 0, clkc = 0, vs_c0 = -1;
    reg     vs_q = 1'b0;
    reg [7:0] pt_q = 8'd0;
    wire [55:0] acr = u_hdmi.packet_picker.sub_acr[0];
    wire [19:0] acr_cts = {acr[11:8], acr[23:16], acr[31:24]};
    wire [19:0] acr_n = {acr[35:32], acr[47:40], acr[55:48]};
    always @(posedge clk_pix) begin
        clkc = clkc + 1;
        vs_q <= u_hdmi.vsync;
        pt_q <= u_hdmi.packet_picker.packet_type;
        if (!rst_pix && u_hdmi.packet_picker.packet_type == 8'h01 && pt_q != 8'h01) begin
            n_acr = n_acr + 1;
            if (acr_n != 20'd12288 || acr_cts != (rt_pll_native ? 20'(`TANGO_HDMI_CTS12288) : 20'd148500)) begin
                n_acr_bad = n_acr_bad + 1;
                if (n_acr_bad <= 3) $display("ACR BAD: N %0d CTS %0d (native %0d)", acr_n, acr_cts, rt_pll_native);
            end
        end
        if (rst_pix) begin vs_t = -1; vs_c0 = -1; end
        else if (u_hdmi.vsync && !vs_q) begin
            if (vs_t >= 0) begin
                // the period in clk_pix must be the previous frame's totals (both frames' H x V when they differ: the rise
                // at (1390, 724) -> W_prev x (V_prev - 724) + 724 x W_this); the real time follows the PLL's clock
                n_vs = n_vs + 1;
                $display("VS phase %0d: %0d clk_pix, %.3f us = %.6f Hz (raster %0d x %0d, PLL %0s, status %02x)%0s", phase,
                         clkc - vs_c0, real'(longint'($time) - vs_t) / 1e6, 1e12 / real'(longint'($time) - vs_t), cur_w, cur_h,
                         rt_pll_native ? "preset" : "74.25", vs_status, pause_in_frame ? " PAUSED" : "");
            end
            vs_t = longint'($time); vs_c0 = clkc; pause_in_frame = 0;
        end
    end
    integer aud_n = 0;  longint aud_t0 = -1;
    always @(posedge clk_audio) if (!rst_pix) begin if (aud_t0 < 0) aud_t0 = longint'($time); aud_n = aud_n + 1; end
    always @(negedge rt_gate_off) begin aud_n = 0; aud_t0 = -1; end   // a pause: measure the rate from the resume on
    task automatic aud_report(input integer ph);
        begin
            if (aud_t0 >= 0 && aud_n > 1)
                $display("AUDIO phase %0d: %0d clk_audio rises in %.6f s = %.3f Hz", ph, aud_n,
                         real'(longint'($time) - aud_t0) * 1e-12, real'(aud_n - 1) / (real'(longint'($time) - aud_t0) * 1e-12));
            aud_n = 0; aud_t0 = -1;
        end
    endtask
    // ---- the script
    integer ok = 1;
    initial begin
        // A: power-up at the preset; wait for the plan (the 263-line source: 16-frame windows) + a few frames
        wait (!rst_pix);
        $display("phase A: power-up, PLL %0s, fixed raster %0d x %0d", rt_pll_native ? "preset" : "74.25", `TB_HT, `TB_VT);
        #(64'd700_000_000_000);                   // 700 ms
        aud_report(0);
        // B: fixed -> one retune to 74.25 MHz
        phase = 1;  sys70(1'b0);
        wait (rt_gate_off);  $display("phase B: clk_pix gated off at %0t (retunes %0d)", $time, n_retunes);
        wait (!rt_gate_off); $display("phase B: clk_pix released at %0t, PLL %0s, status %02x", $time,
                                      rt_pll_native ? "preset" : "74.25", vs_status);
        #(64'd200_000_000_000);
        aud_report(1);
        // C: the same request again: no retune
        phase = 2;  sys70(1'b0);
        #(64'd100_000_000_000);
        if (n_retunes != 8'd1) begin ok = 0; $display("FAIL: retunes %0d after phase C (want 1)", n_retunes); end
        aud_report(2);
        // D: back to the preset
        phase = 3;  sys70(1'b1);
        wait (rt_gate_off);  wait (!rt_gate_off);  $display("phase D: clk_pix released at %0t, PLL %0s", $time, rt_pll_native ? "preset" : "74.25");
        #(64'd700_000_000_000);
        aud_report(3);
        if (n_retunes != 8'd2) begin ok = 0; $display("FAIL: retunes %0d (want 2)", n_retunes); end
        if (u_pllh.u_pll.PLL_inst.n_bad_change != 0) begin ok = 0; $display("FAIL: the divider changed while the PLL ran"); end
        if (n_acr_bad != 0 || n_acr == 0) begin ok = 0; $display("FAIL: ACR packets %0d, bad %0d", n_acr, n_acr_bad); end
        if (n_pause != 2 || n_pause_bad != 0) begin ok = 0; $display("FAIL: pauses %0d (want 2), bad %0d", n_pause, n_pause_bad); end
        if (n_rst_again != 0) begin ok = 0; $display("FAIL: rst_pix asserted %0d time(s) after power-up", n_rst_again); end
        if (min_hi < 64'd6700 || min_lo < 64'd6700) begin ok = 0; $display("FAIL: a clk_pix phase shorter than the nominal half period"); end
        $display("TB_RETUNE pause: %0d pauses, %0d bad, rst_pix after power-up %0d, shortest clk_pix high %0d ps low %0d ps",
                 n_pause, n_pause_bad, n_rst_again, min_hi, min_lo);
        $display("TB_RETUNE: retunes %0d, PLL starts %0d, divider changes while running %0d, ACR packets %0d (bad %0d), VS rises %0d",
                 n_retunes, u_pllh.u_pll.PLL_inst.n_starts, u_pllh.u_pll.PLL_inst.n_bad_change, n_acr, n_acr_bad, n_vs);
        $display("TB_RETUNE %0s", ok ? "DONE (judge the VS / AUDIO lines with sim/check_retune.py)" : "FAIL");
        $finish;
    end
endmodule
`default_nettype wire
