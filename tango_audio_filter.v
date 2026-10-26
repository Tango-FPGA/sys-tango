// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 the Tango authors
// Copyright (C) 2020 Sorgelig (sys/iir_filter.v)
// (c)2020-2026 Alexey Melnikov (sys/audio_out.sv)
//
// This file is derived from MiSTer's audio filter and mix: the MiSTer project's Template_MiSTer
// (https://github.com/MiSTer-devel/Template_MiSTer, compared at commit 5dda373), sys/iir_filter.v (modules IIR_filter,
// iir_filter_tap, DC_blocker) and sys/audio_out.sv (module aud_mix_top): their arithmetic, stage by stage and in
// their order, with their constants (the 48 kHz DC-blocker shifts, the boost localparams) and signal names.  Both
// files state: "This program is free software; you can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your
// option) any later version."  This file is under the same terms (GPL-2.0-or-later; LICENSES/GPL-2.0.txt,
// LICENSES/GPL-3.0.txt), not under the MIT licence of the rest of sys-tango; see NOTICE.  A bitstream that contains it is a work under the GPL.
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
//==============================================================================
// tango_audio_filter -- MiSTer's audio filter (README "Video and audio processing"): the 3-tap IIR at the
// filter file's rate, the 48 kHz hand-over and the DC blocker, between the core's AUDIO_L/R and the framework's
// 48 kHz HDMI sampling (TANGO_AUDIO_FILTER).  The arithmetic is MiSTer's (Template_MiSTer 5dda373
// sys/iir_filter.v 52-97 and 181-208 (IIR), 211-235 (DC_blocker) and sys/audio_out.sv 165-213 (rates,
// mute)); the file values arrive as MiSTer's sys_top keeps them (tango_vproc_regs: rate, cx, cx0-2, cy0-2).
// Per channel and filter step, all 40-bit two's complement (wrapping as MiSTer's 40-bit nets):
//   x   = (in * cx) >>> 20                       in: 16-bit signed sample, cx 40 bit
//   y   = x + t0                                 out = y >>> 20, saturated to 16 bits (iir_filter.v 99)
//   t0' = xs(cx0, x) - ((y[36:0] * cy0) >>> 21) + t1
//   t1' = xs(cx1, x) - ((y[36:0] * cy1) >>> 21) + t2
//   t2' = xs(cx2, x) - ((y[36:0] * cy2) >>> 21)
//   xs(c, x) = (c[0] ? x>>>4 : 0) + (c[1] ? x>>>3 : 0) + (c[2] ? x>>>2 : 0), one's complement if c[7]
// Steps: `rate` per second per channel (an accumulator on clk_pix; MiSTer runs the pair at 2 x rate, alternating the
// channels).  Both channels take the core's sample of the step's clock.  At most one step per 10 clk_pix clocks
// (rate <= 7.425 MHz; larger rates are clamped to that): the pipeline below needs 8.  Every Filters_MiSTer file
// asks 7.056 or 6.144 MHz.
// 48 kHz: the last outputs are taken at an exact 48 kHz strobe (clk_pix x 8 / 12375, the board top's own ratio; a preset's
// pair -- 74.5875 MHz: x 32 / 49725 -- with TANGO_HDMI_PLL_<f>) and
// go through MiSTer's DC blocker (the 48 kHz coefficients: x - x>>>10, y - y>>>9), which reads the value taken at
// the strobe before (audio_out.sv: the IIR's out register and the blocker share sample_ce).  For 8192 strobes after
// a reset or a filter load the output is 0 (audio_out.sv a_en2: dly2[13]).
// Then MiSTer's aud_mix_top per channel (audio_out.sv 246-345), one stage per 48 kHz sample: boost (the core volume's
// bits 6:5, 0x39), the stereo mix with the other channel (AUDIO_MIX from the core: 0 none, 1 25 %, 2 50 %, 3 mono),
// the global attenuation (0x26 UIO_AUDVOL: >>> att[3:0], att[4] mutes) and the 16-bit clamp.  Linux audio (ALSA) is
// MiSTer's only (0 here).  MiSTer powers up muted (sys_top.v 317-318: vol_att = 5'b11111 until Main sends the volume);
// Tango powers up at full volume (att 0): a board without the firmware still sounds.
// Bit-exact model: build_tools/iir_model.py.
//==============================================================================
`default_nettype none
// CLK_HZ_P / S48_* (the TANGO_HDMI_PLL presets): the pixel clock and the exact 48 kHz strobe pair for
// a TANGO_HDMI_PLL_<f> preset (tango_sys passes tango_cfg.vh's TANGO_HDMI_FPIX / _S48_*); defaults = 74.25 MHz, +8 mod 12375.
module tango_audio_filter #(
    parameter [31:0] CLK_HZ_P = 32'd74_250_000,
    parameter integer S48_W = 14, S48_INC = 8, S48_MOD = 12375
) (
    input  wire        clk,                 // clk_pix (74.25 MHz, or a preset's)
    input  wire        rst,
    input  wire        a_rst,               // a filter load is in progress (tango_vproc_regs)
    input  wire [31:0] rate,
    input  wire [39:0] cx,
    input  wire [7:0]  cx0, cx1, cx2,
    input  wire [23:0] cy0, cy1, cy2,
    input  wire [15:0] in_l, in_r,          // the core's samples (signed), clk_pix
    input  wire [4:0]  att,                 // global volume (0x26): >>> att[3:0], att[4] = mute
    input  wire [1:0]  boost,               // core volume boost (0x39 n0[6:5]): 0 off, 1 x2, 2 x4 (compressed)
    input  wire [1:0]  mix,                 // the core's AUDIO_MIX (quasi-static, any clock)
`ifdef TANGO_HDMI_RETUNE
    input  wire        pll_native,          // TANGO_HDMI_RETUNE: 1 = the preset's clock (the parameters), 0 = 74.25 MHz (quasi-static)
`endif
    output reg  [15:0] out_l = 16'd0,       // to the 48 kHz HDMI sampling
    output reg  [15:0] out_r = 16'd0,
    // simulation instruments (the model replays these)
    output reg         dbg_step = 1'b0,     // a filter step took in_l / in_r this clock
    output reg         dbg_s48 = 1'b0       // the 48 kHz strobe (out_* change 3 clocks later)
);
`ifdef TANGO_HDMI_RETUNE
    wire       [31:0] CLK_HZ  = pll_native ? CLK_HZ_P : 32'd74_250_000;   // TANGO_HDMI_RETUNE: follows the PLL (changes only while clk is stopped)
`else
    localparam [31:0] CLK_HZ  = CLK_HZ_P;
`endif
    localparam [31:0] RATE_MAX = 32'd7_425_000;
    wire reset = rst | a_rst;

    //------------------------------------------------------------------ the step and 48 kHz strobes
    // racc adds the rate every clock, modulo CLK_HZ; a step when it wraps.  clk_pix timing (the framework's budget:
    // the first build with the clamp, the add and the compare in one clock missed 74.25 MHz by 0.97 ns): the clamped
    // rate and the clamped rate - CLK_HZ are registered (they change only in a filter load, which holds the reset),
    // so a clock does two 33-bit adds side by side and picks one by the second's sign.
    localparam [32:0] RATE_W0 = {1'b0, 32'd7_056_000} - {1'b0, CLK_HZ_P};
    reg  [31:0] rate_q = 32'd7_056_000;      // min(rate, RATE_MAX)
    reg  [32:0] rate_w = RATE_W0;            // rate_q - CLK_HZ (33-bit two's complement: negative)
    wire [31:0] rate_c = (rate > RATE_MAX) ? RATE_MAX : rate;
    always @(posedge clk) begin rate_q <= rate_c; rate_w <= {1'b0, rate_c} - {1'b0, CLK_HZ}; end
    reg  [31:0] racc = 32'd0;
    wire [32:0] racc_n = {1'b0, racc} + {1'b0, rate_q};     // racc + rate
    wire [32:0] racc_w = {1'b0, racc} + rate_w;             // racc + rate - CLK_HZ: bit 32 clear = it wrapped
    reg         step = 1'b0;
`ifdef TANGO_HDMI_RETUNE
    localparam integer SW = (S48_W > 14) ? S48_W : 14;
    wire [SW-1:0] SINC = pll_native ? SW'(S48_INC) : SW'(8);
    wire [SW-1:0] SMOD = pll_native ? SW'(S48_MOD) : SW'(12375);
`else
    localparam integer SW = S48_W;
    localparam [SW-1:0] SINC = S48_INC, SMOD = S48_MOD;
`endif
    reg  [SW-1:0] sacc = {SW{1'b0}};
    reg         s48 = 1'b0;
    always @(posedge clk) begin
        step <= 1'b0;
        if (!racc_w[32]) begin racc <= racc_w[31:0]; step <= 1'b1; end
        else racc <= racc_n[31:0];
        s48 <= 1'b0;
        if (sacc >= SMOD - SINC) begin sacc <= sacc + SINC - SMOD; s48 <= 1'b1; end
        else sacc <= sacc + SINC;
        if (reset) begin racc <= 32'd0; step <= 1'b0; end
        dbg_step <= step && !reset;
        dbg_s48 <= s48;
    end

    //------------------------------------------------------------------ the IIR pipeline (L at step+1, R one clock later)
    // The taps (tap0-2 per channel): six plain registers, no array.  A filter load must start the new filter from
    // zero, as MiSTer's does (sys_top.v 386-395: areset at every SET_AFILTER; 1580-1582: audio_out held in reset;
    // iir_filter.v 179-181: the taps cleared): a stable filter started from another filter's state can overflow its
    // 40-bit taps and lock into a limit cycle (run at 7 MHz, these low-passes have their poles within 0.02 of z = 1,
    // and their taps hold values far above the output's).  An earlier
    // build held them as 2-entry arrays (t0[0:1] ...), which GowinSynthesis extracted as RAM ("Extracting RAM for
    // identifier 't0'"), and on silicon the clear evidently did not happen: Audio filter Custom -> Internal gave a
    // full-scale 9.6 kHz (48 kHz / 5) oscillation carrying the test tone as sidebands, while every sim cleared them.
    reg  [39:0] tap0_l = 40'd0, tap0_r = 40'd0, tap1_l = 40'd0, tap1_r = 40'd0, tap2_l = 40'd0, tap2_r = 40'd0;
    reg  [15:0] il = 16'd0, ir = 16'd0;
    reg         v1 = 1'b0, v1r = 1'b0;               // issue L, then R
    always @(posedge clk) begin
        v1 <= step && !reset;
        v1r <= v1;
        if (step) begin il <= in_l; ir <= in_r; end
    end
    // S1: operands of the channel being issued
    reg         s1_v = 1'b0, s1_c = 1'b0;
    reg  [15:0] s1_in = 16'd0;
    reg  [39:0] s1_t0 = 40'd0, s1_t1 = 40'd0, s1_t2 = 40'd0;
    always @(posedge clk) begin
        s1_v <= v1 | v1r;  s1_c <= v1r;
        s1_in <= v1r ? ir : il;
        s1_t0 <= v1r ? tap0_r : tap0_l;  s1_t1 <= v1r ? tap1_r : tap1_l;  s1_t2 <= v1r ? tap2_r : tap2_l;
    end
    // S2: in * cx (16 x 40 signed)
    reg         s2_v = 1'b0, s2_c = 1'b0;
    reg  signed [55:0] s2_p = 56'sd0;
    reg  [39:0] s2_t0 = 40'd0, s2_t1 = 40'd0, s2_t2 = 40'd0;
    always @(posedge clk) begin
        s2_v <= s1_v; s2_c <= s1_c;
        s2_p <= $signed(s1_in) * $signed(cx);
        s2_t0 <= s1_t0; s2_t1 <= s1_t1; s2_t2 <= s1_t2;
    end
    // S3: x = p >>> 20, y = x + t0
    reg         s3_v = 1'b0, s3_c = 1'b0;
    reg  [39:0] s3_x = 40'd0, s3_y = 40'd0, s3_t1 = 40'd0, s3_t2 = 40'd0;
    wire [39:0] x_w = s2_p[55:20] | {{4{s2_p[55]}}, 36'd0};      // bits 59:20 of the 60-bit product
    always @(posedge clk) begin
        s3_v <= s2_v; s3_c <= s2_c;
        s3_x <= x_w;
        s3_y <= x_w + s2_t0;
        s3_t1 <= s2_t1; s3_t2 <= s2_t2;
    end
    // S4: y[36:0] * cy0..2 (37 x 24 signed), the x scales
    function [39:0] xs(input [7:0] c, input [39:0] x);
        reg [39:0] a;
        begin
            a = (c[0] ? {{4{x[39]}}, x[39:4]} : 40'd0) + (c[1] ? {{3{x[39]}}, x[39:3]} : 40'd0)
              + (c[2] ? {{2{x[39]}}, x[39:2]} : 40'd0);
            xs = c[7] ? ~a : a;
        end
    endfunction
    reg         s4_v = 1'b0, s4_c = 1'b0;
    reg  signed [60:0] s4_m0 = 61'sd0, s4_m1 = 61'sd0, s4_m2 = 61'sd0;
    reg  [39:0] s4_x0 = 40'd0, s4_x1 = 40'd0, s4_x2 = 40'd0, s4_y = 40'd0, s4_t1 = 40'd0, s4_t2 = 40'd0;
    always @(posedge clk) begin
        s4_v <= s3_v; s4_c <= s3_c;
        s4_m0 <= $signed(s3_y[36:0]) * $signed(cy0);
        s4_m1 <= $signed(s3_y[36:0]) * $signed(cy1);
        s4_m2 <= $signed(s3_y[36:0]) * $signed(cy2);
        s4_x0 <= xs(cx0, s3_x); s4_x1 <= xs(cx1, s3_x); s4_x2 <= xs(cx2, s3_x);
        s4_y <= s3_y; s4_t1 <= s3_t1; s4_t2 <= s3_t2;
    end
    // S5: the taps' new values, the output
    wire [39:0] n0 = s4_x0 - s4_m0[60:21] + s4_t1;
    wire [39:0] n1 = s4_x1 - s4_m1[60:21] + s4_t2;
    wire [39:0] n2 = s4_x2 - s4_m2[60:21];
    wire [15:0] yc = (!s4_y[39] && |s4_y[38:35]) ? 16'h7FFF :
                     ( s4_y[39] && ~&s4_y[38:35]) ? 16'h8000 : s4_y[35:20];
    reg  [15:0] y_l = 16'd0, y_r = 16'd0;            // the last outputs
    always @(posedge clk) begin
        if (reset) begin
            tap0_l <= 40'd0; tap1_l <= 40'd0; tap2_l <= 40'd0;
            tap0_r <= 40'd0; tap1_r <= 40'd0; tap2_r <= 40'd0;
            y_l <= 16'd0; y_r <= 16'd0;
        end else if (s4_v) begin
            if (s4_c) begin tap0_r <= n0; tap1_r <= n1; tap2_r <= n2; y_r <= yc; end
            else      begin tap0_l <= n0; tap1_l <= n1; tap2_l <= n2; y_l <= yc; end
        end
    end

    //------------------------------------------------------------------ 48 kHz: take, DC blocker, mute
    reg  [15:0] h_l = 16'd0, h_r = 16'd0;            // taken at the strobe (the blocker reads the one before)
    reg  [15:0] dc_l = 16'd0, dc_r = 16'd0;          // the DC blocker's output (MiSTer DC_blocker dout), muted
    reg  [39:0] dx1_l = 40'd0, dx1_r = 40'd0, dy_l = 40'd0, dy_r = 40'd0;
    reg  [39:0] a_x0l = 40'd0, a_x0r = 40'd0, a_y1l = 40'd0, a_y1r = 40'd0;
    reg         d1 = 1'b0, d2 = 1'b0;
    reg  [13:0] mute_n = 14'd0;                      // strobes since the reset (to 8192)
    function [39:0] sat39(input [39:0] v);           // keep v in 39 bits signed (iir_filter.v DC_blocker)
        sat39 = (v[39] != v[38]) ? {v[39], v[39], {38{v[38]}}} : v;
    endfunction
    wire [39:0] xl = {h_l[15], h_l, 23'd0}, xr = {h_r[15], h_r, 23'd0};
    always @(posedge clk) begin
        d1 <= s48; d2 <= d1;
        if (s48) begin h_l <= y_l; h_r <= y_r; end
        // stage A (the clock after the strobe, h_* = the value taken at the strobe before)
        if (s48) begin
            a_x0l <= xl - {{10{xl[39]}}, xl[39:10]};  a_x0r <= xr - {{10{xr[39]}}, xr[39:10]};
            a_y1l <= dy_l - {{9{dy_l[39]}}, dy_l[39:9]};  a_y1r <= dy_r - {{9{dy_r[39]}}, dy_r[39:9]};
        end
        // stage B
        if (d1) begin
            dx1_l <= a_x0l;  dx1_r <= a_x0r;
            dy_l <= sat39(a_x0l - dx1_l + a_y1l);
            dy_r <= sat39(a_x0r - dx1_r + a_y1r);
        end
        if (d2) begin
            dc_l <= mute_n[13] ? dy_l[38:23] : 16'd0;
            dc_r <= mute_n[13] ? dy_r[38:23] : 16'd0;
            if (!mute_n[13]) mute_n <= mute_n + 14'd1;
        end
        if (reset) mute_n <= 14'd0;
    end

    //------------------------------------------------------------------ aud_mix_top (audio_out.sv 281-345)
    // Every stage moves on the 48 kHz strobe that takes a new DC-blocker sample, and reads that sample's predecessor
    // (MiSTer's stages and its DC blocker share sample_ce).  boost: below x the magnitude is shifted up by a, above it
    // compressed, (v - x) >> f + b; x, b = MiSTer's localparams for (f, a) = (4, 2) and (8, 4).
    reg  [1:0]  mix_s1 = 2'd0, mix_q = 2'd0;          // AUDIO_MIX, synchronised
    always @(posedge clk) begin mix_s1 <= mix; mix_q <= mix_s1; end
    // (Plain unsigned vectors with the sign extensions written out, and no shift by a variable amount: GowinSynthesis
    // V1.9.11.03 stops with 80 x "error bus name set" on `x >> att[3:0]` / `>>>` -- bisected -- so every
    // shift here is a fixed slice chosen by a mux.  The same bits.)
    localparam [15:0] BX1 = 16'd14044, BB1 = 16'd28088;   // boost_x1 / b1 (f1 4, a1 2): ((32767*3)/7)+1, x1*2
    localparam [15:0] BX2 = 16'd7400,  BB2 = 16'd29600;   // boost_x2 / b2 (f2 8, a2 4): ((32767*7)/31)+1, x2*4
    wire [15:0] bx;  assign bx = boost[1] ? BX2 : BX1;
    wire [15:0] bb;  assign bb = boost[1] ? BB2 : BB1;
    reg  [15:0] v0_l = 16'd0, v0_r = 16'd0, v1_l = 16'd0, v1_r = 16'd0;
    reg         s0_l = 1'b0, s0_r = 1'b0, s1_l = 1'b0, s1_r = 1'b0;
    reg  [15:0] a1_l = 16'd0, a1_r = 16'd0, pre_l = 16'd0, pre_r = 16'd0;   // two's complement
    reg  [16:0] a2_l = 17'd0, a2_r = 17'd0, a3_l = 17'd0, a3_r = 17'd0, a4_l = 17'd0, a4_r = 17'd0;
    // the mix: 1 = a2 - a2>>>3 + pre_other>>>2, 2 = a2 - a2>>>2 + pre_other>>>1, 3 = a2>>>1 + pre_other (17 bits)
    wire [16:0] m1_l = a2_l - {{3{a2_l[16]}}, a2_l[16:3]} + {{3{pre_r[15]}}, pre_r[15:2]};
    wire [16:0] m2_l = a2_l - {{2{a2_l[16]}}, a2_l[16:2]} + {{2{pre_r[15]}}, pre_r[15:1]};
    wire [16:0] m3_l = {a2_l[16], a2_l[16:1]} + {pre_r[15], pre_r};
    wire [16:0] m1_r = a2_r - {{3{a2_r[16]}}, a2_r[16:3]} + {{3{pre_l[15]}}, pre_l[15:2]};
    wire [16:0] m2_r = a2_r - {{2{a2_r[16]}}, a2_r[16:2]} + {{2{pre_l[15]}}, pre_l[15:1]};
    wire [16:0] m3_r = {a2_r[16], a2_r[16:1]} + {pre_l[15], pre_l};
    // boost: v0 << a (a = 2 or 4) below x, else ((v0 - x) >> f) + b (f = 4 or 8)
    wire [15:0] vd_l = v0_l - bx, vd_r = v0_r - bx;
    wire [15:0] vb_l = (v0_l < bx) ? (boost[1] ? {v0_l[13:0], 2'b00} : {v0_l[14:0], 1'b0})
                                   : ((boost[1] ? {3'b000, vd_l[15:3]} : {2'b00, vd_l[15:2]}) + bb);
    wire [15:0] vb_r = (v0_r < bx) ? (boost[1] ? {v0_r[13:0], 2'b00} : {v0_r[14:0], 1'b0})
                                   : ((boost[1] ? {3'b000, vd_r[15:3]} : {2'b00, vd_r[15:2]}) + bb);
    // a3 >>> att[3:0]: shifts by 1, 2, 4, 8, each a sign-extended slice
    wire [16:0] t1_l = att[0] ? {a3_l[16], a3_l[16:1]} : a3_l;
    wire [16:0] t2_l = att[1] ? {{2{t1_l[16]}}, t1_l[16:2]} : t1_l;
    wire [16:0] t4_l = att[2] ? {{4{t2_l[16]}}, t2_l[16:4]} : t2_l;
    wire [16:0] t8_l = att[3] ? {{8{t4_l[16]}}, t4_l[16:8]} : t4_l;
    wire [16:0] t1_r = att[0] ? {a3_r[16], a3_r[16:1]} : a3_r;
    wire [16:0] t2_r = att[1] ? {{2{t1_r[16]}}, t1_r[16:2]} : t1_r;
    wire [16:0] t4_r = att[2] ? {{4{t2_r[16]}}, t2_r[16:4]} : t2_r;
    wire [16:0] t8_r = att[3] ? {{8{t4_r[16]}}, t4_r[16:8]} : t4_r;
    always @(posedge clk) if (d2) begin
        v0_l <= dc_l[15] ? (~dc_l) + 16'd1 : dc_l;       s0_l <= dc_l[15];
        v0_r <= dc_r[15] ? (~dc_r) + 16'd1 : dc_r;       s0_r <= dc_r[15];
        v1_l <= vb_l;   s1_l <= s0_l;
        v1_r <= vb_r;   s1_r <= s0_r;
        a1_l <= (boost != 2'd0) ? (s1_l ? ~(v1_l - 16'd1) : v1_l) : dc_l;
        a1_r <= (boost != 2'd0) ? (s1_r ? ~(v1_r - 16'd1) : v1_r) : dc_r;
        a2_l <= {a1_l[15], a1_l};                        // + Linux audio: none
        a2_r <= {a1_r[15], a1_r};
        pre_l <= a2_l[16:1];  pre_r <= a2_r[16:1];
        a3_l <= (mix_q == 2'd0) ? a2_l : (mix_q == 2'd1) ? m1_l : (mix_q == 2'd2) ? m2_l : m3_l;
        a3_r <= (mix_q == 2'd0) ? a2_r : (mix_q == 2'd1) ? m1_r : (mix_q == 2'd2) ? m2_r : m3_r;
        a4_l <= att[4] ? 17'd0 : t8_l;
        a4_r <= att[4] ? 17'd0 : t8_r;
        out_l <= (a4_l[16] ^ a4_l[15]) ? {a4_l[16], {15{a4_l[15]}}} : a4_l[15:0];
        out_r <= (a4_r[16] ^ a4_r[15]) ? {a4_r[16], {15{a4_r[15]}}} : a4_r[15:0];
    end
endmodule
`default_nettype wire
