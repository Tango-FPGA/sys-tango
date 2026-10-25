// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_vsync -- native-rate HDMI output (README "Native-rate video (vsync_adjust)"; MiSTer's vsync_adjust = 1 by the
// output-totals lever): it MEASURES the core's frame period in clk_pix, PLANS the HDMI raster's totals H x V closest to
// it at the unchanged 74.25 MHz pixel clock, and hands them to every consumer of the raster for the next frame.
// TANGO_VSYNC_ADJUST builds only (tango_cfg.vh).  Our own code; the measuring follows MiSTer's hps_io.sv 948-999 (the
// rate is measured in the FPGA, not declared by the core) and the planning MiSTer's video_mode_adjust (video.cpp
// 2631-2714: act on a change of the core's mode, not continuously; refuse outside refresh_min / refresh_max).
//
// Measuring (clk_pix): the core's VGA_VS (its own clock) through three flops; a frame's period = the clocks between two
// rises.  A window of 2^NWIN_LOG2 frames counts when every frame is within +-TOL of the frame TWO before it (an
// alternating pattern -- interlaced fields, the NES's odd frame -- passes; a change of the frame length, e.g. the TG16's
// 262 <-> 263 lines = 4719 clk_pix, restarts the window); P = the window's sum / 2^NWIN_LOG2, rounded.  No rise for
// 2^21 clocks (28 ms): no signal -- the plan in use stays.
// Planning (a new P that moved more than REPLAN from the planned one): the H x V closest to P over H HMIN..HMAX, V
// VMIN..VMAX by two pointers (H only falls as V rises: ~ (HMAX-HMIN)+(VMIN-VMAX) steps of one add each, no multiplier,
// no divider); for each V the H with H x V just below P and the next one up are candidates.  The plan is used if
// |H x V - P| <= P / 4096 (244 ppm) and P_min <= H x V <= P_max (SYS 0x70's bounds, 0 = none); else the fixed raster.
// Handing over: the totals for the next frame (nxt_w / nxt_h) change only at hdmi.sv's beam (0, COMMIT_Y), mid-frame,
// so every consumer -- each at its own frame boundary, all within a few dozen clocks of hdmi.sv's -- takes the same pair.
// SYS 0x70 VSYNC_ADJUST (tango_link TAP): n0 = {apply, 6'b0, native}: apply = 1 takes the
// mode and the bounds, 0 is a probe; n1-n3 P_min, n4-n6 P_max (24-bit, big-endian, clk_pix of the OUTPUT frame;
// 0 = none).  Reply (byte 3, tango_link): status = {1 present, mode, active, meas_ok, rejected, 3'b000}.
// TANGO_HDMI_RETUNE: the mode also selects the HDMI PLL -- native = the build's preset
// (e.g. 74.5875 MHz), fixed = 74.25 MHz (fixed raster 1650 x 750 = exactly 60.000 Hz) -- through tango_pll_retune (50 MHz),
// as a CLOCK PAUSE (this domain is not reset; its clock stops for the ~18 ms re-lock and resumes at the new frequency).  An
// apply whose native bit differs from the last request makes the request pending; it goes out (rt_tog toggles, rt_native =
// the request) when hdmi.sv's beam reaches (0, TRIG_Y) -- the vertical blanking, where ascal has no read outstanding.  The
// fixed raster is fix_w x fix_h (the PLL's setting now, quasi-static).  Reply bit 2 = a retune is pending or running (the
// request differs from the PLL's setting), bit 1 = this build can retune: the firmware does not talk to the FPGA for ~60 ms
// after an apply, then probes until bit 2 is 0.
//==============================================================================
`default_nettype none
module tango_vsync #(
    parameter integer HT0 = 1650, VT0 = 750,      // the fixed raster: mode 0, no plan yet, a plan refused
    parameter integer HMIN = 1650, HMAX = 1720,   // the search (H >= 1650 keeps ascal's
    parameter integer VMIN = 740, VMAX = 790,     //   per-line DDR3 deadline; V >= 740 its preload)
    parameter integer MODE0 = 1,                  // the mode after reset (TANGO_VSYNC_DEFAULT): 1 native, 0 fixed
    parameter integer NWIN_LOG2 = 4,              // 16 frames a window
    parameter integer TOL = 16,                   // clk_pix
    parameter integer REPLAN = 8,                 // clk_pix
    parameter integer COMMIT_Y = 360,
    parameter integer TRIG_Y = 730                // TANGO_HDMI_RETUNE: the line a PLL retune starts on
) (
    input  wire        clk,                       // clk_pix
    input  wire        rst,
    input  wire        core_vs,                   // the core's VGA_VS (CLK_VIDEO)
    input  wire [11:0] cx,                        // hdmi.sv's beam
    input  wire [10:0] cy,
    // SYS bytes (tango_link TANGO_LINK_TAP)
    input  wire        tap_cmd_stb,
    input  wire        tap_stb,
    input  wire [7:0]  tap_cmd,
    input  wire [15:0] tap_n,
    input  wire [7:0]  tap_byte,
    input  wire        tap_end,
    output wire [7:0]  status,                    // SYS 0x70's reply
`ifdef TANGO_HDMI_RETUNE
    input  wire        pll_native,                // tango_pll_retune: the PLL's setting now (quasi-static)
    input  wire [11:0] fix_w,                     // the fixed raster at that setting (quasi-static)
    input  wire [10:0] fix_h,
    output reg         rt_tog = 1'b0,             // to tango_pll_retune: a new request (never reset)
    output reg         rt_native = (MODE0 != 0),  //   the requested setting
`endif
    output reg  [11:0] nxt_w = HT0,               // the raster's totals for the next frame (HT0 x VT0 from power-up)
    output reg  [10:0] nxt_h = VT0,
    // diagnostics
    output reg  [21:0] meas_p = 22'd0,            // the last good window's frame period (clk_pix)
    output reg         meas_ok = 1'b0,            // a window is good and no timeout since
    output reg  [11:0] plan_w = HT0,              // the last plan
    output reg  [10:0] plan_h = VT0,
    output reg  [21:0] plan_err = 22'd0           // |plan_w x plan_h - P| of it
);
    localparam integer NWIN = 1 << NWIN_LOG2;
    localparam [21:0] PROD0 = HMAX * VMIN;        // the search's first product
    localparam [11:0] HT0W = HT0, HMINW = HMIN, HMAXW = HMAX;
    localparam [10:0] VT0W = VT0, VMINW = VMIN, VMAXW = VMAX, CYW = COMMIT_Y;
    localparam [21:0] TOLW = TOL, REPLANW = REPLAN;
`ifdef TANGO_HDMI_RETUNE
    wire [11:0] fw0 = fix_w;  wire [10:0] fh0 = fix_h;
    // a retune changed the clock under this domain (pll_native flips while the clock is stopped): every period measured and
    // every plan is in the old clock's units -- drop them (srst: the measuring, the planning and the hand-over restart, the
    // fixed raster of the new setting from the next frame on); SYS 0x70's mode and bounds stay
    reg         pn_q = (MODE0 != 0);
    always @(posedge clk) pn_q <= pll_native;
    wire        srst = rst || (pn_q != pll_native);
`else
    wire [11:0] fw0 = HT0W;   wire [10:0] fh0 = VT0W;
    wire        srst = rst;
`endif

    //------------------------------------------------------------------ SYS 0x70
    reg         mode = (MODE0 != 0);
    reg  [23:0] pmin = 24'd0, pmax = 24'd0;
    reg         s_in = 1'b0, s_apply = 1'b0, s_native = 1'b0;
    reg  [23:0] s_pmin = 24'd0, s_pmax = 24'd0;
    reg         cfg_tog = 1'b0;                   // a new mode / bounds: re-plan
`ifdef TANGO_HDMI_RETUNE
    reg         rt_want = (MODE0 != 0);           // the PLL setting the firmware asked for
    reg         rt_pend = 1'b0;                   // it differs from what was last sent to tango_pll_retune
`endif
    always @(posedge clk) begin
        if (tap_cmd_stb) begin s_in <= (tap_cmd == 8'h70); s_apply <= 1'b0; end
        if (tap_stb && s_in) begin
            case (tap_n)
            16'd0: begin s_apply <= tap_byte[7]; s_native <= tap_byte[0]; end
            16'd1: s_pmin[23:16] <= tap_byte;  16'd2: s_pmin[15:8] <= tap_byte;  16'd3: s_pmin[7:0] <= tap_byte;
            16'd4: s_pmax[23:16] <= tap_byte;  16'd5: s_pmax[15:8] <= tap_byte;  16'd6: s_pmax[7:0] <= tap_byte;
            default: ;
            endcase
        end
        if (rst) begin
            mode <= (MODE0 != 0); pmin <= 24'd0; pmax <= 24'd0;
            s_in <= 1'b0; s_apply <= 1'b0;
        end else if (tap_end && s_in) begin
            s_in <= 1'b0;
            if (s_apply) begin
                mode <= s_native; pmin <= s_pmin; pmax <= s_pmax; cfg_tog <= ~cfg_tog;
`ifdef TANGO_HDMI_RETUNE
                if (s_native != rt_want) begin rt_want <= s_native; rt_pend <= 1'b1; end
`endif
            end
        end
`ifdef TANGO_HDMI_RETUNE
        if (rt_pend && cx == 12'd0 && cy == 11'(TRIG_Y)) begin rt_native <= rt_want; rt_tog <= ~rt_tog; rt_pend <= 1'b0; end
`endif
    end

    //------------------------------------------------------------------ measuring
    reg  [2:0]  vs_s = 3'b000;
    always @(posedge clk) vs_s <= {vs_s[1:0], core_vs};
    wire        vs_rise = vs_s[1] && !vs_s[2];
    reg  [21:0] cnt = 22'd0;
    reg         armed = 1'b0;                     // a rise has been seen since reset / the last timeout
    reg  [21:0] p1 = 22'd0, p2 = 22'd0;           // the periods one and two frames before
    reg  [NWIN_LOG2:0] nfr = 0;                   // frames in the window so far
    reg  [25:0] sum = 26'd0;
    reg         meas_stb = 1'b0;
    // clk_pix timing (a single-clock version missed 74.25 MHz by ~1 ns here and in the planner): no add feeds a compare in
    // the same clock -- the period (cnt + 1) is its own counter per_r, the stability window p2 +- TOL is registered when p2
    // is, and a window's average is taken one clock after its sum (meas_stb one clock later than before; nothing waits on it)
    reg  [21:0] per_r = 22'd1;                    // = cnt + 1
    reg  [21:0] p1_hi = 22'd0, p1_lo = 22'd0, p2_hi = 22'd0, p2_lo = 22'd0;   // p1 / p2 +- TOL (lo saturated at 0)
    reg  [25:0] sum_fin = 26'd0;
    reg         fin = 1'b0;
    wire        stable = (nfr < 2) || ((per_r <= p2_hi) && (per_r >= p2_lo));
    always @(posedge clk) begin
        meas_stb <= 1'b0;
        fin <= 1'b0;
        if (fin) begin                            // the window's average, from the sum finished one clock ago
            meas_p   <= 22'((sum_fin + (NWIN / 2)) >> NWIN_LOG2);
            meas_ok  <= 1'b1;
            meas_stb <= 1'b1;
        end
        if (srst) begin
            cnt <= 22'd0; per_r <= 22'd1; armed <= 1'b0; nfr <= 0; sum <= 26'd0; meas_ok <= 1'b0; fin <= 1'b0;
            meas_stb <= 1'b0;
        end else if (vs_rise) begin
            cnt <= 22'd0;  per_r <= 22'd1;
            armed <= 1'b1;
            if (armed) begin
                p2 <= p1;  p1 <= per_r;
                p2_hi <= p1_hi;  p2_lo <= p1_lo;
                p1_hi <= per_r + TOLW;  p1_lo <= (per_r > TOLW) ? per_r - TOLW : 22'd0;
                if (!stable) begin nfr <= 1; sum <= {4'd0, per_r}; end
                else if (nfr == NWIN - 1) begin
                    sum_fin <= sum + {4'd0, per_r};  fin <= 1'b1;
                    nfr <= 0;  sum <= 26'd0;
                end else begin nfr <= nfr + 1'b1; sum <= sum + {4'd0, per_r}; end
            end
        end else if (cnt[21]) begin               // 2^21 clocks without a rise: no signal
            armed <= 1'b0; nfr <= 0; sum <= 26'd0; meas_ok <= 1'b0; fin <= 1'b0; meas_stb <= 1'b0;
        end else begin cnt <= cnt + 22'd1; per_r <= per_r + 22'd1; end
    end

    //------------------------------------------------------------------ planning
    localparam [2:0] P_IDLE = 3'd0, P_DOWN = 3'd1, P_EVAL = 3'd2, P_EVAL2 = 3'd3, P_NEXT = 3'd4, P_DONE = 3'd5, P_EVAL3 = 3'd6;
    reg  [2:0]  ps = P_IDLE;
    reg  [21:0] tp = 22'd0;                       // the P being planned
    reg  [21:0] planned_p = 22'd0;
    reg         have_plan = 1'b0, plan_ok = 1'b0, rejected = 1'b0;
    reg  [11:0] h = 12'd0;  reg [10:0] v = 11'd0;
    reg  [21:0] prod = 22'd0;
    reg  [11:0] bh = 12'd0; reg [10:0] bv = 11'd0;
    reg  [21:0] be = 22'h3FFFFF, bp = 22'd0;      // the best error and its product
    reg         cfg_seen = 1'b0;
    // clk_pix timing: one carry chain per clock -- |H x V - P| and (H + 1) x V registered in P_EVAL, compared in P_EVAL2,
    // |(H + 1) x V - P| registered in P_EVAL2, compared in P_EVAL3; the re-plan window planned_p +- REPLAN registered
    reg  [21:0] dl_q = 22'd0, pu_q = 22'd0, du_q = 22'd0;
    reg  [21:0] planned_hi = 22'd0, planned_lo = 22'd0;
    wire        moved = !have_plan || (meas_p > planned_hi) || (meas_p < planned_lo);
    always @(posedge clk) begin
        if (srst) begin
            ps <= P_IDLE; have_plan <= 1'b0; plan_ok <= 1'b0; rejected <= 1'b0; cfg_seen <= cfg_tog;
            plan_w <= fw0; plan_h <= fh0;
        end else case (ps)
        P_IDLE:
            if ((meas_stb && moved) || (meas_ok && cfg_seen != cfg_tog)) begin
                cfg_seen <= cfg_tog;
                tp <= meas_p;  h <= HMAXW;  v <= VMINW;  prod <= PROD0;  be <= 22'h3FFFFF;
                ps <= P_DOWN;
            end
        P_DOWN:                                   // H down while H x V > P
            if (prod > tp && h > HMINW) begin h <= h - 12'd1; prod <= prod - {11'd0, v}; end
            else ps <= P_EVAL;
        P_EVAL: begin                             // |H x V - P| and (H + 1) x V
            dl_q <= (prod > tp) ? prod - tp : tp - prod;
            pu_q <= prod + {11'd0, v};
            ps <= P_EVAL2;
        end
        P_EVAL2: begin                            // the candidate H x V; |(H + 1) x V - P|
            if (dl_q < be) begin be <= dl_q; bh <= h; bv <= v; bp <= prod; end
            du_q <= (pu_q > tp) ? pu_q - tp : tp - pu_q;
            ps <= P_EVAL3;
        end
        P_EVAL3: begin                            // and (H + 1) x V
            if (h < HMAXW && du_q < be) begin be <= du_q; bh <= h + 12'd1; bv <= v; bp <= pu_q; end
            ps <= P_NEXT;
        end
        P_NEXT:
            if (v == VMAXW) ps <= P_DONE;
            else begin v <= v + 11'd1; prod <= prod + {10'd0, h}; ps <= P_DOWN; end
        P_DONE: begin
            have_plan <= 1'b1;  planned_p <= tp;
            planned_hi <= tp + REPLANW;  planned_lo <= (tp > REPLANW) ? tp - REPLANW : 22'd0;
            plan_w <= bh;  plan_h <= bv;  plan_err <= be;
            plan_ok <= (be <= (tp >> 12)) && (pmin == 24'd0 || {2'd0, bp} >= pmin) && (pmax == 24'd0 || {2'd0, bp} <= pmax);
            rejected <= !((be <= (tp >> 12)) && (pmin == 24'd0 || {2'd0, bp} >= pmin) && (pmax == 24'd0 || {2'd0, bp} <= pmax));
            ps <= P_IDLE;
        end
        default: ps <= P_IDLE;
        endcase
    end

    //------------------------------------------------------------------ handing over (hdmi.sv's beam, mid-frame)
    wire active = mode && have_plan && plan_ok;
    always @(posedge clk)
        if (srst) begin nxt_w <= fw0; nxt_h <= fh0; end
        else if (cx == 12'd0 && cy == CYW) begin
            nxt_w <= active ? plan_w : fw0;
            nxt_h <= active ? plan_h : fh0;
        end
`ifdef TANGO_HDMI_RETUNE
    assign status = {1'b1, mode, active, meas_ok, rejected, rt_want != pll_native, 1'b1, 1'b0};
`else
    assign status = {1'b1, mode, active, meas_ok, rejected, 3'b000};
`endif
endmodule
`default_nettype wire
