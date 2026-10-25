// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// ascal_wrap_sim -- Verilator-only behavioral replacement for ascal_wrap.v.
//
// Drop-in replacement for simulation when compiled with -DTANGO_SIM.
// The real ascal_wrap instantiates the VHDL entity ascal.vhd which Verilator
// cannot compile.  This module:
//   - buffers one complete input frame (from CLK_VIDEO / i_clk) in a Verilog
//     memory array (double-buffered; no SDRAM / Avalon transfers);
//   - generates a free-running 720p output beam (htotal/vtotal parameters);
//   - for each output pixel inside the active window (hmin..hmax, vmin..vmax)
//     applies nearest-neighbour scaling, formula identical to tb_video_golden.py:
//       sx = ((2*ox+1)*iw - 1) / (2*ow)
//       dl = ((2*oy+1)*2*ih - 1) / (2*oh)   sy = dl>>1   (no voff: iauto=1)
//   - drives all Avalon outputs to zero (SDRAM not exercised in unit sim);
//   - exposes four sim-only debug ports used by tango_video.v under TANGO_SIM.
//
// Crop (voff != 0): fully simulated via i_voff / i_ih ports from tango_video_geo.  These are
// latched into fb_voff / fb_ih at VS+1 to avoid a geometry-at-VS race (Fix M + Fix P).
//
// Maximum frame size: SIMW x SIMH (passed from tango_video's MAXW/MAXH).
//   The internal BSRAM is fixed at 512 x 512 = 262144 words (3 MB per bank).
//==============================================================================
`default_nettype none
module ascal_wrap_sim #(
    parameter integer SIMW    = 256,          // max input width  (= MAXW)
    parameter integer SIMH    = 240,          // max input height (= MAXH)
    parameter [31:0]  RAMBASE = 32'h0,        // unused in sim
    parameter [31:0]  RAMSIZE = 32'h0         // unused in sim
) (
    // ---- core video input (i_clk = CLK_VIDEO domain)
    input  wire        i_clk,
    input  wire        i_ce,
    input  wire [7:0]  i_r, i_g, i_b,
    input  wire        i_hs, i_vs, i_de,
    input  wire        i_fl,
    // ---- output video (o_clk = clk_pix domain)
    input  wire        o_clk,
    input  wire        o_ce,          // unused (output runs free)
    output wire [7:0]  o_r, o_g, o_b,
    output wire        o_hs, o_vs, o_de,
    // ---- geometry window (quasi-static, o_clk domain)
    input  wire [11:0] hmin, hmax,
    input  wire [10:0] vmin, vmax,
    // ---- 720p output timing
    input  wire [11:0] htotal, hsstart, hsend, hdisp,
    input  wire [11:0] vtotal, vsstart, vsend, vdisp,
    // ---- scaler mode (unused)
    input  wire [4:0]  mode,
    // ---- freeze: when set, suppress the frame commit at VS so the same frame repeats on output
    input  wire        i_freeze,
    // ---- Avalon (all outputs driven to 0; inputs ignored)
    input  wire        avl_clk,
    output wire [28:0] avl_address,
    output wire        avl_write,
    output wire [63:0] avl_writedata,
    output wire        avl_read,
    output wire [7:0]  avl_burstcount,
    output wire [7:0]  avl_byteenable,
    input  wire [63:0] avl_readdata,
    input  wire        avl_readdatavalid,
    input  wire        avl_waitrequest,
    // ---- active-low async reset
    input  wire        reset_na,
    // ---- sim-only debug ports (connected in tango_video.v under TANGO_SIM)
    output wire [15:0] sim_dbg_frames,   // total input frames committed (i_clk domain)
    output wire [15:0] sim_show_seq,     // live CDC seq (rs_s1); used for T-log staleness check
    output wire [15:0] sim_show_rseq,    // frame-locked seq (rseq_r); consistent with rbank; used for F-log f_seq
    output wire        sim_show_valid,   // read buffer has valid data   (o_clk domain)
    output wire        sim_show_take,    // pulses when reader gets a new frame (o_clk domain)
    output wire        sim_dl_parity,    // dl[0]: doubled-line row parity for scanline dimming
    output wire        sim_frame_end,    // pulses for one clk_pix cycle at last pixel (htotal-1, vtotal-1)
    // ---- crop geometry (from tango_video_geo; quasi-static, o_clk domain)
    input  wire [11:0] sim_cx,           // tango_video's beam (cx, cy): the output is for the beam of the same clock,
    input  wire [10:0] sim_cy,           // as the real ascal's is once tango_video has aligned its sweep (ce_hold)
    input  wire [11:0] i_voff,           // first source line shown (crop offset)
    input  wire [11:0] i_ih,             // cropped source height fed to scaler
    // ---- debug outputs: Avalon handshake monitoring (tied off in sim)
    output wire [31:0] dbg_read_count,
    output wire [31:0] dbg_readdata_valid_count,
    output wire [31:0] dbg_write_count,
    output wire [28:0] dbg_last_address,
    output wire [63:0] dbg_last_readdata,
    output wire        dbg_waitrequest_stuck,
    output wire        dbg_avl_read,
    output wire        dbg_avl_readdatavalid,
    output wire        dbg_avl_waitrequest
);

    // ---- Avalon: all zero
    assign avl_address    = 29'd0;
    assign avl_write      = 1'b0;
    assign avl_writedata  = 64'd0;
    assign avl_read       = 1'b0;
    assign avl_burstcount = 8'd0;
    assign avl_byteenable = 8'd0;

    // ----  quad-buffered frame store (fixed 512×512 to avoid parameterised-array issues)
    // 4 banks are required: at 66 Hz input / 60 Hz output, two input VSes can fire during one output
    // frame period.  With 3 banks: (W+2) ≡ (W-1) mod 3 always, so 2 advances always aliases read.
    // With 4 banks: (W+2) mod 4 ≠ (W-1) mod 4, so 2 advances can never alias the read bank.
    localparam integer FBSZ = 512 * 512;   // 262 144 entries × 24 bits ≈ 3 MB / bank
    reg [23:0] fbuf0 [0:FBSZ-1];
    reg [23:0] fbuf1 [0:FBSZ-1];
    reg [23:0] fbuf2 [0:FBSZ-1];
    reg [23:0] fbuf3 [0:FBSZ-1];

    // ---- write side (i_clk, gated by i_ce) --------------------------------
    reg [9:0]  wx = 10'd0, wy = 10'd0;   // write x/y within active frame
    reg [9:0]  meas_w = 10'd0;           // measured active width (first line only)
    reg        i_vs_r = 1'b0;            // previous VS to detect rise
    reg [1:0]  wbank  = 2'd0;            // current write bank (0→fbuf0, 1→fbuf1, 2→fbuf2, 3→fbuf3)
    reg [9:0]  rbuf_iw = 10'd0, rbuf_ih = 10'd0;  // committed frame dimensions
    reg        rbuf_valid = 1'b0;        // at least one complete frame committed
    reg [15:0] rbuf_seq   = 16'd0;       // seq of most-recently committed frame
    reg [15:0] frame_cnt  = 16'd0;       // total VS pulses seen

    // Flat write index: wy * SIMW + wx (18 bits required: SIMW * SIMH <= FBSZ = 512 * 512, e.g. 640 x 360)
    wire [17:0] widx = {8'b0, wy} * SIMW[17:0] + {8'b0, wx};

    always @(posedge i_clk) begin
        if (i_ce) begin
            i_vs_r <= i_vs;

            if (!reset_na) begin
                wx <= 10'd0; wy <= 10'd0; meas_w <= 10'd0;
                wbank <= 2'd0; rbuf_valid <= 1'b0;
                rbuf_seq <= 16'd0; frame_cnt <= 16'd0;
            end else begin
                if (i_de) begin
                    // store pixel into write bank (widx is combinational from current wx/wy); a line is SIMW wide at
                    // most (it was capped at 511 pixels, so a 640-wide input -- the launcher's 640 x 360 -- measured
                    // 511 and showed scaled x2.5; the pixels past SIMW are dropped, not written into the next row)
                    if (wx < SIMW[9:0]) begin
                        if (wbank == 2'd0)
                            fbuf0[widx] <= {i_r, i_g, i_b};
                        else if (wbank == 2'd1)
                            fbuf1[widx] <= {i_r, i_g, i_b};
                        else if (wbank == 2'd2)
                            fbuf2[widx] <= {i_r, i_g, i_b};
                        else
                            fbuf3[widx] <= {i_r, i_g, i_b};
                        wx <= wx + 10'd1;
                    end
                end else begin
                    // DE fell: end of active pixels on this CE sub-cycle
                    if (wx > 10'd0) begin
                        if (wy == 10'd0) meas_w <= wx;  // latch width from first active line
                        wy <= (wy < 10'd511) ? wy + 10'd1 : 10'd511;
                    end
                    wx <= 10'd0;
                end

                // VS rise: commit the completed frame, swap banks (unless frozen)
                if (i_vs && !i_vs_r) begin
                    frame_cnt  <= frame_cnt + 16'd1;
                    wx <= 10'd0; wy <= 10'd0; meas_w <= 10'd0;
                    if (!i_freeze) begin
                        rbuf_iw    <= meas_w;
                        rbuf_ih    <= wy;
                        rbuf_valid <= (meas_w > 10'd0 && wy > 10'd0);
                        rbuf_seq   <= frame_cnt;
                        wbank      <= wbank + 2'd1;  // 2-bit auto-wrap: 3→0
                    end
                end
            end
        end
    end

    // sim_dbg_frames: used on CLK_VIDEO posedges by the TB → export directly (no CDC)
    assign sim_dbg_frames = frame_cnt;

    // ---- CDC rbuf_valid / rbuf_seq to o_clk domain (2-FF synchroniser) ----
    reg        rv_s0 = 1'b0, rv_s1 = 1'b0;
    reg [15:0] rs_s0 = 16'd0, rs_s1 = 16'd0;
    always @(posedge o_clk) begin
        rv_s0 <= rbuf_valid; rv_s1 <= rv_s0;
        rs_s0 <= rbuf_seq;   rs_s1 <= rs_s0;
    end
    assign sim_show_valid = rv_s1;
    assign sim_show_seq   = rs_s1;    // live CDC: T-log staleness check needs the take time to match commit time

    // sim_show_take: pulse when the o_clk-domain seq changes (new frame available to reader)
    reg [15:0] rs_prev = 16'd0;
    always @(posedge o_clk) rs_prev <= rs_s1;
    assign sim_show_take = (rs_s1 != rs_prev);

    // ---- 720p timing: tango_video's beam.  The output at a clock is for the beam (cx, cy) of that clock, which is what
    // the real ascal's output is after tango_video's sweep alignment (tango_video.v, ce_hold: its vsync rises at
    // (o_hsstart, o_vsstart) of the beam).  This model ran its own counter from time 0 before, at whatever offset the
    // bench's reset gave it: 2 pixels left of the beam in the launcher's TANGO_SIM bench, hidden in the
    // Template's TANGO_SIM gate by that golden's tolerance.
    wire [11:0] hpos = sim_cx;
    wire [11:0] vpos = {1'b0, sim_cy};

    assign o_de  = (hpos < hdisp) && (vpos < vdisp);
    assign o_hs  = (hpos >= hsstart) && (hpos < hsend);
    assign o_vs  = (vpos >= vsstart) && (vpos < vsend);

    // ---- CDC wbank (i_clk domain) to o_clk domain via 2-FF synchroniser.
    // With 4 banks, rbank = (wbank - 1) mod 4: even 2 input-VS per output frame can't alias
    // because (W+2) mod 4 ≠ (W-1) mod 4 for any W.  3 banks were insufficient: (W+2) ≡ (W-1) mod 3.
    reg [1:0] wbank_s0 = 2'd0, wbank_s1 = 2'd0;
    always @(posedge o_clk) begin
        wbank_s0 <= wbank;
        wbank_s1 <= wbank_s0;
    end
    // rbank_r / rseq_r: latched at the LAST pixel of each output frame (htotal−1, vtotal−1).
    // This is ~41 000 clk_pix cycles after VS — far beyond the 2-cycle bank CDC delay — so
    // wbank_s1 is always fully settled before rbank is read for the next frame's active area.
    // Latching at VS instead caused a race when an input VS fired just before output VS:
    // wbank_s1 was still stale, rbank pointed to the previous frame's bank while show_seq had
    // already advanced (the two CDCs are independent).
    // rseq_r is the seq snapshot corresponding to rbank, used as show_seq for the F log line
    // so the golden's expected content always matches the bank being scanned.
    // All per-frame read-side state captured together at the LAST pixel (htotal−1, vtotal−1).
    // This gives the geo engine the full 41 000-cycle (~556 μs) blanking period to settle.
    // rbank_r, rseq_r, and the geometry snapshot are always mutually consistent: the golden's
    // expected content (from rseq_r) matches the geometry used to scan the next frame's pixels.
    reg [1:0]  rbank_r = 2'd3;   // init 3: first commit → bank 0; 0−1 wraps to 3 initially
    reg [15:0] rseq_r  = 16'd0;
    reg [11:0] fb_hmin = 12'd0, fb_hmax = 12'd640;
    reg [10:0] fb_vmin = 11'd0, fb_vmax = 11'd359;
    reg [11:0] fb_ih = 12'd0, fb_voff = 12'd0;
    always @(posedge o_clk) begin
        if (hpos == htotal - 12'd1 && vpos == vtotal - 12'd1) begin
            rbank_r <= wbank_s1 - 2'd1;   // 2-bit arithmetic: 0-1 wraps to 3
            rseq_r  <= rs_s1;
            fb_hmin <= hmin; fb_hmax <= hmax;
            fb_vmin <= vmin; fb_vmax <= vmax;
            fb_ih   <= i_ih;
            fb_voff <= i_voff;
        end
    end
    assign sim_show_rseq = rseq_r;
    wire [1:0] rbank = rbank_r;

    // ---- nearest-neighbour scaler (combinational, o_clk domain) -----------
    wire in_win = o_de &&
                  (hpos >= fb_hmin)          && (hpos <= fb_hmax) &&
                  (vpos[10:0] >= fb_vmin)    && (vpos[10:0] <= fb_vmax);

    wire [11:0] ox  = hpos - {1'b0, fb_hmin};         // output x within window
    wire [10:0] oy  = vpos[10:0] - fb_vmin;            // output y within window
    wire [11:0] ow  = fb_hmax - fb_hmin + 12'd1;       // output window width
    wire [10:0] oh  = fb_vmax - fb_vmin + 11'd1;       // output window height

    // sx = ((2*ox + 1) * iw - 1) / (2 * ow)
    // Widths: ox ≤ 1279 (11-bit after subtraction), iw ≤ SIMW ≤ 512 (10-bit)
    //   (2*ox+1) ≤ 2559 → 12 bits;  product ≤ 2559*512 = 1 310 208 → 21 bits
    wire [20:0] sx_n = ({9'b0, ox[10:0], 1'b0} + 21'd1) * {11'b0, rbuf_iw} - 21'd1;
    wire [12:0] sx_d = {1'b0, ow, 1'b0};          // 2*ow ≤ 2560 → 12-bit
    wire [9:0]  sx   = (ow == 12'd0) ? 10'd0 : sx_n / sx_d[12:0];

    // dl = ((2*oy + 1) * 2*ih - 1) / (2*oh),  sy = voff + dl >> 1
    // i_ih = cropped source height (from geo engine); i_voff = first source line shown.
    // Widths: oy ≤ 719 (10-bit), ih ≤ SIMH ≤ 512 (10-bit)
    //   (2*oy+1) ≤ 1439 → 11 bits; *2*ih ≤ 1439*1024 = 1 473 536 → 21 bits
    wire [9:0]  ih_use = (fb_ih == 12'd0) ? rbuf_ih : fb_ih[9:0];  // fall back to measured if not yet valid
    wire [21:0] dl_n = ({10'b0, oy, 1'b0} + 22'd1) * ({11'b0, ih_use, 1'b0}) - 22'd1;
    wire [11:0] dl_d = {1'b0, oh, 1'b0};           // 2*oh ≤ 1440 → 11-bit
    wire [10:0] dl   = (oh == 11'd0) ? 11'd0 : dl_n[21:0] / {10'b0, dl_d[11:0]};
    wire [9:0]  sy   = fb_voff[9:0] + dl[10:1];    // crop offset + dl >> 1

    // flat index into frame buffer: sy * SIMW + sx  (max 511*512+511 = 262143 < 2^18)
    wire [17:0] ridx = {8'b0, sy} * SIMW[17:0] + {8'b0, sx};

    wire [23:0] rpix = (in_win && rv_s1) ?
                       (rbank == 2'd0 ? fbuf0[ridx] :
                        rbank == 2'd1 ? fbuf1[ridx] :
                        rbank == 2'd2 ? fbuf2[ridx] : fbuf3[ridx]) : 24'd0;
    assign {o_r, o_g, o_b} = rpix;

    // dl[0]: parity of the doubled-line index; used by tango_video for scanline dimming
    assign sim_dl_parity = dl[0];
    assign sim_frame_end = (hpos == htotal - 12'd1 && vpos == vtotal - 12'd1);

    // Debug outputs: tied off in simulation (no real Avalon bus in sim mock)
    assign dbg_read_count = 32'd0;
    assign dbg_readdata_valid_count = 32'd0;
    assign dbg_write_count = 32'd0;
    assign dbg_last_address = 29'd0;
    assign dbg_last_readdata = 64'd0;
    assign dbg_waitrequest_stuck = 1'b0;
    assign dbg_avl_read = 1'b0;
    assign dbg_avl_readdatavalid = 1'b0;
    assign dbg_avl_waitrequest = 1'b0;

endmodule
`default_nettype wire
