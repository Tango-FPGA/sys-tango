// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_ascal_ddr3 -- the DDR3 real-netlist gate: tango_sys with TANGO_VIDEO_ASCAL + TANGO_VIDEO_OPTS +
// TANGO_VIDEO_F1 + TANGO_VIDEO_DDR3 and without TANGO_SIM: the REAL ascal (GHDL netlist at N_DW 128, N_AW 28,
// sys/ascal_ghdl/ascal_fixed_ddr3.v) through tango_ddr3's scaler port into the IP model (sim/tango_ddr3_model.v,
// calibrated on silicon: sim/tango_ddr3_silicon.vh), around the sim-only core sim/tb_ascal_ddr3_core.v (the
// interlaced, then progressive, Genesis-like source of sim/tb_ascal_ext.v).  As sim/tb_ascal_ext.v otherwise:
// clk_pix 74.25 MHz (the hdmi.sv beam, 1650 x 750), clk50, clk48; no BL616 link, no USB devices.  clk_ddr is its own
// 100 MHz clock with a random start phase (asynchronous to clk_pix and the core's clocks).
//   +status=<hex> +status2=<hex> +switch_frame=<k>   the option words, as tb_ascal_ext.v
//   +seed=<n>                the random start phase and the stall times
//   +stall35_us=<n>          cmd_ready low for 35 clk_ddr (350 ns: the DDR3 bring-up test's soak maximum) every n us on average (0: off)
//   +ref8_us=<n>             8 refreshes back to back every n us on average (0: off; NOT seen on silicon)
// Writes out_f<N>.ppm (1280 x 720) for frames FIRST_DUMP..NFRAMES-1 (sim/check_ascal_ext.py judges them).
// The Avalon monitor (clk_ddr, ascal's side of the scaler port; written for this bench):
//   - data: a shadow of every beat ascal wrote (byte enables applied); every beat read back must equal the shadow as
//     it was when the read was taken (the port is in order: a read sees the writes taken before it, not after); beats
//     never written count apart (the model's power-up zero);
//   - protocol: a write burst's beats all arrive before the next command; waitrequest held with a command or beat
//     present <= 32 clk_ddr after the IP's calibration; before it ascal must wait (a command
//     taken in reset would be dropped -- the data check catches that);
//   - the read pattern per output frame (a frame starts with line 0 burst 0 after a line >= 8; the byte address -
//     RAMBASE gives the buffer (RAMSIZE 4 MB each: sys/ascal_ghdl/ascal_fixed_ddr3.vhd), the line and the burst at
//     NB = 4 bursts of 256 B a line (ascal's stride: 320 px x 3 B rounded up to its 256 B bursts, ascal.vhd 1938-1945): the
//     frame's bursts must be whole lines (bursts = (last line + 1) x NB), the line read after the preload's lines 0
//     and 1 must be 2 (the DDA init kept), and line 100's first read must issue at the same output beam position in
//     every frame of a mode (within 4 clk_pix: the request's o_clk -> avl_clk synchroniser, 2-3 clk_ddr).  A mode's
//     first frame (its last line differs from the frame before: the start-up, the option switch) is not judged; at
//     most 2 such frames after FIRST_MON.
// Prints "MON frame ..." per output frame and "MON_RESULT PASS / FAIL" at the end.
//==============================================================================
`default_nettype none
`timescale 1ps/1ps
module tb_ascal_ddr3;
`include "tango_ddr3_silicon.vh"
`include "tango_cfg.vh"
    // the beam = hdmi.sv's raster at the build's totals (tango_cfg.vh TANGO_HDMI_HTOTAL / _VTOTAL; default 1650 x 750)
    localparam integer FW = `TANGO_HDMI_HTOTAL, FH = `TANGO_HDMI_VTOTAL, OW = 1280, OH = 720;
`ifdef TB_BEAM_FW
    localparam integer BFW = `TB_BEAM_FW, BFH = `TB_BEAM_FH;   // NEGATIVE CONTROL only: a beam that is not tango_sys's raster
`else
    localparam integer BFW = FW, BFH = FH;
`endif
`ifdef NFRAMES_DEF
    localparam integer NFRAMES = `NFRAMES_DEF;
`else
    localparam integer NFRAMES = 14;
`endif
    localparam integer FIRST_DUMP = 2;
    localparam integer FIRST_MON = 3;                  // frames judged by the monitor: from this one
    localparam [31:0] RAMBASE = 32'h2000_0000;
    localparam integer NB = 4;
    localparam integer RAMSIZE = 32'h0040_0000;

    integer seed = 1, stall35_us = 0, ref8_us = 0;
    initial begin
        void'($value$plusargs("seed=%d", seed));
        void'($value$plusargs("stall35_us=%d", stall35_us));
        void'($value$plusargs("ref8_us=%d", ref8_us));
    end

    reg clk_pix = 1'b0, clk50 = 1'b0, clk48 = 1'b0, clk_ddr = 1'b0;
    integer pause_frame = -1, pause_line = 730, pause_x = 0, pause_us = 18000, n_pauses = 0, pause_skip = 0;   // TB_PAUSE
    reg     pause_req = 1'b0, pause_done = 1'b0;
`ifdef TANGO_HDMI_PLL_SET
    // the build-time HDMI PLL presets (tango_cfg.vh TANGO_HDMI_PLL_<f>): clk_pix at the preset's EXACT frequency, half periods of 5e11 / f ps (the
    // fraction by a Bresenham accumulator)
    localparam longint FPIX = `TANGO_HDMI_FPIX, HPI = 64'd500000000000 / FPIX, HPR = 64'd500000000000 % FPIX;
    longint hacc = 0;
    always begin
        hacc = hacc + HPR;
        if (hacc >= FPIX) begin hacc = hacc - FPIX; #(HPI + 1); end else #(HPI);
        clk_pix = ~clk_pix;
        if (!clk_pix && pause_req) pause_now();
    end
`else
    localparam longint FPIX = 74250000;
    always begin #6734 clk_pix = ~clk_pix; if (!clk_pix && pause_req) pause_now(); end   // 74.25 MHz
`endif

    always #10000 clk50   = ~clk50;
    always #10417 clk48   = ~clk48;
    integer ph;
    initial begin
        void'($value$plusargs("seed=%d", seed));
        ph = $urandom(seed) % 10000;
        #(ph);
        forever #5000 clk_ddr = ~clk_ddr;    // 100 MHz, its own phase
    end

    reg [6:0] rcnt = 7'd0;
    wire rst_pix = (rcnt != 7'd64);
    always @(posedge clk_pix) if (rst_pix) rcnt <= rcnt + 7'd1;

    reg [11:0] cx = 12'd0;  reg [10:0] cy = 11'd0;
`ifdef TANGO_VSYNC_ADJUST
    // vsync_adjust: the beam is hdmi.sv's with runtime totals -- the next frame's (tango_vsync, vt_nxt_*) taken in reset and at
    // the wrap, the current frame's (bw / bh) given back to tango_sys as hdmi.sv's frame_width / frame_height
    wire [11:0] vt_nxt_w;  wire [10:0] vt_nxt_h;
    reg  [11:0] bw = FW;   reg  [10:0] bh = FH;
    integer     n_raster = 0, out_frame = 0;
    always @(posedge clk_pix) begin
        if (rst_pix) begin cx <= 12'd0; cy <= 11'd0; bw <= vt_nxt_w; bh <= vt_nxt_h; end
        else if (cx == bw - 12'd1) begin
            cx <= 12'd0;
            if (cy == bh - 11'd1) begin
                cy <= 11'd0;  bw <= vt_nxt_w;  bh <= vt_nxt_h;  out_frame = out_frame + 1;
                if (vt_nxt_w != bw || vt_nxt_h != bh) begin
                    n_raster = n_raster + 1;
                    $display("RASTER from output frame %0d: %0d x %0d = %0d clk_pix (%.6f Hz) -- tango_vsync: P %0d, plan %0d x %0d err %0d, status %02x",
                             out_frame, vt_nxt_w, vt_nxt_h, {20'd0, vt_nxt_w} * {21'd0, vt_nxt_h}, 1.0 * FPIX / (1.0 * vt_nxt_w * vt_nxt_h),
                             dut.u_vsync.meas_p, dut.u_vsync.plan_w, dut.u_vsync.plan_h, dut.u_vsync.plan_err, dut.vs_status);
                end
            end else cy <= cy + 11'd1;
        end else cx <= cx + 12'd1;
    end
    // tango_vsync's measurements as they come
    reg [21:0] mp_q = 22'd0;
    always @(posedge clk_pix) begin
        mp_q <= dut.u_vsync.meas_p;
        if (dut.u_vsync.meas_p != mp_q)
            $display("VSYNC measured: P %0d clk_pix (%.6f Hz) at output frame %0d", dut.u_vsync.meas_p,
                     1.0 * FPIX / dut.u_vsync.meas_p, out_frame);
    end
`else
    always @(posedge clk_pix) begin
        if (rst_pix) begin cx <= 12'd0; cy <= 11'd0; end
        else if (cx == BFW - 1) begin cx <= 12'd0; cy <= (cy == BFH - 1) ? 11'd0 : cy + 11'd1; end
        else cx <= cx + 12'd1;
    end
`endif

    // ---- the IP model
    wire         calib, cmd_ready, cmd_en, wr_data_rdy, wr_data_en, rd_data_valid, rd_data_end;
    wire [2:0]   cmd;
    wire [28:0]  addr;
    wire [255:0] wr_data, rd_data;
    wire [31:0]  wr_data_mask, n_refresh, n_rowmiss, n_bad_cmd;
    reg          m_rst_n = 1'b0, force_stall = 1'b0, ref_burst = 1'b0;
    initial #200000 m_rst_n = 1'b1;
    tango_ddr3_model #(.MEM_LOG2(20), .INIT_CLKS(1000), .CMD_DEPTH(SIL_CMD_DEPTH), .WDATA_DEPTH(SIL_WDATA_DEPTH),
                       .RD_LAT(SIL_RD_LAT), .RD_JITTER(SIL_RD_JITTER), .T_BL(1), .T_ROWMISS(SIL_T_ROWMISS),
                       .T_TURN(SIL_T_TURN), .REF_PERIOD(SIL_REF_PERIOD), .REF_LEN(SIL_REF_LEN)) u_m (
        .clk(clk_ddr), .rst_n(m_rst_n), .init_calib_complete(calib),
        .cmd(cmd), .cmd_en(cmd_en), .addr(addr), .cmd_ready(cmd_ready),
        .wr_data(wr_data), .wr_data_en(wr_data_en), .wr_data_end(wr_data_en), .wr_data_mask(wr_data_mask),
        .wr_data_rdy(wr_data_rdy), .rd_data(rd_data), .rd_data_valid(rd_data_valid), .rd_data_end(rd_data_end),
        .force_stall(force_stall), .ref_burst(ref_burst), .fault_en(1'b0), .fault_word(25'd0), .fault_xor(256'd0),
        .n_refresh(n_refresh), .n_rowmiss(n_rowmiss), .n_bad_cmd(n_bad_cmd));

    // ---- stall patterns (clk_ddr): random intervals, uniform 0.5..1.5 x the mean
    integer st_next = 0, st_left = 0, rf_next = 0, n_st = 0, n_rf = 0;
    reg [31:0] dcyc = 32'd0;
    always @(posedge clk_ddr) begin
        dcyc <= dcyc + 32'd1;
        ref_burst <= 1'b0;
        if (calib && stall35_us > 0) begin
            if (st_next == 0) st_next = dcyc + 50 * stall35_us + ($urandom % (100 * stall35_us));
            if (st_left > 0) begin st_left = st_left - 1; force_stall <= (st_left != 0); end
            else if (dcyc >= st_next) begin
                st_left = SIL_WORST_STALL; force_stall <= 1'b1; n_st = n_st + 1;
                st_next = dcyc + 50 * stall35_us + ($urandom % (100 * stall35_us));
            end
        end
        if (calib && ref8_us > 0) begin
            if (rf_next == 0) rf_next = dcyc + 50 * ref8_us + ($urandom % (100 * ref8_us));
            else if (dcyc >= rf_next) begin
                ref_burst <= 1'b1; n_rf = n_rf + 1;
                rf_next = dcyc + 50 * ref8_us + ($urandom % (100 * ref8_us));
            end
        end
    end

    wire [23:0] rgb;
    wire usb1_dp, usb1_dn, usb2_dp, usb2_dn, uart_tx;
    pulldown (usb1_dp); pulldown (usb1_dn); pulldown (usb2_dp); pulldown (usb2_dn);

    reg [127:0] tb_status = 128'd0;
    initial if ($value$plusargs("status=%h", tb_status)) begin
        #2000000;
        $display("tb: option word set to %032x", tb_status);
        dut.u_link.status = tb_status;
        dut.u_link.status_tog = ~dut.u_link.status_tog;
    end

    tango_sys #(.USB_DEBOUNCE(960_000)) dut (
        .clk50(clk50), .clk_pix(clk_pix), .rst_pix(rst_pix), .cx(cx), .cy(cy), .rgb(rgb),
        .audio_l(), .audio_r(), .clk48(clk48), .lock48(1'b1),
        .usb1_dp(usb1_dp), .usb1_dn(usb1_dn), .usb2_dp(usb2_dp), .usb2_dn(usb2_dn),
        .spi_csn(1'b1), .spi_sck(1'b0), .spi_mosi(1'b0), .spi_miso(), .spi_miso_pad(1'b0), .spi_irq_n(),
        .bl616_jtagsel(1'b0), .jtagsel_n_q(1'b1),
        .osd_shown(), .osd_bank(), .osd_swap_pending(), .joy(), .joy_flags(), .link_xfers(),
        .usb1_bad_crc(), .usb2_bad_crc(), .dbg_rows(),
`ifdef TANGO_VSYNC_ADJUST
        .vt_cur_w(bw), .vt_cur_h(bh), .vt_nxt_w(vt_nxt_w), .vt_nxt_h(vt_nxt_h),
`endif
        .dbg_uart_tx(uart_tx),
        .clk_ddr(clk_ddr), .ddr_rst(!m_rst_n), .ddr_calib(calib),
        .ddr_cmd_ready(cmd_ready), .ddr_cmd_en(cmd_en), .ddr_cmd(cmd), .ddr_addr(addr),
        .ddr_wr_data_rdy(wr_data_rdy), .ddr_wr_data_en(wr_data_en), .ddr_wr_data(wr_data),
        .ddr_wr_data_mask(wr_data_mask), .ddr_rd_data_valid(rd_data_valid), .ddr_rd_data(rd_data));

    // ================================================================ the Avalon monitor (clk_ddr)
    wire [27:0]  a_addr = dut.vid_avl_addr;
    wire         a_wr = dut.vid_avl_write, a_rd = dut.vid_avl_read, a_wait = dut.vid_avl_waitreq;
    wire         a_rv = dut.vid_avl_rvalid;
    wire [127:0] a_wd = dut.vid_avl_wdata, a_rdat = dut.vid_avl_rdata;
    wire [7:0]   a_bc = dut.vid_avl_burst;
    wire [15:0]  a_be = dut.vid_avl_be;
    logic [127:0] shadow [int unsigned];
    integer      wb_left = 0, wb_idx = 0;
    reg  [27:0]  wb_addr = 28'd0;
    reg  [27:0]  rq_a [0:15];
    reg  [7:0]   rq_n [0:15];
    reg  [127:0] rq_x [0:255];                       // the expected beats, from the shadow when the read was taken
    reg          rq_u [0:255];                       // ... never written
    integer      rq_w = 0, rq_r = 0, rd_idx = 0;
    integer      n_wbeats = 0, n_wbursts = 0, n_rbursts = 0, n_rbeats = 0, n_err = 0, n_unwr = 0, n_proto = 0;
    integer      n_partial = 0, wait_run = 0, wait_max = 0, wait_pre = 0;
    reg          wait_in_rst = 1'b0;
    // the read pattern
    integer      mf = 0, fr_b = 0, fr_maxl = 0, fr_next = -1, fr_l100 = -1, fr_l100_beam = -1, fr_pre = -1;
    integer      fr_t0 = 0, prev_line = 0, fr_buf = 0;
    reg          rd_issue_q = 1'b0;
    integer      n_bad_frames = 0;
    integer      seg_beam_min = 0, seg_beam_max = 0, seg_maxl = -1, seg_n = 0, beam_spread_max = 0;
    function automatic [127:0] merge_be(input [127:0] o, input [127:0] n, input [15:0] be);
        integer k;
        begin
            merge_be = o;
            for (k = 0; k < 16; k = k + 1) if (be[k]) merge_be[8*k +: 8] = n[8*k +: 8];
        end
    endfunction
    integer      prev_maxl = -1, n_trans = 0, prev_rw = -1, fr_l100_w = -1;   // runtime totals: the raster at line 100's read
    // input frames: ascal writes an input frame into one buffer, so a write burst into
    // another buffer means the frame before it is complete; counted between output frame starts.  At a native rate
    // (TB_VSYNC_CORE) every output frame must see exactly ONE: 0 = the output repeated a frame, 2+ = it dropped one.
    integer      wbuf_cur = -1, in_done = 0, in_since = 0, n_in_switch = 0, in_seen = 0;
    integer      h_in0 = 0, h_in1 = 0, h_in2 = 0, in_phase = -1;
    // TB_PAUSE (+pause_frame=<n> +pause_line=<y> [+pause_x=<x>] +pause_us=<t>): the clock-pause
    // retune at the bench's level -- when the MON's output frame n reaches the beam (x, y), clk_pix stops (low, after a falling edge)
    // for t us and resumes; nothing is reset.  The frame spanning the pause and the one after are not judged (they see the
    // input frames that completed meanwhile); every other check stands.  y in the vertical blanking (730) must PASS; an
    // active line (ascal reads in flight) is the negative control.
    initial begin
        void'($value$plusargs("pause_frame=%d", pause_frame));
        void'($value$plusargs("pause_line=%d", pause_line));
        void'($value$plusargs("pause_x=%d", pause_x));
        void'($value$plusargs("pause_us=%d", pause_us));
    end
    task automatic pause_now;
        begin
            pause_req = 1'b0;  n_pauses = n_pauses + 1;  pause_skip = 2;
            $display("PAUSE %0d: clk_pix stopped at beam (%0d, %0d), MON frame %0d, for %0d us (at %0t)", n_pauses, cx, cy, mf,
                     pause_us, $time);
            #(longint'(pause_us) * 64'd1000000);
            $display("PAUSE %0d: clk_pix resumes (at %0t)", n_pauses, $time);
        end
    endtask
    always @(posedge clk_pix) if (!pause_done && pause_frame >= 0 && mf == pause_frame && cx == 12'(pause_x) && cy == 11'(pause_line)) begin
        pause_req = 1'b1;  pause_done = 1'b1;
    end
    task automatic frame_end;
        reg ok, trans; integer beam;
        begin
            // input frames completed since the output frame before (judged once two input frames are complete)
            if (in_seen >= 2 && mf >= FIRST_MON && pause_skip == 0) begin
                if (in_since == 0) h_in0 = h_in0 + 1; else if (in_since == 1) h_in1 = h_in1 + 1; else h_in2 = h_in2 + 1;
            end
            trans = (fr_maxl != prev_maxl);              // the first frame of a mode (start-up, the option switch)
            if (pause_skip > 0) begin trans = 1'b1; pause_skip = pause_skip - 1; end   // TB_PAUSE: not judged
`ifdef TANGO_VSYNC_ADJUST
            if (fr_l100_w != prev_rw) begin trans = 1'b1; seg_maxl = -1; end   // a new raster: a new mode (runtime totals)
            prev_rw = fr_l100_w;
`endif
            prev_maxl = fr_maxl;
            if (mf >= FIRST_MON && trans) n_trans = n_trans + 1;
            if (mf >= FIRST_MON && !trans) begin
                ok = (fr_b == (fr_maxl + 1) * NB) && (fr_next == 2) && (fr_l100_beam >= 0);
                // line 100's beam position: per mode (a run of frames with the same last line)
                if (fr_maxl != seg_maxl) begin seg_maxl = fr_maxl; seg_n = 0; end
                beam = fr_l100_beam;
                if (seg_n == 0) begin seg_beam_min = beam; seg_beam_max = beam; end
                else begin
                    if (beam < seg_beam_min) seg_beam_min = beam;
                    if (beam > seg_beam_max) seg_beam_max = beam;
                end
                seg_n = seg_n + 1;
                if (seg_beam_max - seg_beam_min > beam_spread_max) beam_spread_max = seg_beam_max - seg_beam_min;
                if (!ok) n_bad_frames = n_bad_frames + 1;
            end
            $display("MON frame %0d: buffer %0d, input frames since the last %0d (last input end at beam %0d), raster width %0d, bursts %0d, last line %0d, line after the preload %0d, preload %0d clk_ddr, line 100 at %0d clk_ddr / beam %0d%0s",
                     mf, fr_buf, in_since, in_phase, fr_l100_w, fr_b, fr_maxl, fr_next, fr_pre, fr_l100, fr_l100_beam,
                     (mf < FIRST_MON) ? " (start-up, not judged)" : trans ? " (a mode's first frame, not judged)" :
                     ((fr_b == (fr_maxl + 1) * NB) && (fr_next == 2) && (fr_l100_beam >= 0)) ? "" : " BAD");
            mf = mf + 1;
            in_since = 0;
        end
    endtask
    integer mondbg = 0;
    initial void'($value$plusargs("mondbg=%d", mondbg));
    always @(posedge clk_ddr) if (mondbg > 0) begin              // +mondbg=<n>: the first n events, for debugging
        if ((a_wr || a_rd) && !a_wait) begin
            $display("DBG %0t avl %s addr %07x bc %0d wd %032x be %04x", $time, a_wr ? "W" : "R", a_addr, a_bc, a_wd, a_be);
            mondbg = mondbg - 1;
        end
        if (a_rv) begin $display("DBG %0t avl rv %032x", $time, a_rdat); mondbg = mondbg - 1; end
        if (cmd_en && cmd_ready) begin
            $display("DBG %0t ip %s addr %07x mask %08x d %064x", $time, (cmd == 3'b000) ? "W" : "R", addr, wr_data_mask,
                     wr_data);
            mondbg = mondbg - 1;
        end
        if (rd_data_valid) begin $display("DBG %0t ip rv %064x", $time, rd_data); mondbg = mondbg - 1; end
    end
    always @(posedge clk_ddr) begin : mon
        reg [31:0] off; integer line, burst, i;
        // waitrequest with a command or a beat present: after the IP's calibration (before it, ascal must wait: the
        // longest such wait is reported apart)
        if ((a_wr || a_rd) && a_wait) begin
            if (wait_run == 0) wait_in_rst = dut.ddr_rs[2];          // a wait that began in reset counts as before
            wait_run = wait_run + 1;
            if (wait_in_rst) begin if (wait_run > wait_pre) wait_pre = wait_run; end
            else if (wait_run > wait_max) wait_max = wait_run;
        end else wait_run = 0;
        // read issue (the first clock of a read command) -> the frame pattern
        rd_issue_q <= a_rd && a_wait;
        if (a_rd && !rd_issue_q) begin
            off = {a_addr, 4'd0} - RAMBASE;
            line = (off % RAMSIZE) / (NB * 256);  burst = ((off % RAMSIZE) / 256) % NB;
            if (line == 0 && burst == 0 && prev_line >= 8) begin
                if (fr_t0 != 0) frame_end;
                fr_t0 = dcyc; fr_b = 0; fr_maxl = 0; fr_next = -1; fr_l100 = -1; fr_l100_beam = -1; fr_pre = -1;
                fr_buf = off / RAMSIZE;
            end
            if (fr_t0 != 0) begin
                fr_b = fr_b + 1;
                if (line > fr_maxl) fr_maxl = line;
                if (fr_next < 0 && line > 1) begin fr_next = line; fr_pre = dcyc - fr_t0; end
`ifdef TANGO_VSYNC_ADJUST
                if (line == 100 && burst == 0 && fr_l100 < 0) begin
                    fr_l100 = dcyc - fr_t0; fr_l100_beam = cy * bw + cx; fr_l100_w = bw;
                end
`else
                if (line == 100 && burst == 0 && fr_l100 < 0) begin fr_l100 = dcyc - fr_t0; fr_l100_beam = cy * FW + cx; end
`endif
            end
            prev_line = line;
        end
        // commands and beats taken
        if (!a_wait) begin
            if (a_wr) begin
                if (wb_left == 0) begin
                    if (((({a_addr, 4'd0} - RAMBASE) / RAMSIZE)) != wbuf_cur) begin
                        if (wbuf_cur >= 0) begin
                            n_in_switch = n_in_switch + 1;  in_since = in_since + 1;  in_seen = in_seen + 1;
                            in_phase = cy * FW + cx;
                        end
                        wbuf_cur = ({a_addr, 4'd0} - RAMBASE) / RAMSIZE;
                    end
                    if (a_bc == 8'd0) n_proto = n_proto + 1;
                    wb_addr = a_addr; wb_left = a_bc; wb_idx = 0; n_wbursts = n_wbursts + 1;
                end
                if (a_be != 16'hFFFF) n_partial = n_partial + 1;
                shadow[wb_addr + wb_idx] = merge_be(shadow.exists(wb_addr + wb_idx) ? shadow[wb_addr + wb_idx] : 128'd0,
                                                    a_wd, a_be);
                wb_idx = wb_idx + 1; wb_left = wb_left - 1; n_wbeats = n_wbeats + 1;
            end
            if (a_rd) begin
                if (wb_left != 0) n_proto = n_proto + 1;              // a read inside a write burst
                rq_a[rq_w % 16] = a_addr; rq_n[rq_w % 16] = a_bc;
                for (i = 0; i < a_bc && i < 16; i = i + 1) begin
                    rq_u[(rq_w % 16) * 16 + i] = !shadow.exists(a_addr + i);
                    rq_x[(rq_w % 16) * 16 + i] = shadow.exists(a_addr + i) ? shadow[a_addr + i] : 128'd0;
                end
                if (a_bc > 16) n_proto = n_proto + 1;
                rq_w = rq_w + 1; n_rbursts = n_rbursts + 1;
            end
        end
        if (a_rv) begin
            if (rq_r == rq_w) n_proto = n_proto + 1;                  // a beat with no read outstanding
            else begin
                i = (rq_r % 16) * 16 + rd_idx;
                if (rq_u[i]) n_unwr = n_unwr + 1;
                if (a_rdat !== rq_x[i]) begin
                    n_err = n_err + 1;
                    if (n_err <= 8) $display("MON data error: word %07x beat %0d got %032x want %032x%0s", rq_a[rq_r % 16],
                                             rd_idx, a_rdat, rq_x[i], rq_u[i] ? " (never written)" : "");
                end
                n_rbeats = n_rbeats + 1; rd_idx = rd_idx + 1;
                if (rd_idx == rq_n[rq_r % 16]) begin rd_idx = 0; rq_r = rq_r + 1; end
            end
        end
    end

    // ---- frame capture: rgb at clock t is for the beam (cx, cy) of clock t-1
    reg [11:0] px = 12'd0;  reg [10:0] py = 11'd0;
    integer frame = 0, fd = 0, nonblack = 0;
`ifdef TANGO_VSYNC_ADJUST
    reg [11:0] pfw = FW;  reg [10:0] pfh = FH;      // the beam's totals one clock late, as px / py
    always @(posedge clk_pix) begin pfw <= bw; pfh <= bh; end
`else
    wire [11:0] pfw = BFW;  wire [10:0] pfh = BFH;
`endif
    reg [8*32-1:0] fname;
    always @(posedge clk_pix) begin
        px <= cx;  py <= cy;
        if (!rst_pix && px < OW && py < OH) begin
            if (fd != 0) $fwrite(fd, "%c%c%c", rgb[23:16], rgb[15:8], rgb[7:0]);
            if (rgb != 24'd0) nonblack = nonblack + 1;
        end
        if (!rst_pix && px == pfw - 12'd1 && py == pfh - 11'd1) begin
            $display("frame %0d: nonblack=%0d  ascal RC=%0d RV=%0d WC=%0d  avl write bursts %0d read bursts %0d  interlaced m_il=%0d l_il=%0d v_sl=%0d",
                     frame, nonblack, dut.dbg_arc, dut.dbg_arv, dut.dbg_awc, n_wbursts, n_rbursts,
                     dut.u_vid.m_il, dut.u_vid.l_il, dut.u_vid.v_sl);
            $display("  geometry: core frame %0dx%0d  geo_ok=%0d  ascal window h %0d..%0d v %0d..%0d",
                     dut.u_vid.l_hs, dut.u_vid.l_vt, dut.u_vid.geo_ok,
                     dut.u_vid.v_hmin, dut.u_vid.v_hmax, dut.u_vid.v_vmin, dut.u_vid.v_vmax);
            if (fd != 0) $fclose(fd);
            fd = 0;  nonblack = 0;
            frame = frame + 1;
            if (frame == NFRAMES) begin
                $display("MON avalon: %0d write bursts (%0d beats, %0d partial), %0d read bursts (%0d beats); data errors %0d (beats never written %0d); protocol errors %0d; longest waitrequest with a command %0d clk_ddr (before calibration %0d)",
                         n_wbursts, n_wbeats, n_partial, n_rbursts, n_rbeats, n_err, n_unwr, n_proto, wait_max, wait_pre);
                $display("MON frames %0d..%0d: bad %0d, a mode's first frame (not judged) %0d (limit 2); line 100's beam position spread per mode %0d clk_pix (limit 4)",
                         FIRST_MON, mf - 1, n_bad_frames, n_trans, beam_spread_max);
                $display("MON model: refreshes %0d, row misses %0d, bad commands %0d; stalls of 35 clk_ddr %0d, 8-refresh bursts %0d",
                         n_refresh, n_rowmiss, n_bad_cmd, n_st, n_rf);
                $display("MON input frames: %0d complete; output frames judged by input frames since the one before: 0 (repeat) %0d, 1 %0d, 2+ (drop) %0d; raster %0d x %0d = %0d clk_pix",
                         n_in_switch, h_in0, h_in1, h_in2, FW, FH, FW * FH);
                $display("MON_RESULT %0s", (n_err == 0 && n_proto == 0 && n_bad_cmd == 0 && wait_max <= 32 &&
                         n_bad_frames == 0 && beam_spread_max <= 4 && mf > FIRST_MON + 4
`ifdef TANGO_VSYNC_ADJUST
                         && n_trans <= 2 + n_raster + 2 * n_pauses
`else
                         && n_trans <= 2
`endif
`ifdef TB_VSYNC_STAGE2
                         // runtime totals: the input/output frame accounting is judged per raster segment by sim/check_vsync_stage2.py
`elsif TB_VSYNC_CORE
                         // a native-rate source: every judged output frame shows exactly one new input frame
                         && h_in0 == 0 && h_in2 == 0 && h_in1 >= 6
`endif
                         ) ? "PASS" : "FAIL");
                $display("TB done: %0d frames", frame);
                $finish;
            end
            if (frame >= FIRST_DUMP) begin
                $sformat(fname, "out_f%0d.ppm", frame);
                fd = $fopen(fname, "wb");
                $fwrite(fd, "P6\n%0d %0d\n255\n", OW, OH);
            end
        end
    end

    reg [127:0] tb_status2 = 128'd0;
    integer     tb_switch = 0;
    initial if ($value$plusargs("status2=%h", tb_status2) && $value$plusargs("switch_frame=%d", tb_switch)) begin
        wait (frame == tb_switch);
        $display("tb: option word set to %032x at the start of frame %0d", tb_status2, frame);
        dut.u_link.status = tb_status2;
        dut.u_link.status_tog = ~dut.u_link.status_tog;
    end
endmodule
`default_nettype wire
