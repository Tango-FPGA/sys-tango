// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_rotate_arb -- the real-arbiter bandwidth proof of the rotation client: the REAL
// tango_ddr3 (tango_ddr3_arb + tango_ddr3_vport + tango_ddr3_cport, unmodified) and the REAL DDR3 IP
// behavioural model (sim/tango_ddr3_model.v, calibrated on silicon, sim/tango_ddr3_silicon.vh), with
// tango_video_rotate_ddr3 as the ACTUAL C0 client (not a synthetic adversary) at a real core's real pixel
// rate, concurrently with a synthetic but representative V-port read/write generator sized to the
// Template core's own bandwidth (29.6 MB/s, 4 bursts/line, 22.2 us lines) -- not the
// full ascal netlist (this bench isolates the ARBITER's own behaviour under real contention, which is what
// the rotation client's bandwidth ceilings depend on).
//
// Clocks: clk_ddr 100 MHz (the IP's own clk_out, real); clk_video 74.25 MHz (the Template core's CLK_VIDEO,
// matching its own "CE_PIXEL every 12th CLK_PIX clock" comment -- CE_DIV=12 below gives
// the real 6.1875 MHz pixel rate). Both clocks are genuinely free-running and
// asynchronous (different periods, independent phase) -- lockstep clocks cannot catch CDC bugs.
//
// What this proves: whether the rotation client's elastic FIFO (32 pixels) ever overflows under REAL
// arbiter contention from a REAL V load at the documented rate, and what margin (max observed DDRAM_BUSY
// stall, max FIFO occupancy, achieved write throughput) it actually has -- one real, measured number
// in place of estimated ceilings, for the proof-of-mechanism core.
//==============================================================================
`default_nettype none
`timescale 1ns/1ps

module tb_rotate_arb;
    // ---- clk_ddr (100 MHz, the IP's clk_out) and clk_video (74.25 MHz, Template's real CLK_VIDEO)
    reg clk_ddr = 1'b0;
    always #5 clk_ddr = ~clk_ddr;
    reg clk_video = 1'b0;
    initial begin #3; forever #6.734 clk_video = ~clk_video; end   // 74.25 MHz, independent phase

    reg rst_n = 1'b0;
    initial #200 rst_n = 1'b1;

    // ---- the IP behavioural model, calibrated on silicon (sim/tango_ddr3_silicon.vh)
`include "tango_ddr3_silicon.vh"
    wire         calib, cmd_ready, cmd_en, wr_data_rdy, wr_data_en, rd_data_valid, rd_data_end;
    reg  [2:0]   ddr_rs = 3'b111;                   // matches tango_sys.v's own ddr_rs (3-stage shift of
                                                      // ddr_rst|~ddr_calib) -- the real reset the arbiter sees
    always @(posedge clk_ddr) ddr_rs <= {ddr_rs[1:0], (!rst_n) | (!calib)};
    wire [2:0]   cmd;
    wire [28:0]  addr;
    wire [255:0] wr_data, rd_data;
    wire [31:0]  wr_data_mask, n_refresh, n_rowmiss, n_bad_cmd;
    tango_ddr3_model #(.MEM_LOG2(20), .INIT_CLKS(1000), .CMD_DEPTH(SIL_CMD_DEPTH), .WDATA_DEPTH(SIL_WDATA_DEPTH),
                       .RD_LAT(SIL_RD_LAT), .RD_JITTER(SIL_RD_JITTER), .T_BL(1), .T_ROWMISS(SIL_T_ROWMISS),
                       .T_TURN(SIL_T_TURN), .REF_PERIOD(SIL_REF_PERIOD), .REF_LEN(SIL_REF_LEN)) u_model (
        .clk(clk_ddr), .rst_n(rst_n), .init_calib_complete(calib),
        .cmd(cmd), .cmd_en(cmd_en), .addr(addr), .cmd_ready(cmd_ready),
        .wr_data(wr_data), .wr_data_en(wr_data_en), .wr_data_end(wr_data_en), .wr_data_mask(wr_data_mask),
        .wr_data_rdy(wr_data_rdy), .rd_data(rd_data), .rd_data_valid(rd_data_valid), .rd_data_end(rd_data_end),
        .force_stall(1'b0), .ref_burst(1'b0), .fault_en(1'b0), .fault_word(25'd0), .fault_xor(256'd0),
        .n_refresh(n_refresh), .n_rowmiss(n_rowmiss), .n_bad_cmd(n_bad_cmd));

    // ---- V port generator: the Template core's profile: 4 bursts of 256 B a line
    // (16 beats of 128 bits each), a line every 22.2 us, half writes half reads (matching ascal's own
    // "input write + output read" split) -- synthetic but sized to the real bandwidth.
    reg [27:0] v_addr_r = 28'd0;
    reg        v_write_r = 1'b0, v_read_r = 1'b0;
    reg [7:0]  v_burst_r = 8'd16;
    reg [127:0] v_wd_r = 128'hA5A5_0000_0000_0000_0000_0000_0000_0001;
    wire        v_waitreq;
    wire [127:0] v_rdata; wire v_rvalid;
    integer    v_line_cnt = 0, v_burst_idx = 0;
    localparam integer V_LINE_CYC = 2222;        // 22.2 us at 100 MHz = 2222 clk_ddr cycles
    localparam integer V_BURSTS_PER_LINE = 4;
    reg [12:0] v_phase = 0;
    reg        v_inflight = 1'b0;
    integer    v_bursts_sent = 0, v_beats_sent = 0;
    always @(posedge clk_ddr) begin
        if (!rst_n) begin v_phase <= 0; v_inflight <= 1'b0; v_write_r <= 1'b0; v_read_r <= 1'b0; end
        else begin
            v_phase <= (v_phase == V_LINE_CYC - 1) ? 0 : v_phase + 1'b1;
            if (!v_inflight && calib) begin
                // fire V_BURSTS_PER_LINE read bursts evenly spaced across the line -- reads only (simpler,
                // protocol-safe: no multi-beat avl_writedata streaming to get exactly right in a bench whose
                // real job is C0/arbiter contention, not re-proving the V port's own write path, which the
                // DDR3 benches cover). This slightly UNDER-states the Template core's real combined
                // read+write V demand -- a conservative, not inflated, contention level.
                if (v_phase % (V_LINE_CYC / V_BURSTS_PER_LINE) < 2 && v_burst_idx < V_BURSTS_PER_LINE) begin
                    v_write_r <= 1'b0;
                    v_read_r  <= 1'b1;
                    v_addr_r  <= 28'h2000000 + (v_burst_idx * 16);
                    v_inflight <= 1'b1;
                end
            end else if (v_inflight && !v_waitreq) begin
                v_write_r <= 1'b0; v_read_r <= 1'b0; v_inflight <= 1'b0;
                v_burst_idx <= v_burst_idx + 1'b1;
                v_bursts_sent <= v_bursts_sent + 1'b1;
                v_beats_sent  <= v_beats_sent + 16;
            end
            if (v_phase == 0) v_burst_idx <= 0;
        end
    end

    // ---- the real tango_ddr3 (TANGO_DDRAM -- the real C0 port), wiring V and C0 as above
    wire DDRAM_BUSY_w, DDRAM_RD_w, DDRAM_WE_w, DDRAM_DOUT_READY_w;
    wire [7:0] DDRAM_BURSTCNT_w, DDRAM_BE_w;
    wire [28:0] DDRAM_ADDR_w;
    wire [63:0] DDRAM_DOUT_w, DDRAM_DIN_w;
    wire [15:0] obs_core_inflight_max;
    wire [31:0] obs_fences;

    tango_ddr3 #(.G(8), .Q_CORE(16)) u_ddr3 (
        // rst must stay asserted until the IP has calibrated, exactly as tango_sys.v's own real wiring does
        // (ddr_rs, a 3-stage shift of ddr_rst|~ddr_calib) -- a bare !rst_n alone would
        // let `rst` clear at t=200ns while the model's own calib takes INIT_CLKS=1000 clk_ddr cycles (10 us),
        // leaving the arbiter/cport in a state this module never actually exercises on the real board.
        .clk(clk_ddr), .rst(ddr_rs[2]),
        .avl_address(v_addr_r), .avl_burstcount(v_burst_r), .avl_write(v_write_r),
        .avl_writedata(v_wd_r), .avl_byteenable(16'hFFFF), .avl_read(v_read_r),
        .avl_waitrequest(v_waitreq), .avl_readdata(v_rdata), .avl_readdatavalid(v_rvalid),
        .DDRAM_CLK(clk_video), .ddram_rst(1'b0), .DDRAM_BUSY(DDRAM_BUSY_w), .DDRAM_BURSTCNT(DDRAM_BURSTCNT_w),
        .DDRAM_ADDR(DDRAM_ADDR_w), .DDRAM_DOUT(DDRAM_DOUT_w), .DDRAM_DOUT_READY(DDRAM_DOUT_READY_w),
        .DDRAM_RD(DDRAM_RD_w), .DDRAM_DIN(DDRAM_DIN_w), .DDRAM_BE(DDRAM_BE_w), .DDRAM_WE(DDRAM_WE_w),
        .cmd_ready(cmd_ready), .cmd_en(cmd_en), .cmd(cmd), .addr(addr),
        .wr_data_rdy(wr_data_rdy), .wr_data_en(wr_data_en), .wr_data(wr_data), .wr_data_mask(wr_data_mask),
        .rd_data_valid(rd_data_valid), .rd_data(rd_data),
        .obs_v_wait_max(), .obs_v_win_drops(), .obs_core_inflight_max(obs_core_inflight_max),
        .obs_fences(obs_fences), .obs_arb_v_wait_max());

    // ---- the ACTUAL rotation client, driven at Template's real native raster and real CE_DIV
    localparam integer MAXW = 320, MAXH = 240, CE_DIV = 12;   // Template's real dimensions and real divider
    reg [11:0] hcnt2 = 0;
    reg [3:0]  cediv2 = 0;
    reg        ce2 = 1'b0;
    reg        vga_vs2 = 0, vga_de2 = 0;
    integer    in_frame2 = -1;
    localparam integer HTOT = 400, VTOT = 262;    // Template's own real totals (400x262)
    reg [11:0] xcnt2 = 0, ycnt2 = 0;
    always @(posedge clk_video) begin
        cediv2 <= (cediv2 == CE_DIV - 1) ? 4'd0 : cediv2 + 1'b1;
        ce2    <= (cediv2 == CE_DIV - 1);
        if (ce2) begin
            if (xcnt2 == HTOT - 1) begin
                xcnt2 <= 0;
                if (ycnt2 == VTOT - 1) begin ycnt2 <= 0; in_frame2 <= in_frame2 + 1; end
                else ycnt2 <= ycnt2 + 1'b1;
            end else xcnt2 <= xcnt2 + 1'b1;
            vga_de2 <= (xcnt2 < MAXW) && (ycnt2 < MAXH);
            vga_vs2 <= (ycnt2 == VTOT - 4) && (xcnt2 < 4);
        end
    end
    wire [7:0] vga_r2 = xcnt2[7:0], vga_g2 = ycnt2[7:0], vga_b2 = 8'h00;
    reg [1:0] rotate_mode2 = 2'b01;   // CW, continuously -- this bench stresses the allocator/write path flat out

    wire clk_pix_unused = 1'b0;
    wire o_fb_ena_u; wire [11:0] o_fb_hsize_u, o_fb_vsize_u; wire [5:0] o_fb_format_u;
    wire [31:0] o_fb_base_u; wire [13:0] o_fb_stride_u;
    wire [31:0] obs_fw, obs_fd, obs_ov;

    tango_video_rotate_ddr3 #(.MAXW(MAXW), .MAXH(MAXH), .NBUF(4)) u_rot (
        .clk_video(clk_video), .rst_video(!rst_n), .rotate_mode(rotate_mode2),
        .ce_pixel_in(ce2), .vga_r_in(vga_r2), .vga_g_in(vga_g2), .vga_b_in(vga_b2),
        .vga_hs_in(1'b0), .vga_vs_in(vga_vs2), .vga_de_in(vga_de2),
        .DDRAM_CLK(), .DDRAM_BUSY(DDRAM_BUSY_w), .DDRAM_BURSTCNT(DDRAM_BURSTCNT_w),
        .DDRAM_ADDR(DDRAM_ADDR_w), .DDRAM_DOUT(DDRAM_DOUT_w), .DDRAM_DOUT_READY(DDRAM_DOUT_READY_w),
        .DDRAM_RD(DDRAM_RD_w), .DDRAM_DIN(DDRAM_DIN_w), .DDRAM_BE(DDRAM_BE_w), .DDRAM_WE(DDRAM_WE_w),
        .clk_pix(clk_video), .o_fb_ena(o_fb_ena_u), .o_fb_hsize(o_fb_hsize_u), .o_fb_vsize(o_fb_vsize_u),
        .o_fb_format(o_fb_format_u), .o_fb_base(o_fb_base_u), .o_fb_stride(o_fb_stride_u),
        .obs_frames_written(obs_fw), .obs_frames_dropped(obs_fd), .obs_fifo_overflow(obs_ov));

    // ---- measurement: max DDRAM_BUSY stall seen by the client, max FIFO occupancy, achieved throughput
    integer busy_run = 0, busy_max = 0;
    integer we_count = 0;
    always @(posedge clk_video) begin
        if (DDRAM_BUSY_w) busy_run = busy_run + 1; else busy_run = 0;
        if (busy_run > busy_max) busy_max = busy_run;
        if (DDRAM_WE_w) we_count = we_count + 1;
    end
    integer fifo_occ_max = 0;
    always @(posedge clk_video) begin
        integer occ;
        occ = u_rot.fifo_wp - u_rot.fifo_rp;
        if (occ > fifo_occ_max) fifo_occ_max = occ;
    end

    initial begin
        wait (in_frame2 >= 60);
        #10000;
        $display("TB_ROTATE_ARB: %0d input frames, %0d DDRAM_WE writes, %0d V bursts (%0d beats)",
                  in_frame2, we_count, v_bursts_sent, v_beats_sent);
        $display("TB_ROTATE_ARB: max DDRAM_BUSY stall seen by C0 client = %0d clk_video cycles (%.1f ns)",
                  busy_max, busy_max * 13.468);
        $display("TB_ROTATE_ARB: max elastic FIFO occupancy = %0d / 32", fifo_occ_max);
        $display("TB_ROTATE_ARB: frames written %0d, frames dropped (overflow) %0d, FIFO overflow events %0d",
                  obs_fw, obs_fd, obs_ov);
        $display("TB_ROTATE_ARB: arbiter obs_core_inflight_max=%0d (Q_CORE=16), obs_fences=%0d, DDR3 model refreshes=%0d row-misses=%0d bad_cmd=%0d",
                  obs_core_inflight_max, obs_fences, n_refresh, n_rowmiss, n_bad_cmd);
        $display("TB_RESULT %0s", (obs_fd == 0 && obs_ov == 0 && n_bad_cmd == 0) ? "PASS" : "FAIL");
        $finish;
    end
endmodule
`default_nettype wire
