// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_rotate_galaga -- tb_rotate_arb's machinery (REAL tango_ddr3 arbiter + cport + the silicon-calibrated
// DDR3 IP model + a synthetic V-port read load), with the rotation client driven at GALAGA's real raster
// instead of the Template's, PLUS a live per-write position check (tb_rotate_golden's decode). It covers a
// case silicon showed: an arcade core's (Galaga) rotated picture displaced by ~12+ native columns (the
// "1UP HIGH SCORE" row clipped at the top in CW) while the arcade template core's corner marker landed
// exactly.
//
// Galaga's raster, from MiSTer's Galaga core (gen_video.vhd, galaga.vhd) and the core's Tango top:
//   clk_video = CLK_SYS = clock_18 (18.432 MHz); CE_PIXEL = 2 of 6 clock_18 slots (galaga.vhd:449, 6.144 MHz);
//   hcnt 128..511 (384 per line); vcnt 0..263, stepping at hcnt 191; hblank 1 at hcnt 152, 0 at 248
//   (active = hcnt 248..511,128..151 = 288 pixels); vblank 1 at vcnt 240, 0 at vcnt 16 (224 lines);
//   vsync ('1') vcnt 260..3; VGA_VS = ~video_vs (the core's Tango top); VGA_DE = ~(hblank|vblank).
// Tags: R = native x[7:0], G = native y[7:0], B = {7'd0, x[8]} (x runs to 287), decoded by ascal's 32bpp rule.
//==============================================================================
`default_nettype none
`timescale 1ns/1ps

module tb_rotate_galaga;
    // ---- clk_ddr (100 MHz, the IP's clk_out) and clk_video (74.25 MHz, Template's real CLK_VIDEO)
    reg clk_ddr = 1'b0;
    always #5 clk_ddr = ~clk_ddr;
    reg clk_video = 1'b0;
    initial begin #3; forever #27.127 clk_video = ~clk_video; end  // 18.432 MHz (clock_18), independent phase

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

    // ---- the ACTUAL rotation client, driven at Galaga's real raster (header)
    localparam integer MAXW = 288, MAXH = 240, NH = 224;
    reg [2:0]  slot = 3'd0;
    wire       ce2 = (slot == 3'd3) || (slot == 3'd0);           // galaga.vhd:449
    wire       ena_vg = (slot == 3'd5) || (slot == 3'd2);        // ena_vidgen, galaga.vhd:433
    reg [8:0]  hc = 9'd128, vc = 9'd15;
    reg        hbl = 1'b0, vbl = 1'b1, vsy = 1'b0;
    integer    in_frame2 = -1;
    always @(posedge clk_video) begin
        slot <= (slot == 3'd5) ? 3'd0 : slot + 3'd1;
        if (ena_vg) begin                                         // gen_video.vhd's process, v/h_offset 0
            hc <= (hc == 9'd511) ? 9'd128 : hc + 9'd1;
            if (hc == 9'd191) begin
                if (vc == 9'd263) begin vc <= 9'd0; in_frame2 <= in_frame2 + 1; end else vc <= vc + 9'd1;
            end
            if (vc == 9'd260) vsy <= 1'b1; else if (vc == 9'd4) vsy <= 1'b0;
            if (hc == 9'd152) hbl <= 1'b1; else if (hc == 9'd248) hbl <= 1'b0;
            if (vc == 9'd240) vbl <= 1'b1; else if (vc == 9'd16) vbl <= 1'b0;
        end
    end
    // native coordinates of the current pixel (the tag source): x counts DE pixels on CE, y counts lines
    reg [8:0] xcnt2 = 0; reg [8:0] ycnt2 = 0; reg de_d = 1'b0, vs_d = 1'b0;
    wire vga_de2 = !(hbl | vbl);
    wire vga_vs2 = !vsy;
    always @(posedge clk_video) if (ce2) begin
        de_d <= vga_de2; vs_d <= vga_vs2;
        if (vga_de2) xcnt2 <= xcnt2 + 9'd1;
        else if (de_d) begin xcnt2 <= 9'd0; ycnt2 <= ycnt2 + 9'd1; end
        if (vga_vs2 && !vs_d) ycnt2 <= 9'd0;
    end
    wire [7:0] vga_r2 = xcnt2[7:0], vga_g2 = ycnt2[7:0], vga_b2 = {7'd0, xcnt2[8]};
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

    // ---- LIVE per-write position check (tb_rotate_golden's decode; ascal 32bpp rule byte0 R byte1 G byte2 B)
    integer live_checked = 0, live_errs = 0, first_err_off = 0, outside = 0, outside_late = 0;
    always @(posedge clk_video) begin
        if (DDRAM_WE_w) begin : chk
            integer k, b, o, yo_l, xo_l, ex, ey, gx, gy;
            reg [31:0] px32;
            b = -1;
            for (k = 0; k < 4; k = k + 1)
                if (DDRAM_ADDR_w[24:0] >= u_rot.buf_base(k[1:0]) &&
                    DDRAM_ADDR_w[24:0] <  u_rot.buf_base(k[1:0]) + u_rot.stride_w * MAXW) b = k;
            if (b < 0) begin
                // a write no buffer's window (at the CURRENT stride) contains: report it, never skip silently
                outside = outside + 1;
                if (in_frame2 >= 2) outside_late = outside_late + 1;
                if (outside == 1 || (in_frame2 >= 2 && outside_late == 1))
                    $display("OUTSIDE write t=%0t in_frame=%0d addr=%07h stride_w=%0d hsz=%0d vsz=%0d wbuf=%0d",
                             $time, in_frame2, DDRAM_ADDR_w, u_rot.stride_w, u_rot.hsz, u_rot.vsz, u_rot.wbuf);
            end
            if (b >= 0) begin
                o    = DDRAM_ADDR_w[24:0] - u_rot.buf_base(b[1:0]);
                yo_l = o / u_rot.stride_w;
                xo_l = (o % u_rot.stride_w) * 2 + (DDRAM_BE_w[4] ? 1 : 0);
                ex   = yo_l;                 // CW: out(yo,xo) = src(row NH-1-xo, col yo)
                ey   = NH - 1 - xo_l;
                px32 = DDRAM_BE_w[4] ? DDRAM_DIN_w[63:32] : DDRAM_DIN_w[31:0];
                gx   = {px32[16], px32[7:0]};
                gy   = px32[15:8];
                live_checked = live_checked + 1;
                if (gx != ex || gy != ey) begin
                    if (live_errs == 0) first_err_off = gx - ex;
                    live_errs = live_errs + 1;
                    if (live_errs <= 10) $display("POS MISMATCH t=%0t buf=%0d out(yo=%0d,xo=%0d): got src(x=%0d,y=%0d) want src(x=%0d,y=%0d)",
                                                   $time, b, yo_l, xo_l, gx, gy, ex, ey);
                end
            end
        end
    end

    initial begin
        wait (in_frame2 >= 8);
        #10000;
        $display("TB_ROTATE_GALAGA: %0d input frames, %0d DDRAM_WE writes, %0d V bursts (%0d beats)",
                  in_frame2, we_count, v_bursts_sent, v_beats_sent);
        $display("TB_ROTATE_GALAGA: max DDRAM_BUSY stall seen by C0 client = %0d clk_video cycles (%.1f ns)",
                  busy_max, busy_max * 54.254);
        $display("TB_ROTATE_GALAGA: max elastic FIFO occupancy = %0d / 32", fifo_occ_max);
        $display("TB_ROTATE_GALAGA: frames written %0d, frames dropped (overflow) %0d, FIFO overflow events %0d",
                  obs_fw, obs_fd, obs_ov);
        $display("TB_ROTATE_GALAGA: arbiter obs_core_inflight_max=%0d (Q_CORE=16), obs_fences=%0d, DDR3 model refreshes=%0d row-misses=%0d bad_cmd=%0d",
                  obs_core_inflight_max, obs_fences, n_refresh, n_rowmiss, n_bad_cmd);
        $display("TB_ROTATE_GALAGA: o_fb_hsize=%0d o_fb_vsize=%0d (want 224 x 288), %0d writes position-checked, %0d mismatches (first offset %0d), %0d outside every buffer (%0d from input frame 2 on)",
                 o_fb_hsize_u, o_fb_vsize_u, live_checked, live_errs, first_err_off, outside, outside_late);
        $display("TB_RESULT %0s", (obs_fd == 0 && obs_ov == 0 && n_bad_cmd == 0 && live_errs == 0 && live_checked > 0 && outside_late == 0
                                   && o_fb_hsize_u == 12'd224 && o_fb_vsize_u == 12'd288) ? "PASS" : "FAIL");
        $finish;
    end
endmodule
`default_nettype wire
