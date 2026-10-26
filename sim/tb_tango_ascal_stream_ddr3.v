// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_tango_ascal_stream_ddr3 -- the DDR3 scaler port (tango_ddr3) under ascal's real traffic, a core streaming flat out
// and stall patterns.  Derived from an earlier bench of the same name without _ddr3,
// retargeted:
//   - ascal at N_DW 128 (MiSTer's): bursts of 16 beats of 128 bits, avl_clk = clk_ddr = 100 MHz, RAMBASE 0x2000_0000,
//     RAMSIZE 8 MB (Template_MiSTer sys/sys_top.v 714-741); a line's stride is its bursts x 256 B;
//   - the input (ascal.vhd 1247-1253: a write burst each 256 B of a line, the last one partial), triple buffered
//     (393-399 buf_next); a write request while the one before is still pending merges (1638-1641): a lost burst.
//     Default: a Genesis H40 frame, 224 lines of 320 px at 24 bpp = 4 bursts, a pixel each 149.0 ns (8 MCLK), lines
//     of 63.69 us, 262 a frame (59.92 Hz).  -DLAUNCHER: the launcher (menu-tango's launcher
//     core), 640 x 360 native = 8 bursts a line (the last half full), a pixel each 2 clocks of
//     74.25 MHz on even beam lines: native lines of 44.44 us, 375 a frame;
//   - the output, 720p60 (lines of 22.22 us, 750 a frame), ascal's read machine as the source has it:
//       * an hsync at each line sets the pending flag o_hsp; one that comes while it is still set is LOST (1978-1983,
//         2001: a flag), and the picture below would move down ("bounce");
//       * sDISP takes the flag; sHSYNC decides whether this line reads a source line (the DDA: the preload's lines 0
//         and 1 at the hsyncs of lines 747 and 748 (fload, 1976-1982), then source line n at output line
//         (n - 2) x 720 / lines, one per hsync); sREAD issues a burst when o_readlev < 2 (2054), two o_clk after it is
//         entered (o_adrsb); sWAITREAD waits for the command's acknowledge (readack, synchronised: 1962-1964), then
//         the next burst or sDISP;
//       * o_readlev counts bursts issued and not yet copied, across lines: a burst's slot frees at the END of its
//         copy into the line buffer (lev_dec_v 2190, 2242-2247); the copies run one after another, each starting
//         when its data are in (readdataack at the last beat, 1745-1748, synchronised) and the copy before has ended,
//         and taking one o_clk per output pixel it makes (o_dcpt + 1, 2162): the burst's pixels x WIN_W / input
//         width / 74.25 MHz;
//       * the Avalon machine: one command at a time, a pending write first (1687-1731); requests and acknowledges
//         cross o_clk <-> avl_clk through 2-flop synchronisers (4 clocks each way here);
//   - the memory: tango_ddr3 on the IP model (refresh 28 clocks every 780, row misses, in order), with the core port
//     driven by an adversary on a 96 MHz clock (-DNOCORE: none): ADV 0 single-beat reads and writes (1 in 4) to random
//     rows of one bank (every IP word a row miss), back to back; ADV 1 bursts of 1-255 beats at random addresses of the
//     core window, half writes, back to back; ADV 2 a probe: single-beat reads to random rows, one at a time, each
//     timed (the core-read latency bound, 2.5 us);
//   - stall patterns, every STALL_EVERY frames at output line STALL_LINE (747 = the preload, 160 = where
//     source line 50 is read, -1 = a random line): STALL=1 cmd_ready low for STALL_LEN clocks (a pll_stop), STALL=2
//     8 refreshes back to back; IPHASE: the input's line when the output starts (the phase sweep).
// Pass criteria, every output frame after the first two:
//   sHSYNC -> sDISP, timed from the hsync (any wait for earlier copies included), <= DISP_PCT % of 22.22 us (50 by
//   default); the preload's line 0 <= DISP_PCT % of 25.7 us, and never past 25.7 us (the DDA init would be lost); no
//   lost hsync; every frame reads (lines + 2) x bursts; source line 100's read starts at the same time after the
//   preload's hsync every frame (+-1 clock); 0 lost write bursts; every beat of the written lines = what the input
//   wrote there in that buffer's newest frame; avl_waitrequest held with a command present <= 32 clocks; ADV 2: the
//   core read latency <= 2.5 us.  Also printed: the longest line from its hsync to its last copy, the longest wait of
//   an hsync before sHSYNC, the read-machine floor (NB - 2 copies).  TB_RESULT PASS / FAIL.
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_tango_ascal_stream_ddr3;
`include "tango_ddr3_silicon.vh"                       // the model calibrated on silicon (D1)
`ifndef Q_CORE
`define Q_CORE 8
`endif
    localparam [31:0] RAMBASE = 32'h2000_0000;
    localparam integer OLINE10 = 22222, OLINE = 2222;         // ten output lines, one (clk_ddr)
    localparam integer PRE_HARD = 2570;
    // run-time knobs (plusargs, defaults in brackets): the source +i_w +i_h +i_tot +iline (clocks a line) +i_pxc
    // (clocks x 100 a pixel) +i_t0 (the first burst's request in the line) [the Genesis H40: 320 224 262 6369 1490 186;
    // +launcher: 640 360 375 4444 269 280], +win_w [1280], +core [1: the adversary on; IDEAL builds: none] +adv [0],
    // +stall [0] +stall_len [1000] +stall_line [747; -1 random] +stall_every [1], +iphase [0], +oframes [8], the margin
    // limits +disp_lim [1111] +pre_lim [1285] (clocks: the runner gives floor + half the slack)
    integer I_W, I_H, I_TOT, ILINE, I_PXC, I_T0, NB, NL, WIN_W, CORE_ON, ADV, STALL, STALL_LEN, STALL_LINE, STALL_EVERY;
    integer IPHASE, OFRAMES, DISP_MAX, PRE_MAX;
    localparam integer SYNC = 4, SREAD_D = 3;                 // o_clk <-> avl_clk crossings; sREAD's o_adrsb
    localparam real    CORE_LAT_MAX = 2500.0;                 // ns, the core-read latency bound
    // burst k's input pixels (x3; bursts are 256 B = 85.33 px), its copy time and its write time in the input line
    function integer pend3(input integer k);                  // 3 x the pixel after burst k
        pend3 = ((k + 1) * 256 < 3 * I_W) ? (k + 1) * 256 : 3 * I_W;
    endfunction
    function integer tcopy(input integer k);                  // (pixels x WIN_W / I_W) / 74.25 MHz, in 10 ns clocks
        integer opx100;                                       // the output pixels x 100 (32-bit safe in two steps)
        begin
            opx100 = ((pend3(k) - ((k == 0) ? 0 : pend3(k - 1))) * WIN_W * 100) / (3 * I_W);
            tcopy = (opx100 * 100) / 7425;
        end
    endfunction
    function integer twrite(input integer k);                 // the request time of burst k in the input line
        twrite = I_T0 + ((pend3(k) - pend3(0)) * I_PXC) / 300;
    endfunction

    reg clk = 1'b0, cclk = 1'b0;
    always #5      clk  = ~clk;
    always #5.2083 cclk = ~cclk;
    reg rst = 1'b1, crst = 1'b1;
    reg [31:0] cyc = 32'd0;
    always @(posedge clk) cyc <= cyc + 32'd1;

    // ---- the model and the DUT
    wire         init_calib_complete, cmd_ready, cmd_en, wr_data_rdy, wr_data_en, rd_data_valid, rd_data_end;
    wire [2:0]   cmd;
    wire [28:0]  addr;
    wire [255:0] wr_data, rd_data;
    wire [31:0]  wr_data_mask, n_refresh, n_rowmiss, n_bad_cmd;
    reg          m_rst_n = 1'b0, force_stall = 1'b0, ref_burst = 1'b0;
    reg  [27:0]  avl_address = 0;
    reg          avl_write = 1'b0, avl_read = 1'b0;
    reg  [127:0] avl_writedata = 0;
    wire         avl_waitrequest, avl_readdatavalid;
    wire [127:0] avl_readdata;
    wire         DDRAM_BUSY, DDRAM_DOUT_READY;
    reg  [28:0]  DDRAM_ADDR = 0;
    reg  [7:0]   DDRAM_BURSTCNT = 8'd1;
    wire [63:0]  DDRAM_DOUT;
    reg          DDRAM_RD = 1'b0, DDRAM_WE = 1'b0;
    reg  [63:0]  DDRAM_DIN = 64'd0;
    wire [15:0]  v_wait_max, core_infl_max;
    wire [31:0]  v_win_drops, fences, arb_v_wait;
`ifndef IDEAL
    tango_ddr3_model #(.MEM_LOG2(20), .INIT_CLKS(100), .CMD_DEPTH(SIL_CMD_DEPTH), .WDATA_DEPTH(SIL_WDATA_DEPTH),
                       .RD_LAT(SIL_RD_LAT), .RD_JITTER(SIL_RD_JITTER), .T_BL(1), .T_ROWMISS(SIL_T_ROWMISS),
                       .T_TURN(SIL_T_TURN), .REF_PERIOD(SIL_REF_PERIOD), .REF_LEN(SIL_REF_LEN)) u_m (
        .clk(clk), .rst_n(m_rst_n), .init_calib_complete(init_calib_complete),
        .cmd(cmd), .cmd_en(cmd_en), .addr(addr), .cmd_ready(cmd_ready),
        .wr_data(wr_data), .wr_data_en(wr_data_en), .wr_data_end(wr_data_en), .wr_data_mask(wr_data_mask),
        .wr_data_rdy(wr_data_rdy), .rd_data(rd_data), .rd_data_valid(rd_data_valid), .rd_data_end(rd_data_end),
        .force_stall(force_stall), .ref_burst(ref_burst), .fault_en(1'b0), .fault_word(25'd0), .fault_xor(256'd0),
        .n_refresh(n_refresh), .n_rowmiss(n_rowmiss), .n_bad_cmd(n_bad_cmd));
    tango_ddr3 #(.G(8), .Q_CORE(`Q_CORE)) dut (
        .clk(clk), .rst(rst),
        .avl_address(avl_address), .avl_burstcount(8'd16), .avl_write(avl_write),
        .avl_writedata(avl_writedata), .avl_byteenable(16'hFFFF), .avl_read(avl_read),
        .avl_waitrequest(avl_waitrequest), .avl_readdata(avl_readdata), .avl_readdatavalid(avl_readdatavalid),
        .DDRAM_CLK(cclk), .ddram_rst(crst), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT),
        .DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
        .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(8'hFF), .DDRAM_WE(DDRAM_WE),
        .cmd_ready(cmd_ready), .cmd_en(cmd_en), .cmd(cmd), .addr(addr), .wr_data_rdy(wr_data_rdy),
        .wr_data_en(wr_data_en), .wr_data(wr_data), .wr_data_mask(wr_data_mask),
        .rd_data_valid(rd_data_valid), .rd_data(rd_data),
        .obs_v_wait_max(v_wait_max), .obs_v_win_drops(v_win_drops), .obs_core_inflight_max(core_infl_max),
        .obs_fences(fences), .obs_arb_v_wait_max(arb_v_wait));
    initial begin
        #100 m_rst_n = 1'b1;
        wait (init_calib_complete);
        @(posedge clk); rst = 1'b0;
        @(posedge cclk); crst = 1'b0;
    end
`else
    // an instant memory (the floor: ascal's read machine alone): no waitrequest, a read's 16 beats from the clock
    // after it is taken, or right after the read before it (ascal has up to two out: o_readlev < 2) -- a queue of
    // taken reads, so a second read never cuts the first one's beats short
    reg  [127:0] imem [0:(1 << 19) - 1];                 // {buffer, line, burst, beat}
    reg  [18:0]  id_a = 0;
    reg  [4:0]   id_n = 0;
    reg          id_v = 1'b0;
    reg  [127:0] id_d = 0;
    reg  [14:0]  idq [0:3];                              // taken reads waiting: {buffer, line, burst}
    reg  [1:0]   idq_w = 0, idq_r = 0;
    reg  [2:0]   idq_n = 0;
    assign avl_waitrequest = 1'b0; assign avl_readdatavalid = id_v; assign avl_readdata = id_d;
    assign n_refresh = 0; assign n_rowmiss = 0; assign n_bad_cmd = 0; assign core_infl_max = 0; assign fences = 0;
    assign arb_v_wait = 0; assign v_wait_max = 0; assign v_win_drops = 0;
    assign DDRAM_BUSY = 1'b1; assign DDRAM_DOUT_READY = 1'b0; assign DDRAM_DOUT = 64'd0;
    initial begin #100; @(posedge clk); rst = 1'b0; @(posedge cclk); crst = 1'b0; end
`endif

    // ---- the core adversary (DDRAM_CLK 96 MHz), a negedge machine: (A) retire what the last rising edge took,
    // (B) put up the next command, (C) note whether the next rising edge takes the bus (!DDRAM_BUSY), (D) returns.
    // Addresses: byte 0x3000_0000 + DDRAM_ADDR * 8 (banks 6-7); DDRAM_ADDR[21:20] = 3 keeps the model's 2^20 words
    // clear of the scaler's 24 MB.
    reg  [31:0] clfsr = 32'h1357_9BDF;
    integer     c_ops = 0, c_beats_left = 0, c_rd_out = 0, c_got = 0, c_tw = 0, c_tr = 0, c_lat_n = 0;
    real        c_bytes = 0.0, c_bytes0 = 0.0, c_t0 = 0.0;           // bytes moved; at checking start; its time
    reg         c_acc = 1'b0;
    real        c_lat_max = 0.0, c_lat_sum = 0.0;
    real        c_tq [0:1023];                          // read issue times, in order
    integer     c_tbeats [0:1023];
    task automatic c_next;
        begin
            clfsr = {clfsr[30:0], clfsr[31] ^ clfsr[21] ^ clfsr[1] ^ clfsr[0]};
            if (ADV == 1) begin
                DDRAM_BURSTCNT = (clfsr[7:0] == 8'd0) ? 8'd1 : clfsr[7:0];
                // DDRAM_ADDR[19] = 0: a 255-beat burst cannot carry into bit 20 (the model's alias of the scaler)
                DDRAM_ADDR = {4'b0011, clfsr[24], clfsr[14:13], 2'b11, 1'b0, clfsr[22:15], clfsr[10:0]};
                DDRAM_WE = clfsr[25];
            end else begin
                DDRAM_BURSTCNT = 8'd1;
                DDRAM_ADDR = {4'b0011, 1'b0, clfsr[14:13], 2'b11, clfsr[10:0], clfsr[23:15]};
                DDRAM_WE = (ADV == 0) && clfsr[24] && clfsr[25];      // ADV 0: 1 in 4 writes; ADV 2: reads only
            end
            DDRAM_RD = !DDRAM_WE;
            DDRAM_DIN = {32'hC0DE_0000, clfsr};
            c_beats_left = DDRAM_WE ? DDRAM_BURSTCNT : 0;
        end
    endtask
    always @(negedge cclk) begin
        if (!crst && CORE_ON != 0) begin
            // (A)
            if (c_acc) begin
                if (DDRAM_WE) begin
                    c_bytes = c_bytes + 8.0;
                    if (c_beats_left > 1) begin c_beats_left = c_beats_left - 1; DDRAM_DIN = DDRAM_DIN + 64'd1; end
                    else begin c_ops = c_ops + 1; DDRAM_WE = 1'b0; end
                end else begin
                    c_ops = c_ops + 1;
                    c_tq[c_tw % 1024] = $realtime; c_tbeats[c_tw % 1024] = DDRAM_BURSTCNT; c_tw = c_tw + 1;
                    c_rd_out = c_rd_out + 1;
                    DDRAM_RD = 1'b0;
                end
            end
            // (B)
            if (!DDRAM_RD && !DDRAM_WE && (ADV != 2 || c_rd_out == 0)) c_next;
            // (C)
            c_acc = (DDRAM_RD || DDRAM_WE) && !DDRAM_BUSY;
            // (D)
            if (DDRAM_DOUT_READY) begin
                c_got = c_got + 1; c_bytes = c_bytes + 8.0;
                if (c_got == c_tbeats[c_tr % 1024]) begin
                    if ($realtime - c_tq[c_tr % 1024] > c_lat_max) c_lat_max = $realtime - c_tq[c_tr % 1024];
                    c_lat_sum = c_lat_sum + ($realtime - c_tq[c_tr % 1024]); c_lat_n = c_lat_n + 1;
                    c_tr = c_tr + 1; c_got = 0; c_rd_out = c_rd_out - 1;
                end
            end
        end
    end

    // ---- the picture pattern: {input frame, buffer, line, burst, beat} and its complement (128 bits)
    function [127:0] pat(input [15:0] f, input [1:0] b, input [8:0] l, input [3:0] u, input [3:0] t);
        pat = {f, b, l, u, t, 29'h15A5_AC3C, ~{f, b, l, u, t}, 29'h0A5A_53C3};
    endfunction
    function [1:0] bnext(input [1:0] a, input [1:0] b);      // ascal.vhd 393-399 buf_next
        bnext = ((a == 2'd0 && b == 2'd1) || (a == 2'd1 && b == 2'd0)) ? 2'd2 :
                ((a == 2'd1 && b == 2'd2) || (a == 2'd2 && b == 2'd1)) ? 2'd0 : 2'd1;
    endfunction
    function [27:0] vaddr(input [1:0] b, input [8:0] l, input [3:0] u);   // 128-bit word address
        reg [31:0] x;
        begin x = RAMBASE + ({30'd0, b} << 23) + ({23'd0, l} * NB + {28'd0, u}) * 256; vaddr = x[31:4]; end
    endfunction
    function [9:0] need(input [8:0] n);                       // the output line whose hsync reads source line n
        reg [31:0] q;
        begin
            q = ({23'd0, n} - 32'd2) * 32'd720 / I_H;
            need = (n < 9'd2) ? (10'd747 + {9'd0, n[0]}) : q[9:0];
        end
    endfunction

    // ---- the input side
    reg  [31:0] i_t = 32'd0;
    reg  [8:0]  i_line = 9'd0;
    reg  [15:0] i_frame = 16'd1;
    reg  [1:0]  i_buf = 2'd0, i_done = 2'd2;
    reg  [15:0] i_fof [0:3];
    initial begin i_fof[0] = 16'd0; i_fof[1] = 16'd0; i_fof[2] = 16'd0; i_fof[3] = 16'd0; end
    reg         wq_v = 1'b0;
    reg  [1:0]  wq_buf = 2'd0;
    reg  [3:0]  wq_u = 4'd0;
    reg  [8:0]  wq_l = 9'd0;
    reg  [15:0] wq_f = 16'd0;
    reg  [31:0] n_wbrk = 0, n_wbursts = 0;

    // ---- the output side: timing, ascal's read machine, the copies
    reg  [31:0] o_t10 = 0;
    reg  [9:0]  o_line = 10'd0;
    reg  [1:0]  o_buf = 2'd1;
    reg  [31:0] o_frames = 0;
    reg         o_live = 1'b0;
    reg  [8:0]  o_next = 9'd0;
    reg         hsp = 1'b0;                                   // o_hsp
    reg  [31:0] hs_t = 0;
    reg  [9:0]  hs_line = 0;
    localparam [1:0] F_DISP = 2'd0, F_READ = 2'd1, F_WAIT = 2'd2;
    reg  [1:0]  fs = F_DISP;
    reg  [3:0]  fk = 0;                                       // the burst being issued
    reg  [8:0]  f_line = 0;
    reg  [31:0] f_t = 0, f_wait = 0;
    reg  [2:0]  readlev = 0;
    reg  [SYNC-1:0] req_sr = 0, ack_sr = 0, dat_sr = 0;
    reg         rq_pend = 1'b0;
    reg  [8:0]  rq_line = 0;
    reg  [3:0]  rq_u = 0;
    // the copies: bursts whose data are in, in order {line, burst, the line's hsync time}
    reg  [8:0]  cq_line [0:3];
    reg  [3:0]  cq_u [0:3];
    reg  [31:0] cq_t [0:3];
    reg  [1:0]  cq_wp = 0, cq_rp = 0;
    reg  [2:0]  cq_n = 3'd0;
    reg         copying = 1'b0;
    reg  [31:0] copy_end = 0;
    reg  [3:0]  cp_u = 0;
    reg  [31:0] cp_t = 0;
    // the Avalon read beats: reads in flight, in order
    reg  [8:0]  fl_line [0:3];
    reg  [3:0]  fl_u [0:3];
    reg  [1:0]  fl_buf [0:3];
    reg  [15:0] fl_f [0:3];
    reg  [31:0] fl_t [0:3];
    reg  [1:0]  fl_wp = 0, fl_rp = 0;
    reg  [3:0]  fl_beat = 0;
    // measures
    reg  [31:0] disp_max = 0, pre_max = 0, line_max = 0, hsw_max = 0, n_dispover = 0, n_preover = 0, n_prehard = 0;
    reg  [31:0] n_over100 = 0;
    reg  [31:0] n_hslost = 0, n_lines = 0, pre_t = 0, l100_min = 32'hFFFF_FFFF, l100_max = 0;
    reg  [31:0] fr_bursts = 0, n_frbad = 0;
    reg  [31:0] n_rbeats = 0, n_err = 0, n_unchecked = 0;
    reg  [31:0] n_unal = 0;                              // IP commands whose column is not 8-aligned (JEDEC BL8 wrap)
`ifndef IDEAL
    always @(posedge clk) if (cmd_en && addr[2:0] != 3'd0) n_unal <= n_unal + 1'b1;
`endif
    localparam [1:0] A_IDLE = 2'd0, A_WR = 2'd1, A_RD = 2'd2;
    reg  [1:0]  a_st = A_IDLE;
    reg  [4:0]  a_beat = 0;
    reg  [1:0]  a_buf = 0;
    reg  [3:0]  a_u = 0;
    reg  [8:0]  a_l = 0;
    reg  [15:0] a_f = 0;
    reg  [31:0] wait_c = 0, wait_max = 0, stall_line_now = 32'd747, n_stalls = 0, stall_left = 0;
    reg  [31:0] lfsr = 32'hACE1_2345;
    wire        checking = o_frames >= 32'd3;
    integer     floor_clk;
    integer     TCOPY [0:15], TWRITE [0:15];                  // per burst (tables: no function calls per clock)
    initial begin : g_floor                                   // the knobs; the tables; the read machine's time with an
        integer k;                                            // instant memory
        I_W = 320; I_H = 224; I_TOT = 262; ILINE = 6369; I_PXC = 1490; I_T0 = 186;
        if ($test$plusargs("launcher")) begin I_W = 640; I_H = 360; I_TOT = 375; ILINE = 4444; I_PXC = 269; I_T0 = 280; end
        if ($value$plusargs("i_w=%d", I_W)) ;
        if ($value$plusargs("i_h=%d", I_H)) ;
        if ($value$plusargs("i_tot=%d", I_TOT)) ;
        if ($value$plusargs("iline=%d", ILINE)) ;
        if ($value$plusargs("i_pxc=%d", I_PXC)) ;
        if ($value$plusargs("i_t0=%d", I_T0)) ;
        if (!$value$plusargs("win_w=%d", WIN_W)) WIN_W = 1280;
        if (!$value$plusargs("core=%d", CORE_ON)) CORE_ON = 1;
`ifdef IDEAL
        CORE_ON = 0;
`endif
        if (!$value$plusargs("adv=%d", ADV)) ADV = 0;
        if (!$value$plusargs("stall=%d", STALL)) STALL = 0;
        if (!$value$plusargs("stall_len=%d", STALL_LEN)) STALL_LEN = 1000;
        if (!$value$plusargs("stall_line=%d", STALL_LINE)) STALL_LINE = 747;
        if (!$value$plusargs("stall_every=%d", STALL_EVERY)) STALL_EVERY = 1;
        if (!$value$plusargs("iphase=%d", IPHASE)) IPHASE = 0;
        if (!$value$plusargs("oframes=%d", OFRAMES)) OFRAMES = 8;
        if (!$value$plusargs("disp_lim=%d", DISP_MAX)) DISP_MAX = 1111;
        if (!$value$plusargs("pre_lim=%d", PRE_MAX)) PRE_MAX = 1285;
        NB = (I_W * 3 + 255) / 256;                           // bursts a line
        NL = I_H + 2;                                         // source lines read a frame (the filter's bottom edge)
        i_line = IPHASE[8:0];
        for (k = 0; k < 16; k = k + 1) begin TCOPY[k] = tcopy(k); TWRITE[k] = twrite(k); end
        floor_clk = SREAD_D + 2 * SYNC;
        for (k = 0; k + 2 < NB; k = k + 1) floor_clk = floor_clk + tcopy(k) + 1;
    end

    always @(posedge clk) begin : g_main
        reg lev_inc, lev_dec, hs_now, cq_take, id_go, id_push, id_pop;
        integer wk;
        ref_burst <= 1'b0;
        lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
        if (stall_left != 0) begin stall_left <= stall_left - 1'b1; force_stall <= (stall_left > 1); end
        lev_inc = 1'b0; lev_dec = 1'b0; hs_now = 1'b0; cq_take = 1'b0; id_go = 1'b0; id_push = 1'b0; id_pop = 1'b0;
        if (!rst) begin
        //------------------------------------------------ input timing and write requests
        if (i_t == ILINE - 1) begin
            i_t <= 32'd0;
            if (i_line == I_TOT - 1) begin
                i_line <= 9'd0;
                i_fof[i_buf] <= (IPHASE != 0 && i_frame == 16'd1) ? 16'd0 : i_frame;
                i_done <= i_buf; i_buf <= bnext(i_buf, o_buf); i_frame <= i_frame + 16'd1;
            end else i_line <= i_line + 9'd1;
        end else i_t <= i_t + 32'd1;
        if (i_line < I_H)
            for (wk = 0; wk < NB; wk = wk + 1)
                if (i_t == TWRITE[wk]) begin
                    if (wq_v && checking) n_wbrk <= n_wbrk + 32'd1;
                    wq_v <= 1'b1; wq_buf <= i_buf; wq_l <= i_line; wq_f <= i_frame; wq_u <= wk[3:0];
                    n_wbursts <= n_wbursts + 32'd1;
                end
        //------------------------------------------------ output timing: an hsync each line
        if (o_t10 + 32'd10 >= OLINE10) begin : g_line
            reg [9:0] nl;
            hs_now = 1'b1;
            o_t10 <= o_t10 + 32'd10 - OLINE10;
            nl = (o_line == 10'd749) ? 10'd0 : o_line + 10'd1;
            o_line <= nl;
            if (o_line == 10'd729) o_buf <= i_done;
            if (nl == 10'd747) begin
                o_live <= 1'b1; o_next <= 9'd0; o_frames <= o_frames + 32'd1;
                if (o_frames == 32'd2) begin c_bytes0 = c_bytes; c_t0 = $realtime; end   // the core's throughput window
                if (checking && o_frames > 32'd3 && fr_bursts != NL * NB) n_frbad <= n_frbad + 1'b1;
                fr_bursts <= 0;
            end
            if (hsp) begin if (checking) n_hslost <= n_hslost + 1'b1; end
            else begin hsp <= 1'b1; hs_t <= cyc; hs_line <= nl; end
            if (STALL != 0 && o_frames >= 32'd3 && (o_frames % STALL_EVERY) == 0 &&
                nl == ((STALL_LINE >= 0) ? STALL_LINE : stall_line_now)) begin
                if (STALL == 1) stall_left <= STALL_LEN; else ref_burst <= 1'b1;
                n_stalls <= n_stalls + 1'b1;
                stall_line_now <= {22'd0, lfsr[9:0]} % 32'd750;
            end
        end else o_t10 <= o_t10 + 32'd10;
        //------------------------------------------------ ascal's read machine (o_clk side)
        case (fs)
            F_DISP: if (hsp && !hs_now) begin                     // sDISP takes the pending hsync; sHSYNC decides
                    hsp <= 1'b0;
                    if (checking && cyc - hs_t > hsw_max) hsw_max <= cyc - hs_t;
                    if (o_live && o_next < NL &&
                        ((o_next < 9'd2) ? (hs_line == need(o_next)) : (hs_line >= need(o_next) && hs_line < 10'd720))) begin
                        fs <= F_READ; fk <= 0; f_line <= o_next; f_t <= hs_t; f_wait <= SREAD_D;
                        o_next <= o_next + 9'd1;
                        if (o_next == NL - 1) o_live <= 1'b0;
                        if (o_next == 9'd0) pre_t <= hs_t;
                        if (o_next == 9'd100 && checking) begin
                            if (cyc - pre_t < l100_min) l100_min <= cyc - pre_t;
                            if (cyc - pre_t > l100_max) l100_max <= cyc - pre_t;
                        end
                    end
                end
            F_READ: begin                                         // sREAD: o_adrsb, then o_readlev < 2
                    if (f_wait != 0) f_wait <= f_wait - 1'b1;
                    else if (readlev < 3'd2) begin
                        lev_inc = 1'b1;
                        rq_line <= f_line; rq_u <= fk;
                        fs <= F_WAIT;
                    end
                end
            default: if (ack_sr[SYNC-1]) begin                    // sWAITREAD: the acknowledge
                    if (fk == NB - 1) begin
                        fs <= F_DISP; n_lines <= n_lines + 1'b1;
                        if (checking) begin
                            if (cyc - f_t > disp_max) disp_max <= cyc - f_t;
                            if (cyc - f_t > DISP_MAX) n_dispover <= n_dispover + 1'b1;
                            if (cyc - f_t > OLINE) n_over100 <= n_over100 + 1'b1;
                            if (f_line == 9'd0) begin
                                if (cyc - f_t > pre_max) pre_max <= cyc - f_t;
                                if (cyc - f_t > PRE_MAX) n_preover <= n_preover + 1'b1;
                                if (cyc - f_t > PRE_HARD) n_prehard <= n_prehard + 1'b1;
                            end
                        end
                    end else begin fk <= fk + 1'b1; fs <= F_READ; f_wait <= SREAD_D; end
                end
        endcase
        // the crossings (a request, an acknowledge and a data-in each pass through SYNC flops)
        req_sr <= {req_sr[SYNC-2:0], lev_inc};
        ack_sr <= {ack_sr[SYNC-2:0], 1'b0};
        dat_sr <= {dat_sr[SYNC-2:0], 1'b0};
        if (req_sr[SYNC-1]) rq_pend <= 1'b1;
        // the copies (cq_n: bursts whose data are in and whose copy has not started)
        if (copying && cyc >= copy_end) begin
            copying <= 1'b0; lev_dec = 1'b1;
            if (cp_u == NB - 1 && checking && cyc - cp_t > line_max) line_max <= cyc - cp_t;
        end else if (!copying && cq_n != 0) begin
            copying <= 1'b1; copy_end <= cyc + 1 + TCOPY[cq_u[cq_rp]];
            cp_u <= cq_u[cq_rp]; cp_t <= cq_t[cq_rp];
            cq_rp <= cq_rp + 1'b1; cq_take = 1'b1;
        end
        cq_n <= cq_n + (dat_sr[SYNC-1] ? 3'd1 : 3'd0) - (cq_take ? 3'd1 : 3'd0);
        readlev <= readlev + (lev_inc ? 3'd1 : 3'd0) - (lev_dec ? 3'd1 : 3'd0);
        //------------------------------------------------ ascal's Avalon machine (a pending write first)
        case (a_st)
            A_IDLE: if (wq_v) begin
                    a_st <= A_WR; a_beat <= 0; a_buf <= wq_buf; a_l <= wq_l; a_u <= wq_u; a_f <= wq_f; wq_v <= 1'b0;
                    avl_address <= vaddr(wq_buf, wq_l, wq_u);
                    avl_write <= 1'b1; avl_writedata <= pat(wq_f, wq_buf, wq_l, wq_u, 4'd0);
                end else if (rq_pend) begin
                    a_st <= A_RD; rq_pend <= 1'b0;
                    avl_address <= vaddr(o_buf, rq_line, rq_u);
                    avl_read <= 1'b1;
                end
            A_WR: if (avl_write && !avl_waitrequest) begin
`ifdef IDEAL
                    imem[{a_buf, a_l, a_u, a_beat[3:0]}] <= avl_writedata;
`endif
                    a_beat <= a_beat + 1'b1;
                    avl_writedata <= pat(a_f, a_buf, a_l, a_u, a_beat[3:0] + 4'd1);
                    if (a_beat == 5'd15) begin avl_write <= 1'b0; a_st <= A_IDLE; end
                end
            default: if (avl_read && !avl_waitrequest) begin
                    avl_read <= 1'b0; a_st <= A_IDLE;
                    fl_line[fl_wp] <= rq_line; fl_u[fl_wp] <= rq_u; fl_buf[fl_wp] <= o_buf;
                    fl_f[fl_wp] <= i_fof[o_buf]; fl_t[fl_wp] <= f_t; fl_wp <= fl_wp + 2'd1;
                    ack_sr[0] <= 1'b1;
                    fr_bursts <= fr_bursts + 1'b1;
`ifdef IDEAL
                    if (id_n <= 5'd1 && idq_n == 3'd0) begin id_a <= {o_buf, rq_line, rq_u, 4'd0}; id_n <= 5'd16; id_go = 1'b1; end
                    else begin idq[idq_w] <= {o_buf, rq_line, rq_u}; idq_w <= idq_w + 1'b1; id_push = 1'b1; end
`endif
                end
        endcase
`ifdef IDEAL
        id_v <= (id_n != 0);
        if (id_n != 0) begin id_d <= imem[id_a]; id_a <= id_a + 1'b1; end
        if (!id_go) begin                                  // (a read taken this clock started already)
            if (id_n <= 5'd1 && idq_n != 3'd0) begin       // the next queued read, right after this one's last beat
                id_a <= {idq[idq_r], 4'd0}; id_n <= 5'd16; idq_r <= idq_r + 1'b1; id_pop = 1'b1;
            end else if (id_n != 0) id_n <= id_n - 1'b1;
        end
        idq_n <= idq_n + (id_push ? 3'd1 : 3'd0) - (id_pop ? 3'd1 : 3'd0);
`endif
        if ((avl_read || avl_write) && avl_waitrequest) begin
            wait_c <= wait_c + 1'b1; if (wait_c + 1'b1 > wait_max && checking) wait_max <= wait_c + 1'b1;
        end else wait_c <= 0;
        //------------------------------------------------ the read beats
        if (avl_readdatavalid) begin
            n_rbeats <= n_rbeats + 1'b1;
            fl_beat <= fl_beat + 1'b1;
            if (fl_line[fl_rp] < I_H && fl_f[fl_rp] != 16'd0 && checking) begin
                if (avl_readdata !== pat(fl_f[fl_rp], fl_buf[fl_rp], fl_line[fl_rp], fl_u[fl_rp], fl_beat)) begin
                    if (n_err < 8) $display("TB cycle %0d: buffer %0d line %0d burst %0d beat %0d got %032x want %032x",
                        cyc, fl_buf[fl_rp], fl_line[fl_rp], fl_u[fl_rp], fl_beat, avl_readdata,
                        pat(fl_f[fl_rp], fl_buf[fl_rp], fl_line[fl_rp], fl_u[fl_rp], fl_beat));
                    n_err <= n_err + 1'b1;
                end
            end else n_unchecked <= n_unchecked + 1'b1;
            if (fl_beat == 4'd15) begin
                fl_rp <= fl_rp + 2'd1;
                dat_sr[0] <= 1'b1;
                cq_line[cq_wp] <= fl_line[fl_rp]; cq_u[cq_wp] <= fl_u[fl_rp]; cq_t[cq_wp] <= fl_t[fl_rp];
                cq_wp <= cq_wp + 1'b1;
            end
        end
        //------------------------------------------------ the end
        if (o_frames == OFRAMES && o_line == 10'd700 && hs_now) begin : g_end
            reg ok;
            $display("TB source %0d x %0d px (%0d lines, %0d clocks a line), %0d bursts a line; window %0d (copy %0d / last %0d clocks)%0s; output frames %0d, lines read %0d, write bursts %0d",
                     I_W, I_H, I_TOT, ILINE, NB, WIN_W, tcopy(0), tcopy(NB - 1),
`ifdef IDEAL
                     " (instant memory: the floor)",
`else
                     "",
`endif
                     o_frames, n_lines, n_wbursts);
            $display("TB stalls %0d (pattern %0d, %0d clocks at line %0d); core ADV %0d%0s",
                     n_stalls, STALL, STALL_LEN, STALL_LINE, ADV, (CORE_ON != 0) ? "" : " (no core)");
            $display("TB sHSYNC->sDISP worst %0d clocks = %0d %% of a line (margin limit %0d), over %0d; past a whole line %0d",
                     disp_max, disp_max * 100 / OLINE, DISP_MAX, n_dispover, n_over100);
            $display("TB preload line 0 worst %0d clocks = %0d %% of 25.7 us (margin limit %0d), over %0d; DDA init lost %0d",
                     pre_max, pre_max * 100 / 2570, PRE_MAX, n_preover, n_prehard);
            $display("TB lost hsyncs %0d; frames with a wrong burst count %0d (want %0d); source line 100 read %0d..%0d clocks after the preload's hsync",
                     n_hslost, n_frbad, NL * NB, l100_min, l100_max);
            $display("TB info: longest hsync wait %0d clocks; longest line to its last copy %0d (one line %0d)",
                     hsw_max, line_max, OLINE);
            $display("TB lost write bursts %0d, data errors %0d (beats %0d, %0d unchecked), longest waitrequest with a command %0d clocks; unaligned IP addresses %0d",
                     n_wbrk, n_err, n_rbeats, n_unchecked, wait_max, n_unal);
            $display("TB core: Q_CORE %0d, in the IP max %0d, fences %0d, ops %0d; read latency max %0.0f ns, mean %0.0f ns (%0d reads); V at the arbiter max %0d clocks; refreshes %0d, row misses %0d",
                     `Q_CORE, core_infl_max, fences, c_ops, c_lat_max, (c_lat_n > 0) ? c_lat_sum / c_lat_n : 0.0, c_lat_n,
                     arb_v_wait, n_refresh, n_rowmiss);
            begin : g_verdict
                reg hard, margin; real mbs;
                hard = n_wbrk == 0 && n_err == 0 && n_over100 == 0 && n_prehard == 0 && n_hslost == 0 && n_frbad == 0 &&
                       (l100_max - l100_min <= 1) && wait_max <= 32 && n_lines > 0 && n_bad_cmd == 0 && n_unal == 0 &&
                       (ADV != 2 || c_lat_max <= CORE_LAT_MAX);
                margin = n_dispover == 0 && n_preover == 0;
                mbs = ($realtime > c_t0 && c_t0 > 0.0) ? (c_bytes - c_bytes0) / ($realtime - c_t0) * 1000.0 : 0.0;
                $display("TB SUMMARY disp=%0d pre=%0d over100=%0d prehard=%0d hslost=%0d frbad=%0d l100d=%0d wbrk=%0d err=%0d wait=%0d clat=%0.0f cmbs=%0.1f inip=%0d fences=%0d hard=%0d margin=%0d",
                         disp_max, pre_max, n_over100, n_prehard, n_hslost, n_frbad, l100_max - l100_min, n_wbrk, n_err,
                         wait_max, c_lat_max, mbs, core_infl_max, fences, hard, margin);
                $display("TB_HARD %0s", hard ? "PASS" : "FAIL");
                $display("TB_MARGIN %0s", margin ? "PASS" : "FAIL");
                $display("TB_RESULT %0s", (hard && margin) ? "PASS" : "FAIL");
            end
            $finish;
        end
        end
    end
endmodule
`default_nettype wire
