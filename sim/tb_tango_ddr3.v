// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_tango_ddr3 -- tango_ddr3 (the V port, a core port, the arbiter) on the IP model (sim/tango_ddr3_model.v), the
// DDR3 path's functional bench.
//   V: an ascal-exact Avalon master (one command at a time; a write brings beat 0 with the command; beats while
//      waitrequest is low; a read is taken and its beats follow) in clk_ddr: 16-beat bursts of 128 bits, random reads
//      and writes over 512 KB of the scaler's region, and 1 write in 64 outside it (must be dropped).
//   C: a MiSTer DDRAM master in its own clock (96 MHz, asynchronous): random reads and writes, bursts of 1..MAXB
//      beats of 64 bits, random byte enables, over 512 KB of the core window, with random DDRAM_ADDR[28:25] (forced
//      to the window).  -DADVERSARIAL: bursts of MAXB = 255 beats back to back, no gaps.
//   Shadows: V's at 128-bit beats, C's at 64-bit words; a read's expected data is the shadow when the read is taken
//   (each port is in order).  The model's memory (2^16 words) holds both regions without aliasing.
// Checks: every beat read = its shadow; the dropped writes counted; core commands in the IP <= Q_CORE; no protocol
// violation at the IP.  Prints the V and C read latencies (command taken to last beat) and V's longest waitrequest
// with a command present (V saturates here, far above ascal's rate: the <= 32 clock bound is
// checked with ascal's real pacing in the stream bench).  TB_RESULT PASS / FAIL.
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_tango_ddr3;
`include "tango_ddr3_silicon.vh"                       // the model calibrated on silicon (the DDR3 bring-up test)
`ifndef NV
`define NV 4000
`endif
`ifdef ADVERSARIAL
    localparam integer MAXB = 255, CGAP = 0;
`else
    localparam integer MAXB = 64, CGAP = 1;
`endif
`ifndef Q_CORE
`define Q_CORE 8
`endif
    reg clk = 1'b0, cclk = 1'b0;
    always #5      clk  = ~clk;
    always #5.2083 cclk = ~cclk;
    reg  rst = 1'b1, crst = 1'b1;
    // ---- the model
    wire         init_calib_complete, cmd_ready, cmd_en, wr_data_rdy, wr_data_en, rd_data_valid, rd_data_end;
    wire [2:0]   cmd;
    wire [28:0]  addr;
    wire [255:0] wr_data, rd_data;
    wire [31:0]  wr_data_mask, n_refresh, n_rowmiss, n_bad_cmd;
    reg          m_rst_n = 1'b0;
    tango_ddr3_model #(.MEM_LOG2(16), .INIT_CLKS(100), .CMD_DEPTH(SIL_CMD_DEPTH), .WDATA_DEPTH(SIL_WDATA_DEPTH),
                       .RD_LAT(SIL_RD_LAT), .RD_JITTER(SIL_RD_JITTER), .T_BL(1), .T_ROWMISS(SIL_T_ROWMISS),
                       .T_TURN(SIL_T_TURN), .REF_PERIOD(SIL_REF_PERIOD), .REF_LEN(SIL_REF_LEN)) u_m (
        .clk(clk), .rst_n(m_rst_n), .init_calib_complete(init_calib_complete),
        .cmd(cmd), .cmd_en(cmd_en), .addr(addr), .cmd_ready(cmd_ready),
        .wr_data(wr_data), .wr_data_en(wr_data_en), .wr_data_end(wr_data_en), .wr_data_mask(wr_data_mask),
        .wr_data_rdy(wr_data_rdy), .rd_data(rd_data), .rd_data_valid(rd_data_valid), .rd_data_end(rd_data_end),
        .force_stall(1'b0), .ref_burst(1'b0), .fault_en(1'b0), .fault_word(25'd0), .fault_xor(256'd0),
        .n_refresh(n_refresh), .n_rowmiss(n_rowmiss), .n_bad_cmd(n_bad_cmd));
    // ---- the DUT
    reg  [27:0]  avl_address = 0;
    reg  [7:0]   avl_burstcount = 8'd16;
    reg          avl_write = 1'b0, avl_read = 1'b0;
    reg  [127:0] avl_writedata = 0;
    reg  [15:0]  avl_byteenable = 16'hFFFF;
    wire         avl_waitrequest, avl_readdatavalid;
    wire [127:0] avl_readdata;
    wire         DDRAM_BUSY, DDRAM_DOUT_READY;
    reg  [7:0]   DDRAM_BURSTCNT = 1;
    reg  [28:0]  DDRAM_ADDR = 0;
    wire [63:0]  DDRAM_DOUT;
    reg          DDRAM_RD = 1'b0, DDRAM_WE = 1'b0;
    reg  [63:0]  DDRAM_DIN = 0;
    reg  [7:0]   DDRAM_BE = 8'hFF;
    wire [15:0]  v_wait_max, core_infl_max;
    wire [31:0]  v_win_drops, fences, arb_v_wait;
    tango_ddr3 #(.G(8), .Q_CORE(`Q_CORE)) dut (
        .clk(clk), .rst(rst),
        .avl_address(avl_address), .avl_burstcount(avl_burstcount), .avl_write(avl_write),
        .avl_writedata(avl_writedata), .avl_byteenable(avl_byteenable), .avl_read(avl_read),
        .avl_waitrequest(avl_waitrequest), .avl_readdata(avl_readdata), .avl_readdatavalid(avl_readdatavalid),
        .DDRAM_CLK(cclk), .ddram_rst(crst), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT),
        .DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
        .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE),
        .cmd_ready(cmd_ready), .cmd_en(cmd_en), .cmd(cmd), .addr(addr), .wr_data_rdy(wr_data_rdy),
        .wr_data_en(wr_data_en), .wr_data(wr_data), .wr_data_mask(wr_data_mask),
        .rd_data_valid(rd_data_valid), .rd_data(rd_data),
        .obs_v_wait_max(v_wait_max), .obs_v_win_drops(v_win_drops), .obs_core_inflight_max(core_infl_max),
        .obs_fences(fences), .obs_arb_v_wait_max(arb_v_wait));
`ifdef PRECAL
    // -DPRECAL: the core leaves its own reset long before the IP has calibrated and starts at once: DDRAM_BUSY must
    // hold it until tango_ddr3 leaves reset; a beat or command taken before is an error
    initial begin
        #100 m_rst_n = 1'b1;
        #200 @(posedge cclk); crst = 1'b0;
        wait (init_calib_complete);
        @(posedge clk); rst = 1'b0;
    end
`else
    initial begin
        #100 m_rst_n = 1'b1;
        wait (init_calib_complete);
        @(posedge clk); rst = 1'b0;
        @(posedge cclk); crst = 1'b0;
    end
`endif
    integer n_precal = 0, n_precal_try = 0;
    always @(posedge cclk) if (rst && !crst && (DDRAM_RD || DDRAM_WE)) begin
        n_precal_try = n_precal_try + 1;
        if (!DDRAM_BUSY) n_precal = n_precal + 1;
    end

    // ---- shadows (associative: word address -> data)
    reg [127:0] vsh [int unsigned];
    reg [63:0]  csh [int unsigned];
    integer errs = 0, vdone = 0, cdone = 0, vdrop_words = 0, n_unal = 0;
    // every IP command's column 8-aligned (a 256-bit word; DDR3's sequential BL8 wraps the lanes otherwise, as the DDR3 bring-up test showed)
    always @(posedge clk) if (cmd_en && addr[2:0] != 3'd0) begin
        n_unal = n_unal + 1; errs = errs + 1;
        if (n_unal < 5) $display("TB FAIL unaligned IP address %07x at %0t", addr, $time);
    end
    longint vlat_max = 0, clat_max = 0, vlat_sum = 0, nvr = 0;
    function automatic [127:0] vexp(input int unsigned a);
        vexp = vsh.exists(a) ? vsh[a] : 128'd0;
    endfunction
    function automatic [63:0] cexp(input int unsigned a);
        cexp = csh.exists(a) ? csh[a] : 64'd0;
    endfunction

    // ---- V: ascal's Avalon master (one command at a time).  Every falling edge: (A) retire what the last rising edge
    // took, (B) start the next command if idle, (C) decide whether the bus as it is now is taken at the next rising
    // edge (waitrequest is stable between rising edges), (D) check returned beats.
    localparam [27:0] VBASE = 28'h0200_0000;                 // byte 0x2000_0000 / 16
    reg [127:0] vq [0:15];
    integer vst = 0, vb = 0, vkind = 0;
    reg     vacc_w = 1'b0, vacc_r = 1'b0;
    int unsigned va = 0;
    longint vt0 = 0;
    function automatic [127:0] vdat(input integer n, input int unsigned a, input integer b);
        vdat = {n[31:0], a[27:0], b[3:0], $urandom, 28'h5A5A5A5, 4'd0};
    endfunction
    always @(negedge clk) begin : g_v
        if (!rst && vdone < `NV) begin
            if (vacc_w) begin
                vacc_w = 1'b0; vb = vb + 1;
                if (vb == 16) begin avl_write = 1'b0; vst = 0; vdone = vdone + 1;
                                    if (vkind == 0) vdrop_words = vdrop_words + 8; end
                else avl_writedata = vdat(vdone, va, vb);
            end
            if (vacc_r) begin vacc_r = 1'b0; avl_read = 1'b0; vst = 3; end
            if (vst == 0 && vdone < `NV) begin                  // (none after the last: it would dangle on the bus)
                vkind = $urandom % 64;
                va = VBASE + (($urandom % 2048) << 4);
                if (vkind == 0) va = 28'h0100_0000 + (($urandom % 2048) << 4);   // outside the window: dropped
                vb = 0; avl_address = va; avl_burstcount = 8'd16; avl_byteenable = 16'hFFFF;
                if (vkind < 40) begin vst = 1; avl_write = 1'b1; avl_writedata = vdat(vdone, va, 0); end
                else begin vst = 2; avl_read = 1'b1; end
            end
            if (vst == 1 && !avl_waitrequest) begin
                if (vkind != 0) vsh[va + vb] = avl_writedata;
                vacc_w = 1'b1;
            end
            if (vst == 2 && !avl_waitrequest) begin
                for (vb = 0; vb < 16; vb = vb + 1) vq[vb] = vexp(va + vb);
                vb = 0; vacc_r = 1'b1; vt0 = $time;
            end
            if (vst == 3 && avl_readdatavalid) begin
                if (avl_readdata !== vq[vb]) begin
                    errs = errs + 1;
                    if (errs < 10) $display("TB FAIL V read 0x%07x beat %0d: got %032x want %032x", va, vb, avl_readdata, vq[vb]);
                end
                vb = vb + 1;
                if (vb == 16) begin
                    if ($time - vt0 > vlat_max) vlat_max = $time - vt0;
                    vlat_sum = vlat_sum + ($time - vt0); nvr = nvr + 1;
                    vst = 0; vdone = vdone + 1;
                end
            end
        end
    end

    // ---- C: a MiSTer DDRAM master, the same four steps on the falling edge of its clock
    reg [63:0] cq [0:255];
    integer cst = 0, cb = 0, cnb = 0, cgap = 0;
    reg     cacc_w = 1'b0, cacc_r = 1'b0;
    int unsigned ca = 0;
    longint ct0 = 0;
    always @(negedge cclk) begin : g_c
        reg [3:0] top; reg [63:0] o; integer y;
        if (!crst && vdone < `NV) begin
            if (cacc_w) begin
                cacc_w = 1'b0; cb = cb + 1;
                if (cb == cnb) begin DDRAM_WE = 1'b0; cst = 0; cdone = cdone + 1; cgap = CGAP ? ($urandom % 8) : 0; end
                else begin
                    DDRAM_DIN = {cdone[15:0], ca[23:0], cb[7:0], 16'hC0DE} ^ {$urandom, $urandom};
                    DDRAM_BE = ($urandom % 4 == 0) ? $urandom : 8'hFF;
                end
            end
            if (cacc_r) begin cacc_r = 1'b0; DDRAM_RD = 1'b0; cst = 3; end
            if (cst == 0) begin
                if (cgap > 0) cgap = cgap - 1;
                else begin
                    cnb = 1 + ($urandom % MAXB);
                    ca = 32'h2_0000 + ($urandom % 32'h1_0000);
                    if (ca + cnb > 32'h3_0000) ca = 32'h3_0000 - cnb;
                    top = $urandom;
                    DDRAM_ADDR = {top, ca[24:0]}; DDRAM_BURSTCNT = cnb; cb = 0;
                    if ($urandom % 2) begin
                        cst = 1; DDRAM_WE = 1'b1;
                        DDRAM_DIN = {cdone[15:0], ca[23:0], 8'd0, 16'hC0DE} ^ {$urandom, $urandom};
                        DDRAM_BE = ($urandom % 4 == 0) ? $urandom : 8'hFF;
                    end else begin cst = 2; DDRAM_RD = 1'b1; end
                end
            end
            if (cst == 1 && !DDRAM_BUSY) begin
                o = cexp(ca + cb);
                for (y = 0; y < 8; y = y + 1) if (DDRAM_BE[y]) o[8 * y +: 8] = DDRAM_DIN[8 * y +: 8];
                csh[ca + cb] = o;
                cacc_w = 1'b1;
            end
            if (cst == 2 && !DDRAM_BUSY) begin
                for (cb = 0; cb < cnb; cb = cb + 1) cq[cb] = cexp(ca + cb);
                cb = 0; cacc_r = 1'b1; ct0 = $time;
            end
            if (cst == 3 && DDRAM_DOUT_READY) begin
                if (DDRAM_DOUT !== cq[cb]) begin
                    errs = errs + 1;
                    if (errs < 10) $display("TB FAIL C read 0x%07x beat %0d/%0d: got %016x want %016x", ca, cb, cnb, DDRAM_DOUT, cq[cb]);
                end
                cb = cb + 1;
                if (cb == cnb) begin
                    if ($time - ct0 > clat_max) clat_max = $time - ct0;
                    cst = 0; cdone = cdone + 1; cgap = CGAP ? ($urandom % 8) : 0;
                end
            end
        end
    end

    initial begin
        wait (vdone == `NV);
        repeat (2000) @(posedge clk);
        $display("TB V commands %0d, C commands %0d; model refreshes %0d, row misses %0d", vdone, cdone, n_refresh, n_rowmiss);
        $display("TB V read burst latency: max %0d ns, mean %0d ns", vlat_max, (nvr > 0) ? vlat_sum / nvr : 0);
        $display("TB C read burst latency (to the last beat): max %0d ns", clat_max);
        $display("TB V waitrequest with a command present: max %0d clocks (<= 32)", v_wait_max);
        $display("TB V requests waiting at the arbiter: max %0d clocks", arb_v_wait);
        $display("TB core commands in the IP: max %0d (Q_CORE %0d); fences %0d", core_infl_max, `Q_CORE, fences);
        $display("TB V writes dropped outside the window: %0d words (want %0d)", v_win_drops, vdrop_words);
        // the V master here saturates (back-to-back 256-B bursts, far above ascal's rate), so a full posting FIFO is
        // expected: the <= 32 clock bound is checked against ascal's real pacing in tb_tango_ascal_stream_ddr3
        $display("TB (V saturates: waitrequest is information here)");
        if (core_infl_max > `Q_CORE) begin errs = errs + 1; $display("TB FAIL core inflight > Q_CORE"); end
        if (v_win_drops != vdrop_words) begin errs = errs + 1; $display("TB FAIL window drops"); end
        if (n_bad_cmd != 0) begin errs = errs + 1; $display("TB FAIL %0d protocol violations at the IP", n_bad_cmd); end
        $display("TB C before calibration: %0d clocks with a command, %0d taken (must be 0)", n_precal_try, n_precal);
        if (n_precal != 0) errs = errs + 1;
`ifdef PRECAL
        if (n_precal_try == 0) begin errs = errs + 1; $display("TB FAIL PRECAL: the core never tried before calibration"); end
`endif
        $display("TB_RESULT %0s (%0d errors)", (errs == 0) ? "PASS" : "FAIL", errs);
        $finish;
    end
    initial begin #400000000; $display("TB timeout (V %0d, C %0d)", vdone, cdone); $display("TB_RESULT FAIL (timeout)"); $finish; end
endmodule
`default_nettype wire
