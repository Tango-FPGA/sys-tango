// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_ddr3_arb_equiv -- lockstep equivalence of tango_ddr3_arb (the DUT, the tree's file) against tango_ddr3_arb_ref
// (sim/ddr3_arb_ref/: the arbiter and queue before their timing fix, module names suffixed _ref).
// Both get the SAME inputs every clock: the same V / C requests (random, held until taken, as the real ports hold
// them), the same IP-side inputs (cmd_ready, wr_data_rdy, rd_data_valid, rd_data) -- those come from one IP model
// driven by the REFERENCE's command stream; since the DUT's outputs are compared every clock, the DUT sees what its
// own commands would have got.  At every falling clock edge EVERY output of the two is compared (v_ready, c_ready,
// r_valid, r_data, cmd_en, cmd, addr, wr_data_en, wr_data, wr_data_mask, the obs_ counters) and the internal state the
// change touches (inflight, core_room, vlim, tags_ok, cr, pend, vstreak, tw, tr, rr, fence_t, hd_v); the first
// difference is printed and stops the run.  The fix claims no latency change, so the compare is exact, clock by clock.
// Traffic phases change every PH clocks (idle, V only, C only, both flat out, read heavy, write heavy, sparse ...),
// the IP model has random force_stall and refresh bursts, the model calibrated on silicon (tango_ddr3_silicon.vh).
// Coverage counters must be non-zero at the end (otherwise the pass proves nothing): TB_RESULT PASS / FAIL.
//   -DNC=n (1 or 2)  -DQ_CORE=n  -DGV=n (the V streak limit G)  -DTAGL=n (TAG_LOG2)  -DFENCEW=n  -DNCLK=n  -DSEED=n
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_ddr3_arb_equiv;
`include "tango_ddr3_silicon.vh"
`ifndef NC
`define NC 1
`endif
`ifndef Q_CORE
`define Q_CORE 16
`endif
`ifndef GV
`define GV 8
`endif
`ifndef TAGL
`define TAGL 6
`endif
`ifndef FENCEW
`define FENCEW 64
`endif
`ifndef NCLK
`define NCLK 600000
`endif
`ifndef SEED
`define SEED 1
`endif
    localparam integer NC = `NC;
    localparam integer PH = 1500;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst = 1'b1;
    reg m_rst_n = 1'b0;

    // ---- the IP model, fed by the reference's commands
    wire         init_calib_complete, cmd_ready, wr_data_rdy, rd_data_valid, rd_data_end;
    wire [255:0] rd_data;
    wire [31:0]  n_refresh, n_rowmiss, n_bad_cmd;
    reg          force_stall = 1'b0, ref_burst = 1'b0;
    wire         r_cmd_en, r_wr_data_en;
    wire [2:0]   r_cmd;
    wire [28:0]  r_addr;
    wire [255:0] r_wr_data;
    wire [31:0]  r_wr_data_mask;
    tango_ddr3_model #(.MEM_LOG2(16), .INIT_CLKS(100), .CMD_DEPTH(SIL_CMD_DEPTH), .WDATA_DEPTH(SIL_WDATA_DEPTH),
                       .RD_LAT(SIL_RD_LAT), .RD_JITTER(SIL_RD_JITTER), .T_BL(1), .T_ROWMISS(SIL_T_ROWMISS),
                       .T_TURN(SIL_T_TURN), .REF_PERIOD(SIL_REF_PERIOD), .REF_LEN(SIL_REF_LEN)) u_m (
        .clk(clk), .rst_n(m_rst_n), .init_calib_complete(init_calib_complete),
        .cmd(r_cmd), .cmd_en(r_cmd_en), .addr(r_addr), .cmd_ready(cmd_ready),
        .wr_data(r_wr_data), .wr_data_en(r_wr_data_en), .wr_data_end(r_wr_data_en), .wr_data_mask(r_wr_data_mask),
        .wr_data_rdy(wr_data_rdy), .rd_data(rd_data), .rd_data_valid(rd_data_valid), .rd_data_end(rd_data_end),
        .force_stall(force_stall), .ref_burst(ref_burst), .fault_en(1'b0), .fault_word(25'd0), .fault_xor(256'd0),
        .n_refresh(n_refresh), .n_rowmiss(n_rowmiss), .n_bad_cmd(n_bad_cmd));

    // ---- the requests (shared by both)
    reg              v_valid = 1'b0, v_write = 1'b0;
    reg  [28:0]      v_addr = 0;
    reg  [255:0]     v_wdata = 0;
    reg  [31:0]      v_wmask = 0;
    reg  [NC-1:0]    c_valid = 0, c_write = 0;
    reg  [NC*29-1:0] c_addr = 0;
    reg  [NC*256-1:0] c_wdata = 0;
    reg  [NC*32-1:0] c_wmask = 0;

    // ---- the reference and the DUT
    wire              r_v_ready, d_v_ready;
    wire [NC-1:0]     r_c_ready, d_c_ready;
    wire [NC:0]       r_r_valid, d_r_valid;
    wire [255:0]      r_r_data, d_r_data;
    wire              d_cmd_en, d_wr_data_en;
    wire [2:0]        d_cmd;
    wire [28:0]       d_addr;
    wire [255:0]      d_wr_data;
    wire [31:0]       d_wr_data_mask;
    wire [15:0]       r_o_max, d_o_max, r_o_infl, d_o_infl;
    wire [31:0]       r_o_wait, d_o_wait, r_o_fen, d_o_fen;
    tango_ddr3_arb_ref #(.NC(NC), .G(`GV), .Q_CORE(`Q_CORE), .TAG_LOG2(`TAGL), .FENCE_WAIT(`FENCEW), .FENCE_ADDR(29'h1234)) u_ref (
        .clk(clk), .rst(rst),
        .v_valid(v_valid), .v_write(v_write), .v_addr(v_addr), .v_wdata(v_wdata), .v_wmask(v_wmask), .v_ready(r_v_ready),
        .c_valid(c_valid), .c_write(c_write), .c_addr(c_addr), .c_wdata(c_wdata), .c_wmask(c_wmask), .c_ready(r_c_ready),
        .r_valid(r_r_valid), .r_data(r_r_data),
        .cmd_ready(cmd_ready), .wr_data_rdy(wr_data_rdy), .cmd_en(r_cmd_en), .cmd(r_cmd), .addr(r_addr),
        .wr_data_en(r_wr_data_en), .wr_data(r_wr_data), .wr_data_mask(r_wr_data_mask),
        .rd_data_valid(rd_data_valid), .rd_data(rd_data),
        .obs_core_inflight_max(r_o_max), .obs_v_wait_max(r_o_wait), .obs_fences(r_o_fen), .obs_core_inflight(r_o_infl));
    tango_ddr3_arb #(.NC(NC), .G(`GV), .Q_CORE(`Q_CORE), .TAG_LOG2(`TAGL), .FENCE_WAIT(`FENCEW), .FENCE_ADDR(29'h1234)) u_dut (
        .clk(clk), .rst(rst),
        .v_valid(v_valid), .v_write(v_write), .v_addr(v_addr), .v_wdata(v_wdata), .v_wmask(v_wmask), .v_ready(d_v_ready),
        .c_valid(c_valid), .c_write(c_write), .c_addr(c_addr), .c_wdata(c_wdata), .c_wmask(c_wmask), .c_ready(d_c_ready),
        .r_valid(d_r_valid), .r_data(d_r_data),
        .cmd_ready(cmd_ready), .wr_data_rdy(wr_data_rdy), .cmd_en(d_cmd_en), .cmd(d_cmd), .addr(d_addr),
        .wr_data_en(d_wr_data_en), .wr_data(d_wr_data), .wr_data_mask(d_wr_data_mask),
        .rd_data_valid(rd_data_valid), .rd_data(rd_data),
        .obs_core_inflight_max(d_o_max), .obs_v_wait_max(d_o_wait), .obs_fences(d_o_fen), .obs_core_inflight(d_o_infl));

    // ---- the traffic: a phase table, a new request chosen when the last was taken (or none was pending)
    // (Verilator's $random(seed) collapses to shifted constants after a few thousand calls: the bench has its own xorshift32)
    reg [31:0] rs = 32'h9E3779B9 ^ (`SEED * 32'h01000193);
    integer clkn = 0;
    integer pv = 100, pc = 100, pwv = 30, pwc = 30, pstall = 0, pburst = 0;     // percent
    integer phase = 0;
    function [31:0] rnd32(input integer dummy);
        begin rs = rs ^ (rs << 13); rs = rs ^ (rs >> 17); rs = rs ^ (rs << 5); rnd32 = rs; end
    endfunction
    function integer rnd(input integer n);        // 0..n-1
        reg [31:0] r;
        begin r = rnd32(0); rnd = (r >> 3) % n; end
    endfunction
    always @(posedge clk) if (clkn % PH == 0) begin
        case (rnd(12))
            0:  begin pv = 100; pc = 100; pwv = 30;  pwc = 30;  end    // both flat out
            1:  begin pv = 100; pc = 0;   pwv = 20;  pwc = 0;   end    // V only
            2:  begin pv = 0;   pc = 100; pwv = 0;   pwc = 50;  end    // C only
            3:  begin pv = 100; pc = 100; pwv = 0;   pwc = 0;   end    // all reads
            4:  begin pv = 100; pc = 100; pwv = 100; pwc = 100; end    // all writes
            5:  begin pv = 20;  pc = 20;  pwv = 50;  pwc = 50;  end    // sparse
            6:  begin pv = 60;  pc = 90;  pwv = 10;  pwc = 70;  end
            7:  begin pv = 100; pc = 100; pwv = 5;   pwc = 5;   end    // read heavy: fences, tags
            8:  begin pv = 3;   pc = 100; pwv = 50;  pwc = 95;  end    // C writes with a trickle of V: fences by wait
            9:  begin pv = 100; pc = 30;  pwv = 50;  pwc = 0;   end
            10: begin pv = 0;   pc = 0;   pwv = 0;   pwc = 0;   end    // idle (drains)
            11: begin pv = 50;  pc = 50;  pwv = 50;  pwc = 50;  end
        endcase
        case (rnd(4))
            0: begin pstall = 0;  pburst = 0; end
            1: begin pstall = 1;  pburst = 0; end
            2: begin pstall = 5;  pburst = 1; end
            3: begin pstall = 30; pburst = 0; end
        endcase
    end
    function [28:0] raddr(input integer dummy);
        reg [31:0] r;
        begin r = rnd32(0); raddr = (r[0]) ? {r[28:6], 6'd0} | r[5:0] : {16'd0, r[15:3]}; end   // far or close (row hits)
    endfunction
    integer q;
    reg [255:0] w256;
    always @(posedge clk) begin
        clkn <= clkn + 1;
        force_stall <= (rnd(100) < pstall) ? ((rnd(8) == 0) ? 1'b1 : force_stall ^ (rnd(2) == 0)) : 1'b0;
        ref_burst <= (pburst != 0) && (rnd(3000) == 0);
        if (!rst && init_calib_complete) begin
            if (!v_valid || d_v_ready) begin       // (taken or none pending; both ready flags are equal when the bench passes)
                v_valid <= (rnd(100) < pv) && !quiet;
                v_write <= (rnd(100) < pwv);
                v_addr  <= raddr(0);
                for (q = 0; q < 8; q = q + 1) w256[32*q +: 32] = rnd32(0);
                v_wdata <= w256;
                v_wmask <= rnd32(0);
            end
            for (q = 0; q < NC; q = q + 1) if (!c_valid[q] || d_c_ready[q]) begin
                c_valid[q] <= (rnd(100) < pc) && !quiet;
                c_write[q] <= (rnd(100) < pwc);
                c_addr[q*29 +: 29] <= raddr(0);
                begin : cw
                    integer z;
                    for (z = 0; z < 8; z = z + 1) w256[32*z +: 32] = rnd32(0);
                end
                c_wdata[q*256 +: 256] <= w256;
                c_wmask[q*32 +: 32] <= rnd32(0);
            end
        end
    end
    initial begin
        repeat (10) @(posedge clk); m_rst_n = 1'b1; rst <= 1'b1;
        repeat (5) @(posedge clk); rst <= 1'b0;
    end
    // resets of both arbiters in the middle of the run.  The first follows 3,000 clocks without new requests (the IP drains:
    // a reset system as the real one is, whose IP does not keep reads for an arbiter that forgot them).  The second, near the
    // end, has traffic in the queues and reads in the IP (their returns reach a reset arbiter): not a sane system state, but
    // equality of the two arbiters on every input is the claim, and the compare keeps running.
    localparam integer R1 = `NCLK / 4, R2 = `NCLK - 4000;
    wire quiet = (clkn >= R1 - 3000) && (clkn < R1 + 10);
    integer rst_pulses = 0;
    always @(posedge clk) if (clkn == R1 || clkn == R2) begin
        rst <= 1'b1; rst_pulses = rst_pulses + 1;
        repeat (3) @(posedge clk);
        rst <= 1'b0;
    end

    // ---- the compare, every falling edge
    integer errors = 0;
    task fail(input [255:0] what);
        begin
            $display("EQUIV MISMATCH at clock %0d (%0t): %0s", clkn, $time, what);
            errors = errors + 1;
            $display("TB_RESULT FAIL");
            $finish;
        end
    endtask
    always @(negedge clk) if (clkn > 0) begin
        if (r_v_ready !== d_v_ready) fail("v_ready");
        if (r_c_ready !== d_c_ready) fail("c_ready");
        if (r_r_valid !== d_r_valid) fail("r_valid");
        if (r_r_data  !== d_r_data)  fail("r_data");
        if (r_cmd_en !== d_cmd_en)   fail("cmd_en");
        if (r_wr_data_en !== d_wr_data_en) fail("wr_data_en");
        if (r_cmd !== d_cmd)         fail("cmd");
        if (r_addr !== d_addr)       fail("addr");
        if (r_wr_data_mask !== d_wr_data_mask) fail("wr_data_mask");
        if (r_wr_data !== d_wr_data) fail("wr_data");
        if (r_o_max !== d_o_max || r_o_wait !== d_o_wait || r_o_fen !== d_o_fen || r_o_infl !== d_o_infl) fail("obs");
        if (u_ref.inflight !== u_dut.inflight) fail("inflight");
        if (u_ref.core_room !== u_dut.core_room) fail("core_room");
        if (u_ref.vlim !== u_dut.vlim) fail("vlim");
        if (u_ref.tags_ok !== u_dut.tags_ok) fail("tags_ok");
        if (u_ref.cr !== u_dut.cr) fail("cr");
        if (u_ref.pend !== u_dut.pend) fail("pend");
        if (u_ref.vstreak !== u_dut.vstreak) fail("vstreak");
        if (u_ref.tw !== u_dut.tw || u_ref.tr !== u_dut.tr) fail("tw/tr");
        if (u_ref.rr !== u_dut.rr) fail("rr");
        if (u_ref.fence_t !== u_dut.fence_t) fail("fence_t");
        if (u_ref.hd_v !== u_dut.hd_v) fail("hd_v");
        if (u_ref.m_n !== u_dut.m_n) fail("m_n");
        if (u_ref.u_qv.n !== u_dut.u_qv.n) fail("u_qv.n");
    end

    // ---- coverage (DUT side; equal to the reference's while the compare passes)
    integer n_gv = 0, n_gc = 0, n_gf = 0, n_gc_rdv = 0, n_infl_max = 0, n_vlim_hold = 0, n_tags_stall = 0, n_room_stall = 0,
            n_cr_stall = 0, n_cmd = 0, n_cmd_rd = 0, n_cmd_wr = 0, n_ready_low = 0, n_gv_wr = 0, n_gv_rd = 0, n_vpush = 0,
            n_cpush = 0, n_wb_pop_push = 0, n_ret = 0, n_stall = 0, n_gc_vcan = 0;
    always @(posedge clk) if (!rst && clkn > 20) begin
        if (u_dut.gv) begin n_gv = n_gv + 1; if (u_dut.hv_write) n_gv_wr = n_gv_wr + 1; else n_gv_rd = n_gv_rd + 1; end
        if (u_dut.gc) begin n_gc = n_gc + 1; if (u_dut.rdv_q) n_gc_rdv = n_gc_rdv + 1; if (u_dut.v_can) n_gc_vcan = n_gc_vcan + 1; end
        if (u_dut.gf) n_gf = n_gf + 1;
        if (u_dut.inflight == `Q_CORE) n_infl_max = n_infl_max + 1;
        if (u_dut.v_can && u_dut.cany && u_dut.vlim) n_vlim_hold = n_vlim_hold + 1;
        if (u_dut.hv_valid && !u_dut.hv_write && !u_dut.tags_ok && u_dut.cr_ok) n_tags_stall = n_tags_stall + 1;
        if (u_dut.hc_valid[0] && !u_dut.core_room) n_room_stall = n_room_stall + 1;
        if ((u_dut.hv_valid || u_dut.hc_valid[0]) && !u_dut.cr_ok) n_cr_stall = n_cr_stall + 1;
        if (d_cmd_en) begin n_cmd = n_cmd + 1; if (d_cmd == 3'b001) n_cmd_rd = n_cmd_rd + 1; else n_cmd_wr = n_cmd_wr + 1; end
        if (u_dut.hd_v && !cmd_ready) n_ready_low = n_ready_low + 1;
        if (v_valid && d_v_ready) n_vpush = n_vpush + 1;
        if (c_valid[0] && d_c_ready[0]) n_cpush = n_cpush + 1;
        if (u_dut.hv_pop && v_valid && d_v_ready) n_wb_pop_push = n_wb_pop_push + 1;   // out_wb: a pop and a push together
        if (rd_data_valid) n_ret = n_ret + 1;
        if (force_stall) n_stall = n_stall + 1;
    end
    always @(posedge clk) if (clkn == `NCLK) begin
        $display("TB coverage (NC %0d, Q_CORE %0d, G %0d, TAG_LOG2 %0d, FENCE_WAIT %0d, seed %0d, %0d clocks):", NC, `Q_CORE, `GV, `TAGL, `FENCEW, `SEED, clkn);
        $display("TB   grants: V %0d (writes %0d, reads %0d), C %0d (with a return the same clock %0d; V also able %0d), fence %0d", n_gv, n_gv_wr, n_gv_rd, n_gc, n_gc_rdv, n_gc_vcan, n_gf);
        $display("TB   commands at the IP: %0d (reads %0d, writes %0d); returns %0d; head waiting on cmd_ready %0d; force_stall clocks %0d", n_cmd, n_cmd_rd, n_cmd_wr, n_ret, n_ready_low, n_stall);
        $display("TB   inflight at Q_CORE %0d clocks; C held by the V streak limit %0d; V read blocked by tags %0d; C blocked by core_room %0d; blocked by credits %0d", n_infl_max, n_vlim_hold, n_tags_stall, n_room_stall, n_cr_stall);
        $display("TB   V pushes %0d, C pushes %0d, V pop and push in one clock %0d; refreshes %0d; bad commands at the IP %0d; mid-run resets %0d", n_vpush, n_cpush, n_wb_pop_push, n_refresh, n_bad_cmd, rst_pulses);
        if (n_gv_wr == 0 || n_gv_rd == 0 || n_gc_rdv == 0 || n_gf == 0 || n_infl_max == 0 || n_vlim_hold == 0 || n_room_stall == 0 ||
            n_cr_stall == 0 || n_wb_pop_push == 0 || n_ready_low == 0 || n_cmd_wr == 0 || n_cmd_rd == 0 || n_bad_cmd != 0 ||
            (`TAGL <= 4 && n_tags_stall == 0))
            begin $display("TB coverage: a counter is zero (or the IP model saw a bad command): the run proves nothing"); $display("TB_RESULT FAIL"); end
        else $display("TB_RESULT PASS (%0d clocks, every output and the internal state equal in each)", clkn);
        $finish;
    end
endmodule
`default_nettype wire
