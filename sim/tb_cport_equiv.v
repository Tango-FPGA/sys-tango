// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_cport_equiv -- lockstep equivalence of tango_ddr3_cport (the DUT, the tree's file) against tango_ddr3_cport_ref
// (sim/cport_ref/: the core port before its timing fix, module name suffixed _ref).
//
// Both get the SAME inputs on every clock of both domains: the core's MiSTer DDRAM_* requests (held while DDRAM_BUSY, bursts
// 0..255, all four start lanes, byte enables, gaps inside a write burst, back-to-back commands, random addresses incl. the
// 2^25 wrap), the two resets (rst: clk_ddr side until the IP is calibrated, crst: the core's), and on the arbiter side
// q_ready (random stalls) and the in-order returns r_valid / r_data (one IP read answered per clock at most, with a latency
// the phase table sets from 1 clock to 150, so that the return-info ring (31 reads) and the return FIFO fill and bind).
// The returns follow the REFERENCE's reads; the DUT's outputs are compared on every clock, so the DUT would have drawn the
// same.  The two clocks are free-running and unrelated (-DCORE_HP: the core clock's half period in ns; clk_ddr is 100 MHz).
//
// At every FALLING edge of each clock EVERY output of both is compared, exactly (no latency tolerance, the change claims
// none): core side DDRAM_BUSY, DDRAM_DOUT, DDRAM_DOUT_READY; clk_ddr side q_valid, q_write, q_addr, q_wdata, q_wmask; and
// the splitter's state that the change touches (busy, sa, sn, nb, skn, acc_n, fl, ri_wp, ri_rp, rf_res, rf_ok, ri_ok ...).
// The first difference stops the run (TB_RESULT FAIL).
//
// A second, independent check keeps the bench honest: a functional scoreboard (an ideal memory behind q_*, a shadow of what
// the core wrote) checks the DUT's DOUT beats against what the core should read -- so the stimulus is known to run the
// module properly, not merely equal-to-the-reference-wherever-both-are-wrong.
//
// Coverage counters must be non-zero at the end (otherwise the pass proves nothing).
//   -DSEED=n  -DNCLK=n (clk_ddr clocks)  -DCORE_HP=ns  -DRF_LOG2=n  -DRESET_EVERY=n  -DNO_BC0 (no zero-length bursts)
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_cport_equiv;
`ifndef SEED
`define SEED 1
`endif
`ifndef NCLK
`define NCLK 600000
`endif
`ifndef CORE_HP
`define CORE_HP 6.734
`endif
`ifndef RF_LOG2
`define RF_LOG2 7
`endif
`ifndef RESET_EVERY
`define RESET_EVERY 70000
`endif
    localparam integer PH = 1500;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg cclk = 1'b0;
    initial begin #3.1; forever #(`CORE_HP) cclk = ~cclk; end

    reg rst = 1'b1, crst = 1'b1;

    // ---- the shared inputs
    reg          g_rd = 1'b0, g_we = 1'b0;
    reg  [28:0]  g_addr = 29'd0;
    reg  [7:0]   g_bc = 8'd0;
    reg  [63:0]  g_din = 64'd0;
    reg  [7:0]   g_be = 8'd0;
    reg          q_ready = 1'b1;
    reg          r_valid = 1'b0;
    reg  [255:0] r_data = 256'd0;

    // ---- the reference and the DUT
    wire         r_busy, d_busy, r_dready, d_dready;
    wire [63:0]  r_dout, d_dout;
    wire         r_qv, d_qv, r_qw, d_qw;
    wire [28:0]  r_qa, d_qa;
    wire [255:0] r_qd, d_qd;
    wire [31:0]  r_qm, d_qm;
    tango_ddr3_cport_ref #(.RF_LOG2(`RF_LOG2)) u_ref (
        .DDRAM_CLK(cclk), .crst(crst), .DDRAM_BUSY(r_busy), .DDRAM_BURSTCNT(g_bc), .DDRAM_ADDR(g_addr), .DDRAM_DOUT(r_dout),
        .DDRAM_DOUT_READY(r_dready), .DDRAM_RD(g_rd), .DDRAM_DIN(g_din), .DDRAM_BE(g_be), .DDRAM_WE(g_we),
        .clk(clk), .rst(rst), .q_valid(r_qv), .q_write(r_qw), .q_addr(r_qa), .q_wdata(r_qd), .q_wmask(r_qm), .q_ready(q_ready),
        .r_valid(r_valid), .r_data(r_data));
    tango_ddr3_cport #(.RF_LOG2(`RF_LOG2)) u_dut (
        .DDRAM_CLK(cclk), .crst(crst), .DDRAM_BUSY(d_busy), .DDRAM_BURSTCNT(g_bc), .DDRAM_ADDR(g_addr), .DDRAM_DOUT(d_dout),
        .DDRAM_DOUT_READY(d_dready), .DDRAM_RD(g_rd), .DDRAM_DIN(g_din), .DDRAM_BE(g_be), .DDRAM_WE(g_we),
        .clk(clk), .rst(rst), .q_valid(d_qv), .q_write(d_qw), .q_addr(d_qa), .q_wdata(d_qd), .q_wmask(d_qm), .q_ready(q_ready),
        .r_valid(r_valid), .r_data(r_data));

    // ---- random numbers (Verilator's $random(seed) collapses after a few thousand calls: own xorshift32)
    reg [31:0] rs = 32'h9E3779B9 ^ (`SEED * 32'h01000193);
    function [31:0] rnd32(input integer dummy);
        begin rs = rs ^ (rs << 13); rs = rs ^ (rs >> 17); rs = rs ^ (rs << 5); rnd32 = rs; end
    endfunction
    function integer rnd(input integer n);        // 0..n-1
        reg [31:0] r;
        begin r = rnd32(0); rnd = (r >> 3) % n; end
    endfunction

    // ---- the phase table (changes every PH clk_ddr clocks)
    integer clkn = 0;
    integer ph_cmd = 100, ph_wr = 50, ph_len = 0, ph_gap = 0, ph_algn = 0, ph_back = 50;   // core side
    integer ph_qr = 0, ph_lat = 0;                                                             // arbiter side
    always @(posedge clk) begin
        clkn <= clkn + 1;
        if (clkn % PH == 0) begin
            ph_cmd  = (rnd(4) == 0) ? 10 : (rnd(3) == 0 ? 40 : 100);
            ph_wr   = (rnd(5) == 0) ? 100 : (rnd(5) == 0 ? 0 : 20 + rnd(60));
            ph_len  = rnd(8);          // 0 mix, 1 single, 2 short, 3 long, 4 max (255), 5 medium, 6 two words, 7 mix
            ph_gap  = (rnd(3) == 0) ? 40 : 0;
            ph_algn = (rnd(4) == 0) ? 1 : 0;
            ph_back = (rnd(2) == 0) ? 90 : 20;
            ph_qr   = rnd(5);          // 0 always, 1 50%, 2 10%, 3 stall bursts, 4 90%
            ph_lat  = rnd(5);          // 0 short, 1 medium, 2 long (ri ring binds), 3 minimal, 4 mixed
        end
    end

    // ---- the core-side generator (MiSTer protocol: a request is held until DDRAM_BUSY is low at a DDRAM_CLK edge)
    reg [63:0] shadow [int];
    reg [255:0] qmem [int];
    reg [63:0] expq [$];
    reg [24:0] expa [$];                          // (the beat's word address, for the failure report)
    function [63:0] h64(input [24:0] idx);
        reg [63:0] x;
        begin x = {39'd0, idx} * 64'h9E3779B97F4A7C15; h64 = x ^ (x >> 29) ^ 64'hA5A5_0000_5A5A_1234; end
    endfunction
    function [63:0] sh_get(input [24:0] idx);
        begin if (shadow.exists(int'(idx))) sh_get = shadow[int'(idx)]; else sh_get = h64(idx); end
    endfunction
    function [255:0] qm_get(input [22:0] w);
        reg [63:0] w0, w1, w2, w3;
        begin
            if (qmem.exists(int'(w))) qm_get = qmem[int'(w)];
            else begin
                w0 = h64({w, 2'd0}); w1 = h64({w, 2'd1}); w2 = h64({w, 2'd2}); w3 = h64({w, 2'd3});
                qm_get = {w3, w2, w1, w0};
            end
        end
    endfunction
    integer   gbeat = 0, gtotal = 0, ggap = 0;
    reg [24:0] gidx = 25'd0;
    reg        gmid = 1'b0;                     // inside a write burst
    reg        gacc;
    reg [24:0] apick, widx;
    integer    k;
    reg [63:0] sw, nw;
    integer    n_rd_cmds = 0, n_wr_cmds = 0, n_b0 = 0, n_b1 = 0, n_b255 = 0, n_gap = 0, n_back = 0, n_dout = 0;
    integer    n_lane_rd [0:3];
    integer    n_lane_wr [0:3];
    initial for (k = 0; k < 4; k = k + 1) begin n_lane_rd[k] = 0; n_lane_wr[k] = 0; end
    function integer pick_len(input integer dummy);
        integer m, l;
        begin
            m = ph_len;
            if (m == 0 || m == 7) begin
                case (rnd(10))
                    0, 1, 2, 3: l = 1;
                    4, 5:       l = 2 + rnd(3);
                    6:          l = 5 + rnd(12);
                    7:          l = 17 + rnd(48);
                    8:          l = 65 + rnd(191);
                    default:    l = 255;
                endcase
            end else if (m == 1) l = 1;
            else if (m == 2) l = 1 + rnd(4);
            else if (m == 3) l = 65 + rnd(191);
            else if (m == 4) l = 255;
            else if (m == 5) l = 5 + rnd(60);
            else l = 4 + rnd(5);
`ifndef NO_BC0
            if (rnd(6000) == 0) l = 0;
`endif
            pick_len = l;
        end
    endfunction
    always @(posedge cclk) begin
        if (crst) begin
            g_rd <= 1'b0; g_we <= 1'b0; gmid = 1'b0; ggap = 0; shadow.delete(); expq.delete(); expa.delete();
        end else begin
            gacc = (g_rd || g_we) && !r_busy;           // the request presented now is taken at this edge
            if (gacc && g_rd) begin
                for (k = 0; k < ((g_bc == 0) ? 0 : g_bc); k = k + 1) begin expq.push_back(sh_get(gidx + 25'(k))); expa.push_back(gidx + 25'(k)); end
                g_rd <= 1'b0; n_rd_cmds = n_rd_cmds + 1;
            end
            if (gacc && g_we) begin
                widx = gidx + 25'(gbeat);                           // (25 bits: the address wraps at 2^25)
                sw = sh_get(widx); nw = sw;
                for (k = 0; k < 8; k = k + 1) if (g_be[k]) nw[8*k +: 8] = g_din[8*k +: 8];
                shadow[int'(widx)] = nw;
                gbeat = gbeat + 1;
                if (gbeat >= ((g_bc == 0) ? 1 : g_bc)) begin g_we <= 1'b0; gmid = 1'b0; n_wr_cmds = n_wr_cmds + 1; end
                else begin
                    g_din <= {rnd32(0), rnd32(0)}; g_be <= (rnd(3) == 0) ? 8'hFF : rnd32(0);
                    if (ph_gap != 0 && rnd(100) < ph_gap) begin g_we <= 1'b0; ggap = 1 + rnd(4); n_gap = n_gap + 1; end
                end
            end
            // a gap inside a write burst: re-present the next beat after it
            if (gmid && !g_we && !g_rd) begin
                if (ggap > 0) ggap = ggap - 1;
                if (ggap == 0) g_we <= 1'b1;
            end
            // a new command: when nothing is presented (or, back to back, the one presented was just taken)
            if (!gmid && ((!g_rd && !g_we) || (gacc && rnd(100) < ph_back)) && rnd(100) < ph_cmd) begin
                gtotal = pick_len(0);
                g_bc <= gtotal[7:0];
                apick = (rnd(10) < 7) ? 25'(rnd(2048)) : (rnd(10) == 0 ? 25'h1FFFFF0 + 25'(rnd(16)) : 25'(rnd32(0)));
                if (ph_algn == 1) apick[1:0] = 2'b00;
                g_addr <= {4'(rnd32(0) % 16), apick};                         // [28:25] ignored by the port
                gidx = apick;
                if (gacc) n_back = n_back + 1;
                gbeat = 0;
                if (gtotal == 0) n_b0 = n_b0 + 1;
                if (gtotal == 1) n_b1 = n_b1 + 1;
                if (gtotal == 255) n_b255 = n_b255 + 1;
                if (rnd(100) < ph_wr) begin
                    g_we <= 1'b1; g_rd <= 1'b0; gmid = (gtotal > 1);
                    g_din <= {rnd32(0), rnd32(0)}; g_be <= (rnd(3) == 0) ? 8'hFF : rnd32(0);
                    n_lane_wr[gidx[1:0]] = n_lane_wr[gidx[1:0]] + 1;
                end else begin
                    g_rd <= 1'b1; g_we <= 1'b0;
                    n_lane_rd[gidx[1:0]] = n_lane_rd[gidx[1:0]] + 1;
                end
            end
        end
    end

    // ---- the arbiter side: q_ready, an ideal memory behind q_*, the in-order returns
    reg [255:0] pipe_d [$];
    integer     pipe_t [$];
    integer     last_t = 0;
    integer     stallrun = 0;
    integer     lat;
    reg [255:0] wd;
    integer     bi;
    reg         qacc;
    always @(posedge clk) begin
        if (rst) begin
            pipe_d.delete(); pipe_t.delete(); qmem.delete(); r_valid <= 1'b0; last_t = clkn;
        end else begin
            qacc = r_qv && q_ready;
            if (qacc) begin
                if (r_qw) begin
                    wd = qm_get(r_qa[25:3]);
                    for (bi = 0; bi < 32; bi = bi + 1) if (!r_qm[bi]) wd[8*bi +: 8] = r_qd[8*bi +: 8];
                    qmem[int'(r_qa[25:3])] = wd;
                end else begin
                    case (ph_lat)
                        0: lat = 6 + rnd(5);
                        1: lat = 15 + rnd(26);
                        2: lat = 60 + rnd(91);
                        3: lat = 1;
                        default: lat = 1 + rnd(40);
                    endcase
                    if (clkn + lat <= last_t) lat = last_t + 1 - clkn;
                    last_t = clkn + lat;
                    pipe_d.push_back(qm_get(r_qa[25:3])); pipe_t.push_back(last_t);
                end
            end
            if (pipe_t.size() > 0 && pipe_t[0] <= clkn) begin
                r_valid <= 1'b1; r_data <= pipe_d[0]; pipe_d.pop_front(); pipe_t.pop_front();
            end else begin
                r_valid <= 1'b0; r_data <= {rnd32(0), rnd32(0), rnd32(0), rnd32(0), rnd32(0), rnd32(0), rnd32(0), rnd32(0)};
            end
        end
        // q_ready for the next clock
        case (ph_qr)
            0: q_ready <= 1'b1;
            1: q_ready <= (rnd(2) == 0);
            2: q_ready <= (rnd(10) == 0);
            3: begin
                if (stallrun > 0) begin stallrun = stallrun - 1; q_ready <= 1'b0; end
                else begin q_ready <= 1'b1; if (rnd(20) == 0) stallrun = 5 + rnd(40); end
            end
            default: q_ready <= (rnd(10) != 0);
        endcase
    end

`ifdef DBG
    always @(posedge clk) if (!rst && clkn > `DBG_T0 && clkn < `DBG_T1) begin
        if (r_qv && q_ready) $display("DBG q accept clkn=%0d write %b addr %h mask %h wdata[63:0] %h", clkn, r_qw, r_qa, r_qm, r_qd[63:0]);
        if (r_valid) $display("DBG r_valid clkn=%0d data %h", clkn, r_data);
    end
    always @(posedge cclk) if (!crst && clkn > `DBG_T0 && clkn < `DBG_T1) begin
        if ((g_rd || g_we) && !r_busy) $display("DBG core accept clkn=%0d rd %b we %b bc %0d addr %h gidx %h gbeat %0d be %h din %h", clkn, g_rd, g_we, g_bc, g_addr, gidx, gbeat, g_be, g_din);
        if (d_dready) $display("DBG DOUT clkn=%0d %h expect %h", clkn, d_dout, expq.size() > 0 ? expq[0] : 64'hx);
    end
`endif
    // ---- resets: both overlap, each held long enough for the slower domain's synchronisers; release order random
    integer rcnt = 0;
    integer n_rst_busy = 0;
    initial begin
        rst = 1'b1; crst = 1'b1;
        #1500; rst = 1'b0; crst = 1'b0;
        forever begin
            #(10 * (`RESET_EVERY / 2 + rnd(`RESET_EVERY)));
            if (clkn < `NCLK) begin
                rcnt = rcnt + 1;
                if (u_ref.busy || u_ref.rf_res != 0 || u_ref.ri_wp != u_ref.ri_rp) n_rst_busy = n_rst_busy + 1;
                if (rnd(2) == 0) begin rst = 1'b1; #(rnd(30)); crst = 1'b1; end else begin crst = 1'b1; #(rnd(30)); rst = 1'b1; end
                #(1500 + 10 * rnd(200));
                if (rnd(2) == 0) begin rst = 1'b0; #(rnd(300)); crst = 1'b0; end else begin crst = 1'b0; #(rnd(300)); rst = 1'b0; end
            end
        end
    end

    // ---- the compare
    reg failed = 1'b0;
    task fail(input [255:0] what);
        begin
            if (!failed) begin
                failed <= 1'b1;
                $display("TB EQUIV MISMATCH %0s at %0t (clkn %0d)", what, $time, clkn);
                $display("   ref: busy %b dout %h dready %b | qv %b qw %b qa %h qm %h", r_busy, r_dout, r_dready, r_qv, r_qw, r_qa, r_qm);
                $display("   dut: busy %b dout %h dready %b | qv %b qw %b qa %h qm %h", d_busy, d_dout, d_dready, d_qv, d_qw, d_qa, d_qm);
                $display("TB_RESULT FAIL");
                $finish;
            end
        end
    endtask
    always @(negedge cclk) begin
        if (r_busy !== d_busy) fail("DDRAM_BUSY");
        if (r_dout !== d_dout) fail("DDRAM_DOUT");
        if (r_dready !== d_dready) fail("DDRAM_DOUT_READY");
        if (u_ref.inb !== u_dut.inb || u_ref.left !== u_dut.left || u_ref.rf_have !== u_dut.rf_have) fail("core state");
        // the functional scoreboard: what the DUT hands the core is what the core should read
        if (!crst && d_dready) begin
            n_dout = n_dout + 1;
            if (expq.size() == 0) begin $display("TB FUNC: DOUT beat with nothing expected at %0t", $time); fail("func extra"); end
            else begin
                if (expq[0] !== d_dout) begin $display("TB FUNC: DOUT %h expected %h (word %h, in shadow %b, in qmem %b) at %0t", d_dout, expq[0], expa[0], shadow.exists(int'(expa[0])), qmem.exists(int'(expa[0] >> 2)), $time); fail("func data"); end
                expq.pop_front(); expa.pop_front();
            end
        end
    end
    // coverage (from the reference's state; the DUT's equal to it by the compare)
    integer c_ri_block = 0, c_rf_block = 0, c_rgo_rv = 0, c_wgo_b2b = 0, c_rgo_b2b = 0, c_ri_max = 0, c_qstall = 0, c_cf_full = 0,
            c_wf_full = 0, c_skn2 = 0, c_pop_go = 0, c_wgo = 0, c_rgo = 0, c_wgo_stall = 0;
    reg prev_wgo = 1'b0, prev_rgo = 1'b0;
    wire [5:0] ri_cnt_ref = u_ref.ri_wp - u_ref.ri_rp;
    wire ref_qfree = !r_qv || q_ready;
    wire ref_rgo = u_ref.busy && !u_ref.s_w && u_ref.sn != 0 && ref_qfree && u_ref.rf_ok && u_ref.ri_ok;
    wire ref_wgo = u_ref.busy && u_ref.s_w && u_ref.sn != 0 && ({6'd0, u_ref.acc_n} == u_ref.nb) && ref_qfree;
    always @(negedge clk) if (!rst) begin
        if (u_ref.busy && !u_ref.s_w && u_ref.sn != 0 && ref_qfree && u_ref.rf_ok && !u_ref.ri_ok) c_ri_block = c_ri_block + 1;
        if (u_ref.busy && !u_ref.s_w && u_ref.sn != 0 && ref_qfree && !u_ref.rf_ok && u_ref.ri_ok) c_rf_block = c_rf_block + 1;
        if (ref_rgo && r_valid) c_rgo_rv = c_rgo_rv + 1;
        if (ref_wgo && prev_wgo) c_wgo_b2b = c_wgo_b2b + 1;
        if (ref_rgo && prev_rgo) c_rgo_b2b = c_rgo_b2b + 1;
        if (ref_wgo) c_wgo = c_wgo + 1;
        if (ref_rgo) c_rgo = c_rgo + 1;
        if (ri_cnt_ref > c_ri_max) c_ri_max = ri_cnt_ref;
        if (r_qv && !q_ready) c_qstall = c_qstall + 1;
        if (u_ref.busy && u_ref.s_w && u_ref.sn != 0 && ({6'd0, u_ref.acc_n} == u_ref.nb) && !ref_qfree) c_wgo_stall = c_wgo_stall + 1;
        if (u_ref.cf_free < 5'd2) c_cf_full = c_cf_full + 1;
        if (u_ref.wf_free < 9'd2) c_wf_full = c_wf_full + 1;
        if (u_ref.skn == 2'd2) c_skn2 = c_skn2 + 1;
        prev_wgo = ref_wgo; prev_rgo = ref_rgo;
    end
    always @(negedge clk) begin
        if (r_qv !== d_qv) fail("q_valid");
        if (r_qw !== d_qw) fail("q_write");
        if (r_qa !== d_qa) fail("q_addr");
        if (r_qd !== d_qd) fail("q_wdata");
        if (r_qm !== d_qm) fail("q_wmask");
        if (u_ref.busy !== u_dut.busy || u_ref.sa !== u_dut.sa || u_ref.sn !== u_dut.sn || u_ref.nb !== u_dut.nb
            || u_ref.s_w !== u_dut.s_w || u_ref.skn !== u_dut.skn || u_ref.rdp !== u_dut.rdp || u_ref.acc_n !== u_dut.acc_n
            || u_ref.fl !== u_dut.fl || u_ref.cf_pend !== u_dut.cf_pend || u_ref.cf_rd !== u_dut.cf_rd) fail("splitter state");
        if (u_ref.ri_wp !== u_dut.ri_wp || u_ref.ri_rp !== u_dut.ri_rp || u_ref.rf_res !== u_dut.rf_res
            || u_ref.rf_ok !== u_dut.rf_ok || u_ref.ri_ok !== u_dut.ri_ok || u_ref.rf_we !== u_dut.rf_we) fail("return state");
        if (u_ref.wf_rd !== u_dut.wf_rd) fail("wf_rd");
        if (clkn >= `NCLK && !failed) begin
            $display("TB cport equiv: %0d clk_ddr clocks, %0d resets (%0d with traffic in flight); core cmds: %0d rd %0d wr, bursts 0:%0d 1:%0d 255:%0d, back-to-back %0d, write gaps %0d, DOUT beats checked %0d",
                     clkn, rcnt, n_rst_busy, n_rd_cmds, n_wr_cmds, n_b0, n_b1, n_b255, n_back, n_gap, n_dout);
            $display("TB   start lanes rd %0d %0d %0d %0d  wr %0d %0d %0d %0d", n_lane_rd[0], n_lane_rd[1], n_lane_rd[2], n_lane_rd[3],
                     n_lane_wr[0], n_lane_wr[1], n_lane_wr[2], n_lane_wr[3]);
            $display("TB   cover (w_go b2b is structurally 0: a word needs a beat taken between two sends): r_go %0d w_go %0d, r_go b2b %0d, w_go b2b %0d, r_go+r_valid same clock %0d, ri_ok blocks r_go %0d, rf_ok blocks r_go %0d, ri ring max %0d, q stall %0d (w_go held by it %0d), cmd FIFO full %0d, beat FIFO full %0d, skid 2 %0d",
                     c_rgo, c_wgo, c_rgo_b2b, c_wgo_b2b, c_rgo_rv, c_ri_block, c_rf_block, c_ri_max, c_qstall, c_wgo_stall, c_cf_full, c_wf_full, c_skn2);
            if (c_rgo == 0 || c_wgo == 0 || c_rgo_b2b == 0 || c_rgo_rv == 0 || c_ri_block == 0 || c_rf_block == 0 || c_ri_max < 31
                || c_qstall == 0 || c_wgo_stall == 0 || c_cf_full == 0 || c_wf_full == 0 || c_skn2 == 0 || n_b1 == 0 || n_b255 == 0
                || n_gap == 0 || n_back == 0 || n_dout == 0 || n_rst_busy == 0
                || n_lane_rd[0] == 0 || n_lane_rd[1] == 0 || n_lane_rd[2] == 0 || n_lane_rd[3] == 0
                || n_lane_wr[0] == 0 || n_lane_wr[1] == 0 || n_lane_wr[2] == 0 || n_lane_wr[3] == 0) begin
                $display("TB COVERAGE HOLE: a coverage counter above is zero (or the ri ring never reached 31)");
                $display("TB_RESULT FAIL");
            end else $display("TB_RESULT PASS");
            $finish;
        end
    end
endmodule
