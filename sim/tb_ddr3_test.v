// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_ddr3_test -- the DDR3 bring-up test's logic (ddr3/test/tango_ddr3_test_core.v) on the IP model (sim/tango_ddr3_model.v), with the
// four clocks asynchronous (clk_ref 99.966 MHz against clk_x1 100 MHz, clk_pix 74.25, clk50 50).
// Small sizes: a 1024-word march, 64 latency samples per mode, 1024-command bandwidth runs, 4096-clock stall windows,
// 1 ms "seconds", a 6.25 Mbaud UART with a line every 1 ms.  The raster steps CY_STEP lines at a time (25: a 667 us
// frame; the display takes a bank per frame, and a real 16.7 ms frame would make the run 10 times longer); the
// picture itself is checked by sim/tb_ddr3_ui.v on hdmi.sv's raster.  After NLOOP loops of the tester it checks the
// measurements against what the model was given (below), freezes the display, and writes:
//   obj/bank.txt   the 114 slots on screen (hex, one per line),
// which sim/tb_ddr3_ui.v draws (tango_ddr3_ov) and sends (tango_ddr3_uart), and sim/run_tb_ddr3_test.sh hands its
// frame and UART line to ddr3/test/decode_d1.py: both decodes must equal bank.txt.  Throughout, it checks:
//   every UART line received equals the copy the UART read (its RAM bank, its board slots), all 114 slots;
//   no display RAM write ever goes to the bank on screen, no UART RAM write to the bank the UART reads (after the
//   clear pass), the statistics RAM's users never collide, cm is never written while the copier reads it.
// Defines: SILICON (the model calibrated on silicon, sim/tango_ddr3_silicon.vh: the decode must show the silicon run's numbers),
// FAULT (model flips bit 37 of word 700 on every read: the checker must report exactly that), JITTER
// (read latency +0..8, an executor stall of 300 clocks every 5000), PSTOP (pll_stop low 1 us every 200 us),
// UTIMEOUT (the UART's requests are kept from clk_x1 from 0.9 to 2.5 ms, as if clk_x1 stopped: the first line must
// time out and go with the copy it has, the late answer must land in the other bank, and a later line must be fresh;
// the run then waits for 2 checked lines).
// TB_RESULT PASS / FAIL.
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_ddr3_test;
`ifndef NLOOP
`define NLOOP 3
`endif
`ifdef JITTER
    localparam integer RDJ = 8, STP = 5000, STL = 300;
`elsif SILICON
    localparam integer RDJ = 2, STP = 0, STL = 0;                // (SIL_RD_JITTER)
`else
    localparam integer RDJ = 0, STP = 0, STL = 0;
`endif
`ifdef SILICON
`include "tango_ddr3_silicon.vh"
    // the model calibrated on silicon (sim/tango_ddr3_silicon.vh): this bench then measures what silicon measured
    localparam integer RD_LAT = SIL_RD_LAT, REF_PERIOD = SIL_REF_PERIOD, REF_LEN = SIL_REF_LEN, CMD_DEPTH = SIL_CMD_DEPTH,
                       T_RM = SIL_T_ROWMISS, BAUD = 8;
`else
    localparam integer RD_LAT = 18, REF_PERIOD = 780, REF_LEN = 28, CMD_DEPTH = 32, T_RM = 3, BAUD = 8;
`endif

    reg clk50 = 1'b0, clk_ref = 1'b0, clk_x1 = 1'b0, clk_pix = 1'b0;
    always #10     clk50   = ~clk50;
    always #5.0017 clk_ref = ~clk_ref;
    always #5      clk_x1  = ~clk_x1;
    always #6.734  clk_pix = ~clk_pix;
    reg [11:0] cx = 12'd0;
    reg [10:0] cy = 11'd0;
`ifndef CY_STEP
`define CY_STEP 25
`endif
    always @(posedge clk_pix)
        if (cx == 12'd1649) begin cx <= 12'd0; cy <= (cy >= 11'd750 - `CY_STEP) ? 11'd0 : cy + `CY_STEP; end
        else cx <= cx + 1'b1;
    reg locks = 1'b0;
    initial #3000 locks = 1'b1;
    reg pll_stop = 1'b1;
    reg freeze = 1'b0;

    wire         ip_rst_n, init_calib_complete, cmd_ready, cmd_en, wr_data_rdy, wr_data_en, rd_data_valid, rd_data_end;
    wire [2:0]   cmd;
    wire [28:0]  addr;
    wire [255:0] wr_data, rd_data;
    wire [31:0]  wr_data_mask, n_refresh, n_rowmiss, n_bad_cmd;
    wire [23:0]  rgb;
    wire         uart_tx, led;
`ifdef FAULT
    localparam FEN = 1'b1;
`else
    localparam FEN = 1'b0;
`endif
    tango_ddr3_model #(.MEM_LOG2(14), .INIT_CLKS(500), .CMD_DEPTH(CMD_DEPTH), .WDATA_DEPTH(32), .RD_LAT(RD_LAT),
                       .RD_JITTER(RDJ), .T_BL(1), .T_ROWMISS(T_RM), .T_TURN(2), .REF_PERIOD(REF_PERIOD),
                       .REF_LEN(REF_LEN), .STALL_PERIOD(STP), .STALL_LEN(STL), .ADDR_LOW_PASS(1)) u_m (
        .clk(clk_x1), .rst_n(ip_rst_n), .init_calib_complete(init_calib_complete),
        .cmd(cmd), .cmd_en(cmd_en), .addr(addr), .cmd_ready(cmd_ready),
        .wr_data(wr_data), .wr_data_en(wr_data_en), .wr_data_end(wr_data_en), .wr_data_mask(wr_data_mask),
        .wr_data_rdy(wr_data_rdy), .rd_data(rd_data), .rd_data_valid(rd_data_valid), .rd_data_end(rd_data_end),
        .force_stall(1'b0), .ref_burst(1'b0), .fault_en(FEN), .fault_word(25'd700), .fault_xor(256'h1 << 37),
        .n_refresh(n_refresh), .n_rowmiss(n_rowmiss), .n_bad_cmd(n_bad_cmd));
    tango_ddr3_test_core #(.WORDS_LOG2(10), .NLAT_LOG2(6), .BW_LOG2(10), .STALL_LOG2(12), .SNAP_LOG2(12),
                           .SNAP5_LOG2(10), .SEC_CLKS(50000), .BAUD_DIV(BAUD), .UART_PERIOD(50000),
                           .UART_TO_LOG2(16), .DRAIN_CLKS(256)) u_core (
        .clk50(clk50), .clk_ref(clk_ref), .clk_x1(clk_x1), .clk_pix(clk_pix), .rst_pix(!locks),
        .ddr_pll_lock(locks), .hdmi_lock(locks), .pll_stop(pll_stop), .ip_rst_n(ip_rst_n),
        .init_calib_complete(init_calib_complete), .ddr_rst(!ip_rst_n),
        .cmd_ready(cmd_ready), .cmd_en(cmd_en), .cmd(cmd), .addr(addr),
        .wr_data_rdy(wr_data_rdy), .wr_data_en(wr_data_en), .wr_data(wr_data), .wr_data_mask(wr_data_mask),
        .rd_data_valid(rd_data_valid), .rd_data(rd_data),
        .cx(cx), .cy(cy), .rgb(rgb), .freeze(freeze), .uart_tx(uart_tx), .led(led));

`ifdef PSTOP
    initial begin
        wait (init_calib_complete);
        forever begin #200000; pll_stop = 1'b0; #1000; pll_stop = 1'b1; end
    end
`endif

    // ---- progress
    reg [3:0] ph_q = 4'd15;
    reg [5:0] st_q = 6'd63;
    always @(posedge clk_x1) begin
        ph_q <= u_core.u_t.ph; st_q <= u_core.u_t.ss;
        if (u_core.u_t.ph != ph_q || (`ifdef TRACE_ST 1'b1 `else 1'b0 `endif && u_core.u_t.ss != st_q))
            $display("TB ph %0d st %0d at %0t ns (loops %0d, outstanding %0d)", u_core.u_t.ph, u_core.u_t.ss, $time,
                     u_core.u_t.loops, u_core.u_t.outst);
    end
    initial begin wait (init_calib_complete); $display("TB calibrated at %0t ns", $time); end
`ifdef TRACE_SEC
    always @(posedge clk50) if (u_core.sec == 0) $display("TB sec %0d: x1 %0d ref %0d", u_core.uptime, u_core.x1_per_s, u_core.rf_per_s);
`endif

    // ---- a UART receiver: every line is checked against the copy the UART read (its RAM bank and board slots, which
    // stay put until its next line starts, 1 ms after this one ends)
    reg [7:0]   ubuf [0:2047];
    integer     un = 0, ulines = 0, uok = 0, ubad = 0;
    function [3:0] hexv(input [7:0] c);
        hexv = (c >= 8'h30 && c <= 8'h39) ? c - 8'h30 : c - 8'h41 + 4'd10;
    endfunction
    initial begin : g_urx
        integer b, s, j, nd; reg [7:0] ch; reg [31:0] got, want;
        forever begin
            @(negedge uart_tx);
            #(BAUD * 20 / 2);
            ch = 8'd0;
            for (b = 0; b < 8; b = b + 1) begin #(BAUD * 20); ch[b] = uart_tx; end
            #(BAUD * 20);
            if (un < 2048) begin ubuf[un] = ch; un = un + 1; end
            if (ch == 8'h0A) begin
                ulines = ulines + 1;
                if (un == 1030 && ubuf[0] == 8'h44 && ubuf[1] == 8'h31) begin
                    nd = 0;
                    for (s = 0; s < 114; s = s + 1) begin
                        got = 32'd0;
                        for (j = 0; j < 8; j = j + 1) got = {got[27:0], hexv(ubuf[3 + 9 * s + j])};
                        want = (s < 12) ? u_core.u5[32 * s +: 32] : u_core.um[{u_core.ur, s[6:0]}];
                        if (got != want) begin
                            nd = nd + 1;
                            if (nd <= 3) $display("TB UART line %0d slot %0d: sent %08x, its copy %08x", ulines, s, got, want);
                        end
                    end
                    if (nd == 0) uok = uok + 1; else ubad = ubad + 1;
                end else if (un > 2) begin
                    ubad = ubad + 1; $display("TB UART line %0d: %0d chars", ulines, un);
                end
                un = 0;
            end
        end
    end

`ifdef UTIMEOUT
    // ---- the UART's requests held from the copier (as a stopped clk_x1 would); timeouts counted
    integer uto_n = 0, ufresh = 0;
    always @(posedge clk50) if (u_core.ust == 2'd2 && (&u_core.uto) && u_core.uas[2] != u_core.u_rq) uto_n = uto_n + 1;
    always @(posedge clk50) if (u_core.ust == 2'd2 && u_core.uas[2] == u_core.u_rq) ufresh = ufresh + 1;
    initial begin : g_uto
        reg [2:0] v;
        #900000; v = u_core.urq_s; force u_core.urq_s = v;
        $display("TB UART requests held from clk_x1 at %0t", $time);
        #1600000; release u_core.urq_s;
        $display("TB UART requests released at %0t", $time);
    end
`endif

    // ---- the snapshot protocol and the statistics RAM's users, every clock
    integer perr = 0;
    always @(posedge clk_x1) if (u_core.u_t.clr_done) begin
        if (u_core.dm_we && u_core.dm_wa[7] == u_core.S) begin
            perr = perr + 1; if (perr < 5) $display("TB FAIL display RAM write to the bank on screen at %0t", $time);
        end
        if (u_core.um_we && u_core.um_wa[7] == u_core.ur) begin
            perr = perr + 1; if (perr < 5) $display("TB FAIL UART RAM write to the bank the UART reads at %0t", $time);
        end
        if ({1'b0, u_core.u_t.cr_v} + {1'b0, u_core.u_t.hv5} > 2'd1) begin
            perr = perr + 1; if (perr < 5) $display("TB FAIL statistics RAM: two users in one clock at %0t", $time);
        end
        if (u_core.u_t.cmw_e && u_core.u_t.cp_run) begin
            perr = perr + 1; if (perr < 5) $display("TB FAIL cm written while the copier reads it at %0t", $time);
        end
    end

    // ---- the checks
    integer errs = 0, f, i, x, y, hs;
    reg [31:0] v;
    // the committed latency statistics (tester slots in cm): mode m's {max, min} at 37 + 3m, sum at 38 + 3m; the
    // bursts' {max, min} at 55 (mode 3) / 57 (mode 4); histogram row r (modes 0..3, 5) at 59 + 8r: {bin 2g+1, bin 2g}
    function [15:0] lmin(input integer m); lmin = u_core.u_t.cm[37 + 3 * m][15:0];  endfunction
    function [15:0] lmax(input integer m); lmax = u_core.u_t.cm[37 + 3 * m][31:16]; endfunction
    function [31:0] lsum(input integer m); lsum = u_core.u_t.cm[38 + 3 * m];        endfunction
    function [15:0] hbin(input integer r, input integer b);
        reg [31:0] w;
        begin w = u_core.u_t.cm[59 + 8 * r + b / 2]; hbin = (b % 2) ? w[31:16] : w[15:0]; end
    endfunction
    task automatic chk(input [255:0] what, input ok, input [31:0] got);
        begin
            if (!ok) begin errs = errs + 1; $display("TB FAIL %0s: got %0d (0x%08x)", what, got, got); end
            else $display("TB ok   %0s: %0d (0x%08x)", what, got, got);
        end
    endtask
    initial begin
        wait (u_core.u_t.loops >= `NLOOP);
        wait (u_core.uptime >= 3);                  // the first "second" counts from time 0: skip it
        $display("TB loops %0d at %0t ns; model refreshes %0d, row misses %0d", u_core.u_t.loops, $time,
                 n_refresh, n_rowmiss);
        chk("protocol violations (cmd_en without cmd_ready, bad cmd)", n_bad_cmd == 0, n_bad_cmd);
        chk("march passes", u_core.u_t.passes >= `NLOOP, u_core.u_t.passes);
`ifdef FAULT
        chk("errors (fault)", u_core.u_t.err_words >= 3 * `NLOOP, u_core.u_t.err_words);
        chk("first error address = word 700 x 32", u_core.u_t.first_addr == 32'd700 * 32, u_core.u_t.first_addr);
        chk("first error DQ fold = bit 5", u_core.u_t.first_dq == 32'h20, u_core.u_t.first_dq);
        chk("first error byte map = lane 1 byte 0", u_core.u_t.first_lanes == 32'h10, u_core.u_t.first_lanes);
        chk("bad passes = passes", u_core.u_t.bad_passes == u_core.u_t.passes, u_core.u_t.bad_passes);
`else
        chk("errors", u_core.u_t.err_words == 0, u_core.u_t.err_words);
        chk("bad passes", u_core.u_t.bad_passes == 0, u_core.u_t.bad_passes);
`endif
        chk("words checked (64K units, >= 0)", 1'b1, u_core.u_t.words_checked[31:0]);
        chk("mask polarity 1 (1 = masked)", u_core.u_t.ord_pol == 1, u_core.u_t.ord_pol);
        chk("mask map", u_core.u_t.ord_okA == 32'hFFFF_FFFF, u_core.u_t.ord_okA);
        chk("burst order +0", u_core.u_t.ord_off0 == 32'hFEDC_BA98, u_core.u_t.ord_off0);
        chk("burst order +1", u_core.u_t.ord_off1 == 32'hCFED_8BA9, u_core.u_t.ord_off1);
        chk("burst order +2", u_core.u_t.ord_off2 == 32'hDCFE_98BA, u_core.u_t.ord_off2);
        chk("burst order +4", u_core.u_t.ord_off4 == 32'hBA98_FEDC, u_core.u_t.ord_off4);
        chk("FIFO writes taken before the first drop >= model depth", u_core.u_t.fw_first_max >= CMD_DEPTH,
            u_core.u_t.fw_first_max);
        chk("FIFO reads taken before the first drop >= model depth", u_core.u_t.fr_first_max >= CMD_DEPTH,
            u_core.u_t.fr_first_max);
        chk("refresh: longest read-stream gap >= REF_LEN", u_core.u_t.st_gap_c >= REF_LEN, u_core.u_t.st_gap_c);
        chk("refresh: longest read-stream gap <= REF_LEN + 24 + stalls", u_core.u_t.st_gap_c <= REF_LEN + 24 + STL + RDJ,
            u_core.u_t.st_gap_c);
        chk("refresh: gaps >= 16 in a 4096-clock window", u_core.u_t.st_gaps16 >= 4, u_core.u_t.st_gaps16);
        chk("latency m0 min >= RD_LAT", lmin(0) >= RD_LAT, lmin(0));
        chk("latency m0 min <= RD_LAT + 8", lmin(0) <= RD_LAT + 8, lmin(0));
        chk("latency m2 (row miss) min > m0 min", lmin(2) > lmin(0), lmin(2));
        chk("latency m5 (loaded) mean > m0 mean", lsum(5) > lsum(0), lsum(5));
        chk("burst m3 last >= first + 7", u_core.u_t.cm[55][15:0] >= lmin(3) + 7, u_core.u_t.cm[55][15:0]);
        for (i = 0; i < 5; i = i + 1) begin
            hs = 0;
            for (x = 0; x < 16; x = x + 1) hs = hs + hbin(i, x);
            chk("histogram sums to 64 samples", hs == 64, hs);
        end
        chk("cum latency max >= m5 max", u_core.u_t.cum_rdlat >= lmax(5), u_core.u_t.cum_rdlat);
        chk("bw seq write >= 1024 clocks", u_core.u_t.bw0 >= 1024, u_core.u_t.bw0);
        chk("bw rnd read > seq read", u_core.u_t.bw3 > u_core.u_t.bw1, u_core.u_t.bw3);
        chk("init clocks > 0", u_core.init_clk > 0, u_core.init_clk);
        chk("clk_x1 per (1 ms) second = 100000 +-2", (u_core.x1_per_s >= 99998) && (u_core.x1_per_s <= 100002),
            u_core.x1_per_s);
        chk("calibration drops", u_core.cal_drops == 0, u_core.cal_drops);
`ifdef PSTOP
        chk("pll_stop events after calibration", u_core.ps_ev_pc >= 2, u_core.ps_ev_pc);
        chk("pll_stop longest after calibration = 50 clk50 +-2", (u_core.ps_max_pc >= 48) && (u_core.ps_max_pc <= 52),
            u_core.ps_max_pc);
`endif
`ifdef UTIMEOUT
        wait (uok >= 2);
        chk("UART: lines timed out (clk_x1 held)", uto_n >= 1, uto_n);
        chk("UART: lines with a fresh copy", ufresh >= 1, ufresh);
`endif
        chk("UART lines equal to their copies (114 slots)", uok >= 1 && ubad == 0, uok);
        chk("UART lines wrong", ubad == 0, ubad);
        chk("snapshot protocol / statistics RAM violations", perr == 0, perr);
        // ---- freeze the display after two fresh banks, then dump the one on screen (frame, UART line: tb_ddr3_ui)
        begin : g_two                               // two banks taken since now
            integer n0;
            n0 = u_core.ncopy;
            wait (u_core.ncopy >= n0 + 2);
        end
        @(posedge clk_pix); freeze = 1'b1;
        repeat (8) @(posedge clk_pix);
        f = $fopen("bank.txt", "w");
        for (i = 0; i < 12; i = i + 1) $fdisplay(f, "%08x", u_core.b5_show[32 * i +: 32]);
        for (i = 12; i < 114; i = i + 1) $fdisplay(f, "%08x", u_core.dm[{u_core.S, i[6:0]}]);
        $fclose(f);
        $display("TB bank written (sim/tb_ddr3_ui.v draws it)");
        $display("TB_RESULT %0s (%0d failures)", (errs == 0) ? "PASS" : "FAIL", errs);
        $finish;
    end
    initial begin #200000000; $display("TB timeout"); $display("TB_RESULT FAIL (timeout)"); $finish; end
endmodule
`default_nettype wire
