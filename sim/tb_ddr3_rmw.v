// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_ddr3_rmw -- ddr3/test/tango_ddr3_rmw.v (the DDR3 bring-up test's read-modify-write statistics RAM) against a model that applies the
// operations in order: random operations, mostly back to back, on 4 addresses (and now and then any of the 32), so
// every forwarding distance (1, 2 and 3 operations ahead) and the read during a write at one address (which the RAM
// returns as 0xDEADBEEF in Verilator) come up thousands of times.  Every result (out_val, out_tag) must equal the
// model's, and the RAM must equal the model at the end.  Counts are printed per forwarding distance.
// TB_RESULT PASS / FAIL.
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_ddr3_rmw;
    localparam integer N = 400000;
    localparam [1:0] OP_INC = 2'd0, OP_MRG = 2'd1, OP_RDCLR = 2'd2, OP_WR = 2'd3;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg        in_v = 1'b0, in_hi = 1'b0;
    reg [1:0]  in_op = 2'd0;
    reg [4:0]  in_a = 5'd0;
    reg [31:0] in_d = 32'd0;
    reg [7:0]  in_tag = 8'd0;
    wire       out_v, busy;
    wire [31:0] out_val;
    wire [7:0] out_tag;
    tango_ddr3_rmw u (.clk(clk), .in_v(in_v), .in_op(in_op), .in_a(in_a), .in_hi(in_hi), .in_d(in_d), .in_tag(in_tag),
                      .out_v(out_v), .out_val(out_val), .out_tag(out_tag), .busy(busy));

    // the model and the expected results, in issue order
    reg [31:0] mm [0:31];
    reg [31:0] eq_val [0:4095];
    reg [7:0]  eq_tag [0:4095];
    integer    eq_w = 0, eq_r = 0, errs = 0, nres = 0, i;
    integer    dst [0:3];                      // operations issued 1 / 2 / 3 behind one at the same address
    reg [4:0]  la [0:2];                        // the last 3 issued addresses (la[0] newest)
    reg        lv [0:2];
    reg [31:0] xs = 32'h1D1_0001;               // xorshift32 (Verilator's $random(seed) gave a short, skewed sequence)
    integer    nop [0:4];                       // INC, MRG, RDCLR, WR, idle clocks
    function [31:0] xs32(input [31:0] x);
        reg [31:0] y;
        begin y = x ^ (x << 13); y = y ^ (y >> 17); xs32 = y ^ (y << 5); end
    endfunction
    function [15:0] sat_inc(input [15:0] x); sat_inc = (x == 16'hFFFF) ? x : x + 1'b1; endfunction
    task automatic issue(input [1:0] op, input [4:0] a, input hi, input [31:0] d, input [7:0] tag);
        reg [31:0] o, n; reg [16:0] s; integer h;
        begin
            o = mm[a];
            case (op)
                OP_INC:   n = hi ? {sat_inc(o[31:16]), o[15:0]} : {o[31:16], sat_inc(o[15:0])};
                OP_MRG:   begin s = {1'b0, o[31:16]} + {1'b0, d[31:16]};
                                n = {s[16] ? 16'hFFFF : s[15:0], (d[15:0] > o[15:0]) ? d[15:0] : o[15:0]}; end
                OP_RDCLR: n = 32'd0;
                default:  n = d;
            endcase
            mm[a] = n;
            eq_val[eq_w % 4096] = (op == OP_RDCLR) ? o : n; eq_tag[eq_w % 4096] = tag; eq_w = eq_w + 1;
            for (h = 0; h < 3; h = h + 1) if (lv[h] && la[h] == a) dst[h + 1] = dst[h + 1] + 1;
            la[2] = la[1]; lv[2] = lv[1]; la[1] = la[0]; lv[1] = lv[0]; la[0] = a; lv[0] = 1'b1;
            in_v <= 1'b1; in_op <= op; in_a <= a; in_hi <= hi; in_d <= d; in_tag <= tag;
        end
    endtask
    task automatic idle;
        begin
            in_v <= 1'b0;
            la[2] = la[1]; lv[2] = lv[1]; la[1] = la[0]; lv[1] = lv[0]; lv[0] = 1'b0;
        end
    endtask

    always @(posedge clk) begin
        if (out_v) begin
            if (eq_r == eq_w) begin errs = errs + 1; $display("TB FAIL result with none expected at %0t", $time); end
            else begin
                if (out_val !== eq_val[eq_r % 4096] || out_tag !== eq_tag[eq_r % 4096]) begin
                    errs = errs + 1;
                    if (errs < 10) $display("TB FAIL result %0d: got %08x tag %02x, want %08x tag %02x", eq_r, out_val,
                                            out_tag, eq_val[eq_r % 4096], eq_tag[eq_r % 4096]);
                end
                eq_r = eq_r + 1; nres = nres + 1;
            end
        end
    end

    initial begin : g_run
        integer r, k; reg [31:0] rr, d; reg [4:0] a; reg [1:0] op;
        for (i = 0; i < 32; i = i + 1) mm[i] = 32'd0;
        for (i = 0; i < 4; i = i + 1) dst[i] = 0;
        for (i = 0; i < 5; i = i + 1) nop[i] = 0;
        for (i = 0; i < 3; i = i + 1) begin la[i] = 5'd0; lv[i] = 1'b0; end
        // the clear pass, as the tester's
        for (k = 0; k < 32; k = k + 1) begin @(posedge clk); issue(OP_WR, k[4:0], 1'b0, 32'd0, 8'd0); end
        for (k = 0; k < N; k = k + 1) begin
            @(posedge clk);
            xs = xs32(xs); rr = xs;
            if (rr[3:0] < 4'd3) begin idle; nop[4] = nop[4] + 1; end    // ~19% gaps
            else begin
                a  = (rr[7:4] == 4'd0) ? rr[12:8] : {3'b000, rr[9:8]};   // mostly 4 addresses
                r  = rr[17:14];                                          // INC 8/16, MRG 3/16, RDCLR 2/16, WR 3/16
                op = (r < 8) ? OP_INC : (r < 11) ? OP_MRG : (r < 13) ? OP_RDCLR : OP_WR;
                xs = xs32(xs); d = xs;
                if (op == OP_WR && rr[18]) d = {16'hFFF0 + rr[22:19], 16'hFFF0 + rr[26:23]};   // near saturation
                nop[op] = nop[op] + 1;
                issue(op, a, rr[13], d, rr[31:24]);
            end
        end
        @(posedge clk); idle;
        repeat (12) @(posedge clk);
        if (eq_r != eq_w) begin errs = errs + 1; $display("TB FAIL %0d results missing", eq_w - eq_r); end
        for (i = 0; i < 32; i = i + 1)
            if (u.mem[i] !== mm[i]) begin errs = errs + 1; $display("TB FAIL word %0d: RAM %08x, model %08x", i, u.mem[i], mm[i]); end
        $display("TB %0d operations, %0d results; same address 1 / 2 / 3 operations ahead: %0d / %0d / %0d", eq_w, nres,
                 dst[1], dst[2], dst[3]);
        $display("TB INC %0d, MRG %0d, RDCLR %0d, WR %0d, idle clocks %0d", nop[0], nop[1], nop[2], nop[3], nop[4]);
        $display("TB_RESULT %0s (%0d failures)", (errs == 0 && dst[1] > 1000 && dst[2] > 1000 && dst[3] > 1000) ?
                 "PASS" : "FAIL", errs);
        $finish;
    end
endmodule
`default_nettype wire
