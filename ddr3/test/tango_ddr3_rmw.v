// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_rmw -- the DDR3 bring-up test's running statistics: a 32 x 32 block RAM updated by a pipelined
// read-modify-write, one operation a clock, any address, back to back.
// Operations (in_op):
//   OP_INC    word + 1 in its high (in_hi) or low 16-bit half, saturating at 0xFFFF       (a histogram bin)
//   OP_MRG    {sat16(word[31:16] + d[31:16]), max(word[15:0], d[15:0])}                (a count and a maximum)
//   OP_RDCLR  the result is the word; the word becomes 0                                  (read out and clear)
//   OP_WR     the word becomes d
// Pipeline (an operation enters on in_v):
//   P0  its registers give the RAM its address: the RAM reads at P0's closing edge
//   P1  the RAM's word (its read register, bypass)
//   P2  the word registered, the newest value chosen (below), modified
//   P3  the write (at P3's closing edge); out_v / out_val / out_tag: the operation's result (OP_RDCLR: the old word,
//       the others: the new one)
//   P4, P5  P3's write, kept for the forwarding
// Forwarding.  The three operations ahead of an operation X write at X's read edge (the one in P3 then) or after it
// (the ones in P2, P1): X read the RAM before their writes landed, or during one.  In P2, X takes its word from P3, P4
// or P5 when the operation there wrote X's address (the newest first; the choice is registered in P1), and from the
// RAM only when none did.  So a read during a write at the same address -- undefined on the BSRAM: GowinSynthesis
// leaves no bypass logic around an inferred RAM (syn_ramstyle no_rw_check, its default: SUG550 Table 5-36) -- is
// never used, and the RAM's read-during-write mode does not matter.  In Verilator such a read returns 0xDEADBEEF, so a
// missed forward shows; sim/tb_ddr3_rmw.v drives back-to-back operations on few addresses against a model.
//==============================================================================
`default_nettype none
module tango_ddr3_rmw (
    input  wire        clk,
    input  wire        in_v,
    input  wire [1:0]  in_op,
    input  wire [4:0]  in_a,
    input  wire        in_hi,
    input  wire [31:0] in_d,
    input  wire [7:0]  in_tag,
    output wire        out_v,
    output wire [31:0] out_val,
    output wire [7:0]  out_tag,
    output wire        busy                     // an operation in P0..P3
);
    localparam [1:0] OP_INC = 2'd0, OP_MRG = 2'd1, OP_RDCLR = 2'd2, OP_WR = 2'd3;
    (* syn_ramstyle = "block_ram" *) reg [31:0] mem [0:31];
    reg        v0 = 1'b0, v1 = 1'b0, v2 = 1'b0, v3 = 1'b0, v4 = 1'b0;
    reg [1:0]  op0 = 2'd0, op1 = 2'd0, op2 = 2'd0, op3 = 2'd0;
    reg [4:0]  a0 = 5'd0, a1 = 5'd0, a2 = 5'd0, a3 = 5'd0, a4 = 5'd0;
    reg        hi0 = 1'b0, hi1 = 1'b0, hi2 = 1'b0;
    reg [31:0] d0 = 32'd0, d1 = 32'd0, d2 = 32'd0;
    reg [7:0]  t0 = 8'd0, t1 = 8'd0, t2 = 8'd0, t3 = 8'd0;
    reg [31:0] rq = 32'd0, rq2 = 32'd0;         // the RAM's word (P1) and its register (P2)
    reg [31:0] n3 = 32'd0, n4 = 32'd0, n5 = 32'd0;   // the new words in P3, P4, P5
    reg [31:0] o3 = 32'd0;                      // P3's old word
    reg        f3 = 1'b0, f4 = 1'b0, f5 = 1'b0; // P2: the word is P3's / P4's / P5's
    wire [31:0] old2 = f3 ? n3 : f4 ? n4 : f5 ? n5 : rq2;
    wire [16:0] sum2 = {1'b0, old2[31:16]} + {1'b0, d2[31:16]};
    wire [15:0] inc_hi = (old2[31:16] == 16'hFFFF) ? 16'hFFFF : old2[31:16] + 1'b1;
    wire [15:0] inc_lo = (old2[15:0] == 16'hFFFF) ? 16'hFFFF : old2[15:0] + 1'b1;
    reg  [31:0] new2;
    always @* begin
        case (op2)
            OP_INC:   new2 = hi2 ? {inc_hi, old2[15:0]} : {old2[31:16], inc_lo};
            OP_MRG:   new2 = {sum2[16] ? 16'hFFFF : sum2[15:0], (d2[15:0] > old2[15:0]) ? d2[15:0] : old2[15:0]};
            OP_RDCLR: new2 = 32'd0;
            default:  new2 = d2;
        endcase
    end
    always @(posedge clk) begin
        // P0: the operation's registers
        v0 <= in_v; op0 <= in_op; a0 <= in_a; hi0 <= in_hi; d0 <= in_d; t0 <= in_tag;
        // the RAM: read at P0's closing edge, written from P3
        rq <= mem[a0];
`ifdef VERILATOR
        if (v3 && a3 == a0) rq <= 32'hDEAD_BEEF;            // read during a write at one address: undefined
`endif
        if (v3) mem[a3] <= n3;
        // P1, and the forwarding choice for P2: the operations now in P2 / P3 / P4 are in P3 / P4 / P5 then
        v1 <= v0; op1 <= op0; a1 <= a0; hi1 <= hi0; d1 <= d0; t1 <= t0;
        f3 <= v2 && (a2 == a1); f4 <= v3 && (a3 == a1); f5 <= v4 && (a4 == a1);
        // P2
        v2 <= v1; op2 <= op1; a2 <= a1; hi2 <= hi1; d2 <= d1; t2 <= t1; rq2 <= rq;
        // P3
        v3 <= v2; op3 <= op2; a3 <= a2; t3 <= t2; n3 <= new2; o3 <= old2;
        // P4, P5
        v4 <= v3; a4 <= a3; n4 <= n3;
        n5 <= n4;
    end
    assign out_v   = v3;
    assign out_val = (op3 == OP_RDCLR) ? o3 : n3;
    assign out_tag = t3;
    assign busy    = v0 | v1 | v2 | v3;
endmodule
`default_nettype wire
