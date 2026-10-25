// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_fifo -- a one-clock FIFO with its head always presented (q / q_valid; pop takes it), for the DDR3 ports.
// The memory is read with a registered read (block RAM friendly: q <= mem[rp]) into the head register whenever the
// head is empty or being popped, so a word written at clock t is at the head at t + 2 at the earliest.
// count = words held (memory + head); full when count = 2^AW.  Write only when !full.  The memory's word count and
// "not empty" are registers (timing, below).
//==============================================================================
`default_nettype none
module tango_ddr3_fifo #(
    parameter integer W  = 32,
    parameter integer AW = 4
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          we,
    input  wire [W-1:0]  wd,
    output wire [AW:0]   count,
    output wire          full,
    output reg  [W-1:0]  q = {W{1'b0}},
    output reg           q_valid = 1'b0,
    input  wire          pop
);
    reg [W-1:0]  mem [0:(1<<AW)-1];
    reg [AW:0]   wp = 0, rp = 0;
    // the words in the memory, a register, and "not empty", a register (timing: wp - rp -> != 0 -> load -> q_valid
    // failed clk_x1 by 0.215 ns in an earlier Template core DDR3 build); both next values are computed for a load and
    // for none from registers, and the late load only selects
    reg [AW:0]   inmem = 0;
    reg          ne = 1'b0;
    assign count = inmem + (q_valid ? 1'b1 : 1'b0);
    assign full  = count >= (1 << AW);
    wire load = ne && (!q_valid || pop);
    // the candidates are kept as nets of their own (syn_keep, the comment form of the board top's spi_sck_clk), so
    // synthesis cannot fold the load into their adders
    // (it did: an earlier build failed by 0.485 ns on load -> inmem's carry chain)
    wire [AW:0] n0 /* synthesis syn_keep = 1 */ = inmem + (we ? 1'b1 : 1'b0);           // no load
    wire [AW:0] n1 /* synthesis syn_keep = 1 */ = inmem + (we ? 1'b1 : 1'b0) - 1'b1;    // a load
    wire        ne0 /* synthesis syn_keep = 1 */ = we || ne;                            // n0 != 0
    wire        ne1 /* synthesis syn_keep = 1 */ = (inmem > 1) || (ne && we);           // n1 != 0
    always @(posedge clk) begin
        if (we) mem[wp[AW-1:0]] <= wd;
        if (load) q <= mem[rp[AW-1:0]];
        if (rst) begin wp <= 0; rp <= 0; inmem <= 0; ne <= 1'b0; q_valid <= 1'b0; end
        else begin
            if (we) wp <= wp + 1'b1;
            if (load) begin rp <= rp + 1'b1; q_valid <= 1'b1; end
            else if (pop) q_valid <= 1'b0;
            inmem <= load ? n1 : n0;
            ne    <= load ? ne1 : ne0;
        end
    end
endmodule
`default_nettype wire
//==============================================================================
// tango_ddr3_skid -- a 2-entry queue whose input ready is a register: the producer's logic never waits on the
// consumer's same-clock decision (the consumer pops the head combinationally, e.g. from the DDR3 IP's cmd_ready).
// in_ready high at a clock guarantees room for one word that clock.  out_pop only with out_valid.
// out_wb is out_data[WB] as a REGISTER, while the queue holds a word (timing: the arbiter's grant starts at the head's
// write flag; read from rp ? e1 : e0, that was a 316-load pointer through a LUT in front of the longest path on
// clk_x1).  It is kept from the queue's count n alone: with one word it is the head's; with two, the head's and
// (out_wo) the second one's.  The next head's flag is therefore, by n:
//   n = 0   the word being pushed (if any)        n = 1   a pop takes the word being pushed, else the head stays
//   n = 2   a pop brings the second word (out_wo), else the head stays
// so it depends on the late out_pop only in a last 2:1 select, and not on rp, wp or the push at all.  (Empty, out_wb
// is not meant to follow anything.)  It equals out_data[WB] every clock the queue is not empty (checked in simulation).
//==============================================================================
`default_nettype none
module tango_ddr3_skid #(
    parameter integer W  = 32,
    parameter integer WB = W - 1                  // the bit out_wb follows
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          in_valid,
    input  wire [W-1:0]  in_data,
    output reg           in_ready = 1'b1,
    output wire          out_valid,
    output wire [W-1:0]  out_data,
    output reg           out_wb = 1'b0,           // = out_data[WB] when out_valid, a register
    input  wire          out_pop
);
    reg [W-1:0] e0 = {W{1'b0}}, e1 = {W{1'b0}};
    reg         rp = 1'b0, wp = 1'b0;
    reg [1:0]   n = 2'd0;
    wire        push = in_valid && in_ready;
    wire [2:0]  nn = {1'b0, n} + (push ? 3'd1 : 3'd0) - (out_pop ? 3'd1 : 3'd0);
    assign out_valid = (n != 2'd0);
    assign out_data  = rp ? e1 : e0;
    reg out_wo = 1'b0;                              // the second word's flag (n = 2), set when it is pushed
    wire wb_pop  /* synthesis syn_keep = 1 */ = (n == 2'd2) ? out_wo : in_data[WB];   // after a pop
    wire wb_stay /* synthesis syn_keep = 1 */ = (n == 2'd0) ? in_data[WB] : out_wb;   // after no pop
    always @(posedge clk) begin
        out_wb <= out_pop ? wb_pop : wb_stay;
        if (n == 2'd1) out_wo <= in_data[WB];   // the second word's push (else a value nobody reads: out_wo is read only with n = 2, set from n = 1)
        if (push) begin if (wp) e1 <= in_data; else e0 <= in_data; end
        if (rst) begin rp <= 1'b0; wp <= 1'b0; n <= 2'd0; in_ready <= 1'b1; end
        else begin
            if (push) wp <= ~wp;
            if (out_pop) rp <= ~rp;
            n <= nn[1:0];
            in_ready <= (nn <= 3'd1);
        end
    end
    // The registered flag equals the entry the head shows, every clock the queue holds a word.  Simulation only (translate_off, as the
    // arbiter's contract checks).
    /* synthesis translate_off */
    integer wbc_clocks = 0;
    always @(posedge clk) begin
        if (out_valid && out_wb !== out_data[WB]) begin
            $display("TANGO CONTRACT: DDR3 skid out_wb differs at %0t (%m): out_wb %b, out_data[WB] %b", $time, out_wb, out_data[WB]);
            $fatal(1, "TANGO CONTRACT: DDR3 skid out_wb violated (see the line above)");
        end
        wbc_clocks <= wbc_clocks + 1;
    end
    final
        $display("TANGO CONTRACT OK: DDR3 skid out_wb equal to out_data[WB] in %0d clocks (%m)", wbc_clocks);
    /* synthesis translate_on */
endmodule
`default_nettype wire
