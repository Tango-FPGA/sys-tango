// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_fifo -- a one-clock FIFO with its head always presented (q / q_valid; pop takes it), for the DDR3 ports.
// The memory is read with a registered read (block RAM friendly: q <= mem[rp]) into the head register whenever the
// head is empty or being popped, so a word written at clock t is at the head at t + 2 at the earliest.
// count = words held (memory + head); full when count = 2^AW.  Write only when !full.  The memory's word count and
// "not empty" are registers (timing, below).
// WALL = 1: the memory entry at the write pointer is written with wd EVERY clock, we or not; we only advances the
// pointer and the count (timing: in the V port's command FIFO the late write enable reached
// each entry's 37 enables through a pointer decode, two long hops).  The entry at wp is always free: the entries
// rp .. wp-1 hold the words, and with !full at every write there are at most 2^AW - 1 of them whenever a clock writes
// (checked in simulation); an entry is read only into q, by a load, while it is counted.  So q, count and every
// output are unchanged; only free entries hold other values.
// FW > 0: the top FW bits of every word are held in registers instead of the memory (an array of flops, syn_ramstyle
// "registers", written with the same enable and pointer as the memory), and the head's top FW bits in a register
// loaded from it with the same load and read pointer (timing: in the V port's
// return buffer the pop reads the head's two flag bits, and the pop is in the load, the memory's read enable; read
// from the block RAM's output register, that loop began at a 2 ns clock-to-out and twice crossed the distance
// between the memory and the pop's logic).  Each entry and the head hold exactly the bits the memory held, so q,
// count and every output are unchanged.
//==============================================================================
`default_nettype none
module tango_ddr3_fifo #(
    parameter integer W  = 32,
    parameter integer AW = 4,
    parameter integer WALL = 0,
    parameter integer FW = 0                      // top bits of each word held in registers (0..W-1)
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          we,
    input  wire [W-1:0]  wd,
    output wire [AW:0]   count,
    output wire          full,
    output wire [W-1:0]  q,
    output reg           q_valid = 1'b0,
    input  wire          pop
);
    localparam integer MW = W - FW;                // the bits held in the memory
    reg [MW-1:0] mem [0:(1<<AW)-1];
    reg [MW-1:0] qm = {MW{1'b0}};
    reg [AW:0]   wp = 0, rp = 0;
    // the words in the memory, a register, and "not empty", a register (timing: wp - rp -> != 0 -> load -> q_valid
    // failed clk_x1 by 0.215 ns in a Template DDR3 build); both next values are computed for a load and
    // for none from registers, and the late load only selects
    reg [AW:0]   inmem = 0;
    reg          ne = 1'b0;
    assign count = inmem + (q_valid ? 1'b1 : 1'b0);
    assign full  = count >= (1 << AW);
    wire load = ne && (!q_valid || pop);
    // the candidates are kept as nets of their own (syn_keep, the comment form of the board top's spi_sck_clk), so
    // synthesis cannot fold the load into their adders
    // (it did: the first build of this failed by 0.485 ns on load -> inmem's carry chain)
    wire [AW:0] n0 /* synthesis syn_keep = 1 */ = inmem + (we ? 1'b1 : 1'b0);           // no load
    wire [AW:0] n1 /* synthesis syn_keep = 1 */ = inmem + (we ? 1'b1 : 1'b0) - 1'b1;    // a load
    wire        ne0 /* synthesis syn_keep = 1 */ = we || ne;                            // n0 != 0
    wire        ne1 /* synthesis syn_keep = 1 */ = (inmem > 1) || (ne && we);           // n1 != 0
    always @(posedge clk) begin
        if (we || WALL != 0) mem[wp[AW-1:0]] <= wd[MW-1:0];
        if (load) qm <= mem[rp[AW-1:0]];
        if (rst) begin wp <= 0; rp <= 0; inmem <= 0; ne <= 1'b0; q_valid <= 1'b0; end
        else begin
            if (we) wp <= wp + 1'b1;
            if (load) begin rp <= rp + 1'b1; q_valid <= 1'b1; end
            else if (pop) q_valid <= 1'b0;
            inmem <= load ? n1 : n0;
            ne    <= load ? ne1 : ne0;
        end
    end
    generate if (FW == 0) begin : g_q
        assign q = qm;
    end else begin : g_qf
        (* syn_ramstyle = "registers" *) reg [FW-1:0] memf [0:(1<<AW)-1];
        (* syn_preserve = 1 *) reg [FW-1:0] qf = {FW{1'b0}};
        always @(posedge clk) begin
            if (we || WALL != 0) memf[wp[AW-1:0]] <= wd[W-1:MW];
            if (load) qf <= memf[rp[AW-1:0]];
        end
        assign q = {qf, qm};
    end endgenerate
    // WALL: the entry at wp never holds a word (inmem < 2^AW).  Simulation only (translate_off, as tango_ddr3_arb.v's
    // contract checks).
    /* synthesis translate_off */
    always @(posedge clk) if (WALL != 0 && !rst && inmem >= (1 << AW)) begin
        $display("TANGO CONTRACT: DDR3 fifo entry at the write pointer holds a word at %0t (%m): inmem %0d", $time, inmem);
        $fatal(1, "TANGO CONTRACT: DDR3 fifo WALL violated (see the line above)");
    end
    /* synthesis translate_on */
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
// out_valid is a register too (= n != 0, from the same next count as in_ready), so the consumer's grant starts at a
// register instead of a decode of n.
// The entries' write enables (timing): push && wp was one net enabling all W bits of an entry
// (313 loads in the arbiter's queues).  The entries are written in lanes of 32 bits; lane k's enables read lane k's
// own copy of the write pointer (wp_l[k], syn_preserve: GowinSynthesis merges identical LUTs),
// so no two lanes share an enable net.  Every copy is toggled by the same push as wp and equals it every clock
// (checked in simulation); what each entry holds is unchanged.
// The head's select (timing): rp ? e1 : e0 was one net selecting all W bits
// (rp: 314 loads in the V queue, ahead of the arbiter's output register).  Lane k of out_data selects with lane k's
// own copy of the read pointer (rp_l[k], syn_preserve), toggled by the same pop as rp, so it equals rp every clock
// (checked in simulation) and out_data is unchanged.
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
    output reg           in_ready = 1'b1,         // to the producer only (the queue's own push uses in_ready_w)
    output wire          out_valid,
    output wire [W-1:0]  out_data,
    output reg           out_wb = 1'b0,           // = out_data[WB] when out_valid, a register
    input  wire          out_pop
);
    localparam integer NL = (W + 31) / 32;         // write lanes: lane k is bits [32k +: 32] (the last may be shorter)
    reg [W-1:0] e0 = {W{1'b0}}, e1 = {W{1'b0}};
    reg         rp = 1'b0;
    (* syn_preserve = 1 *) reg [NL-1:0] rp_l = {NL{1'b0}};   // the read pointer, one copy per lane (selects out_data)
    (* syn_preserve = 1 *) reg [NL-1:0] wp_l = {NL{1'b0}};   // the write pointer, one copy per lane
    reg [1:0]   n = 2'd0;
    reg         ov = 1'b0;                          // = n != 0, a register
    // in_ready's copy for the queue's own push (the lanes' enables, the count): the producer's in_ready is a net to
    // the producer alone (timing); both equal every clock (checked in simulation)
    (* syn_preserve = 1 *) reg in_ready_w = 1'b1;
    wire        push = in_valid && in_ready_w;
    wire [2:0]  nn = {1'b0, n} + (push ? 3'd1 : 3'd0) - (out_pop ? 3'd1 : 3'd0);
    assign out_valid = ov;
    genvar gl;
    generate for (gl = 0; gl < NL; gl = gl + 1) begin : g_lane
        localparam integer LO = 32 * gl;
        localparam integer LW = (W - LO < 32) ? (W - LO) : 32;
        always @(posedge clk) begin
            if (in_valid && in_ready_w && !wp_l[gl]) e0[LO +: LW] <= in_data[LO +: LW];
            if (in_valid && in_ready_w &&  wp_l[gl]) e1[LO +: LW] <= in_data[LO +: LW];
            if (rst) wp_l[gl] <= 1'b0;
            else if (in_valid && in_ready_w) wp_l[gl] <= ~wp_l[gl];
            if (rst) rp_l[gl] <= 1'b0;
            else if (out_pop) rp_l[gl] <= ~rp_l[gl];
        end
        assign out_data[LO +: LW] = rp_l[gl] ? e1[LO +: LW] : e0[LO +: LW];
    end endgenerate
    reg out_wo = 1'b0;                              // the second word's flag (n = 2), set when it is pushed
    wire wb_pop  /* synthesis syn_keep = 1 */ = (n == 2'd2) ? out_wo : in_data[WB];   // after a pop
    wire wb_stay /* synthesis syn_keep = 1 */ = (n == 2'd0) ? in_data[WB] : out_wb;   // after no pop
    always @(posedge clk) begin
        out_wb <= out_pop ? wb_pop : wb_stay;
        if (n == 2'd1) out_wo <= in_data[WB];   // the second word's push (else a value nobody reads: out_wo is read only with n = 2, set from n = 1)
        if (rst) begin rp <= 1'b0; n <= 2'd0; ov <= 1'b0; in_ready <= 1'b1; in_ready_w <= 1'b1; end
        else begin
            if (out_pop) rp <= ~rp;
            n <= nn[1:0];
            ov <= (nn[1:0] != 2'd0);
            in_ready <= (nn <= 3'd1);
            in_ready_w <= (nn <= 3'd1);
        end
    end
    // The registered flag equals the entry the head shows, every clock the queue holds a word; out_valid equals n != 0,
    // the lanes' write pointers are equal and the lanes' read pointers equal rp, every clock.  Simulation only (translate_off, as the arbiter's contract
    // checks).
    /* synthesis translate_off */
    integer wbc_clocks = 0;
    always @(posedge clk) begin
        if (out_valid && out_wb !== out_data[WB]) begin
            $display("TANGO CONTRACT: DDR3 skid out_wb differs at %0t (%m): out_wb %b, out_data[WB] %b", $time, out_wb, out_data[WB]);
            $fatal(1, "TANGO CONTRACT: DDR3 skid out_wb violated (see the line above)");
        end
        if (ov !== (n != 2'd0) || wp_l !== {NL{wp_l[0]}} || in_ready_w !== in_ready || rp_l !== {NL{rp}}) begin
            $display("TANGO CONTRACT: DDR3 skid copies differ at %0t (%m): ov %b n %0d wp_l %b in_ready %b/%b rp %b rp_l %b", $time, ov, n, wp_l, in_ready, in_ready_w, rp, rp_l);
            $fatal(1, "TANGO CONTRACT: DDR3 skid out_valid / pointer copies violated (see the line above)");
        end
        wbc_clocks <= wbc_clocks + 1;
    end
    final
        $display("TANGO CONTRACT OK: DDR3 skid out_wb equal to out_data[WB], out_valid and %0d write- and read-pointer copies in %0d clocks (%m)", NL, wbc_clocks);
    /* synthesis translate_on */
endmodule
`default_nettype wire
