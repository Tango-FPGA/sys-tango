// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_tester -- the DDR3 bring-up test's measurement engine on the Gowin DDR3 IP's user interface.
// Clocked by the IP's clk_out (100 MHz).  It loops forever over these phases, each refreshing its results:
//   ORDER  byte-mask polarity and map (32 masked writes over zeroed words) and the burst order of reads at column
//          offsets 0/1/2/4 of a word whose eight 32-bit lanes are 0xA0+j in every byte;
//   FIFO   commands taken back to back (one bank, a new row each: slower to serve than to take) until cmd_ready
//          (or wr_data_rdy) first falls, and until it is first low for 8 clocks; writes then reads, 4 times each;
//   STALL  2^STALL_LOG2 clocks of saturating sequential reads (the longest gap between returns, gaps >= 16 clocks,
//          the longest cmd_ready-low run), then of writes (the longest not-ready run, runs >= 16);
//   LAT    2^NLAT_LOG2 samples each of: single sequential reads, single random reads (1 GB), single row-miss reads
//          (bank 5, random rows), bursts of 8 sequential reads, bursts of 8 random, and single random reads issued
//          every >= 32 clocks under a saturating write stream ("loaded").  Command-to-data latency in wall clocks:
//          min / max / sum and a 16-bin histogram per run, and the maximum since configuration;
//   BW     2^BW_LOG2 commands each: sequential write, sequential read, random write, random read, mixed (write and
//          read alternating) -- wall clocks for each;
//   MARCH  a data-line walk (64 words: 1 << k and its complement in every lane), then over 2^WORDS_LOG2 words of
//          32 B (25 = the whole 1 GB): up w(P); up r(P) w(~P); down r(~P) w(P); up r(P).  P(w) puts the word index
//          in every lane ({w, lane, 4'hA}) xor a per-pass LFSR seed rotated per lane, so every address line and
//          every data bit is exercised in both polarities; the IP's in-order execution (IPUG281-2.6E 4.4.5) lets
//          each r/w pair go back to back.  Every word read is checked: the error count, the first error's byte
//          address, its DQ fold (bit b = some lane's bit b wrong) and byte map (bit 4j+y = lane j byte y wrong),
//          the same ORed over every error, passes and passes with errors.
// Always, over every read and every wanted command since reset: the longest command-to-data latency and the longest
// runs of cmd_ready / wr_data_rdy low (wall clocks).  wall is a free-running 100 MHz count from an ungated PLL
// output (pll_ddr3 CLKOUT0), so the times hold even if the IP stops memory_clk (pll_stop) and with it clk.
// The results: 102 slots (tango_ddr3_test_core adds 12 of its own in front).  40 are registers
// (counters, the probes' results); the latency statistics are in block RAM:
//   hs  (tango_ddr3_rmw, a read-modify-write pipeline with forwarding): the running latency run's histogram, and per
//       mode {samples >= 1024 clocks, the maximum} since configuration;
//   cm  the committed results, a slot a word: at a run's end the commit writes the run's {max, min} and sum, the
//       bursts', the since-configuration word merged with the run's, and the histogram (read out of hs and cleared).
// The copier writes snapshots into two block RAMs that tango_ddr3_test_core reads in other clocks:
//   the display RAM, two banks: the display shows one and the copier writes the other (d_done when written); the
//     display takes it at its next frame (d_shown), and only then is the old one written again;
//   the UART RAM, two banks: before each line the UART asks for a copy in the bank it does not read (u_req, u_bank)
//     and reads that bank once the copy is answered (u_ack).
// A pass (every 2^SNAP_LOG2 clocks once the display has taken the last bank, and on each UART request) captures the
// 40 register slots in one clock, then writes slot after slot: those, and the 62 committed ones from cm.  The copier
// and the commit never run together, so a snapshot holds whole runs; no reader ever reads a bank being written.
// After configuration a clear pass writes every RAM (GW5A block RAM has no usable initial values on
// silicon); the tester starts once it is done.
// Structure (timing at 100 MHz; the first build closed at 52 MHz):
//   - the sequencer issues run descriptors; an expander makes one request a clock from them into a 2-entry queue
//     with a registered ready; the queue's head goes to the IP (cmd_en = head valid && cmd_ready && (write ?
//     wr_data_rdy : a read slot)).  Everything that measures the IP (accept counts and times, not-ready runs, read
//     tags) is taken at the queue's head, where the IP takes the command, a clock later from registered events;
//   - the read-slot limit is a register computed a clock ahead; the sequencer waits on registered drain flags;
//   - returned words go through 5 register stages (tag, compare, reduce, account, histogram);
//   - the resets are local registered copies of rst.
//==============================================================================
`default_nettype none
// Shift chains (the return pipeline) as registers: GowinSynthesis otherwise maps them to SSRAM, which GW5A lacks
// (RP0007).
(* syn_srlstyle = "registers" *)
module tango_ddr3_tester #(
    parameter integer WORDS_LOG2 = 25,
    parameter integer NLAT_LOG2  = 15,
    parameter integer BW_LOG2    = 20,
    parameter integer STALL_LOG2 = 20,
    parameter integer TAG_LOG2   = 9,
    parameter integer SNAP_LOG2  = 21,
    parameter integer FIFO_MAX   = 1000,
    parameter integer DRAIN_CLKS = 2048
) (
    input  wire          clk,
    input  wire          rst,
    input  wire [23:0]   wall,
    input  wire          cmd_ready,
    output wire          cmd_en,
    output wire [2:0]    cmd,
    output wire [28:0]   addr,
    input  wire          wr_data_rdy,
    output wire          wr_data_en,
    output wire [255:0]  wr_data,
    output wire [31:0]   wr_data_mask,
    input  wire          rd_data_valid,
    input  wire [255:0]  rd_data,
    // the snapshot RAMs' write ports (the RAMs are in tango_ddr3_test_core), and their handshakes
    output reg           dm_we = 1'b0,            // the display RAM: write wd at dm_wa = {bank, slot}
    output reg  [7:0]    dm_wa = 8'd0,
    output reg           um_we = 1'b0,            // the UART RAM: write wd at um_wa = {bank, slot}
    output reg  [7:0]    um_wa = 8'd0,
    output reg  [31:0]   wd = 32'd0,
    input  wire          d_shown,                 // the display RAM's bank on screen (synchronized)
    output reg           d_done = 1'b0,           // toggles when the other bank is written
    input  wire          u_req,                   // toggles: the UART asks for a copy (synchronized) ...
    input  wire          u_bank,                  // ... into this bank of the UART RAM (synchronized; set first)
    output reg           u_ack = 1'b0             // = u_req once the copy is written
);
    localparam integer TN = 1 << TAG_LOG2;
    localparam [3:0] PH_IDLE = 4'd0, PH_ORDER = 4'd1, PH_FIFO = 4'd2, PH_STALL = 4'd3, PH_LAT = 4'd4,
                     PH_BW = 4'd5, PH_MARCH = 4'd6;
    localparam [3:0] K_NONE = 4'd0, K_LAT = 4'd1, K_M0 = 4'd2, K_M1 = 4'd3, K_WALK = 4'd4, K_ORDER = 4'd5,
                     K_STALL = 4'd6;
    localparam [31:0] NLAT = 32'd1 << NLAT_LOG2;
    localparam [31:0] NBW  = 32'd1 << BW_LOG2;
    localparam [31:0] NSTW = 32'd1 << STALL_LOG2;
    localparam [31:0] NWRD = 32'd1 << WORDS_LOG2;

    // ---- local registered resets (timing: rst comes from tango_ddr3_test_core; each copy drives one part)
    (* syn_preserve = 1 *) reg rst_s = 1'b1;       // the sequencer
    (* syn_preserve = 1 *) reg rst_e = 1'b1;       // the expander, the queue, the drain counts
    always @(posedge clk) begin rst_s <= rst; rst_e <= rst; end

    // ---- wall time, 32 bits (the 24-bit input plus a wrap count), registered (a uniform one-clock delay)
    reg  [7:0]  wall_hi = 8'd0;
    reg         wall23_q = 1'b0;
    reg  [31:0] now = 32'd0;
    wire [7:0]  wall_hi_n = wall_hi + ((wall23_q && !wall[23]) ? 8'd1 : 8'd0);
    always @(posedge clk) begin wall23_q <= wall[23]; wall_hi <= wall_hi_n; now <= {wall_hi_n, wall}; end

    // ---- patterns
    function [255:0] pat(input [24:0] w, input inv, input [31:0] seed);
        integer j; reg [31:0] r;
        begin
            for (j = 0; j < 8; j = j + 1) begin
                r = (seed << (4 * j)) | (seed >> (32 - 4 * j));
                pat[32 * j +: 32] = {w, j[2:0], 4'hA} ^ r ^ {32{inv}};
            end
        end
    endfunction
    function [255:0] walkpat(input [5:0] k);
        reg [31:0] v;
        begin
            v = 32'h1 << k[4:0];
            if (k[5]) v = ~v;
            walkpat = {8{v}};
        end
    endfunction
    function [255:0] lanepat(input dummy);
        integer j;
        begin
            for (j = 0; j < 8; j = j + 1) lanepat[32 * j +: 32] = {4{4'hA, 1'b0, j[2:0]}};
        end
    endfunction
    function [3:0] bin16(input [15:0] l);
        begin
            if      (l < 16'd16)  bin16 = 4'd0;
            else if (l < 16'd20)  bin16 = 4'd1;
            else if (l < 16'd24)  bin16 = 4'd2;
            else if (l < 16'd28)  bin16 = 4'd3;
            else if (l < 16'd32)  bin16 = 4'd4;
            else if (l < 16'd40)  bin16 = 4'd5;
            else if (l < 16'd48)  bin16 = 4'd6;
            else if (l < 16'd64)  bin16 = 4'd7;
            else if (l < 16'd80)  bin16 = 4'd8;
            else if (l < 16'd96)  bin16 = 4'd9;
            else if (l < 16'd128) bin16 = 4'd10;
            else if (l < 16'd192) bin16 = 4'd11;
            else if (l < 16'd256) bin16 = 4'd12;
            else if (l < 16'd512) bin16 = 4'd13;
            else if (l < 16'd1024) bin16 = 4'd14;
            else                  bin16 = 4'd15;
        end
    endfunction

    // ---- the request: from the expander (below), which turns the sequencer's run descriptors into requests, one a
    // clock.  A write carries its data as a descriptor (kind, value; mask: every byte but byte value[4:0] masked),
    // expanded when it enters the queue.
    localparam [2:0] D_ZERO = 3'd0, D_ONES = 3'd1, D_LANE = 3'd2, D_REP = 3'd3, D_WALK = 3'd4, D_PAT0 = 3'd5,
                     D_PAT1 = 3'd6;
    localparam [1:0] OP_W = 2'd0, OP_R = 2'd1, OP_RW = 2'd2, OP_WR2 = 2'd3;   // write; read; read then write the same
                                                                            // address; write at a then read at b
    // the run being expanded (d_*): op, addresses and their step, the count of steps left, data kind / value / step,
    // the mask kind, the read tag's kind / x / step, burst tagging (x = {last, first, mode}), random addresses
    reg          d_v = 1'b0, d_sub = 1'b0, d_first = 1'b1;
    reg  [1:0]   d_op = OP_W;
    reg  [28:0]  d_a = 29'd0, d_b = 29'd0, d_as = 29'd0;
    reg          d_rand = 1'b0;
    reg  [31:0]  d_cnt = 32'd0;
    reg  [2:0]   d_dk = D_ZERO;
    reg  [31:0]  d_dv = 32'd0, d_ds = 32'd0;
    reg          d_mk = 1'b0;
    reg  [3:0]   d_k = K_NONE;
    reg  [24:0]  d_x = 25'd0, d_xs = 25'd0;
    reg          d_xb = 1'b0;
    reg  [2:0]   d_xm = 3'd0;
    reg  [31:0]  lfsr = 32'hACE1_2468;
    wire [31:0]  lfsr_n = {lfsr[30:0], 1'b0} ^ (lfsr[31] ? 32'h0040_0007 : 32'd0);
    wire [28:0]  a_rand_core = {1'b0, 2'b11, lfsr[22:0], 3'b000};
    wire         rq    = d_v;
    wire         rq_w  = (d_op == OP_W) || (d_op == OP_RW && d_sub) || (d_op == OP_WR2 && !d_sub);
    wire [28:0]  rq_a  = d_rand ? a_rand_core : (d_op == OP_WR2 && d_sub) ? d_b : d_a;
    wire [2:0]   rq_dk = d_dk;
    wire [31:0]  rq_dv = d_dv;
    wire         rq_mk = d_mk;
    wire [3:0]   rq_k  = rq_w ? K_NONE : d_k;
    wire [24:0]  rq_x  = d_xb ? {20'd0, (d_cnt == 32'd1), d_first, d_xm} : d_x;
    reg          sq_room = 1'b1;
    wire         push = rq && sq_room;
    reg  [31:0]  seed = 32'h1234_5679;             // the march pattern's seed (changes only between passes)

    // ---- the queue (2 entries) and the IP side: {write, addr, kind, x}; the write data expanded beside it
    localparam integer QW = 1 + 29 + 4 + 25;
    reg  [QW-1:0] e0 = {QW{1'b0}}, e1 = {QW{1'b0}};
    reg           sq_rp = 1'b0;
    reg  [1:0]    sq_n = 2'd0;
    wire [QW-1:0] hd   = sq_rp ? e1 : e0;
    wire          h_v  = (sq_n != 2'd0);
    wire          h_w  = hd[QW-1];
    wire [28:0]   h_a  = hd[QW-2 -: 29];
    wire [3:0]    h_k  = hd[28:25];
    wire [24:0]   h_x  = hd[24:0];
    reg           can_rd_q = 1'b0;                    // a read may go now (outstanding below the limit)
    wire          ip_fire  = h_v && cmd_ready && (h_w ? wr_data_rdy : can_rd_q);
    wire          ip_firer = ip_fire && !h_w;
    assign cmd_en       = ip_fire;
    assign cmd          = h_w ? 3'b000 : 3'b001;
    assign addr         = h_a;
    assign wr_data_en   = ip_fire && h_w;
    // each entry's data and mask, expanded from the request's descriptor (from the d_* registers), so the IP's write
    // data is a 2-way select of registers (timing: the IP's write buffer input)
    wire [255:0] rq_wd = (rq_dk == D_ONES) ? {256{1'b1}} : (rq_dk == D_LANE) ? lanepat(1'b0) :
                         (rq_dk == D_REP) ? {8{rq_dv}} : (rq_dk == D_WALK) ? walkpat(rq_dv[5:0]) :
                         (rq_dk == D_PAT0) ? pat(rq_dv[24:0], 1'b0, seed) : (rq_dk == D_PAT1) ? pat(rq_dv[24:0], 1'b1, seed) :
                         256'd0;
    wire [31:0]  rq_wm = rq_mk ? ~(32'h1 << rq_dv[4:0]) : 32'd0;
    // The free entry is written every clock with the request on offer (an empty queue: the head's place; one entry:
    // the other; full: neither), so a push writes its entry without an enable of its own.  The entries' write enables
    // are registered from the queue's next state, one copy per 32-bit lane and one for the descriptors (timing: the
    // data registers lie along the IP's write-data inputs, ~17 rows; an earlier build failed on a push enable of
    // fanout 348 merged into their data logic).
    reg  [1:0]    rd_inq = 2'd0;                      // reads in the queue
    wire [2:0]    sq_n_nx  = {1'b0, sq_n} + (push ? 3'd1 : 3'd0) - (ip_fire ? 3'd1 : 3'd0);
    wire          sq_rp_nx = sq_rp ^ ip_fire;
    wire          en0_nx = (sq_n_nx == 3'd0 && !sq_rp_nx) || (sq_n_nx == 3'd1 && sq_rp_nx);
    wire          en1_nx = (sq_n_nx == 3'd0 && sq_rp_nx) || (sq_n_nx == 3'd1 && !sq_rp_nx);
    (* syn_preserve = 1 *) reg [8:0] en0 = 9'h1FF;    // copies: lanes 0..7, the descriptors
    (* syn_preserve = 1 *) reg [8:0] en1 = 9'h000;
    reg  [255:0] x0 = 256'd0, x1 = 256'd0;
    reg  [31:0]  xm0 = 32'd0, xm1 = 32'd0;
    always @(posedge clk) begin : qdata                   // (no reset: an entry is valid only after its push)
        integer j;
        for (j = 0; j < 8; j = j + 1) begin
            if (en0[j]) begin x0[32 * j +: 32] <= rq_wd[32 * j +: 32]; xm0[4 * j +: 4] <= rq_wm[4 * j +: 4]; end
            if (en1[j]) begin x1[32 * j +: 32] <= rq_wd[32 * j +: 32]; xm1[4 * j +: 4] <= rq_wm[4 * j +: 4]; end
        end
        if (en0[8]) e0 <= {rq_w, rq_a, rq_k, rq_x};
        if (en1[8]) e1 <= {rq_w, rq_a, rq_k, rq_x};
    end
    assign wr_data      = sq_rp ? x1 : x0;
    assign wr_data_mask = sq_rp ? xm1 : xm0;
    always @(posedge clk) begin
        if (rst_e) begin
            sq_rp <= 1'b0; sq_n <= 2'd0; sq_room <= 1'b1; rd_inq <= 2'd0; en0 <= 9'h1FF; en1 <= 9'h000;
        end else begin
            sq_rp   <= sq_rp_nx;
            sq_n    <= sq_n_nx[1:0];
            sq_room <= (sq_n_nx <= 3'd1);
            en0 <= {9{en0_nx}}; en1 <= {9{en1_nx}};
            rd_inq  <= rd_inq + ((push && !rq_w) ? 2'd1 : 2'd0) - (ip_firer ? 2'd1 : 2'd0);
        end
    end

    // ---- read tags, in issue order: {issue wall[23:0], kind, x}; outstanding reads and the read-slot limit
    reg  [TAG_LOG2:0] max_out = 1;
    reg  [TAG_LOG2:0] tag_wr = 0, tag_rd = 0;
    wire [TAG_LOG2:0] outst = tag_wr - tag_rd;
    wire [TAG_LOG2:0] outst_p1 = outst + 1'b1, outst_m1 = outst - 1'b1;
    wire ok_0 = (outst < max_out) && (outst < TN - 2);
    wire ok_p = (outst_p1 < max_out) && (outst_p1 < TN - 2);
    wire ok_m = (outst_m1 < max_out) && (outst_m1 < TN - 2);
    always @(posedge clk)
        can_rd_q <= (ip_firer && !rd_data_valid) ? ok_p : (!ip_firer && rd_data_valid) ? ok_m : ok_0;
    (* syn_ramstyle = "block_ram" *) reg [52:0] tagm [0:TN-1];   // (GowinSynthesis took it for SSRAM, which GW5A lacks: RP0007)
    reg  [52:0] tag_q = 53'd0;
    always @(posedge clk) begin
        if (ip_firer) tagm[tag_wr[TAG_LOG2-1:0]] <= {now[23:0], h_k, h_x};
        tag_q <= tagm[tag_rd[TAG_LOG2-1:0]];
    end
    // the IP side's events, registered: the bookkeeping that measures the IP runs a clock later, with the time of
    // the event (so a measure is exact, and nothing but the handshake depends on cmd_ready in the same clock)
    reg          ipf_q = 1'b0, ipfr_q = 1'b0, iplat_q = 1'b0, hv_q = 1'b0, hw_q = 1'b0, cr_q = 1'b0, wr_q = 1'b0;
    reg  [31:0]  ipnow_q = 32'd0;
    always @(posedge clk) begin
        ipf_q <= ip_fire; ipfr_q <= ip_firer; iplat_q <= ip_firer && (h_k == K_LAT) && h_x[3];
        hv_q <= h_v; hw_q <= h_w; cr_q <= cmd_ready; wr_q <= wr_data_rdy; ipnow_q <= now;
    end
    // reads pushed and not yet back (queue + IP), and whether there are none, as of this clock (from the next state)
    reg  [TAG_LOG2+1:0] rd_all = 0;
    wire [TAG_LOG2+1:0] rd_all_n = rd_all + ((push && !rq_w) ? 1'b1 : 1'b0) - (rd_data_valid ? 1'b1 : 1'b0);
    reg          rd_none_q = 1'b1;
    always @(posedge clk) begin
        if (rst_e) begin rd_all <= 0; rd_none_q <= 1'b1; end
        else begin rd_all <= rd_all_n; rd_none_q <= (rd_all == 0) && !(rq && !rq_w); end   // conservative, registers
    end

    // ---- return pipeline: s1 the word (and its tag read), s2 the tag, s3 compare, s4 reduce, s5 account / the
    // histogram's bin, then the statistics RAM
    reg          rv1 = 1'b0, rv2 = 1'b0, rv3 = 1'b0, rv4 = 1'b0;
    reg  [255:0] rd1 = 256'd0, rd2 = 256'd0, rd3 = 256'd0, xr3 = 256'd0;
    reg  [31:0]  rw1 = 32'd0, rw2 = 32'd0, rw3 = 32'd0, rw4 = 32'd0;
    reg  [52:0]  tg2 = 53'd0;
    reg  [3:0]   k3 = K_NONE, k4 = K_NONE;
    reg  [24:0]  x3 = 25'd0, x4 = 25'd0;
    reg  [23:0]  lat3 = 24'd0, lat4 = 24'd0;
    reg  [15:0]  l16_4 = 16'd0;
    reg          chk3 = 1'b0, chk4 = 1'b0, any4 = 1'b0;
    reg  [31:0]  dq4 = 32'd0, ln4 = 32'd0, sff4 = 32'd0, nm4 = 32'd0;
    reg          oka4 = 1'b0, okb4 = 1'b0;
    reg          ow4 = 1'b0, os04 = 1'b0;               // s4: an ORDER mask read (x < 32), the first one
    reg  [31:0]  ob4 = 32'd0;                           // s4: its bit (1 << x)
    reg  [3:0]   oo4 = 4'd0;                            // s4: an ORDER lane read at +0 / +1 / +2 / +4
    reg          hv5 = 1'b0;                            // s5: a latency sample for the histogram, its bin
    reg  [3:0]   hb5 = 4'd0;
    reg  [15:0]  lb4 = 16'd0, gap4 = 16'd0;             // s4: a burst's time to its last word; a read-stream gap
    reg          gap4_v = 1'b0, gap4_16 = 1'b0, have3 = 1'b0;
    reg  [31:0]  last3 = 32'd0;
    wire [3:0]   k2 = tg2[28:25];
    wire [24:0]  x2 = tg2[24:0];
    wire [255:0] exp2 = (k2 == K_M0) ? pat(x2, 1'b0, seed) : (k2 == K_M1) ? pat(x2, 1'b1, seed) : walkpat(x2[5:0]);
    wire         pipe_busy = rv1 | rv2 | rv3 | rv4 | hv5;

    // ---- the sequencer's state: one flop per step (one-hot by construction: GowinSynthesis extracts state machines
    // only in its two-process style and kept the earlier 6-bit step register binary, a bit of it fanning out to ~300
    // loads), stepped with parallel_case; runs go out as descriptors (nd_*: staged, then the expander's d_*), and
    // results are kept by the results block on the sequencer's registered events, so no step decodes into a result
    // register (timing at 100 MHz)
    reg  [3:0]   ph = PH_IDLE;                    // the phase (for the screen)
    localparam integer SS_IDLE = 0,
        SS_O0 = 1, SS_O0W = 2, SS_O1 = 3, SS_O1W = 4, SS_O2 = 5, SS_O2W = 6, SS_O3 = 7, SS_O3W = 8, SS_O3L = 9,
        SS_O3LW = 10, SS_O4 = 11,
        SS_F0 = 12, SS_F0R = 13, SS_F1 = 14,
        SS_T0 = 15, SS_T0R = 16, SS_T1 = 17, SS_T2 = 18, SS_T2R = 19, SS_T3 = 20,
        SS_L0 = 21, SS_L1 = 22, SS_L2 = 23, SS_L5 = 24, SS_L3 = 25, SS_L4 = 26,
        SS_B0 = 27, SS_B0W = 28, SS_B1 = 29,
        SS_M0 = 30, SS_M0W = 31, SS_M1 = 32, SS_M1W = 33, SS_M2 = 34, SS_M2W = 35, SS_M3 = 36, SS_M3W = 37,
        SS_M4 = 38, SS_M4W = 39, SS_M5 = 40, SS_M5W = 41, SS_M6 = 42;
    localparam integer NSS = 43;
    function [NSS-1:0] S1(input integer k);       // the step vector of step k
        S1 = {{(NSS-1){1'b0}}, 1'b1} << k;
    endfunction
    reg  [NSS-1:0] st = {{(NSS-1){1'b0}}, 1'b1};
    // the step's number, for the benches' progress lines (no load in synthesis)
    reg  [5:0]   ss;
    always @* begin : g_ss
        integer k;
        ss = 6'd63;
        for (k = 0; k < NSS; k = k + 1) if (st[k]) ss = k[5:0];
    end
    // the staged descriptor
    reg          nd_v = 1'b0;
    reg  [1:0]   nd_op = OP_W;
    reg  [28:0]  nd_a = 29'd0, nd_b = 29'd0, nd_as = 29'd0;
    reg          nd_rand = 1'b0;
    reg  [31:0]  nd_cnt = 32'd0;
    reg  [2:0]   nd_dk = D_ZERO;
    reg  [31:0]  nd_dv = 32'd0, nd_ds = 32'd0;
    reg          nd_mk = 1'b0;
    reg  [3:0]   nd_k = K_NONE;
    reg  [24:0]  nd_x = 25'd0, nd_xs = 25'd0;
    reg          nd_xb = 1'b0;
    reg  [2:0]   nd_xm = 3'd0;
    reg          d_abort = 1'b0;                  // a pulse: the expander drops the rest of its run
    reg          stage = 1'b0;                    // a pulse: issue() staged a run (nd_v rises a clock later)
    // no run being staged, staged or expanded, registered: a clock late, so the steps that wait on it also check
    // !stage (stage is high in the clock after an issue; from then until the run ends, stage, nd_v or d_v is high,
    // so ex_idle_q is low from the clock after that; timing: nd_v -> the step register)
    reg          ex_idle_q = 1'b1;
    always @(posedge clk) ex_idle_q <= !stage && !nd_v && !d_v;
    // the sequencer's events for the results block (one-clock pulses) and the data they take (registered every clock)
    reg          ev_ord = 1'b0, fd_ev = 1'b0, fa_ev = 1'b0, ev_t1 = 1'b0, ev_t3 = 1'b0, ev_bw = 1'b0, ev_m6 = 1'b0;
    reg          fd_frd = 1'b0, fa_frd = 1'b0, fa_seen = 1'b0, fa_satw = 1'b0, fa_satr = 1'b0;
    reg  [15:0]  fd_fn = 16'd0, fa_fn = 16'd0;
    reg  [31:0]  fd_t = 32'd0, fd_ret = 32'd0, bw_d = 32'd0;
    reg  [1:0]   fd_which = 2'd0;
    reg  [2:0]   ev_bwn = 3'd0;
    reg          in_f1 = 1'b0, in_t0r = 1'b0, in_t2r = 1'b0;      // the step, a clock ago
    reg          hv_q2 = 1'b0, cr_q2 = 1'b0, wr_q2 = 1'b0, ipf_q2 = 1'b0;
    // the sequencer's compares, registered (each read only after its inputs have held for a clock, or set where
    // they change; see the steps)
    reg          n_last = 1'b0, q_done = 1'b0, q_lt_n = 1'b0, wc_f1 = 1'b0, wc_win = 1'b0, wc_t3 = 1'b0;
    reg          fn_nz = 1'b0, fn_full = 1'b0, okA_all = 1'b0, okB_all = 1'b0, perr = 1'b0;
    reg  [21:0]  wn = 22'd0;                      // LAT loaded: the write runs' next word
    reg  [31:0]  n = 32'd0, wc = 32'd0, q = 32'd0;
    reg  [2:0]   mode = 3'd0, bwn = 3'd0;
    reg  [1:0]   rep = 2'd0;
    reg          frd = 1'b0;                      // FIFO: 0 = the write runs, 1 = the read runs
    reg  [2:0]   oj = 3'd0;                       // ORDER: the lane-word reads (+0, +1, +2, +4)
    reg  [31:0]  t0 = 32'd0, burst_t0 = 32'd0;
    reg          bw_arm = 1'b0;
    reg          stat_clr = 1'b0, stat_store = 1'b0;      // one-clock pulses from the sequencer (LAT)
    reg  [2:0]   stat_mode = 3'd0;                        // the mode they are for
    reg          st_new = 1'b0;                           // a pulse: a stall read window starts (s5 clears its counts)
    // everything drained (nothing queued, being built or staged, no read out, the return pipeline empty), from
    // registers: idle_q at clock t+1 = the drain at t+1 (rv4 stands for the stage behind it)
    reg          idle_q = 1'b1;
    always @(posedge clk)
        idle_q <= (sq_n == 2'd0) && !rq && !stage && !nd_v && (rd_all == 0) && !rd_data_valid && !rv1 && !rv2 && !rv3 && !rv4;

    // results (registers)
    reg  [15:0]  loopn = 16'd0;
    reg  [31:0]  loops = 0, passes = 0, bad_passes = 0, err_words = 0, first_addr = 32'hFFFF_FFFF;
    reg  [31:0]  first_dq = 0, first_lanes = 0, cum_dq = 0, cum_lanes = 0, pass_err0 = 0;
    reg  [47:0]  words_checked = 0;
    reg  [31:0]  cum_rdlat = 0, cum_cmdlow = 0, cum_wrdylow = 0;
    reg          any_err = 1'b0, first_set = 1'b0;
    reg  [31:0]  okA = 0, okB = 0, s0 = 0, ord_off0 = 0, ord_off1 = 0, ord_off2 = 0, ord_off4 = 0;
    reg  [31:0]  s0_r = 0, ord_pol = 0, ord_okA = 0, ord_okB = 0;
    reg  [15:0]  fw_first = 0, fw_8 = 0, fr_first = 0, fr_8 = 0, fw_first_max = 0, fw_8_max = 0;
    reg  [15:0]  fr_first_max = 0, fr_8_max = 0;
    reg  [1:0]   fw_which = 2'd0;
    reg          f_seen = 1'b0, f_sat_w = 1'b0, f_sat_r = 1'b0;
    reg  [15:0]  fn = 0;
    reg  [31:0]  f_t0 = 0, fw_t = 0, fr_t = 0, fw_t_max = 0, fr_t_max = 0, f_ret = 0, fr_ret = 0, fr_ret_max = 0;
    reg  [7:0]   f_low = 8'd0;
    reg  [15:0]  st_gap_w = 0, st_gap_c = 0, st_low_w = 0, st_low_c = 0, st_wrun_w = 0, st_wrun_c = 0;
    reg  [31:0]  st_gaps16 = 0, st_rwords = 0, st_wruns16 = 0, st_wwords = 0;
    reg  [31:0]  st_gaps16_n = 0, st_rwords_n = 0, st_wruns16_n = 0, st_wwords_n = 0;
    reg  [15:0]  run = 0;
    reg  [31:0]  bw0 = 0, bw1 = 0, bw2 = 0, bw3 = 0, bw4 = 0;
    // a latency run's statistics while it runs (its histogram: in hs); committed into cm at its end
    reg  [15:0]  c_min = 16'hFFFF, c_max = 0, c_cmax = 0, c_nsat = 0, c_bmin = 16'hFFFF, c_bmax = 0;
    reg  [31:0]  c_sum = 0, c_bsum = 0;
    reg  [31:0]  low_c0 = 0, low_w0 = 0;          // wall when cmd_ready / wr_data_rdy went low with a request
    reg  [31:0]  runc = 0, runw = 0;              // the runs' lengths so far
    reg          low_c = 1'b0, low_wr = 1'b0;

    // ---- addresses (IP addr = {rank 0, bank[2:0], row[14:0], col[9:0]}, a 32-B word = 8 columns)
    function [28:0] wa(input [2:0] bank, input [21:0] word);      // word index within a bank
        wa = {1'b0, bank, word, 3'b000};
    endfunction
    localparam [28:0] A_ORD  = {1'b0, 3'd0, 15'd1, 10'd0};
    localparam [28:0] A_WALK = {1'b0, 3'd0, 15'd2, 10'd0};

    // ================= the return pipeline's data path (no reset: drained by the sequencer) =================
    always @(posedge clk) begin : retpipe
        integer jj, yy, bb, ll;
        reg [31:0] dq, ln, sff, nm;
        // s1
        rv1 <= rd_data_valid; rd1 <= rd_data; rw1 <= now;
        // s2: the tag read in s1 belongs to s1's word
        rv2 <= rv1; rd2 <= rd1; rw2 <= rw1; tg2 <= tag_q;
        // s3: compare with what was written; the latency
        rv3 <= rv2; rd3 <= rd2; rw3 <= rw2; k3 <= k2; x3 <= x2;
        lat3 <= rw2[23:0] - tg2[52:29];
        chk3 <= rv2 && (k2 == K_M0 || k2 == K_M1 || k2 == K_WALK);
        xr3  <= rd2 ^ exp2;
        // s4: reduce
        rv4 <= rv3; k4 <= k3; x4 <= x3; rw4 <= rw3; chk4 <= chk3; lat4 <= lat3;
        l16_4 <= (lat3 > 24'hFFFF) ? 16'hFFFF : lat3[15:0];
        dq = 32'd0; ln = 32'd0;
        for (jj = 0; jj < 8; jj = jj + 1) begin
            dq = dq | xr3[32 * jj +: 32];
            for (yy = 0; yy < 4; yy = yy + 1) ln[4 * jj + yy] = |xr3[32 * jj + 8 * yy +: 8];
        end
        dq4 <= dq; ln4 <= ln; any4 <= |xr3;
        lb4 <= (rw3 - burst_t0 > 32'hFFFF) ? 16'hFFFF : rw3[15:0] - burst_t0[15:0];
        if (rv3 && k3 == K_STALL) begin
            gap4 <= (rw3 - last3 > 32'hFFFF) ? 16'hFFFF : rw3[15:0] - last3[15:0];
            gap4_v <= have3; gap4_16 <= (rw3 - last3 >= 32'd16);
            last3 <= rw3; have3 <= 1'b1;
        end
        if (ph != PH_STALL) have3 <= 1'b0;                    // a new stall window: no previous return
        for (bb = 0; bb < 32; bb = bb + 1) sff[bb] = (rd3[8 * bb +: 8] == 8'hFF);
        for (ll = 0; ll < 8; ll = ll + 1)
            nm[4 * ll +: 4] = {(rd3[32 * ll +: 32] == {4{4'hA, 1'b0, rd3[32 * ll +: 3]}}), rd3[32 * ll +: 3]};
        sff4 <= sff; nm4 <= nm;
        // the ORDER reads' checks and where they go, decoded here (s5 only stores)
        oka4 <= (sff == (32'h1 << x3[4:0])); okb4 <= (sff == ~(32'h1 << x3[4:0]));
        ow4  <= rv3 && (k3 == K_ORDER) && (x3 < 25'd32);
        os04 <= rv3 && (k3 == K_ORDER) && (x3 == 25'd0);
        ob4  <= 32'h1 << x3[4:0];
        oo4  <= {4{rv3 && (k3 == K_ORDER)}} & {(x3 >= 25'd35), (x3 == 25'd34), (x3 == 25'd33), (x3 == 25'd32)};
        // s5: a latency sample's histogram bin (to hs below)
        hv5 <= rv4 && (k4 == K_LAT) && x4[3] && (x4[2:0] != 3'd4);
        hb5 <= bin16(l16_4);
    end

    // ================= a latency run's statistics (registers) and its histogram (hs): updated from s5, cleared on
    // the sequencer's pulse; the commit (below) writes them into cm at the run's end
    always @(posedge clk) begin
        if (stat_clr) begin
            c_min <= 16'hFFFF; c_max <= 16'd0; c_sum <= 32'd0; c_bmin <= 16'hFFFF; c_bmax <= 16'd0; c_bsum <= 32'd0;
            c_cmax <= 16'd0; c_nsat <= 16'd0;
        end else if (rv4 && k4 == K_LAT) begin
            if (x4[3]) begin                               // the sample (a burst's first read)
                if (l16_4 < c_min) c_min <= l16_4;
                if (l16_4 > c_max) c_max <= l16_4;
                if (l16_4 > c_cmax) c_cmax <= l16_4;
                if (l16_4 >= 16'd1024 && c_nsat != 16'hFFFF) c_nsat <= c_nsat + 1'b1;
                c_sum <= c_sum + l16_4;
            end
            if (x4[4] && (x4[2:0] == 3'd3 || x4[2:0] == 3'd4)) begin   // a burst's last read: all 8 back
                if (lb4 < c_bmin) c_bmin <= lb4;
                if (lb4 > c_bmax) c_bmax <= lb4;
                c_bsum <= c_bsum + lb4;
            end
        end
    end

    // ================= the clear pass after configuration: every RAM written once (block RAM has no usable initial
    // values on GW5A); nothing else touches the RAMs, and the sequencer does not start, until it is done
    reg          clr_done = 1'b0;
    reg  [7:0]   ci = 8'd0;
    always @(posedge clk) if (!clr_done) begin ci <= ci + 1'b1; if (ci == 8'd255) clr_done <= 1'b1; end
    // cm's initial words: {max 0, min 0xFFFF} where a run's {max, min} goes, 0 elsewhere (as the registers were)
    function [31:0] cm_init(input [6:0] a);
        cm_init = (a == 7'd37 || a == 7'd40 || a == 7'd43 || a == 7'd46 || a == 7'd49 || a == 7'd52 || a == 7'd55 ||
                   a == 7'd57) ? 32'h0000_FFFF : 32'd0;
    endfunction

    // ================= hs: the running histogram and the since-configuration words (tango_ddr3_rmw) =================
    // words 0..7: the running run's bins {2g+1, 2g}; 8 + m: mode m's {samples >= 1024 clocks, the maximum}.  Its users
    // never overlap: the clear pass, then a run's samples (s5), then (drained) the commit's operations.
    localparam [1:0] RMW_INC = 2'd0, RMW_MRG = 2'd1, RMW_RDCLR = 2'd2, RMW_WR = 2'd3;
    reg          cr_v = 1'b0;                     // the commit's operation (registers)
    reg  [1:0]   cr_op = RMW_INC;
    reg  [4:0]   cr_a = 5'd0;
    reg  [31:0]  cr_d = 32'd0;
    reg  [7:0]   cr_tag = 8'd0;                   // {write the result to cm, its cm address}
    wire         clr_hs = !clr_done && (ci < 8'd32);
    wire         hs_v   = clr_hs | cr_v | hv5;
    wire [1:0]   hs_op  = clr_hs ? RMW_WR : cr_v ? cr_op : RMW_INC;
    wire [4:0]   hs_a   = clr_hs ? ci[4:0] : cr_v ? cr_a : {2'b00, hb5[3:1]};
    wire [31:0]  hs_d   = clr_hs ? 32'd0 : cr_d;
    wire [7:0]   hs_tag = cr_v ? cr_tag : 8'd0;
    wire         hs_ov, hs_busy;
    wire [31:0]  hs_oval;
    wire [7:0]   hs_otag;
    tango_ddr3_rmw u_hs (.clk(clk), .in_v(hs_v), .in_op(hs_op), .in_a(hs_a), .in_hi(hb5[0]), .in_d(hs_d),
                         .in_tag(hs_tag), .out_v(hs_ov), .out_val(hs_oval), .out_tag(hs_otag), .busy(hs_busy));

    // ================= cm: the committed results (slot t at address t) =================
    (* syn_ramstyle = "block_ram" *) reg [31:0] cm [0:127];
    reg          cmw_e = 1'b0;                    // cm's write port (registers)
    reg  [6:0]   cmw_a = 7'd0;
    reg  [31:0]  cmw_d = 32'd0;
    reg  [6:0]   cp_t = 7'd0;                     // the copier's slot (its read address)
    reg  [31:0]  cm_q = 32'd0;
    always @(posedge clk) begin
        if (cmw_e) cm[cmw_a] <= cmw_d;
        cm_q <= cm[cp_t];                         // (the copier and the commit never run together)
    end

    // ================= the commit: a latency run's results into cm (stat_store; the sequencer waits for it) =========
    // steps: 0 {max, min}, 1 sum, 2 / 3 the bursts' {max, min} and sum (modes 3, 4), 4 the since-configuration word
    // merged in hs (its result to cm), 5..12 the histogram's 8 words read out of hs and cleared (to cm, but for mode 4,
    // which has none); then the last results land (hs: 4 clocks) and it is done.
    reg          cm_req = 1'b0, cm_busy = 1'b0;
    reg          cp_busy = 1'b0;                  // the copier runs (below)
    reg  [2:0]   cm_m = 3'd0;
    reg  [4:0]   cs = 5'd0;
    reg  [6:0]   cm_a0 = 7'd0, cm_hb = 7'd0, cm_ba = 7'd0;   // the mode's {max, min} slot; its histogram's; its bursts'
    reg          cm_bu = 1'b0, cm_hi = 1'b0;      // the mode has burst results / a histogram
    always @(posedge clk) begin
        cmw_e <= 1'b0; cr_v <= 1'b0;
        if (stat_store) begin
            cm_req <= 1'b1; cm_m <= stat_mode;
            case (stat_mode)
                3'd0: begin cm_a0 <= 7'd37; cm_hb <= 7'd59; end
                3'd1: begin cm_a0 <= 7'd40; cm_hb <= 7'd67; end
                3'd2: begin cm_a0 <= 7'd43; cm_hb <= 7'd75; end
                3'd3: begin cm_a0 <= 7'd46; cm_hb <= 7'd83; end
                3'd4: begin cm_a0 <= 7'd49; cm_hb <= 7'd0;  end
                default: begin cm_a0 <= 7'd52; cm_hb <= 7'd91; end
            endcase
            cm_ba <= (stat_mode == 3'd3) ? 7'd55 : 7'd57;
            cm_bu <= (stat_mode == 3'd3) || (stat_mode == 3'd4);
            cm_hi <= (stat_mode != 3'd4);
        end
        if (!clr_done) begin cmw_e <= !ci[7]; cmw_a <= ci[6:0]; cmw_d <= cm_init(ci[6:0]); end
        else if (hs_ov && hs_otag[7]) begin cmw_e <= 1'b1; cmw_a <= hs_otag[6:0]; cmw_d <= hs_oval; end
        else if (cm_busy) begin
            case (cs)
                5'd0: begin cmw_e <= 1'b1;  cmw_a <= cm_a0;         cmw_d <= {c_max, c_min}; end
                5'd1: begin cmw_e <= 1'b1;  cmw_a <= cm_a0 + 7'd1;  cmw_d <= c_sum; end
                5'd2: begin cmw_e <= cm_bu; cmw_a <= cm_ba;         cmw_d <= {c_bmax, c_bmin}; end
                5'd3: begin cmw_e <= cm_bu; cmw_a <= cm_ba + 7'd1;  cmw_d <= c_bsum; end
                default: ;
            endcase
        end
        if (clr_done && cm_req && !cm_busy && !cp_busy) begin cm_req <= 1'b0; cm_busy <= 1'b1; cs <= 5'd0; end
        else if (cm_busy) begin
            cs <= cs + 1'b1;
            if (cs == 5'd4) begin
                cr_v <= 1'b1; cr_op <= RMW_MRG; cr_a <= {2'b01, cm_m}; cr_d <= {c_nsat, c_cmax};
                cr_tag <= {1'b1, cm_a0 + 7'd2};
            end else if (cs >= 5'd5 && cs <= 5'd12) begin
                cr_v <= 1'b1; cr_op <= RMW_RDCLR; cr_a <= {2'b00, cs[2:0] - 3'd5};
                cr_tag <= {cm_hi, cm_hb + {4'd0, cs[2:0] - 3'd5}};
            end
            if (cs == 5'd20) cm_busy <= 1'b0;                   // the last result (step 12) landed at step 18
        end
    end

    // stage a run for the expander (the sequencer's steps call this once and then wait for ex_idle_q && !stage)
    task issue(input [1:0] op, input [28:0] a, input [28:0] b, input [28:0] as, input rnd, input [31:0] cnt,
               input [2:0] dk, input [31:0] dv, input [31:0] ds, input mk, input [3:0] k, input [24:0] x,
               input [24:0] xs, input xb, input [2:0] xm);
        begin
            nd_op <= op; nd_a <= a; nd_b <= b; nd_as <= as; nd_rand <= rnd; nd_cnt <= cnt;
            nd_dk <= dk; nd_dv <= dv; nd_ds <= ds; nd_mk <= mk; nd_k <= k; nd_x <= x; nd_xs <= xs; nd_xb <= xb; nd_xm <= xm;
            stage <= 1'b1;
        end
    endtask

    // ================= the expander: a run descriptor -> one request a clock (counters and adders only) =================
    always @(posedge clk) begin
        if (push) lfsr <= lfsr_n;
        if (rst_e) begin
            d_v <= 1'b0; nd_v <= 1'b0;
        end else begin
            if (push) begin
                if ((d_op == OP_RW || d_op == OP_WR2) && !d_sub) d_sub <= 1'b1;      // the pair's second request next
                else begin
                    d_sub <= 1'b0; d_first <= 1'b0;
                    d_a <= d_a + d_as; d_b <= d_b + d_as; d_dv <= d_dv + d_ds; d_x <= d_x + d_xs;
                    d_cnt <= d_cnt - 1'b1;
                    if (d_cnt == 32'd1) d_v <= 1'b0;                                   // the run is done
                end
            end
            if (d_abort) d_v <= 1'b0;
            else if (!d_v && nd_v) begin                                              // load the staged run
                d_v <= 1'b1; d_sub <= 1'b0; d_first <= 1'b1; nd_v <= 1'b0;
                d_op <= nd_op; d_a <= nd_a; d_b <= nd_b; d_as <= nd_as; d_rand <= nd_rand; d_cnt <= nd_cnt;
                d_dk <= nd_dk; d_dv <= nd_dv; d_ds <= nd_ds; d_mk <= nd_mk;
                d_k <= nd_k; d_x <= nd_x; d_xs <= nd_xs; d_xb <= nd_xb; d_xm <= nd_xm;
            end
            if (stage) nd_v <= 1'b1;
        end
    end

    // ================= s5: checks and statistics (no reset: the pipeline is empty in reset; timing) =================
    always @(posedge clk) begin : s5
        integer bi;
        if (rv4) begin
            if (lat4 > cum_rdlat[23:0]) cum_rdlat <= {8'd0, lat4};
            if (chk4) begin
                words_checked <= words_checked + 1'b1;
                if (any4) begin
                    any_err <= 1'b1;
                    if (err_words != 32'hFFFF_FFFF) err_words <= err_words + 1'b1;
                    cum_dq <= cum_dq | dq4; cum_lanes <= cum_lanes | ln4;
                    if (!first_set) begin
                        first_set <= 1'b1; first_dq <= dq4; first_lanes <= ln4;
                        first_addr <= (k4 == K_WALK) ? {A_WALK[27:3] + x4[5:0], 5'd0} : {x4, 5'd0};
                    end
                end
            end
        end
        // the stall read window's counts (cleared when a window starts: st_new, from the sequencer)
        if (st_new) begin
            st_gap_w <= 16'd0; st_gaps16_n <= 32'd0; st_rwords_n <= 32'd0;
        end else if (rv4 && k4 == K_STALL) begin
            if (gap4_v) begin
                if (gap4 > st_gap_w) st_gap_w <= gap4;
                if (gap4_16) st_gaps16_n <= st_gaps16_n + 1'b1;
            end
            st_rwords_n <= st_rwords_n + 1'b1;
        end
        // ORDER: the mask map (bit x of okA / okB), the first word's bytes, the lane words
        for (bi = 0; bi < 32; bi = bi + 1)
            if (ow4 && ob4[bi]) begin okA[bi] <= oka4; okB[bi] <= okb4; end
        if (os04) s0 <= sff4;
        if (oo4[0]) ord_off0 <= nm4;
        if (oo4[1]) ord_off1 <= nm4;
        if (oo4[2]) ord_off2 <= nm4;
        if (oo4[3]) ord_off4 <= nm4;
    end

    // ================= the IP side: the longest not-ready runs with a request waiting (a run's length so far is
    // registered, then compared with the maximum a clock later: timing); burst / BW starts
    always @(posedge clk) begin
        if (hv_q && !cr_q) begin
            if (!low_c) begin low_c <= 1'b1; low_c0 <= ipnow_q; end
        end else low_c <= 1'b0;
        if (hv_q && hw_q && !wr_q) begin
            if (!low_wr) begin low_wr <= 1'b1; low_w0 <= ipnow_q; end
        end else low_wr <= 1'b0;
        runc <= low_c ? ipnow_q - low_c0 : 32'd0;
        runw <= low_wr ? ipnow_q - low_w0 : 32'd0;
        if (runc > cum_cmdlow) cum_cmdlow <= runc;
        if (runw > cum_wrdylow) cum_wrdylow <= runw;
        if (iplat_q) burst_t0 <= ipnow_q;
    end

    always @(posedge clk) begin : g_main
        if (ip_firer) tag_wr <= tag_wr + 1'b1;
        if (rd_data_valid) tag_rd <= tag_rd + 1'b1;
        stat_clr <= 1'b0; stat_store <= 1'b0; st_new <= 1'b0;
        ev_ord <= 1'b0; fd_ev <= 1'b0; fa_ev <= 1'b0; ev_t1 <= 1'b0; ev_t3 <= 1'b0; ev_bw <= 1'b0; ev_m6 <= 1'b0;
        // the events' data and the registered compares
        fd_frd <= frd; fd_fn <= fn; fd_t <= ipnow_q - f_t0; fd_which <= {!wr_q, !cr_q}; fd_ret <= f_ret;
        fa_frd <= frd; fa_fn <= fn; fa_seen <= f_seen; fa_satw <= fn_full; fa_satr <= fn_full || !can_rd_q;
        bw_d <= now - t0;
        in_f1 <= st[SS_F1]; in_t0r <= st[SS_T0R]; in_t2r <= st[SS_T2R];
        hv_q2 <= hv_q; cr_q2 <= cr_q; wr_q2 <= wr_q; ipf_q2 <= ipf_q;
        n_last <= (n + 1'b1 == NLAT); q_done <= (q >= NLAT); q_lt_n <= (q < n);
        wc_f1  <= st[SS_F1] && idle_q && (wc == DRAIN_CLKS - 1);           // = (wc == DRAIN_CLKS) next clock
        wc_win <= (st[SS_T0R] || st[SS_T2R]) && (wc == NSTW - 1);           // = (wc == NSTW) next clock
        wc_t3  <= st[SS_T3] && (wc >= DRAIN_CLKS - 1);                      // = (wc >= DRAIN_CLKS) next clock
        okA_all <= (okA == 32'hFFFF_FFFF); okB_all <= (okB == 32'hFFFF_FFFF); perr <= (err_words != pass_err0);

        if (rst_s) begin
            ph <= PH_IDLE; st <= S1(SS_IDLE); tag_wr <= 0; tag_rd <= 0; n <= 0; wc <= 0; stage <= 1'b0;
            d_abort <= 1'b0; bw_arm <= 1'b0;
        end else begin
            if (bw_arm && ipf_q) begin t0 <= ipnow_q; bw_arm <= 1'b0; end

            // ================= the sequencer =================
            d_abort <= 1'b0; stage <= 1'b0;
            case (1'b1) /* synthesis parallel_case */
            st[SS_IDLE]: if (clr_done) begin ph <= PH_ORDER; st <= S1(SS_O0); end   // the RAMs cleared

            // ---------------- ORDER ----------------
            st[SS_O0]: begin issue(OP_W, A_ORD, 29'd0, 29'd8, 1'b0, 32'd32, D_ZERO, 32'd0, 32'd0, 1'b0, K_NONE, 25'd0, 25'd0,
                                   1'b0, 3'd0); st <= S1(SS_O0W); end                // zero words k = 0..31
            st[SS_O0W]: if (ex_idle_q && !stage) st <= S1(SS_O1);
            st[SS_O1]: begin issue(OP_W, A_ORD, 29'd0, 29'd8, 1'b0, 32'd32, D_ONES, 32'd0, 32'd1, 1'b1, K_NONE, 25'd0, 25'd0,
                                   1'b0, 3'd0); st <= S1(SS_O1W); end                // all ones, every byte but k masked
            st[SS_O1W]: if (ex_idle_q && !stage) st <= S1(SS_O2);
            st[SS_O2]: begin issue(OP_W, A_ORD + {7'd64, 3'b000}, 29'd0, 29'd0, 1'b0, 32'd1, D_LANE, 32'd0, 32'd0, 1'b0,
                                   K_NONE, 25'd0, 25'd0, 1'b0, 3'd0); st <= S1(SS_O2W); end   // the lane word at word 64
            st[SS_O2W]: if (ex_idle_q && !stage) st <= S1(SS_O3);
            st[SS_O3]: begin                                                        // read back the 32 masked words
                max_out <= 64;
                issue(OP_R, A_ORD, 29'd0, 29'd8, 1'b0, 32'd32, D_ZERO, 32'd0, 32'd0, 1'b0, K_ORDER, 25'd0, 25'd1, 1'b0, 3'd0);
                st <= S1(SS_O3W); oj <= 3'd0;
            end
            st[SS_O3W]: if (ex_idle_q && !stage) st <= S1(SS_O3L);
            st[SS_O3L]: begin                                                       // the lane word at +0 / +1 / +2 / +4
                issue(OP_R, A_ORD + {7'd64, 3'b000} + ((oj == 3'd3) ? 29'd4 : {26'd0, oj}), 29'd0, 29'd0, 1'b0, 32'd1,
                      D_ZERO, 32'd0, 32'd0, 1'b0, K_ORDER, 25'd32 + {22'd0, oj}, 25'd0, 1'b0, 3'd0);
                st <= S1(SS_O3LW);
            end
            st[SS_O3LW]: if (ex_idle_q && !stage) begin
                if (oj == 3'd3) st <= S1(SS_O4); else begin oj <= oj + 1'b1; st <= S1(SS_O3L); end
            end
            st[SS_O4]: if (idle_q) begin                                            // all back: the verdicts (results)
                ev_ord <= 1'b1;
                ph <= PH_FIFO; st <= S1(SS_F0); frd <= 1'b0; rep <= 2'd0;
            end

            // ---------------- FIFO depth (counted where the IP takes commands: the queue's head) ----------------
            st[SS_F0]: begin                                                        // a run: bank 1, a new row each
                max_out <= TN - 2;
                issue(frd ? OP_R : OP_W, {1'b0, 3'd1, 15'd0, 10'd0}, 29'd0, 29'd1024, 1'b0, FIFO_MAX + 64, D_ZERO, 32'd0,
                      32'd0, 1'b0, K_NONE, 25'd0, 25'd0, 1'b0, 3'd0);
                fn <= 16'd0; fn_nz <= 1'b0; fn_full <= 1'b0; f_seen <= 1'b0; f_low <= 8'd0; f_ret <= 0;
                st <= S1(SS_F0R);
            end
            st[SS_F0R]: begin
                if (rd_data_valid) f_ret <= f_ret + 1'b1;
                if (ipf_q) begin
                    fn <= fn + 1'b1; fn_nz <= 1'b1; fn_full <= (fn >= FIFO_MAX - 1); f_low <= 8'd0;
                    if (!fn_nz) f_t0 <= ipnow_q;
                end else if (hv_q && fn_nz) begin
                    if (!f_seen) begin f_seen <= 1'b1; fd_ev <= 1'b1; end            // the first drop (results)
                    f_low <= f_low + 1'b1;
                end
                if (fn_full || (!can_rd_q && frd && fn_nz) || f_low == 8'd8 || (ex_idle_q && !stage)) begin
                    d_abort <= 1'b1; st <= S1(SS_F1); wc <= 0; fa_ev <= 1'b1;          // the run's end (results)
                end
            end
            st[SS_F1]: begin                                                        // drain: all back, DRAIN_CLKS quiet
                if (!idle_q) wc <= 0; else wc <= wc + 1'b1;
                if (wc_f1) begin
                    if (rep == 2'd3) begin
                        rep <= 2'd0;
                        if (!frd) begin frd <= 1'b1; st <= S1(SS_F0); end
                        else begin                                                  // done: to the stall windows
                            ph <= PH_STALL; st <= S1(SS_T0); wc <= 0; st_new <= 1'b1;
                        end
                    end else begin rep <= rep + 1'b1; st <= S1(SS_F0); end
                end
            end

            // ---------------- STALL windows (the not-ready runs at the IP: counted by the results block) ---------
            st[SS_T0]: begin                                                        // saturating sequential reads, bank 2
                max_out <= TN - 2;
                issue(OP_R, wa(3'd2, 22'd0), 29'd0, 29'd8, 1'b0, NSTW + NSTW, D_ZERO, 32'd0, 32'd0, 1'b0, K_STALL, 25'd0,
                      25'd0, 1'b0, 3'd0);
                wc <= 0; st <= S1(SS_T0R);
            end
            st[SS_T0R]: begin
                wc <= wc + 1'b1;
                if (wc_win) begin d_abort <= 1'b1; st <= S1(SS_T1); end
            end
            st[SS_T1]: if (idle_q) begin ev_t1 <= 1'b1; st <= S1(SS_T2); end
            st[SS_T2]: begin                                                        // saturating sequential writes, bank 2
                issue(OP_W, wa(3'd2, 22'd0), 29'd0, 29'd8, 1'b0, NSTW + NSTW, D_REP, 32'd0, 32'd1, 1'b0, K_NONE, 25'd0,
                      25'd0, 1'b0, 3'd0);
                wc <= 0; st <= S1(SS_T2R);
            end
            st[SS_T2R]: begin
                wc <= wc + 1'b1;
                if (wc_win) begin d_abort <= 1'b1; st <= S1(SS_T3); wc <= 0; end
            end
            st[SS_T3]: begin
                wc <= wc + 1'b1;
                if (wc_t3 && idle_q) begin ev_t3 <= 1'b1; ph <= PH_LAT; st <= S1(SS_L0); mode <= 3'd0; end
            end

            // ---------------- LATENCY ----------------
            st[SS_L0]: begin                                                        // a mode's run statistics from zero
                stat_clr <= 1'b1; stat_mode <= mode;
                n <= 0; q <= 0; n_last <= (NLAT == 32'd1); q_done <= 1'b0; q_lt_n <= 1'b0;
                max_out <= (mode == 3'd3 || mode == 3'd4) ? 8 : 1;
                st <= (mode == 3'd5) ? S1(SS_L5) : S1(SS_L1);
            end
            st[SS_L1]: begin                                                        // one sample (a burst of 8 for 3, 4)
                case (mode)
                    3'd0: issue(OP_R, wa(3'd3, n[21:0]), 29'd0, 29'd0, 1'b0, 32'd1, D_ZERO, 32'd0, 32'd0, 1'b0, K_LAT,
                                25'd0, 25'd0, 1'b1, mode);
                    3'd1: issue(OP_R, {1'b0, lfsr[24:0], 3'b000}, 29'd0, 29'd0, 1'b0, 32'd1, D_ZERO, 32'd0, 32'd0, 1'b0,
                                K_LAT, 25'd0, 25'd0, 1'b1, mode);
                    3'd2: issue(OP_R, {1'b0, 3'd5, lfsr[14:0], lfsr[21:15], 3'b000}, 29'd0, 29'd0, 1'b0, 32'd1, D_ZERO,
                                32'd0, 32'd0, 1'b0, K_LAT, 25'd0, 25'd0, 1'b1, mode);
                    3'd3: issue(OP_R, wa(3'd3, {n[18:0], 3'b000}), 29'd0, 29'd8, 1'b0, 32'd8, D_ZERO, 32'd0, 32'd0, 1'b0,
                                K_LAT, 25'd0, 25'd0, 1'b1, mode);
                    default: issue(OP_R, {1'b0, lfsr[21:0], 3'b000, 3'b000}, 29'd0, 29'd8, 1'b0, 32'd8, D_ZERO, 32'd0,
                                   32'd0, 1'b0, K_LAT, 25'd0, 25'd0, 1'b1, mode);
                endcase
                st <= S1(SS_L2);
            end
            st[SS_L2]: if (ex_idle_q && !stage && idle_q) begin                     // the sample's data back
                if (n_last) st <= S1(SS_L3);
                else begin n <= n + 1'b1; st <= S1(SS_L1); end
            end
            st[SS_L5]: if (ex_idle_q && !stage) begin                               // loaded: write runs of 32, and a read
                if (q_done) begin if (rd_none_q && !pipe_busy) st <= S1(SS_L3); end //   (one at a time) between them
                else if (rd_none_q && q_lt_n) begin
                    issue(OP_R, {1'b0, 3'd7, lfsr[21:0], 3'b000}, 29'd0, 29'd0, 1'b0, 32'd1, D_ZERO, 32'd0, 32'd0, 1'b0,
                          K_LAT, 25'd0, 25'd0, 1'b1, 3'd5);
                    q <= q + 1'b1;
                end else begin
                    issue(OP_W, wa(3'd6, wn), 29'd0, 29'd8, 1'b0, 32'd32, D_REP, {10'd0, wn}, 32'd1, 1'b0, K_NONE, 25'd0,
                          25'd0, 1'b0, 3'd0);
                    wn <= wn + 22'd32; n <= n + 1'b1;
                end
            end
            st[SS_L3]: if (ex_idle_q && !stage && idle_q) begin                     // the run is over: commit it
                stat_store <= 1'b1; stat_mode <= mode; st <= S1(SS_L4);
            end
            st[SS_L4]: if (!stat_store && !cm_req && !cm_busy) begin                // committed
                if (mode == 3'd5) begin ph <= PH_BW; st <= S1(SS_B0); bwn <= 3'd0; end
                else begin mode <= mode + 1'b1; st <= S1(SS_L0); end
            end

            // ---------------- BANDWIDTH (from the IP's first take to the last word back) ----------------
            st[SS_B0]: begin                                                        // 2^BW_LOG2 commands
                max_out <= TN - 2; bw_arm <= 1'b1;
                case (bwn)
                    3'd0: issue(OP_W, wa(3'd6, 22'd0), 29'd0, 29'd8, 1'b0, NBW, D_REP, 32'd0, 32'd1, 1'b0, K_NONE, 25'd0,
                                25'd0, 1'b0, 3'd0);
                    3'd1: issue(OP_R, wa(3'd6, 22'd0), 29'd0, 29'd8, 1'b0, NBW, D_ZERO, 32'd0, 32'd0, 1'b0, K_NONE, 25'd0,
                                25'd0, 1'b0, 3'd0);
                    3'd2: issue(OP_W, 29'd0, 29'd0, 29'd0, 1'b1, NBW, D_REP, 32'd0, 32'd1, 1'b0, K_NONE, 25'd0, 25'd0,
                                1'b0, 3'd0);
                    3'd3: issue(OP_R, 29'd0, 29'd0, 29'd0, 1'b1, NBW, D_ZERO, 32'd0, 32'd0, 1'b0, K_NONE, 25'd0, 25'd0,
                                1'b0, 3'd0);
                    default: issue(OP_WR2, wa(3'd6, 22'd0), wa(3'd7, 22'd0), 29'd8, 1'b0, NBW / 2, D_REP, 32'd0, 32'd1,
                                   1'b0, K_NONE, 25'd0, 25'd0, 1'b0, 3'd0);                   // write / read alternating
                endcase
                st <= S1(SS_B0W);
            end
            st[SS_B0W]: if (ex_idle_q && !stage) st <= S1(SS_B1);
            st[SS_B1]: if (idle_q) begin                                            // the wall clocks (results)
                ev_bw <= 1'b1; ev_bwn <= bwn;
                if (bwn == 3'd4) begin ph <= PH_MARCH; st <= S1(SS_M0); end
                else begin bwn <= bwn + 1'b1; st <= S1(SS_B0); end
            end

            // ---------------- MARCH ----------------
            st[SS_M0]: begin                                                        // the walk: 64 words written
                max_out <= TN - 2;
                issue(OP_W, A_WALK, 29'd0, 29'd8, 1'b0, 32'd64, D_WALK, 32'd0, 32'd1, 1'b0, K_NONE, 25'd0, 25'd0, 1'b0, 3'd0);
                st <= S1(SS_M0W);
            end
            st[SS_M0W]: if (ex_idle_q && !stage) st <= S1(SS_M1);
            st[SS_M1]: begin                                                        // ... and read
                issue(OP_R, A_WALK, 29'd0, 29'd8, 1'b0, 32'd64, D_ZERO, 32'd0, 32'd0, 1'b0, K_WALK, 25'd0, 25'd1, 1'b0, 3'd0);
                st <= S1(SS_M1W);
            end
            st[SS_M1W]: if (ex_idle_q && !stage) st <= S1(SS_M2);
            st[SS_M2]: begin                                                        // E1: up w(P)
                issue(OP_W, 29'd0, 29'd0, 29'd8, 1'b0, NWRD, D_PAT0, 32'd0, 32'd1, 1'b0, K_NONE, 25'd0, 25'd0, 1'b0, 3'd0);
                st <= S1(SS_M2W);
            end
            st[SS_M2W]: if (ex_idle_q && !stage) st <= S1(SS_M3);
            st[SS_M3]: begin                                                        // E2: up r(P) w(~P)
                issue(OP_RW, 29'd0, 29'd0, 29'd8, 1'b0, NWRD, D_PAT1, 32'd0, 32'd1, 1'b0, K_M0, 25'd0, 25'd1, 1'b0, 3'd0);
                st <= S1(SS_M3W);
            end
            st[SS_M3W]: if (ex_idle_q && !stage) st <= S1(SS_M4);
            st[SS_M4]: begin                                                        // E3: down r(~P) w(P)
                issue(OP_RW, {1'b0, NWRD[24:0] - 1'b1, 3'b000}, 29'd0, -29'sd8, 1'b0, NWRD, D_PAT0, NWRD - 1, 32'hFFFF_FFFF,
                      1'b0, K_M1, NWRD[24:0] - 1'b1, 25'h1FF_FFFF, 1'b0, 3'd0);
                st <= S1(SS_M4W);
            end
            st[SS_M4W]: if (ex_idle_q && !stage) st <= S1(SS_M5);
            st[SS_M5]: begin                                                        // E4: up r(P)
                issue(OP_R, 29'd0, 29'd0, 29'd8, 1'b0, NWRD, D_ZERO, 32'd0, 32'd0, 1'b0, K_M0, 25'd0, 25'd1, 1'b0, 3'd0);
                st <= S1(SS_M5W);
            end
            st[SS_M5W]: if (ex_idle_q && !stage) st <= S1(SS_M6);
            st[SS_M6]: if (idle_q) begin                                            // all back: the pass is done (results)
                ev_m6 <= 1'b1;
                ph <= PH_ORDER; st <= S1(SS_O0);
            end
            default: begin ph <= PH_IDLE; st <= S1(SS_IDLE); end
            endcase
        end
    end

    // ================= the results block: the probes' results, on the sequencer's events (a clock after them, with
    // their data registered), so each result register's logic is local (no reset: an event fires only after reset)
    always @(posedge clk) begin : results
        // ORDER: the verdicts; the FIFO maxima from zero
        if (ev_ord) begin
            ord_okA <= okA; ord_okB <= okB; s0_r <= s0;
            ord_pol <= okA_all ? 32'd1 : okB_all ? 32'd2 : 32'd0;
            fw_first_max <= 16'd0; fw_8_max <= 16'd0; fr_first_max <= 16'd0; fr_8_max <= 16'd0;
        end
        // FIFO: the first drop, the run's end, and the maxima while it drains
        if (fd_ev) begin
            if (!fd_frd) begin fw_first <= fd_fn; fw_which <= fd_which; fw_t <= fd_t; end
            else begin fr_first <= fd_fn; fr_t <= fd_t; fr_ret <= fd_ret; end
        end
        if (fa_ev) begin
            if (!fa_frd) begin
                fw_8 <= fa_fn; if (fa_satw) f_sat_w <= 1'b1;
                if (fa_fn > fw_8_max) fw_8_max <= fa_fn;
                if (!fa_seen) fw_first <= fa_fn;
            end else begin
                fr_8 <= fa_fn; if (fa_satr) f_sat_r <= 1'b1;
                if (fa_fn > fr_8_max) fr_8_max <= fa_fn;
                if (!fa_seen) fr_first <= fa_fn;
            end
        end
        if (in_f1) begin
            if (fw_first > fw_first_max) begin fw_first_max <= fw_first; fw_t_max <= fw_t; end
            if (fr_first > fr_first_max) begin fr_first_max <= fr_first; fr_t_max <= fr_t; fr_ret_max <= fr_ret; end
        end
        // STALL: the not-ready runs at the IP in each window (a clock behind the window: in_t0r, in_t2r and the
        // registered IP events are the window's, shifted), and the windows' results
        if (st_new) begin st_low_w <= 16'd0; run <= 16'd0; end
        else if (in_t0r) begin
            if (hv_q2 && !cr_q2) run <= run + 1'b1;
            else begin if (run > st_low_w) st_low_w <= run; run <= 16'd0; end
        end
        if (ev_t1) begin
            if (st_gap_w > st_gap_c) st_gap_c <= st_gap_w;
            if (st_low_w > st_low_c) st_low_c <= st_low_w;
            st_gaps16 <= st_gaps16_n; st_rwords <= st_rwords_n;
            run <= 16'd0; st_wrun_w <= 16'd0; st_wruns16_n <= 32'd0; st_wwords_n <= 32'd0;
        end else if (in_t2r) begin
            if (ipf_q2) st_wwords_n <= st_wwords_n + 1'b1;
            if (hv_q2 && !(cr_q2 && wr_q2)) run <= run + 1'b1;
            else begin
                if (run > st_wrun_w) st_wrun_w <= run;
                if (run >= 16'd16) st_wruns16_n <= st_wruns16_n + 1'b1;
                run <= 16'd0;
            end
        end
        if (ev_t3) begin
            if (st_wrun_w > st_wrun_c) st_wrun_c <= st_wrun_w;
            st_wruns16 <= st_wruns16_n; st_wwords <= st_wwords_n;
        end
        // BW: the wall clocks (bw_d = now - t0 at the event's clock); the march's error count at its start
        if (ev_bw) begin
            case (ev_bwn)
                3'd0: bw0 <= bw_d;
                3'd1: bw1 <= bw_d;
                3'd2: bw2 <= bw_d;
                3'd3: bw3 <= bw_d;
                default: begin bw4 <= bw_d; pass_err0 <= err_words; end
            endcase
        end
        // MARCH: a pass done
        if (ev_m6) begin
            passes <= passes + 1'b1;
            if (perr) bad_passes <= bad_passes + 1'b1;
            seed <= seed ^ lfsr ^ {passes[15:0], loopn};
            loops <= loops + 1'b1; loopn <= loopn + 1'b1;
        end
    end

    // ================= the copier =================
    // The 40 slots in registers: tester slots 0..36 and 99..101 (index h 37..39).
    localparam integer NH = 40;
    wire [NH*32-1:0] live;
    assign live[  0*32 +: 32] = {loopn, 9'd0, any_err, (ph != PH_IDLE), 1'b0, ph};
    assign live[  1*32 +: 32] = loops;
    assign live[  2*32 +: 32] = passes;
    assign live[  3*32 +: 32] = bad_passes;
    assign live[  4*32 +: 32] = err_words;
    assign live[  5*32 +: 32] = first_addr;
    assign live[  6*32 +: 32] = first_dq;
    assign live[  7*32 +: 32] = first_lanes;
    assign live[  8*32 +: 32] = cum_dq;
    assign live[  9*32 +: 32] = cum_lanes;
    assign live[ 10*32 +: 32] = words_checked[47:16];
    assign live[ 11*32 +: 32] = cum_rdlat;
    assign live[ 12*32 +: 32] = cum_cmdlow;
    assign live[ 13*32 +: 32] = cum_wrdylow;
    assign live[ 14*32 +: 32] = ord_pol;
    assign live[ 15*32 +: 32] = ord_okA;
    assign live[ 16*32 +: 32] = ord_okB;
    assign live[ 17*32 +: 32] = s0_r;
    assign live[ 18*32 +: 32] = ord_off0;
    assign live[ 19*32 +: 32] = ord_off1;
    assign live[ 20*32 +: 32] = ord_off2;
    assign live[ 21*32 +: 32] = ord_off4;
    assign live[ 22*32 +: 32] = {fw_8_max, fw_first_max};
    assign live[ 23*32 +: 32] = {fr_8_max, fr_first_max};
    assign live[ 24*32 +: 32] = {28'd0, f_sat_r, f_sat_w, fw_which};
    assign live[ 25*32 +: 32] = {st_gap_c, st_gap_w};
    assign live[ 26*32 +: 32] = st_gaps16;
    assign live[ 27*32 +: 32] = st_rwords;
    assign live[ 28*32 +: 32] = {st_low_c, st_low_w};
    assign live[ 29*32 +: 32] = {st_wrun_c, st_wrun_w};
    assign live[ 30*32 +: 32] = st_wruns16;
    assign live[ 31*32 +: 32] = st_wwords;
    assign live[ 32*32 +: 32] = bw0;
    assign live[ 33*32 +: 32] = bw1;
    assign live[ 34*32 +: 32] = bw2;
    assign live[ 35*32 +: 32] = bw3;
    assign live[ 36*32 +: 32] = bw4;
    assign live[ 37*32 +: 32] = fw_t_max;
    assign live[ 38*32 +: 32] = fr_t_max;
    assign live[ 39*32 +: 32] = fr_ret_max;
    // A pass: cp_go starts it (from registers); in the next clock (cp_cap) the 40 register slots are captured, and
    // slot t = 0..101 goes through stage A (cp_t: cm's address), B (cm's word; the register slot's group of 8), C (the
    // slot's word) and D (the RAMs' write registers).  The last write toggles d_done / answers the UART.
    reg  [NH*32-1:0] cap = {NH*32{1'b0}};
    reg  [SNAP_LOG2-1:0] sn = 0;
    reg          tick = 1'b0;                     // a display update is due
    reg          d_wb = 1'b1;                     // the display bank the copier writes (the display shows the other)
    reg          d_wait = 1'b0;                   // it was written: waiting for the display to show it
    reg          cp_run = 1'b0, cp_cap = 1'b0;    // the pass reads slots (stage A); the capture clock
    reg          cp_d = 1'b0, cp_u = 1'b0;        // the pass writes the display RAM / the UART RAM
    reg          cp_db = 1'b0, cp_ub = 1'b0, cp_ur = 1'b0;   // their banks; the UART request it answers
    reg          pb_v = 1'b0, pc_v = 1'b0, pb_r = 1'b0, pc_r = 1'b0, pb_l = 1'b0, pc_l = 1'b0, pw_l = 1'b0;
    reg  [5:0]   pb_h = 6'd0, pc_h = 6'd0;        // the register slot's index
    reg  [6:0]   pb_s = 7'd0, pc_s = 7'd0;        // the slot's number on the screen (t + 12)
    reg  [31:0]  cm_q2 = 32'd0;
    reg  [159:0] m1 = 160'd0;                     // B: in each group of 8 register slots, slot h mod 8
    (* syn_preserve = 1 *) reg [14:0] pb_hr = 15'd0; // B: h mod 8, a copy per group (timing: the selects' fanout)
    wire         u_pend = (u_req != u_ack);
    wire         cp_go  = clr_done && !cp_busy && !cm_busy && !cm_req && ((tick && !d_wait) || u_pend);
    always @(posedge clk) begin : copier
        integer k;
        reg [31:0] m2;
        sn <= sn + 1'b1;
        tick <= (tick && !(cp_go && !d_wait)) || (&sn);                           // due; used by a pass
        if (d_wait && d_shown == d_wb) begin d_wait <= 1'b0; d_wb <= ~d_wb; end    // the display shows it
        cp_cap <= 1'b0;
        if (cp_go) begin
            cp_busy <= 1'b1; cp_run <= 1'b1; cp_cap <= 1'b1; cp_t <= 7'd0;
            cp_d <= tick && !d_wait; cp_db <= d_wb; cp_u <= u_pend; cp_ub <= u_bank; cp_ur <= u_req;
        end else if (cp_run) begin
            cp_t <= cp_t + 1'b1;
            if (cp_t == 7'd101) cp_run <= 1'b0;
        end
        if (cp_cap) cap <= live;
        // A -> B
        pb_v <= cp_run; pb_r <= (cp_t >= 7'd37) && (cp_t <= 7'd98); pb_l <= (cp_t == 7'd101);
        pb_h <= (cp_t >= 7'd99) ? cp_t[5:0] - 6'd62 : cp_t[5:0];
        pb_hr <= {5{(cp_t >= 7'd99) ? cp_t[2:0] - 3'd6 : cp_t[2:0]}};
        pb_s <= cp_t + 7'd12;
        // B -> C
        pc_v <= pb_v; pc_r <= pb_r; pc_h <= pb_h; pc_s <= pb_s; pc_l <= pb_l; cm_q2 <= cm_q;
        for (k = 0; k < 5; k = k + 1) m1[32 * k +: 32] <= cap[32 * (8 * k + pb_hr[3 * k +: 3]) +: 32];
        // C -> D: the RAMs' write registers (the clear pass first)
        case (pc_h[5:3])
            3'd0: m2 = m1[  0 +: 32];
            3'd1: m2 = m1[ 32 +: 32];
            3'd2: m2 = m1[ 64 +: 32];
            3'd3: m2 = m1[ 96 +: 32];
            default: m2 = m1[128 +: 32];
        endcase
        if (!clr_done) begin
            dm_we <= 1'b1; dm_wa <= ci; um_we <= 1'b1; um_wa <= ci; wd <= 32'd0; pw_l <= 1'b0;
        end else begin
            dm_we <= pc_v && cp_d; dm_wa <= {cp_db, pc_s};
            um_we <= pc_v && cp_u; um_wa <= {cp_ub, pc_s};
            wd    <= pc_r ? cm_q2 : m2;
            pw_l  <= pc_v && pc_l;
        end
        // D: the last slot is written at this edge: the bank is done / the UART's copy is answered
        if (pw_l) begin
            cp_busy <= 1'b0;
            if (cp_d) begin d_done <= ~d_done; d_wait <= 1'b1; end
            if (cp_u) u_ack <= cp_ur;
        end
    end
endmodule
`default_nettype wire
