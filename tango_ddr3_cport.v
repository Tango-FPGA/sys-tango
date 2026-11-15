// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_cport -- a core's DDR3 port: MiSTer's DDRAM_* (Template_MiSTer sys/emu_ports.vh
// 98-109: 64-bit Avalon MM with bursts, in the core's DDRAM_CLK) onto tango_ddr3_arb in the IP's clk_out domain.
//   - Address: the core window.  MiSTer cores address DDRAM at 0x30000000 (DDRAM_ADDR = {4'b0011, ...}: Genesis_MiSTer
//     rtl/ddram.sv 59, NeoGeo_MiSTer rtl/mem/ddram.sv 70); here DDRAM_ADDR[28:25] are ignored and the byte address is
//     0x3000_0000 + DDRAM_ADDR[24:0] * 8 (DDR3 banks 6-7), so no core can reach the scaler's region.
//   - Crossing: three tango_async_fifo (sys/tango_cdc.v): commands {write, address, burst count} and write beats
//     {byte enables, data} to clk_ddr; returned words {first lane, last lane, data} to the core.
//   - DDRAM_BUSY (waitrequest): commands while the command FIFO has room; write beats while the beat FIFO has room.
//   - clk_ddr side, in order: a write burst becomes 32-B IP words of up to 4 beats (unwritten lanes masked); a read
//     burst becomes one IP read per 32-B word it touches, each sent only with room reserved for its word in the return
//     FIFO (the IP's rd_data has no ready); the core side then gives the word's lanes as DOUT beats, one a clock.
// Burst counts 1..255 (MiSTer's); a read burst may cover up to 64 words (the return FIFO): 255 beats need 65 words
// when unaligned, so the return FIFO is 2^RF_LOG2 = 128 words.
//==============================================================================
`default_nettype none
module tango_ddr3_cport #(
    parameter integer RF_LOG2 = 7
) (
    // the core (DDRAM_CLK)
    input  wire          DDRAM_CLK,
    input  wire          crst,
    output wire          DDRAM_BUSY,
    input  wire [7:0]    DDRAM_BURSTCNT,
    input  wire [28:0]   DDRAM_ADDR,
    output reg  [63:0]   DDRAM_DOUT = 64'd0,
    output reg           DDRAM_DOUT_READY = 1'b0,
    input  wire          DDRAM_RD,
    input  wire [63:0]   DDRAM_DIN,
    input  wire [7:0]    DDRAM_BE,
    input  wire          DDRAM_WE,
    // tango_ddr3_arb (clk_ddr)
    input  wire          clk,
    input  wire          rst,
    output reg           q_valid = 1'b0,
    output reg           q_write = 1'b0,
    output reg  [28:0]   q_addr = 29'd0,
    output reg  [255:0]  q_wdata = 256'd0,
    output reg  [31:0]   q_wmask = 32'd0,
    input  wire          q_ready,
    input  wire          r_valid,
    input  wire [255:0]  r_data
);
    // ================= core side =================
    wire [4:0] cf_free;
    wire [8:0] wf_free;
    reg        inb = 1'b0;
    reg [7:0]  left = 8'd0;
    // the clk_ddr side out of reset (rst: until the IP has calibrated), synchronised to DDRAM_CLK: the core waits on
    // DDRAM_BUSY until then (as on the V port) -- without it the async FIFOs' write
    // sides showed room while their read sides were held in reset, and the core's commands were taken and lost
    reg  [1:0] rdy_s = 2'b00;
    always @(posedge DDRAM_CLK) rdy_s <= {rdy_s[0], !rst};
    wire rdy       = rdy_s[1] && !crst;
    wire cmd_room  = rdy && (cf_free >= 5'd2) && (wf_free >= 9'd2);
    wire beat_room = rdy && (wf_free >= 9'd2);
    assign DDRAM_BUSY = inb ? !beat_room : !cmd_room;
    wire take_cmd  = !inb && (DDRAM_RD || DDRAM_WE) && cmd_room;
    wire take_beat = DDRAM_WE && (inb ? beat_room : cmd_room);
    always @(posedge DDRAM_CLK) begin
        if (crst) inb <= 1'b0;
        else if (take_beat) begin
            if (!inb) begin
                if (DDRAM_BURSTCNT > 8'd1) begin inb <= 1'b1; left <= DDRAM_BURSTCNT - 1'b1; end
            end else begin
                if (left == 8'd1) inb <= 1'b0;
                left <= left - 1'b1;
            end
        end
    end
    // commands {write, addr[24:0], burstcount} and beats {be, data}
    wire        cf_empty, wf_empty;
    wire [33:0] cf_q;
    wire [71:0] wf_q;
    reg         cf_rd = 1'b0;
    reg         wf_rd;
    tango_async_fifo #(.AW(4), .W(34)) u_cf (.wclk(DDRAM_CLK), .wrst(crst), .we(take_cmd),
        .wd({DDRAM_WE, DDRAM_ADDR[24:0], DDRAM_BURSTCNT}), .free(cf_free),
        .rclk(clk), .rrst(rst), .rd(cf_rd), .rdata(cf_q), .empty(cf_empty));
    tango_async_fifo #(.AW(8), .W(72)) u_wf (.wclk(DDRAM_CLK), .wrst(crst), .we(take_beat), .wd({DDRAM_BE, DDRAM_DIN}),
        .free(wf_free), .rclk(clk), .rrst(rst), .rd(wf_rd), .rdata(wf_q), .empty(wf_empty));
    // returned words {first lane, last lane, data} to the core: give each word's lanes as beats
    wire [RF_LOG2:0] rf_free;
    wire         rf_empty;
    wire [259:0] rf_q;
    reg          rf_rd = 1'b0, rf_have = 1'b0;
    reg  [259:0] rw = 260'd0;
    reg  [1:0]   lane = 2'd0;
    reg          rf_we = 1'b0;
    reg  [259:0] rf_wd = 260'd0;
    tango_async_fifo #(.AW(RF_LOG2), .W(260)) u_rf (.wclk(clk), .wrst(rst), .we(rf_we), .wd(rf_wd), .free(rf_free),
        .rclk(DDRAM_CLK), .rrst(crst), .rd(rf_rd), .rdata(rf_q), .empty(rf_empty));
    reg rf_pend = 1'b0;                                       // a read issued, its data next clock
    always @(posedge DDRAM_CLK) begin
        rf_rd <= 1'b0; DDRAM_DOUT_READY <= 1'b0;
        if (crst) begin rf_have <= 1'b0; rf_pend <= 1'b0; end
        else begin
            rf_pend <= rf_rd;
            if (rf_pend) begin rw <= rf_q; rf_have <= 1'b1; lane <= rf_q[259:258]; end
            else if (rf_have) begin
                DDRAM_DOUT <= rw[64 * lane +: 64]; DDRAM_DOUT_READY <= 1'b1;
                if (lane == rw[257:256]) rf_have <= 1'b0;
                lane <= lane + 1'b1;
            end
            if (!rf_have && !rf_pend && !rf_rd && !rf_empty) rf_rd <= 1'b1;
        end
    end

    // ================= clk_ddr side: the splitter =================
    // Timing structure: the two go decisions (r_go, w_go) and pop_b are ONE OR TWO LUTs of
    // registers.  What used to be recomputed from the 9-bit sn / nb / 3-bit acc_n every clock (sn != 0, sn == nb,
    // acc_n == nb, acc_n < nb) is kept as registered flags, updated with the state they describe; and the late go
    // signals only SELECT between values that are computed from registers (as the arbiter's grant).
    // Nothing visible at the port changes: a lockstep simulation compares every output of the old module
    // with this one on every clock.
    reg        busy = 1'b0, s_w = 1'b0;
    reg [24:0] sa = 25'd0;                                     // next beat (64-bit word) address
    reg [8:0]  sn = 9'd0;                                      // beats left in the command
    reg        cf_pend = 1'b0;                                 // a command read, its data now on cf_q
    // registered flags (each equal to the expression in its comment, whenever busy)
    reg        ract = 1'b0;                                    // busy && !s_w && sn != 0   (a read command with beats left)
    reg        wact = 1'b0;                                    // busy &&  s_w && sn != 0   (a write command with beats left)
    reg        lastw = 1'b1;                                   // sn == nb                  (the next IP word is the last)
    reg        acc_full = 1'b1;                                // acc_n == nb               (the word being built is complete)
    // write beats: a 2-entry skid in front of the beat FIFO (its data comes one clock after rd)
    reg [71:0] sk0 = 0, sk1 = 0;
    reg [1:0]  skn = 2'd0;
    reg        rdp = 1'b0;
    wire       pop_b;
    wire       want_rd = !wf_empty && ({1'b0, skn} - (pop_b ? 3'd1 : 3'd0) + (rdp ? 3'd1 : 3'd0)) < 3'd2;
    always @(*) wf_rd = want_rd;
    // the IP word being built
    wire [1:0] l0 = sa[1:0];
    // beats in this IP word, min(beats left, 4 - first lane): a register, updated with sa / sn (timing)
    reg  [8:0] nb = 9'd0;
    function [8:0] nbf(input [8:0] n, input [1:0] lo);
        nbf = (n < 9'd4 - {7'd0, lo}) ? n : 9'd4 - {7'd0, lo};
    endfunction
    wire [1:0] l1 = l0 + nb[1:0] - 1'b1;
    reg  [255:0] acc_d = 0;
    reg  [31:0]  acc_m = 32'hFFFF_FFFF;
    reg  [2:0]   acc_n = 3'd0;
    integer      ab;                                            // lane loop index (acc_m)
    reg  [1:0]   fl = 2'd0;                                     // the lane the next beat fills (a register)
    // acc_n <= nb always (a beat is taken only while acc_n < nb), so acc_n < nb is !acc_full
    assign pop_b = wact && !acc_full && (skn != 0);
    // The word's four 64-bit lanes (acc_d, acc_m) take their beat enables from their own copies of wact, acc_full,
    // skn, fl and acc_n == 0 (timing: pop_b fanned out to 55 loads and its lane decode to 48
    // more, two hops of 3.8-3.9 ns into acc_d's enables failed clk_x1 by 0.19 ns in a multi-game Scramble draw).  The
    // copies are syn_preserve registers (GowinSynthesis merges identical LUTs), each updated
    // as its original from its lane's own pop and the data side's go, and equal to it every clock (checked below in
    // simulation); pop_b keeps the beat skid, the counts and the beat FIFO's read.
    (* syn_preserve = 1 *) reg [3:0] bw_l  = 4'b0000;        // wact, per lane
    (* syn_preserve = 1 *) reg [3:0] bf_l  = 4'b1111;        // acc_full, per lane
    (* syn_preserve = 1 *) reg [7:0] bs_l  = 8'd0;           // skn, per lane at [2k +: 2]
    (* syn_preserve = 1 *) reg [7:0] bfl_l = 8'd0;           // fl, per lane at [2k +: 2]
    (* syn_preserve = 1 *) reg [3:0] bz_l  = 4'b1111;        // acc_n == 0, per lane
    wire [3:0] pb_l;                                         // = pop_b, per lane
    genvar gb;
    generate for (gb = 0; gb < 4; gb = gb + 1) begin : g_bl
        assign pb_l[gb] = bw_l[gb] && !bf_l[gb] && (bs_l[2 * gb +: 2] != 2'd0);
    end endgenerate
    // reads: the return info in issue order {first lane, last lane}; words reserved in the return FIFO
    reg [3:0]  ri [0:31];
    reg [5:0]  ri_wp = 0, ri_rp = 0;
    reg [RF_LOG2+1:0] rf_res = 0;
    // room in the return FIFO for one more reserved word, and in the return info: registers (timing).  rf_ok is
    // conservative by one word: free - reserved shrinks by at most one a clock (a reservation), so > 2 now means
    // > 1 next clock; ri_ok is exact (from the next pointers).
    reg  rf_ok = 1'b0, ri_ok = 1'b1;
    wire q_free  = !q_valid || q_ready;
    wire r_go    = ract && q_free && rf_ok && ri_ok;
    wire w_go    = wact && acc_full && q_free;
    wire go      = r_go || w_go;                                // (never both: ract and wact are exclusive)
    // The request's data enables (q_wdata / q_wmask / q_addr, ~320 registers) take their go from their own copies of
    // q_valid, ract, wact and acc_full (timing: w_go enabled both the data registers and,
    // through go, sa / sn; one slow route on that 242-load net failed clk_x1 by 1.4 ns at sn).  The copies are
    // syn_preserve registers (GowinSynthesis merges identical LUTs), each updated as its
    // original but from the data side's own go, and equal to it every clock (checked below in simulation); the control
    // registers above keep go, w_go and r_go, now a net of ~60 loads.
    (* syn_preserve = 1 *) reg q_valid_d = 1'b0;
    (* syn_preserve = 1 *) reg ract_d = 1'b0;
    (* syn_preserve = 1 *) reg wact_d = 1'b0;
    (* syn_preserve = 1 *) reg acc_full_d = 1'b1;
    wire q_free_d = !q_valid_d || q_ready;
    wire r_go_d   = ract_d && q_free_d && rf_ok && ri_ok;
    wire w_go_d   = wact_d && acc_full_d && q_free_d;
    // q_addr's enable (w_go_d || r_go_d) is one more LUT after w_go_d's ~290 loads (timing: a Frogger build failed
    // clk_x1 by 0.22 ns on q_ready -> w_go_d -> that LUT -> q_addr's CE).  q_addr takes
    // its go from a third set of syn_preserve copies (q_valid_a, ract_a, wact_a, acc_full_a), updated as the data side's
    // copies but from q_addr's own go, and equal to the originals every clock (checked below in simulation).
    (* syn_preserve = 1 *) reg q_valid_a = 1'b0;
    (* syn_preserve = 1 *) reg ract_a = 1'b0;
    (* syn_preserve = 1 *) reg wact_a = 1'b0;
    (* syn_preserve = 1 *) reg acc_full_a = 1'b1;
    wire q_free_a = !q_valid_a || q_ready;
    wire r_go_a   = ract_a && q_free_a && rf_ok && ri_ok;
    wire w_go_a   = wact_a && acc_full_a && q_free_a;
    // ---- next state after a go, from registers only (the go selects, below).  The IP word just sent took nb beats:
    //      what is left is sn_m; if any is left the command was cut at a 4-lane boundary (nb = 4 - first lane), so the next
    //      word starts at lane 0 and holds min(sn_m, 4) beats -- nbf(sn_m, l0 + nb) with l0 + nb = 0 (checked below).
    wire [8:0] sn_m   = sn - nb;
    wire [8:0] nb_m   = (sn_m < 9'd4) ? sn_m : 9'd4;
    wire       last_m = (sn_m <= 9'd4);                         // nb_m == sn_m
    wire [24:0] sa_m  = sa + {22'd0, nb[2:0]};                  // nb <= 4 (checked below)
    // the same for a command just read off the FIFO (cf_q: {write, address[24:0], count[7:0]})
    wire [8:0] n_c    = {1'b0, cf_q[7:0]};
    wire [1:0] lo_c   = cf_q[9:8];
    wire       last_c = (n_c <= 9'd4 - {7'd0, lo_c});           // nbf(n_c, lo_c) == n_c
    wire       nz_c   = (cf_q[7:0] != 8'd0);
    // ri_ok and rf_res: candidates from registers; the late r_go / r_valid select (the pointers' modular 6-bit arithmetic)
    wire       ric00 /* synthesis syn_keep = 1 */ = ((ri_wp - ri_rp) < 6'd31);
    wire       ric10 /* synthesis syn_keep = 1 */ = (((ri_wp + 6'd1) - ri_rp) < 6'd31);
    wire       ric01 /* synthesis syn_keep = 1 */ = ((ri_wp - (ri_rp + 6'd1)) < 6'd31);
    wire [RF_LOG2+1:0] rfr_g /* synthesis syn_keep = 1 */ = rf_res + 1'b1 - (rf_we ? 1'b1 : 1'b0);
    wire [RF_LOG2+1:0] rfr_n /* synthesis syn_keep = 1 */ = rf_res - (rf_we ? 1'b1 : 1'b0);
    integer bi;
    always @(posedge clk) begin
        cf_rd <= 1'b0; rf_we <= 1'b0;
        if (rst) begin
            busy <= 1'b0; q_valid <= 1'b0; skn <= 2'd0; rdp <= 1'b0; cf_pend <= 1'b0; acc_n <= 3'd0;
            ri_wp <= 0; ri_rp <= 0; rf_res <= 0; ract <= 1'b0; wact <= 1'b0;
            q_valid_d <= 1'b0; ract_d <= 1'b0; wact_d <= 1'b0;
            q_valid_a <= 1'b0; ract_a <= 1'b0; wact_a <= 1'b0;
            bw_l <= 4'b0000; bs_l <= 8'd0; bz_l <= 4'b1111;
        end else begin
            // the lanes' copies, updated as the originals below (cf_pend, the beat, the word's send)
            for (bi = 0; bi < 4; bi = bi + 1) begin
                if (cf_pend) begin bz_l[bi] <= 1'b1; bfl_l[2 * bi +: 2] <= lo_c; bw_l[bi] <= cf_q[33] && nz_c; bf_l[bi] <= !nz_c; end
                case ({pb_l[bi], rdp})
                    2'b10: bs_l[2 * bi +: 2] <= bs_l[2 * bi +: 2] - 1'b1;
                    2'b01: bs_l[2 * bi +: 2] <= bs_l[2 * bi +: 2] + 1'b1;
                    default: ;
                endcase
                if (pb_l[bi]) begin
                    bz_l[bi] <= 1'b0; bfl_l[2 * bi +: 2] <= bfl_l[2 * bi +: 2] + 1'b1;
                    bf_l[bi] <= ({6'd0, acc_n} + 9'd1 == nb);
                end
                if (w_go_d) begin
                    bz_l[bi] <= 1'b1; bfl_l[2 * bi +: 2] <= sa[1:0] + nb[1:0]; bf_l[bi] <= lastw;
                    if (lastw) bw_l[bi] <= 1'b0;
                end
            end
            if (q_valid && q_ready) q_valid <= 1'b0;
            // the data side's copies, updated as the originals below, from the data side's go
            if (q_valid_d && q_ready) q_valid_d <= 1'b0;
            if (cf_pend) begin ract_d <= !cf_q[33] && nz_c; wact_d <= cf_q[33] && nz_c; acc_full_d <= !nz_c; end
            if (pop_b) acc_full_d <= ({6'd0, acc_n} + 9'd1 == nb);
            if (w_go_d) begin q_valid_d <= 1'b1; acc_full_d <= lastw; if (lastw) wact_d <= 1'b0; end
            if (r_go_d) begin q_valid_d <= 1'b1; if (lastw) ract_d <= 1'b0; end
            // q_addr's copies, the same updates from q_addr's go
            if (q_valid_a && q_ready) q_valid_a <= 1'b0;
            if (cf_pend) begin ract_a <= !cf_q[33] && nz_c; wact_a <= cf_q[33] && nz_c; acc_full_a <= !nz_c; end
            if (pop_b) acc_full_a <= ({6'd0, acc_n} + 9'd1 == nb);
            if (w_go_a) begin q_valid_a <= 1'b1; acc_full_a <= lastw; if (lastw) wact_a <= 1'b0; end
            if (r_go_a) begin q_valid_a <= 1'b1; if (lastw) ract_a <= 1'b0; end
            // -- the next command
            cf_pend <= cf_rd;
            if (!busy && !cf_rd && !cf_pend && !cf_empty) cf_rd <= 1'b1;
            if (cf_pend) begin
                busy <= 1'b1; s_w <= cf_q[33]; sa <= cf_q[32:8]; sn <= n_c;
                nb <= nbf(n_c, lo_c);
                acc_n <= 3'd0; fl <= lo_c;
                ract <= !cf_q[33] && nz_c; wact <= cf_q[33] && nz_c; lastw <= last_c; acc_full <= !nz_c;
            end
            // -- the beat skid (its data: the block below)
            rdp <= want_rd;
            case ({pop_b, rdp})
                2'b10: skn <= skn - 1'b1;
                2'b01: skn <= skn + 1'b1;
                default: ;
            endcase
            // -- writes: gather the word's beats, then send it
            if (pop_b) begin
                acc_n <= acc_n + 1'b1; fl <= fl + 1'b1;
                acc_full <= ({6'd0, acc_n} + 9'd1 == nb);
            end
            if (w_go) begin
                q_valid <= 1'b1; q_write <= 1'b1;
                acc_n <= 3'd0; fl <= sa[1:0] + nb[1:0]; acc_full <= lastw;
                if (lastw) wact <= 1'b0;
            end
            // -- reads: one IP read per 32-B word, each with its return-FIFO word reserved
            if (r_go) begin
                q_valid <= 1'b1; q_write <= 1'b0;
                ri_wp <= ri_wp + 1'b1;
                if (lastw) ract <= 1'b0;
            end
            // -- the next IP word of the command (either kind)
            if (go) begin
                sa <= sa_m; sn <= sn_m; nb <= nb_m; lastw <= last_m;
                if (lastw) busy <= 1'b0;
            end
            rf_res <= r_go ? rfr_g : rfr_n;
            rf_ok  <= ({1'b0, rf_free} > rf_res + 2'd2);
            ri_ok  <= r_go ? (r_valid ? ric00 : ric10) : (r_valid ? ric01 : ric00);
            // -- returns: the word and its lanes into the return FIFO
            if (r_valid) begin
                rf_we <= 1'b1; ri_rp <= ri_rp + 1'b1;
            end
        end
    end
    // the data path: no reset (the controls above qualify it), so the reset gates none of these enables (timing)
    always @(posedge clk) begin
        case ({pop_b, rdp})
            2'b11: begin if (skn == 2'd1) sk0 <= wf_q; else begin sk0 <= sk1; sk1 <= wf_q; end end
            2'b10: sk0 <= sk1;
            2'b01: begin if (skn == 2'd0) sk0 <= wf_q; else sk1 <= wf_q; end
            default: ;
        endcase
        // the write mask is cleared lazily: a word's first beat (acc_n == 0) writes its own lane and
        // FF into the other lanes; later beats write their own lane only.  acc_n is 0 after rst, cf_pend and w_go, and every
        // w_go has acc_n == nb >= 1, so q_wmask <= acc_m loads the same mask as an explicit clear at cf_pend / w_go would.
        // acc_m's cone is pop_b, acc_n, fl and sk0 (registers only): the late q_ready / w_go is no part of it.
        // (each lane from its own copies: pb_l = pop_b, bfl_l = fl, bz_l = acc_n == 0)
        for (ab = 0; ab < 4; ab = ab + 1) begin
            if (pb_l[ab] && (bfl_l[2 * ab +: 2] == ab[1:0])) acc_m[8 * ab +: 8] <= ~sk0[71:64];
            else if (pb_l[ab] && bz_l[ab]) acc_m[8 * ab +: 8] <= 8'hFF;
            if (pb_l[ab] && (bfl_l[2 * ab +: 2] == ab[1:0])) acc_d[64 * ab +: 64] <= sk0[63:0];
        end
        if (w_go_d) begin q_wdata <= acc_d; q_wmask <= acc_m; end
        if (w_go_a || r_go_a) q_addr <= {1'b0, 2'b11, sa[24:2], 3'b000};
        // the return info: the slot at ri_wp is the next free one (ri_ok keeps at most 31 of the 32 held), so it is written
        // EVERY clock with the current word's lanes; ri_wp only moves on r_go, so the slot a read takes is exactly the one
        // written in its r_go clock, and r_go (a late, wide signal) is no part of the 32-way write enable
        ri[ri_wp[4:0]] <= {l0, l1};
        if (r_valid) rf_wd <= {ri[ri_rp[4:0]], r_data};
    end
    // The state the flags stand for, and the precondition of the unconditional ri write, checked in simulation every clock
    // (the form of the arbiter's contract checks: GowinSynthesis skips the code between the pragma comments, Verilator
    // compiles it).  A difference prints "TANGO CONTRACT: DDR3 cport ..." and stops the run; the end of a run prints the
    // clocks checked.
    /* synthesis translate_off */
    integer cpc_clocks = 0;
    wire [5:0] ri_cnt = ri_wp - ri_rp;
    always @(posedge clk) if (!rst) begin
        if ((ract !== (busy && !s_w && sn != 9'd0)) || (wact !== (busy && s_w && sn != 9'd0)) || (busy && (lastw !== (sn == nb)))
            || (busy && (acc_full !== ({6'd0, acc_n} == nb))) || (busy && ({6'd0, acc_n} > nb)) || (nb > 9'd4)) begin
            $display("TANGO CONTRACT: DDR3 cport flags differ at %0t (%m): ract %b wact %b lastw %b acc_full %b | busy %b s_w %b sn %0d nb %0d acc_n %0d",
                     $time, ract, wact, lastw, acc_full, busy, s_w, sn, nb, acc_n);
            $fatal(1, "TANGO CONTRACT: DDR3 cport flags violated (see the line above)");
        end
        // nb_m / last_m are nbf of the general form whenever the command goes on: the next word starts at lane 0
        if (go && ((nb_m !== nbf(sn_m, sa[1:0] + nb[1:0])) || (last_m !== (nbf(sn_m, sa[1:0] + nb[1:0]) == sn_m)))) begin
            $display("TANGO CONTRACT: DDR3 cport next word differs at %0t (%m): sn %0d nb %0d sa[1:0] %0d", $time, sn, nb, sa[1:0]);
            $fatal(1, "TANGO CONTRACT: DDR3 cport next word violated (see the line above)");
        end
        // the data side's copies equal the control side's registers (so w_go_d / r_go_d equal w_go / r_go)
        if (q_valid_d !== q_valid || ract_d !== ract || wact_d !== wact || acc_full_d !== acc_full
            || q_valid_a !== q_valid || ract_a !== ract || wact_a !== wact || acc_full_a !== acc_full) begin
            $display("TANGO CONTRACT: DDR3 cport data-side copies differ at %0t (%m): q_valid %b/%b/%b ract %b/%b/%b wact %b/%b/%b acc_full %b/%b/%b",
                     $time, q_valid, q_valid_d, q_valid_a, ract, ract_d, ract_a, wact, wact_d, wact_a, acc_full, acc_full_d, acc_full_a);
            $fatal(1, "TANGO CONTRACT: DDR3 cport data-side copies violated (see the line above)");
        end
        // the lanes' copies equal the originals
        if (bw_l !== {4{wact}} || bf_l !== {4{acc_full}} || bs_l !== {4{skn}} || bfl_l !== {4{fl}} || bz_l !== {4{acc_n == 3'd0}}) begin
            $display("TANGO CONTRACT: DDR3 cport lane copies differ at %0t (%m): wact %b/%b acc_full %b/%b skn %0d/%b fl %0d/%b acc_n %0d/%b",
                     $time, wact, bw_l, acc_full, bf_l, skn, bs_l, fl, bfl_l, acc_n, bz_l);
            $fatal(1, "TANGO CONTRACT: DDR3 cport lane copies violated (see the line above)");
        end
        // the lazy acc_m clear needs a beat in every word that is sent (acc_m is read only at w_go)
        if (w_go && acc_n == 3'd0) begin
            $display("TANGO CONTRACT: DDR3 cport w_go with no beat in the word at %0t (%m): nb %0d", $time, nb);
            $fatal(1, "TANGO CONTRACT: DDR3 cport lazy mask clear violated (see the line above)");
        end
        // the return-info ring: at most 31 of 32 slots held (so ri_wp's slot is free to overwrite), a return only for a held one
        if (ri_cnt > 6'd31 || (r_valid && ri_cnt == 6'd0)) begin
            $display("TANGO CONTRACT: DDR3 cport return-info ring at %0t (%m): wp %0d rp %0d r_valid %b", $time, ri_wp, ri_rp, r_valid);
            $fatal(1, "TANGO CONTRACT: DDR3 cport return-info ring violated (see the line above)");
        end
        cpc_clocks <= cpc_clocks + 1;
    end
    final
        $display("TANGO CONTRACT OK: DDR3 cport flags, data-side copies, lane copies, next word, lazy mask clear, return-info ring in %0d clocks (%m)", cpc_clocks);
    /* synthesis translate_on */
endmodule
`default_nettype wire
