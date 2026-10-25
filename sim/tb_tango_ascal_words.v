// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_tango_ascal_words -- tango_ascal_words (TANGO_ASCAL_EXT's helper) between an ascal-exact
// Avalon master and a 16-bit word memory that takes words at random and returns reads in order after random
// latencies, as a core's SDRAM controller that also serves its own CPUs would.
// The master is ascal.vhd's Avaloir process (ascal.vhd 1686-1733, 1752-1758; as the Template core's SDRAM bench):
//   write: the address in sIDLE; avl_write rises the next clock with beat 0; a beat is taken at an edge with
//     avl_write = 1 and avl_waitrequest = 0, and the next beat is on the bus the clock after; avl_write falls after the
//     last beat;
//   read: avl_read rises with the address and falls when taken; the beats follow on avl_readdatavalid, and the next
//     command may be taken before they have all come back (up to 4 reads outstanding here).
// 64 burst slots over all 4 banks and several rows (byte address bits 24:8); phase 1 writes every slot, phase 2 runs
// random reads and writes with random gaps.  The word memory:
//   w_take  random every clock (WTAKE_PCT % of clocks), with stalls of up to 63 clocks now and then (1 in 64);
//   reads   the word as it is at the take, returned in order RLAT_MIN.. RLAT_MIN+15 clocks later, r_valid itself
//           gapped at random.
// -DRANDOM_BE: writes in phase 2 use random byte enables half the time (legal Avalon; ascal drives all ones).
// -DRANDOM_LEN: bursts of 1..32 beats (ascal's are 32).
// Checks: every read beat = the shadow ({generation, slot, beat} pattern and its complement, merged by byte enable) as
// it was when the read was taken (the module serves one burst at a time, so a later write cannot reach the memory first);
// every word access at the address and in the direction of its burst; no read beats beyond the reads taken; every
// read complete.  TB_RESULT PASS / FAIL.
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_tango_ascal_words;
    localparam integer NSLOT = 64, NOPS = 20000, BLEN = 32;
`ifndef WTAKE_PCT
`define WTAKE_PCT 60
`endif
`ifndef RLAT_MIN
`define RLAT_MIN 2
`endif

    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg [3:0] rst_cnt = 4'd0;
    wire      rst = (rst_cnt != 4'd8);
    always @(posedge clk) if (rst) rst_cnt <= rst_cnt + 4'd1;

    // ---- DUT
    reg  [28:0] avl_address = 29'd0;
    reg         avl_write = 1'b0, avl_read = 1'b0;
    reg  [63:0] avl_writedata = 64'd0;
    reg  [7:0]  avl_burstcount = BLEN[7:0], avl_byteenable = 8'hFF;
    wire [63:0] avl_readdata;
    wire        avl_readdatavalid, avl_waitrequest;
    wire        w_valid, w_we, busy;
    wire [24:1] w_addr;
    wire [15:0] w_data;
    wire [1:0]  w_be;
    reg         w_take = 1'b0, r_valid = 1'b0;
    reg  [15:0] r_data = 16'd0;
    tango_ascal_words dut (
        .clk(clk), .rst(rst),
        .avl_address(avl_address), .avl_write(avl_write), .avl_writedata(avl_writedata), .avl_read(avl_read),
        .avl_burstcount(avl_burstcount), .avl_byteenable(avl_byteenable),
        .avl_readdata(avl_readdata), .avl_readdatavalid(avl_readdatavalid), .avl_waitrequest(avl_waitrequest),
        .w_valid(w_valid), .w_we(w_we), .w_addr(w_addr), .w_data(w_data), .w_be(w_be), .w_take(w_take),
        .r_valid(r_valid), .r_data(r_data), .busy(busy));

    reg [31:0] lfsr = 32'hACE1_2345, lfsr2 = 32'h1357_9BDF;
    reg [31:0] cyc = 32'd0;

    // ---- the word memory (byte address [24:1] = 16 M words)
    reg  [15:0] mem [0:(1 << 24) - 1];
    reg  [5:0]  stall = 6'd0;
    // read words in flight: {data, due cycle}
    reg  [15:0] rf_d   [0:255];
    reg  [31:0] rf_due [0:255];
    reg  [7:0]  rf_wp = 8'd0, rf_rp = 8'd0;
    reg  [31:0] n_wtake = 32'd0, n_stallclk = 32'd0, max_rf = 32'd0;
    wire        took = w_valid && w_take;
    always @(posedge clk) begin
        lfsr2 <= {lfsr2[30:0], lfsr2[31] ^ lfsr2[21] ^ lfsr2[1] ^ lfsr2[0]};
        // w_take: random, and now and then a long stall
        if (stall != 6'd0) begin stall <= stall - 6'd1; w_take <= 1'b0; n_stallclk <= n_stallclk + 32'd1; end
        else if (lfsr2[13:8] == 6'd0) begin stall <= lfsr2[19:14]; w_take <= 1'b0; end
        else w_take <= ({25'd0, lfsr2[6:0]} % 32'd100) < `WTAKE_PCT;
        if (took) begin
            n_wtake <= n_wtake + 32'd1;
            if (w_we) begin
                if (w_be[0]) mem[w_addr][7:0]  <= w_data[7:0];
                if (w_be[1]) mem[w_addr][15:8] <= w_data[15:8];
            end else begin
                rf_d[rf_wp]   <= mem[w_addr];
                rf_due[rf_wp] <= cyc + `RLAT_MIN + {28'd0, lfsr2[23:20]};
                rf_wp <= rf_wp + 8'd1;
            end
        end
        // in-order returns, gapped at random
        r_valid <= 1'b0;
        if (rf_rp != rf_wp && rf_due[rf_rp] <= cyc && lfsr2[25:24] != 2'd0) begin
            r_valid <= 1'b1;  r_data <= rf_d[rf_rp];  rf_rp <= rf_rp + 8'd1;
        end
        if ({24'd0, rf_wp - rf_rp} > max_rf) max_rf <= {24'd0, rf_wp - rf_rp};
    end
    // the due cycles are not monotonic (random latency): a word waits for the ones before it -- in order, as required

    // ---- slots: bank s[1:0], 256-byte block s[3:2] of the row, row from s[5:4] (as the Template core's SDRAM bench)
    function [31:0] slot_addr(input [5:0] s);
        reg [12:0] r;
        begin
            case (s[5:4]) 2'd0: r = 13'h1000; 2'd1: r = 13'd5; 2'd2: r = 13'd1116; default: r = 13'd8191; endcase
            slot_addr = {7'd0, s[1:0], r, s[3:2], 8'd0};
        end
    endfunction
    function [63:0] pat(input [15:0] g, input [5:0] s, input [4:0] i);
        pat = {g, s, i, 5'b10110, ~{g, s, i, 5'b10110}};
    endfunction
    function [63:0] bemask(input [7:0] be);
        integer k;
        begin for (k = 0; k < 8; k = k + 1) bemask[8 * k +: 8] = be[k] ? 8'hFF : 8'h00; end
    endfunction

    reg [63:0] shadow [0:NSLOT * BLEN - 1];
    reg [15:0] gen    [0:NSLOT - 1];
    integer n;
    initial for (n = 0; n < NSLOT; n = n + 1) gen[n] = 16'd0;

    // ---- the master (ascal's Avaloir)
    localparam [2:0] M_DEC = 3'd0, M_WADR = 3'd1, M_WBEAT = 3'd2, M_RREQ = 3'd3, M_GAP = 3'd4, M_DRAIN = 3'd5;
    reg [2:0]  m_st = M_DEC;
    reg [5:0]  m_slot = 6'd0, m_beat = 6'd0, m_len = 6'd32;
    reg [15:0] m_gen = 16'd0;
    reg [7:0]  m_gapc = 8'd0;
    reg [31:0] ops = 32'd0, n_wops = 32'd0, n_rops = 32'd0, n_rbeats_want = 32'd0, last_rv = 32'd0;
    reg [31:0] d_err = 32'd0, a_err = 32'd0, extra_rv = 32'd0, n_rv = 32'd0, max_wait = 32'd0, wait_cnt = 32'd0;
    reg [31:0] n_partial = 32'd0;
    // outstanding reads
    reg [5:0]  rq_slot [0:3];
    reg [5:0]  rq_len  [0:3];
    reg [63:0] rq_exp  [0:127];                      // the beats each outstanding read must return: {queue entry, beat}
    integer    b;
    reg [1:0]  rq_wp = 2'd0, rq_rp = 2'd0;
    reg [2:0]  rq_n = 3'd0;
    reg [5:0]  rq_beat = 6'd0;
    wire       rq_last = avl_readdatavalid && rq_n != 3'd0 && rq_beat == rq_len[rq_rp] - 6'd1;

    // the expected word accesses of the burst being served: base, index, direction (taken from the command)
    reg [31:0] x_base = 32'd0;  reg [9:0] x_idx = 10'd0, x_last = 10'd0;  reg x_we = 1'b0;  reg x_live = 1'b0;
    wire       cmd_rd = avl_read && !avl_waitrequest;
    wire       cmd_wr = avl_write && !avl_waitrequest && m_st == M_WBEAT && m_beat == 6'd0;

    always @(posedge clk) begin
        cyc  <= cyc + 32'd1;
        lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};

        if ((avl_read || avl_write) && avl_waitrequest) wait_cnt <= wait_cnt + 32'd1;
        else begin if (wait_cnt > max_wait) max_wait <= wait_cnt; wait_cnt <= 32'd0; end

        case (m_st)
        M_DEC: if (!rst) begin
            if (ops == NOPS) m_st <= M_DRAIN;
            else begin
`ifdef RANDOM_LEN
                m_len <= (ops < NSLOT) ? 6'd32 : ({1'b0, lfsr[28:24]} + 6'd1);
`else
                m_len <= 6'd32;
`endif
                if (ops < NSLOT || lfsr[0]) begin m_slot <= (ops < NSLOT) ? ops[5:0] : lfsr[13:8]; m_st <= M_WADR; end
                else begin m_slot <= lfsr[13:8]; m_st <= M_RREQ; end
            end
        end
        M_WADR: begin                                         // sIDLE: the address, no write yet
            avl_address    <= slot_addr(m_slot) >> 3;
            avl_burstcount <= {2'd0, m_len};
            m_gen          <= gen[m_slot] + 16'd1;
            avl_writedata  <= pat(gen[m_slot] + 16'd1, m_slot, 5'd0);
`ifdef RANDOM_BE
            avl_byteenable <= (ops >= NSLOT && lfsr[1]) ? lfsr[31:24] : 8'hFF;
`endif
            m_beat         <= 6'd0;
            m_st           <= M_WBEAT;
        end
        M_WBEAT: begin                                        // sWRITE
            avl_write <= 1'b1;
            if (avl_write && !avl_waitrequest) begin
                shadow[{m_slot, m_beat[4:0]}] <= (shadow[{m_slot, m_beat[4:0]}] & ~bemask(avl_byteenable)) |
                                                 (avl_writedata & bemask(avl_byteenable));
                if (avl_byteenable != 8'hFF) n_partial <= n_partial + 32'd1;
`ifdef RANDOM_BE
                if (avl_byteenable != 8'hFF) avl_byteenable <= lfsr[31:24];
`endif
                if (m_beat == m_len - 6'd1) begin
                    avl_write <= 1'b0;
                    gen[m_slot] <= m_gen;
                    ops <= ops + 32'd1;  n_wops <= n_wops + 32'd1;
                    m_gapc <= lfsr[17:16] == 2'd0 ? 8'd0 : {3'd0, lfsr[22:18]};
                    m_st <= M_GAP;
                end
                m_beat <= m_beat + 6'd1;
                avl_writedata <= pat(m_gen, m_slot, m_beat[4:0] + 5'd1);
            end
        end
        M_RREQ: if (rq_n < 3'd4 || rq_last) begin             // sREAD (this master keeps at most 4 outstanding)
            avl_address    <= slot_addr(m_slot) >> 3;
            avl_burstcount <= {2'd0, m_len};
            avl_read       <= 1'b1;
            if (avl_read && !avl_waitrequest) begin
                avl_read <= 1'b0;
                rq_slot[rq_wp] <= m_slot;  rq_len[rq_wp] <= m_len;  rq_wp <= rq_wp + 2'd1;
                for (b = 0; b < BLEN; b = b + 1) rq_exp[{rq_wp, b[4:0]}] <= shadow[{m_slot, b[4:0]}];
                ops <= ops + 32'd1;  n_rops <= n_rops + 32'd1;  n_rbeats_want <= n_rbeats_want + {26'd0, m_len};
                m_gapc <= lfsr[17:16] == 2'd0 ? 8'd0 : {3'd0, lfsr[22:18]};
                m_st <= M_GAP;
            end
        end
        M_GAP: if (m_gapc == 8'd0) m_st <= M_DEC; else m_gapc <= m_gapc - 8'd1;
        M_DRAIN: if ((rq_n == 3'd0 && !busy) || cyc - last_rv > 32'd5000) begin
            $display("TB ops=%0d writes=%0d reads=%0d read_beats=%0d (want %0d) cycles=%0d partial_beats=%0d",
                     ops, n_wops, n_rops, n_rv, n_rbeats_want, cyc, n_partial);
            $display("TB words taken=%0d stall_clocks=%0d max_reads_in_flight=%0d max_waitrequest=%0d",
                     n_wtake, n_stallclk, max_rf, max_wait);
            $display("TB data_errors=%0d address_errors=%0d extra_beats=%0d reads_outstanding=%0d",
                     d_err, a_err, extra_rv, rq_n);
            if (d_err == 0 && a_err == 0 && extra_rv == 0 && rq_n == 0 && n_rv == n_rbeats_want && !busy)
                $display("TB_RESULT PASS");
            else
                $display("TB_RESULT FAIL");
            $finish;
        end
        default: m_st <= M_DEC;
        endcase

        if (m_st == M_RREQ && avl_read && !avl_waitrequest) begin
            if (!rq_last) rq_n <= rq_n + 3'd1;
        end else if (rq_last) rq_n <= rq_n - 3'd1;

        // ---- returned beats
        if (avl_readdatavalid) begin
            n_rv <= n_rv + 32'd1;  last_rv <= cyc;
            if (rq_n == 3'd0) extra_rv <= extra_rv + 32'd1;
            else begin
                if (avl_readdata !== rq_exp[{rq_rp, rq_beat[4:0]}]) begin
                    if (d_err < 32'd10)
                        $display("TB cycle %0d: slot %0d beat %0d got %016x want %016x", cyc, rq_slot[rq_rp],
                                 rq_beat, avl_readdata, rq_exp[{rq_rp, rq_beat[4:0]}]);
                    d_err <= d_err + 32'd1;
                end
                if (rq_last) begin rq_beat <= 6'd0;  rq_rp <= rq_rp + 2'd1; end
                else rq_beat <= rq_beat + 6'd1;
            end
        end

        // ---- every word access: the burst's next address, in its direction
        if (cmd_rd || cmd_wr) begin
            if (x_live && a_err < 32'd10) $display("TB cycle %0d: a command was taken inside a burst", cyc);
            if (x_live) a_err <= a_err + 32'd1;
            x_base <= {avl_address, 3'b000};  x_idx <= 10'd0;  x_we <= cmd_wr;  x_live <= 1'b1;
            x_last <= {avl_burstcount, 2'b00} - 10'd1;
        end
        if (took) begin
            if (!x_live || w_we != x_we || {7'd0, w_addr, 1'b0} != x_base + {21'd0, x_idx, 1'b0}) begin
                if (a_err < 32'd10)
                    $display("TB cycle %0d: word access %s at %07x, want %s at %07x (live %0d)", cyc, w_we ? "W" : "R",
                             {w_addr, 1'b0}, x_we ? "W" : "R", x_base + {21'd0, x_idx, 1'b0}, x_live);
                a_err <= a_err + 32'd1;
            end
            x_idx <= x_idx + 10'd1;
            if (x_idx == x_last && !(cmd_rd || cmd_wr)) x_live <= 1'b0;
        end

        if (cyc == 32'd40_000_000) begin $display("TB_RESULT FAIL (timeout, m_st=%0d)", m_st); $finish; end
    end
endmodule
`default_nettype wire
