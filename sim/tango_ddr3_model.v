// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_model -- a behavioural model of the Gowin DDR3 Memory Interface IP's user interface (sims only).
// Our configuration of the IP: 1:4 clock ratio, x32 DDR3-800, so one command is one BL8 burst of
// one 256-bit word, and the user clock (clk_out) is 100 MHz.  What the model does, and where each behaviour comes
// from:
//   - init_calib_complete rises INIT_CLKS clocks after rst_n (IPUG281-2.6E 4.3.1, 4.4.1);
//   - cmd / cmd_en / cmd_ready: a command is taken when cmd_en and cmd_ready are both high; read = 3'b001,
//     write = 3'b000 (4.4.5, Table 4-1).  Commands queue in a FIFO of CMD_DEPTH; cmd_ready is low while it is full;
//   - write data: wr_data_en / wr_data_end / wr_data_rdy; one word per write command (4.4.7, 1:4); a FIFO of
//     WDATA_DEPTH.  A write command waits at the head of the queue until its data is there;
//   - in order: commands execute in the order taken (4.4.5) and reads return in that order (4.4.8);
//   - rd_data has no ready: a read returns RD_LAT (+ 0..RD_JITTER) clocks after it executes, one word per clock;
//   - the address is {rank, bank[2:0], row[14:0], col[9:0]} in 32-bit DQ words (Figure 4-8, our ROW 15 / COL 10);
//     one open row per bank: a command to another row of its bank costs T_ROWMISS clocks more, one changing
//     direction T_TURN more; a BL8 takes T_BL clocks;
//   - a read whose col[2:0] is not 0 returns the burst in DDR3's sequential BL8 order (JESD79-3 Table 4-?:
//     start 1 -> 1,2,3,0,5,6,7,4; 2 -> 2,3,0,1,6,7,4,5; 4 -> 4,5,6,7,0,1,2,3; ...) when ADDR_LOW_PASS = 1, as the
//     IP passes addr to the DDR3 ("addr directly reflects the DDR memory address", 4.4.5); writes ignore col[2:0];
//   - the user word: beat k of the burst (column base + k) is bits [32k+31:32k] -- the model's convention, which the
//     DDR3 bring-up test's (ddr3/test) order test checks on silicon;
//   - wr_data_mask bit b = 1 masks byte b ("1: The corresponding wr_data byte mask is valid", Table 6-1);
//   - auto refresh every REF_PERIOD clocks: after the command in progress, the executor stops for REF_LEN clocks and
//     every row closes ("MC will precharge all banks after executing the last read/write command, and then execute
//     Refresh command. The refresh operation has higher priority", 4.4.9);
//   - STALL_PERIOD / STALL_LEN: extra stalls of the executor (pll_stop / calibration events);
//   - fault injection for the bring-up test checker's sims: reads of word fault_word return data ^ fault_xor;
//   - force_stall (cmd_ready and wr_data_rdy low, the executor stopped: a pll_stop-like event) and ref_burst (8
//     refreshes back to back: a postponed-refresh catch-up) for the benches' stall patterns.
// Every number is a parameter: the bring-up test measures the real ones on silicon (sim/tango_ddr3_silicon.vh).
// Storage: 2^MEM_LOG2 words of 256 bits, indexed by the word address modulo that (aliasing beyond).
//==============================================================================
`default_nettype none
module tango_ddr3_model #(
    parameter integer MEM_LOG2      = 14,
    parameter integer INIT_CLKS     = 300,
    parameter integer CMD_DEPTH     = 32,
    parameter integer WDATA_DEPTH   = 32,
    parameter integer RD_LAT        = 18,
    parameter integer RD_JITTER     = 0,
    parameter integer T_BL          = 1,
    parameter integer T_ROWMISS     = 3,
    parameter integer T_TURN        = 2,
    parameter integer REF_PERIOD    = 780,
    parameter integer REF_LEN       = 28,
    parameter integer STALL_PERIOD  = 0,
    parameter integer STALL_LEN     = 0,
    parameter integer ADDR_LOW_PASS = 1
) (
    input  wire         clk,
    input  wire         rst_n,
    output reg          init_calib_complete = 1'b0,
    input  wire [2:0]   cmd,
    input  wire         cmd_en,
    input  wire [28:0]  addr,
    output wire         cmd_ready,
    input  wire [255:0] wr_data,
    input  wire         wr_data_en,
    input  wire         wr_data_end,
    input  wire [31:0]  wr_data_mask,
    output wire         wr_data_rdy,
    output reg  [255:0] rd_data = 256'd0,
    output reg          rd_data_valid = 1'b0,
    output reg          rd_data_end = 1'b0,
    input  wire         force_stall,          // benches: cmd_ready / wr_data_rdy low and the executor stopped
    input  wire         ref_burst,            // benches: a pulse queues 8 refreshes back to back
    input  wire         fault_en,
    input  wire [24:0]  fault_word,
    input  wire [255:0] fault_xor,
    // observation (benches)
    output reg  [31:0]  n_refresh = 32'd0,
    output reg  [31:0]  n_rowmiss = 32'd0,
    output reg  [31:0]  n_bad_cmd = 32'd0
);
    // ---- init
    integer init_cnt = 0;
    always @(posedge clk) begin
        if (!rst_n) begin init_cnt <= 0; init_calib_complete <= 1'b0; end
        else if (init_cnt < INIT_CLKS) init_cnt <= init_cnt + 1;
        else init_calib_complete <= 1'b1;
    end

    // ---- queues
    reg [31:0]  cq [0:CMD_DEPTH-1];          // {is_write, addr[28:0]} (bits 31:30 spare)
    integer     cq_wr = 0, cq_rd = 0, cq_n = 0;
    reg [287:0] wq [0:WDATA_DEPTH-1];        // {mask[31:0], data[255:0]}
    integer     wq_wr = 0, wq_rd = 0, wq_n = 0;
    assign cmd_ready   = init_calib_complete && (cq_n < CMD_DEPTH) && !force_stall;
    assign wr_data_rdy = init_calib_complete && (wq_n < WDATA_DEPTH) && !force_stall;
    integer ref_owed = 0;

    // ---- memory
    reg [255:0] mem [0:(1<<MEM_LOG2)-1];
    integer i;
    initial for (i = 0; i < (1<<MEM_LOG2); i = i + 1) mem[i] = 256'd0;

    // ---- return queue (in order): {due time, data}
    localparam integer RQ = 256;
    reg [255:0] rq_d [0:RQ-1];
    longint     rq_t [0:RQ-1];
    integer     rq_wr = 0, rq_rd = 0, rq_n = 0;

    // ---- executor state
    longint now = 0;
    longint busy_until = 0;
    longint next_ref = REF_PERIOD;
    longint next_stall = STALL_PERIOD;
    reg [14:0]  open_row [0:7];
    reg [7:0]   row_open = 8'd0;
    reg         last_write = 1'b0;

    function automatic [255:0] burst_order(input [255:0] w, input [2:0] c);
        integer k, src;
        begin
            for (k = 0; k < 8; k = k + 1) begin
                // JESD79-3 sequential BL8: the first four beats are (c + k) mod 4 within the half of c[2], then the
                // other half in the same order
                src = (k < 4) ? ({c[2], 2'b00} | ((c[1:0] + k) & 3)) : ({~c[2], 2'b00} | ((c[1:0] + k - 4) & 3));
                burst_order[32*k +: 32] = w[32*src +: 32];
            end
        end
    endfunction

    reg [31:0]  h;          // head command
    reg [287:0] hw;
    reg [2:0]   hb;
    reg [14:0]  hr;
    reg [24:0]  hwi;
    integer     cost, b;
    reg [255:0] rdw;
    reg         take_c, take_w, ex_rd, ex_wr, ret;

    always @(posedge clk) begin
        now <= now + 1;
        rd_data_valid <= 1'b0; rd_data_end <= 1'b0;
        if (!rst_n) begin
            cq_wr <= 0; cq_rd <= 0; cq_n <= 0; wq_wr <= 0; wq_rd <= 0; wq_n <= 0;
            rq_wr <= 0; rq_rd <= 0; rq_n <= 0; busy_until <= 0; row_open <= 8'd0;
            next_ref <= now + REF_PERIOD; next_stall <= now + STALL_PERIOD;
        end else begin
            take_c = cmd_en && cmd_ready;
            take_w = wr_data_en && wr_data_rdy;
            ex_rd = 1'b0; ex_wr = 1'b0; ret = 1'b0;
            // -- take a command / write data (the user's handshake)
            if (cmd_en && !cmd_ready) n_bad_cmd <= n_bad_cmd + 1;       // the IP would drop it: a user bug
            if (take_c) begin
                if (cmd != 3'b000 && cmd != 3'b001) n_bad_cmd <= n_bad_cmd + 1;
                cq[cq_wr] <= {2'b00, (cmd == 3'b000), addr};
                cq_wr <= (cq_wr + 1) % CMD_DEPTH;
            end
            if (take_w) begin
                wq[wq_wr] <= {wr_data_mask, wr_data};
                wq_wr <= (wq_wr + 1) % WDATA_DEPTH;
            end
            if (ref_burst) ref_owed = ref_owed + 8;
            // -- refresh and stalls block the executor once the command in progress is done
            if (force_stall) begin
                if (busy_until <= now) busy_until <= now + 1;
            end else if (ref_owed > 0 && now >= busy_until) begin
                busy_until <= now + REF_LEN; row_open <= 8'd0; n_refresh <= n_refresh + 1; ref_owed = ref_owed - 1;
            end else if (now >= next_ref && now >= busy_until) begin
                busy_until <= now + REF_LEN;
                next_ref   <= next_ref + REF_PERIOD;
                row_open   <= 8'd0;                     // precharge all
                n_refresh  <= n_refresh + 1;
            end else if (STALL_PERIOD > 0 && now >= next_stall && now >= busy_until) begin
                busy_until <= now + STALL_LEN;
                next_stall <= next_stall + STALL_PERIOD;
            end else if (now >= busy_until && cq_n > 0) begin
                // -- execute the head command when it is complete (a write needs its data)
                h = cq[cq_rd];
                if (!h[29] || wq_n > 0) begin
                    hb  = h[27:25]; hr = h[24:10]; hwi = h[27:3];
                    cost = T_BL;
                    if (!row_open[hb] || open_row[hb] != hr) begin cost = cost + T_ROWMISS; n_rowmiss <= n_rowmiss + 1; end
                    if (last_write != h[29]) cost = cost + T_TURN;
                    open_row[hb] <= hr; row_open[hb] <= 1'b1; last_write <= h[29];
                    busy_until <= now + cost;
                    cq_rd <= (cq_rd + 1) % CMD_DEPTH;
                    if (h[29]) begin
                        ex_wr = 1'b1;
                        hw = wq[wq_rd];
                        wq_rd <= (wq_rd + 1) % WDATA_DEPTH;
                        for (b = 0; b < 32; b = b + 1)
                            if (!hw[256 + b]) mem[hwi[MEM_LOG2-1:0]][8*b +: 8] <= hw[8*b +: 8];
                    end else begin
                        ex_rd = 1'b1;
                        rdw = mem[hwi[MEM_LOG2-1:0]];
                        if (ADDR_LOW_PASS != 0) rdw = burst_order(rdw, h[2:0]);
                        if (fault_en && hwi == fault_word) rdw = rdw ^ fault_xor;
                        rq_d[rq_wr] <= rdw;
                        rq_t[rq_wr] <= now + cost + RD_LAT + ((RD_JITTER > 0) ? ($urandom % (RD_JITTER + 1)) : 0);
                        rq_wr <= (rq_wr + 1) % RQ;
                    end
                end
            end
            // -- returns, in order, one per clock
            if (rq_n > 0 && now >= rq_t[rq_rd]) begin
                ret = 1'b1;
                rd_data <= rq_d[rq_rd]; rd_data_valid <= 1'b1; rd_data_end <= 1'b1;
                rq_rd <= (rq_rd + 1) % RQ;
            end
            cq_n <= cq_n + (take_c ? 1 : 0) - ((ex_rd || ex_wr) ? 1 : 0);
            wq_n <= wq_n + (take_w ? 1 : 0) - (ex_wr ? 1 : 0);
            rq_n <= rq_n + (ex_rd ? 1 : 0) - (ret ? 1 : 0);
        end
    end
endmodule
`default_nettype wire
