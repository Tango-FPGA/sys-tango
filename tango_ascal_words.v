// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ascal_words -- ascal's Avalon MM master (TANGO_ASCAL_EXT, tango_cfg.vh) served as 16-bit word accesses, for a
// core whose own SDRAM controller also serves the scaler's frame buffers.  One clock: the core's
// memory clock, which is ascal's avl_clk (tango_core_ports.vh ASCAL_AVL_CLK).
//
// The Avalon side is ascal's master as ascal.vhd 1686-1760 drives it:
//   - one command at a time; avl_address is a 64-bit word address (byte address {avl_address, 3'b000});
//   - a read: avl_read with the address and avl_burstcount (ascal: 32 beats = 256 B, 256-B aligned), held until taken;
//     the beats come back in order on avl_readdata / avl_readdatavalid, one per clock when ready, and may still be
//     arriving after the next command was taken;
//   - a write: avl_write with the address, avl_burstcount and beat 0, held until taken; then the later beats, each held
//     until taken; avl_byteenable bit i enables byte i of the beat (ascal drives all ones, ascal.vhd 1758);
//   - avl_waitrequest high = the command or beat on the port is not taken at this clock.  Registered here: low means a
//     command or beat present in this clock is taken.
// Word 0 of a beat is its bits [15:0], the lowest address; word k is bits [16k+15:16k] with byte enables [2k+1:2k].
//
// The word side, in order:
//   w_valid      a word access is ready: w_we (1 write, 0 read), w_addr (the 16-bit word's address, byte address bits
//                [24:1]), w_data / w_be (writes);
//   w_take       the controller takes it at this clock (it may hold w_take low as long as it likes: this module waits);
//   r_valid      a read word comes back, r_data, in the order the reads were taken (the controller's job).
// A read burst's word accesses are all offered before the next command is taken, so the controller sees one burst at a
// time; its read words may still be returning while the next burst's accesses are offered (a write burst then: the
// controller keeps DQ turnaround, as it does for its own accesses).  A write burst's beats wait in a two-beat buffer.
// The module never checks addresses: the window (tango_cfg.vh RAMBASE / RAMSIZE) and the one-row bursts are ascal's.
//==============================================================================
`default_nettype none
module tango_ascal_words (
    input  wire        clk,
    input  wire        rst,                 // synchronous to clk
    // ascal's Avalon master
    input  wire [28:0] avl_address,
    input  wire        avl_write,
    input  wire [63:0] avl_writedata,
    input  wire        avl_read,
    input  wire [7:0]  avl_burstcount,
    input  wire [7:0]  avl_byteenable,
    output reg  [63:0] avl_readdata = 64'd0,
    output reg         avl_readdatavalid = 1'b0,
    output reg         avl_waitrequest = 1'b1,
    // word accesses to the core's controller
    output wire        w_valid,
    output wire        w_we,
    output wire [24:1] w_addr,
    output wire [15:0] w_data,
    output wire [1:0]  w_be,
    input  wire        w_take,
    input  wire        r_valid,
    input  wire [15:0] r_data,
    // status (sims)
    output wire        busy                 // a burst's words are still being offered
);
    localparam [1:0] S_IDLE = 2'd0, S_RD = 2'd1, S_WR = 2'd2;
    reg  [1:0]  st = S_IDLE;
    reg  [28:0] base = 29'd0;               // the burst's first 64-bit word
    reg  [9:0]  widx = 10'd0;               // the next word access of the burst (0 .. 4 x burstcount - 1)
    reg  [9:0]  wlast = 10'd0;              // its last
    reg  [8:0]  nbeats = 9'd0, beats_in = 9'd0;
    // the write beats: a two-entry ring of {byteenable, data}
    reg  [71:0] wb0 = 72'd0, wb1 = 72'd0;
    reg         wb_wp = 1'b0, wb_rp = 1'b0;
    reg  [1:0]  wb_n = 2'd0;

    wire take_rd  = (st == S_IDLE) && avl_read && !avl_write && !avl_waitrequest;
    wire take_wr0 = (st == S_IDLE) && avl_write && !avl_waitrequest;          // beat 0 comes with the command
    wire take_wrn = (st == S_WR) && avl_write && !avl_waitrequest;            // the later beats
    wire push     = take_wr0 | take_wrn;
    wire [71:0] wb_head = wb_rp ? wb1 : wb0;
    wire [1:0]  wk = widx[1:0];

    assign w_valid = (st == S_RD) || (st == S_WR && wb_n != 2'd0);
    assign w_we    = (st == S_WR);
    wire [31:0] baddr = {base, 3'b000} + {21'd0, widx, 1'b0};
    assign w_addr  = baddr[24:1];
    assign w_data  = wb_head[16 * wk +: 16];
    assign w_be    = wb_head[64 + 2 * wk +: 2];
    assign busy    = (st != S_IDLE);
    wire took      = w_valid && w_take;
    wire pop       = took && (st == S_WR) && (wk == 2'd3);
    wire done      = took && (widx == wlast);

    wire [1:0]  wb_n_n     = wb_n + {1'b0, push} - {1'b0, pop};
    wire [8:0]  beats_in_n = take_wr0 ? 9'd1 : (beats_in + {8'd0, take_wrn});
    wire [8:0]  nbeats_n   = take_wr0 ? {1'b0, avl_burstcount} : nbeats;

    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; widx <= 10'd0; wb_wp <= 1'b0; wb_rp <= 1'b0; wb_n <= 2'd0; beats_in <= 9'd0;
            avl_waitrequest <= 1'b1;
        end else begin
            // the command
            if (take_rd || take_wr0) begin
                base  <= avl_address;
                wlast <= {avl_burstcount, 2'b00} - 10'd1;
                widx  <= 10'd0;
                st    <= take_wr0 ? S_WR : S_RD;
            end else if (took) begin
                widx <= widx + 10'd1;
                if (done) st <= S_IDLE;
            end
            // the write beats
            if (push) begin
                if (wb_wp) wb1 <= {avl_byteenable, avl_writedata}; else wb0 <= {avl_byteenable, avl_writedata};
                wb_wp <= ~wb_wp;
            end
            if (pop) wb_rp <= ~wb_rp;
            wb_n     <= wb_n_n;
            beats_in <= beats_in_n;
            nbeats   <= nbeats_n;
            // ready for a command or beat in the next clock: idle (and not just taking one), or a write burst that
            // still expects beats and has room for one more whatever happens this clock
            if (take_rd || take_wr0)
                avl_waitrequest <= !(take_wr0 && beats_in_n < nbeats_n && wb_n_n < 2'd2);
            else if (st == S_IDLE)
                avl_waitrequest <= 1'b0;
            else if (st == S_RD)
                avl_waitrequest <= !done;
            else                                                                  // S_WR
                avl_waitrequest <= done ? 1'b0 : !(beats_in_n < nbeats && wb_n_n < 2'd2);
        end
    end

    // read words back into beats, in order
    reg [1:0]  rk = 2'd0;
    reg [47:0] racc = 48'd0;
    always @(posedge clk) begin
        avl_readdatavalid <= 1'b0;
        if (rst) rk <= 2'd0;
        else if (r_valid) begin
            rk <= rk + 2'd1;
            case (rk)
                2'd0: racc[15:0]  <= r_data;
                2'd1: racc[31:16] <= r_data;
                2'd2: racc[47:32] <= r_data;
                default: begin avl_readdata <= {r_data, racc}; avl_readdatavalid <= 1'b1; end
            endcase
        end
    end
endmodule
`default_nettype wire
