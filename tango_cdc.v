// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_cdc_word -- carries a multi-bit value from clk_a to clk_b, continuously.
// Four-phase-free toggle handshake: clk_a holds a copy (hold) and flips req; clk_b sees
// req through 3 flops, takes hold (stable since req flipped) and answers with ack, which
// clk_a sees through 2 flops before it samples again.  Every transfer is one coherent
// sample of din; latency a few clocks of each side.  No multi-bit synchroniser anywhere.
//==============================================================================
`default_nettype none
module tango_cdc_word #(
    parameter integer W = 8,
    parameter [W-1:0] INIT = {W{1'b0}}
) (
    input  wire         clk_a,
    input  wire [W-1:0] din,
    input  wire         clk_b,
    output reg  [W-1:0] dout = INIT
);
    reg [W-1:0] hold = INIT;
    reg         req = 1'b0;
    reg [1:0]   ack_s = 2'b00;
    reg         ack = 1'b0;
    always @(posedge clk_a) begin
        ack_s <= {ack_s[0], ack};
        if (req == ack_s[1]) begin hold <= din; req <= ~req; end
    end
    reg [2:0] req_s = 3'b000;
    always @(posedge clk_b) begin
        req_s <= {req_s[1:0], req};
        if (req_s[2] != req_s[1]) begin dout <= hold; ack <= req_s[1]; end
    end
endmodule

//==============================================================================
// tango_async_fifo -- dual-clock FIFO, gray-coded pointers (2 flops each way), one BSRAM.
// Write side reports free entries; read side is a registered read (data one clock after rd).
//==============================================================================
module tango_async_fifo #(
    parameter integer AW = 11,              // 2^AW entries
    parameter integer W  = 10
) (
    input  wire          wclk,
    input  wire          wrst,
    input  wire          we,
    input  wire [W-1:0]  wd,
    output reg  [AW:0]   free = {1'b1, {AW{1'b0}}},
    input  wire          rclk,
    input  wire          rrst,
    input  wire          rd,                // only when !empty
    output reg  [W-1:0]  rdata,
    output wire          empty
);
    (* syn_ramstyle = "block_ram" *) reg [W-1:0] mem [0:(1<<AW)-1];
    reg [AW:0] wp = 0, wg = 0, rp = 0, rg = 0;
    reg [AW:0] rg_w1 = 0, rg_w2 = 0, wg_r1 = 0, wg_r2 = 0;
    function [AW:0] g2b(input [AW:0] g);
        integer k;
        begin
            g2b[AW] = g[AW];
            for (k = AW - 1; k >= 0; k = k - 1) g2b[k] = g2b[k+1] ^ g[k];
        end
    endfunction
    // write side
    wire [AW:0] wp_n = wp + {{AW{1'b0}}, 1'b1};
    always @(posedge wclk) begin
        rg_w1 <= rg; rg_w2 <= rg_w1;
        if (wrst) begin wp <= 0; wg <= 0; end
        else if (we) begin mem[wp[AW-1:0]] <= wd; wp <= wp_n; wg <= wp_n ^ (wp_n >> 1); end
        free <= (1 << AW) - ((we ? wp_n : wp) - g2b(rg_w2));
    end
    // read side
    wire [AW:0] rp_n = rp + {{AW{1'b0}}, 1'b1};
    always @(posedge rclk) begin
        wg_r1 <= wg; wg_r2 <= wg_r1;
        if (rrst) begin rp <= 0; rg <= 0; end
        else if (rd) begin rp <= rp_n; rg <= rp_n ^ (rp_n >> 1); end
        rdata <= mem[rp[AW-1:0]];
    end
    assign empty = (rg == wg_r2);
endmodule
`default_nettype wire
