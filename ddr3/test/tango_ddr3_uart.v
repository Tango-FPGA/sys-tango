// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_uart -- the DDR3 bring-up test's UART report: every PERIOD clocks one line,
//   "D1" then, for each of the NS slots, a space and its 8 hex digits (slot 0 first), then CR LF,
// 8N1 at clk / BAUD_DIV (115200 at 50 MHz).  The slots are read from the UART bank (one clock of read latency).
// When a line is due it raises want and starts on go (tango_ddr3_test_core: once its copy of the slots is ready).
// On the Console 138K the pin is U15, the Sipeed debugger's UART RX while the BL616 is the PC's USB debugger (JTAG
// loads), as Sipeed's DDR3 test uses it ("DDR3 TEST OKAY!").
//==============================================================================
`default_nettype none
module tango_ddr3_uart #(
    parameter integer NS       = 111,
    parameter integer BAUD_DIV = 434,
    parameter integer PERIOD   = 100000000
) (
    input  wire        clk,
    output reg  [6:0]  slot_addr = 7'd0,
    input  wire [31:0] slot_data,
    output reg         tx = 1'b1,
    output reg         want = 1'b0,          // a line is due
    input  wire        go                    // start it
);
    // ---- the line: char index c = 0, 1: "D1"; then 9 per slot (space, 8 digits); then CR, LF
    localparam integer NCH = 2 + 9 * NS + 2;
    reg [31:0] pt = 0;
    reg        busy = 1'b0;
    reg [11:0] c = 0;
    reg [6:0]  s = 0;            // slot of char c
    reg [3:0]  k = 0;            // char within the slot: 0 = space, 1..8 = digits
    reg [31:0] sd = 0;
    reg [9:0]  sh = 10'h3FF;     // shift register: start bit, 8 data, stop bit (LSB first)
    reg [3:0]  nb = 0;
    reg [15:0] bt = 0;
    reg [1:0]  ph = 0;           // 0 = address the slot, 1 = the bank reads it, 2 = choose the char, 3 = send
    function [7:0] hexc(input [3:0] v);
        hexc = (v < 4'd10) ? (8'h30 + v) : (8'h41 + v - 4'd10);
    endfunction
    reg [7:0] ch;
    always @(posedge clk) begin
        if (!busy) begin
            tx <= 1'b1;
            if (want) begin
                if (go) begin want <= 1'b0; busy <= 1'b1; c <= 0; s <= 0; k <= 0; ph <= 0; end
            end else if (pt == PERIOD - 1) begin pt <= 0; want <= 1'b1; end
            else pt <= pt + 1'b1;
        end else begin
            case (ph)
                2'd0: begin slot_addr <= s; ph <= 2'd1; end
                2'd1: ph <= 2'd2;                                  // the bank's read register loads
                2'd2: begin
                    if (k == 4'd1) sd <= slot_data;
                    // pick the char
                    if (c == 0) ch = 8'h44;                        // 'D'
                    else if (c == 1) ch = 8'h31;                   // '1'
                    else if (c == NCH - 2) ch = 8'h0D;
                    else if (c == NCH - 1) ch = 8'h0A;
                    else if (k == 4'd0) ch = 8'h20;
                    else if (k == 4'd1) ch = hexc(slot_data[31:28]);
                    else ch = hexc(sd[31 - 4 * (k - 1) -: 4]);
                    sh <= {1'b1, ch, 1'b0}; nb <= 0; bt <= 0; ph <= 2'd3;
                end
                default: begin
                    tx <= sh[0];
                    if (bt == BAUD_DIV - 1) begin
                        bt <= 0; sh <= {1'b1, sh[9:1]}; nb <= nb + 1'b1;
                        if (nb == 4'd9) begin
                            // next char
                            ph <= 2'd0;
                            if (c == NCH - 1) begin busy <= 1'b0; pt <= 0; end
                            c <= c + 1'b1;
                            if (c >= 2 && c < NCH - 2) begin
                                if (k == 4'd8) begin k <= 0; s <= s + 1'b1; end else k <= k + 1'b1;
                            end
                        end
                    end else bt <= bt + 1'b1;
                end
            endcase
        end
    end
endmodule
`default_nettype wire
