// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tango_beam_lead.v -- verbatim copy of the tango_beam_lead module from tango_overlay.v
// (the launcher's earlier overlay).  tango_overlay.v keeps its copy for its reference build; the
// menu core (tango_menu_core) uses this file and never includes tango_overlay.v.
`default_nettype none
//------------------------------------------------------------------------------
// tango_beam_lead -- hdmi.sv beam + D clocks (registered): lcx/lcy at clock t+1
// = the hdmi.sv beam of clock t+1+D.  (From cx(t): cx(t)+D+1 mod FW, cy carried.)
// hdmi.sv holds cx = cy = 0 in reset, so the lead is exact from the first clock
// after reset release on.
//------------------------------------------------------------------------------
module tango_beam_lead #(parameter integer D = 5, parameter integer FW = 1650, parameter integer FH = 750) (
    input  wire        clk,
    input  wire [11:0] cx,
    input  wire [10:0] cy,
    output reg  [11:0] lcx = 12'd0,
    output reg  [10:0] lcy = 11'd0
);
    wire [12:0] s = {1'b0, cx} + D + 1;
    always @(posedge clk)
        if (s >= FW) begin
            lcx <= s - FW;
            lcy <= (cy == FH - 1) ? 11'd0 : cy + 11'd1;
        end else begin
            lcx <= s[11:0];
            lcy <= cy;
        end
endmodule
`default_nettype wire
