// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//------------------------------------------------------------------------------
// tango_beam_lead_rt -- tango_beam_lead with the raster's totals as INPUTS (TANGO_VSYNC_ADJUST builds; README
// "Native-rate video (vsync_adjust)"): lcx/lcy at clock t+1 = the hdmi.sv beam of clock t+1+D.  fw / fh must be the
// totals of the frame the INPUT beam (cx, cy) is in -- hdmi.sv's frame_width / frame_height outputs, which change at
// its wrap -- because the lead leaves the frame by adding D+1 to cx: past fw it is on line 0 of the next frame at
// x = cx + D + 1 - fw, whatever the next frame's totals are (D + 1 < any width).  Identical to tango_beam_lead when
// fw / fh are that module's FW / FH.
//------------------------------------------------------------------------------
`default_nettype none
module tango_beam_lead_rt #(parameter integer D = 5) (
    input  wire        clk,
    input  wire [11:0] cx,
    input  wire [10:0] cy,
    input  wire [11:0] fw,                 // the input beam's frame: hdmi.sv frame_width
    input  wire [10:0] fh,                 //                         hdmi.sv frame_height
    output reg  [11:0] lcx = 12'd0,
    output reg  [10:0] lcy = 11'd0
);
    wire [12:0] s = {1'b0, cx} + D + 1;
    always @(posedge clk)
        if (s >= {1'b0, fw}) begin
            lcx <= s[11:0] - fw;
            lcy <= (cy == fh - 11'd1) ? 11'd0 : cy + 11'd1;
        end else begin
            lcx <= s[11:0];
            lcy <= cy;
        end
endmodule
`default_nettype wire
