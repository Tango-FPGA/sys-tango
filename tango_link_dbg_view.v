// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_link_dbg_view -- on-screen link debug for the menu core's debug build
// (tango_menu138dbg_top).  Draws N_ROWS rows of 8 blocks (bit 7 left) at the top-left
// of the 720p picture, left of the Tango panel: a lit block = 1, a dark grey block = 0.
// Each block is 16x16 px on a 20 px grid starting at (8, 8); rows alternate colours.
// Row meanings: tango_link.v "debug (never reset)" + row 11 from tango_menu_core.
// Sits between tango_menu_core's rgb and hdmi.sv: rgb_in is the pixel for hdmi's (cx, cy)
// of the previous clock, so the block test uses the beam registered by one clock.
//==============================================================================
`default_nettype none
module tango_link_dbg_view #(parameter integer N_ROWS = 12) (
    input  wire               clk,
    input  wire [11:0]        cx,
    input  wire [10:0]        cy,
    input  wire [23:0]        rgb_in,
    input  wire [8*N_ROWS-1:0] rows,       // row 0 in the top byte
    output wire [23:0]        rgb_out
);
    reg [11:0] x = 0;  reg [10:0] y = 0;
    always @(posedge clk) begin x <= cx; y <= cy; end
    wire [11:0] xr = x - 12'd8;
    wire [10:0] yr = y - 11'd8;
    // 20 px grid: block index = xr / 20, inside = xr % 20 < 16 (small constant dividers)
    reg  [4:0]  bx, by;  reg inx, iny;  reg [7:0] row;   // 5 bits: up to 32 rows
    integer i;
    always @(*) begin
        bx = 5'd31; inx = 1'b0;
        for (i = 0; i < 8; i = i + 1)
            if (xr >= 20 * i && xr < 20 * i + 16) begin bx = i; inx = 1'b1; end
        by = 5'd31; iny = 1'b0;
        for (i = 0; i < N_ROWS; i = i + 1)
            if (yr >= 20 * i && yr < 20 * i + 16) begin by = i; iny = 1'b1; end
        row = 8'd0;
        for (i = 0; i < N_ROWS; i = i + 1) if (by == i) row = rows[8*(N_ROWS-1-i) +: 8];
    end
    wire        on_blk = inx && iny && (x >= 12'd8) && (y >= 11'd8);
    wire        bit_v = row[3'd7 - bx[2:0]];
    wire [23:0] c1 = by[0] ? 24'h40E0FF : 24'hFFD040;   // lit: cyan / amber rows
    assign rgb_out = on_blk ? (bit_v ? c1 : 24'h303030) : rgb_in;
endmodule
`default_nettype wire
