// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_shmask -- MiSTer's shadow mask (README "Video and audio processing"), our own code, on the scaled picture
// after the scaler (and the framework's scanline dimming), before the OSD, as MiSTer's sys_top.v 1156-1176 places
// its shadowmask.sv (Template_MiSTer 19e89ce, read for behaviour only).
//
// Commands (SYS 0x3E words, MiSTer's: Main_MiSTer video.cpp 808-815, shadowmask.sv 130-145):
//   15:13 = 000  bits 3:1 = {enable, rotate, 2x}; the LUT index restarts at 0
//           001  vmax (rows - 1, 0..15)        010  hmax (columns - 1, 0..15)
//           011  bits 10:0 = the next LUT entry (16 per row, row-major)
// The mask cell of a picture pixel: column = (x - hmin) mod period, row = (y - vmin) mod period, i.e. counted from
// the picture's first pixel of each line and its first line, as MiSTer's counters restart at the picture's left
// border (pde) and count picture lines (shadowmask.sv 54-71); periods (rotate ? vmax : hmax) << 2x | 2x and
// (rotate ? hmax : vmax) << 2x | 2x; 2x halves the counters; rotate makes the column count the LUT row (41-48).
// Entry -> per colour a 1.4 multiplier: lut[10:8] (R, G, B) choose {1, lut[7:4]} (>= 1.0) or {0, lut[3:0]} (< 1.0);
// disabled = 1.0 (77-85).  Product: the sum of v>>4, v>>3, v>>2, v>>1, v for multiplier bits 0..4, each term
// truncated, clamped to 255 (99-125).  Here the counters come from the beam instead of HS / border edges: the same
// cells for every pixel of the picture.
// Pipeline: LAT = 6 clocks, din -> dout; cx / cy are the beam of din (same clock).
//==============================================================================
`default_nettype none
module tango_shmask #(
    parameter integer FW = 1650             // beam columns per line (cx wraps here)
) (
    input  wire        clk,
    input  wire        cmd_wr,
    input  wire [15:0] cmd,
    input  wire [11:0] cx,                  // beam of din
    input  wire [10:0] cy,
    input  wire [11:0] hmin,                // the picture's top-left on the canvas (tango_video's ascal window)
    input  wire [10:0] vmin,
    input  wire [23:0] din,
    output reg  [23:0] dout = 24'd0
);
    localparam integer LAT = 6;
    //------------------------------------------------------------------ commands
    reg  [3:0]  hmax = 4'd0, vmax = 4'd0;
    reg         m_en = 1'b0, m_rot = 1'b0, m_2x = 1'b0;
    reg  [7:0]  widx = 8'd0;
    reg  [10:0] lut [0:255];
    always @(posedge clk) if (cmd_wr) begin
        case (cmd[15:13])
        3'b000: begin {m_en, m_rot, m_2x} <= cmd[3:1]; widx <= 8'd0; end
        3'b001: vmax <= cmd[3:0];
        3'b010: hmax <= cmd[3:0];
        3'b011: begin lut[widx] <= cmd[10:0]; widx <= widx + 8'd1; end
        default: ;
        endcase
    end
    // periods (minus one) of the column and row counters
    reg  [4:0]  hp = 5'd0, vp = 5'd0;
    always @(posedge clk) begin
        hp <= ({1'b0, m_rot ? vmax : hmax} << m_2x) | {4'd0, m_2x};
        vp <= ({1'b0, m_rot ? hmax : vmax} << m_2x) | {4'd0, m_2x};
    end

    //------------------------------------------------------------------ stage 1: counters for the beam of din
    reg  [4:0]  hc = 5'd0, vc = 5'd0;
    wire        line0 = (cx == hmin);            // the picture's first column on this line
    always @(posedge clk) begin
        if (line0) begin
            hc <= 5'd0;
            vc <= (cy == vmin) ? 5'd0 : ((vc == vp) ? 5'd0 : vc + 5'd1);
        end else
            hc <= (hc == hp) ? 5'd0 : hc + 5'd1;
    end
    // stage 2: the LUT index (MiSTer's {row, column}, 4 bits each; 2x halves; rotate swaps)
    wire [3:0]  hi = m_2x ? hc[4:1] : hc[3:0];
    wire [3:0]  vi = m_2x ? vc[4:1] : vc[3:0];
    reg  [7:0]  idx = 8'd0;
    always @(posedge clk) idx <= m_rot ? {hi, vi} : {vi, hi};
    // stage 3: the entry (BSRAM read)
    reg  [10:0] ent = 11'd0;
    always @(posedge clk) ent <= lut[idx];
    // stage 4: the multipliers (1.4 fixed point); disabled = 1.0
    reg  [4:0]  mr = 5'b10000, mg = 5'b10000, mb = 5'b10000;
    always @(posedge clk) begin
        mr <= !m_en ? 5'b10000 : ent[10] ? {1'b1, ent[7:4]} : {1'b0, ent[3:0]};
        mg <= !m_en ? 5'b10000 : ent[9]  ? {1'b1, ent[7:4]} : {1'b0, ent[3:0]};
        mb <= !m_en ? 5'b10000 : ent[8]  ? {1'b1, ent[7:4]} : {1'b0, ent[3:0]};
    end
    // the pixel, delayed to meet its multipliers at stage 5
    reg  [23:0] d1 = 24'd0, d2 = 24'd0, d3 = 24'd0, d4 = 24'd0;
    always @(posedge clk) begin d1 <= din; d2 <= d1; d3 <= d2; d4 <= d3; end
    // stage 5: the terms of v * m / 16, in two partial sums;  stage 6: their sum, clamped
    function [7:0] lo_terms(input [7:0] v, input [4:0] m);        // m bits 0, 1: v>>4 + v>>3 (<= 46)
        lo_terms = (m[0] ? {4'd0, v[7:4]} : 8'd0) + (m[1] ? {3'd0, v[7:3]} : 8'd0);
    endfunction
    function [8:0] hi_terms(input [7:0] v, input [4:0] m);        // m bits 2, 3, 4: v>>2 + v>>1 + v (<= 445)
        hi_terms = (m[2] ? {3'd0, v[7:2]} : 9'd0) + (m[3] ? {2'd0, v[7:1]} : 9'd0) + (m[4] ? {1'b0, v} : 9'd0);
    endfunction
    reg  [7:0]  lr = 8'd0, lg = 8'd0, lb = 8'd0;
    reg  [8:0]  ur = 9'd0, ug = 9'd0, ub = 9'd0;
    always @(posedge clk) begin
        lr <= lo_terms(d4[23:16], mr);  ur <= hi_terms(d4[23:16], mr);
        lg <= lo_terms(d4[15:8],  mg);  ug <= hi_terms(d4[15:8],  mg);
        lb <= lo_terms(d4[7:0],   mb);  ub <= hi_terms(d4[7:0],   mb);
    end
    wire [8:0]  sr = {1'b0, lr} + ur, sg = {1'b0, lg} + ug, sb = {1'b0, lb} + ub;   // <= 491
    always @(posedge clk)
        dout <= {sr[8] ? 8'hFF : sr[7:0], sg[8] ? 8'hFF : sg[7:0], sb[8] ? 8'hFF : sb[7:0]};
endmodule
`default_nettype wire
