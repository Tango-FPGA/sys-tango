// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_gamma -- MiSTer's gamma correction (README "Video and audio processing"), our own code: three 256 x 8 tables,
// one per colour, applied to the core's pixels before the scaler, in the core's video clock, as MiSTer's
// video_mixer does (Template_MiSTer 19e89ce sys/video_mixer.sv 105-135 -> gamma_corr.sv 64-118 "gamma_fast",
// read for behaviour only: out = en ? table_c[in_c] : in_c per channel).
// Tables: written from the link (clk_pix, SYS 0x33: address {channel 0 R / 1 G / 2 B, index}, hps_io.sv 528-532),
// read here at CE_PIXEL: three dual-clock BSRAMs.  en: SYS 0x32, from clk_pix through two flops (it changes only
// on a link command, a whole frame or more apart).
// Pipeline: two CE stages for every signal, table on or off, so RGB stays with HS / VS / DE:
//   CE n:   q_c <= table_c[in_c] (the BSRAM's read register), d_c <= in_c, sync1 <= {hs, vs, de}
//   CE n+1: out_c <= en ? q_c : d_c, sync_out <= sync1
// The outputs change only at CE clocks, as the core's did, so ascal (which samples at CE) sees a clean stream.
//==============================================================================
`default_nettype none
module tango_gamma (
    // tables and enable (clk_pix)
    input  wire       clk_w,
    input  wire       wr,
    input  wire [9:0] wa,
    input  wire [7:0] wd,
    input  wire       en_w,
    // the core's video (clk_video)
    input  wire       clk,
    input  wire       ce,
    input  wire [7:0] r_in, g_in, b_in,
    input  wire       hs_in, vs_in, de_in,
    output reg  [7:0] r_out = 8'd0, g_out = 8'd0, b_out = 8'd0,
    output reg        hs_out = 1'b0, vs_out = 1'b0, de_out = 1'b0
);
    reg [7:0] tab_r [0:255];
    reg [7:0] tab_g [0:255];
    reg [7:0] tab_b [0:255];
    always @(posedge clk_w) if (wr) begin
        if (wa[9:8] == 2'd0) tab_r[wa[7:0]] <= wd;
        if (wa[9:8] == 2'd1) tab_g[wa[7:0]] <= wd;
        if (wa[9:8] == 2'd2) tab_b[wa[7:0]] <= wd;
    end
    reg [1:0] en_s = 2'b00;
    always @(posedge clk) en_s <= {en_s[0], en_w};

    reg [7:0] q_r = 8'd0, q_g = 8'd0, q_b = 8'd0;
    reg [7:0] d_r = 8'd0, d_g = 8'd0, d_b = 8'd0;
    reg       hs1 = 1'b0, vs1 = 1'b0, de1 = 1'b0;
    always @(posedge clk) if (ce) begin
        q_r <= tab_r[r_in];  q_g <= tab_g[g_in];  q_b <= tab_b[b_in];
    end
    always @(posedge clk) if (ce) begin
        d_r <= r_in;  d_g <= g_in;  d_b <= b_in;
        hs1 <= hs_in; vs1 <= vs_in; de1 <= de_in;
        r_out <= en_s[1] ? q_r : d_r;
        g_out <= en_s[1] ? q_g : d_g;
        b_out <= en_s[1] ? q_b : d_b;
        hs_out <= hs1; vs_out <= vs1; de_out <= de1;
    end
endmodule
`default_nettype wire
