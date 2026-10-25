// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tango_tmds_ser -- bench-proven at 720p on the Tang Console 138K.
// At 720p: clk_pix = 74.25 MHz, clk_ser = 371.25 MHz (pll_c1485_mod ODIV1/ODIV0).
// menu_tmds_ser -- 10:1 TMDS serializer for the menu display engine.
// Three OSER10 lanes (5x pixel clock, DDR inside the primitive) plus the
// pixel-rate clock lane, all through pseudo-differential ELVDS_OBUF pairs
// (LVCMOS33D on the Tang 25K dock HDMI connector).
`default_nettype none
module tango_tmds_ser (
    input  wire       clk_pix,      // 148.5 MHz pixel clock
    input  wire       clk_ser,      // 742.5 MHz serial clock
    input  wire       resetn,
    input  wire [9:0] sym_ch0,      // blue   + syncs
    input  wire [9:0] sym_ch1,      // green
    input  wire [9:0] sym_ch2,      // red
    output wire       tmds_clk_p, tmds_clk_n,
    output wire [2:0] tmds_d_p,  tmds_d_n
);
    wire [2:0] ser_bit;

    OSER10 u_ser [2:0] (
        .Q (ser_bit),
        .D0({sym_ch2[0], sym_ch1[0], sym_ch0[0]}),
        .D1({sym_ch2[1], sym_ch1[1], sym_ch0[1]}),
        .D2({sym_ch2[2], sym_ch1[2], sym_ch0[2]}),
        .D3({sym_ch2[3], sym_ch1[3], sym_ch0[3]}),
        .D4({sym_ch2[4], sym_ch1[4], sym_ch0[4]}),
        .D5({sym_ch2[5], sym_ch1[5], sym_ch0[5]}),
        .D6({sym_ch2[6], sym_ch1[6], sym_ch0[6]}),
        .D7({sym_ch2[7], sym_ch1[7], sym_ch0[7]}),
        .D8({sym_ch2[8], sym_ch1[8], sym_ch0[8]}),
        .D9({sym_ch2[9], sym_ch1[9], sym_ch0[9]}),
        .PCLK(clk_pix), .FCLK(clk_ser), .RESET(~resetn));

    ELVDS_OBUF u_obuf [3:0] (
        .I ({clk_pix,    ser_bit}),
        .O ({tmds_clk_p, tmds_d_p}),
        .OB({tmds_clk_n, tmds_d_n}));
endmodule
`default_nettype wire
