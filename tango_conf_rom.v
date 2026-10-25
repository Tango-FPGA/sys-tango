// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_conf_rom -- a core's CONF_STR as a byte ROM for SYS READ_CFG.
// The core instantiates it with its string, as MiSTer cores pass CONF_STR to hps_io
// (Template_MiSTer Template.sv 58-86, 91: hps_io #(.CONF_STR(CONF_STR))), and wires it to
// the clk / conf_addr / conf_char ports of tango_core:
//     tango_conf_rom #(.CONF_STR(CONF_STR)) u_conf (.clk(CLK_PIX), .addr(conf_addr), .q(conf_char));
// Held in logic (GW5A BSRAM initial contents do not survive on silicon).  Byte 0 = the first
// character; 00 past the end.  q follows addr TWO clk edges later: a 32:1 select per group of 32
// bytes, registered, then the group, registered.  tango_link holds each address for ~40 clocks
// (one SPI byte) before it uses the byte, so the latency is free, and no CONF_STR length (up to
// 4095 bytes) puts a long mux into one clock.  (It was combinational: a 551-byte CONF_STR -- the
// A2600 -- missed clk_pix by up to 0.56 ns, first as a compare chain, then as one mux tree.)
//==============================================================================
`default_nettype none
module tango_conf_rom #(
    parameter CONF_STR = ""
) (
    input  wire        clk,             // CLK_PIX (conf_addr comes from tango_link in it)
    input  wire [11:0] addr,
    output wire [7:0]  q
);
    localparam integer LEN = $bits(CONF_STR) / 8;
    localparam integer G = (LEN + 31) / 32;           // groups of 32 bytes
    wire [8*LEN-1:0] str = CONF_STR;
    wire [8*32*G-1:0] s;                              // byte i at s[8*i +: 8], zero-padded
    genvar g;
    generate for (g = 0; g < 32 * G; g = g + 1) begin : rev
        if (g < LEN) begin : b assign s[8*g +: 8] = str[8*(LEN-1-g) +: 8]; end
        else begin : z assign s[8*g +: 8] = 8'h00; end
    end endgenerate
    reg [7:0] grp [0:G-1];
    reg [6:0] ah = 7'd0;
    reg       past = 1'b1;
    reg [7:0] qr = 8'h00;
    integer k;
    always @(posedge clk) begin
        for (k = 0; k < G; k = k + 1) grp[k] <= s[8*(32*k + addr[4:0]) +: 8];
        ah   <= addr[11:5];
        past <= (addr[11:5] >= G);
        qr   <= past ? 8'h00 : grp[ah];
    end
    assign q = qr;
endmodule
`default_nettype wire
