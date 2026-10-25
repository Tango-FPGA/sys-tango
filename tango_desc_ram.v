// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_desc_ram -- the USB descriptor store of one port: 2048 x 8, written by the USB host in clk48
// (device descriptor @000, configuration @100, HID report descriptor @400: usb_fs_host.v), read by the link in clk_pix with a
// registered output (latency 1 clock after the address register).  One BSRAM; GW5A has no LUT RAM, so it is hinted block_ram.
// No initial contents (a $readmemh/initial image does not survive Gowin synthesis).  The reader only trusts bytes inside the
// lengths the host published after it finished writing (tango_link, desc_rdy).
//==============================================================================
`default_nettype none
module tango_desc_ram (
    input  wire        wclk,
    input  wire        we,
    input  wire [10:0] wa,
    input  wire [7:0]  wd,
    input  wire        rclk,
    input  wire [10:0] ra,
    output reg  [7:0]  rd = 8'd0
);
    (* syn_ramstyle = "block_ram" *) reg [7:0] mem [0:2047];
    always @(posedge wclk) if (we) mem[wa] <= wd;
    always @(posedge rclk) rd <= mem[ra];
endmodule
`default_nettype wire
