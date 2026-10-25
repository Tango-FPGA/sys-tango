// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_ascal_ddr3_core -- a SIM-ONLY tango_core for sim/tb_ascal_ddr3.v (TANGO_VIDEO_DDR3 + TANGO_VIDEO_F1): the video
// of sim/tb_ascal_ext_core.v (an interlaced, then progressive, Genesis H40 source whose pixel colours tell
// sim/check_ascal_ext.py which source line it sees), without its memory: under TANGO_VIDEO_DDR3 ascal's Avalon port
// is the framework's (tango_ddr3's scaler port), and this core has no SDRAM and no DDRAM port.
//   clocks   its own: clk_mem 107.386 MHz (behavioural), CLK_SYS = CLK_VIDEO = clk_mem / 2, as the ext core.
//   status   [1] progressive; the framework's Video bits [127:120] are not read (TANGO_VIDEO_OPTS).
//==============================================================================
`default_nettype none
`timescale 1ps/1ps
module tango_core (
    `include "tango_core_ports.vh"
);
    reg clk_mem = 1'b0;
    always #4656 clk_mem = ~clk_mem;                 // 107.39 MHz (9.312 ns)
    reg clk_sys = 1'b0;
    always @(posedge clk_mem) clk_sys <= ~clk_sys;
    assign CLK_SYS = clk_sys;
    assign status_menumask = 16'd0;
    assign ioctl_wait = 1'b0;
    assign conf_char = 8'd0;                         // an empty CONF_STR

    // ---- video (CLK_VIDEO = clk_sys)
    reg  [2:0] cediv = 3'd0;
    reg        ce = 1'b0;
    reg  [8:0] hx = 9'd0, vy = 9'd0;
    reg        f1 = 1'b0, prog = 1'b0;
    wire [8:0] vlast = prog ? 9'd261 : (f1 ? 9'd262 : 9'd261);   // F1 = 1 field: 263 lines (0..262)
    always @(posedge clk_sys) begin
        cediv <= cediv + 3'd1;
        ce    <= (cediv == 3'd7);
        if (ce) begin
            if (hx == 9'd427) begin
                hx <= 9'd0;
                if (vy == vlast) begin vy <= 9'd0; prog <= status[1]; end
                else vy <= vy + 9'd1;
                if (vy == 9'd223) f1 <= prog ? 1'b0 : ~f1;              // toggles in the vertical blanking
            end else hx <= hx + 9'd1;
        end
    end
    wire [9:0] L = prog ? {1'b0, vy} : {vy, f1};
    assign CLK_VIDEO = clk_sys;
    assign CE_PIXEL  = ce;
    reg  [7:0] r = 8'd0, g = 8'd0, b = 8'd0;
    reg        hs = 1'b0, vs = 1'b0, de = 1'b0, fl = 1'b0;
    always @(posedge clk_sys) if (ce) begin            // outputs change only at CE clocks (the CE_PIXEL rule)
        de <= (hx < 9'd320) && (vy < 9'd224);
        hs <= (hx >= 9'd340) && (hx < 9'd372);
        vs <= (vy >= 9'd234) && (vy < 9'd237);
        fl <= f1;
        r  <= L[7:0];
        g  <= prog ? 8'h63 : {L[8], 7'h3C};
        b  <= hx[7:0];
    end
    assign VGA_R = r;  assign VGA_G = g;  assign VGA_B = b;
    assign VGA_HS = hs;  assign VGA_VS = vs;  assign VGA_DE = de;
    assign VGA_F1 = fl;
    assign VGA_SL = 2'd0;                              // TANGO_VIDEO_OPTS: sys's Scanlines
    assign VIDEO_ARX = 13'd4;  assign VIDEO_ARY = 13'd3;
    assign HDMI_FREEZE = 1'b0;
    assign VIDEO_SCALE = 3'd0;  assign VIDEO_CROP_SIZE = 12'd0;  assign VIDEO_CROP_OFF = 5'd0;
    assign AUDIO_L = 16'd0;  assign AUDIO_R = 16'd0;
`ifdef TANGO_CORE_AUDIO_MIX
    assign AUDIO_MIX = 2'd0;
`endif
endmodule
`default_nettype wire
