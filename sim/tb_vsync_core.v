// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_vsync_core -- a SIM-ONLY tango_core for sim/tb_ascal_ddr3.v (TB_VSYNC_CORE): a
// progressive source whose frame is an EXACT number of clk_pix, so the bench can run ascal at a native rate.  Its line
// is the TG16's at the exact clock: 1365 master clocks of 945/44 MHz = 4719 clk_pix of 74.25 MHz exactly (74.25 x 1365
// x 44 / 945), made here as 363 dots of CE_PIXEL every 13 clk_pix (CLK_VIDEO = CLK_PIX).  +core_lines=<n> lines per
// frame (default 263: 1,241,097 clk_pix = 59.826105 Hz; 262: 1,236,378 = 60.054449 Hz).  320 x 240 visible (ascal's
// stride is then 4 bursts of 256 B a line, as the bench's monitor expects), HS dots 330-349, VS lines 245-247.
// +core_delay=<n>: the source starts n clk_pix after reset (a phase sweep against the output raster).
// +core_lines2=<n> +core_switch=<k>: from its k-th frame on the source runs n lines a frame (the TG16's
// 262 <-> 263, which a HuCard picks with the VCE's CR bit 2 at run time); the change takes effect at a frame wrap.
// +core_ce=<n>: CE_PIXEL every n clk_pix (default 13: the TG16's line; the negative control uses a faster source whose
// frame keeps its VS lines -- fewer lines than 248 would lose them).
// Pixel colours: R = the line (low 8 bits), G = 0x5A, B = the dot.
// TB_CORE_CLK (the build-time HDMI PLL presets: clk_pix is no longer a whole number of TG16 lines): the source runs on its
// OWN clock, the TG16's clk_ram = 945/11 MHz = 4 x the master clock (half periods 5820 + 20/189 ps, Bresenham), its line
// 420 dots x 13 = 5460 clocks = 1365 master clocks exactly (CLK_SYS = CLK_VIDEO = that clock, asynchronous to clk_pix as
// on the board); RESET is synchronised into it; +core_delay counts its clocks.
//==============================================================================
`default_nettype none
`timescale 1ps/1ps
module tango_core (
    `include "tango_core_ports.vh"
);
`ifdef TB_CORE_CLK
    reg cclk = 1'b0;  integer cacc = 0;
    always begin                                   // 945/11 MHz: 11e12 / 1890e6 = 5820 + 20/189 ps a half period
        cacc = cacc + 20;
        if (cacc >= 189) begin cacc = cacc - 189; #5821; end else #5820;
        cclk = ~cclk;
    end
    wire CK = cclk;
    localparam integer DOTS = 420;                 // 420 x 13 = 5460 clocks = 1365 master clocks (one TG16 line)
    reg [1:0] rst_s = 2'b11;
    always @(posedge CK) rst_s <= {rst_s[0], RESET};
    wire RST = rst_s[1];
`else
    wire CK = CLK_PIX;
    localparam integer DOTS = 363;                 // 363 x 13 = 4719 clk_pix = 1365 master clocks at 74.25 MHz
    wire RST = RESET;
`endif
    assign CLK_SYS = CK;
    assign status_menumask = 16'd0;
    assign ioctl_wait = 1'b0;
    assign conf_char = 8'd0;                         // an empty CONF_STR

    integer core_delay = 0, core_lines = 263, core_lines2 = 0, core_switch = 0, nframes = 0, core_ce = 13;
    initial begin
        void'($value$plusargs("core_delay=%d", core_delay));
        void'($value$plusargs("core_lines=%d", core_lines));
        void'($value$plusargs("core_lines2=%d", core_lines2));
        void'($value$plusargs("core_switch=%d", core_switch));
        void'($value$plusargs("core_ce=%d", core_ce));
        $display("tb_vsync_core: %0d lines of %0d dots x %0d clocks = %0d clocks a frame (%0s), start %0d clocks after reset",
                 core_lines, DOTS, core_ce, core_lines * DOTS * core_ce,
`ifdef TB_CORE_CLK
                 "its own clock, 945/11 MHz",
`else
                 "clk_pix",
`endif
                 core_delay);
    end
    reg  [31:0] dly = 32'd0;
    reg         run = 1'b0;
    always @(posedge CK)
        if (RST) begin dly <= 32'd0; run <= 1'b0; end
        else if (!run) begin dly <= dly + 32'd1; if (dly >= core_delay) run <= 1'b1; end

    reg  [3:0] cediv = 4'd0;
    reg        ce = 1'b0;
    reg  [8:0] hx = 9'd0, vy = 9'd0;
    always @(posedge CK) begin
        if (!run) begin cediv <= 4'd0; ce <= 1'b0; hx <= 9'd0; vy <= 9'd0; end
        else begin
            cediv <= (cediv == core_ce - 1) ? 4'd0 : cediv + 4'd1;
            ce    <= (cediv == core_ce - 1);
            if (ce) begin
                if (hx == 9'(DOTS - 1)) begin
                    hx <= 9'd0;
                    if (vy == core_lines - 1) begin
                        vy <= 9'd0;
                        nframes = nframes + 1;
                        if (core_lines2 != 0 && nframes == core_switch) begin
                            $display("tb_vsync_core: from source frame %0d on, %0d lines (%0d clocks a frame) at %0t", nframes,
                                     core_lines2, core_lines2 * DOTS * core_ce, $time);
                            core_lines = core_lines2;
                        end
                    end else vy <= vy + 9'd1;
                end else hx <= hx + 9'd1;
            end
        end
    end
    assign CLK_VIDEO = CK;
    assign CE_PIXEL  = ce;
    reg  [7:0] r = 8'd0, g = 8'd0, b = 8'd0;
    reg        hs = 1'b0, vs = 1'b0, de = 1'b0;
    always @(posedge CK) if (ce) begin                 // outputs change only at CE clocks (the CE_PIXEL rule)
        de <= (hx < 9'd320) && (vy < 9'd240);
        hs <= (hx >= 9'd330) && (hx < 9'd350);
        vs <= (vy >= 9'd245) && (vy < 9'd248);
        r  <= vy[7:0];
        g  <= 8'h5A;
        b  <= hx[7:0];
    end
    assign VGA_R = r;  assign VGA_G = g;  assign VGA_B = b;
    assign VGA_HS = hs;  assign VGA_VS = vs;  assign VGA_DE = de;
`ifdef TANGO_VIDEO_F1
    assign VGA_F1 = 1'b0;
`endif
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
