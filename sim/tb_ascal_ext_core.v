// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_ascal_ext_core -- a SIM-ONLY tango_core for sim/tb_ascal_ext.v: TANGO_ASCAL_EXT + TANGO_VIDEO_F1 + TANGO_SDRAM.
//   clocks   its own: clk_mem 107.386 MHz (behavioural), CLK_SYS = CLK_VIDEO = clk_mem / 2, as a Genesis
//            core's clocks in simulation; ASCAL_AVL_CLK = clk_mem.
//   video    a Genesis H40 frame: CE_PIXEL every 8 CLK_VIDEO clocks, 428 pixels a line (320 active), 224 active lines;
//            interlaced (status[1] = 0): fields of 262 / 263 lines, VGA_F1 toggling at line 224 of each field (the
//            vertical blanking, before VS); progressive (status[1] = 1): 262 lines, VGA_F1 = 0.
//            Pixel colours tell the checker (sim/check_ascal_ext.py) which source line it sees:
//              interlaced:  woven line L = 2 x (field line) + VGA_F1: R = L[7:0], G = {L[8], 7'h3C}, B = x[7:0]
//              progressive: line L: R = L[7:0], G = 8'h63, B = x[7:0]
//            (ascal.vhd 1183-1199: a field whose previous field had i_fl = 0 is written one line down; with i_fl
//            toggling every field that is the i_fl = 1 field.)
//   memory   ascal's Avalon port through sys/tango_ascal_words.v into a 16-bit word memory that takes words at random
//            (TAKE_PCT % of clk_mem clocks, stalls of up to 63 clocks 1 in 64) and returns reads in order 4..19
//            clocks after the take: a stand-in for a core controller that serves its CPUs first.
//   status   [1] progressive; the framework's Video bits [127:120] are not read (TANGO_VIDEO_OPTS).
//==============================================================================
`default_nettype none
`timescale 1ps/1ps
module tango_core (
    `include "tango_core_ports.vh"
);
`ifndef TAKE_PCT
`define TAKE_PCT 50
`endif
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
    // the SDRAM pins: idle (this core keeps its memory inside, below)
    assign SDRAM_CLK = 1'b0;  assign SDRAM_CKE = 1'b1;  assign SDRAM_A = 13'd0;  assign SDRAM_BA = 2'd0;
    assign SDRAM_DQ = 16'hzzzz;  assign SDRAM_DQML = 1'b1;  assign SDRAM_DQMH = 1'b1;
    assign SDRAM_nCS = 1'b1;  assign SDRAM_nCAS = 1'b1;  assign SDRAM_nRAS = 1'b1;  assign SDRAM_nWE = 1'b1;

    // ---- ascal's Avalon port: tango_ascal_words + a word memory with random stalls and latency
    assign ASCAL_AVL_CLK = clk_mem;
    reg  [2:0]  rst_m = 3'b111;
    always @(posedge clk_mem) rst_m <= {rst_m[1:0], RESET};
    wire        w_valid, w_we, w_take_w;
    wire [24:1] w_addr;
    wire [15:0] w_data;
    wire [1:0]  w_be;
    reg         w_take = 1'b0, r_valid = 1'b0;
    reg  [15:0] r_data = 16'd0;
    tango_ascal_words u_words (
        .clk(clk_mem), .rst(rst_m[2]),
        .avl_address(ascal_avl_address), .avl_write(ascal_avl_write), .avl_writedata(ascal_avl_writedata),
        .avl_read(ascal_avl_read), .avl_burstcount(ascal_avl_burstcount), .avl_byteenable(ascal_avl_byteenable),
        .avl_readdata(ascal_avl_readdata), .avl_readdatavalid(ascal_avl_readdatavalid),
        .avl_waitrequest(ascal_avl_waitrequest),
        .w_valid(w_valid), .w_we(w_we), .w_addr(w_addr), .w_data(w_data), .w_be(w_be), .w_take(w_take),
        .r_valid(r_valid), .r_data(r_data), .busy());
    reg  [15:0] mem [0:(1 << 24) - 1];
    reg  [31:0] lfsr = 32'h1357_9BDF, mcyc = 32'd0;
    reg  [5:0]  stall = 6'd0;
    reg  [15:0] rf_d   [0:255];
    reg  [31:0] rf_due [0:255];
    reg  [7:0]  rf_wp = 8'd0, rf_rp = 8'd0;
    // statistics for the bench (hierarchical reads)
    reg  [31:0] n_wr = 32'd0, n_rd = 32'd0, n_rv = 32'd0, n_outside = 32'd0;
    wire        took = w_valid && w_take;
    always @(posedge clk_mem) begin
        mcyc <= mcyc + 32'd1;
        lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
        if (stall != 6'd0) begin stall <= stall - 6'd1; w_take <= 1'b0; end
        else if (lfsr[13:8] == 6'd0) begin stall <= lfsr[19:14]; w_take <= 1'b0; end
        else w_take <= ({25'd0, lfsr[6:0]} % 32'd100) < `TAKE_PCT;
        if (took) begin
            // ascal's window: tango_cfg.vh RAMBASE 0x0140_0000, 3 x RAMSIZE 0x0040_0000 (byte addresses)
            if ({w_addr, 1'b0} < 25'h140_0000) n_outside <= n_outside + 32'd1;
            if (w_we) begin
                n_wr <= n_wr + 32'd1;
                if (w_be[0]) mem[w_addr][7:0]  <= w_data[7:0];
                if (w_be[1]) mem[w_addr][15:8] <= w_data[15:8];
            end else begin
                n_rd <= n_rd + 32'd1;
                rf_d[rf_wp] <= mem[w_addr];  rf_due[rf_wp] <= mcyc + 32'd4 + {28'd0, lfsr[23:20]};  rf_wp <= rf_wp + 8'd1;
            end
        end
        r_valid <= 1'b0;
        if (rf_rp != rf_wp && rf_due[rf_rp] <= mcyc && lfsr[25:24] != 2'd0) begin
            r_valid <= 1'b1;  r_data <= rf_d[rf_rp];  rf_rp <= rf_rp + 8'd1;  n_rv <= n_rv + 32'd1;
        end
    end
endmodule
`default_nettype wire
