// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_ddr3_ui -- the DDR3 bring-up test's results screen and UART report (ddr3/test/tango_ddr3_ov.v, tango_ddr3_uart.v) from a bank of 114
// slots (+bank=<file>, written by sim/tb_ddr3_test.v): one 1280 x 720 frame into frame.ppm, one UART line into
// uart.txt, for ddr3/test/decode_d1.py (sim/run_tb_ddr3_test.sh compares both decodes with the bank).  The display's
// read path is as in tango_ddr3_test_core.v (the word 2 clocks after the overlay's address: the RAM's read, then the
// board-slot select registered), and so is the UART's (1 clock); the raster is hdmi.sv's (1650 x 750, 1280 x 720
// active, cx / cy counting the pixel whose rgb is due).
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_ddr3_ui;
    localparam integer BAUD = 4;
    reg clk_pix = 1'b0, clk50 = 1'b0;
    always #6.734 clk_pix = ~clk_pix;
    always #10    clk50   = ~clk50;
    reg [11:0] cx = 12'd0;
    reg [10:0] cy = 11'd0;
    always @(posedge clk_pix)
        if (cx == 12'd1649) begin cx <= 12'd0; cy <= (cy == 11'd749) ? 11'd0 : cy + 1'b1; end
        else cx <= cx + 1'b1;
    reg [31:0] bank [0:127];
    string fn;
    integer i;
    initial begin
        for (i = 0; i < 128; i = i + 1) bank[i] = 32'd0;
        if (!$value$plusargs("bank=%s", fn)) fn = "bank.txt";
        $readmemh(fn, bank, 0, 113);
    end
    // the display: as tango_ddr3_test_core.v
    wire [6:0]  dra;
    reg  [31:0] dq = 32'd0, drd = 32'd0;
    always @(posedge clk_pix) begin dq <= bank[dra]; drd <= dq; end
    wire [23:0] rgb;
    tango_ddr3_ov u_ov (.clk(clk_pix), .cx(cx), .cy(cy), .slot_addr(dra), .slot_data(drd),
        .err(bank[16]), .passes(bank[14]), .uptime(bank[3]), .loops(bank[13]), .calib(bank[1][1]),
        .any_err(bank[12][6] | (bank[16] != 0)), .beat(1'b1), .rgb(rgb));
    // the UART: as tango_ddr3_test_core.v
    wire [6:0]  ura;
    reg  [31:0] urd = 32'd0;
    always @(posedge clk50) urd <= bank[ura];
    wire tx, uwant;
    tango_ddr3_uart #(.NS(114), .BAUD_DIV(BAUD), .PERIOD(1000)) u_uart (.clk(clk50), .slot_addr(ura), .slot_data(urd), .tx(tx),
                                                                     .want(uwant), .go(1'b1));
    reg [7:0] ubuf [0:2047];
    integer   un = 0;
    reg       ugot = 1'b0;
    initial begin : g_urx
        integer b; reg [7:0] ch;
        forever begin
            @(negedge tx);
            #(BAUD * 20 / 2);
            ch = 8'd0;
            for (b = 0; b < 8; b = b + 1) begin #(BAUD * 20); ch[b] = tx; end
            #(BAUD * 20);
            if (!ugot) begin
                if (un < 2048) begin ubuf[un] = ch; un = un + 1; end
                if (ch == 8'h0A) begin if (un > 2 && ubuf[0] == 8'h44) ugot = 1'b1; else un = 0; end
            end
        end
    end
    integer f, n;
    initial begin
        @(posedge clk_pix); #1;
        wait (cx == 12'd1649 && cy == 11'd749);
        f = $fopen("frame.ppm", "wb");
        $fwrite(f, "P6\n1280 720\n255\n");
        n = 0;
        while (n < 1280 * 720) begin
            @(posedge clk_pix); #1;
            if (cx < 1280 && cy < 720) begin $fwrite(f, "%c%c%c", rgb[23:16], rgb[15:8], rgb[7:0]); n = n + 1; end
        end
        $fclose(f);
        wait (ugot);
        f = $fopen("uart.txt", "w");
        for (n = 0; n < un; n = n + 1) $fwrite(f, "%c", ubuf[n]);
        $fclose(f);
        $display("TB_RESULT PASS (frame.ppm, uart.txt written)");
        $finish;
    end
endmodule
`default_nettype wire
