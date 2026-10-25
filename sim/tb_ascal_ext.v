// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_ascal_ext -- tango_sys with TANGO_VIDEO_ASCAL + TANGO_VIDEO_OPTS + TANGO_SDRAM + TANGO_ASCAL_EXT + TANGO_VIDEO_F1
// and without TANGO_SIM: the REAL ascal (GHDL netlist, sys/ascal_ghdl/) on the core-owned Avalon port, around the
// sim-only core sim/tb_ascal_ext_core.v (an interlaced, then progressive, Genesis-like source whose memory is a word
// memory behind sys/tango_ascal_words.v).  As the Template core's ascal bench otherwise: clk_pix 74.25 MHz (the
// hdmi.sv beam, 1650 x 750), clk50, clk48; no BL616 link, no USB devices.
//   +status=<hex>    the option word from 2 us in (tango_link's status + status_tog, as that bench), e.g. Scanlines
//                    50 % = status[123:122] = 2;  +status2=<hex> +switch_frame=<k>: a second word at output frame k
//                    (the core's status[1] = 1: progressive)
// Writes out_f<N>.ppm (1280 x 720) for frames FIRST_DUMP..NFRAMES-1; prints per-frame stats, ascal's interlace flag
// (dut.u_vid.u_ascal...) as tango_video sees it (m_il / l_il) and the word memory's counters.
// sim/check_ascal_ext.py reads the frames: woven (both fields, in order) while interlaced, no scanlines then;
// one field's lines and the scanlines once progressive.
//==============================================================================
`default_nettype none
`timescale 1ps/1ps
module tb_ascal_ext;
    localparam integer FW = 1650, FH = 750, OW = 1280, OH = 720;
`ifdef NFRAMES_DEF
    localparam integer NFRAMES = `NFRAMES_DEF;
`else
    localparam integer NFRAMES = 14;
`endif
    localparam integer FIRST_DUMP = 2;

    reg clk_pix = 1'b0, clk50 = 1'b0, clk48 = 1'b0;
    always #6734  clk_pix = ~clk_pix;        // 74.25 MHz
    always #10000 clk50   = ~clk50;
    always #10417 clk48   = ~clk48;

    reg [6:0] rcnt = 7'd0;
    wire rst_pix = (rcnt != 7'd64);
    always @(posedge clk_pix) if (rst_pix) rcnt <= rcnt + 7'd1;

    reg [11:0] cx = 12'd0;  reg [10:0] cy = 11'd0;
    always @(posedge clk_pix) begin
        if (rst_pix) begin cx <= 12'd0; cy <= 11'd0; end
        else if (cx == FW - 1) begin cx <= 12'd0; cy <= (cy == FH - 1) ? 11'd0 : cy + 11'd1; end
        else cx <= cx + 12'd1;
    end

    wire [23:0] rgb;
    wire usb1_dp, usb1_dn, usb2_dp, usb2_dn, uart_tx;
    pulldown (usb1_dp); pulldown (usb1_dn); pulldown (usb2_dp); pulldown (usb2_dn);
    wire        SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE;
    wire [12:0] SDRAM_A;  wire [1:0] SDRAM_BA;  wire [15:0] SDRAM_DQ;

    reg [127:0] tb_status = 128'd0;
    initial if ($value$plusargs("status=%h", tb_status)) begin
        #2000000;
        $display("tb: option word set to %032x", tb_status);
        dut.u_link.status = tb_status;
        dut.u_link.status_tog = ~dut.u_link.status_tog;
    end

    tango_sys #(.USB_DEBOUNCE(960_000)) dut (
        .clk50(clk50), .clk_pix(clk_pix), .rst_pix(rst_pix), .cx(cx), .cy(cy), .rgb(rgb),
        .audio_l(), .audio_r(), .clk48(clk48), .lock48(1'b1),
        .usb1_dp(usb1_dp), .usb1_dn(usb1_dn), .usb2_dp(usb2_dp), .usb2_dn(usb2_dn),
        .spi_csn(1'b1), .spi_sck(1'b0), .spi_mosi(1'b0), .spi_miso(), .spi_miso_pad(1'b0), .spi_irq_n(),
        .bl616_jtagsel(1'b0), .jtagsel_n_q(1'b1),
        .osd_shown(), .osd_bank(), .osd_swap_pending(), .joy(), .joy_flags(), .link_xfers(),
        .usb1_bad_crc(), .usb2_bad_crc(), .dbg_rows(),
        .dbg_uart_tx(uart_tx),
        .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQ(SDRAM_DQ),
        .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH), .SDRAM_nCS(SDRAM_nCS), .SDRAM_nCAS(SDRAM_nCAS),
        .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nWE(SDRAM_nWE));

    // ---- frame capture: rgb at clock t is for the beam (cx, cy) of clock t-1
    reg [11:0] px = 12'd0;  reg [10:0] py = 11'd0;
    integer frame = 0, fd = 0, nonblack = 0;
    reg [8*32-1:0] fname;
    always @(posedge clk_pix) begin
        px <= cx;  py <= cy;
        if (!rst_pix && px < OW && py < OH) begin
            if (fd != 0) $fwrite(fd, "%c%c%c", rgb[23:16], rgb[15:8], rgb[7:0]);
            if (rgb != 24'd0) nonblack = nonblack + 1;
        end
        if (!rst_pix && px == FW - 1 && py == FH - 1) begin
            $display("frame %0d: nonblack=%0d  ascal RC=%0d RV=%0d WC=%0d  words wr=%0d rd=%0d rv=%0d outside=%0d  interlaced m_il=%0d l_il=%0d v_sl=%0d",
                     frame, nonblack, dut.dbg_arc, dut.dbg_arv, dut.dbg_awc,
                     dut.u_core.n_wr, dut.u_core.n_rd, dut.u_core.n_rv, dut.u_core.n_outside,
                     dut.u_vid.m_il, dut.u_vid.l_il, dut.u_vid.v_sl);
            $display("  geometry: core frame %0dx%0d  geo_ok=%0d  ascal window h %0d..%0d v %0d..%0d",
                     dut.u_vid.l_hs, dut.u_vid.l_vt, dut.u_vid.geo_ok,
                     dut.u_vid.v_hmin, dut.u_vid.v_hmax, dut.u_vid.v_vmin, dut.u_vid.v_vmax);
            if (fd != 0) $fclose(fd);
            fd = 0;  nonblack = 0;
            frame = frame + 1;
            if (frame == NFRAMES) begin
                $display("TB done: %0d frames; word memory: %0d writes, %0d reads, %0d returned, %0d outside the window",
                         frame, dut.u_core.n_wr, dut.u_core.n_rd, dut.u_core.n_rv, dut.u_core.n_outside);
                $finish;
            end
            if (frame >= FIRST_DUMP) begin
                $sformat(fname, "out_f%0d.ppm", frame);
                fd = $fopen(fname, "wb");
                $fwrite(fd, "P6\n%0d %0d\n255\n", OW, OH);
            end
        end
    end

    reg [127:0] tb_status2 = 128'd0;
    integer     tb_switch = 0;
    initial if ($value$plusargs("status2=%h", tb_status2) && $value$plusargs("switch_frame=%d", tb_switch)) begin
        wait (frame == tb_switch);
        $display("tb: option word set to %032x at the start of frame %0d", tb_status2, frame);
        dut.u_link.status = tb_status2;
        dut.u_link.status_tog = ~dut.u_link.status_tog;
    end
endmodule
`default_nettype wire
