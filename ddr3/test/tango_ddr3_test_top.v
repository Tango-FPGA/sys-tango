// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_test_top -- the DDR3 bring-up test (ddr3/test) for the Tang Console 138K.
// Standalone: no framework, no BL616 link; loaded over JTAG (the JTAG balls stay JTAG: no -use_jtag_as_gpio).
//   clocks  the 50 MHz board clock (V22) -> pll_c27 -> pll_c1485 (720p: 371.25 / 74.25 MHz) for HDMI, exactly as
//           the framework (board/console138k/tango_sys_top138.v); -> pll_ddr3 at PLL_L[0]:
//           CLKOUT2 = memory_clk 400 MHz gated by the IP's pll_stop (IPUG281-2.6E 4.4.4), CLKOUT0 = clk_ref 100 MHz
//           free running (the tester's wall clock);
//   DDR3    the Gowin DDR3 Memory Interface IP, generated at build time (ddr3/tango_ddr3_ip.tcl: 1:4, 400 MHz, x32,
//           Sipeed's parameters); clk = the 50 MHz board clock (IPUG281 4.4.2), clk_out = clk_x1 100 MHz;
//   logic   tango_ddr3_test_core (the tester, the results screen, the UART report);
//   HDMI    hdmi.sv + tango_tmds_ser, as the framework; audio silent.
// Pins: tango_ddr3_test.cst (Sipeed's DDR3 constraints, the board's HDMI, U15 UART TX, V13 LED).
//==============================================================================
`default_nettype none
module tango_ddr3_test_top (
    input  wire        clk,
    output wire        tmds_clk_p, tmds_clk_n,
    output wire [2:0]  tmds_d_p, tmds_d_n,
    output wire        uart_tx,
    output wire        led,
    output wire [14:0] ddr_addr,
    output wire [2:0]  ddr_bank,
    output wire        ddr_cs, ddr_ras, ddr_cas, ddr_we,
    output wire        ddr_ck, ddr_ck_n, ddr_cke, ddr_odt, ddr_reset_n,
    output wire [3:0]  ddr_dm,
    inout  wire [31:0] ddr_dq,
    inout  wire [3:0]  ddr_dqs, ddr_dqs_n
);
    // ---- clocks
    wire clk27, clk_ser, clk_pix, lk27, lkh;
    pll_c27   u_pll27 (.clkin(clk),   .init_clk(clk), .clkout0(clk27), .lock(lk27));
    pll_c1485 u_pllh  (.clkin(clk27), .init_clk(clk), .clkout0(clk_ser), .clkout1(clk_pix), .lock(lkh),
                       .irst(1'b0), .mdsel(7'd0), .mdsel_frac(3'd0));   // static MDIV (the vsync_adjust retune ports unused)
    wire clk_ref, memory_clk, lkd, pll_stop;
    pll_ddr3  u_plld  (.clkin(clk), .init_clk(clk), .enclk2(pll_stop), .clkout0(clk_ref), .clkout2(memory_clk),
                       .lock(lkd));
    reg [7:0] rstp_sr = 8'd0;
    always @(posedge clk_pix or negedge lkh)
        if (!lkh) rstp_sr <= 8'd0; else rstp_sr <= {rstp_sr[6:0], 1'b1};
    wire rst_pix = ~rstp_sr[7];

    // ---- the DDR3 IP
    wire         clk_x1, ddr_rst, init_calib_complete, ip_rst_n;
    wire         cmd_ready, cmd_en, wr_data_rdy, wr_data_en, rd_data_valid, rd_data_end;
    wire [2:0]   cmd;
    wire [28:0]  addr;
    wire [255:0] wr_data, rd_data;
    wire [31:0]  wr_data_mask;
    DDR3_Memory_Interface_Top u_ddr3 (
        .clk(clk), .pll_stop(pll_stop), .memory_clk(memory_clk), .pll_lock(lkd), .rst_n(ip_rst_n),
        .clk_out(clk_x1), .ddr_rst(ddr_rst), .init_calib_complete(init_calib_complete),
        .cmd_ready(cmd_ready), .cmd(cmd), .cmd_en(cmd_en), .addr(addr),
        .wr_data_rdy(wr_data_rdy), .wr_data(wr_data), .wr_data_en(wr_data_en), .wr_data_end(wr_data_en),
        .wr_data_mask(wr_data_mask),
        .rd_data(rd_data), .rd_data_valid(rd_data_valid), .rd_data_end(rd_data_end),
        .sr_req(1'b0), .ref_req(1'b0), .sr_ack(), .ref_ack(), .burst(1'b1),
        .O_ddr_addr(ddr_addr), .O_ddr_ba(ddr_bank), .O_ddr_cs_n(ddr_cs), .O_ddr_ras_n(ddr_ras),
        .O_ddr_cas_n(ddr_cas), .O_ddr_we_n(ddr_we), .O_ddr_clk(ddr_ck), .O_ddr_clk_n(ddr_ck_n),
        .O_ddr_cke(ddr_cke), .O_ddr_odt(ddr_odt), .O_ddr_reset_n(ddr_reset_n), .O_ddr_dqm(ddr_dm),
        .IO_ddr_dq(ddr_dq), .IO_ddr_dqs(ddr_dqs), .IO_ddr_dqs_n(ddr_dqs_n));

    // ---- the test
    wire [11:0] cx; wire [10:0] cy;
    wire [23:0] rgb;
    tango_ddr3_test_core u_core (
        .clk50(clk), .clk_ref(clk_ref), .clk_x1(clk_x1), .clk_pix(clk_pix), .rst_pix(rst_pix),
        .ddr_pll_lock(lkd), .hdmi_lock(lkh), .pll_stop(pll_stop), .ip_rst_n(ip_rst_n),
        .init_calib_complete(init_calib_complete), .ddr_rst(ddr_rst),
        .cmd_ready(cmd_ready), .cmd_en(cmd_en), .cmd(cmd), .addr(addr),
        .wr_data_rdy(wr_data_rdy), .wr_data_en(wr_data_en), .wr_data(wr_data), .wr_data_mask(wr_data_mask),
        .rd_data_valid(rd_data_valid), .rd_data(rd_data),
        .cx(cx), .cy(cy), .rgb(rgb), .freeze(1'b0), .uart_tx(uart_tx), .led(led));

    // ---- HDMI (tango_sys_top138.v), audio silent
    reg [13:0] aacc = 14'd0;
    (* syn_preserve = 1 *) reg clk_audio = 1'b0;
    always @(posedge clk_pix) begin
        if (aacc >= 14'd12375 - 14'd16) begin aacc <= aacc + 14'd16 - 14'd12375; clk_audio <= ~clk_audio; end
        else aacc <= aacc + 14'd16;
    end
    wire [15:0] audio_word [1:0];
    assign audio_word[0] = 16'd0; assign audio_word[1] = 16'd0;
    wire [9:0] sym0, sym1, sym2;
    hdmi #(.AUDIO_RATE(48000), .AUDIO_BIT_WIDTH(16), .IT_CONTENT(1'b1)) u_hdmi (
        .frame_height_adj(4'd0), .clk_pixel_x5(clk_ser), .clk_pixel(clk_pix),
        .clk_audio(clk_audio), .reset(rst_pix), .rgb(rgb), .audio_sample_word(audio_word),
        .tmds_ch0(sym0), .tmds_ch1(sym1), .tmds_ch2(sym2), .cx(cx), .cy(cy),
        .frame_width(), .frame_height(), .screen_width(), .screen_height());
    tango_tmds_ser u_ser (
        .clk_pix(clk_pix), .clk_ser(clk_ser), .resetn(~rst_pix),
        .sym_ch0(sym0), .sym_ch1(sym1), .sym_ch2(sym2),
        .tmds_clk_p(tmds_clk_p), .tmds_clk_n(tmds_clk_n),
        .tmds_d_p(tmds_d_p), .tmds_d_n(tmds_d_n));
endmodule
`default_nettype wire
