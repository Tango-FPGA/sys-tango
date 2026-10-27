// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_sys_top138 -- the Tango framework's board top for the Tang Console 138K
// (GW5AST-LV138PG484AC1/I0): PLLs, HDMI 720p (60 Hz; a build's own rate with TANGO_HDMI_HTOTAL /
// TANGO_HDMI_VTOTAL, tango_cfg.vh), the BL616 link pins, the USB-A pins, the JTAG
// hand-back, and tango_sys (link + overlay + pads + core interface) around the core's module
// tango_core.  Every Console 138K core -- the menu core included -- is built with this top.
//
// Clock / HDMI / serialiser structure (bench-proven at 720p on this board):
//   clk 50 (V22) --pll_c27--> clk27 --pll_c1485--> clk_ser 371.25 + clk_pix 74.25 (a TANGO_HDMI_PLL_<f> build: 5 x f
//   and f, tango_cfg.vh -- the audio accumulator, the ACR's N / CTS and the SDC follow)
//   clk 50 (V22) --pll_c48--> clk48 (USB hosts)
//   pixel reset: async assert on !lock, sync release 8 clk_pix later
// BL616 link pins (tango_console138k.cst; Sipeed's Console 138K schematic; Gowin GW5AST-138B PBGA484A pin file):
//   spi_csn T13 (TMS)  spi_sck V12 (TCK)  spi_mosi R13 (TDI)  spi_miso U13 (TDO)  spi_irqn U15
//   bl616_jtagsel V14 (BL616 GPIO28)      jtagseln: JTAGSEL_N from logic ("JTAG hand-back", below)
// The four JTAG balls are dual-purpose: the build sets -use_jtag_as_gpio 1.
//==============================================================================
`include "tango_cfg.vh"
`default_nettype none
module tango_sys_top138 #(
    parameter integer DBG = 0          // 1: tango_link_dbg_view blocks over the picture (debug build)
) (
    input  wire        clk,            // 50 MHz osc, V22
    output wire        tmds_clk_p, tmds_clk_n,
    output wire [2:0]  tmds_d_p, tmds_d_n,
    // BL616 link
    input  wire        spi_csn,
    input  wire        spi_sck,
    input  wire        spi_mosi,
    inout  wire        spi_miso,       // output; read back through its input buffer for debug
    output wire        spi_irqn,
    input  wire        bl616_jtagsel,
    output wire        jtagseln,       // JTAGSEL_N from logic, see "JTAG hand-back"
    // USB-A host ports
    inout  wire        usb1_dp, usb1_dn,     // USB-A1 M15 / M16
    inout  wire        usb2_dp, usb2_dn      // USB-A2 H13 / G13
`ifdef TANGO_SDRAM
    ,
    // the SDRAM module's pins (tango_console138k_sdram.cst), to the core; SDRAM_CKE has no pin
    output wire        SDRAM_CLK,
    output wire [12:0] SDRAM_A,
    output wire [1:0]  SDRAM_BA,
    inout  wire [15:0] SDRAM_DQ,
    output wire        SDRAM_DQML, SDRAM_DQMH,
    output wire        SDRAM_nCS, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nWE
`endif
`ifdef TANGO_VIDEO_DDR3
    ,
    // TANGO_VIDEO_DDR3: the DDR3 pins (tango_console138k_ddr3.cst: Sipeed's DDR3 test constraints, as the bring-up test's)
    output wire [14:0] ddr_addr,
    output wire [2:0]  ddr_bank,
    output wire        ddr_cs, ddr_ras, ddr_cas, ddr_we,
    output wire        ddr_ck, ddr_ck_n, ddr_cke, ddr_odt, ddr_reset_n,
    output wire [3:0]  ddr_dm,
    inout  wire [31:0] ddr_dq,
    inout  wire [3:0]  ddr_dqs, ddr_dqs_n
`endif
`ifdef TANGO_FW_SDRAM
    ,
    // SDRAM module pins (tango_console138k_sdram.cst); framework owns the controller (TANGO_ASCAL_EXT: the core's,
    // through the TANGO_SDRAM ports above).
    // SDRAM_CLK is driven here from Gowin_PLL_SDRAM clkout1 (phase-shifted 96 MHz).
    output wire        SDRAM_CLK,
    output wire [12:0] SDRAM_A,
    output wire [1:0]  SDRAM_BA,
    inout  wire [15:0] SDRAM_DQ,
    output wire        SDRAM_DQML, SDRAM_DQMH,
    // SDRAM_CKE: no ball on the Console 138K SDRAM module (tied high on the module)
    output wire        SDRAM_nCS, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nWE
`endif
);
    // ---- JTAG hand-back: the BL616 takes the JTAG balls back to reprogram a running core.  JTAGSEL_N selects what the
    // four JTAG balls are when they are set as GPIO (-use_jtag_as_gpio 1): 0 = configuration (JTAG), 1 = user I/O
    // (UG290-2.7.6E Table 4-2, Table 7-3); SUG1018-1.6E 2.10.2 (JTAGSEL_N Net Constraints) gives a net of the design
    // control of it, NET_LOC "<net>" V_JTAGSELN.  Here that net is the top-level port jtagseln = NOT bl616_jtagsel:
    // the BL616 drives V14 high to program (JTAGSEL_N low), low to run the link; tango_console138k.cst has the
    // constraints (NET_LOC ... V_JTAGSELN, PULL_MODE=DOWN, and IO_LOC T16, because place and route also puts the port
    // on a user I/O).  Driving JTAGSEL_N from an internal net instead left the MISO pad (TDO/U13) not following the
    // fabric on silicon, although Table 4-2 says JTAGSEL_N = 1 makes the JTAG balls GPIO.
    assign jtagseln = ~bl616_jtagsel;
    wire jtagsel_n = ~bl616_jtagsel;
    reg jtagsel_n_q /* synthesis syn_preserve = 1 */;
    always @(posedge clk_pix) jtagsel_n_q <= jtagsel_n;

    // ---- clocks (PLL_INIT PLLs) ------------------------------------------------
    wire clk27, clk_ser, clk_pix, clk48;
    wire lk27, lkh, lk48;
    pll_c27   u_pll27 (.clkin(clk),   .init_clk(clk), .clkout0(clk27), .lock(lk27));
`ifdef TANGO_HDMI_RETUNE
    // The retune at core start: MDIV from tango_pll_retune -- the preset (native) or 55 + 0/8 = 74.25 MHz (fixed),
    // switched by SYS 0x70's native bit through tango_vsync; at configuration: the preset if TANGO_VSYNC_DEFAULT is 1.
    // A retune is a CLOCK PAUSE: clk_pix = the PLL's raw CLKOUT1 through a DCE whose enable (from two negedge flops of the
    // raw clock, so it changes only while that clock is low) drops before PLL_INIT's I_RST and rises 1.3 ms after the new
    // lock -- the clk_pix domain stops cleanly and resumes at the new frequency, nothing in it is reset; the OSER10
    // serialiser is held in reset meanwhile (clk_ser runs ungated through the re-lock) and released 2 clk_pix after the resume.
    wire       rt_irst, rt_pll_native, rt_tog_pix, rt_req_native, rt_gate_off, clk_pix_raw;
    wire [6:0] rt_mdsel;  wire [2:0] rt_mdsel_frac;
    pll_c1485 #(.MDIV(`TANGO_HDMI_MDIV), .MDIV_FRAC(`TANGO_HDMI_MDIV_FRAC), .DYN_MDIV("TRUE"))
              u_pllh  (.clkin(clk27), .init_clk(clk), .clkout0(clk_ser), .clkout1(clk_pix_raw), .lock(lkh),
                       .irst(rt_irst), .mdsel(rt_mdsel), .mdsel_frac(rt_mdsel_frac));
    tango_pll_retune #(.MDIV_N(`TANGO_HDMI_MDIV), .FRAC_N(`TANGO_HDMI_MDIV_FRAC), .NATIVE0(`TANGO_VSYNC_DEFAULT)) u_retune (
        .clk(clk), .req_tog(rt_tog_pix), .req_native(rt_req_native), .lock(lkh), .irst(rt_irst), .mdsel(rt_mdsel),
        .mdsel_frac(rt_mdsel_frac), .native(rt_pll_native), .gate_off(rt_gate_off), .n_retunes());
    (* syn_preserve = 1 *) reg [1:0] rt_ce_s = 2'b11;
    always @(negedge clk_pix_raw) rt_ce_s <= {rt_ce_s[0], ~rt_gate_off};
    DCE u_dce_pix (.CLKIN(clk_pix_raw), .CE(rt_ce_s[1]), .CLKOUT(clk_pix));
    generate if (1) begin : g_retune_checks
`ifdef TANGO_HDMI_RETUNE_NEEDS_A_PRESET
        tango_hdmi_retune_needs_a_TANGO_HDMI_PLL_preset u_error ();
`endif
`ifdef TANGO_HDMI_RETUNE_NEEDS_VSYNC_ADJUST
        tango_hdmi_retune_needs_TANGO_VSYNC_ADJUST u_error2 ();
`endif
    end endgenerate
`else
    pll_c1485 #(.MDIV(`TANGO_HDMI_MDIV), .MDIV_FRAC(`TANGO_HDMI_MDIV_FRAC))   // tango_cfg.vh: 55 + 0/8 = 74.25 MHz unless
              u_pllh  (.clkin(clk27), .init_clk(clk), .clkout0(clk_ser), .clkout1(clk_pix), .lock(lkh),   // TANGO_HDMI_PLL_<f>
                       .irst(1'b0), .mdsel(7'd0), .mdsel_frac(3'd0));
`endif
    pll_c48   u_pll48 (.clkin(clk),   .init_clk(clk), .clkout0(clk48), .lock(lk48));
`ifdef TANGO_FW_SDRAM
    // 96 MHz SDRAM PLL (TANGO_VIDEO_ASCAL: framework owns the SDRAM, ascal uses it for frame buffers)
    // clkout0 = 96 MHz fabric (clk_sdram); clkout1 = 96 MHz + 320 deg (SDRAM_CLK pin)
    wire clk_sdram, clk_sdram_phase, lock_sdram;
    Gowin_PLL_SDRAM u_pll_sdram (
        .clkin(clk27), .lock(lock_sdram), .clkout0(clk_sdram), .clkout1(clk_sdram_phase));
    assign SDRAM_CLK = clk_sdram_phase;
`endif
`ifdef TANGO_VIDEO_DDR3
    // ---- the DDR3 (as the bring-up test's ddr3/test/tango_ddr3_test_top.v): pll_ddr3 pinned at PLL_L[0] (INS_LOC
    // "u_plld/u_pllm/PLL_inst", tango_console138k_ddr3.cst), CLKOUT2 = memory_clk 400 MHz gated by the IP's pll_stop;
    // the Gowin DDR3 Memory Interface IP generated by the build (ddr3/tango_ddr3_ip.tcl), its clk the raw 50 MHz pin
    // (IPUG281 4.4.2, as Sipeed's test); its reset released 255 clocks after the PLL locks (Sipeed's test top.v)
    wire memory_clk, lkd, pll_stop;
    pll_ddr3 u_plld (.clkin(clk), .init_clk(clk), .enclk2(pll_stop), .clkout0(), .clkout2(memory_clk), .lock(lkd));
    reg  [1:0] lkd_s = 2'b00;
    reg  [7:0] ddr_rcnt = 8'd0;
    reg        ddr_ip_rst_n = 1'b0;
    always @(posedge clk) begin
        lkd_s <= {lkd_s[0], lkd};
        if (!lkd_s[1]) begin ddr_rcnt <= 8'd0; ddr_ip_rst_n <= 1'b0; end
        else if (ddr_rcnt != 8'hFF) ddr_rcnt <= ddr_rcnt + 1'b1;
        else ddr_ip_rst_n <= 1'b1;
    end
    wire         clk_ddr, ddr_rst, ddr_calib;
    wire         ddr_cmd_ready, ddr_cmd_en, ddr_wr_data_rdy, ddr_wr_data_en, ddr_rd_data_valid, ddr_rd_data_end;
    wire [2:0]   ddr_cmd;
    wire [28:0]  ddr_iaddr;
    wire [255:0] ddr_wr_data, ddr_rd_data;
    wire [31:0]  ddr_wr_data_mask;
    DDR3_Memory_Interface_Top u_ddr3 (
        .clk(clk), .pll_stop(pll_stop), .memory_clk(memory_clk), .pll_lock(lkd), .rst_n(ddr_ip_rst_n),
        .clk_out(clk_ddr), .ddr_rst(ddr_rst), .init_calib_complete(ddr_calib),
        .cmd_ready(ddr_cmd_ready), .cmd(ddr_cmd), .cmd_en(ddr_cmd_en), .addr(ddr_iaddr),
        .wr_data_rdy(ddr_wr_data_rdy), .wr_data(ddr_wr_data), .wr_data_en(ddr_wr_data_en), .wr_data_end(ddr_wr_data_en),
        .wr_data_mask(ddr_wr_data_mask),
        .rd_data(ddr_rd_data), .rd_data_valid(ddr_rd_data_valid), .rd_data_end(ddr_rd_data_end),
        .sr_req(1'b0), .ref_req(1'b0), .sr_ack(), .ref_ack(), .burst(1'b1),
        .O_ddr_addr(ddr_addr), .O_ddr_ba(ddr_bank), .O_ddr_cs_n(ddr_cs), .O_ddr_ras_n(ddr_ras),
        .O_ddr_cas_n(ddr_cas), .O_ddr_we_n(ddr_we), .O_ddr_clk(ddr_ck), .O_ddr_clk_n(ddr_ck_n),
        .O_ddr_cke(ddr_cke), .O_ddr_odt(ddr_odt), .O_ddr_reset_n(ddr_reset_n), .O_ddr_dqm(ddr_dm),
        .IO_ddr_dq(ddr_dq), .IO_ddr_dqs(ddr_dqs), .IO_ddr_dqs_n(ddr_dqs_n));
`endif

    reg [7:0] rstp_sr = 8'd0;
`ifdef TANGO_HDMI_RETUNE
    // TANGO_HDMI_RETUNE: rst_pix at power-up only -- a retune drops PLL_INIT's lock too, but the clk_pix domain is paused,
    // not reset
    reg  rt_por = 1'b0;
    always @(posedge clk) if (lkh) rt_por <= 1'b1;
    wire lk_rst = lkh | rt_por;
    always @(posedge clk_pix or negedge lk_rst)
        if (!lk_rst) rstp_sr <= 8'd0; else rstp_sr <= {rstp_sr[6:0], 1'b1};
`else
    always @(posedge clk_pix or negedge lkh)
        if (!lkh) rstp_sr <= 8'd0; else rstp_sr <= {rstp_sr[6:0], 1'b1};
`endif
    // rst_pix: hold until pll_c1485 has been locked for 8 clk_pix cycles.
    // TANGO_VIDEO_ASCAL: also hold until Gowin_PLL_SDRAM has locked (lock_sdram), so tango_sdram's
    // init sequence always runs with a stable clk_sdram.
`ifdef TANGO_FW_SDRAM
    wire rst_pix = ~rstp_sr[7] | ~lock_sdram;
`else
    // no SDRAM PLL here: no TANGO_VIDEO_ASCAL, TANGO_ASCAL_EXT (the core's controller has its own clocks), or
    // TANGO_VIDEO_DDR3 without TANGO_NES_SDRAM (tango_ddr3 waits for calibration itself)
    wire rst_pix = ~rstp_sr[7];
`endif

    // ---- SCK as a clock: an internal net, so the cst can put it on a global clock (CLOCK_LOC does
    // not take an input port, CT1020)
    wire spi_sck_clk /* synthesis syn_keep = 1 */;
    assign spi_sck_clk = spi_sck;

    // ---- the framework fabric + the core ---------------------------------------
    wire [11:0] cx;  wire [10:0] cy;
`ifdef TANGO_VSYNC_ADJUST
    // the raster's totals at runtime (tango_vsync.v): hdmi.sv's current frame's -> tango_sys, and the
    // next frame's (tango_vsync) -> hdmi.sv, which takes them at its wrap
    wire [11:0] vt_cur_w, vt_nxt_w;  wire [10:0] vt_cur_h, vt_nxt_h;
`endif
    wire [23:0] rgb;
    wire [15:0] core_l, core_r;
    wire miso_q;
    wire sys_spi_irqn;  // the link's IRQ: level, low = pending
    wire [15:0] link_xfers;
`ifdef TANGO_VIDEO_ASCAL
    // U15 is the link IRQ under the Tango BL616 firmware, whose link task sleeps until it (main.c, no
    // timeout), and the Sipeed debugger's UART RX when the BL616 is the PC's USB debugger (JTAG loads).
    // The ascal instruments' UART (tango_sys) drives it until the link's first transfer made while the
    // BL616 is not asking for JTAG: the firmware holds GPIO28 low while it runs the link; as the debugger
    // that line (V14) idles high, and JTAG traffic on the shared SPI/JTAG balls is ignored.
    wire dbg_uart_tx;
    reg  [1:0]  link_jsel_s = 2'b11;
    reg  [15:0] link_xfers_q = 16'd0;
    reg         link_seen = 1'b0;
    always @(posedge clk_pix) begin
        link_jsel_s  <= {link_jsel_s[0], bl616_jtagsel};
        link_xfers_q <= link_xfers;
        if (link_xfers != link_xfers_q && !link_jsel_s[1]) link_seen <= 1'b1;
    end
    assign spi_irqn = link_seen ? sys_spi_irqn : dbg_uart_tx;
`else
    assign spi_irqn = sys_spi_irqn;
`endif
    tango_sys u_sys (
        .clk50(clk), .jtagsel_n_q(jtagsel_n_q),
        .clk_pix(clk_pix), .rst_pix(rst_pix), .cx(cx), .cy(cy), .rgb(rgb), .audio_l(core_l), .audio_r(core_r),
        .clk48(clk48), .lock48(lk48),
        .usb1_dp(usb1_dp), .usb1_dn(usb1_dn), .usb2_dp(usb2_dp), .usb2_dn(usb2_dn),
        .spi_csn(spi_csn), .spi_sck(spi_sck_clk), .spi_mosi(spi_mosi),
        .spi_miso(miso_q), .spi_miso_pad(spi_miso), .spi_irq_n(sys_spi_irqn), .bl616_jtagsel(bl616_jtagsel),
        .osd_shown(), .osd_bank(), .osd_swap_pending(), .joy(), .joy_flags(), .link_xfers(link_xfers),
        .usb1_bad_crc(), .usb2_bad_crc(), .dbg_rows(dbg_rows)
`ifdef TANGO_VSYNC_ADJUST
        , .vt_cur_w(vt_cur_w), .vt_cur_h(vt_cur_h), .vt_nxt_w(vt_nxt_w), .vt_nxt_h(vt_nxt_h)
`ifdef TANGO_HDMI_RETUNE
        , .pll_native(rt_pll_native), .rt_tog(rt_tog_pix), .rt_native(rt_req_native)
`endif
`endif
`ifdef TANGO_SDRAM
        , .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQ(SDRAM_DQ), .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH), .SDRAM_nCS(SDRAM_nCS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nWE(SDRAM_nWE)
`endif
`ifdef TANGO_VIDEO_ASCAL
`ifdef TANGO_ASCAL_EXT
        , .dbg_uart_tx(dbg_uart_tx)
`elsif TANGO_VIDEO_DDR3
        , .dbg_uart_tx(dbg_uart_tx),
          .clk_ddr(clk_ddr), .ddr_rst(ddr_rst), .ddr_calib(ddr_calib),
          .ddr_cmd_ready(ddr_cmd_ready), .ddr_cmd_en(ddr_cmd_en), .ddr_cmd(ddr_cmd), .ddr_addr(ddr_iaddr),
          .ddr_wr_data_rdy(ddr_wr_data_rdy), .ddr_wr_data_en(ddr_wr_data_en), .ddr_wr_data(ddr_wr_data),
          .ddr_wr_data_mask(ddr_wr_data_mask), .ddr_rd_data_valid(ddr_rd_data_valid), .ddr_rd_data(ddr_rd_data)
`ifdef TANGO_FW_SDRAM
        // TANGO_VIDEO_DDR3 + TANGO_NES_SDRAM: tango_sdram for the NES's ports A/B
        , .clk_sdram(clk_sdram),
          .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQ(SDRAM_DQ),
          .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH),
          .SDRAM_nCS(SDRAM_nCS), .SDRAM_nCAS(SDRAM_nCAS),
          .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nWE(SDRAM_nWE)
`endif
`else
        // clk_sdram (fabric 96 MHz) drives tango_sdram; SDRAM_CLK pin driven above from PLL clkout1
        , .clk_sdram(clk_sdram),
          .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQ(SDRAM_DQ),
          .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH),
          .SDRAM_nCS(SDRAM_nCS), .SDRAM_nCAS(SDRAM_nCAS),
          .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nWE(SDRAM_nWE),
          .dbg_uart_tx(dbg_uart_tx)
`endif
`endif
        );
    wire [143:0] dbg_rows;
    wire [23:0] rgb_hdmi;
    generate if (DBG) begin : g_dbg
        tango_link_dbg_view #(.N_ROWS(18)) u_dbg (.clk(clk_pix), .cx(cx), .cy(cy), .rgb_in(rgb),
            .rows(dbg_rows), .rgb_out(rgb_hdmi));
    end else begin : g_nodbg
        assign rgb_hdmi = rgb;
    end endgenerate
    assign spi_miso = bl616_jtagsel ? 1'bz : miso_q;

    // ---- clk_audio 48 kHz: +16 mod 12375 on clk_pix (74.25 MHz x 16 / 12375 = 96 kHz
    // toggles); a TANGO_HDMI_PLL_<f> build: its own exact pair (tango_cfg.vh TANGO_HDMI_AUD_W / _INC / _MOD, e.g. 74.5875 MHz:
    // +64 mod 49725), so the delivered rate stays EXACTLY 48 kHz with the ACR's N / CTS below
`ifdef TANGO_HDMI_RETUNE
    // TANGO_HDMI_RETUNE: the pair follows the PLL (quasi-static: rt_pll_native changes only while clk_pix is stopped)
    localparam integer AW = 16;
    wire [AW-1:0] AINC = rt_pll_native ? 16'(`TANGO_HDMI_AUD_INC) : 16'd16;
    wire [AW-1:0] AMOD = rt_pll_native ? 16'(`TANGO_HDMI_AUD_MOD) : 16'd12375;
`else
    localparam integer AW = `TANGO_HDMI_AUD_W;
    localparam [AW-1:0] AINC = `TANGO_HDMI_AUD_INC, AMOD = `TANGO_HDMI_AUD_MOD;
`endif
    reg [AW-1:0] aacc = {AW{1'b0}};
    (* syn_preserve = 1 *) reg clk_audio = 1'b0;
    always @(posedge clk_pix) begin
        if (aacc >= AMOD - AINC) begin
            aacc <= aacc + AINC - AMOD; clk_audio <= ~clk_audio;
        end else aacc <= aacc + AINC;
    end
    // the core's samples (clk_pix) are held while clk_audio is high and change only on the clk_pix
    // edge that drops it, so packet_picker (clocked by clk_audio's rise) sees a stable pair
    reg  [15:0] aud_l = 16'd0, aud_r = 16'd0;
    always @(posedge clk_pix)
        if (aacc >= AMOD - AINC && clk_audio) begin aud_l <= core_l; aud_r <= core_r; end
    wire [15:0] audio_word [1:0];
    assign audio_word[0] = aud_l;  assign audio_word[1] = aud_r;

    // ---- hdl-util hdmi + serialiser ------------------------------------------------
    // The raster's totals and VIC from tango_cfg.vh (TANGO_HDMI_HTOTAL / _VTOTAL / _VIC; defaults 1650 x 750, VIC 4 =
    // every build as before).  tango_sys gives the same pair to every other consumer.
    // Out of the range tango_cfg.vh documents: refuse the build.
    generate if (`TANGO_HDMI_HTOTAL < 1626 || `TANGO_HDMI_HTOTAL > 1720 || `TANGO_HDMI_VTOTAL < 740 ||
                 `TANGO_HDMI_VTOTAL > 790) begin : g_bad_totals
        tango_hdmi_totals_out_of_range u_error ();
    end endgenerate
    wire [9:0] sym0, sym1, sym2;
    hdmi #(.AUDIO_RATE(48000), .AUDIO_BIT_WIDTH(16), .IT_CONTENT(1'b1),
`ifdef TANGO_HDMI_RETUNE
           .ACR_N(12288),                       // retune: N 12288 at both clocks, the CTS at run time (acr_cts_rt below)
`elsif TANGO_HDMI_PLL_SET
           .ACR_N(`TANGO_HDMI_ACR_N), .ACR_CTS(`TANGO_HDMI_ACR_CTS),   // the preset's exact 48 kHz pair (hdmi.sv modification 4)
`endif
           .FRAME_WIDTH(`TANGO_HDMI_HTOTAL), .FRAME_HEIGHT(`TANGO_HDMI_VTOTAL), .VIC(`TANGO_HDMI_VIC)) u_hdmi (
        .frame_height_adj(4'd0), .clk_pixel_x5(clk_ser), .clk_pixel(clk_pix),
        .clk_audio(clk_audio), .reset(rst_pix), .rgb(rgb_hdmi), .audio_sample_word(audio_word),
`ifdef TANGO_HDMI_RETUNE
        .acr_cts_rt(rt_pll_native ? 20'(`TANGO_HDMI_CTS12288) : 20'd148500),   // 74.25e6 x 12288 / (128 x 48000) = 148500
`endif
        .tmds_ch0(sym0), .tmds_ch1(sym1), .tmds_ch2(sym2), .cx(cx), .cy(cy),
`ifdef TANGO_VSYNC_ADJUST
        .next_frame_width(vt_nxt_w), .next_frame_height(vt_nxt_h), .frame_width(vt_cur_w), .frame_height(vt_cur_h),
`else
        .frame_width(), .frame_height(),
`endif
        .screen_width(), .screen_height());
`ifdef TANGO_HDMI_RETUNE
    // the serialiser through a retune: reset asserted at once when the pause starts (clocks still running), released 2 clk_pix
    // after the resume
    reg [1:0] rt_ser_rst = 2'b11;
    always @(posedge clk_pix or posedge rt_gate_off)
        if (rt_gate_off) rt_ser_rst <= 2'b11; else rt_ser_rst <= {rt_ser_rst[0], 1'b0};
    wire ser_resetn = ~(rst_pix | rt_ser_rst[1]);
`else
    wire ser_resetn = ~rst_pix;
`endif
    tango_tmds_ser u_ser (
        .clk_pix(clk_pix), .clk_ser(clk_ser), .resetn(ser_resetn),
        .sym_ch0(sym0), .sym_ch1(sym1), .sym_ch2(sym2),
        .tmds_clk_p(tmds_clk_p), .tmds_clk_n(tmds_clk_n),
        .tmds_d_p(tmds_d_p), .tmds_d_n(tmds_d_n));
endmodule
`default_nettype wire
