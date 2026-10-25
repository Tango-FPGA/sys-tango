// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tango_sys_top138_dbg -- tango_sys_top138 with DBG = 1: the link-debug blocks of tango_link_dbg_view
// at the top-left (row meanings: tango_link.v "debug (never reset)", tango_sys.v dbg_rows).  Same
// ports; its constraints name the internal nets under the wrapper instance u/.
`include "tango_cfg.vh"
`default_nettype none
module tango_sys_top138_dbg (
    input  wire clk,
    output wire tmds_clk_p, tmds_clk_n,
    output wire [2:0] tmds_d_p, tmds_d_n,
    input  wire spi_csn, spi_sck, spi_mosi,
    inout  wire spi_miso,
    output wire spi_irqn,
    input  wire bl616_jtagsel,
    output wire jtagseln,
    inout  wire usb1_dp, usb1_dn, usb2_dp, usb2_dn
`ifdef TANGO_SDRAM
    , output wire SDRAM_CLK, output wire [12:0] SDRAM_A, output wire [1:0] SDRAM_BA, inout wire [15:0] SDRAM_DQ,
    output wire SDRAM_DQML, SDRAM_DQMH, SDRAM_nCS, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nWE
`endif
);
    tango_sys_top138 #(.DBG(1)) u (.clk(clk), .tmds_clk_p(tmds_clk_p), .tmds_clk_n(tmds_clk_n),
        .tmds_d_p(tmds_d_p), .tmds_d_n(tmds_d_n), .spi_csn(spi_csn), .spi_sck(spi_sck), .spi_mosi(spi_mosi),
        .spi_miso(spi_miso), .spi_irqn(spi_irqn), .bl616_jtagsel(bl616_jtagsel), .jtagseln(jtagseln),
        .usb1_dp(usb1_dp), .usb1_dn(usb1_dn), .usb2_dp(usb2_dp), .usb2_dn(usb2_dn)
`ifdef TANGO_SDRAM
        , .SDRAM_CLK(SDRAM_CLK), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQ(SDRAM_DQ), .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH), .SDRAM_nCS(SDRAM_nCS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nWE(SDRAM_nWE)
`endif
        );
endmodule
`default_nettype wire
