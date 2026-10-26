// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tango_console138k_sdram162.sdc -- clk_sdram constraint for TANGO_VIDEO_ASCAL builds.
// Appended by build_core.sh when TANGO_VIDEO_ASCAL=1 (after the board's base SDC).
// clk_sdram = Gowin_PLL_SDRAM162 clkout0, 162 MHz: tango_sdram + ascal Avalon.
// clk_sdram_phase (SDRAM_CLK pin) is a PLL output to the external SDRAM; no FPGA fabric is clocked by it.
create_clock -name clk_sdram -period 6.173 [get_nets {clk_sdram}]
// clk_sdram is asynchronous to all other clocks; all crossings are through tango_sdram's Avalon
// waitrequest handshake (clk_sdram → avl_clk = clk_sdram, fully synchronous) and registered
// at the avl_clk/clk_pix boundary inside tango_video (ascal handles its own CDC).
set_clock_groups -asynchronous -group [get_clocks {clk_sdram}] -group [get_clocks {clk clk27 clk_ser clk_pix clk48 spi_sck clk_audio}]
