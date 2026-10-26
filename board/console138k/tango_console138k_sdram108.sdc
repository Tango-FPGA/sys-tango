// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tango_console138k_sdram108.sdc -- clk_sdram constraint for TANGO_VIDEO_ASCAL builds.
// Appended by build_core.sh when TANGO_VIDEO_ASCAL=1 (after the board's base SDC).
// clk_sdram = Gowin_PLL_SDRAM clkout0.  The period here is the ROUTER TARGET, not the PLL freq.
// Overconstrain (tighter than hardware) to force compact placement: at 108 MHz the router leaves
// a 2.452 ns routing gap in tango_sdram with TANGO_NES_SDRAM; 120 MHz forces it tighter.
// PLL runs at 96 MHz (ODIV=9); build scripts pass --relax clk_sdram:96.0 to sta_gate.py.
// Fmax with TANGO_NES_SDRAM = 100.994 MHz > 96 MHz threshold -> STA PASS; R33 slack = +0.515 ns.
// Frequency history: 162 MHz (STA -1.456 ns), 126 MHz (STA -0.647 ns), 108 MHz (r8 STA PASS
//   without TANGO_NES_SDRAM), 108 MHz + TANGO_NES_SDRAM (STA -0.660 ns),
//   120 MHz overconstrain + ab_col_r + relax 108 MHz (false pass: TANGO_NES_SDRAM not compiled in),
//   96 MHz PLL + 120 MHz overconstrain + ab_col_r + relax 96 MHz (STA PASS, 100.994 MHz actual).
// clk_sdram_phase (SDRAM_CLK pin) is a PLL output to the external SDRAM; no FPGA fabric clocked.
create_clock -name clk_sdram -period 8.333 [get_nets {clk_sdram}]
// clk_sdram is asynchronous to all other clocks; all crossings are through tango_sdram's
// Avalon waitrequest handshake (clk_sdram → avl_clk = clk_sdram, fully synchronous) and
// registered at the avl_clk/clk_pix boundary inside tango_video (ascal handles its CDC).
set_clock_groups -asynchronous -group [get_clocks {clk_sdram}] -group [get_clocks {clk clk27 clk_ser clk_pix clk48 spi_sck clk_audio}]
