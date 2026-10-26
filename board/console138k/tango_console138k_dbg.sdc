// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tango_sys_top138_dbg: = tango_console138k.sdc with the clock nets under the wrapper instance u/.
// tango_sys_top138 (every Console 138K core): clk, clk27, clk_ser, clk_pix, clk_audio + clk48 for the USB hosts
// (pll_c48).  Clock domains are asynchronous groups; every crossing is by construction:
//   SPI: SCK is a clock; byte handover SCK -> clk_pix by toggle (3 flops) + held data; reply byte
//   clk_pix -> SCK held >= 7 SCK periods before use; CSN is the SCK domain's asynchronous reset
//   bl616_jtagsel pin -> clk_pix: 2 flops (tango_sys); jtagseln / MISO tristate: combinational
//   clk48 -> clk_pix: USB report = held bus + rpt_tog toggle, 3 flops (tango_ds4_joy)
//   clk_pix <-> the core's clk_sys: tango_io (status toggle, tango_cdc_word handshakes, async FIFO).
//   A core whose CLK_SYS is not CLK_PIX adds its clocks in its own <core>.sdc, which the core's
//   build appends to this file (create_clock + its own set_clock_groups -asynchronous).
create_clock -name clk       -period 20.000    [get_ports {clk}]
create_clock -name clk27     -period 37.037    [get_nets {u/clk27}]
create_clock -name clk_ser   -period 2.694     [get_nets {u/clk_ser}]   // 720p: 371.25 MHz
create_clock -name clk_pix   -period 13.468    [get_nets {u/clk_pix}]   // 720p: 74.25 MHz
create_clock -name clk48     -period 20.833    [get_nets {u/clk48}]     // USB hosts, pll_c48
create_clock -name spi_sck   -period 75.000    [get_ports {spi_sck}]   // BL616 SPI 13.33 MHz (mcu_hw.c 980)
create_clock -name clk_audio -period 20833.333 [get_nets {u/clk_audio}]
set_clock_groups -asynchronous -group [get_clocks {clk}] -group [get_clocks {clk27}] -group [get_clocks {clk_ser clk_pix}] -group [get_clocks {clk48}] -group [get_clocks {spi_sck}] -group [get_clocks {clk_audio}]
report_timing -setup -max_paths 300 -max_common_paths 1
