// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tango_ddr3_test.sdc -- the DDR3 bring-up test's clocks: the board's (board/console138k/tango_console138k.sdc) and the DDR3 ones (Sipeed's
// ddr3_1v4_hs.sdc: memory_clk 400 MHz, the IP's CLKDIV output 100 MHz).  Every domain is asynchronous to the others;
// the crossings are by construction (tango_ddr3_test_core.v header: Gray counts, quasi-static snapshot buses).
create_clock -name clk        -period 20.000    [get_ports {clk}]
create_clock -name clk27      -period 37.037    [get_nets {clk27}]
create_clock -name clk_ser    -period 2.694     [get_nets {clk_ser}]
create_clock -name clk_pix    -period 13.468    [get_nets {clk_pix}]
create_clock -name clk_audio  -period 20833.333 [get_nets {clk_audio}]
create_clock -name clk_ref    -period 10.000    [get_nets {clk_ref}]
create_clock -name memory_clk -period 2.500     [get_nets {memory_clk}]
create_clock -name clk_x1     -period 10.000    [get_pins {u_ddr3/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
set_clock_groups -asynchronous -group [get_clocks {clk}] -group [get_clocks {clk27}] -group [get_clocks {clk_ser clk_pix}] -group [get_clocks {clk_audio}] -group [get_clocks {clk_ref}] -group [get_clocks {memory_clk}] -group [get_clocks {clk_x1}]
report_timing -setup -max_paths 300 -max_common_paths 1
// the 400 MHz paths too (their clock path names the DDR3 PLL's site: build_ddr3_test.sh records it), and the clk_x1
// and clk_pix paths of the test's own logic
report_timing -setup -from_clock [get_clocks {memory_clk}] -max_paths 5 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {clk_x1}] -to_clock [get_clocks {clk_x1}] -max_paths 50 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {clk_pix}] -to_clock [get_clocks {clk_pix}] -max_paths 20 -max_common_paths 1
