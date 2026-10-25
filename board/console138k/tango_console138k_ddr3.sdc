// tango_console138k_ddr3.sdc -- TANGO_VIDEO_DDR3 builds (build_core.sh appends it, and adds both clocks to the build's
// one set_clock_groups line as groups of their own unless that line has them already): the DDR3 IP's clocks, as the
// DDR3 bring-up test's ddr3/test/tango_ddr3_test.sdc (Sipeed's ddr3_1v4_hs.sdc): memory_clk 400 MHz (pll_ddr3
// CLKOUT2), clk_x1 = the IP's 100 MHz user clock (its CLKDIV), which drives tango_ddr3 and ascal's Avalon side.
create_clock -name memory_clk -period 2.500  [get_nets {memory_clk}]
create_clock -name clk_x1     -period 10.000 [get_pins {u_ddr3/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
