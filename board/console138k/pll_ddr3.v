// Gowin IP generator output: the PLL wrapper (the module with its pll_ddr3_MOD and PLL_INIT instances and the PLL_INIT
// defparams) that the Gowin IDE's PLL_ADV generator writes beside pll_ddr3_mod.v; the generator's output carries
// "Copyright (C)2014-2025 Gowin Semiconductor Corporation. All rights reserved."  Not under sys-tango's MIT licence;
// see NOTICE ("Gowin IP templates").  Reformatted by the Tango authors; changed: the enclk2 input and the clkout2 output.
// pll_ddr3 -- 50 MHz board clock -> the DDR3 IP's memory_clk (400 MHz, CLKOUT2, gated by the IP's pll_stop on
// ENCLK2) and a free-running 100 MHz reference (CLKOUT0), with PLL_INIT as every Console 138K PLL (pll_c1485.v;
// a GW5AST PLL without PLL_INIT does not lock; Sipeed's DDR3 test gowin_pll.v: MULTI_FAC 16, CLK_PERIOD 20).
// Place at PLL_L[0]: INS_LOC "<inst>/u_pllm/PLL_inst" PLL_L[0] (check the name in
// the timing report: GowinSynthesis renamed a same-named inner instance to u_pll_0 in an earlier build).
module pll_ddr3(clkin, init_clk, enclk2, clkout0, clkout2, lock);
input clkin; input init_clk; input enclk2; output clkout0; output clkout2; output lock;   // lock = PLL_INIT O_LOCK
wire [5:0] icpsel; wire [2:0] lpfres; wire pll_lock; wire pll_rst;
    pll_ddr3_MOD u_pllm(.clkout0(clkout0), .clkout1(), .clkout2(clkout2), .lock(pll_lock), .clkin(clkin),
        .reset(pll_rst), .icpsel(icpsel), .lpfres(lpfres), .lpfcap(2'b00), .enclk2(enclk2));
    PLL_INIT u_pll_init(.CLKIN(init_clk), .I_RST(1'b0), .O_RST(pll_rst),
        .PLLLOCK(pll_lock), .O_LOCK(lock), .ICPSEL(icpsel), .LPFRES(lpfres));
    defparam u_pll_init.CLK_PERIOD = 20;
    defparam u_pll_init.MULTI_FAC = 16;
endmodule
