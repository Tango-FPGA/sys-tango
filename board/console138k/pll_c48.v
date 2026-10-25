// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// Proven on the Console 138K: the two USB-A host ports run on it.
// pll_c48 -- 50 MHz -> 48 MHz for the USB host ports, GW5AST PLL + PLL_INIT.
// PLL_INIT is mandatory on GW5AST (without it the PLL does not lock): MULTI_FAC
// must equal MDIV_SEL (24); init_clk is the 50 MHz oscillator (CLK_PERIOD 20 ns).
module pll_c48(clkin, init_clk, clkout0, lock);
input clkin; input init_clk; output clkout0; output lock;
wire [5:0] icpsel; wire [2:0] lpfres; wire pll_lock; wire pll_rst;
    pll_c48_MOD u_pll(.clkout0(clkout0), .lock(pll_lock), .clkin(clkin),
        .reset(pll_rst), .icpsel(icpsel), .lpfres(lpfres), .lpfcap(2'b00));
    PLL_INIT u_pll_init(.CLKIN(init_clk), .I_RST(1'b0), .O_RST(pll_rst),
        .PLLLOCK(pll_lock), .O_LOCK(lock), .ICPSEL(icpsel), .LPFRES(lpfres));
    defparam u_pll_init.CLK_PERIOD = 20;
    defparam u_pll_init.MULTI_FAC = 24;
endmodule
