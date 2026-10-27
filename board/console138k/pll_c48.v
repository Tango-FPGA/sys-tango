// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// pll_c48 -- 50 MHz -> 48 MHz for the USB host ports: pll_c48_MOD (the PLL primitive) with PLL_INIT, which every GW5AST
// PLL needs to lock.  PLL_INIT (pll_init.v, from the Gowin IDE: build_core.sh) steps the PLL's charge pump (ICPSEL) and
// loop filter (LPFRES) through its table while holding the PLL in reset, and gives lock (O_LOCK) once a setting has locked.
//   CLK_PERIOD: PLL_INIT's clock (init_clk) period in ns: 20 = 50 MHz.
//   MULTI_FAC:  the PLL's integer MDIV (pll_c48_MOD: 24), which picks PLL_INIT's table (MDIV <= 16, 17-34, > 34).
`default_nettype none
module pll_c48 (
    input  wire clkin,      // 50 MHz
    input  wire init_clk,   // 50 MHz, PLL_INIT's clock
    output wire clkout0,    // 48 MHz
    output wire lock        // PLL_INIT O_LOCK
);
    wire [5:0] icpsel; wire [2:0] lpfres; wire pll_lock; wire pll_rst;
    pll_c48_MOD u_pll (.clkout0(clkout0), .lock(pll_lock), .clkin(clkin),
        .reset(pll_rst), .icpsel(icpsel), .lpfres(lpfres), .lpfcap(2'b00));
    PLL_INIT #(.CLK_PERIOD(20), .MULTI_FAC(24)) u_pll_init (.CLKIN(init_clk), .I_RST(1'b0), .O_RST(pll_rst),
        .PLLLOCK(pll_lock), .O_LOCK(lock), .ICPSEL(icpsel), .LPFRES(lpfres));
endmodule
`default_nettype wire
