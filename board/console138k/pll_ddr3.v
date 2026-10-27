// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// pll_ddr3 -- 50 MHz board clock -> the DDR3 IP's memory_clk (400 MHz, CLKOUT2, stopped by the IP's pll_stop through
// ENCLK2) and a free-running 100 MHz reference (CLKOUT0): pll_ddr3_MOD (the PLL primitive) with PLL_INIT, which every
// GW5AST PLL needs to lock (Sipeed's DDR3 example for this board also wraps its PLL with it: MULTI_FAC 16, CLK_PERIOD 20).
// PLL_INIT (pll_init.v, from the Gowin IDE: build_core.sh) steps the PLL's charge pump (ICPSEL) and loop filter (LPFRES)
// through its table while holding the PLL in reset, and gives lock (O_LOCK) once a setting has locked.
//   CLK_PERIOD: PLL_INIT's clock (init_clk) period in ns: 20 = 50 MHz.
//   MULTI_FAC:  the PLL's integer MDIV (pll_ddr3_MOD: 16), which picks PLL_INIT's table (MDIV <= 16, 17-34, > 34).
// Placed at PLL_L[0] by tango_console138k_ddr3.cst, INS_LOC "<inst>/u_pllm/PLL_inst" PLL_L[0]: the instance names
// u_pllm / PLL_inst are part of that constraint.
`default_nettype none
module pll_ddr3 (
    input  wire clkin,      // 50 MHz
    input  wire init_clk,   // 50 MHz, PLL_INIT's clock
    input  wire enclk2,     // CLKOUT2 enable (the DDR3 IP's pll_stop)
    output wire clkout0,    // 100 MHz, free running
    output wire clkout2,    // 400 MHz memory_clk
    output wire lock        // PLL_INIT O_LOCK
);
    wire [5:0] icpsel; wire [2:0] lpfres; wire pll_lock; wire pll_rst;
    pll_ddr3_MOD u_pllm (.clkout0(clkout0), .clkout1(), .clkout2(clkout2), .lock(pll_lock), .clkin(clkin),
        .reset(pll_rst), .icpsel(icpsel), .lpfres(lpfres), .lpfcap(2'b00), .enclk2(enclk2));
    PLL_INIT #(.CLK_PERIOD(20), .MULTI_FAC(16)) u_pll_init (.CLKIN(init_clk), .I_RST(1'b0), .O_RST(pll_rst),
        .PLLLOCK(pll_lock), .O_LOCK(lock), .ICPSEL(icpsel), .LPFRES(lpfres));
endmodule
`default_nettype wire
