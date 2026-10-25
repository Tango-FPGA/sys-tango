// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// MDIV / MDIV_FRAC (the TANGO_HDMI_PLL presets): tango_cfg.vh TANGO_HDMI_MDIV / _MDIV_FRAC from the board top; PLL_INIT's
// MULTI_FAC = the integer MDIV (its ICP/LPF table has three buckets, <= 16 / 17..34 / > 34: every preset is in "> 34").
// TANGO_HDMI_RETUNE: DYN_MDIV = "TRUE" -- MDIV from mdsel / mdsel_frac (tango_pll_retune, clocked by
// init_clk) and irst = PLL_INIT's I_RST: held high while the divider changes, its release re-runs PLL_INIT's ICP/LPF sweep
// (~18 ms at 50 MHz) and the PLL re-locks at the new frequency. "FALSE" (default): the ports are ignored, irst may be 0.
module pll_c1485 #(parameter integer MDIV = 55, parameter integer MDIV_FRAC = 0, parameter DYN_MDIV = "FALSE")
                 (clkin, init_clk, clkout0, clkout1, lock, irst, mdsel, mdsel_frac);
input clkin; input init_clk; output clkout0; output clkout1; output lock;   // lock = PLL_INIT O_LOCK
input irst; input [6:0] mdsel; input [2:0] mdsel_frac;
wire [5:0] icpsel; wire [2:0] lpfres; wire pll_lock; wire pll_rst;
    pll_c1485_MOD #(.MDIV(MDIV), .MDIV_FRAC(MDIV_FRAC), .DYN_MDIV(DYN_MDIV)) u_pll(.clkout0(clkout0), .clkout1(clkout1),
        .lock(pll_lock), .clkin(clkin), .reset(pll_rst), .icpsel(icpsel), .lpfres(lpfres), .lpfcap(2'b00),
        .mdsel(mdsel), .mdsel_frac(mdsel_frac));
    PLL_INIT u_pll_init(.CLKIN(init_clk), .I_RST(irst), .O_RST(pll_rst),
        .PLLLOCK(pll_lock), .O_LOCK(lock), .ICPSEL(icpsel), .LPFRES(lpfres));
    defparam u_pll_init.CLK_PERIOD = 20;
    defparam u_pll_init.MULTI_FAC = MDIV;
endmodule
