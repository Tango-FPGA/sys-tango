// Gowin IP generator output: the PLL wrapper (the module with its pll_c27_MOD and PLL_INIT instances and the PLL_INIT
// defparams) that the Gowin IDE's PLL_ADV generator writes beside pll_c27_mod.v; the generator's output carries
// "Copyright (C)2014-2025 Gowin Semiconductor Corporation. All rights reserved."  Not under sys-tango's MIT licence;
// see NOTICE ("Gowin IP templates").  Reformatted by the Tango authors (the lock comment).
module pll_c27(clkin, init_clk, clkout0, lock);
input clkin; input init_clk; output clkout0; output lock;   // lock = PLL_INIT O_LOCK (as pll_c48)
wire [5:0] icpsel; wire [2:0] lpfres; wire pll_lock; wire pll_rst;
    pll_c27_MOD u_pll(.clkout0(clkout0), .lock(pll_lock), .clkin(clkin),
        .reset(pll_rst), .icpsel(icpsel), .lpfres(lpfres), .lpfcap(2'b00));
    PLL_INIT u_pll_init(.CLKIN(init_clk), .I_RST(1'b0), .O_RST(pll_rst),
        .PLLLOCK(pll_lock), .O_LOCK(lock), .ICPSEL(icpsel), .LPFRES(lpfres));
    defparam u_pll_init.CLK_PERIOD = 20;
    defparam u_pll_init.MULTI_FAC = 27;
endmodule
