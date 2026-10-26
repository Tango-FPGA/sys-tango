// Gowin IP generator output: the PLL instance and its defparams follow the Gowin IDE's PLL generator output for
// GW5AST-138 (as pll_c27_mod.v), which carries "Copyright (C)2014-2025 Gowin Semiconductor Corporation. All rights
// reserved."  Not under sys-tango's MIT licence; see NOTICE ("Gowin IP templates").  The settings and comments are
// the Tango authors'.
// Gowin_PLL_SDRAM126 -- PLL config for the menu SDRAM domain at 126 MHz.
// GW5AST-LV138PG484AC1/I0 uses the "PLL" primitive (not PLLA).
// 27 MHz in, IDIV 1 (PFD 27), MDIV 28 (VCO 756, legal 700-1400), ODIV0 6 ->
// clkout0 = 126.0 MHz fabric clock, ODIV1 6 -> clkout1 = 126.0 MHz shifted
// for the sdram_clk pin.  W9825G6KH-6 CL3 valid: tCK_min=6ns, 126MHz=7.937ns.
// Clocked down from 162 MHz (gowin_pll_sdram162.v) after that build could not
// close timing in the multi-driver SDRAM_A mux tree (-1.456 ns at 162 MHz).
// 126 MHz yields ~347 ps of margin from the same P&R (r5) path, confirmed
// closed in r7.  Bandwidth 126*2 bytes = 252 MB/s burst peak; ascal frame buffer
// (480x240 NTSC) needs <15 MB/s, so the cut is purely for timing, not bandwidth.
// Phase: (PE_COARSE + PE_FINE/8)/ODIV * 360 = (5+0.25)/6 * 360 = 315 deg -- same
// formula and same 315-deg target as the 162 MHz PLL, because ODIV is unchanged.
// The phase sweep was done for 162 MHz; 126 MHz re-sweep pending silicon validation.
module Gowin_PLL_SDRAM126 #(
    parameter PE_COARSE = 5,
    parameter PE_FINE   = 2
) (
    output wire lock,
    output wire clkout0,     // 126 MHz, fabric (clk_sdram)
    output wire clkout1,     // 126 MHz, +315 deg (sdram_clk pin)
    input  wire clkin        // 27 MHz
);
    wire clkout2_o, clkout3_o, clkout4_o, clkout5_o, clkout6_o;
    wire clkfbout_o;
    wire gw_gnd = 1'b0;
    wire gw_vcc = 1'b1;

    PLL PLL_inst (
        .LOCK(lock),
        .CLKOUT0(clkout0),
        .CLKOUT1(clkout1),
        .CLKOUT2(clkout2_o),
        .CLKOUT3(clkout3_o),
        .CLKOUT4(clkout4_o),
        .CLKOUT5(clkout5_o),
        .CLKOUT6(clkout6_o),
        .CLKFBOUT(clkfbout_o),
        .CLKIN(clkin),
        .CLKFB(gw_gnd),
        .RESET(gw_gnd),
        .PLLPWD(gw_gnd),
        .RESET_I(gw_gnd),
        .RESET_O(gw_gnd),
        .FBDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .IDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .MDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .MDSEL_FRAC({gw_gnd,gw_gnd,gw_gnd}),
        .ODSEL0({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .ODSEL0_FRAC({gw_gnd,gw_gnd,gw_gnd}),
        .ODSEL1({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .ODSEL2({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .ODSEL3({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .ODSEL4({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .ODSEL5({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .ODSEL6({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .DT0({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .DT1({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .DT2({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .DT3({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .ICPSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .LPFRES({gw_gnd,gw_gnd,gw_gnd}),
        .LPFCAP({gw_gnd,gw_gnd}),
        .PSSEL({gw_gnd,gw_gnd,gw_gnd}),
        .PSDIR(gw_gnd),
        .PSPULSE(gw_gnd),
        .ENCLK0(gw_vcc),
        .ENCLK1(gw_vcc),
        .ENCLK2(gw_gnd),
        .ENCLK3(gw_gnd),
        .ENCLK4(gw_gnd),
        .ENCLK5(gw_gnd),
        .ENCLK6(gw_gnd),
        .SSCPOL(gw_gnd),
        .SSCON(gw_gnd),
        .SSCMDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .SSCMDSEL_FRAC({gw_gnd,gw_gnd,gw_gnd})
    );

    defparam PLL_inst.FCLKIN = "27";
    defparam PLL_inst.IDIV_SEL = 1;
    defparam PLL_inst.FBDIV_SEL = 1;
    defparam PLL_inst.CLKFB_SEL = "INTERNAL";
    defparam PLL_inst.ODIV0_SEL = 6;
    defparam PLL_inst.ODIV0_FRAC_SEL = 0;
    defparam PLL_inst.ODIV1_SEL = 6;
    defparam PLL_inst.ODIV2_SEL = 8;
    defparam PLL_inst.ODIV3_SEL = 8;
    defparam PLL_inst.ODIV4_SEL = 8;
    defparam PLL_inst.ODIV5_SEL = 8;
    defparam PLL_inst.ODIV6_SEL = 8;
    defparam PLL_inst.MDIV_SEL = 28;          // 27 × 28 / 6 = 126 MHz (was 36 → 162 MHz)
    defparam PLL_inst.MDIV_FRAC_SEL = 0;
    defparam PLL_inst.CLKOUT0_EN = "TRUE";
    defparam PLL_inst.CLKOUT1_EN = "TRUE";
    defparam PLL_inst.CLKOUT2_EN = "FALSE";
    defparam PLL_inst.CLKOUT3_EN = "FALSE";
    defparam PLL_inst.CLKOUT4_EN = "FALSE";
    defparam PLL_inst.CLKOUT5_EN = "FALSE";
    defparam PLL_inst.CLKOUT6_EN = "FALSE";
    defparam PLL_inst.CLKOUT0_DT_DIR = 1'b1;
    defparam PLL_inst.CLKOUT1_DT_DIR = 1'b1;
    defparam PLL_inst.CLKOUT2_DT_DIR = 1'b1;
    defparam PLL_inst.CLKOUT3_DT_DIR = 1'b1;
    defparam PLL_inst.CLKOUT0_DT_STEP = 0;
    defparam PLL_inst.CLKOUT1_DT_STEP = 0;
    defparam PLL_inst.CLKOUT2_DT_STEP = 0;
    defparam PLL_inst.CLKOUT3_DT_STEP = 0;
    defparam PLL_inst.CLK0_IN_SEL = 1'b0;
    defparam PLL_inst.CLK0_OUT_SEL = 1'b0;
    defparam PLL_inst.CLK1_IN_SEL = 1'b0;
    defparam PLL_inst.CLK1_OUT_SEL = 1'b0;
    defparam PLL_inst.CLK2_IN_SEL = 1'b0;
    defparam PLL_inst.CLK2_OUT_SEL = 1'b0;
    defparam PLL_inst.CLK3_IN_SEL = 1'b0;
    defparam PLL_inst.CLK3_OUT_SEL = 1'b0;
    defparam PLL_inst.CLK4_IN_SEL = 2'b00;
    defparam PLL_inst.CLK4_OUT_SEL = 1'b0;
    defparam PLL_inst.CLK5_IN_SEL = 1'b0;
    defparam PLL_inst.CLK5_OUT_SEL = 1'b0;
    defparam PLL_inst.CLK6_IN_SEL = 1'b0;
    defparam PLL_inst.CLK6_OUT_SEL = 1'b0;
    defparam PLL_inst.CLKOUT0_PE_COARSE = 0;
    defparam PLL_inst.CLKOUT0_PE_FINE = 0;
    defparam PLL_inst.CLKOUT1_PE_COARSE = PE_COARSE;
    defparam PLL_inst.CLKOUT1_PE_FINE = PE_FINE;
    defparam PLL_inst.CLKOUT2_PE_COARSE = 0;
    defparam PLL_inst.CLKOUT2_PE_FINE = 0;
    defparam PLL_inst.CLKOUT3_PE_COARSE = 0;
    defparam PLL_inst.CLKOUT3_PE_FINE = 0;
    defparam PLL_inst.CLKOUT4_PE_COARSE = 0;
    defparam PLL_inst.CLKOUT4_PE_FINE = 0;
    defparam PLL_inst.CLKOUT5_PE_COARSE = 0;
    defparam PLL_inst.CLKOUT5_PE_FINE = 0;
    defparam PLL_inst.CLKOUT6_PE_COARSE = 0;
    defparam PLL_inst.CLKOUT6_PE_FINE = 0;
    defparam PLL_inst.DYN_DPA_EN = "FALSE";
    defparam PLL_inst.DYN_PE0_SEL = "FALSE";
    defparam PLL_inst.DYN_PE1_SEL = "FALSE";
    defparam PLL_inst.DYN_PE2_SEL = "FALSE";
    defparam PLL_inst.DYN_PE3_SEL = "FALSE";
    defparam PLL_inst.DYN_PE4_SEL = "FALSE";
    defparam PLL_inst.DYN_PE5_SEL = "FALSE";
    defparam PLL_inst.DYN_PE6_SEL = "FALSE";
    defparam PLL_inst.DE0_EN = "FALSE";
    defparam PLL_inst.DE1_EN = "FALSE";
    defparam PLL_inst.DE2_EN = "FALSE";
    defparam PLL_inst.DE3_EN = "FALSE";
    defparam PLL_inst.DE4_EN = "FALSE";
    defparam PLL_inst.DE5_EN = "FALSE";
    defparam PLL_inst.DE6_EN = "FALSE";
    defparam PLL_inst.RESET_I_EN = "FALSE";
    defparam PLL_inst.RESET_O_EN = "FALSE";
    defparam PLL_inst.ICP_SEL = 6'bXXXXXX;
    defparam PLL_inst.LPF_RES = 3'bXXX;
    defparam PLL_inst.LPF_CAP = 2'b00;
    defparam PLL_inst.SSC_EN = "FALSE";
    defparam PLL_inst.DYN_IDIV_SEL = "FALSE";
    defparam PLL_inst.DYN_FBDIV_SEL = "FALSE";
    defparam PLL_inst.DYN_MDIV_SEL = "FALSE";
    defparam PLL_inst.DYN_ODIV0_SEL = "FALSE";
    defparam PLL_inst.DYN_ODIV1_SEL = "FALSE";
    defparam PLL_inst.DYN_ODIV2_SEL = "FALSE";
    defparam PLL_inst.DYN_ODIV3_SEL = "FALSE";
    defparam PLL_inst.DYN_ODIV4_SEL = "FALSE";
    defparam PLL_inst.DYN_ODIV5_SEL = "FALSE";
    defparam PLL_inst.DYN_ODIV6_SEL = "FALSE";
    defparam PLL_inst.DYN_DT0_SEL = "FALSE";
    defparam PLL_inst.DYN_DT1_SEL = "FALSE";
    defparam PLL_inst.DYN_DT2_SEL = "FALSE";
    defparam PLL_inst.DYN_DT3_SEL = "FALSE";
    defparam PLL_inst.DYN_ICP_SEL = "FALSE";
    defparam PLL_inst.DYN_LPF_SEL = "FALSE";
endmodule
