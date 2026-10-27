// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// Gowin_PLL_SDRAM -- the framework SDRAM's clocks (TANGO_FW_SDRAM): 27 MHz (pll_c27) -> 96 MHz, one GW5AST PLL primitive
// (the GW5AST-138 has the "PLL" primitive, not PLLA) with PLL_INIT.  Primitive: Gowin UG306-1.0.4E (Arora V Clock
// User Guide) 5.1: ports Table 5-2, parameters Table 5-3, frequencies "Functional Description", phase "Phase Shift".
//   Fpfd = 27 / IDIV 1 = 27 MHz; Fvco = 27 x FBDIV 1 x MDIV 32 = 864 MHz; CLKOUT0 = CLKOUT1 = 864 / ODIV 9 = 96.0 MHz.
//   CLKOUT0: the fabric clock clk_sdram.  CLKOUT1: the SDRAM_CLK pin, +320 deg ((8 + 0/8) / 9 x 360), so its rising edge
//   comes 40 deg (1.16 ns) before clk_sdram's next one.  W9825G6KH-6 at CL3: tCK >= 6 ns; 96 MHz = 10.417 ns.
//   RFRSH_CYCLES = 750 (64 ms x 96 MHz / 8192 rows).
// One module name: when the frequency is re-swept, MDIV_SEL / ODIV and the PE parameters change here.
// Frequency history: 162 MHz (STA -1.456 ns), 126 MHz (STA -1.647 ns), 108 MHz (STA -0.660 ns with TANGO_NES_SDRAM,
// Fmax 100.994 MHz), 96 MHz.  The PLL's ICPSEL / LPFRES ports carry PLL_INIT's values but DYN_ICP_SEL / DYN_LPF_SEL are
// "FALSE": the static ICP_SEL / LPF_RES (the tool's choice) set the loop; PLL_INIT's reset and lock sequencing is used.
`default_nettype none
module Gowin_PLL_SDRAM #(
    parameter PE_COARSE = 8,   // CLKOUT1 phase, whole VCO periods ...
    parameter PE_FINE   = 0    // ... plus eighths: (PE_COARSE + PE_FINE / 8) / ODIV1 x 360 deg
) (
    output wire lock,
    output wire clkout0,     // fabric clock (clk_sdram)
    output wire clkout1,     // the same frequency, phase-shifted (the SDRAM_CLK pin)
    input  wire clkin        // 27 MHz (pll_c27)
);
    wire clkout2_o, clkout3_o, clkout4_o, clkout5_o, clkout6_o, clkfbout_o;   // unused outputs
    // PLL_INIT (pll_init.v, from the Gowin IDE): sets the charge pump / loop filter by trial at power-up and gives lock
    // once the PLL has locked.  CLK_PERIOD: its clock's period in ns (clkin, 27 MHz: 37); MULTI_FAC: the PLL's integer
    // MDIV, which picks its ICP / LPF table (<= 16, 17-34, > 34).
    wire [5:0] icpsel; wire [2:0] lpfres; wire pll_lock; wire pll_rst;
    PLL_INIT #(.CLK_PERIOD(37), .MULTI_FAC(32)) u_pll_init (
        .CLKIN(clkin), .I_RST(1'b0), .O_RST(pll_rst), .PLLLOCK(pll_lock), .O_LOCK(lock), .ICPSEL(icpsel), .LPFRES(lpfres));

    PLL #(
        .FCLKIN("27"),                                           // the reference frequency on CLKIN, MHz
        .IDIV_SEL(1),                                            // input divider: Fpfd = 27 / 1 = 27 MHz
        .FBDIV_SEL(1),                                           // feedback divider 1
        .CLKFB_SEL("INTERNAL"),                                  // feedback taken inside the PLL (CLKFB unused)
        // VCO multiplier 32 + 0/8: Fvco = 27 x 1 x 32 = 864 MHz
        .MDIV_SEL(32), .MDIV_FRAC_SEL(0),
        .ODIV0_SEL(9), .ODIV0_FRAC_SEL(0),                       // CLKOUT0 = Fvco / 9 = 96 MHz
        .ODIV1_SEL(9),                                           // CLKOUT1 = Fvco / 9 = 96 MHz
        .ODIV2_SEL(9),                                           // channel 2 divider (output off)
        .ODIV3_SEL(9),                                           // channel 3 divider (output off)
        .ODIV4_SEL(9),                                           // channel 4 divider (output off)
        .ODIV5_SEL(9),                                           // channel 5 divider (output off)
        .ODIV6_SEL(9),                                           // channel 6 divider (output off)
        // output enables, channels 0-6
        .CLKOUT0_EN("TRUE"), .CLKOUT1_EN("TRUE"), .CLKOUT2_EN("FALSE"), .CLKOUT3_EN("FALSE"), .CLKOUT4_EN("FALSE"),
        .CLKOUT5_EN("FALSE"), .CLKOUT6_EN("FALSE"),
        // IDIV / FBDIV / MDIV from the parameters ("TRUE": the IDSEL / FBDSEL / MDSEL, MDSEL_FRAC ports)
        .DYN_IDIV_SEL("FALSE"), .DYN_FBDIV_SEL("FALSE"), .DYN_MDIV_SEL("FALSE"),
        // ODIV0-6 from the parameters ("TRUE": the ODSELn ports)
        .DYN_ODIV0_SEL("FALSE"), .DYN_ODIV1_SEL("FALSE"), .DYN_ODIV2_SEL("FALSE"), .DYN_ODIV3_SEL("FALSE"),
        .DYN_ODIV4_SEL("FALSE"), .DYN_ODIV5_SEL("FALSE"), .DYN_ODIV6_SEL("FALSE"),
        // each divider fed from the VCO (1 / CLK4 2'b10: from CLKIN; 2'b01: cascade)
        .CLK0_IN_SEL(1'b0), .CLK1_IN_SEL(1'b0), .CLK2_IN_SEL(1'b0), .CLK3_IN_SEL(1'b0), .CLK4_IN_SEL(2'b00),
        .CLK5_IN_SEL(1'b0), .CLK6_IN_SEL(1'b0),
        // each output from its divider (1: CLKIN passed through)
        .CLK0_OUT_SEL(1'b0), .CLK1_OUT_SEL(1'b0), .CLK2_OUT_SEL(1'b0), .CLK3_OUT_SEL(1'b0), .CLK4_OUT_SEL(1'b0),
        .CLK5_OUT_SEL(1'b0), .CLK6_OUT_SEL(1'b0),
        .CLKOUT0_PE_COARSE(0), .CLKOUT0_PE_FINE(0),              // channel 0 static phase: 0
        // channel 1 static phase: (COARSE + FINE/8) / ODIV1 x 360 deg
        .CLKOUT1_PE_COARSE(PE_COARSE), .CLKOUT1_PE_FINE(PE_FINE),
        .CLKOUT2_PE_COARSE(0), .CLKOUT2_PE_FINE(0),              // channel 2 static phase: 0
        .CLKOUT3_PE_COARSE(0), .CLKOUT3_PE_FINE(0),              // channel 3 static phase: 0
        .CLKOUT4_PE_COARSE(0), .CLKOUT4_PE_FINE(0),              // channel 4 static phase: 0
        .CLKOUT5_PE_COARSE(0), .CLKOUT5_PE_FINE(0),              // channel 5 static phase: 0
        .CLKOUT6_PE_COARSE(0), .CLKOUT6_PE_FINE(0),              // channel 6 static phase: 0
        // dynamic phase adjustment (PSSEL / PSDIR / PSPULSE) off
        .DYN_DPA_EN("FALSE"),
        // each phase from the PE parameters, not the dynamic adjustment
        .DYN_PE0_SEL("FALSE"), .DYN_PE1_SEL("FALSE"), .DYN_PE2_SEL("FALSE"), .DYN_PE3_SEL("FALSE"),
        .DYN_PE4_SEL("FALSE"), .DYN_PE5_SEL("FALSE"), .DYN_PE6_SEL("FALSE"),
        // duty-cycle adjustment off: 50 % duty
        .DE0_EN("FALSE"), .DE1_EN("FALSE"), .DE2_EN("FALSE"), .DE3_EN("FALSE"), .DE4_EN("FALSE"), .DE5_EN("FALSE"),
        .DE6_EN("FALSE"),
        // static duty trim direction, channels 0-3 (no effect: 0 steps)
        .CLKOUT0_DT_DIR(1'b1), .CLKOUT1_DT_DIR(1'b1), .CLKOUT2_DT_DIR(1'b1), .CLKOUT3_DT_DIR(1'b1),
        // static duty trim, channels 0-3: 0 steps of 50 ps
        .CLKOUT0_DT_STEP(0), .CLKOUT1_DT_STEP(0), .CLKOUT2_DT_STEP(0), .CLKOUT3_DT_STEP(0),
        // duty trim from the parameters, not the DTn ports
        .DYN_DT0_SEL("FALSE"), .DYN_DT1_SEL("FALSE"), .DYN_DT2_SEL("FALSE"), .DYN_DT3_SEL("FALSE"),
        // static charge pump / loop filter: X = the tool's choice, capacitor C0
        .ICP_SEL(6'bXXXXXX), .LPF_RES(3'bXXX), .LPF_CAP(2'b00),
        // these static settings, not the ICPSEL / LPFRES / LPFCAP ports
        .DYN_ICP_SEL("FALSE"), .DYN_LPF_SEL("FALSE"),
        .RESET_I_EN("FALSE"), .RESET_O_EN("FALSE"),              // the RESET_I / RESET_O ports off
        .SSC_EN("FALSE")                                         // spread spectrum off
    ) PLL_inst (
        .CLKIN(clkin), .CLKFB(1'b0),                             // reference clock; CLKFB unused (internal feedback)
        .RESET(pll_rst),                                         // held by PLL_INIT while it calibrates
        // never powered down; the divider resets (RESET_I_EN / _O_EN off) unused
        .PLLPWD(1'b0), .RESET_I(1'b0), .RESET_O(1'b0),
        .LOCK(pll_lock),                                         // PLL lock, to PLL_INIT
        // dynamic FBDIV / IDIV: not selected (DYN_FBDIV_SEL / DYN_IDIV_SEL "FALSE")
        .FBDSEL(6'd0), .IDSEL(6'd0),
        .MDSEL(7'd0), .MDSEL_FRAC(3'd0),                         // dynamic MDIV: not selected (DYN_MDIV_SEL "FALSE")
        .ODSEL0(7'd0), .ODSEL0_FRAC(3'd0), .ODSEL1(7'd0),        // dynamic ODIVn: not selected (DYN_ODIVn_SEL "FALSE")
        .ODSEL2(7'd0), .ODSEL3(7'd0), .ODSEL4(7'd0), .ODSEL5(7'd0), .ODSEL6(7'd0), // (the same)
        .DT0(4'd0), .DT1(4'd0), .DT2(4'd0), .DT3(4'd0),          // dynamic duty trim: not selected
        // PLL_INIT's outputs; ignored here (DYN_ICP_SEL / DYN_LPF_SEL "FALSE")
        .ICPSEL(icpsel), .LPFRES(lpfres), .LPFCAP(2'b00),
        .PSSEL(3'd0), .PSDIR(1'b0), .PSPULSE(1'b0),              // dynamic phase adjustment: off (DYN_DPA_EN "FALSE")
        .ENCLK0(1'b1), .ENCLK1(1'b1), .ENCLK2(1'b0), .ENCLK3(1'b0), // CLKOUT0 / 1 on, CLKOUT2 / 3 off
        .ENCLK4(1'b0), .ENCLK5(1'b0), .ENCLK6(1'b0),             // CLKOUT4-6 off
        .SSCPOL(1'b0), .SSCON(1'b0), .SSCMDSEL(7'd0), .SSCMDSEL_FRAC(3'd0), // spread spectrum: off
        // 96 MHz (fabric), 96 MHz +320 deg (pin); CLKOUT2-3 unused
        .CLKOUT0(clkout0), .CLKOUT1(clkout1), .CLKOUT2(clkout2_o), .CLKOUT3(clkout3_o),
        .CLKOUT4(clkout4_o), .CLKOUT5(clkout5_o), .CLKOUT6(clkout6_o), .CLKFBOUT(clkfbout_o)  // unused
    );
endmodule
`default_nettype wire
