// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// pll_c27_MOD -- 50 MHz (the board oscillator) -> 27.000 MHz, one GW5AST PLL primitive; the clock pll_c1485 and
// the SDRAM PLL multiply from.  Primitive: Gowin UG306-1.0.4E (Arora V Clock
// User Guide) 5.1: ports Table 5-2, parameters Table 5-3, frequencies "Functional Description", phase "Phase Shift".
//   Fpfd = 50 / IDIV 2 = 25 MHz; Fvco = 25 x FBDIV 1 x MDIV 27 = 675 MHz; CLKOUT0 = 675 / ODIV0 25 = 27.000 MHz.
// The PLL's charge pump and loop filter are set at run time by PLL_INIT (pll_init.v from the Gowin IDE; the
// wrapper): it holds RESET while it steps ICPSEL / LPFRES through its table, and reports LOCK once a setting locks.
// Instance name PLL_inst: the timing reports and placement constraints name it.
`default_nettype none
module pll_c27_MOD (
    output wire       lock,       // PLL lock (to PLL_INIT)
    output wire       clkout0,    // 27 MHz
    input  wire       clkin,      // 50 MHz
    input  wire       reset,      // from PLL_INIT
    input  wire [5:0] icpsel,     // charge-pump current, from PLL_INIT
    input  wire [2:0] lpfres,     // loop-filter resistor, from PLL_INIT
    input  wire [1:0] lpfcap      // loop-filter capacitor
);
    wire clkout1, clkout2, clkout3, clkout4, clkout5, clkout6, clkfbout;   // unused outputs
    PLL #(
        .FCLKIN("50"),                                           // the reference frequency on CLKIN, MHz
        .IDIV_SEL(2),                                            // input divider: Fpfd = 50 / 2 = 25 MHz
        .FBDIV_SEL(1),                                           // feedback divider 1
        .CLKFB_SEL("INTERNAL"),                                  // feedback taken inside the PLL (CLKFB unused)
        // VCO multiplier 27 + 0/8: Fvco = 25 x 1 x 27 = 675 MHz
        .MDIV_SEL(27), .MDIV_FRAC_SEL(0),
        .ODIV0_SEL(25), .ODIV0_FRAC_SEL(0),                      // CLKOUT0 = Fvco / 25 = 27 MHz
        .ODIV1_SEL(8),                                           // channel 1 divider (output off)
        .ODIV2_SEL(8),                                           // channel 2 divider (output off)
        .ODIV3_SEL(8),                                           // channel 3 divider (output off)
        .ODIV4_SEL(8),                                           // channel 4 divider (output off)
        .ODIV5_SEL(8),                                           // channel 5 divider (output off)
        .ODIV6_SEL(8),                                           // channel 6 divider (output off)
        // output enables, channels 0-6
        .CLKOUT0_EN("TRUE"), .CLKOUT1_EN("FALSE"), .CLKOUT2_EN("FALSE"), .CLKOUT3_EN("FALSE"), .CLKOUT4_EN("FALSE"),
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
        .CLKOUT1_PE_COARSE(0), .CLKOUT1_PE_FINE(0),              // channel 1 static phase: 0
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
        // static charge pump / loop filter: not used (below); X = the tool's choice
        .ICP_SEL(6'bXXXXXX), .LPF_RES(3'bXXX), .LPF_CAP(2'b00),
        // charge pump from ICPSEL, loop filter from LPFRES / LPFCAP (the ports)
        .DYN_ICP_SEL("TRUE"), .DYN_LPF_SEL("TRUE"),
        .RESET_I_EN("FALSE"), .RESET_O_EN("FALSE"),              // the RESET_I / RESET_O ports off
        .SSC_EN("FALSE")                                         // spread spectrum off
    ) PLL_inst (
        .CLKIN(clkin), .CLKFB(1'b0),                             // reference clock; CLKFB unused (internal feedback)
        .RESET(reset),                                           // held by PLL_INIT while it calibrates
        // never powered down; the divider resets (RESET_I_EN / _O_EN off) unused
        .PLLPWD(1'b0), .RESET_I(1'b0), .RESET_O(1'b0),
        .LOCK(lock),                                             // PLL lock, to PLL_INIT
        // dynamic FBDIV / IDIV: not selected (DYN_FBDIV_SEL / DYN_IDIV_SEL "FALSE")
        .FBDSEL(6'd0), .IDSEL(6'd0),
        .MDSEL(7'd0), .MDSEL_FRAC(3'd0),                         // dynamic MDIV: not selected (DYN_MDIV_SEL "FALSE")
        .ODSEL0(7'd0), .ODSEL0_FRAC(3'd0), .ODSEL1(7'd0),        // dynamic ODIVn: not selected (DYN_ODIVn_SEL "FALSE")
        .ODSEL2(7'd0), .ODSEL3(7'd0), .ODSEL4(7'd0), .ODSEL5(7'd0), .ODSEL6(7'd0), // (the same)
        .DT0(4'd0), .DT1(4'd0), .DT2(4'd0), .DT3(4'd0),          // dynamic duty trim: not selected
        // charge pump / loop filter from PLL_INIT (DYN_ICP_SEL / DYN_LPF_SEL "TRUE")
        .ICPSEL(icpsel), .LPFRES(lpfres), .LPFCAP(lpfcap),
        .PSSEL(3'd0), .PSDIR(1'b0), .PSPULSE(1'b0),              // dynamic phase adjustment: off (DYN_DPA_EN "FALSE")
        .ENCLK0(1'b1), .ENCLK1(1'b1), .ENCLK2(1'b1), .ENCLK3(1'b1), // channel output enables: always on
        .ENCLK4(1'b1), .ENCLK5(1'b1), .ENCLK6(1'b1),             // (the same)
        .SSCPOL(1'b0), .SSCON(1'b0), .SSCMDSEL(7'd0), .SSCMDSEL_FRAC(3'd0), // spread spectrum: off
        .CLKOUT0(clkout0), .CLKOUT1(clkout1), .CLKOUT2(clkout2), .CLKOUT3(clkout3), // 27 MHz out; CLKOUT1-3 unused
        .CLKOUT4(clkout4), .CLKOUT5(clkout5), .CLKOUT6(clkout6), .CLKFBOUT(clkfbout)  // unused
    );
endmodule
`default_nettype wire
