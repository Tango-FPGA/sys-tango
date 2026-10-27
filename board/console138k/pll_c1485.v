// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// pll_c1485 -- the HDMI serial and pixel clocks from 27 MHz: pll_c1485_MOD (the PLL primitive) with PLL_INIT, which every
// GW5AST PLL needs to lock.  PLL_INIT (pll_init.v, from the Gowin IDE: build_core.sh) steps the PLL's charge pump
// (ICPSEL) and loop filter (LPFRES) through its table while holding the PLL in reset, and gives lock (O_LOCK) once a
// setting has locked.
//   CLK_PERIOD: PLL_INIT's clock (init_clk) period in ns: 20 = 50 MHz.
//   MULTI_FAC:  the PLL's integer MDIV, which picks PLL_INIT's table (MDIV <= 16, 17-34, > 34): MDIV here.  Every preset
//               (tango_cfg.vh TANGO_HDMI_MDIV) and both settings of a retune are in the "> 34" table.
// MDIV / MDIV_FRAC: the VCO multiplier MDIV + MDIV_FRAC / 8 (defaults 55 + 0/8: 74.25 MHz pixel, 371.25 MHz serial).
// DYN_MDIV = "TRUE" (TANGO_HDMI_RETUNE, tango_cfg.vh): MDIV from mdsel / mdsel_frac (tango_pll_retune, clocked by
// init_clk), and irst = PLL_INIT's I_RST: held high while the divider changes, its release re-runs PLL_INIT's
// ICP / LPF sweep (~18 ms at 50 MHz) and the PLL re-locks at the new frequency.  "FALSE" (default): the ports are
// ignored, irst may be 0.
`default_nettype none
module pll_c1485 #(
    parameter integer MDIV      = 55,
    parameter integer MDIV_FRAC = 0,
    parameter         DYN_MDIV  = "FALSE"
) (
    input  wire       clkin,      // 27 MHz
    input  wire       init_clk,   // 50 MHz, PLL_INIT's clock
    output wire       clkout0,    // serial clock (5 x pixel)
    output wire       clkout1,    // pixel clock
    output wire       lock,       // PLL_INIT O_LOCK
    input  wire       irst,       // PLL_INIT I_RST (DYN_MDIV "TRUE")
    input  wire [6:0] mdsel,      // DYN_MDIV "TRUE": 128 - MDIV integer
    input  wire [2:0] mdsel_frac  // DYN_MDIV "TRUE": 7 - MDIV eighths
);
    wire [5:0] icpsel; wire [2:0] lpfres; wire pll_lock; wire pll_rst;
    pll_c1485_MOD #(.MDIV(MDIV), .MDIV_FRAC(MDIV_FRAC), .DYN_MDIV(DYN_MDIV)) u_pll (.clkout0(clkout0), .clkout1(clkout1),
        .lock(pll_lock), .clkin(clkin), .reset(pll_rst), .icpsel(icpsel), .lpfres(lpfres), .lpfcap(2'b00),
        .mdsel(mdsel), .mdsel_frac(mdsel_frac));
    PLL_INIT #(.CLK_PERIOD(20), .MULTI_FAC(MDIV)) u_pll_init (.CLKIN(init_clk), .I_RST(irst), .O_RST(pll_rst),
        .PLLLOCK(pll_lock), .O_LOCK(lock), .ICPSEL(icpsel), .LPFRES(lpfres));
endmodule
`default_nettype wire
