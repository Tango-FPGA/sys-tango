// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// gw_pll_hdmi_sim.v -- SIM-ONLY behavioural model of the Gowin GW5A `PLL` primitive as pll_c1485_mod.v uses it (the HDMI PLL
// retune at core start of vsync_adjust; never synthesised).  27 MHz in (SIM_FIN_HZ; the CLKIN pin is not used for timing),
// IDIV / ODIV0 / ODIV1 from the parameters, MDIV = MDIV_SEL + MDIV_FRAC_SEL / 8, or with DYN_MDIV_SEL = "TRUE" the ports
// (UG306 Tables 5-8 / 5-9: MDIV integer = 128 - MDSEL, eighths = 7 - MDSEL_FRAC), latched when RESET falls.  RESET high:
// no clocks, LOCK low.  RESET low: CLKOUT1 (and CLKOUT0 if SIM_CLKOUT0) run at the latched setting, exact on average
// (an integer-ps half period + a Bresenham fraction), LOCK rises SIM_LOCK_PS later.  A change of MDSEL / MDSEL_FRAC while
// RESET is low (the PLL running) is reported ("PLL MODEL: MDSEL changed while running") and counted (n_bad_change).
// Every other parameter / port exists only so pll_c1485_mod.v elaborates unchanged.
//==============================================================================
`default_nettype none
`timescale 1ps/1ps
module PLL #(
    parameter FCLKIN = "27",
    parameter IDIV_SEL = 1,
    parameter FBDIV_SEL = 0,
    parameter ODIV0_SEL = 4,
    parameter ODIV1_SEL = 20,
    parameter ODIV2_SEL = 0,
    parameter ODIV3_SEL = 0,
    parameter ODIV4_SEL = 0,
    parameter ODIV5_SEL = 0,
    parameter ODIV6_SEL = 0,
    parameter MDIV_SEL = 55,
    parameter MDIV_FRAC_SEL = 0,
    parameter ODIV0_FRAC_SEL = 0,
    parameter CLKOUT0_EN = "FALSE",
    parameter CLKOUT1_EN = "FALSE",
    parameter CLKOUT2_EN = "FALSE",
    parameter CLKOUT3_EN = "FALSE",
    parameter CLKOUT4_EN = "FALSE",
    parameter CLKOUT5_EN = "FALSE",
    parameter CLKOUT6_EN = "FALSE",
    parameter CLKFB_SEL = "FALSE",
    parameter CLKOUT0_DT_DIR = 0,
    parameter CLKOUT1_DT_DIR = 0,
    parameter CLKOUT2_DT_DIR = 0,
    parameter CLKOUT3_DT_DIR = 0,
    parameter CLKOUT0_DT_STEP = 0,
    parameter CLKOUT1_DT_STEP = 0,
    parameter CLKOUT2_DT_STEP = 0,
    parameter CLKOUT3_DT_STEP = 0,
    parameter CLK0_IN_SEL = 0,
    parameter CLK0_OUT_SEL = 0,
    parameter CLK1_IN_SEL = 0,
    parameter CLK1_OUT_SEL = 0,
    parameter CLK2_IN_SEL = 0,
    parameter CLK2_OUT_SEL = 0,
    parameter CLK3_IN_SEL = 0,
    parameter CLK3_OUT_SEL = 0,
    parameter CLK4_IN_SEL = 0,
    parameter CLK4_OUT_SEL = 0,
    parameter CLK5_IN_SEL = 0,
    parameter CLK5_OUT_SEL = 0,
    parameter CLK6_IN_SEL = 0,
    parameter CLK6_OUT_SEL = 0,
    parameter DYN_DPA_EN = "FALSE",
    parameter CLKOUT0_PE_COARSE = 0,
    parameter CLKOUT0_PE_FINE = 0,
    parameter CLKOUT1_PE_COARSE = 0,
    parameter CLKOUT1_PE_FINE = 0,
    parameter CLKOUT2_PE_COARSE = 0,
    parameter CLKOUT2_PE_FINE = 0,
    parameter CLKOUT3_PE_COARSE = 0,
    parameter CLKOUT3_PE_FINE = 0,
    parameter CLKOUT4_PE_COARSE = 0,
    parameter CLKOUT4_PE_FINE = 0,
    parameter CLKOUT5_PE_COARSE = 0,
    parameter CLKOUT5_PE_FINE = 0,
    parameter CLKOUT6_PE_COARSE = 0,
    parameter CLKOUT6_PE_FINE = 0,
    parameter DYN_PE0_SEL = "FALSE",
    parameter DYN_PE1_SEL = "FALSE",
    parameter DYN_PE2_SEL = "FALSE",
    parameter DYN_PE3_SEL = "FALSE",
    parameter DYN_PE4_SEL = "FALSE",
    parameter DYN_PE5_SEL = "FALSE",
    parameter DYN_PE6_SEL = "FALSE",
    parameter DE0_EN = "FALSE",
    parameter DE1_EN = "FALSE",
    parameter DE2_EN = "FALSE",
    parameter DE3_EN = "FALSE",
    parameter DE4_EN = "FALSE",
    parameter DE5_EN = "FALSE",
    parameter DE6_EN = "FALSE",
    parameter RESET_I_EN = "FALSE",
    parameter RESET_O_EN = "FALSE",
    parameter ICP_SEL = 0,
    parameter LPF_RES = 0,
    parameter LPF_CAP = 0,
    parameter SSC_EN = "FALSE",
    parameter DYN_IDIV_SEL = "FALSE",
    parameter DYN_FBDIV_SEL = "FALSE",
    parameter DYN_MDIV_SEL = "FALSE",
    parameter DYN_ODIV0_SEL = "FALSE",
    parameter DYN_ODIV1_SEL = "FALSE",
    parameter DYN_ODIV2_SEL = "FALSE",
    parameter DYN_ODIV3_SEL = "FALSE",
    parameter DYN_ODIV4_SEL = "FALSE",
    parameter DYN_ODIV5_SEL = "FALSE",
    parameter DYN_ODIV6_SEL = "FALSE",
    parameter DYN_DT0_SEL = "FALSE",
    parameter DYN_DT1_SEL = "FALSE",
    parameter DYN_DT2_SEL = "FALSE",
    parameter DYN_DT3_SEL = "FALSE",
    parameter DYN_ICP_SEL = "FALSE",
    parameter DYN_LPF_SEL = "FALSE",
    parameter longint SIM_FIN_HZ = 27000000,
    parameter longint SIM_LOCK_PS = 100000000,       // 100 us from RESET low to LOCK
    parameter integer SIM_CLKOUT0 = 0
) (
    output reg LOCK = 1'b0,
    output reg CLKOUT0 = 1'b0,
    output reg CLKOUT1 = 1'b0,
    output wire CLKOUT2,
    output wire CLKOUT3,
    output wire CLKOUT4,
    output wire CLKOUT5,
    output wire CLKOUT6,
    output wire CLKFBOUT,
    input  wire CLKIN,
    input  wire CLKFB,
    input  wire RESET,
    input  wire PLLPWD,
    input  wire RESET_I,
    input  wire RESET_O,
    input  wire [5:0] FBDSEL,
    input  wire [5:0] IDSEL,
    input  wire [6:0] MDSEL,
    input  wire [2:0] MDSEL_FRAC,
    input  wire [6:0] ODSEL0,
    input  wire [2:0] ODSEL0_FRAC,
    input  wire [6:0] ODSEL1,
    input  wire [6:0] ODSEL2,
    input  wire [6:0] ODSEL3,
    input  wire [6:0] ODSEL4,
    input  wire [6:0] ODSEL5,
    input  wire [6:0] ODSEL6,
    input  wire [3:0] DT0,
    input  wire [3:0] DT1,
    input  wire [3:0] DT2,
    input  wire [3:0] DT3,
    input  wire [5:0] ICPSEL,
    input  wire [2:0] LPFRES,
    input  wire [1:0] LPFCAP,
    input  wire [2:0] PSSEL,
    input  wire PSDIR,
    input  wire PSPULSE,
    input  wire ENCLK0,
    input  wire ENCLK1,
    input  wire ENCLK2,
    input  wire ENCLK3,
    input  wire ENCLK4,
    input  wire ENCLK5,
    input  wire ENCLK6,
    input  wire SSCPOL,
    input  wire SSCON,
    input  wire [6:0] SSCMDSEL,
    input  wire [2:0] SSCMDSEL_FRAC
);
    assign CLKOUT2 = 1'b0; assign CLKOUT3 = 1'b0; assign CLKOUT4 = 1'b0; assign CLKOUT5 = 1'b0; assign CLKOUT6 = 1'b0;
    assign CLKFBOUT = 1'b0;
    // the setting, in eighths of MDIV
    wire [9:0] m8_port = 10'((128 - MDSEL) * 8 + (7 - MDSEL_FRAC));
    localparam integer M8_PAR = MDIV_SEL * 8 + MDIV_FRAC_SEL;
    wire [9:0] m8_now = (DYN_MDIV_SEL == "TRUE") ? m8_port : 10'(M8_PAR);
    integer    m8 = M8_PAR;
    integer    n_bad_change = 0, n_starts = 0;
    reg  [9:0] m8_q = 10'd0;
    longint    hpi1 = 0, hpr1 = 0, den = 1, acc1 = 0, hpi0 = 0, hpr0 = 0, acc0 = 0;
    reg        run = 1'b0;
    longint    t_lock = 0;
    // RESET: stop; its fall: latch the setting, start the clocks, LOCK after SIM_LOCK_PS
    always @(RESET) begin
        if (RESET) begin run = 1'b0; LOCK = 1'b0; end
        else begin
            m8 = m8_now;  n_starts = n_starts + 1;
            // half period (ps) = ODIV x IDIV x 8 x 1e12 / (2 x f_in x m8) = ODIV x IDIV x 4e12 / (f_in x m8)
            den  = SIM_FIN_HZ * longint'(m8);
            hpi1 = (longint'(ODIV1_SEL) * longint'(IDIV_SEL) * 64'd4000000000000) / den;
            hpr1 = (longint'(ODIV1_SEL) * longint'(IDIV_SEL) * 64'd4000000000000) % den;
            hpi0 = (longint'(ODIV0_SEL) * longint'(IDIV_SEL) * 64'd4000000000000) / den;
            hpr0 = (longint'(ODIV0_SEL) * longint'(IDIV_SEL) * 64'd4000000000000) % den;
            acc1 = 0; acc0 = 0;
            run = 1'b1;
            t_lock = longint'($time) + SIM_LOCK_PS;
        end
    end
    always @(posedge run) begin
        #(SIM_LOCK_PS);
        if (run && longint'($time) >= t_lock) LOCK = 1'b1;
    end
    always begin
        if (!run) begin CLKOUT1 = 1'b0; @(posedge run); end
        acc1 = acc1 + hpr1;
        if (acc1 >= den) begin acc1 = acc1 - den; #(hpi1 + 1); end else #(hpi1);
        if (run) CLKOUT1 = ~CLKOUT1;
    end
    generate if (SIM_CLKOUT0 != 0) begin : g_c0
        always begin
            if (!run) begin CLKOUT0 = 1'b0; @(posedge run); end
            acc0 = acc0 + hpr0;
            if (acc0 >= den) begin acc0 = acc0 - den; #(hpi0 + 1); end else #(hpi0);
            if (run) CLKOUT0 = ~CLKOUT0;
        end
    end endgenerate
    // a divider change while the PLL runs (the sequencer must only change it with the PLL held in reset)
    always @(m8_now) if (!RESET && run && m8_now != 10'(m8)) begin
        n_bad_change = n_bad_change + 1;
        $display("PLL MODEL: MDSEL changed while running (m8 %0d -> %0d) at %0t", m8, m8_now, $time);
    end
endmodule
`default_nettype wire
