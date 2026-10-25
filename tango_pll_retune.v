// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_pll_retune -- TANGO_HDMI_RETUNE (README "Native-rate video (vsync_adjust)"): the HDMI pixel clock retuned AT RUN TIME between
// 74.25 MHz (MDIV 55 + 0/8: CEA 720p60, Tango.ini VSYNC_ADJUST=0 = exactly 60.000 Hz at 1650 x 750) and the build's native
// preset (tango_cfg.vh TANGO_HDMI_PLL_<f>, e.g. 74.5875 MHz = MDIV 55 + 2/8: the TG16's frames to a few ppm with the live
// totals) -- as a CLOCK PAUSE: the clk_pix domain is not reset, its clock is gated off (the board top's DCE, its enable
// from a negedge flop of the PLL's raw output) for the whole re-lock and comes back at the new frequency, so every register
// there just waits (a warm reset is NOT safe: ascal's output-side handshakes and tango_link's cold-boot IRQ).
//
// The request (tango_vsync, clk_pix): a toggle + the setting, issued only at the HDMI beam's line TRIG_Y in the vertical
// blanking, where ascal has no read outstanding (its reads run from the preload 3 lines before line 0 to line ~718).
// Sequence (clk = 50 MHz, PLL_INIT's own clock, which never stops): the toggle through two flops; 4 clocks later (the
// setting has passed its own two flops) a request that differs from the PLL's setting:
//   GATE  gate_off = 1 (the clock enable falls within ~2 clk_pix; the board top also holds the OSER10 serialiser in reset)
//         and PRE clocks of margin;
//   HOLD  PLL_INIT's I_RST for HOLD clocks -- the PLL held in reset, O_LOCK low -- with `native` (hence MDSEL / MDSEL_FRAC,
//         UG306 Tables 5-8 / 5-9: MDSEL = 128 - MDIV integer, MDSEL_FRAC = 7 - eighths) changed halfway;
//   LOCK  I_RST released: PLL_INIT's ICP/LPF sweep (6 steps of 2^17 clocks + the final one, ~16-18 ms) until O_LOCK; no
//         lock within 2^23 clocks (168 ms): I_RST again (PLL_INIT never leaves its last step without a lock);
//   SETTLE 2^SETTLE_LOG2 clocks after the lock (1.3 ms), then gate_off = 0: clk_pix resumes at the new frequency.
// `native` is quasi-static for the clk_pix domain: it changes only while that clock is stopped (the audio / ACR constants
// and the fixed raster follow it there).  Both settings keep MDIV's integer part in PLL_INIT's ">34" bucket (MULTI_FAC is
// static).
//==============================================================================
`default_nettype none
module tango_pll_retune #(
    parameter integer MDIV_N  = 55,               // the native preset: MDIV = MDIV_N + FRAC_N / 8
    parameter integer FRAC_N  = 2,
    parameter integer NATIVE0 = 1,                // the PLL at configuration: 1 the native preset, 0 74.25 MHz
    parameter integer PRE     = 32,               // clocks between gate_off and I_RST
    parameter integer HOLD    = 64,               // clocks of I_RST (1.28 us at 50 MHz)
    parameter integer SETTLE_LOG2 = 16            // clocks after the lock before the clock is released (2^16 = 1.31 ms)
) (
    input  wire       clk,                        // 50 MHz
    input  wire       req_tog,                    // clk_pix: a new request (at the trigger line)
    input  wire       req_native,                 // clk_pix: the requested setting (stable from the toggle on)
    input  wire       lock,                       // PLL_INIT O_LOCK (any clock: synchronised here)
    output reg        irst = 1'b0,                // PLL_INIT I_RST
    output wire [6:0] mdsel,
    output wire [2:0] mdsel_frac,
    output reg        native = (NATIVE0 != 0),    // the PLL's setting now
    output reg        gate_off = 1'b0,            // clk_pix's clock enable, inverted (board top)
    output reg  [7:0] n_retunes = 8'd0            // diagnostics
);
    localparam [6:0] MDSEL_74 = 7'd73, MDSEL_N = 7'(128 - MDIV_N);
    localparam [2:0] FRAC_74  = 3'd7,  FRAC_SN = 3'(7 - FRAC_N);
    assign mdsel      = native ? MDSEL_N : MDSEL_74;
    assign mdsel_frac = native ? FRAC_SN : FRAC_74;

    reg  [2:0] t_s = 3'b000;
    reg  [1:0] n_s = {2{NATIVE0 != 0}};
    reg  [1:0] l_s = 2'b00;
    always @(posedge clk) begin t_s <= {t_s[1:0], req_tog}; n_s <= {n_s[0], req_native}; l_s <= {l_s[0], lock}; end
    localparam [2:0] S_IDLE = 3'd0, S_REQ = 3'd1, S_GATE = 3'd2, S_HOLD = 3'd3, S_LOCK = 3'd4, S_SETTLE = 3'd5;
    reg  [2:0]  st = S_IDLE;
    reg  [23:0] cnt = 24'd0;
    reg         want = (NATIVE0 != 0);
    always @(posedge clk) begin
        cnt <= cnt + 24'd1;
        case (st)
        S_IDLE:   if (t_s[2] != t_s[1]) begin st <= S_REQ; cnt <= 24'd0; end
        S_REQ:    if (cnt == 24'd3) begin
                      want <= n_s[1];
                      if (n_s[1] != native) begin st <= S_GATE; cnt <= 24'd0; gate_off <= 1'b1; end
                      else st <= S_IDLE;
                  end
        S_GATE:   if (cnt == 24'(PRE - 1)) begin st <= S_HOLD; cnt <= 24'd0; irst <= 1'b1; end
        S_HOLD:   begin
                      if (cnt == 24'(HOLD / 2) && native != want) begin native <= want; n_retunes <= n_retunes + 8'd1; end
                      if (cnt == 24'(HOLD - 1)) begin irst <= 1'b0; st <= S_LOCK; cnt <= 24'd0; end
                  end
        S_LOCK:   if (l_s[1] && cnt > 24'd16) begin st <= S_SETTLE; cnt <= 24'd0; end
                  else if (cnt[23]) begin st <= S_HOLD; cnt <= 24'd0; irst <= 1'b1; end   // no lock: sweep again
        default:  if (cnt[SETTLE_LOG2]) begin gate_off <= 1'b0; st <= S_IDLE; end   // S_SETTLE
        endcase
    end
endmodule
`default_nettype wire
