// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// gw_dce_sim.v -- SIM-ONLY model of the Gowin GW5A `DCE` (UG306 3.1, dynamic clock enable of a GCLK): CLKOUT = CLKIN while CE.
// Modelled as a plain AND, i.e. the WORST case: it cuts a pulse if CE changes while CLKIN is high -- the board top changes CE
// only on CLKIN's falling edge (two negedge flops), so the model shows whether the design avoids runts, not whether the
// silicon primitive is glitch-free (that is a silicon question).
`default_nettype none
module DCE (input wire CLKIN, input wire CE, output wire CLKOUT);
    assign CLKOUT = CLKIN & CE;
endmodule
`default_nettype wire
