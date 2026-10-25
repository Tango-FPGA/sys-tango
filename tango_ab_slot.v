// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ab_slot -- the core-clock (CLK_SYS) front end of tango_sdram's ports A and B (TANGO_NES_SDRAM).
// A core asks for a byte the way NES_MiSTer / nestang do: it raises oe or we with a stable address (and data)
// and holds them for a window; one request per rising edge of oe|we (a level held across windows is one
// request).  The core also gives `slot_ce`, high for one CLK_SYS clock per slot (the NES: once per PPU dot,
// the clock after the dot boundary, tango_nes.v), with every request of the slot already raised by then.
// Here, on each slot_ce clock: the slot's requests (a request that rose at this clock, or earlier since the
// last slot) are registered into h_* and held until the next slot_ce, and slot_tog flips.  tango_sdram takes
// h_* on the clk_sdram clock after its 2-flop synchroniser saw slot_tog change: the held-data + toggle
// crossing (README "Timing constraints"), bounded by set_max_delay in the core SDC.  At most one request per port
// per slot: a second rise before the slot_ce replaces the first and sets `lost` (debug).
//==============================================================================
`default_nettype none
module tango_ab_slot (
    input  wire        clk,            // CLK_SYS
    input  wire        slot_ce,
    input  wire [21:0] addrA,  input wire oeA, weA,  input wire [7:0] dinA,
    input  wire [21:0] addrB,  input wire oeB, weB,  input wire [7:0] dinB,
    output reg  [21:0] h_addrA = 22'd0,
    output reg         h_oeA   = 1'b0, h_weA = 1'b0,
    output reg  [7:0]  h_dinA  = 8'd0,
    output reg  [21:0] h_addrB = 22'd0,
    output reg         h_oeB   = 1'b0, h_weB = 1'b0,
    output reg  [7:0]  h_dinB  = 8'd0,
    output reg         slot_tog = 1'b0,
    output reg         lost     = 1'b0
);
    wire lvA = oeA | weA, lvB = oeB | weB;
    reg  lvA_d = 1'b0, lvB_d = 1'b0;
    wire riseA = lvA && !lvA_d, riseB = lvB && !lvB_d;
    // a request that rose between two slot_ce clocks
    reg        pA = 1'b0, pB = 1'b0;
    reg [21:0] qA_ad = 22'd0, qB_ad = 22'd0;
    reg        qA_we = 1'b0, qB_we = 1'b0;
    reg [7:0]  qA_d = 8'd0, qB_d = 8'd0;
    always @(posedge clk) begin
        lvA_d <= lvA;  lvB_d <= lvB;
        if (slot_ce) begin
            slot_tog <= ~slot_tog;
            h_oeA <= riseA ? oeA && !weA : pA && !qA_we;   h_weA <= riseA ? weA : pA && qA_we;
            h_addrA <= riseA ? addrA : qA_ad;              h_dinA <= riseA ? dinA : qA_d;
            h_oeB <= riseB ? oeB && !weB : pB && !qB_we;   h_weB <= riseB ? weB : pB && qB_we;
            h_addrB <= riseB ? addrB : qB_ad;              h_dinB <= riseB ? dinB : qB_d;
            if ((riseA && pA) || (riseB && pB)) lost <= 1'b1;
            pA <= 1'b0;  pB <= 1'b0;
        end else begin
            if (riseA) begin if (pA) lost <= 1'b1; pA <= 1'b1; qA_ad <= addrA; qA_we <= weA; qA_d <= dinA; end
            if (riseB) begin if (pB) lost <= 1'b1; pB <= 1'b1; qB_ad <= addrB; qB_we <= weB; qB_d <= dinB; end
        end
    end
endmodule
`default_nettype wire
