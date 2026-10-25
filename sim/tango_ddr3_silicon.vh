// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_silicon.vh -- sim/tango_ddr3_model.v's parameters calibrated on silicon: a board run of the
// DDR3 bring-up test (ddr3/test).  Each value is chosen so that the test's own
// bench (sim/tb_ddr3_test.v -DSILICON) measures what silicon measured; the model adds its own pipeline (a read's
// latency is RD_LAT + 3 at the tester, a refresh gap in a read stream REF_LEN + ~7, and the FIFO probe counts the
// commands the executor takes while it fills: depth + 2).
//   SIL_CMD_DEPTH   8    silicon: 9 commands taken back to back before cmd_ready first fell (one bank, new rows)
//   SIL_RD_LAT      23   silicon: sequential reads 260 ns min, 270 ns mean (26-27 clocks)
//   SIL_RD_JITTER   2    silicon: sequential 260-460 ns, most within 240-320 ns
//   SIL_T_ROWMISS   7    silicon: random / row-miss reads 324 ns mean (~32 clocks)
//   SIL_REF_PERIOD  760  silicon: one gap >= 160 ns per 7.60 us in a saturating read stream
//   SIL_REF_LEN     26   silicon: the longest read-stream gap 330 ns (33 clocks)
//   SIL_WDATA_DEPTH 32   silicon: wr_data_rdy never fell (>= 9; not measured further)
// The worst case seen (soak maxima over 22 G reads): a read 1,250 ns, cmd_ready low with a command waiting 350 ns
// (SIL_WORST_STALL clocks), all 8 words of a burst back in 650 ns.  The DDR3 benches' worst-case mode is a 35-clock
// executor stall (cmd_ready low) at the worst line, with the core keeping the command queue full of row misses.
// 8 postponed refreshes back to back (~2.2 us) were NOT seen on silicon (longest gap 330 ns): a stress row only.
//==============================================================================
localparam integer SIL_CMD_DEPTH   = 8;
localparam integer SIL_WDATA_DEPTH = 32;
localparam integer SIL_RD_LAT      = 23;
localparam integer SIL_RD_JITTER   = 2;
localparam integer SIL_T_ROWMISS   = 7;
localparam integer SIL_T_TURN      = 2;
localparam integer SIL_REF_PERIOD  = 760;
localparam integer SIL_REF_LEN     = 26;
localparam integer SIL_WORST_STALL = 35;
