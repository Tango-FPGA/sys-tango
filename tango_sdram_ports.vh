// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tango_sdram_ports.vh -- the SDRAM pins, MiSTer's emu names (sys/emu_ports.vh: SDRAM_*), for cores built
// with TANGO_SDRAM (tango_cfg.vh).  As on MiSTer the core brings its own controller and drives the pins
// directly.  The Console 138K's SDRAM module (the "GPIO SDRAM", W9825G6KH class, 16 bit, 4 banks x 8K rows x
// 512 cols = 32 MB): pins in sys/board/console138k/tango_console138k_sdram.cst.  CKE has no pin on this
// board (tied high on the module): SDRAM_CKE is left unconnected by the board top.
output wire         SDRAM_CLK,
output wire         SDRAM_CKE,
output wire [12:0]  SDRAM_A,
output wire [1:0]   SDRAM_BA,
inout  wire [15:0]  SDRAM_DQ,
output wire         SDRAM_DQML,
output wire         SDRAM_DQMH,
output wire         SDRAM_nCS,
output wire         SDRAM_nCAS,
output wire         SDRAM_nRAS,
output wire         SDRAM_nWE
