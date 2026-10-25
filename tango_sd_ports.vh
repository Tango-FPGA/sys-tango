// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tango_sd_ports.vh -- MiSTer's block-device ports (Template_MiSTer sys/hps_io.sv 128-143, master 3ea1134), for
// cores built with TANGO_SD (build_core.sh TANGO_SD=1; Verilator +define+TANGO_SD).  VDNUM =
// `TANGO_SD_VDNUM (default 1, up to 4); 512-byte blocks (BLKSZ 2); 8-bit data (WIDE 0).  Meanings: tango_sd.v
// and menu-tango docs/spec/tango-spec.html "Block device".  CLK_SYS domain.
input  wire [`TANGO_SD_VDNUM-1:0] img_mounted,     // one clock: drive d's image changed (size 0 = ejected)
input  wire         img_readonly,
input  wire [63:0]  img_size,
output wire [31:0]  sd_lba [`TANGO_SD_VDNUM],
output wire [5:0]   sd_blk_cnt [`TANGO_SD_VDNUM],  // blocks - 1
output wire [`TANGO_SD_VDNUM-1:0] sd_rd,
output wire [`TANGO_SD_VDNUM-1:0] sd_wr,
input  wire [`TANGO_SD_VDNUM-1:0] sd_ack,
input  wire [13:0]  sd_buff_addr,
input  wire [7:0]   sd_buff_dout,
output wire [7:0]   sd_buff_din [`TANGO_SD_VDNUM],
input  wire         sd_buff_wr
