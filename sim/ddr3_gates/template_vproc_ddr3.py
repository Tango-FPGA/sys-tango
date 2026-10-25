#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
"""template_vproc_ddr3.py <template-dir> -- retarget a Template core export's video-processing bench (sim/tb_vproc.v,
sim/run_tb_vproc.sh) at TANGO_VIDEO_DDR3, for the gate "the Template's tb_vproc with TANGO_VIDEO_DDR3".
It edits the COPY in <template-dir> (an export, e.g. `git archive origin/main | tar -x`, with
sys/ -> this sys-tango checkout), never the Template core's repository:
  tb_vproc.v      under TANGO_VIDEO_DDR3: no SDRAM model; clk_ddr (100 MHz, its own phase) and the IP model
                  (sim/tango_ddr3_model.v, calibrated on silicon: sim/tango_ddr3_silicon.vh) on tango_sys's DDR3 ports
  run_tb_vproc.sh -DTANGO_VIDEO_DDR3; ascal's DDR3 netlist at FRAC 8 (ascal_fixed8_ddr3.v, its shim); tango_ddr3's
                  files and the model in place of tango_sdram and the SDRAM model
Then sim/run_vproc_gates.sh runs unchanged: every video case against its golden (tools/vproc_golden.py), case nn's
frame md5 against the framework's SDRAM frame (REF_NN), the audio cases against iir_model.py."""
import os, sys
T = sys.argv[1]

def edit(path, pairs):
    p = os.path.join(T, path)
    s = open(p).read()
    for a, b in pairs:
        if a not in s:
            sys.exit(f'{path}: anchor not found: {a[:70]!r}')
        s = s.replace(a, b, 1)
    open(p, 'w').write(s)

edit('sim/tb_vproc.v', [
("""    reg [6:0] rcnt = 7'd0;""",
"""`ifdef TANGO_VIDEO_DDR3
    // TANGO_VIDEO_DDR3 (sys-tango sim/ddr3_gates/template_vproc_ddr3.py): the DDR3 IP model on tango_sys's DDR3 ports
`include "tango_ddr3_silicon.vh"
    reg clk_ddr = 1'b0;
    initial begin #3217; forever #5000 clk_ddr = ~clk_ddr; end       // 100 MHz, its own phase
    wire         ddr_calib, ddr_cmd_ready, ddr_cmd_en, ddr_wr_data_rdy, ddr_wr_data_en, ddr_rd_data_valid;
    wire [2:0]   ddr_cmd;
    wire [28:0]  ddr_addr;
    wire [255:0] ddr_wr_data, ddr_rd_data;
    wire [31:0]  ddr_wr_data_mask, ddr_n_refresh, ddr_n_rowmiss, ddr_n_bad_cmd;
    reg          ddr_rst_n = 1'b0;
    initial #200000 ddr_rst_n = 1'b1;
    tango_ddr3_model #(.MEM_LOG2(20), .INIT_CLKS(1000), .CMD_DEPTH(SIL_CMD_DEPTH), .WDATA_DEPTH(SIL_WDATA_DEPTH),
                       .RD_LAT(SIL_RD_LAT), .RD_JITTER(SIL_RD_JITTER), .T_BL(1), .T_ROWMISS(SIL_T_ROWMISS),
                       .T_TURN(SIL_T_TURN), .REF_PERIOD(SIL_REF_PERIOD), .REF_LEN(SIL_REF_LEN)) u_ddr (
        .clk(clk_ddr), .rst_n(ddr_rst_n), .init_calib_complete(ddr_calib),
        .cmd(ddr_cmd), .cmd_en(ddr_cmd_en), .addr(ddr_addr), .cmd_ready(ddr_cmd_ready),
        .wr_data(ddr_wr_data), .wr_data_en(ddr_wr_data_en), .wr_data_end(ddr_wr_data_en),
        .wr_data_mask(ddr_wr_data_mask), .wr_data_rdy(ddr_wr_data_rdy), .rd_data(ddr_rd_data),
        .rd_data_valid(ddr_rd_data_valid), .rd_data_end(), .force_stall(1'b0), .ref_burst(1'b0),
        .fault_en(1'b0), .fault_word(25'd0), .fault_xor(256'd0),
        .n_refresh(ddr_n_refresh), .n_rowmiss(ddr_n_rowmiss), .n_bad_cmd(ddr_n_bad_cmd));
`endif

    reg [6:0] rcnt = 7'd0;"""),
("""    initial if ($value$plusargs("fill_sdram=%h", fill_w)) begin""",
"""`ifndef TANGO_VIDEO_DDR3
    initial if ($value$plusargs("fill_sdram=%h", fill_w)) begin"""),
("""        $display("tb: +fill_sdram: every SDRAM word = %04x", fill_w);
    end""",
"""        $display("tb: +fill_sdram: every SDRAM word = %04x", fill_w);
    end
`else
    // TANGO_VIDEO_DDR3: +fill_sdram=<hex> fills the DDR3 model's memory with the 16-bit word repeated (the board's DDR3 is
    // not zero at power-up either: what ascal reads from a buffer no frame has been written to)
    initial if ($value$plusargs("fill_sdram=%h", fill_w)) begin
        #1;   // after the model's own power-up zeroing (its initial block)
        for (fi = 0; fi < (1 << 20); fi = fi + 1) u_ddr.mem[fi] = {16{fill_w}};
        $display("tb: +fill_sdram: every DDR3 model word = %04x x 16", fill_w);
    end
`endif"""),
("""        .clk_sdram(clk_sdram), .SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA),
        .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE));""",
"""`ifdef TANGO_VIDEO_DDR3
        .clk_ddr(clk_ddr), .ddr_rst(!ddr_rst_n), .ddr_calib(ddr_calib),
        .ddr_cmd_ready(ddr_cmd_ready), .ddr_cmd_en(ddr_cmd_en), .ddr_cmd(ddr_cmd), .ddr_addr(ddr_addr),
        .ddr_wr_data_rdy(ddr_wr_data_rdy), .ddr_wr_data_en(ddr_wr_data_en), .ddr_wr_data(ddr_wr_data),
        .ddr_wr_data_mask(ddr_wr_data_mask), .ddr_rd_data_valid(ddr_rd_data_valid), .ddr_rd_data(ddr_rd_data));
`else
        .clk_sdram(clk_sdram), .SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA),
        .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE));
`endif"""),
("""    wire [31:0] m_errors, m_aref, m_act, m_rd, m_wr, m_masked, m_gap;
    sdram_w9825_model u_mem (""",
"""    wire [31:0] m_errors, m_aref, m_act, m_rd, m_wr, m_masked, m_gap;
`ifdef TANGO_VIDEO_DDR3
    assign m_errors = ddr_n_bad_cmd; assign m_masked = 32'd0;      // the model's protocol errors stand in
`else
    sdram_w9825_model u_mem ("""),
("""        .n_masked(m_masked), .max_rfsh_gap(m_gap));""",
"""        .n_masked(m_masked), .max_rfsh_gap(m_gap));
`endif"""),
])
edit('sim/run_tb_vproc.sh', [
("NETLIST=sys/ascal_ghdl/ascal_fixed8.v", "NETLIST=sys/ascal_ghdl/ascal_fixed8_ddr3.v   # TANGO_VIDEO_DDR3 (template_vproc_ddr3.py)"),
("SHIM=sys/ascal_ghdl/ascal_shim8.v", "SHIM=sys/ascal_ghdl/ascal_shim8_ddr3.v"),
("-DTANGO_VIDEO_ASCAL -DTANGO_VIDEO_OPTS -DTANGO_VIDEO_PROC", "-DTANGO_VIDEO_DDR3 -DTANGO_VIDEO_ASCAL -DTANGO_VIDEO_OPTS -DTANGO_VIDEO_PROC"),
("""--top-module tb_vproc -I"$ROOT/sys" -I"$ROOT/rtl" \\""", """--top-module tb_vproc -I"$ROOT/sys" -I"$ROOT/sys/sim" -I"$ROOT/rtl" \\"""),
("""| grep -v -e tb_template.v -e ascal_wrap_sim.v \\""", """| grep -v -e tb_template.v -e ascal_wrap_sim.v -e tango_sdram.v \\"""),
("sys/tango_audio_filter.v sim/sdram_w9825_model.v sim/tb_vproc.v",
 "sys/tango_audio_filter.v sys/tango_ddr3.v sys/tango_ddr3_arb.v sys/tango_ddr3_vport.v sys/tango_ddr3_cport.v \\\n    sys/tango_ddr3_fifo.v sys/sim/tango_ddr3_model.v sim/tb_vproc.v"),
])
print('template_vproc_ddr3: tb_vproc.v and run_tb_vproc.sh retargeted at TANGO_VIDEO_DDR3 in', T)
