// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tb_ds4_equiv.v -- the existing hardwired DualShock 4 decode (tango_ds4_joy.v, silicon-proven) next to tango_pad_decode with the
// built-in DS4 table, fed the same reports.  tb_ds4_equiv.cpp compares tango_ds4_joy's `rw` (the decode of the captured bytes) with the
// engine's raw word for every report.  Purpose: the DS4 table equals the hardwired decode at RTL level, so mode 0 (hardwired) can be
// retired later without a behaviour change.
`default_nettype none
module tb_ds4_equiv (
    input  wire         clk,        // clk48
    input  wire         clk_pix,    // 74.25 MHz
    input  wire         rst_n,
    input  wire [63:0]  rpt64,
    input  wire         rpt_tog,
    input  wire         rpt_valid,
    // table writer (clk_pix domain)
    input  wire         tw_we,
    input  wire [5:0]   tw_addr,
    input  wire [35:0]  tw_din,
    input  wire [1:0]   cfg_mode,
    input  wire [7:0]   cfg_min_len,
    input  wire [6:0]   cfg_count,
    input  wire         cfg_hold,
    output wire [31:0]  eng_raw,
    output wire         eng_tog,
    output wire [31:0]  old_raw,
    output wire [31:0]  old_rw,
    output wire [15:0]  old_joy
);
    wire [383:0] padmap = {384{1'b0}};
    wire [31:0] core; wire [15:0] vid, pid; wire [1:0] kind; wire [63:0] kbd; wire present;
    tango_ds4_joy #(.QUIET_CYC(2)) u_old (.clk(clk_pix), .rst(~rst_n), .rpt(rpt64), .rpt_tog(rpt_tog), .rpt_valid(rpt_valid),
        .vid_in(16'h054c), .pid_in(16'h09cc), .kind_in(2'd0), .padmap(padmap),
        .rpt16_in(128'd0), .rpt_len_in(8'd0), .rpt2_tog(1'b0), .rpt2_valid(1'b0), .ver_in(16'd0), .traw_in(32'd0), .traw_tog(1'b0), .tmode(1'b0),
        .joy(old_joy), .core(core), .raw(old_raw), .vid(vid), .pid(pid), .kind(kind), .kbd(kbd), .present(present), .rep16(), .rep_len(), .ver(), .live());
    assign old_rw = u_old.rw;
    tango_pad_decode u_eng (.clk(clk), .rst_n(rst_n), .rpt({64'd0, rpt64}), .rpt_len(8'd64), .rpt_tog(rpt_tog), .rpt_valid(rpt_valid),
        .wclk(clk_pix), .tw_we(tw_we), .tw_addr(tw_addr), .tw_din(tw_din), .cfg_mode(cfg_mode), .cfg_rid_en(1'b0), .cfg_rid_val(8'd0),
        .cfg_min_len(cfg_min_len), .cfg_count(cfg_count), .cfg_hold(cfg_hold),
        .raw(eng_raw), .raw_tog(eng_tog), .raw_valid(), .busy());
endmodule
`default_nettype wire
