// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3 -- the framework's DDR3 subsystem (README "The DDR3 path"): the scaler port (tango_ddr3_vport, ascal's
// Avalon master), one core port (tango_ddr3_cport, MiSTer's DDRAM_*) and the arbiter (tango_ddr3_arb) onto the Gowin
// DDR3 IP's user interface.  Everything here runs in the IP's clk_out (clk_ddr, 100 MHz) except the core port's core
// side (DDRAM_CLK).  The IP, its PLL (board/console138k/pll_ddr3.v) and the pins are the board top's; sims put
// sim/tango_ddr3_model.v on the same user interface.
// Address map: the scaler's buffers at 0x2000_0000 (its writes limited to 0x2000_0000-0x21FF_FFFF), the
// core window 0x3000_0000-0x3FFF_FFFF (forced by the core port), fence reads at 0.
// Parameters (tango_ddr3_arb): G = consecutive V grants before a waiting core command goes; Q_CORE = the most core
// commands inside the IP at once, which bounds the scaler's wait behind them (lower it for wide inputs).
// TANGO_DDRAM (build option): the core port exists; without it the core side is tied off.
//==============================================================================
`default_nettype none
module tango_ddr3 #(
    parameter integer G      = 8,
    parameter integer Q_CORE = 16
) (
    input  wire          clk,                  // clk_ddr = the IP's clk_out
    input  wire          rst,                  // ddr_rst | !init_calib_complete, synchronous to clk
    // ascal's Avalon master (avl_clk = clk)
    input  wire [27:0]   avl_address,
    input  wire [7:0]    avl_burstcount,
    input  wire          avl_write,
    input  wire [127:0]  avl_writedata,
    input  wire [15:0]   avl_byteenable,
    input  wire          avl_read,
    output wire          avl_waitrequest,
    output wire [127:0]  avl_readdata,
    output wire          avl_readdatavalid,
    // the core's DDRAM_* (MiSTer's emu ports)
    input  wire          DDRAM_CLK,
    input  wire          ddram_rst,
    output wire          DDRAM_BUSY,
    input  wire [7:0]    DDRAM_BURSTCNT,
    input  wire [28:0]   DDRAM_ADDR,
    output wire [63:0]   DDRAM_DOUT,
    output wire          DDRAM_DOUT_READY,
    input  wire          DDRAM_RD,
    input  wire [63:0]   DDRAM_DIN,
    input  wire [7:0]    DDRAM_BE,
    input  wire          DDRAM_WE,
    // the IP's user interface
    input  wire          cmd_ready,
    output wire          cmd_en,
    output wire [2:0]    cmd,
    output wire [28:0]   addr,
    input  wire          wr_data_rdy,
    output wire          wr_data_en,
    output wire [255:0]  wr_data,
    output wire [31:0]   wr_data_mask,
    input  wire          rd_data_valid,
    input  wire [255:0]  rd_data,
    // observation
    output wire [15:0]   obs_v_wait_max,       // the longest avl_waitrequest with a command present (clocks)
    output wire [31:0]   obs_v_win_drops,
    output wire [15:0]   obs_core_inflight_max,
    output wire [31:0]   obs_fences,
    output wire [31:0]   obs_arb_v_wait_max
);
    wire         v_valid, v_write, v_ready;
    wire [28:0]  v_addr;
    wire [255:0] v_wdata;
    wire [31:0]  v_wmask;
    wire         c_valid, c_write, c_ready;
    wire [28:0]  c_addr;
    wire [255:0] c_wdata;
    wire [31:0]  c_wmask;
    wire [1:0]   r_valid;
    wire [255:0] r_data;
    wire [7:0]   wq_max, cq_max;
    wire [31:0]  nwb, nrb;
    // a registered copy of rst per block, placed beside it (timing: a Template DDR3 build failed by
    // 0.93 ns on the framework's reset register -> the video port across the die); all three a clock late together
    (* syn_preserve = 1 *) reg rst_v = 1'b1;
    (* syn_preserve = 1 *) reg rst_c = 1'b1;
    (* syn_preserve = 1 *) reg rst_a = 1'b1;
    always @(posedge clk) begin rst_v <= rst; rst_c <= rst; rst_a <= rst; end
    tango_ddr3_vport u_v (
        .clk(clk), .rst(rst_v),
        .avl_address(avl_address), .avl_burstcount(avl_burstcount), .avl_write(avl_write),
        .avl_writedata(avl_writedata), .avl_byteenable(avl_byteenable), .avl_read(avl_read),
        .avl_waitrequest(avl_waitrequest), .avl_readdata(avl_readdata), .avl_readdatavalid(avl_readdatavalid),
        .q_valid(v_valid), .q_write(v_write), .q_addr(v_addr), .q_wdata(v_wdata), .q_wmask(v_wmask), .q_ready(v_ready),
        .r_valid(r_valid[0]), .r_data(r_data),
        .obs_wbursts(nwb), .obs_rbursts(nrb), .obs_win_drops(obs_v_win_drops), .obs_wait_max(obs_v_wait_max),
        .obs_wq_max(wq_max), .obs_cq_max(cq_max));
`ifdef TANGO_DDRAM
    tango_ddr3_cport u_c (
        .DDRAM_CLK(DDRAM_CLK), .crst(ddram_rst), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT),
        .DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
        .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE),
        .clk(clk), .rst(rst_c),
        .q_valid(c_valid), .q_write(c_write), .q_addr(c_addr), .q_wdata(c_wdata), .q_wmask(c_wmask), .q_ready(c_ready),
        .r_valid(r_valid[1]), .r_data(r_data));
`else
    assign c_valid = 1'b0; assign c_write = 1'b0; assign c_addr = 29'd0; assign c_wdata = 256'd0;
    assign c_wmask = 32'hFFFF_FFFF;
    assign DDRAM_BUSY = 1'b1; assign DDRAM_DOUT = 64'd0; assign DDRAM_DOUT_READY = 1'b0;
`endif
    tango_ddr3_arb #(.NC(1), .G(G), .Q_CORE(Q_CORE)) u_arb (
        .clk(clk), .rst(rst_a),
        .v_valid(v_valid), .v_write(v_write), .v_addr(v_addr), .v_wdata(v_wdata), .v_wmask(v_wmask), .v_ready(v_ready),
        .c_valid(c_valid), .c_write(c_write), .c_addr(c_addr), .c_wdata(c_wdata), .c_wmask(c_wmask), .c_ready(c_ready),
        .r_valid(r_valid), .r_data(r_data),
        .cmd_ready(cmd_ready), .wr_data_rdy(wr_data_rdy), .cmd_en(cmd_en), .cmd(cmd), .addr(addr),
        .wr_data_en(wr_data_en), .wr_data(wr_data), .wr_data_mask(wr_data_mask),
        .rd_data_valid(rd_data_valid), .rd_data(rd_data),
        .obs_core_inflight_max(obs_core_inflight_max), .obs_v_wait_max(obs_arb_v_wait_max), .obs_fences(obs_fences),
        .obs_core_inflight());
endmodule
`default_nettype wire
