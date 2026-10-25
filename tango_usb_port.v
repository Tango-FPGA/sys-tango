// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_usb_port -- one USB-A host port for a full-speed HID pad (DualShock 4):
// usb_ls_detect + usb_fs_phy + usb_fs_host, all clk48 (usb/: our own
// clean-room FS host, silicon-proven on a DualShock 4; receive CRC16 check and the
// sticky ever_valid guard included).
//
// The wiring is a silicon-proven one (usb_ls_detect DEBOUNCE 960_000, dev_present =
// full-speed status 2, the PHY's debug ports left open).  On the Console 138K the
// D+/D- pins are plain LVCMOS33 GPIO with PULL_MODE=DOWN beside the board's 15k
// pull-downs (board/console138k/tango_console138k.cst).
//
// Outputs are clk48-domain (vid / pid: the device descriptor's, set before the first report and held
// until re-enumeration, so they cross with the report bytes): rpt = report bytes 0..7 of the last CRC-good input
// report (report byte 0 in [7:0]), stable between flips of rpt_tog; rpt_valid is a
// level.  The consumer (tango_ds4_joy) crosses them with the rpt_tog handshake.
//==============================================================================
`default_nettype none
module tango_usb_port #(
    parameter integer DEBOUNCE_CYCLES = 960_000     // 20 ms at 48 MHz
) (
    input  wire        clk48,
    input  wire        rst_n,
    inout  wire        dp,
    inout  wire        dn,
    output wire [63:0] rpt,
    output wire        rpt_valid,
    output wire        rpt_tog,
    output wire [1:0]  status,          // usb_ls_detect: 0 none, 1 LS, 2 FS, 3 invalid
    output wire [7:0]  bad_crc,         // CRC-rejected reports (wraps)
    output wire [15:0] vid,             // GET_DESCRIPTOR(DEVICE); 0000 if not fetched (usb_fs_host)
    output wire [15:0] pid,
    output wire [1:0]  kind,            // 0 pad (DS4 decode), 1 boot keyboard, 2 other low-speed device
    // ---- generic pads, all clk48 unless noted
    output wire [127:0] rpt16,          // the last CRC-good report of 1 byte or more, payload bytes 0..15 (zero beyond rpt_len)
    output wire [7:0]  rpt_len,
    output wire        rpt2_tog,        // flips per such report (the data is untouched for >= 1 ms)
    output wire        rpt2_valid,      // level
    output wire [15:0] ver,             // bcdDevice
    output wire        desc_rdy,        // level: the descriptor phase is over (stored or given up) -- lengths and the store are stable
    output wire [8:0]  len_cfg,
    output wire [10:0] len_rpt,
    output wire [7:0]  len_dev,
    output wire [1:0]  desc_trunc,
    // the descriptor store's read port, in the reader's clock (the link, clk_pix)
    input  wire        rclk,
    input  wire [10:0] ds_ra,
    output wire [7:0]  ds_rd
);
    wire tx_start, tx_valid, tx_last, tx_rd, tx_busy;
    wire [7:0] tx_data;
    wire rx_active, rx_strobe, rx_eop, rx_bserr; wire [7:0] rx_data;
    wire bus_reset;
    usb_ls_detect #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) u_det (
        .clk(clk48), .rst_n(rst_n), .dp(dp), .dn(dn), .status(status));
    // full speed (D+ pulled up) or low speed (D- pulled up: keyboards)
    wire dev_present = (status == 2'd2) || (status == 2'd1);
    wire ls, keepalive;
    usb_fs_phy u_phy (.clk48(clk48), .rst_n(rst_n),
        .dp(dp), .dn(dn), .bus_reset(bus_reset), .ls(ls), .tx_keepalive(keepalive),
        .tx_start(tx_start), .tx_data(tx_data), .tx_valid(tx_valid), .tx_last(tx_last),
        .tx_rd(tx_rd), .tx_busy(tx_busy),
        .rx_active(rx_active), .rx_strobe(rx_strobe), .rx_data(rx_data),
        .rx_eop(rx_eop), .rx_bitstuff_err(rx_bserr),
        .dbg_tx_st(), .dbg_tx_field(), .dbg_tx_bitcnt(),
        .dbg_tx_phase(), .dbg_tx_shift(), .dbg_tx_cur_k(),
        .dbg_rx_phase(), .dbg_rx_bitcnt(), .dbg_rx_ones(),
        .dbg_transitioned(), .dbg_line_j(), .dbg_line_k());
    wire ds_we; wire [10:0] ds_addr; wire [7:0] ds_data;
    usb_fs_host u_host (.clk48(clk48), .rst_n(rst_n),
        .tx_start(tx_start), .tx_data(tx_data), .tx_valid(tx_valid), .tx_last(tx_last),
        .tx_rd(tx_rd), .tx_busy(tx_busy),
        .rx_active(rx_active), .rx_strobe(rx_strobe), .rx_data(rx_data), .rx_eop(rx_eop),
        .bus_reset(bus_reset), .dev_present(dev_present), .dev_ls(status == 2'd1), .ls(ls), .tx_keepalive(keepalive),
        .dbg_st(), .dbg_ep(), .to_count(), .rx_pkts(),
        .last_pid(), .desc_bytes(rpt), .desc_valid(rpt_valid), .rpt_tog(rpt_tog),
        .bad_crc(bad_crc), .dev_vid(vid), .dev_pid(pid), .dev_kind(kind),
        .rpt16(rpt16), .rpt_len(rpt_len), .rpt2_tog(rpt2_tog), .rpt2_valid(rpt2_valid), .dev_ver(ver),
        .ds_we(ds_we), .ds_addr(ds_addr), .ds_data(ds_data), .desc_rdy(desc_rdy), .len_cfg(len_cfg), .len_rpt(len_rpt),
        .len_dev(len_dev), .desc_trunc(desc_trunc));
    tango_desc_ram u_ds (.wclk(clk48), .we(ds_we), .wa(ds_addr), .wd(ds_data), .rclk(rclk), .ra(ds_ra), .rd(ds_rd));
endmodule
`default_nettype wire
