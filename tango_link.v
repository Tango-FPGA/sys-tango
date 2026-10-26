// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_link -- the BL616 <-> FPGA link slave of a Tango core: FPGA Companion's SPI
// wire format (targets SYS 0, HID 1, OSD 2), with target 2 carrying Tango's OSD
// commands.  Our own RTL, written to the protocol as the Companion FIRMWARE uses it
// (bl616/companion/src, Apache-2.0).  No MiSTle HDL was copied (A2600Nano's
// mcu_spi.v / sysctrl.v / hid.v and sd_card.v, GPL-3, were read for behaviour only; the SD
// STATUS reply below follows sd_card.v's layout; see below and NOTICE).
//
// ---- Wire (FPGA Companion's wire format, unchanged; README "Features"; firmware facts; line numbers are those of
//      MiSTle-Dev/FPGA-Companion src/ at ccb06d2, as menu-tango bl616/companion carries it) --------
//  * SPI mode 1, MSB first: idle SCK low, data changes on the rising edge and is sampled
//    on the falling edge (mcu_hw.c 1039-1045: SPI_MODE1, SPI_BIT_MSB; SPI.md 44-46).
//    Where the BL616 hardware really samples MISO is not documented (the SDK programs
//    MODE1 as SCLK_PH = 0, bflb_spi.c 129-150); silicon rejected a slave whose MISO
//    changed ~35 ns after the falling edge, so MISO here holds from rising edge to rising
//    edge like the proven slaves.  Console clock: 12 MHz asked, 13.33 MHz (mcu_hw.c 980).
//  * One transfer = CSN low .. CSN high.  Byte 0 = target, byte 1 = command, then
//    data.  Every byte is a full-duplex exchange (mcu_hw_spi_tx_u08 returns the MISO
//    byte clocked during the byte it sends, mcu_hw.c 1077-1079).
//  * Replies: the MISO byte during byte i (i >= 3) is computed from target, command,
//    i and live state -- never from the value of byte i-1 -- so it can be loaded the
//    moment byte i-1 completes.  Byte 2 is the dummy/argument byte of every read
//    (the firmware sends it and discards its MISO byte), replies start at byte 3.
//
// ---- Implementation: SCK is a clock (the structure of the silicon-proven FPGA Companion
//      slaves, e.g. A2600Nano mcu_spi.v -- read for facts only, our own code) ---------
//  SCK domain (spi_sck on a BUFG; CSN high = asynchronous reset of the bit/byte counters):
//    * falling edge: shift MOSI in; after 8 bits the byte and its index go to a holding
//      register and a toggle flips (never reset); the reply for the NEXT byte is loaded
//      into the transmit byte (bytes 1 and 2 answer 00);
//    * rising edge: MISO <= bit (7 - bit count) of the transmit byte, so each bit is
//      launched on the rising edge and held until the next one -- the master may sample
//      anywhere from the falling edge to just after the next rising edge.
//  clk domain (clk_pix): the toggle crosses in 3 flops; the byte and index (stable until the
//  next byte, >= 8 SCK periods later) are read on its edge and processed as before.  The
//  reply for byte n is computed in clk as rep_ahead = R(n) with n = last processed index + 2,
//  i.e. TWO bytes ahead, and held until the next byte is processed: when the SCK side samples
//  it (at the end of byte n-1) it has been stable for >= 7 SCK periods, even with no gap
//  between bytes.  Replies never depend on the value of the byte before them, so this is
//  exact; the live values a reply reports (pending interrupts, joystick, cold boot, OSD
//  status) are snapshotted when R(3) is computed and the side effects (ack, re-arm) use the
//  snapshot.  An earlier, oversampled design launched MISO 27-40 ns
//  after the falling edge from clk_pix: on silicon the BL616 never read 5C 42; a sim sweep
//  shows it fails when the master samples at/after the rising edge.
//  CSN/SCK/MOSI are still synchronised into clk, for the transfer boundaries and the debug view.
// ---- SYS target 0 (sysctrl.c) ----------------------------------------------------
//  cmd 0 STATUS    bytes 3..6 = 5C 42 CORE_ID coldboot   (sysctrl.c 38-62:
//                  b0 5C, b1 42 checked; core_id warned if non-zero; coldboot byte
//                  compared with core_id).  Reading does not clear coldboot.
//  cmd 5 IRQ_CTRL  byte 2 = ack mask, byte 3 = pending mask BEFORE the ack
//                  (sysctrl.c 98-104; main.c 72,108 call it with ack 0xff).
//                  pending bit0 SYS (set at reset = cold boot), bit1 HID.
//  cmd 6 IRQ_SRC   byte 3 = {5'b0, buttons 0, port 0, coldboot}; reading clears
//                  coldboot (sysctrl.c 254-257, 298-330 use bit0/1/2).
//  cmd 7 PORT      byte 3 = 0 ports (sysctrl.c 111-128: "!ports" -> no port).
//  cmd 8 READ_CFG  bytes 3.. = CFG_STR then NUL, 0 after (sysctrl.c 443-: c = byte 3).
//  cmd 9 JTAGSEL   byte 2 bit0 stored (dbg_jtagsel) with NO pin effect: the Console
//                  firmware drives GPIO28 (FPGA V14) first and deinitialises the SPI
//                  pins before sending it (mcu_hw.c 2240-2262, 2465-2479), so the pin
//                  is what the core obeys (top: jtagseln = ~bl616_jtagsel, as the
//                  A2600Nano Console 138K top, a2600_top.vhd 321).
//  cmd 1 LEDS, 2 RGB, 3 BUTTONS (reads 0), 4 SETVAL, 10 TIME: accepted, no effect.
//  All SYS/HID state is reset while `hold` (the BL616 JTAG request) is high, as the
//  A2600Nano Console top holds its MCU interface in reset (a2600_top.vhd 322, 871):
//  the coldboot flag and SYS interrupt are therefore raised again afterwards.
// ---- HID target 1 (hid.c 932-948, mcu_hw.c 807-816) ------------------------------
//  cmd 0 STATUS    bytes 3,4 = 5C 42 (the firmware's comment, mcu_hw.c 807).
//  cmd 4 GET_DB9   byte 3 = joy[7:0], byte 4 = joy[15:8], byte 5 = joy_flags;
//                  reading (re-)arms the HID interrupt (main.c 74-77: "by default,
//                  DB9 interrupts are disabled. Reading the DB9 state enables them").
//                  An armed change of joy raises pending bit1 and disarms.
//                  joy is MiSTer order: 0 R, 1 L, 2 D, 3 U, 4 A, 5 B, 6 X, 7 Y, 8 L,
//                  9 R, 10 Select, 11 Start; menu.c 1351-1368 uses bits 0-5 of byte 3.
//  cmds 1-3 (keyboard/mouse/joystick into the core): accepted, no effect.
//  cmd 0x12 REPORT (README "Controllers")  01 12 port: byte 3 = {4'b0, A2 changed, A2 connected,
//                  A1 changed, A1 connected} (both ports: byte 3 is answered before `port` arrives);
//                  bytes 4-11 = VID, PID, raw[31:0] of `port` (0 USB-A1, 1 USB-A2), big-endian, taken
//                  in one clock (byte 4 live, 5-11 from the snapshot of that clock), which clears that
//                  port's "changed" and re-arms the HID interrupt as GET_DB9 does.
//  cmd 0x13 SET_PADMAP  01 13 port kind first (p, a)*: kind 1 = the core map of `port`, joystick bit j =
//                  raw[p_j] | raw[a_j] (FF = none), entries first.. ; kind FE = default map; kind 0 (the
//                  system map) reserved, ignored.  Staged, applied 8 clocks after CSN rises (as SET_STATUS).
//  HID interrupt (pending bit1): an armed change of joy, of either raw word or of the connect flags.
//  GENERIC PADS (the generic pad decoder, tango_pad_decode.v), all additions:
//  cmd 0x17 REPORT2    01 17 port: b3 = REPORT's flags with bit6 A1 live, bit7 A2 live; b4-5 VID, b6-7 PID, b8-9 bcdDevice, b10-13 raw,
//                      b14 payload length, b15-30 report bytes 0-15, b31 {rpt desc truncated, cfg truncated, descriptors ready}.  b4 live,
//                      b5.. from a shift register loaded in one clock (the REPORT idiom), shifted once per reply byte.
//  cmd 0x16 DESC_SELECT 01 16 port sel (0 device, 1 configuration, 2 report descriptor): the read cursor = 0.
//  cmd 0x11 DESCRIPTOR 01 11 port: b3 {A2 rdy, A1 rdy}, b4 0, b5-6 stored length of the selected descriptor of `port` (0 absent), b7.. its
//                      bytes from the cursor (00 past the end); the cursor advances by the data bytes clocked (the upload READ idiom).
//  cmd 0x18 SET_DECODE 01 18 port mode rid_en rid_val min_len count (count x 5 bytes of a 36-bit table entry): staged, applied 8 clocks
//                      after CSN rises; the port's engine is held from the first byte until 10 clocks after.  An incomplete table = mode 0.
//  cmd 0x19 DECODE_STATUS 01 19 port: b3 D1, b4 0, b5 mode, b6 count, b7 {desc rdy, live-and-owned, hold, busy}, b8 transfers accepted, b9 64.
//  cmd 0x1A PAD_CTRL   01 1A bits: bit0 ext events (HID interrupt also on live / descriptors-ready changes), bit1 swap the core's joystick words.
// ---- OSD target 2: Tango (replaces the Companion's OSD target; README "Features") ----------
//  0x01 ENABLE b2 bit0 | 0x02 WRITE ignored (no 128x64 buffer) | 0x10 CAPS bytes 3..15
//  | 0x11 GEOMETRY | 0x12 PALETTE | 0x13 ATLAS | 0x14 GLYPHS | 0x15 ROWS | 0x16 SWAP
//  | 0x17 STATUS byte 3 = {6'b0, swap pending, shown bank}.  Multi-byte fields are
//  big-endian (MSB first), as the Companion sends SDC sector numbers (sdc.c 93-96).
// ---- TANGO_LINK_TAP (TANGO_VIDEO_PROC / TANGO_AUDIO_FILTER, tango_cfg.vh; README "Video and audio processing") ---------
//  The SYS bytes this module has processed go out on tap_* (registered: one clock after the byte is processed) to
//  tango_vproc_regs, which decodes SYS 0x2A 0x2B 0x32 0x33 0x39 0x3E itself.  Byte 3 of those commands is answered
//  here: 0x2B with vp_flags, 0x32 / 0x39 / 0x3E with 1, each only if vp_has says the block exists (else 00, as any
//  unknown command); TANGO_VSYNC_ADJUST builds also 0x70 VSYNC_ADJUST with vs_status (tango_vsync.v decodes its bytes).  The answer is registered when the command byte is processed, so the reply mux gains one
//  term of a flop, as its others.  Without the define, none of this exists.
//==============================================================================
`include "tango_cfg.vh"                         // TANGO_LINK_TAP (TANGO_VIDEO_PROC / TANGO_AUDIO_FILTER)
`default_nettype none
module tango_link #(
    parameter [7:0]  CORE_ID = 8'h00,           // sysctrl.c 49-50: should be 0
    parameter integer ATW    = 48,              // atlas word bits (2 x glyph rows), multiple of 8
    parameter integer ATLAS_ROWS = 24,          // CAPS: glyph rows in the atlas
    parameter integer DL_CAP = 512              // SDC IMAGE STATUS: most bytes offered per chunk
) (
    input  wire            clk,                 // clk_pix
    input  wire            rst,
    // pins (asynchronous to clk)
    input  wire            spi_csn,
    input  wire            spi_sck,
    input  wire            spi_mosi,
    output reg             spi_miso,
    input  wire            spi_miso_pad,        // the U13 pad read back (debug; asynchronous)
    output wire            irq_n,
    input  wire            hold,                // BL616 JTAG request (synchronised): link in reset
    // joystick (clk domain, active high)
    input  wire [15:0]     joy,
    input  wire [7:0]      joy_flags,
    // pads, per USB-A port (tango_ds4_joy, clk domain): raw buttons, device ids, connected (armed)
    input  wire [31:0]     raw1, raw2,          // 1 = USB-A1, 2 = USB-A2
    input  wire [15:0]     vid1, pid1, vid2, pid2,
    input  wire [1:0]      pad_con,             // bit0 USB-A1, bit1 USB-A2
    input  wire [1:0]      pad_kbd,             // bit0 USB-A1 is a boot keyboard, bit1 USB-A2
    input  wire [63:0]     kbd1, kbd2,          // their 8-byte boot reports (byte 0 in [7:0])
    // ---- generic pads, see the header
    input  wire [1:0]      pad_live,            // clk domain (tango_ds4_joy live): a CRC-good report stream is live, armed or not
    input  wire [1:0]      desc_rdy,            // clk48 (tango_usb_port): the descriptor phase is over
    input  wire [15:0]     ver1, ver2,          // bcdDevice (tango_ds4_joy, clk domain)
    input  wire [127:0]    rep16_1, rep16_2,    // the last report of 1 byte or more (clk domain)
    input  wire [7:0]      replen1, replen2,
    input  wire [8:0]      len_cfg1, len_cfg2,  // clk48, stable once desc_rdy
    input  wire [10:0]     len_rpt1, len_rpt2,
    input  wire [7:0]      len_dev1, len_dev2,
    input  wire [1:0]      desc_trunc1, desc_trunc2,
    output reg  [10:0]     ds_ra = 11'd0,       // the descriptor stores' read address (both stores), data 2 clocks later
    input  wire [7:0]      ds1_rd, ds2_rd,
    input  wire [1:0]      eng_busy,            // clk48, tango_pad_decode busy
    output reg             tw_we1 = 1'b0, tw_we2 = 1'b0,   // table memory write (shared address / data), clk domain
    output reg  [5:0]      tw_addr = 6'd0,
    output reg  [35:0]     tw_din = 36'd0,
    output wire [26:0]     cfg1, cfg2,          // per port {hold, count[6:0], min_len[7:0], rid_val[7:0], rid_en, mode[1:0]}
    output wire [1:0]      tmode,               // a table (or quiet) is in force and not being written
    output reg             ext_en = 1'b0,       // PAD_CTRL bit0
    output reg             pad_swap = 1'b0,     // PAD_CTRL bit1
    output reg  [10:0]     ps2_key = 11'd0,     // MiSTer ps2_key (hps_io.sv 258-310), from HID KEYBOARD (1)
    // block device (tango_sd; README "The core interface"): the core's posted block request, the sector buffers
    input  wire [38:0]     sd_req,              // {seq, valid, write, drive[3:0], lba[31:0]}, clk domain
    output reg             sd_in_we = 1'b0,
    output reg  [8:0]      sd_in_wa = 9'd0,
    output reg  [7:0]      sd_in_wd = 8'd0,
    output reg             sd_in_done_tog = 1'b0,
    output reg  [8:0]      sd_out_ra = 9'd0,
    input  wire [7:0]      sd_out_rd,
    output reg             sd_out_done_tog = 1'b0,
    output reg             mnt_tog = 1'b0,
    output reg  [3:0]      mnt_drive = 4'd0,
    output reg             mnt_ro = 1'b0,
    output reg  [63:0]     mnt_size = 64'd0,
    output reg  [383:0]    padmap1,             // SET_PADMAP kind 1, USB-A1 (joystick_1); power-up: default
    output reg  [383:0]    padmap2,             // USB-A2 (joystick_0)
    // OSD write side -> tango_menu_overlay
    output reg             at_we = 1'b0,  output reg [9:0] at_wa = 10'd0, output reg [ATW-1:0] at_wd = 0,
    output reg             gl_we = 1'b0,  output reg [10:0] gl_wa = 11'd0, output reg [31:0] gl_wd = 32'd0,
    output reg             rt_we = 1'b0,  output reg [6:0] rt_wa = 7'd0,  output reg [31:0] rt_wd = 32'd0,
    output reg             pal_we = 1'b0, output reg [4:0] pal_wa = 5'd0, output reg [23:0] pal_wd = 24'd0,
    output reg             geo_we = 1'b0, output reg [3:0] geo_wa = 4'd0, output reg [7:0] geo_wd = 8'd0,
    output reg             en_we = 1'b0,  output reg en_wd = 1'b0,
    output reg             swap_we = 1'b0, output reg swap_wd = 1'b0,
    output reg             stage_busy = 1'b0,
    input  wire            shown_bank,
    input  wire            swap_pending,
    // core interface (clk domain; tango_io crosses it into the core's clock)
    output wire [11:0]     conf_addr,           // READ_CFG: the core's CONF_STR, byte conf_addr ...
    input  wire [7:0]      conf_char,           // ... 2 clocks later (tango_conf_rom in the core)
`ifdef TANGO_PRESET_STATUS
    output reg  [127:0]    status = `TANGO_PRESET_STATUS,   // the build's option word at power-up (tango_cfg.vh)
`else
    output reg  [127:0]    status = 128'd0,     // SYS 0x10 SET_STATUS, whole transfers at a time
`endif
    output reg             status_tog = 1'b0,   // flips when status changes
    input  wire [15:0]     menumask,            // SYS 0x11 GET_MENUMASK
    output reg             dl_we = 1'b0,        // download FIFO write: {ctrl[1:0], byte}
    output reg  [9:0]      dl_wd = 10'd0,       //   ctrl 01 = start (byte = index), 10 = end, 00 = data
    input  wire [11:0]     dl_free,             // free FIFO entries (clk domain)
    output reg  [7:0]      dl_dropped = 8'd0,   // debug: WRITE bytes beyond the space offered
    // debug
    output reg             dbg_jtagsel = 1'b0,
    output reg  [15:0]     dbg_xfers = 16'd0,   // completed transfers (wraps)
    // on-screen link debug (tango_link_dbg_view), clk domain; never reset
    output wire [135:0]    dbg_view
`ifdef TANGO_IOCTL_UPLOAD
    ,
    // ---- upload (TANGO_IOCTL_UPLOAD): SDC_IMAGE sub-commands 4 (UL_SELECT)
    // and 5 (UL_STATUS), decoded below; sub-command 3 (READ) returns tango_io's buffer over MISO. ul_idx /
    // ul_size_o are HELD (not pulsed) and ul_tog flips once per decision -- the same idiom as sd_req's own
    // mnt_drive/mnt_ro/mnt_size + mnt_tog: tango_io detects a new decision on the toggle and samples the held
    // data then, no multi-bit synchroniser needed. ul_ra/ul_rd are tango_io's upload buffer's random-access
    // read port (mirrors tango_sd's out_ra/out_rd), read at the SAME 2-bytes-ahead lookahead every other
    // reply in this file already uses (rep_upd/ahead_n, below).
    output reg             ul_tog = 1'b0,
    output reg  [7:0]      ul_idx = 8'd0,
    output reg  [23:0]     ul_size_o = 24'd0,   // 0 = closed
    output reg  [10:0]     ul_ra = 11'd0,
    input  wire [7:0]      ul_rd,
    input  wire            ul_ready
`endif
`ifdef TANGO_LINK_TAP
    ,
    // the SYS bytes processed (TANGO_LINK_TAP, header), for tango_vproc_regs
    output reg             tap_cmd_stb = 1'b0,  // a SYS transfer's command byte: tap_cmd
    output reg             tap_stb = 1'b0,      // a SYS transfer's data byte n = tap_n: tap_byte
    output reg  [7:0]      tap_cmd = 8'd0,
    output reg  [15:0]     tap_n = 16'd0,
    output reg  [7:0]      tap_byte = 8'd0,
    output reg             tap_end = 1'b0,      // a transfer ended (8 clocks after CSN is seen high)
    input  wire [7:0]      vp_flags,            // SYS 0x2B FLTNUM's reply
    input  wire [3:0]      vp_has               // bit0 FLTNUM, bit1 SET_GAMMA, bit2 SET_AFILTER, bit3 SHADOWMASK
`ifdef TANGO_VSYNC_ADJUST
    ,
    input  wire [7:0]      vs_status            // SYS 0x70 VSYNC_ADJUST's reply (tango_vsync)
`endif
`endif
);
    localparam integer BPC = ATW / 8;           // bytes per atlas column
    localparam [7:0] T_SYS = 8'd0, T_HID = 8'd1, T_OSD = 8'd2, T_SDC = 8'd3;
    localparam [7:0] O_ENABLE = 8'h01, O_CAPS = 8'h10, O_GEOM = 8'h11, O_PAL = 8'h12, O_ATLAS = 8'h13,
                     O_GLYPHS = 8'h14, O_ROWS = 8'h15, O_SWAP = 8'h16, O_STATUS = 8'h17;
    localparam [7:0] TANGO_VERSION = 8'd1;

    // ---------------------------------------------------------------- pins
    // 3-flop synchronisers; [0] is the IOB input register (-ireg_in_iob).
    reg [2:0] sck_s = 3'b000, csn_s = 3'b111, mosi_s = 3'b000;
    always @(posedge clk) begin
        sck_s  <= {sck_s[1:0],  spi_sck};
        csn_s  <= {csn_s[1:0],  spi_csn};
        mosi_s <= {mosi_s[1:0], spi_mosi};
    end
    wire lrst = rst | hold;
    wire act  = !csn_s[1] && !lrst;             // inside a transfer

    // ---------------------------------------------------------------- SCK domain
    reg  [2:0]  k_bc = 3'd0;                    // bits of the current byte (falling edges)
    reg  [15:0] k_idx = 16'd0;                  // index of the byte being received (saturates)
    reg  [6:0]  k_sr = 7'd0;
    reg  [7:0]  k_tx = 8'd0;                    // reply byte being sent
    reg  [7:0]  rep_ahead = 8'd0;               // clk domain: reply for byte (last processed + 2)
    always @(negedge spi_sck or posedge spi_csn) begin
        if (spi_csn) begin
            k_bc <= 3'd0; k_idx <= 16'd0; k_tx <= 8'd0;
        end else begin
            k_sr <= {k_sr[5:0], spi_mosi};
            k_bc <= k_bc + 3'd1;
            if (k_bc == 3'd7) begin
                if (k_idx != 16'hFFFF) k_idx <= k_idx + 16'd1;
                k_tx <= (k_idx < 16'd2) ? 8'h00 : rep_ahead;   // reply for byte k_idx + 1
            end
        end
    end
    reg  [7:0]  k_rx = 8'd0;  reg [15:0] k_rxi = 16'd0;  reg k_tog = 1'b0;   // never reset
    always @(negedge spi_sck)
        if (!spi_csn && k_bc == 3'd7) begin k_rx <= {k_sr, spi_mosi}; k_rxi <= k_idx; k_tog <= ~k_tog; end
    reg k_miso = 1'b0;
    always @(posedge spi_sck or posedge spi_csn)
        if (spi_csn) k_miso <= 1'b0; else k_miso <= k_tx[~k_bc];
    always @(*) spi_miso = k_miso;

    // ---------------------------------------------------------------- clk domain: bytes in
    reg  [2:0]  tog_s = 3'b000;
    reg         rx_stb = 1'b0;
    reg  [7:0]  rx_byte = 8'd0;
    reg  [15:0] rx_idx = 16'd0;
    reg  [15:0] n_q = 16'hFFFE;                 // rx_idx - 2, registered with it
    reg         act_q = 1'b0;
    reg         csn_q1 = 1'b1;
    reg         rep_upd = 1'b0;                 // recompute rep_ahead this clock
    reg  [15:0] ahead_n = 16'd1;                // index of the byte rep_ahead answers
    wire [15:0] nidx = ahead_n;
    // reply pre-decode (clk_pix margin): the reply mux (rsp -> rep_ahead) decided on 16-bit compares of ahead_n
    // and 8-bit compares of tgt / cmd in the clock it loads rep_ahead (an NES build: ahead_n -> rsp ->
    // rep_ahead +1.362 ns).  The same decisions are now flops, loaded in the SAME clocks as the values they
    // decode (ahead_n, tgt, cmd, sdc_sub), so they are never stale and every reply is computed in the same clock.
    reg         a_ge3 = 1'b0;                   // ahead_n >= 3
    reg  [3:0]  a_i = 4'd1;                     // ahead_n if < 16, else 0 (the small byte indices replies use)
    reg         a_sdo = 1'b0;                   // 3 <= ahead_n <= 514 (CORE_SECTOR_OUT data)
`ifdef TANGO_IOCTL_UPLOAD
    reg         a_ulr = 1'b0;                   // ahead_n is a READ (sub 3) data byte of an open upload
`endif
    wire [15:0] an_rx = rx_idx + 16'd2;         // ahead_n loaded from a byte
    reg  [16:0] rq = 17'd1;                     // (tgt, cmd) one-hot, see rq_of
    reg         seen_cmd = 1'b0;                // this transfer's command byte is in
    // ---------------------------------------------------------------- protocol state
    reg  [7:0]  tgt = 8'd0, cmd = 8'd0;
    reg         sys_int = 1'b1;                 // pending bit0: set at reset (cold boot)
    reg         coldboot = 1'b1;
    reg         hid_irq = 1'b0, hid_irq_en = 1'b0;
    reg  [15:0] joy_q = 16'd0, joy_snap = 16'd0;
    // ---------------------------------------------------------------- pad map (Define buttons)
    // entry j = {a[5:0], p[5:0]}, bit 5 = none.  The default = the fixed decode (tango_ds4_joy joy):
    // 0-3 directions <- raw 0-3, A <- 5 circle, B <- 4 cross, X <- 7 triangle, Y <- 6 square,
    // L <- 8 L1, R <- 9 R1, Select <- 12 share, Start <- 13 options, 12-31 none.
    function [383:0] default_padmap(input integer dummy);
        integer e; reg [5:0] p;
        begin
            default_padmap = {384{1'b0}};
            for (e = 0; e < 32; e = e + 1) begin
                case (e)
                    0, 1, 2, 3, 8, 9: p = e[5:0];
                    4: p = 6'd5;   5: p = 6'd4;   6: p = 6'd7;   7: p = 6'd6;
                    10: p = 6'd12; 11: p = 6'd13;
                    default: p = 6'h20;
                endcase
                default_padmap[12*e +: 12] = {6'h20, p};
            end
        end
    endfunction
    localparam [383:0] DEFAULT_PADMAP = default_padmap(0);
    initial begin padmap1 = DEFAULT_PADMAP; padmap2 = DEFAULT_PADMAP; end
    reg          pm_port = 1'b0, pm_dirty = 1'b0, pm_half = 1'b0;
    reg  [7:0]   pm_kind = 8'd0;
    reg  [5:0]   pm_j = 6'd0;
    reg  [383:0] pm_shadow = {384{1'b0}};
    reg          rp_port = 1'b0;
    reg  [63:0]  rp_snap = 64'd0;
    reg  [31:0]  raw1_q = 32'd0, raw2_q = 32'd0;
    reg  [1:0]   con_q = 2'b00;
    reg  [1:0]   rp_chg = 2'b00;
    wire [15:0]  rp_vid_live = rp_port ? vid2 : vid1;
    // keyboards: KBD_REPORT (0x15) and ps2_key from Companion's HID KEYBOARD (ps2helper.c 26-131)
    reg  [63:0]  kb_snap = 64'd0, kbd1_q = 64'd0, kbd2_q = 64'd0;
    wire [63:0]  kb_live = rp_port ? kbd2 : kbd1;
    reg  [31:0]  ps2_raw = 32'd0;               // hps_io.sv 258: ps2_key_raw
    reg          ps2_have = 1'b0, ps2_pend = 1'b0;
    wire         ps2_pressed  = (ps2_raw[15:8] != 8'hF0);                                   // hps_io.sv 259
    wire         ps2_extended = ~ps2_pressed ? (ps2_raw[23:16] == 8'hE0) : (ps2_raw[15:8] == 8'hE0);   // 260
    reg  [7:0]  flags_snap = 8'd0;
    reg  [1:0]  pend_rep = 2'd0;                // IRQ_CTRL: the pending bits actually reported
    // file download (SDC IMAGE): open between SELECT size > 0 and SELECT size 0
    reg         dl_open = 1'b0;
    reg  [7:0]  sdc_sub = 8'd0, dl_index = 8'd0;  reg [23:0] dl_size = 24'd0;
    reg  [11:0] dl_free_snap = 12'd0;
`ifdef TANGO_IOCTL_UPLOAD
    // upload (TANGO_IOCTL_UPLOAD): UL_SELECT's own size accumulator (mirrors dl_size's 3-shift self-clearing
    // idiom exactly: n = 2, 3, 4 fully overwrite all 24 bits regardless of what a PRIOR UL_SELECT left here).
    // ul_idx/ul_size_o/ul_tog (the port outputs posted to tango_io) are set once, at n5, from dl_index (the
    // shared idx register, already valid by n5 as SELECT's own use of it already relies on) and this.
    reg  [23:0] ul_size = 24'd0;
    localparam [23:0] UL_MAX = 24'd2048;    // MUST match tango_io.v's UL_DEPTH (1 << UL_AW, UL_AW = 11)
    // ul_off: the session's read cursor, in buffer bytes already consumed by EARLIER, completed READ
    // transfers of this same open session -- reset to 0 by UL_SELECT's own open decision, advanced at the
    // end of every READ transfer by ul_got (the real data-byte count THAT transfer actually clocked in,
    // counted in the byte-processing block below, never from the reply's own 2-bytes-ahead ahead_n: an
    // open session is read across more than one SPI transfer -- SELECT opens it once, then any number of
    // READ transfers each continue from where the last one left off -- and ahead_n always restarts at 1 for
    // every new transfer, so without ul_off every READ would re-read the same bytes from the start (the
    // first chunk of a split read would match and every later chunk be wrong; the upload benches read
    // sessions split over several transfers for this reason).
    reg  [23:0] ul_off = 24'd0, ul_got = 24'd0;
    // sub3_now: resolves "is THIS transfer's sub-command READ (3)" for the byte that just arrived, valid
    // even for the very first instance (byte 2, the sub byte itself) before the registered sdc_sub has had a
    // clock to update -- rx_byte IS byte 2's value in that one case, exactly as rq_of(rx_byte, cmd) already
    // uses rx_byte directly (not a lagged tgt) at byte 0. Byte 3 onward: sdc_sub (set when byte 2 was
    // processed, long since stable by the time any later byte is) is used directly, as sub1/sub2 do.
    wire sub3_now = (rx_idx == 16'd2) ? (rx_byte == 8'd3) : (sdc_sub == 8'd3);
    // clk_pix timing: ul_lim pre-registers a_ulr's bound (ul_size_o + 4 - ul_off) so the
    // live, per-byte a_ulr compute (below) is a plain compare against a REGISTERED value, not a
    // compare-of-two-sums. ul_off and ul_size_o only change at two infrequent points -- UL_SELECT
    // (ul_size_o set, ul_off reset to 0, both at n5 of a SELECT transfer) and end_cnt==8 of a completed
    // READ transfer (ul_off advanced by ul_got, below) -- and in both cases at least one whole transfer's
    // worth of clk_pix cycles (far more than 1) separates that update from the next transfer's first
    // a_ulr evaluation (a_ulr only ever fires for sub3_now with an_rx>=4, i.e. byte 4+ of a READ transfer,
    // never on the SELECT/end-of-transfer cycle itself), so this registered value (now two-cycle, see the
    // second-pass note below) is always caught up by the time a_ulr needs it. NOTE: registering only on UL_SELECT's own n5 (as first
    // floated) would be a bug -- ul_off ALSO changes mid-session at end_cnt==8, not only at UL_SELECT --
    // so this registers unconditionally, every clock, instead of trying to detect "session changed".
    // Clamp to 0 when ul_off has run past ul_size_o+4 (the subtract would underflow/wrap in 24-bit
    // unsigned math otherwise): the original expression also returns false in that regime, PROVIDED
    // neither of ITS OWN two 24-bit adds wraps either (an_rx>=4 makes ul_off+an_rx >= ul_off+4 >
    // ul_size_o+4 only when that arithmetic doesn't itself wrap at 2^24). sim/tb_ul_lim_equiv.v (now driving
    // the real tango_ul_lim module below, run by sim/run_tb_ul_lim_equiv.sh) sweeps this: exhaustive over a
    // bounded region (ul_size_o in [0,UL_MAX], ul_off in [0,2*UL_MAX], 67.2M checks, 0 mismatches), the
    // whole ul_off range at 12 sizes, and 20M random 24-bit pairs, plus an algebraic argument covering the
    // much larger region a real session can actually reach (ul_off keeps advancing on every completed READ while the slot stays open, not just
    // at UL_SELECT -- see the comment above and the end_cnt == 8 update below -- so [0,2*UL_MAX] is a sweep bound, NOT
    // a reachability bound): equivalence holds wherever ul_off+an_rx can't itself wrap 24 bits, which
    // covers everything up to ~16 MB of READ traffic within one open session (an_rx's 16-bit width caps
    // the per-transfer margin needed below 2^24). A separate, documented divergence was confirmed ONLY at
    // the extreme edge of that range (ul_off = 2^24-1 in the tested samples).
    // Two stages: the single-register form
    // `ul_lim <= (ul_off > (ul_size_o + 4)) ? 0 : (ul_size_o + 4 - ul_off)` was still ONE cycle holding an
    // add, a compare and a subtract (16 logic levels; -0.142 ns in one arcade core's DDR3 build,
    // ul_size_o -> ul_lim).  Now two registered stages, ONE subtractor, the borrow IS the clamp:
    //   stage 1  ul_sz4 <= ul_size_o + 4                        (24-bit, wraps exactly as before)
    //   stage 2  {brw,diff} = {0,ul_sz4} - {0,ul_off}           (25-bit; brw <=> ul_off > ul_sz4)
    //            ul_lim <= brw ? 0 : diff
    // Identical value to the old ul_lim for every (ul_size_o, ul_off), but it lags its inputs by TWO clocks,
    // not one (ul_off alone: still one; ul_size_o: two).  Harmless: a_ulr (above) first reads ul_lim at an
    // rx_stb with an_rx >= 4, i.e. rx_idx >= 2 = the 3rd byte of a LATER transfer, >= 24 SCK periods
    // (>= 1.8 us at the 13.33 MHz SDC limit) after CSN falls and after the UL_SELECT n5 / end_cnt==8 update
    // that moved ul_off/ul_size_o -- against 2 clk_pix cycles of lag.
    // The module is separate only so sim/tb_ul_lim_equiv.v can drive the real RTL; synthesis flattens it.
    wire [23:0] ul_lim;
    tango_ul_lim u_ul_lim (.clk(clk), .ul_size_o(ul_size_o), .ul_off(ul_off), .ul_lim(ul_lim));
`endif
    // one FIFO entry stays in reserve for the END marker
    wire [11:0] dl_offer_raw = (dl_free == 12'd0) ? 12'd0 : dl_free - 12'd1;
    wire [11:0] dl_offer = (dl_offer_raw > DL_CAP) ? DL_CAP[11:0] : dl_offer_raw;
    // block device: the posted request is shown (STATUS, IRQ) until the block is served (its seq noted)
    reg         sd_served = 1'b0;               // seq of the last block served
    wire        sd_shown = sd_req[37] && (sd_req[38] != sd_served);
    wire [3:0]  sd_drv = sd_req[35:32];
    reg  [9:0]  sd_cnt = 10'd0;                 // CORE_SECTOR_IN bytes / CORE_SECTOR_OUT reads of this transfer
    reg         sd_ok = 1'b0;                   // this transfer matches the shown request
    reg         mnt_dirty = 1'b0;
    wire        sdc_int = (dl_open && (dl_offer >= 12'd256)) || sd_shown;   // a level: an IRQ ack does not clear it
`ifdef TANGO_PRESET_STATUS
    reg  [127:0] st_stage = `TANGO_PRESET_STATUS;
`else
    reg  [127:0] st_stage = 128'd0;
`endif
    reg st_dirty = 1'b0;  reg [7:0] st_first = 8'd0;  reg [3:0] end_cnt = 4'd15;
    wire [7:0]  pending = {4'd0, sdc_int, 1'b0, hid_irq, sys_int};
    assign irq_n = ~(sys_int | hid_irq | sdc_int);

    // ---------------------------------------------------------------- reply for byte nidx
    function [7:0] caps_byte(input [3:0] i);
        case (i)
            4'd0:  caps_byte = 8'h5A;           // 'Z'
            4'd1:  caps_byte = 8'h4F;           // 'O'
            4'd2:  caps_byte = TANGO_VERSION;
            4'd3:  caps_byte = 8'h04;           // atlas columns 1024 (BE)
            4'd4:  caps_byte = 8'h00;
            4'd5:  caps_byte = ATLAS_ROWS;      // glyph rows per atlas column
            4'd6:  caps_byte = 8'h04;           // glyph slots per bank 1024 (BE)
            4'd7:  caps_byte = 8'h00;
            4'd8:  caps_byte = 8'd32;           // rows per bank (row 31 = banner)
            4'd9:  caps_byte = 8'h05;           // max panel w 1280 (BE)
            4'd10: caps_byte = 8'h00;
            4'd11: caps_byte = 8'h02;           // max panel h 720 (BE)
            4'd12: caps_byte = 8'hD0;
            default: caps_byte = 8'h00;
        endcase
    endfunction
    // READ_CFG: the CONF_STR ROM (tango_conf_rom in the core: q follows addr 2 clocks later) is read one
    // byte AHEAD: within a transfer the next byte to answer is always ahead_n + 1 (rx_idx counts up by one),
    // and ahead_n holds for ~40 clocks between bytes (one SPI byte), so cfg_q -- 3 clocks after conf_a_q --
    // is CONF_STR[ci] long before rsp uses it.  No part of the ROM is in the rsp -> rep_ahead clock.
    wire [15:0] ci_next = nidx - 16'd2;         // ci of the byte after this one
    reg  [11:0] conf_a_q = 12'hFFF;
    reg         conf_end_q = 1'b1;
    reg  [7:0]  cfg_q = 8'h00;
    always @(posedge clk) begin
        conf_a_q   <= (ci_next > 16'd4095) ? 12'hFFF : ci_next[11:0];
        conf_end_q <= (ci_next > 16'd4094);
        cfg_q      <= conf_end_q ? 8'h00 : conf_char;
    end
    assign conf_addr = conf_a_q;
    wire [7:0]  cfg_ch = cfg_q;                 // the core's CONF_STR (4095 bytes max)
    // (tgt, cmd) -> the reply decoder's one-hot rq; loaded wherever tgt / cmd are (lrst, bytes 0 and 1)
    function [16:0] rq_of(input [7:0] t, input [7:0] c);
        rq_of = {t == T_HID && c == 8'h19, t == T_HID && c == 8'h11, t == T_HID && c == 8'h17,                  // 16 15 14
                 t == T_SDC && c == 8'd8,   t == T_SDC && c == 8'd10,  t == T_SDC && c == 8'd1,           // 13 12 11
                 t == T_OSD && c == O_STATUS, t == T_OSD && c == O_CAPS,                                 // 10 9
                 t == T_HID && c == 8'h12,  t == T_HID && c == 8'h15,  t == T_HID && c == 8'd4,          // 8 7 6
                 t == T_HID && c == 8'd0,   t == T_SYS && c == 8'h11,  t == T_SYS && c == 8'd8,          // 5 4 3
                 t == T_SYS && c == 8'd6,   t == T_SYS && c == 8'd5,   t == T_SYS && c == 8'd0};         // 2 1 0
    endfunction
    reg         sub0 = 1'b1;                    // sdc_sub == 0, loaded with sdc_sub
    wire [7:0]  pst = {2'd0, pad_kbd, rp_chg[1], pad_con[1], rp_chg[0], pad_con[0]};   // REPORT / KBD byte 3
    reg  [7:0]  r_sys0, r_hid4, r_hid15, r_hid12, r_caps, r_sdc1, r_sdc8;
    // ---------------------------------------------------------------- generic pads
    reg  [1:0]  rdy_s1 = 2'd0, rdy_s2 = 2'd0;          // desc_rdy (clk48) -> clk, 2 flops per port
    reg  [1:0]  busy_s1 = 2'd0, busy_s2 = 2'd0;
    wire [1:0]  rdy_c = rdy_s2;
    reg  [1:0]  live_q = 2'd0, rdy_q = 2'd0;
    // REPORT2: b4 live (VID high byte), b5..b31 from this shift register (27 bytes, loaded in the clock b4 is computed, shifted once per byte)
    reg  [215:0] pi_sr = 216'd0;
    reg          a_pi = 1'b0;                          // 5 <= ahead_n <= 31
    wire [15:0]  rp_ver   = rp_port ? ver2 : ver1;
    wire [15:0]  rp_pid_l = rp_port ? pid2 : pid1;
    wire [31:0]  rp_raw_l = rp_port ? raw2 : raw1;
    wire [7:0]   rp_len_l = rp_port ? replen2 : replen1;
    wire [127:0] rp_rep_l = rp_port ? rep16_2 : rep16_1;
    // b31: bit0 descriptors ready, bit1 configuration truncated, bit2 report descriptor truncated
    wire [7:0]   rp_b31   = rp_port ? {5'd0, desc_trunc2[1], desc_trunc2[0], rdy_c[1]} : {5'd0, desc_trunc1[1], desc_trunc1[0], rdy_c[0]};
    reg  [127:0] rep_be;                               // report bytes, byte 0 first (most significant)
    integer rbi;
    always @(*) for (rbi = 0; rbi < 16; rbi = rbi + 1) rep_be[8*(15-rbi) +: 8] = rp_rep_l[8*rbi +: 8];
    // DESCRIPTOR: the selected descriptor of `port`, its stored length, the cursor
    reg  [1:0]   dsel = 2'd0;                          // DESC_SELECT: 0 device, 1 configuration, 2 report descriptor
    reg  [10:0]  ds_off = 11'd0;                       // cursor (bytes already read in earlier transfers of this selection)
    reg  [10:0]  ds_got = 11'd0;                       // data bytes clocked in this transfer
    reg  [10:0]  dsc_len = 11'd0;                      // stored length of (port, dsel), 0 when the port's descriptors are not ready
    reg  [10:0]  dsc_lim = 11'd0;                      // bytes left from the cursor
    reg          a_dsc = 1'b0;                         // ahead_n is a DESCRIPTOR data byte inside the stored length
    always @(posedge clk) begin
        rdy_s1 <= desc_rdy; rdy_s2 <= rdy_s1;
        busy_s1 <= eng_busy; busy_s2 <= busy_s1;
        case (dsel)
            2'd0:    dsc_len <= rp_port ? {3'd0, len_dev2} : {3'd0, len_dev1};
            2'd1:    dsc_len <= rp_port ? {2'd0, len_cfg2} : {2'd0, len_cfg1};
            default: dsc_len <= rp_port ? len_rpt2 : len_rpt1;
        endcase
        dsc_lim <= (!rdy_c[rp_port] || dsc_len <= ds_off) ? 11'd0 : dsc_len - ds_off;
    end
    wire [7:0]  ds_rd_sel = rp_port ? ds2_rd : ds1_rd;
    // table programming state (SET_DECODE) per port
    reg  [1:0]   cm_hold = 2'b00, cm_ok = 2'b00;        // engine held while the table is written; a table is in force (and the device is the one it was made for)
    reg  [1:0]   cm_mode [0:1];
    reg          cm_rid_en [0:1];
    reg  [7:0]   cm_rid_val [0:1], cm_min_len [0:1];
    reg  [6:0]   cm_count [0:1];
    initial begin cm_mode[0] = 2'd0; cm_mode[1] = 2'd0; cm_rid_en[0] = 1'b0; cm_rid_en[1] = 1'b0; cm_rid_val[0] = 8'd0; cm_rid_val[1] = 8'd0;
                  cm_min_len[0] = 8'd0; cm_min_len[1] = 8'd0; cm_count[0] = 7'd0; cm_count[1] = 7'd0; end
    reg          sd_port = 1'b0, sd_dirty = 1'b0, sd_bad = 1'b0;
    reg  [8:0]   sd_j = 9'd0;                           // payload bytes received (after the port byte)
    reg  [1:0]   sd_mode = 2'd0;  reg sd_rid_en = 1'b0;  reg [7:0] sd_rid_val = 8'd0, sd_min_len = 8'd0;  reg [6:0] sd_count = 7'd0;
    reg  [6:0]   sd_e = 7'd0;  reg [2:0] sd_k = 3'd0;  reg [39:0] sd_sh = 40'd0;
    reg  [7:0]   sd_xfers = 8'd0;
    reg  [4:0]   sd_hs = 5'd0;  reg sd_rel = 1'b0;     // header byte one-hot; the hold is released at end_cnt 10
    reg          dsc_cnt = 1'b0;                        // DESCRIPTOR: counting data bytes
    assign cfg1 = {cm_hold[0], cm_count[0], cm_min_len[0], cm_rid_val[0], cm_rid_en[0], cm_ok[0] ? cm_mode[0] : 2'd0};
    assign cfg2 = {cm_hold[1], cm_count[1], cm_min_len[1], cm_rid_val[1], cm_rid_en[1], cm_ok[1] ? cm_mode[1] : 2'd0};
    assign tmode = {cm_ok[1] && cm_mode[1] != 2'd0 && !cm_hold[1], cm_ok[0] && cm_mode[0] != 2'd0 && !cm_hold[0]};
    wire [7:0]  r_dst_mode  = rp_port ? {6'd0, cm_ok[1] ? cm_mode[1] : 2'd0} : {6'd0, cm_ok[0] ? cm_mode[0] : 2'd0};
    wire [7:0]  r_dst_count = rp_port ? {1'b0, cm_count[1]} : {1'b0, cm_count[0]};
    wire [7:0]  r_dst_flags = rp_port ? {rdy_c[1], tmode[1] && pad_live[1], cm_hold[1], busy_s2[1], 4'd0} : {rdy_c[0], tmode[0] && pad_live[0], cm_hold[0], busy_s2[0], 4'd0};
`ifdef TANGO_LINK_TAP
    reg  [7:0]  vp_rep = 8'd0;                  // byte 3 of this transfer's SYS 0x2B / 0x32 / 0x39 / 0x3E, else 0
`endif
    always @(*) begin
        case (a_i)                                                  // SYS STATUS
            4'd3: r_sys0 = 8'h5C;  4'd4: r_sys0 = 8'h42;  4'd5: r_sys0 = CORE_ID;  4'd6: r_sys0 = {7'd0, coldboot};
            default: r_sys0 = 8'h00;
        endcase
        case (a_i)                                                  // GET_DB9: byte 3 live, captured in this clock
            4'd3: r_hid4 = joy[7:0];  4'd4: r_hid4 = joy_snap[15:8];  4'd5: r_hid4 = flags_snap;
            default: r_hid4 = 8'h00;
        endcase
        case (a_i)                                                  // KBD_REPORT: byte 4 live, snapshot this clock
            4'd3: r_hid15 = pst;              4'd4: r_hid15 = kb_live[7:0];     4'd5: r_hid15 = kb_snap[15:8];
            4'd6: r_hid15 = kb_snap[23:16];   4'd7: r_hid15 = kb_snap[31:24];   4'd8: r_hid15 = kb_snap[39:32];
            4'd9: r_hid15 = kb_snap[47:40];   4'd10: r_hid15 = kb_snap[55:48];  4'd11: r_hid15 = kb_snap[63:56];
            default: r_hid15 = 8'h00;
        endcase
        case (a_i)                                                  // REPORT: byte 4 live, snapshot this clock
            4'd3: r_hid12 = pst;              4'd4: r_hid12 = rp_vid_live[15:8]; 4'd5: r_hid12 = rp_snap[55:48];
            4'd6: r_hid12 = rp_snap[47:40];   4'd7: r_hid12 = rp_snap[39:32];   4'd8: r_hid12 = rp_snap[31:24];
            4'd9: r_hid12 = rp_snap[23:16];   4'd10: r_hid12 = rp_snap[15:8];   4'd11: r_hid12 = rp_snap[7:0];
            default: r_hid12 = 8'h00;
        endcase
        r_caps = (a_i >= 4'd3) ? caps_byte(a_i - 4'd3) : 8'h00;     // CAPS bytes 3..15 (ci < 13)
        case (a_i)                                                  // SDC STATUS (Companion sd_card.v 342-361)
            4'd3:  r_sdc1 = 8'd1 << sd_drv[2:0];
            4'd4:  r_sdc1 = sd_req[31:24];  4'd5: r_sdc1 = sd_req[23:16];  4'd6: r_sdc1 = sd_req[15:8];
            4'd7:  r_sdc1 = sd_req[7:0];    4'd8: r_sdc1 = 8'h80;          4'd9: r_sdc1 = {7'd0, sd_req[36]};
            4'd10: r_sdc1 = {4'd0, sd_drv};
            default: r_sdc1 = 8'h00;
        endcase
        if (!sd_shown) r_sdc1 = 8'h00;
`ifdef TANGO_IOCTL_UPLOAD
        // STATUS (sub 0, unchanged) and UL_STATUS (sub 5, new): the same byte-4/5/6 shape, STATUS's own
        // {dl_open, dl_free_snap} or UL_STATUS's {ul_ready, ul_size_o} depending on which sub-command this
        // transfer is -- sub0/sub5 are mutually exclusive (sdc_sub can't be both), so selecting between them
        // here, instead of a separate rsp term, keeps the non-upload build's case statement (below) untouched.
        case (a_i)
            4'd4: r_sdc8 = sub0 ? {dl_open, 7'd0} : {ul_ready, 7'd0};
            4'd5: r_sdc8 = sub0 ? {4'd0, dl_free_snap[11:8]} : ul_size_o[15:8];
            4'd6: r_sdc8 = sub0 ? dl_free_snap[7:0] : ul_size_o[7:0];
            default: r_sdc8 = 8'h00;
        endcase
        if (!sub0 && !sub5) r_sdc8 = 8'h00;
`else
        case (a_i)                                                  // IMAGE STATUS: bytes 4, 5, 6
            4'd4: r_sdc8 = {dl_open, 7'd0};  4'd5: r_sdc8 = {4'd0, dl_free_snap[11:8]};  4'd6: r_sdc8 = dl_free_snap[7:0];
            default: r_sdc8 = 8'h00;
        endcase
        if (!sub0) r_sdc8 = 8'h00;
`endif
    end
    // generic pads: REPORT2 (b3 flags with the live bits, b4 VID high live, b5.. the shift register), DESCRIPTOR, DECODE_STATUS
    wire [7:0] r_rep2 = a_pi ? pi_sr[215:208] : (a_i == 4'd3) ? {pad_live[1], pad_live[0], pst[5:0]} : (a_i == 4'd4) ? rp_vid_live[15:8] : 8'h00;
    wire [7:0] r_dsc  = a_dsc ? ds_rd_sel : (a_i == 4'd3) ? {6'd0, rdy_c[1], rdy_c[0]} :
                        (a_i == 4'd5) ? {5'd0, dsc_len[10:8] & {3{rdy_c[rp_port]}}} : (a_i == 4'd6) ? (dsc_len[7:0] & {8{rdy_c[rp_port]}}) : 8'h00;
    wire [7:0] r_dst  = (a_i == 4'd3) ? 8'hD1 : (a_i == 4'd5) ? r_dst_mode : (a_i == 4'd6) ? r_dst_count : (a_i == 4'd7) ? r_dst_flags :
                        (a_i == 4'd8) ? sd_xfers : (a_i == 4'd9) ? 8'd64 : 8'h00;
    // rsp = the reply for byte nidx (0 below byte 3): one term per (tgt, cmd), the rq bits are exclusive
    wire [7:0] rsp = {8{a_ge3}} & (
          ({8{rq[0]}}  & r_sys0)
        | ({8{rq[1]  && a_i == 4'd3}} & pending)                                            // IRQ_CTRL (before the ack)
        | ({8{rq[2]  && a_i == 4'd3}} & {7'd0, coldboot})                                   // IRQ_SRC
        | ({8{rq[3]}}  & cfg_ch)                                                            // READ_CFG
        | ({8{rq[4]}}  & ((a_i == 4'd3) ? menumask[15:8] : (a_i == 4'd4) ? menumask[7:0] : 8'h00))
        | ({8{rq[5]}}  & ((a_i == 4'd3) ? 8'h5C : (a_i == 4'd4) ? 8'h42 : 8'h00))          // HID STATUS
        | ({8{rq[6]}}  & r_hid4)
        | ({8{rq[7]}}  & r_hid15)
        | ({8{rq[8]}}  & r_hid12)
        | ({8{rq[9]}}  & r_caps)
        | ({8{rq[10] && a_i == 4'd3}} & {6'd0, swap_pending, shown_bank})                   // OSD STATUS
        | ({8{rq[11]}} & r_sdc1)
        | ({8{rq[12] && a_sdo}} & sd_out_rd)                                                // CORE_SECTOR_OUT
        | ({8{rq[13]}} & r_sdc8)
        | ({8{rq[14]}} & r_rep2)                                                            // REPORT2
        | ({8{rq[15]}} & r_dsc)                                                             // DESCRIPTOR
        | ({8{rq[16]}} & r_dst)                                                             // DECODE_STATUS
`ifdef TANGO_IOCTL_UPLOAD
        | ({8{rq[13] && a_ulr}} & ul_rd)                                                    // READ (sub 3)
`endif
`ifdef TANGO_LINK_TAP
        | ({8{a_i == 4'd3}} & vp_rep)                                                        // 0x2B 0x32 0x39 0x3E
`endif
        );

    // ---- byte processing pre-decode (clk_pix margin, set 2).  The byte processing below used to decode
    // rx_idx / n_q (16-bit compares), tgt / cmd (8-bit compares) and the pad-map and SET_STATUS indices
    // (12 * pm_j, st_first + n - 1) in the clock it writes, and that cone ended on every write register:
    // the worst clk_pix paths of set 1 (NES n_q -> gl_wd +0.085 ns; A2600 12*pm_j DSP -> pm_shadow).  Now
    // every such decision is a flop: the index flags are registered with n_q from the same k_rxi (stable,
    // >= 8 SCK periods, when the toggle is seen), and the state flags (tgt / cmd, pad-map slot, sdc_sub,
    // ecnt vs bpc_m1, ...) are registered every clock from state that only changes in a byte-processing
    // clock, >= 8 SCK periods (~40 clk_pix) before the next one reads them.  Same clock, same writes.
    wire [15:0] kn = k_rxi - 16'd2;             // n of the byte arriving
    reg         i0 = 1'b0, i1 = 1'b0;           // rx_idx == 0, 1
    reg         n0 = 1'b0, n1 = 1'b0, n2 = 1'b0, n5 = 1'b0, n11 = 1'b0;          // n == 0, 1, 2, 5, 11
    reg         n_lt12 = 1'b0, n_le4 = 1'b0, n_eq4 = 1'b0, n_le8 = 1'b0, n_eq8 = 1'b0, n_le512 = 1'b0;
    reg         st_ok = 1'b0;  reg [15:0] st_sel = 16'd0;   // SET_STATUS: byte in range, one-hot byte slot
    always @(posedge clk)
        if (tog_s[2] != tog_s[1]) begin
            i0 <= (k_rxi == 16'd0); i1 <= (k_rxi == 16'd1);
            n0 <= (kn == 16'd0); n1 <= (kn == 16'd1); n2 <= (kn == 16'd2); n5 <= (kn == 16'd5); n11 <= (kn == 16'd11);
            n_lt12 <= (kn < 16'd12); n_le4 <= (kn <= 16'd4); n_eq4 <= (kn == 16'd4);
            n_le8 <= (kn <= 16'd8); n_eq8 <= (kn == 16'd8); n_le512 <= (kn <= 16'd512);
            // st_first was written by byte n = 0, >= 8 SCK periods ago (unused for the n = 0 byte itself)
            st_ok  <= ({8'd0, st_first} + kn - 16'd1 < 16'd16);
            st_sel <= 16'd1 << (st_first[3:0] + kn[3:0] - 4'd1);
        end
    always @(posedge clk) begin
        act_q  <= act;
        csn_q1 <= csn_s[1];
        tog_s  <= {tog_s[1:0], k_tog};
        rx_stb <= 1'b0;
        rep_upd <= 1'b0;
        if (tog_s[2] != tog_s[1]) begin             // a byte from the SCK domain (stable until the next)
            rx_stb <= !lrst; rx_byte <= k_rx; rx_idx <= k_rxi; n_q <= k_rxi - 16'd2;
        end
        if (rx_stb) begin ahead_n <= an_rx; rep_upd <= 1'b1;               // (tgt/cmd are updated this clock too)
            a_ge3 <= (an_rx >= 16'd3); a_i <= (an_rx < 16'd16) ? an_rx[3:0] : 4'd0;
            a_sdo <= (an_rx >= 16'd3) && (an_rx <= 16'd514);
            a_pi  <= (an_rx >= 16'd5) && (an_rx <= 16'd31);
            a_dsc <= (an_rx >= 16'd7) && ((an_rx - 16'd7) < {5'd0, dsc_lim});
`ifdef TANGO_IOCTL_UPLOAD
            // READ (sub 3) data byte: sub3_now below (its own comment) resolves the sub-command for the byte
            // that just arrived even for the very first one (byte 2, before sdc_sub itself has updated).
            // Bound against the SESSION's own global position (ul_off + this transfer's own local position,
            // byte 4 = local position 0), not just this transfer's own an_rx: a bound that only looked at
            // an_rx would be correct for the first chunk of a session (ul_off = 0) and would return STALE
            // buffer bytes, instead of 00, for any later chunk once ul_off is nonzero (the upload benches
            // read 5 bytes past a 37-byte session's end and check them against 00).
            // clk_pix timing: was `(ul_off + {8'd0, an_rx}) < (ul_size_o + 24'd4)`, a 24-bit
            // add of ul_off+an_rx AND a 24-bit add of ul_size_o+4, both combinational in the same cycle as
            // an_rx (the clk_pix-critical cone: rx_idx -> an_rx -> a_ulr). ul_lim (above) pre-registers
            // ul_size_o+4-ul_off every clock, so the live per-byte compute is just one magnitude compare.
            a_ulr <= sub3_now && (an_rx >= 16'd4) && ({8'd0, an_rx} < ul_lim)
                     && (ul_size_o != 24'd0);
`endif
        end
        if (csn_q1 && !csn_s[1]) begin                                     // a transfer starts
            ahead_n <= 16'd1; rep_upd <= 1'b1; a_ge3 <= 1'b0; a_i <= 4'd1; a_sdo <= 1'b0; a_pi <= 1'b0; a_dsc <= 1'b0;
`ifdef TANGO_IOCTL_UPLOAD
            a_ulr <= 1'b0;
`endif
        end
        if (rep_upd) rep_ahead <= rsp;
        if (act_q && !act) dbg_xfers <= dbg_xfers + 16'd1;
    end

    // ---------------------------------------------------------------- debug (never reset)
    // Rows for tango_link_dbg_view, 8 bits each, row 0 first:
    //  0 CSN falling edges   1 bytes received   2 last target   3 last command   4 last data byte
    //  5 last reply byte prepared (rep_ahead)   6 SCK falls in the last transfer
    //  7 {min SCK high run, min SCK low run} in clk samples over the last transfer (sat. 15)
    //  8 SYS STATUS transfers that reached byte 6
    //  9 {BL616 JTAG request (hold), 0, CSN, SCK, MOSI (live, synchronised), IRQn, SCK moved in ~56 ms, in transfer}
    // 10 {bit count at the last CSN rise [2:0] (0 = whole bytes), coldboot, SYS irq, HID irq, HID armed, stage_busy}
    // 12 SCK falls (in transfers) where the MISO PAD read back != the value the fabric drives (wraps)
    // 13 SCK falls (in transfers) where the MISO pad read 1 (wraps)
    // 14 / 15 the reply bytes the core prepared for bytes 3 / 4 of the last SYS STATUS (expect 5C / 42)
    // 16 / 17 the MISO PAD sampled at the falling edges of bytes 3 / 4 of the last transfer, i.e.
    //         what the BL616 should read (a SYS STATUS poll: expect 5C / 42)
    // (row 11 is the core's)
    reg [7:0] d_csnf = 0, d_bytes = 0, d_tgt = 0, d_cmd = 0, d_dat = 0, d_rsp = 0, d_falls = 0, d_nstat = 0;
    reg [7:0] d_fcnt = 0;
    reg [3:0] d_hi = 15, d_lo = 15, r_hi = 15, r_lo = 15, run = 0;
    reg [2:0] d_bitc = 0;
    reg       d_sck_seen = 1'b0;  reg [21:0] d_seen_t = 0;
    reg       csn_q = 1'b1;
    always @(posedge clk) begin
        csn_q <= csn_s[1];
        if (csn_q && !csn_s[1]) begin d_csnf <= d_csnf + 8'd1; d_fcnt <= 8'd0; r_hi <= 4'd15; r_lo <= 4'd15; end
        if (!csn_q && csn_s[1]) begin d_falls <= d_fcnt; d_hi <= r_hi; d_lo <= r_lo; d_bitc <= k_bc; end
        if (!csn_s[1]) begin
            if (sck_s[2] != sck_s[1]) begin           // an SCK edge: close the run of the old level
                if (sck_s[2] && run < r_hi) r_hi <= run;
                if (!sck_s[2] && run < r_lo) r_lo <= run;
                run <= 4'd1;
                if (sck_s[2]) d_fcnt <= d_fcnt + 8'd1;
            end else if (run != 4'd15) run <= run + 4'd1;
        end else run <= 4'd0;
        if (sck_s[2] != sck_s[1]) begin d_sck_seen <= 1'b1; d_seen_t <= 22'd0; end
        else if (d_seen_t == 22'h3FFFFF) d_sck_seen <= 1'b0; else d_seen_t <= d_seen_t + 22'd1;
        if (rx_stb) begin
            d_bytes <= d_bytes + 8'd1;
            if (rx_idx == 16'd0) d_tgt <= rx_byte;
            if (rx_idx == 16'd1) d_cmd <= rx_byte;
            if (rx_idx >= 16'd2) d_dat <= rx_byte;
            if (rx_idx == 16'd6 && tgt == T_SYS && cmd == 8'd0) d_nstat <= d_nstat + 8'd1;
        end
        if (rep_upd && ahead_n >= 16'd3) d_rsp <= rsp;
    end
    // pad readback, in the SCK domain: the MISO pad at each falling edge (where a mode-1 master
    // nominally samples) vs the bit the core launched; bytes 3/4 as the pad showed them
    reg [7:0] d_mis = 0, d_ones = 0, d_r3 = 0, d_r4 = 0, d_p3 = 0, d_p4 = 0;  reg [6:0] psh = 0;
    always @(negedge spi_sck) if (!spi_csn) begin
        if (spi_miso_pad != k_miso) d_mis <= d_mis + 8'd1;
        if (spi_miso_pad) d_ones <= d_ones + 8'd1;
        psh <= {psh[5:0], spi_miso_pad};
        if (k_bc == 3'd7 && k_idx == 16'd3) d_p3 <= {psh, spi_miso_pad};
        if (k_bc == 3'd7 && k_idx == 16'd4) d_p4 <= {psh, spi_miso_pad};
    end
    always @(posedge clk)
        if (rep_upd && rq[0]) begin
            if (a_i == 4'd3) d_r3 <= rsp;
            if (a_i == 4'd4) d_r4 <= rsp;
        end
    assign dbg_view = {d_csnf, d_bytes, d_tgt, d_cmd, d_dat, d_rsp, d_falls, {d_hi, d_lo}, d_nstat,
                       {hold, 1'b0, csn_s[1], sck_s[1], mosi_s[1], irq_n, d_sck_seen, act},
                       {d_bitc, coldboot, sys_int, hid_irq, hid_irq_en, stage_busy},
                       d_mis, d_ones, d_r3, d_r4, d_p3, d_p4};

`ifdef TANGO_LINK_TAP
    // ---------------------------------------------------------------- TANGO_LINK_TAP (header)
    always @(posedge clk) begin
        tap_cmd_stb <= 1'b0; tap_stb <= 1'b0;
        tap_end <= (end_cnt == 4'd8);
        if (lrst) vp_rep <= 8'd0;
        else if (rx_stb && i1) begin
            vp_rep <= (tgt != T_SYS) ? 8'd0 :
`ifdef TANGO_VSYNC_ADJUST
                      (rx_byte == 8'h70) ? vs_status :                    // TANGO_VSYNC_ADJUST (tango_vsync.v)
`endif
                      (rx_byte == 8'h2B && vp_has[0]) ? vp_flags :
                      (rx_byte == 8'h39 && vp_has[2]) ? 8'h03 :           // audio filter + boost (sys_top 396)
                      ((rx_byte == 8'h32 && vp_has[1]) || (rx_byte == 8'h3E && vp_has[3])) ? 8'h01 : 8'h00;
            tap_cmd <= rx_byte; tap_cmd_stb <= (tgt == T_SYS);
        end else if (rx_stb && !i0 && tgt == T_SYS) begin
            tap_stb <= 1'b1; tap_n <= n_q; tap_byte <= rx_byte;
        end
    end
`endif

    // ---------------------------------------------------------------- byte processing
    // One clock after the byte completes.  n = data byte number (byte index - 2).
    // registered with rx_idx (n_q), not subtracted here: the 16-bit subtract sat in front of the whole
    // command decode, and the A2600 build missed clk_pix by 41 ps there (rx_idx -> rw).  The decisions
    // are pre-decoded flops too ("pre-decode" above); n itself is only data here (addresses, counts).
    wire [15:0] n = n_q;
    reg  [2:0]  ecnt = 3'd0;                    // byte within the current entry
    reg  [ATW-9:0] aw = 0;                      // atlas word bytes so far
    reg  [9:0]  acol = 10'd0;
    // ATLAS column size: GEOMETRY byte 11 = glyph rows -> ceil(rows / 4) bytes per column
    // (the one GEOMETRY field that also governs ATLAS), taken
    // at once (the frame-latched copy in the overlay is irrelevant for ATLAS), clamped to
    // 1..BPC.  Default = the atlas height of this build.  Shorter columns are stored
    // zero-extended: coverage of glyph row gy stays at bits 2gy+1:2gy.
    reg  [2:0]  bpc_m1 = BPC - 1;
    wire [8:0]  grows_q = {1'b0, rx_byte} + 9'd3;   // rows + 3, then / 4
    reg         gbank = 1'b0;  reg [9:0] gidx = 10'd0;  reg [23:0] gw = 24'd0;
    reg         rbank = 1'b0;  reg [4:0] rrow = 5'd0;  reg [23:0] rw = 24'd0;
    reg  [7:0]  pidx = 8'd0, pcnt = 8'd0;  reg [15:0] pw = 16'd0;
    // state flags (see "pre-decode" above): one per (target, command) the byte processing handles, ...
    reg f_sys5 = 1'b0, f_sys6 = 1'b0, f_sys9 = 1'b0, f_sys10 = 1'b0;
    reg f_hid_port = 1'b0, f_hid_kbd = 1'b0, f_hid_pm = 1'b0;
    reg f_hid_dsel = 1'b0, f_hid_sd = 1'b0, f_hid_pc = 1'b0, f_hid_dsc = 1'b0;
    reg f_sdc_ins = 1'b0, f_sdc_insl = 1'b0, f_sdc_sec = 1'b0, f_sdc_secin = 1'b0, f_sdc_secout = 1'b0, f_sdc_img = 1'b0;
    reg f_osd_en = 1'b0, f_osd_swap = 1'b0, f_osd_geom = 1'b0, f_osd_pal = 1'b0, f_osd_atlas = 1'b0;
    reg f_osd_gl = 1'b0, f_osd_rows = 1'b0;
    // ... and the per-command state the writes are conditioned on
    reg sub1 = 1'b0, sub2 = 1'b0, dl_size_nz = 1'b0, pm_k1 = 1'b0, e_at = 1'b0;
`ifdef TANGO_IOCTL_UPLOAD
    reg sub3 = 1'b0, sub4 = 1'b0, sub5 = 1'b0, ul_size_nz = 1'b0;   // READ (3), UL_SELECT (4), UL_STATUS (5)
`endif
    reg [63:0] pm_sel = 64'd0;                  // SET_PADMAP: one-hot 6-bit slot {pm_j[4:0], pm_half}, none if pm_j[5]
    integer pk;
    always @(posedge clk) begin
        f_sys5  <= (tgt == T_SYS) && (cmd == 8'd5);
        f_sys6  <= (tgt == T_SYS) && (cmd == 8'd6);
        f_sys9  <= (tgt == T_SYS) && (cmd == 8'd9);
        f_sys10 <= (tgt == T_SYS) && (cmd == 8'h10);
        f_hid_port <= (tgt == T_HID) && (cmd == 8'h12 || cmd == 8'h15 || cmd == 8'h17 || cmd == 8'h11 || cmd == 8'h19);
        f_hid_dsel <= (tgt == T_HID) && (cmd == 8'h16);
        f_hid_sd   <= (tgt == T_HID) && (cmd == 8'h18);
        f_hid_pc   <= (tgt == T_HID) && (cmd == 8'h1A);
        f_hid_dsc  <= (tgt == T_HID) && (cmd == 8'h11);
        f_hid_kbd  <= (tgt == T_HID) && (cmd == 8'h01);
        f_hid_pm   <= (tgt == T_HID) && (cmd == 8'h13);
        f_sdc_ins    <= (tgt == T_SDC) && (cmd == 8'd4 || cmd == 8'd7);
        f_sdc_insl   <= (cmd == 8'd7);
        f_sdc_sec    <= (tgt == T_SDC) && (cmd == 8'd9 || cmd == 8'd10);
        f_sdc_secin  <= (cmd == 8'd9);
        f_sdc_secout <= (cmd == 8'd10);
        f_sdc_img    <= (tgt == T_SDC) && (cmd == 8'd8);
        f_osd_en    <= (tgt == T_OSD) && (cmd == O_ENABLE);
        f_osd_swap  <= (tgt == T_OSD) && (cmd == O_SWAP);
        f_osd_geom  <= (tgt == T_OSD) && (cmd == O_GEOM);
        f_osd_pal   <= (tgt == T_OSD) && (cmd == O_PAL);
        f_osd_atlas <= (tgt == T_OSD) && (cmd == O_ATLAS);
        f_osd_gl    <= (tgt == T_OSD) && (cmd == O_GLYPHS);
        f_osd_rows  <= (tgt == T_OSD) && (cmd == O_ROWS);
        sub1 <= (sdc_sub == 8'd1); sub2 <= (sdc_sub == 8'd2);
        dl_size_nz <= (dl_size != 24'd0);
`ifdef TANGO_IOCTL_UPLOAD
        // sub5 is NOT lagged like sub3/sub4 here: see the n0 branch below, same reasoning as sub0's own.
        sub3 <= (sdc_sub == 8'd3); sub4 <= (sdc_sub == 8'd4);
        ul_size_nz <= (ul_size != 24'd0);
`endif
        pm_k1 <= (pm_kind == 8'h01);
        e_at  <= (ecnt == bpc_m1);
        for (pk = 0; pk < 64; pk = pk + 1) pm_sel[pk] <= !pm_j[5] && ({pm_j[4:0], pm_half} == pk);
    end
    always @(posedge clk) begin
        at_we <= 1'b0; gl_we <= 1'b0; rt_we <= 1'b0; pal_we <= 1'b0; geo_we <= 1'b0;
        en_we <= 1'b0; swap_we <= 1'b0;
        if (csn_q1 && !csn_s[1]) seen_cmd <= 1'b0;
`ifdef TANGO_IOCTL_UPLOAD
        if (csn_q1 && !csn_s[1]) ul_got <= 24'd0;
`endif
        if (rx_stb && rx_idx >= 16'd1) seen_cmd <= 1'b1;
        stage_busy <= act && seen_cmd && (tgt == T_OSD) && (cmd == O_GEOM || cmd == O_PAL);
        joy_q <= joy;
        raw1_q <= raw1; raw2_q <= raw2; con_q <= pad_con; kbd1_q <= kbd1; kbd2_q <= kbd2;
        // HID interrupt: an armed change of the joystick state
        if (hid_irq_en && (joy != joy_q || raw1 != raw1_q || raw2 != raw2_q || pad_con != con_q ||
                           kbd1 != kbd1_q || kbd2 != kbd2_q)) begin
            hid_irq <= 1'b1; hid_irq_en <= 1'b0;
        end
        // ext events (PAD_CTRL bit0): a port going live / losing its stream, or its descriptors becoming ready, is an event too -- a pad
        // that the legacy decode never arms (its idle report reads as pressed) would otherwise never be seen
        live_q <= pad_live; rdy_q <= rdy_c;
        if (ext_en) begin
            if (pad_live != live_q || rdy_c != rdy_q) begin
                if (hid_irq_en) begin hid_irq <= 1'b1; hid_irq_en <= 1'b0; end
                rp_chg <= rp_chg | ((pad_live ^ live_q) | (rdy_c ^ rdy_q));
            end
        end
        // REPORT: each port's raw word or connect flag changed since its last REPORT
        if (rep_upd && a_i == 4'd4 && rq[8]) begin
            rp_snap <= rp_port ? {vid2, pid2, raw2} : {vid1, pid1, raw1};
            rp_chg[rp_port] <= 1'b0;
            hid_irq_en <= 1'b1;
        end
        if (rep_upd && a_i == 4'd4 && rq[7]) begin
            kb_snap <= kb_live; rp_chg[rp_port] <= 1'b0; hid_irq_en <= 1'b1;
        end
        // REPORT2: the whole reply from b5 on is taken in this one clock, then shifted out one byte per reply byte
        if (rep_upd && a_i == 4'd4 && rq[14]) begin
            pi_sr <= {rp_vid_live[7:0], rp_pid_l, rp_ver, rp_raw_l, rp_len_l, rep_be, rp_b31};
            rp_chg[rp_port] <= 1'b0; hid_irq_en <= 1'b1;
        end else if (rep_upd && rq[14] && a_pi) pi_sr <= {pi_sr[207:0], 8'd0};
        if (raw1 != raw1_q || pad_con[0] != con_q[0] || kbd1 != kbd1_q) rp_chg[0] <= 1'b1;
        if (raw2 != raw2_q || pad_con[1] != con_q[1] || kbd2 != kbd2_q) rp_chg[1] <= 1'b1;
        // HID KEYBOARD: when the transfer is over, the collected set-2 bytes become ps2_key (hps_io.sv 302-310)
        if (end_cnt == 4'd8 && ps2_pend) begin
            ps2_pend <= 1'b0;
            if (ps2_have) begin
                ps2_key <= {~ps2_key[10], ps2_pressed, ps2_extended, ps2_raw[7:0]};
                if (ps2_raw == 32'hE012E07C) ps2_key[9:0] <= 10'h37C;   // PrtScr pressed
                if (ps2_raw == 32'h7CE0F012) ps2_key[9:0] <= 10'h17C;   // PrtScr released
                if (ps2_raw == 32'hF014F077) ps2_key[9:0] <= 10'h377;   // Pause pressed
            end
        end
        // SET_PADMAP: the staged map reaches tango_ds4_joy when the transfer is over (as SET_STATUS)
        if (end_cnt == 4'd8 && pm_dirty) begin
            pm_dirty <= 1'b0;
            if (pm_kind == 8'hFE) begin if (pm_port) padmap2 <= DEFAULT_PADMAP; else padmap1 <= DEFAULT_PADMAP; end
            else if (pm_kind == 8'h01) begin if (pm_port) padmap2 <= pm_shadow; else padmap1 <= pm_shadow; end
        end
        // ---- generic pads: SET_DECODE apply / release, the descriptor read follower and cursor, device loss
        tw_we1 <= 1'b0; tw_we2 <= 1'b0;
        cm_ok <= cm_ok & pad_live;                                  // a table never outlives its device (the stream is gone for ~1.5 s after an unplug)
        if (end_cnt == 4'd8 && sd_dirty) begin                      // the transfer is over: whole table or nothing
            sd_dirty <= 1'b0; sd_rel <= 1'b1;
            if (!sd_bad && sd_hs == 5'd0 && sd_k == 3'd0 && sd_e == sd_count) begin
                cm_mode[sd_port] <= sd_mode; cm_rid_en[sd_port] <= sd_rid_en; cm_rid_val[sd_port] <= sd_rid_val;
                cm_min_len[sd_port] <= sd_min_len; cm_count[sd_port] <= sd_count;
                cm_ok[sd_port] <= pad_live[sd_port];
                sd_xfers <= sd_xfers + 8'd1;
            end else begin cm_mode[sd_port] <= 2'd0; cm_ok[sd_port] <= 1'b0; end
        end
        if (end_cnt == 4'd10 && sd_rel) begin cm_hold[sd_port] <= 1'b0; sd_rel <= 1'b0; end   // >= 2 clocks after the header changed
        if (end_cnt == 4'd8 && dsc_cnt) begin ds_off <= ds_off + ds_got; dsc_cnt <= 1'b0; end
        begin : ds_follow                                           // DESCRIPTOR data: the store address follows ahead_n (data byte n = 7 reads ahead_n = 6)
            reg [10:0] dsa;
            dsa = ds_off + ((ahead_n >= 16'd6) ? (ahead_n[10:0] - 11'd6) : 11'd0);
            ds_ra <= (dsel == 2'd0) ? {3'b000, dsa[7:0]} : (dsel == 2'd1) ? {3'b001, dsa[7:0]} : {1'b1, dsa[9:0]};
        end
        if (csn_q1 && !csn_s[1]) begin ds_got <= 11'd0; dsc_cnt <= 1'b0; end
        // Reads are snapshotted in the clock their byte-3 reply (rep_ahead for n = 3) is computed,
        // right after the command byte is processed; the SCK side sends exactly that value:
        //  - GET_DB9: joy/flags snapshot AND re-arm in that same clock, so a change one clock
        //    later is not lost between the snapshot and the arm;
        //  - IRQ_CTRL: remember which pending bits were reported; the ack clears only those,
        //    so an interrupt raised after the snapshot stays pending (IRQ stays low).
        if (rep_upd && a_i == 4'd3) begin
            if (rq[6]) begin joy_snap <= joy; flags_snap <= joy_flags; hid_irq_en <= 1'b1; end
            if (rq[1]) pend_rep <= pending[1:0];
        end
        // IMAGE STATUS: the space offered is snapshotted with byte 4, so bytes 5-6 agree
        if (rep_upd && a_i == 4'd4 && rq[13] && sub0) dl_free_snap <= dl_offer;
        // SET_STATUS: the staged word reaches the core when the transfer is over (8 clocks after CSN
        // is seen high, by when the last byte has come through the toggle synchroniser)
        if (act) end_cnt <= 4'd0; else if (end_cnt != 4'd15) end_cnt <= end_cnt + 4'd1;
        if (end_cnt == 4'd8 && st_dirty) begin status <= st_stage; status_tog <= ~status_tog; st_dirty <= 1'b0; end
`ifdef TANGO_IOCTL_UPLOAD
        // READ transfer over: advance the session's read cursor by the REAL byte count this transfer
        // actually clocked in (ul_got), never by a value derived from ahead_n's own 2-bytes-ahead lookahead
        // (which would over-count by up to 2 at a short/early-ending transfer -- the exact hazard this
        // module's own "reply prepared two bytes ahead" rule creates for anything timed off ahead_n).
        if (end_cnt == 4'd8 && sub3 && (ul_size_o != 24'd0)) ul_off <= ul_off + ul_got;
`endif
        // block device: prefetch the CORE_SECTOR_OUT byte one ahead (as the CONF_STR read), finish blocks and
        // mounts when the transfer is over
        sd_out_ra <= (ahead_n >= 16'd2) ? ahead_n[8:0] - 9'd2 : 9'd0;
`ifdef TANGO_IOCTL_UPLOAD
        // READ (sub 3): the same "continuous follower of ahead_n" idiom as sd_out_ra just above, offset by
        // one more than CORE_SECTOR_OUT's own "-2" (its first data byte is byte 3; READ's is byte 4, one
        // later, because of the idx byte at byte 3) -- i.e. offset = first-data-byte-position - 1, the same
        // relationship CORE_SECTOR_OUT's own "2 = 3 - 1" already uses. The GATING threshold must be one less
        // than the first data byte's own ahead_n (sd_out_ra's is "ahead_n >= 2" for a byte-3 target, the
        // same "-1" relationship): ul_ra/ul_rd are a 2-stage pipeline behind ahead_n, so whatever rsp
        // actually consumes for byte N was settled from ahead_n = N-1 (with ~44 clk_pix cycles to settle,
        // every byte after the first) -- byte 4's own reply needs ul_ra to have ALREADY computed the right
        // address while ahead_n was still 3 (the idx byte), or it falls back on the "ahead_n < threshold"
        // branch's stale value. Gating at >=4 (the naive, "first data byte" choice) would leave exactly that
        // one byte per transfer reading address ul_off+0 regardless of ul_off -- invisible when ul_off is 0
        // (it coincidentally reads the right byte), wrong for local byte 0 of every later chunk of a split
        // READ; hence >=3. ul_off is the session's running cursor (its own comment above): added so a READ
        // transfer that isn't the first one of its session continues from where the last one left off.
        ul_ra <= (ahead_n >= 16'd3) ? (ul_off[10:0] + ahead_n[10:0] - 11'd3) : 11'd0;
`endif
        sd_in_we <= 1'b0;
        if (rep_upd && rq[12] && sd_ok && a_sdo)
            sd_cnt <= ahead_n[9:0] - 10'd2;         // reads answered so far
        if (end_cnt == 4'd8) begin
            if (sd_ok && sd_cnt == 10'd512) begin
                if (sd_req[36]) sd_out_done_tog <= ~sd_out_done_tog; else sd_in_done_tog <= ~sd_in_done_tog;
                sd_served <= sd_req[38];
            end
            sd_ok <= 1'b0; sd_cnt <= 10'd0;
            if (mnt_dirty) begin mnt_tog <= ~mnt_tog; mnt_dirty <= 1'b0; end
        end
        dl_we <= 1'b0;
        if (rst) begin bpc_m1 <= BPC - 1; dl_open <= 1'b0; end
        if (lrst) begin
            tgt <= 8'd0; cmd <= 8'd0; rq <= rq_of(8'd0, 8'd0);
            sys_int <= 1'b1; coldboot <= 1'b1; hid_irq <= 1'b0; hid_irq_en <= 1'b0;
            stage_busy <= 1'b0;
            padmap1 <= DEFAULT_PADMAP; padmap2 <= DEFAULT_PADMAP; pm_dirty <= 1'b0; rp_chg <= 2'b00;
            cm_hold <= 2'b00; cm_ok <= 2'b00; cm_mode[0] <= 2'd0; cm_mode[1] <= 2'd0; sd_dirty <= 1'b0; sd_rel <= 1'b0;
            ext_en <= 1'b0; pad_swap <= 1'b0; dsel <= 2'd0; ds_off <= 11'd0; dsc_cnt <= 1'b0; sd_xfers <= 8'd0;
        end else if (rx_stb) begin
            if (f_hid_dsc && (n5 || dsc_cnt)) begin dsc_cnt <= 1'b1; ds_got <= ds_got + 11'd1; end   // DESCRIPTOR: data bytes clocked (from byte 7)
            if (i0) begin tgt <= rx_byte; rq <= rq_of(rx_byte, cmd); end
            else if (i1) begin cmd <= rx_byte; rq <= rq_of(tgt, rx_byte); ecnt <= 3'd0; end
            // rx_idx >= 2, by command (the flags are exclusive; same order as the case it replaces)
            else if (f_sys5) begin                              // IRQ_CTRL: ack what was reported
                if (n0) begin
                    if (rx_byte[0] && pend_rep[0]) sys_int <= 1'b0;
                    if (rx_byte[1] && pend_rep[1]) hid_irq <= 1'b0;
                end
            end
            else if (f_sys6) begin if (n0) coldboot <= 1'b0; end               // IRQ_SRC read acks cold boot
            else if (f_sys9) begin if (n0) dbg_jtagsel <= rx_byte[0]; end
            else if (f_sys10) begin                             // SET_STATUS: first byte index, then bytes
                if (n0) st_first <= rx_byte;
                else if (st_ok) begin
                    for (pk = 0; pk < 16; pk = pk + 1) if (st_sel[pk]) st_stage[8 * pk +: 8] <= rx_byte;
                    st_dirty <= 1'b1;
                end
            end
            // HID (GET_DB9 re-armed above)
            else if (f_hid_port) begin if (n0) rp_port <= rx_byte[0]; end      // REPORT / KBD_REPORT: port
            else if (f_hid_kbd) begin                           // KEYBOARD (Companion)
                if (n0) begin ps2_raw <= 32'd0; ps2_have <= 1'b0; ps2_pend <= 1'b1; end
                else if (!n1) begin ps2_raw <= {ps2_raw[23:0], rx_byte}; ps2_have <= 1'b1; end
            end
            else if (f_hid_pm) begin                            // SET_PADMAP
                if (n0) begin
                    pm_port <= rx_byte[0]; pm_shadow <= rx_byte[0] ? padmap2 : padmap1;
                end else if (n1) begin
                    pm_kind <= rx_byte;
                    if (rx_byte == 8'hFE) pm_dirty <= 1'b1;
                end else if (n2) begin
                    pm_j <= (rx_byte < 8'd32) ? rx_byte[5:0] : 6'd32; pm_half <= 1'b0;
                end else if (pm_k1 && !pm_j[5]) begin
                    // entry pm_j, half pm_half = bits 12 * pm_j + 6 * pm_half = slot {pm_j, pm_half} of 6 bits
                    for (pk = 0; pk < 64; pk = pk + 1)
                        if (pm_sel[pk]) pm_shadow[6 * pk +: 6] <= (rx_byte < 8'd32) ? {1'b0, rx_byte[4:0]} : 6'h20;
                    pm_half <= ~pm_half;
                    if (pm_half) pm_j <= pm_j + 6'd1;
                    pm_dirty <= 1'b1;
                end
            end
            else if (f_hid_dsel) begin                          // DESC_SELECT port sel
                if (n1) begin dsel <= (rx_byte > 8'd2) ? 2'd2 : rx_byte[1:0]; ds_off <= 11'd0; end
            end
            else if (f_hid_sd) begin                            // SET_DECODE port mode rid_en rid_val min_len count (count x 5)
                if (n0) begin
                    sd_port <= rx_byte[0]; cm_hold[rx_byte[0]] <= 1'b1; sd_dirty <= 1'b1; sd_bad <= 1'b0; sd_hs <= 5'b00001;
                    sd_e <= 7'd0; sd_k <= 3'd0; sd_count <= 7'd0; sd_mode <= 2'd0;
                end else if (sd_hs != 5'd0) begin               // the five header bytes
                    sd_hs <= {sd_hs[3:0], 1'b0};
                    if (sd_hs[0]) begin sd_mode <= rx_byte[1:0]; if (rx_byte > 8'd2) sd_bad <= 1'b1; end
                    if (sd_hs[1]) sd_rid_en <= rx_byte[0];
                    if (sd_hs[2]) sd_rid_val <= rx_byte;
                    if (sd_hs[3]) sd_min_len <= rx_byte;
                    if (sd_hs[4]) begin sd_count <= rx_byte[6:0]; if (rx_byte > 8'd64) sd_bad <= 1'b1; end
                end else begin                                  // entries, 5 bytes each (36 bits: the top 4 of the first byte)
                    sd_sh <= {sd_sh[31:0], rx_byte};
                    if (sd_k == 3'd4) begin
                        sd_k <= 3'd0;
                        if (sd_e < sd_count && !sd_bad) begin
                            if (sd_port) tw_we2 <= 1'b1; else tw_we1 <= 1'b1;
                            tw_addr <= sd_e[5:0]; tw_din <= {sd_sh[27:0], rx_byte};
                            sd_e <= sd_e + 7'd1;
                        end else sd_bad <= 1'b1;                 // more entries than announced
                    end else sd_k <= sd_k + 3'd1;
                end
            end
            else if (f_hid_pc) begin                            // PAD_CTRL bits
                if (n0) begin ext_en <= rx_byte[0]; pad_swap <= rx_byte[1]; end
            end
            // SDC
            else if (f_sdc_ins) begin                           // INSERTED / INS_LARGE (sdc.c 630-660)
                if (n0) begin mnt_drive <= rx_byte[3:0]; mnt_ro <= rx_byte[7]; mnt_size <= 64'd0; end
                else if (f_sdc_insl ? n_le8 : n_le4) begin
                    mnt_size <= {mnt_size[55:0], rx_byte};
                    if (f_sdc_insl ? n_eq8 : n_eq4) mnt_dirty <= 1'b1;
                end
            end
            else if (f_sdc_sec) begin                           // CORE_SECTOR_IN / OUT
                if (n0)
                    sd_ok <= sd_shown && rx_byte[3:0] == sd_drv && (sd_req[36] == f_sdc_secout);
                else if (f_sdc_secin && sd_ok && n_le512) begin
                    sd_in_we <= 1'b1; sd_in_wa <= n[8:0] - 9'd1; sd_in_wd <= rx_byte; sd_cnt <= n[9:0];
                end
            end
            else if (f_sdc_img) begin                           // IMAGE (sdc.c 435-519, 668-687)
                if (n0) begin
                    sdc_sub <= rx_byte; sub0 <= (rx_byte == 8'd0);
`ifdef TANGO_IOCTL_UPLOAD
                    // sub5, same clock as sub0 (NOT the lagged sdc_sub-follower idiom sub1-sub4 use): both
                    // feed r_sdc8's case directly, at a_i's own timing (one clock fresher than the lagged
                    // followers settle to) -- a lagged sub5 here raced sdc_sub for exactly the first reply
                    // byte of a transfer whose sub differs from the one before it (an upload bench saw byte
                    // 0 of the first-ever READ come back 0x91, ul_rd's correct 0x11 OR'd with a stray 0x80
                    // from r_sdc8 because sub5 still read the PRECEDING transfer's sub for that one clock).
                    // sub0 was never affected: it was already loaded here, at the same instant as sdc_sub.
                    sub5 <= (rx_byte == 8'd5);
`endif
                end
                else if (n1) dl_index <= rx_byte;
                else if (sub1) begin                            // SELECT: size, 4 bytes big-endian
                    if (n_le4) dl_size <= {dl_size[15:0], rx_byte};
                    if (n5) begin
                        if (dl_size_nz || rx_byte != 8'd0) begin
                            if (!dl_open) begin dl_we <= 1'b1; dl_wd <= {2'b01, dl_index}; dl_open <= 1'b1; end
                        end else if (dl_open) begin
                            dl_we <= 1'b1; dl_wd <= {2'b10, 8'd0}; dl_open <= 1'b0;
                        end
                    end
                end else if (sub2) begin                        // WRITE: data
                    if (dl_open && dl_free > 12'd1) begin dl_we <= 1'b1; dl_wd <= {2'b00, rx_byte}; end
                    else dl_dropped <= dl_dropped + 8'd1;
`ifdef TANGO_IOCTL_UPLOAD
                end else if (sub4) begin                        // UL_SELECT: size, 4 bytes big-endian (mirrors
                    // SELECT exactly, but posts to tango_io via ul_tog instead of the download FIFO: a size
                    // above 0 opens an upload (idx = dl_index, already valid by n5 as SELECT's own n5 use of
                    // it already relies on); size 0 closes it. Unlike dl_size, ul_size keeps the full 4-byte
                    // value (the last byte folded in here, not just used for the zero test) -- UL_STATUS
                    // reports it, and tango_io's fill engine needs the real byte count, not just a flag.
                    if (n_le4) ul_size <= {ul_size[15:0], rx_byte};
                    if (n5) begin
                        ul_idx <= dl_index;
                        // Clamped to UL_MAX (tango_io.v's UL_DEPTH -- MUST stay in sync; there is no shared
                        // header between the two files for it, same as DL_CAP vs. the download FIFO's own
                        // depth are two independent numbers today) here, at the source, so UL_STATUS reports
                        // the size that will actually be usable and a_ulr's own bound (above) never has to
                        // reason about tango_io's separate clamp: a session asking for more than UL_MAX
                        // bytes is accepted but silently truncated to UL_MAX, the same policy tango_io.v's
                        // fill engine already enforces on its own side (ul_size_l > UL_DEPTH).
                        ul_size_o <= (ul_size_nz || rx_byte != 8'd0)
                                     ? (({ul_size[15:0], rx_byte} > UL_MAX) ? UL_MAX : {ul_size[15:0], rx_byte})
                                     : 24'd0;
                        ul_tog <= ~ul_tog;
                        ul_off <= 24'd0;             // a fresh session's cursor always starts at 0
                    end
                end else if (sub3) begin             // READ: count real data bytes (idx was n1, above);
                    ul_got <= ul_got + 24'd1;        // committed into ul_off at end_cnt == 8, below
`endif
                end
            end
            // OSD (WRITE, CAPS, STATUS: nothing to store)
            else if (f_osd_en)   begin if (n0) begin en_we <= 1'b1; en_wd <= rx_byte[0]; end end
            else if (f_osd_swap) begin if (n0) begin swap_we <= 1'b1; swap_wd <= rx_byte[0]; end end
            else if (f_osd_geom) begin
                if (n_lt12) begin
                    geo_we <= 1'b1; geo_wa <= n[3:0]; geo_wd <= rx_byte;
                    if (n11)
                        bpc_m1 <= (rx_byte == 8'd0) ? 3'd0 :
                                  (grows_q[8:2] > BPC) ? BPC - 1 : grows_q[4:2] - 3'd1;
                end
            end
            else if (f_osd_pal) begin
                if (n0) pidx <= rx_byte;
                else if (n1) begin pcnt <= rx_byte; ecnt <= 3'd0; end
                else if (ecnt == 3'd2) begin
                    ecnt <= 3'd0;
                    if (pcnt != 8'd0) begin
                        pcnt <= pcnt - 8'd1; pidx <= pidx + 8'd1;
                        if (pidx <= 8'd16) begin pal_we <= 1'b1; pal_wa <= pidx[4:0]; pal_wd <= {pw, rx_byte}; end
                    end
                end else begin pw <= {pw[7:0], rx_byte}; ecnt <= ecnt + 3'd1; end
            end
            else if (f_osd_atlas) begin
                if (n0) acol[9:8] <= rx_byte[1:0];
                else if (n1) begin acol[7:0] <= rx_byte; ecnt <= 3'd0; end
                else if (e_at) begin                            // ecnt == bpc_m1
                    ecnt <= 3'd0;
                    at_we <= 1'b1; at_wa <= acol;
                    at_wd <= (ecnt == 3'd0) ? {{(ATW-8){1'b0}}, rx_byte} : {aw, rx_byte};
                    acol <= acol + 10'd1;
                end else begin
                    // first byte of a column clears the older bytes (zero-extension)
                    aw <= (ecnt == 3'd0) ? {{(ATW-16){1'b0}}, rx_byte} : {aw[ATW-17:0], rx_byte};
                    ecnt <= ecnt + 3'd1;
                end
            end
            else if (f_osd_gl) begin
                // GLYPHS: bank, first entry (2 bytes BE, tango_proto.c send_glyphs's tx16): the firmware
                // already sends a full 16-bit index, so widening to 10 bits (1024 slots/bank) only needs
                // RTL to read both low bits of the high byte instead of 1 (old firmware/cores that only
                // ever set bit 0 there, index < 512, are unaffected: bit 9 is simply 0).
                if (n0) gbank <= rx_byte[0];
                else if (n1) gidx[9:8] <= rx_byte[1:0];
                else if (n2) begin gidx[7:0] <= rx_byte; ecnt <= 3'd0; end
                else if (ecnt == 3'd3) begin
                    ecnt <= 3'd0;
                    gl_we <= 1'b1; gl_wa <= {gbank, gidx}; gl_wd <= {gw, rx_byte};
                    gidx <= gidx + 10'd1;
                end else begin gw <= {gw[15:0], rx_byte}; ecnt <= ecnt + 3'd1; end
            end
            else if (f_osd_rows) begin
                if (n0) rbank <= rx_byte[0];
                else if (n1) begin rrow <= rx_byte[4:0]; ecnt <= 3'd0; end
                else begin
                    ecnt <= ecnt + 3'd1;                         // wraps 7 -> 0
                    if (ecnt == 3'd3) begin                      // bytes 0-3 = entry[63:32]
                        rt_we <= 1'b1; rt_wa <= {rbank, rrow, 1'b1}; rt_wd <= {rw, rx_byte};
                    end else if (ecnt == 3'd7) begin             // bytes 4-7 = entry[31:0]
                        rt_we <= 1'b1; rt_wa <= {rbank, rrow, 1'b0}; rt_wd <= {rw, rx_byte};
                        rrow <= rrow + 5'd1;
                    end else rw <= {rw[15:0], rx_byte};
                end
            end
        end
    end
endmodule
`default_nettype wire

`ifdef TANGO_IOCTL_UPLOAD
// ul_lim for tango_link's a_ulr (see the comment at the instance): max(ul_size_o + 4 - ul_off, 0), as two
// registered stages around one subtractor.  Result lags the inputs by two clocks (ul_off: one).
module tango_ul_lim (
    input  wire        clk,
    input  wire [23:0] ul_size_o,
    input  wire [23:0] ul_off,
    output reg  [23:0] ul_lim = 24'd0
);
    reg  [23:0] ul_sz4 = 24'd4;
    wire [24:0] ul_sub = {1'b0, ul_sz4} - {1'b0, ul_off};   // bit 24 = borrow = (ul_off > ul_sz4)
    always @(posedge clk) begin
        ul_sz4 <= ul_size_o + 24'd4;
        ul_lim <= ul_sub[24] ? 24'd0 : ul_sub[23:0];
    end
endmodule
`endif
