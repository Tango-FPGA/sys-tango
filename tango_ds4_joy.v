// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ds4_joy -- DualShock 4 report (clk48, from tango_usb_port) -> MiSTer joystick
// word in the destination clock (clk_pix), for the link's HID GET_DB9.
//
// CDC (clk48 -> clk), the silicon-proven pattern:
//   rpt_tog flips once per accepted report in the same clk48 clock that rpt is
//   written, and rpt is then untouched for >= 1 ms.  rpt_tog crosses through 3 flops;
//   the report bytes are captured ONLY on its edge (tog_s[2] != tog_s[1]), >= 2 clk
//   after the flip, so the multi-bit bus is sampled while stable.  Free-running
//   byte synchronisers are not used (they made phantom presses).
//   rpt_valid is a level: 2-flop synchroniser.  While it is low the captured bytes
//   are forced to "released", so a stale report can never be reused.
// ARM-AFTER-QUIET (as usb/tang_ds4_pad.sv 72-107): after rpt_valid rises, nothing is
//   admitted until the decode reads all-released for QUIET_CYC clocks.
//
// Decode (DS4 USB input report, report id 0x01; bit positions silicon-calibrated in
// usb/tang_ds4_pad.sv 37-56; host rpt byte k = report byte k):
//   byte 5 = {triangle, circle, cross, square, hat[3:0]}  hat 0 N .. 7 NW, 8 released
//   byte 6 = {R3, L3, options, share, R2, L2, R1, L1}
//   byte 7 = {counter[5:0], touchpad, PS}
// Outputs, all from the same captured bytes and the same arm-after-quiet gate, one register:
//   raw  [31:0]  the pad's buttons (README "Controllers"): 0 right, 1 left, 2 down, 3 up (hat),
//                4 cross, 5 circle, 6 square, 7 triangle, 8 L1, 9 R1, 10 L2, 11 R2, 12 share,
//                13 options, 14 L3, 15 R3, 16 PS, 17 touchpad, 31:18 = 0
//   joy  [15:0]  the FIXED default decode (the menu's map, MiSTer's def_mmap: GET_DB9, Start+Select),
//                MiSTer joystick order (tango_core_ports.vh; menu.c 1351-1368): 0 right, 1 left,
//                2 down, 3 up, 4 A = circle, 5 B = cross, 6 X = triangle, 7 Y = square, 8 L = L1,
//                9 R = R1, 10 Select = Share, 11 Start = Options; 15:12 = 0
//   core [31:0]  the core map (SET_PADMAP kind 1) applied to raw: bit j = raw[p_j] | raw[a_j];
//                map = 32 entries x {a[5:0], p[5:0]}, bit 5 = none.  Combinational from the captured
//                bytes into this one register: no added lag.
//   vid / pid    the device descriptor's (tango_usb_port), captured with the report bytes.
//   kind / kbd   what the device is (usb_fs_host dev_kind); a keyboard's boot report goes to kbd and its pad
//                outputs (raw, joy, core) stay 0.
// GENERIC PADS (the table-driven decode of tango_pad_decode.v; the link commands are in tango_link.v's header):
//   tmode = 1 (clk domain, from tango_link: a decode table is in force for this port) replaces the hardwired decode: the raw word is the
//           engine's (tango_pad_decode, clk48, crossed with its own toggle, 32 bits, held >= 1 us between words), the menu word and the core
//           map are computed from it, and the arm-after-quiet rule judges bits 17:0 only and gives up after ~1.8 s.  tmode = 0 is the code above,
//           unchanged: the DS4 path, the power-up default and every old firmware's behaviour.
//   rep16/rep_len/ver/live  the last report of 1 byte or more (REPORT2), bcdDevice, and "a report stream is live" (independent of arming).
//==============================================================================
`default_nettype none
module tango_ds4_joy #(
    parameter integer QUIET_CYC = 16
) (
    input  wire        clk,             // destination clock (clk_pix)
    input  wire        rst,
    input  wire [63:0] rpt,             // clk48 domain, stable between rpt_tog flips
    input  wire        rpt_tog,         // clk48 domain
    input  wire        rpt_valid,       // clk48 domain level
    input  wire [15:0] vid_in, pid_in,  // clk48 domain, stable while reports flow
    input  wire [1:0]  kind_in,         // clk48 (usb_fs_host dev_kind): 0 pad, 1 boot keyboard, 2 other
    input  wire [383:0] padmap,          // clk domain (tango_link SET_PADMAP): entry j at [12j +: 12]
    // ---- generic pads
    input  wire [127:0] rpt16_in,        // clk48: payload bytes 0..15 of the last report of 1 byte or more
    input  wire [7:0]   rpt_len_in,
    input  wire         rpt2_tog,
    input  wire         rpt2_valid,
    input  wire [15:0]  ver_in,          // bcdDevice
    input  wire [31:0]  traw_in,         // clk48: tango_pad_decode's raw word
    input  wire         traw_tog,
    input  wire         tmode,           // clk domain: a table (or quiet) is in force for this port
    output reg  [15:0] joy = 16'd0,
    output reg  [31:0] core = 32'd0,
    output reg  [31:0] raw = 32'd0,
    output reg  [15:0] vid = 16'd0,
    output reg  [15:0] pid = 16'd0,
    output reg  [1:0]  kind = 2'd0,
    output reg  [63:0] kbd = 64'd0,     // a keyboard's 8-byte boot report (byte 0 in [7:0]); 0 for a pad
    output reg         present = 1'b0,  // a report stream is live and armed
    output reg  [127:0] rep16 = 128'd0,  // REPORT2: the captured report (zero beyond rep_len)
    output reg  [7:0]  rep_len = 8'd0,
    output reg  [15:0] ver = 16'd0,
    output wire        live             // a CRC-good report stream is live (legacy or generic), armed or not
);
    reg [2:0] tog_s = 3'b000;
    reg [1:0] val_s = 2'b00;
    reg [7:0] b5 = 8'h08, b6 = 8'h00, b7 = 8'h00;   // captured report bytes (hat 8 = released)
    // generic pads: the same crossing for the second toggle/level pair (reports of 1 byte or more) and for the engine's word
    reg [2:0] tog2_s = 3'b000, ttog_s = 3'b000;
    reg [1:0] val2_s = 2'b00;
    wire edge1 = (tog_s[2] != tog_s[1]);
    wire edge2 = (tog2_s[2] != tog2_s[1]);
    wire live_any = val_s[1] | val2_s[1];
    assign live = live_any;
    always @(posedge clk) begin
        tog_s <= {tog_s[1:0], rpt_tog};
        val_s <= {val_s[0], rpt_valid};
        tog2_s <= {tog2_s[1:0], rpt2_tog};
        val2_s <= {val2_s[0], rpt2_valid};
        ttog_s <= {ttog_s[1:0], traw_tog};
        if (rst || !val_s[1]) begin b5 <= 8'h08; b6 <= 8'h00; b7 <= 8'h00; kbd <= 64'd0; end
        else if (edge1) begin
            if (kind_in == 2'd0) begin b5 <= rpt[47:40]; b6 <= rpt[55:48]; b7 <= rpt[63:56]; kbd <= 64'd0; end
            else begin b5 <= 8'h08; b6 <= 8'h00; b7 <= 8'h00; kbd <= (kind_in == 2'd1) ? rpt : 64'd0; end   // not a pad
        end
        // ids, kind, bcdDevice and the REPORT2 report: on either toggle (they flip in the same clock for reports of 6 bytes or more, so a
        // pad of the legacy path sees these at the instants it always did)
        if (rst || !live_any) begin vid <= 16'd0; pid <= 16'd0; kind <= 2'd0; ver <= 16'd0; rep16 <= 128'd0; rep_len <= 8'd0; end
        else if (edge1 || edge2) begin
            vid <= vid_in; pid <= pid_in; kind <= kind_in; ver <= ver_in;
            if (edge2) begin rep16 <= rpt16_in; rep_len <= rpt_len_in; end
        end
    end
    reg up, down, left, right;
    always @(*) begin
        up = 1'b0; down = 1'b0; left = 1'b0; right = 1'b0;
        case (b5[3:0])
            4'd0: up = 1'b1;
            4'd1: begin up = 1'b1; right = 1'b1; end
            4'd2: right = 1'b1;
            4'd3: begin down = 1'b1; right = 1'b1; end
            4'd4: down = 1'b1;
            4'd5: begin down = 1'b1; left = 1'b1; end
            4'd6: left = 1'b1;
            4'd7: begin up = 1'b1; left = 1'b1; end
            default: ;                                  // 8+ = released
        endcase
    end
    wire [15:0] dflt = {4'd0, b6[5], b6[4], b6[1], b6[0],      // Start, Select, R, L
                        b5[4], b5[7], b5[5], b5[6],            // Y=square, X=triangle, B=cross, A=circle
                        up, down, left, right};
    wire [31:0] rw_ds4 = {14'd0, b7[1], b7[0],                 // 17 touchpad, 16 PS
                      b6[7], b6[6], b6[5], b6[4],              // 15 R3, 14 L3, 13 options, 12 share
                      b6[3], b6[2], b6[1], b6[0],              // 11 R2, 10 L2, 9 R1, 8 L1
                      b5[7], b5[4], b5[6], b5[5],              // 7 triangle, 6 square, 5 circle, 4 cross
                      up, down, left, right};
    // table mode: the engine's word, captured on its own toggle; zero until the first word of this table
    reg [31:0] traw_q = 32'd0;
    reg        got_word = 1'b0;
    always @(posedge clk) begin
        if (rst || !val2_s[1] || !tmode) begin traw_q <= 32'd0; got_word <= 1'b0; end
        else if (ttog_s[2] != ttog_s[1]) begin traw_q <= traw_in; got_word <= 1'b1; end
    end
    // the fixed menu word from a raw word (the same bits tango_ds4_joy.v builds above from the report bytes)
    // The menu directions are the d-pad OR the left stick (raw bits 21 up, 20 down, 19 left, 18 right): a pad whose d-pad is on its X / Y axes while its descriptor
    // declares a hat that never moves (the DragonRise 0079:0006 family) has no d-pad bit before Define buttons, and without a direction the user could not reach
    // Define buttons at all.  The core word is not touched: that is the core map's, and Define builds it.
    wire [15:0] dflt_t = {4'd0, traw_q[13], traw_q[12], traw_q[9], traw_q[8],      // Start, Select, R, L
                          traw_q[6], traw_q[7], traw_q[4], traw_q[5],              // Y, X, B, A
                          traw_q[3] | traw_q[21], traw_q[2] | traw_q[20], traw_q[1] | traw_q[19], traw_q[0] | traw_q[18]};   // up, down, left, right
    wire [31:0] rw = tmode ? traw_q : rw_ds4;
    wire [15:0] dflt_s = tmode ? dflt_t : dflt;
    wire        valid_s = tmode ? val2_s[1] : val_s[1];
    reg [31:0] mapped;
    integer j;
    always @(*) begin
        for (j = 0; j < 32; j = j + 1)
            mapped[j] = (!padmap[12*j + 5] && rw[padmap[12*j +: 5]]) | (!padmap[12*j + 11] && rw[padmap[12*j + 6 +: 5]]);
    end
    reg [4:0] quiet = 5'd0;
    reg       armed = 1'b0;
    reg       tmode_q = 1'b0;
    reg [26:0] tmo = 27'd0;                            // table mode: arm after ~1.8 s regardless (a worn stick, a trigger resting at its end)
    always @(posedge clk) begin
        tmode_q <= tmode;
        if (rst || !valid_s || (tmode != tmode_q)) begin quiet <= 5'd0; armed <= 1'b0; tmo <= 27'd0; end
        else if (!armed) begin
            if (tmode) begin
                if (!got_word || traw_q[17:0] != 18'd0) quiet <= 5'd0;
                else if (quiet == QUIET_CYC[4:0]) armed <= 1'b1;
                else quiet <= quiet + 5'd1;
                if (got_word) begin tmo <= tmo + 27'd1; if (&tmo) armed <= 1'b1; end
            end else begin
                if (rw != 32'd0) quiet <= 5'd0;
                else if (quiet == QUIET_CYC[4:0]) armed <= 1'b1;
                else quiet <= quiet + 5'd1;
            end
        end
        joy     <= (valid_s && armed) ? dflt_s : 16'd0;
        raw     <= (valid_s && armed) ? rw : 32'd0;
        core    <= (valid_s && armed) ? mapped : 32'd0;
        present <= valid_s && armed;
    end
endmodule
`default_nettype wire
