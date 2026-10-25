// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_pad_decode -- table-driven pad decode, in the USB host's clock (clk48), one instance per USB-A port
// (tango_sys.v; the tables come from the firmware's controller database over the link, tango_link.v SET_DECODE).
// Reference model: menu-tango bl616/tango/tango_paddec.c pd_eval(); sim/pads/pad_vectors.txt is generated from it
// and replayed by sim/pads/.
//
// WHAT IT DOES.  A table of up to 64 entries says "raw bit OUT is set when this FIELD of the report satisfies this TEST".
// On every accepted USB report (rpt_tog flips) it walks the table ONE ENTRY PER CLOCK and ORs the answers into a 32-bit raw
// word, which it publishes with a toggle.  Latency from the report to the word = count + 4 clk48 (about 0.5 to 1.4 us for
// real pads, measured in sim/pads/); the host's own 1 ms poll is 1000x longer.  The word then crosses to clk_pix exactly as
// the report bytes do today (rpt_tog + held data), but it is 32 bits instead of 64, and no decode logic sits in clk_pix.
//
// TABLE ENTRY (36 bits, the width of a GW5A BSRAM word):
//   [35:31] out      raw bit 0..31          [30:28] cmp  0 !=0  1 ==0  2 >=thr  3 <=thr  4 ==thr  5 HAT
//   [27:24] byte     first byte of a 3-byte window (bytes past 15 and past the report length read 0)
//   [23:21] shift    0..7                   [20:17] widthm1  field width - 1 (1..16 bits)
//   [16]    bias     field ^= 1 << widthm1 (two's complement -> offset binary)
//   [15:0]  thr      compare value; for HAT: [11:8] = value of North, [7:0] = membership mask over n = field - base
//   field = ((window >> shift) & mask(width)) ^ bias_bit
//
// HEADER (cfg_*, wclk domain, written by the link while cfg_hold is 1): mode 0 = this block is off (the built-in DS4 decode
// in tango_ds4_joy is used), 1 = table, 2 = quiet (raw word stays 0); rid_en/rid_val: accept only reports whose byte 0 equals
// rid_val; min_len: accept only payloads of at least this many bytes; count: entries in use.
//
// CDC.  Table memory: written in wclk, read in clk (a dual-clock simple RAM, as tango_sd.v's buffers).  cfg_hold crosses with
// 2 flops; the engine reads the header and the memory only while cfg_hold (synchronised) is 0, and the link raises cfg_hold
// before it writes the first word and lowers it >= 8 wclk after the last, so nothing is read while it changes (the SET_PADMAP
// idiom, tango_link.v).  When cfg_hold falls the held report is evaluated again at once: a pad that sends reports only on
// change would otherwise keep the old decode until its next press.
//
// The engine copies the 16 report bytes into its own registers when it starts (zero beyond the length), so the host may
// overwrite rpt at any time during a run.
//==============================================================================
`default_nettype none
module tango_pad_decode (
    input  wire         clk,            // clk48
    input  wire         rst_n,
    // the report, clk domain (usb_fs_host: bytes 0..15 of the last CRC-good payload, byte 0 in [7:0])
    input  wire [127:0] rpt,
    input  wire [7:0]   rpt_len,        // payload length in bytes, saturating at 255
    input  wire         rpt_tog,        // flips per accepted report
    input  wire         rpt_valid,      // level: a report stream is live
    // the table, written by the link (wclk domain)
    input  wire         wclk,
    input  wire         tw_we,
    input  wire [5:0]   tw_addr,
    input  wire [35:0]  tw_din,
    input  wire [1:0]   cfg_mode,
    input  wire         cfg_rid_en,
    input  wire [7:0]   cfg_rid_val,
    input  wire [7:0]   cfg_min_len,
    input  wire [6:0]   cfg_count,
    input  wire         cfg_hold,
    // result, clk domain
    output reg  [31:0]  raw = 32'd0,
    output reg          raw_tog = 1'b0, // flips per evaluated report
    output wire         raw_valid,      // level: this block owns the pad (mode 1 or 2), not holding, and the host stream is live
    output wire         busy
);
    // ---------------------------------------------------------------- table memory
    (* syn_ramstyle = "block_ram" *) reg [35:0] mem [0:63];      // one 36-bit-wide BSRAM; GW5A has no LUT RAM, so unhinted it could become 2304 flops
    always @(posedge wclk) if (tw_we) mem[tw_addr] <= tw_din;
    reg [5:0]  ra = 6'd0;
    reg [35:0] rd = 36'd0;
    always @(posedge clk) rd <= mem[ra];

    // ---------------------------------------------------------------- hold / mode
    reg [2:0] hold_s = 3'b111;
    always @(posedge clk) hold_s <= {hold_s[1:0], cfg_hold};
    wire hold   = hold_s[1];
    wire active = !hold && (cfg_mode != 2'd0);
    assign raw_valid = active && rpt_valid;

    // ---------------------------------------------------------------- start conditions
    reg tog_q = 1'b0;
    always @(posedge clk) tog_q <= rpt_tog;
    wire new_rpt   = (rpt_tog != tog_q);
    wire hold_fall = (hold_s[2] && !hold_s[1]);

    // ---------------------------------------------------------------- the run
    localparam [1:0] S_IDLE = 2'd0, S_ISSUE = 2'd1, S_DRAIN = 2'd2;
    reg [1:0]   st = S_IDLE;
    reg [6:0]   i = 7'd0;
    reg [127:0] rb = 128'd0;           // the report as the run sees it
    reg         pend = 1'b0;           // a start request that arrived while busy (hold fall): run again afterwards
    assign busy = (st != S_IDLE);

    // stage 0 -> 1 -> 2: address issued, memory word, window + entry
    reg         v0 = 1'b0, l0 = 1'b0;
    reg         v1 = 1'b0, l1 = 1'b0;
    reg         v2 = 1'b0, l2 = 1'b0;
    reg [23:0]  w_q = 24'd0;
    reg [35:0]  e_q = 36'd0;
    reg [31:0]  acc = 32'd0;

    // header copies used during a run (stable: the engine only runs while !hold)
    wire [6:0]  count = cfg_count;
    wire        rid_ok = !cfg_rid_en || (rpt[7:0] == cfg_rid_val);
    wire        len_ok = (rpt_len >= cfg_min_len);

    // stage A: the 3-byte window from the entry's byte field
    wire [3:0]  a_byte = rd[27:24];
    wire [7:0]  m0 = rb[{a_byte, 3'b000} +: 8];
    wire [7:0]  m1 = (a_byte == 4'd15) ? 8'h00 : rb[{(a_byte + 4'd1), 3'b000} +: 8];
    wire [7:0]  m2 = (a_byte >= 4'd14) ? 8'h00 : rb[{(a_byte + 4'd2), 3'b000} +: 8];

    // stage B: field and test
    wire [2:0]  b_shift = e_q[23:21];
    wire [3:0]  b_wm1   = e_q[20:17];
    wire        b_bias  = e_q[16];
    wire [15:0] b_thr   = e_q[15:0];
    wire [2:0]  b_cmp   = e_q[30:28];
    wire [4:0]  b_out   = e_q[35:31];
    wire [23:0] b_sh    = w_q >> b_shift;
    wire [16:0] b_mask  = (17'd1 << ({1'b0, b_wm1} + 5'd1)) - 17'd1;
    wire [15:0] b_f0    = b_sh[15:0] & b_mask[15:0];
    wire [15:0] b_f     = b_bias ? (b_f0 ^ (16'd1 << b_wm1)) : b_f0;
    wire [3:0]  b_base  = b_thr[11:8];
    wire [15:0] b_n     = b_f - {12'd0, b_base};
    reg         b_pred;
    always @(*) begin
        case (b_cmp)
            3'd0:    b_pred = (b_f != 16'd0);
            3'd1:    b_pred = (b_f == 16'd0);
            3'd2:    b_pred = (b_f >= b_thr);
            3'd3:    b_pred = (b_f <= b_thr);
            3'd4:    b_pred = (b_f == b_thr);
            3'd5:    b_pred = (b_f >= {12'd0, b_base}) && (b_n <= 16'd7) && b_thr[b_n[2:0]];
            default: b_pred = 1'b0;
        endcase
    end
    wire [31:0] b_acc = acc | ({31'd0, b_pred} << b_out);

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; i <= 7'd0; pend <= 1'b0; ra <= 6'd0;
            v0 <= 1'b0; l0 <= 1'b0; v1 <= 1'b0; l1 <= 1'b0; v2 <= 1'b0; l2 <= 1'b0;
            acc <= 32'd0; raw <= 32'd0; raw_tog <= 1'b0; rb <= 128'd0; w_q <= 24'd0; e_q <= 36'd0;
        end else begin
            // pipeline registers
            v1 <= v0; l1 <= l0;
            v2 <= v1; l2 <= l1;
            if (v1) begin
                w_q <= {m2, m1, m0};
                e_q <= rd;
            end
            if (v2) begin
                if (l2) begin
                    raw <= b_acc; raw_tog <= ~raw_tog; acc <= 32'd0;
                end else acc <= b_acc;
            end

            if (new_rpt || hold_fall) pend <= 1'b1;      // remembered: a report (or a table change) that lands while a run is going is not lost
            if (!rpt_valid) pend <= 1'b0;                // ...but nothing is replayed from a stream that is gone
            v0 <= 1'b0; l0 <= 1'b0;

            case (st)
            S_IDLE: begin
                if ((new_rpt || pend || hold_fall) && active && rpt_valid) begin
                    pend <= 1'b0;
                    if (cfg_mode == 2'd2) begin                 // quiet: the word stays 0, one toggle per report
                        raw <= 32'd0; raw_tog <= ~raw_tog;
                    end else if (rid_ok && len_ok) begin
                        for (k = 0; k < 16; k = k + 1) rb[8*k +: 8] <= (k < rpt_len) ? rpt[8*k +: 8] : 8'h00;
                        acc <= 32'd0;
                        if (count == 7'd0) begin                // nothing in the table: the word is 0
                            raw <= 32'd0; raw_tog <= ~raw_tog;
                        end else begin
                            i <= 7'd0; st <= S_ISSUE;
                        end
                    end
                end
            end
            S_ISSUE: begin
                ra <= i[5:0];
                v0 <= 1'b1; l0 <= (i == count - 7'd1);
                i <= i + 7'd1;
                if (i == count - 7'd1) st <= S_DRAIN;
            end
            S_DRAIN: begin
                // wait for the last entry to leave stage B
                if (l2 && v2) st <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
