// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tang_ds4_pad.sv -- map a DualShock 4 USB HID input report to the NES
// controller shift-register protocol, drop-in for tang_fake_pad.
//
// DS4 wired input report (report id 0x01), byte offsets into the report
// BODY (i.e. rpt[0]=report id 0x01, rpt[1]=LX ... ):
//   rpt[5] = {triangle, circle, cross, square, dpad_hat[3:0]}
//   rpt[6] = {R3, L3, options, share, R2, L2, R1, L1}
//   rpt[7][0] = PS button
// dpad_hat: 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW,8=released.
// Verified on silicon (X = cross = rpt[5][5]; dpad-left =
// hat 6, observed together as rpt[5]=0x83 hat? -- see mapping note).
//
// NES bit order out of the shift register: A,B,Select,Start,Up,Down,
// Left,Right (ACTIVE-HIGH on D0, per the proven tb_console model, same
// as tang_fake_pad).
//
// Button map (silicon-calibrated): O->A, X->B,
// Share->Select, Options->Start, dpad hat->D-pad.
`default_nettype none
module tang_ds4_pad (
    input  wire        clk, rst_n,        // NES clock domain
    input  wire        strobe,            // ctrl_out[0]
    input  wire        cyc_is_get,
    input  wire        oe1_n,
    // live HID report bytes (report BODY, rpt[0]=id); latched by the host
    input  wire [7:0]  rpt5, rpt6, rpt7,
    input  wire        rpt_valid,         // a real report has been seen
    output wire        d0,
    output wire [7:0]  btns               // parallel NES vector {R,L,D,U,Start,Sel,B,A}
);
    // ---- decode the DS4 buttons (all silicon-calibrated).
    // rpt5 = report byte with face buttons + hat (host's rxb[6]);
    // rpt6 = report byte with share/options/L*/R* (host's rxb[7]).
    // (Port names rpt5/rpt6 are the module's own; wire the host's rxb[6]
    //  to rpt5 and rxb[7] to rpt6.)
    wire btn_circle = rpt5[6];   // O  (0x08->0x48)
    wire btn_cross  = rpt5[5];   // X  (0x08->0x28)
    wire btn_share  = rpt6[4];   // Share   (rpt6 0x00->0x10)
    wire btn_options= rpt6[5];   // Options (rpt6 0x00->0x20)
    wire [3:0] hat = rpt5[3:0];
    // hat -> up/down/left/right (8 = centered/released)
    reg up, down, left, right;
    always @* begin
        up=0; down=0; left=0; right=0;
        case (hat)
            4'd0: up=1;
            4'd1: begin up=1; right=1; end
            4'd2: right=1;
            4'd3: begin down=1; right=1; end
            4'd4: down=1;
            4'd5: begin down=1; left=1; end
            4'd6: left=1;
            4'd7: begin up=1; left=1; end
            default: ; // 8+ = released
        endcase
    end

    // NES button vector, ACTIVE-HIGH. The shift register emits sh[0] FIRST
    // and the NES read order is A,B,Sel,Start,U,D,L,R, so A must sit in
    // bit 0 (LSB), R in bit 7 -- hence this order is the reverse of the
    // human A..R reading. (Matches tang_fake_pad, whose Start=bit3.)
    // The map: O->A, X->B, Share->Select, Options->Start.
    // Gated by rpt_valid so a not-yet-enumerated pad reads as all-released.
    wire [7:0] raw_buttons =
        {right, left, down, up, btn_options, btn_share, btn_cross, btn_circle};

    // ⭐ ARM-AFTER-QUIET. MEASURED: with the controller untouched,
    // this decoder fired **31 phantom press events and sticky=FF -- ALL EIGHT
    // buttons**. Gating on rpt_valid alone is not enough.
    //
    // Why: rpt_valid and the report bytes cross clk48 -> clk_mem through
    // SEPARATE 2-flop synchronizers, and before the first report desc_bytes
    // carries the DESCRIPTOR-path packing (usb_fs_host.v:281 -- a different
    // byte layout with rx_cnt in the low byte). At the instant rpt_valid
    // rises, rpt5/rpt6 can still hold enumeration bytes, which decode as
    // buttons. sticky=FF is the proof: a synchronizer race gives one or two
    // adjacent bits, never 0xFF. usb_fs_host.v:343 also clears desc_valid on
    // a USB error, so every re-arm repeats it mid-game -- which is why the
    // presses were rare but never stopped, and why SMB3 could reach its world
    // map without a Start press.
    //
    // The fix does not trust the valid flag's timing at all: after any rise
    // of rpt_valid, admit nothing until the decode has read ALL-RELEASED for
    // QUIET_CYC consecutive cycles. Stale enumeration bytes are never quiet,
    // so they can never be admitted; a genuinely idle pad passes in ~microseconds.
    localparam integer QUIET_CYC = 16;
    reg [4:0] quiet_cnt;
    reg       armed;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin quiet_cnt <= 5'd0; armed <= 1'b0; end
        else if (!rpt_valid) begin quiet_cnt <= 5'd0; armed <= 1'b0; end
        else if (!armed) begin
            if (raw_buttons != 8'h00) quiet_cnt <= 5'd0;
            else if (quiet_cnt == QUIET_CYC[4:0]) armed <= 1'b1;
            else quiet_cnt <= quiet_cnt + 5'd1;
        end
    end
    wire [7:0] buttons = (rpt_valid && armed) ? raw_buttons : 8'h00;

    // ---- NES shift-register protocol (identical to tang_fake_pad)
    reg [7:0] sh; reg oe_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sh <= 8'h00; oe_q <= 1'b1; end
        else begin
            oe_q <= oe1_n;
            if (strobe && !cyc_is_get) sh <= buttons;
            else if (oe1_n && !oe_q)   sh <= {1'b1, sh[7:1]};
        end
    end
    assign d0 = sh[0];
    assign btns = buttons;   // parallel tap for the soft-core menu (P2b)
endmodule
`default_nettype wire
