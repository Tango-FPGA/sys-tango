// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// usb_fs_phy.v -- clean-room USB 2.0 full-speed (12Mbps) bit-level engine.
// Written directly from the USB 2.0 spec (NRZI encoding, bit stuffing,
// SYNC/EOP framing) -- no reference code. Our board has bare D+/D- GPIO
// (no PHY chip). Runs on 48MHz (4x oversample of 12Mbps); the receiver
// re-syncs bit phase on every line transition. FS idle/J=(dp=1,dn=0);
// K=(dp=0,dn=1); SE0=(dp=0,dn=0).
//
// PROVEN ON SILICON: enumerated a DualShock 4 and streamed its
// HID input reports (see usb_fs_host.v). Three bugs were found via the
// two-instance loopback sim (tb_phy_loopback.cpp) before ever flashing:
//  1) NRZI decode must compare line state at consecutive BIT CENTERS --
//     sampling the 1-cycle `transitioned` pulse at the center decodes
//     everything as 1s.
//  2) A byte presented during the SYNC field must not clobber the SYNC
//     shift register (FIELD_BYTE gate on the latch).
//  3) The bit registered at phase 3 drives the NEXT cell, so the final
//     data bit needs one full held cell (TXS_LASTBIT) before EOP's SE0.
//
// LOW SPEED (keyboards): `ls` = 1 runs the same engine at 1.5 Mbit/s: every sequential step of
// the receiver and transmitter advances only on a clock enable of 48 MHz / 8 (6 MHz = 4 x 1.5 Mbit/s, so the
// 4-phase bit cell, the squelch, SE0 and idle counters all scale by 8), and D+/D- swap roles (USB 2.0 7.1.5 /
// Table 7-2: a low-speed J is D- high).  tx_start / tx_keepalive are latched on any clock (tx_busy rises at
// once) and served on the next enable.  tx_keepalive sends a bare EOP (SE0 2 bit times, J 1): the low-speed
// keep-alive a host sends instead of SOF (USB 2.0 11.8.4.1).  With ls = 0 the enable is always 1, the pins are
// not swapped and no keep-alive is asked for: the full-speed path is the one proven on silicon.
`default_nettype none
module usb_fs_phy (
    input  wire clk48, rst_n,
    inout  wire dp, dn,
    input  wire bus_reset,      // drive continuous SE0 (host port reset)
    input  wire ls,             // 1 = low speed (1.5 Mbit/s, D+/D- swapped); hold stable while attached
    input  wire tx_keepalive,   // one clock: send a low-speed keep-alive (EOP only)

    input  wire       tx_start,
    input  wire [7:0] tx_data,
    input  wire       tx_valid,
    input  wire       tx_last,
    output reg        tx_rd,
    output reg        tx_busy,

    output reg        rx_active,
    output reg        rx_strobe,
    output reg [7:0]  rx_data,
    output reg        rx_eop,
    output reg        rx_bitstuff_err,
    output wire [2:0] dbg_tx_st, output wire dbg_tx_field, output wire [3:0] dbg_tx_bitcnt,
    output wire [1:0] dbg_tx_phase, output wire [7:0] dbg_tx_shift, output wire dbg_tx_cur_k,
    output wire [1:0] dbg_rx_phase, output wire [3:0] dbg_rx_bitcnt, output wire [2:0] dbg_rx_ones,
    output wire dbg_transitioned, output wire dbg_line_j, output wire dbg_line_k
);
    // ---------------------------------------------------------- line I/O
    // line_k: 1=drive K, 0=drive J, only meaningful while line_drive is
    // set; during EOP the tx_st-based override below takes precedence
    // (SE0 for 2 bit-times, then J for 1, per USB EOP framing).
    reg line_drive, line_k;
    wire se0_phase  = line_drive && ((tx_st == TXS_EOP1) || (tx_st == TXS_EOP2));
    wire eopj_phase = line_drive && (tx_st == TXS_EOPJ);
    // LOW SPEED: line_drive is released by TXS_IDLE on the next bit-cell enable, up to 8 clocks after the EOP's J bit; line_k still holds the last data bit
    // and, with ls = 1, would drive that stale K onto the bus for up to a quarter of a bit time right after every EOP (the low-speed bench's wire
    // monitor read it as a start of packet 1 bit after the EOP, and as both lines high against the device's own J).  With ls = 1 the data
    // level is therefore driven only while sending; with ls = 0 line_k is used as it always was (the full-speed path is unchanged: its glitch is one clock).
    wire k_now = line_k && (!ls || tx_st == TXS_SEND || tx_st == TXS_LASTBIT);
    wire dp_drive_val = se0_phase ? 1'b0 : (eopj_phase ? 1'b1 : (k_now ? 1'b0 : 1'b1));
    wire dn_drive_val = se0_phase ? 1'b0 : (eopj_phase ? 1'b0 : (k_now ? 1'b1 : 1'b0));
    // low speed: the J/K roles of D+ and D- swap
    assign dp = bus_reset ? 1'b0 : (line_drive ? (ls ? dn_drive_val : dp_drive_val) : 1'bZ);
    assign dn = bus_reset ? 1'b0 : (line_drive ? (ls ? dp_drive_val : dn_drive_val) : 1'bZ);

    reg dp_s0, dp_s1, dn_s0, dn_s1;
    always @(posedge clk48) begin
        dp_s0 <= ls ? dn : dp; dp_s1 <= dp_s0;       // "dp" / "dn" below = the full-speed roles
        dn_s0 <= ls ? dp : dn; dn_s1 <= dn_s0;
    end
    // bit-cell clock enable: every clock at full speed, one in 8 at low speed
    reg [2:0] ce_div = 3'd0;
    always @(posedge clk48) ce_div <= ce_div + 3'd1;
    wire ce = !ls || (ce_div == 3'd0);
    // LOW SPEED inter-packet gap (found by the low-speed bench's independent wire monitor): the host answered a device packet (the ACK of an
    // interrupt report or of a control data packet) one bit time after the device's EOP SE0 ended, i.e. as the device was still driving its EOP J bit
    // (both drivers on the wire; D+ and D- both high in the sim) -- the full-speed path is 2+ bit times only because its sequencing is 8x faster than
    // the bit rate, not by design.  At low speed a transmission now starts only when the line has been out of SE0 for at least 2 bit times
    // (8 enables), the bench's criterion for the full-speed host too ("host inter-packet delay >= 2 bit times", USB 2.0 7.1.18.1 as cited there;
    // the spec text is not in this tree).  With ls = 0 tx_gap_ok is 1: the full-speed path is unchanged.
    reg [3:0] j_since = 4'd15;
    wire cur_se0_i = ~dp_s1 & ~dn_s1;
    always @(posedge clk48) if (ce) j_since <= cur_se0_i ? 4'd0 : (j_since == 4'd15 ? 4'd15 : j_since + 4'd1);
    wire tx_gap_ok = !ls || (j_since >= 4'd8);
    wire cur_j   = dp_s1 & ~dn_s1;
    wire cur_k   = ~dp_s1 & dn_s1;
    wire cur_se0 = ~dp_s1 & ~dn_s1;

    // ============================================================ RECEIVE
    // NRZI decode: sample the LINE STATE (J vs K) at each bit-cell center
    // and compare against the state at the PREVIOUS bit center -- same
    // state = 1, changed = 0. The SYNC field (NRZI ...0000001, line
    // KJKJKJKK) is consumed here: data-byte assembly starts only after
    // SYNC's terminating 1-bit, and per spec that 1 also seeds the
    // bit-stuffing ones-counter.
    reg [1:0] rx_phase;
    reg [2:0] rx_ones;
    reg [3:0] rx_bitcnt;
    reg [7:0] rx_shift;
    reg [2:0] se0_cnt;
    reg       prev_j, prev_k;
    reg       rx_in_sync;         // consuming SYNC bits, no data yet
    reg       rx_last_center_k;   // line state at the previous bit center
    reg [6:0] rx_still;           // cycles without a transition while active
    reg [4:0] rx_squelch;         // post-TX mask: ignore release transients
    wire      transitioned = (cur_j != prev_j) | (cur_k != prev_k);

    always @(posedge clk48 or negedge rst_n) begin
        if (!rst_n) begin
            rx_phase <= 0; rx_active <= 0; rx_strobe <= 0;
            rx_ones <= 0; rx_bitcnt <= 0; prev_j <= 1; prev_k <= 0;
            rx_eop <= 0; se0_cnt <= 0; rx_bitstuff_err <= 0;
            rx_in_sync <= 0; rx_last_center_k <= 0; rx_squelch <= 0;
        end else begin
            rx_strobe <= 0; rx_eop <= 0;
            if (ce) begin
            prev_j <= cur_j; prev_k <= cur_k;
            se0_cnt <= cur_se0 ? (se0_cnt == 3'd7 ? se0_cnt : se0_cnt + 3'd1) : 3'd0;

            if (line_drive || bus_reset) begin
                // never decode our own transmission (single shared phy on
                // real hardware; a loopback sim's receiver instance has its
                // own line_drive=0 so this does not affect that test)
                rx_active <= 0;
                rx_squelch <= 5'd12;   // and stay masked ~3 bit-times after
                                       // release: the bus rings when our
                                       // driver lets go, and those glitches
                                       // decoded our OWN SOF tails as junk
                                       // packets on real silicon (a real
                                       // response can't arrive before the
                                       // 2-bit-time turnaround anyway)
            end else if (rx_squelch != 0) begin
                rx_squelch <= rx_squelch - 5'd1;
                rx_active <= 0;
            end else if (!rx_active) begin
                if (cur_k && prev_j) begin
                    rx_active <= 1; rx_phase <= 2'd1;
                    rx_ones <= 0; rx_bitcnt <= 0; rx_shift <= 0;
                    rx_bitstuff_err <= 0;
                    rx_in_sync <= 1;
                    rx_last_center_k <= 0;  // idle-J reference state
                    rx_still <= 0;
                end
            end else if (se0_cnt >= 3'd6) begin
                rx_active <= 0; rx_eop <= 1; rx_phase <= 0;
            end else if (rx_still >= 7'd80) begin
                // idle-abort: bit stuffing guarantees a transition at least
                // every 7 bit cells (28 cycles) inside a real packet, so
                // ~20 bit-times of stillness that is not SE0 means the
                // "packet" was a glitch (e.g. a device unplug mid-line).
                // Without this, rx_active sticks high forever and the host
                // waits on a response that never ends. (Hit on real
                // hardware after a power-outage mid-session.)
                rx_active <= 0; rx_phase <= 0;
            end else begin
                rx_still <= transitioned ? 7'd0 : (rx_still + 7'd1);
                if (transitioned) begin
                    // a transition re-syncs phase; it must NOT also count
                    // as this cycle's sample point
                    rx_phase <= 2'd1;
                end else begin
                    rx_phase <= rx_phase + 2'd1;
                    if (rx_phase == 2'd2) begin : sample_point
                        reg bit_val;
                        bit_val = (cur_k == rx_last_center_k);
                        rx_last_center_k <= cur_k;
                        if (rx_in_sync) begin
                            if (bit_val) begin
                                rx_in_sync <= 0;
                                rx_ones <= 3'd1;
                                rx_bitcnt <= 0;
                            end
                        end else if (rx_ones == 3'd6) begin
                            if (bit_val) rx_bitstuff_err <= 1;
                            rx_ones <= 0;   // stuffed 0 consumed, not data
                        end else begin
                            rx_ones <= bit_val ? (rx_ones + 3'd1) : 3'd0;
                            if (rx_bitcnt == 4'd7) begin
                                rx_data  <= {bit_val, rx_shift[7:1]};
                                rx_strobe <= 1;
                                rx_bitcnt <= 0;
                            end else begin
                                rx_shift  <= {bit_val, rx_shift[7:1]};
                                rx_bitcnt <= rx_bitcnt + 4'd1;
                            end
                        end
                    end
                end
            end
            end     // ce
        end
    end

    // ============================================================= SEND
    localparam TXS_IDLE=0, TXS_SEND=1, TXS_LASTBIT=5, TXS_EOP1=2, TXS_EOP2=3, TXS_EOPJ=4;
    localparam FIELD_SYNC=0, FIELD_BYTE=1;
    reg [2:0] tx_st;
    reg [1:0] tx_phase;
    reg       tx_field;
    reg [7:0] tx_shift;
    reg [3:0] tx_bitcnt;
    reg [2:0] tx_ones;
    reg       tx_cur_k;             // NRZI running line state, 1=K
    reg       tx_last_q;
    reg       tx_have_byte;
    reg       tx_start_p, tx_ka_p;      // latched requests (served on the next enable)

    wire tx_stuff_needed = (tx_field == FIELD_BYTE) && (tx_ones == 3'd6);
    wire tx_bit_now = tx_stuff_needed ? 1'b0 : tx_shift[0];

    always @(posedge clk48 or negedge rst_n) begin
        if (!rst_n) begin
            tx_st <= TXS_IDLE; tx_busy <= 0; tx_rd <= 0; line_drive <= 0;
            tx_phase <= 0; tx_cur_k <= 0; tx_ones <= 0; line_k <= 0;
            tx_start_p <= 0; tx_ka_p <= 0;
        end else begin
            tx_rd <= 0;
            if (tx_start)     begin tx_start_p <= 1; tx_busy <= 1; end
            if (tx_keepalive) begin tx_ka_p <= 1;    tx_busy <= 1; end
            if (ce) case (tx_st)
            TXS_IDLE: begin
                line_drive <= 0;
                if (tx_ka_p && tx_gap_ok) begin         // low-speed keep-alive: EOP only
                    tx_ka_p <= 0; tx_busy <= 1; line_drive <= 1;
                    tx_phase <= 0; tx_st <= TXS_EOP1;
                end else if ((tx_start_p || tx_start) && tx_gap_ok) begin
                    tx_start_p <= 0;
                    tx_busy <= 1; line_drive <= 1;
                    tx_cur_k <= 0;              // start from J (idle) reference
                    line_k <= 0;                // first cell drives J, not a
                                                // stale K from the prior packet
                    tx_field <= FIELD_SYNC;
                    tx_shift <= 8'h80;          // 00000001, LSB sent first
                    tx_bitcnt <= 0; tx_ones <= 0; tx_phase <= 0;
                    tx_st <= TXS_SEND;
                end else if (!tx_start_p && !tx_ka_p && !tx_start && !tx_keepalive) tx_busy <= 0;
            end
            TXS_SEND: begin
                if (tx_field == FIELD_BYTE && tx_bitcnt == 4'd0 && tx_phase == 2'd0 && !tx_have_byte && !tx_rd)
                    tx_rd <= 1;
                if (tx_field == FIELD_BYTE && tx_valid && !tx_have_byte) begin
                    tx_shift <= tx_data; tx_last_q <= tx_last; tx_have_byte <= 1;
                end

                if (tx_phase == 2'd3) begin
                    tx_phase <= 0;
                    if (!tx_bit_now) tx_cur_k <= ~tx_cur_k;
                    line_k <= !tx_bit_now ? ~tx_cur_k : tx_cur_k;

                    if (tx_stuff_needed) begin
                        tx_ones <= 0;   // stuffed bit consumed, shift reg untouched
                    end else begin
                        tx_shift <= {1'b0, tx_shift[7:1]};
                        tx_ones  <= tx_bit_now ? (tx_ones + 3'd1) : 3'd0;
                        if (tx_bitcnt == 4'd7) begin
                            tx_bitcnt <= 0;
                            if (tx_field == FIELD_SYNC) begin
                                tx_field <= FIELD_BYTE; tx_have_byte <= 0; tx_ones <= 0;
                            end else if (tx_last_q) begin
                                // the bit registered on THIS edge drives the
                                // NEXT cell -- hold one full cell so the
                                // final bit actually appears on the wire
                                // before SE0 (EOP) takes over
                                tx_st <= TXS_LASTBIT;
                            end else begin
                                tx_have_byte <= 0;
                            end
                        end else tx_bitcnt <= tx_bitcnt + 4'd1;
                    end
                end else tx_phase <= tx_phase + 2'd1;
            end
            TXS_LASTBIT: begin
                if (tx_phase == 2'd3) begin
                    tx_phase <= 0; tx_st <= TXS_EOP1;
                end else tx_phase <= tx_phase + 2'd1;
            end
            TXS_EOP1, TXS_EOP2: begin
                if (tx_phase == 2'd3) begin
                    tx_phase <= 0;
                    tx_st <= (tx_st == TXS_EOP1) ? TXS_EOP2 : TXS_EOPJ;
                end else tx_phase <= tx_phase + 2'd1;
            end
            TXS_EOPJ: begin
                if (tx_phase == 2'd3) begin
                    tx_phase <= 0; tx_st <= TXS_IDLE;
                    if (!tx_start_p && !tx_ka_p) tx_busy <= 0;
                end else tx_phase <= tx_phase + 2'd1;
            end
            endcase
        end
    end

    assign dbg_tx_st = tx_st; assign dbg_tx_field = tx_field; assign dbg_tx_bitcnt = tx_bitcnt;
    assign dbg_tx_phase = tx_phase; assign dbg_tx_shift = tx_shift; assign dbg_tx_cur_k = tx_cur_k;
    assign dbg_rx_phase = rx_phase; assign dbg_rx_bitcnt = rx_bitcnt; assign dbg_rx_ones = rx_ones;
    assign dbg_transitioned = transitioned; assign dbg_line_j = cur_j; assign dbg_line_k = cur_k;
endmodule
`default_nettype wire
