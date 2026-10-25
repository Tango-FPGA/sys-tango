// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_vproc_regs -- the registers and loaders of MiSTer's video and audio processing (README "Video and audio processing"):
// decodes the SYS link commands 0x2A FLTCOEF, 0x2B FLTNUM, 0x32 SET_GAMMA, 0x33 GAMCURV, 0x39 SET_AFILTER and
// 0x3E SHADOWMASK (MiSTer's UIO_* numbers, Main_MiSTer user_io.h 52-72) from the bytes tango_link has processed
// (its TANGO_LINK_TAP outputs), and drives ascal's polyphase port, the gamma tables, the shadow mask and the audio
// filter's coefficients.  Our own code; the meanings are MiSTer's, read from Template_MiSTer 19e89ce sys_top.v
// 339-347 / 370-397 / 468-503, hps_io.sv 527-532 and ascal.vhd 2408-2437.
//
// Wire format (README, link commands; byte n = data byte n, i.e. link byte n + 2; multi-byte fields big-endian):
//   0x2A  n0 set (0 H, 1 V, 2 H adaptive, 3 V adaptive), n1 k (the transfer carries 2^k phases, k 0..8), then per
//         phase 4 taps (t0..t3, source pixels -1..+2) as 16-bit words, bits 9:0 two's complement, 256 = 1.0.
//         ascal here has FRAC = TANGO_VPROC_FRAC (8, MiSTer's: 256 phase slots per set; tango_cfg.vh).  Slot s gets
//         phase floor(s * 2^k / 2^FRAC): with FRAC 8 every file's phases (k <= 8) are all kept, each repeated
//         2^(8-k) times, which is MiSTer's scale_phases (video.cpp 352-374) followed by its full-precision send
//         (send_phases 483-501: all 256 phases, 10 bits; sys_top.v 469-470 writes them to ascal's poly port as
//         they come).  (FRAC 4, the earlier builds: k > 4 kept every 2^(k-4)-th phase, as send_phases_legacy.)
//         ascal writes one tap per poly_wr into a holding word and the whole 40-bit word into the phase one clock
//         later (ascal.vhd 2412-2433), so a phase's 4 taps are written once and each further slot it fills needs
//         one more write: a 1-phase file fills 256 slots in 259 clocks, less than its 8 bytes take on the link
//         (8 x 8 SCK periods at 13.3 MHz = 356 clocks) or from the preset ROM (8 x 64 clocks).
//   0x2B  n0 mode: bit0 polyphase on (ascal mode 100, else 000 nearest: sys_top 791), bit1 the V set holds the Scan
//         filter: the framework's scanline dimming is off.  Reply (link byte 3): flags {2'b0, sl[1:0],
//         4'b0111}: bits 1:0 = 3 adaptive sets supported, bit 2 = 1 full precision (256 phases, 10 bits: FRAC 8;
//         0 with FRAC 4), bits 5:4 = the scanline level tango_video applies (MiSTer: {fb_en, VGA_SL, 4'b0111},
//         sys_top 364, 400-402).
//   0x32  n0 bit0 gamma on.  Reply 1.
//   0x33  n0 first index, then R G B per index, auto-increment, up to index 255 (hps_io.sv 528-532: table
//         ch*256 + index).
//   0x39  the command byte restores MiSTer's defaults and holds the filter in reset until the transfer ends
//         (sys_top 386-397); n0 core volume {boost[6:5] (not built), att[4:0]}, n1-4 rate, n5-9 cx (40 bit), n10 cx0,
//         n11 cx1, n12 cx2, n13-15 cy0, n16-18 cy1, n19-21 cy2 (24 bit).  At the end: cx >>= att (sys_top 375).
//         The volume byte's boost[6:5] goes to the mix stage (vol_boost, sys_top 477).  Reply 3 (bit 1: boost, as
//         sys_top 396; audio.cpp 34).
//   0x3E  16-bit words to the shadow mask (MiSTer's command words, video.cpp 808-815).  Reply 1.
//   0x26  n0 the global volume {mute, att[3:0]} (UIO_AUDVOL, audio.cpp 126-137; sys_top.v 466), to the audio
//         filter's mix stage.  No reply.
// Replies are byte 3 of the transfer: tango_link answers them (TANGO_LINK_TAP), FLTNUM's with `flags`.
// TANGO_VPROC_PRESET: a compiled-in byte stream of such transfers (build_tools/vproc_preset.py, a logic ROM:
// `$readmemh` BSRAM contents do not survive Gowin synthesis) is replayed through the same decode after reset, one
// byte per 64 clocks; link bytes are ignored until it has finished (a few ms after reset, long before the BL616
// firmware talks to a core).  The firmware's later commands replace what it loaded.  Two records besides transfers,
// for silicon tests of sequences (vproc_preset.py --afilter-cycle): length FFFF = a wait (u16 milliseconds follow),
// length FFFE = back to the ROM's start (the replay then never ends: the link stays ignored).
//==============================================================================
`include "tango_cfg.vh"
`default_nettype none
module tango_vproc_regs (
    input  wire        clk,                 // clk_pix
    input  wire        rst,
    // tango_link TANGO_LINK_TAP: the SYS-target bytes it has processed
    input  wire        tap_cmd_stb,         // a SYS command byte (link byte 1) was processed; tap_cmd is it
    input  wire        tap_stb,             // a SYS data byte (link byte >= 2) was processed
    input  wire [7:0]  tap_cmd,
    input  wire [15:0] tap_n,               // its data byte number (link byte - 2)
    input  wire [7:0]  tap_byte,
    input  wire        tap_end,             // the transfer has ended (tango_link end_cnt == 8)
    input  wire [1:0]  sl_level,            // the scanline level tango_video applies (FLTNUM flags 5:4)
    output wire [7:0]  flags,               // FLTNUM's reply; tango_link answers byte 3 of these commands itself
`ifdef TANGO_VIDEO_PROC
    // ascal polyphase port (clk), the scaler mode, gamma, shadow mask
    output reg         poly_wr = 1'b0,
`ifdef TANGO_VIDEO_PROC
    output reg  [`TANGO_VPROC_FRAC+3:0] poly_a = 0,     // {set[1:0], phase[FRAC-1:0], tap[1:0]} (ascal FRAC+3..0)
`else
    output reg  [7:0]  poly_a = 8'd0,
`endif
    output reg  [9:0]  poly_dw = 10'd0,
    output reg  [1:0]  flt_mode = 2'd0,
    output reg         gam_en = 1'b0,
    output reg         gam_wr = 1'b0,
    output reg  [9:0]  gam_a = 10'd0,       // {channel 0 R / 1 G / 2 B, index}
    output reg  [7:0]  gam_d = 8'd0,
    output reg         mask_wr = 1'b0,
    output reg  [15:0] mask_cmd = 16'd0,
`endif
    // audio filter coefficients (MiSTer's defaults: sys_top 339-346)
    output reg         a_rst = 1'b0,
    output reg  [31:0] a_rate = 32'd7056000,
    output reg  [39:0] a_cx  = 40'd4258969,
    output reg  [7:0]  a_cx0 = 8'd3,
    output reg  [7:0]  a_cx1 = 8'd3,
    output reg  [7:0]  a_cx2 = 8'd1,
    output reg  [23:0] a_cy0 = -24'sd6216759,
    output reg  [23:0] a_cy1 =  24'sd6143386,
    output reg  [23:0] a_cy2 = -24'sd2023767,
    output reg  [1:0]  a_boost = 2'd0,      // 0x39 n0[6:5]: MiSTer's vol_boost (sys_top.v 477)
    output reg  [4:0]  a_vol = 5'd0,        // 0x26 UIO_AUDVOL: the global attenuation (sys_top.v 466; MiSTer powers up
                                            // muted, 5'b11111: Tango at full volume, for boards without the firmware)
    output wire        preset_busy          // the compiled-in preset is still being replayed
);
    localparam [7:0] C_FLTCOEF = 8'h2A, C_FLTNUM = 8'h2B, C_GAMMA = 8'h32, C_GAMCURV = 8'h33,
                     C_AFILTER = 8'h39, C_SHMASK = 8'h3E, C_AUDVOL = 8'h26;

    //------------------------------------------------------------------ the byte source: preset replay, then the link
    reg        p_cmd_stb = 1'b0, p_stb = 1'b0, p_end = 1'b0;
    reg [7:0]  p_cmd = 8'd0, p_byte = 8'd0;
    reg [15:0] p_n = 16'd0;
    reg        p_busy;
`ifdef TANGO_VPROC_PRESET
    `include "tango_vproc_preset.vh"     // function [7:0] vproc_preset_byte(input [15:0] a); localparam VPROC_PRESET_LEN
    // ROM layout (build_tools/vproc_preset.py): per transfer a 16-bit big-endian length, then that many bytes:
    // target (00), command, data; a length of 0, or the end of the ROM, ends the replay.
    initial p_busy = 1'b1;
    reg [15:0] p_a = 16'd0;              // ROM address
    reg [2:0]  p_st = 3'd0;              // 0 length high byte, 1 length low byte, 2 the transfer's bytes, 3 / 4 a
                                         // wait's milliseconds (high, low byte), 5 waiting
    reg [15:0] p_ms = 16'd0;             // a wait: milliseconds left
    reg [16:0] p_div = 17'd0;            // clocks of the current millisecond (74250 at 74.25 MHz)
    reg [15:0] p_len = 16'd0, p_i = 16'd0;
    reg [5:0]  p_tick = 6'd0;
    reg [7:0]  p_q = 8'd0;               // the ROM byte at p_a, two clocks after p_a (it has 64)
    // The ROM is logic: a 16-bit case.  In two stages, so a large one (a 256-phase file is 2 KB per set) meets clk_pix:
    // one 256-byte block per value of p_a[15:8], each block's case with its high byte constant (8 levels of mux),
    // then the block's byte by p_a[15:8].  (In one stage, a 4 KB ROM missed 74.25 MHz by 3.7 ns.)
    // (A flat vector, not an array of wires: GowinSynthesis stops with "error bus name set" on the latter.)
    localparam integer P_NBLK = (VPROC_PRESET_LEN + 255) / 256;
    reg  [8*P_NBLK-1:0] p_qb = 0;
    integer pk;
    always @(posedge clk)
        for (pk = 0; pk < P_NBLK; pk = pk + 1) p_qb[8*pk +: 8] <= vproc_preset_byte(pk * 256 + p_a[7:0]);
    reg [7:0] p_ahi = 8'd0;
    always @(posedge clk) begin p_ahi <= p_a[15:8]; p_q <= p_qb[8*p_ahi +: 8]; end
    always @(posedge clk) begin
        p_cmd_stb <= 1'b0; p_stb <= 1'b0; p_end <= 1'b0;
        if (rst) begin
            p_busy <= 1'b1; p_a <= 16'd0; p_st <= 3'd0; p_tick <= 6'd0;
        end else if (p_busy && p_st == 3'd5) begin      // a wait
            if (p_div == 17'd74249) begin
                p_div <= 17'd0; p_ms <= p_ms - 16'd1;
                if (p_ms == 16'd1) begin p_st <= 3'd0; p_tick <= 6'd0; end
            end else p_div <= p_div + 17'd1;
        end else if (p_busy) begin
            p_tick <= p_tick + 6'd1;
            if (p_tick == 6'd63) begin                   // one byte per 64 clocks (p_q settled long before)
                if (p_a >= VPROC_PRESET_LEN) p_busy <= 1'b0;
                else begin
                    p_a <= p_a + 16'd1;
                    case (p_st)
                    3'd0: begin p_len[15:8] <= p_q; p_st <= 3'd1; end
                    3'd1: begin
                        p_len[7:0] <= p_q; p_i <= 16'd0; p_st <= 3'd2;
                        if ({p_len[15:8], p_q} == 16'd0) p_busy <= 1'b0;
                        else if ({p_len[15:8], p_q} == 16'hFFFF) p_st <= 3'd3;                         // a wait
                        else if ({p_len[15:8], p_q} == 16'hFFFE) begin p_a <= 16'd0; p_st <= 3'd0; end  // the start
                    end
                    3'd3: begin p_ms[15:8] <= p_q; p_st <= 3'd4; end
                    3'd4: begin
                        p_ms[7:0] <= p_q; p_div <= 17'd0;
                        p_st <= ({p_ms[15:8], p_q} == 16'd0) ? 3'd0 : 3'd5;
                    end
                    default: begin
                        p_i <= p_i + 16'd1;
                        if (p_i == 16'd1) begin p_cmd <= p_q; p_cmd_stb <= 1'b1; end
                        else if (p_i >= 16'd2) begin p_byte <= p_q; p_n <= p_i - 16'd2; p_stb <= 1'b1; end
                        if (p_i + 16'd1 == p_len) begin p_end <= 1'b1; p_st <= 3'd0; end
                    end
                    endcase
                end
            end
        end
    end
`else
    initial p_busy = 1'b0;
    always @(posedge clk) p_busy <= 1'b0;
`endif
    assign preset_busy = p_busy;
    // the decode's inputs: the preset while it plays, else the link.  A preset's end pulse comes with its last byte,
    // so its end is delayed one clock to follow that byte (the link's end comes 8 clocks after its last byte).
    // Registered (one clock, every input alike): the decode then starts from flops, not from the preset/link mux
    // (clk_pix timing: the mux on p_busy was the next-worst path of the first preset build, +1.3 ns).
    reg  p_end_q = 1'b0;
    always @(posedge clk) p_end_q <= p_end;
    reg        cmd_stb = 1'b0, stb = 1'b0, xend = 1'b0;
    reg [7:0]  cmdv = 8'd0, b = 8'd0;
    reg [15:0] n = 16'd0;
    // n == 0 and n == 1, the compares the decode below uses, taken in this stage too (n_is0 = (n == 0) and n_is1 =
    // (n == 1) at every clock): they were the first LUT levels of the FLTCOEF writer's load (n -> the 40 w_taps flops),
    // which missed clk_pix by 0.25 ns in an Atari 2600 scaler build.
    reg        n_is0 = 1'b1, n_is1 = 1'b0;
    wire [15:0] n_in = p_busy ? p_n : tap_n;
    always @(posedge clk) begin
        cmd_stb <= p_busy ? p_cmd_stb : tap_cmd_stb;
        stb     <= p_busy ? p_stb     : tap_stb;
        cmdv    <= p_busy ? p_cmd     : tap_cmd;
        n       <= n_in;
        n_is0   <= (n_in == 16'd0);
        n_is1   <= (n_in == 16'd1);
        b       <= p_busy ? p_byte    : tap_byte;
        xend    <= p_busy ? p_end_q   : tap_end;
    end
    reg  [7:0] cur = 8'd0;                   // the command of the transfer being decoded
    always @(posedge clk) if (cmd_stb) cur <= cmdv;

    //------------------------------------------------------------------ FLTNUM's reply
`ifdef TANGO_VIDEO_PROC
    localparam integer FRAC = `TANGO_VPROC_FRAC;        // ascal's FRAC: 2^FRAC phase slots per set
    assign flags = {2'b00, sl_level, (FRAC >= 8) ? 4'b0111 : 4'b0011};
`else
    assign flags = {2'b00, sl_level, 4'b0011};
`endif

`ifdef TANGO_VIDEO_PROC
    //------------------------------------------------------------------ 0x2A FLTCOEF: phases -> ascal's 2^FRAC slots
    reg  [1:0]  c_set = 2'd0;
    reg  [3:0]  c_k = 4'd4;                  // 2^k phases in this transfer
    reg  [8:0]  c_ph = 9'd0;                 // index of the phase being received (0 .. 2^k - 1, 256 max)
    reg  [1:0]  c_tap = 2'd0;
    reg         c_hi = 1'b0;                 // the high byte of a tap word is in
    reg  [1:0]  c_th = 2'd0;                 // its bits 9:8
    reg  [39:0] c_taps = 40'd0;              // the phase's taps so far, t0 in 39:30
    // the writer: a received phase -> ascal (4 tap writes to its first slot, then one write per further slot)
    reg         w_go = 1'b0;                 // a phase is waiting
    reg  [39:0] w_taps = 40'd0;
    reg  [1:0]  w_set = 2'd0;
    reg  [FRAC-1:0] w_slot = 0;              // first slot
    reg  [FRAC:0]   w_cnt = 0;               // slots it fills (1 .. 2^FRAC)
    reg  [2:0]  w_t = 3'd0;                  // writes done for the first slot (0-3 taps), 4 = the further slots
    reg  [FRAC:0]   w_s = 0;                 // further slots done
    // which slots a phase fills: k <= FRAC: slots ph * 2^(FRAC-k) .. +2^(FRAC-k)-1; k > FRAC: slot ph >> (k-FRAC)
    // if the low bits are 0 (FRAC 8: k <= 8 always, every phase kept)
    wire        k_le  = (c_k <= FRAC);
    wire [3:0]  sh_up = FRAC - c_k;          // k <= FRAC
    wire [3:0]  sh_dn = c_k - FRAC;          // k > FRAC
    wire [8:0]  lowmask = (9'd1 << sh_dn) - 9'd1;
    wire        keep  = k_le || ((c_ph & lowmask) == 9'd0);
    wire [8:0]  slot0 = k_le ? (c_ph << sh_up) : (c_ph >> sh_dn);
    wire [FRAC:0] nslot = k_le ? ({{FRAC{1'b0}}, 1'b1} << sh_up) : 1;
    function [9:0] tap_of(input [39:0] t, input [1:0] i);
        tap_of = t[39 - 10 * i -: 10];
    endfunction
`endif

    //------------------------------------------------------------------ the decode
    reg [8:0] g_idx = 9'd0;                  // 0x33: index (9 bits: stops at 256)
    reg [1:0] g_ch = 2'd0;
    reg [4:0] a_att = 5'd0;                  // 0x39: the core volume's att, applied at the end
    reg       a_open = 1'b0;                 // a 0x39 transfer is being decoded
    reg [7:0] m_hi = 8'd0;                   // 0x3E: high byte of a word
    always @(posedge clk) begin
`ifdef TANGO_VIDEO_PROC
        gam_wr <= 1'b0; mask_wr <= 1'b0;
`endif
        if (cmd_stb) begin
            if (cmdv == C_AFILTER) begin                 // sys_top 386-397: defaults, filter held in reset
                a_rate <= 32'd7056000;  a_cx <= 40'd4258969;  a_cx0 <= 8'd3;  a_cx1 <= 8'd3;  a_cx2 <= 8'd1;
                a_cy0 <= -24'sd6216759; a_cy1 <= 24'sd6143386; a_cy2 <= -24'sd2023767;
                a_att <= 5'd0; a_open <= 1'b1; a_rst <= 1'b1;
            end
`ifdef TANGO_VIDEO_PROC
            if (cmdv == C_FLTCOEF) begin c_ph <= 9'd0; c_tap <= 2'd0; c_hi <= 1'b0; end
`endif
        end
        if (xend && a_open) begin                        // sys_top 375: acx >> acx_att when the transfer is over
            a_open <= 1'b0; a_rst <= 1'b0;
            a_cx <= a_cx >> a_att;
        end
        if (stb) begin
            case (cur)
            C_AUDVOL: if (n_is0) a_vol <= b[4:0];
            C_AFILTER: begin
                if (n_is0) begin a_att <= b[4:0]; a_boost <= b[6:5]; end
                else if (n <= 16'd4)  a_rate <= {a_rate[23:0], b};
                else if (n <= 16'd9)  a_cx   <= {a_cx[31:0], b};
                else if (n == 16'd10) a_cx0  <= b;
                else if (n == 16'd11) a_cx1  <= b;
                else if (n == 16'd12) a_cx2  <= b;
                else if (n <= 16'd15) a_cy0  <= {a_cy0[15:0], b};
                else if (n <= 16'd18) a_cy1  <= {a_cy1[15:0], b};
                else if (n <= 16'd21) a_cy2  <= {a_cy2[15:0], b};
            end
`ifdef TANGO_VIDEO_PROC
            C_FLTNUM:  if (n_is0) flt_mode <= b[1:0];
            C_GAMMA:   if (n_is0) gam_en <= b[0];
            C_GAMCURV: begin
                if (n_is0) begin g_idx <= {1'b0, b}; g_ch <= 2'd0; end
                else if (!g_idx[8]) begin
                    gam_wr <= 1'b1; gam_a <= {g_ch, g_idx[7:0]}; gam_d <= b;
                    if (g_ch == 2'd2) begin g_ch <= 2'd0; g_idx <= g_idx + 9'd1; end
                    else g_ch <= g_ch + 2'd1;
                end
            end
            C_SHMASK: begin
                if (!n[0]) m_hi <= b;
                else begin mask_wr <= 1'b1; mask_cmd <= {m_hi, b}; end
            end
            C_FLTCOEF: begin
                if (n_is0) c_set <= b[1:0];
                else if (n_is1) c_k <= (b > 8'd8) ? 4'd8 : b[3:0];
                else if (!c_ph[8] && (c_ph < (9'd1 << c_k))) begin
                    if (!c_hi) begin c_th <= b[1:0]; c_hi <= 1'b1; end
                    else begin
                        c_hi <= 1'b0;
                        c_taps[39 - 10 * c_tap -: 10] <= {c_th, b};
                        c_tap <= c_tap + 2'd1;
                        if (c_tap == 2'd3) begin                 // the phase is complete: hand it to the writer
                            c_ph <= c_ph + 9'd1;
                            if (keep && slot0 < (9'd1 << FRAC)) begin
                                w_go <= 1'b1; w_set <= c_set; w_slot <= slot0[FRAC-1:0]; w_cnt <= nslot;
                                w_taps <= {c_taps[39:10], c_th, b}; w_t <= 3'd0; w_s <= 1;
                            end
                        end
                    end
                end
            end
`endif
            default: ;
            endcase
        end
`ifdef TANGO_VIDEO_PROC
        // the writer: one poly_wr per clock (a phase's 8 bytes take >= 8 x 8 SCK periods, or 8 x 64 clocks from the
        // preset; the longest job, 4 + 255 writes at FRAC 8, takes 259 clocks)
        poly_wr <= 1'b0;
        if (w_go) begin
            if (w_t < 3'd4) begin
                poly_wr <= 1'b1; poly_a <= {w_set, w_slot, w_t[1:0]}; poly_dw <= tap_of(w_taps, w_t[1:0]);
                w_t <= w_t + 3'd1;
                if (w_t == 3'd3 && w_cnt == 1) w_go <= 1'b0;
            end else begin                                       // further slots: tap 3 again, same word
                poly_wr <= 1'b1; poly_a <= {w_set, w_slot + w_s[FRAC-1:0], 2'd3}; poly_dw <= tap_of(w_taps, 2'd3);
                w_s <= w_s + 1'b1;
                if (w_s + 1'b1 == w_cnt) w_go <= 1'b0;
            end
        end
`endif
        // the power-up values, as logic: on silicon the first TANGO_VIDEO_PROC builds kept the filter's power-up
        // coefficients as register initial values only, and without a load the IIR ran unstable (no firmware, JTAG
        // presets without an audio filter: noise and a 2.4 kHz limit cycle at the HDMI output); a load
        // (MiSTer's 0x39) was exact.  rst_pix is held at power-up until the pixel PLL has run for 8 clocks.
        if (rst) begin
            a_rate <= 32'd7056000;  a_cx <= 40'd4258969;  a_cx0 <= 8'd3;  a_cx1 <= 8'd3;  a_cx2 <= 8'd1;
            a_cy0 <= -24'sd6216759; a_cy1 <= 24'sd6143386; a_cy2 <= -24'sd2023767;
            a_att <= 5'd0; a_open <= 1'b0; a_rst <= 1'b0;
        end
    end
endmodule
`default_nettype wire
