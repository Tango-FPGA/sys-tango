// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_sdram -- the framework's SDRAM controller for TANGO_VIDEO_ASCAL builds: one W9825G6KH-6
// (32 MB, 4 banks x 8192 rows x 512 columns x 16 bit), datasheet Rev. A03 (2016-06-01), shared by
//   A, B  a core's two real-time 8-bit ports (TANGO_NES_SDRAM: A = PPU, B = CPU), 22-bit byte address in
//         banks 0/1: bank {0, addr[21]}, row addr[20:10], column addr[9:1], byte addr[0].  The requests
//         come from tango_ab_slot (CLK_SYS): at most one per port per slot, held for the whole slot, and a
//         slot toggle on `clkref` (AB_SLOTS = 1).  A request starts before the next slot is captured; its byte
//         is on doutA / doutB 9-17 clk after its slot is captured while refresh finds empty slots (the NES),
//         up to 27 clk when both ports ask in every slot and refresh is forced (RF_FORCE), and stays there
//         until that port's next read (measured: the Template core's sim/tb_tango_sdram_ab.v, RD_CAP 1, mode 0:
//         A 9-10, B 9-17; mode 1: A 9-20, B 16-27; RD_CAP 0: one clk less).
//   C     ascal's Avalon MM port: 64-bit beats, bursts of 1..255 beats, frame buffers in banks 2/3
//         (tango_cfg.vh RAMBASE 0x0140_0000): bank addr[24:23], row addr[22:10], column addr[9:1].  A burst
//         must stay in one 1 KB row (ascal's bursts are N_BURST = 256 B aligned, ascal.vhd 1944-1945).
// Scheduling, one command per clock, fixed priority init > refresh > A/B > C:
//   - A/B: ACTIVE, READ (no auto-precharge) or WRITE with auto-precharge, then (reads) a single-bank
//     PRECHARGE.  A before B (its deadline is earlier).  One access at a time.
//   - C: ACTIVE, gapless READs (one per 64-bit beat, BL 4) or 4 single-location WRITEs per beat, then a
//     single-bank PRECHARGE (never PRECHARGE ALL: an A/B row may be open).  C's row in bank 2/3 stays open
//     while A/B work in banks 0/1 (bank interleaving).  C issues nothing while an A/B access waits for its
//     READ/WRITE, and never cuts one of its own beats.
//   - refresh: one AUTO REFRESH owed per RFRSH_CYCLES.  With AB_SLOTS it starts only on a slot that brought
//     no A/B request (A/B are never delayed by it), or after RF_FORCE clk without such a slot (the next
//     slot's A/B wait for it: the 27 clk above).
// W9825G6KH-6 rules (datasheet section): tRCD 15 ns, tRP 15 ns, tRC 60 ns, tRAS 42 ns, tRRD 2 tCK, tWR 2 tCK,
// tRSC 2 tCK, CL 3 for tCK >= 6 ns (9.5); DQM read latency 2, write latency 0 (pin description); a READ
// may interrupt a READ burst (7.7); a WRITE with auto-precharge starts precharging tWR after the write, which
// must be >= tRAS after ACTIVE, and the bank is free tDAL = tWR + tRP after the write (7.14, 11.16); no
// READ/WRITE/PRECHARGE during a READ with auto-precharge burst (7.14), so reads close their row with
// PRECHARGE; data continues CL-1 clocks after a PRECHARGE (11.20); AUTO REFRESH needs every bank idle
// (8, Table 1) and tRC before the next ACTIVE / AUTO REFRESH (9.5); power-up: 200 us with DQM and CKE high,
// PRECHARGE ALL, 8 AUTO REFRESH, MODE REGISTER SET (7.1, 7.2).
// Timing structure: each command is decided one clock ahead, from registered state only, into the one-hot
// grant register `g`; the pins (IOB registers: build_core.tcl -oreg_in_iob / -ioreg_in_iob) take an AND-OR
// of `g` with pre-registered operands.  A command granted at edge k is on the pins after edge k+1.
// Read capture: word 0 of a READ on the pins at edge T is taken at the falling edge T+CL+1.5 (RD_CAP 1, on the
// Console 138K: the rising edge T+CL+1 still sees the previous word), RD_CAP 0 at the rising
// edge T+CL+1, RD_CAP 2 at the rising edge T+CL+2.
//==============================================================================
`default_nettype none
module tango_sdram #(
    parameter integer CL           = 3,     // CAS latency (CL 3: tCK >= 6 ns, 9.5)
    parameter integer T_RCD        = 2,     // ACTIVE -> READ/WRITE            15 ns
    parameter integer T_RP         = 2,     // PRECHARGE -> ACTIVE             15 ns
    parameter integer T_RC         = 6,     // ACTIVE/REFRESH -> ACTIVE/REFRESH 60 ns
    parameter integer T_RAS        = 5,     // ACTIVE -> PRECHARGE             42 ns
    parameter integer T_RRD        = 2,     // ACTIVE -> ACTIVE (other bank)   2 tCK (kept by g_bank below)
    parameter integer T_WR         = 2,     // write recovery                  2 tCK
    parameter integer T_RSC        = 2,     // MODE REGISTER SET cycle         2 tCK
    parameter integer RFRSH_CYCLES = 750,   // 64 ms / 8192 at 96 MHz
    parameter integer RF_FORCE     = 255,   // AB_SLOTS: refresh anyway after this many clk owed (< 256)
    parameter integer INIT_CYCLES  = 20000, // power-up pause (>= 200 us at 96 MHz)
    parameter integer RD_CAP       = 1,     // read capture instant (header)
    parameter integer AB_SLOTS     = 0      // 1: ports A/B in use (clkref = tango_ab_slot's slot toggle)
) (
    input  wire        clk,        // clk_sdram
    input  wire        resetn,
    output reg         busy = 1'b1,// power-up initialisation running

    // ---- SDRAM pins (MiSTer emu naming); SDRAM_CLK is driven by the board top (PLL clkout1)
    inout  wire [15:0] SDRAM_DQ,
    output wire [12:0] SDRAM_A,
    output reg  [1:0]  SDRAM_BA   = 2'd0,
    output wire        SDRAM_DQML, SDRAM_DQMH,
    output wire        SDRAM_CKE,
    output wire        SDRAM_nCS,
    output wire        SDRAM_nRAS,
    output wire        SDRAM_nCAS,
    output wire        SDRAM_nWE,

    // ---- ports A / B: tango_ab_slot's held requests (CLK_SYS registers, stable for a whole slot)
    input  wire [21:0] addrA,
    input  wire        oeA, weA,
    input  wire [7:0]  dinA,
    output reg  [7:0]  doutA = 8'd0,
    input  wire [21:0] addrB,
    input  wire        oeB, weB,
    input  wire [7:0]  dinB,
    output reg  [7:0]  doutB = 8'd0,
    input  wire        clkref,     // slot toggle: one edge per slot; addr/oe/we/din change only with it

    // ---- port C: ascal's Avalon MM.  avl_address is a 64-bit word address (N_AW = 29).
    input  wire [28:0] avl_address,
    input  wire        avl_write,
    input  wire [63:0] avl_writedata,
    input  wire        avl_read,
    input  wire [7:0]  avl_burstcount,
    input  wire [7:0]  avl_byteenable,
    output reg  [63:0] avl_readdata      = 64'd0,
    output reg         avl_readdatavalid = 1'b0,
    output reg         avl_waitrequest   = 1'b1
);
    localparam integer BL      = 4;                    // read burst length (MODE_REG)
    localparam integer T_RD2WR = CL + BL + 1;          // READ -> WRITE: the BL words are off DQ plus one idle clk
    localparam integer T_WRA   = T_RAS - T_WR;         // ACTIVE -> WRITE with auto-precharge (7.14: tRAS)
    localparam integer T_DAL   = T_WR + T_RP;          // WRITE with auto-precharge -> ACTIVE (7.14)
    localparam [2:0]   CL3     = CL;
    // "k - a >= N" between grants at edges a and k: the counters below see an event from the clock after the
    // grant register held it, so N <= 2 needs only !g[event], and N > 2 also counter >= N-2 (m2).
    function integer m2(input integer n); m2 = (n > 2) ? n - 2 : 0; endfunction

    // ---------------------------------------------------------------- reset
    // resetn comes from another clock (tango_sys: ~rst_pix, clk_pix): 2 flops before any clk_sdram logic uses it,
    // so its release is synchronous here and the source net has one load in this module.
    reg rst_s1 = 1'b0, rst_s2 = 1'b0;
    always @(posedge clk) begin rst_s1 <= resetn; rst_s2 <= rst_s1; end
    wire rstn = rst_s2;

    // ---------------------------------------------------------------- pins
    reg [3:0]  cmd_r  = 4'b1111;   // {nCS, nRAS, nCAS, nWE}
    reg [12:0] a_r    = 13'd0;
    reg        dq_oen = 1'b1;      // 1 = DQ high-Z
    reg [15:0] dq_out = 16'd0;
    reg [1:0]  dqm_r  = 2'b11;     // {DQMH, DQML}
    assign {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} = cmd_r;
    assign SDRAM_A    = a_r;
    assign SDRAM_DQML = dqm_r[0];
    assign SDRAM_DQMH = dqm_r[1];
    assign SDRAM_CKE  = 1'b1;
    assign SDRAM_DQ   = dq_oen ? {16{1'bz}} : dq_out;
    wire [15:0] dq_in = SDRAM_DQ;
    reg  [15:0] dq_neg = 16'd0;                        // DQ sampled on the falling edge (RD_CAP 1)
    always @(negedge clk) dq_neg <= dq_in;
    wire [15:0] dq_cap = (RD_CAP == 1) ? dq_neg : dq_in;

    // command codes {nCS, nRAS, nCAS, nWE} (8, Table 1)
    localparam [3:0] CMD_NOP = 4'b1111, CMD_ACT = 4'b0011, CMD_READ = 4'b0101, CMD_WRITE = 4'b0100,
                     CMD_PRE = 4'b0010, CMD_AREF = 4'b0001, CMD_MRS = 4'b0000;
    // mode register (7.2): burst read BL 4 sequential, single-location write (A9 = 1), CL
    localparam [12:0] MODE_REG = {3'b000, 1'b1, 2'b00, CL3, 1'b0, 3'b010};

    // ---------------------------------------------------------------- grant register (one-hot)
    localparam integer G_PREALL = 0, G_AREF = 1, G_MRS = 2,
                       G_AB_ACT = 3, G_AB_RD = 4, G_AB_WR = 5, G_AB_PRE = 6,
                       G_C_ACT  = 7, G_C_RD  = 8, G_C_WR  = 9, G_C_PRE  = 10;
    reg [10:0] g = 11'd0;
    wire g_rd   = g[G_AB_RD] | g[G_C_RD];
    wire g_wr   = g[G_AB_WR] | g[G_C_WR];
    // a bank command was granted at the last edge: no ACTIVE in this clock (this keeps tRRD = 2, and covers
    // the clock before the bank counters have taken the event)
    wire g_bank = g[G_PREALL] | g[G_AREF] | g[G_AB_ACT] | g[G_AB_WR] | g[G_AB_PRE] | g[G_C_ACT] | g[G_C_PRE];

    // ---------------------------------------------------------------- "clocks since" counters (from g)
    reg [3:0] s_rdc = 4'hF, s_rd = 4'hF, s_wr = 4'hF, s_aref = 4'hF, s_abact = 4'hF, s_cact = 4'hF;
    reg f_rdc_bl = 1'b1;     // a READ may follow the last C READ (C beats are never cut)
    reg f_rd_wr  = 1'b1;     // a WRITE may follow the last READ (bus turnaround)
    reg f_aref   = 1'b1;     // AUTO REFRESH -> AUTO REFRESH (tRC)
    reg f_ab_wra = 1'b1;     // A/B: ACTIVE -> WRITE with auto-precharge
    reg f_ab_ras = 1'b1;     // A/B: ACTIVE -> PRECHARGE (tRAS)
    reg f_ab_rcd = 1'b1;     // A/B: ACTIVE -> READ/WRITE (tRCD)
    reg f_c_rcd  = 1'b1;     // C:   ACTIVE -> READ/WRITE (tRCD)
    reg f_c_ras  = 1'b1;     // C:   ACTIVE -> PRECHARGE (tRAS)
    reg f_wr     = 1'b1;     // WRITE -> PRECHARGE (tWR)
    function [3:0] inc4(input [3:0] v); inc4 = (v == 4'hF) ? 4'hF : v + 4'd1; endfunction
    wire [3:0] n_rdc   = g[G_C_RD]   ? 4'd0 : inc4(s_rdc);
    wire [3:0] n_rd    = g_rd        ? 4'd0 : inc4(s_rd);
    wire [3:0] n_wr    = g_wr        ? 4'd0 : inc4(s_wr);
    wire [3:0] n_aref  = g[G_AREF]   ? 4'd0 : inc4(s_aref);
    wire [3:0] n_abact = g[G_AB_ACT] ? 4'd0 : inc4(s_abact);
    wire [3:0] n_cact  = g[G_C_ACT]  ? 4'd0 : inc4(s_cact);
    always @(posedge clk) begin
        s_rdc <= n_rdc;  s_rd <= n_rd;  s_wr <= n_wr;  s_aref <= n_aref;  s_abact <= n_abact;  s_cact <= n_cact;
        f_rdc_bl <= (n_rdc   >= m2(BL));
        f_rd_wr  <= (n_rd    >= m2(T_RD2WR));
        f_aref   <= (n_aref  >= m2(T_RC));
        f_ab_wra <= (n_abact >= m2(T_WRA));
        f_ab_ras <= (n_abact >= m2(T_RAS));
        f_ab_rcd <= (n_abact >= m2(T_RCD));
        f_c_rcd  <= (n_cact  >= m2(T_RCD));
        f_c_ras  <= (n_cact  >= m2(T_RAS));
        f_wr     <= (n_wr    >= m2(T_WR));
    end

    // ---------------------------------------------------------------- A/B
    // slot toggle: 3 flops; the held requests are taken on the clock after the second flop changed
    reg cs1 = 1'b0, cs2 = 1'b0, cs3 = 1'b0;
    always @(posedge clk) begin cs1 <= clkref; cs2 <= cs1; cs3 <= cs2; end
    wire cap       = (AB_SLOTS != 0) && (cs2 ^ cs3) && !busy;
    wire reqA      = oeA | weA;                         // held for the whole slot (tango_ab_slot)
    wire reqB      = oeB | weB;
    wire cap_empty = cap && !reqA && !reqB;
    // pending (taken, not started) and the access in progress
    reg        pa_v = 1'b0, pb_v = 1'b0;
    reg [21:0] pa_ad = 22'd0, pb_ad = 22'd0;
    reg        pa_we = 1'b0, pb_we = 1'b0;
    reg [7:0]  pa_d = 8'd0, pb_d = 8'd0;
    reg        ab_i = 1'b1, ab_a = 1'b0, ab_rw = 1'b0, ab_p = 1'b0;   // one-hot: idle, ACTIVE, READ/WRITE, PRECHARGE
    reg        ab_port = 1'b0;                          // 0 = A, 1 = B
    reg        ab_bank = 1'b0;                          // bank {0, ab_bank}
    reg [12:0] ab_row  = 13'd0;
    reg [8:0]  ab_col  = 9'd0;
    reg        ab_byte = 1'b0, ab_we = 1'b0;
    reg [7:0]  ab_din  = 8'd0;
    reg        ab_overrun = 1'b0;                        // debug (sim): a port's request came before its last one started
    wire ab_end  = g[G_AB_WR] | g[G_AB_PRE];             // the access's last command was granted at the last edge
    wire ab_free = ab_i | ab_end;
    wire ab_wait = ab_a | ab_rw | pa_v | pb_v;           // an A/B access waits for its READ/WRITE: C holds off

    // ---------------------------------------------------------------- refresh
    reg [11:0] rf_cnt = 12'd0;
    reg [1:0]  rf_due = 2'd0;                            // AUTO REFRESHes owed
    reg [7:0]  rf_age = 8'd0;                            // clk one has been owed without starting
    reg        rf_p = 1'b0, rf_r = 1'b0;                 // one-hot: PRECHARGE ALL, AUTO REFRESH
    wire       rf_busy = rf_p | rf_r;
    wire       rf_tick = (rf_cnt == RFRSH_CYCLES[11:0] - 12'd1);

    // ---------------------------------------------------------------- C
    reg        c_i = 1'b1, c_a = 1'b0, c_r = 1'b0;       // one-hot: idle, ACTIVE needed, row open
    reg        c_rd = 1'b0;                              // the burst reads
    reg [1:0]  c_bank = 2'd0;
    reg [12:0] c_row  = 13'd0;
    reg [8:0]  c_rcol = 9'd0, c_wcol = 9'd0;             // column of the next READ / WRITE
    reg [7:0]  c_rleft = 8'd0;                           // READs still to grant
    reg        c_rl2 = 1'b0, c_rl1 = 1'b0;               // c_rleft >= 2, >= 1
    reg [9:0]  c_wleft = 10'd0;                          // WRITE words still to grant
    reg        c_wl2 = 1'b0, c_wl1 = 1'b0;
    reg [7:0]  c_bleft = 8'd0;                           // write beats still to accept from the master
    // write data: two beat buffers (a ring, filled a whole beat at a time) and the head word, the next WRITE's
    // data, registered so the pins take it directly.  The head advances one word per C WRITE granted.
    reg [63:0] wp0 = 64'd0, wp1 = 64'd0;                 // beat buffers
    reg [7:0]  wm0 = 8'd0,  wm1 = 8'd0;                  // their ~byte enables (DQM high masks)
    reg        wv0 = 1'b0,  wv1 = 1'b0;                  // the buffer holds words not yet granted
    reg        ws  = 1'b0;                               // the buffer the next beat goes to
    reg        rs  = 1'b0;                               // the buffer of the head word
    reg [1:0]  ri  = 2'd0;                               // the head word's index in its buffer
    reg [15:0] hd  = 16'd0;                              // the head word
    reg [1:0]  hm  = 2'b00;                              // its {DQMH, DQML}
    reg        hv  = 1'b0;                               // hd holds a word
    reg        h2  = 1'b0;                               // and another word follows it
    reg [63:0] c_rdata = 64'd0;
    reg        c_err_row = 1'b0;                         // debug (sim): a burst would leave its row
    wire       c_block = rf_busy | ab_wait | busy;

    // ---------------------------------------------------------------- init (7.1)
    reg [14:0] i_cnt = 15'd0;
    reg [3:0]  i_n   = 4'd0;
    reg        i_w = 1'b1, i_p = 1'b0, i_f = 1'b0, i_m = 1'b0, i_e = 1'b0;   // one-hot: pause, PREALL, AREFs, MRS, end
    reg [3:0]  i_t   = 4'd0;

    // ---------------------------------------------------------------- banks (from g)
    // open: a row is open.  wt: clk (as N-2) before an ACTIVE is allowed: tRC after ACTIVE, tRP after
    // PRECHARGE, tDAL after a WRITE with auto-precharge, tRC after AUTO REFRESH.  ok = !open && wt == 0.
    wire [3:0] bk_open, bk_ok;
    genvar gb;
    generate for (gb = 0; gb < 4; gb = gb + 1) begin : bank
        reg       open = 1'b0;
        reg [2:0] wt   = 3'd0;
        reg       ok   = 1'b1;
        wire      ev_act = (g[G_AB_ACT] && gb == {1'b0, ab_bank}) || (g[G_C_ACT] && gb == c_bank);
        wire      ev_pre = g[G_PREALL] || (g[G_AB_PRE] && gb == {1'b0, ab_bank}) || (g[G_C_PRE] && gb == c_bank);
        wire      ev_wra = g[G_AB_WR] && gb == {1'b0, ab_bank};
        wire [2:0] w0 = (wt != 3'd0) ? wt - 3'd1 : 3'd0;
        wire [2:0] w1 = ev_act ? m2(T_RC) : w0;
        wire [2:0] w2 = (ev_pre && w1 < m2(T_RP)) ? m2(T_RP) : w1;
        wire [2:0] w3 = (ev_wra && w2 < m2(T_DAL)) ? m2(T_DAL) : w2;
        wire [2:0] w4 = (g[G_AREF] && w3 < m2(T_RC)) ? m2(T_RC) : w3;
        wire       o1 = ev_act ? 1'b1 : (ev_pre || ev_wra) ? 1'b0 : open;
        always @(posedge clk) begin
            open <= o1 && rstn;
            wt   <= w4;
            ok   <= !(o1 && rstn) && (w4 == 3'd0);
        end
        assign bk_open[gb] = open;
        assign bk_ok[gb]   = ok;
    end endgenerate

    // ================================================================ decisions (registered state only)
    wire d_i_pre   = i_p && !g[G_PREALL];
    wire d_i_aref  = i_f && (i_t == 4'd0) && !g[G_AREF];
    wire d_i_mrs   = i_m && (i_t == 4'd0) && !g[G_MRS];
    // refresh: PRECHARGE ALL once every open row may close (C's last beat out, tWR, tRAS; A/B is idle while
    // rf_busy), then AUTO REFRESH once every bank is ready (tRP, tDAL, tRC) and tRC after the last one
    wire d_rf_pre  = rf_p && (bk_open != 4'd0) && !g[G_PREALL] && !g[G_C_RD] && f_rdc_bl && !g_wr && f_wr
                     && !g[G_C_ACT] && f_c_ras;
    wire d_rf_aref = rf_r && !g_bank && (bk_ok == 4'hF) && f_aref;
    // A/B
    wire ab_bok    = ab_bank ? bk_ok[1] : bk_ok[0];
    wire d_ab_act  = ab_a && !g_bank && ab_bok && !rf_busy && !busy;
    wire d_ab_rd   = ab_rw && !ab_we && !g[G_AB_RD] && f_ab_rcd && !g[G_C_RD] && f_rdc_bl && !rf_busy;
    wire d_ab_wr   = ab_rw &&  ab_we && !g[G_AB_WR] && f_ab_rcd && f_ab_wra && !g_rd && f_rd_wr && !rf_busy;
    wire d_ab_pre  = ab_p && !g[G_AB_PRE] && f_ab_ras;
    // C: never while an A/B access waits, and after the A/B PRECHARGE
    wire c_bok     = c_bank[1] ? (c_bank[0] ? bk_ok[3] : bk_ok[2]) : (c_bank[0] ? bk_ok[1] : bk_ok[0]);
    wire c_go      = !c_block && !d_ab_pre;
    wire c_done    = c_rd ? !c_rl1 : !c_wl1;            // every READ / WRITE of the burst was granted
    wire d_c_act   = c_go && c_a && !g_bank && c_bok;
    wire d_c_rd    = c_go && c_r && c_rd && (c_rl2 || (c_rl1 && !g[G_C_RD])) && !g[G_C_RD] && f_rdc_bl
                     && !g[G_C_ACT] && f_c_rcd;
    wire wv_rs     = rs ? wv1 : wv0;
    wire c_hd_ok   = g[G_C_WR] ? h2 : (hv || wv_rs);    // the head register holds a word at the next edge
    wire d_c_wr    = c_go && c_r && !c_rd && (c_wl2 || (c_wl1 && !g[G_C_WR])) && c_hd_ok
                     && !g_rd && f_rd_wr && !g[G_C_ACT] && f_c_rcd;
    wire d_c_pre   = c_go && c_r && c_done && !g[G_C_PRE] && !g[G_C_RD] && f_rdc_bl && !g_wr && f_wr
                     && !g[G_C_ACT] && f_c_ras;

    always @(posedge clk)
        g <= rstn ? {d_c_pre, d_c_wr, d_c_rd, d_c_act, d_ab_pre, d_ab_wr, d_ab_rd, d_ab_act,
                       d_i_mrs, d_i_aref | d_rf_aref, d_i_pre | d_rf_pre} : 11'd0;

    // ================================================================ pins: AND-OR of g and registered operands
    wire [15:0] ab_wdata = {ab_din, ab_din};
    wire [1:0]  ab_wdqm  = ab_byte ? 2'b01 : 2'b10;      // write the addressed byte only (DQM high masks it)
    always @(posedge clk) begin
        cmd_r <= ~(({4{g[G_PREALL] | g[G_AB_PRE] | g[G_C_PRE]}} & ~CMD_PRE)
                 | ({4{g[G_AREF]}} & ~CMD_AREF) | ({4{g[G_MRS]}} & ~CMD_MRS)
                 | ({4{g[G_AB_ACT] | g[G_C_ACT]}} & ~CMD_ACT)
                 | ({4{g_rd}} & ~CMD_READ) | ({4{g_wr}} & ~CMD_WRITE));
        a_r   <=  ({13{g[G_PREALL]}} & 13'h0400)                       // A10 = 1: all banks
                | ({13{g[G_MRS]}}    & MODE_REG)
                | ({13{g[G_AB_ACT]}} & ab_row)
                | ({13{g[G_AB_RD]}}  & {4'b0000, ab_col})              // A10 = 0: the row stays open
                | ({13{g[G_AB_WR]}}  & {4'b0010, ab_col})              // A10 = 1: auto-precharge
                | ({13{g[G_C_ACT]}}  & c_row)
                | ({13{g[G_C_RD]}}   & {4'b0000, c_rcol})
                | ({13{g[G_C_WR]}}   & {4'b0000, c_wcol});             // single-bank PRECHARGE: A10 = 0
        SDRAM_BA <= ({2{g[G_AB_ACT] | g[G_AB_RD] | g[G_AB_WR] | g[G_AB_PRE]}} & {1'b0, ab_bank})
                  | ({2{g[G_C_ACT] | g[G_C_RD] | g[G_C_WR] | g[G_C_PRE]}} & c_bank);
        // DQM high through the power-up pause (7.1); write byte masks (latency 0); else low, so every word of a
        // read is driven (latency 2: a word sampled at T+CL needs DQM low at T+CL-2)
        dqm_r  <= busy ? 2'b11 : (({2{g[G_AB_WR]}} & ab_wdqm) | ({2{g[G_C_WR]}} & hm));
        dq_oen <= !g_wr;
        dq_out <= ({16{g[G_AB_WR]}} & ab_wdata) | ({16{g[G_C_WR]}} & hd);
    end

    // ================================================================ read return
    // A READ granted at edge k is on the pins after k+1; its word i is processed at edge k+RPROC+i, where its
    // tag (shifted in at k+1) is at index RPROC-2+i.
    localparam integer RPROC = 1 + CL + 1 + ((RD_CAP != 0) ? 1 : 0);
    localparam integer RI    = RPROC - 2;
    reg [RI+3:0] rt_c  = {(RI+4){1'b0}};
    reg [RI:0]   rt_ab = {(RI+1){1'b0}}, rt_pt = {(RI+1){1'b0}}, rt_by = {(RI+1){1'b0}};
    always @(posedge clk) begin
        rt_c  <= {rt_c[RI+2:0],  g[G_C_RD]};
        rt_ab <= {rt_ab[RI-1:0], g[G_AB_RD]};
        rt_pt <= {rt_pt[RI-1:0], ab_port};
        rt_by <= {rt_by[RI-1:0], ab_byte};
        if (rt_ab[RI]) begin
            if (rt_pt[RI]) doutB <= rt_by[RI] ? dq_cap[15:8] : dq_cap[7:0];
            else           doutA <= rt_by[RI] ? dq_cap[15:8] : dq_cap[7:0];
        end
        avl_readdatavalid <= 1'b0;
        if (rt_c[RI])   c_rdata[15:0]  <= dq_cap;
        if (rt_c[RI+1]) c_rdata[31:16] <= dq_cap;
        if (rt_c[RI+2]) c_rdata[47:32] <= dq_cap;
        if (rt_c[RI+3]) begin avl_readdata <= {dq_cap, c_rdata[47:0]}; avl_readdatavalid <= 1'b1; end
    end

    // ================================================================ state
    wire [31:0] avl_byte_addr = {avl_address, 3'b000};
    wire acc     = !avl_waitrequest && (avl_read || avl_write);   // the master saw this clock accept it
    wire acc_cmd = acc && c_i;                                    // a new burst (a write brings beat 0)
    wire acc_bt  = acc && !c_i && avl_write;                      // a further write beat
    // write data: the pins took the head word at this edge (pop); a beat accepted at this edge goes to buffer ws
    // (push).  The head moves to the next word (its buffer is freed after word 3); a head that is empty loads the
    // word at (rs, ri) of a buffer filled at an earlier edge (never the beat arriving now: one clock later).
    wire       w_pop   = g[G_C_WR];
    wire       w_push  = (acc_cmd && avl_write) || acc_bt;
    wire       w_rel   = w_pop && (ri == 2'd3);                   // the head leaves buffer rs
    wire [1:0] ri_n    = ri + {1'b0, w_pop};
    wire       rs_n    = rs ^ w_rel;
    wire       wv0_n   = (wv0 && !(w_rel && !rs)) || (w_push && !ws);
    wire       wv1_n   = (wv1 && !(w_rel &&  rs)) || (w_push &&  ws);
    wire       ws_n    = ws ^ w_push;
    wire       hv_n    = w_pop ? h2 : (hv || wv_rs);
    wire [63:0] w_buf  = rs_n ? wp1 : wp0;
    wire [7:0]  w_bm   = rs_n ? wm1 : wm0;
    // waitrequest (registered): low = a command or beat present in the next clock is accepted.  Idle: accept a
    // burst.  Writing: accept a beat while the buffer it would go to is free.
    wire       w_idle  = (c_i && !acc_cmd) || g[G_C_PRE] || (g[G_PREALL] && !busy && c_r && c_done);
    wire       w_wmode = acc_cmd ? avl_write : (!c_i && !c_rd);
    wire [7:0] w_bleft = acc_cmd ? (avl_read ? 8'd0 : avl_burstcount - 8'd1) : (acc_bt ? c_bleft - 8'd1 : c_bleft);
    always @(posedge clk) begin
        // ---- write data
        if (w_push) begin
            if (ws) begin wp1 <= avl_writedata;  wm1 <= ~avl_byteenable; end
            else    begin wp0 <= avl_writedata;  wm0 <= ~avl_byteenable; end
        end
        wv0 <= wv0_n;  wv1 <= wv1_n;  ws <= ws_n;  rs <= rs_n;  ri <= ri_n;
        if (w_pop || !hv) begin
            case (ri_n)
            2'd0:    begin hd <= w_buf[15:0];  hm <= w_bm[1:0]; end
            2'd1:    begin hd <= w_buf[31:16]; hm <= w_bm[3:2]; end
            2'd2:    begin hd <= w_buf[47:32]; hm <= w_bm[5:4]; end
            default: begin hd <= w_buf[63:48]; hm <= w_bm[7:6]; end
            endcase
        end
        hv <= hv_n;
        h2 <= hv_n && ((ri_n != 2'd3) || (rs_n ? wv0_n : wv1_n));
        avl_waitrequest <= busy || !(w_idle || (w_wmode && w_bleft != 8'd0 && !(ws_n ? wv1_n : wv0_n)));

        // ---- init (7.1): pause with DQM high, PRECHARGE ALL, 8 x AUTO REFRESH tRC apart, MRS, tRSC
        if (i_t != 4'd0) i_t <= i_t - 4'd1;
        if (i_w) begin
            i_cnt <= i_cnt + 15'd1;
            if (i_cnt == INIT_CYCLES[14:0]) begin i_w <= 1'b0; i_p <= 1'b1; end
        end
        if (g[G_PREALL] && i_p) begin i_p <= 1'b0; i_f <= 1'b1; i_t <= T_RP[3:0]; i_n <= 4'd0; end
        if (g[G_AREF] && i_f) begin
            i_t <= T_RC[3:0];  i_n <= i_n + 4'd1;
            if (i_n == 4'd7) begin i_f <= 1'b0; i_m <= 1'b1; end
        end
        if (g[G_MRS]) begin i_m <= 1'b0; i_e <= 1'b1; i_t <= T_RSC[3:0]; end
        if (i_e && i_t == 4'd0) begin i_e <= 1'b0; busy <= 1'b0; end

        // ---- refresh: owed every RFRSH_CYCLES; started in an empty slot (AB_SLOTS) or when overdue
        rf_cnt <= rf_tick ? 12'd0 : rf_cnt + 12'd1;
        if (rf_tick && !(g[G_AREF] && !busy)) begin
            if (rf_due != 2'd3) rf_due <= rf_due + 2'd1;
        end else if (!rf_tick && g[G_AREF] && !busy)
            rf_due <= rf_due - 2'd1;
        rf_age <= (rf_due == 2'd0 || rf_busy) ? 8'd0 : ((rf_age == 8'hFF) ? 8'hFF : rf_age + 8'd1);
        if (!busy && !rf_busy && rf_due != 2'd0 && ab_i && !pa_v && !pb_v
                && (AB_SLOTS == 0 || cap_empty || rf_age >= RF_FORCE[7:0]))
            rf_p <= 1'b1;
        if (rf_p && (g[G_PREALL] || (bk_open == 4'd0 && !g_bank))) begin rf_p <= 1'b0; rf_r <= 1'b1; end
        if (rf_r && g[G_AREF]) rf_r <= 1'b0;

        // ---- A/B: start the next access when free (pending A, pending B, then this slot's A, B); take the
        //      slot's requests into pending otherwise
        if (ab_free) begin
            ab_i <= 1'b0;  ab_a <= 1'b1;  ab_rw <= 1'b0;  ab_p <= 1'b0;
            if (pa_v) begin
                pa_v <= 1'b0;  ab_port <= 1'b0;  ab_bank <= pa_ad[21];  ab_row <= {2'b00, pa_ad[20:10]};
                ab_col <= pa_ad[9:1];  ab_byte <= pa_ad[0];  ab_we <= pa_we;  ab_din <= pa_d;
            end else if (pb_v) begin
                pb_v <= 1'b0;  ab_port <= 1'b1;  ab_bank <= pb_ad[21];  ab_row <= {2'b00, pb_ad[20:10]};
                ab_col <= pb_ad[9:1];  ab_byte <= pb_ad[0];  ab_we <= pb_we;  ab_din <= pb_d;
            end else if (cap && reqA) begin
                ab_port <= 1'b0;  ab_bank <= addrA[21];  ab_row <= {2'b00, addrA[20:10]};
                ab_col <= addrA[9:1];  ab_byte <= addrA[0];  ab_we <= weA;  ab_din <= dinA;
            end else if (cap && reqB) begin
                ab_port <= 1'b1;  ab_bank <= addrB[21];  ab_row <= {2'b00, addrB[20:10]};
                ab_col <= addrB[9:1];  ab_byte <= addrB[0];  ab_we <= weB;  ab_din <= dinB;
            end else begin
                ab_i <= 1'b1;  ab_a <= 1'b0;
            end
        end else begin
            if (g[G_AB_ACT]) begin ab_a <= 1'b0; ab_rw <= 1'b1; end
            if (g[G_AB_RD])  begin ab_rw <= 1'b0; ab_p <= 1'b1; end
        end
        if (cap) begin
            if (reqA && !(ab_free && !pa_v && !pb_v)) begin
                if (pa_v && !ab_free) ab_overrun <= 1'b1;
                pa_v <= 1'b1;  pa_ad <= addrA;  pa_we <= weA;  pa_d <= dinA;
            end
            if (reqB && !(ab_free && !pa_v && !pb_v && !reqA)) begin
                if (pb_v && !(ab_free && !pa_v)) ab_overrun <= 1'b1;
                pb_v <= 1'b1;  pb_ad <= addrB;  pb_we <= weB;  pb_d <= dinB;
            end
        end

        // ---- C: accept a burst when idle; ACTIVE; READs / WRITEs; PRECHARGE.  A refresh's PRECHARGE ALL
        //      closes the row: ACTIVE again, or idle if every command of the burst was already granted.
        if (acc_cmd) begin
            c_i <= 1'b0;  c_a <= 1'b1;  c_rd <= avl_read;
            c_bank <= avl_byte_addr[24:23];  c_row <= avl_byte_addr[22:10];
            c_rcol <= avl_byte_addr[9:1];    c_wcol <= avl_byte_addr[9:1];
            c_rleft <= avl_read ? avl_burstcount : 8'd0;
            c_rl2 <= avl_read && (avl_burstcount >= 8'd2);
            c_rl1 <= avl_read && (avl_burstcount != 8'd0);
            c_wleft <= avl_read ? 10'd0 : {avl_burstcount, 2'b00};
            c_wl2 <= !avl_read && (avl_burstcount != 8'd0);
            c_wl1 <= !avl_read && (avl_burstcount != 8'd0);
            c_bleft <= avl_read ? 8'd0 : avl_burstcount - 8'd1;
            if ({1'b0, avl_byte_addr[9:3]} + {1'b0, avl_burstcount} > 9'd128) c_err_row <= 1'b1;
        end
        if (acc_bt) c_bleft <= c_bleft - 8'd1;
        if (g[G_C_ACT]) begin c_a <= 1'b0; c_r <= 1'b1; end
        if (g[G_C_RD]) begin
            c_rcol <= c_rcol + 9'd4;  c_rleft <= c_rleft - 8'd1;
            c_rl2 <= (c_rleft >= 8'd3);  c_rl1 <= (c_rleft >= 8'd2);
        end
        if (g[G_C_WR]) begin
            c_wcol <= c_wcol + 9'd1;  c_wleft <= c_wleft - 10'd1;
            c_wl2 <= (c_wleft >= 10'd3);  c_wl1 <= (c_wleft >= 10'd2);
        end
        if (g[G_C_PRE]) begin c_r <= 1'b0; c_i <= 1'b1; end
        if (g[G_PREALL] && !busy && c_r) begin
            c_r <= 1'b0;
            if (c_done) c_i <= 1'b1; else c_a <= 1'b1;
        end

        if (!rstn) begin
            busy <= 1'b1;  i_cnt <= 15'd0;  i_w <= 1'b1;  i_p <= 1'b0;  i_f <= 1'b0;  i_m <= 1'b0;  i_e <= 1'b0;
            avl_waitrequest <= 1'b1;
            rf_cnt <= 12'd0;  rf_due <= 2'd0;  rf_p <= 1'b0;  rf_r <= 1'b0;
            ab_i <= 1'b1;  ab_a <= 1'b0;  ab_rw <= 1'b0;  ab_p <= 1'b0;  pa_v <= 1'b0;  pb_v <= 1'b0;
            c_i <= 1'b1;  c_a <= 1'b0;  c_r <= 1'b0;
            wv0 <= 1'b0;  wv1 <= 1'b0;  ws <= 1'b0;  rs <= 1'b0;  ri <= 2'd0;  hv <= 1'b0;  h2 <= 1'b0;
        end
    end
endmodule
`default_nettype wire
