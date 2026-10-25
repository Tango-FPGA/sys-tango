// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_test_core -- the DDR3 bring-up test's logic around the DDR3 IP: everything of the test
// top except the PLLs, the IP and the HDMI encoder, so the sims run it on tango_ddr3_model.
//   clk50   the board clock: the IP's reset release (after the DDR3 PLL's lock, as Sipeed's test top.v), uptime,
//           calibration time and drops, pll_stop statistics (all, and after calibration), clk_x1 and clk_ref
//           per second, the 12 "board" slots and the UART report;
//   clk_ref the free-running 100 MHz wall clock (pll_ddr3 CLKOUT0): a 28-bit Gray count;
//   clk_x1  the IP's clk_out: tango_ddr3_tester (slots 12..113), whose copier writes snapshots into two block RAMs
//           here: the display RAM (two banks) and the UART RAM (two banks);
//   clk_pix the display: it shows one bank of the display RAM and takes the other at a frame's start once the copier
//           has written it (d_done); the board slots (0..11) come from clk50 and change at a frame's start too; the
//           banner's values are read from the bank in the vertical blanking; tango_ddr3_ov draws;
//   clk50   also the UART: before each line it asks the copier for a copy in the UART RAM bank it does not read, and
//           reads that bank once answered (or, if clk_x1 does not answer, the bank it has).
// So no reader ever reads a bank being written: the screen and the UART line show whole snapshots.
// Crossings: Gray counts through 2 flops; the snapshots through the dual-clock block RAMs; their handshakes (toggles,
// and a bank bit that settles before its toggle) through 2-3 flops; the board slots as a quasi-static bus (held >=
// 2^SNAP5_LOG2 clocks after their toggle, copied when the toggle arrives through 3 flops); single-bit levels through
// 2 flops.
//==============================================================================
`default_nettype none
module tango_ddr3_test_core #(
    parameter integer WORDS_LOG2  = 25,
    parameter integer NLAT_LOG2   = 15,
    parameter integer BW_LOG2     = 20,
    parameter integer STALL_LOG2  = 20,
    parameter integer SNAP_LOG2   = 21,
    parameter integer SNAP5_LOG2  = 20,
    parameter integer SEC_CLKS    = 50000000,
    parameter integer BAUD_DIV    = 434,         // 50 MHz / 115200
    parameter integer UART_PERIOD = 100000000,   // clk50 clocks between UART reports (2 s)
    parameter integer UART_TO_LOG2 = 23,         // the UART waits up to 2^this clk50 clocks for its copy (168 ms)
    parameter integer DRAIN_CLKS  = 2048         // the tester's quiet time after each FIFO probe
) (
    input  wire          clk50,
    input  wire          clk_ref,
    input  wire          clk_x1,
    input  wire          clk_pix,
    input  wire          rst_pix,
    input  wire          ddr_pll_lock,
    input  wire          hdmi_lock,
    input  wire          pll_stop,
    output reg           ip_rst_n = 1'b0,
    // IP user interface (clk_x1)
    input  wire          init_calib_complete,
    input  wire          ddr_rst,
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
    // display (clk_pix)
    input  wire [11:0]   cx,
    input  wire [10:0]   cy,
    output wire [23:0]   rgb,
    input  wire          freeze,                 // sims: stop display updates (0 on the board)
    output wire          uart_tx,
    output wire          led
);
    localparam integer NS = 114;
    localparam [31:0] MAGIC = 32'hD1D0_0002;

    // ================= clk_ref: the wall clock (28-bit Gray) =================
    reg [27:0] cr = 28'd0, gr = 28'd0;
    always @(posedge clk_ref) begin cr <= cr + 1'b1; gr <= cr ^ (cr >> 1); end
    function [27:0] g2b(input [27:0] g);
        integer k;
        begin
            g2b[27] = g[27];
            for (k = 26; k >= 0; k = k - 1) g2b[k] = g2b[k + 1] ^ g[k];
        end
    endfunction

    // ================= clk_x1: the tester =================
    (* syn_preserve = 1 *) reg [27:0] grx1 = 0, grx2 = 0;
    reg [27:0] wallb = 0;
    reg [1:0]  calx = 2'b00;
    reg        trst = 1'b1;
    reg [27:0] cx1 = 0, gx1 = 0;              // clk_x1's own count, Gray, for clk50's per-second measure
    always @(posedge clk_x1) begin
        grx1 <= gr; grx2 <= grx1; wallb <= g2b(grx2);
        calx <= {calx[0], init_calib_complete};
        trst <= ddr_rst | ~calx[1];
        cx1 <= cx1 + 1'b1; gx1 <= cx1 ^ (cx1 >> 1);
    end
    // the handshakes, synchronized into clk_x1 (S: the display's bank; u_rq / u_rb: the UART's request and bank)
    (* syn_preserve = 1 *) reg [1:0] dsh_s = 2'b00, urb_s = 2'b11;
    (* syn_preserve = 1 *) reg [2:0] urq_s = 3'b000;
    reg  S = 1'b0;                                 // clk_pix: the display RAM bank on screen
    reg  u_rq = 1'b0, u_rb = 1'b1;                 // clk50: the UART's request toggle and the bank it asks for
    always @(posedge clk_x1) begin dsh_s <= {dsh_s[0], S}; urq_s <= {urq_s[1:0], u_rq}; urb_s <= {urb_s[0], u_rb}; end
    wire         dm_we, um_we, d_done, u_ack;
    wire [7:0]   dm_wa, um_wa;
    wire [31:0]  t_wd;
    tango_ddr3_tester #(.WORDS_LOG2(WORDS_LOG2), .NLAT_LOG2(NLAT_LOG2), .BW_LOG2(BW_LOG2), .STALL_LOG2(STALL_LOG2),
                        .SNAP_LOG2(SNAP_LOG2), .DRAIN_CLKS(DRAIN_CLKS)) u_t (
        .clk(clk_x1), .rst(trst), .wall(wallb[23:0]),
        .cmd_ready(cmd_ready), .cmd_en(cmd_en), .cmd(cmd), .addr(addr),
        .wr_data_rdy(wr_data_rdy), .wr_data_en(wr_data_en), .wr_data(wr_data), .wr_data_mask(wr_data_mask),
        .rd_data_valid(rd_data_valid), .rd_data(rd_data),
        .dm_we(dm_we), .dm_wa(dm_wa), .um_we(um_we), .um_wa(um_wa), .wd(t_wd),
        .d_shown(dsh_s[1]), .d_done(d_done), .u_req(urq_s[2]), .u_bank(urb_s[1]), .u_ack(u_ack));
    // the snapshot RAMs: written in clk_x1, read in clk_pix (display) and clk50 (UART); {bank, slot}, slots 12..113
    (* syn_ramstyle = "block_ram" *) reg [31:0] dm [0:255];
    (* syn_ramstyle = "block_ram" *) reg [31:0] um [0:255];
    always @(posedge clk_x1) if (dm_we) dm[dm_wa] <= t_wd;
    always @(posedge clk_x1) if (um_we) um[um_wa] <= t_wd;

    // ================= clk50: board slots 0..11 =================
    reg [1:0]  s_lockd = 0, s_lockh = 0, s_cal = 0, s_ps = 0, s_drst = 0;
    reg [27:0] g_x1a = 0, g_x1b = 0, g_rfa = 0, g_rfb = 0;
    reg [7:0]  rcnt = 8'd0;
    reg        cal_ever = 1'b0, cal_q = 1'b0, ps_q = 1'b1, timing = 1'b0;
    reg [31:0] init_clk = 0, uptime = 0, cal_drops = 0, x1_per_s = 0, rf_per_s = 0;
    reg [31:0] ps_ev = 0, ps_max = 0, ps_ev_pc = 0, ps_max_pc = 0, ps_tot_pc = 0, ps_run = 0;
    reg [31:0] sec = 0;
    reg [27:0] x1_prev = 0, rf_prev = 0;
    always @(posedge clk50) begin
        s_lockd <= {s_lockd[0], ddr_pll_lock}; s_lockh <= {s_lockh[0], hdmi_lock};
        s_cal <= {s_cal[0], init_calib_complete}; s_ps <= {s_ps[0], pll_stop}; s_drst <= {s_drst[0], ddr_rst};
        g_x1a <= gx1; g_x1b <= g_x1a; g_rfa <= gr; g_rfb <= g_rfa;
        // the IP's reset: released 255 clocks after the PLL locks (Sipeed's test top.v), synchronously
        if (!s_lockd[1]) begin rcnt <= 8'd0; ip_rst_n <= 1'b0; end
        else if (rcnt != 8'hFF) rcnt <= rcnt + 1'b1;
        else ip_rst_n <= 1'b1;
        // calibration time and drops
        cal_q <= s_cal[1];
        if (ip_rst_n && !cal_ever && !s_cal[1]) init_clk <= init_clk + 1'b1;
        if (s_cal[1]) cal_ever <= 1'b1;
        if (cal_q && !s_cal[1]) cal_drops <= cal_drops + 1'b1;
        // pll_stop (low = memory_clk stopped): events and runs, all and after calibration
        ps_q <= s_ps[1];
        if (ps_q && !s_ps[1]) begin ps_ev <= ps_ev + 1'b1; if (cal_ever) ps_ev_pc <= ps_ev_pc + 1'b1; end
        if (!s_ps[1]) begin
            ps_run <= ps_run + 1'b1;
            if (ps_run + 1'b1 > ps_max) ps_max <= ps_run + 1'b1;
            if (cal_ever) begin
                ps_tot_pc <= ps_tot_pc + 1'b1;
                if (ps_run + 1'b1 > ps_max_pc) ps_max_pc <= ps_run + 1'b1;
            end
        end else ps_run <= 0;
        // seconds
        if (sec == SEC_CLKS - 1) begin
            sec <= 0; uptime <= uptime + 1'b1;
            x1_per_s <= {4'd0, g2b(g_x1b) - x1_prev}; x1_prev <= g2b(g_x1b);
            rf_per_s <= {4'd0, g2b(g_rfb) - rf_prev}; rf_prev <= g2b(g_rfb);
        end else sec <= sec + 1'b1;
    end
    wire [31:0] flags = {25'd0, s_drst[1], s_ps[1], cal_ever, s_lockh[1], ip_rst_n, s_cal[1], s_lockd[1]};
    wire [12*32-1:0] live5 = {ps_tot_pc, ps_max_pc, ps_ev_pc, ps_max, ps_ev, rf_per_s, x1_per_s, cal_drops,
                              uptime, init_clk, flags, MAGIC};
    reg [12*32-1:0] hold5 = 0;
    reg             hold5_tog = 1'b0;
    reg [SNAP5_LOG2-1:0] sn5 = 0;
    always @(posedge clk50) begin
        sn5 <= sn5 + 1'b1;
        if (&sn5) begin hold5 <= live5; hold5_tog <= ~hold5_tog; end
    end

    // a board slot (0..11) of a 12-slot copy
    function [31:0] slot12(input [383:0] v, input [3:0] i);
        case (i)
            4'd0: slot12 = v[  0 +: 32];  4'd1: slot12 = v[ 32 +: 32];  4'd2:  slot12 = v[ 64 +: 32];
            4'd3: slot12 = v[ 96 +: 32];  4'd4: slot12 = v[128 +: 32];  4'd5:  slot12 = v[160 +: 32];
            4'd6: slot12 = v[192 +: 32];  4'd7: slot12 = v[224 +: 32];  4'd8:  slot12 = v[256 +: 32];
            4'd9: slot12 = v[288 +: 32];  4'd10: slot12 = v[320 +: 32]; 4'd11: slot12 = v[352 +: 32];
            default: slot12 = 32'd0;
        endcase
    endfunction

    // ================= clk_pix: the display =================
    // At a frame's start (the vertical blanking begins: cy reaches 720) the display takes the bank the copier has
    // written since the last one it took (d_done toggled: the other bank), and the board slots that came since; then,
    // still in the blanking, it reads the banner's values out of the bank (slots 12, 13, 14, 16).  Frozen (sims):
    // neither.  The overlay's slot word arrives 2 clocks after its address (the RAM's read, then the board-slot
    // select registered: timing), one clock later than tango_ddr3_ov plans for; the address only changes where no
    // bit is drawn (a slot column's marker, a line's start), so the picture is the same (sim/tb_ddr3_ui.v models it).
    reg  [2:0]   t5s = 3'd0;
    reg  [383:0] b5_new = 384'd0, b5_show = 384'd0;  // the board slots: the latest / on screen
    reg  [2:0]   dds = 3'd0;                          // d_done, synchronized
    reg          dseen = 1'b0;                        // the d_done the display took last
    reg          vb_q = 1'b0;
    reg  [2:0]   bn = 3'd0;                           // the banner's reads: 1..4 read slots 12, 13, 14, 16
    reg  [15:0]  ncopy = 16'd0;                       // banks taken
    reg  [31:0]  ban_err = 0, ban_pass = 0, ban_up = 0, ban_loops = 0, ban_flags = 0, ban_tstat = 0;
    reg  [31:0]  dm_q = 32'd0;                        // the display RAM's word
    wire         vb = (cy >= 11'd720);
    always @(posedge clk_pix) begin
        t5s <= {t5s[1:0], hold5_tog};
        if (t5s[2] != t5s[1]) b5_new <= hold5;       // quasi-static: stable since its toggle
        dds <= {dds[1:0], d_done};
        vb_q <= vb;
        bn <= (bn != 3'd0 && bn != 3'd5) ? bn + 1'b1 : 3'd0;
        if (vb && !vb_q && !freeze) begin
            if (dds[2] != dseen) begin S <= ~S; dseen <= dds[2]; ncopy <= ncopy + 1'b1; end
            b5_show <= b5_new; ban_flags <= b5_new[32 +: 32]; ban_up <= b5_new[96 +: 32];
            bn <= 3'd1;
        end
        case (bn)                                     // the banner's words, a clock after their reads
            3'd2: ban_tstat <= dm_q;
            3'd3: ban_loops <= dm_q;
            3'd4: ban_pass  <= dm_q;
            3'd5: ban_err   <= dm_q;
            default: ;
        endcase
    end
    wire [6:0]  dra;
    wire [6:0]  dra_m = (bn == 3'd1) ? 7'd12 : (bn == 3'd2) ? 7'd13 : (bn == 3'd3) ? 7'd14 : (bn == 3'd4) ? 7'd16 : dra;
    reg         sel5 = 1'b0;
    reg  [31:0] b5_q = 32'd0, drd = 32'd0;
    always @(posedge clk_pix) begin
        dm_q <= dm[{S, dra_m}];
        sel5 <= (dra < 7'd12);
        b5_q <= slot12(b5_show, dra[3:0]);
        drd  <= sel5 ? b5_q : dm_q;
    end
    tango_ddr3_ov u_ov (
        .clk(clk_pix), .cx(cx), .cy(cy), .slot_addr(dra), .slot_data(drd),
        .err(ban_err), .passes(ban_pass), .uptime(ban_up), .loops(ban_loops),
        .calib(ban_flags[1]), .any_err(ban_tstat[6] | (ban_err != 0)), .beat(ncopy[3]), .rgb(rgb));

    // ================= clk50: the UART report =================
    // Before each line (u_want) the UART asks the copier for a copy of slots 12..113 in the UART RAM bank it does not
    // read: the bank first (u_rb), the request toggle a clock later (u_rq).  Once answered (u_ack == u_rq) it reads
    // that bank, with the board slots of that moment (u5).  If clk_x1 does not answer within 2^UART_TO_LOG2 clocks the
    // line goes with the bank it has; the request stays out, and a later line takes its answer.
    reg  [2:0]   uas = 3'd0;                          // u_ack, synchronized
    reg  [1:0]   ust = 2'd0;
    reg          ur = 1'b0;                           // the UART RAM bank the UART reads
    reg  [UART_TO_LOG2-1:0] uto = 0;
    reg  [383:0] u5 = 384'd0;
    reg          u_go = 1'b0;
    wire         u_want;
    always @(posedge clk50) begin
        uas <= {uas[1:0], u_ack};
        u_go <= 1'b0;
        case (ust)
            2'd0: if (u_want && !u_go) begin
                      if (uas[2] == u_rq) begin u_rb <= ~ur; ust <= 2'd1; end           // ask for a copy
                      else begin uto <= 0; ust <= 2'd2; end                               // one is still asked for
                  end
            2'd1: begin u_rq <= ~u_rq; uto <= 0; ust <= 2'd2; end
            2'd2: if (uas[2] == u_rq) begin ur <= u_rb; u5 <= live5; u_go <= 1'b1; ust <= 2'd3; end
                  else if (&uto) begin u5 <= live5; u_go <= 1'b1; ust <= 2'd3; end          // no answer
                  else uto <= uto + 1'b1;
            default: ust <= 2'd0;
        endcase
    end
    wire [6:0]  ura;
    reg  [31:0] um_q = 32'd0, u5_q = 32'd0;
    reg         usel = 1'b0;
    always @(posedge clk50) begin
        um_q <= um[{ur, ura}];
        usel <= (ura < 7'd12);
        u5_q <= slot12(u5, ura[3:0]);
    end
    wire [31:0] urd = usel ? u5_q : um_q;            // the slot, a clock after its address (as the UART reads it)
    tango_ddr3_uart #(.NS(NS), .BAUD_DIV(BAUD_DIV), .PERIOD(UART_PERIOD)) u_uart (
        .clk(clk50), .slot_addr(ura), .slot_data(urd), .tx(uart_tx), .want(u_want), .go(u_go));

    // ================= LED (V13, low = on): on when calibrated and no error; blinking with errors
    assign led = !(s_cal[1] && (ban_err == 0 ? 1'b1 : uptime[0]));
endmodule
`default_nettype wire
