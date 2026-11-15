// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_arb -- the DDR3's command arbiter: one scaler port (V) and NC core ports (C) onto the
// Gowin DDR3 IP's user interface (IPUG281-2.6E 4.4), in the IP's clk_out domain.
// Each port offers one IP-sized request at a time (q_valid / q_ready): a 32-B word, read or write with its data and
// byte mask, at an IP address (32-bit DQ words; a BL8 is 8 of them).  The arbiter offers the IP at most one command
// a clock, with its data, when cmd_ready (and for a write wr_data_rdy) is high, so command and data are always taken
// together.  The rules:
//   1. V first, with a guaranteed core share: after G consecutive V grants with a core command able to go, one core
//      command goes; the C ports take turns (round-robin).
//   2. Q_CORE: at most Q_CORE core commands inside the IP.  The IP executes in order (IPUG281 4.4.5, 4.4.8), so a
//      scaler command waits behind everything queued before it; capping the cores' share of that queue caps the
//      scaler's wait.  A core read retires when its word returns; a write has no completion signal, so it retires
//      when any later read returns (in order, it was done first): every read's tag carries the number of core
//      commands granted since the previous read, itself included.  When core writes are outstanding and no read has
//      followed for FENCE_WAIT clocks, or the cores are at Q_CORE with only writes outstanding, the arbiter issues a
//      fence read (FENCE_ADDR, its word discarded).
//   3. Read tags in issue order route each returned word to its port (r_valid[port], r_data shared), one clock after
//      the IP's rd_data_valid.  A port offers a read only when it has room for its word: the IP's rd_data has no
//      ready.
// Port numbering for r_valid: bit 0 = V, bit 1+i = C port i.
// Each port feeds a 2-entry queue whose ready is a register (tango_ddr3_skid): the grant below works on the queues'
// heads and only pops them.  v_ready / c_ready are those registered readies.
// The IP stage (the IP's data registers lie along ~17 rows, so what drives them comes from registers):
// the grant decides from registers only -- credits for the IP queue, not the IP's cmd_ready -- and its command and
// data go to an output register beside the arbiter, then into a 4-entry queue (each entry's write enable registered,
// a copy per 32-bit lane) whose head drives the IP, a 4-way select by a register.  The IP takes the head when
// cmd_ready (and wr_data_rdy for a write).  Q_CORE and the read tags count from the grant (the queue is in order).
// Observation outputs (sims and debug): the most core commands in the IP, the longest a V request waited, fences.
//==============================================================================
`default_nettype none
module tango_ddr3_arb #(
    parameter integer        NC         = 1,
    parameter integer        G          = 8,
    parameter integer        Q_CORE     = 16,
    parameter integer        TAG_LOG2   = 6,
    parameter integer        FENCE_WAIT = 64,
    parameter [28:0]         FENCE_ADDR = 29'd0
) (
    input  wire              clk,
    input  wire              rst,
    input  wire              rst_qv,           // = rst, a separate register for the V queue (tango_ddr3.v)
    input  wire [NC-1:0]     rst_qc,           // = rst, one per core queue
    // V
    input  wire              v_valid,
    input  wire              v_write,
    input  wire [28:0]       v_addr,
    input  wire [255:0]      v_wdata,
    input  wire [31:0]       v_wmask,
    output wire              v_ready,
    // C ports, packed (port i at [i*w +: w])
    input  wire [NC-1:0]     c_valid,
    input  wire [NC-1:0]     c_write,
    input  wire [NC*29-1:0]  c_addr,
    input  wire [NC*256-1:0] c_wdata,
    input  wire [NC*32-1:0]  c_wmask,
    output wire [NC-1:0]     c_ready,
    // returns
    output reg  [NC:0]       r_valid = 0,
    output reg  [255:0]      r_data = 256'd0,
    // the IP
    input  wire              cmd_ready,
    input  wire              wr_data_rdy,
    output wire              cmd_en,
    output wire [2:0]        cmd,
    output wire [28:0]       addr,
    output wire              wr_data_en,
    output wire [255:0]      wr_data,
    output wire [31:0]       wr_data_mask,
    input  wire              rd_data_valid,
    input  wire [255:0]      rd_data,
    // observation
    output reg  [15:0]       obs_core_inflight_max = 0,
    output reg  [31:0]       obs_v_wait_max = 0,
    output reg  [31:0]       obs_fences = 0,
    output reg  [15:0]       obs_core_inflight = 0
);
    localparam integer TN = 1 << TAG_LOG2;
    localparam integer PW = (NC + 1 > 1) ? $clog2(NC + 2) : 1;     // port id width (V, C ports, fence)
    localparam [PW-1:0] P_FENCE = NC + 1;

    // ---- tags: {port, core commands retired by this read's return}
    // a block RAM read synchronously (the head tag below forwards a write into the place being read)
    (* syn_ramstyle = "block_ram" *) reg [PW+7:0] tagm [0:TN-1];
    reg  [TAG_LOG2:0] tw = 0, tr = 0;
    reg  [7:0]  pend = 8'd0;                 // core commands granted since the last read
    reg  [15:0] inflight = 16'd0;            // core commands in the IP
    reg  [15:0] fence_t = 16'd0;
    // The grant is computed once per group of the registers it loads (timing: one grant net
    // fanned out to ~40 registers; with one slow route, tags_ok -> c_can -> gv -> gc -> core_room failed by 1.2 ns).
    // Each group's copy reads its own copies of the registers it needs most -- room for a tag, the core's room in the IP,
    // the V streak limit -- each updated from that group's own grant (syn_preserve: GowinSynthesis merges identical
    // LUTs), so no grant net is shared between groups.  All copies are equal every clock
    // (checked in simulation).  The groups: INF inflight / core_room; TAG pend, tags, fence_t, tw; STR vstreak, vlim,
    // rr; QC the core queues' pop; IPS the IP stage (ao_*, ien*, cr); QV the V queue's pop.
    localparam integer NG = 6;
    localparam integer G_INF = 0, G_TAG = 1, G_STR = 2, G_QC = 3, G_IPS = 4, G_QV = 5;
    // room for a tag, the core's room in the IP and the V streak limit: registers, computed from the next state
    // (exact), so the grant below is a few gates after cmd_ready -- one copy per group
    (* syn_preserve = 1 *) reg [NG-1:0] tags_ok_g   = {NG{1'b1}};
    (* syn_preserve = 1 *) reg [NG-1:0] core_room_g = {NG{1'b1}};
    (* syn_preserve = 1 *) reg [NG-1:0] vlim_g      = {NG{1'b0}};
    wire tags_ok = tags_ok_g[G_TAG], core_room = core_room_g[G_INF], vlim = vlim_g[G_STR];
    // pend != 0 and fence_t >= FENCE_WAIT as registers (exact, updated with pend and fence_t): the fence decision is one
    // gate of registers
    reg  pend_nz = 1'b0;
    reg  ft_ge   = (16'd0 >= FENCE_WAIT);

    // ---- the ports' queues; the grant works on their heads
    wire         hv_valid;
    wire [317:0] hv;
    wire         hv_pop;
    wire         hv_write;                    // = hv[317], a register of the queue (timing)
    tango_ddr3_skid #(.W(318)) u_qv (.clk(clk), .rst(rst_qv), .in_valid(v_valid), .in_data({v_write, v_addr, v_wdata, v_wmask}),
        .in_ready(v_ready), .out_valid(hv_valid), .out_data(hv), .out_wb(hv_write), .out_pop(hv_pop));
    wire [28:0]  hv_addr  = hv[316:288];
    wire [255:0] hv_wdata = hv[287:32];
    wire [31:0]  hv_wmask = hv[31:0];
    wire [NC-1:0]     hc_valid, hc_write, hc_pop;
    wire [NC*29-1:0]  hc_addr;
    wire [NC*256-1:0] hc_wdata;
    wire [NC*32-1:0]  hc_wmask;
    genvar g;
    generate for (g = 0; g < NC; g = g + 1) begin : g_qc
        wire [317:0] h;
        tango_ddr3_skid #(.W(318)) u_qc (.clk(clk), .rst(rst_qc[g]), .in_valid(c_valid[g]),
            .in_data({c_write[g], c_addr[g * 29 +: 29], c_wdata[g * 256 +: 256], c_wmask[g * 32 +: 32]}),
            .in_ready(c_ready[g]), .out_valid(hc_valid[g]), .out_data(h), .out_wb(hc_write[g]), .out_pop(hc_pop[g]));
        assign hc_addr[g * 29 +: 29] = h[316:288];
        assign hc_wdata[g * 256 +: 256] = h[287:32];
        assign hc_wmask[g * 32 +: 32] = h[31:0];
    end endgenerate

    // ---- who can go this clock (credits for the IP queue: entries free, counting the output register's)
    localparam integer IQ = 4;
    reg  [2:0] cr = IQ;
    reg        cr_ok = (IQ != 0);           // = cr != 0, a register (its next value from cr, the grant and fire)
    // round-robin among the C ports: the first that can go at or after rr
    reg  [PW-1:0] rr = 0;
    reg  [7:0] vstreak = 8'd0;
    // the grant, one copy per group gi (the rules of the header; gv: V, gc: a core port, gf: a fence)
    wire [NG-1:0]    gv_g, gc_g, gf_g, cany_g;
    wire [NG*PW-1:0] csel_g;
    genvar gi;
    generate for (gi = 0; gi < NG; gi = gi + 1) begin : g_grant
        wire ok_r  = cr_ok && tags_ok_g[gi];
        wire ok_w  = cr_ok;
        wire v_can = hv_valid && (hv_write ? ok_w : ok_r);
        wire [NC-1:0] c_can;
        genvar gp;
        for (gp = 0; gp < NC; gp = gp + 1) begin : g_can
            assign c_can[gp] = hc_valid[gp] && core_room_g[gi] && (hc_write[gp] ? ok_w : ok_r);
        end
        reg  [PW-1:0] csel;
        reg           cany;
        integer k, kk;
        always @(*) begin
            csel = 0; cany = 1'b0;
            for (k = 0; k < NC; k = k + 1) begin
                kk = (rr + k) % NC;
                if (!cany && c_can[kk]) begin csel = kk; cany = 1'b1; end
            end
        end
        wire fence_can = pend_nz && (ft_ge || !core_room_g[gi]) && ok_r;
        // V first unless the streak limit holds a core command back; a core command; else a fence
        // (gv = v_can && !(cany && vlim); gc = !gv && cany; gf = !gv && !gc && fence_can, written out)
        assign gv_g[gi]   = v_can && !(cany && vlim_g[gi]);
        assign gc_g[gi]   = cany && (!v_can || vlim_g[gi]);
        assign gf_g[gi]   = !v_can && !cany && fence_can;
        assign cany_g[gi] = cany;
        assign csel_g[gi * PW +: PW] = csel;
    end endgenerate
    assign hv_pop = gv_g[G_QV];
    generate for (g = 0; g < NC; g = g + 1) begin : g_pop
        assign hc_pop[g] = gc_g[G_QC] && (csel_g[G_QC * PW +: PW] == g);
    end endgenerate
    // the IP stage's copy
    wire          gv_e = gv_g[G_IPS], gc_e = gc_g[G_IPS], gf_e = gf_g[G_IPS];
    wire [PW-1:0] csel_e = csel_g[G_IPS * PW +: PW];
    wire          grant = gv_e || gc_e || gf_e;
    wire          g_write = gv_e ? hv_write : gc_e ? hc_write[csel_e] : 1'b0;
    // ---- the IP stage: the granted command, registered beside the arbiter -- both candidates every clock and which
    // one was granted (registers), so no grant decision selects the 317 bits (timing: a Template DDR3 build
    // failed by 0.49 ns on tags_ok -> the grant -> ao_d) ...
    reg          ao_v = 1'b0, ao_w = 1'b0, ao_c = 1'b0, ao_f = 1'b0;   // granted; a write; the core's; the fence
    reg  [28:0]  aov_a = 29'd0, aoc_a = 29'd0;
    reg  [255:0] aov_d = 256'd0, aoc_d = 256'd0;
    reg  [31:0]  aov_m = 32'd0, aoc_m = 32'd0;
    always @(posedge clk) begin
        ao_v <= grant && !rst; ao_w <= g_write; ao_c <= gc_e; ao_f <= gf_e;
        aov_a <= hv_addr; aov_d <= hv_wdata; aov_m <= hv_wmask;
        aoc_a <= hc_addr[csel_e * 29 +: 29]; aoc_d <= hc_wdata[csel_e * 256 +: 256]; aoc_m <= hc_wmask[csel_e * 32 +: 32];
    end
    wire [28:0]  ao_a = ao_f ? FENCE_ADDR : ao_c ? aoc_a : aov_a;
    wire [255:0] ao_d = ao_c ? aoc_d : aov_d;
    wire [31:0]  ao_m = ao_c ? aoc_m : aov_m;
    // ... then the IP queue: the entry at the write pointer is written from the output register EVERY clock, granted or
    // not (ien[k]: the decode of the next write pointer, registered a clock ahead; copies: lanes 0..7 and the command);
    // the head register (below) drives the IP.  That entry is always free: the credits bound ao_v + m_n + hd_v by IQ,
    // and with m_n != 0 an empty head loads at once, so m_n <= IQ - 1 (checked in simulation) and the entry at iwp holds
    // no command; an entry reaches the head (and the IP) only after a command was written into it (m_n counts it).
    // So the grant is no part of the 36 entry-lane enables (timing: grant -> ien, a net along
    // the IP's data rows, failed clk_x1 by 0.69 ns).
    reg  [1:0]   iwp = 2'd0;
    (* syn_preserve = 1 *) reg [8:0] ien0 = 9'd0;
    (* syn_preserve = 1 *) reg [8:0] ien1 = 9'd0;
    (* syn_preserve = 1 *) reg [8:0] ien2 = 9'd0;
    (* syn_preserve = 1 *) reg [8:0] ien3 = 9'd0;
    // (read asynchronously: registers; GowinSynthesis would take them for SSRAM, which GW5A lacks: RP0007)
    (* syn_ramstyle = "registers" *) reg [29:0]  iq_c [0:IQ-1];   // {write, address}
    (* syn_ramstyle = "registers" *) reg [255:0] iq_d [0:IQ-1];
    (* syn_ramstyle = "registers" *) reg [31:0]  iq_m [0:IQ-1];
    reg  [IQ-1:0] iq_w = {IQ{1'b0}};                         // each entry's write flag
    wire [1:0]   iwp_n = iwp + (ao_v ? 2'd1 : 2'd0);         // the entry the next granted command goes to (registers)
    always @(posedge clk) begin : iqdata                      // (no reset: an entry counts only once written)
        integer j;
        ien0 <= {9{iwp_n == 2'd0}}; ien1 <= {9{iwp_n == 2'd1}};
        ien2 <= {9{iwp_n == 2'd2}}; ien3 <= {9{iwp_n == 2'd3}};
        for (j = 0; j < 8; j = j + 1) begin
            if (ien0[j]) begin iq_d[0][32 * j +: 32] <= ao_d[32 * j +: 32]; iq_m[0][4 * j +: 4] <= ao_m[4 * j +: 4]; end
            if (ien1[j]) begin iq_d[1][32 * j +: 32] <= ao_d[32 * j +: 32]; iq_m[1][4 * j +: 4] <= ao_m[4 * j +: 4]; end
            if (ien2[j]) begin iq_d[2][32 * j +: 32] <= ao_d[32 * j +: 32]; iq_m[2][4 * j +: 4] <= ao_m[4 * j +: 4]; end
            if (ien3[j]) begin iq_d[3][32 * j +: 32] <= ao_d[32 * j +: 32]; iq_m[3][4 * j +: 4] <= ao_m[4 * j +: 4]; end
        end
        if (ien0[8]) begin iq_c[0] <= {ao_w, ao_a}; iq_w[0] <= ao_w; end
        if (ien1[8]) begin iq_c[1] <= {ao_w, ao_a}; iq_w[1] <= ao_w; end
        if (ien2[8]) begin iq_c[2] <= {ao_w, ao_a}; iq_w[2] <= ao_w; end
        if (ien3[8]) begin iq_c[3] <= {ao_w, ao_a}; iq_w[3] <= ao_w; end
    end
    // ---- the head register: the IP's command, address, write data and mask come straight from registers (timing: irp
    // -> the 4:1 entry select -> the IP's wr_buf_din failed or nearly failed with every placement: +0.85 ns in one build,
    // +1.88 in a Genesis core build, +0.061 in a Genesis debug build).  The head is loaded from entry irp (a registered
    // pointer: the select ends at these registers, placed by the queue) when the queue has an entry and the head is
    // empty or firing; its load enable is copied per lane (below).  The IP then sees only register outputs.
    // A command reaches the IP a clock later than before (grant -> ao -> entry -> head); the credits (cr, IQ = 4)
    // cover the three stages, so back-to-back commands still issue one a clock.
    // The load enable's copies (timing: a wide enable comes from registers, one copy per lane): syn_keep on the hload
    // WIRES did not keep them -- GowinSynthesis merged the nine identical LUTs into one net (hload_0_s1, fanout ~325).
    // TG16 core builds failed clk_x1 on that one net's route, -0.151 ns to a far lane's hd_d CE and -0.081 ns
    // through m_nn to m_ne (a 4.1 ns hop), while another placement of the same netlist had the same path
    // at +1.3 ns: its two load groups pull the one LUT opposite ways -- the 256-bit hd_d / hd_m lane enables along
    // the IP's data rows, and the queue's counter and pointers.  So every copy now reads its own REGISTERS: m_ne,
    // hd_vl and hd_wl hold m_n != 0, hd_v and hd_w once per lane (syn_preserve, which this synthesizer does honour
    // on registers: ien0..3 above, tango_ddr3_vport's b_push / lo_have), each lane's copies are updated through that
    // lane's own enable, and no two lanes share a LUT input, so nothing is left to merge.  Lanes: 0..7 the data lanes
    // (hd_d / hd_m, 36 enables each), 8 the command lane (hd_a, hd_w, hd_v: what the IP's command side reads), 9 the
    // control lane (irp, m_n: the queue side -- a first draft of this change gave hd_a and the pointers one
    // lane, and its worst remaining hload path was exactly that lane's, hload_8 -> hd_a with a 3.7 ns hop).  Every
    // copy equals its original every clock (same reset, same update, equal inputs by induction: checked below in
    // simulation); hd_v and hd_w themselves still drive fire and the IP, combinational on cmd_ready / wr_data_rdy
    // as before (a cmd_en while cmd_ready is low would be a dropped command in the IP).
    reg  [2:0]   m_n = 3'd0;                                  // entries in the queue's memory (not the head)
    reg          hd_v = 1'b0, hd_w = 1'b0;                      // the head holds a command; a write
    reg  [28:0]  hd_a = 29'd0;
    reg  [255:0] hd_d = 256'd0;
    reg  [31:0]  hd_m = 32'd0;
    (* syn_preserve = 1 *) reg [9:0] m_ne  = 10'd0;           // m_n != 0, a register: one copy per lane
    (* syn_preserve = 1 *) reg [9:0] hd_vl = 10'd0;           // hd_v, one copy per lane
    (* syn_preserve = 1 *) reg [9:0] hd_wl = 10'd0;           // hd_w, one copy per lane (no reset, as hd_w)
    // The queue's read pointer, one copy per lane, each advanced by its own lane's hload (timing:
    // one pointer selected all ~320 head bits, a 315-load net; a 6.9 ns route from it into the hd_w select failed clk_x1
    // by 0.046 ns).  Lane k's head bits are selected by lane k's copy; irp is the control lane's.
    (* syn_preserve = 1 *) reg [19:0] irp_l = 20'd0;          // lane k's copy at [2k +: 2]
    wire [1:0]   irp = irp_l[19:18];
    wire         fire = hd_v && cmd_ready && (!hd_w || wr_data_rdy);
    wire [9:0]   hload;                                       // the head loads: lane k's copy (0..7 data, 8 command, 9 control)
    wire [9:0]   m_ne_n, hd_vl_n;                             // the copies' next state, each from its own lane
    genvar hl;
    generate for (hl = 0; hl < 10; hl = hl + 1) begin : g_hload
        wire       fire_l = hd_vl[hl] && cmd_ready && (!hd_wl[hl] || wr_data_rdy);        // = fire
        wire [2:0] m_nn_l = m_n + (ao_v ? 3'd1 : 3'd0) - (hload[hl] ? 3'd1 : 3'd0);      // = m_nn (below)
        assign hload[hl]   = m_ne[hl] && (!hd_vl[hl] || (cmd_ready && (!hd_wl[hl] || wr_data_rdy)));
        assign m_ne_n[hl]  = (m_nn_l != 3'd0);
        assign hd_vl_n[hl] = hload[hl] ? 1'b1 : (fire_l ? 1'b0 : hd_vl[hl]);
    end endgenerate
    always @(posedge clk) begin : head                        // (no reset: hd_v qualifies it)
        integer j;
        for (j = 0; j < 8; j = j + 1)
            if (hload[j]) begin hd_d[32 * j +: 32] <= iq_d[irp_l[2 * j +: 2]][32 * j +: 32]; hd_m[4 * j +: 4] <= iq_m[irp_l[2 * j +: 2]][4 * j +: 4]; end
        if (hload[8]) begin hd_a <= iq_c[irp_l[17:16]][28:0]; hd_w <= iq_w[irp_l[17:16]]; end
        for (j = 0; j < 10; j = j + 1)
            if (hload[j]) hd_wl[j] <= iq_w[irp_l[2 * j +: 2]];  // hd_w's copies, loaded as hd_w is (each through its lane's pointer)
    end
    assign cmd_en       = fire;
    assign cmd          = hd_w ? 3'b000 : 3'b001;
    assign addr         = hd_a;
    assign wr_data_en   = fire && hd_w;
    assign wr_data      = hd_d;
    assign wr_data_mask = hd_m;
    wire [2:0]   m_nn = m_n + (ao_v ? 3'd1 : 3'd0) - (hload[9] ? 3'd1 : 3'd0);   // the control lane's copy
    integer      hli;
    always @(posedge clk) begin
        if (rst) begin iwp <= 2'd0; irp_l <= 20'd0; m_n <= 3'd0; m_ne <= 10'd0; hd_v <= 1'b0; hd_vl <= 10'd0; cr <= IQ; cr_ok <= (IQ != 0); end
        else begin
            if (ao_v) iwp <= iwp + 1'b1;
            for (hli = 0; hli < 10; hli = hli + 1)
                if (hload[hli]) irp_l[2 * hli +: 2] <= irp_l[2 * hli +: 2] + 2'd1;   // each lane's copy by its own load
            m_n  <= m_nn;
            m_ne <= m_ne_n;                                    // each lane's (m_nn != 0)
            hd_v   <= hload[8] ? 1'b1 : (fire ? 1'b0 : hd_v);  // the command lane's copy
            hd_vl  <= hd_vl_n;                                 // each lane's hd_v
            cr   <= cr - (grant ? 3'd1 : 3'd0) + (fire ? 3'd1 : 3'd0);
            // = (the cr above) != 0: -1 leaves 0 only from 1, +1 only from 7, -1+1 only from 0
            cr_ok <= fire ? (grant ? (cr != 3'd0) : (cr != 3'd7)) : (grant ? (cr != 3'd1) : (cr != 3'd0));
        end
    end
    // The copies' equivalence, checked in simulation every clock: each lane's hload, m_ne, hd_vl and hd_wl equal the
    // originals (m_ne's original is m_n != 0: both are registered from the same m_nn with the same reset).  A
    // difference prints "TANGO CONTRACT: DDR3 head-load copies ..." and stops the simulation with $fatal; the end of a
    // run prints the clocks checked.  Simulation only: GowinSynthesis skips the code between the two pragma comments
    // (SUG550-2.0.1E section 5.19, the block-comment form, as tango_video.v's contract checks); Verilator compiles it.
    /* synthesis translate_off */
    integer hlc_clocks = 0;
    always @(posedge clk) begin
        if (m_n > IQ - 1) begin
            $display("TANGO CONTRACT: DDR3 IP queue full at %0t (%m): m_n %0d (the entry at iwp would hold a command)", $time, m_n);
            $fatal(1, "TANGO CONTRACT: DDR3 IP queue entry at the write pointer not free (see the line above)");
        end
        if (hload != {10{hload[0]}} || m_ne != {10{m_n != 3'd0}} || hd_vl != {10{hd_v}} || hd_wl != {10{hd_w}} || irp_l != {10{irp}}) begin
            $display("TANGO CONTRACT: DDR3 head-load copies differ at %0t (%m): hload %b, m_ne %b (m_n %0d), hd_vl %b (hd_v %b), hd_wl %b (hd_w %b), irp_l %b",
                     $time, hload, m_ne, m_n, hd_vl, hd_v, hd_wl, hd_w, irp_l);
            $fatal(1, "TANGO CONTRACT: DDR3 head-load copies violated (see the line above)");
        end
        hlc_clocks <= hlc_clocks + 1;
    end
    final
        $display("TANGO CONTRACT OK: DDR3 head-load copies (hload, m_ne, hd_vl, hd_wl, irp x 10 lanes) equal to the originals, the entry at iwp free, in %0d clocks (%m)", hlc_clocks);
    // The grant's copies: every group's tags_ok / core_room / vlim and its gv / gc / gf / cany / csel equal group 0's, and
    // the registered flags equal what they stand for (cr_ok = cr != 0, pend_nz = pend != 0, ft_ge = fence_t >= FENCE_WAIT),
    // every clock.  "TANGO CONTRACT: DDR3 grant copies ..." and $fatal on a difference.
    integer grc_clocks = 0;
    always @(posedge clk) begin
        if (rst_qv !== rst || rst_qc !== {NC{rst}}) begin
            $display("TANGO CONTRACT: DDR3 queue resets differ from rst at %0t (%m): rst %b rst_qv %b rst_qc %b", $time, rst, rst_qv, rst_qc);
            $fatal(1, "TANGO CONTRACT: DDR3 queue reset copies violated (see the line above)");
        end
        if (tags_ok_g != {NG{tags_ok_g[0]}} || core_room_g != {NG{core_room_g[0]}} || vlim_g != {NG{vlim_g[0]}}
            || gv_g != {NG{gv_g[0]}} || gc_g != {NG{gc_g[0]}} || gf_g != {NG{gf_g[0]}} || cany_g != {NG{cany_g[0]}}
            || csel_g != {NG{csel_g[PW-1:0]}}
            || cr_ok !== (cr != 3'd0) || pend_nz !== (pend != 8'd0) || ft_ge !== (fence_t >= FENCE_WAIT)) begin
            $display("TANGO CONTRACT: DDR3 grant copies differ at %0t (%m): tags_ok %b core_room %b vlim %b gv %b gc %b gf %b cany %b | cr_ok %b cr %0d pend_nz %b pend %0d ft_ge %b fence_t %0d",
                     $time, tags_ok_g, core_room_g, vlim_g, gv_g, gc_g, gf_g, cany_g, cr_ok, cr, pend_nz, pend, ft_ge, fence_t);
            $fatal(1, "TANGO CONTRACT: DDR3 grant copies violated (see the line above)");
        end
        grc_clocks <= grc_clocks + 1;
    end
    final
        $display("TANGO CONTRACT OK: DDR3 grant copies (%0d groups) and the registered flags cr_ok, pend_nz, ft_ge in %0d clocks (%m)", NG, grc_clocks);
    /* synthesis translate_on */

    // ---- bookkeeping (each register from its group's copy of the grant)
    wire          gv_t = gv_g[G_TAG], gc_t = gc_g[G_TAG], gf_t = gf_g[G_TAG];
    wire [PW-1:0] csel_t = csel_g[G_TAG * PW +: PW];
    wire          g_read = (gv_t && !hv_write) || (gc_t && !hc_write[csel_t]) || gf_t;   // a read granted
    // What the grant selects among is computed from registers (timing: gc ended in the carry
    // chain of inflight + gc -- four LUT levels after the grant on the longest clk_x1 path, -0.534 ns in a launcher
    // build).  Each candidate is a net of its own (syn_keep, the comment form, as tango_ddr3_fifo.v: without
    // it synthesis folds the late bit back into the adder); the grant only selects.
    wire [7:0]    pend_p1 /* synthesis syn_keep = 1 */ = pend + 8'd1;
    wire [7:0]    ret_n  = gc_t ? pend_p1 : pend;
    wire [PW+7:0] tag_new = {gv_t ? {PW{1'b0}} : gc_t ? (csel_t + 1'b1) : P_FENCE, ret_n};
    // a granted read's tag is written into tagm a clock later (timing: the grant is a long path).  The head tag (the
    // next return's) is tagm[next tr], read every clock (a block RAM's registered read), or the tag written into that
    // place at the read's own edge (forwarded: a read during a write at one address is undefined on the BSRAM,
    // no_rw_check); a later write there is read a clock later.  A read's word comes back far later than 2 clocks
    // after its grant (the IP's latency), so the head is always current when it is used.
    reg  [PW+7:0] tag_wq = 0, tag_rd = 0, fwd_d = 0;
    reg  [TAG_LOG2-1:0] tag_wa = 0;
    reg           tag_we = 1'b0, fwd_q = 1'b0;
    wire [TAG_LOG2-1:0] tr_n = tr[TAG_LOG2-1:0] + (rd_data_valid ? 1'b1 : 1'b0);
    wire [PW+7:0] tag_h = fwd_q ? fwd_d : tag_rd;
    // the core commands a return retires, counted a clock after the return (registered: timing; a Template DDR3
    // build failed by 0.88 ns on the IP's rd_data_valid -> inflight): core_room is conservative by a clock
    reg           rdv_q = 1'b0;
    reg  [7:0]    ret_q = 8'd0;
    wire [15:0]   inflight_r /* synthesis syn_keep = 1 */ = inflight - (rdv_q ? {8'd0, ret_q} : 16'd0);   // after the last clock's return
    wire [16:0]   infl_p1 /* synthesis syn_keep = 1 */ = {1'b0, inflight_r} + 17'd1;             // ... and one more core command
    wire [15:0]   inflight_n = gc_g[G_INF] ? infl_p1[15:0] : inflight_r;
    wire [7:0]    vs_p1 /* synthesis syn_keep = 1 */ = vstreak + 8'd1;
    wire          gv_s = gv_g[G_STR], gc_s = gc_g[G_STR];
    wire [7:0]    vstreak_n  = gc_s ? 8'd0 : gv_s ? (cany_g[G_STR] ? vs_p1 : 8'd0) : vstreak;
    // the room for a core command and the V streak limit after this clock, each a pair of compares of registers
    // (+ this clock's grant or not), the grant choosing
    wire          room_gc  /* synthesis syn_keep = 1 */ = (infl_p1 < Q_CORE);       // gc: one more core command
    wire          room_ngc /* synthesis syn_keep = 1 */ = ({1'b0, inflight_r} < Q_CORE);
    wire          lim_inc  /* synthesis syn_keep = 1 */ = (vs_p1 >= G);             // gv with a core command able to go
    wire          lim_hold /* synthesis syn_keep = 1 */ = (vstreak >= G);           // neither
    wire          lim_clr  = (8'd0 >= G);                                           // gc, or gv without one
    wire [15:0]   ft_p1 /* synthesis syn_keep = 1 */ = fence_t + 16'd1;
    reg  [31:0]   vwait = 0;
    // the copies: each group's from its own grant
    integer cg;
    always @(posedge clk) begin
        if (rst) begin tags_ok_g <= {NG{1'b1}}; core_room_g <= {NG{1'b1}}; vlim_g <= {NG{1'b0}}; end
        else for (cg = 0; cg < NG; cg = cg + 1) begin
            tags_ok_g[cg]   <= (tw - tr) < TN - 2;               // conservative: as if this clock granted and none returned
            core_room_g[cg] <= gc_g[cg] ? room_gc : room_ngc;    // exact; gc chooses last
            vlim_g[cg]      <= gc_g[cg] ? lim_clr : gv_g[cg] ? (cany_g[cg] ? lim_inc : lim_clr) : lim_hold;   // = vstreak_n >= G
        end
    end
    always @(posedge clk) begin
        tag_we <= g_read; tag_wa <= tw[TAG_LOG2-1:0]; tag_wq <= tag_new;
        if (tag_we) tagm[tag_wa] <= tag_wq;
        tag_rd <= tagm[tr_n];
        fwd_q <= tag_we && (tag_wa == tr_n); fwd_d <= tag_wq;
        rdv_q <= rd_data_valid; ret_q <= tag_h[7:0];
        if (rd_data_valid) r_data <= rd_data;                // (no reset: r_valid qualifies it)
        r_valid <= 0;
        if (rst) begin
            tw <= 0; tr <= 0; pend <= 0; inflight <= 0; fence_t <= 0; vstreak <= 0; rr <= 0; vwait <= 0;
            pend_nz <= 1'b0; ft_ge <= (16'd0 >= FENCE_WAIT);
        end else begin
            if (g_read) tw <= tw + 1'b1;
            if (rd_data_valid) begin
                tr <= tr + 1'b1;
                if (tag_h[PW+7:8] != P_FENCE) r_valid[tag_h[PW+7:8]] <= 1'b1;
            end
            // core commands in the IP: + granted, - retired by a returning read
            inflight <= inflight_n;
            // pend: 0 after a read, +1 after a core command (candidates from registers, the grant selects)
            pend    <= g_read ? 8'd0 : gc_t ? pend_p1 : pend;
            pend_nz <= g_read ? 1'b0 : gc_t ? (pend != 8'hFF) : (pend != 8'd0);
            if (g_read || !pend_nz) begin fence_t <= 16'd0; ft_ge <= (16'd0 >= FENCE_WAIT); end
            else begin fence_t <= ft_p1; ft_ge <= (ft_p1 >= FENCE_WAIT); end
            vstreak <= vstreak_n;
            if (gc_s) rr <= (csel_g[G_STR * PW +: PW] + 1'b1 == NC) ? {PW{1'b0}} : csel_g[G_STR * PW +: PW] + 1'b1;
            if (gf_t) obs_fences <= obs_fences + 1'b1;
            if (hv_valid && !gv_g[G_QV]) vwait <= vwait + 1'b1; else vwait <= 0;
            if (vwait > obs_v_wait_max) obs_v_wait_max <= vwait;
            if (inflight > obs_core_inflight_max) obs_core_inflight_max <= inflight;
            obs_core_inflight <= inflight;
        end
    end
endmodule
`default_nettype wire
