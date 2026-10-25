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
// Each port feeds a 2-entry queue whose ready is a register (tango_ddr3_skid_ref): the grant below works on the queues'
// heads and only pops them.  v_ready / c_ready are those registered readies.
// The IP stage (the IP's data registers lie along ~17 rows; what drives them comes from registers):
// the grant decides from registers only -- credits for the IP queue, not the IP's cmd_ready -- and its command and
// data go to an output register beside the arbiter, then into a 4-entry queue (each entry's write enable registered,
// a copy per 32-bit lane) whose head drives the IP, a 4-way select by a register.  The IP takes the head when
// cmd_ready (and wr_data_rdy for a write).  Q_CORE and the read tags count from the grant (the queue is in order).
// Observation outputs (sims and debug): the most core commands in the IP, the longest a V request waited, fences.
//==============================================================================
`default_nettype none
module tango_ddr3_arb_ref #(
    parameter integer        NC         = 1,
    parameter integer        G          = 8,
    parameter integer        Q_CORE     = 16,
    parameter integer        TAG_LOG2   = 6,
    parameter integer        FENCE_WAIT = 64,
    parameter [28:0]         FENCE_ADDR = 29'd0
) (
    input  wire              clk,
    input  wire              rst,
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
    // room for a tag, the core's room in the IP and the V streak limit: registers, computed from the next state
    // (exact), so the grant below is a few gates after cmd_ready
    reg  tags_ok = 1'b1, core_room = 1'b1, vlim = 1'b0;
    reg  [7:0]  pend = 8'd0;                 // core commands granted since the last read
    reg  [15:0] inflight = 16'd0;            // core commands in the IP
    reg  [15:0] fence_t = 16'd0;

    // ---- the ports' queues; the grant works on their heads
    wire         hv_valid;
    wire [317:0] hv;
    wire         hv_pop;
    tango_ddr3_skid_ref #(.W(318)) u_qv (.clk(clk), .rst(rst), .in_valid(v_valid), .in_data({v_write, v_addr, v_wdata, v_wmask}),
        .in_ready(v_ready), .out_valid(hv_valid), .out_data(hv), .out_pop(hv_pop));
    wire         hv_write = hv[317];
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
        tango_ddr3_skid_ref #(.W(318)) u_qc (.clk(clk), .rst(rst), .in_valid(c_valid[g]),
            .in_data({c_write[g], c_addr[g * 29 +: 29], c_wdata[g * 256 +: 256], c_wmask[g * 32 +: 32]}),
            .in_ready(c_ready[g]), .out_valid(hc_valid[g]), .out_data(h), .out_pop(hc_pop[g]));
        assign hc_write[g] = h[317];
        assign hc_addr[g * 29 +: 29] = h[316:288];
        assign hc_wdata[g * 256 +: 256] = h[287:32];
        assign hc_wmask[g * 32 +: 32] = h[31:0];
    end endgenerate

    // ---- who can go this clock (credits for the IP queue: entries free, counting the output register's)
    localparam integer IQ = 4;
    reg  [2:0] cr = IQ;
    wire cr_ok = (cr != 3'd0);
    wire ok_r = cr_ok && tags_ok;
    wire ok_w = cr_ok;
    wire v_can = hv_valid && (hv_write ? ok_w : ok_r);
    wire [NC-1:0] c_can;
    generate for (g = 0; g < NC; g = g + 1) begin : g_can
        assign c_can[g] = hc_valid[g] && core_room && (hc_write[g] ? ok_w : ok_r);
    end endgenerate
    // round-robin among the C ports: the first that can go at or after rr
    reg  [PW-1:0] rr = 0;
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
    reg  [7:0] vstreak = 8'd0;
    wire fence_need = (pend != 0) && ((fence_t >= FENCE_WAIT) || !core_room);
    wire fence_can  = fence_need && ok_r;
    wire gv = v_can && !(cany && vlim);
    wire gc = !gv && cany;
    wire gf = !gv && !gc && fence_can;
    wire grant = gv || gc || gf;
    wire g_write = gv ? hv_write : gc ? hc_write[csel] : 1'b0;
    assign hv_pop = gv;
    generate for (g = 0; g < NC; g = g + 1) begin : g_pop
        assign hc_pop[g] = gc && (csel == g);
    end endgenerate
    // ---- the IP stage: the granted command, registered beside the arbiter -- both candidates every clock and which
    // one was granted (registers), so no grant decision selects the 317 bits (timing: a Template core DDR3 build
    // without it failed by 0.49 ns on tags_ok -> the grant -> ao_d) ...
    reg          ao_v = 1'b0, ao_w = 1'b0, ao_c = 1'b0, ao_f = 1'b0;   // granted; a write; the core's; the fence
    reg  [28:0]  aov_a = 29'd0, aoc_a = 29'd0;
    reg  [255:0] aov_d = 256'd0, aoc_d = 256'd0;
    reg  [31:0]  aov_m = 32'd0, aoc_m = 32'd0;
    always @(posedge clk) begin
        ao_v <= grant && !rst; ao_w <= g_write; ao_c <= gc; ao_f <= gf;
        aov_a <= hv_addr; aov_d <= hv_wdata; aov_m <= hv_wmask;
        aoc_a <= hc_addr[csel * 29 +: 29]; aoc_d <= hc_wdata[csel * 256 +: 256]; aoc_m <= hc_wmask[csel * 32 +: 32];
    end
    wire [28:0]  ao_a = ao_f ? FENCE_ADDR : ao_c ? aoc_a : aov_a;
    wire [255:0] ao_d = ao_c ? aoc_d : aov_d;
    wire [31:0]  ao_m = ao_c ? aoc_m : aov_m;
    // ... then the IP queue: entry k written from the output register when ien[k] (registered a clock ahead from the
    // grant and the write pointer; copies: lanes 0..7 and the command); the head register (below) drives the IP
    reg  [1:0]   iwp = 2'd0, irp = 2'd0;
    (* syn_preserve = 1 *) reg [8:0] ien0 = 9'd0;
    (* syn_preserve = 1 *) reg [8:0] ien1 = 9'd0;
    (* syn_preserve = 1 *) reg [8:0] ien2 = 9'd0;
    (* syn_preserve = 1 *) reg [8:0] ien3 = 9'd0;
    // (read asynchronously: registers; GowinSynthesis would take them for SSRAM, which GW5A lacks: RP0007)
    (* syn_ramstyle = "registers" *) reg [29:0]  iq_c [0:IQ-1];   // {write, address}
    (* syn_ramstyle = "registers" *) reg [255:0] iq_d [0:IQ-1];
    (* syn_ramstyle = "registers" *) reg [31:0]  iq_m [0:IQ-1];
    reg  [IQ-1:0] iq_w = {IQ{1'b0}};                         // each entry's write flag
    wire [1:0]   iwp_n = iwp + (ao_v ? 2'd1 : 2'd0);         // the entry the next granted command goes to
    always @(posedge clk) begin : iqdata                      // (no reset: an entry counts only once written)
        integer j;
        ien0 <= {9{grant && iwp_n == 2'd0}}; ien1 <= {9{grant && iwp_n == 2'd1}};
        ien2 <= {9{grant && iwp_n == 2'd2}}; ien3 <= {9{grant && iwp_n == 2'd3}};
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
    // +1.88 in a Genesis core release, +0.061 in a Genesis debug build).  The head is loaded from entry irp (a registered
    // pointer: the select ends at these registers, placed by the queue) when the queue has an entry and the head is
    // empty or firing; its load enable is copied per lane (below).  The IP then sees only register outputs.
    // A command reaches the IP a clock later than before (grant -> ao -> entry -> head); the credits (cr, IQ = 4)
    // cover the three stages, so back-to-back commands still issue one a clock.
    // The load enable's copies (timing): syn_keep on the hload WIRES did not keep
    // them -- GowinSynthesis merged the nine identical LUTs into one net (hload_0_s1, fanout ~325).  TG16 core builds
    // failed clk_x1 on that one net's route, -0.151 ns to a far lane's hd_d CE in one and -0.081 ns
    // through m_nn to m_ne in another (a 4.1 ns hop), while another placement of the same netlist had the same path
    // at +1.3 ns: its two load groups pull the one LUT opposite ways -- the 256-bit hd_d / hd_m lane enables along
    // the IP's data rows, and the queue's counter and pointers.  So every copy now reads its own REGISTERS: m_ne,
    // hd_vl and hd_wl hold m_n != 0, hd_v and hd_w once per lane (syn_preserve, which this synthesizer does honour
    // on registers: ien0..3 above, tango_ddr3_vport's b_push / lo_have), each lane's copies are updated through that
    // lane's own enable, and no two lanes share a LUT input, so nothing is left to merge.  Lanes: 0..7 the data lanes
    // (hd_d / hd_m, 36 enables each), 8 the command lane (hd_a, hd_w, hd_v: what the IP's command side reads), 9 the
    // control lane (irp, m_n: the queue side -- an earlier draft of this change gave hd_a and the pointers one
    // lane, and its worst remaining hload path was exactly that lane's, hload_8 -> hd_a with a 3.7 ns hop).  Every
    // copy equals its original every clock (same reset, same update, equal inputs by induction: checked below in
    // simulation); hd_v and hd_w themselves still drive fire and the IP, combinational on cmd_ready / wr_data_rdy
    // as before (a cmd_en while cmd_ready is low would be a dropped command, sim/tango_ddr3_model.v).
    reg  [2:0]   m_n = 3'd0;                                  // entries in the queue's memory (not the head)
    reg          hd_v = 1'b0, hd_w = 1'b0;                      // the head holds a command; a write
    reg  [28:0]  hd_a = 29'd0;
    reg  [255:0] hd_d = 256'd0;
    reg  [31:0]  hd_m = 32'd0;
    (* syn_preserve = 1 *) reg [9:0] m_ne  = 10'd0;           // m_n != 0, a register: one copy per lane
    (* syn_preserve = 1 *) reg [9:0] hd_vl = 10'd0;           // hd_v, one copy per lane
    (* syn_preserve = 1 *) reg [9:0] hd_wl = 10'd0;           // hd_w, one copy per lane (no reset, as hd_w)
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
            if (hload[j]) begin hd_d[32 * j +: 32] <= iq_d[irp][32 * j +: 32]; hd_m[4 * j +: 4] <= iq_m[irp][4 * j +: 4]; end
        if (hload[8]) begin hd_a <= iq_c[irp][28:0]; hd_w <= iq_w[irp]; end
        for (j = 0; j < 10; j = j + 1)
            if (hload[j]) hd_wl[j] <= iq_w[irp];                // hd_w's copies, loaded as hd_w is
    end
    assign cmd_en       = fire;
    assign cmd          = hd_w ? 3'b000 : 3'b001;
    assign addr         = hd_a;
    assign wr_data_en   = fire && hd_w;
    assign wr_data      = hd_d;
    assign wr_data_mask = hd_m;
    wire [2:0]   m_nn = m_n + (ao_v ? 3'd1 : 3'd0) - (hload[9] ? 3'd1 : 3'd0);   // the control lane's copy
    always @(posedge clk) begin
        if (rst) begin iwp <= 2'd0; irp <= 2'd0; m_n <= 3'd0; m_ne <= 10'd0; hd_v <= 1'b0; hd_vl <= 10'd0; cr <= IQ; end
        else begin
            if (ao_v) iwp <= iwp + 1'b1;
            if (hload[9]) irp <= irp + 1'b1;                    // the control lane's copy
            m_n  <= m_nn;
            m_ne <= m_ne_n;                                    // each lane's (m_nn != 0)
            hd_v   <= hload[8] ? 1'b1 : (fire ? 1'b0 : hd_v);  // the command lane's copy
            hd_vl  <= hd_vl_n;                                 // each lane's hd_v
            cr   <= cr - (grant ? 3'd1 : 3'd0) + (fire ? 3'd1 : 3'd0);
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
        if (hload != {10{hload[0]}} || m_ne != {10{m_n != 3'd0}} || hd_vl != {10{hd_v}} || hd_wl != {10{hd_w}}) begin
            $display("TANGO CONTRACT: DDR3 head-load copies differ at %0t (%m): hload %b, m_ne %b (m_n %0d), hd_vl %b (hd_v %b), hd_wl %b (hd_w %b)",
                     $time, hload, m_ne, m_n, hd_vl, hd_v, hd_wl, hd_w);
            $fatal(1, "TANGO CONTRACT: DDR3 head-load copies violated (see the line above)");
        end
        hlc_clocks <= hlc_clocks + 1;
    end
    final
        $display("TANGO CONTRACT OK: DDR3 head-load copies (hload, m_ne, hd_vl, hd_wl x 10 lanes) equal to the originals in %0d clocks (%m)", hlc_clocks);
    /* synthesis translate_on */

    // ---- bookkeeping
    wire          g_read = grant && !g_write;
    wire [7:0]    ret_n  = pend + (gc ? 8'd1 : 8'd0);
    wire [PW+7:0] tag_new = {gv ? {PW{1'b0}} : gc ? (csel + 1'b1) : P_FENCE, ret_n};
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
    // the core commands a return retires, counted a clock after the return (registered: timing; a Template core DDR3
    // build without it failed by 0.88 ns on the IP's rd_data_valid -> inflight): core_room is conservative by a clock
    reg           rdv_q = 1'b0;
    reg  [7:0]    ret_q = 8'd0;
    wire [15:0]   inflight_r = inflight - (rdv_q ? {8'd0, ret_q} : 16'd0);   // after the last clock's return
    wire [15:0]   inflight_n = inflight_r + (gc ? 16'd1 : 16'd0);
    wire [7:0]    vstreak_n  = gc ? 8'd0 : gv ? (cany ? vstreak + 1'b1 : 8'd0) : vstreak;
    reg  [31:0]   vwait = 0;
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
            tags_ok <= 1'b1; core_room <= 1'b1; vlim <= 1'b0;
        end else begin
            if (g_read) tw <= tw + 1'b1;
            if (rd_data_valid) begin
                tr <= tr + 1'b1;
                if (tag_h[PW+7:8] != P_FENCE) r_valid[tag_h[PW+7:8]] <= 1'b1;
            end
            // core commands in the IP: + granted, - retired by a returning read
            inflight <= inflight_n;
            tags_ok <= (tw - tr) < TN - 2;                  // conservative: as if this clock granted and none returned
            core_room <= gc ? (inflight_r + 16'd1 < Q_CORE) : (inflight_r < Q_CORE);   // exact; gc chooses last
            vlim <= vstreak_n >= G;
            if (g_read) pend <= 8'd0;
            else if (gc) pend <= pend + 1'b1;
            if (g_read || pend == 0) fence_t <= 0; else fence_t <= fence_t + 1'b1;
            vstreak <= vstreak_n;
            if (gc) rr <= (csel + 1'b1 == NC) ? {PW{1'b0}} : csel + 1'b1;
            if (gf) obs_fences <= obs_fences + 1'b1;
            if (hv_valid && !gv) vwait <= vwait + 1'b1; else vwait <= 0;
            if (vwait > obs_v_wait_max) obs_v_wait_max <= vwait;
            if (inflight > obs_core_inflight_max) obs_core_inflight_max <= inflight;
            obs_core_inflight <= inflight;
        end
    end
endmodule
`default_nettype wire
