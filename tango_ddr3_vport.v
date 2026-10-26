// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_vport -- the scaler's DDR3 port: ascal's Avalon MM master (N_DW 128, N_AW 28, as
// MiSTer's vbuf: Template_MiSTer sys/sys_top.v 739-741) onto tango_ddr3_arb, in the IP's clk_out domain (ascal's
// avl_clk, as MiSTer's clk_100m).  It never makes ascal's state machine wait on the memory:
//   - commands are taken at once into an in-order command FIFO (2^CQ_LOG2 = 16: posted bursts + 2 + margin); a
//     write is taken only with room for a whole burst in the posting FIFO, then streams one beat a
//     clock (ascal's sWRITE, ascal.vhd 1710-1717); avl_waitrequest depends on the FIFOs' levels only;
//   - write beats are paired into 32-B IP words (beat at an even 16-B address = bits 127:0) in a posting FIFO of
//     2^WQ_LOG2 words (64 = 8 of ascal's 256-B bursts);
//   - the splitter turns the head command into IP words, strictly in order (so a read after a write to the same
//     buffer goes after it); an odd start or end masks the unused half;
//   - reads go out only with room in the return buffer (2^RB_LOG2 = 32 words; ascal has at most 2 bursts = 16 words in
//     flight, ascal.vhd 2054, so the room test -- a register, conservative by one word -- never holds it back), which
//     turns each 256-bit word into two 128-bit beats, one a clock, in order;
//   - writes outside [WIN_LO, WIN_HI) are dropped (every byte masked) and counted: the scaler's region;
//     reads may go anywhere (MiSTer's framebuffer mode).
// No read-ahead (MiSTer has none).
//==============================================================================
`default_nettype none
module tango_ddr3_vport #(
    parameter integer CQ_LOG2  = 4,
    parameter integer WQ_LOG2  = 6,
    parameter integer RB_LOG2  = 5,
    parameter integer MAXBURST = 16,
    parameter [31:0]  WIN_LO   = 32'h2000_0000,
    parameter [31:0]  WIN_HI   = 32'h2200_0000
) (
    input  wire          clk,
    input  wire          rst,
    // ascal
    input  wire [27:0]   avl_address,
    input  wire [7:0]    avl_burstcount,
    input  wire          avl_write,
    input  wire [127:0]  avl_writedata,
    input  wire [15:0]   avl_byteenable,
    input  wire          avl_read,
    output wire          avl_waitrequest,
    output reg  [127:0]  avl_readdata = 128'd0,
    output reg           avl_readdatavalid = 1'b0,
    // tango_ddr3_arb
    output reg           q_valid = 1'b0,
    output reg           q_write = 1'b0,
    output reg  [28:0]   q_addr = 29'd0,
    output reg  [255:0]  q_wdata = 256'd0,
    output reg  [31:0]   q_wmask = 32'd0,
    input  wire          q_ready,
    input  wire          r_valid,
    input  wire [255:0]  r_data,
    // observation
    output reg  [31:0]   obs_wbursts = 0,
    output reg  [31:0]   obs_rbursts = 0,
    output reg  [31:0]   obs_win_drops = 0,
    output reg  [15:0]   obs_wait_max = 0,      // longest avl_waitrequest with a command present (clocks)
    output reg  [7:0]    obs_wq_max = 0,
    output reg  [7:0]    obs_cq_max = 0
);
    // ================= the Avalon side =================
    wire [CQ_LOG2:0] cq_n;  wire cq_full;
    wire [WQ_LOG2:0] wq_n;  wire wq_full;
    localparam integer WQ_RES = MAXBURST / 2 + 3;          // IP words a burst can need (odd start) + the one
                                                           // still in the pairing register when the next comes
                                                           // + the one in the beat register (below)
    reg        inb = 1'b0;                                 // in a write burst
    reg [7:0]  wleft = 8'd0;
    reg [27:0] wa = 28'd0;                                 // address of the next beat
    // room for a command: a register, conservative by one (each count grows by at most one a clock) (timing).
    // None in reset: rst is high until the IP has calibrated (tango_sys), and ascal must wait for it with
    // waitrequest -- without this the FIFOs, held in reset, showed room, so commands and beats
    // were taken and dropped (sim/tb_ascal_ddr3.v: ascal's first write burst and reads before calibration)
    reg  can_cmd = 1'b0;
    always @(posedge clk) can_cmd <= !rst && (cq_n + 1'b1 < (1 << CQ_LOG2)) && (wq_n + 1'b1 + WQ_RES <= (1 << WQ_LOG2));
    assign avl_waitrequest = inb ? 1'b0 : !can_cmd;
    wire take_cmd = !inb && (avl_write || avl_read) && can_cmd;
    wire take_beat = avl_write && (inb || can_cmd);         // beat 0 comes with the command
    wire [27:0] ba = inb ? wa : avl_address;                // this beat's address
    wire [7:0]  bl = inb ? wleft : avl_burstcount;          // beats left including this one
    always @(posedge clk) begin
        if (rst) inb <= 1'b0;
        else if (take_beat) inb <= (bl != 8'd1);
        if (take_beat) begin wa <= ba + 1'b1; if (bl != 8'd1) wleft <= bl - 1'b1; end
    end
    // ---- the beats into IP words, in two stages (timing: a Template + TANGO_VIDEO_PROC DDR3 build
    // failed clk_x1 by 0.44 ns on wleft -> "last beat?" -> the enable of the 288-bit posting register, a 6 ns route).
    // Stage 1 registers the beat and its decisions; stage 2 pairs beats into words from registers only.  Every select
    // and enable of the wide registers is a register of its own, one copy per 32-bit lane (syn_preserve).
    // A beat at an even 16-B address is the low half (bits 127:0) of its word, at an odd one the high half; a word is
    // pushed with its high half, or with a low half that ends the burst.
    reg  [127:0] b_d = 128'd0;                              // stage 1: the beat (no enable: b_v qualifies it)
    reg  [15:0]  b_be = 16'd0;
    reg          b_v = 1'b0, b_odd = 1'b0;
    (* syn_preserve = 1 *) reg [7:0] b_push = 8'd0;        // per lane: stage 2 pushes a word
    (* syn_preserve = 1 *) reg [7:0] b_lo = 8'd0;          // per lane: stage 2 keeps a low half (even, not last)
    (* syn_preserve = 1 *) reg [7:0] b_oddl = 8'd0;        // per lane: the beat is a high half
    always @(posedge clk) begin
        b_d <= avl_writedata; b_be <= avl_byteenable;
        b_v <= take_beat && !rst; b_odd <= ba[0];
        b_push <= {8{take_beat && !rst && (ba[0] || bl == 8'd1)}};
        b_lo   <= {8{take_beat && !rst && !ba[0] && bl != 8'd1}};
        b_oddl <= {8{ba[0]}};
    end
    reg  [127:0] lo_d = 128'd0;                             // stage 2: the pairing register
    reg  [15:0]  lo_be = 16'd0;
    (* syn_preserve = 1 *) reg [7:0] lo_have = 8'd0;       // per lane: lo_d holds a low half
    reg          wq_we = 1'b0;
    reg  [287:0] wq_wd = 288'd0;                            // {mask[31:0], data[255:0]}, address in cq
    always @(posedge clk) begin
        wq_we <= b_push[0];
        if (rst) lo_have <= 8'd0;
        else if (b_v) lo_have <= {8{!b_odd && !b_push[0]}};
    end
    genvar k;
    generate for (k = 0; k < 4; k = k + 1) begin : g_lane
        // lane k of the low half (bits 32k+31:32k) and of the high half (128+32k+31:128+32k), and their masks
        always @(posedge clk) begin
            if (b_lo[k]) begin lo_d[32*k +: 32] <= b_d[32*k +: 32]; lo_be[4*k +: 4] <= b_be[4*k +: 4]; end
            // low half: an even beat's data, or (an odd beat) the low half kept for it, or none (masked)
            wq_wd[32*k +: 32]       <= b_oddl[k] ? (lo_have[k] ? lo_d[32*k +: 32] : 32'd0) : b_d[32*k +: 32];
            wq_wd[256 + 4*k +: 4]   <= b_oddl[k] ? (lo_have[k] ? ~lo_be[4*k +: 4] : 4'hF) : ~b_be[4*k +: 4];
            // high half: an odd beat's data, else none (masked)
            wq_wd[128 + 32*k +: 32] <= b_oddl[4+k] ? b_d[32*k +: 32] : 32'd0;
            wq_wd[272 + 4*k +: 4]   <= b_oddl[4+k] ? ~b_be[4*k +: 4] : 4'hF;
        end
    end endgenerate
    // the command FIFO: {write, address[27:0], burstcount[7:0]}
    wire [36:0] cq_q;  wire cq_v;  wire cq_pop;
    tango_ddr3_fifo #(.W(37), .AW(CQ_LOG2)) u_cq (.clk(clk), .rst(rst), .we(take_cmd),
        .wd({avl_write, avl_address, avl_burstcount}), .count(cq_n), .full(cq_full), .q(cq_q), .q_valid(cq_v), .pop(cq_pop));
    wire [287:0] wq_q;  wire wq_v;  wire wq_pop;
    tango_ddr3_fifo #(.W(288), .AW(WQ_LOG2)) u_wq (.clk(clk), .rst(rst), .we(wq_we), .wd(wq_wd), .count(wq_n),
        .full(wq_full), .q(wq_q), .q_valid(wq_v), .pop(wq_pop));
    // waitrequest held with a command present
    reg [15:0] wcnt = 0;
    always @(posedge clk) begin
        if (!inb && (avl_write || avl_read) && !can_cmd) wcnt <= wcnt + 1'b1; else wcnt <= 0;
        if (wcnt > obs_wait_max) obs_wait_max <= wcnt;
        if (cq_n > obs_cq_max) obs_cq_max <= cq_n;
        if (wq_n > obs_wq_max) obs_wq_max <= wq_n;
        if (take_cmd && avl_write) obs_wbursts <= obs_wbursts + 1'b1;
        if (take_cmd && avl_read)  obs_rbursts <= obs_rbursts + 1'b1;
    end

    // ================= the splitter =================
    // returns' halves, in issue order: {lo_en, hi_en}
    wire [1:0] ri_q;  wire ri_v;  reg ri_we = 1'b0;  reg [1:0] ri_wd = 2'b11;
    wire [RB_LOG2+1:0] ri_n;  wire ri_full;
    tango_ddr3_fifo #(.W(2), .AW(RB_LOG2 + 1)) u_ri (.clk(clk), .rst(rst), .we(ri_we), .wd(ri_wd), .count(ri_n),
        .full(ri_full), .q(ri_q), .q_valid(ri_v), .pop(r_valid));
    wire [RB_LOG2:0]   rb_n;
    reg  [RB_LOG2+1:0] rb_out = 0;                           // reads taken by the arbiter, word not yet back
    reg        sp_w = 1'b0;                                  // the command in the splitter: write?
    reg [27:0] sa = 28'd0;                                   // its next beat address
    reg [8:0]  sn = 9'd0;                                    // beats left (0 = the splitter is empty)
    // sn != 0 and sn == 1 as registers (timing: sn -> "empty / one beat left?" -> w_go -> the posting FIFO's pop was
    // the worst path of earlier Template DDR3 builds): next values from registers, selected by the late go
    reg        sn_nz = 1'b0, sn_one = 1'b0;
    wire [8:0] sn_m1 = sn - 9'd1, sn_m2 = sn - 9'd2;
    function automatic win(input [27:0] a);
        win = ({a, 4'b0} >= WIN_LO) && ({a, 4'b0} < WIN_HI);
    endfunction
    wire q_free  = !q_valid || q_ready;                      // q can take the next request this clock
    // room for a read's word: the words reserved (out at the IP, in the buffer, a read in q) grow by at most one a
    // clock (a read into q), so a register of "room for one more" is safe a clock later (timing: the return buffer's
    // count -> the splitter's issue failed tp2 of a build by 1.46 ns)
    reg  rb_ok = 1'b1;
    always @(posedge clk) rb_ok <= (rb_out + rb_n + ((q_valid && !q_write) ? 1'b1 : 1'b0) + 1'b1) < (1 << RB_LOG2);
    wire w_go    = q_free && sn_nz && sp_w && wq_v;
    // room in the return-info FIFO: a register, conservative by two (a read a clock, and its entry written a clock
    // later); rb_ok keeps it far from full anyway (timing)
    reg  ri_ok = 1'b1;
    always @(posedge clk) ri_ok <= (ri_n + 2'd2) < (1 << (RB_LOG2 + 1));
    wire r_go    = q_free && sn_nz && !sp_w && rb_ok && ri_ok;
    wire one     = sa[0] || sn_one;                          // this IP word holds one beat
    assign wq_pop = w_go;
    assign cq_pop = cq_v && !sn_nz;
    always @(posedge clk) begin
        ri_we <= 1'b0;
        if (rst) begin sn <= 9'd0; sn_nz <= 1'b0; sn_one <= 1'b0; q_valid <= 1'b0; rb_out <= 0; end
        else begin
            rb_out <= rb_out + ((q_valid && q_ready && !q_write) ? 1'b1 : 1'b0) - (r_valid ? 1'b1 : 1'b0);
            if (q_free) begin
                if (w_go) begin
                    q_valid <= 1'b1; q_write <= 1'b1;
                    if (!win(sa)) obs_win_drops <= obs_win_drops + 1'b1;
                end else if (r_go) begin
                    q_valid <= 1'b1; q_write <= 1'b0;
                    ri_we <= 1'b1; ri_wd <= {!sa[0], !(!sa[0] && sn_one)};
                end else q_valid <= 1'b0;
            end
            if (w_go || r_go) begin
                if (one) begin sa <= sa + 1'b1; sn <= sn_m1; sn_nz <= (sn_m1 != 9'd0); sn_one <= (sn_m1 == 9'd1); end
                else begin sa <= sa + 2'd2; sn <= sn_m2; sn_nz <= (sn_m2 != 9'd0); sn_one <= (sn_m2 == 9'd1); end
            end else if (cq_pop) begin
                sp_w <= cq_q[36]; sa <= cq_q[35:8]; sn <= {1'b0, cq_q[7:0]};
                sn_nz <= (cq_q[7:0] != 8'd0); sn_one <= (cq_q[7:0] == 8'd1);
            end
        end
    end

    // the request's data path: no reset (q_valid qualifies it), so the reset does not gate 300 enables (timing)
    always @(posedge clk) begin
        if (w_go) begin
            q_addr <= {1'b0, sa[25:1], 3'b000}; q_wdata <= wq_q[255:0];
            q_wmask <= win(sa) ? wq_q[287:256] : 32'hFFFF_FFFF;
        end else if (r_go) q_addr <= {1'b0, sa[25:1], 3'b000};
    end

    // ================= the return buffer: words in, beats out (one a clock) =================
    wire [257:0] rb_q;  wire rb_v;
    reg  lo_done = 1'b0;
    wire rb_lo  = rb_v && rb_q[257] && !lo_done;             // the low half goes now
    wire rb_pop = rb_v && (rb_lo ? !rb_q[256] : 1'b1);       // the word's last half goes now
    tango_ddr3_fifo #(.W(258), .AW(RB_LOG2)) u_rb (.clk(clk), .rst(rst), .we(r_valid), .wd({ri_q, r_data}),
        .count(rb_n), .full(), .q(rb_q), .q_valid(rb_v), .pop(rb_pop));
    always @(posedge clk) begin
        avl_readdatavalid <= rb_v && (rb_lo || rb_q[256]);
        avl_readdata <= rb_lo ? rb_q[127:0] : rb_q[255:128];
        if (rst) lo_done <= 1'b0;
        else if (rb_v) lo_done <= rb_lo && rb_q[256];
    end
endmodule
`default_nettype wire
