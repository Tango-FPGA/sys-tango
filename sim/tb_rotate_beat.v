// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_rotate_beat -- a dedicated, fast unit bench for tango_video_rotate_ddr3's buffer allocator. The full
// real-netlist pipeline (tb_ascal_ddr3.v/sim/run_tb_ascal_ddr3.sh) takes several real minutes per run, and
// drives one fixed-ratio beam -- it never exercises two genuinely independent clocks at the real
// 59.04 Hz-in/60.00 Hz-out ratio. This bench does, and runs in well under a second.
//
// Two real, asynchronous clocks (lockstep clocks cannot catch CDC bugs -- not derived from one another,
// independent periods, a random start-phase offset):
//   clk_video 169 time units; clk_pix 167 time units (both arbitrary, chosen only to be non-commensurate).
// Frame cadence, scaled to the REAL 59.04/60.00 ratio EXACTLY (not approximated): 60.00/59.04 = 125/123
// exactly (gcd(6000,5904)=48; 6000/48=125, 5904/48=123) -- so the input VS period is IN_CYC=125 clk_video
// cycles and the "ascal" stub's own VS period is OUT_CYC=123 clk_pix cycles. Since clk_video and clk_pix
// periods are themselves different (169 vs 167) AND the cycle counts differ (125 vs 123), the real-time beat
// ratio is (125*169)/(123*167) = 21125/20541 = 1.028... -- not exactly 60/59.04, because two plain integer
// periods cannot give the exact 125:123 cycle ratio AND a genuinely async clock period ratio at once.
// Compromise taken: the CYCLE ratio (125:123, exact) drives the frame-level beat this
// bench's checks care about; the clock PERIOD difference (169 vs 167, ~1.2%) is kept only to keep the
// CDC paths (o_fb_* and the ascal stub's own sampling) asynchronous, not to claim a second,
// independent source of the real 59.04/60.00 figure. Both matter for different reasons and neither one alone
// would give both the real beat and asynchronous CDC paths.
//
// DUT: tango_video_rotate_ddr3, small MAXW/MAXH (8x8) so each native frame is a handful of pixels -- this
// bench tests the ALLOCATOR's own state machine (wbuf/rbuf/next_wbuf/the fence), not pixel-level addressing
// (that is sim/tb_rotate_golden.v's job) or real DDR3 timing (sim/tb_rotate_arb.v). The DDRAM_* responder
// below is a trivial fixed-latency stub, not the real IP model -- deliberately so: it isolates the
// allocator's own correctness from DDR3/arbiter timing variance.
//
// Checks, every clock, across >= 200 input frames (more than 3x the beat period, ~61.6 input frames):
//   1. BANK SEPARATION, always: bank(wbuf) != bank(rbuf) (the premise of the bank-aware 4-buffer allocator).
//   2. NO OVERWRITE: at every new-frame allocation, the buffer `next_wbuf` picks is never the current `rbuf`.
//   3. NO TEAR: the "ascal" stub never latches a NEW `fb_base` value mid-way through anything -- it only
//      samples at its own VS edges, and the design's job is that whatever it samples was a value that had
//      already been fully settled: `tango_cdc_word` is a toggle-handshake CDC (its own header: "No
//      multi-bit synchroniser anywhere"), so `o_fb_base` only ever updates as one complete, atomic word --
//      the real "no tear" check is that whatever value the stub samples always decodes to one of the 4
//      actual buffer addresses, never a garbage/metastable one (checked dynamically below, `is_valid_base`).
//   4. THE BEAT ITSELF: counts, from the ascal stub's own sampled sequence of `o_fb_base` values across the
//      run, how many consecutive-equal samples (repeats) vs. genuinely new values (changes) occur, and
//      requires both to occur at least once -- proving the bench actually drove an unlocked, beating
//      relationship, not an accidental lock-step. With the REAL ratio's direction (60.00 Hz output strictly
//      faster than 59.04 Hz input), the output side can only ever repeat or catch a frame exactly once --
//      it can never fall behind far enough to skip one outright, so this bench demonstrates the repeat half
//      of the beat, not a skip; proving a skip would need the opposite (input faster than output), a
//      different scenario, not built here.
// TB_RESULT PASS / FAIL.
//==============================================================================
`default_nettype none
`timescale 1ns/1ps

module tb_rotate_beat;
    localparam integer MAXW = 4, MAXH = 4;        // 16 pixels/frame -- this bench tests the allocator, not
                                                    // pixel-level 2D addressing (sim/tb_rotate_golden.v)
    localparam integer IN_CYC  = 500;             // clk_video cycles per input VS period: 4x the bare
    localparam integer OUT_CYC = 492;             // 125:123 ratio (still EXACTLY 60.00/59.04), scaled up to
                                                    // leave room for a realistic (sparse) CE_DIV below
    localparam integer N_INPUT_FRAMES = 200;      // > 3 beat periods (~61.6 input frames each)

    // ---- two genuinely async clocks (not derived from one another)
    reg clk_video = 1'b0;
    always #84.5 clk_video = ~clk_video;          // period 169
    reg clk_pix = 1'b0;
    initial begin
        #37;                                      // an arbitrary, non-zero start-phase offset
        forever #83.5 clk_pix = ~clk_pix;          // period 167
    end

    // ---- input raster: a tiny MAXWxMAXH active window inside an IN_CYC-cycle frame, at a SPARSE CE_PIXEL
    // (CE_DIV raw clk_video clocks per real pixel tick) -- real cores never run CE_PIXEL at clk_video's own
    // rate (Template is clk_video/12); the client's write FSM needs 2 clk_video
    // cycles/pixel to drain, so CE_DIV=1 would flood the FIFO regardless of any allocator question -- not
    // representative of any real core, and not what this bench is testing. IN_CYC/OUT_CYC are scaled up 4x
    // from the bare 125:123 ratio (still exactly 500:492 = 125:123) to leave room for a realistic CE_DIV.
    localparam integer CE_DIV = 8;
    reg [9:0] hcnt = 0;
    reg [3:0] cediv_cnt = 0;
    reg       ce_pixel_r = 1'b0;
    reg       vga_hs = 0, vga_vs = 0, vga_de = 0;
    reg [7:0] frame_tag = 0;                      // written into every pixel's R channel this frame
    integer   in_frame_no = -1;                   // -1: before the first VS (start-up)
    always @(posedge clk_video) begin
        cediv_cnt  <= (cediv_cnt == CE_DIV - 1) ? 4'd0 : cediv_cnt + 1'b1;
        ce_pixel_r <= (cediv_cnt == CE_DIV - 1);   // one real pixel tick every CE_DIV raw clocks
        if (hcnt == IN_CYC - 1) begin
            hcnt        <= 0;
            in_frame_no <= in_frame_no + 1;
            frame_tag   <= frame_tag + 1'b1;
        end else hcnt <= hcnt + 1'b1;
        vga_vs <= (hcnt < 4);                      // VS pulse at the start of the frame period
        vga_hs <= 1'b0;
        vga_de <= (hcnt >= 10) && (hcnt < 10 + MAXW * MAXH * CE_DIV);  // MAXW*MAXH real pixel ticks, spread out
    end
    wire ce_pixel = ce_pixel_r;
    wire [7:0] vga_r = frame_tag, vga_g = 8'h00, vga_b = 8'h00;

    // ---- DUT
    wire        DDRAM_CLK, DDRAM_BUSY, DDRAM_RD, DDRAM_WE;
    wire [7:0]  DDRAM_BURSTCNT, DDRAM_BE;
    wire [28:0] DDRAM_ADDR;
    wire [63:0] DDRAM_DIN;
    reg  [63:0] DDRAM_DOUT = 64'd0;
    reg         DDRAM_DOUT_READY = 1'b0;
    wire        o_fb_ena;
    wire [11:0] o_fb_hsize, o_fb_vsize;
    wire [5:0]  o_fb_format;
    wire [31:0] o_fb_base;
    wire [13:0] o_fb_stride;
    wire [1:0]  rotate_mode = 2'b01;               // CW, fixed -- this bench exercises the allocator continuously

    tango_video_rotate_ddr3 #(.MAXW(MAXW), .MAXH(MAXH), .NBUF(4)) dut (
        .clk_video(clk_video), .rst_video(1'b0), .rotate_mode(rotate_mode),
        .ce_pixel_in(ce_pixel), .vga_r_in(vga_r), .vga_g_in(vga_g), .vga_b_in(vga_b),
        .vga_hs_in(vga_hs), .vga_vs_in(vga_vs), .vga_de_in(vga_de),
        .DDRAM_CLK(DDRAM_CLK), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT),
        .DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
        .DDRAM_RD(DDRAM_RD), .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE),
        .clk_pix(clk_pix), .o_fb_ena(o_fb_ena), .o_fb_hsize(o_fb_hsize), .o_fb_vsize(o_fb_vsize),
        .o_fb_format(o_fb_format), .o_fb_base(o_fb_base), .o_fb_stride(o_fb_stride));

    // ---- trivial DDRAM responder: never busy, reads complete DOUT_READY 4 cycles later (fixed latency)
    assign DDRAM_BUSY = 1'b0;
    reg [2:0] rd_pend = 3'd0;
    always @(posedge DDRAM_CLK) begin
        DDRAM_DOUT_READY <= 1'b0;
        rd_pend <= {rd_pend[1:0], DDRAM_RD};
        if (rd_pend[2]) DDRAM_DOUT_READY <= 1'b1;
    end

    // ---- invariant checks (clk_video domain: direct visibility into the DUT's own allocator state)
    integer n_bank_viol = 0, n_overwrite_viol = 0, n_alloc_events = 0;
    reg [1:0] wbuf_q = 2'd0;
    always @(posedge clk_video) begin
        wbuf_q <= dut.wbuf;
        // check 1: bank separation, always, once the allocator has done anything at all
        if (dut.wbuf[1] == dut.rbuf[1] && dut.buf_ready[dut.rbuf]) begin
            n_bank_viol = n_bank_viol + 1;
            if (n_bank_viol <= 10) $display("BANK VIOLATION at %0t: wbuf=%0d rbuf=%0d share bank %0d; fence_buf=%0d fence_st=%0d frame_bad=%0d last_slot=%02b wbuf_valid=%0d alloc_fire=%0d",
                     $time, dut.wbuf, dut.rbuf, dut.wbuf[1], dut.fence_buf, dut.fence_st, dut.frame_bad, dut.last_slot, dut.wbuf_valid, dut.alloc_fire);
        end
        // check 2: no overwrite -- at the moment wbuf changes (a new allocation), it must never equal the
        // rbuf that was active just before the change
        if (dut.wbuf != wbuf_q) begin
            n_alloc_events = n_alloc_events + 1;
            if (dut.wbuf == dut.rbuf) begin
                n_overwrite_viol = n_overwrite_viol + 1;
                $display("OVERWRITE VIOLATION at %0t: new wbuf=%0d == rbuf=%0d", $time, dut.wbuf, dut.rbuf);
            end
        end
    end

    // ---- the "ascal" stub (clk_pix domain): latches o_fb_base/ena at its OWN VS edges, exactly as
    // ascal.vhd's avl_o_vs edge does (1655-1667) -- never in between
    integer ocnt = 0;
    reg      out_vs = 1'b0, out_vs_q = 1'b0;
    always @(posedge clk_pix) begin
        out_vs <= (ocnt < 2);
        ocnt   <= (ocnt == OUT_CYC - 1) ? 0 : ocnt + 1;
    end
    reg [31:0] latched_base = 32'hFFFF_FFFF;
    reg        latched_ena  = 1'b0;
    integer    n_out_vs = 0, n_repeat = 0, n_change = 0, n_bad_base = 0;
    // check 3: an earlier version flagged "TEAR RISK" purely on timing proximity (o_fb_base changing a few clk_pix
    // cycles before a VS latch), which was a false positive: `tango_cdc_word` (tango_cdc.v) is a toggle-handshake
    // CDC ("No multi-bit synchroniser anywhere," its own header), not a plain multi-bit sync, so `o_fb_base` only
    // ever updates as one complete, atomic word -- a value changing shortly before a VS edge is not evidence of a
    // torn read, just two independent, async cadences landing close together by chance. The real check for "no tear"
    // is that whatever `o_fb_base` decodes to is ALWAYS one of the 4 valid, aligned buffer addresses (never a
    // garbage/metastable value) -- trivially guaranteed by the handshake, checked dynamically anyway as a real net,
    // not just trusted by inspection.
    function is_valid_base(input [31:0] b);
        integer k; reg found;
        begin
            found = 1'b0;
            for (k = 0; k < 4; k = k + 1) if (b == {4'b0011, dut.buf_base(k[1:0]), 3'b000}) found = 1'b1;
            is_valid_base = found;
        end
    endfunction
    always @(posedge clk_pix) begin
        out_vs_q <= out_vs;
        if (out_vs && !out_vs_q) begin                       // a VS rising edge: latch now
            n_out_vs = n_out_vs + 1;
            if (n_out_vs > 2) begin                            // skip the first couple (start-up)
                if (o_fb_base == latched_base) n_repeat = n_repeat + 1;
                else n_change = n_change + 1;
                if (!is_valid_base(o_fb_base)) begin
                    n_bad_base = n_bad_base + 1;
                    $display("BAD BASE at %0t: o_fb_base=%08x is not one of the 4 valid buffer addresses", $time, o_fb_base);
                end
            end
            latched_base <= o_fb_base;
            latched_ena  <= o_fb_ena;
        end
    end

    // ---- end of run: judge
    initial begin
        wait (in_frame_no >= 10); $display("DEBUG: frame 10 at %0t", $time);
        wait (in_frame_no >= 50); $display("DEBUG: frame 50 at %0t", $time);
        wait (in_frame_no >= N_INPUT_FRAMES); $display("DEBUG: frame %0d at %0t", N_INPUT_FRAMES, $time);
        #(IN_CYC * 169 * 2);                      // let the last frame's fence finish draining
        $display("TB_ROTATE_BEAT: %0d input frames, %0d output VS samples (%0d repeats, %0d changes)",
                  in_frame_no, n_out_vs, n_repeat, n_change);
        $display("TB_ROTATE_BEAT: allocator events %0d, bank violations %0d, overwrite violations %0d",
                  n_alloc_events, n_bank_viol, n_overwrite_viol);
        $display("TB_ROTATE_BEAT: frames written %0d, frames dropped (FIFO overflow) %0d, FIFO overflows %0d, bad base reads %0d",
                  dut.obs_frames_written, dut.obs_frames_dropped, dut.obs_fifo_overflow, n_bad_base);
        $display("TB_RESULT %0s", (n_bank_viol == 0 && n_overwrite_viol == 0 && n_repeat > 0 && n_change > 0 &&
                  dut.obs_fifo_overflow == 0 && n_bad_base == 0) ? "PASS" : "FAIL");
        $finish;
    end
endmodule
`default_nettype wire
