// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_video_rotate_ddr3 -- screen rotation through the DDR3 (TANGO_VIDEO_ROTATE, README "Rotation").  It replaces
// an on-chip BSRAM corner-turn, which needed 338 of the 340 BSRAM blocks (99%) in an arcade core such as Galaga, over
// the 98% budget.  The design follows MiSTer's own screen_rotate (Arcade-Galaga_MiSTer sys/arcade_video.v 168-340),
// adapted to Tango's DDRAM_* C0 port shape:
//   - a write-only client of the C0 port (tango_ddr3_cport.v, through tango_ddr3's DDRAM_* ports, build option
//     TANGO_DDRAM); DDRAM_CLK = clk_video here (this client's own clock: the crossing into clk_ddr is inside
//     tango_ddr3_cport.v);
//   - rotation is address arithmetic during the write (addr +- stride per pixel), in C0's 25-bit DDRAM_ADDR
//     word-address space (screen_rotate's own bufsize/next_addr address MiSTer's different 8 MB-per-buffer window,
//     so they are not reused);
//   - ascal's own framebuffer read engine (o_fb_ena/hsize/vsize/format/base/stride, ascal_wrap.v) reads the result
//     back and does all of the rotated image's scaling and HDMI timing, as MiSTer's sys_top.v wires FB_* into
//     ascal's o_fb_* ports; there is no separate readout path.
//
// This module does not re-time or re-shape the output raster: ascal's output engine generates the 720p raster from
// htotal/hdisp/etc. whatever the source orientation.  This module only WRITES, at the core's native input raster
// timing, and tells ascal where the result lives.
//
// ascal's `i_*` input feed is unchanged and not gated by rotation: MiSTer keeps feeding ascal's input while FB_EN is
// set (sys_top.v 735-742), and Tango does the same, so tango_video.v's CE_PIXEL contract is untouched.  ascal's own
// capture traffic (not displayed while rotating) stays in its DDR3 bank 4, which cannot clash with this module's
// bank 6/7 buffers (below).
//
// BANK SEPARATION: tango_ddr3_cport.v forces C0's address into the 0x3000_0000 window, which spans DDR3 bank 6
// (DDRAM_ADDR bit 24 = 0) and bank 7 (bit 24 = 1).  The buffers are placed (buf_base below) so that the buffer this
// module writes is always in the other bank from the buffer ascal reads via o_fb_base: the writer and the reader
// never share a bank, as the DDR3 layout keeps the scaler and the cores in separate banks.
//
// NBUF = 4.  A frame is small (~258 KB for 288x224x4B), so memory is not the constraint:
//   NBUF=2: bank separation always holds, but is drop-free only with the output LOCKED to the input -- not safe as
//           the default.
//   NBUF=3: does NOT preserve bank separation: with 3 buffers over 2 banks, two share a bank, which can put the
//           writer and ascal's read in the same bank -- not used.
//   NBUF=4 (this module): 2 buffers per bank, with the bank-aware allocator below -- the writer always targets the
//           bank opposite the one `rbuf` (ascal's current buffer) is in, and WITHIN that bank alternates between its
//           two slots each visit, so a given physical buffer is revisited only every 4 frames, not every 2 --
//           margin against ascal's read pipeline lagging behind o_fb_base's swap by up to one extra "bank visit".
//           Bank separation holds AND the rate stays full (no clock lock needed).
//
// DROP POLICY: the 59.04 Hz in / 60.00 Hz out BEAT (a frame repeated or dropped about once a second) is NOT this
// module dropping anything.  In normal operation (no FIFO overflow) every native input frame is written, fenced and
// marked ready, advancing `rbuf`.  ascal, on its own independent VS edge, samples whatever `o_fb_base` is at that
// instant: if the writer has not finished a NEWER frame since ascal's last VS, ascal re-latches the SAME base (a
// natural repeat); if the writer finished more than one frame between two ascal VS edges, only the latest is seen
// (a natural skip).  Neither involves any code here: it is a side effect of o_fb_base having (or not having)
// changed.  The ONLY case where this module abandons a write in progress is elastic-FIFO overflow (below): a failure
// to keep up with the pixel rate, not a consequence of the input/output rate mismatch.
//
// BACKPRESSURE AND OVERFLOW: MiSTer's screen_rotate never reads DDRAM_BUSY (DDRAM_WE fires unconditionally on
// CE_PIXEL && VGA_DE); this module does.  An elastic FIFO (FIFO_DEPTH, 64 pixels, sized at the parameter) sits ahead
// of the write state machine.  Overflow policy: DROP THE FRAME, DON'T CORRUPT IT.  On overflow there are no further
// writes for the rest of the current input frame, that buffer is never marked ready, and o_fb_base (via the CDC
// below) keeps pointing at the last completed buffer -- ascal shows a held, stale but whole frame, never a torn one.
//
// FRAME FENCE: the C0 port's writes have no completion signal of their own (in tango_ddr3_arb a write retires when
// any later read returns).  Before marking a buffer ready, this module issues one DDRAM_RD of its own (the buffer's
// base word) and waits for DDRAM_DOUT_READY -- since the IP executes in order, that proves every earlier write of
// the frame has landed, not merely been accepted into the port's async FIFOs.
//
// CDC into ascal's o_clk domain: which buffer is ready is decided in clk_video; o_fb_ena/hsize/vsize/stride/base
// cross to clk_pix (ascal's o_clk) through tango_cdc_word, the same primitive and the same "quasi-static, updated at
// frame boundary" discipline as every other ascal geometry input in this framework (ascal_wrap.v's port comment:
// "geometry / config (quasi-static, o_clk domain, updated at frame boundary)").  ascal.vhd then latches them into
// its avl_clk domain at the output's VS edge (ascal.vhd 1655-1667, marked "<ASYNC>" there).
//
// PIXEL FORMAT: 32bpp, o_fb_format = 6'b000110 (ascal.vhd's "110=32bpp" table at line 42; MiSTer's FB_FORMAT for
// this case); the byte order is described at the elastic FIFO below.
//==============================================================================
`default_nettype none

module tango_video_rotate_ddr3 #(
    parameter integer MAXW       = 256,              // native max active pixels/line (input)
    parameter integer MAXH       = 240,              // native max active lines/frame (input)
    parameter integer NBUF       = 4,                // the buffer count of the table above, not a live
                                                       // switch: the buffer allocator (buf_base / next_wbuf
                                                       // below) implements only NBUF=4's bank-aware scheme;
                                                       // NBUF=2 and NBUF=3 are the alternatives the table
                                                       // rejects, not built as code paths.  The generate-time
                                                       // guard below fails elaboration if this is set to
                                                       // anything else in simulators (Verilator etc.), but
                                                       // GowinSynthesis ignores it (EX2488, "Elaboration
                                                       // system task 'error' is ignored for synthesis"), so
                                                       // it is no guarantee on that toolchain.
    parameter integer FIFO_DEPTH = 64,                // elastic FIFO entries ahead of the DDRAM_* write FSM:
                                                        // the arbiter bench measures a worst-case DDRAM_BUSY
                                                        // stall of 762 clk_video cycles (10.26 us) at the
                                                        // Template core's rate under V-port contention, which
                                                        // needs ~63 pixels of buffering at its pixel period
                                                        // (~162 ns) to absorb without overflow; 32 entries
                                                        // overflowed (1 overflow in 60 frames)
    // C0's own 256 MB window: DDRAM_ADDR is a 25-bit word (8 B) address, byte address =
    // 0x3000_0000 + DDRAM_ADDR*8. Bank 6: DDRAM_ADDR[24]=0 (0x000_0000-0x0FF_FFFF); bank 7: DDRAM_ADDR[24]=1
    // (0x100_0000-0x1FF_FFFF). With NBUF=4, buffers 0/1 sit in bank 6 and buffers 2/3 in bank 7 (buf_base
    // below); each is sized BUF_STRIDE apart, comfortably larger than one native frame (288x224x4B ~258 KB).
    parameter [24:0] WIN_BASE    = 25'd0,              // this module's own base within DDRAM_ADDR's 25 bits
    parameter [23:0] BUF_STRIDE  = 24'h08_0000         // 512K words (4 MB) per buffer slot: 2 slots per bank
                                                        // = 8 MB, inside bank 6/7's 128 MB each
) (
    // ---- core's native raster (clk_video domain): read-only, never gated (ascal's own i_* feed, unchanged,
    // takes the same signals in parallel -- this module never re-drives vga_*)
    input  wire        clk_video,
    input  wire        rst_video,
    input  wire [1:0]  rotate_mode,        // CDC'd upstream (tango_cdc_word): 00 Normal,
                                            // 01 CW, 10 CCW, 11 reserved (decoded as Normal)
    input  wire        ce_pixel_in,
    input  wire [7:0]  vga_r_in, vga_g_in, vga_b_in,
    input  wire        vga_hs_in, vga_vs_in, vga_de_in,

    // ---- C0 port (DDRAM_*, MiSTer's shape, tango_ddr3_cport.v): DDRAM_CLK = clk_video, no new CDC here
    output wire         DDRAM_CLK,
    input  wire         DDRAM_BUSY,
    output wire [7:0]   DDRAM_BURSTCNT,                 // always single-beat (BURSTCNT=1), as screen_rotate.v: a
                                                        // continuous assign below, NOT an initialized reg (Gowin
                                                        // drops an output reg's initializer, EX2478, and a reg
                                                        // with no other driver then leaves the port undriven: a
                                                        // burst count of 0 never arms tango_ddr3_cport)
    output reg  [28:0]  DDRAM_ADDR = 29'd0,
    input  wire [63:0]  DDRAM_DOUT,
    input  wire         DDRAM_DOUT_READY,
    output reg          DDRAM_RD = 1'b0,
    output reg  [63:0]  DDRAM_DIN = 64'd0,
    output reg  [7:0]   DDRAM_BE = 8'd0,
    output reg          DDRAM_WE = 1'b0,

    // ---- ascal's o_fb_* (clk_pix = ascal's o_clk domain; CDC'd below)
    input  wire         clk_pix,
    output wire         o_fb_ena,
    output wire [11:0]  o_fb_hsize,
    output wire [11:0]  o_fb_vsize,
    output wire [5:0]   o_fb_format,
    output wire [31:0]  o_fb_base,
    output wire [13:0]  o_fb_stride,

    // observation (sim/debug)
    output reg  [31:0]  obs_frames_written = 0,
    output reg  [31:0]  obs_frames_dropped = 0,
    output reg  [31:0]  obs_fifo_overflow  = 0
);
    assign DDRAM_CLK = clk_video;
    assign DDRAM_BURSTCNT = 8'd1;                      // single-beat, always (see the port list)

    // generate-time check: only NBUF=4 is implemented (comment on the parameter above).
    generate
        if (NBUF != 4) begin : nbuf_not_implemented
            $error("tango_video_rotate_ddr3: NBUF=%0d is not implemented, only NBUF=4's bank-aware allocator is (docs/ROTATION.md 7.3.2)", NBUF);
        end
    endgenerate

    // ================= input stage: the core's raster registered once (timing) =================
    // CE_PIXEL is a combinational function of the core's counters (Template: three LUTs after a counter bit) and fed ~70
    // enables here; one build put that net 8.5 ns across the die (clk_pix/clk_video -1.4 ns).  The whole raster and the pixel
    // are delayed by one clk_video together, so nothing in here changes but a constant one-clock shift of the pixel stream.
    reg         ce_pixel_r = 1'b0, vga_vs_r = 1'b0, vga_de_r = 1'b0;
    reg  [7:0]  vga_r_r = 8'd0, vga_g_r = 8'd0, vga_b_r = 8'd0;
    always @(posedge clk_video) begin
        ce_pixel_r <= ce_pixel_in; vga_vs_r <= vga_vs_in; vga_de_r <= vga_de_in;
        vga_r_r <= vga_r_in; vga_g_r <= vga_g_in; vga_b_r <= vga_b_in;
    end

    // ================= geometry: measured each frame, like screen_rotate.v's hsz/vsz =================
    reg  [11:0] hsz = MAXW[11:0], vsz = MAXH[11:0];        // native (unrotated) measured size
    reg  [11:0] hcnt = 12'd0, vcnt = 12'd0;
    reg         de_q = 1'b0, vs_q = 1'b0;
    wire        vs_rise = vga_vs_r && !vs_q;
    wire        de_fall = !vga_de_r && de_q;
    always @(posedge clk_video) begin
        if (rst_video) begin
            hcnt <= 12'd0; vcnt <= 12'd0; de_q <= 1'b0; vs_q <= 1'b0; hsz <= MAXW[11:0]; vsz <= MAXH[11:0];
        end else if (ce_pixel_r) begin
            de_q <= vga_de_r; vs_q <= vga_vs_r;
            if (vga_de_r) hcnt <= hcnt + 12'd1;
            if (de_fall) begin hsz <= hcnt; hcnt <= 12'd0; end
            if (vs_rise)  begin vsz <= vcnt; vcnt <= 12'd0; end
            else if (de_fall) vcnt <= vcnt + 12'd1;
        end
    end
    // rotated (output) dimensions: CW/CCW swap W and H; Normal is a don't-care (o_fb_ena=0 in that mode)
    wire        rot_ccw   = (rotate_mode == 2'b10);
    wire        rotating  = (rotate_mode == 2'b01) || (rotate_mode == 2'b10);
    wire [11:0] fb_w = vsz;                                 // rotated width  = native height
    wire [11:0] fb_h = hsz;                                 // rotated height = native width

    // 32bpp: 4 bytes/pixel, so stride in bytes = fb_w * 4, rounded up to a 16-byte (4-pixel) boundary to match
    // screen_rotate.v's own {bwidth[11:2],4'd0} shape (there, 8-byte granularity; here, 4 B/pixel already
    // divides evenly into 16 B every 4 pixels), adapted to 32bpp. o_fb_stride
    // (ascal's port) wants BYTES; the write FSM's own next_addr is a WORD (8 B) address, so it steps by
    // stride_w = stride_b/8 words, computed directly (not by slicing stride_b) to avoid a divide-by-shift bug.
    wire [9:0]  fb_w4   = fb_w[11:2] + (|fb_w[1:0] ? 10'd1 : 10'd0);   // rotated width rounded up to 4 pixels
    wire [13:0] stride_b = {fb_w4, 4'd0};                               // bytes:  fb_w4 * 16
    wire [10:0] stride_w = {fb_w4, 1'b0};                               // words:  fb_w4 * 2  (16 B = 2 words)

    // ================= buffer bookkeeping: NBUF=4, bank-aware =================
    // buf_bank(n) = n[1]: buffers 0,1 in bank 6 (DDRAM_ADDR[24]=0), buffers 2,3 in bank 7 (DDRAM_ADDR[24]=1).
    // buf_slot(n) = n[0]: which of the bank's 2 physical slots. This is a FIXED layout, not an even division
    // of the window: bank separation has to be exact, not approximate.
    function [24:0] buf_base(input [1:0] n);
        reg [24:0] bank_base, slot_off;
        begin
            bank_base = n[1] ? 25'h100_0000 : 25'h0;    // bank 6 (0) or bank 7 (0x100_0000 in DDRAM_ADDR units)
            slot_off  = n[0] ? {1'b0, BUF_STRIDE} : 25'd0;   // slot 0 or slot 1 within that bank
            buf_base  = WIN_BASE + bank_base + slot_off;
        end
    endfunction

    reg [1:0] wbuf = 2'd2;     // the buffer this module is currently writing: {bank=1,slot=0} -- opposite
                                // bank from rbuf's own init below, so the invariant holds from power-up too
    reg [1:0] rbuf = 2'd0;     // the buffer ascal is currently told to read (o_fb_base) -- "the latest
                                // complete frame," never overwritten by construction (wbuf's bank is always
                                // rbuf's opposite bank, so wbuf can never equal rbuf)
    reg [1:0] last_slot = 2'b00; // last_slot[bank]: which slot bank 0/1 used last time it was written, so
                                   // the NEXT visit to that bank alternates slots (margin for ascal's own
                                   // read pipeline lagging by up to one extra "bank visit")
    reg       buf_ready [0:3]; // per-buffer: has a complete, fenced frame ever landed in it
    integer   bi;
    initial for (bi = 0; bi < 4; bi = bi + 1) buf_ready[bi] = 1'b0;

    // bank-aware allocator: picks the bank opposite the one the buffer `ref_buf` names is in,
    // alternating slots within that bank (`last_slot`) so a physical buffer is revisited only every 4 frames.
    function [1:0] next_wbuf(input [1:0] ref_buf);
        reg target_bank, new_slot;
        begin
            target_bank = ~ref_buf[1];
            new_slot    = ~last_slot[target_bank];
            next_wbuf   = {target_bank, new_slot};
        end
    endfunction

    // WHEN the allocator runs, and what it avoids: choosing the new wbuf eagerly at vs_rise (opposite the
    // buffer about to be fenced) is right for the STEADY STATE but wrong for the WINDOW while that fence is
    // still in flight: `rbuf` itself does not advance until the fence succeeds (a few clk_ddr cycles later,
    // ~270 ns), so for that whole window the newly chosen wbuf and the *still-current* rbuf are, by the
    // alternating-bank construction, exactly TWO generations apart -- which lands them in the SAME bank
    // (two bank flips cancel out; sim/tb_rotate_beat.v catches that case).  So the allocator does not run
    // until `rbuf` itself is about to settle, i.e. at the fence's own success (using `fence_buf`, the value
    // `rbuf` becomes on this same edge, as the reference) or immediately at `vs_rise` only in the
    // no-fence-needed dropped-frame case (where `rbuf` never moves, so the CURRENT `rbuf` is already the
    // right reference). Nothing writes into the new buffer until this allocation has happened (`wbuf_valid`,
    // below) -- the elastic FIFO absorbs the few clocks' worth of pixels that arrive in the meantime.
    reg        wbuf_valid = 1'b1;     // is `wbuf` the buffer THIS frame should actually write into yet?
    wire       alloc_drop    = (fence_st == FENCE_IDLE) && vs_rise && rotating && frame_bad;
    wire       alloc_success = (fence_st == FENCE_WAIT) && DDRAM_DOUT_READY;
    wire       alloc_giveup  = (fence_st == FENCE_WAIT) && !DDRAM_DOUT_READY && (fence_to == 16'hFFFF);
    wire       alloc_fire    = alloc_drop || alloc_success || alloc_giveup;
    // the reference buffer each case must avoid: `rbuf` never moves in the drop/give-up cases; `fence_buf`
    // is what `rbuf` is THIS edge becoming in the success case (reading `rbuf` there would be the same stale
    // reference described above)
    wire [1:0] alloc_ref = alloc_success ? fence_buf : rbuf;
    wire [1:0] wbuf_new_val = next_wbuf(alloc_ref);

    // ================= elastic FIFO: {b,g,r} ahead of the write FSM, drained at the DDR3's own pace =================
    // PIXEL BYTE ORDER: each FIFO entry is {B,G,R}, so each 32-bit half of DDRAM_DIN below is {8'd0,B,G,R} =
    // byte0 R, byte1 G, byte2 B. That is the
    // order ascal.vhd's 32bpp RGB reader takes (o_fb_format bit 4 = 0): avl_dw = swap(avl_readdata) (BYTESWAP
    // true) puts byte0 on top, shift_opack loads it at shift(0 TO 7), shift_opix reads r => shift(0 TO 7). It is
    // also MiSTer's own packing (sys/arcade_video.v: ram_data <= {8'd0,VGA_B,VGA_G,VGA_R}, FB_FORMAT 5'b00110).
    // An earlier version stored {R,G,B} (byte0 = B), which showed red as blue in rotated modes.
    localparam FAW = $clog2(FIFO_DEPTH);
    reg  [23:0] fifo_mem [0:FIFO_DEPTH-1];
    reg  [FAW:0] fifo_wp = 0, fifo_rp = 0;                  // one extra bit: full/empty by compare, no separate flag
    wire        fifo_empty = (fifo_wp == fifo_rp);
    wire        fifo_full  = (fifo_wp[FAW-1:0] == fifo_rp[FAW-1:0]) && (fifo_wp[FAW] != fifo_rp[FAW]);
    reg         frame_bad  = 1'b0;                          // this input frame already dropped (FIFO overflowed)
    wire        fifo_push  = rotating && ce_pixel_r && vga_de_r && !frame_bad;

    always @(posedge clk_video) begin
        if (rst_video) begin
            fifo_wp <= 0; frame_bad <= 1'b0; obs_fifo_overflow <= 0;
        end else begin
            if (vs_rise) frame_bad <= 1'b0;                 // a fresh input frame, a fresh chance
            if (fifo_push) begin
                if (!fifo_full) begin
                    fifo_mem[fifo_wp[FAW-1:0]] <= {vga_b_r, vga_g_r, vga_r_r};   // {B,G,R}: byte0 = R, see above
                    fifo_wp <= fifo_wp + 1'b1;
                end else begin
                    frame_bad <= 1'b1;                      // drop the rest of this frame, cleanly (header)
                    obs_fifo_overflow <= obs_fifo_overflow + 1'b1;
                end
            end
        end
    end

    // ================= write FSM: pop the FIFO, step the rotated address, issue DDRAM_WE =================
    // Address algebra (checked pixel-exact against a golden model by sim/tb_rotate_golden.v).  A native pixel
    // (x, y) of a W x H frame goes to the rotated fb_w x fb_h frame (fb_w = H, fb_h = W) at:
    //   CW:  output column H-1-y, output row x;        CCW: output column y, output row W-1-x.
    // So the FIRST native row lands in the LAST output column for CW (the first for CCW), and each later native
    // row moves one column BACKWARD for CW (forward for CCW).  The module tracks the output COLUMN of the current
    // native row explicitly (`xo_val`, fixed for the whole row, stepped only at a row wrap, in the direction and
    // from the starting value the rotation mode requires) and derives the row's starting word address and BE half
    // FRESH from `xo_val` at the frame start and at every row wrap, rather than stepping a raw address and
    // inferring the half from one of its bits:
    //   CW:  xo_val starts at fb_w-1 (the LAST output column), decrements each native row, to 0.
    //   CCW: xo_val starts at 0 (the FIRST output column), increments each native row, to fb_w-1.
    //   Within a row (fixed xo_val), the output ROW increases (CW) or decreases (CCW) as native columns
    //   arrive: the WITHIN-ROW step is `+-stride_w` per pixel.
    // The byte-enable half is the output column's parity (xo_val[0]), NOT a bit of the word address: with 2
    // output columns per word (and an even stride_w), `next_addr`'s LSB is xo[1], not xo[0].
    // One 64-bit DDRAM beat holds 2 pixels (32bpp); BE picks the half, as screen_rotate.v's own
    // `ram_addr[2] ? 8'hF0 : 8'h0F` (the same SHAPE, fixed per output column's parity).
    reg [24:0] next_addr = 25'd0;
    reg [13:0] hrun = 14'd0;                                 // native columns left in the current row
    reg [11:0] xo_val = 12'd0;                                // THIS row's fixed output column
    reg        wbusy = 1'b0;
    reg [23:0] cur_px = 24'd0;
    reg        have_px = 1'b0;

    // the word address for the START of a native row, given which output column (xo) that row lands in:
    // CW starts its within-row stepping from output row 0 (yo=0); CCW starts from the LAST output row
    // (yo=fb_h-1) and steps downward -- so the row's starting WORD differs by mode too, not just xo_val.
    function [24:0] row_start_addr(input [1:0] b, input [11:0] xo, input ccw);
        reg [24:0] yo_term;
        begin
            yo_term = ccw ? ({11'd0, fb_h - 12'd1} * {14'd0, stride_w}) : 25'd0;
            row_start_addr = buf_base(b) + {13'd0, xo[11:1]} + yo_term;
        end
    endfunction

    always @(posedge clk_video) begin
        DDRAM_WE <= 1'b0;
        if (rst_video) begin
            wbusy <= 1'b0; have_px <= 1'b0;
        end else begin
            // pop one pixel from the FIFO when the write side is free to take it
            if (!have_px && !fifo_empty) begin
                cur_px  <= fifo_mem[fifo_rp[FAW-1:0]];
                fifo_rp <= fifo_rp + 1'b1;
                have_px <= 1'b1;
            end
            // the address stepper resets on `alloc_fire` (the allocator above), not on `vs_rise`: the new
            // wbuf is not actually chosen until then (vs_rise alone, when a fence is needed, only STARTS
            // that fence -- the allocation itself waits for it to settle, see the allocator). Using the
            // combinational `wbuf_new_val` here, computed from the same stable pre-edge state the fence
            // block reads, avoids the register race a plain read of `wbuf` would have on this same edge.
            if (alloc_fire) begin
                xo_val    <= rot_ccw ? 12'd0 : (fb_w - 12'd1);
                next_addr <= row_start_addr(wbuf_new_val, rot_ccw ? 12'd0 : (fb_w - 12'd1), rot_ccw);
                hrun      <= {2'b00, hsz};
            end
            if (have_px && !DDRAM_BUSY && wbuf_valid) begin
                // DDRAM_ADDR itself is driven by the single-driver block near the end of the module
                // (`wr_addr_fire`/`wr_addr_val`) -- not assigned in this block, so
                // this is the only real driver for every OTHER DDRAM_* signal this FSM owns.
                DDRAM_DIN  <= {8'd0, cur_px, 8'd0, cur_px};   // both halves driven; BE below selects one;
                                                               // cur_px = {B,G,R}, each half {0,B,G,R} (byte0 R)
                DDRAM_BE   <= xo_val[0] ? 8'hF0 : 8'h0F;      // THIS row's own fixed column parity, not a
                                                               // bit of the evolving word address (the
                                                               // address algebra above)
                DDRAM_WE   <= 1'b1;
                have_px    <= 1'b0;
                next_addr  <= rot_ccw ? (next_addr - {14'd0, stride_w}) : (next_addr + {14'd0, stride_w});
                if (hrun == 14'd1) begin
                    hrun  <= {2'b00, hsz};
                    // next native row: xo_val steps -1 (CW) or +1 (CCW); the row's starting word is
                    // recomputed fresh from the NEW xo_val, not incremented from the old next_addr (a
                    // plain "+1 word" has neither the right sign nor the right magnitude once output
                    // columns pack 2 per word)
                    xo_val    <= rot_ccw ? (xo_val + 12'd1) : (xo_val - 12'd1);
                    next_addr <= row_start_addr(wbuf, rot_ccw ? (xo_val + 12'd1) : (xo_val - 12'd1), rot_ccw);
                end else hrun <= hrun - 14'd1;
            end
        end
    end

    // ================= frame fence + buffer swap =================
    // `fence_buf` captures, at vs_rise, which buffer the frame that JUST ended was written into (the OLD
    // `wbuf`, read before this same edge's update) -- the fence FSM reads that one back, not whatever `wbuf`
    // has become by the time FENCE_ISSUE actually runs a few clocks later.
    reg [2:0] fence_st = 3'd0;
    localparam FENCE_IDLE = 0, FENCE_ISSUE = 1, FENCE_WAIT = 2, FENCE_DONE = 3;
    reg [15:0] fence_to = 16'd0;
    reg [1:0]  fence_buf = 2'd0;
    always @(posedge clk_video) begin
        if (rst_video) begin
            fence_st <= FENCE_IDLE; DDRAM_RD <= 1'b0; obs_frames_written <= 0; obs_frames_dropped <= 0;
            wbuf_valid <= 1'b1;
        end else begin
            DDRAM_RD <= 1'b0;
            // the allocator (above): fires exactly on alloc_drop, alloc_success or alloc_giveup, each
            // mutually exclusive with the others by construction (different fence_st values required), so
            // these two assignments never race each other on the same edge.
            if (alloc_fire) begin
                wbuf       <= wbuf_new_val;
                wbuf_valid <= 1'b1;
                last_slot[wbuf_new_val[1]] <= wbuf_new_val[0];
            end
            case (fence_st)
                FENCE_IDLE: if (vs_rise && rotating) begin
                    // the native frame that just ended (the OLD wbuf, captured now) is complete iff this
                    // frame was not dropped by the elastic-FIFO overflow path -- NOT the normal
                    // 59.04/60.00 beat, which this module never treats as a drop (the header's DROP
                    // POLICY): a rate-mismatch repeat/skip is purely ascal's own read-side
                    // sampling of whatever `o_fb_base` is at its own VS, with no action needed here.
                    fence_buf <= wbuf;
                    if (!frame_bad) begin
                        fence_st   <= FENCE_ISSUE;
                        wbuf_valid <= 1'b0;   // no writes into the new wbuf until the allocator actually
                                               // runs, at this fence's own success below
                    end else begin
                        obs_frames_dropped <= obs_frames_dropped + 1'b1;
                        // alloc_drop (above) fires THIS SAME edge: rbuf never moved, so the allocator can
                        // run immediately using the current (still valid) rbuf as its reference -- no fence
                        // needed, no wbuf_valid gap.
                    end
                end
                FENCE_ISSUE: if (!DDRAM_BUSY) begin
                    // DDRAM_ADDR itself is driven below (its single driver) --
                    // this state only needs to request it via fence_addr_fire, which is true exactly when
                    // this condition (fence_st==FENCE_ISSUE && !DDRAM_BUSY) is true.
                    DDRAM_RD   <= 1'b1;
                    fence_to   <= 16'd0;
                    fence_st   <= FENCE_WAIT;
                end
                FENCE_WAIT: begin
                    fence_to <= fence_to + 1'b1;
                    if (DDRAM_DOUT_READY) begin
                        buf_ready[fence_buf] <= 1'b1;
                        rbuf <= fence_buf;                    // ascal now reads the just-fenced buffer --
                                                               // "the latest complete frame," never the one
                                                               // currently being written (bank-separated by
                                                               // construction). alloc_success fires
                                                               // this same edge, referencing fence_buf (not
                                                               // the stale rbuf) for the new wbuf.
                        obs_frames_written <= obs_frames_written + 1'b1;
                        fence_st <= FENCE_IDLE;
                    end else if (fence_to == 16'hFFFF) fence_st <= FENCE_IDLE;  // alloc_giveup fires this
                                                               // same edge, referencing the (unchanged) rbuf
                end
                default: fence_st <= FENCE_IDLE;
            endcase
        end
    end

    // ================= DDRAM_ADDR: the ONE driver for this net =================
    // Both the write FSM and the fence FSM above need to set DDRAM_ADDR.  Their firing conditions are mutually
    // exclusive BY CONSTRUCTION: `wbuf_valid` is low for the whole FENCE_ISSUE/FENCE_WAIT window (the comment at
    // `wbuf_valid`'s declaration above), so `wr_addr_fire` is false on every cycle `fence_addr_fire` is true.
    // A separate `DDRAM_ADDR <=` in each FSM's own `always` block simulates correctly (a simulator schedules
    // whichever nonblocking assignment fires on a given edge), but GowinSynthesis's static driver analysis does
    // not reason about runtime mutual exclusion across two `always` blocks and refuses to build two sequential
    // drivers for one net (EX2000, "Net 'DDRAM_ADDR[28]' is constantly driven from multiple places").  Verilator
    // has no check for that shape: its MULTIDRIVEN warning fires only for "multiple driving blocks with
    // DIFFERENT clocking", and both FSMs here use the same `posedge clk_video`.
    //
    // So DDRAM_ADDR has exactly ONE sequential driver (this block), fed by a combinational select between the
    // two FSMs' own address requests. `fence_addr_fire` is checked first only as a style choice (not a
    // tiebreak: there is no cycle where both are true, which the sim-only contract check below verifies on
    // every cycle).
    wire        wr_addr_fire    = !rst_video && have_px && !DDRAM_BUSY && wbuf_valid;
    wire [28:0] wr_addr_val     = {4'd0, next_addr};
    wire        fence_addr_fire = !rst_video && (fence_st == FENCE_ISSUE) && !DDRAM_BUSY;
    wire [28:0] fence_addr_val  = {4'd0, buf_base(fence_buf)};   // read back the JUST-FINISHED buffer, not wbuf
    always @(posedge clk_video) begin
        if (fence_addr_fire)        DDRAM_ADDR <= fence_addr_val;
        else if (wr_addr_fire)      DDRAM_ADDR <= wr_addr_val;
    end

    //==================================================================== DDRAM_ADDR exclusivity contract
    // (simulation only): proves, every cycle, what the comment above claims by construction -- that
    // wr_addr_fire and fence_addr_fire never fire together. Same pragma pair as tango_video.v's "TANGO
    // CONTRACT" blocks: GowinSynthesis skips the code between the two pragma comments below
    // (SUG550-2.0.1E section 5.19); Verilator has no such pragma and compiles the block. No netlist change.
    /* synthesis translate_off */
    always @(posedge clk_video)
        if (wr_addr_fire && fence_addr_fire)
            $fatal(1, "TANGO CONTRACT: DDRAM_ADDR drivers not mutually exclusive (see docs/ROTATION.md 7.12, %m, %0t)", $time);
    /* synthesis translate_on */

    // ================= CDC into clk_pix (ascal's o_clk): quasi-static, updated at frame boundary =================
    // the same tango_cdc_word primitive and discipline every other ascal geometry input in this framework
    // uses (ascal_wrap.v's own port comment: "quasi-static, o_clk domain, updated at frame boundary")
    // byte address = 0x3000_0000 + buf_base(rbuf)*8 (tango_ddr3_cport.v's header comment): buf_base is 25 bits,
    // *8 is a 3-bit left shift, so the byte address's top 4 bits are fixed at 4'b0011 and its low 3 bits are
    // always 0.
    wire [31:0] fb_base_vrot = {4'b0011, buf_base(rbuf), 3'b000};  // 4 + 25 + 3 = 32 bits
    localparam integer FBW = 1 + 12 + 12 + 14 + 32;                 // rotating, fb_w, fb_h, stride_b, base
    wire [FBW-1:0] fb_word     = {rotating, fb_w, fb_h, stride_b, fb_base_vrot};  // fb_w = rotated WIDTH (o_fb_hsize)
    wire [FBW-1:0] fb_word_pix;
    tango_cdc_word #(.W(FBW)) u_fb_cdc (
        .clk_a(clk_video), .din(fb_word),
        .clk_b(clk_pix),   .dout(fb_word_pix));
    assign o_fb_ena    = fb_word_pix[FBW-1];
    assign o_fb_hsize  = fb_word_pix[FBW-2:FBW-13];     // fb_w: rotated width
    assign o_fb_vsize  = fb_word_pix[FBW-14:FBW-25];    // fb_h: rotated height
    assign o_fb_stride = fb_word_pix[FBW-26:FBW-39];
    assign o_fb_base   = fb_word_pix[31:0];
    assign o_fb_format = 6'b000110;                               // 32bpp (ascal.vhd header: "110=32bpp")

endmodule
`default_nettype wire
