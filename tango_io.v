// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_io -- the core-facing side of the framework: what MiSTer's hps_io gives an
// emu, with MiSTer's signal meanings (Template_MiSTer sys/hps_io.sv, 19e89ce; meanings
// only, no code copied), carried from the link's clock (clk_pix) into the core's
// clk_sys (any clock the core chooses, possibly clk_pix itself).
//
//   joystick_0/1 [31:0]  hps_io.sv 41-42: bit 0 right, 1 left, 2 down, 3 up, then the core's
//                        buttons -- here A B X Y L R Select Start (bits 4-11), 12-31 zero.
//                        joystick_0 = the pad on USB-A2, joystick_1 = USB-A1.  Both read 0
//                        while the OSD is shown (menu input never reaches the core).
//   status [127:0]       hps_io.sv 119, 469-481: the option word, set by SYS 0x10 SET_STATUS.
//   status_menumask[15:0] hps_io.sv 122, 522: from the core, read by SYS 0x11 GET_MENUMASK.
//   ioctl_download / index / wr / addr / dout, ioctl_wait   hps_io.sv 146-157, 618-705:
//                        a file sent by the BL616 (SDC IMAGE), one byte per ioctl_wr pulse,
//                        ioctl_addr 0,1,2,...; ioctl_wait high holds the next byte back.
// Crossings: status = held word + toggle (3 flops); joysticks and menumask = tango_cdc_word
// handshakes; the download = tango_async_fifo (2048 entries) whose free count the link
// offers to the BL616, so nothing is ever lost while the core waits. TANGO_IOCTL_UPLOAD:
// the reverse direction (ioctl_upload/ioctl_rd/ioctl_din) = a toggle + held data from the link
// (tango_sd.v's own mnt_tog/mnt_drive idiom) opening/closing a session, and a dual-port buffer (2048 bytes,
// non-destructive on the link's read side) the fill engine below fills from the core; no FIFO, see that
// block's own comment for why.
//==============================================================================
`include "tango_cfg.vh"
`default_nettype none
module tango_io (
    // link side (clk_pix)
    input  wire         clk_pix,
    input  wire [31:0]  joy_a2, joy_a1,     // pads through the core map (SET_PADMAP), MiSTer order
    input  wire         osd_shown,
    input  wire [127:0] status_l,           // link: SET_STATUS word, stable between toggles
    input  wire         status_tog,
    output wire [15:0]  menumask_l,         // to the link
    input  wire         dl_we,
    input  wire [9:0]   dl_wd,
    output wire [11:0]  dl_free,
    // core side (clk_sys)
    input  wire         clk_sys,
    output wire [31:0]  joystick_0, joystick_1,
    output reg          osd_status = 1'b0,  // clk_sys: the OSD is shown (MiSTer OSD_STATUS); 2 flops
    input  wire [10:0]  ps2_key_pix,        // clk_pix (tango_link, HID KEYBOARD)
    output wire [10:0]  ps2_key,            // clk_sys: MiSTer ps2_key (tango_cdc_word)
`ifdef TANGO_PRESET_STATUS
    output reg  [127:0] status = `TANGO_PRESET_STATUS,  // the build's option word at power-up (tango_cfg.vh)
`else
    output reg  [127:0] status = 128'd0,
`endif
    input  wire [15:0]  status_menumask,
    output reg          ioctl_download = 1'b0,
    output reg  [15:0]  ioctl_index = 16'd0,
    output reg          ioctl_wr = 1'b0,
    output reg  [26:0]  ioctl_addr = 27'd0,
    output reg  [7:0]   ioctl_dout = 8'd0,
    input  wire         ioctl_wait
`ifdef TANGO_IOCTL_UPLOAD
    ,
    // ---- upload (TANGO_IOCTL_UPLOAD): the reverse of the download FIFO above.
    // Core side (clk_sys), directions as corrected in tango_core_ports.vh (framework -> core: ioctl_upload,
    // ioctl_rd; core -> framework: ioctl_upload_req/_index, ioctl_din):
    output reg          ioctl_upload = 1'b0,
    input  wire         ioctl_upload_req,    // unused (an open policy question: whether the firmware
    input  wire [7:0]   ioctl_upload_index,  //   ever honours a core-initiated save; this build only
    input  wire [7:0]   ioctl_din,           //   ever drives uploads on tango_link's own schedule)
    output reg          ioctl_rd = 1'b0,
    // Link side (clk_pix): tango_link's own UL_SELECT decision -- a toggle (ul_tog) + held data (ul_idx,
    // ul_size_l), the same idiom as tango_sd.v's mnt_tog/mnt_drive/mnt_ro/mnt_size -- and the upload buffer's
    // random-access read port (tango_link drives the address; mirrors tango_sd's out_ra/out_rd exactly).
    input  wire         ul_tog,
    input  wire [7:0]   ul_idx,
    input  wire [23:0]  ul_size_l,           // 0 = close
    input  wire [10:0]  ul_ra,
    output reg  [7:0]   ul_rd = 8'd0,
    output wire         ul_ready             // clk_pix: filled and open (for UL_STATUS)
`endif
);
    // ---- pads (gated while the OSD is open), clk_pix -> clk_sys
    reg [63:0] jp = 64'd0;
    always @(posedge clk_pix) jp <= osd_shown ? 64'd0 : {joy_a1, joy_a2};
    tango_cdc_word #(.W(11)) u_ps2 (.clk_a(clk_pix), .din(ps2_key_pix), .clk_b(clk_sys), .dout(ps2_key));
    reg osd_s0 = 1'b0;
    always @(posedge clk_sys) begin osd_s0 <= osd_shown; osd_status <= osd_s0; end
    wire [63:0] js;
    tango_cdc_word #(.W(64)) u_joy (.clk_a(clk_pix), .din(jp), .clk_b(clk_sys), .dout(js));
    assign joystick_0 = js[31:0];
    assign joystick_1 = js[63:32];

    // ---- status: the link holds the word between toggles
    reg [2:0] st_s = 3'b000;
    always @(posedge clk_sys) begin
        st_s <= {st_s[1:0], status_tog};
        if (st_s[2] != st_s[1]) status <= status_l;
    end

    // ---- menumask, clk_sys -> clk_pix
    tango_cdc_word #(.W(16)) u_mask (.clk_a(clk_sys), .din(status_menumask), .clk_b(clk_pix), .dout(menumask_l));

    // ---- download: FIFO of {ctrl, byte}: 01 start (byte = index), 10 end, 00 data
    wire [11:0] free_w;
    wire [9:0]  fq;
    wire        fempty;
    reg         frd = 1'b0, fv = 1'b0;
    tango_async_fifo #(.AW(11), .W(10)) u_fifo (
        .wclk(clk_pix), .wrst(1'b0), .we(dl_we), .wd(dl_wd), .free(free_w),
        .rclk(clk_sys), .rrst(1'b0), .rd(frd), .rdata(fq), .empty(fempty));
    assign dl_free = free_w;
    reg first = 1'b1;
`ifdef TANGO_IOCTL_UPLOAD
    // upload FSM state, declared here so the single always block below (which now also drives it) can see
    // it; the buffer/ready-crossing declarations stay below, unchanged, since nothing else needs them early.
    localparam integer UL_AW    = 11;
    localparam integer UL_DEPTH = (1 << UL_AW);
    localparam [1:0] U_IDLE = 2'd0, U_FILL = 2'd1;
    reg [1:0]       ust  = U_IDLE;
    reg [UL_AW-1:0] uwa  = {UL_AW{1'b0}};
    reg [23:0]      urem = 24'd0;
    reg [1:0]       uph  = 2'd0;
    reg [2:0]       ul_tog_s = 3'b000;
    wire            ul_new = (ul_tog_s[2] != ul_tog_s[1]);
    always @(posedge clk_sys) ul_tog_s <= {ul_tog_s[1:0], ul_tog};
    // pend_idx/pend_size: ul_idx/ul_size_l latched AT ul_new's own edge, not read live whenever this FSM
    // happens to act. tango_link's own idiom (mirrored from tango_sd.v's mnt_tog/mnt_drive) only promises
    // those two wires are stable until the NEXT ul_tog -- safe to sample right at the edge the toggle is
    // seen, not safe to re-read an arbitrary number of clocks later, because a SECOND UL_SELECT can land
    // on tango_link before this side ever acts on the
    // first one (deferred behind an open download, see ul_take below), tearing the capture between two
    // different requests' bytes. Latching here makes "the latest request wins" (ul_pend's own update,
    // unchanged) exact and glitch-free.
    reg [7:0]  pend_idx  = 8'd0;
    reg [23:0] pend_size = 24'd0;
    always @(posedge clk_sys) if (ul_new) begin pend_idx <= ul_idx; pend_size <= ul_size_l; end
    // ul_pend: a UL_SELECT (open or close) that arrived while blocked is remembered, not lost -- without
    // this, tango_link already latched the new ul_idx/ul_size_l and tango_link's own reply (UL_STATUS)
    // would report "not ready, size = <the new request's size>" forever, since nothing here would ever act
    // on it: tango_ul_pump's own "avail == 0 means give up" check (its comment) never fires either, because
    // avail is that nonzero size, not 0 -- an infinite poll, not a clean failure. ul_take (below) is the
    // single place that decides "consume the pending/new request this clock"; the sticky bit's own update
    // is a single assignment so a new ul_new arriving in the SAME clock a previous one is consumed is never
    // dropped (the set side of the OR always wins over the clear).
    reg ul_pend = 1'b0;
    // dl_busy: a download is open, OR a FIFO pop is already in flight (frd this clock, or fv -- the popped
    // entry not yet acted on). A close (pend_size == 0) is NEVER gated on this: it only ever lowers
    // ioctl_upload, touching nothing the download side reads, so it cannot deadlock against it and must
    // not be made to wait regardless of ust. Opening a NEW session (pend_size != 0) from U_IDLE DOES wait
    // on !dl_busy, not just !ioctl_download: gating on ioctl_download alone would leave a real deadlock --
    // close(A) and start(B) are two entries already queued in the SAME
    // download FIFO; ioctl_download drops to 0 for exactly one clock between them while B's own start entry
    // is still in flight (frd/fv), and taking the upload open in that one clock would let ioctl_download
    // rise again for B while ioctl_upload is already 1. Once both are 1: the download FIFO's own frd guard
    // (!ioctl_upload, below) stalls B's remaining bytes forever, and -- if a close were also gated on
    // !ioctl_download while idle -- the upload's own eventual close would then wait on
    // ioctl_download, which can now never fall either. dl_busy closes that window; a close being exempt
    // closes the rest of it, so neither side can ever wait on the other.
    wire dl_busy = ioctl_download | frd | fv;
    wire ul_take = ul_pend && ((ust == U_FILL) || (pend_size == 24'd0) || !dl_busy);
    always @(posedge clk_sys) ul_pend <= (ul_pend & ~ul_take) | ul_new;
`endif
    // the upload buffer: declared here, before its first use just below (and tango_sd.v's own out_buf
    // precedent, read by the link elsewhere in this file), rather than after -- Verilator accepts a forward
    // reference to a memory from inside an always block, but this file does not rely on Gowin's tools
    // accepting it too.
`ifdef TANGO_IOCTL_UPLOAD
    (* syn_ramstyle = "block_ram" *) reg [7:0] ul_buf [0:UL_DEPTH-1];
    // the buffer's write port, as a registered decode + a one-line write block -- tango_sd.v's own
    // ob_we/ob_wa/ob_wd idiom for out_buf, not `ul_buf[uwa] <= ioctl_din` inline inside the FSM's own case.
    // GW5A has no LUT-RAM: Gowin infers BSRAM from this exact shape
    // on tango_sd.v's own silicon-proven precedent; inlining the write address/data/enable computation
    // directly in a deep case arm is behaviourally identical but risks the inference Gowin performs on a
    // real build, which no simulation can check.
    reg              ub_we = 1'b0;
    reg [UL_AW-1:0]  ub_wa = {UL_AW{1'b0}};
    reg [7:0]        ub_wd = 8'd0;
    always @(posedge clk_sys) if (ub_we) ul_buf[ub_wa] <= ub_wd;
`endif
    // ioctl_index/ioctl_addr are shared between the download FIFO drain below and the upload fill engine
    // (TANGO_IOCTL_UPLOAD): ONE always block drives both, with the two paths mutually exclusive by
    // construction (an upload never opens while a download is busy, ul_take's own definition above; a
    // download never starts while an upload is open, frd's own extra `&& !ioctl_upload` term just below --
    // held in the FIFO, not lost, the same back-pressure ioctl_wait already gives it. MiSTer's own
    // ioctl_download/ioctl_upload are already documented as "never both asserted at once", tango_core_ports
    // .vh's own comment on the port). Two separate always blocks, each assigning the same reg, would
    // elaborate and simulate fine (Verilator does not flag MULTIDRIVEN across two same-clock blocks it
    // cannot prove are mutually exclusive), but is not valid synthesisable Verilog: a real Gowin build
    // would refuse it, or worse, resolve it in some tool-defined way.
    always @(posedge clk_sys) begin
        ioctl_wr <= 1'b0;
        fv  <= frd;                             // fq holds the popped entry one clock after frd
        frd <= 1'b0;
        if (!frd && !fv && !ioctl_wr && !fempty && !ioctl_wait
`ifdef TANGO_IOCTL_UPLOAD
            && !ioctl_upload && !ul_take
`endif
            ) frd <= 1'b1;
        if (fv) case (fq[9:8])
            2'b01: begin ioctl_download <= 1'b1; ioctl_index <= {8'd0, fq[7:0]}; ioctl_addr <= 27'd0; first <= 1'b1; end
            2'b10: ioctl_download <= 1'b0;
            default: begin
                ioctl_dout <= fq[7:0]; ioctl_wr <= 1'b1;
                ioctl_addr <= first ? 27'd0 : ioctl_addr + 27'd1; first <= 1'b0;
            end
        endcase
`ifdef TANGO_IOCTL_UPLOAD
        ioctl_rd <= 1'b0;
        ub_we <= 1'b0;
        case (ust)
            U_IDLE: if (ul_take) begin
                if (pend_size == 24'd0) begin
                    ioctl_upload <= 1'b0;                       // UL_SELECT close (or a close with none open)
                end else begin
                    ioctl_upload <= 1'b1; ioctl_index <= {8'd0, pend_idx}; ioctl_addr <= 27'd0;
                    uwa  <= {UL_AW{1'b0}};
                    urem <= (pend_size > UL_DEPTH[23:0]) ? UL_DEPTH[23:0] : pend_size;
                    uph  <= 2'd0; ust <= U_FILL;
                end
            end
            U_FILL: begin
                // A close, or a brand-new UL_SELECT, arriving mid-fill must not be ignored (ul_new
                // is a one-clock pulse off a toggle): the OLD request would keep filling to completion,
                // with the NEW one lost. A short fill (e.g. 37 bytes) finishes long before the next SPI
                // transfer could possibly arrive; a ~2 KB fill takes ~128 us of clk_sys time, well within
                // reach of a firmware issuing a close or a fresh open while one is still in flight. Handled
                // explicitly, via
                // the same ul_take this state is always allowed to act on (ul_take's own "ust == U_FILL"
                // term never waits on dl_busy, unlike U_IDLE's own open transition above):
                if (ul_take) begin
                    if (pend_size == 24'd0) begin ioctl_upload <= 1'b0; ust <= U_IDLE; end
                    else begin                      // restart the fetch for the new session, right here
                        ioctl_index <= {8'd0, pend_idx}; ioctl_addr <= 27'd0;
                        uwa <= {UL_AW{1'b0}};
                        urem <= (pend_size > UL_DEPTH[23:0]) ? UL_DEPTH[23:0] : pend_size;
                        uph <= 2'd0;                 // ioctl_upload, ust: already 1 / U_FILL
                    end
                end else if (urem == 24'd0) ust <= U_IDLE;
                else if (!ioctl_wait) begin
                    uph <= uph + 2'd1;
                    case (uph)
                        2'd0: ioctl_rd <= 1'b1;
                        2'd2: begin
                            ub_we <= 1'b1; ub_wa <= uwa; ub_wd <= ioctl_din;
                            ioctl_addr <= ioctl_addr + 27'd1; uwa <= uwa + 1'b1;
                            urem <= urem - 24'd1; uph <= 2'd0;
                        end
                        default: ;
                    endcase
                end
            end
            default: ust <= U_IDLE;
        endcase
`endif
    end

`ifdef TANGO_IOCTL_UPLOAD
    // ---- upload (TANGO_IOCTL_UPLOAD): a true dual-port BSRAM buffer (ul_buf), written here (clk_sys, pulled
    // from the core) and read by the link side (clk_pix), non-destructively, at whatever address tango_link's
    // own 2-bytes-ahead reply lookahead asks for -- the same structure tango_sd.v's out_buf/out_ra/out_rd
    // already proves on silicon for CORE_SECTOR_OUT, generalised from a fixed 512-byte block to a variable
    // size.
    //
    // Why not a FIFO (tango_async_fifo, the download direction's own building block, just reversed)? A FIFO's
    // read side is DESTRUCTIVE (rd pops one entry). tango_link's reply is always prepared two bytes ahead of
    // the byte actually being shifted out (tango_link.v's own "reply for byte n is prepared two bytes ahead"
    // rule) -- so at the end of every SPI transfer it computes two replies that are never sent (bytes beyond
    // the one the master actually stops at). If popping the FIFO were tied to that lookahead, every transfer
    // boundary would silently drop 2 real upload bytes the moment one logical upload is split across more
    // than one READ transfer. A
    // random-access buffer has no such hazard: reading (ahead_n - 3) over and over, including for the two
    // phantom ahead-of-time positions at the end of a transfer, has no side effect, because nothing is ever
    // consumed -- the buffer just holds the session's bytes until it is closed.
    //
    // Fill side (clk_sys): triggered by ul_tog, posted once per UL_SELECT decision (open: size != 0, close:
    // size == 0); ul_idx/ul_size_l are only guaranteed stable by tango_link until its NEXT UL_SELECT, so
    // they are latched into pend_idx/pend_size right at ul_new's own edge (above), never re-read later. On
    // open, pulls up to UL_DEPTH bytes from the core, one byte per 3 clk_sys clocks (pulse ioctl_rd, wait,
    // capture ioctl_din --
    // the same 3-phase idiom as tango_sd.v's S_FILL/S_COPY), giving ioctl_din >= 2 clk_sys clocks after the
    // address settles, as MiSTer cores assume registered RAM. This always finishes well before the firmware's
    // first READ transfer starts: UL_SELECT and the READ transfers that follow it are separate SPI transfers
    // with microseconds between them, orders of magnitude more slack than the ~1-SPI-byte-period margin the
    // reply-lookahead mechanism itself runs on, so the random-access read side never races the fill side. A
    // 2048-byte buffer (matching the download FIFO's own 2048-entry depth) comfortably covers realistic
    // NVRAM/cheat blobs; a session asking for more than that is silently clamped by tango_link.v's own copy
    // of UL_DEPTH (UL_MAX, which MUST stay in sync with this file's UL_DEPTH), which is what UL_STATUS
    // actually reports, so the firmware never believes more is available than this buffer actually holds.
    // ust/uwa/urem/uph (the fill engine's own state), ul_tog_s/ul_new (the UL_SELECT toggle detector),
    // pend_idx/pend_size (the latched request) and ul_buf itself are all declared above, next to the single
    // always block (shared with the download FIFO drain) that drives ioctl_index/ioctl_addr -- see that
    // block's own comment for why they could not stay separate, and why ul_buf had to move up with them.
    always @(posedge clk_pix) ul_rd <= ul_buf[ul_ra];      // link-side random-access read (1-cycle latency)
    // "ready" for UL_STATUS: filled (idle) and currently open. A single, slowly-changing level crossed
    // clk_sys -> clk_pix with a plain 3-flop synchroniser is the standard, sufficient idiom for one bit
    // (unlike a multi-bit bus such as a counter, which is why the download direction's tango_async_fifo
    // uses Gray-coded pointers instead -- there is no equivalent multi-bit hazard here).
    reg       ul_ready_r = 1'b0;
    always @(posedge clk_sys) ul_ready_r <= (ust == U_IDLE) && ioctl_upload;
    reg [2:0] ul_ready_s = 3'b000;
    always @(posedge clk_pix) ul_ready_s <= {ul_ready_s[1:0], ul_ready_r};
    assign ul_ready = ul_ready_s[2];
`endif
endmodule
`default_nettype wire
