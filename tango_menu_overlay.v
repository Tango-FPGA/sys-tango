// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_menu_overlay -- the Tango menu renderer for the menu core, fed entirely by
// the BL616 over the SPI link (tango_link).  No built-in page: the atlas, both glyph
// banks, both row banks, the palette and the geometry are written by OSD commands
// (menu-tango docs/TANGO-SPEC.md "OSD commands (target 2)", "FPGA memory", "Renderer").
//
// Derived from tango_overlay.v (the launcher's earlier overlay, gated 0 mismatches vs the model), with:
//   - the TEST-ONLY start-up copier and its logic ROMs removed; the three BSRAMs
//     get a write port driven by the link (GW5A BSRAM initial contents do not
//     survive on silicon, so nothing here relies on RAM init);
//   - geometry (x0, y0, w, h, strip width, row pitch, top offset) held in registers,
//     reset to the Cores-page values of tango_ov_geom.vh, loaded by GEOMETRY;
//   - staged palette/geometry/enable and a swap request, all applied together at the
//     frame boundary FB = lead beam (cx 1, cy VIS_H), i.e. the first vertical-blank
//     line, before the setup of any panel line of the next frame.  A PALETTE or
//     GEOMETRY transfer still in progress at FB (stage_busy) is not split: it is
//     applied at the next FB.  SWAP/ENABLE are single bytes and always atomic;
//   - the per-line setup trigger moved from x 1100 to SETUP_X 1288 (just after the
//     1280 visible pixels + pipeline), so a panel may span the whole 1280 width.
// Unchanged: the per-line FSM, the glyph walker, the 6-clock pixel pipeline, the
// shift-add blends, the corner/cap masks (fixed logic ROMs, tango_ov_geom.vh).
//
// Memories (all BSRAM, one write port from the link, one read port for the renderer,
// same clock):
//   atlas  1024 x ATW  column-major glyph columns, bit[2*gy+1:2*gy] = coverage of row gy
//   glist  2048 x 32   2 banks x 1024 glyph entries {spare,colour,width,colbase,x}
//   rtab    128 x 32   2 banks x 32 rows x 2 words (word 2r = entry[31:0], 2r+1 = [63:32]);
//            content rows (0-30) pack first glyph = bits[8:0] of word 2r plus bit 9 stolen from
//            word 2r+1 bit 30 (that bit was always unused: texty there is <= 5 bits in practice,
//            menu-tango docs/TANGO-SPEC.md "FPGA memory" marks 63:62 reserved); row 31 (banner) has no bar/rule
//            fields, so it packs first = bits[9:0] / count = bits[16:10] of word 2r directly, no
//            stolen bit needed.
//   palette 16 x RGB888 + K (index 16) in flops, staged + live
// Palette roles fixed in logic: index OV_STRIP_COL (6) = strip / accent colour;
// index 16 = K = the 3/4 panel colour used for the translucent blend (game>>2 + K).
//
// Beam interface as tango_overlay (see there): cx/cy lead hdmi.sv by LAT-1 clocks.
//==============================================================================
`default_nettype none
module tango_menu_overlay #(
    parameter integer FW      = 1650,   // hdmi.sv 720p frame_width
    parameter integer FH      = 750,    // hdmi.sv 720p frame_height
    parameter integer VIS_H   = 720,    // visible lines: FB = first blanking line
    parameter integer SETUP_X = 1288,   // line setup starts here (after the 1280 visible + pipeline)
    parameter integer ATW     = 48      // atlas word width = 2 * glyph rows (must equal 2*OV_SH)
) (
    input  wire        clk,             // clk_pix
    input  wire        rst,
    input  wire [11:0] cx,
    input  wire [10:0] cy,
`ifdef TANGO_VSYNC_ADJUST
    input  wire [10:0] fh_rt,           // the raster's height at runtime (hdmi.sv frame_height; replaces FH): this beam
                                        // is 5 clocks ahead of hdmi.sv's, and FH is only read at trig (x SETUP_X)
`endif
    input  wire [23:0] bg,              // background for (cx, cy) of the previous clock
    output reg  [23:0] rgb = 24'd0,     // composited pixel for (cx, cy) of LAT clocks ago
    // ---- link write side (clk domain; tango_link) ----
    input  wire            at_we,  input wire [9:0] at_wa,  input wire [ATW-1:0] at_wd,
    input  wire            gl_we,  input wire [10:0] gl_wa, input wire [31:0]    gl_wd,
    input  wire            rt_we,  input wire [6:0] rt_wa,  input wire [31:0]    rt_wd,
    input  wire            pal_we, input wire [4:0] pal_wa, input wire [23:0]    pal_wd,
    input  wire            geo_we, input wire [3:0] geo_wa, input wire [7:0]     geo_wd,
    input  wire            en_we,  input wire       en_wd,
    input  wire            swap_we, input wire      swap_wd,
    input  wire            stage_busy,  // a PALETTE / GEOMETRY transfer is in progress
    output reg             shown_bank = 1'b0,
    output reg             swap_pending = 1'b0,
    output reg             shown = 1'b0 // ENABLE as applied
) /* synthesis syn_romstyle = "logic" */;   // corner/cap mask ROMs: never pROM
    `include "tango_ov_geom.vh"
    localparam integer LAT = 6;
    localparam integer BANNER_ROW = 31;

    // ======================= memories ====================================
    (* syn_ramstyle = "block_ram" *) reg [ATW-1:0] atlas [0:1023];
    (* syn_ramstyle = "block_ram" *) reg [31:0]    glist [0:2047];
    (* syn_ramstyle = "block_ram" *) reg [31:0]    rtab  [0:127];
`ifdef TANGO_SIM_RANDOM_RAM
    // Simulation only: silicon BSRAM powers up with arbitrary contents, so the sim
    // starts every RAM with garbage.  Never seen by synthesis (macro undefined there).
    integer ri;
    initial begin
        for (ri = 0; ri < 1024; ri = ri + 1) atlas[ri] = {$urandom, $urandom};
        for (ri = 0; ri < 2048; ri = ri + 1) glist[ri] = $urandom;
        for (ri = 0; ri < 128; ri = ri + 1) rtab[ri] = $urandom;
    end
`endif
    always @(posedge clk) if (at_we) atlas[at_wa] <= at_wd;
    always @(posedge clk) if (gl_we) glist[gl_wa] <= gl_wd;
    always @(posedge clk) if (rt_we) rtab[rt_wa]  <= rt_wd;

    // ======================= staged + live state ==========================
    reg [23:0] pal_st [0:16];           // staged palette (16 = K)
    reg [7:0]  geo_st [0:11];           // staged GEOMETRY bytes (big-endian fields)
    reg        pal_dirty = 1'b0, geo_dirty = 1'b0;
    reg        en_st = 1'b0, swap_bank_st = 1'b0;
    reg [23:0] pal [0:15];              // live palette
    reg [23:0] kpan = 24'd0;            // live K
    // live geometry (reset = the Cores page of tango_ov_geom.vh)
    reg [10:0] g_x0 = OV_X0, g_y0 = OV_Y0, g_w = OV_PW, g_h = OV_PH;
    reg [7:0]  g_sw = OV_SW, g_rh = OV_RH, g_top = OV_TOP;
    // derived, registered one clock after the live values (they only change at FB)
    reg [11:0] d_x1 = OV_X0 + OV_PW;
    reg [10:0] d_y1 = OV_Y0 + OV_PH;
    reg [10:0] d_pw_m1 = OV_PW - 1, d_pw_m16 = OV_PW - 16, d_ph_m1 = OV_PH - 1, d_ph_m16 = OV_PH - 16;
    reg [4:0]  d_rh_m1 = OV_RH - 1;
    // FB registered (commit at lead beam (1, VIS_H)): the decode from the beam counters must not
    // drive the ~450 clock enables of the staged -> live copy in the same clock (it was the worst
    // clk_pix path of build g2e8bd8f, +0.208 ns).  Still in the blanking, before any line setup.
    reg        fb = 1'b0;
    always @(posedge clk) fb <= (cx == 12'd0) && (cy == VIS_H);
    integer k;
    always @(posedge clk) begin
        if (rst) begin
            pal_dirty <= 1'b0; geo_dirty <= 1'b0; en_st <= 1'b0; shown <= 1'b0;
            swap_pending <= 1'b0; shown_bank <= 1'b0; swap_bank_st <= 1'b0;
            g_x0 <= OV_X0; g_y0 <= OV_Y0; g_w <= OV_PW; g_h <= OV_PH;
            g_sw <= OV_SW; g_rh <= OV_RH; g_top <= OV_TOP;
        end else begin
            // ---- frame boundary: apply what was staged (writes below win the same clock)
            if (fb) begin
                shown <= en_st;
                if (swap_pending) begin shown_bank <= swap_bank_st; swap_pending <= 1'b0; end
                if (!stage_busy && pal_dirty) begin
                    for (k = 0; k < 16; k = k + 1) pal[k] <= pal_st[k];
                    kpan <= pal_st[16];
                    pal_dirty <= 1'b0;
                end
                if (!stage_busy && geo_dirty) begin
                    g_x0  <= {geo_st[0][2:0], geo_st[1]};
                    g_y0  <= {geo_st[2][2:0], geo_st[3]};
                    g_w   <= {geo_st[4][2:0], geo_st[5]};
                    g_h   <= {geo_st[6][2:0], geo_st[7]};
                    g_sw  <= geo_st[8];
                    g_rh  <= geo_st[9];
                    g_top <= geo_st[10];
                    // geo_st[11] (text baseline row): not used by the renderer; text y is per row
                    geo_dirty <= 1'b0;
                end
            end
            // ---- link writes (staged)
            if (pal_we && pal_wa <= 5'd16) begin pal_st[pal_wa] <= pal_wd; pal_dirty <= 1'b1; end
            if (geo_we && geo_wa <= 4'd11) begin geo_st[geo_wa] <= geo_wd; geo_dirty <= 1'b1; end
            if (en_we) en_st <= en_wd;
            if (swap_we) begin swap_bank_st <= swap_wd; swap_pending <= 1'b1; end
        end
        d_x1     <= {1'b0, g_x0} + {1'b0, g_w};
        d_y1     <= g_y0 + g_h;
        d_pw_m1  <= g_w - 11'd1;
        d_pw_m16 <= g_w - 11'd16;
        d_ph_m1  <= g_h - 11'd1;
        d_ph_m16 <= g_h - 11'd16;
        d_rh_m1  <= g_rh[4:0] - 5'd1;
    end
    wire bank = shown_bank;

    // ======================= read ports ===================================
    reg  [9:0]  at_ra;  reg [ATW-1:0] at_rd;
    reg  [10:0] gl_ra;  reg [31:0] gl_rd;
    reg  [6:0]  rt_ra;  reg [31:0] rt_rd;
    always @(posedge clk) at_rd <= atlas[at_ra];
    always @(posedge clk) gl_rd <= glist[gl_ra];
    always @(posedge clk) rt_rd <= rtab[rt_ra];

    // ======================= line setup FSM ===============================
    localparam [3:0] S_IDLE = 4'd0, S_RLO = 4'd1, S_RHI = 4'd2, S_B0 = 4'd3, S_B1 = 4'd4,
                     S_B2 = 4'd5, S_SCAN = 4'd6, S_AF = 4'd7, S_AD = 4'd8, S_TXT = 4'd9, S_STREAM = 4'd10;
    reg [3:0] st = S_IDLE;
    wire      trig = (cx == SETUP_X);
    // next line (cy + 1), panel-local.  Registered one clock ahead: they are constant along a
    // line and only used at trig (x SETUP_X), and the combinational chain cy -> +1 -> -y0 ->
    // compares -> row counters was the worst clk_pix path of debug build g66c6783 (-0.782 ns).
`ifdef TANGO_VSYNC_ADJUST
    // the runtime height minus one, REGISTERED: fh_rt (hdmi.sv's frame_height) changes only at the raster's wrap (local
    // modification 3), and ny_c's wrap compare matters only on the frame's last line, so one clock of latency is invisible
    // here; the subtraction (and the logic the synthesiser shares with tango_beam_lead_rt) leaves the clk_pix path
    // fh_r -> ny_c -> n_in (18 levels: an Atari 2600 TANGO_VSYNC_ADJUST build missed clk_pix by 0.404 ns on it)
    reg  [10:0] fh_m1 = 11'(FH - 1);
    always @(posedge clk) fh_m1 <= fh_rt - 11'd1;
    wire [10:0] ny_c  = (cy == fh_m1) ? 11'd0 : cy + 11'd1;
    /* synthesis translate_off */
    // THE EQUIVALENCE CHECK (sims only): ny_c against the combinational original at every clock.  This beam (tango_sys's
    // lcx / lcy) runs ahead of hdmi.sv's, so when fh_rt changes at hdmi.sv's wrap this beam is already on the new frame's
    // line 0 and fh_m1's one stale clock cannot meet the last-line compare; the counts show the case was exercised.
    wire [10:0] ny_c_ref = (cy == fh_rt - 11'd1) ? 11'd0 : cy + 11'd1;
    reg  [10:0] fh_q = 11'd0;
    integer     ov_fh_changes = 0, ov_ny_bad = 0, ov_lastline = 0;
    always @(posedge clk) begin
        fh_q <= fh_rt;
        if (fh_q != 11'd0 && fh_rt != fh_q) ov_fh_changes = ov_fh_changes + 1;
        if (cy == fh_rt - 11'd1 && cx == 12'd0) ov_lastline = ov_lastline + 1;
        if (ny_c !== ny_c_ref) begin
            ov_ny_bad = ov_ny_bad + 1;
            if (ov_ny_bad <= 5)
                $display("TANGO OVERLAY: ny_c %0d != the combinational %0d at beam (%0d, %0d), fh_rt %0d, fh_m1 %0d (%m, %0t)",
                         ny_c, ny_c_ref, cx, cy, fh_rt, fh_m1, $time);
        end
    end
    final $display("TANGO OVERLAY: registered fh_rt - 1 == the combinational compare at every clock: %0s (%0d mismatches; %0d raster-height changes, %0d last lines seen) (%m)",
                   ov_ny_bad == 0 ? "YES" : "NO", ov_ny_bad, ov_fh_changes, ov_lastline);
    /* synthesis translate_on */
`else
    wire [10:0] ny_c  = (cy == FH - 1) ? 11'd0 : cy + 11'd1;
`endif
    reg  [10:0] nly = 11'd0;                            // valid when n_in
    reg         n_in = 1'b0, n_top = 1'b0, n_after = 1'b0;
    always @(posedge clk) begin
        nly     <= ny_c - g_y0;
        n_in    <= (ny_c >= g_y0) && (ny_c < d_y1);
        n_top   <= (ny_c - g_y0) == {3'd0, g_top};
        n_after <= (ny_c - g_y0) >  {3'd0, g_top};
    end
    // row counters for the line being set up
    reg  [5:0]  s_ri = 6'd0;  reg [4:0] s_ty = 5'd0;  reg s_row = 1'b0;
    reg  [10:0] s_ly = 11'd0; reg s_in = 1'b0;
    wire        s_rowok = s_row && (s_ri < BANNER_ROW);
    reg  [31:0] s_lo = 32'd0;
    // banner scan
    reg  [9:0]  sp = 10'd0, send = 10'd0;
    reg         sp_q = 1'b0;                           // a scan read is in flight
    reg         bm_v = 1'b0;  reg [9:0] bm_a = 10'd0;  reg [3:0] bm_c = 4'd0;
    // the entry read back, registered before the match (clk_pix margin, set 2: glist DO -> e_d subtract ->
    // compare -> bm_* enables was the menu build's worst path, +0.831 ns); the scan waits for it (sc_v)
    reg         sc_v = 1'b0;  reg [11:0] sc_d = 12'd0;  reg [4:0] sc_w = 5'd0;  reg [10:0] sc_cb = 11'd0;
    reg  [3:0]  sc_c = 4'd0;
    // glyph entry fields of the data coming back
    wire [10:0] e_x  = gl_rd[10:0];
    wire [10:0] e_cb = gl_rd[21:11];
    wire [4:0]  e_w  = gl_rd[26:22];
    wire [3:0]  e_c  = gl_rd[30:27];
    wire [11:0] e_d  = {1'b0, e_x} - {1'b0, s_ly};      // banner: column = bottom y - line

    // ---- current-line registers (written after the panel of the line before) ----
    reg        l_in = 1'b0;                 // line inside the panel
    reg        l_myv = 1'b0;  reg [3:0] l_my = 4'd0;
    reg        l_rule = 1'b0; reg [3:0] l_rcol = 4'd0;
    reg        l_bar = 1'b0;  reg [3:0] l_bcol = 4'd0;
    reg [10:0] l_bx0 = 11'd0, l_bx1 = 11'd0;
    reg [4:0]  l_yy = 5'd0;
    reg        l_gl = 1'b0;   reg [4:0] l_gy = 5'd0;
    reg        l_bnv = 1'b0;  reg [3:0] l_bnc = 4'd0; reg [5:0] l_bx = 6'd0;
    reg [ATW-1:0] l_bnw = 0;
    reg [31:0] l_crow = 32'hffffffff;       // corner mask row for the line
    reg [2*OV_CAPL-1:0] l_capl = 0;
    reg [2*OV_CAPR-1:0] l_capr = 0;
    // text stream
    reg [9:0]  tp_ptr = 10'd0, tp_end = 10'd0;
    reg        tp_q = 1'b0;                 // a stream read is in flight
    reg [31:0] ff [0:3];
    reg [1:0]  ff_rp = 2'd0, ff_wp = 2'd0;
    reg [2:0]  ff_n = 3'd0;

    // decoded row entry (s_lo = [31:0] latched, rt_rd = [63:32] arriving in S_B0)
    wire        r_B    = s_lo[16];
    wire [3:0]  r_bcol = s_lo[20:17];
    wire [10:0] r_bx0  = s_lo[31:21];
    wire [10:0] r_bx1  = rt_rd[10:0];
    wire [2:0]  r_ins  = rt_rd[13:11];
    wire        r_R    = rt_rd[14];
    wire [3:0]  r_rcol = rt_rd[18:15];
    wire [4:0]  r_ry   = rt_rd[23:19];
    wire [5:0]  r_ty   = rt_rd[29:24];
    wire [6:0]  r_gy   = {2'b00, s_ty} - {1'b0, r_ty};     // gy = ty - texty (7-bit two's complement)
    wire [4:0]  r_yy   = s_ty - {2'b00, r_ins};
    wire [7:0]  r_bend = g_rh - {5'd0, r_ins};             // bar lines: ins <= ty < RH - ins

    // stream: pop at s0 (below) + push of returning reads
    wire        pop;
    wire        tp_issue = (st == S_STREAM) && (tp_ptr != tp_end) && ({1'b0, ff_n} + {3'b000, tp_q} < 4'd4);

    always @(*) begin
        rt_ra = 7'd0;
        case (st)
            S_RLO: rt_ra = {bank, s_ri[4:0], 1'b0};
            S_RHI: rt_ra = {bank, s_ri[4:0], 1'b1};
            S_B0:  rt_ra = {bank, 5'd31, 1'b0};
            default: rt_ra = {bank, 5'd31, 1'b1};
        endcase
        gl_ra = (st == S_SCAN) ? {bank, sp} : {bank, tp_ptr};
    end

    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; s_row <= 1'b0; tp_q <= 1'b0; sp_q <= 1'b0;
            ff_rp <= 2'd0; ff_wp <= 2'd0; ff_n <= 3'd0; tp_ptr <= 9'd0; tp_end <= 9'd0;
            l_in <= 1'b0;
        end else begin
            tp_q <= 1'b0;
            if (trig) begin
                // row counters for line ny: row 0 starts at TOP; ty wraps at RH
                s_ly <= nly; s_in <= n_in;
                if (n_in && n_top) begin s_row <= 1'b1; s_ri <= 6'd0; s_ty <= 5'd0; end
                else if (!(n_in && n_after)) s_row <= 1'b0;
                else if (s_ty == d_rh_m1) begin s_ty <= 5'd0; s_ri <= s_ri + 6'd1; end
                else s_ty <= s_ty + 5'd1;
                st <= S_RLO;
            end else case (st)
                S_RLO:  st <= S_RHI;
                S_RHI:  begin s_lo <= s_rowok ? rt_rd : 32'd0; st <= S_B0; end
                S_B0:   begin                                   // rt_rd = row entry [63:32]
                    l_in   <= s_in;
                    l_myv  <= s_in && (s_ly < 16 || s_ly >= d_ph_m16);
                    l_my   <= (s_ly < 16) ? s_ly[3:0] : (d_ph_m1[3:0] - s_ly[3:0]);
                    l_rule <= s_rowok && r_R && ({1'b0, s_ty} == {1'b0, r_ry});
                    l_rcol <= r_rcol;
                    l_bar  <= s_rowok && r_B && (s_ty >= r_ins) && ({3'd0, s_ty} < r_bend);
                    l_bcol <= r_bcol; l_bx0 <= r_bx0; l_bx1 <= r_bx1; l_yy <= r_yy;
                    l_gl   <= s_rowok && !r_gy[6] && (r_gy < OV_SH);
                    l_gy   <= r_gy[4:0];
                    // first glyph = s_lo[8:0] with bit 9 stolen from this row's HI word bit 30
                    // (rt_rd here = the content row's HI word, fetched using S_RHI's address;
                    // see the file header "Memories" note on the row table's packing).
                    tp_ptr <= s_rowok ? {rt_rd[30], s_lo[8:0]} : 10'd0;
                    tp_end <= s_rowok ? ({rt_rd[30], s_lo[8:0]} + {3'b000, s_lo[15:9]}) : 10'd0;
                    st <= S_B1;
                end
                S_B1:   begin sp <= rt_rd[9:0]; send <= rt_rd[9:0] + {3'b000, rt_rd[16:10]}; bm_v <= 1'b0; st <= S_B2; end  // banner: first[9:0], count[16:10] (no bar/rule fields in row 31, so no stolen bit)
                S_B2:   begin l_bx <= rt_rd[29:24]; sp_q <= 1'b0; st <= S_SCAN; end
                S_SCAN: begin
                    sp_q <= (sp != send);
                    if (sp != send) sp <= sp + 9'd1;
                    if (sc_v && !sc_d[11] && (sc_d[10:0] < {6'd0, sc_w})) begin
                        bm_v <= 1'b1; bm_a <= sc_cb[9:0] + sc_d[9:0]; bm_c <= sc_c;
                    end
                    if (sp == send && !sp_q && !sc_v) st <= S_AF;
                end
                S_AF:   st <= S_AD;                             // at_ra = bm_a
                S_AD:   begin l_bnw <= at_rd; st <= S_TXT; end  // at_rd = banner column (1 clock only)
                S_TXT:  begin
                    l_bnv <= bm_v && s_in; l_bnc <= bm_c;
                    ff_rp <= 2'd0; ff_wp <= 2'd0; ff_n <= 3'd0;
                    st <= S_STREAM;
                end
                default: ;                                      // S_IDLE, S_STREAM
            endcase
            // text stream FIFO (S_STREAM only; S_TXT clears it)
            if (st == S_STREAM) begin
                if (tp_issue) begin tp_ptr <= tp_ptr + 9'd1; tp_q <= 1'b1; end
                if (tp_q) begin ff[ff_wp] <= gl_rd; ff_wp <= ff_wp + 2'd1; end
                if (pop) ff_rp <= ff_rp + 2'd1;
                ff_n <= ff_n + {2'b00, tp_q} - {2'b00, pop};
            end
        end
    end
    // the scan's pipeline stage: entry fields of a scan read (sp_q) one clock later
    always @(posedge clk) begin
        sc_v <= !rst && (st == S_SCAN) && sp_q;
        sc_d <= e_d; sc_w <= e_w; sc_cb <= e_cb; sc_c <= e_c;
    end
    // corner row + cap rows follow the line registers (fixed logic ROMs)
    always @(posedge clk) begin
        l_crow <= l_myv ? ov_corner_row(l_my) : 32'hffffffff;
        l_capl <= ov_capl_row(l_yy);
        l_capr <= ov_capr_row(l_yy);
    end

    // ======================= per-line colours (registered, stable in a line) ===
    function [7:0] mix8(input [7:0] a, input [7:0] b, input [1:0] lv);
        reg signed [12:0] d, p;
        begin
            d = $signed({5'd0, b}) - $signed({5'd0, a});
            case (lv)
                2'd0: p = 13'sd0;
                2'd1: p = (d <<< 1) + d;        // x3
                2'd2: p = (d <<< 2) + d;        // x5
                default: p = d <<< 3;           // x8
            endcase
            p = p >>> 3;                         // floor, as Python >>
            mix8 = a + p[7:0];
        end
    endfunction
    function [23:0] mix24(input [23:0] a, input [23:0] b, input [1:0] lv);
        mix24 = {mix8(a[23:16], b[23:16], lv), mix8(a[15:8], b[15:8], lv), mix8(a[7:0], b[7:0], lv)};
    endfunction
    // c_bn*: the banner colour lookup is registered (c_bnp) before the blends (clk_pix margin, set 2: l_bnc ->
    // 16:1 palette mux -> mix24 -> c_bn1/2 was on the NES build's worst list), so they follow l_bnc (S_TXT) by
    // 2 clocks instead of 1; the line's first pixel is still far behind (S_STREAM fills the text FIFO first).
    reg [23:0] c_acc = 0, c_rule = 0, c_bar = 0, c_bnp = 0, c_bn1 = 0, c_bn2 = 0, c_bn3 = 0;
    always @(posedge clk) begin
        c_acc  <= pal[OV_STRIP_COL];
        c_rule <= pal[l_rcol];
        c_bar  <= pal[l_bcol];
        c_bnp  <= pal[l_bnc];
        c_bn1  <= mix24(c_acc, c_bnp, 2'd1);
        c_bn2  <= mix24(c_acc, c_bnp, 2'd2);
        c_bn3  <= mix24(c_acc, c_bnp, 2'd3);
    end

    // ======================= pixel pipeline ===============================
    // s0 (beam at this clock): walker
    wire [11:0] lx0  = cx - {1'b0, g_x0};
    wire        inx0 = (cx >= {1'b0, g_x0}) && (cx < d_x1);
    wire [31:0] ffh  = ff[ff_rp];
    assign pop = (st == S_STREAM) && (ff_n != 3'd0) && ({1'b0, ffh[10:0]} == lx0);
    reg        cur_v = 1'b0;
    reg [10:0] cur_x = 11'd0, cur_cb = 11'd0;
    reg [4:0]  cur_w = 5'd0;  reg [3:0] cur_c = 4'd0;
    reg [10:0] s1_lx = 11'd0; reg s1_in = 1'b0;
    always @(posedge clk) begin
        if (st == S_TXT) cur_v <= 1'b0;
        else if (pop) begin
            cur_v <= 1'b1; cur_x <= ffh[10:0]; cur_cb <= ffh[21:11]; cur_w <= ffh[26:22]; cur_c <= ffh[30:27];
        end
        s1_lx <= lx0[10:0];
        s1_in <= inx0 && l_in && shown;
    end
    // s0 -> s1 precompute (clk_pix margin): everything s1 decides from the pixel position alone is computed here
    // from lx0 (= s1_lx's D) and registered with it, so s1 is only flops -> small muxes -> s2 (s1_lx -> strip /
    // banner compare -> banner column select -> 5:1 colour mux -> s2_c was the worst clk_pix path of the
    // launcher build it was measured in, +0.592 ns).  The line registers used here (l_bx, l_bnw, l_bnv, l_bar, l_bx0, l_bx1,
    // c_acc, c_bn*) are thus read one clock earlier than before; they only change during the line setup, which
    // starts at SETUP_X (after the 1280 visible pixels), so no panel pixel sees a difference.  Same latency.
    wire [10:0] lx0s   = lx0[10:0];
    wire        strip0 = ({3'd0, lx0s} < {6'd0, g_sw});
    reg  [10:0] bx_abs = 11'd0;                                   // g_x0 + l_bx: banner column 0 on the beam
    always @(posedge clk) bx_abs <= g_x0 + {5'd0, l_bx};
    // by0_r: pre-register to cut the cx carry chain off the s1_scol path (clk_pix margin, set 3:
    // cx[0] -> 11-bit subtract -> compare -> bit-select -> 24-bit mux -> s1_scol was -0.127 ns in
    // the ASCAL NES build).  Load cx - bx_abs + 1 so one cycle later by0_r = cx(current) - bx_abs
    // (bx_abs is stable during active pixels; cx +1 each cycle so +1 corrects the 1-cycle advance).
    reg  [10:0] by0_r  = 11'd0;
    always @(posedge clk) by0_r <= cx[10:0] - bx_abs + 11'd1;
    wire        bnin0  = l_bnv && (by0_r < OV_SH);
    wire [1:0]  bnc0   = bnin0 ? l_bnw[{by0_r[4:0], 1'b0} +: 2] : 2'd0;
    reg  [23:0] scol0;
    always @(*) case (bnc0)
        2'd0: scol0 = c_acc;
        2'd1: scol0 = c_bn1;
        2'd2: scol0 = c_bn2;
        default: scol0 = c_bn3;
    endcase
    wire [10:0] d0     = lx0s - l_bx0;
    wire [10:0] dr0    = l_bx1 - 11'd1 - lx0s;
    wire [10:0] mxr0   = d_pw_m1 - lx0s;
    reg         s1_strip = 1'b0;  reg [23:0] s1_scol = 24'd0;         // strip, and its colour at this x
    reg         s1_inb = 1'b0, s1_bfull = 1'b0, s1_bleft = 1'b0;  reg [4:0] s1_d = 5'd0, s1_dr = 5'd0;   // bar
    reg         s1_mxv = 1'b0;  reg [3:0] s1_mx = 4'd0;                // corner column
    always @(posedge clk) begin
        s1_strip <= strip0;
        s1_scol  <= scol0;
        s1_inb   <= l_bar && !strip0 && (lx0s >= l_bx0) && (lx0s < l_bx1);
        s1_bfull <= (d0 >= OV_CAPL) && (dr0 >= OV_CAPR);
        s1_bleft <= (d0 < OV_CAPL);
        s1_d     <= d0[4:0];  s1_dr <= dr0[4:0];
        s1_mxv   <= (lx0s < 16) || (lx0s >= d_pw_m16);
        s1_mx    <= (lx0s < 16) ? lx0s[3:0] : mxr0[3:0];
    end

    // s1: bg = pixel; atlas address; base colour; bar + corner coverage
    wire [10:0] sx1   = s1_lx - cur_x;
    wire        strip1 = s1_strip;
    wire        gv1   = cur_v && l_gl && !strip1 && (sx1 < {6'd0, cur_w});
    always @(*) at_ra = (st == S_AF) ? bm_a : (cur_cb[9:0] + sx1[9:0]);
    wire [23:0] trans1 = {bg[23:18], bg[15:10], bg[7:2]};         // bg >> 2 per channel (6-bit)
    wire [23:0] tk1   = {{2'b00, trans1[17:12]} + kpan[23:16],
                         {2'b00, trans1[11:6]}  + kpan[15:8],
                         {2'b00, trans1[5:0]}   + kpan[7:0]};
    wire [23:0] base1 = strip1 ? s1_scol : tk1;
    reg  [1:0]  bcov1;
    always @(*) begin
        if (!s1_inb) bcov1 = 2'd0;
        else if (s1_bfull) bcov1 = 2'd3;
        else if (s1_bleft) bcov1 = l_capl[{s1_d, 1'b0} +: 2];
        else bcov1 = l_capr[{s1_dr, 1'b0} +: 2];
    end
    wire [1:0]  cov1  = s1_mxv ? l_crow[{s1_mx, 1'b0} +: 2] : 2'd3;

    reg [23:0] s2_c = 0, s2_g = 0;  reg [1:0] s2_bcov = 0, s2_cov = 0;
    reg        s2_gv = 0, s2_rule = 0, s2_in = 0;  reg [3:0] s2_gc = 0;
    always @(posedge clk) begin
        s2_c <= base1; s2_g <= bg; s2_bcov <= bcov1; s2_cov <= cov1;
        s2_gv <= gv1; s2_gc <= cur_c; s2_rule <= l_rule && !strip1; s2_in <= s1_in;
    end
    // s2: glyph coverage from the atlas word; rule mix
    wire [1:0] a2 = s2_gv ? at_rd[{l_gy, 1'b0} +: 2] : 2'd0;
    reg [23:0] s3_c = 0, s3_g = 0, s3_gp = 0;  reg [1:0] s3_bcov = 0, s3_cov = 0, s3_a = 0;  reg s3_in = 0;
    always @(posedge clk) begin
        s3_c <= s2_rule ? mix24(s2_c, c_rule, 2'd2) : s2_c;
        s3_a <= a2; s3_gp <= pal[s2_gc]; s3_bcov <= s2_bcov; s3_cov <= s2_cov; s3_g <= s2_g; s3_in <= s2_in;
    end
    // s3: bar mix
    reg [23:0] s4_c = 0, s4_g = 0, s4_gp = 0;  reg [1:0] s4_cov = 0, s4_a = 0;  reg s4_in = 0;
    always @(posedge clk) begin
        s4_c <= mix24(s3_c, c_bar, s3_bcov);
        s4_a <= s3_a; s4_gp <= s3_gp; s4_cov <= s3_cov; s4_g <= s3_g; s4_in <= s3_in;
    end
    // s4: glyph mix
    reg [23:0] s5_c = 0, s5_g = 0;  reg [1:0] s5_cov = 0;  reg s5_in = 0;
    always @(posedge clk) begin
        s5_c <= mix24(s4_c, s4_gp, s4_a);
        s5_cov <= s4_cov; s5_g <= s4_g; s5_in <= s4_in;
    end
    // s5: corner edge blend, output register
    function [7:0] edge8(input [7:0] g, input [7:0] c, input [1:0] cov);
        reg [9:0] g3;
        begin
            g3 = {1'b0, g, 1'b0} + {2'b00, g};                   // 3g
            case (cov)
                2'd0: edge8 = g;
                2'd1: edge8 = g3[9:2] + {2'b00, c[7:2]};         // (3g>>2) + (c>>2)
                2'd2: edge8 = {1'b0, g[7:1]} + {1'b0, c[7:1]};   // (g>>1) + (c>>1)
                default: edge8 = c;
            endcase
        end
    endfunction
    always @(posedge clk)
        rgb <= !s5_in ? s5_g : {edge8(s5_g[23:16], s5_c[23:16], s5_cov),
                                edge8(s5_g[15:8],  s5_c[15:8],  s5_cov),
                                edge8(s5_g[7:0],   s5_c[7:0],   s5_cov)};
endmodule
`default_nettype wire
