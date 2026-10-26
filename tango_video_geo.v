// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 the Tango authors
// Copyright (c) 2020 Grabulosaure, (c) 2021 Alexey Melnikov (sys/video_freak.sv: video crop)
// Copyright (c) 2021 Alexey Melnikov (sys/video_freak.sv: integer scaling)
// (c)2017-2020 Alexey Melnikov (sys/sys_top.v)
//
// This file is derived from MiSTer's picture placement: the MiSTer project's Template_MiSTer
// (https://github.com/MiSTer-devel/Template_MiSTer, compared at commit 5dda373), sys/video_freak.sv (modules
// video_freak and video_scale_int: crop, aspect and the SCALE modes, in their order and with their names: vcrop,
// voff, arxo, aryo, htarget, wres, ...) and sys/sys_top.v 856-944 (the window from the aspect or size).
// video_freak.sv states "This program is GPL Licensed. See COPYING for the full license." without a version
// (Template_MiSTer has no COPYING; its LICENSE file is the text of the GNU General Public License version 2);
// sys_top.v states "either version 2 of the License, or (at your option) any later version".
// GPLv2 section 9: "If the Program does not specify a version number of this License, you may choose any version
// ever published by the Free Software Foundation."
// On that basis (video_freak.sv) and sys_top.v's own "or later", this file is under the GNU General Public License
// version 2 or (at your option) any later version (GPL-2.0-or-later; LICENSES/GPL-2.0.txt, LICENSES/GPL-3.0.txt),
// not under the MIT licence of the rest of sys-tango; see NOTICE.  Every build with a video path (TANGO_VIDEO_FB or
// TANGO_VIDEO_ASCAL: tango_video instantiates this module) contains it, and such a bitstream is a work under the GPL.
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
//==============================================================================
// tango_video_geo -- the geometry of one source frame on the 720p screen, and the nearest-neighbour
// step constants the scaler (tango_video) walks with, from MiSTer's video path:
//   1. crop (video_freak.sv 58-62, 76, 116-118): vcrop = CROP_SIZE unless >= the frame's lines;
//      voff = (vtot - vcrop)/2 + CROP_OFF, clamped to [0, vtot - vcrop];
//   2. crop keeps the aspect (video_freak.sv 80-114): arx*vtot : ary*vcrop, normalised to 12 bits;
//   3. SCALE (module video_scale_int, video_freak.sv 141-328 as in Template_MiSTer 19e89ce and NES_MiSTer
//      2668b7e -- MiSTer's current sys/; Genesis_MiSTer's 2023 copy is older): 0 Normal, 1 V-Integer,
//      2 Narrower HV-Integer, 3 Wider HV-Integer, 4 HV-Integer (the nearer); the result is a size (bit 12 set):
//        height oh = vsize * floor(720/vsize) (193-223); with an aspect, the target width
//        htarget = oh*arx/ary (225-249) and hinteger = hsize * max(1, floor(htarget/hsize)) (255-267);
//        with no aspect (arx = ary = 0), or when that hinteger is wider than 1280, k = floor(1280/hsize),
//        hinteger = hsize*k and oh = vsize*k instead (269-296); wideres = hinteger + hsize (299);
//        Wider = hinteger when it equals htarget, else wideres if it fits (301, 316);
//        HV-Integer = hinteger when it is the nearer to htarget or wideres does not fit (300, 317);
//        V-Integer = htarget (or 1280 on the no-aspect / too-wide path: div_num, 318);
//      (MiSTer's state machine loops, and 300-301 read wideres from its previous pass: once settled that is
//      this pass's value, which is what this one-shot engine uses);
//   4. the window (sys_top.v 856-944): size as given, or from the aspect (720*arx/ary x 1280*ary/arx),
//      clipped to 1280x720 and centred; the rest of the screen is black;
//   5. the steps: for output pixel ox of the window, the source pixel is floor(((2ox+1)*iw - 1) / (2*ow))
//      (the output pixel's centre, a tie going to the left pixel); for output line oy the "doubled line"
//      dl = floor(((2oy+1)*2*ih - 1) / (2*oh)), source line dl>>1, scanline half dl[0]
//      (MiSTer's scanlines act on the scandoubled picture: sys_top.v 1293-1309 before ascal 696-712).
// A core that runs MiSTer's own video_freak passes its VIDEO_ARX/ARY (bit 12 = size) with SCALE 0 and
// CROP_SIZE 0: steps 1-3 then change nothing, as in MiSTer.
// Everything runs in a slow sequential engine (one shift-add multiplier, one restoring divider) that
// needs ~400 clocks per frame whose inputs changed; nothing here is on the pixel path.
//==============================================================================
`default_nettype none
module tango_video_geo #(
    parameter integer OW = 1280,            // output picture (hdmi.sv 720p)
    parameter integer OH = 720
) (
    input  wire        clk,
    input  wire        go,                  // one clock: compute for the inputs below (held stable)
    input  wire [11:0] hs,                  // active width of the frame (first line), <= the block's MAXW
    input  wire [11:0] vt,                  // active lines of the frame, <= MAXH
    input  wire [12:0] arx, ary,            // MiSTer VIDEO_ARX/ARY: aspect, or (bit 12) a size in pixels
    input  wire [2:0]  scale,               // video_freak SCALE
    input  wire [11:0] crop_size,           // video_freak CROP_SIZE (0 = off)
    input  wire [4:0]  crop_off,            // video_freak CROP_OFF, -16..+15
    output reg         busy = 1'b0,
    // results (valid while !busy after a go)
    output reg         valid = 1'b0,        // a picture to show (else black)
    output reg  [11:0] x0 = 12'd0, x1 = 12'd0,   // window columns [x0, x1)
    output reg  [10:0] y0 = 11'd0, y1 = 11'd0,   // window lines   [y0, y1)
    output reg  [11:0] voff = 12'd0,        // first source line shown (crop)
    output reg  [11:0] ih_crop = 12'd0,    // cropped source height fed to scaler (vsize)
    output reg  [11:0] hq0 = 12'd0, hq = 12'd0,  // horizontal: first source pixel, step quotient
    output reg  [12:0] ha0 = 13'd0, hr = 13'd0,  //             first accumulator, step remainder
    output reg  signed [14:0] hdn = 15'sd0,      //             hr - 2*ow (the wrap test)
    output reg  [12:0] vq0 = 13'd0, vq = 13'd0,  // vertical (doubled lines)
    output reg  [12:0] va0 = 13'd0, vr = 13'd0,
    output reg  signed [14:0] vdn = 15'sd0,
    // what MiSTer's video_freak would give (VIDEO_ARX/ARY), and the window size, for sims and debug
    output reg  [12:0] fx = 13'd0, fy = 13'd0,
    output reg  [11:0] ow_o = 12'd0, oh_o = 12'd0,
    // ascal output window (inclusive pixel coordinates on the 720p canvas)
    output reg  [11:0] hmin = 12'd0, hmax = 12'd0,  // hmax = hmin + vw - 1
    output reg  [10:0] vmin = 11'd0, vmax = 11'd0   // vmax = vmin + vh - 1
);
    // ---------------------------------------------------------------- the arithmetic units
    reg        mgo = 1'b0, mbusy = 1'b0;
    reg [11:0] ma = 12'd0, mb = 12'd0;
    reg [23:0] m_a = 24'd0, mp = 24'd0;
    reg [11:0] m_b = 12'd0;
    always @(posedge clk)                   // shift-add, one multiplier bit per clock
        if (mgo) begin m_a <= {12'd0, ma}; m_b <= mb; mp <= 24'd0; mbusy <= 1'b1; end
        else if (mbusy) begin
            if (m_b == 12'd0) mbusy <= 1'b0;
            else begin
                if (m_b[0]) mp <= mp + m_a;
                m_a <= m_a << 1;
                m_b <= m_b >> 1;
            end
        end
    reg        dgo = 1'b0, dbusy = 1'b0;
    reg [23:0] dn = 24'd0, dq = 24'd0;
    reg [12:0] dd = 13'd0, d_d = 13'd0, dr = 13'd0;
    reg [4:0]  dcnt = 5'd0;
    wire [13:0] drn = {dr, dq[23]};
    always @(posedge clk)                   // restoring division, one quotient bit per clock
        if (dgo) begin dq <= dn; d_d <= dd; dr <= 13'd0; dcnt <= 5'd24; dbusy <= 1'b1; end
        else if (dbusy) begin
            if (drn >= {1'b0, d_d}) begin dr <= drn[12:0] - d_d; dq <= {dq[22:0], 1'b1}; end
            else                    begin dr <= drn[12:0];         dq <= {dq[22:0], 1'b0}; end
            dcnt <= dcnt - 5'd1;
            if (dcnt == 5'd1) dbusy <= 1'b0;
        end

    // ---------------------------------------------------------------- the program
    reg [4:0]  pc = 5'd0;
    reg        wt = 1'b0;                   // waiting for a unit
    reg [11:0] i_hs = 12'd0, i_vt = 12'd0, i_cs = 12'd0;
    reg [12:0] i_arx = 13'd0, i_ary = 13'd0;
    reg [2:0]  i_sc = 3'd0;
    reg [4:0]  i_co = 5'd0;
    reg [11:0] vcrop = 12'd0, vsize = 12'd0, vadj = 12'd0;
    reg [11:0] arxo = 12'd0, aryo = 12'd0;
    reg [23:0] g1 = 24'd0, g2 = 24'd0, dnum = 24'd0, wn = 24'd0;
    reg [4:0]  nrm = 5'd0;
    reg [11:0] oh = 12'd0, htarget = 12'd0, wres = 12'd0;
    reg        narrow = 1'b0;
    reg [11:0] ax = 12'd0, ay = 12'd0, wc = 12'd0, hc = 12'd0, vw = 12'd0, vh = 12'd0;
    reg        axy = 1'b0;
    // video_scale_int's 12-bit registers (video_freak.sv 174): wn[11:0] = hinteger, wide_w = wideres
    wire [11:0] wide_w = wn[11:0] + i_hs;                   // video_freak.sv 299 (12-bit)
    wire [11:0] lo_w   = htarget - wn[11:0];                // 300, 12-bit arithmetic as there
    wire [11:0] hi_w   = wide_w - htarget;
    wire [11:0] vadj_h = {1'b0, vadj[11:1]};
    wire [11:0] kq     = (dq[11:0] != 12'd0) ? dq[11:0] : 12'd1;
    localparam [23:0] OW24 = OW[23:0], OH24 = OH[23:0];
    localparam [4:0] P_IDLE = 5'd0, P_SI = 5'd6, P_S8 = 5'd13, P_S10 = 5'd16, P_SYS = 5'd18, P_WIN = 5'd24,
                     P_FH = 5'd30;

    always @(posedge clk) begin
        mgo <= 1'b0; dgo <= 1'b0;
        if (wt) begin
            if (!mgo && !dgo && !mbusy && !dbusy) wt <= 1'b0;
        end else case (pc)
        P_IDLE: if (go) begin
                i_hs <= hs; i_vt <= vt; i_arx <= arx; i_ary <= ary; i_sc <= scale; i_cs <= crop_size; i_co <= crop_off;
                busy <= 1'b1; pc <= 5'd1;
            end
        5'd1: begin                                                     // crop (video_freak 58-62, 76, 116)
                if (i_hs == 12'd0 || i_vt == 12'd0) begin valid <= 1'b0; busy <= 1'b0; pc <= P_IDLE; end
                else begin
                    vcrop    <= (i_cs >= i_vt) ? 12'd0 : i_cs;
                    vsize    <= (i_cs >= i_vt || i_cs == 12'd0) ? i_vt : i_cs;
                    ih_crop  <= (i_cs >= i_vt || i_cs == 12'd0) ? i_vt : i_cs;
                    vadj     <= (i_vt - ((i_cs >= i_vt) ? 12'd0 : i_cs)) + {{6{i_co[4]}}, i_co, 1'b0};
                    pc <= 5'd2;
                end
            end
        5'd2: begin                                                     // video_freak 117; aspect 80-83
                voff <= (vcrop == 12'd0 || vadj[11]) ? 12'd0 :
                        ((vadj_h + vcrop) > i_vt) ? i_vt - vcrop : vadj_h;
                if (i_arx[12] | i_ary[12]) begin fx <= i_arx; fy <= i_ary; pc <= P_SYS; end
                else if (vcrop == 12'd0 || i_ary[11:0] == 12'd0 || i_arx[11:0] == 12'd0) begin
                    arxo <= i_arx[11:0]; aryo <= i_ary[11:0]; pc <= P_SI;
                end else begin ma <= i_arx[11:0]; mb <= i_vt; mgo <= 1'b1; wt <= 1'b1; pc <= 5'd3; end
            end
        5'd3: begin g1 <= mp; ma <= i_ary[11:0]; mb <= vcrop; mgo <= 1'b1; wt <= 1'b1; pc <= 5'd4; end
        5'd4: begin g2 <= mp; nrm <= 5'd0; pc <= 5'd5; end
        5'd5: if (g1[23] | g2[23] | nrm == 5'd24) begin                // video_freak 107-114
                arxo <= g1[23:12]; aryo <= g2[23:12]; pc <= P_SI;
            end else begin g1 <= g1 << 1; g2 <= g2 << 1; nrm <= nrm + 5'd1; end
        // ---- SCALE: video_scale_int (video_freak.sv 141-328); dnum plays its div_num, wn its hinteger
        P_SI: if (i_sc == 3'd0 || (aryo == 12'd0 && arxo != 12'd0)) begin   // 182-185: as given
                fx <= {1'b0, arxo}; fy <= {1'b0, aryo}; pc <= P_SYS;
            end else begin dn <= OH24; dd <= {1'b0, vsize}; dgo <= 1'b1; wt <= 1'b1; pc <= 5'd7; end  // 193-197
        5'd7: if (dq[11:0] == 12'd0) begin fx <= {1'b0, arxo}; fy <= {1'b0, aryo}; pc <= P_SYS; end   // 202-208
              else begin ma <= vsize; mb <= dq[11:0]; mgo <= 1'b1; wt <= 1'b1; pc <= 5'd8; end         // 209-213
        5'd8: begin                                                     // 218-229: oheight
                oh <= mp[11:0];
                if (aryo == 12'd0) pc <= P_S8;                          // no aspect: the 1280/hsize path
                else begin ma <= mp[11:0]; mb <= arxo; mgo <= 1'b1; wt <= 1'b1; pc <= 5'd9; end
            end
        5'd9:  begin dn <= mp; dd <= {1'b0, aryo}; dgo <= 1'b1; wt <= 1'b1; pc <= 5'd10; end       // 234-238
        5'd10: begin                                                    // 244-249: htarget = oh*arx/ary
                htarget <= dq[11:0]; dnum <= dq;
                dn <= dq; dd <= {1'b0, i_hs}; dgo <= 1'b1; wt <= 1'b1; pc <= 5'd11;
            end
        5'd11: begin ma <= i_hs; mb <= kq; mgo <= 1'b1; wt <= 1'b1; pc <= 5'd12; end                 // 255-259
        5'd12: if (mp <= OW24) begin wn <= mp; pc <= P_S10; end         // 264-267: hinteger fits
               else pc <= P_S8;
        P_S8:  begin dnum <= OW24; dn <= OW24; dd <= {1'b0, i_hs}; dgo <= 1'b1; wt <= 1'b1; pc <= 5'd14; end // 269-273
        5'd14: begin ma <= i_hs; mb <= kq; mgo <= 1'b1; wt <= 1'b1; pc <= 5'd15; end                 // 278-282
        5'd15: begin wn <= mp; ma <= vsize; mb <= kq; mgo <= 1'b1; wt <= 1'b1; pc <= P_FH; end     // 287-292
        P_FH:  begin oh <= mp[11:0]; pc <= P_S10; end                   // 294-296: oheight = vsize * k
        P_S10: begin                                                    // 298-302
                narrow <= (lo_w <= hi_w) || (wide_w > OW);
                wres   <= (wn[11:0] == htarget) ? wn[11:0] : wide_w;
                pc <= 5'd17;
            end
        5'd17: begin                                                    // 313-321
                case (i_sc)
                3'd2:    fx <= {1'b1, wn[11:0]};
                3'd3:    fx <= {1'b1, (wres > OW) ? wn[11:0] : wres};
                3'd4:    fx <= {1'b1, narrow ? wn[11:0] : wres};
                default: fx <= {1'b1, dnum[11:0]};
                endcase
                fy <= {1'b1, oh};
                pc <= P_SYS;
            end
        P_SYS: begin                                                    // sys_top.v 858-883 (no ARC: 0)
                if (fy == 13'd0) begin ax <= 12'd0; ay <= 12'd0; axy <= 1'b0; end
                else begin ax <= fx[11:0]; ay <= fy[11:0]; axy <= fx[12] | fy[12]; end
                pc <= 5'd19;
            end
        5'd19: if (ax == 12'd0 || ay == 12'd0) begin wc <= OW; hc <= OH; pc <= P_WIN; end   // 897-900
               else if (axy) begin wc <= ax; hc <= ay; pc <= P_WIN; end                     // 902-905
               else begin ma <= OH; mb <= ax; mgo <= 1'b1; wt <= 1'b1; pc <= 5'd20; end
        5'd20: begin dn <= mp; dd <= {1'b0, ay}; dgo <= 1'b1; wt <= 1'b1; pc <= 5'd21; end  // 908-915
        5'd21: begin wc <= dq[11:0]; ma <= OW; mb <= ay; mgo <= 1'b1; wt <= 1'b1; pc <= 5'd22; end
        5'd22: begin dn <= mp; dd <= {1'b0, ax}; dgo <= 1'b1; wt <= 1'b1; pc <= 5'd23; end  // 918-925
        5'd23: begin hc <= dq[11:0]; pc <= P_WIN; end
        P_WIN: begin                                                    // 930-933
                vw <= (wc > OW) ? OW : wc;
                vh <= (hc > OH) ? OH : hc;
                pc <= 5'd25;
            end
        5'd25: begin                                                    // 936-941, centred
                x0 <= (OW - vw) >> 1;  x1 <= ((OW - vw) >> 1) + vw;
                y0 <= (OH - vh) >> 1;  y1 <= ((OH - vh) >> 1) + vh;
                ow_o <= vw; oh_o <= vh;
                // ascal window: inclusive [hmin, hmax] x [vmin, vmax]
                hmin <= (OW - vw) >> 1;
                hmax <= ((OW - vw) >> 1) + vw - 12'd1;
                vmin <= (OH - vh) >> 1;
                vmax <= ((OH - vh) >> 1) + vh - 11'd1;
                if (vw == 12'd0 || vh == 12'd0) begin valid <= 1'b0; busy <= 1'b0; pc <= P_IDLE;
                    hmin <= 12'd0; hmax <= 12'd0; vmin <= 11'd0; vmax <= 11'd0;
                end else begin dn <= {11'd0, i_hs, 1'b0}; dd <= {vw, 1'b0}; dgo <= 1'b1; wt <= 1'b1; pc <= 5'd26; end
            end
        5'd26: begin hq <= dq[11:0]; hr <= dr; dn <= {12'd0, i_hs - 12'd1}; dd <= {vw, 1'b0}; dgo <= 1'b1; wt <= 1'b1; pc <= 5'd27; end
        5'd27: begin hq0 <= dq[11:0]; ha0 <= dr; dn <= {10'd0, vsize, 2'b00}; dd <= {vh, 1'b0}; dgo <= 1'b1; wt <= 1'b1; pc <= 5'd28; end
        5'd28: begin vq <= dq[12:0]; vr <= dr; dn <= {11'd0, vsize, 1'b0} - 24'd1; dd <= {vh, 1'b0}; dgo <= 1'b1; wt <= 1'b1; pc <= 5'd29; end
        5'd29: begin
                vq0 <= dq[12:0]; va0 <= dr;
                hdn <= $signed({2'b00, hr}) - $signed({2'b00, vw, 1'b0});
                vdn <= $signed({2'b00, vr}) - $signed({2'b00, vh, 1'b0});
                valid <= 1'b1; busy <= 1'b0; pc <= P_IDLE;
            end
        default: pc <= P_IDLE;
        endcase
    end
endmodule
`default_nettype wire
