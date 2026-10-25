// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_rot_scale -- the arcade Scale entry (Normal / V-Integer only, MiSTer arcade parity).
// What tango_video does with status[125:124] in an ARCADE build (TANGO_VIDEO_ROTATE + TANGO_VIDEO_OPTS, Aspect =
// Original), run through the SAME code tango_video includes:
//   scale = {1'b0, scale_arc(status[125:124])}  (tango_video_scale_arc.vh: 2/3 = a stale CFG -> 0 Normal)
//   geo   = rot_geo_in(rot, ...)                 (tango_video_rot_geo.vh: the rotated framebuffer's size + swapped aspect)
// into the REAL tango_video_geo (1280x720). Checked window = hmin..hmax x vmin..vmax against a fixed table:
//   - every (core, rotation, status 0..3): status 2 and 3 give EXACTLY the Normal window, 0 and 1 the table's own;
//   - the table's Normal / V-Integer rows match MiSTer's own rule: a separate bench (not part of this repo: GPL
//     source) fed the same 8 Normal / V-Integer cases to the UNMODIFIED Template_MiSTer sys/video_freak.sv
//     video_scale_int + sys_top.v's window rule: all 8 MATCH (its HV-Integer rows differ, irrelevant to the
//     arcade 2-value list);
//   - CONSOLE path (no TANGO_VIDEO_ROTATE: tango_video passes status[125:124] straight through): raw scale 2/3 still
//     gives the HV-Integer windows, i.e. tango_video_geo itself is unchanged;
//   - scale_arc() over all 4 inputs: 0->0, 1->1, 2->0, 3->0.
// Galaga: native 288x224, VIDEO_ARX/ARY 2880:2191, the rotated framebuffer 224x288 (tango_video_rotate_ddr3).
// Template: 288x224 at 4:3 (the same raster as the arcade template core's test pattern).
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_rot_scale;
`include "tango_video_rot_geo.vh"
`include "tango_video_scale_arc.vh"
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg go = 1'b0;
    reg [11:0] t_hs = 0, t_vt = 0;
    reg [12:0] t_arx = 0, t_ary = 0;
    reg [2:0]  t_sc = 0;
    wire busy, valid;
    wire [11:0] x0, x1, voff, ih_crop, hq0, hq, ow_o, oh_o, hmin, hmax;
    wire [10:0] y0, y1, vmin, vmax;
    wire [12:0] ha0, hr, vq0, vq, va0, vr, fx, fy;
    wire signed [14:0] hdn, vdn;
    tango_video_geo #(.OW(1280), .OH(720)) u_geo (
        .clk(clk), .go(go), .hs(t_hs), .vt(t_vt), .arx(t_arx), .ary(t_ary), .scale(t_sc), .crop_size(12'd0), .crop_off(5'd0),
        .busy(busy), .valid(valid), .x0(x0), .x1(x1), .y0(y0), .y1(y1), .voff(voff), .ih_crop(ih_crop),
        .hq0(hq0), .hq(hq), .ha0(ha0), .hr(hr), .hdn(hdn), .vq0(vq0), .vq(vq), .va0(va0), .vr(vr), .vdn(vdn),
        .fx(fx), .fy(fy), .ow_o(ow_o), .oh_o(oh_o), .hmin(hmin), .hmax(hmax), .vmin(vmin), .vmax(vmax));

    integer errs = 0, checks = 0;
    integer w_hmin, w_hmax, w_vmin, w_vmax, w_ok;

    // one engine run, as tango_video's geo FSM fires it (go one clock, wait for !busy)
    task run_geo(input [71:0] g, input [2:0] sc);
        begin
            t_hs = g[71:60]; t_vt = g[59:48]; t_arx = g[45:33]; t_ary = g[32:20]; t_sc = sc;
            @(posedge clk); go <= 1'b1; @(posedge clk); go <= 1'b0;
            @(posedge clk); while (busy) @(posedge clk);
            w_hmin = hmin; w_hmax = hmax; w_vmin = vmin; w_vmax = vmax; w_ok = valid;
        end
    endtask

    // expected window (hmin, hmax, vmin, vmax) for (core, rot, effective scale 0..3)
    task expect_win(input gal, input rot, input [1:0] sc, output integer eh0, output integer eh1, output integer ev0, output integer ev1);
        begin
            case ({gal, rot, sc})
                // Template 4:3, Normal rotation
                4'b0000: begin eh0 = 160; eh1 = 1119; ev0 = 0;  ev1 = 719; end   // Normal          960x720
                4'b0001: begin eh0 = 192; eh1 = 1087; ev0 = 24; ev1 = 695; end   // V-Integer       896x672
                4'b0010: begin eh0 = 208; eh1 = 1071; ev0 = 24; ev1 = 695; end   // Narrower HV     864x672
                4'b0011: begin eh0 = 64;  eh1 = 1215; ev0 = 24; ev1 = 695; end   // Wider HV       1152x672
                // Template rotated (224x288, 3:4)
                4'b0100: begin eh0 = 370; eh1 = 909;  ev0 = 0;  ev1 = 719; end   // Normal          540x720
                4'b0101: begin eh0 = 424; eh1 = 855;  ev0 = 72; ev1 = 647; end   // V-Integer       432x576
                4'b0110: begin eh0 = 528; eh1 = 751;  ev0 = 72; ev1 = 647; end   // Narrower HV     224x576
                4'b0111: begin eh0 = 416; eh1 = 863;  ev0 = 72; ev1 = 647; end   // Wider HV        448x576
                // Galaga 2880:2191, Normal rotation
                4'b1000: begin eh0 = 167; eh1 = 1112; ev0 = 0;  ev1 = 719; end   // Normal          946x720
                4'b1001: begin eh0 = 198; eh1 = 1080; ev0 = 24; ev1 = 695; end   // V-Integer       883x672
                4'b1010: begin eh0 = 208; eh1 = 1071; ev0 = 24; ev1 = 695; end   // Narrower HV     864x672
                4'b1011: begin eh0 = 64;  eh1 = 1215; ev0 = 24; ev1 = 695; end   // Wider HV       1152x672
                // Galaga rotated (224x288, 2191:2880)
                4'b1100: begin eh0 = 366; eh1 = 912;  ev0 = 0;  ev1 = 719; end   // Normal          547x720
                4'b1101: begin eh0 = 421; eh1 = 858;  ev0 = 72; ev1 = 647; end   // V-Integer       438x576
                4'b1110: begin eh0 = 528; eh1 = 751;  ev0 = 72; ev1 = 647; end   // Narrower HV     224x576
                default: begin eh0 = 416; eh1 = 863;  ev0 = 72; ev1 = 647; end   // Wider HV        448x576
            endcase
        end
    endtask

    // path: 1 = arcade (scale_arc applied), 0 = console (status passed straight through)
    task one(input arc, input gal, input rot, input [1:0] st);
        reg [71:0] g; reg [2:0] sc; reg [1:0] eff; integer eh0, eh1, ev0, ev1; reg ok;
        begin
            g = gal ? rot_geo_in(rot, 12'd288, 12'd224, 12'd224, 12'd288, {2'b00, 13'd2880, 13'd2191, 3'd0, 12'd0, 5'd0})
                    : rot_geo_in(rot, 12'd288, 12'd224, 12'd224, 12'd288, {2'b00, 13'd4,    13'd3,    3'd0, 12'd0, 5'd0});
            eff = arc ? scale_arc(st) : st;
            sc  = {1'b0, eff};
            run_geo(g, sc);
            // the spec, written independently of scale_arc(): arcade 2/3 -> the Normal window; console -> the status itself
            expect_win(gal, rot, (arc && st[1]) ? 2'd0 : st, eh0, eh1, ev0, ev1);
            ok = w_ok && w_hmin == eh0 && w_hmax == eh1 && w_vmin == ev0 && w_vmax == ev1;
            checks = checks + 1; if (!ok) errs = errs + 1;
            $display("ROW %0s %-8s %-6s status %0d -> scale %0d : %0dx%0d x %0d..%0d y %0d..%0d  (want x %0d..%0d y %0d..%0d) %0s",
                     arc ? "ARCADE " : "CONSOLE", gal ? "Galaga" : "Template", rot ? "CW/CCW" : "Normal", st, eff,
                     w_hmax - w_hmin + 1, w_vmax - w_vmin + 1, w_hmin, w_hmax, w_vmin, w_vmax, eh0, eh1, ev0, ev1,
                     ok ? "OK" : "MISMATCH");
        end
    endtask

    integer s, gl, rt, nf = 0;
    initial begin
        // scale_arc over its whole input range
        for (s = 0; s < 4; s = s + 1) begin
            checks = checks + 1;
            if (scale_arc(s[1:0]) != ((s >= 2) ? 2'd0 : s[1:0])) begin errs = errs + 1; nf = nf + 1; end
            $display("FN  scale_arc(%0d) = %0d", s, scale_arc(s[1:0]));
        end
        // arcade: every core x rotation x status 0..3; status 2/3 must equal the Normal (status 0) window
        for (gl = 0; gl < 2; gl = gl + 1)
            for (rt = 0; rt < 2; rt = rt + 1)
                for (s = 0; s < 4; s = s + 1) one(1'b1, gl[0], rt[0], s[1:0]);
        // console path: raw 2/3 still the HV windows (tango_video_geo unchanged)
        for (gl = 0; gl < 2; gl = gl + 1)
            for (rt = 0; rt < 2; rt = rt + 1)
                for (s = 2; s < 4; s = s + 1) one(1'b0, gl[0], rt[0], s[1:0]);
        $display("TB_ROT_SCALE: %0d checks, %0d mismatches", checks, errs);
        $display("TB_RESULT %0s", (errs == 0) ? "PASS" : "FAIL");
        $finish;
    end
endmodule
`default_nettype wire
