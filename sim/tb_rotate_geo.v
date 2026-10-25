// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tb_rotate_geo -- the aspect/size swap tango_video applies while the rotation
// framebuffer is shown (tango_video_rot_geo.vh, the SAME function tango_video includes), run through the REAL
// tango_video_geo, checked against window sizes computed here independently from MiSTer's sys_top.v rule
// (wcalc = 720*arx/ary, hcalc = 1280*ary/arx, each clipped to the screen, centred: sys_top.v 921-972).
//   - Normal (rot = 0): the function returns its inputs bit for bit (10000 random vectors).
//   - Rotated: hs/vt = the framebuffer's size, arx/ary swapped as whole 13-bit values (ratio and bit-12 size form).
//   - Windows: Galaga 288x224 2880:2191 Normal 946x720 at x 167, rotated (224x288, 2191:2880) 547x720 at x 366;
//     Template 4:3 Normal 960x720 at x 160, rotated 540x720 at x 370; Galaga rotated V-Integer 576 lines tall.
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_rotate_geo;
`include "tango_video_rot_geo.vh"
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg go = 1'b0;
    reg [11:0] hs = 0, vt = 0;
    reg [12:0] arx = 0, ary = 0;
    reg [2:0]  scale = 0;
    wire busy, valid;
    wire [11:0] x0, x1, voff, ih_crop, hq0, hq, ow_o, oh_o, hmin, hmax;
    wire [10:0] y0, y1, vmin, vmax;
    wire [12:0] ha0, hr, vq0, vq, va0, vr, fx, fy;
    wire signed [14:0] hdn, vdn;
    tango_video_geo #(.OW(1280), .OH(720)) u_geo (
        .clk(clk), .go(go), .hs(hs), .vt(vt), .arx(arx), .ary(ary), .scale(scale), .crop_size(12'd0), .crop_off(5'd0),
        .busy(busy), .valid(valid), .x0(x0), .x1(x1), .y0(y0), .y1(y1), .voff(voff), .ih_crop(ih_crop),
        .hq0(hq0), .hq(hq), .ha0(ha0), .hr(hr), .hdn(hdn), .vq0(vq0), .vq(vq), .va0(va0), .vr(vr), .vdn(vdn),
        .fx(fx), .fy(fy), .ow_o(ow_o), .oh_o(oh_o), .hmin(hmin), .hmax(hmax), .vmin(vmin), .vmax(vmax));

    integer errs = 0, checks = 0;
    task run_case(input [127:0] name, input rot, input [11:0] nhs, input [11:0] nvt, input [12:0] carx, input [12:0] cary,
                  input [2:0] sc, input integer want_w, input integer want_h);
        reg [71:0] g;
        integer w, h, wmin, vmn;
        begin
            // the native raster's size; the framebuffer's size when rotated is native height x native width
            g = rot_geo_in(rot, nhs, nvt, nvt, nhs, {2'b00, carx, cary, sc, 12'd0, 5'd0});
            hs = g[71:60]; vt = g[59:48]; arx = g[45:33]; ary = g[32:20]; scale = g[19:17];
            @(posedge clk); go <= 1'b1; @(posedge clk); go <= 1'b0;
            @(posedge clk); while (busy) @(posedge clk);
            w = hmax - hmin + 1; h = vmax - vmin + 1;
            wmin = (1280 - want_w) / 2; vmn = (720 - want_h) / 2;
            checks = checks + 1;
            if (w != want_w || h != want_h || hmin != wmin || vmin != vmn) begin
                errs = errs + 1;
                $display("GEO MISMATCH %0s: geo in hs=%0d vt=%0d arx=%0d ary=%0d sc=%0d -> window %0dx%0d at (%0d,%0d), want %0dx%0d at (%0d,%0d)",
                         name, hs, vt, arx, ary, sc, w, h, hmin, vmin, want_w, want_h, wmin, vmn);
            end else
                $display("GEO ok %0s: geo in hs=%0d vt=%0d arx=%0d ary=%0d sc=%0d -> window %0dx%0d at (%0d,%0d)",
                         name, hs, vt, arx, ary, sc, w, h, hmin, vmin);
        end
    endtask

    integer i;
    reg [11:0] rh, rv, fh, fv; reg [47:0] ro; reg [71:0] gg;
    initial begin
        // Normal: identity, bit for bit; rotated: exact swap layout
        for (i = 0; i < 10000; i = i + 1) begin
            rh = $urandom; rv = $urandom; fh = $urandom; fv = $urandom; ro = {$urandom, $urandom};
            gg = rot_geo_in(1'b0, rh, rv, fh, fv, ro);
            if (gg !== {rh, rv, ro}) begin errs = errs + 1; if (errs < 5) $display("IDENTITY MISMATCH %h", gg); end
            gg = rot_geo_in(1'b1, rh, rv, fh, fv, ro);
            if (gg !== {fh, fv, ro[47:46], ro[32:20], ro[45:33], ro[19:0]}) begin errs = errs + 1; if (errs < 5) $display("SWAP MISMATCH %h", gg); end
            checks = checks + 2;
        end
        // bit-12 size form swaps whole: arx = size 300 | flag, ary = size 400 | flag
        gg = rot_geo_in(1'b1, 12'd288, 12'd224, 12'd224, 12'd288, {2'b00, 13'h112C, 13'h1190, 3'd0, 12'd0, 5'd0});
        if (gg[45:33] !== 13'h1190 || gg[32:20] !== 13'h112C) begin errs = errs + 1; $display("SIZE-FORM SWAP MISMATCH"); end
        checks = checks + 1;
        // windows (want_w/want_h computed by hand from the sys_top.v rule, see header)
        run_case("galaga normal",      1'b0, 12'd288, 12'd224, 13'd2880, 13'd2191, 3'd0, 946, 720);
        run_case("galaga rotated",     1'b1, 12'd288, 12'd224, 13'd2880, 13'd2191, 3'd0, 547, 720);
        run_case("template normal",    1'b0, 12'd288, 12'd224, 13'd4,    13'd3,    3'd0, 960, 720);
        run_case("template rotated",   1'b1, 12'd288, 12'd224, 13'd4,    13'd3,    3'd0, 540, 720);
        // V-Integer rotated Galaga: oh = 288*floor(720/288) = 576, ow = 576*2191/2880 = 438 (video_freak 193-249)
        run_case("galaga rot V-int",   1'b1, 12'd288, 12'd224, 13'd2880, 13'd2191, 3'd1, 438, 576);
        $display("TB_ROTATE_GEO: %0d checks, %0d mismatches", checks, errs);
        $display("TB_RESULT %0s", errs == 0 ? "PASS" : "FAIL");
        $finish;
    end
endmodule
`default_nettype wire
