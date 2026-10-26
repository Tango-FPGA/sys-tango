// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_ddr3_ov -- the DDR3 bring-up test's 720p results screen, drawn from cx / cy (hdmi.sv) for a capture decoder
// (ddr3/test/decode_d1.py): solid blocks, sampled at their centres.
//   grid: 3 columns x 38 rows of slots; slot s = column * 38 + row at x = 24 + 416 * column, y = 80 + 16 * row;
//     a 12 x 12 marker (its colour = the slot's group), then the slot's 32 bits, most significant first: bit i at
//     x + 16 + 11 * (31 - i), 9 x 12 px, white = 1, dark grey = 0;
//   banner (y 16..63): the status block (x 24..71: green = calibrated and no error, red = an error, yellow = not
//     calibrated), four values in hex (3x 8 x 8 font, y 28..51) after a 24 x 24 label: errors (red / green),
//     march passes (cyan), uptime s (white), loops (yellow); a heartbeat square (x 1232) that flips every 8 updates.
// Timing (clk_pix 74.25 MHz): no arithmetic on cx.  The horizontal positions come from counters that run along the
// scanline (hdmi.sv: cx + 1 every clock, 1649 -> 0), loaded when the next pixel reaches a region's left edge; the
// vertical ones are registered from cy (it changes at cx = 0, and nothing is drawn left of x = 24).  The picture is
// the one the earlier arithmetic version drew, pixel for pixel (checked in sim over whole frames).
// Latency: 3 clk_pix from cx to rgb (the bank read is the second); the drawing is for cx + 3, so blocks sit where
// the numbers say (hdmi.sv's own rgb register aside, which the decoder's offset search absorbs).
//==============================================================================
`default_nettype none
module tango_ddr3_ov (
    input  wire        clk,
    input  wire [11:0] cx,
    input  wire [10:0] cy,
    output reg  [6:0]  slot_addr = 7'd0,      // to the display bank; its word arrives one clock later
    input  wire [31:0] slot_data,
    input  wire [31:0] err, passes, uptime, loops,
    input  wire        calib, any_err, beat,
    output reg  [23:0] rgb = 24'd0
);
    localparam [23:0] C_BG = 24'h102030, C_WH = 24'hFFFFFF, C_DK = 24'h383838, C_CY = 24'h00C0C0,
                      C_OR = 24'hF07000, C_MA = 24'hD000D0, C_YE = 24'hE0E000, C_GR = 24'h00C000,
                      C_BL = 24'h2040FF, C_LB = 24'h60A0FF, C_PU = 24'h9040C0, C_RD = 24'hE00000;

    function [7:0] font(input [3:0] c, input [2:0] r);
        reg [63:0] g;
        begin
            case (c)
                4'h0: g = 64'h3C666E7666663C00; 4'h1: g = 64'h1838181818187E00;
                4'h2: g = 64'h3C66060C30607E00; 4'h3: g = 64'h3C66061C06663C00;
                4'h4: g = 64'h0C1C3C6C7E0C0C00; 4'h5: g = 64'h7E607C0606663C00;
                4'h6: g = 64'h3C607C6666663C00; 4'h7: g = 64'h7E060C1830303000;
                4'h8: g = 64'h3C66663C66663C00; 4'h9: g = 64'h3C66663E060C3800;
                4'hA: g = 64'h183C66667E666600; 4'hB: g = 64'h7C66667C66667C00;
                4'hC: g = 64'h3C66606060663C00; 4'hD: g = 64'h786C6666666C7800;
                4'hE: g = 64'h7E60607C60607E00; default: g = 64'h7E60607C60606000;
            endcase
            font = g[63 - 8 * r -: 8];
        end
    endfunction
    function [23:0] mcol(input [6:0] s);
        begin
            if      (s == 7'd0)  mcol = C_WH;
            else if (s < 7'd12)  mcol = C_CY;
            else if (s < 7'd26)  mcol = C_OR;
            else if (s < 7'd34)  mcol = C_MA;
            else if (s < 7'd37)  mcol = C_YE;
            else if (s < 7'd44)  mcol = C_GR;
            else if (s < 7'd49)  mcol = C_BL;
            else if (s < 7'd71)  mcol = C_LB;
            else if (s < 7'd111) mcol = C_PU;
            else                 mcol = C_YE;
        end
    endfunction
    function [2:0] div3(input [4:0] v);                       // v / 3 for v = 0..23 (a font row)
        div3 = (v < 5'd3) ? 3'd0 : (v < 5'd6) ? 3'd1 : (v < 5'd9) ? 3'd2 : (v < 5'd12) ? 3'd3 :
               (v < 5'd15) ? 3'd4 : (v < 5'd18) ? 3'd5 : (v < 5'd21) ? 3'd6 : 3'd7;
    endfunction

    // ---- vertical: registered from cy
    reg        gy_r = 1'b0, by_r = 1'b0, sy_r = 1'b0;
    reg [5:0]  r0_r = 6'd0;
    reg [2:0]  frow_r = 3'd0;
    always @(posedge clk) begin : vert
        reg [10:0] yv, fy;
        yv = cy - 11'd80;
        fy = cy - 11'd28;
        gy_r   <= (cy >= 11'd80) && (yv[10:4] < 7'd38) && (yv[3:0] < 4'd12);
        r0_r   <= yv[9:4];
        by_r   <= (cy >= 11'd28) && (cy < 11'd52);
        frow_r <= div3(fy[4:0]);
        sy_r   <= (cy >= 11'd16) && (cy < 11'd64);
    end

    // ---- horizontal: counters for the drawn pixel xa = cx + 3, loaded from xn = cx + 4 (the next xa)
    wire [11:0] xn = cx + 12'd4;
    // grid columns: x = 24 + 416 * gcol + goff
    reg        gon = 1'b0;
    reg [1:0]  gcol = 2'd0;
    reg [8:0]  goff = 9'd0;
    // a slot's bits: from goff = 16, bit bi (0 = the most significant) at offset bo (0..10)
    reg        bon = 1'b0;
    reg [4:0]  bi = 5'd0;
    reg [3:0]  bo = 4'd0;
    // banner groups: x = 112 + 288 * grp + gx; digits from gx = 32: digit dg (0..7), its column fx (0..23) = 3 * fc + f3
    reg        ban = 1'b0, don = 1'b0;
    reg [1:0]  grp = 2'd0;
    reg [8:0]  gxo = 9'd0;
    reg [2:0]  dg = 3'd0, fc = 3'd0;
    reg [4:0]  fx = 5'd0;
    reg [1:0]  f3 = 2'd0;
    // plain spans: the status block (24..71) and the heartbeat (1232..1255)
    reg        sx = 1'b0, hx = 1'b0;
    always @(posedge clk) begin
        // grid
        if (xn == 12'd24) begin gon <= 1'b1; gcol <= 2'd0; goff <= 9'd0; end
        else if (gon) begin
            if (goff == 9'd415) begin
                goff <= 9'd0;
                if (gcol == 2'd2) gon <= 1'b0; else gcol <= gcol + 1'b1;
            end else goff <= goff + 1'b1;
        end
        if (gon && goff == 9'd15) begin bon <= 1'b1; bi <= 5'd0; bo <= 4'd0; end
        else if (bon) begin
            if (bo == 4'd10) begin
                bo <= 4'd0;
                if (bi == 5'd31) bon <= 1'b0; else bi <= bi + 1'b1;
            end else bo <= bo + 1'b1;
        end
        // banner
        if (xn == 12'd112) begin ban <= 1'b1; grp <= 2'd0; gxo <= 9'd0; end
        else if (ban) begin
            if (gxo == 9'd287) begin
                gxo <= 9'd0;
                if (grp == 2'd3) ban <= 1'b0; else grp <= grp + 1'b1;
            end else gxo <= gxo + 1'b1;
        end
        if (ban && gxo == 9'd31) begin don <= 1'b1; dg <= 3'd0; fx <= 5'd0; fc <= 3'd0; f3 <= 2'd0; end
        else if (don) begin
            if (fx == 5'd23) begin
                fx <= 5'd0; fc <= 3'd0; f3 <= 2'd0;
                if (dg == 3'd7) don <= 1'b0; else dg <= dg + 1'b1;
            end else begin
                fx <= fx + 1'b1;
                if (f3 == 2'd2) begin f3 <= 2'd0; fc <= fc + 1'b1; end else f3 <= f3 + 1'b1;
            end
        end
        if (xn == 12'd24) sx <= 1'b1; else if (xn == 12'd72) sx <= 1'b0;
        if (xn == 12'd1232) hx <= 1'b1; else if (xn == 12'd1256) hx <= 1'b0;
    end

    // ---- stage 0 (from the counters and the vertical registers)
    wire inm = gon && gy_r && (goff < 9'd12);
    wire inb = bon && gy_r && (bo < 4'd9);
    wire inl0 = by_r && ban && (gxo < 9'd24);
    wire ind0 = by_r && don;
    wire ins0 = sx && sy_r;
    wire inh0 = hx && by_r;

    // ---- stage 1
    reg        inm1 = 0, inb1 = 0, inl1 = 0, ind1 = 0, ins1 = 0, inh1 = 0;
    reg [4:0]  bi1 = 0;
    reg [1:0]  g1 = 0;
    reg [2:0]  d1 = 0, frow1 = 0, fcol1 = 0;
    always @(posedge clk) begin
        slot_addr <= ((gcol == 2'd2) ? 7'd76 : (gcol == 2'd1) ? 7'd38 : 7'd0) + {1'b0, r0_r};
        inm1 <= inm; inb1 <= inb; bi1 <= bi;
        inl1 <= inl0; ind1 <= ind0; ins1 <= ins0; inh1 <= inh0;
        g1 <= grp; d1 <= dg; frow1 <= frow_r; fcol1 <= fc;
    end
    // ---- stage 2: the display bank reads slot_addr (its word is valid in the next clock); the digit's nibble
    reg        inm2 = 0, inb2 = 0, inl2 = 0, ind2 = 0, ins2 = 0, inh2 = 0;
    reg [4:0]  bi2 = 0;
    reg [1:0]  g2 = 0;
    reg [2:0]  frow2 = 0, fcol2 = 0;
    reg [6:0]  slot2 = 0;
    reg [3:0]  nib2 = 0;
    wire [31:0] gv1 = (g1 == 2'd0) ? err : (g1 == 2'd1) ? passes : (g1 == 2'd2) ? uptime : loops;
    always @(posedge clk) begin
        inm2 <= inm1; inb2 <= inb1; bi2 <= bi1; inl2 <= inl1; ind2 <= ind1; ins2 <= ins1; inh2 <= inh1;
        g2 <= g1; frow2 <= frow1; fcol2 <= fcol1; slot2 <= slot_addr;
        nib2 <= gv1[31 - 4 * d1 -: 4];
    end
    // ---- stage 3: slot_data is slot2's word
    wire [7:0]  fb = font(nib2, frow2);
    always @(posedge clk) begin
        if (inm2)      rgb <= mcol(slot2);
        else if (inb2) rgb <= slot_data[31 - bi2] ? C_WH : C_DK;
        else if (ins2) rgb <= !calib ? C_YE : any_err ? C_RD : C_GR;
        else if (inl2) rgb <= (g2 == 2'd0) ? ((err != 0) ? C_RD : C_GR) : (g2 == 2'd1) ? C_CY : (g2 == 2'd2) ? C_WH : C_YE;
        else if (ind2) rgb <= fb[3'd7 - fcol2] ? C_WH : C_BG;
        else if (inh2) rgb <= beat ? C_WH : C_DK;
        else           rgb <= C_BG;
    end
endmodule
`default_nettype wire
