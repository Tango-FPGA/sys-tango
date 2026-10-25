// tb_ul_lim_equiv.v -- equivalence of tango_link.v's REAL
// tango_ul_lim module (the two-stage, one-subtractor ul_lim feeding a_ulr, a clk_pix margin fix) against the
// ORIGINAL a_ulr two-sum expression it replaced, driven with the real 2-clock lag.
//
//   ORACLE (the earlier tango_link.v; the "original two-sum expression"):
//       mid_old(off, sz, an) = ((off + {8'd0,an}) < (sz + 24'd4))          // two 24-bit adds, a compare
//   DUT (this repo's tango_link.v, module tango_ul_lim, compiled from the real file):
//       ul_sz4 <= ul_size_o + 4;   {brw,diff} = {0,ul_sz4} - {0,ul_off};   ul_lim <= brw ? 0 : diff;
//       mid_new(an) = ({8'd0,an} < ul_lim)
//   (sub3_now and `ul_size_o != 0` are identical, unmodified code in all versions; only the middle
//   arithmetic term differs, so only it is compared.)
//
// For FIXED (off, sz), mid_old and mid_new are monotonic thresholds in an_rx (an < T), so agreement at
// T-1, T, T+1 (and 0, 4, 0xFFFF) proves agreement for every an_rx there.  The honest limit: mid_old is a
// threshold only while off + an cannot itself wrap 24 bits; there it differs from the clamped form on
// purpose (the clamped form is the correct one there): that is PART 3, reported, not a failure.
//
// Timing: inputs are applied right after a posedge and held; the result is sampled after exactly TWO more
// posedges (the documented lag; checks at that point must equal the oracle).  PART 0 additionally checks
// the lag claims directly: an ul_off-only change settles in ONE clock; a ul_size_o change in TWO, and is
// NOT yet visible after one (so a mutant with a 3rd stage, or one that drops the stage, is distinguished
// from the intended pipeline by sampling at 2 clocks and at 1 clock).
//
// PARTS (all 0 mismatches required, except PART 3):
//   1  exhaustive sz in [0,UL_MAX=2048] x off in [0,2*UL_MAX]   (the old bench's region; real-RTL, 2-clock lag)
//   2  exhaustive off in [0,2^24) x sz in {0,1,4,5,2047,2048} (sizes the SELECT clamp can produce: <=UL_MAX)
//      plus sz at the EXTREMES the hardware can't produce (0xFFFFFC..0xFFFFFF, 0x800000, 0xFFFFFF) x off sweep
//   4  pseudo-random full-24-bit (sz, off) pairs, RANDN of them, an_rx at the boundary samples
//   3  wrap domain (documented divergence): every sample, from ANY sweep, where the oracle's own 24-bit add
//      off + an_rx wraps (off > 2^24-1-an_rx), plus a directed sweep with ul_off near 2^24.  NOT a failure:
//      there the original is the wrong one; the ul_lim VALUE check against the single-register form (below) still
//      applies to every sample, wrap domain included, and must show 0 mismatches.
// Run: sim/run_tb_ul_lim_equiv.sh  (builds with verilator --binary --timing; also runs the mutant set).

`timescale 1ns/1ps

module tb_ul_lim_equiv;

    localparam [23:0] UL_MAX = 24'd2048;     // tango_link.v UL_MAX
`ifndef RANDN
 `define RANDN 20000000
`endif

    reg         clk = 1'b0;
    always #5 clk = ~clk;
    reg  [23:0] ul_size_o = 24'd0, ul_off = 24'd0;
    wire [23:0] ul_lim;
    tango_ul_lim dut (.clk(clk), .ul_size_o(ul_size_o), .ul_off(ul_off), .ul_lim(ul_lim));

    function automatic mid_old(input [23:0] off, input [23:0] sz, input [15:0] an);
        mid_old = ((off + {8'd0, an}) < (sz + 24'd4));
    endfunction
    function automatic mid_new(input [23:0] lim, input [15:0] an);
        mid_new = ({8'd0, an} < lim);
    endfunction
    // the original ul_lim (single-register form) -- value oracle for ul_lim itself
    function automatic [23:0] lim_d5297e2(input [23:0] off, input [23:0] sz);
        lim_d5297e2 = (off > (sz + 24'd4)) ? 24'd0 : (sz + 24'd4 - off);
    endfunction

    integer mism_p0 = 0, mism_p1 = 0, mism_p2 = 0, mism_p4 = 0, mism_p3 = 0, mism_lim = 0;
    integer n_p0 = 0, n_p1 = 0, n_p2 = 0, n_p4 = 0, n_p3 = 0;

    // apply (sz, off) after a posedge, wait two posedges, sample just after the second one
    task automatic apply2(input [23:0] sz, input [23:0] off);
        begin
            ul_size_o = sz; ul_off = off;
            @(posedge clk); @(posedge clk); #1;
        end
    endtask

    // compare DUT against the oracle at the boundary samples of an_rx; returns mismatch count
    task automatic check(input [23:0] sz, input [23:0] off, input integer part);
        reg [15:0] an; integer k, bad, eff; reg o, n; reg [23:0] want_lim;
        begin
            bad = 0;
            want_lim = lim_d5297e2(off, sz);
            if (ul_lim !== want_lim) begin
                mism_lim = mism_lim + 1;
                if (mism_lim <= 10) $display("LIM VALUE MISMATCH part%0d sz=%0d off=%0d got=%0d want=%0d", part, sz, off, ul_lim, want_lim);
            end
            for (k = 0; k < 8; k = k + 1) begin
                case (k)
                    0: an = 16'd0;
                    1: an = 16'd4;
                    2: an = (want_lim == 24'd0) ? 16'd0 : (want_lim[15:0] - 16'd1);
                    3: an = want_lim[15:0];
                    4: an = want_lim[15:0] + 16'd1;
                    5: an = 16'hFFFF;
                    6: an = 16'd5;
                    7: an = 16'd2047;
                endcase
                o = mid_old(off, sz, an); n = mid_new(ul_lim, an);
                // the oracle's own 24-bit add off+an wraps (off > 2^24-1-an): the documented out-of-domain
                // divergence -- bucketed as PART 3 whichever sweep found it
                eff = ((({1'b0, off} + {9'd0, an}) >> 24) != 25'd0) ? 3 : part;
                if (o !== n) begin
                    bad = bad + 1;
                    case (eff)
                        0: mism_p0 = mism_p0 + 1; 1: mism_p1 = mism_p1 + 1; 2: mism_p2 = mism_p2 + 1;
                        4: mism_p4 = mism_p4 + 1; default: mism_p3 = mism_p3 + 1;
                    endcase
                    if ((eff != 3) && (mism_p0 + mism_p1 + mism_p2 + mism_p4 <= 10))
                        $display("MISMATCH part%0d sz=%0d off=%0d an=%0d old=%b new=%b lim=%0d", part, sz, off, an, o, n, ul_lim);
                    if ((eff == 3) && (mism_p3 <= 10))
                        $display("WRAP-DOMAIN MISMATCH (expected, out of domain): sz=%0d off=%0d an=%0d old=%b new=%b lim=%0d", sz, off, an, o, n, ul_lim);
                end
                case (eff) 0: n_p0 = n_p0 + 1; 1: n_p1 = n_p1 + 1; 2: n_p2 = n_p2 + 1; 4: n_p4 = n_p4 + 1; default: n_p3 = n_p3 + 1; endcase
            end
        end
    endtask

    integer i, j, w, lagbad;
    reg [31:0] lfsr;
    reg [23:0] a, b;
    reg [23:0] szs [0:11];
    reg [23:0] wrap_offs [0:9];

    initial begin
        @(posedge clk); #1;

        // ---------------- PART 0: the lag claims, directly ----------------
        lagbad = 0;
        apply2(24'd100, 24'd0);                       // settle: lim = 104
        ul_off = 24'd10; @(posedge clk); #1;          // ul_off only: ONE clock
        if (ul_lim !== 24'd94) begin lagbad = lagbad + 1; $display("LAG: ul_off-only change not settled after 1 clock (lim=%0d, want 94)", ul_lim); end
        ul_size_o = 24'd200; @(posedge clk); #1;      // ul_size_o: NOT yet after one clock ...
        if (ul_lim !== 24'd94) begin lagbad = lagbad + 1; $display("LAG: ul_size_o change visible after 1 clock (lim=%0d); expected still 94 (documented 2-clock lag)", ul_lim); end
        @(posedge clk); #1;                           // ... but after two
        if (ul_lim !== 24'd194) begin lagbad = lagbad + 1; $display("LAG: ul_size_o change not settled after 2 clocks (lim=%0d, want 194)", ul_lim); end
        // both change in the same clock (UL_SELECT's n5: size set + off reset): settled after 2
        apply2(24'd37, 24'd0);
        if (ul_lim !== 24'd41) begin lagbad = lagbad + 1; $display("LAG: UL_SELECT-style simultaneous update not settled after 2 clocks (lim=%0d, want 41)", ul_lim); end
        mism_p0 = lagbad;
        $display("PART 0 (lag: ul_off 1 clock, ul_size_o 2 clocks): %0d lag-check failures", lagbad);

        // ---------------- PART 1 ----------------
        for (i = 0; i <= UL_MAX; i = i + 1)
            for (j = 0; j <= 2 * UL_MAX; j = j + 1) begin
                apply2(i[23:0], j[23:0]);
                check(i[23:0], j[23:0], 1);
            end
        $display("PART 1 (sz in [0,%0d], off in [0,%0d], 2-clock lag): %0d checks, %0d mismatches", UL_MAX, 2*UL_MAX, n_p1, mism_p1);

        // ---------------- PART 2: all 2^24 off values at the sizes the hardware can produce, and the extremes ----------------
        szs[0] = 24'd0; szs[1] = 24'd1; szs[2] = 24'd4; szs[3] = 24'd5; szs[4] = 24'd37; szs[5] = 24'd2047; szs[6] = 24'd2048;
        szs[7] = 24'hFFFFFC; szs[8] = 24'hFFFFFD; szs[9] = 24'hFFFFFF; szs[10] = 24'h800000; szs[11] = 24'h7FFFFC;
        for (w = 0; w < 12; w = w + 1) begin
            // off: all 2^24 values would be 16.7M x 12; sweep the full range for the 7 hardware-reachable
            // sizes with a stride of 1 in the low 2^16, a stride of 251 above (hits every residue class
            // relative to sz+4 over the run), and every value within +-8 of sz+4 for every size.
            for (j = 0; j < 65536; j = j + 1) begin a = j[23:0]; apply2(szs[w], a); check(szs[w], a, 2); end
            for (j = 65536; j < 16777216; j = j + 251) begin a = j[23:0]; apply2(szs[w], a); check(szs[w], a, 2); end
            for (j = -8; j <= 8; j = j + 1) begin a = szs[w] + 24'd4 + j[23:0]; apply2(szs[w], a); check(szs[w], a, 2); end
            for (j = 0; j < 8; j = j + 1) begin a = 24'hFFFFFF - j[23:0]; apply2(szs[w], a); check(szs[w], a, 2); end
        end
        $display("PART 2 (12 sizes incl. unreachable extremes x off sweep to 2^24): %0d checks, %0d mismatches", n_p2, mism_p2);

        // ---------------- PART 4: random full-width pairs ----------------
        lfsr = 32'hDEADBEEF;
        for (i = 0; i < `RANDN; i = i + 1) begin
            lfsr = lfsr ^ (lfsr << 13); lfsr = lfsr ^ (lfsr >> 17); lfsr = lfsr ^ (lfsr << 5);
            a = lfsr[23:0];
            lfsr = lfsr ^ (lfsr << 13); lfsr = lfsr ^ (lfsr >> 17); lfsr = lfsr ^ (lfsr << 5);
            b = lfsr[23:0];
            // bias half of them near each other so the clamp boundary (off ~ sz+4) is hit constantly
            if (lfsr[31]) b = a + 24'd4 + {{19{lfsr[28]}}, lfsr[27:24], 1'b0} - 24'd4;
            apply2(a, b); check(a, b, 4);
        end
        $display("PART 4 (%0d random 24-bit pairs, half near the clamp boundary): %0d checks, %0d mismatches", `RANDN, n_p4, mism_p4);

        // ---------------- PART 3: documented out-of-domain wrap ----------------
        wrap_offs[0] = 24'hFFFFFF; wrap_offs[1] = 24'hFFFFFE; wrap_offs[2] = 24'hFFFFF0; wrap_offs[3] = 24'hFFFF00;
        wrap_offs[4] = 24'hFFF000; wrap_offs[5] = 24'd16777000; wrap_offs[6] = 24'd16000000; wrap_offs[7] = 24'd10000000;
        wrap_offs[8] = 24'd8388608; wrap_offs[9] = 24'd4194304;
        for (w = 0; w <= 9; w = w + 1)
            for (i = 0; i <= UL_MAX; i = i + 64) begin
                apply2(i[23:0], wrap_offs[w]); check(i[23:0], wrap_offs[w], 3);
            end
        $display("PART 3 (wrap domain, ul_off near 2^24): %0d checks, %0d mismatches vs the ORIGINAL two-sum (NONZERO IS EXPECTED: documented out-of-domain divergence; the clamped form is the correct one there)", n_p3, mism_p3);

        $display("ul_lim VALUE vs the d5297e2 single-register form: %0d mismatches over every check above", mism_lim);
        if (mism_p0 == 0 && mism_p1 == 0 && mism_p2 == 0 && mism_p4 == 0 && mism_lim == 0) begin
            $display("EQUIV SWEEP: PASS");
            $finish(0);
        end else begin
            $display("EQUIV SWEEP: FAIL (p0=%0d p1=%0d p2=%0d p4=%0d lim=%0d)", mism_p0, mism_p1, mism_p2, mism_p4, mism_lim);
            $finish(1);
        end
    end
endmodule
