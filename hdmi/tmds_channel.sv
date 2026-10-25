// Implementation of HDMI Spec v1.4a Section 5.4: Encoding, Section 5.2.2.1: Video Guard Band, Section 5.2.3.3: Data Island Guard Bands.
// By Sameer Puri https://github.com/sameer
//
// Modification (MIT terms preserved): PIPELINED for 148.5 MHz.
// The original computes popcount -> 7-deep XOR chain -> popcount -> disparity
// compare/add in ONE cycle (15 logic levels, Fmax ~102 MHz). Stage 1 now
// registers q_m, both branch candidate symbols/adders, and the N1/N0 flags
// (all feed-forward); the acc feedback loop is only a 3-way select + 5-bit
// add. Every mode path (control/terc4/guard) is registered one stage too, so
// ALL symbols shift uniformly by one pixel -- invisible to the sink.
// Two feed-forward stages: q_m registered after the XOR chain; adders/flags
// registered after the second popcount (symbols derived, not stored - diet). Verified bit-identical (2-cycle-delayed) against the original by
// an A/B testbench before silicon.

module tmds_channel
#(
    // TMDS Channel number.
    // There are only 3 possible channel numbers in HDMI 1.4a: 0, 1, 2.
    parameter int CN = 0
)
(
    input logic clk_pixel,
    input logic [7:0] video_data,
    input logic [3:0] data_island_data,
    input logic [1:0] control_data,
    input logic [2:0] mode,  // Mode select (0 = control, 1 = video, 2 = video guard, 3 = island, 4 = island guard)
    output logic [9:0] tmds = 10'b1101010100
);

// See Section 5.4.4.1
// Below is a direct implementation of Figure 5-7, using the same variable names.

logic signed [4:0] acc = 5'sd0;

logic [8:0] q_m;
logic [9:0] q_out;
logic [9:0] video_coding;
assign video_coding = q_out;

logic [3:0] N1D;
logic signed [4:0] N1q_m07;
logic signed [4:0] N0q_m07;
always_comb
begin
    N1D = video_data[0] + video_data[1] + video_data[2] + video_data[3] + video_data[4] + video_data[5] + video_data[6] + video_data[7];
    case(q_m[0] + q_m[1] + q_m[2] + q_m[3] + q_m[4] + q_m[5] + q_m[6] + q_m[7])
        4'b0000: N1q_m07 = 5'sd0;
        4'b0001: N1q_m07 = 5'sd1;
        4'b0010: N1q_m07 = 5'sd2;
        4'b0011: N1q_m07 = 5'sd3;
        4'b0100: N1q_m07 = 5'sd4;
        4'b0101: N1q_m07 = 5'sd5;
        4'b0110: N1q_m07 = 5'sd6;
        4'b0111: N1q_m07 = 5'sd7;
        4'b1000: N1q_m07 = 5'sd8;
        default: N1q_m07 = 5'sd0;
    endcase
    N0q_m07 = 5'sd8 - N1q_m07;
end

logic signed [4:0] acc_add;

integer i;

// ---- stage 1: everything feed-forward, combinational then REGISTERED ----
always_comb
begin
    if (N1D > 4'd4 || (N1D == 4'd4 && video_data[0] == 1'd0))
    begin
        q_m[0] = video_data[0];
        for(i = 0; i < 7; i++)
            q_m[i + 1] = q_m[i] ~^ video_data[i + 1];
        q_m[8] = 1'b0;
    end
    else
    begin
        q_m[0] = video_data[0];
        for(i = 0; i < 7; i++)
            q_m[i + 1] = q_m[i] ^ video_data[i + 1];
        q_m[8] = 1'b1;
    end
end

// stage A register: q_m survives a cycle so the XOR chain + first popcount
// don't share a cycle with the second popcount + candidate adders.
logic [8:0] q_m_r = 9'd0;
always_ff @(posedge clk_pixel) q_m_r <= q_m;

// second popcount, now over the REGISTERED q_m
logic signed [4:0] N1B, N0B;
always_comb
begin
    case(q_m_r[0] + q_m_r[1] + q_m_r[2] + q_m_r[3] + q_m_r[4] + q_m_r[5] + q_m_r[6] + q_m_r[7])
        4'b0000: N1B = 5'sd0;
        4'b0001: N1B = 5'sd1;
        4'b0010: N1B = 5'sd2;
        4'b0011: N1B = 5'sd3;
        4'b0100: N1B = 5'sd4;
        4'b0101: N1B = 5'sd5;
        4'b0110: N1B = 5'sd6;
        4'b0111: N1B = 5'sd7;
        4'b1000: N1B = 5'sd8;
        default: N1B = 5'sd0;
    endcase
    N0B = 5'sd8 - N1B;
end

// stage B1 (step-D1 cut): register the SECOND POPCOUNT before the adders.
// q_m_r -> popcount -> adder in one cycle was the recurring -0.1 ns family
// (tmds_gen[*] q_m_r -> addB*_r). Splitting it retires that family for
// good; total feed-forward delay becomes 3.
logic signed [4:0] N1B_r = 5'sd0, N0B_r = 5'sd0;
logic [8:0]        q_m_r1b = 9'd0;
always_ff @(posedge clk_pixel)
begin
    N1B_r   <= N1B;
    N0B_r   <= N0B;
    q_m_r1b <= q_m_r;
end

// stage B2: adders + flags registered (they feed the acc loop). The three
// candidate SYMBOLS are NOT registered -- all derive from q_m delayed one
// more stage (q_m_r2) with wiring + inverters, and q_out feeds only the
// tmds output mux (slack path), never the acc loop.
logic [8:0]        q_m_r2 = 9'd0;
logic signed [4:0] addA_r = 5'sd0, addB1_r = 5'sd0, addB2_r = 5'sd0;
logic eq_r = 1'b0, gt_r = 1'b0, lt_r = 1'b0;
always_ff @(posedge clk_pixel)
begin
    q_m_r2  <= q_m_r1b;
    addA_r  <= q_m_r1b[8] ? (N1B_r - N0B_r) : (N0B_r - N1B_r);
    addB1_r <= (N0B_r - N1B_r) + (q_m_r1b[8] ? 5'sd2 : 5'sd0);
    addB2_r <= (N1B_r - N0B_r) - (~q_m_r1b[8] ? 5'sd2 : 5'sd0);
    eq_r <= N1B_r == N0B_r;
    gt_r <= N1B_r > N0B_r;
    lt_r <= N1B_r < N0B_r;
end
logic [9:0] qA_c, qB1_c, qB2_c;
assign qA_c  = q_m_r2[8] ? {~q_m_r2[8], q_m_r2[8], q_m_r2[7:0]}
                         : {~q_m_r2[8], q_m_r2[8], ~q_m_r2[7:0]};
assign qB1_c = {1'b1, q_m_r2[8], ~q_m_r2[7:0]};
assign qB2_c = {1'b0, q_m_r2[8], q_m_r2[7:0]};

// ---- stage 2: the (short) disparity loop -------------------------------
logic branchA, condB;
assign branchA = (acc == 5'sd0) || eq_r;
assign condB   = (acc > 5'sd0 && gt_r) || (acc < 5'sd0 && lt_r);
always_comb
begin
    q_out   = branchA ? qA_c   : (condB ? qB1_c   : qB2_c);
    acc_add = branchA ? addA_r : (condB ? addB1_r : addB2_r);
end

// the non-video coding paths get the SAME one-stage delay so every symbol
// stays aligned; mode is delayed alongside.
logic [2:0] mode_r = 3'd0, mode_r2 = 3'd0, mode_r3 = 3'd0;
logic [9:0] control_coding_r = 10'b1101010100, control_coding_r2 = 10'b1101010100, control_coding_r3 = 10'b1101010100;
logic [9:0] terc4_coding_r = 10'd0, terc4_coding_r2 = 10'd0, terc4_coding_r3 = 10'd0;
logic [9:0] data_guard_band_r = 10'd0, data_guard_band_r2 = 10'd0, data_guard_band_r3 = 10'd0;

always_ff @(posedge clk_pixel) acc <= mode_r3 != 3'd1 ? 5'sd0 : acc + acc_add;

// See Section 5.4.2
logic [9:0] control_coding;
always_comb
begin
    unique case(control_data)
        2'b00: control_coding = 10'b1101010100;
        2'b01: control_coding = 10'b0010101011;
        2'b10: control_coding = 10'b0101010100;
        2'b11: control_coding = 10'b1010101011;
    endcase
end

// See Section 5.4.3
logic [9:0] terc4_coding;
always_comb
begin
    unique case(data_island_data)
        4'b0000 : terc4_coding = 10'b1010011100;
        4'b0001 : terc4_coding = 10'b1001100011;
        4'b0010 : terc4_coding = 10'b1011100100;
        4'b0011 : terc4_coding = 10'b1011100010;
        4'b0100 : terc4_coding = 10'b0101110001;
        4'b0101 : terc4_coding = 10'b0100011110;
        4'b0110 : terc4_coding = 10'b0110001110;
        4'b0111 : terc4_coding = 10'b0100111100;
        4'b1000 : terc4_coding = 10'b1011001100;
        4'b1001 : terc4_coding = 10'b0100111001;
        4'b1010 : terc4_coding = 10'b0110011100;
        4'b1011 : terc4_coding = 10'b1011000110;
        4'b1100 : terc4_coding = 10'b1010001110;
        4'b1101 : terc4_coding = 10'b1001110001;
        4'b1110 : terc4_coding = 10'b0101100011;
        4'b1111 : terc4_coding = 10'b1011000011;
    endcase
end

// See Section 5.2.2.1
logic [9:0] video_guard_band;
generate
    if (CN == 0 || CN == 2)
        assign video_guard_band = 10'b1011001100;
    else
        assign video_guard_band = 10'b0100110011;
endgenerate

// See Section 5.2.3.3
logic [9:0] data_guard_band;
generate
    if (CN == 1 || CN == 2)
        assign data_guard_band = 10'b0100110011;
    else
        assign data_guard_band = control_data == 2'b00 ? 10'b1010001110
            : control_data == 2'b01 ? 10'b1001110001
            : control_data == 2'b10 ? 10'b0101100011
            : 10'b1011000011;
endgenerate

// Apply selected mode (one stage later than the original; uniform shift).
always_ff @(posedge clk_pixel)
begin
    mode_r <= mode;                     mode_r2 <= mode_r;   mode_r3 <= mode_r2;
    control_coding_r <= control_coding; control_coding_r2 <= control_coding_r; control_coding_r3 <= control_coding_r2;
    terc4_coding_r <= terc4_coding;     terc4_coding_r2 <= terc4_coding_r;     terc4_coding_r3 <= terc4_coding_r2;
    data_guard_band_r <= data_guard_band; data_guard_band_r2 <= data_guard_band_r; data_guard_band_r3 <= data_guard_band_r2;
end

always @(posedge clk_pixel)
begin
    case (mode_r3)
        3'd0: tmds <= control_coding_r3;
        3'd1: tmds <= video_coding;
        3'd2: tmds <= video_guard_band;
        3'd3: tmds <= terc4_coding_r3;
        3'd4: tmds <= data_guard_band_r3;
    endcase
end

endmodule
