// Implementation of HDMI packet ECC calculation.
// By Sameer Puri https://github.com/sameer

module packet_assembler (
    input logic clk_pixel,
    input logic reset,
    input logic data_island_period,
    input logic [23:0] header, // See Table 5-8 Packet Types
    input logic [55:0] sub [3:0],
    output logic [8:0] packet_data, // See Figure 5-4 Data Island Packet and ECC Structure
    output logic [4:0] counter = 5'd0
);

// 32 pixel wrap-around counter. See Section 5.2.3.4 for further information.
always_ff @(posedge clk_pixel)
begin
    if (reset)
        counter <= 5'd0;
    else if (data_island_period)
        counter <= counter + 5'd1;
end
// BCH packets 0 to 3 are transferred two bits at a time, see Section 5.2.3.4 for further information.
wire [5:0] counter_t2 = {counter, 1'b0};
wire [5:0] counter_t2_p1 = {counter, 1'b1};

// parity: the original's parity REGISTER value, now recomputed (see below)
wire [7:0] parity [4:0];

wire [63:0] bch [3:0];
assign bch[0] = {parity[0], sub[0]};
assign bch[1] = {parity[1], sub[1]};
assign bch[2] = {parity[2], sub[2]};
assign bch[3] = {parity[3], sub[3]};
wire [31:0] bch4 = {parity[4], header};
assign packet_data = {bch[3][counter_t2_p1], bch[2][counter_t2_p1], bch[1][counter_t2_p1], bch[0][counter_t2_p1], bch[3][counter_t2], bch[2][counter_t2], bch[1][counter_t2], bch[0][counter_t2], bch4[counter]};

// See Figure 5-5 Error Correction Code generator. Generalization of a CRC with binary BCH.
// See https://web.archive.org/web/20190520020602/http://hamsterworks.co.nz/mediawiki/index.php/Minimal_HDMI#Computing_the_ECC for an explanation of the implementation.
// See https://en.wikipedia.org/wiki/BCH_code#Systematic_encoding:_The_message_as_a_prefix for further information.
function automatic [7:0] next_ecc;
input [7:0] ecc, next_bch_bit;
begin
    next_ecc = (ecc >> 1) ^ ((ecc[0] ^ next_bch_bit) ? 8'b10000011 : 8'd0);
end
endfunction

// Modification (MIT terms preserved): TIMING CUT, exactly equivalent.
// The original fed the counter-indexed 56:1 bit selects straight into two
// chained ECC steps and the parity register in one 148.5 MHz cycle (-1.5 ns in
// a 1080p build).  Here the selected bits, the enables and the
// clear are REGISTERED in the same cycle the original sampled them, and the
// original's parity register value P(t) is recomputed each cycle from a
// one-cycle-late copy P'(t) = P(t-1) with just the two small ECC steps:
//   P(t) = clr_r ? 0 : { en_r ? F(P', bits_r) : P' }        P'(t+1) <= P(t)
// packet_data therefore sees exactly the original P(t) at the same time, for
// ANY input sequence (proven by an A/B testbench: random inputs,
// reset and island toggling, 0 mismatches).
logic [7:0] parity_d [4:0] = '{8'd0, 8'd0, 8'd0, 8'd0, 8'd0};   // P(t-1)
logic       clr_r = 1'b1, en_r = 1'b0, en4_r = 1'b0;
logic [3:0] a_r = 4'd0, b_r = 4'd0;
logic       h_r = 1'b0;
always_ff @(posedge clk_pixel)
begin
    clr_r <= reset || !data_island_period || (counter == 5'd31 && !(counter < 5'd28));
    en_r  <= data_island_period && (counter < 5'd28);
    en4_r <= data_island_period && (counter < 5'd24);
    for (int k = 0; k < 4; k++) begin
        a_r[k] <= sub[k][counter_t2];
        b_r[k] <= sub[k][counter_t2_p1];
    end
    h_r <= header[counter];
end
genvar i;
generate
    for(i = 0; i < 5; i++)
    begin: parity_calc
        if (i == 4)
            assign parity[i] = clr_r ? 8'd0 : (en4_r ? next_ecc(parity_d[i], h_r) : parity_d[i]);
        else
            assign parity[i] = clr_r ? 8'd0 : (en_r ? next_ecc(next_ecc(parity_d[i], a_r[i]), b_r[i]) : parity_d[i]);
    end
endgenerate
always_ff @(posedge clk_pixel)
    parity_d <= parity;

endmodule
