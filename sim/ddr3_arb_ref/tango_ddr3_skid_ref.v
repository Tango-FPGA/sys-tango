//==============================================================================
// tango_ddr3_skid_ref -- a 2-entry queue whose input ready is a register: the producer's logic never waits on the
// consumer's same-clock decision (the consumer pops the head combinationally, e.g. from the DDR3 IP's cmd_ready).
// in_ready high at a clock guarantees room for one word that clock.  out_pop only with out_valid.
//==============================================================================
`default_nettype none
module tango_ddr3_skid_ref #(
    parameter integer W = 32
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          in_valid,
    input  wire [W-1:0]  in_data,
    output reg           in_ready = 1'b1,
    output wire          out_valid,
    output wire [W-1:0]  out_data,
    input  wire          out_pop
);
    reg [W-1:0] e0 = {W{1'b0}}, e1 = {W{1'b0}};
    reg         rp = 1'b0, wp = 1'b0;
    reg [1:0]   n = 2'd0;
    wire        push = in_valid && in_ready;
    wire [2:0]  nn = {1'b0, n} + (push ? 3'd1 : 3'd0) - (out_pop ? 3'd1 : 3'd0);
    assign out_valid = (n != 2'd0);
    assign out_data  = rp ? e1 : e0;
    always @(posedge clk) begin
        if (push) begin if (wp) e1 <= in_data; else e0 <= in_data; end
        if (rst) begin rp <= 1'b0; wp <= 1'b0; n <= 2'd0; in_ready <= 1'b1; end
        else begin
            if (push) wp <= ~wp;
            if (out_pop) rp <= ~rp;
            n <= nn[1:0];
            in_ready <= (nn <= 3'd1);
        end
    end
endmodule
`default_nettype wire
