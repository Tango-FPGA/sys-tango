// SPDX-License-Identifier: MIT
// hdmi_acr_sim.sv -- SIM-ONLY twin of hdmi/audio_clock_regeneration_packet.sv for Verilator 5.020, which cannot fold that
// file's real-valued localparams (line 40: 20'(int'(VIDEO_RATE * int'(N) / 128 / AUDIO_RATE)) "can't convert a CAST to
// constant") -- the reason no bench ever instantiated hdmi.sv.  The same logic with the constants worked out for this
// board (VIDEO_RATE 74.25e6, AUDIO_RATE 48000): N = 16 x 48000 / 125 = 6144, CTS = 74.25e6 x 6144 / 128 / 48000 = 74250,
// counter width $clog2(int(74250 x 1.1)) = $clog2(81675) = 17.  Used by sim/tb_hdmi_totals.sv only (for both the
// new and the original hdmi.sv, so their comparison is unaffected); never synthesised.  Derived from hdl-util/hdmi's
// audio_clock_regeneration_packet.sv by Sameer Puri (MIT, hdmi/LICENSE-MIT; NOTICE).
module audio_clock_regeneration_packet
#(
    parameter real VIDEO_RATE = 25.2E6,
    parameter int AUDIO_RATE = 48e3,
    parameter int ACR_N = 0,          // the real file's TANG PATCH 2: 0 = this board's 74.25 MHz values
    parameter int ACR_CTS = 0
)
(
    input logic clk_pixel,
    input logic clk_audio,
    output logic clk_audio_counter_wrap = 0,
    output logic [23:0] header,
    output logic [55:0] sub [3:0]
`ifdef TANGO_HDMI_RETUNE
    , input logic [19:0] acr_cts_rt   // TANG PATCH 3 (Stage 3): the CTS at run time (hdmi.sv modification 5)
`endif
);
localparam int NI = ACR_N != 0 ? ACR_N : 6144;
localparam int CTSI = ACR_CTS != 0 ? ACR_CTS : 74250;
localparam bit [19:0] N = 20'(NI);
localparam int CLK_AUDIO_COUNTER_WIDTH = $clog2(NI / 128);
localparam bit [CLK_AUDIO_COUNTER_WIDTH-1:0] CLK_AUDIO_COUNTER_END = CLK_AUDIO_COUNTER_WIDTH'(NI / 128 - 1);
logic [CLK_AUDIO_COUNTER_WIDTH-1:0] clk_audio_counter = CLK_AUDIO_COUNTER_WIDTH'(0);
logic internal_clk_audio_counter_wrap = 1'd0;
always_ff @(posedge clk_audio)
begin
    if (clk_audio_counter == CLK_AUDIO_COUNTER_END)
    begin
        clk_audio_counter <= CLK_AUDIO_COUNTER_WIDTH'(0);
        internal_clk_audio_counter_wrap <= !internal_clk_audio_counter_wrap;
    end
    else
        clk_audio_counter <= clk_audio_counter + 1'd1;
end
logic [1:0] clk_audio_counter_wrap_synchronizer_chain = 2'd0;
always_ff @(posedge clk_pixel)
    clk_audio_counter_wrap_synchronizer_chain <= {internal_clk_audio_counter_wrap, clk_audio_counter_wrap_synchronizer_chain[1]};
localparam bit [19:0] CYCLE_TIME_STAMP_COUNTER_IDEAL = 20'(CTSI);
localparam int CYCLE_TIME_STAMP_COUNTER_WIDTH = $clog2(CTSI * 11 / 10);   // 17 for 74250 (the real file: $clog2(int(CTS x 1.1)))
logic [19:0] cycle_time_stamp = 20'd0;
logic [CYCLE_TIME_STAMP_COUNTER_WIDTH-1:0] cycle_time_stamp_counter = CYCLE_TIME_STAMP_COUNTER_WIDTH'(0);
always_ff @(posedge clk_pixel)
begin
    if (clk_audio_counter_wrap_synchronizer_chain[1] ^ clk_audio_counter_wrap_synchronizer_chain[0])
    begin
        cycle_time_stamp_counter <= CYCLE_TIME_STAMP_COUNTER_WIDTH'(0);
        cycle_time_stamp <= {(20-CYCLE_TIME_STAMP_COUNTER_WIDTH)'(0), cycle_time_stamp_counter + CYCLE_TIME_STAMP_COUNTER_WIDTH'(1)};
        clk_audio_counter_wrap <= !clk_audio_counter_wrap;
    end
    else
        cycle_time_stamp_counter <= cycle_time_stamp_counter + CYCLE_TIME_STAMP_COUNTER_WIDTH'(1);
end
assign header = {8'd0, 8'd0, 8'd1};
`ifdef TANGO_HDMI_RETUNE
wire [19:0] cts_fixed = acr_cts_rt;
`else
wire [19:0] cts_fixed = CYCLE_TIME_STAMP_COUNTER_IDEAL;
`endif
genvar i;
generate
    for (i = 0; i < 4; i++)
    begin: same_packet
        assign sub[i] = {N[7:0], N[15:8], {4'd0, N[19:16]}, cts_fixed[7:0], cts_fixed[15:8], {4'd0, cts_fixed[19:16]}, 8'd0};
    end
endgenerate
endmodule
