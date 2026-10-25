// Implementation of HDMI audio clock regeneration packet
// By Sameer Puri https://github.com/sameer

// See HDMI 1.4b Section 5.3.3
module audio_clock_regeneration_packet
#(
    parameter real VIDEO_RATE = 25.2E6,
    parameter int AUDIO_RATE = 48e3,
    // TANG PATCH 2: N and CTS as integers for a pixel clock other than 74.25 MHz
    // (hdmi.sv local modification 4; tango_cfg.vh TANGO_HDMI_PLL_<f>). 0 = computed below exactly as before.
    parameter int ACR_N = 0,
    parameter int ACR_CTS = 0
)
(
    input logic clk_pixel,
    input logic clk_audio,
    output logic clk_audio_counter_wrap = 0,
    output logic [23:0] header,
    output logic [55:0] sub [3:0]
`ifdef TANGO_HDMI_RETUNE
    , input logic [19:0] acr_cts_rt   // TANG PATCH 3 (TANGO_HDMI_RETUNE): the CTS at run time (hdmi.sv modification 5)
`endif
);

// See Section 7.2.3, values derived from "Other" row in Tables 7-1, 7-2, 7-3.
localparam bit [19:0] N = ACR_N != 0 ? 20'(ACR_N) : AUDIO_RATE % 125 == 0 ? 20'(16 * AUDIO_RATE / 125) : AUDIO_RATE % 225 == 0 ? 20'(32 * AUDIO_RATE / 225) : 20'(AUDIO_RATE * 16 / 125);

localparam int CLK_AUDIO_COUNTER_WIDTH = $clog2(N / 128);
localparam bit [CLK_AUDIO_COUNTER_WIDTH-1:0] CLK_AUDIO_COUNTER_END = CLK_AUDIO_COUNTER_WIDTH'(N / 128 - 1);
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

localparam bit [19:0] CYCLE_TIME_STAMP_COUNTER_IDEAL = ACR_CTS != 0 ? 20'(ACR_CTS) : 20'(int'(VIDEO_RATE * int'(N) / 128 / AUDIO_RATE));
localparam int CYCLE_TIME_STAMP_COUNTER_WIDTH = $clog2(20'(int'(real'(CYCLE_TIME_STAMP_COUNTER_IDEAL) * 1.1))); // Account for 10% deviation in audio clock

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

// "An HDMI Sink shall ignore bytes HB1 and HB2 of the Audio Clock Regeneration Packet header."
`ifdef MODEL_TECH
assign header = {8'd0, 8'd0, 8'd1};
`else
assign header = {8'd0, 8'd0, 8'd1};   // was X,X -- Gowin X-sweep hazard
`endif

// TANG PATCH: our clk_audio and clk_pixel are COHERENT (both divided from
// the single 50 MHz crystal), so per HDMI 1.4b 7.2.1 the source should send
// the FIXED ideal CTS rather than a runtime measurement. The stock code's
// measured cycle_time_stamp jitters +/-1 because our clk_audio (a fractional
// accumulator toggle) has +/-1 pixel-clock edge jitter; the sink then
// regenerates a jittery audio clock, whose DAC sampling error grows with
// signal slew -> audible "crack" on loud/busy passages (SMB overworld),
// inaudible when quiet. Sending the constant ideal removes that jitter at
// the source. CYCLE_TIME_STAMP_COUNTER_IDEAL = VIDEO_RATE*N/128/AUDIO_RATE
// = 74.25e6*6144/128/48000 = 74250 for 720p/48kHz.
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
