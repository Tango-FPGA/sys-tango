// Implementation of HDMI packet choice logic.
// By Sameer Puri https://github.com/sameer
//
// THINNED COPY (MIT; modified): the stock 256-entry
// headers[]/subs[] mux arrays (indexed by the full 8-bit packet type with
// only 6 entries ever driven) are replaced by an explicit mux over the
// packets we actually send: NULL, ACR, Audio Sample, AVI InfoFrame, Audio
// InfoFrame. The optional Source Product Description InfoFrame is removed
// entirely (it only named the input "Unknown FPGA" on some TVs).

module packet_picker
#(
    parameter int VIDEO_ID_CODE = 4,
    parameter real VIDEO_RATE = 0,
    parameter bit IT_CONTENT = 1'b0,
    parameter int AUDIO_BIT_WIDTH = 0,
    parameter int AUDIO_RATE = 0,
    parameter int ACR_N = 0,          // Tango (TANGO_HDMI_PLL presets): hdmi.sv local modification 4
    parameter int ACR_CTS = 0
)
(
    input logic clk_pixel,
    input logic clk_audio,
    input logic reset,
    input logic video_field_end,
    input logic packet_enable,
    input logic [4:0] packet_pixel_counter,
    input logic [AUDIO_BIT_WIDTH-1:0] audio_sample_word [1:0],
`ifdef TANGO_HDMI_RETUNE
    input logic [19:0] acr_cts_rt,    // Tango (TANGO_HDMI_RETUNE): hdmi.sv local modification 5
`endif
    output logic [23:0] header,
    output logic [55:0] sub [3:0]
);

// Packet select: explicit enumeration of the five packets we transmit.
// Values keep the HDMI packet-type byte for readability; the mux below is
// what synthesizes, not a 256-entry array.
logic [7:0] packet_type = 8'd0;

logic [23:0] header_acr, header_audio, header_avi, header_aif;
logic [55:0] sub_acr [3:0];
logic [55:0] sub_audio [3:0];
logic [55:0] sub_avi [3:0];
logic [55:0] sub_aif [3:0];

// STEP-13 CUT: the 5-way mux output is REGISTERED. packet_enable now
// fires one cycle earlier (hdmi.sv: mod constant 18->19 with its own
// window), so the pipeline is: pick (packet_type + audio words latch) ->
// mux registers (this stage, capturing the NEW audio words) -> emission.
// Emission waveforms are bit-identical; the deep packet_type->56b-mux->
// bit-select->ecc cone into packet_assembler is cut in half.
always_ff @(posedge clk_pixel) begin
    case (packet_type)
        8'd1:    begin header <= header_acr;   sub <= sub_acr;   end
        8'd2:    begin header <= header_audio; sub <= sub_audio; end
        8'h82:   begin header <= header_avi;   sub <= sub_avi;   end
        8'h84:   begin header <= header_aif;   sub <= sub_aif;   end
        // NULL packet: "An HDMI Sink shall ignore bytes HB1 and HB2 of the
        // Null Packet Header and all bytes of the Null Packet Body."
        default: begin header <= 24'd0; sub <= '{56'd0, 56'd0, 56'd0, 56'd0}; end
    endcase
end

// Audio Clock Regeneration Packet
logic clk_audio_counter_wrap;
audio_clock_regeneration_packet #(.VIDEO_RATE(VIDEO_RATE), .AUDIO_RATE(AUDIO_RATE), .ACR_N(ACR_N), .ACR_CTS(ACR_CTS)) audio_clock_regeneration_packet (.clk_pixel(clk_pixel), .clk_audio(clk_audio), .clk_audio_counter_wrap(clk_audio_counter_wrap),
`ifdef TANGO_HDMI_RETUNE
    .acr_cts_rt(acr_cts_rt),
`endif
    .header(header_acr), .sub(sub_acr));

// Audio Sample packet
localparam bit [3:0] SAMPLING_FREQUENCY = AUDIO_RATE == 32000 ? 4'b0011
    : AUDIO_RATE == 44100 ? 4'b0000
    : AUDIO_RATE == 88200 ? 4'b1000
    : AUDIO_RATE == 176400 ? 4'b1100
    : AUDIO_RATE == 48000 ? 4'b0010
    : AUDIO_RATE == 96000 ? 4'b1010
    : AUDIO_RATE == 192000 ? 4'b1110
    : 4'bXXXX;
localparam int AUDIO_BIT_WIDTH_COMPARATOR = AUDIO_BIT_WIDTH < 20 ? 20 : AUDIO_BIT_WIDTH == 20 ? 25 : AUDIO_BIT_WIDTH < 24 ? 24 : AUDIO_BIT_WIDTH == 24 ? 29 : -1;
localparam bit [2:0] WORD_LENGTH = 3'(AUDIO_BIT_WIDTH_COMPARATOR - AUDIO_BIT_WIDTH);
localparam bit WORD_LENGTH_LIMIT = AUDIO_BIT_WIDTH <= 20 ? 1'b0 : 1'b1;

logic [AUDIO_BIT_WIDTH-1:0] audio_sample_word_transfer [1:0];
logic audio_sample_word_transfer_control = 1'd0;
always_ff @(posedge clk_audio)
begin
    audio_sample_word_transfer <= audio_sample_word;
    audio_sample_word_transfer_control <= !audio_sample_word_transfer_control;
end

logic [1:0] audio_sample_word_transfer_control_synchronizer_chain = 2'd0;
always_ff @(posedge clk_pixel)
    audio_sample_word_transfer_control_synchronizer_chain <= {audio_sample_word_transfer_control, audio_sample_word_transfer_control_synchronizer_chain[1]};

logic sample_buffer_current = 1'b0;
logic [1:0] samples_remaining = 2'd0;
logic [23:0] audio_sample_word_buffer [1:0] [3:0] [1:0];
logic [AUDIO_BIT_WIDTH-1:0] audio_sample_word_transfer_mux [1:0];
always_comb
begin
    if (audio_sample_word_transfer_control_synchronizer_chain[0] ^ audio_sample_word_transfer_control_synchronizer_chain[1])
        audio_sample_word_transfer_mux = audio_sample_word_transfer;
    else
        audio_sample_word_transfer_mux = '{audio_sample_word_buffer[sample_buffer_current][samples_remaining][1][23:(24-AUDIO_BIT_WIDTH)], audio_sample_word_buffer[sample_buffer_current][samples_remaining][0][23:(24-AUDIO_BIT_WIDTH)]};
end

logic sample_buffer_used = 1'b0;
logic sample_buffer_ready = 1'b0;

always_ff @(posedge clk_pixel)
begin
    if (sample_buffer_used)
        sample_buffer_ready <= 1'b0;

    if (audio_sample_word_transfer_control_synchronizer_chain[0] ^ audio_sample_word_transfer_control_synchronizer_chain[1])
    begin
        audio_sample_word_buffer[sample_buffer_current][samples_remaining][0] <=  24'(audio_sample_word_transfer_mux[0])<<(24-AUDIO_BIT_WIDTH);
        audio_sample_word_buffer[sample_buffer_current][samples_remaining][1] <=  24'(audio_sample_word_transfer_mux[1])<<(24-AUDIO_BIT_WIDTH);
        if (samples_remaining == 2'd3)
        begin
            samples_remaining <= 2'd0;
            sample_buffer_ready <= 1'b1;
            sample_buffer_current <= !sample_buffer_current;
        end
        else
            samples_remaining <= samples_remaining + 1'd1;
    end
end

logic [23:0] audio_sample_word_packet [3:0] [1:0];
logic [3:0] audio_sample_word_present_packet;

logic [7:0] frame_counter = 8'd0;
always_ff @(posedge clk_pixel)
begin
    if (reset)
    begin
        frame_counter <= 8'd0;
    end
    else if (packet_pixel_counter == 5'd30 && packet_type == 8'h02) // Keep track of current IEC 60958 frame (sampled at 30: with the early pick, packet_type advances during 31)
    begin
        frame_counter = frame_counter + 8'd4;
        if (frame_counter >= 8'd192)
            frame_counter = frame_counter - 8'd192;
    end
end
audio_sample_packet #(.SAMPLING_FREQUENCY(SAMPLING_FREQUENCY), .WORD_LENGTH({{WORD_LENGTH[0], WORD_LENGTH[1], WORD_LENGTH[2]}, WORD_LENGTH_LIMIT})) audio_sample_packet (.frame_counter(frame_counter), .audio_sample_word(audio_sample_word_packet), .audio_sample_word_present(audio_sample_word_present_packet), .header(header_audio), .sub(sub_audio));


auxiliary_video_information_info_frame #(
    .VIDEO_ID_CODE(7'(VIDEO_ID_CODE)),
    .IT_CONTENT(IT_CONTENT)
) auxiliary_video_information_info_frame(.header(header_avi), .sub(sub_avi));


audio_info_frame audio_info_frame(.header(header_aif), .sub(sub_aif));


// "A Source shall always transmit... [an InfoFrame] at least once per two Video Fields"
logic audio_info_frame_sent = 1'b0;
logic auxiliary_video_information_info_frame_sent = 1'b0;
logic last_clk_audio_counter_wrap = 1'b0;
always_ff @(posedge clk_pixel)
begin
    if (sample_buffer_used)
        sample_buffer_used <= 1'b0;

    if (reset || video_field_end)
    begin
        audio_info_frame_sent <= 1'b0;
        auxiliary_video_information_info_frame_sent <= 1'b0;
        packet_type <= 8'd0;   // was 8'dx -- Gowin sweeps audio pkt logic on X (findings report Finding #2)
    end
    else if (packet_enable)
    begin
        if (last_clk_audio_counter_wrap ^ clk_audio_counter_wrap)
        begin
            packet_type <= 8'd1;
            last_clk_audio_counter_wrap <= clk_audio_counter_wrap;
        end
        else if (sample_buffer_ready)
        begin
            packet_type <= 8'd2;
            audio_sample_word_packet <= audio_sample_word_buffer[!sample_buffer_current];
            audio_sample_word_present_packet <= 4'b1111;
            sample_buffer_used <= 1'b1;
        end
        else if (!audio_info_frame_sent)
        begin
            packet_type <= 8'h84;
            audio_info_frame_sent <= 1'b1;
        end
        else if (!auxiliary_video_information_info_frame_sent)
        begin
            packet_type <= 8'h82;
            auxiliary_video_information_info_frame_sent <= 1'b1;
        end
        else
            packet_type <= 8'd0;
    end
end

endmodule
