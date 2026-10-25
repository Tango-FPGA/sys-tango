// Implementation of HDMI Spec v1.4a
// By Sameer Puri https://github.com/sameer
//
// THINNED COPY (MIT; modified): single-mode CEA VIC 4
// (1280x720p60, 74.25 MHz) only. Removed: all other video-mode arms, the
// VIDEO_RATE refresh ladder, the DVI_OUTPUT branch, START_X/START_Y, and
// the Source Product Description InfoFrame (optional per spec). This is
// the NESTang thinning doctrine applied at the source level: one fixed
// mode, so nothing pays rent. Wire behavior is unchanged except the SPD
// packet is no longer sent.

module hdmi
#(
    // IT content bit for the AVI InfoFrame (framebuffer-style pixels,
    // full-range RGB).
    parameter bit IT_CONTENT = 1'b1,

    // 16-bit or more L-PCM at 32/44.1/48 kHz (Section 7.3 minimum).
    parameter int AUDIO_RATE = 48000,

    // 16..24-bit audio.
    parameter int AUDIO_BIT_WIDTH = 16,

    // ⭐ Tango (local modification 2): the raster totals and the AVI
    // InfoFrame's VIC as parameters (see the second local modification below).
    // The defaults are the upstream 720p VARIANT values: 1650 x 750, VIC 4.
    parameter int FRAME_WIDTH = 1650,
    parameter int FRAME_HEIGHT = 750,
    parameter int VIC = 4,

    // ⭐ Tango (local modification 4): the
    // Audio Clock Regeneration packet's N and CTS as integers when the pixel
    // clock is not 74.25 MHz (tango_cfg.vh TANGO_HDMI_PLL_<f>: CTS = f_pix x N /
    // (128 x 48000), e.g. 74.5875 MHz -> N 12288, CTS 149175). 0 = computed from
    // AUDIO_RATE / VIDEO_RATE (74.25 MHz) exactly as before: N 6144, CTS 74250.
    parameter int ACR_N = 0,
    parameter int ACR_CTS = 0
)
(
    input logic clk_pixel_x5,
    input logic clk_pixel,
    input logic clk_audio,
    // synchronous reset back to 0,0
    input logic reset,
    input logic [23:0] rgb,
    input logic [AUDIO_BIT_WIDTH-1:0] audio_sample_word [1:0],
`ifdef TANGO_HDMI_RETUNE
    // ⭐ Tango (LOCAL MODIFICATION 5): the ACR packet's CTS at run time -- the pixel
    // clock's preset is switched by tango_pll_retune (N stays the ACR_N parameter, 12288 for every preset: CTS 148500 at
    // 74.25 MHz, 149175 at 74.5875). Quasi-static: it changes only while this module is held in reset (the PLL re-lock).
    input logic [19:0] acr_cts_rt,
`endif

    // Patched for Tang: expose raw 10-bit TMDS words; external
    // tang_tmds_ser does the OSER10 serialization (Xilinx serializer.sv
    // removed). Note: this file is hdl-util's (MIT), modified as stated
    // in this header -- NOT our code; the tango_* and usb/ files are ours.
    output logic [9:0] tmds_ch0,
    output logic [9:0] tmds_ch1,
    output logic [9:0] tmds_ch2,

    // All outputs below this line stay inside the FPGA
    // They are used (by you) to pick the color each pixel should have
    // i.e. always_ff @(posedge pixel_clk) rgb <= {8'd0, 8'(cx), 8'(cy)};
    output logic [11:0] cx = 12'd0,
    output logic [10:0] cy = 11'd0,

    // The screen is at the upper left corner of the frame.
    // 0,0 = 0,0 in video
    // the frame includes extra space for sending auxiliary data
    output logic [11:0] frame_width,
    // ⭐ Local modification: extra scanlines to append to V_TOTAL this frame (0 = upstream).
    // ⚠ NO DEFAULT VALUE: Verilator rejects defaults on module inputs
    // (%Error-UNSUPPORTED). Every instantiation of THIS copy must connect
    // it; tie it to 4'd0 for upstream behaviour.
    input  logic  [3:0] frame_height_adj,
`ifdef TANGO_VSYNC_ADJUST
    // ⭐ Tango (TANGO_VSYNC_ADJUST): the totals of the NEXT frame, taken at the
    // raster's wrap (LOCAL MODIFICATION 3 below); frame_width / frame_height then output the CURRENT frame's.
    input  logic [11:0] next_frame_width,
    input  logic [10:0] next_frame_height,
`endif
    output logic [10:0] frame_height,
    output logic [11:0] screen_width,
    output logic [10:0] screen_height
);

localparam int NUM_CHANNELS = 3;
localparam int VIDEO_ID_CODE = VIC;  // 720p VARIANT: 1280x720p60 (VIC 4) unless the parameter says otherwise
localparam int BIT_WIDTH = 12;
localparam int BIT_HEIGHT = 11;
logic hsync;
logic vsync;

// CEA-861-D VIC 16: 1920x1080p60, positive sync both axes.
// ⭐⭐ TANGO LOCAL MODIFICATION 2: the
// totals come from the FRAME_WIDTH / FRAME_HEIGHT parameters (defaults 1650 /
// 750, i.e. exactly as before) and the AVI VIC from VIC (default 4), so a build
// can run the unchanged 74.25 MHz pixel clock at a core's native frame rate,
// 74.25e6 / (FRAME_WIDTH x FRAME_HEIGHT). Only the back porches change: every
// sync / island / guard position
// below is relative to screen_width / screen_height or to frame_width, as
// upstream. Valid FRAME_WIDTH >= 1626 (the video preamble at frame_width-10 must
// follow the last data island, which ends at 1615) and FRAME_HEIGHT >= 731 (the
// VSYNC lines 724-729); the board top checks the range.
`ifndef TANGO_VSYNC_ADJUST
assign frame_width = 12'(FRAME_WIDTH);      // 720p VARIANT
`endif
// ⭐⭐ TANGO LOCAL MODIFICATION 1 -- the FIRST change to this vendored file
// beside the thinning in the header (the second is the parameters above).
// Upstream is hdl-util/hdmi by Sameer Puri (MIT / Apache-2.0); the modifications
// are listed in hdmi/README.md and NOTICE.
//
// WHY. frame_height was a hard constant, so V_TOTAL could not be varied at
// runtime -- and varying it is the only lever available to lock the display to
// the console. The two clocks are 36.6 ppm apart (display 60.000000 Hz, console
// 59.997806 Hz), which walks the phase a full frame every 456 s; with a SINGLE
// -BUFFERED frame buffer the reader then catches the writer and tears.
// Nudging either clock instead is impossible -- 148.5 MHz
// is exactly 27 x 11/2 and 36.6 ppm is far finer than the PLL divider grid.
//
// Tie frame_height_adj to 4'd0 and this behaves EXACTLY as upstream.
`ifndef TANGO_VSYNC_ADJUST
assign frame_height = 11'(FRAME_HEIGHT) + {7'd0, frame_height_adj};   // 720p VARIANT
`endif
assign screen_width = 12'd1280;
assign screen_height = 11'd720;
localparam bit [11:0] HSYNC_PULSE_START = 12'd110;   // VIC 4
localparam bit [11:0] HSYNC_PULSE_SIZE = 12'd40;
localparam bit [10:0] VSYNC_PULSE_START = 11'd5;
localparam bit [10:0] VSYNC_PULSE_SIZE = 11'd5;

always_comb begin
    hsync <= cx >= screen_width + HSYNC_PULSE_START && cx < screen_width + HSYNC_PULSE_START + HSYNC_PULSE_SIZE;
    // vsync pulses should begin and end at the start of hsync, so special
    // handling is required for the lines on which vsync starts and ends
    if (cy == screen_height + VSYNC_PULSE_START - 1)
        vsync <= cx >= screen_width + HSYNC_PULSE_START;
    else if (cy == screen_height + VSYNC_PULSE_START + VSYNC_PULSE_SIZE - 1)
        vsync <= cx < screen_width + HSYNC_PULSE_START;
    else
        vsync <= cy >= screen_height + VSYNC_PULSE_START && cy < screen_height + VSYNC_PULSE_START + VSYNC_PULSE_SIZE;
end

localparam real VIDEO_RATE = 74.25E6;   // 720p VARIANT

// cy-dependent guard/preamble term, registered once per line (step-10 cut):
// cy only changes at the cx wrap, and the VG/VP windows sit at END of line
// (cx >= frame_width-10), so a value settled by cx==1 is always correct.
// This takes the 11-bit cy comparators out of the failing end-of-line paths.
logic cy_video_ok = 1'b0;
always_ff @(posedge clk_pixel)
    cy_video_ok <= (cy == frame_height - 1 || cy < screen_height - 1 /* no VG/VP at end of last line */);

// Wrap-around pixel position counters indicating the pixel to be generated by the user in THIS clock and sent out in the NEXT clock.
// Raster wrap via a registered strobe (step-13): the 12-bit cx==2199
// comparator fed cx's own RESET, cy's CE, and external line ticks -- one
// hot cone, three fanout classes, -0.44 ns. wrap_q makes it drive ONE
// flop; waveforms identical (cx==2198 registered == wrap during 2199).
logic wrap_q = 1'b0;
always_ff @(posedge clk_pixel)
begin
    if (reset)
    begin
        cx <= 12'd0;
        cy <= 11'd0;
        wrap_q <= 1'b0;
    end
    else
    begin
        wrap_q <= cx == frame_width - 2'd2;
        cx <= wrap_q ? 12'd0 : cx + 1'b1;
        if (wrap_q)
            cy <= cy == frame_height-1'b1 ? 11'd0 : cy + 1'b1;
    end
end

`ifdef TANGO_VSYNC_ADJUST
// ⭐⭐ TANGO LOCAL MODIFICATION 3 (TANGO_VSYNC_ADJUST): runtime totals. The frame's
// totals are registers, loaded from next_frame_width / next_frame_height in reset and at the wrap from the
// frame's last pixel to (0, 0) -- the clock cx and cy wrap -- so every frame runs the totals it started with and
// frame_width / frame_height (outputs) are always those of the frame cx / cy are in. Every comparison above and
// below reads frame_width / frame_height as before. Without TANGO_VSYNC_ADJUST: the parameters (modification 2).
logic [11:0] fw_r = 12'(FRAME_WIDTH);
logic [10:0] fh_r = 11'(FRAME_HEIGHT);
always_ff @(posedge clk_pixel)
    if (reset || (wrap_q && cy == frame_height - 1'b1))
    begin
        fw_r <= next_frame_width;
        fh_r <= next_frame_height;
    end
assign frame_width = fw_r;
assign frame_height = fh_r + {7'd0, frame_height_adj};
`endif

// See Section 5.2
logic video_data_period = 0;
always_ff @(posedge clk_pixel)
begin
    if (reset)
        video_data_period <= 0;
    else
        video_data_period <= cx < screen_width && cy < screen_height;
end

logic [2:0] mode = 3'd1;
logic [23:0] video_data = 24'd0;
logic [5:0] control_data = 6'd0;
logic [11:0] data_island_data = 12'd0;

logic video_guard = 1;
logic video_preamble = 0;
always_ff @(posedge clk_pixel)
begin
    if (reset)
    begin
        video_guard <= 1;
        video_preamble <= 0;
    end
    else
    begin
        video_guard <= cx >= frame_width - 2 && cx < frame_width && cy_video_ok;
        video_preamble <= cx >= frame_width - 10 && cx < frame_width - 2 && cy_video_ok;
    end
end

// See Section 5.2.3.1
// VIC 4 h-blank = 370 clocks; (370 - 2 - 8 - 4 - 2 - 2 - 8 - 4) / 32 = 10
// packets fit alongside, under the 18-packet ceiling.
localparam bit [4:0] NUM_PACKETS_ALONGSIDE = 5'd10;  // 720p VARIANT (370 px blanking, hdl-util default) // 1080p blanking is 280px: 16+32*7+2=242 fits before the video preamble at frame_width-10 (720p used 10)

// Island strobes computed against cx_next and REGISTERED (step-12 cut):
// identical waveforms, but the wide comparators come off the CE fanout into
// packet_picker (the -1.8 ns cx->CE family; strobe pre-registration is the
// house pattern for every raster-derived control at 148.5 MHz).
// NO adder in the strobe cones: comparing RAW cx against (bound-1) is
// identical to comparing cx+1 against the bound (cx is monotonic; the
// island windows are nowhere near the frame wrap). The cx_next adder was
// ~3 LUT levels inside every one of these register cones (build 9's
// residual -0.4 family).
logic data_island_period_instantaneous = 1'b0;
logic packet_enable = 1'b0;
always_ff @(posedge clk_pixel)
begin
    data_island_period_instantaneous <= cx >= screen_width + 13 && cx < screen_width + 13 + NUM_PACKETS_ALONGSIDE * 32;
    // pick fires ONE cycle before the old point (constant 18->19 on the
    // +1 frame, i.e. 20 on raw cx): the picker's mux-register stage
    // realigns header/sub arrival with emission exactly as before.
    packet_enable <= (cx >= screen_width + 12 && cx < screen_width + 12 + NUM_PACKETS_ALONGSIDE * 32) && 5'(cx + screen_width + 20) == 5'd0;
end

logic data_island_guard = 0;
logic data_island_preamble = 0;
logic data_island_period = 0;
always_ff @(posedge clk_pixel)
begin
    if (reset)
    begin
        data_island_guard <= 0;
        data_island_preamble <= 0;
        data_island_period <= 0;
    end
    else
    begin
        data_island_guard <= (cx >= screen_width + 12 && cx < screen_width + 14) /* leading guard */ ||
            (cx >= screen_width + 14 + NUM_PACKETS_ALONGSIDE * 32 && cx < screen_width + 14 + NUM_PACKETS_ALONGSIDE * 32 + 2) /* trailing guard */;
        data_island_preamble <= cx >= screen_width + 4 && cx < screen_width + 12;
        data_island_period <= data_island_period_instantaneous;
    end
end

// See Section 5.2.3.4
logic [23:0] header;
logic [55:0] sub [3:0];
logic video_field_end = 1'b0;
always_ff @(posedge clk_pixel)
    video_field_end <= cx == screen_width - 2'd2 && cy == screen_height - 1'b1;
logic [4:0] packet_pixel_counter;
packet_picker #(
    .VIDEO_ID_CODE(VIDEO_ID_CODE),
    .VIDEO_RATE(VIDEO_RATE),
    .IT_CONTENT(IT_CONTENT),
    .AUDIO_RATE(AUDIO_RATE),
    .AUDIO_BIT_WIDTH(AUDIO_BIT_WIDTH),
    .ACR_N(ACR_N),
    .ACR_CTS(ACR_CTS)
) packet_picker (.clk_pixel(clk_pixel), .clk_audio(clk_audio), .reset(reset), .video_field_end(video_field_end), .packet_enable(packet_enable), .packet_pixel_counter(packet_pixel_counter), .audio_sample_word(audio_sample_word),
`ifdef TANGO_HDMI_RETUNE
    .acr_cts_rt(acr_cts_rt),
`endif
    .header(header), .sub(sub));
logic [8:0] packet_data;
packet_assembler packet_assembler (.clk_pixel(clk_pixel), .reset(reset), .data_island_period(data_island_period), .header(header), .sub(sub), .packet_data(packet_data), .counter(packet_pixel_counter));


always_ff @(posedge clk_pixel)
begin
    if (reset)
    begin
        mode <= 3'd2;
        video_data <= 24'd0;
        control_data = 6'd0;
        data_island_data <= 12'd0;
    end
    else
    begin
        mode <= data_island_guard ? 3'd4 : data_island_period ? 3'd3 : video_guard ? 3'd2 : video_data_period ? 3'd1 : 3'd0;
        video_data <= rgb;
        control_data <= {{1'b0, data_island_preamble}, {1'b0, video_preamble || data_island_preamble}, {vsync, hsync}}; // ctrl3, ctrl2, ctrl1, ctrl0, vsync, hsync
        data_island_data[11:4] <= packet_data[8:1];
        data_island_data[3] <= cx != 0;
        data_island_data[2] <= packet_data[0];
        data_island_data[1:0] <= {vsync, hsync};
    end
end

// All logic below relates to the production and output of the 10-bit TMDS code.
logic [9:0] tmds_internal [NUM_CHANNELS-1:0] /* verilator public_flat */ ;
genvar i;
generate
    // TMDS code production.
    for (i = 0; i < NUM_CHANNELS; i++)
    begin: tmds_gen
        tmds_channel #(.CN(i)) tmds_channel (.clk_pixel(clk_pixel), .video_data(video_data[i*8+7:i*8]), .data_island_data(data_island_data[i*4+3:i*4]), .control_data(control_data[i*2+1:i*2]), .mode(mode), .tmds(tmds_internal[i]));
    end
endgenerate

assign tmds_ch0 = tmds_internal[0];
assign tmds_ch1 = tmds_internal[1];
assign tmds_ch2 = tmds_internal[2];

endmodule
