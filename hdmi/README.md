# hdmi/ -- the HDMI encoder (hdl-util/hdmi, modified)

Upstream: [hdl-util/hdmi](https://github.com/hdl-util/hdmi) by Sameer Puri, offered under MIT or Apache-2.0 and used
here under the MIT licence (`LICENSE-MIT` in this folder).  Every file keeps its "By Sameer Puri" header, and each
modified file says in its header what was changed.

The copy is thinned to the one mode Tango sends, CEA VIC 4: 1280x720p60, 74.25 MHz pixel clock, frame 1650 x 750.

- `hdmi.sv`: the VIC 4 constants inlined; the other video modes, the VIDEO_RATE refresh ladder, the DVI_OUTPUT
  branch, START_X/START_Y and the Source Product Description (SPD) InfoFrame removed.  The raw 10-bit TMDS words are
  outputs: `board/console138k/tango_tmds_ser.v` serialises them (upstream's Xilinx `serializer.sv` is not used).  A
  `frame_height_adj` input adds lines to V_TOTAL for one frame (tie it to 0 for upstream behaviour).  The end-of-line
  guard/preamble terms, the raster wrap and the data-island strobes are registered for timing (the same waveforms).
- `packet_picker.sv`: the 256-entry header/sub mux arrays replaced by an explicit mux over the packets sent (NULL,
  audio clock regeneration, audio sample, AVI InfoFrame, audio InfoFrame), its output registered; the optional SPD
  InfoFrame removed.
- `audio_clock_regeneration_packet.sv`: the fixed ideal CTS is sent, since the audio and pixel clocks come from one
  crystal (HDMI 1.4b 7.2.1), instead of a measured value with +/-1 jitter; constant header bytes instead of X.
- `audio_sample_packet.sv`: the constant-zero valid_bit / user_data_bit inputs folded in; 0 instead of X.
- `tmds_channel.sv`: pipelined (two feed-forward register stages); every symbol comes one pixel later, which the
  sink does not see.
- `packet_assembler.sv`: the parity / ECC path registered, with exactly the original output (a timing cut).
- `source_product_description_info_frame.sv`: removed.

On the wire the behaviour is upstream's, except that no SPD packet is sent.

## Local modifications for the Tango framework's video options

`hdmi.sv` numbers its local modifications in its comments ("TANGO LOCAL MODIFICATION n"); with every parameter at its
default and no option defined, the module is exactly the thinned copy above.

1. `frame_height_adj` (above).
2. **Raster totals and VIC as parameters.** `frame_width` / `frame_height` come from the parameters FRAME_WIDTH /
   FRAME_HEIGHT (defaults 1650 / 750) and the AVI InfoFrame's VIC from VIC (default 4). A build sets them from
   tango_cfg.vh's TANGO_HDMI_HTOTAL / TANGO_HDMI_VTOTAL / TANGO_HDMI_VIC (the board top passes them) to run the
   unchanged 74.25 MHz pixel clock at a core's native frame rate. Only the back porches change; the active picture,
   the sync pulses, the data islands and the guard / preamble positions are where they were.
3. **Runtime totals** (`TANGO_VSYNC_ADJUST`): the frame's totals are registers, loaded from `next_frame_width` /
   `next_frame_height` in reset and at the raster's wrap, so every frame runs the totals it started with;
   `frame_width` / `frame_height` output the current frame's.
4. **ACR N / CTS as integer parameters** (ACR_N / ACR_CTS, passed through `packet_picker.sv` to
   `audio_clock_regeneration_packet.sv`, "TANG PATCH 2" there): when the pixel clock is not 74.25 MHz (tango_cfg.vh
   TANGO_HDMI_PLL_<f>) the board top gives the Audio Clock Regeneration packet the N / CTS pair for EXACTLY 48 kHz at
   that clock, CTS = f_pix x N / (128 x 48000) (HDMI 1.4b 7.2): 74.5875 MHz -> N 12288 / CTS 149175 (N 6144 would need
   CTS 74587.5), 75.6 MHz -> 6144 / 75600. The board top's clk_audio accumulator moves with them (exactly 96 kHz
   toggles on clk_pix). 0 (the default) = N and CTS computed from AUDIO_RATE / VIDEO_RATE as before (6144 / 74250).
5. **The CTS at run time** (`TANGO_HDMI_RETUNE`, input `acr_cts_rt`, "TANG PATCH 3"): when the pixel clock is retuned
   at core start, the CTS follows the PLL (N stays 12288); it changes only while the module is held in reset.

`sim/hdmi_acr_sim.sv` is a simulation-only copy of `audio_clock_regeneration_packet.sv` with its real-valued constants
worked out (Verilator cannot fold them); it takes the same N / CTS pair.
