// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// tango_core_ports.vh -- the port list every Tango core's top module (module tango_core) has,
// as MiSTer cores `include "sys/emu_ports.vh"`.  Use:  module tango_core ( `include "tango_core_ports.vh" );
// Meanings: README "The core interface" (and menu-tango docs/MAKING-A-CORE.md "The core-facing interface").
`include "tango_cfg.vh"
`include "tango_video_opts.vh"      // the Video options' CONF_STR entries: `TANGO_VIDEO_CONF / `TANGO_VIDEO_CONF_P(...)
// ---- clocks and reset
input  wire         CLK_50M,        // board oscillator (for the core's own PLLs)
input  wire         CLK_PIX,        // 74.25 MHz, the 720p pixel clock (video below is in it)
input  wire         RESET,          // high while the pixel PLL is not locked; asynchronous to CLK_SYS
output wire         CLK_SYS,        // the core's system clock: every MiSTer-style signal below is in it
// ---- MiSTer hps_io equivalents (CLK_SYS)
input  wire [31:0]  joystick_0,     // USB-A2 pad: 0 R, 1 L, 2 D, 3 U, 4 A, 5 B, 6 X, 7 Y, 8 L, 9 R, 10 Select, 11 Start
input  wire [31:0]  joystick_1,     // USB-A1 pad
input  wire [127:0] status,         // the option word (CONF_STR O/T/R bits); status[0] = soft reset
output wire [15:0]  status_menumask,// H/D prefixes of the CONF_STR test these bits
input  wire         ioctl_download, // a file is being sent (F entry)
input  wire [15:0]  ioctl_index,    // which: F digit or menu position + extension index * 64
input  wire         ioctl_wr,       // one pulse per byte
input  wire [26:0]  ioctl_addr,     // byte offset of ioctl_dout
input  wire [7:0]   ioctl_dout,
output wire         ioctl_wait,     // hold high to pause the download (nothing is lost)
// ---- CONF_STR (combinational; tango_conf_rom)
input  wire [11:0]  conf_addr,
output wire [7:0]   conf_char,
`ifdef TANGO_VIDEO_FB
// ---- video (CLK_VIDEO): MiSTer's emu video ports (Template_MiSTer sys/emu ports, Genesis_MiSTer Genesis.sv
//      31-60); the framework buffers the frame and scales it to 720p (sys/tango_video.v)
//      RULE: CE_PIXEL must keep pulsing through horizontal and vertical blanking; VGA_HS / VGA_VS / VGA_DE are
//      sampled only on CE (ascal.vhd 1162-1166, tango_video.v 123-124; MiSTer's sys/video_mixer.sv, Genesis_MiSTer
//      adc0c42, changes them only at CE_PIXEL clocks: 188, 206-215).  A CE_PIXEL limited to the picture never shows
//      the framework a VS rise or a DE fall, so no frame is ever taken.  Every
//      sim that builds sys/tango_video.v checks it per input frame and stops with "TANGO CONTRACT: CE_PIXEL ..."
//      (tango_video.v, "CE_PIXEL contract").
output wire         CLK_VIDEO,      // the core's video clock
output wire         CE_PIXEL,       // one pixel per CLK_VIDEO clock where high; keeps pulsing in the blanking (RULE)
output wire [7:0]   VGA_R,
output wire [7:0]   VGA_G,
output wire [7:0]   VGA_B,
output wire         VGA_HS,         // positive (as MiSTer's video_mixer output); not used by the scaler
output wire         VGA_VS,         // positive; the rising edge ends a frame
output wire         VGA_DE,         // = ~(HBlank | VBlank): the active picture
output wire [1:0]   VGA_SL,         // scanlines: 0 none, 1 25 %, 2 50 %, 3 75 % (MiSTer VGA_SL)
output wire [12:0]  VIDEO_ARX,      // aspect ratio, or (bit 12 set) the picture size in pixels (MiSTer)
output wire [12:0]  VIDEO_ARY,
input  wire [11:0]  HDMI_WIDTH,     // 1280: for a core that runs MiSTer's own video_freak
input  wire [11:0]  HDMI_HEIGHT,    // 720
output wire         HDMI_FREEZE,    // keep showing the last frame (MiSTer HDMI_FREEZE)
output wire [2:0]   VIDEO_SCALE,    // Tango: video_freak's SCALE (0 Normal, 1 V-Integer, 2/3/4 HV-Integer), done in sys/
//                                     (arcade builds, TANGO_VIDEO_ROTATE + TANGO_VIDEO_OPTS: the menu's Scale offers only
//                                     Normal / V-Integer, MiSTer arcade parity -- MiSTer arcade cores have no Scale entry,
//                                     only MiSTer.ini vscale_mode 0/1, Main_MiSTer video.cpp 3073-3108; sys/tango_video_opts.vh)
output wire [11:0]  VIDEO_CROP_SIZE,// Tango: video_freak's CROP_SIZE (0 = none)
output wire [4:0]   VIDEO_CROP_OFF, // Tango: video_freak's CROP_OFF (-16..15)
`ifdef TANGO_VIDEO_IDX
output wire [`TANGO_VIDEO_IDXW-1:0] VGA_IDX,   // index mode: the colour index (VGA_R/G/B unused)
output wire         VIDEO_PAL_WR,   // index mode: palette write, CLK_VIDEO
output wire [`TANGO_VIDEO_IDXW-1:0] VIDEO_PAL_A,
output wire [23:0]  VIDEO_PAL_D,
`endif
`elsif TANGO_VIDEO_ASCAL
// ---- video (CLK_VIDEO): same MiSTer emu ports as TANGO_VIDEO_FB;
//      ascal.vhd + SDRAM replaces the BSRAM frame buffer; index mode is not supported.
//      RULE (as TANGO_VIDEO_FB above): CE_PIXEL must keep pulsing through horizontal and vertical blanking;
//      VGA_HS / VGA_VS / VGA_DE are sampled only on CE.  ascal starts an input frame on the VS rise and ends a line on
//      the DE fall it sees at i_ce clocks (ascal.vhd 1162-1166, 1180, 1220, 1427); the sims stop with
//      "TANGO CONTRACT: CE_PIXEL ..." when a core breaks it (tango_video.v, "CE_PIXEL contract").
//      VIDEO OPTIONS (opt in: build option TANGO_VIDEO_OPTS, sys/tango_video_opts.vh): the framework owns Aspect ratio
//      (status[127:126]), Scale (status[125:124]; arcade TANGO_VIDEO_ROTATE builds: Normal / V-Integer only, MiSTer
//      arcade parity, sys/tango_video_opts.vh) and Scanlines (status[123:122]), entries from `TANGO_VIDEO_CONF /
//      `TANGO_VIDEO_CONF_P("P1") in the CONF_STR; status[121:118] stays reserved (a future Filter entry, then
//      Rotation, TANGO_VO_B_ROTATE, in TANGO_VIDEO_ROTATE builds). The core then outputs its NATIVE shape on VIDEO_ARX/ARY
//      (what the menu calls "Original", e.g. NES 64:49), ties VIDEO_SCALE and VGA_SL to 0 (sys does not use them; the
//      sims stop with "TANGO CONTRACT: VIDEO_OPTS ..." otherwise) and reads none of status[127:118].  Without the
//      option the core drives all four itself, as before.
output wire         CLK_VIDEO,
output wire         CE_PIXEL,       // keeps pulsing in the blanking (RULE above)
output wire [7:0]   VGA_R,
output wire [7:0]   VGA_G,
output wire [7:0]   VGA_B,
output wire         VGA_HS,
output wire         VGA_VS,
output wire         VGA_DE,
output wire [1:0]   VGA_SL,         // TANGO_VIDEO_OPTS: 0 (sys's Scanlines)
output wire [12:0]  VIDEO_ARX,      // TANGO_VIDEO_OPTS: the native aspect ("Original")
output wire [12:0]  VIDEO_ARY,
input  wire [11:0]  HDMI_WIDTH,
input  wire [11:0]  HDMI_HEIGHT,
output wire         HDMI_FREEZE,
output wire [2:0]   VIDEO_SCALE,    // TANGO_VIDEO_OPTS: 0 (sys's Scale)
output wire [11:0]  VIDEO_CROP_SIZE,
output wire [4:0]   VIDEO_CROP_OFF,
`ifdef TANGO_VIDEO_F1
output wire         VGA_F1,         // TANGO_VIDEO_F1: the interlace field (MiSTer's VGA_F1, CLK_VIDEO): ascal's i_fl
`endif
`ifdef TANGO_ASCAL_EXT
// ---- ascal's Avalon MM master, served by the core's own SDRAM controller (TANGO_ASCAL_EXT, tango_cfg.vh;
//      README "A core that owns the SDRAM").  All in ASCAL_AVL_CLK, which the core drives: its memory clock.  64-bit beats, word 0
//      (the lowest address) in bits [15:0]; bursts of avl_burstcount beats (ascal: 32 = 256 B, 256-B aligned, never
//      across a 1 KB row); one command at a time; a write brings beat 0 with the command; read data in order, one beat
//      per clock with ascal_avl_readdatavalid, possibly after the next command was taken.  ascal_avl_waitrequest high =
//      the command / beat on the port is not taken this clock.  The addresses are in the ascal window only (tango_cfg.vh
//      RAMBASE / RAMSIZE: banks 2/3).  sys/tango_ascal_words.v serves the port as 16-bit word accesses.
output wire         ASCAL_AVL_CLK,
input  wire [28:0]  ascal_avl_address,      // 64-bit word address: byte address {ascal_avl_address, 3'b000}
input  wire         ascal_avl_write,
input  wire [63:0]  ascal_avl_writedata,
input  wire         ascal_avl_read,
input  wire [7:0]   ascal_avl_burstcount,
input  wire [7:0]   ascal_avl_byteenable,
output wire [63:0]  ascal_avl_readdata,
output wire         ascal_avl_readdatavalid,
output wire         ascal_avl_waitrequest,
`endif
`else
// ---- video (CLK_PIX): the core colours the 720p beam, no scaler
input  wire [11:0]  VID_CX,         // beam x 0..HTOTAL-1 (visible 0..1279; HTOTAL 1650 unless tango_cfg.vh TANGO_HDMI_HTOTAL)
input  wire [10:0]  VID_CY,         // beam y 0..VTOTAL-1 (visible 0..719;  VTOTAL 750 unless tango_cfg.vh TANGO_HDMI_VTOTAL)
output wire [23:0]  VID_RGB,        // {R,G,B} for (VID_CX, VID_CY) of TWO clocks earlier
`endif
// ---- audio (CLK_PIX), sampled at 48 kHz by the framework
output wire [15:0]  AUDIO_L,
output wire [15:0]  AUDIO_R,
`ifdef TANGO_AUDIO_FILTER
 `define TANGO_CORE_AUDIO_MIX
`endif
`ifdef TANGO_VIDEO_PROC
 `define TANGO_CORE_AUDIO_MIX
`endif
`ifdef TANGO_CORE_AUDIO_MIX
// TANGO_AUDIO_FILTER / TANGO_VIDEO_PROC: MiSTer's AUDIO_MIX (emu port; audio_out.sv 329-334): the stereo mix the
// framework applies, 0 none, 1 25 %, 2 50 %, 3 mono (quasi-static; any clock)
output wire [1:0]   AUDIO_MIX,
`endif
// ---- the OSD (CLK_SYS): 1 while it is shown (MiSTer emu_ports.vh 153, sys_top.v 1859)
input  wire         OSD_STATUS,
// ---- keyboard (CLK_SYS): MiSTer hps_io ps2_key (hps_io.sv 103): [7:0] set-2 code, [8] extended, [9] pressed,
//      [10] toggles per key event
input  wire [10:0]  ps2_key
`include "tango_cfg.vh"
`ifdef TANGO_SD
,
// ---- block device (TANGO_SD builds only; MiSTer hps_io sd_* / img_*)
`include "tango_sd_ports.vh"
`endif
`ifdef TANGO_SDRAM
,
// ---- SDRAM (TANGO_SDRAM builds only; MiSTer emu SDRAM_* ports)
`include "tango_sdram_ports.vh"
`endif
`ifdef TANGO_IOCTL_UPLOAD
,
// ---- upload (TANGO_IOCTL_UPLOAD builds only; MiSTer hps_io.sv 142-150's upload direction, the reverse of
//      ioctl_download above -- reads a blob back from the core instead of sending one.  Shares ioctl_addr/
//      ioctl_index with ioctl_download (hps_io.sv's own FIO_FILE_TX/FIO_FILE_TX_DAT state machine, lines
//      659-703: the two directions are distinguished by ioctl_download/ioctl_upload, never both asserted at
//      once, and the core sees the same address bus either way). ioctl_upload_req/ioctl_upload_index let the
//      core itself ask to be saved (an open policy question: whether the
//      framework ever honours this, or only ever drives an upload on its own schedule).
// DIRECTIONS: this is the CORE's port list, so every direction is the opposite of hps_io.sv's OWN port list
// (the FRAMEWORK's perspective), exactly as the existing ioctl_download/ioctl_wr/etc. above are `input` here
// because hps_io.sv declares them `output`.
input  wire         ioctl_upload,       // an upload is active (mirrors ioctl_download); framework -> core
output wire         ioctl_upload_req,   // the core requests a save; policy on whether this is honoured is firmware-side
output wire [7:0]   ioctl_upload_index, // which index the core wants to upload, when requesting one
output wire [7:0]   ioctl_din,          // the core's current byte at ioctl_addr (mirrors ioctl_dout, opposite direction)
input  wire         ioctl_rd            // "sample ioctl_din now, then advance ioctl_addr" (mirrors ioctl_wr)
`endif
`ifdef TANGO_NES_SDRAM
,
// ---- NES SDRAM ports A+B (TANGO_NES_SDRAM+TANGO_VIDEO_ASCAL): two real-time 8-bit ports into the framework's
//      SDRAM (tango_ab_slot -> tango_sdram), banks 0/1.  A request: raise oe (read) or we (write) with a stable
//      address (and din) and hold them until the slot's clkref clock; one request per rising edge of oe|we.
//      clkref: high for ONE CLK_SYS clock per slot, at a clock by which the slot's requests are raised (the NES:
//      once per PPU dot, the clock after the dot boundary, nes_div == 0).  At most one request per port per slot.
//      doutA/doutB: the read byte, on the port 9-17 clk_sdram after tango_sdram takes the slot (2-3 clk_sdram after
//      the clkref clock) at the NES's request rate, up to 27 clk_sdram when both ports ask in every slot (tango_sdram
//      header), held until that port's next read; asynchronous to CLK_SYS (the core SDC bounds the path with
//      set_max_delay).
output wire [21:0]  addrA,      // port A (PPU/CHR): address
output wire         oeA,        // port A: read
output wire         weA,        // port A: write
output wire [7:0]   dinA,       // port A: write data
input  wire [7:0]   doutA,      // port A: read data
output wire [21:0]  addrB,      // port B (CPU/ROM): address
output wire         oeB,
output wire         weB,
output wire [7:0]   dinB,
input  wire [7:0]   doutB,
output wire         clkref,     // slot enable (CLK_SYS), see above
input  wire         sdram_busy  // tango_sdram busy (power-up initialisation)
`endif
