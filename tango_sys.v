// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_sys -- the Tango framework's fabric (board-independent): the BL616 link, the OSD
// overlay, the USB-A pad readers, and the core-facing interface (tango_io), around the
// core (module tango_core, ports: tango_core_ports.vh).  The board top (sys/board/<board>/)
// adds PLLs, the HDMI encoder and the pins.  The same module is the sim gate's DUT
// (menu-tango fpga/sim/tb_menu.v): a core's sim compiles its own tango_core with it.
//
//   clk_pix: hdmi.sv beam (cx, cy) -> tango_beam_lead -> the core's beam -> the core's RGB
//            -> tango_menu_overlay (OSD) -> rgb for hdmi.sv's (cx, cy) of the previous clock.
//            TANGO_VIDEO_FB builds: the core outputs MiSTer's emu video ports (its own clock and timing)
//            and tango_video (frame buffer + scaler) gives the overlay its background instead.
//            tango_link: BL616 SPI slave (bits in the SCK domain, bytes processed in clk_pix).
//   clk48:   two tango_usb_port FS hosts (USB-A1 M15/M16, USB-A2 H13/G13); DS4 reports cross
//            into clk_pix in tango_ds4_joy.
//   clk_sys: the core's clock; tango_io carries joysticks, status, menumask and the download.
//==============================================================================
`include "tango_cfg.vh"
`ifdef TANGO_VIDEO_ROTATE
`include "tango_video_opts.vh"   // `TANGO_VO_B_ROTATE (status[119:118]): the CDC'd rotate_mode source below
`endif
`default_nettype none
module tango_sys #(
    parameter integer USB_DEBOUNCE = 960_000
) (
    input  wire        clk50,           // board oscillator, passed to the core
    // video (clk_pix)
    input  wire        clk_pix,
    input  wire        rst_pix,
    input  wire [11:0] cx,              // hdmi.sv beam
    input  wire [10:0] cy,
    output wire [23:0] rgb,             // for (cx, cy) of the previous clock
    output wire [15:0] audio_l, audio_r,// from the core (clk_pix)
    // USB (clk48)
    input  wire        clk48,
    input  wire        lock48,
    inout  wire        usb1_dp, usb1_dn,     // USB-A1 (M15 / M16)
    inout  wire        usb2_dp, usb2_dn,     // USB-A2 (H13 / G13)
    // BL616 link pins (asynchronous)
    input  wire        spi_csn,
    input  wire        spi_sck,
    input  wire        spi_mosi,
    output wire        spi_miso,        // value; the top tristates it while bl616_jtagsel
    input  wire        spi_miso_pad,    // the MISO pad read back (debug)
    output wire        spi_irq_n,
    input  wire        bl616_jtagsel,   // BL616 GPIO28 -> V14: JTAG request
    input  wire        jtagsel_n_q,     // the JTAGSEL_N level as driven (top), for the debug rows
    // debug / status (clk_pix unless noted)
    output wire        osd_shown,
    output wire        osd_bank,
    output wire        osd_swap_pending,
    output wire [15:0] joy,
    output wire [7:0]  joy_flags,
    output wire [15:0] link_xfers,
    output wire [7:0]  usb1_bad_crc,    // clk48
    output wire [7:0]  usb2_bad_crc,    // clk48
    // on-screen debug rows (tango_link_dbg_view), clk_pix; rows 0-10 from tango_link, row 11:
    // {BL616 JTAG request, JTAGSEL_N as driven, overlay shown, shown bank, swap pending,
    //  USB-A2 pad live, USB-A1 pad live, rst_pix}, rows 12-17 from tango_link
    output wire [143:0] dbg_rows
`ifdef TANGO_VSYNC_ADJUST
    ,
    // TANGO_VSYNC_ADJUST: the raster's totals at runtime -- in: hdmi.sv's frame_width /
    // frame_height, the CURRENT frame's (in step with cx / cy); out: the next frame's (tango_vsync below; hdmi.sv and
    // every consumer here take them at their own frame boundary)
    input  wire [11:0] vt_cur_w,
    input  wire [10:0] vt_cur_h,
    output wire [11:0] vt_nxt_w,
    output wire [10:0] vt_nxt_h
`ifdef TANGO_HDMI_RETUNE
    ,
    // TANGO_HDMI_RETUNE: tango_pll_retune (board top, 50 MHz) <-> tango_vsync: the PLL's
    // setting now (quasi-static: changes only while rst_pix is held), and SYS 0x70's request (a toggle + the setting, never
    // reset in clk_pix)
    input  wire        pll_native,
    output wire        rt_tog,
    output wire        rt_native
`endif
`endif
`ifdef TANGO_VIDEO_ASCAL
    ,
    output wire        dbg_uart_tx   // UART TX from ascal debug sender → route to U15
`endif
`ifdef TANGO_SDRAM
    ,
    // the SDRAM pins, straight through to the core (TANGO_SDRAM)
    `include "tango_sdram_ports.vh"
`endif
`ifdef TANGO_VIDEO_DDR3
    ,
    // TANGO_VIDEO_DDR3: ascal's frames in the DDR3.  The board top has the Gowin DDR3 IP, its PLL
    // (pll_ddr3 at PLL_L[0]) and the pins; tango_ddr3 (here) drives the IP's user interface, in the IP's clk_out
    input  wire         clk_ddr,           // the IP's clk_out (100 MHz)
    input  wire         ddr_rst,           // the IP's ddr_rst
    input  wire         ddr_calib,         // the IP's init_calib_complete
    input  wire         ddr_cmd_ready,
    output wire         ddr_cmd_en,
    output wire [2:0]   ddr_cmd,
    output wire [28:0]  ddr_addr,
    input  wire         ddr_wr_data_rdy,
    output wire         ddr_wr_data_en,
    output wire [255:0] ddr_wr_data,
    output wire [31:0]  ddr_wr_data_mask,
    input  wire         ddr_rd_data_valid,
    input  wire [255:0] ddr_rd_data
`endif
`ifdef TANGO_FW_SDRAM
    ,
    // SDRAM pins owned by tango_sdram (framework) in TANGO_VIDEO_ASCAL builds (TANGO_ASCAL_EXT: the core's, TANGO_SDRAM).
    // SDRAM_CLK is NOT here: the board top drives it directly from Gowin_PLL_SDRAM clkout1.
    input  wire        clk_sdram,    // 96 MHz fabric (from board PLL clkout0)
    inout  wire [15:0] SDRAM_DQ,
    output wire [12:0] SDRAM_A,
    output wire [1:0]  SDRAM_BA,
    output wire        SDRAM_DQML,
    output wire        SDRAM_DQMH,
    // SDRAM_CKE: no ball on the Console 138K module; tango_sdram drives it to 1 internally
    output wire        SDRAM_nCS,
    output wire        SDRAM_nRAS,
    output wire        SDRAM_nCAS,
    output wire        SDRAM_nWE
`endif
);
    localparam integer OV_LAT = 6;      // tango_menu_overlay pipeline (its own LAT)
    // the HDMI raster's totals (tango_cfg.vh TANGO_HDMI_HTOTAL / _VTOTAL; default 1650 x 750): the board top's hdmi.sv runs
    // them, and every consumer of its beam here takes the same pair -- both beam leads, the overlay, tango_video's ascal
    // alignment and ascal's output timing
    localparam integer HT = `TANGO_HDMI_HTOTAL, VT = `TANGO_HDMI_VTOTAL;
    localparam [11:0]  HT12 = `TANGO_HDMI_HTOTAL, VT12 = `TANGO_HDMI_VTOTAL;
    localparam integer LAT = OV_LAT + 1;// + the bg register below
    localparam integer ATW = 48;        // atlas word = 2 x 24 glyph rows (tango_ov_geom.vh OV_SH)
`ifdef TANGO_LINK_TAP
    // MiSTer's video / audio processing (README "Video and audio processing"; tango_link tap -> tango_vproc_regs, below)
    wire        tap_cmd_stb, tap_stb, tap_end;
    wire [7:0]  tap_cmd, tap_byte, vp_flags;
    wire [15:0] tap_n;
`ifdef TANGO_VIDEO_PROC
    wire [3:0]  vp_has = 4'b1111;           // FLTNUM, SET_GAMMA, SET_AFILTER, SHADOWMASK
    wire [1:0]  vp_sl;
    wire        vp_poly_wr, vp_gam_en, vp_gam_wr, vp_mask_wr;
    wire [`TANGO_VPROC_FRAC+3:0] vp_poly_a;  // ascal's poly_a: {set, phase[FRAC-1:0], tap}
    wire [7:0]  vp_gam_d;
    wire [9:0]  vp_poly_dw, vp_gam_a;
    wire [1:0]  vp_flt_mode;
    wire [15:0] vp_mask_cmd;
`else
    wire [3:0]  vp_has = 4'b0100;           // SET_AFILTER only
    wire [1:0]  vp_sl = 2'd0;
`endif
    wire        af_rst;
    wire [31:0] af_rate;
    wire [39:0] af_cx;
    wire [7:0]  af_cx0, af_cx1, af_cx2;
    wire [23:0] af_cy0, af_cy1, af_cy2;
    wire [1:0]  af_boost, core_audio_mix;     // the core volume's boost; the core's AUDIO_MIX
    wire [4:0]  af_vol;                       // the global volume (0x26)
    wire [15:0] core_audio_l, core_audio_r;  // the core's samples, into tango_audio_filter
`endif
`ifdef TANGO_VIDEO_OPTS
`ifndef TANGO_VIDEO_ASCAL
    // the framework's Video options (tango_video_opts.vh) are decoded in the ascal video path only: refuse the build
    tango_video_opts_needs_TANGO_VIDEO_ASCAL u_error ();
`endif
`endif
`ifdef TANGO_VIDEO_PROC
`ifndef TANGO_VIDEO_ASCAL
    // MiSTer's video processing (README "Video and audio processing") lives in the ascal video path: refuse the build
    tango_video_proc_needs_TANGO_VIDEO_ASCAL u_error2 ();
`endif
`endif
`ifdef TANGO_ASCAL_EXT
`ifndef TANGO_VIDEO_ASCAL
    // ascal's Avalon port to the core (tango_cfg.vh) needs ascal: refuse the build
    tango_ascal_ext_needs_TANGO_VIDEO_ASCAL u_error3 ();
`endif
`ifndef TANGO_SDRAM
    // ... and a core that owns the SDRAM (its controller serves the port): refuse the build
    tango_ascal_ext_needs_TANGO_SDRAM u_error4 ();
`endif
`ifdef TANGO_NES_SDRAM
    // ports A/B are tango_sdram's, which TANGO_ASCAL_EXT does not build: refuse the build
    tango_ascal_ext_excludes_TANGO_NES_SDRAM u_error5 ();
`endif
`endif
`ifdef TANGO_VIDEO_F1
`ifndef TANGO_VIDEO_ASCAL
    // the field goes to ascal (i_fl): refuse the build without it
    tango_video_f1_needs_TANGO_VIDEO_ASCAL u_error6 ();
`endif
`endif
`ifdef TANGO_VIDEO_DDR3
`ifndef TANGO_VIDEO_ASCAL
    // ascal's frames in the DDR3: refuse the build without ascal
    tango_video_ddr3_needs_TANGO_VIDEO_ASCAL u_error7 ();
`endif
`ifdef TANGO_ASCAL_EXT
    // ascal's Avalon port goes to the DDR3, not to the core's controller: refuse both at once
    tango_video_ddr3_excludes_TANGO_ASCAL_EXT u_error8 ();
`endif
`ifdef TANGO_SIM
    // tango_video's sim scaler (ascal_wrap_sim) has the 64-bit SDRAM Avalon port only
    tango_video_ddr3_excludes_TANGO_SIM u_error10 ();
`endif
`endif
`ifdef TANGO_VSYNC_ADJUST
`ifndef TANGO_VIDEO_ASCAL
    // the core's frame period is measured on its emu video ports and ascal follows the raster: refuse the build without
    tango_vsync_adjust_needs_TANGO_VIDEO_ASCAL u_error11 ();
`endif
`ifndef TANGO_LINK_TAP
    // SYS 0x70 comes through the link tap (TANGO_VIDEO_PROC or TANGO_AUDIO_FILTER): refuse the build without
    tango_vsync_adjust_needs_TANGO_VIDEO_PROC_or_TANGO_AUDIO_FILTER u_error12 ();
`endif
    wire [7:0] vs_status;                       // SYS 0x70's reply (tango_vsync, below), answered by tango_link
`endif

    // ---------------------------------------------------------------- video
    // The core colours the beam ccx/ccy with 2 clocks of latency (tango_core_ports.vh).  Its colour is
    // registered once more here (bg_q) before the overlay: the core's VID_RGB may be combinational after its
    // last register (e.g. a2600_fb.v's scanline mux), and the overlay's first stage (translucent blend + add,
    // tango_menu_overlay s1) must start from a flop -- an Atari 2600 core build missed clk_pix by 0.509 ns on
    // exactly that path.  The overlay takes its background one clock after its own beam, so:
    //   core beam    = hdmi.sv beam + LAT          (tango_beam_lead D = LAT = OV_LAT + 1)
    //   core colour  = VID_RGB at clock t is for ccx(t-2) = hdmi(t+LAT-2); bg_q at t is for hdmi(t+LAT-3)
    //   overlay beam = core beam delayed 2 = hdmi.sv beam + OV_LAT - 1 (as before: tango_ov_video / the
    //                  first menu core), so bg_q at t is for the overlay's beam of t-1, its contract
    wire [11:0] ccx;  wire [10:0] ccy;
`ifdef TANGO_VSYNC_ADJUST
    tango_beam_lead_rt #(.D(LAT)) u_lead (.clk(clk_pix), .cx(cx), .cy(cy), .fw(vt_cur_w), .fh(vt_cur_h), .lcx(ccx), .lcy(ccy));
`else
    tango_beam_lead #(.D(LAT), .FW(HT), .FH(VT)) u_lead (.clk(clk_pix), .cx(cx), .cy(cy), .lcx(ccx), .lcy(ccy));
`endif
    reg  [11:0] ccx_q = 12'd0, lcx = 12'd0;  reg [10:0] ccy_q = 11'd0, lcy = 11'd0;
    always @(posedge clk_pix) begin ccx_q <= ccx; ccy_q <= ccy; lcx <= ccx_q; lcy <= ccy_q; end
    wire [23:0] bg;
    reg  [23:0] bg_q = 24'd0;
    always @(posedge clk_pix) bg_q <= bg;
`ifdef TANGO_VIDEO_FB
    // TANGO_VIDEO_FB: the core outputs MiSTer's emu video ports; tango_video buffers the frame and scales it.
    // Its rgb (a register, for its beam of VLAT clocks earlier) is the bg that bg_q above registers once more
    // at the module boundary, so the overlay's first stage never sees the frame buffer's logic.  bg_q at t
    // must be for hdmi(t+LAT-3) (above); v_rgb at t-1 is for the video beam of t-1-VLAT, so the video block's
    // beam leads hdmi.sv by LAT - 3 + 1 + VLAT = LAT + VLAT - 2 (16 with LAT 7).
    localparam integer VLAT = 11;       // tango_video's pipeline (E1..E11)
    wire [11:0] vcx;  wire [10:0] vcy;
    tango_beam_lead #(.D(LAT + VLAT - 2), .FW(HT), .FH(VT)) u_vlead (.clk(clk_pix), .cx(cx), .cy(cy), .lcx(vcx), .lcy(vcy));
    wire [23:0] v_rgb;
    assign bg = v_rgb;
    wire        clk_video, ce_pixel, vga_hs, vga_vs, vga_de, hdmi_freeze;
    wire [7:0]  vga_r, vga_g, vga_b;
    wire [1:0]  vga_sl;
    wire [12:0] video_arx, video_ary;
    wire [2:0]  video_scale;
    wire [11:0] video_crop_size;
    wire [4:0]  video_crop_off;
`ifdef TANGO_VIDEO_IDX
    localparam integer VIDXW = `TANGO_VIDEO_IDXW;
    wire [VIDXW-1:0] vga_idx, vpal_a;  wire vpal_wr;  wire [23:0] vpal_d;
`else
    localparam integer VIDXW = 0;
    wire vga_idx = 1'b0, vpal_a = 1'b0, vpal_wr = 1'b0;  wire [23:0] vpal_d = 24'd0;
`endif
    tango_video #(.MAXW(`TANGO_VIDEO_MAXW), .MAXH(`TANGO_VIDEO_MAXH), .BPC(`TANGO_VIDEO_BPC),
                  .IDXW(VIDXW), .NBUF(`TANGO_VIDEO_NBUF), .FW(HT), .FH(VT)) u_vid (
        .clk_video(clk_video), .ce_pixel(ce_pixel), .vga_r(vga_r), .vga_g(vga_g), .vga_b(vga_b),
        .vga_hs(vga_hs), .vga_vs(vga_vs), .vga_de(vga_de), .vga_idx(vga_idx), .vga_sl(vga_sl),
        .video_arx(video_arx), .video_ary(video_ary), .video_scale(video_scale),
        .video_crop_size(video_crop_size), .video_crop_off(video_crop_off), .hdmi_freeze(hdmi_freeze),
        .pal_wr(vpal_wr), .pal_a(vpal_a), .pal_d(vpal_d),
        .clk_pix(clk_pix), .rst_pix(rst_pix), .cx(vcx), .cy(vcy), .rgb(v_rgb),
        .show_seq(), .show_valid(), .show_take(), .dbg_lost(), .dbg_frames());
`endif
`ifdef TANGO_VIDEO_ASCAL
    // TANGO_VIDEO_ASCAL: ascal direct-input + SDRAM. tango_sdram.v owns the SDRAM pins.
    // VLAT = 2: ascal output (1 register in tango_video) + bg_q (1 register here).
    // Beam lead = LAT + VLAT - 2.
    // TANGO_VIDEO_PROC: tango_video's shadow mask adds 7 clocks (tango_video.v VLAT = 9).
`ifdef TANGO_VIDEO_PROC
    localparam integer VLAT = 9;
`else
    localparam integer VLAT = 2;
`endif
    wire [11:0] vcx;  wire [10:0] vcy;
`ifdef TANGO_VSYNC_ADJUST
    tango_beam_lead_rt #(.D(LAT + VLAT - 2)) u_vlead (
        .clk(clk_pix), .cx(cx), .cy(cy), .fw(vt_cur_w), .fh(vt_cur_h), .lcx(vcx), .lcy(vcy));
`else
    tango_beam_lead #(.D(LAT + VLAT - 2), .FW(HT), .FH(VT)) u_vlead (
        .clk(clk_pix), .cx(cx), .cy(cy), .lcx(vcx), .lcy(vcy));
`endif
    wire [23:0] v_rgb;
    assign bg = v_rgb;
    wire        clk_video, ce_pixel, vga_hs, vga_vs, vga_de, hdmi_freeze;
    wire [7:0]  vga_r, vga_g, vga_b;
    wire [1:0]  vga_sl;
    wire [12:0] video_arx, video_ary;
    wire [2:0]  video_scale;
    wire [11:0] video_crop_size;
    wire [4:0]  video_crop_off;
    // Avalon bus between tango_video and tango_sdram
`ifdef TANGO_VIDEO_DDR3
    wire [27:0]  vid_avl_addr;  // 28-bit 16-byte word address from ascal (N_AW=28: the DDR3 scaler port)
    wire         vid_avl_write, vid_avl_read;
    wire [127:0] vid_avl_wdata, vid_avl_rdata;
    wire [7:0]   vid_avl_burst;
    wire [15:0]  vid_avl_be;
`else
    wire [28:0] vid_avl_addr;  // 29-bit word address from ascal (N_AW=29)
    wire        vid_avl_write, vid_avl_read;
    wire [63:0] vid_avl_wdata, vid_avl_rdata;
    wire [7:0]  vid_avl_burst, vid_avl_be;
`endif
    wire        vid_avl_rvalid, vid_avl_waitreq;
    wire        sdram_busy;
    wire [31:0] dbg_arc, dbg_arv, dbg_awc;  wire dbg_aws;  wire [63:0] dbg_ald;   // ascal instruments
`ifdef TANGO_VIDEO_F1
    wire        vga_f1;                     // the core's interlace field (TANGO_VIDEO_F1), CLK_VIDEO
`endif
`ifdef TANGO_ASCAL_EXT
    wire        ascal_avl_clk;              // ascal's Avalon side: the core's memory clock (TANGO_ASCAL_EXT)
`endif
`ifdef TANGO_NES_SDRAM
    // the core's ports A/B (CLK_SYS) and tango_ab_slot's held slot requests into tango_sdram (clk_sdram)
    wire [21:0] c_addrA, c_addrB; wire c_oeA, c_weA, c_oeB, c_weB, c_clkref;
    wire [7:0]  c_dinA, c_dinB, c_doutA, c_doutB;
    wire [21:0] h_addrA, h_addrB; wire h_oeA, h_weA, h_oeB, h_weB, h_slot;
    wire [7:0]  h_dinA, h_dinB;
    localparam integer AB_SLOTS = 1;
`else
    localparam integer AB_SLOTS = 0;
`endif
`ifdef TANGO_ASCAL_EXT
    // TANGO_ASCAL_EXT: no tango_sdram; the core's controller serves ascal's Avalon port (the core ports below)
    assign sdram_busy = 1'b0;
`elsif TANGO_VIDEO_DDR3
    // TANGO_VIDEO_DDR3: tango_ddr3's scaler port serves ascal, its core port tied off unless TANGO_DDRAM
    // (the rotation client, below).  Reset: until the IP has calibrated, synchronous to clk_ddr.  No
    // tango_sdram, unless TANGO_NES_SDRAM (below, after this chain: ports A/B only).
`ifndef TANGO_FW_SDRAM
    assign sdram_busy = 1'b0;
`endif
    reg [2:0] ddr_rs = 3'b111;
    always @(posedge clk_ddr) ddr_rs <= {ddr_rs[1:0], ddr_rst | ~ddr_calib};
    tango_ddr3 u_ddr3 (
        .clk(clk_ddr), .rst(ddr_rs[2]),
        .avl_address(vid_avl_addr), .avl_burstcount(vid_avl_burst), .avl_write(vid_avl_write),
        .avl_writedata(vid_avl_wdata), .avl_byteenable(vid_avl_be), .avl_read(vid_avl_read),
        .avl_waitrequest(vid_avl_waitreq), .avl_readdata(vid_avl_rdata), .avl_readdatavalid(vid_avl_rvalid),
`ifdef TANGO_VIDEO_ROTATE
        // TANGO_DDRAM (above, build_core.sh): C0 is tango_video_rotate_ddr3's own client, in clk_video (its
        // own DDRAM_CLK -- no new CDC here, the crossing into clk_ddr is entirely
        // inside tango_ddr3_cport.v). crst tied low, the same "rely on each register's own =0
        // initial value, not a hard reset" discipline this framework uses for the rotation client's rst_video
        // (below).
        .DDRAM_CLK(clk_video), .ddram_rst(1'b0), .DDRAM_BUSY(rot_ddram_busy_w), .DDRAM_BURSTCNT(rot_ddram_burstcnt_w),
        .DDRAM_ADDR(rot_ddram_addr_w), .DDRAM_DOUT(rot_ddram_dout_w), .DDRAM_DOUT_READY(rot_ddram_doutrdy_w),
        .DDRAM_RD(rot_ddram_rd_w), .DDRAM_DIN(rot_ddram_din_w), .DDRAM_BE(rot_ddram_be_w), .DDRAM_WE(rot_ddram_we_w),
`else
        .DDRAM_CLK(clk_ddr), .ddram_rst(1'b1), .DDRAM_BUSY(), .DDRAM_BURSTCNT(8'd0), .DDRAM_ADDR(29'd0),
        .DDRAM_DOUT(), .DDRAM_DOUT_READY(), .DDRAM_RD(1'b0), .DDRAM_DIN(64'd0), .DDRAM_BE(8'd0), .DDRAM_WE(1'b0),
`endif
        .cmd_ready(ddr_cmd_ready), .cmd_en(ddr_cmd_en), .cmd(ddr_cmd), .addr(ddr_addr),
        .wr_data_rdy(ddr_wr_data_rdy), .wr_data_en(ddr_wr_data_en), .wr_data(ddr_wr_data),
        .wr_data_mask(ddr_wr_data_mask), .rd_data_valid(ddr_rd_data_valid), .rd_data(ddr_rd_data),
        .obs_v_wait_max(), .obs_v_win_drops(), .obs_core_inflight_max(), .obs_fences(), .obs_arb_v_wait_max());
`else
    // tango_sdram (framework owns SDRAM)
    tango_sdram #(.AB_SLOTS(AB_SLOTS)) u_sdram (
        .clk    (clk_sdram),
        .resetn (~rst_pix),
        .busy   (sdram_busy),
        .SDRAM_DQ  (SDRAM_DQ),
        .SDRAM_A   (SDRAM_A),
        .SDRAM_BA  (SDRAM_BA),
        .SDRAM_DQML(SDRAM_DQML),
        .SDRAM_DQMH(SDRAM_DQMH),
        .SDRAM_CKE (),          // no ball; 1'b1 internally in tango_sdram
        .SDRAM_nCS (SDRAM_nCS),
        .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS),
        .SDRAM_nWE (SDRAM_nWE),
`ifdef TANGO_NES_SDRAM
        // ports A/B: the core's requests as tango_ab_slot holds them, one slot at a time
        .addrA(h_addrA), .oeA(h_oeA), .weA(h_weA), .dinA(h_dinA), .doutA(c_doutA),
        .addrB(h_addrB), .oeB(h_oeB), .weB(h_weB), .dinB(h_dinB), .doutB(c_doutB),
        .clkref(h_slot),
`else
        // ports A/B unused (tied off)
        .addrA(22'd0), .oeA(1'b0), .weA(1'b0), .dinA(8'd0),
        .addrB(22'd0), .oeB(1'b0), .weB(1'b0), .dinB(8'd0),
        .clkref(1'b0),
`endif
        // port C: ascal Avalon MM
        .avl_address    (vid_avl_addr),
        .avl_write      (vid_avl_write),
        .avl_writedata  (vid_avl_wdata),
        .avl_read       (vid_avl_read),
        .avl_burstcount (vid_avl_burst),
        .avl_byteenable (vid_avl_be),
        .avl_readdata   (vid_avl_rdata),
        .avl_readdatavalid(vid_avl_rvalid),
        .avl_waitrequest(vid_avl_waitreq));
`endif
`ifdef TANGO_VIDEO_DDR3
`ifdef TANGO_FW_SDRAM
    // TANGO_VIDEO_DDR3 + TANGO_NES_SDRAM: tango_sdram serves the NES's ports A/B as in the SDRAM builds; its port C
    // (ascal's in those builds) is idle -- ascal's frames are in the DDR3 (tango_ddr3, above)
    tango_sdram #(.AB_SLOTS(AB_SLOTS)) u_sdram (
        .clk    (clk_sdram),
        .resetn (~rst_pix),
        .busy   (sdram_busy),
        .SDRAM_DQ  (SDRAM_DQ),
        .SDRAM_A   (SDRAM_A),
        .SDRAM_BA  (SDRAM_BA),
        .SDRAM_DQML(SDRAM_DQML),
        .SDRAM_DQMH(SDRAM_DQMH),
        .SDRAM_CKE (),          // no ball; 1'b1 internally in tango_sdram
        .SDRAM_nCS (SDRAM_nCS),
        .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS),
        .SDRAM_nWE (SDRAM_nWE),
        .addrA(h_addrA), .oeA(h_oeA), .weA(h_weA), .dinA(h_dinA), .doutA(c_doutA),
        .addrB(h_addrB), .oeB(h_oeB), .weB(h_weB), .dinB(h_dinB), .doutB(c_doutB),
        .clkref(h_slot),
        .avl_address(29'd0), .avl_write(1'b0), .avl_writedata(64'd0), .avl_read(1'b0), .avl_burstcount(8'd0),
        .avl_byteenable(8'd0), .avl_readdata(), .avl_readdatavalid(), .avl_waitrequest());
`endif
`endif
`ifdef TANGO_VIDEO_ROTATE
    // tango_video_rotate_ddr3 (README "Rotation"): rotation through the DDR3, not an on-chip BSRAM corner-turn
    // (with an on-chip corner-turn, an arcade core's build used 338 of the device's 340 block RAMs). This client WRITES a
    // rotated copy of the core's own raster into DDR3's C0 window (tango_ddr3_cport.v, requires
    // `TANGO_DDRAM` alongside `TANGO_VIDEO_DDR3` -- `build_core.sh` must define both together for this flag,
    // not a tango_sys.v concern); it never touches vga_r/g/b/hs/vs/de, which still go into tango_video/ascal
    // completely unchanged below -- ascal's own capture keeps running too (MiSTer keeps feeding ascal's input
    // while FB_EN is set, so Tango does the same; no CE_PIXEL-contract question, no re-timed raster to generate
    // here).
    // rotate_mode: status[`TANGO_VO_B_ROTATE] via tango_cdc_word, clk_pix -> clk_video -- not a Verilog
    // constant, which would let synthesis constant-fold the block away.
    wire [1:0]  rotate_mode_vrot;
    tango_cdc_word #(.W(2)) u_vrot_cdc (
        .clk_a(clk_pix), .din(status_l[`TANGO_VO_B_ROTATE]),
        .clk_b(clk_video), .dout(rotate_mode_vrot));
`ifdef TANGO_VIDEO_DDR3
    wire        rot_fb_ena_w, rot_fb_vbl_w;
    wire [11:0] rot_fb_hsize_w, rot_fb_vsize_w;
    wire [5:0]  rot_fb_format_w;
    wire [31:0] rot_fb_base_w;
    wire [13:0] rot_fb_stride_w;
    wire        rot_ddram_busy_w, rot_ddram_rd_w, rot_ddram_we_w, rot_ddram_doutrdy_w;
    wire [7:0]  rot_ddram_burstcnt_w, rot_ddram_be_w;
    wire [28:0] rot_ddram_addr_w;
    wire [63:0] rot_ddram_dout_w, rot_ddram_din_w;
    tango_video_rotate_ddr3 #(
        .MAXW(`TANGO_VIDEO_ASCAL_MAXW), .MAXH(`TANGO_VIDEO_ASCAL_MAXH), .NBUF(4)
    ) u_vrot_ddr3 (
        .clk_video(clk_video), .rst_video(1'b0), .rotate_mode(rotate_mode_vrot),
        .ce_pixel_in(ce_pixel),
        .vga_r_in(vga_r), .vga_g_in(vga_g), .vga_b_in(vga_b),
        .vga_hs_in(vga_hs), .vga_vs_in(vga_vs), .vga_de_in(vga_de),
        .DDRAM_CLK(), .DDRAM_BUSY(rot_ddram_busy_w), .DDRAM_BURSTCNT(rot_ddram_burstcnt_w),
        .DDRAM_ADDR(rot_ddram_addr_w), .DDRAM_DOUT(rot_ddram_dout_w), .DDRAM_DOUT_READY(rot_ddram_doutrdy_w),
        .DDRAM_RD(rot_ddram_rd_w), .DDRAM_DIN(rot_ddram_din_w), .DDRAM_BE(rot_ddram_be_w),
        .DDRAM_WE(rot_ddram_we_w),
        .clk_pix(clk_pix),
        .o_fb_ena(rot_fb_ena_w), .o_fb_hsize(rot_fb_hsize_w), .o_fb_vsize(rot_fb_vsize_w),
        .o_fb_format(rot_fb_format_w), .o_fb_base(rot_fb_base_w), .o_fb_stride(rot_fb_stride_w));
`endif
`endif
    // tango_video (ascal version)
    tango_video #(
        .MAXW (`TANGO_VIDEO_ASCAL_MAXW),
        .MAXH (`TANGO_VIDEO_ASCAL_MAXH),
        .ASCAL_RAMBASE(`TANGO_VIDEO_ASCAL_RAMBASE),
        .ASCAL_RAMSIZE(`TANGO_VIDEO_ASCAL_RAMSIZE),
        .FW (HT), .FH (VT)
    ) u_vid (
        .clk_video      (clk_video),
        .ce_pixel       (ce_pixel),
        .vga_r          (vga_r),   .vga_g(vga_g),   .vga_b(vga_b),
        .vga_hs         (vga_hs),  .vga_vs(vga_vs), .vga_de(vga_de),
`ifdef TANGO_VIDEO_ROTATE
`ifdef TANGO_VIDEO_DDR3
        .rot_fb_ena(rot_fb_ena_w), .rot_fb_hsize(rot_fb_hsize_w), .rot_fb_vsize(rot_fb_vsize_w),
        .rot_fb_format(rot_fb_format_w), .rot_fb_base(rot_fb_base_w), .rot_fb_stride(rot_fb_stride_w),
        .rot_fb_vbl(rot_fb_vbl_w),   // ascal's own o_vbl; not yet consumed by u_vrot_ddr3 (an open
                                     // item -- the client currently swaps on its own input-side
                                     // vs_rise + fence completion, not on this signal; carried out here so
                                     // it exists to wire in once that's revisited, not left dangling)
`endif
`endif
        .vga_idx        (1'b0),    .vga_sl(vga_sl),
        .video_arx      (video_arx), .video_ary(video_ary),
        .video_scale    (video_scale),
        .video_crop_size(video_crop_size),
        .video_crop_off (video_crop_off),
        .hdmi_freeze    (hdmi_freeze),
`ifdef TANGO_VIDEO_F1
        .vga_f1         (vga_f1),       // the interlace field (TANGO_VIDEO_F1)
`endif
`ifdef TANGO_VIDEO_OPTS
        .status         (status_l),     // the framework's Video options (tango_video_opts.vh): tango_link's word, clk_pix
`endif
`ifdef TANGO_VIDEO_PROC
        .poly_wr(vp_poly_wr), .poly_a(vp_poly_a), .poly_dw(vp_poly_dw), .flt_mode(vp_flt_mode),
        .gam_en(vp_gam_en), .gam_wr(vp_gam_wr), .gam_a(vp_gam_a), .gam_d(vp_gam_d),
        .mask_wr(vp_mask_wr), .mask_cmd(vp_mask_cmd), .sl_level(vp_sl),
`endif
        .pal_wr(1'b0), .pal_a(1'b0), .pal_d(24'd0),
        .clk_pix(clk_pix), .rst_pix(rst_pix),
        .cx(vcx), .cy(vcy), .rgb(v_rgb),
`ifdef TANGO_VSYNC_ADJUST
        .vt_nxt_w(vt_nxt_w), .vt_nxt_h(vt_nxt_h),
`endif
        .show_seq(), .show_valid(), .show_take(), .dbg_lost(), .dbg_frames(),
        // Avalon to tango_sdram port C
        .avl_address    (vid_avl_addr),
        .avl_write      (vid_avl_write),
        .avl_writedata  (vid_avl_wdata),
        .avl_read       (vid_avl_read),
        .avl_burstcount (vid_avl_burst),
        .avl_byteenable (vid_avl_be),
        .avl_readdata   (vid_avl_rdata),
        .avl_readdatavalid(vid_avl_rvalid),
        .avl_waitrequest(vid_avl_waitreq),
`ifdef TANGO_ASCAL_EXT
        .avl_clk        (ascal_avl_clk),
`elsif TANGO_VIDEO_DDR3
        .avl_clk        (clk_ddr),
`else
        .avl_clk        (clk_sdram),
`endif
        .dbg_arc(dbg_arc), .dbg_arv(dbg_arv), .dbg_awc(dbg_awc), .dbg_aws(dbg_aws), .dbg_ald(dbg_ald),
        // ascal mode bits (ascal.vhd header): [2:0] 000 = nearest, [3] 1 = triple buffering, [4] unused
        .video_mode     (5'b11000),
        // 720p output timing (hdmi.sv values; the totals are the raster's, HT x VT above)
        .o_htotal (HT12),     .o_hsstart(12'd1390), .o_hsend  (12'd1430),
        .o_hdisp  (12'd1280), .o_vtotal (VT12),     .o_vsstart(12'd725),
        .o_vsend  (12'd730),  .o_vdisp  (12'd720));

    // ---------------------------------------------------------------- ascal instruments -> UART
    // Every 0.5 s clk50 asks clk_sdram for a snapshot (toggle, 3 flops); clk_sdram copies the counters
    // into held registers and toggles back; clk50 then loads the report from the held copy, which does
    // not change until the next request, and sends it at 115200 baud on dbg_uart_tx:
    //   "RC=<read cmds> RV=<read beats> WC=<write beats> WS=<stuck> LD=<last read beat>\r\n" (hex)
    reg         snap_req = 1'b0, snap_ack = 1'b0;
    reg  [2:0]  snap_req_s = 3'b000, snap_ack_s = 3'b000;
    reg  [31:0] h_arc = 32'd0, h_arv = 32'd0, h_awc = 32'd0;
    reg         h_aws = 1'b0;
    reg  [63:0] h_ald = 64'd0;
`ifdef TANGO_ASCAL_EXT
    always @(posedge ascal_avl_clk) begin
`elsif TANGO_VIDEO_DDR3
    always @(posedge clk_ddr) begin
`else
    always @(posedge clk_sdram) begin
`endif
        snap_req_s <= {snap_req_s[1:0], snap_req};
        if (snap_req_s[2] != snap_req_s[1]) begin
            h_arc <= dbg_arc;  h_arv <= dbg_arv;  h_awc <= dbg_awc;  h_aws <= dbg_aws;  h_ald <= dbg_ald;
            snap_ack <= ~snap_ack;
        end
    end
    function [7:0] hexc(input [3:0] n);
        hexc = (n < 4'd10) ? 8'h30 + {4'd0, n} : 8'h37 + {4'd0, n};
    endfunction
    function [63:0] hex8(input [31:0] v);
        integer i;
        begin
            for (i = 0; i < 8; i = i + 1) hex8[8*i +: 8] = hexc(v[4*i +: 4]);
        end
    endfunction
    wire [62*8-1:0] rpt = {"RC=", hex8(h_arc), " RV=", hex8(h_arv), " WC=", hex8(h_awc),
                           " WS=", hexc({3'd0, h_aws}), " LD=", hex8(h_ald[63:32]), hex8(h_ald[31:0]),
                           8'h0D, 8'h0A};
    localparam integer BAUD_DIV = 434;            // 50 MHz / 115200
    reg  [24:0]  u_tick = 25'd0;
    reg  [8:0]   u_baud = 9'd0;
    reg  [9:0]   u_bits = 10'd0;                  // bits still to send
    reg  [619:0] u_sh   = {620{1'b1}};            // 62 x {stop, char, start}, first character in [9:0]
    integer k;
    always @(posedge clk50) begin
        snap_ack_s <= {snap_ack_s[1:0], snap_ack};
        u_tick <= (u_tick == 25'd24_999_999) ? 25'd0 : u_tick + 25'd1;
        if (u_tick == 25'd24_999_999 && u_bits == 10'd0) snap_req <= ~snap_req;
        if (snap_ack_s[2] != snap_ack_s[1]) begin
            for (k = 0; k < 62; k = k + 1) u_sh[10*k +: 10] <= {1'b1, rpt[8*(61-k) +: 8], 1'b0};
            u_bits <= 10'd620;  u_baud <= 9'd0;
        end else if (u_bits != 10'd0) begin
            if (u_baud == BAUD_DIV - 1) begin
                u_baud <= 9'd0;  u_sh <= {1'b1, u_sh[619:1]};  u_bits <= u_bits - 10'd1;
            end else u_baud <= u_baud + 9'd1;
        end
    end
    assign dbg_uart_tx = u_sh[0];
`endif

    wire            at_we, gl_we, rt_we, pal_we, geo_we, en_we, en_wd, swap_we, swap_wd, stage_busy;
    wire [9:0]      at_wa;
    wire [10:0]     gl_wa;
    wire [6:0]      rt_wa;
    wire [4:0]      pal_wa;
    wire [3:0]      geo_wa;
    wire [ATW-1:0]  at_wd;
    wire [31:0]     gl_wd, rt_wd;
    wire [23:0]     pal_wd;
    wire [7:0]      geo_wd;
    tango_menu_overlay #(.ATW(ATW), .FW(HT), .FH(VT)) u_ov (
        .clk(clk_pix), .rst(rst_pix), .cx(lcx), .cy(lcy), .bg(bg_q), .rgb(rgb),
`ifdef TANGO_VSYNC_ADJUST
        .fh_rt(vt_cur_h),
`endif
        .at_we(at_we), .at_wa(at_wa), .at_wd(at_wd),
        .gl_we(gl_we), .gl_wa(gl_wa), .gl_wd(gl_wd),
        .rt_we(rt_we), .rt_wa(rt_wa), .rt_wd(rt_wd),
        .pal_we(pal_we), .pal_wa(pal_wa), .pal_wd(pal_wd),
        .geo_we(geo_we), .geo_wa(geo_wa), .geo_wd(geo_wd),
        .en_we(en_we), .en_wd(en_wd), .swap_we(swap_we), .swap_wd(swap_wd),
        .stage_busy(stage_busy),
        .shown_bank(osd_bank), .swap_pending(osd_swap_pending), .shown(osd_shown));

    // ---------------------------------------------------------------- link
    // bl616_jtagsel (asynchronous pin) -> clk_pix
    reg [1:0] jsel_s = 2'b11;
    always @(posedge clk_pix) jsel_s <= {jsel_s[0], bl616_jtagsel};
    // clk_pix timing: tango_link's reset from a registered copy of rst_pix placed beside it
    // (a Template DDR3 build failed clk_pix by 0.85 ns on the board top's
    // rstp_sr -> tango_link -> gl_wd, a 4.6 ns route around the DDR3 IP). In every build, not only
    // `ifdef TANGO_VIDEO_DDR3`: the same rstp_sr_7 -> rst_link -> lrst -> {ul_size_o|gl_wd|aw|pal_wd|at_wd}
    // paths exist in every build, DDR3 or not.
    // Costs one extra clk_pix cycle of reset-release delay on u_link only (a reset signal, not a data
    // path); every build's netlist for u_link's reset net changes, so each build family needs its own
    // strict-gate re-verification.
    (* syn_preserve = 1 *) reg rst_link = 1'b1;
    always @(posedge clk_pix) rst_link <= rst_pix;
    tango_link #(.ATW(ATW), .ATLAS_ROWS(ATW / 2)) u_link (
        .clk(clk_pix), .rst(rst_link),
        .spi_csn(spi_csn), .spi_sck(spi_sck), .spi_mosi(spi_mosi),
        .spi_miso(spi_miso), .spi_miso_pad(spi_miso_pad), .irq_n(spi_irq_n), .hold(jsel_s[1]),
        .joy(joy), .joy_flags(joy_flags),
        .raw1(r1), .raw2(r2), .vid1(vid1), .pid1(pid1), .vid2(vid2), .pid2(pid2), .pad_con({p2, p1}),
        .pad_kbd({k2 == 2'd1, k1 == 2'd1}), .kbd1(kb1), .kbd2(kb2), .ps2_key(ps2_key_pix),
        .padmap1(pm1), .padmap2(pm2),
        .pad_live({l2, l1}), .desc_rdy({rdy2, rdy1}), .ver1(gver1), .ver2(gver2), .rep16_1(grep1), .rep16_2(grep2),
        .replen1(glen1), .replen2(glen2), .len_cfg1(lcfg1), .len_cfg2(lcfg2), .len_rpt1(lrpt1), .len_rpt2(lrpt2),
        .len_dev1(ldev1), .len_dev2(ldev2), .desc_trunc1(dtr1), .desc_trunc2(dtr2),
        .ds_ra(ds_ra), .ds1_rd(ds_rd1), .ds2_rd(ds_rd2), .eng_busy({ebusy2, ebusy1}),
        .tw_we1(tw_we1), .tw_we2(tw_we2), .tw_addr(tw_addr), .tw_din(tw_din), .cfg1(pcfg1), .cfg2(pcfg2), .tmode(ptmode),
        .ext_en(), .pad_swap(pad_swap),
        .at_we(at_we), .at_wa(at_wa), .at_wd(at_wd),
        .gl_we(gl_we), .gl_wa(gl_wa), .gl_wd(gl_wd),
        .rt_we(rt_we), .rt_wa(rt_wa), .rt_wd(rt_wd),
        .pal_we(pal_we), .pal_wa(pal_wa), .pal_wd(pal_wd),
        .geo_we(geo_we), .geo_wa(geo_wa), .geo_wd(geo_wd),
        .en_we(en_we), .en_wd(en_wd), .swap_we(swap_we), .swap_wd(swap_wd),
        .stage_busy(stage_busy), .shown_bank(osd_bank), .swap_pending(osd_swap_pending),
        .conf_addr(conf_addr), .conf_char(conf_char), .status(status_l), .status_tog(status_tog),
        .menumask(menumask_l), .dl_we(dl_we), .dl_wd(dl_wd), .dl_free(dl_free), .dl_dropped(),
        .dbg_jtagsel(), .dbg_xfers(link_xfers), .dbg_view(link_dbg),
        .sd_req(sd_req), .sd_in_we(sd_in_we), .sd_in_wa(sd_in_wa), .sd_in_wd(sd_in_wd), .sd_in_done_tog(sd_in_done),
        .sd_out_ra(sd_out_ra), .sd_out_rd(sd_out_rd), .sd_out_done_tog(sd_out_done),
        .mnt_tog(mnt_tog), .mnt_drive(mnt_drive), .mnt_ro(mnt_ro), .mnt_size(mnt_size)
`ifdef TANGO_IOCTL_UPLOAD
        , .ul_tog(ul_tog), .ul_idx(ul_idx), .ul_size_o(ul_size_o), .ul_ra(ul_ra), .ul_rd(ul_rd), .ul_ready(ul_ready)
`endif
`ifdef TANGO_LINK_TAP
        , .tap_cmd_stb(tap_cmd_stb), .tap_stb(tap_stb), .tap_cmd(tap_cmd), .tap_n(tap_n), .tap_byte(tap_byte),
        .tap_end(tap_end), .vp_flags(vp_flags), .vp_has(vp_has)
`ifdef TANGO_VSYNC_ADJUST
        , .vs_status(vs_status)
`endif
`endif
        );
`ifdef TANGO_LINK_TAP
    // ---------------------------------------------------------------- MiSTer's video / audio processing
    // (README "Video and audio processing"): tango_link's SYS bytes -> tango_vproc_regs -> tango_video, tango_audio_filter
    tango_vproc_regs u_vproc (
        .clk(clk_pix), .rst(rst_pix),
        .tap_cmd_stb(tap_cmd_stb), .tap_stb(tap_stb), .tap_cmd(tap_cmd), .tap_n(tap_n), .tap_byte(tap_byte),
        .tap_end(tap_end), .sl_level(vp_sl), .flags(vp_flags),
`ifdef TANGO_VIDEO_PROC
        .poly_wr(vp_poly_wr), .poly_a(vp_poly_a), .poly_dw(vp_poly_dw), .flt_mode(vp_flt_mode),
        .gam_en(vp_gam_en), .gam_wr(vp_gam_wr), .gam_a(vp_gam_a), .gam_d(vp_gam_d),
        .mask_wr(vp_mask_wr), .mask_cmd(vp_mask_cmd),
`endif
        .a_rst(af_rst), .a_rate(af_rate), .a_cx(af_cx), .a_cx0(af_cx0), .a_cx1(af_cx1), .a_cx2(af_cx2),
        .a_cy0(af_cy0), .a_cy1(af_cy1), .a_cy2(af_cy2), .a_boost(af_boost), .a_vol(af_vol), .preset_busy());
`ifdef TANGO_VSYNC_ADJUST
    // ---------------------------------------------------------------- native-rate output (TANGO_VSYNC_ADJUST)
    // tango_vsync measures the core's frame period on its VGA_VS, plans the raster's totals for it and hands them out
    // mid-frame (vt_nxt_*); SYS 0x70 (Tango.ini VSYNC_ADJUST via the firmware) sets the mode and the bounds.
    tango_vsync #(.HT0(`TANGO_HDMI_HTOTAL), .VT0(`TANGO_HDMI_VTOTAL), .MODE0(`TANGO_VSYNC_DEFAULT)) u_vsync (
        .clk(clk_pix), .rst(rst_pix), .core_vs(vga_vs), .cx(cx), .cy(cy),
        .tap_cmd_stb(tap_cmd_stb), .tap_stb(tap_stb), .tap_cmd(tap_cmd), .tap_n(tap_n), .tap_byte(tap_byte),
        .tap_end(tap_end), .status(vs_status), .nxt_w(vt_nxt_w), .nxt_h(vt_nxt_h),
`ifdef TANGO_HDMI_RETUNE
        // the fixed raster: the build's TANGO_HDMI_HTOTAL x _VTOTAL at the native preset (e.g. 1662 x 748 at 74.5875 MHz),
        // 1650 x 750 at 74.25 MHz (exactly 60.000 Hz: Tango.ini VSYNC_ADJUST=0)
        .pll_native(pll_native), .fix_w(pll_native ? 12'(`TANGO_HDMI_HTOTAL) : 12'd1650),
        .fix_h(pll_native ? 11'(`TANGO_HDMI_VTOTAL) : 11'd750), .rt_tog(rt_tog), .rt_native(rt_native),
`endif
        .meas_p(), .meas_ok(), .plan_w(), .plan_h(), .plan_err());
`endif
    // the audio filter between the core's samples and the board top's 48 kHz HDMI sampling
    tango_audio_filter #(.CLK_HZ_P(`TANGO_HDMI_FPIX), .S48_W(`TANGO_HDMI_S48_W), .S48_INC(`TANGO_HDMI_S48_INC),
                         .S48_MOD(`TANGO_HDMI_S48_MOD)) u_afilt (   // tango_cfg.vh: 74.25 MHz, +8 mod 12375 unless a preset
        .clk(clk_pix), .rst(rst_pix), .a_rst(af_rst), .rate(af_rate), .cx(af_cx),
        .cx0(af_cx0), .cx1(af_cx1), .cx2(af_cx2), .cy0(af_cy0), .cy1(af_cy1), .cy2(af_cy2),
        .in_l(core_audio_l), .in_r(core_audio_r), .att(af_vol), .boost(af_boost), .mix(core_audio_mix),
`ifdef TANGO_HDMI_RETUNE
        .pll_native(pll_native),
`endif
        .out_l(audio_l), .out_r(audio_r), .dbg_step(), .dbg_s48());
`endif
    // block device (menu-tango docs/spec "Block device"): tango_sd in TANGO_SD builds, else no request is ever posted
    wire [38:0] sd_req;  wire sd_in_we, sd_in_done, sd_out_done, mnt_tog, mnt_ro;
    wire [8:0]  sd_in_wa, sd_out_ra;  wire [7:0] sd_in_wd, sd_out_rd;  wire [3:0] mnt_drive;  wire [63:0] mnt_size;
    wire [135:0] link_dbg;

    // ---------------------------------------------------------------- USB (clk48)
    // reset: 4-flop release after lock, then one preserved flop per port
    // (a shared reset was clk48's worst path in an earlier design)
    reg [3:0] usb_rst_sr = 4'd0;
    always @(posedge clk48) usb_rst_sr <= {usb_rst_sr[2:0], lock48};
    (* syn_preserve = 1 *) reg usb1_rst_n = 1'b0;
    (* syn_preserve = 1 *) reg usb2_rst_n = 1'b0;
    always @(posedge clk48) begin usb1_rst_n <= usb_rst_sr[3]; usb2_rst_n <= usb_rst_sr[3]; end

    wire [63:0] rpt1, rpt2;  wire v1, v2, tg1, tg2;
    // generic pads: the second report path (1 byte or more), bcdDevice, the descriptor store, the decode engines
    wire [127:0] r16_1, r16_2;  wire [7:0] rl1, rl2;  wire t2g1, t2g2, v21, v22;  wire [15:0] ver_u1, ver_u2;
    wire rdy1, rdy2;  wire [8:0] lcfg1, lcfg2;  wire [10:0] lrpt1, lrpt2;  wire [7:0] ldev1, ldev2;  wire [1:0] dtr1, dtr2;
    wire [10:0] ds_ra;  wire [7:0] ds_rd1, ds_rd2;
    wire [31:0] eraw1, eraw2;  wire etog1, etog2, ebusy1, ebusy2;
    wire tw_we1, tw_we2;  wire [5:0] tw_addr;  wire [35:0] tw_din;  wire [26:0] pcfg1, pcfg2;  wire [1:0] ptmode;  wire pad_swap;
    wire l1, l2;  wire [127:0] grep1, grep2;  wire [7:0] glen1, glen2;  wire [15:0] gver1, gver2;
    wire [15:0] uvid1, upid1, uvid2, upid2;         // clk48, GET_DESCRIPTOR(DEVICE)
    wire [1:0]  ukind1, ukind2;                     // clk48, GET_DESCRIPTOR(CONFIGURATION): pad / keyboard / other
    tango_usb_port #(.DEBOUNCE_CYCLES(USB_DEBOUNCE)) u_usb1 (
        .clk48(clk48), .rst_n(usb1_rst_n), .dp(usb1_dp), .dn(usb1_dn),
        .rpt(rpt1), .rpt_valid(v1), .rpt_tog(tg1), .status(), .bad_crc(usb1_bad_crc), .vid(uvid1), .pid(upid1), .kind(ukind1),
        .rpt16(r16_1), .rpt_len(rl1), .rpt2_tog(t2g1), .rpt2_valid(v21), .ver(ver_u1), .desc_rdy(rdy1), .len_cfg(lcfg1), .len_rpt(lrpt1),
        .len_dev(ldev1), .desc_trunc(dtr1), .rclk(clk_pix), .ds_ra(ds_ra), .ds_rd(ds_rd1));
    tango_usb_port #(.DEBOUNCE_CYCLES(USB_DEBOUNCE)) u_usb2 (
        .clk48(clk48), .rst_n(usb2_rst_n), .dp(usb2_dp), .dn(usb2_dn),
        .rpt(rpt2), .rpt_valid(v2), .rpt_tog(tg2), .status(), .bad_crc(usb2_bad_crc), .vid(uvid2), .pid(upid2), .kind(ukind2),
        .rpt16(r16_2), .rpt_len(rl2), .rpt2_tog(t2g2), .rpt2_valid(v22), .ver(ver_u2), .desc_rdy(rdy2), .len_cfg(lcfg2), .len_rpt(lrpt2),
        .len_dev(ldev2), .desc_trunc(dtr2), .rclk(clk_pix), .ds_ra(ds_ra), .ds_rd(ds_rd2));
    // the table-driven decode engines (clk48), one per port; tables are written by tango_link (SET_DECODE)
    tango_pad_decode u_dec1 (.clk(clk48), .rst_n(usb1_rst_n), .rpt(r16_1), .rpt_len(rl1), .rpt_tog(t2g1), .rpt_valid(v21),
        .wclk(clk_pix), .tw_we(tw_we1), .tw_addr(tw_addr), .tw_din(tw_din), .cfg_mode(pcfg1[1:0]), .cfg_rid_en(pcfg1[2]), .cfg_rid_val(pcfg1[10:3]),
        .cfg_min_len(pcfg1[18:11]), .cfg_count(pcfg1[25:19]), .cfg_hold(pcfg1[26]),
        .raw(eraw1), .raw_tog(etog1), .raw_valid(), .busy(ebusy1));
    tango_pad_decode u_dec2 (.clk(clk48), .rst_n(usb2_rst_n), .rpt(r16_2), .rpt_len(rl2), .rpt_tog(t2g2), .rpt_valid(v22),
        .wclk(clk_pix), .tw_we(tw_we2), .tw_addr(tw_addr), .tw_din(tw_din), .cfg_mode(pcfg2[1:0]), .cfg_rid_en(pcfg2[2]), .cfg_rid_val(pcfg2[10:3]),
        .cfg_min_len(pcfg2[18:11]), .cfg_count(pcfg2[25:19]), .cfg_hold(pcfg2[26]),
        .raw(eraw2), .raw_tog(etog2), .raw_valid(), .busy(ebusy2));

    // ---------------------------------------------------------------- clk48 -> clk_pix
    // per port: joy = the fixed default decode (GET_DB9, the menu, Start+Select: MiSTer's def_mmap),
    // c = the core map applied (joystick_0/1), r = the raw buttons, vid/pid (REPORT); menu-tango docs/spec "Define buttons"
    wire [15:0] j1, j2;  wire p1, p2;
    wire [31:0] c1, c2, r1, r2;  wire [15:0] vid1, pid1, vid2, pid2;
    wire [383:0] pm1, pm2;
    wire [1:0]  k1, k2;  wire [63:0] kb1, kb2;  wire [10:0] ps2_key_pix;
    tango_ds4_joy u_j1 (.clk(clk_pix), .rst(rst_pix), .rpt(rpt1), .rpt_tog(tg1), .rpt_valid(v1),
        .vid_in(uvid1), .pid_in(upid1), .kind_in(ukind1), .padmap(pm1),
        .rpt16_in(r16_1), .rpt_len_in(rl1), .rpt2_tog(t2g1), .rpt2_valid(v21), .ver_in(ver_u1), .traw_in(eraw1), .traw_tog(etog1), .tmode(ptmode[0]),
        .joy(j1), .core(c1), .raw(r1), .vid(vid1), .pid(pid1), .kind(k1), .kbd(kb1), .present(p1),
        .rep16(grep1), .rep_len(glen1), .ver(gver1), .live(l1));
    tango_ds4_joy u_j2 (.clk(clk_pix), .rst(rst_pix), .rpt(rpt2), .rpt_tog(tg2), .rpt_valid(v2),
        .vid_in(uvid2), .pid_in(upid2), .kind_in(ukind2), .padmap(pm2),
        .rpt16_in(r16_2), .rpt_len_in(rl2), .rpt2_tog(t2g2), .rpt2_valid(v22), .ver_in(ver_u2), .traw_in(eraw2), .traw_tog(etog2), .tmode(ptmode[1]),
        .joy(j2), .core(c2), .raw(r2), .vid(vid2), .pid(pid2), .kind(k2), .kbd(kb2), .present(p2),
        .rep16(grep2), .rep_len(glen2), .ver(gver2), .live(l2));
    reg [15:0] joy_r = 16'd0;
    reg [7:0]  flags_r = 8'd0;
    always @(posedge clk_pix) begin
        joy_r   <= j1 | j2;
        flags_r <= {6'd0, p2, p1};      // bit0 USB-A1 pad live, bit1 USB-A2 pad live
    end
    assign joy = joy_r;

    // ---------------------------------------------------------------- the core
    wire         clk_sys;
    wire [31:0]  joystick_0, joystick_1;
    wire [127:0] status_c, status_l;
    wire         status_tog;
    wire [15:0]  menumask_c, menumask_l;
    wire         ioctl_download, ioctl_wr, ioctl_wait;
    wire [15:0]  ioctl_index;
    wire [26:0]  ioctl_addr;
    wire [7:0]   ioctl_dout;
    wire [11:0]  conf_addr;  wire [7:0] conf_char;
    wire         dl_we;  wire [9:0] dl_wd;  wire [11:0] dl_free;
`ifdef TANGO_IOCTL_UPLOAD
    // upload (TANGO_IOCTL_UPLOAD): tango_link <-> tango_io <-> tango_core
    wire         ioctl_upload, ioctl_upload_req, ioctl_rd;
    wire [7:0]   ioctl_upload_index, ioctl_din;
    wire         ul_tog, ul_ready;
    wire [7:0]   ul_idx, ul_rd;
    wire [23:0]  ul_size_o;
    wire [10:0]  ul_ra;
`endif
    tango_core u_core (
        .CLK_50M(clk50), .CLK_PIX(clk_pix), .RESET(rst_pix), .CLK_SYS(clk_sys),
        .joystick_0(joystick_0), .joystick_1(joystick_1),
        .status(status_c), .status_menumask(menumask_c),
        .ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr),
        .ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout), .ioctl_wait(ioctl_wait),
        .conf_addr(conf_addr), .conf_char(conf_char),
`ifdef TANGO_VIDEO_FB
        .CLK_VIDEO(clk_video), .CE_PIXEL(ce_pixel), .VGA_R(vga_r), .VGA_G(vga_g), .VGA_B(vga_b),
        .VGA_HS(vga_hs), .VGA_VS(vga_vs), .VGA_DE(vga_de), .VGA_SL(vga_sl),
        .VIDEO_ARX(video_arx), .VIDEO_ARY(video_ary), .HDMI_WIDTH(12'd1280), .HDMI_HEIGHT(12'd720),
        .HDMI_FREEZE(hdmi_freeze), .VIDEO_SCALE(video_scale), .VIDEO_CROP_SIZE(video_crop_size),
        .VIDEO_CROP_OFF(video_crop_off),
`ifdef TANGO_VIDEO_IDX
        .VGA_IDX(vga_idx), .VIDEO_PAL_WR(vpal_wr), .VIDEO_PAL_A(vpal_a), .VIDEO_PAL_D(vpal_d),
`endif
`elsif TANGO_VIDEO_ASCAL
        .CLK_VIDEO(clk_video), .CE_PIXEL(ce_pixel), .VGA_R(vga_r), .VGA_G(vga_g), .VGA_B(vga_b),
        .VGA_HS(vga_hs), .VGA_VS(vga_vs), .VGA_DE(vga_de), .VGA_SL(vga_sl),
        .VIDEO_ARX(video_arx), .VIDEO_ARY(video_ary), .HDMI_WIDTH(12'd1280), .HDMI_HEIGHT(12'd720),
        .HDMI_FREEZE(hdmi_freeze), .VIDEO_SCALE(video_scale), .VIDEO_CROP_SIZE(video_crop_size),
        .VIDEO_CROP_OFF(video_crop_off),
`ifdef TANGO_VIDEO_F1
        .VGA_F1(vga_f1),
`endif
`ifdef TANGO_ASCAL_EXT
        .ASCAL_AVL_CLK(ascal_avl_clk),
        .ascal_avl_address(vid_avl_addr), .ascal_avl_write(vid_avl_write), .ascal_avl_writedata(vid_avl_wdata),
        .ascal_avl_read(vid_avl_read), .ascal_avl_burstcount(vid_avl_burst), .ascal_avl_byteenable(vid_avl_be),
        .ascal_avl_readdata(vid_avl_rdata), .ascal_avl_readdatavalid(vid_avl_rvalid),
        .ascal_avl_waitrequest(vid_avl_waitreq),
`endif
`else
        .VID_CX(ccx), .VID_CY(ccy), .VID_RGB(bg),
`endif
`ifdef TANGO_AUDIO_FILTER
        .AUDIO_L(core_audio_l), .AUDIO_R(core_audio_r),     // through tango_audio_filter (above)
        .AUDIO_MIX(core_audio_mix),                           // MiSTer's stereo mix (tango_core_ports.vh)
`else
        .AUDIO_L(audio_l), .AUDIO_R(audio_r),
`endif
        .OSD_STATUS(osd_status_sys), .ps2_key(ps2_key_sys)
`ifdef TANGO_IOCTL_UPLOAD
        , .ioctl_upload(ioctl_upload), .ioctl_upload_req(ioctl_upload_req), .ioctl_upload_index(ioctl_upload_index),
        .ioctl_din(ioctl_din), .ioctl_rd(ioctl_rd)
`endif
`ifdef TANGO_SD
        , .img_mounted(img_mounted), .img_readonly(img_readonly), .img_size(img_size), .sd_lba(sd_lba),
        .sd_blk_cnt(sd_blk_cnt), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack), .sd_buff_addr(sd_buff_addr),
        .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr)
`endif
`ifdef TANGO_SDRAM
        , .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQ(SDRAM_DQ), .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH), .SDRAM_nCS(SDRAM_nCS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nWE(SDRAM_nWE)
`endif
`ifdef TANGO_NES_SDRAM
        , .addrA(c_addrA), .oeA(c_oeA), .weA(c_weA), .dinA(c_dinA), .doutA(c_doutA),
          .addrB(c_addrB), .oeB(c_oeB), .weB(c_weB), .dinB(c_dinB), .doutB(c_doutB),
          .clkref(c_clkref), .sdram_busy(sdram_busy)
`endif
        );
`ifdef TANGO_NES_SDRAM
    // ports A/B front end (CLK_SYS): the slot's requests, held for the slot, + the slot toggle for tango_sdram
    tango_ab_slot u_ab_slot (
        .clk(clk_sys), .slot_ce(c_clkref),
        .addrA(c_addrA), .oeA(c_oeA), .weA(c_weA), .dinA(c_dinA),
        .addrB(c_addrB), .oeB(c_oeB), .weB(c_weB), .dinB(c_dinB),
        .h_addrA(h_addrA), .h_oeA(h_oeA), .h_weA(h_weA), .h_dinA(h_dinA),
        .h_addrB(h_addrB), .h_oeB(h_oeB), .h_weB(h_weB), .h_dinB(h_dinB),
        .slot_tog(h_slot), .lost());
`endif
    tango_io u_io (
        .clk_pix(clk_pix), .joy_a2(pad_swap ? c1 : c2), .joy_a1(pad_swap ? c2 : c1), .osd_shown(osd_shown),   // PAD_CTRL bit1: MiSTer's joyswap
        .status_l(status_l), .status_tog(status_tog), .menumask_l(menumask_l),
        .dl_we(dl_we), .dl_wd(dl_wd), .dl_free(dl_free),
        .clk_sys(clk_sys), .joystick_0(joystick_0), .joystick_1(joystick_1),
        .status(status_c), .status_menumask(menumask_c),
        .ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr),
        .ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout), .ioctl_wait(ioctl_wait), .osd_status(osd_status_sys),
        .ps2_key_pix(ps2_key_pix), .ps2_key(ps2_key_sys)
`ifdef TANGO_IOCTL_UPLOAD
        , .ioctl_upload(ioctl_upload), .ioctl_upload_req(ioctl_upload_req), .ioctl_upload_index(ioctl_upload_index),
        .ioctl_din(ioctl_din), .ioctl_rd(ioctl_rd),
        .ul_tog(ul_tog), .ul_idx(ul_idx), .ul_size_l(ul_size_o), .ul_ra(ul_ra), .ul_rd(ul_rd), .ul_ready(ul_ready)
`endif
        );
    wire osd_status_sys;  wire [10:0] ps2_key_sys;
`ifdef TANGO_SD
    localparam integer VDN = `TANGO_SD_VDNUM;
    wire [VDN-1:0] img_mounted, sd_rd, sd_wr, sd_ack;  wire img_readonly, sd_buff_wr;  wire [63:0] img_size;
    wire [31:0] sd_lba [VDN];  wire [5:0] sd_blk_cnt [VDN];  wire [7:0] sd_buff_din [VDN];
    wire [13:0] sd_buff_addr;  wire [7:0] sd_buff_dout;
    tango_sd #(.VDNUM(VDN)) u_sd (
        .clk_pix(clk_pix), .in_we(sd_in_we), .in_wa(sd_in_wa), .in_wd(sd_in_wd), .in_done_tog(sd_in_done),
        .out_ra(sd_out_ra), .out_rd(sd_out_rd), .out_done_tog(sd_out_done),
        .mnt_tog(mnt_tog), .mnt_drive(mnt_drive), .mnt_ro(mnt_ro), .mnt_size(mnt_size), .req_pix(sd_req),
        .clk_sys(clk_sys), .img_mounted(img_mounted), .img_readonly(img_readonly), .img_size(img_size),
        .sd_lba(sd_lba), .sd_blk_cnt(sd_blk_cnt), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
        .sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr));
`else
    assign sd_req = 39'd0;  assign sd_out_rd = 8'd0;
`endif
    assign dbg_rows = {link_dbg[135:48], jsel_s[1], jtagsel_n_q, osd_shown, osd_bank, osd_swap_pending, p2, p1, rst_pix,
                       link_dbg[47:0]};   // rows 0-10 link, 11 core, 12-17 link
    assign joy_flags = flags_r;
endmodule
`default_nettype wire
