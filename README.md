# sys-tango -- the Tango framework

**Version 2026-10-27** (date code `20261027`)

sys-tango is the framework every Tango core is built on: a MiSTer-style `sys/` for the
**Sipeed Tang Console 138K**. A core mounts this repository as a git submodule at `sys/` and keeps its own code beside
it, the way a MiSTer core carries `sys/`. The framework supplies the board top, HDMI output, the link to the board's
BL616 microcontroller and its on-screen menu, the USB controller ports, the video scaler with its frame buffers in
SDRAM or DDR3, MiSTer's video and audio processing, and the build scripts that turn a core into a bitstream.

The framework has no bitstream of its own. It is built into every core, and each core's release carries that core's
bitstreams (`.bin`). sys-tango is MIT-licensed except five files derived from MiSTer, which are under the GNU GPL: see
[Licence](#licence). A bitstream built with the GPL files is a work under the GPL. Nothing of Gowin's is in this
repository: a build takes `PLL_INIT` (and, in DDR3 builds, the DDR3 IP) from the user's Gowin EDA installation. The
BL616 firmware is released separately in [firmware-tango](https://github.com/Tango-FPGA/firmware-tango), and the launcher
that loads cores from the SD card in [menu-tango](https://github.com/Tango-FPGA/menu-tango).

## Contents

- [Features](#features)
- [Board support](#board-support)
- [Using it in a core](#using-it-in-a-core)
- [Build options](#build-options)
- [Video](#video)
- [Rotation](#rotation)
- [The DDR3 path](#the-ddr3-path)
- [Native-rate video (vsync_adjust)](#native-rate-video-vsync_adjust)
- [Video and audio processing](#video-and-audio-processing)
- [Controllers](#controllers)
- [The shared SDRAM controller](#the-shared-sdram-controller)
- [Timing constraints](#timing-constraints)
- [Building](#building)
- [Simulation benches](#simulation-benches)
- [Layout](#layout)
- [Attributions](#attributions)
- [What was ported or adapted](#what-was-ported-or-adapted)
- [Licence](#licence)

## Features

- **Board top for the Tang Console 138K** (`board/console138k/tango_sys_top138.v`): PLLs (with Gowin's `PLL_INIT`,
  which the GW5AST needs to lock, taken from the Gowin installation at build time), **HDMI 1280x720p60** with 48 kHz 16-bit stereo audio, the BL616 link pins and the
  JTAG hand-back (the BL616 can still take the JTAG pins back to load the next core), the two USB-A host ports, and
  in DDR3 builds the board's DDR3 with the Gowin DDR3 Memory Interface IP.
- **BL616 link** (`tango_link.v`): the SPI link to the BL616 in the FPGA Companion wire format (targets SYS 0, HID 1,
  OSD 2). Over it the firmware reads the core's `CONF_STR`, writes the option word, sends files (ROMs) into the core,
  reads files back from it (uploads), serves SD-card sectors, reads the pads and keyboards, programs the pad decoder
  and draws the menu.
- **OSD overlay** (`tango_menu_overlay.v`): the renderer of the on-screen menu. The firmware uploads the glyph atlas,
  the glyph and row lists, the palette and the geometry; the overlay draws a translucent rounded panel with
  anti-aliased text over the running core, and switches pages at a frame boundary.
- **USB hosts** (`usb/`, `tango_usb_port.v`, `tango_ds4_joy.v`, `tango_pad_decode.v`): two USB-A host ports, our own
  full- and low-speed host: DualShock 4 pads (a built-in decode), any other HID pad through a table-driven decoder that
  the firmware programs from its controller database, and HID boot-protocol keyboards (as MiSTer's `ps2_key`). The
  firmware's "Define buttons" maps any pad button to any core button. See [Controllers](#controllers).
- **Core interface** (`tango_core_ports.vh`, `tango_io.v`): what MiSTer's `hps_io` gives an emu, with MiSTer's signal
  meanings: `joystick_0/1`, the 128-bit option word `status`, `status_menumask`, file download (`ioctl_*`, with
  `ioctl_wait`), file upload (`TANGO_IOCTL_UPLOAD`), `OSD_STATUS`, `ps2_key`; the core runs in a clock of its own
  choice.
- **ascal scaler** (`TANGO_VIDEO_ASCAL`): the core outputs MiSTer's emu video ports; TEMLIB's ascal (the scaler MiSTer
  uses) triple-buffers the frame and scales it to 720p. The frame buffers are in the 32 MB SDRAM module
  (`tango_sdram.v`, 96 MHz, shared with two real-time byte ports a core can use, `TANGO_NES_SDRAM`), or in the
  board's 1 GB DDR3 (`TANGO_VIDEO_DDR3`, which leaves the SDRAM to the core), or a core's own SDRAM controller serves
  ascal (`TANGO_ASCAL_EXT`).
- **Video page** (`TANGO_VIDEO_OPTS`): the framework owns the standard Video entries of the menu -- Aspect ratio
  (Original, Full Screen, 4:3, 16:9), Scale (Normal, V-Integer, Narrower / Wider HV-Integer, as MiSTer's
  `video_freak`) and Scanlines (None, 25%, 50%, 75%) -- so a core no longer wires them. Arcade builds add Rotation.
- **Screen rotation** (`TANGO_VIDEO_ROTATE`, arcade cores): Normal, CW and CCW, written through the DDR3 and shown by
  ascal's frame-buffer reader, as MiSTer's `screen_rotate`.
- **Native-rate video** (`TANGO_VSYNC_ADJUST`): the HDMI raster follows the core's own frame rate (MiSTer's
  `vsync_adjust`), with optional build-time HDMI pixel-clock presets and a pixel-clock retune at core start.
- **MiSTer video processing** (`TANGO_VIDEO_PROC`): the scaler's polyphase filters (ascal at MiSTer's 256 phases),
  gamma correction and shadow masks, loaded by the firmware from MiSTer's own filter, gamma and mask files.
- **Audio filter and mix** (`TANGO_AUDIO_FILTER`): MiSTer's audio filter (an IIR at the filter file's rate, then a DC
  blocker), its boost, stereo mix and global volume, in front of the 48 kHz HDMI audio.
- **SD block device** (`TANGO_SD`): MiSTer's `sd_*` / `img_*` ports (save RAM, disk images); the BL616 owns the
  card and moves 512-byte sectors over the link.
- **Contracts, checked in every simulation:** the CE_PIXEL contract (a core's `CE_PIXEL` keeps pulsing through the
  blanking, and its sync signals change only on CE) and the VIDEO_OPTS contract (a core that opts in to the Video
  page leaves `VIDEO_SCALE` and `VGA_SL` at 0). A core that breaks one stops the simulation with
  `TANGO CONTRACT: ...`. The checks are simulation-only; they never reach a bitstream.
- **Build-time presets** for testing without the firmware (over JTAG): `TANGO_PRESET_STATUS` (the option word at
  power-up) and `TANGO_VPROC_PRESET` (a compiled-in load of filters, gamma, mask and audio filter, replayed after
  reset).
- **Build and gate scripts**: `build_core.sh` (the Gowin build of the framework + the core), `sta_gate.py` (a
  timing and resource gate: a build that fails it must not be flashed), `fs2bin.py` (the `.bin` the BL616 loader
  streams), `confstr_expand.py` (a core's `CONF_STR` with its macros expanded), `tango_capped.sh` (runs a heavy job
  under a memory cap).

## Board support

| Board | FPGA | Status |
|---|---|---|
| **Sipeed Tang Console 138K** | GW5AST-LV138PG484AC1/I0, device version B | supported: HDMI, BL616 link, two USB-A ports; the SDRAM video path and `TANGO_SDRAM` need the board's SDRAM module (W9825G6KH, 32 MB); `TANGO_VIDEO_DDR3` uses the board's DDR3 (two H5TQ4G63EFR x16 as one 32-bit bus, 1 GB) |
| Other Tang boards (Console 60K, Mega 138K Pro, Primer 25K, ...) | | not supported |

The BL616 on the board runs one of two firmwares. With the **Tango firmware** (`firmware-tango`) it loads the launcher
and the cores from the SD card and runs their menus. When the board's USB-C for the BL616 is plugged into a PC, the
BL616 acts as **Sipeed's USB-JTAG debugger** instead, and a bitstream can be loaded over JTAG. That debugger
firmware is Sipeed's and is not part of any Tango release: get it from Sipeed.

## Using it in a core

Start from the console template (`console-template-tango`): a forkable core with this framework at `sys/`, its
simulation gates and its build script. The framework goes into a core as a submodule:

    # add it to a new core
    git submodule add https://github.com/Tango-FPGA/sys-tango.git sys
    # check out a core with it, or fetch it in an existing checkout
    git clone --recursive <core repository>
    git submodule update --init

The submodule commit a core records is the framework version it was built and tested with. To take a newer
framework, move the pointer (`git -C sys pull`), rebuild and re-test, then commit the pointer.

A core's top module is `tango_core`, with the framework's port list:

```verilog
module tango_core (
    `include "tango_core_ports.vh"
);
```

The board top instantiates the framework (`tango_sys.v`), and the framework instantiates `tango_core`, as MiSTer's
`sys_top` instantiates `emu`. The only framework module a core instantiates itself is `tango_conf_rom`, which serves
its `CONF_STR` to the BL616:

```verilog
localparam CONF_STR = {
    "MyCore;;",
    "-;",
    "P1,Video;",
    `TANGO_VIDEO_CONF_P("P1"),          // Aspect ratio, Scale, Scanlines (TANGO_VIDEO_OPTS)
    "-;",
    "O[2],My option,Off,On;",
    "R[0],Reset and close OSD;",
    "V,",`BUILD_DATE                    // build_core.sh defines BUILD_DATE as "v<yymmdd>"
};
tango_conf_rom #(.CONF_STR(CONF_STR)) u_conf (.clk(CLK_PIX), .addr(conf_addr), .q(conf_char));
```

A `CONF_STR` may be up to 4095 bytes (12-bit `conf_addr`).

### The core interface

`tango_core_ports.vh` declares the ports; their meanings are MiSTer's (`hps_io` and the emu ports).

| Signal | Dir | Clock | Meaning |
|---|---|---|---|
| `CLK_50M` | in | | the board oscillator, for the core's own PLLs (wrap each GW5AST PLL with `PLL_INIT`, as `board/console138k/pll_*.v`; every build already adds `PLL_INIT`, so a core does not add another copy) |
| `CLK_PIX` | in | | 74.25 MHz, the 720p pixel clock (or a `TANGO_HDMI_PLL` preset's) |
| `RESET` | in | | high while the pixel PLL is not locked |
| `CLK_SYS` | out | | the core's system clock; every MiSTer-style signal below is in it (it may be `CLK_PIX`) |
| `joystick_0`, `joystick_1` [31:0] | in | CLK_SYS | the pads on USB-A2 and USB-A1: bit 0 right, 1 left, 2 down, 3 up, 4 A, 5 B, 6 X, 7 Y, 8 L, 9 R, 10 Select, 11 Start; 0 while the menu is open |
| `status` [127:0] | in | CLK_SYS | the option word: the `CONF_STR` `O`/`T`/`R` bits; `status[0]` is the soft reset; with `TANGO_VIDEO_OPTS`, `status[127:118]` belongs to the framework |
| `status_menumask` [15:0] | out | CLK_SYS | the bits the `CONF_STR` `H`/`D` prefixes test |
| `ioctl_download`, `ioctl_index`, `ioctl_wr`, `ioctl_addr`, `ioctl_dout` | in | CLK_SYS | a file from an `F` entry, one byte per `ioctl_wr` |
| `ioctl_wait` | out | CLK_SYS | high holds the next byte back; nothing is lost |
| `ioctl_upload`, `ioctl_rd` / `ioctl_din` | in / out | CLK_SYS | with `TANGO_IOCTL_UPLOAD`: the firmware reads a file back from the core (e.g. NVRAM), MiSTer's upload direction: the framework raises `ioctl_upload`, sets `ioctl_addr` / `ioctl_index` and pulses `ioctl_rd`; the core answers on `ioctl_din` (up to 2048 bytes; `ioctl_upload_req` / `_index` are present but not acted on) |
| `conf_addr` / `conf_char` | in / out | | the `CONF_STR` ROM (`tango_conf_rom`) |
| `AUDIO_L`, `AUDIO_R` [15:0] | out | CLK_PIX | audio samples, taken at 48 kHz |
| `AUDIO_MIX` [1:0] | out | | with `TANGO_AUDIO_FILTER` / `TANGO_VIDEO_PROC`: MiSTer's stereo mix (none, 25%, 50%, mono) |
| `OSD_STATUS` | in | CLK_SYS | high while the menu is shown |
| `ps2_key` [10:0] | in | CLK_SYS | a keyboard event, as MiSTer's: `[7:0]` set-2 code, `[8]` extended, `[9]` pressed, `[10]` toggles per event |
| video | | | the 720p beam, or (with a scaler build) MiSTer's emu video ports: see [Video](#video) |
| `VGA_F1` | out | CLK_VIDEO | with `TANGO_VIDEO_F1`: MiSTer's interlace field |
| `sd_*`, `img_*` | | CLK_SYS | with `TANGO_SD`: MiSTer's block device (`tango_sd_ports.vh`) |
| `SDRAM_*` | | | with `TANGO_SDRAM`: the SDRAM pins, for a core with its own controller (`tango_sdram_ports.vh`) |
| `ascal_avl_*`, `ASCAL_AVL_CLK` | | | with `TANGO_ASCAL_EXT`: ascal's Avalon master, served by the core's own SDRAM controller (see [A core that owns the SDRAM](#a-core-that-owns-the-sdram)) |
| `addrA/B`, `oeA/B`, `weA/B`, `dinA/B`, `doutA/B`, `clkref`, `sdram_busy` | | | with `TANGO_NES_SDRAM`: two byte ports into the framework's SDRAM controller |

Not provided: `joystick_2..5`, analog sticks, paddles and spinners, a mouse, keyboards behind a hub, `status_set`,
RTC, and MiSTer's `DDRAM_*` port for a core's own use (the DDR3's core port serves the rotation, see
[Rotation](#rotation)).

## Build options

A core's build script calls `sys/board/console138k/build_core.sh` with the options as environment variables:

    TANGO_VIDEO_ASCAL=1 TANGO_VIDEO_OPTS=1 TANGO_OUT=build/out \
        sys/board/console138k/build_core.sh tango_mycore138 build/mycore.sdc rtl/tango_core_mycore.v rtl/...

Arguments: the output name, the core's own SDC (or `-`), then the core's HDL files. The script generates a
`tango_defines.v` with the options as `` `define ``s (compiled first), builds with Gowin, runs `sta_gate.py` and
archives `<name>_<YYYYMMDD-HHMM>_g<git>.{fs,bin}` under `$TANGO_ARCHIVE` (default `$TANGO_OUT/archive`;
`TANGO_NO_ARCHIVE=1` skips it). Simulations pass the same options as `-D` / `+define+`. Without an option, every
build is exactly as it was before the option existed.

| Option | What it does |
|---|---|
| (none) | the core colours the 720p beam itself (`VID_CX`, `VID_CY` -> `VID_RGB`, two clocks of latency); no scaler |
| `TANGO_VIDEO_ASCAL=1` | the core outputs MiSTer's emu video ports; ascal scales them, its frames in the SDRAM. The framework owns the SDRAM: do not set `TANGO_SDRAM` (unless `TANGO_ASCAL_EXT` or `TANGO_VIDEO_DDR3`). `TANGO_VIDEO_ASCAL_MAXW` / `_MAXH` size the input (defaults 256 x 240) |
| `TANGO_VIDEO_DDR3=1` | (with ascal; not with `TANGO_ASCAL_EXT` or `TANGO_DBG`) ascal's frames in the DDR3: see [The DDR3 path](#the-ddr3-path). The SDRAM is then free for the core (`TANGO_SDRAM`), or keeps serving ports A/B (`TANGO_NES_SDRAM`) |
| `TANGO_ASCAL_EXT=1` | (with ascal and `TANGO_SDRAM`; not with `TANGO_NES_SDRAM`) the core owns the SDRAM and its controller also serves ascal: see [A core that owns the SDRAM](#a-core-that-owns-the-sdram) |
| `TANGO_VIDEO_F1=1` | (with ascal) the core drives `VGA_F1`: ascal detects interlaced video and weaves the fields, as on MiSTer, and the framework's scanlines are off while the input is interlaced |
| `TANGO_VIDEO_OPTS=1` | (with ascal) the framework's Video page: Aspect ratio, Scale, Scanlines in `status[127:122]` |
| `TANGO_VIDEO_ROTATE=1` | (with `TANGO_VIDEO_DDR3`; sets `TANGO_DDRAM`) screen rotation for arcade cores: see [Rotation](#rotation) |
| `TANGO_VIDEO_PROC=1` | (with ascal) MiSTer's video processing: polyphase filters, gamma, shadow mask; implies `TANGO_AUDIO_FILTER` |
| `TANGO_AUDIO_FILTER=1` | MiSTer's audio filter, boost, stereo mix and global volume (any core) |
| `TANGO_VSYNC_ADJUST=1` [`TANGO_VSYNC_DEFAULT=0\|1`] | (with ascal and `TANGO_VIDEO_PROC` or `TANGO_AUDIO_FILTER`) native-rate video: see [Native-rate video](#native-rate-video-vsync_adjust) |
| `TANGO_HDMI_HTOTAL=<1626..1720>` `TANGO_HDMI_VTOTAL=<740..790>` [`TANGO_HDMI_VIC=0\|4`] | the HDMI raster's totals at build time (default 1650 x 750, VIC 4) |
| `TANGO_HDMI_PLL=745875\|75600` | the HDMI pixel clock at 74.5875 or 75.6 MHz instead of 74.25 MHz (the SDC, the audio clock and the ACR follow) |
| `TANGO_HDMI_RETUNE=1` | (with `TANGO_VSYNC_ADJUST` and `TANGO_HDMI_PLL`) the pixel clock switched at core start between the preset and 74.25 MHz |
| `TANGO_IOCTL_UPLOAD=1` | MiSTer's upload direction (`ioctl_upload`, `ioctl_rd`, `ioctl_din`) |
| `TANGO_NES_SDRAM=1` | (with ascal) ports A and B into the framework's SDRAM controller |
| `TANGO_SDRAM=1` | the SDRAM pins go to the core, which brings its own controller (as on MiSTer) |
| `TANGO_SD=1` (`TANGO_SD_VDNUM=1..4`) | MiSTer's block device |
| `TANGO_PRESET_STATUS=<hex>` | the option word at power-up (up to 32 hex digits) |
| `TANGO_VPROC_PRESET="<vproc_preset.py arguments>"` | a compiled-in video/audio processing load, e.g. `"--preset 'Scanlines - Soft.ini'"`; needs `TANGO_FILTERS_ROOT` |
| `TANGO_BUILD_DATE=<yymmdd>` | the version date the core shows (default today), to rebuild an old build bit for bit |
| `TANGO_DBG=1` | a debug build: the link's state as blocks at the top-left of the picture |
| `TANGO_ASCAL_SRC=vhdl` | build ascal from `ascal.vhd` with Gowin's VHDL front end instead of the GHDL netlist (not recommended: see [Video](#video)) |
| `TANGO_EXTRA_CST=<file>` | a core's placement constraints (`INS_LOC` lines only), appended to the board's |
| `TANGO_SYN_RETIME=1` | Gowin's own retiming, timing-driven synthesis, maximum place-and-route effort and hold fixing (changes no logic) |
| `TANGO_MEM_CAP` | the memory cap of the Gowin job (default `4G`) |
| `TANGO_VIDEO_FB=1` | an earlier BSRAM frame-buffer path; `tango_video.v` is now the ascal version, so new cores use `TANGO_VIDEO_ASCAL` |

`tango_cfg.vh` documents every option and refuses combinations that cannot work.

## Video

**Without a scaler** the core colours the 720p beam: `VID_CX` 0..1649, `VID_CY` 0..749 (visible 0..1279 x 0..719),
and `VID_RGB` is the colour for the beam position of two clocks earlier.

**With `TANGO_VIDEO_ASCAL`** the core outputs MiSTer's emu video ports (`CLK_VIDEO`, `CE_PIXEL`, `VGA_R/G/B`,
`VGA_HS/VS/DE`, `VGA_SL`, `VIDEO_ARX/ARY`, `VIDEO_SCALE`, `VIDEO_CROP_SIZE/OFF`, `HDMI_FREEZE`). `tango_video_geo.v`
computes where the picture goes on the 1280x720 screen with the rules of MiSTer's `video_freak` (crop, aspect ratio,
SCALE 0-4), and ascal draws it there. ascal triple-buffers frames, so the core's frame rate need not match 60 Hz.

- **The CE_PIXEL rule.** `CE_PIXEL` must keep pulsing through the horizontal and vertical blanking, and
  `VGA_HS/VS/DE` must change only on `CE_PIXEL` clocks: ascal reads sync only on CE, so a core that pulses CE only in
  the picture never starts a frame (the screen stays on ascal's no-input picture). Every simulation that builds
  `tango_video.v` checks this per frame.
- **ascal comes from a GHDL netlist.** Gowin's VHDL front end builds `ascal.vhd` wrongly (on the board the picture is
  squeezed to 3/4 of its width) without any warning. The board build and the simulations therefore use GHDL's
  Verilog netlist of the same file (`ascal_ghdl/ascal_fixed.v`; `ascal_fixed8.v` for `TANGO_VIDEO_PROC`; the
  `_ddr3` variants for `TANGO_VIDEO_DDR3`), so the board runs exactly the logic the simulations run. Its generics
  are fixed in `ascal_ghdl/ascal_fixed*.vhd`, which is why `build_core.sh` refuses other `TANGO_VIDEO_ASCAL_RAMBASE` /
  `_RAMSIZE` values. `ascal_ghdl/gen.sh` (`gen.sh 8`, `4d`, `8d`) regenerates the netlists.

### Black until the core's first frame

ascal draws its output window from the moment it runs, frame or no frame. The framework therefore keeps the picture
black until the core's first frame has been measured and its geometry computed, and opens it at an output VS rise:
the first picture shown is a whole frame of the core's, and a core that sends no picture yet (a console core before a
ROM is loaded) shows a black screen. The on-screen menu is drawn over the black. Once open, the gate stays open until
the pixel clock's reset.

### The Video page (`TANGO_VIDEO_OPTS`)

The framework owns the top bits of the option word and decodes them itself:

| Bits | Entry | Values |
|---|---|---|
| `O[127:126]` | Aspect ratio | Original (the core's `VIDEO_ARX/ARY`), Full Screen, 4:3, 16:9 |
| `O[125:124]` | Scale | Normal, V-Integer, Narrower HV-Integer, Wider HV-Integer; arcade builds (`TANGO_VIDEO_ROTATE`): Normal, V-Integer |
| `O[123:122]` | Scanlines | None, 25%, 50%, 75% |
| `O[121:120]` | (reserved) | |
| `O[119:118]` | Rotation (only in `TANGO_VIDEO_ROTATE` builds) | Normal, CW, CCW |

A core that opts in puts the entries into its `CONF_STR` with `` `TANGO_VIDEO_CONF `` (no page prefix) or
`` `TANGO_VIDEO_CONF_P("P1") `` (never with `""`: an empty string literal is a NUL byte, which ends the `CONF_STR`),
outputs its native shape on `VIDEO_ARX/ARY` (what the menu calls "Original", e.g. 4:3), ties `VIDEO_SCALE` and
`VGA_SL` to 0, keeps its own entries below bit 118 and reads none of the framework's bits. Any script that takes the
`CONF_STR` out of the RTL must expand the macros: `build_tools/confstr_expand.py <core.v>` prints the string the
synthesiser builds.

The arcade Scale entry has two values, as MiSTer's arcade cores have no Scale entry and scale only with MiSTer.ini's
`vscale_mode` 0 / 1; a stale value 2 or 3 (a saved setting from another build) is shown and decoded as Normal.

## Rotation

`TANGO_VIDEO_ROTATE` (arcade cores; needs `TANGO_VIDEO_DDR3`) rotates the picture by 90 degrees, clockwise or
counter-clockwise, chosen in the Video page (`O[119:118]`). As MiSTer's `screen_rotate`, `tango_video_rotate_ddr3.v`
writes each of the core's frames rotated into memory and ascal shows it with its frame-buffer reader (`o_fb_*`):

- the writer is a client of the DDR3's core port C0 (MiSTer's `DDRAM_*` shape), in `CLK_VIDEO`; the crossing into the
  DDR3's clock is inside the core port. It writes 32-bit pixels, the rotated address computed per pixel;
- four buffers, two in each of DDR3 banks 6 and 7, so the buffer being written is never in the bank ascal reads;
  ascal's own frames are in bank 4;
- a frame is marked ready only after a read from its buffer returns (the DDR3 executes in order, so every write of
  the frame has landed); a 64-entry FIFO absorbs the port's stalls, and on an overflow the frame is dropped whole,
  never shown torn. The input and output rates differ, so ascal repeats or skips a frame now and then, as on MiSTer;
- ascal's input keeps running unchanged, and in Normal ascal shows it as without the option; the geometry engine
  swaps the core's width and height and its aspect ratio while a rotated picture is shown
  (`tango_video_rot_geo.vh`).

The option claims the whole core port: a core that also wants `DDRAM_*` traffic of its own is not supported.
Without the option no rotation logic is built and the `CONF_STR` has no Rotation entry.

## The DDR3 path

`TANGO_VIDEO_DDR3` moves ascal's frame buffers from the SDRAM into the board's DDR3:

- **The IP.** The build generates Gowin's DDR3 Memory Interface IP (`ddr3/tango_ddr3_ip.tcl`, the configuration of
  Sipeed's DDR3 example for the board): both x16 chips as one 32-bit bus (1 GB), 1:4 clock ratio, 400 MHz memory
  clock (DDR3-800), CL 6, a 256-bit user port at 100 MHz (`clk_x1`). Its PLL (`board/console138k/pll_ddr3.v`) is
  placed at `PLL_L[0]`; pins and placement are in `board/console138k/tango_console138k_ddr3.cst`, its clocks in
  `tango_console138k_ddr3.sdc`. The IP's files are generated into the build and are not part of this repository.
- **`tango_ddr3.v`**: the scaler port V (`tango_ddr3_vport.v`: ascal's Avalon master, 128-bit, MiSTer's `N_DW` /
  `N_AW`), the core port C0 (`tango_ddr3_cport.v`: MiSTer's `DDRAM_*`, 64-bit Avalon with bursts, in the core's
  clock; built with `TANGO_DDRAM`) and the arbiter (`tango_ddr3_arb.v`) onto the IP's user interface, all in
  `clk_x1`. The arbiter and the core port are pipelined to close timing at 100 MHz.
- **Address map:** the scaler's buffers from `0x2000_0000` (bank 4); the core window `0x3000_0000-0x3FFF_FFFF`
  (banks 6 and 7), forced by the core port, so no core can reach the scaler's region.
- The SDRAM is then free: a core can take it with `TANGO_SDRAM`, or keep the framework's controller for ports A/B
  with `TANGO_NES_SDRAM`. No debug top exists for DDR3 builds yet (`TANGO_DBG`).

`ddr3/test/` is a standalone DDR3 bring-up test for the board (`build_ddr3_test.sh`): it measures the IP's
calibration, command FIFO, refresh stalls, latency and bandwidth and runs a full memory test, and reports on an HDMI
results screen (decoded from a capture by `decode_d1.py` / `d1_verdict.py`) and a UART line.

## Native-rate video (vsync_adjust)

By default the HDMI output runs at 60 Hz (1650 x 750 at 74.25 MHz) and ascal repeats or drops a frame whenever the
core's rate differs. As MiSTer's `vsync_adjust`, the framework can instead run the HDMI raster at the core's own rate:

- **Build-time totals** (`TANGO_HDMI_HTOTAL` / `_VTOTAL`, `TANGO_HDMI_VIC`): the raster is HTOTAL x VTOTAL clocks of
  the unchanged pixel clock, so the frame rate is 74.25e6 / (HTOTAL x VTOTAL). The active 1280 x 720 picture, the
  sync pulses and the data islands stay where they are; the extra pixels and lines are back porch. VIC 0 (no VIC)
  is for a raster that is not CEA's.
- **Runtime totals** (`TANGO_VSYNC_ADJUST`, `tango_vsync.v`): the framework measures the core's frame period on its
  `VGA_VS`, plans the totals H x V (H 1650..1720, V 740..790) closest to it, and switches every consumer of the raster
  at a frame boundary; a plan more than 244 ppm off, or outside the bounds the firmware gives, keeps the fixed raster.
  A new plan is made when the core's rate changes (e.g. 262 / 263-line frames). The firmware sets the mode with link
  command SYS `0x70` (its `Tango.ini` key `VSYNC_ADJUST`); `TANGO_VSYNC_DEFAULT` is the mode at power-up (1 native).
- **Pixel-clock presets** (`TANGO_HDMI_PLL=745875|75600`): 74.5875 MHz (27 MHz x (55 + 2/8) / 20) or 75.6 MHz
  (27 x 56 / 20) instead of 74.25 MHz, so that the totals can meet a core's rate to a few ppm. The audio clock
  generator, the 48 kHz strobe, the HDMI Audio Clock Regeneration N / CTS (exactly 48 kHz) and the SDC follow.
- **Retune at core start** (`TANGO_HDMI_RETUNE`, `tango_pll_retune.v`): the pixel clock is switched at run time
  between the preset (native) and 74.25 MHz (fixed: exactly 60.000 Hz) by SYS `0x70`. A retune is a clock pause:
  the pixel clock is gated off while the PLL re-locks (about 20 ms) and resumes at the new frequency; nothing in its
  domain is reset. The firmware waits for the retune to finish (reply bit 2).

## Video and audio processing

With `TANGO_VIDEO_PROC` the framework carries MiSTer's picture processing, and with `TANGO_AUDIO_FILTER` (implied by
`TANGO_VIDEO_PROC`) its audio filter. As on MiSTer, these settings belong to the firmware, not to the core's option
word: the BL616 reads MiSTer's files from the SD card, sends them over the link with MiSTer's command numbers, and
saves them per core.

| Block | Where | What |
|---|---|---|
| polyphase filters | ascal's coefficient port (`tango_vproc_regs.v` -> `ascal_wrap.v`) | MiSTer's Horizontal, Vertical, Scan and Interlace filter files; ascal at MiSTer's FRAC 8 (256 phases per set, adaptive sets included), its coefficient memories in block RAM |
| gamma | `tango_gamma.v`, before the scaler | three 256 x 8 tables, as MiSTer's `gamma_corr` |
| shadow mask | `tango_shmask.v`, after the scaler | MiSTer's mask files (up to 16 x 16 cells, 1x / 2x, rotated); derived from MiSTer's `shadowmask.sv`, GPL-2.0-or-later |
| audio filter | `tango_audio_filter.v` | MiSTer's 3-tap IIR at the filter file's rate, the 48 kHz DC blocker, boost, stereo mix and global volume; derived from MiSTer's `iir_filter.v` and `audio_out.sv`, GPL-2.0-or-later |

**Link commands** (SYS target 0; the numbers are MiSTer's `UIO_*` except `0x70`; a core without the block answers 0):

| Command | Name | Data |
|---|---|---|
| `0x2A` | FLTCOEF | a coefficient set (H, V, H adaptive, V adaptive) of 2^n phases, 4 taps each |
| `0x2B` | FLTNUM | polyphase on/off, and whether the V set is the Scan filter (then the framework's scanline dimming is off); the reply reports the scanline level in use |
| `0x32` / `0x33` | SET_GAMMA / GAMCURV | gamma on/off / the R, G, B tables |
| `0x39` | SET_AFILTER | the core volume (boost, attenuation) and the filter's rate and coefficients; the load restarts the filter |
| `0x26` | AUDVOL | the global volume |
| `0x3E` | SHADOWMASK | MiSTer's mask command words |
| `0x70` | VSYNC_ADJUST | (`TANGO_VSYNC_ADJUST`) the mode (native / fixed) and the frame-period bounds; the reply reports the state |

**The files are not in any Tango release.** MiSTer's filter, gamma, shadow-mask and preset files have no licence
file, so Tango does not redistribute them. Get them from MiSTer-devel:
[Filters_MiSTer](https://github.com/MiSTer-devel/Filters_MiSTer) (scaler filters, gamma, audio filters),
[ShadowMasks_MiSTer](https://github.com/MiSTer-devel/ShadowMasks_MiSTer) and
[Presets_MiSTer](https://github.com/MiSTer-devel/Presets_MiSTer).

**Without the firmware** (JTAG tests) a build can carry its own load: `build_tools/vproc_preset.py` turns a MiSTer
preset (or single files) into the link transfers the firmware would send, as a logic ROM that `tango_vproc_regs.v`
replays after reset. Point it at the files with `--root` or `TANGO_FILTERS_ROOT`: either a card layout (`Filters/`,
`Gamma/`, `Shadow_Masks/`, `Filters_Audio/`, `Presets/`) or the three MiSTer-devel repositories cloned side by side.

`build_tools/ascal_poly_model.py` and `build_tools/iir_model.py` are bit-exact models of the scaler's polyphase
arithmetic and of the audio filter, which the cores' simulations compare with the RTL. `vproc_preset.py` (derived from
Main_MiSTer's file readers) is GPL-3.0-only and `iir_model.py` (derived from MiSTer's audio filter) GPL-2.0-or-later;
see [Licence](#licence).

## Controllers

Each USB-A port has its own host (`usb/usb_fs_host.v`, `usb_fs_phy.v`, `usb_ls_detect.v`, in a 48 MHz clock): it
detects the device's speed, runs **full speed (12 Mbit/s) or low speed (1.5 Mbit/s**, with keep-alives instead of
start-of-frame packets), enumerates the device and polls its interrupt-IN endpoint. Every report is CRC-checked, and
a pad's buttons count only after its reports have once read as all released.

- **DualShock 4** pads use a built-in decode (`tango_ds4_joy.v`).
- **Other pads**, full or low speed -- HID pads, and Xbox 360-class pads (vendor class FF/5D/01, which get their
  start-up reads) -- go through the table-driven decoder `tango_pad_decode.v`. The host stores the device, configuration and HID report
  descriptors; the firmware reads them, looks the pad up in its controller database and programs a decode table of up
  to 64 entries (field, test, output bit) for the port; the decoder evaluates it on every report, in the USB clock.
- **Keyboards** in the HID boot protocol become MiSTer's `ps2_key` events.

The firmware's menu reads the pads, "Define buttons" maps any pad button to any core button, and the firmware can
swap the two ports' joystick words (MiSTer's joyswap). `joystick_0` is the pad on USB-A2, `joystick_1` the pad on
USB-A1. Hubs are not supported.

## The shared SDRAM controller

`tango_sdram.v` drives the Console 138K's SDRAM module (one W9825G6KH-6: 32 MB, 4 banks x 8192 rows x 512 columns x
16 bit) at 96 MHz with CAS latency 3, one command per clock, with fixed priority: initialisation, refresh, port A,
port B, port C.

| Port | User | SDRAM area |
|---|---|---|
| A, B | a core's two real-time byte ports (`TANGO_NES_SDRAM`); 22-bit byte address: bank `{0, addr[21]}`, row `addr[20:10]`, column `addr[9:1]`, byte `addr[0]` | banks 0 and 1 (2 MB each) |
| C | ascal's frame buffers (Avalon, 64-bit beats, bursts), at `0x0140_0000`, three 4 MB buffers | banks 2 and 3 |

**What a core using ports A/B must do** (`tango_ab_slot.v`, `tango_core_ports.vh`):
- raise `oe` (read) or `we` (write) with a stable address (and `din`) and hold them until the slot's `clkref` clock;
  one request per rising edge, at most one per port per slot;
- pulse `clkref` for one `CLK_SYS` clock per slot, at a clock by which all of the slot's requests are raised (the
  NES: once per PPU dot);
- read `doutA` / `doutB` only after the latency: 9-17 `clk_sdram` clocks after the slot is taken at the NES's
  request rate, up to 27 when both ports ask in every slot and a refresh is forced. The byte stays until that port's
  next read;
- raise no request before `sdram_busy` falls (power-up initialisation, about 0.2 ms);
- add the two clock-crossing constraints below.

A core that only uses the ascal path (port C) adds nothing. A core with its own controller uses `TANGO_SDRAM`
instead and drives the pins itself (`tango_sdram_ports.vh`, pins in `board/console138k/tango_console138k_sdram.cst`).

The read data is taken on a falling edge 1.5 clocks after the CAS latency (`RD_CAP` 1), which works on this board for
a read round trip of 5.2-15.6 ns. The SDRAM pins use IOB registers; no input/output delays are constrained.

### A core that owns the SDRAM

Some cores need their own SDRAM controller, as MiSTer cores have, because their CPUs wait on that controller's timing
(`TANGO_SDRAM`). With `TANGO_ASCAL_EXT` (and `TANGO_VIDEO_ASCAL`) that controller also serves ascal: the core gets
ascal's Avalon master as ports (`ascal_avl_*`) and says which clock ascal's Avalon side runs on (`ASCAL_AVL_CLK`, its
memory clock). No `tango_sdram`, no SDRAM PLL and no `clk_sdram` constraint are built.

- **The protocol** is the one port C serves: one command at a time; 64-bit beats, word 0 in bits `[15:0]`; bursts of
  `ascal_avl_burstcount` beats that stay in one 1 KB row (ascal's are 32 beats, 256-byte aligned); a write brings
  beat 0 with the command; read beats in order, possibly after the next command was taken; `ascal_avl_waitrequest`
  high = not taken this clock. `tango_ascal_words.v` implements all of it and offers the core 16-bit word accesses,
  so the core's controller only schedules words.
- **The window** is the one the GHDL netlist bakes in: three 4 MB buffers from `0x0140_0000` (banks 2 and 3). The
  core keeps its own data out of it.
- **Clocks.** ascal synchronises its input side (`CLK_VIDEO`) to its Avalon clock and that to its output side itself.
  When `CLK_VIDEO` and `ASCAL_AVL_CLK` come from one PLL, the core's SDC times that crossing as a related-clock path
  (or bounds it with `set_max_delay`).
- **Bandwidth.** ascal moves (lines + 2) x ceil(width x 3 / 256) bursts of 256 bytes per output frame for reading and
  the same per input frame for writing: tens of MB/s for typical console resolutions.

## Timing constraints

The board SDC (`board/console138k/tango_console138k.sdc`) declares the board's clocks (`clk` 50 MHz, `clk27`,
`clk_pix` 74.25 MHz, `clk_ser` 371.25 MHz, `clk48`, `spi_sck`, `clk_audio`) and puts them in asynchronous groups;
every crossing between them is a synchroniser or a handshake. SDRAM builds add `clk_sdram`
(`tango_console138k_sdram108.sdc`): declared at 120 MHz to force a compact placement, and gated at its real 96 MHz
(`sta_gate.py --relax clk_sdram:96.0`, read from the SDC's comment by `build_core.sh`). DDR3 builds add `memory_clk`
(400 MHz) and `clk_x1` (100 MHz) as groups of their own (`tango_console138k_ddr3.sdc`). `TANGO_HDMI_PLL` builds get
the preset's exact `clk_pix` / `clk_ser` periods, and `TANGO_HDMI_RETUNE` builds add the PLL's ungated output
`clk_pix_raw` to `clk_pix`'s group.

**A core with clocks of its own** passes its SDC to `build_core.sh`. Gowin makes every clock that a multi-group
`set_clock_groups` does not list asynchronous to all others, and a later `set_clock_groups` cannot undo that, so the
core's SDC must carry the **one** `set_clock_groups` line of the build: every board group plus its own (`build_core.sh`
drops the board's line and refuses a core SDC without one).

**A core with SDRAM ports A/B** has two crossings between its `CLK_SYS` and `clk_sdram`, bounded rather than cut. Put
the two clocks in one group of that line, then, for example with a core clock `core_clk`:

```
set_max_delay -from [get_clocks {core_clk}] -to [get_clocks {clk_sdram}] 15
set_false_path -from [get_clocks {core_clk}] -to [get_clocks {clk_sdram}] -hold
set_max_delay -from [get_clocks {clk_sdram}] -to [get_clocks {core_clk}] 30
set_false_path -from [get_clocks {clk_sdram}] -to [get_clocks {core_clk}] -hold
```

The second value keeps the read latency plus the path inside the point where the core samples the byte.

**The STA gate** (`build_tools/sta_gate.py`) fails a build with any hold violation, a setup violation, a clock under
its constraint, a worst setup slack under +0.10 ns (a build that closed with less worked in the tool and failed on
the board), or logic or block RAM use over 98%.

## Building

You need, on Linux (x86-64):

- **Gowin EDA** (the IDE with `gw_sh`), V1.9.11.03, with a licence that covers the GW5AST-138 (and, for
  `TANGO_VIDEO_DDR3`, its DDR3 Memory Interface IP). `GOWIN_HOME` is the installation directory, the one that holds
  `IDE/` (default `~/gowin`). `build_core.sh` runs `IDE/bin/gw_sh` from it headless, with `QT_QPA_PLATFORM=minimal`
  and the system's `libfreetype` preloaded (`gw_sh` crashes with its bundled one on some distributions; the path is
  Debian/Ubuntu's). From the same installation the build takes two things this repository does not ship (they stay
  under the installation's licence: `NOTICE`):
  - **`PLL_INIT`**, `IDE/ipcore/PLL_ADV/data/PLL/pll_init.v`: the start-up calibration every PLL of the Console 138K's
    GW5AST-138 needs to lock. Every build adds it (the framework's PLL wrappers instantiate it, and a core's own PLL
    may too). `build_core.sh` prints the sha256 of the file it uses; the V1.9.11.03 one begins `3ef05a1b`.
    `sim/run_tb_retune.sh` and `ddr3/test/build_ddr3_test.sh` take it from the same place.
  - **The DDR3 Memory Interface IP** (`TANGO_VIDEO_DDR3` builds), generated into each build by `ddr3/tango_ddr3_ip.tcl`.

  If `GOWIN_HOME` has no `IDE/bin/gw_sh` or no `pll_init.v`, the scripts stop with an error that names the path.
- **Python 3** for the build tools.
- **GHDL 4** (with `--synth`; 4.1.0 made the shipped netlists) only to regenerate the ascal netlists
  (`ascal_ghdl/gen.sh`), which also needs **Verilator 5** for its lint and GNU `patch`.
- **Verilator 5** (5.020 tested) for the simulations, which compile these files with each core's testbench.
- **systemd** with user scopes for `build_tools/tango_capped.sh`, which runs every Gowin build (and the
  simulations) in a memory-capped cgroup, so a runaway job is killed instead of freezing the machine. A 138K build
  needs a few GB of memory; run one build or simulation at a time.

The framework is not built on its own: build a core (see its README). For example, in a core checkout:

    TANGO_NO_ARCHIVE=1 TANGO_VIDEO_ASCAL=1 TANGO_VIDEO_OPTS=1 TANGO_VIDEO_ASCAL_MAXW=384 TANGO_OUT=build/out \
        sys/board/console138k/build_core.sh tango_template_ascal138 - rtl/tango_core_template.v rtl/template_cos.vh

The bitstream is `build/out/<name>/impl/pnr/<name>.fs`; `build_tools/fs2bin.py <name>.fs <name>.bin` makes the `.bin`
the Tango firmware loads from the SD card. Never flash a build whose STA gate failed.

## Simulation benches

The cores' simulation gates compile the framework with their own testbenches. `sim/` holds the framework's own
benches for the blocks a core cannot exercise alone; each `run_tb_*.sh` builds with Verilator under
`tango_capped.sh` and prints a PASS / FAIL line:

| Script | What it checks |
|---|---|
| `sim/run_tb_tango_ddr3.sh`, `run_tb_tango_ascal_stream_ddr3.sh`, `run_tb_ascal_ddr3.sh`, `run_ddr3_matrix.py` | the DDR3 subsystem on a model of the IP's user interface (`sim/tango_ddr3_model.v`), ascal's real netlist streaming through it, and its margins |
| `sim/run_tb_ddr3_arb_equiv.sh`, `run_tb_cport_equiv.sh` | the timing-closed arbiter and core port, clock for clock against their earlier forms (`sim/ddr3_arb_ref/`, `sim/cport_ref/`), with mutants |
| `sim/run_tb_ddr3_test.sh` | the DDR3 bring-up test's logic on the IP model |
| `sim/run_tb_rotate.sh`, `run_tb_rot_scale.sh` | the rotation client (allocator, port traffic, a pixel-exact golden model) and the arcade Scale entry |
| `sim/run_tb_vsync_ddr3.sh`, `run_tb_hdmi_totals.sh`, `run_tb_retune.sh` | native-rate video, the HDMI raster at other totals, the pixel-clock retune |
| `sim/run_tb_ascal_ext.sh`, `run_tb_tango_ascal_words.sh` | `TANGO_ASCAL_EXT` and `tango_ascal_words.v` |
| `sim/run_tb_ul_lim_equiv.sh` | the upload read bound in `tango_link.v` |
| `sim/pads/run_tb_pad_decode.sh` | `tango_pad_decode.v` against vectors generated from the firmware's C reference decode, and its DS4 table against the built-in decode |

Some benches compare against files of the Template core or of the framework's own history and need those at hand
(see each script's header).

## Layout

| Path | What |
|---|---|
| `tango_sys.v` | the framework fabric: link, overlay, pads, core interface, video and audio around `tango_core` |
| `tango_core_ports.vh`, `tango_cfg.vh`, `tango_video_opts.vh`, `tango_sd_ports.vh`, `tango_sdram_ports.vh` | what a core includes: the port list, the build options, the Video page entries |
| `tango_link.v`, `tango_link_dbg_view.v` | the BL616 link slave; the debug build's on-screen link state |
| `tango_menu_overlay.v`, `tango_ov_geom.vh`, `tango_beam_lead.v`, `tango_beam_lead_rt.v` | the OSD renderer (`tango_ov_geom.vh` is generated by menu-tango's renderer model) |
| `tango_io.v`, `tango_cdc.v`, `tango_conf_rom.v`, `tango_sd.v` | the core-facing side: pads, option word, downloads and uploads, `CONF_STR`, block device |
| `tango_usb_port.v`, `tango_ds4_joy.v`, `tango_pad_decode.v`, `tango_desc_ram.v`, `usb/` | the USB hosts, the descriptor store and the pad / keyboard decoding |
| `tango_video.v`, `tango_video_geo.v`, `ascal_wrap.v`, `ascal_wrap_sim.v` | the ascal video path: geometry, the Video page, scanlines; `ascal_wrap_sim.v` is a behavioural stand-in for quick simulations |
| `tango_video_rotate_ddr3.v`, `tango_video_rot_geo.vh`, `tango_video_scale_arc.vh` | rotation and the arcade Scale entry |
| `tango_vsync.v`, `tango_pll_retune.v` | native-rate video: the raster planner and the pixel-clock retune |
| `ascal.vhd`, `ascal_ghdl/` | TEMLIB ascal, and the GHDL netlists the builds and simulations use (`gen.sh` regenerates them) |
| `tango_sdram.v`, `tango_ab_slot.v`, `tango_ascal_words.v` | the shared SDRAM controller, the A/B ports' core-clock front end, the word adapter for `TANGO_ASCAL_EXT` |
| `tango_ddr3.v`, `tango_ddr3_arb.v`, `tango_ddr3_vport.v`, `tango_ddr3_cport.v`, `tango_ddr3_fifo.v`, `ddr3/` | the DDR3 subsystem, the IP's generation script and the bring-up test |
| `tango_vproc_regs.v`, `tango_gamma.v`, `tango_shmask.v`, `tango_audio_filter.v` | MiSTer's video and audio processing |
| `hdmi/` | the HDMI encoder (hdl-util, modified; see `hdmi/README.md`) |
| `board/console138k/` | the board top, PLLs, pin constraints (`.cst`), SDCs, `build_core.sh` / `build_core.tcl` |
| `build_tools/` | `tango_capped.sh`, `sta_gate.py`, `fs2bin.py`, `confstr_expand.py`, `vproc_preset.py`, `ascal_poly_model.py`, `iir_model.py` |
| `sim/` | the framework's own simulation benches and models |

## Attributions

Third-party files, with their terms as the files state them (see `NOTICE`):

**Derived from MiSTer, under the GNU GPL** (each file's header names its source and quotes the terms):

| File | Derived from ([MiSTer-devel](https://github.com/MiSTer-devel), compared at Template_MiSTer `5dda373` / Main_MiSTer `c3dc8ed`) | Upstream copyright as stated | Licence (SPDX) |
|---|---|---|---|
| `tango_audio_filter.v`, `build_tools/iir_model.py` | Template_MiSTer `sys/iir_filter.v` (`IIR_filter`, `iir_filter_tap`, `DC_blocker`), `sys/audio_out.sv` (`aud_mix_top`) | "Copyright (C) 2020 Sorgelig"; "(c)2020-2026 Alexey Melnikov" | `GPL-2.0-or-later` (as both files: "version 2 of the License, or (at your option) any later version") |
| `tango_shmask.v` | Template_MiSTer `sys/shadowmask.sv` | none in the file | `GPL-2.0-or-later` (no header, no version stated; GPLv2 section 9: "If the Program does not specify a version number of this License, you may choose any version ever published by the Free Software Foundation") |
| `tango_video_geo.v` | Template_MiSTer `sys/video_freak.sv` (`video_freak`, `video_scale_int`), `sys/sys_top.v` lines 856-944 | "Copyright (c) 2020 Grabulosaure, (c) 2021 Alexey Melnikov"; "Copyright (c) 2021 Alexey Melnikov"; "(c)2017-2020 Alexey Melnikov" | `GPL-2.0-or-later` (video_freak.sv: "GPL Licensed", no version, so GPLv2 section 9; sys_top.v: version 2 or later) |
| `build_tools/vproc_preset.py` | Main_MiSTer `audio.cpp`, `video.cpp`, `file_io.cpp` | none in the files | `GPL-3.0-only` (no headers; Main_MiSTer's LICENSE is the GPL version 3) |

**The SDRAM pin locations** in `board/console138k/tango_console138k_sdram.cst` and **the BL616 link pin locations** in
`tango_console138k.cst` / `tango_console138k_dbg.cst` (all MIT) are those of Sipeed's Tang Console 138K schematic,
`Tang_Mega_138K_Console_32001C__Schematics.pdf` (Rev 1.3: the `SDRAM0_*` nets; the `JTAG.*` and `FPGA_UART.*` nets; from
[dl.sipeed.com](https://dl.sipeed.com/shareURL/TANG/Console/02_Schematic); the document states no licence or terms).
The link block's pulls, drives and `jtagseln` constraints are the Tango authors' settings, each with its reason (and
Gowin's documentation) cited in the file.

**Other third-party files:**

| Component | Files | Author | Terms |
|---|---|---|---|
| **ascal**, the "Avalon scaler" | `ascal.vhd` (byte-identical to `sys/ascal.vhd` of MiSTer-devel/Genesis_MiSTer at commit `adc0c42`); the GHDL netlists `ascal_ghdl/ascal_fixed*.v` (machine translations); `ascal_ghdl/ascal_poly_bram.patch` (our modification, applied to a copy) | TEMLIB (2018-2020), as carried and extended by the MiSTer project | ascal.vhd's own terms: "freely distributed and used for any purpose"; bugs and enhancements to info@temlib.org |
| **hdmi** | `hdmi/*.sv` (modified copies, see `hdmi/README.md`); `sim/hdmi_acr_sim.sv` (a simulation-only copy of one of them) | Sameer Puri, [hdl-util/hdmi](https://github.com/hdl-util/hdmi) | MIT (`hdmi/LICENSE-MIT`), offered by upstream as MIT or Apache-2.0 |
| **Gowin `PLL_INIT`** | not in this repository: `IDE/ipcore/PLL_ADV/data/PLL/pll_init.v` of the user's Gowin EDA installation, added to every build by `build_core.sh` (see [Building](#building)) | Gowin Semiconductor Corporation | the licence of the user's Gowin EDA |
| **Gowin DDR3 Memory Interface IP** | not in this repository: generated into each `TANGO_VIDEO_DDR3` build by `ddr3/tango_ddr3_ip.tcl` | Gowin Semiconductor Corporation | the licence of the user's Gowin EDA |

The PLL modules (`board/console138k/pll_c27.v`, `pll_c27_mod.v`, `pll_c48.v`, `pll_c48_mod.v`, `pll_c1485.v`,
`pll_c1485_mod.v`, `pll_ddr3.v`, `pll_ddr3_mod.v`, `gowin_pll_sdram.v`) are the Tango authors' (MIT): each instantiates
Gowin's PLL primitive with its settings, each parameter commented from Gowin's UG306 (Arora V Clock User Guide).

**Behaviour read and reimplemented** (the files above aside):

- **MiSTer-devel** ([Template_MiSTer](https://github.com/MiSTer-devel/Template_MiSTer), Main_MiSTer and the cores): the
  meanings of `hps_io` (joysticks, the option word, the menu mask, `ioctl` download and upload, `ps2_key`, the `sd_*`
  block device) and of the emu ports (including `DDRAM_*`); the `CONF_STR` conventions; `video_freak`'s crop,
  aspect-ratio and SCALE rules (in `tango_video_geo.v`, derived, above); `video_mixer`'s CE behaviour and the scanline
  levels; `sys_top`'s filter / gamma / mask / audio-filter commands, its frame-buffer wiring of ascal; `arcade_video`'s
  `screen_rotate` (the rotation); `vsync_adjust` (Main_MiSTer's `video.cpp` and `hps_io`'s frame-rate measurement);
  the arcade cores' Scale behaviour; `gamma_corr` (gamma); Main_MiSTer's file formats. Each file that follows MiSTer
  cites the MiSTer file and lines. MiSTer data in otherwise-MIT files, listed in `NOTICE`: the audio filter's default
  coefficients (`tango_vproc_regs.v`, from `sys_top.v`), the Video page's CONF_STR text (`tango_video_opts.vh`) and
  Main_MiSTer's `UIO_*` command numbers.
- **FPGA Companion** ([MiSTle-Dev/FPGA-Companion](https://github.com/MiSTle-Dev/FPGA-Companion), Apache-2.0): the
  SPI wire format of the BL616 link (targets, commands, replies), implemented from the way the Companion firmware
  uses it (`tango_link.v`); the Xbox 360-class pads' start-up reads (`usb/usb_fs_host.v`).
- **A2600Nano** ([MiSTle-Dev/A2600Nano](https://github.com/MiSTle-Dev/A2600Nano), GPL-3.0), an earlier reference
  only: its Console 138K build was the first to run the BL616 link and the JTAG hand-back on this board. The BL616 link
  block of `tango_console138k.cst` / `_dbg.cst` was first taken from it; it is now the Tango authors' own text (MIT,
  above), with the same values (the ones proven on silicon) and their sources in Sipeed's schematic and Gowin's
  documentation. The JTAG hand-back in `tango_sys_top138.v` (`jtagseln` = NOT `bl616_jtagsel`, with `NET_LOC`
  `V_JTAGSELN`) was first seen there; the file cites Gowin's documentation for it. The SD STATUS reply in `tango_link.v`
  follows the byte layout of its `sd_card.v`. Its HDL was otherwise read for behaviour only.
- **nestang** ([nand2mario/nestang](https://github.com/nand2mario/nestang), GPL-3.0): its Console 138K constraints
  were the earlier reference for the SDRAM module's ball map (now taken from Sipeed's schematic, above); ports A/B take
  requests the way NES cores of nestang and MiSTer do.
- **Sipeed**: the Tang Console 138K schematic (the USB-A pins, the SDRAM module's balls) and Sipeed's DDR3 example for the board (the DDR3
  IP's configuration, its PLL settings and the DDR3 pin constraints). **Winbond**: the W9825G6KH datasheet (every
  SDRAM timing rule in `tango_sdram.v` cites its section). **Gowin**: IPUG281 (the DDR3 IP's user interface) and
  UG306 (the PLL). **Bouffalo Lab**: the BL616 SDK (the SPI mode). The USB 2.0 and HID 1.11 specifications (the USB
  host). The HDMI 1.4 and CEA-861 specifications (in hdl-util's encoder). JEDEC JESD79-3 (the DDR3 model's burst
  order).

## What was ported or adapted

- **From MiSTer (derived, GPL):** `video_freak`'s placement rules (`tango_video_geo.v`), the shadow mask
  (`tango_shmask.v`), the audio filter and mix (`tango_audio_filter.v`, `iir_model.py`), the file readers of the
  build-time presets (`vproc_preset.py`).
- **From MiSTer (reimplemented from behaviour):** the core interface of `hps_io` and the emu ports, so a MiSTer core's
  `emu` maps onto `tango_core` almost signal for signal; the `CONF_STR` and option word; the Video page entries; the
  screen rotation; native-rate video; the filter and gamma processing, which load MiSTer's own files unchanged.
- **From TEMLIB / MiSTer (used as is):** the ascal scaler, through GHDL's netlist; one modification (the coefficient
  memories as synchronous RAMs, so they map to block RAM) is applied as a patch to a copy, and `ascal.vhd` stays
  byte-identical to MiSTer's.
- **From FPGA Companion (reimplemented):** the FPGA side of the BL616 link; Tango's OSD drawing commands, the pad
  decoder's commands and the upload commands are its own.
- **From hdl-util (vendored, modified):** the HDMI encoder, thinned to 720p and pipelined for timing, with the raster
  totals and the audio clock regeneration values as parameters.
- **From Gowin (not shipped):** `PLL_INIT` and the DDR3 Memory Interface IP, both taken from the user's Gowin
  installation at build time. The PLL modules are written from Gowin's documentation of the PLL primitive (UG306).

## Licence

MIT (`LICENSE`): Copyright (c) 2026 the Tango authors, **except**:

- the five files derived from MiSTer (see [Attributions](#attributions)): `tango_audio_filter.v`,
  `build_tools/iir_model.py`, `tango_shmask.v` and `tango_video_geo.v` (GPL-2.0-or-later),
  `build_tools/vproc_preset.py` (GPL-3.0-only);
- the third-party files listed in `NOTICE`, under their own terms (TEMLIB ascal, hdl-util's HDMI encoder).

Nothing of Gowin's is in this repository: `PLL_INIT` and the DDR3 IP come from the user's Gowin installation at build
time and stay under its licence (`NOTICE`, [Building](#building)).

Each source file's SPDX line says which licence applies; `NOTICE` lists the few files without one (generated or
hash-locked files, the third-party files) and the MiSTer data in otherwise-MIT files. The GPL texts are in `LICENSES/`
(`GPL-2.0.txt`, `GPL-3.0.txt`).

**Bitstreams:** a core built with the GPL files is a combined work under the GNU GPL, whatever the licence of the rest
of the core. `tango_video_geo.v` is in every build with a video path (`TANGO_VIDEO_FB` or `TANGO_VIDEO_ASCAL`), so in
practice in every core; `tango_audio_filter.v` is in every `TANGO_AUDIO_FILTER` build and `tango_shmask.v` in every
`TANGO_VIDEO_PROC` build. The HDL files from MiSTer are all GPL-2.0-or-later: with an MIT or GPL-2.0-or-later core the
bitstream is under GPL-2.0-or-later, with a GPL-3.0 core under GPL-3.0. `PLL_INIT` and the DDR3 IP, which a build takes
from the user's Gowin installation, keep Gowin's terms (`NOTICE`). `vproc_preset.py` and `iir_model.py` are host tools and are
not part of a bitstream; a `TANGO_VPROC_PRESET` build contains `vproc_preset.py`'s generated output, not its code.
