// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_video_opts.vh -- the framework's Video options: Aspect ratio, Scale and Scanlines as CONF_STR entries that
// sys-tango owns.  A core that opts in (build option TANGO_VIDEO_OPTS, with TANGO_VIDEO_ASCAL) puts the entries into
// its CONF_STR with one macro below, outputs only its native picture shape ("Original") on VIDEO_ARX/ARY, and ties
// VIDEO_SCALE and VGA_SL to 0: tango_video.v reads these status bits itself and does the rest.  No core wires the
// three choices any more (per-core wiring once read Scale and the scanlines from the wrong status bits;
// check_confstr.py now finds that).  Included by tango_core_ports.vh, so every core has the macros.
//
// THE STATUS BITS sys owns: status[127:118] (TANGO_VO_HI..TANGO_VO_LO), the top 10 bits of the option word.  A core's
// own CONF_STR entries stay below bit 118, and its RTL reads none of these bits: menu-tango tools/check_confstr.py
// fails either.  Why this range (a survey of the cores' CONF_STRs, repeated for the Rotation bits below):
//   - Tango cores: the highest bit any core's own entry uses is 61 (the TG16 core); the only one above was
//     the Template's old "O[122:121],Aspect ratio" (copied from Template_MiSTer 19e89ce Template.sv 52, 61), which
//     this replaced.  The Genesis (55), Template (47) and Atari 2600 (22) cores all stay well clear too.
//   - MiSTer cores fill the word from bit 0; Template_MiSTer keeps only its Aspect ratio at the top (122:121).  Highest
//     other bits seen: N64_MiSTer O[106] and status[107], PSX_MiSTer O[92], NES_MiSTer O[72:70]; SNES, Gameboy,
//     Genesis, MegaDrive, Saturn and Arcade-Cave stay below 82.  So a ported core keeps its own bit numbers.
//
//   O[127:126]  Aspect ratio  Original (the core's VIDEO_ARX/ARY), Full Screen (0:0), 4:3, 16:9
//   O[125:124]  Scale         Normal, V-Integer, Narrower HV-Integer, Wider HV-Integer (video_freak's SCALE 0-3)
//               ARCADE builds (TANGO_VIDEO_ROTATE: the arcade cores) get Normal, V-Integer only
//               (TANGO_VO_E_SCALE_ARC), as on MiSTer: MiSTer's arcade cores have no
//               Scale entry and no video_freak (Arcade-Galaga_MiSTer Arcade-Galaga.sv 203-226 CONF_STR); their only
//               integer scaling is MiSTer.ini vscale_mode 0/1 (Main_MiSTer video.cpp 3073-3108: vscale_mode 1 = integer
//               vertical scale of the ROTATED height, width from the aspect) = Tango's Normal / V-Integer.  Console
//               builds keep the 4-value entry, byte for byte.  tango_video.v also decodes a stale 2/3 (a CFG saved from
//               an older build) as Normal in arcade builds: the BL616 shows an out-of-range value as value 0
//               (menu-tango bl616/tango/tango_coremenu.c 163), so the picture must match what the menu says.
//   O[123:122]  Scanlines     None, 25%, 50%, 75% (MiSTer's VGA_SL 0-3: scanlines.v's 25 / 50 / 75 % reduction)
//   O[121:120]  reserved for a later Filter entry (ascal's interpolation); nothing reads them yet
//   O[119:118]  Rotation      Normal, CW, CCW (3rd encoding reserved).
//     TANGO_VO_B_ROTATE / TANGO_VO_E_ROTATE are defined below. `TANGO_VIDEO_CONF / `TANGO_VIDEO_CONF_P add the
//     entry ONLY when the core's own build also defines `TANGO_VIDEO_ROTATE (rotation is for the arcade cores,
//     never console-style cores). A core that defines TANGO_VIDEO_OPTS but not
//     TANGO_VIDEO_ROTATE gets the exact same three-entry CONF_STR text it always has: the Rotate entry is
//     textually absent from its menu, not merely inert, so a console core's CONF_STR is byte-for-byte
//     unchanged by this addition (a console-identity simulation checks it).
//     The bits themselves (status[119:118]) are consumed only inside tango_sys.v's own `ifdef
//     TANGO_VIDEO_ROTATE block (the CDC into tango_video_rotate_ddr3.v), which no console core's build
//     defines either -- so even the RTL read, not just the menu text, is gated the same way.
// "Full Screen" is 0:0 as Template_MiSTer (Template.sv 54-55: ar 1 -> ARX 0, ARY 0); 4:3 and 16:9 are literal values
// where MiSTer offers [ARC1] / [ARC2] (MiSTer.ini custom ratios, which Tango has no ini for; the BL616 firmware hides
// those two values: menu-tango bl616/tango/tango_coremenu.c value_ok).
//
// IN THE CORE'S CONF_STR -- each macro is a comma-separated list of whole entries (string literals):
//     `TANGO_VIDEO_CONF,                the entries, no prefix (they land on the page the list is on: page 0)
//     `TANGO_VIDEO_CONF_P("P1"),        each entry with the prefix "P1": on page 1 (any H#/D#/P# prefix works)
//   e.g. a Video page:
//     "P1,Video;",
//     `TANGO_VIDEO_CONF_P("P1"),
//     "-;",
//   Never `TANGO_VIDEO_CONF_P(""): an empty string literal is one NUL byte (Verilator 5.020: {"", "A"} is 16 bits,
//   16'h0041), and the firmware stops reading CONF_STR at a NUL.  Scripts that take the CONF_STR out of the RTL must
//   expand the macros: build_tools/confstr_expand.py does (check_confstr.py uses it).
// In a TANGO_VIDEO_ASCAL / TANGO_VIDEO_FB build without TANGO_VIDEO_OPTS the macros name an undeclared identifier,
// so a core whose menu has the entries but whose build would ignore them does not compile.  In a build with no
// scaler (the 720p beam, VID_CX / VID_CY / VID_RGB) they are ordinary entries: the core may read them itself, with
// an allow comment for the lint (check_confstr.py: "// check_confstr: status[127:126] <why>").
//==============================================================================
`ifndef TANGO_VIDEO_OPTS_VH
`define TANGO_VIDEO_OPTS_VH
// the bits sys owns (check_confstr.py reads these two)
`define TANGO_VO_HI  127
`define TANGO_VO_LO  118
// each entry: TANGO_VO_B_<x> = its bits (tango_video.v decodes status[`TANGO_VO_B_<x>]), TANGO_VO_E_<x> = the entry,
// written with the same bits (check_confstr.py checks the firmware parses E to exactly B).  Menu order = file order.
`define TANGO_VO_B_ASPECT  127:126
`define TANGO_VO_E_ASPECT  "O[127:126],Aspect ratio,Original,Full Screen,4:3,16:9;"
`define TANGO_VO_B_SCALE   125:124
`define TANGO_VO_E_SCALE   "O[125:124],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;"
// arcade builds (TANGO_VIDEO_ROTATE): the same bits, Normal / V-Integer only (see the O[125:124] note above).
// TANGO_VO_B_SCALE_ARC must exist and equal TANGO_VO_B_SCALE: build_tools/confstr_expand.py pairs every TANGO_VO_E_<x>
// with TANGO_VO_B_<x>, and check_confstr.py FAIL (5)s an entry named "Scale" whose pair has no bits.
`define TANGO_VO_B_SCALE_ARC 125:124
`define TANGO_VO_E_SCALE_ARC "O[125:124],Scale,Normal,V-Integer;"
`define TANGO_VO_B_SL      123:122
`define TANGO_VO_E_SL      "O[123:122],Scanlines,None,25%,50%,75%;"
// O[121:120] stays reserved for a later Filter entry: no TANGO_VO_B_/E_ pair yet, nothing reads it, check_confstr.py
// still fails any core entry or read that touches it (it's inside TANGO_VO_HI..TANGO_VO_LO either way).
// Rotation: bits reserved, entry defined, added to
// `TANGO_VIDEO_CONF/`TANGO_VIDEO_CONF_P ONLY under `ifdef TANGO_VIDEO_ROTATE (below) -- a core that defines
// TANGO_VIDEO_OPTS without TANGO_VIDEO_ROTATE (every console-style core) gets the
// unchanged 3-entry list, not a 4th dead entry.
`define TANGO_VO_B_ROTATE  119:118
`define TANGO_VO_E_ROTATE  "O[119:118],Rotation,Normal,CW,CCW;"
// the lists a core puts in its CONF_STR (the TANGO_VO_E_* above, in that order). TANGO_VIDEO_ROTATE's presence
// (not TANGO_VIDEO_OPTS's) decides whether the Rotate entry is in the list, and whether Scale is the arcade
// 2-value entry (TANGO_VO_E_SCALE_ARC) or the console 4-value one -- see the O[125:124] note above:
// this is the ONLY place in the framework that must stay gated this way, so no console
// core (which never defines TANGO_VIDEO_ROTATE) can ever reach the entry, by construction, not by convention.
`ifdef TANGO_VIDEO_OPTS
 `ifdef TANGO_VIDEO_ROTATE
 `define TANGO_VIDEO_CONF       `TANGO_VO_E_ASPECT, `TANGO_VO_E_SCALE_ARC, `TANGO_VO_E_SL, `TANGO_VO_E_ROTATE
 `define TANGO_VIDEO_CONF_P(PX) {PX, `TANGO_VO_E_ASPECT}, {PX, `TANGO_VO_E_SCALE_ARC}, {PX, `TANGO_VO_E_SL}, {PX, `TANGO_VO_E_ROTATE}
 `else
 `define TANGO_VIDEO_CONF       `TANGO_VO_E_ASPECT, `TANGO_VO_E_SCALE, `TANGO_VO_E_SL
 `define TANGO_VIDEO_CONF_P(PX) {PX, `TANGO_VO_E_ASPECT}, {PX, `TANGO_VO_E_SCALE}, {PX, `TANGO_VO_E_SL}
 `endif
`elsif TANGO_VIDEO_ASCAL
`define TANGO_VIDEO_CONF       tango_video_conf_needs_TANGO_VIDEO_OPTS
`define TANGO_VIDEO_CONF_P(PX) tango_video_conf_needs_TANGO_VIDEO_OPTS
`elsif TANGO_VIDEO_FB
`define TANGO_VIDEO_CONF       tango_video_conf_needs_TANGO_VIDEO_OPTS
`define TANGO_VIDEO_CONF_P(PX) tango_video_conf_needs_TANGO_VIDEO_OPTS
`else
 `ifdef TANGO_VIDEO_ROTATE
 `define TANGO_VIDEO_CONF       `TANGO_VO_E_ASPECT, `TANGO_VO_E_SCALE_ARC, `TANGO_VO_E_SL, `TANGO_VO_E_ROTATE
 `define TANGO_VIDEO_CONF_P(PX) {PX, `TANGO_VO_E_ASPECT}, {PX, `TANGO_VO_E_SCALE_ARC}, {PX, `TANGO_VO_E_SL}, {PX, `TANGO_VO_E_ROTATE}
 `else
 `define TANGO_VIDEO_CONF       `TANGO_VO_E_ASPECT, `TANGO_VO_E_SCALE, `TANGO_VO_E_SL
 `define TANGO_VIDEO_CONF_P(PX) {PX, `TANGO_VO_E_ASPECT}, {PX, `TANGO_VO_E_SCALE}, {PX, `TANGO_VO_E_SL}
 `endif
`endif
`endif
