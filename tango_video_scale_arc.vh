// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_video_scale_arc.vh -- the Scale value an ARCADE build (TANGO_VIDEO_ROTATE) gives tango_video_geo
// (README "Rotation"). `include'd INSIDE tango_video, only under `ifdef TANGO_VIDEO_ROTATE (a function, not
// a module: no new file for the build lists; build_core.sh copies sys/*.vh into the include directory), and by
// sim/tb_rot_scale.v, so the bench runs this exact function.
//
// Arcade builds offer Scale = Normal / V-Integer only (tango_video_opts.vh TANGO_VO_E_SCALE_ARC: MiSTer's arcade
// cores have no Scale entry and no video_freak; their only integer scaling is MiSTer.ini vscale_mode 0/1, Main_MiSTer
// video.cpp 3073-3108). status[125:124] can still hold 2 or 3: a CFG saved by an older build (4-value list) is
// loaded whole. The BL616 firmware shows such an out-of-range value as value 0, so the picture must be Normal too:
// 2 and 3 decode as 0. 0 and 1 pass through unchanged.
//==============================================================================
function [1:0] scale_arc(input [1:0] s);
    scale_arc = s[1] ? 2'd0 : s;
endfunction
