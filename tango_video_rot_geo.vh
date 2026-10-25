// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_video_rot_geo.vh -- what tango_video's geometry engine (tango_video_geo) must see while the rotation
// framebuffer is on screen (README "Rotation"). `include'd INSIDE tango_video (a function, not a module:
// no new file for the build lists; build_core.sh copies sys/*.vh into the include directory).
//
// When ascal shows the rotated framebuffer (rot = tango_video_rotate_ddr3's o_fb_ena, CW or CCW), it scales
// fb_hsize x fb_vsize (the ROTATED size, native height x native width) into the window the engine computes, so
// the engine must compute that window for the rotated picture:
//   - size: hs/vt = the framebuffer's own width/height (rot_fb_hsize / rot_fb_vsize), not the native raster's;
//   - aspect: the core's VIDEO_ARX and VIDEO_ARY swapped, whole 13-bit values, so both the ratio form and the
//     bit-12 "size" form swap (tango_video_geo: bit 12 = a size in pixels). MiSTer does the same per core:
//     Arcade-Galaga.sv 199-200 `VIDEO_ARX = (!ar) ? (status[2] ? 2880 : 2191) : (ar-1); VIDEO_ARY = (!ar) ?
//     (status[2] ? 2191 : 2880) : 0` -- only the core's own ratio swaps; the other choices do not (in Tango
//     those are the framework's Aspect options 4:3 / 16:9 / Full Screen, applied after this, unswapped).
// With rot = 0 the result is the inputs, bit for bit (Normal unchanged).
// opt layout = tango_video's opt_in: {vga_sl[1:0], arx[12:0], ary[12:0], scale[2:0], crop_size[11:0], crop_off[4:0]}
//==============================================================================
function [71:0] rot_geo_in(input rot, input [11:0] hs, input [11:0] vt,
                           input [11:0] fb_hsize, input [11:0] fb_vsize, input [47:0] opt);
    rot_geo_in = rot ? {fb_hsize, fb_vsize, opt[47:46], opt[32:20], opt[45:33], opt[19:0]}
                     : {hs, vt, opt};
endfunction
