#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
"""ascal_poly_model.py -- a model of what TEMLIB ascal (sys/ascal.vhd, as GHDL builds it: sys/ascal_ghdl/; FRAC 8 in the
TANGO_VIDEO_PROC builds (ascal_fixed8.v, the coefficient memories patched to block RAM: the same arithmetic), FRAC 4
in the others)
draws for a static source frame in direct-input mode, nearest (mode 000) or polyphase (mode 100), and of the video
stages Tango puts after it (tango_video.v: scanline dimming, tango_shmask.v).  Our own code, written from reading
ascal.vhd (line numbers of this repository's copy), as the reference of the video-processing sims.

Geometry (one axis; the same for both, ascal 1980-1997 (V), 2130-2175 (H)):
  output pixel k of the picture window (0 .. size-1, size = max - min + 1), input size isize:
    N = size + isize + 2 k isize        (H: o_hacc_ini = hsize + ihsize, one step per output pixel; V: o_vacc_ini =
                                         vsize - ivsize, and the line on screen uses the step made at the end of the
                                         line before, so one step more: the same N)
    C = N div 2 size, acc = N mod 2 size
    window = source pixels / lines C-2, C-1, C, C+1, each clamped to 0 .. isize-1 (ascal 2226-2235: the first pixel
             is repeated, the last held; 2780-2797: the last line repeated)
    fraction: H FRAC bits, floor(2^FRAC acc / 2 size) (the pipelined divider computes FRAC bits, 2442-2526);
              V 12 bits, floor(4096 acc / 2 size) (1778-1805); the phase is its top FRAC bits (2287-2297)
  i.e. the window's taps 1 and 2 surround the source position (k + 0.5) isize / size - 0.5.
Arithmetic (959-1066): taps t0..t3 signed 10 bit, 256 = 1.0; nearest = taps {0,256,0,0} below half a pixel, else
  {0,0,256,0} (poly_nn); poly_calc: (t0<<7) p0 + (t1<<7) p1 and (t2<<7) p2 + (t3<<7) p3, each >>> 8, summed in 19 bits,
  bound to 0..255 from bit 15 (712-722, 1026-1033).  Adaptive (2295-2352): with the set's adaptive flag the taps are
  ((a (256 - lum) + b lum) as 20 bits) bits 18:1, lum = max(R, G, B) of the nearer of taps 1 / 2 (poly_lum 1081-1112).
H first, into line buffers of 8-bit pixels; then V across four of them (2569-2614, 2865-2894).
"""

def clamp(i, n):
    return 0 if i < 0 else (n - 1 if i >= n else i)

def s_(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v

def bound(a19):
    """ascal bound(a, 15) of a 19-bit vector"""
    a19 &= (1 << 19) - 1
    if a19 >> 18:
        return 0
    if a19 >> 15:
        return 255
    return (a19 >> 7) & 255

def poly_px(taps18, p):
    """poly_calc + poly_final on one colour: taps18 = 4 coefficients in 3.15 (18-bit signed), p = 4 pixels 0..255"""
    r0 = taps18[0] * p[0] + taps18[1] * p[1]
    r1 = taps18[2] * p[2] + taps18[3] * p[3]
    return bound((r0 >> 8) + (r1 >> 8))

def cvt(t):
    """poly_cvt: 10-bit signed tap -> 3.15"""
    return s_(t, 10) << 7

def lerp(a, b, lum):
    """poly_lerp of one tap: a (256 - lum) + b lum in 20 bits, bits 18:1 -> 18-bit signed"""
    v = (s_(a, 10) * (256 - lum) + s_(b, 10) * lum) & ((1 << 20) - 1)
    return s_((v >> 1) & ((1 << 18) - 1), 18)

def nn_set(frac):
    """poly_nn for the 2^FRAC phases (1069-1078): the top bit of the fraction picks tap 1 or tap 2"""
    h = 1 << (frac - 1)
    return [(0, 256, 0, 0)] * h + [(0, 0, 256, 0)] * h

def axis(size, isize, frac_bits, phase_bits):
    """per output pixel k: (window indices, phase 0 .. 2^phase_bits - 1, top fraction bit); frac_bits = the
    fraction's precision (H: FRAC, V: 12), phase_bits = FRAC"""
    out = []
    for k in range(size):
        n = size + isize + 2 * k * isize
        c, acc = divmod(n, 2 * size)
        f = (acc << frac_bits) // (2 * size)
        f12 = f << (12 - frac_bits)
        out.append(([clamp(c + d, isize) for d in (-2, -1, 0, 1)], f12 >> (12 - phase_bits), f12 >> 11))
    return out

def taps_for(phase_set, adapt_set, phase, near_pixel, nn):
    """the 3.15 taps for one pixel: the set's phase (or the NN taps nn if phase_set is None), blended with the
    adaptive set by the near pixel's luminance when adapt_set is given"""
    t = (nn if phase_set is None else phase_set)[phase]
    if adapt_set is None:
        return [cvt(x) for x in t]
    lum = max(near_pixel)
    b = adapt_set[phase]
    return [lerp(t[i], b[i], lum) for i in range(4)]

def scale(src, iw, ih, win, poly=False, hset=None, vset=None, hadapt=None, vadapt=None, frac=4):
    """src[y][x] = (r, g, b), iw x ih; win = (hmin, hmax, vmin, vmax) on the 1280 x 720 canvas.  poly False = mode
    000 (nearest through poly_nn); True = mode 100 with hset / vset (2^frac phases x 4 taps, 10-bit) and the
    optional adaptive set of one direction.  Returns the canvas rows (None outside the window: ascal's border
    colour 0).  frac: ascal's FRAC (4, or 8 in the TANGO_VIDEO_PROC builds)."""
    hmin, hmax, vmin, vmax = win
    hs, vs = hmax - hmin + 1, vmax - vmin + 1
    hax, vax = axis(hs, iw, frac, frac), axis(vs, ih, 12, frac)
    NN = nn_set(frac)
    hS = hset if poly else None
    vS = vset if poly else None
    hA = hadapt if poly else None
    vA = vadapt if poly else None
    # H pass over every source line
    lines = []
    for y in range(ih):
        row = src[y]; o = []
        for idx, ph, nn in hax:
            p = [row[i] for i in idx]
            t = taps_for(hS, hA, ph, p[2] if nn else p[1], NN)
            o.append(tuple(poly_px(t, [q[c] for q in p]) for c in range(3)))
        lines.append(o)
    # V pass
    out = [[None] * 1280 for _ in range(720)]
    for j, (idx, ph, nn) in enumerate(vax):
        L = [lines[i] for i in idx]
        oy = vmin + j
        for k in range(hs):
            p = [L[i][k] for i in range(4)]
            t = taps_for(vS, vA, ph, p[2] if nn else p[1], NN)
            out[oy][hmin + k] = tuple(poly_px(t, [q[c] for q in p]) for c in range(3))
    return out

def dim(v, s):
    """tango_video.v dim(): MiSTer scanlines.v's 75 / 50 / 25 %"""
    return [v, (v >> 1) + (v >> 2), v >> 1, v >> 2][s]

def shadow_mask(canvas, win, words):
    """tango_shmask.v on a canvas: words = the SYS 0x3E command words (MiSTer's).  Pixels outside the window are 0
    already (border), the mask multiplies them all the same."""
    hmax = vmax = 0; en = rot = x2 = 0; lut = [0] * 256; idx = 0
    for w in words:
        k = w >> 13
        if k == 0: en, rot, x2 = (w >> 3) & 1, (w >> 2) & 1, (w >> 1) & 1; idx = 0
        elif k == 1: vmax = w & 15
        elif k == 2: hmax = w & 15
        elif k == 3: lut[idx & 255] = w & 0x7FF; idx += 1
    if not en:
        return canvas
    hp = (((vmax if rot else hmax) << x2) | x2)
    vp = (((hmax if rot else vmax) << x2) | x2)
    hmin, _, vmin, _ = win
    def mul(v, m):
        lo = ((v >> 4) if m & 1 else 0) + ((v >> 3) if m & 2 else 0)
        hi = ((v >> 2) if m & 4 else 0) + ((v >> 1) if m & 8 else 0) + (v if m & 16 else 0)
        s = lo + hi
        return 255 if s >= 256 else s
    out = []
    for y, row in enumerate(canvas):
        vc = (y - vmin) % (vp + 1) if y >= vmin else None
        o = []
        for x, px in enumerate(row):
            if px is None or vc is None or x < hmin:
                o.append(px); continue
            hc = (x - hmin) % (hp + 1)
            hi_ = (hc >> 1) if x2 else hc & 15
            vi_ = (vc >> 1) if x2 else vc & 15
            e = lut[(hi_ << 4 | vi_) if rot else (vi_ << 4 | hi_)]
            ms = [(16 | ((e >> 4) & 15)) if (e >> (10 - c)) & 1 else (e & 15) for c in range(3)]
            o.append(tuple(mul(px[c], ms[c]) for c in range(3)))
        out.append(o)
    return out
