#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 the Tango authors
# Copyright (C) 2020 Sorgelig (sys/iir_filter.v)
# (c)2020-2026 Alexey Melnikov (sys/audio_out.sv)
#
# This file is derived from MiSTer's audio filter and mix: the MiSTer project's Template_MiSTer
# (https://github.com/MiSTer-devel/Template_MiSTer, compared at commit 5dda373), sys/iir_filter.v (modules IIR_filter,
# iir_filter_tap, DC_blocker) and sys/audio_out.sv (module aud_mix_top), whose arithmetic it models step by step.
# Both files state: "This program is free software; you can redistribute it and/or modify it under the terms of the
# GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at
# your option) any later version."  This file is under the same terms (GPL-2.0-or-later; LICENSES/GPL-2.0.txt,
# LICENSES/GPL-3.0.txt), not under the MIT licence of the rest of sys-tango; see NOTICE.
# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
"""iir_model.py -- the bit-exact model of tango_audio_filter.v (README "Video and audio processing"): MiSTer's 3-tap IIR
arithmetic (Template_MiSTer 5dda373 sys/iir_filter.v 52-97, 181-208) on Python integers,
the 48 kHz hand-over and MiSTer's DC blocker (iir_filter.v 211-235), with the RTL's own scheduling.

  iir_model.py check <alog> [--json <tables.json>]   (+ the mix stage: MiSTer's aud_mix_top, audio_out.sv 281-345)
      replays a tb_vproc audio log (sim/tb_vproc.v +alog=...): the coefficients each stretch runs with are the log's
      own K records (after each reset), checked against --json's at the end; every step's inputs through the model IIR, compared
      with every output the RTL wrote (W records); every 48 kHz hand-over (H) against the model's held outputs; every
      HDMI-side output (O) against the model's DC blocker fed the H values.  Exit 0 only if all are equal.
  iir_model.py check <alog> --keep-taps : the same with a filter load that does NOT clear the taps (the previous filter's
      state carried into the new coefficients) -- a defect an earlier build had on silicon: replays a bench built
      with that defect bit for bit.
  iir_model.py spectrum <alog> : per stretch between filter loads, the IIR's output (from 5 ms after the load) and the
      HDMI-side output (after the 8192-sample mute): RMS, peak and the strongest lines (left channel).
  iir_model.py response <coefficient json> : the filter's frequency response from its coefficients (for the docs).
"""
import json, math, struct, sys

M40 = (1 << 40) - 1

def s(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v

def xs(c, x):
    """x's scales (iir_filter.v 166-175): bits 0-2 add x>>>4, x>>>3, x>>>2; bit 7 = one's complement"""
    x = s(x, 40)
    a = 0
    if c & 1: a += x >> 4
    if c & 2: a += x >> 3
    if c & 4: a += x >> 2
    a &= M40
    return (~a) & M40 if c & 0x80 else a

class IIR:
    def __init__(self, cx, cx0, cx1, cx2, cy0, cy1, cy2):
        self.cx = s(cx, 40); self.cxk = (cx0 & 0xFF, cx1 & 0xFF, cx2 & 0xFF)
        self.cy = (s(cy0, 24), s(cy1, 24), s(cy2, 24))
        self.reset()

    def reset(self):
        self.t = [[0, 0, 0], [0, 0, 0]]          # per channel t0, t1, t2 (40-bit, as unsigned)

    def step(self, ch, inp):
        """one filter step of channel ch with the 16-bit signed sample inp; returns the 16-bit output (unsigned)"""
        t0, t1, t2 = self.t[ch]
        p = s(inp, 16) * self.cx                 # 16 x 40 signed
        x = (p >> 20) & M40                      # bits 59:20 of the 60-bit product
        y = (x + t0) & M40
        ys = s(y, 40)
        hi = (y >> 35) & 0xF                     # y[38:35]
        if not (y >> 39) and hi:
            yc = 0x7FFF
        elif (y >> 39) and hi != 0xF:
            yc = 0x8000
        else:
            yc = (y >> 20) & 0xFFFF
        y37 = s(y, 37)                           # y[36:0] as signed
        m = [((y37 * c) >> 21) & M40 for c in self.cy]
        n0 = (xs(self.cxk[0], x) - m[0] + t1) & M40
        n1 = (xs(self.cxk[1], x) - m[1] + t2) & M40
        n2 = (xs(self.cxk[2], x) - m[2]) & M40
        self.t[ch] = [n0, n1, n2]
        return yc

class DC:
    """MiSTer's DC blocker at 48 kHz (x - x>>>10, y - y>>>9), as tango_audio_filter.v stages A / B"""
    def __init__(self):
        self.x1 = 0; self.y = 0

    def step(self, din):
        x = s(din, 16) << 23                     # {din[15], din, 23'd0}
        x0 = (x - (x >> 10)) & M40
        ys = s(self.y, 40)
        y1 = (ys - (ys >> 9)) & M40
        y0 = (x0 - self.x1 + y1) & M40
        b39, b38 = (y0 >> 39) & 1, (y0 >> 38) & 1
        if b39 != b38:
            y0 = (b39 << 39) | (b39 << 38) | (((1 << 38) - 1) if b38 else 0)
        self.x1 = x0; self.y = y0
        return (y0 >> 23) & 0xFFFF               # y[38:23]

# MiSTer's aud_mix_top (audio_out.sv 281-345), as tango_audio_filter.v runs it: both channels, one stage per 48 kHz
# sample, each stage reading the registers' values from before the strobe (the DC blocker's output of the sample
# before).  boost_x / boost_b / log2 a / log2 f for (f, a) = (4, 2) and (8, 4):
BX = (((32767 * 3) // 7) + 1, ((32767 * 7) // 31) + 1)        # 14044, 7400
BB = (BX[0] * 2, BX[1] * 4)
BA = (1, 2); BF = (2, 3)

class Mix:
    def __init__(self):
        self.st = [dict(v0=0, s0=0, v1=0, s1=0, a1=0, a2=0, pre=0, a3=0, a4=0, out=0) for _ in (0, 1)]

    def step(self, dc, att, boost, mix):
        """one strobe: dc = the DC blocker outputs (16-bit) the stages read; returns the outputs (16-bit) after it"""
        i = 1 if boost & 2 else 0
        new = []
        for c in (0, 1):
            S, O = self.st[c], self.st[1 - c]
            d = dc[c] & 0xFFFF
            n = {}
            n['v0'] = ((~d) + 1) & 0xFFFF if d >> 15 else d
            n['s0'] = d >> 15
            if S['v0'] < BX[i]:
                n['v1'] = (S['v0'] << BA[i]) & 0xFFFF
            else:
                n['v1'] = ((((S['v0'] - BX[i]) & 0xFFFF) >> BF[i]) + BB[i]) & 0xFFFF
            n['s1'] = S['s0']
            if boost:
                n['a1'] = s((~(S['v1'] - 1)) & 0xFFFF, 16) if S['s1'] else s(S['v1'], 16)
            else:
                n['a1'] = s(d, 16)
            n['a2'] = S['a1']                                   # + Linux audio: none
            n['pre'] = S['a2'] >> 1                             # a2[16:1]
            a2, pin = S['a2'], O['pre']
            if mix == 0:   a3 = a2
            elif mix == 1: a3 = a2 - (a2 >> 3) + (pin >> 2)
            elif mix == 2: a3 = a2 - (a2 >> 2) + (pin >> 1)
            else:          a3 = (a2 >> 1) + pin
            n['a3'] = s(a3, 17)
            n['a4'] = 0 if att & 16 else s(S['a3'] >> (att & 15), 17)
            a4 = S['a4'] & 0x1FFFF
            n['out'] = (0x7FFF if not (a4 >> 16) else 0x8000) if ((a4 >> 16) ^ (a4 >> 15)) & 1 else a4 & 0xFFFF
            new.append(n)
        self.st = new
        return [new[0]['out'], new[1]['out']]

def read_log(path):
    """tb_vproc.v's text log: '<tag> <a hex> <b hex>' per line (streamed: a long log is hundreds of MB)"""
    op = __import__('gzip').open if path.endswith('.gz') else open
    with op(path, 'rt') as f:
        for line in f:
            p = line.split()
            if len(p) == 3:
                yield (ord(p[0]), int(p[1], 16), int(p[2], 16))

def check(path, coeffs, keep_taps=False):
    recs = read_log(path)
    iir = IIR(coeffs['cx'], coeffs['cx0'], coeffs['cx1'], coeffs['cx2'], coeffs['cy0'], coeffs['cy1'], coeffs['cy2'])
    dc = [DC(), DC()]
    pend = []                                    # expected (channel, value) writes of the steps taken
    y = [0, 0]; h_prev = [0, 0]                  # the RTL's y_l / y_r; h_* (the blocker reads the one before)
    mute = 0; in_reset = False; kw = {}
    mix = Mix(); par = (0, 0, 0); dc_prev = [0, 0]; mix_out = [0, 0]
    stats = dict(S=0, W=0, H=0, O=0, Z=0, M=0, bad_W=0, bad_H=0, bad_O=0, bad_M=0, nonzero_O=0, nonzero_M=0)
    first_bad = []
    for tag, a, b in recs:
        if tag == 0x48:                          # H: the hand-over takes y now
            stats['H'] += 1
            if [a, b] != y:
                stats['bad_H'] += 1
                if len(first_bad) < 5: first_bad.append(('H', stats['H'], (a, b), tuple(y)))
            # the blocker's stage A on this strobe uses the value taken at the strobe before
            outs = [dc[c].step(h_prev[c]) for c in (0, 1)]
            h_prev = [a, b]
            pend_out = outs
        elif tag == 0x57:                        # W: channel a writes b
            stats['W'] += 1
            if not pend:
                stats['bad_W'] += 1
                if len(first_bad) < 5: first_bad.append(('W unexpected', stats['W'], (a, b)))
                continue
            ch, v, _ = pend.pop(0)
            if ch != a or v != b:
                stats['bad_W'] += 1
                if len(first_bad) < 5: first_bad.append(('W', stats['W'], (a, b), (ch, v)))
            y[a] = b
        elif tag == 0x53:                        # S: a step with inputs a (L), b (R)
            stats['S'] += 1
            before = [list(iir.t[0]), list(iir.t[1])]           # (a reset before the write drops the step's update)
            pend.append((0, iir.step(0, a), before[0]))
            pend.append((1, iir.step(1, b), before[1]))
        elif tag == 0x4F:                        # O: the HDMI-side output
            stats['O'] += 1
            exp = pend_out if mute & 0x2000 else [0, 0]
            if [a, b] != exp:
                stats['bad_O'] += 1
                if len(first_bad) < 5: first_bad.append(('O', stats['O'], (a, b), tuple(exp), mute))
            if a or b: stats['nonzero_O'] += 1
            if not in_reset and not (mute & 0x2000):
                mute += 1
            mix_out = mix.step(dc_prev, *par)                  # the mix stages read the sample before
            dc_prev = [a, b]
        elif tag == 0x50:                        # P: the mix stage's inputs now: att, {boost, mix}
            par = (a & 31, (b >> 2) & 3, b & 3)
        elif tag == 0x4D:                        # M: the final output (after the mix stage), one clock after O
            stats['M'] += 1
            if [a, b] != mix_out:
                stats['bad_M'] += 1
                if len(first_bad) < 5: first_bad.append(('M', stats['M'], (a, b), tuple(mix_out)))
            if a or b: stats['nonzero_M'] += 1
        elif tag == 0x5A:                        # Z: reset from this edge
            stats['Z'] += 1
            if not keep_taps:
                iir.reset()
            else:                                # the steps not yet written are dropped: their channels keep the taps
                seen = set()                     # from before the first of them
                for ch, _, bt in pend:
                    if ch not in seen: iir.t[ch] = bt; seen.add(ch)
            pend = []; y = [0, 0]; mute = 0; in_reset = True
        elif tag == 0x7A:
            in_reset = False; kw = {}
        elif tag == 0x4B:                        # K: a coefficient piece; the 14th completes the set the RTL now uses
            kw[a] = b
            if len(kw) == 14:
                coeffs = dict(rate=kw[0] << 16 | kw[1], cx=kw[2] << 32 | kw[3] << 16 | kw[4], cx0=kw[5], cx1=kw[6],
                              cx2=kw[7], cy0=kw[8] << 16 | kw[9], cy1=kw[10] << 16 | kw[11], cy2=kw[12] << 16 | kw[13])
                taps = iir.t
                iir = IIR(coeffs['cx'], coeffs['cx0'], coeffs['cx1'], coeffs['cx2'], coeffs['cy0'], coeffs['cy1'], coeffs['cy2'])
                if keep_taps: iir.t = taps
                stats['K'] = stats.get('K', 0) + 1
    ok = (stats['bad_W'] == 0 and stats['bad_H'] == 0 and stats['bad_O'] == 0 and stats['bad_M'] == 0 and stats['W'] > 0
          and stats['O'] > 0)
    last = {k: v & ((1 << 40) - 1) for k, v in coeffs.items() if k in ('rate', 'cx', 'cx0', 'cx1', 'cx2', 'cy0', 'cy1', 'cy2')}
    return ok, stats, first_bad, last

def spectrum(path):
    """per stretch between filter loads: the IIR's own output (H records: y at each 48 kHz hand-over, from 5 ms after
    the load, mute or not) and the HDMI-side output (O records, after the 8192-sample mute and 10 ms more)"""
    import numpy as np
    segs = []; cur = None; kw = {}
    for tag, a, b in read_log(path):
        if tag == 0x5A:                          # Z: a load starts
            cur = None
        elif tag == 0x4B:
            kw[a] = b
            if len(kw) == 14:
                c = dict(rate=kw[0] << 16 | kw[1], cx=kw[2] << 32 | kw[3] << 16 | kw[4], cx0=kw[5], cx1=kw[6], cx2=kw[7],
                         cy0=kw[8] << 16 | kw[9], cy1=kw[10] << 16 | kw[11], cy2=kw[12] << 16 | kw[13])
                cur = dict(coeffs=c, h=[], o=[], nh=0, no=0); segs.append(cur); kw = {}
        elif tag == 0x48 and cur is not None:
            cur['nh'] += 1
            if cur['nh'] > 240: cur['h'].append(s(a, 16))
        elif tag == 0x4F and cur is not None:
            cur['no'] += 1
            if cur['no'] > 8192 + 480: cur['o'].append(s(a, 16))
    def lines(o):
        sp = np.abs(np.fft.rfft(o * np.hanning(len(o)))); fr = np.fft.rfftfreq(len(o), 1 / 48000.0)
        pk = []
        for k in np.argsort(sp)[::-1][:60]:      # the strongest distinct lines (3 bins apart)
            if all(abs(k - j) > 3 for j in pk): pk.append(k)
            if len(pk) == 6: break
        ref = sp[pk[0]]
        return ', '.join('%.0f Hz %+.0f dB' % (fr[k], 20 * np.log10(max(sp[k], 1e-9) / ref)) for k in pk)
    for i, g in enumerate(segs):
        c = g['coeffs']
        name = 'MiSTer default' if (c['cx'], c['cx0'], c['cx1'], c['cx2']) == (4258969, 3, 3, 1) else 'cx %d cx0-2 %d %d %d' % (
            c['cx'], c['cx0'], c['cx1'], c['cx2'])
        h = np.array(g['h'], dtype=float); o = np.array(g['o'], dtype=float)
        msg = 'stretch %d (%s):' % (i, name)
        if len(h) >= 480:
            msg += ' IIR out %.0f ms rms %.0f peak %.0f [%s]' % (len(h) / 48.0, np.sqrt(np.mean(h ** 2)), np.abs(h).max(), lines(h))
        if len(o) >= 480:
            msg += '; HDMI side %.0f ms rms %.0f peak %.0f [%s]' % (len(o) / 48.0, np.sqrt(np.mean(o ** 2)), np.abs(o).max(), lines(o))
        print(msg)

def response(coeffs, freqs, fs=None):
    """|H(f)| in dB of the filter as tango_audio_filter.v runs it: H(z) = (cx/2^40)*2^19... in exact terms:
    y = x + t0 with x = in * cx / 2^20 and out = y / 2^20, numerator (cx/2^40) * [1, xs0, xs1, xs2] with xs_k the
    x-scale factor (c/16, negated for bit 7), denominator [1, cy0, cy1, cy2] / 2^21."""
    fs = fs or coeffs['rate']
    def scale(c):
        f = ((c & 1) / 16 + ((c >> 1) & 1) / 8 + ((c >> 2) & 1) / 4)
        return -f if c & 0x80 else f
    g = s(coeffs['cx'], 40) / 2 ** 40
    b = [g, g * scale(coeffs['cx0']), g * scale(coeffs['cx1']), g * scale(coeffs['cx2'])]
    a = [1.0] + [s(coeffs[k], 24) / 2 ** 21 for k in ('cy0', 'cy1', 'cy2')]
    out = []
    for f in freqs:
        z = complex(math.cos(2 * math.pi * f / fs), -math.sin(2 * math.pi * f / fs))
        num = sum(b[k] * z ** k for k in range(4)); den = sum(a[k] * z ** k for k in range(4))
        out.append(20 * math.log10(abs(num / den)))
    return out

def main():
    if len(sys.argv) >= 3 and sys.argv[1] == 'check':
        coeffs = None
        if '--json' in sys.argv:
            coeffs = json.load(open(sys.argv[sys.argv.index('--json') + 1]))['audio']
        if coeffs is None:
            sys.path.insert(0, __import__('os').path.dirname(__file__))
            from vproc_preset import AF_DEFAULTS
            coeffs = AF_DEFAULTS
        ok, st, bad, last = check(sys.argv[2], coeffs, keep_taps='--keep-taps' in sys.argv)
        if '--json' in sys.argv:
            exp = {k: coeffs[k] & ((1 << 40) - 1) for k in ('rate', 'cx', 'cx0', 'cx1', 'cx2', 'cy0', 'cy1', 'cy2')}
            if last != exp:
                print('iir_model: the RTL ended with coefficients %s, the file gives %s' % (last, exp)); ok = False
            else:
                print('iir_model: the RTL ended with the file\'s coefficients (rate %d, cx %d, cx0-2 %d %d %d, cy %d %d %d)'
                      % tuple(last[k] for k in ('rate', 'cx', 'cx0', 'cx1', 'cx2', 'cy0', 'cy1', 'cy2')))
        print('iir_model: %s  steps %d, IIR writes %d (bad %d), hand-overs %d (bad %d), outputs %d (bad %d, non-zero %d), '
              'mixed %d (bad %d, non-zero %d), resets %d'
              % ('PASS' if ok else 'FAIL', st['S'], st['W'], st['bad_W'], st['H'], st['bad_H'], st['O'], st['bad_O'],
                 st['nonzero_O'], st['M'], st['bad_M'], st['nonzero_M'], st['Z']))
        for x in bad: print('  first mismatches:', x)
        sys.exit(0 if ok else 1)
    if len(sys.argv) >= 3 and sys.argv[1] == 'spectrum':
        spectrum(sys.argv[2]); return
    if len(sys.argv) >= 3 and sys.argv[1] == 'response':
        c = json.load(open(sys.argv[2]))
        c = c.get('audio', c)
        fr = [20, 100, 1000, 2000, 5000, 10000, 15000, 20000, 24000, 30000, 50000]
        for f, db in zip(fr, response(c, fr)):
            print('%7d Hz  %7.2f dB' % (f, db))
        return
    print(__doc__); sys.exit(2)

if __name__ == '__main__':
    main()
