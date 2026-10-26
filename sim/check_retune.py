#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
"""check_retune.py <run.log> --fpix <Hz> --fixed WxH --plan WxH -- judges sim/tb_retune.v (the HDMI PLL retune at
core start, the clock-pause design: frames spanning a pause are marked PAUSED and not judged).
Every steady VS period (clk_pix count == W x H of the raster printed with it) must last exactly W x H / f_pix in real time,
f_pix = the preset's clock when the line says "PLL preset", 74.25 MHz when "PLL 74.25" (the PLL model's clocks are exact on
average, so the period's error must be below 1e-6 relative); phase 0 must end on the plan (WxH), phases 1-2 run ONLY
1650 x 750 at 74.25 MHz (= 60.000000 Hz) after the retune, phase 3 must end on the plan again at the preset; every AUDIO line
48 kHz within 0.02 %; the bench's own TB_RETUNE line must say DONE (retunes 2, no divider change while running, ACR clean)."""
import re, sys, argparse
ap = argparse.ArgumentParser(); ap.add_argument('log'); ap.add_argument('--fpix', type=float, required=True)
ap.add_argument('--fixed', required=True); ap.add_argument('--plan', required=True)
a = ap.parse_args()
fixed = tuple(map(int, a.fixed.split('x'))); plan = tuple(map(int, a.plan.split('x')))
L = open(a.log).read().splitlines()
rx = re.compile(r'^VS phase (\d+): (\d+) clk_pix, ([\d.]+) us = ([\d.]+) Hz \(raster (\d+) x (\d+), PLL (\S+), status ([0-9a-f]+)\)( PAUSED)?')
ok = True; per = {0: [], 1: [], 2: [], 3: []}; nsteady = 0
for l in L:
    m = rx.match(l)
    if not m: continue
    ph, clk, us, hz, w, h, pll = int(m[1]), int(m[2]), float(m[3]), float(m[4]), int(m[5]), int(m[6]), m[7]
    if m[9]: print(f'phase {ph}: a frame spanning the pause ({us:.1f} us) -- not judged'); continue
    per[ph].append((w, h, pll, clk, hz))
    if clk == w * h:
        nsteady += 1
        f = a.fpix if pll == 'preset' else 74.25e6
        want = f / (w * h)
        if abs(hz / want - 1) > 1e-6:
            ok = False; print(f'FAIL: phase {ph} {w}x{h} at {pll}: {hz:.6f} Hz, want {want:.6f}')
print(f'steady VS periods checked against the clock: {nsteady}')
for ph in range(4):
    seq = []
    for w, h, pll, clk, hz in per[ph]:
        if not seq or seq[-1][:3] != (w, h, pll): seq.append((w, h, pll, hz))
    print(f'phase {ph}: ' + ' -> '.join(f'{w}x{h} @{pll} {hz:.6f} Hz' for w, h, pll, hz in seq))
    if not per[ph]:
        ok = False; print(f'FAIL: phase {ph} has no VS'); continue
    last = per[ph][-1]
    if ph in (0, 3) and (last[0], last[1], last[2]) != (plan[0], plan[1], 'preset'):
        ok = False; print(f'FAIL: phase {ph} does not end on the plan {plan} at the preset')
    # phase 1: the apply waits for line 730, the pause, then the new fixed raster from the next frame: from the first
    # 1650 x 750 at 74.25 MHz on, nothing else; phase 2 (the same request again): only that
    if ph in (1, 2):
        fr = per[ph]
        k = next((i for i, (w, h, pll, clk, hz) in enumerate(fr) if (w, h, pll) == (1650, 750, '74.25')), None)
        if k is None or (ph == 2 and k != 0) or any((w, h, pll) != (1650, 750, '74.25') for w, h, pll, clk, hz in fr[k:]):
            ok = False; print(f'FAIL: phase {ph} does not settle on (only) 1650 x 750 at 74.25 MHz')
    if ph in (1, 2) and abs(per[ph][-1][4] - 60.0) > 1e-6:
        ok = False; print(f'FAIL: phase {ph}: {per[ph][-1][4]:.6f} Hz, not 60.000000')
for l in L:
    m = re.match(r'^AUDIO phase (\d+): (\d+) clk_audio rises in ([\d.]+) s = ([\d.]+) Hz', l)
    if m:
        print(l)
        if abs(float(m[4]) / 48000 - 1) > 2e-4: ok = False; print('FAIL: audio not 48 kHz')
tb = [l for l in L if l.startswith('TB_RETUNE')]
for l in tb: print(l)
if not any('TB_RETUNE DONE' in l for l in tb): ok = False; print('FAIL: the bench did not finish clean')
print('CHECK_RETUNE ' + ('PASS' if ok else 'FAIL'))
sys.exit(0 if ok else 1)
