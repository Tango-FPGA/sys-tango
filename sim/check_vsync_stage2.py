#!/usr/bin/env python3
"""check_vsync_stage2.py <run.log> [--want W1xV1,P1 W2xV2,P2 ...] -- judges a STAGE2=1 run of sim/tb_ascal_ddr3.v
(TANGO_VSYNC_ADJUST + TB_VSYNC_CORE + TB_VSYNC_STAGE2; sim/run_tb_vsync_ddr3.sh STAGE2=1: the runtime raster totals).

The source switches its frame length once (+core_lines -> +core_lines2 at +core_switch).  Required:
  1. tango_vsync measured each source rate exactly (P within +-1 clk_pix of the expected P_i) and planned the expected
     totals for it, in order: the RASTER lines (output raster changes) are W1xV1 then W2xV2 ... (the fixed raster first);
  2. every output frame of a native raster after its first two shows exactly ONE new input frame (no repeat, no drop),
     except frames whose source rate no longer matches the raster (between the source's switch and the re-plan): there
     at most one repeat or drop (the transition's slip) is allowed;
  3. ascal's output kept reading to the end (MON frame records up to the last output frames: a stalled scaler leaves none);
  4. "TANGO VIDEO OK" (ascal aligned once, every output VS on target through every raster change) and MON_RESULT PASS.
Prints CHECK_VSYNC_STAGE2 PASS / FAIL."""
import re
import sys


def main():
    log = open(sys.argv[1]).read().splitlines()
    want = []
    skip = set()                       # --skip a,b,...: output frames not judged (TB_PAUSE: the frames spanning a clock pause)
    if '--skip' in sys.argv:
        skip = {int(x) for x in sys.argv[sys.argv.index('--skip') + 1].split(',') if x}
        del sys.argv[sys.argv.index('--skip'):sys.argv.index('--skip') + 2]
    if '--want' in sys.argv:
        for w in sys.argv[sys.argv.index('--want') + 1:]:
            hv, p = w.split(',')
            h, v = hv.split('x')
            want.append((int(h), int(v), int(p)))
    ok = True
    meas = [int(m.group(1)) for m in (re.match(r'^VSYNC measured: P (\d+)', l) for l in log) if m]
    rasters = [(int(m.group(1)), int(m.group(2)), int(m.group(3))) for m in
               (re.match(r'^RASTER from output frame (\d+): (\d+) x (\d+)', l) for l in log) if m]
    switch = [l for l in log if l.startswith('tb_vsync_core: from source frame')]
    print(f'source: {switch[0] if switch else "no switch"}')
    print(f'measured P: {meas}')
    print('raster changes: ' + ', '.join(f'frame {f}: {h}x{v}' for f, h, v in rasters))
    # 1. measured and planned, in order
    for i, (h, v, p) in enumerate(want):
        hit = [m for m in meas if abs(m - p) <= 1]
        if not hit:
            print(f'FAIL: no measurement of P {p} (+-1)'); ok = False
        if i >= len(rasters) or (rasters[i][1], rasters[i][2]) != (h, v):
            print(f'FAIL: raster change {i + 1} is not {h}x{v}'); ok = False
    if len(rasters) != len(want):
        print(f'FAIL: {len(rasters)} raster changes, want {len(want)}'); ok = False
    # 2. the frame accounting per raster segment
    frames = [(int(m.group(1)), int(m.group(2)), int(m.group(3))) for m in
              (re.match(r'^MON frame (\d+): buffer \d+, input frames since the last (\d+) .*raster width (-?\d+),', l)
               for l in log) if m]
    native = {h for h, v, p in want}
    sw_mf = None                      # the first output frame after the source's switch (from the log order)
    if switch:
        idx = log.index(switch[0])
        later = [l for l in log[idx:] if l.startswith('MON frame ')]
        if later:
            sw_mf = int(re.match(r'^MON frame (\d+):', later[0]).group(1))
    seg_w, seg_n, trans_bad, steady_bad = None, 0, 0, 0
    replan_w = rasters[-1][1] if rasters else None
    for mf, ins, w in frames:
        if mf in skip:
            print(f'  output frame {mf}: spans the clock pause, not judged ({ins} new input frames)'); continue
        if w != seg_w:
            seg_w, seg_n = w, 0
        seg_n += 1
        if w not in native or seg_n <= 2:
            continue
        in_transition = sw_mf is not None and mf >= sw_mf and w != replan_w
        if ins != 1:
            if in_transition:
                trans_bad += 1
                print(f'  transition: output frame {mf} (raster width {w}) shows {ins} new input frames')
            else:
                steady_bad += 1
                print(f'FAIL: output frame {mf} (raster width {w}, steady) shows {ins} new input frames')
    judged = sum(1 for mf, ins, w in frames if w in native)
    print(f'frames on a native raster: {judged}; steady-state repeats/drops {steady_bad}; transition slips {trans_bad} (limit 1)')
    if steady_bad or trans_bad > 1:
        ok = False
    # 3. the output kept reading to the end: ascal's output stalled (e.g. a clock pause with reads in flight) leaves no MON frame
    #    records after the stall while the raster runs on -- every other check here only judges the frames it sees
    done = [int(m.group(1)) for m in (re.match(r'^TB done: (\d+) frames', l) for l in log) if m]
    last_mf = frames[-1][0] if frames else -1
    if done:
        print(f'output frames: {done[-1]}, the last one ascal read (MON frame record): {last_mf}')
        if last_mf < done[-1] - 6:              # the MON records lag the raster by 4 frames in a normal run
            print(f'FAIL: ascal stopped reading after output frame {last_mf} of {done[-1]} (the scaler stalled)'); ok = False
    # 4. alignment and the monitor
    tv = [l for l in log if l.startswith('TANGO VIDEO')]
    print(tv[-1] if tv else 'no TANGO VIDEO line')
    if not tv or not tv[-1].startswith('TANGO VIDEO OK'):
        print('FAIL: the alignment rule'); ok = False
    if not any(l.startswith('MON_RESULT PASS') for l in log):
        print('FAIL: MON_RESULT'); ok = False
    print('CHECK_VSYNC_STAGE2 ' + ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
