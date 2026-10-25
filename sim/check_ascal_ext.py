#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
"""check_ascal_ext.py <dir> [--woven a-b] [--prog a-b] -- judge sim/tb_ascal_ext.v's frames (out_f<N>.ppm).

The source (sim/tb_ascal_ext_core.v) colours every pixel with the line it came from:
  interlaced  woven line L: R = L[7:0], G = {L[8], 0x3C}      dimmed 50 %: G = 0x1E / 0x5E
  progressive line L:       R = L[7:0], G = 0x63              dimmed 50 %: G = 0x31
Each output row is read at the picture's centre column.
  woven frame       every row interlaced and undimmed (no scanlines on interlaced video); L never goes back and
                    steps by at most 1 from row to row; all 448 woven lines present (both fields, in order)
  progressive frame odd output rows dimmed (the scanlines, cy[0]), even rows not; the undimmed rows' L never goes back;
                    all 224 lines present
--woven / --prog: the frame numbers that must be woven / progressive (inclusive ranges); other frames are printed only.
Exit 0 = every judged frame passes."""
import argparse, glob, os, re, sys

def read_ppm(p):
    d = open(p, 'rb').read()
    m = re.match(rb'P6\s+(\d+)\s+(\d+)\s+255\s', d)
    if not m:
        return None
    w, h = int(m.group(1)), int(m.group(2))
    px = d[m.end():]
    return (w, h, px) if len(px) >= w * h * 3 else None

def rows(ppm):
    w, h, px = ppm
    out = []
    for y in range(h):
        o = (y * w + w // 2) * 3
        out.append((px[o], px[o + 1], px[o + 2]))
    return out

def classify(r, g, b):
    if (r, g, b) == (0, 0, 0):
        return ('black', None)
    if g in (0x3C, 0xBC):
        return ('il', ((g >> 7) << 8) | r)
    if g in (0x1E, 0x5E):
        return ('il_dim', None)
    if g == 0x63:
        return ('prog', r)
    if g == 0x31:
        return ('prog_dim', None)
    return ('other', None)

def judge(rs):
    kinds = [classify(*p) for p in rs]
    act = [(y, k, L) for y, (k, L) in enumerate(kinds) if k != 'black']
    cnt = {}
    for _, k, _ in act:
        cnt[k] = cnt.get(k, 0) + 1
    res = {'rows': len(act), 'kinds': cnt}
    il = [L for _, k, L in act if k == 'il']
    if il and len(il) == len(act):
        back = sum(1 for a, b in zip(il, il[1:]) if b < a)
        big = sum(1 for a, b in zip(il, il[1:]) if b - a > 1)
        res.update(mode='woven?', back=back, jumps=big, distinct=len(set(il)), lo=min(il), hi=max(il))
        res['woven'] = back == 0 and big == 0 and len(set(il)) == 448 and min(il) == 0 and max(il) == 447
    pr = [(y, L) for y, k, L in act if k == 'prog']
    dm = [y for y, k, _ in act if k == 'prog_dim']
    if pr and len(pr) + len(dm) == len(act):
        Ls = [L for _, L in pr]
        back = sum(1 for a, b in zip(Ls, Ls[1:]) if b < a)
        par_ok = all(y % 2 == 0 for y, _ in pr) and all(y % 2 == 1 for y in dm)
        res.update(mode='prog?', back=back, distinct=len(set(Ls)), dimmed=len(dm), parity_ok=par_ok,
                   lo=min(Ls), hi=max(Ls))
        res['prog'] = back == 0 and par_ok and len(dm) > 0 and len(set(Ls)) == 224 and min(Ls) == 0 and max(Ls) == 223
    return res

def rng(s):
    if not s:
        return set()
    a, b = s.split('-')
    return set(range(int(a), int(b) + 1))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('--woven', default='')
    ap.add_argument('--prog', default='')
    a = ap.parse_args()
    want_w, want_p = rng(a.woven), rng(a.prog)
    files = sorted(glob.glob(os.path.join(a.dir, 'out_f*.ppm')), key=lambda p: int(re.findall(r'(\d+)', os.path.basename(p))[0]))
    bad = 0
    seen = set()
    for p in files:
        n = int(re.findall(r'(\d+)', os.path.basename(p))[0])
        ppm = read_ppm(p)
        if ppm is None:
            print(f'f{n}: (incomplete)')
            continue
        seen.add(n)
        r = judge(rows(ppm))
        verdict = 'WOVEN' if r.get('woven') else 'PROGRESSIVE+SCANLINES' if r.get('prog') else '-'
        need = 'woven' if n in want_w else 'prog' if n in want_p else None
        ok = (need is None) or (need == 'woven' and r.get('woven')) or (need == 'prog' and r.get('prog'))
        bad += 0 if ok else 1
        print(f"f{n}: {verdict:22s} {'' if need is None else ('ok' if ok else 'FAIL (want ' + need + ')')}  {r}")
    missing = (want_w | want_p) - seen
    if missing:
        print('missing frames:', sorted(missing))
        bad += len(missing)
    print('CHECK_ASCAL_EXT', 'PASS' if bad == 0 else f'FAIL ({bad})')
    return 0 if bad == 0 else 1

if __name__ == '__main__':
    sys.exit(main())
