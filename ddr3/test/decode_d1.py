#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
"""decode_d1.py [--json] [--report] <capture.png|jpg|ppm ... | --uart log.txt> -- decode the DDR3 bring-up test's
results.

A capture: the board's 1280 x 720 picture, or the capture card's 1920 x 1080 of it (any size: coordinates scale).
Every slot block is sampled at its centre (the mean of a 3 x 3 patch; luma > 128 = 1).  The decoder searches a pixel
offset (dx -8..8, dy -4..4) for the one where slot 0 reads the magic word and the marker colours fit best, so the
encoder's register delays and the capture's position do not matter.
A UART log: lines "D1 <114 x 8 hex digits>", optionally after a timestamp ("123.45 D1 ..." or
"[seconds] D1 ...") -- the host's time is then the record's time.
Prints one record per input: its time (the file's modification time, or the log's timestamp), and the 114 slots;
--report adds the derived measurements (d1_verdict.report).  --json prints the records as JSON.
Exit 1 if an input cannot be decoded (no magic word)."""
import sys, os, re, json, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import d1_slots as S


def decode_image(fn):
    from PIL import Image
    im = Image.open(fn).convert('RGB')
    W, H = im.size
    px = im.load()
    sx, sy = W / 1280.0, H / 720.0

    def at(x, y, dx, dy):
        X, Y = (x + dx + 0.5) * sx - 0.5, (y + dy + 0.5) * sy - 0.5
        X, Y = int(round(X)), int(round(Y))
        cs = [px[i, j] for i in range(X - 1, X + 2) for j in range(Y - 1, Y + 2) if 0 <= i < W and 0 <= j < H]
        return tuple(sum(c[k] for c in cs) / len(cs) for k in range(3))

    def luma(c):
        return 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2]

    def word(s, dx, dy):
        v, margin = 0, 1e9
        for i in range(32):
            l = luma(at(*S.bit_centre(s, i), dx, dy))
            margin = min(margin, abs(l - 128))
            if l > 128:
                v |= 1 << i
        return v, margin

    # every offset where slot 0 reads the magic word, with its marker-colour error; the picture's true position is
    # the middle of that plateau (its edges sample next to the blocks' borders, where a capture's blur bites first)
    good = []
    for dy in range(-6, 7):
        for dx in range(-8, 9):
            m, mg = word(0, dx, dy)
            if m != S.MAGIC:
                continue
            err = 0
            for s in range(S.NS):
                c = at(*S.marker_centre(s), dx, dy); w = S.marker_colour(s)
                err += sum((a - b) ** 2 for a, b in zip(c, w))
            good.append((err, dx, dy))
    if not good:
        return None, None
    emin = min(e for e, _, _ in good)
    plateau = [(dx, dy) for e, dx, dy in good if e <= 1.5 * emin + 1e3 * S.NS]
    xs = sorted(d[0] for d in plateau); ys = sorted(d[1] for d in plateau)
    cx, cy = xs[len(xs) // 2], ys[len(ys) // 2]
    dx, dy = min(plateau, key=lambda d: (d[0] - cx) ** 2 + (d[1] - cy) ** 2)
    vals = [word(s, dx, dy)[0] for s in range(S.NS)]
    return vals, (dx, dy)


def parse_uart(fn):
    recs = []
    for n, line in enumerate(open(fn, errors='replace')):
        m = re.search(r'D1((?: [0-9A-Fa-f]{8}){%d})' % S.NS, line)
        if not m:
            continue
        t = None
        mt = re.match(r'\s*\[?\s*([0-9]+(?:\.[0-9]+)?)\s*\]?\s', line)
        if mt and line.find('D1') > mt.end() - 1:
            t = float(mt.group(1))
        vals = [int(x, 16) for x in m.group(1).split()]
        recs.append({'src': f'{fn}:{n + 1}', 't': t, 'slots': vals})
    return recs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('inputs', nargs='*')
    ap.add_argument('--uart', action='append', default=[])
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--report', action='store_true')
    a = ap.parse_args()
    recs, bad = [], 0
    for fn in a.inputs:
        vals, off = decode_image(fn)
        if vals is None:
            print(f'{fn}: NO DECODE (no magic word at any offset)', file=sys.stderr); bad += 1; continue
        recs.append({'src': fn, 't': os.path.getmtime(fn), 'slots': vals, 'offset': off})
    for fn in a.uart:
        r = parse_uart(fn)
        if not r:
            print(f'{fn}: no D1 lines', file=sys.stderr); bad += 1
        recs += r
    if a.json:
        print(json.dumps(recs, indent=1))
    else:
        for r in recs:
            print(f"== {r['src']}  t={r['t']}" + (f"  offset {r['offset']}" if 'offset' in r else ''))
            for i, v in enumerate(r['slots']):
                print(f'  {i:3d} {S.NAMES[i]:28s} {v:08X}  {v}')
            if a.report:
                import d1_verdict
                for line in d1_verdict.report(r['slots']):
                    print('  ' + line)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
