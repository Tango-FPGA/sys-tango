#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
"""check_capture_decode.py <frame.ppm> <bank.txt> -- the DDR3 bring-up test's results screen as a capture card would
give it (1920 x 1080, bilinear, blurred, shifted, JPEG at quality 70 / 50 / 35 with 4:2:0 chroma): every version must
decode (decode_d1.py) to the bank exactly.  Exit 0 = all equal."""
import sys, os, json, subprocess
from PIL import Image, ImageFilter
frame, bankf = sys.argv[1], sys.argv[2]
here = os.path.dirname(os.path.abspath(__file__))
bank = [int(l, 16) for l in open(bankf) if l.strip()]
im = Image.open(frame).convert('RGB')
ok = True
for q, blur, sh in ((70, 0.0, (0, 0)), (50, 1.0, (5, 3)), (35, 1.5, (-7, -4))):
    x = im.resize((1920, 1080), Image.BILINEAR)
    if blur:
        x = x.filter(ImageFilter.GaussianBlur(blur))
    if sh != (0, 0):
        y = Image.new('RGB', x.size, (16, 32, 48)); y.paste(x, sh); x = y
    fn = os.path.join(os.path.dirname(frame), f'cap_q{q}.jpg')
    x.save(fn, quality=q, subsampling=2)
    out = subprocess.run([sys.executable, os.path.join(here, 'decode_d1.py'), '--json', fn], capture_output=True,
                         text=True).stdout
    r = json.loads(out or '[]')
    eq = bool(r) and r[0]['slots'] == bank
    print(f'== ui capture 1080p q{q} blur {blur} shift {sh}: ' + ('EQUAL' if eq else 'DIFFERENT') +
          (f"  (offset {r[0]['offset']})" if r else ''))
    ok &= eq
sys.exit(0 if ok else 1)
