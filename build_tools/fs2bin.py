#!/usr/bin/env python3
"""fs2bin.py -- convert a Gowin .fs (ASCII bitstream) into the raw .bin that BL616 loaders stream over JTAG.

Packs the '0'/'1' bit lines MSB-first. Verified: A2600Nano v2.5 a2600nano_console138k.fs converts
byte-identical to that release's own a2600nano_console138k.bin (4,656,528 bytes).
Usage: fs2bin.py in.fs out.bin
"""
import sys
bits = []
with open(sys.argv[1]) as f:
    for line in f:
        line = line.strip()
        if line and line[0] in '01' and set(line) <= {'0', '1'}:
            bits.append(line)
s = ''.join(bits)
if len(s) % 8:
    sys.exit(f'bit count {len(s)} is not a multiple of 8')
with open(sys.argv[2], 'wb') as f:
    f.write(bytes(int(s[i:i + 8], 2) for i in range(0, len(s), 8)))
print(f'{sys.argv[2]}: {len(s) // 8} bytes')
