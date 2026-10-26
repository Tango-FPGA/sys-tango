#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 the Tango authors
#
# This file is derived from MiSTer's Main_MiSTer (https://github.com/MiSTer-devel/Main_MiSTer, compared at commit
# c3dc8ed): audio.cpp (setFilter: the audio filter file reader) and video.cpp (the scaler filter, gamma and shadow-mask
# file readers, the default nearest-neighbour filter, the preset reader) and file_io.cpp (FileReadLine), whose parsing
# it follows step by step with their names (read_video_filter, scale_phases, N_PHASES, SM_FLAG_*, ...).  audio.cpp,
# video.cpp and file_io.cpp carry no copyright or licence header of their own; Main_MiSTer's LICENSE file is the
# text of the GNU General Public License version 3, and none of them says "or any later version".  This file is under
# that version, the GNU General Public License version 3 (GPL-3.0-only; LICENSES/GPL-3.0.txt), not under the MIT
# licence of the rest of sys-tango; see NOTICE.
# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
"""vproc_preset.py -- MiSTer's video/audio processing files -> the Tango link transfers that load them
(README "Video and audio processing"), from Main_MiSTer (line numbers below are video.cpp / audio.cpp / file_io.cpp
of an earlier Main_MiSTer master; at c3dc8ed some have moved, e.g. read_video_filter starts at video.cpp 411).
The same parsing the BL616 firmware does in C (menu-tango bl616/tango/tango_vproc.c); the host tests compare the two
byte for byte.

Uses:
  * a build-time preset (TANGO_VPROC_PRESET, JTAG tests): --vh writes tango_vproc_preset.vh, a logic ROM of the
    transfers that tango_vproc_regs replays after reset;
  * a sim: --stream writes the transfers in the link-stream format (u16 little-endian length, then the bytes), for
    the testbenches' SPI master; --json writes the tables the FPGA should end up holding (the models' input).

  vproc_preset.py --root "$TANGO_FILTERS_ROOT" --preset 'Presets/Scanlines - Soft.ini' --scanlines 2 \\
                  --afilter 'General LPF/LPF 20khz 3rd BW.txt' --vh out/tango_vproc_preset.vh
  --afilter-cycle A,B (with --afilter): after the transfers, wait A ms, the Audio filter's Internal (0x39 with the
  volume byte alone), wait B ms, then from the start again, for ever (ROM records: length FFFF = a wait, u16 ms;
  FFFE = the start): the in-core OSD's Custom <-> Internal toggling as a JTAG test.
  In --stream the cycle is written --cycles times (tb_vproc.v wait records: length FFFF, u32 microseconds).
File names are relative to their folder (MiSTer's rule, Presets_MiSTer README): Filters/, Gamma/, Shadow_Masks/,
Filters_Audio/ under --root: a card's layout, or MiSTer-devel's repositories cloned side by side (Filters_MiSTer/,
ShadowMasks_MiSTer/, Presets_MiSTer/).  --root defaults to $TANGO_FILTERS_ROOT.
"""
import argparse, json, os, re, struct, sys

OUT_HEIGHT = 720                       # the Tango output height: shadow-mask "resolution=" sections compare with it
N_PHASES = 256                         # video.cpp 331

# ---------------------------------------------------------------- files as the card gives them
def read_ci(path):
    """the file's bytes, its path matched case-insensitively component by component (the card is FAT: MiSTer's
    presets name 'Poly_gamma/...' for the folder Poly_Gamma), or None if there is no such file"""
    if not path:
        return None
    if os.path.isfile(path):
        return open(path, 'rb').read()
    parts = os.path.normpath(path).split(os.sep)
    cur = os.sep if path.startswith(os.sep) else '.'
    for i, comp in enumerate(parts):
        if comp in ('', '.'):
            continue
        nxt = os.path.join(cur, comp)
        if not os.path.exists(nxt):
            try:
                low = comp.lower()
                m = [e for e in os.listdir(cur) if e.lower() == low]
            except OSError:
                return None
            if not m:
                return None
            nxt = os.path.join(cur, m[0])
        cur = nxt
    return open(cur, 'rb').read() if os.path.isfile(cur) else None

# ---------------------------------------------------------------- text files as Main reads them
def read_lines(data):
    """FileReadLine (file_io.cpp 2173-2193): lines end at CR or LF; leading spaces/tabs/CR/LF are skipped; lines that
    then start with '#' or ';', or are empty, are skipped.  Trailing whitespace stays."""
    s = data.decode('latin-1')
    s = s.split('\x00', 1)[0]
    out = []
    for raw in re.split(r'[\r\n]', s):
        t = raw.lstrip(' \t\r\n')
        if not t or t[0] in '#;':
            continue
        out.append(t)
    return out

_INT = re.compile(r'\s*([+-]?\d+)')
def scan_ints(line, n, sep=','):
    """sscanf(line, "%d,%d,...") with n conversions: returns the values converted before the first mismatch."""
    vals, pos = [], 0
    for i in range(n):
        if i:
            if pos >= len(line) or line[pos] != sep:
                break
            pos += 1
        m = _INT.match(line, pos)
        if not m:
            break
        vals.append(int(m.group(1)))
        pos = m.end()
    return vals

def c_short(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v

# ---------------------------------------------------------------- scaler filters (video.cpp 352-466)
NN_FILTER = [(0, 256, 0, 0), (0, 0, 256, 0)]

def scale_phases(phases, count):
    """video.cpp 352-375: each phase repeated 256 / count times; None if count does not divide 256."""
    if not count:
        return None
    dup = N_PHASES // count
    if count * dup != N_PHASES:
        return None
    return [phases[i // dup] for i in range(N_PHASES)]

def read_video_filter(path):
    """read_video_filter (video.cpp 377-466).  Returns dict(valid, adaptive, phases[256], adaptive_phases[256],
    file_phases (the file's own phase list, before scale_phases), count)."""
    phases, count, adaptive, scale = [], 0, False, 2
    data = read_ci(path)
    if data is not None:
        for line in read_lines(data):
            if count == 0 and line.lower() == 'adaptive':
                adaptive = True; continue
            if count == 0 and line.lower() == '10bit':
                scale = 1; continue
            v = scan_ints(line, 4)
            if len(v) == 4:
                if count >= (N_PHASES * 2 if adaptive else N_PHASES):
                    return dict(valid=False, too_many=True)          # 413: "too many" -> the slot is cleared
                phases.append(tuple(c_short(x * scale) for x in v))
                count += 1
    r = dict(adaptive=False, adaptive_phases=None, too_many=False)
    if adaptive:
        r['adaptive'] = True
        a = scale_phases(phases, count // 2)
        b = scale_phases(phases[count // 2:], count // 2) if a else None
        valid = a is not None and b is not None
        r.update(phases=a, adaptive_phases=b, file_phases=phases[:count // 2], file_adaptive=phases[count // 2:])
    elif count == 32:                                                 # legacy: H and V sets, the first 16 only
        a = scale_phases(phases, 16)
        valid = a is not None
        r.update(phases=a, file_phases=phases[:16])
    else:
        a = scale_phases(phases, count)
        valid = a is not None
        r.update(phases=a, file_phases=phases)
    if not valid:
        r.update(adaptive=False, phases=scale_phases(NN_FILTER, 2), adaptive_phases=None, file_phases=list(NN_FILTER))
    r['valid'] = valid
    return r

def file_phase_list(phases, file_phases):
    """The phases to send in one FLTCOEF transfer: the file's own (a power of two dividing 256) -> (k, list)."""
    n = len(file_phases)
    k = n.bit_length() - 1
    assert n == 1 << k and k <= 8, n
    assert scale_phases(file_phases, n) == phases
    return k, file_phases

FRAC = 8      # ascal's FRAC in a TANGO_VIDEO_PROC build (tango_cfg.vh TANGO_VPROC_FRAC): 2^FRAC slots per set

def slots(phases256, frac=None):
    """What tango_vproc_regs leaves in ascal's 2^FRAC slots: FRAC 8, all 256 phases as MiSTer's scale_phases makes
    them (video.cpp 352-374) and its full-precision send writes them (send_phases 483-501, sys_top.v 469-470);
    FRAC 4 (the earlier builds): slot s = phase 16 s (= MiSTer's 16-phase send, video.cpp 468-481)."""
    frac = FRAC if frac is None else frac
    step = 256 >> frac
    return [phases256[step * s] for s in range(1 << frac)]

def slots16(phases256):
    return slots(phases256, 4)

# ---------------------------------------------------------------- gamma (video.cpp 699-743)
def read_gamma(path):
    """setGamma: lines of "r,g,b" or "v"; 256 entries max.  Returns [(r, g, b)] as the FPGA stores them (& 0xFF)."""
    tab = []
    data = read_ci(path)
    if data is None:
        return None
    for line in read_lines(data):
        v = scan_ints(line, 3)
        if len(v) == 1:
            v = [v[0]] * 3
        if len(v) == 3:
            tab.append(tuple(x & 0xFF for x in v))
            if len(tab) >= 256:
                break
    return tab

# ---------------------------------------------------------------- shadow masks (video.cpp 808-928)
SM_FLAG_2X, SM_FLAG_ROTATED, SM_FLAG_ENABLED = 1 << 1, 1 << 2, 1 << 3
def SM_FLAG(v): return (0 << 13) | v
def SM_VMAX(v): return (1 << 13) | v
def SM_HMAX(v): return (2 << 13) | v
def SM_LUT(v):  return (3 << 13) | v
MASK_MODES = ['none', '1x', '2x', '1x rotated', '2x rotated']

def mask_flag(mode):
    return SM_FLAG({0: 0, 1: SM_FLAG_ENABLED, 2: SM_FLAG_ENABLED | SM_FLAG_2X, 3: SM_FLAG_ENABLED | SM_FLAG_ROTATED,
                    4: SM_FLAG_ENABLED | SM_FLAG_ROTATED | SM_FLAG_2X}.get(mode, 0))

def _scan_hex(line, n):
    vals, pos = [], 0
    for i in range(n):
        if i:
            if pos >= len(line) or line[pos] != ',':
                break
            pos += 1
        m = re.compile(r'\s*([+-]?(?:0[xX])?[0-9a-fA-F]+)').match(line, pos)
        if not m:
            break
        vals.append(int(m.group(1), 16) & 0xFFFFFFFF)
        pos = m.end()
    return vals

def read_mask(path, mode, height=OUT_HEIGHT):
    """setShadowMask (827-928): the 16-bit words for SYS 0x3E.  Entries a row does not give (x >= w) are sent as 0
    (Main sends whatever its stack holds there; they are never displayed)."""
    words = [mask_flag(mode)]
    loaded = False
    data = read_ci(path) if path else None
    lines = read_lines(data) if data is not None else None
    if lines is not None:
        start = 0
        for i, line in enumerate(lines):                    # the last resolution= section with N <= height
            if line[:11].lower() == 'resolution=':
                v = scan_ints(line[11:], 1)
                if v and height >= v[0]:
                    start = i + 1
        w = h = -1; y = 0; v2 = False
        for line in lines[start:]:
            if w == -1:
                if line.lower() == 'v2':
                    v2 = True; continue
                if line[:11].lower() == 'resolution=':
                    continue
                v = scan_ints(line, 2)
                if len(v) != 2 or v[0] <= 0 or v[1] <= 0 or v[0] > 16 or v[1] > 16:
                    break
                w, h = v
            else:
                p = _scan_hex(line, 16)
                if len(p) != w:
                    break
                p = p + [0] * (16 - len(p))
                for x in range(16):
                    if x < w:
                        words.append(SM_LUT((p[x] & 0x7FF) if v2 else (((p[x] & 7) << 8) | 0x2A)))
                    else:
                        words.append(SM_LUT(0))
                y += 1
                if y == h:
                    loaded = True
                    break
        if loaded:
            words += [SM_HMAX(w - 1), SM_VMAX(h - 1)]
    if not loaded:
        words.append(SM_FLAG(0))
    return words, loaded

# ---------------------------------------------------------------- audio filters (audio.cpp 23-108)
def read_afilter(path, vol=0):
    """setFilter: the data bytes of SYS 0x39 (README, link commands): vol, rate(4), cx(5), cx0, cx1, cx2, cy0(3), cy1(3), cy2(3),
    as far as the file gives them.  Also returns the values."""
    data = read_ci(path)
    out = bytes([vol & 0xFF]); vals = {}
    if data is None:                                  # audio.cpp 38: no file -> the volume alone (the defaults)
        return out, vals
    end = len(data); pos = 0; line = 0
    text = data.decode('latin-1')
    while pos < end and line < 9:
        st = pos
        while pos < end and text[pos] != '\x00' and text[pos] != '\n':
            pos += 1
        s = text[st:pos]; pos += 1
        s = s.lstrip(' \t\r')
        if not s or s[0] in '#;':
            continue
        if line == 0:
            if s[:2].lower() != 'v1':
                break
            line += 1
        elif line in (1, 3, 4, 5):
            v = scan_ints(s, 1)
            if v:
                v = v[0]
                if line == 1:
                    out += struct.pack('>I', v & 0xFFFFFFFF); vals['rate'] = v & 0xFFFFFFFF
                else:
                    out += bytes([v & 0xFF]); vals['cx%d' % (line - 3)] = v & 0xFF
                line += 1
        elif line == 2:
            m = re.match(r'\s*([+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?)', s)
            if m:
                c = int(0x8000000000 * float(m.group(1))) & ((1 << 64) - 1)
                out += (c & ((1 << 40) - 1)).to_bytes(5, 'big'); vals['cx'] = c & ((1 << 40) - 1)
                line += 1
        else:
            m = re.match(r'\s*([+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?)', s)
            if m:
                c = int(0x200000 * float(m.group(1)))
                c &= 0xFFFFFFFF                                           # int32
                out += (c & 0xFFFFFF).to_bytes(3, 'big'); vals['cy%d' % (line - 6)] = c & 0xFFFFFF
                line += 1
    return out, vals

AF_DEFAULTS = dict(rate=7056000, cx=4258969, cx0=3, cx1=3, cx2=1,       # Template_MiSTer sys_top.v 386-397
                   cy0=(-6216759) & 0xFFFFFF, cy1=6143386, cy2=(-2023767) & 0xFFFFFF)

# ---------------------------------------------------------------- presets (video.cpp 1010-1125)
def read_preset(path):
    """video_loadPreset: the settings a preset .ini changes.  Values: a file name, or 'off' (also 'same' for the
    filters).  Tango adds afilter= (an audio filter file, or off)."""
    r = {}
    data = read_ci(path)
    if data is None:
        return r
    for line in read_lines(data):
        low = line.lower()
        for key in ('hfilter=', 'vfilter=', 'sfilter=', 'ifilter=', 'mask=', 'maskmode=', 'gamma=', 'afilter='):
            if low.startswith(key):
                arg = line[len(key):].rstrip(' \t\r\n')
                if arg:
                    r[key[:-1]] = arg
                break
    return r

# ---------------------------------------------------------------- the transfers
SUBTREE = {'Filters': 'Filters_MiSTer/Filters', 'Gamma': 'Filters_MiSTer/Gamma',
           'Filters_Audio': 'Filters_MiSTer/Filters_Audio', 'Shadow_Masks': 'ShadowMasks_MiSTer/Shadow_Masks',
           'Presets': 'Presets_MiSTer/Presets'}
def folder(root, sub):
    """<root>/<sub>: a card layout, or MiSTer-devel's repositories cloned side by side (Filters_MiSTer/ ...)"""
    if os.path.isdir(os.path.join(root, 'Filters_MiSTer')):
        return os.path.join(root, SUBTREE[sub])
    return os.path.join(root, sub)

def transfers(cfg, root, sl_level=0, interlaced=False):
    """cfg: hfilter / vfilter / sfilter / ifilter / gamma / mask / maskmode / afilter as read_preset gives them.
    The BL616 firmware's order at core start: gamma, FLTNUM + coefficient sets, mask, audio filter.
    Returns (list of transfers [bytes], tables dict)."""
    D = lambda sub, name: os.path.join(folder(root, sub), name)
    xs, tab = [], {}
    def on(key):
        v = cfg.get(key)
        return v is not None and v.lower() not in ('off', 'same', 'none')
    # gamma (setGamma 699-743: a readable file = GAMCURV, then SET_GAMMA with the on/off; no file = nothing at all)
    gname = cfg.get('gamma')
    g = read_gamma(D('Gamma', gname)) if gname is not None and gname.lower() not in ('off', 'none') else None
    if g is not None:
        xs.append(bytes([0x00, 0x33, 0x00]) + b''.join(bytes(t) for t in g))
        xs.append(bytes([0x00, 0x32, 0x01]))
        tab['gamma'] = g; tab['gamma_en'] = 1
    else:
        tab['gamma_en'] = 0
    # scaler (set_vfilter, video.cpp 549-576)
    hmode = 1 if on('hfilter') else 0
    flt = {}
    for key in ('hfilter', 'vfilter', 'sfilter', 'ifilter'):
        flt[key] = read_video_filter(D('Filters', cfg[key])) if on(key) else None
        if flt[key] is not None and flt[key].get('too_many'):
            flt[key] = None                                               # Main clears the slot (689-692)
    if hmode and flt['hfilter'] is None:
        hmode = 0
    if interlaced:
        vkey = 'ifilter' if flt['ifilter'] else 'hfilter'
    elif sl_level and flt['sfilter']:
        vkey = 'sfilter'
    elif flt['vfilter']:
        vkey = 'vfilter'
    else:
        vkey = 'hfilter'
    scan = 1 if (hmode and vkey == 'sfilter') else 0
    xs.append(bytes([0x00, 0x2B, hmode | (scan << 1)]))
    tab['flt_mode'] = hmode | (scan << 1)
    H = flt['hfilter'] or read_video_filter('')                           # no file: the NN filter
    V = flt[vkey] or H
    sets = [(0, H['phases'], H['file_phases']), (1, V['phases'], V['file_phases'])]
    if H['adaptive']:
        sets.append((2, H['adaptive_phases'], H['file_adaptive']))
    elif V['adaptive']:
        sets.append((3, V['adaptive_phases'], V['file_adaptive']))
    tab['sets'] = {}
    for s, ph256, fph in sets:
        k, lst = file_phase_list(ph256, fph)
        b = bytes([0x00, 0x2A, s, k])
        for p in lst:
            for t in p:
                b += struct.pack('>H', t & 0x3FF)
        xs.append(b)
        tab['sets'][s] = slots(ph256)
    tab['vkey'] = vkey
    # shadow mask (setShadowMask 827-928): the mode's flag, then the file's words (a missing file: SM_FLAG(0))
    mode = 0
    mm = cfg.get('maskmode')
    if mm is not None and mm.lower() in MASK_MODES:
        mode = MASK_MODES.index(mm.lower())
    mname = cfg.get('mask')
    if mname is not None and mname.lower() in ('off', 'none'):
        mode, mname = 0, None
    words, loaded = read_mask(D('Shadow_Masks', mname) if mname else '', mode)
    b = bytes([0x00, 0x3E]) + b''.join(struct.pack('>H', w) for w in words)
    xs.append(b)
    tab['mask_words'] = words; tab['mask_loaded'] = loaded; tab['mask_mode'] = mode if loaded else 0
    # audio filter (setFilter): a file, else just the volume (the defaults); the volume byte is the core volume
    # {boost[6:5], att[4:0]} (get_core_volume, audio.cpp 144-152)
    vol = int(cfg.get('volume', 0)) & 0x7F
    if on('afilter'):
        data, vals = read_afilter(D('Filters_Audio', cfg['afilter']), vol)
    else:
        data, vals = bytes([vol]), {}
    a = dict(AF_DEFAULTS); a.update(vals)
    xs.append(bytes([0x00, 0x39]) + data)
    tab['audio'] = a; tab['volume'] = vol
    # the global volume (send_volume, audio.cpp 126-137: UIO_AUDVOL {mute, att[3:0]}), only when asked
    if cfg.get('audvol') is not None:
        av = int(cfg['audvol']) & 0x1F
        xs.append(bytes([0x00, 0x26, av]))
        tab['audvol'] = av
    return xs, tab

# xs: transfers (bytes) and, for --afilter-cycle, ('wait', ms) and ('loop',) records
def write_stream(xs, path, cycles=1):
    with open(path, 'wb') as f:
        for _ in range(cycles if any(isinstance(x, tuple) and x[0] == 'loop' for x in xs) else 1):
            for x in xs:
                if isinstance(x, tuple):
                    if x[0] == 'wait': f.write(b'\xff\xff' + struct.pack('<I', x[1] * 1000))
                else:
                    f.write(struct.pack('<H', len(x)) + x)

def rom_record(x):
    if isinstance(x, tuple):
        return b'\xff\xff' + struct.pack('>H', x[1]) if x[0] == 'wait' else b'\xff\xfe'
    return struct.pack('>H', len(x)) + x

def write_vh(xs, path, comment):
    rom = b''.join(rom_record(x) for x in xs) + b'\x00\x00'
    with open(path, 'w') as f:
        f.write('// tango_vproc_preset.vh -- GENERATED by sys/build_tools/vproc_preset.py; do not edit.\n')
        for c in comment:
            f.write('// %s\n' % c)
        f.write('// %d transfers, %d ROM bytes (per transfer: length, 16 bit big-endian, then target, command, data;\n'
                '// length FFFF = a wait, u16 milliseconds; FFFE = from the start again)\n'
                % (sum(1 for x in xs if not isinstance(x, tuple)), len(rom)))
        f.write("localparam [15:0] VPROC_PRESET_LEN = 16'd%d;\n" % len(rom))
        f.write('function [7:0] vproc_preset_byte(input [15:0] a);\n    case (a)\n')
        for i, v in enumerate(rom):
            if v:
                f.write("    16'd%d: vproc_preset_byte = 8'h%02X;\n" % (i, v))
        f.write("    default: vproc_preset_byte = 8'h00;\n    endcase\nendfunction\n")
    return len(rom)

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--root', default=os.environ.get('TANGO_FILTERS_ROOT'),
                    help="folder holding MiSTer's Filters/ Gamma/ Shadow_Masks/ Filters_Audio/ Presets/ (a card's "
                         "layout), or MiSTer-devel's Filters_MiSTer/, ShadowMasks_MiSTer/ and Presets_MiSTer/ "
                         "side by side (default: $TANGO_FILTERS_ROOT)")
    ap.add_argument('--preset', help='a MiSTer preset .ini (relative to <root>/Presets, or a path)')
    for k in ('hfilter', 'vfilter', 'sfilter', 'ifilter', 'gamma', 'mask', 'maskmode', 'afilter'):
        ap.add_argument('--' + k)
    ap.add_argument('--volume', type=lambda v: int(v, 0), help="the core volume byte of 0x39: {boost[6:5], att[4:0]} (default 0)")
    ap.add_argument('--audvol', type=lambda v: int(v, 0), help="a 0x26 transfer: the global volume {mute[4], att[3:0]}")
    ap.add_argument('--scanlines', type=int, default=None, help='the scanline level 0-3 (selects the Scan filter)')
    ap.add_argument('--status', help='the option word (hex); its bits 123:122 give --scanlines')
    ap.add_argument('--vh'); ap.add_argument('--stream'); ap.add_argument('--json')
    ap.add_argument('--frac', type=int, choices=(4, 8), default=FRAC, help="ascal's FRAC for --json's tables (default %d)" % FRAC)
    ap.add_argument('--afilter-cycle', help='A,B: then wait A ms, Internal, wait B ms, from the start again (with --afilter)')
    ap.add_argument('--cycles', type=int, default=3, help='--stream: the cycle written this many times (default 3)')
    a = ap.parse_args()
    globals()['FRAC'] = a.frac
    root = a.root
    if root is None and (a.preset or any(getattr(a, k) for k in ('hfilter', 'vfilter', 'sfilter', 'ifilter', 'gamma',
                                                                 'mask', 'afilter'))):
        ap.error("--root (or TANGO_FILTERS_ROOT): the folder with MiSTer's filter files")
    cfg = {}
    if a.preset:
        p = a.preset if os.path.exists(a.preset) else os.path.join(folder(root, 'Presets'), a.preset)
        cfg = read_preset(p)
    for k in ('hfilter', 'vfilter', 'sfilter', 'ifilter', 'gamma', 'mask', 'maskmode', 'afilter', 'volume', 'audvol'):
        if getattr(a, k) is not None:
            cfg[k] = getattr(a, k)
    sl = a.scanlines
    if sl is None:
        sl = (int(a.status, 16) >> 122) & 3 if a.status else 0
    xs, tab = transfers(cfg, root, sl)
    comment = ['settings: ' + json.dumps(cfg, sort_keys=True), 'scanline level %d -> V set from %s' % (sl, tab['vkey'])]
    if a.afilter_cycle:
        if not cfg.get('afilter'):
            ap.error('--afilter-cycle needs --afilter')
        ca, cb = (int(v) for v in a.afilter_cycle.split(','))
        xs += [('wait', ca), bytes([0x00, 0x39, tab['volume']]), ('wait', cb), ('loop',)]
        tab['afilter_cycle'] = [ca, cb]
        comment.append('then for ever: wait %d ms, the Audio filter Internal (00 39 %02X), wait %d ms, from the start' % (ca, tab['volume'], cb))
    if a.vh:
        n = write_vh(xs, a.vh, comment)
        print('%s: %d transfers, %d ROM bytes' % (a.vh, sum(1 for x in xs if not isinstance(x, tuple)), n))
    if a.stream:
        write_stream(xs, a.stream, a.cycles)
    if a.json:
        t = dict(tab); t['sets'] = {str(k): v for k, v in tab['sets'].items()}
        t['settings'] = cfg; t['scanlines'] = sl; t['frac'] = FRAC
        json.dump(t, open(a.json, 'w'), indent=1)
    if not (a.vh or a.stream or a.json):
        for x in xs:
            print(('%s %s' % x) if isinstance(x, tuple) and len(x) == 2 else (x[0] if isinstance(x, tuple) else
                  '%02X %02X +%d bytes' % (x[0], x[1], len(x) - 2)))

if __name__ == '__main__':
    main()
