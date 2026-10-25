#!/usr/bin/env python3
"""run_ddr3_matrix.py -- the DDR3 scaler port's margin matrix.

For each source x output window (sim/tb_tango_ascal_stream_ddr3.v, one plusarg set per run):
  floor  ascal's read machine with an instant memory (the IDEAL build): sHSYNC -> sDISP and the preload's line 0;
  idle   the DDR3 port on the IP model, no core;  core   the core streaming flat out (single-beat row misses);
  ref8   8 refreshes back to back at the preload, every frame (a stress row: NOT seen on silicon -- the DDR3
         bring-up test's longest refresh gap was 330 ns);  both   the core flat out and the 8 refreshes;
  wst    the worst case seen on silicon (sim/tango_ddr3_silicon.vh): the core flat out (row misses, the IP's command
         queue kept full) and cmd_ready low 35 clocks (350 ns, the soak maximum on silicon) at the preload, every frame.
  The IP model is calibrated on the DDR3 bring-up test's silicon numbers (sim/tango_ddr3_silicon.vh).
  limit = floor + (100 % - floor) / 2, for the line (22.22 us) and for the preload's line 0 (25.7 us): the memory's added
  time stays within half the slack.  Hard limits in every run (the bench's TB_HARD): no line past 100 %, no lost hsync,
  the DDA init never lost, line 100 at a constant time, no lost or merged write, data exact, waitrequest <= 32 clocks.
Then Q_CORE: the largest of 16 / 12 / 8 / 4 whose 'both' and 'wst' pass on every row and over the phase sweep (input line at
the output's start 0, 30 .. 261) on the rows with the least margin; the core's throughput (flat out) and latency (a
probe: one read at a time) at that Q_CORE and at 16; the 10 us pll_stop row (report only: silicon showed cmd_ready low
350 ns at most, pll_stop 0 after calibration).
Builds: one Verilator binary per Q_CORE plus the IDEAL one, memory-capped by build_tools/tango_capped.sh; -j N runs
N sims at once.  Writes <out>/matrix.json and prints the tables (markdown).
"""
import argparse, json, os, re, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = ['sim/tango_ddr3_model.v', 'tango_cdc.v', 'tango_ddr3_fifo.v', 'tango_ddr3_vport.v', 'tango_ddr3_cport.v',
       'tango_ddr3_arb.v', 'tango_ddr3.v', 'sim/tb_tango_ascal_stream_ddr3.v']
LINE, PRE = 2222, 2570          # clk_ddr clocks: an output line; the preload's DDA-init limit (25.7 us)

# sources: name, plusargs (I_W I_H I_TOT ILINE I_PXC), windows.  Clocks are 10 ns; I_PXC is clocks x 100 a pixel.
#   Genesis H40: 320 px of 8 MCLK (53.693175 MHz), lines of 3420 MCLK, 262 a frame; H32: 10 MCLK a pixel;
#   H40 + border: 347 x 243; NES: 341 dots of 186.2 ns; Atari 2600: 160 of 228 colour clocks (3.58 MHz);
#   Template: 320 x 240 of 400 x 262 at 6.1875 MHz (the Template core, size 0);
#   launcher: 640 x 360 native, a pixel each 2 clocks of 74.25 MHz on even beam lines (menu-tango's launcher).
SOURCES = [
    ('Genesis H40',   dict(i_w=320, i_h=224, i_tot=262, iline=6369, i_pxc=1490), (960, 1280)),
    ('Genesis H32',   dict(i_w=256, i_h=224, i_tot=262, iline=6369, i_pxc=1862), (960, 1280)),
    ('Genesis H40 + border', dict(i_w=347, i_h=243, i_tot=262, iline=6369, i_pxc=1490), (960, 1280)),
    ('NES',           dict(i_w=256, i_h=240, i_tot=262, iline=6351, i_pxc=1862), (960, 1280)),
    ('Atari 2600',    dict(i_w=160, i_h=192, i_tot=262, iline=6370, i_pxc=2794), (960, 1280)),
    ('Template',      dict(i_w=320, i_h=240, i_tot=262, iline=6465, i_pxc=1616), (960, 1280)),
    ('Launcher',      dict(i_w=640, i_h=360, i_tot=375, iline=4444, i_pxc=269), (1280,)),
]
COLS = {
    'idle': dict(core=0),
    'core': dict(core=1, adv=0),
    'ref8': dict(core=0, stall=2, stall_line=747),
    'both': dict(core=1, adv=0, stall=2, stall_line=747),
    'wst':  dict(core=1, adv=0, stall=1, stall_len=35, stall_line=747),
}
QCOLS = ('both', 'wst')         # the columns run at every Q_CORE (and that choose it)
PHASES = [0, 30, 60, 100, 130, 160, 200, 230, 250, 261]


def sh(cmd, cwd, log):
    with open(log, 'w') as f:
        return subprocess.run(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT).returncode


def build(out, name, defs):
    obj = os.path.join(out, 'obj_' + name)
    os.makedirs(obj, exist_ok=True)
    cap = os.path.join(ROOT, 'build_tools', 'tango_capped.sh')
    cmd = ['nice', '-n', '10', cap, 'verilator', '--binary', '--timing', '--timescale', '1ns/1ps', '-O2', '-fno-life',
           '-Isim', '-Wall', '-Wno-fatal', '-DTANGO_DDRAM'] + defs + [
           '-Wno-WIDTHEXPAND', '-Wno-WIDTHTRUNC', '-Wno-DECLFILENAME', '-Wno-UNUSEDSIGNAL', '-Wno-UNUSEDPARAM',
           '-Wno-BLKSEQ', '-Wno-PINCONNECTEMPTY', '-Wno-TIMESCALEMOD', '--Mdir', obj, '--top-module',
           'tb_tango_ascal_stream_ddr3'] + SRC + ['-o', 'Vtb']
    if sh(cmd, ROOT, os.path.join(obj, 'build.log')) != 0:
        sys.exit(f'build {name} failed: {obj}/build.log')
    return os.path.join(obj, 'Vtb')


def run(binary, args, log):
    cap = os.path.join(ROOT, 'build_tools', 'tango_capped.sh')
    cmd = ['timeout', '3600', cap, binary] + [f'+{k}={v}' if v is not None else f'+{k}' for k, v in args.items()]
    sh(cmd, os.path.dirname(binary), log)
    txt = open(log, errors='replace').read()
    m = re.search(r'TB SUMMARY (.*)', txt)
    if not m:
        return {'error': 'no summary', 'log': log}
    r = {k: (float(v) if '.' in v else int(v)) for k, v in re.findall(r'(\w+)=(-?[0-9.]+)', m.group(1))}
    r['log'] = log
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-j', type=int, default=1)
    ap.add_argument('--out', default=os.path.join(ROOT, 'sim', 'obj_ddr3_matrix'))
    ap.add_argument('--quick', action='store_true', help='one row, Q_CORE 8 only (a smoke test)')
    ap.add_argument('--rows', default='', help='spot rows: comma-separated substrings of the row names, e.g. '
                    '"Launcher,Template, 1280,border, 1280" (every column; the core numbers on the first row)')
    ap.add_argument('--qs', default='', help='the Q_CORE values, e.g. "16" (the columns run at the first)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    qs = [8] if a.quick else [16, 12, 8, 4]
    if a.qs:
        qs = [int(x) for x in a.qs.split(',')]
    QC = 8 if 8 in qs else qs[0]                    # the columns' Q_CORE
    bins = {'ideal': build(a.out, 'ideal', ['-DIDEAL'])}
    for q in qs:
        bins[q] = build(a.out, f'q{q}', [f'-DQ_CORE={q}'])
    print(f'built {len(bins)} binaries in {time.time() - t0:.0f} s', flush=True)
    rows = []
    for name, sa, wins in (SOURCES[:1] if a.quick else SOURCES):
        for w in wins:
            rows.append((f'{name}, {w} wide', dict(sa, win_w=w)))
    if a.rows:
        rows = [r for r in rows if any(k.strip() and k in r[0] for k in a.rows.split(','))]
    pool = ThreadPoolExecutor(max_workers=a.j)
    res = {'rows': {}, 'q': {}, 'sweep': {}, 'core': {}, 'pll': {}}
    n = [0]

    def job(binary, args, tag):
        n[0] += 1
        log = os.path.join(a.out, re.sub(r'[^A-Za-z0-9_.-]+', '_', tag) + '.log')
        r = run(binary, args, log)
        print(f'  {tag}: ' + ('ERROR ' + r['error'] if 'error' in r else
              f"disp {r['disp']} pre {r['pre']} hard {r['hard']} margin {r['margin']}"), flush=True)
        return r

    # 1. floors
    fl = {rn: pool.submit(job, bins['ideal'], dict(ra), f'floor {rn}') for rn, ra in rows}
    floors = {rn: f.result() for rn, f in fl.items()}
    lim = {}
    for rn, _ in rows:
        f = floors[rn]
        lim[rn] = (f['disp'] + (LINE - f['disp']) // 2, f['pre'] + (PRE - f['pre']) // 2)
    # 2. the columns at Q_CORE 8, and 'both' at every Q
    futs = {}
    for rn, ra in rows:
        dl, pl = lim[rn]
        for c, ca in COLS.items():
            futs[(rn, c, QC)] = pool.submit(job, bins[QC], dict(ra, **ca, disp_lim=dl, pre_lim=pl), f'{rn} {c} q{QC}')
        for q in qs:
            if q != QC:
                for c in QCOLS:
                    futs[(rn, c, q)] = pool.submit(job, bins[q], dict(ra, **COLS[c], disp_lim=dl, pre_lim=pl),
                                                   f'{rn} {c} q{q}')
    out = {k: f.result() for k, f in futs.items()}
    ok = lambda r: r.get('hard') == 1 and r.get('margin') == 1
    for rn, _ in rows:
        res['rows'][rn] = {'floor': floors[rn], 'limit': lim[rn],
                           'cols': {c: out[(rn, c, QC)] for c in COLS}}
        for q in qs:
            res['q'].setdefault(str(q), {})[rn] = {c: out[(rn, c, q)] for c in QCOLS}
    # 3. the phase sweep of 'both' and 'wst' on the three rows with the least margin (at Q_CORE 8), largest passing Q
    def margin_left(rn):
        return min((lim[rn][0] - res['rows'][rn]['cols'][c].get('disp', 1 << 30)) / max(1, LINE - floors[rn]['disp'])
                   for c in QCOLS)
    tight = sorted([rn for rn, _ in rows], key=margin_left)[:3]
    chosen = None
    for q in qs:
        if not all(ok(res['q'][str(q)][rn][c]) for rn, _ in rows for c in QCOLS):
            continue
        sw = {}
        for rn, ra in rows:
            if rn not in tight:
                continue
            dl, pl = lim[rn]
            for ph in PHASES[1:]:
                for c in QCOLS:
                    sw[(rn, c, ph)] = pool.submit(job, bins[q], dict(ra, **COLS[c], iphase=ph, disp_lim=dl, pre_lim=pl),
                                                  f'{rn} {c} q{q} phase {ph}')
        sw = {k: f.result() for k, f in sw.items()}
        res['sweep'][str(q)] = {f'{rn} {c} @{ph}': r for (rn, c, ph), r in sw.items()}
        if all(ok(r) for r in sw.values()):
            chosen = q
            break
    res['chosen_q'] = chosen
    # 4. the core's throughput and latency at the chosen Q_CORE and at 16; the pll_stop row
    g = dict(rows[1][1]) if not (a.quick or a.rows) else dict(rows[0][1])   # Genesis H40, 1280 wide (spot: the first row)
    for q in sorted({x for x in (chosen, 16) if x in bins}):
        res['core'][str(q)] = {
            'flat': pool.submit(job, bins[q], dict(g, core=1, adv=0), f'core flat q{q}').result(),
            'bursts': pool.submit(job, bins[q], dict(g, core=1, adv=1), f'core bursts q{q}').result(),
            'probe': pool.submit(job, bins[q], dict(g, core=1, adv=2), f'core probe q{q}').result()}
    rn0 = rows[1][0] if not (a.quick or a.rows) else rows[0][0]
    res['pll'] = pool.submit(job, bins[QC], dict(g, core=1, adv=0, stall=1, stall_len=1000, stall_line=747,
                                                disp_lim=lim[rn0][0], pre_lim=lim[rn0][1]), 'pll_stop 10 us').result()
    json.dump(res, open(os.path.join(a.out, 'matrix.json'), 'w'), indent=1, default=str)
    # tables
    pct = lambda c, base: f"{100.0 * c / base:.1f}"
    print(f'\nLine: sHSYNC -> sDISP worst, % of a line (22.22 us), Q_CORE {QC}.  "8 refreshes" and "both" are a stress case NOT seen '
          'on silicon; "worst seen" = the core flat out + cmd_ready low 350 ns (D1 soak maxima).')
    print('\n| Case | floor | limit | idle | core flat out | 8 refreshes | both | worst seen | preload (both / worst seen): floor / limit / measured | verdict |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    allok = True
    for rn, _ in rows:
        R = res['rows'][rn]; f = R['floor']; dl, pl = R['limit']; C = R['cols']
        rok = all(ok(C[c]) for c in COLS); allok = allok and rok
        cells = [pct(C[c].get('disp', 0), LINE) + ('' if C[c].get('hard') == 1 else ' HARD') +
                 ('' if C[c].get('margin') == 1 else ' OVER') for c in COLS]
        print(f"| {rn} | {pct(f['disp'], LINE)} | {pct(dl, LINE)} | " + ' | '.join(cells) +
              f" | {pct(f['pre'], PRE)} / {pct(pl, PRE)} / {pct(C['both'].get('pre', 0), PRE)}, {pct(C['wst'].get('pre', 0), PRE)}"
              f" | {'PASS' if rok else 'FAIL'} |")
    print(f"\nQ_CORE chosen: {chosen}")
    for q in qs:
        bad = [f'{rn} {c}' for rn, _ in rows for c in QCOLS if not ok(res['q'][str(q)][rn][c])]
        worst = max(((rn, c) for rn, _ in rows for c in QCOLS), key=lambda k: (res['q'][str(q)][k[0]][k[1]].get('disp', 0) -
                    floors[k[0]]['disp']) / max(1, LINE - floors[k[0]]['disp']))
        print(f"  Q_CORE {q}: fails on {bad if bad else 'none'}; the most used slack: {worst[0]} {worst[1]}")
    for q, sw in res['sweep'].items():
        bad = [k for k, r in sw.items() if not ok(r)]
        print(f"  phase sweep at Q_CORE {q} ({len(sw)} runs on {tight}): {'all pass' if not bad else 'fails: ' + str(bad)}")
    for q, d in res['core'].items():
        print(f"  Q_CORE {q}: core flat out {d['flat'].get('cmbs')} MB/s (single-beat row misses), bursts {d['bursts'].get('cmbs')} MB/s, "
              f"probe read latency max {d['probe'].get('clat')} ns")
    p = res['pll']
    print(f"  pll_stop 10 us (report only; not seen on silicon): line {pct(p.get('disp', 0), LINE)} %, preload {pct(p.get('pre', 0), PRE)} %, "
          f"hard {p.get('hard')}, margin {p.get('margin')}")
    v = allok and chosen is not None
    print(f'\nrun_ddr3_matrix: {n[0]} runs in {time.time() - t0:.0f} s; results {a.out}/matrix.json')
    print('run_ddr3_matrix: ' + ('PASS' if v else 'FAIL') + f' (every row x column at Q_CORE {QC}; Q_CORE chosen {chosen})')
    sys.exit(0 if v else 1)


if __name__ == '__main__':
    main()
