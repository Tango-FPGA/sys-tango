#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
# run_tb_tango_ascal_stream_ddr3.sh -- the DDR3 scaler port under ascal's read machine, a core streaming flat out and
# the stall patterns below (sim/tb_tango_ascal_stream_ddr3.v), as a gate.
# The bench's knobs are run-time plusargs (stall, stall_len, stall_line, adv, core, launcher, win_w, iphase, oframes,
# disp_lim, pre_lim); only Q_CORE and IDEAL are compile-time.  So: one Verilator binary per Q_CORE (4, 8, 16) and the
# IDEAL one (an instant memory), then every configuration is a run of one of them.
# Margins: per source and window, the floor is the IDEAL run's sHSYNC -> sDISP
# and preload line 0; limit = floor + (100 % - floor) / 2 for the line (2222 clocks) and the preload (2570 clocks).
# Hard limits in every run (the bench's TB_HARD).  Stalls in clk_ddr clocks (100 MHz): stall=1 cmd_ready low for
# stall_len (a pll_stop), stall=2 8 refreshes back to back; stall_line 747 = the preload, 160 = source line 50's read,
# -1 = a random line; iphase = the input's line when the output starts; adv = the core adversary (0 single-beat row
# misses, 1 bursts of 1-255, 2 a timed probe); launcher = the launcher's 640 x 360.
# Gate configurations must print TB_RESULT PASS; "r/" configurations are reported only: the 10 us pll_stop (beyond
# what the DDR3 bring-up test measured on silicon).  Run from the sys-tango checkout.
set -eu
cd "$(dirname "$0")/.."
exec python3 - "$@" <<'PY'
import os, re, subprocess, sys
ROOT = os.getcwd()
CAP = os.path.join(ROOT, 'build_tools', 'tango_capped.sh')
os.environ.setdefault('TANGO_MEM_CAP', '4G')
SRC = ['sim/tango_ddr3_model.v', 'tango_cdc.v', 'tango_ddr3_fifo.v', 'tango_ddr3_vport.v', 'tango_ddr3_cport.v',
       'tango_ddr3_arb.v', 'tango_ddr3.v', 'sim/tb_tango_ascal_stream_ddr3.v']
LINE, PRE = 2222, 2570
R8 = 'stall=2 stall_line=747'
P10 = 'stall=1 stall_len=1000'
CFGS = [('nocore', 8, 'core=0'), ('base', 8, ''), ('adv1', 8, 'adv=1'), ('probe', 8, 'adv=2'), ('w960', 8, 'win_w=960'),
        ('q4', 4, ''), ('q16', 16, ''), ('pre_ref8', 8, R8), ('l50_ref8', 8, 'stall=2 stall_line=160'),
        ('rand_ref8', 8, 'stall=2 stall_line=-1 oframes=12'), ('adv1_pre_ref8', 8, R8 + ' adv=1'),
        ('q16_pre_ref8', 16, R8)]
CFGS += [(f'ph{p}_pre_ref8', 8, f'{R8} iphase={p}') for p in (30, 60, 100, 130, 160, 200, 230, 250, 261)]
CFGS += [('l_nocore', 8, 'launcher core=0'), ('l_base', 8, 'launcher'), ('l_q4', 4, 'launcher'),
         ('l_q16', 16, 'launcher'), ('l_adv1', 8, 'launcher adv=1'), ('l_pre_ref8', 8, 'launcher ' + R8),
         ('l_probe', 8, 'launcher adv=2')]
# the worst case seen on silicon (sim/tango_ddr3_silicon.vh): cmd_ready low 35 clocks (350 ns) at the worst line, the
# core keeping the 9-deep queue full of row misses; and 8 postponed refreshes (ref8 above) -- NOT seen on silicon
W35 = 'stall=1 stall_len=35'
CFGS += [('wst_pre', 8, W35 + ' stall_line=747'), ('wst_l50', 8, W35 + ' stall_line=160'),
         ('wst_rand', 8, W35 + ' stall_line=-1 oframes=12'), ('l_wst_pre', 8, 'launcher ' + W35 + ' stall_line=747')]
CFGS += [('r/pre_pll10', 8, P10 + ' stall_line=747'), ('r/l50_pll10', 8, P10 + ' stall_line=160'),
         ('r/rand_pll10', 8, P10 + ' stall_line=-1 oframes=12'), ('r/l_pre_pll10', 8, 'launcher ' + P10 + ' stall_line=747')]
only = os.environ.get('CFGS')                             # a subset by name (space-separated)
if only:
    CFGS = [c for c in CFGS if c[0] in only.split() or c[0].lstrip('r/') in only.split()]

def run_cmd(cmd, log, cwd=ROOT):
    with open(log, 'w') as f:
        return subprocess.run(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT).returncode

def build(name, defs):
    obj = f'sim/obj_stream_ddr3_{name}'
    os.makedirs(obj, exist_ok=True)
    # -fno-life: Verilator 5.020 faults in V3Life (lifeAll) on this bench at -O1/-O2 ("internal fault"); the pass only
    # removes dead assignments, so turning it off changes nothing but speed
    cmd = ['nice', '-n', '10', CAP, 'verilator', '--binary', '--timing', '--timescale', '1ns/1ps', '-O2', '-fno-life',
           '-Isim', '-Wall', '-Wno-fatal', '-DTANGO_DDRAM'] + defs + ['-Wno-WIDTHEXPAND', '-Wno-WIDTHTRUNC', '-Wno-DECLFILENAME',
           '-Wno-UNUSEDSIGNAL', '-Wno-UNUSEDPARAM', '-Wno-BLKSEQ', '-Wno-PINCONNECTEMPTY', '--Mdir', obj,
           '--top-module', 'tb_tango_ascal_stream_ddr3'] + SRC + ['-o', 'Vtb']
    if run_cmd(cmd, f'{obj}/build.log') != 0:
        print(open(f'{obj}/build.log').read()[-3000:]); sys.exit(f'build {name} failed')
    return os.path.join(ROOT, obj, 'Vtb')

def run(binary, name, args):
    obj = f'sim/obj_stream_ddr3_run_{name.replace("/", "_")}'
    os.makedirs(obj, exist_ok=True)
    plus = ['+' + a for a in args.split()]
    run_cmd(['timeout', '3600', CAP, binary] + plus, f'{obj}/run.log', cwd=os.path.join(ROOT, obj))
    txt = open(f'{obj}/run.log', errors='replace').read()
    m = re.search(r'TB SUMMARY (.*)', txt)
    s = {k: int(float(v)) for k, v in re.findall(r'(\w+)=(-?[0-9.]+)', m.group(1))} if m else {}
    return txt, s

bins = {q: build(f'q{q}', [f'-DQ_CORE={q}']) for q in sorted({c[1] for c in CFGS})}
ideal = build('ideal', ['-DIDEAL'])
# floors and limits per source x window
def src_key(args):
    a = args.split()
    return ('launcher' if 'launcher' in a else 'genesis', next((x.split('=')[1] for x in a if x.startswith('win_w=')), '1280'))
lims = {}
for key in sorted({src_key(c[2]) for c in CFGS}):
    args = ('launcher ' if key[0] == 'launcher' else '') + ('' if key[1] == '1280' else f'win_w={key[1]}')
    txt, s = run(ideal, f'floor_{key[0]}_{key[1]}', args)
    if 'disp' not in s or s.get('hard') != 1:              # an invalid floor (e.g. the instant memory losing beats)
        print(txt[-2000:]); sys.exit(f'floor run {key} failed its hard checks: no floor')
    lims[key] = (s['disp'] + (LINE - s['disp']) // 2, s['pre'] + (PRE - s['pre']) // 2)
    print(f'== floor {key[0]} {key[1]}: line {s["disp"]} clocks = {100 * s["disp"] / LINE:.1f} %, preload line 0 '
          f'{s["pre"]} = {100 * s["pre"] / PRE:.1f} %; limits {lims[key][0]} = {100 * lims[key][0] / LINE:.1f} % / '
          f'{lims[key][1]} = {100 * lims[key][1] / PRE:.1f} %')
R = 0
for name, q, args in CFGS:
    gate = not name.startswith('r/')
    dl, pl = lims[src_key(args)]
    txt, s = run(bins[q], name, f'{args} disp_lim={dl} pre_lim={pl}')
    print(f'== {name} (Q_CORE {q}; +{" +".join(args.split()) if args else "defaults"})' + ('' if gate else ' [report only]'))
    for l in txt.splitlines():
        if l.startswith('TB '): print('   ' + l)
    if 'TB_RESULT PASS' in txt: print(f'   {name}: PASS')
    elif gate: print(f'   {name}: FAIL'); R = 1
    else: print(f'   {name}: outside the criteria (report)')
print('run_tb_tango_ascal_stream_ddr3: ' + ('PASS' if R == 0 else 'FAIL'))
sys.exit(R)
PY
