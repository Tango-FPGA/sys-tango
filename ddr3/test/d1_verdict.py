#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
"""d1_verdict.py [--min-duration S] <capture ... | --uart log ...> -- the DDR3 bring-up test's verdict over a series of
captures and / or UART lines (decoded by decode_d1.py): a decode tool, rates, a mechanical verdict.

PASS needs, in EVERY record:
  - slot 0 = the magic word; FLAGS: DDR3 PLL locked, calibrated, the IP out of reset; 0 calibration drops;
  - 0 wrong words, 0 bad passes (the march, the walk);
  - ORDER: the mask polarity found (1 or 2) and its map complete; the +0 burst order the identity;
  - at least one march pass, except in the warm-up (the records before the first pass);
and over the series (at least --min-duration seconds, 1200 by default: 20 min; at least 3 records), after the warm-up:
  - uptime advancing with the records' clock (each interval within 2 s + 2 %);
  - liveness: march passes and loops rise within every window of 3 x the measured pass period (the series' mean time
    a pass); words checked rise between consecutive UART records and between consecutive captures; and the passes'
    rate in every interval long enough to hold 10 passes within 0.85-1.15 x the series' median rate;
  - a capture and a UART record are compared only when they are at least one snapshot apart (--snap-s, 2.1 s: the
    UART's period): closer, they may show the same tester snapshot.
An earlier liveness rule asked every interval between consecutive records to show a new pass; with UART records
every 2.09 s and a 1 GB pass taking 6.8 s (up to 9.0 s), that failed by design (on silicon it gave 1054 such FAILs
with the memory clean).
The report (always printed): calibration time, clk_x1 / clk_ref per second, pll_stop (all / after calibration),
the FIFO probes, the stall windows (refresh gap, gaps per window = refreshes), latency per mode (min / mean / max /
max ever, histogram), the soak maxima (latency, cmd_ready low, wr_data_rdy low), bandwidth (MB/s), the byte / burst
order.  Exit 0 = PASS."""
import sys, os, argparse, statistics
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import d1_slots as S
import decode_d1

F_WALL = 100e6          # the wall clock (pll_ddr3 CLKOUT0)


def g(v, n):
    return v[S.IDX[n]]


def hi(x): return x >> 16
def lo(x): return x & 0xFFFF


def mbps(clocks, n=S.NBW, nbytes=32):
    return (n * nbytes) / (clocks / F_WALL) / 1e6 if clocks else 0.0


def order_text(v):
    out = []
    pol = g(v, 'ORDER_POL')
    out.append('byte mask: ' + {1: 'bit = 1 masks its byte (IPUG281 Table 6-1)', 2: 'bit = 1 WRITES its byte (inverted)',
                                0: 'INCONSISTENT'}.get(pol, '?') +
               f"  (map 1: {g(v, 'ORDER_OKA'):08X}, map 2: {g(v, 'ORDER_OKB'):08X}, k=0 bytes FF: {g(v, 'ORDER_S0'):08X})")
    for nm, off in (('ORDER_OFF0', 0), ('ORDER_OFF1', 1), ('ORDER_OFF2', 2), ('ORDER_OFF4', 4)):
        x = g(v, nm)
        lanes = []
        for l in range(8):
            nib = (x >> (4 * l)) & 0xF
            lanes.append(str(nib & 7) if nib & 8 else '?')
        out.append(f'read at column +{off}: user lane 0..7 <- written lane ' + ' '.join(lanes))
    return out


def report(v):
    r = []
    fl = g(v, 'FLAGS')
    r.append(f"flags {fl:08X}: DDR3 PLL lock {fl & 1}, calibrated {fl >> 1 & 1}, IP out of reset {fl >> 2 & 1}, "
             f"HDMI PLL lock {fl >> 3 & 1}, calibrated once {fl >> 4 & 1}, pll_stop now {fl >> 5 & 1}, ddr_rst {fl >> 6 & 1}")
    r.append(f"calibration: {g(v, 'INIT_CLK50') / 50.0:.1f} us after the IP's reset release; drops {g(v, 'CALIB_DROPS')}")
    r.append(f"uptime {g(v, 'UPTIME_S')} s; clk_x1 {g(v, 'CLKX1_PER_S')} /s; clk_ref {g(v, 'CLKREF_PER_S')} /s")
    r.append(f"pll_stop: {g(v, 'PSTOP_EVENTS')} events, longest {g(v, 'PSTOP_LOW_MAX') / 50.0:.2f} us; after calibration "
             f"{g(v, 'PSTOP_EVENTS_POSTCAL')} events, longest {g(v, 'PSTOP_LOW_MAX_POSTCAL') / 50.0:.2f} us, total "
             f"{g(v, 'PSTOP_LOW_TOTAL_POSTCAL') / 50.0:.2f} us")
    st = g(v, 'T_STATUS')
    r.append(f"tester: phase {st & 15}, running {st >> 5 & 1}, any error {st >> 6 & 1}; loops {g(v, 'LOOPS')}, march "
             f"passes {g(v, 'MARCH_PASSES')} (bad {g(v, 'MARCH_BAD_PASSES')}), words checked {g(v, 'WORDS_CHECKED_64K') * 65536}")
    r.append(f"errors {g(v, 'ERR_WORDS')}; first at 0x{g(v, 'FIRST_ERR_ADDR'):08X} DQ {g(v, 'FIRST_ERR_DQ'):08X} bytes "
             f"{g(v, 'FIRST_ERR_LANES'):08X}; all: DQ {g(v, 'CUM_ERR_DQ'):08X} bytes {g(v, 'CUM_ERR_LANES'):08X}")
    r.append(f"soak maxima: read latency {g(v, 'CUM_RDLAT_MAX') * 10} ns; cmd_ready low {g(v, 'CUM_CMDLOW_MAX') * 10} ns; "
             f"wr_data_rdy low {g(v, 'CUM_WRDYLOW_MAX') * 10} ns")
    fw, fr, ff = g(v, 'FIFO_WR'), g(v, 'FIFO_RD'), g(v, 'FIFO_FLAGS')
    r.append(f"FIFO (one bank, a new row per command, max of 4): writes taken {lo(fw)} before the first not-ready "
             f"({'cmd_ready' if ff & 1 else ''}{' wr_data_rdy' if ff & 2 else ''} low) in {g(v, 'FIFO_WR_T') * 10} ns, "
             f"{hi(fw)} before 8 clocks low; reads taken {lo(fr)} before the first drop in {g(v, 'FIFO_RD_T') * 10} ns "
             f"with {g(v, 'FIFO_RD_RET')} returned, {hi(fr)} before 8 low" +
             ('  (write probe SATURATED)' if ff & 4 else '') + ('  (read probe SATURATED: >= tag limit)' if ff & 8 else ''))
    gp, cl, wrn = g(v, 'STALL_RD_GAP'), g(v, 'STALL_RD_CMDLOW'), g(v, 'STALL_WR_RUN')
    win_s = S.STALL_WIN / F_WALL
    r.append(f"stall windows ({win_s * 1e3:.2f} ms): read stream longest gap {lo(gp) * 10} ns (ever {hi(gp) * 10} ns), "
             f"gaps >= 160 ns {g(v, 'STALL_RD_GAPS16')} (one per {win_s / max(1, g(v, 'STALL_RD_GAPS16')) * 1e6:.2f} us), "
             f"{g(v, 'STALL_RD_WORDS')} words = {g(v, 'STALL_RD_WORDS') * 32 / win_s / 1e6:.0f} MB/s; cmd_ready low "
             f"{lo(cl) * 10} ns (ever {hi(cl) * 10} ns)")
    r.append(f"  write stream longest not-ready {lo(wrn) * 10} ns (ever {hi(wrn) * 10} ns), runs >= 160 ns "
             f"{g(v, 'STALL_WR_RUNS16')}, {g(v, 'STALL_WR_WORDS')} words = {g(v, 'STALL_WR_WORDS') * 32 / win_s / 1e6:.0f} MB/s")
    for i, m in enumerate(S.LAT_MODES):
        mm, sm, cm = g(v, f'LAT_{m}_MINMAX'), g(v, f'LAT_{m}_SUM'), g(v, f'LAT_{m}_CMAX')
        line = (f"latency {m:12s}: min {lo(mm) * 10} ns, mean {sm / S.NLAT * 10:.1f} ns, max {hi(mm) * 10} ns, "
                f"max ever {lo(cm) * 10} ns, >= 10.24 us {hi(cm)}")
        if m in ('burst8_seq', 'burst8_rand'):
            lmm, lsm = g(v, f'LAST_{m}_MINMAX'), g(v, f'LAST_{m}_SUM')
            line += f"; all 8 back: min {lo(lmm) * 10} ns, mean {lsm / S.NLAT * 10:.1f} ns, max {hi(lmm) * 10} ns"
        r.append(line)
        if m in S.HIST_MODES:
            bins = []
            for j in range(8):
                x = g(v, f'HIST_{m}_{2 * j}_{2 * j + 1}')
                bins += [lo(x), hi(x)]
            r.append('    histogram (ns >=): ' + ' '.join(f'{S.BIN_LO[b] * 10}:{c}' for b, c in enumerate(bins) if c))
    r.append(f"bandwidth (32 MB each): seq write {mbps(g(v, 'BW_SEQ_WR')):.0f} MB/s, seq read {mbps(g(v, 'BW_SEQ_RD')):.0f}, "
             f"random write {mbps(g(v, 'BW_RND_WR')):.0f}, random read {mbps(g(v, 'BW_RND_RD')):.0f}, mixed "
             f"{mbps(g(v, 'BW_MIX')):.0f}")
    r += order_text(v)
    return r


def static_checks(v):
    bad = []
    fl = g(v, 'FLAGS')
    if g(v, 'MAGIC') != S.MAGIC: bad.append('magic')
    if not (fl & 1): bad.append('DDR3 PLL not locked')
    if not (fl >> 1 & 1): bad.append('not calibrated')
    if not (fl >> 2 & 1): bad.append('IP in reset')
    if g(v, 'CALIB_DROPS'): bad.append(f"{g(v, 'CALIB_DROPS')} calibration drops")
    if g(v, 'ERR_WORDS'): bad.append(f"{g(v, 'ERR_WORDS')} wrong words")
    if g(v, 'MARCH_BAD_PASSES'): bad.append(f"{g(v, 'MARCH_BAD_PASSES')} bad passes")
    pol = g(v, 'ORDER_POL')
    if pol not in (1, 2): bad.append('byte-mask polarity inconsistent')
    elif (g(v, 'ORDER_OKA') if pol == 1 else g(v, 'ORDER_OKB')) != 0xFFFFFFFF: bad.append('byte-mask map incomplete')
    if g(v, 'ORDER_OFF0') != 0xFEDCBA98: bad.append(f"burst order at +0 not the identity ({g(v, 'ORDER_OFF0'):08X})")
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('inputs', nargs='*')
    ap.add_argument('--uart', action='append', default=[])
    ap.add_argument('--min-duration', type=float, default=1200.0)
    ap.add_argument('--snap-s', type=float, default=2.1)
    a = ap.parse_args()
    recs = []
    for fn in a.inputs:
        vals, off = decode_d1.decode_image(fn)
        if vals is None:
            print(f'FAIL {fn}: no decode'); return 1
        recs.append({'src': fn, 't': os.path.getmtime(fn), 'slots': vals})
    for fn in a.uart:
        rs = decode_d1.parse_uart(fn)
        for k, r in enumerate(rs):
            if r['t'] is None:
                r['t'] = float(k)
        recs += rs
    recs.sort(key=lambda r: r['t'])
    for r in recs:
        r['cap'] = not r['src'].endswith('.log') and ':' not in os.path.basename(r['src'])   # a capture (else UART)
    ok = True
    for r in recs:
        bad = static_checks(r['slots'])
        if bad:
            ok = False
            print(f"FAIL {r['src']} (t={r['t']}): " + '; '.join(bad))
    # the warm-up: the records before the first march pass (exempt from the pass criteria and the series checks)
    k0 = next((k for k, r in enumerate(recs) if g(r['slots'], 'MARCH_PASSES') > 0), None)
    if k0 is None:
        print('FAIL no march pass in any record'); ok = False
        k0 = len(recs)
    elif k0:
        print(f"warm-up: {k0} record(s) before the first march pass (to t={recs[k0]['t'] - recs[0]['t']:.1f} s), exempt")
    ser = recs[k0:]
    if len(ser) < 3:
        print(f'FAIL only {len(ser)} records after the warm-up (need 3)'); ok = False
    else:
        span = ser[-1]['t'] - recs[0]['t']
        if span < a.min_duration:
            print(f'FAIL span {span:.0f} s < {a.min_duration:.0f} s'); ok = False
        # the measured pass period and the windows passes / loops must rise within
        npass = g(ser[-1]['slots'], 'MARCH_PASSES') - g(ser[0]['slots'], 'MARCH_PASSES')
        period = (ser[-1]['t'] - ser[0]['t']) / npass if npass > 0 else float('inf')
        win = 3 * period
        print(f'pass period {period:.2f} s measured over {npass} passes; passes and loops must rise within {win:.1f} s')
        for nm in ('MARCH_PASSES', 'LOOPS'):
            # never back, within each source (a UART line's copy is taken ~90 ms before its time stamp, so across
            # sources a record can show an older snapshot than the one before it in time)
            for kind in (False, True):
                chain = [r for r in ser if r['cap'] == kind]
                for p, q in zip(chain, chain[1:]):
                    if g(q['slots'], nm) < g(p['slots'], nm):
                        print(f"FAIL {nm} went back ({g(p['slots'], nm)} -> {g(q['slots'], nm)}) at {q['src']}"); ok = False
            # a rise (a new highest value, any source) within every window
            t_inc, top, worst = ser[0]['t'], g(ser[0]['slots'], nm), 0.0
            for q in ser[1:]:
                x = g(q['slots'], nm)
                if x > top:
                    worst = max(worst, q['t'] - t_inc); t_inc, top = q['t'], x
            worst = max(worst, ser[-1]['t'] - t_inc)
            print(f'{nm}: longest time without a rise {worst:.1f} s (window {win:.1f} s)')
            if worst > win:
                print(f'FAIL {nm} did not rise within {win:.1f} s ({worst:.1f} s)'); ok = False
        # words checked: between consecutive records of the same kind (UART after UART, capture after capture)
        for kind in (False, True):
            chain = [r for r in ser if r['cap'] == kind]
            for p, q in zip(chain, chain[1:]):
                if g(q['slots'], 'WORDS_CHECKED_64K') <= g(p['slots'], 'WORDS_CHECKED_64K'):
                    print(f"FAIL WORDS_CHECKED_64K did not rise ({g(p['slots'], 'WORDS_CHECKED_64K')} -> "
                          f"{g(q['slots'], 'WORDS_CHECKED_64K')}) {p['src']} -> {q['src']}"); ok = False
        # uptime and the rate, over consecutive records (a capture and a UART record only a snapshot or more apart)
        rates = []
        for p, q in zip(ser, ser[1:]):
            dt = q['t'] - p['t']
            if p['cap'] != q['cap'] and dt < a.snap_s:
                continue
            vp, vq = p['slots'], q['slots']
            du = g(vq, 'UPTIME_S') - g(vp, 'UPTIME_S')
            if abs(du - dt) > 2 + 0.02 * dt:
                print(f"FAIL uptime advanced {du} s in {dt:.1f} s ({p['src']} -> {q['src']})"); ok = False
            if dt > 0:
                rates.append(((g(vq, 'MARCH_PASSES') - g(vp, 'MARCH_PASSES')) / dt, dt))
        # the passes' rate over every interval long enough to hold 8 passes (a pass more or less: <= 12.5 %), within
        # 0.85-1.15 x the median of those intervals: between consecutive captures, or UART records if there are none
        chain = [r for r in ser if r['cap']] or [r for r in ser if not r['cap']]
        long_r = []
        for p, q in zip(chain, chain[1:]):
            dt = q['t'] - p['t']
            if dt / period >= 8:
                long_r.append(((g(q['slots'], 'MARCH_PASSES') - g(p['slots'], 'MARCH_PASSES')) / dt, p, q))
        medl = statistics.median([x for x, _, _ in long_r]) if long_r else 0.0
        print(f'rate: {len(long_r)} intervals hold >= 8 passes, median {medl:.4f}/s')
        for x, p, q in long_r:
            if not (0.85 * medl <= x <= 1.15 * medl):
                print(f"FAIL passes {x:.4f}/s vs median {medl:.4f}/s ({p['src']} -> {q['src']})"); ok = False
        if False:
            pass
    if recs:
        last = recs[-1]
        print(f"== report from the last record ({last['src']}, t={last['t']}):")
        for line in report(last['slots']):
            print('  ' + line)
    print('D1 VERDICT: ' + ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
