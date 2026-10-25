#!/usr/bin/env python3
"""sta_gate.py -- pass/fail gate on a Gowin build's timing + resources.

Usage: sta_gate.py [impl_dir] [--relax NAME:FREQ_MHZ ...]
  (default impl_dir: fpga/build/out/menu_link565/impl/pnr)

  --relax NAME:FREQ  Replace the SDC constraint for clock NAME with FREQ MHz when
                     checking Fmax.  Used when the SDC is overconstrained to force
                     compact placement but the PLL runs at a lower safe frequency
                     (e.g. --relax clk_sdram:108.0 when PLL = 108 MHz, SDC = 120 MHz).
                     When --relax is active and ALL Fmax checks pass (using the relaxed
                     thresholds), the setup violated-endpoint count and worst setup slack
                     are NOT used as fail signals: those metrics are measured against the
                     overconstrained SDC target, not the hardware frequency.

FAILS (exit 1) if:
  - any Hold violated endpoints
  - any Setup violated endpoints  (skipped when --relax active + all Fmax pass)
  - any clock's Actual Fmax < its Constraint (or its --relax override)
  - worst setup slack < +0.10 ns (R33: a build that closed under +0.10 ns worked in the tool
    and failed on silicon)
    (skipped when --relax active + all Fmax pass)
  - Logic > 98% or BSRAM > 98%  (Gowin 100%-fill breaks HDMI on silicon)
Prints the numbers either way. A bitstream from a failing build must NOT be
flashed -- marginal clk_mem reads ARE visible artifacts (nav bleed).
"""
import os, re, sys, glob

args = sys.argv[1:]
impl = "fpga/build/out/menu_link565/impl/pnr"
relax = {}  # clock name -> override MHz
i = 0
while i < len(args):
    if args[i] == "--relax" and i + 1 < len(args):
        name, freq = args[i+1].split(":", 1)
        relax[name] = float(freq)
        i += 2
    elif not args[i].startswith("--"):
        impl = args[i]; i += 1
    else:
        i += 1
fails = []

tr = glob.glob(os.path.join(impl, "*_tr_content.html"))
if not tr:
    print(f"STA GATE: no *_tr_content.html under {impl}"); sys.exit(1)
h = open(tr[0]).read()

def grab(label):
    m = re.search(label + r"\s*</td>\s*<td[^>]*>\s*([0-9]+)", h)
    return int(m.group(1)) if m else None

sv = grab("Numbers of Setup Violated Endpoints")
hv = grab("Numbers of Hold Violated Endpoints")
print(f"violated endpoints: setup={sv} hold={hv}")
if sv is None or hv is None: fails.append("could not parse violated-endpoint counts")
if hv: fails.append(f"{hv} hold-violated endpoints")
# setup violated count: deferred — checked after Fmax to handle overconstrain case

# Max Frequency Summary rows: name, constraint(MHz), actual(MHz)
# every <td> may carry attributes: Gowin marks a clock that misses its constraint
# <td style="color: #FF0000;" class = "error">, and that row must not be dropped
rows = re.findall(r"<td[^>]*>\s*(\w+)\s*</td>\s*<td[^>]*>\s*([0-9.]+)\(MHz\)\s*</td>\s*<td[^>]*>\s*([0-9.]+)\(MHz\)\s*</td>", h)
if not rows: fails.append("could not parse Max Frequency Summary")
fmax_all_pass = True
for name, cons, act in rows:
    cons, act = float(cons), float(act)
    threshold = relax.get(name, cons)
    ok = act >= threshold
    note = f" (relaxed to {threshold} MHz)" if name in relax else ""
    print(f"  {name:10s} constraint {cons:8.3f} MHz  actual {act:8.3f} MHz  {'MEET' if ok else 'FAIL'}{note}")
    if not ok:
        fails.append(f"{name} Fmax {act} < {threshold} (required{note})")
        fmax_all_pass = False

# Now apply the setup violated-endpoint check, suppressed when overconstrain active + Fmax all pass
if sv:
    if relax and fmax_all_pass:
        print(f"  note: {sv} setup violation(s) are against overconstrained SDC; Fmax passes at relaxed threshold(s)")
    else:
        fails.append(f"{sv} setup-violated endpoints")

# worst setup slack from timing_paths (blocks of: =====, SETUP, slack, ...)
tp = glob.glob(os.path.join(impl, "*.timing_paths"))
if tp:
    lines = [l.strip() for l in open(tp[0])]
    slacks = [float(lines[i+1]) for i, l in enumerate(lines)
              if l == "SETUP" and i + 1 < len(lines)]
    if slacks:
        w = min(slacks)
        print(f"worst setup slack: {w:+.3f} ns")

        if relax and fmax_all_pass:
            print(f"  note: worst slack measured against overconstrained SDC; Fmax passes at relaxed threshold(s)")
        else:
            # R33: a build that closed under +0.10 ns worked in the tool and failed on silicon.
            MIN_SLACK = 0.10
            if w < MIN_SLACK: fails.append(f"worst setup slack {w:+.3f} ns < +{MIN_SLACK:.2f} ns (R33 flashing margin)")

rpt = glob.glob(os.path.join(impl, "*.rpt.txt"))
if rpt:
    r = open(rpt[0]).read()
    for res, cap in (("Logic", 98), ("BSRAM", 98)):
        m = re.search(res + r"\s*\|\s*([0-9]+)/([0-9]+)", r)
        if m:
            used, tot = int(m.group(1)), int(m.group(2))
            pct = 100.0 * used / tot
            print(f"  {res:8s} {used}/{tot} ({pct:.0f}%)")
            if pct > cap: fails.append(f"{res} {pct:.0f}% > {cap}% budget")

if fails:
    print("STA GATE: FAIL"); [print("  -", f) for f in fails]; sys.exit(1)
print("STA GATE: PASS")
