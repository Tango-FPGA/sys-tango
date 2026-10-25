#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
"""confstr_expand.py -- the CONF_STR a core's RTL builds, with its macros expanded as the Verilog preprocessor does.

A core's `localparam CONF_STR = { ... };` (the string tango_conf_rom serves to the BL616) may hold macros: the
framework's Video entries `TANGO_VIDEO_CONF / `TANGO_VIDEO_CONF_P("P1") (tango_video_opts.vh) and `BUILD_DATE.  Taking
only the quoted parts out of the source (what the sim runners and check_confstr.py did) then gives a different
string from the one the synthesiser builds.  This tool preprocesses the core file the way Verilator and GowinSynthesis
do for this purpose: `define (also with arguments) / `undef / `ifdef / `ifndef / `elsif / `else / `endif, `include
followed through the core's directory and its sys/, and macros expanded inside the CONF_STR; then it evaluates the
concatenation (string literals only, their escapes decoded) as Verilog does: an empty string literal "" is one NUL
byte (measured, Verilator 5.020: {"", "A"} = 16'h0041), and the BL616 reads CONF_STR up to the first NUL.
  usage: confstr_expand.py [--sys DIR] [-D NAME[=TEXT]]... [--json] <core.v>
    --sys DIR   the framework (default: the nearest sys/ above the core file with tango_core_ports.vh, else this
                tool's own checkout)
    -D          a build define, as build_core.sh's tango_defines.v or a sim's -D (e.g. -D TANGO_VIDEO_ASCAL,
                -D 'BUILD_DATE="v261002"').  TANGO_VIDEO_OPTS is always defined here: the entries are expanded as an
                opted-in build sees them (a scaler build without it does not compile when the CONF_STR uses them).
                `BUILD_DATE with no definition expands to nothing.
    --json      also the framework facts check_confstr.py needs: the Video entries' character spans in the string,
                the bits sys owns (TANGO_VO_HI/LO) and each entry's declared bits (TANGO_VO_B_* / TANGO_VO_E_*)
  Prints the string (no trailing newline added) or the JSON; exit 2 with a message when the CONF_STR cannot be
  evaluated (an unknown macro, a NUL inside the string, anything other than string literals and concatenation).
As a module: expand(core_path, sys_dir=None, defines=None) -> dict (the --json fields).
"""
import json, os, re, sys

FRAGMENT_MACROS = ('TANGO_VIDEO_CONF', 'TANGO_VIDEO_CONF_P')
OPEN, CLOSE = '\x01', '\x02'                    # marks around a fragment macro's expansion (outside strings)


class ExpandError(Exception):
    pass


def find_sys(core_path):
    """The nearest directory sys/ above the core file that holds tango_core_ports.vh, or None."""
    d = os.path.dirname(os.path.abspath(core_path))
    while True:
        s = os.path.join(d, 'sys')
        if os.path.isfile(os.path.join(s, 'tango_core_ports.vh')):
            return s
        up = os.path.dirname(d)
        if up == d:
            return None
        d = up


def _string_end(text, i):
    """index just past the string literal starting at text[i] == '"'"""
    j = i + 1
    while j < len(text) and text[j] != '"':
        j += 2 if text[j] == '\\' else 1
    return j + 1


def _strip_comments(text):
    """comments -> spaces (newlines kept), string literals untouched"""
    out, i, n = [], 0, len(text)
    while i < n:
        if text[i] == '"':
            j = _string_end(text, i); out.append(text[i:j]); i = j
        elif text.startswith('//', i):
            j = text.find('\n', i); j = n if j < 0 else j
            out.append(' ' * (j - i)); i = j
        elif text.startswith('/*', i):
            j = text.find('*/', i + 2); j = n if j < 0 else j + 2
            out.append(re.sub(r'[^\n]', ' ', text[i:j])); i = j
        else:
            out.append(text[i]); i += 1
    return ''.join(out)


class Pre:
    """the directive pass: defines, conditionals, includes; collects the active text of the top file"""

    def __init__(self, inc_dirs, defines):
        self.inc_dirs = inc_dirs
        self.macros = {}                       # name -> (params or None, body)
        for k, v in defines.items():
            self.macros[k] = (None, v)
        self.files = []

    def run(self, path, top=True, depth=0):
        if depth > 16:
            raise ExpandError('`include nesting deeper than 16 at %s' % path)
        self.files.append(path)
        lines = _strip_comments(open(path).read()).split('\n')
        out, stack = [], []                    # stack of [active, taken]
        active = lambda: all(s[0] for s in stack)
        k = 0
        while k < len(lines):
            line = lines[k]; k += 1
            m = re.match(r'\s*`(ifdef|ifndef|elsif|else|endif|define|undef|include)\b(.*)$', line)
            if not m:
                out.append(line if active() else ''); continue
            d, rest = m.group(1), m.group(2)
            while rest.rstrip().endswith('\\') and k < len(lines):     # continued `define
                rest = rest.rstrip()[:-1] + '\n' + lines[k]; out.append(''); k += 1
            out.append('')
            name = (re.match(r'\s*([A-Za-z_]\w*)', rest) or [None, None])[1]
            if d in ('ifdef', 'ifndef'):
                outer = active()
                c = (name in self.macros) == (d == 'ifdef')
                stack.append([outer and c, c])
            elif d == 'elsif':
                if not stack: raise ExpandError('`elsif without `ifdef in %s' % path)
                c = (not stack[-1][1]) and (name in self.macros)
                stack[-1][1] |= c
                stack[-1][0] = c and all(s[0] for s in stack[:-1])
            elif d == 'else':
                if not stack: raise ExpandError('`else without `ifdef in %s' % path)
                c = not stack[-1][1]; stack[-1][1] = True
                stack[-1][0] = c and all(s[0] for s in stack[:-1])
            elif d == 'endif':
                if not stack: raise ExpandError('`endif without `ifdef in %s' % path)
                stack.pop()
            elif not active():
                pass
            elif d == 'define':
                m2 = re.match(r'\s*([A-Za-z_]\w*)(\(([^)]*)\))?(.*)$', rest, re.S)
                if not m2: raise ExpandError('bad `define in %s: %s' % (path, rest.strip()))
                params = [p.strip() for p in m2.group(3).split(',')] if m2.group(2) else None
                self.macros[m2.group(1)] = (params, m2.group(4).strip())
            elif d == 'undef':
                self.macros.pop(name, None)
            elif d == 'include':
                m2 = re.match(r'\s*"([^"]+)"', rest)
                if not m2: raise ExpandError('bad `include in %s' % path)
                for base in [os.path.dirname(path)] + self.inc_dirs:
                    p = os.path.join(base, m2.group(1))
                    if os.path.isfile(p):
                        self.run(p, top=False, depth=depth + 1); break
                else:
                    raise ExpandError('`include "%s" (from %s) not found in %s'
                                      % (m2.group(1), path, ', '.join([os.path.dirname(path)] + self.inc_dirs)))
        return '\n'.join(out) if top else None


def _args(text, i):
    """parse "( a, b )" at text[i] (after spaces): -> (args, index past ')')"""
    while i < len(text) and text[i] in ' \t\n': i += 1
    if i >= len(text) or text[i] != '(':
        raise ExpandError('a macro with arguments used without them')
    args, cur, depth, i = [], [], 0, i + 1
    while i < len(text):
        c = text[i]
        if c == '"':
            j = _string_end(text, i); cur.append(text[i:j]); i = j; continue
        if c in '([{': depth += 1
        elif c in ')]}':
            if depth == 0 and c == ')':
                args.append(''.join(cur).strip()); return args, i + 1
            depth -= 1
        elif c == ',' and depth == 0:
            args.append(''.join(cur).strip()); cur = []; i += 1; continue
        cur.append(c); i += 1
    raise ExpandError('unterminated macro arguments')


def _subst(body, params, args):
    """replace parameter names (identifiers outside strings) by the arguments"""
    if len(args) != len(params):
        raise ExpandError('macro wants %d argument(s), got %d' % (len(params), len(args)))
    table = dict(zip(params, args))
    out, i = [], 0
    while i < len(body):
        if body[i] == '"':
            j = _string_end(body, i); out.append(body[i:j]); i = j; continue
        m = re.match(r'[A-Za-z_]\w*', body[i:])
        if m and (i == 0 or not (body[i - 1].isalnum() or body[i - 1] in '_`$')):
            out.append(table.get(m.group(0), m.group(0))); i += m.end(); continue
        out.append(body[i]); i += 1
    return ''.join(out)


def _expand(text, macros, used, depth=0):
    if depth > 32:
        raise ExpandError('macro expansion deeper than 32 (recursive macro?)')
    out, i = [], 0
    while i < len(text):
        c = text[i]
        if c == '"':
            j = _string_end(text, i); out.append(text[i:j]); i = j; continue
        if c == '`':
            m = re.match(r'`([A-Za-z_]\w*)', text[i:])
            if not m: raise ExpandError('a lone ` in CONF_STR')
            name, i = m.group(1), i + m.end()
            if name not in macros:
                if name == 'BUILD_DATE':
                    used.append(name); continue           # not given: nothing (the version text only)
                raise ExpandError('`%s in CONF_STR is not defined (not in the core file, its includes or -D)' % name)
            params, body = macros[name]
            if params is not None:
                args, i = _args(text, i)
                body = _subst(body, params, args)
            used.append(name)
            exp = _expand(body, macros, used, depth + 1)
            out.append(OPEN + exp + CLOSE if name in FRAGMENT_MACROS else exp)
            continue
        out.append(c); i += 1
    return ''.join(out)


_ESC = {'n': '\n', 't': '\t', '\\': '\\', '"': '"', 'v': '\v', 'f': '\f', 'a': '\a'}


def _decode(lit):
    """the characters of a Verilog string literal (without its quotes)"""
    out, i = [], 0
    while i < len(lit):
        if lit[i] != '\\':
            out.append(lit[i]); i += 1; continue
        i += 1
        m = re.match(r'[0-7]{1,3}', lit[i:])
        if m:
            out.append(chr(int(m.group(0), 8) & 0xFF)); i += m.end(); continue
        m = re.match(r'x[0-9A-Fa-f]{1,2}', lit[i:])
        if m:
            out.append(chr(int(m.group(0)[1:], 16))); i += m.end(); continue
        out.append(_ESC.get(lit[i], lit[i])); i += 1
    s = ''.join(out)
    return s if s else '\0'                      # "" is one NUL byte (module docstring)


def expand(core_path, sys_dir=None, defines=None):
    core_path = os.path.abspath(core_path)
    sys_dir = sys_dir or find_sys(core_path) or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    defs = {'TANGO_VIDEO_OPTS': ''}
    defs.update(defines or {})
    pre = Pre([sys_dir, os.path.dirname(core_path)], defs)
    text = pre.run(core_path)
    k = text.find('localparam CONF_STR = {')
    if k < 0:
        raise ExpandError('%s: no "localparam CONF_STR = {" in the active text' % core_path)
    e = text.find('};', k)
    body = text[k + len('localparam CONF_STR = {'):e]
    used = []
    exp = _expand(body, pre.macros, used)
    conf, spans, i, start = [], [], 0, None
    n = 0                                          # characters so far
    while i < len(exp):
        c = exp[i]
        if c == '"':
            j = _string_end(exp, i); s = _decode(exp[i + 1:j - 1]); conf.append(s); n += len(s); i = j; continue
        if c == OPEN: start = n
        elif c == CLOSE: spans.append((start, n))
        elif c not in ' \t\r\n{},':
            m = re.match(r'[^"\x01\x02]{1,40}', exp[i:])
            raise ExpandError('CONF_STR holds something other than string literals: "%s" (after macro expansion)'
                              % m.group(0).strip())
        i += 1
    conf = ''.join(conf)
    z = conf.find('\0')
    if z >= 0:
        if conf[z:].strip('\0'):
            raise ExpandError('CONF_STR has a NUL at byte %d of %d (an empty string "" or a \\0 inside it): the BL616 '
                              'stops reading there' % (z, len(conf)))
        conf = conf[:z]
    hdr = os.path.join(sys_dir, 'tango_video_opts.vh')
    num = lambda m: int(pre.macros[m][1]) if m in pre.macros and pre.macros[m][1].strip().isdigit() else None
    fields = []
    for name, (params, body) in pre.macros.items():
        if name.startswith('TANGO_VO_E_') and params is None:
            b = pre.macros.get('TANGO_VO_B_' + name[11:], (None, ''))[1]
            mb = re.fullmatch(r'\s*(\d+)\s*:\s*(\d+)\s*', b)
            lits = re.findall(r'"((?:[^"\\]|\\.)*)"', body)
            fields.append(dict(name=name[11:], bits=[int(mb[1]), int(mb[2])] if mb else None,
                               entry=''.join(_decode(x) for x in lits)))
    lo, hi = num('TANGO_VO_LO'), num('TANGO_VO_HI')
    return dict(conf=conf, video_spans=[list(s) for s in spans], sys=sys_dir,
                header=hdr if os.path.isfile(hdr) else None,
                reserved=[lo, hi] if lo is not None and hi is not None else None, fields=fields,
                macros_used=sorted(set(used)), files=pre.files)


def main(argv):
    sys_dir, defs, as_json, core = None, {}, False, None
    it = iter(argv)
    for a in it:
        if a == '--sys': sys_dir = next(it)
        elif a == '--json': as_json = True
        elif a == '-D' or a.startswith('-D'):
            d = next(it) if a == '-D' else a[2:]
            nm, _, val = d.partition('=')
            defs[nm] = val
        elif core is None: core = a
        else: print(__doc__); return 2
    if core is None:
        print(__doc__); return 2
    try:
        r = expand(core, sys_dir, defs)
    except ExpandError as ex:
        print('confstr_expand: %s' % ex, file=sys.stderr); return 2
    sys.stdout.write(json.dumps(r, indent=1) + '\n' if as_json else r['conf'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
