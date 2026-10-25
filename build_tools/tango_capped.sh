#!/usr/bin/env bash
# tango_capped.sh -- run a heavy job (Gowin build, Verilator sim) inside its own
# memory-capped cgroup so a runaway is OOM-killed INSIDE the box instead of
# thrash-freezing the machine (a 15 GB RAM, 2 GB swap machine froze that way).
#   usage: tango_capped.sh <cmd...>          (cap from TANGO_MEM_CAP, default 9G)
# No swap for the job (MemorySwapMax=0): swap thrash is what froze the box.
# Prints the job's peak RSS so we learn real numbers.
# Needs systemd user scopes (systemd-run --user) with the memory controller delegated (cgroup v2).
set -u
CAP="${TANGO_MEM_CAP:-9G}"
echo "[tango_capped] cap=$CAP swap=0 : $*" >&2
systemd-run --user --scope -q -p MemoryMax="$CAP" -p MemorySwapMax=0 \
    /usr/bin/time -f "[tango_capped] exit=%x peak_rss=%MkB wall=%es" "$@"
rc=$?
{ [ $rc -eq 137 ] || [ $rc -eq 143 ]; } && echo "[tango_capped] KILLED (exit $rc) -- likely the $CAP memory cap; check: journalctl -k | grep \"Memory cgroup out of memory\"" >&2
exit $rc
