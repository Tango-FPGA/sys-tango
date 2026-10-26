#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
# identity.sh <core-dir> <template|nes> <sys-rev> <out.fs> -- the DDR3 path's identity gate: rebuild a core's
# bench-tested non-DDR3 configuration with sys/ checked out at <sys-rev>, exactly as its build script does (same
# options, same BUILD_DATE as its archive, TANGO_OUT=build/out), and copy the bitstream to <out.fs>.  The caller compares
# it byte for byte with the core's archived baseline bitstream.  <core-dir> is an export of the core repo at the
# baseline's commit whose sys/ is a git checkout (so <sys-rev> can be checked out in place: same paths every build).
# BD=<yymmdd> (required) is the BUILD_DATE of the baseline: the date in the archived bitstream's version string, which
# the core shows and which therefore enters the bitstream (a build that starts before midnight and ends after it
# carries the earlier day).  Without the same BD the rebuild can never be byte-identical, so there is no default.
# Never archives, never flashes.  Run inside a lane (take_lane.py).
set -eu
usage () { echo "usage: BD=<yymmdd> $0 <core-dir> <template|nes> <sys-rev> <out.fs>"; echo "  BD = the BUILD_DATE of the archived baseline bitstream"; exit 2; }
[ $# -eq 4 ] || usage
[[ "${BD:-}" =~ ^[0-9]{6}$ ]] || { echo "identity: BD is not set to a yymmdd date"; usage; }
C=$1; K=$2; REV=$3; OUT=$4
case $K in template|nes) ;; *) echo "unknown core $K"; exit 2;; esac
git -C "$C/sys" checkout -q --detach "$REV"
cd "$C"
case $K in
  template)
    NAME=tango_template_ascal138; SDC=-; FILES="$C/rtl/tango_core_template.v $C/rtl/template_cos.vh"
    ENV="TANGO_BUILD_DATE=$BD TANGO_VIDEO_ASCAL=1 TANGO_VIDEO_OPTS=1 TANGO_VIDEO_PROC=1 TANGO_VIDEO_ASCAL_MAXW=384";;
  nes)
    NAME=tango_nes_138; SDC="$C/build/nes.sdc"; N="$C/rtl/nestang"
    FILES="$C/rtl/nes_config.sv $N/t65/T65_Pack.v $N/t65/T65_ALU.v $N/t65/T65_MCode.v $N/t65/T65.v
       $N/apu.v $N/cart.sv $N/compat.v $N/dpram.v $N/EEPROM_24C0x.sv $N/nes.v $N/ppu.v $N/game_loader.v
       $N/autofire.v
       $N/mappers/generic.sv $N/mappers/iir_filter.v $N/mappers/JYCompany.sv $N/mappers/misc.sv $N/mappers/MMC1.sv
       $N/mappers/MMC2.sv $N/mappers/MMC3.sv $N/mappers/MMC5.sv $N/mappers/Namco.sv $N/mappers/Sachen.sv
       $N/mappers/Sunsoft.sv
       $C/rtl/pll_nes_mod.v $C/rtl/nes_clocks.v $C/rtl/nes_audio_hold.v $C/rtl/tango_nes.v"
    ENV="TANGO_BUILD_DATE=$BD TANGO_NES_SDRAM=1 TANGO_SD=1 TANGO_VIDEO_ASCAL=1 TANGO_VIDEO_OPTS=1 TANGO_VIDEO_PROC=1";;
  *) echo "unknown core $K"; exit 2;;
esac
rm -rf build/out/$NAME
echo "identity: $K, sys $(git -C sys rev-parse --short HEAD), $ENV"
env $ENV TANGO_NO_ARCHIVE=1 TANGO_OUT=build/out TANGO_MEM_CAP=8G sys/board/console138k/build_core.sh "$NAME" "$SDC" $FILES \
    > "$OUT.log" 2>&1 || { tail -20 "$OUT.log"; echo "identity: build FAILED"; exit 1; }
cp build/out/$NAME/impl/pnr/$NAME.fs "$OUT"
grep -E 'STA GATE|worst setup' "$OUT.log" | tail -2
sha256sum "$OUT"
