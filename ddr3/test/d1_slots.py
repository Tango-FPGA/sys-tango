# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the Tango authors
"""d1_slots.py -- the DDR3 bring-up test's results screen: its slot map and geometry (ddr3/test/tango_ddr3_test_core.v,
tango_ddr3_tester.v, tango_ddr3_ov.v).  Shared by decode_d1.py and d1_verdict.py."""

NS = 114
MAGIC = 0xD1D0_0002
# geometry (1280 x 720): slot s at column s // 38, row s % 38
X0, Y0, CP, RP, ROWS = 24, 80, 416, 16, 38


def slot_xy(s):
    return X0 + CP * (s // ROWS), Y0 + RP * (s % ROWS)


def bit_centre(s, i):            # bit i (31..0) of slot s: its 9 x 12 block's centre
    x, y = slot_xy(s)
    return x + 16 + 11 * (31 - i) + 4.0, y + 5.5


def marker_centre(s):
    x, y = slot_xy(s)
    return x + 5.5, y + 5.5


NAMES = ['MAGIC', 'FLAGS', 'INIT_CLK50', 'UPTIME_S', 'CALIB_DROPS', 'CLKX1_PER_S', 'CLKREF_PER_S', 'PSTOP_EVENTS',
         'PSTOP_LOW_MAX', 'PSTOP_EVENTS_POSTCAL', 'PSTOP_LOW_MAX_POSTCAL', 'PSTOP_LOW_TOTAL_POSTCAL',
         'T_STATUS', 'LOOPS', 'MARCH_PASSES', 'MARCH_BAD_PASSES', 'ERR_WORDS', 'FIRST_ERR_ADDR', 'FIRST_ERR_DQ',
         'FIRST_ERR_LANES', 'CUM_ERR_DQ', 'CUM_ERR_LANES', 'WORDS_CHECKED_64K', 'CUM_RDLAT_MAX', 'CUM_CMDLOW_MAX',
         'CUM_WRDYLOW_MAX', 'ORDER_POL', 'ORDER_OKA', 'ORDER_OKB', 'ORDER_S0', 'ORDER_OFF0', 'ORDER_OFF1',
         'ORDER_OFF2', 'ORDER_OFF4', 'FIFO_WR', 'FIFO_RD', 'FIFO_FLAGS', 'STALL_RD_GAP', 'STALL_RD_GAPS16',
         'STALL_RD_WORDS', 'STALL_RD_CMDLOW', 'STALL_WR_RUN', 'STALL_WR_RUNS16', 'STALL_WR_WORDS',
         'BW_SEQ_WR', 'BW_SEQ_RD', 'BW_RND_WR', 'BW_RND_RD', 'BW_MIX']
LAT_MODES = ['seq', 'rand', 'rowmiss', 'burst8_seq', 'burst8_rand', 'loaded']
for m in LAT_MODES:
    NAMES += [f'LAT_{m}_MINMAX', f'LAT_{m}_SUM', f'LAT_{m}_CMAX']
NAMES += ['LAST_burst8_seq_MINMAX', 'LAST_burst8_seq_SUM', 'LAST_burst8_rand_MINMAX', 'LAST_burst8_rand_SUM']
HIST_MODES = ['seq', 'rand', 'rowmiss', 'burst8_seq', 'loaded']
for m in HIST_MODES:
    NAMES += [f'HIST_{m}_{2 * i}_{2 * i + 1}' for i in range(8)]
NAMES += ['FIFO_WR_T', 'FIFO_RD_T', 'FIFO_RD_RET']
assert len(NAMES) == NS, len(NAMES)
IDX = {n: i for i, n in enumerate(NAMES)}
# the histogram bins' lower edges (wall clocks of 10 ns), tango_ddr3_tester.v bin16
BIN_LO = [0, 16, 20, 24, 28, 32, 40, 48, 64, 80, 96, 128, 192, 256, 512, 1024]
# the marker colours (tango_ddr3_ov.v mcol)
def marker_colour(s):
    if s == 0: return (255, 255, 255)
    if s < 12: return (0x00, 0xC0, 0xC0)
    if s < 26: return (0xF0, 0x70, 0x00)
    if s < 34: return (0xD0, 0x00, 0xD0)
    if s < 37: return (0xE0, 0xE0, 0x00)
    if s < 44: return (0x00, 0xC0, 0x00)
    if s < 49: return (0x20, 0x40, 0xFF)
    if s < 71: return (0x60, 0xA0, 0xFF)
    if s < 111: return (0x90, 0x40, 0xC0)
    return (0xE0, 0xE0, 0x00)
# the sizes the board build uses (tango_ddr3_test_top.v -> tango_ddr3_test_core defaults)
NLAT = 1 << 15
NBW = 1 << 20
STALL_WIN = 1 << 20
