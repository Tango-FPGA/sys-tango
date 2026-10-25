// tb_pad_decode.cpp -- unit bench for tango_pad_decode.v.
//
// Replays sim/pads/pad_vectors.txt, which the firmware's (menu-tango's) bl616/tango/test/pd_vectors.c writes from the C reference decode (pd_eval).
// The two clocks are asynchronous and not in phase: clk = 48 MHz (the USB host), wclk = 74.25 MHz (clk_pix, where the link writes the
// table).  For every table the bench does what tango_link will do: raise cfg_hold, write the entries and the header, wait, lower
// cfg_hold.  For every report it flips rpt_tog as usb_fs_host does and checks, against the vector:
//   - accepted reports flip raw_tog exactly once, raw equals the expected word, and the latency (clk edges from the flip to the new
//     word) is count + a few;
//   - rejected reports (report id, length) flip nothing and leave raw alone.
// Then hand-written scenarios: mode 0 (off), report held across a table change (re-evaluated), report arriving while the table is
// being written, the host overwriting its bytes during a run, quiet mode, an empty table.
//
// Prints "PAD_DECODE PASS" or "PAD_DECODE FAIL".
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include "Vtango_pad_decode.h"
#include "verilated.h"

static const long CLK_PS = 20833, WCLK_PS = 13468;     // 48 MHz, 74.25 MHz
static Vtango_pad_decode *d;
static long now_ps = 0, next_clk = CLK_PS / 2, next_wclk = 4000;     // wclk phase offset
static long clk_edges = 0, wclk_edges = 0;
static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails < 40) { printf("  FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

struct Entry { uint64_t w; };
struct Rep { int len; uint8_t b[16]; int acc; uint32_t raw; };
struct Table { std::string name; int mode, rid_en, rid_val, min_len, count; std::vector<uint64_t> ent; std::vector<Rep> reps; };

// ---- clocks: advance to the next edge of either clock; posedge actions happen in step()
static void tick_one()
{
    bool do_clk = next_clk <= next_wclk, do_w = next_wclk <= next_clk;
    long t = do_clk ? next_clk : next_wclk;
    now_ps = t;
    if (do_clk) { d->clk = !d->clk; next_clk += CLK_PS / 2; if (d->clk) clk_edges++; }
    if (do_w)   { d->wclk = !d->wclk; next_wclk += WCLK_PS / 2; if (d->wclk) wclk_edges++; }
    d->eval();
}
static void run_clk(int n)      { long e = clk_edges + n; while (clk_edges < e) tick_one(); }
static void run_wclk(int n)     { long e = wclk_edges + n; while (wclk_edges < e) tick_one(); }

// ---- inputs are changed right after a rising edge of their own clock, as flops would
static void set_rpt(const uint8_t *b, int len)
{
    for (int w = 0; w < 4; w++) {
        uint32_t v = 0;
        for (int k = 0; k < 4; k++) v |= (uint32_t)b[4 * w + k] << (8 * k);
        d->rpt[w] = v;
    }
    d->rpt_len = len > 255 ? 255 : len;
}

static void sync_to_clk()  { while (!(d->clk && clk_edges >= 0)) tick_one(); long e = clk_edges; while (clk_edges == e) tick_one(); }

static void write_table(const Table &t, int mode_override = -1)
{
    // wclk domain: hold, entries, header, release
    long e = wclk_edges; while (wclk_edges == e) tick_one();
    d->cfg_hold = 1;
    run_wclk(3);
    d->cfg_mode = mode_override >= 0 ? mode_override : t.mode;
    d->cfg_rid_en = t.rid_en; d->cfg_rid_val = t.rid_val; d->cfg_min_len = t.min_len; d->cfg_count = t.count;
    for (size_t i = 0; i < t.ent.size(); i++) {
        d->tw_we = 1; d->tw_addr = i; d->tw_din = t.ent[i];
        run_wclk(1);
    }
    d->tw_we = 0;
    run_wclk(9);
    d->cfg_hold = 0;
}

static int latency_max[65], latency_min[65];

// one report: flip the toggle, wait for the answer; returns the clk edges taken, -1 if raw_tog never flipped
static int send_report(const Rep &r, uint32_t &raw_after, int &flips)
{
    set_rpt(r.b, r.len);
    uint32_t tog0 = d->raw_tog;
    d->rpt_tog = !d->rpt_tog;
    long e0 = clk_edges;
    int flips_seen = 0; int lat = -1;
    for (int n = 0; n < 400; n++) {
        long e = clk_edges; while (clk_edges == e) tick_one();
        if ((uint32_t)d->raw_tog != tog0) { flips_seen++; if (lat < 0) lat = (int)(clk_edges - e0); tog0 = d->raw_tog; }
        if (!d->busy && n > 3 && (lat >= 0 || n > 8)) break;
    }
    run_clk(4);
    if ((uint32_t)d->raw_tog != tog0) { flips_seen++; tog0 = d->raw_tog; }
    raw_after = d->raw;
    flips = flips_seen;
    return lat;
}

static std::vector<Table> load(const char *path)
{
    std::vector<Table> v; FILE *f = fopen(path, "r");
    if (!f) { perror(path); exit(2); }
    char line[512];
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        if (!strncmp(line, "TABLE ", 6)) {
            Table t; char nm[64]; sscanf(line + 6, "%63s %d %d %d %d %d", nm, &t.mode, &t.rid_en, &t.rid_val, &t.min_len, &t.count);
            t.name = nm; v.push_back(t);
        } else if (!strncmp(line, "ENT ", 4)) {
            v.back().ent.push_back(strtoull(line + 4, nullptr, 16));
        } else if (!strncmp(line, "REPORT ", 7)) {
            Rep r; char hex[64]; unsigned raw;
            sscanf(line + 7, "%d %32s %d %x", &r.len, hex, &r.acc, &raw); r.raw = raw;
            for (int i = 0; i < 16; i++) { char h[3] = { hex[2 * i], hex[2 * i + 1], 0 }; r.b[i] = (uint8_t)strtoul(h, nullptr, 16); }
            v.back().reps.push_back(r);
        }
    }
    fclose(f);
    return v;
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    const char *vec = argc > 1 && argv[1][0] != '+' ? argv[1] : "sim/pads/pad_vectors.txt";
    d = new Vtango_pad_decode;
    d->clk = 0; d->wclk = 0; d->rst_n = 0; d->rpt_valid = 0; d->rpt_tog = 0; d->tw_we = 0; d->cfg_hold = 1; d->cfg_mode = 0; d->cfg_count = 0;
    d->cfg_rid_en = 0; d->cfg_rid_val = 0; d->cfg_min_len = 0; d->tw_addr = 0; d->tw_din = 0; d->rpt_len = 0;
    uint8_t z[16] = { 0 }; set_rpt(z, 0);
    d->eval();
    run_clk(6); d->rst_n = 1; run_clk(6);
    d->rpt_valid = 1;
    for (int i = 0; i <= 64; i++) { latency_max[i] = 0; latency_min[i] = 1 << 30; }

    std::vector<Table> tabs = load(vec);
    long nrep = 0, nacc = 0, nrej = 0;
    int worst_over = 0;
    for (const Table &t : tabs) {
        write_table(t);
        run_clk(12);                                   // the synchronised hold falls; the held report is evaluated again (ignored here)
        while (d->busy) run_clk(1);
        run_clk(4);
        for (const Rep &r : t.reps) {
            uint32_t before = d->raw, after; int flips;
            int lat = send_report(r, after, flips);
            nrep++;
            if (r.acc) {
                nacc++;
                CHECK(flips == 1, "%s: accepted report flipped raw_tog %d times", t.name.c_str(), flips);
                CHECK(after == r.raw, "%s: raw %08x want %08x (len %d)", t.name.c_str(), after, r.raw, r.len);
                if (lat >= 0 && t.mode == 1 && t.count > 0) {
                    int c = t.count;
                    latency_max[c] = std::max(latency_max[c], lat); latency_min[c] = std::min(latency_min[c], lat);
                    worst_over = std::max(worst_over, lat - c);
                }
            } else {
                nrej++;
                CHECK(flips == 0, "%s: rejected report flipped raw_tog (len %d)", t.name.c_str(), r.len);
                CHECK(after == before, "%s: rejected report changed raw %08x -> %08x", t.name.c_str(), before, after);
            }
            run_clk(40 + (int)(r.len % 7));
        }
    }
    printf("vectors: %zu tables, %ld reports (%ld accepted, %ld rejected)\n", tabs.size(), nrep, nacc, nrej);
    int shown = 0;
    printf("latency, clk48 edges from the report toggle to the new word (min..max by table size):");
    for (int c = 1; c <= 64; c++) if (latency_max[c]) { if (shown++ < 14) printf("  n=%d: %d..%d", c, latency_min[c], latency_max[c]); }
    printf("\nworst latency above the entry count: %d clk (%.0f ns at 48 MHz)\n", worst_over, worst_over * 20.833);
    CHECK(worst_over <= 8, "latency above count + 8");

    // ---------------------------------------------------------------- scenarios
    printf("scenarios\n");
    auto find = [&](const char *n) -> const Table & { for (const Table &t : tabs) if (t.name == n) return t; fprintf(stderr, "no table %s\n", n); exit(2); };
    const Table &ds4 = find("ds4"), &xin = find("xinput"), &quiet = find("quiet"), &empty = find("empty");
    // find a DS4 report that is accepted and non-zero, and an xinput one
    const Rep *rd = nullptr, *rx = nullptr, *rd0 = nullptr;
    for (const Rep &r : ds4.reps) if (r.acc && r.raw != 0 && !rd) rd = &r;
    for (const Rep &r : ds4.reps) if (r.acc && r.raw != 0 && rd && &r != rd && r.raw != rd->raw && !rd0) rd0 = &r;
    for (const Rep &r : xin.reps) if (r.acc && r.raw != 0 && !rx) rx = &r;
    CHECK(rd && rx && rd0, "scenario reports found");

    {   // (a) mode 0: the block is off; raw_valid low; reports do nothing
        write_table(ds4, 0); run_clk(12);
        CHECK(!d->raw_valid, "mode 0: raw_valid low");
        uint32_t after; int fl; send_report(*rd, after, fl);
        CHECK(fl == 0, "mode 0: no evaluation");
    }
    {   // (b) a report held across a table change is evaluated again by the new table, with no new report
        write_table(ds4); run_clk(12); while (d->busy) run_clk(1);
        uint32_t after; int fl; send_report(*rd, after, fl);
        CHECK(fl == 1 && after == rd->raw, "ds4 table, report: raw %08x want %08x", after, rd->raw);
        CHECK(d->raw_valid, "table mode: raw_valid high");
        // the same bytes under the XInput table: whatever pd_eval(xinput) says; use the vector file's own xinput report for the expectation
        write_table(quiet); run_clk(30); while (d->busy) run_clk(1); run_clk(4);
        CHECK(d->raw == 0, "quiet table over a held report: raw %08x", d->raw);
        write_table(ds4); run_clk(30); while (d->busy) run_clk(1); run_clk(4);
        CHECK(d->raw == rd->raw, "back to the ds4 table: the held report is decoded again with no new report: raw %08x want %08x", d->raw, rd->raw);
    }
    {   // (c) a report arriving while the table is being written is not lost
        write_table(ds4); run_clk(30); while (d->busy) run_clk(1);
        set_rpt(rd0->b, rd0->len);
        // start a table write, flip the toggle inside it
        long e = wclk_edges; while (wclk_edges == e) tick_one();
        d->cfg_hold = 1; run_wclk(4);
        d->rpt_tog = !d->rpt_tog;                   // the report lands while the table is in flux
        run_clk(20);
        d->tw_we = 1; for (size_t i = 0; i < ds4.ent.size(); i++) { d->tw_addr = i; d->tw_din = ds4.ent[i]; run_wclk(1); }
        d->tw_we = 0; run_wclk(9); d->cfg_hold = 0;
        run_clk(30); while (d->busy) run_clk(1); run_clk(4);
        CHECK(d->raw == rd0->raw, "report during the table write is applied after it: raw %08x want %08x", d->raw, rd0->raw);
    }
    {   // (d) the host overwrites its bytes during a run: the answer is for the report that started it
        write_table(ds4); run_clk(30); while (d->busy) run_clk(1);
        set_rpt(rd->b, rd->len);
        d->rpt_tog = !d->rpt_tog;
        run_clk(3);
        uint8_t junk[16]; for (int i = 0; i < 16; i++) junk[i] = 0xFF - i;
        set_rpt(junk, 16);                          // while the engine is walking the table
        run_clk(60); while (d->busy) run_clk(1); run_clk(4);
        CHECK(d->raw == rd->raw, "bytes overwritten mid-run: raw %08x want %08x", d->raw, rd->raw);
    }
    {   // (e) quiet mode: a toggle per report, word 0
        write_table(quiet); run_clk(30); while (d->busy) run_clk(1);
        uint32_t after; int fl; send_report(*rd, after, fl);
        CHECK(fl == 1 && after == 0 && d->raw_valid, "quiet: word 0, one toggle, valid (flips %d raw %08x)", fl, after);
    }
    {   // (f) empty table
        write_table(empty); run_clk(30); while (d->busy) run_clk(1);
        uint32_t after; int fl; send_report(*rd, after, fl);
        CHECK(after == 0, "empty table: word 0 (%08x)", after);
    }
    {   // (g) the stream goes away: raw_valid drops with rpt_valid; no evaluation while it is low
        write_table(ds4); run_clk(30); while (d->busy) run_clk(1);
        d->rpt_valid = 0; run_clk(2);
        CHECK(!d->raw_valid, "rpt_valid low -> raw_valid low");
        uint32_t after; int fl; send_report(*rd, after, fl);
        CHECK(fl == 0, "no evaluation while the stream is not valid");
        d->rpt_valid = 1; run_clk(2);
    }
    {   // (g2) a report that lands while a run is going is not lost: the second one is decoded after the first
        write_table(ds4); run_clk(30); while (d->busy) run_clk(1);
        set_rpt(rd->b, rd->len); d->rpt_tog = !d->rpt_tog;
        run_clk(6);
        CHECK(d->busy, "the first run is still going");
        set_rpt(rd0->b, rd0->len); d->rpt_tog = !d->rpt_tog;           // 6 clocks later: inside the run
        run_clk(80); while (d->busy) run_clk(1); run_clk(4);
        CHECK(d->raw == rd0->raw, "report during a run: the later report wins: raw %08x want %08x", d->raw, rd0->raw);
    }
    {   // (h) 50 table swaps back and forth with reports in between
        for (int k = 0; k < 50; k++) {
            const Table &t = (k & 1) ? ds4 : xin; const Rep &r = (k & 1) ? *rd : *rx;
            write_table(t); run_clk(30); while (d->busy) run_clk(1);
            uint32_t after; int fl; send_report(r, after, fl);
            CHECK(fl == 1 && after == r.raw, "swap %d: raw %08x want %08x flips %d", k, after, r.raw, fl);
        }
    }
    printf("%d checks, %d failed\n", checks, fails);
    printf(fails ? "PAD_DECODE FAIL\n" : "PAD_DECODE PASS\n");
    delete d;
    return fails ? 1 : 0;
}
