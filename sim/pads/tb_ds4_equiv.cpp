// tb_ds4_equiv.cpp -- see tb_ds4_equiv.v.  Reads the "ds4" table from sim/pads/pad_vectors.txt, programs the engine, then sends N random
// DualShock 4 reports (hat 0..8, random buttons) through both decoders and compares the raw words.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include "Vtb_ds4_equiv.h"
#include "verilated.h"
static Vtb_ds4_equiv *d; static long next_clk = 10416, next_pix = 4000; static long clk_e = 0, pix_e = 0;
static void tick() {
    bool a = next_clk <= next_pix, b = next_pix <= next_clk;
    if (a) { d->clk = !d->clk; next_clk += 10416; if (d->clk) clk_e++; }
    if (b) { d->clk_pix = !d->clk_pix; next_pix += 6734; if (d->clk_pix) pix_e++; }
    d->eval();
}
static void run_clk(int n) { long e = clk_e + n; while (clk_e < e) tick(); }
static void run_pix(int n) { long e = pix_e + n; while (pix_e < e) tick(); }
static uint32_t rs = 0xBADC0DE;
static uint32_t rnd() { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const char *vec = argc > 1 ? argv[1] : "sim/pads/pad_vectors.txt";
    std::vector<uint64_t> ent; int mode = 1, minlen = 8, count = 0;
    { FILE *f = fopen(vec, "r"); if (!f) { perror(vec); return 2; } char l[256]; bool in = false;
      while (fgets(l, sizeof l, f)) {
        if (!strncmp(l, "TABLE ", 6)) { char nm[64]; int ri, rv; int m, mn, c; sscanf(l + 6, "%63s %d %d %d %d %d", nm, &m, &ri, &rv, &mn, &c); in = !strcmp(nm, "ds4"); if (in) { mode = m; minlen = mn; count = c; } }
        else if (in && !strncmp(l, "ENT ", 4)) ent.push_back(strtoull(l + 4, nullptr, 16));
        else if (!strncmp(l, "END", 3)) in = false; }
      fclose(f); }
    if ((int)ent.size() != count || !count) { printf("no ds4 table\n"); return 2; }
    d = new Vtb_ds4_equiv; d->clk = 0; d->clk_pix = 0; d->rst_n = 0; d->rpt_valid = 0; d->rpt_tog = 0; d->tw_we = 0; d->cfg_hold = 1;
    d->cfg_mode = 0; d->cfg_min_len = 0; d->cfg_count = 0; d->rpt64 = 0; d->eval();
    run_clk(6); d->rst_n = 1; run_clk(6);
    // program the table (clk_pix domain)
    { long e = pix_e; while (pix_e == e) tick(); d->cfg_hold = 1; run_pix(3); d->cfg_mode = mode; d->cfg_min_len = minlen; d->cfg_count = count;
      for (size_t i = 0; i < ent.size(); i++) { d->tw_we = 1; d->tw_addr = i; d->tw_din = ent[i]; run_pix(1); }
      d->tw_we = 0; run_pix(9); d->cfg_hold = 0; run_clk(20); }
    d->rpt_valid = 1; run_clk(20);
    long n = 0, bad = 0, nz = 0;
    for (int k = 0; k < 60000; k++) {
        uint64_t r = ((uint64_t)rnd() << 32) | rnd();
        uint8_t b5 = (r >> 40) & 0xFF;
        if (k & 1) b5 = (b5 & 0xF0) | (rnd() % 9);              // real hat values: 0-7 and 8 = released
        r = (r & ~(0xFFull << 40)) | ((uint64_t)b5 << 40);
        d->rpt64 = r;
        d->rpt_tog = !d->rpt_tog;
        run_clk(80);                                              // engine done (25 entries + 4); clk_pix captured the toggle (3 flops)
        run_pix(8);
        uint32_t rw = d->old_rw, eng = d->eng_raw;
        n++; if (rw) nz++;
        if (rw != eng) { if (bad++ < 5) printf("MISMATCH k=%d report %016llx: hardwired rw %08x, engine %08x\n", k, (unsigned long long)r, rw, eng); }
    }
    printf("%ld reports (%ld with a button down): hardwired DS4 decode vs table engine: %ld mismatches\n", n, nz, bad);
    printf(bad ? "DS4_EQUIV FAIL\n" : "DS4_EQUIV PASS\n");
    return bad ? 1 : 0;
}
