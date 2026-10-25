// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
//==============================================================================
// tango_sd -- MiSTer's block device for cores (save RAM, disk images), built with TANGO_SD.
// menu-tango docs/spec/tango-spec.html "Core contract" -> "Block device".  Core ports and meanings as Template_MiSTer
// sys/hps_io.sv 128-143 (master 3ea1134): the core raises sd_rd[d] / sd_wr[d] with sd_lba[d] and
// sd_blk_cnt[d] (blocks - 1); sd_ack[d] is high for the whole transfer (hps_io.sv 329-331, 312-316); a read
// hands every byte over as sd_buff_dout with a one-clock sd_buff_wr at sd_buff_addr 0, 1, 2, ... (293-297,
// 401-406); a write sets sd_buff_addr and takes sd_buff_din one clock later (408-413).  sd_buff_addr runs on
// across the blocks of one transfer (14 bits, as hps_io's AW for WIDE = 0).
// The BL616 owns the card, so each 512-byte block crosses the link (tango_link: SDC STATUS / CORE_SECTOR_IN 9 /
// CORE_SECTOR_OUT 10 / INSERTED 4 / INS_LARGE 7):
//   clk_sys engine: pick the lowest drive with a request, sd_ack it; per block: (write) copy the core's block
//   into out_buf, then post {lba + block, write, drive} to the link; wait for the link's "block done";
//   (read) copy in_buf into the core.  After sd_blk_cnt + 1 blocks: sd_ack falls, the post is withdrawn.
//   in_buf: written by the link (clk_pix), read here (clk_sys).  out_buf: written here, read by the link.
// Crossings: the post = tango_cdc_word (seq bit flips per block, so the link can tell a new post from one it
// has served); "block done" and "mounted" = toggles through 3 flops with their data held by the link.
//==============================================================================
`default_nettype none
module tango_sd #(
    parameter integer VDNUM = 1
) (
    // ---- link side (clk_pix)
    input  wire        clk_pix,
    input  wire        in_we,
    input  wire [8:0]  in_wa,
    input  wire [7:0]  in_wd,
    input  wire        in_done_tog,         // flips when a CORE_SECTOR_IN block is complete
    input  wire [8:0]  out_ra,
    output reg  [7:0]  out_rd = 8'd0,
    input  wire        out_done_tog,        // flips when a CORE_SECTOR_OUT block has been read out
    input  wire        mnt_tog,             // flips per INSERTED / INS_LARGE (data held)
    input  wire [3:0]  mnt_drive,
    input  wire        mnt_ro,
    input  wire [63:0] mnt_size,
    output wire [38:0] req_pix,             // {seq, valid, write, drive[3:0], lba[31:0]} in clk_pix
    // ---- core side (clk_sys), hps_io.sv 128-143
    input  wire        clk_sys,
    output reg  [VDNUM-1:0] img_mounted = {VDNUM{1'b0}},
    output reg         img_readonly = 1'b0,
    output reg  [63:0] img_size = 64'd0,
    input  wire [31:0] sd_lba [VDNUM],
    input  wire [5:0]  sd_blk_cnt [VDNUM],
    input  wire [VDNUM-1:0] sd_rd,
    input  wire [VDNUM-1:0] sd_wr,
    output reg  [VDNUM-1:0] sd_ack = {VDNUM{1'b0}},
    output reg  [13:0] sd_buff_addr = 14'd0,
    output reg  [7:0]  sd_buff_dout = 8'd0,
    input  wire [7:0]  sd_buff_din [VDNUM],
    output reg         sd_buff_wr = 1'b0
);
    // ---------------------------------------------------------------- sector buffers
    (* syn_ramstyle = "block_ram" *) reg [7:0] in_buf  [0:511];
    (* syn_ramstyle = "block_ram" *) reg [7:0] out_buf [0:511];
    always @(posedge clk_pix) begin
        if (in_we) in_buf[in_wa] <= in_wd;
        out_rd <= out_buf[out_ra];
    end
    reg        ob_we = 1'b0;  reg [8:0] ob_wa = 9'd0;  reg [7:0] ob_wd = 8'd0;
    reg  [8:0] ib_ra = 9'd0;  reg [7:0] ib_q = 8'd0;
    always @(posedge clk_sys) begin
        if (ob_we) out_buf[ob_wa] <= ob_wd;
        ib_q <= in_buf[ib_ra];
    end

    // ---------------------------------------------------------------- crossings into clk_sys
    reg [2:0] ind_s = 3'd0, outd_s = 3'd0, mnt_s = 3'd0;
    always @(posedge clk_sys) begin
        ind_s <= {ind_s[1:0], in_done_tog}; outd_s <= {outd_s[1:0], out_done_tog}; mnt_s <= {mnt_s[1:0], mnt_tog};
    end
    wire in_done  = ind_s[2] != ind_s[1];
    wire out_done = outd_s[2] != outd_s[1];

    // mounts (hps_io.sv 459-466): one-clock img_mounted, size and read-only alongside
    always @(posedge clk_sys) begin
        img_mounted <= {VDNUM{1'b0}};
        if (mnt_s[2] != mnt_s[1]) begin
            if (mnt_drive < VDNUM) img_mounted[mnt_drive] <= 1'b1;
            img_size <= mnt_size; img_readonly <= mnt_ro;
        end
    end

    // ---------------------------------------------------------------- the engine
    localparam [2:0] S_IDLE = 3'd0, S_FILL = 3'd1, S_POST = 3'd2, S_WAIT = 3'd3, S_COPY = 3'd4, S_NEXT = 3'd5;
    reg [2:0]  st = S_IDLE;
    reg [3:0]  drv = 4'd0;
    reg        wr = 1'b0;
    reg [31:0] lba = 32'd0;
    reg [5:0]  cnt = 6'd0, blk = 6'd0;
    reg [9:0]  i = 10'd0;
    reg [1:0]  ph = 2'd0;
    reg        seq = 1'b0, valid = 1'b0;
    integer d;
    reg        any; reg [3:0] pick;
    always @(*) begin
        any = 1'b0; pick = 4'd0;
        for (d = VDNUM - 1; d >= 0; d = d - 1)
            if (sd_rd[d] | sd_wr[d]) begin any = 1'b1; pick = d[3:0]; end
    end
    always @(posedge clk_sys) begin
        sd_buff_wr <= 1'b0; ob_we <= 1'b0;
        case (st)
            S_IDLE: if (any) begin
                drv <= pick; wr <= sd_wr[pick]; lba <= sd_lba[pick]; cnt <= sd_blk_cnt[pick]; blk <= 6'd0;
                sd_ack <= {VDNUM{1'b0}}; sd_ack[pick] <= 1'b1;
                i <= 10'd0; ph <= 2'd0;
                st <= sd_wr[pick] ? S_FILL : S_POST;
            end
            S_FILL: begin                           // core block -> out_buf, 3 clocks a byte
                ph <= ph + 2'd1;
                if (ph == 2'd0) sd_buff_addr <= {blk[4:0], i[8:0]};
                if (ph == 2'd2) begin
                    ob_we <= 1'b1; ob_wa <= i[8:0]; ob_wd <= sd_buff_din[drv];
                    ph <= 2'd0; i <= i + 10'd1;
                    if (i == 10'd511) st <= S_POST;
                end
            end
            S_POST: begin valid <= 1'b1; seq <= ~seq; st <= S_WAIT; end
            S_WAIT: if (wr ? out_done : in_done) begin
                valid <= 1'b0;
                i <= 10'd0; ph <= 2'd0;
                st <= wr ? S_NEXT : S_COPY;
            end
            S_COPY: begin                           // in_buf -> core, one byte per 3 clocks
                ph <= ph + 2'd1;
                if (ph == 2'd0) ib_ra <= i[8:0];
                if (ph == 2'd2) begin
                    sd_buff_dout <= ib_q; sd_buff_addr <= {blk[4:0], i[8:0]}; sd_buff_wr <= 1'b1;
                    ph <= 2'd0; i <= i + 10'd1;
                    if (i == 10'd511) st <= S_NEXT;
                end
            end
            S_NEXT: begin
                if (blk == cnt) begin sd_ack <= {VDNUM{1'b0}}; st <= S_IDLE; end
                else begin blk <= blk + 6'd1; i <= 10'd0; ph <= 2'd0; st <= wr ? S_FILL : S_POST; end
            end
            default: st <= S_IDLE;
        endcase
    end
    wire [38:0] req_sys = {seq, valid, wr, drv, lba + {26'd0, blk}};
    tango_cdc_word #(.W(39)) u_req (.clk_a(clk_sys), .din(req_sys), .clk_b(clk_pix), .dout(req_pix));
endmodule
`default_nettype wire
