// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// usb_fs_host.v -- clean-room minimal USB full-speed host controller.
// Bus reset, SOF keep-alives, scripted enumeration (SET_ADDRESS(1),
// GET_DESCRIPTOR(DEVICE) for VID/PID, SET_CONFIGURATION(1)), then continuous interrupt-IN polling with an
// endpoint scanner (different controllers keep their input report on
// different endpoints -- the DS4 uses EP4).
//
// PROVEN ON SILICON: enumerated a DualShock 4 first-try
// (t=00) and streamed live HID input reports.
//
// Protocol facts from the USB 2.0 spec:
//  - Token: PID + 11 bits {ENDP[3:0],ADDR[6:0]} + CRC5, all LSB-first.
//  - CRC5 poly x^5+x^2+1 (reflected 0x14), init 11111, complemented.
//  - CRC16 poly x^16+x^15+x^2+1 (reflected 0xA001), init FFFF,
//    complemented, appended little-endian.
//  - FS frames are 1 ms; a host must emit SOF each frame or the device
//    suspends after 3 ms of idle.
//  - Control write: SETUP+DATA0 -> ACK; status = IN -> DATA1(len 0) -> ACK.
`default_nettype none
module usb_fs_host (
    input  wire clk48, rst_n,

    // PHY byte interface
    output reg        tx_start,
    output reg [7:0]  tx_data,
    output reg        tx_valid,
    output reg        tx_last,
    input  wire       tx_rd,
    input  wire       tx_busy,
    input  wire       rx_active,
    input  wire       rx_strobe,
    input  wire [7:0] rx_data,
    input  wire       rx_eop,
    output reg        bus_reset,

    input  wire       dev_present,   // debounced: a device (FS: D+ high, LS: D- high)
    input  wire       dev_ls,        // the device is low speed (usb_ls_detect status 1): latched at attach
    output reg        ls,            // to the PHY: this attach runs at low speed
    output reg        tx_keepalive,  // to the PHY: a low-speed keep-alive instead of SOF

    // telemetry
    output wire [3:0] dbg_st,
    output wire [4:0] dbg_ep,
    output reg [7:0]  to_count,      // response timeouts seen
    output reg [7:0]  rx_pkts,       // non-self packets received
    output reg [7:0]  last_pid,      // PID byte of last received packet

    // result surface: bytes 1..8 of the last HID input report
    output reg [63:0] desc_bytes,
    output reg        desc_valid,
    // ⭐ ONE BIT THAT TOGGLES PER ACCEPTED REPORT. desc_bytes is 64 bits of
    // free-running data in THIS clock domain; anything downstream that
    // samples it with its own synchronizers is doing a multi-bit CDC, which
    // is unsafe by construction. This toggle is the safe single-bit crossing:
    // the consumer syncs it, and latches desc_bytes only on its edge, by
    // which time the source has not touched the bytes for several clocks.
    output reg        rpt_tog,
    output reg  [7:0] bad_crc,     // corrupted reports REJECTED -- was the phantom source
    // device identity (GET_DESCRIPTOR DEVICE, bytes 8-11 = idVendor, idProduct little-endian).
    // Set during enumeration, before the first report; 0000 if the fetch failed (then the pad
    // still enumerates and reports).  Stable from before the first rpt_tog flip until re-enumeration.
    output reg [15:0] dev_vid,
    output reg [15:0] dev_pid,
    // what the device is (GET_DESCRIPTOR CONFIGURATION: the first interface's class/subclass/protocol,
    // configuration bytes 14-16): 0 pad (anything full speed that is not a boot keyboard: the DS4 path),
    // 1 boot keyboard (03/01/01, switched to the boot protocol), 2 other low-speed device (not decoded)
    output reg [1:0]  dev_kind,
    // ---- generic pads (the descriptor store and the second report path for tango_pad_decode.v).  Additions only:
    // none of the outputs above changes behaviour.
    // the last CRC-good report of 1 byte or more (the legacy desc_bytes path above still needs 6): payload bytes 0..15 (byte 0 in [7:0], zero
    // beyond the length), its length, a toggle per report (same CDC rule as rpt_tog: the data is untouched for >= 1 ms) and a level
    output reg [127:0] rpt16,
    output reg [7:0]   rpt_len,
    output reg         rpt2_tog,
    output reg         rpt2_valid,
    output reg [15:0]  dev_ver,        // bcdDevice (device descriptor bytes 12, 13); 0000 if not fetched
    // the descriptor store (written here in clk48, read by tango_usb_port's RAM): device @000, configuration @100, report @400
    output reg         ds_we,
    output reg [10:0]  ds_addr,
    output reg [7:0]   ds_data,
    output reg         desc_rdy,       // the descriptor phase is over (stored, or given up, or not run): the poll has started
    output reg [8:0]   len_cfg,        // stored lengths (0: absent)
    output reg [10:0]  len_rpt,
    output reg [7:0]   len_dev,
    output reg [1:0]   desc_trunc      // bit0 configuration longer than stored, bit1 report descriptor longer than stored
);
    // ---------------------------------------------------------------- PIDs
    localparam [7:0] PID_SOF=8'hA5, PID_SETUP=8'h2D, PID_IN=8'h69,
                     PID_OUT=8'hE1, PID_DATA0=8'hC3, PID_DATA1=8'h4B,
                     PID_ACK=8'hD2;

    // ------------------------------------------------------------- CRC fns
    function [4:0] crc5_11(input [10:0] d);  // 11 bits LSB-first
        reg [4:0] c; integer i;
        begin
            c = 5'h1F;
            for (i=0;i<11;i=i+1) begin
                if (c[0] ^ d[i]) c = (c>>1) ^ 5'h14;
                else             c = (c>>1);
            end
            crc5_11 = ~c;
        end
    endfunction
    function [15:0] crc16_step(input [15:0] c, input [7:0] d);
        reg [15:0] x; integer i;
        begin
            x = c;
            for (i=0;i<8;i=i+1) begin
                if (x[0] ^ d[i]) x = (x>>1) ^ 16'hA001;
                else             x = (x>>1);
            end
            crc16_step = x;
        end
    endfunction

    // ------------------------------------------------------ 1ms frame tick
    reg [15:0] us_cnt;
    reg [9:0]  ms_us;
    wire tick_1ms = (us_cnt == 16'd47) && (ms_us == 10'd999);
    always @(posedge clk48 or negedge rst_n) begin
        if (!rst_n) begin us_cnt<=0; ms_us<=0; end
        else if (us_cnt == 16'd47) begin
            us_cnt <= 0;
            ms_us <= (ms_us == 10'd999) ? 10'd0 : ms_us + 10'd1;
        end else us_cnt <= us_cnt + 16'd1;
    end

    // --------------------------------------------------------- TX engine
    reg [7:0] pkt [0:11];
    reg [3:0] pkt_len, pkt_idx;
    reg       pkt_go;
    reg       pkt_sending;
    reg       ka_go;                 // low-speed keep-alive request (instead of an SOF packet)

    always @(posedge clk48 or negedge rst_n) begin
        if (!rst_n) begin
            tx_start<=0; tx_valid<=0; tx_last<=0; pkt_sending<=0; pkt_idx<=0; tx_keepalive<=0;
        end else begin
            tx_start <= 0; tx_keepalive <= 0;
            if (ka_go && !pkt_sending) begin
                pkt_sending <= 1; tx_keepalive <= 1;
            end else if (pkt_go && !pkt_sending) begin
                pkt_sending <= 1; pkt_idx <= 0;
                tx_start <= 1;
                tx_data <= pkt[0]; tx_valid <= 1; tx_last <= (pkt_len == 1);
            end else if (pkt_sending) begin
                if (tx_rd) begin
                    if (pkt_idx + 4'd1 < pkt_len) begin
                        pkt_idx <= pkt_idx + 4'd1;
                        tx_data <= pkt[pkt_idx + 4'd1];
                        tx_last <= (pkt_idx + 4'd2 == pkt_len);
                    end
                end
                if (!tx_busy && !tx_start && !tx_keepalive) begin
                    pkt_sending <= 0; tx_valid <= 0; tx_last <= 0;
                end
            end
        end
    end

    // --------------------------------------------------------- RX capture
    reg [7:0] rxb [0:14];              // PID + 12 data bytes + CRC (a device descriptor's VID/PID are data 8-11)
    reg [7:0] pb [0:15];               // payload bytes 0..15 of the packet being received (generic pads)
    reg [3:0] rx_cnt;
    reg       rx_done;
    // ⭐ RECEIVE-SIDE CRC16 -- THE ROOT-CAUSE FIX FOR PHANTOM BUTTON PRESSES.
    // MEASURED: a report was accepted on PID + length ALONE
    // (rx_cnt >= 9). Nothing checked that the packet was intact, so a
    // corrupted USB DATA packet became a controller report and its garbage
    // bytes became button presses -- ~1 phantom every 3.5 s, in bursts,
    // with the controller untouched. crc16_step already existed in this file
    // but was only ever used for TRANSMIT (line ~271).
    // That is why two CDC fixes did not help: the data was already wrong
    // before it crossed any clock boundary.
    // USB CRC16 is checked by running the same CRC over the data AND the two
    // trailing CRC bytes; an intact packet leaves a constant residual.
    // ⛔ THAT CONSTANT IS 16'hB001 FOR THIS REFLECTED/A001 FORM, NOT 16'h800D.
    // An earlier build used 800D -- the residual for the NON-reflected convention --
    // and it rejected EVERY packet, which killed the controller outright on
    // silicon (no button reached the core). Verified
    // the right way before the rebuild: 200 random payloads of 8..64 bytes
    // all leave 0xB001, and a single bit-flip leaves 0x7CC0, so the check
    // both passes good packets and catches corruption.
    reg [15:0] rx_crc;
    reg        rx_crc_ok;
    // descriptor capture: the FSM names the descriptor being read (cap_kind 1 device, 2
    // configuration) and the offset of this packet's first data byte (cap_off); the bytes that matter are
    // taken by offset as they stream in (any ep0 packet size): device 8-11 = idVendor, idProduct (little-
    // endian), configuration 14-16 = bInterfaceClass / SubClass / Protocol of the first interface
    reg [6:0]  rx_n;                 // bytes of this packet so far (PID included), not capped
    reg [2:0]  cap_kind;             // 1 device, 2 configuration (first 18 bytes, the legacy kind probe), 3 full configuration, 4 HID report descriptor
    reg [6:0]  cap_off;
    reg [7:0]  t_vid_l, t_vid_h, t_pid_l, t_pid_h, t_cls, t_sub, t_pro;
    reg [7:0]  t_ver_l, t_ver_h, t_mps;     // bcdDevice, bMaxPacketSize0 of the device descriptor (written in the receive block only)
    reg        t_clr;                       // one clock, from the FSM
    wire [6:0] cap_o = cap_off + rx_n - 7'd1;
    // ⭐ STICKY: set by the first CRC-valid report and never cleared. Guards the
    // pre-enumeration diagnostic write above. ⛔ NOT desc_valid, which the
    // re-enumeration path clears on any device dropout.
    reg        ever_valid;
    always @(posedge clk48 or negedge rst_n) begin
        if (!rst_n) begin rx_cnt<=0; rx_done<=0; rx_pkts<=0; last_pid<=0;
                          rx_crc<=16'hFFFF; rx_crc_ok<=1'b0; rx_n<=0; end
        else begin
            rx_done <= 0;
            if (t_clr) begin t_mps <= 8'd0; t_ver_l <= 8'd0; t_ver_h <= 8'd0; end    // the FSM: a new device, or the device descriptor was given up
            if (rx_strobe && !pkt_sending) begin
                if (rx_cnt == 4'd0) begin
                    last_pid <= rx_data;
                    rx_crc   <= 16'hFFFF;        // CRC covers data only, not the PID
                end else begin
                    rx_crc   <= crc16_step(rx_crc, rx_data);
                end
                if (rx_cnt != 4'd15) begin rxb[rx_cnt] <= rx_data; rx_cnt <= rx_cnt + 4'd1; end
                if (rx_n != 7'd127) rx_n <= rx_n + 7'd1;
                // payload bytes 0..15 of any packet (pb[k] = packet byte k+1); published to rpt16 only for a CRC-good report (S_GOT_DATA)
                if (rx_n >= 7'd1 && rx_n <= 7'd16) pb[rx_n[3:0] - 4'd1] <= rx_data;
                if (rx_n != 7'd0 && cap_kind == 3'd1) case (cap_o)
                    7'd7: t_mps <= rx_data;
                    7'd8: t_vid_l <= rx_data;  7'd9: t_vid_h <= rx_data;
                    7'd10: t_pid_l <= rx_data; 7'd11: t_pid_h <= rx_data;
                    7'd12: t_ver_l <= rx_data; 7'd13: t_ver_h <= rx_data;
                    default: ;
                endcase
                if (rx_n != 7'd0 && cap_kind == 3'd2) case (cap_o)
                    7'd14: t_cls <= rx_data; 7'd15: t_sub <= rx_data; 7'd16: t_pro <= rx_data;
                    default: ;
                endcase
            end
            if (rx_eop && !pkt_sending) rx_crc_ok <= (rx_crc == 16'hB001);
            if (rx_eop && !pkt_sending) begin rx_done <= 1; rx_pkts <= rx_pkts + 8'd1; end
            if (st_clear_rx) begin rx_cnt <= 0; rx_n <= 0; end
        end
    end
    reg st_clear_rx;

    // ------------------------------------------------------------ main FSM
    localparam S_WAIT_DEV   = 0;
    localparam S_ATTACH_DLY = 1;
    localparam S_RESET      = 2;
    localparam S_POST_RESET = 3;
    localparam S_SOF_IDLE   = 4;
    localparam S_TX_WAIT    = 5;
    localparam S_RX_WAIT    = 6;
    localparam S_SCRIPT     = 12;
    localparam S_SEND_DATA0 = 13;
    localparam S_GOT_DATA   = 14;
    localparam S_GOT_ACK    = 15;
    localparam S_SEND_ZLP   = 16;       // control-read status stage: OUT DATA1 of length 0

    reg [4:0]  st, st_ret_tx, st_ret_rx;
    assign dbg_st = st[3:0];
    reg [7:0]  ms_ctr;
    reg [10:0] frame_no;
    reg [4:0]  script_pc;            // 0-4, 8-15 legacy; 16-21 the generic-pad descriptor phase (below)
    // ---- generic-pad descriptor phase state
    reg [10:0] doff;                 // bytes of the current control read stored so far
    reg [7:0]  exp_pid;              // the data PID the next control-read packet must carry (DATA1 first, then toggling)
    reg [15:0] rq_wlen;              // wLength of the report-descriptor request; wIndex = the HID interface
    reg [7:0]  rq_widx;
    reg        ep_from_cfg;          // the polled endpoint came from the configuration descriptor, not the scan
    // Xbox 360 class (interface FF/5D/01) start-up reads, pc22-24: six control reads, each SETUP / data IN (discarded) / status
    // OUT, in the order of FPGA Companion's usbh_xbox_client_thread (bl616/companion/src/bl616/mcu_hw.c 635-698, whose xpad quirk packets
    // QUIRK_360_START_PKT_1..3 these are): string 2 (2 and 32 bytes), class IN interface 0100/0x20, class IN interface 0000/8, class IN
    // device 0000/4, vendor IN interface 0100/20.  A read that fails (STALL, no answer) is given up after DESC_TRIES and the next one runs.
    reg        is_xin;               // the first interface is class FF subclass 5D protocol 01
    reg [2:0]  qi;
    reg [5:0]  ep_miss;
    reg [15:0] rx_toctr;
    reg [16:0] rx_hardcap;
    reg [10:0] fail_streak;
    reg        sof_due;

    // ---------------------------------------------------------------- descriptor store writer + configuration walker
    // The payload bytes of a control-read data packet go to the store as they stream in, delayed by two bytes so the two CRC bytes are never
    // written (payload byte p, packet index p + 1, is written when packet index p + 3 arrives).  cap_kind 1 = device descriptor
    // (offset cap_off), 3 = configuration, 4 = HID report descriptor (offset doff).  A duplicate data packet (the device did not see our
    // ACK: its PID is not the expected toggle) is not stored.  A packet with a bad CRC is stored too, but its offset is not advanced and the
    // device (not ACKed) resends it over the same place; the configuration walker below cannot take bytes back, so a bad CRC in the
    // configuration restarts that read (S_GOT_DATA).
    reg [7:0]  d1, d2;
    reg        wk_v;  reg [7:0] wk_b;
    wire [10:0] w_off = (cap_kind == 3'd1) ? {4'd0, cap_off + rx_n - 7'd3} : (doff + {4'd0, rx_n} - 11'd3);
    // control-read data packets alternate DATA1, DATA0 ...; the FIRST packet of a read is taken whatever its PID, a later one with the old
    // toggle is a duplicate (the device missed our ACK)
    wire        first_pkt = (cap_kind == 3'd1 || cap_kind == 3'd2) ? (cap_off == 7'd0) : (doff == 11'd0);
    wire        w_dup = (cap_kind != 3'd0) && !first_pkt && (rxb[0] != exp_pid);
    wire        w_pidok = (rxb[0] == PID_DATA0) || (rxb[0] == PID_DATA1);
    always @(posedge clk48 or negedge rst_n) begin
        if (!rst_n) begin ds_we <= 1'b0; ds_addr <= 11'd0; ds_data <= 8'd0; wk_v <= 1'b0; d1 <= 8'd0; d2 <= 8'd0; wk_b <= 8'd0; end
        else begin
            ds_we <= 1'b0; wk_v <= 1'b0;
            if (rx_strobe && !pkt_sending) begin
                d1 <= rx_data; d2 <= d1;
                if (rx_n >= 7'd3 && w_pidok && !w_dup) begin
                    if (cap_kind == 3'd1 && w_off < 11'd32) begin ds_we <= 1'b1; ds_data <= d2; ds_addr <= {3'b000, w_off[7:0]}; end
                    if (cap_kind == 3'd3 && w_off < 11'd256) begin ds_we <= 1'b1; ds_data <= d2; ds_addr <= {3'b001, w_off[7:0]}; wk_v <= 1'b1; wk_b <= d2; end
                    if (cap_kind == 3'd4 && w_off < 11'd1024) begin ds_we <= 1'b1; ds_data <= d2; ds_addr <= {1'b1, w_off[9:0]}; end
                end
            end
        end
    end
    // the walker: one registered byte at a time, a descriptor header (bLength, bDescriptorType) then its fields by index
    reg        walk_rst;                                  // request from the FSM: start a fresh walk
    reg [7:0]  w_rem, w_i, w_type, cur_if, cur_alt, cur_cls, cur_sub, ep_a;
    reg        h_sel, h_found, ep_found;
    reg [7:0]  h_if;  reg [15:0] h_rlen, cfg_total;  reg [3:0] h_ep;
    always @(posedge clk48 or negedge rst_n) begin
        if (!rst_n) begin
            w_rem <= 8'd0; w_i <= 8'd0; w_type <= 8'd0; cur_if <= 8'd0; cur_alt <= 8'd0; cur_cls <= 8'd0; cur_sub <= 8'd0; ep_a <= 8'd0;
            h_sel <= 1'b0; h_found <= 1'b0; ep_found <= 1'b0; h_if <= 8'd0; h_rlen <= 16'd0; cfg_total <= 16'd0; h_ep <= 4'd0;
        end else if (walk_rst) begin
            w_rem <= 8'd0; w_i <= 8'd0; w_type <= 8'd0; cur_if <= 8'd0; cur_alt <= 8'd0; cur_cls <= 8'd0; cur_sub <= 8'd0; ep_a <= 8'd0;
            h_sel <= 1'b0; h_found <= 1'b0; ep_found <= 1'b0; h_if <= 8'd0; h_rlen <= 16'd0; cfg_total <= 16'd0; h_ep <= 4'd0;
        end else if (wk_v) begin
            if (w_rem == 8'd0) begin                      // the first byte of a descriptor: bLength (0 is malformed: one byte consumed, go on)
                w_rem <= (wk_b == 8'd0) ? 8'd0 : wk_b - 8'd1; w_i <= 8'd1;
            end else begin
                w_rem <= w_rem - 8'd1; w_i <= w_i + 8'd1;
                if (w_i == 8'd1) w_type <= wk_b;
                else case (w_type)
                    8'd2:  begin if (w_i == 8'd2) cfg_total[7:0] <= wk_b; else if (w_i == 8'd3) cfg_total[15:8] <= wk_b; end
                    8'd4:  begin                          // interface: number, alternate setting, class (the first HID interface at alternate 0 wins)
                               if (w_i == 8'd2) cur_if <= wk_b;
                               else if (w_i == 8'd3) cur_alt <= wk_b;
                               else if (w_i == 8'd5) begin cur_cls <= wk_b; h_sel <= 1'b0; end
                               else if (w_i == 8'd6) cur_sub <= wk_b;
                               else if (w_i == 8'd7) begin     // protocol: the first HID interface at alternate 0 that is not a boot keyboard / mouse
                                   if (cur_cls == 8'd3 && cur_alt == 8'd0 && !h_found && !(cur_sub == 8'd1 && wk_b != 8'd0)) begin
                                       h_found <= 1'b1; h_if <= cur_if; h_sel <= 1'b1;
                                   end
                               end
                           end
                    8'h21: begin if (h_sel && w_i == 8'd7) h_rlen[7:0] <= wk_b; else if (h_sel && w_i == 8'd8) h_rlen[15:8] <= wk_b; end
                    8'd5:  if (h_sel && !ep_found) begin  // endpoint of the chosen interface: the first interrupt IN
                               if (w_i == 8'd2) ep_a <= wk_b;
                               else if (w_i == 8'd3 && ep_a[7] && wk_b[1:0] == 2'd3 && ep_a[3:0] != 4'd0) begin ep_found <= 1'b1; h_ep <= ep_a[3:0]; end
                           end
                    default: ;
                endcase
            end
        end
    end
    wire [10:0] want_rlen = (h_rlen > 16'd1024) ? 11'd1024 : h_rlen[10:0];
    wire [7:0]  mps_eff   = (t_mps >= 8'd8) ? t_mps : 8'd8;
    wire [7:0]  qlen      = (qi == 3'd0) ? 8'd2 : (qi == 3'd3) ? 8'd8 : (qi == 3'd4) ? 8'd4 : (qi == 3'd5) ? 8'd20 : 8'd32;

    // enumeration script:
    //  pc0: SET_ADDRESS(1)   SETUP+DATA0 -> ACK
    //  pc1: status IN (addr0) -> DATA1 -> we ACK      [device now addr 1]
    //  pc2: SET_CONFIGURATION(1) SETUP+DATA0 (addr1) -> ACK
    //  pc3: status IN (addr1) -> DATA1 -> we ACK
    //  pc4: forever: IN poll addr1, scanned ep -> NAK (skip) or DATAx -> ACK
    // inserted between pc1 and pc2 (Define buttons: MiSTer keys maps by VID/PID):
    //  pc8: GET_DESCRIPTOR(DEVICE, 18) SETUP+DATA0 (addr1) -> ACK
    //  pc9: data IN (addr1 ep0) -> DATAx, repeated until data bytes 8-11 are in (a 64-byte ep0
    //       brings all 18 at once; an 8-byte ep0 needs two packets) -> we ACK
    //  pc10: status OUT (addr1 ep0) + DATA1 length 0 -> ACK   (the host may end a control read's
    //       data stage early: USB 2.0 8.5.3)
    //  then pc2.  More than DESC_TRIES tries in pc8-10 (STALL, no answer): VID = PID = 0, go on to pc2.
    localparam [7:0] REQ_SETADDR [0:7] = '{8'h00,8'h05,8'h01,8'h00,8'h00,8'h00,8'h00,8'h00};
    localparam [7:0] REQ_SETCFG  [0:7] = '{8'h00,8'h09,8'h01,8'h00,8'h00,8'h00,8'h00,8'h00};
    localparam [7:0] REQ_GETDEV  [0:7] = '{8'h80,8'h06,8'h00,8'h01,8'h00,8'h00,8'h12,8'h00};
    // keyboards: GET_DESCRIPTOR(CONFIGURATION, 18) for the first interface's class, and for a
    // boot keyboard (03/01/01) SET_PROTOCOL(boot) on interface 0 (HID 1.11 7.2.6: bmRequestType 21, bRequest
    // 0B, wValue 0 = boot, wIndex = interface)
    localparam [7:0] REQ_GETCFG  [0:7] = '{8'h80,8'h06,8'h00,8'h02,8'h00,8'h00,8'h12,8'h00};
    localparam [7:0] REQ_SETPROTO[0:7] = '{8'h21,8'h0B,8'h00,8'h00,8'h00,8'h00,8'h00,8'h00};
    //  script with them: pc0 SET_ADDRESS, pc1 status IN, pc8 GET_DESCRIPTOR(DEVICE), pc9 data IN (until
    //  bytes 8-11), pc10 status OUT, pc11 GET_DESCRIPTOR(CONFIGURATION), pc12 data IN (until bytes 14-16),
    //  pc13 status OUT, pc2 SET_CONFIGURATION, pc3 status IN, [keyboard: pc14 SET_PROTOCOL, pc15 status IN],
    //  pc4 poll.  More than DESC_TRIES tries in pc8-13: VID/PID 0000, kind by speed, on to pc2; in pc14-15:
    //  on to pc4.
    localparam [7:0] DESC_TRIES = 8'd40;
    reg [7:0]  desc_tries;

    // interrupt-IN endpoint scan: a poll that TIMES OUT means "no such
    // endpoint" -> try the next; NAK or DATA identifies the live endpoint.
    reg [3:0] poll_ep;
    reg       ep_locked;
    assign dbg_ep = {ep_locked, poll_ep};

    function [10:0] tok_ae(input [4:0] pc);
        case (pc)
            4'd0, 4'd1: tok_ae = 11'h000;             // addr0 ep0
            4'd2, 4'd3, 4'd8, 4'd9, 4'd10, 4'd11, 4'd12, 4'd13, 4'd14, 4'd15,
            5'd16, 5'd17, 5'd18, 5'd19, 5'd20, 5'd21, 5'd22, 5'd23, 5'd24: tok_ae = {4'd0, 7'd1};   // addr1 ep0
            default:    tok_ae = {poll_ep, 7'd1};     // addr1, scanned ep
        endcase
    endfunction

    // generic pads: pc16 GET_DESCRIPTOR(CONFIGURATION, wLength 255), pc19 GET_DESCRIPTOR(HID REPORT 0x22, wIndex = interface, wLength)
    localparam [7:0] REQ_GETCFGF [0:7] = '{8'h80,8'h06,8'h00,8'h02,8'h00,8'h00,8'hFF,8'h00};
    function [7:0] req_byte(input [4:0] pc, input [2:0] b);
        case (pc)
            5'd16:   req_byte = REQ_GETCFGF[b];
            5'd22:   case (qi)
                         3'd0: case (b) 3'd0: req_byte = 8'h80; 3'd1: req_byte = 8'h06; 3'd2: req_byte = 8'h02; 3'd3: req_byte = 8'h03; 3'd4: req_byte = 8'h09; 3'd5: req_byte = 8'h04; 3'd6: req_byte = 8'h02; default: req_byte = 8'h00; endcase
                         3'd1: case (b) 3'd0: req_byte = 8'h80; 3'd1: req_byte = 8'h06; 3'd2: req_byte = 8'h02; 3'd3: req_byte = 8'h03; 3'd4: req_byte = 8'h09; 3'd5: req_byte = 8'h04; 3'd6: req_byte = 8'h20; default: req_byte = 8'h00; endcase
                         3'd2: case (b) 3'd0: req_byte = 8'hA1; 3'd1: req_byte = 8'h01; 3'd2: req_byte = 8'h00; 3'd3: req_byte = 8'h01; 3'd4: req_byte = 8'h00; 3'd5: req_byte = 8'h00; 3'd6: req_byte = 8'h20; default: req_byte = 8'h00; endcase
                         3'd3: case (b) 3'd0: req_byte = 8'hA1; 3'd1: req_byte = 8'h01; 3'd2: req_byte = 8'h00; 3'd3: req_byte = 8'h00; 3'd4: req_byte = 8'h00; 3'd5: req_byte = 8'h00; 3'd6: req_byte = 8'h08; default: req_byte = 8'h00; endcase
                         3'd4: case (b) 3'd0: req_byte = 8'hA0; 3'd1: req_byte = 8'h01; 3'd2: req_byte = 8'h00; 3'd3: req_byte = 8'h00; 3'd4: req_byte = 8'h00; 3'd5: req_byte = 8'h00; 3'd6: req_byte = 8'h04; default: req_byte = 8'h00; endcase
                         default: case (b) 3'd0: req_byte = 8'hC1; 3'd1: req_byte = 8'h01; 3'd2: req_byte = 8'h00; 3'd3: req_byte = 8'h01; 3'd4: req_byte = 8'h00; 3'd5: req_byte = 8'h00; 3'd6: req_byte = 8'h14; default: req_byte = 8'h00; endcase
                     endcase
            5'd19:   case (b) 3'd0: req_byte = 8'h81; 3'd1: req_byte = 8'h06; 3'd2: req_byte = 8'h00; 3'd3: req_byte = 8'h22;
                              3'd4: req_byte = rq_widx; 3'd5: req_byte = 8'h00; 3'd6: req_byte = rq_wlen[7:0]; default: req_byte = rq_wlen[15:8]; endcase
            4'd0:    req_byte = REQ_SETADDR[b];
            4'd8:    req_byte = REQ_GETDEV[b];
            4'd11:   req_byte = REQ_GETCFG[b];
            4'd14:   req_byte = REQ_SETPROTO[b];
            default: req_byte = REQ_SETCFG[b];
        endcase
    endfunction
    integer k;
    reg [15:0] c16;
    reg [4:0]  c5;
    reg [10:0] ae;
    reg        ack_ok;
    always @(posedge clk48 or negedge rst_n) begin
        if (!rst_n) begin
            st<=S_WAIT_DEV; bus_reset<=0; pkt_go<=0; ms_ctr<=0; frame_no<=0;
            script_pc<=0; desc_valid<=0; ever_valid<=0; rpt_tog<=1'b0; bad_crc<=8'd0; to_count<=0; st_clear_rx<=0; sof_due<=0;
            poll_ep<=4'd1; ep_locked<=0; fail_streak<=0;
            dev_vid<=16'd0; dev_pid<=16'd0; desc_tries<=8'd0; ka_go<=0; ls<=0; dev_kind<=2'd0;
            cap_kind<=3'd0; cap_off<=7'd0;
            rpt16<=128'd0; rpt_len<=8'd0; rpt2_tog<=1'b0; rpt2_valid<=1'b0; dev_ver<=16'd0; desc_rdy<=1'b0;
            len_cfg<=9'd0; len_rpt<=11'd0; len_dev<=8'd0; desc_trunc<=2'b00; doff<=11'd0; exp_pid<=PID_DATA1; rq_wlen<=16'd0; rq_widx<=8'd0;
            ep_from_cfg<=1'b0; ep_miss<=6'd0; walk_rst<=1'b0; t_clr<=1'b0; is_xin<=1'b0; qi<=3'd0;
        end else begin
            ka_go <= 0;
            pkt_go <= 0; st_clear_rx <= 0; walk_rst <= 1'b0; t_clr <= 1'b0;
            if (tick_1ms) sof_due <= 1;
            // the poll has started (every route to pc4: the generic descriptor phase done, given up, or not run for this device): the BL616 may read the store
            if (script_pc == 5'd4) desc_rdy <= 1'b1;

            case (st)
            S_WAIT_DEV: begin
                bus_reset <= 0;
                dev_vid <= 16'd0; dev_pid <= 16'd0; desc_tries <= 8'd0; dev_kind <= 2'd0;
                cap_kind <= 3'd0; cap_off <= 7'd0;
                desc_rdy <= 1'b0; len_cfg <= 9'd0; len_rpt <= 11'd0; len_dev <= 8'd0; desc_trunc <= 2'b00; dev_ver <= 16'd0;
                ep_from_cfg <= 1'b0; ep_miss <= 6'd0; t_clr <= 1'b1; walk_rst <= 1'b1; is_xin <= 1'b0; qi <= 3'd0;
                if (dev_present) begin st <= S_ATTACH_DLY; ms_ctr <= 0; ls <= dev_ls; end
            end
            S_ATTACH_DLY: begin
                if (tick_1ms) begin
                    ms_ctr <= ms_ctr + 8'd1;
                    if (ms_ctr == 8'd100) begin st <= S_RESET; ms_ctr <= 0; bus_reset <= 1; end
                end
            end
            S_RESET: begin
                if (tick_1ms) begin
                    ms_ctr <= ms_ctr + 8'd1;
                    if (ms_ctr == 8'd15) begin
                        bus_reset <= 0; st <= S_POST_RESET; ms_ctr <= 0; sof_due <= 0;
                        frame_no <= 0;
                    end
                end
            end
            S_POST_RESET: begin
                if (sof_due) begin
                    sof_due <= 0;
                    ms_ctr <= ms_ctr + 8'd1;
                    c5 = crc5_11(frame_no);
                    pkt[0] <= PID_SOF;
                    pkt[1] <= frame_no[7:0];
                    pkt[2] <= {c5, frame_no[10:8]};
                    pkt_len <= 4'd3; frame_no <= frame_no + 11'd1;
                    if (ls) ka_go <= 1; else pkt_go <= 1;   // low speed: keep-alive (USB 2.0 11.8.4.1)
                    if (ms_ctr == 8'd20) begin st <= S_SOF_IDLE; script_pc <= 0; end
                end
            end
            S_SOF_IDLE: begin
                if (sof_due) begin
                    sof_due <= 0;
                    c5 = crc5_11(frame_no);
                    pkt[0] <= PID_SOF;
                    pkt[1] <= frame_no[7:0];
                    pkt[2] <= {c5, frame_no[10:8]};
                    pkt_len <= 4'd3; frame_no <= frame_no + 11'd1;
                    if (ls) ka_go <= 1; else pkt_go <= 1;
                    st <= S_TX_WAIT; st_ret_tx <= S_SCRIPT;
                end
            end
            S_SCRIPT: begin
                ae = tok_ae(script_pc);
                c5 = crc5_11(ae);
                // descriptor / protocol phase out of tries: give up (VID/PID 0000, kind by speed), go on
                if (script_pc >= 4'd8 && desc_tries >= DESC_TRIES) begin
                    if (script_pc >= 5'd22) begin                       // an Xbox start-up read that failed: the next one
                        if (qi == 3'd5) script_pc <= 5'd4; else begin qi <= qi + 3'd1; script_pc <= 5'd22; end
                    end
                    else if (is_xin && script_pc >= 5'd16) begin script_pc <= 5'd22; qi <= 3'd0; end   // the configuration phase failed: the start-up reads still run
                    else if (script_pc >= 4'd14) script_pc <= 4'd4;
                    else begin
                        script_pc <= 4'd2; dev_kind <= ls ? 2'd2 : 2'd0;
                        // a failed configuration probe (pc11-13) keeps the identity the device descriptor already gave (the low-speed bench's stall=2 run);
                        // only a failed device descriptor read clears VID / PID / bcdDevice
                        if (script_pc < 5'd11) begin dev_vid <= 16'd0; dev_pid <= 16'd0; dev_ver <= 16'd0; t_clr <= 1'b1; len_dev <= 8'd0; end
                    end
                    desc_tries <= 8'd0; cap_kind <= 3'd0;
                    st <= S_SOF_IDLE;
                end else begin
                if (script_pc >= 4'd8) desc_tries <= desc_tries + 8'd1;
                case (script_pc)
                5'd10, 5'd13, 5'd18, 5'd21, 5'd24: begin     // status stage of GET_DESCRIPTOR: OUT token, then DATA1 len 0
                    pkt[0] <= PID_OUT;
                    pkt[1] <= {ae[7], ae[6:0]};
                    pkt[2] <= {c5, ae[10:8]};
                    pkt_len <= 4'd3; pkt_go <= 1;
                    st <= S_TX_WAIT; st_ret_tx <= S_SEND_ZLP;
                end
                5'd0, 5'd2, 5'd8, 5'd11, 5'd14, 5'd16, 5'd19, 5'd22: begin // SETUP token, then DATA0 payload
                    pkt[0] <= PID_SETUP;
                    pkt[1] <= {ae[7], ae[6:0]};
                    pkt[2] <= {c5, ae[10:8]};
                    pkt_len <= 4'd3; pkt_go <= 1;
                    st <= S_TX_WAIT; st_ret_tx <= S_SEND_DATA0;
                end
                default: begin          // pc1/pc3/pc15 status IN, pc9/pc12 descriptor data IN, pc4 report poll
                    if (script_pc == 5'd9 || script_pc == 5'd12) cap_kind <= (script_pc == 5'd9) ? 3'd1 : 3'd2;
                    else if (script_pc == 5'd17) cap_kind <= 3'd3;
                    else if (script_pc == 5'd20) cap_kind <= 3'd4;
                    else if (script_pc == 5'd23) cap_kind <= 3'd5;     // a start-up read: discarded, but the toggle rules apply
                    else cap_kind <= 3'd0;
                    pkt[0] <= PID_IN;
                    pkt[1] <= {ae[7], ae[6:0]};
                    pkt[2] <= {c5, ae[10:8]};
                    pkt_len <= 4'd3; pkt_go <= 1;
                    st_clear_rx <= 1;
                    st <= S_TX_WAIT; st_ret_tx <= S_RX_WAIT;
                    st_ret_rx <= S_GOT_DATA; rx_toctr <= 0; rx_hardcap <= 0;
                end
                endcase
                end
            end
            S_SEND_ZLP: begin
                pkt[0] <= PID_DATA1; pkt[1] <= 8'h00; pkt[2] <= 8'h00;   // CRC16 of no data = 0000
                pkt_len <= 4'd3; pkt_go <= 1;
                st_clear_rx <= 1;
                st <= S_TX_WAIT; st_ret_tx <= S_RX_WAIT;
                st_ret_rx <= S_GOT_ACK; rx_toctr <= 0; rx_hardcap <= 0;
            end
            S_SEND_DATA0: begin
                c16 = 16'hFFFF;
                for (k=0;k<8;k=k+1)
                    c16 = crc16_step(c16, req_byte(script_pc, k[2:0]));
                c16 = ~c16;
                pkt[0] <= PID_DATA0;
                for (k=0;k<8;k=k+1)
                    pkt[1+k] <= req_byte(script_pc, k[2:0]);
                pkt[9]  <= c16[7:0];
                pkt[10] <= c16[15:8];
                pkt_len <= 4'd11; pkt_go <= 1;
                st_clear_rx <= 1;
                st <= S_TX_WAIT; st_ret_tx <= S_RX_WAIT;
                st_ret_rx <= S_GOT_ACK; rx_toctr <= 0; rx_hardcap <= 0;
            end
            S_GOT_ACK: begin
                if (rxb[0] == PID_ACK) case (script_pc)
                    5'd10:   begin script_pc <= 5'd11; cap_off <= 7'd0; end   // device descriptor done
                    5'd13:   script_pc <= 5'd2;                               // configuration descriptor done
                    5'd8:    begin script_pc <= 5'd9; cap_off <= 7'd0; end
                    5'd11:   begin script_pc <= 5'd12; cap_off <= 7'd0; end
                    // generic pads: SETUP of the full configuration read -> its data; its status stage -> the report descriptor (if the
                    // configuration has a HID interface with a report descriptor) or the poll; and the same for the report descriptor
                    5'd16:   begin script_pc <= 5'd17; doff <= 11'd0; exp_pid <= PID_DATA1; walk_rst <= 1'b1; end
                    5'd18:   begin
                                 if (ep_found && h_found) begin poll_ep <= h_ep; ep_locked <= 1'b1; ep_from_cfg <= 1'b1; ep_miss <= 6'd0; end
                                 if (h_found && h_rlen != 16'd0) begin
                                     rq_widx <= h_if; rq_wlen <= {5'd0, want_rlen};
                                     if (h_rlen > 16'd1024) desc_trunc[1] <= 1'b1;
                                     script_pc <= 5'd19;
                                 end else begin script_pc <= is_xin ? 5'd22 : 5'd4; qi <= 3'd0; end
                             end
                    5'd19:   begin script_pc <= 5'd20; doff <= 11'd0; exp_pid <= PID_DATA1; end
                    5'd21:   begin script_pc <= is_xin ? 5'd22 : 5'd4; qi <= 3'd0; end
                    5'd22:   begin script_pc <= 5'd23; doff <= 11'd0; exp_pid <= PID_DATA1; end
                    5'd24:   begin if (qi == 3'd5) script_pc <= 5'd4; else begin qi <= qi + 3'd1; script_pc <= 5'd22; end end
                    default: script_pc <= script_pc + 5'd1;                  // pc0 -> 1, 2 -> 3, 14 -> 15
                endcase
                // pre-enumeration diagnostics: expose the raw response
                // (length + first 7 bytes) in the report field while no
                // real report has been captured yet
                // ⛔⛔ GATED ON ever_valid, NOT desc_valid -- THIS PATH WAS THE
                // REMAINING SOURCE OF PHANTOM BUTTON PRESSES.
                // It writes RAW, UN-CRC'd packet bytes into the very register
                // the pad state is read from. The receive CRC gate
                // guards the REPORT path below; it never guarded this one.
                // ⭐ THE SEQUENCE, as measured: a reflash or ROM load
                // makes the controller miss polls -> fail_streak reaches 1500 ->
                // desc_valid clears -> this line begins writing rxb[] and
                // rx_cnt into desc_bytes -> the device returns, a valid report
                // toggles rpt_tog, and the consumer latches whatever its
                // pipeline holds. Diagnostic bytes become BUTTONS.
                // That is why phantoms appeared AT LOAD TIME and why the held
                // buttons differed every time (Balloon Fight LEFT+UP+A, Kung Fu
                // LEFT): they are packet bytes, not a stuck line.
                // ⭐ ever_valid is STICKY: once a real report has ever landed,
                // this diagnostic never writes again, even if desc_valid is
                // later cleared by the re-enumeration path. The debug value
                // survives for its actual purpose -- first bring-up, before any
                // report exists -- and cannot reach a running game.
                if (!ever_valid)
                    desc_bytes <= {rxb[6],rxb[5],rxb[4],rxb[3],rxb[2],rxb[1],rxb[0],{4'd0,rx_cnt}};
                st <= S_SOF_IDLE;
            end
            S_GOT_DATA: begin
                fail_streak <= 0;   // any response at all = device is alive
                ep_miss <= 6'd0;
                ack_ok = 1'b1;
                if (rxb[0]==PID_DATA1 || rxb[0]==PID_DATA0) begin
                    if (script_pc == 5'd4) begin
                        ep_locked <= 1;
                        // generic pads: ANY CRC-good report of 1 byte or more (the legacy path below needs 6 and is untouched)
                        if (rx_crc_ok && rx_n >= 7'd4) begin
                            for (k = 0; k < 16; k = k + 1) rpt16[8*k +: 8] <= (k < (rx_n - 7'd3)) ? pb[k] : 8'h00;
                            rpt_len <= {1'b0, rx_n - 7'd3};
                            rpt2_valid <= 1'b1;
                            rpt2_tog <= ~rpt2_tog;
                        end
                        // ⛔ rx_crc_ok is the new gate. Length alone let
                        // corrupted packets through as button presses.
                        if (rx_cnt >= 4'd9 && rx_crc_ok) begin
                            desc_bytes <= {rxb[8],rxb[7],rxb[6],rxb[5],rxb[4],rxb[3],rxb[2],rxb[1]};
                            desc_valid <= 1;
                            ever_valid <= 1;          // ⭐ sticky, never cleared
                            rpt_tog    <= ~rpt_tog;   // new report landed
                        end else if (rx_cnt >= 4'd9) begin
                            bad_crc <= bad_crc + 8'd1;   // rejected, and counted
                        end
                    end else if (script_pc == 5'd9 || script_pc == 5'd12) begin
                        // descriptor data (captured by offset while it streamed in): move on once the bytes
                        // needed are in (device: all 18, for the generic pads; configuration 17) or the packet was short (< 8 data bytes)
                        // (a bad CRC is not ACKed -- the device resends -- and a duplicate is ACKed and dropped)
                        if (!rx_crc_ok) ack_ok = 1'b0;
                        else if (!first_pkt && rxb[0] != exp_pid) begin end
                        else begin
                            exp_pid <= (rxb[0] == PID_DATA1) ? PID_DATA0 : PID_DATA1;
                            cap_off <= cap_off + (rx_n - 7'd3);
                            if (cap_off + (rx_n - 7'd3) >= ((script_pc == 5'd9) ? 7'd18 : 7'd17) || rx_n < 7'd11) begin
                                if (script_pc == 5'd9) begin
                                    if (cap_off + (rx_n - 7'd3) >= 7'd12) begin
                                        dev_vid <= {t_vid_h, t_vid_l}; dev_pid <= {t_pid_h, t_pid_l};
                                    end
                                    if (cap_off + (rx_n - 7'd3) >= 7'd14) dev_ver <= {t_ver_h, t_ver_l};
                                    len_dev <= (cap_off + (rx_n - 7'd3) > 7'd18) ? 8'd18 : {1'b0, cap_off + (rx_n - 7'd3)};
                                    script_pc <= 4'd10;
                                end else begin
                                    is_xin <= (cap_off + (rx_n - 7'd3) >= 7'd17 && t_cls == 8'hFF && t_sub == 8'h5D && t_pro == 8'h01);
                                    dev_kind <= (cap_off + (rx_n - 7'd3) >= 7'd17 && t_cls == 8'h03 && t_sub == 8'h01 && t_pro == 8'h01)
                                                ? 2'd1 : (ls ? 2'd2 : 2'd0);
                                    script_pc <= 4'd13;
                                end
                                cap_kind <= 3'd0;
                            end
                        end
                    end else if (script_pc == 5'd17 || script_pc == 5'd20 || script_pc == 5'd23) begin
                        // generic pads: the full configuration (17) / the HID report descriptor (20), possibly many packets.  Only a CRC-good
                        // packet with the expected toggle is taken; a bad CRC gets no ACK (the device resends; the configuration walker cannot
                        // take bytes back, so that read restarts); a duplicate (PID is the old toggle: our ACK was lost) is ACKed and dropped.
                        if (!rx_crc_ok) begin
                            ack_ok = 1'b0;
                            if (script_pc == 5'd17) begin script_pc <= 5'd16; cap_kind <= 3'd0; end
                        end else if (first_pkt || rxb[0] == exp_pid) begin
                            exp_pid <= (rxb[0] == PID_DATA1) ? PID_DATA0 : PID_DATA1;
                            doff <= doff + {4'd0, rx_n - 7'd3};
                            desc_tries <= 8'd0;
                            if (script_pc == 5'd17) begin
                                if ((doff + {4'd0, rx_n - 7'd3}) >= 11'd255 || (cfg_total != 16'd0 && {5'd0, doff + {4'd0, rx_n - 7'd3}} >= cfg_total)
                                    || {1'b0, rx_n - 7'd3} < {1'b0, mps_eff}) begin
                                    len_cfg <= ((doff + {4'd0, rx_n - 7'd3}) > 11'd256) ? 9'd256 : (doff + {4'd0, rx_n - 7'd3});
                                    if (cfg_total > {5'd0, doff + {4'd0, rx_n - 7'd3}}) desc_trunc[0] <= 1'b1;
                                    script_pc <= 5'd18; cap_kind <= 3'd0;
                                end
                            end else if (script_pc == 5'd23) begin       // a start-up read: done at its length or a short packet
                                if ((doff + {4'd0, rx_n - 7'd3}) >= {3'd0, qlen} || {1'b0, rx_n - 7'd3} < {1'b0, mps_eff}) begin
                                    script_pc <= 5'd24; cap_kind <= 3'd0;
                                end
                            end else begin
                                if ((doff + {4'd0, rx_n - 7'd3}) >= want_rlen || {1'b0, rx_n - 7'd3} < {1'b0, mps_eff}) begin
                                    len_rpt <= ((doff + {4'd0, rx_n - 7'd3}) > 11'd1024) ? 11'd1024 : (doff + {4'd0, rx_n - 7'd3});
                                    script_pc <= 5'd21; cap_kind <= 3'd0;
                                end
                            end
                        end
                    end else begin                                  // a status stage done
                        case (script_pc)
                            5'd1:    begin script_pc <= 5'd8; desc_tries <= 8'd0; end
                            // a boot keyboard: SET_PROTOCOL; everything else, FULL OR LOW SPEED, runs the generic descriptor phase (pc16...).  A low-speed
                            // device that is not a boot keyboard must not go straight to the poll (`ls ? 5'd4`): its report stream would work but no
                            // descriptor would be stored, so a low-speed retro adapter (e.g. a DragonRise 0079:0006) could not be parsed.  The control reads are
                            // the same at 1.5 Mbit/s (bMaxPacketSize0 8 -> 8-byte packets; one transaction per frame, each far shorter than 1 ms).
                            5'd3:    begin script_pc <= (dev_kind == 2'd1) ? 5'd14 : 5'd16; desc_tries <= 8'd0; end
                            5'd15:   script_pc <= 5'd4;
                            default: script_pc <= script_pc + 5'd1;
                        endcase
                    end
                    if (ack_ok) begin
                        pkt[0] <= PID_ACK; pkt_len <= 4'd1; pkt_go <= 1;
                        st <= S_TX_WAIT; st_ret_tx <= S_SOF_IDLE;
                    end else st <= S_SOF_IDLE;
                end else begin
                    // NAK/STALL: nothing to ACK, retry next frame; a NAK on
                    // the scanned endpoint proves it exists -> lock it
                    if (script_pc == 4'd4) ep_locked <= 1;
                    st <= S_SOF_IDLE;
                end
            end
            S_TX_WAIT: begin
                // (ka_go too: a keep-alive not yet taken by the sender would swallow the next pkt_go)
                if (!pkt_sending && !pkt_go && !ka_go && !tx_busy) st <= st_ret_tx;
            end
            S_RX_WAIT: begin
                // hard cap regardless of rx_active: even a stuck receiver
                // must not wedge the whole host (belt to the PHY's
                // idle-abort suspenders)
                rx_hardcap <= rx_hardcap + 17'd1;
                if (rx_done) begin
                    st <= st_ret_rx;
                end else if (rx_hardcap >= 17'd100000) begin  // ~2ms
                    to_count <= to_count + 8'd1;
                    fail_streak <= fail_streak + 11'd1;
                    st <= S_SOF_IDLE;
                end else if (rx_active) begin
                    // response arriving -- a 64-byte report takes ~45us,
                    // longer than the no-answer timeout, so the timeout
                    // must only run while the bus is silent
                    rx_toctr <= 0;
                end else if (rx_toctr == 16'd2000) begin  // ~42us of silence
                    to_count <= to_count + 8'd1;
                    fail_streak <= fail_streak + 11'd1;
                    if (script_pc == 5'd4 && !ep_locked)
                        poll_ep <= (poll_ep == 4'd15) ? 4'd1 : poll_ep + 4'd1;
                    // an endpoint taken from the configuration descriptor that never answers: give the scan back its job
                    if (script_pc == 5'd4 && ep_locked && ep_from_cfg) begin
                        ep_miss <= ep_miss + 6'd1;
                        if (ep_miss == 6'd31) begin ep_locked <= 1'b0; ep_from_cfg <= 1'b0; ep_miss <= 6'd0; end
                    end
                    st <= S_SOF_IDLE;
                end else rx_toctr <= rx_toctr + 16'd1;
            end
            default: st <= S_WAIT_DEV;
            endcase

            // ---- reconnect watchdog: ~1.5s of consecutive unanswered
            // transactions means the device dropped (unplug, power glitch,
            // controller turned itself off). Restart the full sequence so
            // a returning device gets reset + re-enumerated.
            if (fail_streak >= 11'd1500) begin
                st <= S_WAIT_DEV; script_pc <= 0;
                poll_ep <= 4'd1; ep_locked <= 0; ep_from_cfg <= 1'b0;
                desc_valid <= 0; fail_streak <= 0; rpt2_valid <= 1'b0;
            end
        end
    end
endmodule
`default_nettype wire
