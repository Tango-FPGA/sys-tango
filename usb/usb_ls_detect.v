// SPDX-License-Identifier: MIT
// Copyright (c) 2026 the Tango authors
// usb_ls_detect.v -- USB device presence + speed detection (clean-room).
// Per USB 2.0 spec 7.1.5: a full-speed device pulls D+ high via its 1.5K
// resistor; a low-speed device pulls D- high instead. Host provides
// pulldowns (FPGA pin PULL_MODE=DOWN). No device: both low.
module usb_ls_detect #(
    parameter integer DEBOUNCE_CYCLES = 24'd1_000_000
)(
    input  wire clk, rst_n,
    input  wire dp, dn,
    output reg [1:0] status      // 0=none 1=low-speed 2=full-speed 3=invalid
);
    reg [1:0] raw, raw_q;
    reg [23:0] stable_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            raw_q <= 2'b00; stable_cnt <= 0; status <= 2'd0;
        end else begin
            raw <= {dp, dn};
            raw_q <= raw;
            if (raw != raw_q) begin
                stable_cnt <= 0;
            end else if (stable_cnt != DEBOUNCE_CYCLES) begin
                stable_cnt <= stable_cnt + 24'd1;
            end else begin
                case (raw_q)
                    2'b00: status <= 2'd0;
                    2'b01: status <= 2'd1;
                    2'b10: status <= 2'd2;
                    2'b11: status <= 2'd3;
                endcase
            end
        end
    end
endmodule
