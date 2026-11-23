/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * pid8_regs - write-only byte register interface of the 8-bit PID.
 *
 * Bus cycles (one byte each, on the synchronized WR rising edge):
 *   AD = 1 : address cycle -> ADDR <= D[3:0]
 *   AD = 0 : data cycle    -> REG[ADDR] <= D ; ADDR <= ADDR + 1
 *
 * There is no readback: uo_out always shows the control output U.
 * 12-bit registers: write the LOW byte first (held in a staging register),
 * then the HIGH byte (bits [3:0]); the 12-bit value is committed atomically.
 *
 * Address map:
 *   0x0 CTRL  [0]EN [1]AW_EN [2]REV [4:3]TICK_SRC          (reset 0x02)
 *   0x1 CMD   [0]SW_TICK [1]CLR (pulses, nothing stored)
 *   0x2 SP    setpoint 0..255
 *   0x3 FB    feedback 0..255 (starts a sample if TICK_SRC = 10)
 *   0x4 KP_L  0x5 KP_H (commit)   UQ4.8
 *   0x6 KI_L  0x7 KI_H (commit)   UQ4.8
 *   0x8 KD_L  0x9 KD_H (commit)   UQ4.8
 *   0xA TS_L  0xB TS_H (commit)   timer period = TS+1 clk (reset 999)
 */
`default_nettype none

module pid8_regs (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        wr_evt,      // synchronized WR rising edge
    input  wire        ad,
    input  wire [7:0]  din,
    output wire        en,
    output wire        aw_en,
    output wire        rev,
    output wire [1:0]  tick_src,
    output reg  [7:0]  sp,
    output reg  [7:0]  fb,
    output reg  [11:0] kp,
    output reg  [11:0] ki,
    output reg  [11:0] kd,
    output reg  [11:0] ts,
    output wire        sw_tick,
    output wire        clr_cmd,
    output reg         fb_evt       // one cycle AFTER the FB register update
);
    localparam [3:0] A_CTRL = 4'h0, A_CMD  = 4'h1, A_SP   = 4'h2, A_FB   = 4'h3,
                     A_KP_L = 4'h4, A_KP_H = 4'h5, A_KI_L = 4'h6, A_KI_H = 4'h7,
                     A_KD_L = 4'h8, A_KD_H = 4'h9, A_TS_L = 4'hA, A_TS_H = 4'hB;

    localparam [11:0] TS_RESET = 12'd999;

    reg  [3:0] addr;
    reg  [7:0] stage;
    reg  [4:0] ctrl;

    wire       wr_addr = wr_evt &  ad;
    wire       wr_data = wr_evt & ~ad;
    wire [11:0] v12    = {din[3:0], stage};

    assign en       = ctrl[0];
    assign aw_en    = ctrl[1];
    assign rev      = ctrl[2];
    assign tick_src = ctrl[4:3];

    assign sw_tick  = wr_data & (addr == A_CMD) & din[0];
    assign clr_cmd  = wr_data & (addr == A_CMD) & din[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr   <= 4'd0;
            stage  <= 8'd0;
            ctrl   <= 5'b0_0010;           // AW_EN = 1, everything else 0
            sp     <= 8'd0;
            fb     <= 8'd0;
            kp     <= 12'd0;
            ki     <= 12'd0;
            kd     <= 12'd0;
            ts     <= TS_RESET;
            fb_evt <= 1'b0;
        end else begin
            fb_evt <= 1'b0;
            if (wr_addr) begin
                addr <= din[3:0];
            end else if (wr_data) begin
                addr <= addr + 4'd1;
                case (addr)
                    A_CTRL: ctrl <= din[4:0];
                    A_SP:   sp   <= din;
                    A_FB:   begin fb <= din; fb_evt <= 1'b1; end
                    A_KP_L, A_KI_L, A_KD_L, A_TS_L: stage <= din;
                    A_KP_H: kp <= v12;
                    A_KI_H: ki <= v12;
                    A_KD_H: kd <= v12;
                    A_TS_H: ts <= v12;
                    default: ;
                endcase
            end
        end
    end

endmodule

`default_nettype wire
