/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * pid8_tick - sample tick generator (12-bit timer).
 *
 *   TICK_SRC = 00 : internal timer, period = (TS + 1) clk cycles
 *   TICK_SRC = 01 : external pin TICK (synchronized rising edge)
 *   TICK_SRC = 10 : feedback write (host writes FB -> new sample)
 *   TICK_SRC = 11 : software only (CMD.SW_TICK)
 *
 * CMD.SW_TICK generates a tick in every mode. No tick while EN = 0.
 */
`default_nettype none

module pid8_tick (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,
    input  wire [1:0]  src,
    input  wire [11:0] ts,
    input  wire        ext_evt,
    input  wire        fb_evt,
    input  wire        sw_evt,
    output wire        tick
);
    reg  [11:0] cnt;
    wire        timer_mode = (src == 2'b00);
    wire        timer_hit  = (cnt == 12'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                      cnt <= 12'd0;
        else if (!en || !timer_mode)     cnt <= ts;
        else if (timer_hit)              cnt <= ts;
        else                             cnt <= cnt - 12'd1;
    end

    assign tick = en & ( (timer_mode       & timer_hit)
                       | ((src == 2'b01)   & ext_evt)
                       | ((src == 2'b10)   & fb_evt)
                       | sw_evt );

endmodule

`default_nettype wire
