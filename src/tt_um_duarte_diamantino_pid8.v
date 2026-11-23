/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * tt_um_duarte_diamantino_pid8 - 8-bit discrete PID controller for one Tiny Tapeout tile.
 *
 * Reduced version of a 10-bit PID (tt_um_digital_pid): 8-bit setpoint, feedback
 * and output (0..255), 12-bit gains (UQ4.8), write-only registers, no PWM.
 * The control output U is always present as a number on uo_out.
 *
 * Pinout
 *   ui_in[7:0]   D[7:0]   write data / address byte
 *   uo_out[7:0]  U[7:0]   control output 0..255 (registered)
 *   uio[0]  in   WR       write strobe (rising edge)
 *   uio[1]  in   AD       1 = address cycle, 0 = data cycle
 *   uio[2]  in   TICK     external sample tick (TICK_SRC = 01)
 *   uio[3]  out  HOLD     integration inhibited by anti-windup in the last sample
 *   uio[4]  out  DRDY     new U since the last write / sample start
 *   uio[5]  out  SAT_HI   output clipped at 255
 *   uio[6]  out  SAT_LO   output clipped at 0
 *   uio[7]  out  BUSY     computation in progress (39 clk per sample)
 *
 * Reset: asynchronous assert, synchronous de-assert (2-FF synchronizer), as in
 * the 10-bit version.
 */
`default_nettype none

module tt_um_duarte_diamantino_pid8 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // reset: asynchronous assert, synchronous de-assert
    reg [1:0] rst_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_sync <= 2'b00;
        else        rst_sync <= {rst_sync[0], 1'b1};
    end
    wire rst_i = rst_sync[1];

    // asynchronous host strobes -> single-cycle events
    wire wr_evt, ext_evt;
    pid_sync u_sync_wr  (.clk(clk), .rst_n(rst_i), .d(uio_in[0]), .rise(wr_evt));
    pid_sync u_sync_tck (.clk(clk), .rst_n(rst_i), .d(uio_in[2]), .rise(ext_evt));

    // registers
    wire        en, aw_en, rev, sw_tick, clr_cmd, fb_evt;
    wire [1:0]  tick_src;
    wire [7:0]  sp, fb;
    wire [11:0] kp, ki, kd, ts;

    pid8_regs u_regs (
        .clk(clk), .rst_n(rst_i), .wr_evt(wr_evt), .ad(uio_in[1]), .din(ui_in),
        .en(en), .aw_en(aw_en), .rev(rev), .tick_src(tick_src),
        .sp(sp), .fb(fb), .kp(kp), .ki(ki), .kd(kd), .ts(ts),
        .sw_tick(sw_tick), .clr_cmd(clr_cmd), .fb_evt(fb_evt)
    );

    // sample tick
    wire tick;
    pid8_tick u_tick (
        .clk(clk), .rst_n(rst_i), .en(en), .src(tick_src), .ts(ts),
        .ext_evt(ext_evt), .fb_evt(fb_evt), .sw_evt(sw_tick), .tick(tick)
    );

    // PID core
    wire       busy, done, sat_hi, sat_lo, i_hold;
    wire [7:0] u;
    pid8_core u_core (
        .clk(clk), .rst_n(rst_i), .en(en), .clr(clr_cmd), .start(tick),
        .aw_en(aw_en), .rev(rev), .sp(sp), .fb(fb), .kp(kp), .ki(ki), .kd(kd),
        .busy(busy), .done(done), .u(u), .sat_hi(sat_hi), .sat_lo(sat_lo),
        .i_hold(i_hold)
    );

    // DRDY: set when a sample completes, cleared by a bus write or a new sample
    reg drdy;
    always @(posedge clk or negedge rst_i) begin
        if (!rst_i)                 drdy <= 1'b0;
        else if (done)              drdy <= 1'b1;
        else if (wr_evt || tick)    drdy <= 1'b0;
    end

    // pins
    assign uo_out  = u;
    assign uio_out = {busy, sat_lo, sat_hi, drdy, i_hold, 3'b000};
    assign uio_oe  = 8'b1111_1000;

    wire _unused = &{ena, uio_in[7:3], 1'b0};

endmodule

`default_nettype wire
