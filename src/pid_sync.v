/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * pid_sync - 2-FF synchronizer + rising-edge detector for asynchronous
 * host strobes (WR, RD, TICK_EXT).
 *
 * The chain resets to all-ones so that a pin that is already HIGH when reset
 * is released does NOT produce a spurious edge; only a real 0->1 transition
 * after reset generates `rise`.
 *
 * Latency: the pin edge is reported 2..3 clk cycles after it happens.
 */
`default_nettype none

module pid_sync (
    input  wire clk,
    input  wire rst_n,
    input  wire d,      // asynchronous input pin
    output wire rise    // one-cycle pulse on synchronized rising edge
);
    reg [2:0] s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) s <= 3'b111;
        else        s <= {s[1:0], d};
    end

    assign rise = s[1] & ~s[2];

endmodule

`default_nettype wire
