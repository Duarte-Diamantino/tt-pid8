/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * pid8_core - 8-bit sequential discrete PID with one serial shift-add MAC.
 * Same equations as src/pid_core_seq.v (ARCH 2), with narrower formats:
 *
 *   e     = SP - FB                      (REV=1: FB - SP)          S9.0
 *   de    = e - e_prev   (0 on the first sample after clear)       S10.0
 *   hold  = AW_EN & ((SAT_HI & e>0) | (SAT_LO & e<0))  (conditional integration)
 *   I     = clamp(I + (hold ? 0 : Ki*e), 0, 255<<8)                UQ8.8
 *   S     = I + 2^7 + Kp*e + Kd*de                                 S14.8 (23 b)
 *   u     = clamp(S >>> 8, 0, 255)   (round half up)               UQ8.0
 *   SAT_HI = (S>>>8) > 255 ; SAT_LO = (S>>>8) < 0
 *
 * Gains are UQ4.8 (12-bit unsigned, LSB = 1/256, max 15.996).
 * One gain bit per clock: 12 clk per product, 39 clk per sample.
 *
 * Accumulator width: |Kp*e|, |Ki*e| <= 4095*255 < 2^20, |Kd*de| <= 4095*510 < 2^21,
 * I + 2^7 < 2^16  =>  |S| < 2^22  -> 23-bit two's complement, never overflows.
 */
`default_nettype none

module pid8_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,       // 0: controller held in the cleared state, u = 0
    input  wire        clr,      // clear state (pulse)
    input  wire        start,    // sample tick (ignored while busy)
    input  wire        aw_en,
    input  wire        rev,
    input  wire [7:0]  sp,
    input  wire [7:0]  fb,
    input  wire [11:0] kp,
    input  wire [11:0] ki,
    input  wire [11:0] kd,
    output wire        busy,
    output reg         done,     // one-cycle pulse, u updated
    output reg  [7:0]  u,
    output reg         sat_hi,
    output reg         sat_lo,
    output reg         i_hold    // integration inhibited in the last sample
);
    localparam ACC_W = 23;
    localparam [ACC_W-1:0] I_MAX = 23'd65280;     // 255 << 8
    localparam [ACC_W-1:0] RND   = 23'd128;       // 0.5 LSB of the output

    localparam [2:0] S_IDLE = 3'd0, S_MI = 3'd1, S_CLI = 3'd2,
                     S_MP   = 3'd3, S_MD = 3'd4, S_SAT = 3'd5;

    reg        [2:0]        state;
    reg  signed [8:0]       e_q;      // error of the current sample
    reg  signed [8:0]       e_prev;
    reg                     first;
    reg        [15:0]       integ;    // integrator UQ8.8
    reg  signed [ACC_W-1:0] acc;

    // serial MAC: acc += gsh[0] ? xsh : 0, then xsh <<= 1, gsh >>= 1
    reg  signed [20:0]      xsh;      // |x| <= 510, shifted up to 11 -> < 2^20
    reg        [11:0]       gsh;
    reg        [3:0]        cnt;

    assign busy = (state != S_IDLE);

    // ---------------- error, derivative, anti-windup decision -------------
    wire        [7:0] op_a = rev ? fb : sp;
    wire        [7:0] op_b = rev ? sp : fb;
    wire signed [8:0] e_in = $signed({1'b0, op_a}) - $signed({1'b0, op_b});

    wire        e_pos   = ~e_in[8] & (|e_in);
    wire        e_neg   =  e_in[8];
    wire        hold_in = aw_en & ((sat_hi & e_pos) | (sat_lo & e_neg));

    wire signed [9:0] de = first ? 10'sd0
                                 : ($signed({e_q[8], e_q}) - $signed({e_prev[8], e_prev}));

    // ---------------- integrator clamp (acc = I_old + Ki*e) ----------------
    wire        acc_neg     = acc[ACC_W-1];
    wire        acc_gt_imax = ~acc_neg & (acc[ACC_W-2:0] > I_MAX[ACC_W-2:0]);
    wire [15:0] i_clamped   = acc_neg     ? 16'd0 :
                              acc_gt_imax ? I_MAX[15:0] : acc[15:0];

    // ---------------- output saturation (acc = S) --------------------------
    wire signed [14:0] v      = acc[ACC_W-1:8];      // floor(S / 256)
    wire               v_neg  = v[14];
    wire               v_hi   = ~v_neg & (v[13:0] > 14'd255);
    wire        [7:0]  u_next = v_neg ? 8'd0 : (v_hi ? 8'd255 : v[7:0]);

    // ---------------- serial MAC -------------------------------------------
    wire                    mac_last = (cnt == 4'd11);
    wire signed [ACC_W-1:0] mac_sum  = acc + (gsh[0] ? {{(ACC_W-21){xsh[20]}}, xsh}
                                                     : {ACC_W{1'b0}});

    // datapath registers: no reset needed (always loaded before use)
    always @(posedge clk) begin
        case (state)
            S_IDLE: begin
                xsh <= {{12{e_in[8]}}, e_in};
                gsh <= hold_in ? 12'd0 : ki;      // hold: the I product is 0
                cnt <= 4'd0;
                acc <= {7'd0, integ};             // I_old
            end
            S_CLI: begin
                xsh <= {{12{e_q[8]}}, e_q};
                gsh <= kp;
                cnt <= 4'd0;
                acc <= {7'd0, i_clamped} + RND;   // I_new + 0.5
            end
            S_MI, S_MP, S_MD: begin
                cnt <= mac_last ? 4'd0 : cnt + 4'd1;   // 12 steps (no 4-bit wrap)
                acc <= mac_sum;
                if (mac_last && state == S_MP) begin
                    xsh <= {{11{de[9]}}, de};     // next product: Kd*de
                    gsh <= kd;
                end else begin
                    xsh <= xsh <<< 1;
                    gsh <= gsh >> 1;
                end
            end
            default: ;
        endcase
    end

    // ---------------- control FSM + architectural state --------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= S_IDLE;
            integ  <= 16'd0;
            e_q    <= 9'sd0;
            e_prev <= 9'sd0;
            first  <= 1'b1;
            u      <= 8'd0;
            sat_hi <= 1'b0;
            sat_lo <= 1'b0;
            i_hold <= 1'b0;
            done   <= 1'b0;
        end else begin
            done <= 1'b0;
            if (!en || clr) begin
                state  <= S_IDLE;
                integ  <= 16'd0;
                e_prev <= 9'sd0;
                first  <= 1'b1;
                u      <= 8'd0;
                sat_hi <= 1'b0;
                sat_lo <= 1'b0;
                i_hold <= 1'b0;
            end else begin
                case (state)
                    S_IDLE: if (start) begin
                        e_q    <= e_in;
                        i_hold <= hold_in;
                        state  <= S_MI;
                    end
                    S_MI:  if (mac_last) state <= S_CLI;
                    S_CLI: begin
                        integ <= i_clamped;
                        state <= S_MP;
                    end
                    S_MP:  if (mac_last) state <= S_MD;
                    S_MD:  if (mac_last) state <= S_SAT;
                    S_SAT: begin
                        u      <= u_next;
                        sat_hi <= v_hi;
                        sat_lo <= v_neg;
                        e_prev <= e_q;
                        first  <= 1'b0;
                        done   <= 1'b1;
                        state  <= S_IDLE;
                    end
                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule

`default_nettype wire
