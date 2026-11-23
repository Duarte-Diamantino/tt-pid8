// SPDX-License-Identifier: Apache-2.0
//
// tb_pid8.v - chip-level self-checking testbench for tt_um_duarte_diamantino_pid8 (8-bit PID).
// Drives only the Tiny Tapeout pins, like the host would. The same testbench
// runs on the RTL and on the gate-level netlist generated for Wokwi
// (wokwi/pid8_gates.v, rebuilt from diagram.json).
//
//   T1  reset state of the pins
//   T2  worked example: Kp = 1, SP 100, FB 40 -> U = 60
//   T3  random samples vs bit-exact model (gains incl. 0 / max, AW, REV, CLR,
//       FB-write, software and external ticks)
//   T4  timer tick period = TS + 1 clk
//   T5  latency: BUSY high for 38 clk (U ready 39 clk after the tick);
//       DRDY cleared by a write
//   T6  EN = 0: U = 0 and no samples
//   T7  asynchronous reset with the clock stopped: U = 0 at once
//   T8  closed loop with a first-order plant (step + load disturbance)
//
// Plusargs: +N=<random samples> (default 3000)
`timescale 1ns/1ps
`default_nettype none

module tb_pid8;
    reg        clk = 0;
    reg        rst_n = 0;
    reg  [7:0] ui_in = 0;
    reg  [7:0] uio_in = 0;
    wire [7:0] uo_out, uio_out, uio_oe;
    reg        clk_en = 1;

    tt_um_duarte_diamantino_pid8 dut (
        .ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in), .uio_out(uio_out),
        .uio_oe(uio_oe), .ena(1'b1), .clk(clk), .rst_n(rst_n)
    );

    always #50 if (clk_en) clk = ~clk;          // 10 MHz

    integer errors = 0;

    // ------------------------------------------------------------ pins / regs
    localparam P_WR = 0, P_AD = 1, P_TICK = 2;
    localparam O_HOLD = 3, O_DRDY = 4, O_SATHI = 5, O_SATLO = 6, O_BUSY = 7;
    localparam [3:0] R_CTRL = 4'h0, R_CMD = 4'h1, R_SP = 4'h2, R_FB = 4'h3,
                     R_KP = 4'h4, R_KI = 4'h6, R_KD = 4'h8, R_TS = 4'hA;
    localparam [7:0] C_EN = 8'h01, C_AW = 8'h02, C_REV = 8'h04,
                     C_TIMER = 8'h00, C_EXT = 8'h08, C_FB = 8'h10, C_SW = 8'h18;

    // ------------------------------------------------------------ host bus
    task bus_cycle(input ad_i, input [7:0] d);
        begin
            @(negedge clk);
            ui_in = d; uio_in[P_AD] = ad_i;
            @(negedge clk);
            uio_in[P_WR] = 1'b1;
            repeat (4) @(negedge clk);
            uio_in[P_WR] = 1'b0;
            repeat (4) @(negedge clk);
        end
    endtask
    task wr8(input [3:0] a, input [7:0] d);   begin bus_cycle(1, {4'h0, a}); bus_cycle(0, d); end endtask
    task wr12(input [3:0] a, input [11:0] v);
        begin bus_cycle(1, {4'h0, a}); bus_cycle(0, v[7:0]); bus_cycle(0, {4'h0, v[11:8]}); end
    endtask
    task wait_drdy(input integer timeout);
        integer t;
        begin
            t = 0;
            while (uio_out[O_DRDY] !== 1'b1 && t < timeout) begin @(negedge clk); t = t + 1; end
            if (t >= timeout) begin
                $display("ERROR @%0t: timeout waiting for DRDY", $time); errors = errors + 1;
            end
        end
    endtask
    task chk(input [255:0] what, input integer got, input integer exp);
        if (got !== exp) begin
            $display("ERROR %0s: got %0d expected %0d", what, got, exp); errors = errors + 1;
        end
    endtask

    // ------------------------------------------------------------ bit-exact model
    reg signed [63:0] m_I, m_eprev, m_e;
    reg               m_first, m_sat_hi, m_sat_lo, m_hold;
    reg        [7:0]  m_u;
    integer n_hold, n_sathi, n_satlo, n_clamp_hi, n_clamp_lo;

    task model_reset;
        begin m_I = 0; m_eprev = 0; m_e = 0; m_first = 1; m_sat_hi = 0; m_sat_lo = 0;
              m_hold = 0; m_u = 0; end
    endtask
    task model_step(input integer sp, fb, kp, ki, kd, input aw, rev);
        reg signed [63:0] de, s, v;
        begin
            m_e    = rev ? (fb - sp) : (sp - fb);
            de     = m_first ? 0 : (m_e - m_eprev);
            m_hold = aw && ((m_sat_hi && m_e > 0) || (m_sat_lo && m_e < 0));
            if (!m_hold) m_I = m_I + ki * m_e;
            if (m_I < 0)          begin m_I = 0;     n_clamp_lo = n_clamp_lo + 1; end
            else if (m_I > 65280) begin m_I = 65280; n_clamp_hi = n_clamp_hi + 1; end
            s = m_I + 128 + kp * m_e + kd * de;
            v = s >>> 8;
            m_sat_hi = (v > 255);
            m_sat_lo = (v < 0);
            m_u      = (v < 0) ? 8'd0 : ((v > 255) ? 8'd255 : v[7:0]);
            m_eprev  = m_e;
            m_first  = 0;
            if (m_hold)   n_hold  = n_hold + 1;
            if (m_sat_hi) n_sathi = n_sathi + 1;
            if (m_sat_lo) n_satlo = n_satlo + 1;
        end
    endtask
    task check_outputs(input [255:0] what);
        begin
            chk({what, " U"},      uo_out,           m_u);
            chk({what, " SAT_HI"}, uio_out[O_SATHI], m_sat_hi);
            chk({what, " SAT_LO"}, uio_out[O_SATLO], m_sat_lo);
            chk({what, " HOLD"},   uio_out[O_HOLD],  m_hold);
        end
    endtask

    // ------------------------------------------------------------ helpers
    function [11:0] rnd_gain(input integer r);
        case (r % 8)
            0: rnd_gain = 12'd0;
            1: rnd_gain = 12'hFFF;
            2: rnd_gain = $urandom % 16;           // tiny
            3: rnd_gain = 12'd256;                 // 1.0
            default: rnd_gain = $urandom % 4096;
        endcase
    endfunction

    integer i, k, t0, t1, nbusy, N, seed;
    integer sp, fb, kp, ki, kd;
    reg     aw, rev;
    reg [7:0] ctrl;

    // T8 plant
    real y, load, ymax;
    integer cl_err;

    initial begin
        N = 3000;
        if ($value$plusargs("N=%d", N)) ;
        n_hold = 0; n_sathi = 0; n_satlo = 0; n_clamp_hi = 0; n_clamp_lo = 0;
        model_reset;

        // ---------------------------------------------------------------- T1
        repeat (3) @(negedge clk);
        chk("T1 uo_out in reset", uo_out, 0);
        chk("T1 uio_oe", uio_oe, 8'hF8);
        chk("T1 uio_out in reset", uio_out & 8'hF8, 0);
        rst_n = 1;
        repeat (5) @(negedge clk);
        chk("T1 uo_out after reset", uo_out, 0);
        chk("T1 BUSY/DRDY after reset", uio_out & 8'hF8, 0);
        $display("T1 reset state: done");

        // ---------------------------------------------------------------- T2
        wr12(R_KP, 12'd256);                       // Kp = 1.0
        wr12(R_KI, 12'd0);
        wr12(R_KD, 12'd0);
        wr8(R_CTRL, C_EN | C_AW | C_FB);
        wr8(R_SP, 8'd100);
        wr8(R_FB, 8'd40);                          // starts the sample
        wait_drdy(200);
        chk("T2 U (Kp=1, SP=100, FB=40)", uo_out, 60);
        $display("T2 worked example: U = %0d (expected 60)", uo_out);

        // ---------------------------------------------------------------- T3
        wr8(R_CMD, 8'h02);                         // CLR
        model_reset;
        kp = 256; ki = 0; kd = 0; aw = 1; rev = 0; sp = 100;
        for (i = 0; i < N; i = i + 1) begin
            // occasionally change the configuration
            if (i % 50 == 0) begin
                kp = rnd_gain($urandom); ki = rnd_gain($urandom); kd = rnd_gain($urandom);
                if (i % 200 == 0) begin kp = 12'hFFF; ki = 12'hFFF; kd = 12'hFFF; end
                aw = $urandom % 4 != 0; rev = $urandom % 4 == 0;
                wr12(R_KP, kp); wr12(R_KI, ki); wr12(R_KD, kd);
                wr8(R_CTRL, C_EN | (aw ? C_AW : 0) | (rev ? C_REV : 0) | C_FB);
            end
            if ($urandom % 97 == 0) begin wr8(R_CMD, 8'h02); model_reset; end
            // setpoint: mostly held, sometimes stepped, extremes included
            if ($urandom % 8 == 0) begin
                k = $urandom % 6;
                sp = (k == 0) ? 0 : (k == 1) ? 255 : $urandom % 256;
                wr8(R_SP, sp);
            end
            k = $urandom % 6;
            fb = (k == 0) ? 0 : (k == 1) ? 255 : $urandom % 256;
            model_step(sp, fb, kp, ki, kd, aw, rev);
            case ($urandom % 10)
                0: begin                            // software tick
                    wr8(R_CTRL, C_EN | (aw ? C_AW : 0) | (rev ? C_REV : 0) | C_SW);
                    wr8(R_FB, fb);
                    wr8(R_CMD, 8'h01);
                    wait_drdy(200);
                    wr8(R_CTRL, C_EN | (aw ? C_AW : 0) | (rev ? C_REV : 0) | C_FB);
                end
                1: begin                            // external tick pin
                    wr8(R_CTRL, C_EN | (aw ? C_AW : 0) | (rev ? C_REV : 0) | C_EXT);
                    wr8(R_FB, fb);
                    @(negedge clk); uio_in[P_TICK] = 1;
                    repeat (3) @(negedge clk); uio_in[P_TICK] = 0;
                    wait_drdy(200);
                    wr8(R_CTRL, C_EN | (aw ? C_AW : 0) | (rev ? C_REV : 0) | C_FB);
                end
                default: begin                      // FB write starts the sample
                    wr8(R_FB, fb);
                    wait_drdy(200);
                end
            endcase
            check_outputs("T3");
            if (errors > 20) begin $display("too many errors"); $finish; end
        end
        $display("T3 random: %0d samples, holds %0d, SAT_HI %0d, SAT_LO %0d, I clamp hi %0d lo %0d",
                 N, n_hold, n_sathi, n_satlo, n_clamp_hi, n_clamp_lo);
        if (n_hold == 0 || n_sathi == 0 || n_satlo == 0 || n_clamp_hi == 0 || n_clamp_lo == 0) begin
            $display("ERROR T3: a corner case was never reached"); errors = errors + 1;
        end

        // ---------------------------------------------------------------- T4
        wr12(R_TS, 12'd99);
        wr8(R_CTRL, C_EN | C_AW | C_TIMER);
        @(posedge uio_out[O_BUSY]); t0 = $time;
        @(posedge uio_out[O_BUSY]); t1 = $time;
        chk("T4 timer period (clk)", (t1 - t0) / 100, 100);
        $display("T4 timer: period %0d clk (TS = 99)", (t1 - t0) / 100);

        // ---------------------------------------------------------------- T5
        // tick seen in IDLE (1 clk) + BUSY 38 clk = U updated 39 clk after the tick
        @(posedge uio_out[O_BUSY]); t0 = $time;
        @(negedge uio_out[O_BUSY]); t1 = $time;
        chk("T5 BUSY width (clk)", (t1 - t0) / 100, 38);
        repeat (2) @(negedge clk);
        chk("T5 DRDY after sample", uio_out[O_DRDY], 1);
        wr8(R_CTRL, C_EN | C_AW | C_SW);           // a write clears DRDY
        chk("T5 DRDY cleared by write", uio_out[O_DRDY], 0);
        $display("T5 latency: BUSY %0d clk", (t1 - t0) / 100);

        // ---------------------------------------------------------------- T6
        wr12(R_KP, 12'd512); wr12(R_KI, 12'd0); wr12(R_KD, 12'd0);
        wr8(R_CMD, 8'h02);
        wr8(R_SP, 200); wr8(R_FB, 0);
        wr8(R_CMD, 8'h01); wait_drdy(200);
        if (uo_out == 0) begin $display("ERROR T6: U should be > 0 before EN = 0"); errors = errors + 1; end
        wr8(R_CTRL, C_AW | C_TIMER);               // EN = 0
        repeat (3) @(negedge clk);
        chk("T6 U with EN = 0", uo_out, 0);
        nbusy = 0;
        for (k = 0; k < 500; k = k + 1) begin @(negedge clk); if (uio_out[O_BUSY]) nbusy = nbusy + 1; end
        chk("T6 no samples with EN = 0", nbusy, 0);
        $display("T6 EN = 0: U = 0, no samples");

        // ---------------------------------------------------------------- T7
        wr8(R_CTRL, C_EN | C_AW | C_SW);
        wr8(R_CMD, 8'h01); wait_drdy(200);
        if (uo_out == 0) begin $display("ERROR T7: U should be > 0 before reset"); errors = errors + 1; end
        @(negedge clk); clk_en = 0;                // stop the clock
        #300 rst_n = 0;
        #10  chk("T7 U after reset, clock stopped", uo_out, 0);
        chk("T7 outputs after reset, clock stopped", uio_out & 8'hF8, 0);
        #300 clk_en = 1; rst_n = 1;
        repeat (5) @(negedge clk);
        $display("T7 asynchronous reset without clock: U = %0d", uo_out);

        // ---------------------------------------------------------------- T8
        // plant: tau*dy/dt = u - y - load, tau = 20 samples, Ts = 400 clk
        // Kp = 2.0, Ki = 0.25 (Ti = 8 Ts), SP 128, load 60 half-way
        wr12(R_KP, 12'd512); wr12(R_KI, 12'd64); wr12(R_KD, 12'd0);
        wr8(R_CTRL, C_EN | C_AW | C_FB);
        wr8(R_CMD, 8'h02);
        wr8(R_SP, 8'd128);
        y = 0; load = 0; ymax = 0; cl_err = 0;
        for (i = 0; i < 300; i = i + 1) begin
            if (i == 150) load = 60;
            fb = (y < 0) ? 0 : (y > 255) ? 255 : $rtoi(y + 0.5);
            wr8(R_FB, fb);
            wait_drdy(200);
            y = y + (uo_out - y - load) / 20.0;    // one sample of the plant
            if (i < 150 && y > ymax) ymax = y;
            if (i == 149 || i == 299) cl_err = fb - 128;
            if (i == 149 && (cl_err > 1 || cl_err < -1)) begin
                $display("ERROR T8: error before load %0d", cl_err); errors = errors + 1;
            end
        end
        if (cl_err > 1 || cl_err < -1) begin $display("ERROR T8: final error %0d", cl_err); errors = errors + 1; end
        if (ymax > 128 * 1.15) begin $display("ERROR T8: overshoot %0.1f %%", (ymax - 128) / 1.28); errors = errors + 1; end
        $display("T8 closed loop: overshoot %0.1f %%, final error %0d (with load 60)",
                 (ymax - 128) / 1.28, cl_err);

        if (errors == 0) $display("tb_pid8: TEST PASSED");
        else             $display("tb_pid8: TEST FAILED (%0d errors)", errors);
        $finish;
    end
endmodule

`default_nettype wire
