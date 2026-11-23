# SPDX-License-Identifier: Apache-2.0
#
# cocotb tests for tt_um_duarte_diamantino_pid8 (8-bit PID), run by the Tiny
# Tapeout CI on the RTL and on the gate-level netlist (make GATES=yes).
# Only the top-level pins are used, as a host MCU would.

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, Timer

# ---------------------------------------------------------------- registers
R_CTRL, R_CMD, R_SP, R_FB, R_KP, R_KI, R_KD, R_TS = 0x0, 0x1, 0x2, 0x3, 0x4, 0x6, 0x8, 0xA
C_EN, C_AW, C_REV = 0x01, 0x02, 0x04
C_TIMER, C_EXT, C_FB, C_SW = 0x00, 0x08, 0x10, 0x18
CMD_TICK, CMD_CLR = 0x01, 0x02

# uio pins
P_WR, P_AD, P_TICK = 0, 1, 2
O_HOLD, O_DRDY, O_SATHI, O_SATLO, O_BUSY = 3, 4, 5, 6, 7


def val(sig):
    v = sig.value
    return v.to_unsigned() if hasattr(v, "to_unsigned") else int(v)


class Host:
    """Bus functional model of the host MCU (address byte, then data bytes)."""

    def __init__(self, dut):
        self.dut = dut
        self.uio = 0
        dut.uio_in.value = 0
        dut.ui_in.value = 0

    def _set(self, bit, level):
        self.uio = (self.uio | (1 << bit)) if level else (self.uio & ~(1 << bit))
        self.dut.uio_in.value = self.uio

    async def cycle(self, ad, data):
        await FallingEdge(self.dut.clk)
        self.dut.ui_in.value = data & 0xFF
        self._set(P_AD, ad)
        await FallingEdge(self.dut.clk)
        self._set(P_WR, 1)
        await ClockCycles(self.dut.clk, 4, rising=False)
        self._set(P_WR, 0)
        await ClockCycles(self.dut.clk, 4, rising=False)

    async def wr8(self, a, d):
        await self.cycle(1, a)
        await self.cycle(0, d)

    async def wr12(self, a, v):
        await self.cycle(1, a)
        await self.cycle(0, v & 0xFF)
        await self.cycle(0, (v >> 8) & 0x0F)

    async def tick_pin(self):
        await FallingEdge(self.dut.clk)
        self._set(P_TICK, 1)
        await ClockCycles(self.dut.clk, 3, rising=False)
        self._set(P_TICK, 0)

    def pin(self, bit):
        return (val(self.dut.uio_out) >> bit) & 1

    @property
    def u(self):
        return val(self.dut.uo_out)

    async def wait_drdy(self, timeout=200):
        for _ in range(timeout):
            if self.pin(O_DRDY):
                return
            await FallingEdge(self.dut.clk)
        assert False, "timeout waiting for DRDY"


class PidModel:
    """Bit-exact reference model (same equations as sim/tb_pid8.v)."""

    I_MAX = 255 * 256

    def __init__(self):
        self.reset()

    def reset(self):
        self.I, self.e_prev, self.first = 0, 0, True
        self.sat_hi = self.sat_lo = self.hold = False
        self.u = 0

    def step(self, sp, fb, kp, ki, kd, aw=True, rev=False):
        e = (fb - sp) if rev else (sp - fb)
        de = 0 if self.first else e - self.e_prev
        self.hold = aw and ((self.sat_hi and e > 0) or (self.sat_lo and e < 0))
        if not self.hold:
            self.I += ki * e
        self.I = min(max(self.I, 0), self.I_MAX)
        v = (self.I + 128 + kp * e + kd * de) >> 8      # Python >> is arithmetic
        self.sat_hi, self.sat_lo = v > 255, v < 0
        self.u = min(max(v, 0), 255)
        self.e_prev, self.first = e, False
        return self.u

    def check(self, host, what):
        got = (host.u, host.pin(O_SATHI), host.pin(O_SATLO), host.pin(O_HOLD))
        exp = (self.u, int(self.sat_hi), int(self.sat_lo), int(self.hold))
        assert got == exp, f"{what}: (U, SAT_HI, SAT_LO, HOLD) = {got}, expected {exp}"


async def start(dut):
    clock = Clock(dut.clk, 100, unit="ns")    # 10 MHz
    clock.start()
    dut.ena.value = 1
    host = Host(dut)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    return host, clock


@cocotb.test()
async def test_reset(dut):
    host, _ = await start(dut)
    assert val(dut.uio_oe) == 0xF8, f"uio_oe = {val(dut.uio_oe):#04x}"
    assert host.u == 0
    assert val(dut.uio_out) & 0xF8 == 0, "HOLD/DRDY/SAT/BUSY low after reset"


@cocotb.test()
async def test_worked_example(dut):
    """Kp = 1.0, SP = 100, FB = 40 -> U = 60 (the 'How to test' steps of docs/info.md)."""
    host, _ = await start(dut)
    await host.cycle(1, R_KP)                 # one burst: KP = 256, KI = 0, KD = 0
    for b in (0x00, 0x01, 0x00, 0x00, 0x00, 0x00):
        await host.cycle(0, b)
    await host.wr8(R_CTRL, C_EN | C_AW | C_FB)
    await host.wr8(R_SP, 100)
    await host.wr8(R_FB, 40)                  # TICK_SRC = FB write: starts the sample
    await host.wait_drdy()
    assert host.u == 60, f"U = {host.u}"
    dut._log.info(f"U = {host.u} (expected 60)")


@cocotb.test()
async def test_random_against_model(dut):
    """Random gains/setpoints/feedback, AW, REV, CLR and all tick sources."""
    host, _ = await start(dut)
    rng = random.Random(2026)
    model = PidModel()
    n_hold = n_hi = n_lo = 0

    def gain():
        r = rng.randrange(8)
        return [0, 0xFFF, rng.randrange(16), 256][r] if r < 4 else rng.randrange(4096)

    kp, ki, kd, aw, rev, sp = 256, 0, 0, True, False, 100
    await host.wr8(R_CMD, CMD_CLR)
    for i in range(400):
        if i % 25 == 0:
            kp, ki, kd = (0xFFF, 0xFFF, 0xFFF) if i % 100 == 0 else (gain(), gain(), gain())
            aw, rev = rng.randrange(4) != 0, rng.randrange(4) == 0
            await host.wr12(R_KP, kp)
            await host.wr12(R_KI, ki)
            await host.wr12(R_KD, kd)
        ctrl = C_EN | (C_AW if aw else 0) | (C_REV if rev else 0)
        if i % 25 == 0:
            await host.wr8(R_CTRL, ctrl | C_FB)
        if rng.randrange(60) == 0:
            await host.wr8(R_CMD, CMD_CLR)
            model.reset()
        if rng.randrange(8) == 0:
            sp = rng.choice([0, 255, rng.randrange(256)])
            await host.wr8(R_SP, sp)
        fb = rng.choice([0, 255] + [rng.randrange(256)] * 4)
        model.step(sp, fb, kp, ki, kd, aw, rev)
        mode = rng.randrange(10)
        if mode == 0:                          # software tick
            await host.wr8(R_CTRL, ctrl | C_SW)
            await host.wr8(R_FB, fb)
            await host.wr8(R_CMD, CMD_TICK)
            await host.wait_drdy()
            await host.wr8(R_CTRL, ctrl | C_FB)
        elif mode == 1:                        # external tick pin
            await host.wr8(R_CTRL, ctrl | C_EXT)
            await host.wr8(R_FB, fb)
            await host.tick_pin()
            await host.wait_drdy()
            await host.wr8(R_CTRL, ctrl | C_FB)
        else:                                  # writing FB starts the sample
            await host.wr8(R_FB, fb)
            await host.wait_drdy()
        model.check(host, f"sample {i} (sp={sp} fb={fb} kp={kp} ki={ki} kd={kd} aw={aw} rev={rev})")
        n_hold += model.hold
        n_hi += model.sat_hi
        n_lo += model.sat_lo
    dut._log.info(f"400 samples match the model (HOLD {n_hold}, SAT_HI {n_hi}, SAT_LO {n_lo})")
    assert n_hold and n_hi and n_lo, "a corner case was never reached"


@cocotb.test()
async def test_timer_and_latency(dut):
    """TS = 99 -> one sample every 100 clk; BUSY high 38 clk; a write clears DRDY."""
    host, _ = await start(dut)
    await host.wr12(R_KP, 256)
    await host.wr8(R_SP, 50)
    await host.wr12(R_TS, 99)
    await host.wr8(R_CTRL, C_EN | C_AW | C_TIMER)

    async def clk_until(bit, level):
        n = 0
        while host.pin(bit) != level:
            await FallingEdge(dut.clk)
            n += 1
        return n

    await clk_until(O_BUSY, 0)
    await clk_until(O_BUSY, 1)
    busy = await clk_until(O_BUSY, 0)
    await ClockCycles(dut.clk, 2, rising=False)
    assert host.pin(O_DRDY) == 1, "DRDY after the sample"
    assert host.u == 50, f"U = {host.u} (Kp = 1, e = 50)"
    rest = 2 + await clk_until(O_BUSY, 1)
    assert busy == 38, f"BUSY width {busy} clk"
    assert busy + rest == 100, f"timer period {busy + rest} clk"
    await clk_until(O_BUSY, 0)
    await ClockCycles(dut.clk, 2, rising=False)
    assert host.pin(O_DRDY) == 1
    await host.wr8(R_CTRL, C_EN | C_AW | C_SW)   # ends ~60 clk before the next tick
    assert host.pin(O_DRDY) == 0, "DRDY cleared by a write"
    dut._log.info(f"timer period {busy + rest} clk, BUSY {busy} clk")


@cocotb.test()
async def test_enable_and_async_reset(dut):
    """EN = 0 forces U = 0 and stops sampling; reset works with the clock stopped."""
    host, clock = await start(dut)
    await host.wr12(R_KP, 512)
    await host.wr8(R_CTRL, C_EN | C_AW | C_SW)
    await host.wr8(R_SP, 200)
    await host.wr8(R_FB, 0)
    await host.wr8(R_CMD, CMD_TICK)
    await host.wait_drdy()
    assert host.u == 255, f"U = {host.u} (Kp = 2, e = 200 -> saturated)"
    await host.wr8(R_CTRL, C_AW | C_TIMER)    # EN = 0, timer selected
    await ClockCycles(dut.clk, 3, rising=False)
    assert host.u == 0, "U = 0 with EN = 0"
    for _ in range(300):
        await FallingEdge(dut.clk)
        assert host.pin(O_BUSY) == 0, "no samples with EN = 0"

    await host.wr8(R_CTRL, C_EN | C_AW | C_SW)
    await host.wr8(R_CMD, CMD_TICK)
    await host.wait_drdy()
    assert host.u == 255
    await FallingEdge(dut.clk)
    clock.stop()
    await Timer(300, unit="ns")
    dut.rst_n.value = 0
    await Timer(10, unit="ns")
    assert host.u == 0, "U = 0 at once on reset, clock stopped"
    assert val(dut.uio_out) & 0xF8 == 0
    dut.rst_n.value = 1


@cocotb.test()
async def test_closed_loop(dut):
    """First-order plant (tau = 20 samples): step to 128, then a load of 60."""
    host, _ = await start(dut)
    await host.wr12(R_KP, 512)                # Kp = 2.0
    await host.wr12(R_KI, 64)                 # Ki = 0.25
    await host.wr12(R_KD, 0)
    await host.wr8(R_CTRL, C_EN | C_AW | C_FB)
    await host.wr8(R_SP, 128)
    y, load, y_max = 0.0, 0.0, 0.0
    for i in range(300):
        if i == 150:
            load = 60.0
        fb = min(max(int(y + 0.5), 0), 255)
        await host.wr8(R_FB, fb)
        await host.wait_drdy()
        y += (host.u - y - load) / 20.0
        if i < 150:
            y_max = max(y_max, y)
        if i in (149, 299):
            assert abs(fb - 128) <= 1, f"sample {i}: FB = {fb}"
    overshoot = (y_max - 128) / 1.28
    assert overshoot < 15, f"overshoot {overshoot:.1f} %"
    dut._log.info(f"closed loop: overshoot {overshoot:.1f} %, settled at 128 with and without load")
