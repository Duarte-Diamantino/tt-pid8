#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
gen_wokwi.py - turn the Yosys gate netlist of tt_um_duarte_diamantino_pid8 (build/gates.json,
made by gates.ys) into a Wokwi diagram.json built only from Wokwi logic parts,
plus the host (Arduino Mega), the plant chip, a potentiometer and a switch.

Cell mapping (semantics checked in Wokwi, see README.md):
  $_NOT_                          wokwi-gate-not      A->IN, Y->OUT
  $_AND_ $_OR_ $_XOR_ $_NAND_ ... wokwi-gate-<op>-2   A, B -> OUT
  $_MUX_                          wokwi-mux-2         A, B, S->SEL, Y->OUT (SEL=0 -> A)
  $_DFF_P_                        wokwi-flip-flop-d   C->CLK, D, Q
  $_DFF_PN0_ / $_DFF_PN1_         wokwi-flip-flop-dsr C->CLK, D, Q, R/S active high
                                  (driven by one NOT of the active-low reset net)
  $_BUF_                          merged into the net
Constants come from two buffers tied to the Mega's GND and 5V pins.
The chip pins are buffers ("port" parts) so the boundary of the circuit is visible.

Usage: python3 gen_wokwi.py build/gates.json diagram.json
"""
import json
import math
import sys
from collections import defaultdict

SRC = sys.argv[1] if len(sys.argv) > 1 else "build/gates.json"
DST = sys.argv[2] if len(sys.argv) > 2 else "diagram.json"

TOP = "tt_um_duarte_diamantino_pid8"
GATE2 = {"$_AND_": "and", "$_OR_": "or", "$_XOR_": "xor",
         "$_NAND_": "nand", "$_NOR_": "nor", "$_XNOR_": "xnor"}

# chip pin -> host connection(s); outputs may go to several places
PORT_IN = {   # (TT port, bit) : (label, Mega pin)
    ("ui_in", i): (f"D{i}", f"mega:{22 + i}") for i in range(8)}
PORT_IN.update({
    ("uio_in", 0): ("WR", "mega:2"),
    ("uio_in", 1): ("AD", "mega:3"),
    ("uio_in", 2): ("TICK", "mega:5"),
    ("rst_n", 0): ("RST_N", "mega:6"),
    ("clk", 0): ("CLK", "mega:11"),
})
PORT_OUT = {
    ("uo_out", i): (f"U{i}", [f"mega:{37 - i}", f"plant:U{i}"]) for i in range(8)}
PORT_OUT.update({
    ("uio_out", 3): ("HOLD", ["mega:12"]),
    ("uio_out", 4): ("DRDY", ["mega:7"]),
    ("uio_out", 5): ("SAT_HI", ["mega:8"]),
    ("uio_out", 6): ("SAT_LO", ["mega:9"]),
    ("uio_out", 7): ("BUSY", ["mega:10"]),
})

BLOCKS = [  # (instance names, label, wire colour)
    (("top", "u_sync_wr", "u_sync_tck"), "SINCRONIZACAO + RESET + DRDY", "violet"),
    (("u_regs",), "REGISTOS (SP FB KP KI KD TS CTRL)", "blue"),
    (("u_tick",), "TIMER DE AMOSTRAGEM", "orange"),
    (("u_core",), "NUCLEO PID (MAC SERIE)", "green"),
]

# ---------------------------------------------------------------- netlist
j = json.load(open(SRC))
mods = j["modules"]

parent = {}


def find(n):
    while parent.setdefault(n, n) != n:
        parent[n] = parent[parent[n]]
        n = parent[n]
    return n


def union(a, b):
    if isinstance(a, str) or isinstance(b, str):
        raise SystemExit(f"constant merged into a net ({a}, {b}): not supported")
    parent[find(a)] = find(b)


counter = [0]


def new_net():
    counter[0] += 1
    return counter[0]


cells = []   # dict(name, block, type, conns{port: net})


def flatten(modname, path, block, bitmap):
    m = mods[modname]
    local = {}

    def g(b):
        if isinstance(b, str):
            return b                       # "0", "1", "x"
        if b in bitmap:
            return bitmap[b]
        if b not in local:
            local[b] = new_net()
        return local[b]

    for cname, c in m["cells"].items():
        t = c["type"]
        if t in mods:
            sub = mods[t]
            subbitmap = {}
            for p, bits in c["connections"].items():
                for sb, pb in zip(sub["ports"][p]["bits"], bits):
                    gp = g(pb)
                    if sb in subbitmap and subbitmap[sb] != gp:
                        union(subbitmap[sb], gp)
                    else:
                        subbitmap[sb] = gp
            flatten(t, path + [cname], block if block != "top" else cname, subbitmap)
        else:
            cells.append(dict(name=".".join(path + [cname]), block=block, type=t,
                              conns={p: g(bits[0]) for p, bits in c["connections"].items()}))


top = mods[TOP]
topmap = {}
ports = {}          # (port, bit) -> net
for pname, p in top["ports"].items():
    for i, b in enumerate(p["bits"]):
        if isinstance(b, str):
            continue
        if b not in topmap:
            topmap[b] = new_net()
        ports[(pname, i)] = topmap[b]
flatten(TOP, [], "top", topmap)


def R(n):
    return n if isinstance(n, str) else find(n)


# buffers: merge nets
for c in [c for c in cells if c["type"] == "$_BUF_"]:
    union(c["conns"]["A"], c["conns"]["Y"])
cells = [c for c in cells if c["type"] != "$_BUF_"]

# ---------------------------------------------------------------- parts
parts = []          # dict(id, type, block, level, attrs)
pins = []           # (part_id, pin, net, is_driver)


def add_part(pid, ptype, block, attrs=None):
    parts.append(dict(id=pid, type=ptype, block=block, attrs=attrs or {}))
    return pid


def connect(pid, pin, net, drv=False):
    pins.append((pid, pin, net, drv))


# constant sources
add_part("tie0", "wokwi-gate-buffer", "top")
add_part("tie1", "wokwi-gate-buffer", "top")
connect("tie0", "OUT", "0", True)
connect("tie1", "OUT", "1", True)

rst_inv = {}        # active-low reset net -> NOT part id


def reset_high(net):
    net = R(net)
    if net not in rst_inv:
        pid = add_part(f"rstinv{len(rst_inv)}", "wokwi-gate-not", "top")
        connect(pid, "IN", net)
        inv_net = new_net()
        connect(pid, "OUT", inv_net, True)
        rst_inv[net] = (pid, inv_net)
    return rst_inv[net][1]


count = defaultdict(int)
for c in cells:
    t, k = c["type"], c["conns"]
    count[t] += 1
    pid = f"g{len(parts)}"
    blk = c["block"]
    if t in GATE2:
        add_part(pid, f"wokwi-gate-{GATE2[t]}-2", blk)
        connect(pid, "A", k["A"]); connect(pid, "B", k["B"]); connect(pid, "OUT", k["Y"], True)
    elif t == "$_NOT_":
        add_part(pid, "wokwi-gate-not", blk)
        connect(pid, "IN", k["A"]); connect(pid, "OUT", k["Y"], True)
    elif t == "$_MUX_":
        add_part(pid, "wokwi-mux-2", blk)
        connect(pid, "A", k["A"]); connect(pid, "B", k["B"]); connect(pid, "SEL", k["S"])
        connect(pid, "OUT", k["Y"], True)
    elif t == "$_DFF_P_":
        add_part(pid, "wokwi-flip-flop-d", blk)
        connect(pid, "D", k["D"]); connect(pid, "CLK", k["C"]); connect(pid, "Q", k["Q"], True)
    elif t in ("$_DFF_PN0_", "$_DFF_PN1_"):
        add_part(pid, "wokwi-flip-flop-dsr", blk)
        connect(pid, "D", k["D"]); connect(pid, "CLK", k["C"]); connect(pid, "Q", k["Q"], True)
        rh = reset_high(k["R"])
        if t == "$_DFF_PN0_":
            connect(pid, "R", rh); connect(pid, "S", "0")
        else:
            connect(pid, "S", rh); connect(pid, "R", "0")
    else:
        raise SystemExit(f"unsupported cell {t} ({c['name']})")

# chip pins
external = []       # (part pin, host pin, colour)
for key, (label, host) in PORT_IN.items():
    pid = f"in_{label}"
    add_part(pid, "wokwi-gate-buffer", "in")
    connect(pid, "OUT", ports[key], True)
    external.append((f"{pid}:IN", host, "gray"))
for key, (label, hosts) in PORT_OUT.items():
    pid = f"out_{label}"
    add_part(pid, "wokwi-gate-buffer", "out")
    connect(pid, "IN", ports[key])
    for h in hosts:
        external.append((f"{pid}:OUT", h, "gray"))
external.append(("tie0:IN", "mega:GND.3", "black"))
external.append(("tie1:IN", "mega:5V", "red"))

# ---------------------------------------------------------------- nets
drivers = {}
sinks = defaultdict(list)
for pid, pin, net, drv in pins:
    net = R(net)
    if drv:
        if net in drivers:
            raise SystemExit(f"net {net} has two drivers: {drivers[net]} and {pid}:{pin}")
        drivers[net] = (pid, pin)
    else:
        sinks[net].append((pid, pin))
undriven = [n for n in sinks if n not in drivers]
if undriven:
    raise SystemExit(f"{len(undriven)} nets without a driver, e.g. {undriven[:5]}")

# levels (logic depth) for the layout: flip-flops, pins and constants = 0
ptype = {p["id"]: p["type"] for p in parts}
inputs_of = defaultdict(list)
for pid, pin, net, drv in pins:
    if not drv:
        inputs_of[pid].append(R(net))
level = {}


def lvl(pid):
    if pid in level:
        return level[pid]
    t = ptype[pid]
    if "flip-flop" in t or pid.startswith(("in_", "tie")):
        level[pid] = 0
        return 0
    level[pid] = 0          # guard against loops
    level[pid] = 1 + max([lvl(drivers[n][0]) for n in inputs_of[pid]] or [0])
    return level[pid]


for p in parts:
    p["level"] = lvl(p["id"])

# ---------------------------------------------------------------- layout
DX, DY, ROWS = 140, 70, 36
X0, Y0 = 900, 0             # the host (Mega, plant, pot, switch) sits left of X0
placed = {}
labels = []


def place_column(ids, x, y):
    for i, pid in enumerate(ids):
        placed[pid] = (x, y + i * DY)


in_ids = [p["id"] for p in parts if p["block"] == "in"]
place_column(in_ids, X0, Y0 + 60)
labels.append(("CHIP: ENTRADAS", X0, Y0))
x = X0 + 2 * DX
block_colour = {}
for names, label, colour in BLOCKS:
    ids = [p["id"] for p in parts if p["block"] in names]
    for n in names:
        block_colour[n] = colour
    ids.sort(key=lambda pid: (level[pid], ptype[pid], pid))
    ncol = math.ceil(len(ids) / ROWS)
    rows = math.ceil(len(ids) / ncol)
    for col in range(ncol):
        place_column(ids[col * rows:(col + 1) * rows], x + col * DX, Y0 + 60)
    labels.append((f"{label} - {len(ids)} pecas", x, Y0))
    x += ncol * DX + DX
out_ids = [p["id"] for p in parts if p["block"] == "out"]
place_column(out_ids, x, Y0 + 60)
labels.append(("CHIP: SAIDAS", x, Y0))
block_colour.update({"in": "gray", "out": "gray"})

# ---------------------------------------------------------------- diagram.json
diag_parts = [
    {"type": "wokwi-arduino-mega", "id": "mega", "top": 330, "left": 0, "attrs": {}},
    {"type": "chip-plant8", "id": "plant", "top": 40, "left": 330,
     "attrs": {"tauMs": "200", "load": "75", "gain": "1"}},
    {"type": "wokwi-potentiometer", "id": "pot", "top": 60, "left": 40, "attrs": {"value": "512"}},
    {"type": "wokwi-slide-switch", "id": "sw", "top": 80, "left": 190, "attrs": {"bounce": "0"}},
]
n_ff_cells = sum(v for t, v in count.items() if "DFF" in t)
n_gate_cells = sum(count.values()) - n_ff_cells
diag_parts.append({"type": "wokwi-text", "id": "title", "top": -60, "left": 0, "attrs": {
    "text": f"PID 8 BITS (tt_um_pid8) EM PORTAS LOGICAS: {n_gate_cells} portas + "
            f"{n_ff_cells} flip-flops. Potenciometro = SP, interruptor = carga. Circuito a direita ->"}})
for i, (text, lx, ly) in enumerate(labels):
    diag_parts.append({"type": "wokwi-text", "id": f"label{i}", "top": ly, "left": lx,
                       "attrs": {"text": text}})
for p in parts:
    x, y = placed[p["id"]]
    diag_parts.append({"type": p["type"], "id": p["id"], "top": y, "left": x, "attrs": p["attrs"]})
    if p["block"] in ("in", "out"):
        diag_parts.append({"type": "wokwi-text", "id": f"t_{p['id']}", "top": y - 14,
                           "left": x + 20, "attrs": {"text": p["id"].split("_", 1)[1]}})

part_block = {p["id"]: p["block"] for p in parts}
clk_net = R(ports[("clk", 0)])
rst_nets = {rst_inv[n][1] for n in rst_inv} | set(rst_inv)
conns = []
for net, (dp, dpin) in drivers.items():
    if net == clk_net:
        colour = "gold"
    elif net in rst_nets:
        colour = "red"
    elif net in ("0", "1"):
        colour = "black"
    else:
        colour = block_colour.get(part_block[dp], "green")
    for sp, spin in sinks.get(net, []):
        conns.append([f"{dp}:{dpin}", f"{sp}:{spin}", colour, []])
for a, b, colour in external:
    conns.append([a, b, colour, []])
conns += [
    ["plant:Y", "mega:A1", "gold", []],
    ["plant:GND", "mega:GND.2", "black", []],
    ["plant:VCC", "mega:5V", "red", []],
    ["pot:SIG", "mega:A0", "gold", []],
    ["pot:SIG", "plant:SP", "gold", []],
    ["pot:VCC", "mega:5V", "red", []],
    ["pot:GND", "mega:GND.2", "black", []],
    ["sw:2", "plant:DIST", "white", []],
    ["sw:1", "mega:GND.2", "black", []],
    ["sw:3", "mega:5V", "red", []],
]

diagram = {"version": 1, "author": "tt_pid", "editor": "wokwi", "parts": diag_parts,
           "connections": conns, "serialMonitor": {"display": "always"}, "dependencies": {}}
with open(DST, "w", newline="\n") as f:
    json.dump(diagram, f, separators=(",", ":"))

n_ff = sum(1 for p in parts if "flip-flop" in p["type"])
n_gate = sum(1 for p in parts if p["block"] not in ("in", "out") and "flip-flop" not in p["type"])
print(f"{DST}: {len(diag_parts)} parts ({n_gate} gates incl. reset/constant helpers, "
      f"{n_ff} flip-flops, {len(in_ids)} input pins, {len(out_ids)} output pins), "
      f"{len(conns)} wires, max logic depth {max(level.values())}")
print("cells:", dict(sorted(count.items())))
