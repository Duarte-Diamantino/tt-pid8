#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Yosys synthesis of tt_um_duarte_diamantino_pid8 onto SKY130 HD (cell-area estimate before place & route).
#   hier : hierarchy kept -> area per block
#   flat : flattened      -> total area and cell / flip-flop count
# Needs the liberty file: ../scripts/get_liberty.sh
set -euo pipefail
cd "$(dirname "$0")"
LIB=${LIB:-lib/sky130_fd_sc_hd__tt_025C_1v80.lib}
RTL="../src/pid_sync.v ../src/pid8_tick.v ../src/pid8_regs.v ../src/pid8_core.v ../src/tt_um_duarte_diamantino_pid8.v"
TOP=tt_um_duarte_diamantino_pid8
mkdir -p results
for mode in hier flat; do
    flat_cmd=""; [ "$mode" = flat ] && flat_cmd="flatten;"
    yosys -q -l "results/${mode}.log" -p "
        read_verilog $RTL;
        synth -top $TOP;
        $flat_cmd
        dfflibmap -liberty $LIB;
        abc -liberty $LIB;
        opt_clean -purge;
        tee -o results/${mode}.stat stat -liberty $LIB;
        write_verilog -noattr results/${mode}_netlist.v
    " 2>&1 | grep -v "unsupported expression" || true
done
grep -E "Chip area for (top )?module|Number of cells" results/hier.stat
grep -E "Chip area|Number of cells|df[a-z]+_[0-9]" results/flat.stat
