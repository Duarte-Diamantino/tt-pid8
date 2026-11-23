#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Everything for the 8-bit PID (tt_um_duarte_diamantino_pid8), in order:
#   1. lint + RTL testbench (bit-exact model, 20 000 random samples)
#   2. SKY130 synthesis -> area (needs synth/lib, see scripts/get_liberty.sh)
#   3. Wokwi gates: Yosys -> build/gates.json -> diagram.json
#   4. diagram.json read back as Verilog -> the SAME testbench on the gates
#   5. plant8.chip.c compiled natively and checked
# Run from WSL / Linux:  bash pid8/run_all.sh      (NSAMPLE=... to change T3)
set -uo pipefail
cd "$(dirname "$0")"
NSAMPLE=${NSAMPLE:-20000}
RTL="src/pid_sync.v src/pid8_tick.v src/pid8_regs.v src/pid8_core.v src/tt_um_duarte_diamantino_pid8.v"
mkdir -p sim/build wokwi/build
FAILED=()

step () { echo; echo "=== $*"; }
check () { if grep -q "$2" "$3"; then echo "PASS $1"; else echo "FAIL $1"; FAILED+=("$1"); tail -15 "$3"; fi; }

step "lint"
if iverilog -g2005 -Wall -s tt_um_duarte_diamantino_pid8 -o /dev/null $RTL 2>&1 | tee sim/build/lint.log | grep -q .; then
    cat sim/build/lint.log; FAILED+=("lint")
else echo "PASS lint"; fi

step "RTL testbench ($NSAMPLE random samples)"
iverilog -g2012 -Wall -Wno-timescale -s tb_pid8 -o sim/build/tb_rtl.vvp sim/tb_pid8.v $RTL \
    && vvp -n sim/build/tb_rtl.vvp +N="$NSAMPLE" > sim/build/tb_rtl.log
grep -E "^T[0-9]" sim/build/tb_rtl.log
check "rtl" "TEST PASSED" sim/build/tb_rtl.log

step "SKY130 synthesis"
if [ -f synth/lib/sky130_fd_sc_hd__tt_025C_1v80.lib ]; then
    bash synth/synth.sh | grep -E "Chip area for (top )?module|Number of cells" | tail -4
else echo "skipped: liberty not found (run scripts/get_liberty.sh)"; fi

step "Wokwi gates"
(cd wokwi && yosys -q gates.ys && python3 gen_wokwi.py build/gates.json diagram.json \
    && python3 diagram_to_verilog.py diagram.json build/pid8_gates.v) || FAILED+=("wokwi-gen")

step "testbench on the gates read back from diagram.json (3000 samples)"
iverilog -g2012 -Wno-timescale -s tb_pid8 -o sim/build/tb_gates.vvp sim/tb_pid8.v wokwi/build/pid8_gates.v \
    && vvp -n sim/build/tb_gates.vvp +N=3000 > sim/build/tb_gates.log
grep -E "^T[0-9]" sim/build/tb_gates.log
check "gates" "TEST PASSED" sim/build/tb_gates.log

step "plant8.chip.c (native)"
gcc -std=c11 -Wall -I wokwi/test -o wokwi/build/plant8_host wokwi/test/plant8_host.c wokwi/plant8.chip.c -lm \
    && wokwi/build/plant8_host > wokwi/build/plant8_host.log
cat wokwi/build/plant8_host.log
check "plant8-chip" "PASS plant8" wokwi/build/plant8_host.log

echo
if [ ${#FAILED[@]} -eq 0 ]; then echo "ALL PASSED"; else echo "FAILED: ${FAILED[*]}"; exit 1; fi
