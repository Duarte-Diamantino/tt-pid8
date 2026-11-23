#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Download the SKY130 HD standard-cell liberty (typical corner, 25 C, 1.80 V)
# used by synth/synth.sh. ~12.8 MB. Source: OpenROAD-flow-scripts (the google
# skywater-pdk-libs-sky130_fd_sc_hd repo only ships the .lib.json sources).
#
# Alternative if you have a full PDK (volare / open_pdks):
#   LIB=$PDK_ROOT/sky130A/libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib ./synth/synth.sh
set -euo pipefail
cd "$(dirname "$0")/../synth"
mkdir -p lib
URL=https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD-flow-scripts/master/flow/platforms/sky130hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib
if [ -f lib/sky130_fd_sc_hd__tt_025C_1v80.lib ]; then
    echo "already present: synth/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"
else
    curl -fL -o lib/sky130_fd_sc_hd__tt_025C_1v80.lib "$URL"
fi
