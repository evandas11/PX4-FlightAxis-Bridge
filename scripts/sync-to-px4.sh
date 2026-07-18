#!/usr/bin/env bash
# Install/update the flightaxis integration files FROM this repo INTO the PX4 tree.
# Run this after editing anything in this repo.
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

[ -d "$PX4_DIR/Tools/simulation" ] || {
	echo "$PX4_DIR does not look like a PX4-Autopilot checkout (set PX4_DIR)"; exit 1; }

rsync -av --delete --exclude __pycache__ \
	"$REPO_DIR/Tools/simulation/flightaxis/" \
	"$PX4_DIR/Tools/simulation/flightaxis/"

cp -v "$REPO_DIR/src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake" \
	"$PX4_DIR/src/modules/simulation/simulator_mavlink/"

cp -v "$REPO_DIR"/ROMFS/px4fmu_common/init.d-posix/airframes/12??_flightaxis_* \
	"$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"

if ! grep -q "sitl_targets_flightaxis" "$PX4_DIR/src/modules/simulation/simulator_mavlink/CMakeLists.txt"; then
	echo
	echo "WARNING: sitl_targets_flightaxis.cmake is NOT included from"
	echo "  $PX4_DIR/src/modules/simulation/simulator_mavlink/CMakeLists.txt"
	echo "Add:  include(sitl_targets_flightaxis.cmake)   (see README.md install step 2)"
fi
if ! grep -q "flightaxis" "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"; then
	echo
	echo "WARNING: flightaxis airframes are NOT registered in"
	echo "  $PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt (README step 3)"
fi

echo
echo "Synced repo -> PX4."
