#!/usr/bin/env bash
# Mirror the flightaxis integration files FROM the PX4 tree INTO this repo.
# Run this after editing anything directly in the PX4 tree.
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

[ -d "$PX4_DIR/Tools/simulation/flightaxis" ] || {
	echo "flightaxis not found in $PX4_DIR (set PX4_DIR to your PX4-Autopilot checkout)"; exit 1; }

mkdir -p "$REPO_DIR/Tools/simulation/flightaxis"
rsync -av --delete --exclude __pycache__ \
	"$PX4_DIR/Tools/simulation/flightaxis/" \
	"$REPO_DIR/Tools/simulation/flightaxis/"

mkdir -p "$REPO_DIR/src/modules/simulation/simulator_mavlink"
cp -v "$PX4_DIR/src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake" \
	"$REPO_DIR/src/modules/simulation/simulator_mavlink/"

mkdir -p "$REPO_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
cp -v "$PX4_DIR"/ROMFS/px4fmu_common/init.d-posix/airframes/12??_flightaxis_* \
	"$REPO_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"

echo
echo "Synced PX4 -> repo. Remember: the include() in simulator_mavlink/CMakeLists.txt and"
echo "the airframe entries in init.d-posix/airframes/CMakeLists.txt are documented in"
echo "README.md, not mirrored as files."
