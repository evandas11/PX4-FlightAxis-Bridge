#!/usr/bin/env bash
# Mirror the flightaxis integration files FROM the PX4 tree INTO this repo.
# Run this after editing anything directly in the PX4 tree.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# Shared resolution: argument, then $PX4_DIR, then auto-detection.
# shellcheck source=detect-px4.sh
. "$REPO_DIR/scripts/detect-px4.sh"
px4_resolve "${1:-}" || exit 1
PX4_DIR="$PX4_RESOLVED"
echo "PX4 tree: $PX4_DIR  ($PX4_RESOLVED_HOW)"

[ -d "$PX4_DIR/Tools/simulation/flightaxis" ] || {
	echo "flightaxis is not installed in $PX4_DIR — run ./install.sh first"; exit 1; }

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
