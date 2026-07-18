#!/usr/bin/env bash
#
# sync-to-px4.sh — mirror the flightaxis files FROM this repo INTO the PX4 tree.
#
#   ./scripts/sync-to-px4.sh [PATH_TO_PX4] [--dry-run] [--yes]
#
# A maintainer tool for the edit-here-run-there loop; end users want ./install.sh,
# which also does the CMakeLists registrations, the backups and the build.
#
# Deliberately does NOT --delete: the payload directory in the PX4 tree is where
# users are told to keep their own models/<name>.json, and this script is not
# allowed to destroy them. install.sh follows the same rule (writing into the
# PX4 tree never deletes); only sync-from-px4.sh, whose destination is this repo,
# uses --delete.
#
set -euo pipefail

REPO_DIR="$(cd -P "$(dirname "$0")/.." && pwd -P)"

usage() {
	cat <<'EOF'
Usage: ./scripts/sync-to-px4.sh [PATH_TO_PX4] [options]

Copies this repo's flightaxis files into a PX4-Autopilot checkout.

Options:
  --dry-run   Show exactly what would change; write nothing.
  --yes, -y   Do not ask for confirmation.
  -h, --help  Show this help.
EOF
}

PX4_ARG=""
DRY_RUN=0
ASSUME_YES=0
while [ $# -gt 0 ]; do
	case "$1" in
		--dry-run) DRY_RUN=1 ;;
		-y|--yes)  ASSUME_YES=1 ;;
		-h|--help) usage; exit 0 ;;
		# Options used to fall through to px4_resolve "${1:-}", so
		# `sync-to-px4.sh --dry-run` was interpreted as a PX4 *path* and died
		# with a confusing "that path does not exist".
		-*)        echo "unknown option: $1 (try --help)" >&2; exit 1 ;;
		*)
			[ -z "$PX4_ARG" ] || { echo "unexpected extra argument: $1" >&2; exit 1; }
			PX4_ARG="$1"
			;;
	esac
	shift
done

# shellcheck source=detect-px4.sh
. "$REPO_DIR/scripts/detect-px4.sh"
px4_resolve "$PX4_ARG" || exit 1
PX4_DIR="$PX4_RESOLVED"
echo "PX4 tree: $PX4_DIR  ($PX4_RESOLVED_HOW)"

[ "$PX4_DIR" != "$REPO_DIR" ] || { echo "error: the target PX4 tree cannot be this repository" >&2; exit 1; }

# The same guard install.sh has: an unattended run must never write into a tree
# that was merely guessed at.
if [ "$PX4_RESOLVED_EXPLICIT" -eq 0 ] && [ ! -t 0 ] && [ "$DRY_RUN" -eq 0 ]; then
	echo "error: refusing to write into an auto-detected PX4 tree in a non-interactive run." >&2
	echo "       Pass the path explicitly or set \$PX4_DIR." >&2
	exit 1
fi

# Always show the diff first: this overwrites files in a live PX4 tree and used
# to do it with no preview, no prompt and no --dry-run at all.
echo
echo "Changes this would make in $PX4_DIR:"
rsync -a --exclude __pycache__ --exclude '*.pyc' -n -i \
	"$REPO_DIR/Tools/simulation/flightaxis/" \
	"$PX4_DIR/Tools/simulation/flightaxis/" | sed 's/^/  /'

if [ "$DRY_RUN" -eq 1 ]; then
	echo
	echo "Dry run - nothing was written."
	exit 0
fi

# Confirm when the tree was auto-detected: rsync into the wrong checkout is not
# something the user can undo.
if [ "$PX4_RESOLVED_EXPLICIT" -eq 0 ] && [ "$ASSUME_YES" -eq 0 ] && [ -t 0 ]; then
	printf '\nThis PX4 tree was auto-detected. Write these changes into it? [y/N] '
	read -r reply || reply=""
	case "$reply" in
		y|Y|yes|YES|Yes) : ;;
		*) echo "Aborted; nothing was changed."; exit 1 ;;
	esac
fi

# --exclude '*.pyc' matches install.sh; it was missing here, so a stray .pyc
# built in the repo got mirrored into the PX4 tree.
rsync -a --exclude __pycache__ --exclude '*.pyc' -v \
	"$REPO_DIR/Tools/simulation/flightaxis/" \
	"$PX4_DIR/Tools/simulation/flightaxis/"

cp -v "$REPO_DIR/src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake" \
	"$PX4_DIR/src/modules/simulation/simulator_mavlink/"

# Copy the airframes by name from the shared manifest. The old unquoted
# 12??_flightaxis_* glob expanded to a literal, unmatched pattern when nothing
# matched, and aborted the script under `set -e`.
for a in $FA_AIRFRAMES; do
	cp -v "$REPO_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/$a" \
		"$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"
done

# Same whole-line matchers install.sh and uninstall.sh use, so all three agree
# on whether a registration is present.
if ! fa_has_include "$PX4_DIR/src/modules/simulation/simulator_mavlink/CMakeLists.txt" \
	'include(sitl_targets_flightaxis.cmake)'; then
	echo
	echo "WARNING: sitl_targets_flightaxis.cmake is NOT included from"
	echo "  $PX4_DIR/src/modules/simulation/simulator_mavlink/CMakeLists.txt"
	echo "Add:  include(sitl_targets_flightaxis.cmake)   (see README.md install step 2)"
fi
if ! grep -qE "$FA_ENTRY_RE" "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"; then
	echo
	echo "WARNING: flightaxis airframes are NOT registered in"
	echo "  $PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt (README step 3)"
fi

echo
echo "Synced repo -> PX4."
