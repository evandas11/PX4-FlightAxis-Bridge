#!/usr/bin/env bash
#
# sync-from-px4.sh — mirror the flightaxis files FROM the PX4 tree INTO this repo.
#
#   ./scripts/sync-from-px4.sh [PATH_TO_PX4] [--dry-run] [--yes]
#
# This is the only script in the project that rsyncs with --delete, and the
# destination is the published repository, so it is also the only one that can
# destroy work that is not recoverable from the PX4 tree. It therefore refuses
# to run unless the source tree holds a COMPLETE payload: the old version
# checked only that Tools/simulation/flightaxis existed, so an aborted install
# or a hand-cleaned tree happily propagated its incompleteness into the repo and
# deleted the missing files here - untracked ones for good.
#
set -euo pipefail

REPO_DIR="$(cd -P "$(dirname "$0")/.." && pwd -P)"

usage() {
	cat <<'EOF'
Usage: ./scripts/sync-from-px4.sh [PATH_TO_PX4] [options]

Copies the flightaxis files out of a PX4-Autopilot checkout into this repo.
Uses rsync --delete, so files absent from the PX4 tree are removed here.

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

[ "$PX4_DIR" != "$REPO_DIR" ] || { echo "error: the source PX4 tree cannot be this repository" >&2; exit 1; }

# The same guard install.sh has, which this script lacked entirely.
if [ "$PX4_RESOLVED_EXPLICIT" -eq 0 ] && [ ! -t 0 ] && [ "$DRY_RUN" -eq 0 ]; then
	echo "error: refusing to sync from an auto-detected PX4 tree in a non-interactive run." >&2
	echo "       Pass the path explicitly or set \$PX4_DIR." >&2
	exit 1
fi

# --- completeness gate --------------------------------------------------------
# Every file of the shared manifest must exist in the PX4 tree before we are
# willing to --delete anything here.
MISSING=""
while IFS= read -r rel; do
	[ -n "$rel" ] || continue
	[ -f "$PX4_DIR/$rel" ] || MISSING="$MISSING$rel
"
done <<EOF
$(fa_payload_files)
EOF

if [ -n "$MISSING" ]; then
	echo >&2
	echo "error: the flightaxis payload in $PX4_DIR is incomplete:" >&2
	printf '%s' "$MISSING" | sed 's/^/         /' >&2
	echo "       Syncing --delete from an incomplete tree would remove these files from" >&2
	echo "       this repository too, and untracked ones would not come back." >&2
	echo "       Run ./install.sh first, or fix the tree." >&2
	exit 1
fi
echo "payload complete in the PX4 tree ($(fa_payload_files | grep -c .) files)"

# --- preview ------------------------------------------------------------------
echo
echo "Changes this would make in $REPO_DIR:"
mkdir -p "$REPO_DIR/Tools/simulation/flightaxis"
rsync -a --delete --exclude __pycache__ --exclude '*.pyc' -n -i \
	"$PX4_DIR/Tools/simulation/flightaxis/" \
	"$REPO_DIR/Tools/simulation/flightaxis/" | sed 's/^/  /'

if [ "$DRY_RUN" -eq 1 ]; then
	echo
	echo "Dry run - nothing was written."
	exit 0
fi

if [ "$PX4_RESOLVED_EXPLICIT" -eq 0 ] && [ "$ASSUME_YES" -eq 0 ] && [ -t 0 ]; then
	printf '\nThis PX4 tree was auto-detected, and this deletes files here. Continue? [y/N] '
	read -r reply || reply=""
	case "$reply" in
		y|Y|yes|YES|Yes) : ;;
		*) echo "Aborted; nothing was changed."; exit 1 ;;
	esac
fi

# --- copy ---------------------------------------------------------------------
rsync -a --delete --exclude __pycache__ --exclude '*.pyc' -v \
	"$PX4_DIR/Tools/simulation/flightaxis/" \
	"$REPO_DIR/Tools/simulation/flightaxis/"

mkdir -p "$REPO_DIR/src/modules/simulation/simulator_mavlink"
cp -v "$PX4_DIR/src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake" \
	"$REPO_DIR/src/modules/simulation/simulator_mavlink/"

# By name from the shared manifest. The old unquoted 12??_flightaxis_* glob
# aborted the script under `set -e` when it matched nothing - and it did so
# AFTER the destructive rsync had already run.
mkdir -p "$REPO_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
for a in $FA_AIRFRAMES; do
	cp -v "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/$a" \
		"$REPO_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"
done

echo
echo "Synced PX4 -> repo. Remember: the include() in simulator_mavlink/CMakeLists.txt and"
echo "the airframe entries in init.d-posix/airframes/CMakeLists.txt are documented in"
echo "README.md, not mirrored as files."
