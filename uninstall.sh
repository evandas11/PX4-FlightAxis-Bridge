#!/usr/bin/env bash
#
# uninstall.sh — remove the PX4-FlightAxis-Bridge integration from a PX4 tree.
#
#   ./uninstall.sh [PATH_TO_PX4] [--dry-run] [--yes]
#
# Reverses install.sh exactly: deletes the flightaxis payload and the four
# airframes, and removes the two splices from the PX4-owned CMakeLists.txt
# files (restoring the .flightaxis.bak backups when they still match, otherwise
# removing precisely the lines we added so unrelated later edits survive).
#
# Every other PX4 file is left untouched. Never uses sudo.
#
set -euo pipefail

REPO_DIR="$(cd -P "$(dirname "$0")" && pwd -P)"

# ---------------------------------------------------------------- output ----
if [ -t 1 ] && [ -z "${NO_COLOR:-}" ] && command -v tput >/dev/null 2>&1 \
   && [ "$(tput colors 2>/dev/null || echo 0)" -ge 8 ]; then
	C_RED=$(tput setaf 1); C_GRN=$(tput setaf 2); C_YEL=$(tput setaf 3)
	C_BLU=$(tput setaf 4); C_BLD=$(tput bold);    C_OFF=$(tput sgr0)
else
	C_RED=''; C_GRN=''; C_YEL=''; C_BLU=''; C_BLD=''; C_OFF=''
fi

step() { printf '%s==>%s %s%s%s\n' "$C_BLU" "$C_OFF" "$C_BLD" "$*" "$C_OFF"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '    %sok%s   %s\n' "$C_GRN" "$C_OFF" "$*"; }
warn() { printf '%swarning:%s %s\n' "$C_YEL" "$C_OFF" "$*" >&2; }
die()  { printf '%serror:%s %s\n' "$C_RED" "$C_OFF" "$*" >&2; exit 1; }

REMOVED=""
record() { REMOVED="${REMOVED}    $1"$'\n'; }

# ------------------------------------------------------------------ args ----
usage() {
	cat <<'EOF'
Usage: ./uninstall.sh [PATH_TO_PX4] [options]

Removes the FlightAxis (RealFlight) SITL integration from a PX4-Autopilot
checkout and undoes both CMakeLists.txt registrations. Nothing else is touched.

The target tree is resolved exactly like install.sh:
  1. PATH_TO_PX4 positional argument
  2. $PX4_DIR environment variable
  3. auto-detection (enclosing tree, siblings of this repo, conventional
     locations under $HOME, then a bounded search under $HOME)

Options:
  --dry-run     Print every action that would be taken; change nothing.
  --yes, -y     Do not ask for confirmation.
  -h, --help    Show this help.
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
		-*)        die "unknown option: $1 (try --help)" ;;
		*)
			[ -z "$PX4_ARG" ] || die "unexpected extra argument: $1 (try --help)"
			PX4_ARG="$1"
			;;
	esac
	shift
done

# shellcheck source=scripts/detect-px4.sh
. "$REPO_DIR/scripts/detect-px4.sh"

printf '\n%s%sPX4-FlightAxis-Bridge uninstaller%s\n' "$C_BLD" "$C_BLU" "$C_OFF"

px4_resolve "$PX4_ARG" || exit 1
PX4_DIR="$PX4_RESOLVED"
[ "$PX4_DIR" != "$REPO_DIR" ] || die "the target PX4 tree cannot be this repository"

info "target: $PX4_DIR"
info "        version: $(px4_version "$PX4_DIR")"
info "        selected via: $PX4_RESOLVED_HOW"

if [ "$PX4_RESOLVED_EXPLICIT" -eq 0 ] && [ ! -t 0 ] && [ "$DRY_RUN" -eq 0 ]; then
	die "refusing to modify an auto-detected PX4 tree in a non-interactive run.
       Pass the path explicitly or set \$PX4_DIR."
fi

if [ "$DRY_RUN" -eq 0 ] && [ "$ASSUME_YES" -eq 0 ] && [ -t 0 ]; then
	printf '\nRemove the FlightAxis integration from the tree above? [y/N] '
	read -r _reply || _reply=""
	case "$_reply" in
		y|Y|yes|YES|Yes) : ;;
		*) printf 'Aborted; nothing was changed.\n'; exit 1 ;;
	esac
fi
printf '\n'

SIM_CMAKE_REL="src/modules/simulation/simulator_mavlink/CMakeLists.txt"
AF_CMAKE_REL="ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
SIM_CMAKE="$PX4_DIR/$SIM_CMAKE_REL"
AF_CMAKE="$PX4_DIR/$AF_CMAKE_REL"
# HITL airframes are registered in init.d (a board never reads init.d-posix).
HIL_AF_CMAKE_REL="ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt"
HIL_AF_CMAKE="$PX4_DIR/$HIL_AF_CMAKE_REL"

# From scripts/detect-px4.sh - the single source of truth for what we own.
AIRFRAMES="$FA_AIRFRAMES"
HIL_AIRFRAMES="$FA_HIL_AIRFRAMES"
INCLUDE_LINE='include(sitl_targets_flightaxis.cmake)'

run() {
	if [ "$DRY_RUN" -eq 1 ]; then
		printf '    %s[dry-run]%s %s\n' "$C_YEL" "$C_OFF" "$*"
	else
		"$@"
	fi
}

# --------------------------------------------------------------- splices ----
step "1/3  Undoing the CMakeLists.txt registrations"

# Removal filters: the exact inverse of what install.sh inserts. They live in
# scripts/detect-px4.sh alongside the insertion filter, so the two can never
# drift apart - when they did, a CRLF tree was left with entries that install
# thought were registered and uninstall could not remove.
strip_include()   { fa_strip_include "$1" "$INCLUDE_LINE"; }
strip_airframes() { fa_strip_airframes "$1"; }
strip_hil_airframes() { fa_strip_hil_airframes "$1"; }

# Restore from the backup when it is byte-identical to what a surgical removal
# would produce (the normal case). If they differ, the file has been edited
# since install time: keep those edits, remove only our lines, and keep the
# backup around rather than throwing the user's work away.
undo_file() {
	file="$1"; label="$2"; stripper="$3"
	bak="$file.flightaxis.bak"

	[ -f "$file" ] || { warn "$label is missing; skipping"; return 0; }

	# --dry-run is documented as "change nothing", so it must not create the
	# scratch file next to the target either (that also made --dry-run fail on a
	# read-only tree, which is exactly where you would want to use it).
	if [ "$DRY_RUN" -eq 1 ]; then
		tmp="$(mktemp "${TMPDIR:-/tmp}/flightaxis-undo.XXXXXX")"
	else
		tmp="$file.flightaxis.undo.$$"
	fi
	"$stripper" "$file" > "$tmp"

	if cmp -s "$tmp" "$file"; then
		rm -f "$tmp"
		ok "$label contains no flightaxis entries (nothing to undo)"
		if [ -f "$bak" ]; then
			run rm -f "$bak"
			[ "$DRY_RUN" -eq 1 ] || record "deleted  $label.flightaxis.bak (stale backup)"
		fi
		return 0
	fi

	if [ -f "$bak" ] && cmp -s "$tmp" "$bak"; then
		rm -f "$tmp"
		if [ "$DRY_RUN" -eq 1 ]; then
			printf '    %s[dry-run]%s restore %s from %s.flightaxis.bak, then delete the backup\n' \
				"$C_YEL" "$C_OFF" "$label" "$label"
		else
			mv "$bak" "$file"
			ok "restored $label from its .flightaxis.bak backup"
			record "restored $label (from .flightaxis.bak)"
			record "deleted  $label.flightaxis.bak"
		fi
		return 0
	fi

	if [ -f "$bak" ]; then
		warn "$label has been modified since installation; NOT restoring the backup."
		warn "Removing only the flightaxis lines. Backup kept at $label.flightaxis.bak"
	fi
	if [ "$DRY_RUN" -eq 1 ]; then
		printf '    %s[dry-run]%s would remove the flightaxis lines from %s:\n' "$C_YEL" "$C_OFF" "$label"
		diff -u "$file" "$tmp" | sed 's/^/      /' || true
		rm -f "$tmp"
	else
		mv "$tmp" "$file"
		ok "removed the flightaxis lines from $label"
		record "edited   $label (flightaxis lines removed)"
	fi
}

undo_file "$SIM_CMAKE" "$SIM_CMAKE_REL" strip_include
undo_file "$AF_CMAKE"  "$AF_CMAKE_REL"  strip_airframes
# Guarded: a PX4 tree predating the HITL airframes has no such registration to
# undo, and an absent file must not be treated as a failure.
[ -f "$HIL_AF_CMAKE" ] && undo_file "$HIL_AF_CMAKE" "$HIL_AF_CMAKE_REL" strip_hil_airframes

# ---------------------------------------------------------------- payload ----
step "2/3  Removing the FlightAxis payload"

# Delete exactly the files install.sh placed - the shared manifest in
# scripts/detect-px4.sh - and nothing else.
#
# This used to be `rm -rf Tools/simulation/flightaxis`, which destroyed the
# user's own work without a prompt, a listing or a backup. README's "Adding a
# new aircraft" tells users to create models/<name>.json inside that very
# directory, and that directory is untracked in PX4's git, so nothing could
# recover it: follow our own documentation, fly your model for a month, run
# ./uninstall.sh for a clean reinstall, and it was gone permanently. Anything we
# did not install is now left in place and listed for the user.
while IFS= read -r rel; do
	[ -n "$rel" ] || continue
	f="$PX4_DIR/$rel"
	if [ -f "$f" ]; then
		run rm -f "$f"
		[ "$DRY_RUN" -eq 1 ] || record "deleted  $rel"
	fi
done <<EOF
$(fa_payload_files)
EOF
if [ "$DRY_RUN" -eq 0 ]; then
	ok "removed the payload manifest ($(fa_payload_files | grep -c .) files)"
else
	printf '    %s[dry-run]%s would delete the %s files of the payload manifest\n' \
		"$C_YEL" "$C_OFF" "$(fa_payload_files | grep -c .)"
fi

# Now prune the directories the manifest lived in, deepest first, but only when
# they are empty. A directory that still has something in it is the user's:
# report it, keep it.
PAYLOAD_DIR="$PX4_DIR/Tools/simulation/flightaxis"
if [ -d "$PAYLOAD_DIR" ] && [ "$DRY_RUN" -eq 0 ]; then
	# -depth so children are attempted before their parents; rmdir refuses to
	# remove a non-empty directory, which is exactly the guarantee we want.
	find "$PAYLOAD_DIR" -depth -type d -exec rmdir {} + 2>/dev/null || true
fi

if [ -d "$PAYLOAD_DIR" ]; then
	# ${#} rather than sed: PX4_DIR is an arbitrary user path and a '|' or a
	# regex metacharacter in it would mangle the listing.
	SURVIVORS="$(find "$PAYLOAD_DIR" -mindepth 1 \( -type f -o -type l \) 2>/dev/null \
		| while IFS= read -r s; do printf '%s\n' "${s#"$PAYLOAD_DIR"/}"; done | sort || true)"
	if [ -n "$SURVIVORS" ]; then
		warn "Tools/simulation/flightaxis/ still contains files this installer did not
         create, so the directory has been left in place:"
		printf '%s\n' "$SURVIVORS" | sed 's|^|         |' >&2
		printf '       If you no longer want them:  rm -rf %s\n' "$PAYLOAD_DIR" >&2
	fi
elif [ "$DRY_RUN" -eq 0 ]; then
	ok "Tools/simulation/flightaxis/ removed (it was empty afterwards)"
fi

# Stale copies inside an existing build directory would keep the make targets
# alive until the next cmake re-run; remove ours, leave everything else.
BUILD_AF="$PX4_DIR/build/px4_sitl_nolockstep/etc/init.d-posix/airframes"
for a in $AIRFRAMES; do
	if [ -f "$BUILD_AF/$a" ]; then
		run rm -f "$BUILD_AF/$a"
		[ "$DRY_RUN" -eq 1 ] || record "deleted  build/px4_sitl_nolockstep/etc/init.d-posix/airframes/$a"
	fi
done
BRIDGE_BUILD="$PX4_DIR/build/px4_sitl_nolockstep/build_flightaxis_bridge"
if [ -d "$BRIDGE_BUILD" ]; then
	run rm -rf "$BRIDGE_BUILD"
	ok "build/px4_sitl_nolockstep/build_flightaxis_bridge/"
	[ "$DRY_RUN" -eq 1 ] || record "deleted  build/px4_sitl_nolockstep/build_flightaxis_bridge/"
fi

# ----------------------------------------------------------------- verify ----
step "3/3  Verifying removal"

LEFT=0
check_gone() {
	if [ -e "$1" ]; then
		printf '    %sFAIL%s still present: %s\n' "$C_RED" "$C_OFF" "${1#"$PX4_DIR"/}" >&2
		LEFT=1
	fi
}
if [ "$DRY_RUN" -eq 0 ]; then
	# Only the manifest is checked. The payload *directory* is deliberately not
	# checked: it legitimately survives when it still holds files the user
	# added, and those were listed in step 2.
	while IFS= read -r rel; do
		[ -n "$rel" ] || continue
		check_gone "$PX4_DIR/$rel"
	done <<EOF
$(fa_payload_files)
EOF
	# The include must be gone as a whole line - the same trimmed match
	# fa_strip_include uses. A substring grep here fired on an indented include
	# that the stripper had (correctly) already removed in an older version, and
	# on a mention inside a comment.
	if fa_has_include "$SIM_CMAKE" "$INCLUDE_LINE" 2>/dev/null; then
		printf '    %sFAIL%s %s still includes %s\n' "$C_RED" "$C_OFF" "$SIM_CMAKE_REL" "$INCLUDE_LINE" >&2
		LEFT=1
	fi
	# FA_ENTRY_RE, not an id range: a user's own 1204_flightaxis_cessna entry is
	# not ours, is intentionally left registered, and must not fail this check.
	if grep -qE "$FA_ENTRY_RE" "$AF_CMAKE" 2>/dev/null; then
		printf '    %sFAIL%s %s still lists our flightaxis airframes\n' "$C_RED" "$C_OFF" "$AF_CMAKE_REL" >&2
		LEFT=1
	fi
	if grep -qE "$FA_HIL_ENTRY_RE" "$HIL_AF_CMAKE" 2>/dev/null; then
		printf '    %sFAIL%s %s still lists our flightaxis HITL airframes\n' "$C_RED" "$C_OFF" "$HIL_AF_CMAKE_REL" >&2
		LEFT=1
	fi
	[ "$LEFT" -eq 0 ] || die "uninstall did not fully complete (see FAIL lines above)."
	ok "no flightaxis files or registrations of ours remain"
else
	info "dry-run: nothing was checked"
fi

printf '\n%s%s' "$C_BLD" "$C_GRN"
if [ "$DRY_RUN" -eq 1 ]; then
	printf 'Dry run complete - nothing was modified.'
else
	printf 'FlightAxis integration removed.'
fi
printf '%s\n\n' "$C_OFF"

if [ "$DRY_RUN" -eq 0 ]; then
	if [ -n "$REMOVED" ]; then
		printf '%sChanges made under %s:%s\n' "$C_BLD" "$PX4_DIR" "$C_OFF"
		printf '%s\n' "$REMOVED"
	else
		printf '    Nothing to remove - the integration was not installed here.\n\n'
	fi
	printf '    Re-run cmake/make in the PX4 tree to drop the flightaxis make targets:\n'
	printf '      make -C %s px4_sitl_nolockstep\n\n' "$PX4_DIR"
fi
