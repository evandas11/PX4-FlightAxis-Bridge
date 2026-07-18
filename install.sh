#!/usr/bin/env bash
#
# install.sh — install the PX4-FlightAxis-Bridge integration into a PX4-Autopilot checkout.
#
#   ./install.sh [PATH_TO_PX4] [--no-build] [--force] [--dry-run]
#
# Copies the flightaxis payload into the PX4 tree, splices two one-line
# registrations into PX4-owned CMakeLists.txt files (idempotently), builds
# px4_sitl_nolockstep + the bridge, and verifies the result.
#
# Never uses sudo. Never overwrites unrelated PX4 files. Backs up the two
# PX4-owned CMakeLists.txt files to <file>.flightaxis.bak before touching them.
#
set -euo pipefail

# pwd -P, to match px4_resolve()'s canonicalisation. With the logical pwd a
# symlinked checkout gives REPO_DIR and PX4_RESOLVED two different strings for
# the same directory, and the "target cannot be this repository" guards below
# compare unlike things and pass when they should fire.
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

# Actions actually performed on disk, for the final summary.
CHANGES=""
record() { CHANGES="${CHANGES}    $1"$'\n'; }

# Pre-image of the SIM CMakeLists, set only for the window between the 5a
# splice landing and the 5b splice completing. Step 5 registers two files and
# they are only meaningful together: an include(sitl_targets_flightaxis.cmake)
# whose airframes were never registered gives a confusing cmake error much
# later. If we die inside that window we put 5a back.
ROLLBACK_SIM=""
# The 5a summary line, held back until 5b confirms (see above).
SIM_RECORD=""

_rollback() {
	[ -n "$ROLLBACK_SIM" ] || return 0
	[ -f "$ROLLBACK_SIM" ] || { ROLLBACK_SIM=""; return 0; }
	if mv -f "$ROLLBACK_SIM" "$SIM_CMAKE" 2>/dev/null; then
		printf '       rolled back the %s edit (the two registrations only work as a pair).\n' \
			"$SIM_CMAKE_REL" >&2
	else
		printf '       WARNING: could not roll back %s - remove this line by hand:\n         %s\n' \
			"$SIM_CMAKE_REL" "$INCLUDE_LINE" >&2
	fi
	ROLLBACK_SIM=""
}

# Never exit on an error without telling the user what is already on disk and
# how to undo it: a half-installed tree that reports nothing is the worst case.
die() {
	printf '%serror:%s %s\n' "$C_RED" "$C_OFF" "$*" >&2
	_rollback
	if [ -n "$CHANGES" ] && [ "${DRY_RUN:-0}" -eq 0 ]; then
		printf '\n%sChanges already made under %s:%s\n' "$C_BLD" "${PX4_DIR:-?}" "$C_OFF" >&2
		printf '%s' "$CHANGES" >&2
		printf '\n    Undo them with:  %s/uninstall.sh %s\n\n' "$REPO_DIR" "${PX4_DIR:-}" >&2
	fi
	exit 1
}

# ------------------------------------------------------------------ args ----
usage() {
	cat <<'EOF'
Usage: ./install.sh [PATH_TO_PX4] [options]

Installs the FlightAxis (RealFlight) SITL integration into a PX4-Autopilot
checkout: copies the payload, registers it in PX4's two CMakeLists.txt files,
builds it, and verifies the result.

Target PX4 tree resolution order (first hit wins, and the choice is reported):
  1. PATH_TO_PX4 positional argument
  2. $PX4_DIR environment variable
  3. auto-detection:
       a. an enclosing PX4 tree above the current directory or above this repo
       b. next to this repo: ../PX4-Autopilot, ../../PX4-Autopilot,
          ../px4/PX4-Autopilot, the parent
       c. conventional spots under $HOME (PX4-Autopilot, src/, Code/, dev/,
          workspace/, git/, projects/, ...)
       d. a bounded, pruned, time-limited search under $HOME (max depth 4)
  If several checkouts are found, they are all listed and nothing is installed:
  disambiguate with an argument or $PX4_DIR.

Options:
  --no-build    Install and register files only; skip the build step.
  --force       Proceed despite an unrecognised PX4 layout, an unexpected PX4
                version, or uncommitted local modifications to the files this
                installer writes. Use with care.
  --dry-run     Print every action that would be taken; change nothing.
  --yes, -y     Do not ask for confirmation (for scripts and CI).
  -h, --help    Show this help.

Examples:
  ./install.sh
  ./install.sh ~/src/PX4-Autopilot --no-build
  PX4_DIR=~/src/PX4-Autopilot ./install.sh --dry-run
EOF
}

PX4_ARG=""
NO_BUILD=0
FORCE=0
DRY_RUN=0
ASSUME_YES=0

while [ $# -gt 0 ]; do
	case "$1" in
		--no-build) NO_BUILD=1 ;;
		--force)    FORCE=1 ;;
		--dry-run)  DRY_RUN=1 ;;
		-y|--yes)   ASSUME_YES=1 ;;
		-h|--help)  usage; exit 0 ;;
		-*)         die "unknown option: $1 (try --help)" ;;
		*)
			[ -z "$PX4_ARG" ] || die "unexpected extra argument: $1 (try --help)"
			PX4_ARG="$1"
			;;
	esac
	shift
done

# ------------------------------------------------- target tree resolution ----
# shellcheck source=scripts/detect-px4.sh
. "$REPO_DIR/scripts/detect-px4.sh"

printf '\n%s%sPX4-FlightAxis-Bridge installer%s\n' "$C_BLD" "$C_BLU" "$C_OFF"
info "repo:   $REPO_DIR"

px4_resolve "$PX4_ARG" || exit 1
PX4_DIR="$PX4_RESOLVED"

[ "$PX4_DIR" != "$REPO_DIR" ] || die "the target PX4 tree cannot be this repository"

PX4_VERSION="$(px4_version "$PX4_DIR")"
info "target: $PX4_DIR"
info "        version: $PX4_VERSION"
info "        selected via: $PX4_RESOLVED_HOW"

# An unattended run must never install into a tree we merely guessed at.
if [ "$PX4_RESOLVED_EXPLICIT" -eq 0 ] && [ ! -t 0 ] && [ "$DRY_RUN" -eq 0 ]; then
	die "refusing to install into an auto-detected PX4 tree in a non-interactive run.
       Pass the path explicitly or set \$PX4_DIR:
         ./install.sh $PX4_DIR"
fi

# Confirm before writing, when there is a human to ask.
if [ "$DRY_RUN" -eq 0 ] && [ "$ASSUME_YES" -eq 0 ] && [ -t 0 ]; then
	printf '\n'
	printf 'Install the FlightAxis integration into the tree above? [y/N] '
	read -r _reply || _reply=""
	case "$_reply" in
		y|Y|yes|YES|Yes) : ;;
		*) printf 'Aborted; nothing was changed.\n'; exit 1 ;;
	esac
fi

# --------------------------------------------------------------- layout ----
SIM_CMAKE_REL="src/modules/simulation/simulator_mavlink/CMakeLists.txt"
AF_CMAKE_REL="ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
SIM_CMAKE="$PX4_DIR/$SIM_CMAKE_REL"
AF_CMAKE="$PX4_DIR/$AF_CMAKE_REL"
AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
BUILD_DIR="$PX4_DIR/build/px4_sitl_nolockstep"
BRIDGE_BIN="$BUILD_DIR/build_flightaxis_bridge/flightaxis_bridge"

# Both lists come from scripts/detect-px4.sh, the single source of truth for
# what this integration owns (see the "what we own" block there).
AIRFRAMES="$FA_AIRFRAMES"
MODELS="$FA_MODELS"

INCLUDE_LINE='include(sitl_targets_flightaxis.cmake)'
ANCHOR_LINE='include(sitl_targets_flightgear.cmake)'

run() {
	if [ "$DRY_RUN" -eq 1 ]; then
		printf '    %s[dry-run]%s %s\n' "$C_YEL" "$C_OFF" "$*"
	else
		"$@"
	fi
}

if [ "$DRY_RUN" -eq 1 ]; then
	printf '    %sdry-run: nothing will be modified%s\n' "$C_YEL" "$C_OFF"
fi
printf '\n'

# ================================================================= 1/7 ======
step "1/7  Validating the target PX4 tree"

px4_is_checkout "$PX4_DIR" \
	|| die "$PX4_DIR does not look like a PX4-Autopilot checkout (missing one of:
       Makefile, Tools/simulation/, $SIM_CMAKE_REL, $AF_CMAKE_REL)"
ok "PX4 layout looks right (Makefile, Tools/simulation, both CMakeLists.txt)"

# The v1.16 per-simulator sitl_targets_*.cmake pattern is what we splice into.
SIM_DIR="$PX4_DIR/src/modules/simulation/simulator_mavlink"
if [ ! -f "$SIM_DIR/sitl_targets_flightgear.cmake" ]; then
	MSG="$SIM_CMAKE_REL exists but sitl_targets_flightgear.cmake does not.
       This tree does not use the per-simulator sitl_targets_*.cmake pattern
       that this integration relies on. PX4 v1.13-v1.15 register SITL targets in
       platforms/posix/cmake/sitl_target.cmake instead, which needs a different
       (manual) integration - see README.md and the spec, section 3."
	if [ "$FORCE" -eq 1 ]; then
		warn "$MSG"
		warn "--force given: continuing anyway; the splice will very likely fail."
	else
		die "$MSG"
	fi
fi

if [ "$PX4_VERSION" = "unknown" ]; then
	warn "could not determine the PX4 version (no git tags, no version.txt)."
	warn "This integration is developed and tested against PX4 v1.16.x."
else
	info "detected PX4 version: $PX4_VERSION"
	case "$PX4_VERSION" in
		v1.16*|1.16*) ok "v1.16-era tree - this is the tested configuration" ;;
		*)
			warn "PX4 $PX4_VERSION is not a v1.16 release."
			warn "This integration is tested against v1.16.0 only. On v1.13-v1.15 the"
			warn "sitl_targets_*.cmake pattern does not exist (SITL targets live in"
			warn "platforms/posix/cmake/sitl_target.cmake) and this installer's splices"
			warn "will not produce a working build."
			if [ "$FORCE" -eq 1 ] || [ "$DRY_RUN" -eq 1 ]; then
				warn "continuing (--force/--dry-run)."
			else
				die "refusing to install on an untested PX4 version; re-run with --force to override."
			fi
			;;
	esac
fi

# ================================================================= 2/7 ======
step "2/7  Checking for conflicts"

# SYS_AUTOSTART id collisions.
#
# We install ids 1200-1203, so only a *different* airframe occupying one of
# those four ids is a real collision. The check used to span the whole reserved
# 1200-1219 range, which put it in direct conflict with README's "Adding a new
# aircraft": a user's own 1204_flightaxis_cessna made the installer refuse
# outright, and that refusal honoured neither --force nor --dry-run. Ids
# 1204-1219 stay reserved by documentation, but they are the user's to use.
COLLISIONS=""
for f in "$AF_DIR"/120[0-3]_*; do
	[ -e "$f" ] || continue
	base="$(basename "$f")"
	case " $AIRFRAMES " in
		*" $base "*) : ;;
		*) COLLISIONS="$COLLISIONS $base" ;;
	esac
done
if [ -n "$COLLISIONS" ]; then
	MSG="SYS_AUTOSTART id collision in $AF_DIR
$(for c in $COLLISIONS; do printf '       %s occupies one of the ids 1200-1203 that this integration installs\n' "$c"; done)
       Installing would give two airframes the same SYS_AUTOSTART id. Move or
       rename them, or renumber this integration (airframe files + README)."
	if [ "$FORCE" -eq 1 ] || [ "$DRY_RUN" -eq 1 ]; then
		warn "$MSG"
		warn "continuing (--force/--dry-run)."
	else
		die "$MSG
       Re-run with --force to install anyway."
	fi
else
	ok "no SYS_AUTOSTART collisions on the ids 1200-1203 we install"
fi

# Uncommitted local modifications to files we are about to overwrite.
if px4_is_git "$PX4_DIR"; then
	TRACKED="$SIM_CMAKE_REL $AF_CMAKE_REL Tools/simulation/flightaxis src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake"
	for a in $AIRFRAMES; do
		TRACKED="$TRACKED ROMFS/px4fmu_common/init.d-posix/airframes/$a"
	done
	# shellcheck disable=SC2086
	STATUS="$(git -C "$PX4_DIR" status --short -- $TRACKED 2>/dev/null || true)"
	if [ -n "$STATUS" ]; then
		info "git status for the paths this installer touches:"
		printf '%s\n' "$STATUS" | sed 's/^/      /'
		# Modifications to PX4-owned files are the dangerous ones; our own payload
		# showing up as untracked (??) is normal and expected on a re-install.
		DIRTY="$(printf '%s\n' "$STATUS" | grep -v '^??' | grep -E "($SIM_CMAKE_REL|$AF_CMAKE_REL)" || true)"
		if [ -n "$DIRTY" ]; then
			if [ "$FORCE" -eq 1 ] || [ "$DRY_RUN" -eq 1 ]; then
				warn "PX4-owned CMakeLists.txt files have uncommitted local changes; continuing (--force/--dry-run)."
			else
				printf '%serror:%s the PX4-owned CMakeLists.txt files this installer edits have\n' "$C_RED" "$C_OFF" >&2
				printf '       uncommitted local modifications:\n' >&2
				printf '%s\n' "$DIRTY" | sed 's/^/         /' >&2
				printf '       Commit or stash them first, or re-run with --force.\n' >&2
				printf '       (A previous run of this installer also shows up here - that is\n' >&2
				printf '        harmless, the splices below are idempotent; use --force.)\n' >&2
				exit 1
			fi
		else
			ok "no conflicting uncommitted changes to PX4-owned files"
		fi
	else
		ok "target git tree is clean for the paths we touch"
	fi
else
	info "target is not a git checkout; skipping the local-modification check"
fi

# ================================================================= 3/7 ======
step "3/7  Backing up PX4-owned CMakeLists.txt files"

# backup <file> <stripper-command...>
#
# The backup must always hold the PX4-owned file as it is *right now* minus our
# own lines. Blindly keeping a pre-existing backup was a data-loss bug: install
# -> git pull (upstream updates the CMakeLists) -> re-install kept the stale
# pre-pull backup, and the next uninstall "restored" it, silently reverting the
# upstream update. So we compare the backup against the current file with our
# lines stripped out, and refresh it when they disagree.
backup() {
	src="$1"; shift
	bak="$src.flightaxis.bak"
	rel="${src#"$PX4_DIR"/}"

	if [ ! -f "$bak" ]; then
		run cp -p "$src" "$bak"
		ok "backed up $rel -> ${bak#"$PX4_DIR"/}"
		[ "$DRY_RUN" -eq 1 ] || record "created  ${bak#"$PX4_DIR"/}"
		return 0
	fi

	# What the backup *should* contain: the current file without our lines.
	pristine="$src.flightaxis.pristine.$$"
	"$@" "$src" > "$pristine" 2>/dev/null || { rm -f "$pristine"; }
	if [ -f "$pristine" ] && cmp -s "$pristine" "$bak"; then
		rm -f "$pristine"
		info "backup already exists and still matches: ${bak#"$PX4_DIR"/}"
		return 0
	fi

	warn "$rel has changed upstream since the existing backup was taken
         (a git pull, or a manual edit). Refreshing the backup - keeping the
         old one would make uninstall.sh revert that change."
	if [ "$DRY_RUN" -eq 1 ]; then
		rm -f "$pristine"
		printf '    %s[dry-run]%s refresh %s\n' "$C_YEL" "$C_OFF" "${bak#"$PX4_DIR"/}"
		return 0
	fi
	if [ -f "$pristine" ]; then
		cat "$pristine" > "$bak"
		rm -f "$pristine"
	else
		cp -p "$src" "$bak"
	fi
	ok "refreshed backup ${bak#"$PX4_DIR"/}"
	record "refreshed ${bak#"$PX4_DIR"/}"
}
# fa_strip_include takes the line to remove as its second argument; the backup
# helper passes only the file, so bind INCLUDE_LINE here.
fa_strip_include_of() { fa_strip_include "$1" "$INCLUDE_LINE"; }

backup "$SIM_CMAKE" fa_strip_include_of
backup "$AF_CMAKE"  fa_strip_airframes

# ================================================================= 4/7 ======
step "4/7  Copying the FlightAxis payload"

# Deliberately NO --delete here, and none in sync-to-px4.sh either.
#
# The payload directory is where README's "Adding a new aircraft" tells users to
# put models/<name>.json, so rsync'ing --delete into it would destroy their work
# on every upgrade. The cost of that choice is that a file we drop in a later
# version lingers in the tree; uninstall.sh removes exactly our manifest, so the
# clean way to shed a stale file is uninstall + install. Writing INTO the PX4
# tree never deletes; only sync-from-px4.sh (repo is the mirror) uses --delete.
copy_tree() {
	# copy_tree <src-dir>/ <dst-dir>/ ; preserves modes, excludes __pycache__
	if command -v rsync >/dev/null 2>&1; then
		run rsync -a --exclude __pycache__ --exclude '*.pyc' "$1" "$2"
	else
		run mkdir -p "$2"
		if [ "$DRY_RUN" -eq 1 ]; then
			printf '    %s[dry-run]%s cp -R %s. %s\n' "$C_YEL" "$C_OFF" "$1" "$2"
		else
			(cd "$1" && find . -name __pycache__ -prune -o -print0 \
				| cpio -0pdmu --quiet "$2" 2>/dev/null) \
			|| { cp -R "$1". "$2"; find "$2" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true; }
		fi
	fi
}

# A copy that fails midway used to abort on the raw rsync/cp error under
# `set -e`, saying nothing about the partial payload now sitting in the tree and
# nothing about uninstall.sh. Route every copy through die() instead, which
# prints the change list and the undo command.
copy_tree "$REPO_DIR/Tools/simulation/flightaxis/" "$PX4_DIR/Tools/simulation/flightaxis/" \
	|| die "failed to copy Tools/simulation/flightaxis/ into the PX4 tree.
       The tree may now hold a partially copied payload."
if [ "$DRY_RUN" -eq 0 ]; then
	ok "Tools/simulation/flightaxis/"
	record "copied   Tools/simulation/flightaxis/  (payload manifest)"
fi

run cp -p "$REPO_DIR/src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake" "$SIM_DIR/" \
	|| die "failed to copy sitl_targets_flightaxis.cmake into $SIM_CMAKE_REL's directory."
if [ "$DRY_RUN" -eq 0 ]; then
	ok "src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake"
	record "copied   src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake"
fi

for a in $AIRFRAMES; do
	run cp -p "$REPO_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/$a" "$AF_DIR/" \
		|| die "failed to copy the airframe $a into the PX4 tree."
	if [ "$DRY_RUN" -eq 0 ]; then
		ok "ROMFS/px4fmu_common/init.d-posix/airframes/$a"
		record "copied   ROMFS/px4fmu_common/init.d-posix/airframes/$a"
	fi
done

if [ "$DRY_RUN" -eq 0 ]; then
	# The manifest is shared with uninstall.sh (fa_payload_files, in
	# scripts/detect-px4.sh) so what install verifies is exactly what uninstall
	# removes - they cannot drift.
	EXPECTED="$(fa_payload_files)"
	# Collected in a variable rather than through a predictable path in the
	# world-writable /tmp (/tmp/.flightaxis_missing.$$ was a symlink-attack
	# target and needed no file at all).
	NOT_LANDED="$(
		printf '%s\n' "$EXPECTED" | while IFS= read -r rel; do
			[ -n "$rel" ] || continue
			[ -f "$PX4_DIR/$rel" ] || printf '%s\n' "$rel"
		done
	)"
	if [ -n "$NOT_LANDED" ]; then
		die "these files did not land in the PX4 tree:
$(printf '%s\n' "$NOT_LANDED" | sed 's/^/         /')"
	fi
	# sitl_run.sh must stay executable or the make target dies at launch.
	[ -x "$PX4_DIR/Tools/simulation/flightaxis/sitl_run.sh" ] \
		|| die "Tools/simulation/flightaxis/sitl_run.sh lost its executable bit"
	ok "all expected payload files verified present (sitl_run.sh is executable)"
fi

# ================================================================= 5/7 ======
step "5/7  Registering with the PX4 build system"

# --- 5a: include(sitl_targets_flightaxis.cmake) -----------------------------
# fa_has_include is the exact counterpart of the fa_strip_include that
# uninstall.sh uses: same trimmed whole-line comparison, so an indented include
# (legal CMake, and what README Method 2 invites) is detected here AND removed
# there. An unanchored grep -qF here also matched a commented-out include and
# reported "already includes", producing a build with no flightaxis targets.
if fa_has_include "$SIM_CMAKE" "$INCLUDE_LINE"; then
	ok "$SIM_CMAKE_REL already includes sitl_targets_flightaxis.cmake (no change)"
else
	fa_has_include "$SIM_CMAKE" "$ANCHOR_LINE" || die \
"could not find the anchor line
         $ANCHOR_LINE
       in $SIM_CMAKE_REL
       Refusing to guess where to insert our include(). Add this line manually
       next to the other include(sitl_targets_*.cmake) lines:
         $INCLUDE_LINE"

	# Insert immediately before the flightgear include (keeps the list
	# alphabetically sorted, which is how PX4 maintains it). The anchor is
	# matched trimmed - the same test fa_has_include just passed, so we can
	# never "find" an anchor here that the awk then fails to match - and our
	# line copies the anchor's own indentation and line ending.
	TMP="$SIM_CMAKE.flightaxis.tmp.$$"
	awk -v ins="$INCLUDE_LINE" -v anchor="$ANCHOR_LINE" '
		{
			line = $0; cr = ""
			if (line ~ /\r$/) { cr = "\r"; sub(/\r$/, "", line) }
			if (!done) {
				t = line
				sub(/^[[:space:]]+/, "", t); sub(/[[:space:]]+$/, "", t)
				if (t == anchor) {
					indent = line
					sub(/[^ \t].*$/, "", indent)
					print indent ins cr
					done = 1
				}
			}
			print line cr
		}
	' "$SIM_CMAKE" > "$TMP"
	fa_has_include "$TMP" "$INCLUDE_LINE" \
		|| { rm -f "$TMP"; die "splice into $SIM_CMAKE_REL produced no change"; }

	if [ "$DRY_RUN" -eq 1 ]; then
		printf '    %s[dry-run]%s would insert "%s" before "%s" in %s:\n' \
			"$C_YEL" "$C_OFF" "$INCLUDE_LINE" "$ANCHOR_LINE" "$SIM_CMAKE_REL"
		diff -u "$SIM_CMAKE" "$TMP" | sed 's/^/      /' || true
		rm -f "$TMP"
	else
		# Keep a pre-image until 5b has also succeeded (see ROLLBACK_SIM): an
		# include() pointing at a cmake file whose airframes were never
		# registered is worse than no include() at all.
		ROLLBACK_SIM="$SIM_CMAKE.flightaxis.pre.$$"
		cp -p "$SIM_CMAKE" "$ROLLBACK_SIM"
		mv "$TMP" "$SIM_CMAKE"
		ok "added $INCLUDE_LINE to $SIM_CMAKE_REL"
		SIM_RECORD="edited   $SIM_CMAKE_REL  (+1 line: $INCLUDE_LINE)"
	fi
fi

# --- 5b: the four airframe entries -----------------------------------------
# fa_splice_airframes (scripts/detect-px4.sh) rewrites the px4_add_romfs_files()
# list: it drops any existing flightaxis entry in our reserved 1200-1219 range,
# then re-inserts $AIRFRAMES as one block in sorted position. It lives next to
# the matching removal filter so install and uninstall cannot drift apart - that
# drift is what let a CRLF tree end up with duplicated entries.
MISSING_AF=""
for a in $AIRFRAMES; do
	grep -qE "^[[:space:]]*${a}[[:space:]]*$" "$AF_CMAKE" || MISSING_AF="$MISSING_AF $a"
done

if [ -z "$MISSING_AF" ]; then
	ok "$AF_CMAKE_REL already lists all four flightaxis airframes (no change)"
else
	grep -q '^px4_add_romfs_files(' "$AF_CMAKE" || die \
"could not find the 'px4_add_romfs_files(' list in
         $AF_CMAKE_REL
       Refusing to guess where to register the airframes. Add these manually,
       in sorted position inside that list:
$(for a in $AIRFRAMES; do printf '         %s\n' "$a"; done)"

	TMP="$AF_CMAKE.flightaxis.tmp.$$"
	if ! fa_splice_airframes "$AF_CMAKE" "$AIRFRAMES" > "$TMP"; then
		rm -f "$TMP"
		die "could not find a sorted insertion point (no airframe id above 1219) inside the
       px4_add_romfs_files() list in
       $AF_CMAKE_REL
       Refusing to blind-append. Register the four 120x_flightaxis_* entries manually."
	fi
	for a in $AIRFRAMES; do
		grep -qE "^[[:space:]]*${a}[[:space:]]*$" "$TMP" \
			|| { rm -f "$TMP"; die "splice into $AF_CMAKE_REL did not register $a"; }
	done
	# "is each name present?" is not enough on its own: the CRLF duplication bug
	# re-emitted every entry and still passed that check. Count them, and require
	# every one to sit inside the px4_add_romfs_files() list.
	# Counted with FA_ENTRY_RE (the exact four names), not an id range: a
	# user-authored 1204_flightaxis_cessna is not ours and must not be counted,
	# or this check fails on a tree that is perfectly correct.
	N_ENTRIES="$(grep -cE "$FA_ENTRY_RE" "$TMP" || true)"
	N_EXPECT="$(printf '%s\n' $AIRFRAMES | grep -c . || true)"
	[ "$N_ENTRIES" -eq "$N_EXPECT" ] || { rm -f "$TMP"; die \
		"splice into $AF_CMAKE_REL produced $N_ENTRIES flightaxis entries, expected $N_EXPECT
       (duplicate or stray entries - refusing to write the file)"; }
	N_INSIDE="$(awk -v entry_re="$FA_ENTRY_RE" '
		/^px4_add_romfs_files\(/ { inlist = 1; next }
		inlist && /^[ \t]*\)/    { inlist = 0; next }
		inlist {
			line = $0; sub(/\r$/, "", line)
			if (line ~ entry_re) { n++ }
		}
		END { print n + 0 }
	' "$TMP")"
	[ "$N_INSIDE" -eq "$N_EXPECT" ] || { rm -f "$TMP"; die \
		"splice into $AF_CMAKE_REL put $N_INSIDE of $N_EXPECT entries inside the
       px4_add_romfs_files() list; the rest would land at top level and break cmake."; }

	if [ "$DRY_RUN" -eq 1 ]; then
		printf '    %s[dry-run]%s would register the airframes in %s:\n' \
			"$C_YEL" "$C_OFF" "$AF_CMAKE_REL"
		diff -u "$AF_CMAKE" "$TMP" | sed 's/^/      /' || true
		rm -f "$TMP"
	else
		mv "$TMP" "$AF_CMAKE"
		ok "registered$MISSING_AF in $AF_CMAKE_REL"
		record "edited   $AF_CMAKE_REL  (+ airframe entries:$MISSING_AF)"
	fi
fi

# Both halves of step 5 are in place, so the 5a edit is committed: drop the
# pre-image and start reporting it as a change. A later failure (the build,
# verification) must NOT undo the registration - it is correct, and the error
# message tells the user to fix the build and re-run.
if [ -n "$ROLLBACK_SIM" ]; then
	rm -f "$ROLLBACK_SIM"
	ROLLBACK_SIM=""
fi
[ -z "${SIM_RECORD:-}" ] || { record "$SIM_RECORD"; SIM_RECORD=""; }

# ================================================================= 6/7 ======
step "6/7  Building"

if [ "$NO_BUILD" -eq 1 ]; then
	info "--no-build given: skipping the build."
elif [ "$DRY_RUN" -eq 1 ]; then
	printf '    %s[dry-run]%s make -C %s px4_sitl_nolockstep\n' "$C_YEL" "$C_OFF" "$PX4_DIR"
	printf '    %s[dry-run]%s build target flightaxis_bridge\n' "$C_YEL" "$C_OFF"
else
	warn "a fresh PX4 tree takes 10-30 minutes to build. Output follows."
	printf '\n'
	info "\$ make -C $PX4_DIR px4_sitl_nolockstep"
	make -C "$PX4_DIR" px4_sitl_nolockstep \
		|| die "'make px4_sitl_nolockstep' failed (see the output above).
       The files are installed; fix the build error and re-run './install.sh $PX4_DIR'."
	record "built    build/px4_sitl_nolockstep/  (px4 + ROMFS)"

	info "\$ build flightaxis_bridge"
	if command -v ninja >/dev/null 2>&1 && [ -f "$BUILD_DIR/build.ninja" ]; then
		ninja -C "$BUILD_DIR" flightaxis_bridge \
			|| die "building the flightaxis_bridge target failed (see the output above)."
	else
		cmake --build "$BUILD_DIR" --target flightaxis_bridge \
			|| die "building the flightaxis_bridge target failed (see the output above)."
	fi
	record "built    build/px4_sitl_nolockstep/build_flightaxis_bridge/flightaxis_bridge"

	[ -x "$BRIDGE_BIN" ] || die "the build reported success but the bridge binary is missing:
       $BRIDGE_BIN"
	ok "bridge binary: ${BRIDGE_BIN#"$PX4_DIR"/}"
fi

# ================================================================= 7/7 ======
step "7/7  Verifying the installation"

# The whole failure path here used to be dead code. Under `set -euo pipefail` a
# bare `cmd` / `[ ... ]` on the line before `vcheck $?` exits the shell the
# moment it fails, so vcheck never ran: no FAIL line, no summary, no die()
# message - the installer just stopped with status 1 straight after the last
# `ok`, immediately after a 30-minute build, in the one code path whose entire
# job is to explain that the files installed but the build output is wrong.
# Every check below therefore runs as `if cmd; then vcheck 0 ... else vcheck 1
# ... fi`, so the failure is reported rather than fatal.
VERIFY_FAIL=0
vcheck() {
	if [ "$1" -eq 0 ]; then ok "$2"; else
		printf '    %sFAIL%s %s\n' "$C_RED" "$C_OFF" "$2" >&2
		VERIFY_FAIL=1
	fi
}

# Model JSONs parse.
if command -v python3 >/dev/null 2>&1; then
	MODELS_DIR="$PX4_DIR/Tools/simulation/flightaxis/flightaxis_bridge/models"
	if [ "$DRY_RUN" -eq 1 ]; then
		MODELS_DIR="$REPO_DIR/Tools/simulation/flightaxis/flightaxis_bridge/models"
	fi
	for m in $MODELS; do
		if python3 -c "import json,sys; json.load(open(sys.argv[1]))" "$MODELS_DIR/$m.json" >/dev/null 2>&1; then
			vcheck 0 "models/$m.json parses as JSON"
		else
			vcheck 1 "models/$m.json parses as JSON"
		fi
	done

	# get_FAbridge_params.py emits argv for each model (it resolves the JSON
	# relative to the CWD, hence the subshell cd).
	GETP_DIR="$(dirname "$MODELS_DIR")"
	for m in $MODELS; do
		OUT="$( (cd "$GETP_DIR" && python3 ./get_FAbridge_params.py "models/$m.json") 2>/dev/null || true)"
		if [ -n "$OUT" ]; then
			ok "get_FAbridge_params.py $m -> $(printf '%s' "$OUT" | cut -c1-48)..."
		else
			printf '    %sFAIL%s get_FAbridge_params.py produced no argv for %s\n' "$C_RED" "$C_OFF" "$m" >&2
			VERIFY_FAIL=1
		fi
	done
else
	warn "python3 not found; skipping the model JSON / get_FAbridge_params.py checks"
fi

if [ "$DRY_RUN" -eq 1 ]; then
	info "dry-run: skipping build-output checks"
elif [ "$NO_BUILD" -eq 1 ]; then
	info "--no-build: skipping make-target and ROMFS checks"
else
	if command -v ninja >/dev/null 2>&1 && [ -f "$BUILD_DIR/build.ninja" ]; then
		TARGETS="$(ninja -C "$BUILD_DIR" -t targets all 2>/dev/null | cut -d: -f1 || true)"
		for m in $MODELS; do
			if printf '%s\n' "$TARGETS" | grep -qx "flightaxis_$m"; then
				vcheck 0 "make target flightaxis_$m exists"
			else
				vcheck 1 "make target flightaxis_$m exists"
			fi
		done
	else
		info "ninja not available; skipping the make-target check"
	fi
	for a in $AIRFRAMES; do
		if [ -f "$BUILD_DIR/etc/init.d-posix/airframes/$a" ]; then
			vcheck 0 "airframe installed: build/px4_sitl_nolockstep/etc/init.d-posix/airframes/$a"
		else
			vcheck 1 "airframe installed: build/px4_sitl_nolockstep/etc/init.d-posix/airframes/$a"
		fi
	done
	if [ -x "$BRIDGE_BIN" ]; then
		vcheck 0 "bridge binary present and executable"
	else
		vcheck 1 "bridge binary present and executable"
	fi
fi

[ "$VERIFY_FAIL" -eq 0 ] || die "post-install verification failed (see FAIL lines above)."

# -------------------------------------------------------------- summary ----
printf '\n%s%s' "$C_BLD" "$C_GRN"
if [ "$DRY_RUN" -eq 1 ]; then
	printf 'Dry run complete - nothing was modified.'
else
	printf 'FlightAxis integration installed successfully.'
fi
printf '%s\n\n' "$C_OFF"

if [ "$DRY_RUN" -eq 0 ]; then
	printf '%sChanges made under %s:%s\n' "$C_BLD" "$PX4_DIR" "$C_OFF"
	printf '%s' "$CHANGES"
	printf '\n    Revert everything with:  %s/uninstall.sh %s\n\n' "$REPO_DIR" "$PX4_DIR"
fi

printf '%sWhat to do next%s\n' "$C_BLD" "$C_OFF"
printf '  1. On the Windows machine running RealFlight, enable %sFlightAxis Link%s\n' "$C_BLD" "$C_OFF"
printf '     (Settings -> Physics -> Quality -> enable "RealFlight Link"; on RealFlight 8/9\n'
printf '     it is Settings -> Physics. It listens on TCP 18083.)\n'
printf '     Use a wired network - WiFi cannot hold the ~250 Hz SOAP rate.\n'
printf '  2. Launch SITL from %s:\n' "$PX4_DIR"
printf '\n       %sPX4_FLIGHTAXIS_IP=<windows-ip> make px4_sitl_nolockstep flightaxis_plane%s\n\n' "$C_BLD" "$C_OFF"
printf '     (also: flightaxis_quad, flightaxis_quadplane, flightaxis_heli)\n'
printf '  3. QGroundControl connects on UDP 14550 as usual.\n\n'
printf '  Aircraft setup and channel maps: %s/README.md\n\n' "$REPO_DIR"
