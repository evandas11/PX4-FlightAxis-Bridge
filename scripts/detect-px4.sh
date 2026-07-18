#!/usr/bin/env bash
#
# detect-px4.sh — shared PX4-Autopilot checkout detection.
#
# Sourced by install.sh, uninstall.sh, scripts/sync-to-px4.sh and
# scripts/sync-from-px4.sh so all four agree on which tree they target.
# Nothing here is hardcoded to any particular machine or user: every path is
# derived from the caller's $HOME, $PWD, or the location of this repository.
#
# Public API:
#   px4_is_checkout DIR      -> 0 if DIR structurally looks like PX4-Autopilot
#   px4_version DIR          -> prints a version string (or "unknown")
#   px4_abspath DIR          -> prints the canonical absolute path
#   px4_resolve [EXPLICIT]   -> resolves the target tree; on success sets
#                               PX4_RESOLVED       (absolute path)
#                               PX4_RESOLVED_HOW   (human-readable reason)
#                               PX4_RESOLVED_EXPLICIT (1 if from arg/$PX4_DIR)
#                               and returns 0. On failure prints an actionable
#                               message to stderr and returns 1.
#
# Resolution order (first hit wins):
#   1. explicit argument
#   2. $PX4_DIR
#   3. auto-detection:  a) ancestors of $PWD and of this repo
#                       b) siblings/parents of this repo
#                       c) conventional locations under $HOME
#                       d) bounded, pruned, timed find under $HOME
#   Multiple auto-detected candidates -> abort and ask the user to disambiguate.

# ------------------------------------------------------------------ basics --

px4_abspath() {
	# Canonical absolute path without requiring realpath(1).
	if [ -d "$1" ]; then
		(cd "$1" 2>/dev/null && pwd)
	else
		printf '%s\n' "$1"
	fi
}

# The four structural markers this integration needs. A directory that misses
# any of them is not a usable PX4-Autopilot checkout for our purposes.
px4_is_checkout() {
	[ -n "${1:-}" ] || return 1
	[ -d "$1" ] || return 1
	[ -f "$1/Makefile" ] || return 1
	[ -d "$1/Tools/simulation" ] || return 1
	[ -f "$1/src/modules/simulation/simulator_mavlink/CMakeLists.txt" ] || return 1
	[ -f "$1/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt" ] || return 1
	return 0
}

px4_version() {
	_v=""
	if [ -d "$1/.git" ] && command -v git >/dev/null 2>&1; then
		_v="$(git -C "$1" describe --tags --always --dirty 2>/dev/null || true)"
	fi
	if [ -z "$_v" ] && [ -f "$1/version.txt" ]; then
		_v="$(head -n1 "$1/version.txt" 2>/dev/null || true)"
	fi
	[ -n "$_v" ] || _v="unknown"
	printf '%s\n' "$_v"
}

# ------------------------------------------------------------ candidate set --

# Newline-separated, de-duplicated list of confirmed checkouts.
_px4_cands=""
_px4_reasons=""

_px4_add() {
	# _px4_add DIR REASON — validate, canonicalise, de-duplicate, remember.
	px4_is_checkout "$1" || return 0
	_abs="$(px4_abspath "$1")"
	case "
$_px4_cands" in
		*"
$_abs
"*) return 0 ;;
	esac
	_px4_cands="$_px4_cands$_abs
"
	_px4_reasons="$_px4_reasons$_abs|$2
"
}

_px4_reason_for() {
	printf '%s' "$_px4_reasons" | while IFS='|' read -r p r; do
		[ "$p" = "$1" ] && { printf '%s\n' "$r"; break; }
	done
}

_px4_count() {
	printf '%s' "$_px4_cands" | grep -c . 2>/dev/null || true
}

# (a) walk up from a starting directory looking for an enclosing checkout
_px4_walk_up() {
	_d="$(px4_abspath "$1")"
	_label="$2"
	while [ -n "$_d" ] && [ "$_d" != "/" ]; do
		_px4_add "$_d" "$_label ($_d)"
		_d="$(dirname "$_d")"
	done
}

# (d) bounded last-resort search: depth-limited, prunes heavy directories,
#     time-limited so it cannot run away on a huge home directory.
_px4_find_home() {
	[ -d "${HOME:-}" ] || return 0
	_finder="find"
	if command -v timeout >/dev/null 2>&1; then
		_finder="timeout 20 find"
	fi
	# shellcheck disable=SC2086
	$_finder "$HOME" -maxdepth 4 \
		\( -name .git -o -name build -o -name node_modules -o -name .cache \
		   -o -name Library -o -name snap -o -name .local -o -name venv \
		   -o -name .venv -o -name __pycache__ \) -prune -o \
		-type d -name 'PX4-Autopilot*' -print 2>/dev/null | head -n 20
}

# --------------------------------------------------------------- resolution --

# px4_resolve [EXPLICIT_PATH]
# REPO_DIR must be set by the caller (absolute path of this repository).
px4_resolve() {
	_explicit="${1:-}"
	_repo="${REPO_DIR:-$(px4_abspath "$(dirname "${BASH_SOURCE[0]:-$0}")/..")}"

	PX4_RESOLVED=""
	PX4_RESOLVED_HOW=""
	PX4_RESOLVED_EXPLICIT=0

	# --- 1. explicit CLI argument (always wins) ---------------------------
	if [ -n "$_explicit" ]; then
		case "$_explicit" in "~"|"~/"*) _explicit="$HOME${_explicit#\~}" ;; esac
		if [ ! -d "$_explicit" ]; then
			printf 'error: the PX4 path you passed does not exist: %s\n' "$_explicit" >&2
			return 1
		fi
		if ! px4_is_checkout "$_explicit"; then
			printf 'error: %s does not look like a PX4-Autopilot checkout.\n' "$_explicit" >&2
			printf '       Expected to find: Makefile, Tools/simulation/,\n' >&2
			printf '       src/modules/simulation/simulator_mavlink/CMakeLists.txt and\n' >&2
			printf '       ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt\n' >&2
			return 1
		fi
		PX4_RESOLVED="$(px4_abspath "$_explicit")"
		PX4_RESOLVED_HOW="explicit command-line argument"
		PX4_RESOLVED_EXPLICIT=1
		return 0
	fi

	# --- 2. $PX4_DIR ------------------------------------------------------
	if [ -n "${PX4_DIR:-}" ]; then
		_p="$PX4_DIR"
		case "$_p" in "~"|"~/"*) _p="$HOME${_p#\~}" ;; esac
		if px4_is_checkout "$_p"; then
			PX4_RESOLVED="$(px4_abspath "$_p")"
			PX4_RESOLVED_HOW="\$PX4_DIR environment variable"
			PX4_RESOLVED_EXPLICIT=1
			return 0
		fi
		printf 'error: $PX4_DIR is set to "%s" but that is not a PX4-Autopilot checkout.\n' "$PX4_DIR" >&2
		printf '       Fix or unset $PX4_DIR, or pass the path explicitly.\n' >&2
		return 1
	fi

	# --- 3. auto-detection ------------------------------------------------
	# Searched in tiers: the first tier that finds anything decides. A tier that
	# finds exactly one checkout wins outright; a tier that finds several is
	# ambiguous and aborts. This way an enclosing tree beats a tree that merely
	# sits next to the repo, instead of the two colliding as a false ambiguity.
	printf 'No PX4 path given and $PX4_DIR is unset - auto-detecting...\n' >&2

	for _tier in enclosing siblings home find; do
		_px4_cands=""
		_px4_reasons=""

		case "$_tier" in
			enclosing)
				# (a) a checkout containing the cwd, or containing this repo
				#     (covers a clone placed inside PX4-Autopilot/, e.g. Tools/)
				_px4_walk_up "$PWD"   "enclosing PX4 tree above the current directory"
				_px4_walk_up "$_repo" "enclosing PX4 tree above this repository"
				;;
			siblings)
				# (b) next to / above this repository
				_px4_add "$(dirname "$_repo")" "this repository's parent directory"
				for _rel in ../PX4-Autopilot ../../PX4-Autopilot ../px4/PX4-Autopilot; do
					_px4_add "$_repo/$_rel" "relative to this repository ($_rel)"
				done
				;;
			home)
				# (c) conventional locations under the invoking user's home
				[ -n "${HOME:-}" ] || continue
				for _sub in "" src Code code dev develop workspace ws git repos Projects projects Documents; do
					if [ -z "$_sub" ]; then
						_px4_add "$HOME/PX4-Autopilot" "conventional location \$HOME/PX4-Autopilot"
					else
						_px4_add "$HOME/$_sub/PX4-Autopilot" "conventional location \$HOME/$_sub/PX4-Autopilot"
					fi
				done
				;;
			find)
				# (d) bounded, pruned, time-limited last resort
				printf '  nothing in the usual places; running a bounded search under $HOME...\n' >&2
				_hits="$(_px4_find_home || true)"
				[ -n "$_hits" ] || continue
				_old_ifs="$IFS"; IFS='
'
				for _h in $_hits; do
					[ -n "$_h" ] || continue
					_px4_add "$_h" "bounded search under \$HOME"
				done
				IFS="$_old_ifs"
				;;
		esac

		if [ "$(_px4_count)" -gt 0 ]; then break; fi
	done

	_n="$(_px4_count)"

	# --- no candidates ----------------------------------------------------
	if [ "$_n" -eq 0 ]; then
		printf '\n' >&2
		printf 'error: could not find a PX4-Autopilot checkout automatically.\n' >&2
		printf '\n' >&2
		printf '       Looked for a directory containing all of:\n' >&2
		printf '         Makefile\n' >&2
		printf '         Tools/simulation/\n' >&2
		printf '         src/modules/simulation/simulator_mavlink/CMakeLists.txt\n' >&2
		printf '         ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt\n' >&2
		printf '\n' >&2
		printf '       ...in: directories above the current one and above this repo,\n' >&2
		printf '       next to this repo, the usual spots under $HOME, and a bounded\n' >&2
		printf '       search of $HOME (max depth 4).\n' >&2
		printf '\n' >&2
		printf '       Tell it where your checkout is:\n' >&2
		printf '         %s /path/to/PX4-Autopilot\n' "${0##*/}" >&2
		printf '       or:\n' >&2
		printf '         export PX4_DIR=/path/to/PX4-Autopilot\n' >&2
		printf '\n' >&2
		printf '       If you have not cloned PX4 yet:\n' >&2
		printf '         git clone --recursive https://github.com/PX4/PX4-Autopilot.git\n' >&2
		return 1
	fi

	# --- ambiguous: never guess ------------------------------------------
	if [ "$_n" -gt 1 ]; then
		printf '\n' >&2
		printf 'error: found %s PX4-Autopilot checkouts - refusing to guess which one you mean.\n' "$_n" >&2
		printf '\n' >&2
		_old_ifs="$IFS"; IFS='
'
		for _c in $_px4_cands; do
			[ -n "$_c" ] || continue
			printf '         %s\n' "$_c" >&2
			printf '           version: %s\n' "$(px4_version "$_c")" >&2
			printf '           found:   %s\n' "$(_px4_reason_for "$_c")" >&2
		done
		IFS="$_old_ifs"
		printf '\n' >&2
		printf '       Pick one explicitly:\n' >&2
		printf '         %s /path/to/PX4-Autopilot\n' "${0##*/}" >&2
		printf '       or:\n' >&2
		printf '         export PX4_DIR=/path/to/PX4-Autopilot\n' >&2
		return 1
	fi

	# --- exactly one ------------------------------------------------------
	PX4_RESOLVED="$(printf '%s' "$_px4_cands" | grep . | head -n1)"
	PX4_RESOLVED_HOW="auto-detected: $(_px4_reason_for "$PX4_RESOLVED")"
	PX4_RESOLVED_EXPLICIT=0
	return 0
}
