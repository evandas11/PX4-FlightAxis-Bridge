# Contributing / maintainer notes

## ⚠️ Keep the repo and the PX4 tree in sync

This repository is a **mirror** of files that live inside a PX4 source tree.
The copy in the PX4 tree is the one that actually builds and runs; the copy
here is the one that gets published.

**If you edit a file directly in the PX4 tree, you MUST update the same file in
this repository too — and vice versa.** Otherwise the published repo goes stale
and becomes useless to everyone else.

| You just edited… | Run |
|---|---|
| files in the PX4 tree | `./scripts/sync-from-px4.sh` |
| files in this repo | `./scripts/sync-to-px4.sh` |

Both scripts locate the PX4 checkout with the shared resolver in
`scripts/detect-px4.sh`: an explicit path argument, else `$PX4_DIR`, else
auto-detection (see README.md, Method 1). Both take `--dry-run` (show the diff,
write nothing) and `--yes`, and both prompt before writing when the tree was
auto-detected rather than named.

`sync-from-px4.sh` is the only script that rsyncs with `--delete`, and its
destination is this repository, so it first checks that the PX4 tree holds a
*complete* payload and refuses to run otherwise — syncing from a half-installed
tree would delete the missing files here, and untracked ones would not come back.
Neither script writing *into* the PX4 tree ever deletes, because that is where
users keep their own `models/<name>.json`.

**Golden rule: edit → sync → commit → push.** Never let the two copies drift.

## Not mirrored as files

Two one-line registrations live inside files owned by PX4 and are **not** mirrored
as files. `./install.sh` applies them automatically (idempotently); only a manual
install needs them added by hand (README Method 2, steps 2–3):

1. `include(sitl_targets_flightaxis.cmake)` in
   `src/modules/simulation/simulator_mavlink/CMakeLists.txt`
2. The four `120x_flightaxis_*` entries in
   `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

`sync-to-px4.sh` prints a warning when either is missing.

## What this integration owns

`scripts/detect-px4.sh` holds the single source of truth: `FA_AIRFRAMES` (the
four airframe names), `FA_MODELS`, `FA_ENTRY_RE` (the exact-name matcher shared
by install, uninstall and both sync scripts) and `fa_payload_files` (the install
manifest). Change those in one place and every script follows.

Two rules the tooling depends on, and which are easy to break:

- **We own the names, not the id range.** Ids 1200–1219 are reserved by
  documentation but only `1200`–`1203` are installed. Never widen a matcher to
  a range: a user's own `1204_flightaxis_cessna` must be invisible to us, or
  uninstall silently de-registers their airframe.
- **Install and uninstall must use the same matcher.** Anything install detects,
  uninstall must remove, byte for byte — that is why the matchers live in
  `detect-px4.sh` and not in either script.

Adding a payload file means adding it to `fa_payload_files`, or uninstall will
leave it behind (C++ sources under `flightaxis_bridge/src/` are enumerated
automatically).
