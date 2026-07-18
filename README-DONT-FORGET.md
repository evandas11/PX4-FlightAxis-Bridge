# ⚠️ DON'T FORGET — SYNC REMINDER

This repository is a **mirror** of files that live inside a PX4 source tree.
The copy in the PX4 tree is the one that actually builds and runs; the copy
here is the one that gets published.

**If you edit a file directly in the PX4 tree, you MUST update the same file in
this repository too — and vice versa.** Otherwise this GitHub repo goes stale
and becomes useless to everyone else.

## How to stay in sync

| You just edited… | Run |
|---|---|
| files in the PX4 tree | `./scripts/sync-from-px4.sh` |
| files in this repo | `./scripts/sync-to-px4.sh` |

Both scripts locate the PX4 checkout with the shared resolver in
`scripts/detect-px4.sh`: an explicit path argument, else `$PX4_DIR`, else
auto-detection (see README.md, Method 1).

## Not covered by the scripts

Two one-line registrations live inside files owned by PX4 and are **not** mirrored
as files. `./install.sh` applies them automatically (idempotently); only a manual
install needs them added by hand (README Method 2, steps 2–3):

1. `include(sitl_targets_flightaxis.cmake)` in
   `src/modules/simulation/simulator_mavlink/CMakeLists.txt`
2. The four `120x_flightaxis_*` entries in
   `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

`sync-to-px4.sh` prints a warning when either is missing.

## Golden rule

**Edit → sync → commit → push.** Never let the two copies drift.
