# PX4-FlightAxis-Bridge

**RealFlight (FlightAxis Link) as a PX4 SITL simulator** — integrated the same way as the
FlightGear bridge: files live in the PX4 tree, get compiled by the PX4 make system, and
launch with one `make` command.

```bash
# RealFlight running on a Windows box (FlightAxis Link enabled in RealFlight settings)
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_plane
# QGC connects on UDP 14550 as usual
```

Full design rationale, frame conversions, and timing logic:
[`FLIGHTAXIS_PX4_INTEGRATION.md`](FLIGHTAXIS_PX4_INTEGRATION.md) (the spec — §6/§7 are
verified against ArduPilot `SIM_FlightAxis.cpp`).

---

## Repository layout

Paths are identical to where the files go inside `PX4-Autopilot/`, so installation is a
straight copy:

```
Tools/simulation/flightaxis/
├── sitl_run.sh                          # runner invoked by the make target
└── flightaxis_bridge/
    ├── CMakeLists.txt
    ├── FA_check.py                      # sanity-ping RealFlight :18083 before start
    ├── get_FAbridge_params.py           # models/<name>.json → bridge argv
    ├── cmake/FindMAVLink.cmake
    ├── models/{plane,quad,quadplane,heli}.json   # RealFlight channel maps
    └── src/
        ├── flightaxis_bridge.cpp        # main loop + ArduPilot 3-branch timing (§7)
        ├── fa_communicator.{h,cpp}      # SOAP client, port of ArduPilot socket logic (§8.1)
        ├── vehicle_state.{h,cpp}        # RF→NED conversions (§6) + sensor synthesis
        ├── px4_communicator.{h,cpp}     # TCP 4560 HIL link (from PX4-FlightGear-Bridge)
        └── geo_mag_declination.{h,cpp}  # WMM tables (from PX4-FlightGear-Bridge)

src/modules/simulation/simulator_mavlink/
└── sitl_targets_flightaxis.cmake        # make-target registration (PX4 v1.16 pattern)
    # + one include() line added to that directory's CMakeLists.txt — install.sh does this

install.sh / uninstall.sh                # one-command install and clean removal
scripts/
├── detect-px4.sh                        # shared PX4-checkout detection (all scripts)
├── sync-to-px4.sh                       # repo -> PX4 tree (development)
└── sync-from-px4.sh                     # PX4 tree -> repo (development)

ROMFS/px4fmu_common/init.d-posix/airframes/
├── 1200_flightaxis_plane
├── 1201_flightaxis_quad
├── 1202_flightaxis_quadplane
└── 1203_flightaxis_heli
    # + four entries added to that directory's CMakeLists.txt — install.sh does this
```

## Install into a PX4 tree

Tested against **PX4 v1.16.0**. Two ways in: **Method 1 is the normal choice**; Method 2 is
there if you would rather apply the changes yourself, or if your PX4 tree is a layout the
installer refuses.

### Method 1 — Automatic install (recommended)

```bash
git clone https://github.com/<you>/PX4-FlightAxis-Bridge.git
cd PX4-FlightAxis-Bridge
./install.sh
```

That copies the payload into your PX4 checkout, registers it in PX4's two `CMakeLists.txt`
files, builds `px4_sitl_nolockstep` plus the bridge, and verifies the result. **The build
takes roughly 10–30 minutes on a fresh PX4 tree** (a couple of minutes if the tree is
already built). Running it a second time is safe — it detects what is already in place and
changes nothing.

Variants:

```bash
./install.sh /path/to/PX4-Autopilot     # explicit target tree
PX4_DIR=/path/to/PX4-Autopilot ./install.sh
./install.sh --dry-run                  # print every action, change nothing
./install.sh --no-build                 # install and register files only
./install.sh --yes                      # skip the confirmation prompt (CI)
./install.sh --force                    # override the safety refusals (see below)
./install.sh --help
```

**Which PX4 tree does it pick?** The first hit wins, and the choice is always printed
(path, version, and how it was found) before anything is written:

1. the `PATH_TO_PX4` argument, if given;
2. `$PX4_DIR`, if set;
3. auto-detection, in tiers — the first tier that finds anything decides:
   a. a PX4 checkout *containing* the current directory or containing this repo (so a
      clone placed inside `PX4-Autopilot/` is found);
   b. next to this repo: `../PX4-Autopilot`, `../../PX4-Autopilot`, the repo's parent;
   c. conventional spots under `$HOME`: `PX4-Autopilot`, `src/`, `Code/`, `dev/`,
      `workspace/`, `git/`, `projects/`, …;
   d. a bounded, pruned, time-limited search under `$HOME` (max depth 4).

If several checkouts turn up, the installer lists them all with their versions and stops —
disambiguate with an argument or `$PX4_DIR`. If none turn up, it says so and asks for the
path; it never guesses or creates a directory. A candidate only counts if it structurally
looks like PX4 (`Makefile`, `Tools/simulation/`, and both `CMakeLists.txt` files).

**What it refuses to do** (each overridable with `--force` where it is safe to):
an unrecognised or non-v1.16 PX4 layout; an existing `120[0-3]_*` airframe that is not
ours (SYS_AUTOSTART collision); uncommitted local modifications to the two PX4-owned
`CMakeLists.txt` files; a missing splice anchor (it aborts and tells you what to add by
hand rather than guessing). It never uses `sudo`. Both PX4-owned `CMakeLists.txt` files are
backed up to `<file>.flightaxis.bak` first, and everything it changed is listed at the end.

Reverse it all at any time:

```bash
./uninstall.sh                 # same target resolution and --dry-run/--yes flags
```

It removes the payload and the four airframes, undoes both registrations (restoring the
backups, or — if you edited those files afterwards — removing only the lines it added), and
touches nothing else.

Then jump to [Run it](#run-it).

### Method 2 — Manual install

Do this if you want to see exactly what changes, or if your tree is one the installer
refuses: PX4 **v1.13–v1.15** register SITL targets in `platforms/posix/cmake/sitl_target.cmake`
rather than in per-simulator `sitl_targets_*.cmake` files, so the splice below does not
apply as written (see spec §3).

1. Copy `Tools/`, `src/`, and `ROMFS/` from this repo over your `PX4-Autopilot/` checkout,
   preserving the paths (they already mirror the PX4 layout). `./scripts/sync-to-px4.sh`
   does exactly this.

2. In `PX4-Autopilot/src/modules/simulation/simulator_mavlink/CMakeLists.txt`, add one line
   to the `include(sitl_targets_*.cmake)` block, keeping it alphabetical:

   ```cmake
   include(sitl_targets_flightaxis.cmake)
   include(sitl_targets_flightgear.cmake)
   include(sitl_targets_gazebo-classic.cmake)
   ```

3. In `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`, register
   the four airframes inside `px4_add_romfs_files(...)`, in sorted position (after the
   `10xx` block, before `2507_gazebo-classic_cloudship`), tab-indented like its neighbours:

   ```cmake
   	1200_flightaxis_plane
   	1201_flightaxis_quad
   	1202_flightaxis_quadplane
   	1203_flightaxis_heli
   ```

4. Build from the PX4 tree:

   ```bash
   make px4_sitl_nolockstep
   ninja -C build/px4_sitl_nolockstep flightaxis_bridge
   ```

   The bridge binary should appear at
   `build/px4_sitl_nolockstep/build_flightaxis_bridge/flightaxis_bridge`.

### Run it

Both methods end in the same place — from your PX4 tree:

```bash
PX4_FLIGHTAXIS_IP=<windows-ip> make px4_sitl_nolockstep flightaxis_plane
```

(also `flightaxis_quad`, `flightaxis_quadplane`, `flightaxis_heli`; QGC connects on UDP 14550)

## RealFlight setup (once per aircraft)

- Enable **FlightAxis Link** in RealFlight (listens on TCP 18083). Wired network or
  same-host only — WiFi cannot hold the ~250 Hz SOAP rate.
- In the RealFlight aircraft editor: strip expo/mixes/gyros, max servo speed, one channel
  per actuator. Tridge's ArduPilot RealFlight model collection is directly reusable.
- The QGC **Actuators** geometry must match the `"px4"` indices in the corresponding
  `models/<name>.json`.

## Adding a new aircraft

Three steps (same workflow as the FlightGear bridge):
1. New `models/<name>.json` (channel map — spec §5).
2. Add the model name to the `models` list in `sitl_targets_flightaxis.cmake`.
3. New airframe script `12xx_flightaxis_<name>` (+ CMakeLists entry).

## Credits / references

- ArduPilot `SIM_FlightAxis.{h,cpp}` — ground truth for conversions, timing, and the
  SOAP socket/reconnect logic.
- [PX4-FlightGear-Bridge](https://github.com/PX4/PX4-FlightGear-Bridge) (ThunderFly
  s.r.o.) — the integration template; `px4_communicator` and `geo_mag_declination` are
  reused from it (BSD-3-Clause).

## License

BSD-3-Clause (same as PX4 and the FlightGear bridge components this reuses).
