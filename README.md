# PX4-FlightAxis-Bridge

**RealFlight (FlightAxis Link) as a PX4 SITL simulator** — integrated the same way as the
FlightGear bridge: files live in the PX4 tree, get compiled by the PX4 make system, and
launch with one `make` command.

```bash
# RealFlight running on a Windows box (FlightAxis Link enabled in RealFlight settings)
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_plane
# QGC connects on UDP 14550 as usual
```

Day-to-day operation — network setup, per-vehicle channel maps, home position, MAVLink
endpoints, troubleshooting: **[RUNNING.md](RUNNING.md)**.

Full design rationale, frame conversions, and timing logic:
[`FLIGHTAXIS_PX4_INTEGRATION.md`](FLIGHTAXIS_PX4_INTEGRATION.md) (the spec — §6/§7 are
verified against ArduPilot `SIM_FlightAxis.cpp`).

---

## Status

Tested against **PX4 v1.16.0**. **No part of this has yet been run against a real RealFlight
installation** — RealFlight is Windows-only and no Windows machine has been in the loop.
What has been verified, and what has not, is listed under [Validation status](#validation-status)
before you rely on any of it.

## Prerequisites

On the Linux box that builds and runs PX4:

| Need | Why |
|---|---|
| **Eigen3 development headers** | the bridge's only external library (`find_package(Eigen3 REQUIRED)`) — `apt install libeigen3-dev` |
| **python3** | `sitl_run.sh` calls `FA_check.py` and `get_FAbridge_params.py` at launch |
| **cmake** | builds the bridge as a PX4 `ExternalProject` |
| **ninja or make** | either works; `install.sh` prefers ninja and falls back to `cmake --build` |
| rsync *(optional)* | `install.sh` uses it to copy the payload; falls back to `cp -R` |
| everything PX4 itself needs | see PX4's own setup docs — this repo adds no other requirement |

**MAVLink headers are not a separate dependency.** They come from the PX4 build via the bundled
`cmake/FindMAVLink.cmake`; there is nothing to install and no version to match.

RealFlight itself runs on a **separate Windows machine** (or VM), reachable over a wired network.

## Repository layout

Everything except the top-level documents and scripts sits at exactly the path it occupies inside
`PX4-Autopilot/`, so installation is a straight copy:

```
README.md                                # this file - the user manual
FLIGHTAXIS_PX4_INTEGRATION.md            # the spec - design rationale, frame conversions, timing
README-DONT-FORGET.md                    # maintainer note: keeping repo and PX4 tree in sync
LICENSE
.gitignore

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
        ├── px4_communicator.{h,cpp}     # TCP 4560 HIL link (adapted from PX4-FlightGear-Bridge)
        └── geo_mag_declination.{h,cpp}  # WMM tables (verbatim from PX4-FlightGear-Bridge)

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

Two ways in: **Method 1 is the normal choice**; Method 2 is
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

4. Build from the PX4 tree — first PX4 itself, then the bridge target:

   ```bash
   make px4_sitl_nolockstep

   # then either, depending on your generator:
   ninja -C build/px4_sitl_nolockstep flightaxis_bridge          # ninja builds
   cmake --build build/px4_sitl_nolockstep --target flightaxis_bridge   # any generator
   ```

   The second form works everywhere and is what `install.sh` falls back to when ninja is
   absent. The bridge binary should appear at
   `build/px4_sitl_nolockstep/build_flightaxis_bridge/flightaxis_bridge`.

### Run it

Both methods end in the same place — from your PX4 tree:

```bash
PX4_FLIGHTAXIS_IP=<windows-ip> make px4_sitl_nolockstep flightaxis_plane
```

(also `flightaxis_quad`, `flightaxis_quadplane`, `flightaxis_heli`; QGC connects on UDP 14550)

## RealFlight setup

Once per RealFlight installation:

- Enable the FlightAxis link. **RealFlight Evolution:** press ESC, then
  **Settings -> Physics -> Quality**, and enable **"RealFlight Link"**. **RealFlight 8/9:** the
  same option lives directly under **Settings -> Physics**. It listens on TCP 18083.
- In the same Physics settings, set **"Automatic Reset Delay(sec)"** to `2.0`, and set both
  **"Pause Sim When in Background"** and **"Pause Sim when in Menu"** to **No** — otherwise
  RealFlight stops feeding the bridge the moment it loses focus.
- Restart RealFlight after changing these.
- Wired network or same-host only — WiFi cannot hold the ~250 Hz SOAP rate.
- Leave the physics speed multiplier at 1.0; the bridge warns if it is not.

Once per aircraft:

- In the RealFlight aircraft editor: strip expo/mixes/gyros, max servo speed, one channel
  per actuator. Tridge's ArduPilot RealFlight model collection is directly reusable.
- The QGC **Actuators** geometry must match the `"px4"` indices in the corresponding
  `models/<name>.json` — see [Model JSON](#model-json-channel-maps) below.

## Home position

The bridge anchors the RealFlight world to a geodetic home position, overridable by environment
variable:

| Variable | Default |
|---|---|
| `PX4_HOME_LAT` | `47.397742` |
| `PX4_HOME_LON` | `8.545594` |
| `PX4_HOME_ALT` | `488.0` (metres) |

```bash
PX4_HOME_LAT=51.4769 PX4_HOME_LON=-0.0005 PX4_HOME_ALT=15 \
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_plane
```

The barometer is derived from this same altitude datum rather than from RealFlight's scenery
elevation, deliberately — see spec §6.

## Model JSON: channel maps

Each `models/<name>.json` maps RealFlight transmitter channels (`rf`) to PX4 actuator outputs
(`px4` = an index into `HIL_ACTUATOR_CONTROLS.controls[]`). The `px4` numbers are dictated by the
airframe script, not chosen freely:

- `PWM_MAIN_FUNC<N>` lands on `controls[N-1]`.
- Function `101+k` is `CA_ROTOR<k>` — a motor, range `[0,1]` → `"scale": "unipolar"`.
- Function `201+k` is `CA_SV_CS<k>` — a surface, range `[-1,1]` → `"scale": "bipolar"`.
- The allocator emits **motors before servos**, so on mixed airframes the motors take the low
  indices no matter where they sit on the transmitter. This is why the shipped maps look
  scrambled — e.g. `quadplane.json` maps `rf0..rf7` to `px4` `5, 7, 4, 8, 0, 1, 2, 3`.

Other keys:

- `"reverse": true` is applied *after* scaling, as `v -> 1-v`.
- `"disarm"` is the value sent while PX4 is disarmed. **`-1`, which is also the default when the
  key is omitted, means "hold the last output"** — correct for surfaces, wrong for motors, so
  every motor row sets `"disarm": 0.0` explicitly.
- `"UnmappedDefault"` is sent on every RealFlight channel the map does not mention.
- **Duplicate `rf` or `px4` indices abort the bridge at startup** with a message naming both
  offending entries, rather than silently letting one win.

### Options

`"Options"` is a list of flags; `get_FAbridge_params.py` flattens it to a bitmask.

| Option | Effect |
|---|---|
| `ResetPosition` | issue `ResetAircraft` on startup, so every run begins from a known state. On by default in all four shipped models. |
| `Rev4Servos` | swap RealFlight channels 1–4 with 5–8 wholesale, for RF models built that way. **Do not combine with an already-reordered map** such as `quadplane.json` — you would double-swap. |
| `HeliDemix` | convert the three swash servo outputs back into the roll/pitch/collective triple RealFlight expects (`roll=s1−s2`, `pitch=(s1+s2)/2−s3`, `col=(s1+s2+s3)/3`). Requires the swash geometry `1203_flightaxis_heli` forces via `CA_SP0_ANG*` = 300/60/180. |
| `SilenceFPS` | suppress the periodic FPS/glitch line on stderr. |

Full derivation, per-model tables, and the heli traps: spec §5.

## Adding a new aircraft

Four steps (same workflow as the FlightGear bridge):
1. New `models/<name>.json` (channel map — see above, and spec §5).
2. Add the model name to the `models` list in `sitl_targets_flightaxis.cmake`.
3. New airframe script `12xx_flightaxis_<name>`, whose `PWM_MAIN_FUNC*` assignments must agree
   with the JSON's `px4` indices.
4. **Register that airframe in
   `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`.** Skipping this is the classic
   mistake: everything builds cleanly and the airframe simply never reaches the ROMFS, so PX4
   starts with no matching `SYS_AUTOSTART`.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `RealFlight FlightAxis not reachable at <ip>:18083` | The pre-flight check failed before anything started. RealFlight not running, the link option not enabled (see [RealFlight setup](#realflight-setup)), the wrong IP in `PX4_FLIGHTAXIS_IP`, or a Windows firewall blocking TCP 18083. |
| `get_FAbridge_params.py failed for models/<model>.json` | The model JSON is missing, malformed, or names an unknown `Options` / `scale` value. The exact reason is printed just above. |
| `bad channel map: RealFlight channel rfN is mapped twice` <br> `bad channel map: PX4 control index px4[N] is mapped twice` | Two rows in the model JSON share an `rf` or a `px4` index. The message names both entries; fix the JSON. The bridge refuses to start rather than silently letting one win. |
| `WARNING: RealFlight physics speed multiplier is <x> (set it to 1.0)` | RealFlight is running fast or slow motion. All timing and sensor synthesis assume real time — reset the multiplier in RealFlight. |
| `glitch 0.35s` lines appearing | The bridge absorbed a physics-time jump (network hiccup) so it does not reach EKF2 as a time jump. Occasional lines are benign; a steady stream means the link cannot keep up — go wired, or move the bridge closer to the RealFlight machine. |
| **Hangs silently at `waiting for PX4 on TCP 4560`** | The bridge is up and waiting, but PX4 never connected. Usually PX4 exited at startup — scroll back for its error. The most common cause is the missing airframes `CMakeLists.txt` registration (see step 4 above): the airframe is absent from the ROMFS, so `SYS_AUTOSTART` matches nothing. Also check that nothing else already holds TCP 4560. |
| Aircraft twitches or flies inverted on one axis | Channel map mismatch. Compare the JSON's `px4` indices against the `PWM_MAIN_FUNC*` block in the matching airframe script — they are derived, not free (see [Model JSON](#model-json-channel-maps)). |
| Heli yaw permanently offset | The tail row must be `"scale": "unipolar"` — under `CA_AIRFRAME 10` the tail is a motor and PX4 already normalises it to `[0,1]`. |

## Validation status

**Verified on this machine:**

- The bridge builds; the four `flightaxis_*` make targets exist and resolve.
- All four model JSONs flatten cleanly through `get_FAbridge_params.py`.
- `FA_check.py` fails correctly against an unreachable host.
- **End to end against a mock FlightAxis server:** PX4 connects, EKF2 converges, the synthesised
  sensors are sane (baro and GPS altitudes agree), the plane and quad channel maps are correct
  end to end including bipolar/reverse/unipolar scaling and disarm values, and the
  reconnect / reset / glitch / sim-death resilience cases all behave.

**Not yet verified — needs a real Windows RealFlight machine:**

- Rate sign conventions, taxi N/E tracking, high-alpha pitot behaviour, compass heading, and the
  nose-up-90° attitude case.
- A flown circuit, so EKF innovation bounds in real flight are unknown.
- The quadplane and heli channel maps beyond static reasoning, and `HeliDemix` against a real swash.

Spec §11 has the full checklist these correspond to.

## Credits / references

- [ArduPilot](https://github.com/ArduPilot/ardupilot) `SIM_FlightAxis.{h,cpp}` — ground
  truth for conversions, timing, and the SOAP socket/reconnect logic. `fa_communicator`,
  the three-branch timing in `flightaxis_bridge.cpp`, and the conversions in
  `vehicle_state.cpp` are ports of it, which is why this project is GPLv3. GPLv3,
  © ArduPilot Dev Team.
- [PX4-FlightGear-Bridge](https://github.com/PX4/PX4-FlightGear-Bridge) (ThunderFly
  s.r.o.) — the integration template. `geo_mag_declination.{h,cpp}` and
  `cmake/FindMAVLink.cmake` are reused verbatim; `px4_communicator.{h,cpp}` is adapted from
  it (per-message decimation added for RealFlight's ~250 Hz frame rate, plus the
  DISTANCE_SENSOR path). BSD-3-Clause.

## License

**GPLv3 or later** — see [LICENSE](LICENSE), and [COPYRIGHT.md](COPYRIGHT.md) for
per-file provenance.

The bridge's SOAP client, physics-time handling and RealFlight→NED conversions are
literal ports of ArduPilot's GPLv3 `SIM_FlightAxis.{h,cpp}`, so the combined work is
GPLv3. The files reused from PX4-FlightGear-Bridge keep their BSD-3-Clause notices,
which is GPL-compatible.

This covers the files in this repository only. The bridge is a standalone executable
that talks to PX4 over MAVLink — installing it does not relicense your PX4 checkout.
