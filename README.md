# PX4-FlightAxis-Bridge

**RealFlight (FlightAxis Link) as a PX4 SITL simulator** — integrated the way PX4's in-tree
simulator bridges are: files live in the PX4 tree, get compiled by the PX4 make system, and
launch with one `make` command.

```bash
# RealFlight running on a Windows box (RealFlight Link enabled in RealFlight settings)
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 PX4_HOME_LON=175.7433944 PX4_HOME_ALT=48.0 PX4_HOME_YAW=235 \
make px4_sitl_nolockstep flightaxis_plane
# QGC connects on UDP 14550 as usual
```

Day-to-day operation — network setup, per-vehicle channel maps, home position, MAVLink
endpoints, troubleshooting: **[RUNNING.md](RUNNING.md)**.

Using it with ROS 2 — uXRCE-DDS topics, offboard control, topic rates and the timestamp
caveats: **[ROS2.md](ROS2.md)**.

Driving a real flight-controller board instead of SITL: **[HITL.md](HITL.md)**.

Full design rationale, frame conversions, and timing logic:
[`FLIGHTAXIS_PX4_INTEGRATION.md`](FLIGHTAXIS_PX4_INTEGRATION.md) (the spec — §6/§7 follow the
upstream FlightAxis implementation named in [COPYRIGHT.md](COPYRIGHT.md)).

---

## Status

Targets **PX4 v1.16.0**.

## Prerequisites

On the Linux box that builds and runs PX4:

| Need | Why |
|---|---|
| **an existing PX4-Autopilot checkout** | this integration installs *into* a PX4 tree; it never clones or creates one. Clone PX4 v1.16 and run its setup script first — `git clone -b v1.16.0 --recursive https://github.com/PX4/PX4-Autopilot.git` |
| **Eigen3 development headers** | the bridge's only external library (`find_package(Eigen3 REQUIRED)`) — `apt install libeigen3-dev` |
| **python3** | `sitl_run.sh` calls `FA_check.py` and `get_FAbridge_params.py` at launch |
| **cmake** | builds the bridge as a PX4 `ExternalProject` |
| **ninja or make** | either works; `install.sh` prefers ninja and falls back to `cmake --build` |
| **QGroundControl** | connects on UDP 14550. Not optional in practice: sensor calibration and radio calibration are only possible there, and both are needed before a first flight |
| **procps** (`pkill`, `pgrep`) | `sitl_run.sh`'s cleanup trap uses them to take PX4 and the bridge down together on Ctrl-C |
| **git**, **tput** *(optional)* | `install.sh` uses `git` to refuse an install over uncommitted local modifications, and `tput` for coloured output; without either it still works, with that check and the colour dropped |
| rsync *(optional)* | `install.sh` uses it to copy the payload; falls back to `cp -R` |
| everything PX4 itself needs | see PX4's own setup docs — beyond the rows above, this repo adds no other requirement |

**MAVLink headers are not a separate dependency.** They come from the PX4 build via the bundled
`cmake/FindMAVLink.cmake`; there is nothing to install and no version to match.

RealFlight itself runs on a **separate Windows machine** (or VM), reachable over a wired network.

To fly manually you also need a **control source**: either a transmitter RealFlight can see on
the Windows box (an InterLink, or any TX RealFlight accepts as an input device), whose sticks the
bridge forwards to PX4 as RC; or a QGC virtual joystick on the Linux box. Neither is needed for a
headless or offboard run — see [Flying it manually](#flying-it-manually).

## Repository layout

Everything except the top-level documents and scripts sits at exactly the path it occupies inside
`PX4-Autopilot/`, so installation is a straight copy:

```
README.md                                # this file - the user manual
RUNNING.md                               # day-to-day operation, channel maps, troubleshooting
ROS2.md                                  # uXRCE-DDS topics, offboard control, rates
HITL.md                                  # hardware-in-the-loop against a real board
FLIGHTAXIS_PX4_INTEGRATION.md            # the spec - design rationale, frame conversions, timing
COPYRIGHT.md                             # per-file provenance and licensing
LICENSE
.gitignore
.gitattributes

Tools/simulation/flightaxis/
├── sitl_run.sh                          # runner invoked by the make target
├── hitl_run.sh                          # hardware-in-the-loop runner
└── flightaxis_bridge/
    ├── CMakeLists.txt
    ├── FA_check.py                      # sanity-ping RealFlight :18083 before start
    ├── get_FAbridge_params.py           # models/<name>.json → bridge argv
    ├── cmake/FindMAVLink.cmake
    ├── models/{plane,quad,quadplane,heli}.json   # RealFlight channel maps
    └── src/
        ├── flightaxis_bridge.cpp        # main loop + 4-branch physics-time handling (§7)
        ├── fa_communicator.{h,cpp}      # SOAP client, socket and reconnect logic (§8.1)
        ├── vehicle_state.{h,cpp}        # RF→NED conversions (§6) + sensor synthesis
        ├── px4_communicator.{h,cpp}     # HIL link: TCP 4560 for SITL, serial/UDP for HITL
        ├── battery_link.{h,cpp}         # RealFlight pack/tank → BATTERY_STATUS on UDP 14580+i
        └── geo_mag_declination.{h,cpp}  # WMM tables — verbatim upstream, see COPYRIGHT.md

src/modules/simulation/simulator_mavlink/
└── sitl_targets_flightaxis.cmake        # make-target registration (PX4 v1.16 pattern)
    # + one include() line added to that directory's CMakeLists.txt — install.sh does this

install.sh / uninstall.sh                # one-command install and clean removal
scripts/
└── detect-px4.sh                        # shared PX4-checkout detection (all scripts)

ROMFS/px4fmu_common/init.d-posix/airframes/
├── 1200_flightaxis_plane
├── 1201_flightaxis_quad
├── 1202_flightaxis_quadplane
└── 1203_flightaxis_heli
    # + four entries added to that directory's CMakeLists.txt — install.sh does this

ROMFS/px4fmu_common/init.d/airframes/          # note: init.d, not init.d-posix
├── 1200_flightaxis_plane.hil
├── 1201_flightaxis_quad.hil
├── 1202_flightaxis_quadplane.hil
└── 1203_flightaxis_heli.hil
    # the HITL airframes. A real board boots from init.d, so these need their own
    # directory and their own registration in that directory's CMakeLists.txt
    # + four entries added there as well — install.sh does this
```

## Install into a PX4 tree

Two ways in: **Method 1 is the normal choice**; Method 2 is
there if you would rather apply the changes yourself, or if your PX4 tree is a layout the
installer refuses.

### Method 1 — Automatic install (recommended)

```bash
# replace YOUR-USER with wherever you cloned this from
git clone https://github.com/YOUR-USER/PX4-FlightAxis-Bridge.git
cd PX4-FlightAxis-Bridge
./install.sh
```

That copies the payload into your PX4 checkout, registers it in PX4's three `CMakeLists.txt`
files, builds `px4_sitl_nolockstep` plus the bridge, and checks the result. **The build
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
   b. next to this repo: `../PX4-Autopilot`, `../../PX4-Autopilot`,
      `../px4/PX4-Autopilot`, the repo's parent;
   c. conventional spots under `$HOME`: `PX4-Autopilot`, `src/`, `Code/`, `dev/`,
      `workspace/`, `git/`, `projects/`, …;
   d. a bounded, pruned, time-limited search under `$HOME` (max depth 4).

If several checkouts turn up, the installer lists them all with their versions and stops —
disambiguate with an argument or `$PX4_DIR`. If none turn up, it says so and asks for the
path; it never guesses or creates a directory. A candidate only counts if it structurally
looks like PX4 (`Makefile`, `Tools/simulation/`, and the `simulator_mavlink` and
`init.d-posix/airframes` `CMakeLists.txt` files).

**What it refuses to do** — an unrecognised or non-v1.16 PX4 layout; an existing
`120[0-3]_*` airframe that is not ours (a genuine SYS_AUTOSTART collision, since those are
the four ids it installs); uncommitted local modifications to the three PX4-owned
`CMakeLists.txt` files. Each of those is overridable with `--force`, and none of them stops
a `--dry-run`. The one refusal `--force` does **not** override is a missing splice anchor:
there it aborts and tells you what to add by hand rather than guessing where your build
system wants it. It never uses `sudo`. All three PX4-owned `CMakeLists.txt` files are backed up
to `<file>.flightaxis.bak` first, and everything it changed is listed at the end.

Ids **1204–1219 are reserved by this document but not used**, and the installer ignores
them entirely — that is the range for your own aircraft (see
[Adding a new aircraft](#adding-a-new-aircraft)).

Reverse it all at any time:

```bash
./uninstall.sh                 # same target resolution and --dry-run/--yes flags
```

It undoes all three registrations (restoring the backups, or — if you edited those files
afterwards — removing only the lines it added) and then deletes **exactly the files it
installed**, listed from a manifest. Specifically:

- Your own `models/<name>.json`, your own `12xx_flightaxis_*` airframe files and their
  `CMakeLists.txt` entries are **not** ours and are left alone.
- `Tools/simulation/flightaxis/` is removed only if it is empty once our files are gone.
  If anything of yours is still in there, the directory stays and the survivors are listed
  so you can decide.
- If you added the `include()` line by hand (Method 2), the uninstaller finds it whatever
  its indentation — but it must be a real, uncommented `include(sitl_targets_flightaxis.cmake)`
  line of its own, which is also the form the installer recognises when deciding whether
  the registration is already present.

Then jump to [Run it](#run-it).

### Method 2 — Manual install

Do this if you want to see exactly what changes, or if your tree is one the installer
refuses: PX4 **v1.13–v1.15** register SITL targets in `platforms/posix/cmake/sitl_target.cmake`
rather than in per-simulator `sitl_targets_*.cmake` files, so the splice below does not
apply as written (see spec §3).

1. Copy `Tools/`, `src/`, and `ROMFS/` from this repo over your `PX4-Autopilot/` checkout,
   preserving the paths — they already mirror the PX4 layout, so a recursive copy of those
   three directories from the repo root lands every file where it belongs:

   ```bash
   cp -R Tools src ROMFS /path/to/PX4-Autopilot/
   ```

   Copy, do not mirror-with-delete: `Tools/simulation/flightaxis/flightaxis_bridge/models/`
   is also where your own model JSONs live.

2. In `PX4-Autopilot/src/modules/simulation/simulator_mavlink/CMakeLists.txt`, add one line
   to the `include(sitl_targets_*.cmake)` block, keeping it alphabetical:

   ```cmake
   include(sitl_targets_flightaxis.cmake)
   ```

   The block is a run of `include(sitl_targets_<name>.cmake)` lines, one per simulator PX4
   ships; ours sorts to the top of it.

   Indent it however you like — leading and trailing whitespace are ignored — but keep it
   as a line of its own and do not comment it out. That is how `install.sh` recognises the
   registration as already present, and how `uninstall.sh` finds the line to remove; a
   commented-out or merged line will be treated as "not installed".

3. In `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`, register
   the four airframes inside `px4_add_romfs_files(...)`, in sorted numeric position — after
   the `10xx` block and before the next id above `1203` — tab-indented like its neighbours:

   ```cmake
   	1200_flightaxis_plane
   	1201_flightaxis_quad
   	1202_flightaxis_quadplane
   	1203_flightaxis_heli
   ```

4. In `PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` — the **`init.d`**
   one, not the `init.d-posix` one edited in step 3 — register the four HITL airframes:

   ```cmake
   	1200_flightaxis_plane.hil
   	1201_flightaxis_quad.hil
   	1202_flightaxis_quadplane.hil
   	1203_flightaxis_heli.hil
   ```

   That file is a single `px4_add_romfs_files(...)` list with `if(CONFIG_*)` guards nested
   inside it, and the entries must land **inside the `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM`
   guard**, beside PX4's own `1001`/`1002`/`110x` HIL airframes — `install.sh` anchors its
   splice on `1103_standard_vtol_sih.hil` for exactly this reason. Sorting them by id instead
   would put them in the `MC_RATE_CONTROL` guard, a different block. Everything in that list is
   tab-indented.

   Skip this step and `flightaxis_hitl_*` builds cleanly while producing no ROMFS airframe at
   all — the same failure described in step 3 of
   [Adding a new aircraft](#adding-a-new-aircraft). It is only needed for HITL; SITL runs are
   unaffected.

5. Build from the PX4 tree — first PX4 itself, then the bridge target:

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

Both methods end in the same place. Use this form — the home position pinned to the field
you actually fly at, and the aircraft started on the runway heading:

```bash
cd ~/PX4-Autopilot

PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 \
PX4_HOME_LON=175.7433944 \
PX4_HOME_ALT=48.0 \
PX4_HOME_YAW=235 \
make px4_sitl_nolockstep flightaxis_quadplane
```

(also `flightaxis_quad`, `flightaxis_quadplane`, `flightaxis_heli`; QGC connects on UDP 14550)

`PX4_FLIGHTAXIS_IP` defaults to `127.0.0.1`, so it can be left out only when RealFlight is on
this same machine; anything else and it is required. Leaving the rest out puts home at PX4's
default in Zurich while RealFlight flies you around a field on the other side of the world.
Everything on the QGC map hangs off that origin, and `PX4_HOME_YAW` is what lines the runway
up with the heading you see in the simulator, so it is worth setting all five every time.

Stop it with Ctrl-C in that terminal: PX4 and the bridge both exit, and the bridge hands
RealFlight back to your transmitter on the way out.

That same terminal is PX4's own interactive shell. Once it is up you get a `pxh>` prompt there,
and every `param …` command in this document — and in RUNNING.md — is typed at it, in the
running instance, alongside the log output. Ctrl-C at that prompt is what quits.

#### Flying it manually

**Out of the box the sticks do nothing, deliberately.** All four airframes ship
`param set-default COM_RC_IN_MODE 4` — "ignore any stick input" — because a headless run
(QGC optional, no transmitter, offboard or mission control only) has no manual control source
at all, and 4 is the only value that skips the RC-loss failsafe. Leave it and commander ignores
the sticks: arming in a manual mode is refused.

The bridge does forward a transmitter regardless of this parameter. RealFlight echoes the
sticks of any TX it can see back in every frame, and the bridge sends those twelve channels to
PX4 as `RC_CHANNELS`, which `SimulatorMavlink` publishes as `input_rc` — the same topic a real
receiver driver feeds. So `listener input_rc` shows live, moving stick values whether or not
commander is allowed to act on them. If passthrough looks broken while `input_rc` is populated
and moving, `COM_RC_IN_MODE` is why. (Passthrough is SITL only; for HITL it is disabled,
because the pilot's transmitter is wired to the board and that receiver is the authoritative
RC path.)

To fly by hand, pick the value that matches your control source:

```
param set COM_RC_IN_MODE 0     # RC transmitter only
param set COM_RC_IN_MODE 2     # transmitter and a QGC joystick, with fallback
```

Use `0` when the transmitter is the only stick source — it also keeps a QGC virtual joystick
from competing with it. Either can be set at the `pxh>` prompt of a running instance, or added
to the airframe file. Set at the prompt it becomes an explicitly saved value, so it outranks the
airframe's `param set-default` on every later run in that working directory too.

Enabling the sticks necessarily re-arms the RC-loss failsafe, which is why no single value
serves both cases: PX4 overloads this one parameter for both jobs. Once it is not 4, a run that
loses (or never has) manual control takes `NAV_RCL_ACT` — default Return — as soon as it arms.
The long comment in `1200_flightaxis_plane` sets out the whole trade.

**Then calibrate the transmitter in QGC, exactly as for real hardware:** Vehicle Setup → Radio.
PX4 does not act on `input_rc` until `RC_MAP_*` and the per-channel `RC_MIN`/`RC_TRIM`/`RC_MAX`
exist, so an uncalibrated transmitter is inert even with `COM_RC_IN_MODE` correct. One quirk
survives into calibration: the **InterLink sends ch2 inverted** relative to what PX4 expects, so
reverse channel 2 in the calibration. ArduPilot's FlightAxis backend forces `RC2_REVERSED = 1`
for the same reason. Calibrating without reversing it gives inverted pitch.

#### The very first run

On a genuinely first run there is no `parameters.bson` anywhere to seed from (see
[A working directory per model](#a-working-directory-per-model) below), so PX4 comes up with
magnetometer and gyro *ids* but no offsets and reports `Compass 0 fault`, and then
`Airspeed invalid` as a consequence. Nothing has gone wrong. Calibrate once in QGC —
**Vehicle Setup → Sensors**, compass then gyroscope — and it is stored in that working
directory for good, and is what every later directory seeds from.

#### PX4's in-flight sensor learning is turned off for you

The four shipped airframes set `SENS_IMU_AUTOCAL`, `SENS_MAG_AUTOCAL` and `IMU_GYRO_CAL_EN`
to `0`. All three default to `1`, and on a real aircraft that is correct: the sensors drift,
PX4 learns the bias in flight and saves it. Here every sensor is synthesised from RealFlight's
state and has no bias to find, so what PX4 learns instead is its own estimator transients —
and it writes them to `parameters.bson` as permanent calibration.

Measured in one session: an accelerometer offset moving from zero to
`+0.308, -0.173, +0.040 m/s²`. A 0.36 m/s² bias dead-reckons to roughly 21 m/s over a minute,
which presents as the aircraft wandering across the QGC map while it sits perfectly still in
RealFlight. The offsets accumulate across sessions rather than resetting with each run, so the
symptom looks intermittent and unrelated to whatever was last changed.

Nothing to do for a new working directory. If you have flown one before this was set — the
giveaway is `ACC 0 offset committed` in the console — the saved offsets are still there and
outrank the airframe; clearing them is in
[RUNNING.md §7](RUNNING.md#7-troubleshooting) under the sensor-learning entry.

#### Stored values against the airframe's defaults

If a stored value seems to be overriding the airframe, reset that parameter rather than the
store: `param show -c` marks explicitly saved values with `x +`, and those outrank the
airframe's `param set-default`. `param reset PWM_MAIN_FUNC*` clears a specific family.
Deleting `parameters.bson` also clears your radio calibration, sensor calibration and flight
mode assignments, which nothing in the airframe will put back.

### A working directory per model

Add one line, `PX4_FLIGHTAXIS_ROOTFS`, and that model gets its own directory:

```bash
cd ~/PX4-Autopilot

PX4_FLIGHTAXIS_ROOTFS=~/sitl/PX4/Plane \
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 \
PX4_HOME_LON=175.7433944 \
PX4_HOME_ALT=48.0 \
PX4_HOME_YAW=235 \
make px4_sitl_nolockstep flightaxis_plane
```

**There is no separate first-run command.** That is the whole invocation, every time. The
directory is created if it is missing, so the first run comes up on the airframe's defaults
and every later run picks up whatever you have tuned, saved and uploaded since. Stop with
Ctrl-C; tomorrow, run the same line again.

Three things end up in there, and they are the three that make the separation worth having:

```
~/sitl/fa-rootfs/quadplane/
├── parameters.bson           # tuning, calibration, flight modes
├── parameters_backup.bson    # PX4's own copy, restored if the primary is unreadable
├── dataman                   # missions, geofence, rally points
├── log/                      # flight logs, one file per flight
├── etc -> …/build/px4_sitl_nolockstep/etc    # symlinks back into the build tree
└── test_data -> …/PX4-Autopilot/test_data
```

`log/` fills one file per flight rather than one per boot: all four airframes set
`SDLOG_MODE 0` ("when armed until disarm"), against rcS's own `1` ("from boot until disarm"),
which would otherwise record every idle stretch before you ever armed. It is
`@reboot_required`, so it is a startup default and not something to change at the `pxh>` prompt.

`parameters.bson` holds only what has actually been set — a fresh fixed-wing directory is
around twenty entries. Every parameter compiled into the firmware still *appears* in QGC and
in `param show`, `VT_*` and multicopter ones included, because `px4_sitl_nolockstep` is a
single binary carrying every vehicle type. Their presence in the list says nothing about your
airframe; nothing reads them unless the airframe selects that vehicle type. `param show -c`
lists only the stored ones, which is the honest view of what the directory is keeping.

A directory with no `parameters.bson` of its own is seeded from
`build/px4_sitl_nolockstep/rootfs` on the way in, so a new one starts with the sensor and
radio calibration you already have. Only when the file is absent — it never overwrites what
you have since saved — and only that file, so no mission comes with it. What survives is then
`rcS`'s decision: a different airframe makes it run `param reset_all`, which keeps `RC*`,
`CAL_*` and `COM_FLTMODE*` and drops the rest. Set `PX4_FLIGHTAXIS_SEED=0` to start empty and
calibrate from scratch instead.

Without that seed the first boot has magnetometer and gyro *ids* but no offsets, which raises
`Compass 0 fault`; the airspeed validator then rejects the pitot as well, since it checks it
against an EKF whose yaw the bad compass has already spoiled. The fix is the same one as on a
first-ever run — calibrate once in QGC, see [The very first run](#the-very-first-run) — and
[RUNNING.md §7](RUNNING.md#7-troubleshooting) goes through it in full.

Give every model one and they stop reaching into each other's:

```bash
PX4_FLIGHTAXIS_ROOTFS=~/sitl/fa-rootfs/plane      ... flightaxis_plane
PX4_FLIGHTAXIS_ROOTFS=~/sitl/fa-rootfs/quad       ... flightaxis_quad
PX4_FLIGHTAXIS_ROOTFS=~/sitl/fa-rootfs/quadplane  ... flightaxis_quadplane
PX4_FLIGHTAXIS_ROOTFS=~/sitl/fa-rootfs/heli       ... flightaxis_heli
```

Leave the line out and the model runs in `build/px4_sitl_nolockstep/rootfs` instead — the
directory every model shares if it is not given one of its own. Nothing else about the
invocation changes:

| | |
|---|---|
| Run in the shared directory | omit `PX4_FLIGHTAXIS_ROOTFS` |
| Run in its own directory | set `PX4_FLIGHTAXIS_ROOTFS` |

Pick one and apply it to every model. Sharing is what costs you: the working directory holds
`parameters.bson`, dataman and `log/`, so models that share one share all three. A quadplane
mission is still loaded when you next start the plane and gets rejected by feasibility
checking; and PX4, seeing the saved `SYS_AUTOSTART` disagree with the model it is starting,
runs `param reset_all` — which keeps `RC*`, `CAL_*` and `COM_FLTMODE*` but discards tuning.
Give one model its own directory and leave the rest sharing, and the rest still do this to
each other.

The path is yours, and it is not tied to the model name: two configurations of one aircraft
are just two directories — `~/sitl/fa-rootfs/heli-3d` and `~/sitl/fa-rootfs/heli-scale` both
run `make px4_sitl_nolockstep flightaxis_heli` and keep separate parameters. Relative paths
are resolved before PX4 starts, so `PX4_FLIGHTAXIS_ROOTFS=./fa-plane` behaves as you would
expect.

Inside or outside the PX4 tree both work, and the difference is what happens when you clean
the build:

```bash
# Outside the PX4 tree. Survives `make clean`, `rm -rf build/` and reinstalling,
# so saved parameters, missions and logs outlive the build directory.
PX4_FLIGHTAXIS_ROOTFS=~/sitl/fa-rootfs/plane

# Inside it. Cleaned away with the build - which is what you want for a scratch
# directory, and what you do not want for one you have tuned.
PX4_FLIGHTAXIS_ROOTFS=~/PX4-Autopilot/build/px4_sitl_nolockstep/rootfs_plane
```

`make` still has to be run from the PX4 tree either way; only the working directory moves.

Multi-instance is unaffected: the variable moves the working directory and nothing else, so it
composes with `PX4_FLIGHTAXIS_INSTANCE`, and leaving it unset gives the per-instance defaults
exactly as before — see [RUNNING.md §6](RUNNING.md#6-multiple-instances).

If PX4 refuses to start with `PX4 server already running for instance 0`, a `px4` process for
that instance really is still alive — find it and stop it:

```bash
pgrep -a px4
kill <pid>          # then re-run
```

The message is not a stale file. PX4 takes an `fcntl` write lock on `/tmp/px4_lock-<instance>`
and probes it with `F_GETLK`, and the kernel drops an advisory lock as soon as the holder dies,
so a process that has exited cannot produce this. Deleting the lock file does not clear
anything; it only lets a second instance start alongside the first. Also check for a stray
`bin/px4` still holding TCP 4560.

### Restarting on a RealFlight respawn

A RealFlight respawn — spacebar, or the automatic reset after a crash — force-disarms PX4 and
then shuts it down; `sitl_run.sh` brings PX4 and the bridge back up together in the same
terminal. This happens on its own, with nothing to add to the command:

```bash
cd ~/PX4-Autopilot

PX4_FLIGHTAXIS_ROOTFS=~/sitl/fa-rootfs/plane \
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 \
PX4_HOME_LON=175.7433944 \
PX4_HOME_ALT=48.0 \
PX4_HOME_YAW=235 \
make px4_sitl_nolockstep flightaxis_plane
```

`PX4_FLIGHTAXIS_RESTART_ON_RESET` turns it off if you need the old behaviour:

| | |
|---|---|
| Unset — the default | PX4 and the bridge restart on every respawn |
| `0` | one run per invocation; a respawn leaves the session running, with the consequences below |

Any other value leaves the restart on.

**Bring the aircraft to rest first.** Throttle to zero and disarm, then press spacebar. The
bridge only learns about a respawn once RealFlight has already moved the model, so a reset
taken at power puts the aircraft on the runway with the prop still turning and it rolls away
before anything here can intervene — how far depends on how much propulsion the model has.

**How the bridge notices.** Not from `m_resetButtonHasBeenPressed`: that field does not fire on
a spacebar respawn. Instrumented over two respawns in one session, the reported position jumped
92.3 m and then 120.9 m in a single frame while the flag stayed `0` both times. The
discontinuity is the signal instead, and it is unambiguous — frames arrive about 4 ms apart, so
92 m in one of them is 23 km/s. The test compares the distance moved against the distance the
reported velocity could actually have covered, rather than against a fixed threshold, so it
stays correct when the glitch compensator has just absorbed a network stall and a legitimate
frame really does span two seconds of flight. On detection the bridge prints:

```
[flightaxis_bridge] aircraft teleported 92.3 m (RealFlight reset) - re-anchoring
```

**What a respawn does regardless of this option.** The position anchor is re-captured, so PX4 is
told the model is back at the point it entered the world. The heading datum is *not* re-derived
— it is latched once per session, because it defines the frame every other quantity is expressed
in (see [Home position and heading](#home-position-and-heading)).

**And what it does when the restart is disabled.** The bridge additionally asks PX4 to reset its
position outright — `MAV_CMD_EXTERNAL_POSITION_ESTIMATE` carrying the home coordinates and a
claimed 0.5 m accuracy, a sub-metre figure being what makes EKF2 treat the message as a hard
reset rather than one more fusion update. It is offered every 200 ms for five seconds rather
than fired once: at the instant of the respawn the vehicle is airborne and still fusing GNSS, so
the request comes back `TEMPORARILY_REJECTED`, and the state EKF2 will accept it in — dead
reckoning, or on the ground and not fusing GNSS — arrives a second or two later, once the
rejected position has pushed the estimator there itself.

**Why a restart is on offer at all.** Re-anchoring makes the bridge report the model at home
again; it does not make PX4 believe it. A freshly started session is always correct because an
estimator with no state accepts whatever it is given. A respawn is not, because EKF2 has
converged on the flight that just ended and reads a hundred-metre teleport as a lying GPS rather
than as a moved aircraft — so it rejects the position and dead-reckons, which is what the pilot
sees as QGC still flying the old trajectory after the model is back on the runway.

There is no way to bypass the estimator in PX4 SITL. ArduPilot has one — `EKFType::SIM`, an AHRS
backend that takes the simulator's state as the answer, which is why the same respawn is seamless
there. PX4's equivalent exists only in HITL, where `mavlink_receiver` publishes
`HIL_STATE_QUATERNION` straight onto `vehicle_attitude` and `vehicle_local_position`; in SITL
that same message reaches `_gpos_ground_truth_pub` and nothing else. The position-reset request
above is a negotiation with EKF2 and depends on it entering a state where it will listen; a
restart does not negotiate, and that is the whole of the difference between them.

**What it costs.** A gap of a few seconds while both sides come back, a new log file per
respawn, and a ground-station reconnect each time; teardown also prints a burst of `Broken pipe`
lines and a pair of sensor `TIMEOUT!` errors, which are expected and not faults. PX4 stays in
the foreground throughout, so the `pxh>` prompt survives the loop and Ctrl-C still ends the
session. [RUNNING.md §6.1](RUNNING.md#61-px4_flightaxis_restart_on_reset) goes through the whole
list.

Under HITL, set it to `0`: `hitl_run.sh` runs the bridge with no restart loop around it, so a
respawn would end the session and leave the board in HIL — see [HITL.md](HITL.md).

### Finding which RealFlight channel a surface is on

`PX4_FA_DUMP_CHANNELS=<hz>` prints every channel's current value and its travel since the
dump began:

```bash
PX4_FA_DUMP_CHANNELS=1 PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_quadplane
```

```
[flightaxis_bridge] channels (RF ch1-12), ARMED:
    ch1  rf0  = 0.512  travel 0.240
    ch2  rf1  = 0.500  travel 0.000  <- not moving
```

This answers the one question the bridge cannot answer from its own configuration: the JSON
map, the `PWM_MAIN_FUNC*` assignments and `actuator_outputs` in the ulog can all be correct
while a surface sits motionless, because the RealFlight **model** decides what channel N
drives. Arm, move one surface at a time, and the row whose travel grows is the channel it is
on. A channel with travel while the surface is still means the bridge is sending to a channel
the model does not use — fix that in RealFlight, or repoint `PWM_MAIN_FUNC<N>`.

Note that **while disarmed every mapped channel is pinned to its `disarm` value and ignores
the sticks**, so surfaces not moving on the ground is expected, not a fault. Arm first.

## RealFlight setup

Do the network setup first, and only once: finding the Windows machine's address for
`PX4_FLIGHTAXIS_IP` (`ipconfig`), opening the Windows firewall to TCP 18083, and verifying the
link is reachable before you launch anything are all in
[RUNNING.md §1](RUNNING.md#1-network-setup-do-this-first), step by step. Skipping it is the
usual reason a first run stops at `RealFlight FlightAxis not reachable`.

Once per RealFlight installation:

- Enable **"RealFlight Link Enabled"**, under **Simulation → Settings… → Physics → Quality**
  (press ESC to reach the menu). On RealFlight 8/9 it sits directly under **Settings →
  Physics**, and on some builds the Quality preset has to be **Custom** before the checkbox
  unlocks. RealFlight then listens on TCP 18083. The in-tree strings quote shorter versions of
  this path — [RUNNING.md §1.1](RUNNING.md#11-enable-realflight-link-in-realflight) reconciles
  them.
- In the same Physics settings, set **"Automatic Reset Delay(sec)"** to `2.0`, and set both
  **"Pause Sim When in Background"** and **"Pause Sim when in Menu"** to **No** — otherwise
  RealFlight stops feeding the bridge the moment it loses focus.
- Restart RealFlight after changing these.
- Wired network or same-host only. Every SOAP exchange is a blocking round trip, so the step
  rate is bounded by network latency rather than by bandwidth; WiFi latency and jitter drag it
  far below what the bridge needs.
- Leave the physics speed multiplier at 1.0; the bridge warns if it is not. Take that warning
  as the diagnosis and stop reading the rest of the log, because a stalled physics clock
  cascades into failures that all look like something else:

  ```
  WARNING: RealFlight physics speed multiplier is 0.00 (set it to 1.0)
  WARNING: realtime factor 0.04 (physics s per wall s)
  Preflight Fail: vertical velocity unstable / heading estimate not stable
  ERROR [vehicle_magnetometer] MAG #0 failed:  TIMEOUT!
  ERROR [vehicle_air_data]     BARO #0 failed:  TIMEOUT!
  [flightaxis_bridge] ExchangeData failed, retrying ... / exiting
  WARN  [simulator_mavlink] Failed sending mavlink message: Broken pipe   (x hundreds)
  ```

  Only the first line is a cause. PX4 timestamps the sensor stream with its own clock on
  arrival, so a stopped physics clock means it integrates real time over a simulation that
  is not advancing — hence the unstable-estimate failures. The sensors then time out, the
  bridge gives up on RealFlight and exits, and every `Broken pipe` after that is just PX4
  writing to a socket nobody holds. Fix the multiplier, un-pause the sim, and release the
  model if it reports `*** RealFlight model is LOCKED ***`.

Once per aircraft:

- In the RealFlight aircraft editor: strip expo/mixes/gyros, max servo speed, one channel
  per actuator.
- The RealFlight model's channel order must match the airframe's `PWM_MAIN_FUNC*` assignments,
  which is where the ordering is set; the `models/<name>.json` map is identity and stays that
  way — see [Model JSON](#model-json-channel-maps) below.

## Home position and heading

The bridge anchors the RealFlight world to a geodetic home position, overridable by environment
variable:

| Variable | Default |
|---|---|
| `PX4_HOME_LAT` | `47.397742` |
| `PX4_HOME_LON` | `8.545594` |
| `PX4_HOME_ALT` | `488.0` (metres) |
| `PX4_HOME_YAW` | unset — RealFlight's world used as-is |

```bash
PX4_HOME_LAT=-37.7304917 PX4_HOME_LON=175.7433944 PX4_HOME_ALT=48.0 PX4_HOME_YAW=235 \
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_plane
```

The barometer is derived from this same altitude datum rather than from RealFlight's scenery
elevation, deliberately — see spec §6.

`PX4_HOME_YAW` is the true heading (degrees) the aircraft should **start** on. RealFlight's world
north is arbitrary, so the bridge reads the model's actual attitude on the first frame and rotates
the whole RF→NED mapping by the difference — exactly the way `PX4_HOME_ALT` shifts the altitude
datum without RealFlight knowing about it. The rotation it derives is logged at startup.

The rotation is applied to *every* world-frame quantity — attitude, position, velocity and wind —
so heading and direction of travel stay consistent and EKF2 sees no contradiction. Body-frame
sensors (accelerometer, gyro) and all Down components are untouched. Leaving the variable unset
rotates nothing, which is also what ArduPilot's FlightAxis backend does: it accepts a heading in
`--custom-location` but its `SIM_FlightAxis` overwrites the attitude from RealFlight on the first
frame, so the value has no effect there.

The datum is **latched once per session**, on the first frame, and is not re-derived when
RealFlight respawns the model. It defines the frame every other quantity is expressed in, so
re-deriving it mid-session would rotate the world out from under a position and velocity history
that were recorded in the old frame. A respawn re-captures the position anchor only — see
[Restarting on a RealFlight respawn](#restarting-on-a-realflight-respawn).

## Model JSON: channel maps

Each `models/<name>.json` maps RealFlight transmitter channels (`rf`) to PX4 actuator outputs
(`px4` = an index into `HIL_ACTUATOR_CONTROLS.controls[]`).

> ### The one rule
>
> **RealFlight channel N is driven by PX4 output channel N — all twelve of them.**
>
> Every shipped map is a pure identity map covering **ch1–ch12**: `rf` and `px4` are equal on
> every row, and every row from `rf0` to `rf11` is present. No exceptions, no renumbering, and no
> `"reverse": true` anywhere.
>
> What each channel *carries* is decided solely by `PWM_MAIN_FUNC<N>` (SITL) / `HIL_ACT_FUNC<N>`
> (HITL). To change what comes out on a RealFlight channel, change that parameter — not the JSON.

**Twelve is the ceiling, and it is FlightAxis's.** RealFlight's `ExchangeData` SOAP call carries
exactly twelve channel values, so ch12 is as far as anything can go. PX4 itself would reach 16.
Channels with no FUNC assigned sit at a steady neutral `0.5`, so mapping all twelve costs nothing
on an aircraft that uses four. To put flaps, gear, a gripper or lights on a spare channel it is a
parameter change only:

```
param set PWM_MAIN_FUNC9  406      # RC Flaps     -> RealFlight channel 9
param set PWM_MAIN_FUNC10 400      # Landing Gear -> RealFlight channel 10
```

The channel order is decided in the airframe instead, by `PWM_MAIN_FUNC<N>` (SITL) or
`HIL_ACT_FUNC<N>` (HITL) — the same place ArduPilot puts it (`SERVOn_FUNCTION`) and PX4's own
Gazebo bridge puts it (`SIM_GZ_*_FUNC*`). One channel numbering therefore holds from SITL through
HITL to the physical output pinout:

- `PWM_MAIN_FUNC<N>` selects which allocator output lands on `controls[N-1]`, so choosing the
  FUNC values *is* choosing the channel order.
- Function `101+k` is `CA_ROTOR<k>` — a motor, range `[0,1]` → `"scale": "unipolar"`.
- Function `201+k` is servo output `k` — on fixed-wing and VTOL airframes that is `CA_SV_CS<k>`,
  a control surface, range `[-1,1]` → `"scale": "bipolar"`. On the helicopter (`CA_AIRFRAME 11`)
  the servo block is not control surfaces at all: `201` is the tail rotor and `202`–`204` are the
  swashplate servos, still bipolar.
- The allocator emits **motors before servos**, so on mixed airframes the motors take the low
  function numbers no matter where they sit on the transmitter. That mismatch has to be absorbed
  somewhere; it is absorbed by the FUNC list, which is why the shipped FUNC assignments look
  scrambled — e.g. `1202_flightaxis_quadplane` sets `PWM_MAIN_FUNC1..9` to
  `201, 203, 105, 204, 101, 102, 103, 104, 202`.
- If your RealFlight model uses a different channel order, **change the FUNC params and keep the
  JSON an identity map.** Re-scrambling the JSON splits the ordering across two files.

What remains in the JSON is what is genuinely not ordering and cannot be expressed as a FUNC
value — `scale` (PX4 normalises motors to `[0,1]` but servos to `[-1,1]`, while RealFlight always
wants `[0,1]`), `disarm`, and the option transforms:

- **`scale` is the one column a FUNC change can force you to touch.** `unipolar` is correct if and
  only if that channel's FUNC is a **Motor**; `bipolar` is correct for everything else. Both
  mistakes are silent — a motor read as `bipolar` idles at half throttle and never stops, a
  surface read as `unipolar` only ever deflects one way.
- **Reversal is not in the JSON at all.** No shipped model contains `"reverse": true`. Use the
  `PWM_MAIN_REV` bitmask (bit `N-1` = channel `N`), which *is* honoured in SITL, or flip the servo
  in the RealFlight model. Note `PWM_MAIN_MIN`/`MAX` are **not** honoured in SITL — `PWMSim`
  overwrites them — so endpoint trimming has no effect there.
- `"disarm"` is the value sent while PX4 is disarmed. **`-1`, which is also the default when the
  key is omitted, means "hold the last output"** — correct for a plain surface, wrong for a
  motor, so every motor row sets `"disarm": 0.0` explicitly. On any slot `HeliDemix` or
  `Rev4Servos` rewrites, `get_FAbridge_params.py` rejects hold-last outright and demands a
  value; the bridge applies those post-passes to a scratch copy, so holding is safe as the
  code stands, but nothing in a model JSON can tell whether that is still true. Every row in
  `heli.json` therefore carries an explicit `"disarm"` (`0.5` for the swash and tail servos,
  `0.0` for the rotor).
- `"UnmappedDefault"` is sent on every RealFlight channel the map does not mention.
- **Duplicate `rf` or `px4` indices abort the bridge at startup** with a message naming both
  offending entries, rather than silently letting one win.

### Options

`"Options"` is a list of flags; `get_FAbridge_params.py` flattens it to a bitmask.

| Option | Effect |
|---|---|
| `ResetPosition` | issue `ResetAircraft` on startup, so every run begins from a known state. On by default in all four shipped models. |
| `Rev4Servos` | swap RealFlight channels 1–4 with 5–8 wholesale, for RF models built that way. **Do not enable it on a model whose FUNC params already produce the right order** — every shipped model, `quadplane.json` included — as it can only swap a correct order into a wrong one. |
| `HeliDemix` | **Off by default** — see "Swash mixing" below. Converts the three swash servo outputs back into the roll/pitch/collective triple a CCPM RealFlight model expects (`roll=(s1−s2)/1.732`, `pitch=((s1+s2)/2−s3)/1.5`, `col=(s1+s2+s3)/3`). The two divisors normalise the raw differences to unit gain — without them the cyclic saturates at 0.577 of commanded roll and 0.667 of pitch. They are exact only for the swash geometry `1203_flightaxis_heli` forces via `CA_SP0_ANG*` = 300/60/180, which is why that airframe pins the angles and the arm lengths. **The divisors are ours; ArduPilot's `SIM_FlightAxis.cpp:348-350` has no equivalent** and leaves roll and pitch in the ratio 2/√3 on the 120° head both projects use. The channel *order* follows ArduPilot (swash 1–3, tail 4, RSC 8); the demix *gains* deliberately do not. |
| `HeliDemix` + `Rev4Servos` | **Mutually exclusive — `get_FAbridge_params.py` rejects the pair and the bridge will not start.** `Rev4Servos` runs first, so the demix would read the swapped-in rf4–6 instead of the swash servos and emit a constant roll 0 / pitch 0 / collective 0.5 — a swashplate frozen dead centre on an armed aircraft, with nothing in the output to show it. Refused at load time because the failure is silent; the full account is in [RUNNING.md §2.4](RUNNING.md#when-to-re-enable-helidemix). |
| `SilenceFPS` | suppress the periodic `exchanges=… rtf=… glitches=…` line on stderr. **On in all four shipped models** — it prints once a second forever and buries everything else in the terminal. The alarms still print: each swallowed glitch gets its own `glitch 0.62s` line and an out-of-range realtime factor still warns, both independently of this option. Drop it from `Options` if you want the running heartbeat back. |

### Swash mixing (helicopter)

A collective-pitch heli needs exactly **one** CCPM mix between roll/pitch/collective and the
three swash servos — in PX4's allocator, in the bridge, or in the RealFlight model, but in
exactly one of them. Two mixes in series do not cancel, they compose, and roll and pitch
cross-couple badly.

**This integration puts the mix in PX4**, so `HeliDemix` is off in `heli.json` and the bridge
sends the allocator's three swash **servo positions** to RealFlight untouched. `heli.json` is
a pure identity map for the heli exactly as it is for the other three vehicles — RealFlight
channel N carries PX4 output channel N, literally.

That means **the RealFlight heli model must be wired non-mixed / direct-servo**: turn its own
CCPM off in the aircraft editor and map each channel straight to its servo, the same way you
would wire a fixed-wing model's ail/ele/thr/rud to channels 1–4. This also matches ArduPilot's
default: `SIM_FlightAxis.cpp:129-130` only enables ArduPilot's own demix when the frame name
contains the substring `helidemix`.

**Re-enable `HeliDemix` only if your model has a CCPM head whose mixing cannot be disabled.**
Add `"HeliDemix"` back to `Options` in `heli.json` (bitmask 9 → 13) and change nothing else.
Never enable the model's mixing and the bridge's demix at the same time.

With PX4 as the sole mixer, `CA_SP0_ANG*` describes the **physical** swash of your model. The
airframe sets 300/60/180 — a 120° head with the odd servo aft, matching ArduPilot's default.
If your model's third servo is at the front, use 120/240/0. That freedom exists only while
`HeliDemix` is off — re-enabling it pins the angles back to 300/60/180, since the demix divisors
are derived from that geometry alone.

The heli rate gains, collective curve and yaw compensation in `1203_flightaxis_heli` are a
generic collective-pitch starting point, not a tune for any particular machine — expect to
retune them for your model.

Full derivation, per-model tables, and the heli traps: spec §5.

## Adding a new aircraft

Three steps. **Number your airframe in the
1204–1219 range** — those ids are reserved for exactly this and the installer and
uninstaller leave them, and your model JSONs, strictly alone:

1. New `Tools/simulation/flightaxis/flightaxis_bridge/models/<name>.json` (channel map — see
   above, and spec §5).
2. New airframe script `12xx_flightaxis_<name>`, whose `PWM_MAIN_FUNC*` assignments set the
   channel order — choose them so that `controls[]` comes out in your RealFlight model's own
   channel order, which lets the JSON stay an identity map.
3. **Register that airframe in
   `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`.** Skipping this is the classic
   mistake: everything builds cleanly and the airframe simply never reaches the ROMFS, so PX4
   starts with no matching `SYS_AUTOSTART`.

There is no model list to edit: `sitl_targets_flightaxis.cmake` globs
`*_flightaxis_*` out of the airframes directory and derives the targets from it, so the
airframe file in step 2 is what creates both `flightaxis_<name>` and
`flightaxis_hitl_<name>`. A missing `models/<name>.json` is reported as a CMake warning at
configure time rather than failing at launch.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `RealFlight FlightAxis not reachable at <ip>:18083` | The pre-flight check failed before anything started. RealFlight not running, the link option not enabled (see [RealFlight setup](#realflight-setup)), the wrong IP in `PX4_FLIGHTAXIS_IP`, or a Windows firewall blocking TCP 18083. Finding the address and opening the firewall are [RUNNING.md §1.2](RUNNING.md#12-find-the-windows-machines-ip) and [§1.3](RUNNING.md#13-open-the-windows-firewall). |
| `get_FAbridge_params.py failed for models/<model>.json` | The model JSON is missing, malformed, or names an unknown `Options` / `scale` value. The exact reason is printed just above. |
| `bad channel map: RealFlight channel rfN is mapped twice` <br> `bad channel map: PX4 control index px4[N] is mapped twice` | Two rows in the model JSON share an `rf` or a `px4` index. The message names both entries; fix the JSON. The bridge refuses to start rather than silently letting one win. |
| `WARNING: RealFlight physics speed multiplier is <x> (set it to 1.0)` | RealFlight is running fast or slow motion. All timing and sensor synthesis assume real time — reset the multiplier in RealFlight. |
| `glitch 0.35s` lines appearing | The bridge absorbed a physics-time jump (network hiccup) so it does not reach EKF2 as a time jump. Occasional lines are benign; a steady stream means the link cannot keep up — go wired, or move the bridge closer to the RealFlight machine. |
| **Hangs silently at `waiting for PX4 on TCP 4560`** | The bridge is up and waiting, but PX4 never connected. Usually PX4 exited at startup — scroll back for its error. The most common cause is the missing airframes `CMakeLists.txt` registration (see step 3 of [Adding a new aircraft](#adding-a-new-aircraft)): the airframe is absent from the ROMFS, so `SYS_AUTOSTART` matches nothing. Also check that nothing else already holds TCP 4560. |
| Aircraft twitches or flies inverted on one axis | Channel order mismatch. First confirm the JSON is still an identity map (`rf` == `px4` on every row); if it is, the fix is in the `PWM_MAIN_FUNC*` block of the matching airframe script, which must put `controls[]` in your RealFlight model's channel order (see [Model JSON](#model-json-channel-maps)). Inverted on exactly one axis with the right surface moving is a direction problem, not an ordering one: set that channel's bit in `PWM_MAIN_REV` (bit `N-1` for channel `N`) or flip the servo in the RealFlight model. |
| Heli has no left yaw at all; the tail sits on its lower stop | The tail row must be `"scale": "bipolar"` with `"disarm": 0.5`. `1203_flightaxis_heli` uses `CA_AIRFRAME 11` ("tail Servo"), so the tail is a servo on `[-1,1]`. Under `CA_AIRFRAME 10` it would be a motor clamped to `[0,1]` and the whole negative half of the yaw command would be clipped away. |
| `battery source:` names a fuel tank, and the percentage looks wrong | The line below it prints the raw reading beside the reference it is measured against — `tank: 47.500 raw (m-fuelRemaining-OZ), reference 95.000 -> 50%`. FlightAxis names the field in ounces but nothing upstream consumes it, so the unit rests on the name alone; the bridge divides by the largest value it has ever seen and is therefore correct for ounces, percent or a 0..1 fraction alike. A full tank printing `100.0` or `1.000` rather than a tank size tells you which it actually is. The reference is the largest reading of the session, so one below the true tank size only means no full tank has been seen yet — a refuel or a RealFlight reset re-arms it upward. It re-arms *downward* too, on a confirmed step up to a level still under the old reference, which is what swapping one internal-combustion model for a smaller-tanked one looks like; without that, a 10 oz tank behind a 40 oz one would read 25% while brim full and trip the low-battery failsafe. Burning fuel only ever falls, so it can never re-arm the reference down onto itself. |
| Bridge exits with `PX4 link lost - shutting down` | Expected, not a fault: a dead PX4 link is terminal by design. PX4 exited, the board rebooted, or the cable was pulled. Restart both sides — the bridge deliberately does not re-accept a fresh PX4 whose clock starts at zero. |

## Credits / references

Parts of this bridge are ported or adapted from upstream projects, and parts are reused
verbatim. Every file's provenance, the upstream project it came from, and its licence are
recorded in [COPYRIGHT.md](COPYRIGHT.md).

## License

Copyright © 2026 Evangels Brilliant Dasmasela.

**GPLv3 or later** — see [LICENSE](LICENSE), and [COPYRIGHT.md](COPYRIGHT.md) for
per-file provenance.

The bridge's SOAP client, physics-time handling and RealFlight→NED conversions are
literal ports of ArduPilot's GPLv3 `SIM_FlightAxis.{h,cpp}`, so the combined work is
GPLv3. The files reused from BSD-3-Clause upstreams keep their own notices, which is
GPL-compatible.

This covers the files in this repository only. The bridge is a standalone executable
that talks to PX4 over MAVLink — installing it does not relicense your PX4 checkout.
