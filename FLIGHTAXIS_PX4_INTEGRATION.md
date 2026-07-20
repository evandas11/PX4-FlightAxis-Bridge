# FlightAxis → PX4 Integration Specification

**RealFlight (FlightAxis Link) as a PX4 SITL simulator, integrated the way PX4's in-tree simulator bridges are: files live in the PX4 tree, get compiled by the PX4 make system, and launch with one `make` command.**

Version: 3.4 — July 2026
- v3.4: documentation corrections only — no behaviour change. Six statements that had drifted
  from the shipped code or from upstream are corrected: `RAW_RPM` *is* handled by
  `SimulatorMavlink`, and the reason for dropping it from the HITL profile is restated as the
  SITL/HITL receive-path asymmetry (§8.2); RC passthrough is documented as implemented via
  `RC_CHANNELS` at 50 Hz rather than as future work (§8.2, §11.1); ~250 Hz is stated as this
  bridge's design assumption rather than as RealFlight's measured frame rate (§7); the abridged
  `sitl_run.sh` sketch regains the `instance` assignment and `px4 -i` it had lost (§4); the
  `201+k` = `CA_SV_CS<k>` gloss is qualified as fixed-wing/VTOL only, since `CA_AIRFRAME 11`
  puts the tail rotor and swashplate there (§5); the `CA_R_REV` escape hatch from motor
  `[0,1]` clamping is described, along with why `CA_AIRFRAME 11` remains the better answer (§5);
  and the on-ground accel override is brought in line with the code's additional triggers,
  minimum `dt` and synthetic clamp (§6).
- v3.3: `PX4_HOME_YAW` heading datum (§6); the multi-instance id is threaded from `sitl_run.sh` to both ends (§4, §11.3).
- v3.2: reconciled with the shipped implementation a second time. The helicopter moved to `CA_AIRFRAME 11` (tail servo) with a bipolar tail, real rate gains, a collective curve and yaw compensation (§5, §9, §11); the `HeliDemix` over-gain is corrected in the bridge (§5, §11); `fields_updated` is sub-rated rather than `0x1FFF` on every message (§8.2); a realtime-factor monitor was added (§7); a dead PX4 link is terminal (§8.2); the quadplane's `FD_ACT_*` overrides were removed because the detector cannot fire in v1.16 SITL at all (§11); HITL support and a ROS 2 offboard path landed (§8.2); rotor-geometry magnitudes are normalised away by the allocator and only the symmetry matters (§9); airframe parameter forcing extended beyond `EKF2_GPS_DELAY` (§9); per-file line-number citations replaced with function and file names, which do not rot.
- v3.1: reconciled with the shipped implementation. Registration moved to the **PX4 v1.16 pattern** (`src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake`, §3); baro/pitot units and the baro datum corrected (§6); DISTANCE_SENSOR and RAW_RPM documented, RC passthrough marked not implemented (§8.2); model-JSON example replaced with the real shipped channel maps and the rule that generates them (§5); airframe example marked abridged (§9); pitfalls ledger pruned of stale rows and extended with the fixes actually made (§11).
- v3.0: restructured around PX4's **standalone-bridge pattern** (`Tools/simulation/` standalone bridge + build-system registration + `sitl_run.sh` + per-model JSON) — build it with the PX4 you already have, no PX4 stack modification.
- v2.0: all frame conversions & timing taken from the upstream FlightAxis implementation (§6–§7 — unchanged, still the ground truth).
- v1.0: external-bridge protocol groundwork.

> **This is a design document — it explains *why*.** For installation, day-to-day
> use, prerequisites, and troubleshooting, see [`README.md`](README.md), which is
> the **user manual and the authoritative document wherever the two disagree**.

Supported PX4: **v1.16.0**. See §3 for v1.13–v1.15.

---

## 1. The Pattern

PX4 already has a slot for an out-of-tree simulator that speaks Simulator-MAVLink, and
FlightAxis fills it exactly. The pieces:

| Piece | FlightAxis (this spec) |
|---|---|
| Bridge app | `Tools/simulation/flightaxis/flightaxis_bridge/` (standalone C++, own CMake) |
| Sim link | SOAP/HTTP TCP **18083** (RealFlight) |
| PX4 link | MAVLink HIL over **TCP 4560** (`px4_communicator.cpp`) |
| Sim launcher | `FA_check.py` (RealFlight runs on Windows — bridge just connects; script only pings 18083) |
| Model config | `models/<name>.json` (channel map, scale, options) |
| Runner | `Tools/simulation/flightaxis/sitl_run.sh` |
| Make registration | `simulator_mavlink/sitl_targets_flightaxis.cmake` (§3) |
| Build target | `make px4_sitl_nolockstep flightaxis_plane` |
| Airframe script | `ROMFS/.../init.d-posix/airframes/*_flightaxis_plane` etc. |

Key consequences:

- **No PX4 source modification.** PX4 sees a standard Simulator-MAVLink simulator on TCP 4560. The bridge is just compiled *alongside* PX4 by its make system (ExternalProject).
- **`nolockstep` board already exists** — it is the board for simulators whose clock cannot be stepped by PX4, which is exactly RealFlight's case. Nothing new to add in `boards/`.
- Integration is visible on the console from the runner + bridge. The block below is an
  **illustrative sketch of the shape of that output** — the counter values are representative.
  ```
  $ make px4_sitl_nolockstep flightaxis_plane
  ...
  FlightAxis setup
  [flightaxis_bridge] connecting to RealFlight at 192.168.10.1:18083
  [flightaxis_bridge] controller injected, aircraft reset
  [flightaxis_bridge] waiting for PX4 on TCP 4560 ... connected
  [flightaxis_bridge] exchanges=247.3/s loop=612.7/s avg=248.1 FPS rtf=1.00 glitches=0
  INFO  [simulator_mavlink] Simulator connected on TCP port 4560
  ```

> **PX4 version note:** this pattern requires only the Simulator-MAVLink SITL path (TCP 4560)
> and a place to hang an `ExternalProject` plus the `make` targets. **v1.16 is the layout this repo
> targets**, and there each simulator owns a
> `src/modules/simulation/simulator_mavlink/sitl_targets_<sim>.cmake` — so FlightAxis ships
> one too (§3). **v1.13–v1.15** instead centralised the same hooks in
> `platforms/posix/cmake/sitl_target.cmake`; that file no longer exists in v1.16, and the
> equivalent splice for those older trees is kept in §3.1. Either way no PX4 stack code is
> modified — only two `CMakeLists.txt` registration lines (§3.2).

---

## 2. Files to Add

```
PX4-Autopilot/
├── Tools/simulation/flightaxis/                      # NEW (submodule or in-tree)
│   ├── sitl_run.sh
│   ├── hitl_run.sh                                   # HITL runner (see HITL.md)
│   └── flightaxis_bridge/
│       ├── CMakeLists.txt
│       ├── FA_check.py                               # sanity-ping 18083 before start
│       ├── get_FAbridge_params.py                    # JSON → argv
│       ├── cmake/
│       │   └── FindMAVLink.cmake                     # REQUIRED — verbatim upstream, see COPYRIGHT.md
│       ├── models/
│       │   ├── plane.json
│       │   ├── quad.json
│       │   ├── quadplane.json
│       │   └── heli.json
│       └── src/
│           ├── flightaxis_bridge.cpp                 # main loop
│           ├── px4_communicator.{cpp,h}              # TCP 4560, HIL msgs — adapted, see COPYRIGHT.md
│           ├── fa_communicator.{cpp,h}               # SOAP client + parser
│           ├── vehicle_state.{cpp,h}                 # RF→NED conversions (§6) + sensor synth (§7)
│           └── geo_mag_declination.{cpp,h}           # WMM tables — verbatim upstream, see COPYRIGHT.md
├── src/modules/simulation/simulator_mavlink/
│   ├── sitl_targets_flightaxis.cmake                 # NEW (§3)
│   └── CMakeLists.txt                                # MODIFIED: one include() line (§3.2)
└── ROMFS/px4fmu_common/init.d-posix/airframes/
    ├── 1200_flightaxis_plane                         # NEW (numbers per local convention)
    ├── 1201_flightaxis_quad
    ├── 1202_flightaxis_quadplane
    ├── 1203_flightaxis_heli
    └── CMakeLists.txt                                # MODIFIED: the four airframes (§3.2)
```

Two registrations live in files PX4 owns, and both are mandatory — see §3.2. In particular the
four airframe scripts must **also** be listed in the airframes `CMakeLists.txt`; dropping the
files in without that entry configures and builds cleanly and they simply never reach the ROMFS.

Copied verbatim from upstream (byte-identical; provenance in `COPYRIGHT.md`):
`geo_mag_declination.{cpp,h}` and `cmake/FindMAVLink.cmake`. The latter is not optional — the bridge's `CMakeLists.txt` puts
`./cmake` on `CMAKE_MODULE_PATH` and `find_package(MAVLink)` is what supplies the MAVLink
headers out of the PX4 build, so MAVLink is never a separate dependency to install.
`px4_communicator.{cpp,h}` is **adapted**, not copied: the original emits every
message on every frame, which at RealFlight's ~250 Hz would swamp PX4, so it gained per-message
decimation state and intervals, the `fields_updated` sub-rating, the DISTANCE_SENSOR path
(§8.2), the serial and UDP transports used for HITL, and the dead-link policy.

---

## 3. Build-System Registration — `sitl_targets_flightaxis.cmake` (PX4 v1.16)

In v1.16 each simulator owns one self-contained file under
`src/modules/simulation/simulator_mavlink/`, included from that directory's `CMakeLists.txt`.
FlightAxis follows that established file shape exactly. The whole file guards on
`ENABLE_LOCKSTEP_SCHEDULER STREQUAL "no"`, so the targets only exist on the `nolockstep` board —
which is correct, RealFlight free-runs and cannot be stepped.

`src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake`:

```cmake
if(ENABLE_LOCKSTEP_SCHEDULER STREQUAL "no")

	# RealFlight runs on a remote Windows machine, so there is no local
	# simulator binary to find_program() for.

	include(ExternalProject)
	ExternalProject_Add(flightaxis_bridge
		SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/flightaxis/flightaxis_bridge
		CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCMAKE_BUILD_TYPE=RelWithDebInfo
		BINARY_DIR ${PX4_BINARY_DIR}/build_flightaxis_bridge
		INSTALL_COMMAND ""
		USES_TERMINAL_CONFIGURE true
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
		BUILD_ALWAYS 1
	)

	# model list -> make px4_sitl_nolockstep flightaxis_<model>
	set(models plane quad quadplane heli)

	foreach(model ${models})
		add_custom_target(flightaxis_${model}
			COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/flightaxis/sitl_run.sh
				$<TARGET_FILE:px4> ${model} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
			WORKING_DIRECTORY ${SITL_WORKING_DIR}
			USES_TERMINAL
			DEPENDS px4 flightaxis_bridge
		)
	endforeach()
endif()
```

Three details that are deliberate rather than incidental, and that the ExternalProject
boilerplate elsewhere gets wrong:

- **No `PREFIX`.** Setting `BINARY_DIR` alone puts the build exactly where `sitl_run.sh`
  looks for it (`$build_path/build_flightaxis_bridge/flightaxis_bridge`); adding `PREFIX`
  as well only creates a redundant stamp/tmp tree.
- **`CMAKE_BUILD_TYPE=RelWithDebInfo`.** The bridge has to sustain 250+ Hz SOAP round-trips.
  Without it the sub-project builds unoptimised and both Eigen and the MAVLink encoders are
  left slow. Optimising the simulation build is not optional at this frame rate.
- **`WORKING_DIRECTORY ${SITL_WORKING_DIR}`** on the run target, so the `rootfs` and logs land
  where every other PX4 SITL target puts them.

The shipped file also warns at configure time if a listed model has no
`models/<model>.json` or no matching `*_flightaxis_<model>` airframe — both are otherwise
silent misconfigurations that only surface as a Python traceback at launch. It also declares a
`flightaxis_hitl_<model>` target per model, which runs `hitl_run.sh` and **depends on
`flightaxis_bridge` only, deliberately not on `px4`**: in HITL the firmware is on a board and
there is no `px4` binary to build or run. Those targets live in this file, which is included
from `simulator_mavlink/CMakeLists.txt`, purely because this is where the `flightaxis_bridge`
ExternalProject is defined and a target cannot depend on it from elsewhere — HITL does not use
`simulator_mavlink` at all. The consequence is that you configure a `px4_sitl_nolockstep` tree
to get a HITL runner, which is odd but harmless, since nothing SITL is built or started.
`hitl_run.sh` can equally be called directly; it needs nothing from CMake beyond the binary.

### 3.1 PX4 v1.13–v1.15 — `platforms/posix/cmake/sitl_target.cmake`

**Alternative, for older trees only.** `platforms/posix/cmake/sitl_target.cmake` does not exist
in v1.16. In v1.13–v1.15 it is where every simulator was registered centrally, so there the same
three pieces are spliced into that one file instead, alongside the entries already there for
the simulators that tree ships:

```cmake
# 1. add the simulator name to the sim list
set(simulators ... flightaxis ...)

# 2. model list for target generation
set(models_flightaxis plane quad quadplane heli)

# 3. the same ExternalProject_Add(flightaxis_bridge ...) shown above

# 4. hook the runner, inside the foreach that dispatches sim runners
elseif(viewer STREQUAL "flightaxis")
	add_custom_target(${_targ_name}
		COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/flightaxis/sitl_run.sh
			$<TARGET_FILE:px4> ${model} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
		DEPENDS px4 flightaxis_bridge
		...)
```

Exact splice points differ slightly per release — trace how an existing simulator name appears
in *your* checkout's `sitl_target.cmake` and replicate every occurrence. `install.sh` targets the
v1.16 layout only and refuses anything it does not recognise.

### 3.2 The two PX4-owned registration lines

Neither is optional, and neither is a file this repo can simply overwrite:

1. `src/modules/simulation/simulator_mavlink/CMakeLists.txt` — add
   `include(sitl_targets_flightaxis.cmake)` to the existing `include(sitl_targets_*.cmake)`
   block (alphabetical; it sorts first).
2. `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` — add the four
   `120x_flightaxis_*` names inside `px4_add_romfs_files(...)`, in sorted position. **Without
   this the airframes are never packed into the ROMFS** and `make px4_sitl_nolockstep
   flightaxis_plane` starts PX4 with no matching `SYS_AUTOSTART`.

`install.sh` applies both idempotently, backing each file up first; README Method 2 gives the
manual form.

### 3.3 Bridge `CMakeLists.txt`

Same skeleton as PX4's other in-tree bridges. Eigen3 is the only external package it needs; MAVLink arrives
via the in-tree copy of `FindMAVLink.cmake`, which is why `./cmake` goes on
`CMAKE_MODULE_PATH`. Abridged:

```cmake
cmake_minimum_required(VERSION 3.10)
project(flightaxis_bridge)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")   # bundled FindMAVLink, see COPYRIGHT.md
find_package(Eigen3 REQUIRED)
find_package(MAVLink)
add_executable(flightaxis_bridge
	src/flightaxis_bridge.cpp
	src/px4_communicator.cpp
	src/fa_communicator.cpp
	src/vehicle_state.cpp
	src/geo_mag_declination.cpp)
target_include_directories(flightaxis_bridge BEFORE PUBLIC ${MAVLINK_INCLUDE_DIRS})
target_compile_options(flightaxis_bridge PUBLIC -g -fexceptions -Wno-cast-align -Wno-address-of-packed-member)
target_link_libraries(flightaxis_bridge Eigen3::Eigen pthread)
```

Two things the shipped file adds that the sketch above omits, both for the sake of a standalone
`cmake && make` rather than the ExternalProject path:

- **It defaults `CMAKE_BUILD_TYPE` to `RelWithDebInfo`** when neither that nor
  `CMAKE_CONFIGURATION_TYPES` is set. The ExternalProject registration passes the build type
  explicitly, but a hand-run configure would otherwise get `-O0` Eigen quaternion maths and
  MAVLink encoders, which surfaces only as a climbing `glitches=` counter with no hint that the
  build type is the cause.
- **It turns a missing MAVLink into a `FATAL_ERROR` with instructions**, rather than leaving
  `find_package(MAVLink)` un-`REQUIRED`, as PX4's other in-tree bridges leave it. That is
  survivable for them because they are only ever configured by the ExternalProject step; here a
  miss leaves the include directory as the literal string `_MAVLINK_INCLUDE_DIR-NOTFOUND` and
  cmake fails at generate time without mentioning MAVLink at all. `FindMAVLink` looks in
  `<build dir>/../mavlink/`, so a standalone build must be configured from inside a PX4 build
  tree — which the ExternalProject step does implicitly and a hand-run cmake elsewhere does not.

---

## 4. Runner — `Tools/simulation/flightaxis/sitl_run.sh`

Follows the shape of PX4's other simulator runners, with one difference: RealFlight isn't launched (it lives on the Windows gaming box) — we only verify reachability. Abridged below. The shipped script also echoes its arguments and the SITL command; honours `DONT_RUN` and `NO_PXH` (the latter passing `-d` so the target can be driven from CI or a non-tty); invokes both Python helpers as `python3 ./script.py` rather than relying on the exec bit and shebang, which a zip round-trip or a `noexec` mount would defeat — and which would make the `||` branch misreport a working network as unreachable; and captures PX4's exit status *before* the cleanup `kill`, since otherwise a PX4 crash reports success and a clean Ctrl-C reports failure, timing-dependently:

```bash
#!/usr/bin/env bash
set -e
if [ "$#" -lt 4 ]; then echo "usage: sitl_run.sh sitl_bin model src_path build_path"; exit 1; fi

sitl_bin="$1"; model="$2"; src_path="$3"; build_path="$4"

rootfs="$build_path/rootfs"; mkdir -p "$rootfs"
export PX4_SIM_MODEL=flightaxis_${model}

# RealFlight host: env override, default localhost (RealFlight in a VM / same box)
FA_IP="${PX4_FLIGHTAXIS_IP:-127.0.0.1}"

# PX4 instance id: offsets the bridge's TCP port and, via `px4 -i` below, every
# PX4-side port and MAV_SYS_ID. The shipped script also validates it as a
# non-negative integer, since a non-numeric value reaches both consumers as a
# silent 0 and collides with instance 0 instead of failing (§11.3).
instance="${PX4_FLIGHTAXIS_INSTANCE:-0}"

echo "FlightAxis setup"
cd "${src_path}/Tools/simulation/flightaxis/flightaxis_bridge/"
./FA_check.py "${FA_IP}" || { echo "RealFlight FlightAxis not reachable at ${FA_IP}:18083"; exit 1; }

# Capture the bridge argv BEFORE backgrounding the bridge: substituting the
# python call straight into the command line would hide any failure, because
# empty argv makes the bridge print usage and exit, after which PX4 blocks
# forever on TCP 4560 with no diagnostic at all.
if ! fa_bridge_params=`./get_FAbridge_params.py "models/${model}.json"`; then
    echo "get_FAbridge_params.py failed for models/${model}.json" >&2
    exit 1
fi
if [ -z "$fa_bridge_params" ]; then
    echo "get_FAbridge_params.py produced no parameters for models/${model}.json" >&2
    exit 1
fi

"${build_path}/build_flightaxis_bridge/flightaxis_bridge" "$instance" "${FA_IP}" \
    $fa_bridge_params &
FA_BRIDGE_PID=$!

pushd "$rootfs" >/dev/null
set +e
# -i is load-bearing: it is what makes px4_instance non-zero inside rcS, which
# is what offsets the simulator TCP port the bridge is already listening on
# (§11.3). Without it PX4 boots as instance 0 whatever the bridge was told.
eval "\"$sitl_bin\" -i $instance \"$build_path\"/etc"
popd >/dev/null
kill $FA_BRIDGE_PID
```

Run experience:

```bash
# RealFlight running on Windows box 192.168.10.1, RealFlight Link enabled
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_plane
# QGC connects on UDP 14550 as usual
```

**Multi-instance: reachable from `make`.** The first bridge argv is the PX4 instance id and it
offsets the listen port (`PX4Communicator::InitTcpServer()` binds `portBase + portOffset`). `sitl_run.sh` reads
`PX4_FLIGHTAXIS_INSTANCE` (default 0) and threads it to both ends — earlier drafts hard-coded a
`0` there, so no `make` target could start a second instance even though the bridge always
supported it. Each instance still needs its own
RealFlight host — one RealFlight instance serves one aircraft. Full treatment in §11.3.

---

## 5. Model JSON — the "any airframe" mechanism

`models/*.json` carries the RealFlight channel map, and `get_FAbridge_params.py` flattens it to argv — the same JSON-to-argv arrangement PX4's other in-tree bridges use for their model configuration.

Below is the **shipped `models/quadplane.json`** (comments trimmed). The `rf` and `px4` columns
are identical, and that is deliberate — see the rule that follows.

```json
{
  "Comment":   "Quadplane -- four lift motors, a pusher and four control surfaces -- on a RealFlight custom model",
  "RfModel":   "informational only - aircraft is selected inside RealFlight",
  "Options":   ["ResetPosition", "SilenceFPS"],
  "Channels": [
    {"rf": 0,  "px4": 0,  "scale": "bipolar",                 "comment": "aileron left   <- controls[0]"},
    {"rf": 1,  "px4": 1,  "scale": "bipolar",                 "comment": "elevator       <- controls[1]"},
    {"rf": 2,  "px4": 2,  "scale": "unipolar", "disarm": 0.0, "comment": "fwd throttle   <- controls[2] pusher motor"},
    {"rf": 3,  "px4": 3,  "scale": "bipolar",                 "comment": "rudder         <- controls[3]"},
    {"rf": 4,  "px4": 4,  "scale": "unipolar", "disarm": 0.0, "comment": "lift motor 1   <- controls[4]"},
    {"rf": 5,  "px4": 5,  "scale": "unipolar", "disarm": 0.0, "comment": "lift motor 2   <- controls[5]"},
    {"rf": 6,  "px4": 6,  "scale": "unipolar", "disarm": 0.0, "comment": "lift motor 3   <- controls[6]"},
    {"rf": 7,  "px4": 7,  "scale": "unipolar", "disarm": 0.0, "comment": "lift motor 4   <- controls[7]"},
    {"rf": 8,  "px4": 8,  "scale": "bipolar",                 "comment": "aileron right  <- controls[8]"},
    {"rf": 9,  "px4": 9,  "scale": "bipolar",  "disarm": 0.5, "comment": "unassigned     <- controls[9]"},
    {"rf": 10, "px4": 10, "scale": "bipolar",  "disarm": 0.5, "comment": "unassigned     <- controls[10]"},
    {"rf": 11, "px4": 11, "scale": "bipolar",  "disarm": 0.5, "comment": "unassigned     <- controls[11]"}
  ],
  "UnmappedDefault": 0.5
}
```

**The rule: RealFlight channel N == PX4 output channel N.** Every shipped map is a pure identity
map. The JSON does not renumber anything; the ordering decision lives in the airframe, in
`PWM_MAIN_FUNC<N>` (SITL) and `HIL_ACT_FUNC<N>` (HITL) — the same place ArduPilot puts it
(`SERVOn_FUNCTION`) and the same place PX4's own Gazebo bridge puts it (`SIM_GZ_*_FUNC*`). Keeping
it there means one channel numbering runs from SITL through HITL to the physical FC pinout.

- `PWM_MAIN_FUNC<N>` in the airframe script selects which allocator output lands on
  `HIL_ACTUATOR_CONTROLS.controls[N-1]`, so choosing the FUNC values *is* choosing the order.
- Function value `101+k` is `CA_ROTOR<k>` — **a motor, range [0,1]** → `"scale": "unipolar"`.
- Function value `201+k` is **Servo *k*+1** — **range [-1,1]** → `"scale": "bipolar"`. *Which*
  actuator that servo is depends on `CA_AIRFRAME`, and the common gloss "`201+k` is
  `CA_SV_CS<k>`" holds only for fixed-wing and VTOL, where the servo block is the control
  surfaces. Under `CA_AIRFRAME 11` (helicopter, tail servo) the allocator lays out motors first
  (the main rotor), then the tail, then `CA_SP0_COUNT` swashplate servos
  (`ActuatorEffectivenessHelicopter.cpp:130-141`) — so Servo 1 = `201` is the **tail rotor** and
  `202`–`204` are **swashplate servos**, and none of the four is a `CA_SV_CS` at all. The
  bipolar range still applies; only the naming does not.
- The allocator emits motors before servos, so on any mixed airframe the motors take the low
  function numbers regardless of where they sit on the RealFlight transmitter. That mismatch is
  real and still has to be absorbed somewhere — it is absorbed by the FUNC list. It is exactly why
  the quadplane's `PWM_MAIN_FUNC1..9` are the scattered `201, 203, 105, 204, 101, 102, 103, 104,
  202` rather than `101..104, 105, 201..204`: the RealFlight model wants aileron, elevator,
  throttle, rudder, then the four lift motors, which is the conventional channel order for this
  class, and the FUNCs deliver `controls[]` in exactly that order.
- If a user's RealFlight model has a different channel order, the fix is to change
  `PWM_MAIN_FUNC<N>` to match it and keep the JSON identity — **not** to scramble the JSON.

Each shipped airframe script carries this derivation as a comment block next to its
`PWM_MAIN_FUNC*` lines; that block and the JSON must still agree, and a disagreement silently
drives the wrong actuator. The other three models:

| Model | RealFlight channel order | `PWM_MAIN_FUNC1..N` | Note |
|---|---|---|---|
| `plane` | aileron, elevator, throttle, rudder | `201, 203, 101, 204, 202` | `FUNC5` parks aileron right (Servo 2) on the spare `controls[4]`, unmapped |
| `quad` | motors 1–4 | `101, 102, 103, 104` | the one all-motor airframe, so the allocator order is already the RealFlight order |
| `quadplane` | aileron, elevator, throttle, rudder, lift 1–4 | `201, 203, 105, 204, 101, 102, 103, 104, 202` | as above; `FUNC9` parks aileron right on `controls[8]` |
| `heli` | swash 1–3, tail, (5–7 idle), main | `202, 203, 204, 201, 0, 0, 0, 101` | `CA_AIRFRAME 11` numbers motors first, then servos — main rotor = Motor 1 = 101, yaw tail = Servo 1 = 201, swash = Servos 2–4 = 202–204 — and the FUNCs reorder those onto **ArduPilot's heli channel order**: swash on rf0–2, tail on rf3, RSC on rf7, rf4–6 idle. Ground truth: `AP_MotorsHeli_Swash.cpp:201-202` (swash → `SERVO1/2/3`, `k_motor1/2/3`), `AP_MotorsHeli_Single.cpp:207` (`add_motor_num(CH_4)`, tail → `SERVO4`), `AP_MotorsHeli.h:34` (`#define AP_MOTORS_HELI_RSC CH_8`, `k_heli_rsc` = 31) |

Three heli-specific traps, all documented inside `heli.json`:

- **The tail rotor is bipolar**, with `"disarm": 0.5`. Under `CA_AIRFRAME 11` ("Helicopter (tail
  Servo)") the yaw tail is a *servo* on `[-1,1]`, and 0.5 on the wire is zero tail pitch — which
  is what the collective-pitch tail rotor of nearly every stock RealFlight single-rotor heli
  wants. `CA_AIRFRAME 10` ("tail ESC") would make it a motor clamped to `[0,1]`, clipping the
  entire negative half of the yaw command: a yaw command of −3 rad/s comes out as
  `rf3 = 0.000` with the tail on its lower stop, i.e. **no left yaw authority at all**. Type 10
  is only right for a heli with a separate unidirectional electric tail motor.

  The clipping is not quite unconditional, and the exception is worth knowing because it looks
  like a fix. `ControlAllocator` clamps motors to `[0,1]` and servos to `[-1,1]`
  (`ControlAllocator.cpp:501-513` and `:517-524`), but at `:508-509` a motor whose bit is set in
  **`CA_R_REV`** takes a minimum of `-1` instead of `0` — the reversible-motor escape hatch. So
  `CA_AIRFRAME 10` plus the tail motor's bit in `CA_R_REV` does restore the negative half of the
  yaw command. It is still the wrong answer here. `CA_R_REV` declares the *motor* reversible,
  which is a claim about an ESC that can spin backwards through zero; the actuator on a stock
  RealFlight single-rotor heli is a collective-pitch tail whose neutral is mid-travel and which
  never reverses direction. `CA_AIRFRAME 11` describes that hardware directly, gets the
  disarm-at-centre behaviour for free (`"disarm": 0.5`), and needs no second parameter kept in
  step with the first. Reach for `CA_R_REV` only when the motor really is reversible.
- **Every row needs an explicit `"disarm"`**, including the swash servos. This is belt-and-braces
  now that `HeliDemix` is off by default — the swash rows are plain pass-through and nothing
  rewrites them after the fact, so holding would be safe on that ground alone. The rule stays
  enforced so that *re-enabling* the demix for a CCPM model cannot quietly reintroduce the
  hazard: when applied as a post-pass over the built channel array, a row on the `-1` "hold last
  output" default would have its already-demixed value re-demixed on the next frame. That
  iteration is a non-involutive linear map and diverges: from a plausible in-flight swash it
  rails within about three frames. Neutral is a fixed point, so it only shows up on the
  armed→disarmed transition with the swash deflected — every landing — and nothing is logged.
  (The bridge additionally transforms out of a scratch copy, so holding is safe today either
  way; nothing in a model JSON can tell whether that is still true.)
- **Swash mixing lives in exactly one place, and that place is PX4.** `heli.json` ships without
  `HeliDemix`, so the allocator's three swash servo positions reach RealFlight untouched and the
  RealFlight model must be wired non-mixed / direct-servo. Two mixes in series compose rather
  than cancel, cross-coupling roll and pitch. This also matches ArduPilot's default, where the
  demix is opt-in via a `helidemix` substring in the frame name
  (`SIM_FlightAxis.cpp:129-130`). Re-enable it only for a CCPM model whose mixing cannot be
  turned off; on such a model the demix arithmetic below and the pinned swash geometry it
  depends on both come into play, and neither applies otherwise.
- **When enabled, `HeliDemix` only inverts correctly for the swash geometry the airframe forces**
  via `CA_SP0_ANG*` = 300/60/180, with equal arm lengths and `CA_MAX_SVO_THROW 0` (§11). With the
  demix off those angles instead describe the **physical** head of the RealFlight model.

- `px4` = index into `HIL_ACTUATOR_CONTROLS.controls[]`, which the airframe's `PWM_MAIN_FUNC*` assignments populate and the QGC Actuators tab displays. On every shipped model it equals `rf`; what varies between models is the FUNC list, not this column.
- Scaling: `unipolar` = clamp(v,0,1) for motors (HIL controls are 0..1 for motors); `bipolar` = (v+1)/2 for surfaces (−1..1). Net effect is the same as normalising a 1000–2000 µs PWM band to 0..1. `reverse` is applied *after* scaling, as `v -> 1-v`.
- `disarm` = the value sent while PX4 is disarmed or the control is NaN. **`disarm: -1` (also the default when the key is absent) means "hold the last output"** rather than driving the channel anywhere — right for a plain control surface, wrong for a motor, which is why every motor row states `"disarm": 0.0` explicitly, and wrong for any row an option post-pass rewrites, which is why every row in `heli.json` states one (see the heli traps above).
- **Duplicate indices are rejected at startup, not tolerated.** Two rows sharing an `rf` index would silently last-wins in `buildChannels()`, and a repeated `px4` index is almost always a typo, so the bridge refuses to start on either and names the two offending entries.
- `UnmappedDefault` is the value sent on every RealFlight channel the map does not mention.
- `Options` (a bit per option, flattened to a bitmask by `get_FAbridge_params.py`): `ResetPosition` (=1, ResetAircraft on startup, default), `Rev4Servos` (=2, swap ch1–4 ↔ 5–8 wholesale for RF models built that way — do **not** enable it on a model whose FUNC params already produce the right order, such as the quadplane, since it can only swap a correct order into a wrong one), `HeliDemix` (=4, **not enabled on any shipped model**; swash servos → RF roll/pitch/collective: `roll=(s1−s2)/1.732`, `pitch=((s1+s2)/2−s3)/1.5`, `col=(s1+s2+s3)/3`, recentered 0..1 — the divisors are gain normalisation, see below — for CCPM RealFlight models whose own mixing cannot be disabled), `SilenceFPS` (=8). All four shipped models flatten to **9** (`ResetPosition|SilenceFPS`).
- **The `HeliDemix` gain normalisation.** The geometric inverse alone is not unit-gain. `buildChannels()` maps each bipolar swash servo to `(v+1)/2` before the post-pass runs, and with the pinned 300/60/180 geometry that leaves `s1−s2` at gain 0.866 and `(s1+s2)/2−s3` at 0.750, against the 0.5 about a 0.5 centre that RealFlight wants for a full-scale command. Undivided, the cyclic saturates at ±0.577 of commanded roll torque and ±0.667 of pitch, roll hotter than pitch. So roll is divided by 0.866/0.5 = √3 = 1.732 and pitch by 0.750/0.5 = 1.5. **Not** by 0.866 and 0.750: that recovers raw torque and clips at half command. Collective, `(s1+s2+s3)/3`, is already exactly gain 0.5 and is deliberately untouched. The constants are only valid for the pinned geometry — changing `CA_SP0_ANG*` or the arm lengths invalidates them. **These divisors are ours and ArduPilot has no equivalent:** `SIM_FlightAxis.cpp:348-350` is a plain unweighted inverse, exact only for a 140° head (`AP_MotorsHeli_Swash.cpp:138-145` mixes `H3_140` with flat ±1.0 factors). On the 120° head both projects actually use — `AP_MotorsHeli_Swash.cpp:147-154` defaults `add_servo_angle` to −60/+60/180, the same geometry as our `CA_SP0_ANG` 300/60/180 — ArduPilot's forward mixer carries cos 30° = 0.866 on roll against 0.5/1.0 on pitch, so its own round trip returns roll and pitch gains in the ratio 2/√3 = 1.155 rather than 1. Matching ArduPilot here would reintroduce that imbalance. **The channel order follows ArduPilot; the demix gains deliberately do not.**
- **Both post-passes read a scratch copy, not the persistent channel array.** `channels[]` holds the untransformed per-channel state so that hold-last has something stable to hold; `out[]` is what goes to RealFlight. Transforming in place would re-transform every hold-last channel every frame — `HeliDemix` diverges, and `Rev4Servos`, whose swap is its own inverse, instead ping-pongs a held channel between `rf i` and `rf i+4` on alternate frames, a full-amplitude servo buzz at loop rate.
- Adding a new aircraft takes **four** steps, not three: new JSON, one model name in the `models` list in `sitl_targets_flightaxis.cmake`, one airframe script — and that airframe must also be added to `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` (§3.2) or it never reaches the ROMFS.

RealFlight model prep (once per aircraft, in the RF editor): strip expo/mixes/gyros, max servo speed, one channel per actuator.

---

## 6. Frame Conversions

These go in `vehicle_state.cpp`. They are line-for-line what the upstream implementation named in `COPYRIGHT.md` ships — not inferred. RealFlight's conventions are internally inconsistent (position swapped, velocity not); follow literally.

**Quaternion (RF → NED):** `q_ned = (w=W, x=RF_Y, y=RF_X, z=−RF_Z)` — swap X↔Y, negate Z.

**Gyro:** `p=+roll_rate, q=+pitch_rate, r=−yaw_rate` (deg→rad, constrain ±2000 °/s). **Only yaw negated.**

**Position (NED):** `(N=RF_Y, E=RF_X, D=−altASL)`. Capture `position_offset` on first frame and after every `m-resetButtonHasBeenPressed`; subtract it so PX4's home (bridge argv / `PX4_HOME_*`) anchors the RF world. NED→LLA for HIL_GPS.

**World velocity:** `(vN=U, vE=V, vD=W)` — **used directly, no swap.** The asymmetry with position
is real and deliberate: it is what upstream ships. Applying position's swap here as well would
transpose north against east and make every mission leg track the wrong heading.

**Wind:** swapped like position: `(windY, windX, windZ)`.

**Heading datum (optional, `PX4_HOME_YAW`):** RealFlight's world north is arbitrary, so the conversions above map it onto NED unrotated. When `PX4_HOME_YAW` is set it names the true heading the aircraft should *start* on; the rotation that implies is derived on the first frame — `θ = wrapPi(home_yaw − yaw(q_ned))`, taken from the converted quaternion rather than `m-azimuth-DEG` so it cannot disagree with the attitude the pipeline uses — and latched alongside `position_offset`, sharing its lifetime so a reset re-derives it. `θ` is then applied about the down axis to **every** world-frame quantity: `q_ned` (pre-multiplied), position (after the offset subtraction, so the world turns about home and not RealFlight's origin), world velocity, and wind. Body-frame quantities and all Down components are left alone — rotating the world does not change what a strapdown sensor measures. HIL_GPS, baro, mag and COG need no special handling: each is derived from something already rotated. Applying `θ` to attitude alone would leave heading disagreeing with direction of travel, which EKF2 reads as an inconsistency it cannot null out. Unset is NAN, not 0, since 0 is a valid request for due north; at `θ = 0` every path is a strict no-op.

**Accelerometer (specific force):** in flight use `m-accelerationBodyA*` directly. On ground it's
garbage — RealFlight's ground-contact accelerometer output is noisy enough to prevent a
helicopter disarming — so substitute a finite difference of velocity. The shipped override goes
beyond the upstream port in three deliberate ways: it triggers on more than ground contact, it
refuses to differentiate across too short an interval, and it bounds the result far tighter than
the sensor clamp:

```cpp
accel_norm   = |m_accelerationBodyA*|;
accel_absent = m_hasLostComponents || !isfinite(accel_norm) || accel_norm < 0.05;

if (touching_ground || accel_absent) {
    // dt_true, NOT the glitch-capped dt: a swallowed glitch would divide the
    // real velocity change by a shorter interval and overstate the accel
    if (dt_true >= 0.005f && have_last_velocity) {
        accel_ef  = (velocity_ef - last_velocity_ef) / dt_true;
        accel_ef.z -= 9.80665f;
        accel_body = R_ned_to_body * accel_ef;  // yields exactly (0,0,-g) at rest
        clamp(accel_body, ±3 g);                // synthetic, not measured
    } else {
        accel_body = R_ned_to_body * (0, 0, -9.80665f);   // "at rest"
    }
}
clamp(accel_body, ±16 g);
```

Each addition answers a distinct failure:

- **`m_hasLostComponents`.** A model that has shed components reports an accelerometer that no
  longer describes a rigid body. The finite difference is the better source in that state.
- **`accel_norm < 0.05`.** RealFlight can report a near-zero specific force for a stationary
  aircraft, which is not what an accelerometer at rest measures — it should read 1 g. EKF2 uses
  the accelerometer as its only tilt-correction observation, and a gravity observation nowhere
  near 1 g is rejected, leaving attitude uncorrected. The threshold catches that case whether or
  not ground contact is flagged.
- **The 5 ms minimum `dt`.** Differentiating amplifies by `1/dt`, so a small velocity step across
  a short frame becomes an enormous acceleration — and at the ~4 ms nominal frame period, short
  frames are ordinary. Below the threshold the previous value is held rather than a quotient
  dominated by `1/dt` being reported.
- **The 3 g synthetic clamp**, applied *inside* the branch and in addition to the ±16 g sensor
  clamp. The 16 g bound exists to match what a Pixhawk can *report*, which is the right bound
  for something measured and far too loose for something synthesised by division — a 1/dt spike
  can pass ±16 g untouched.

Switching source mid-flight would make the next finite difference span a discontinuity, so the
velocity history is dropped on the edge in either direction, not only on the reset button.

**Pitot airspeed — computed, not read:** `m_airspeed_MPS` is total TAS; a pitot wants body-X:

```cpp
airspeed_pitot = max( (R_ned_to_body * (velocity_ef - wind_ef)).x , 0 );
diff_pressure  = 0.5f * 1.225f * airspeed_pitot * airspeed_pitot / 100.0f;   // Pa -> hPa
```

Critical for 3D/high-alpha models — total airspeed lies during hover/harrier/knife-edge.

**Pressure units are hPa, not Pa.** Both `HIL_SENSOR.abs_pressure` and `HIL_SENSOR.diff_pressure`
are hectopascals in MAVLink, so the `/100` above is not cosmetic — dropping it feeds PX4 an
airspeed error of a factor of 10. Both are declared as hPa in `vehicle_state.h`.

**Rangefinder:** `AGL / dcm.c.z` when `dcm.c.z > 0`, else invalid (inverted). This is not
computed and discarded: when valid it is packed into a `DISTANCE_SENSOR` message and sent to PX4
at 20 Hz (§8.2), which is what makes terrain-relative modes and landing usable.

**Magnetometer:** WMM field at home via `geo_mag_declination.cpp` (bundled verbatim, see `COPYRIGHT.md`), rotated to body with the quaternion above.

**Barometer:** ISA, `p = 101325·(1 − 2.25577e-5·h)^5.25588` Pa, converted to **hPa**. Temp ~25 °C.

The altitude `h` fed into that formula is **deliberately not RealFlight's ASL**. It is
`h = home_alt − position_ned.z()` — the same datum `nedToLLA()` uses for the GPS altitude it
reports. The reason is that a RealFlight runway sits at
whatever arbitrary ASL its scenery author chose, which has nothing to do with the home altitude
PX4 was given. Deriving baro from RF ASL while deriving GPS from `home_alt` would present EKF2
with a *constant* baro-vs-GPS height disagreement — the estimator would either fight it forever
or reject one of the two sources. Anchoring both to the same datum makes the two agree by
construction; the cost is that the baro no longer encodes the scenery's real elevation, which
nothing downstream cares about. A mission commanded to a height above home therefore holds that
height against a single consistent datum rather than against two that differ by a constant.

---

## 7. Timing — extrapolation & glitch handling

RealFlight free-runs, and the bridge is designed against a **nominal ~250 Hz** frame rate.

That figure is a design assumption, not a measured or documented property of FlightAxis. No
upstream source states a delivery rate: ArduPilot's 250 is the rate of its *own* scheduler
(`SIM_FlightAxis.cpp:128`, `rate_hz = 250 / target_speedup;`), and the ArduPilot wiki's advice
that RealFlight's "Graphics Frames/Sec" stay above 200 is a different quantity on a different
clock. In practice, on a fast machine and a low-latency link, the observed FlightAxis exchange
rate is nearer 750–800 Hz. The assumption is used only as a conservative floor for sizing
thresholds and intervals, and `px4_communicator.h` frames it the same way: every gate below is a
time comparison against the physics clock rather than a frame count, so a higher actual rate only
makes the quantisation finer and the achieved sub-rates land closer to their targets. Nothing
breaks if the assumption is wrong in either direction.

Upsampling alone is enough to dodge PX4's stale-sensor detection, but not enough to survive duplicate frames, network glitches and a simulator that stops producing physics altogether; the bridge uses a four-branch scheme, keyed on `m-currentPhysicsTime-SEC`:

0. **leaving a keep-alive** (physics time has advanced past the last frame we consumed, and a keep-alive was running) → re-base the epoch onto the clock already exported, so the next frame resumes at exactly the clock PX4 last saw, and skip glitch compensation for this frame. See below.
1. **dt < 0** → RealFlight restarted: re-base initial time, zero position offset, continue.
2. **dt < 1e-5 s** (same physics frame — bridge outran RF): do **not** resend an identical HIL_SENSOR. Extrapolate in 1 ms steps (propagate attitude by `q⊗exp(½ω·δt)`, hold accel/gyro), never beyond `average_frame_time` (EMA `0.98/0.02`).
3. **normal frame** → full pipeline, then glitch compensation: if physics time jumped 50 ms–2 s (network hiccup), swallow the excess by advancing the epoch (`initial_time += dt − 50 ms`), cap dt at 50 ms, count the glitch. Backwards >500 ms = true reset, accept.

Timestamps in HIL messages = physics time (µs since epoch capture). Watch `m-currentPhysicsSpeedMultiplier ≠ 1` → warn. Print the FPS line every 1000 frames.

**The wall-clock keep-alive**, and why branch 0 exists to unwind it. Two situations stop
RealFlight's physics clock while PX4 still needs a sensor stream: the controller reinject after
a spacebar reset or an aircraft change, and a *stalled* clock — sim paused, a modal dialog open,
the window minimised, with `ExchangeData` still returning valid replies carrying the same
`m-currentPhysicsTime-SEC` over and over. From the bridge both look identical, a good frame with
no new physics in it, so both get the same treatment: hold the last state, advance the exported
clock off **wall** time at ~1 kHz, and keep sending. A single step is capped at 100 ms so a long
stall cannot teleport the clock forward.

The stall is only declared after 200 ms, and that threshold is load-bearing in both directions.
Arriving at "no new frame" is *normal* once per frame — the loop free-runs at several kHz against
a nominally 250 Hz simulator and routinely finishes interpolating with time to spare — so too low a
threshold makes branch 0 re-base on nearly every real frame, which measurably consumed most of
them and left the realtime-factor monitor permanently reading `n/a`. Too high and PX4's sensor
topics go stale, since the arming checks want each one updated within 1 s. 200 ms sits an order
of magnitude above any plausible frame period (4 ms at 250 Hz, 20 ms even at a badly degraded
50 Hz) and comfortably under that 1 s timeout.

Sending nothing was the original behaviour and it was worse. Under an induced 6 s freeze,
HIL_SENSOR stopped about 52 ms into the stall, and PX4 went on to log `MAG #0 failed:
TIMEOUT`, `BARO #0 failed: TIMEOUT` and `angular velocity no longer valid (timeout)`, failing
every sensor arming check — and the whole stall was later applied to the clock as one 6 s jump,
silently, since that exceeds the 2 s glitch ceiling.

Branch 0 is what keeps the two clocks from drifting apart afterwards. The keep-alive advances
the exported clock while the physics epoch stands still, so by the time real frames resume the
two disagree by the length of the stall. Falling straight through to branch 3 would compute the
new time from that stale epoch: diverged backwards, the monotonic clamp **freezes** the exported
clock, and because the offset is constant — physics resumes exactly where it paused — it never
self-corrects and stays frozen for the rest of the session. A frozen clock is severe even in
SITL, where PX4 discards our timestamps anyway, because every send-rate gate is measured against
it: HIL_GPS, DISTANCE_SENSOR and the baro/mag sub-rates stop being sent at all and only the
ungated HIL_SENSOR survives. Re-basing the epoch onto the clock already exported removes both the
freeze and the jump. The upstream implementation the other three branches were ported from has
no wall-clock keep-alive and so never meets any of this: branch 0 and the keep-alive are
original to this bridge, which is why the provenance notices still describe a *three*-branch
port.

The reinject itself is throttled to one attempt per 300 ms, separately from the keep-alive:
`startController()` is three SOAP round-trips *including* `ResetAircraft`, and the branch is
entered thousands of times a second while the flag is low.

**Realtime-factor monitor.** The branches above keep the bridge's *exported* clock clean,
which is necessary and not sufficient. PX4 timestamps the SITL sensor stream with its own clock
on arrival (`SimulatorMavlink::handle_message_hil_sensor`), not with the physics time carried in
the message, so what actually reaches EKF2 is the wall-clock arrival rate. If RealFlight's
physics falls behind wall time, PX4 integrates the vehicle's motion over more real time than the
physics covered: the aircraft flies in slow motion while every sensor value looks perfectly
correct, and velocities, accelerations and every control loop are scaled wrong. Nothing in the
sensor data reveals it, and the glitch compensation actively hides it, because it smooths the
exported clock rather than the arrival rate. The bridge therefore measures physics seconds per
wall second over each reporting window, prints it as `rtf=` in the FPS line, and warns (rate
limited to once per 5 s) outside 0.95–1.05. This is distinct from the
`m-currentPhysicsSpeedMultiplier` warning, which is RealFlight's *self-reported setting*: the
realtime factor catches RealFlight reporting 1.0 and the machine failing to keep up. One sample
is suppressed after a keep-alive, where the physics clock was paused on purpose.

---

## 8. Bridge Internals

### 8.1 `fa_communicator` — SOAP client

- **New TCP connection per request**, latency hidden by a background *socket-creator thread* that always has the next connected socket parked (100 ms connect timeout, condition-variable handoff). This, not keep-alive, is the pattern RealFlight's SOAP endpoint actually tolerates: it closes the connection after each reply.
- Reply read: find `Content-Length`, drain to `\r\n\r\n`+length, close socket.
- Parser: sequential `strstr` scan over the key table **in document order** (12 `item` echoes first, then all `m-*` fields), `true/false→1/0`, `atof`. Any missing key → flag re-init (schema/version change self-heals).
- Startup sequence (exact order): `RestoreOriginalControllerDevice` → (`ResetAircraft` if ResetPosition) → `InjectUAVControllerInterface`.
- Re-run startup whenever no socket AND (`!controller_started` OR `m-flightAxisControllerIsActive==0` OR `m-resetButtonHasBeenPressed`) — makes aircraft-change and spacebar in RealFlight self-heal.
- `m-selectedChannels`: send **0 until PX4 is up** (first HIL_ACTUATOR_CONTROLS received), then 4095. RealFlight holds neutral meanwhile.

### 8.2 `px4_communicator`

TCP server on 4560+instance; PX4 (`simulator_mavlink`) connects. Receives `HIL_ACTUATOR_CONTROLS`
(armed flag in `mode`; NaN/disarmed → JSON `disarm` values).

`Send()` runs at the full RealFlight frame rate plus every 1 ms extrapolation step (§7), which is
far faster than most of these messages need. Only `HIL_SENSOR` genuinely wants that rate, so
everything else is decimated against the bridge's own physics clock; the interval constants and
the dispatch both live in `px4_communicator`. One of each is forced out on the first frame so
PX4 has a complete picture immediately:

| Message | Rate | Notes |
|---|---|---|
| `HIL_SENSOR` | every frame / extrapolation step (~1 kHz) | pressures in hPa (§6); `fields_updated` sub-rated, below |
| `HIL_GPS` | **10 Hz** (`GPS_INTERVAL_US`) | COG = `atan2(vE,vN)`, **not** RF azimuth. PX4 publishes one `sensor_gps` per message with no rate limiting of its own, hence the decimation |
| `HIL_STATE_QUATERNION` | **50 Hz** (`STATE_QUAT_INTERVAL_US`) | ground truth only — for log analysis, not consumed by the estimator |
| `DISTANCE_SENSOR` | **20 Hz** (`DISTANCE_INTERVAL_US`) | built by `VehicleState::getDistanceSensorMsg()`; downward-facing (`PITCH_270`), 0.1–40 m, cm units. **Gated on `rangefinderValid()`** — suppressed entirely while the aircraft is inverted rather than sent as a bogus reading |
| `RC_CHANNELS` | **50 Hz** (`RC_INTERVAL_US`) | built by `VehicleState::getRcChannelsMsg()` from the 12 InterLink channels echoed in every ExchangeData reply. **Gated on `rcValid()`** — see the RC passthrough note below |
| `RAW_RPM` | every frame | index 0, from RealFlight's prop/rotor RPM. `SimulatorMavlink` *does* handle it — it decodes the message and publishes `rpm_s` on the `rpm` topic (it is one of the ten ids of §11.2) — but nothing in the SITL stack consumes that topic for a simulated airframe, so the value is low-return bandwidth on any link where bandwidth is not free. The HITL profile drops it, and there the asymmetry is decisive: `mavlink_receiver.cpp`, which is the receive path on a real board, has **no** `RAW_RPM` handler at all — `RAW_RPM` exists there only as an outbound stream, so a HITL `RAW_RPM` is parsed and discarded |

**Sub-rates *within* `HIL_SENSOR`.** `vehicle_state` builds every message with
`fields_updated = 0x1FFF` — "every sensor is new" — and `px4_communicator` then masks that
bitmask down on the way out. Accel and gyro are never masked and follow the full `HIL_SENSOR`
rate; the magnetometer (100 Hz), barometer (50 Hz) and differential pressure (50 Hz) are masked
out between their own intervals. This applies **in SITL as well as HITL**, which is why the
intervals are not zero by default. The motivation on a real board is CPU and fidelity: 250 Hz of
baro and mag into an MCU whose real drivers would run them at 50–100 Hz misrepresents the sensor
suite to the estimator. In SITL it is log volume: `SimulatorMavlink::update_sensors()` publishes
`sensor_baro` *twice* for every message carrying the BARO bits, plus mag and differential
pressure once each, so at the ~1 kHz rate `Send()` runs the baro went out at roughly 20× and the
mag and pitot at 10× their intended rates — surplus that `VehicleAirData` drains and averages
away after inflating every ulog for nothing.

Two details make this fiddlier than it looks. The masks are **all-bits-equal** tests, not
any-bit: `BARO` needs bits 9, 11 and 12 together. And bit 12 is what latches
`_sensors_temperature`, which accel, gyro and mag then read — so `BARO` must never be masked out
of the *first* message. It is not: a `sent_first` flag backdates every interval timer so message
one carries the full `0x1FFF`.

**A dead PX4 link is terminal.** PX4 can vanish silently — the SITL binary exits, a USB cable is
pulled, a board reboots — after which every send fails. The bridge does not retry: once the link
is declared lost the communicator stops sending, `LinkLost()` latches, and the main loop breaks
out and exits cleanly, which stops the SOAP traffic as a consequence. The alternative is
thousands of log lines a second and continued hammering of RealFlight for a simulation nobody is
consuming, and on serial up to 100 ms burnt in `poll(POLLOUT)` per failed frame. Reconnecting is
deliberately not attempted either: for the TCP server that would mean re-`accept()`ing a fresh
PX4 whose clock starts at zero while the bridge's has not, and the supervisor that started both
sides is the right place to restart the pair. Detection is `POLLHUP`/`POLLERR`/`POLLNVAL`, a
zero-length TCP read, a fatal `send()` errno, or 100 consecutive send failures — the last of
which catches the serial `POLLOUT`-timeout case, which has no distinguishing errno. **UDP has no
such signal at all**; a datagram to a dead board succeeds forever. That is inherent to the
transport, not an omission.

**HITL transports.** Beyond the SITL TCP server, `px4_communicator` also offers a serial client
and a UDP client, selected by `PX4_HITL_TRANSPORT`, together with a HITL message profile that
decimates `HIL_SENSOR`, drops `HIL_STATE_QUATERNION` and `RAW_RPM`, and adds a `HEARTBEAT`.
`HIL_STATE_QUATERNION` matters most: in SITL it is ground truth for logging, but a real board
*consumes* it and republishes straight onto EKF2's own output topics. Full treatment, including
the bandwidth budget and the `_datarate > 5000` gate, is in [`HITL.md`](HITL.md).

**RC passthrough.** The 12 `item` values echoed in every ExchangeData reply *are* the physical
InterLink TX channels. The bridge parses them into `FAState.rcin[12]`, and
`VehicleState::getRcChannelsMsg()` packs them into a **`RC_CHANNELS`** message that
`px4_communicator` sends at 50 Hz — so the RealFlight transmitter drives PX4's `input_rc`, and
Manual/Acro from the TX works.

The message is `RC_CHANNELS`, **not** `HIL_RC_INPUTS_RAW`. That is not a stylistic choice:
`SimulatorMavlink::handle_message()` dispatches on exactly ten ids (§11.2) and `RC_CHANNELS` is
one of them; `HIL_RC_INPUTS_RAW` is not, and would be parsed and silently discarded. Four
details of the packing follow from what the consumer does with it:

- Each `rcin` value is 0..1 and is mapped to 1000–2000 µs, **clamped**, because RealFlight
  reports slightly outside 0..1 for a TX whose endpoints exceed the model's configured travel;
  an unclamped cast can wrap the `uint16_t`. NaN is treated as centre rather than cast.
- Channels 13–18 are set to `UINT16_MAX`, MAVLink's "not available" sentinel, not 0.
  `SimulatorMavlink` copies all 18 raw fields into `input_rc.values[]` regardless of
  `chancount`, so zeros there would read as real channels pegged below minimum.
- `rssi` is **100**, not MAVLink's idiomatic 254. `SimulatorMavlink` assigns the field straight
  into `input_rc.rssi`, which PX4 defines as a *percentage* with 100 as full reception.
- The send is gated on `rcValid()`, which is false until the first FlightAxis frame has
  arrived. Publishing a frame of zeros beforehand would tell PX4 the RC link is up with every
  stick at minimum — including throttle and whatever channel carries the arming switch.

One transmitter quirk survives into calibration: the InterLink sends **ch2 inverted** relative
to what PX4 expects, so reverse it in RC calibration. Upstream corroborates this — ArduPilot's
FlightAxis backend forces `RC2_REVERSED = 1` with the comment "interlink has reversed rc2"
(`SIM_FlightAxis.cpp:81`).

### 8.3 Main loop

```
take parked socket → ExchangeData(channels) → parse
branch on dt (§7) → convert (§6) → HIL_SENSOR [+GPS/GT on schedule]
drain 4560 (non-blocking) → latest HIL_ACTUATOR_CONTROLS → JSON channel map → next channels
```

Loop rate is set by RealFlight's SOAP RTT (~2–5 ms → 200–500 Hz). Wired network or same-host only; WiFi cannot hold 200 Hz.

---

## 9. Airframe Script Example — `1200_flightaxis_plane`

**Heavily abridged — this is the shape, not a working airframe.** The four real scripts run
44–108 lines each, and the bulk of that is the actuator geometry (`CA_AIRFRAME`, `CA_ROTOR*`,
`CA_SV_CS*`, `PWM_MAIN_FUNC*`) plus per-vehicle tuning, none of which is optional. Read the
shipped files — `ROMFS/px4fmu_common/init.d-posix/airframes/120{0,1,2,3}_flightaxis_*` — before
writing a new one; each carries a comment block deriving its `PWM_MAIN_FUNC*` values from the
allocator's own numbering, which is where the channel order is decided and which must stay in
step with the matching `models/<name>.json` — an identity map (§5).

```sh
#!/bin/sh
# @name FlightAxis Plane
# @type Plane
. ${R}etc/init.d/rc.fw_defaults

# RealFlight free-runs (no lockstep), GPS samples arrive fresh.
param set EKF2_GPS_DELAY 0
param set EKF2_MULTI_IMU 1
param set-default SDLOG_MODE 4
param set-default COM_RC_IN_MODE 4

# ... 60 more lines: CA_* geometry, PWM_MAIN_FUNC* assignments, per-vehicle tuning ...
```

**`param set` versus `param set-default`.** The first two use a plain `param set` deliberately,
because `px4-rc.mavlinksim` runs *after* the airframe script and issues its own
`param set-default` for both, which would quietly win over a default set here. A plain
`param set` takes precedence and survives. The rest are `set-default`, so a user's saved value
still wins. What each is for:

| Parameter | Value | Why |
|---|---|---|
| `EKF2_GPS_DELAY` | `0` | RealFlight free-runs and the synthesised GPS samples are current, with none of the transport delay the 10 ms default compensates for. |
| `EKF2_MULTI_IMU` | `1` | The bridge sends `HIL_SENSOR` with `id = 0` only, so `SimulatorMavlink` only ever fills IMU instance 0. `px4-rc.mavlinksim` asks for 3, leaving two EKF instances running on dead sensors. `0` is not an alternative — EKF2 clamps this to a minimum of 1 when `SENS_IMU_MODE` is 0. |
| `SDLOG_MODE` | `4` | `rcS` sets 1, "from boot until disarm", which closes the log at the *first* disarm. A session spanning several arm/disarm cycles would then be mostly unlogged — including any groundtruth-versus-estimate comparison. 4 is "from first arm until shutdown". `@reboot_required`, so it has to be a startup default rather than a `pxh>` command. |
| `COM_RC_IN_MODE` | `4` | A headless run has no manual control source at all, and 4 ("ignore any stick input") is the only value that skips the RC-loss failsafe. `NAV_RCL_ACT` is deliberately left alone: it is unreachable once this is 4, and the value usually suggested for it is outside the parameter's declared range. |

**Rotor geometry: the symmetry matters, the magnitudes do not.** PX4 normalises the multirotor mixing matrix by a scale derived from the mix
columns themselves (`ControlAllocationPseudoInverse`), so the arm length divides straight back
out: a ±0.15 m square, a ±0.25 m square and a ±0.50 m square produce a bit-identical matrix for
any `KM` and `CT`, all of them ±0.7071 per motor — the cos(45°) values a quad-X frame table
would give directly. **So `CA_ROTOR*_P[XY]` needs no adjustment per RealFlight model, and no doc
should suggest measuring it.** What is *not* free is asymmetry: feed asymmetric arms to a
symmetric aircraft and roll and pitch stay clean while yaw cross-couples — with the asymmetric
arms of PX4's stock standard-VTOL airframe (PY 0.245 / -0.1875), a unit yaw command also
produces a 53 % roll disturbance. That figure falls out of the allocator's mixing matrix
directly — it is a property of the pseudo-inverse — and it is structural, so no rate tuning
removes it. The symmetric
square in the shipped airframes is therefore a deliberate choice, not a placeholder awaiting
real numbers.

---

## 10. Implementation Order (each step testable)

1. Lay out the bridge folder; bring in `px4_communicator` and `geo_mag_declination`; stub out the sim-side communicator.
2. `fa_communicator`: standalone test against RealFlight — print physics time ≥200 FPS. (No PX4 involved yet.)
3. `vehicle_state` conversions → HIL_SENSOR; run PX4, check `listener sensor_accel` / QGC attitude mirrors RealFlight.
4. Actuator path: QGC Actuators sliders move RF surfaces through the JSON map.
5. Timing hardening (§7) + reconnect state machine.
6. Options (Rev4/HeliDemix), remaining model JSONs, `sitl_targets_flightaxis.cmake` polish, RC passthrough (§8.2).

## 11. Pitfalls Ledger

| Pitfall | Fix |
|---|---|
| Lockstep vs free-running RF | use the existing `px4_sitl_nolockstep` target |
| Duplicate physics frames → zero-dt sensors | §7 branch 2 extrapolation |
| Network hiccup → EKF time jump | 50 ms glitch swallow |
| Ground-contact accel noise | finite-difference override (§6) |
| Pitot fed total airspeed | body-X from velocity−wind (§6) |
| Position swapped, velocity not, wind swapped | §6 literally |
| Channels sent before PX4 ready | selectedChannels 0 → 4095 after first actuator msg |
| Aircraft change / spacebar mid-session | reconnect state machine (§8.1) |
| Keep-alive assumption | per-request sockets + creator thread (§8.1) |
| **Baro-vs-GPS datum mismatch** — RF runway sits at an arbitrary scenery ASL, so baro from RF ASL disagrees with GPS from `home_alt` by a constant, forever | derive both from `home_alt − position_ned.z` (§6) |
| **Pressures sent in Pa** — MAVLink `HIL_SENSOR` wants hPa | `/100` on both `abs_pressure` and `diff_pressure` (§6) |
| **`EKF2_GPS_DELAY` and `EKF2_MULTI_IMU` silently clobbered** — `px4-rc.mavlinksim` runs after the airframe and re-defaults both | use `param set`, not `param set-default`, for those two (§9) |
| **RC-loss failsafe on a headless run** — no joystick, no RC, no manual control source at all | `COM_RC_IN_MODE 4`, the only value that skips the check (§9) |
| **Log closes at the first disarm**, so a session spanning several arm/disarm cycles is mostly unlogged | `SDLOG_MODE 4` (§9) |
| **Heli swash mixing applied twice** — PX4's allocator mixes, and the RealFlight model's own CCPM mixes again; the two compose rather than cancel and roll/pitch cross-couple in flight, with nothing logged | put the mix in exactly one place: `HeliDemix` off in `heli.json` (default) and the RealFlight model wired non-mixed / direct-servo (§5) |
| **Heli cyclic response is rotated** — `CA_SP0_ANG*` does not match the model's actual head, so a pitch command banks the aircraft | force 300/60/180 in `1203_flightaxis_heli` (120° head, odd servo aft, ArduPilot's default) and pin the arm lengths and `CA_MAX_SVO_THROW 0`; use 120/240/0 for an odd-servo-forward head. |
| **Heli has no left yaw** — under `CA_AIRFRAME 10` the tail is a *motor* clamped to `[0,1]`, so the whole negative half of the yaw command is clipped and the tail sits on its lower stop | `CA_AIRFRAME 11` (tail Servo), with `rf3` `bipolar` and `disarm` 0.5 in `heli.json` (§5) |
| **Heli demix over-drives the cyclic** — the raw differences are gain 0.866 / 0.750, not 0.5, so roll saturates at 0.577 of command and pitch at 0.667, roll hotter than pitch | divide roll by √3 and pitch by 1.5 in the bridge's `HeliDemix` post-pass (§5) |
| **Heli hold-last swash re-demixed every frame**, diverging within ~3 frames on the armed→disarmed transition | every `heli.json` row carries an explicit `disarm`, and the post-passes read a scratch copy rather than the persistent channel array (§5) |
| **Motor-failure detection does not work in SITL** — `SimulatorMavlink` reports 1–16 A per ESC while armed, and the undercurrent test `1 + 15c < 2c` has no solution for any throttle | nothing to fix, and nothing to suppress: the quadplane's inherited `FD_ACT_EN 0` / `FD_ACT_MOT_TOUT 500` were suppressing a detector that cannot fire, so they were **removed**. Left at firmware defaults, detection starts working if upstream fixes the SITL ESC current model. Corollary: no motor-failure detection in SITL today, for any vehicle. |
| **Rotor arm lengths treated as a per-model measurement** — they are not; the allocator normalises the magnitudes away and only the symmetry pattern survives | keep the symmetric square; never "correct" it towards a reference airframe's measured arms (§9) |
| **Dead PX4 link retried forever** — thousands of log lines a second and continued SOAP load for a simulation nobody consumes | a dead link is terminal: stop sending, latch, exit cleanly (§8.2) |
| **Silent channel-map typo** — duplicate `rf` last-wins, duplicate `px4` is a typo | bridge refuses to start and names both entries (§5) |
| **Physics falling behind wall time, invisibly** — PX4 timestamps on arrival, so the vehicle flies in slow motion with entirely correct-looking sensor values | realtime-factor monitor, printed as `rtf=` and warned outside 0.95–1.05 (§7) |
| WiFi to RealFlight | wired / same host only |
| Bridge lives on wrong side | run bridge on (or wired-adjacent to) the RealFlight machine; SOAP RTT dominates — MAVLink to PX4 tolerates more latency |

### 11.1 Not implemented

Three things earlier drafts of this ledger listed as unsolved. All three now ship:

- ~~**RC passthrough.**~~ **Implemented** — `rcin[12]` is packed into `RC_CHANNELS` and sent at
  50 Hz (§8.2). The "InterLink RC ch2 is reversed" advice therefore applies again, and is
  restated there.
- ~~**Battery / ICE fuel telemetry.**~~ **Implemented** — see §11.2.
- ~~**Multi-instance from `make`.**~~ **Implemented** — see §11.3.

### 11.2 Battery and fuel telemetry

RealFlight's battery is now carried into PX4's `battery_status`, replacing the synthetic
`battery_simulator` ramp. The route is not the obvious one, and the two dead ends are worth
recording because both look like they should work:

- **Not the simulator link.** `SimulatorMavlink::handle_message()` switches on exactly ten message
  ids (`HIL_SENSOR`, `HIL_OPTICAL_FLOW`, `ODOMETRY`, `VISION_POSITION_ESTIMATE`,
  `DISTANCE_SENSOR`, `HIL_GPS`, `RC_CHANNELS`, `LANDING_TARGET`, `HIL_STATE_QUATERNION`,
  `RAW_RPM`) with no `BATTERY_STATUS` case and no default. A `BATTERY_STATUS` sent down TCP 4560 is
  parsed and silently discarded. `simulator_mavlink` only ever *subscribes* to `battery_status`.
- **Not HITL, at all.** On a real board `MavlinkReceiver::handle_message_hil_sensor()` publishes a
  **hardcoded** battery (`voltage_v = 16.0f`, `current_a = 10.0f`, `remaining = 0.70`) on **every**
  `HIL_SENSOR`. The bridge sends `HIL_SENSOR` at ~250 Hz, so a real reading injected at 2 Hz would
  be overwritten ~125 times between updates. No parameter disables it. Real battery telemetry on a
  HITL board requires patching PX4, which this project does not do.

What is implemented: the bridge opens a second UDP socket and sends `BATTERY_STATUS` at **2 Hz** to
`127.0.0.1:14580+instance`, PX4's API/offboard MAVLink link, where
`MavlinkReceiver::handle_message_battery_status()` does handle it. Two constraints on that path,
both of which silently eat the message if violated:

1. **MAVLink v2 only.** A v1 `BATTERY_STATUS` never reaches the topic, even though id 147 encodes
   fine in v1 and the `CRC_EXTRA` is identical (154). The bridge forces v2 on its channel.
2. **`sysid` must equal PX4's `MAV_SYS_ID`, `compid` must differ from PX4's.** The handler's first
   statement is
   `if ((msg->sysid != mavlink_system.sysid) || (msg->compid == mavlink_system.compid)) return;`.
   `rcS` sets `MAV_SYS_ID` to `px4_instance+1`, so the bridge transmits as sysid `instance+1`,
   compid 200.

The link is chosen deliberately: 14580 pins its remote with `-o`, so PX4 does **not** retarget its
output stream at the sender — a 10-packet burst into it returns 0 packets and 0 bytes,
which is what makes it safe to drive from the ~800 Hz exchange loop. The GCS link (18570) is
**not** used: it has no `-o` and latches its partner address from the first packet it receives, so
sending there would hijack the stream away from a real QGroundControl.

Three sources, selected per frame and logged on change:

| RealFlight reports | `remaining` from | `voltage_v` | Notes |
|---|---|---|---|
| pack voltage > 0 (electric) | linear per-cell SOC, 3.30 V empty → 4.20 V full | measured | cell count inferred once from the first sample; `discharged_mah` coulomb-integrated |
| voltage = −1, fuel > 0 (IC) | `fuel_oz / max_fuel_oz_seen` | **nominal, not measured** | this is the −1 case the ledger used to flag; fuel is the physically meaningful "flying left", and a nominal voltage stops `BAT_V_EMPTY` firing on an armed aircraft |
| neither | 1.0 | nominal | obviously synthetic; logged as such |

Because the bridge is now the single publisher, the four SITL airframes set
`param set-default SIM_BAT_ENABLE 0`; otherwise `battery_simulator` and the bridge alternate on the
topic and the reading flickers between them. If the bridge cannot open its socket it says so, and
`SIM_BAT_ENABLE` should be set back to 1.

Full reasoning lives in `Tools/simulation/flightaxis/flightaxis_bridge/src/battery_link.h`.

### 11.3 Multi-instance

Wired through. `sitl_run.sh` reads **`PX4_FLIGHTAXIS_INSTANCE`** (default 0, validated as a
non-negative integer) and threads it to both ends: the bridge gets it as argv[1] (TCP `4560+i`) and
PX4 is launched with `-i $instance`, which is what makes `px4_instance` non-zero inside `rcS` and
therefore offsets the simulator port, every MAVLink UDP port, and `MAV_SYS_ID`. Instance 0 keeps the
historical `$build_path/rootfs` working directory; further instances use
`$build_path/instance_$i`, following PX4's own `Tools/simulation/sitl_multiple_run.sh`.

    PX4_FLIGHTAXIS_INSTANCE=1 PX4_FLIGHTAXIS_IP=<host> make px4_sitl_nolockstep flightaxis_quad

Note that earlier drafts claimed multi-instance "follows the FlightGear convention". That was true
and useless: PX4's own `Tools/simulation/flightgear/sitl_run.sh` hard-codes `0` in exactly the same
place, so neither runner supported it. In practice each instance still needs its own RealFlight
host — one RealFlight instance serves one aircraft.

## 12. References

- The upstream projects this bridge ports, adapts and reuses code from — including the ground
  truth for the §6–§8 conversions, timing and socket logic, and the integration template — are
  listed with their licences in [`COPYRIGHT.md`](COPYRIGHT.md).
- `realflight-bridge` Rust crate — SOAP perf notes (WiFi can't hold 200 Hz)
- xuhao1/RealFlightBridge protocol doc; F16Capstone `flightaxis.py` (historical PoC)
- PX4 docs: Simulation → Simulator MAVLink API (TCP 4560 message set)
