# FlightAxis → PX4 Integration Specification

**RealFlight (FlightAxis Link) as a PX4 SITL simulator, integrated the same way as the FlightGear bridge: files live in the PX4 tree, get compiled by the PX4 make system, and launch with one `make` command.**

Version: 3.1 — July 2026
- v3.1: reconciled with the shipped implementation. Registration moved to the **PX4 v1.16 pattern** (`src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake`, §3); baro/pitot units and the baro datum corrected (§6); DISTANCE_SENSOR and RAW_RPM documented, RC passthrough marked not implemented (§8.2); model-JSON example replaced with the real shipped channel maps and the rule that generates them (§5); airframe example marked abridged (§9); pitfalls ledger pruned of stale rows and extended with the fixes actually made (§12); validation status stated explicitly (§11).
- v3.0: restructured around the **FlightGear-bridge pattern** (`Tools/simulation/` standalone bridge + build-system registration + `sitl_run.sh` + per-model JSON) — build it with the PX4 you already have, no PX4 stack modification.
- v2.0: all frame conversions & timing verified against ArduPilot `SIM_FlightAxis.cpp` source (§6–§7 — unchanged, still the ground truth).
- v1.0: external-bridge protocol groundwork.

> **This is a design document — it explains *why*.** For installation, day-to-day
> use, prerequisites, and troubleshooting, see [`README.md`](README.md), which is
> the **user manual and the authoritative document wherever the two disagree**.

Supported PX4: **tested against v1.16.0.** See §3 for v1.13–v1.15.

---

## 1. The Pattern (copied from FlightGear)

The FlightGear integration proves the template. Side by side:

| Piece | FlightGear | FlightAxis (this spec) |
|---|---|---|
| Bridge app | `Tools/simulation/flightgear/flightgear_bridge/` (standalone C++, own CMake) | `Tools/simulation/flightaxis/flightaxis_bridge/` |
| Sim link | UDP generic protocol ports 15200/15300 | SOAP/HTTP TCP **18083** (RealFlight) |
| PX4 link | MAVLink HIL over **TCP 4560** (`px4_communicator.cpp`) | identical — **reuse `px4_communicator.cpp` nearly verbatim** |
| Sim launcher | `FG_run.py` starts fgfs with args | `FA_check.py` (RealFlight runs on Windows — bridge just connects; script only pings 18083) |
| Model config | `models/<name>.json` (FgModel, Controls[]) | `models/<name>.json` (channel map, scale, options) |
| Runner | `Tools/simulation/flightgear/sitl_run.sh` | `Tools/simulation/flightaxis/sitl_run.sh` |
| Make registration | `simulator_mavlink/sitl_targets_flightgear.cmake` | `simulator_mavlink/sitl_targets_flightaxis.cmake` (§3) |
| Build target | `make px4_sitl_nolockstep flightgear_rascal` | `make px4_sitl_nolockstep flightaxis_plane` |
| Airframe script | `ROMFS/.../init.d-posix/airframes/*_flightgear_rascal` | `*_flightaxis_plane` etc. |

Key consequences:

- **No PX4 source modification.** PX4 sees a standard Simulator-MAVLink simulator on TCP 4560. The bridge is just compiled *alongside* PX4 by its make system (ExternalProject), exactly like `build_flightgear_bridge`.
- **`nolockstep` board already exists** — FlightGear uses it because a real-time sim can't be stepped; RealFlight is the same. Nothing new to add in `boards/`.
- Integration is visible on the console from the runner + bridge. The block below is an
  **illustrative sketch of the expected shape of that output, not a captured transcript** —
  for what has actually been observed and what has not, see the validation status at the end of §11.
  ```
  $ make px4_sitl_nolockstep flightaxis_plane
  ...
  FlightAxis setup
  [flightaxis_bridge] connecting to RealFlight at 192.168.10.1:18083
  [flightaxis_bridge] controller injected, aircraft reset
  [flightaxis_bridge] waiting for PX4 on TCP 4560 ... connected
  [flightaxis_bridge] 247.3/250.0 FPS avg=248.1 glitches=0
  INFO  [simulator_mavlink] Simulator connected on TCP port 4560
  ```

> **PX4 version note:** this pattern requires only the Simulator-MAVLink SITL path (TCP 4560)
> and a place to hang an `ExternalProject` plus the `make` targets. **v1.16 is what this repo
> is developed and tested against**, and there each simulator owns a
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
│   └── flightaxis_bridge/
│       ├── CMakeLists.txt
│       ├── FA_check.py                               # sanity-ping 18083 before start
│       ├── get_FAbridge_params.py                    # JSON → argv (FG pattern)
│       ├── cmake/
│       │   └── FindMAVLink.cmake                     # COPY from flightgear_bridge — REQUIRED
│       ├── models/
│       │   ├── plane.json
│       │   ├── quad.json
│       │   ├── quadplane.json
│       │   └── heli.json
│       └── src/
│           ├── flightaxis_bridge.cpp                 # main loop
│           ├── px4_communicator.{cpp,h}              # ADAPTED from flightgear_bridge (TCP 4560, HIL msgs)
│           ├── fa_communicator.{cpp,h}               # SOAP client + parser (replaces fg_communicator)
│           ├── vehicle_state.{cpp,h}                 # RF→NED conversions (§6) + sensor synth (§7)
│           └── geo_mag_declination.{cpp,h}           # COPY from flightgear_bridge
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

Copied verbatim from the FlightGear bridge (byte-identical): `geo_mag_declination.{cpp,h}` and
`cmake/FindMAVLink.cmake`. The latter is not optional — the bridge's `CMakeLists.txt` puts
`./cmake` on `CMAKE_MODULE_PATH` and `find_package(MAVLink)` is what supplies the MAVLink
headers out of the PX4 build, so MAVLink is never a separate dependency to install.
`px4_communicator.{cpp,h}` is **adapted**, not copied: the FlightGear original emits every
message on every frame, which at RealFlight's ~250 Hz would swamp PX4, so it gained
per-message decimation state and intervals (`px4_communicator.h:81-92`,
`px4_communicator.cpp:182-239`) and the DISTANCE_SENSOR path (§8.2).

---

## 3. Build-System Registration — `sitl_targets_flightaxis.cmake` (PX4 v1.16)

In v1.16 each simulator owns one self-contained file under
`src/modules/simulation/simulator_mavlink/`, included from that directory's `CMakeLists.txt`.
FlightAxis follows the `sitl_targets_flightgear.cmake` shape exactly. The whole file guards on
`ENABLE_LOCKSTEP_SCHEDULER STREQUAL "no"`, so the targets only exist on the `nolockstep` board —
which is correct, RealFlight free-runs and cannot be stepped.

`src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake`:

```cmake
if(ENABLE_LOCKSTEP_SCHEDULER STREQUAL "no")

	# RealFlight runs on a remote Windows machine, so unlike FlightGear/jMAVSim
	# there is no local simulator binary to find_program() for.

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
  left slow — ArduPilot builds its own SITL at `-O3` for the same reason.
- **`WORKING_DIRECTORY ${SITL_WORKING_DIR}`** on the run target, so the `rootfs` and logs land
  where every other PX4 SITL target puts them.

The shipped file additionally warns at configure time if a listed model has no
`models/<model>.json` or no matching `*_flightaxis_<model>` airframe — both are otherwise
silent misconfigurations that only surface as a Python traceback at launch.

### 3.1 PX4 v1.13–v1.15 — `platforms/posix/cmake/sitl_target.cmake`

**Alternative, for older trees only.** `platforms/posix/cmake/sitl_target.cmake` does not exist
in v1.16. In v1.13–v1.15 it is where every simulator was registered centrally, so there the same
three pieces are spliced into that one file instead — mirroring the `flightgear` entries:

```cmake
# 1. add the simulator name to the sim list
set(simulators ... flightgear flightaxis ...)

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

Exact splice points differ slightly per release — diff against how `flightgear` appears in
*your* checkout's `sitl_target.cmake` and replicate every occurrence. `install.sh` targets the
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

Same skeleton as the FG bridge. Eigen3 is the only external package it needs; MAVLink arrives
via the copied `FindMAVLink.cmake`, and the shipped file also falls back to the FlightGear
bridge's own `cmake/` directory so an in-tree checkout works even if the copy is missing:

```cmake
cmake_minimum_required(VERSION 3.10)
project(flightaxis_bridge)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")   # FindMAVLink from FG bridge
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

---

## 4. Runner — `Tools/simulation/flightaxis/sitl_run.sh`

Adapted line-for-line from the FlightGear one; the difference is RealFlight isn't launched (it lives on the Windows gaming box) — we only verify reachability. Abridged below (the shipped script also echoes its arguments and the SITL command):

```bash
#!/usr/bin/env bash
set -e
if [ "$#" -lt 4 ]; then echo "usage: sitl_run.sh sitl_bin model src_path build_path"; exit 1; fi

sitl_bin="$1"; model="$2"; src_path="$3"; build_path="$4"

rootfs="$build_path/rootfs"; mkdir -p "$rootfs"
export PX4_SIM_MODEL=flightaxis_${model}

# RealFlight host: env override, default localhost (RealFlight in a VM / same box)
FA_IP="${PX4_FLIGHTAXIS_IP:-127.0.0.1}"

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

"${build_path}/build_flightaxis_bridge/flightaxis_bridge" 0 "${FA_IP}" \
    $fa_bridge_params &
FA_BRIDGE_PID=$!

pushd "$rootfs" >/dev/null
set +e
eval "\"$sitl_bin\" \"$build_path\"/etc"
popd >/dev/null
kill $FA_BRIDGE_PID
```

Run experience:

```bash
# RealFlight running on Windows box 192.168.10.1, FlightAxis Link enabled
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_plane
# QGC connects on UDP 14550 as usual
```

**Multi-instance: supported by the bridge, not reachable from `make`.** The first bridge argv is
the PX4 instance id and it does offset the listen port (`flightaxis_bridge.cpp:239,302`;
`px4_communicator.cpp:59`, `portBase+portOffset`). But `sitl_run.sh:47` passes a hard-coded `0`
with no environment override, so **no `make` target can start a second instance** — a second
`flightaxis_*` target would just collide on TCP 4560. Running two aircraft at once today means
invoking the bridge binary by hand with a different instance id (and pointing each at its own
RealFlight host — one RealFlight instance serves one aircraft anyway). Threading an
`PX4_FLIGHTAXIS_INSTANCE`-style variable through `sitl_run.sh` is the obvious fix and is not
done.

---

## 5. Model JSON — the "any airframe" mechanism

FG's `models/*.json` carries `FgModel` + `Controls` triplets; ours carries the RealFlight channel map. `get_FAbridge_params.py` flattens it to argv exactly like `get_FGbridge_params.py` does.

Below is the **shipped `models/quadplane.json`** (comments trimmed). The `px4` indices are *not*
sequential, and that is the whole point of the file — see the rule that follows.

```json
{
  "Comment":   "reference-class quadplane on a RealFlight custom model",
  "RfModel":   "informational only - aircraft is selected inside RealFlight",
  "Options":   ["ResetPosition"],
  "Channels": [
    {"rf": 0, "px4": 5, "scale": "bipolar",  "reverse": false, "comment": "aileron      <- controls[5] aileron left"},
    {"rf": 1, "px4": 7, "scale": "bipolar",  "reverse": true,  "comment": "elevator     <- controls[7]"},
    {"rf": 2, "px4": 4, "scale": "unipolar", "disarm": 0.0,    "comment": "fwd throttle <- controls[4] pusher motor"},
    {"rf": 3, "px4": 8, "scale": "bipolar",  "reverse": false, "comment": "rudder       <- controls[8]"},
    {"rf": 4, "px4": 0, "scale": "unipolar", "disarm": 0.0,    "comment": "lift motor 1 <- controls[0]"},
    {"rf": 5, "px4": 1, "scale": "unipolar", "disarm": 0.0,    "comment": "lift motor 2 <- controls[1]"},
    {"rf": 6, "px4": 2, "scale": "unipolar", "disarm": 0.0,    "comment": "lift motor 3 <- controls[2]"},
    {"rf": 7, "px4": 3, "scale": "unipolar", "disarm": 0.0,    "comment": "lift motor 4 <- controls[3]"}
  ],
  "UnmappedDefault": 0.5
}
```

**The rule that generates the `px4` column.** The JSON does not get to choose these numbers; PX4's
control allocator does, and the JSON must mirror whatever the airframe script sets up:

- `PWM_MAIN_FUNC<N>` in the airframe script lands on `HIL_ACTUATOR_CONTROLS.controls[N-1]`.
- Function value `101+k` is `CA_ROTOR<k>` — **a motor, range [0,1]** → `"scale": "unipolar"`.
- Function value `201+k` is `CA_SV_CS<k>` — **a control surface, range [-1,1]** → `"scale": "bipolar"`.
- The allocator emits motors before servos, so on any mixed airframe the motors occupy the low
  `controls[]` indices regardless of where they sit on the RealFlight transmitter. That is exactly
  why the quadplane's RealFlight channel order (aileron, elevator, throttle, rudder, then the four
  lift motors — the ArduPilot RealFlight convention) maps to the scattered PX4 order
  `5, 7, 4, 8, 0, 1, 2, 3`.

Each shipped airframe script carries this derivation as a comment block next to its
`PWM_MAIN_FUNC*` lines; that block and the JSON must be edited together. The other three models:

| Model | RealFlight channel order | `px4` indices | Note |
|---|---|---|---|
| `plane` | aileron, elevator, throttle, rudder | `1, 3, 0, 4` | `controls[0]` is the motor, so throttle is `rf2 -> px4 0` |
| `quad` | motors 1–4 | `0, 1, 2, 3` | the only model where the orders coincide |
| `quadplane` | aileron, elevator, throttle, rudder, lift 1–4 | `5, 7, 4, 8, 0, 1, 2, 3` | as above |
| `heli` | swash 1–3, tail, main | `2, 3, 4, 1, 0` | `CA_AIRFRAME 10` orders motors first: `controls[0]`=main rotor, `[1]`=yaw tail, `[2..4]`=swash |

Two heli-specific traps, both documented inside `heli.json`: the **tail rotor is unipolar**, not
bipolar, because under `CA_AIRFRAME 10` the yaw tail is a *motor* and PX4 has already normalised it
to `[0,1]` — scaling it as bipolar would fold `[0,1]` into `[0.5,1.0]` and leave yaw permanently
offset. And `HeliDemix` only inverts correctly for the swash geometry the airframe forces via
`CA_SP0_ANG*` (§12).

- `px4` = index into `HIL_ACTUATOR_CONTROLS.controls[]` (defined by the airframe's actuator config, visible in the QGC Actuators tab — the JSON must mirror it).
- Scaling: `unipolar` = clamp(v,0,1) for motors (HIL controls are 0..1 for motors); `bipolar` = (v+1)/2 for surfaces (−1..1). Net effect identical to ArduPilot's `(pwm−1000)/1000`. `reverse` is applied *after* scaling, as `v -> 1-v`.
- `disarm` = the value sent while PX4 is disarmed or the control is NaN. **`disarm: -1` (also the default when the key is absent) means "hold the last output"** rather than driving the channel anywhere — right for control surfaces, wrong for motors, which is why every motor row states `"disarm": 0.0` explicitly (`flightaxis_bridge.cpp:180-187`).
- **Duplicate indices are rejected at startup, not tolerated.** Two rows sharing an `rf` index would silently last-wins in `buildChannels()`, and a repeated `px4` index is almost always a typo, so the bridge refuses to start on either and names the two offending entries (`flightaxis_bridge.cpp:269-289`).
- `UnmappedDefault` is the value sent on every RealFlight channel the map does not mention.
- `Options` (port of ArduPilot `SIM_FLTAX_OPTS` bits, flattened to a bitmask by `get_FAbridge_params.py`): `ResetPosition` (=1, ResetAircraft on startup, default), `Rev4Servos` (=2, swap ch1–4 ↔ 5–8 wholesale for RF models built that way — do **not** combine it with an already-reordered map like the quadplane's or you double-swap), `HeliDemix` (=4, swash servos → RF roll/pitch/collective: `roll=s1−s2`, `pitch=(s1+s2)/2−s3`, `col=(s1+s2+s3)/3`, recentered 0..1), `SilenceFPS` (=8).
- Adding a new aircraft takes **four** steps, not three: new JSON, one model name in the `models` list in `sitl_targets_flightaxis.cmake`, one airframe script — and that airframe must also be added to `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` (§3.2) or it never reaches the ROMFS.

RealFlight model prep (once per aircraft, in the RF editor): strip expo/mixes/gyros, max servo speed, one channel per actuator. Tridge's ArduPilot RealFlight model collection is directly reusable.

---

## 6. Frame Conversions — verified from ArduPilot `SIM_FlightAxis.cpp`

These go in `vehicle_state.cpp`. They are line-for-line what ArduPilot ships — not inferred. RealFlight's conventions are internally inconsistent (position swapped, velocity not); follow literally.

**Quaternion (RF → NED):** `q_ned = (w=W, x=RF_Y, y=RF_X, z=−RF_Z)` — swap X↔Y, negate Z.

**Gyro:** `p=+roll_rate, q=+pitch_rate, r=−yaw_rate` (deg→rad, constrain ±2000 °/s). **Only yaw negated.**

**Position (NED):** `(N=RF_Y, E=RF_X, D=−altASL)`. Capture `position_offset` on first frame and after every `m-resetButtonHasBeenPressed`; subtract it so PX4's home (bridge argv / `PX4_HOME_*`) anchors the RF world. NED→LLA for HIL_GPS.

**World velocity:** `(vN=U, vE=V, vD=W)` — **used directly, no swap** (asymmetric with position; it's what works).

**Wind:** swapped like position: `(windY, windX, windZ)`.

**Accelerometer (specific force):** in flight use `m-accelerationBodyA*` directly. On ground it's garbage (ArduPilot comment: *"accel on the ground is nasty in realflight, and prevents helicopter disarm"*) — override:

```cpp
if (touching_ground) {
    accel_ef  = (velocity_ef - last_velocity_ef) / dt;
    accel_ef.z -= 9.80665f;
    accel_body = R_ned_to_body * accel_ef;      // yields exactly (0,0,-g) at rest
}
clamp(accel_body, ±16 g);
```

**Pitot airspeed — computed, not read:** `m_airspeed_MPS` is total TAS; a pitot wants body-X:

```cpp
airspeed_pitot = max( (R_ned_to_body * (velocity_ef - wind_ef)).x , 0 );
diff_pressure  = 0.5f * 1.225f * airspeed_pitot * airspeed_pitot / 100.0f;   // Pa -> hPa
```

Critical for 3D/high-alpha models — total airspeed lies during hover/harrier/knife-edge.

**Pressure units are hPa, not Pa.** Both `HIL_SENSOR.abs_pressure` and `HIL_SENSOR.diff_pressure`
are hectopascals in MAVLink, so the `/100` above is not cosmetic — dropping it feeds PX4 an
airspeed error of a factor of 10. Declared as hPa at `vehicle_state.h:126,128`.

**Rangefinder:** `AGL / dcm.c.z` when `dcm.c.z > 0`, else invalid (inverted). This is not
computed and discarded: when valid it is packed into a `DISTANCE_SENSOR` message and sent to PX4
at 20 Hz (§8.2), which is what makes terrain-relative modes and landing usable.

**Magnetometer:** WMM field at home via `geo_mag_declination.cpp` (already in the FG bridge — copy), rotated to body with the quaternion above.

**Barometer:** ISA, `p = 101325·(1 − 2.25577e-5·h)^5.25588` Pa, converted to **hPa**. Temp ~25 °C.

The altitude `h` fed into that formula is **deliberately not RealFlight's ASL**. It is
`h = home_alt − position_ned.z()` (`vehicle_state.cpp:202-209`) — the same datum `nedToLLA()`
uses for the GPS altitude it reports (`:267`). The reason is that a RealFlight runway sits at
whatever arbitrary ASL its scenery author chose, which has nothing to do with the home altitude
PX4 was given. Deriving baro from RF ASL while deriving GPS from `home_alt` would present EKF2
with a *constant* baro-vs-GPS height disagreement — the estimator would either fight it forever
or reject one of the two sources. Anchoring both to the same datum makes the two agree by
construction; the cost is that the baro no longer encodes the scenery's real elevation, which
nothing downstream cares about.

---

## 7. Timing — extrapolation & glitch handling (from ArduPilot `update()`)

RealFlight free-runs (~250 Hz). The FG bridge already upsamples to dodge PX4's stale-sensor detection; we port ArduPilot's stronger three-branch logic, keyed on `m-currentPhysicsTime-SEC`:

1. **dt < 0** → RealFlight restarted: re-base initial time, zero position offset, continue.
2. **dt < 1e-5 s** (same physics frame — bridge outran RF): do **not** resend an identical HIL_SENSOR. Extrapolate in 1 ms steps (propagate attitude by `q⊗exp(½ω·δt)`, hold accel/gyro), never beyond `average_frame_time` (EMA `0.98/0.02`).
3. **normal frame** → full pipeline, then glitch compensation: if physics time jumped 50 ms–2 s (network hiccup), swallow the excess by advancing the epoch (`initial_time += dt − 50 ms`), cap dt at 50 ms, count the glitch. Backwards >500 ms = true reset, accept.

Timestamps in HIL messages = physics time (µs since epoch capture). Watch `m-currentPhysicsSpeedMultiplier ≠ 1` → warn. Print the ArduPilot-style FPS line every 1000 frames.

---

## 8. Bridge Internals

### 8.1 `fa_communicator` — SOAP client (port of ArduPilot's socket logic)

- **New TCP connection per request**, latency hidden by a background *socket-creator thread* that always has the next connected socket parked (100 ms connect timeout, condition-variable handoff). This is ArduPilot's proven pattern — not keep-alive.
- Reply read: find `Content-Length`, drain to `\r\n\r\n`+length, close socket.
- Parser: sequential `strstr` scan over the key table **in document order** (12 `item` echoes first, then all `m-*` fields), `true/false→1/0`, `atof`. Any missing key → flag re-init (schema/version change self-heals).
- Startup sequence (exact order): `RestoreOriginalControllerDevice` → (`ResetAircraft` if ResetPosition) → `InjectUAVControllerInterface`.
- Re-run startup whenever no socket AND (`!controller_started` OR `m-flightAxisControllerIsActive==0` OR `m-resetButtonHasBeenPressed`) — makes aircraft-change and spacebar in RealFlight self-heal.
- `m-selectedChannels`: send **0 until PX4 is up** (first HIL_ACTUATOR_CONTROLS received), then 4095. RealFlight holds neutral meanwhile.

### 8.2 `px4_communicator` — adapted from the FG bridge

TCP server on 4560+instance; PX4 (`simulator_mavlink`) connects. Receives `HIL_ACTUATOR_CONTROLS`
(armed flag in `mode`; NaN/disarmed → JSON `disarm` values).

`Send()` runs at the full RealFlight frame rate plus every 1 ms extrapolation step (§7), which is
far faster than most of these messages need. Only `HIL_SENSOR` genuinely wants that rate, so
everything else is decimated against the bridge's own physics clock — the interval constants live
at `px4_communicator.h:86-88` and the dispatch at `px4_communicator.cpp:182-250`. One of each is
forced out on the first frame so PX4 has a complete picture immediately:

| Message | Rate | Notes |
|---|---|---|
| `HIL_SENSOR` | every frame / extrapolation step (~250 Hz+) | `fields_updated=0x1FFF`; pressures in hPa (§6) |
| `HIL_GPS` | **10 Hz** (`GPS_INTERVAL_US`) | COG = `atan2(vE,vN)`, **not** RF azimuth. PX4 publishes one `sensor_gps` per message with no rate limiting of its own, hence the decimation |
| `HIL_STATE_QUATERNION` | **50 Hz** (`STATE_QUAT_INTERVAL_US`) | ground truth only — for log analysis, not consumed by the estimator |
| `DISTANCE_SENSOR` | **20 Hz** (`DISTANCE_INTERVAL_US`) | built in `vehicle_state.cpp:362-382`; downward-facing (`PITCH_270`), 0.1–40 m, cm units. **Gated on `rangefinderValid()`** — suppressed entirely while the aircraft is inverted rather than sent as a bogus reading |
| `RAW_RPM` | every frame | index 0, from RealFlight's prop/rotor RPM |

**RC passthrough — NOT IMPLEMENTED (future work).** The 12 `item` values echoed in every
ExchangeData reply *are* the physical InterLink TX channels, and the bridge does parse them into
`FAState.rcin[12]` (`fa_communicator.cpp:499`) — but nothing ever reads that array, and no
`HIL_RC_INPUTS_RAW` is constructed or sent anywhere in the bridge. **You cannot currently fly PX4
Manual/Acro from the RealFlight transmitter.** The data is one short function away from being
usable, which is why the field is populated; if it is wired up, note that ArduPilot ships
`RC2_REVERSED=1` for the InterLink, so expect to reverse ch2 in RC calibration.

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
writing a new one; each carries a comment block deriving its `controls[]` indices, which must
stay in step with the matching `models/<name>.json` (§5).

```sh
#!/bin/sh
# @name FlightAxis Plane
# @type Plane
. ${R}etc/init.d/rc.fw_defaults

# RealFlight free-runs (no lockstep), GPS samples arrive fresh.
param set EKF2_GPS_DELAY 0

# ... 60 more lines: CA_* geometry, PWM_MAIN_FUNC* assignments, FW tuning ...
```

Note `param set`, **not** `param set-default`. All four airframes need it: `px4-rc.mavlinksim`
runs *after* the airframe script and issues its own `param set-default EKF2_GPS_DELAY 10`
(`px4-rc.mavlinksim:5`), which would quietly win over a default set here. A plain `param set`
takes precedence and survives. The value itself is 0 because RealFlight free-runs and the GPS
samples the bridge synthesises are current, with none of the transport delay the 10 ms default
compensates for.

---

## 10. Implementation Order (each step testable)

1. Copy FG bridge folder → rename; keep `px4_communicator`, `geo_mag_declination`; stub out `fg_communicator`.
2. `fa_communicator`: standalone test against RealFlight — print physics time ≥200 FPS. (No PX4 involved yet.)
3. `vehicle_state` conversions → HIL_SENSOR; run PX4, check `listener sensor_accel` / QGC attitude mirrors RealFlight.
4. Actuator path: QGC Actuators sliders move RF surfaces through the JSON map.
5. Timing hardening (§7) + reconnect state machine.
6. Options (Rev4/HeliDemix), remaining model JSONs, `sitl_targets_flightaxis.cmake` polish. (RC passthrough was planned here and is not done — §8.2.)

## 11. Validation Checklist

1. **Static:** level on runway → accel (0,0,−9.81) with zero jitter (proves ground override), gyro 0, heading = RF compass; nose-up 90° clean (quaternion path, no Euler singularity).
2. **Rates:** roll right p>0, pull up q>0, yaw right r>0 (only yaw negated from RF).
3. **Taxi** north/east → local N/E and vN/vE track; QGC map mirrors RF world.
4. **High alpha:** hover a 3D model → pitot ≈ 0 while `m_airspeed` isn't (proves §6 pitot).
5. **EKF:** innovations bounded over a manual circuit; groundtruth-vs-estimate in ulog.
6. **Resilience:** spacebar in RF (auto re-inject + re-offset), aircraft change in RF (auto restart), 1 s network pull (glitch counter++, no EKF time-jump fault), kill bridge mid-flight (PX4 sensor-timeout failsafe, not a hang).

### Validation status

The six items above are the *target*. This is where they actually stand. The distinction matters
because RealFlight is Windows-only and no Windows machine has been in the loop yet. (README.md
carries the same status in shorter form; keep the two in step.)

**Verified here:**

- The bridge builds, and the `flightaxis_{plane,quad,quadplane,heli}` make targets exist and resolve.
- All four model JSONs flatten cleanly through `get_FAbridge_params.py`.
- `FA_check.py` fails correctly, with its diagnostic, against an unreachable host.
- **End-to-end against a mock FlightAxis server** (a local SOAP responder replaying the RealFlight
  schema): PX4 connects on TCP 4560, EKF2 converges, and the synthesised sensors are sane — in
  particular baro and GPS altitudes agree, confirming the shared-datum choice in §6.
- Channel maps correct end to end for **plane and quad**, including bipolar/reverse/unipolar
  scaling and the disarm values.
- The resilience cases (item 6): reconnect, aircraft reset, glitch swallow, and sim death all
  behave as §7 and §8.1 describe.

**Still requires a real Windows RealFlight machine** — untested, and not to be presented otherwise:

- Item 2, the rate sign conventions (roll/pitch/yaw polarity out of RealFlight).
- Item 3, taxi N/E tracking against the RF world.
- Item 4, high-alpha pitot behaviour.
- The compass-heading and nose-up-90° parts of item 1.
- Item 5 — no circuit has been flown, so EKF innovation bounds over a real manoeuvre are unknown.
- The quadplane and heli channel maps beyond static reasoning, and `HeliDemix` against a real swash.

## 12. Pitfalls Ledger

| Pitfall | Fix |
|---|---|
| Lockstep vs free-running RF | use the existing `px4_sitl_nolockstep` target (FG precedent) |
| Duplicate physics frames → zero-dt sensors | §7 branch 2 extrapolation |
| Network hiccup → EKF time jump | 50 ms glitch swallow |
| Ground-contact accel noise | finite-difference override (§6) |
| Pitot fed total airspeed | body-X from velocity−wind (§6) |
| Position swapped, velocity not, wind swapped | §6 literally |
| Channels sent before PX4 ready | selectedChannels 0 → 4095 after first actuator msg |
| Aircraft change / spacebar mid-session | reconnect state machine (§8.1) |
| Keep-alive assumption | per-request sockets + creator thread (§8.1) |
| **Baro-vs-GPS datum mismatch** — RF runway sits at an arbitrary scenery ASL, so baro from RF ASL disagrees with GPS from `home_alt` by a constant, forever | derive both from `home_alt − position_ned.z` (`vehicle_state.cpp:202-206`, §6) |
| **Pressures sent in Pa** — MAVLink `HIL_SENSOR` wants hPa | `/100` on both `abs_pressure` and `diff_pressure` (§6) |
| **`EKF2_GPS_DELAY` silently clobbered** — `px4-rc.mavlinksim` runs after the airframe and re-defaults it to 10 | use `param set`, not `param set-default`, in the airframe (§9) |
| **Heli swash demix cross-couples roll into pitch** — PX4's default `CA_SP0_ANG*` 0/140/220 puts the mirrored servo pair on 2 and 3, which the `HeliDemix` inverse does not expect | force 300/60/180 in the airframe (`1203_flightaxis_heli:40-53`) |
| **Heli yaw permanently offset** — under `CA_AIRFRAME 10` the tail is a *motor*, already `[0,1]`; scaling it bipolar folds it into `[0.5,1.0]` | `rf3` is `unipolar` in `heli.json` (`heli.json:4`) |
| **Quadplane trips the actuator failure detector** — lift motors idle in cruise and SITL reports 0 A per ESC, which reads as a dead motor | `FD_ACT_EN 0` (`1202_flightaxis_quadplane:15-17`) |
| **Silent channel-map typo** — duplicate `rf` last-wins, duplicate `px4` is a typo | bridge refuses to start and names both entries (`flightaxis_bridge.cpp:269-289`, §5) |
| WiFi to RealFlight | wired / same host only |
| Bridge lives on wrong side | run bridge on (or wired-adjacent to) the RealFlight machine; SOAP RTT dominates — MAVLink to PX4 tolerates more latency |

### 12.1 Not implemented

Three things earlier drafts of this ledger listed as solved. They are not, and nothing downstream
should assume them:

- **RC passthrough.** `rcin[12]` is parsed and never read; no `HIL_RC_INPUTS_RAW` is sent (§8.2).
  The old "InterLink RC ch2 is reversed" row was advice for a feature that does not exist.
- **Battery / ICE fuel telemetry.** `m-batteryVoltage-VOLTS` and `m-batteryCurrentDraw-AMPS` are
  parsed into `FAState` (`fa_communicator.cpp:518-519`) but are not carried into any MAVLink
  message, so PX4 sees no simulated battery at all and the old "clamp/synthetic for ICE models
  reporting −1" fix has nothing to attach to. PX4 falls back to its own SITL battery model.
- **Multi-instance from `make`.** Supported by the bridge, unreachable through `sitl_run.sh` (§4).

## 13. References

- ArduPilot `SIM_FlightAxis.{h,cpp}` — ground truth for §6–§8 conversions, timing, socket & reconnect logic
- PX4-FlightGear-Bridge (`Tools/simulation/flightgear/`) — the integration template: `sitl_run.sh`, `CMakeLists.txt`, `px4_communicator.cpp`, `geo_mag_declination.cpp`, JSON model system, `sitl_targets_*.cmake` registration, README workflow
- `realflight-bridge` Rust crate — SOAP perf notes (WiFi can't hold 200 Hz)
- xuhao1/RealFlightBridge protocol doc; F16Capstone `flightaxis.py` (historical PoC)
- PX4 docs: Simulation → Simulator MAVLink API (TCP 4560 message set)
- Example ExchangeData dump: uav.tridgell.net/RealFlight/data-exchange.txt
