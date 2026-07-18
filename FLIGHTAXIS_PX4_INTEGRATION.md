# FlightAxis → PX4 Integration Specification

**RealFlight (FlightAxis Link) as a PX4 SITL simulator, integrated the same way as the FlightGear bridge: files live in the PX4 tree, get compiled by the PX4 make system, and launch with one `make` command.**

Version: 3.0 — July 2026
- v3.0: restructured around the **FlightGear-bridge pattern** (`Tools/simulation/` standalone bridge + `sitl_target.cmake` registration + `sitl_run.sh` + per-model JSON) — build it with the PX4 you already have, no PX4 stack modification.
- v2.0: all frame conversions & timing verified against ArduPilot `SIM_FlightAxis.cpp` source (§6–§7 — unchanged, still the ground truth).
- v1.0: external-bridge protocol groundwork.

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
| Make registration | `platforms/posix/cmake/sitl_target.cmake` | same file, new sim entry |
| Build target | `make px4_sitl_nolockstep flightgear_rascal` | `make px4_sitl_nolockstep flightaxis_plane` |
| Airframe script | `ROMFS/.../init.d-posix/airframes/*_flightgear_rascal` | `*_flightaxis_plane` etc. |

Key consequences:

- **No PX4 source modification.** PX4 sees a standard Simulator-MAVLink simulator on TCP 4560. The bridge is just compiled *alongside* PX4 by its make system (ExternalProject), exactly like `build_flightgear_bridge`.
- **`nolockstep` board already exists** — FlightGear uses it because a real-time sim can't be stepped; RealFlight is the same. Nothing new to add in `boards/`.
- Console proof of integration comes from the runner + bridge:
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

> **PX4 version note:** this pattern requires the Simulator-MAVLink SITL path (TCP 4560) and the `sitl_target.cmake` third-party-sim hooks — present through v1.13/v1.14/v1.15-era trees (the same ones that carried the FlightGear/JSBSim bridges). On the newest mains where those bridges were moved out-of-tree, the identical layout still works; you just keep the folder as a repo overlay (git submodule in `Tools/simulation/`), which is literally how PX4-FlightGear-Bridge itself is vendored.

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
│       ├── models/
│       │   ├── plane.json
│       │   ├── quad.json
│       │   ├── quadplane.json
│       │   └── heli.json
│       └── src/
│           ├── flightaxis_bridge.cpp                 # main loop
│           ├── px4_communicator.{cpp,h}              # COPY from flightgear_bridge (TCP 4560, HIL msgs)
│           ├── fa_communicator.{cpp,h}               # SOAP client + parser (replaces fg_communicator)
│           ├── vehicle_state.{cpp,h}                 # RF→NED conversions (§6) + sensor synth (§7)
│           └── geo_mag_declination.{cpp,h}           # COPY from flightgear_bridge
├── platforms/posix/cmake/sitl_target.cmake           # MODIFIED (§3)
└── ROMFS/px4fmu_common/init.d-posix/airframes/
    ├── 1200_flightaxis_plane                         # NEW (numbers per local convention)
    ├── 1201_flightaxis_quad
    ├── 1202_flightaxis_quadplane
    └── 1203_flightaxis_heli
```

Two files are direct copies from the FlightGear bridge (`px4_communicator`, `geo_mag_declination`) — that's roughly half the bridge done before writing a line.

---

## 3. Build-System Registration — `platforms/posix/cmake/sitl_target.cmake`

Mirror the flightgear entries:

```cmake
# 1. add simulator name to the sim list
set(simulators ... flightgear flightaxis ...)

# 2. model list for target generation:  make px4_sitl_nolockstep flightaxis_<model>
set(models_flightaxis plane quad quadplane heli)

# 3. build the bridge alongside PX4 (pattern: build_flightgear_bridge)
include(ExternalProject)
ExternalProject_Add(flightaxis_bridge
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/flightaxis/flightaxis_bridge
	PREFIX ${PX4_BINARY_DIR}/build_flightaxis_bridge
	BINARY_DIR ${PX4_BINARY_DIR}/build_flightaxis_bridge
	INSTALL_COMMAND ""
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
)

# 4. hook the runner (inside the foreach that dispatches sim runners)
elseif(viewer STREQUAL "flightaxis")
	add_custom_target(${_targ_name}
		COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/flightaxis/sitl_run.sh
			$<TARGET_FILE:px4> ${model} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
		DEPENDS px4 flightaxis_bridge
		...)
```

Exact splice points differ slightly per PX4 version — diff against how `flightgear` appears in *your* checkout's `sitl_target.cmake` and replicate every occurrence.

### 3.1 Bridge `CMakeLists.txt`

Same skeleton as the FG bridge:

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

Adapted line-for-line from the FlightGear one; the difference is RealFlight isn't launched (it lives on the Windows gaming box) — we only verify reachability:

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

"${build_path}/build_flightaxis_bridge/flightaxis_bridge" 0 "${FA_IP}" \
    `./get_FAbridge_params.py "models/${model}.json"` &
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

Multi-instance follows the FG convention: first bridge argv is the PX4 instance id (offsets the 4560 port).

---

## 5. Model JSON — the "any airframe" mechanism

FG's `models/*.json` carries `FgModel` + `Controls` triplets; ours carries the RealFlight channel map. `get_FAbridge_params.py` flattens it to argv exactly like `get_FGbridge_params.py` does.

```json
{
  "Comment":   "reference-class quadplane on a RealFlight custom model",
  "RfModel":   "informational only - aircraft is selected inside RealFlight",
  "Options":   ["ResetPosition"],
  "Channels": [
    {"rf": 0, "px4": 4, "scale": "bipolar",  "reverse": false, "comment": "aileron"},
    {"rf": 1, "px4": 5, "scale": "bipolar",  "reverse": true,  "comment": "elevator"},
    {"rf": 2, "px4": 6, "scale": "unipolar", "disarm": 0.0,    "comment": "fwd throttle"},
    {"rf": 3, "px4": 7, "scale": "bipolar",  "comment": "rudder"},
    {"rf": 4, "px4": 0, "scale": "unipolar", "disarm": 0.0},
    {"rf": 5, "px4": 1, "scale": "unipolar", "disarm": 0.0},
    {"rf": 6, "px4": 2, "scale": "unipolar", "disarm": 0.0},
    {"rf": 7, "px4": 3, "scale": "unipolar", "disarm": 0.0}
  ],
  "UnmappedDefault": 0.5
}
```

- `px4` = index into `HIL_ACTUATOR_CONTROLS.controls[]` (defined by the airframe's actuator config in QGC — the JSON must mirror it).
- Scaling: `unipolar` = clamp(v,0,1) for motors (HIL controls are 0..1 for motors); `bipolar` = (±v+1)/2 for surfaces (−1..1). Net effect identical to ArduPilot's `(pwm−1000)/1000`.
- `Options` (port of ArduPilot `SIM_FLTAX_OPTS` bits): `ResetPosition` (ResetAircraft on startup, default), `Rev4Servos` (swap ch1–4 ↔ 5–8 for quadplane RF models), `HeliDemix` (swash servos → RF roll/pitch/collective: `roll=s1−s2`, `pitch=(s1+s2)/2−s3`, `col=(s1+s2+s3)/3`, recentered 0..1), `SilenceFPS`.
- Adding a new aircraft = new JSON + one model-name in `models_flightaxis` in `sitl_target.cmake` + one airframe script — same three steps the FG README documents.

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
diff_pressure  = 0.5f * 1.225f * airspeed_pitot * airspeed_pitot;   // Pa
```

Critical for 3D/high-alpha models — total airspeed lies during hover/harrier/knife-edge.

**Rangefinder:** `AGL / dcm.c.z` when `dcm.c.z > 0`, else invalid (inverted).

**Magnetometer:** WMM field at home via `geo_mag_declination.cpp` (already in the FG bridge — copy), rotated to body with the quaternion above.

**Barometer:** ISA from ASL: `p = 101325·(1 − 2.25577e-5·h)^5.25588` Pa. Temp ~25 °C.

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

### 8.2 `px4_communicator` — reused from FG bridge

TCP server on 4560+instance; PX4 (`simulator_mavlink`) connects. Sends `HIL_SENSOR` (`fields_updated=0x1FFF`) every frame/extrapolation step, `HIL_GPS` decimated to 10 Hz (COG = `atan2(vE,vN)`, **not** RF azimuth), `HIL_STATE_QUATERNION` ground truth; receives `HIL_ACTUATOR_CONTROLS` (armed flag in `mode`; NaN/disarmed → JSON `disarm` values).

Optional RC passthrough: the 12 `item` values echoed in every ExchangeData reply are the physical InterLink TX channels → forward as `HIL_RC_INPUTS_RAW` and fly PX4 Manual/Acro with the real transmitter (note ArduPilot ships `RC2_REVERSED=1` for InterLink — expect to reverse ch2 in RC calibration).

### 8.3 Main loop

```
take parked socket → ExchangeData(channels) → parse
branch on dt (§7) → convert (§6) → HIL_SENSOR [+GPS/GT on schedule]
drain 4560 (non-blocking) → latest HIL_ACTUATOR_CONTROLS → JSON channel map → next channels
```

Loop rate is set by RealFlight's SOAP RTT (~2–5 ms → 200–500 Hz). Wired network or same-host only; WiFi cannot hold 200 Hz.

---

## 9. Airframe Script Example — `1200_flightaxis_plane`

```sh
#!/bin/sh
# @name FlightAxis plane
# @type Plane
. ${R}etc/init.d/rc.fw_defaults
param set-default EKF2_GPS_DELAY 0
# Actuator geometry (QGC Actuators tab) MUST match models/plane.json "px4" indices
```

---

## 10. Implementation Order (each step testable)

1. Copy FG bridge folder → rename; keep `px4_communicator`, `geo_mag_declination`; stub out `fg_communicator`.
2. `fa_communicator`: standalone test against RealFlight — print physics time ≥200 FPS. (No PX4 involved yet.)
3. `vehicle_state` conversions → HIL_SENSOR; run PX4, check `listener sensor_accel` / QGC attitude mirrors RealFlight.
4. Actuator path: QGC Actuators sliders move RF surfaces through the JSON map.
5. Timing hardening (§7) + reconnect state machine.
6. Options (Rev4/HeliDemix), RC passthrough, remaining model JSONs, `sitl_target.cmake` polish.

## 11. Validation Checklist

1. **Static:** level on runway → accel (0,0,−9.81) with zero jitter (proves ground override), gyro 0, heading = RF compass; nose-up 90° clean (quaternion path, no Euler singularity).
2. **Rates:** roll right p>0, pull up q>0, yaw right r>0 (only yaw negated from RF).
3. **Taxi** north/east → local N/E and vN/vE track; QGC map mirrors RF world.
4. **High alpha:** hover a 3D model → pitot ≈ 0 while `m_airspeed` isn't (proves §6 pitot).
5. **EKF:** innovations bounded over a manual circuit; groundtruth-vs-estimate in ulog.
6. **Resilience:** spacebar in RF (auto re-inject + re-offset), aircraft change in RF (auto restart), 1 s network pull (glitch counter++, no EKF time-jump fault), kill bridge mid-flight (PX4 sensor-timeout failsafe, not a hang).

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
| InterLink RC ch2 | reversed (ArduPilot: `RC2_REVERSED=1`) |
| ICE models battery = −1 | clamp/synthetic |
| WiFi to RealFlight | wired / same host only |
| Bridge lives on wrong side | run bridge on (or wired-adjacent to) the RealFlight machine; SOAP RTT dominates — MAVLink to PX4 tolerates more latency |

## 13. References

- ArduPilot `SIM_FlightAxis.{h,cpp}` — ground truth for §6–§8 conversions, timing, socket & reconnect logic
- PX4-FlightGear-Bridge (`Tools/simulation/flightgear/`) — the integration template: `sitl_run.sh`, `CMakeLists.txt`, `px4_communicator.cpp`, `geo_mag_declination.cpp`, JSON model system, `sitl_target.cmake` registration, README workflow
- `realflight-bridge` Rust crate — SOAP perf notes (WiFi can't hold 200 Hz)
- xuhao1/RealFlightBridge protocol doc; F16Capstone `flightaxis.py` (historical PoC)
- PX4 docs: Simulation → Simulator MAVLink API (TCP 4560 message set)
- Example ExchangeData dump: uav.tridgell.net/RealFlight/data-exchange.txt
