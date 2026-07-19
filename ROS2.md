# ROS 2 with the FlightAxis SITL bridge

How RealFlight data reaches ROS 2, what you get, and what to watch out for.

**Status:** the ROS 2 path was verified end-to-end on 2026-07-19 against a **mock FlightAxis
server**, not against real RealFlight (no Windows machine has ever been in the loop — see
[Validation status](README.md#validation-status)). Everything marked *verified* below was
observed on a live run; everything else is called out explicitly.

---

## 1. Architecture

The bridge speaks **MAVLink HIL to PX4 over TCP 4560**. It does not know about uORB and it
does not know about ROS 2, and it should not. ROS 2 support is entirely PX4's own
uXRCE-DDS stack; the bridge's only job is to make PX4 believe it has sensors.

```
RealFlight  --SOAP-->  flightaxis_bridge  --MAVLink HIL-->  PX4  --uORB-->  uxrce_dds_client
                                                                                   |
                                                                                 DDS/UDP :8888
                                                                                   |
                                                                            MicroXRCEAgent
                                                                                   |
                                                                                 ROS 2
```

Consequences worth internalising:

- **Nothing in this repo affects the ROS 2 layer.** The airframes, the runner and the bridge
  contain no `UXRCE_DDS_*` parameter, no DDS configuration and no ROS 2 code. `uxrce_dds_client`
  is started unconditionally by stock PX4 (`ROMFS/px4fmu_common/init.d-posix/rcS:317`),
  independently of which airframe is selected. **Verified** — it came up and connected under
  `flightaxis_quad` with no extra steps.
- **The topic set you get is stock PX4 v1.16.** It is defined by
  `src/modules/uxrce_dds_client/dds_topics.yaml`, not by anything here.
- **What the bridge does control is *rates and freshness*** — which uORB topics are populated
  at all, and how often. That is where this document has something FlightAxis-specific to say.

---

## 2. Prerequisites

| Need | Check |
|---|---|
| ROS 2 (Humble or later) | `echo $ROS_DISTRO` |
| `MicroXRCEAgent` | `which MicroXRCEAgent` |
| `px4_msgs` **built from PX4 v1.16** | see the version warning below |

> ### ⚠ `px4_msgs` must match PX4 v1.16
>
> This is the one thing most likely to bite you, and it did bite during verification.
>
> A `px4_msgs` built from PX4 `main` will *appear* to work — most topics echo fine — but any
> message whose definition changed silently fails at the DDS layer. On the verification machine,
> `battery_status` produced a continuous stream of:
>
> ```
> [RTPS_READER_HISTORY Error] Change payload size of '184' bytes is larger than
> the history payload size of '183' bytes and cannot be resized.
> ```
>
> and `ros2 topic hz /fmu/out/battery_status` reported **nothing at all**, while the same topic
> was publishing happily at 100 Hz inside PX4 (`uorb top` showed it). The installed `px4_msgs`
> had `BatteryStatus` at `MESSAGE_VERSION = 1` / `MAX_INSTANCES = 3` with `serial_number`
> removed; PX4 v1.16 has `MESSAGE_VERSION = 0` / `MAX_INSTANCES = 4` with `serial_number`
> present.
>
> This is **not** a bug in this integration — it is a PX4/`px4_msgs` version skew on the host.
> Build `px4_msgs` from the matching tag:
>
> ```bash
> cd ~/ros2_ws/src
> git clone https://github.com/PX4/px4_msgs.git -b release/1.16
> cd .. && colcon build --packages-select px4_msgs
> ```
>
> A topic that is silent on the ROS 2 side but alive in `uorb top` is almost always this.

---

## 3. Running it

Three processes. The bridge and PX4 come up together from the usual make target; the agent is
separate and you start it yourself.

**Terminal 1 — the agent** (start it first; the client retries, so order is not critical):

```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — RealFlight + PX4:**

```bash
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_quad
```

**Terminal 3 — ROS 2:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

Confirm the client is actually connected, from the PX4 console:

```
pxh> uxrce_dds_client status
INFO  [uxrce_dds_client] Running, connected
INFO  [uxrce_dds_client] Using transport:     udp
INFO  [uxrce_dds_client] Agent IP:            127.0.0.1
INFO  [uxrce_dds_client] Agent port:          8888
INFO  [uxrce_dds_client] Payload tx:          75564 B/s
INFO  [uxrce_dds_client] timesync converged: true
```

`Running, disconnected` with `timesync converged: false` means the agent is not reachable —
that is the expected output when no `MicroXRCEAgent` is running, and it is harmless to PX4
itself. **Both states were observed during verification.**

### Namespaces

Instance 0 (the default, what the make target gives you) publishes at `/fmu/out/...` with no
prefix. A non-zero instance prefixes everything with `/px4_<N>` (`rcS:293-297`):

```
px4 -i 1   ->   /px4_1/fmu/out/vehicle_attitude
```

Verification was run on **instance 1** deliberately, to stay off the default ports while other
work was using them — so the recorded topic names below carry the `/px4_1` prefix. On a normal
single-vehicle run they will not.

Relevant environment overrides: `PX4_UXRCE_DDS_PORT` (default 8888), `PX4_UXRCE_DDS_NS`, and
`ROS_DOMAIN_ID` — which PX4 picks up automatically and copies into `UXRCE_DDS_DOM_ID`
(`rcS:303-309`).

---

## 4. uORB coverage: what the bridge actually populates

`SimulatorMavlink` translates our MAVLink into uORB. Per message
(`src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp`):

| We send | Handler | uORB topics | Gating |
|---|---|---|---|
| `HIL_SENSOR` | `:479` | `sensor_accel`, `sensor_gyro`, `sensor_mag`, `sensor_baro`, `differential_pressure` | per-field, via the `fields_updated` bitmask (`:192,199,242,285,304,329`) — and the bridge sub-rates that bitmask, see below |
| `HIL_GPS` | `:408` | `sensor_gps` (`PublicationMulti`, `:456-472`) | — |
| `HIL_STATE_QUATERNION` | `:527` | `vehicle_attitude_groundtruth` `:556`, `vehicle_local_position_groundtruth` `:614`, `vehicle_global_position_groundtruth` `:569`, `vehicle_angular_velocity_groundtruth` `:544` | — |
| `DISTANCE_SENSOR` | `:401` | `distance_sensor` | — |
| `RAW_RPM` | `:390` | `rpm` (`:396`) | — |

Everything downstream — `sensor_combined`, `vehicle_attitude`, `vehicle_local_position`,
`vehicle_global_position`, `vehicle_odometry`, `airspeed`, `vehicle_air_data` — is produced by
EKF2 and the sensor hub from those primitives, exactly as on a real vehicle.

### Verified live (`uorb top`, `flightaxis_quad`, mock, vehicle stationary)

```
sensor_accel            1380 Hz     sensor_gps                            10 Hz
sensor_gyro             1380 Hz     distance_sensor                       20 Hz
rpm                     1380 Hz     vehicle_attitude_groundtruth          50 Hz
sensor_combined          204 Hz     vehicle_local_position_groundtruth    50 Hz
vehicle_attitude         204 Hz     vehicle_global_position_groundtruth   50 Hz
battery_status           100 Hz     vehicle_angular_velocity_groundtruth  50 Hz
vehicle_local_position   100 Hz
```

10 / 20 / 50 Hz match the bridge's decimation constants exactly (`GPS_INTERVAL_US = 100000`,
`DISTANCE_INTERVAL_US = 50000`, `STATE_QUAT_INTERVAL_US = 20000`, all in
`px4_communicator.h`).

> **These figures predate the `fields_updated` sub-rating and no longer describe mag, baro or
> differential pressure.** That run was taken when every `HIL_SENSOR` carried `0x1FFF` ("every
> sensor is new"), so `sensor_mag`, `sensor_baro` and `differential_pressure` were republished
> at the full `HIL_SENSOR` rate — 1380 Hz alongside the IMU. The bridge now masks the slow
> fields down to their own intervals in SITL as well as HITL: **magnetometer 100 Hz, barometer
> 50 Hz, differential pressure 50 Hz.** Accel and gyro are never masked and still follow the
> frame rate. Expect those three topics at their sub-rates, not at the IMU rate; the surplus
> was being dropped by `VehicleAirData` anyway, after inflating every ulog for nothing.

Values were sanity-checked and are physically correct for a stationary aircraft: accel
`z = -9.804 m/s²`, gyro ≈ 0, baro 95599 Pa at 488 m, mag consistent with the Zurich default
home position, `distance_sensor 0.1 m`, GPS `fix_type 3` with 10 satellites at 47.3977 / 8.5456.

> **The 1380 Hz figure is a mock artifact, not what RealFlight will give you.** The mock does
> not rate-limit its SOAP exchanges to wall clock, so the bridge free-ran at ~11 000 exchanges/s
> and `HIL_SENSOR` — which is sent every frame in the SITL profile (`sensor_interval_us = 0` in
> `px4_communicator.cpp`) — followed it up. Against real
> RealFlight, expect IMU rates at roughly the SOAP frame rate (~250–300 Hz). The
> *time-decimated* topics (GPS/ground truth/distance) are unaffected and were exact, which is
> itself evidence the bridge decimates on its physics clock rather than on frame count.

### What a normal PX4 has that this does not

- **Ground truth is not exposed to ROS 2.** The four `*_groundtruth` topics are populated in
  uORB at 50 Hz, but `dds_topics.yaml` contains **zero** `groundtruth` entries — verified by
  `grep -c groundtruth` returning `0`, and by their absence from `ros2 topic list`. If you want
  ground truth in ROS 2 for scoring or evaluation, you must add those entries to
  `dds_topics.yaml` and rebuild PX4. This is stock PX4 behaviour and identical under Gazebo.
- **`vehicle_angular_velocity` is commented out** in `dds_topics.yaml` (line 44-45). It is
  available in uORB but not over DDS. Relevant if you intended to close a rate loop offboard.
- **Optical flow** (`HIL_OPTICAL_FLOW` → `sensor_optical_flow`) is handled by
  `SimulatorMavlink` (`:849`) but the bridge never sends it, so `vehicle_optical_flow` stays
  empty. RealFlight exposes no optical-flow quantity, so this is a genuine out-of-scope gap
  rather than an omission — a flow-based offboard app will not work here.
- **`battery_status` and `system_power` are simulated by PX4**, not by us — `battery_simulator`
  and `system_power_simulator` are separate SITL modules and both were live at 100 Hz. Battery
  values are therefore fictional and will not track RealFlight's own battery model.

No topic that a ROS 2 offboard application would reasonably subscribe to was found empty,
with the two exceptions above (optical flow, ground truth) and the `px4_msgs` version issue.

---

## 5. Rates and timestamps as seen from ROS 2

### Measured (`ros2 topic hz`, live run, healthy stack)

| Topic | Rate |
|---|---|
| `vehicle_attitude` | 100.0 Hz |
| `sensor_combined` | 99.8 Hz |
| `vehicle_local_position` | 100.0 Hz |
| `vehicle_global_position` | 100.0 Hz |
| `vehicle_odometry` | 99.8 Hz |
| `vehicle_gps_position` | 10.0 Hz |
| `vehicle_status_v1` | 2.0 Hz |
| `failsafe_flags` | 2.0 Hz |
| `vehicle_land_detected` | 1.0 Hz |
| `timesync_status` | 1.0 Hz |

### The 100 Hz ceiling

Nothing reaches ROS 2 faster than ~100 Hz, regardless of its uORB rate — `vehicle_attitude`
runs at 204 Hz internally and arrives at 100 Hz. This is not decimation by us and not a
per-topic setting (`dds_topics.yaml` has no rate fields). The client polls its subscriptions
with a **10 ms timeout** (`uxrce_dds_client.cpp:657`, `orb_poll_timeout_ms = 10`, used at
`:668`), which caps the publish loop at 100 Hz.

**If you need faster than 100 Hz in ROS 2, uXRCE-DDS will not give it to you** without patching
that constant. For most offboard work this is irrelevant — PX4 only requires a 2 Hz setpoint
heartbeat — but it matters if you intended to run a high-rate attitude or rate loop off-board.

### Timestamps

`hrt_absolute_time()` in a non-lockstep build is the **host monotonic clock** — verified: PX4's
`hrt` read ~80 700 s while the host's `CLOCK_MONOTONIC` read 80 887 s (machine uptime), not
seconds-since-PX4-boot.

Two things follow, and the second is a trap:

1. **HIL sensor data is timestamped on arrival, not by the simulator.**
   `handle_message_hil_sensor` takes `hrt_abstime now_us = hrt_absolute_time()` at
   `SimulatorMavlink.cpp:497` and passes *that* into `update_sensors(now_us, imu)`. The
   `imu.time_usec` field we send is used only for `px4_clock_settime` (`:494`), which drives the
   simulated clock **in lockstep builds only** — and FlightAxis is deliberately non-lockstep, so
   it has no effect. Sensor age is therefore true wall-clock latency, which is what you want for
   a free-running simulator. Measured message age at the PX4 layer was consistently **under
   10 ms**, typically 1–4 ms.

2. **Only `timestamp` and `timestamp_sample` are converted to ROS time.** The generated CDR
   serializers add the timesync offset to exactly those two fields, by name
   (`Tools/msg/templates/ucdr/msg.h.em:138-144`); every other `uint64` timestamp field is copied
   raw. Observed on one `vehicle_local_position` message:

   ```
   timestamp:        1784418874517267   <- epoch microseconds (converted)
   timestamp_sample: 1784418874516395   <- epoch microseconds (converted)
   ref_timestamp:      80600451066      <- PX4 boot-relative microseconds (NOT converted)
   ```

   Same for `nav_state_timestamp` in `vehicle_status`. **Do not compare a `ref_timestamp` with a
   `timestamp`** — they are in different epochs and differ by ~1.78e15 µs. Use `timestamp` /
   `timestamp_sample` for anything time-critical.

Timesync itself was healthy: `timesync_status` reported a **108 µs round-trip** and a stable
estimated offset, and the client reported `timesync converged: true`.

---

## 6. Offboard control

**Verified working end-to-end**, commanded entirely from ROS 2, against the mock.

PX4's requirements are the stock ones. The FlightAxis airframes change none of the offboard
parameters proper: `COM_OBL_*` and the offboard-loss failsafe are untouched, so PX4 defaults
apply. What they do set, beyond control-allocation geometry, `PWM_MAIN_FUNC*` and per-vehicle
tuning (see `1201_flightaxis_quad`), is worth knowing about:

| Parameter | Value | Why it matters here |
|---|---|---|
| `EKF2_GPS_DELAY` | `0` | RealFlight free-runs and the synthesised GPS samples are current. Forced with `param set`, not `set-default` — `px4-rc.mavlinksim` runs after the airframe and would re-default it to 10 ms. |
| `EKF2_MULTI_IMU` | `1` | The bridge only ever supplies IMU instance 0, so the two extra EKF instances `px4-rc.mavlinksim` asks for would run on dead sensors. Also forced with `param set`. |
| `COM_RC_IN_MODE` | `4` | "Ignore any stick input" — the only value that skips the RC-loss failsafe. A headless ROS 2 run has no manual control source at all, so **this is what stops RC-loss failsafe from pre-empting your offboard session.** |
| `SDLOG_MODE` | `4` | "From first arm until shutdown", so a log spans several arm/disarm cycles instead of closing at the first disarm. |

`COM_RC_IN_MODE 4` is the one with a behavioural consequence for offboard work: if you port
these airframes or set the parameters by hand and leave it at the default of 1, an offboard
session with no joystick and no RC will eventually trip the RC-loss failsafe.

What you must do:

1. Publish `OffboardControlMode` at **> 2 Hz continuously**, starting *before* you request
   offboard mode and never stopping. PX4 drops out of offboard if the stream lapses.
2. Publish a matching setpoint (`TrajectorySetpoint` for position control).
3. Request the mode with `VehicleCommand.VEHICLE_CMD_DO_SET_MODE`, `param1 = 1`, `param2 = 6`.
4. Arm with `VEHICLE_CMD_COMPONENT_ARM_DISARM`, `param1 = 1`.
5. Set `target_system` to **`MAV_SYS_ID` = instance + 1** (so `1` for a default run, `2` for
   `-i 1`) — `rcS:140`.
6. Subscribe with **`BEST_EFFORT` + `TRANSIENT_LOCAL`** QoS. PX4 publishes best-effort; a
   default (reliable) ROS 2 subscription silently receives nothing. This is the second most
   common way to get an inexplicably empty topic.

Observed result (`offboard_test.py`, 20 Hz heartbeat, setpoint `[0, 0, -5]` NED):

```
nav_state=14 arming=2 z=-0.009 (status msgs=71, pos msgs=3500)
FINAL nav_state=14 arming_state=2
```

`nav_state 14` is `NAVIGATION_STATE_OFFBOARD`; `arming_state 2` is armed. So PX4 accepted the
mode change and the arm command from ROS 2, and streamed state back.

The control path was confirmed to reach the simulator, not merely to be accepted:

```
disarmed:                actuator_motors.control = [0.0006, 0.0015, 0.0012, 0.0008]
armed, offboard:         actuator_motors.control = [0.913,  0.914,  0.998,  1.000]
received by the mock:                              [0.914,  0.914,  0.999,  1.000]
```

That closes the loop in both directions — ROS 2 → uORB → controller → `HIL_ACTUATOR_CONTROLS`
→ bridge → SOAP → simulator, and simulator → `HIL_*` → uORB → DDS → ROS 2 — and incidentally
confirms the `quad.json` channel map (`rf0..rf3 ← px4[0..3]`, unmapped channels at the 0.5
default).

> **The aircraft did not climb**, and this is expected: `z` stayed at ~0 throughout. The mock
> FlightAxis server is a protocol stub with **no flight dynamics** — it returns a fixed aircraft
> state whatever actuator values it is given. The motors saturating near 1.0 is the position
> controller correctly winding up against a vehicle that refuses to move. **Whether the aircraft
> actually flies to an offboard setpoint can only be established against real RealFlight and has
> not been tested.**

---

## 7. Known limitations

**Of the non-lockstep design** (inherent, not defects):

- **No lockstep.** RealFlight free-runs; PX4 free-runs. There is no barrier between them.
  If your machine stalls, or the network to the Windows box hiccups, PX4 sees a genuine sensor
  gap and EKF2 reacts as it would to a real dropout. This is the correct model for a
  hardware-like simulator, but it means runs are **not bit-reproducible** — the same offboard
  script will not produce an identical trajectory twice.
- **Wall-clock timing.** You cannot pause, single-step or run faster than real time.
  Debugger breakpoints in an offboard node let real time run on and the aircraft will
  drift or crash while you are stopped.
- **Offboard timing is real-time-critical.** Because there is no lockstep to hide it, a ROS 2
  node that misses its heartbeat deadline genuinely loses offboard mode.

**Of the ROS 2 layer:**

- ~100 Hz ceiling on every topic (§5).
- Ground truth and `vehicle_angular_velocity` not exposed without editing `dds_topics.yaml` (§4).
- `px4_msgs` must match v1.16 or topics fail silently (§2).

---

## 8. What has *not* been verified

Stated plainly, because the distinction matters:

- **Nothing has been run against real RealFlight.** All of the above used a mock SOAP server.
  The MAVLink→uORB→DDS→ROS 2 half of the pipeline is fully exercised and is independent of the
  data source, but the SOAP half is not.
- **No actual flight.** The mock has no dynamics, so offboard was proven to be *accepted and
  wired through to the actuators*, not to *fly the aircraft to a setpoint*.
- **Only `flightaxis_quad` was tested over ROS 2.** `plane`, `quadplane` and `heli` were not.
  Nothing in the DDS path is airframe-specific, so they are expected to behave identically,
  but that is inference, not observation.
- **Single vehicle only.** Multi-vehicle namespacing was reasoned from `rcS` and exercised
  incidentally at instance 1; it was not tested as a multi-vehicle scenario.
- **`battery_status` over ROS 2 was never seen working**, because the only `px4_msgs` on the
  verification machine was version-skewed. The failure is understood and attributed, but the
  working case was not demonstrated.

## Reproducing the verification

The mock server, stack scripts and the offboard test node used above live in the scratchpad
(`mock_flightaxis.py`, `up.sh` / `down.sh`, `offboard_test.py`). Any FlightAxis-shaped SOAP
responder will do; the point is that the ROS 2 half of the pipeline can be exercised with no
Windows machine present.
