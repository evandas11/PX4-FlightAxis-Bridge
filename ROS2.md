# ROS 2 with the FlightAxis SITL bridge

How RealFlight data reaches ROS 2, what you get, and what to watch out for.

---

## 1. Architecture

The bridge speaks **MAVLink HIL to PX4 over TCP 4560**. It doesn't know about uORB and it
doesn't know about ROS 2, and it shouldn't. ROS 2 support is entirely PX4's own
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
  independently of which airframe is selected. It comes up and connects under the FlightAxis
  airframes with no extra steps.
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

> ### `px4_msgs` must match PX4 v1.16
>
> This is the one thing most likely to bite you.
>
> A `px4_msgs` built from PX4 `main` will *appear* to work — most topics echo fine — but any
> message whose definition changed silently fails at the DDS layer. A skewed `battery_status`,
> for instance, produces a continuous stream of:
>
> ```
> [RTPS_READER_HISTORY Error] Change payload size of '184' bytes is larger than
> the history payload size of '183' bytes and cannot be resized.
> ```
>
> while `ros2 topic hz /fmu/out/battery_status` reports **nothing at all**, even though the same
> topic is publishing happily at 100 Hz inside PX4 (`uorb top` shows it). In that case
> `px4_msgs` had `BatteryStatus` at `MESSAGE_VERSION = 1` / `MAX_INSTANCES = 3` with
> `serial_number` removed; PX4 v1.16 has `MESSAGE_VERSION = 0` / `MAX_INSTANCES = 4` with
> `serial_number` present.
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
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 \
PX4_HOME_LON=175.7433944 \
PX4_HOME_ALT=48.0 \
PX4_HOME_YAW=235 \
make px4_sitl_nolockstep flightaxis_quad
```

The home variables are read by the bridge at startup and stored nowhere, so they belong on
every run; substitute your own field. See RUNNING.md §3.

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
itself.

### Namespaces

Instance 0 (the default, what the make target gives you) publishes at `/fmu/out/...` with no
prefix. A non-zero instance prefixes everything with `/px4_<N>` (`rcS:293-297`):

```
px4 -i 1   ->   /px4_1/fmu/out/vehicle_attitude
```

Running on a non-zero instance is a convenient way to stay off the default ports when other
work is using them, at the cost of the `/px4_<N>` prefix on every topic name.

Relevant environment overrides: `PX4_UXRCE_DDS_PORT` (default 8888), `PX4_UXRCE_DDS_NS`, and
`ROS_DOMAIN_ID` — which PX4 picks up automatically and copies into `UXRCE_DDS_DOM_ID`
(`rcS:303-309`); the port itself defaults to 8888 at `rcS:311`, with `PX4_UXRCE_DDS_PORT`
overriding it (`rcS:311-315`).

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

### uORB rates (`uorb top`, `flightaxis_quad`, vehicle stationary)

```
sensor_accel            1380 Hz     sensor_gps                            10 Hz
sensor_gyro             1380 Hz     distance_sensor                       20 Hz
rpm                     1380 Hz     vehicle_attitude_groundtruth          50 Hz
sensor_combined          204 Hz     vehicle_local_position_groundtruth    50 Hz
vehicle_attitude         204 Hz     vehicle_global_position_groundtruth   50 Hz
battery_status             2 Hz     vehicle_angular_velocity_groundtruth  50 Hz
vehicle_local_position   100 Hz
```

10 / 20 / 50 Hz match the bridge's decimation constants exactly (`GPS_INTERVAL_US = 100000`,
`DISTANCE_INTERVAL_US = 50000`, `STATE_QUAT_INTERVAL_US = 20000`, all in
`px4_communicator.h`).

> **These figures predate the `fields_updated` sub-rating and no longer describe mag, baro or
> differential pressure.** They were taken when every `HIL_SENSOR` carried `0x1FFF` ("every
> sensor is new"), so `sensor_mag`, `sensor_baro` and `differential_pressure` were republished
> at the full `HIL_SENSOR` rate — 1380 Hz alongside the IMU. The bridge now masks the slow
> fields down to their own intervals in SITL as well as HITL: **magnetometer 100 Hz, barometer
> 50 Hz, differential pressure 50 Hz.** Accel and gyro are never masked and still follow the
> frame rate. Expect those three topics at their sub-rates, not at the IMU rate; the surplus
> was being dropped by `VehicleAirData` anyway, after inflating every ulog for nothing.
>
> `battery_status` is corrected too. The 100 Hz in the original capture was PX4's own
> `battery_simulator`; the airframes now set `SIM_BAT_ENABLE 0` and the topic comes from the
> bridge's separate UDP link at its `SEND_INTERVAL_US = 500000` — **2 Hz** (see below).

The accompanying values are what a stationary aircraft should show: accel `z = -9.804 m/s²`,
gyro ≈ 0, baro 95599 Pa at 488 m, mag consistent with the Zurich default home position,
`distance_sensor 0.1 m`, GPS `fix_type 3` with 10 satellites at 47.3977 / 8.5456.

> **Do not read 1380 Hz as the IMU rate you will get.** `HIL_SENSOR` is sent every frame in the
> SITL profile (`sensor_interval_us = 0` in `px4_communicator.cpp`), so the IMU topics simply
> follow the SOAP exchange rate — that figure came from a source that did not rate-limit its
> exchanges to wall clock, letting the bridge free-run at ~11 000 exchanges/s. Against real
> RealFlight, expect IMU rates at roughly the SOAP frame rate (**~250–300 Hz**). The
> *time-decimated* topics (GPS, ground truth, distance) are unaffected by the exchange rate,
> because the bridge decimates on its physics clock rather than on frame count.

### What a normal PX4 has that this does not

- **Ground truth is not exposed to ROS 2.** The four `*_groundtruth` topics are populated in
  uORB at 50 Hz, but `dds_topics.yaml` contains **zero** `groundtruth` entries — `grep -c
  groundtruth` returns `0`, and they are absent from `ros2 topic list`. If you want
  ground truth in ROS 2 for scoring or evaluation, you must add those entries to
  `dds_topics.yaml` and rebuild PX4. This is stock PX4 behaviour, not something this
  integration changes.
- **`vehicle_angular_velocity` is commented out** in `dds_topics.yaml` (line 44-45). It is
  available in uORB but not over DDS. Relevant if you intended to close a rate loop offboard.
- **Optical flow** (`HIL_OPTICAL_FLOW` → `sensor_optical_flow`) is handled by
  `SimulatorMavlink::handle_message_optical_flow` (`:849`, dispatched at `:358-359`) but the
  bridge never sends it, so `vehicle_optical_flow` stays
  empty. RealFlight exposes no optical-flow quantity, so this is a genuine out-of-scope gap
  rather than an omission — a flow-based offboard app will not work here.
- **`system_power` is simulated by PX4**, not by us. `system_power_simulator` is a SITL module
  started unconditionally (`rcS:254`) and publishes at 100 Hz, so those values are fictional.
  **`battery_status`, by contrast, does come from the bridge.** PX4's `battery_simulator` is
  gated on `SIM_BAT_ENABLE 1` (`rcS:249-251`) and every FlightAxis airframe sets
  `SIM_BAT_ENABLE 0`, so it never starts; the battery state you see over DDS is the bridge's,
  delivered over MAVLink and derived from the pack voltage and current RealFlight reports.

Beyond the two exceptions above (optical flow, ground truth) and the `px4_msgs` version issue,
the topics a ROS 2 offboard application would reasonably subscribe to are populated.

---

## 5. Rates and timestamps as seen from ROS 2

### Rates (`ros2 topic hz`, healthy stack)

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
runs at 204 Hz internally and arrives at 100 Hz. That isn't decimation by us, and it isn't a
per-topic setting (`dds_topics.yaml` has no rate fields). It comes out of the client's main
loop, which waits on its uORB subscriptions with a **10 ms timeout**
(`uxrce_dds_client.cpp:657`, `orb_poll_timeout_ms = 10`, used at `:668`). That 10 ms is a
maximum *idle* wait rather than a hard rate limit — when there is already inbound traffic
(`bytes_available > 10`) the timeout is set to 0 at `:661-665` and the loop spins immediately —
but with a quiet agent link the wait dominates and each iteration costs about 10 ms, so the
outbound publish rate settles near 100 Hz.

**In practice uXRCE-DDS will not hand you much more than 100 Hz**, whatever the uORB rate. For
most offboard work this is irrelevant — the setpoint heartbeat PX4 requires is far slower — but
it matters if you intended to run a high-rate attitude or rate loop off-board.

### Timestamps

`hrt_absolute_time()` in a non-lockstep build is the **host monotonic clock**: PX4's `hrt` reads
machine uptime — ~80 700 s against the host's `CLOCK_MONOTONIC` at 80 887 s — not
seconds-since-PX4-boot.

Two things follow, and the second is a trap:

1. **HIL sensor data is timestamped on arrival, not by the simulator.**
   `handle_message_hil_sensor` takes `hrt_abstime now_us = hrt_absolute_time()` at
   `SimulatorMavlink.cpp:497` and passes *that* into `update_sensors(now_us, imu)`. The
   `imu.time_usec` field we send is used only for `px4_clock_settime` (`:494`), which drives the
   simulated clock **in lockstep builds only** — and FlightAxis is deliberately non-lockstep, so
   it has no effect. Sensor age is therefore true wall-clock latency, which is what you want for
   a free-running simulator. Message age at the PX4 layer sits **under 10 ms**, typically
   1–4 ms.

2. **Only `timestamp` and `timestamp_sample` are converted to ROS time.** The generated CDR
   serializers add the timesync offset to exactly those two fields, by name
   (`Tools/msg/templates/ucdr/msg.h.em:138-144`); every other `uint64` timestamp field is copied
   raw. On a `vehicle_local_position` message:

   ```
   timestamp:        1784418874517267   <- epoch microseconds (converted)
   timestamp_sample: 1784418874516395   <- epoch microseconds (converted)
   ref_timestamp:      80600451066      <- PX4 boot-relative microseconds (NOT converted)
   ```

   Same for `nav_state_timestamp` in `vehicle_status`. **Do not compare a `ref_timestamp` with a
   `timestamp`** — they are in different epochs and differ by ~1.78e15 µs. Use `timestamp` /
   `timestamp_sample` for anything time-critical.

A healthy timesync looks like this: `timesync_status` reporting a round-trip in the low hundreds
of microseconds (**108 µs** on loopback) and a stable estimated offset, with the client
reporting `timesync converged: true`.

---

## 6. Offboard control

Offboard control is commanded entirely from ROS 2; nothing in this integration sits in that path.

PX4's requirements are the stock ones. The FlightAxis airframes change none of the offboard
parameters proper: `COM_OBL_*` and the offboard-loss failsafe are untouched, so PX4 defaults
apply. What they do set, beyond control-allocation geometry, `PWM_MAIN_FUNC*` and per-vehicle
tuning (see `1201_flightaxis_quad`), is worth knowing about:

| Parameter | Value | Why it matters here |
|---|---|---|
| `EKF2_GPS_DELAY` | `0` | RealFlight free-runs and the synthesised GPS samples are current. Forced with `param set`, not `set-default` — `px4-rc.mavlinksim` runs after the airframe and would re-default it to 10 ms. |
| `EKF2_MULTI_IMU` | `1` | The bridge only ever supplies IMU instance 0, so the two extra EKF instances `px4-rc.mavlinksim` asks for would run on dead sensors. Also forced with `param set`. |
| `COM_RC_IN_MODE` | `4` | "Ignore any stick input" — the only value that skips the RC-loss failsafe. A headless ROS 2 run has no manual control source at all, so **this is what stops RC-loss failsafe from pre-empting your offboard session.** |
| `SDLOG_MODE` | `0` | "When armed until disarm" — one bounded log file per flight. `rcS` sets 1, "from boot until disarm", which opens the log at boot and records the whole idle stretch before you ever arm, so the override back to PX4's own default is not redundant. |
| `SENS_IMU_AUTOCAL`, `SENS_MAG_AUTOCAL`, `IMU_GYRO_CAL_EN` | `0` | PX4 learns accel, gyro and mag offsets in flight and commits them as permanent calibration. Every sensor here is synthesised from RealFlight's state and has no bias to find, so what gets learned is the estimator's own transients — and then saved, so the next run starts from them. **This is the one that will waste your afternoon:** it survives restarts, so a position that drifts in `vehicle_local_position` while the aircraft sits still in RealFlight looks unrelated to whatever you changed last. Set as `set-default`, so a working directory flown before this landed keeps its learned offsets until you clear them (RUNNING.md §7). |

`COM_RC_IN_MODE 4` is the one with a behavioural consequence for offboard work: if you port
these airframes or set the parameters by hand and leave it at the default of 1, an offboard
session with no joystick and no RC will eventually trip the RC-loss failsafe.

What you must do:

1. Publish `OffboardControlMode` **continuously**, starting *before* you request offboard mode
   and never stopping. PX4 drops out of offboard if the stream lapses for longer than
   `COM_OF_LOSS_T` (`commander_params.c:322`), which defaults to **1.0 s** and is checked in
   `offboardCheck.cpp:46-47` — so the code requirement is faster than 1 Hz. **Use 2 Hz or
   better**; the margin costs nothing and absorbs scheduling jitter.
2. Publish a matching setpoint (`TrajectorySetpoint` for position control).
3. Request the mode with `VehicleCommand.VEHICLE_CMD_DO_SET_MODE`, `param1 = 1`, `param2 = 6`.
4. Arm with `VEHICLE_CMD_COMPONENT_ARM_DISARM`, `param1 = 1`.
5. Set `target_system` to **`MAV_SYS_ID` = instance + 1** (so `1` for a default run, `2` for
   `-i 1`) — `rcS:140`.
6. Subscribe with **`BEST_EFFORT` + `TRANSIENT_LOCAL`** QoS. PX4 publishes best-effort; a
   default (reliable) ROS 2 subscription silently receives nothing. This is the second most
   common way to get an inexplicably empty topic.

To confirm PX4 accepted the mode change and the arm command, watch `vehicle_status`: `nav_state
14` is `NAVIGATION_STATE_OFFBOARD` and `arming_state 2` is armed.

Acceptance is not the same as reaching the simulator, so check the far end of the chain too.
Compare `actuator_motors.control` inside PX4 against the channel values RealFlight receives —
they should track each other, with `quad.json` mapping `rf0..rf3 ← px4[0..3]` and unmapped
channels sitting at the 0.5 default. That exercises the loop in both directions: ROS 2 → uORB →
controller → `HIL_ACTUATOR_CONTROLS` → bridge → SOAP → simulator, and simulator → `HIL_*` →
uORB → DDS → ROS 2.

> **If the vehicle does not move, look at the far end before the near end.** A position
> controller whose motor outputs saturate near 1.0 while `z` stays at ~0 is winding up correctly
> against a vehicle that is not responding — the setpoint path is fine and the dynamics are not
> reaching it. Check that RealFlight is unpaused, that its physics speed multiplier is 1.0, and
> that the model is the one the airframe expects.

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
- **A RealFlight respawn restarts PX4, and so ends the DDS session.** This is the default. When
  the pilot presses spacebar the bridge sees the model teleport, force-disarms, and exits so the
  runner can bring PX4 and the bridge back up. The reason is that a respawn leaves EKF2 holding a
  converged solution for a flight that no longer exists: it reads the position step as a lying
  GPS rather than a moved aircraft, rejects it and dead-reckons, and the aircraft drifts across
  the map while it sits on the runway. A new estimator is the only thing that fixes that.
  **The consequence for ROS 2 is a hard discontinuity:** the uXRCE-DDS client goes down and comes
  back, every topic drops out for the gap, the log file changes, and your node must reconnect,
  re-arm and re-establish its setpoint stream rather than assuming continuity across a reset.
  Design the offboard node to tolerate PX4 disappearing.
- `PX4_FLIGHTAXIS_RESTART_ON_RESET=0` opts out, keeping one continuous session across a respawn.
  That preserves the DDS connection but leaves you with the diverged estimate above — the bridge
  falls back to re-anchoring its position offset and asking PX4 for an external position reset,
  which EKF2 gates on dead-reckoning or being on the ground without GNSS fusion and so may refuse
  at the moment of the teleport and accept some seconds later.

---

## 8. Working without RealFlight

The MAVLink → uORB → DDS → ROS 2 half of the pipeline is independent of where the sensor data
comes from, so it can be driven with no Windows machine present: point the bridge at any
FlightAxis-shaped SOAP responder and the whole ROS 2 side comes up normally. This is convenient
for developing offboard nodes, message plumbing and QoS settings.

A stub responder has no flight dynamics — it returns a fixed aircraft state whatever actuator
values it is given — so a position controller will wind its motor outputs up against a vehicle
that never moves. That is enough to develop against the topic and command interfaces, but
anything involving actual dynamics needs real RealFlight; see [`RUNNING.md`](RUNNING.md) and
attach the agent and ROS 2 terminals from §3.

Nothing in the DDS path is airframe-specific, so `plane`, `quadplane` and `heli` present the
same topic set as `quad`; the difference between them is confined to control allocation and
tuning.
