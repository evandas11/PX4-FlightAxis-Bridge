# Hardware-in-the-loop (HITL) against RealFlight

How to drive a **real flight controller board** from RealFlight, instead of PX4 SITL.
For the SITL workflow see [`RUNNING.md`](RUNNING.md); for the design rationale see
[`FLIGHTAXIS_PX4_INTEGRATION.md`](FLIGHTAXIS_PX4_INTEGRATION.md).

Assumes the bridge is installed into a PX4 v1.16 checkout (`./install.sh`) and that
`~/PX4-Autopilot` is that checkout.

> **This path drives a real flight controller and real ESCs.** Remove every propeller before
> connecting anything. Confirm each actuator output on the bench — direction, ordering,
> endpoints and failsafe behaviour — before the airframe is assembled or flight power is
> applied. §1 is the full safety discussion and §11 is the bring-up checklist; read both before
> connecting a board.
>
> HITL differs from the SITL path in more than the transport: it uses a different PX4 module
> (`mavlink` receiver, not `simulator_mavlink`), a different transport (serial/UDP, not TCP
> loopback), a different output-parameter family (`HIL_ACT_FUNC<N>`, not `PWM_MAIN_FUNC<N>`) and
> a different clock (the board's). Behaviour on one path does not carry over to the other.

---

## 1. Safety — read this first

HITL makes a real flight controller believe it is flying. It runs the real firmware, the real
estimator and the real control allocator.

### REMOVE ALL PROPELLERS. Every time. No exceptions.

PX4 does provide a software interlock, and it is a good one:

- With `SYS_HITL > 0`, `rcS` starts `pwm_out_sim start -m hil` and **never starts `pwm_out` or
  `dshot`** — the real output drivers are not loaded at all
  (`ROMFS/px4fmu_common/init.d/rcS:451-464`).
- `commander start -h` puts the vehicle in `HIL_STATE_ON`, and commander then holds
  `_actuator_armed.lockdown = true` permanently (`Commander.cpp:1892`).

So on a correctly configured board the servo rail should stay silent even when armed.

**That is a software interlock, not a physical one.** It does not protect you from:

- a wrong or stale `SYS_HITL` (it is `@reboot_required` — setting it is not enough, you must
  reboot),
- flashing firmware that lacks `pwm_out_sim`, so the HIL path silently does not engage,
- an ESC or servo powered from a separate BEC that ignores the FMU entirely,
- loading a different airframe afterwards and forgetting.

Props off is the only measure that actually protects your hands. The runner prints this
warning on every start; it is not decoration.

Two further notes:

- **`CBRK_SUPPLY_CHK`.** Stock HIL airframes set `CBRK_SUPPLY_CHK 894281` to allow arming
  without a battery (`init.d/airframes/1001_rc_quad_x.hil`). That circuit breaker disables a
  real safety check. Set it for HITL, and **clear it before you ever fly the airframe for
  real.**
- **Undo the HITL parameters before real flight.** `SYS_HITL=1` on a vehicle you then try to
  fly is a crash. Save a parameter file for each mode and reload deliberately.

---

## 2. SITL vs HITL — what actually differs

| | SITL | HITL |
|---|---|---|
| Where PX4 runs | `px4` binary on your PC | firmware on the board |
| Who receives HIL messages | `simulator_mavlink` module | `mavlink` module's receiver |
| Transport | TCP server, port 4560+instance | USB CDC-ACM / UART / UDP |
| Bridge role | TCP **server** (PX4 connects in) | **client** (bridge opens the link) |
| Lockstep | available (`px4_sitl_default`) | **never** — time comes from the board's own clock |
| Actuator source topic | `actuator_outputs_sim` | `actuator_outputs_sim` — *same on both* |
| Output function params | `PWM_MAIN_FUNC<N>` | `HIL_ACT_FUNC<N>` |
| Real PWM/DShot outputs | n/a | **not started at all** |

The actuator topic is deliberately listed as identical because it is a natural place to expect a
difference and there is none. `SimulatorMavlink.cpp:1005` subscribes to `actuator_outputs_sim`,
and in SITL `pwm_out_sim -m sim` (`init.d-posix/rcS:279`) is what publishes it; `PWMSim.cpp:93`
is the sole publisher on either path. What actually differs is the *parameter prefix* that names
each channel's function — `PWMSim.hpp:47-51` selects `PWM_MAIN` or `HIL_ACT` at compile time —
which is the row immediately below.

**Lockstep is not involved.** Lockstep is compiled in via `ENABLE_LOCKSTEP_SCHEDULER` and only
exists for SITL; a real board runs on its own hardware clock and cannot be stepped. This is why
the bridge is built under `px4_sitl_nolockstep` and why HITL needs no timing negotiation. Both
sides free-run and the sensor timestamps carry the simulation clock.

### Messages the board consumes

`MavlinkReceiver::handle_message()` gates the HIL set behind `get_hil_enabled()`
(`mavlink_receiver.cpp:347-378`):

| Message | Consumed in HITL? | Notes |
|---|---|---|
| `HIL_SENSOR` | yes | → accel/gyro/mag/baro/diff-pressure (`:2302`) |
| `HIL_GPS` | yes | also accepted outside HIL if `use_hil_gps` (`:2405`) |
| `HIL_OPTICAL_FLOW` | yes | `:877` — the bridge does not send it; RealFlight exposes no flow sensor |
| `HIL_STATE_QUATERNION` | yes — **and that is the problem**, see below (`:2602`) |
| `DISTANCE_SENSOR` | yes | handled unconditionally at `:207`, needs no HIL gating |
| `RAW_RPM` | **no handler exists** | sending it is pure wasted bandwidth |

### Messages the board emits

`HIL_ACTUATOR_CONTROLS` at **200 Hz**, configured by `set_hil_enabled()`
(`mavlink_main.cpp:673`). Two things about that function matter enormously:

```c
if (hil_enabled && !_hil_enabled && _datarate > 5000) {      // mavlink_main.cpp:671
        _hil_enabled = true;
        ret = configure_stream("HIL_ACTUATOR_CONTROLS", 200.0f);
        if (_param_sys_hitl.get() == 2) {                    // SIH only
                configure_stream("HIL_STATE_QUATERNION", 25.0f);
```

1. **The `_datarate > 5000` gate.** If the MAVLink instance's datarate is at or below 5000 B/s,
   PX4 silently never enables the HIL streams, with no error message. Which instance you use
   decides whether that bites: `module.yaml:98` gives `MAV_x_RATE` a default of `[1200, 0, 0]`,
   so **only instance 0 defaults to 1200 B/s** — squarely under the gate. Instances 1 and 2
   default to `0`, meaning baud/20, which clears the gate at any baud above 100000 (at 921600
   it is 46080 B/s) and *fails* it at 100000 baud or below. So the classic silent failure is an
   unconfigured `MAV_0_RATE`, or a slow UART on any instance. §6.2 sets it explicitly rather
   than relying on either default; §9 has the symptom.
2. `HIL_STATE_QUATERNION` is only streamed **out** when `SYS_HITL == 2` (SIH), as ground truth.

### Why the bridge must not send `HIL_STATE_QUATERNION` in HITL

This is the biggest behavioural difference from the SITL path, and it isn't a bandwidth
question.

In SITL, `simulator_mavlink` treats `HIL_STATE_QUATERNION` as ground truth for logging. On a
real board it is *consumed*: `handle_message_hil_state_quaternion()`
(`mavlink_receiver.cpp:2602-2732`) publishes straight onto

- `vehicle_attitude`
- `vehicle_local_position`
- `vehicle_global_position`
- `airspeed`

— which are **EKF2's own output topics**. Sending it makes the bridge a second publisher racing
the estimator. The vehicle then flies beautifully and proves nothing, because it is flying on
injected truth rather than on its own state estimate.

The bridge therefore disables it in the HITL profile. If you deliberately want that behaviour
(the "HIL state level" semantic — useful for isolating a control problem from an
estimation problem), set `PX4_HITL_STATE_QUAT_BYPASS=1` and accept what it means. The bridge
prints a loud warning when you do.

---

## 3. Bandwidth — why 250 Hz

On-wire cost is payload + 12 bytes of MAVLink v2 framing (10 byte header + 2 byte CRC).
Payloads are trailing-zero truncated:

| Message | On wire |
|---|---|
| `HIL_SENSOR` | **77 B** |
| `HIL_GPS` | 52 B |
| `DISTANCE_SENSOR` | 55 B |
| `HEARTBEAT` | 25 B |
| `HIL_ACTUATOR_CONTROLS` (inbound) | 93 B |

A serial line at 8N1 carries 10 bits per byte, so **bytes/s = baud / 10**.

**What the SITL profile would cost.** `HIL_SENSOR` and `RAW_RPM` go out on every `Send()`, and
`Send()` runs once per RealFlight frame (~250 Hz) *plus* once per 1 ms extrapolation step —
about 1000 Hz:

```
1000×77 + 1000×17 + 10×52 + 50×77 + 20×55  ≈  98.3 kB/s
```

That is free over TCP loopback and **exceeds even 921600 baud (92.2 kB/s)**. Serial HITL must
decimate.

**HITL profile at 250 Hz** (no `HIL_STATE_QUATERNION`, no `RAW_RPM`, no `RC_CHANNELS` — see §7):

```
250×77 + 5×52 + 20×55 + 1×25  =  19250 + 260 + 1100 + 25  ≈  20.6 kB/s  =  206 kbit/s
```

| Baud | Capacity | Load at 250 Hz | Verdict |
|---|---|---|---|
| 115200 | 11.5 kB/s | 179 % | unusable — even 100 Hz does not fit |
| 230400 | 23.0 kB/s | 90 % | too tight, no headroom |
| 460800 | 46.1 kB/s | 45 % | fine |
| **921600** | **92.2 kB/s** | **22 %** | **recommended**; 500 Hz also fits (43 %) |
| USB CDC-ACM | 100 kB/s | 21 % | best option; baud is nominal and ignored, and the limit is PX4 capping its own datarate at 100000 B/s rather than anything about the wire |

Return direction is independent (serial is full duplex): 200 Hz × 93 B = **18.6 kB/s** of
`HIL_ACTUATOR_CONTROLS`. Note this must clear the `_datarate > 5000` gate from §2.

**Why 250 Hz is the default:** it matches RealFlight's own physics frame rate, so nothing is
lost — the 1 ms extrapolation steps are interpolation between frames, not new information. It
fits comfortably from 460800 up. Decimation had to be added here at all because PX4's other
simulator bridges run over loopback, where bytes are free and no decimation is needed; a
serial link to a board is the case they do not cover.

### Rate quantisation (read before comparing observed rates)

Each rate gate is `if (now - last >= interval) { last = now; ... }` against the **physics**
clock. The achieved period is therefore the requested period **rounded up to however far the
physics clock moves per `Send()` call**.

When the main loop free-runs (several thousand iterations/s against RealFlight) that step is the
1 ms extrapolation quantum, and 250 Hz comes out exact. If the loop is throttled — a slow link,
a reader that isn't draining — the step grows and the rate drops: at a 3 ms step, a 4 ms
interval yields 6 ms, i.e. 166 Hz. Expect the achieved rate to sit at or below the requested
one, and to track loop granularity rather than the gate being misconfigured.

The error is always in the safe direction (never faster than requested, so never over budget),
which is why the simple form is kept instead of accumulating `last += interval` — the latter
would burst to catch up after exactly the kind of stall that caused it.

### `fields_updated`

`vehicle_state.cpp` builds every `HIL_SENSOR` with `fields_updated = 0x1FFF` — "every sensor is
new". Left alone, that makes the receiver republish the magnetometer, barometer and
differential pressure at the full `HIL_SENSOR` rate: on a real board, 250 Hz of baro and mag
into an MCU whose real drivers would run them at 50–100 Hz. It costs no extra bytes (the fields
are in the message either way) but it costs CPU and misrepresents the sensor suite to the
estimator. In SITL it is if anything worse — `SimulatorMavlink::update_sensors()` publishes
`sensor_baro` *twice* for every message carrying the BARO bits — which is why the masking is
applied on both targets rather than only in HITL.

`px4_communicator` therefore masks the bitmask down on the way out. Accel and gyro are never
masked and stay at the full `HIL_SENSOR` rate. The three slow sensors are masked down to:

| Field | SITL | HITL |
|---|---|---|
| magnetometer | 100 Hz | 50 Hz |
| barometer | 50 Hz | 50 Hz |
| differential pressure | 50 Hz | 50 Hz |

Bit values are `SensorSource` in `mavlink_receiver.h:365-371`. Note the masks are
**all-bits-equal** tests, not any-bit: `BARO` needs bits 9, 11 and 12 together. Bit 12 also
latches `_sensors_temperature` (`SimulatorMavlink.cpp:190-195`), which accel, gyro and mag then
read — so `BARO` must never be masked out of the *first* message. It is not: `sent_first`
backdates every `last_*_us` so message one always carries the full `0x1FFF`.

### `HEARTBEAT`

The HITL profile sends a `HEARTBEAT` — 10 Hz until the board answers, then 1 Hz. PX4 will not
bring a USB MAVLink instance up until it hears from the other end. SITL does not need it and
does not send it.

---

## 4. Hardware and wiring

**Option A — USB (recommended).** A single USB cable from the PC to the board's USB port.
Appears as `/dev/ttyACM0`. PX4 forces its datarate to 100000 B/s on USB
(`mavlink_main.cpp:2166-2181`), which clears the `_datarate > 5000` gate automatically and is
why USB is the path of least resistance. Baud is nominal and ignored.

**Option B — TELEM UART.** USB-to-serial adapter (FTDI/CP2102) to `TELEM1`/`TELEM2`, appearing
as `/dev/ttyUSB0`. Wire TX↔RX crossed and share ground:

```
adapter GND  ── GND   TELEMx
adapter TX   ── RX    TELEMx
adapter RX   ── TX    TELEMx
```

Do **not** connect the adapter's 5 V to the board if the board is otherwise powered. The bridge
configures the port with **no flow control**, matching a 3-wire cable; PX4 only enables RTS/CTS
when asked with `-z`/`-Z`.

**Option C — Ethernet/UDP.** Only on boards with Ethernet and `CONFIG_NET` firmware. Note PX4's
`mavlink` module supports **SERIAL and UDP only — there is no TCP transport**, which is why the
bridge offers a UDP client and deliberately no TCP client mode.

Powering the board from USB alone is normal and sufficient for HITL, since the output drivers
never start.

---

## 5. Firmware requirements

The board's firmware must include `pwm_out_sim`, or the whole HIL output path is missing:

```
CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y
```

This is enabled in most default builds (`px4_fmu-v5_default`, `px4_fmu-v6x_default`, …). If you
build a trimmed firmware, check it. Without it there is no `actuator_outputs_sim` publisher and
no `HIL_ACTUATOR_CONTROLS` will ever be emitted — with no error message explaining why.

---

## 6. Board parameter setup

Set these with QGroundControl (Vehicle Setup → Parameters) or `param set` on the MAVLink
console. **`SYS_HITL` is `@reboot_required` — reboot after setting it.**

### 6.1 Core

| Parameter | Value | Why |
|---|---|---|
| `SYS_HITL` | `1` | HITL on. `0` off, `2` = SIH (onboard sim), `-1` = "external HITL" flag only (`lib/systemlib/system_params.c:84`) |
| `CBRK_SUPPLY_CHK` | `894281` | allow arming with no battery — **see §1** |
| `COM_RC_IN_MODE` | `1` or `4` | no real RC transmitter attached |
| `EKF2_MULTI_IMU` | `1` | The bridge sends `HIL_SENSOR` with `id = 0` only, so only IMU instance 0 is ever supplied; leaving this higher runs EKF instances on dead sensors (same as the SITL airframes) |
| `SDLOG_MODE` | `4` | Optional, but it makes a log span several arm/disarm cycles instead of closing at the first disarm. `@reboot_required` |

**Leave `EKF2_GPS_DELAY` at its firmware default.** The SITL airframes force it to `0`, but that
value is SITL-specific and wrong on a board — HITL `HIL_GPS` arrives over a real link with real
latency, and zeroing the delay mis-times EKF fusion. The shipped `.hil` airframes deliberately do
not set it. See §6.3.

### 6.2 MAVLink instance

For **USB** (`/dev/ttyACM0`) the defaults are usually fine. For a **hardware UART**:

| Parameter | Value |
|---|---|
| `MAV_1_CONFIG` | the TELEM port you wired (e.g. `102` = TELEM2) |
| `SER_TEL2_BAUD` | `921600` |
| `MAV_1_MODE` | `0` (Normal) |
| `MAV_1_RATE` | **`> 5000`**, e.g. `80000` — the §2 gate. `0` means baud/20 |
| `MAV_1_FORWARD` | `0` |

There is **no HIL-specific `MAV_x_MODE`**. The mode enum (`mavlink_main.h:202-216`) has no HIL
entry in v1.16 — HIL streams are added dynamically by `set_hil_enabled()` when commander reports
`HIL_STATE_ON`, not by selecting a mode. Do not go looking for one.

### 6.3 Airframe and output mapping

**Four HITL airframes are shipped.** `install.sh` places them in the PX4 tree and registers them:

| `SYS_AUTOSTART` | File | Model JSON |
|---|---|---|
| 1200 | `ROMFS/px4fmu_common/init.d/airframes/1200_flightaxis_plane.hil` | `plane.json` |
| 1201 | `…/1201_flightaxis_quad.hil` | `quad.json` |
| 1202 | `…/1202_flightaxis_quadplane.hil` | `quadplane.json` |
| 1203 | `…/1203_flightaxis_heli.hil` | `heli.json` |

They carry the same ids as the SITL airframes deliberately, so **1201 means "FlightAxis quad"
whether the firmware is SITL or on a board**. They are separate files because a real board boots
from `init.d`, never `init.d-posix`, and because the two startup paths route outputs differently:
SITL uses `PWM_MAIN_FUNC<N>`, a HITL board uses `HIL_ACT_FUNC<N>` (`pwm_out_sim/module_hil.yaml`,
prefix `HIL_ACT`, 16 channels, visible only when `SYS_HITL>0`). Each `.hil` file forces
`SYS_HITL 1` itself.

> ### ⚠ Two things to know before you use these
>
> **1. Confirm the mapping and the gains on your own bench.** These files were derived by
> mechanical substitution from the four SITL airframes, so the channel mapping is exact by
> construction — but a board will happily drive whatever you have actually wired to it. With
> the propellers off, step each output and confirm it moves the surface or motor you expect, in
> the direction you expect, to the endpoints you expect. Treat the controller gains as a
> starting point to be tuned on your airframe, particularly the helicopter's.
>
> **2. Your board must be built with `pwm_out_sim` enabled, or the airframe will not exist.**
> These live inside the `if(CONFIG_MODULES_SIMULATION_PWM_OUT_SIM)` block of
> `init.d/airframes/CMakeLists.txt`, and only three boards in the tree set that symbol by default
> (`holybro/kakuteh7mini`, `modalai/voxl2`, `voxl2-slpi`). No `px4_fmu-v6x`/`v6c` default enables
> it. On a typical board, `SYS_AUTOSTART 1200` will resolve to nothing and `HIL_ACT_FUNC*` will
> not appear until you enable it:
>
> ```bash
> make <board> boardconfig      # Modules -> Simulation -> pwm_out_sim
> make <board>
> ```
>
> This is upstream behaviour and affects PX4's own stock `1001`/`1002` HIL airframes identically —
> it is not something these files introduce — but it is the first thing that will bite you.

**Why the model JSONs are unchanged.** `HIL_ACT_FUNC<N>` lands on
`HIL_ACTUATOR_CONTROLS.controls[N-1]`, exactly as `PWM_MAIN_FUNC<N>` does in SITL. The
substitution is one-for-one at the same `<N>` with the same numeric function value, so the `px4`
indices in `models/*.json` mean the same thing on both paths.

That is the whole reason the ordering decision lives in these params rather than in the JSON.
RealFlight channel N is driven by PX4 output channel N — every shipped `models/*.json` is a pure
identity map — and the channel order is chosen once, in `PWM_MAIN_FUNC<N>` for SITL and in the
`HIL_ACT_FUNC<N>` twin for HITL, the same place ArduPilot puts it (`SERVOn_FUNCTION`) and PX4's
own Gazebo bridge puts it (`SIM_GZ_*_FUNC*`). One channel numbering therefore survives from SITL
through HITL to the physical output pinout, and moving a vehicle from the simulator to a board
changes no channel numbers anywhere. If your RealFlight model uses a different order, change
`HIL_ACT_FUNC<N>` and leave the JSON as an identity map.

**What was dropped from the SITL airframes**, and why — these are SITL-only and wrong or
meaningless on a board. A line-by-line diff of each `.hil` against its SITL twin gives **seven**
dropped settings, identical in all four pairs:

- `EKF2_GPS_DELAY 0` — existed only to countermand `px4-rc.mavlinksim`, which runs after the
  airframe in SITL. No such script exists on a board, and forcing 0 is actively wrong there: HITL
  GPS arrives over a real link with real latency, so it would mis-time EKF fusion.
- `EKF2_MULTI_IMU 1` — same rationale; the firmware default is already 1.
- `SDLOG_MODE 4` — "from first arm until shutdown" never closes the log, which risks a truncated
  file on a power-cut with a real SD card.
- `SIM_BAT_ENABLE` — a SITL-only parameter that does not exist in a board build. See §6.4 for why
  HITL cannot have real battery telemetry at all.
- `COM_FLTT_LOW_ACT 0` — SITL disables the low-flight-time action. Dropping it restores the
  firmware default.
- `COM_FAIL_ACT_T 0` — SITL zeroes the failsafe action delay. Dropping it restores the firmware
  default.
- `BAT1_SOURCE 1` — declares the battery as externally provided, which suits SITL's simulated
  pack. Dropping it restores the firmware default.

The last three are failsafe-adjacent, and dropping them is the **safe** direction: the board falls
back to the real firmware failsafe defaults instead of inheriting SITL's deliberately-disabled
ones. Keeping them would have carried a simulator's relaxed failsafe configuration onto hardware.
They are listed explicitly because this list is the one a reader auditing failsafe behaviour will
read as exhaustive.

The per-vehicle tables below are kept as **reference for debugging a mis-wired model** — they are
no longer instructions.

**Quad** (`CA_AIRFRAME 0`, model `quad.json`):

| Param | Value | → `controls[]` | RealFlight ch |
|---|---|---|---|
| `HIL_ACT_FUNC1` | 101 (Motor 1) | `[0]` | rf0 |
| `HIL_ACT_FUNC2` | 102 (Motor 2) | `[1]` | rf1 |
| `HIL_ACT_FUNC3` | 103 (Motor 3) | `[2]` | rf2 |
| `HIL_ACT_FUNC4` | 104 (Motor 4) | `[3]` | rf3 |

Plus `CA_ROTOR_COUNT 4` and the `CA_ROTOR*_P[XY]`/`CA_ROTOR*_KM` values from
`1201_flightaxis_quad`.

**Plane** (`CA_AIRFRAME 1`, model `plane.json`):

| Param | Value | → `controls[]` | RealFlight ch |
|---|---|---|---|
| `HIL_ACT_FUNC1` | 201 (Servo 1, aileron L) | `[0]` | rf0 |
| `HIL_ACT_FUNC2` | 203 (Servo 3, elevator) | `[1]` | rf1 |
| `HIL_ACT_FUNC3` | 101 (Motor 1, throttle) | `[2]` | rf2 |
| `HIL_ACT_FUNC4` | 204 (Servo 4, rudder) | `[3]` | rf3 |
| `HIL_ACT_FUNC5` | 202 (Servo 2, aileron R) | `[4]` | rf4 |

Plus the `CA_SV_CS*` surface definitions from `1200_flightaxis_plane`.

The values are non-sequential because the allocator numbers its outputs by kind — motors
(`101…`) before servos (`201…`) — which is not the order a RealFlight fixed-wing is wired in.
The FUNC list absorbs that mismatch, which is why the quad above needs no reordering (it is all
motors) and the other three do.

**Quadplane** (`CA_AIRFRAME 2`): `HIL_ACT_FUNC1..9` = 201, 203, 105, 204, 101, 102, 103, 104, 202
— surfaces on `controls[0..3]`, the four lift motors on `[4..7]`, aileron right on `[8]` = rf8 —
mirroring `1202_flightaxis_quadplane`. `FUNC10..12` are unset, so RealFlight ch10–12 sit at a
steady neutral 0.5 until you assign something to them.

**Heli** (`CA_AIRFRAME 11`): `HIL_ACT_FUNC1..4` = 202, 203, 204, 201 and `HIL_ACT_FUNC8` = 101,
with `FUNC5..7` explicitly 0 — swash servos 1–3 on `controls[0..2]`, yaw tail on `[3]`, main
rotor on `[7]` — mirroring `1203_flightaxis_heli`. That is ArduPilot's own heli channel order
(`AP_MotorsHeli_Swash.cpp:201-202` swash → `SERVO1/2/3`, `AP_MotorsHeli_Single.cpp:207` tail →
`SERVO4`, `AP_MotorsHeli.h:34` RSC → `SERVO8`), so a RealFlight heli model already set up for
ArduPilot needs no re-mapping; RealFlight channels 5–7 stay idle at 0.5 in both. As in SITL,
PX4 does the CCPM mix and the bridge ships the three swash **servo positions** untouched —
`HeliDemix` is **off** in `heli.json` — so the RealFlight model must be wired non-mixed /
direct-servo. That choice is made bridge-side in the model JSON and needs no board parameter.

Verify the result in QGC's **Actuators** tab, which reads `HIL_ACT_FUNC*` once `SYS_HITL>0`.

### 6.4 Battery telemetry is SITL-only

In SITL the bridge forwards RealFlight's real battery (pack voltage, current draw, or fuel on
internal-combustion models) into `battery_status`. **On a HITL board it cannot, and this is not a
gap that can be closed from our side.**

`MavlinkReceiver::handle_message_hil_sensor()` publishes a **hardcoded** battery on **every**
`HIL_SENSOR` message it receives:

```c
hil_battery_status.voltage_v = 16.0f;
hil_battery_status.current_a = 10.0f;
hil_battery_status.remaining = 0.70;
```

The bridge sends `HIL_SENSOR` at ~250 Hz. A real reading injected at 2 Hz would be overwritten
roughly 125 times between updates, so the topic would show 16.0 V / 10 A / 70 % essentially always.
No parameter disables this. Changing it means patching PX4 itself, which this project deliberately
does not do — `install.sh` only ever *adds* files and splices CMakeLists registrations, so that the
integration keeps working across PX4 versions.

The bridge therefore does not even open its battery socket unless the transport is SITL
(`tcp-server`); it logs one line saying so on the HITL paths. **Expect a constant 16.0 V / 70 % on
a HITL board, and do not use HITL to test low-battery failsafes** — use SITL for that.

---

## 7. Running it

Build the bridge (this doesn't build or start PX4 SITL):

```bash
cd ~/PX4-Autopilot
ninja -C build/px4_sitl_nolockstep flightaxis_bridge
```

Run against the board:

```bash
# USB, quad model, RealFlight on this machine
./Tools/simulation/flightaxis/hitl_run.sh quad "$PWD" "$PWD/build/px4_sitl_nolockstep"

# explicit device
./Tools/simulation/flightaxis/hitl_run.sh quad "$PWD" "$PWD/build/px4_sitl_nolockstep" /dev/ttyUSB0

# RealFlight on another machine
PX4_FLIGHTAXIS_IP=192.168.1.50 \
  ./Tools/simulation/flightaxis/hitl_run.sh plane "$PWD" "$PWD/build/px4_sitl_nolockstep"
```

There is also a make-style target, though see the caveat below:

```bash
ninja -C build/px4_sitl_nolockstep flightaxis_hitl_quad
```

> **On the `flightaxis_hitl_*` targets.** They depend on `flightaxis_bridge` only —
> deliberately not on `px4`. The awkward part, stated plainly: they are declared in
> `src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake`, and HITL does not
> use `simulator_mavlink` at all. They live there because that is where the
> `flightaxis_bridge` ExternalProject is defined and a target cannot depend on it from
> elsewhere. The consequence is that you configure a `px4_sitl_nolockstep` tree to get a HITL
> runner — odd, but harmless, since nothing SITL is built or started. If you would rather not
> carry that oddity, call `hitl_run.sh` directly; it needs nothing from CMake beyond the built
> binary.

### Interface

The positional argv protocol is **unchanged**, so existing callers
(`get_FAbridge_params.py` + `sitl_run.sh`) keep working verbatim. Transport selection is by
environment variable — consistent with `PX4_HOME_LAT/LON/ALT/YAW` and `PX4_FLIGHTAXIS_IP`, and it
avoids an optional leading flag that every downstream positional index would have to account
for.

| Variable | Default | Meaning |
|---|---|---|
| `PX4_HITL_TRANSPORT` | `tcp-server` | `tcp-server` (SITL) \| `serial` \| `udp`. The default is the *bridge's*; `hitl_run.sh` defaults it to `serial` instead and rejects `tcp-server`, since a HITL runner has no SITL to serve. |
| `PX4_HITL_SERIAL_DEV` | — | e.g. `/dev/ttyACM0` |
| `PX4_HITL_SERIAL_BAUD` | `921600` | ignored on USB CDC-ACM |
| `PX4_HITL_UDP_HOST` | — | board IP |
| `PX4_HITL_UDP_PORT` | `14550` | |
| `PX4_HITL_SENSOR_HZ` | `250` serial/udp, `0` tcp | `0` = every frame (~1 kHz); only on USB |
| `PX4_HITL_STATE_QUAT_BYPASS` | unset | debug only — see §2 |
| `PX4_FLIGHTAXIS_IP` | `127.0.0.1` | RealFlight host |

Selecting `serial` or `udp` also selects the HITL message profile (no `HIL_STATE_QUATERNION`,
no `RAW_RPM`, no `RC_CHANNELS`, `HIL_GPS` at 5 Hz, mag/baro/airspeed at 50 Hz, plus `HEARTBEAT`).

**`RC_CHANNELS` is disabled too** (`px4_communicator.cpp:130`), so RealFlight's transmitter sticks
are not forwarded and a HITL board gets no RC passthrough. Two reasons, and neither is bandwidth:
`mavlink_receiver.cpp` has no `RC_CHANNELS` handler, so the stream would be discarded anyway; and
on a real board the pilot's own receiver is the authoritative RC path and must not be
second-guessed from the simulator. This is coherent with the `COM_RC_IN_MODE 4` of §6.1 — RC input
ignored entirely. Fly from the GCS, or from a real receiver bound to the board with
`COM_RC_IN_MODE` set to match.

---

## 8. Bringing it up

1. Props off. Confirm visually.
2. Set the §6 parameters, **reboot the board**.
3. Start RealFlight, load the matching model, enable RealFlight Link.
4. Close QGroundControl if it holds the port you are about to use. It may stay open on a
   *different* port — see §11, "Sharing the board with QGroundControl".
5. Run `hitl_run.sh`.
6. Watch for `first HIL_ACTUATOR_CONTROLS received, enabling channels`. If it never appears,
   go to §9.
7. Check the board's estimator has converged (QGC on a *different* link, or the MAVLink
   console) before attempting to arm.

---

## 9. Troubleshooting

**No `HIL_ACTUATOR_CONTROLS` ever arrives.** In order of likelihood:

1. `MAV_x_RATE` ≤ 5000 on a hardware UART — `set_hil_enabled()` refuses to configure the
   streams and says nothing (`mavlink_main.cpp:671`). Two ways to land there: instance 0, whose
   default is 1200 B/s, or any instance left at `0` (baud/20) on a port running at 100000 baud
   or slower. The recommended setup in §6.2 — `MAV_1_RATE` set explicitly on a 921600 UART —
   avoids both. Set it to `80000`, or use USB where PX4 forces 100000 B/s.
2. `SYS_HITL` set but the board not rebooted. It is `@reboot_required`.
3. Firmware built without `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM` (§5).
4. Wrong port, or QGC/MAVProxy holding it — the runner checks this with `fuser`.
5. UART TX/RX not crossed.

**`cannot open /dev/ttyACM0: Permission denied`** — add yourself to `dialout`:
`sudo usermod -aG dialout $USER`, then log out and back in.

**Estimator will not converge.** Confirm `HIL_GPS` is arriving (5 Hz). Confirm `EKF2_GPS_DELAY`
is at its firmware default and has **not** been forced to `0` — that value belongs to the SITL
airframes and mis-times fusion on a board, where the samples cross a real link with real latency
(§6.3). Confirm you have **not** set `PX4_HITL_STATE_QUAT_BYPASS`, which fights EKF2 (§2).

**Garbage / `BAD_DATA` on the link.** Baud mismatch between `PX4_HITL_SERIAL_BAUD` and
`SER_TELx_BAUD`, or flow control expected by the adapter but absent on a 3-wire cable.

**Rates lower than requested.** Expected if the loop is throttled — see the quantisation note
in §3. Check the bridge's `loop=` figure; if it is near the frame rate rather than several
kHz, the link is backpressuring.

**`WARNING: RealFlight physics speed multiplier is …`** — set it to 1.0 in RealFlight.

---

## 10. Reverting to SITL

Nothing to undo on the PC side: omit the `PX4_HITL_*` variables and `sitl_run.sh` behaves
exactly as before. On the board, set `SYS_HITL 0`, clear `CBRK_SUPPLY_CHK`, restore
`COM_RC_IN_MODE`, and reboot.

---

## 11. Bench bring-up checklist

**Props off for all of it.** This section is what to confirm on your own bench, in order, before
the airframe is assembled and long before flight power is applied. A board will drive whatever
is wired to it; nothing in this document can tell you what that is.

### Link and stream

- The board enters HIL and streams `HIL_ACTUATOR_CONTROLS` — the runner prints
  `first HIL_ACTUATOR_CONTROLS received, enabling channels`. If it does not, work §9 in order.
- The `_datarate > 5000` gate is cleared on your transport (§2). USB forces 100000 B/s; a
  hardware UART needs `MAV_x_RATE` set explicitly.
- Sustained throughput at your chosen baud. Watch the bridge's `loop=` figure and the achieved
  `HIL_SENSOR` rate; if the rate sits well below the requested one, the link is backpressuring
  (§3).
- Whether 250 Hz is adequate for EKF2 on your board, and whether its CPU keeps up. Raise or
  lower `PX4_HITL_SENSOR_HZ` against the §3 bandwidth budget.
- UDP transport, if you are using it rather than serial or USB.

### Outputs — the part that can hurt you

Step each output individually, with the propellers off and the airframe unassembled, and
confirm:

- **Ordering.** `HIL_ACT_FUNC<N>` moves the surface or motor you expect on RealFlight channel N
  (§6.3). Check every channel, not a sample.
- **Direction.** Each surface deflects the correct way and each motor spins the correct way.
- **Endpoints.** Full-scale commands do not drive a surface into a mechanical stop.
- **Failsafe behaviour.** Confirm what the outputs do when the link drops, when the vehicle
  disarms, and on a board reset.
- QGC's **Actuators** tab renders the geometry as intended once `SYS_HITL>0`.

### Parameters and preflight

- The §1 interlocks on your own board rather than on trust: with `SYS_HITL > 0` set and the
  board rebooted, watch the servo rail and confirm it stays silent when armed. Props stay off
  regardless of the result.
- Whether `CBRK_SUPPLY_CHK` and `COM_RC_IN_MODE 4` interact with any board-specific preflight
  check.
- **Controller gains.** They are dimensional and inherited from the SITL airframes. Tune them on
  your vehicle, starting conservatively; the helicopter's rate gains in particular should not be
  assumed to suit your model.

### Effect on the SITL path

Selecting a HITL transport changes the message profile only. In the SITL profile the
`HIL_SENSOR` and `RAW_RPM` intervals are `0` (send every call), `HIL_GPS` /
`HIL_STATE_QUATERNION` / `DISTANCE_SENSOR` keep their original constants, `RC_CHANNELS` is
enabled, the heartbeat is disabled, and the send path resolves to the same `send()` on the same
accepted socket. Of these, `HIL_STATE_QUATERNION`, `RAW_RPM` and `RC_CHANNELS` are the three the
HITL profile switches off.

**One SITL behaviour does differ from stock:** the `fields_updated` sub-rates apply in SITL too
(mag 100 Hz, baro and differential pressure 50 Hz) rather than every message carrying `0x1FFF`.
That is a deliberate, separately-motivated change — it keeps logs from being inflated by
redundant sensor republication — and it is not required by HITL, which sets its own values in
`Configure()`. It is called out because it is the one place the SITL wire content is not
byte-identical to stock.

### Sharing the board with QGroundControl

**Run the bridge and a GCS on two different MAVLink instances.** This is the recommended setup
and it needs no change to the bridge: bridge on TELEM2, QGroundControl on USB. Step 7 of §8
already assumes it.

What makes it work is that HIL is a *per-instance* property. `set_hil_enabled()` is called from
`Mavlink::handleStatus()` (`mavlink_main.cpp:2542`) on every instance independently, so the
instance carrying the bridge enters HIL and adds its `HIL_ACTUATOR_CONTROLS` stream while another
instance goes on serving normal telemetry to a GCS. PX4 v1.16 gives you three `MAV_x_CONFIG`
instances plus the auto-started USB instance (`init.d/rcS:498-505`). Any instance that clears the
`_datarate > 5000` gate of §2 can be the HIL one; the others are unaffected.

The genuine constraint is narrower than "the link cannot be shared": **two `mavlink` instances
cannot bind the same device.** `mavlink_main.cpp:2196-2200` rejects the second one outright. So
the real limitation is a board with only one usable port — there you must choose between the
bridge and a GCS. On any board with USB plus a TELEM port, which is essentially all of them, there
is nothing to work around.

**What is not a workaround:** `MAV_x_FORWARD`. It forwards only messages that are "either
broadcast or the target is not the autopilot" (`module.yaml:100-108`). It does not duplicate an
instance's telemetry streams, so it will not give QGroundControl a view of a HIL session on
another link. §6.2 leaves `MAV_1_FORWARD` at `0` for that reason.
