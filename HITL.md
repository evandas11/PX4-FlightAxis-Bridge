# Hardware-in-the-loop (HITL) against RealFlight

How to drive a **real flight controller board** from RealFlight, instead of PX4 SITL.
For the SITL workflow see [`RUNNING.md`](RUNNING.md); for the design rationale see
[`FLIGHTAXIS_PX4_INTEGRATION.md`](FLIGHTAXIS_PX4_INTEGRATION.md).

Assumes the bridge is installed into a PX4 v1.16 checkout (`./install.sh`) and that
`~/PX4-Autopilot` is that checkout.

> **Status.** The transport, framing, decimation, message profile and receive path are
> verified without hardware (PTY loopback + a MAVLink-decoding harness). **Nothing in this
> document has been run against a physical board.** §11 states exactly what is verified and
> what is not — read it before trusting anything here.

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
| Actuator source topic | `actuator_outputs` | `actuator_outputs_sim` |
| Output function params | `PWM_MAIN_FUNC<N>` | `HIL_ACT_FUNC<N>` |
| Real PWM/DShot outputs | n/a | **not started at all** |

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
   PX4 silently never enables the HIL streams. `MAV_x_RATE` defaults to **1200 B/s**. This is
   the single most common reason a HITL link looks alive and never produces actuator controls.
   See §7.
2. `HIL_STATE_QUATERNION` is only streamed **out** when `SYS_HITL == 2` (SIH), as ground truth.

### Why the bridge must not send `HIL_STATE_QUATERNION` in HITL

This is the biggest behavioural difference from the SITL path and it is not a bandwidth
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
Payloads are trailing-zero truncated. Measured on the PTY harness:

| Message | Measured on wire |
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

**HITL profile at 250 Hz** (no `HIL_STATE_QUATERNION`, no `RAW_RPM`):

```
250×77 + 5×52 + 20×55 + 1×25  =  19250 + 260 + 1100 + 25  ≈  20.6 kB/s  =  206 kbit/s
```

| Baud | Capacity | Load at 250 Hz | Verdict |
|---|---|---|---|
| 115200 | 11.5 kB/s | 179 % | unusable — even 100 Hz does not fit |
| 230400 | 23.0 kB/s | 90 % | too tight, no headroom |
| 460800 | 46.1 kB/s | 45 % | fine |
| **921600** | **92.2 kB/s** | **22 %** | **recommended**; 500 Hz also fits (43 %) |
| USB CDC-ACM | ~1 MB/s | ~2 % | baud is nominal and ignored; best option |

Return direction is independent (serial is full duplex): 200 Hz × 93 B = **18.6 kB/s** of
`HIL_ACTUATOR_CONTROLS`. Note this must clear the `_datarate > 5000` gate from §2.

**Why 250 Hz is the default:** it matches RealFlight's own physics frame rate, so nothing is
lost — the 1 ms extrapolation steps are interpolation between frames, not new information. It
fits comfortably from 460800 up. Decimation had to be added here at all because PX4's other
simulator bridges run over loopback, where bytes are free and no decimation is needed; a
serial link to a board is the case they do not cover.

### Rate quantisation (read before comparing measured rates)

Each rate gate is `if (now - last >= interval) { last = now; ... }` against the **physics**
clock. The achieved period is therefore the requested period **rounded up to however far the
physics clock moves per `Send()` call**.

When the main loop free-runs (several thousand iterations/s against RealFlight) that step is the
1 ms extrapolation quantum, and 250 Hz comes out exact. If the loop is throttled — a slow link,
a reader that is not draining — the step grows and the rate drops: at a 3 ms step, a 4 ms
interval yields 6 ms, i.e. 166 Hz. This is what the PTY measurements in §11 show.

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
| `EKF2_GPS_DELAY` | `0` | RealFlight free-runs; GPS samples arrive fresh (same as the SITL airframes) |
| `EKF2_MULTI_IMU` | `1` | The bridge sends `HIL_SENSOR` with `id = 0` only, so only IMU instance 0 is ever supplied; leaving this higher runs EKF instances on dead sensors (same as the SITL airframes) |
| `SDLOG_MODE` | `4` | Optional, but it makes a log span several arm/disarm cycles instead of closing at the first disarm. `@reboot_required` |

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

**Our `1200`–`1203` airframes cannot be reused.** They live in `init.d-posix`, which is the
POSIX/SITL startup path; a real board boots from `init.d`. They also use `PWM_MAIN_FUNC<N>`,
whereas a HITL board routes through `HIL_ACT_FUNC<N>` (`pwm_out_sim/module_hil.yaml`, prefix
`HIL_ACT`, 16 channels, visible only when `SYS_HITL>0`).

Rather than ship ROMFS airframes for boards we cannot test, here is the parameter list to set
by hand. Pick a stock airframe of the right type (`SYS_AUTOSTART`), then set the geometry
exactly as the matching `init.d-posix/airframes/120x_flightaxis_*` file does, and map the
outputs with `HIL_ACT_FUNC<N>` instead of `PWM_MAIN_FUNC<N>`.

The rule is a straight substitution — **`HIL_ACT_FUNC<N>` lands on
`HIL_ACTUATOR_CONTROLS.controls[N-1]`**, exactly as `PWM_MAIN_FUNC<N>` does in SITL — so the
`px4` indices in `models/*.json` are unchanged.

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
| `HIL_ACT_FUNC1` | 101 (Motor 1, throttle) | `[0]` | rf2 |
| `HIL_ACT_FUNC2` | 201 (Servo 1, aileron L) | `[1]` | rf0 |
| `HIL_ACT_FUNC3` | 202 (Servo 2, aileron R) | `[2]` | unmapped |
| `HIL_ACT_FUNC4` | 203 (Servo 3, elevator) | `[3]` | rf1 (reversed) |
| `HIL_ACT_FUNC5` | 204 (Servo 4, rudder) | `[4]` | rf3 |

Plus the `CA_SV_CS*` surface definitions from `1200_flightaxis_plane`.

**Quadplane** (`CA_AIRFRAME 2`): `HIL_ACT_FUNC1..5` = 101–105, `HIL_ACT_FUNC6..9` = 201–204,
mirroring `1202_flightaxis_quadplane`.

**Heli** (`CA_AIRFRAME 11`): `HIL_ACT_FUNC1` = 101, `HIL_ACT_FUNC2..5` = 201–204, mirroring
`1203_flightaxis_heli`. The bridge's `HeliDemix` option is applied bridge-side and needs no
board parameter.

Verify the result in QGC's **Actuators** tab, which reads `HIL_ACT_FUNC*` once `SYS_HITL>0`.

---

## 7. Running it

Build the bridge (this does not build or start PX4 SITL):

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
environment variable — consistent with `PX4_HOME_LAT/LON/ALT` and `PX4_FLIGHTAXIS_IP`, and it
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
no `RAW_RPM`, `HIL_GPS` at 5 Hz, mag/baro/airspeed at 50 Hz, plus `HEARTBEAT`).

---

## 8. Bringing it up

1. Props off. Confirm visually.
2. Set the §6 parameters, **reboot the board**.
3. Start RealFlight, load the matching model, enable FlightAxis Link.
4. Close QGroundControl if it holds the port you are about to use.
5. Run `hitl_run.sh`.
6. Watch for `first HIL_ACTUATOR_CONTROLS received, enabling channels`. If it never appears,
   go to §9.
7. Check the board's estimator has converged (QGC on a *different* link, or the MAVLink
   console) before attempting to arm.

---

## 9. Troubleshooting

**No `HIL_ACTUATOR_CONTROLS` ever arrives.** In order of likelihood:

1. `MAV_x_RATE` ≤ 5000 on a hardware UART. This is the big one — `set_hil_enabled()` refuses
   to configure the streams and says nothing (`mavlink_main.cpp:671`). Set it to `80000`, or
   use USB where PX4 forces 100000 B/s.
2. `SYS_HITL` set but the board not rebooted. It is `@reboot_required`.
3. Firmware built without `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM` (§5).
4. Wrong port, or QGC/MAVProxy holding it — the runner checks this with `fuser`.
5. UART TX/RX not crossed.

**`cannot open /dev/ttyACM0: Permission denied`** — add yourself to `dialout`:
`sudo usermod -aG dialout $USER`, then log out and back in.

**Estimator will not converge.** Confirm `HIL_GPS` is arriving (5 Hz) and that `EKF2_GPS_DELAY`
is `0`. Confirm you have **not** set `PX4_HITL_STATE_QUAT_BYPASS`, which fights EKF2 (§2).

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

## 11. What is verified and what is not

**Honest summary: no physical flight controller was involved at any point.**

### Verified without hardware

- **SITL not regressed.** Full stack (mock RealFlight + bridge + `px4`) for 50 s on an isolated
  loopback address: EKF2 converged (`home set`, `Ready for takeoff!`), both processes alive at
  teardown, bridge steady at 250.0 FPS, channel map exactly `[0,0,0,0, 0.5×8]` for `quad`. The
  three `simulator_mavlink poll timeout` messages all occur before readiness and are the
  pre-existing 5 s `sleep()` in `PX4Communicator::Init()`, not a regression.
- **SITL transport path unchanged by construction.** In the SITL profile the `HIL_SENSOR` and
  `RAW_RPM` intervals are `0` (send every call), `HIL_GPS`/`HIL_STATE_QUATERNION`/
  `DISTANCE_SENSOR` keep their original constants, the heartbeat is disabled, and the send path
  resolves to the same `send()` on the same accepted socket.
  **One SITL behaviour does change:** the `fields_updated` sub-rates now also apply in SITL
  (mag 100 Hz, baro/differential-pressure 50 Hz) rather than every message carrying `0x1FFF`.
  That was a deliberate, separately-motivated change (it keeps ulogs from being inflated by
  redundant sensor republication) and it is not required by HITL — HITL sets its own values in
  `Configure()`. It is called out here because it is the one place the SITL wire content is not
  byte-identical to before.
- **Serial transport over a PTY pair.** termios accepted at 921600; framing decodes cleanly
  with **zero** undecodable bytes; **zero** `HIL_SENSOR` timestamp regressions; injected
  `HIL_ACTUATOR_CONTROLS` at 200 Hz received and applied (`first HIL_ACTUATOR_CONTROLS
  received`).
- **`hitl_run.sh` end to end**, including the `fuser` port-in-use guard (verified by tripping
  it deliberately), the missing-device and permission guards, and the non-USB advisory.
- **HITL message profile**, 18 s free-running run: `HIL_SENSOR` **224.8 Hz**,
  `DISTANCE_SENSOR` 19.5 Hz (target 20), `HIL_GPS` 4.9 Hz (target 5), `HEARTBEAT` 0.9 Hz
  (target 1). `HIL_STATE_QUATERNION` and `RAW_RPM` confirmed **absent**.
- **Aggregate bandwidth 17.7 kB/s** = 177 kbit/s, i.e. **19 % of 921600 baud** — consistent
  with the §3 budget and leaving ample headroom.
- **Message sizes** measured on the wire: 77 / 52 / 55 / 25 B (§3).
- **Everything builds** warning-free at `-O2 -Wall -Wextra`; all pre-existing make targets
  still present.

`HIL_SENSOR` lands at 224.8 Hz rather than exactly 250 — the quantisation effect in §3, always
toward *fewer* messages, so the bandwidth budget is never exceeded. An earlier run of the same
binary measured only 166 Hz because the harness held the PTY slave open, throttling the bridge
loop to 250 iterations/s (3 ms physics step, so a 4 ms interval rounds to 6 ms). Releasing the
slave let the loop free-run at ~12000 iterations/s and the rate rose to 224.8 Hz, which
confirms the mechanism: **achieved rate is a function of loop granularity, not of the gate
being wrong.**

### NOT verified — requires a board

- That a real board enters HIL and streams `HIL_ACTUATOR_CONTROLS` at all.
- The `_datarate > 5000` gate behaviour in practice, and whether USB really needs no tuning.
- Sustained throughput at a real 921600 baud. **A PTY has no baud rate** — it accepts
  `cfsetspeed` and ignores it. Real UART timing, buffer depth and overruns are untested.
- Whether 250 Hz is genuinely adequate for a real EKF2 on real hardware, or whether the board's
  CPU keeps up with it.
- The `HIL_ACT_FUNC<N>` mappings in §6.3. These are derived by substitution from the working
  SITL `PWM_MAIN_FUNC<N>` maps and the stock `.hil` airframes; the reasoning is sound and the
  normalisation is provably identical (`PWMSim::updateOutputs` is the single shared function),
  but no channel has been confirmed to move the right surface on a real board.
- UDP transport end to end — implemented and compiled, never run against a networked FC.
- Every safety claim in §1 as *observed behaviour*. The code paths were read and cited; the
  servo rail was never watched.

### Known gap

The bridge occupies the board's MAVLink link exclusively, so QGroundControl cannot share it.
For a board with only one convenient port you must choose between the bridge and a GCS. A
MAVLink router/forwarder in the bridge would fix this and is the obvious next feature; it is
not implemented.
