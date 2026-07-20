# Running PX4 SITL against RealFlight (FlightAxis)

Operational guide: how to actually fly it. For *why* it works the way it does, see
[`FLIGHTAXIS_PX4_INTEGRATION.md`](FLIGHTAXIS_PX4_INTEGRATION.md). For installation, see
[`README.md`](README.md).

Assumes the bridge is already installed into a PX4 v1.16 checkout (`./install.sh`), and that
`~/PX4-Autopilot` is that checkout — substitute `<your-px4>` if it lives elsewhere.

---

## Quick reference — every shipped model

Four vehicles ship. Each has a self-contained section in §2 — jump straight to yours, you do not
need to read the others. Everything below assumes the **`px4_sitl_nolockstep`** build.

| Make target | Airframe (`SYS_AUTOSTART`) | Vehicle type | Description |
|---|---|---|---|
| `flightaxis_plane` | `1200_flightaxis_plane` (1200) | Fixed-wing (`CA_AIRFRAME 1`) | Conventional plane: aileron / elevator / throttle / rudder |
| `flightaxis_quad` | `1201_flightaxis_quad` (1201) | Multirotor quad-X (`CA_AIRFRAME 0`) | Four motors, direct from PX4 motor outputs |
| `flightaxis_quadplane` | `1202_flightaxis_quadplane` (1202) | Standard VTOL (`CA_AIRFRAME 2`) | 4 lift motors + pusher + 4 control surfaces |
| `flightaxis_heli` | `1203_flightaxis_heli` (1203) | Helicopter, tail servo (`CA_AIRFRAME 11`) | Collective-pitch heli; PX4 does the CCPM mix, bridge passes swash servos through untouched |

`flightaxis` on its own is an alias for `flightaxis_plane`.

RealFlight is an RC flight simulator, so these four aircraft classes are what it can model.

---

## 1. Network setup (do this first)

This is where people get stuck. RealFlight runs on Windows; PX4 and the bridge run on Linux.
Nothing below is optional.

### 1.1 Enable RealFlight Link in RealFlight

**Simulation → Settings… → Physics → Quality → tick "RealFlight Link Enabled".**

RealFlight then listens on **TCP 18083**.

> On some RealFlight builds the Quality preset has to be set to **Custom** before the
> checkbox becomes editable.

Also in RealFlight, before flying:

- **Physics speed multiplier must be 1.0.** The bridge warns if it is not. Speed-multiplier
  scaling and a real-time physics simulator such as RealFlight do not mix: the multiplier makes
  the physics clock advance at a rate the bridge's timing logic cannot reconcile.
- Turn off any RealFlight-side flight assistance / virtual gyros on the model.

### 1.2 Find the Windows machine's IP

On the Windows box:

```
ipconfig
```

Take the IPv4 address of the adapter on the same wired subnet as the Linux machine
(e.g. `192.168.10.1`).

### 1.3 Open the Windows firewall

Windows Firewall blocks inbound 18083 by default. In an **Administrator** PowerShell on the
Windows machine:

```powershell
New-NetFirewallRule -DisplayName "RealFlight FlightAxis" -Direction Inbound -Protocol TCP -LocalPort 18083 -Action Allow
```

Or allow RealFlight itself through "Windows Defender Firewall → Allow an app". If the network
profile is "Public", either switch it to "Private" or make sure the rule applies to Public too.

### 1.4 Use a wired link

**WiFi cannot hold the ~250 Hz SOAP round-trip rate.** Every physics frame is one full HTTP/SOAP
request/response; WiFi latency and jitter turn that into a rising `glitches=` counter and an
unflyable simulation. Wired Ethernet, or RealFlight in a VM on the same box. This is a hard
requirement, not a recommendation.

### 1.5 Verify reachability *before* launching anything

From Linux:

```bash
cd ~/PX4-Autopilot/Tools/simulation/flightaxis/flightaxis_bridge
./FA_check.py 192.168.10.1
```

Healthy:

```
FlightAxis check: RealFlight reachable at 192.168.10.1:18083
```

Unreachable (exit code 1, and `make` will stop here too):

```
FlightAxis check: cannot reach 192.168.10.1:18083 (timed out)
  - Is RealFlight running with RealFlight Link enabled?
    (RealFlight: Simulation -> Settings... -> Physics -> Quality ->
     tick 'RealFlight Link Enabled')
  - Is the host reachable / firewall open on TCP 18083?
  - Set PX4_FLIGHTAXIS_IP if RealFlight is not on 192.168.10.1.
```

Or without the script:

```bash
nc -zv 192.168.10.1 18083
```

### 1.6 Same-host case

If RealFlight runs in a VM on the same machine with host networking, `PX4_FLIGHTAXIS_IP` can be
omitted entirely — **`sitl_run.sh` defaults it to `127.0.0.1`**.

```bash
make px4_sitl_nolockstep flightaxis_plane      # talks to 127.0.0.1:18083
```

---

## 2. Launch, per vehicle

All four targets require the **`px4_sitl_nolockstep`** build. RealFlight free-runs its own
physics clock; the lockstep build will not work and the targets do not exist there (they are
guarded by `if(ENABLE_LOCKSTEP_SCHEDULER STREQUAL "no")`).

Targets that exist: `flightaxis` (alias for plane), `flightaxis_plane`, `flightaxis_quad`,
`flightaxis_quadplane`, `flightaxis_heli`.

**In every command below, `192.168.10.1` is a worked example — it is the address of the Windows
box running RealFlight.** Substitute your own: the variable is
`PX4_FLIGHTAXIS_IP=<your-windows-ip>`, read by `sitl_run.sh`, and it defaults to `127.0.0.1`
(same-host case, §1.6). Find the right value with §1.2 and prove it is reachable with §1.5
*before* launching anything.

**The commands below all run in the shared working directory**, `build/px4_sitl_nolockstep/rootfs`,
which every model uses unless given one of its own. That is fine for one model. Fly a second and
the two share `parameters.bson`, dataman and `log/`: PX4 sees the saved `SYS_AUTOSTART` disagree
with the model it is starting and runs `param reset_all`, which keeps `RC*`, `CAL_*` and
`COM_FLTMODE*` but drops the tuning; and the mission left over from the first is still loaded for
the second. Add one line to any command here to avoid both:

```bash
PX4_FLIGHTAXIS_ROOTFS=~/sitl/fa-rootfs/plane \
```

The directory is created if missing and seeded with the calibration you already have, and the
same command then serves every later run — there is no separate first-run form. Full
explanation in [*A working directory per model*](README.md#a-working-directory-per-model).

**Switching from one vehicle to another** is that same rootfs argument plus two manual steps, in
this order:

1. **Ctrl-C the running target.** The bridge exits with PX4; there is nothing else to stop.
2. **Load the matching aircraft in RealFlight.** The bridge does not choose the model — it drives
   whatever RealFlight has loaded, so a plane airframe against a quad model flies exactly as badly
   as that sounds.
3. **Run the new target with its own `PX4_FLIGHTAXIS_ROOTFS`**, one directory per model. This is
   what keeps the two models' `parameters.bson`, missions and logs apart; sharing them is the
   `param reset_all` and stale-mission case described just above.

Changing the aircraft *while the bridge is running* is also handled, but as a recovery rather than
a workflow: RealFlight takes the controller back, and the bridge re-injects it and re-anchors —
see the `RealFlight controller inactive - reinjecting controller` entry in §7. PX4 is still
running the old airframe at that point, so restart it anyway.

Each section below is self-contained: launch command, airframe, RealFlight model preparation,
channel table, QGC check, gotchas, and a first-flight check for that vehicle. The
model-preparation steps immediately below apply to all four.

Sections are HITL-agnostic — they describe SITL. For hardware-in-the-loop, see `HITL.md`.

### Reading the channel tables

- **RF ch** — RealFlight channel slot (0-based, as the bridge counts them; RealFlight's own UI
  numbers channels from 1, so RF ch 0 is "Channel 1" there).
- **`controls[N]`** — index into `HIL_ACTUATOR_CONTROLS.controls[]`, set by `PWM_MAIN_FUNC<N+1>`
  in the airframe script.
- **Scale** — `unipolar` = the PX4 value is already `0..1` (motors), sent through unchanged;
  `bipolar` = the PX4 value is `-1..1` (surfaces), sent as `(v+1)/2`.
- **Disarm** — value sent while PX4 is disarmed or the control is NaN. The flattener also accepts
  `"disarm": -1`, meaning "keep the last output" (neutral 0.5 before the first one), but **no
  shipped model uses it**: all four tables below state a number on every row.

> ### One numbering, end to end
>
> **RealFlight channel N is driven by PX4 output channel N — for all twelve channels.**
>
> Every shipped `models/*.json` is a pure identity map covering **ch1–ch12**: the `rf` and `px4`
> columns are equal on every row, and every row is present. There are **no exceptions** and no
> renumbering anywhere in the JSON.
>
> `PWM_MAIN_FUNC<N>` (SITL) / `HIL_ACT_FUNC<N>` (HITL) alone decides *what* channel N carries.
> To change what comes out on a RealFlight channel, change that parameter — never the JSON.

(The tables below are 0-based, as the bridge counts: `rf` 0 is RealFlight "Channel 1". So
`rf` 11 is RealFlight Channel 12, the last one FlightAxis accepts.)

**Reversal is not done in the JSON.** No shipped model contains `"reverse": true`; direction
belongs to whoever sets up the aircraft. See [Reversing a channel](#reversing-a-channel).

The ordering decision lives in the airframe, in `PWM_MAIN_FUNC<N>` (SITL) or `HIL_ACT_FUNC<N>`
(HITL) — the same place ArduPilot puts it (`SERVOn_FUNCTION`) and the same place PX4's own Gazebo
bridge puts it (`SIM_GZ_*_FUNC*`). `PWM_MAIN_FUNC<N>` selects which allocator output lands on
`controls[N-1]`, so choosing the FUNC values *is* choosing the channel order. Keeping it there
rather than in the JSON means one channel numbering survives from SITL through HITL to the
physical flight-controller pinout.

This is why the FUNC values look scrambled for three of the four vehicles. PX4's control
allocator numbers its outputs by kind, always emitting **motors before servos** (`101…` then
`201…`), which is rarely the order a RealFlight model is wired in. Something has to absorb that
mismatch; it is now the FUNC list, which is a parameter you can inspect in QGC, rather than a
table buried in a JSON file.

If your RealFlight model uses a different channel order, **change `PWM_MAIN_FUNC<N>` to match it
and leave the JSON as an identity map.** Do not re-scramble the JSON — that splits the ordering
across two files that then have to be read together.

What stays in the JSON is only what *cannot* be expressed as a FUNC value: `scale`, `disarm`, and
the `HeliDemix` / `Rev4Servos` option transforms.

### `scale` is the one column you may still have to touch

`scale` is a **signal-domain conversion, not a preference**, and it is the single thing the JSON
cannot infer from your parameters. `PWMSim` emits non-reversible **Motor** functions as `[0,1]`
and **everything else** as `[-1,1]` ([`PWMSim.cpp:77-88`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/pwm_out_sim/PWMSim.cpp)),
while RealFlight always wants `[0,1]`. So:

| What you assigned to `PWM_MAIN_FUNC<N>` | Correct `scale` for that row | `disarm` |
|---|---|---|
| A **Motor** (`101…112`) | `unipolar` — passed through | `0.0` |
| Anything else — servo, control surface, flaps, gear, gimbal, peripheral | `bipolar` — sent as `(v+1)/2` | `0.5` (or `-1` for hold-last) |

**Both failure modes are silent.** A motor read as `bipolar` idles at half throttle and never
stops; a surface read as `unipolar` loses its entire negative half and only deflects one way.

**Rule:** whenever you change a channel's FUNC *between a Motor and anything else*, flip that
row's `scale` to match. Changing between two servo functions, or two motor functions, needs no
JSON edit at all.

### Using channels 9–12 (and any spare channel)

All twelve channels are mapped in every shipped model, so adding flaps, retractable gear, a
payload release or lights is a **parameter change only** — no JSON edit:

```
param set PWM_MAIN_FUNC9  406      # RC Flaps        -> RealFlight channel 9
param set PWM_MAIN_FUNC10 400      # Landing Gear    -> RealFlight channel 10
param set PWM_MAIN_FUNC11 430      # Gripper         -> RealFlight channel 11
param set PWM_MAIN_FUNC12 301      # Peripheral via Actuator Set 1 -> channel 12
```

Make it permanent by adding `param set-default PWM_MAIN_FUNC9 406` to the airframe script. Then
wire RealFlight channel 9 to the matching servo in the aircraft editor.

A channel whose FUNC is unset emits `0.0`, which the `bipolar` conversion sends as a steady
**0.5** — neutral, and exactly what these channels sent before they had rows. So mapping all
twelve costs nothing on an aircraft that only uses four.

**12 is a hard ceiling, and it is FlightAxis's, not ours** — the `ExchangeData` SOAP call carries
exactly twelve `<item>` values. PX4 itself would go to 16. See
[Channel limits](#channel-limits-what-actually-caps-you-at-12) for where each layer stops.

### Reversing a channel

No shipped model contains `"reverse": true`. There are two places to set direction, both outside
the JSON:

1. **PX4 side — `PWM_MAIN_REV`**, a bitmask over output channels; **bit `N-1` is channel `N`**.
   It *does* work in SITL: it is applied in `MixingOutput::output_limit_calc_single`
   (`mixer_module.cpp:531`), upstream of `PWMSim`, so it reaches RealFlight.

   ```
   param set PWM_MAIN_REV 2       # reverse channel 2 (elevator on plane/quadplane)
   param set PWM_MAIN_REV 256     # reverse channel 9  (bit 8)
   param set PWM_MAIN_REV 258     # both at once (2 | 256)
   ```

2. **RealFlight side** — each servo has its own direction in the model's servo setup.

> ⚠️ `PWM_MAIN_MIN` / `PWM_MAIN_MAX` are a **different story and do not work in SITL**: `PWMSim`
> overwrites them with `setAllMinValues` / `setAllMaxValues` (`PWMSim.cpp:49-50`), so per-channel
> endpoint and trim adjustment has no effect. Only `PWM_MAIN_REV` survives.

**Symptom of a wrong direction:** the surface moves the right amount the wrong way, so the
aircraft diverges instead of correcting — a reversed elevator pitches *down* when PX4 commands
nose-up. On a helicopter, do **not** reverse a single swash servo; the three are a coordinated
triple and flipping one reads as roll/pitch cross-coupling. Fix swash direction in the RealFlight
model or with `CA_SP0_ANG*`. The tail servo is the one heli channel a plain reversal suits.

### Channel limits: what actually caps you at 12

**The binding constraint is FlightAxis itself at 12** — it is the only layer in the chain below
16:

| Layer | Limit | Where |
|---|---|---|
| **RealFlight FlightAxis SOAP** | **12** ← *binding* | `ExchangeData` sends exactly 12 `<item>` values (`fa_communicator.cpp:204-217`). ArduPilot does the same: `float scaled_servos[12]` (`SIM_FlightAxis.cpp:329`). |
| Bridge channel array | 12 | `RF_CHANNELS = 12` (`flightaxis_bridge.cpp:109`); `exchangeData` clamps to 12 (`fa_communicator.cpp:772-775`); `selectedChannels = 4095` = `0xFFF` = 12 bits (`flightaxis_bridge.cpp:932`). |
| Model JSON validator | 12 | `RF_CHANNELS = 12`, `PX4_CONTROLS = 16` (`get_FAbridge_params.py:63-64`). |
| MAVLink `HIL_ACTUATOR_CONTROLS` | 16 | `float controls[16]` — protocol-fixed. |
| PX4 `actuator_outputs` | 16 | `NUM_ACTUATOR_OUTPUTS = 16` (`msg/ActuatorOutputs.msg:2`). |
| `PWMSim` / `pwm_out_sim` | 16 | `MAX_ACTUATORS = PWM_OUTPUT_MAX_CHANNELS = 16` (`drv_pwm_output.h:51`); `module_sim.yaml` declares `num_channels: 16`. |
| `PWM_MAIN_FUNC<N>` (SITL) | 16 | `PWM_MAIN_FUNC1…16` all exist. |
| `HIL_ACT_FUNC<N>` (HITL) | 16 | `module_hil.yaml` also declares `num_channels: 16` — same limit, no SITL/HITL difference. |
| Control allocation | 12 rotors + 8 surfaces = 20 | `CA_ROTOR_COUNT` max 12, `CA_SV_CS_COUNT` max 8 — more than enough to fill 12. |

So PX4 could drive 16, but only the first 12 can reach RealFlight. Assigning `PWM_MAIN_FUNC13`
and above is not an error — those outputs simply have nowhere to go.

### Preparing the RealFlight model (all vehicles)

Do this once per aircraft in RealFlight's aircraft editor:

- **Strip all expo and mixing** from the model's channel setup. The autopilot expects a linear
  channel → surface relationship; RealFlight's expo will fight the rate loops.
- **Remove any gyro / stabilisation / flight-assist** on the model.
- **Set servo speed to maximum** (zero servo lag). Modelled servo lag adds phase delay the PX4
  rate controllers were not tuned for.
- **One channel per actuator.** No RealFlight-side mixing of elevons, V-tails, or differential
  ailerons — PX4's control allocator does all mixing, and the bridge hands you raw per-actuator
  outputs.
- Match the model's channel order to the table for your vehicle below.

---

### 2.1 Plane

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 \
PX4_HOME_LON=175.7433944 \
PX4_HOME_ALT=48.0 \
PX4_HOME_YAW=235 \
make px4_sitl_nolockstep flightaxis_plane
```

Substitute your own field's coordinates and the runway heading you start on; see
[Home position and heading](#3-home-position-and-heading) for what each one does.

Airframe: `1200_flightaxis_plane` (`SYS_AUTOSTART` 1200, `CA_AIRFRAME 1`).
Channel map: `models/plane.json`. Options: `ResetPosition` + `SilenceFPS` (bitmask **9** = 1 | 8).

**RealFlight aircraft:** any conventional fixed-wing with aileron / elevator / throttle / rudder
on channels 1–4, which is how RealFlight fixed-wing models are conventionally wired.

| RF ch | PX4 | Drives | `PWM_MAIN_FUNC` | Scale | Disarm |
|---|---|---|---|---|---|
| 0 | `controls[0]` | aileron (PX4 **aileron left**) | 1 = 201 Servo 1 | bipolar | 0.5 |
| 1 | `controls[1]` | elevator | 2 = 203 Servo 3 | bipolar | 0.5 |
| 2 | `controls[2]` | throttle (Motor 1) | 3 = 101 Motor 1 | unipolar | **0.0** |
| 3 | `controls[3]` | rudder | 4 = 204 Servo 4 | bipolar | 0.5 |
| 4 | `controls[4]` | aileron **right** — see below | 5 = 202 Servo 2 | bipolar | 0.5 |
| 5–11 | `controls[5..11]` | *unassigned* — steady 0.5 | 6–12 unset | bipolar | 0.5 |

The FUNC values are non-sequential precisely because the allocator's own numbering is not the
RealFlight order — see [Reading the channel tables](#reading-the-channel-tables).

**Gotcha — split ailerons on RF ch5.** PX4 declares two ailerons (`CA_SV_CS0` left →
`controls[0]`, `CA_SV_CS1` right → `controls[4]`), and the airframe puts the right one on
`PWM_MAIN_FUNC5`, so **RealFlight channel 5 now carries a full-amplitude roll signal**. The two
allocator outputs are exact mirrors and `controls[0]` already swings the full ±1 range, so a
RealFlight model with one mixed aileron channel needs nothing on ch5.

⚠️ **RealFlight models commonly use ch5 for flaps, spoilers or gear.** If yours does, a
roll-correlated signal would drive that instead. Disable it:

```
param set PWM_MAIN_FUNC5 0     # ch5 returns to a steady neutral 0.5
```

If your model really is split-aileron, check the two surfaces move in **opposite** directions and
reverse ch5 with `param set PWM_MAIN_REV 16` (bit 4) if they do not.

**QGC check:** Vehicle Setup → Actuators should show Motor 1 plus four servos in the order
aileron-left, aileron-right, elevator, rudder.

**First flight.** Do the generic static/rates/taxi checks in §9 first, then: arm on the runway in
**Position** mode and confirm the throttle idles at zero (disarm value 0.0, not half). Take off
in **Stabilized**, hold wings level, and check the stick-to-surface polarity matches §9 step 2
(roll right → `p > 0`, pull up → `q > 0`, yaw right → `r > 0`).
Only then try **Mission** — the airframe sets `RWTO_TKOFF 1`, so a runway takeoff is expected,
and `NAV_ACC_RAD 15` means waypoints are considered hit from 15 m out.

---

### 2.2 Quad (multirotor)

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 \
PX4_HOME_LON=175.7433944 \
PX4_HOME_ALT=48.0 \
PX4_HOME_YAW=235 \
make px4_sitl_nolockstep flightaxis_quad
```

Airframe: `1201_flightaxis_quad` (`SYS_AUTOSTART` 1201, `CA_AIRFRAME 0`, quad X).
Channel map: `models/quad.json`. Options: `ResetPosition` + `SilenceFPS` (bitmask **9** = 1 | 8).

**RealFlight aircraft:** any quad-X multirotor whose four motors sit on channels 1–4
individually (no RealFlight-side mixing).

| RF ch | PX4 | Drives | `PWM_MAIN_FUNC` | Scale | Disarm |
|---|---|---|---|---|---|
| 0 | `controls[0]` | motor 1 | 1 = 101 Motor 1 | unipolar | **0.0** |
| 1 | `controls[1]` | motor 2 | 2 = 102 Motor 2 | unipolar | **0.0** |
| 2 | `controls[2]` | motor 3 | 3 = 103 Motor 3 | unipolar | **0.0** |
| 3 | `controls[3]` | motor 4 | 4 = 104 Motor 4 | unipolar | **0.0** |
| 4–11 | `controls[4..11]` | *unassigned* — steady 0.5 | 5–12 unset | bipolar | 0.5 |

If you assign a **motor** to one of ch5–12 (a hex or octo on this frame), change that row's
`scale` to `unipolar` and its `disarm` to `0.0` in `quad.json` — see
[`scale` is the one column you may still have to touch](#scale-is-the-one-column-you-may-still-have-to-touch).

Airframe side: `PWM_MAIN_FUNC1..4 = 101..104`. The one model where the allocator's own numbering
is already the RealFlight order, so the FUNC list comes out sequential and nothing has to be
absorbed anywhere — an all-motor airframe has no motors-before-servos problem to solve.

**Gotcha — motor numbering and spin direction.** PX4's quad-X numbering is
1=front-right (CW-position, `KM +0.05`), 2=rear-left, 3=front-left, 4=rear-right, per the
`CA_ROTOR*_PX/PY/KM` values in the airframe. If your RealFlight model numbers its motors
differently, or spins them the other way, the vehicle will flip on takeoff. Reorder
`PWM_MAIN_FUNC1..4` to match your model and leave `quad.json` as the identity map it is.

**QGC check:** Actuators shows exactly four motors, no servos. Sliding Motor 1 must spin the
RealFlight front-right rotor.

**First flight.** Verify the motor mapping *on the ground* before leaving it — in QGC Actuators,
slide each motor one at a time and confirm the correct RealFlight rotor spins, in the correct
direction. A wrong map here flips the aircraft in the first half second. Then arm in
**Stabilized**, lift to ~2 m, and check that a small roll-right input drops the right side. Only
then switch to **Position** and hover hands-off to confirm EKF position hold.

---

### 2.3 Quadplane (VTOL)

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 \
PX4_HOME_LON=175.7433944 \
PX4_HOME_ALT=48.0 \
PX4_HOME_YAW=235 \
make px4_sitl_nolockstep flightaxis_quadplane
```

Airframe: `1202_flightaxis_quadplane` (`SYS_AUTOSTART` 1202, `CA_AIRFRAME 2`, standard VTOL,
`VT_TYPE 2`). Channel map: `models/quadplane.json`. Options: `ResetPosition` + `SilenceFPS` (bitmask **9** = 1 | 8).

**RealFlight aircraft:** a quadplane with four lift motors, a pusher and four control surfaces — aileron / elevator / forward throttle /
rudder on channels 1–4, four lift motors on channels 5–8.

| RF ch | PX4 | Drives | `PWM_MAIN_FUNC` | Scale | Disarm |
|---|---|---|---|---|---|
| 0 | `controls[0]` | aileron (PX4 **aileron left**) | 1 = 201 Servo 1 | bipolar | 0.5 |
| 1 | `controls[1]` | elevator | 2 = 203 Servo 3 | bipolar | 0.5 |
| 2 | `controls[2]` | forward / pusher throttle (Motor 5) | 3 = 105 Motor 5 | unipolar | **0.0** |
| 3 | `controls[3]` | rudder | 4 = 204 Servo 4 | bipolar | 0.5 |
| 4 | `controls[4]` | lift motor 1 | 5 = 101 Motor 1 | unipolar | **0.0** |
| 5 | `controls[5]` | lift motor 2 | 6 = 102 Motor 2 | unipolar | **0.0** |
| 6 | `controls[6]` | lift motor 3 | 7 = 103 Motor 3 | unipolar | **0.0** |
| 7 | `controls[7]` | lift motor 4 | 8 = 104 Motor 4 | unipolar | **0.0** |
| 8 | `controls[8]` | aileron **right** — see below | 9 = 202 Servo 2 | bipolar | 0.5 |
| 9–11 | `controls[9..11]` | *unassigned* — steady 0.5 | 10–12 unset | bipolar | 0.5 |

This is the clearest illustration of the `scale` rule: rf2 and rf4–7 are motors (`unipolar`,
disarm `0.0`) while rf0, rf1, rf3 and rf8 are surfaces (`bipolar`).

⚠️ **RF ch9 now carries the right aileron.** As on the plane, the two allocator outputs are exact
mirrors and `controls[0]` already swings the full ±1 range, so a model with one mixed aileron
channel needs nothing there. If your RF model uses ch9 for something else, `param set
PWM_MAIN_FUNC9 0` returns it to a steady neutral 0.5; if it really is split-aileron and the
surfaces move the same way, reverse it with `param set PWM_MAIN_REV 256` (bit 8).

This is the clearest case of the FUNC list absorbing the allocator's ordering. PX4's allocator
always emits **motors before servos**, so its own numbering puts the four lift motors first
(`101..104`) even though the RealFlight model carries them on channels 5–8. The FUNC assignment
undoes that in one place, and `quadplane.json` stays a plain identity map rf0→0 … rf7→7.

**Gotcha — do NOT enable `Rev4Servos`.** That option swaps RF channels 0–3 with 4–7 wholesale.
The airframe already places the control surfaces on `controls[0..3]` and the lift motors on
`controls[4..7]`, which is the order the RealFlight model wants, so enabling `Rev4Servos` would
swap them back — driving the control surfaces with lift-motor throttles and the lift motors with
surface deflections. It exists for RF models built the other way round.

**No shipped model uses `Rev4Servos`, and that is deliberate — it is not an oversight and no
demonstration model is coming.** The option exists for *user-authored* models only. All four
shipped models (`plane`, `quad`, `quadplane`, `heli`) are identity maps whose channel order is
already correct, having been set in the airframe's FUNC params, and for any such model
`Rev4Servos` is strictly harmful: it can only swap a correct order into a wrong one. The option
pays off exactly when a RealFlight model's channel order is a wholesale 0–3 ↔ 4–7 rotation of what
the airframe emits *and* you would rather flip one flag than renumber eight FUNC params. Shipping a model
whose only purpose was to exercise the flag would mean shipping a model deliberately authored
against the house style, immediately next to a `quadplane.json` that carries an explicit
`OptionsComment` warning readers off the same flag. If you enable it, note the validator
requirement: `get_FAbridge_params.py` rejects any `Rev4Servos`-covered row (rf0–7) left on
`"disarm": -1` / hold-last, because the swap rewrites the slot every frame and a held channel would
ping-pong between `rf i` and `rf i+4` at the loop rate — a full-amplitude servo buzz at ~250 Hz.
Give those rows an explicit `disarm` (0.5 for a bipolar servo, 0.0 for a motor).

**Gotcha — transition.** The airframe sets `VT_F_TRANS_THR 0.75`, `VT_FWD_THRUST_EN 4`,
`FW_AIRSPD_MAX 25`. Transition needs the pusher to actually accelerate the model. These values
assume an airframe whose pusher can reach `VT_ARSP_TRANS` in level flight within
`VT_F_TRANS_DUR`; if your RealFlight model is appreciably draggier, or light enough that the
lift motors hold it below that airspeed, transition will stall or never complete. Tune
`VT_F_TRANS_DUR` / `VT_ARSP_TRANS` for your model rather than assuming the defaults fit.

**Gotcha — there is no motor-failure detection.** The airframe used to carry `FD_ACT_EN 0`,
inherited from a stock PX4 standard-VTOL airframe along with a comment claiming SITL reports
0 A per ESC. It does not: `SimulatorMavlink` publishes `esc_current = 1 + |control| * 15` while
armed. The undercurrent test is `esc_current < esc_throttle * FD_ACT_MOT_C2T` with
`FD_ACT_MOT_C2T` defaulting to 2.0, and `1 + 15c < 2c` has no solution for any throttle — so
the detector cannot fire in v1.16 SITL at all, for any vehicle. The `FD_ACT_*` overrides were
therefore suppressing nothing and have been **removed entirely**; the parameters sit at their
firmware defaults so that detection starts working here if upstream ever fixes the SITL ESC
current model. The corollary is worth stating plainly: you get no motor-failure detection in
SITL, and a lift motor that quietly dies in cruise will not be reported.

**QGC check:** Actuators shows five motors then four servos. Motors 1–4 must move the RealFlight
lift rotors, Motor 5 the pusher.

**First flight.** Confirm on the ground that Motors 1–4 drive the *lift* rotors and Motor 5 the
pusher — if they are swapped you have the `Rev4Servos` double-swap described above. Then hover in
**Stabilized** on the lift motors alone and confirm multirotor control is sane *before*
attempting any transition. Transition last, with plenty of altitude: it is the part most
sensitive to your particular RealFlight model (see the transition gotcha above), and a failed
forward transition drops the aircraft.

---

### 2.4 Helicopter

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 \
PX4_HOME_LON=175.7433944 \
PX4_HOME_ALT=48.0 \
PX4_HOME_YAW=235 \
make px4_sitl_nolockstep flightaxis_heli
```

Airframe: `1203_flightaxis_heli` (`SYS_AUTOSTART` 1203, `CA_AIRFRAME 11` = "Helicopter
(tail Servo)"). Channel map: `models/heli.json`. Options: `ResetPosition` + `SilenceFPS`
(bitmask **9** = 1 | 8) — the same set the other three airframes use. **`HeliDemix` is
deliberately not enabled**; see "Who does the swash mixing" below.

**RealFlight aircraft:** a collective-pitch heli wired **direct-servo, with its own CCPM
mixing switched off** — three swash servos on channels 1–3, tail on 4, rotor speed
controller on channel 8, each channel straight through to the matching servo. This is
**ArduPilot's own heli channel order and its default (non-demixed) convention**, so a
model already set up for an ArduPilot pilot needs no re-mapping.

| RF ch | PX4 | Drives | `PWM_MAIN_FUNC` | Scale | Disarm |
|---|---|---|---|---|---|
| 0 | `controls[0]` | swash plate servo 1 | 1 = 202 Servo 2 | bipolar | **0.5** |
| 1 | `controls[1]` | swash plate servo 2 | 2 = 203 Servo 3 | bipolar | **0.5** |
| 2 | `controls[2]` | swash plate servo 3 | 3 = 204 Servo 4 | bipolar | **0.5** |
| 3 | `controls[3]` | tail rotor pitch / yaw (Servo 1) | 4 = 201 Servo 1 | bipolar | **0.5** |
| 4–6 | `controls[4..6]` | *unassigned* — steady 0.5 | 5–7 explicitly `0` | bipolar | 0.5 |
| 7 | `controls[7]` | main rotor / RSC (Motor 1) | 8 = 101 Motor 1 | unipolar | **0.0** |
| 8–11 | `controls[8..11]` | *unassigned* — steady 0.5 | 9–12 unset | bipolar | 0.5 |

RF ch5–7 stay idle to keep ArduPilot parity (below); they are mapped rows now, but with their
FUNC at `0` they emit a steady neutral 0.5 exactly as before.

That layout is taken from ArduPilot: `AP_MotorsHeli_Swash.cpp:201-202` defaults the three
swash servos to `SERVO1/2/3` (`k_motor1/2/3` = 33/34/35), `AP_MotorsHeli_Single.cpp:207`
(`add_motor_num(CH_4)`) puts the tail rotor on `SERVO4` (`k_motor4` = 36), and
`AP_MotorsHeli.h:34` (`#define AP_MOTORS_HELI_RSC CH_8`) puts the rotor speed controller on
`SERVO8` (`k_heli_rsc` = 31). `SIM_FlightAxis.cpp` ships output channel N as RealFlight
channel N unpermuted, so ArduPilot's RealFlight heli is swash on 1–3, tail on 4, throttle on
8, with 5–7 idle — which is exactly the table above.

Airframe side: `PWM_MAIN_FUNC1..4 = 202, 203, 204, 201` and `PWM_MAIN_FUNC8 = 101`
(`FUNC5..7` explicitly 0) →
`controls[0..2]`=Servos 2–4, the swash plate servos 1–3, `[3]`=Servo 1 yaw tail,
`[7]`=Motor 1 main rotor. Note that the allocator's own numbering under `CA_AIRFRAME 11` is
unchanged by any of this — it is a property of the airframe type, and it still puts the main
rotor first as Motor 1 = 101, then the yaw tail as Servo 1 = 201, then the swash as
Servos 2–4 = 202–204. The FUNC list is what reorders those onto RealFlight's channels.

**Every row here carries an explicit `disarm`** — as every row of all four shipped models does,
and `get_FAbridge_params.py` refuses to flatten the file if one is missing. The reason for
stating them *here* is particular to the heli. With the demix off these rows are plain pass-through and nothing
rewrites them after the fact, so holding would be safe on that ground alone — but the values
stay stated, so that re-enabling `HeliDemix` for a CCPM model cannot quietly reintroduce the
hazard below. The bridge also demixes out of a scratch copy and leaves the
persistent channel array holding raw servo values, so a `-1` "hold last output" row would in
fact hold correctly even then. It did not always. When the demix was applied in place, a held row
had its already-demixed value fed back through the demix on the next frame, and that
iteration diverges — from a plausible in-flight swash it railed within about three frames.
Neutral is a fixed point, so it only bit on the armed→disarmed transition with the swash
deflected, which is to say on every landing, and nothing was logged when it did. The rule
stays enforced because the property that makes holding safe lives in one function in the
bridge, and nothing in a model JSON can see whether it still holds.

#### Who does the swash mixing

A collective-pitch heli needs exactly **one** CCPM mix between roll/pitch/collective and the
three swash servos. It can live in PX4's allocator, in the bridge, or in the RealFlight
model — but in exactly one of them. Two mixes in series do not cancel, they compose, and roll
and pitch cross-couple badly.

**This integration puts it in PX4.** The allocator (`CA_AIRFRAME 11` plus the `CA_SP0_*`
geometry) produces three swash **servo positions**, and the bridge sends those three numbers
to RealFlight untouched — `heli.json` is a pure identity map with `HeliDemix` *not* in its
options. RealFlight channel N carries PX4 servo N, literally, exactly as the plane airframe
puts aileron/elevator/throttle/rudder on channels 1–4.

So **the RealFlight model must be non-mixed / direct-servo**: turn its CCPM off in the
aircraft editor and wire each channel straight to its servo. This also matches ArduPilot's
default — `SIM_FlightAxis.cpp:129-130` only enables ArduPilot's own demix when the frame name
contains the substring `helidemix`, so a plain ArduPilot RealFlight heli likewise ships raw
swash positions.

What RealFlight receives is therefore:

| RF ch | RealFlight sees |
|---|---|
| 0 | swash servo 1 (at `CA_SP0_ANG0` = 300°) |
| 1 | swash servo 2 (at `CA_SP0_ANG1` = 60°) |
| 2 | swash servo 3 (at `CA_SP0_ANG2` = 180°) |
| 3 | tail rotor pitch / yaw |
| 7 | main rotor throttle / RSC |

Note there is no "roll channel" any more — every cyclic input is spread across all three
swash channels by the mix PX4 performs.

#### When to re-enable `HeliDemix`

**Only if your RealFlight model has a CCPM/eCCPM head whose mixing cannot be disabled.** Such
a model wants roll/pitch/collective on channels 1–3, not servo positions, so the bridge has to
undo PX4's swash mix first. Add `"HeliDemix"` back to `Options` in `heli.json` (bitmask 9 → 13)
and change nothing else. **Never enable both** the model's mixing and the bridge's demix — and
never enable the demix against a model you have already wired direct-servo.

The option remains fully supported in the code; it is simply not the right default for a
directly-wired model. **It is also a dead path by default**: with the demix off, the demix
arithmetic below is not exercised at all, and the `CA_SP0_ANG` 300/60/180 geometry is a claim
about the model's head rather than an input to the demix inverse. Treat re-enabling `HeliDemix`
as bring-up on a new model, and work through §9 again afterwards.

When enabled it rewrites RF channels 0–2 as:

```
roll  = (s1 - s2) / 1.732
pitch = ((s1 + s2)/2 - s3) / 1.5
col   = (s1 + s2 + s3)/3      (recentred to 0..1)
```

giving `rf0` = roll, `rf1` = pitch, `rf2` = collective in place of the servo positions above.

**The two divisors are ours, not ArduPilot's.** `SIM_FlightAxis.cpp:348-350` divides by
nothing at all — it is a plain unweighted inverse, exact only for a 140° head
(`AP_MotorsHeli_Swash.cpp:138-145` mixes `H3_140` with flat ±1.0 factors). On the 120° head
both ArduPilot and this airframe actually use — `AP_MotorsHeli_Swash.cpp:147-154` defaults
`add_servo_angle` to −60/+60/180, the same geometry as our `CA_SP0_ANG` 300/60/180 — the
forward mixer carries cos 30° = 0.866 on roll against 0.5/1.0 on pitch, so ArduPilot's own
round trip returns roll and pitch gains in the ratio 2/√3 = 1.155 rather than 1. Matching
ArduPilot here would reintroduce that imbalance, so the channel **order** follows ArduPilot
and the demix **gains** deliberately do not.

The divisors are gain normalisation, not part of the geometric inverse. With the swash
angles the airframe pins (below) and the 0.5 per-servo scale, the raw differences come out at
gain 0.866 for roll and 0.750 for pitch, where RealFlight wants 0.5 about its 0.5 centre;
dividing by 0.866/0.5 = √3 and 0.750/0.5 = 1.5 lands them exactly. Without this the cyclic
saturates at ±0.577 of commanded roll and ±0.667 of pitch, with roll noticeably hotter than
pitch. Collective is already at gain 0.5 and is left alone.

**Gotcha — the swashplate angles now describe your model's actual head.** With PX4 as the
only mixer, `CA_SP0_ANG*` is a claim about where the servos physically sit on the RealFlight
model's swash, because the number PX4 computes for servo *i* is delivered to that servo
unmodified. The airframe sets **`CA_SP0_ANG0/1/2 = 300/60/180`** — a standard 120° head with
the **odd servo aft** — because that is the geometry ArduPilot's heli defaults to
(`AP_MotorsHeli_Swash.cpp:147-154`, `add_servo_angle` −60/+60/180). PX4's firmware defaults
(0/140/220) describe a different head, odd servo *forward*. If your model's third servo is at
the front, use 120/240/0 instead. Getting this wrong does not give a subtle trim error: it
rotates the cyclic response, so the aircraft banks when commanded to pitch. Verify it against
the model in RealFlight's aircraft editor.

`CA_SP0_ARM_L0/1/2` are pinned to 1.0 (equal arms, right for a symmetric head) and
`CA_MAX_SVO_THROW` to 0 (no `asin()` linearisation — that correction is only right if it
matches the linkage the model actually simulates, and a mismatched one is worse than none).
Both previously *had* to hold for the demix inverse to be exact; that constraint is gone, but
the values remain the sane defaults.

**Gotcha — the tail is a servo, so it is bipolar.** Under `CA_AIRFRAME 11` the yaw tail is a
*servo* on `[-1,1]`, and 0.5 on the wire is zero tail pitch — which is what a collective-pitch
tail rotor wants, and what nearly every stock RealFlight single-rotor heli models. Under
`CA_AIRFRAME 10` ("tail ESC") the tail would be a motor clamped to `[0,1]`: the entire negative
half of the yaw command is clipped, the tail idles on its lower stop, and there is **no left
yaw authority at all**. If you genuinely fly a model with a separate unidirectional electric
tail motor, switch to `CA_AIRFRAME 10` and renumber `PWM_MAIN_FUNC1..4` to `201/202/203/102`
(`FUNC5` is already `101`). Under type 10 the tail becomes Motor 2 = 102 and the swash servos
shift down one index to Servo 1–3 = 201–203, because the tail no longer consumes Servo 1 — the
FUNC list absorbs that shift, which is exactly what it is there for. Also change `rf3` back to
`"unipolar"` with `"disarm": 0.0`, since the tail is then a `[0,1]` motor. Nothing else in the
JSON moves: it stays the identity map, and the `controls[]` indices are unchanged.

**Gotcha — tuning.** The airframe ships real rate gains, typical of collective-pitch helicopters
rather than derived from this airframe: roll/pitch `P 0.025 I 0.15 D 0.001 FF 0.15`, yaw `P 0.18 I 0.12 D 0.003
FF 0.02`. They are feed-forward dominant on purpose — a swash gives a large, fast, near-linear
torque response, so a large P term only chases rotor phase lag. They are **a starting point, not
a tune for your model** — rotor head, blade and inertia differences between RealFlight
helicopters are large enough to matter. Refine it in Acro. The airframe
also sets an explicit collective curve (`CA_HELI_PITCH_C*`, spanning RealFlight 0.45–0.85 so
hover sits near mid-stick rather than near the top of a quarter of the travel) and yaw
compensation for main-rotor torque (`CA_HELI_YAW_CP_S 0.25`); the blade angle a given channel
value actually produces is a property of the RealFlight model, so verify and rescale.

**Gotcha — the RealFlight model needs work.** More than the other three vehicles. RC heli
models ship with stabilisation and mixing inside the model, and all of it fights PX4: the
tail heading-hold gyro must be zeroed (it is an independent yaw controller and will produce a
slow oscillation no `MC_YAWRATE_*` value fixes), CCPM/eCCPM swash mixing must be off because
PX4 has already done the CCPM mix and what arrives is three finished servo positions — wire
each channel straight to its servo, one-to-one — and the model's own collective and throttle
curves must be flattened because PX4 owns both. Leave a governor on, if the model has one — PX4
holds the throttle channel at a constant 100 % and expects constant head speed.

**QGC check:** Actuators shows Motor 1, then four servos — yaw tail first, then the three swash
servos — and the swashplate angles read 300/60/180.

**First flight.** Check the swashplate on the ground first: cyclic right must tilt the RealFlight
swash right with **no** pitch component, and cyclic forward must tilt it forward with no roll
component. Cross-coupling here has two likely causes, and both must be ruled out before
spooling up: either **the RealFlight model is still doing its own CCPM mixing** (its mix
composing with PX4's — turn the model's mixing off), or **`CA_SP0_ANG*` does not match the
model's actual head** (e.g. the odd servo is at the front, wanting 120/240/0 rather than the
300/60/180 the airframe sets). Check yaw in both directions while you
are there: full left and full right must move the tail servo symmetrically about its centre. If
left yaw does nothing, the airframe is on `CA_AIRFRAME 10` or `rf3` is unipolar. Then bring the
rotor to speed, raise collective to a light skid, and confirm yaw holds before lifting off.
Expect to refine the rate gains and the collective curve for your model.

---

## 3. Home position and heading

Where the aircraft sits on the map, and which way it faces, is set by four environment variables:

| Variable | Default | Meaning |
|---|---|---|
| `PX4_HOME_LAT` | `47.397742` | degrees |
| `PX4_HOME_LON` | `8.545594` | degrees |
| `PX4_HOME_ALT` | `488.0` | metres AMSL |
| `PX4_HOME_YAW` | unset | degrees true — start heading; unset leaves RealFlight's world unrotated |

(Position defaults are PX4's usual Zurich origin, read in `flightaxis_bridge.cpp` via
`envOrDefault()`.)

All four are read by the bridge, not by PX4. `PX4_HOME_LAT/LON/ALT` are also read by upstream
PX4, but only in the gazebo-classic plugins and `px4-rc.sihsim` — neither of which is in this
path — and upstream has no `PX4_HOME_YAW` at all. Setting any of them affects this integration
only because the bridge consults them.

Full copy-pasteable command with a worked set of coordinates:

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 \
PX4_HOME_LON=175.7433944 \
PX4_HOME_ALT=48.0 \
PX4_HOME_YAW=205 \
make px4_sitl_nolockstep flightaxis_plane
```

**Heading.** `PX4_HOME_YAW` is the true heading the aircraft should **start** on. RealFlight's
world north is arbitrary, so the bridge reads the model's actual attitude on the first frame and
rotates the whole RF→NED mapping by the difference — the same kind of re-anchoring `PX4_HOME_ALT`
does for altitude, and equally invisible to RealFlight. The rotation it derives is logged at
startup:

```
[flightaxis_bridge] heading datum: RealFlight reports 12.3 deg,
PX4_HOME_YAW asks for 205 deg -> rotating the RF world by 192.7 deg
```

The rotation is applied at the RF→NED ingest boundary to *every* world-frame quantity — attitude,
position, velocity and wind — so heading and direction of travel stay consistent and EKF2 sees no
contradiction. Body-frame sensors (accelerometer, gyro) and all Down components are untouched.
GPS, baro, mag and COG follow for free, each being derived from something already rotated.

The heading datum is latched alongside the position anchor and dropped with it, so a reset in
RealFlight re-derives it against the post-reset attitude and your requested heading survives.

Leaving it unset rotates nothing — that is the historical behaviour, and also what ArduPilot does:
it accepts a heading as the 4th field of `--custom-location`, but `SIM_FlightAxis` overwrites the
attitude from RealFlight on the first frame, so the value has no effect there. Alternatively, just
point the model where you want it in RealFlight; the two approaches are interchangeable.

**How home anchors the world.** The bridge captures a `position_offset` from the first RealFlight
frame (and re-captures it after every reset), subtracts it from the raw RealFlight position, and
then converts the resulting local NED to lat/lon around `PX4_HOME_*` on a spherical earth. So:

- The RealFlight runway becomes exactly `(PX4_HOME_LAT, PX4_HOME_LON)` on the QGC map, whatever
  the scenery's real-world location is.
- Altitude is derived as `PX4_HOME_ALT - position_ned.z` for **both** baro and GPS, deliberately —
  RealFlight's own scenery ASL is arbitrary, and using it would leave baro and GPS altitude
  disagreeing by a constant forever.
- `PX4_HOME_*` also seeds the magnetic field model (declination/inclination/strength), so setting
  it to something far from where you intend to fly gives you a wrong compass.

---

## 4. MAVLink endpoints

### What you get for free

PX4 SITL already starts a GCS link in `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`:

```sh
mavlink start -x -u $udp_gcs_port_local -r 4000000 -f
```

with `udp_gcs_port_local = 18570 + px4_instance`. `-o` is not given, so it defaults to remote
port **14550** on **127.0.0.1**. That is the link QGC picks up. Two more instances start by
default: the offboard/API link (local 14580, remote 14540 — this is the one MAVSDK uses) and
onboard camera/gimbal links.

Note `-f` here is *message forwarding between MAVLink instances*, not "broadcast" — broadcast is
`-p`. Because `-f` is set on the GCS link, traffic is forwarded across instances.

### Adding more endpoints

There is no PX4 command-line flag for this. Do it at the `pxh>` prompt after boot:

```
pxh> mavlink start -x -u 18571 -r 4000000 -o 14551 -t 127.0.0.1
pxh> mavlink start -x -u 18572 -r 4000000 -o 14552 -t 127.0.0.1
```

Flags that matter (`mavlink help` at the `pxh>` prompt prints the full list):

| Flag | Meaning |
|---|---|
| `-u <port>` | local UDP port to bind — **must be unique per instance** |
| `-o <port>` | remote UDP port to send to (default 14550) |
| `-t <ip>` | partner IP (default 127.0.0.1) |
| `-r <B/s>` | max send rate; SITL uses 4000000 |
| `-p` | enable broadcast |
| `-m <mode>` | stream set: `normal`, `onboard`, `gimbal`, `minimal`, … |
| `-x` | enable FTP (needed for QGC parameter/mission download) |

Verify with:

```
pxh> mavlink status
```

**Worked example — three endpoints.** 14550 already exists, so add two:

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=-37.7304917 PX4_HOME_LON=175.7433944 PX4_HOME_ALT=48.0 PX4_HOME_YAW=205 \
make px4_sitl_nolockstep flightaxis_plane
```

then at the prompt:

```
pxh> mavlink start -x -u 18571 -r 4000000 -o 14551 -t 127.0.0.1
pxh> mavlink start -x -u 18572 -r 4000000 -o 14552 -t 127.0.0.1
pxh> mavlink status
```

To make that permanent, append the same two lines to
`ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` (use `$((18571+px4_instance))` style if you
care about instances) and rebuild. That file is PX4-owned, not part of this repo — expect to
re-apply the edit after a PX4 update.

To reach a GCS on another machine, give `-t` that machine's IP:

```
pxh> mavlink start -x -u 18573 -r 4000000 -o 14550 -t 192.168.10.50
```

### Connecting clients

- **QGroundControl** — just start it. It listens on UDP 14550 and auto-connects. If it does not,
  add a Comm Link: type UDP, port 14550.
- **MAVSDK** — `udpin://0.0.0.0:14540` (the default offboard link already targets 14540; nothing
  extra needed).
- **pymavlink** — point it at one of the extra endpoints so it does not fight QGC for 14550:

  ```python
  from pymavlink import mavutil
  m = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
  m.wait_heartbeat()
  print(m.target_system, m.target_component)
  ```

- **mavlink-router / mavproxy** — same idea: consume one dedicated endpoint and fan out from
  there, rather than adding many PX4 instances.

---

## 5. What a healthy run looks like

Annotated, in order:

```
FlightAxis setup
FlightAxis check: RealFlight reachable at 192.168.10.1:18083
```
↑ `FA_check.py` passed. If this fails, `make` stops here and nothing else starts.

```
[flightaxis_bridge] MAVLink to FlightAxis (RealFlight) bridge
[flightaxis_bridge] options=0x9 unmapped_default=0.5 channels=12
  rf0 <- px4[0] bipolar disarm=0.5
  rf1 <- px4[1] bipolar disarm=0.5
  rf2 <- px4[2] unipolar disarm=0
  rf3 <- px4[3] bipolar disarm=0.5
  rf4 <- px4[4] bipolar disarm=0.5
  rf5 <- px4[5] bipolar disarm=0.5
  rf6 <- px4[6] bipolar disarm=0.5
  rf7 <- px4[7] bipolar disarm=0.5
  rf8 <- px4[8] bipolar disarm=0.5
  rf9 <- px4[9] bipolar disarm=0.5
  rf10 <- px4[10] bipolar disarm=0.5
  rf11 <- px4[11] bipolar disarm=0.5
```
↑ `channels=12` on every shipped model, and **no row says `reversed`** — that is the identity
pipe. A `reversed` here would mean someone added `"reverse": true` back to the JSON.
↑ The channel map the bridge actually parsed. **Check this against §2 for your vehicle** — it is
the fastest way to catch an edited JSON. Every shipped model is an identity map, so each row
should read `rf<N> <- px4[N]`; anything else means the JSON has been edited (§7).
No shipped model prints `disarm=-1` (hold last output) on any row — every row states a number.
`options=0x9` is `ResetPosition|SilenceFPS`.

```
[flightaxis_bridge] waiting for PX4 on TCP 4560 ...
[flightaxis_bridge] waiting for PX4 on TCP 4560 ... connected
PX4 Communicator: PX4 Connected.
```
↑ The bridge listens first and blocks in `accept()`; the `px4` binary launched by the same script
connects a moment later. A hang between these two lines means PX4 never got that far (§7).

```
INFO  [init] found model autostart file as SYS_AUTOSTART=1200
INFO  [simulator_mavlink] Simulator connected on TCP port 4560.
```
↑ PX4's side of the same connection, and confirmation the right airframe was selected.

```
[flightaxis_bridge] battery telemetry -> UDP 127.0.0.1:14580 as sysid 1
[flightaxis_bridge]   (airframe must set SIM_BAT_ENABLE 0 so battery_simulator does not publish over it)
```
↑ The bridge feeds PX4's battery from RealFlight's own electrical or fuel model, over its own UDP
link to PX4's API/offboard MAVLink port. Every shipped airframe sets `param set SIM_BAT_ENABLE 0`
for exactly this reason, so that there is one publisher on `battery_status` and not two. On a HITL
transport the line reads `battery telemetry disabled on <transport>` instead — see `HITL.md`.

```
[flightaxis_bridge] connecting to RealFlight at 192.168.10.1:18083
[flightaxis_bridge] controller injected, aircraft reset
```
↑ The UAV controller interface was injected into RealFlight and (because `ResetPosition` is set)
the aircraft was reset to the runway. RealFlight's own controls are now overridden by PX4.

```
[flightaxis_bridge] heading datum: RealFlight reports 12.4 deg, PX4_HOME_YAW asks for 235 deg -> rotating the RF world by 222.6 deg
```
↑ Printed once, when the heading datum is captured. Only appears if you set `PX4_HOME_YAW`; the
QGC check below sends you looking for it. See [Home position and
heading](#3-home-position-and-heading).

```
[flightaxis_bridge] battery source: electric (RealFlight pack voltage/current)
```
↑ Printed once, and again whenever the source changes. The three forms are `electric …`,
`fuel / internal combustion …` and `synthetic (no pack and no tank seen yet; nominal full)` —
the last means RealFlight has shown neither a pack nor a tank yet, so the bridge is sending a
nominal full battery so preflight can pass. It is normal for the first frames.

```
[flightaxis_bridge] first HIL_ACTUATOR_CONTROLS received, enabling channels
```
↑ PX4 has produced outputs; the bridge switches RealFlight's `selectedChannels` from 0 to all 12.
Before this the model deliberately sees nothing.

```
[flightaxis_bridge] exchanges=248.3/s loop=612.7/s avg=249.1 FPS rtf=1.00 glitches=0
```
↑ Printed every 1000 bridge frames. **The shipped models set `SilenceFPS`, so you will not
see this line** — once a second, forever, it buries everything else in the terminal. You lose
only the heartbeat, not the alarms: each swallowed glitch still prints its own `glitch 0.62s`
line and an out-of-range realtime factor still warns. To watch the rates live, drop
`"SilenceFPS"` from `Options` in `models/<name>.json`.

Read it as:

| Field | Meaning |
|---|---|
| `exchanges=` | SOAP round-trips completed per second of physics time |
| `loop=` | bridge main-loop iterations per second — informational only; the loop free-runs and extrapolates between real frames, so this is normally *higher* than the exchange rate |
| `avg=` | the true physics frame rate, `1/average_frame_time`. Want ~200–300. |
| `rtf=` | **realtime factor** — physics seconds per wall-clock second over the reporting window. **This is the one to watch.** Want 1.00. |
| `glitches=` | cumulative count of >50 ms physics-time discontinuities swallowed. **Should stay at 0 or near it.** A steadily rising count means the network (usually WiFi). |

`rtf` deserves the attention because nothing in the sensor data can reveal what it catches. PX4
timestamps the SITL sensor stream with its *own* clock on arrival, not with the physics time in
the message, so if RealFlight's physics falls to 0.8× the aircraft flies in slow motion while
every sensor value still looks perfectly correct — and velocities, accelerations and every
control loop are scaled wrong. The glitch compensation actively hides this, because it smooths
the bridge's exported clock rather than the arrival rate. The bridge warns (at most once every
5 s) when `rtf` leaves 0.95–1.05. Note this is a different fault from the physics speed
multiplier warning below it: that one is RealFlight's self-reported *setting*, while `rtf`
catches RealFlight reporting 1.0 and the machine simply not keeping up.

Occasional single lines are normal and benign:

```
[flightaxis_bridge] glitch 0.11s
```

### Before you fly, check in QGC

1. **Attitude mirrors RealFlight.** Roll the model in RealFlight; the QGC artificial horizon must
   follow, same direction, no lag.
2. **Position on the map** sits at your `PX4_HOME_LAT/LON`, and the vehicle icon moves the right
   way when you taxi.
3. **Heading matches** what you asked for. If you set `PX4_HOME_YAW`, the QGC compass should read
   it on the first frame — check the `heading datum:` line in the bridge output for the rotation
   it derived. Then taxi: the compass and the direction the icon travels must agree. If they
   disagree, that is a frame problem, not a tuning one.
4. **EKF healthy** — no "Preflight fail: EKF" messages; attitude, velocity and position estimates
   all valid. Give it 20–30 s after start.
5. **Actuators move.** Vehicle Setup → Actuators, enable sliders, and confirm each PX4 actuator
   moves the *expected* RealFlight surface/rotor in the *expected* direction (§2 tables).
6. **Altitude sane** — baro and GPS altitude both around `PX4_HOME_ALT`, and agreeing with each
   other (they are derived from the same datum by design).
7. **No physics-speed warning** in the console.

---

## 6. Multiple instances

Multi-instance is wired through the make targets. Set **`PX4_FLIGHTAXIS_INSTANCE`**:

```bash
# instance 1: bridge on TCP 4561, PX4 on 4561, MAVLink ports +1, MAV_SYS_ID 2
PX4_FLIGHTAXIS_INSTANCE=1 PX4_FLIGHTAXIS_IP=192.168.10.2 \
  make px4_sitl_nolockstep flightaxis_plane
```

`sitl_run.sh` threads it to both ends — the bridge takes it as argv[1] (`px4_communicator.cpp`
binds `portBase + portOffset`, i.e. `4560 + instance`) and `px4` is launched with `-i $instance`.
That `-i` is the load-bearing part: it is what sets `px4_instance` inside `rcS`, which is what
offsets the simulator TCP port (`px4-rc.mavlinksim`), every MAVLink UDP port (`px4-rc.mavlink`) and
`MAV_SYS_ID` (`px4_instance + 1`). Passing the instance to the bridge alone would leave PX4
listening on 4560 regardless.

Working directories: instance 0 keeps `build/px4_sitl_nolockstep/rootfs` so existing saved
parameters and logs stay put; instance *N* > 0 uses `build/px4_sitl_nolockstep/instance_N`,
following PX4's own `Tools/simulation/sitl_multiple_run.sh`.

`PX4_FLIGHTAXIS_ROOTFS` overrides both, and it is the right tool when you want to separate
**models** rather than instances — see
[*A working directory per model*](README.md#a-working-directory-per-model) in README.md for
that. A directory with no `parameters.bson` is seeded from
`build/px4_sitl_nolockstep/rootfs`, so a new one starts with the calibration you already
have; §7 has what happens without it.

The two are not interchangeable. `-i` is wired to `MAV_SYS_ID`, the simulator TCP port and
every MAVLink UDP port, so using it to mean "model" burns a port range per airframe and
misreports system IDs. `PX4_FLIGHTAXIS_ROOTFS` touches none of those — it moves only the
working directory, so it composes with `PX4_FLIGHTAXIS_INSTANCE` rather than competing with
it. Set both and you get instance *N*'s ports with your chosen directory — but give each
instance a different one, since two concurrent instances pointed at the same directory would
be writing one `parameters.bson` and one dataman between them.

A non-numeric `PX4_FLIGHTAXIS_INSTANCE` is rejected with an error rather than being silently
truncated to 0 — which would otherwise produce a "second" instance that collides with the first.

In practice each instance still needs its own RealFlight host: one RealFlight instance serves one
aircraft.

**Launching by hand.** Still perfectly valid, and what you want if you are debugging the bridge
itself:

```bash
cd ~/PX4-Autopilot/Tools/simulation/flightaxis/flightaxis_bridge

# instance 1 -> listens on TCP 4561
PX4_HOME_LAT=-37.7304917 PX4_HOME_LON=175.7433944 PX4_HOME_ALT=48.0 PX4_HOME_YAW=205 \
~/PX4-Autopilot/build/px4_sitl_nolockstep/build_flightaxis_bridge/flightaxis_bridge \
  1 192.168.10.2 $(./get_FAbridge_params.py models/plane.json) &

# then PX4 instance 1 in its own rootfs
mkdir -p /tmp/px4_i1 && cd /tmp/px4_i1
PX4_SIM_MODEL=flightaxis_plane \
~/PX4-Autopilot/build/px4_sitl_nolockstep/bin/px4 -i 1 \
  ~/PX4-Autopilot/build/px4_sitl_nolockstep/etc
```

Note `PX4_HOME_*` must be set on the **bridge** process, not on `px4`.

---

## 7. Troubleshooting

Keyed off the strings the software actually prints.

### `RealFlight FlightAxis not reachable at 192.168.10.1:18083`

From `sitl_run.sh` after `FA_check.py` failed; nothing else was started. In order:
RealFlight running? RealFlight Link ticked (§1.1)? Right IP (`ipconfig`)? Firewall (§1.3)?
Wired link up? Confirm independently with `nc -zv <ip> 18083`.

### `get_FAbridge_params.py failed for models/<m>.json`

The JSON is malformed, missing, or uses a name the flattener does not know. It prints the
specific cause on stderr:

- `get_FAbridge_params: unknown option '<x>'` — only `ResetPosition`, `Rev4Servos`, `HeliDemix`,
  `SilenceFPS` are valid.
- `get_FAbridge_params: unknown scale '<x>'` — only `unipolar` and `bipolar`.

Reproduce it directly:

```bash
cd ~/PX4-Autopilot/Tools/simulation/flightaxis/flightaxis_bridge
./get_FAbridge_params.py models/plane.json
```

Healthy output is one line of numbers — options, unmapped default, channel count, then five
numbers (`rf`, `px4`, `scale`, `reverse`, `disarm`) per channel. For `models/plane.json`:

```
9 0.5 12 0 0 1 0 0.5 1 1 1 0 0.5 2 2 0 0 0 3 3 1 0 0.5 4 4 1 0 0.5 5 5 1 0 0.5 6 6 1 0 0.5 7 7 1 0 0.5 8 8 1 0 0.5 9 9 1 0 0.5 10 10 1 0 0.5 11 11 1 0 0.5
```

A related message, `get_FAbridge_params.py produced no parameters for …`, means it exited 0 but
printed nothing — an empty `Channels` array.

### `bad channel map: RealFlight channel rf2 is mapped twice (entries 1 and 3)`

or `bad channel map: PX4 control index px4[3] is mapped twice …`. The bridge refuses to start on
either, deliberately: a duplicate `rf` would silently last-wins, and a duplicate `px4` is almost
always a typo. Fix the model JSON; the message names both offending entries.

You may also see `bad channel map: expected N args, got M` — the argv from the flattener was
truncated, which usually means the JSON was edited while running.

### `WARNING: RealFlight physics speed multiplier is 2.00 (set it to 1.0)`

RealFlight is running its physics faster or slower than realtime. PX4's controllers and EKF
assume wall-clock. Set the multiplier back to 1.0 in RealFlight. The warning repeats with the
FPS line until you do.

### `glitch 0.62s` / `glitches=` climbing

A physics-time discontinuity larger than 50 ms was detected and swallowed (so PX4 never sees a
time jump — the EKF stays healthy). Isolated glitches are normal. A **rising count is a network
problem**, essentially always WiFi. Move to wired (§1.4). Also check the Windows box is not
swapping, alt-tabbed into something heavy, or throttling RealFlight in the background.

### Silent hang at `waiting for PX4 on TCP 4560 ...`

The bridge is in `accept()` and PX4 never connected. Causes:

- Another SITL or a stale bridge still holds 4560: `ss -ltnp | grep 4560`, then kill it.
- The `px4` binary died during startup — scroll up for its error, e.g.
  `ERROR [init] Unknown model flightaxis_<x>` (airframe not in ROMFS — the airframe file must
  also be listed in `init.d-posix/airframes/CMakeLists.txt`).
- You built the lockstep target. The flightaxis targets only exist under
  `px4_sitl_nolockstep`.
- The bridge was never built, in which case you get `ERROR: bridge binary not found at <path>`
  from `sitl_run.sh` instead of a hang, and it names the fix:
  `ninja -C <build_path> flightaxis_bridge`.

Note the ordering: the bridge waits for PX4 **before** it connects to RealFlight, so a hang here
is never a RealFlight problem.

### `ExchangeData failed, retrying ...`

Lost the SOAP connection mid-run. The bridge retries and prints
`ExchangeData recovered after N retries` when it comes back. Persistent failure = RealFlight was
closed, the aircraft was changed, or the network dropped.

### `PX4 link lost - shutting down`

The PX4 side vanished — the `px4` binary exited or was killed — and the bridge exited with it.
**A dead PX4 link is terminal by design; the bridge does not retry.** Left retrying it would log
thousands of lines a second and keep hammering RealFlight with SOAP requests for a simulation
nobody is consuming. Reconnecting is deliberately not attempted either: re-accepting a fresh PX4
whose clock starts at zero, while the bridge's has not, produces a worse failure than exiting.
Restart both sides; under `make` that means re-running the target. Detection is by `POLLHUP`, a
zero-length TCP read, a fatal `send()` errno, or 100 consecutive send failures.

If you see this immediately at startup, PX4 died during boot — scroll up for its error, and see
the silent-hang entry above.

### `RealFlight controller inactive - reinjecting controller` / `... reset - reinjecting`

RealFlight took control back — someone hit the spacebar (reset), changed aircraft, or changed
scenery. The bridge re-injects the controller every 300 ms until it sticks and re-anchors the
position offset. Expected and self-healing; if it loops forever, RealFlight has a modal dialog
open or the model has no FlightAxis-controllable channels.

### `FlightAxis controller injection failed, retrying ...`

The **startup** injection loop, not the mid-run reinject above. RealFlight answered on 18083 —
so the network is fine and `FA_check.py` passed — but it will not hand the controller over. Two
causes account for nearly all of it: RealFlight has a **modal dialog** open (it will not accept
the controller while one is up), or the loaded model has **no FlightAxis-controllable channels**.
The bridge keeps retrying and pumps HEARTBEAT to PX4 meanwhile, so it will pick up the moment you
dismiss the dialog.

### `connect to <ip>:18083 failed - is RealFlight running with FlightAxis Link enabled?`

The runtime reconnect path, distinct from the pre-launch `FA_check.py` message at the top of this
section: the bridge was already up and the SOAP socket stopped connecting. It repeats as
`connect to <ip>:18083 still failing (N attempts)` at most once every 5 s, and prints
`connect to <ip>:18083 recovered after N failed attempts` when it comes back. RealFlight was
closed or restarted, the Windows box slept, or the link dropped. No action needed if it recovers.

### `failed to find key <name>`

The SOAP reply parsed, but a field the bridge expects was not in it. This is the signature of a
**RealFlight version whose FlightAxis reply schema differs** from the one the parse table was
written against. The bridge forces a controller re-init and the caller self-heals, so you may see
it cycle. It is rare and completely opaque from any other symptom; if it repeats steadily, the
RealFlight build is the thing that changed.

### `PX4 Communicator: send to PX4 failed (N consecutive)`

A `send()` to the PX4 MAVLink socket failed, with the errno appended; logged once and then at most
once a second. An isolated one is a transient. A count that climbs means the PX4 side has stopped
draining or is on its way out — 100 consecutive failures is one of the conditions that declares
the link lost, so this usually precedes `PX4 link lost - shutting down` above.

### `*** RealFlight model has LOST COMPONENTS (crash/breakup) ***`

**Read this one carefully: nothing else will tell you.** A wing, a rotor or some other part came
off the RealFlight model. The bridge prints, at the transition:

```
[flightaxis_bridge] *** RealFlight model has LOST COMPONENTS (crash/breakup) ***
[flightaxis_bridge]     PX4 cannot detect this - sensors keep streaming and the aircraft looks healthy from the flight stack's side.
[flightaxis_bridge]     Reset the aircraft in RealFlight (spacebar) to continue.
```

Everything downstream stays nominal after this. PX4 keeps receiving well-formed sensor data and
keeps commanding actuators, into an airframe that no longer exists — so what you observe is a
healthy flight stack fighting physics that make no sense, with no fault reported anywhere else.
There is no recovery in software: **reset the aircraft in RealFlight (spacebar)**, after which you
will see `RealFlight model components restored (aircraft reset)` and, in PX4, the settling
described under [`Disarming denied: not landed`](#disarming-denied-not-landed-after-a-spacebar-reset).

Related lines from the same model-health check:

- `WARNING: RealFlight model has already LOST COMPONENTS at startup - reset the aircraft in
  RealFlight (spacebar) before flying` — the model was already broken when the bridge attached.
  A healthy model at startup prints nothing, so silence here is the expected case.
- `RealFlight model components restored (aircraft reset)` — **benign**, the recovery line.

### `*** RealFlight model is LOCKED - it will not respond to controls ***`

RealFlight has the model locked; it will ignore everything the bridge sends while that lasts.
`RealFlight model unlocked` reports the return. `WARNING: RealFlight model is LOCKED at startup -
it will not respond to controls` is the same condition found already true when the bridge
attached. Reported a quarter-second late: a change is only announced once the flag has held its
new value for 250 ms.

That debounce is why you may instead see:

```
[flightaxis_bridge] NOTE: RealFlight m-isLocked is FLAPPING (4+ changes in 5000 ms) - suppressing per-change lines; this flag is advisory only and no control path uses it
```

**This is benign, and it is a log-suppression notice rather than a fault.** The flag has been
observed alternating several times a second, which used to bury the messages that actually
explained a flight. Nothing in the bridge acts on it — it is logged and nowhere else — so
suppressing the per-change lines deprives no control path of anything.

### `RealFlight engine RUNNING` / `RealFlight engine STOPPED`

**Not a fault either way.** It is normal before takeoff and after a deliberate shutdown, and it is
printed because the failure it covers has no other symptom: an engine that quit on its own —
flooded, out of fuel, or never started because the model needs a manual ignition — presents to
PX4 as an aircraft that simply will not climb, with no clue anywhere else in the system.

If it happens with the aircraft off the ground, the battery link says so more loudly:

```
[flightaxis_bridge] ENGINE STOPPED IN FLIGHT (fuel 12.4% remaining)
```

with `ENGINE RESTARTED IN FLIGHT (fuel …)` for the return. Both are gated on not touching the
ground, so a normal pre-start sit on the runway and the shutdown after landing stay silent.

### `WARNING: could not open the battery telemetry socket; PX4 will have no battery unless SIM_BAT_ENABLE is 1`

The bridge feeds `battery_status` from RealFlight's own electrical or fuel model over a UDP link
to PX4's API/offboard MAVLink port (§5). Every shipped airframe sets `param set SIM_BAT_ENABLE 0`
so that this bridge is the *only* publisher — which means that if the socket does not open,
**PX4 has no battery at all**, not a fallback one.

It is not fatal to the simulation, and the message names the remedy: re-enable PX4's own simulated
pack with

```
param set SIM_BAT_ENABLE 1
```

which gives you a synthetic battery instead of RealFlight's. Usual cause is the UDP port
(`14580 + instance`) already being held by another SITL instance — see §6 if you are running more
than one.

### `Disarming denied: not landed` after a spacebar reset

You crashed, pressed spacebar, the aircraft is sitting on the runway — and PX4 refuses to
disarm. **Nothing is broken and there is nothing to configure.** Wait about 10–20 seconds and
disarm again; in the default configuration PX4 will usually have disarmed itself before you
get there.

What is happening: the reset teleports the aircraft, and the bridge immediately re-anchors to
the new position, so everything it sends is correct from the very next frame — position ~0,
velocity ~0, rangefinder on the ground. What takes time is **EKF2 re-converging after a
teleport**. A reset from 50 m looks to the estimator like an instantaneous 50 m position step;
it rejects the innovation for several seconds, then does a height reset. Until the estimate
catches up, `vehicle_land_detected.landed` and `.maybe_landed` are both false, and
`Commander.cpp` refuses a non-forced disarm unless one of them is true.

Typical timings for a quad reset from 50 m AGL:

| time after reset | what PX4 thinks | explicit `disarm` |
|---|---|---|
| 0–7 s | EKF2 still rejecting the height step (`z` stuck ~25 m) | **denied** |
| ~8 s | height reset lands, `maybe_landed` true | **accepted** |
| ~12 s | `landed` true | accepted |
| ~14 s | PX4 auto-disarms (`COM_DISARM_LAND`, default 2 s after landing) | — |

Reset from a position far from the start point takes longer, because the horizontal estimate
has to converge too — a reset from 500 m out takes nearer 18 s than 8 s.

If you want to disarm *immediately* instead of waiting, do it the way the land detector allows:

- **Be in Stabilized, Acro or Manual, not Altitude or Position.**
  `MulticopterLandDetector::_get_ground_contact_state()` only ANDs in `_in_descend` when
  `_flag_control_climb_rate_enabled` — i.e. in the altitude-controlled modes. In a manual
  thrust mode, ground contact reduces to low throttle alone.
- **Disarm from the transmitter, not from QGroundControl.** `Commander.cpp` permits a disarm
  that is *not* landed when all of: rotary wing, manual thrust mode, `COM_DISARM_MAN` set
  (default 1), **and the request came from the RC** — a stick gesture (throttle low + full left
  yaw), an arm switch, or an arm button. A disarm from the QGC button or from the `commander`
  console arrives with a different calling reason and can never take that path, which is why
  the QGC button appears "stuck" while the transmitter works.

So: throttle down, hold the disarm gesture on the transmitter, in Stabilized. That is the
intended pilot-in-command path and it is not subject to the estimator settling at all.

### `physics time went backwards - RealFlight restart, re-basing`

RealFlight was restarted while the bridge stayed up. Handled: the clock epoch is rebased so PX4
never sees time move backwards. No action needed.

### Low `avg=` FPS

Below ~150 the simulation feels wrong and the EKF gets noisy. Check, in order: wired link;
RealFlight's physics quality setting; the Windows machine's CPU load; whether the bridge binary
was built RelWithDebInfo (it is, via `sitl_targets_flightaxis.cmake` — a hand-built Debug bridge
will not hold rate); and that the bridge is running on or wired-adjacent to the RealFlight
machine, since SOAP round-trip time dominates.

### Surfaces move the wrong way, or the wrong surface moves

Almost always the channel map. Diagnose with the sliders in QGC's Actuators tab (one actuator at
a time), then:

- **Wrong surface moves** → the channel order is wrong. The bridge's startup `rf<N> <- px4[M]`
  lines should read `rf0 <- px4[0]`, `rf1 <- px4[1]`, … on every shipped model; if any row is not
  an identity, the JSON has been edited and should be put back. If they are identities and the
  wrong surface still moves, the mismatch is in the airframe: compare the `PWM_MAIN_FUNC*` block
  (and its comment) against §2 and against your RealFlight model's actual channel order, and fix
  it there.
- **Right surface, wrong direction** → set that channel's bit in `PWM_MAIN_REV` (bit `N-1` for
  channel `N`), or flip the servo's direction in the RealFlight model. Not a JSON edit — see
  [Reversing a channel](#reversing-a-channel).
- **Surface only moves through half its travel, or sits off-centre** → wrong `scale`. Motors are
  `unipolar`, control surfaces are `bipolar`. Scaling a motor as bipolar folds `[0,1]` into
  `[0.5,1]`; scaling a servo as unipolar throws away its whole negative half.
- **Nothing moves at all** → the RealFlight model may still have its own mixes/expo, or PX4 is
  disarmed and the row is sitting at its `disarm` value.

To see what the bridge is actually sending, without QGC in the loop, set
**`PX4_FA_DUMP_CHANNELS=<hz>`** on the make command line: it prints every channel's current value
and its travel, so the channel a surface is on becomes obvious by moving the surface. Documented
in [*Finding which RealFlight channel a surface is on*](README.md#finding-which-realflight-channel-a-surface-is-on).

#### First suspect: a *saved* `PWM_MAIN_FUNC*` from an older install

The airframe assigns the FUNC values with `param set-default`, and a value stored in
`parameters.bson` **beats a default**. The FUNC assignments changed when the channel map was made
an identity, so any parameter you had previously saved to the *old* value silently keeps winning
after the update — and PX4 logs nothing at all when it happens.

You only have a saved value if something wrote one explicitly: QGC's **Actuators tab** does this
whenever you apply a change there, as does a manual `param set`. Merely having run the older
airframe does not, because a `set-default` value that was never touched is not persisted.

What it looks like on a `1200_flightaxis_plane` rootfs where `PWM_MAIN_FUNC1` had been explicitly
saved as its old value `101`:

```
pxh> param show PWM_MAIN_FUNC1
x + PWM_MAIN_FUNC1 : 101          <-- "+" = saved; the airframe's set-default 201 lost

pxh> pwm_out_sim status
Channel 0: func: 101, ...         <-- should be 201, aileron left
Channel 2: func: 101, ...         <-- function 101 now assigned TWICE
```

Aileron left is no longer produced by any output, and RealFlight channel 1 receives the throttle.
No error, no warning. Check with `param show PWM_MAIN_FUNC*` (SITL) or `param show HIL_ACT_FUNC*`
(HITL) and look for the `+` marker. To clear it, `param reset PWM_MAIN_FUNC*` and reboot — that
is enough, and it touches nothing else. Deleting the working directory also works and is a much
blunter instrument: it takes your radio calibration, sensor calibration and flight-mode
assignments with it, and no airframe file will put those back. On a real board, QGC's
**Tools → Reset all parameters** followed by re-selecting the airframe has the same cost.

### `Compass 0 fault` and `Airspeed invalid` in a working directory you just made

Neither message names its cause, and the second is a consequence of the first.

`rcS` writes the magnetometer and gyro **ids** during autoconfig but not their **offsets**, so
a directory that has never been calibrated comes up with `CAL_MAG0_ID` set and
`CAL_MAG0_XOFF/YOFF/ZOFF` absent. That is the compass fault.

The airspeed message follows from it, but not by the route you might expect. The validator's
innovation check — the one `ASPD_FS_INNOV` feeds — **cannot fire on the ground at all**:
`AirspeedValidator::check_airspeed_innovation()` forces `_innovations_check_failed = false`
unless `_in_fixed_wing_flight`, and a preflight message is by definition raised before flight.
(For completeness, exceeding `ASPD_FS_INNOV` only starts an integrator; `ASPD_FS_INTEG` is what
actually trips it.)

What happens on the ground is the fallback. With no valid sensor index, `airspeed_selector`
falls back to *ground-minus-wind*, and that requires `_gnss_lpos_valid`: a recent local position
with `v_xy_valid` **and** EKF2 actively fusing GNSS position or velocity. An EKF whose yaw is
spoiled by the missing compass calibration has not begun GNSS fusion, so the fallback is refused,
the selected index goes to `DISABLED_INDEX`, `calibrated_airspeed_m_s` is published as NaN, and
the preflight check reports "Airspeed invalid" on that NaN alone. So the remedy is unchanged —
fix the compass and the airspeed message goes with it — but nothing about the pitot or
`ASPD_FS_*` is involved.

`sitl_run.sh` avoids this by seeding a working directory that has no `parameters.bson` from
`build/px4_sitl_nolockstep/rootfs`, and says so when it does:

```
seeded parameters.bson from .../build/px4_sitl_nolockstep/rootfs (calibration carried over)
```

If you do not see that line, the directory already had a `parameters.bson` — seeding never
overwrites one — or the shared directory has none to give. Either calibrate in QGC
(**Vehicle Setup → Sensors**, compass then gyroscope) and it is stored for good, or delete
that one file and run again to let the seed fire. `PX4_FLIGHTAXIS_SEED=0` disables seeding
if you want a directory calibrated from scratch.

### `Preflight Fail: ekf2 missing data`

**Benign, and the wording is misleading.** The underlying event text is "Waiting for estimator to
initialize"; only the MAVLink string says "Fail". You get it on every cold start and again after
every spacebar reset, and it clears itself once EKF2 has data to work with. Give it the same
20–30 s the QGC checklist in §5 asks for. It is the same family of message as
[`Disarming denied: not landed`](#disarming-denied-not-landed-after-a-spacebar-reset): a
transient consequence of the estimator settling, not a configuration problem.

### `Preflight Fail: Airspeed selector module down`

A **different** failure from `Airspeed invalid` above, and worth separating. This one means the
`airspeed_selector` module has not published for more than 2 s — the module is not running or is
not being scheduled, rather than running and declaring the airspeed unusable. In this integration
the usual causes are a **stalled physics clock** (RealFlight paused, a modal dialog up, or the
`rtf` collapse described in §5) or a **bridge restart** while PX4 stayed up. Fix the thing feeding
the clock; the module recovers on its own once time advances again.

### `Preflight Fail: Compass %u uncalibrated`

The sibling of `Compass %u fault` above, and the same remedy: it is a working directory whose
magnetometer has no stored offsets. Calibrate in QGC (**Vehicle Setup → Sensors**) or let
`sitl_run.sh` seed the directory — see
[`Compass 0 fault` and `Airspeed invalid`](#compass-0-fault-and-airspeed-invalid-in-a-working-directory-you-just-made).

### `Mission rejected: Landing waypoint/pattern required` / `Switching to Mission is currently not available`

These are the two strings you actually see when a **stale mission from another model** is still
in the shared dataman (§2 preamble). Missions live in the working directory's `dataman` file, not
in the airframe, so flying a second model in the shared
`build/px4_sitl_nolockstep/rootfs` hands it the first model's mission — and a mission written for
a fixed wing is rejected outright for a multirotor, or vice versa.

`MIS_TKO_LAND_REQ` is the parameter that decides whether a landing item is mandatory; the shipped
airframes leave it at its firmware default for the vehicle type, which is why the requirement
appears without anyone having set it. Two ways to clear the mission:

- **QGC → Plan → Clear** (upload an empty plan), or
- delete the `dataman` file in the working directory with PX4 stopped.

The durable fix is not to share the directory at all — give each model its own
`PX4_FLIGHTAXIS_ROOTFS` (§2 preamble).

### `vehicle_command_ack lost, generation N -> M`

**Benign.** A uORB queue-overflow detector in `mavlink_main.cpp`, logged at `PX4_ERR` so it
arrives coloured like a fault. It means a `vehicle_command_ack` was overwritten before this
MAVLink instance read it — common as soon as several MAVLink clients are attached (QGC plus a ROS
2 or MAVSDK client, or the extra endpoints of §4). Nothing is lost that matters; the command
itself was executed and acknowledged to whoever asked.

### Where the logs land

`$rootfs/log/<date>/`, where `$rootfs` is the working directory — the shared
`build/px4_sitl_nolockstep/rootfs`, or whatever you gave `PX4_FLIGHTAXIS_ROOTFS` (§2 preamble).

These airframes `param set-default SDLOG_MODE 0`, which logs from arm to disarm rather than from
boot (`rcS` would otherwise set 1), so
**there is one file per flight** and the newest file in the newest dated directory is the flight
you just flew. That also means a run in which you never armed leaves no file at all, which is
usually the explanation for a missing log rather than a logging failure.

### Aircraft resets or teleports

- On startup, once: expected — `ResetPosition` is set in every shipped model, so the bridge calls
  `ResetAircraft` when it injects the controller.
- Mid-flight: someone pressed spacebar in RealFlight, or the bridge re-injected (see above).
  The position offset is re-captured, so PX4 sees the aircraft back at home rather than a
  position jump.
- If you do *not* want the reset on startup, remove `"ResetPosition"` from that model's
  `Options` — but then PX4's home and the RealFlight position start out inconsistent until the
  first frame anchors them.

### Other pitfalls worth knowing (still real)

| Symptom | Cause / fix |
|---|---|
| Baro and GPS altitude disagree by a constant | Should not happen — both are derived from `PX4_HOME_ALT - position_ned.z` by design. If it does, someone changed `vehicle_state.cpp`. |
| EKF rejects GPS with a fixed delay error | The airframes force `EKF2_GPS_DELAY 0` with `param set` (not `set-default`) because `px4-rc.mavlinksim` runs *after* the airframe and would re-default it to 10 ms. Don't change it back. |
| Quadplane reports a motor failure in cruise | Shouldn't happen: the detector cannot fire in v1.16 SITL at all. `SimulatorMavlink` reports 1–16 A per ESC while armed, and the undercurrent test `1 + 15c < 2c` has no solution. If you see this, something outside SITL changed. |
| Heli rolls when you command pitch | Swashplate angles are not 300/60/180, or the arm lengths are not all 1.0 (§2.4). |
| Heli has no left yaw, tail sits at zero | Someone put `1203` back on `CA_AIRFRAME 10` (tail ESC), which clamps the tail to [0,1], or set `rf3` unipolar in `heli.json`. It ships as `CA_AIRFRAME 11` (tail servo) with `rf3` bipolar, disarm 0.5 (§2.4). |
| Heli swash rails or goes chaotic on landing | Shouldn't happen: the demix runs on a scratch copy, and a `heli.json` row missing its `"disarm"` is rejected by `get_FAbridge_params.py` before the bridge starts. If you see it anyway, the demix is being applied in place again — check `buildChannels()` (§2.4). |
| Bridge on the wrong side of the network | SOAP round-trip dominates the loop; MAVLink to PX4 tolerates far more latency. Put the bridge next to RealFlight, not next to a remote PX4. |

---

## 8. ROS 2

`uxrce_dds_client` starts automatically. It is stock PX4 (`rcS`), not something this
integration configures, and none of the FlightAxis airframes touch it. You only run the agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

Check it took with `uxrce_dds_client status` in the PX4 console — you want `Running, connected`
and `timesync converged: true`.

Two gotchas account for most "the topic is empty" reports. `px4_msgs` must be built from PX4
**v1.16**; a `main` build fails silently at the DDS layer wherever a message changed. And ROS 2
subscribers must use **BEST_EFFORT** QoS.

Topic rates, offboard setup and the timestamp caveats: **[ROS2.md](ROS2.md)**.

---

## 9. Bringing up a new RealFlight model

Several settings are properties of *your* RealFlight aircraft rather than of the bridge, so a new
model needs its own bring-up:

- Whether `CA_SP0_ANG*` 300/60/180 matches the head of the heli model you fly, and whether the
  three swash servos are wired to channels 1–3 in that order (§2.4).
- The **heli rate gains, collective curve and yaw compensation** (§2.4) — generic
  collective-pitch values, not tuned to any particular model. Expect to retune.
- **Quadplane transition** parameters (`VT_F_TRANS_DUR`, `VT_ARSP_TRANS`), which assume a pusher
  that can reach transition airspeed on your model (§2.3).
- `HeliDemix`, if your model has a CCPM head whose mixing cannot be disabled (§2.4).

### Bring-up procedure

Run these in order the first time you fly a new RealFlight model. Anything that fails here is
something to fix on the ground rather than fly around.

1. **Static.** Aircraft level on the runway, disarmed. `listener sensor_accel` at `pxh>` →
   approximately `(0, 0, -9.81)` with **zero jitter** (proves the ground-contact override);
   `listener sensor_gyro` → zero. QGC heading matches the RealFlight compass. Pitch the model
   nose-up 90° in RealFlight — QGC attitude must follow cleanly with no singularity.
2. **Rates.** Roll right → `p > 0`. Pull up → `q > 0`. Yaw right → `r > 0`. (Only yaw is negated
   from RealFlight; if two axes are wrong, the conversion is wrong, not your model.)
3. **Taxi.** Drive north, then east. Local position N/E and velocity vN/vE must track, and the
   QGC map must mirror the RealFlight world.
4. **High alpha.** Hover a 3D model nose-up: the pitot airspeed must read ≈ 0 while RealFlight's
   own `m_airspeed` does not.
5. **EKF.** Fly a manual circuit. Innovations bounded throughout; compare groundtruth against
   estimate in the ulog afterwards.
6. **Resilience.** Press spacebar in RealFlight (bridge must re-inject and re-offset). Change
   aircraft in RealFlight (must auto-restart). Unplug the network for 1 s (glitch counter++, no
   EKF time-jump fault). Kill the bridge mid-flight (PX4 must hit its sensor-timeout failsafe,
   not hang).

Only after 1–4 pass is it reasonable to trust an autonomous mission.
