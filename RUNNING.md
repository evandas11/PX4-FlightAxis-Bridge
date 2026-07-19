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

| Make target | Airframe (`SYS_AUTOSTART`) | Vehicle type | Description | Status |
|---|---|---|---|---|
| `flightaxis_plane` | `1200_flightaxis_plane` (1200) | Fixed-wing (`CA_AIRFRAME 1`) | Conventional plane: aileron / elevator / throttle / rudder | **Working** — channel map verified end to end |
| `flightaxis_quad` | `1201_flightaxis_quad` (1201) | Multirotor quad-X (`CA_AIRFRAME 0`) | Four motors, direct from PX4 motor outputs | **Working** — channel map verified end to end |
| `flightaxis_quadplane` | `1202_flightaxis_quadplane` (1202) | Standard VTOL (`CA_AIRFRAME 2`) | reference-class: 4 lift motors + pusher + 4 surfaces | **Partial** — map verified by reasoning only, not flown (§9) |
| `flightaxis_heli` | `1203_flightaxis_heli` (1203) | Helicopter, tail servo (`CA_AIRFRAME 11`) | Collective-pitch heli; bridge demixes swash to roll/pitch/collective | **Partial** — `HeliDemix` never tested against a real swashplate, gains never flown (§9) |

`flightaxis` on its own is an alias for `flightaxis_plane`.

RealFlight is an RC flight simulator, so these four aircraft classes are what it can model.

---

## 1. Network setup (do this first)

This is where people get stuck. RealFlight runs on Windows; PX4 and the bridge run on Linux.
Nothing below is optional.

### 1.1 Enable FlightAxis Link in RealFlight

**Simulation → Settings… → Physics → Quality → tick "FlightAxis Link Enabled".**

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
  - Is RealFlight running with FlightAxis Link enabled?
    (RealFlight: Simulation -> Settings... -> Physics -> Quality ->
     tick 'FlightAxis Link Enabled')
  - Is the host reachable / firewall open on TCP 18083?
  - Set PX4_FLIGHTAXIS_IP if RealFlight is not on 192.168.10.1.
```

Or without the script:

```bash
nc -zv 192.168.10.1 18083
```

### 1.6 Same-host case

If RealFlight runs in a VM on the same machine with host networking, or you are testing against a
mock server, `PX4_FLIGHTAXIS_IP` can be omitted entirely — **`sitl_run.sh` defaults it to
`127.0.0.1`**.

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
- **Reverse** — applied *after* scaling, as `v → 1-v`.
- **Disarm** — value sent while PX4 is disarmed or the control is NaN. `hold` means "keep the
  last output" (neutral 0.5 before the first one).
- Every RealFlight channel not in the table is driven at **0.5** (`UnmappedDefault`).

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
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_plane
```

Airframe: `1200_flightaxis_plane` (`SYS_AUTOSTART` 1200, `CA_AIRFRAME 1`).
Channel map: `models/plane.json`. Options: `ResetPosition` (bitmask **1**).

**RealFlight aircraft:** any conventional fixed-wing with aileron / elevator / throttle / rudder
on channels 1–4, which is how RealFlight fixed-wing models are conventionally wired.

| RF ch | PX4 | Drives | Scale | Reverse | Disarm |
|---|---|---|---|---|---|
| 0 | `controls[1]` | aileron (PX4 **aileron left**) | bipolar | no | hold |
| 1 | `controls[3]` | elevator | bipolar | **yes** | hold |
| 2 | `controls[0]` | throttle (Motor 1) | unipolar | no | **0.0** |
| 3 | `controls[4]` | rudder | bipolar | no | hold |

Airframe side: `PWM_MAIN_FUNC1..5 = 101, 201, 202, 203, 204` →
`controls[0]`=Motor 1, `[1]`=Servo 1 aileron left, `[2]`=Servo 2 aileron right,
`[3]`=Servo 3 elevator, `[4]`=Servo 4 rudder.

**Gotcha — split ailerons.** PX4 declares two ailerons (`CA_SV_CS0` left → `controls[1]`,
`CA_SV_CS1` right → `controls[2]`) but RealFlight models normally drive both ailerons from one
mixed channel. Only the **left** aileron is mapped; `controls[2]` is deliberately unmapped. The
two allocator outputs carry the same magnitude with opposite sign, so driving a single mixed
RealFlight channel from one of them loses nothing. If your RF model drives the ailerons
independently, add a row for the other one on a spare RF slot, e.g.:

```json
{"rf": 5, "px4": 2, "scale": "bipolar", "reverse": true}
```

**QGC check:** Vehicle Setup → Actuators should show Motor 1 plus four servos in the order
aileron-left, aileron-right, elevator, rudder. Moving the "Aileron left" slider must move the
RealFlight ailerons; "Aileron right" must move nothing.

**First flight.** Do the generic static/rates/taxi checks in §9 first, then: arm on the runway in
**Position** mode and confirm the throttle idles at zero (disarm value 0.0, not half). Take off
in **Stabilized**, hold wings level, and check the stick-to-surface polarity matches §9 step 2.
Only then try **Mission** — the airframe sets `RWTO_TKOFF 1`, so a runway takeoff is expected,
and `NAV_ACC_RAD 15` means waypoints are considered hit from 15 m out.

---

### 2.2 Quad (multirotor)

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_quad
```

Airframe: `1201_flightaxis_quad` (`SYS_AUTOSTART` 1201, `CA_AIRFRAME 0`, quad X).
Channel map: `models/quad.json`. Options: `ResetPosition` (bitmask **1**).

**RealFlight aircraft:** any quad-X multirotor whose four motors sit on channels 1–4
individually (no RealFlight-side mixing).

| RF ch | PX4 | Drives | Scale | Reverse | Disarm |
|---|---|---|---|---|---|
| 0 | `controls[0]` | motor 1 | unipolar | no | **0.0** |
| 1 | `controls[1]` | motor 2 | unipolar | no | **0.0** |
| 2 | `controls[2]` | motor 3 | unipolar | no | **0.0** |
| 3 | `controls[3]` | motor 4 | unipolar | no | **0.0** |

Airframe side: `PWM_MAIN_FUNC1..4 = 101..104`. The only model where the RealFlight order and the
PX4 order coincide.

**Gotcha — motor numbering and spin direction.** PX4's quad-X numbering is
1=front-right (CW-position, `KM +0.05`), 2=rear-left, 3=front-left, 4=rear-right, per the
`CA_ROTOR*_PX/PY/KM` values in the airframe. If your RealFlight model numbers its motors
differently, or spins them the other way, the vehicle will flip on takeoff. Reorder the `rf`
indices in `quad.json` (not the `px4` ones) to match your model.

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
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_quadplane
```

Airframe: `1202_flightaxis_quadplane` (`SYS_AUTOSTART` 1202, `CA_AIRFRAME 2`, standard VTOL,
`VT_TYPE 2`). Channel map: `models/quadplane.json`. Options: `ResetPosition` (bitmask **1**).

**RealFlight aircraft:** a reference-class quadplane — aileron / elevator / forward throttle /
rudder on channels 1–4, four lift motors on channels 5–8.

| RF ch | PX4 | Drives | Scale | Reverse | Disarm |
|---|---|---|---|---|---|
| 0 | `controls[5]` | aileron (PX4 **aileron left**) | bipolar | no | hold |
| 1 | `controls[7]` | elevator | bipolar | **yes** | hold |
| 2 | `controls[4]` | forward / pusher throttle (Motor 5) | unipolar | no | **0.0** |
| 3 | `controls[8]` | rudder | bipolar | no | hold |
| 4 | `controls[0]` | lift motor 1 | unipolar | no | **0.0** |
| 5 | `controls[1]` | lift motor 2 | unipolar | no | **0.0** |
| 6 | `controls[2]` | lift motor 3 | unipolar | no | **0.0** |
| 7 | `controls[3]` | lift motor 4 | unipolar | no | **0.0** |

Airframe side: `PWM_MAIN_FUNC1..9 = 101,102,103,104,105,201,202,203,204` →
`controls[0..3]`=lift motors 1–4, `[4]`=forward motor, `[5]`=aileron left,
`[6]`=aileron right (unmapped), `[7]`=elevator, `[8]`=rudder.

The scattered `px4` column is the whole point of the JSON: PX4's allocator always emits **motors
before servos**, so the four lift motors land on `controls[0..3]` even though they sit on RF
channels 5–8.

**Gotcha — do NOT enable `Rev4Servos`.** That option swaps RF channels 0–3 with 4–7 wholesale.
`quadplane.json` already writes the `rf` indices out directly in the order the RealFlight model
wants, so enabling `Rev4Servos` would double-swap and put the lift motors on the surfaces.
It exists for RF models built the other way round, where the map is written sequentially.

**Gotcha — transition.** The airframe sets `VT_F_TRANS_THR 0.75`, `VT_FWD_THRUST_EN 4`,
`FW_AIRSPD_MAX 25`. Transition needs the pusher to actually accelerate the model; if the
RealFlight airframe is much draggier or lighter than a reference airframe, transition will stall or never
complete. Tune `VT_F_TRANS_DUR` / `VT_ARSP_TRANS` for your model rather than assuming the
defaults fit.

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
attempting any transition. Transition last, with plenty of altitude: it is the least verified
part of this airframe (§9), and a failed forward transition drops the aircraft.

---

### 2.4 Helicopter

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_heli
```

Airframe: `1203_flightaxis_heli` (`SYS_AUTOSTART` 1203, `CA_AIRFRAME 11` = "Helicopter
(tail Servo)"). Channel map: `models/heli.json`. Options: `ResetPosition` + **`HeliDemix`**
(bitmask **5** = 1 | 4).

**RealFlight aircraft:** a collective-pitch heli expecting the usual
roll / pitch / collective / tail / RSC channel set on channels 1–5.

| RF ch | PX4 | Drives (before demix) | Scale | Reverse | Disarm |
|---|---|---|---|---|---|
| 0 | `controls[2]` | swash plate servo 1 | bipolar | no | **0.5** |
| 1 | `controls[3]` | swash plate servo 2 | bipolar | no | **0.5** |
| 2 | `controls[4]` | swash plate servo 3 | bipolar | no | **0.5** |
| 3 | `controls[1]` | tail rotor pitch / yaw (Servo 1) | bipolar | no | **0.5** |
| 4 | `controls[0]` | main rotor / RSC (Motor 1) | unipolar | no | **0.0** |

Airframe side: `PWM_MAIN_FUNC1..5 = 101, 201, 202, 203, 204` →
`controls[0]`=Motor 1 main rotor, `[1]`=Servo 1 yaw tail, `[2..4]`=Servos 2–4, the swash
plate servos 1–3.

**Every row carries an explicit `disarm`**, and `get_FAbridge_params.py` refuses to flatten
the file if one is missing. The bridge demixes out of a scratch copy and leaves the
persistent channel array holding raw servo values, so a `-1` "hold last output" row would in
fact hold correctly today. It did not always. When the demix was applied in place, a held row
had its already-demixed value fed back through the demix on the next frame, and that
iteration diverges — from a plausible in-flight swash it railed within about three frames.
Neutral is a fixed point, so it only bit on the armed→disarmed transition with the swash
deflected, which is to say on every landing, and nothing was logged when it did. The rule
stays enforced because the property that makes holding safe lives in one function in the
bridge, and nothing in a model JSON can see whether it still holds.

**Then `HeliDemix` rewrites RF channels 0–2** before they are sent, because RealFlight expects
roll/pitch/collective, not three swash servos:

```
roll  = (s1 - s2) / 1.732
pitch = ((s1 + s2)/2 - s3) / 1.5
col   = (s1 + s2 + s3)/3      (recentred to 0..1)
```

The two divisors are gain normalisation, not part of the geometric inverse. With the swash
angles the airframe pins (below) and the 0.5 per-servo scale, the raw differences come out at
gain 0.866 for roll and 0.750 for pitch, where RealFlight wants 0.5 about its 0.5 centre;
dividing by 0.866/0.5 = √3 and 0.750/0.5 = 1.5 lands them exactly. Without this the cyclic
saturates at ±0.577 of commanded roll and ±0.667 of pitch, with roll noticeably hotter than
pitch. Collective is already at gain 0.5 and is left alone.

So what RealFlight actually receives is:

| RF ch | RealFlight sees |
|---|---|
| 0 | roll (aileron / cyclic) |
| 1 | pitch (elevator / cyclic) |
| 2 | collective |
| 3 | tail / yaw |
| 4 | main rotor RSC |

**Gotcha — swashplate angles are mandatory.** That demix is only an exact inverse for a swash
geometry with servos 1 and 2 mirrored about the X axis and servo 3 on the axis. PX4's defaults
(`CA_SP0_ANG*` = 0/140/220) put the mirrored pair on servos 2 and 3, which leaves the demix
cross-coupling roll into pitch. The airframe therefore **forces `CA_SP0_ANG0/1/2 = 300/60/180`**.
With those, the demix decouples exactly — no cross terms, no sign inversion on any axis — and
the two divisors above are the correct ones. Change the swash geometry in QGC and both
properties are lost. The airframe also pins `CA_SP0_ARM_L0/1/2` to 1.0 and `CA_MAX_SVO_THROW`
to 0 for the same reason: unequal arms break the inverse, and a non-zero throw limit enables a
per-servo `asin()` linearisation that makes the mapping non-linear.

**Gotcha — the tail is a servo, so it is bipolar.** Under `CA_AIRFRAME 11` the yaw tail is a
*servo* on `[-1,1]`, and 0.5 on the wire is zero tail pitch — which is what a collective-pitch
tail rotor wants, and what nearly every stock RealFlight single-rotor heli models. Under
`CA_AIRFRAME 10` ("tail ESC") the tail would be a motor clamped to `[0,1]`: the entire negative
half of the yaw command is clipped, the tail idles on its lower stop, and there is **no left
yaw authority at all**. If you genuinely fly a model with a separate unidirectional electric
tail motor, switch to `CA_AIRFRAME 10`, renumber `PWM_MAIN_FUNC2..5` to `102/201/202/203` — the
swash shifts down one servo index because the tail no longer consumes Servo 1 — and change
`rf3` back to `"unipolar"` with `"disarm": 0.0`. The `controls[]` indices are unchanged by that
swap.

**Gotcha — tuning.** The airframe ships real rate gains, typical of collective-pitch helicopters
rather than derived from this airframe: roll/pitch `P 0.025 I 0.15 D 0.001 FF 0.15`, yaw `P 0.18 I 0.12 D 0.003
FF 0.02`. They are feed-forward dominant on purpose — a swash gives a large, fast, near-linear
torque response, so a large P term only chases rotor phase lag. This is a starting point, not a
tune: **it has never been flown against RealFlight physics.** Refine it in Acro. The airframe
also sets an explicit collective curve (`CA_HELI_PITCH_C*`, spanning RealFlight 0.45–0.85 so
hover sits near mid-stick rather than near the top of a quarter of the travel) and yaw
compensation for main-rotor torque (`CA_HELI_YAW_CP_S 0.25`); the blade angle a given channel
value actually produces is a property of the RealFlight model, so verify and rescale.

**Gotcha — the RealFlight model needs work.** More than the other three vehicles. RC heli
models ship with stabilisation and mixing inside the model, and all of it fights PX4: the
tail heading-hold gyro must be zeroed (it is an independent yaw controller and will produce a
slow oscillation no `MC_YAWRATE_*` value fixes), CCPM/eCCPM swash mixing must be off since the
bridge already sends demixed roll/pitch/collective, and the model's own collective and throttle
curves must be flattened because PX4 owns both. Leave a governor on, if the model has one — PX4
holds the throttle channel at a constant 100 % and expects constant head speed.

**QGC check:** Actuators shows Motor 1, then four servos — yaw tail first, then the three swash
servos — and the swashplate angles read 300/60/180.

**First flight.** Check the swashplate on the ground first: cyclic right must tilt the RealFlight
swash right with **no** pitch component, and cyclic forward must tilt it forward with no roll
component. Cross-coupling here means the `CA_SP0_ANG*` override did not take, and `HeliDemix` is
no longer a correct inverse — fix it before spooling up. Check yaw in both directions while you
are there: full left and full right must move the tail servo symmetrically about its centre. If
left yaw does nothing, the airframe is on `CA_AIRFRAME 10` or `rf3` is unipolar. Then bring the
rotor to speed, raise collective to a light skid, and confirm yaw holds before lifting off.
Expect to refine the rate gains and the collective curve for your model.

---

## 3. Home position

Where the aircraft sits on the map is set by three environment variables:

| Variable | Default | Meaning |
|---|---|---|
| `PX4_HOME_LAT` | `47.397742` | degrees |
| `PX4_HOME_LON` | `8.545594` | degrees |
| `PX4_HOME_ALT` | `488.0` | metres AMSL |

(Defaults are PX4's usual Zurich origin, read in `flightaxis_bridge.cpp` via `envOrDefault()`.)

Full copy-pasteable command with a worked set of coordinates:

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=50.400900 \
PX4_HOME_LON=-111.010772 \
PX4_HOME_ALT=795 \
make px4_sitl_nolockstep flightaxis_plane
```

**There is no yaw / heading variable.** The bridge reads only the three `PX4_HOME_*` variables
above; there is no fourth one for initial heading. Heading comes from
RealFlight's own aircraft attitude, so the aircraft points wherever the RealFlight model is
pointing on its runway. If you need a specific initial heading, set it in RealFlight.

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
PX4_HOME_LAT=50.400900 PX4_HOME_LON=-111.010772 PX4_HOME_ALT=795 \
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
[flightaxis_bridge] options=0x1 unmapped_default=0.5 channels=4
  rf0 <- px4[1] bipolar disarm=-1
  rf1 <- px4[3] bipolar reversed disarm=-1
  rf2 <- px4[0] unipolar disarm=0
  rf3 <- px4[4] bipolar disarm=-1
```
↑ The channel map the bridge actually parsed. **Check this against §2 for your vehicle** — it is
the fastest way to catch an edited JSON. `disarm=-1` means "hold last output".

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
[flightaxis_bridge] connecting to RealFlight at 192.168.10.1:18083
[flightaxis_bridge] controller injected, aircraft reset
```
↑ The UAV controller interface was injected into RealFlight and (because `ResetPosition` is set)
the aircraft was reset to the runway. RealFlight's own controls are now overridden by PX4.

```
[flightaxis_bridge] first HIL_ACTUATOR_CONTROLS received, enabling channels
```
↑ PX4 has produced outputs; the bridge switches RealFlight's `selectedChannels` from 0 to all 12.
Before this the model deliberately sees nothing.

```
[flightaxis_bridge] exchanges=248.3/s loop=612.7/s avg=249.1 FPS rtf=1.00 glitches=0
```
↑ Printed every 1000 bridge frames. Read it as:

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
3. **EKF healthy** — no "Preflight fail: EKF" messages; attitude, velocity and position estimates
   all valid. Give it 20–30 s after start.
4. **Actuators move.** Vehicle Setup → Actuators, enable sliders, and confirm each PX4 actuator
   moves the *expected* RealFlight surface/rotor in the *expected* direction (§2 tables).
5. **Altitude sane** — baro and GPS altitude both around `PX4_HOME_ALT`, and agreeing with each
   other (they are derived from the same datum by design).
6. **No physics-speed warning** in the console.

---

## 6. Multiple instances

**Be clear about this: multi-instance is not wired through the make targets.**

- The bridge *does* support it. Its first argv is the PX4 instance id, and
  `px4_communicator.cpp` binds `portBase + portOffset`, i.e. `4560 + instance`.
- But `sitl_run.sh` passes a **hard-coded `0`**, with no environment override:

  ```bash
  "${build_path}/build_flightaxis_bridge/flightaxis_bridge" 0 "${FA_IP}" $fa_bridge_params &
  ```

So a second `make ... flightaxis_*` would just collide on TCP 4560. There is no
`PX4_FLIGHTAXIS_INSTANCE` variable. Threading one through `sitl_run.sh` is the obvious fix and it
has not been done.

**Manual workaround** (documented as a workaround, not a supported path). Launch the bridge and
`px4` by hand:

```bash
cd ~/PX4-Autopilot/Tools/simulation/flightaxis/flightaxis_bridge

# instance 1 -> listens on TCP 4561
PX4_HOME_LAT=50.400900 PX4_HOME_LON=-111.010772 PX4_HOME_ALT=795 \
~/PX4-Autopilot/build/px4_sitl_nolockstep/build_flightaxis_bridge/flightaxis_bridge \
  1 192.168.10.2 $(./get_FAbridge_params.py models/plane.json) &

# then PX4 instance 1 in its own rootfs
mkdir -p /tmp/px4_i1 && cd /tmp/px4_i1
PX4_SIM_MODEL=flightaxis_plane \
~/PX4-Autopilot/build/px4_sitl_nolockstep/bin/px4 -i 1 \
  ~/PX4-Autopilot/build/px4_sitl_nolockstep/etc
```

Note `PX4_HOME_*` must be set on the **bridge** process, not on `px4`. And in practice each
instance needs its own RealFlight host anyway — one RealFlight instance serves one aircraft.

---

## 7. Troubleshooting

Keyed off the strings the software actually prints.

### `RealFlight FlightAxis not reachable at 192.168.10.1:18083`

From `sitl_run.sh` after `FA_check.py` failed; nothing else was started. In order:
RealFlight running? FlightAxis Link ticked (§1.1)? Right IP (`ipconfig`)? Firewall (§1.3)?
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

Healthy output is one line of numbers, e.g. `1 0.5 4 0 1 1 0 -1 1 3 1 1 -1 2 0 0 0 0 3 4 1 0 -1`.
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

- **Wrong surface moves** → the `rf`↔`px4` pairing is wrong. Compare the bridge's startup
  `rf<N> <- px4[M]` lines against §2 and against the `PWM_MAIN_FUNC*` comment block in the
  airframe file.
- **Right surface, wrong direction** → flip `"reverse"` on that row.
- **Surface only moves through half its travel, or sits off-centre** → wrong `scale`. Motors are
  `unipolar`, control surfaces are `bipolar`. Scaling a motor as bipolar folds `[0,1]` into
  `[0.5,1]`; scaling a servo as unipolar throws away its whole negative half.
- **Nothing moves at all** → the RealFlight model may still have its own mixes/expo, or PX4 is
  disarmed and the row's `disarm` is holding it.

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

Measured topic rates, offboard setup and the timestamp caveats: **[ROS2.md](ROS2.md)**.

---

## 9. Validation before you trust it

Read this before drawing conclusions from a session. The status below mirrors
[`FLIGHTAXIS_PX4_INTEGRATION.md`](FLIGHTAXIS_PX4_INTEGRATION.md) §11 — RealFlight is Windows-only
and **no Windows machine has been in the loop yet**.

### Verified — against a mock FlightAxis server

Everything in this list was observed against a local SOAP responder replaying the RealFlight
schema. It has **no flight dynamics**: it returns a fixed aircraft state whatever actuator
values it is given. So this list is evidence about the software path and nothing else — no
aircraft has been flown, in RealFlight or anywhere.

- The bridge builds; all four `flightaxis_*` and all four `flightaxis_hitl_*` make targets exist
  and resolve.
- All four model JSONs flatten cleanly through `get_FAbridge_params.py`.
- `FA_check.py` fails correctly, with its diagnostics, against an unreachable host.
- **End to end**: PX4 connects on TCP 4560, EKF2 converges, the synthesised sensors are sane,
  and baro and GPS altitudes agree.
- Channel maps correct end to end for **plane and quad**, including bipolar/reverse/unipolar
  scaling and disarm values.
- Resilience: reconnect, aircraft reset, glitch swallow, and bridge death all behave as designed.
- The ROS 2 path, including an offboard node arming and driving the actuators — see
  [`ROS2.md`](ROS2.md) §8 for what that does and does not establish.

### Not yet verified — needs a real RealFlight session

- **Rate sign conventions** (roll/pitch/yaw polarity out of RealFlight).
- **Taxi tracking** of local N/E against the RealFlight world.
- **High-alpha pitot** behaviour.
- Compass heading, and the nose-up-90° attitude case.
- **A flown circuit** — EKF innovation bounds over a real manoeuvre are unknown.
- The **quadplane and heli** channel maps beyond static reasoning, and `HeliDemix` against a real
  swashplate.
- The **heli rate gains, collective curve and yaw compensation** (§2.4). They are generic
  collective-pitch helicopter values, not measured against RealFlight physics.

Hardware-in-the-loop has its own, separate status: nothing in [`HITL.md`](HITL.md) has been run
against a physical flight-controller board.

### First-flight procedure

Run these in order on your first real session, and treat a failure as a bug to report rather
than something to fly around.

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
