# Running PX4 SITL against RealFlight (FlightAxis)

Operational guide: how to actually fly it. For *why* it works the way it does, see
[`FLIGHTAXIS_PX4_INTEGRATION.md`](FLIGHTAXIS_PX4_INTEGRATION.md). For installation, see
[`README.md`](README.md).

Assumes the bridge is already installed into a PX4 v1.16 checkout (`./install.sh`), and that
`~/PX4-Autopilot` is that checkout — substitute `<your-px4>` if it lives elsewhere.

---

## 0. If you know the ArduPilot command

This ArduPilot invocation:

```bash
cd ~/ardupilot/ArduPlane
sim_vehicle.py -v ArduPlane \
  -f flightaxis:192.168.10.1 \
  -I0 \
  --custom-location=50.400900,-111.010772,795,90 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14552
```

becomes this PX4 one:

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=50.400900 PX4_HOME_LON=-111.010772 PX4_HOME_ALT=795 \
make px4_sitl_nolockstep flightaxis_plane
```

Extra MAVLink endpoints are not command-line flags in PX4 — you add them in the px4 shell after
boot (§4).

| ArduPilot | PX4 equivalent | Notes |
|---|---|---|
| `-v ArduPlane` + `-f flightaxis:<ip>` | `make px4_sitl_nolockstep flightaxis_plane` | The make target picks both the airframe and the RealFlight channel map. |
| the `:192.168.10.1` in `-f` | `PX4_FLIGHTAXIS_IP=192.168.10.1` | Environment variable, read by `sitl_run.sh`. Defaults to `127.0.0.1`. |
| `-f flightaxis` (copter) | `flightaxis_quad` | |
| `-f flightaxis` (quadplane) | `flightaxis_quadplane` | |
| `-f flightaxis` (heli) | `flightaxis_heli` | |
| `-I0` | *(nothing — instance is hard-coded to 0)* | See §6. Multi-instance is not wired through `make`. |
| `--custom-location=lat,lon,alt,yaw` | `PX4_HOME_LAT` / `PX4_HOME_LON` / `PX4_HOME_ALT` | **There is no yaw/heading variable.** Heading comes from RealFlight itself (§3). |
| `--out=udp:127.0.0.1:14550` | already there by default | PX4 SITL sends to `127.0.0.1:14550` out of the box. |
| additional `--out=` endpoints | `mavlink start -u <local> -o <remote> -t <ip>` in the px4 shell | §4. |
| `--console` / MAVProxy prompt | the `pxh>` shell in the same terminal | |
| `param set X Y` in MAVProxy | `param set X Y` at `pxh>` | |

---

## 1. Network setup (do this first)

This is where people get stuck. RealFlight runs on Windows; PX4 and the bridge run on Linux.
Nothing below is optional.

### 1.1 Enable FlightAxis Link in RealFlight

**Simulation → Settings… → Physics → Quality → tick "FlightAxis Link Enabled".**

RealFlight then listens on **TCP 18083**.

> The in-tree strings disagree about this path and neither is complete: `FA_check.py` prints
> `Settings -> Physics -> Quality -> FlightAxis Link Enabled` (right sub-path, omits the
> Simulation menu) and `install.sh` prints `Simulation > Settings > FlightAxis Link` (right
> menu, omits Physics → Quality). The full path above is the one that matches ArduPilot's
> RealFlight SITL documentation. On some RealFlight builds the Quality preset must be set to
> **Custom** before the checkbox becomes editable.

Also in RealFlight, before flying:

- **Physics speed multiplier must be 1.0.** The bridge warns if it is not; ArduPilot's own SITL
  parameter documentation carries the same warning ("do not use if realtime physics, like
  RealFlight, is being used").
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
    (RealFlight: Settings -> Physics -> Quality -> FlightAxis Link Enabled)
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

Do this once per aircraft in RealFlight's aircraft editor. This is ArduPilot's hard-won advice
(from its RealFlight SITL documentation) and applies unchanged here, because these models are
meant to stay compatible with tridge's RealFlight model collection:

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
Channel map: `models/plane.json`. Options: `ResetPosition`.

**RealFlight aircraft:** any conventional fixed-wing with aileron / elevator / throttle / rudder
on channels 1–4 — the standard ArduPilot RealFlight convention.

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
mixed channel. Only the **left** aileron is mapped; `controls[2]` is deliberately unmapped. This
is the same choice PX4's own FlightGear `rascal.json` makes. If your RF model drives the ailerons
independently, add a row for the other one on a spare RF slot, e.g.:

```json
{"rf": 5, "px4": 2, "scale": "bipolar", "reverse": true}
```

**QGC check:** Vehicle Setup → Actuators should show Motor 1 plus four servos in the order
aileron-left, aileron-right, elevator, rudder. Moving the "Aileron left" slider must move the
RealFlight ailerons; "Aileron right" must move nothing.

---

### 2.2 Quad (multirotor)

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_quad
```

Airframe: `1201_flightaxis_quad` (`SYS_AUTOSTART` 1201, `CA_AIRFRAME 0`, quad X).
Channel map: `models/quad.json`. Options: `ResetPosition`.

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

---

### 2.3 Quadplane (VTOL)

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_quadplane
```

Airframe: `1202_flightaxis_quadplane` (`SYS_AUTOSTART` 1202, `CA_AIRFRAME 2`, standard VTOL,
`VT_TYPE 2`). Channel map: `models/quadplane.json`. Options: `ResetPosition`.

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

**Gotcha — actuator failure detector.** `FD_ACT_EN 0` is set deliberately: the lift motors idle
in fixed-wing cruise and SITL reports 0 A for every ESC, which the detector otherwise reads as a
dead motor. Do not re-enable it here.

**QGC check:** Actuators shows five motors then four servos. Motors 1–4 must move the RealFlight
lift rotors, Motor 5 the pusher.

---

### 2.4 Helicopter

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_heli
```

Airframe: `1203_flightaxis_heli` (`SYS_AUTOSTART` 1203, `CA_AIRFRAME 10` = "Helicopter
(tail ESC)"). Channel map: `models/heli.json`. Options: `ResetPosition`, **`HeliDemix`**.

**RealFlight aircraft:** a collective-pitch heli expecting the usual
roll / pitch / collective / tail / RSC channel set on channels 1–5.

| RF ch | PX4 | Drives (before demix) | Scale | Reverse | Disarm |
|---|---|---|---|---|---|
| 0 | `controls[2]` | swash plate servo 1 | bipolar | no | hold |
| 1 | `controls[3]` | swash plate servo 2 | bipolar | no | hold |
| 2 | `controls[4]` | swash plate servo 3 | bipolar | no | hold |
| 3 | `controls[1]` | tail rotor / yaw (Motor 2) | **unipolar** | no | **0.0** |
| 4 | `controls[0]` | main rotor / RSC (Motor 1) | unipolar | no | **0.0** |

Airframe side: `PWM_MAIN_FUNC1..5 = 101, 102, 201, 202, 203` →
`controls[0]`=Motor 1 main rotor, `[1]`=Motor 2 yaw tail, `[2..4]`=swash servos 1–3.

**Then `HeliDemix` rewrites RF channels 0–2** before they are sent, because RealFlight expects
roll/pitch/collective, not three swash servos:

```
roll  = s1 - s2
pitch = (s1 + s2)/2 - s3
col   = (s1 + s2 + s3)/3      (recentred to 0..1)
```

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
With those, the demix yields exactly `(1.732*roll, 1.5*pitch, collective)` with no cross terms
and no sign inversion. If you change the swash geometry in QGC, `HeliDemix` stops being correct.

**Gotcha — the tail is a motor, so it is unipolar.** Under `CA_AIRFRAME 10` the yaw tail is a
*motor*, and PX4 has already normalised it to `[0,1]`. Scaling it bipolar would fold `[0,1]` into
`[0.5,1.0]` and leave yaw permanently offset. If you switch to `CA_AIRFRAME 11` ("Helicopter
(tail Servo)") you must also change the `rf3` row to `"bipolar"`. Change one, change the other.

**Gotcha — tuning.** The airframe deliberately ships FF-dominant rate loops
(`MC_*RATE_P/I/D = 0`, `MC_*RATE_FF = 0.1`). Set the FF gain for your RealFlight model first,
then add P and I. It will not fly well out of the box.

**QGC check:** Actuators shows Motor 1, Motor 2, then three swash servos, and the swashplate
angles read 300/60/180.

---

## 3. Home position

The equivalent of ArduPilot's `--custom-location`:

| Variable | Default | Meaning |
|---|---|---|
| `PX4_HOME_LAT` | `47.397742` | degrees |
| `PX4_HOME_LON` | `8.545594` | degrees |
| `PX4_HOME_ALT` | `488.0` | metres AMSL |

(Defaults are PX4's usual Zurich origin, read in `flightaxis_bridge.cpp` via `envOrDefault()`.)

Full copy-pasteable command with the coordinates from the ArduPilot example:

```bash
cd ~/PX4-Autopilot
PX4_FLIGHTAXIS_IP=192.168.10.1 \
PX4_HOME_LAT=50.400900 \
PX4_HOME_LON=-111.010772 \
PX4_HOME_ALT=795 \
make px4_sitl_nolockstep flightaxis_plane
```

**There is no yaw / heading variable.** ArduPilot's fourth `--custom-location` field (`90`) has no
counterpart: the bridge reads only the three `PX4_HOME_*` variables. Heading comes from
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

### Adding more endpoints (the `--out=` equivalent)

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

**Worked example — three endpoints like the ArduPilot command.** 14550 already exists, so add two:

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
[flightaxis_bridge] exchanges=248.3/s loop=612.7/s avg=249.1 FPS glitches=0
```
↑ Printed every 1000 bridge frames. Read it as:

| Field | Meaning |
|---|---|
| `exchanges=` | SOAP round-trips completed per second of physics time |
| `loop=` | bridge main-loop iterations per second — informational only; the loop free-runs and extrapolates between real frames, so this is normally *higher* than the exchange rate |
| `avg=` | **the authoritative number** — the true physics frame rate, `1/average_frame_time`. This is what you judge the setup on. Want ~200–300. |
| `glitches=` | cumulative count of >50 ms physics-time discontinuities swallowed. **Should stay at 0 or near it.** A steadily rising count means the network (usually WiFi). |

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
  `[0.5,1]` (this is the classic heli tail bug).
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
| Quadplane reports a motor failure in cruise | `FD_ACT_EN 0` — already set. Don't re-enable. |
| Heli rolls when you command pitch | Swashplate angles are not 300/60/180 (§2.4). |
| Heli yaw permanently offset | `rf3` scaled bipolar instead of unipolar (§2.4). |
| Bridge on the wrong side of the network | SOAP round-trip dominates the loop; MAVLink to PX4 tolerates far more latency. Put the bridge next to RealFlight, not next to a remote PX4. |

---

## 8. Validation before you trust it

Read this before drawing conclusions from a session. The status below mirrors
[`FLIGHTAXIS_PX4_INTEGRATION.md`](FLIGHTAXIS_PX4_INTEGRATION.md) §11 — RealFlight is Windows-only
and **no Windows machine has been in the loop yet**.

### Verified

- The bridge builds; all four `flightaxis_*` make targets exist and resolve.
- All four model JSONs flatten cleanly through `get_FAbridge_params.py`.
- `FA_check.py` fails correctly, with its diagnostics, against an unreachable host.
- **End-to-end against a mock FlightAxis server**: PX4 connects on TCP 4560, EKF2 converges, the
  synthesised sensors are sane, and baro and GPS altitudes agree.
- Channel maps correct end to end for **plane and quad**, including bipolar/reverse/unipolar
  scaling and disarm values.
- Resilience: reconnect, aircraft reset, glitch swallow, and bridge death all behave as designed.

### Not yet verified — needs a real RealFlight session

- **Rate sign conventions** (roll/pitch/yaw polarity out of RealFlight).
- **Taxi tracking** of local N/E against the RealFlight world.
- **High-alpha pitot** behaviour.
- Compass heading, and the nose-up-90° attitude case.
- **A flown circuit** — EKF innovation bounds over a real manoeuvre are unknown.
- The **quadplane and heli** channel maps beyond static reasoning, and `HeliDemix` against a real
  swashplate.

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
