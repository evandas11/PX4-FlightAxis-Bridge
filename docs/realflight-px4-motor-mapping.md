# RealFlight ↔ PX4 — Motor Direction & Reverse Map, per Airframe

Configuration notes for motor spin direction (CW/CCW) and reverse for every
FlightAxis airframe, so that setting up the model in RealFlight against
QGroundControl is unambiguous — which box to tick, and what to reverse.

Every value here is derived **directly** from the shipped `CA_ROTOR*` geometry in
`ROMFS/px4fmu_common/init.d-posix/airframes/12xx_flightaxis_*`. That geometry is
deliberately kept **verbatim from stock PX4** — do not edit the `KM` signs. If a
motor spins the wrong way, fix it **on the RealFlight side**, not by changing `KM`
in PX4.

> **Scope — SITL vs HITL.** The mapping tables below are **identical** for both:
> the RealFlight model and the PX4 geometry are the same whether you run SITL (the
> RealFlight simulator only) or HITL (a real flight-controller board + real ESCs
> via the `.hil` airframes). Only the *verification step* differs — in SITL it is a
> purely visual check, in HITL it is a physical bench test with **propellers off**.
> Wording throughout is written for SITL; the HITL differences are called out
> explicitly. For the HITL side, see [HITL.md](../HITL.md).

---

## Universal rules (apply to every model)

### 1. PX4 `KM` sign → CCW checkbox in QGC

| `CA_ROTORx_KM` | Direction in PX4 | CCW column in QGC (Actuators) |
|----------------|------------------|-------------------------------|
| positive / unset (default `+0.05`) | **CCW** | ✅ **ticked** |
| `-0.05` | **CW** | ⬜ **unticked** |

### 2. RealFlight direction = **opposite** of PX4

For **every** motor: whatever is CCW in QGC → set it **CW** in RealFlight, and
vice versa. PX4 and RealFlight use opposite spin-direction conventions (ArduPilot
happens to match RealFlight; PX4 is the inverse). It is not a bug — just remember
it as a single rule.

### 3. Position convention (from `PX`/`PY`/`PZ`)

| Axis | Positive | Negative |
|------|----------|----------|
| `PX` | front (nose) | rear |
| `PY` | right | left |
| `PZ` | bottom | top |

For coaxial frames, `PZ -0.05` = **top** rotor, `PZ +0.05` = **bottom** rotor; two
motors with the same `PX`/`PY` but opposite `PZ` are the **same arm** (a
counter-rotating pair).

### 4. Control surfaces / servos ≠ the rule above

Aileron, elevator, rudder, elevon, tilt, swash — their deflection direction is
**not** a CW/CCW motor concern. It is set by:
- **PX4:** `PWM_MAIN_REV` (a per-channel bitmask; bit `N-1` for channel `N`), or
- **RealFlight:** the servo direction in the model.

Always verify it in the simulator: move the stick → confirm the surface deflects
the correct way (in HITL, do the same on the bench with props off). The "invert
everything" rule is for **lift motors only**; it does not carry over to servos.

> Motor number `N` = `CA_ROTOR(N-1)`. For pure multirotors, RF channel `N` carries
> Motor `N` (identity pipe). The "Position" column uses the raw `(PX, PY[, PZ])`
> so it stays unambiguous.

---

## Quad family

### 1201 — Quadrotor X

| Motor | Position (PX,PY) | Arm | PX4 / QGC | RealFlight |
|-------|------------------|-----|-----------|------------|
| 1 | (0.15, 0.15) | front-right | CCW ✅ | **CW** |
| 2 | (-0.15, -0.15) | rear-left | CCW ✅ | **CW** |
| 3 | (0.15, -0.15) | front-left | CW ⬜ | **CCW** |
| 4 | (-0.15, 0.15) | rear-right | CW ⬜ | **CCW** |

### 1204 — Quadrotor + (plus)

| Motor | Position (PX,PY) | Arm | PX4 / QGC | RealFlight |
|-------|------------------|-----|-----------|------------|
| 1 | (0, 1) | right | CCW ✅ | **CW** |
| 2 | (0, -1) | left | CCW ✅ | **CW** |
| 3 | (1, 0) | front | CW ⬜ | **CCW** |
| 4 | (-1, 0) | rear | CW ⬜ | **CCW** |

### 1202 — Quadplane (Standard VTOL)

Lift motors = **Motor 1–4** (geometry identical to Quad X 1201):

| Motor | Position (PX,PY) | Arm | PX4 / QGC | RealFlight |
|-------|------------------|-----|-----------|------------|
| 1 | (0.15, 0.15) | front-right | CCW ✅ | **CW** |
| 2 | (-0.15, -0.15) | rear-left | CCW ✅ | **CW** |
| 3 | (0.15, -0.15) | front-left | CW ⬜ | **CCW** |
| 4 | (-0.15, 0.15) | rear-right | CW ⬜ | **CCW** |

Plus a **pusher motor** (Motor 5, forward) + left/right aileron, elevator, rudder.
The pusher is a single motor (just make sure it thrusts forward). Control surfaces
follow rule #4 (reverse channel).

---

## Hexa family

### 1205 — Hexarotor X

| Motor | Position (PX,PY) | Arm | PX4 / QGC | RealFlight |
|-------|------------------|-----|-----------|------------|
| 1 | (0, 0.5) | right (mid) | CW ⬜ | **CCW** |
| 2 | (0, -0.5) | left (mid) | CCW ✅ | **CW** |
| 3 | (0.43, -0.25) | front-left | CW ⬜ | **CCW** |
| 4 | (-0.43, 0.25) | rear-right | CCW ✅ | **CW** |
| 5 | (0.43, 0.25) | front-right | CCW ✅ | **CW** |
| 6 | (-0.43, -0.25) | rear-left | CW ⬜ | **CCW** |

### 1206 — Hexarotor + (plus)

| Motor | Position (PX,PY) | Arm | PX4 / QGC | RealFlight |
|-------|------------------|-----|-----------|------------|
| 1 | (0.5, 0) | front | CW ⬜ | **CCW** |
| 2 | (-0.5, 0) | rear | CCW ✅ | **CW** |
| 3 | (-0.25, -0.43) | rear-left | CW ⬜ | **CCW** |
| 4 | (0.25, 0.43) | front-right | CCW ✅ | **CW** |
| 5 | (0.25, -0.43) | front-left | CCW ✅ | **CW** |
| 6 | (-0.25, 0.43) | rear-right | CW ⬜ | **CCW** |

### 1207 — Hexarotor Coaxial (3 arms × 2)

| Motor | Position (PX,PY,PZ) | Arm | Top/Bottom | PX4 / QGC | RealFlight |
|-------|---------------------|-----|-----------|-----------|------------|
| 1 | (0.25, 0.433, -0.05) | front-right | top | CW ⬜ | **CCW** |
| 2 | (0.25, 0.433, +0.05) | front-right | bottom | CCW ✅ | **CW** |
| 3 | (-0.5, 0, -0.05) | rear | top | CW ⬜ | **CCW** |
| 4 | (-0.5, 0, +0.05) | rear | bottom | CCW ✅ | **CW** |
| 5 | (0.25, -0.433, -0.05) | front-left | top | CW ⬜ | **CCW** |
| 6 | (0.25, -0.433, +0.05) | front-left | bottom | CCW ✅ | **CW** |

Same-arm pairs: (M1,M2), (M3,M4), (M5,M6) — each counter-rotating.

---

## Octo family

### 1208 — Octorotor X

| Motor | Position (PX,PY) | Arm (approx.) | PX4 / QGC | RealFlight |
|-------|------------------|---------------|-----------|------------|
| 1 | (0.46, 0.19) | front, right | CW ⬜ | **CCW** |
| 2 | (-0.46, -0.19) | rear, left | CW ⬜ | **CCW** |
| 3 | (0.19, 0.46) | right, front | CCW ✅ | **CW** |
| 4 | (-0.46, 0.19) | rear, right | CCW ✅ | **CW** |
| 5 | (0.46, -0.19) | front, left | CCW ✅ | **CW** |
| 6 | (-0.19, -0.46) | left, rear | CCW ✅ | **CW** |
| 7 | (0.19, -0.46) | left, front | CW ⬜ | **CCW** |
| 8 | (-0.19, 0.46) | right, rear | CW ⬜ | **CCW** |

### 1209 — Octorotor + (plus)

| Motor | Position (PX,PY) | Arm | PX4 / QGC | RealFlight |
|-------|------------------|-----|-----------|------------|
| 1 | (0.5, 0) | front | CW ⬜ | **CCW** |
| 2 | (-0.5, 0) | rear | CW ⬜ | **CCW** |
| 3 | (0.35, 0.35) | front-right | CCW ✅ | **CW** |
| 4 | (-0.35, 0.35) | rear-right | CCW ✅ | **CW** |
| 5 | (0.35, -0.35) | front-left | CCW ✅ | **CW** |
| 6 | (-0.35, -0.35) | rear-left | CCW ✅ | **CW** |
| 7 | (0, -0.5) | left | CW ⬜ | **CCW** |
| 8 | (0, 0.5) | right | CW ⬜ | **CCW** |

### 1210 — Octo Coaxial (4 arms × 2)

| Motor | Position (PX,PY,PZ) | Arm | Top/Bottom | PX4 / QGC | RealFlight |
|-------|---------------------|-----|-----------|-----------|------------|
| 1 | (0.35, 0.35, -0.05) | front-right | top | CCW ✅ | **CW** |
| 2 | (0.35, -0.35, -0.05) | front-left | top | CW ⬜ | **CCW** |
| 3 | (-0.35, -0.35, -0.05) | rear-left | top | CCW ✅ | **CW** |
| 4 | (-0.35, 0.35, -0.05) | rear-right | top | CW ⬜ | **CCW** |
| 5 | (0.35, -0.35, +0.05) | front-left | bottom | CCW ✅ | **CW** |
| 6 | (0.35, 0.35, +0.05) | front-right | bottom | CW ⬜ | **CCW** |
| 7 | (-0.35, 0.35, +0.05) | rear-right | bottom | CCW ✅ | **CW** |
| 8 | (-0.35, -0.35, +0.05) | rear-left | bottom | CW ⬜ | **CCW** |

Same-arm pairs (top + bottom):
- Front-right: M1 (top) + M6 (bottom)
- Front-left: M2 (top) + M5 (bottom)
- Rear-left: M3 (top) + M8 (bottom)
- Rear-right: M4 (top) + M7 (bottom)

> Note: the bottom motors (M5–M8) do **not** run in the same order as the top set.
> M5 sits under the front-left arm, not on M1's arm.

---

## 1211 — Dodecarotor Coaxial (6 arms × 2)

| Motor | Position (PX,PY,PZ) | Arm | Top/Bottom | PX4 / QGC | RealFlight |
|-------|---------------------|-----|-----------|-----------|------------|
| 1 | (0, 0.5, +0.05) | right | bottom | CCW ✅ | **CW** |
| 2 | (0, -0.5, +0.05) | left | bottom | CW ⬜ | **CCW** |
| 3 | (0.433, -0.25, +0.05) | front-left | bottom | CCW ✅ | **CW** |
| 4 | (-0.433, 0.25, +0.05) | rear-right | bottom | CW ⬜ | **CCW** |
| 5 | (0.433, 0.25, +0.05) | front-right | bottom | CW ⬜ | **CCW** |
| 6 | (-0.344, -0.25, +0.05) | rear-left | bottom | CCW ✅ | **CW** |
| 7 | (0, 0.5, -0.05) | right | top | CW ⬜ | **CCW** |
| 8 | (0, -0.5, -0.05) | left | top | CCW ✅ | **CW** |
| 9 | (0.433, -0.25, -0.05) | front-left | top | CW ⬜ | **CCW** |
| 10 | (-0.433, 0.25, -0.05) | rear-right | top | CCW ✅ | **CW** |
| 11 | (0.433, 0.25, -0.05) | front-right | top | CCW ✅ | **CW** |
| 12 | (-0.344, -0.25, -0.05) | rear-left | top | CW ⬜ | **CCW** |

Same-arm pairs (bottom + top): (M1,M7), (M2,M8), (M3,M9), (M4,M10), (M5,M11),
(M6,M12).

> The `PX -0.344` on the rear-left arm (M6/M12) is asymmetric versus `0.433` — that
> is inherited from stock PX4 geometry and left as-is.

---

## Other VTOL

### 1215 — Tiltrotor VTOL

Lift/tilt rotors = **Motor 1–4** (Quad-X pattern):

| Motor | Position (PX,PY) | Arm | PX4 / QGC | RealFlight |
|-------|------------------|-----|-----------|------------|
| 1 | (1, 1) | front-right | CCW ✅ | **CW** |
| 2 | (-1, -1) | rear-left | CCW ✅ | **CW** |
| 3 | (1, -1) | front-left | CW ⬜ | **CCW** |
| 4 | (-1, 1) | rear-right | CW ⬜ | **CCW** |

Plus tilt servos + control surfaces (Motor 5–12 = servos). All servos follow
rule #4.

### 1216 — Tailsitter VTOL

| Motor | Position (PX,PY) | Arm | PX4 / QGC | RealFlight |
|-------|------------------|-----|-----------|------------|
| 1 | (0, -0.2) | left | CW ⬜ | **CCW** |
| 2 | (0, 0.2) | right | CCW ✅ | **CW** |

Plus 2 elevons (Motor 3–4 = servos) → rule #4.

### 1212 — Tricopter Y

| Motor | Position (PX,PY) | Arm | PX4 / QGC | RealFlight |
|-------|------------------|-----|-----------|------------|
| 1 | (0.25, 0.433) | front-right | CCW ✅ | **CW** |
| 2 | (0.25, -0.43) | front-left | CCW ✅ | **CW** |
| 3 | (-0.5, 0) | rear | CCW ✅ | **CW** |

> **Tricopter is special:** yaw comes from the **tail tilt servo** (Motor 4 =
> servo), not from motor-torque differential. That is why all three motors are
> modelled as CCW in PX4 (no `KM` set). The tail-tilt servo direction follows
> rule #4 and **must** be verified (yaw stick → tail tilts the correct way).

---

## Fixed-wing (not a CW/CCW motor map)

The models below have **no** multirotor motor-direction table. "Spin direction" is
not relevant the way it is for lift motors; what matters is the reverse channel of
the control surfaces (rule #4) and the thrust direction of the single motor.

| Model | Main actuators | Notes |
|-------|----------------|-------|
| 1200 — Plane | 1 motor + aileron/elevator/rudder | Single motor: ensure forward thrust. Surfaces: rule #4. |
| 1213 — Flying Wing | 1 motor + 2 elevons | Elevons (CS type 5/6) mix pitch+roll; check deflection per stick. |
| 1214 — Plane A-Tail | 1 motor + aileron + V-tail + flap | 7 control surfaces; check each one (rule #4). |

---

## Helicopters (channel-function map, not CW/CCW)

Helicopters have **no** CW/CCW motor table either. What matters is which
PX4 output **function** lands on which RealFlight channel, and the reverse of each
channel. The bridge is a pure identity pipe (RF channel `N` = PX4 output `N`), so
the ordering lives in `PWM_MAIN_FUNC<N>`; the shipped order reproduces ArduPilot's
heli channel order so an ArduPilot RealFlight model flies with no re-mapping.

### 1203 — Single-rotor Helicopter (`CA_AIRFRAME 11`)

PX4 always computes the CCPM mix internally; the allocator's outputs are fixed:
Motor1 = main rotor (RSC), Servo1 = tail, Servo2–4 = the three swash servos. There
are **two ways** to drive a RealFlight model, and you pick one — mixing must live on
exactly **one** side of the link, never both.

**Path A — direct-swash (shipped default).** The RealFlight model is **non-mixed**
(CCPM turned OFF); PX4 sends three finished swash-servo positions.

| RF ch | PX4 output | Carries | RF model |
|-------|-----------|---------|----------|
| 1 | Servo2 (`202`) | swash servo 1 (300°) | swash servo 1, direct |
| 2 | Servo3 (`203`) | swash servo 2 (60°) | swash servo 2, direct |
| 3 | Servo4 (`204`) | swash servo 3 (180°) | swash servo 3, direct |
| 4 | Servo1 (`201`) | tail rotor pitch (yaw) | tail servo |
| 8 | Motor1 (`101`) | main rotor throttle / RSC | engine / RSC |

> Do **not** reverse a single swash channel here — the three are a coordinated
> triple; flipping one tilts the swash wrong about one axis and reads as
> roll/pitch cross-coupling. Fix swash direction in the model's servo setup or via
> `CA_SP0_ANG*`. Only the **tail** (ch4) is a normal reversible channel.

**Path B — HeliDemix (for a CCPM model you cannot un-mix, e.g. a stock ArduPilot
RealFlight heli).** Add `"HeliDemix"` to `Options` in `heli.json`. The bridge
un-mixes PX4's swash back into separate axes; the model **keeps** its own CCPM.

| RF ch | PX4 output | Carries after demix | RF model (ArduPilot-style) |
|-------|-----------|---------------------|-----------------------------|
| 1 | Servo2 (`202`) | **roll** (cyclic) | aileron servo *(reversed)* |
| 2 | Servo3 (`203`) | **pitch** (cyclic) | elevator servo *(reversed)* |
| 3 | Servo4 (`204`) | **collective** | collective / "pitch" servo |
| 4 | Servo1 (`201`) | **yaw** | tail servo |
| 8 | Motor1 (`101`) | main rotor throttle / RSC | engine servo |

> With HeliDemix ON, ch1/ch2/ch3 are **independent axes** — reversing one is fine
> (like a plane's aileron/elevator), and the swash-triple warning above does not
> apply. Reverse on **one** side only: keep the model's existing ch1/ch2 reverse,
> **or** move it to PX4 with `PWM_MAIN_REV` (bit `N-1`; ch1+ch2 = `1+2` = `3`) —
> not both. Verify each axis in the sim and flip whichever is backwards.

Both paths, common:
- **Main-rotor spin:** `CA_HELI_YAW_CCW 0` = clockwise seen from above (the usual
  RealFlight heli). If yaw runs away on liftoff, set it to `1`. If the tail fights
  the commanded direction, reverse ch4.
- **Model prep (both paths):** turn OFF the tail heading-hold/rate gyro (it fights
  PX4's yaw loop), and flatten the model's throttle and collective curves (PX4 owns
  them via `CA_HELI_THR_C*` / `CA_HELI_PITCH_C*`), plus remove cyclic/rudder
  expo/dual-rate. The **difference** is CCPM: Path A turns it **off**, Path B leaves
  it **on**.

### 1217 — Coaxial Helicopter (`CA_AIRFRAME 12`)

**No tail rotor.** Yaw = **differential** rotor throttle, collective = **common**
rotor throttle (`rotor_CW = throttle − yaw`, `rotor_CCW = throttle + yaw`), and the
swash carries **cyclic only** (roll/pitch) — there is no collective term on the
swash. So the single-rotor `CA_HELI_YAW_*` / `CA_HELI_THR_C*` / `CA_HELI_PITCH_C*`
params are **not read** under `CA_AIRFRAME 12`. Allocator outputs are fixed:
Motor1 = CW rotor, Motor2 = CCW rotor, Servo1–3 = swash (note: swash starts at
Servo1 here, not Servo2, because no tail consumes Servo1).

**Path A — direct-swash (shipped default).** Model's CCPM OFF; PX4 sends three
finished swash positions.

| RF ch | PX4 output | Carries | RF model |
|-------|-----------|---------|----------|
| 1 | Servo1 (`201`) | swash servo 1 (300°) | swash servo 1, direct |
| 2 | Servo2 (`202`) | swash servo 2 (60°) | swash servo 2, direct |
| 3 | Servo3 (`203`) | swash servo 3 (180°) | swash servo 3, direct |
| 7 | Motor1 (`101`) | clockwise main rotor | CW rotor |
| 8 | Motor2 (`102`) | counter-clockwise main rotor | CCW rotor |

(`ch4–6` idle at 0.5.)

**Path B — HeliDemix.** Add `"HeliDemix"` to `Options` in `helicoax.json`. It
un-mixes the **swash triple only** (rf0–2) into roll/pitch/collective; it does not
touch the rotor channels.

| RF ch | PX4 output | Carries after demix | RF model |
|-------|-----------|---------------------|----------|
| 1 | Servo1 (`201`) | **roll** (cyclic) | aileron servo |
| 2 | Servo2 (`202`) | **pitch** (cyclic) | elevator servo |
| 3 | Servo3 (`203`) | **collective** (swash artefact — usually ignored) | often unused |
| 7 | Motor1 (`101`) | clockwise main rotor | CW rotor |
| 8 | Motor2 (`102`) | counter-clockwise main rotor | CCW rotor |

> On a coaxial the demix "collective" is a **swash artefact** — real collective is
> common rotor throttle on ch7/ch8, not on the swash. So Path B is rarely needed:
> use it **only** if your coaxial model has a CCPM swash whose mixing you cannot
> disable, and expect ch3 to be ignored by the model. Otherwise leave demix off.

Common to both paths:
- **The two rotors must be independently drivable and spin in OPPOSITE directions**
  — coaxial yaw *is* the difference between them. Tie them to one channel and PX4
  has zero yaw authority.
- **Yaw wrong / spins up on liftoff:** swap the two rotor channels (ch7 ↔ ch8) or
  reverse the model's rotor spin directions — **not** the swash. (`PWM_MAIN_REV`
  for ch7 = bit 6 = `64`.)
- **Swash reverse:** same caution as the single-rotor Path A — never flip a single
  swash channel; fix it in the model's servo setup or via `CA_SP0_ANG*`.
- **Model prep:** gyro off; flatten the model's rotor throttle / collective curves
  (PX4 owns collective as common rotor throttle); remove cyclic expo/dual-rate;
  CCPM off for Path A, left on for Path B.

> **Note:** changing `Options` in `heli.json` or `helicoax.json` (e.g. enabling
> HeliDemix) alters the shipped default for **everyone** and would break a
> direct-swash model. Treat it as a local choice for your own CCPM model; do not
> commit it as the repo default.

---

## Quick setup flow (multirotor models)

1. **QGC → Actuators:** verify each motor's CCW column against the "PX4 / QGC"
   column for the model. It normally already matches the airframe default — just
   confirm, do not change it.
2. **RealFlight → model setup:** place each motor on the correct arm (mind the
   Position/Arm column), and set its spin to the "RealFlight" column (opposite of
   QGC).
3. **In the simulator:** arm gently and give small pitch/roll/yaw, then watch the
   RealFlight model (or the QGC Actuators test sliders) — confirm each motor
   responds correctly and each coaxial pair counter-rotates. This is a purely
   visual check; there is no hardware and nothing to spin down.
4. If a motor is reversed → flip that motor's direction **on the RealFlight side**
   only. **Do not** change `KM` in PX4.

> **HITL only:** if you are driving a real flight-controller board and real ESCs
> (the `.hil` airframes), do the same check with **propellers off / no flight
> power** — there it is a genuine safety step, not just a visual one. See
> [HITL.md](../HITL.md).
