# Copyright and licensing

Copyright © 2026 Evangels Brilliant Dasmasela.

Released under the **GNU General Public License v3 or later** (see [LICENSE](LICENSE)),
so that anyone who finds it useful can take it further.

## What you may do with it

Use it, study it, change it, and pass it on — commercially or not, to as many people
as you like. That is what GPLv3 grants, and nothing here narrows it.

One obligation comes with it: if you distribute this bridge, or anything built from
it, you must ship the corresponding source under GPLv3 too, so whoever receives it
keeps the same freedoms. Running it privately, or modifying it for your own use,
carries no such requirement.

Installing it does not affect the licence of your PX4 checkout. The bridge is a
separate executable that talks to PX4 over a MAVLink socket, built as an
ExternalProject rather than linked into the PX4 binary — it is never part of the
PX4 binary, so nothing in PX4 becomes a derivative of it.

## What this project adds

Written for this project, with no upstream counterpart: `hitl_run.sh` (PX4 ships no
HITL runner), the model JSONs and the channel-map schema they define, `FA_check.py`
and `get_FAbridge_params.py`, the installer and its supporting scripts, and the
documentation. `battery_link.{h,cpp}` is original too, apart from the internal-resistance
estimator noted in the table; neither ArduPilot nor PX4-FlightGear-Bridge has a
battery link of any kind.

Reworked substantially inside files that started from an upstream source — see the
provenance table for what each began as: the transports, per-message decimation,
`fields_updated` sub-rating and dead-link policy in `px4_communicator.*`; the
absolute-clock design, realtime-factor monitor and reconnect handling in
`flightaxis_bridge.cpp`; the rangefinder path and sensor synthesis in
`vehicle_state.*`; and the four airframes with their control-allocation and
parameter configuration.

## Third-party notices — required, do not remove

The notices in the source headers, and the provenance table further down, are not a
disclaimer of ownership; they are a licence obligation.
BSD-3-Clause clause 1 requires that its copyright notices be retained in
redistributions, and GPLv3 §5 requires preserving the notices on a derivative work.
Removing them would make this repository non-compliant and would put its distributor
at risk — so they stay, and they are what makes the copyright claim above credible
rather than overbroad.

## Why GPLv3 and not BSD-3

The bridge is not an independent implementation of the FlightAxis protocol. Its
SOAP client, its physics-time handling, and its RealFlight→NED conversions are
**literal ports of ArduPilot's `libraries/SITL/SIM_FlightAxis.{h,cpp}`**, which is
GPLv3. That fidelity is the point — RealFlight's conventions are internally
inconsistent (position swapped, velocity not; ground accelerometer unusable), and
ArduPilot's code is the only public source that gets them right. Deriving from it
means this work is a derivative work, and GPLv3 propagates.

The rest of the integration follows PX4's BSD-3-Clause bridge pattern, but a
combined work containing GPLv3 code is distributed under GPLv3. BSD-3-Clause is
GPL-compatible, so the PX4-derived files may be combined.

**Where the notices live.** Files that carry a BSD-3-Clause notice upstream keep
it verbatim in their own header. Two files come from upstream sources that ship
them *without* any header notice — `cmake/FindMAVLink.cmake`, taken verbatim from
PX4-FlightGear-Bridge, and `Tools/simulation/flightaxis/sitl_run.sh`, which began
as PX4's FlightGear runner but has since been rewritten far enough that only 21
of its 177 lines still match upstream. We have not added a notice to either,
because inventing a copyright line that the copyright holder never wrote would
misstate the provenance. Their origin and licence are recorded in the table below
instead, which is where you should look for those two.

## Provenance by file

`Tools/simulation/flightaxis/flightaxis_bridge/src/`

| File | Origin | Original licence |
|---|---|---|
| `fa_communicator.{h,cpp}` | Ported from ArduPilot `SIM_FlightAxis.{h,cpp}` — socket/creator-thread design, SOAP request bodies, reply parser key table and scan, startup and reconnect logic | GPLv3, © ArduPilot Dev Team |
| `vehicle_state.{h,cpp}` | Frame conversions, ground-accelerometer override, pitot derivation and rangefinder ported from `SIM_FlightAxis.cpp`; sensor-message synthesis follows PX4-FlightGear-Bridge `vehicle_state.cpp`. The single-channel RPM selection (main rotor first, prop fallback, else zero) is **original** — upstream fills two channels unconditionally (`rpm[0] = m_heliMainRotorRPM; rpm[1] = m_propRPM; motor_mask = 3`) and has no fallback | GPLv3, © ArduPilot Dev Team; BSD-3, © 2020 ThunderFly s.r.o. |
| `flightaxis_bridge.cpp` | Three-branch physics-time handling (restart / duplicate-frame extrapolation / glitch compensation), `Rev4Servos` and `HeliDemix` ported from `SIM_FlightAxis.cpp` `update()` and `exchange_data()`; overall structure follows `flightgear_bridge.cpp` | GPLv3, © ArduPilot Dev Team; BSD-3, © 2020 ThunderFly s.r.o. |
| `px4_communicator.{h,cpp}` | **Adapted** from PX4-FlightGear-Bridge. Added: per-message decimation, `fields_updated` sub-rating, a non-blocking receive drain, the serial and UDP transports and HITL message profile, and the dead-link policy | BSD-3, © 2020 ThunderFly s.r.o. |
| `battery_link.{h,cpp}` | Original for this project — the separate UDP link, the propulsion-class detection and the fuel/pack accounting have no upstream counterpart. `updateInternalResistance()` is **ported** from PX4's `Battery::updateInternalResistanceEstimation` (`src/lib/battery/battery.cpp`), same recursive-least-squares formulation and constants | BSD-3, © 2019–2021 PX4 Development Team |
| `geo_mag_declination.{cpp,h}` | **Verbatim** from PX4-FlightGear-Bridge (which itself carries it verbatim from the MAV GEO Library) | BSD-3, © 2014 MAV GEO Library (MAVGEO) |

`Tools/simulation/flightaxis/flightaxis_bridge/cmake/FindMAVLink.cmake` — verbatim
from PX4-FlightGear-Bridge (BSD-3, © 2020 ThunderFly s.r.o.), byte-for-byte
including its lack of a header notice: the upstream file carries none, so there
is none here either and this entry is the notice of record.

`Tools/simulation/flightaxis/sitl_run.sh`, `flightaxis_bridge/CMakeLists.txt`,
`src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake` and the
`ROMFS` airframes follow the corresponding PX4 files (BSD-3, © PX4 Development Team).

`Tools/simulation/flightaxis/hitl_run.sh` is original work for this project and
carries its own GPLv3 header. It mirrors `sitl_run.sh`'s structure but has no PX4
counterpart to follow: PX4 ships no HITL runner.

Model JSONs, `FA_check.py`, `get_FAbridge_params.py`, `install.sh`, `uninstall.sh`,
`scripts/detect-px4.sh` and the documentation are original work for this project.

## Upstream projects

- **ArduPilot** — <https://github.com/ArduPilot/ardupilot>, GPLv3.
  `libraries/SITL/SIM_FlightAxis.{h,cpp}` is the ground truth for §6–§8 of the spec.
- **PX4-FlightGear-Bridge** — <https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge>,
  BSD-3-Clause, © ThunderFly s.r.o. The integration template.
- **PX4-Autopilot** — <https://github.com/PX4/PX4-Autopilot>, BSD-3-Clause.

## Contributing

Patches are welcome under the same terms. Contributors keep copyright in what they
write; add yourself to the header of any file you substantially change, and to the
list above if you add something original.
