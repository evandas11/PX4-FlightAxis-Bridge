# Copyright and licensing

This project is released under the **GNU General Public License v3 or later**
(see [LICENSE](LICENSE)).

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
GPL-compatible, so the PX4-derived files may be combined; their original notices
are retained below and in the file headers.

## Provenance by file

`Tools/simulation/flightaxis/flightaxis_bridge/src/`

| File | Origin | Original licence |
|---|---|---|
| `fa_communicator.{h,cpp}` | Ported from ArduPilot `SIM_FlightAxis.{h,cpp}` — socket/creator-thread design, SOAP request bodies, reply parser key table and scan, startup and reconnect logic | GPLv3, © ArduPilot Dev Team |
| `vehicle_state.{h,cpp}` | Frame conversions, ground-accelerometer override, pitot derivation and rangefinder ported from `SIM_FlightAxis.cpp`; sensor-message synthesis follows PX4-FlightGear-Bridge `vehicle_state.cpp` | GPLv3, © ArduPilot Dev Team; BSD-3, © 2020 ThunderFly s.r.o. |
| `flightaxis_bridge.cpp` | Three-branch physics-time handling (restart / duplicate-frame extrapolation / glitch compensation), `Rev4Servos` and `HeliDemix` ported from `SIM_FlightAxis.cpp` `update()` and `exchange_data()`; overall structure follows `flightgear_bridge.cpp` | GPLv3, © ArduPilot Dev Team; BSD-3, © 2020 ThunderFly s.r.o. |
| `px4_communicator.{h,cpp}` | **Adapted** from PX4-FlightGear-Bridge (message decimation and a non-blocking receive drain added) | BSD-3, © 2020 ThunderFly s.r.o. |
| `geo_mag_declination.{cpp,h}` | **Verbatim** from PX4-FlightGear-Bridge | BSD-3, © 2020 ThunderFly s.r.o. |

`Tools/simulation/flightaxis/flightaxis_bridge/cmake/FindMAVLink.cmake` — verbatim
from PX4-FlightGear-Bridge (BSD-3, © 2020 ThunderFly s.r.o.).

`Tools/simulation/flightaxis/sitl_run.sh`, `flightaxis_bridge/CMakeLists.txt`,
`src/modules/simulation/simulator_mavlink/sitl_targets_flightaxis.cmake` and the
`ROMFS` airframes follow the corresponding PX4 files (BSD-3, © PX4 Development Team).

Model JSONs, `FA_check.py`, `get_FAbridge_params.py`, `install.sh`, `uninstall.sh`,
`scripts/*` and the documentation are original work for this project.

## Upstream projects

- **ArduPilot** — <https://github.com/ArduPilot/ardupilot>, GPLv3.
  `libraries/SITL/SIM_FlightAxis.{h,cpp}` is the ground truth for §6–§8 of the spec.
- **PX4-FlightGear-Bridge** — <https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge>,
  BSD-3-Clause, © ThunderFly s.r.o. The integration template.
- **PX4-Autopilot** — <https://github.com/PX4/PX4-Autopilot>, BSD-3-Clause.

## What this means for you

You may use, study, modify and redistribute this bridge under GPLv3. If you
distribute a modified version, or software that incorporates it, you must make
the corresponding source available under the same terms.

This licence covers **only the files in this repository**. It does not change the
licence of PX4-Autopilot: the bridge is a standalone executable that talks to PX4
over MAVLink on a TCP socket, exactly as the FlightGear and JSBSim bridges do, and
it is compiled by PX4's build system as an ExternalProject rather than linked into
the PX4 binary. Installing it does not relicense your PX4 checkout.
