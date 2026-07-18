# PX4-FlightAxis-Bridge

**RealFlight (FlightAxis Link) as a PX4 SITL simulator** — integrated the same way as the
FlightGear bridge: files live in the PX4 tree, get compiled by the PX4 make system, and
launch with one `make` command.

```bash
# RealFlight running on a Windows box (FlightAxis Link enabled in RealFlight settings)
PX4_FLIGHTAXIS_IP=192.168.10.1 make px4_sitl_nolockstep flightaxis_plane
# QGC connects on UDP 14550 as usual
```

Full design rationale, frame conversions, and timing logic:
[`FLIGHTAXIS_PX4_INTEGRATION.md`](FLIGHTAXIS_PX4_INTEGRATION.md) (the spec — §6/§7 are
verified against ArduPilot `SIM_FlightAxis.cpp`).

---

## ⚠️ PENGINGAT SINKRONISASI / SYNC REMINDER

> **ID:** Repo ini adalah *mirror* dari file yang hidup di dalam source tree PX4
> (`/home/evangels/PX4-Autopilot`). Kalau kamu mengedit file langsung di tree PX4,
> **WAJIB update juga file yang sama di folder ini** (dan sebaliknya) — kalau tidak,
> repo GitHub ini jadi basi dan tidak bisa dipakai orang lain.
> Jalankan `./scripts/sync-from-px4.sh` setelah mengedit di tree PX4,
> atau `./scripts/sync-to-px4.sh` setelah mengedit di repo ini.
>
> **EN:** This repo mirrors files that live inside a PX4 source tree. If you edit a file
> directly in the PX4 tree, **you must update the copy here too** (and vice versa).
> Use `./scripts/sync-from-px4.sh` / `./scripts/sync-to-px4.sh`.

---

## Repository layout

Paths are identical to where the files go inside `PX4-Autopilot/`, so installation is a
straight copy:

```
Tools/simulation/flightaxis/
├── sitl_run.sh                          # runner invoked by the make target
└── flightaxis_bridge/
    ├── CMakeLists.txt
    ├── FA_check.py                      # sanity-ping RealFlight :18083 before start
    ├── get_FAbridge_params.py           # models/<name>.json → bridge argv
    ├── cmake/FindMAVLink.cmake
    ├── models/{plane,quad,quadplane,heli}.json   # RealFlight channel maps
    └── src/
        ├── flightaxis_bridge.cpp        # main loop + ArduPilot 3-branch timing (§7)
        ├── fa_communicator.{h,cpp}      # SOAP client, port of ArduPilot socket logic (§8.1)
        ├── vehicle_state.{h,cpp}        # RF→NED conversions (§6) + sensor synthesis
        ├── px4_communicator.{h,cpp}     # TCP 4560 HIL link (from PX4-FlightGear-Bridge)
        └── geo_mag_declination.{h,cpp}  # WMM tables (from PX4-FlightGear-Bridge)

src/modules/simulation/simulator_mavlink/
└── sitl_targets_flightaxis.cmake        # make-target registration (PX4 v1.16 pattern)
    # + one include() line added to that directory's CMakeLists.txt — see Install step 3

ROMFS/px4fmu_common/init.d-posix/airframes/
├── 1200_flightaxis_plane
├── 1201_flightaxis_quad
├── 1202_flightaxis_quadplane
└── 1203_flightaxis_heli
    # + four entries added to that directory's CMakeLists.txt — see Install step 3
```

## Install into a PX4 tree

Tested against **PX4 v1.16.0**. (Older v1.13–v1.15 trees use
`platforms/posix/cmake/sitl_target.cmake` instead of per-sim cmake files — see spec §3.)

1. Copy `Tools/`, `src/`, and `ROMFS/` from this repo over your `PX4-Autopilot/` checkout
   (or run `./scripts/sync-to-px4.sh`).
2. In `PX4-Autopilot/src/modules/simulation/simulator_mavlink/CMakeLists.txt`, add next to
   the flightgear include:
   ```cmake
   include(sitl_targets_flightaxis.cmake)
   ```
3. In `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`, register
   the four `120x_flightaxis_*` airframes the same way the flightgear ones are listed.
4. Build: `make px4_sitl_nolockstep flightaxis_plane`

## RealFlight setup (once per aircraft)

- Enable **FlightAxis Link** in RealFlight (listens on TCP 18083). Wired network or
  same-host only — WiFi cannot hold the ~250 Hz SOAP rate.
- In the RealFlight aircraft editor: strip expo/mixes/gyros, max servo speed, one channel
  per actuator. Tridge's ArduPilot RealFlight model collection is directly reusable.
- The QGC **Actuators** geometry must match the `"px4"` indices in the corresponding
  `models/<name>.json`.

## Adding a new aircraft

Three steps (same workflow as the FlightGear bridge):
1. New `models/<name>.json` (channel map — spec §5).
2. Add the model name to the `models` list in `sitl_targets_flightaxis.cmake`.
3. New airframe script `12xx_flightaxis_<name>` (+ CMakeLists entry).

## Credits / references

- ArduPilot `SIM_FlightAxis.{h,cpp}` — ground truth for conversions, timing, and the
  SOAP socket/reconnect logic.
- [PX4-FlightGear-Bridge](https://github.com/PX4/PX4-FlightGear-Bridge) (ThunderFly
  s.r.o.) — the integration template; `px4_communicator` and `geo_mag_declination` are
  reused from it (BSD-3-Clause).

## License

BSD-3-Clause (same as PX4 and the FlightGear bridge components this reuses).
