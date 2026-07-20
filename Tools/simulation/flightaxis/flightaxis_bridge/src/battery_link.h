/****************************************************************************
 *
 * This file is part of the PX4-FlightAxis-Bridge project.
 * Copyright (c) 2026 Evangels Brilliant Dasmasela
 *
 * The recursive-least-squares internal-resistance estimator in this file is
 * ported from PX4-Autopilot src/lib/battery/battery.cpp (Copyright (c)
 * 2019-2021 PX4 Development Team, BSD-3-Clause). BSD-3-Clause is GPL-compatible,
 * so both notices coexist in this combined work:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 ****************************************************************************/

/**
 * @file battery_link.h
 *
 * RealFlight battery / fuel telemetry -> PX4 `battery_status`.
 *
 * WHY THIS IS A SEPARATE UDP SOCKET AND NOT PART OF THE SIMULATOR LINK
 * ===================================================================
 * The obvious implementation - add BATTERY_STATUS to the messages the bridge
 * already sends down the simulator link on TCP 4560 - does not work, and the
 * reason is worth recording so nobody tries it again:
 *
 *   SimulatorMavlink::handle_message() (SimulatorMavlink.cpp) switches on
 *   exactly ten message ids: HIL_SENSOR, HIL_OPTICAL_FLOW, ODOMETRY,
 *   VISION_POSITION_ESTIMATE, DISTANCE_SENSOR, HIL_GPS, RC_CHANNELS,
 *   LANDING_TARGET, HIL_STATE_QUATERNION and RAW_RPM. There is no
 *   BATTERY_STATUS case and no default case. A BATTERY_STATUS arriving on
 *   TCP 4560 is parsed, matched against nothing, and silently dropped.
 *   simulator_mavlink only ever SUBSCRIBES to battery_status (to fill in
 *   esc_status voltages); it never publishes it.
 *
 * The module that does handle BATTERY_STATUS is the normal MAVLink receiver -
 * MavlinkReceiver::handle_message_battery_status(), reached from the
 * MAVLINK_MSG_ID_BATTERY_STATUS case in mavlink_receiver.cpp. That runs on
 * PX4's ordinary MAVLink links, not on the simulator link, so the bridge needs
 * a second socket to reach it.
 *
 * The target is the API/offboard link, UDP 14580+instance, started by
 * px4-rc.mavlink as:
 *      mavlink start -x -u $((14580+px4_instance)) -r 4000000 -f -m onboard \
 *                    -o $((14540+px4_instance))
 * Two properties of that link matter:
 *   - Its remote is PINNED by -o. PX4 therefore does NOT retarget its output
 *     stream at whoever sends to it, so this socket costs one sendto() per
 *     update and provokes ZERO return traffic. Measured: 0 packets / 0 bytes
 *     back after a 10-packet burst. That is what makes it safe to do from the
 *     ~800 Hz exchange loop.
 *   - It is the conventional companion-computer injection port. The GCS link
 *     (18570) is deliberately NOT used: it has no -o, so it latches its
 *     partner address from the first packet it receives, and sending there
 *     would hijack the stream away from a real QGroundControl.
 *
 * TWO THINGS THAT WILL SILENTLY EAT THE MESSAGE
 * ---------------------------------------------
 *  1. MAVLINK v1. BATTERY_STATUS is id 147 and encodes fine in v1, but PX4
 *     drops it. Everything here is packed through a v2 channel. This cost an
 *     hour to find: COMMAND_LONG from the same socket was processed normally
 *     while BATTERY_STATUS vanished, which reads as a handler bug rather than
 *     a framing one.
 *  2. The handler's own gate:
 *         if ((msg->sysid != mavlink_system.sysid) ||
 *             (msg->compid == mavlink_system.compid)) return;
 *     The source system id must EQUAL PX4's MAV_SYS_ID and the component id
 *     must DIFFER from PX4's (1). rcS sets `param set MAV_SYS_ID
 *     $((px4_instance+1))`, so the sysid used here is instance+1 - which is
 *     also why this has to know the instance.
 *
 * HITL IS NOT SUPPORTED AND CANNOT BE
 * -----------------------------------
 * On a real board, MavlinkReceiver::handle_message_hil_sensor() publishes a
 * HARDCODED battery on EVERY HIL_SENSOR it receives:
 *      hil_battery_status.voltage_v = 16.0f;
 *      hil_battery_status.current_a = 10.0f;
 *      hil_battery_status.remaining = 0.70;
 * The bridge sends HIL_SENSOR at ~250 Hz, so a real reading injected at 2 Hz
 * would be overwritten roughly 125 times between updates. There is no
 * parameter that disables it. Getting real battery telemetry onto a HITL board
 * requires patching PX4 itself, which this project deliberately does not do -
 * install.sh only ever ADDS files and splices two CMakeLists.txt registrations.
 * So this link is started only for the SITL (tcp-server) transport.
 */

#ifndef BATTERY_LINK_H
#define BATTERY_LINK_H

#include <cstdint>

#include "fa_communicator.h"

class BatteryLink
{
public:
	/*
	 * Open the UDP socket toward PX4's API/offboard MAVLink link.
	 *
	 * instance: the PX4 SITL instance id. Selects both the destination port
	 *           (14580+instance) and the source system id (instance+1), for
	 *           the reasons in the file header.
	 *
	 * Returns false if the socket could not be created. A false return is NOT
	 * fatal to the bridge: the simulation runs perfectly well without battery
	 * telemetry, so the caller logs and carries on.
	 */
	bool init(int instance);

	/*
	 * Feed one FlightAxis frame.
	 *
	 * Safe and cheap to call on every exchange: the send is gated on
	 * now_us against a 2 Hz interval, so at the measured ~800 Hz exchange
	 * rate this is 399 predictable-branch returns out of every 400 calls.
	 * The gate is TIME-based, not frame-count based, deliberately - a
	 * frame-count gate would change rate with the exchange rate and stall
	 * completely whenever RealFlight's physics clock stalls.
	 *
	 * now_us: wall-clock microseconds (the bridge's micros()). Wall clock,
	 *         not the physics clock, so a paused/stalled RealFlight still
	 *         keeps the battery topic alive rather than letting it go stale
	 *         and trip a "battery timeout" style check.
	 */
	void update(const FAState &fa, uint64_t now_us);

	void close();

	/*
	 * Ask PX4's estimator to hard-reset its position to a known point.
	 *
	 * Sends MAV_CMD_EXTERNAL_POSITION_ESTIMATE (43003) over the same onboard
	 * MAVLink link the battery uses. EKF2 handles it in EKF2.cpp and, when the
	 * claimed accuracy is under a metre, takes the hard-reset branch in
	 * Ekf::resetGlobalPosToExternalObservation() rather than fusing it - the
	 * one its own comment describes as letting a caller "send hard resets at
	 * any time".
	 *
	 * This exists for a RealFlight respawn. The simulator teleports the model
	 * back to where it entered; the bridge reports that faithfully, but EKF2
	 * has converged state from the flight just ended and treats the jump as a
	 * lying GPS rather than a moved aircraft. There is no HIL message for "the
	 * vehicle has been repositioned", so this is the nearest thing PX4 offers.
	 *
	 * EKF2 gates it on the vehicle dead-reckoning, or being on the ground and
	 * not fusing GNSS, so acceptance is not guaranteed on the first attempt.
	 * Returns whether the datagram was sent, NOT whether PX4 acted on it.
	 */
	bool requestPositionReset(double lat_deg, double lon_deg, double accuracy_m, uint64_t now_us);

	bool active() const { return _fd >= 0; }

private:
	/*
	 * PROPULSION CLASS - decided from the data, never from a parameter.
	 *
	 * Electric and Fuel are LATCHED once seen. Synthetic is only ever a
	 * REPORTED label, never a latched class: a model that has proven it has a
	 * pack or a tank must not be able to fall back to "nominal full" just
	 * because one frame came back odd. That fallback is precisely how an
	 * empty fuel tank used to report 100%.
	 */
	enum class Source {
		None,		// nothing decided yet
		Electric,	// RealFlight reported a plausible propulsion pack voltage
		Fuel,		// internal combustion: a fuel tank exists
		Synthetic,	// reported only: model has shown neither pack nor tank
	};

	// Adopt a new propulsion class and drop every per-class accumulator, so a
	// model swap mid-session cannot carry a stale cell count or tank size over.
	void adopt(Source s);

	// Recursive least squares over (current, voltage) for this pack's open
	// circuit voltage and internal resistance. Ported from PX4's Battery class
	// so a simulated pack ages the same way a real one behind a power module
	// does - see the comment on the definition.
	void updateInternalResistance(double voltage_v, double current_a);

	void send(double voltage_v, double current_a, double remaining,
		  double discharged_mah, int cells, uint8_t batt_type,
		  uint64_t now_us);

	int _fd{-1};
	int _instance{0};
	uint8_t _sysid{1};

	uint64_t _last_send_us{0};
	uint64_t _last_update_us{0};

	// electric: inferred once from a PLAUSIBLE measured voltage, so voltages[]
	// can be filled per-cell. Deliberately NOT written by the synthetic path -
	// a synthetic 4S guess latched here would divide a real 3S pack by 4 and
	// report it flat, which PX4 turns straight into an EMERGENCY.
	int _cell_count{0};

	// coulomb counter, mAh
	double _discharged_mah{0.0};

	// Low-passed open-circuit per-cell voltage driving `remaining`. Negative
	// means "not seeded yet" - see the sag block in BatteryLink::update().
	double _per_cell_filt{-1.0};

	// Estimated per-cell internal resistance, and the RLS state behind it.
	// Seeded from PX4's own initial guess and then driven by the data: holding
	// it fixed is what pinned `remaining` at 1.0, because a fixed 5 mOhm/cell
	// becomes a 1.35 V/cell correction at the 270 A this airframe draws, far
	// wider than the 0.90 V/cell window the SoC map spans.
	// 0.005 is PX4's own initial per-cell guess (battery.h:195, R_DEFAULT), kept
	// literal here because the .cpp constant is not visible from this header.
	double _cell_r_internal{0.005};
	bool   _rls_init{false};
	double _rls_ocv{0.0};		// pack open-circuit voltage estimate
	double _rls_r{0.0};		// pack internal resistance estimate
	double _rls_p00{0.0}, _rls_p01{0.0}, _rls_p10{0.0}, _rls_p11{0.0};
	double _rls_p_norm{0.0};

	// fuel: largest reading seen, taken as "full tank". Re-armed in BOTH
	// directions rather than latched. Upwards immediately, so a refuel or an
	// aircraft reset does not leave the fraction pinned below 1.0 forever.
	// Downwards on a sustained step UP to a level still under the reference,
	// which is the signature of a smaller tank being loaded - adopt() cannot
	// cover that, because swapping one IC model for another never changes the
	// class. Without it a 10 oz tank behind a 40 oz one reads 25% when full.
	double _fuel_full_oz{0.0};

	// fuel: previous frame's raw reading, and the candidate maximum plus its
	// agreement streak behind a downward re-arm. Same idiom as _class_streak:
	// several consecutive frames must agree before the reference moves.
	double _fuel_last_oz{0.0};
	double _fuel_cand_oz{0.0};
	int    _fuel_streak{0};

	// Last values actually derived from a plausible sample. A single bad frame
	// re-sends these rather than recomputing a bogus 0 V -> 0% -> EMERGENCY.
	double _last_voltage_v{0.0};
	double _last_remaining{1.0};

	// Consecutive frames of evidence DISAGREEING with the latched class.
	// Reclassification needs several; one glitch is not a model swap.
	int _class_streak{0};

	// fuel: edge detector for engine-out reporting.
	bool _engine_known{false};
	bool _engine_running{false};

	Source _source{Source::None};		// latched propulsion class
	Source _reported{Source::None};		// last label logged, for change-only logging
};

#endif
