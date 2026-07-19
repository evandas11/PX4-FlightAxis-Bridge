/****************************************************************************
 *
 * This file is part of the PX4-FlightAxis-Bridge project.
 * Copyright (c) 2026 Evangels Brilliant Dasmasela
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

	bool active() const { return _fd >= 0; }

private:
	// Which of the three sources the last update used. Logged on change only.
	enum class Source {
		None,		// nothing decided yet
		Electric,	// RealFlight reported a real pack voltage
		Fuel,		// internal combustion: voltage is -1, fuel is real
		Synthetic,	// model reports neither; a nominal full pack is sent
	};

	void send(double voltage_v, double current_a, double remaining,
		  double discharged_mah, uint64_t now_us);

	int _fd{-1};
	int _instance{0};
	uint8_t _sysid{1};

	uint64_t _last_send_us{0};
	uint64_t _last_update_us{0};

	// electric: inferred once, so voltages[] can be filled per-cell
	int _cell_count{0};

	// coulomb counter, mAh
	double _discharged_mah{0.0};

	// fuel: largest reading seen, taken as "full tank". Re-armed upwards on
	// refuel / aircraft reset rather than latched, so a mid-session reset
	// does not leave the fraction pinned below 1.0 forever.
	double _fuel_full_oz{0.0};

	Source _source{Source::None};
};

#endif
