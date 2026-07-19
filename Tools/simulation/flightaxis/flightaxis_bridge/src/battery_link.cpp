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
 * @file battery_link.cpp
 *
 * See battery_link.h for why this is a separate socket, why it is v2-only,
 * why the sysid is instance+1, and why HITL cannot have it.
 */

#include "battery_link.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <common/mavlink.h>

namespace
{

// PX4's API/offboard MAVLink link: px4-rc.mavlink starts it on 14580+instance.
constexpr int PORT_BASE = 14580;

// Component id we transmit as. MUST differ from PX4's own (1) or
// handle_message_battery_status() drops the message. 200 is what the bridge
// already uses for its HIL_* traffic, so the two agree.
constexpr uint8_t BATTERY_COMPID = 200;

// 2 Hz. A battery is a slowly varying quantity and PX4 applies its own
// filtering, so there is nothing to gain from going faster - and this shares
// its budget with an exchange loop measured at 750-800 Hz.
constexpr uint64_t SEND_INTERVAL_US = 500000;

// Per-cell endpoints for the voltage->state-of-charge estimate. RealFlight
// reports pack voltage and current draw but no capacity, so there is no
// coulomb-counting route to a TRUE remaining fraction; a linear map across the
// usable LiPo range is the honest approximation and it does track real sag,
// which is the whole point of preferring it to battery_simulator's ramp.
constexpr double CELL_EMPTY_V = 3.30;
constexpr double CELL_FULL_V  = 4.20;

// Nominal pack used when the RealFlight model reports no battery at all.
constexpr double SYNTHETIC_CELLS = 4.0;
constexpr double SYNTHETIC_V     = 4.0 * 4.10;

double clamp(double v, double lo, double hi)
{
	return (v < lo) ? lo : ((v > hi) ? hi : v);
}

} // namespace

bool BatteryLink::init(int instance)
{
	_instance = instance;

	// rcS: `param set MAV_SYS_ID $((px4_instance+1))`. The receiver's gate
	// compares against that exact value, so mirror the arithmetic.
	_sysid = (uint8_t)(instance + 1);

	_fd = ::socket(AF_INET, SOCK_DGRAM, 0);

	if (_fd < 0) {
		return false;
	}

	// Force MAVLink v2 on this channel, explicitly.
	//
	// This is not belt-and-braces, it is the bug. PX4 silently drops a v1
	// BATTERY_STATUS: id 147 encodes perfectly well in v1 and the CRC_EXTRA
	// is identical (154), but it never reaches the uORB topic. It was
	// diagnosed only by noticing that a COMMAND_LONG sent down the very same
	// socket WAS processed and logged. Relying on the channel status being
	// zero-initialised to v2 would leave that landmine armed for anyone who
	// later sets MAVLINK_STATUS_FLAG_OUT_MAVLINK1 anywhere in the process.
	mavlink_status_t *chan = mavlink_get_channel_status(MAVLINK_COMM_1);

	if (chan != nullptr) {
		chan->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
	}

	return true;
}

void BatteryLink::close()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

void BatteryLink::update(const FAState &fa, uint64_t now_us)
{
	if (_fd < 0) {
		return;
	}

	// Time-based gate. Everything below this line runs at 2 Hz regardless of
	// how fast the exchange loop is turning.
	if (_last_send_us != 0 && (now_us - _last_send_us) < SEND_INTERVAL_US) {
		return;
	}

	const double dt_s = (_last_update_us == 0) ? 0.0 : (double)(now_us - _last_update_us) * 1.0e-6;
	_last_update_us = now_us;

	const double v_raw    = fa.m_batteryVoltage_VOLTS;
	const double i_raw    = fa.m_batteryCurrentDraw_AMPS;
	const double fuel_oz  = fa.m_fuelRemaining_OZ;

	double voltage_v = 0.0;
	double current_a = 0.0;
	double remaining = 1.0;
	Source source    = Source::Synthetic;

	if (std::isfinite(v_raw) && v_raw > 0.0) {
		// ---- Electric: RealFlight is modelling a real pack ----------------
		source    = Source::Electric;
		voltage_v = v_raw;

		// The current field goes negative on some models (regen / prop
		// windmilling) and is -1 on models that do not model current at all.
		// Either way a negative draw must not run the coulomb counter
		// backwards, so clamp at zero rather than trusting the sign.
		current_a = (std::isfinite(i_raw) && i_raw > 0.0) ? i_raw : 0.0;

		// Infer the cell count once, from the first voltage seen. PX4 sums
		// voltages[] to recover voltage_v, so the split has to be sane even
		// though only the sum is load-bearing.
		if (_cell_count == 0) {
			int n = (int)std::lround(voltage_v / CELL_FULL_V);

			if (n < 1) {
				n = 1;
			}

			if (n > 10) {
				n = 10;
			}

			_cell_count = n;
		}

		const double per_cell = voltage_v / (double)_cell_count;
		remaining = clamp((per_cell - CELL_EMPTY_V) / (CELL_FULL_V - CELL_EMPTY_V), 0.0, 1.0);

		_discharged_mah += current_a * dt_s * (1000.0 / 3600.0);

	} else if (std::isfinite(fuel_oz) && fuel_oz > 0.0) {
		// ---- Internal combustion --------------------------------------------
		// This is the case the spec's pitfalls ledger flags: RealFlight's IC
		// models report battery = -1. They do report fuel, and fuel is the
		// physically meaningful "how much flying is left", so it drives
		// `remaining` - which is what PX4's BAT_*_THR failsafes actually act
		// on.
		//
		// The voltage reported alongside is NOMINAL, not measured. It exists
		// so that PX4's voltage-based low-battery checks (BAT_V_EMPTY and
		// friends) see a healthy pack instead of 0 V and fire an emergency
		// landing the instant the aircraft is armed. It is flagged in the log
		// so nobody mistakes it for telemetry.
		source    = Source::Fuel;
		voltage_v = SYNTHETIC_V;
		current_a = 0.0;

		// Take the largest reading seen as a full tank. Re-arm upwards so a
		// refuel or an aircraft reset restores 100% instead of leaving the
		// fraction pinned low for the rest of the session.
		if (fuel_oz > _fuel_full_oz) {
			_fuel_full_oz = fuel_oz;
		}

		remaining = (_fuel_full_oz > 0.0) ? clamp(fuel_oz / _fuel_full_oz, 0.0, 1.0) : 1.0;

	} else {
		// ---- Model reports neither ------------------------------------------
		// The airframes turn battery_simulator off (SIM_BAT_ENABLE 0) so that
		// there is exactly ONE publisher on battery_status; that makes this
		// bridge responsible for always producing something. A model with no
		// electrical or fuel model at all still has to pass preflight, so send
		// a nominal full pack. Obviously synthetic, and logged as such.
		source    = Source::Synthetic;
		voltage_v = SYNTHETIC_V;
		current_a = 0.0;
		remaining = 1.0;

		if (_cell_count == 0) {
			_cell_count = (int)SYNTHETIC_CELLS;
		}
	}

	if (source != _source) {
		const char *name = "unknown";

		switch (source) {
		case Source::Electric:
			name = "electric (RealFlight pack voltage/current)";
			break;

		case Source::Fuel:
			name = "fuel (internal combustion; battery=-1, voltage reported is NOMINAL not measured)";
			break;

		case Source::Synthetic:
			name = "synthetic (model reports neither battery nor fuel)";
			break;

		default:
			break;
		}

		fprintf(stderr, "[flightaxis_bridge] battery source: %s\n", name);
		_source = source;
	}

	send(voltage_v, current_a, remaining, _discharged_mah, now_us);
}

void BatteryLink::send(double voltage_v, double current_a, double remaining,
		       double discharged_mah, uint64_t now_us)
{
	_last_send_us = now_us;

	int cells = (_cell_count > 0) ? _cell_count : 1;
	const double per_cell_mv = (voltage_v / (double)cells) * 1000.0;

	uint16_t voltages[10];

	for (int i = 0; i < 10; i++) {
		// UINT16_MAX terminates the list PX4 walks in
		// handle_message_battery_status(), so unused cells must carry it
		// rather than 0 - a 0 would be summed into voltage_v as a dead cell.
		voltages[i] = (i < cells) ? (uint16_t)clamp(per_cell_mv, 0.0, 65534.0) : UINT16_MAX;
	}

	uint16_t voltages_ext[4] = { 0, 0, 0, 0 };

	mavlink_message_t msg;

	// MAVLINK_COMM_1: a channel of its own, so this stream's sequence numbers
	// do not interleave with the simulator link's on MAVLINK_COMM_0.
	mavlink_msg_battery_status_pack_chan(
		_sysid, BATTERY_COMPID, MAVLINK_COMM_1, &msg,
		0,					// id
		MAV_BATTERY_FUNCTION_ALL,		// battery_function
		MAV_BATTERY_TYPE_LIPO,			// type
		INT16_MAX,				// temperature: "unknown" per spec
		voltages,
		(int16_t)clamp(current_a * 100.0, -32768.0, 32767.0),	// cA
		(int32_t)clamp(discharged_mah, 0.0, 2147483647.0),	// mAh
		-1,					// energy_consumed: not modelled
		(int8_t)clamp(remaining * 100.0, 0.0, 100.0),		// percent
		0,					// time_remaining: not modelled
		MAV_BATTERY_CHARGE_STATE_UNDEFINED,
		voltages_ext,
		0,					// mode
		0);					// fault_bitmask

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	const int len = mavlink_msg_to_send_buffer(buf, &msg);

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = htons((uint16_t)(PORT_BASE + _instance));

	// MSG_DONTWAIT: never let a full socket buffer stall the exchange loop.
	// The result is deliberately ignored - a dropped battery packet is
	// unimportant and the next one is 500 ms away.
	(void)::sendto(_fd, buf, (size_t)len, MSG_DONTWAIT,
		       (struct sockaddr *)&addr, sizeof(addr));
}
