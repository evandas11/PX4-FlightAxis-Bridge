/****************************************************************************
 *
 * This file is part of the PX4-FlightAxis-Bridge project.
 * Copyright (c) 2026 the PX4-FlightAxis-Bridge contributors.
 *
 * The RealFlight->NED frame conversions, the ground-accelerometer override, the
 * pitot-derived airspeed and the rangefinder handling declared here are ported
 * from ArduPilot libraries/SITL/SIM_FlightAxis.{h,cpp} (Copyright (C) ArduPilot
 * Dev Team, GPLv3). Because this file is a derivative work of that GPLv3 code,
 * it is itself licensed under GPLv3 or later:
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
 * The surrounding structure also follows PX4-FlightGear-Bridge
 * (Copyright (c) 2020 ThunderFly s.r.o., BSD-3-Clause); BSD-3-Clause is
 * GPL-compatible, so both notices coexist in this combined work.
 *
 ****************************************************************************/

/**
 * @file vehicle_state.h
 *
 * RealFlight (FlightAxis) state -> PX4 HIL messages.
 *
 * Frame conversions follow ArduPilot SIM_FlightAxis.cpp literally (they are
 * verified ground truth; RealFlight's conventions are internally
 * inconsistent - position swapped, velocity not, wind swapped).
 */

#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include <cmath>
#include <random>

#include <common/mavlink.h>
#include <Eigen/Geometry>

#include "fa_communicator.h"

class VehicleState
{
public:
	mavlink_hil_gps_t hil_gps_msg;
	double rpm;

	VehicleState(double home_lat_deg, double home_lon_deg, double home_alt_m);

	// Full RF->NED conversion pipeline for a NORMAL physics frame.
	// dt_true = the real elapsed physics time for this frame (NOT glitch-capped).
	//           Used only for the on-ground accel finite difference, so a
	//           swallowed glitch does not overstate the synthesised acceleration.
	// The message clock is NOT advanced here - the caller owns it and pushes it
	// in with setTimeUsec() (see the comment on that function).
	void setFAData(const FAState &fa, double dt_true);

	// Extrapolation step (duplicate physics frame): advance attitude by
	// q (x) exp(0.5*omega*dt_step), hold accel/gyro. Does not touch the clock.
	void extrapolate(double dt_step);

	// Capture position_offset from the current FA position fields (call on
	// first frame and after every reset) so home anchors the RF world.
	void resetPositionOffset(const FAState &fa);

	// Drop the captured offset so the NEXT real frame re-anchors from a
	// post-reset position (ArduPilot SIM_FlightAxis position_offset.zero()).
	// Called on a RealFlight reset or restart. The velocity history has to go
	// with the position anchor: the aircraft teleports to the runway, and the
	// on-ground accelerometer override would otherwise difference the
	// pre-reset flight velocity against zero across one frame - 30 m/s over
	// 4 ms is 765 g, clipped to the 16 g limit on all three axes and fed
	// straight into EKF2.
	void invalidatePositionOffset() { offset_captured = false; have_last_velocity = false; }

	mavlink_hil_sensor_t getSensorMsg(int offset_us);
	mavlink_hil_state_quaternion_t getStateQuatMsg(int offset_us);

	// Rangefinder (spec 6): AGL along the body-down axis, NAN when inverted.
	bool rangefinderValid() const { return std::isfinite(rangefinder_m); }
	mavlink_distance_sensor_t getDistanceSensorMsg(int offset_us);

	// Physics clock, us. This is a pure MIRROR of the bridge main loop's
	// glitch-compensated `time_now_us`, which is the single source of truth:
	// VehicleState never accumulates time of its own (an accumulator here
	// double-counted the extrapolation steps against the following real frame
	// and ran the whole message clock at 2x wall time).
	//
	// The setter clamps monotonically. The main loop is already designed to
	// keep time_now_us monotonic across a RealFlight restart and a glitch
	// swallow, so this is a belt-and-braces guard.
	//
	// What a backwards timestamp costs depends on the target, and the SITL
	// answer is NOT the obvious one:
	//  - HITL: the board's mavlink_receiver stamps the sensor topics with the
	//    time we send, so a backwards timestamp does fault EKF2.
	//  - SITL: it does not reach the estimator at all. SimulatorMavlink
	//    timestamps everything with its own hrt_absolute_time() on arrival
	//    (SimulatorMavlink.cpp:497,512 for HIL_SENSOR, :452 for HIL_GPS,
	//    :1489 for DISTANCE_SENSOR); the px4_clock_settime() call at :490-494
	//    is a no-op on a nolockstep build (it resolves to clock_settime() on
	//    CLOCK_MONOTONIC, which Linux rejects with EINVAL). What matters in
	//    SITL is the WALL-CLOCK ARRIVAL RATE - see the realtime-factor monitor
	//    in flightaxis_bridge.cpp.
	// The clock is still load-bearing in SITL for a different reason: it is
	// what every send-rate gate in px4_communicator.cpp is measured against,
	// so a stalled clock silently stops HIL_GPS, the baro/mag sub-rates and
	// DISTANCE_SENSOR entirely.
	void setTimeUsec(uint64_t t_us)
	{
		if (t_us > time_usec) {
			time_usec = t_us;
		}

		hil_gps_msg.time_usec = time_usec;
	}

	uint64_t timeUsec() const { return time_usec; }

	void setPXControls(const mavlink_hil_actuator_controls_t &controls);
	const mavlink_hil_actuator_controls_t &lastControls() const { return last_controls; }
	bool receivedFirstControls() const { return received_first_controls; }
	bool armed() const { return (last_controls.mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0; }

private:
	void updateGPSMsg();
	void nedToLLA(const Eigen::Vector3d &pos_ned, double &lat_deg, double &lon_deg, double &alt_m) const;

	// home / geodetic anchor
	double home_lat;	// deg
	double home_lon;	// deg
	double home_alt;	// m ASL

	// physics time since epoch capture (us), mirrored from the main loop;
	// getSensorMsg() / getDistanceSensorMsg() add offset_us on top
	uint64_t time_usec;

	// vehicle state (NED / body FRD)
	Eigen::Quaterniond q_ned;		// body -> NED rotation
	Eigen::Vector3d gyro;			// body rad/s
	Eigen::Vector3d accel_body;		// specific force, m/s^2
	Eigen::Vector3d velocity_ef;		// NED m/s
	Eigen::Vector3d last_velocity_ef;	// NED m/s (for ground accel override)
	Eigen::Vector3d wind_ef;		// NED m/s
	Eigen::Vector3d position_ned;		// m, offset-corrected
	Eigen::Vector3d position_offset;	// m, captured RF-world origin
	bool offset_captured;
	bool have_last_velocity;

	// synthesized sensors
	Eigen::Vector3d mag_ned;		// earth field at home, gauss
	Eigen::Vector3d mag_body;		// gauss
	double temperature;			// degC
	double abs_pressure;			// hPa
	double pressure_alt;			// m
	double diff_pressure;			// hPa
	double airspeed_true;			// m/s (RF total TAS)
	double airspeed_pitot;			// m/s (body-X, computed)
	double rangefinder_m;			// m along body-down axis, NAN if invalid

	// controls
	mavlink_hil_actuator_controls_t last_controls;
	bool received_first_controls;

	// sensor noise
	std::default_random_engine random_generator_;
	std::normal_distribution<double> standard_normal_distribution_;
	double acc_nois;
	double gyro_nois;
	double mag_nois;
	double baro_alt_nois;
	double temp_nois;
	double abs_pressure_nois;
	double diff_pressure_nois;
};

#endif
