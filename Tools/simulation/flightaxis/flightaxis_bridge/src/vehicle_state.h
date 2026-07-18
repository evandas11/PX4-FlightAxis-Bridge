/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
	// dt = physics-time delta in seconds (already glitch-capped by caller).
	void setFAData(const FAState &fa, double dt);

	// Extrapolation step (duplicate physics frame): advance attitude by
	// q (x) exp(0.5*omega*dt_step), hold accel/gyro, bump internal time.
	void extrapolate(double dt_step);

	// Capture position_offset from the current FA position fields (call on
	// first frame and after every reset) so home anchors the RF world.
	void resetPositionOffset(const FAState &fa);

	mavlink_hil_sensor_t getSensorMsg(int offset_us);
	mavlink_hil_state_quaternion_t getStateQuatMsg(int offset_us);

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

	// internal physics-time since epoch capture (s); getSensorMsg adds offset_us
	double time_sec;

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
