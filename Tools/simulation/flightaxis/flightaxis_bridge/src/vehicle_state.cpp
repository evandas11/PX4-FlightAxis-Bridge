/****************************************************************************
 *
 * This file is part of the PX4-FlightAxis-Bridge project.
 * Copyright (c) 2026 the PX4-FlightAxis-Bridge contributors.
 *
 * The RealFlight->NED frame conversions, the ground-accelerometer override, the
 * pitot-derived airspeed and the rangefinder handling in this file are ported
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
 * @file vehicle_state.cpp
 *
 * RealFlight (FlightAxis) state -> PX4 HIL messages.
 *
 * Conversions ported literally from ArduPilot SIM_FlightAxis.cpp (verified
 * ground truth). Sensor-noise / mag / baro synthesis style follows the
 * FlightGear bridge's vehicle_state.cpp.
 */

#include "vehicle_state.h"

#include "geo_mag_declination.h"

#include <cmath>
#include <cstring>
#include <limits>

using namespace Eigen;

static constexpr double GRAVITY_MSS = 9.80665;
static constexpr double EARTH_RADIUS_M = 6371000.0;
static constexpr double DEG2RAD = M_PI / 180.0;
static constexpr double RAD2DEG = 180.0 / M_PI;

/*
 * This file defines no constrain() of its own, so every call below used to
 * resolve to the `const Scalar &constrain(const Scalar&, ...)` TEMPLATE that
 * geo_mag_declination.h happens to bring in for the magnetics. That is
 * accidental coupling - the magnetics header is a byte-identical third-party
 * copy and is not this file's utility library - and it behaves differently:
 * being a comparison chain that returns a reference, it propagates NaN
 * unchanged. getDistanceSensorMsg() then casts the result to uint16_t, which
 * is undefined behaviour, and is safe today only because a guard in a
 * different file (rangefinderValid(), checked by px4_communicator.cpp) gates
 * the call. A non-template overload on double is an exact match and so wins
 * over the template for every call here, restoring local ownership.
 */
static double constrain(double v, double lo, double hi)
{
	return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// ISA pressure in hPa at geodetic altitude h. Used both for the running baro
// in setFAData() and for the constructor's initial value, which MUST use the
// same datum - see the note in the constructor.
static double isaPressureHpa(double h_m)
{
	return 101325.0 * std::pow(1.0 - 2.25577e-5 * h_m, 5.25588) / 100.0;
}

VehicleState::VehicleState(double home_lat_deg, double home_lon_deg, double home_alt_m) :
	rpm(0.0),
	home_lat(home_lat_deg),
	home_lon(home_lon_deg),
	home_alt(home_alt_m),
	time_usec(0),
	q_ned(Quaterniond::Identity()),
	gyro(Vector3d::Zero()),
	accel_body(0.0, 0.0, -GRAVITY_MSS),
	velocity_ef(Vector3d::Zero()),
	last_velocity_ef(Vector3d::Zero()),
	wind_ef(Vector3d::Zero()),
	position_ned(Vector3d::Zero()),
	position_offset(Vector3d::Zero()),
	offset_captured(false),
	have_last_velocity(false),
	mag_body(Vector3d::Zero()),
	temperature(25.0),
	// These initial values ARE SENT: the bridge's reinject keep-alive calls
	// Send() before any RealFlight frame has arrived. Sea-level ISA (1013.25)
	// against a pressure_alt of home_alt therefore put PX4's baro and HIL_GPS
	// ~480 m apart for the whole pre-injection window at the default 488 m
	// home. Both must come off the same datum as setFAData(), which derives
	// them from h = home_alt - position_ned.z(); at startup position_ned is
	// zero, so h is exactly home_alt.
	abs_pressure(isaPressureHpa(home_alt_m)),
	pressure_alt(home_alt_m),
	diff_pressure(0.0),
	airspeed_true(0.0),
	airspeed_pitot(0.0),
	rangefinder_m(std::numeric_limits<double>::quiet_NaN()),
	received_first_controls(false)
{
	std::memset(&hil_gps_msg, 0, sizeof(hil_gps_msg));
	std::memset(&last_controls, 0, sizeof(last_controls));

	standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

	// same magnitudes as the FlightGear bridge
	acc_nois = 0.0001;
	gyro_nois = 0.001;
	mag_nois = 0.001;
	baro_alt_nois = 0.01;
	temp_nois = 0.01;
	abs_pressure_nois = 0.05;
	diff_pressure_nois = 0.01;

	// WMM earth field at home (gauss), NED - same synthesis as FG bridge
	float strength_ga = 0.01f * get_mag_strength(home_lat, home_lon);
	float declination_rad = get_mag_declination(home_lat, home_lon) * (float)DEG2RAD;
	float inclination_rad = get_mag_inclination(home_lat, home_lon) * (float)DEG2RAD;

	float H = strength_ga * cosf(inclination_rad);
	float Z = H * tanf(inclination_rad);
	float X = H * cosf(declination_rad);
	float Y = H * sinf(declination_rad);

	mag_ned = Vector3d(X, Y, Z);
	mag_body = mag_ned;

	updateGPSMsg();
}

void VehicleState::resetPositionOffset(const FAState &fa)
{
	// position NED: (N = RF_Y, E = RF_X, D = -altASL)
	position_offset = Vector3d(fa.m_aircraftPositionY_MTR,
				   fa.m_aircraftPositionX_MTR,
				   -fa.m_altitudeASL_MTR);
	offset_captured = true;
}

void VehicleState::setFAData(const FAState &fa, double dt_true)
{
	if (dt_true <= 0.0) {
		dt_true = 0.001;
	}

	// Quaternion RF -> NED: q_ned = (w=W, x=RF_Y, y=RF_X, z=-RF_Z)
	q_ned = Quaterniond(fa.m_orientationQuaternionW,
			    fa.m_orientationQuaternionY,
			    fa.m_orientationQuaternionX,
			    -fa.m_orientationQuaternionZ);
	q_ned.normalize();

	// Gyro: p=+roll, q=+pitch, r=-yaw (deg->rad, constrain +/-2000 deg/s)
	gyro = Vector3d(DEG2RAD * constrain(fa.m_rollRate_DEGpSEC, -2000.0, 2000.0),
			DEG2RAD * constrain(fa.m_pitchRate_DEGpSEC, -2000.0, 2000.0),
			-DEG2RAD * constrain(fa.m_yawRate_DEGpSEC, -2000.0, 2000.0));

	// World velocity used DIRECTLY: (vN=U, vE=V, vD=W) - no swap (intentional)
	velocity_ef = Vector3d(fa.m_velocityWorldU_MPS,
			       fa.m_velocityWorldV_MPS,
			       fa.m_velocityWorldW_MPS);

	if (!have_last_velocity) {
		last_velocity_ef = velocity_ef;
		have_last_velocity = true;
	}

	// Position NED: (N=RF_Y, E=RF_X, D=-altASL), minus captured offset
	Vector3d position_raw(fa.m_aircraftPositionY_MTR,
			      fa.m_aircraftPositionX_MTR,
			      -fa.m_altitudeASL_MTR);

	// Re-anchor whenever there is no valid anchor. This is deliberately the
	// ONLY condition: m_resetButtonHasBeenPressed is NOT checked here, because
	// this function is unreachable while that flag is set - the bridge's main
	// loop catches the flag and continue()s long before setFAData() is called
	// (flightaxis_bridge.cpp, the reinject branch). A `|| resetButtonPressed`
	// clause here would read as a safety net while being dead code. The bridge
	// instead calls invalidatePositionOffset() unconditionally on the flag,
	// which clears offset_captured and lands us here on the next real frame.
	if (!offset_captured) {
		resetPositionOffset(fa);
	}

	position_ned = position_raw - position_offset;

	// Wind swapped like position: (windY, windX, windZ)
	wind_ef = Vector3d(fa.m_windY_MPS, fa.m_windX_MPS, fa.m_windZ_MPS);

	// Accelerometer (specific force): body values in flight ...
	accel_body = Vector3d(fa.m_accelerationBodyAX_MPS2,
			      fa.m_accelerationBodyAY_MPS2,
			      fa.m_accelerationBodyAZ_MPS2);

	// ... but on the ground RF accel is garbage - finite-difference override
	// (yields exactly (0,0,-g) at rest)
	if (fa.m_isTouchingGround) {
		// use the TRUE elapsed time: during a swallowed glitch dt is capped and
		// would overstate the synthesised ground acceleration
		Vector3d accel_ef = (velocity_ef - last_velocity_ef) / dt_true;
		accel_ef.z() -= GRAVITY_MSS;
		accel_body = q_ned.inverse() * accel_ef;	// R_ned_to_body * accel_ef
	}

	// limit to 16 g to match pixhawk
	const double a_limit = GRAVITY_MSS * 16.0;
	accel_body.x() = constrain(accel_body.x(), -a_limit, a_limit);
	accel_body.y() = constrain(accel_body.y(), -a_limit, a_limit);
	accel_body.z() = constrain(accel_body.z(), -a_limit, a_limit);

	last_velocity_ef = velocity_ef;

	// Pitot airspeed: body-X component of (vel - wind); RF's m_airspeed_MPS
	// is total TAS and lies during hover/harrier/knife-edge
	airspeed_true = fa.m_airspeed_MPS;
	Vector3d airspeed3d = q_ned.inverse() * (velocity_ef - wind_ef);
	airspeed_pitot = std::max(airspeed3d.x(), 0.0);
	diff_pressure = 0.5 * 1.225 * airspeed_pitot * airspeed_pitot / 100.0;	// Pa -> hPa

	// Barometer: ISA from geodetic altitude (HIL_SENSOR.abs_pressure is hPa).
	// This MUST use the same datum as the GPS altitude reported by nedToLLA()
	// (home_alt - position_ned.z), not the raw RealFlight ASL: the RF runway
	// sits at its own arbitrary ASL and any mismatch shows up to EKF2 as a
	// constant baro-vs-GPS height disagreement. position_ned is already set.
	const double h = home_alt - position_ned.z();
	abs_pressure = isaPressureHpa(h);
	pressure_alt = h;
	temperature = 25.0;

	// Magnetometer: home earth field rotated to body (gauss)
	mag_body = q_ned.inverse() * mag_ned;

	// Rangefinder (ArduPilot SIM_FlightAxis): AGL projected onto the body-down
	// axis; invalid when the body-down axis points up (vehicle inverted).
	const double dcm_c_z = q_ned.toRotationMatrix()(2, 2);

	if (dcm_c_z > 0.0) {
		rangefinder_m = fa.m_altitudeAGL_MTR / dcm_c_z;

	} else {
		rangefinder_m = std::numeric_limits<double>::quiet_NaN();
	}

	// RPM (ArduPilot selection logic, single channel)
	if (fa.m_propRPM > 0) {
		rpm = fa.m_propRPM;

	} else if (fa.m_heliMainRotorRPM > 0) {
		rpm = fa.m_heliMainRotorRPM;

	} else {
		rpm = 0;
	}

	updateGPSMsg();
}

void VehicleState::extrapolate(double dt_step)
{
	if (dt_step <= 0.0) {
		return;
	}

	// advance attitude: q = q (x) exp(0.5 * omega * dt) (body rates)
	Vector3d theta = gyro * dt_step;
	double angle = theta.norm();

	if (angle > 1e-12) {
		Quaterniond dq(AngleAxisd(angle, theta / angle));
		q_ned = (q_ned * dq).normalized();
	}

	// hold accel/gyro; attitude-dependent sensors follow the new attitude
	mag_body = q_ned.inverse() * mag_ned;

	// the clock is owned by the caller (setTimeUsec), not advanced here
}

void VehicleState::nedToLLA(const Vector3d &pos_ned, double &lat_deg, double &lon_deg, double &alt_m) const
{
	// spherical earth around home, like ArduPilot/jMAVSim
	lat_deg = home_lat + RAD2DEG * (pos_ned.x() / EARTH_RADIUS_M);
	lon_deg = home_lon + RAD2DEG * (pos_ned.y() / (EARTH_RADIUS_M * std::cos(home_lat * DEG2RAD)));
	alt_m = home_alt - pos_ned.z();
}

void VehicleState::updateGPSMsg()
{
	double lat_deg, lon_deg, alt_m;
	nedToLLA(position_ned, lat_deg, lon_deg, alt_m);

	hil_gps_msg.time_usec = time_usec;
	hil_gps_msg.fix_type = 3;
	hil_gps_msg.lat = (int32_t)(lat_deg * 1e7);
	hil_gps_msg.lon = (int32_t)(lon_deg * 1e7);
	hil_gps_msg.alt = (int32_t)(alt_m * 1000.0);
	hil_gps_msg.eph = 100;
	hil_gps_msg.epv = 100;
	hil_gps_msg.vn = (int16_t)(velocity_ef.x() * 100.0);
	hil_gps_msg.ve = (int16_t)(velocity_ef.y() * 100.0);
	hil_gps_msg.vd = (int16_t)(velocity_ef.z() * 100.0);
	const double ground_speed = std::sqrt(velocity_ef.x() * velocity_ef.x() +
					      velocity_ef.y() * velocity_ef.y());
	hil_gps_msg.vel = (uint16_t)(ground_speed * 100.0);

	// COG from velocity (NOT RF azimuth). At rest the direction is undefined -
	// atan2(0,0) is 0, which claims "heading due north" - so report MAVLink's
	// explicit unknown sentinel (65535), which SimulatorMavlink.cpp:441 maps
	// to NAN. Cosmetic: sensor_gps.cog_rad has no reader in EKF2 in v1.16
	// (only GPS drivers and fake_gps write it). Kept because it costs nothing
	// and the sentinel is what the field is for.
	if (ground_speed < 0.1) {
		hil_gps_msg.cog = 65535;

	} else {
		double cog = std::atan2(velocity_ef.y(), velocity_ef.x()) * RAD2DEG;

		if (cog < 0) {
			cog += 360.0;
		}

		hil_gps_msg.cog = (uint16_t)(cog * 100.0);
	}
	hil_gps_msg.satellites_visible = 10;
}

mavlink_hil_sensor_t VehicleState::getSensorMsg(int offset_us)
{
	mavlink_hil_sensor_t sensor_msg{};

	sensor_msg.time_usec = time_usec + offset_us;

	sensor_msg.xacc = accel_body.x() + acc_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.yacc = accel_body.y() + acc_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.zacc = accel_body.z() + acc_nois * standard_normal_distribution_(random_generator_);

	sensor_msg.xgyro = gyro.x() + gyro_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.ygyro = gyro.y() + gyro_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.zgyro = gyro.z() + gyro_nois * standard_normal_distribution_(random_generator_);

	sensor_msg.xmag = mag_body.x() + mag_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.ymag = mag_body.y() + mag_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.zmag = mag_body.z() + mag_nois * standard_normal_distribution_(random_generator_);

	sensor_msg.temperature = temperature + temp_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.abs_pressure = abs_pressure + abs_pressure_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.pressure_alt = pressure_alt + baro_alt_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.diff_pressure = diff_pressure + diff_pressure_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.fields_updated = 0x1FFF;

	return sensor_msg;
}

mavlink_hil_state_quaternion_t VehicleState::getStateQuatMsg(int offset_us)
{
	mavlink_hil_state_quaternion_t msg{};

	msg.time_usec = time_usec + offset_us;

	msg.attitude_quaternion[0] = (float)q_ned.w();
	msg.attitude_quaternion[1] = (float)q_ned.x();
	msg.attitude_quaternion[2] = (float)q_ned.y();
	msg.attitude_quaternion[3] = (float)q_ned.z();

	msg.rollspeed = (float)gyro.x();
	msg.pitchspeed = (float)gyro.y();
	msg.yawspeed = (float)gyro.z();

	double lat_deg, lon_deg, alt_m;
	nedToLLA(position_ned, lat_deg, lon_deg, alt_m);
	msg.lat = (int32_t)(lat_deg * 1e7);
	msg.lon = (int32_t)(lon_deg * 1e7);
	msg.alt = (int32_t)(alt_m * 1000.0);

	msg.vx = (int16_t)(velocity_ef.x() * 100.0);
	msg.vy = (int16_t)(velocity_ef.y() * 100.0);
	msg.vz = (int16_t)(velocity_ef.z() * 100.0);

	msg.ind_airspeed = (uint16_t)(airspeed_pitot * 100.0);
	msg.true_airspeed = (uint16_t)(airspeed_true * 100.0);

	// mG
	msg.xacc = (int16_t)(accel_body.x() * 1000.0 / GRAVITY_MSS);
	msg.yacc = (int16_t)(accel_body.y() * 1000.0 / GRAVITY_MSS);
	msg.zacc = (int16_t)(accel_body.z() * 1000.0 / GRAVITY_MSS);

	return msg;
}

mavlink_distance_sensor_t VehicleState::getDistanceSensorMsg(int offset_us)
{
	// PX4 SimulatorMavlink::publish_distance_topic() reads the distance fields
	// in cm and covariance in cm^2; PITCH_270 maps to ROTATION_DOWNWARD_FACING.
	const double min_m = 0.1;
	const double max_m = 40.0;

	mavlink_distance_sensor_t msg{};

	msg.time_boot_ms = (uint32_t)((time_usec + offset_us) / 1000);
	msg.min_distance = (uint16_t)(min_m * 100.0);
	msg.max_distance = (uint16_t)(max_m * 100.0);
	/*
	 * Out of range must be SIGNALLED, not clamped.
	 *
	 * EKF2's validity test is inclusive - SensorRangeFinder::isDataInRange()
	 * is `rng >= min && rng <= max` (sensor_range_finder.cpp:117) - so a
	 * reading clamped to exactly max_m is ACCEPTED as a real measurement. In
	 * cruise that hands FixedwingPositionControl.cpp:3069-3071 a landing-flare
	 * terrain estimate pinned 40 m below the aircraft, which then jumps as the
	 * aircraft descends through 40 m; it also reaches mission_block.cpp:1002
	 * and the hover-thrust estimator. Reporting max + 1 m instead falls
	 * outside the inclusive bound and is rejected properly.
	 *
	 * rangefinder_m is also NAN when the vehicle is inverted. px4_communicator
	 * gates this call on rangefinderValid(), but a float->uint16_t cast of NaN
	 * is undefined behaviour, so do not depend on a guard in another file for
	 * well-definedness: handle it here too, as out-of-range.
	 */
	const double out_of_range_m = max_m + 1.0;
	double d;

	if (!std::isfinite(rangefinder_m) || rangefinder_m > max_m) {
		d = out_of_range_m;

	} else {
		d = constrain(rangefinder_m, min_m, max_m);
	}

	msg.current_distance = (uint16_t)(d * 100.0);
	msg.type = MAV_DISTANCE_SENSOR_LASER;
	msg.id = 0;
	msg.orientation = MAV_SENSOR_ROTATION_PITCH_270;	// downward facing
	// UINT8_MAX is MAVLink's "unknown" sentinel. NOT 0: PX4 computes
	// distance_sensor.variance = covariance * 1e-4 (SimulatorMavlink.cpp:1494),
	// so 0 asserts a PERFECT sensor with zero variance rather than an unknown
	// one. Low impact today because EKF2 uses EKF2_RNG_NOISE instead, but the
	// value was simply wrong under a comment claiming otherwise.
	msg.covariance = UINT8_MAX;				// unknown
	msg.signal_quality = 100;

	return msg;
}

void VehicleState::setPXControls(const mavlink_hil_actuator_controls_t &controls)
{
	last_controls = controls;
	received_first_controls = true;
}
