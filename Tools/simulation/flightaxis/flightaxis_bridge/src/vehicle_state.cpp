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

VehicleState::VehicleState(double home_lat_deg, double home_lon_deg, double home_alt_m) :
	rpm(0.0),
	home_lat(home_lat_deg),
	home_lon(home_lon_deg),
	home_alt(home_alt_m),
	time_sec(0.0),
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
	abs_pressure(1013.25),
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

void VehicleState::setFAData(const FAState &fa, double dt, double dt_true)
{
	if (dt <= 0.0) {
		dt = 0.001;
	}

	if (dt_true <= 0.0) {
		dt_true = dt;
	}

	time_sec += dt;

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

	if (!offset_captured || fa.m_resetButtonHasBeenPressed) {
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
	abs_pressure = 101325.0 * std::pow(1.0 - 2.25577e-5 * h, 5.25588) / 100.0;
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

	time_sec += dt_step;
	hil_gps_msg.time_usec = (uint64_t)(time_sec * 1e6);
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

	hil_gps_msg.time_usec = (uint64_t)(time_sec * 1e6);
	hil_gps_msg.fix_type = 3;
	hil_gps_msg.lat = (int32_t)(lat_deg * 1e7);
	hil_gps_msg.lon = (int32_t)(lon_deg * 1e7);
	hil_gps_msg.alt = (int32_t)(alt_m * 1000.0);
	hil_gps_msg.eph = 100;
	hil_gps_msg.epv = 100;
	hil_gps_msg.vn = (int16_t)(velocity_ef.x() * 100.0);
	hil_gps_msg.ve = (int16_t)(velocity_ef.y() * 100.0);
	hil_gps_msg.vd = (int16_t)(velocity_ef.z() * 100.0);
	hil_gps_msg.vel = (uint16_t)(std::sqrt(velocity_ef.x() * velocity_ef.x() +
					       velocity_ef.y() * velocity_ef.y()) * 100.0);

	// COG from velocity (NOT RF azimuth)
	double cog = std::atan2(velocity_ef.y(), velocity_ef.x()) * RAD2DEG;

	if (cog < 0) {
		cog += 360.0;
	}

	hil_gps_msg.cog = (uint16_t)(cog * 100.0);
	hil_gps_msg.satellites_visible = 10;
}

mavlink_hil_sensor_t VehicleState::getSensorMsg(int offset_us)
{
	mavlink_hil_sensor_t sensor_msg{};

	sensor_msg.time_usec = (uint64_t)(time_sec * 1e6) + offset_us;

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

	msg.time_usec = (uint64_t)(time_sec * 1e6) + offset_us;

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

	msg.time_boot_ms = (uint32_t)(((uint64_t)(time_sec * 1e6) + offset_us) / 1000);
	msg.min_distance = (uint16_t)(min_m * 100.0);
	msg.max_distance = (uint16_t)(max_m * 100.0);
	msg.current_distance = (uint16_t)(constrain(rangefinder_m, min_m, max_m) * 100.0);
	msg.type = MAV_DISTANCE_SENSOR_LASER;
	msg.id = 0;
	msg.orientation = MAV_SENSOR_ROTATION_PITCH_270;	// downward facing
	msg.covariance = 0;					// unknown
	msg.signal_quality = 100;

	return msg;
}

void VehicleState::setPXControls(const mavlink_hil_actuator_controls_t &controls)
{
	last_controls = controls;
	received_first_controls = true;
}
