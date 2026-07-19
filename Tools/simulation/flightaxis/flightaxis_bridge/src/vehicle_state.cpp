/****************************************************************************
 *
 * This file is part of the PX4-FlightAxis-Bridge project.
 * Copyright (c) 2026 Evangels Brilliant Dasmasela
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
 * Conversions ported literally from the upstream implementation named in the
 * licence header above (verified ground truth).
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

/*
 * SOUTHERN-HEMISPHERE / WESTERN-LONGITUDE CORRECTION FOR THE WMM LOOKUP.
 *
 * geo_mag_declination.cpp is a byte-identical copy of an old PX4 file and
 * carries that file's rounding bug: get_table_data() finds the south-west grid
 * corner with
 *
 *     float min_lat = int(lat / SAMPLING_RES) * SAMPLING_RES;
 *
 * int() truncates TOWARDS ZERO, not down. For lat = -37.73 that yields -30,
 * which is NORTH of the query point, so the interpolation weight
 * (lat - min_lat)/10 = -0.773 comes out negative and is then clamped to 0 by
 * the constrain() on lat_scale. The result is that every latitude in
 * (-40, -30] silently returns the value AT -30, and likewise for every other
 * southern band and for all negative longitudes. Measured against the current
 * PX4 world_magnetic_model at the user's field (-37.7304, 175.7438):
 *
 *     bridge   decl 16.57 deg   incl -55.85 deg   |B| 0.4985 G
 *     PX4 WMM  decl 21.00 deg   incl -63.14 deg   |B| 0.5374 G
 *
 * i.e. 4.4 deg of declination, 7.3 deg of inclination and 7.2% of strength.
 * The same comparison at Zurich agrees to 0.84 deg / 0.10 deg / 0.3% - the
 * error is invisible at the default home and appears only south of the equator,
 * which is why it survived this long.
 *
 * This matters because EKF2 does NOT use this table. Ekf::updateWorldMagneticModel()
 * calls the modern get_mag_declination_degrees()/..._inclination_degrees()/
 * ..._strength_gauss() at the live GPS position, and Ekf::getMagDeclination()
 * feeds that declination into mag fusion. So the synthetic field we hand PX4
 * encodes one declination while the estimator is simultaneously told the true
 * one, leaving a constant heading disagreement that no amount of fusion can
 * null out.
 *
 * The fix cannot live in geo_mag_declination.cpp - that file is a verbatim
 * third-party copy and is owned elsewhere - so it is applied at the call site
 * instead. Note that get_table_data() is EXACT when queried at a grid point:
 * for any multiple of SAMPLING_RES, int() and floor() agree, both scales come
 * out 0 or 1, and the raw table entry is returned. That makes it safe to use
 * the existing accessors as a plain table read at the four surrounding corners
 * and do the bilinear interpolation here, with a floor() that rounds the right
 * way for both signs.
 */
static constexpr double MAG_SAMPLING_RES = 10.0;
static constexpr double MAG_MIN_LAT = -60.0;
static constexpr double MAG_MAX_LAT = 60.0;
static constexpr double MAG_MIN_LON = -180.0;
static constexpr double MAG_MAX_LON = 180.0;

static double magTableLookup(float (*table_get)(float, float), double lat, double lon)
{
	// Outside the table's latitude span there is nothing to interpolate between;
	// clamp to the edge rather than let the corner sampling straddle it.
	lat = constrain(lat, MAG_MIN_LAT, MAG_MAX_LAT);
	lon = constrain(lon, MAG_MIN_LON, MAG_MAX_LON);

	// South-west corner, rounded DOWN (this is the actual fix), then held back
	// one cell from the north/east edge so corner + SAMPLING_RES stays in range.
	double lat0 = constrain(std::floor(lat / MAG_SAMPLING_RES) * MAG_SAMPLING_RES,
				MAG_MIN_LAT, MAG_MAX_LAT - MAG_SAMPLING_RES);
	double lon0 = constrain(std::floor(lon / MAG_SAMPLING_RES) * MAG_SAMPLING_RES,
				MAG_MIN_LON, MAG_MAX_LON - MAG_SAMPLING_RES);

	const double sw = table_get((float)lat0, (float)lon0);
	const double se = table_get((float)lat0, (float)(lon0 + MAG_SAMPLING_RES));
	const double nw = table_get((float)(lat0 + MAG_SAMPLING_RES), (float)lon0);
	const double ne = table_get((float)(lat0 + MAG_SAMPLING_RES), (float)(lon0 + MAG_SAMPLING_RES));

	const double lat_scale = constrain((lat - lat0) / MAG_SAMPLING_RES, 0.0, 1.0);
	const double lon_scale = constrain((lon - lon0) / MAG_SAMPLING_RES, 0.0, 1.0);

	const double south = sw + lon_scale * (se - sw);
	const double north = nw + lon_scale * (ne - nw);

	return south + lat_scale * (north - south);
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
	rc_valid(false),
	received_first_controls(false)
{
	std::memset(&rc_raw, 0, sizeof(rc_raw));
	std::memset(&hil_gps_msg, 0, sizeof(hil_gps_msg));
	std::memset(&last_controls, 0, sizeof(last_controls));

	standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

	// sensor noise magnitudes
	acc_nois = 0.0001;
	gyro_nois = 0.001;
	mag_nois = 0.001;
	baro_alt_nois = 0.01;
	temp_nois = 0.01;
	abs_pressure_nois = 0.05;
	diff_pressure_nois = 0.01;

	// WMM earth field at home (gauss), NED - same synthesis as FG bridge, but
	// through magTableLookup() rather than the accessors directly; see the note
	// on that function for why the raw accessors are wrong south of the equator.
	// The 0.01 is the table's own unit: it stores strength in centi-gauss
	// (~50 at mid latitudes), so 0.01 * 50 = 0.5 G. The declaration comment in
	// geo_mag_declination.h calling this "centi-Tesla" is simply wrong.
	double strength_ga = 0.01 * magTableLookup(get_mag_strength, home_lat, home_lon);
	double declination_rad = magTableLookup(get_mag_declination, home_lat, home_lon) * DEG2RAD;
	double inclination_rad = magTableLookup(get_mag_inclination, home_lat, home_lon) * DEG2RAD;

	// NED components. Z is positive DOWN, and tan(inclination) carries the sign:
	// in the northern hemisphere inclination is positive and the field points
	// into the ground (Z > 0); in the southern hemisphere it is negative and the
	// field points up out of the ground (Z < 0). Nothing here needs a hemisphere
	// special case - the sign falls out of the inclination.
	double H = strength_ga * std::cos(inclination_rad);
	double Z = H * std::tan(inclination_rad);
	double X = H * std::cos(declination_rad);
	double Y = H * std::sin(declination_rad);

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

	/*
	 * A CRASHED MODEL REPORTS EXACTLY ZERO, AND ZERO IS NOT "NO READING".
	 *
	 * An accelerometer measures SPECIFIC FORCE, so (0,0,0) is not a neutral
	 * value - it is the reading of an instrument in perfect free fall. A real
	 * one never produces it: at rest it reads (0,0,-g). But RealFlight zeroes
	 * m_accelerationBodyA{X,Y,Z} once the model breaks up, and reports
	 * m_isTouchingGround = false for the wreck, so the ground override below
	 * never ran and the zeros went to PX4 verbatim.
	 *
	 * Measured in log 09_57_12: for the 150 s the model sat crashed, every
	 * sensor_accel sample was exactly (0.0047884, 0, 0) - one LSB - while the
	 * gyro showed normal noise and ground truth vz was exactly 0. PX4 was being
	 * told the aircraft had been falling at 1 g for two and a half minutes.
	 * EKF2 integrated that against a static GPS and baro, which cost 317 vz
	 * resets and swung the estimated vertical velocity +-12 m/s on an airframe
	 * that was not moving. The land detector never fired either - its
	 * vertical-movement test is defeated by the phantom velocity - so PX4 kept
	 * the motors armed and saturated the whole time.
	 *
	 * That divergence is what makes a respawn violent. On the reset edge the
	 * position re-anchor teleports GPS by up to 350 m and baro by ~20 m in a
	 * single sample; EKF2 declares a velocity reset with delta_vz over 20 m/s,
	 * and FlightTask::_checkEkfResetCounters (FlightTask.cpp:83-86) hands that
	 * to _ekfResetHandlerVelocityZ, whose _smoothing.setCurrentVelocity() IS
	 * the output setpoint. The climb-rate command is therefore overwritten with
	 * the diverged estimate and the throttle stick is discarded entirely -
	 * measured, -2.56 m/s commanded climb with the stick at -1.00.
	 *
	 * So: treat an exactly-zero triad as a missing reading rather than a
	 * measurement, and fall through to the finite-difference path, which yields
	 * (0,0,-g) for a stationary wreck.
	 *
	 * THE FIRST VERSION OF THIS TEST COMPARED ALL THREE AXES TO EXACT ZERO, ON
	 * THE THEORY THAT ZERO WAS RealFlight's SENTINEL. It never once fired.
	 * Measured in flight 10_32_18: for 8.84 s (1786 consecutive samples,
	 * t=21.77-30.61) PX4 received |a| = 0.0047884 m/s2 - one accelerometer LSB,
	 * not zero - while groundtruth stood still. The branch is provably not
	 * taken there: it subtracts g in the earth frame, and rotation preserves
	 * magnitude, so any sample it produced would read at least 9.8.
	 *
	 * So test what is actually meant. Two independent conditions:
	 *
	 *  - m_hasLostComponents. RealFlight already tells us the model broke up.
	 *    The bridge has always parsed it (fa_communicator.cpp:592) and printed
	 *    it as "the single most useful line this bridge can print" - and then
	 *    acted on it nowhere. This is the place it was needed.
	 *
	 *  - A magnitude threshold rather than an equality. A resting accelerometer
	 *    reads 1 g; anything near zero is either free fall or a dead model, and
	 *    the finite-difference path gives the physically correct answer for
	 *    BOTH, so a false positive here is harmless.
	 *
	 * The cost of getting this wrong was not the height estimate - baro and GPS
	 * agreed to 0.008 m mean throughout. It was ATTITUDE. With |a| near zero the
	 * gravity observation is nowhere near 1 g, so EKF2 rejected it on 100% of
	 * samples (mean test ratio 25.6) and lost its only tilt-correction path.
	 * Tilt error went from ~2 deg to 74.6 deg and stayed there for 34 s; the
	 * EKF then rotated a correct static accel into NED, got -3.5 instead of
	 * -9.81, and integrated the 6.3 m/s2 remainder into a phantom climb that
	 * reset roughly once a second. That is the whole of the 100 vz / 58 z reset
	 * episode, which is confined to t=20.8-76.4 - after attitude recovered
	 * there were ZERO resets across 150 s and two complete VTOL transitions.
	 */
	const double accel_norm = std::sqrt(
			fa.m_accelerationBodyAX_MPS2 * fa.m_accelerationBodyAX_MPS2 +
			fa.m_accelerationBodyAY_MPS2 * fa.m_accelerationBodyAY_MPS2 +
			fa.m_accelerationBodyAZ_MPS2 * fa.m_accelerationBodyAZ_MPS2);

	const bool accel_absent = fa.m_hasLostComponents ||
				  !std::isfinite(accel_norm) ||
				  accel_norm < 0.05;

	// ... but on the ground RF accel is garbage - finite-difference override
	// (yields exactly (0,0,-g) at rest)
	const bool use_finite_difference = fa.m_isTouchingGround || accel_absent;

	if (use_finite_difference) {
		/*
		 * Differentiating velocity amplifies by 1/dt, so a tiny velocity step
		 * across a short frame becomes an enormous acceleration. Measured: one
		 * sample at t=30.7188 came out at 45.0 m/s2 - the only reading above
		 * 40 in the entire log - right after a respawn, from a dt of about
		 * 3 ms. It passed the 16 g sensor clamp below untouched.
		 *
		 * Two guards. Below MIN_DIFF_DT_S the quotient is noise amplification
		 * rather than measurement, so hold the previous value instead. And the
		 * result is bounded to 3 g: the 16 g clamp exists to match what a
		 * Pixhawk can REPORT, which is the right bound for something measured
		 * and far too loose for something we synthesised by division.
		 */
		const double MIN_DIFF_DT_S = 0.005;
		const double SYNTH_LIMIT   = GRAVITY_MSS * 3.0;

		if (dt_true >= MIN_DIFF_DT_S && have_last_velocity) {
			Vector3d accel_ef = (velocity_ef - last_velocity_ef) / dt_true;
			accel_ef.z() -= GRAVITY_MSS;
			accel_body = q_ned.inverse() * accel_ef;	// R_ned_to_body * accel_ef

			accel_body.x() = constrain(accel_body.x(), -SYNTH_LIMIT, SYNTH_LIMIT);
			accel_body.y() = constrain(accel_body.y(), -SYNTH_LIMIT, SYNTH_LIMIT);
			accel_body.z() = constrain(accel_body.z(), -SYNTH_LIMIT, SYNTH_LIMIT);

		} else {
			// No usable interval to differentiate over: report the aircraft at
			// rest rather than a quotient dominated by 1/dt. For a stationary
			// wreck - the case this branch exists for - this is also the right
			// answer, and it keeps the gravity observation valid so EKF2 can
			// still correct tilt.
			accel_body = q_ned.inverse() * Vector3d(0.0, 0.0, -GRAVITY_MSS);
		}
	}

	/*
	 * Switching accel source mid-flight means the next finite difference would
	 * span a discontinuity, so drop the history on the edge. invalidatePosition
	 * Offset() already does this for a respawn (vehicle_state.h:84), but that
	 * only runs on the reset button.
	 */
	if (use_finite_difference != last_used_finite_difference) {
		have_last_velocity = false;
		last_used_finite_difference = use_finite_difference;
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

	// Rangefinder: AGL projected onto the body-down
	// axis; invalid when the body-down axis points up (vehicle inverted).
	const double dcm_c_z = q_ned.toRotationMatrix()(2, 2);

	if (dcm_c_z > 0.0) {
		rangefinder_m = fa.m_altitudeAGL_MTR / dcm_c_z;

	} else {
		rangefinder_m = std::numeric_limits<double>::quiet_NaN();
	}

	// RPM (single channel)
	if (fa.m_propRPM > 0) {
		rpm = fa.m_propRPM;

	} else if (fa.m_heliMainRotorRPM > 0) {
		rpm = fa.m_heliMainRotorRPM;

	} else {
		rpm = 0;
	}

	/*
	 * RC transmitter passthrough. fa.rcin[] is RealFlight's echo of the physical
	 * TX channels, normalised 0..1.
	 *
	 * 1000 + x*1000 is the conventional expression of an RC channel as a raw
	 * servo pulse width: 1000 us at one extreme, 1500 us centred, 2000 us at the
	 * other. That is the unit input_rc.values[] is in, and it is what QGC's RC
	 * calibration and the RC_MIN/RC_TRIM/RC_MAX parameters are expressed in, so
	 * a standard calibration run maps these straight onto normalised stick
	 * inputs with no scaling anywhere else in the chain.
	 *
	 * Clamped rather than trusted: RealFlight will report slightly outside 0..1
	 * for a TX whose endpoints are set beyond the model's configured travel, and
	 * an unclamped cast of, say, 1.3 would land at 2300 us - or, negative, wrap
	 * to a huge uint16_t. NaN is treated as centre; a NaN cast to uint16_t is
	 * undefined behaviour and must not reach the cast.
	 */
	for (int i = 0; i < RC_CHANNELS_N; i++) {
		const double x = fa.rcin[i];

		if (!std::isfinite(x)) {
			rc_raw[i] = 1500;

		} else {
			rc_raw[i] = (uint16_t)(1000.0 + constrain(x, 0.0, 1.0) * 1000.0);
		}
	}

	// Only now are the stick positions real; see rcValid() in vehicle_state.h.
	rc_valid = true;

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
	// spherical earth around home
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

mavlink_rc_channels_t VehicleState::getRcChannelsMsg(int offset_us)
{
	mavlink_rc_channels_t msg{};

	msg.time_boot_ms = (uint32_t)((time_usec + offset_us) / 1000);
	msg.chancount = RC_CHANNELS_N;

	// RealFlight gives us exactly 12. The remaining six get UINT16_MAX, which is
	// MAVLink's "channel not available" sentinel for these fields - NOT 0, which
	// would read as a real channel pegged below its minimum. chancount already
	// says 12, but SimulatorMavlink copies all 18 raw fields into
	// input_rc.values[] regardless of chancount (SimulatorMavlink.cpp:900-917),
	// so the sentinel is what actually keeps 13-18 from looking live to anything
	// that reads values[] without checking channel_count first.
	uint16_t *const chan[18] = {
		&msg.chan1_raw,  &msg.chan2_raw,  &msg.chan3_raw,
		&msg.chan4_raw,  &msg.chan5_raw,  &msg.chan6_raw,
		&msg.chan7_raw,  &msg.chan8_raw,  &msg.chan9_raw,
		&msg.chan10_raw, &msg.chan11_raw, &msg.chan12_raw,
		&msg.chan13_raw, &msg.chan14_raw, &msg.chan15_raw,
		&msg.chan16_raw, &msg.chan17_raw, &msg.chan18_raw,
	};

	for (int i = 0; i < 18; i++) {
		*chan[i] = (i < RC_CHANNELS_N) ? rc_raw[i] : UINT16_MAX;
	}

	/*
	 * RSSI. The simulated link is perfect, but the value has to suit the
	 * CONSUMER, and the two scales here disagree:
	 *   - MAVLink RC_CHANNELS.rssi is 0-254 (254 = 100%), UINT8_MAX = unknown.
	 *   - PX4's input_rc.rssi is a PERCENTAGE, 0-100 (InputRc.msg: RSSI_MAX =
	 *     100, "100: full reception").
	 * SimulatorMavlink assigns rc_channels.rssi straight into input_rc.rssi with
	 * no rescaling, so the MAVLink-idiomatic 254 would reach PX4 as an rssi of
	 * 254 - 2.5x its own declared maximum. 100 is the value that is correct on
	 * the side that actually reads it, and is still a legal MAVLink value.
	 */
	msg.rssi = 100;

	return msg;
}

void VehicleState::setPXControls(const mavlink_hil_actuator_controls_t &controls)
{
	last_controls = controls;
	received_first_controls = true;
}
