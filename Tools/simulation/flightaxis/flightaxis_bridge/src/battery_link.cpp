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

/*
 * PER-CELL INTERNAL RESISTANCE, ESTIMATED ONLINE.
 *
 * The sag correction is `per_cell + I * R_cell`, which is exactly what PX4's
 * own Battery class does (battery.cpp:220-231), and BAT_R_INTERNAL is likewise
 * documented as a PER-CELL value (module.yaml:42-51). What PX4 does NOT do is
 * leave that number fixed: BAT_R_INTERNAL defaults to -1, meaning "estimate it
 * online", and battery.cpp:238-265 runs a recursive least-squares filter over
 * (current, voltage) pairs to do so.
 *
 * A fixed guess was the original defect here. Held at 5 mOhm/cell it produced a
 * 1.35 V/cell correction at the 270 A this quadplane actually draws - 10.8 V
 * across an 8S pack - which is far larger than the 0.90 V/cell window the SoC
 * map spans, so `remaining` pinned at 1.0 and the gauge stopped meaning
 * anything. The failure is not the formula, it is asserting R for a pack nobody
 * has measured.
 *
 * So estimate it, with the same algorithm and the same constants PX4 uses. The
 * seed below is only a starting point now, not an answer.
 */
constexpr double CELL_R_DEFAULT = 0.005;	// battery.h:195, initial per-cell estimate
constexpr double RLS_LAMBDA     = 0.95;		// battery.h:194, forgetting factor
constexpr double RLS_R_COV      = 0.1;		// battery.h:197, per-cell R covariance
constexpr double RLS_OCV_COV    = 1.5;		// battery.h:198, per-cell OCV covariance

// Ceiling on the estimate, from BAT_R_INTERNAL's own declared max (module.yaml:51).
// The estimator is fed simulator data and can be driven somewhere silly by a
// pathological frame; this bounds the damage.
constexpr double CELL_R_MAX = 0.2;

// Below this the sag correction is meaningless and the regressor is degenerate
// (current is the only excitation the estimator gets), so hold the estimate.
constexpr double RLS_MIN_CURRENT_A = 1.0;

// Time constant of the low-pass on the compensated per-cell voltage. The sag
// correction handles the sustained component; this handles the spikes that
// survive it. Sized well above the 500 ms send interval and well below a flight.
constexpr double SOC_FILTER_TAU_S = 3.0;

// Nominal pack used when the RealFlight model reports no battery at all.
constexpr int    SYNTHETIC_CELLS = 4;
constexpr double SYNTHETIC_V     = 4.0 * 4.10;

/*
 * PLAUSIBILITY FLOOR FOR A PROPULSION PACK.
 *
 * RealFlight uses -1 as the "this model does not have one" sentinel, and 0 also
 * shows up transiently (aircraft reset, the first frames after the controller is
 * injected, physics not yet stepped). Neither is a battery.
 *
 * 3.0 V is under a fully flat 1S cell, so nothing real is rejected, while every
 * sentinel is. The floor is NOT set high enough to reject a 4.8 V receiver pack,
 * because it cannot be: a 4.8 V NiMH and a 2S LiPo overlap, and no voltage
 * threshold separates them. The fuel field is what separates them - see below.
 */
constexpr double PACK_MIN_V = 3.0;

// A tank reading above this means the model HAS a tank. Same sentinel story:
// -1 for electrics, and a genuinely dry tank reads 0.0 - which is information,
// not absence, so it must not be mistaken for "no fuel system".
constexpr double FUEL_EPS_OZ = 1.0e-6;

// Frames of DISAGREEING evidence required to move an already-latched class.
// At the 2 Hz send rate this is ~1.5 s of consistent contradiction, which a
// model swap produces and a one-frame glitch does not.
constexpr int CLASS_CONFIRM_FRAMES = 3;

// How far below the candidate a confirming frame may sit and still count as
// "the tank is holding at a new maximum" rather than having fallen back to the
// level it stepped up from. Across the ~1.5 s the streak spans, a running
// engine takes a fraction of a percent off any real tank, while a one-frame
// spike drops the whole way back - so 5% separates the two with room to spare.
constexpr double FUEL_HOLD_FRACTION = 0.95;

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

	/*
	 * ======================= PROPULSION AUTO-DETECTION =======================
	 *
	 * Which of these two numbers is meaningful is decided HERE, from the frame,
	 * with no parameter and nothing for the user to set. A glow/gas/turbine
	 * model has to be right the moment it loads.
	 *
	 * THE RULE, and it is deliberately asymmetric:
	 *
	 *   a tank exists  -> FUEL, whatever the voltage field says
	 *   else a plausible pack voltage -> ELECTRIC
	 *   else -> undecided; report a nominal pack until something shows up
	 *
	 * Fuel wins over voltage rather than the other way round because the
	 * conflict case is real and only resolves one way. Plenty of RealFlight IC
	 * models carry an onboard receiver/ignition battery, so they can report BOTH
	 * a tank and a small positive voltage. Treating that 4.8 V NiMH as the
	 * propulsion pack is a disaster: the per-cell map calls it 1 cell, and the
	 * moment it reads below 4.2 V the aircraft is "flat" - remaining hits 0,
	 * PX4 raises EMERGENCY and takes away every navigation mode, mid-flight,
	 * on an aircraft whose tank is full. An electric model, by contrast, has no
	 * tank at all and reports fuel = -1, so it never enters this branch. The
	 * asymmetry costs electrics nothing and saves IC models from the exact
	 * failure mode this bridge already ate once.
	 *
	 * WHAT THIS CANNOT DISTINGUISH, stated plainly: a genuine hybrid - electric
	 * propulsion with a fuel-burning generator - would be classed as Fuel. Its
	 * tank is still the thing that ends the flight, so the answer is arguably
	 * right anyway, and RealFlight does not model such an aircraft.
	 *
	 * Once Electric or Fuel is latched it survives bad frames (see the streak
	 * counter). That matters most for a dry tank: fuel = 0 is the single most
	 * important reading an IC aircraft ever produces, and the old code read it
	 * as "no fuel system" and reported a nominal FULL pack - reporting 100% at
	 * the exact moment the engine is about to quit.
	 */
	const bool fuel_plausible = std::isfinite(fuel_oz) && fuel_oz > FUEL_EPS_OZ;
	const bool pack_plausible = std::isfinite(v_raw) && v_raw >= PACK_MIN_V;

	Source observed = Source::None;

	if (fuel_plausible) {
		observed = Source::Fuel;

	} else if (pack_plausible) {
		observed = Source::Electric;
	}

	if (observed == Source::None) {
		// No evidence this frame. Never a reason to abandon a latched class -
		// it is the normal reading for a dry tank and for a momentary dropout.
		_class_streak = 0;

	} else if (_source == Source::None) {
		// First real evidence: adopt immediately, no waiting. The very first
		// battery packet PX4 sees should already be the truth.
		adopt(observed);

	} else if (observed != _source) {
		// Sustained contradiction = the user loaded a different aircraft.
		if (++_class_streak >= CLASS_CONFIRM_FRAMES) {
			adopt(observed);
		}

	} else {
		_class_streak = 0;
	}

	double voltage_v = 0.0;
	double current_a = 0.0;
	double remaining = 1.0;
	int    cells     = SYNTHETIC_CELLS;
	uint8_t batt_type = MAV_BATTERY_TYPE_LIPO;
	Source source    = Source::Synthetic;

	if (_source == Source::Electric && !pack_plausible) {
		/*
		 * Latched electric, but THIS frame's voltage is a sentinel or garbage.
		 * Re-send the last values derived from a real sample instead of running
		 * a 0 V reading through the per-cell map. That map would return 0%,
		 * and PX4 latches the resulting EMERGENCY upward and never lowers it -
		 * so one bad frame would permanently end a flight on a full pack.
		 */
		source    = Source::Electric;
		voltage_v = _last_voltage_v;
		current_a = 0.0;
		remaining = _last_remaining;
		cells     = (_cell_count > 0) ? _cell_count : SYNTHETIC_CELLS;

	} else if (_source == Source::Electric) {
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

		/*
		 * Undo load sag BEFORE mapping voltage to state of charge.
		 *
		 * CELL_EMPTY_V/CELL_FULL_V describe a pack AT REST. Feeding them the
		 * loaded terminal voltage asks a resting curve about a working pack and
		 * gets the answer wrong in the one direction that matters: low. The
		 * error is worst exactly when current is highest, which for a quadplane
		 * is the hover - i.e. every takeoff.
		 *
		 * The window is only 0.90 V wide per cell, so the map is savagely
		 * sensitive down there: on an 8S pack, 27.5 V reads 15% and 26.9 V reads
		 * 7%. Six hundred millivolts across the whole pack - well inside hover
		 * sag on a healthy battery - is the entire distance from LOW to
		 * CRITICAL.
		 *
		 * That distance is a one-way trip. PX4 latches the warning upward while
		 * armed and never lowers it (batteryCheck.cpp:189-192), and at CRITICAL
		 * it marks every navigation mode unavailable (NavModes::All, ibid:205-207)
		 * - which is a full failsafe, reached with COM_LOW_BAT_ACT still on
		 * "warning only", because that parameter does not gate this path. So a
		 * SINGLE sagged sample ends the flight, and the pack it condemns may be
		 * nearly full.
		 *
		 * Estimating open-circuit voltage costs one multiply and removes the
		 * sustained component; the filter below removes what is left. The
		 * resistance it multiplies is estimated from this pack's own behaviour
		 * (see updateInternalResistance) rather than asserted.
		 */
		updateInternalResistance(voltage_v, current_a);

		const double per_cell_raw = voltage_v / (double)_cell_count;
		const double per_cell_ocv = per_cell_raw + current_a * _cell_r_internal;

		if (_per_cell_filt < 0.0 || dt_s <= 0.0) {
			// first sample: seed rather than filter, so the reported SoC starts
			// at the truth instead of ramping up to it from zero
			_per_cell_filt = per_cell_ocv;

		} else {
			const double alpha = dt_s / (SOC_FILTER_TAU_S + dt_s);
			_per_cell_filt += alpha * (per_cell_ocv - _per_cell_filt);
		}

		remaining = clamp((_per_cell_filt - CELL_EMPTY_V) / (CELL_FULL_V - CELL_EMPTY_V), 0.0, 1.0);

		_discharged_mah += current_a * dt_s * (1000.0 / 3600.0);

		cells           = _cell_count;
		_last_voltage_v = voltage_v;
		_last_remaining = remaining;

	} else if (_source == Source::Fuel && !(std::isfinite(fuel_oz) && fuel_oz >= 0.0)) {
		/*
		 * Latched fuel, but THIS frame's tank reading is a sentinel (-1) or
		 * garbage. The electric branch above holds for the same reason, but the
		 * condition here is deliberately NARROWER than `fuel_plausible`.
		 *
		 * `fuel_plausible` also rejects a tank at or near zero, and that is a
		 * real reading, not a bad one: an empty tank is precisely the state the
		 * fuel failsafe exists to act on, and holding it would report flying
		 * time remaining on an aircraft whose engine is about to stop. Only a
		 * negative or non-finite value is physically impossible; running one
		 * through the fraction below clamps `remaining` to 0, which PX4 latches
		 * to CRITICAL and never lowers.
		 *
		 * _fuel_last_oz is deliberately NOT updated from here either, so a
		 * sentinel cannot be mistaken for the step change that re-arms the tank
		 * reference.
		 */
		source    = Source::Fuel;
		voltage_v = SYNTHETIC_V;
		current_a = 0.0;
		cells     = SYNTHETIC_CELLS;
		batt_type = MAV_BATTERY_TYPE_UNKNOWN;
		remaining = _last_remaining;

	} else if (_source == Source::Fuel) {
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
		cells     = SYNTHETIC_CELLS;
		batt_type = MAV_BATTERY_TYPE_UNKNOWN;	// it is a tank, not a pack

		/*
		 * TANK CAPACITY: FlightAxis does not report one.
		 *
		 * m-fuelRemaining-OZ is instantaneous ounces and there is no companion
		 * "capacity" key anywhere in the ExchangeData reply, so a true percentage
		 * is not derivable. The honest fallback is the largest reading ever seen,
		 * taken as full - and it is normally exactly right, because the first
		 * frame after a model load or an aircraft reset is a full tank.
		 *
		 * Re-armed UPWARDS rather than latched so a refuel or a reset restores
		 * 100% instead of pinning the fraction low for the rest of the session.
		 *
		 * It also has to re-arm DOWNWARDS, because adopt() - which resets it -
		 * only fires on a CLASS change, and swapping one IC model for another
		 * keeps the class at Fuel. Without the branch below, a 40 oz glow model
		 * followed by a 10 oz one leaves the reference at 40: the new tank,
		 * brim full, reports 25% remaining, which is a low-battery failsafe on
		 * a full tank. That is the wrong direction to be wrong in.
		 *
		 * What separates a swap from ordinary burning: burning is a monotone
		 * decline away from the reference, whereas a swap is a STEP UP from
		 * whatever the previous model was showing - the sentinel/near-zero
		 * frames a reload passes through, or a part-used old tank - to a level
		 * still under the old reference, which then HOLDS. So the trigger is a
		 * rise, never a low reading on its own; a nearly dry tank only ever
		 * falls and can never re-arm the reference down onto itself.
		 *
		 * Confirmed over CLASS_CONFIRM_FRAMES the same way the class latch is,
		 * at the same 2 Hz gate, so a single spurious high frame - which drops
		 * straight back to the old level - is not mistaken for a new tank.
		 *
		 * Where it is wrong, and it cannot be fixed from here: if the bridge
		 * attaches to an aircraft that is ALREADY part-used, that partial tank
		 * becomes "full" and the reported percentage reads high until the next
		 * reset re-arms it. Load the model fresh and this never arises.
		 */
		if (fuel_oz > _fuel_full_oz) {
			_fuel_full_oz  = fuel_oz;
			_fuel_streak   = 0;
			_fuel_cand_oz  = 0.0;

		} else if (fuel_plausible && fuel_oz > _fuel_last_oz) {
			// A step up that does not reach the reference: candidate new tank.
			// Take the largest reading of the streak as the candidate, so a
			// swap caught mid-rise still ends up at the true full mark.
			if (fuel_oz > _fuel_cand_oz) {
				_fuel_cand_oz = fuel_oz;
			}

			++_fuel_streak;

		} else if (_fuel_streak > 0 && fuel_plausible
			   && fuel_oz >= _fuel_cand_oz * FUEL_HOLD_FRACTION) {
			// Still sitting at the candidate: the level is holding, not spiking.
			++_fuel_streak;

		} else {
			_fuel_streak = 0;
			_fuel_cand_oz = 0.0;
		}

		if (_fuel_streak >= CLASS_CONFIRM_FRAMES && _fuel_cand_oz > 0.0) {
			_fuel_full_oz = _fuel_cand_oz;
			_fuel_streak  = 0;
			_fuel_cand_oz = 0.0;
		}

		_fuel_last_oz = fuel_oz;

		remaining = (_fuel_full_oz > 0.0) ? clamp(fuel_oz / _fuel_full_oz, 0.0, 1.0) : 1.0;

		_last_voltage_v = voltage_v;
		_last_remaining = remaining;

		/*
		 * ENGINE-OUT: report it, do NOT act on it.
		 *
		 * A dead engine aloft is the defining emergency of an IC aircraft and
		 * the user should hear about it immediately - RealFlight's own cue is
		 * just the sound stopping. But it must not be folded into `remaining`:
		 * PX4 turns a low remaining into WARNING_CRITICAL and strips every
		 * navigation mode, and a dead-stick aircraft is still flyable and still
		 * needs those modes to reach a field. So this is a message and nothing
		 * more - the tank fraction stays the tank fraction.
		 *
		 * Gated on not-touching-ground so that a normal pre-start sit on the
		 * runway, and the shutdown after landing, stay silent.
		 */
		const bool running = fa.m_anEngineIsRunning;

		if (!_engine_known) {
			_engine_known   = true;
			_engine_running = running;

		} else if (running != _engine_running) {
			_engine_running = running;

			if (!fa.m_isTouchingGround) {
				fprintf(stderr, "[flightaxis_bridge] ENGINE %s IN FLIGHT (fuel %.1f%% remaining)\n",
					running ? "RESTARTED" : "STOPPED", remaining * 100.0);
			}
		}

	} else {
		// ---- Nothing has shown a pack or a tank yet -------------------------
		// The airframes turn battery_simulator off (SIM_BAT_ENABLE 0) so that
		// there is exactly ONE publisher on battery_status; that makes this
		// bridge responsible for always producing something. A model with no
		// electrical or fuel model at all still has to pass preflight, so send
		// a nominal full pack. Obviously synthetic, and logged as such.
		//
		// This does NOT write _cell_count. It used to, and that was a live bug
		// for electric models: any frame reaching here first - an aircraft
		// reset, the frames before RealFlight's electrical model starts
		// stepping - latched 4 cells permanently. A 3S pack then divided by 4
		// reads 2.78 V/cell, i.e. 0%, i.e. EMERGENCY on a full battery.
		source    = Source::Synthetic;
		voltage_v = SYNTHETIC_V;
		current_a = 0.0;
		remaining = 1.0;
		cells     = SYNTHETIC_CELLS;
	}

	if (source != _reported) {
		const char *name = "unknown";

		switch (source) {
		case Source::Electric:
			name = "electric (RealFlight pack voltage/current)";
			break;

		case Source::Fuel:
			name = "fuel / internal combustion (tank drives remaining; "
			       "the voltage reported to PX4 is NOMINAL, not measured)";
			break;

		case Source::Synthetic:
			name = "synthetic (no pack and no tank seen yet; nominal full)";
			break;

		default:
			break;
		}

		fprintf(stderr, "[flightaxis_bridge] battery source: %s\n", name);

		/*
		 * FlightAxis names the field m-fuelRemaining-OZ, but nothing upstream
		 * consumes it - ArduPilot parses the key and never reads it - so the
		 * unit is asserted by the name alone. It does not matter to the code
		 * below, which divides by the largest value it has seen and is
		 * therefore correct whether RealFlight reports ounces, percent or a
		 * 0..1 fraction. It does matter to anyone reading the number, so print
		 * the raw reading alongside the reference it is being measured against:
		 * a full tank showing 100.0 or 1.000 settles the question in one flight.
		 */
		if (source == Source::Fuel) {
			fprintf(stderr, "[flightaxis_bridge]   tank: %.3f raw (m-fuelRemaining-OZ), "
				"reference %.3f -> %.0f%%\n",
				fuel_oz, _fuel_full_oz,
				(_fuel_full_oz > 0.0) ? 100.0 * remaining : 100.0);
		}

		_reported = source;
	}

	send(voltage_v, current_a, remaining, _discharged_mah, cells, batt_type, now_us);
}

/*
 * RECURSIVE LEAST SQUARES ON (CURRENT, VOLTAGE) -> (OCV, INTERNAL RESISTANCE).
 *
 * Ported from PX4's Battery::updateInternalResistanceEstimation
 * (battery.cpp:238-265) with the same constants, so that a pack behaves the
 * same way here as it would behind a real power module. Written out in scalars
 * because the state is 2x1 and the covariance 2x2; pulling in a matrix library
 * for that would obscure rather than clarify.
 *
 * Model: V = OCV - I*R, regressor x = [1, -I], state est = [OCV, R_pack].
 *
 * Two details are load-bearing and both are PX4's, not inventions:
 *
 *  - The covariance-norm gate. An update is accepted ONLY if it shrinks the
 *    covariance. Without it a long hover at constant current - which is most of
 *    a quadplane flight - is a degenerate regressor: every sample carries the
 *    same information, and the filter happily drifts along the one direction
 *    the data cannot constrain. When the update is rejected the OCV term is
 *    still re-centred on the current reading, so the estimate keeps tracking
 *    the pack draining without touching R.
 *
 *  - The forgetting factor 0.95, which lets R follow a pack warming up.
 *
 * Held below RLS_MIN_CURRENT_A because current is the only excitation there is:
 * at zero current the regressor is [1, 0], which says nothing about R at all.
 */
void BatteryLink::updateInternalResistance(double voltage_v, double current_a)
{
	if (_cell_count < 1 || current_a < RLS_MIN_CURRENT_A) {
		return;
	}

	if (!_rls_init) {
		_rls_init  = true;
		_rls_ocv   = voltage_v;
		_rls_r     = CELL_R_DEFAULT * _cell_count;
		_rls_p00   = RLS_OCV_COV * _cell_count;
		_rls_p01   = 0.0;
		_rls_p10   = 0.0;
		_rls_p11   = RLS_R_COV * _cell_count;
		_rls_p_norm = std::sqrt(_rls_p00 * _rls_p00 + 2.0 * _rls_p10 * _rls_p10 + _rls_p11 * _rls_p11);
		return;
	}

	// x = [1, -I]
	const double x1 = -current_a;

	// P*x
	const double px0 = _rls_p00 + _rls_p01 * x1;
	const double px1 = _rls_p10 + _rls_p11 * x1;

	// x'*P*x
	const double xpx = px0 + x1 * px1;

	const double denom = RLS_LAMBDA + xpx;

	if (!(std::fabs(denom) > 1e-12)) {
		return;
	}

	const double g0 = px0 / denom;
	const double g1 = px1 / denom;

	const double err = voltage_v - (_rls_ocv + x1 * _rls_r);

	// x'*P (row)
	const double xtp0 = _rls_p00 + x1 * _rls_p10;
	const double xtp1 = _rls_p01 + x1 * _rls_p11;

	const double p00 = (_rls_p00 - g0 * xtp0) / RLS_LAMBDA;
	const double p01 = (_rls_p01 - g0 * xtp1) / RLS_LAMBDA;
	const double p10 = (_rls_p10 - g1 * xtp0) / RLS_LAMBDA;
	const double p11 = (_rls_p11 - g1 * xtp1) / RLS_LAMBDA;

	const double p_norm = std::sqrt(p00 * p00 + 2.0 * p10 * p10 + p11 * p11);

	const double ocv_new = _rls_ocv + g0 * err;
	const double r_new   = _rls_r   + g1 * err;

	// A non-finite state is rejected the same way a non-improving covariance is,
	// rather than being written and then papered over downstream. Substituting
	// 0 Ohm for a non-finite R - which is what this used to do - throws away a
	// good estimate AND disables the sag correction outright, while the branch
	// immediately below keeps R for the far milder case of an uninformative
	// sample. Worse, the poisoned _rls_r stayed in the filter, so every later
	// sample produced a non-finite R too and the substitution became permanent.
	if (std::isfinite(p_norm) && p_norm < _rls_p_norm
	    && std::isfinite(ocv_new) && std::isfinite(r_new)) {
		_rls_ocv    = ocv_new;
		_rls_r      = r_new;
		_rls_p00    = p00;
		_rls_p01    = p01;
		_rls_p10    = p10;
		_rls_p11    = p11;
		_rls_p_norm = p_norm;

		_cell_r_internal = clamp(_rls_r / (double)_cell_count, 0.0, CELL_R_MAX);

	} else {
		// Covariance did not improve: keep R, re-centre OCV on this sample.
		_rls_ocv = voltage_v + _rls_r * current_a;
	}
}

void BatteryLink::adopt(Source s)
{
	_source       = s;
	_class_streak = 0;

	// Every accumulator below is per-class. Carrying any of them across a model
	// swap is worse than starting over: a stale cell count mis-scales the new
	// pack, and a stale tank size mis-scales the new tank.
	_cell_count     = 0;
	_per_cell_filt  = -1.0;

	// The resistance estimate belongs to the pack that was measured, not to
	// whatever is loaded next. Re-seed rather than carry it over.
	_cell_r_internal = CELL_R_DEFAULT;
	_rls_init        = false;
	_fuel_full_oz   = 0.0;
	_fuel_last_oz   = 0.0;
	_fuel_cand_oz   = 0.0;
	_fuel_streak    = 0;
	_discharged_mah = 0.0;
	_last_voltage_v = 0.0;
	_last_remaining = 1.0;
	_engine_known   = false;
	_engine_running = false;
}

void BatteryLink::send(double voltage_v, double current_a, double remaining,
		       double discharged_mah, int cells, uint8_t batt_type,
		       uint64_t now_us)
{
	_last_send_us = now_us;

	if (cells < 1) {
		cells = 1;
	}

	if (cells > 10) {
		cells = 10;
	}

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
		batt_type,				// LIPO, or UNKNOWN for a fuel tank
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

bool BatteryLink::requestPositionReset(double lat_deg, double lon_deg, double accuracy_m, uint64_t now_us)
{
	if (_fd < 0) {
		return false;
	}

	mavlink_message_t msg;

	// param2 is the age of the observation in seconds - zero, this is now.
	// param3 is the claimed one-sigma accuracy: under a metre is what selects
	// the hard reset over a fused update. param7 (altitude) must be NaN; the
	// message does not carry height and EKF2 checks only lat/lon for finiteness.
	mavlink_msg_command_long_pack_chan(
		_sysid, BATTERY_COMPID, MAVLINK_COMM_1, &msg,
		1,					// target_system: the autopilot
		1,					// target_component
		43003,					// MAV_CMD_EXTERNAL_POSITION_ESTIMATE
		0,					// confirmation
		(float)(now_us * 1e-6),			// param1: sender timestamp
		0.0f,					// param2: processing delay
		(float)accuracy_m,			// param3: accuracy, metres
		0.0f,					// param4: unused
		(float)lat_deg,				// param5
		(float)lon_deg,				// param6
		NAN);					// param7: altitude not supported

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	const int len = mavlink_msg_to_send_buffer(buf, &msg);

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = htons((uint16_t)(PORT_BASE + _instance));

	return ::sendto(_fd, buf, (size_t)len, MSG_DONTWAIT,
			(struct sockaddr *)&addr, sizeof(addr)) == len;
}
