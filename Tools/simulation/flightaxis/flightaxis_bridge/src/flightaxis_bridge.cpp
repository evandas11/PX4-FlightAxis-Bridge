/****************************************************************************
 *
 * This file is part of the PX4-FlightAxis-Bridge project.
 * Copyright (c) 2026 Evangels Brilliant Dasmasela
 *
 * The three-branch physics-time handling (restart / duplicate-frame
 * extrapolation / glitch compensation) and the Rev4Servos and HeliDemix
 * transforms in this file are ported from ArduPilot
 * libraries/SITL/SIM_FlightAxis.{h,cpp} update() and exchange_data()
 * (Copyright (C) ArduPilot Dev Team, GPLv3). Because this file is a derivative
 * work of that GPLv3 code, it is itself licensed under GPLv3 or later:
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
 * @file flightaxis_bridge.cpp
 *
 * FlightAxis (RealFlight) <-> PX4 SITL bridge, main loop.
 *
 * The three-branch physics-time handling (restart / duplicate-frame
 * extrapolation / glitch compensation) is a literal port of the upstream
 * implementation named in the licence header above.
 *
 * argv protocol (produced by get_FAbridge_params.py from models/<m>.json):
 *   flightaxis_bridge <instance> <ip> <options_bitmask> <unmapped_default>
 *                     <nchannels> [rf_idx px4_idx scale reverse disarm]*nchannels
 *   options bits: 1=ResetPosition 2=Rev4Servos 4=HeliDemix 8=SilenceFPS
 *   scale:   0=unipolar (clamp 0..1), 1=bipolar ((v+1)/2)
 *   reverse: 0/1, applied after scaling (v -> 1-v)
 *   disarm:  0..1 value sent when disarmed or control is NaN,
 *            or -1 = hold last output (neutral 0.5 before the first one)
 *
 * The argv protocol above is UNCHANGED and remains positional, so existing
 * callers (get_FAbridge_params.py output spliced by sitl_run.sh) keep working
 * verbatim. Transport selection is done with environment variables instead of
 * new argv slots: the bridge already takes its home position that way
 * (PX4_HOME_LAT/LON/ALT) and sitl_run.sh already passes PX4_FLIGHTAXIS_IP, so
 * this stays consistent, and it avoids an optional leading flag that every
 * positional index downstream would have to account for.
 *
 * Environment (all optional; defaults reproduce the original SITL behaviour):
 *
 *   PX4_HITL_TRANSPORT   tcp-server (default) | serial | udp
 *                        tcp-server = SITL: listen on 4560+instance.
 *                        serial/udp = HITL: talk to a real board.
 *   PX4_HITL_SERIAL_DEV  serial device, e.g. /dev/ttyACM0 (serial only)
 *   PX4_HITL_SERIAL_BAUD baud, default 921600 (serial only; ignored by USB
 *                        CDC-ACM, where the rate is nominal)
 *   PX4_HITL_UDP_HOST    board IP (udp only)
 *   PX4_HITL_UDP_PORT    board UDP port, default 14550 (udp only)
 *   PX4_HITL_SENSOR_HZ   HIL_SENSOR rate. Default 250 for serial/udp,
 *                        0 (= every frame, ~1 kHz) for tcp-server. 0 forces
 *                        every frame on any transport - only do that on USB.
 *   PX4_HITL_STATE_QUAT_BYPASS
 *                        if set (to anything), re-enable HIL_STATE_QUATERNION
 *                        in HITL. DEBUG ONLY - it makes the board fly on
 *                        injected truth instead of its own estimator.
 *
 * Selecting serial or udp also selects the HITL message profile, which:
 *   - stops sending HIL_STATE_QUATERNION (it would race EKF2 on a real board)
 *   - stops sending RAW_RPM (no receiver handler exists for it)
 *   - stops sending RC_CHANNELS (no RC passthrough to a real board)
 *   - drops HIL_GPS to 5 Hz and mag/baro/airspeed to 50 Hz
 *   - adds a HEARTBEAT, without which PX4 never brings a USB link up
 * These are correctness requirements on a real board, not bandwidth tweaks -
 * see px4_communicator.h.
 */

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <limits>

#include <time.h>
#include <signal.h>
#include <unistd.h>

#include "px4_communicator.h"
#include "fa_communicator.h"
#include "vehicle_state.h"
#include "battery_link.h"

using namespace std;

// Distinct exit code so sitl_run.sh can tell a requested restart from a crash.
static constexpr int EXIT_RESTART_REQUESTED = 42;

// options bitmask
static const uint32_t OPT_RESET_POSITION = 1;
static const uint32_t OPT_REV4_SERVOS    = 2;
static const uint32_t OPT_HELI_DEMIX     = 4;
static const uint32_t OPT_SILENCE_FPS    = 8;

static const int RF_CHANNELS = 12;

struct ChannelMap {
	int rf;			// RealFlight channel slot 0..11
	int px4;		// index into HIL_ACTUATOR_CONTROLS.controls[]
	int scale;		// 0=unipolar, 1=bipolar
	int reverse;		// 0/1
	double disarm;		// 0..1, or -1 = hold last/neutral
};

static volatile sig_atomic_t stop = 0;

static void termSignalHandler(int)
{
	stop = 1;
}

static void intSignalHandler(int)
{
	cerr << "[flightaxis_bridge] Signal SIGINT received" << endl;
	stop = 1;
}

static void setup_unix_signals()
{
	struct sigaction term;
	term.sa_handler = termSignalHandler;
	sigemptyset(&term.sa_mask);
	term.sa_flags |= SA_RESTART;

	if (sigaction(SIGTERM, &term, nullptr)) {
		cerr << "[flightaxis_bridge] Error when setting SIGTERM handler" << endl;
	}

	// ignore pipe error - handled by return codes in the communicators
	signal(SIGPIPE, SIG_IGN);

	struct sigaction term2;
	term2.sa_handler = intSignalHandler;
	sigemptyset(&term2.sa_mask);
	term2.sa_flags |= SA_RESTART;

	if (sigaction(SIGINT, &term2, nullptr)) {
		cerr << "[flightaxis_bridge] Error when setting SIGINT handler" << endl;
	}
}

static double constrain(double v, double lo, double hi)
{
	return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// wall-clock microseconds, monotonic. Used to rate-limit retry/log paths and to
// pace the keep-alive stream while RealFlight's physics clock is not advancing.
static uint64_t micros()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}

static double envOrDefault(const char *name, double dflt)
{
	const char *s = getenv(name);

	if (s != nullptr && s[0] != '\0') {
		return atof(s);
	}

	return dflt;
}

/*
 * Report RealFlight model health: m_hasLostComponents, m_isLocked and
 * m_anEngineIsRunning.
 *
 * WHY THESE ARE LOGGED AND NOT FORWARDED INTO A uORB TOPIC
 * --------------------------------------------------------
 * These three are the only parsed FlightAxis fields that carry state PX4 has
 * genuinely no other way to learn - everything else RealFlight reports is
 * either already sent (attitude, velocity, position) or is a redundant
 * re-expression of something already sent. A crashed model, a locked-up model
 * and a healthy model are byte-for-byte identical from PX4's side: the
 * physics keeps streaming, the attitude keeps updating, and the aircraft
 * simply stops responding to controls.
 *
 * There is nonetheless no uORB topic that means any of this. The closest
 * candidates all lie: publishing to a failure/health topic would make PX4's
 * own failure detector and health checks act on a SIMULATOR bookkeeping flag
 * as though it were a vehicle fault, changing arming and failsafe behaviour
 * on the strength of something the flight stack should not be reasoning about
 * at all. Forwarding a value into a topic nobody consumes is worse than
 * leaving it parsed; forwarding it into a topic that DOES consume it, and
 * consumes it wrongly, is worse still.
 *
 * So the consumer is the human reading the bridge log. That is the party who
 * actually needs to know, and this is the difference between "my aircraft
 * stopped responding, is the bridge broken?" and one line saying the model
 * shed a wing forty seconds ago.
 *
 * EDGE TRIGGERED. The main loop turns at 750-800 Hz; level-triggered logging
 * here would emit thousands of identical lines per second and is exactly the
 * kind of thing that competes with the exchange budget. Every branch below is
 * a compare against the previous frame's value.
 */
struct ModelHealth {
	bool  initialised{false};
	bool  lost_components{false};
	bool  locked{false};		// last REPORTED lock state
	bool  engine_running{false};

	// m_isLocked debounce / flap suppression - see reportModelHealth().
	bool     lock_raw{false};		// last RAW value seen from RealFlight
	uint64_t lock_raw_since_us{0};		// when lock_raw last changed
	unsigned lock_flaps{0};			// raw transitions in the current window
	uint64_t lock_window_us{0};		// start of the flap-counting window
	bool     lock_flap_warned{false};
};

static void reportModelHealth(ModelHealth &prev, const FAState &state)
{
	if (!prev.initialised) {
		prev.initialised     = true;
		prev.lost_components = state.m_hasLostComponents;
		prev.locked          = state.m_isLocked;
		prev.engine_running  = state.m_anEngineIsRunning;
		prev.lock_raw        = state.m_isLocked;

		// Announce the starting condition only when it is already bad -
		// a healthy model at startup is the expected case and needs no line.
		if (state.m_hasLostComponents) {
			cerr << "[flightaxis_bridge] WARNING: RealFlight model has already LOST COMPONENTS at startup"
			     << " - reset the aircraft in RealFlight (spacebar) before flying" << endl;
		}

		if (state.m_isLocked) {
			cerr << "[flightaxis_bridge] WARNING: RealFlight model is LOCKED at startup"
			     << " - it will not respond to controls" << endl;
		}

		return;
	}

	if (state.m_hasLostComponents != prev.lost_components) {
		if (state.m_hasLostComponents) {
			// The single most useful line this bridge can print. Everything
			// downstream still looks nominal after this: PX4 keeps receiving
			// well-formed sensor data and keeps commanding actuators, but the
			// airframe that was flying no longer exists.
			cerr << "[flightaxis_bridge] *** RealFlight model has LOST COMPONENTS (crash/breakup) ***" << endl;
			cerr << "[flightaxis_bridge]     PX4 cannot detect this - sensors keep streaming and the"
			     << " aircraft looks healthy from the flight stack's side." << endl;
			cerr << "[flightaxis_bridge]     Reset the aircraft in RealFlight (spacebar) to continue." << endl;

		} else {
			cerr << "[flightaxis_bridge] RealFlight model components restored (aircraft reset)" << endl;
		}

		prev.lost_components = state.m_hasLostComponents;
	}

	/*
	 * m_isLocked is DEBOUNCED; the other two flags are reported on the raw edge.
	 *
	 * WHY ONLY THIS ONE. m_hasLostComponents and m_anEngineIsRunning are latched
	 * physical events - a wing comes off once, an engine quits once - so a raw
	 * edge is exactly the right trigger. m_isLocked is different: it has been
	 * observed alternating several times per second in real flight, which
	 * produced ~20 contradictory lines in the seconds around a failsafe landing
	 * and buried the messages that actually explained the flight.
	 *
	 * Nothing in this bridge ACTS on m_isLocked - it is logged and nowhere else
	 * (and ArduPilot, whose parse table we mirror key-for-key, parses the field
	 * into its state struct and then never reads it at all). So a flapping value
	 * cannot affect control; it can only affect the log. That makes the log the
	 * correct place to fix it, and makes suppression safe: no control path is
	 * being deprived of information, because no control path consumes it.
	 *
	 * A change is reported only once the new value has been STABLE for
	 * LOCK_DEBOUNCE_US. A genuine lock - the case worth knowing about, where the
	 * model really has stopped responding - persists, so it still gets reported,
	 * a quarter second late. Flicker never survives the filter.
	 *
	 * The raw transitions are still counted, because "this value is flapping" is
	 * itself a diagnosis worth one line. It is emitted at most once per window
	 * so it cannot become the flood it replaces.
	 */
	{
		const uint64_t LOCK_DEBOUNCE_US    =  250000;	// stable-for before reporting
		const uint64_t LOCK_FLAP_WINDOW_US = 5000000;	// flap-counting window
		const unsigned LOCK_FLAP_LIMIT     = 4;		// raw changes/window = flapping

		const uint64_t now_us = micros();

		if (state.m_isLocked != prev.lock_raw) {
			prev.lock_raw = state.m_isLocked;
			prev.lock_raw_since_us = now_us;

			// restart the window if the last one has expired
			if (prev.lock_window_us == 0 || now_us - prev.lock_window_us > LOCK_FLAP_WINDOW_US) {
				prev.lock_window_us = now_us;
				prev.lock_flaps = 0;
				prev.lock_flap_warned = false;
			}

			prev.lock_flaps++;

			if (prev.lock_flaps >= LOCK_FLAP_LIMIT && !prev.lock_flap_warned) {
				prev.lock_flap_warned = true;
				cerr << "[flightaxis_bridge] NOTE: RealFlight m-isLocked is FLAPPING ("
				     << prev.lock_flaps << "+ changes in "
				     << (unsigned)(LOCK_FLAP_WINDOW_US / 1000) << " ms)"
				     << " - suppressing per-change lines; this flag is advisory only"
				     << " and no control path uses it" << endl;
			}
		}

		// report only a value that has settled
		if (prev.lock_raw != prev.locked && now_us - prev.lock_raw_since_us >= LOCK_DEBOUNCE_US) {
			if (prev.lock_raw) {
				cerr << "[flightaxis_bridge] *** RealFlight model is LOCKED - it will not respond to controls ***" << endl;

			} else {
				cerr << "[flightaxis_bridge] RealFlight model unlocked" << endl;
			}

			prev.locked = prev.lock_raw;
		}
	}

	if (state.m_anEngineIsRunning != prev.engine_running) {
		// Not a fault either way: it is normal before takeoff and after a
		// deliberate shutdown. It is logged because an engine that quit on
		// its own - flooded, out of fuel, or never started because the model
		// needs a manual ignition - presents to PX4 as an aircraft that
		// simply will not climb, with no other clue anywhere.
		cerr << "[flightaxis_bridge] RealFlight engine "
		     << (state.m_anEngineIsRunning ? "RUNNING" : "STOPPED") << endl;
		prev.engine_running = state.m_anEngineIsRunning;
	}
}

/*
 * Seed the persistent channels[] with the state a DISARMED aircraft should be
 * in: each mapped slot at its own "disarm" value, everything else at
 * UnmappedDefault. A hold-last row (disarm == -1) has no value of its own, so
 * it gets 0.5 - neutral for a bipolar surface, and the only defensible guess
 * for a slot whose whole configuration is "keep whatever was there".
 *
 * Called twice, and the second call is the one that matters. At startup this is
 * just initialisation. It is called AGAIN on the RealFlight reset edge
 * (spacebar respawn), because otherwise nothing in the pipeline ever clears
 * channels[] and a hold-last slot carries its PRE-CRASH value across the
 * respawn and holds it indefinitely: a plane reset after a nose-up divergence
 * comes back with rf1 (elevator) pinned at 0.0 - full deflection - on an
 * aircraft sitting still on the runway, and it stays there through the disarm. The respawn is precisely the
 * moment the previous flight's state stops being a description of anything, so
 * that is where it has to go.
 *
 * This CANNOT disarm anything and deliberately does not try. It only rewrites
 * the values the bridge sends to RealFlight. If PX4 is still armed and still
 * commanding, the very next buildChannels() overwrites every mapped slot with
 * live output on the first frame after the reset flag clears - the seed is what
 * is on the wire during the reset, not a new policy about arming. Whether the
 * aircraft is armed stays where it belongs, with the pilot.
 */
static void seedDisarmChannels(const ChannelMap *maps, int nmaps, double unmapped_default,
			       double channels[RF_CHANNELS])
{
	for (int i = 0; i < RF_CHANNELS; i++) {
		channels[i] = constrain(unmapped_default, 0.0, 1.0);
	}

	for (int i = 0; i < nmaps; i++) {
		if (maps[i].rf >= 0 && maps[i].rf < RF_CHANNELS) {
			channels[maps[i].rf] = (maps[i].disarm >= 0.0) ? constrain(maps[i].disarm, 0.0, 1.0) : 0.5;
		}
	}
}

/*
 * Build the 12 RealFlight channel values (0..1) from the latest
 * HIL_ACTUATOR_CONTROLS through the JSON channel map, then apply the
 * Rev4Servos / HeliDemix options, in that order.
 *
 * channels[] is PERSISTENT and holds the UNTRANSFORMED per-channel state, so
 * that disarm=-1 ("hold last output") has something stable to hold. out[] is
 * what actually goes to RealFlight.
 *
 * The two must not be the same array. Applying the option post-passes in place
 * re-transforms every hold-last channel on every frame, because the held value
 * that comes back round is already transformed:
 *  - HeliDemix diverges. Feeding demixed swash values back through the demix
 *    compounds without bound: three held channels
 *    go 0.74/0.28/0.51 -> 0.55/0.40/0.72 -> 0.98/0.70/0.64 on successive
 *    frames. Neutral is a fixed point, which is why it only shows up on the
 *    armed->disarmed transition with a non-neutral swash - i.e. every landing.
 *  - Rev4Servos does not diverge (the swap is its own inverse) but makes a
 *    hold-last channel PING-PONG between rf i and rf i+4 on alternate frames:
 *    a full-amplitude servo buzz at the loop rate.
 * Armed mapped rows happen to be safe (the fresh write dominates) and unmapped
 * rows are re-seeded from unmapped_default each call, so only hold-last rows
 * are exposed - which is exactly the case that is silent until someone watches
 * the servos. Transforming out of a scratch copy makes the post-passes
 * idempotent across repeated frames, which is what hold-last requires.
 */
static void buildChannels(const VehicleState &veh, const ChannelMap *maps, int nmaps,
			  double unmapped_default, uint32_t options, double channels[RF_CHANNELS],
			  double out[RF_CHANNELS])
{
	bool mapped[RF_CHANNELS] = {};

	for (int i = 0; i < nmaps; i++) {
		if (maps[i].rf >= 0 && maps[i].rf < RF_CHANNELS) {
			mapped[maps[i].rf] = true;
		}
	}

	for (int i = 0; i < RF_CHANNELS; i++) {
		if (!mapped[i]) {
			channels[i] = constrain(unmapped_default, 0.0, 1.0);
		}
	}

	const bool have_controls = veh.receivedFirstControls();
	const bool armed = have_controls && veh.armed();
	const mavlink_hil_actuator_controls_t &c = veh.lastControls();

	for (int i = 0; i < nmaps; i++) {
		const ChannelMap &m = maps[i];

		if (m.rf < 0 || m.rf >= RF_CHANNELS) {
			continue;
		}

		double v = NAN;

		if (have_controls && m.px4 >= 0 && m.px4 < 16) {
			v = c.controls[m.px4];
		}

		if (!armed || std::isnan(v)) {
			if (m.disarm >= 0.0) {
				channels[m.rf] = constrain(m.disarm, 0.0, 1.0);
			}

			// disarm == -1: hold last output (channels[] already has it)
			continue;
		}

		// NOT named `out`: that shadows the out[] parameter, in the one
		// function whose whole point is that channels[] and out[] are
		// different arrays.
		double scaled;

		if (m.scale == 1) {
			scaled = (v + 1.0) / 2.0;	// bipolar surface, -1..1 -> 0..1

		} else {
			scaled = v;			// unipolar motor, already 0..1
		}

		if (m.reverse) {
			scaled = 1.0 - scaled;
		}

		channels[m.rf] = constrain(scaled, 0.0, 1.0);
	}

	// From here on the option post-passes read the untransformed channels[] and
	// write only out[]; channels[] keeps the hold-last state (see above).
	memcpy(out, channels, sizeof(double) * RF_CHANNELS);

	if (options & OPT_REV4_SERVOS) {
		// swap first 4 and last 4 servos, for quadplane RF models
		memcpy(&out[0], &channels[4], sizeof(double) * 4);
		memcpy(&out[4], &channels[0], sizeof(double) * 4);
	}

	if (options & OPT_HELI_DEMIX) {
		// FlightAxis expects "roll/pitch/collective" input; PX4 outputs swash
		// servos. Read the swash from out[] so that Rev4Servos, if also set,
		// still composes in the right order (Rev4Servos first, then the demix)
		// - but never from a previous frame's demixed result.
		double swash1 = out[0];
		double swash2 = out[1];
		double swash3 = out[2];

		/*
		 * GAIN NORMALISATION. The raw differences below are not unit-gain, and
		 * feeding them to RealFlight straight OVER-DRIVES the cyclic.
		 *
		 * The swash servos sit at CA_SP0_ANG = 300 / 60 / 180 degrees with
		 * equal arm lengths (the airframe pins those angles for exactly this
		 * reason - CHANGING THEM INVALIDATES THE CONSTANTS BELOW). With the
		 * 0.5 per-servo scale that geometry gives
		 *   swash1 - swash2               = 2*sin(60) * 0.5 = 0.866 * roll
		 *   (swash1+swash2)/2 - swash3    = 1.5       * 0.5 = 0.750 * pitch
		 * Measured through live PX4 output: commanding 3 rad/s at FF 0.1
		 * (torque 0.3) produced rf0 = 0.7600 and rf1 = 0.7250, i.e. exactly
		 * 0.866*0.3 + 0.5 and 0.75*0.3 + 0.5.
		 *
		 * RealFlight wants a full command to reach 1.0, i.e. a gain of 0.5
		 * about the 0.5 centre. At 0.866 and 0.75 the cyclic instead saturates
		 * at only +/-0.577 of roll torque and +/-0.667 of pitch. So divide by
		 * 0.866/0.5 = sqrt(3) = 1.732 and 0.75/0.5 = 1.5.
		 *
		 * NOT by 0.866 and 0.75 - that recovers raw torque and makes it worse
		 * (a full command would land at 1.5 and clip at half command).
		 *
		 * Collective is (s1+s2+s3)/3, which is already exactly gain 0.5 and is
		 * deliberately left alone.
		 */
		const double HELI_ROLL_GAIN  = 1.732;	// sqrt(3)
		const double HELI_PITCH_GAIN = 1.500;	// 3/2

		double roll_rate  = (swash1 - swash2) / HELI_ROLL_GAIN;
		double pitch_rate = ((swash1 + swash2) / 2.0 - swash3) / HELI_PITCH_GAIN;
		double col        = (swash1 + swash2 + swash3) / 3.0;

		out[0] = constrain(roll_rate + 0.5, 0.0, 1.0);
		out[1] = constrain(pitch_rate + 0.5, 0.0, 1.0);
		out[2] = constrain(col, 0.0, 1.0);
	}
}

int main(int argc, char **argv)
{
	cerr << "[flightaxis_bridge] MAVLink to FlightAxis (RealFlight) bridge" << endl;

	if (argc < 6) {
		cerr << "Use: flightaxis_bridge <instance> <ip> <options_bitmask> <unmapped_default>"
		     << " <nchannels> [rf_idx px4_idx scale reverse disarm]*nchannels" << endl;
		return -1;
	}

	const int instance = atoi(argv[1]);
	const char *fa_ip = argv[2];
	const uint32_t options = (uint32_t)strtoul(argv[3], nullptr, 0);
	const double unmapped_default = atof(argv[4]);
	const int nmaps = atoi(argv[5]);

	if (nmaps < 0 || nmaps > RF_CHANNELS || argc < 6 + 5 * nmaps) {
		cerr << "[flightaxis_bridge] bad channel map: expected " << (6 + 5 * nmaps)
		     << " args, got " << argc << endl;
		return -1;
	}

	ChannelMap *maps = new ChannelMap[nmaps > 0 ? nmaps : 1];

	cout << "[flightaxis_bridge] options=0x" << hex << options << dec
	     << " unmapped_default=" << unmapped_default << " channels=" << nmaps << endl;

	for (int i = 0; i < nmaps; i++) {
		maps[i].rf      = atoi(argv[6 + 5 * i + 0]);
		maps[i].px4     = atoi(argv[6 + 5 * i + 1]);
		maps[i].scale   = atoi(argv[6 + 5 * i + 2]);
		maps[i].reverse = atoi(argv[6 + 5 * i + 3]);
		maps[i].disarm  = atof(argv[6 + 5 * i + 4]);

		cout << "  rf" << maps[i].rf << " <- px4[" << maps[i].px4 << "] "
		     << (maps[i].scale ? "bipolar" : "unipolar")
		     << (maps[i].reverse ? " reversed" : "")
		     << " disarm=" << maps[i].disarm << endl;
	}

	// a duplicate rf index would silently last-wins in buildChannels(); a
	// duplicate px4 index is almost always a typo in the model JSON
	for (int i = 0; i < nmaps; i++) {
		for (int j = i + 1; j < nmaps; j++) {
			if (maps[i].rf == maps[j].rf) {
				cerr << "[flightaxis_bridge] bad channel map: RealFlight channel rf"
				     << maps[i].rf << " is mapped twice (entries " << i << " and " << j
				     << ") - fix the model JSON" << endl;
				delete [] maps;
				return -1;
			}

			if (maps[i].px4 == maps[j].px4) {
				cerr << "[flightaxis_bridge] bad channel map: PX4 control index px4["
				     << maps[i].px4 << "] is mapped twice (entries " << i << " and " << j
				     << ") - fix the model JSON" << endl;
				delete [] maps;
				return -1;
			}
		}
	}

	// home position (anchors the RealFlight world in PX4's geodetic frame)
	const double home_lat = envOrDefault("PX4_HOME_LAT", 47.397742);
	const double home_lon = envOrDefault("PX4_HOME_LON", 8.545594);
	const double home_alt = envOrDefault("PX4_HOME_ALT", 488.0);
	// ... and the heading it starts on. Unset means "leave the RF world where
	// it is", which is the historical behaviour, so the default is NAN rather
	// than 0 - 0 is a legitimate request for due north and must not be what an
	// absent variable looks like.
	const double home_yaw = envOrDefault("PX4_HOME_YAW",
					     std::numeric_limits<double>::quiet_NaN());

	FACommunicator fa(fa_ip);
	VehicleState vehicle(home_lat, home_lon, home_alt, home_yaw);
	PX4Communicator px4(&vehicle);

	// ---- transport selection (see the header comment) -------------------
	const char *transport_env = getenv("PX4_HITL_TRANSPORT");
	PX4Transport transport = PX4Transport::TcpServer;
	PX4Profile prof = PX4Profile::Sitl;

	if (transport_env != nullptr && transport_env[0] != '\0'
	    && strcmp(transport_env, "tcp-server") != 0) {
		if (strcmp(transport_env, "serial") == 0) {
			transport = PX4Transport::Serial;
			prof = PX4Profile::Hitl;

		} else if (strcmp(transport_env, "udp") == 0) {
			transport = PX4Transport::Udp;
			prof = PX4Profile::Hitl;

		} else {
			cerr << "[flightaxis_bridge] unknown PX4_HITL_TRANSPORT '" << transport_env
			     << "' (expected tcp-server, serial or udp)" << endl;
			delete [] maps;
			return -1;
		}
	}

	{
		// 250 Hz is the serial-safe default and matches RealFlight's physics
		// frame rate; 0 (every frame) stays the SITL default over loopback.
		const double default_sensor_hz = (prof == PX4Profile::Hitl) ? 250.0 : 0.0;
		const double sensor_hz = envOrDefault("PX4_HITL_SENSOR_HZ", default_sensor_hz);

		px4.Configure(transport, prof,
			      getenv("PX4_HITL_SERIAL_DEV"),
			      (int)envOrDefault("PX4_HITL_SERIAL_BAUD", 921600),
			      getenv("PX4_HITL_UDP_HOST"),
			      (int)envOrDefault("PX4_HITL_UDP_PORT", 14550),
			      sensor_hz);

		if (prof == PX4Profile::Hitl) {
			cerr << "[flightaxis_bridge] *** HITL MODE - REAL HARDWARE - REMOVE PROPELLERS ***" << endl;
			cerr << "[flightaxis_bridge] transport=" << px4.TransportName()
			     << " HIL_SENSOR=" << (sensor_hz > 0.0 ? sensor_hz : 0.0) << " Hz"
			     << (sensor_hz > 0.0 ? "" : " (every frame)") << endl;
			cerr << "[flightaxis_bridge] HIL_STATE_QUATERNION and RAW_RPM disabled"
			     << " (not valid for a real board)" << endl;

			// A hardware UART is the classic silent failure: PX4 refuses to
			// enable the HIL streams unless its own datarate exceeds 5000 B/s
			// (mavlink_main.cpp:671) and MAV_x_RATE defaults to 1200 B/s.
			const char *dev = getenv("PX4_HITL_SERIAL_DEV");

			if (transport == PX4Transport::Serial && dev != nullptr
			    && strncmp(dev, "/dev/ttyACM", 11) != 0) {
				cerr << "[flightaxis_bridge] NOTE: " << dev << " is not a USB CDC-ACM port."
				     << " Set SER_*_BAUD=921600 and MAV_*_RATE>5000 on the board, or PX4"
				     << " will never enable the HIL streams (and will not tell you)." << endl;
			}

			// The other classic silent failure, and the one that costs a whole
			// debugging session: nothing the bridge sends can put PX4 into HIL
			// mode. MavlinkReceiver gates HIL_SENSOR / HIL_STATE_QUATERNION
			// (mavlink_receiver.cpp:347) and HIL_GPS (:367) on get_hil_enabled(),
			// which tracks vehicle_status.hil_state == HIL_STATE_ON. In this PX4
			// tree that is set ONLY by Commander::enable_hil()
			// (Commander.cpp:2688), reachable only from `commander start -h`
			// (Commander.cpp:2681) - which rcS runs only when SYS_HITL > 0
			// (rcS:451-453). There is no runtime SET_MODE path to it, so the
			// bridge cannot fix this for you; it can only tell you.
			//
			// Without it the board silently DISCARDS HIL_SENSOR and HIL_GPS and
			// only DISTANCE_SENSOR (handled unconditionally, :207) gets through.
			cerr << "[flightaxis_bridge] PRECONDITION: the board must have SYS_HITL=1 set"
			     << " AND HAVE BEEN REBOOTED. Without it PX4 never calls"
			     << " Commander::enable_hil(), and mavlink_receiver DISCARDS every"
			     << " HIL_SENSOR and HIL_GPS this bridge sends - silently, with no"
			     << " error at either end. Check with `param show SYS_HITL` and"
			     << " `commander status` (expect HIL enabled) on the board." << endl;

			if (getenv("PX4_HITL_STATE_QUAT_BYPASS") != nullptr) {
				px4.EnableStateQuaternionBypass();
				cerr << "[flightaxis_bridge] WARNING: HIL_STATE_QUATERNION bypass ENABLED."
				     << " The board will publish injected truth onto vehicle_attitude and"
				     << " vehicle_local_position, RACING EKF2. The vehicle will fly well and"
				     << " prove NOTHING about the estimator. Debug use only." << endl;
			}
		}
	}

	// mirror the FG bridge ordering: bring the PX4 side up first (for the SITL
	// TCP server this blocks in accept() until the px4 binary launched by
	// sitl_run.sh connects; serial/UDP just open the link)
	if (transport == PX4Transport::TcpServer) {
		cerr << "[flightaxis_bridge] waiting for PX4 on TCP " << (4560 + instance) << " ..." << endl;
	}

	if (px4.Init(instance) != 0) {
		cerr << "[flightaxis_bridge] Unable to Init PX4 Communication" << endl;
		delete [] maps;
		return -1;
	}

	if (transport == PX4Transport::TcpServer) {
		cerr << "[flightaxis_bridge] waiting for PX4 on TCP " << (4560 + instance) << " ... connected" << endl;

	} else {
		cerr << "[flightaxis_bridge] PX4 link up on " << px4.TransportName()
		     << " - waiting for HIL_ACTUATOR_CONTROLS from the board" << endl;
	}

	setup_unix_signals();
	stop = 0;

	// Battery / fuel telemetry -> PX4's battery_status.
	//
	// SITL only, and battery_link.h explains at length why: the simulator link
	// this bridge already owns cannot carry it (SimulatorMavlink has no
	// BATTERY_STATUS case at all), so it needs its own UDP socket to PX4's
	// API/offboard MAVLink link; and on a HITL board the value would be
	// overwritten ~125 times between updates by the hardcoded battery that
	// mavlink_receiver publishes on every HIL_SENSOR.
	BatteryLink battery;

	if (transport == PX4Transport::TcpServer) {
		if (battery.init(instance)) {
			cerr << "[flightaxis_bridge] battery telemetry -> UDP 127.0.0.1:"
			     << (14580 + instance) << " as sysid " << (instance + 1) << endl;
			cerr << "[flightaxis_bridge]   (airframe must set SIM_BAT_ENABLE 0 so battery_simulator"
			     << " does not publish over it)" << endl;

		} else {
			// Not fatal. The simulation is entirely usable without it; only
			// the battery reading is lost, and battery_simulator can be left
			// enabled to cover for it.
			cerr << "[flightaxis_bridge] WARNING: could not open the battery telemetry socket;"
			     << " PX4 will have no battery unless SIM_BAT_ENABLE is 1" << endl;
		}

	} else {
		cerr << "[flightaxis_bridge] battery telemetry disabled on " << px4.TransportName()
		     << ": a HITL board overwrites battery_status from every HIL_SENSOR"
		     << " (see battery_link.h)" << endl;
	}

	ModelHealth model_health;

	// inject the UAV controller interface into RealFlight
	cerr << "[flightaxis_bridge] connecting to RealFlight at " << fa_ip << ":18083" << endl;

	const bool reset_position = (options & OPT_RESET_POSITION) != 0;

	// Pump the HEARTBEAT (and drain the PX4 link) while we retry the injection.
	// This loop is the ONLY thing running before the main loop, and Send() -
	// the only other heartbeat source - is unreachable from here. PX4 will not
	// bring a USB MAVLink instance up until it hears from the other end, so
	// without this, starting the bridge before RealFlight leaves the board-side
	// link uninitialised for as long as RealFlight takes to appear, with no
	// diagnostic at either end. No-op in the SITL profile.
	while (stop == 0 && !fa.startController(reset_position)) {
		cerr << "[flightaxis_bridge] FlightAxis controller injection failed, retrying ..." << endl;
		fa.markNeedsRestart();

		for (int i = 0; i < 20 && stop == 0 && !px4.LinkLost(); i++) {
			px4.SendHeartbeat();
			px4.Recieve(false);
			usleep(50000);
		}

		if (px4.LinkLost()) {
			break;
		}
	}

	if (px4.LinkLost()) {
		cerr << "[flightaxis_bridge] PX4 link died while waiting for RealFlight - exiting" << endl;
		// the injection may already have gone through on the last retry
		fa.releaseController();
		px4.Clean();
		delete [] maps;
		return -1;
	}

	if (stop == 0) {
		cerr << "[flightaxis_bridge] controller injected, aircraft reset" << endl;
	}

	// Persistent, UNTRANSFORMED RealFlight channel values (0..1); start mapped
	// slots at their disarm value (or neutral) so the first frames are safe.
	// tx_channels[] is the transformed copy that actually goes out - see
	// buildChannels().
	double channels[RF_CHANNELS];
	double tx_channels[RF_CHANNELS];

	// PX4_FA_DUMP_CHANNELS=<hz>, 0/unset = off. See the dump block in the loop.
	// Seeded inverted so the first sample sets both ends rather than measuring
	// travel against a 0.0 that was never on the wire.
	const double dump_hz = envOrDefault("PX4_FA_DUMP_CHANNELS", 0.0);
	const uint64_t dump_interval_us = (dump_hz > 0.0) ? (uint64_t)(1000000.0 / dump_hz) : 0;
	uint64_t last_dump_us = 0;
	double dump_min[RF_CHANNELS];
	double dump_max[RF_CHANNELS];

	for (int i = 0; i < RF_CHANNELS; i++) {
		dump_min[i] =  1e9;
		dump_max[i] = -1e9;
	}

	seedDisarmChannels(maps, nmaps, unmapped_default, channels);

	// timing state
	double initial_time_s = 0.0;		// physics-time epoch capture
	// Explicit "epoch captured" flag. This must NOT be inferred from
	// initial_time_s <= 0.0: after a RealFlight restart the epoch is rebased to
	// (physics_time - already_exported_clock), which is legitimately NEGATIVE
	// once the bridge has been up longer than RealFlight. Using the sign as a
	// sentinel re-ran the first-frame capture on every subsequent frame, which
	// pinned each real frame to a fixed +1 ms and ran the clock 1.25x fast.
	bool have_epoch = false;
	double last_time_s = 0.0;		// last physics time seen
	double average_frame_time_s = 0.0;	// EMA 0.98/0.02
	double extrapolated_s = 0.0;		// how far we've extrapolated past the last real frame
	uint64_t time_now_us = 0;		// physics us since epoch (glitch-compensated)
	uint32_t glitch_count = 0;

	// reinject throttle (wall clock): startController() is three SOAP round-trips
	// including ResetAircraft, so it must not run once per loop iteration
	const uint64_t REINJECT_INTERVAL_US = 300000;	// 300 ms

	/*
	 * A frame is a teleport when the aircraft moved further than the velocity
	 * it reported could carry it. The margin absorbs the discretisation of a
	 * position sampled at one instant against a velocity sampled at another;
	 * the floor keeps a stationary model, whose reachable distance is zero,
	 * from tripping on numerical noise in its own coordinates.
	 */
	/*
	 * How long to keep offering PX4 a position reset after a teleport, and how
	 * often. Five seconds covers the gap between the respawn and EKF2 entering
	 * the dead-reckoning state that lets it accept one; 200 ms is frequent
	 * enough not to miss a brief window and far too slow to matter as traffic.
	 */
	/*
	 * Restart PX4 on a respawn. ON by default; set the variable to 0 to keep a
	 * session running across one.
	 *
	 * Not a preference. A respawn leaves EKF2 holding a converged solution for
	 * a flight that no longer exists, and it treats the hundred-metre step as a
	 * lying GPS rather than a moved aircraft - it rejects the position and dead
	 * reckons, which is the aircraft drifting across the map while it sits on
	 * the runway. Nothing short of a new estimator fixes that: the external
	 * position reset PX4 offers is refused at exactly the moment it would help,
	 * and SITL has no way to bypass the estimator the way ArduPilot's
	 * EKFType::SIM does. Everyone who presses spacebar needs this, so it is not
	 * something to find out about first.
	 */
	/*
	 * Whole-string compare, not restart_env[0], so this agrees with the test in
	 * sitl_run.sh. Testing the first character alone made a value like "0x"
	 * disable the bridge half while the script still looped, and the two halves
	 * of one feature disagreeing is not something a user can diagnose.
	 */
	const char *restart_env = ::getenv("PX4_FLIGHTAXIS_RESTART_ON_RESET");
	const bool restart_on_teleport = (restart_env == nullptr || ::strcmp(restart_env, "0") != 0);

	const uint64_t RESET_RETRY_WINDOW_US   = 5000000;
	const uint64_t RESET_RETRY_INTERVAL_US = 200000;
	uint64_t reset_retry_until_us = 0;
	uint64_t reset_retry_next_us  = 0;

	const double TELEPORT_MARGIN  = 4.0;

	// The deadband a reset must clear when the model is at rest (speed ~ 0, so
	// the TELEPORT_MARGIN * speed term is ~ 0). It only has to sit above
	// RealFlight's position noise, NOT above real motion - the margin term
	// already covers motion, even across a glitch frame. Kept low so a
	// multirotor reset near its spawn - which moves only a few metres, far under
	// the old 10 m - is caught too, not just a plane flown well away. Raise it
	// with PX4_FLIGHTAXIS_TELEPORT_FLOOR_M (metres) if a jittery model
	// false-triggers; lower it to catch even smaller resets.
	double TELEPORT_FLOOR_M = 2.0;
	if (const char *floor_env = ::getenv("PX4_FLIGHTAXIS_TELEPORT_FLOOR_M")) {
		const double v = ::atof(floor_env);
		if (v > 0.0) {
			TELEPORT_FLOOR_M = v;
		}
	}

	double last_position_x = 0.0, last_position_y = 0.0, last_position_z = 0.0;
	double last_position_t = 0.0;
	bool have_last_position = false;
	uint64_t last_reinject_us = 0;

	// FPS reporting
	uint64_t frame_counter = 0;
	uint64_t socket_frame_counter = 0;
	uint64_t last_socket_frame_counter = 0;
	double last_frame_count_s = 0.0;

	/*
	 * REALTIME FACTOR (physics seconds per wall second).
	 *
	 * This is the one number that decides whether a SITL run is meaningful,
	 * because in SITL PX4 THROWS OUR TIMESTAMPS AWAY.
	 * SimulatorMavlink::handle_message_hil_sensor() does call
	 * px4_clock_settime(CLOCK_MONOTONIC, imu.time_usec)
	 * (SimulatorMavlink.cpp:490-494), but on a nolockstep build that macro is
	 * plain clock_settime() (px4_platform_common/time.h:24 ->
	 * visibility.h:65), and Linux never allows CLOCK_MONOTONIC to be set - it
	 * fails with EINVAL, unchecked. What is actually passed to update_sensors()
	 * is hrt_absolute_time() (:497 and :512), and HIL_GPS (:452) and
	 * DISTANCE_SENSOR (:1489) timestamp themselves the same way.
	 *
	 * So what reaches EKF2 is the WALL-CLOCK ARRIVAL RATE of our messages, not
	 * the physics time in them. If RealFlight's physics runs at 0.8x, PX4
	 * integrates the vehicle's motion over 25% more real time than the physics
	 * actually covered: the aircraft flies in slow motion while every sensor
	 * value looks perfectly correct, and velocities, accelerations and every
	 * control loop are scaled wrong. Nothing in the sensor data can reveal it.
	 *
	 * Worse, the glitch compensation below actively HIDES this, because it
	 * smooths the bridge's own exported clock rather than the arrival rate.
	 *
	 * This is distinct from the m_currentPhysicsSpeedMultiplier warning below,
	 * which is RealFlight's SELF-REPORTED setting: this catches the case where
	 * RealFlight says 1.0 and the machine simply cannot keep up.
	 */
	double last_frame_count_wall_s = 0.0;
	double realtime_factor = 0.0;
	uint64_t last_rtf_warn_us = 0;
	// suppress one sample after a keep-alive: the physics clock was paused for
	// that window on purpose, which is not the fault this is looking for
	bool rtf_skip_sample = true;

	// Must be called from BOTH the extrapolation path and the normal-frame
	// path, otherwise the printed rate drifts
	auto report_FPS = [&](const FAState &st) {
		if (frame_counter++ % 1000 != 0) {
			return;
		}

		const uint64_t now_wall_us = micros();
		const double wall_s = now_wall_us * 1.0e-6;

		if (last_frame_count_s != 0.0) {
			const uint64_t frames = socket_frame_counter - last_socket_frame_counter;
			last_socket_frame_counter = socket_frame_counter;
			const double dtw = st.m_currentPhysicsTime_SEC - last_frame_count_s;
			const double d_wall = wall_s - last_frame_count_wall_s;

			// physics seconds per wall second over this reporting window
			if (!rtf_skip_sample && dtw > 0.0 && d_wall > 0.0) {
				realtime_factor = dtw / d_wall;

				if ((realtime_factor < 0.95 || realtime_factor > 1.05)
				    && (last_rtf_warn_us == 0 || now_wall_us - last_rtf_warn_us >= 5000000)) {
					last_rtf_warn_us = now_wall_us;
					fprintf(stderr,
						"[flightaxis_bridge] WARNING: realtime factor %.2f (physics s per wall s)."
						" PX4 timestamps the SITL sensor stream with its OWN clock on arrival"
						" (SimulatorMavlink.cpp:497,512), so it is integrating %.0f%% %s real"
						" time than the physics covers: the vehicle will fly %s with entirely"
						" correct-looking sensor values, and velocities, accelerations and every"
						" control loop are scaled wrong. Reduce RealFlight's graphics load or"
						" close other applications.\n",
						realtime_factor,
						fabs(100.0 / realtime_factor - 100.0),
						(realtime_factor < 1.0) ? "more" : "less",
						(realtime_factor < 1.0) ? "in slow motion" : "fast");
				}
			}

			rtf_skip_sample = false;

			if (!(options & OPT_SILENCE_FPS) && dtw > 0.0 && average_frame_time_s > 0.0) {
				// exchanges/loop are informational (loop free-runs against a fast
				// RealFlight); avg= is the authoritative physics frame rate, and
				// rtf= is the one to watch - see the block where it is declared
				char rtf_str[32];

				if (realtime_factor > 0.0) {
					snprintf(rtf_str, sizeof(rtf_str), "%.2f", realtime_factor);

				} else {
					snprintf(rtf_str, sizeof(rtf_str), "n/a");
				}

				fprintf(stderr,
					"[flightaxis_bridge] exchanges=%.1f/s loop=%.1f/s avg=%.1f FPS rtf=%s glitches=%u\n",
					frames / dtw, 1000.0 / dtw, 1.0 / average_frame_time_s,
					rtf_str, (unsigned)glitch_count);
			}

			if (fabs(st.m_currentPhysicsSpeedMultiplier - 1.0) > 0.01) {
				fprintf(stderr, "[flightaxis_bridge] WARNING: RealFlight physics speed multiplier is %.2f"
					" (set it to 1.0)\n", st.m_currentPhysicsSpeedMultiplier);
			}
		}

		last_frame_count_s = st.m_currentPhysicsTime_SEC;
		last_frame_count_wall_s = wall_s;
	};

	bool announced_first_controls = false;
	unsigned fail_count = 0;
	bool have_fa_data = false;

	// Set while a keep-alive is driving the clock off wall time.
	// Cleared by the rebase on the first good frame afterwards - see there.
	bool keepalive_active = false;

	/*
	 * WALL-CLOCK KEEP-ALIVE.
	 *
	 * Shared by the two situations where RealFlight's physics clock is not
	 * advancing but PX4 still needs a sensor stream:
	 *   - the controller reinject (spacebar reset / aircraft change), and
	 *   - a STALLED physics clock: RealFlight paused, a modal dialog open, the
	 *     window minimised. The bridge keeps getting valid ExchangeData replies
	 *     carrying the same m-currentPhysicsTime-SEC over and over.
	 *
	 * Both look identical from here - a valid frame with no new physics in it -
	 * so they get identical treatment: hold the last state, advance the exported
	 * clock off WALL time at ~1 kHz, and send. Pacing off the loop iteration
	 * instead would run the clock at the several-kHz loop rate.
	 *
	 * Setting keepalive_active hands the reconciliation to branch 0, which
	 * rebases the physics epoch onto the clock we have already exported once
	 * real frames resume. That is what keeps this from desynchronising the two
	 * clocks, and it is why both callers must go through here rather than
	 * rolling their own clock advance.
	 */
	uint64_t last_alive_us = 0;		// keep-alive pacing (wall clock)

	/*
	 * How long the physics clock must be stalled before the keep-alive takes
	 * over. This threshold is load-bearing in BOTH directions.
	 *
	 * Too low and the keep-alive fires during normal operation: the bridge
	 * free-runs at several kHz against a 250 Hz RealFlight, so it routinely
	 * finishes interpolating a frame and then waits out the rest of the frame
	 * period with nothing to do. Treating that as a stall means branch 0
	 * rebases the epoch on nearly every frame - which, measured, consumed most
	 * real frames and left the realtime-factor monitor permanently reading
	 * "n/a" because branch 0 also suppresses its next sample.
	 *
	 * Too high and PX4's sensor topics go stale: the arming checks want each
	 * one updated within 1 s.
	 *
	 * 200 ms sits an order of magnitude above any plausible frame period
	 * (4 ms at RealFlight's 250 Hz, 20 ms even at a badly degraded 50 Hz) and
	 * a comfortable factor below the 1 s timeout.
	 */
	const uint64_t STALL_TIMEOUT_US = 200000;	// 200 ms
	uint64_t stall_since_us = 0;		// wall time the physics clock stalled at

	auto keepAlive = [&](uint64_t now_us) {
		if (last_alive_us == 0) {
			last_alive_us = now_us;
			return;
		}

		uint64_t alive_dt_us = now_us - last_alive_us;

		if (alive_dt_us < 1000) {
			return;
		}

		// a long stall must not teleport the clock forward
		if (alive_dt_us > 100000) {
			alive_dt_us = 100000;
		}

		last_alive_us = now_us;
		time_now_us += alive_dt_us;
		vehicle.setTimeUsec(time_now_us);
		vehicle.extrapolate(alive_dt_us * 1.0e-6);
		px4.Send(0);
		// the exported clock and initial_time_s have now diverged; branch 0
		// reconciles them on the way out
		keepalive_active = true;
	};

	// The FA-side state that decides whether an ExchangeData failure warrants
	// re-running the full startup sequence (ArduPilot SIM_FlightAxis.cpp:295-298
	// keeps the equivalent in its persistent `state`).
	bool last_controller_active = true;
	bool last_reset_pressed = false;

	while (stop == 0) {

		// The PX4 side can vanish silently (SITL exits, USB unplugged). The
		// communicator latches that; there is nothing left to simulate for, so
		// stop - which also stops hammering RealFlight with SOAP requests.
		// See the dead-link policy block in px4_communicator.h.
		if (px4.LinkLost()) {
			cerr << "[flightaxis_bridge] PX4 link lost - shutting down" << endl;
			break;
		}

		buildChannels(vehicle, maps, nmaps, unmapped_default, options, channels, tx_channels);

		/*
		 * PX4_FA_DUMP_CHANNELS=<hz>: print what is actually on the wire.
		 *
		 * This exists because the one question this bridge cannot answer from
		 * its own configuration is which RealFlight channel a given surface is
		 * bound to. Everything on the PX4 side is checkable here - the JSON map,
		 * the FUNC assignments, the actuator_outputs in the ulog - and all of it
		 * can be correct while a surface still sits motionless, because the
		 * RealFlight MODEL decides what channel N drives. A user whose elevator
		 * does not move has no way to tell "the bridge is not sending it" from
		 * "the bridge is sending it to a channel this model does not use", and
		 * those two have opposite fixes.
		 *
		 * So this prints the value AND the per-channel travel since the dump
		 * started. Travel is what actually answers the question: a channel that
		 * is being driven has a span, a channel that is not is pinned. Move one
		 * surface at a time and the row with the span is the channel it is on.
		 *
		 * Off unless the variable is set, and rate-limited, because it prints
		 * every mapped channel and would otherwise bury the health messages
		 * this bridge exists to surface.
		 */
		const uint64_t dump_now_us = (dump_interval_us > 0) ? micros() : 0;

		if (dump_interval_us > 0 && dump_now_us - last_dump_us >= dump_interval_us) {
			last_dump_us = dump_now_us;

			for (int i = 0; i < RF_CHANNELS; i++) {
				if (tx_channels[i] < dump_min[i]) {
					dump_min[i] = tx_channels[i];
				}

				if (tx_channels[i] > dump_max[i]) {
					dump_max[i] = tx_channels[i];
				}
			}

			fprintf(stderr, "[flightaxis_bridge] channels (RF ch1-12), %s:\n",
				vehicle.armed() ? "ARMED" : "disarmed - surfaces sit at their disarm value");

			for (int i = 0; i < RF_CHANNELS; i++) {
				const double span = dump_max[i] - dump_min[i];
				fprintf(stderr, "    ch%-2d rf%-2d = %.3f  travel %.3f %s\n",
					i + 1, i, tx_channels[i], span,
					(span < 0.002) ? "<- not moving" : "");
			}
		}

		// send selectedChannels=0 until PX4 is up (RealFlight holds neutral)
		const uint32_t selectedChannels = vehicle.receivedFirstControls() ? 4095 : 0;

		FAState state;

		if (!fa.exchangeData(tx_channels, RF_CHANNELS, selectedChannels, state)) {
			// connect/send/parse failure: force startup re-run, back off and retry
			fa.markNeedsRestart();

			if (fail_count == 0) {
				cerr << "[flightaxis_bridge] ExchangeData failed, retrying ..." << endl;
			}

			fail_count++;
			// cap the shift: usleep() is only specified for < 1 s
			unsigned shift = (fail_count < 6) ? fail_count : 6;
			usleep(10000u << shift);	// 20 ms .. 640 ms backoff

			// Keep servicing the PX4 side while RealFlight is unreachable.
			// This branch used to touch px4 not at all, which broke the
			// dead-link policy that the rest of the bridge relies on: link
			// loss is only ever detected inside Recieve() (poll HUP/ERR, a
			// 0-length TCP read) or Send(), so with RealFlight down NOTHING
			// was watching PX4. Measured: kill the simulator, then kill px4 -
			// the bridge spun here forever instead of exiting, and the
			// LinkLost() check at the top of the loop never fired because
			// nothing ever set the latch.
			//
			// It also stopped draining 4560 while PX4 kept streaming
			// HIL_ACTUATOR_CONTROLS at 200 Hz into a socket nobody was
			// reading. The heartbeat is a no-op in the SITL profile.
			//
			// Deliberately NOT the sensor keep-alive: unlike the reinject
			// case, RealFlight is GONE here, so there is no simulation to keep
			// PX4's estimator alive for. Draining and noticing a dead peer is
			// the whole job.
			px4.SendHeartbeat();
			px4.Recieve(false);

			if (px4.LinkLost()) {
				continue;	// the check at the top of the loop shuts us down
			}

			// Re-run the startup sequence only when the LAST KNOWN FlightAxis
			// state says the controller actually needs re-injecting - i.e.
			// ArduPilot's condition (SIM_FlightAxis.cpp:295-298), minus the
			// socket test, which has no analogue here.
			//
			// `if (!fa.controllerStarted())` on its own is vacuous: the
			// markNeedsRestart() two lines above has already forced that false,
			// so it reads as a condition while being unconditional. Every failed
			// exchange therefore ran three SOAP round-trips INCLUDING
			// ResetAircraft - so a transient network hiccup mid-flight repeatedly
			// teleported the aircraft back to the runway instead of recovering
			// in place, and recovery took far longer than the backoff suggests.
			//
			// Safety valve on top of ArduPilot's condition: if RealFlight comes
			// back WITHOUT our controller injected, ExchangeData itself faults,
			// so we never get a state to read m_flightAxisControllerIsActive
			// from and the two terms above stay stale-good forever. Retry the
			// injection occasionally in that case - but far less often than
			// once per failure, which is the whole point of this change.
			const bool needs_restart = !last_controller_active || last_reset_pressed;
			const bool stuck = (fail_count >= 20 && (fail_count % 32) == 0);

			if (needs_restart || stuck) {
				fa.startController(reset_position);
			}

			continue;
		}

		// Edge: our injected controller was active last frame and is gone this
		// one. Reaching here means ExchangeData SUCCEEDED (we are on the success
		// path) yet RealFlight reports the controller dropped - which is a
		// spacebar respawn or an aircraft change, not a socket blip (a blip
		// faults ExchangeData and takes the failure path above instead). Unlike
		// the position-teleport test below it does not depend on how far the
		// model moved, so a multirotor reset while hovering over its own spawn -
		// a jump far under TELEPORT_FLOOR_M - is caught just the same.
		const bool controller_dropped_edge =
			last_controller_active && !state.m_flightAxisControllerIsActive;

		// Edge: the RealFlight reset flag went clear -> set. A spacebar respawn
		// leaves this flag at 0 on most setups, but the transmitter's own reset
		// button DOES raise it - and it does so whatever the aircraft's position,
		// height or attitude, a genuinely unconditional trigger. Treat it exactly
		// like the controller drop: no distance/attitude minimum of any kind.
		const bool reset_flag_edge =
			!last_reset_pressed && state.m_resetButtonHasBeenPressed;

		last_controller_active = state.m_flightAxisControllerIsActive;
		last_reset_pressed = state.m_resetButtonHasBeenPressed;

		// Both are edge/time gated internally, so calling them on every
		// exchange is cheap: reportModelHealth() is three compares, and
		// battery.update() returns on a single subtraction 399 times out of
		// 400 at the measured exchange rate.
		reportModelHealth(model_health, state);
		battery.update(state, micros());

		if (fail_count > 0) {
			cerr << "[flightaxis_bridge] ExchangeData recovered after " << fail_count << " retries" << endl;
			fail_count = 0;
		}

		socket_frame_counter++;

		if (!announced_first_controls && vehicle.receivedFirstControls()) {
			cerr << "[flightaxis_bridge] first HIL_ACTUATOR_CONTROLS received, enabling channels" << endl;
			announced_first_controls = true;
		}

		/*
		 * TELEPORT DETECTION.
		 *
		 * m_resetButtonHasBeenPressed does NOT fire when the pilot presses
		 * spacebar. Measured, on two respawns in one session: the reported
		 * position moved 92.3 m and 120.9 m in a single frame while the flag
		 * stayed 0 both times. Whatever that flag reports - an aircraft change,
		 * a transmitter button - it is not the reset the pilot actually uses,
		 * so a bridge that waits for it never learns the aircraft moved and
		 * PX4 keeps flying the old trajectory.
		 *
		 * The discontinuity itself is the signal, and it is unambiguous.
		 * Frames arrive about 4 ms apart, so 92 m in one of them is 23 km/s.
		 * Comparing against the distance the reported velocity could actually
		 * cover - rather than a fixed threshold - keeps this correct when the
		 * glitch compensator has just swallowed a long network stall and a
		 * legitimate frame really does span two seconds of flight.
		 */
		if (have_last_position) {
			const double dx = state.m_aircraftPositionX_MTR - last_position_x;
			const double dy = state.m_aircraftPositionY_MTR - last_position_y;
			const double dz = state.m_altitudeASL_MTR - last_position_z;
			const double moved = std::sqrt(dx * dx + dy * dy + dz * dz);

			const double speed = std::sqrt(
				state.m_velocityWorldU_MPS * state.m_velocityWorldU_MPS +
				state.m_velocityWorldV_MPS * state.m_velocityWorldV_MPS +
				state.m_velocityWorldW_MPS * state.m_velocityWorldW_MPS);

			// Physics interval for THIS frame. Computed here rather than reused
			// from the clock handling further down, which has not run yet.
			const double frame_dt = state.m_currentPhysicsTime_SEC - last_position_t;
			const double reachable = TELEPORT_MARGIN * speed * frame_dt + TELEPORT_FLOOR_M;

			if (moved > reachable) {
				cerr << "[flightaxis_bridge] aircraft teleported " << moved
				     << " m (RealFlight reset) - re-anchoring" << endl;
				vehicle.invalidatePositionOffset();

				/*
				 * Re-anchoring makes the bridge report the model at home again,
				 * which is what a freshly started session would report. It does
				 * not make PX4 believe it. EKF2 has converged state from the
				 * flight just ended and reads a 100 m step as a lying GPS rather
				 * than a moved aircraft, so it rejects the position and dead-
				 * reckons - which is the pilot's "QGC keeps flying".
				 *
				 * There is no HIL message for "the vehicle has been
				 * repositioned", but PX4 does accept an external position fix
				 * with a stated accuracy, and treats a sub-metre one as grounds
				 * for a hard reset rather than a fusion update. Ask for it. The
				 * estimator gates the request on dead-reckoning or being on the
				 * ground without GNSS fusion, so it may be refused on the first
				 * attempt and accepted moments later once the divergence starts;
				 * that is still a recovery in seconds rather than minutes.
				 */
				if (restart_on_teleport) {
					/*
					 * PX4_FLIGHTAXIS_RESTART_ON_RESET: leave, and let
					 * sitl_run.sh bring PX4 and this bridge back up.
					 *
					 * An estimator with no state accepts whatever it is given,
					 * which is why a freshly started session is always correct
					 * and a respawn is not. Everything short of a restart has to
					 * persuade EKF2 to abandon a converged solution, and it is
					 * built not to: an external position reset comes back
					 * TEMPORARILY_REJECTED at the moment of the teleport,
					 * because the vehicle is airborne and still fusing GNSS.
					 *
					 * The cost is a gap of a few seconds, a new log file, and a
					 * ground station reconnect. The exit code is what tells the
					 * runner this was deliberate rather than a crash.
					 */
					cerr << "[flightaxis_bridge]   restarting PX4 "
					     << "(PX4_FLIGHTAXIS_RESTART_ON_RESET=0 disables this)"
					     << endl;

					// Force-disarm, then shut PX4 down; sitl_run.sh brings it
					// and this bridge back up together. If PX4 is still alive a
					// few seconds from now the request was refused, and because
					// the runner is blocked on it in the foreground nothing here
					// can recover that - say so rather than leave a silent hang.
					if (battery.active() && !battery.requestReboot()) {
						cerr << "[flightaxis_bridge]   WARNING: could not send the"
						     << " shutdown request. If PX4 keeps running with"
						     << " \"Broken pipe\" warnings, Ctrl-C the session."
						     << endl;
					}

					fa.releaseController();
					return EXIT_RESTART_REQUESTED;
				}

				reset_retry_until_us = micros() + RESET_RETRY_WINDOW_US;
				reset_retry_next_us = 0;
			}
		}

		/*
		 * The reset request is retried, not fired once.
		 *
		 * Measured: sent at the instant of the teleport it comes back
		 * TEMPORARILY_REJECTED, because EKF2 only accepts an external position
		 * while the vehicle is dead-reckoning, or on the ground and not fusing
		 * GNSS (EKF2.cpp:534). At the moment of a respawn none of that holds -
		 * GNSS fusion was active 100% of the time in the run that produced this
		 * - and the condition only arrives a second or two later, once the
		 * rejected position has pushed the estimator into dead reckoning. So
		 * keep offering it for a few seconds and let EKF2 take it when it can.
		 *
		 * Being accepted more than once is harmless: every request carries the
		 * same home coordinates, which is where the model is standing.
		 */
		if (reset_retry_until_us != 0) {
			const uint64_t now_us = micros();

			if (now_us >= reset_retry_until_us) {
				reset_retry_until_us = 0;

			} else if (now_us >= reset_retry_next_us && battery.active()) {
				battery.requestPositionReset(home_lat, home_lon, 0.5, now_us);
				reset_retry_next_us = now_us + RESET_RETRY_INTERVAL_US;
			}
		}

		last_position_t = state.m_currentPhysicsTime_SEC;
		last_position_x = state.m_aircraftPositionX_MTR;
		last_position_y = state.m_aircraftPositionY_MTR;
		last_position_z = state.m_altitudeASL_MTR;
		have_last_position = true;

		// spacebar reset or aircraft change inside RealFlight: re-inject and re-anchor
		if (!state.m_flightAxisControllerIsActive || state.m_resetButtonHasBeenPressed) {
			// A spacebar respawn drops the controller (the edge computed above)
			// but need not move the aircraft far enough for the position-teleport
			// test to fire - a multirotor reset while hovering over its spawn
			// barely moves. So treat the controller-drop edge as a reset in its
			// own right, on equal footing with a teleport: re-anchor, and when
			// restart-on-reset is enabled (the default) restart PX4 exactly as
			// the teleport path does, so EVERY spacebar gets the clean-slate
			// estimator a respawn needs - near spawn or far. This runs once, on
			// the edge, so holding the flag low for many frames does not repeat
			// it. PX4_FLIGHTAXIS_RESTART_ON_RESET=0 keeps re-anchor only, same as
			// for the teleport path.
			if (controller_dropped_edge || reset_flag_edge) {
				vehicle.invalidatePositionOffset();
				if (restart_on_teleport) {
					cerr << "[flightaxis_bridge] "
					     << (reset_flag_edge ? "reset button pressed" : "controller dropped")
					     << " (RealFlight reset) - restarting PX4"
					     << " (PX4_FLIGHTAXIS_RESTART_ON_RESET=0 disables this)" << endl;
					if (battery.active() && !battery.requestReboot()) {
						cerr << "[flightaxis_bridge]   WARNING: could not send the"
						     << " shutdown request. If PX4 keeps running with"
						     << " \"Broken pipe\" warnings, Ctrl-C the session." << endl;
					}
					fa.releaseController();
					return EXIT_RESTART_REQUESTED;
				}
			}

			// The main loop free-runs at several kHz, so this branch is entered
			// thousands of times per second while the flag is low. startController()
			// is three SOAP round-trips INCLUDING ResetAircraft, so reinjecting on
			// every iteration is a request storm against RealFlight that would also
			// re-reset the aircraft thousands of times. Attempt it at most every
			// REINJECT_INTERVAL_MS, regardless of whether it succeeds or fails.
			const uint64_t now_us = micros();

			// Drop the position anchor on the RESET FLAG ITSELF, not on the
			// reinject succeeding. `state` was parsed BEFORE ResetAircraft ran,
			// so it describes the pre-reset location and the anchor is stale
			// either way. Doing this only inside `if (fa.startController(...))`
			// meant that when injection failed - RealFlight busy reloading, a
			// dropped socket, i.e. exactly when a reset is most likely - the
			// offset was never dropped, the flag cleared on the next frame, and
			// the vehicle flew the rest of the session anchored to its pre-reset
			// position. GPS and baro both derive from that anchor, so they agree
			// with each other and the estimator cannot notice. ArduPilot does it
			// unconditionally (SIM_FlightAxis.cpp:512-514).
			//
			// Also deliberately outside the reinject throttle below: the SOAP
			// storm is what needs rate limiting, dropping a flag does not.
			// invalidatePositionOffset() is idempotent - the next real frame
			// re-anchors via VehicleState's offset_captured flag.
			if (state.m_resetButtonHasBeenPressed) {
				vehicle.invalidatePositionOffset();

				// Drop the PREVIOUS FLIGHT's channel state for the same
				// reason the position anchor is dropped: after a respawn it
				// describes an aircraft that no longer exists. Without this,
				// hold-last slots keep whatever the crash left them at -
				// measured, a fully deflected elevator - on an aircraft the
				// pilot is watching sit still on the runway.
				//
				// Alongside invalidatePositionOffset() rather than inside the
				// reinject throttle below, and for the same reason: the SOAP
				// storm is what needs rate limiting, re-seeding an array does
				// not. It is idempotent, so running it on every iteration
				// while the flag is high is free and keeps the state clean for
				// however long RealFlight holds the flag.
				//
				// Not a disarm, and not a substitute for one. buildChannels()
				// overwrites every mapped slot from live PX4 output on the
				// first frame after the flag clears, so if PX4 is still armed
				// and still commanding throttle, throttle comes straight back.
				// Disarming remains the pilot's action - see the report in
				// RUNNING.md on why PX4 may refuse it for several seconds.
				seedDisarmChannels(maps, nmaps, unmapped_default, channels);
			}

			if (last_reinject_us == 0 || now_us - last_reinject_us >= REINJECT_INTERVAL_US) {
				last_reinject_us = now_us;

				cerr << "[flightaxis_bridge] RealFlight controller "
				     << (state.m_resetButtonHasBeenPressed ? "reset" : "inactive")
				     << " - reinjecting controller" << endl;
				fa.markNeedsRestart();

				if (fa.startController(reset_position)) {
					cerr << "[flightaxis_bridge] controller injected, aircraft reset" << endl;
				}
			}

			// Keep the sensor stream alive across the reinject. There is no
			// single "sensor timeout failsafe" to name here: what actually
			// breaks is (a) the arming checks, which require each sensor topic
			// to have been updated within 1 s (accelerometerCheck.cpp:56 and
			// its gyro/mag/baro siblings), and (b) EKF2's own per-source
			// timeouts. (COM_RC_LOSS_T is 0.5 s but is RC loss and unrelated.)
			// This has to self-heal. NOT gated on the throttle above
			// - only the SOAP reinject is. Hold the last state, advance the clock,
			// and keep draining actuator controls so 4560 cannot back up.
			//
			// RealFlight's physics clock is unusable here (the controller is not
			// injected, so we are not getting fresh frames), so the keep-alive runs
			// off wall time - see keepAlive() above.
			keepAlive(now_us);

			px4.Recieve(false);
			continue;
		}

		last_reinject_us = 0;

		// ---- branch 0: leaving the reinject keep-alive.
		//
		// The keep-alive above advanced time_now_us off the WALL clock while
		// initial_time_s - the physics epoch - was left untouched. The two are
		// now out of step by (wall elapsed - physics elapsed) over the stall,
		// and physics elapsed is ~0 because RealFlight was paused. Falling
		// straight through to branch 3 would compute new_time_us from that
		// stale epoch:
		//   - diverged backwards, setTimeUsec()'s monotonic clamp FREEZES the
		//     exported clock until physics catches up;
		//   - diverged forwards by more than the 2 s glitch ceiling, the whole
		//     jump is applied in one step.
		//
		// The backwards case is the bad one, and it is not self-correcting: the
		// divergence is a CONSTANT offset (physics resumes where it paused), so
		// physics never does catch up and the exported clock stays frozen for
		// the rest of the session - unless the gap happens to exceed 500 ms, in
		// which case the glitch branch below rebases, but at the cost of a
		// spurious invalidatePositionOffset() that drops the position anchor.
		//
		// A frozen clock is severe even in SITL, where PX4 discards our
		// timestamps (see the realtime-factor block above): time_now_us is what
		// every send-rate gate in px4_communicator.cpp is measured against, so
		// once it stops, HIL_GPS, DISTANCE_SENSOR and the baro/mag sub-rates
		// stop being sent AT ALL and only the ungated HIL_SENSOR survives. In
		// HITL it is worse still, because there the board does timestamp its
		// sensor topics from what we send. (ArduPilot has no wall-clock
		// keep-alive, so it never hits any of this.)
		//
		// Fix it the way the restart branch already does: rebase the epoch onto
		// the clock we have ALREADY exported, so the next frame resumes at
		// exactly time_now_us, and skip glitch compensation for this frame.
		//
		// The physics-time test is what makes "leaving" mean leaving. Branch 0
		// runs before branch 2, so a keep-alive driven FROM branch 2 (a stalled
		// physics clock) would otherwise be unwound by this rebase on the very
		// next loop iteration - and since the rebase also clears stall_since_us,
		// the stall would have to be re-detected from scratch each time. That
		// throttles the keep-alive from ~1 kHz to one sample per
		// STALL_TIMEOUT_US: measured against `freeze 12`, HIL_SENSOR arrived
		// every ~0.2 s, leaving sensor_baro up to 0.80 s stale against a 1 s
		// arming-check timeout, and PX4 still logged MAG/BARO TIMEOUT.
		//
		// A keep-alive is only over once RealFlight produces a physics frame we
		// have not already consumed. Until then there is nothing to rebase onto.
		if (keepalive_active && state.m_currentPhysicsTime_SEC > last_time_s) {
			keepalive_active = false;
			// the paused window is not a realtime-factor problem; don't warn
			rtf_skip_sample = true;
			initial_time_s = state.m_currentPhysicsTime_SEC - time_now_us * 1.0e-6;
			have_epoch = true;
			last_time_s = state.m_currentPhysicsTime_SEC;
			// this frame is consumed by the rebase (as in branch 1); the
			// keep-alive already sent a sample ~1 ms ago, so nothing is owed
			extrapolated_s = 0.0;
			// the keep-alive is over; the next one must start pacing afresh
			// rather than measure against a timestamp from this one
			last_alive_us = 0;
			stall_since_us = 0;
			px4.Recieve(false);
			continue;
		}

		// EMA of the physics frame time
		double dt = state.m_currentPhysicsTime_SEC - last_time_s;

		if (have_fa_data && dt > 0.0 && dt < 0.1) {
			if (average_frame_time_s < 1.0e-6) {
				average_frame_time_s = dt;
			}

			average_frame_time_s = average_frame_time_s * 0.98 + dt * 0.02;
		}

		// ---- branch 1: RealFlight restarted while connected (time went backwards)
		if (have_fa_data && dt < 0.0) {
			cerr << "[flightaxis_bridge] physics time went backwards - RealFlight restart, re-basing" << endl;
			// Rebase the epoch onto the clock we have ALREADY exported, so the next
			// frame resumes at exactly time_now_us and PX4 never sees the timestamp
			// go backwards. (Anchoring at time_now_us*1e-6 instead - i.e. treating
			// the epoch as if physics time restarted from our own clock - makes
			// (phys - initial_time_s) negative when RealFlight restarts near zero,
			// which underflows the uint64 cast below into a huge forward jump.)
			initial_time_s = state.m_currentPhysicsTime_SEC - time_now_us * 1.0e-6;
			have_epoch = true;
			last_time_s = state.m_currentPhysicsTime_SEC;
			// extrapolated_s counts how far past the LAST REAL FRAME we have
			// run; that frame no longer exists after a rebase. Clearing it only
			// in branch 3 left a stale value here, which then shortened (or
			// zeroed) the first extrapolation steps after the restart.
			extrapolated_s = 0.0;
			last_alive_us = 0;
			stall_since_us = 0;
			// genuine restart: drop the anchor, the next real frame recaptures it
			vehicle.invalidatePositionOffset();
			px4.Recieve(false);
			continue;
		}

		// ---- branch 2: same physics frame (bridge outran RealFlight) - extrapolate
		if (have_fa_data && dt < 1.0e-5) {
			double delta_time = 0.001;

			// Don't go past the next expected frame - but only once we have an
			// estimate of when that is. The EMA skips the very first frame, so
			// average_frame_time_s is still 0 here on the first duplicate
			// frames, and the cap then drove delta_time to -extrapolated_s
			// (i.e. <= 0): the loop span without ever sending, for as long as
			// RealFlight repeated a frame. With no estimate, just step 1 ms.
			if (average_frame_time_s > 0.0 && delta_time + extrapolated_s > average_frame_time_s) {
				delta_time = average_frame_time_s - extrapolated_s;
			}

			if (delta_time > 0.0) {
				time_now_us += (uint64_t)(delta_time * 1.0e6);
				// push the clock in FIRST: time_now_us is the single source of
				// truth for every timestamp PX4 sees
				vehicle.setTimeUsec(time_now_us);
				vehicle.extrapolate(delta_time);
				px4.Send(0);
				extrapolated_s += delta_time;

			} else {
				/*
				 * Interpolation is exhausted: we have already run a full
				 * average_frame_time_s past the last real frame and RealFlight
				 * still has not produced a new one. Its physics clock has
				 * STALLED - paused sim, a modal dialog, a minimised window.
				 *
				 * Extrapolating further is not an option: it would push the
				 * exported clock past physics time, and when physics resumes
				 * the epoch arithmetic in branch 3 sees the clock go backwards
				 * - either freezing it (< 500 ms) or, worse, tripping the
				 * "genuine reset" path and dropping the position anchor.
				 *
				 * But sending NOTHING is what the code used to do here, and
				 * that is worse. With a 6 s physics freeze, HIL_SENSOR
				 * stopped ~52 ms after the stall began and PX4 went
				 * on to log "MAG #0 failed: TIMEOUT", "BARO #0 failed:
				 * TIMEOUT" and "angular velocity no longer valid (timeout)",
				 * failing every sensor arming check - exactly the failure the
				 * reinject keep-alive exists to prevent, reached by a
				 * different route. (It also meant the whole stall was later
				 * applied to the clock as one jump: a 6 s freeze produced a
				 * 6 s forward step, silently, since it exceeds the 2 s glitch
				 * ceiling.)
				 *
				 * So hand it to the same wall-clock keep-alive the reinject
				 * path uses. Branch 0 then rebases the epoch when real frames
				 * resume, which is what removes the jump as well.
				 *
				 * Only after STALL_TIMEOUT_US, though: arriving here is NORMAL
				 * once per frame (the bridge outruns RealFlight and waits out
				 * the rest of the frame period), and treating that as a stall
				 * breaks the realtime monitor - see the threshold's comment.
				 */
				const uint64_t now_wall_us = micros();

				if (stall_since_us == 0) {
					stall_since_us = now_wall_us;

				} else if (now_wall_us - stall_since_us >= STALL_TIMEOUT_US) {
					keepAlive(now_wall_us);
				}
			}

			px4.Recieve(false);
			report_FPS(state);
			continue;
		}

		// ---- branch 3: normal frame
		extrapolated_s = 0.0;
		// a real frame arrived, so no keep-alive is in progress; drop the
		// pacing timestamp so a later stall does not measure its first step
		// against a stale one (which would advance the clock by that gap).
		last_alive_us = 0;
		stall_since_us = 0;

		// Time-epoch rebase only. This
		// condition stays true for every frame while RealFlight's physics clock
		// is still near zero, so it must NOT re-capture the position offset -
		// that is driven by VehicleState's offset_captured flag instead.
		if (!have_epoch || !have_fa_data) {
			dt = 0.001;
			have_epoch = true;
			// Rebase so this frame lands exactly dt PAST the clock we have already
			// handed to PX4 (time_now_us may be non-zero: the reinject keep-alive
			// runs before the first real frame). Anchoring at dt instead would make
			// the clock stall until physics caught back up.
			initial_time_s = state.m_currentPhysicsTime_SEC - dt - time_now_us * 1.0e-6;
		}

		// glitch compensation: swallow 50 ms - 2 s network hiccups by
		// advancing the epoch; backwards > 500 ms is a true reset
		const uint64_t new_time_us = (uint64_t)((state.m_currentPhysicsTime_SEC - initial_time_s) * 1.0e6);

		if (new_time_us < time_now_us) {
			const uint64_t back_us = time_now_us - new_time_us;

			if (back_us > 500000) {
				// Physics time jumped backwards far enough to be a genuine reset.
				// ArduPilot pulls its clock back to match; we must NOT, because a
				// backwards HIL timestamp faults EKF2. Rebase the epoch instead so
				// the clock we export continues forward from where it is - same
				// treatment the "RealFlight restart" branch above applies.
				initial_time_s = state.m_currentPhysicsTime_SEC - time_now_us * 1.0e-6;
				have_epoch = true;
				vehicle.invalidatePositionOffset();
			}

		} else {
			uint64_t dt_us = new_time_us - time_now_us;
			const uint64_t glitch_threshold_us = 50000;
			const uint64_t glitch_max_us = 2000000;

			if (dt_us > glitch_threshold_us && dt_us < glitch_max_us) {
				const double adjustment_s = (dt_us - glitch_threshold_us) * 1.0e-6;
				initial_time_s += adjustment_s;
				fprintf(stderr, "[flightaxis_bridge] glitch %.2fs\n", adjustment_s);
				dt_us = glitch_threshold_us;
				glitch_count++;
			}

			time_now_us += dt_us;
		}

		last_time_s = state.m_currentPhysicsTime_SEC;
		have_fa_data = true;

		// The clock comes from time_now_us (already glitch-capped above); dt is
		// passed only as the TRUE elapsed physics time for the on-ground accel
		// finite difference, so a swallowed glitch does not overstate it.
		vehicle.setTimeUsec(time_now_us);
		vehicle.setFAData(state, dt);
		px4.Send(0);

		// drain HIL_ACTUATOR_CONTROLS (non-blocking) for the next exchange
		px4.Recieve(false);

		// FPS report every 1000 frames
		report_FPS(state);
	}

	cerr << "[flightaxis_bridge] exiting" << endl;

	// Hand RealFlight back to the physical transmitter. Every way out of the
	// loop above ends here: SIGINT/SIGTERM (the handlers only set `stop`, the
	// SOAP work happens here on the main path, never in the handler), the
	// PX4 dead-link break, and a normal fall-through. Time-bounded and
	// idempotent - see FACommunicator::releaseController().
	fa.releaseController();

	battery.close();
	px4.Clean();
	delete [] maps;

	return 0;
}
