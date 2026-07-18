/****************************************************************************
 *
 * This file is part of the PX4-FlightAxis-Bridge project.
 * Copyright (c) 2026 the PX4-FlightAxis-Bridge contributors.
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
 * Structure follows the FlightGear bridge (flightgear_bridge.cpp); the
 * three-branch physics-time handling (restart / duplicate-frame
 * extrapolation / glitch compensation) is a literal port of ArduPilot's
 * libraries/SITL/SIM_FlightAxis.cpp update().
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

#include <time.h>
#include <signal.h>
#include <unistd.h>

#include "px4_communicator.h"
#include "fa_communicator.h"
#include "vehicle_state.h"

using namespace std;

// options bitmask (port of ArduPilot SIM_FLTAX_OPTS)
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
 * Build the 12 RealFlight channel values (0..1) from the latest
 * HIL_ACTUATOR_CONTROLS through the JSON channel map, then apply the
 * Rev4Servos / HeliDemix options (ArduPilot exchange_data() order).
 * channels[] is persistent so disarm=-1 can hold the last output.
 */
static void buildChannels(const VehicleState &veh, const ChannelMap *maps, int nmaps,
			  double unmapped_default, uint32_t options, double channels[RF_CHANNELS])
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

		double out;

		if (m.scale == 1) {
			out = (v + 1.0) / 2.0;	// bipolar surface, -1..1 -> 0..1

		} else {
			out = v;		// unipolar motor, already 0..1
		}

		if (m.reverse) {
			out = 1.0 - out;
		}

		channels[m.rf] = constrain(out, 0.0, 1.0);
	}

	if (options & OPT_REV4_SERVOS) {
		// swap first 4 and last 4 servos, for quadplane RF models
		double saved[4];
		memcpy(saved, &channels[0], sizeof(saved));
		memcpy(&channels[0], &channels[4], sizeof(saved));
		memcpy(&channels[4], saved, sizeof(saved));
	}

	if (options & OPT_HELI_DEMIX) {
		// FlightAxis expects "roll/pitch/collective" input; PX4 outputs swash servos
		double swash1 = channels[0];
		double swash2 = channels[1];
		double swash3 = channels[2];

		double roll_rate  = swash1 - swash2;
		double pitch_rate = (swash1 + swash2) / 2.0 - swash3;
		double col        = (swash1 + swash2 + swash3) / 3.0;

		channels[0] = constrain(roll_rate + 0.5, 0.0, 1.0);
		channels[1] = constrain(pitch_rate + 0.5, 0.0, 1.0);
		channels[2] = constrain(col, 0.0, 1.0);
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

	FACommunicator fa(fa_ip);
	VehicleState vehicle(home_lat, home_lon, home_alt);
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

	// inject the UAV controller interface into RealFlight
	cerr << "[flightaxis_bridge] connecting to RealFlight at " << fa_ip << ":18083" << endl;

	const bool reset_position = (options & OPT_RESET_POSITION) != 0;

	while (stop == 0 && !fa.startController(reset_position)) {
		cerr << "[flightaxis_bridge] FlightAxis controller injection failed, retrying ..." << endl;
		fa.markNeedsRestart();
		usleep(1000000);
	}

	if (stop == 0) {
		cerr << "[flightaxis_bridge] controller injected, aircraft reset" << endl;
	}

	// persistent RealFlight channel values (0..1); start mapped slots at their
	// disarm value (or neutral) so the first frames are safe
	double channels[RF_CHANNELS];

	for (int i = 0; i < RF_CHANNELS; i++) {
		channels[i] = constrain(unmapped_default, 0.0, 1.0);
	}

	for (int i = 0; i < nmaps; i++) {
		if (maps[i].rf >= 0 && maps[i].rf < RF_CHANNELS) {
			channels[maps[i].rf] = (maps[i].disarm >= 0.0) ? constrain(maps[i].disarm, 0.0, 1.0) : 0.5;
		}
	}

	// timing state (port of ArduPilot SIM_FlightAxis)
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
	uint64_t last_reinject_us = 0;
	uint64_t last_alive_us = 0;		// keep-alive pacing while reinjecting

	// FPS reporting (ArduPilot report_FPS)
	uint64_t frame_counter = 0;
	uint64_t socket_frame_counter = 0;
	uint64_t last_socket_frame_counter = 0;
	double last_frame_count_s = 0.0;

	// ArduPilot report_FPS(): called from BOTH the extrapolation path and the
	// normal-frame path, otherwise the printed rate drifts
	auto report_FPS = [&](const FAState &st) {
		if (frame_counter++ % 1000 != 0) {
			return;
		}

		if (last_frame_count_s != 0.0) {
			const uint64_t frames = socket_frame_counter - last_socket_frame_counter;
			last_socket_frame_counter = socket_frame_counter;
			const double dtw = st.m_currentPhysicsTime_SEC - last_frame_count_s;

			if (!(options & OPT_SILENCE_FPS) && dtw > 0.0 && average_frame_time_s > 0.0) {
				// exchanges/loop are informational (loop free-runs against a fast
				// RealFlight); avg= is the authoritative physics frame rate
				fprintf(stderr, "[flightaxis_bridge] exchanges=%.1f/s loop=%.1f/s avg=%.1f FPS glitches=%u\n",
					frames / dtw, 1000.0 / dtw, 1.0 / average_frame_time_s,
					(unsigned)glitch_count);
			}

			if (fabs(st.m_currentPhysicsSpeedMultiplier - 1.0) > 0.01) {
				fprintf(stderr, "[flightaxis_bridge] WARNING: RealFlight physics speed multiplier is %.2f"
					" (set it to 1.0)\n", st.m_currentPhysicsSpeedMultiplier);
			}
		}

		last_frame_count_s = st.m_currentPhysicsTime_SEC;
	};

	bool announced_first_controls = false;
	unsigned fail_count = 0;
	bool have_fa_data = false;

	while (stop == 0) {

		buildChannels(vehicle, maps, nmaps, unmapped_default, options, channels);

		// send selectedChannels=0 until PX4 is up (RealFlight holds neutral)
		const uint32_t selectedChannels = vehicle.receivedFirstControls() ? 4095 : 0;

		FAState state;

		if (!fa.exchangeData(channels, RF_CHANNELS, selectedChannels, state)) {
			// connect/send/parse failure: force startup re-run, back off and retry
			fa.markNeedsRestart();

			if (fail_count == 0) {
				cerr << "[flightaxis_bridge] ExchangeData failed, retrying ..." << endl;
			}

			fail_count++;
			// cap the shift: usleep() is only specified for < 1 s
			unsigned shift = (fail_count < 6) ? fail_count : 6;
			usleep(10000u << shift);	// 20 ms .. 640 ms backoff

			if (!fa.controllerStarted()) {
				fa.startController(reset_position);
			}

			continue;
		}

		if (fail_count > 0) {
			cerr << "[flightaxis_bridge] ExchangeData recovered after " << fail_count << " retries" << endl;
			fail_count = 0;
		}

		socket_frame_counter++;

		if (!announced_first_controls && vehicle.receivedFirstControls()) {
			cerr << "[flightaxis_bridge] first HIL_ACTUATOR_CONTROLS received, enabling channels" << endl;
			announced_first_controls = true;
		}

		// spacebar reset or aircraft change inside RealFlight: re-inject and re-anchor
		if (!state.m_flightAxisControllerIsActive || state.m_resetButtonHasBeenPressed) {
			// The main loop free-runs at several kHz, so this branch is entered
			// thousands of times per second while the flag is low. startController()
			// is three SOAP round-trips INCLUDING ResetAircraft, so reinjecting on
			// every iteration is a request storm against RealFlight that would also
			// re-reset the aircraft thousands of times. Attempt it at most every
			// REINJECT_INTERVAL_MS, regardless of whether it succeeds or fails.
			const uint64_t now_us = micros();

			if (last_reinject_us == 0 || now_us - last_reinject_us >= REINJECT_INTERVAL_US) {
				last_reinject_us = now_us;

				cerr << "[flightaxis_bridge] RealFlight controller "
				     << (state.m_resetButtonHasBeenPressed ? "reset" : "inactive")
				     << " - reinjecting controller" << endl;
				fa.markNeedsRestart();

				if (fa.startController(reset_position)) {
					cerr << "[flightaxis_bridge] controller injected, aircraft reset" << endl;
					// `state` was parsed BEFORE ResetAircraft ran, so it describes the
					// pre-reset location. Drop the offset (ArduPilot zeroes it) and let
					// the next real frame re-anchor from the post-reset position.
					vehicle.invalidatePositionOffset();
				}
			}

			// Keep the sensor stream alive across the reinject: PX4 trips its
			// sensor-timeout failsafe after ~0.5 s of silence, and spec 11.6 wants
			// this to self-heal. This keep-alive is NOT gated on the throttle above
			// - only the SOAP reinject is. Hold the last state, advance the clock,
			// and keep draining actuator controls so 4560 cannot back up.
			//
			// RealFlight's physics clock is unusable here (the controller is not
			// injected, so we are not getting fresh frames), so the keep-alive runs
			// off wall time at ~1 kHz. Pacing it off the loop iteration instead
			// would run the clock at the several-kHz loop rate.
			if (last_alive_us == 0) {
				last_alive_us = now_us;
			}

			uint64_t alive_dt_us = now_us - last_alive_us;

			if (alive_dt_us >= 1000) {
				// a long stall must not teleport the clock forward
				if (alive_dt_us > 100000) {
					alive_dt_us = 100000;
				}

				last_alive_us = now_us;
				time_now_us += alive_dt_us;
				vehicle.setTimeUsec(time_now_us);
				vehicle.extrapolate(alive_dt_us * 1.0e-6);
				px4.Send(0);
			}

			px4.Recieve(false);
			continue;
		}

		last_reinject_us = 0;
		last_alive_us = 0;

		// EMA of the physics frame time (ArduPilot exchange_data())
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
			// genuine restart: drop the anchor, the next real frame recaptures it
			vehicle.invalidatePositionOffset();
			px4.Recieve(false);
			continue;
		}

		// ---- branch 2: same physics frame (bridge outran RealFlight) - extrapolate
		if (have_fa_data && dt < 1.0e-5) {
			double delta_time = 0.001;

			// don't go past the next expected frame
			if (delta_time + extrapolated_s > average_frame_time_s) {
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
			}

			px4.Recieve(false);
			report_FPS(state);
			continue;
		}

		// ---- branch 3: normal frame
		extrapolated_s = 0.0;

		// Time-epoch rebase only (ArduPilot SIM_FlightAxis.cpp update()). This
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

		// FPS report every 1000 frames (ArduPilot report_FPS)
		report_FPS(state);
	}

	cerr << "[flightaxis_bridge] exiting" << endl;
	px4.Clean();
	delete [] maps;

	return 0;
}
