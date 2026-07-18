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

	// mirror the FG bridge ordering: bring the PX4 side up first (blocks in
	// accept() until the px4 binary launched by sitl_run.sh connects)
	cerr << "[flightaxis_bridge] waiting for PX4 on TCP " << (4560 + instance) << " ..." << endl;

	if (px4.Init(instance) != 0) {
		cerr << "[flightaxis_bridge] Unable to Init PX4 Communication" << endl;
		delete [] maps;
		return -1;
	}

	cerr << "[flightaxis_bridge] waiting for PX4 on TCP " << (4560 + instance) << " ... connected" << endl;

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
	double last_time_s = 0.0;		// last physics time seen
	double average_frame_time_s = 0.0;	// EMA 0.98/0.02
	double extrapolated_s = 0.0;		// how far we've extrapolated past the last real frame
	uint64_t time_now_us = 0;		// physics us since epoch (glitch-compensated)
	uint32_t glitch_count = 0;

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
				fprintf(stderr, "[flightaxis_bridge] %.1f/%.1f FPS avg=%.1f glitches=%u\n",
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

			} else {
				usleep(200000);
			}

			// Keep the sensor stream alive across the reinject: PX4 trips its
			// sensor-timeout failsafe after ~0.5 s of silence, and spec 11.6 wants
			// this to self-heal. Hold the last state, advance the clock one step,
			// and keep draining actuator controls so the 4560 buffer cannot back up.
			vehicle.extrapolate(0.001);
			time_now_us += 1000;
			px4.Send(0);
			px4.Recieve(false);
			continue;
		}

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
			initial_time_s = time_now_us * 1.0e-6;
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
				vehicle.extrapolate(delta_time);
				// VehicleState advances its internal physics clock in
				// extrapolate()/setFAData(), so no extra offset here
				// (FG passes elapsed-us since the last sim frame instead)
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
		if (initial_time_s <= 0.0 || !have_fa_data) {
			dt = 0.001;
			initial_time_s = state.m_currentPhysicsTime_SEC - dt;
		}

		// glitch compensation: swallow 50 ms - 2 s network hiccups by
		// advancing the epoch; backwards > 500 ms is a true reset
		double dt_effective = dt;
		const uint64_t new_time_us = (uint64_t)((state.m_currentPhysicsTime_SEC - initial_time_s) * 1.0e6);

		if (new_time_us < time_now_us) {
			const uint64_t back_us = time_now_us - new_time_us;

			if (back_us > 500000) {
				// time going backwards
				time_now_us = new_time_us;
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
				dt_effective = glitch_threshold_us * 1.0e-6;
			}

			time_now_us += dt_us;
		}

		last_time_s = state.m_currentPhysicsTime_SEC;
		have_fa_data = true;

		// dt is the true elapsed physics time; dt_effective may have been capped
		// by the glitch compensation above
		vehicle.setFAData(state, dt_effective, dt);
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
