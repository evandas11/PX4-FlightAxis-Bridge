/****************************************************************************
 *
 *   Copyright (c) 2020 ThunderFly s.r.o.. All rights reserved.
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

/****************************************************************************
 *
 * MODIFICATION NOTICE
 *
 * This file is adapted from PX4-FlightGear-Bridge for the PX4-FlightAxis-Bridge
 * project. Changes made here by the PX4-FlightAxis-Bridge contributors:
 *  - per-message decimation of HIL_GPS (10 Hz), HIL_STATE_QUATERNION (50 Hz)
 *    and DISTANCE_SENSOR (20 Hz), because Send() runs at RealFlight's ~250 Hz
 *    frame rate plus every 1 ms extrapolation step and only HIL_SENSOR needs
 *    to go out at that full rate
 *  - the HIL_STATE_QUATERNION (ground truth) and DISTANCE_SENSOR sends
 *  - a non-blocking receive drain, so the bridge is never stalled by PX4
 *  - a transport abstraction (TCP server / serial / UDP client) so the same
 *    sensor stream can drive either PX4 SITL or a real flight controller
 *    board running HITL firmware, plus the HITL message profile and the
 *    configurable HIL_SENSOR rate that serial bandwidth makes necessary
 *
 * The file is distributed as part of a work licensed under GPLv3 or later, but
 * its own terms remain the BSD-3-Clause licence above, as GPLv3 section 7
 * permits. It is not relicensed.
 *
 ****************************************************************************/

/**
 * @file px4_communicator.h
 *
 * @author ThunderFly s.r.o., Vít Hanousek <info@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace
 *
 * PX4 communication socket.
 */

#ifndef PX4_COMMUNICATOR_H
#define PX4_COMMUNICATOR_H

#include "vehicle_state.h"


#include <stdio.h>
#include <common/mavlink.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <cstdint>
#include <time.h>

#define TIMEOUTS 5
#define TIMEOUTUS 0

/*
 * Transport selection.
 *
 * TcpServer is the original (and default) SITL behaviour: listen on
 * 4560+instance and wait for the px4 binary to connect. Serial and Udp exist
 * for HITL, where the firmware runs on a real flight controller board.
 *
 * Note there is deliberately no TCP *client* mode: PX4's mavlink module has
 * no TCP transport at all. Protocol is SERIAL or UDP only
 * (src/modules/mavlink/mavlink_main.h:98-101), so a TCP client would have
 * nothing to talk to on a real board.
 */
enum class PX4Transport {
	TcpServer,	// SITL: TCP server on 4560+instance (default, unchanged)
	Serial,		// HITL: /dev/ttyACM* or /dev/ttyUSB*
	Udp,		// HITL: UDP client to a board with Ethernet (CONFIG_NET)
};

/*
 * Message profile.
 *
 * Sitl targets src/modules/simulation/simulator_mavlink, which consumes
 * HIL_STATE_QUATERNION as ground truth and RAW_RPM as telemetry.
 *
 * Hitl targets the mavlink module's receiver on a real board, which is a
 * genuinely different consumer:
 *
 *  - HIL_STATE_QUATERNION must NOT be sent. In HITL,
 *    MavlinkReceiver::handle_message_hil_state_quaternion()
 *    (mavlink_receiver.cpp:2602-2732) publishes vehicle_attitude,
 *    vehicle_local_position and vehicle_global_position DIRECTLY - it is a
 *    second publisher on EKF2's own output topics and fights the estimator.
 *    PX4 only streams it back out as ground truth when SYS_HITL==2 (SIH);
 *    see mavlink_main.cpp:675-680.
 *  - RAW_RPM must NOT be sent: mavlink_receiver.cpp has no handler for it,
 *    so it is pure wasted bandwidth on a serial link.
 *  - HIL_SENSOR / HIL_GPS / DISTANCE_SENSOR are all consumed
 *    (mavlink_receiver.cpp:347-378 for the HIL set, :207 for DISTANCE_SENSOR,
 *    which is handled unconditionally and so needs no HIL gating).
 */
enum class PX4Profile {
	Sitl,
	Hitl,
};

/*
 * BANDWIDTH ARITHMETIC (drives the HITL defaults below).
 *
 * On-wire cost = payload + 12 bytes of MAVLink v2 framing (10 byte header +
 * 2 byte CRC, no signing). Payloads are trailing-zero truncated by
 * mavlink_msg_to_send_buffer(), so these are the realistic sizes:
 *
 *   HIL_SENSOR            64 + 12 =  76 B   (LEN 65, id=0 truncates to 64)
 *   HIL_GPS               39 + 12 =  51 B
 *   HIL_STATE_QUATERNION  64 + 12 =  76 B
 *   DISTANCE_SENSOR       39 + 12 =  51 B
 *   RAW_RPM                5 + 12 =  17 B
 *   HIL_ACTUATOR_CONTROLS 81 + 12 =  93 B   (inbound, from the board)
 *
 * A serial line at 8N1 carries 10 bits per byte, so bytes/s = baud/10.
 *
 * SITL profile as sent today: HIL_SENSOR and RAW_RPM go out on every Send(),
 * and Send() runs once per RealFlight frame (~250 Hz) plus once per 1 ms
 * extrapolation step, i.e. ~1000 Hz. That is
 *   1000*76 + 1000*17 + 10*51 + 50*76 + 20*51 = ~98.3 kB/s,
 * which is nothing over TCP loopback but exceeds even 921600 baud
 * (92.2 kB/s). Serial HITL therefore MUST decimate HIL_SENSOR.
 *
 * HITL profile (no HIL_STATE_QUATERNION, no RAW_RPM) at 250 Hz HIL_SENSOR:
 *   250*76 + 10*51 + 20*51 = 19000 + 510 + 1020 = 20530 B/s = 205.3 kbit/s
 *
 * Against that budget:
 *   baud      capacity     load    verdict
 *   115200    11.5 kB/s    178%    unusable (even 100 Hz is over)
 *   230400    23.0 kB/s     89%    too tight, no headroom
 *   460800    46.1 kB/s     45%    fine at 250 Hz
 *   921600    92.2 kB/s     22%    fine; headroom for 500 Hz (43%)
 *   USB CDC   ~100 kB/s     21%    baud is nominal; see below
 *
 * 250 Hz is therefore the HITL default: it matches RealFlight's own physics
 * frame rate (so no information is lost - the extrapolated 1 ms steps are
 * interpolation, not new data), and it fits comfortably from 460800 up.
 *
 * The return direction is independent (serial is full duplex): PX4 streams
 * HIL_ACTUATOR_CONTROLS at 200 Hz (mavlink_main.cpp:673) = 200*93 =
 * 18.6 kB/s. Note PX4 refuses to enable the HIL streams at all unless its
 * own link datarate exceeds 5000 B/s (mavlink_main.cpp:671), and MAV_x_RATE
 * defaults to only 1200 B/s - see HITL.md, this is the single most common
 * reason a HITL link looks alive but never sends actuator controls.
 *
 * On USB (/dev/ttyACM*) PX4 ignores baud entirely and defaults its datarate
 * to 100000 B/s (mavlink_main.cpp:2166-2181), so USB is the path of least
 * resistance and can carry the full 1000 Hz stream if desired
 * (PX4_HITL_SENSOR_HZ=0 disables decimation).
 *
 * For comparison: jMAVSim and Gazebo do not decimate either - they emit
 * HIL_SENSOR at their physics rate (250 Hz and 250-1000 Hz respectively)
 * over UDP/TCP loopback, where bandwidth is free. Neither is designed for a
 * serial HITL link, which is exactly why this decimation had to be added.
 * (jMAVSim's documented HITL invocation is -b 921600 -r 250, which is where
 * the 250 Hz default comes from.)
 *
 * RATE QUANTISATION - worth knowing when reading measured rates.
 *
 * Every gate below is "if (now - last >= interval) { last = now; ... }"
 * against the PHYSICS clock, so the period actually achieved is the
 * requested period rounded UP to a multiple of however far the physics
 * clock moves per Send() call. When the main loop free-runs (it does
 * several thousand iterations/s against RealFlight) that step is the 1 ms
 * extrapolation quantum and 250 Hz comes out exact. If the loop is throttled
 * - a slow link, a stalled reader - the step grows and the achieved rate
 * drops: at a 3 ms step, a 4 ms interval yields 6 ms, i.e. 166 Hz rather
 * than 250 Hz. This was measured on a PTY harness and is the reason the
 * numbers there sit below target.
 *
 * The error is always in the safe direction (never faster than requested,
 * so never over the bandwidth budget), which is why the simple "last = now"
 * form is kept rather than accumulating "last += interval" - the latter
 * would burst to catch up after exactly the kind of stall that caused it.
 */

class PX4Communicator
{

private:
	VehicleState *vehicle;


	struct sockaddr_in  px4_mavlink_addr;
	struct sockaddr_in  simulator_mavlink_addr;
	int listenMavlinkSock = -1;
	int px4MavlinkSock = -1;

	// The fd every Send()/Recieve() actually uses. For TcpServer this is
	// px4MavlinkSock (the accepted connection); for Serial it is the tty;
	// for Udp it is a connect()ed datagram socket. Keeping one fd here is
	// what lets the message code below stay transport agnostic.
	int commFd = -1;

	PX4Transport transport = PX4Transport::TcpServer;
	PX4Profile profile = PX4Profile::Sitl;

	// Serial / UDP configuration (unused in TcpServer mode)
	const char *serial_dev = nullptr;
	int serial_baud = 921600;
	const char *udp_host = nullptr;
	int udp_port = 14550;

    const int portBase=4560;

	// Send() runs at the RealFlight frame rate (~250 Hz) plus every 1 ms
	// extrapolation step. HIL_SENSOR must go out at that full rate, but PX4
	// publishes one sensor_gps per HIL_GPS with no rate limiting of its own,
	// and HIL_STATE_QUATERNION / DISTANCE_SENSOR do not need it either.
	static const uint64_t GPS_INTERVAL_US = 100000;      // 10 Hz  (spec 8.2)
	static const uint64_t STATE_QUAT_INTERVAL_US = 20000; // 50 Hz (ground truth only)
	static const uint64_t DISTANCE_INTERVAL_US = 50000;   // 20 Hz (spec 6)

	// Sentinel for "never send this message in this profile".
	static const uint64_t INTERVAL_DISABLED = UINT64_MAX;

	// Effective intervals. These default to the SITL behaviour exactly
	// (sensor and rpm every Send(), i.e. interval 0) and are only changed by
	// Configure() for the HITL profile, so the SITL path is untouched.
	uint64_t sensor_interval_us = 0;
	uint64_t gps_interval_us = GPS_INTERVAL_US;
	uint64_t state_quat_interval_us = STATE_QUAT_INTERVAL_US;
	uint64_t distance_interval_us = DISTANCE_INTERVAL_US;
	uint64_t rpm_interval_us = 0;

	uint64_t last_sensor_us = 0;
	uint64_t last_gps_us = 0;
	uint64_t last_state_quat_us = 0;
	uint64_t last_distance_us = 0;
	uint64_t last_rpm_us = 0;
	bool sent_first = false;

	/*
	 * Sub-rates WITHIN HIL_SENSOR.
	 *
	 * vehicle_state.cpp sets fields_updated = 0x1FFF, i.e. "every sensor is
	 * new", on every message. In SITL that is harmless. On a real board it
	 * means MavlinkReceiver::handle_message_hil_sensor()
	 * (mavlink_receiver.cpp:2302-2402) republishes the magnetometer, the
	 * barometer and the differential pressure at the full HIL_SENSOR rate -
	 * 250 Hz of baro and mag into an MCU whose real drivers would run those
	 * at 50-100 Hz. It costs no extra bandwidth (the fields are in the
	 * message either way) but it does cost CPU and it misrepresents the
	 * sensor suite to the estimator.
	 *
	 * So in the HITL profile the accel/gyro bits stay set on every message
	 * and the mag/baro/diff-pressure bits are masked out except at their own
	 * interval. Bit values are SensorSource in mavlink_receiver.h:365-371.
	 */
	static const uint32_t FIELD_ACCEL      = 0b111;
	static const uint32_t FIELD_GYRO       = 0b111000;
	static const uint32_t FIELD_MAG        = 0b111000000;
	static const uint32_t FIELD_BARO       = 0b1101000000000;
	static const uint32_t FIELD_DIFF_PRESS = 0b10000000000;
	static const uint32_t FIELD_ALL        = 0x1FFF;

	// 0 = every HIL_SENSOR (SITL behaviour, unchanged)
	uint64_t mag_interval_us = 0;
	uint64_t baro_interval_us = 0;
	uint64_t diff_press_interval_us = 0;

	uint64_t last_mag_us = 0;
	uint64_t last_baro_us = 0;
	uint64_t last_diff_press_us = 0;

	/*
	 * HEARTBEAT. Required in HITL and not in SITL: PX4 will not bring a USB
	 * MAVLink instance up until it hears from the other end, and without it
	 * the CDC-ACM buffer simply fills. Gazebo Classic does the same thing for
	 * the same reason (sitl_gazebo-classic mavlink_interface.cpp:215-228).
	 * Sent quickly until the board answers with HIL_ACTUATOR_CONTROLS, then
	 * at 1 Hz to keep the link marked alive.
	 */
	static const uint64_t HEARTBEAT_FAST_INTERVAL_US = 100000;	// 10 Hz
	static const uint64_t HEARTBEAT_SLOW_INTERVAL_US = 1000000;	// 1 Hz
	bool heartbeat_enabled = false;
	uint64_t last_heartbeat_us = 0;
	bool seen_controls = false;

	// Transport helpers
	int InitTcpServer(int portOffset);
	int InitSerial();
	int InitUdp();
	bool SendBuffer(const uint8_t *buf, int len);

public:
	PX4Communicator(VehicleState *v);

	/*
	 * Select the transport before Init(). Called by the bridge from the
	 * environment; not calling it at all leaves the SITL TCP server default.
	 *
	 * sensor_rate_hz: 0 = send HIL_SENSOR on every Send() (SITL behaviour),
	 * otherwise decimate to that rate.
	 */
	void Configure(PX4Transport t, PX4Profile p,
		       const char *device, int baud,
		       const char *host, int port,
		       double sensor_rate_hz);

	/*
	 * HITL only, and dangerous: re-enable HIL_STATE_QUATERNION.
	 *
	 * On a real board this is NOT ground truth - the receiver publishes it
	 * straight onto vehicle_attitude / vehicle_local_position /
	 * vehicle_global_position, racing EKF2 on the estimator's own output
	 * topics. The vehicle then flies well and proves nothing, because it is
	 * flying on injected truth. This is the Gazebo hil_state_level=1
	 * semantic and it exists only for deliberately bypassing the estimator.
	 */
	void EnableStateQuaternionBypass();

	int Init(int portOffset);
	int Clean();
    //void CheckClientReconect();

	int Send(int offset_us);
	int Recieve(bool blocking);

	// For startup logging
	const char *TransportName() const;

	int Test();
};


#endif
