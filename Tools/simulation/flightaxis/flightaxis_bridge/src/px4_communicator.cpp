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
 * @file px4_communicator.cpp
 *
 * @author ThunderFly s.r.o., Vít Hanousek <info@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace
 *
 * PX4 communication socket.
 */

#include "px4_communicator.h"
#include <iostream>


PX4Communicator::PX4Communicator(VehicleState * v)
{
	this->vehicle=v;
}

const char *PX4Communicator::TransportName() const
{
    switch (transport) {
    case PX4Transport::Serial: return "serial";
    case PX4Transport::Udp:    return "udp";
    case PX4Transport::TcpServer:
    default:                   return "tcp-server";
    }
}

void PX4Communicator::Configure(PX4Transport t, PX4Profile p,
                                const char *device, int baud,
                                const char *host, int port,
                                double sensor_rate_hz)
{
    transport = t;
    profile = p;

    if (device != nullptr && device[0] != '\0') {
        serial_dev = device;
    }

    if (baud > 0) {
        serial_baud = baud;
    }

    if (host != nullptr && host[0] != '\0') {
        udp_host = host;
    }

    if (port > 0) {
        udp_port = port;
    }

    // HIL_SENSOR decimation. 0 means "every Send()", which is the SITL
    // behaviour and is what a USB link can also afford.
    if (sensor_rate_hz > 0.0) {
        sensor_interval_us = (uint64_t)(1.0e6 / sensor_rate_hz);
    } else {
        sensor_interval_us = 0;
    }

    if (profile == PX4Profile::Hitl) {
        // See the bandwidth block in px4_communicator.h. Both of these are
        // correctness issues on a real board, not just bandwidth savings:
        // HIL_STATE_QUATERNION would fight EKF2 for vehicle_attitude /
        // vehicle_local_position, and RAW_RPM has no receiver handler at all.
        state_quat_interval_us = INTERVAL_DISABLED;
        rpm_interval_us = INTERVAL_DISABLED;

        // A real GPS module is 5-10 Hz, so there is nothing to gain from
        // 10 Hz on a link where bytes are finite.
        gps_interval_us = 200000;   // 5 Hz

        // Real mag/baro/airspeed run far slower than the IMU; see the
        // fields_updated note in px4_communicator.h.
        mag_interval_us = 20000;        // 50 Hz
        baro_interval_us = 20000;       // 50 Hz
        diff_press_interval_us = 20000; // 50 Hz

        heartbeat_enabled = true;
    }
}

void PX4Communicator::EnableStateQuaternionBypass()
{
    state_quat_interval_us = STATE_QUAT_INTERVAL_US;
}

int PX4Communicator::Init(int portOffset)
{
    switch (transport) {
    case PX4Transport::Serial:
        return InitSerial();

    case PX4Transport::Udp:
        return InitUdp();

    case PX4Transport::TcpServer:
    default:
        return InitTcpServer(portOffset);
    }
}

/*
 * Map an integer baud rate to its termios constant. Only the rates PX4's own
 * serial layer offers are accepted (Tools/serial/serial_params.c.jinja);
 * anything else is a typo and is better rejected loudly than silently run at
 * the wrong speed.
 */
static bool baudToSpeed(int baud, speed_t &out)
{
    switch (baud) {
    case 9600:    out = B9600;    return true;
    case 19200:   out = B19200;   return true;
    case 38400:   out = B38400;   return true;
    case 57600:   out = B57600;   return true;
    case 115200:  out = B115200;  return true;
    case 230400:  out = B230400;  return true;
    case 460800:  out = B460800;  return true;
    case 500000:  out = B500000;  return true;
    case 921600:  out = B921600;  return true;
    case 1000000: out = B1000000; return true;
#ifdef B1500000
    case 1500000: out = B1500000; return true;
#endif
#ifdef B2000000
    case 2000000: out = B2000000; return true;
#endif
#ifdef B3000000
    case 3000000: out = B3000000; return true;
#endif
    default:      return false;
    }
}

int PX4Communicator::InitSerial()
{
    if (serial_dev == nullptr) {
        std::cerr << "PX4 Communicator: serial transport selected but no device set"
                  << " (set PX4_HITL_SERIAL_DEV)" << std::endl;
        return -1;
    }

    speed_t speed;

    if (!baudToSpeed(serial_baud, speed)) {
        std::cerr << "PX4 Communicator: unsupported baud rate " << serial_baud << std::endl;
        return -1;
    }

    // O_NONBLOCK so the open() itself cannot hang on a modem-control line that
    // never asserts; the fd stays non-blocking, which is what Recieve(false)
    // needs anyway and what keeps a stalled board from blocking Send().
    commFd = open(serial_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (commFd < 0) {
        std::cerr << "PX4 Communicator: cannot open " << serial_dev << ": "
                  << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tio;
    memset(&tio, 0, sizeof(tio));

    if (tcgetattr(commFd, &tio) != 0) {
        std::cerr << "PX4 Communicator: tcgetattr failed: " << strerror(errno) << std::endl;
        close(commFd);
        commFd = -1;
        return -1;
    }

    // Raw 8N1, no flow control. This mirrors what PX4's own mavlink module
    // configures on a serial link (mavlink_main.cpp:591-628): MAVLink framing
    // is binary, so any canonical/echo/translation processing corrupts it.
    // Hardware flow control is deliberately OFF: PX4 only enables it when
    // asked with -z/-Z, and a 3-wire TELEM cable has no RTS/CTS at all.
    cfmakeraw(&tio);
    tio.c_cflag &= ~(unsigned)CSIZE;
    tio.c_cflag |= CS8;             // 8 data bits
    tio.c_cflag &= ~(unsigned)PARENB; // no parity
    tio.c_cflag &= ~(unsigned)CSTOPB; // 1 stop bit
    tio.c_cflag &= ~(unsigned)CRTSCTS; // no hardware flow control
    tio.c_cflag |= (unsigned)(CLOCAL | CREAD); // ignore modem lines, enable receiver
    tio.c_iflag &= ~(unsigned)(IXON | IXOFF | IXANY); // no software flow control
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
        std::cerr << "PX4 Communicator: cfsetspeed failed: " << strerror(errno) << std::endl;
        close(commFd);
        commFd = -1;
        return -1;
    }

    if (tcsetattr(commFd, TCSANOW, &tio) != 0) {
        std::cerr << "PX4 Communicator: tcsetattr failed: " << strerror(errno) << std::endl;
        close(commFd);
        commFd = -1;
        return -1;
    }

    tcflush(commFd, TCIOFLUSH);

    std::cerr << "PX4 Communicator: serial " << serial_dev << " open at "
              << serial_baud << " baud (8N1, no flow control)" << std::endl;

    return 0;
}

int PX4Communicator::InitUdp()
{
    if (udp_host == nullptr) {
        std::cerr << "PX4 Communicator: udp transport selected but no host set"
                  << " (set PX4_HITL_UDP_HOST)" << std::endl;
        return -1;
    }

    commFd = socket(AF_INET, SOCK_DGRAM, 0);

    if (commFd < 0) {
        std::cerr << "PX4 Communicator: creating UDP socket failed: " << strerror(errno) << std::endl;
        return -1;
    }

    struct sockaddr_in board_addr;
    memset((char *) &board_addr, 0, sizeof(board_addr));
    board_addr.sin_family = AF_INET;
    board_addr.sin_port = htons((uint16_t)udp_port);

    if (inet_pton(AF_INET, udp_host, &board_addr.sin_addr) != 1) {
        std::cerr << "PX4 Communicator: bad UDP host address '" << udp_host << "'" << std::endl;
        close(commFd);
        commFd = -1;
        return -1;
    }

    // connect() a datagram socket so send()/recv() work without carrying the
    // peer address around, and so the kernel filters out anything that is not
    // from the board. This keeps Send()/Recieve() identical across transports.
    if (connect(commFd, (struct sockaddr *)&board_addr, sizeof(board_addr)) < 0) {
        std::cerr << "PX4 Communicator: UDP connect failed: " << strerror(errno) << std::endl;
        close(commFd);
        commFd = -1;
        return -1;
    }

    std::cerr << "PX4 Communicator: UDP client to " << udp_host << ":" << udp_port << std::endl;

    return 0;
}

/*
 * Write one framed MAVLink packet.
 *
 * UDP is a datagram transport: send() delivers the whole frame or none of it,
 * so there is nothing to retry and a short write cannot occur.
 *
 * Serial and TCP are BYTE STREAMS and are handled identically, because they
 * fail identically: a short write is possible on both (a full tty output
 * buffer on the non-blocking serial fd; a signal interrupting a socket write),
 * and abandoning the tail of a half-written frame desynchronises the MAVLink
 * parser on the far side permanently - it resynchronises only by luck, on the
 * next byte that happens to look like a start-of-frame. TCP used to take the
 * one-shot `send() == len` path and drop the remainder, so the transport that
 * carries every SITL run had strictly weaker framing guarantees than the one
 * added for HITL.
 */
bool PX4Communicator::SendBuffer(const uint8_t *buf, int len)
{
    if (commFd < 0) {
        return false;
    }

    if (transport == PX4Transport::Udp) {
        return send(commFd, buf, (size_t)len, MSG_NOSIGNAL) == (ssize_t)len;
    }

    int written = 0;

    while (written < len) {
        ssize_t n;

        if (transport == PX4Transport::Serial) {
            n = write(commFd, buf + written, (size_t)(len - written));

        } else {
            // MSG_NOSIGNAL rather than relying on the bridge's SIGPIPE
            // disposition: a closed peer must surface as EPIPE here, which
            // NoteSendFailure() treats as fatal.
            n = send(commFd, buf + written, (size_t)(len - written), MSG_NOSIGNAL);
        }

        if (n > 0) {
            written += (int)n;
            continue;
        }

        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // Output buffer full: the link is saturated. Wait briefly for
            // drain rather than dropping a half-written frame.
            struct pollfd pfd = {};
            pfd.fd = commFd;
            pfd.events = POLLOUT;

            if (poll(&pfd, 1, 100) <= 0) {
                return false;
            }

            continue;
        }

        if (n < 0 && errno == EINTR) {
            continue;
        }

        return false;
    }

    return true;
}

int PX4Communicator::InitTcpServer(int portOffset)
{

    memset((char *) &simulator_mavlink_addr, 0, sizeof(px4_mavlink_addr));
    memset((char *) &px4_mavlink_addr, 0, sizeof(px4_mavlink_addr));
    simulator_mavlink_addr.sin_family = AF_INET;
    simulator_mavlink_addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    simulator_mavlink_addr.sin_port = htons(portBase+portOffset);

    if ((listenMavlinkSock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        std::cerr<<"PX4 Communicator: Creating TCP socket failed: " << strerror(errno) << std::endl;
    }

    //do not accumulate messages by waiting for ACK
    int yes = 1;
    int result = setsockopt(listenMavlinkSock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    if (result != 0)
    {
        std::cerr<<"PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }

    //try to close as fast as posible 
    struct linger nolinger;
    nolinger.l_onoff = 1;
    nolinger.l_linger = 0;
    result = setsockopt(listenMavlinkSock, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
    if (result != 0)
    {
        std::cerr<<"PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }

    // The socket reuse is necessary for reconnecting to the same address
    // if the socket does not close but gets stuck in TIME_WAIT. This can happen
    // if the server is suddenly closed, for example if the simulated vehicle goes away.
    int socket_reuse = 1;
    result = setsockopt(listenMavlinkSock, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
    if (result != 0) 
    {
         std::cerr<<"PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }

    // Same as above but for a given port
    result = setsockopt(listenMavlinkSock, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
    if (result != 0) 
    {
        std::cerr<<"PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }


    if (bind(listenMavlinkSock, (struct sockaddr *)&simulator_mavlink_addr, sizeof(simulator_mavlink_addr)) < 0)
    {
        std::cerr<<"PX4 Communicator: bind failed:  " << strerror(errno) << std::endl;
    }

    errno = 0;
    result=listen(listenMavlinkSock, 5);
    if (result < 0)
    {
        std::cerr<<"PX4 Communicator: listen failed: " << strerror(errno) << std::endl;
    }

    sleep(5);

    unsigned int px4_addr_len=sizeof(px4_mavlink_addr);
    while(true)
    {
        px4MavlinkSock = accept(listenMavlinkSock, (struct sockaddr *)&px4_mavlink_addr, &px4_addr_len);
        if (px4MavlinkSock<0)
        {
            std::cerr<<"PX4 Communicator: accept failed: " << strerror(errno) << std::endl;
        }
        else
        {
            std::cerr<<"PX4 Communicator: PX4 Connected."<< std::endl;
            commFd = px4MavlinkSock;
            break;
        }
    }

	return result;
}

/*void PX4Communicator::CheckClientReconect()
{
    struct pollfd fds[1] = {};
    fds[0].fd = listenMavlinkSock;
    fds[0].events = POLLIN;

    int p=poll(&fds[0], 1, 1);
    if(p<0)
        fprintf(stderr,"Pool for new client error\n");

    if(p==0)
    {
        //fprintf(stderr,"No new Client\n");
    }
    else
    {
        fprintf(stderr,"New Client Connected to Bridge\n");
        close(px4MavlinkSock);
        unsigned int px4_addr_len=sizeof(px4_mavlink_addr);;
        px4MavlinkSock = accept(listenMavlinkSock, (struct sockaddr *)&px4_mavlink_addr, &px4_addr_len);
    }
}*/


int PX4Communicator::Clean()
{
    if (transport == PX4Transport::TcpServer)
    {
        if (px4MavlinkSock >= 0)
        {
            close(px4MavlinkSock);
            px4MavlinkSock = -1;
        }

        if (listenMavlinkSock >= 0)
        {
            close(listenMavlinkSock);
            listenMavlinkSock = -1;
        }
    }
    else if (commFd >= 0)
    {
        close(commFd);
    }

    commFd = -1;
    return 0;
}

/*
 * Wall-clock microseconds. Used for the heartbeat interval and the send-failure
 * log dedup, both of which have to keep working when the physics clock does not.
 */
static uint64_t wallMicros()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}

void PX4Communicator::MarkLinkLost(const char *reason)
{
    if (link_lost) {
        return;
    }

    link_lost = true;
    std::cerr << "PX4 Communicator: PX4 link is DEAD (" << reason << ")."
              << " No further messages will be sent; the bridge will shut down"
              << " (this also stops the RealFlight SOAP traffic)." << std::endl;
}

/*
 * One failed write. Logs at most once a second - a dead link fails on every
 * message of every Send(), which without dedup is thousands of identical lines
 * per second - and applies the dead-link policy from px4_communicator.h.
 */
void PX4Communicator::NoteSendFailure()
{
    const int err = errno;
    send_fail_count++;

    const uint64_t now_us = wallMicros();

    if (send_fail_count == 1 || now_us - last_send_fail_log_us >= 1000000)
    {
        last_send_fail_log_us = now_us;
        std::cerr << "PX4 Communicator: send to PX4 failed (" << send_fail_count
                  << " consecutive): " << strerror(err) << std::endl;
    }

    // Fatal errnos mean the peer is gone and will not come back on this fd.
    if (err == EPIPE || err == ECONNRESET || err == ENOTCONN || err == ECONNREFUSED
        || err == EBADF || err == ENXIO || err == ENODEV || err == EIO || err == ESHUTDOWN)
    {
        MarkLinkLost(strerror(err));
        return;
    }

    // A serial link that never drains has no distinguishing errno (SendBuffer
    // just times out in poll(POLLOUT)), so fall back on a repetition count.
    if (send_fail_count >= SEND_FAIL_LIMIT)
    {
        MarkLinkLost("too many consecutive send failures");
    }
}

bool PX4Communicator::Emit(mavlink_message_t &msg)
{
    if (link_lost)
    {
        return false;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int packetlen = mavlink_msg_to_send_buffer(buffer, &msg);

    if (SendBuffer(buffer, packetlen))
    {
        if (send_fail_count > 0)
        {
            std::cerr << "PX4 Communicator: send recovered after " << send_fail_count
                      << " failures" << std::endl;
            send_fail_count = 0;
        }

        return true;
    }

    NoteSendFailure();
    return false;
}

/*
 * HEARTBEAT is paced off the WALL clock, not the physics clock: its whole job
 * is to prove the link is alive, including while RealFlight is not producing
 * frames (and, via SendHeartbeat() from the bridge's injection retry loop,
 * before RealFlight is reachable at all).
 */
int PX4Communicator::SendHeartbeat()
{
    if (!heartbeat_enabled || link_lost)
    {
        return 0;
    }

    const uint64_t wall_us = wallMicros();
    const uint64_t hb_interval = seen_controls ? HEARTBEAT_SLOW_INTERVAL_US
                                               : HEARTBEAT_FAST_INTERVAL_US;

    if (last_heartbeat_us != 0 && wall_us - last_heartbeat_us < hb_interval)
    {
        return 0;
    }

    last_heartbeat_us = wall_us;

    mavlink_heartbeat_t hb{};
    hb.type = MAV_TYPE_GENERIC;
    hb.autopilot = MAV_AUTOPILOT_INVALID;   // we are not an autopilot
    hb.base_mode = 0;
    hb.custom_mode = 0;
    hb.system_status = MAV_STATE_ACTIVE;

    // BRIDGE_SYSID/BRIDGE_COMPID, not the vehicle's 1/200 - see the identity
    // note in px4_communicator.h.
    mavlink_message_t msg;
    mavlink_msg_heartbeat_encode_chan(BRIDGE_SYSID, BRIDGE_COMPID, MAVLINK_COMM_0, &msg, &hb);

    return Emit(msg) ? 0 : -1;
}

int PX4Communicator::Send(int offset_us)
{

    mavlink_message_t msg;

    if (link_lost)
    {
        return -1;
    }

    // Every message is decimated against the bridge's own physics clock: see
    // the interval members and the bandwidth block in px4_communicator.h.
    // In the SITL profile sensor_interval_us and rpm_interval_us are 0, so
    // those two go out on every call exactly as before.
    const uint64_t now_us = vehicle->timeUsec() + offset_us;

    if (!sent_first)
    {
        // send one of each immediately so PX4 has a full picture from frame 1
        last_sensor_us = last_gps_us = last_state_quat_us = last_distance_us
            = last_rpm_us = last_mag_us = last_baro_us = last_diff_press_us
            = now_us - 1000000;
        sent_first = true;
    }

    if (SendHeartbeat() != 0)
    {
        return -1;
    }

    if (now_us - last_sensor_us >= sensor_interval_us)
    {
        last_sensor_us = now_us;

        mavlink_hil_sensor_t sensor_msg =vehicle->getSensorMsg(offset_us);

        // Mask the slow sensors down to their own rates. vehicle_state always
        // reports 0x1FFF ("everything is new"), which at the rate Send() runs
        // at over-publishes baro/mag/pitot on BOTH targets - see the
        // fields_updated block in px4_communicator.h. Accel and gyro are never
        // masked, so the IMU stream is untouched.
        {
            uint32_t fields = sensor_msg.fields_updated & FIELD_ALL;

            if (now_us - last_mag_us >= mag_interval_us) {
                last_mag_us = now_us;

            } else {
                fields &= ~FIELD_MAG;
            }

            if (now_us - last_baro_us >= baro_interval_us) {
                last_baro_us = now_us;

            } else {
                fields &= ~FIELD_BARO;
            }

            if (now_us - last_diff_press_us >= diff_press_interval_us) {
                last_diff_press_us = now_us;

            } else {
                fields &= ~FIELD_DIFF_PRESS;
            }

            sensor_msg.fields_updated = fields;
        }

        mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);

        if (!Emit(msg))
        {
            return -1;
        }
    }

    if (gps_interval_us != INTERVAL_DISABLED && now_us - last_gps_us >= gps_interval_us)
    {
        last_gps_us = now_us;

        mavlink_hil_gps_t hil_gps_msg=vehicle->hil_gps_msg;
        hil_gps_msg.time_usec+=offset_us;

        mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);

        if (!Emit(msg))
        {
            return -1;
        }
    }

    // ground truth (SITL only - in HITL this topic is consumed by the board's
    // mavlink receiver and would fight EKF2; see Configure())
    if (state_quat_interval_us != INTERVAL_DISABLED
        && now_us - last_state_quat_us >= state_quat_interval_us)
    {
        last_state_quat_us = now_us;

        mavlink_hil_state_quaternion_t state_quat_msg = vehicle->getStateQuatMsg(offset_us);

        mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &state_quat_msg);

        if (!Emit(msg))
        {
            return -1;
        }
    }

    // rangefinder - only while the reading is valid (vehicle not inverted)
    if (distance_interval_us != INTERVAL_DISABLED && vehicle->rangefinderValid()
        && now_us - last_distance_us >= distance_interval_us)
    {
        last_distance_us = now_us;

        mavlink_distance_sensor_t dist_msg = vehicle->getDistanceSensorMsg(offset_us);

        mavlink_msg_distance_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &dist_msg);

        if (!Emit(msg))
        {
            return -1;
        }
    }

    // RAW_RPM (SITL only - mavlink_receiver.cpp has no handler for it, so on a
    // real board it is purely wasted serial bandwidth; see Configure())
    if (rpm_interval_us != INTERVAL_DISABLED && now_us - last_rpm_us >= rpm_interval_us)
    {
        last_rpm_us = now_us;

        mavlink_raw_rpm_t rpmmessage;
        rpmmessage.index=0;
        rpmmessage.frequency=vehicle->rpm;
        mavlink_msg_raw_rpm_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &rpmmessage);

        if (!Emit(msg))
        {
            return -1;
        }
    }

    return 0;
}

int PX4Communicator::Recieve(bool blocking)
{

        mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        if (commFd < 0)
        {
            return 0;
        }

        struct pollfd fds[1] = {};
        fds[0].fd = commFd;
        fds[0].events = POLLIN;

        // The non-blocking flavour must not stall the bridge loop: RealFlight's SOAP
        // round-trip is the only thing allowed to pace us. Drain everything that is
        // already queued and keep the newest actuator controls (latest wins).
        int got = 0;

        while (true)
        {
            int p=poll(&fds[0], 1, (blocking && got==0 ? -1 : 0));

            if(p<0)
            {
                std::cerr<<"PX4 Communicator: PX4 Pool error\n" << std::endl;
                break;
            }

            if(p==0)
                break;      // nothing (more) pending

            if(!(fds[0].revents & POLLIN))
            {
                // Checked only when there is no readable data left, because a
                // TCP peer that closed after writing raises POLLHUP alongside
                // POLLIN and that data is still ours to drain.
                if (fds[0].revents & (POLLHUP | POLLERR | POLLNVAL))
                {
                    MarkLinkLost("poll reported hangup/error (PX4 gone, or the device was removed)");
                }

                break;
            }

            ssize_t rlen;

            if (transport == PX4Transport::Serial)
            {
                rlen = read(commFd, buffer, sizeof(buffer));
            }
            else
            {
                unsigned int slen=sizeof(px4_mavlink_addr);
                rlen = recvfrom(commFd, buffer, sizeof(buffer), 0, (struct sockaddr *)&px4_mavlink_addr, &slen);
            }

            // A 0-length read on a stream socket is an orderly close: the PX4
            // process exited. (On a serial fd VMIN=0 makes 0 a normal "nothing
            // to read", so this must stay TCP-only.)
            if (rlen == 0 && transport == PX4Transport::TcpServer)
            {
                MarkLinkLost("PX4 closed the connection");
                break;
            }

            if (rlen <= 0)
                break;

            const unsigned int len = (unsigned int)rlen;

            mavlink_status_t status;
            for (unsigned i = 0; i < len; ++i)
            {
              if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
              {
                    if(msg.msgid==MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS)
                    {
                            mavlink_hil_actuator_controls_t controls;
                            mavlink_msg_hil_actuator_controls_decode(&msg, &controls);
                            vehicle->setPXControls(controls);
                            got = 1;
                            seen_controls = true;   // slows the HITL heartbeat to 1 Hz
                    }
              }
            }
        }

    return got;
}

