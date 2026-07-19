/****************************************************************************
 *
 * This file is part of the PX4-FlightAxis-Bridge project.
 * Copyright (c) 2026 Evangels Brilliant Dasmasela
 *
 * The socket/creator-thread design, the SOAP request bodies, the reply parser
 * key table and scan, and the startup and reconnect logic in this file are
 * ported from ArduPilot libraries/SITL/SIM_FlightAxis.{h,cpp}
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
 ****************************************************************************/

/****************************************************************************
 *
 * FlightAxis (RealFlight) SOAP communicator for the PX4 flightaxis bridge.
 *
 * Ported from ArduPilot SIM_FlightAxis.{h,cpp}:
 *  - one new TCP connection per SOAP request, hidden behind a background
 *    socket-creator thread that always keeps one connected socket parked
 *  - HTTP POST framing and SOAP envelopes copied verbatim
 *  - reply reading via Content-Length
 *  - extremely primitive sequential strstr parser over a key table in
 *    document order (uav.tridgell.net/RealFlight/data-exchange.txt)
 *
 ****************************************************************************/

#include "fa_communicator.h"

#include <chrono>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>

/*
  the ExchangeData reply key table, in document order. The 12 "item"
  echoes (physical TX channels) come first, then the m-* fields. The
  order matters: the parser scans forward through the document, which
  is what allows the repeated "item" key to decode as an array.
 */
enum {
    KEY_ITEM0 = 0,      // rcin[0..11]
    // 1..11 are the remaining items
    KEY_AIRSPEED = 12,
    KEY_ALT_ASL,
    KEY_ALT_AGL,
    KEY_GROUNDSPEED,
    KEY_PITCH_RATE,
    KEY_ROLL_RATE,
    KEY_YAW_RATE,
    KEY_AZIMUTH,
    KEY_INCLINATION,
    KEY_ROLL,
    KEY_POS_X,
    KEY_POS_Y,
    KEY_VEL_WORLD_U,
    KEY_VEL_WORLD_V,
    KEY_VEL_WORLD_W,
    KEY_VEL_BODY_U,
    KEY_VEL_BODY_V,
    KEY_VEL_BODY_W,
    KEY_ACC_WORLD_AX,
    KEY_ACC_WORLD_AY,
    KEY_ACC_WORLD_AZ,
    KEY_ACC_BODY_AX,
    KEY_ACC_BODY_AY,
    KEY_ACC_BODY_AZ,
    KEY_WIND_X,
    KEY_WIND_Y,
    KEY_WIND_Z,
    KEY_PROP_RPM,
    KEY_HELI_RPM,
    KEY_BATT_VOLTS,
    KEY_BATT_AMPS,
    KEY_BATT_MAH,        // parsed, not exposed in FAState
    KEY_FUEL_OZ,
    KEY_IS_LOCKED,
    KEY_LOST_COMPONENTS,
    KEY_ENGINE_RUNNING,
    KEY_TOUCHING_GROUND,
    KEY_AIRCRAFT_STATUS, // parsed, not exposed in FAState
    KEY_PHYSICS_TIME,
    KEY_PHYSICS_SPEED,
    KEY_QUAT_X,
    KEY_QUAT_Y,
    KEY_QUAT_Z,
    KEY_QUAT_W,
    KEY_CONTROLLER_ACTIVE,
    KEY_RESET_PRESSED,
    KEY_COUNT
};

static_assert(KEY_COUNT == 58, "key table size mismatch");

static const char *const key_names[KEY_COUNT] = {
    "item", "item", "item", "item", "item", "item",
    "item", "item", "item", "item", "item", "item",
    "m-airspeed-MPS",
    "m-altitudeASL-MTR",
    "m-altitudeAGL-MTR",
    "m-groundspeed-MPS",
    "m-pitchRate-DEGpSEC",
    "m-rollRate-DEGpSEC",
    "m-yawRate-DEGpSEC",
    "m-azimuth-DEG",
    "m-inclination-DEG",
    "m-roll-DEG",
    "m-aircraftPositionX-MTR",
    "m-aircraftPositionY-MTR",
    "m-velocityWorldU-MPS",
    "m-velocityWorldV-MPS",
    "m-velocityWorldW-MPS",
    "m-velocityBodyU-MPS",
    "m-velocityBodyV-MPS",
    "m-velocityBodyW-MPS",
    "m-accelerationWorldAX-MPS2",
    "m-accelerationWorldAY-MPS2",
    "m-accelerationWorldAZ-MPS2",
    "m-accelerationBodyAX-MPS2",
    "m-accelerationBodyAY-MPS2",
    "m-accelerationBodyAZ-MPS2",
    "m-windX-MPS",
    "m-windY-MPS",
    "m-windZ-MPS",
    "m-propRPM",
    "m-heliMainRotorRPM",
    "m-batteryVoltage-VOLTS",
    "m-batteryCurrentDraw-AMPS",
    "m-batteryRemainingCapacity-MAH",
    "m-fuelRemaining-OZ",
    "m-isLocked",
    "m-hasLostComponents",
    "m-anEngineIsRunning",
    "m-isTouchingGround",
    "m-currentAircraftStatus",
    "m-currentPhysicsTime-SEC",
    "m-currentPhysicsSpeedMultiplier",
    "m-orientationQuaternion-X",
    "m-orientationQuaternion-Y",
    "m-orientationQuaternion-Z",
    "m-orientationQuaternion-W",
    "m-flightAxisControllerIsActive",
    "m-resetButtonHasBeenPressed",
};

/*
  SOAP bodies for the startup sequence, copied from ArduPilot
 */
static const char *const soap_body_restore =
    "<?xml version='1.0' encoding='UTF-8'?>\n"
    "<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>\n"
    "<soap:Body>\n"
    "<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>\n"
    "</soap:Body>\n"
    "</soap:Envelope>";

static const char *const soap_body_reset =
    "<?xml version='1.0' encoding='UTF-8'?>\n"
    "<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>\n"
    "<soap:Body>\n"
    "<ResetAircraft><a>1</a><b>2</b></ResetAircraft>\n"
    "</soap:Body>\n"
    "</soap:Envelope>";

static const char *const soap_body_inject =
    "<?xml version='1.0' encoding='UTF-8'?>\n"
    "<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>\n"
    "<soap:Body>\n"
    "<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>\n"
    "</soap:Body>\n"
    "</soap:Envelope>";

static const char *const soap_body_exchange_fmt =
    "<?xml version='1.0' encoding='UTF-8'?><soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>\n"
    "<soap:Body>\n"
    "<ExchangeData>\n"
    "<pControlInputs>\n"
    "<m-selectedChannels>%u</m-selectedChannels>\n"
    "<m-channelValues-0to1>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "<item>%.4f</item>\n"
    "</m-channelValues-0to1>\n"
    "</pControlInputs>\n"
    "</ExchangeData>\n"
    "</soap:Body>\n"
    "</soap:Envelope>";

/*
  connect a TCP socket with a millisecond timeout. Returns a connected,
  blocking fd with TCP_NODELAY set, or -1.
 */
static int connect_timeout(const char *ip, uint16_t port, uint32_t timeout_ms)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
        printf("[flightaxis_bridge] bad RealFlight IP '%s'\n", ip);
        close(fd);
        return -1;
    }

    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    // non-blocking connect with poll timeout
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int ret = connect(fd, (struct sockaddr *)&addr, sizeof(addr));
    if (ret != 0) {
        if (errno != EINPROGRESS) {
            close(fd);
            return -1;
        }
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLOUT;
        if (poll(&pfd, 1, (int)timeout_ms) != 1) {
            close(fd);
            return -1;
        }
        int err = 0;
        socklen_t errlen = sizeof(err);
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &errlen) != 0 || err != 0) {
            close(fd);
            return -1;
        }
    }

    // back to blocking; all reads below use poll for timeouts
    fcntl(fd, F_SETFL, flags);
    return fd;
}

static bool send_all(int fd, const char *buf, size_t len)
{
    while (len > 0) {
        ssize_t n = send(fd, buf, len, MSG_NOSIGNAL);
        if (n <= 0) {
            if (n < 0 && (errno == EINTR)) {
                continue;
            }
            return false;
        }
        buf += n;
        len -= (size_t)n;
    }
    return true;
}

// recv with a poll timeout; <=0 on timeout/error/EOF
static ssize_t recv_timeout(int fd, char *buf, size_t len, uint32_t timeout_ms)
{
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, (int)timeout_ms) != 1) {
        return -1;
    }
    return recv(fd, buf, len, 0);
}

FACommunicator::FACommunicator(const char *ip) :
    _controller_port(18083),
    _controller_started(false),
    _socknext(-1),
    _stop(false),
    _connect_fail_count(0),
    _last_connect_fail_log(0)
{
    snprintf(_controller_ip, sizeof(_controller_ip), "%s", ip ? ip : "127.0.0.1");
    _replybuf[0] = 0;
    _creator_thread = std::thread(&FACommunicator::socketCreator, this);
}

FACommunicator::~FACommunicator()
{
    {
        std::lock_guard<std::mutex> lk(_sockmtx);
        _stop = true;
    }
    _sockcond1.notify_all();
    _sockcond2.notify_all();
    if (_creator_thread.joinable()) {
        _creator_thread.join();
    }
    if (_socknext >= 0) {
        close(_socknext);
        _socknext = -1;
    }
}

/*
  socket-creator thread. Always keeps one connected socket parked so a
  request never pays the connect latency (ArduPilot's socket_creator()).
 */
void FACommunicator::socketCreator()
{
    while (true) {
        {
            std::unique_lock<std::mutex> lk(_sockmtx);
            _sockcond2.wait(lk, [this] { return _socknext < 0 || _stop; });
            if (_stop) {
                return;
            }
        }
        /*
          don't let the connection take more than 100ms (10Hz). Longer
          than this and we are better off trying for a new socket
         */
        int fd = connect_timeout(_controller_ip, _controller_port, 100);
        if (fd < 0) {
            /*
              This retries every 5 ms, so logging every failure produced ~200
              lines/s of identical text while RealFlight was unreachable. Keep
              the message - it is the main diagnostic when the Windows box is
              misconfigured - but log the first one, then one summary every 5 s
              with the suppressed count.
             */
            const time_t now = time(nullptr);
            _connect_fail_count++;

            if (_connect_fail_count == 1) {
                printf("[flightaxis_bridge] connect to %s:%u failed - is RealFlight running "
                       "with FlightAxis Link enabled?\n",
                       _controller_ip, (unsigned)_controller_port);
                _last_connect_fail_log = now;

            } else if (now - _last_connect_fail_log >= 5) {
                printf("[flightaxis_bridge] connect to %s:%u still failing (%u attempts)\n",
                       _controller_ip, (unsigned)_controller_port,
                       (unsigned)_connect_fail_count);
                _last_connect_fail_log = now;
            }

            {
                std::lock_guard<std::mutex> lk(_sockmtx);
                if (_stop) {
                    return;
                }
            }
            usleep(5000);
            continue;
        }
        if (_connect_fail_count > 0) {
            printf("[flightaxis_bridge] connect to %s:%u recovered after %u failed attempts\n",
                   _controller_ip, (unsigned)_controller_port, (unsigned)_connect_fail_count);
            _connect_fail_count = 0;
        }

        {
            std::lock_guard<std::mutex> lk(_sockmtx);
            if (_stop) {
                close(fd);
                return;
            }
            _socknext = fd;
        }
        _sockcond1.notify_all();
    }
}

/*
  take the parked socket, waiting up to timeout_ms for the creator
  thread to produce one
 */
int FACommunicator::takeSocket(uint32_t timeout_ms)
{
    std::unique_lock<std::mutex> lk(_sockmtx);
    if (!_sockcond1.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                             [this] { return _socknext >= 0 || _stop; })) {
        return -1;
    }
    if (_stop || _socknext < 0) {
        return -1;
    }
    int fd = _socknext;
    _socknext = -1;
    lk.unlock();
    _sockcond2.notify_all();
    return fd;
}

/*
  send one SOAP request on a fresh parked socket. HTTP framing copied
  from ArduPilot's soap_request_start().
 */
int FACommunicator::soapRequestStart(const char *action, const char *body)
{
    int fd = takeSocket(3000);
    if (fd < 0) {
        printf("[flightaxis_bridge] no connection to RealFlight at %s:%u\n",
               _controller_ip, (unsigned)_controller_port);
        return -1;
    }

    char req[4096];
    int len = snprintf(req, sizeof(req),
                       "POST / HTTP/1.1\n"
                       "soapaction: '%s'\n"
                       "content-length: %u\n"
                       "content-type: text/xml;charset='UTF-8'\n"
                       "Connection: Keep-Alive\n"
                       "\n"
                       "%s",
                       action, (unsigned)strlen(body), body);
    if (len <= 0 || len >= (int)sizeof(req)) {
        printf("[flightaxis_bridge] SOAP request too large\n");
        close(fd);
        return -1;
    }
    if (!send_all(fd, req, (size_t)len)) {
        printf("[flightaxis_bridge] send failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

/*
  read the reply: find Content-Length, drain to end of \r\n\r\n + length,
  close the socket (ArduPilot's soap_request_end())
 */
char *FACommunicator::soapRequestEnd(int fd, uint32_t timeout_ms)
{
    ssize_t ret = recv_timeout(fd, _replybuf, sizeof(_replybuf) - 1, timeout_ms);
    if (ret <= 0) {
        close(fd);
        return nullptr;
    }
    _replybuf[ret] = 0;

    char *p = strstr(_replybuf, "Content-Length: ");
    if (p == nullptr) {
        printf("[flightaxis_bridge] no Content-Length in reply\n");
        close(fd);
        return nullptr;
    }

    // get the content length
    uint32_t content_length = strtoul(p + 16, nullptr, 10);
    char *body = strstr(p, "\r\n\r\n");
    if (body == nullptr) {
        printf("[flightaxis_bridge] no body in reply\n");
        close(fd);
        return nullptr;
    }
    body += 4;

    // get the rest of the body
    int32_t expected_length = (int32_t)(content_length + (body - _replybuf));
    if (expected_length >= (int32_t)sizeof(_replybuf)) {
        printf("[flightaxis_bridge] reply too large %d\n", (int)expected_length);
        close(fd);
        return nullptr;
    }
    while (ret < expected_length) {
        ssize_t ret2 = recv_timeout(fd, &_replybuf[ret], sizeof(_replybuf) - (size_t)(1 + ret), 1000);
        if (ret2 <= 0) {
            close(fd);
            return nullptr;
        }
        // nul terminate
        _replybuf[ret + ret2] = 0;
        ret += ret2;
    }
    close(fd);
    return _replybuf;
}

/*
  extremely primitive SOAP parser that assumes the format used by
  FlightAxis. Sequential scan in document order; repeated "item" keys
  decode as an array (ArduPilot's parse_reply()).
 */
bool FACommunicator::parseReply(const char *reply, double *vals)
{
    const char *reply0 = reply;
    for (unsigned i = 0; i < NUM_KEYS; i++) {
        const char *p = strstr(reply, key_names[i]);
        if (p == nullptr) {
            p = strstr(reply0, key_names[i]);
        }
        if (p == nullptr) {
            printf("[flightaxis_bridge] failed to find key %s\n", key_names[i]);
            // schema change / wrong reply -> force re-init, caller self-heals
            _controller_started = false;
            return false;
        }
        p += strlen(key_names[i]) + 1;
        double v;
        if (strncmp(p, "true", 4) == 0) {
            v = 1;
        } else if (strncmp(p, "false", 5) == 0) {
            v = 0;
        } else {
            v = atof(p);
        }
        vals[i] = v;
        // this assumes key order and allows us to decode arrays
        p = strchr(p, '>');
        if (p != nullptr) {
            reply = p;
        }
    }
    return true;
}

void FACommunicator::fillState(const double *vals, FAState &state)
{
    for (int i = 0; i < 12; i++) {
        state.rcin[i] = vals[KEY_ITEM0 + i];
    }
    state.m_currentPhysicsTime_SEC          = vals[KEY_PHYSICS_TIME];
    state.m_currentPhysicsSpeedMultiplier   = vals[KEY_PHYSICS_SPEED];
    state.m_airspeed_MPS                    = vals[KEY_AIRSPEED];
    state.m_altitudeASL_MTR                 = vals[KEY_ALT_ASL];
    state.m_altitudeAGL_MTR                 = vals[KEY_ALT_AGL];
    state.m_groundspeed_MPS                 = vals[KEY_GROUNDSPEED];
    state.m_accelerationWorldAX_MPS2        = vals[KEY_ACC_WORLD_AX];
    state.m_accelerationWorldAY_MPS2        = vals[KEY_ACC_WORLD_AY];
    state.m_accelerationWorldAZ_MPS2        = vals[KEY_ACC_WORLD_AZ];
    state.m_accelerationBodyAX_MPS2         = vals[KEY_ACC_BODY_AX];
    state.m_accelerationBodyAY_MPS2         = vals[KEY_ACC_BODY_AY];
    state.m_accelerationBodyAZ_MPS2         = vals[KEY_ACC_BODY_AZ];
    state.m_windX_MPS                       = vals[KEY_WIND_X];
    state.m_windY_MPS                       = vals[KEY_WIND_Y];
    state.m_windZ_MPS                       = vals[KEY_WIND_Z];
    state.m_propRPM                         = vals[KEY_PROP_RPM];
    state.m_heliMainRotorRPM                = vals[KEY_HELI_RPM];
    state.m_batteryVoltage_VOLTS            = vals[KEY_BATT_VOLTS];
    state.m_batteryCurrentDraw_AMPS         = vals[KEY_BATT_AMPS];
    state.m_fuelRemaining_OZ                = vals[KEY_FUEL_OZ];
    state.m_isLocked                        = vals[KEY_IS_LOCKED] != 0;
    state.m_hasLostComponents               = vals[KEY_LOST_COMPONENTS] != 0;
    state.m_anEngineIsRunning               = vals[KEY_ENGINE_RUNNING] != 0;
    state.m_isTouchingGround                = vals[KEY_TOUCHING_GROUND] != 0;
    state.m_flightAxisControllerIsActive    = vals[KEY_CONTROLLER_ACTIVE] != 0;
    state.m_resetButtonHasBeenPressed       = vals[KEY_RESET_PRESSED] != 0;
    state.m_roll_DEG                        = vals[KEY_ROLL];
    state.m_inclination_DEG                 = vals[KEY_INCLINATION];
    state.m_azimuth_DEG                     = vals[KEY_AZIMUTH];
    state.m_orientationQuaternionX          = vals[KEY_QUAT_X];
    state.m_orientationQuaternionY          = vals[KEY_QUAT_Y];
    state.m_orientationQuaternionZ          = vals[KEY_QUAT_Z];
    state.m_orientationQuaternionW          = vals[KEY_QUAT_W];
    state.m_rollRate_DEGpSEC                = vals[KEY_ROLL_RATE];
    state.m_pitchRate_DEGpSEC               = vals[KEY_PITCH_RATE];
    state.m_yawRate_DEGpSEC                 = vals[KEY_YAW_RATE];
    state.m_aircraftPositionX_MTR           = vals[KEY_POS_X];
    state.m_aircraftPositionY_MTR           = vals[KEY_POS_Y];
    state.m_velocityWorldU_MPS              = vals[KEY_VEL_WORLD_U];
    state.m_velocityWorldV_MPS              = vals[KEY_VEL_WORLD_V];
    state.m_velocityWorldW_MPS              = vals[KEY_VEL_WORLD_W];
    state.m_velocityBodyU_MPS               = vals[KEY_VEL_BODY_U];
    state.m_velocityBodyV_MPS               = vals[KEY_VEL_BODY_V];
    state.m_velocityBodyW_MPS               = vals[KEY_VEL_BODY_W];
}

/*
  startup sequence, exact order: RestoreOriginalControllerDevice ->
  (ResetAircraft if resetPosition) -> InjectUAVControllerInterface.

  The Restore call allows us to connect after the aircraft is changed
  in RealFlight. As in ArduPilot, the SOAP replies are not checked:
  "already injected" style faults are treated as success.
 */
/*
  Diagnostic only. The startup sequence deliberately does NOT check SOAP faults
  (an "already injected" reply IS a fault and means success), so the reply is
  still ignored for control flow - but a MISSING reply is a different animal:
  it means the request never reached RealFlight, or something in between (a
  proxy/NAT/firewall with an idle timeout) dropped the pre-parked socket. That
  used to break startup with zero output. Say something.
 */
void FACommunicator::reportStartupReply(const char *action, const char *reply)
{
    if (reply == nullptr) {
        printf("[flightaxis_bridge] WARNING: no reply to %s from %s:%u - the request may not have "
               "reached RealFlight (dropped/idle-timed-out connection?); startup may be incomplete\n",
               action, _controller_ip, (unsigned)_controller_port);
        return;
    }

    // a valid SOAP reply is always an Envelope, fault or not
    if (strstr(reply, "Envelope") == nullptr) {
        printf("[flightaxis_bridge] WARNING: short/unrecognised reply to %s from %s:%u "
               "(%u bytes); startup may be incomplete\n",
               action, _controller_ip, (unsigned)_controller_port, (unsigned)strlen(reply));
    }
}

bool FACommunicator::startController(bool resetPosition)
{
    printf("[flightaxis_bridge] starting controller at %s\n", _controller_ip);

    int fd = soapRequestStart("RestoreOriginalControllerDevice", soap_body_restore);
    if (fd < 0) {
        return false;
    }
    reportStartupReply("RestoreOriginalControllerDevice", soapRequestEnd(fd, 1000));

    if (resetPosition) {
        fd = soapRequestStart("ResetAircraft", soap_body_reset);
        if (fd < 0) {
            return false;
        }
        reportStartupReply("ResetAircraft", soapRequestEnd(fd, 1000));
    }

    fd = soapRequestStart("InjectUAVControllerInterface", soap_body_inject);
    if (fd < 0) {
        return false;
    }
    reportStartupReply("InjectUAVControllerInterface", soapRequestEnd(fd, 1000));

    _controller_started = true;
    return true;
}

bool FACommunicator::controllerStarted() const
{
    return _controller_started;
}

void FACommunicator::markNeedsRestart()
{
    _controller_started = false;
}

/*
  one ExchangeData round-trip
 */
bool FACommunicator::exchangeData(const double *channels, int nch, uint32_t selectedChannels, FAState &state)
{
    // maximum number of servos to send is 12 with new FlightAxis
    double ch[12] = {};
    if (nch > 12) {
        nch = 12;
    }
    for (int i = 0; i < nch; i++) {
        ch[i] = channels[i];
    }

    char body[4096];
    int blen = snprintf(body, sizeof(body), soap_body_exchange_fmt,
                        (unsigned)selectedChannels,
                        ch[0], ch[1], ch[2], ch[3], ch[4], ch[5],
                        ch[6], ch[7], ch[8], ch[9], ch[10], ch[11]);
    if (blen <= 0 || blen >= (int)sizeof(body)) {
        printf("[flightaxis_bridge] ExchangeData body too large\n");
        return false;
    }

    int fd = soapRequestStart("ExchangeData", body);
    if (fd < 0) {
        return false;
    }

    char *reply = soapRequestEnd(fd, 1000);
    if (reply == nullptr) {
        return false;
    }

    double vals[NUM_KEYS] = {};
    if (!parseReply(reply, vals)) {
        // parseReply already flagged the restart
        return false;
    }

    fillState(vals, state);
    return true;
}
