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
 * Pattern: one new TCP connection per SOAP request, with connect latency
 * hidden by a background socket-creator thread that always keeps one
 * connected socket parked (100 ms connect timeout, condition-variable
 * handoff).
 *
 ****************************************************************************/

#pragma once
#include <cstdint>
#include <ctime>

#include <condition_variable>
#include <mutex>
#include <thread>

struct FAState {
    double rcin[12];                       // the 12 "item" echoes (physical TX channels, 0..1)
    double m_currentPhysicsTime_SEC;
    double m_currentPhysicsSpeedMultiplier;
    double m_airspeed_MPS;
    double m_altitudeASL_MTR;
    double m_altitudeAGL_MTR;
    double m_groundspeed_MPS;
    double m_accelerationWorldAX_MPS2, m_accelerationWorldAY_MPS2, m_accelerationWorldAZ_MPS2;
    double m_accelerationBodyAX_MPS2,  m_accelerationBodyAY_MPS2,  m_accelerationBodyAZ_MPS2;
    double m_windX_MPS, m_windY_MPS, m_windZ_MPS;
    double m_propRPM, m_heliMainRotorRPM;
    double m_batteryVoltage_VOLTS, m_batteryCurrentDraw_AMPS, m_fuelRemaining_OZ;
    bool   m_isLocked, m_hasLostComponents, m_anEngineIsRunning, m_isTouchingGround;
    bool   m_flightAxisControllerIsActive, m_resetButtonHasBeenPressed;
    double m_roll_DEG, m_inclination_DEG, m_azimuth_DEG;
    double m_orientationQuaternionX, m_orientationQuaternionY, m_orientationQuaternionZ, m_orientationQuaternionW;
    double m_rollRate_DEGpSEC, m_pitchRate_DEGpSEC, m_yawRate_DEGpSEC;
    double m_aircraftPositionX_MTR, m_aircraftPositionY_MTR;
    double m_velocityWorldU_MPS, m_velocityWorldV_MPS, m_velocityWorldW_MPS;
    double m_velocityBodyU_MPS, m_velocityBodyV_MPS, m_velocityBodyW_MPS;
};

class FACommunicator {
public:
    explicit FACommunicator(const char *ip);   // RealFlight host, port always 18083
    ~FACommunicator();
    // One ExchangeData round-trip. channels: 0..1 values, nch<=12. selectedChannels goes
    // into <m-selectedChannels>. Returns true and fills state on success; false on
    // connect/send/parse failure (caller decides what to do).
    bool exchangeData(const double *channels, int nch, uint32_t selectedChannels, FAState &state);
    // Startup sequence, exact order: RestoreOriginalControllerDevice -> (ResetAircraft if
    // resetPosition) -> InjectUAVControllerInterface. Returns true on success.
    bool startController(bool resetPosition);

    /*
     * Send ResetAircraft on its own, without the surrounding startup sequence.
     *
     * Used to re-place the model after a respawn has already been detected. The
     * pilot's own reset happened while the throttle was still up, so RealFlight
     * applied that thrust to the freshly placed aircraft and it rolled away
     * before the bridge could learn anything had happened. Cutting the throttle
     * first and then resetting again puts it back with the prop already
     * stopped, which is the state a reset is supposed to produce.
     */
    bool resetAircraft();
    // Shutdown counterpart of startController(): deselect our channels and send
    // RestoreOriginalControllerDevice, so RealFlight hands the model back to the
    // physical transmitter instead of leaving our injected controller attached
    // with nobody driving it.
    //
    // Idempotent (a second call is a no-op) and time-bounded: every step uses a
    // short timeout and failures are ignored, because a bridge that refuses to
    // exit is worse than one that leaves RealFlight injected. Main thread only.
    void releaseController();
    bool controllerStarted() const;
    void markNeedsRestart();                   // next exchange/start must re-run startup

private:
    // number of keys in the ExchangeData reply key table (12 "item" echoes + m-* fields)
    static const unsigned NUM_KEYS = 58;

    // socket-creator thread body: always keeps one connected socket parked
    void socketCreator();
    // take the parked socket (blocks up to timeout_ms for the creator thread); -1 on timeout
    int takeSocket(uint32_t timeout_ms);
    // send one SOAP request on a fresh socket; returns the socket fd or -1 on failure.
    // socket_timeout_ms bounds the wait for the creator thread's parked socket;
    // quiet suppresses the "no connection to RealFlight" line (used on the
    // shutdown path, where an absent RealFlight is expected and not an error).
    int soapRequestStart(const char *action, const char *body,
                         uint32_t socket_timeout_ms = 3000, bool quiet = false);
    // read the full HTTP reply (headers + Content-Length body) and close the socket.
    // Returns pointer to _replybuf (NUL terminated, full reply) or nullptr on failure.
    char *soapRequestEnd(int fd, uint32_t timeout_ms);
    // parse an ExchangeData reply into raw key values; false if any key is missing
    bool parseReply(const char *reply, double *vals);
    // sequential-scan value table -> public FAState
    static void fillState(const double *vals, FAState &state);
    // log (only) when a startup request got no / an unrecognisable reply; SOAP
    // faults stay deliberately ignored
    void reportStartupReply(const char *action, const char *reply);

    char _controller_ip[64];
    uint16_t _controller_port;

    bool _controller_started;
    // _controller_started is cleared by markNeedsRestart() on every failed
    // exchange, so it cannot answer "does RealFlight still have our controller?".
    // _ever_injected latches the moment an InjectUAVControllerInterface leaves
    // the bridge and is what the shutdown release keys off.
    bool _ever_injected;
    bool _released;

    // parked-socket handoff (sockcond1 / sockcond2 / sockmtx below)
    std::thread _creator_thread;
    std::mutex _sockmtx;
    std::condition_variable _sockcond1;        // signalled when a socket is parked
    std::condition_variable _sockcond2;        // signalled when the parked socket is taken
    int _socknext;                             // parked connected socket, -1 if none
    bool _stop;

    // connect-failure log dedup (creator thread only; retries every 5 ms)
    uint32_t _connect_fail_count;
    time_t _last_connect_fail_log;

    char _replybuf[10000];
};
