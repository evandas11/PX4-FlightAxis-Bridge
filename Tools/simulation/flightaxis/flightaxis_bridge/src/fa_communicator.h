/****************************************************************************
 *
 * FlightAxis (RealFlight) SOAP communicator for the PX4 flightaxis bridge.
 *
 * Socket / SOAP / parser / reconnect logic ported from ArduPilot
 * libraries/SITL/SIM_FlightAxis.{h,cpp} (GPLv3, (C) ArduPilot dev team).
 *
 * Pattern: one new TCP connection per SOAP request, with connect latency
 * hidden by a background socket-creator thread that always keeps one
 * connected socket parked (100 ms connect timeout, condition-variable
 * handoff).
 *
 ****************************************************************************/

#pragma once
#include <cstdint>

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
    bool controllerStarted() const;
    void markNeedsRestart();                   // next exchange/start must re-run startup

private:
    // number of keys in the ExchangeData reply key table (12 "item" echoes + m-* fields)
    static const unsigned NUM_KEYS = 58;

    // socket-creator thread body: always keeps one connected socket parked
    void socketCreator();
    // take the parked socket (blocks up to timeout_ms for the creator thread); -1 on timeout
    int takeSocket(uint32_t timeout_ms);
    // send one SOAP request on a fresh socket; returns the socket fd or -1 on failure
    int soapRequestStart(const char *action, const char *body);
    // read the full HTTP reply (headers + Content-Length body) and close the socket.
    // Returns pointer to _replybuf (NUL terminated, full reply) or nullptr on failure.
    char *soapRequestEnd(int fd, uint32_t timeout_ms);
    // parse an ExchangeData reply into raw key values; false if any key is missing
    bool parseReply(const char *reply, double *vals);
    // sequential-scan value table -> public FAState
    static void fillState(const double *vals, FAState &state);

    char _controller_ip[64];
    uint16_t _controller_port;

    bool _controller_started;

    // parked-socket handoff (ArduPilot's sockcond1/sockcond2/sockmtx pattern)
    std::thread _creator_thread;
    std::mutex _sockmtx;
    std::condition_variable _sockcond1;        // signalled when a socket is parked
    std::condition_variable _sockcond2;        // signalled when the parked socket is taken
    int _socknext;                             // parked connected socket, -1 if none
    bool _stop;

    char _replybuf[10000];
};
