#!/usr/bin/env python3
"""Sanity check that RealFlight's FlightAxis Link is reachable.

RealFlight runs on a Windows machine (or VM); this only verifies that
something is listening on <ip>:18083 before the bridge is started.

Usage: FA_check.py <ip>
Exit code: 0 = reachable, 1 = not reachable / bad usage.
"""

import socket
import sys

FLIGHTAXIS_PORT = 18083
TIMEOUT_S = 2.0


def main():
    if len(sys.argv) != 2:
        print("usage: FA_check.py <realflight_ip>")
        return 1

    ip = sys.argv[1]

    try:
        with socket.create_connection((ip, FLIGHTAXIS_PORT), timeout=TIMEOUT_S):
            pass
    except OSError as e:
        print("FlightAxis check: cannot reach {}:{} ({})".format(ip, FLIGHTAXIS_PORT, e))
        print("  - Is RealFlight running with the FlightAxis (RealFlight Link) option enabled?")
        print("    (RealFlight Evolution: press ESC, Settings -> Physics -> Quality,")
        print("     enable 'RealFlight Link'; on RealFlight 8/9 it is Settings -> Physics)")
        print("  - Is the host reachable / firewall open on TCP {}?".format(FLIGHTAXIS_PORT))
        print("  - Set PX4_FLIGHTAXIS_IP if RealFlight is not on {}.".format(ip))
        return 1

    print("FlightAxis check: RealFlight reachable at {}:{}".format(ip, FLIGHTAXIS_PORT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
