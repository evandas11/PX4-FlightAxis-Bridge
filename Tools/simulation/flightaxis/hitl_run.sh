#!/usr/bin/env bash
#
# This file is part of the PX4-FlightAxis-Bridge project.
# Copyright (c) 2026 the PX4-FlightAxis-Bridge contributors.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# ---------------------------------------------------------------------------
#
# Launch the FlightAxis bridge against a REAL flight controller board (HITL).
#
# Mirrors sitl_run.sh's structure and its FA_check gate, but there is no local
# px4 binary to run: the firmware is on the board. This script therefore
# launches the bridge in the foreground and nothing else.
#
# usage: hitl_run.sh model src_path build_path [serial_device]
#
# See HITL.md for board setup - in particular SYS_HITL=1, the
# CONFIG_MODULES_SIMULATION_PWM_OUT_SIM firmware requirement, and the
# MAV_x_RATE > 5000 trap.

set -e

if [ "$#" -lt 3 ]; then
	echo "usage: hitl_run.sh model src_path build_path [serial_device]"
	echo ""
	echo "  model         one of the models/*.json names (plane, quad, quadplane, heli)"
	echo "  src_path      PX4-Autopilot source root"
	echo "  build_path    build directory containing build_flightaxis_bridge/"
	echo "  serial_device board serial port, default \$PX4_HITL_SERIAL_DEV or /dev/ttyACM0"
	exit 1
fi

model="$1"
src_path="$2"
build_path="$3"

# ---------------------------------------------------------------------------
# Transport. USB CDC-ACM is the default and strongly preferred: PX4 forces its
# datarate to 100000 B/s on /dev/ttyACM* (mavlink_main.cpp:2166-2181), whereas
# a hardware UART inherits MAV_x_RATE, which defaults to 1200 B/s - below the
# 5000 B/s threshold that set_hil_enabled() requires (mavlink_main.cpp:671).
# Below that threshold PX4 never enables the HIL streams and never says so.
# ---------------------------------------------------------------------------
TRANSPORT="${PX4_HITL_TRANSPORT:-serial}"
SERIAL_DEV="${4:-${PX4_HITL_SERIAL_DEV:-/dev/ttyACM0}}"
SERIAL_BAUD="${PX4_HITL_SERIAL_BAUD:-921600}"
SENSOR_HZ="${PX4_HITL_SENSOR_HZ:-250}"

# RealFlight host: env override, default localhost (RealFlight in a VM / same box)
FA_IP="${PX4_FLIGHTAXIS_IP:-127.0.0.1}"

cat <<'EOF'

  ============================================================
   HARDWARE IN THE LOOP - THIS DRIVES A REAL FLIGHT CONTROLLER
  ============================================================

   REMOVE ALL PROPELLERS BEFORE CONTINUING.

   With SYS_HITL=1 PX4 does not start pwm_out or dshot at all
   (ROMFS/px4fmu_common/init.d/rcS:451-464) and commander holds
   actuator lockdown permanently (Commander.cpp:1892), so the
   FMU servo rail should stay silent even when armed.

   That is a software interlock, not a physical one. It does not
   protect you from a wrong SYS_HITL value, a stale parameter, an
   airframe that was not rebooted, or a separately powered ESC.
   Props off is the only thing that actually protects you.

EOF

echo "HITL ARGS"
echo "  model:      $model"
echo "  src_path:   $src_path"
echo "  build_path: $build_path"
echo "  transport:  $TRANSPORT"

if [ "$TRANSPORT" = "serial" ]; then
	echo "  device:     $SERIAL_DEV @ $SERIAL_BAUD baud"
elif [ "$TRANSPORT" = "udp" ]; then
	echo "  udp:        ${PX4_HITL_UDP_HOST:-<unset>}:${PX4_HITL_UDP_PORT:-14550}"
fi

echo "  HIL_SENSOR: $SENSOR_HZ Hz"
echo ""

# ---------------------------------------------------------------------------
# Preflight checks
# ---------------------------------------------------------------------------
if [ "$TRANSPORT" = "serial" ]; then
	if [ ! -e "$SERIAL_DEV" ]; then
		echo "ERROR: serial device $SERIAL_DEV does not exist." >&2
		echo "  - Is the board plugged in and booted?" >&2
		echo "  - Try: ls /dev/ttyACM* /dev/ttyUSB*" >&2
		exit 1
	fi

	if [ ! -r "$SERIAL_DEV" ] || [ ! -w "$SERIAL_DEV" ]; then
		echo "ERROR: no read/write permission on $SERIAL_DEV." >&2
		echo "  - Add yourself to the 'dialout' group:" >&2
		echo "      sudo usermod -aG dialout \$USER   (then log out and back in)" >&2
		exit 1
	fi

	case "$SERIAL_DEV" in
	/dev/ttyACM*) ;;
	*)
		echo "NOTE: $SERIAL_DEV is not a USB CDC-ACM port."
		echo "  On a hardware UART you must set, on the board:"
		echo "    MAV_x_CONFIG   = the TELEM port you wired"
		echo "    SER_TELx_BAUD  = $SERIAL_BAUD"
		echo "    MAV_x_RATE     > 5000   (default is 1200 - HIL will NOT enable)"
		echo ""
		;;
	esac

	# A GCS or another MAVLink client holding the port is a common and
	# confusing failure: the bridge opens it fine and then sees nothing.
	if command -v fuser >/dev/null 2>&1; then
		if fuser "$SERIAL_DEV" >/dev/null 2>&1; then
			echo "ERROR: $SERIAL_DEV is already open by another process:" >&2
			fuser -v "$SERIAL_DEV" >&2 || true
			echo "  Close QGroundControl / MAVProxy / screen first - the bridge needs" >&2
			echo "  exclusive use of the board's MAVLink link." >&2
			exit 1
		fi
	fi
elif [ "$TRANSPORT" = "udp" ]; then
	if [ -z "$PX4_HITL_UDP_HOST" ]; then
		echo "ERROR: PX4_HITL_TRANSPORT=udp requires PX4_HITL_UDP_HOST (the board's IP)." >&2
		exit 1
	fi
else
	echo "ERROR: unsupported PX4_HITL_TRANSPORT '$TRANSPORT' (expected serial or udp)." >&2
	exit 1
fi

echo "FlightAxis setup"
cd "${src_path}/Tools/simulation/flightaxis/flightaxis_bridge/"
# Invoked via python3 rather than ./FA_check.py, matching sitl_run.sh: relying on
# the exec bit and the shebang means a zip round-trip, a noexec mount or a
# filesystem without permission bits produces "Permission denied" / "bad
# interpreter", and the || branch would then misreport a working network as
# unreachable.
python3 ./FA_check.py "${FA_IP}" || { echo "RealFlight FlightAxis not reachable at ${FA_IP}:18083"; exit 1; }

if ! fa_bridge_params=`python3 ./get_FAbridge_params.py "models/${model}.json"`; then
	echo "get_FAbridge_params.py failed for models/${model}.json" >&2
	exit 1
fi

if [ -z "$fa_bridge_params" ]; then
	echo "get_FAbridge_params.py produced no parameters for models/${model}.json" >&2
	exit 1
fi

bridge_bin="${build_path}/build_flightaxis_bridge/flightaxis_bridge"

if [ ! -x "$bridge_bin" ]; then
	echo "ERROR: bridge binary not found at $bridge_bin" >&2
	echo "  Build it with:  ninja -C $build_path flightaxis_bridge" >&2
	exit 1
fi

# The bridge runs in the FOREGROUND: unlike SITL there is no px4 process to
# wait on, so the bridge is the thing the user Ctrl-C's.
export PX4_HITL_TRANSPORT="$TRANSPORT"
export PX4_HITL_SERIAL_DEV="$SERIAL_DEV"
export PX4_HITL_SERIAL_BAUD="$SERIAL_BAUD"
export PX4_HITL_SENSOR_HZ="$SENSOR_HZ"
# Exported with the default already applied so the bridge uses exactly the port
# printed in the banner above, rather than re-deriving its own default.
if [ "$TRANSPORT" = "udp" ]; then
	export PX4_HITL_UDP_HOST
	export PX4_HITL_UDP_PORT="${PX4_HITL_UDP_PORT:-14550}"
fi

echo "starting bridge (Ctrl-C to stop)"
exec "$bridge_bin" 0 "${FA_IP}" $fa_bridge_params
