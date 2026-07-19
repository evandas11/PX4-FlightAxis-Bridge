#!/usr/bin/env bash

set -e

if [ "$#" -lt 4 ]; then
	echo "usage: sitl_run.sh sitl_bin model src_path build_path"
	exit 1
fi

if [[ -n "$DONT_RUN" ]]; then
	echo "Not running simulation (DONT_RUN is set)."
	exit 0
fi

sitl_bin="$1"
model="$2"
src_path="$3"
build_path="$4"

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo model: $model
echo src_path: $src_path
echo build_path: $build_path

# PX4 instance id. Everything that has to be unique per instance derives from
# it: the bridge's TCP listen port (4560+instance, PX4Communicator::InitTcpServer),
# PX4's own simulator port (px4-rc.mavlinksim: simulator_tcp_port=$((4560+px4_instance))),
# every MAVLink UDP port (px4-rc.mavlink), MAV_SYS_ID (rcS: px4_instance+1) and
# the working directory.
#
# This used to be a hard-coded 0 here, which meant no make target could start a
# second instance even though the bridge had always supported it -- a second
# flightaxis_* target just collided on TCP 4560. (PX4's own
# Tools/simulation/flightgear/sitl_run.sh still hard-codes 0 the same way, so
# "it follows the FlightGear convention" was true and useless.)
instance="${PX4_FLIGHTAXIS_INSTANCE:-0}"

# Validate: a non-numeric value would otherwise reach `px4 -i` and the port
# arithmetic as a silent 0, producing a second instance that collides with the
# first instead of failing.
case "$instance" in
	''|*[!0-9]*)
		echo "PX4_FLIGHTAXIS_INSTANCE must be a non-negative integer, got '$instance'" >&2
		exit 1
		;;
esac

# Working directory. Instance 0 keeps the historical "$build_path/rootfs" path
# so existing saved parameters and logs stay where they were; additional
# instances follow PX4's own multi-instance convention from
# Tools/simulation/sitl_multiple_run.sh, which uses "$build_path/instance_$n".
if [ "$instance" -eq 0 ]; then
	rootfs="$build_path/rootfs" # this is the working directory
else
	rootfs="$build_path/instance_$instance"
fi

mkdir -p "$rootfs"

# To disable user input: without this PX4 opens the interactive pxh> shell, so
# the target cannot be driven from CI or a non-tty
if [[ -n "$NO_PXH" ]]; then
	no_pxh=-d
else
	no_pxh=""
fi

export PX4_SIM_MODEL=flightaxis_${model}

# RealFlight host: env override, default localhost (RealFlight in a VM / same box)
FA_IP="${PX4_FLIGHTAXIS_IP:-127.0.0.1}"

echo "FlightAxis setup"
cd "${src_path}/Tools/simulation/flightaxis/flightaxis_bridge/"
# Invoked via python3 rather than ./FA_check.py: relying on the exec bit and the
# shebang means a zip round-trip, a noexec mount or a filesystem without
# permission bits produces "Permission denied" / "bad interpreter", and the ||
# branch would then misreport a working network as unreachable.
python3 ./FA_check.py "${FA_IP}" || { echo "RealFlight FlightAxis not reachable at ${FA_IP}:18083"; exit 1; }

# Capture the bridge argv first: the bridge is backgrounded below, so a failure
# here would otherwise be invisible (empty argv -> bridge prints usage and dies,
# and PX4 then blocks forever on TCP 4560 with no diagnostic).
if ! fa_bridge_params=`python3 ./get_FAbridge_params.py "models/${model}.json"`; then
	echo "get_FAbridge_params.py failed for models/${model}.json" >&2
	exit 1
fi

if [ -z "$fa_bridge_params" ]; then
	echo "get_FAbridge_params.py produced no parameters for models/${model}.json" >&2
	exit 1
fi

bridge_bin="${build_path}/build_flightaxis_bridge/flightaxis_bridge"

# Checked before backgrounding, for the same reason the argv is captured above:
# a missing binary would otherwise produce one "No such file" line from the
# subshell and then PX4 would block forever on TCP 4560, which reads as a
# simulator hang rather than a missing build target.
if [ ! -x "$bridge_bin" ]; then
	echo "ERROR: bridge binary not found at $bridge_bin" >&2
	echo "  Build it with:  ninja -C $build_path flightaxis_bridge" >&2
	exit 1
fi

# A PX4_HITL_TRANSPORT left exported by a previous hitl_run.sh session would put
# the bridge on a serial port / UDP socket while PX4 SITL waits on TCP 4560 -
# again an unexplained hang. SITL is always the bridge's tcp-server default.
export PX4_HITL_TRANSPORT=tcp-server

"$bridge_bin" "$instance" "${FA_IP}" \
	$fa_bridge_params &
FA_BRIDGE_PID=$!

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

# -i is what makes px4_instance non-zero inside rcS (px4-alias.sh_in takes it as
# $1), and that is what offsets the simulator TCP port and every MAVLink UDP
# port. Without it PX4 would boot as instance 0 and listen on 4560 no matter
# what the bridge was told.
sitl_command="\"$sitl_bin\" -i $instance $no_pxh \"$build_path\"/etc"

echo SITL COMMAND: $sitl_command

eval $sitl_command
# Capture PX4's status now: the script's last command would otherwise be the
# kill below, so a PX4 segfault / param import failure / the unknown-model abort
# in rcS would report success, and a clean Ctrl-C where the bridge had already
# exited would report Error 1 - timing dependent, so it looks intermittent.
px4_status=$?

popd >/dev/null

kill $FA_BRIDGE_PID 2>/dev/null
wait $FA_BRIDGE_PID 2>/dev/null

exit $px4_status
