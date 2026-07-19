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

rootfs="$build_path/rootfs" # this is the working directory
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

"$bridge_bin" 0 "${FA_IP}" \
	$fa_bridge_params &
FA_BRIDGE_PID=$!

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

sitl_command="\"$sitl_bin\" $no_pxh \"$build_path\"/etc"

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
