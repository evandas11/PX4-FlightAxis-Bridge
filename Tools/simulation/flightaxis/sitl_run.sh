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
#
# PX4_FLIGHTAXIS_ROOTFS overrides both, because every model otherwise shares one
# working directory and so one parameters.bson, one dataman and one log tree.
# The airframe itself is not the problem: rcS notices that the saved
# SYS_AUTOSTART disagrees with the one derived from PX4_SIM_MODEL and runs
# "param reset_all" before writing the new id. What that reset does not reach is
# dataman, so the quadplane's VTOL mission survives into a plane run and is then
# rejected by feasibility checking; and what it does reach is every parameter
# outside its exclusion list, so whatever you had tuned is discarded each time
# you switch models. Point this at a per-model directory to give each airframe
# its own parameters, missions and logs, and neither happens.
if [ -n "$PX4_FLIGHTAXIS_ROOTFS" ]; then
	rootfs="$PX4_FLIGHTAXIS_ROOTFS"
elif [ "$instance" -eq 0 ]; then
	rootfs="$build_path/rootfs" # this is the working directory
else
	rootfs="$build_path/instance_$instance"
fi

mkdir -p "$rootfs"

# Absolutise: the mkdir above runs in the invocation directory but the pushd
# below runs after a cd into the bridge model directory, so a relative override
# such as PX4_FLIGHTAXIS_ROOTFS=. would create one directory and then boot PX4
# in a different one.
rootfs="$(cd "$rootfs" && pwd)"

# Seed a working directory that has no parameters of its own from the shared one,
# so a new per-model directory starts with the sensor and radio calibration you
# already have rather than none at all. Without this the first boot in a fresh
# directory has magnetometer and gyro ids but no offsets, which raises "Compass 0
# fault"; the airspeed validator then rejects the pitot too, because it checks it
# against an EKF whose yaw the bad compass has already spoiled.
#
# Only when the file is absent, so this never overwrites what you have since
# saved. Nothing else is copied -- dataman in particular stays behind, since
# inheriting another model's mission is one of the things a per-model directory
# exists to prevent. rcS decides what survives: an airframe change makes it run
# "param reset_all", which keeps RC*, CAL_* and COM_FLTMODE* and drops the rest.
#
# Set PX4_FLIGHTAXIS_SEED=0 to start genuinely empty and calibrate from scratch.
default_rootfs="$build_path/rootfs"
if [ "${PX4_FLIGHTAXIS_SEED:-1}" != "0" ] &&
   [ ! -e "$rootfs/parameters.bson" ] &&
   [ -f "$default_rootfs/parameters.bson" ] &&
   [ "$rootfs" != "$(cd "$default_rootfs" && pwd)" ]; then
	cp "$default_rootfs/parameters.bson" "$rootfs/parameters.bson"
	echo "seeded parameters.bson from $default_rootfs (calibration carried over)"
fi

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

# Take everything down with this script, however it ends.
#
# Without this, the kill at the bottom only runs when PX4 returns normally. Kill
# the script itself - or disrupt its process group - and BOTH children are
# orphaned: the bridge keeps holding TCP 4560 and keeps hammering RealFlight
# with SOAP, and PX4 keeps emitting heartbeats so the ground station still shows
# a live vehicle that no longer exists. The next run then fails to bind, and the
# stale vehicle has to be cleared by restarting the GCS.
#
# `pkill -P $$` reaps every direct child, which is both the bridge and PX4 -
# PX4 runs in the foreground so its pid is never captured in a variable.
#
# Scope, honestly: Ctrl-C needs none of this. It signals the whole foreground
# process group, so the bridge and PX4 each receive SIGINT directly and exit on
# their own - verified. What this covers is the script exiting by itself: a
# `set -e` abort, or PX4 returning while the bridge is still up. It does NOT
# rescue `kill <pid-of-this-script>`, because bash defers trap handlers until
# the running foreground command finishes, and that command is PX4. Kill the
# process group rather than the script if you need to take a session down from
# outside its terminal.
fa_cleanup() {
	trap - EXIT INT TERM HUP
	pkill -INT -P $$ 2>/dev/null
	for _ in 1 2 3 4 5 6 7 8 9 10; do
		pgrep -P $$ >/dev/null 2>&1 || return 0
		sleep 0.2
	done
	pkill -KILL -P $$ 2>/dev/null
}
trap fa_cleanup EXIT
trap 'fa_cleanup; exit 130' INT
trap 'fa_cleanup; exit 143' TERM HUP

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

# The EXIT trap does the reaping; wait so the shell does not exit while the
# bridge is still closing its RealFlight session.
fa_cleanup
wait 2>/dev/null

exit $px4_status
