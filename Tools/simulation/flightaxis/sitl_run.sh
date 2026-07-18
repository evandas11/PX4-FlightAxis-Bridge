#!/usr/bin/env bash

set -e

if [ "$#" -lt 4 ]; then
	echo "usage: sitl_run.sh sitl_bin model src_path build_path"
	exit 1
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

export PX4_SIM_MODEL=flightaxis_${model}

# RealFlight host: env override, default localhost (RealFlight in a VM / same box)
FA_IP="${PX4_FLIGHTAXIS_IP:-127.0.0.1}"

echo "FlightAxis setup"
cd "${src_path}/Tools/simulation/flightaxis/flightaxis_bridge/"
./FA_check.py "${FA_IP}" || { echo "RealFlight FlightAxis not reachable at ${FA_IP}:18083"; exit 1; }

"${build_path}/build_flightaxis_bridge/flightaxis_bridge" 0 "${FA_IP}" \
	`./get_FAbridge_params.py "models/${model}.json"` &
FA_BRIDGE_PID=$!

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

sitl_command="\"$sitl_bin\" \"$build_path\"/etc"

echo SITL COMMAND: $sitl_command

eval $sitl_command

popd >/dev/null

kill $FA_BRIDGE_PID
