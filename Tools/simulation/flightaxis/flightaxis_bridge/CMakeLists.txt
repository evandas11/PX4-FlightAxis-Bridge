cmake_minimum_required(VERSION 3.10)

project(flightaxis_bridge)

# FindMAVLink lives in ./cmake (a copy of the FlightGear bridge's, kept in-tree
# so this does not depend on that submodule being initialised)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# The ExternalProject registration passes -DCMAKE_BUILD_TYPE=RelWithDebInfo, but
# a direct `cmake .. && make` (how the docs describe manual and multi-instance
# runs, and how HITL is likely driven) would otherwise get an unoptimised build:
# -O0 Eigen quaternion maths and MAVLink encoders show up only as a climbing
# glitches= counter, with no hint that the build type is the cause.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
	message(STATUS "CMAKE_BUILD_TYPE not set, defaulting to ${CMAKE_BUILD_TYPE}")
endif()

find_package(Eigen3 REQUIRED)
find_package(MAVLink)

message(STATUS "${MAVLINK_INCLUDE_DIRS}")

add_executable(flightaxis_bridge
	src/flightaxis_bridge.cpp
	src/px4_communicator.cpp
	src/fa_communicator.cpp
	src/vehicle_state.cpp
	src/geo_mag_declination.cpp
	)

target_include_directories(flightaxis_bridge
	BEFORE
	PUBLIC ${MAVLINK_INCLUDE_DIRS})

target_compile_options(flightaxis_bridge PUBLIC -g -fexceptions -Wno-cast-align -Wno-address-of-packed-member)
target_link_libraries(flightaxis_bridge Eigen3::Eigen pthread)
