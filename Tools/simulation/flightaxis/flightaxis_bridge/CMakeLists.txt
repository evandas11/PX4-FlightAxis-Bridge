cmake_minimum_required(VERSION 3.10)

project(flightaxis_bridge)

# FindMAVLink lives in ./cmake (copy from the FlightGear bridge); fall back to
# the FlightGear bridge's module dir so an in-tree checkout works out of the box
list(APPEND CMAKE_MODULE_PATH
	"${CMAKE_CURRENT_LIST_DIR}/cmake"
	"${CMAKE_CURRENT_LIST_DIR}/../../flightgear/flightgear_bridge/cmake"
	)

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
