if(ENABLE_LOCKSTEP_SCHEDULER STREQUAL "no")

	# RealFlight (FlightAxis Link) runs on a remote Windows machine,
	# so there is no local simulator binary to find_program() for.

	include(ExternalProject)
	ExternalProject_Add(flightaxis_bridge
		SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/flightaxis/flightaxis_bridge
		CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		BINARY_DIR ${PX4_BINARY_DIR}/build_flightaxis_bridge
		INSTALL_COMMAND ""
		USES_TERMINAL_CONFIGURE true
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
		BUILD_ALWAYS 1
	)

	# flightaxis targets
	set(models
		plane
		quad
		quadplane
		heli
	)

	# find corresponding airframes
	file(GLOB flightaxis_airframes
		RELATIVE ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes
		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_flightaxis_*
	)

	# remove any .post files
	foreach(flightaxis_airframe IN LISTS flightaxis_airframes)
		if(flightaxis_airframe MATCHES ".post")
			list(REMOVE_ITEM flightaxis_airframes ${flightaxis_airframe})
		endif()
	endforeach()
	list(REMOVE_DUPLICATES flightaxis_airframes)

	# default flightaxis target
	add_custom_target(flightaxis
		COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/flightaxis/sitl_run.sh $<TARGET_FILE:px4> "plane" ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS px4 flightaxis_bridge
	)

	foreach(model ${models})

		# match model to airframe
		set(airframe_model_only)
		set(airframe_sys_autostart)
		set(flightaxis_airframe_found)
		foreach(flightaxis_airframe IN LISTS flightaxis_airframes)

			string(REGEX REPLACE ".*_flightaxis_" "" airframe_model_only ${flightaxis_airframe})
			string(REGEX REPLACE "_flightaxis_.*" "" airframe_sys_autostart ${flightaxis_airframe})

			if(model STREQUAL ${airframe_model_only})
				set(flightaxis_airframe_found ${flightaxis_airframe})
				break()
			endif()
		endforeach()

		if(flightaxis_airframe_found)
			#message(STATUS "flightaxis model: ${model} (${airframe_model_only}), airframe: ${flightaxis_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
		else()
			message(WARNING "flightaxis missing model: ${model} (${airframe_model_only}), airframe: ${flightaxis_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
		endif()

		add_custom_target(flightaxis_${model}
			COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/flightaxis/sitl_run.sh $<TARGET_FILE:px4> ${model} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
			WORKING_DIRECTORY ${SITL_WORKING_DIR}
			USES_TERMINAL
			DEPENDS px4 flightaxis_bridge
		)
	endforeach()
endif()
