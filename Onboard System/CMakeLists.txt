cmake_minimum_required(VERSION 3.0.0)
project(GardienOnboard)
set (ONBOARD_ROOT ./)
set (CMAKE_CXX_STANDARD 17)

if(COMPILE_MODE STREQUAL "airsim")
    ADD_DEFINITIONS(-DMODE_AIRSIM)
    message("Compiling in AIRSIM MODE")
    set (AIRSIM_ROOT /opt/AirSim)
    set (CMAKE_CXX_STANDARD 17)
    LIST(APPEND CMAKE_MODULE_PATH "cmake_templates") 
    INCLUDE("CoreSystem/cmake_templates/CommonSetup.cmake")
    CommonSetup()

	# Include the Necessary Headers
	# Set the path to AirLib directly correctly !!!
	include_directories(
		${AIRSIM_ROOT}/AirLib/include
		${RPC_LIB_INCLUDES}
		${AIRSIM_ROOT}/MavLinkCom/include
		${AIRSIM_ROOT}/MavLinkCom/common_utils
		${AIRSIM_ROOT}/AirLib/deps/eigen3
		)


	# Add the Linker Directories
	# Set the path to AirLib directly correctly !!!
	link_directories(lib/
		build/
		${AIRSIM_ROOT}/AirLib/lib/x64/Debug
		)
endif()

# Include the Necessary Headers
include_directories(${ONBOARD_ROOT}/CoreSystem/include
    ${ONBOARD_ROOT}/ControlServer/include
    ${ONBOARD_ROOT}/SensorServer/include
    )


# Add the Linker Directories
link_directories(${ONBOARD_ROOT}/CoreSystem/build/
    ${ONBOARD_ROOT}/ControlServer/build/
    ${ONBOARD_ROOT}/SensorServer/build/
    ${AIRSIM_ROOT}/AirLib/lib/x64/Debug
    )

# Some global configurations
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Build our Executable
set(SERVER_SOURCES "main.cpp")
add_executable(${PROJECT_NAME} ${SERVER_SOURCES})
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_link_libraries(${PROJECT_NAME} ControlServer)
#target_link_libraries(${PROJECT_NAME} SensorServer)
target_link_libraries(${PROJECT_NAME} OnboardCore)
target_link_libraries(${PROJECT_NAME} rpc)
target_link_libraries(${PROJECT_NAME} boost_system)