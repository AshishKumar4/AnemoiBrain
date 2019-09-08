cmake_minimum_required(VERSION 3.0.0)
project(OnboardCore)
set (ONBOARD_ROOT ./)
set (AIRSIM_ROOT /opt/AirSim)
set (CMAKE_CXX_STANDARD 17)

# We need to compile in AIRSIM MODE
add_definitions(-DMODE_AIRSIM)

LIST(APPEND CMAKE_MODULE_PATH "cmake_templates") 
INCLUDE("cmake_templates/CommonSetup.cmake")
CommonSetup()

# Include the Necessary Headers
# Set the path to AirLib directly correctly !!!
include_directories(include
    include/LowLevel
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

# Some global configurations
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Build our Library
# Build our Abstract Server and Flight Controller Interface
set(SERVER_SOURCES "source/AbstractServer.cpp"
	"source/AutoNavigation.cpp" 
    "source/Sensors/InertialMeasurement.cpp" "source/Sensors/Location.cpp" "source/Sensors/SensorFusion.cpp"
    "source/Sensors/Sensors.cpp"
	"source/Core.cpp")
add_library(${PROJECT_NAME} SHARED ${SERVER_SOURCES})
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} rpc)
target_link_libraries(${PROJECT_NAME} MavLinkCom)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)
