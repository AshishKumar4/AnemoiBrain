cmake_minimum_required(VERSION 3.0.0)
project(GardienOnboard)
set (ONBOARD_ROOT ./)

if(COMPILE_MODE STREQUAL "airsim")
    message("AIRSIM MODE")
    set (AIRSIM_ROOT /opt/AirSim)
    set (CMAKE_CXX_STANDARD 11)
    LIST(APPEND CMAKE_MODULE_PATH "cmake_templates") 
    INCLUDE("Utils/cmake_templates/CommonSetup.cmake")
    CommonSetup()
endif()

# Include the Necessary Headers
include_directories(${ONBOARD_ROOT}/Utils/include
    ${ONBOARD_ROOT}/ControlServer/include
    ${ONBOARD_ROOT}/SensorServer/include
    )


# Add the Linker Directories
link_directories(${ONBOARD_ROOT}/Utils/build/
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
target_link_libraries(${PROJECT_NAME} SensorServer)
target_link_libraries(${PROJECT_NAME} OnboardUtils)
target_link_libraries(${PROJECT_NAME} rpc)