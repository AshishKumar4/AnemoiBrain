cmake_minimum_required(VERSION 3.0.0)
project(GardienOnboard)
set (ONBOARD_ROOT ./)

set (AIRSIM_ROOT /opt/AirSim)
set (CMAKE_CXX_STANDARD 11)

LIST(APPEND CMAKE_MODULE_PATH "cmake_templates") 
INCLUDE("Utils/cmake_templates/CommonSetup.cmake")
#CommonSetup()

# Include the Necessary Headers
include_directories(${ONBOARD_ROOT}/Utils/include
    ${ONBOARD_ROOT}/ControlAbstraction/include
    ${ONBOARD_ROOT}/SensorAbstraction/include
    )


# Add the Linker Directories
link_directories(${ONBOARD_ROOT}/Utils/build/
    ${ONBOARD_ROOT}/ControlAbstraction/build/
    ${ONBOARD_ROOT}/SensorAbstraction/build/
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