cmake_minimum_required(VERSION 3.0.0)
project(SensorServer)
set (ONBOARD_ROOT ../)
set (CMAKE_CXX_STANDARD 11)

# Include the Necessary Headers
include_directories(include
    include/LowLevel
    ${ONBOARD_ROOT}/Utils/include
    )

# Add the Linker Directories
# Set the path to AirLib directly correctly !!!
link_directories(build/
    ${ONBOARD_ROOT}/Utils/build
    )

# Some global configurations
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Build our Control Server
set(SERVER_SOURCES "source/SensorsServer.cpp")
#add_executable(${PROJECT_NAME} ${SERVER_SOURCES})
add_library(${PROJECT_NAME} SHARED ${SERVER_SOURCES})
target_link_libraries(${PROJECT_NAME} OnboardUtils)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)
