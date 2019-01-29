cmake_minimum_required(VERSION 3.0.0)
project(Drone)

set (CMAKE_CXX_STANDARD 14)

# Include the Necessary Headers
include_directories(./include
    ./include/Controls
    ./include/Sensors
    ./include/Utils
)


# Add the Linker Directories
#link_directories()

# Some global configurations
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Build our Executable
set(BASE_SOURCES source/Drone.cpp 
    source/Sensors/Sensors.cpp
    source/Controls/DirectControls.cpp
)

add_library(${PROJECT_NAME} STATIC ${BASE_SOURCES})
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_link_libraries(${PROJECT_NAME} rpc)

#install(TARGETS $(PROJECT_NAME) DESTINATION lib)