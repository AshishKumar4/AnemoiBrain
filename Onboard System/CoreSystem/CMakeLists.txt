cmake_minimum_required(VERSION 3.0.0)
project(OnboardCore)
set (ONBOARD_ROOT ./)

# Include the Necessary Headers
include_directories(include
    include/LowLevel
    )


# Add the Linker Directories
link_directories(lib/
    build/
    )

# Initialize the Source Variables
file(GLOB LOWLEVELS "source/*/*.cpp")
#file(GLOB SOURCES "source/*.cpp")

# Some global configurations
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_C_FLAGS "-O3")
set (CMAKE_CXX_STANDARD 17)

# Build our Library
# Build our Abstract Server and Flight Controller Interface
set(SERVER_SOURCES "source/AbstractServer.cpp"
	"source/AutoNavigation.cpp" 
    "source/Sensors/drivers/GeoEstimator.cpp" "source/Sensors/drivers/InertialEstimator.cpp"
	"source/Sensors/InertialSensorFusion.cpp" "source/Sensors/NavSensorFusion.cpp" 
    "source/Sensors/Sensors.cpp"
	"source/Core.cpp")
add_library(${PROJECT_NAME} SHARED ${SERVER_SOURCES})
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_link_libraries(${PROJECT_NAME} libmspclient.so)
#target_link_libraries(${PROJECT_NAME} libmsp.so)
target_link_libraries(${PROJECT_NAME} libmsp_fcu.so)
#target_link_libraries(${PROJECT_NAME} libmsp_msg_print.so)
target_link_libraries(${PROJECT_NAME} boost_system)

target_link_libraries(${PROJECT_NAME} libRTIMULib.so)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)