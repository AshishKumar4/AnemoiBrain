cmake_minimum_required(VERSION 3.0.0)
project(OnboardUtils)
set (ONBOARD_ROOT ./)
set (CMAKE_CXX_STANDARD 11)

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

# Build our Library
# Build our Abstract Server and Flight Controller Interface
set(SERVER_SOURCES "source/AbstractServer.cpp" "source/HighLevelControl.cpp")
add_library(${PROJECT_NAME} SHARED ${SERVER_SOURCES})
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_link_libraries(${PROJECT_NAME} libmspclient.so)
target_link_libraries(${PROJECT_NAME} libmsp.so)
target_link_libraries(${PROJECT_NAME} libmsp_fcu.so)
target_link_libraries(${PROJECT_NAME} libmsp_msg_print.so)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)