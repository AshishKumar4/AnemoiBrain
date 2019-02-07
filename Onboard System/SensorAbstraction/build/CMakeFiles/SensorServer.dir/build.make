# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction/build"

# Include any dependencies generated for this target.
include CMakeFiles/SensorServer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SensorServer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SensorServer.dir/flags.make

CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.o: CMakeFiles/SensorServer.dir/flags.make
CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.o: ../source/SensorsServer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.o -c "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction/source/SensorsServer.cpp"

CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction/source/SensorsServer.cpp" > CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.i

CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction/source/SensorsServer.cpp" -o CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.s

# Object files for target SensorServer
SensorServer_OBJECTS = \
"CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.o"

# External object files for target SensorServer
SensorServer_EXTERNAL_OBJECTS =

libSensorServer.so: CMakeFiles/SensorServer.dir/source/SensorsServer.cpp.o
libSensorServer.so: CMakeFiles/SensorServer.dir/build.make
libSensorServer.so: CMakeFiles/SensorServer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libSensorServer.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SensorServer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SensorServer.dir/build: libSensorServer.so

.PHONY : CMakeFiles/SensorServer.dir/build

CMakeFiles/SensorServer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SensorServer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SensorServer.dir/clean

CMakeFiles/SensorServer.dir/depend:
	cd "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction/build" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction/build" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/SensorAbstraction/build/CMakeFiles/SensorServer.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/SensorServer.dir/depend
