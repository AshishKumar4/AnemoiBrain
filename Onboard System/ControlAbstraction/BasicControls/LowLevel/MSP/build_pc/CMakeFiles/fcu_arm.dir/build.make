# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_SOURCE_DIR = "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc"

# Include any dependencies generated for this target.
include CMakeFiles/fcu_arm.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/fcu_arm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/fcu_arm.dir/flags.make

CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.o: CMakeFiles/fcu_arm.dir/flags.make
CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.o: ../examples/fcu_arm_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.o -c "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/examples/fcu_arm_test.cpp"

CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/examples/fcu_arm_test.cpp" > CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.i

CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/examples/fcu_arm_test.cpp" -o CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.s

# Object files for target fcu_arm
fcu_arm_OBJECTS = \
"CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.o"

# External object files for target fcu_arm
fcu_arm_EXTERNAL_OBJECTS =

fcu_arm: CMakeFiles/fcu_arm.dir/examples/fcu_arm_test.cpp.o
fcu_arm: CMakeFiles/fcu_arm.dir/build.make
fcu_arm: libmsp_fcu.so
fcu_arm: libmspclient.so
fcu_arm: CMakeFiles/fcu_arm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable fcu_arm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fcu_arm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/fcu_arm.dir/build: fcu_arm

.PHONY : CMakeFiles/fcu_arm.dir/build

CMakeFiles/fcu_arm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/fcu_arm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/fcu_arm.dir/clean

CMakeFiles/fcu_arm.dir/depend:
	cd "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc/CMakeFiles/fcu_arm.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/fcu_arm.dir/depend
