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
include CMakeFiles/client_async_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/client_async_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/client_async_test.dir/flags.make

CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.o: CMakeFiles/client_async_test.dir/flags.make
CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.o: ../examples/client_async_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.o -c "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/examples/client_async_test.cpp"

CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/examples/client_async_test.cpp" > CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.i

CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/examples/client_async_test.cpp" -o CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.s

# Object files for target client_async_test
client_async_test_OBJECTS = \
"CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.o"

# External object files for target client_async_test
client_async_test_EXTERNAL_OBJECTS =

client_async_test: CMakeFiles/client_async_test.dir/examples/client_async_test.cpp.o
client_async_test: CMakeFiles/client_async_test.dir/build.make
client_async_test: libmspclient.so
client_async_test: libmsp_msg_print.so
client_async_test: CMakeFiles/client_async_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable client_async_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/client_async_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/client_async_test.dir/build: client_async_test

.PHONY : CMakeFiles/client_async_test.dir/build

CMakeFiles/client_async_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/client_async_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/client_async_test.dir/clean

CMakeFiles/client_async_test.dir/depend:
	cd "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/ControlAbstraction/BasicControls/LowLevel/MSP/build_pc/CMakeFiles/client_async_test.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/client_async_test.dir/depend

