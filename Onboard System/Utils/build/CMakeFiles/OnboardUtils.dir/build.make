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
CMAKE_SOURCE_DIR = "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/build"

# Include any dependencies generated for this target.
include CMakeFiles/OnboardUtils.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/OnboardUtils.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OnboardUtils.dir/flags.make

CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.o: CMakeFiles/OnboardUtils.dir/flags.make
CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.o: ../source/AbstractServer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.o -c "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/source/AbstractServer.cpp"

CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/source/AbstractServer.cpp" > CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.i

CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/source/AbstractServer.cpp" -o CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.s

CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.o: CMakeFiles/OnboardUtils.dir/flags.make
CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.o: ../source/HighLevelControl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.o -c "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/source/HighLevelControl.cpp"

CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/source/HighLevelControl.cpp" > CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.i

CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/source/HighLevelControl.cpp" -o CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.s

# Object files for target OnboardUtils
OnboardUtils_OBJECTS = \
"CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.o" \
"CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.o"

# External object files for target OnboardUtils
OnboardUtils_EXTERNAL_OBJECTS =

libOnboardUtils.so: CMakeFiles/OnboardUtils.dir/source/AbstractServer.cpp.o
libOnboardUtils.so: CMakeFiles/OnboardUtils.dir/source/HighLevelControl.cpp.o
libOnboardUtils.so: CMakeFiles/OnboardUtils.dir/build.make
libOnboardUtils.so: CMakeFiles/OnboardUtils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libOnboardUtils.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OnboardUtils.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OnboardUtils.dir/build: libOnboardUtils.so

.PHONY : CMakeFiles/OnboardUtils.dir/build

CMakeFiles/OnboardUtils.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/OnboardUtils.dir/cmake_clean.cmake
.PHONY : CMakeFiles/OnboardUtils.dir/clean

CMakeFiles/OnboardUtils.dir/depend:
	cd "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/build" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/build" "/root/Desktop/Anemoi Drone Project/Drone Software/Onboard System/Utils/build/CMakeFiles/OnboardUtils.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/OnboardUtils.dir/depend
