# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /root/Gardien-System/Deps/MSP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/Gardien-System/Deps/MSP/build_opizero

# Include any dependencies generated for this target.
include CMakeFiles/msp_connection_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/msp_connection_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/msp_connection_test.dir/flags.make

CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o: CMakeFiles/msp_connection_test.dir/flags.make
CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o: ../examples/msp_connection_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/Gardien-System/Deps/MSP/build_opizero/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o -c /root/Gardien-System/Deps/MSP/examples/msp_connection_test.cpp

CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/Gardien-System/Deps/MSP/examples/msp_connection_test.cpp > CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.i

CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/Gardien-System/Deps/MSP/examples/msp_connection_test.cpp -o CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.s

CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o.requires:

.PHONY : CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o.requires

CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o.provides: CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o.requires
	$(MAKE) -f CMakeFiles/msp_connection_test.dir/build.make CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o.provides.build
.PHONY : CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o.provides

CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o.provides.build: CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o


# Object files for target msp_connection_test
msp_connection_test_OBJECTS = \
"CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o"

# External object files for target msp_connection_test
msp_connection_test_EXTERNAL_OBJECTS =

msp_connection_test: CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o
msp_connection_test: CMakeFiles/msp_connection_test.dir/build.make
msp_connection_test: libmsp.so
msp_connection_test: CMakeFiles/msp_connection_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/Gardien-System/Deps/MSP/build_opizero/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable msp_connection_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/msp_connection_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/msp_connection_test.dir/build: msp_connection_test

.PHONY : CMakeFiles/msp_connection_test.dir/build

CMakeFiles/msp_connection_test.dir/requires: CMakeFiles/msp_connection_test.dir/examples/msp_connection_test.cpp.o.requires

.PHONY : CMakeFiles/msp_connection_test.dir/requires

CMakeFiles/msp_connection_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/msp_connection_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/msp_connection_test.dir/clean

CMakeFiles/msp_connection_test.dir/depend:
	cd /root/Gardien-System/Deps/MSP/build_opizero && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/Gardien-System/Deps/MSP /root/Gardien-System/Deps/MSP /root/Gardien-System/Deps/MSP/build_opizero /root/Gardien-System/Deps/MSP/build_opizero /root/Gardien-System/Deps/MSP/build_opizero/CMakeFiles/msp_connection_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/msp_connection_test.dir/depend

