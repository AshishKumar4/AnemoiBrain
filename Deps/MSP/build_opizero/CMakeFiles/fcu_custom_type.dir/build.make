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
include CMakeFiles/fcu_custom_type.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/fcu_custom_type.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/fcu_custom_type.dir/flags.make

CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o: CMakeFiles/fcu_custom_type.dir/flags.make
CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o: ../examples/fcu_custom_type.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/Gardien-System/Deps/MSP/build_opizero/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o -c /root/Gardien-System/Deps/MSP/examples/fcu_custom_type.cpp

CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/Gardien-System/Deps/MSP/examples/fcu_custom_type.cpp > CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.i

CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/Gardien-System/Deps/MSP/examples/fcu_custom_type.cpp -o CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.s

CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o.requires:

.PHONY : CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o.requires

CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o.provides: CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o.requires
	$(MAKE) -f CMakeFiles/fcu_custom_type.dir/build.make CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o.provides.build
.PHONY : CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o.provides

CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o.provides.build: CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o


# Object files for target fcu_custom_type
fcu_custom_type_OBJECTS = \
"CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o"

# External object files for target fcu_custom_type
fcu_custom_type_EXTERNAL_OBJECTS =

fcu_custom_type: CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o
fcu_custom_type: CMakeFiles/fcu_custom_type.dir/build.make
fcu_custom_type: libmsp_fcu.so
fcu_custom_type: libmspclient.so
fcu_custom_type: CMakeFiles/fcu_custom_type.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/Gardien-System/Deps/MSP/build_opizero/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable fcu_custom_type"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fcu_custom_type.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/fcu_custom_type.dir/build: fcu_custom_type

.PHONY : CMakeFiles/fcu_custom_type.dir/build

CMakeFiles/fcu_custom_type.dir/requires: CMakeFiles/fcu_custom_type.dir/examples/fcu_custom_type.cpp.o.requires

.PHONY : CMakeFiles/fcu_custom_type.dir/requires

CMakeFiles/fcu_custom_type.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/fcu_custom_type.dir/cmake_clean.cmake
.PHONY : CMakeFiles/fcu_custom_type.dir/clean

CMakeFiles/fcu_custom_type.dir/depend:
	cd /root/Gardien-System/Deps/MSP/build_opizero && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/Gardien-System/Deps/MSP /root/Gardien-System/Deps/MSP /root/Gardien-System/Deps/MSP/build_opizero /root/Gardien-System/Deps/MSP/build_opizero /root/Gardien-System/Deps/MSP/build_opizero/CMakeFiles/fcu_custom_type.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/fcu_custom_type.dir/depend

