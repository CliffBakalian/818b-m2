# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /usr/local/share/player/examples/plugins/opaquedriver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/justin/818b-m2/playercontroller/build

# Include any dependencies generated for this target.
include CMakeFiles/opaquedriver.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/opaquedriver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/opaquedriver.dir/flags.make

CMakeFiles/opaquedriver.dir/opaquedriver.o: CMakeFiles/opaquedriver.dir/flags.make
CMakeFiles/opaquedriver.dir/opaquedriver.o: /usr/local/share/player/examples/plugins/opaquedriver/opaquedriver.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/justin/818b-m2/playercontroller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/opaquedriver.dir/opaquedriver.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opaquedriver.dir/opaquedriver.o -c /usr/local/share/player/examples/plugins/opaquedriver/opaquedriver.cc

CMakeFiles/opaquedriver.dir/opaquedriver.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opaquedriver.dir/opaquedriver.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/local/share/player/examples/plugins/opaquedriver/opaquedriver.cc > CMakeFiles/opaquedriver.dir/opaquedriver.i

CMakeFiles/opaquedriver.dir/opaquedriver.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opaquedriver.dir/opaquedriver.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/local/share/player/examples/plugins/opaquedriver/opaquedriver.cc -o CMakeFiles/opaquedriver.dir/opaquedriver.s

# Object files for target opaquedriver
opaquedriver_OBJECTS = \
"CMakeFiles/opaquedriver.dir/opaquedriver.o"

# External object files for target opaquedriver
opaquedriver_EXTERNAL_OBJECTS =

libopaquedriver.so: CMakeFiles/opaquedriver.dir/opaquedriver.o
libopaquedriver.so: CMakeFiles/opaquedriver.dir/build.make
libopaquedriver.so: CMakeFiles/opaquedriver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/justin/818b-m2/playercontroller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libopaquedriver.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opaquedriver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/opaquedriver.dir/build: libopaquedriver.so

.PHONY : CMakeFiles/opaquedriver.dir/build

CMakeFiles/opaquedriver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/opaquedriver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/opaquedriver.dir/clean

CMakeFiles/opaquedriver.dir/depend:
	cd /home/justin/818b-m2/playercontroller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/local/share/player/examples/plugins/opaquedriver /usr/local/share/player/examples/plugins/opaquedriver /home/justin/818b-m2/playercontroller/build /home/justin/818b-m2/playercontroller/build /home/justin/818b-m2/playercontroller/build/CMakeFiles/opaquedriver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/opaquedriver.dir/depend

