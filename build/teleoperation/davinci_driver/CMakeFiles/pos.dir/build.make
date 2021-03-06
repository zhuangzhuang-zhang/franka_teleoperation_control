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
CMAKE_SOURCE_DIR = /home/master/franka_final_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/master/franka_final_ws/build

# Include any dependencies generated for this target.
include teleoperation/davinci_driver/CMakeFiles/pos.dir/depend.make

# Include the progress variables for this target.
include teleoperation/davinci_driver/CMakeFiles/pos.dir/progress.make

# Include the compile flags for this target's objects.
include teleoperation/davinci_driver/CMakeFiles/pos.dir/flags.make

teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o: teleoperation/davinci_driver/CMakeFiles/pos.dir/flags.make
teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o: /home/master/franka_final_ws/src/teleoperation/davinci_driver/src/motor/pos.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/master/franka_final_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o"
	cd /home/master/franka_final_ws/build/teleoperation/davinci_driver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pos.dir/src/motor/pos.cpp.o -c /home/master/franka_final_ws/src/teleoperation/davinci_driver/src/motor/pos.cpp

teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pos.dir/src/motor/pos.cpp.i"
	cd /home/master/franka_final_ws/build/teleoperation/davinci_driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/master/franka_final_ws/src/teleoperation/davinci_driver/src/motor/pos.cpp > CMakeFiles/pos.dir/src/motor/pos.cpp.i

teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pos.dir/src/motor/pos.cpp.s"
	cd /home/master/franka_final_ws/build/teleoperation/davinci_driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/master/franka_final_ws/src/teleoperation/davinci_driver/src/motor/pos.cpp -o CMakeFiles/pos.dir/src/motor/pos.cpp.s

teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o.requires:

.PHONY : teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o.requires

teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o.provides: teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o.requires
	$(MAKE) -f teleoperation/davinci_driver/CMakeFiles/pos.dir/build.make teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o.provides.build
.PHONY : teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o.provides

teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o.provides.build: teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o


# Object files for target pos
pos_OBJECTS = \
"CMakeFiles/pos.dir/src/motor/pos.cpp.o"

# External object files for target pos
pos_EXTERNAL_OBJECTS =

/home/master/franka_final_ws/devel/lib/libpos.so: teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o
/home/master/franka_final_ws/devel/lib/libpos.so: teleoperation/davinci_driver/CMakeFiles/pos.dir/build.make
/home/master/franka_final_ws/devel/lib/libpos.so: /home/master/franka_final_ws/devel/lib/libSDO.so
/home/master/franka_final_ws/devel/lib/libpos.so: /home/master/franka_final_ws/src/teleoperation/davinci_driver/lib/libcontrolcan.so
/home/master/franka_final_ws/devel/lib/libpos.so: /home/master/franka_final_ws/devel/lib/libcan.so
/home/master/franka_final_ws/devel/lib/libpos.so: /home/master/franka_final_ws/src/teleoperation/davinci_driver/lib/libcontrolcan.so
/home/master/franka_final_ws/devel/lib/libpos.so: teleoperation/davinci_driver/CMakeFiles/pos.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/master/franka_final_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/master/franka_final_ws/devel/lib/libpos.so"
	cd /home/master/franka_final_ws/build/teleoperation/davinci_driver && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pos.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
teleoperation/davinci_driver/CMakeFiles/pos.dir/build: /home/master/franka_final_ws/devel/lib/libpos.so

.PHONY : teleoperation/davinci_driver/CMakeFiles/pos.dir/build

teleoperation/davinci_driver/CMakeFiles/pos.dir/requires: teleoperation/davinci_driver/CMakeFiles/pos.dir/src/motor/pos.cpp.o.requires

.PHONY : teleoperation/davinci_driver/CMakeFiles/pos.dir/requires

teleoperation/davinci_driver/CMakeFiles/pos.dir/clean:
	cd /home/master/franka_final_ws/build/teleoperation/davinci_driver && $(CMAKE_COMMAND) -P CMakeFiles/pos.dir/cmake_clean.cmake
.PHONY : teleoperation/davinci_driver/CMakeFiles/pos.dir/clean

teleoperation/davinci_driver/CMakeFiles/pos.dir/depend:
	cd /home/master/franka_final_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/master/franka_final_ws/src /home/master/franka_final_ws/src/teleoperation/davinci_driver /home/master/franka_final_ws/build /home/master/franka_final_ws/build/teleoperation/davinci_driver /home/master/franka_final_ws/build/teleoperation/davinci_driver/CMakeFiles/pos.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : teleoperation/davinci_driver/CMakeFiles/pos.dir/depend

