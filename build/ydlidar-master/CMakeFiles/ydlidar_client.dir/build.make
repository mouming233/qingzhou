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
CMAKE_SOURCE_DIR = /home/qyb/qingzhou_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/qyb/qingzhou_ws/build

# Include any dependencies generated for this target.
include ydlidar-master/CMakeFiles/ydlidar_client.dir/depend.make

# Include the progress variables for this target.
include ydlidar-master/CMakeFiles/ydlidar_client.dir/progress.make

# Include the compile flags for this target's objects.
include ydlidar-master/CMakeFiles/ydlidar_client.dir/flags.make

ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o: ydlidar-master/CMakeFiles/ydlidar_client.dir/flags.make
ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o: /home/qyb/qingzhou_ws/src/ydlidar-master/src/ydlidar_client.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qyb/qingzhou_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o"
	cd /home/qyb/qingzhou_ws/build/ydlidar-master && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o -c /home/qyb/qingzhou_ws/src/ydlidar-master/src/ydlidar_client.cpp

ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.i"
	cd /home/qyb/qingzhou_ws/build/ydlidar-master && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qyb/qingzhou_ws/src/ydlidar-master/src/ydlidar_client.cpp > CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.i

ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.s"
	cd /home/qyb/qingzhou_ws/build/ydlidar-master && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qyb/qingzhou_ws/src/ydlidar-master/src/ydlidar_client.cpp -o CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.s

ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o.requires:

.PHONY : ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o.requires

ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o.provides: ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o.requires
	$(MAKE) -f ydlidar-master/CMakeFiles/ydlidar_client.dir/build.make ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o.provides.build
.PHONY : ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o.provides

ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o.provides.build: ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o


# Object files for target ydlidar_client
ydlidar_client_OBJECTS = \
"CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o"

# External object files for target ydlidar_client
ydlidar_client_EXTERNAL_OBJECTS =

/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: ydlidar-master/CMakeFiles/ydlidar_client.dir/build.make
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /opt/ros/melodic/lib/libroscpp.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /opt/ros/melodic/lib/librosconsole.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /opt/ros/melodic/lib/librostime.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /opt/ros/melodic/lib/libcpp_common.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client: ydlidar-master/CMakeFiles/ydlidar_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qyb/qingzhou_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client"
	cd /home/qyb/qingzhou_ws/build/ydlidar-master && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ydlidar_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ydlidar-master/CMakeFiles/ydlidar_client.dir/build: /home/qyb/qingzhou_ws/devel/lib/ydlidar/ydlidar_client

.PHONY : ydlidar-master/CMakeFiles/ydlidar_client.dir/build

ydlidar-master/CMakeFiles/ydlidar_client.dir/requires: ydlidar-master/CMakeFiles/ydlidar_client.dir/src/ydlidar_client.cpp.o.requires

.PHONY : ydlidar-master/CMakeFiles/ydlidar_client.dir/requires

ydlidar-master/CMakeFiles/ydlidar_client.dir/clean:
	cd /home/qyb/qingzhou_ws/build/ydlidar-master && $(CMAKE_COMMAND) -P CMakeFiles/ydlidar_client.dir/cmake_clean.cmake
.PHONY : ydlidar-master/CMakeFiles/ydlidar_client.dir/clean

ydlidar-master/CMakeFiles/ydlidar_client.dir/depend:
	cd /home/qyb/qingzhou_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qyb/qingzhou_ws/src /home/qyb/qingzhou_ws/src/ydlidar-master /home/qyb/qingzhou_ws/build /home/qyb/qingzhou_ws/build/ydlidar-master /home/qyb/qingzhou_ws/build/ydlidar-master/CMakeFiles/ydlidar_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ydlidar-master/CMakeFiles/ydlidar_client.dir/depend

