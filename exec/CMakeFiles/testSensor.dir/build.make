# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec

# Include any dependencies generated for this target.
include CMakeFiles/testSensor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/testSensor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testSensor.dir/flags.make

CMakeFiles/testSensor.dir/testSensor.cpp.o: CMakeFiles/testSensor.dir/flags.make
CMakeFiles/testSensor.dir/testSensor.cpp.o: testSensor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/testSensor.dir/testSensor.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testSensor.dir/testSensor.cpp.o -c /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/testSensor.cpp

CMakeFiles/testSensor.dir/testSensor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testSensor.dir/testSensor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/testSensor.cpp > CMakeFiles/testSensor.dir/testSensor.cpp.i

CMakeFiles/testSensor.dir/testSensor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testSensor.dir/testSensor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/testSensor.cpp -o CMakeFiles/testSensor.dir/testSensor.cpp.s

CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.o: CMakeFiles/testSensor.dir/flags.make
CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.o: ../src/doubleoptimizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.o -c /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/doubleoptimizer.cpp

CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/doubleoptimizer.cpp > CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.i

CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/doubleoptimizer.cpp -o CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.s

CMakeFiles/testSensor.dir/src/optimizer.cpp.o: CMakeFiles/testSensor.dir/flags.make
CMakeFiles/testSensor.dir/src/optimizer.cpp.o: ../src/optimizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/testSensor.dir/src/optimizer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testSensor.dir/src/optimizer.cpp.o -c /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/optimizer.cpp

CMakeFiles/testSensor.dir/src/optimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testSensor.dir/src/optimizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/optimizer.cpp > CMakeFiles/testSensor.dir/src/optimizer.cpp.i

CMakeFiles/testSensor.dir/src/optimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testSensor.dir/src/optimizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/optimizer.cpp -o CMakeFiles/testSensor.dir/src/optimizer.cpp.s

# Object files for target testSensor
testSensor_OBJECTS = \
"CMakeFiles/testSensor.dir/testSensor.cpp.o" \
"CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.o" \
"CMakeFiles/testSensor.dir/src/optimizer.cpp.o"

# External object files for target testSensor
testSensor_EXTERNAL_OBJECTS =

testSensor: CMakeFiles/testSensor.dir/testSensor.cpp.o
testSensor: CMakeFiles/testSensor.dir/src/doubleoptimizer.cpp.o
testSensor: CMakeFiles/testSensor.dir/src/optimizer.cpp.o
testSensor: CMakeFiles/testSensor.dir/build.make
testSensor: /usr/lib/x86_64-linux-gnu/libcxsparse.so
testSensor: /usr/lib/x86_64-linux-gnu/libcholmod.so
testSensor: /usr/lib/x86_64-linux-gnu/libglog.so
testSensor: CMakeFiles/testSensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable testSensor"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testSensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testSensor.dir/build: testSensor

.PHONY : CMakeFiles/testSensor.dir/build

CMakeFiles/testSensor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testSensor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testSensor.dir/clean

CMakeFiles/testSensor.dir/depend:
	cd /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles/testSensor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/testSensor.dir/depend
