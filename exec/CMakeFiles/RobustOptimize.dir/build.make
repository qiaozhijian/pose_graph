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
CMAKE_SOURCE_DIR = /home/cobra/Slam/MySlam/pose_graph

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cobra/Slam/MySlam/pose_graph/exec

# Include any dependencies generated for this target.
include CMakeFiles/RobustOptimize.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RobustOptimize.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RobustOptimize.dir/flags.make

CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.o: CMakeFiles/RobustOptimize.dir/flags.make
CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.o: RobustOptimize.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cobra/Slam/MySlam/pose_graph/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.o -c /home/cobra/Slam/MySlam/pose_graph/exec/RobustOptimize.cpp

CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cobra/Slam/MySlam/pose_graph/exec/RobustOptimize.cpp > CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.i

CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cobra/Slam/MySlam/pose_graph/exec/RobustOptimize.cpp -o CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.s

CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.o: CMakeFiles/RobustOptimize.dir/flags.make
CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.o: ../src/RobustOptimizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cobra/Slam/MySlam/pose_graph/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.o -c /home/cobra/Slam/MySlam/pose_graph/src/RobustOptimizer.cpp

CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cobra/Slam/MySlam/pose_graph/src/RobustOptimizer.cpp > CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.i

CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cobra/Slam/MySlam/pose_graph/src/RobustOptimizer.cpp -o CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.s

CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.o: CMakeFiles/RobustOptimize.dir/flags.make
CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.o: ../src/optimizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cobra/Slam/MySlam/pose_graph/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.o -c /home/cobra/Slam/MySlam/pose_graph/src/optimizer.cpp

CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cobra/Slam/MySlam/pose_graph/src/optimizer.cpp > CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.i

CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cobra/Slam/MySlam/pose_graph/src/optimizer.cpp -o CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.s

# Object files for target RobustOptimize
RobustOptimize_OBJECTS = \
"CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.o" \
"CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.o" \
"CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.o"

# External object files for target RobustOptimize
RobustOptimize_EXTERNAL_OBJECTS =

RobustOptimize: CMakeFiles/RobustOptimize.dir/RobustOptimize.cpp.o
RobustOptimize: CMakeFiles/RobustOptimize.dir/src/RobustOptimizer.cpp.o
RobustOptimize: CMakeFiles/RobustOptimize.dir/src/optimizer.cpp.o
RobustOptimize: CMakeFiles/RobustOptimize.dir/build.make
RobustOptimize: /usr/lib/x86_64-linux-gnu/libcxsparse.so
RobustOptimize: /usr/lib/x86_64-linux-gnu/libcholmod.so
RobustOptimize: /usr/lib/x86_64-linux-gnu/libglog.so
RobustOptimize: CMakeFiles/RobustOptimize.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cobra/Slam/MySlam/pose_graph/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable RobustOptimize"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RobustOptimize.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RobustOptimize.dir/build: RobustOptimize

.PHONY : CMakeFiles/RobustOptimize.dir/build

CMakeFiles/RobustOptimize.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RobustOptimize.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RobustOptimize.dir/clean

CMakeFiles/RobustOptimize.dir/depend:
	cd /home/cobra/Slam/MySlam/pose_graph/exec && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cobra/Slam/MySlam/pose_graph /home/cobra/Slam/MySlam/pose_graph /home/cobra/Slam/MySlam/pose_graph/exec /home/cobra/Slam/MySlam/pose_graph/exec /home/cobra/Slam/MySlam/pose_graph/exec/CMakeFiles/RobustOptimize.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RobustOptimize.dir/depend

