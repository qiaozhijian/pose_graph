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
include CMakeFiles/unary_edge.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/unary_edge.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/unary_edge.dir/flags.make

CMakeFiles/unary_edge.dir/unary_edge.cpp.o: CMakeFiles/unary_edge.dir/flags.make
CMakeFiles/unary_edge.dir/unary_edge.cpp.o: unary_edge.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/unary_edge.dir/unary_edge.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unary_edge.dir/unary_edge.cpp.o -c /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/unary_edge.cpp

CMakeFiles/unary_edge.dir/unary_edge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unary_edge.dir/unary_edge.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/unary_edge.cpp > CMakeFiles/unary_edge.dir/unary_edge.cpp.i

CMakeFiles/unary_edge.dir/unary_edge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unary_edge.dir/unary_edge.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/unary_edge.cpp -o CMakeFiles/unary_edge.dir/unary_edge.cpp.s

CMakeFiles/unary_edge.dir/src/optimizer.cpp.o: CMakeFiles/unary_edge.dir/flags.make
CMakeFiles/unary_edge.dir/src/optimizer.cpp.o: ../src/optimizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/unary_edge.dir/src/optimizer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unary_edge.dir/src/optimizer.cpp.o -c /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/optimizer.cpp

CMakeFiles/unary_edge.dir/src/optimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unary_edge.dir/src/optimizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/optimizer.cpp > CMakeFiles/unary_edge.dir/src/optimizer.cpp.i

CMakeFiles/unary_edge.dir/src/optimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unary_edge.dir/src/optimizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/optimizer.cpp -o CMakeFiles/unary_edge.dir/src/optimizer.cpp.s

# Object files for target unary_edge
unary_edge_OBJECTS = \
"CMakeFiles/unary_edge.dir/unary_edge.cpp.o" \
"CMakeFiles/unary_edge.dir/src/optimizer.cpp.o"

# External object files for target unary_edge
unary_edge_EXTERNAL_OBJECTS =

unary_edge: CMakeFiles/unary_edge.dir/unary_edge.cpp.o
unary_edge: CMakeFiles/unary_edge.dir/src/optimizer.cpp.o
unary_edge: CMakeFiles/unary_edge.dir/build.make
unary_edge: /usr/lib/x86_64-linux-gnu/libcxsparse.so
unary_edge: /usr/lib/x86_64-linux-gnu/libcholmod.so
unary_edge: /usr/lib/x86_64-linux-gnu/libglog.so
unary_edge: CMakeFiles/unary_edge.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable unary_edge"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/unary_edge.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/unary_edge.dir/build: unary_edge

.PHONY : CMakeFiles/unary_edge.dir/build

CMakeFiles/unary_edge.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/unary_edge.dir/cmake_clean.cmake
.PHONY : CMakeFiles/unary_edge.dir/clean

CMakeFiles/unary_edge.dir/depend:
	cd /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles/unary_edge.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/unary_edge.dir/depend

