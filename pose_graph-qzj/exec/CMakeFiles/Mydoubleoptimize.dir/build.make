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
include CMakeFiles/Mydoubleoptimize.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Mydoubleoptimize.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Mydoubleoptimize.dir/flags.make

CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.o: CMakeFiles/Mydoubleoptimize.dir/flags.make
CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.o: Mydoubleoptimize.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.o -c /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/Mydoubleoptimize.cpp

CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/Mydoubleoptimize.cpp > CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.i

CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/Mydoubleoptimize.cpp -o CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.s

CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.o: CMakeFiles/Mydoubleoptimize.dir/flags.make
CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.o: ../src/doubleoptimizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.o -c /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/doubleoptimizer.cpp

CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/doubleoptimizer.cpp > CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.i

CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/src/doubleoptimizer.cpp -o CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.s

# Object files for target Mydoubleoptimize
Mydoubleoptimize_OBJECTS = \
"CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.o" \
"CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.o"

# External object files for target Mydoubleoptimize
Mydoubleoptimize_EXTERNAL_OBJECTS =

Mydoubleoptimize: CMakeFiles/Mydoubleoptimize.dir/Mydoubleoptimize.cpp.o
Mydoubleoptimize: CMakeFiles/Mydoubleoptimize.dir/src/doubleoptimizer.cpp.o
Mydoubleoptimize: CMakeFiles/Mydoubleoptimize.dir/build.make
Mydoubleoptimize: /usr/lib/x86_64-linux-gnu/libcxsparse.so
Mydoubleoptimize: /usr/lib/x86_64-linux-gnu/libcholmod.so
Mydoubleoptimize: /usr/lib/x86_64-linux-gnu/libglog.so
Mydoubleoptimize: CMakeFiles/Mydoubleoptimize.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable Mydoubleoptimize"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Mydoubleoptimize.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Mydoubleoptimize.dir/build: Mydoubleoptimize

.PHONY : CMakeFiles/Mydoubleoptimize.dir/build

CMakeFiles/Mydoubleoptimize.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Mydoubleoptimize.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Mydoubleoptimize.dir/clean

CMakeFiles/Mydoubleoptimize.dir/depend:
	cd /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec /home/cobra/Slam/MySlam/pose_graph/pose_graph-qzj/exec/CMakeFiles/Mydoubleoptimize.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Mydoubleoptimize.dir/depend
