# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /home/foscar/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/foscar/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/foscar/Auto-Race-/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/foscar/Auto-Race-/build

# Utility rule file for lane_detection_geneus.

# Include the progress variables for this target.
include lane_detection/CMakeFiles/lane_detection_geneus.dir/progress.make

lane_detection_geneus: lane_detection/CMakeFiles/lane_detection_geneus.dir/build.make

.PHONY : lane_detection_geneus

# Rule to build all files generated by this target.
lane_detection/CMakeFiles/lane_detection_geneus.dir/build: lane_detection_geneus

.PHONY : lane_detection/CMakeFiles/lane_detection_geneus.dir/build

lane_detection/CMakeFiles/lane_detection_geneus.dir/clean:
	cd /home/foscar/Auto-Race-/build/lane_detection && $(CMAKE_COMMAND) -P CMakeFiles/lane_detection_geneus.dir/cmake_clean.cmake
.PHONY : lane_detection/CMakeFiles/lane_detection_geneus.dir/clean

lane_detection/CMakeFiles/lane_detection_geneus.dir/depend:
	cd /home/foscar/Auto-Race-/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/foscar/Auto-Race-/src /home/foscar/Auto-Race-/src/lane_detection /home/foscar/Auto-Race-/build /home/foscar/Auto-Race-/build/lane_detection /home/foscar/Auto-Race-/build/lane_detection/CMakeFiles/lane_detection_geneus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lane_detection/CMakeFiles/lane_detection_geneus.dir/depend

