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

# Utility rule file for run_tests_fiducial_slam_rostest_test_auto_init_403.test.

# Include the progress variables for this target.
include fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/progress.make

fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test:
	cd /home/foscar/Auto-Race-/build/fiducials/fiducial_slam && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/run_tests.py /home/foscar/Auto-Race-/build/test_results/fiducial_slam/rostest-test_auto_init_403.xml "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/foscar/Auto-Race-/src/fiducials/fiducial_slam --package=fiducial_slam --results-filename test_auto_init_403.xml --results-base-dir \"/home/foscar/Auto-Race-/build/test_results\" /home/foscar/Auto-Race-/src/fiducials/fiducial_slam/test/auto_init_403.test "

run_tests_fiducial_slam_rostest_test_auto_init_403.test: fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test
run_tests_fiducial_slam_rostest_test_auto_init_403.test: fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/build.make

.PHONY : run_tests_fiducial_slam_rostest_test_auto_init_403.test

# Rule to build all files generated by this target.
fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/build: run_tests_fiducial_slam_rostest_test_auto_init_403.test

.PHONY : fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/build

fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/clean:
	cd /home/foscar/Auto-Race-/build/fiducials/fiducial_slam && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/cmake_clean.cmake
.PHONY : fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/clean

fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/depend:
	cd /home/foscar/Auto-Race-/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/foscar/Auto-Race-/src /home/foscar/Auto-Race-/src/fiducials/fiducial_slam /home/foscar/Auto-Race-/build /home/foscar/Auto-Race-/build/fiducials/fiducial_slam /home/foscar/Auto-Race-/build/fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/depend
