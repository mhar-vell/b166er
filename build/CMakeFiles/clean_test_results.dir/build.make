# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 4.0

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
CMAKE_COMMAND = /Users/marcoreis/miniforge3/envs/ros_env11/bin/cmake

# The command to remove a file.
RM = /Users/marcoreis/miniforge3/envs/ros_env11/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/marcoreis/b166er/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/marcoreis/b166er/build

# Utility rule file for clean_test_results.

# Include any custom commands dependencies for this target.
include CMakeFiles/clean_test_results.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/clean_test_results.dir/progress.make

CMakeFiles/clean_test_results:
	/Users/marcoreis/miniforge3/envs/ros_env11/bin/python3.11 /Users/marcoreis/miniforge3/envs/ros_env11/share/catkin/cmake/test/remove_test_results.py /Users/marcoreis/b166er/build/test_results

CMakeFiles/clean_test_results.dir/codegen:
.PHONY : CMakeFiles/clean_test_results.dir/codegen

clean_test_results: CMakeFiles/clean_test_results
clean_test_results: CMakeFiles/clean_test_results.dir/build.make
.PHONY : clean_test_results

# Rule to build all files generated by this target.
CMakeFiles/clean_test_results.dir/build: clean_test_results
.PHONY : CMakeFiles/clean_test_results.dir/build

CMakeFiles/clean_test_results.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clean_test_results.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clean_test_results.dir/clean

CMakeFiles/clean_test_results.dir/depend:
	cd /Users/marcoreis/b166er/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/marcoreis/b166er/src /Users/marcoreis/b166er/src /Users/marcoreis/b166er/build /Users/marcoreis/b166er/build /Users/marcoreis/b166er/build/CMakeFiles/clean_test_results.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/clean_test_results.dir/depend

