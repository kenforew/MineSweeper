# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.12

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files (x86)\Microsoft Visual Studio\2017\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files (x86)\Microsoft Visual Studio\2017\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\kenforew\cppcode

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\kenforew\cppcode\build

# Utility rule file for ExperimentalStart.

# Include the progress variables for this target.
include CMakeFiles/ExperimentalStart.dir/progress.make

CMakeFiles/ExperimentalStart:
	cd /d C:\Users\kenforew\cppcode\build && "C:\Program Files (x86)\Microsoft Visual Studio\2017\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe" -D ExperimentalStart

ExperimentalStart: CMakeFiles/ExperimentalStart
ExperimentalStart: CMakeFiles/ExperimentalStart.dir/build.make

.PHONY : ExperimentalStart

# Rule to build all files generated by this target.
CMakeFiles/ExperimentalStart.dir/build: ExperimentalStart

.PHONY : CMakeFiles/ExperimentalStart.dir/build

CMakeFiles/ExperimentalStart.dir/clean:
	cd /d C:\Users\kenforew\cppcode\build && $(CMAKE_COMMAND) -P CMakeFiles\ExperimentalStart.dir\cmake_clean.cmake
.PHONY : CMakeFiles/ExperimentalStart.dir/clean

CMakeFiles/ExperimentalStart.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\kenforew\cppcode C:\Users\kenforew\cppcode C:\Users\kenforew\cppcode\build C:\Users\kenforew\cppcode\build C:\Users\kenforew\cppcode\build\CMakeFiles\ExperimentalStart.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ExperimentalStart.dir/depend

