# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/yuzy/workspace/myProject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuzy/workspace/myProject/build

# Include any dependencies generated for this target.
include controller/gait/CMakeFiles/gait.dir/depend.make

# Include the progress variables for this target.
include controller/gait/CMakeFiles/gait.dir/progress.make

# Include the compile flags for this target's objects.
include controller/gait/CMakeFiles/gait.dir/flags.make

controller/gait/CMakeFiles/gait.dir/gait.cpp.o: controller/gait/CMakeFiles/gait.dir/flags.make
controller/gait/CMakeFiles/gait.dir/gait.cpp.o: ../controller/gait/gait.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuzy/workspace/myProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object controller/gait/CMakeFiles/gait.dir/gait.cpp.o"
	cd /home/yuzy/workspace/myProject/build/controller/gait && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gait.dir/gait.cpp.o -c /home/yuzy/workspace/myProject/controller/gait/gait.cpp

controller/gait/CMakeFiles/gait.dir/gait.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gait.dir/gait.cpp.i"
	cd /home/yuzy/workspace/myProject/build/controller/gait && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuzy/workspace/myProject/controller/gait/gait.cpp > CMakeFiles/gait.dir/gait.cpp.i

controller/gait/CMakeFiles/gait.dir/gait.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gait.dir/gait.cpp.s"
	cd /home/yuzy/workspace/myProject/build/controller/gait && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuzy/workspace/myProject/controller/gait/gait.cpp -o CMakeFiles/gait.dir/gait.cpp.s

# Object files for target gait
gait_OBJECTS = \
"CMakeFiles/gait.dir/gait.cpp.o"

# External object files for target gait
gait_EXTERNAL_OBJECTS =

controller/gait/libgait.a: controller/gait/CMakeFiles/gait.dir/gait.cpp.o
controller/gait/libgait.a: controller/gait/CMakeFiles/gait.dir/build.make
controller/gait/libgait.a: controller/gait/CMakeFiles/gait.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yuzy/workspace/myProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libgait.a"
	cd /home/yuzy/workspace/myProject/build/controller/gait && $(CMAKE_COMMAND) -P CMakeFiles/gait.dir/cmake_clean_target.cmake
	cd /home/yuzy/workspace/myProject/build/controller/gait && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gait.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
controller/gait/CMakeFiles/gait.dir/build: controller/gait/libgait.a

.PHONY : controller/gait/CMakeFiles/gait.dir/build

controller/gait/CMakeFiles/gait.dir/clean:
	cd /home/yuzy/workspace/myProject/build/controller/gait && $(CMAKE_COMMAND) -P CMakeFiles/gait.dir/cmake_clean.cmake
.PHONY : controller/gait/CMakeFiles/gait.dir/clean

controller/gait/CMakeFiles/gait.dir/depend:
	cd /home/yuzy/workspace/myProject/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuzy/workspace/myProject /home/yuzy/workspace/myProject/controller/gait /home/yuzy/workspace/myProject/build /home/yuzy/workspace/myProject/build/controller/gait /home/yuzy/workspace/myProject/build/controller/gait/CMakeFiles/gait.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controller/gait/CMakeFiles/gait.dir/depend

