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
CMAKE_SOURCE_DIR = /home/yuzy/workspace/learnCpp/myProject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuzy/workspace/learnCpp/myProject/build

# Include any dependencies generated for this target.
include controller/library/CMakeFiles/robot_kinematics.dir/depend.make

# Include the progress variables for this target.
include controller/library/CMakeFiles/robot_kinematics.dir/progress.make

# Include the compile flags for this target's objects.
include controller/library/CMakeFiles/robot_kinematics.dir/flags.make

controller/library/CMakeFiles/robot_kinematics.dir/kinematics.cpp.o: controller/library/CMakeFiles/robot_kinematics.dir/flags.make
controller/library/CMakeFiles/robot_kinematics.dir/kinematics.cpp.o: ../controller/library/kinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuzy/workspace/learnCpp/myProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object controller/library/CMakeFiles/robot_kinematics.dir/kinematics.cpp.o"
	cd /home/yuzy/workspace/learnCpp/myProject/build/controller/library && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robot_kinematics.dir/kinematics.cpp.o -c /home/yuzy/workspace/learnCpp/myProject/controller/library/kinematics.cpp

controller/library/CMakeFiles/robot_kinematics.dir/kinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot_kinematics.dir/kinematics.cpp.i"
	cd /home/yuzy/workspace/learnCpp/myProject/build/controller/library && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuzy/workspace/learnCpp/myProject/controller/library/kinematics.cpp > CMakeFiles/robot_kinematics.dir/kinematics.cpp.i

controller/library/CMakeFiles/robot_kinematics.dir/kinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot_kinematics.dir/kinematics.cpp.s"
	cd /home/yuzy/workspace/learnCpp/myProject/build/controller/library && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuzy/workspace/learnCpp/myProject/controller/library/kinematics.cpp -o CMakeFiles/robot_kinematics.dir/kinematics.cpp.s

# Object files for target robot_kinematics
robot_kinematics_OBJECTS = \
"CMakeFiles/robot_kinematics.dir/kinematics.cpp.o"

# External object files for target robot_kinematics
robot_kinematics_EXTERNAL_OBJECTS =

controller/library/librobot_kinematics.a: controller/library/CMakeFiles/robot_kinematics.dir/kinematics.cpp.o
controller/library/librobot_kinematics.a: controller/library/CMakeFiles/robot_kinematics.dir/build.make
controller/library/librobot_kinematics.a: controller/library/CMakeFiles/robot_kinematics.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yuzy/workspace/learnCpp/myProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library librobot_kinematics.a"
	cd /home/yuzy/workspace/learnCpp/myProject/build/controller/library && $(CMAKE_COMMAND) -P CMakeFiles/robot_kinematics.dir/cmake_clean_target.cmake
	cd /home/yuzy/workspace/learnCpp/myProject/build/controller/library && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robot_kinematics.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
controller/library/CMakeFiles/robot_kinematics.dir/build: controller/library/librobot_kinematics.a

.PHONY : controller/library/CMakeFiles/robot_kinematics.dir/build

controller/library/CMakeFiles/robot_kinematics.dir/clean:
	cd /home/yuzy/workspace/learnCpp/myProject/build/controller/library && $(CMAKE_COMMAND) -P CMakeFiles/robot_kinematics.dir/cmake_clean.cmake
.PHONY : controller/library/CMakeFiles/robot_kinematics.dir/clean

controller/library/CMakeFiles/robot_kinematics.dir/depend:
	cd /home/yuzy/workspace/learnCpp/myProject/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuzy/workspace/learnCpp/myProject /home/yuzy/workspace/learnCpp/myProject/controller/library /home/yuzy/workspace/learnCpp/myProject/build /home/yuzy/workspace/learnCpp/myProject/build/controller/library /home/yuzy/workspace/learnCpp/myProject/build/controller/library/CMakeFiles/robot_kinematics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controller/library/CMakeFiles/robot_kinematics.dir/depend

