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
CMAKE_SOURCE_DIR = /home/pmore/Documents/Robotics/pumasim/1-code_template/1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pmore/Documents/Robotics/pumasim/1-code_template/1/controlDLL

# Include any dependencies generated for this target.
include CMakeFiles/controlDLL.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/controlDLL.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/controlDLL.dir/flags.make

CMakeFiles/controlDLL.dir/control.cpp.o: CMakeFiles/controlDLL.dir/flags.make
CMakeFiles/controlDLL.dir/control.cpp.o: control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pmore/Documents/Robotics/pumasim/1-code_template/1/controlDLL/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/controlDLL.dir/control.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controlDLL.dir/control.cpp.o -c /home/pmore/Documents/Robotics/pumasim/1-code_template/1/controlDLL/control.cpp

CMakeFiles/controlDLL.dir/control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controlDLL.dir/control.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pmore/Documents/Robotics/pumasim/1-code_template/1/controlDLL/control.cpp > CMakeFiles/controlDLL.dir/control.cpp.i

CMakeFiles/controlDLL.dir/control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controlDLL.dir/control.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pmore/Documents/Robotics/pumasim/1-code_template/1/controlDLL/control.cpp -o CMakeFiles/controlDLL.dir/control.cpp.s

# Object files for target controlDLL
controlDLL_OBJECTS = \
"CMakeFiles/controlDLL.dir/control.cpp.o"

# External object files for target controlDLL
controlDLL_EXTERNAL_OBJECTS =

libcontrolDLL.so: CMakeFiles/controlDLL.dir/control.cpp.o
libcontrolDLL.so: CMakeFiles/controlDLL.dir/build.make
libcontrolDLL.so: CMakeFiles/controlDLL.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pmore/Documents/Robotics/pumasim/1-code_template/1/controlDLL/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libcontrolDLL.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controlDLL.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/controlDLL.dir/build: libcontrolDLL.so

.PHONY : CMakeFiles/controlDLL.dir/build

CMakeFiles/controlDLL.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/controlDLL.dir/cmake_clean.cmake
.PHONY : CMakeFiles/controlDLL.dir/clean

CMakeFiles/controlDLL.dir/depend:
	cd /home/pmore/Documents/Robotics/pumasim/1-code_template/1/controlDLL && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pmore/Documents/Robotics/pumasim/1-code_template/1 /home/pmore/Documents/Robotics/pumasim/1-code_template/1 /home/pmore/Documents/Robotics/pumasim/1-code_template/1/controlDLL /home/pmore/Documents/Robotics/pumasim/1-code_template/1/controlDLL /home/pmore/Documents/Robotics/pumasim/1-code_template/1/controlDLL/CMakeFiles/controlDLL.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/controlDLL.dir/depend

