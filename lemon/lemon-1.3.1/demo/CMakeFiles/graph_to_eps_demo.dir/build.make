# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1

# Include any dependencies generated for this target.
include demo/CMakeFiles/graph_to_eps_demo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include demo/CMakeFiles/graph_to_eps_demo.dir/compiler_depend.make

# Include the progress variables for this target.
include demo/CMakeFiles/graph_to_eps_demo.dir/progress.make

# Include the compile flags for this target's objects.
include demo/CMakeFiles/graph_to_eps_demo.dir/flags.make

demo/CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.o: demo/CMakeFiles/graph_to_eps_demo.dir/flags.make
demo/CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.o: demo/graph_to_eps_demo.cc
demo/CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.o: demo/CMakeFiles/graph_to_eps_demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object demo/CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.o"
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT demo/CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.o -MF CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.o.d -o CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.o -c /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo/graph_to_eps_demo.cc

demo/CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.i"
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo/graph_to_eps_demo.cc > CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.i

demo/CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.s"
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo/graph_to_eps_demo.cc -o CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.s

# Object files for target graph_to_eps_demo
graph_to_eps_demo_OBJECTS = \
"CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.o"

# External object files for target graph_to_eps_demo
graph_to_eps_demo_EXTERNAL_OBJECTS =

demo/graph_to_eps_demo: demo/CMakeFiles/graph_to_eps_demo.dir/graph_to_eps_demo.cc.o
demo/graph_to_eps_demo: demo/CMakeFiles/graph_to_eps_demo.dir/build.make
demo/graph_to_eps_demo: lemon/libemon.a
demo/graph_to_eps_demo: demo/CMakeFiles/graph_to_eps_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable graph_to_eps_demo"
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/graph_to_eps_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
demo/CMakeFiles/graph_to_eps_demo.dir/build: demo/graph_to_eps_demo
.PHONY : demo/CMakeFiles/graph_to_eps_demo.dir/build

demo/CMakeFiles/graph_to_eps_demo.dir/clean:
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo && $(CMAKE_COMMAND) -P CMakeFiles/graph_to_eps_demo.dir/cmake_clean.cmake
.PHONY : demo/CMakeFiles/graph_to_eps_demo.dir/clean

demo/CMakeFiles/graph_to_eps_demo.dir/depend:
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1 /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1 /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/demo/CMakeFiles/graph_to_eps_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : demo/CMakeFiles/graph_to_eps_demo.dir/depend

