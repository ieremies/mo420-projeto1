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
include tools/CMakeFiles/lgf-gen.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include tools/CMakeFiles/lgf-gen.dir/compiler_depend.make

# Include the progress variables for this target.
include tools/CMakeFiles/lgf-gen.dir/progress.make

# Include the compile flags for this target's objects.
include tools/CMakeFiles/lgf-gen.dir/flags.make

tools/CMakeFiles/lgf-gen.dir/lgf-gen.cc.o: tools/CMakeFiles/lgf-gen.dir/flags.make
tools/CMakeFiles/lgf-gen.dir/lgf-gen.cc.o: tools/lgf-gen.cc
tools/CMakeFiles/lgf-gen.dir/lgf-gen.cc.o: tools/CMakeFiles/lgf-gen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tools/CMakeFiles/lgf-gen.dir/lgf-gen.cc.o"
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tools/CMakeFiles/lgf-gen.dir/lgf-gen.cc.o -MF CMakeFiles/lgf-gen.dir/lgf-gen.cc.o.d -o CMakeFiles/lgf-gen.dir/lgf-gen.cc.o -c /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools/lgf-gen.cc

tools/CMakeFiles/lgf-gen.dir/lgf-gen.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lgf-gen.dir/lgf-gen.cc.i"
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools/lgf-gen.cc > CMakeFiles/lgf-gen.dir/lgf-gen.cc.i

tools/CMakeFiles/lgf-gen.dir/lgf-gen.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lgf-gen.dir/lgf-gen.cc.s"
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools/lgf-gen.cc -o CMakeFiles/lgf-gen.dir/lgf-gen.cc.s

# Object files for target lgf-gen
lgf__gen_OBJECTS = \
"CMakeFiles/lgf-gen.dir/lgf-gen.cc.o"

# External object files for target lgf-gen
lgf__gen_EXTERNAL_OBJECTS =

tools/lgf-gen: tools/CMakeFiles/lgf-gen.dir/lgf-gen.cc.o
tools/lgf-gen: tools/CMakeFiles/lgf-gen.dir/build.make
tools/lgf-gen: lemon/libemon.a
tools/lgf-gen: tools/CMakeFiles/lgf-gen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable lgf-gen"
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lgf-gen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tools/CMakeFiles/lgf-gen.dir/build: tools/lgf-gen
.PHONY : tools/CMakeFiles/lgf-gen.dir/build

tools/CMakeFiles/lgf-gen.dir/clean:
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools && $(CMAKE_COMMAND) -P CMakeFiles/lgf-gen.dir/cmake_clean.cmake
.PHONY : tools/CMakeFiles/lgf-gen.dir/clean

tools/CMakeFiles/lgf-gen.dir/depend:
	cd /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1 /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1 /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools /home/ieremies/proj/grad/pli/lemon/lemon-1.3.1/tools/CMakeFiles/lgf-gen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tools/CMakeFiles/lgf-gen.dir/depend

