# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/shenao/下载/clion-2018.3.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/shenao/下载/clion-2018.3.3/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/shenao/osm-bundler/p4-clothsim-zsano1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug

# Include any dependencies generated for this target.
include src/CMakeFiles/clothsim.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/clothsim.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/clothsim.dir/flags.make

src/CMakeFiles/clothsim.dir/cloth.cpp.o: src/CMakeFiles/clothsim.dir/flags.make
src/CMakeFiles/clothsim.dir/cloth.cpp.o: ../src/cloth.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/clothsim.dir/cloth.cpp.o"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clothsim.dir/cloth.cpp.o -c /home/shenao/osm-bundler/p4-clothsim-zsano1/src/cloth.cpp

src/CMakeFiles/clothsim.dir/cloth.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clothsim.dir/cloth.cpp.i"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shenao/osm-bundler/p4-clothsim-zsano1/src/cloth.cpp > CMakeFiles/clothsim.dir/cloth.cpp.i

src/CMakeFiles/clothsim.dir/cloth.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clothsim.dir/cloth.cpp.s"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shenao/osm-bundler/p4-clothsim-zsano1/src/cloth.cpp -o CMakeFiles/clothsim.dir/cloth.cpp.s

src/CMakeFiles/clothsim.dir/clothMesh.cpp.o: src/CMakeFiles/clothsim.dir/flags.make
src/CMakeFiles/clothsim.dir/clothMesh.cpp.o: ../src/clothMesh.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/clothsim.dir/clothMesh.cpp.o"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clothsim.dir/clothMesh.cpp.o -c /home/shenao/osm-bundler/p4-clothsim-zsano1/src/clothMesh.cpp

src/CMakeFiles/clothsim.dir/clothMesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clothsim.dir/clothMesh.cpp.i"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shenao/osm-bundler/p4-clothsim-zsano1/src/clothMesh.cpp > CMakeFiles/clothsim.dir/clothMesh.cpp.i

src/CMakeFiles/clothsim.dir/clothMesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clothsim.dir/clothMesh.cpp.s"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shenao/osm-bundler/p4-clothsim-zsano1/src/clothMesh.cpp -o CMakeFiles/clothsim.dir/clothMesh.cpp.s

src/CMakeFiles/clothsim.dir/collision/sphere.cpp.o: src/CMakeFiles/clothsim.dir/flags.make
src/CMakeFiles/clothsim.dir/collision/sphere.cpp.o: ../src/collision/sphere.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/clothsim.dir/collision/sphere.cpp.o"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clothsim.dir/collision/sphere.cpp.o -c /home/shenao/osm-bundler/p4-clothsim-zsano1/src/collision/sphere.cpp

src/CMakeFiles/clothsim.dir/collision/sphere.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clothsim.dir/collision/sphere.cpp.i"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shenao/osm-bundler/p4-clothsim-zsano1/src/collision/sphere.cpp > CMakeFiles/clothsim.dir/collision/sphere.cpp.i

src/CMakeFiles/clothsim.dir/collision/sphere.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clothsim.dir/collision/sphere.cpp.s"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shenao/osm-bundler/p4-clothsim-zsano1/src/collision/sphere.cpp -o CMakeFiles/clothsim.dir/collision/sphere.cpp.s

src/CMakeFiles/clothsim.dir/collision/plane.cpp.o: src/CMakeFiles/clothsim.dir/flags.make
src/CMakeFiles/clothsim.dir/collision/plane.cpp.o: ../src/collision/plane.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/clothsim.dir/collision/plane.cpp.o"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clothsim.dir/collision/plane.cpp.o -c /home/shenao/osm-bundler/p4-clothsim-zsano1/src/collision/plane.cpp

src/CMakeFiles/clothsim.dir/collision/plane.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clothsim.dir/collision/plane.cpp.i"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shenao/osm-bundler/p4-clothsim-zsano1/src/collision/plane.cpp > CMakeFiles/clothsim.dir/collision/plane.cpp.i

src/CMakeFiles/clothsim.dir/collision/plane.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clothsim.dir/collision/plane.cpp.s"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shenao/osm-bundler/p4-clothsim-zsano1/src/collision/plane.cpp -o CMakeFiles/clothsim.dir/collision/plane.cpp.s

src/CMakeFiles/clothsim.dir/main.cpp.o: src/CMakeFiles/clothsim.dir/flags.make
src/CMakeFiles/clothsim.dir/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/clothsim.dir/main.cpp.o"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clothsim.dir/main.cpp.o -c /home/shenao/osm-bundler/p4-clothsim-zsano1/src/main.cpp

src/CMakeFiles/clothsim.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clothsim.dir/main.cpp.i"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shenao/osm-bundler/p4-clothsim-zsano1/src/main.cpp > CMakeFiles/clothsim.dir/main.cpp.i

src/CMakeFiles/clothsim.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clothsim.dir/main.cpp.s"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shenao/osm-bundler/p4-clothsim-zsano1/src/main.cpp -o CMakeFiles/clothsim.dir/main.cpp.s

src/CMakeFiles/clothsim.dir/clothSimulator.cpp.o: src/CMakeFiles/clothsim.dir/flags.make
src/CMakeFiles/clothsim.dir/clothSimulator.cpp.o: ../src/clothSimulator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/clothsim.dir/clothSimulator.cpp.o"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clothsim.dir/clothSimulator.cpp.o -c /home/shenao/osm-bundler/p4-clothsim-zsano1/src/clothSimulator.cpp

src/CMakeFiles/clothsim.dir/clothSimulator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clothsim.dir/clothSimulator.cpp.i"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shenao/osm-bundler/p4-clothsim-zsano1/src/clothSimulator.cpp > CMakeFiles/clothsim.dir/clothSimulator.cpp.i

src/CMakeFiles/clothsim.dir/clothSimulator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clothsim.dir/clothSimulator.cpp.s"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shenao/osm-bundler/p4-clothsim-zsano1/src/clothSimulator.cpp -o CMakeFiles/clothsim.dir/clothSimulator.cpp.s

src/CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.o: src/CMakeFiles/clothsim.dir/flags.make
src/CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.o: ../src/misc/sphere_drawing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.o"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.o -c /home/shenao/osm-bundler/p4-clothsim-zsano1/src/misc/sphere_drawing.cpp

src/CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.i"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shenao/osm-bundler/p4-clothsim-zsano1/src/misc/sphere_drawing.cpp > CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.i

src/CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.s"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shenao/osm-bundler/p4-clothsim-zsano1/src/misc/sphere_drawing.cpp -o CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.s

src/CMakeFiles/clothsim.dir/misc/file_utils.cpp.o: src/CMakeFiles/clothsim.dir/flags.make
src/CMakeFiles/clothsim.dir/misc/file_utils.cpp.o: ../src/misc/file_utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/CMakeFiles/clothsim.dir/misc/file_utils.cpp.o"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clothsim.dir/misc/file_utils.cpp.o -c /home/shenao/osm-bundler/p4-clothsim-zsano1/src/misc/file_utils.cpp

src/CMakeFiles/clothsim.dir/misc/file_utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clothsim.dir/misc/file_utils.cpp.i"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shenao/osm-bundler/p4-clothsim-zsano1/src/misc/file_utils.cpp > CMakeFiles/clothsim.dir/misc/file_utils.cpp.i

src/CMakeFiles/clothsim.dir/misc/file_utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clothsim.dir/misc/file_utils.cpp.s"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shenao/osm-bundler/p4-clothsim-zsano1/src/misc/file_utils.cpp -o CMakeFiles/clothsim.dir/misc/file_utils.cpp.s

src/CMakeFiles/clothsim.dir/camera.cpp.o: src/CMakeFiles/clothsim.dir/flags.make
src/CMakeFiles/clothsim.dir/camera.cpp.o: ../src/camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/CMakeFiles/clothsim.dir/camera.cpp.o"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clothsim.dir/camera.cpp.o -c /home/shenao/osm-bundler/p4-clothsim-zsano1/src/camera.cpp

src/CMakeFiles/clothsim.dir/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clothsim.dir/camera.cpp.i"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shenao/osm-bundler/p4-clothsim-zsano1/src/camera.cpp > CMakeFiles/clothsim.dir/camera.cpp.i

src/CMakeFiles/clothsim.dir/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clothsim.dir/camera.cpp.s"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shenao/osm-bundler/p4-clothsim-zsano1/src/camera.cpp -o CMakeFiles/clothsim.dir/camera.cpp.s

# Object files for target clothsim
clothsim_OBJECTS = \
"CMakeFiles/clothsim.dir/cloth.cpp.o" \
"CMakeFiles/clothsim.dir/clothMesh.cpp.o" \
"CMakeFiles/clothsim.dir/collision/sphere.cpp.o" \
"CMakeFiles/clothsim.dir/collision/plane.cpp.o" \
"CMakeFiles/clothsim.dir/main.cpp.o" \
"CMakeFiles/clothsim.dir/clothSimulator.cpp.o" \
"CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.o" \
"CMakeFiles/clothsim.dir/misc/file_utils.cpp.o" \
"CMakeFiles/clothsim.dir/camera.cpp.o"

# External object files for target clothsim
clothsim_EXTERNAL_OBJECTS =

clothsim: src/CMakeFiles/clothsim.dir/cloth.cpp.o
clothsim: src/CMakeFiles/clothsim.dir/clothMesh.cpp.o
clothsim: src/CMakeFiles/clothsim.dir/collision/sphere.cpp.o
clothsim: src/CMakeFiles/clothsim.dir/collision/plane.cpp.o
clothsim: src/CMakeFiles/clothsim.dir/main.cpp.o
clothsim: src/CMakeFiles/clothsim.dir/clothSimulator.cpp.o
clothsim: src/CMakeFiles/clothsim.dir/misc/sphere_drawing.cpp.o
clothsim: src/CMakeFiles/clothsim.dir/misc/file_utils.cpp.o
clothsim: src/CMakeFiles/clothsim.dir/camera.cpp.o
clothsim: src/CMakeFiles/clothsim.dir/build.make
clothsim: CGL/src/libCGL.a
clothsim: ext/nanogui/libnanogui.so
clothsim: /usr/lib/x86_64-linux-gnu/libfreetype.so
clothsim: src/CMakeFiles/clothsim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX executable ../clothsim"
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/clothsim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/clothsim.dir/build: clothsim

.PHONY : src/CMakeFiles/clothsim.dir/build

src/CMakeFiles/clothsim.dir/clean:
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src && $(CMAKE_COMMAND) -P CMakeFiles/clothsim.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/clothsim.dir/clean

src/CMakeFiles/clothsim.dir/depend:
	cd /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shenao/osm-bundler/p4-clothsim-zsano1 /home/shenao/osm-bundler/p4-clothsim-zsano1/src /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src /home/shenao/osm-bundler/p4-clothsim-zsano1/cmake-build-debug/src/CMakeFiles/clothsim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/clothsim.dir/depend

