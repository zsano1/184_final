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
CMAKE_SOURCE_DIR = /home/shenao/homework/cs184/p4-clothsim-zsano1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug

# Include any dependencies generated for this target.
include ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/depend.make

# Include the progress variables for this target.
include ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/progress.make

# Include the compile flags for this target's objects.
include ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o: ../ext/nanogui/ext/glfw/src/context.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/context.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/context.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/context.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/context.c > CMakeFiles/glfw_objects.dir/context.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/context.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/context.c -o CMakeFiles/glfw_objects.dir/context.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o: ../ext/nanogui/ext/glfw/src/init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/init.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/init.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/init.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/init.c > CMakeFiles/glfw_objects.dir/init.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/init.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/init.c -o CMakeFiles/glfw_objects.dir/init.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o: ../ext/nanogui/ext/glfw/src/input.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/input.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/input.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/input.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/input.c > CMakeFiles/glfw_objects.dir/input.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/input.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/input.c -o CMakeFiles/glfw_objects.dir/input.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o: ../ext/nanogui/ext/glfw/src/monitor.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/monitor.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/monitor.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/monitor.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/monitor.c > CMakeFiles/glfw_objects.dir/monitor.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/monitor.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/monitor.c -o CMakeFiles/glfw_objects.dir/monitor.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o: ../ext/nanogui/ext/glfw/src/vulkan.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/vulkan.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/vulkan.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/vulkan.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/vulkan.c > CMakeFiles/glfw_objects.dir/vulkan.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/vulkan.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/vulkan.c -o CMakeFiles/glfw_objects.dir/vulkan.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o: ../ext/nanogui/ext/glfw/src/window.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/window.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/window.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/window.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/window.c > CMakeFiles/glfw_objects.dir/window.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/window.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/window.c -o CMakeFiles/glfw_objects.dir/window.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_init.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_init.c.o: ../ext/nanogui/ext/glfw/src/x11_init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_init.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/x11_init.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/x11_init.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/x11_init.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/x11_init.c > CMakeFiles/glfw_objects.dir/x11_init.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/x11_init.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/x11_init.c -o CMakeFiles/glfw_objects.dir/x11_init.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_monitor.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_monitor.c.o: ../ext/nanogui/ext/glfw/src/x11_monitor.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_monitor.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/x11_monitor.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/x11_monitor.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_monitor.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/x11_monitor.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/x11_monitor.c > CMakeFiles/glfw_objects.dir/x11_monitor.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_monitor.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/x11_monitor.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/x11_monitor.c -o CMakeFiles/glfw_objects.dir/x11_monitor.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_window.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_window.c.o: ../ext/nanogui/ext/glfw/src/x11_window.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_window.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/x11_window.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/x11_window.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_window.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/x11_window.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/x11_window.c > CMakeFiles/glfw_objects.dir/x11_window.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_window.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/x11_window.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/x11_window.c -o CMakeFiles/glfw_objects.dir/x11_window.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/xkb_unicode.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/xkb_unicode.c.o: ../ext/nanogui/ext/glfw/src/xkb_unicode.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/xkb_unicode.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/xkb_unicode.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/xkb_unicode.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/xkb_unicode.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/xkb_unicode.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/xkb_unicode.c > CMakeFiles/glfw_objects.dir/xkb_unicode.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/xkb_unicode.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/xkb_unicode.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/xkb_unicode.c -o CMakeFiles/glfw_objects.dir/xkb_unicode.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/linux_joystick.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/linux_joystick.c.o: ../ext/nanogui/ext/glfw/src/linux_joystick.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/linux_joystick.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/linux_joystick.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/linux_joystick.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/linux_joystick.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/linux_joystick.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/linux_joystick.c > CMakeFiles/glfw_objects.dir/linux_joystick.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/linux_joystick.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/linux_joystick.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/linux_joystick.c -o CMakeFiles/glfw_objects.dir/linux_joystick.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_time.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_time.c.o: ../ext/nanogui/ext/glfw/src/posix_time.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_time.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/posix_time.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/posix_time.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_time.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/posix_time.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/posix_time.c > CMakeFiles/glfw_objects.dir/posix_time.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_time.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/posix_time.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/posix_time.c -o CMakeFiles/glfw_objects.dir/posix_time.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o: ../ext/nanogui/ext/glfw/src/posix_tls.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/posix_tls.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/posix_tls.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/posix_tls.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/posix_tls.c > CMakeFiles/glfw_objects.dir/posix_tls.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/posix_tls.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/posix_tls.c -o CMakeFiles/glfw_objects.dir/posix_tls.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/glx_context.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/glx_context.c.o: ../ext/nanogui/ext/glfw/src/glx_context.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/glx_context.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/glx_context.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/glx_context.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/glx_context.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/glx_context.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/glx_context.c > CMakeFiles/glfw_objects.dir/glx_context.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/glx_context.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/glx_context.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/glx_context.c -o CMakeFiles/glfw_objects.dir/glx_context.c.s

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/egl_context.c.o: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/egl_context.c.o: ../ext/nanogui/ext/glfw/src/egl_context.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building C object ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/egl_context.c.o"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/egl_context.c.o   -c /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/egl_context.c

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/egl_context.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/egl_context.c.i"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/egl_context.c > CMakeFiles/glfw_objects.dir/egl_context.c.i

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/egl_context.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/egl_context.c.s"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src/egl_context.c -o CMakeFiles/glfw_objects.dir/egl_context.c.s

glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_init.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_monitor.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_window.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/xkb_unicode.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/linux_joystick.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_time.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/glx_context.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/egl_context.c.o
glfw_objects: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/build.make

.PHONY : glfw_objects

# Rule to build all files generated by this target.
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/build: glfw_objects

.PHONY : ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/build

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/clean:
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && $(CMAKE_COMMAND) -P CMakeFiles/glfw_objects.dir/cmake_clean.cmake
.PHONY : ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/clean

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/depend:
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shenao/homework/cs184/p4-clothsim-zsano1 /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/depend

