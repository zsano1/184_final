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
include ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/depend.make

# Include the progress variables for this target.
include ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/progress.make

# Include the compile flags for this target's objects.
include ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/flags.make

# Object files for target glfw
glfw_OBJECTS =

# External object files for target glfw
glfw_EXTERNAL_OBJECTS = \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_init.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_monitor.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_window.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/xkb_unicode.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/linux_joystick.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_time.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/glx_context.c.o" \
"/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/egl_context.c.o"

ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_init.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_monitor.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/x11_window.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/xkb_unicode.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/linux_joystick.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_time.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/glx_context.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/egl_context.c.o
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/build.make
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: /usr/lib/x86_64-linux-gnu/librt.so
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: /usr/lib/x86_64-linux-gnu/libm.so
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: /usr/lib/x86_64-linux-gnu/libX11.so
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: /usr/lib/x86_64-linux-gnu/libXrandr.so
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: /usr/lib/x86_64-linux-gnu/libXinerama.so
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: /usr/lib/x86_64-linux-gnu/libXxf86vm.so
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: /usr/lib/x86_64-linux-gnu/libXcursor.so
ext/nanogui/ext_build/glfw/src/libglfw.so.3.2: ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Linking C shared library libglfw.so"
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glfw.dir/link.txt --verbose=$(VERBOSE)
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && $(CMAKE_COMMAND) -E cmake_symlink_library libglfw.so.3.2 libglfw.so.3 libglfw.so

ext/nanogui/ext_build/glfw/src/libglfw.so.3: ext/nanogui/ext_build/glfw/src/libglfw.so.3.2
	@$(CMAKE_COMMAND) -E touch_nocreate ext/nanogui/ext_build/glfw/src/libglfw.so.3

ext/nanogui/ext_build/glfw/src/libglfw.so: ext/nanogui/ext_build/glfw/src/libglfw.so.3.2
	@$(CMAKE_COMMAND) -E touch_nocreate ext/nanogui/ext_build/glfw/src/libglfw.so

# Rule to build all files generated by this target.
ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/build: ext/nanogui/ext_build/glfw/src/libglfw.so

.PHONY : ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/build

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/clean:
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src && $(CMAKE_COMMAND) -P CMakeFiles/glfw.dir/cmake_clean.cmake
.PHONY : ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/clean

ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/depend:
	cd /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shenao/homework/cs184/p4-clothsim-zsano1 /home/shenao/homework/cs184/p4-clothsim-zsano1/ext/nanogui/ext/glfw/src /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src /home/shenao/homework/cs184/p4-clothsim-zsano1/cmake-build-debug/ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ext/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/depend

