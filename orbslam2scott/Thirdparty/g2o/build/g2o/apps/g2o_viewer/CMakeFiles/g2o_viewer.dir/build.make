# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build

# Include any dependencies generated for this target.
include g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/depend.make

# Include the progress variables for this target.
include g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/progress.make

# Include the compile flags for this target's objects.
include g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/flags.make

g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o: g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/flags.make
g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o: ../g2o/apps/g2o_viewer/g2o_viewer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/apps/g2o_viewer && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o -c /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/apps/g2o_viewer/g2o_viewer.cpp

g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.i"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/apps/g2o_viewer && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/apps/g2o_viewer/g2o_viewer.cpp > CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.i

g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.s"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/apps/g2o_viewer && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/apps/g2o_viewer/g2o_viewer.cpp -o CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.s

g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o.requires:
.PHONY : g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o.requires

g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o.provides: g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o.requires
	$(MAKE) -f g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/build.make g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o.provides.build
.PHONY : g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o.provides

g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o.provides.build: g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o

# Object files for target g2o_viewer
g2o_viewer_OBJECTS = \
"CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o"

# External object files for target g2o_viewer
g2o_viewer_EXTERNAL_OBJECTS =

../bin/g2o_viewer: g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o
../bin/g2o_viewer: g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/build.make
../bin/g2o_viewer: ../lib/libg2o_viewer.so
../bin/g2o_viewer: ../lib/libg2o_cli.so
../bin/g2o_viewer: ../lib/libg2o_core.so
../bin/g2o_viewer: ../lib/libg2o_stuff.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libQGLViewer.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libQtOpenGL.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libQtGui.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libQtXml.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libQtCore.so
../bin/g2o_viewer: ../lib/libg2o_opengl_helper.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libSM.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libICE.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libX11.so
../bin/g2o_viewer: /usr/lib/x86_64-linux-gnu/libXext.so
../bin/g2o_viewer: g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../../../bin/g2o_viewer"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/apps/g2o_viewer && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/g2o_viewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/build: ../bin/g2o_viewer
.PHONY : g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/build

g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/requires: g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/g2o_viewer.cpp.o.requires
.PHONY : g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/requires

g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/clean:
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/apps/g2o_viewer && $(CMAKE_COMMAND) -P CMakeFiles/g2o_viewer.dir/cmake_clean.cmake
.PHONY : g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/clean

g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/depend:
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/apps/g2o_viewer /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/apps/g2o_viewer /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : g2o/apps/g2o_viewer/CMakeFiles/g2o_viewer.dir/depend

