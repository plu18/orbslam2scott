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
include g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/depend.make

# Include the progress variables for this target.
include g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/progress.make

# Include the compile flags for this target's objects.
include g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/flags.make

g2o/examples/slam2d/ui_base_main_window.h: ../g2o/examples/slam2d/base_main_window.ui
	$(CMAKE_COMMAND) -E cmake_progress_report /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ui_base_main_window.h"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/lib/x86_64-linux-gnu/qt4/bin/uic -o /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d/ui_base_main_window.h /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/base_main_window.ui

g2o/examples/slam2d/moc_main_window.cxx: ../g2o/examples/slam2d/main_window.h
	$(CMAKE_COMMAND) -E cmake_progress_report /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating moc_main_window.cxx"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/lib/x86_64-linux-gnu/qt4/bin/moc @/home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d/moc_main_window.cxx_parameters

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/flags.make
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o: ../g2o/examples/slam2d/main_window.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/slam2d_g2o.dir/main_window.cpp.o -c /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/main_window.cpp

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam2d_g2o.dir/main_window.cpp.i"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/main_window.cpp > CMakeFiles/slam2d_g2o.dir/main_window.cpp.i

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam2d_g2o.dir/main_window.cpp.s"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/main_window.cpp -o CMakeFiles/slam2d_g2o.dir/main_window.cpp.s

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o.requires:
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o.requires

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o.provides: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o.requires
	$(MAKE) -f g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/build.make g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o.provides.build
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o.provides

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o.provides.build: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/flags.make
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o: ../g2o/examples/slam2d/slam2d_viewer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o -c /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/slam2d_viewer.cpp

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.i"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/slam2d_viewer.cpp > CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.i

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.s"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/slam2d_viewer.cpp -o CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.s

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o.requires:
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o.requires

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o.provides: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o.requires
	$(MAKE) -f g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/build.make g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o.provides.build
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o.provides

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o.provides.build: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/flags.make
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o: ../g2o/examples/slam2d/slam2d_g2o.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o -c /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/slam2d_g2o.cpp

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.i"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/slam2d_g2o.cpp > CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.i

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.s"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/slam2d_g2o.cpp -o CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.s

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o.requires:
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o.requires

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o.provides: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o.requires
	$(MAKE) -f g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/build.make g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o.provides.build
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o.provides

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o.provides.build: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/flags.make
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o: ../g2o/examples/slam2d/draw_helpers.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o -c /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/draw_helpers.cpp

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.i"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/draw_helpers.cpp > CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.i

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.s"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d/draw_helpers.cpp -o CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.s

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o.requires:
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o.requires

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o.provides: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o.requires
	$(MAKE) -f g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/build.make g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o.provides.build
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o.provides

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o.provides.build: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/flags.make
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o: g2o/examples/slam2d/moc_main_window.cxx
	$(CMAKE_COMMAND) -E cmake_progress_report /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o -c /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d/moc_main_window.cxx

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.i"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d/moc_main_window.cxx > CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.i

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.s"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d/moc_main_window.cxx -o CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.s

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o.requires:
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o.requires

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o.provides: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o.requires
	$(MAKE) -f g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/build.make g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o.provides.build
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o.provides

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o.provides.build: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o

# Object files for target slam2d_g2o
slam2d_g2o_OBJECTS = \
"CMakeFiles/slam2d_g2o.dir/main_window.cpp.o" \
"CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o" \
"CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o" \
"CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o" \
"CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o"

# External object files for target slam2d_g2o
slam2d_g2o_EXTERNAL_OBJECTS =

../bin/slam2d_g2o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o
../bin/slam2d_g2o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o
../bin/slam2d_g2o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o
../bin/slam2d_g2o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o
../bin/slam2d_g2o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o
../bin/slam2d_g2o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/build.make
../bin/slam2d_g2o: ../lib/libg2o_core.so
../bin/slam2d_g2o: ../lib/libg2o_solver_csparse.so
../bin/slam2d_g2o: ../lib/libg2o_types_slam2d.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libQGLViewer.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libQtXml.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libQtOpenGL.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libQtGui.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libQtCore.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/slam2d_g2o: ../lib/libg2o_csparse_extension.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../bin/slam2d_g2o: ../lib/libg2o_core.so
../bin/slam2d_g2o: ../lib/libg2o_stuff.so
../bin/slam2d_g2o: ../lib/libg2o_opengl_helper.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libSM.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libICE.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libX11.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libXext.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/slam2d_g2o: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/slam2d_g2o: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../../../bin/slam2d_g2o"
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/slam2d_g2o.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/build: ../bin/slam2d_g2o
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/build

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/requires: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/main_window.cpp.o.requires
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/requires: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_viewer.cpp.o.requires
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/requires: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/slam2d_g2o.cpp.o.requires
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/requires: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/draw_helpers.cpp.o.requires
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/requires: g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/moc_main_window.cxx.o.requires
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/requires

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/clean:
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d && $(CMAKE_COMMAND) -P CMakeFiles/slam2d_g2o.dir/cmake_clean.cmake
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/clean

g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/depend: g2o/examples/slam2d/ui_base_main_window.h
g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/depend: g2o/examples/slam2d/moc_main_window.cxx
	cd /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/g2o/examples/slam2d /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d /home/scott/catkin_ws/src/orb_slam_rgbd/ORB_SLAM2_modified/Thirdparty/g2o/build/g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : g2o/examples/slam2d/CMakeFiles/slam2d_g2o.dir/depend

