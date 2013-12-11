cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_boost_directories()

#lib for actuating the gripper
rosbuild_add_library(gripper src/control/gripper_client.cpp)
rosbuild_link_boost( gripper thread filesystem)

#lib for actuating the torso lift
rosbuild_add_library(torso src/control/torso_client.cpp)
rosbuild_link_boost( torso thread filesystem)

#lib for utils in traj operations
rosbuild_add_library(dynutils src/utils/utils.cpp src/utils/trajectory_utils.cpp src/utils/eigen_utils.cpp)
rosbuild_link_boost( dynutils thread filesystem)

#robot libraries for running unfiltered trajectories
rosbuild_add_library(armtrajectory src/trajectory/ArmTrajectoryHandler.cpp)
rosbuild_link_boost(armtrajectory thread filesystem)
target_link_libraries(armtrajectory dynutils)

rosbuild_add_executable(demo_forward_dynamics src/test/demo_forward_dynamics.cpp)
rosbuild_link_boost(demo_forward_dynamics thread filesystem)
target_link_libraries(demo_forward_dynamics dynamicsmodel)

rosbuild_add_executable(run_trajectory src/trajectory/run_trajectory.cpp)
rosbuild_link_boost(run_trajectory thread filesystem)
target_link_libraries(run_trajectory torso gripper yaml-cpp armtrajectory)
