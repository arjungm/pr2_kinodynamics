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

#custom boost install
set(BOOST_ROOT "/home/agmenon/ros_workspace/code/boost_install")
set(Boost_NO_SYSTEM_PATHS TRUE)
set(Boost_NO_BOOST_CMAKE TRUE)
set(Boost_ADDITIONAL_VERSIONS "1.55" "1.55.0")
find_package(Boost 1.55.0 COMPONENTS thread system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#dynamics model library
rosbuild_add_library(dynamicsmodel src/dynamics_model.cpp)
target_link_libraries(dynamicsmodel ${Boost_LIBRARIES})

#rosbuild_add_executable(demo_forward_dynamics src/test/demo_forward_dynamics.cpp)
#target_link_libraries(demo_forward_dynamics dynamicsmodel ${Boost_LIBRARIES})
