cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
rosbuild_add_boost_directories()

# include cmake file for MIRA and make sure cmake can parse booleans that come
# as strings, e.g. "true" and "false". This worked in previous versions of cmake.
cmake_policy(PUSH)
cmake_policy(SET CMP0012 NEW)
include(CMakeLists.mira)
cmake_policy(POP)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

mira_require_package(RobotDataTypes)
mira_require_package(Navigation)
mira_require_package(Maps)

#uncomment if you have defined messages
rosbuild_genmsg()

#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable(cognidrive_ros
  src/cognidriveros.cpp
  src/dummydrive.cpp
  src/main.cpp
  src/movebaseaction.cpp
)

target_link_libraries(cognidrive_ros RobotDataTypes Navigation)
rosbuild_link_boost(cognidrive_ros thread)
rosbuild_add_mira(cognidrive_ros)