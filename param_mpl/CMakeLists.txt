cmake_minimum_required(VERSION 2.8.3)
project(param_mpl)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(motion_primitive_library REQUIRED)
find_package(decomp_util REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
  decomp_ros_msgs
  decomp_ros_utils
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES param_mpl
 CATKIN_DEPENDS decomp_ros_msgs decomp_ros_utils
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# add_executable (param_mpl src/map_planner_node.cpp)
# target_link_libraries(param_mpl
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES})

