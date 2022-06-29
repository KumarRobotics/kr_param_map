cmake_minimum_required(VERSION 2.8.3)
project(param_mpl)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES param_mpl
 CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

# add_library(param_mpl
#     )
# target_link_libraries(param_mpl
#     ${catkin_LIBRARIES} 
#     )  
