cmake_minimum_required(VERSION 3.10)
project(vicon_env)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  pcl_conversions
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)


# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Point2d.msg
  Point3d.msg
  Ellipse.msg
  Circle.msg
  Polygon.msg
  Cylinder.msg
  Polyhedron.msg
  SemanticArray.msg
  )
 
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  )


catkin_package(
  CATKIN_DEPENDS message_runtime
  DEPENDS
  )
  
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable (vicon_map2d src/vicon_map2d.cpp )
target_link_libraries(vicon_map2d
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}) 
add_dependencies(vicon_map2d 
  ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable (vicon_map3d src/vicon_map3d.cpp )
target_link_libraries(vicon_map3d
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}) 
add_dependencies(vicon_map3d 
   ${${PROJECT_NAME}_EXPORTED_TARGETS})
