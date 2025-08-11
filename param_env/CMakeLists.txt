cmake_minimum_required(VERSION 3.10)
project(param_env)

set(CMAKE_CXX_STANDARD 17)

# ROS 2 core
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# External libs
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

include_directories(
  SYSTEM
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIRS}
  ${SDL_IMAGE_INCLUDE_DIRS}
)

# Executable: structure_map
add_executable(structure_map src/structure_map.cpp)
ament_target_dependencies(structure_map
  rclcpp
  std_msgs
  geometry_msgs
  pcl_conversions
  nav_msgs
  visualization_msgs
  rosbag2_cpp
)
target_link_libraries(structure_map
  ${PCL_LIBRARIES}
)

# Executable: read_grid_map
add_executable(read_grid_map src/read_grid_map.cpp)
ament_target_dependencies(read_grid_map
  rclcpp
  std_msgs
  geometry_msgs
  pcl_conversions
  nav_msgs
  visualization_msgs
  rosbag2_cpp
)
target_link_libraries(read_grid_map
  ${PCL_LIBRARIES}
  ${SDL_LIBRARY}
  ${SDL_IMAGE_LIBRARIES}
)

install(TARGETS
  structure_map
  read_grid_map
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY utils/
  DESTINATION share/${PROJECT_NAME}/utils
)

# Export package
ament_export_include_directories(include)
ament_export_libraries(${PCL_LIBRARIES})

ament_package()
