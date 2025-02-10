cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)  # Should match package.xml name

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ament build system and dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)       # For PX4 messages
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Vision utilities library
add_library(vision_utils 
  src/vision_utils.cpp
)
target_include_directories(vision_utils PUBLIC
  include
  ${Eigen3_INCLUDE_DIRS}  # Header-only Eigen
)
target_link_libraries(vision_utils
  ${OpenCV_LIBS}           # Link against OpenCV libraries
  Eigen3::Eigen
)

# Offboard control node
add_executable(offb_node src/offb_node.cpp)
target_include_directories(offb_node PRIVATE include)
ament_target_dependencies(offb_node
  rclcpp
  std_msgs
  px4_msgs
)
target_link_libraries(offb_node
  vision_utils
)

# Image listener node
add_executable(save_image_listener src/save_image_listener.cpp)
ament_target_dependencies(save_image_listener
  rclcpp
  image_transport
  cv_bridge
)
target_link_libraries(save_image_listener
  ${OpenCV_LIBS}           # Link against OpenCV libraries
)

# PBVS control node
add_executable(pbvs_node src/pbvs_node.cpp)
target_include_directories(pbvs_node PRIVATE include)
ament_target_dependencies(pbvs_node
  rclcpp
  std_msgs
  px4_msgs
)
target_link_libraries(pbvs_node
  vision_utils
)

# Installation
install(TARGETS
  vision_utils
  offb_node
  save_image_listener
  pbvs_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies and libraries
ament_export_dependencies(
  rclcpp
  std_msgs
  px4_msgs
  cv_bridge
  image_transport
  OpenCV
  Eigen3
)
ament_export_libraries(vision_utils)

ament_package()
