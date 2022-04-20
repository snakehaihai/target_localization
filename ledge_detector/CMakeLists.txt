cmake_minimum_required(VERSION 2.8.3)
project(ledge_detector)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
  image_transport
  cv_bridge
  tf2
  OpenCV
  nav_msgs
  trajectory_msgs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ledge_detector src/ledge_detector.cpp)

target_link_libraries(ledge_detector ${catkin_LIBRARIES})
