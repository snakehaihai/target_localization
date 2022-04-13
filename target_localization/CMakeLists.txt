cmake_minimum_required(VERSION 2.8.3)
project(target_localization)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)
catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(target_localization src/target_localization.cpp)

target_link_libraries(target_localization ${catkin_LIBRARIES} ${OpenCV_LIBS})
