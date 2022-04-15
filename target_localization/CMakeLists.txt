cmake_minimum_required(VERSION 2.8.3)
project(target_localization)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
  darknet_ros
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  bbox.msg
  bbox_array.msg
)


generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


add_executable(target_localization src/target_localization.cpp)

target_link_libraries(target_localization ${catkin_LIBRARIES} ${OpenCV_LIBS})
