cmake_minimum_required(VERSION 3.0.2)
project(read_rosbag)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES read_rosbag
 CATKIN_DEPENDS nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
 DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_cut read_rosbag.cpp)
target_link_libraries(test_cut
  ${catkin_LIBRARIES}
)
