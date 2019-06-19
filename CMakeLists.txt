cmake_minimum_required(VERSION 2.8.3)
project(object_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
  geometry_msgs
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})


find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(object_loc_ROS src/object_loc_ROS.cpp)
target_link_libraries(object_loc_ROS ${catkin_LIBRARIES})
target_link_libraries(object_loc_ROS ${PCL_LIBRARIES})

add_executable (listenerCamera src/listenerCamera.cpp)
target_link_libraries (listenerCamera ${catkin_LIBRARIES})

add_executable (listenerBaseLink src/listenerBaseLink.cpp)
target_link_libraries (listenerBaseLink ${catkin_LIBRARIES})