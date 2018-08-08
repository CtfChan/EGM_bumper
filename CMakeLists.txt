cmake_minimum_required(VERSION 2.8.3)
project(EGM_class)

find_package(catkin REQUIRED COMPONENTS nodelet 
	roscpp 
	std_msgs 
	sensor_msgs
	pcl_conversions
  pcl_ros
  grid_map_core
	grid_map_ros
  grid_map_msgs
  grid_map_rviz_plugin
  grid_map_visualization
  tf_conversions
  zio_obstacle_msgs

)

## Setup include directories
include_directories(${catkin_INCLUDE_DIRS},
        src)

catkin_package(
  LIBRARIES EGM_class
  CATKIN_DEPENDS nodelet 
	roscpp 
	std_msgs 
	sensor_msgs
	pcl_conversions
  pcl_ros
  grid_map_core
	grid_map_ros
  grid_map_msgs
  grid_map_rviz_plugin
  grid_map_visualization
  tf_conversions
  zio_obstacle_msgs

)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")



## Create the nodelet tutorial library
add_library(EGM_class src/main.cpp)
target_link_libraries(EGM_class ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(EGM_class ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS EGM_class
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_definition.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
