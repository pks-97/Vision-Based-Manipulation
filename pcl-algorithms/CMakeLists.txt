cmake_minimum_required(VERSION 2.8.3)
project(pcl-algorithms)

find_package(catkin REQUIRED COMPONENTS pcl_msgs roscpp sensor_msgs std_msgs tf)

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
  INCLUDE_DIRS ${PCL_COMMON_INCLUDE_DIRS}
  CATKIN_DEPENDS pcl_msgs roscpp sensor_msgs std_msgs tf
)

add_executable(pcl_proc src/plane-segmentation.cpp)
add_dependencies(pcl_proc ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_proc ${PCL_LIBRARIES} ${catkin_LIBRARIES})
