cmake_minimum_required(VERSION 2.8.3)
project(pcl-algorithms)

find_package(catkin REQUIRED COMPONENTS pcl_msgs roscpp sensor_msgs std_msgs)

find_package(PCL REQUIRED QUIET COMPONENTS common)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_COMMON_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS ${PCL_COMMON_INCLUDE_DIRS}
  CATKIN_DEPENDS pcl_msgs roscpp sensor_msgs std_msgs
)
find_package(VTK 7.1.1 REQUIRED COMPONENTS vtkCommonCore vtkCommonDataModel)
add_executable(plane-segmentation scripts/plane-segmentation.cpp)
target_link_libraries(plane-segmentation ${catkin_LIBRARIES})
add_dependencies(plane-segmentation pcl-algorithms_generate_messages_cpp)

# Add gtest based cpp test target
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test scripts/plane-segmentation.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()