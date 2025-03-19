cmake_minimum_required(VERSION 3.0.2)
project(kdl_dynamics)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  kdl_parser
  std_msgs
  geometry_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
find_package(orocos_kdl REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kdl_kuka_node src/kdl_kuka_node.cpp)
add_dependencies(kdl_kuka_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(kdl_kuka_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
