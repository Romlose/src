cmake_minimum_required(VERSION 3.0.2)
project(kuka_controllers)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  geometry_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  kdl_conversions
  sensor_msgs
  std_msgs
  tf_conversions
  pluginlib 
  orocos_kdl
  gazebo_msgs
  class_loader
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

add_message_files(
  FILES
  KukaState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    geometry_msgs
    gazebo_msgs
    control_toolbox
    realtime_tools
    urdf
    forward_command_controller
    kdl_parser
    message_runtime
    roscpp
    pluginlib
    class_loader
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/kuka_inverse_dynamic_controller.cpp
  src/kuka_kuka_state_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES plugins/kuka_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_kuka_state_controller test/test_controller.cpp)

  target_link_libraries(test_kuka_state_controller
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
endif()