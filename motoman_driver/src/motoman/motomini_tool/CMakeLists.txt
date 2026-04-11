cmake_minimum_required(VERSION 3.0.2)
project(motomini_tool)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  industrial_msgs
  motoman_msgs
  roscpp
  sensor_msgs
  std_srvs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib control_msgs industrial_msgs motoman_msgs roscpp sensor_msgs std_srvs trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fake_motomini_node src/fake_motomini_node.cpp)
target_link_libraries(fake_motomini_node ${catkin_LIBRARIES})
add_dependencies(fake_motomini_node ${catkin_EXPORTED_TARGETS})

add_executable(tool_service_node src/tool_service_node.cpp)
target_link_libraries(tool_service_node ${catkin_LIBRARIES})
add_dependencies(tool_service_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  fake_motomini_node
  tool_service_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)