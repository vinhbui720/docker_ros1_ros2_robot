cmake_minimum_required(VERSION 3.8)
project(env_joint_states_publish)

# Require C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ────────────────────────────── ROS 2 deps ────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

# ────────────────────────────── Executables ──────────────────────────────────
add_executable(joint_states_merger
  src/joint_states_merger.cpp
)


target_include_directories(joint_states_merger
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(joint_states_merger
  rclcpp
  sensor_msgs
  message_filters
)


# ────────────────────────────── Install ───────────────────────────────────────
install(TARGETS joint_states_merger 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
