cmake_minimum_required(VERSION 3.8)
project(gantry_controller)

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
find_package(trajectory_msgs REQUIRED)

# ────────────────────────────── libmodbus ─────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# ────────────────────────────── Executable ────────────────────────────────────
add_executable(modbus_hw_bridge
  src/modbus_hw_bridge.cpp
)
add_executable(gantry_demo src/gantry_demo.cpp)
ament_target_dependencies(gantry_demo
  rclcpp
  sensor_msgs
)
target_include_directories(modbus_hw_bridge
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${LIBMODBUS_INCLUDE_DIRS}
)

target_link_libraries(modbus_hw_bridge
  ${LIBMODBUS_LIBRARIES}
)

ament_target_dependencies(modbus_hw_bridge
  rclcpp
  sensor_msgs
  trajectory_msgs
)

# ────────────────────────────── Install ───────────────────────────────────────
install(TARGETS modbus_hw_bridge gantry_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ────────────────────────────── Linting (optional) ────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
