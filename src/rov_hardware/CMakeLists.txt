cmake_minimum_required(VERSION 3.8)
project(rov_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rov_msgs REQUIRED)
find_package(message_filters REQUIRED)

# ── Thruster Driver Node ─────────────────────────────────────────────────────
add_executable(thruster_driver_node
  src/thruster_driver_node.cpp
  src/thruster_driver_main.cpp
)
target_include_directories(thruster_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(thruster_driver_node
  rclcpp std_msgs rov_msgs
)

# ── Sensor Sync Node ─────────────────────────────────────────────────────────
add_executable(sensor_sync_node
  src/sensor_sync_node.cpp
  src/sensor_sync_main.cpp
)
target_include_directories(sensor_sync_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sensor_sync_node
  rclcpp std_msgs rov_msgs message_filters
)

# ── Serial Bridge Node ───────────────────────────────────────────────────────
add_executable(serial_bridge_node
  src/serial_bridge_node.cpp
  src/serial_bridge_main.cpp
)
target_include_directories(serial_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(serial_bridge_node
  rclcpp std_msgs rov_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  thruster_driver_node
  sensor_sync_node
  serial_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
