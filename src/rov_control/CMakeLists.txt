cmake_minimum_required(VERSION 3.8)
project(rov_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rov_msgs REQUIRED)

# ── Control Node ──────────────────────────────────────────────────────────────
add_executable(control_node
  src/control_node.cpp
  src/control_main.cpp
)
target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(control_node
  rclcpp std_msgs rov_msgs
)

# ── Teleop Node ───────────────────────────────────────────────────────────────
add_executable(teleop_node
  src/teleop_node.cpp
  src/teleop_main.cpp
)
target_include_directories(teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(teleop_node
  rclcpp std_msgs sensor_msgs rov_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS control_node teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
