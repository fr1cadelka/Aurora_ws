cmake_minimum_required(VERSION 3.8)
project(control_telega_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== Зависимости =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ===== Старый узел (Pixhawk / MAVROS) =====
add_executable(control_telega_pkg_node src/control_telega_pkg_node.cpp)
ament_target_dependencies(control_telega_pkg_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  geometry_msgs
  mavros_msgs
)

# ===== Новый узел (прямое управление VESC) =====
add_executable(control_vesk_pkg_node src/control_vesk_pkg_node.cpp)
ament_target_dependencies(control_vesk_pkg_node
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    geometry_msgs
    mavros_msgs
)

# ===== Установка =====
install(TARGETS
  control_telega_pkg_node
  control_vesk_pkg_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
