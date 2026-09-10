cmake_minimum_required(VERSION 3.8)
project(odometry_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)   # <-- ОБЯЗАТЕЛЬНО
find_package(std_msgs REQUIRED)

add_executable(odometry_pkg_node src/odometry_pkg_node.cpp)

target_include_directories(odometry_pkg_node PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${vesc_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}   # <-- ДОБАВИТЬ
)

target_link_libraries(odometry_pkg_node
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${vesc_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}   # <-- ДОБАВИТЬ
  ${std_msgs_LIBRARIES}
)

install(TARGETS odometry_pkg_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
