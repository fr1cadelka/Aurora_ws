cmake_minimum_required(VERSION 3.8)
project(uwb_localization_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlink_parser2 REQUIRED)

add_executable(field_visualizer_pkg_node src/field_visualizer_pkg_node.cpp)
ament_target_dependencies(field_visualizer_pkg_node
  rclcpp geometry_msgs visualization_msgs tf2 tf2_ros)

add_executable(tag_localizer_pkg_node src/tag_localizer_pkg_node.cpp)
ament_target_dependencies(tag_localizer_pkg_node
  rclcpp geometry_msgs visualization_msgs nav_msgs std_srvs tf2 tf2_ros nlink_parser2)

install(TARGETS field_visualizer_pkg_node tag_localizer_pkg_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
