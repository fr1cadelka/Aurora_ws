cmake_minimum_required(VERSION 3.8)
project(navigation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# === СБОРКА ИСПОЛНЯЕМОГО ФАЙЛА ===
add_executable(navigation_pkg_node
  src/navigation_pkg_node.cpp
)

ament_target_dependencies(navigation_pkg_node
  rclcpp
  std_msgs
)

# Устанавливаем исполняемый файл
install(TARGETS
  navigation_pkg_node
  DESTINATION lib/${PROJECT_NAME}/
)

# === УСТАНОВКА ПАПОК С КОНФИГАМИ И LAUNCH-ФАЙЛАМИ ===
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Для Python-скриптов (если есть)
install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "*.pyc" EXCLUDE
)

# === ТЕСТЫ ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
