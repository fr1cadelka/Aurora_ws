cmake_minimum_required(VERSION 3.8)
project(costmap_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)      # ЭТОТ ПАКЕТ НУЖЕН
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVDEV REQUIRED libevdev)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Создаем исполняемый файл
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# Подключаем заголовки libevdev
include_directories(${LIBEVDEV_INCLUDE_DIRS})

# Подключаем библиотеки
target_link_libraries(${PROJECT_NAME}_node ${LIBEVDEV_LIBRARIES})

# Подключаем зависимости ROS 2
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    sensor_msgs
    nav_msgs        # ОБЯЗАТЕЛЬНО ДОБАВИТЬ
    tf2
    tf2_ros
    tf2_sensor_msgs
)

# Устанавливаем исполняемый файл
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
