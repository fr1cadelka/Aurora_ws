cmake_minimum_required(VERSION 3.8)
project(launch_rover)

# Устанавливаем стандарт C++ (для совместимости)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Отключаем предупреждения, если они не нужны (оставляем как есть)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск необходимых зависимостей (для launch-пакета нужен только ament_cmake)
find_package(ament_cmake REQUIRED)

# --- Тесты (опционально) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Пропускаем проверку copyright и cpplint, так как у нас нет исходников
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Установка launch-файлов и конфигурации RViz ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# --- Сборка пакета ---
ament_package()
