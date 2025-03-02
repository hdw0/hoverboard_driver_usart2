cmake_minimum_required(VERSION 3.8)
project(hoverboard_driver_usart2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(tf2_geometry_msgs REQUIRED)

# Добавляем путь к модулям cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Поиск библиотеки serial с использованием pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

add_executable(hoverboard_node src/hoverboard_node.cpp)
target_include_directories(hoverboard_node PRIVATE ${SERIAL_INCLUDE_DIRS})

ament_target_dependencies(hoverboard_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(hoverboard_node
  ${Boost_LIBRARIES}
  ${SERIAL_LIBRARIES}
)

install(TARGETS hoverboard_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
