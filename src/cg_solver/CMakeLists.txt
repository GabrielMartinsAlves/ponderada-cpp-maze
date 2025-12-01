cmake_minimum_required(VERSION 3.8)
project(cg_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

add_executable(autonomous_navigator_node src/autonomous_navigator_node.cpp)
ament_target_dependencies(autonomous_navigator_node rclcpp cg_interfaces)

install(TARGETS autonomous_navigator_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the build time check of the license contained in the package.xml
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips the build time check of the package name in the package.xml
  set(ament_cmake_package_name_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
