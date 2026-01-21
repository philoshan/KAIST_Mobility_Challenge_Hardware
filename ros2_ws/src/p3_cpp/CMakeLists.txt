cmake_minimum_required(VERSION 3.5)
project(p3_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) #

add_executable(control_p3 src/control_p3.cpp)
ament_target_dependencies(control_p3 rclcpp std_msgs geometry_msgs)

add_executable(main_p3 src/main_p3.cpp)
ament_target_dependencies(main_p3 rclcpp std_msgs geometry_msgs)


install(TARGETS
  control_p3
  main_p3
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()