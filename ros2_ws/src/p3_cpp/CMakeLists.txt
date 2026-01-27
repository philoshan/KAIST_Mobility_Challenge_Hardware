cmake_minimum_required(VERSION 3.5)
project(p3_cpp)

# [필수] 소스 코드에서 C++17 문법(Structured Binding 등)을 사용하므로 17로 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Target 1: Stanley Tracker (Control) ---
add_executable(control_p3 src/control_p3.cpp)
ament_target_dependencies(control_p3 rclcpp std_msgs geometry_msgs)

# --- Target 2: Main Traffic Controller (Main) ---
add_executable(main_p3 src/main_p3.cpp)
ament_target_dependencies(main_p3 rclcpp std_msgs geometry_msgs)

# --- Install 설정 (실행 파일) ---
install(TARGETS
  control_p3
  main_p3
  DESTINATION lib/${PROJECT_NAME}
)

# [수정 완료] launch, tool 폴더와 함께 'config' 폴더도 반드시 설치해야 합니다!
# 그래야 launch 파일이 config/bridge_xx.yaml 파일을 찾을 수 있습니다.
install(DIRECTORY launch tool config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()