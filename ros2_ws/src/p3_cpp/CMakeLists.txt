cmake_minimum_required(VERSION 3.8)
project(p3_cpp)

# [핵심] C++ 17 표준 강제 설정 (구조적 바인딩 에러 해결)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)               # 추가됨
find_package(tf2_geometry_msgs REQUIRED) # 추가됨

# 1. 차량용 노드 빌드 (control_p3)
add_executable(control_p3 src/control_p3.cpp)
ament_target_dependencies(control_p3 
  rclcpp 
  std_msgs 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs
)

# 2. PC용 메인 노드 빌드 (main_p3)
add_executable(main_p3 src/main_p3.cpp)
ament_target_dependencies(main_p3 
  rclcpp 
  std_msgs 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs
)

# 3. 설치 설정 (install)
# 실행 파일 설치
install(TARGETS
  control_p3
  main_p3
  DESTINATION lib/${PROJECT_NAME}
)

# Launch 파일 폴더 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# CSV 파일 폴더(tool) 설치
install(DIRECTORY tool
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips the linter which checks for uninitialized variables
  # comment the line when there is no uninitialized variable
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()