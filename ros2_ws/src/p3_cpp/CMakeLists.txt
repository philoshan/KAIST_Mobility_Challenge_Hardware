cmake_minimum_required(VERSION 3.8)
project(p3_cpp)

<<<<<<< HEAD
# [필수] 소스 코드에서 C++17 문법(Structured Binding 등)을 사용하므로 17로 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
=======
# [핵심] C++ 17 표준 강제 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
>>>>>>> a61f35df3e42efd3f2f80f4a12e7ec9aed34c6f3
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

<<<<<<< HEAD
=======
# 의존성 패키지 찾기
>>>>>>> a61f35df3e42efd3f2f80f4a12e7ec9aed34c6f3
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
<<<<<<< HEAD

# --- Target 1: Stanley Tracker (Control) ---
add_executable(control_p3 src/control_p3.cpp)
ament_target_dependencies(control_p3 rclcpp std_msgs geometry_msgs)

# --- Target 2: Main Traffic Controller (Main) ---
add_executable(main_p3 src/main_p3.cpp)
ament_target_dependencies(main_p3 rclcpp std_msgs geometry_msgs)

# --- Install 설정 (실행 파일) ---
=======
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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
>>>>>>> a61f35df3e42efd3f2f80f4a12e7ec9aed34c6f3
install(TARGETS
  control_p3
  main_p3
  DESTINATION lib/${PROJECT_NAME}
)

<<<<<<< HEAD
# [수정 완료] launch, tool 폴더와 함께 'config' 폴더도 반드시 설치해야 합니다!
# 그래야 launch 파일이 config/bridge_xx.yaml 파일을 찾을 수 있습니다.
install(DIRECTORY launch tool config
=======
# [중요 수정] launch, tool 뿐만 아니라 'config' 폴더도 반드시 설치해야 합니다!
install(DIRECTORY 
  launch
  tool
  config
>>>>>>> a61f35df3e42efd3f2f80f4a12e7ec9aed34c6f3
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()