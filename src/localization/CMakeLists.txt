cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###### find dependencies ######
find_package(ament_cmake REQUIRED)

##### Copy folders to install directory #####
install(
  DIRECTORY 
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Python 스크립트들을 실행 파일로 설치
install(
  PROGRAMS
    localization/ekf_odom_localizer_node.py
    localization/pf_global_localizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

