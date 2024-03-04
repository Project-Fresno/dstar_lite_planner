cmake_minimum_required(VERSION 3.8)
project(dstar_lite_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(plan src/planner.cpp src/Dstar.cpp  include/dstar-lite/Dstar.h)
add_executable(test_ src/test.cpp src/Dstar.cpp  include/dstar-lite/Dstar.h)

target_include_directories(plan PRIVATE include/dstar-lite)
target_include_directories(test_ PRIVATE include/dstar-lite)

ament_target_dependencies(plan rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs)

install(TARGETS
  plan
  test_
  DESTINATION lib/${PROJECT_NAME})

ament_package()
