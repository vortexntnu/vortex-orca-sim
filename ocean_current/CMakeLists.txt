cmake_minimum_required(VERSION 3.8)
project(ocean_current)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ocean_current_publisher
  src/ocean_current_publisher.cpp
)

target_include_directories(ocean_current_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(ocean_current_publisher
 rclcpp 
 std_msgs
 vortex_msgs
 nav_msgs
 geometry_msgs
)

install(TARGETS 
  ocean_current_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME})

ament_package()
