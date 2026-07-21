cmake_minimum_required(VERSION 3.8)
project(flight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(GEOGRAPHIC_LIBRARY
  NAMES Geographic GeographicLib
)

add_executable(flight_test src/flight_test.cpp)
add_executable(gps_test src/gps_test.cpp)

ament_target_dependencies(flight_test rclcpp px4_msgs geometry_msgs)
target_include_directories(flight_test PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(gps_test rclcpp px4_msgs geometry_msgs)
target_include_directories(gps_test PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(gps_test ${GEOGRAPHIC_LIBRARY})

install(TARGETS
  flight_test
  gps_test
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
