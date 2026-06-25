cmake_minimum_required(VERSION 3.8)
project(multirotor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(flight_test src/flight_test.cpp)

ament_target_dependencies(flight rclcpp px4_msgs geometry_msgs)
target_include_directories(flight PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(flight_test rclcpp px4_msgs geometry_msgs)
target_include_directories(flight_test PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(land_test rclcpp px4_msgs geometry_msgs)
target_include_directories(land_test PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(following_flight rclcpp px4_msgs geometry_msgs)
target_include_directories(following_flight PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(position_mode rclcpp px4_msgs geometry_msgs)
target_include_directories(position_mode PUBLIC ${EIGEN3_INCLUDE_DIRS})


install(TARGETS
  land_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
