cmake_minimum_required(VERSION 3.8)
project(stack_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mission src/mission.cpp src/mission_manager.cpp)
add_executable(flight src/flight.cpp src/mission_manager.cpp)
add_executable(target src/target.cpp src/mission_manager.cpp)
add_executable(logger src/logger.cpp src/mission_manager.cpp)

include_directories(include ${catkin_INCLUDE_DIRS})

ament_target_dependencies(mission rclcpp std_msgs px4_msgs)

ament_target_dependencies(flight rclcpp std_msgs px4_msgs)
target_link_libraries(flight yaml-cpp)
target_include_directories(flight PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(target rclcpp std_msgs geometry_msgs px4_msgs)
target_include_directories(target PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(logger rclcpp std_msgs px4_msgs)

install(TARGETS
  mission
  flight
  target
  logger
  DESTINATION lib/${PROJECT_NAME})

ament_package()
