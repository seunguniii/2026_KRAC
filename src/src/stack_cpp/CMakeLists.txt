cmake_minimum_required(VERSION 3.8)
project(stack_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(master src/master.cpp src/mission_manager.cpp)
add_executable(logger src/logger.cpp src/mission_manager.cpp)

ament_target_dependencies(master rclcpp std_msgs px4_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

ament_target_dependencies(logger rclcpp std_msgs px4_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

install(TARGETS
  master
  logger
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
