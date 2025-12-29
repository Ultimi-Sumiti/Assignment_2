cmake_minimum_required(VERSION 3.8)
project(group_24_assignment_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)


#find_package(rclcpp_components REQUIRED)

#add_library(group_24_assignment_2
#)

#ament_target_dependencies(group_24_assignment_2
#  rclcpp
#)



ament_package()
