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
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

#ament_target_dependencies(group_24_assignment_2
#    rclcpp
#    std_msgs
#    geometry_msgs
#)


include_directories(include)

add_library(group_24_assignment_2
    include/utils.h
    src/utils.cpp
)

ament_target_dependencies(group_24_assignment_2
    rclcpp 
    interfaces
    rclcpp_action 
    rclcpp_components
    std_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    moveit_core
    moveit_ros_planning_interface
)



add_executable(planner_action_server src/planner_action_server.cpp)
ament_target_dependencies(planner_action_server 
    rclcpp 
    interfaces
    rclcpp_action 
    rclcpp_components
    std_msgs
    geometry_msgs
    moveit_ros_planning_interface
    moveit_core
)

add_executable(coordinator_action_client src/coordinator_action_client.cpp)
ament_target_dependencies(coordinator_action_client
    rclcpp 
    interfaces
    rclcpp_action 
    rclcpp_components
    std_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    moveit_core
    moveit_ros_planning_interface
)

target_link_libraries(coordinator_action_client group_24_assignment_2)

add_executable(gripper src/gripper.cpp)
ament_target_dependencies(gripper
    moveit_ros_planning_interface
    rclcpp
    moveit_core
    std_msgs
)

install(TARGETS
    planner_action_server
    coordinator_action_client
    gripper
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY cfg
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
