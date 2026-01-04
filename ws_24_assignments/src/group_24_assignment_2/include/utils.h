#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <memory>
#include <thread> 
#include <chrono>
#include <vector>
#include <string.h>
// MoveIt Headers
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <cmath>

// Transformation libraries.
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Messages libraries.
#include "std_msgs/msg/string.hpp"

// Language libraries.
#include <array>



using namespace std::chrono_literals;

struct BoxConfig {
    std::string id;
    double width, depth, height;
    double x_offset, y_offset, z_offset;
};

//void print_joint(const std::vector<double>& current_joint_values, const std::vector<std::string>& joint_names);
std::vector<moveit_msgs::msg::CollisionObject> get_collision_object(std::vector<BoxConfig>& boxes_to_add, const std::string FRAME_ID);

// NUOVO METODO: Ritorna true se trova le trasformate, e riempie i vettori passati per reference
bool get_tags_position(std::vector<double>& tag1_xyz, std::vector<double>& tag10_xyz, const tf2_ros::Buffer& tf_buffer_);
