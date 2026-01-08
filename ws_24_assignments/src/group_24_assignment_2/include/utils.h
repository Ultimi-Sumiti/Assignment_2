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

// This struct contains all the elements associated with position and shape of a box.
struct BoxConfig {
    std::string id;
    double width, depth, height;
    double x_offset, y_offset, z_offset;
};
 
/*
This fuction returns the collision objects associated with the given objects.

Args:
    -boxes_to_add: vector of BoxConfig to consider in the collision objects returned vector.
    -FRAME_ID: frame we need to place the collision object.
*/
std::vector<moveit_msgs::msg::CollisionObject> get_collision_object(std::vector<BoxConfig>& boxes_to_add, const std::string FRAME_ID);

/*
This function return treu when the tag position are retrived correctly.

Args:
    -tag1_xyz: the tag1 position.
    -tag10_xyz: the tag10 position.
    -tf_buffer_: the ros tf topic buffer.
*/
bool get_tags_position(std::vector<double>& tag1_xyz, std::vector<double>& tag10_xyz, const tf2_ros::Buffer& tf_buffer_);
