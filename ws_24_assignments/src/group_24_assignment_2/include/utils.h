#ifndef UTILS_H
#define UTILS_H

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>

namespace utils {

// Transform deg -> rad.
double deg2rad(double deg);

// Given RPY angles, return the associated quaternion.
tf2::Quaternion RPY2q (double r, double p, double y);

// Return the geodesic distance between q1 and q2.
double orientation_error(tf2::Quaternion q1, tf2::Quaternion q2);

// Return the euclideian distance between x1 and x2.
double position_error(const geometry_msgs::msg::PoseStamped& x1, const geometry_msgs::msg::PoseStamped& x2);

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

} // end namespace

#endif
