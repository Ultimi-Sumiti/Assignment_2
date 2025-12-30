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


// 1. Definisci la funzione di conversione
auto to_rad = [](double deg) { return deg * M_PI / 180.0; };

struct BoxConfig {
    std::string id;
    double width, depth, height;
    double x_offset, y_offset, z_offset;
};

std::vector<moveit_msgs::msg::CollisionObject> get_collision_object(std::vector<BoxConfig>& boxes_to_add, const std::string FRAME_ID);

//void print_joint(const std::vector<double>& current_joint_values, const std::vector<std::string>& joint_names);

// Try node class definition .
class TryNode : public rclcpp::Node
{
 public:
    TryNode(const rclcpp::NodeOptions &options)
        : Node("try_node", options),
          tag1_frame_("tag36h11:1"), // Assicurati che questi nomi siano corretti nel tuo TF tree
          tag10_frame_("tag36h11:10"),
          base_frame_("base_link")
    {
        // Init transform buffer and transform listener.
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // NUOVO METODO: Ritorna true se trova le trasformate, e riempie i vettori passati per reference
    bool get_tags_position(std::vector<double>& tag1_xyz, std::vector<double>& tag10_xyz);

 private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    const std::string tag1_frame_;
    const std::string tag10_frame_;
    const std::string base_frame_;
};
