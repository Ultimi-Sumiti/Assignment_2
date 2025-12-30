#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

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

#include "interfaces/action/plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "../include/utils.h"

static const std::string FRAME_ID = "base_link";
static const std::string PLANNING_GROUP = "ir_arm";
static const std::string GRIPPER_GROUP = "ir_gripper";

namespace custom_action_cpp
{



// 1. Definisci la funzione di conversione
auto to_rad = [](double deg) { return deg * M_PI / 180.0; };
// Define the action client node.
class CsActionClient : public rclcpp::Node
{
public:

  using Plan = interfaces::action::Plan;
  // Using the handleBattery namespace to indicate the battery goal handle.
  using GoalHandlePlan = rclcpp_action::ClientGoalHandle<Plan>;

  // The constructor take node options.
  explicit CsActionClient(const rclcpp::NodeOptions & options)
  : Node("cs_action_client", options)
  {

    node_options_ = options;
    tag1_pos = std::vector<double>{3, 0.0};
    tag10_pos = std::vector<double>{0.57, -0.01, 0.44};
    
    // Here we set the initial pose
    current_pose.header.frame_id = "base_link";
    current_pose.header.stamp = this->now();
    // 2. Position: Coordinate in metri rispetto alla base
    current_pose.pose.position.x = 0.4; 
    current_pose.pose.position.y = 0.1; 
    current_pose.pose.position.z = 0.5;

    // 3. Orientation: Quaternione 
    current_pose.pose.orientation.x = 0.0;
    current_pose.pose.orientation.y = 0.0;
    current_pose.pose.orientation.z = 0.0;
    current_pose.pose.orientation.w = 1.0;


    boxes_to_add = {
    {"table_1", 0.4, 0.4, 0.35, tag1_pos[0] + 0.025,tag1_pos[1] + 0.025, tag1_pos[2] - 0.1 - (0.35)/2},
    {"tag_1", 0.05, 0.05, 0.1, tag1_pos[0] + 0.025, tag1_pos[1] - 0.025, tag1_pos[2] - 0.1 + (0.1)/2},
    {"table_2", 0.4, 0.4, 0.35, tag10_pos[0] + 0.025,tag10_pos[1] + 0.025, tag10_pos[2] - 0.1 - (0.35)/2},
    {"tag_2", 0.05, 0.05, 0.1, tag10_pos[0] + 0.025, tag10_pos[1] + 0.01, tag10_pos[2] - 0.1 + (0.1)/2}
    // ID, Width, Depth, Height, OffsetX, OffsetY, OffsetZ
    };
    // Setting to false since we are just using intialization values.
    box_added = false;

    // Here we set the client pointer using the create client function, Battery client.
    // This command call all the user callback.
    this->client_ptr_ = rclcpp_action::create_client<Plan>(
      this,
      "plan");

    // Here we define the timer callback, which send the goal at intervals.
    auto timer_callback_lambda = [this](){ return this->send_goal(); };

    // Here we created the timer wich calls the timer callback.
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      timer_callback_lambda);
  }

  void get_tag_position(){

    // ### GET APRIL-TAGS POSITION ###
    auto tf_receiver_node = std::make_shared<TryNode>(node_options_);
    // Spin TF node in a separate thread.
    std::thread([&tf_receiver_node]() { rclcpp::spin(tf_receiver_node); }).detach();
    // Allow moveit to initialize.
    std::this_thread::sleep_for(std::chrono::seconds(2)); 

    RCLCPP_INFO(this->get_logger(), "Waiting for AprilTags transforms...");
    bool tags_found = false;
    int max_retries = 20; // Prova per 20 secondi
    
    while(rclcpp::ok() && !tags_found && max_retries > 0) {
        if(tf_receiver_node->get_tags_position(tag1_pos, tag10_pos)) {
            tags_found = true;
            RCLCPP_INFO(this->get_logger(), "Tags Found! Tag1: [%.2f, %.2f, %.2f]", tag1_pos[0], tag1_pos[1], tag1_pos[2]);
            RCLCPP_INFO(this->get_logger(), "Tags Found! Tag10: [%.2f, %.2f, %.2f]", tag10_pos[0], tag10_pos[1], tag10_pos[2]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Transforms not available yet, retrying...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            max_retries--;
        }
    }

    if (!tags_found) {
        RCLCPP_ERROR(this->get_logger(), "Could not find tags. Exiting.");
        return;
    }else{
      // Adding new values of boxes after first value of tags receival.
      if(!box_added){
        add_boxes();
      }
    }
  }

  void add_boxes(){
    // Correcting boxes based on current values of tags.
    boxes_to_add = {
    {"table_1", 0.4, 0.4, 0.35, tag1_pos[0] + 0.025,tag1_pos[1] + 0.025, tag1_pos[2] - 0.1 - (0.35)/2},
    {"tag_1", 0.05, 0.05, 0.1, tag1_pos[0] + 0.025, tag1_pos[1] - 0.025, tag1_pos[2] - 0.1 + (0.1)/2},
    {"table_2", 0.4, 0.4, 0.35, tag10_pos[0] + 0.025,tag10_pos[1] + 0.025, tag10_pos[2] - 0.1 - (0.35)/2},
    {"tag_2", 0.05, 0.05, 0.1, tag10_pos[0] + 0.025, tag10_pos[1] + 0.01, tag10_pos[2] - 0.1 + (0.1)/2}
    // ID, Width, Depth, Height, OffsetX, OffsetY, OffsetZ
    };

    // Creating the collision objects object.
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects = get_collision_object(boxes_to_add, FRAME_ID);
    // Setting all the objcects in the planning scene.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.addCollisionObjects(collision_objects);
  }

  // This function is used to send goal to the battery server.
  void send_goal()
  {
    using namespace std::placeholders;

    // This function set to 0 the timer while it is executing.
    this->timer_->cancel();

    // Here this client is waiting for the action from the server.
    if (!this->client_ptr_->wait_for_action_server()) {
      // If the timer is expired we shutdown.
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // Here we define the goal.
    auto goal_msg = Plan::Goal();
    
    get_tag_position();

    // Here we set the goal message.

    // set new pose based on position of tag1.
    current_pose.pose.position.x = tag1_pos[0] + 0.03;
    current_pose.pose.position.y = tag1_pos[1] + 0.16;
    current_pose.pose.position.z = tag1_pos[2] + 0.04;

    // define orientation (hard-coded):

    // first get the current orientation (wrt base_link).
    tf2::Quaternion q_attuale;
    tf2::fromMsg(current_pose.pose.orientation, q_attuale);

    // define new desired orientation
    tf2::Quaternion q_rotazione;
    q_rotazione.setRPY(0, M_PI, 0 ); // <--- DEVE ESSERE M_PI, non 0

    // compute new orientation by composition of rotations
    tf2::Quaternion q_finale = q_attuale * q_rotazione;
    q_finale.normalize();

    // set final desired values
    current_pose.pose.orientation.x = q_finale.x();
    current_pose.pose.orientation.y = q_finale.y();
    current_pose.pose.orientation.z = q_finale.z();
    current_pose.pose.orientation.w = q_finale.w();
    current_pose.header.frame_id = FRAME_ID;

    std::cout << "Pos: (x: " 
       << current_pose.pose.position.x << ", y: "
       << current_pose.pose.position.y << ", z: "
       << current_pose.pose.position.z << ", x orientation: "
       << current_pose.pose.orientation.x << ", y orientation: "
       << current_pose.pose.orientation.y << ", z orientation: "
       << current_pose.pose.orientation.z << ", w orientation: "
       << current_pose.pose.orientation.w <<") "<< std::endl;


    goal_msg.target_ee_pose = current_pose;
    auto message = std_msgs::msg::String();
    message.data = "free_cartesian";
    goal_msg.move_type = message;


    // Here we sent the goal to server.
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    
    auto send_goal_options = rclcpp_action::Client<Plan>::SendGoalOptions();
    // This lambda function send goal options and check the response.
    send_goal_options.goal_response_callback = [this](const GoalHandlePlan::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        rclcpp::shutdown();
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    // Here we check the feedback 
    send_goal_options.feedback_callback = [this](
      GoalHandlePlan::SharedPtr,
      const std::shared_ptr<const Plan::Feedback> feedback)
    {
      // Here we print on screen the sequence.
      std::stringstream ss;
      ss << "Received current pose: ";
      ss << "Pos: (x: " 
       << feedback->current_ee_pose.pose.position.x << ", y: "
       << feedback->current_ee_pose.pose.position.y << ", z: "
       << feedback->current_ee_pose.pose.position.z << ", x orientation: "
       << feedback->current_ee_pose.pose.orientation.x << ", y orientation: "
       << feedback->current_ee_pose.pose.orientation.y << ", z orientation: "
       << feedback->current_ee_pose.pose.orientation.z << ", w orientation: "
       << feedback->current_ee_pose.pose.orientation.w <<") "<< std::endl;

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    };

    // Here we manage the results.
    send_goal_options.result_callback = [this](const GoalHandlePlan::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          rclcpp::shutdown();
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Goal was canceled");
          rclcpp::shutdown();
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          rclcpp::shutdown();
          return;
      }

      std::stringstream ss;
      ss << "Final final pose received received: ";
      auto final_pos = result.result->final_ee_pose;
      ss << "Pos: (x: " 
         << final_pos.pose.position.x << ", y: " 
         << final_pos.pose.position.y << ", z: " 
         << final_pos.pose.position.z << ", x orientation:"
         << final_pos.pose.orientation.x << ", y orientation:" 
         << final_pos.pose.orientation.y << ", z orientation:"
         << final_pos.pose.orientation.z << ", w orientation:" 
         << final_pos.pose.orientation.w << ")";

      // Here we print the results received if the code was succeded.
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
      rclcpp::shutdown();
    };
    // Here we send the goal and the options to the client and we get the result.
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Plan>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped current_pose;
  std::vector<double> tag1_pos;
  std::vector<double> tag10_pos;
  std::vector<BoxConfig> boxes_to_add;
  bool box_added;
  rclcpp::NodeOptions node_options_;
};  // class CsActionClient

}  // namespace custom_action_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_options = rclcpp::NodeOptions();
  auto node = std::make_shared<custom_action_cpp::CsActionClient>(node_options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}