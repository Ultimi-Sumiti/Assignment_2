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


namespace custom_action_cpp
{
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
    // Here we set the initial pose
    current_pose.header.frame_id = "base_link";
    current_pose.header.stamp = this->now();
    // 2. Position: Coordinate in metri rispetto alla base
    current_pose.pose.position.x = 0.45; // 45 cm davanti al robot
    current_pose.pose.position.y = 0.10; // 10 cm a sinistra
    current_pose.pose.position.z = 0.30; // 30 cm di altezza

    // 3. Orientation: Quaternione (Fondamentale: deve essere normalizzato!)
    // Esempio: pinza rivolta verso il basso (rotazione di 180° su Y)
    current_pose.pose.orientation.x = 0.0;
    current_pose.pose.orientation.y = 1.0;
    current_pose.pose.orientation.z = 0.0;
    current_pose.pose.orientation.w = 0.0;

    // Here we set the client pointer using the create client function, Battery client.
    // This command call all the user callback.
    this->client_ptr_ = rclcpp_action::create_client<Plan>(
      this,
      "Plan");

    // Here we define the timer callback, which send the goal at intervals.
    auto timer_callback_lambda = [this](){ return this->send_goal(); };

    // Here we created the timer wich calls the timer callback.
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      timer_callback_lambda);
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
    // Here we set the gola message to current battery level.
    goal_msg.target_ee_pose = current_pose;

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
      ss << "Pos: " 
       << feedback->current_ee_pose.pose.position.x << ", "
       << feedback->current_ee_pose.pose.position.y << ", "
       << feedback->current_ee_pose.pose.position.z;

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
      auto final_pos = result.result->final_ee_pose.pose.position;
      ss << "Pos: (" 
         << final_pos.x << ", " 
         << final_pos.y << ", " 
         << final_pos.z << ")";

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