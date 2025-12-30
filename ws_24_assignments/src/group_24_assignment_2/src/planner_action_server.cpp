#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include "interfaces/action/plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

using namespace std::chrono_literals;

namespace plan_action {

class PlannerActionServer : public rclcpp::Node
{

public:

    using Plan = interfaces::action::Plan;
    using GoalHandlePlan = rclcpp_action::ServerGoalHandle<Plan>;

    explicit PlannerActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("plan_action_server", options)
    {
        using namespace std::placeholders;

        // Needed to run callbacks in parallel.
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Function to handle goals (accepts all of them).
        auto handle_goal = 
            [this](
                    const rclcpp_action::GoalUUID & uuid, 
                    std::shared_ptr<const Plan::Goal> goal
            ){ 
                RCLCPP_INFO(this->get_logger(), "Received target pose goal.");
                (void) uuid; // Unused.
                (void) goal; // Unused.
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
            };

        // Function to handle goal cancellation. TODO: i think we cannot cancel goal...
        //                                       so we just need to refuse.
        auto handle_cancel = 
            [this](const std::shared_ptr<GoalHandlePlan> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

        // Function to accept new goals.
        auto handle_accepted = 
            [this](const std::shared_ptr<GoalHandlePlan> goal_handle)
            {
                // Manage goal in another thread.
                auto exe_callback = 
                    [this, goal_handle]() {return this->execute(goal_handle);};
                std::thread{exe_callback}.detach();
            };

        this->action_server_ = rclcpp_action::create_server<Plan>(
                this,   // A ROS 2 node to add the action to
                "plan", // The action name
                handle_goal,   
                handle_cancel,
                handle_accepted
        ); 

        // Init to call the initialization of the moveit group of the ir_planner 
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1), 
                std::bind(&PlannerActionServer::init_moveit, this)
        );
    }


private:

    rclcpp_action::Server<Plan>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> planner_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    void plan_execute(const geometry_msgs::msg::PoseStamped& pose) {
        // Set target pose.
        planner_group_->setPoseTarget(pose);

        // Try to define a plan: max 10 attempts.
        int attempt_count = 1;
        auto res = planner_group_->plan(this->plan_);
        while (res != moveit::core::MoveItErrorCode::SUCCESS && attempt_count < 11) {
            res = planner_group_->plan(this->plan_);
            std::cout<<"Attempt number "<<attempt_count<<std::endl<<std::endl;
            attempt_count++;
        } 

        // Try to execute the defined plan.
        if (res == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning to current pose SUCCESSFUL. Executing...");
            planner_group_->execute(plan_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to current pose FAILED (GOAL_STATE_INVALID likely). This confirms your current pose is illegal in the planning scene.");
        }
    }

    void plan_execute_cartesian(const geometry_msgs::msg::PoseStamped& target)
    {
        // Compute the cartesian path.
        geometry_msgs::msg::Pose start_pose = this->planner_group_->getCurrentPose().pose;

        // Define Waypoints.
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(start_pose);

        waypoints.push_back(target.pose);

        // Define trajectory parameters.
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0; // 0.0 disables the jump check (safe for simple paths)
        const double eef_step = 0.01;      // Resolution of the path (1 cm steps)

        // Compute the trajectory.
        double fraction = this->planner_group_->computeCartesianPath(
                waypoints,
                eef_step,
                jump_threshold,
                trajectory
        );

        RCLCPP_INFO(this->get_logger(), "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

        // Execute the path.
        if (fraction >= 0.9) {
            this->planner_group_->execute(trajectory);
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not compute full path. Aborting.");
        }
    }

    void init_moveit() {
        timer_->cancel();
        auto node_ptr = this->shared_from_this();
        planner_group_= std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "ir_arm");
        RCLCPP_INFO(this->get_logger(), "MoveIt Initialized!");
    }

    // Function that process the goal required by the Client and response to it
    void execute(const std::shared_ptr<GoalHandlePlan> goal_handle) {

        // Recover goal send by the Client
        const auto goal = goal_handle->get_goal(); 
        geometry_msgs::msg::PoseStamped target_ee_pose = goal->target_ee_pose;
        std::string move_type = goal->move_type.data;

        // Inizialization of the Feedback and Result as shared pointer
        auto feedback = std::make_shared<Plan::Feedback>(); // Feedback

        // Feedback callback
        auto send_feedback = [this, goal_handle, feedback]()
        {
            feedback->current_ee_pose = planner_group_->getCurrentPose();
            goal_handle->publish_feedback(feedback);
            double x = feedback->current_ee_pose.pose.position.x;
            double y = feedback->current_ee_pose.pose.position.y;
            double z = feedback->current_ee_pose.pose.position.z;
            RCLCPP_INFO(this->get_logger(), 
                    "Publish feedback [%f, %f, %f]", x, y, z);

        };

        rclcpp::TimerBase::SharedPtr timer = 
            this->create_wall_timer(100ms, send_feedback, cb_group_);

        // The result aka final pose
        auto result = std::make_shared<Plan::Result>();

        // Set the reference frame of the target.
        planner_group_->setPoseReferenceFrame(target_ee_pose.header.frame_id.c_str());

        if (move_type == "path_cartesian")
            plan_execute_cartesian(target_ee_pose);
        else if (move_type == "free_cartesian")
            plan_execute(target_ee_pose);

        // Stop timer
        timer->cancel();

        // Check if goal is done
        if (rclcpp::ok()) { // Check if the node is still operative
                            // notice this line, at left sequence is the field of result described in .action, at right sequence 
                            // is the alias to the field partial_sequence of feedback. Indeed at the end the partial sequence correspond with the complete one
            result->final_ee_pose = feedback->current_ee_pose; 
            goal_handle->succeed(result); // Set goal SUCCEEDED and sent result to Client
            double x = result->final_ee_pose.pose.position.x;
            double y = result->final_ee_pose.pose.position.y;
            double z = result->final_ee_pose.pose.position.z;
            RCLCPP_INFO(this->get_logger(), 
                    "Goal succeeded [%f, %f, %f]", x, y, z);
        }
    };

}; // class PlanActionServer

} // end namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Node options.
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // Create the node.
  auto node = std::make_shared<plan_action::PlannerActionServer>(node_options);

  // Needed to manage movit callbacks.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
