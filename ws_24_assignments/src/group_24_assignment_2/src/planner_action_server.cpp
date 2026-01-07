#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include "interfaces/action/plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;


// PlannerActionServer class: dedicated to planning and executing free or linear Cartesian arm movements requested by the Coordinator.
class PlannerActionServer : public rclcpp::Node
{

public:
    // Using some alias for some verbose types.
    using Plan = interfaces::action::Plan;
    using GoalHandlePlan = rclcpp_action::ServerGoalHandle<Plan>;
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
    using MoveItErrorCode = moveit::core::MoveItErrorCode;

    // --- CUNSTRUCTOR ---
    explicit PlannerActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("plan_action_server", options)
    {
        using namespace std::placeholders;

        // Needed to run callbacks in parallel.
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Function to handle goals (accepts all of them).
        auto handle_goal = 
            [this]( const rclcpp_action::GoalUUID & uuid, 
                    std::shared_ptr<const Plan::Goal> goal)
            { 
                RCLCPP_INFO(this->get_logger(), "Received target pose goal.");
                (void) uuid; // Unused.
                (void) goal; // Unused.
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
            };

        // Function callback to handle goal cancellation. 
        auto handle_cancel = 
            [this](const std::shared_ptr<GoalHandlePlan> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

        // Function callback to accept new goals.
        auto handle_accepted = 
            [this](const std::shared_ptr<GoalHandlePlan> goal_handle)
            {
                // Manage goal in another thread.
                auto exe_callback = 
                    [this, goal_handle]() {return this->execute(goal_handle);};
                std::thread{exe_callback}.detach();
                
            };

        // Action_server intialization.
        this->action_server_ = rclcpp_action::create_server<Plan>(
                this,   
                "plan", 
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

    // --- DATA MEMBERS ---

    // For the action communication protocol with corrdinator node 
    rclcpp_action::Server<Plan>::SharedPtr action_server_;

    // Interfaces for Moveit Library.
    std::shared_ptr<MoveGroupInterface> planner_group_;
    MoveGroupInterface::Plan plan_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;


    // --- MEMBER FUNCTIONS ---
    
    // Function to initialize the Moveit Library.
    void init_moveit()
    {
        // Just init one time.
        timer_->cancel(); 

        auto node_ptr = this->shared_from_this();
        planner_group_= std::make_shared<MoveGroupInterface>(node_ptr, "ir_arm");

        // Planner settings.
        planner_group_->setNumPlanningAttempts(5);
        planner_group_->setPlanningTime(10);
        planner_group_->setPoseReferenceFrame("base_link");

        RCLCPP_INFO(this->get_logger(), "MoveIt Initialized!");
    }

    // Plan and execute function in free cartesian space (no costraints in the motion of the robotic arm).
    MoveItErrorCode plan_execute(const geometry_msgs::msg::PoseStamped& pose)
    {
        // Planner settings.
        planner_group_->setMaxVelocityScalingFactor(1.0);
        planner_group_->setMaxAccelerationScalingFactor(1.0);
        planner_group_->setPoseReferenceFrame(pose.header.frame_id);

        // Set target pose.
        planner_group_->setPoseTarget(pose);

        // Try to define a plan: max 10 attempts.
        int cnt = 1;
        MoveItErrorCode res = planner_group_->plan(this->plan_);
        while (res != MoveItErrorCode::SUCCESS && cnt < 11)
        {
            cnt++;
            RCLCPP_INFO(this->get_logger(), 
                    "Planning Failed... Attempt number [%d]", cnt
            );
            res = planner_group_->plan(this->plan_);
        } 

        // Excute the defined plan, if was successful.
        if (res == MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), 
                    "Planning SUCCESSFUL. Executing..."
            );
            return planner_group_->execute(plan_);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Planning to current pose FAILED (GOAL_STATE_INVALID likely)."
            );
        }
        return MoveItErrorCode::ABORT;
    }

    // Plan and execute function to move the robotic arm using linear cartesian path to mantain in the same position the end effector. 
    MoveItErrorCode plan_execute_cartesian(const geometry_msgs::msg::PoseStamped& target)
    {
        // Planner settings.
        planner_group_->setMaxVelocityScalingFactor(0.1);
        planner_group_->setMaxAccelerationScalingFactor(0.1);
        planner_group_->setPoseReferenceFrame(target.header.frame_id);

        // Compute the cartesian path.
        geometry_msgs::msg::Pose start_pose = this->planner_group_->getCurrentPose().pose;

        // Define Waypoints: from start pose to target pose.
        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Define trajectory parameters.
        moveit_msgs::msg::RobotTrajectory trajectory;

        // Resolution of path (1cm).
        double ee_step = 0.01; 

        waypoints = {start_pose, target.pose};
        double fraction = this->planner_group_->computeCartesianPath(
                waypoints,
                ee_step,
                trajectory
                );

        RCLCPP_INFO(this->get_logger(), "Cartesian path [%.2f%%] achieved", fraction * 100.0);

        // Execute the path.
        if (fraction >= 0.85)
            return planner_group_->execute(trajectory);
        else
            RCLCPP_WARN(this->get_logger(), "Could not compute full path. Aborting...");

        return MoveItErrorCode::ABORT;
    }

    // Function that process the goal required by the Client and response to it.
    void execute(const std::shared_ptr<GoalHandlePlan> goal_handle)
    {

        // Recover goal send by the Client.
        const auto goal = goal_handle->get_goal(); 

        // Inizialization of the Feedback and Result as shared pointer.
        auto feedback = std::make_shared<Plan::Feedback>(); 

        // Feedback callback.
        auto send_feedback = 
            [this, goal_handle, feedback]() {
                feedback->current_ee_pose = planner_group_->getCurrentPose();
                goal_handle->publish_feedback(feedback);
            };
        // Send feedback with a timer of 10Hz freq. 
        rclcpp::TimerBase::SharedPtr timer = 
            this->create_wall_timer(100ms, send_feedback, cb_group_);

        // The result (aka final pose).
        auto result = std::make_shared<Plan::Result>();

        // Select between free-motion planning or linear Cartesian path execution based on the request mode (select one or the other function).
        MoveItErrorCode res = MoveItErrorCode::ABORT;
        if (goal->move_type.data == "path_cartesian")
            res = plan_execute_cartesian(goal->target_ee_pose);
        else if (goal->move_type.data == "free_cartesian")
            res = plan_execute(goal->target_ee_pose);

        // Stop send_feedback callback.
        timer->cancel();

        // Check if goal is done.
        if (rclcpp::ok())
        {
            if (res == MoveItErrorCode::SUCCESS) {
                result->final_ee_pose = planner_group_->getCurrentPose();
                goal_handle->succeed(result);
            } else {
                result->final_ee_pose = feedback->current_ee_pose; 
                goal_handle->abort(result);
            }

        }
    };

}; // class PlanActionServer


// --- MAIN FUNCTION ---
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Node options.
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // Create the node.
    auto node = std::make_shared<PlannerActionServer>(node_options);

    // Needed to manage movit callbacks.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
