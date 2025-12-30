#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include "interfaces/action/plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp" // We are dealing with components and not only simple nodes!
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

using namespace std::chrono_literals;

namespace plan_action
{
class PlannerActionServer : public rclcpp::Node
{
public:
  // Alias fot type to simplify the legibility of the code, c++ could be very verbose with so many template
  using Plan = interfaces::action::Plan;
  using GoalHandlePlan = rclcpp_action::ServerGoalHandle<Plan>;

  //GROUP_24_ASSIGNMENT_2_PUBLIC
  // ----- CONSTRUCTOR -----
  explicit PlannerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("plan_action_server", options)
  {
    using namespace std::placeholders;
    
    // Callback with lambda function for handling goals, it simply accept all goals
    auto handle_goal = [this]( // args of the lambda, // unique identifier of the goal, // pointer to .action message
      const rclcpp_action::GoalUUID & uuid, 
      std::shared_ptr<const Plan::Goal> goal) 
    { 
      RCLCPP_INFO(this->get_logger(), "Received goal request with target_ee_pose and type of movement  " ); //goal->order

      (void)uuid;// typical trick to suppress warining fo an unused variable
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // accept the goal and call the second callback handle_accepted()
    };

    // Callback with lambda function for dealing with cancellation, this implementation just tells the client that it accepted the cancellation
    auto handle_cancel = [this](
      
      const std::shared_ptr<GoalHandlePlan> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    // Callback with lambda function for accepting a new goal and starts processing it
    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandlePlan> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor,
      // so we declare a lambda function to be called inside a new thread
      auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);}; // Launch of the execute() function
      std::thread{execute_in_thread}.detach();
    };

    // Constructor instantiate an ActionServer, in this object Server is implemeted the mechanism of binding,
    // so when a message is sent the various callbacks are called
    // In general an action server requires 6 things: 
    this->action_server_ = rclcpp_action::create_server<Plan>( // templated action type name
      this, // A ROS 2 node to add the action to
      "plan", // The action name
      handle_goal, // A callback function for handling goals
      handle_cancel,// A callback function for handling cancellation
      handle_accepted); // A callback function for handling goal accept


    // Init to call the initialization of the moveit group of the ir_planner 
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&PlannerActionServer::init_moveit, this)
        );


  }



private:
// ----- DATA MEMEBERS -----
  rclcpp_action::Server<Plan>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> planner_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_feedback_;

// ----- MEMBER FUNCTIONS -----
void plan_execute(
        const geometry_msgs::msg::PoseStamped& pose    
    ) {
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

void plan_execute_cartesian( 
        const geometry_msgs::msg::PoseStamped& target
    ) {
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

  
/* 
 Remmeber how is build the .action file: 
 Goal - Request - Feedback
*/
  // Function that process the goal required by the Client and response to it
  void execute(const std::shared_ptr<GoalHandlePlan> goal_handle) {
    
      

    RCLCPP_INFO(this->get_logger(), "Executing goal");



    //rclcpp::Rate loop_rate(1); // 1 Hz freqenzy

    // Recover goal send by the Client
    const auto goal = goal_handle->get_goal(); 
    geometry_msgs::msg::PoseStamped target_ee_pose = goal->target_ee_pose;
    std::string move_type = goal->move_type.data;

    // Inizialization of the Feedback and Result as shared pointer
    auto feedback = std::make_shared<Plan::Feedback>(); // Feedback
    
    // Start to send feedback
    auto send_feedback = [this, goal_handle, feedback]()
    {
    
      feedback->current_ee_pose = planner_group_->getCurrentPose();
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

    };
    timer_feedback_ = this->create_wall_timer(1000ms, send_feedback);

    // result
    auto result = std::make_shared<Plan::Result>(); // Result

    if (move_type == "path_cartesian")
    {

      plan_execute_cartesian(target_ee_pose);
    }
    else if (move_type == "free_cartesian")
    {
      plan_execute(target_ee_pose);
    }

    // Check if goal is done
    if (rclcpp::ok()) { // Check if the node is still operative
      // notice this line, at left sequence is the field of result described in .action, at right sequence 
      // is the alias to the field partial_sequence of feedback. Indeed at the end the partial sequence correspond with the complete one
      result->final_ee_pose = feedback->current_ee_pose; 
      goal_handle->succeed(result); // Set goal SUCCEEDED and sent result to Client
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  };

};  // class PlanActionServer


//    // Loop that iterates with 1 Hz frequency
//    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
//      // Check if there is a cancel request
//      // verify an internal status of the Action server, so if simultaneosly the Client has sent
//      // a notification of canceling
//      if (goal_handle->is_canceling()) {
//        result->sequence = sequence;
//        goal_handle->canceled(result);
//        RCLCPP_INFO(this->get_logger(), "Goal canceled");
//        return;
//      }
//      // Update sequence using the Plan recurency
//      sequence.push_back(sequence[i] + sequence[i - 1]);
//      // Publish feedback
//      goal_handle->publish_feedback(feedback);
//      RCLCPP_INFO(this->get_logger(), "Publish feedback");
//
//      loop_rate.sleep(); // sleep in order to mantain 1 Hz frequency
//    }
//



}  

// This macro is used when we deal with not simple nodes but for example with actions
// and allow this client to work 
int main(int argc, char ** argv)
{
  // Inizializza ROS 2
  rclcpp::init(argc, argv);

  // Crea il nodo (assicurati di usare il namespace corretto se ne hai uno)
  auto node = std::make_shared<plan_action::PlannerActionServer>();

  // Mantiene il nodo attivo (processa le callback)
  rclcpp::spin(node);

  // Chiude ROS 2 quando il nodo viene terminato (Ctrl+C)
  rclcpp::shutdown();

  return 0;
}







