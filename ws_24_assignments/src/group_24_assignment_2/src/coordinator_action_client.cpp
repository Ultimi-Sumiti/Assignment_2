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
#include <array>
#include <string.h>
// MoveIt Headers
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
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


auto to_rad = [](double deg) { return deg * M_PI / 180.0; };

auto relQ = [](double r, double p, double y) {
    tf2::Quaternion q; q.setRPY(r, p, y); return q;
};


// Coordinator node.
class CoordinatorActionClient : public rclcpp::Node
{

public:

    using Plan = interfaces::action::Plan;
    using GoalHandlePlan = rclcpp_action::ClientGoalHandle<Plan>;

    // --- CONSTRUCTOR ---
    explicit CoordinatorActionClient(const rclcpp::NodeOptions & options)
        : Node("cs_action_client", options), frame_id_("base_link")
    {
        // Needed to run the subscription callback in parallel.
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        // Init interface to add collision blocks.
        psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Init tags position (wrt frame_id_ frame).
        tag1_pos_ = std::vector<double>{3, 0.0};
        tag10_pos_ = std::vector<double>{3, 0.0};
        init_tags_position();

        // Add the collision objects.
        add_boxes();

        // Init starting pose and current pose.
        starting_pose_.pose.position.x = 0.0006; 
        starting_pose_.pose.position.y = 0.19145; 
        starting_pose_.pose.position.z = 1;
        starting_pose_.pose.orientation.w = 0.707107;
        starting_pose_.pose.orientation.x = -0.707105;
        starting_pose_.pose.orientation.y = -0.000218838;
        starting_pose_.pose.orientation.z = 0.0012875;
        starting_pose_.header.frame_id = frame_id_; 
        // Pos: [0.000689429, 0.19145, 1.00106], Ori: [0.707107, -0.707105, -0.000219134, 0.00128763]

        current_pose_ = starting_pose_;

        // Define the plan to execute.
        init_plan();

        // Init action client.
        this->action_client_ = rclcpp_action::create_client<Plan>(
                this,
                "plan"
        );

        // Init publisher to comunnicate with gripper.
        grip_pub_ = this->create_publisher<std_msgs::msg::Float32>(
                "/gripper_move", 10 
        );

        // Init subscription to receive status from gripper.
        grip_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/gripper_status", 
                10,
                std::bind(&CoordinatorActionClient::read_gripper_status, this, std::placeholders::_1),
                sub_opts
        );


        // Start sending goals.
        auto callback = [this](){ return this->send_goal(); };
        this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                callback
        );

    }

private:

    // --- DATA MEMBERS ---
    // Reference frame of the poses.
    const std::string frame_id_;
    rclcpp_action::Client<Plan>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Poses.
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped starting_pose_;

    // Relative to tags.
    std::vector<double> tag1_pos_;
    std::vector<double> tag10_pos_;
    std::vector<BoxConfig> boxes_; // Collision objects.
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;

    // Variables used to define the plan.
    std::vector<std::string> gripper_moves_; // "open" or "close".
    std::vector<std::array<double, 3>> positions_;
    std::vector<bool> relative_; // true => relative, false => absolute.
    std::vector<tf2::Quaternion> relative_rotations_; // wrt current orientation.
    std::vector<bool> path_type_; // Cartesian or not.

    // Current step of the plan.
    size_t step_index_ = 0;

    // This is the amount of abort signal received in same goal.
    size_t aborted_count_ = 0;

    // Publiser for the communication with the gripper node.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr grip_pub_;
    // Subscriber to read gripper status.
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grip_sub_;
    // Status of the gripper (true => busy, false => free)
    bool gripper_status_;
    // To make the subscription run in parallel.
    rclcpp::CallbackGroup::SharedPtr cb_group_;


    // --- MEMBER FUNCTIONS ---
    // Function used to init the plan, step by step.
    void init_plan() {
        
        // Step 0: approach tag1, stay a little bit above it.
        positions_.push_back({tag1_pos_[0] + 0.03, tag1_pos_[1] + 0.16, tag1_pos_[2] + 0.04});
        relative_.push_back(false);
        relative_rotations_.push_back(relQ(0, M_PI, 0));
        path_type_.push_back(0);
        gripper_moves_.push_back("none");

        // Step 1: open gripper and move down wrt current position.
        positions_.push_back({0, 0, -0.08});
        relative_.push_back(true);
        relative_rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("open");

        // Step 2: close gripper and move up wrt current position.
        positions_.push_back({0, 0, 0.25});
        relative_.push_back(true);
        relative_rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("close");

        // Step 3: approach dropping point of tag1 (near tag10).
        positions_.push_back({tag10_pos_[0] - 0.25, tag10_pos_[1] + 0.12, tag10_pos_[2] + 0.1});
        relative_.push_back(false);
        relative_rotations_.push_back(relQ(0, 3*M_PI/2, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Step 4: move down wrt current position.
        positions_.push_back({0, 0, -0.1});
        relative_.push_back(true);
        relative_rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Step 5: open gripper and move up wrt current position.
        positions_.push_back({0, 0, 0.2});
        relative_.push_back(true);
        relative_rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("open");

        // Step 6: approach tag10 from above.
        positions_.push_back({tag10_pos_[0] - 0.13, tag10_pos_[1] + 0.009, tag10_pos_[2] +  0.1});
        relative_.push_back(false);
        relative_rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Step 7: move down wrt current position.
        positions_.push_back({0, 0, -0.15});
        relative_.push_back(true);
        relative_rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Step 8: close gripper and move up wrt current position.
        positions_.push_back({0, 0, 0.1});
        relative_.push_back(true);
        relative_rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("close");

        // Step 9: move towards the other table (dropping point of).
        positions_.push_back({tag1_pos_[0] + 0.03, tag1_pos_[1] + 0.16, tag1_pos_[2] + 0.04});
        relative_.push_back(false);
        relative_rotations_.push_back(relQ(0, -3*M_PI/2, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Step 10: move down wrt current position.
        positions_.push_back({0, 0, -0.1});
        relative_.push_back(true);
        relative_rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Step 11: open gripper and move up wrt current position.
        positions_.push_back({0, 0, 0.3});
        relative_.push_back(true);
        relative_rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("open");
    }

    // Callback used to read gripper status.
    void read_gripper_status(std_msgs::msg::Bool::UniquePtr msg)
    {
        bool status = msg->data;
        RCLCPP_INFO(this->get_logger(), "Gripper has concluded.");
        gripper_status_ = status;
    }

    // Function used to add collision objects based on the tags position.
    void add_boxes()
    {
        // Define boxes positions.
        boxes_ = {
            //   ID        W     D     H          OffsetX               OffsetY               OffsetZ
            { "table_1", 0.40, 0.40, 0.35,  tag1_pos_[0]  + 0.025,  tag1_pos_[1]  + 0.025,  tag1_pos_[2]  - 0.1 - (0.35) / 2 },
            { "tag_1",   0.05, 0.05, 0.10,  tag1_pos_[0]  + 0.025,  tag1_pos_[1]  - 0.025,  tag1_pos_[2]  - 0.1 + (0.10) / 2 },
            { "table_2", 0.40, 0.40, 0.35,  tag10_pos_[0] + 0.025,  tag10_pos_[1] + 0.025,  tag10_pos_[2] - 0.1 - (0.35) / 2 },
            { "tag_2",   0.05, 0.05, 0.10,  tag10_pos_[0] + 0.025,  tag10_pos_[1] + 0.010,  tag10_pos_[2] - 0.1 + (0.10) / 2 }
        };

        // Creating the collision objects object.
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects = get_collision_object(boxes_, frame_id_);

        // Setting all the objcects in the planning scene.
        //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        psi_->addCollisionObjects(collision_objects);
    }

    // Function used to init the tags position (stored in data member).
    void init_tags_position()
    {
        // Init transform buffer and transform listener.
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = 
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_ 
            = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Wait.
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
        RCLCPP_INFO(this->get_logger(), "Waiting for AprilTags transforms...");

        // Get tags position from april-tag node.
        bool tags_found = false;
        int max_retries = 20;
        while(rclcpp::ok() && !tags_found && max_retries > 0) {
            if(get_tags_position(tag1_pos_, tag10_pos_, *tf_buffer_)) {
                tags_found = true;
                RCLCPP_INFO(this->get_logger(), "Tags Found! Tag1: [%.2f, %.2f, %.2f]", tag1_pos_[0], tag1_pos_[1], tag1_pos_[2]);
                RCLCPP_INFO(this->get_logger(), "Tags Found! Tag10: [%.2f, %.2f, %.2f]", tag10_pos_[0], tag10_pos_[1], tag10_pos_[2]);
            } else {
                RCLCPP_WARN(this->get_logger(), "Transforms not available yet, retrying...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                max_retries--;
            }
        }

        if (!tags_found) {
            RCLCPP_ERROR(this->get_logger(), "Could not find tags. Exiting...");
            rclcpp::shutdown();
            return;
        }
    }

    // Function used to send the action goal to the action server.
    void send_goal()
    {

        this->timer_->cancel();

        // Here this client is waiting for the action from the server.
        RCLCPP_INFO(this->get_logger(), "Waiting server...");
        if (!this->action_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), 
                    "Action server not available after waiting. Exiting...");
            rclcpp::shutdown();
            return;
        }

        // Quit if plan is complete (all steps done).
        if(step_index_ > positions_.size()){
            rclcpp::shutdown();
            return;
        }

        // Define the goal.
        auto goal_msg = Plan::Goal();
        if (step_index_ == positions_.size()) { // => Last step: return back
                                                //    to starting position 
            goal_msg.target_ee_pose = starting_pose_;
            std_msgs::msg::String path_type;
            path_type.data = "free_cartesian";
            goal_msg.move_type = path_type;

        } else { // All steps except last one.

            // Perform gripper action, if necessary.
            if(gripper_moves_[step_index_] == "open"){

                gripper_status_ = false;
                auto gripper_msg = std_msgs::msg::Float32();
                gripper_msg.data = to_rad(0.0);
                grip_pub_->publish(gripper_msg);
                while (!gripper_status_) /* Wait gripper*/ ;

            } else if(gripper_moves_[step_index_] == "close"){

                gripper_status_ = false;
                auto gripper_msg = std_msgs::msg::Float32();
                gripper_msg.data = to_rad(5.0);
                grip_pub_->publish(gripper_msg);
                while (!gripper_status_) /* Wait gripper*/ ;

            }

            // Change orientation, if needed.
            if (relative_rotations_[step_index_] != relQ(0, 0, 0)) {
                // Current orientation.
                tf2::Quaternion q_current;
                tf2::fromMsg(current_pose_.pose.orientation, q_current);

                // Final orientation is: final = current * relative.
                tf2::Quaternion q_final = q_current * relative_rotations_[step_index_];
                q_final.normalize();

                current_pose_.pose.orientation.x = q_final.x();
                current_pose_.pose.orientation.y = q_final.y();
                current_pose_.pose.orientation.z = q_final.z();
                current_pose_.pose.orientation.w = q_final.w();
            }
            current_pose_.header.frame_id = frame_id_;

            // Set desired pose.
            if (relative_[step_index_]) {
                current_pose_.pose.position.x += positions_[step_index_][0];
                current_pose_.pose.position.y += positions_[step_index_][1];
                current_pose_.pose.position.z += positions_[step_index_][2];
            } else {
                current_pose_.pose.position.x = positions_[step_index_][0];
                current_pose_.pose.position.y = positions_[step_index_][1];
                current_pose_.pose.position.z = positions_[step_index_][2];
            }

            // Set target pose.
            goal_msg.target_ee_pose = current_pose_;

            // Define the path type.
            std_msgs::msg::String path_type;
            path_type.data = "path_cartesian";
            if (!path_type_[step_index_])  {
                path_type.data = "free_cartesian";
            }
            goal_msg.move_type = path_type;
        }


        RCLCPP_INFO(this->get_logger(), "Current step: %lu", step_index_);
        step_index_ += 1;

        // Define goal options.
        auto send_goal_options = rclcpp_action::Client<Plan>::SendGoalOptions();

        // Response.
        send_goal_options.goal_response_callback = [this](const GoalHandlePlan::SharedPtr & goal_handle)
        {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                rclcpp::shutdown();
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };

        // Feedback.
        send_goal_options.feedback_callback = [this](
                GoalHandlePlan::SharedPtr,
                const std::shared_ptr<const Plan::Feedback> feedback)
        {
            std::stringstream ss;
            ss << "Received current pose: ";
            ss << "Pos: [" 
                << feedback->current_ee_pose.pose.position.x << ", "
                << feedback->current_ee_pose.pose.position.y << ", "
                << feedback->current_ee_pose.pose.position.z << "], Ori: ["
                << feedback->current_ee_pose.pose.orientation.w << ", "
                << feedback->current_ee_pose.pose.orientation.x << ", "
                << feedback->current_ee_pose.pose.orientation.y << ", "
                << feedback->current_ee_pose.pose.orientation.z <<"]." << std::endl;

            //RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        };

        // Result.
        send_goal_options.result_callback = [this](const GoalHandlePlan::WrappedResult & result)
        {
            // Final postiion.
            auto final_pos = result.result->final_ee_pose;
            tf2::Quaternion fo; // Final orientation.
            tf2::fromMsg(final_pos.pose.orientation, fo);

            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    // relative_rotations_[step_index_] *= diff; // TO REMOVE
                    current_pose_ = final_pos; // Update pose.
                    aborted_count_ = 0;
                    send_goal(); /* GO TO NEXT STEP (new goal)! */
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted...");
                    if (aborted_count_ < 3) {
                        RCLCPP_INFO(this->get_logger(), 
                                "Sending again same goal... [%lu]", aborted_count_);
                        step_index_--; // Redo the same step.
                        aborted_count_++;
                        send_goal();
                    } else { // Quit.
                        rclcpp::shutdown();
                        return;
                    }
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                    rclcpp::shutdown();
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    rclcpp::shutdown();
                    return;
            }
        };

        // Send the action goal to the action server.
        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void orientation_error(tf2::Quaternion final) {
        tf2::Quaternion current;
        tf2::fromMsg(current_pose_.pose.orientation, current);

        current.normalize();
        final.normalize();

        tf2::Quaternion diff = current.inverse() * final;
        diff.normalize();

        double error = 2 * std::acos(std::abs(current.dot(final)));
        RCLCPP_INFO(this->get_logger(), "Orientation error: %f", error);
    }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Node options.
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // Create the node.
    auto node = std::make_shared<CoordinatorActionClient>(node_options);

    // Needed to manage movit callbacks.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
