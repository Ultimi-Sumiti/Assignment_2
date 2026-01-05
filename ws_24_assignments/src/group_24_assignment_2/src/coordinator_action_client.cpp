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

static const std::string FRAME_ID = "base_link";


auto to_rad = [](double deg) { return deg * M_PI / 180.0; };

auto relQ = [](double r, double p, double y) {
    tf2::Quaternion q; q.setRPY(r, p, y); return q;
};


// Define the action client node.
class CsActionClient : public rclcpp::Node
{
public:

    using Plan = interfaces::action::Plan;
    using GoalHandlePlan = rclcpp_action::ClientGoalHandle<Plan>;

    // The constructor take node options.
    explicit CsActionClient(const rclcpp::NodeOptions & options)
        : Node("cs_action_client", options)
    {
        // Needed to run callbacks in parallel.
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_opts = rclcpp::SubscriptionOptions();
        sub_opts.callback_group = cb_group_;

        node_options_ = options;
        tag1_pos_ = std::vector<double>{3, 0.0};
        tag10_pos_ = std::vector<double>{3, 0.0};
        init_tags_position();

        // Here we set the initial pose
        current_pose.header.frame_id = "base_link";
        current_pose.header.stamp = this->now();
        // 2. Position: Coordinate in metri rispetto alla base
        current_pose.pose.position.x = 0.4; 
        current_pose.pose.position.y = 0.1; 
        current_pose.pose.position.z = 0.5;

        // 3. Orientation: Quaternione 
        current_pose.pose.orientation.x = -0.707105;
        current_pose.pose.orientation.y = -0.000218838;
        current_pose.pose.orientation.z = 0.0012875;
        current_pose.pose.orientation.w = 0.707107;

        // Case 0
        positions_.push_back({tag1_pos_[0] + 0.03, tag1_pos_[1] + 0.16, tag1_pos_[2] + 0.04});
        relative_.push_back(false);
        rotations_.push_back(relQ(0, M_PI, 0));
        path_type_.push_back(0);
        gripper_moves_.push_back("none");

        // Case 1
        positions_.push_back({0, 0, -0.08});
        relative_.push_back(true);
        rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("open");

        // Case 2
        positions_.push_back({0, 0, 0.25});
        relative_.push_back(true);
        rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("close");

        // Case 3
        positions_.push_back({tag10_pos_[0] - 0.25, tag10_pos_[1] + 0.12, tag10_pos_[2] + 0.1});
        relative_.push_back(false);
        rotations_.push_back(relQ(0, 3*M_PI/2, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Case 4
        positions_.push_back({0, 0, -0.1});
        relative_.push_back(true);
        rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Case 5
        positions_.push_back({0, 0, 0.2});
        relative_.push_back(true);
        rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("open");

        // Case 6
        positions_.push_back({tag10_pos_[0] - 0.12, tag10_pos_[1] + 0.009, tag10_pos_[2] +  0.1});
        relative_.push_back(false);
        rotations_.push_back(relQ(0, 0, 0));
        gripper_moves_.push_back("none");

        // Case 7
        positions_.push_back({0, 0, -0.15});
        relative_.push_back(true);
        rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Case 8
        positions_.push_back({0, 0, 0.1});
        relative_.push_back(true);
        rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("close");

        // Case 9
        positions_.push_back({tag1_pos_[0] + 0.03, tag1_pos_[1] + 0.16, tag1_pos_[2] + 0.04});
        relative_.push_back(false);
        rotations_.push_back(relQ(0, -3*M_PI/2, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Case 10
        positions_.push_back({0, 0, -0.1});
        relative_.push_back(true);
        rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("none");

        // Case 11
        positions_.push_back({0, 0, 0.1});
        relative_.push_back(true);
        rotations_.push_back(relQ(0, 0, 0));
        path_type_.push_back(1);
        gripper_moves_.push_back("open");

        this->client_ptr_ = rclcpp_action::create_client<Plan>(
                this,
                "plan"
        );

        publisher_ = this->create_publisher<std_msgs::msg::Float32>(
                "/gripper_move", 10 
        );

        // Init subscriber.
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
                "/gripper_status", 
                10,
                std::bind(&CsActionClient::read_gripper_status, this, std::placeholders::_1),
                sub_opts
        );

        // Here we define the timer callback, which send the goal at intervals.
        auto timer_callback_lambda = [this](){ return this->send_goal(); };

        // Here we created the timer wich calls the timer callback.
        this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                timer_callback_lambda);

    }

    void read_gripper_status(std_msgs::msg::Bool::UniquePtr msg)
    {
        bool status = msg->data;
        RCLCPP_INFO(this->get_logger(), "Gripper status: '%d'", status);
        gripper_status_ = status;
    }

    void init_tags_position()
    {
        // Init transform buffer and transform listener.
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::this_thread::sleep_for(std::chrono::seconds(1)); 

        RCLCPP_INFO(this->get_logger(), "Waiting for AprilTags transforms...");
        bool tags_found = false;
        int max_retries = 20; // Prova per 20 secondi

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
            RCLCPP_ERROR(this->get_logger(), "Could not find tags. Exiting.");
            return;
        }

        add_boxes();
    }

    void add_boxes()
    {
        // Correcting boxes based on current values of tags.
        boxes_ = {
            {"table_1", 0.4, 0.4, 0.35, tag1_pos_[0] + 0.025,tag1_pos_[1] + 0.025, tag1_pos_[2] - 0.1 - (0.35)/2},
            {"tag_1", 0.05, 0.05, 0.1, tag1_pos_[0] + 0.025, tag1_pos_[1] - 0.025, tag1_pos_[2] - 0.1 + (0.1)/2},
            {"table_2", 0.4, 0.4, 0.35, tag10_pos_[0] + 0.025,tag10_pos_[1] + 0.025, tag10_pos_[2] - 0.1 - (0.35)/2},
            {"tag_2", 0.05, 0.05, 0.1, tag10_pos_[0] + 0.025, tag10_pos_[1] + 0.01, tag10_pos_[2] - 0.1 + (0.1)/2}
            // ID, Width, Depth, Height, OffsetX, OffsetY, OffsetZ
        };

        // Creating the collision objects object.
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects = get_collision_object(boxes_, FRAME_ID);
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
        std::cout << "WAIT" << std::endl;
        if (!this->client_ptr_->wait_for_action_server()) {
            // If the timer is expired we shutdown.
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        if(step_index_ == positions_.size()){
            rclcpp::shutdown();
            return;
        }

        // Here we define the goal.
        auto goal_msg = Plan::Goal();

        if(gripper_moves_[step_index_] == "open"){
            gripper_status_ = false;
            auto gripper_msg = std_msgs::msg::Float32();
            gripper_msg.data = to_rad(0.0);
            publisher_->publish(gripper_msg);
            while (!gripper_status_);
        } else if(gripper_moves_[step_index_] == "close"){
            gripper_status_ = false;
            auto gripper_msg = std_msgs::msg::Float32();
            gripper_msg.data = to_rad(5.0);
            publisher_->publish(gripper_msg);
            while (!gripper_status_);
        }

        if (rotations_[step_index_] != relQ(0, 0, 0)) {
            tf2::Quaternion q_attuale;
            tf2::fromMsg(current_pose.pose.orientation, q_attuale);

            // Moltiplichi per il quaternione relativo salvato nel vettore
            tf2::Quaternion q_finale = q_attuale * rotations_[step_index_];
            q_finale.normalize();

            // set final desired values
            current_pose.pose.orientation.x = q_finale.x();
            current_pose.pose.orientation.y = q_finale.y();
            current_pose.pose.orientation.z = q_finale.z();
            current_pose.pose.orientation.w = q_finale.w();
        }
        current_pose.header.frame_id = FRAME_ID;

        // Update current pose.
        if (relative_[step_index_]) {
            current_pose.pose.position.x += positions_[step_index_][0];
            current_pose.pose.position.y += positions_[step_index_][1];
            current_pose.pose.position.z += positions_[step_index_][2];
        } else {
            current_pose.pose.position.x = positions_[step_index_][0];
            current_pose.pose.position.y = positions_[step_index_][1];
            current_pose.pose.position.z = positions_[step_index_][2];
        }

        goal_msg.target_ee_pose = current_pose;
        auto message = std_msgs::msg::String();
        if(path_type_[step_index_]){
            message.data = "path_cartesian";
        }else{
            message.data = "free_cartesian";
        }
        goal_msg.move_type = message;


        std::cout<<"VALUE OF ACTION: "<<step_index_<<std::endl;
        step_index_ ++;


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
            auto final_pos = result.result->final_ee_pose;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    // Update current_pose.
                    current_pose = final_pos;
                    send_goal();
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    //action--;
                    //send_goal();
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
        };

        // Send the action goal to the server.
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Plan>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped current_pose;

    std::vector<double> tag1_pos_;
    std::vector<double> tag10_pos_;
    std::vector<BoxConfig> boxes_;

    std::vector<std::string> gripper_moves_;
    std::vector<std::array<double, 3>> positions_;
    std::vector<bool> relative_;
    std::vector<tf2::Quaternion> rotations_;
    std::vector<bool> path_type_;

    rclcpp::NodeOptions node_options_;
    size_t step_index_ = 0;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    bool gripper_status_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

};  // class CsActionClient


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Node options.
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // Create the node.
    auto node = std::make_shared<CsActionClient>(node_options);

    // Needed to manage movit callbacks.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
