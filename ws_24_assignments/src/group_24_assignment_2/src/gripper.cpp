#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp" 

// Messages
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>

class Gripper : public rclcpp::Node 
{
public:

    Gripper() 
        : Node("gripper"), joint_name_("robotiq_85_left_knuckle_joint")
    {
        // Init publisher.
        publisher_ = this->create_publisher<std_msgs::msg::Bool>(
	            "/gripper_status", 10 
	    );

        // Init subscriber.
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
                "/gripper_move", 
                10,
                std::bind(&Gripper::read_joint_val, this, std::placeholders::_1)
        );

        // Init 
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&Gripper::init_moveit, this)
        );

    }

    // Callback used to read from topic.
    void read_joint_val(std_msgs::msg::Float32::UniquePtr msg) {
        // Read value.
        float rad = msg->data;
        RCLCPP_INFO(this->get_logger(),"I heard: '%f'", rad);
        // Execute move.
        move_gripper(rad);
        
        // TODO: return the status.
    }

private:

    // Needed to init 'gripper_group_' member function. Cannot place inside
    // constructor because 'this->shared_from_this()' is not initializated.
    void init_moveit() {
        timer_->cancel();
        auto node_ptr = this->shared_from_this();
        gripper_group_= std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "ir_gripper");
        RCLCPP_INFO(this->get_logger(), "MoveIt Initialized!");
    }

    // Function used to open/close the gripper.
    void  move_gripper(float rad) {    
        gripper_group_->setJointValueTarget(joint_name_, rad);
        gripper_group_->move(); 
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    std::string joint_name_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gripper>());
    rclcpp::shutdown();
    return 0;
}
