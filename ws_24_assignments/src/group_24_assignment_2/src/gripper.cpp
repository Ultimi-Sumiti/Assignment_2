#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

// Class that implements the interaction with the gripper of the arm.
class Gripper : public rclcpp::Node 
{

public:

    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

    // --- CONSTRUCTOR ---
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

        // Init moveit move group.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&Gripper::init_moveit, this)
        );

    }

private:

    // --- DATA MEMBERS ---
    // Publishes data onto status topic.
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    // Reads the motion to be executed.
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    // Name of the joint associated to the gripper.
    std::string joint_name_;
    // Interface with moveit.
    std::shared_ptr<MoveGroupInterface> gripper_group_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- MEMBER FUNCTIONS ---
    // Callback used to read from topic.
    void read_joint_val(std_msgs::msg::Float32::UniquePtr msg)
    {
        float rad = msg->data; // Get angle.
        RCLCPP_INFO(this->get_logger(), "Input: %f rad", rad);
        move_gripper(rad); // Execute move.
        // Publish status. 
        std_msgs::msg::Bool status;
        status.data = true;
        publisher_->publish(status);
    }

    // Needed to init 'gripper_group_' member function. Cannot place inside
    // constructor because 'this->shared_from_this()' is not initializated.
    void init_moveit() 
    {
        timer_->cancel(); // Just init one time.

        // Init move group.
        auto node_ptr = this->shared_from_this();
        gripper_group_= std::make_shared<MoveGroupInterface>(node_ptr, "ir_gripper");

        // Move group settings.
        gripper_group_->setMaxVelocityScalingFactor(1.0);
        gripper_group_->setMaxAccelerationScalingFactor(1.0);

        // Init to a known state (closed).
        move_gripper(0.8);

        RCLCPP_INFO(this->get_logger(), "MoveIt Initialized!");
    }

    // Function used to open/close the gripper.
    void  move_gripper(float rad)
    {    
        gripper_group_->setJointValueTarget(joint_name_, rad);
        gripper_group_->move(); 
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gripper>());
    rclcpp::shutdown();
    return 0;
}
