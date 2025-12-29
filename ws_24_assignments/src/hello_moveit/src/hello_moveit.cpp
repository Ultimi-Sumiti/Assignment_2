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

// 1. Definisci la funzione di conversione
auto to_rad = [](double deg) { return deg * M_PI / 180.0; };



using namespace std::chrono_literals;

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
    bool get_tags_position(std::vector<double>& tag1_xyz, std::vector<double>& tag10_xyz) {
        geometry_msgs::msg::TransformStamped T_tag1_base;
        geometry_msgs::msg::TransformStamped T_tag10_base;

        try {
            // Cerchiamo le trasformate
            T_tag1_base = this->tf_buffer_->lookupTransform(
                base_frame_, tag1_frame_, tf2::TimePointZero, std::chrono::milliseconds(100)); // Timeout breve per non bloccare
            
            T_tag10_base = this->tf_buffer_->lookupTransform(
               base_frame_, tag10_frame_, tf2::TimePointZero, std::chrono::milliseconds(100));

            // Salviamo i risultati nei vettori passati dal main
            tag1_xyz[0] = T_tag1_base.transform.translation.x;
            tag1_xyz[1] = T_tag1_base.transform.translation.y;
            tag1_xyz[2] = T_tag1_base.transform.translation.z;

            tag10_xyz[0] = T_tag10_base.transform.translation.x;
            tag10_xyz[1] = T_tag10_base.transform.translation.y;
            tag10_xyz[2] = T_tag10_base.transform.translation.z;
            
            return true; // Successo

        } catch (const tf2::TransformException& ex) {
            // Non stampiamo errore ad ogni loop, ma ritorniamo false
            return false;
        }
    }

 private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    const std::string tag1_frame_;
    const std::string tag10_frame_;
    const std::string base_frame_;
};

struct BoxConfig {
    std::string id;
    double width, depth, height;
    double x_offset, y_offset, z_offset;
};

void plan_execute(
        moveit::planning_interface::MoveGroupInterface& group, 
        moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const geometry_msgs::msg::PoseStamped& pose,
        const rclcpp::Logger& LOGGER
    ) {
    // Set target pose.
    group.setPoseTarget(pose);
    
    // Try to define a plan: max 10 attempts.
    int attempt_count = 1;
    auto res = group.plan(plan);
    while (res != moveit::core::MoveItErrorCode::SUCCESS && attempt_count < 11) {
        res = group.plan(plan);
        std::cout<<"Attempt number "<<attempt_count<<std::endl<<std::endl;
        attempt_count++;
    } 

    // Try to execute the defined plan.
    if (res == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Planning to current pose SUCCESSFUL. Executing...");
        group.execute(plan);
    } else {
        RCLCPP_ERROR(LOGGER, "Planning to current pose FAILED (GOAL_STATE_INVALID likely). This confirms your current pose is illegal in the planning scene.");
    }
}

void plan_execute_cartesian(
        moveit::planning_interface::MoveGroupInterface& group, 
        const geometry_msgs::msg::PoseStamped& target,
        const rclcpp::Logger& LOGGER
    ) {
    // Compute the cartesian path.
    geometry_msgs::msg::Pose start_pose = group.getCurrentPose().pose;

    // Define Waypoints.
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);

    waypoints.push_back(target.pose);

    // Define trajectory parameters.
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // 0.0 disables the jump check (safe for simple paths)
    const double eef_step = 0.01;      // Resolution of the path (1 cm steps)

    // Compute the trajectory.
    double fraction = group.computeCartesianPath(
            waypoints,
            eef_step,
            jump_threshold,
            trajectory
    );

    RCLCPP_INFO(LOGGER, "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // Execute the path.
    if (fraction >= 0.9) {
        group.execute(trajectory);
    } else {
        RCLCPP_WARN(LOGGER, "Could not compute full path. Aborting.");
    }
}

// Main function
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // ### INIT STUFF ###
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("hello_moveit_planner", node_options);
    const auto& LOGGER = node->get_logger();

    std::thread([&node]() { rclcpp::spin(node); }).detach();

    std::this_thread::sleep_for(std::chrono::seconds(2)); // Allow moveit to initialize

    static const std::string FRAME_ID = "base_link";
    static const std::string PLANNING_GROUP = "ir_arm";
    static const std::string GRIPPER_GROUP = "ir_gripper";
    moveit::planning_interface::MoveGroupInterface gripper_group(node, GRIPPER_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group.setMaxVelocityScalingFactor(1.0);        // set max velocity scaling factor
    move_group.setMaxAccelerationScalingFactor(1.0);    // set max acceleration scaling factor

    move_group.setPlanningTime(15.0);                    // set planning time


    geometry_msgs::msg::PoseStamped p = move_group.getCurrentPose();
    RCLCPP_INFO(LOGGER, "Current Pose frame id: %s", p.header.frame_id.c_str());

    // Set reference frame as base_link.
    move_group.setPoseReferenceFrame("base_link");


    // ### GET APRIL-TAGS POSITION ###
    auto tf_receiver_node = std::make_shared<TryNode>(node_options);
    // Spin TF node in a separate thread
    std::thread([&tf_receiver_node]() { rclcpp::spin(tf_receiver_node); }).detach();

    std::this_thread::sleep_for(std::chrono::seconds(2)); // Allow moveit to initialize

    std::vector<double> tag1_pos(3, 0.0);
    std::vector<double> tag10_pos{0.57, -0.01, 0.44};
    
    RCLCPP_INFO(LOGGER, "Waiting for AprilTags transforms...");
    bool tags_found = false;
    int max_retries = 20; // Prova per 20 secondi
    
    while(rclcpp::ok() && !tags_found && max_retries > 0) {
        if(tf_receiver_node->get_tags_position(tag1_pos, tag10_pos)) {
            tags_found = true;
            RCLCPP_INFO(LOGGER, "Tags Found! Tag1: [%.2f, %.2f, %.2f]", tag1_pos[0], tag1_pos[1], tag1_pos[2]);
            RCLCPP_INFO(LOGGER, "Tags Found! Tag10: [%.2f, %.2f, %.2f]", tag10_pos[0], tag10_pos[1], tag10_pos[2]);
        } else {
            RCLCPP_WARN(LOGGER, "Transforms not available yet, retrying...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            max_retries--;
        }
    }

    if (!tags_found) {
        RCLCPP_ERROR(LOGGER, "Could not find tags. Exiting.");
        return 0;
    }


    // ### ADD GREEN BLOCKS (constraints) ###
    geometry_msgs::msg::PoseStamped pose = move_group.getCurrentPose();
    
    std::vector<BoxConfig> boxes_to_add = {
    {"table_1", 0.4, 0.4, 0.35, tag1_pos[0] + 0.025,tag1_pos[1] + 0.025, tag1_pos[2] - 0.1 - (0.35)/2},
    {"tag_1", 0.05, 0.05, 0.1, tag1_pos[0] + 0.025, tag1_pos[1] - 0.025, tag1_pos[2] - 0.1 + (0.1)/2},
    {"table_2", 0.4, 0.4, 0.35, tag10_pos[0] + 0.025,tag10_pos[1] + 0.025, tag10_pos[2] - 0.1 - (0.35)/2},
    {"tag_2", 0.05, 0.05, 0.1, tag10_pos[0] + 0.025, tag10_pos[1] + 0.01, tag10_pos[2] - 0.1 + (0.1)/2}
    // ID, Width, Depth, Height, OffsetX, OffsetY, OffsetZ
    };

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    for (const auto& config : boxes_to_add) {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = FRAME_ID;
        obj.id = config.id;

        // Definizione della forma
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = config.width;
        primitive.dimensions[primitive.BOX_Y] = config.depth;
        primitive.dimensions[primitive.BOX_Z] = config.height;

        // Definizione della posa
        geometry_msgs::msg::Pose b_pose; // Usiamo Pose, non PoseStamped
        b_pose.orientation.w = 1.0;
        b_pose.position.x = config.x_offset;
        b_pose.position.y = config.y_offset;
        // Calcolo della Z: posizione del tag + offset - metà altezza per centrare il box
        b_pose.position.z = config.z_offset;

        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(b_pose);
        obj.operation = obj.ADD;

        collision_objects.push_back(obj);
    }

    // Applichiamo tutti gli oggetti alla scena in una volta sola
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.addCollisionObjects(collision_objects);

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // 2. Definisci il valore per il 4° giunto (wrist_1_joint)
    double joint_4_target = to_rad(-177.0); 

    // 3. Modifica solo il valore di quel giunto nello stato
    // Assicurati che il nome "wrist_1_joint" sia corretto nel tuo URDF
    current_state->setJointPositions("wrist_1_joint", &joint_4_target);

    // 4. Imposta questo stato modificato come target dei giunti
    // Questo dice al solutore IK: "Cerca di stare il più vicino possibile a questa configurazione"
    move_group.setJointValueTarget(*current_state);


    /*
     * ENTIRE PIPELINE
     */

    // ### Step 1: Open grip and approach tag1 (red) ###
    gripper_group.setNamedTarget("open");
    gripper_group.move(); 

    // get current end-effector pose in cartesian space
    // set new pose based on position of tag1
    pose.pose.position.x = tag1_pos[0] + 0.03;
    pose.pose.position.y = tag1_pos[1] + 0.16;
    pose.pose.position.z = tag1_pos[2] + 0.04;

    // define orientation (hard-coded):

    // first get the current orientation (wrt base_link).
    tf2::Quaternion q_attuale;
    tf2::fromMsg(pose.pose.orientation, q_attuale);

    // define new desired orientation
    tf2::Quaternion q_rotazione;
    q_rotazione.setRPY(0, M_PI, 0 ); // <--- DEVE ESSERE M_PI, non 0

    // compute new orientation by composition of rotations
    tf2::Quaternion q_finale = q_attuale * q_rotazione;
    q_finale.normalize();

    // set final desired values
    pose.pose.orientation.x = q_finale.x();
    pose.pose.orientation.y = q_finale.y();
    pose.pose.orientation.z = q_finale.z();
    pose.pose.orientation.w = q_finale.w();
    pose.header.frame_id = FRAME_ID;

    plan_execute(move_group, my_plan, pose, LOGGER);
    std::vector<std::string> joint_names = move_group.getJointNames();
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();

    RCLCPP_INFO(LOGGER, "--- Valori Correnti dei Giunti ---");
    for (size_t i = 0; i < joint_names.size(); ++i) {
        RCLCPP_INFO(LOGGER, "Giunto %s: %f rad (%.2f deg)", 
                    joint_names[i].c_str(), 
                    current_joint_values[i], 
                    current_joint_values[i] * 180.0 / M_PI);
    }

    // ### STEP 2: Grab tag1, and lift up ###
    // Limit speed.
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    pose.pose.position.z -= 0.08;
    plan_execute_cartesian(move_group, pose, LOGGER);

    gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint", to_rad(5));
    gripper_group.move(); 

    // ### STEP 3: Move up ee and move towards drop position ###
    pose.pose.position.z += 0.2;
    plan_execute_cartesian(move_group, pose, LOGGER);

    // pos = [0.56, -0.02, 0.68] TO REMOVE (for luca)
    pose.pose.position.x = tag10_pos[0] - 0.25;
    pose.pose.position.y = tag10_pos[1] + 0.12;
    //pose.pose.position.z = tag10_pos[2] - 0.05 + 0.2;

    // Compute new desired orientation.
    tf2::fromMsg(pose.pose.orientation, q_attuale);
    q_rotazione.setRPY(0, 3*M_PI/2, 0 );

    // compute new orientation by composition of rotations
    q_finale = q_attuale * q_rotazione;
    q_finale.normalize();

    // set final desired values
    pose.pose.orientation.x = q_finale.x();
    pose.pose.orientation.y = q_finale.y();
    pose.pose.orientation.z = q_finale.z();
    pose.pose.orientation.w = q_finale.w();
    pose.header.frame_id = FRAME_ID;

    plan_execute_cartesian(move_group, pose, LOGGER);

    // ### STEP 4: drop tag1, open grip and lift up ###
    pose.pose.position.z -=0.2;
    plan_execute_cartesian(move_group, pose, LOGGER);

    gripper_group.setNamedTarget("open");
    gripper_group.move(); 

    pose.pose.position.z = tag10_pos[2] + 0.04;
    plan_execute_cartesian(move_group, pose, LOGGER);

    
    // ### STEP 5: move towards tag10 (blue) and grab it ###
    pose.pose.position.x = tag10_pos[0] - 0.13;
    pose.pose.position.y = tag10_pos[1] + 0.008;
    plan_execute_cartesian(move_group, pose, LOGGER);

    pose.pose.position.z -=0.1;
    plan_execute_cartesian(move_group, pose, LOGGER);

    gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint", to_rad(5));
    gripper_group.move(); 

    // ## STEP 6: lift up tag10 and move towards drop point ###
    pose.pose.position.z +=0.1;
    plan_execute_cartesian(move_group, pose, LOGGER);

    pose.pose.position.x = tag1_pos[0] + 0.03;
    pose.pose.position.y = tag1_pos[1] + 0.16;

    // Compute new desired orientation.
    tf2::fromMsg(pose.pose.orientation, q_attuale);
    q_rotazione.setRPY(0, -3*M_PI/2, 0 );

    // compute new orientation by composition of rotations
    q_finale = q_attuale * q_rotazione;
    q_finale.normalize();

    // set final desired values
    pose.pose.orientation.x = q_finale.x();
    pose.pose.orientation.y = q_finale.y();
    pose.pose.orientation.z = q_finale.z();
    pose.pose.orientation.w = q_finale.w();
    pose.header.frame_id = FRAME_ID;

    plan_execute_cartesian(move_group, pose, LOGGER);

    // STEP 7: drop tag10 ###
    pose.pose.position.z -=0.1;
    plan_execute_cartesian(move_group, pose, LOGGER);

    gripper_group.setNamedTarget("open");
    gripper_group.move(); 

    pose.pose.position.z +=0.1;
    plan_execute_cartesian(move_group, pose, LOGGER);
    
    // ### STEP 8: (TODO) go back to starting position ###

    RCLCPP_INFO(LOGGER, "Process finished. Shutting down.");
    rclcpp::shutdown();
    return 0;
}
