#include "../include/utils.h"

namespace utils {
double deg2rad(double deg) { return deg * M_PI / 180.0; }

tf2::Quaternion RPY2q(double r, double p, double y) {
    tf2::Quaternion q; 
    q.setRPY(r, p, y); 
    return q;
}

double orientation_error(tf2::Quaternion q1, tf2::Quaternion q2) {
    q1.normalize();
    q2.normalize();
    double error = 2 * std::acos(std::abs(q1.dot(q2)));
    return error;
}

double position_error(const geometry_msgs::msg::PoseStamped& x1, const geometry_msgs::msg::PoseStamped& x2) {
    const auto& pos1 = x1.pose.position;
    const auto& pos2 = x2.pose.position;

    double dx, dy, dz;
    dx = pos1.x - pos2.x;
    dy = pos1.x - pos2.x;
    dz = pos1.z - pos2.z;

    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));

    return dist;
}

bool get_tags_position(std::vector<double>& tag1_xyz, std::vector<double>& tag10_xyz, const tf2_ros::Buffer& tf_buffer_) {
    // Defining the variables for the transform message of the 2 tags frame origin position.
    geometry_msgs::msg::TransformStamped T_tag1_base;
    geometry_msgs::msg::TransformStamped T_tag10_base;
    
    // Defining the frame names.
    const std::string tag1_frame_ = "tag36h11:1";
    const std::string tag10_frame_ = "tag36h11:10";
    const std::string base_frame_ = "base_link";

    try {
        // Searching for the frames origins in the tf topic.
        T_tag1_base = tf_buffer_.lookupTransform(
            base_frame_, tag1_frame_, tf2::TimePointZero, std::chrono::milliseconds(100)); // Small Timeout to void blocking.
        
        T_tag10_base = tf_buffer_.lookupTransform(
            base_frame_, tag10_frame_, tf2::TimePointZero, std::chrono::milliseconds(100));

        // Storing the results in x y z, in the given variables.
        tag1_xyz[0] = T_tag1_base.transform.translation.x;
        tag1_xyz[1] = T_tag1_base.transform.translation.y;
        tag1_xyz[2] = T_tag1_base.transform.translation.z;

        tag10_xyz[0] = T_tag10_base.transform.translation.x;
        tag10_xyz[1] = T_tag10_base.transform.translation.y;
        tag10_xyz[2] = T_tag10_base.transform.translation.z;
        
        return true; // If succeded.

    } catch (const tf2::TransformException& ex) {
        // If exception we just return false.
        return false;
    }
}

std::vector<moveit_msgs::msg::CollisionObject> get_collision_object(std::vector<BoxConfig>& boxes_to_add, const std::string FRAME_ID){

    // Return object definition.
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // Iterating through the boxes.
    for (const auto& config : boxes_to_add) {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = FRAME_ID;
        obj.id = config.id;

        // Defining the shape of the solid object to insert in the scene.
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = config.width;
        primitive.dimensions[primitive.BOX_Y] = config.depth;
        primitive.dimensions[primitive.BOX_Z] = config.height;

        // Defining the pose of the object to insert in the scene.
        geometry_msgs::msg::Pose b_pose;
        b_pose.orientation.w = 1.0;
        b_pose.position.x = config.x_offset;
        b_pose.position.y = config.y_offset;
        b_pose.position.z = config.z_offset;

        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(b_pose);
        obj.operation = obj.ADD;

        collision_objects.push_back(obj);
    }
    return collision_objects;
}

} // end namespace
