#include "../include/utils.h"


// NUOVO METODO: Ritorna true se trova le trasformate, e riempie i vettori passati per reference
bool TryNode::get_tags_position(std::vector<double>& tag1_xyz, std::vector<double>& tag10_xyz) {
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



std::vector<moveit_msgs::msg::CollisionObject> get_collision_object(std::vector<BoxConfig>& boxes_to_add, const std::string FRAME_ID){

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
    return collision_objects;
}

/*
void print_joint(const std::vector<double>& current_joint_values, const std::vector<std::string>& joint_names){
    RCLCPP_INFO(LOGGER, "--- Valori Correnti dei Giunti ---");
    for (size_t i = 0; i < joint_names.size(); ++i) {
        RCLCPP_INFO(LOGGER, "Giunto %s: %f rad (%.2f deg)", 
                    joint_names[i].c_str(), 
                    current_joint_values[i], 
                    current_joint_values[i] * 180.0 / M_PI);
    }
}
    */