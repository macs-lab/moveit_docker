#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/point_stamped.h>
#include "inspection_srvs/srv/move_to_pose.hpp"
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>

// using namespace boost::placeholders;

// Help from chatgpt and https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
void addBox(const std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>& planning_scene_interface)
{
    // Define an attached collision object ROS message
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "tool0"; // The link to which the object will be attached

    // The header must contain a valid TF frame
    attached_object.object.header.frame_id = "tool0";

    // The id of the object
    attached_object.object.id = "camera";

    // Define a box to be attached
    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = box_primitive.BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[0] = 0.2;  // Length of the box
    box_primitive.dimensions[1] = 0.15; // Width of the box
    box_primitive.dimensions[2] = 0.3;  // 0.22Height of the box

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 0.7071;
    box_pose.orientation.x = 0.7071;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.1;
    box_pose.position.z = 0.1;

    // Define a cylinder to be attached
    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = cylinder_primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[0] = 0.01; // Height of the cylinder
    cylinder_primitive.dimensions[1] = 0.25; // Radius of the cylinder

    // Define a pose for the cylinder (specified relative to frame_id)
    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.orientation.w = 0.7071;
    cylinder_pose.orientation.x = 0.7071;
    cylinder_pose.orientation.y = 0.0;
    cylinder_pose.orientation.z = 0.0;
    cylinder_pose.position.x = 0.0;
    cylinder_pose.position.y = 0.26;
    cylinder_pose.position.z = 0.13;

    // Add the box and cylinder to the attached object
    attached_object.object.primitives.push_back(box_primitive);
    attached_object.object.primitive_poses.push_back(box_pose);
    attached_object.object.primitives.push_back(cylinder_primitive);
    attached_object.object.primitive_poses.push_back(cylinder_pose);
    attached_object.object.operation = attached_object.object.ADD;

    // Apply the attached collision object to the planning scene
    planning_scene_interface->applyAttachedCollisionObject(attached_object);
}

void move_to_pose(const std::shared_ptr<inspection_srvs::srv::MoveToPose::Request> request,
                  std::shared_ptr<inspection_srvs::srv::MoveToPose::Response> response,
                  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_interface)
{ 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello pose!");

    move_group_interface->setMaxVelocityScalingFactor(0.005); // Scale velocity to 10% of the maximum

    // Get the current pose
    // geometry_msgs::msg::Pose start_pose = request->start_pose.pose;
    // std::cout << "Current pose: " << start_pose.position.x << " " << start_pose.position.y << " " << start_pose.position.z << std::endl;

    // Get the target pose from the request
    geometry_msgs::msg::Pose target_pose = request->target_pose.pose;
    move_group_interface->setPoseTarget(target_pose);
    std::cout << "Targ pose: " << target_pose.position.x << " " << target_pose.position.y << " " << target_pose.position.z << std::endl;
    
    // Create a vector to hold the waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Add the start and end poses to the waypoints
    // waypoints.push_back(start_pose);
    waypoints.push_back(target_pose);
    
    // Create a variable to hold the fraction of the path that was successfully planned
    double fraction = 0.0;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // Try to compute the Cartesian path
    fraction = move_group_interface->computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_, false);
    // waypoints, max_step (m), jump_threshold (m), trajectory, avoid_collisions
    
    // Temporary
    // fraction = 0.0;
    // End temporary
    
    // Check if the path was successfully planned
    if(fraction >= 1.0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Path computed successfully. Moving the robot.");
        move_group_interface->execute(plan);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot moved.");

        response->status = 0;
        response->done = true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to compute path.");
        response->status = 1;
        response->done = false;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hello_moveit");
    // Manipulator
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface = 
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
    // Box
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface = 
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    addBox(planning_scene_interface);
    auto service = node->create_service<inspection_srvs::srv::MoveToPose>("/inspection/move_to_pose",  
        [move_group_interface](const std::shared_ptr<inspection_srvs::srv::MoveToPose::Request> request,
        std::shared_ptr<inspection_srvs::srv::MoveToPose::Response> response) {
            move_to_pose(request, response, move_group_interface);
        }); 
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}