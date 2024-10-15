#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <boost/bind/bind.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/point_stamped.h>
#include "inspection_srvs/srv/move_to_pose.hpp"
#include <iostream>
// using namespace boost::placeholders;

void move_to_pose(const std::shared_ptr<inspection_srvs::srv::MoveToPose::Request> request,
                  std::shared_ptr<inspection_srvs::srv::MoveToPose::Response> response,
                  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_interface)
{ 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello pose!");

    // Get the current pose
    geometry_msgs::msg::Pose start_pose = request->start_pose.pose;
    std::cout << "Current pose: " << start_pose.position.x << " " << start_pose.position.y << " " << start_pose.position.z << std::endl;

    // Get the target pose from the request
    geometry_msgs::msg::Pose target_pose = request->target_pose.pose;
    move_group_interface->setPoseTarget(target_pose);
    std::cout << "Targ pose: " << target_pose.position.x << " " << target_pose.position.y << " " << target_pose.position.z << std::endl;
    
    // Create a vector to hold the waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Add the start and end poses to the waypoints
    waypoints.push_back(start_pose);
    waypoints.push_back(target_pose);
    
    // Create a variable to hold the fraction of the path that was successfully planned
    double fraction = 0.0;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // Try to compute the Cartesian path
    fraction = move_group_interface->computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_, false);
    // waypoints, max_step (m), jump_threshold (m), trajectory, avoid_collisions

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
    // moveit::planning_interface::MoveGroupInterface move_group_interface(node,"ur_manipulator");
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface = 
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");

    // rclcpp::Service<inspection_srvs::srv::MoveToPose>::SharedPtr service =
    //     node->create_service<inspection_srvs::srv::MoveToPose>("/inspection/move_to_pose", boost::bind(move_to_pose, _1, _2, &move_group_interface));
    auto service = node->create_service<inspection_srvs::srv::MoveToPose>("/inspection/move_to_pose",  
        [move_group_interface](const std::shared_ptr<inspection_srvs::srv::MoveToPose::Request> request,
        std::shared_ptr<inspection_srvs::srv::MoveToPose::Response> response) {
            move_to_pose(request, response, move_group_interface);
        }); 
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}