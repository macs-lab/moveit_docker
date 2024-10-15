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
    geometry_msgs::msg::Pose target_pose = request->target_pose.pose;
    move_group_interface->setPoseTarget(target_pose);
    // setTargetPose(request->target_pose, move_group_interface.get());
    // auto const [success, plan] = createPlan(move_group_interface.get());
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const success = static_cast<bool>(move_group_interface->plan(plan));
    std::cout << "Plan success: " << success << std::endl;

    // Printing the end position
    if (!plan.trajectory_.joint_trajectory.points.empty()) {
    const auto& point = plan.trajectory_.joint_trajectory.points.back();
    std::cout << "Moving to position: ";
    for (double position : point.positions) {
        std::cout << position << " ";
    }
    std::cout << std::endl;

    // Printing all positions
    for (const auto& point : plan.trajectory_.joint_trajectory.points) {
        std::cout << "Positions: ";
        for (double position : point.positions) {
            std::cout << position << " ";
        }
        std::cout << std::endl;
    }
}

    if(success) {
        move_group_interface->execute(plan);
        response->status = 0;
        response->done = true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planning failed!");
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