#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped,Pose,TransformStamped
import geometry_msgs.msg
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
from inspection_srvs.srv import MoveToPose

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped,'/pose_stamped',10)
        # self.timer = self.create_timer(1.0, self.pose_sender)

    def pose_sender(self):
        pose_tool0 = PoseStamped()
        pose_tool0.header.stamp = self.get_clock().now().to_msg()
        pose_tool0.header.frame_id = 'tool0'
        pose_tool0.pose.position.x = 0.1
        pose_tool0.pose.position.y = 0.0
        pose_tool0.pose.position.z = 0.0 # 0.01
        pose_tool0.pose.orientation.w = 1.0

        pose_tool0_2 = PoseStamped()
        pose_tool0_2.header.stamp = self.get_clock().now().to_msg()
        pose_tool0_2.header.frame_id = 'tool0'
        pose_tool0_2.pose.position.x = -0.1 # 0.01
        pose_tool0_2.pose.position.y = 0.0
        pose_tool0_2.pose.position.z = 0.0 # 0.01
        pose_tool0_2.pose.orientation.w = 1.0

        pose_tool0_3 = PoseStamped()
        pose_tool0_3.header.stamp = self.get_clock().now().to_msg()
        pose_tool0_3.header.frame_id = 'tool0'
        pose_tool0_3.pose.position.x = 0.0
        pose_tool0_3.pose.position.y = 0.0
        pose_tool0_3.pose.position.z = 0.0
        pose_tool0_3.pose.orientation.w = 1.0
        
        self.publisher_.publish(pose_tool0)
        # time.sleep(1)
        self.get_logger().info(f'Publishing: {pose_tool0}')
        # self.publisher_.publish(pose_tool0_2)
        # # time.sleep(1)
        # self.get_logger().info(f'Publishing: {pose_tool0_2}')
        # self.publisher_.publish(pose_tool0_3)
        # self.get_logger().info(f'Publishing: {pose_tool0_3}')


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = MinimalPublisher()
    time.sleep(1)
    # Publish the PoseStamped message
    pose_publisher.pose_sender()

    # Shutdown the node
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()