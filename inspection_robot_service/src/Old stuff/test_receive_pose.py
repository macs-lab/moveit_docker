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
#from inspection_msgs.srv import MoveToPose
from inspection_srvs.srv import MoveToPose
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(PoseStamped,'/pose_stamped',self.queue_pose,10)
        # self.subscription  # prevent unused variable warning
        self.i = 0
        self.pose_queue = []

        # TF stuff 
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.tf_wt = TransformStamped()
        self.tf_timer = self.create_timer(1.0, self.on_timer)

    def queue_pose(self, msg):
        self.i+=1
        self.pose_queue.append(msg)
        self.get_logger().info(f'{self.i},QLen: {len(self.pose_queue)}')#||RecPose:{msg}')
    
    def on_timer(self):
        if len(self.pose_queue) > 0:
            pose_tool0 = self.pose_queue.pop(0)
            from_frame_rel = pose_tool0.header.frame_id
            to_frame_rel = 'world'

            try:
                self.tf_wt = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())
                self.get_logger().info(f'Got transform: {self.tf_wt}')
                # self.send_request()
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()