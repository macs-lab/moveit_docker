#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped,Pose,TransformStamped, TwistStamped
import geometry_msgs.msg
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from rclpy.clock import ROSClock
#from inspection_msgs.srv import MoveToPose
from inspection_srvs.srv import MoveToPose

class PoseStampedCreator(Node):

    def __init__(self):
        super().__init__('pose_stamped_creator')

        # Create a client to the service
        self.pose_client = self.create_client(MoveToPose, '/inspection/move_to_pose')
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Hello world.')

        # Subscribe to a topic publishing PoseStamped messages
        self.pose_queue = []
        self.subscriber = self.create_subscription(PoseStamped,'/pose_stamped', self.queue_pose, 10)
        self.i = 0

        # Publisher to see where the robot is going
        self.publisher_ = self.create_publisher(PoseStamped, 'published_pose', 10)

        # TF stuff 
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        # srv request (target_pose)
        self.req = MoveToPose.Request()   

        # Create the twist client
        self.twist_publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.twist_client = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.twist_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('twist service not available, waiting again...')
        self.get_logger().info('Twist says hello.')
        self.twist_req = Trigger.Request()
        self.future = self.twist_client.call_async(self.twist_req)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Twist will not start until this is True
        self.twist_started = False
        ###########################

        self.tf_wt = TransformStamped()
        self.tf_timer = self.create_timer(1.0, self.on_timer)        

    def queue_pose(self, msg):
        self.i+=1
        self.pose_queue.append(msg)
        self.get_logger().info(f'Iter: {self.i}, Queue length: {len(self.pose_queue)}')

        
    # Move to the pose we received
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
                # UNCOMMENT THIS LINE TO MOVE THE ROBOT
                self.send_request(pose_tool0)
                print('done send_request')

                # Move with TwistStamped, set self.twist_started to True
                self.twist_started = True

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return

    def send_request(self,pose_tool0):
        
        # Transform the pose from 'tool0' to 'world'
        self.get_logger().info('Transforming pose from tool0 to world')
        
        pose_map = tf2_geometry_msgs.do_transform_pose(pose_tool0.pose, self.tf_wt)
        self.get_logger().info('Transformed pose from tool0 to world')

        # Send the transformed pose as the request
        pose_map_stamped = PoseStamped()
        pose_map_stamped.header.frame_id = 'world'
        pose_map_stamped.pose = pose_map
        self.req.target_pose = pose_map_stamped
        print('Target pose in map coordinates: ')        
        print(self.req) # --> target_pose:
        # publish destination pose to topic 'published_pose'
        self.publisher_.publish(pose_map_stamped)
        req = MoveToPose.Request()
        req.target_pose = pose_map_stamped

        self.pose_client.call(req)
    
    def timer_callback(self):        
        #elapsed_time = self.get_clock().now() - self.start_time
        if not self.twist_started:
            return
        
        msg = TwistStamped()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.header.frame_id = 'tool0'
        msg.twist.linear.z = 0.3
        self.twist_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.twist.linear.z)
        #self.future = self.twist_client.call(twist_req)

def main(args=None):
    rclpy.init(args=args)
    pose_stamped_client = PoseStampedCreator()
    # output = input('Hit enter.') # Stops code, wait for user click enter
    try:
        rclpy.spin(pose_stamped_client)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()