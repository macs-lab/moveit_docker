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
from inspection_srvs.srv import MoveToPose
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
from std_msgs.msg import Float64
import math

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

        # Dynamic reconfigure twist_started
        self.declare_parameter('twist_started', False, descriptor=ParameterDescriptor(dynamic_typing=True)) # dynamic_typing=True makes the parameter dynamically reconfigurable
        self.declare_parameter('x_twist_speed', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('y_twist_speed', 0.1, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('z_twist_speed', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True)) # declare twist_speed as a parameter
        self.declare_parameter('x_twist_angular', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('y_twist_angular', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('z_twist_angular', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True)) # declare z_twist_angular as a parameter
        self.twist_started = self.get_parameter('twist_started').value
        # self.twist_started = False
        self.x_twist_speed = self.get_parameter('x_twist_speed').value
        self.y_twist_speed = self.get_parameter('y_twist_speed').value
        self.z_twist_speed = self.get_parameter('z_twist_speed').value
        self.x_twist_angular = self.get_parameter('x_twist_angular').value
        self.y_twist_angular = self.get_parameter('y_twist_angular').value
        self.z_twist_angular = self.get_parameter('z_twist_angular').value
        self.add_on_set_parameters_callback(self.on_parameter_change) # call callback when parameter changes
        ###########################

        # Sobel initialize
        self.bridge = CvBridge()
        self.sobel_img = self.create_subscription(
            Image,
            '/inspection/perception/sobel/image',
            self.display_image_from_msg,
            10)
        self.sobel_img  # prevent unused variable warning
        self.sobel_sharp = self.create_subscription(
            Float64,
            'inspection/perception/sobel',
            self.sobel_sharpness_value,
            10)
        self.previous_sharpness = [] # List to store previous sharpness values
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
                #self.send_request(pose_tool0)
                print('done send_request')
                # UNCOMMENT THIS LINE TO MOVE THE ROBOT TWIST AFTER SEND_REQUEST
                self.wait_max_focus()

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
    
    # on_parameter_change is a new method that gets called whenever a parameter changes. It updates self.twist_started with the new value of the twist_started parameter
    def on_parameter_change(self, parameters):
        for parameter in parameters:
            if parameter.name == 'twist_started':
                self.twist_started = parameter.value
            elif parameter.name == 'x_twist_speed':
                self.x_twist_speed = parameter.value
            elif parameter.name == 'y_twist_speed':
                self.y_twist_speed = parameter.value
            elif parameter.name == 'z_twist_speed':
                self.z_twist_speed = parameter.value
            elif parameter.name == 'x_twist_angular':
                self.x_twist_angular = parameter.value
            elif parameter.name == 'y_twist_angular':
                self.y_twist_angular = parameter.value
            elif parameter.name == 'z_twist_angular':
                self.z_twist_angular = parameter.value
        return SetParametersResult(successful=True)
    
    def timer_callback(self):
        if not self.twist_started:
            return
        
        msg = TwistStamped()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.header.frame_id = 'tool0'
        msg.twist.linear.x = self.x_twist_speed
        msg.twist.linear.y = self.y_twist_speed
        msg.twist.linear.z = self.z_twist_speed
        msg.twist.angular.x = self.x_twist_angular 
        msg.twist.angular.y = self.y_twist_angular 
        msg.twist.angular.z = self.z_twist_angular 
        self.twist_publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.twist.linear.z)
    
    # Sobel stuff
    def display_image_from_msg(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("view", cv_image)
            cv2.waitKey(10)
        except CvBridgeError as e:
            self.get_logger().error("Could not convert from '%s' to 'bgr8'. Exception: %s" % (msg.encoding, e))

    # Controller here
    def sobel_sharpness_value(self, msg):
        curr_focus_value = msg.data
        self.curr_focus_value = curr_focus_value
        self.get_logger().info('Focus value: "%s"' % curr_focus_value)

    def wait_max_focus(self):
        # Only move if focus value < specified
        if self.curr_focus_value < 100:
            self.twist_started = self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, True)])
        else:
            self.twist_started = self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, False)])

        # TODO: Add hardcoded speeds depending on focus value
        # Check if moving in the right direction
        ### CONCERN: Noise. Add a filter?
        if len(self.previous_sharpness) >= 5: 
            average_previous_value = sum(self.previous_sharpness) / len(self.previous_sharpness)
            self.get_logger().info('Avg Previous: "%s"' % average_previous_value)
            if self.curr_focus_value < average_previous_value:
                # Flip sign of y_twist_speed, but I hardcoded so it starts at a small speed |0.1|
                current_y_twist_speed = self.get_parameter('y_twist_speed').get_parameter_value().double_value
                negated_y_twist_speed = -0.1 if current_y_twist_speed > 0 else 0.1
                self.set_parameters([rclpy.parameter.Parameter('y_twist_speed', rclpy.parameter.Parameter.Type.DOUBLE, negated_y_twist_speed)])
                self.y_twist_speed = negated_y_twist_speed
            else:
                pass
        self.previous_sharpness.append(self.curr_focus_value)  # Add the current value to the list of previous values
        if len(self.previous_sharpness) > 5:  # Keep the list of previous values at a maximum length of 10
            self.previous_sharpness.pop(0)
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