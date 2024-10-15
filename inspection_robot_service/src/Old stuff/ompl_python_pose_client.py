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
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import rosbag2_py

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
        self.pose_queue_sub = self.create_subscription(PoseStamped,'/pose_stamped', self.pose_callback, 10)
        self.i = 0
        self.can_move_to_next_pose = False

        # Publisher to see where the robot is going
        self.pose_publisher = self.create_publisher(PoseStamped, 'published_pose', 10)

        # TF stuff 
        self.tf_buffer = Buffer() # store and manipulate transformations
        self.listener = TransformListener(self.tf_buffer, self)

        # srv request (target_pose)
        self.req = MoveToPose.Request()   

        # Create the twist client
        self.twist_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.twist_client = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.twist_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('twist service not available, waiting again...')
        self.get_logger().info('Twist says hello.')
        self.twist_req = Trigger.Request()
        self.async_twist_call_result = self.twist_client.call_async(self.twist_req)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.twist_callback)

        # Dynamic reconfigure twist_started
        self.declare_parameter('twist_started', False, descriptor=ParameterDescriptor(dynamic_typing=True)) # dynamic_typing=True makes the parameter dynamically reconfigurable
        self.declare_parameter('x_twist_speed', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('y_twist_speed', 0.3, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('z_twist_speed', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True)) # declare twist_speed as a parameter
        self.declare_parameter('x_twist_angular', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('y_twist_angular', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('z_twist_angular', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True)) # declare z_twist_angular as a parameter
        self.twist_started = self.get_parameter('twist_started').value
        self.x_twist_speed = self.get_parameter('x_twist_speed').value
        self.y_twist_speed = self.get_parameter('y_twist_speed').value
        self.z_twist_speed = self.get_parameter('z_twist_speed').value
        self.x_twist_angular = self.get_parameter('x_twist_angular').value
        self.y_twist_angular = self.get_parameter('y_twist_angular').value
        self.z_twist_angular = self.get_parameter('z_twist_angular').value
        self.add_on_set_parameters_callback(self.on_parameter_change) # call callback when parameter changes
        ###########################

        # Sobel initialize
        self.focus_pose_dict = {}
        self.focus_value_sub = self.create_subscription(
            Float64,
            'inspection/perception/focus_value',
            self.focus_value_callback,
            10)
        self.ema_focus_value = 0
        self.curr_focus_value = 0
        self.append_focus_values = []
        self.twist_is_flipping = False
        ###########################

        self.tf_wt = TransformStamped()
        self.tf_timer = self.create_timer(1.0, self.on_timer)    

    def pose_callback(self, msg):
        self.i+=1
        self.pose_queue.append(msg)
        self.get_logger().info(f'Iter: {self.i}, Queue length: {len(self.pose_queue)}')

    # Move to the pose we received
    def on_timer(self):
        # while len(self.pose_queue) > 0:
        #     self.process_pose() # Move to the pose
            # self.simple_feedback() # Feedback control based on focus value
        
        self.iter2_simple_feedback() # Feedback control based on focus value
        if self.can_move_to_next_pose:
            self.get_logger().info('Finished, exiting.')
            rclpy.shutdown()
            sys.exit()
    
    def process_pose(self):
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
            self.get_logger().info(f'Completed transform')
            time.sleep(1)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

    def send_request(self,pose_tool0):
        # Transform the pose from 'tool0' to 'world'
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
        self.pose_publisher.publish(pose_map_stamped)
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
    
    def twist_callback(self):
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
        self.twist_publisher.publish(msg)
    
    def focus_value_callback(self, msg):
        self.curr_focus_value = msg.data
        self.get_logger().info('Focus value: "%s"' % self.curr_focus_value)
        self.save_curr_focus_pose()

    def save_curr_focus_pose(self):
        try:
            time_save_dict = rclpy.time.Time()
            # In general, you would use ROSClock().now().to_msg() when you need the current time as a ROS2 message, 
            # rclpy.time.Time() when you need it as a Python object for calculations or comparisons.
            tf_wt = self.tf_buffer.lookup_transform('world', 'tool0', time_save_dict)
            
            pose_world = PoseStamped()
            pose_world.header.frame_id = 'world'
            pose_world.pose.position.x = tf_wt.transform.translation.x
            pose_world.pose.position.y = tf_wt.transform.translation.y
            pose_world.pose.position.z = tf_wt.transform.translation.z
            pose_world.pose.orientation = tf_wt.transform.rotation

            # Save the current focus value and its associated pose
            self.focus_pose_dict[self.curr_focus_value] = pose_world
            self.focus_pose_dict[self.curr_focus_value] = {'pose': pose_world, 'timestamp': time_save_dict}
            # self.get_logger().info(f'Completed transform')

        except TransformException as ex:
            self.get_logger().info(
                f'Could not fine tune {ex}')
            return

    def iter2_simple_feedback(self):
        self.can_move_to_next_pose = False
        max_focus_value = max(self.focus_pose_dict.keys())

        # Check if moving in the right direction, Update EMA
        K = 2 / (3 + 1)  # Number of data points to average over. Values before last 3 are still considered, just to a lesser degree
        self.ema_focus_value = (K * (self.curr_focus_value - self.ema_focus_value)) + self.ema_focus_value
        
        # Only move if focus value < specified
        if self.curr_focus_value < 50:
            self.twist_started = self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, True)])
        else:
            self.twist_started = self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, False)])
        
        # Feedback
        if (self.curr_focus_value - self.ema_focus_value) < -0.01: # account for noise
            current_y_twist_speed = self.get_parameter('y_twist_speed').get_parameter_value().double_value
            negated_y_twist_speed = -0.3 if current_y_twist_speed > 0 else 0.3
            self.set_parameters([rclpy.parameter.Parameter('y_twist_speed', rclpy.parameter.Parameter.Type.DOUBLE, negated_y_twist_speed)])
            self.y_twist_speed = negated_y_twist_speed

            self.twist_is_flipping = True

        else:
            if self.curr_focus_value < max_focus_value:
                self.twist_started = self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, True)])
            else:
                self.twist_started = self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, False)])
                max_focus_pose = self.focus_pose_dict[max_focus_value]
                print('Max info: ', max_focus_value, max_focus_pose['pose'])
                # self.send_request(max_focus_pose['pose'])

                self.can_move_to_next_pose = True   

    def simple_feedback(self):
        self.can_move_to_next_pose = False

        # Only move if focus value < specified
        if self.curr_focus_value < 100:
            self.twist_started = self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, True)])
        else:
            self.twist_started = self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, False)])

        # TODO: Add hardcoded speeds depending on focus value

        # Check if moving in the right direction, Update EMA
        K = 2 / (3 + 1)  # Number of data points to average over. Values before last 3 are still considered, just to a lesser degree
        self.ema_focus_value = (K * (self.curr_focus_value - self.ema_focus_value)) + self.ema_focus_value

        # Compare curr_focus_value to ema_focus_value instead of average_previous_value
        if (self.curr_focus_value - self.ema_focus_value) < -0.01: # if focus value is decreasing, 0.01 accounts for noise
            # Flip sign of y_twist_speed, but I hardcoded so it starts at a small speed |0.3|
            current_y_twist_speed = self.get_parameter('y_twist_speed').get_parameter_value().double_value
            negated_y_twist_speed = -0.3 if current_y_twist_speed > 0 else 0.3
            self.set_parameters([rclpy.parameter.Parameter('y_twist_speed', rclpy.parameter.Parameter.Type.DOUBLE, negated_y_twist_speed)])
            self.y_twist_speed = negated_y_twist_speed

            self.twist_is_flipping = True
            self.append_focus_values.append(self.curr_focus_value)
        else:
            if self.twist_is_flipping and self.append_focus_values:
                max_value = max(self.append_focus_values)
                if abs(self.curr_focus_value - max_value) < 2:
                    # Stop movement
                    self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, False)])
                    self.can_move_to_next_pose = True
                    return

                # Clear the array for next time
                self.append_focus_values = []

            # Reset flipping state
            self.twist_is_flipping = False

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