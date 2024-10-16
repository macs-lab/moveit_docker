#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped,Pose,TransformStamped, TwistStamped, Quaternion
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
from std_msgs.msg import Float64, Header
from inspection_msgs.msg import FocusValue
import math
import csv
import os
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from datetime import datetime

IDLE = 0
POSES_AVAILABLE = 1
MOVING = 2
FEEDBACK = 3

class PoseStampedCreator(Node):

    def __init__(self):
        super().__init__('pose_stamped_creator')

        self.state = IDLE

        # Create a client to the service
        self.pose_client = self.create_client(MoveToPose, '/inspection/move_to_pose')
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Hello world.')

        # Subscribe to a topic publishing PoseStamped messages
        self.pose_queue = []
        self.pose_queue_sub = self.create_subscription(PoseStamped,'/pose_stamped', self.pose_callback, 10)
        self.i = 0

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
        self.declare_parameter('y_twist_speed', -0.3, descriptor=ParameterDescriptor(dynamic_typing=True))
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
        self.new_focus_value_sub = self.create_subscription(FocusValue, '/image_raw/compressed/focus_value', self.new_focus_value_callback, 10)
        self.ema_focus_value = 0
        self.curr_focus_value = 0
        self.append_focus_values = []
        self.curr_max_fv = None
        self.previous_ema = 0
        ###########################

        self.timer_callback_group = ReentrantCallbackGroup()
        self.tf_wt = TransformStamped()
        self.tf_timer = self.create_timer(0.1, self.on_timer, callback_group=self.timer_callback_group)    

        # Create client to run multiple times
        self.start_callback_group = ReentrantCallbackGroup()
        self.start_service = self.create_service(Trigger, '/inspection/count_callback', self.start_callback, callback_group=self.start_callback_group)
        self.get_logger().info('Start gathering data.')   

    def pose_callback(self, msg):
        self.i+=1
        self.pose_queue.append(msg)
        self.get_logger().info(f'Iter: {self.i}, Queue length: {len(self.pose_queue)}')

    # Move to the pose we received
    def on_timer(self):
        if self.state == IDLE:
            if len(self.pose_queue) > 0:
                self.state = POSES_AVAILABLE
                print('Poses available')
            else:
                self.state = IDLE
        elif self.state == POSES_AVAILABLE:
            print('Moving to pose')
            self.process_pose()
            self.get_logger().info('Moved to pose')
        elif self.state == MOVING:
            pass
        elif self.state == FEEDBACK:
            self.simple_feedback()
            self.get_logger().info('Feedback running')
    
    def start_callback(self, request, response):
        self.counter = 0
        self.curr_max_fv = 0
        self.focus_pose_dict = {}
        if self.state == IDLE:
            self.initial_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            
            print('entering idle')
            self.send_request_world(self.start_pose.pose)
            print('sent request')
            time.sleep(2) # Wait for the robot to move to the start pose. Necessary, otherwise sometimes the robot doesn't move to the start pose
            print('enter fb')
            self.state = FEEDBACK
            
            while self.counter < 5:
                if self.state == IDLE:
                    self.send_request_world(self.start_pose.pose)
                    time.sleep(2)  # Wait for the robot to move to the start pose
                    self.state = FEEDBACK
                time.sleep(1)
            
            self.send_request_world(self.max_focus_pose)
            time.sleep(2) # Wait for the robot to move to the max focus pose. Necessary, otherwise sometimes the robot doesn't move to the max focus pose
            self.get_logger().info(f'MAX FOCUS VALUE: {self.curr_max_fv}')
            return response
        
    # Move to specified pose but overshoot 5 cm backwards
    def process_pose(self):
        self.state = MOVING

        pose_tool0 = self.pose_queue.pop(0)
        pose_tool0.pose.position.y += 0.02 # Move the pose 2 cm backwards in the y direction
        from_frame_rel = pose_tool0.header.frame_id
        to_frame_rel = 'world'

        try:
            self.tf_wt = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            # self.get_logger().info(f'Got transform: {self.tf_wt}')
            self.send_request_tool0(pose_tool0)
            self.get_logger().info(f'Completed transform')
            time.sleep(1)
            return

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

    def send_request_tool0(self,pose_tool0):
        curr_time = rclpy.time.Time()
        tf_wt = self.tf_buffer.lookup_transform('world', 'tool0', curr_time)
        
        # Transform the pose from 'tool0' to 'world'
        pose_map = tf2_geometry_msgs.do_transform_pose(pose_tool0.pose, self.tf_wt)
        self.get_logger().info('Transformed pose from tool0 to world')

        # Send the transformed pose as the request
        target_pose_map = PoseStamped()
        target_pose_map.header.frame_id = 'world'
        target_pose_map.pose = pose_map
        self.req.target_pose = target_pose_map
        # print('Target pose in map coordinates: ',self.req)

        # publish destination pose to topic 'published_pose'
        self.pose_publisher.publish(target_pose_map)

        # Call the service
        req = MoveToPose.Request()
        req.target_pose = target_pose_map

        future = self.pose_client.call_async(req)
        future.add_done_callback(self.response_callback_tool0)
        return

    def send_request_world(self,pose_map):
        curr_time = rclpy.time.Time()
        tf_wt = self.tf_buffer.lookup_transform('world', 'tool0', curr_time)

        # Send the transformed pose as the request
        target_pose_map = PoseStamped()
        target_pose_map.header.frame_id = 'world'
        target_pose_map.pose = pose_map
        self.req.target_pose = target_pose_map

        # publish destination pose to topic 'published_pose'
        self.pose_publisher.publish(target_pose_map)

        # Call the service
        req = MoveToPose.Request()
        req.target_pose = target_pose_map

        future = self.pose_client.call_async(req)
        future.add_done_callback(self.response_callback_world)
        return
    
    def response_callback_tool0(self, future):
        try:
            response = future.result()
            if response.done:
                self.get_logger().info('Service completed successfully.')
                self.state = FEEDBACK
                return
            else:
                self.get_logger().error('Service failed.')
                self.state = IDLE
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))
    
    def response_callback_world(self, future):
        try:
            response = future.result()
            if response.done:
                self.get_logger().info('Final Service completed successfully.')
                self.state = IDLE # State change needs to occur after moving to pose with max focus
                return
            else:
                self.get_logger().error('Final Service failed.')
        except Exception as e:
            self.get_logger().error('Final Service call failed %r' % (e,))
    
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

    def new_focus_value_callback(self, msg):
        self.new_time = msg.header.stamp
        self.new_curr_focus_value = msg.data
        # self.get_logger().info('Focus value: "%s"' % self.new_curr_focus_value)

        try:
            try:
                # Attempt to get the transform at the exact requested time
                tf_wt = self.tf_buffer.lookup_transform('world', 'tool0', self.new_time)
            except tf2_ros.TransformException as ex:
                self.get_logger().info(f'Could not fine tune. Using latest known transform instead') # {ex}')
                # Fallback to the latest available transform within a 1-second duration
                tf_wt = self.tf_buffer.lookup_transform('world', 'tool0', rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
            
            pose_world = PoseStamped()
            pose_world.header.frame_id = 'world'
            pose_world.pose.position.x = tf_wt.transform.translation.x
            pose_world.pose.position.y = tf_wt.transform.translation.y
            pose_world.pose.position.z = tf_wt.transform.translation.z
            pose_world.pose.orientation = tf_wt.transform.rotation

            # Convert Time msg to float value
            timestamp = self.new_time.sec + self.new_time.nanosec / 1e9

            # Save the current focus value and its associated pose
            self.focus_pose_dict[self.new_curr_focus_value] = {'pose': pose_world, 'timestamp': timestamp, 'metric': msg.metric}

        except TransformException as ex:
            self.get_logger().info(
                f'Could not fine tune. Still works fine') # {ex}')
            return

    def simple_feedback(self):
        self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.twist_started = self.get_parameter('twist_started').value
        
        if self.focus_pose_dict:
            self.curr_max_fv = max(self.focus_pose_dict.keys())
            max_focus_pose = self.focus_pose_dict[self.curr_max_fv]['pose'].pose
            # Check if moving in the right direction, Update EMA
            K = 2 / (5 + 1)  # Number of data points to average over. Values before last 5 are still considered, just to a lesser degree
            self.ema_focus_value = (K * (self.curr_focus_value - self.ema_focus_value)) + self.ema_focus_value

            if (self.previous_ema - self.ema_focus_value) > 2:
                self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, False)])
                self.twist_started = self.get_parameter('twist_started').value
                self.save_focus_pose_dict_to_csv()
                self.send_request_world(max_focus_pose)
                time.sleep(0.5) # Wait for the robot to move to the max focus pose. Necessary, otherwise sometimes the robot doesn't move to the max focus pose
                # State change to IDLE placed in send_request_world, otherwise sometimes the robot doesn't move to the max focus pose
                self.get_logger().info(f'MAX FOCUS VALUE: {self.curr_max_fv}')

        print(self.curr_focus_value, self.curr_max_fv, self.ema_focus_value, self.previous_ema)
        self.previous_ema = self.ema_focus_value
        return
    
    def save_focus_pose_dict_to_csv(self):
        # Define the directory and file name
        directory = '/data/'
        # Generate the initial timestamp if it doesn't exist
        
        # Define the file name with the initial timestamp
        file_name = f'focus_pose_dict_{self.initial_timestamp}.csv'
        file_path = os.path.join(directory, file_name)
    
        
        # Ensure the directory exists
        os.makedirs(directory, exist_ok=True)

        # Check if the file exists and is not empty
        file_exists = os.path.isfile(file_path)
        
        with open(file_path, 'a', newline='') as csvfile:
            # Extract the keys for the header
            fieldnames = ['focus_value','raw_focus_value','x','y','z','quaternion','timestamp', 'metric']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            # Write the header only if the file is empty or doesn't exist
            if not file_exists or os.stat(file_path).st_size == 0:
                writer.writeheader()

            t0 = self.focus_pose_dict[list(self.focus_pose_dict.keys())[0]]['timestamp']
            for focus_value, data in self.focus_pose_dict.items():
                raw_focus_value = data['raw_data']
                pose = data['pose'].pose
                positionx = pose.position.x
                positiony = pose.position.y
                positionz = pose.position.z
                quaternion = f"{pose.orientation.x},{pose.orientation.y},{pose.orientation.z},{pose.orientation.w}"
                timestamp = data['timestamp']
                metric = data['metric']
                row = {
                    'focus_value': focus_value,
                    'raw_focus_value': raw_focus_value,
                    'x': positionx,
                    'y': positiony,
                    'z': positionz,
                    'quaternion': quaternion,
                    'timestamp' : timestamp - t0,
                    'metric': metric
                }
                writer.writerow(row)

def main(args=None):
    rclpy.init(args=args)
    pose_stamped_client = PoseStampedCreator()
    executor = MultiThreadedExecutor()
    executor.add_node(pose_stamped_client)
    try:
        # rclpy.spin(pose_stamped_client)
        executor.spin()
    except KeyboardInterrupt:
        # pass
        pose_stamped_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()