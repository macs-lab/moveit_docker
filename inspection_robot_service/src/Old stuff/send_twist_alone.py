#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from rclpy.clock import ROSClock
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Float64

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.client = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()
        self.future = self.client.call_async(self.req)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Default values in parameters are specified here
        # self.declare_parameter('twist_started', False, descriptor=ParameterDescriptor(dynamic_typing=True)) # dynamic_typing=True makes the parameter dynamically reconfigurable
        self.declare_parameter('z_twist_speed', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True)) # declare twist_speed as a parameter
        self.declare_parameter('z_twist_angular', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True)) # declare z_twist_angular as a parameter
        # self.twist_started = self.get_parameter('twist_started').value
        self.twist_started = False
        self.z_twist_speed = self.get_parameter('z_twist_speed').value
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

        # Start the timer after 'Enter' is pressed
        # input("Press Enter to start the timer...")
        # self.timer_started = True
        self.start_time = self.get_clock().now()

    # on_parameter_change is a new method that gets called whenever a parameter changes. It updates self.twist_started with the new value of the twist_started parameter
    def on_parameter_change(self, parameters):
        for parameter in parameters:
            if parameter.name == 'twist_started':
                self.twist_started = parameter.value
            elif parameter.name == 'z_twist_speed':
                self.z_twist_speed = parameter.value
            elif parameter.name == 'z_twist_angular':
                self.z_twist_angular = parameter.value
        return SetParametersResult(successful=True)
    
    def timer_callback(self):
        # print(self.twist_started) # print bool value of twist_started
        if not self.twist_started:
            return
        
        #elapsed_time = self.get_clock().now() - self.start_time
        #if elapsed_time.nanoseconds >= self.twist_length* 10**9:  # 2.5 seconds in nanoseconds
        #    self.timer.cancel()
        #else:
        msg = TwistStamped()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.header.frame_id = 'tool0'
        msg.twist.linear.z = self.z_twist_speed
        msg.twist.angular.z = self.z_twist_angular 
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.twist.linear.z)

    # Sobel stuff
    def display_image_from_msg(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("view", cv_image)
            cv2.waitKey(10)
        except CvBridgeError as e:
            self.get_logger().error("Could not convert from '%s' to 'bgr8'. Exception: %s" % (msg.encoding, e))
    def sobel_sharpness_value(self, msg):
        self.get_logger().info('Focus value: "%s"' % msg.data)
        if msg.data < 100:
            self.twist_started = True
        else:
            self.twist_started = False

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()