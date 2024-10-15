#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from rclpy.clock import ROSClock

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
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        elapsed_time = self.get_clock().now() - self.start_time
        if elapsed_time.nanoseconds >= 2.5e9:  # 2.5 seconds in nanoseconds
            self.timer.cancel()
        else:
            msg = TwistStamped()
            msg.header.stamp = ROSClock().now().to_msg()
            msg.header.frame_id = 'tool0'
            msg.twist.linear.z = 0.3
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.twist.linear.z)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()