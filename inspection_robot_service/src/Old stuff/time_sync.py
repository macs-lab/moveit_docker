#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.clock import Clock

from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Temperature, FluidPressure


class TimeSyncNode(Node):

    def __init__(self):
        super().__init__('sync_node')
        qos = QoSProfile(depth=10)
        self.temp_pub = self.create_publisher(Temperature, 'temp', qos)
        self.fluid_pub = self.create_publisher(FluidPressure, 'fluid', qos)
        self.temp_sub = Subscriber(self, Temperature, "temp")
        self.fluid_sub = Subscriber(self, FluidPressure, "fluid")

        self.timer = self.create_timer(1, self.TimerCallback)
        self.second_timer = self.create_timer(1.05, self.SecondTimerCallback)

        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.temp_sub, self.fluid_sub],
                                                     queue_size, max_delay)
        self.time_sync.registerCallback(self.SyncCallback)
    
    def SyncCallback(self, temp, fluid):
        temp_sec = temp.header.stamp.sec
        fluid_sec = fluid.header.stamp.sec
        self.get_logger().info(f'Sync callback with {temp_sec} and {fluid_sec} as times')
        if (temp.header.stamp.sec > 2.0):
            new_fluid = FluidPressure()
            new_fluid.header.stamp = Clock().now().to_msg()
            new_fluid.header.frame_id = 'test'
            new_fluid.fluid_pressure = 2.5
            self.fluid_pub.publish(new_fluid)

    def TimerCallback(self):
        temp = Temperature()
        self.now = Clock().now().to_msg()

        temp.header.stamp = self.now
        temp.header.frame_id = 'test'
        temp.temperature = 1.0
        self.temp_pub.publish(temp)

    def SecondTimerCallback(self):
        fluid = FluidPressure()
        self.now = Clock().now().to_msg()

        fluid.header.stamp = self.now
        fluid.header.frame_id = "test"
        fluid.fluid_pressure = 2.0
        self.fluid_pub.publish(fluid)

def main(args=None):
    rclpy.init(args=args)

    time_sync = TimeSyncNode()

    rclpy.spin(time_sync)

    time_sync.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()